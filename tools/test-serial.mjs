import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import loadMujoco from '@mujoco/mujoco';
import {packet,PacketParser,syncTargets,signedWord} from '../dist/serial-protocol.js';
import {SerialLink} from '../dist/serial-link.js';
import {validateProfile} from '../dist/hardware-profile.js';
import {LiveJog} from '../dist/live-jog.js';
import {HardwareSession} from '../dist/hardware-session.js';
import {RobotEngine} from '../dist/engine.js';
import {calibrateJointAngles} from '../dist/joint-angles.js';

class FakePort{
  constructor(){this.sent=[];this.missing=new Set();this.errors={};this.corrupt=false;this.raw=[2000,2001,2002,2003,2004];}
  async open(options){
    this.options=options;this.readable=new ReadableStream({start:c=>this.rx=c});
    this.writable=new WritableStream({write:bytes=>{
      this.sent.push([...bytes]);assert.equal(bytes.at(-1),(~[...bytes].slice(2,-1).reduce((a,b)=>a+b,0))&255);
      const [id,inst,address,length]=[bytes[2],bytes[4],bytes[5],bytes[6]];
      if(inst===0x83)return;
      const ids=inst===0x82?[...bytes].slice(7,-1):[id];
      for(const n of ids){
        if(this.missing.has(n))continue;
        const registers=new Uint8Array(80);registers[40]=1;registers[56]=this.raw[n-1]&255;registers[57]=this.raw[n-1]>>8;
        registers[58]=25;registers[59]=128;
        const reply=packet(n,this.errors[n]||0,[...registers.slice(address,address+length)]);
        if(this.corrupt)reply[reply.length-1]^=1;
        this.rx.enqueue(reply.slice(0,3));this.rx.enqueue(reply.slice(3));
      }
    }});
  }
  async setSignals(s){this.signals=s;}
  async close(){this.closed=true;}
}

test('packet format matches supplied sms_sts.SyncWritePosEx',()=>{
  const targets=[2010,2020,2030,2040,1950].map((raw,i)=>({id:i+1,raw,speed:100,acceleration:20}));
  const p=[...syncTargets(targets)];
  assert.deepEqual(p.slice(0,7),[255,255,254,44,131,41,7]);
  assert.deepEqual(p.slice(7,-1),targets.flatMap(t=>[t.id,20,t.raw&255,t.raw>>8,0,0,100,0]));
  assert.equal(p.reduce((s,v,i)=>s+(i<2?0:v),0)&255,255);
  assert.throws(()=>syncTargets([...targets.slice(0,4),{...targets[4],raw:4096}]));
});
test('parser handles fragmentation, noise, concatenation and checksum failure',()=>{
  const p=new PacketParser(),a=packet(1,0,[208,7]);
  assert.deepEqual(p.push([3,9,...a.slice(0,4)]),[]);
  assert.deepEqual(p.push([...a.slice(4),...packet(2,0,[1])]).map(p=>p.id),[1,2]);
  a[a.length-1]^=1;assert.throws(()=>p.push(a),/corrupt/);
  assert.equal(signedWord(25,128),-25);
});
test('Web Serial stream reads all IDs and closes its locks',async()=>{
  const port=new FakePort(),link=new SerialLink(port);await link.open();
  const states=await link.readAll([1,2,3,4,5],{health:true});
  assert.equal(port.options.baudRate,115200);assert.equal(port.signals.dataTerminalReady,false);
  assert.deepEqual(states.map(s=>s.raw),port.raw);assert.ok(states.every(s=>s.speed===-25&&s.mode===0&&s.torque===1));
  assert.ok(port.sent.every(p=>p[4]===2||p[4]===0x82));
  await link.close();assert.equal(port.readable.locked,false);assert.equal(port.writable.locked,false);assert.ok(port.closed);
});
test('missing early ID poisons link; a later response cannot authorize movement',async()=>{
  const port=new FakePort(),link=new SerialLink(port,{timeout:25});await link.open();
  await link.readAll([1,2,3,4,5]);port.missing.add(1);
  await assert.rejects(link.readAll([1,2,3,4,5]),/timeout/);
  port.missing.clear();const before=port.sent.length;
  await assert.rejects(link.readAll([1,2,3,4,5]),/timeout/);assert.equal(port.sent.length,before);
  await link.close();
});
test('motor status and corrupt packets fail the transaction',async()=>{
  for(const kind of ['error','checksum']){
    const port=new FakePort(),link=new SerialLink(port);await link.open();
    if(kind==='error')port.errors[2]=4;else port.corrupt=true;
    await assert.rejects(link.readAll([1,2,3,4,5]),kind==='error'?/Motor 2.*overheat/:/corrupt/);await link.close();
  }
});
test('USB removal aborts pending feedback without commanding any servo',async()=>{
  const port=new FakePort(),link=new SerialLink(port);await link.open();port.missing=new Set([1,2,3,4,5]);
  const pending=link.readAll([1,2,3,4,5]);port.rx.close();await assert.rejects(pending,/disconnected/);await link.close();
  assert.ok(port.sent.every(p=>p[4]===0x82));
});

const mj=await loadMujoco(),root=new URL('../dist/robot/',import.meta.url);
const model=JSON.parse(fs.readFileSync(new URL('model.json',root))),xml=fs.readFileSync(new URL('engg1101.xml',root),'utf8');
const assets=model.assets.map(n=>[n,new Uint8Array(fs.readFileSync(new URL(n,root)))]);
function fixture(){
  const engine=new RobotEngine(mj);engine.load(xml,assets,model);const angles=calibrateJointAngles(engine.joints);
  const data={version:1,servo_family:'sms_sts',calibration_confirmed:true,position_mode_confirmed:true,servos:{}};
  engine.joints.forEach((j,i)=>data.servos[j.name]={id:i+1,joint_degrees:[angles[i].lo,angles[i].hi],raw_positions:i===4?[3800,200]:[200,3800],raw_limits:[200,3800],safe_degrees:[angles[i].lo,angles[i].hi],max_deg_s:i===4?30:10,speed:100,acceleration:20});
  return {engine,angles,data,profile:validateProfile(data,angles)};
}
function destroy(f){f.engine.data.delete();f.engine.model.delete();}
test('profile rejects blanks, reversed bounds, duplicate IDs and unconfirmed calibration',()=>{
  const f=fixture();
  for(const mutate of [d=>d.calibration_confirmed=false,d=>d.servos.elbow.raw_positions=[null,3000],d=>d.servos.shoulder.id=1,d=>d.servos.wrist_pitch.safe_degrees=[-200,90],d=>d.servos.elbow.speed=0,d=>d.speed_scale=Infinity]){
    const d=structuredClone(f.data);mutate(d);assert.throws(()=>validateProfile(d,f.angles));
  }
  for(const m of f.profile.maps){const d=m.safe_degrees[0]+(m.safe_degrees[1]-m.safe_degrees[0])*.4;assert.ok(Math.abs(m.fromRaw(m.toRaw(d))-d)<.2);}
  destroy(f);
});
test('live keyboard levels gradually, respects rates and pitch, and stops gripper ramp',()=>{
  const f=fixture(),c=new LiveJog(f.engine,f.angles,f.profile);
  const q=f.engine.q;c.key('w',true);c.tick();assert.deepEqual(f.engine.q,q);assert.ok(c.enable());
  for(let t=0;t<300;t++){
    const before=f.engine.q;c.tick();f.engine.q.forEach((v,i)=>assert.ok(Math.abs(f.angles[i].gain*(v-before[i]))<=f.profile.maps[i].max_deg_s*.08+1e-8));
    if(c.levelTarget===null)break;
  }
  assert.equal(c.levelTarget,null);assert.ok(Math.abs(c.jog.pitch())<.0002);
  const height=f.engine.pose().p[2];c.key('w',true);c.tick();c.key('w',false);
  assert.ok(f.engine.pose().p[2]>height);assert.ok(Math.abs(c.jog.pitch())<.0002);
  c.key('j',true);c.key('j',false);for(let i=0;i<100;i++)c.tick();assert.ok(Math.abs(f.angles[4].toDisplay(f.engine.q[4])-90)<1e-6);
  c.key('l',true);c.tick();c.stop();const stopped=f.engine.q;for(let i=0;i<10;i++)c.tick();assert.deepEqual(f.engine.q,stopped);destroy(f);
});
class FeedbackBus{
  constructor(f){this.raw=f.engine.q.map((q,i)=>f.profile.maps[i].toRaw(f.angles[i].toDisplay(q)));this.writes=[];this.mode=0;this.torque=1;}
  async readAll(){return this.raw.map((raw,i)=>({id:i+1,raw,mode:this.mode,torque:this.torque}));}
  async writeTargets(t){this.writes.push(t);this.raw=t.map(m=>m.raw);}
  async close(){this.closed=true;}
}
test('session starts read only and emits movement only after enable; measured pose is separate',async()=>{
  const f=fixture(),link=new FeedbackBus(f),feedback=[];let time=1000;
  const s=new HardwareSession({...f,link,clock:()=>time,onFeedback:v=>feedback.push(v)});
  await s.cycle();assert.equal(link.writes.length,0);s.enable();time+=80;await s.cycle();assert.equal(link.writes.length,1);
  assert.notDeepEqual(feedback.at(-1).degrees,f.engine.q.map((q,i)=>f.angles[i].toDisplay(q)));
  s.pause();time+=80;await s.cycle();assert.equal(link.writes.length,2);
  time+=80;await s.cycle();assert.equal(link.writes.length,2);await s.disconnect();destroy(f);
});
test('pause during an outstanding read prevents the old keyboard step',async()=>{
  const f=fixture(),link=new FeedbackBus(f),s=new HardwareSession({...f,link});
  await s.cycle();s.enable();await s.cycle();const before=link.writes.length;
  let release;const read=link.readAll.bind(link);link.readAll=()=>new Promise(resolve=>release=async()=>resolve(await read()));
  const cycle=s.cycle();s.pause();await release();await cycle;
  assert.equal(s.controller.enabled,false);assert.equal(link.writes.length,before+1); // Fresh measured hold only.
  link.readAll=read;await s.disconnect();destroy(f);
});
test('delayed loop pauses; tracking and health faults prevent further setpoints',async()=>{
  for(const fault of ['delay','tracking','mode','torque']){
    const f=fixture(),link=new FeedbackBus(f);let now=1000,message='';
    const s=new HardwareSession({...f,link,clock:()=>now,onFault:e=>message=e.message});
    await s.cycle();s.enable();await s.cycle();const n=link.writes.length;
    if(fault==='delay')now+=600;
    if(fault==='tracking')link.raw[0]+=200;
    if(fault==='mode')link.mode=1;
    if(fault==='torque')link.torque=0;
    await s.cycle();assert.equal(s.controller.enabled,false);
    if(fault==='delay')assert.equal(link.writes.length,n+1);else{assert.ok(message);assert.equal(link.writes.length,n);}
    await s.disconnect();destroy(f);
  }
});
test('uncalibrated connection reads raw encoders but cannot enable motion',async()=>{
  const f=fixture(),link=new FeedbackBus(f),s=new HardwareSession({...f,profile:null,link});
  await s.cycle();assert.throws(()=>s.enable(),/calibration/);assert.equal(link.writes.length,0);await s.disconnect();destroy(f);
});
test('lost control focus cancels jogging even without a keyup event',async()=>{
  const f=fixture(),link=new FeedbackBus(f);let focused=true;
  const s=new HardwareSession({...f,link,canMove:()=>focused});
  await s.cycle();s.enable();await s.cycle();s.key('w',true);
  focused=false;const n=link.writes.length;await s.cycle();
  assert.equal(s.controller.enabled,false);assert.equal(s.controller.held.size,0);assert.equal(link.writes.length,n+1);
  await s.cycle();assert.equal(link.writes.length,n+1);await s.disconnect();destroy(f);
});
