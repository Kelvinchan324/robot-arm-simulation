// Generate independent Python parity fixtures using the actual browser engine.
// Run after npm ci: node tools/generate-pi-reference.mjs
import fs from 'node:fs';
import crypto from 'node:crypto';
import {RobotEngine} from '../dist/engine.js';
import {CartesianJog} from '../dist/jog.js';
import {calibrateJointAngles} from '../dist/joint-angles.js';
const {default:loadMujoco}=await import(process.env.MUJOCO_MODULE||'@mujoco/mujoco');
const root=new URL('../dist/robot/',import.meta.url),config=JSON.parse(fs.readFileSync(new URL('model.json',root)));
const engine=new RobotEngine(await loadMujoco());
engine.load(fs.readFileSync(new URL('engg1101.xml',root),'utf8'),config.assets.map(n=>[n,new Uint8Array(fs.readFileSync(new URL(n,root)))]),config);
const angles=calibrateJointAngles(engine.joints),jog=new CartesianJog(engine);
const snapshot=()=>({q:engine.q,p:engine.pose().p,r:engine.pose().r,display:engine.q.map((q,i)=>angles[i].toDisplay(q)),pitch:jog.pitch(),bodies:Object.fromEntries(engine.bodies.map(b=>[b.name,{p:Array.from(engine.data.xpos.slice(b.id*3,b.id*3+3)),r:Array.from(engine.data.xmat.slice(b.id*9,b.id*9+9))}]))});
const fk=[];
for(let k=0;k<16;k++){engine.q=k===0?[0,0,0,0,0]:engine.joints.map((j,i)=>j.lo+(j.hi-j.lo)*(.5+.45*Math.sin(k*(i+1)*1.71)));fk.push(snapshot());}
engine.q=[0,0,0,0,0];const initial=engine.q,levelled=jog.levelWrist(),level=snapshot(),moves=[];
for(const motion of [{height:.003},{reach:.003},{yaw:.3},{height:-.003},{reach:-.003},{pitch:.03},{height:.003},{pitch:-.03},{height:2},{yaw:20}]){const ok=jog.step(motion);moves.push({motion,ok,...snapshot()});}
const targets=[];
for(const target of [[.05,-.16,.25],[.08,-.13,.3]]){engine.q=[0,0,0,.8,.1];const start=engine.q,result=engine.solve(target);targets.push({start,target,result,...snapshot()});}
const paths=['dist/engine.js','dist/jog.js','dist/joint-angles.js','dist/robot/engg1101.xml','dist/robot/model.json'];
const hashes=Object.fromEntries(paths.map(p=>[p,crypto.createHash('sha256').update(fs.readFileSync(new URL('../'+p,import.meta.url),'utf8').replaceAll('\r\n','\n')).digest('hex')]));
fs.writeFileSync(new URL('../raspberry_pi/tests/browser_reference.json',import.meta.url),JSON.stringify({hashes,fk,initial,levelled,level,moves,targets},null,2)+'\n');
console.log('Saved MuJoCo/browser reference poses, coupled link transforms, jog and IK results.');
