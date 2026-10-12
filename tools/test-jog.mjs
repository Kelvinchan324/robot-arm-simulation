import fs from 'node:fs';import assert from 'node:assert/strict';import loadMujoco from '@mujoco/mujoco';import {RobotEngine} from '../dist/engine.js';import {CartesianJog} from '../dist/jog.js';
const root=new URL('../dist/robot/',import.meta.url),m=JSON.parse(fs.readFileSync(new URL('model.json',root))),e=new RobotEngine(await loadMujoco());
e.load(fs.readFileSync(new URL('engg1101.xml',root),'utf8'),m.assets.map(n=>[n,new Uint8Array(fs.readFileSync(new URL(n,root)))]),m);const jog=new CartesianJog(e);
const close=(a,b,t=3e-5)=>assert.ok(Math.abs(a-b)<t,`${a} != ${b}`);
assert.ok(jog.levelWrist(),'level wrist from reference pose');close(jog.pitch(),0,.0002);close(e.q[0],0);close(e.q[1],0);close(e.q[2],0);close(e.q[4],0);
for(const yaw of [0,-.35,.7]){
  assert.ok(jog.step({yaw}));const f=jog.forward();
  for(const move of [{height:.003},{height:-.003},{reach:.003},{reach:-.003},{pitch:.03},{height:.003},{reach:.003},{pitch:-.03}]){
    const before=e.pose().p,angle=jog.pitch(),q=e.q;
    assert.ok(jog.step(move),JSON.stringify(move));const after=e.pose().p;
    close(after[0],before[0]+(move.reach||0)*f[0]);close(after[1],before[1]+(move.reach||0)*f[1]);close(after[2],before[2]+(move.height||0));close(jog.pitch(),angle+(move.pitch||0),.0003);close(e.q[0],q[0]);close(e.q[4],q[4]);
  }
}
const q=e.q;assert.equal(jog.step({height:2}),false);e.q.forEach((v,i)=>close(v,q[i],1e-12));assert.equal(jog.step({yaw:20}),false);e.q.forEach((v,i)=>close(v,q[i],1e-12));
console.log('PASS: wrist levelling, vertical/radial translation, pitch at fixed centre, yaw-relative forward direction, pitch hold, jaw preservation, rejected unreachable moves and yaw limits.');
