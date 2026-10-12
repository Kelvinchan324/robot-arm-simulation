import fs from 'node:fs';import assert from 'node:assert/strict';import loadMujoco from '@mujoco/mujoco';import {RobotEngine} from '../dist/engine.js';import {calibrateJointAngles} from '../dist/joint-angles.js';
const root=new URL('../dist/robot/',import.meta.url),m=JSON.parse(fs.readFileSync(new URL('model.json',root))),e=new RobotEngine(await loadMujoco());e.load(fs.readFileSync(new URL('engg1101.xml',root),'utf8'),m.assets.map(n=>[n,new Uint8Array(fs.readFileSync(new URL(n,root)))]),m);
const initial=e.pose(),a=calibrateJointAngles(e.joints),d=180/Math.PI;
const close=(x,y,t=1e-8)=>assert.ok(Math.abs(x-y)<t,`${x} != ${y}`);
close(a[0].toDisplay(18.8/d),0);close(a[0].lo,-180);close(a[0].hi,180);close(a[1].toDisplay(e.joints[1].hi),90);close(a[2].toDisplay(e.joints[2].lo),-90);close(a[3].toDisplay(e.joints[3].hi),90);close(a[4].toDisplay(e.joints[4].lo),90);close(a[4].toDisplay(e.joints[4].hi),-90);
assert.deepEqual(e.pose(),initial);
for(let i=0;i<a.length;i++)for(const t of [0,.25,.5,.75,1]){const q=e.joints[i].lo+t*(e.joints[i].hi-e.joints[i].lo);close(a[i].fromDisplay(a[i].toDisplay(q)),q);}
const jawDistance=()=>{const ids=['jaw_a','jaw_b'].map(n=>e.bodies.find(b=>b.name===n).id);return Math.hypot(...[0,1,2].map(i=>e.data.xpos[ids[0]*3+i]-e.data.xpos[ids[1]*3+i]));};
e.q=[0,0,0,0,a[4].fromDisplay(-90)];const closed=jawDistance();e.q=[0,0,0,0,a[4].fromDisplay(90)];const open=jawDistance();assert.ok(open>closed+.06,`Jaw separation: open ${open}, closed ${closed}`);
e.q=[a[0].fromDisplay(0),a[1].fromDisplay(10),a[2].fromDisplay(0),a[3].fromDisplay(0),a[4].fromDisplay(45)];const goal=e.pose().p;e.q=e.q.map((x,i)=>i===4?x:x+.05);assert.ok(e.solve(goal).converged);close(a[4].toDisplay(e.q[4]),45);
console.log('PASS: all requested reference points, reversible input/display mapping, unchanged reference geometry, +90 wider than -90 by over 60 mm, and IK preserves gripper setting.');
console.log(e.joints.map((j,i)=>({joint:j.name,range:[a[i].lo,a[i].hi]})));
