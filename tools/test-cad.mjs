import fs from 'node:fs';import assert from 'node:assert/strict';import loadMujoco from '@mujoco/mujoco';import {RobotEngine} from '../dist/engine.js';
const root=new URL('../dist/robot/',import.meta.url),m=JSON.parse(fs.readFileSync(new URL('model.json',root))),mj=await loadMujoco(),e=new RobotEngine(mj);
const assets=m.assets.map(n=>[n,new Uint8Array(fs.readFileSync(new URL(n,root)))]);e.load(fs.readFileSync(new URL('engg1101.xml',root),'utf8'),assets,m);
assert.equal(e.joints.length,5);assert.equal(e.model.njnt,8);assert.equal(e.model.nmesh,9);assert.equal(e.model.neq,3);
assert.ok(Math.hypot(...e.pose().p.map((x,i)=>x-m.tool_position[i]))<1e-8);
const audit=JSON.parse(fs.readFileSync(new URL('../cad-export/audit-expected.json',import.meta.url)));let maxP=0,maxR=0;
for(const t of audit){e.q=e.joints.map(j=>t.q[j.name]);for(const [name,expected]of Object.entries(t.bodies)){const id=e.bodies.find(b=>b.name===name).id;const err=Math.hypot(...expected.p.map((v,i)=>v-e.data.xpos[id*3+i]));const rot=Math.max(...expected.r.map((v,i)=>Math.abs(v-e.data.xmat[id*9+i])));maxP=Math.max(maxP,err);maxR=Math.max(maxR,rot);}}
assert.ok(maxP<1e-5,`CAD translation error ${maxP}`);assert.ok(maxR<1e-4,`CAD rotation error ${maxR}`);
for(const opening of [-1,.5,1.5]){e.q=[.2,-.3,.4,.8,opening];for(const c of e.couplings)assert.ok(Math.abs(e.data.qpos[c.slave.addr]-e.data.qpos[c.master.addr]*c.multiplier-c.offset)<1e-10);}
for(let k=0;k<20;k++){const q=e.joints.map((j,i)=>i===4?.2:j.lo+(j.hi-j.lo)*(.35+.15*Math.sin((k+1)*(i+1))));e.q=q;const p=e.pose().p;e.q=q.map((v,i)=>i===4?v:v+.12);const result=e.solve(p);assert.ok(result.converged,`IK target ${k}: ${result.error}`);assert.equal(e.q[4],.2);}
e.q=[0,0,0,0,0];const distant=[5,5,5],before=Math.hypot(...e.pose().p.map((x,i)=>x-distant[i]));const best=e.solve(distant);assert.equal(best.converged,false);assert.ok(best.error<before);assert.ok(Math.abs(best.error-Math.hypot(...e.pose().p.map((x,i)=>x-distant[i])))<1e-10);
// The distributed URDF also compiles with all nine mesh assets.
e.load(fs.readFileSync(new URL('engg1101.urdf',root),'utf8'),assets,m);assert.equal(e.joints.length,5);assert.ok(Math.hypot(...e.pose().p.map((x,i)=>x-m.tool_position[i]))<1e-8);
console.log(`PASS: 9 CAD meshes, 8 joints, 3 couplings, all 5 Fusion motion audits (max position error ${(maxP*1000).toFixed(6)} mm; rotation ${maxR}), 20 reachable IK targets, gripper preserved, unreachable target, URDF compilation.`);
