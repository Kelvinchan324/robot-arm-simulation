import assert from 'node:assert/strict';
import fs from 'node:fs';
import loadMujoco from '@mujoco/mujoco';
import {RobotEngine} from '../dist/engine.js';
const mj=await loadMujoco(),e=new RobotEngine(mj);
const xml=fs.readFileSync(new URL('../dist/demo.urdf',import.meta.url),'utf8');e.load(xml);
const close=(a,b,t=1e-7)=>assert.ok(Math.abs(a-b)<t,`${a} != ${b}`);
e.pose().p.forEach((v,i)=>close(v,[.585,0,.135][i]));
e.q=[Math.PI/2,0,0,0,0];e.pose().p.forEach((v,i)=>close(v,[0,.585,.135][i]));
e.q=[0,-Math.PI/2,0,0,0];e.pose().p.forEach((v,i)=>close(v,[0,0,.720][i]));
e.q=[0,0,0,0,0];e.offset=[.05,0,0];close(e.pose().p[0],.635);e.offset=[0,0,0];
let solved=0;
for(let k=1;k<=20;k++){const q=e.joints.map((j,i)=>Math.sin(k*(i+1)*.37)*.8);e.q=q;const target=e.pose().p;e.q=q.map(x=>x+.15);const r=e.solve(target);assert.ok(r.converged,`Reachable target ${k} failed: ${r.error}`);assert.ok(r.error<.0005);e.q.forEach((v,i)=>assert.ok(v>=e.joints[i].lo&&v<=e.joints[i].hi));solved++;}
e.q=[0,0,0,0,0];const fail=e.solve([5,5,5]);assert.equal(fail.converged,false);assert.ok(Number.isFinite(fail.error)&&fail.error>1);
assert.throws(()=>e.solve([NaN,0,0]));
const n=e.model.njnt;assert.throws(()=>e.load('<robot>bad'));assert.equal(e.model.njnt,n);
// Compile an OBJ-backed URDF through MuJoCo's virtual filesystem.
const obj='v 0 0 0\nv .1 0 0\nv 0 .1 0\nv 0 0 .1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n';
const meshXML=xml.replace('<box size="0.13 0.13 0.06" />','<mesh filename="test.obj"/>');e.load(meshXML,[['test.obj',new TextEncoder().encode(obj)]]);assert.ok(e.model.nmesh===1);e.forward();assert.ok(e.pose().p.every(Number.isFinite));
console.log(`PASS: analytical zero/yaw/shoulder FK, tool offset, ${solved} reachable IK targets, unreachable target, limits, invalid input, failed-load retention and OBJ asset import. MuJoCo ${mj.mj_versionString()}.`);
e.data.delete();e.model.delete();
