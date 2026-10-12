import assert from 'node:assert/strict';import * as THREE from 'three';import {pointOnPlane} from '../dist/point-target.js';
const camera=new THREE.PerspectiveCamera(38,800/700,.001,100);camera.up.set(0,0,1);camera.position.set(.8,-1.2,.8);camera.lookAt(0,0,.2);camera.updateMatrixWorld();
const rect={left:30,top:60,width:800,height:700};
for(const p of [[.1,-.2,.15],[-.3,.25,.4],[.0,.0,-.1],[.45,-.15,.65]]){
 const v=new THREE.Vector3(...p),screen=v.clone().project(camera),hit=pointOnPlane(camera,rect,rect.left+(screen.x+1)*rect.width/2,rect.top+(1-screen.y)*rect.height/2,p[2]);
 assert.ok(hit&&hit.distanceTo(v)<1e-9,`Wrong target ${p}`);
}
assert.equal(pointOnPlane(camera,{...rect,width:0},0,0,.2),null);
assert.equal(pointOnPlane(camera,rect,400,400,NaN),null);
const far=new THREE.Vector3(2,0,.2).project(camera);assert.equal(pointOnPlane(camera,rect,rect.left+(far.x+1)*rect.width/2,rect.top+(1-far.y)*rect.height/2,.2),null);
console.log('PASS: screen-to-world target placement across XYZ heights, viewport offsets, out-of-plane bounds and invalid input.');
