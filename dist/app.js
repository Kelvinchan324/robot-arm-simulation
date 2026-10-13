import loadMujoco from './vendor/mujoco.js';
import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';
import {RobotEngine} from './engine.js';
import {CartesianJog} from './jog.js';
import {PointTarget} from './point-target.js';
import {calibrateJointAngles} from './joint-angles.js';
const $=id=>document.getElementById(id);
let jointAngles=[],jog,keyboardEnabled=false,pointPicker,solveRevision=0;
let engine,geomMeshes=[],jointMarkers=[],trace=[],robotConfig=null;
const scene=new THREE.Scene();scene.background=new THREE.Color('#08121d');
const camera=new THREE.PerspectiveCamera(38,1,.001,100);camera.up.set(0,0,1);
let renderer;
try {renderer=new THREE.WebGLRenderer({antialias:true});}catch(e){$('loading').textContent='WebGL is unavailable. Enable hardware acceleration or use a browser with WebGL support.';throw e;}
renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;$('canvas').appendChild(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement);orbit.enableDamping=true;
scene.add(new THREE.HemisphereLight(0xd8f1ff,0x13263a,2.5));const sun=new THREE.DirectionalLight(0xffffff,3);sun.position.set(1,-2,3);scene.add(sun);
const grid=new THREE.GridHelper(2,40,0x285169,0x142d40);grid.rotation.x=Math.PI/2;grid.position.z=-.002;scene.add(grid);
const axes=new THREE.AxesHelper(.12);scene.add(axes);const toolAxes=new THREE.AxesHelper(.075);scene.add(toolAxes);
const target=new THREE.Mesh(new THREE.SphereGeometry(.008,20,12),new THREE.MeshBasicMaterial({color:0xe1b976,depthTest:false}));scene.add(target);target.visible=true;target.renderOrder=5;
const targetRing=new THREE.Mesh(new THREE.TorusGeometry(.018,.001,8,32),new THREE.MeshBasicMaterial({color:0xe1b976,depthTest:false}));target.add(targetRing);targetRing.renderOrder=5;
const targetStem=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineDashedMaterial({color:0xe1b976,dashSize:.008,gapSize:.004,transparent:true,opacity:.5}));scene.add(targetStem);
const errorLine=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineDashedMaterial({color:0xe1b976,dashSize:.007,gapSize:.004,depthTest:false}));scene.add(errorLine);errorLine.renderOrder=4;
const traceLine=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0x5de2ed,transparent:true,opacity:.6}));scene.add(traceLine);
const matrix4=new THREE.Matrix4();
function frame(){requestAnimationFrame(frame);orbit.update();renderer.render(scene,camera);}frame();
new ResizeObserver(()=>{const {width,height}=$('canvas').getBoundingClientRect();renderer.setSize(width,height);camera.aspect=width/height;camera.updateProjectionMatrix();}).observe($('canvas'));
function home(){const box=new THREE.Box3();geomMeshes.forEach(m=>box.expandByObject(m));const center=box.isEmpty()?new THREE.Vector3(0,0,.2):box.getCenter(new THREE.Vector3());const size=box.isEmpty()?.6:Math.max(box.getSize(new THREE.Vector3()).length(),.3);orbit.target.copy(center);camera.position.copy(center).add(new THREE.Vector3(size*1.25,-size*1.8,size*1.0));camera.near=Math.max(.0001,size/10000);camera.far=Math.max(100,size*100);camera.updateProjectionMatrix();orbit.update();}
function makeGeometry(i){const m=engine.model,s=m.geom_size.slice(3*i,3*i+3),t=m.geom_type[i];let geo;
  if(t===2)geo=new THREE.SphereGeometry(s[0],24,16);
  else if(t===6)geo=new THREE.BoxGeometry(s[0]*2,s[1]*2,s[2]*2);
  else if(t===5){geo=new THREE.CylinderGeometry(s[0],s[0],s[1]*2,24);geo.rotateX(Math.PI/2);}
  else if(t===3){geo=new THREE.CapsuleGeometry(s[0],s[1]*2,8,16);geo.rotateX(Math.PI/2);}
  else if(t===4){geo=new THREE.SphereGeometry(1,24,16);geo.scale(...s);}
  else if(t===0)geo=new THREE.PlaneGeometry(s[0]?s[0]*2:2,s[1]?s[1]*2:2);
  else if(t===7){const id=m.geom_dataid[i],va=m.mesh_vertadr[id],vn=m.mesh_vertnum[id],fa=m.mesh_faceadr[id],fn=m.mesh_facenum[id];geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.Float32BufferAttribute(Array.from(m.mesh_vert.slice(va*3,(va+vn)*3)),3));geo.setIndex(Array.from(m.mesh_face.slice(fa*3,(fa+fn)*3)));geo.computeVertexNormals();}
  else throw Error(`Geometry type ${t} is not supported by this viewer.`);return geo;
}
function rebuild(){for(const mesh of [...geomMeshes,...jointMarkers]){scene.remove(mesh);mesh.geometry.dispose();mesh.material.dispose();}geomMeshes=[];jointMarkers=[];const m=engine.model;
  for(let i=0;i<m.ngeom;i++){const c=m.geom_rgba.slice(i*4,i*4+4);const material=new THREE.MeshStandardMaterial({color:new THREE.Color().setRGB(c[0],c[1],c[2],THREE.LinearSRGBColorSpace),roughness:.55,metalness:.2,opacity:c[3],transparent:c[3]<1,side:THREE.DoubleSide});const mesh=new THREE.Mesh(makeGeometry(i),material);mesh.userData.geom=i;scene.add(mesh);geomMeshes.push(mesh);}
  for(const j of engine.joints){const mesh=new THREE.Mesh(new THREE.SphereGeometry(.005,20,16),new THREE.MeshStandardMaterial({color:0x5de2ed,metalness:.4,roughness:.35}));scene.add(mesh);jointMarkers.push(mesh);}
  $('sliders').replaceChildren();engine.joints.forEach((j,i)=>{const angle=jointAngles[i],unit=j.slide?'m':'°';const div=document.createElement('div');div.className='joint';const top=document.createElement('div');top.className='joint-top';const label=document.createElement('label');label.htmlFor=`q${i}`;label.textContent=j.name==='gripper_drive'?'Gripper':j.name.replaceAll('_',' ');const number=document.createElement('input');number.id=`n${i}`;number.type='number';number.min=angle.lo.toFixed(6);number.max=angle.hi.toFixed(6);number.step=j.slide?.001:.1;number.setAttribute('aria-label',`${j.name} ${unit}`);top.append(label,number);const slider=document.createElement('input');slider.id=`q${i}`;slider.type='range';slider.min=number.min;slider.max=number.max;slider.step='any';const limits=document.createElement('div');limits.className='joint-limits';limits.innerHTML=`<span>${angle.lo.toFixed(j.slide?2:0)}${unit}${j.name==='gripper_drive'?' closed':''}</span><span>${angle.hi.toFixed(j.slide?2:0)}${unit}${j.name==='gripper_drive'?' open':''}</span>`;const change=e=>{cancelSolve();pauseKeyboard();const value=Number(e.target.value);if(!Number.isFinite(value)||e.target.value==='')return;const q=engine.q;q[i]=angle.fromDisplay(value);engine.q=q;update();$('solver-result').textContent='Pose changed · solve to match target';};slider.addEventListener('input',change);number.addEventListener('change',change);div.append(top,slider,limits);$('sliders').append(div);});
  $('joint-count').textContent=`${engine.joints.length} CONTROLS`;trace=[];traceLine.geometry.setFromPoints([]);update();home();
}
function update(){if(!engine?.model)return;updateTargetLines();if(jog)$('pitch-value').textContent=`Tool pitch ${(jog.pitch()*180/Math.PI).toFixed(1)}°`;const d=engine.data;geomMeshes.forEach((mesh,i)=>{mesh.position.fromArray(d.geom_xpos,i*3);const r=d.geom_xmat.slice(i*9,i*9+9);matrix4.set(r[0],r[1],r[2],0,r[3],r[4],r[5],0,r[6],r[7],r[8],0,0,0,0,1);mesh.quaternion.setFromRotationMatrix(matrix4);});jointMarkers.forEach((mesh,i)=>{mesh.position.fromArray(d.xanchor,engine.joints[i].id*3);mesh.visible=$('axes').checked;});const {p,r}=engine.pose();toolAxes.position.set(...p);matrix4.set(r[0],r[1],r[2],0,r[3],r[4],r[5],0,r[6],r[7],r[8],0,0,0,0,1);toolAxes.quaternion.setFromRotationMatrix(matrix4);['px','py','pz'].forEach((id,i)=>$(id).textContent=p[i].toFixed(3));const values=[r[0],r[1],r[2],p[0],r[3],r[4],r[5],p[1],r[6],r[7],r[8],p[2],0,0,0,1];$('matrix').innerHTML=values.map(v=>`<span>${Math.abs(v)<.00005?'0.000':v.toFixed(3)}</span>`).join('');engine.q.forEach((q,i)=>{const j=engine.joints[i],v=jointAngles[i].toDisplay(q);$(`q${i}`).value=v;$(`n${i}`).value=(Math.abs(v)<.000001?0:v).toFixed(j.slide?3:1);});if($('trail').checked){if(!trace.length||trace.at(-1).distanceTo(new THREE.Vector3(...p))>.001){trace.push(new THREE.Vector3(...p));if(trace.length>1200)trace.shift();traceLine.geometry.dispose();traceLine.geometry=new THREE.BufferGeometry().setFromPoints(trace);}}}
function example(){cancelSolve();pauseKeyboard();engine.q=robotConfig.example_pose;update();}
async function cad(){
  robotConfig=await (await fetch('./robot/model.json')).json();
  const xml=await (await fetch('./robot/engg1101.xml')).text();
  const assets=await Promise.all(robotConfig.assets.map(async name=>[name,new Uint8Array(await (await fetch('./robot/'+name)).arrayBuffer())]));
  engine.load(xml,assets,robotConfig);jointAngles=calibrateJointAngles(engine.joints);jog=new CartesianJog(engine);
  rebuild();
  engine.pose().p.forEach((v,i)=>$(['tx','ty','tz'][i]).value=v.toFixed(5));targetChange();
}
function cancelSolve(){solveRevision++;if(engine?.model)$('solve').disabled=false;}
function updateTargetLines(){if(!engine?.model)return;const p=new THREE.Vector3(...engine.pose().p);errorLine.geometry.setFromPoints([p,target.position]);errorLine.computeLineDistances();errorLine.visible=p.distanceTo(target.position)>.0005;targetStem.geometry.setFromPoints([new THREE.Vector3(target.position.x,target.position.y,0),target.position]);targetStem.computeLineDistances();}
function targetChange(){
  cancelSolve();const p=['tx','ty','tz'].map(id=>$(id).value===''?NaN:Number($(id).value));
  if(p.every(Number.isFinite)){target.position.set(...p);pointPicker?.setHeight(p[2]);$('target-height').min=Math.min(-.2,p[2]);$('target-height').max=Math.max(.8,p[2]);$('target-height').value=p[2];$('height-value').textContent=p[2].toFixed(3);updateTargetLines();}
}
function setTarget(p){p.forEach((v,i)=>$(['tx','ty','tz'][i]).value=v.toFixed(5));targetChange();}
function setPickMode(enabled){pointPicker?.enable(enabled);$('pick-target').setAttribute('aria-pressed',String(enabled));$('pick-target').textContent=enabled?'Done picking':'Pick target in 3D';$('pick-controls').hidden=!enabled;if(enabled){pauseKeyboard();targetChange();}else cancelSolve();}
async function solveTarget(){
  if(!engine?.model)return;pauseKeyboard();const targetP=['tx','ty','tz'].map(id=>$(id).value===''?NaN:Number($(id).value)),revision=++solveRevision;
  $('solve').disabled=true;$('solver-result').textContent='Solving…';await new Promise(r=>setTimeout(r,20));if(revision!==solveRevision)return;
  try{const result=engine.solve(targetP);update();$('solver-result').classList.toggle('fail',!result.converged);$('solver-result').textContent=`${result.converged?'Reached':'Best found'} · ${(result.error*1000).toFixed(2)} mm error`;}
  catch(e){$('solver-result').textContent=e.message;}finally{if(revision===solveRevision)$('solve').disabled=false;}
}
pointPicker=new PointTarget({scene,camera,canvas:renderer.domElement,orbit,onChange:p=>{setTarget(p);$('solver-result').textContent='Release to solve';},onCommit:solveTarget,onMiss:()=>{$('solver-result').textContent='Point at the highlighted plane';}});
$('pick-target').onclick=()=>setPickMode(!pointPicker.enabled);
$('target-height').oninput=()=>{$('tz').value=$('target-height').value;targetChange();$('solver-result').textContent='Release to solve';};$('target-height').onchange=solveTarget;
['tx','ty','tz'].forEach(id=>$(id).oninput=targetChange);
$('solve').onclick=solveTarget;
$('use-current').onclick=()=>{engine.pose().p.forEach((v,i)=>$(['tx','ty','tz'][i]).value=v.toFixed(5));targetChange();$('solver-result').textContent='Target = current position';};
$('home-view').onclick=home;$('axes').onchange=()=>{axes.visible=toolAxes.visible=$('axes').checked;update();};$('trail').onchange=()=>{trace=[];traceLine.geometry.setFromPoints([]);update();};$('reset').onclick=()=>{cancelSolve();pauseKeyboard();engine.q=engine.joints.map(()=>0);update();};$('example').onclick=example;
const held=new Set(),moveKeys=new Set(['w','s','a','d','arrowup','arrowdown','arrowleft','arrowright','j','l']);
function setGripper(open){
  const i=engine.joints.findIndex(j=>j.name==='gripper_drive');if(i<0)return;
  cancelSolve();const q=engine.q;q[i]=jointAngles[i].fromDisplay(open?90:-90);engine.q=q;update();
  keyboardStatus(open?'On · gripper open':'On · gripper closed');
}
const editing=()=>document.activeElement?.matches('input,textarea,select,[contenteditable="true"]');
function clearKeys(){held.clear();document.querySelectorAll('kbd').forEach(k=>k.classList.remove('pressed'));}
function keyboardStatus(text){if($('keyboard-status').textContent!==text)$('keyboard-status').textContent=text;}
function pauseKeyboard(){clearKeys();keyboardEnabled=false;$('keyboard-toggle').textContent='Enable + level';$('keyboard-toggle').setAttribute('aria-pressed','false');keyboardStatus('Off');}
function syncJog(){update();engine.pose().p.forEach((v,i)=>$(['tx','ty','tz'][i]).value=v.toFixed(5));targetChange();$('solver-result').textContent='Target = current position';}
function levelWrist(){clearKeys();if(!jog)return false;if(!jog.levelWrist()){keyboardStatus('Cannot level at this wrist limit');return false;}syncJog();return true;}
$('keyboard-toggle').onclick=()=>{if(keyboardEnabled){pauseKeyboard();return;}setPickMode(false);if(!levelWrist())return;keyboardEnabled=true;$('keyboard-toggle').textContent='Disable keys';$('keyboard-toggle').setAttribute('aria-pressed','true');keyboardStatus('On · hold a key');$('canvas').focus();};
$('level-gripper').onclick=()=>{if(levelWrist())keyboardStatus(keyboardEnabled?'On · pitch 0°':'Level · enable keys to move');};
function keyboardStep(){
  if(!keyboardEnabled||!jog||!held.size||editing()||document.hidden)return;
  const v=k=>held.has(k)?1:0;
  const motion={height:(v('w')-v('s'))*.0025,reach:(v('arrowup')-v('arrowdown'))*.0025,yaw:(v('arrowright')-v('arrowleft'))*Math.PI/120,pitch:(v('a')-v('d'))*Math.PI/120};
  if(!Object.values(motion).some(Boolean))return;
  const ok=jog.step(motion);if(ok){syncJog();keyboardStatus('On · pitch held');}else keyboardStatus('Limit reached · try another direction');
}
window.addEventListener('keydown',event=>{const key=event.key.toLowerCase();if(key==='escape'){pauseKeyboard();setPickMode(false);return;}if(event.ctrlKey||event.altKey||event.metaKey){clearKeys();return;}if(!keyboardEnabled||!moveKeys.has(key)||editing()||document.hidden)return;event.preventDefault();if(event.repeat||held.has(key))return;held.add(key);document.querySelector(`kbd[data-key="${key}"]`)?.classList.add('pressed');if(key==='j'||key==='l')setGripper(key==='j');else keyboardStep();});
window.addEventListener('keyup',event=>{const key=event.key.toLowerCase();if(!moveKeys.has(key))return;held.delete(key);document.querySelector(`kbd[data-key="${key}"]`)?.classList.remove('pressed');});
window.addEventListener('blur',clearKeys);document.addEventListener('visibilitychange',clearKeys);document.addEventListener('focusin',()=>{if(editing())clearKeys();});setInterval(keyboardStep,80);
document.querySelectorAll('button:not(.close),input,select').forEach(b=>b.disabled=true);
try{const mj=await loadMujoco();engine=new RobotEngine(mj);await cad();$('loading').hidden=true;$('engine-status').textContent='MuJoCo ready';$('dot').style.background='#5de2ed';document.querySelectorAll('button,input,select').forEach(b=>b.disabled=false);}catch(e){$('loading').textContent='Could not initialize MuJoCo: '+e.message;$('engine-status').textContent='Engine unavailable';}
