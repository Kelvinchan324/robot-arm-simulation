from pathlib import Path
p=Path(__file__).resolve().parents[1]/'dist'
s=(p/'app.js').read_text()
s=s.replace("trace=[],isDemo=true", "trace=[],isDemo=true,isCAD=false,robotConfig=null")
s=s.replace("'#152720'","'#08121d'").replace('0xeaf8e0,0x243328','0xd8f1ff,0x13263a').replace('0x52654e,0x2c4033','0x285169,0x142d40').replace('0xb8d77d','0x5de2ed').replace('0xc1db83','0x5de2ed').replace('SphereGeometry(.014','SphereGeometry(.005')
s=s.replace('d.xanchor,i*3','d.xanchor,engine.joints[i].id*3')
s=s.replace('`${engine.joints.length} AXES`','`${engine.joints.length} CONTROLS`')
s=s.replace("engine.q=engine.joints.map((j,i)=>j.slide?0:[.4,-.85,1.4,-.5,.2][i%5])", "engine.q=isCAD?robotConfig.example_pose:engine.joints.map((j,i)=>j.slide?0:[.4,-.85,1.4,-.5,.2][i%5])")
s=s.replace('isDemo=true;$(\'model-name\')','isDemo=true;isCAD=false;$(\'model-name\')').replace('Demonstration geometry · CAD conversion pending','Optional teaching example · simplified geometry')
pos=s.index("for(const mode of")
s=s[:pos]+'''async function cad(){
  robotConfig=await (await fetch('./robot/model.json')).json();
  const xml=await (await fetch('./robot/engg1101.xml')).text();
  const assets=await Promise.all(robotConfig.assets.map(async name=>[name,new Uint8Array(await (await fetch('./robot/'+name)).arrayBuffer())]));
  engine.load(xml,assets,robotConfig);modelXML=xml;modelFilename='engg1101.xml';isDemo=false;isCAD=true;
  $('model-name').textContent='ENGG1101 / YOUR ROBOT';$('model-note').textContent='Fusion geometry · 8 joints · 5 independent controls';
  ['ox','oy','oz'].forEach(id=>$(id).value=0);rebuild();
  engine.pose().p.forEach((v,i)=>$(['tx','ty','tz'][i]).value=v.toFixed(5));targetChange();
}
''' +s[pos:]
s=s.replace("$('download-model').onclick=()=>download(modelXML,modelFilename,'application/xml');", "$('download-model').onclick=()=>{if(isCAD){const a=document.createElement('a');a.href='./robot/engg1101-robot-package.zip';a.download='engg1101-robot-package.zip';a.click();}else download(modelXML,modelFilename,'application/xml');};")
s=s.replace("await demo();$('import-dialog').close()", "await cad();$('import-dialog').close()")
s=s.replace("isDemo=false;$('model-name')", "isDemo=false;isCAD=false;$('model-name')")
s=s.replace("await demo();$('loading')", "await cad();$('loading')").replace("'#78a94f'","'#5de2ed'")
# Linear MJCF equality couplings are applied explicitly by the kinematic viewer.
s=s.replace('engine.load(xml,assets);modelXML', "const couplings=Array.from(doc.querySelectorAll('equality > joint')).map(el=>{const a=(el.getAttribute('polycoef')||'0 1 0 0 0').trim().split(/\\s+/).map(Number);if(!el.getAttribute('joint2')||a.slice(2).some(v=>v!==0))throw Error('Only linear two-joint equality couplings are supported.');return {slave:el.getAttribute('joint1'),master:el.getAttribute('joint2'),offset:a[0],multiplier:a[1]};});engine.load(xml,assets,{couplings});modelXML")
(p/'app.js').write_text(s)
s=(p/'index.html').read_text()
s=s.replace('THE KINEMATICS WORKBENCH','KINEMATICS / CONTROL STATION').replace('From angles to motion.','Command the motion.').replace('Move a joint. Find a position. Understand the connection.','Your ENGG1101 arm. Real geometry. Explore every transform.')
s=s.replace('Teaching arm</span>','ENGG1101 / YOUR ROBOT</span>').replace('Demonstration geometry · CAD conversion pending','Loading Fusion geometry and joint relationships')
s=s.replace('Zero pose','Reference pose').replace('zero pose','reference pose').replace('↓ Robot file','↓ Robot package').replace('Restore teaching arm','Restore ENGG1101 arm')
s=s.replace('Angles are shown in degrees; MuJoCo computes in radians. Sliders respect model limits. Unbounded joints use a teaching range.','Angles are relative to the exported Fusion pose. The arm has four positioning joints plus a gripper drive. Gears and jaws move together automatically; base yaw uses a ±180° teaching range.')
s=s.replace('Your Fusion archive is identified. Its joint and mesh export is still needed to replace the teaching arm.','Your actual Fusion geometry: 9 rigid groups, 8 joints and 3 motion links. The download includes URDF, MuJoCo XML and all link meshes.')
start=s.index('<h2>Make your CAD move correctly</h2>');end=s.index('</dialog>',start)
s=s[:start]+'''<h2>From Fusion to this workbench</h2><p>Converted from <b>ENGG1101_Robot_Arm_relation.f3d</b>. The model uses the original CAD meshes, nine rigid groups, eight as-built joints and three motion links.</p><ol><li><b>Five independent controls:</b> base yaw, shoulder, elbow, wrist pitch and gripper drive. The base gear follows yaw in reverse. Both jaws follow the gripper pinion.</li><li><b>Reference pose:</b> all displayed angles are changes from the exported CAD pose. Shoulder, elbow, wrist and jaw travel come from the Fusion limits. The unrestricted base uses ±180° for teaching.</li><li><b>Tool frame:</b> the midpoint between the jaw-tip extents, calculated from the meshes. Tool +Z points out of the gripper. You can change the selected link or add an offset.</li><li><b>Downloads:</b> the package contains URDF with mimic joints, MuJoCo XML with joint equalities, and metre-scale OBJ meshes. Use the XML when importing this package into this workbench.</li></ol><p>This is a kinematics workbench: MuJoCo computes the transforms and a numerical solver handles position-only inverse kinematics. Inertia and effort values are placeholders; collision geometry and actuators are not calibrated for dynamics.</p><p><a href="https://github.com/google-deepmind/mujoco/tree/main/wasm" target="_blank" rel="noreferrer">MuJoCo browser bindings</a></p>'''+s[end:]
(p/'index.html').write_text(s)
