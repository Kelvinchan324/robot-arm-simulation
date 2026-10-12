"""Convert the inspected Fusion rigid groups and as-built joints to URDF/MJCF.
The exported CAD pose is q=0. Fusion centimetres become metres; +Y becomes +Z.
Inertia is a documented coarse box approximation for kinematics, not dynamics.
"""
from pathlib import Path
import json,math,zipfile
import numpy as np
import xml.etree.ElementTree as E
P=Path(__file__).resolve().parents[1]; src=P/'cad-export';out=P/'dist'/'robot';out.mkdir(exist_ok=True)
d=json.loads((src/'assembly.json').read_text());audit=json.loads((src/'motion-audit.json').read_text())
names=['base','base_gear','yaw_platform','upper_arm','forearm','wrist_carrier','gripper_pinion','jaw_a','jaw_b']
jnames={'01 - BASE YAW':'base_yaw','00 - Base servo gear':'base_gear_drive','02 - SHOULDER':'shoulder','03 - ELBOW':'elbow','04 - WRIST':'wrist_pitch','05 - GRIPPER DRIVE':'gripper_drive','06 - JAW A SLIDE':'jaw_a_slide','07 - JAW B SLIDE':'jaw_b_slide'}
groups=d['rigidGroups'];membership={p:i for i,g in enumerate(groups) for p in g['occurrences']}
occ={o['path']:o for o in d['occurrences']};R=np.array([[1,0,0],[0,0,-1],[0,1,0.]])
vertices=[[] for _ in groups];faces=[[] for _ in groups];counts=[0]*len(groups)
for o in d['occurrences']:
    if not o['meshes']:continue
    assert o['path'] in membership, f'Unassigned body: {o["path"]}'
    if not o.get('visible',True):continue
    gi=membership[o['path']];T=np.array(o['transform_cm']).reshape(4,4);T[:3,3]*=.01
    for mesh in o['meshes']:
        if not mesh['visible']:continue
        vv=[];ff=[]
        for line in (src/mesh['file']).read_text().splitlines():
            a=line.split()
            if a[0]=='v':vv.append([float(x) for x in a[1:4]])
            elif a[0]=='f':ff.append([int(x)-1+counts[gi] for x in a[1:4]])
        v=np.array(vv);v=(v@T[:3,:3].T+T[:3,3])@R.T
        vertices[gi].append(v);faces[gi].extend(ff);counts[gi]+=len(v)
vertices=[np.concatenate(v) for v in vertices]
basejoint=next(j for j in d['joints'] if j['name']=='01 - BASE YAW');baseorigin=R@np.array(basejoint['origin_cm'])*.01
shift=np.array([baseorigin[0],baseorigin[1],vertices[0][:,2].min()])
vertices=[v-shift for v in vertices]
J=[];origins={0:np.zeros(3)}
for j in d['joints']:
    child=membership[j['occurrenceOne']];parent=membership[j['occurrenceTwo']]
    pos=R@np.array(j['origin_cm'])*.01-shift;origins[child]=pos
    slide=j['type']==2;axis=R@np.array(j['slideDirectionVector'] if slide else j['rotationAxisVector']);axis/=np.linalg.norm(axis)
    q0=j['slideValue']*.01 if slide else (j['rotationValue']+math.pi)%(2*math.pi)-math.pi
    lim=j['slideLimits'] if slide else j['rotationLimits'];limited=lim['isMinimumValueEnabled'] and lim['isMaximumValueEnabled']
    scale=.01 if slide else 1
    lo=lim['minimumValue']*scale-q0 if limited else -math.pi;hi=lim['maximumValue']*scale-q0 if limited else math.pi
    J.append(dict(name=jnames[j['name']],fusion_name=j['name'],child=child,parent=parent,origin=pos.tolist(),axis=axis.tolist(),slide=slide,q0=q0,lo=lo,hi=hi,limited=limited))
couplings=[]
for ml in d['motionLinks']:
    if ml['suppressed']:continue
    one=next(j for j in J if j['fusion_name']==ml['jointOne']);two=next(j for j in J if j['fusion_name']==ml['jointTwo'])
    ratio=ml['valueTwo']/ml['valueOne']*(-1 if ml['reversed'] else 1)*(.01 if two['slide'] else 1)/( .01 if one['slide'] else 1)
    couplings.append({'master':one['name'],'slave':two['name'],'multiplier':ratio,'offset':0})
    # Propagate measured travel bounds from a driven jaw to its rotary drive.
    if two['limited']:
        bounds=sorted([two['lo']/ratio,two['hi']/ratio]);one['lo']=max(one['lo'],bounds[0]);one['hi']=min(one['hi'],bounds[1]);one['limited']=True
def fmt(a):return ' '.join(f'{float(x):.10g}' for x in a)
colors=['0.25 0.31 0.38 1','0.62 0.71 0.77 1','0.17 0.50 0.61 1','0.62 0.73 0.81 1','0.62 0.73 0.81 1','0.19 0.53 0.63 1','0.74 0.49 0.19 1','0.73 0.78 0.82 1','0.73 0.78 0.82 1']
for i,v in enumerate(vertices):
    v=v-origins[i]
    with (out/f'{names[i]}.obj').open('w') as f:
        f.write(f'# {groups[i]["name"]}; metres; source CAD geometry\n')
        for p in v:f.write('v '+fmt(p)+'\n')
        for face in faces[i]:f.write('f '+' '.join(str(x+1) for x in face)+'\n')
# Define a reproducible jaw-tip midpoint; tool +Z points out of the gripper.
drive=next(j for j in J if j['name']=='gripper_drive');z=-np.array(drive['axis']);x=np.array(next(j for j in J if j['name']=='jaw_a_slide')['axis']);y=np.cross(z,x);y/=np.linalg.norm(y);x=np.cross(y,z)
toolR=np.column_stack([x,y,z]);jaws=np.concatenate([vertices[7],vertices[8]]);local=jaws@toolR
toolp=toolR@np.array([(local[:,0].min()+local[:,0].max())/2,(local[:,1].min()+local[:,1].max())/2,local[:,2].max()])
def rpy(m):return [math.atan2(m[2,1],m[2,2]),math.atan2(-m[2,0],math.hypot(m[0,0],m[1,0])),math.atan2(m[1,0],m[0,0])]
def quat(m):
    rx,ry,rz=[a/2 for a in rpy(m)];cx,cy,cz=map(math.cos,[rx,ry,rz]);sx,sy,sz=map(math.sin,[rx,ry,rz])
    return [cx*cy*cz+sx*sy*sz,sx*cy*cz-cx*sy*sz,cx*sy*cz+sx*cy*sz,cx*cy*sz-sx*sy*cz]
urdf=E.Element('robot',name='ENGG1101_Robot_Arm');urdf.append(E.Comment('CAD geometry and relations. q=0 is the exported Fusion pose. Inertias are coarse approximations; kinematics use only.'))
mu=E.SubElement(urdf,'mujoco');E.SubElement(mu,'compiler',discardvisual='false',fusestatic='false')
for i,name in enumerate(names):
    l=E.SubElement(urdf,'link',name=name);v=vertices[i]-origins[i];size=np.maximum(v.max(axis=0)-v.min(axis=0),.001);center=(v.max(axis=0)+v.min(axis=0))/2;mass=max(.01,float(np.prod(size)*100));inertia=mass/12*np.array([size[1]**2+size[2]**2,size[0]**2+size[2]**2,size[0]**2+size[1]**2])
    inert=E.SubElement(l,'inertial');E.SubElement(inert,'origin',xyz=fmt(center));E.SubElement(inert,'mass',value=str(mass));E.SubElement(inert,'inertia',ixx=str(inertia[0]),iyy=str(inertia[1]),izz=str(inertia[2]),ixy='0',ixz='0',iyz='0')
    vis=E.SubElement(l,'visual');g=E.SubElement(vis,'geometry');E.SubElement(g,'mesh',filename=f'{name}.obj');mat=E.SubElement(vis,'material',name=name+'_material');E.SubElement(mat,'color',rgba=colors[i])
for j in J:
    el=E.SubElement(urdf,'joint',name=j['name'],type='prismatic' if j['slide'] else ('revolute' if j['limited'] else 'continuous'))
    E.SubElement(el,'parent',link=names[j['parent']]);E.SubElement(el,'child',link=names[j['child']]);E.SubElement(el,'origin',xyz=fmt(np.array(j['origin'])-origins[j['parent']]),rpy='0 0 0');E.SubElement(el,'axis',xyz=fmt(j['axis']))
    E.SubElement(el,'limit',lower=str(j['lo']),upper=str(j['hi']),effort='1',velocity='1')
    c=next((c for c in couplings if c['slave']==j['name']),None)
    if c:E.SubElement(el,'mimic',joint=c['master'],multiplier=str(c['multiplier']),offset='0')
E.SubElement(urdf,'link',name='tool_tip');tj=E.SubElement(urdf,'joint',name='tool_mount',type='fixed');E.SubElement(tj,'parent',link='wrist_carrier');E.SubElement(tj,'child',link='tool_tip');E.SubElement(tj,'origin',xyz=fmt(toolp-origins[5]),rpy=fmt(rpy(toolR)))
E.indent(urdf);E.ElementTree(urdf).write(out/'engg1101.urdf',encoding='utf-8',xml_declaration=True)
mj=E.Element('mujoco',model='ENGG1101_Robot_Arm');mj.append(E.Comment('Kinematics model: no contact geoms, coarse box inertia; actuator parameters are not calibrated.'))
E.SubElement(mj,'compiler',angle='radian',inertiafromgeom='false');E.SubElement(mj,'option',gravity='0 0 -9.81');assets=E.SubElement(mj,'asset')
for name in names:E.SubElement(assets,'mesh',name=name+'_mesh',file=name+'.obj')
world=E.SubElement(mj,'worldbody');bodies={0:E.SubElement(world,'body',name='base')}
def addbody(i):
    if i in bodies:return bodies[i]
    j=next(j for j in J if j['child']==i);parent=addbody(j['parent']);b=E.SubElement(parent,'body',name=names[i],pos=fmt(origins[i]-origins[j['parent']]));bodies[i]=b
    kw={'name':j['name'],'type':'slide' if j['slide'] else 'hinge','axis':fmt(j['axis']),'limited':str(j['limited']).lower()}
    if j['limited']:kw['range']=fmt([j['lo'],j['hi']])
    E.SubElement(b,'joint',**kw);return b
for i,name in enumerate(names):
    b=addbody(i);il=urdf.find(f"link[@name='{name}']/inertial");ii=il.find('inertia');E.SubElement(b,'inertial',pos=il.find('origin').get('xyz'),mass=il.find('mass').get('value'),diaginertia=' '.join(ii.get(k) for k in ['ixx','iyy','izz']))
    E.SubElement(b,'geom',type='mesh',mesh=name+'_mesh',rgba=colors[i],contype='0',conaffinity='0',group='1')
E.SubElement(bodies[5],'body',name='tool_tip',pos=fmt(toolp-origins[5]),quat=fmt(quat(toolR)))
eq=E.SubElement(mj,'equality')
for c in couplings:E.SubElement(eq,'joint',joint1=c['slave'],joint2=c['master'],polycoef=fmt([0,c['multiplier'],0,0,0]))
E.indent(mj);E.ElementTree(mj).write(out/'engg1101.xml',encoding='utf-8',xml_declaration=True)
metadata={'name':'ENGG1101 · Fusion arm','source':'ENGG1101_Robot_Arm_relation.f3d','source_occurrences':len(d['occurrences']),'rigid_links':9,'physical_joints':8,'independent_controls':5,'assets':[n+'.obj' for n in names],'couplings':couplings,'joints':J,'tool_position':toolp.tolist(),'tool_definition':'Midpoint between jaw-tip extents, computed from CAD meshes; +Z points out of the gripper.','cad_to_lab_rotation':R.tolist(),'cad_to_lab_shift_m':shift.tolist(),'reference':'Joint values are relative to the exported Fusion pose. Base yaw has no CAD limits; UI uses +/-180 degrees.','limitations':'Kinematics only. No collision avoidance or calibrated dynamics. Box inertia and effort/velocity entries are placeholders.','example_pose':[.2,-.25,.35,.65,-.2]}
(out/'model.json').write_text(json.dumps(metadata,indent=2))
(out/'README.txt').write_text('ENGG1101 CAD conversion\n\n'+metadata['reference']+'\n'+metadata['limitations']+'\nTool: '+metadata['tool_definition']+'\n\nURDF contains standard mimic joints. For MuJoCo use engg1101.xml, which expresses the couplings as equality constraints. Kinematic viewers must also apply coupled qpos values before mj_forward. All OBJ meshes are in metres and must remain alongside the model files.\n')
with zipfile.ZipFile(out/'engg1101-robot-package.zip','w',compression=zipfile.ZIP_DEFLATED) as zf:
    for file in [out/'engg1101.urdf',out/'engg1101.xml',out/'model.json',out/'README.txt']+[out/(n+'.obj') for n in names]:zf.write(file,file.name)
print(json.dumps({'triangles':sum(len(f) for f in faces),'vertices':sum(counts),'mesh_MB':sum((out/(n+'.obj')).stat().st_size for n in names)/1e6,'package_MB':(out/'engg1101-robot-package.zip').stat().st_size/1e6,'tool_m':toolp.tolist(),'joint_limits':[(j['name'],j['lo'],j['hi']) for j in J]},indent=2))
