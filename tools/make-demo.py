from pathlib import Path
import xml.etree.ElementTree as E
root=E.Element('robot',name='teaching_arm_not_CAD_conversion')
mu=E.SubElement(root,'mujoco');E.SubElement(mu,'compiler',discardvisual='false',fusestatic='false',balanceinertia='true')
def link(name,size,pos,color):
    l=E.SubElement(root,'link',name=name)
    inert=E.SubElement(l,'inertial');E.SubElement(inert,'origin',xyz=pos);E.SubElement(inert,'mass',value='0.2');E.SubElement(inert,'inertia',ixx='0.001',iyy='0.001',izz='0.001',ixy='0',ixz='0',iyz='0')
    v=E.SubElement(l,'visual');E.SubElement(v,'origin',xyz=pos);g=E.SubElement(v,'geometry');E.SubElement(g,'box',size=size);mat=E.SubElement(v,'material',name=name);E.SubElement(mat,'color',rgba=color)
    return l
link('base','0.13 0.13 0.06','0 0 0.03','0.19 0.25 0.22 1')
link('turntable','0.07 0.07 0.065','0 0 0.025','0.7 0.78 0.67 1')
link('upper_arm','0.24 0.045 0.048','0.12 0 0','0.76 0.85 0.7 1')
link('forearm','0.20 0.035 0.038','0.10 0 0','0.76 0.85 0.7 1')
link('wrist','0.06 0.045 0.045','0.03 0 0','0.35 0.49 0.38 1')
link('gripper','0.035 0.07 0.035','0.0175 0 0','0.76 0.85 0.7 1')
link('tool','0.005 0.005 0.005','0 0 0','0.8 0.93 0.37 1')
for name,parent,child,xyz,axis,lo,hi in [('base_yaw','base','turntable','0 0 0.07','0 0 1',-3.14159,3.14159),('shoulder','turntable','upper_arm','0 0 0.065','0 1 0',-1.7,1.7),('elbow','upper_arm','forearm','0.24 0 0','0 1 0',-2.6,2.6),('wrist_pitch','forearm','wrist','0.20 0 0','0 1 0',-1.8,1.8),('wrist_roll','wrist','gripper','0.06 0 0','1 0 0',-3.14159,3.14159),('tool_mount','gripper','tool','0.085 0 0','1 0 0',0,0)]:
    j=E.SubElement(root,'joint',name=name,type='fixed' if lo==hi else 'revolute');E.SubElement(j,'parent',link=parent);E.SubElement(j,'child',link=child);E.SubElement(j,'origin',xyz=xyz,rpy='0 0 0');E.SubElement(j,'axis',xyz=axis)
    if lo!=hi:E.SubElement(j,'limit',lower=str(lo),upper=str(hi),effort='10',velocity='2')
g=root.find("link[@name='gripper']")
for y in [-.029,.029]:
    v=E.SubElement(g,'visual');E.SubElement(v,'origin',xyz=f'0.06 {y} 0');geo=E.SubElement(v,'geometry');E.SubElement(geo,'box',size='0.06 0.012 0.023');mat=E.SubElement(v,'material',name=f'finger{y}');E.SubElement(mat,'color',rgba='0.28 0.36 0.30 1')
E.indent(root)
Path('dist/demo.urdf').write_text(E.tostring(root,encoding='unicode'),encoding='utf-8')
