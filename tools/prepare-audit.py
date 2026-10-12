from pathlib import Path
import json,numpy as np
p=Path(__file__).resolve().parents[1];d=json.loads((p/'cad-export/assembly.json').read_text());a=json.loads((p/'cad-export/motion-audit.json').read_text());m=json.loads((p/'dist/robot/model.json').read_text())
R=np.array(m['cad_to_lab_rotation']);shift=np.array(m['cad_to_lab_shift_m']);names=['base','base_gear','yaw_platform','upper_arm','forearm','wrist_carrier','gripper_pinion','jaw_a','jaw_b'];result=[]
for t in a['tests']:
    bodies={}
    for j in m['joints']:
        idx=j['child'];path=d['rigidGroups'][idx]['occurrences'][0]
        T0=np.array(a['before'][path]).reshape(4,4);T1=np.array(t['transforms'][path]).reshape(4,4);T0[:3,3]*=.01;T1[:3,3]*=.01
        delta=T1@np.linalg.inv(T0);origin=np.array(j['origin']);origCAD=R.T@(origin+shift)
        bodies[names[idx]]={'p':(R@(delta[:3,:3]@origCAD+delta[:3,3])-shift).tolist(),'r':(R@delta[:3,:3]@R.T).flatten().tolist()}
    q={j['name']:((t['joint_values'][j['fusion_name']]*.01-j['q0']) if j['slide'] else ((t['joint_values'][j['fusion_name']]-j['q0']+np.pi)%(2*np.pi)-np.pi)) for j in m['joints']}
    result.append({'name':t['joint'],'q':q,'bodies':bodies})
(p/'cad-export/audit-expected.json').write_text(json.dumps(result))
