import json,numpy as np
d=json.load(open('cad-export/assembly.json'));a=json.load(open('cad-export/motion-audit.json'))
groups=d['rigidGroups'];membership={p:i for i,g in enumerate(groups) for p in g['occurrences']}
for t in a['tests']:
    j=next(j for j in d['joints'] if j['name']==t['joint'])
    c,p=j['occurrenceOne'],j['occurrenceTwo']
    C0=np.array(a['before'][c]).reshape(4,4);P0=np.array(a['before'][p]).reshape(4,4)
    C1=np.array(t['transforms'][c]).reshape(4,4);P1=np.array(t['transforms'][p]).reshape(4,4)
    rel0=np.linalg.inv(P0)@C0;rel1=np.linalg.inv(P1)@C1;delta=rel1@np.linalg.inv(rel0)
    axis=P0[:3,:3].T@np.array(j['rotationAxisVector'])
    skew=np.array([delta[2,1]-delta[1,2],delta[0,2]-delta[2,0],delta[1,0]-delta[0,1]])/2
    actual=t['joint_values'][j['name']]-j['rotationValue']
    print(j['name'],'actual angle',actual,'measured',np.dot(skew,axis),'ratio',np.dot(skew,axis)/actual)
