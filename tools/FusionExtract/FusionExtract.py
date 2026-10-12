"""Run inside Fusion with the robot design active. Does not modify the design.
This extracts evidence and meshes, not a verified URDF. A separate conversion
must resolve the joint graph, rigid groups, zero poses, tool frame and limits.
"""
import adsk.core, adsk.fusion, json, traceback, pathlib

def run(context):
    app=adsk.core.Application.get(); ui=app.userInterface
    destination=pathlib.Path(__file__).resolve().parents[2]/'cad-export'
    destination.mkdir(exist_ok=True)
    try:
        design=adsk.fusion.Design.cast(app.activeProduct)
        if not design: raise RuntimeError('Open the robot design in the Design workspace first.')
        root=design.rootComponent
        out={'design':root.name,'length_units':'cm in matrices; m in OBJ meshes','occurrences':[],'joints':[],'errors':[]}
        def vec(v): return list(v.asArray()) if v else None
        def get(o,key):
            try:return getattr(o,key)
            except:return None
        def occurrence(o):return o.fullPathName if o else '__root__'
        entries=[('__root__',root,None)]+[(o.fullPathName,o.component,o) for o in root.allOccurrences]
        for index,(path,comp,occ) in enumerate(entries):
            item={'path':path,'name':comp.name,'transform_cm':vec(occ.transform2) if occ else [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],'grounded':bool(get(occ,'isGrounded')) if occ else True,'meshes':[]}
            # Native body tessellation stays in the component coordinate frame.
            for bi,body in enumerate(comp.bRepBodies):
                try:
                    calc=body.meshManager.createMeshCalculator();calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.NormalQualityTriangleMesh)
                    mesh=calc.calculate();filename=f'component_{index:03d}_body_{bi:03d}.obj'
                    points=mesh.nodeCoordinates;indices=mesh.nodeIndices
                    with (destination/filename).open('w',encoding='utf-8') as f:
                        for p in points:f.write(f'v {p.x*.01:.10g} {p.y*.01:.10g} {p.z*.01:.10g}\n')
                        for k in range(0,len(indices),3):f.write('f '+' '.join(str(indices[k+j]+1) for j in range(3))+'\n')
                    item['meshes'].append({'file':filename,'body':body.name,'visible':bool(body.isVisible)})
                except:out['errors'].append({'component':path,'body':body.name,'error':traceback.format_exc()})
            out['occurrences'].append(item)
        for collection_name in ['allJoints','allAsBuiltJoints']:
            collection=get(root,collection_name)
            if collection is None:continue
            for joint in collection:
                try:
                    motion=joint.jointMotion
                    item={'name':joint.name,'collection':collection_name,'occurrenceOne':occurrence(joint.occurrenceOne),'occurrenceTwo':occurrence(joint.occurrenceTwo),'type':int(motion.jointType),'motionType':motion.objectType,'suppressed':bool(get(joint,'isSuppressed')),'locked':bool(get(joint,'isLocked')),'geometryOneTransform_cm':vec(get(joint,'geometryOneTransform')),'geometryTwoTransform_cm':vec(get(joint,'geometryTwoTransform'))}
                    for prop in ['rotationAxisVector','slideDirectionVector']:item[prop]=vec(get(motion,prop))
                    for prop in ['rotationValue','slideValue']:item[prop]=get(motion,prop)
                    for prop in ['rotationLimits','slideLimits']:
                        limits=get(motion,prop)
                        if limits:item[prop]={k:get(limits,k) for k in ['isMinimumValueEnabled','isMaximumValueEnabled','minimumValue','maximumValue']}
                    out['joints'].append(item)
                except:out['errors'].append({'joint':joint.name,'error':traceback.format_exc()})
        (destination/'assembly.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
        ui.messageBox(f'Extracted {len(out["occurrences"])} occurrences and {len(out["joints"])} joints.\n{len(out["errors"])} extraction errors.\nSaved to {destination}\nReturn to the chat for joint-graph conversion and verification.')
    except:
        (destination/'error.txt').write_text(traceback.format_exc(),encoding='utf-8')
        ui.messageBox(traceback.format_exc())
