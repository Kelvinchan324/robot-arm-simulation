import adsk.core,adsk.fusion,json,pathlib,traceback
def run(context):
    app=adsk.core.Application.get();root=adsk.fusion.Design.cast(app.activeProduct).rootComponent
    dest=pathlib.Path(__file__).resolve().parent.parent/'cad-export'
    joints=list(root.allAsBuiltJoints)
    def capture():return {o.fullPathName:list(o.transform2.asArray()) for o in root.allOccurrences}
    audit={'before':capture(),'tests':[],'errors':[]}
    for j in joints:
        if not any(j.name.startswith(s) for s in ['01','02','03','04','05']):continue
        motion=j.jointMotion;old=motion.rotationValue
        # 0.01 radians stays within the current shoulder, elbow and wrist ranges.
        try:
            motion.rotationValue=old+.01;adsk.doEvents()
            audit['tests'].append({'joint':j.name,'delta':.01,'transforms':capture(),'joint_values':{k.name:(k.jointMotion.rotationValue if k.jointMotion.jointType==1 else k.jointMotion.slideValue) for k in joints}})
        except:audit['errors'].append(traceback.format_exc())
        finally:motion.rotationValue=old;adsk.doEvents()
    audit['after']=capture()
    (dest/'motion-audit.json').write_text(json.dumps(audit),encoding='utf-8')
    app.log('Motion audit saved; joint values restored.')
