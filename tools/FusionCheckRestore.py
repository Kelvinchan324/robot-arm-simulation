import adsk.core,adsk.fusion,json,pathlib
def run(context):
    app=adsk.core.Application.get();root=adsk.fusion.Design.cast(app.activeProduct).rootComponent
    p=pathlib.Path(__file__).resolve().parent.parent/'cad-export'
    before=json.loads((p/'motion-audit.json').read_text())['before']
    error=max(abs(a-b) for o in root.allOccurrences for a,b in zip(o.transform2.asArray(),before[o.fullPathName]))
    (p/'restore-check.json').write_text(json.dumps({'maximum_transform_error':error}))
    app.log('Original pose restore error: '+str(error))
