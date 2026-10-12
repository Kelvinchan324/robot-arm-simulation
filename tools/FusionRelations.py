import adsk.core, adsk.fusion, json, pathlib, traceback
def run(context):
    app=adsk.core.Application.get(); dest=pathlib.Path(__file__).resolve().parent.parent/'cad-export'
    try:
        root=adsk.fusion.Design.cast(app.activeProduct).rootComponent
        d=json.loads((dest/'assembly.json').read_text(encoding='utf-8'))
        byname={j['name']:j for j in d['joints']}
        links={}
        for j in root.allAsBuiltJoints:
            item=byname[j.name]
            item['transform_cm']=list(j.transform.asArray())
            item['origin_cm']=list(j.geometry.origin.asArray()) if j.geometry else None
            for ml in j.motionLinks:
                links[ml.name]={'name':ml.name,'jointOne':ml.jointOne.name,'jointTwo':ml.jointTwo.name if ml.jointTwo else None,'valueOne':ml.valueOne.value,'valueTwo':ml.valueTwo.value,'reversed':ml.isReversed,'suppressed':ml.isSuppressed}
        d['rigidGroups']=[{'name':g.name,'suppressed':g.isSuppressed,'occurrences':[o.fullPathName for o in g.occurrences]} for g in root.allRigidGroups]
        d['motionLinks']=list(links.values())
        for item in d['occurrences']:
            if item['path']=='__root__':continue
            o=next((x for x in root.allOccurrences if x.fullPathName==item['path']),None)
            item['visible']=o.isVisible if o else False
        (dest/'assembly.json').write_text(json.dumps(d,indent=2),encoding='utf-8')
        app.log(f'EXTRACTED: {len(d["rigidGroups"])} rigid groups, {len(links)} motion links, as-built joint transforms.')
    except:
        (dest/'relations-error.txt').write_text(traceback.format_exc(),encoding='utf-8');app.log(traceback.format_exc())
