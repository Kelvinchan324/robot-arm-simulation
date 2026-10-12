import * as THREE from 'three';
// A screen point defines a ray, so an explicit horizontal plane fixes depth.
export function pointOnPlane(camera,rect,clientX,clientY,height){
  if(!rect.width||!rect.height||!Number.isFinite(height))return null;
  const ray=new THREE.Raycaster();ray.setFromCamera(new THREE.Vector2((clientX-rect.left)/rect.width*2-1,-(clientY-rect.top)/rect.height*2+1),camera);
  const point=ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0,0,1),-height),new THREE.Vector3());
  return point&&Math.abs(point.x)<=1&&Math.abs(point.y)<=1?point:null;
}
export class PointTarget {
  constructor({scene,camera,canvas,orbit,onChange,onCommit,onMiss}){
    Object.assign(this,{scene,camera,canvas,orbit,onChange,onCommit,onMiss});this.enabled=false;this.height=0;this.pointer=null;
    this.plane=new THREE.Mesh(new THREE.PlaneGeometry(2,2),new THREE.MeshBasicMaterial({color:0x58cce6,transparent:true,opacity:.06,side:THREE.DoubleSide,depthWrite:false}));
    this.grid=new THREE.GridHelper(2,20,0x59bcd0,0x2a657a);this.grid.rotation.x=Math.PI/2;this.grid.material.transparent=true;this.grid.material.opacity=.35;
    scene.add(this.plane,this.grid);this.enable(false);
    canvas.addEventListener('pointerdown',e=>{if(!this.enabled||e.button!==0||this.pointer!==null)return;e.preventDefault();e.stopImmediatePropagation();this.pointer=e.pointerId;this.changed=false;orbit.enabled=false;canvas.setPointerCapture(e.pointerId);this.place(e);},true);
    canvas.addEventListener('pointermove',e=>{if(this.pointer!==e.pointerId)return;e.preventDefault();e.stopImmediatePropagation();this.place(e);},true);
    canvas.addEventListener('pointerup',e=>{if(this.pointer!==e.pointerId)return;e.preventDefault();e.stopImmediatePropagation();this.place(e);const changed=this.changed;this.release();if(changed)onCommit();},true);
    canvas.addEventListener('pointercancel',()=>this.release(),true);
    canvas.addEventListener('lostpointercapture',()=>this.release(),true);
    window.addEventListener('blur',()=>this.release());
    document.addEventListener('visibilitychange',()=>{if(document.hidden)this.release();});
  }
  place(event){const p=pointOnPlane(this.camera,this.canvas.getBoundingClientRect(),event.clientX,event.clientY,this.height);if(p){this.changed=true;this.onChange(p.toArray());}else this.onMiss();}
  release(){const id=this.pointer;this.pointer=null;this.orbit.enabled=true;if(id!==null&&this.canvas.hasPointerCapture(id))this.canvas.releasePointerCapture(id);}
  enable(value){this.release();this.enabled=value;this.plane.visible=this.grid.visible=value;this.canvas.style.cursor=value?'crosshair':'';}
  setHeight(value){this.height=value;this.plane.position.z=value;this.grid.position.y=0;this.grid.position.z=value;}
}
