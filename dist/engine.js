// All positions and transforms are computed by MuJoCo, in SI units.
export class RobotEngine {
  constructor(mj) { this.mj=mj; }
  load(xml, assets=[], config={}) {
    const vfs=new this.mj.MjVFS(); let model;
    try {
      for(const [name,bytes] of assets) vfs.addBuffer(name,bytes);
      model=this.mj.MjModel.from_xml_string(xml,vfs);
    } finally { vfs.delete(); }
    let data;
    try { data=new this.mj.MjData(model); } catch(e) { model.delete(); throw e; }
    const unsupported=Array.from(model.jnt_type).some(t=>t!==2&&t!==3);
    if(unsupported) {data.delete();model.delete();throw Error('This lab supports fixed-base models with hinge and slide joints. Remove floating or ball joints.');}
    if(!model.njnt) {data.delete();model.delete();throw Error('No movable joints found. Export the articulated assembly, not a single rigid mesh.');}
    this.data?.delete();this.model?.delete(); this.model=model;this.data=data;
    const obj=this.mj.mjtObj;
    this.joints=Array.from({length:model.njnt},(_,i)=>({id:i,name:this.mj.mj_id2name(model,obj.mjOBJ_JOINT.value,i)||`Joint ${i+1}`,slide:model.jnt_type[i]===2,addr:model.jnt_qposadr[i],lo:model.jnt_limited[i]?model.jnt_range[2*i]:(model.jnt_type[i]===2?-.5:-Math.PI),hi:model.jnt_limited[i]?model.jnt_range[2*i+1]:(model.jnt_type[i]===2?.5:Math.PI),limited:!!model.jnt_limited[i]}));
    this.allJoints=this.joints;
    this.couplings=(config.couplings||[]).map(c=>({...c,master:this.allJoints.find(j=>j.name===c.master),slave:this.allJoints.find(j=>j.name===c.slave)}));
    this.joints=this.allJoints.filter(j=>!this.couplings.some(c=>c.slave===j));
    this.joints.forEach(j=>j.ik=j.name!=='gripper_drive');
    this.bodies=Array.from({length:model.nbody-1},(_,i)=>({id:i+1,name:this.mj.mj_id2name(model,obj.mjOBJ_BODY.value,i+1)||`Link ${i+1}`}));
    this.tip=this.bodies.find(b=>/tool|end_effector|tcp/i.test(b.name))?.id||this.bodies.at(-1).id;
    this.offset=[0,0,0];this.forward();
  }
  get q(){return this.joints.map(j=>this.data.qpos[j.addr]);}
  set q(q){this.joints.forEach((j,i)=>this.data.qpos[j.addr]=Math.max(j.lo,Math.min(j.hi,q[i])));this.forward();}
  forward(){for(const c of this.couplings)this.data.qpos[c.slave.addr]=this.data.qpos[c.master.addr]*c.multiplier+c.offset;this.mj.mj_forward(this.model,this.data);}
  pose(){const r=Array.from(this.data.xmat.slice(this.tip*9,this.tip*9+9)); const p=Array.from(this.data.xpos.slice(this.tip*3,this.tip*3+3));for(let i=0;i<3;i++)p[i]+=r[i*3]*this.offset[0]+r[i*3+1]*this.offset[1]+r[i*3+2]*this.offset[2];return {p,r};}
  solve(target,{iterations=180,tolerance=.0005}={}) {
    if(target.length!==3||!target.every(Number.isFinite))throw Error('Enter three finite target coordinates.');
    let best=this.q, bestError=Infinity,used=0;
    const initial=this.q;
    // Bounded deterministic restarts help escape straight-arm singularities.
    for(let trial=0;trial<5;trial++) {
      this.q=trial===0?initial:this.joints.map((j,i)=>j.ik?Math.max(j.lo,Math.min(j.hi,initial[i]+Math.sin((trial+1)*(i+2))* (j.slide?.05:.6))):initial[i]);
      for(let k=0;k<iterations;k++) {
        used++;const q=this.q,p=this.pose().p,e=target.map((v,i)=>v-p[i]),err=Math.hypot(...e);
        if(err<bestError){bestError=err;best=q.slice();}
        if(err<tolerance){this.q=best;return {converged:true,error:err,iterations:used};}
        const J=this.joints.map((j,i)=>{if(!j.ik)return [0,0,0];const step=q[i]+1e-5<=j.hi?1e-5:-1e-5;const t=q.slice();t[i]+=step;this.q=t;const pp=this.pose().p;return pp.map((v,a)=>(v-p[a])/step);});this.q=q;
        const A=Array.from({length:3},(_,a)=>Array.from({length:3},(_,b)=>J.reduce((s,col)=>s+col[a]*col[b],0)+(a===b?.0001:0)));
        const v=solve3(A,e);this.q=q.map((x,i)=>x+Math.max(-.15,Math.min(.15,J[i].reduce((s,z,a)=>s+z*v[a],0))));
      }
    }
    this.q=best;return {converged:false,error:bestError,iterations:used};
  }
}
function solve3(A,b){const m=A.map((r,i)=>[...r,b[i]]);for(let i=0;i<3;i++){let pivot=i;for(let j=i+1;j<3;j++)if(Math.abs(m[j][i])>Math.abs(m[pivot][i]))pivot=j;[m[i],m[pivot]]=[m[pivot],m[i]];const d=m[i][i];if(Math.abs(d)<1e-15)return [0,0,0];for(let k=i;k<4;k++)m[i][k]/=d;for(let j=0;j<3;j++)if(j!==i){const f=m[j][i];for(let k=i;k<4;k++)m[j][k]-=f*m[i][k];}}return m.map(r=>r[3]);}
