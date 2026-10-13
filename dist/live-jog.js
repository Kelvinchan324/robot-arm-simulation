import {CartesianJog} from './jog.js';
import {stepGripper} from './gripper-jog.js';
// Bounded counterpart of raspberry_pi/controller.py; same Cartesian equations.
export class LiveJog{
  constructor(engine,angles,profile){this.engine=engine;this.angles=angles;this.profile=profile;this.jog=new CartesianJog(engine);this.held=new Set();this.stop();}
  stop(){this.enabled=false;this.held.clear();this.levelTarget=null;this.status='Paused';}
  enable(){
    const q=this.engine.q;
    if(!this.jog.levelWrist()){this.status='Cannot level at this wrist limit';return false;}
    this.levelTarget=this.engine.q[3];this.engine.q=q;this.held.clear();this.enabled=true;this.status='Levelling wrist';return true;
  }
  key(key,down){
    if(!down){this.held.delete(key);if(this.enabled&&(key==='j'||key==='l')&&!this.held.has('j')&&!this.held.has('l'))this.status=this.levelTarget===null?'On · pitch held':'Levelling wrist';return;}
    if(!this.enabled||this.held.has(key))return;
    this.held.add(key);
  }
  tick(dt=.08){
    if(!this.enabled)return false;
    if(!Number.isFinite(dt)||dt<=0)throw Error('Invalid control timestep');dt=Math.min(dt,.08);
    const e=this.engine,before=e.q,max=this.profile.maps.map((m,i)=>m.max_deg_s*dt/Math.abs(this.angles[i].gain));
    const ramp=(value,target,step)=>value+Math.max(-step,Math.min(step,target-value));
    if(this.levelTarget!==null){
      const q=e.q;q[3]=ramp(q[3],this.levelTarget,max[3]);e.q=q;
      if(Math.abs(q[3]-this.levelTarget)<1e-9){this.levelTarget=null;this.status='On · pitch held';}
    }else{
      const v=k=>this.held.has(k)?1:0;
      const motion={height:(v('w')-v('s'))*.0025,reach:(v('arrowup')-v('arrowdown'))*.0025,yaw:(v('arrowright')-v('arrowleft'))*Math.PI/120,pitch:(v('a')-v('d'))*Math.PI/120};
      if(Object.values(motion).some(Boolean)){
        let moved=false;
        for(let attempt=0;attempt<8;attempt++){
          e.q=before;const scale=this.profile.speedScale*(dt/.08)/2**attempt;
          if(this.jog.step(Object.fromEntries(Object.entries(motion).map(([k,v])=>[k,v*scale])))&&e.q.every((q,i)=>Math.abs(q-before[i])<=max[i]+1e-10)){moved=true;break;}
        }
        if(!moved)e.q=before;this.status=moved?'On · pitch held':'Limit reached';
      }
    }
    const direction=Number(this.held.has('j'))-Number(this.held.has('l'));
    if(stepGripper(e,this.angles,direction,dt,this.profile.maps[4].max_deg_s))this.status=direction>0?'Opening gripper':'Closing gripper';
    return e.q.some((q,i)=>q!==before[i]);
  }
}
