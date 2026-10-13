import {LiveJog} from './live-jog.js';

// Owns the only motor-command path. The visible engine receives feedback only;
// a separate engine holds the bounded keyboard setpoints.
export class HardwareSession{
  constructor({link,engine,angles,profile,onFeedback=()=>{},onState=()=>{},onFault=()=>{},canMove=()=>true,clock=()=>performance.now()}){
    Object.assign(this,{link,engine,angles,profile,onFeedback,onState,onFault,canMove,clock});
    this.ids=[1,2,3,4,5];this.active=true;this.ready=false;this.generation=0;this.lastCycle=0;this.lastHealth=-Infinity;this.health=[];this.holdRequested=false;this.enableRequested=false;
    this.originalLimits=engine.joints.map(j=>[j.lo,j.hi]);
    if(profile){
      profile.maps.forEach((m,i)=>{const bounds=m.safe_degrees.map(d=>angles[i].fromDisplay(d)).sort((a,b)=>a-b);[engine.joints[i].lo,engine.joints[i].hi]=bounds;});
      this.controller=new LiveJog(engine,angles,profile);
    }
  }
  async start(){await this.cycle();if(this.active)this.schedule();}
  schedule(){this.timer=setTimeout(async()=>{await this.cycle();if(this.active)this.schedule();},80);}
  targets(degrees){return this.profile.maps.map((m,i)=>({id:m.id,raw:m.toRaw(degrees[i]),speed:m.speed,acceleration:m.acceleration}));}
  enable(){if(!this.ready||!this.profile||!this.active)throw Error('Connect and load measured calibration first');this.enableRequested=true;this.generation++;}
  pause(){
    if(this.controller?.enabled)this.holdRequested=true;
    this.enableRequested=false;this.generation++;this.controller?.stop();
    this.onState(this.ready?'Paused · measured pose':'Connecting…');
  }
  key(key,down){this.controller?.key(key,down);}
  async cycle(){
    if(!this.active||this.busy)return;this.busy=true;
    const started=this.clock(),revision=this.generation;
    try{
      if((this.controller?.enabled||this.enableRequested)&&!this.canMove())this.pause();
      if(this.controller?.enabled&&this.lastCycle&&started-this.lastCycle>500){this.pause();this.onState('Paused · control loop delayed');}
      this.lastCycle=started;
      const checkHealth=!!(this.controller?.enabled||this.enableRequested||started-this.lastHealth>=1000);
      const feedback=await this.link.readAll(this.ids,{health:checkHealth});
      if(!this.active)return;
      if((this.controller?.enabled||this.enableRequested)&&!this.canMove())this.pause();
      if(checkHealth){this.health=feedback;this.lastHealth=this.clock();}
      this.raw=feedback.map(s=>s.raw);
      const measured=this.profile?feedback.map((s,i)=>this.profile.maps[i].fromRaw(s.raw)):null;
      this.onFeedback({feedback,degrees:measured});
      if(!this.ready){this.ready=true;this.onState(this.profile?'Paused · measured pose':'Read only · load calibration to move');}
      // Pause may arrive during a read. Only a freshly measured hold is allowed
      // after that event; never the pre-pause IK result or gripper endpoint.
      if(this.holdRequested&&this.profile){
        this.holdRequested=false;
        if(this.clock()-started>500)throw Error('Feedback delayed — hold not sent');
        await this.link.writeTargets(this.targets(measured));this.engine.q=measured.map((d,i)=>this.angles[i].fromDisplay(d));
        return;
      }
      if(revision!==this.generation)return;
      if(this.enableRequested||this.controller?.enabled){
        if(this.clock()-started>500)throw Error('Feedback delayed — keyboard paused');
        for(const state of this.health){
          if(state.mode!==0)throw Error(`Motor ${state.id}: position mode required`);
          if(state.torque!==1)throw Error(`Motor ${state.id}: torque is off — prepare with vendor tools`);
        }
      }
      if(this.enableRequested){
        this.enableRequested=false;this.engine.q=measured.map((d,i)=>this.angles[i].fromDisplay(d));
        if(!this.controller.enable()){this.onState(this.controller.status);return;}
      }
      if(!this.controller?.enabled)return;
      const displayed=this.engine.q.map((q,i)=>this.angles[i].toDisplay(q));
      if(displayed.some((d,i)=>Math.abs(d-measured[i])>this.profile.trackingError))throw Error('Tracking error — arm did not follow the target');
      const changed=this.controller.tick(.08);
      if(this.clock()-started>500)throw Error('Control step delayed — keyboard paused');
      if(changed&&revision===this.generation&&this.active&&this.controller.enabled){
        await this.link.writeTargets(this.targets(this.engine.q.map((q,i)=>this.angles[i].toDisplay(q))));
      }
      this.onState(this.controller.status);
    }catch(error){await this.fault(error);}
    finally{this.busy=false;this.finishCycle?.();this.finishCycle=null;}
  }
  async fault(error){
    this.active=false;this.ready=false;this.generation++;this.controller?.stop();clearTimeout(this.timer);
    this.onFault(error);await this.link.close();this.restoreLimits();
  }
  restoreLimits(){this.originalLimits.forEach(([lo,hi],i)=>Object.assign(this.engine.joints[i],{lo,hi}));}
  async disconnect(){
    this.pause();clearTimeout(this.timer);
    if(this.busy)await new Promise(resolve=>this.finishCycle=resolve);
    // A disconnect requested while idle still gets one fresh hold attempt.
    if(this.active&&this.holdRequested)await this.cycle();
    this.active=false;this.ready=false;this.generation++;await this.link.close();this.restoreLimits();
  }
}
