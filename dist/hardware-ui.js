import {SerialLink} from './serial-link.js';
import {HardwareSession} from './hardware-session.js';
import {validateProfile} from './hardware-profile.js';

export function setupHardware({engine,commandEngine,angles,onFeedback,onLock,onPauseSimulation}){
  const $=id=>document.getElementById(id);
  let session=null,profile=null,connecting=false,disconnecting=false,loadingProfile=false;
  const hasControlFocus=()=>!document.hidden&&document.hasFocus()&&document.activeElement===$('canvas');
  const state=text=>{$('serial-status').textContent=text;};
  function controls(){
    const locked=!!session?.active||connecting||disconnecting;
    $('serial-connect').disabled=locked||loadingProfile||!navigator.serial;
    $('serial-disconnect').disabled=!session||disconnecting;
    $('profile-file').disabled=locked;$('serial-baud').disabled=locked;
    $('serial-enable').disabled=!session?.ready||!profile||disconnecting||!session.active||session.enableRequested;
    $('serial-pause').disabled=!session?.active;
    $('serial-enable').textContent=session?.controller?.enabled?'Keys enabled':'Enable keys + level';
    onLock(locked);
  }
  $('profile-file').onchange=async event=>{
    profile=null;loadingProfile=true;controls();$('profile-status').textContent='Reading calibration…';
    try{
      const file=event.target.files[0];if(!file){$('profile-status').textContent='No calibration';return;}
      if(file.size>100000)throw Error('Calibration file is too large');
      profile=validateProfile(JSON.parse(await file.text()),angles);
      $('profile-status').textContent=`${file.name} · 5 calibrated motors`;
    }catch(error){$('profile-status').textContent=error.message;}
    finally{loadingProfile=false;controls();}
  };
  $('serial-connect').onclick=async()=>{
    if(connecting||session?.active||disconnecting||loadingProfile)return;connecting=true;onPauseSimulation();controls();
    let link;
    try{
      // Browser device permission is requested only by this user's click.
      if(session){await session.disconnect();session=null;}
      const port=await navigator.serial.requestPort();
      state('Connecting…');link=new SerialLink(port);await link.open(Number($('serial-baud').value));
      session=new HardwareSession({link,engine:commandEngine,angles,profile,canMove:hasControlFocus,
        onFeedback:({feedback,degrees})=>{
          $('serial-raw').textContent=feedback.map(s=>`${s.id}: ${s.raw}`).join('  ·  ');
          if(degrees){engine.q=degrees.map((d,i)=>angles[i].fromDisplay(d));onFeedback();}
          $('pose-source').textContent=degrees?'LIVE · MEASURED':'SIM · UNCALIBRATED';
          $('pose-source').classList.toggle('live',!!degrees);
        },onState:text=>{state(text);controls();},onFault:error=>{
          state(`${error.message} · Connect USB to retry`);$('pose-source').textContent='FEEDBACK LOST';$('pose-source').classList.remove('live');controls();
        }});
      await session.start();
    }catch(error){
      if(link)await link.close();session=null;
      state(error.name==='NotFoundError'?'No device selected':error.message);
    }finally{connecting=false;controls();}
  };
  async function disconnect(){
    if(!session||disconnecting)return;disconnecting=true;controls();
    try{await session.disconnect();}
    finally{session=null;disconnecting=false;$('pose-source').textContent='SIMULATION';$('pose-source').classList.remove('live');$('serial-raw').textContent='1: — · 2: — · 3: — · 4: — · 5: —';state('Disconnected');controls();}
  }
  const pause=()=>{session?.pause();controls();};
  $('serial-disconnect').onclick=disconnect;
  $('serial-pause').onclick=pause;
  $('serial-enable').onclick=()=>{
    try{if(session?.controller?.enabled)pause();else{session?.enable();$('canvas').focus();state('Reading pose · enabling keys…');}}
    catch(error){state(error.message);}controls();
  };
  const keys=new Set(['w','s','a','d','arrowup','arrowdown','arrowleft','arrowright','j','l']);
  const editing=()=>document.activeElement?.matches('input,textarea,select,[contenteditable="true"]');
  window.addEventListener('keydown',event=>{
    if(!session)return;
    const key=event.key.toLowerCase();
    if(key==='escape'||event.ctrlKey||event.altKey||event.metaKey){pause();return;}
    if(key==='e'&&hasControlFocus()&&!event.repeat){event.preventDefault();$('serial-enable').click();return;}
    if(!keys.has(key)||editing()||!hasControlFocus()||!session.controller?.enabled)return;
    event.preventDefault();if(!event.repeat)session.key(key,true);
    document.querySelector(`kbd[data-key="${key}"]`)?.classList.add('pressed');
  });
  window.addEventListener('keyup',event=>{session?.key(event.key.toLowerCase(),false);});
  window.addEventListener('blur',pause);
  document.addEventListener('visibilitychange',()=>{if(document.hidden)pause();});
  document.addEventListener('focusin',()=>{if(editing())pause();});
  window.addEventListener('pagehide',()=>{session?.disconnect().catch(()=>{});});
  if(!navigator.serial)state('USB control needs desktop Chrome or Edge. Simulation is available.');
  controls();return {locked:()=>!!session?.active||connecting||disconnecting,pause};
}
