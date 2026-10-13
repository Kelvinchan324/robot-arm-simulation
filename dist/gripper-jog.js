// Displayed opening degrees, not servo shaft degrees. Full travel takes 6 s.
export const GRIPPER_DEG_PER_SECOND=30;

export function stepGripper(engine,angles,direction,dt,rate=GRIPPER_DEG_PER_SECOND){
  if(!Number.isFinite(dt)||dt<=0||!Number.isFinite(rate)||rate<=0)throw Error('Invalid gripper rate or timestep');
  if(!direction)return false;
  const i=engine.joints.findIndex(j=>j.name==='gripper_drive');
  if(i<0)return false;
  const q=engine.q,j=engine.joints[i],angle=angles[i];
  const step=Math.sign(direction)*Math.min(rate,GRIPPER_DEG_PER_SECOND)*Math.min(dt,.08);
  const next=Math.max(j.lo,Math.min(j.hi,angle.fromDisplay(angle.toDisplay(q[i])+step)));
  if(Math.abs(next-q[i])<1e-12)return false;
  q[i]=next;engine.q=q; // Recompute pinion rotation and coupled jaw positions.
  return true;
}
