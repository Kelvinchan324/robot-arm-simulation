// Teaching/control coordinates. MuJoCo still uses the exported CAD radians.
const degrees=180/Math.PI;
export function calibrateJointAngles(joints){
  return joints.map(j=>{
    let gain=j.slide?1:degrees,offset=0;
    if(j.name==='base_yaw'){
      offset=-18.8;
      // This CAD joint is continuous: centre its full-turn teaching range.
      if(!j.limited){const centre=18.8/degrees;j.lo=centre-Math.PI;j.hi=centre+Math.PI;}
    }
    if(j.name==='shoulder')offset=90-j.hi*degrees;
    if(j.name==='elbow')offset=-90-j.lo*degrees;
    if(j.name==='wrist_pitch')offset=90-j.hi*degrees;
    if(j.name==='gripper_drive'){gain=-180/(j.hi-j.lo);offset=90-gain*j.lo;}
    const toDisplay=q=>gain*q+offset,fromDisplay=value=>(value-offset)/gain;
    const endpoints=[toDisplay(j.lo),toDisplay(j.hi)].sort((a,b)=>a-b);
    return {toDisplay,fromDisplay,lo:endpoints[0],hi:endpoints[1],gain,offset};
  });
}
