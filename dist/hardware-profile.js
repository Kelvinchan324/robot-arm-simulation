export const MOTOR_NAMES=['base_yaw','shoulder','elbow','wrist_pitch','gripper_drive'];
const pair=(p)=>Array.isArray(p)&&p.length===2&&p.every(Number.isFinite);
export function validateProfile(profile,angles){
  if(profile.version!==1||profile.servo_family!=='sms_sts'||profile.calibration_confirmed!==true||profile.position_mode_confirmed!==true)throw Error('Load a confirmed STS3215 calibration exported from config.py');
  const maps=MOTOR_NAMES.map((name,i)=>{
    const c=profile.servos?.[name];
    if(!c||c.id!==i+1)throw Error(`${name}: expected motor ID ${i+1}`);
    for(const k of ['joint_degrees','raw_positions','raw_limits','safe_degrees'])if(!pair(c[k]))throw Error(`${name}: fill ${k}`);
    const [a,b]=c.joint_degrees,[x,y]=c.raw_positions,[lo,hi]=c.safe_degrees,[rlo,rhi]=c.raw_limits;
    if(!(a<b&&x!==y&&rlo>=0&&rlo<rhi&&rhi<=4095&&Number.isInteger(rlo)&&Number.isInteger(rhi)&&a<=lo&&lo<hi&&hi<=b&&lo>=angles[i].lo-1e-6&&hi<=angles[i].hi+1e-6&&Math.min(x,y)>=rlo&&Math.max(x,y)<=rhi))throw Error(`${name}: invalid calibration or travel limits`);
    if(!Number.isFinite(c.max_deg_s)||c.max_deg_s<=0||c.max_deg_s>90||!Number.isInteger(c.speed)||c.speed<1||c.speed>3000||!Number.isInteger(c.acceleration)||c.acceleration<0||c.acceleration>254)throw Error(`${name}: invalid speed / acceleration`);
    const scale=(y-x)/(b-a),rawAt=degrees=>x+(degrees-a)*scale;
    return {...c,name,toRaw(degrees){
      if(!Number.isFinite(degrees)||degrees<lo-1e-7||degrees>hi+1e-7)throw Error(`${name}: target outside safe range`);
      const raw=Math.round(rawAt(degrees));if(raw<rlo||raw>rhi)throw Error(`${name}: raw target outside safe range`);return raw;
    },fromRaw(raw){
      const degrees=a+(raw-x)/scale,epsilon=.51/Math.abs(scale);
      if(!Number.isInteger(raw)||raw<rlo||raw>rhi||degrees<lo-epsilon||degrees>hi+epsilon)throw Error(`${name}: measured position outside calibrated range`);
      return Math.max(lo,Math.min(hi,degrees));
    }};
  });
  const speedScale=profile.speed_scale??.25,trackingError=profile.max_tracking_error_deg??8;
  if(!Number.isFinite(speedScale)||speedScale<=0||speedScale>1||!Number.isFinite(trackingError)||trackingError<=0||trackingError>30)throw Error('Invalid jogging speed or tracking threshold');
  return {maps,speedScale,trackingError};
}
