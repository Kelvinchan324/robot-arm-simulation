// Cartesian jogging for this four-axis arm. Yaw and jaw opening stay fixed
// while the three parallel arm joints solve radial position, height and pitch.
const wrap=a=>Math.atan2(Math.sin(a),Math.cos(a));
export class CartesianJog {
  constructor(engine){this.engine=engine;this.arm=['shoulder','elbow','wrist_pitch'].map(n=>engine.joints.findIndex(j=>j.name===n));this.yaw=engine.joints.findIndex(j=>j.name==='base_yaw');}
  forward(){const r=this.engine.pose().r;const length=Math.hypot(r[0],r[3]);return [-r[3]/length,r[0]/length,0];}
  pitch(){const r=this.engine.pose().r,f=this.forward();return Math.atan2(r[8],r[2]*f[0]+r[5]*f[1]);}
  solve(position,pitch){
    const e=this.engine,initial=e.q,f=this.forward(),weight=.12;
    if(![...position,pitch].every(Number.isFinite))return false;
    const sample=()=>{const {p}=e.pose();return [p[0]*f[0]+p[1]*f[1],p[2],this.pitch()];};
    const goal=[position[0]*f[0]+position[1]*f[1],position[2],pitch];
    for(let k=0;k<100;k++){
      const q=e.q,s=sample(),error=[goal[0]-s[0],goal[1]-s[1],wrap(goal[2]-s[2])*weight];
      if(Math.hypot(...e.pose().p.map((v,i)=>v-position[i]))<.00002&&Math.abs(wrap(pitch-this.pitch()))<.0002)return true;
      const columns=this.arm.map(i=>{const h=q[i]+1e-5<=e.joints[i].hi?1e-5:-1e-5,t=q.slice();t[i]+=h;e.q=t;const sp=sample();return [(sp[0]-s[0])/h,(sp[1]-s[1])/h,wrap(sp[2]-s[2])*weight/h];});e.q=q;
      const A=Array.from({length:3},(_,a)=>Array.from({length:3},(_,b)=>columns.reduce((sum,c)=>sum+c[a]*c[b],0)+(a===b?1e-6:0)));
      const v=solve3(A,error),next=q.slice();this.arm.forEach((i,j)=>next[i]+=Math.max(-.12,Math.min(.12,columns[j].reduce((sum,x,a)=>sum+x*v[a],0))));e.q=next;
    }
    e.q=initial;return false; // Never leave a failed jog in a partial pose.
  }
  step({height=0,reach=0,yaw=0,pitch=0}={}){
    const e=this.engine,initial=e.q,heldPitch=this.pitch();
    if(yaw){const q=e.q,n=q[this.yaw]+yaw,j=e.joints[this.yaw];if(n<j.lo||n>j.hi)return false;q[this.yaw]=n;e.q=q;}
    const p=e.pose().p,f=this.forward();p[0]+=f[0]*reach;p[1]+=f[1]*reach;p[2]+=height;
    if(!height&&!reach&&!pitch)return true;
    if(this.solve(p,heldPitch+pitch))return true;e.q=initial;return false;
  }
  level(){return this.solve(this.engine.pose().p,0);}
  levelWrist(){
    const e=this.engine,q=e.q,i=this.arm[2],joint=e.joints[i];
    const choices=[-1,0,1].map(k=>q[i]-this.pitch()+k*2*Math.PI).filter(v=>v>=joint.lo&&v<=joint.hi).sort((a,b)=>Math.abs(a-q[i])-Math.abs(b-q[i]));
    if(!choices.length)return false;q[i]=choices[0];e.q=q;return true;
  }
}
function solve3(A,b){const m=A.map((r,i)=>[...r,b[i]]);for(let i=0;i<3;i++){let p=i;for(let j=i+1;j<3;j++)if(Math.abs(m[j][i])>Math.abs(m[p][i]))p=j;[m[i],m[p]]=[m[p],m[i]];const d=m[i][i];if(Math.abs(d)<1e-15)return [0,0,0];for(let k=i;k<4;k++)m[i][k]/=d;for(let j=0;j<3;j++)if(j!==i){const t=m[j][i];for(let k=i;k<4;k++)m[j][k]-=t*m[i][k];}}return m.map(r=>r[3]);}
