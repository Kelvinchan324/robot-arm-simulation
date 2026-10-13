// STS3215 packets, matching the supplied scservo_sdk (little-endian, checksum).
export function packet(id, instruction, parameters=[]){
  const data=[255,255,id,parameters.length+2,instruction,...parameters];
  return Uint8Array.from([...data,(~data.slice(2).reduce((a,b)=>a+b,0))&255]);
}
export function syncTargets(targets){
  if(targets.length!==5||new Set(targets.map(t=>t.id)).size!==5)throw Error('Five unique motors required');
  const data=[41,7];
  for(const {id,raw,speed,acceleration} of targets){
    if(!Number.isInteger(id)||id<1||id>252||!Number.isInteger(raw)||raw<0||raw>4095||
       !Number.isInteger(speed)||speed<1||speed>3000||!Number.isInteger(acceleration)||acceleration<0||acceleration>254)throw Error('Invalid servo target');
    data.push(id,acceleration,raw&255,raw>>8,0,0,speed&255,speed>>8);
  }
  return packet(254,0x83,data);
}
export const signedWord=(lo,hi)=>{const n=lo+(hi<<8);return n&0x8000?-(n&0x7fff):n;};
export function servoError(id,bits){
  const labels=[[1,'voltage'],[2,'angle sensor'],[4,'overheat'],[8,'current'],[32,'overload']];
  return `Motor ${id}: ${labels.filter(([b])=>bits&b).map(([,s])=>s).join(', ')||'servo fault'} (status ${bits})`;
}
export class PacketParser{
  constructor(){this.buffer=[];}
  push(bytes){
    this.buffer.push(...bytes);const packets=[];
    while(this.buffer.length>=4){
      if(this.buffer[0]!==255||this.buffer[1]!==255){this.buffer.shift();continue;}
      const length=this.buffer[3];
      if(this.buffer[2]>252||length<2||length>80){this.buffer.shift();continue;}
      if(this.buffer.length<length+4)break;
      const p=this.buffer.splice(0,length+4);
      if(((p.slice(2).reduce((a,b)=>a+b,0))&255)!==255)throw Error(`Motor ${p[2]}: corrupt serial reply`);
      packets.push({id:p[2],error:p[4],data:p.slice(5,-1)});
    }
    if(this.buffer.length>256)throw Error('Serial receive buffer overflow');
    return packets;
  }
}
