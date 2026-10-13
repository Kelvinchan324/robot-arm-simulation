import {packet,PacketParser,servoError,signedWord,syncTargets} from './serial-protocol.js';

// A single outstanding transaction. Timeout or corruption poisons the link:
// reconnect explicitly, rather than misidentifying a late reply as fresh data.
export class SerialLink{
  constructor(port,{timeout=350}={}){this.port=port;this.timeout=timeout;this.parser=new PacketParser();this.pending=null;this.failure=null;this.closed=false;}
  async open(baudRate=115200){
    try{
      await this.port.open({baudRate,dataBits:8,stopBits:1,parity:'none',flowControl:'none',bufferSize:4096});
      // Changing DTR/RTS during open can reset some ESP32 boards.
      this.writer=this.port.writable.getWriter();
      this.readTask=this.readLoop();
    }catch(error){await this.close();throw error;}
  }
  fail(error){
    if(!this.failure)this.failure=error;
    this.pending?.reject(error);this.pending=null;
  }
  async readLoop(){
    while(!this.closed&&this.port.readable){
      this.reader=this.port.readable.getReader();
      try{
        while(!this.closed){
          const {value,done}=await this.reader.read();
          if(done){if(!this.closed)this.fail(Error('Serial device disconnected'));return;}
          for(const response of this.parser.push(value)){
            const p=this.pending;if(!p)continue;
            if(!p.ids.includes(response.id)||p.responses.has(response.id))continue;
            if(response.error)throw Error(servoError(response.id,response.error));
            if(response.data.length!==p.length)throw Error(`Motor ${response.id}: incomplete feedback`);
            p.responses.set(response.id,response.data);
            if(p.responses.size===p.ids.length){this.pending=null;p.resolve(p.responses);}
          }
        }
      }catch(error){
        // Web Serial replaces the readable stream after recoverable UART
        // errors. Never recover an in-flight transaction using partial data.
        const recoverable=['BreakError','FramingError','ParityError','BufferOverrunError'].includes(error.name);
        if(!this.closed&&(!recoverable||this.pending)){this.fail(error);return;}
        this.parser=new PacketParser();
      }finally{this.reader.releaseLock();this.reader=null;}
    }
    if(!this.closed)this.fail(Error('Serial device disconnected'));
  }
  async bounded(promise){
    let timer;
    try{return await Promise.race([promise,new Promise((_,reject)=>timer=setTimeout(()=>reject(Error('Serial timeout — reconnect device')),this.timeout))]);}
    catch(error){this.fail(error);throw error;}
    finally{clearTimeout(timer);}
  }
  async exchange(bytes,ids=[],length=0){
    if(this.failure)throw this.failure;
    if(this.closed||!this.writer)throw Error('Serial device is not connected');
    if(this.busy)throw Error('Serial transaction already in progress');
    this.busy=true;
    let reply;
    try{
      if(ids.length){
        reply=new Promise((resolve,reject)=>{this.pending={ids,length,responses:new Map(),resolve,reject};});
        // The reader may reject before writer.write settles.
        reply.catch(()=>{});
      }
      await this.bounded(this.writer.write(bytes));
      return reply?await this.bounded(reply):null;
    }finally{this.pending=null;this.busy=false;}
  }
  async readRegister(id,address,length){return (await this.exchange(packet(id,2,[address,length]),[id],length)).get(id);}
  async readAll(ids,{health=false}={}){
    const states=new Map();
    if(health)for(const id of ids){
      const mode=(await this.readRegister(id,33,1))[0];
      const torque=(await this.readRegister(id,40,1))[0];
      states.set(id,{mode,torque});
    }
    const replies=await this.exchange(packet(254,0x82,[56,11,...ids]),ids,11);
    return ids.map(id=>{const d=replies.get(id);return {id,raw:signedWord(d[0],d[1]),speed:signedWord(d[2],d[3]),moving:!!d[10],...states.get(id)};});
  }
  async writeTargets(targets){await this.exchange(syncTargets(targets));}
  async close(){
    if(this.closed)return;this.closed=true;this.fail(Error('Serial connection closed'));
    try{await this.reader?.cancel();}catch{}
    await this.readTask;
    if(this.writer){try{await this.writer.abort();}catch{}try{this.writer.releaseLock();}catch{}this.writer=null;}
    try{await this.port.close();}catch{}
  }
}
