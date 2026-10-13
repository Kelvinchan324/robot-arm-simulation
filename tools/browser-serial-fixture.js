// Local-only UI fixture; never shipped in dist. No physical device access.
import {packet} from '/serial-protocol.js';
const banner=document.createElement('div');
banner.style='padding:8px;background:#552219;color:white;font:12px monospace';
banner.textContent='SIMULATED USB TEST — NO HARDWARE ';
const load=document.createElement('button');load.textContent='Load test calibration';banner.append(load);
const fault=document.createElement('button');fault.textContent='Drop servo 1 replies';banner.append(fault);
const count=document.createElement('output');count.id='test-writes';count.textContent=' Motor writes: 0';banner.append(count);
document.body.prepend(banner);
const fixture=await (await fetch('/__test__/serial-calibration.test.json')).json();
let missing=false,writes=0;
fault.onclick=()=>{missing=true;};
load.onclick=()=>{
  const transfer=new DataTransfer();transfer.items.add(new File([JSON.stringify(fixture)],'SYNTHETIC-TEST-ONLY.json',{type:'application/json'}));
  const input=document.getElementById('profile-file');input.files=transfer.files;input.dispatchEvent(new Event('change',{bubbles:true}));
};
class Port{
  async open(){
    missing=false;this.raw=fixture.test_initial_raw.slice();
    this.readable=new ReadableStream({start:controller=>this.rx=controller});
    this.writable=new WritableStream({write:bytes=>{
      if(bytes[4]===0x83){for(let k=7;k<bytes.length-1;k+=8)this.raw[bytes[k]-1]=bytes[k+2]+(bytes[k+3]<<8);count.textContent=` Motor writes: ${++writes}`;return;}
      const ids=bytes[4]===0x82?[...bytes].slice(7,-1):[bytes[2]];
      for(const id of ids){
        if(missing&&id===1)continue;
        const memory=new Uint8Array(80);memory[40]=1;memory[56]=this.raw[id-1]&255;memory[57]=this.raw[id-1]>>8;
        this.rx.enqueue(packet(id,0,[...memory.slice(bytes[5],bytes[5]+bytes[6])]));
      }
    }});
  }
  async setSignals(){}
  async close(){}
}
Object.defineProperty(navigator,'serial',{value:{requestPort:async()=>new Port()},configurable:true});
