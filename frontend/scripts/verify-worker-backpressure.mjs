import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import ts from 'typescript';

const source=fs.readFileSync(new URL('../src/workers/telemetryWorker.ts',import.meta.url),'utf8');
const code=ts.transpileModule(source,{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022}}).outputText;
const posted=[],sockets=[];
class Socket {
  static OPEN=1;
  readyState=1;
  constructor(){sockets.push(this);}
  close(){this.onclose?.();}
  send(message){this.sent=message;}
}
const self={postMessage(message){posted.push(message);}};
vm.runInNewContext(code,{self,WebSocket:Socket,exports:{},console,
  performance:{timeOrigin:0,now:()=>1},Date,setTimeout:()=>1,clearTimeout:()=>{}});
const command=data=>self.onmessage({data});
const frame=tick=>({type:'state',protocol_version:'3.0',coordinate_frame:'dds_world_v1',
  simulation_id:'live',epoch:1,tick,route_version:1,simulation_time_s:tick/10,
  channels:{semantic:{},heavy:{},pose:{ego:{velocity:1,acceleration:0,pose:{x:0,z:0}}}}});
command({type:'CONNECT',url:'ws://local'});
const first=sockets.at(-1);
const receive=(socket,data)=>socket.onmessage({data:JSON.stringify(data)});
for(let tick=1;tick<=1000;tick++)receive(first,frame(tick));
let states=posted.filter(x=>x.type==='STATE_DELTA');
assert.equal(states.length,1,'busy main thread receives at most one state');
assert.equal(posted.filter(x=>x.type==='STATUS').length,2,'status only changes on transitions');
receive(first,{type:'route',waypoints:[[1,2],[3,4]]});
receive(first,{type:'command_result',command_id:'request',status:'applied'});
receive(first,{type:'event',event:{name:'safety'}});
assert.deepEqual(posted.slice(-3).map(x=>x.type),['ROUTE','COMMAND_RESULT','EVENT']);
command({type:'STATE_APPLIED',deliveryId:states[0].deliveryId});
states=posted.filter(x=>x.type==='STATE_DELTA');
assert.equal(states.length,2);
assert.equal(states[1].payload.tick,1000,'release forwards latest, not queued historical states');
receive(first,frame(1001));
command({type:'CONNECT',url:'ws://local'});
receive(sockets.at(-1),frame(1002));
const current=posted.filter(x=>x.type==='STATE_DELTA').at(-1);
receive(sockets.at(-1),frame(1003));
const before=posted.length;
command({type:'STATE_APPLIED',deliveryId:states[1].deliveryId});
assert.equal(posted.length,before,'old connection acknowledgement cannot release new delivery');
command({type:'STATE_APPLIED',deliveryId:current.deliveryId});
assert.equal(posted.at(-1).payload.tick,1003);
command({type:'DISCONNECT'});
command({type:'STATE_APPLIED',deliveryId:posted.at(-1).deliveryId});
console.log('Worker handoff bounded; latest state, reliable events and reconnect acknowledgements verified.');
