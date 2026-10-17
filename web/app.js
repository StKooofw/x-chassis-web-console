const C={PING:1,STATUS:2,POSE:3,SET_VELOCITY:0x10,ESTOP:0x13,CLEAR:0x14,SET_GOAL:0x20,ORIGIN:0x21,CANCEL:0x22,HEARTBEAT:0x23,ACK:0x7e};
const $=id=>document.getElementById(id); const enc=new TextEncoder();
let port=null,reader=null,writer=null,readTask=null,pollTimer=null,seq=0,pending=new Map(),rx=new Uint8Array(),pollBusy=false,goalId=1,manualTimer=null,points=[],selected=-1,connected=false,pathRunning=false,pathAbort=false,ioChain=Promise.resolve(),velocityPending=null,velocitySending=false;

function log(text){const el=$("log"); el.textContent+=`${new Date().toLocaleTimeString()}  ${text}\n`; el.scrollTop=el.scrollHeight}
function crc32(data){let c=0xffffffff;for(const b of data){c^=b;for(let i=0;i<8;i++)c=(c>>>1)^((c&1)?0xedb88320:0)}return c>>>0}
function concat(...parts){const out=new Uint8Array(parts.reduce((n,p)=>n+p.length,0));let o=0;for(const p of parts){out.set(p,o);o+=p.length}return out}
function makeFrame(command,payload=new Uint8Array()){const h=new Uint8Array(10),v=new DataView(h.buffer);h.set([0xa5,0x5a,1,0],0);v.setUint16(4,++seq,true);h[6]=command;v.setUint16(8,payload.length,true);const body=concat(h.slice(2),payload),out=concat(h,payload,new Uint8Array(4));new DataView(out.buffer).setUint32(out.length-4,crc32(body),true);return {seq,bytes:out}}
function parseFrames(data){rx=concat(rx,data);const frames=[];while(rx.length){let start=-1;for(let i=0;i<rx.length-1;i++)if(rx[i]===0xa5&&rx[i+1]===0x5a){start=i;break}if(start<0){rx=rx.slice(rx[rx.length-1]===0xa5?rx.length-1:rx.length);break}if(start)rx=rx.slice(start);if(rx.length<10)break;const dv=new DataView(rx.buffer,rx.byteOffset,rx.byteLength),len=dv.getUint16(8,true),total=14+len;if(len>256){rx=rx.slice(1);continue}if(rx.length<total)break;const packet=rx.slice(0,total);rx=rx.slice(total);const want=new DataView(packet.buffer).getUint32(total-4,true);if(crc32(packet.slice(2,total-4))!==want)continue;frames.push({seq:new DataView(packet.buffer).getUint16(4,true),flags:packet[3],command:packet[6],payload:packet.slice(10,10+len)})}return frames}
async function readLoop(){try{while(reader){const {value,done}=await reader.read();if(done)break;for(const f of parseFrames(value))dispatch(f)}}catch(e){if(connected)log(`串口读取中断：${e.message}`)}}
function dispatch(f){const p=pending.get(f.seq);if(!p)return;pending.delete(f.seq);if(f.command!==C.ACK||f.payload[0]!==p.command||f.payload[1])p.reject(new Error(`命令 0x${p.command.toString(16)} 错误 ${f.payload[1]??"?"}`));else p.resolve(f.payload.slice(2))}
async function request(command,payload=new Uint8Array(),timeout=900){
  const run=async()=>{if(!writer)throw Error("尚未连接串口");const f=makeFrame(command,payload);let last;
    for(let attempt=0;attempt<3;attempt++){
      try{
        let timer;
        const promise=new Promise((resolve,reject)=>{timer=setTimeout(()=>{pending.delete(f.seq);reject(new Error(`命令 0x${command.toString(16)} 超时`))},attempt?Math.min(350,timeout):timeout);pending.set(f.seq,{command,resolve:v=>{clearTimeout(timer);resolve(v)},reject:e=>{clearTimeout(timer);reject(e)}})});
        try{await writer.write(f.bytes)}catch(e){clearTimeout(timer);pending.delete(f.seq);throw e}
        return await promise;
      }catch(e){last=e;if(attempt<2)await new Promise(r=>setTimeout(r,80))}
    }
    throw last;
  };
  const next=ioChain.then(run,run);ioChain=next.catch(()=>{});return next;
}
function i32(v){const b=new Uint8Array(4);new DataView(b.buffer).setInt32(0,v,true);return b}function u32(v){const b=new Uint8Array(4);new DataView(b.buffer).setUint32(0,v,true);return b}function f32(v){const b=new Uint8Array(4);new DataView(b.buffer).setFloat32(0,v,true);return b}
function velocity(vx,vy,wz,enable=true,hold=false,streaming=true){return concat(i32(vx),i32(vy),i32(wz),new Uint8Array([enable?(1|(hold?2:0)|(streaming?4:0)):0]))}
function goal(id,x,y,yaw,timeout=30000,relative=true){return concat(u32(id),f32(x),f32(y),f32(yaw),u32(timeout),new Uint8Array([relative?1:0]))}
function origin(){return concat(f32(0),f32(0),f32(0))}
function decodeStatus(p){const d=new DataView(p.buffer,p.byteOffset,p.byteLength);return{state:p[0],online:!!p[1],estop:!!p[2],yawHold:!!p[3],yaw:d.getFloat32(8,true),setpoint:d.getFloat32(12,true),vx:d.getFloat32(16,true),vy:d.getFloat32(20,true),wz:d.getFloat32(24,true),faults:d.getUint32(28,true),wheels:[0,1,2,3].map(i=>d.getInt32(32+i*4,true))}}
function decodePose(p){const d=new DataView(p.buffer,p.byteOffset,p.byteLength);return{nav:p[0],imu:!!p[1],odom:!!p[2],id:d.getUint32(4,true),x:d.getFloat32(8,true),y:d.getFloat32(12,true),yaw:d.getFloat32(16,true),tx:d.getFloat32(20,true),ty:d.getFloat32(24,true),tyaw:d.getFloat32(28,true),vx:d.getFloat32(32,true),vy:d.getFloat32(36,true),wz:d.getFloat32(40,true),version:d.getUint32(60,true)}}
async function retryRequest(command,payload=new Uint8Array(),attempts=1,timeout=500){let last;for(let i=0;i<attempts;i++){try{return await request(command,payload,timeout)}catch(e){last=e;if(i+1<attempts)await new Promise(r=>setTimeout(r,80))}}throw last}
async function sendGoalReliable(id,q){
  const payload=goal(id,q.x,q.y,q.yaw,40000,false);
  try{return await request(C.SET_GOAL,payload,1200)}
  catch(first){
    let pose;
    try{pose=decodePose(await retryRequest(C.POSE))}catch{return Promise.reject(first)}
    if(pose.id===id)return new Uint8Array();
    try{return await request(C.SET_GOAL,payload,1200)}
    catch(second){
      try{pose=decodePose(await retryRequest(C.POSE));if(pose.id===id)return new Uint8Array()}catch{}
      throw second;
    }
  }
}
function streamVelocity(payload){
  velocityPending=payload;
  if(velocitySending||!writer)return;
  velocitySending=true;
  (async()=>{
    while(velocityPending&&writer){
      const next=velocityPending;velocityPending=null;
      try{await writer.write(makeFrame(C.SET_VELOCITY,next).bytes)}
      catch(e){if(connected)$("message").textContent=e.message;break}
    }
    velocitySending=false;
  })();
}
function setBadges(s,p){$("linkBadge").textContent="串口 在线";$("driveBadge").textContent=`CM4M ${s.online?"在线":"离线"}`;$("imuBadge").textContent=`IMU ${p.imu?"就绪":"校准中"}`;$("estopBadge").textContent=`急停 ${s.estop?"锁存":"已解除"}`;$("versionBadge").textContent=`运动学 v${p.version}`;$("estop").disabled=false;$("rtt").textContent="实时"}
function updateTelemetry(s,p){setBadges(s,p);$("poseX").textContent=p.x.toFixed(3);$("poseY").textContent=p.y.toFixed(3);$("poseYaw").textContent=(p.yaw*180/Math.PI).toFixed(1);[s.wheels[0],s.wheels[1],s.wheels[2],s.wheels[3]].forEach((v,i)=>$("wheels").children[i].innerHTML=`${"ABCD"[i]}<br>${(v/1000).toFixed(1)} rpm`)}
async function poll(){if(!connected||pollBusy||pathRunning)return;pollBusy=true;try{const s=decodeStatus(await retryRequest(C.STATUS)),p=decodePose(await retryRequest(C.POSE));updateTelemetry(s,p)}catch(e){$("message").textContent=e.message}finally{pollBusy=false}}
async function connect(){if(!navigator.serial){$("message").textContent="当前浏览器不支持 Web Serial";return}try{port=await navigator.serial.requestPort();await port.open({baudRate:115200,dataBits:8,stopBits:1,parity:"none",flowControl:"none"});writer=port.writable.getWriter();reader=port.readable.getReader();connected=true;readTask=readLoop();$("connect").textContent="断开串口";$("estop").disabled=false;log("串口已连接，自动解除急停");await request(C.CLEAR);await poll();pollTimer=setInterval(poll,250)}catch(e){$("message").textContent=e.message;await disconnect()}}
async function disconnect(){connected=false;velocityPending=null;if(pollTimer){clearInterval(pollTimer);pollTimer=null}for(const p of pending.values())p.reject(Error("串口已断开"));pending.clear();try{await reader?.cancel()}catch{}try{reader?.releaseLock()}catch{}try{writer?.releaseLock()}catch{}try{await port?.close()}catch{}reader=null;writer=null;port=null;$("connect").textContent="连接串口";$("estop").disabled=true;$("linkBadge").textContent="串口 未连接";log("串口已断开")}
async function estop(){if(!writer)return;pathAbort=true;velocityPending=null;try{while(velocitySending)await new Promise(r=>setTimeout(r,5));await request(C.ESTOP);log("已发送急停")}catch(e){log(e.message)}}
async function stop(){pathAbort=true;velocityPending=null;try{while(velocitySending)await new Promise(r=>setTimeout(r,5));await request(C.CANCEL);await request(C.SET_VELOCITY,velocity(0,0,0,false,false,false));log("已停止当前动作")}catch(e){log(e.message)}}
function manual(vx,vy,wz,hold){streamVelocity(velocity(vx,vy,wz,true,hold,true))}
function beginManual(vx,vy,wz){if(manualTimer)clearInterval(manualTimer);const speed=Number($("speed").value),scale=vx&&vy?Math.SQRT1_2:1,hold=!wz&&$("yawHold").checked;manual(vx*speed/scale,vy*speed/scale,wz*Number($("turn").value)*Math.PI/180*1000,hold);manualTimer=setInterval(()=>manual(vx*speed/scale,vy*speed/scale,wz*Number($("turn").value)*Math.PI/180*1000,hold),100)}
function endManual(){if(manualTimer){clearInterval(manualTimer);manualTimer=null;stop()}}
async function sendGoal(){const x=Number($("goalX").value)/1000,y=Number($("goalY").value)/1000,yaw=Number($("goalYaw").value)*Math.PI/180;try{await request(C.SET_GOAL,goal(goalId++,x,y,yaw,30000,$("relative").checked));log(`目标已发送 x=${x.toFixed(2)} y=${y.toFixed(2)} yaw=${(yaw*180/Math.PI).toFixed(1)}°`)}catch(e){log(e.message)}}
function mapPoint(x,y){const c=$("map"),m=38;return{x:m+x/8*(c.width-2*m),y:c.height-m-(y+2.5)/5*(c.height-2*m)}}function worldPoint(x,y){const c=$("map"),m=38;return{x:Math.max(0,Math.min(8,(x-m)/(c.width-2*m)*8)),y:Math.max(-2.5,Math.min(2.5,(c.height-m-y)/(c.height-2*m)*5-2.5))}}
function drawMap(){const c=$("map"),p=c.getContext("2d");p.fillStyle="#091827";p.fillRect(0,0,c.width,c.height);const m=38;p.strokeStyle="#263b55";for(let x=0;x<=8;x++){const a=mapPoint(x,-2.5),b=mapPoint(x,2.5);p.beginPath();p.moveTo(a.x,a.y);p.lineTo(b.x,b.y);p.stroke()}for(let y=-2;y<=2;y++){const a=mapPoint(0,y),b=mapPoint(8,y);p.beginPath();p.moveTo(a.x,a.y);p.lineTo(b.x,b.y);p.stroke()}p.strokeStyle="#5f7896";p.strokeRect(m,m,c.width-2*m,c.height-2*m);if(points.length>1){p.strokeStyle="#65d8bd";p.lineWidth=3;p.beginPath();points.forEach((q,i)=>{const z=mapPoint(q.x,q.y);i?p.lineTo(z.x,z.y):p.moveTo(z.x,z.y)});p.stroke();p.lineWidth=1}points.forEach((q,i)=>{const z=mapPoint(q.x,q.y);p.fillStyle=i===selected?"#f2b86b":"#68aaff";p.beginPath();p.arc(z.x,z.y,7,0,Math.PI*2);p.fill();p.strokeStyle="#fff";p.beginPath();p.moveTo(z.x,z.y);p.lineTo(z.x+Math.cos(q.yaw)*22,z.y-Math.sin(q.yaw)*22);p.stroke();p.fillStyle="#dbe9f8";p.fillText(String(i+1),z.x+9,z.y-9)})}
function renderPoints(){const body=$("waypoints");body.innerHTML="";points.forEach((q,i)=>{const tr=document.createElement("tr");tr.innerHTML=`<td>${i+1}</td><td><input data-i="${i}" data-k="x" type="number" step="0.01" value="${q.x.toFixed(3)}"></td><td><input data-i="${i}" data-k="y" type="number" step="0.01" value="${q.y.toFixed(3)}"></td><td><input data-i="${i}" data-k="yaw" type="number" step="1" value="${(q.yaw*180/Math.PI).toFixed(1)}"></td><td><button data-del="${i}">删除</button></td>`;body.appendChild(tr)});$("pointCount").textContent=`${points.length} points`;drawMap()}
async function runPath(){if(pathRunning||!points.length){if(!points.length)log("请先添加航点");return}pathRunning=true;pathAbort=false;$("runPath").disabled=true;try{await retryRequest(C.ORIGIN,origin());await retryRequest(C.CLEAR);for(const q of points){if(pathAbort)throw Error("路径已停止");const id=goalId++;const pointIndex=points.indexOf(q)+1;const lastPoint=pointIndex===points.length;await sendGoalReliable(id,q);log(`执行航点 ${pointIndex}/${points.length}`);let done=false,lastPoseGood=performance.now(),poseTimeouts=0;for(let i=0;i<200;i++){if(pathAbort)throw Error("路径已停止");let p;try{p=decodePose(await retryRequest(C.POSE,new Uint8Array(),1,300));lastPoseGood=performance.now()}catch(e){poseTimeouts++;log(`航点 ${pointIndex} 查询位姿暂时超时（${poseTimeouts}），继续等待恢复`);if(performance.now()-lastPoseGood>8000)throw e;await new Promise(r=>setTimeout(r,120));continue}if(p.id===id&&p.nav===4)throw Error("下位机中止当前航点");const yawError=Math.atan2(Math.sin(q.yaw-p.yaw),Math.cos(q.yaw-p.yaw));const near=Math.hypot(q.x-p.x,q.y-p.y)<=.10&&Math.abs(yawError)<=.12;const moving=Math.hypot(p.vx,p.vy)>.025;if(p.id===id&&p.nav===3){done=true;break}if(!lastPoint&&p.id===id&&near&&moving){done=true;log(`航点 ${pointIndex} 前视切换`);break}await new Promise(r=>setTimeout(r,120))}if(!done)throw Error("航点执行超时")}log("自定义路径完成")}catch(e){log(e.message);try{await request(C.CANCEL);await request(C.SET_VELOCITY,velocity(0,0,0,false,false,false));log("路径异常，已取消当前目标并清零速度")}catch(stopError){log(`路径停止回包失败：${stopError.message}`)}}finally{pathRunning=false;$("runPath").disabled=false;poll()}}
$("connect").onclick=()=>connected?disconnect():connect();$("estop").onclick=estop;$("stop").onclick=stop;$("sendGoal").onclick=sendGoal;$("runPath").onclick=runPath;$("clearLog").onclick=()=>$("log").textContent="";$("clearPath").onclick=()=>{points=[];selected=-1;renderPoints()};$("map").onclick=e=>{const r=e.target.getBoundingClientRect(),q=worldPoint((e.clientX-r.left)*e.target.width/r.width,(e.clientY-r.top)*e.target.height/r.height),last=points.at(-1);points.push({x:q.x,y:q.y,yaw:last?Math.atan2(q.y-last.y,q.x-last.x):0});selected=points.length-1;renderPoints()};$("waypoints").onchange=e=>{const el=e.target;if(!el.dataset.i)return;const i=Number(el.dataset.i),k=el.dataset.k;points[i][k]=k==="yaw"?Number(el.value)*Math.PI/180:Number(el.value);selected=i;drawMap()};$("waypoints").onclick=e=>{if(e.target.dataset.del){points.splice(Number(e.target.dataset.del),1);selected=-1;renderPoints()}};$("savePath").onclick=()=>{const a=document.createElement("a");a.href=URL.createObjectURL(new Blob([JSON.stringify(points,null,2)],{type:"application/json"}));a.download="custom_route.json";a.click();URL.revokeObjectURL(a.href)};$("loadPath").onclick=()=>$("filePath").click();$("filePath").onchange=async e=>{try{points=JSON.parse(await e.target.files[0].text());selected=-1;renderPoints()}catch{$("message").textContent="路径 JSON 无效"}};document.querySelectorAll("[data-v]").forEach(b=>{b.onpointerdown=()=>{const [x,y]=b.dataset.v.split(",").map(Number);beginManual(x,y,0)};b.onpointerup=endManual;b.onpointerleave=endManual});document.querySelectorAll("[data-w]").forEach(b=>{b.onpointerdown=()=>beginManual(0,0,Number(b.dataset.w));b.onpointerup=endManual;b.onpointerleave=endManual});window.addEventListener("resize",drawMap);drawMap();
