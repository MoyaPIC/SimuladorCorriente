'use strict';
const $=id=>document.getElementById(id);
const TYPES={
  temperature:{name:'Temperatura',unit:'°C',min:0,max:100},
  pressure:{name:'Presión',unit:'bar',min:0,max:10},
  flow:{name:'Caudal',unit:'L/min',min:0,max:100},
  angle:{name:'Posición angular',unit:'°',min:0,max:360},
  current:{name:'Corriente',unit:'A',min:0,max:100},
  voltage:{name:'Tensión',unit:'V',min:0,max:400},
  custom:{name:'Personalizada',unit:'u',min:0,max:100}
};
const SPP_UUID="00001101-0000-1000-8000-00805f9b34fb";
let serialReadBuffer="";
const clone=x=>JSON.parse(JSON.stringify(x));
const makeScale=(type='current')=>({...clone(TYPES[type]),type,currentMin:4,currentMax:20,measuredLow:4,measuredHigh:20});
const state={
  outScale:makeScale('current'), inScale:makeScale('current'), outputMa:12, inputMa:null, outputOpen:false,
  port:null,reader:null,writer:null,connected:false,simulation:false,simTimer:null,simPhase:0,
  samples:[],chart:[],profiles:[],points:[{t:0,v:0},{t:10,v:100}], rampTimer:null,rampPaused:false,rampState:null,
  installPrompt:null,rampTrace:[]
};
function clamp(v,a,b){return Math.max(a,Math.min(b,v));}
function engToMa(v,s){const lo=Math.min(s.min,s.max),hi=Math.max(s.min,s.max),cv=clamp(Number(v),lo,hi);const span=s.max-s.min||1;const ma=s.currentMin+(cv-s.min)*(s.currentMax-s.currentMin)/span;return clamp(ma,4,20);}
function maToEng(ma,s){const span=s.currentMax-s.currentMin||1;return s.min+(clamp(Number(ma),4,20)-s.currentMin)*(s.max-s.min)/span;}
function fmt(v,n=1){return Number.isFinite(Number(v))?Number(v).toFixed(n):'—';}
function log(msg,dir='•'){const t=$('terminal');t.textContent+=`${new Date().toLocaleTimeString()} ${dir} ${msg}\n`;t.scrollTop=t.scrollHeight;}
function saveLocal(){localStorage.setItem('simcorr_profiles_v12',JSON.stringify(state.profiles));localStorage.setItem('simcorr_outscale_v12',JSON.stringify(state.outScale));localStorage.setItem('simcorr_inscale_v12',JSON.stringify(state.inScale));}
function loadLocal(){try{state.profiles=JSON.parse(localStorage.getItem('simcorr_profiles_v12')||'[]');state.outScale={...makeScale(),...JSON.parse(localStorage.getItem('simcorr_outscale_v12')||'{}')};state.inScale={...makeScale(),...JSON.parse(localStorage.getItem('simcorr_inscale_v12')||'{}')};}catch(e){console.warn(e);}}
function setPill(text,ok=false){$('livePill').textContent=text;$('livePill').className='pill '+(ok?'ok':'muted');}
function updateSummary(){$('scaleSummary').innerHTML=`<b>Salida:</b> ${state.outScale.name}: ${fmt(state.outScale.min,2)} a ${fmt(state.outScale.max,2)} ${state.outScale.unit} ↔ ${fmt(state.outScale.currentMin,1)}–${fmt(state.outScale.currentMax,1)} mA<br><b>Entrada:</b> ${state.inScale.name}: ${fmt(state.inScale.min,2)} a ${fmt(state.inScale.max,2)} ${state.inScale.unit} ↔ ${fmt(state.inScale.currentMin,1)}–${fmt(state.inScale.currentMax,1)} mA`;}
function renderMain(){
  $('outMa').textContent=fmt(state.outputMa,1);$('outEngName').textContent=state.outScale.name;$('outUnit').textContent=state.outScale.unit;$('outEng').textContent=fmt(maToEng(state.outputMa,state.outScale),2);
  $('manualSlider').value=clamp(state.outputMa,4,20);$('manualCurrent').value=fmt(clamp(state.outputMa,4,20),1);
  $('inEngName').textContent=state.inScale.name;$('inUnit').textContent=state.inScale.unit;
  if(state.inputMa==null){$('inMa').textContent='—';$('inEng').textContent='—';}else{$('inMa').textContent=fmt(state.inputMa,3);$('inEng').textContent=fmt(maToEng(state.inputMa,state.inScale),2);}
  const vals=state.samples.map(x=>x.ma);$('statCount').textContent=vals.length;if(vals.length){$('statMin').textContent=fmt(Math.min(...vals),3)+' mA';$('statMax').textContent=fmt(Math.max(...vals),3)+' mA';$('statAvg').textContent=fmt(vals.reduce((a,b)=>a+b,0)/vals.length,3)+' mA';}else{$('statMin').textContent=$('statMax').textContent=$('statAvg').textContent='—';}
  updateSummary();updateScaleCards();drawChart();drawRampChart();
}
async function send(line){
  log(line,'TX');
  if(state.simulation){handleLine(simulateCommand(line));return true;}
  if(!state.writer){log('Puerto no conectado','ERR');return false;}
  try{await state.writer.write(new TextEncoder().encode(line+'\n'));return true;}catch(e){log(e.message,'ERR');return false;}
}
function simulateCommand(line){
  if(line.startsWith('GET:STATUS'))return `DATA:OUT=${fmt(state.outputMa,3)}:IN=${fmt(state.inputMa??state.outputMa,3)}`;
  if(line.startsWith('SET:CURRENT:')){const v=Number(line.split(':')[2]);if(Number.isFinite(v))state.outputMa=clamp(v,4,20);return 'OK';}
  if(line.startsWith('SET:OUTPUT:OPEN')){state.outputOpen=true;return 'OK';}
  if(line.startsWith('SET:OUTPUT:ON')){state.outputOpen=false;return 'OK';}
  if(line.startsWith('PROFILE:'))return 'OK';
  if(line.startsWith('CAL:'))return 'OK';
  return 'OK';
}
function handleLine(line){if(!line)return;log(line,'RX');if(line.startsWith('DATA:')){const mo=line.match(/OUT=([-\d.]+)/),mi=line.match(/IN=([-\d.]+)/);if(mo)state.outputMa=Number(mo[1]);if(mi)addInput(Number(mi[1]));renderMain();}}
function setSerialDiag(msg){const el=$('serialDiag');if(el)el.textContent='Diagnóstico: '+msg;}
async function connectSerial(){
  if($('transportMode').value==='simulation'){startSimulation();return;}
  if(state.connected){await disconnectSerial();return;}
  if(!('serial' in navigator)){
    alert('Web Serial no está disponible. Usá Chrome actualizado en Android o PC.');
    setSerialDiag('Web Serial no disponible en este navegador.');
    return;
  }
  stopSimulation();
  serialReadBuffer='';
  $('connectBtn').disabled=true;
  setSerialDiag('abriendo selector Bluetooth SPP…');
  try{
    try{
      state.port=await navigator.serial.requestPort({allowedBluetoothServiceClassIds:[SPP_UUID]});
    }catch(e){
      if(e.name==='TypeError'){
        log('El navegador no acepta allowedBluetoothServiceClassIds; se abre selector sin filtro.','INFO');
        state.port=await navigator.serial.requestPort();
      }else{
        throw e;
      }
    }

    await state.port.open({baudRate:Number($('baudRate').value)||9600});

    const decoder=new TextDecoderStream();
    state.port.readable.pipeTo(decoder.writable).catch(()=>{});
    state.reader=decoder.readable.getReader();
    state.writer=state.port.writable.getWriter();

    state.simulation=false;
    state.connected=true;
    $('connectBtn').disabled=true;
    $('disconnectBtn').disabled=false;
    $('supportPill').textContent='HC-05 conectado';
    $('supportPill').className='pill ok';
    $('modeStatus').textContent='HC-05 conectado mediante Bluetooth SPP.';
    setSerialDiag('conectado · SPP 0x1101 · '+(Number($('baudRate').value)||9600)+' baud');
    setPill('En línea',true);

    readLoop();
    await send('GET:STATUS');
  }catch(e){
    log(e.message||String(e),'ERR');
    state.port=null;
    state.reader=null;
    state.writer=null;
    state.connected=false;
    $('connectBtn').disabled=false;
    $('disconnectBtn').disabled=true;

    if(e.name==='NotFoundError'){
      setSerialDiag('selección cancelada o HC-05 no disponible en el selector.');
      $('modeStatus').textContent='No se seleccionó el HC-05.';
    }else{
      setSerialDiag((e.name||'Error')+': '+(e.message||String(e)));
      $('modeStatus').textContent='No se pudo conectar: '+(e.message||String(e));
      alert('No se pudo conectar: '+(e.message||String(e)));
    }
  }
}
async function disconnectSerial(){
  stopSimulation();
  state.connected=false;
  try{if(state.reader){await state.reader.cancel();state.reader.releaseLock();}}catch(e){}
  try{if(state.writer){state.writer.releaseLock();}}catch(e){}
  try{if(state.port)await state.port.close();}catch(e){}
  state.reader=state.writer=state.port=null;
  serialReadBuffer='';
  $('connectBtn').disabled=false;
  $('disconnectBtn').disabled=true;
  $('supportPill').textContent='Web Serial';
  $('supportPill').className='pill';
  $('modeStatus').textContent='Desconectado.';
  setSerialDiag('sin conexión.');
  setPill('Sin datos');
}
async function readLoop(){
  try{
    while(state.reader){
      const {value,done}=await state.reader.read();
      if(done)break;
      serialReadBuffer+=value;
      let p;
      while((p=serialReadBuffer.indexOf('\n'))>=0){
        const line=serialReadBuffer.slice(0,p).trim();
        serialReadBuffer=serialReadBuffer.slice(p+1);
        handleLine(line);
      }
    }
  }catch(e){
    log(e.message||String(e),'ERR');
  }finally{
    if(state.connected&&!state.simulation)disconnectSerial();
  }
}
function addInput(ma){if(!Number.isFinite(ma))return;state.inputMa=ma;state.samples.push({t:Date.now(),ma});if(state.samples.length>500)state.samples.shift();state.chart.push({t:Date.now(),out:state.outputMa,inp:ma});if(state.chart.length>300)state.chart.shift();}
function startSimulation(){stopSimulation();state.simulation=true;state.connected=true;$('transportMode').value='simulation';$('connectBtn').disabled=true;$('disconnectBtn').disabled=false;$('supportPill').textContent='Simulación activa';$('supportPill').className='pill ok';setPill('Simulando',true);$('modeStatus').textContent='Simulación activa: la entrada sigue la salida con el desvío configurado.';const tick=()=>{state.simPhase+=0.35;const noise=Number($('simNoise').value)||0;let ma=state.outputOpen?4:clamp(state.outputMa+Math.sin(state.simPhase)*noise,4,20);addInput(ma);renderMain();};tick();state.simTimer=setInterval(tick,Math.max(100,Number($('simInterval').value)||500));}
function stopSimulation(){if(state.simTimer)clearInterval(state.simTimer);state.simTimer=null;state.simulation=false;}
async function applyOutput(ma){if(!$('outputEnabled').checked)return;ma=Number(ma);if(!Number.isFinite(ma))return;state.outputMa=Math.round(clamp(ma,4,20)*10)/10;state.outputOpen=false;renderMain();await send(`SET:CURRENT:${state.outputMa.toFixed(3)}`);}
function populateTypeSelect(id){const s=$(id);s.innerHTML='';Object.entries(TYPES).forEach(([k,v])=>{const o=document.createElement('option');o.value=k;o.textContent=v.name;s.appendChild(o);});}
function fillScaleInputs(prefix,scale){$(prefix+'SensorType').value=scale.type;$(prefix+'SensorUnit').value=scale.unit;$(prefix+'SensorMin').value=scale.min;$(prefix+'SensorMax').value=scale.max;$(prefix+'CurrentMin').value=scale.currentMin;$(prefix+'CurrentMax').value=scale.currentMax;$(prefix+'MeasuredLow').value=scale.measuredLow;$(prefix+'MeasuredHigh').value=scale.measuredHigh;updateCalInfo(prefix,scale);}
function scaleFromInputs(prefix){const type=$(prefix+'SensorType').value;const base=TYPES[type]||TYPES.custom;return{type,name:base.name,unit:$(prefix+'SensorUnit').value||base.unit,min:Number($(prefix+'SensorMin').value),max:Number($(prefix+'SensorMax').value),currentMin:4,currentMax:20,measuredLow:Number($(prefix+'MeasuredLow').value),measuredHigh:Number($(prefix+'MeasuredHigh').value)};}
function setPresetFromType(prefix){const t=$(prefix+'SensorType').value,b=TYPES[t];$(prefix+'SensorUnit').value=b.unit;$(prefix+'SensorMin').value=b.min;$(prefix+'SensorMax').value=b.max;updateCalInfo(prefix,scaleFromInputs(prefix));}
function updateCalInfo(prefix,s){const id=prefix==='out'?'outCalibrationInfo':'inCalibrationInfo';$(id).innerHTML=`<b>${s.name}</b>: ${fmt(s.min,2)} ${s.unit} = ${fmt(s.currentMin,1)} mA · ${fmt(s.max,2)} ${s.unit} = ${fmt(s.currentMax,1)} mA<br>Punto medio: ${fmt((s.min+s.max)/2,2)} ${s.unit} = ${fmt((s.currentMin+s.currentMax)/2,1)} mA`;}
function applyScale(prefix){const s=scaleFromInputs(prefix);if(!Number.isFinite(s.min)||!Number.isFinite(s.max)||s.max===s.min){alert('El rango mínimo y máximo debe ser válido.');return;}if(prefix==='out')state.outScale=s;else state.inScale=s;saveLocal();renderMain();updateCalInfo(prefix,s);}
async function sendCalibration(prefix){applyScale(prefix);const s=prefix==='out'?state.outScale:state.inScale;const ch=prefix==='out'?'OUT':'IN';await send(`CAL:${ch}:TYPE:${s.type}`);await send(`CAL:${ch}:RANGE:${s.min}:${s.max}:${s.unit}`);await send(`CAL:${ch}:CURRENT:${s.currentMin}:${s.currentMax}`);await send(`CAL:${ch}:POINTS:${s.measuredLow}:${s.measuredHigh}`);}

function updateScaleCards(){
  const out=$('outputScaleSummary'),inp=$('inputScaleSummary');
  if(out)out.innerHTML=`<b>${state.outScale.name}</b><br>${fmt(state.outScale.min,2)} a ${fmt(state.outScale.max,2)} ${state.outScale.unit} ↔ 4.0–20.0 mA`;
  if(inp)inp.innerHTML=`<b>${state.inScale.name}</b><br>${fmt(state.inScale.min,2)} a ${fmt(state.inScale.max,2)} ${state.inScale.unit} ↔ 4.0–20.0 mA`;
  if($('rampRunUnit'))$('rampRunUnit').textContent=state.outScale.unit;
}
function updateRampProgress(){
  if(!state.rampState){if($('rampProgressText'))$('rampProgressText').textContent='0%';if($('rampProgressFill'))$('rampProgressFill').style.width='0%';return;}
  const rs=state.rampState,total=Math.max(1,rs.r.repeats*rs.seq.length),done=rs.rep*rs.seq.length+rs.i,p=Math.min(100,Math.round(done*100/total));
  $('rampProgressText').textContent=p+'%';$('rampProgressFill').style.width=p+'%';
}
function drawRampChart(){
  const c=$('rampChart');if(!c)return;const ctx=c.getContext('2d'),w=c.width,h=c.height;ctx.clearRect(0,0,w,h);
  ctx.strokeStyle='#64748b';ctx.lineWidth=1;ctx.beginPath();
  for(let i=0;i<=4;i++){const y=24+(h-48)*i/4;ctx.moveTo(46,y);ctx.lineTo(w-18,y);}ctx.stroke();
  ctx.fillStyle='#94a3b8';ctx.font='24px system-ui';[20,16,12,8,4].forEach((v,i)=>ctx.fillText(v+' mA',2,32+(h-48)*i/4));
  const data=state.rampTrace;if(data.length<2)return;
  const xs=i=>48+(w-70)*i/(data.length-1),ys=v=>24+(h-48)*(20-clamp(Number(v),4,20))/16;
  const line=(key,color)=>{ctx.strokeStyle=color;ctx.lineWidth=4;ctx.beginPath();let started=false;data.forEach((p,i)=>{if(p[key]==null)return;const x=xs(i),y=ys(p[key]);if(!started){ctx.moveTo(x,y);started=true;}else ctx.lineTo(x,y);});ctx.stroke();};
  line('out','#38bdf8');line('inp','#22c55e');
}

function getRampConfig(){return{type:$('rampType').value,repeats:Number($('rampRepeats').value)||1,start:Number($('rampStart').value),end:Number($('rampEnd').value),rise:Number($('rampRise').value)||1,holdHigh:Number($('rampHoldHigh').value)||0,fall:Number($('rampFall').value)||1,holdLow:Number($('rampHoldLow').value)||0,steps:Number($('rampSteps').value)||8,tick:Number($('rampTick').value)||500,points:clone(state.points)};}
function setRampConfig(r){if(!r)return;[['rampType','type'],['rampRepeats','repeats'],['rampStart','start'],['rampEnd','end'],['rampRise','rise'],['rampHoldHigh','holdHigh'],['rampFall','fall'],['rampHoldLow','holdLow'],['rampSteps','steps'],['rampTick','tick']].forEach(([id,k])=>{if(r[k]!=null)$(id).value=r[k];});state.points=clone(r.points||state.points);renderPoints();}
function buildRamp(r){const tick=r.tick/1000,arr=[];const pushSeg=(a,b,d)=>{const n=Math.max(1,Math.round(d/tick));for(let i=0;i<=n;i++)arr.push(a+(b-a)*i/n);};const hold=(v,d)=>{const n=Math.max(0,Math.round(d/tick));for(let i=0;i<n;i++)arr.push(v);};
  if(r.type==='linear')pushSeg(r.start,r.end,r.rise);
  else if(r.type==='triangle'){pushSeg(r.start,r.end,r.rise);pushSeg(r.end,r.start,r.fall);}
  else if(r.type==='steps'){for(let i=0;i<r.steps;i++){const v=r.start+(r.end-r.start)*i/(r.steps-1);hold(v,Math.max(tick,r.rise/r.steps));}}
  else if(r.type==='cycle'){pushSeg(r.start,r.end,r.rise);hold(r.end,r.holdHigh);pushSeg(r.end,r.start,r.fall);hold(r.start,r.holdLow);}
  else{const p=[...r.points].sort((a,b)=>a.t-b.t);for(let j=0;j<p.length-1;j++){const a=p[j],b=p[j+1];pushSeg(a.v,b.v,Math.max(tick,b.t-a.t));}}
  return arr;
}
function stopRamp(){if(state.rampTimer)clearInterval(state.rampTimer);state.rampTimer=null;state.rampState=null;state.rampPaused=false;$('pauseRampBtn').textContent='Ⅱ Pausar';$('rampStatus').textContent='Rampa detenida.';updateRampProgress();drawRampChart();}
function runRamp(){
  if(state.rampTimer)clearInterval(state.rampTimer);
  const r=getRampConfig(),seq=buildRamp(r);if(!seq.length)return;
  state.rampTrace=[];state.rampPaused=false;state.rampState={r,seq,i:0,rep:0};
  $('pauseRampBtn').textContent='Ⅱ Pausar';$('rampStatus').textContent='Rampa ejecutándose…';updateRampProgress();drawRampChart();
  state.rampTimer=setInterval(async()=>{
    if(state.rampPaused)return;
    const rs=state.rampState;if(!rs)return;
    if(rs.i>=rs.seq.length){rs.i=0;rs.rep++;if(rs.rep>=r.repeats){if(state.rampTimer)clearInterval(state.rampTimer);state.rampTimer=null;$('rampStatus').textContent='Rampa finalizada.';$('rampProgressText').textContent='100%';$('rampProgressFill').style.width='100%';return;}}
    const eng=rs.seq[rs.i++],ma=engToMa(eng,state.outScale);
    await applyOutput(ma);
    state.rampTrace.push({t:Date.now(),out:state.outputMa,inp:state.inputMa,eng});if(state.rampTrace.length>500)state.rampTrace.shift();
    $('rampRunEng').textContent=fmt(eng,2);$('rampRunMa').textContent=fmt(state.outputMa,1);$('rampRunUnit').textContent=state.outScale.unit;
    updateRampProgress();drawRampChart();
  },Math.max(100,r.tick));
}
function renderPoints(){$('pointsBody').innerHTML='';state.points.forEach((p,i)=>{const tr=document.createElement('tr');tr.innerHTML=`<td>${i+1}</td><td><input data-i="${i}" data-k="t" type="number" step="0.1" value="${p.t}"></td><td><input data-i="${i}" data-k="v" type="number" step="0.1" value="${p.v}"></td><td><button class="btn small" data-del="${i}">×</button></td>`;$('pointsBody').appendChild(tr);});}
function renderProfiles(){const box=$('profileList');box.innerHTML='';if(!state.profiles.length){box.innerHTML='<p class="hint">No hay perfiles guardados.</p>';return;}state.profiles.forEach((p,i)=>{const d=document.createElement('div');d.className='profile-item';d.innerHTML=`<h3>${escapeHtml(p.name||'Perfil')}</h3><p>${escapeHtml(p.desc||'')} · ${escapeHtml(p.scale?.name||'')} ${p.scale?`${p.scale.min}–${p.scale.max} ${p.scale.unit}`:''}</p><div class="profile-actions"><button class="btn small" data-load="${i}">Cargar</button><button class="btn small primary" data-run="${i}">Ejecutar</button><button class="btn small" data-upload="${i}">Subir equipo</button><button class="btn small danger" data-delete="${i}">Eliminar</button></div>`;box.appendChild(d);});}
function escapeHtml(s){return String(s).replace(/[&<>'"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;',"'":'&#39;','"':'&quot;'}[c]));}
function saveProfile(){const name=$('profileName').value.trim()||`Perfil ${state.profiles.length+1}`;state.profiles.push({name,desc:$('profileDesc').value.trim(),scale:clone(state.outScale),ramp:getRampConfig(),savedAt:new Date().toISOString()});saveLocal();renderProfiles();}
function loadProfile(i){const p=state.profiles[i];if(!p)return;state.outScale=clone(p.scale||state.outScale);fillScaleInputs('out',state.outScale);setRampConfig(p.ramp);saveLocal();renderMain();document.querySelector('[data-tab="ramps"]').click();}
function runProfile(i){loadProfile(i);runRamp();}
async function uploadProfile(p,num){if(!p)return;const seq=buildRamp(p.ramp);await send(`PROFILE:NEW:${num}:${seq.length}`);for(let i=0;i<seq.length;i++){const ma=engToMa(seq[i],p.scale||state.outScale);await send(`PROFILE:POINT:${num}:${i+1}:${(i*p.ramp.tick/1000).toFixed(3)}:${ma.toFixed(3)}`);}await send(`PROFILE:SAVE:${num}`);}
function download(name,text,type='text/plain'){const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([text],{type}));a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);}
function exportProfiles(){download('Simulink-perfiles.json',JSON.stringify(state.profiles,null,2),'application/json');}
function exportCsv(){const rows=['timestamp,salida_mA,entrada_mA,salida_valor,salida_unidad,entrada_valor,entrada_unidad'];state.chart.forEach(p=>rows.push(`${new Date(p.t).toISOString()},${p.out.toFixed(3)},${p.inp.toFixed(3)},${maToEng(p.out,state.outScale).toFixed(3)},${state.outScale.unit},${maToEng(p.inp,state.inScale).toFixed(3)},${state.inScale.unit}`));download('Simulink-datos.csv',rows.join('\n'),'text/csv');}
function drawChart(){const c=$('chart');if(!c)return;const ctx=c.getContext('2d'),w=c.width,h=c.height;ctx.clearRect(0,0,w,h);ctx.strokeStyle='#64748b';ctx.lineWidth=1;ctx.beginPath();for(let i=0;i<=4;i++){const y=20+(h-40)*i/4;ctx.moveTo(50,y);ctx.lineTo(w-20,y);}ctx.stroke();const data=state.chart;if(data.length<2)return;const xs=(i)=>50+(w-70)*i/(data.length-1);const ys=(v)=>20+(h-40)*(22-v)/20;const line=(key,color)=>{ctx.strokeStyle=color;ctx.lineWidth=3;ctx.beginPath();data.forEach((p,i)=>{const x=xs(i),y=ys(p[key]);i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();};line('out','#38bdf8');line('inp','#22c55e');}
function bindTabs(){document.querySelectorAll('.tab').forEach(b=>b.addEventListener('click',()=>{document.querySelectorAll('.tab').forEach(x=>x.classList.remove('active'));document.querySelectorAll('.panel').forEach(x=>x.classList.remove('active'));b.classList.add('active');$('tab-'+b.dataset.tab).classList.add('active');if(b.dataset.tab==='profiles')renderProfiles();if(b.dataset.tab==='ramps')drawRampChart();}));}
function bind(){
  bindTabs();$('connectBtn').onclick=connectSerial;$('disconnectBtn').onclick=disconnectSerial;$('startSimulationBtn').onclick=startSimulation;$('transportMode').onchange=()=>{if($('transportMode').value==='serial'){stopSimulation();state.connected=false;$('connectBtn').disabled=false;$('disconnectBtn').disabled=true;$('supportPill').textContent='Web Serial';$('supportPill').className='pill';$('modeStatus').textContent='Web Serial seleccionado. Vinculá el HC-05 en Android y luego presioná Conectar.';setSerialDiag('esperando selección del puerto.');}else{$('modeStatus').textContent='Modo simulación listo para usar sin hardware.';}};
  $('manualSlider').oninput=e=>{const v=clamp(Number(e.target.value),4,20);$('manualCurrent').value=v.toFixed(1);state.outputMa=v;renderMain();};$('manualSlider').onchange=e=>applyOutput(Number(e.target.value));$('manualCurrent').onchange=e=>applyOutput(Math.round(clamp(Number(e.target.value),4,20)*10)/10);$('minusBtn').onclick=()=>applyOutput(Math.round(clamp(state.outputMa-.1,4,20)*10)/10);$('plusBtn').onclick=()=>applyOutput(Math.round(clamp(state.outputMa+.1,4,20)*10)/10);$('applyCurrentBtn').onclick=()=>applyOutput(Math.round(clamp(Number($('manualCurrent').value),4,20)*10)/10);
  $('outputEnabled').onchange=async e=>send(e.target.checked?'SET:OUTPUT:ON':'SET:OUTPUT:OFF');
  $('resetStatsBtn').onclick=()=>{state.samples=[];renderMain();};$('runRampBtn').onclick=runRamp;$('pauseRampBtn').onclick=()=>{state.rampPaused=!state.rampPaused;$('pauseRampBtn').textContent=state.rampPaused?'▶ Continuar':'Ⅱ Pausar';$('rampStatus').textContent=state.rampPaused?'Rampa pausada.':'Rampa ejecutándose…';};$('stopRampBtn').onclick=stopRamp;
  $('addPointBtn').onclick=()=>{const last=state.points.at(-1)||{t:0,v:0};state.points.push({t:last.t+10,v:last.v});renderPoints();};$('pointsBody').oninput=e=>{if(e.target.dataset.i!=null)state.points[Number(e.target.dataset.i)][e.target.dataset.k]=Number(e.target.value);};$('pointsBody').onclick=e=>{if(e.target.dataset.del!=null){state.points.splice(Number(e.target.dataset.del),1);renderPoints();}};
  $('saveProfileBtn').onclick=saveProfile;$('exportProfilesBtn').onclick=exportProfiles;$('importProfiles').onchange=async e=>{try{const arr=JSON.parse(await e.target.files[0].text());if(!Array.isArray(arr))throw Error('Formato inválido');state.profiles=arr;saveLocal();renderProfiles();}catch(err){alert('No se pudo importar: '+err.message);}e.target.value='';};
  $('profileList').onclick=e=>{const ds=e.target.dataset;if(ds.load!=null)loadProfile(Number(ds.load));if(ds.run!=null)runProfile(Number(ds.run));if(ds.delete!=null){state.profiles.splice(Number(ds.delete),1);saveLocal();renderProfiles();}if(ds.upload!=null)uploadProfile(state.profiles[Number(ds.upload)],Number($('profileNumber').value)||1);};
  ['out','in'].forEach(p=>{$(p+'SensorType').onchange=()=>setPresetFromType(p);['SensorUnit','SensorMin','SensorMax','CurrentMin','CurrentMax','MeasuredLow','MeasuredHigh'].forEach(s=>$(p+s).oninput=()=>updateCalInfo(p,scaleFromInputs(p)));});$('applyOutScaleBtn').onclick=()=>applyScale('out');$('applyInScaleBtn').onclick=()=>applyScale('in');$('sendOutCalBtn').onclick=()=>sendCalibration('out');$('sendInCalBtn').onclick=()=>sendCalibration('in');
  $('getStatusBtn').onclick=()=>send('GET:STATUS');$('uploadProfileBtn').onclick=()=>uploadProfile({scale:clone(state.outScale),ramp:getRampConfig()},Number($('profileNumber').value)||1);$('runDeviceProfileBtn').onclick=()=>send(`PROFILE:RUN:${Number($('profileNumber').value)||1}`);$('terminalSendBtn').onclick=()=>{const v=$('terminalInput').value.trim();if(v){send(v);$('terminalInput').value='';}};$('clearTerminalBtn').onclick=()=>{$('terminal').textContent='';};$('exportCsvBtn').onclick=exportCsv;
  $('themeBtn').onclick=()=>{document.documentElement.classList.toggle('light');localStorage.setItem('simcorr_theme',document.documentElement.classList.contains('light')?'light':'dark');};
  $('installBtn').onclick=async()=>{if(state.installPrompt){state.installPrompt.prompt();await state.installPrompt.userChoice;state.installPrompt=null;$('installBtn').hidden=true;}};
  window.addEventListener('beforeinstallprompt',e=>{e.preventDefault();state.installPrompt=e;$('installBtn').hidden=false;});
}
function init(){
  if(localStorage.getItem('simcorr_theme')==='light')document.documentElement.classList.add('light');
  loadLocal();populateTypeSelect('outSensorType');populateTypeSelect('inSensorType');fillScaleInputs('out',state.outScale);fillScaleInputs('in',state.inScale);renderPoints();renderProfiles();bind();renderMain();
  $('transportMode').value='serial';
  $('connectBtn').disabled=false;$('disconnectBtn').disabled=true;
  $('supportPill').textContent=('serial'in navigator)?'Web Serial disponible':'Web Serial no disponible';
  $('modeStatus').textContent=('serial'in navigator)?'Web Serial listo. El HC-05 debe estar vinculado previamente en Android.':'Web Serial no disponible en este navegador.';
  setSerialDiag(('serial'in navigator)?'listo para seleccionar HC-05 SPP.':'API Web Serial ausente.');
  if('serial'in navigator){
    navigator.serial.addEventListener('disconnect',e=>{if(e.target===state.port){log('HC-05 desconectado','INFO');state.connected=false;$('connectBtn').disabled=false;$('disconnectBtn').disabled=true;$('supportPill').textContent='Desconectado';$('supportPill').className='pill';setSerialDiag('el enlace Bluetooth se desconectó.');}});
  }
  if('serviceWorker'in navigator)navigator.serviceWorker.register('./sw.js').catch(()=>{});
}
document.addEventListener('DOMContentLoaded',init);