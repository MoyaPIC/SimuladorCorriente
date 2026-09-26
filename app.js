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
const PROFILE_TYPE_LABELS={
  linear:'Lineal',
  triangle:'Triangular',
  steps:'Escalonada',
  cycle:'Ciclo',
  custom:'Multipunto',
  unknown:'Tipo desconocido'
};
let serialReadBuffer="";
let previewChart=null;
let rampLiveChart=null;
let inputHistoryChart=null;
const clone=x=>JSON.parse(JSON.stringify(x));
const makeScale=(type='current')=>({...clone(TYPES[type]),type,currentMin:4,currentMax:20,measuredLow:4,measuredHigh:20});
const state={
  outScale:makeScale('current'), inScale:makeScale('current'), outputMa:12, inputMa:null, outputOpen:false,
  port:null,reader:null,writer:null,connected:false,simulation:false,simTimer:null,simPhase:0,
  samples:[],chart:[],profiles:[],points:[{t:0,v:0},{t:10,v:100}], rampTimer:null,rampPaused:false,rampState:null,
  installPrompt:null,rampTrace:[],
  deviceProfiles:Array(16).fill(null),deviceSlotNames:{},awaitingProfileList:false,pendingUploadIndex:null
};
function clamp(v,a,b){return Math.max(a,Math.min(b,v));}
function engToMa(v,s){const lo=Math.min(s.min,s.max),hi=Math.max(s.min,s.max),cv=clamp(Number(v),lo,hi);const span=s.max-s.min||1;const ma=s.currentMin+(cv-s.min)*(s.currentMax-s.currentMin)/span;return clamp(ma,4,20);}
function maToEng(ma,s){const span=s.currentMax-s.currentMin||1;return s.min+(clamp(Number(ma),4,20)-s.currentMin)*(s.max-s.min)/span;}
function fmt(v,n=1){return Number.isFinite(Number(v))?Number(v).toFixed(n):'—';}
function log(msg,dir='•'){const t=$('terminal');t.textContent+=`${new Date().toLocaleTimeString()} ${dir} ${msg}\n`;t.scrollTop=t.scrollHeight;}
function saveLocal(){localStorage.setItem('simcorr_profiles_v12',JSON.stringify(state.profiles));localStorage.setItem('simcorr_outscale_v12',JSON.stringify(state.outScale));localStorage.setItem('simcorr_inscale_v12',JSON.stringify(state.inScale));localStorage.setItem('simcorr_device_slot_names_v14',JSON.stringify(state.deviceSlotNames));}
function loadLocal(){try{state.profiles=JSON.parse(localStorage.getItem('simcorr_profiles_v12')||'[]');state.outScale={...makeScale(),...JSON.parse(localStorage.getItem('simcorr_outscale_v12')||'{}')};state.inScale={...makeScale(),...JSON.parse(localStorage.getItem('simcorr_inscale_v12')||'{}')};state.deviceSlotNames=JSON.parse(localStorage.getItem('simcorr_device_slot_names_v14')||'{}');}catch(e){console.warn(e);}}
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
  if(state.simulation){const response=simulateCommand(line);(Array.isArray(response)?response:[response]).forEach(handleLine);return true;}
  if(!state.writer){log('Puerto no conectado','ERR');return false;}
  try{await state.writer.write(new TextEncoder().encode(line+'\n'));return true;}catch(e){log(e.message,'ERR');return false;}
}
function simulateCommand(line){
  if(line.startsWith('GET:STATUS'))return `DATA:OUT=${fmt(state.outputMa,3)}:IN=${fmt(state.inputMa??state.outputMa,3)}`;
  if(line.startsWith('SET:CURRENT:')){const v=Number(line.split(':')[2]);if(Number.isFinite(v))state.outputMa=clamp(v,4,20);return 'OK';}
  if(line.startsWith('SET:OUTPUT:OPEN')){state.outputOpen=true;return 'OK';}
  if(line.startsWith('SET:OUTPUT:ON')){state.outputOpen=false;return 'OK';}
  if(line==='PROFILE:LIST'){
    const lines=[];
    state.deviceProfiles.forEach((p,i)=>{if(p)lines.push(`PROFILE:${i+1}:COUNT=${p.count}:REP=${p.repeats}:TYPE=${p.type||'unknown'}`);});
    lines.push('OK');
    return lines;
  }
  if(line.startsWith('PROFILE:DELETE:')){const slot=Number(line.split(':')[2]);if(slot>=1&&slot<=16)state.deviceProfiles[slot-1]=null;return 'OK';}
  if(line.startsWith('PROFILE:RUN:'))return 'OK';
  if(line.startsWith('PROFILE:'))return 'OK';
  if(line.startsWith('CAL:'))return 'OK';
  return 'OK';
}
function handleLine(line){
  if(!line)return;
  log(line,'RX');
  if(line.startsWith('DATA:')){
    const mo=line.match(/OUT=([-\d.]+)/),mi=line.match(/IN=([-\d.]+)/);
    if(mo)state.outputMa=Number(mo[1]);
    if(mi)addInput(Number(mi[1]));
    renderMain();
    return;
  }
  const pm=line.match(/^PROFILE:(\d+):COUNT=(\d+):REP=(\d+)(?::TYPE=([a-z]+))?/);
  if(pm){
    const slot=Number(pm[1]);
    if(slot>=1&&slot<=16){
      state.deviceProfiles[slot-1]={
        slot,
        count:Number(pm[2]),
        repeats:Number(pm[3]),
        type:pm[4]||'unknown'
      };
    }
    renderDeviceProfiles();
    return;
  }
  if(line==='ERR:EEPROM_OFFLINE'&&state.awaitingProfileList){
    state.awaitingProfileList=false;
    const mem=$('connectionMemorySummary');
    if(mem)mem.innerHTML='<b>EEPROM 24C512 no detectada</b><br>No fue posible leer los ensayos almacenados.';
    $('deviceMemoryStatus').innerHTML='<b>EEPROM 24C512 no detectada.</b>';
    return;
  }
  if(line==='OK'&&state.awaitingProfileList){
    state.awaitingProfileList=false;
    renderDeviceProfiles();
    updateConnectionMemorySummary();
  }
}
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
    await requestDeviceProfiles();
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

function chartColors(){
  const css=getComputedStyle(document.documentElement);
  return{
    brand:css.getPropertyValue('--brand').trim()||'#c9252d',
    brandStrong:css.getPropertyValue('--brand-strong').trim()||'#a91d24',
    success:css.getPropertyValue('--success').trim()||'#198754',
    text:css.getPropertyValue('--text').trim()||'#18191c',
    muted:css.getPropertyValue('--muted').trim()||'#6c7078',
    border:css.getPropertyValue('--border').trim()||'#dfe2e7',
    surface:css.getPropertyValue('--surface').trim()||'#ffffff'
  };
}
function chartReady(){return typeof window.Chart!=='undefined';}
function chartAxis(title,unit,min,max){
  const c=chartColors();
  const axis={
    grid:{color:c.border,drawBorder:false},
    border:{display:false},
    ticks:{color:c.muted,padding:8,font:{size:11}},
    title:{display:true,text:title+(unit?' ('+unit+')':''),color:c.muted,font:{size:11,weight:'600'},padding:{top:8}}
  };
  if(Number.isFinite(min))axis.min=min;
  if(Number.isFinite(max))axis.max=max;
  return axis;
}
function commonChartOptions(){
  const c=chartColors();
  return{
    responsive:true,
    maintainAspectRatio:false,
    normalized:true,
    animation:false,
    interaction:{mode:'index',intersect:false},
    layout:{padding:{top:6,right:8,bottom:2,left:4}},
    plugins:{
      legend:{
        display:true,
        position:'top',
        align:'start',
        labels:{color:c.text,usePointStyle:true,pointStyle:'line',boxWidth:28,boxHeight:3,padding:16,font:{size:11,weight:'600'}}
      },
      tooltip:{
        enabled:true,
        backgroundColor:c.surface,
        titleColor:c.text,
        bodyColor:c.text,
        borderColor:c.border,
        borderWidth:1,
        padding:11,
        cornerRadius:10,
        displayColors:true,
        titleFont:{weight:'700'},
        bodySpacing:5
      }
    }
  };
}
function chartDataset(label,color,data,extra={}){
  return{
    label,
    data,
    parsing:false,
    borderColor:color,
    backgroundColor:color,
    borderWidth:2.5,
    pointRadius:0,
    pointHoverRadius:4,
    pointHitRadius:12,
    tension:.18,
    cubicInterpolationMode:'monotone',
    spanGaps:true,
    fill:false,
    ...extra
  };
}
function destroyChart(chart){
  if(chart){try{chart.destroy();}catch(e){}}
  return null;
}
function refreshChartsForTheme(){
  previewChart=destroyChart(previewChart);
  rampLiveChart=destroyChart(rampLiveChart);
  inputHistoryChart=destroyChart(inputHistoryChart);
  drawRampPreview();
  drawRampChart();
  drawChart();
}

function updateScaleCards(){
  const inp=$('inputScaleSummary');
  if(inp)inp.innerHTML=`<b>${state.inScale.name}</b><br>${fmt(state.inScale.min,2)} a ${fmt(state.inScale.max,2)} ${state.inScale.unit} ↔ 4.0–20.0 mA`;

  if($('rampRunUnit'))$('rampRunUnit').textContent=state.outScale.unit;
  if($('rampStartUnit'))$('rampStartUnit').textContent=state.outScale.unit;
  if($('rampEndUnit'))$('rampEndUnit').textContent=state.outScale.unit;

  // Limita también los campos de la rampa al rango de ingeniería activo.
  const lo=Math.min(Number(state.outScale.min),Number(state.outScale.max));
  const hi=Math.max(Number(state.outScale.min),Number(state.outScale.max));
  ['rampStart','rampEnd'].forEach(id=>{
    const el=$(id);
    if(!el)return;
    el.min=lo;
    el.max=hi;
  });
}
const RAMP_META={
  linear:{label:'Lineal',guide:'La salida avanza de forma continua desde el valor inicial al final.',fields:['start','end','rise','repeats','tick']},
  triangle:{label:'Triangular',guide:'La salida sube al valor final y luego vuelve al inicial.',fields:['start','end','rise','fall','repeats','tick']},
  steps:{label:'Escalonada',guide:'La salida recorre el rango mediante una cantidad definida de escalones.',fields:['start','end','rise','steps','repeats','tick']},
  cycle:{label:'Ciclo',guide:'Sube, mantiene el máximo, baja y mantiene el mínimo antes de repetir.',fields:['start','end','rise','holdHigh','fall','holdLow','repeats','tick']},
  custom:{label:'Multipunto',guide:'Definí libremente cada punto de tiempo y valor.',fields:['repeats','tick']}
};
function updateRampEditor(){
  const type=$('rampType').value,meta=RAMP_META[type]||RAMP_META.linear;
  document.querySelectorAll('.ramp-type-btn').forEach(b=>b.classList.toggle('active',b.dataset.rampType===type));
  document.querySelectorAll('[data-ramp-field]').forEach(el=>el.hidden=!meta.fields.includes(el.dataset.rampField));
  $('customPointsCard').hidden=type!=='custom';
  $('rampTypeBadge').textContent=meta.label;
  $('rampGuideText').textContent=meta.guide;
  $('rampRiseLabel').textContent=type==='steps'?'Duración total':'Tiempo de subida';
  $('rampStartUnit').textContent=state.outScale.unit;
  $('rampEndUnit').textContent=state.outScale.unit;
  drawRampPreview();
}
function drawRampPreview(){
  const r=getRampConfig(),seq=buildRamp(r);
  const values=(seq.length?seq:[r.start||0]).map(Number);
  const tickSec=Math.max(100,Number(r.tick)||500)/1000;
  const durationOne=Math.max(0,(values.length-1)*tickSec);
  const totalDuration=durationOne*Math.max(1,r.repeats);

  $('rampSummary').innerHTML=
    '<b>'+RAMP_META[r.type].label+'</b>'+
    '<span>'+fmt(r.start,2)+' → '+fmt(r.end,2)+' '+state.outScale.unit+'</span>'+
    '<span>'+Math.max(1,r.repeats)+' rep.</span>'+
    '<span>≈ '+fmt(totalDuration,1)+' s</span>';

  if(!$('tab-ramps').classList.contains('active')||!chartReady())return;
  const lo=Math.min(Number(state.outScale.min),Number(state.outScale.max));
  const hi=Math.max(Number(state.outScale.min),Number(state.outScale.max));
  const c=chartColors();
  const data=values.map((v,i)=>({x:i*tickSec,y:clamp(v,lo,hi)}));
  const canvas=$('rampPreview');if(!canvas)return;

  previewChart=destroyChart(previewChart);
  const options=commonChartOptions();
  options.plugins.legend.display=false;
  options.plugins.tooltip.callbacks={
    title:items=>'Tiempo: '+fmt(items[0]?.parsed?.x,2)+' s',
    label:item=>' '+state.outScale.name+': '+fmt(item.parsed.y,2)+' '+state.outScale.unit
  };
  const xAxis=chartAxis('Tiempo','s',0,Math.max(durationOne,.1));
  xAxis.type='linear';
  xAxis.ticks={...xAxis.ticks,callback:v=>fmt(v,1)+' s'};
  const yAxis=chartAxis(state.outScale.name,state.outScale.unit,lo,hi);
  yAxis.ticks={...yAxis.ticks,callback:v=>fmt(v,1)+' '+state.outScale.unit};
  options.scales={x:xAxis,y:yAxis};

  previewChart=new Chart(canvas,{
    type:'line',
    data:{datasets:[chartDataset('Consigna',c.brand,data,{borderWidth:3})]},
    options
  });
}

function updateRampProgress(){
  if(!state.rampState){if($('rampProgressText'))$('rampProgressText').textContent='0%';if($('rampProgressFill'))$('rampProgressFill').style.width='0%';return;}
  const rs=state.rampState,total=Math.max(1,rs.r.repeats*rs.seq.length),done=rs.rep*rs.seq.length+rs.i,p=Math.min(100,Math.round(done*100/total));
  $('rampProgressText').textContent=p+'%';$('rampProgressFill').style.width=p+'%';
}
function drawRampChart(){
  if(!$('tab-ramps').classList.contains('active')||!chartReady())return;
  const canvas=$('rampChart');if(!canvas)return;

  const data=state.rampTrace||[];
  const t0=data.length?(Number(data[0].t)||Date.now()):Date.now();
  const outData=data.map(p=>({x:Math.max(0,((Number(p.t)||t0)-t0)/1000),y:Number(p.out)})).filter(p=>Number.isFinite(p.y));
  const inData=data.map(p=>({x:Math.max(0,((Number(p.t)||t0)-t0)/1000),y:Number(p.inp)})).filter(p=>Number.isFinite(p.y));
  const maxT=Math.max(1,...outData.map(p=>p.x),...inData.map(p=>p.x));
  const c=chartColors();

  const options=commonChartOptions();
  options.plugins.tooltip.callbacks={
    title:items=>'Tiempo: '+fmt(items[0]?.parsed?.x,2)+' s',
    label:item=>' '+item.dataset.label+': '+fmt(item.parsed.y,3)+' mA',
    afterBody:items=>{
      if(!items?.length)return '';
      const i=items[0].dataIndex;
      const out=outData[i],inp=inData[i];
      if(!out||!inp)return '';
      return 'Error: '+fmt(inp.y-out.y,3)+' mA';
    }
  };
  const xAxis=chartAxis('Tiempo','s',0,maxT);
  xAxis.type='linear';
  xAxis.ticks={...xAxis.ticks,callback:v=>fmt(v,1)+' s'};
  const yAxis=chartAxis('Corriente','mA',4,20);
  yAxis.ticks={...yAxis.ticks,stepSize:4,callback:v=>fmt(v,0)+' mA'};
  options.scales={x:xAxis,y:yAxis};

  const datasets=[
    chartDataset('Consigna',c.brand,outData,{borderWidth:3}),
    chartDataset('Entrada medida',c.success,inData,{borderWidth:2.5})
  ];

  if(!rampLiveChart){
    rampLiveChart=new Chart(canvas,{type:'line',data:{datasets},options});
  }else{
    rampLiveChart.data.datasets=datasets;
    rampLiveChart.options=options;
    rampLiveChart.update('none');
  }
}

function getRampConfig(){
  const lo=Math.min(Number(state.outScale.min),Number(state.outScale.max));
  const hi=Math.max(Number(state.outScale.min),Number(state.outScale.max));
  return{
    type:$('rampType').value,
    repeats:Math.max(1,Number($('rampRepeats').value)||1),
    start:clamp(Number($('rampStart').value),lo,hi),
    end:clamp(Number($('rampEnd').value),lo,hi),
    rise:Math.max(.1,Number($('rampRise').value)||1),
    holdHigh:Math.max(0,Number($('rampHoldHigh').value)||0),
    fall:Math.max(.1,Number($('rampFall').value)||1),
    holdLow:Math.max(0,Number($('rampHoldLow').value)||0),
    steps:Math.max(2,Number($('rampSteps').value)||8),
    tick:Math.max(100,Number($('rampTick').value)||500),
    points:clone(state.points).map(p=>({t:Math.max(0,Number(p.t)||0),v:clamp(Number(p.v),lo,hi)}))
  };
}
function setRampConfig(r){if(!r)return;[['rampType','type'],['rampRepeats','repeats'],['rampStart','start'],['rampEnd','end'],['rampRise','rise'],['rampHoldHigh','holdHigh'],['rampFall','fall'],['rampHoldLow','holdLow'],['rampSteps','steps'],['rampTick','tick']].forEach(([id,k])=>{if(r[k]!=null)$(id).value=r[k];});state.points=clone(r.points||state.points);renderPoints();updateRampEditor();}
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
function renderPoints(){$('pointsBody').innerHTML='';state.points.forEach((p,i)=>{const tr=document.createElement('tr');tr.innerHTML=`<td>${i+1}</td><td><input data-i="${i}" data-k="t" type="number" step="0.1" value="${p.t}"></td><td><input data-i="${i}" data-k="v" type="number" step="0.1" value="${p.v}"></td><td><button class="btn small" data-del="${i}">×</button></td>`;$('pointsBody').appendChild(tr);});drawRampPreview();}
function renderProfiles(){
  const box=$('profileList');box.innerHTML='';
  if(!state.profiles.length){
    box.innerHTML='<p class="hint">Todavía no hay ensayos guardados. Creá una rampa y guardala para reutilizarla o cargarla en el equipo.</p>';
    return;
  }
  state.profiles.forEach((p,i)=>{
    const d=document.createElement('div');d.className='profile-item';
    const rampLabel=(RAMP_META[p.ramp?.type]||{}).label||'Prueba';
    const reps=Math.max(1,Number(p.ramp?.repeats)||1);
    d.innerHTML=`<div class="profile-title-row"><div><h3>${escapeHtml(p.name||'Ensayo')}</h3><p>${escapeHtml(p.desc||'Sin descripción')}</p></div><span class="pill">${escapeHtml(rampLabel)}</span></div><div class="profile-meta"><span>${escapeHtml(p.scale?.name||'Variable')}</span><span>${p.scale?`${p.scale.min}–${p.scale.max} ${escapeHtml(p.scale.unit)}`:''}</span><span>${reps} rep.</span></div><div class="profile-actions"><button class="btn small" data-load="${i}">Editar</button><button class="btn small" data-run="${i}">Probar</button><button class="btn small primary" data-upload="${i}">Guardar en equipo</button><button class="btn small danger" data-delete="${i}">Eliminar</button></div>`;
    box.appendChild(d);
  });
}
function escapeHtml(s){return String(s).replace(/[&<>'"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;',"'":'&#39;','"':'&quot;'}[c]));}
function saveProfile(){
  const name=$('profileName').value.trim()||`Ensayo ${state.profiles.length+1}`;
  state.profiles.push({name,desc:$('profileDesc').value.trim(),scale:clone(state.outScale),ramp:getRampConfig(),savedAt:new Date().toISOString()});
  saveLocal();renderProfiles();
  $('profileName').value='';$('profileDesc').value='';
}
function loadProfile(i){const p=state.profiles[i];if(!p)return;state.outScale=clone(p.scale||state.outScale);fillScaleInputs('out',state.outScale);setRampConfig(p.ramp);saveLocal();renderMain();document.querySelector('[data-tab="ramps"]').click();}
function runProfile(i){loadProfile(i);runRamp();}
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
function profileTypeLabel(type){
  return PROFILE_TYPE_LABELS[type]||PROFILE_TYPE_LABELS.unknown;
}
function updateConnectionMemorySummary(){
  const el=$('connectionMemorySummary');
  if(!el)return;
  const occupied=state.deviceProfiles.filter(Boolean);
  if(!occupied.length){
    el.innerHTML='<b>EEPROM 24C512: 0/16 slots ocupados</b><br>No hay ensayos almacenados en el instrumento.';
    return;
  }
  const details=occupied.map(p=>'Slot '+p.slot+': '+profileTypeLabel(p.type)).join(' · ');
  el.innerHTML='<b>EEPROM 24C512: '+occupied.length+'/16 slots ocupados</b><br>'+escapeHtml(details);
}
function renderDeviceProfiles(){
  const box=$('deviceProfileList');if(!box)return;
  box.innerHTML='';
  for(let slot=1;slot<=16;slot++){
    const p=state.deviceProfiles[slot-1];
    const name=state.deviceSlotNames[slot]||(p?'Ensayo almacenado':'Vacío');
    const d=document.createElement('div');
    d.className='device-slot '+(p?'occupied':'empty');
    d.innerHTML=`<div class="slot-number">SLOT <b>${slot}</b></div><div class="slot-info"><strong>${escapeHtml(name)}</strong><small>${p?`${profileTypeLabel(p.type)} · ${p.count} puntos · ${p.repeats} rep.`:'Disponible para guardar un ensayo'}</small></div><div class="slot-actions">${p?`<button class="btn small primary" data-device-run="${slot}">▶ Ejecutar</button><button class="btn small danger" data-device-delete="${slot}">Borrar</button>`:'<span class="slot-empty-badge">Vacío</span>'}</div>`;
    box.appendChild(d);
  }
  const occupied=state.deviceProfiles.filter(Boolean).length;
  $('deviceMemoryStatus').innerHTML=`<b>${occupied}/16 slots ocupados</b><br>${state.connected||state.simulation?'Memoria sincronizada con la última lectura.':'Conectá el instrumento para actualizar el contenido real.'}`;
}
async function requestDeviceProfiles(){
  if(!state.connected&&!state.simulation){$('deviceMemoryStatus').innerHTML='<b>Instrumento desconectado.</b><br>Conectá el HC-05 antes de leer la memoria.';return;}
  state.deviceProfiles=Array(16).fill(null);
  state.awaitingProfileList=true;
  $('deviceMemoryStatus').textContent='Leyendo memoria del instrumento...';
  const mem=$('connectionMemorySummary');
  if(mem)mem.innerHTML='<b>EEPROM 24C512</b><br>Leyendo slots almacenados...';
  renderDeviceProfiles();
  await send('PROFILE:LIST');
}
function updateDeviceSlotWarning(){
  const slot=Number($('deviceSlotSelect').value);
  const occupied=state.deviceProfiles[slot-1];
  const el=$('deviceSlotWarning');
  if(occupied){
    const name=state.deviceSlotNames[slot]||'Ensayo almacenado';
    el.className='slot-warning danger';
    el.innerHTML=`El slot ${slot} está ocupado por <b>${escapeHtml(name)}</b>. Si continuás, será reemplazado.`;
  }else{
    el.className='slot-warning ok';
    el.textContent=`El slot ${slot} está disponible.`;
  }
}
function openDeviceSaveDialog(index){
  const p=state.profiles[index];if(!p)return;
  state.pendingUploadIndex=index;
  $('deviceSaveSummary').innerHTML=`<b>${escapeHtml(p.name||'Ensayo')}</b><br>${escapeHtml(p.scale?.name||'Variable')}: ${p.scale?`${p.scale.min}–${p.scale.max} ${escapeHtml(p.scale.unit)}`:''} · ${Math.max(1,Number(p.ramp?.repeats)||1)} repeticiones`;
  const select=$('deviceSlotSelect');
  select.innerHTML='';
  for(let slot=1;slot<=16;slot++){
    const op=document.createElement('option');op.value=slot;
    const occupied=state.deviceProfiles[slot-1];
    op.textContent=`Slot ${slot} — ${occupied?(state.deviceSlotNames[slot]||'ocupado'):'vacío'}`;
    select.appendChild(op);
  }
  const firstEmpty=state.deviceProfiles.findIndex(x=>!x);
  select.value=String(firstEmpty>=0?firstEmpty+1:1);
  $('deviceUploadProgress').hidden=true;
  updateDeviceSlotWarning();
  $('deviceSaveDialog').showModal();
}
async function uploadProfile(p,num){
  if(!p||num<1||num>16)return false;
  if(!state.connected&&!state.simulation){alert('Primero conectá el HC-05 con el instrumento.');return false;}
  const seq=buildRamp(p.ramp);
  if(!seq.length){alert('El ensayo no contiene puntos para guardar.');return false;}
  if(seq.length>988){alert('El ensayo supera la capacidad máxima de 988 puntos por slot. Aumentá el intervalo de actualización o reducí la duración.');return false;}
  const repeats=Math.max(1,Number(p.ramp?.repeats)||1);
  const progress=$('deviceUploadProgress'),fill=$('deviceUploadFill'),label=$('deviceUploadText');
  if(progress){progress.hidden=false;fill.style.width='0%';label.textContent='Preparando memoria...';}
  const profileType=(p.ramp&&p.ramp.type)||'unknown';
  if(!(await send(`PROFILE:NEW:${num}:${seq.length}:${repeats}:${profileType}`)))return false;
  await sleep(40);
  for(let i=0;i<seq.length;i++){
    const ma=engToMa(seq[i],p.scale||state.outScale);
    if(!(await send(`PROFILE:POINT:${num}:${i+1}:${(i*p.ramp.tick/1000).toFixed(3)}:${ma.toFixed(3)}`)))return false;
    if(progress){const pc=Math.round((i+1)*100/seq.length);fill.style.width=pc+'%';label.textContent=`Guardando punto ${i+1} de ${seq.length} · ${pc}%`;}
    await sleep(35);
  }
  if(!(await send(`PROFILE:SAVE:${num}`)))return false;
  state.deviceProfiles[num-1]={slot:num,count:seq.length,repeats,type:profileType};
  state.deviceSlotNames[num]=p.name||`Ensayo slot ${num}`;
  saveLocal();renderDeviceProfiles();updateConnectionMemorySummary();
  if(progress){fill.style.width='100%';label.textContent='Ensayo guardado en el instrumento.';}
  return true;
}
async function confirmDeviceSave(){
  const p=state.profiles[state.pendingUploadIndex];
  const slot=Number($('deviceSlotSelect').value);
  if(!p||slot<1||slot>16)return;
  $('confirmDeviceSaveBtn').disabled=true;
  const ok=await uploadProfile(p,slot);
  $('confirmDeviceSaveBtn').disabled=false;
  if(ok){setTimeout(()=>$('deviceSaveDialog').close(),500);}
}
async function deleteDeviceProfile(slot){
  if(slot<1||slot>16)return;
  if(!confirm(`¿Borrar el ensayo almacenado en el slot ${slot}?`))return;
  await send(`PROFILE:DELETE:${slot}`);
  state.deviceProfiles[slot-1]=null;
  delete state.deviceSlotNames[slot];
  saveLocal();renderDeviceProfiles();
}
function download(name,text,type='text/plain'){const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([text],{type}));a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);}
function exportProfiles(){download('Simulink-perfiles.json',JSON.stringify(state.profiles,null,2),'application/json');}
function exportCsv(){const rows=['timestamp,salida_mA,entrada_mA,salida_valor,salida_unidad,entrada_valor,entrada_unidad'];state.chart.forEach(p=>rows.push(`${new Date(p.t).toISOString()},${p.out.toFixed(3)},${p.inp.toFixed(3)},${maToEng(p.out,state.outScale).toFixed(3)},${state.outScale.unit},${maToEng(p.inp,state.inScale).toFixed(3)},${state.inScale.unit}`));download('Simulink-datos.csv',rows.join('\n'),'text/csv');}
function drawChart(){
  if(!$('tab-input').classList.contains('active')||!chartReady())return;
  const canvas=$('inputChart');if(!canvas)return;

  const data=state.chart||[];
  const t0=data.length?(Number(data[0].t)||Date.now()):Date.now();
  const outData=data.map(p=>({x:Math.max(0,((Number(p.t)||t0)-t0)/1000),y:Number(p.out)})).filter(p=>Number.isFinite(p.y));
  const inData=data.map(p=>({x:Math.max(0,((Number(p.t)||t0)-t0)/1000),y:Number(p.inp)})).filter(p=>Number.isFinite(p.y));
  const maxT=Math.max(1,...outData.map(p=>p.x),...inData.map(p=>p.x));
  const c=chartColors();

  const options=commonChartOptions();
  options.plugins.tooltip.callbacks={
    title:items=>'Tiempo: '+fmt(items[0]?.parsed?.x,2)+' s',
    label:item=>' '+item.dataset.label+': '+fmt(item.parsed.y,3)+' mA',
    afterBody:items=>{
      if(!items?.length)return '';
      const i=items[0].dataIndex;
      const out=outData[i],inp=inData[i];
      if(!out||!inp)return '';
      return 'Error entrada-consigna: '+fmt(inp.y-out.y,3)+' mA';
    }
  };
  const xAxis=chartAxis('Tiempo','s',0,maxT);
  xAxis.type='linear';
  xAxis.ticks={...xAxis.ticks,callback:v=>fmt(v,1)+' s'};
  const yAxis=chartAxis('Corriente','mA',4,20);
  yAxis.ticks={...yAxis.ticks,stepSize:4,callback:v=>fmt(v,0)+' mA'};
  options.scales={x:xAxis,y:yAxis};

  const datasets=[
    chartDataset('Consigna',c.brand,outData,{borderWidth:2.5}),
    chartDataset('Entrada medida',c.success,inData,{borderWidth:2.5})
  ];

  if(!inputHistoryChart){
    inputHistoryChart=new Chart(canvas,{type:'line',data:{datasets},options});
  }else{
    inputHistoryChart.data.datasets=datasets;
    inputHistoryChart.options=options;
    inputHistoryChart.update('none');
  }
}

function bindTabs(){
  document.querySelectorAll('.tab').forEach(b=>b.addEventListener('click',()=>{
    document.querySelectorAll('.tab').forEach(x=>x.classList.remove('active'));
    document.querySelectorAll('.panel').forEach(x=>x.classList.remove('active'));
    b.classList.add('active');
    $('tab-'+b.dataset.tab).classList.add('active');

    if(b.dataset.tab==='profiles'){renderProfiles();renderDeviceProfiles();}

    requestAnimationFrame(()=>{
      if(b.dataset.tab==='ramps'){drawRampPreview();drawRampChart();}
      if(b.dataset.tab==='input')drawChart();
    });
  }));
}
function bind(){
  bindTabs();$('connectBtn').onclick=connectSerial;$('disconnectBtn').onclick=disconnectSerial;$('startSimulationBtn').onclick=startSimulation;$('transportMode').onchange=()=>{if($('transportMode').value==='serial'){stopSimulation();state.connected=false;$('connectBtn').disabled=false;$('disconnectBtn').disabled=true;$('supportPill').textContent='Web Serial';$('supportPill').className='pill';$('modeStatus').textContent='Web Serial seleccionado. Vinculá el HC-05 en Android y luego presioná Conectar.';setSerialDiag('esperando selección del puerto.');}else{$('modeStatus').textContent='Modo simulación listo para usar sin hardware.';}};
  $('manualSlider').oninput=e=>{const v=clamp(Number(e.target.value),4,20);$('manualCurrent').value=v.toFixed(1);state.outputMa=v;renderMain();};$('manualSlider').onchange=e=>applyOutput(Number(e.target.value));$('manualCurrent').onchange=e=>applyOutput(Math.round(clamp(Number(e.target.value),4,20)*10)/10);$('minusBtn').onclick=()=>applyOutput(Math.round(clamp(state.outputMa-.1,4,20)*10)/10);$('plusBtn').onclick=()=>applyOutput(Math.round(clamp(state.outputMa+.1,4,20)*10)/10);$('applyCurrentBtn').onclick=()=>applyOutput(Math.round(clamp(Number($('manualCurrent').value),4,20)*10)/10);
  $('outputEnabled').onchange=async e=>send(e.target.checked?'SET:OUTPUT:ON':'SET:OUTPUT:OFF');
  $('resetStatsBtn').onclick=()=>{state.samples=[];renderMain();};document.querySelectorAll('.ramp-type-btn').forEach(b=>b.onclick=()=>{$('rampType').value=b.dataset.rampType;updateRampEditor();});$('rampType').onchange=updateRampEditor;['rampStart','rampEnd','rampRise','rampHoldHigh','rampFall','rampHoldLow','rampSteps','rampRepeats','rampTick'].forEach(id=>$(id).addEventListener('input',drawRampPreview));
  ['rampStart','rampEnd'].forEach(id=>$(id).addEventListener('change',e=>{const r=getRampConfig();e.target.value=id==='rampStart'?r.start:r.end;drawRampPreview();}));$('runRampBtn').onclick=runRamp;$('pauseRampBtn').onclick=()=>{state.rampPaused=!state.rampPaused;$('pauseRampBtn').textContent=state.rampPaused?'▶ Continuar':'Ⅱ Pausar';$('rampStatus').textContent=state.rampPaused?'Rampa pausada.':'Rampa ejecutándose…';};$('stopRampBtn').onclick=stopRamp;
  $('addPointBtn').onclick=()=>{const last=state.points.at(-1)||{t:0,v:0};state.points.push({t:last.t+10,v:last.v});renderPoints();};$('pointsBody').oninput=e=>{if(e.target.dataset.i!=null){state.points[Number(e.target.dataset.i)][e.target.dataset.k]=Number(e.target.value);drawRampPreview();}};$('pointsBody').onclick=e=>{if(e.target.dataset.del!=null){state.points.splice(Number(e.target.dataset.del),1);renderPoints();}};
  $('saveProfileBtn').onclick=saveProfile;$('exportProfilesBtn').onclick=exportProfiles;$('importProfiles').onchange=async e=>{try{const arr=JSON.parse(await e.target.files[0].text());if(!Array.isArray(arr))throw Error('Formato inválido');state.profiles=arr;saveLocal();renderProfiles();}catch(err){alert('No se pudo importar: '+err.message);}e.target.value='';};
  $('profileList').onclick=e=>{const ds=e.target.dataset;if(ds.load!=null)loadProfile(Number(ds.load));if(ds.run!=null)runProfile(Number(ds.run));if(ds.delete!=null){state.profiles.splice(Number(ds.delete),1);saveLocal();renderProfiles();}if(ds.upload!=null)openDeviceSaveDialog(Number(ds.upload));};
  $('refreshDeviceProfilesBtn').onclick=requestDeviceProfiles;
  $('deviceProfileList').onclick=e=>{const ds=e.target.dataset;if(ds.deviceRun!=null)send(`PROFILE:RUN:${Number(ds.deviceRun)}`);if(ds.deviceDelete!=null)deleteDeviceProfile(Number(ds.deviceDelete));};
  $('deviceSlotSelect').onchange=updateDeviceSlotWarning;
  $('confirmDeviceSaveBtn').onclick=confirmDeviceSave;
  $('closeDeviceSaveDialog').onclick=()=>$('deviceSaveDialog').close();
  ['out','in'].forEach(p=>{$(p+'SensorType').onchange=()=>setPresetFromType(p);['SensorUnit','SensorMin','SensorMax','CurrentMin','CurrentMax','MeasuredLow','MeasuredHigh'].forEach(s=>$(p+s).oninput=()=>updateCalInfo(p,scaleFromInputs(p)));});$('applyOutScaleBtn').onclick=()=>applyScale('out');$('applyInScaleBtn').onclick=()=>applyScale('in');$('sendOutCalBtn').onclick=()=>sendCalibration('out');$('sendInCalBtn').onclick=()=>sendCalibration('in');
  $('terminalSendBtn').onclick=()=>{const v=$('terminalInput').value.trim();if(v){send(v);$('terminalInput').value='';}};$('clearTerminalBtn').onclick=()=>{$('terminal').textContent='';};$('exportCsvBtn').onclick=exportCsv;
  $('themeBtn').onclick=()=>{const root=document.documentElement;const next=root.dataset.theme==='dark'?'light':'dark';if(next==='dark')root.dataset.theme='dark';else delete root.dataset.theme;localStorage.setItem('simcorr_theme',next);refreshChartsForTheme();};
  $('installBtn').onclick=async()=>{if(state.installPrompt){state.installPrompt.prompt();await state.installPrompt.userChoice;state.installPrompt=null;$('installBtn').hidden=true;}};
  window.addEventListener('beforeinstallprompt',e=>{e.preventDefault();state.installPrompt=e;$('installBtn').hidden=false;});
}
function init(){
  if(localStorage.getItem('simcorr_theme')==='dark')document.documentElement.dataset.theme='dark';
  loadLocal();populateTypeSelect('outSensorType');populateTypeSelect('inSensorType');fillScaleInputs('out',state.outScale);fillScaleInputs('in',state.inScale);renderPoints();renderProfiles();renderDeviceProfiles();bind();updateRampEditor();renderMain();
  $('transportMode').value='serial';
  $('connectBtn').disabled=false;$('disconnectBtn').disabled=true;
  $('supportPill').textContent=('serial'in navigator)?'Web Serial disponible':'Web Serial no disponible';
  $('modeStatus').textContent=('serial'in navigator)?'Web Serial listo. El HC-05 debe estar vinculado previamente en Android.':'Web Serial no disponible en este navegador.';
  setSerialDiag(('serial'in navigator)?'listo para seleccionar HC-05 SPP.':'API Web Serial ausente.');
  if(!chartReady())log('Chart.js no pudo cargarse; las gráficas quedarán deshabilitadas hasta recuperar la librería.','WARN');
  if('serial'in navigator){
    navigator.serial.addEventListener('disconnect',e=>{if(e.target===state.port){log('HC-05 desconectado','INFO');state.connected=false;$('connectBtn').disabled=false;$('disconnectBtn').disabled=true;$('supportPill').textContent='Desconectado';$('supportPill').className='pill';setSerialDiag('el enlace Bluetooth se desconectó.');}});
  }
  if('serviceWorker'in navigator)navigator.serviceWorker.register('./sw.js').catch(()=>{});
}
document.addEventListener('DOMContentLoaded',init);