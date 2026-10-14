(function (root) {
  'use strict';

  const C = {ink:'#102a43',muted:'#607689',grid:'#e2eaee',blue:'#1f5a94',teal:'#168c8c',orange:'#d95d39',gold:'#deb24c',reference:'#8296aa'};
  const clamp = (x,a,b) => Math.min(b,Math.max(a,x));
  const wrap = x => Math.atan2(Math.sin(x),Math.cos(x));
  const number = (x,digits=2) => Number.isFinite(x) ? x.toLocaleString('fr-CA',{minimumFractionDigits:digits,maximumFractionDigits:digits}) : '—';
  const compact = x => Math.abs(x)<1e-10 ? '0' : Number(x.toPrecision(3)).toLocaleString('fr-CA',{maximumFractionDigits:4});
  const errorText = x => x<1e-9 ? '≈ 0' : number(x,x<.1?4:3);
  function sampleIndex(t,time){
    let lo=0,hi=t.length-1;
    while(lo<hi){const mid=Math.floor((lo+hi+1)/2); if(t[mid]<=time+1e-9)lo=mid;else hi=mid-1;}
    return lo;
  }
  function stopTime(experiment,scenario){
    return scenario.stop && Number.isFinite(scenario.stop.t) ? clamp(scenario.stop.t,0,experiment.duration) : experiment.duration;
  }
  function validateDataset(data,isQuad=false){
    if(!data||!Array.isArray(data.experiments)||!data.experiments.length)throw new Error('Les données de cette expérience ne sont pas disponibles.');
    const dims=isQuad?6:5;
    for(const e of data.experiments){
      if(!Number.isFinite(e.duration)||e.duration<=0||!Array.isArray(e.t)||e.t.length<2||Math.abs(e.t[0])>1e-9)throw new Error('L’échelle de temps de l’expérience est invalide.');
      if(e.t.some((x,i)=>!Number.isFinite(x)||(i>0&&x<=e.t[i-1])))throw new Error('Les instants de l’expérience doivent être croissants.');
      if(!e.reference||!Array.isArray(e.cases)||!e.cases.length)throw new Error('La référence ou les situations sont absentes.');
      for(const series of [e.reference,...e.cases]){
        for(const [key,size] of [['state',dims],['input',2]]){
          if(!Array.isArray(series[key])||series[key].length!==e.t.length||series[key].some(row=>!Array.isArray(row)||row.length!==size||row.some(x=>!Number.isFinite(x))))throw new Error('Un tableau d’états ou d’efforts est incomplet.');
        }
      }
      for(const scenario of e.cases){
        if(scenario.stop&&(!Number.isFinite(scenario.stop.t)||typeof scenario.stop.reason!=='string'))throw new Error('L’arrêt de simulation doit fournir un instant et une raison.');
      }
    }
    return true;
  }
  function positionErrors(e,c){return e.t.map((_,i)=>Math.hypot(c.state[i][0]-e.reference.state[i][0],c.state[i][1]-e.reference.state[i][1]));}
  function sceneBounds(e,c,isQuad){
    const last=sampleIndex(e.t,stopTime(e,c));
    let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity;
    for(const rows of [e.reference.state,c.state.slice(0,last+1)])for(const s of rows){xmin=Math.min(xmin,s[0]);xmax=Math.max(xmax,s[0]);ymin=Math.min(ymin,s[1]);ymax=Math.max(ymax,s[1]);}
    const xmargin=Math.max(.55,(xmax-xmin)*.10),ymargin=Math.max(.55,(ymax-ymin)*.13);
    if(isQuad)ymin=0;
    return {xmin:xmin-xmargin,xmax:xmax+xmargin,ymin:isQuad?-.2:ymin-ymargin,ymax:ymax+(isQuad?Math.max(1,ymargin):ymargin)};
  }
  const core={validateDataset,sampleIndex,stopTime,positionErrors,sceneBounds,wrap};
  root.OpenLoopLabCore=core;
  if(typeof module!=='undefined'&&module.exports)module.exports=core;
  if(typeof document==='undefined')return;

  const $=id=>document.getElementById(id);
  const ui={model:'differentiel',data:null,experiment:null,scenario:null,time:0,playing:false,lastTimestamp:null,rate:1,errors:[],bounds:null,charts:[],frame:null};
  const datasets={differentiel:root.OPEN_LOOP_DIFF,drone:root.OPEN_LOOP_QUAD};
  const available={};
  for(const key of ['differentiel','drone']){
    try{validateDataset(datasets[key],key==='drone');available[key]=true;}
    catch(error){available[key]=false;const button=$('model-'+key);button.disabled=true;button.title=error.message;}
  }
  function option(value,label){const el=document.createElement('option');el.value=String(value);el.textContent=label;return el;}
  function text(id,value){$(id).textContent=value;}
  function metric(id,value,unit){const el=$(id);el.replaceChildren(document.createTextNode(value+' '));if(unit){const small=document.createElement('small');small.textContent=unit;el.append(small);}}
  function legend(id,entries){
    const el=$(id);el.replaceChildren();
    for(const [label,color,dashed] of entries){const span=document.createElement('span'),line=document.createElement('i');line.className='line-key';line.style.borderColor=color;if(dashed)line.style.borderTopStyle='dashed';span.append(line,document.createTextNode(label));el.append(span);}
  }
  function setPlaying(value){
    ui.playing=value;ui.lastTimestamp=null;
    text('play-label',value?'Pause':'Lire');text('play-icon',value?'Ⅱ':'▶');$('play').setAttribute('aria-label',value?'Mettre en pause':'Lire l’animation');
    if(value&&!ui.frame)ui.frame=requestAnimationFrame(tick);
  }
  function setModel(model,writeHash=false){
    if(!available[model]){
      const fallback=available.differentiel?'differentiel':available.drone?'drone':null;
      if(!fallback){$('load-error').hidden=false;text('load-error','Les données ne sont pas disponibles. Conservez index.html, app.js, styles.css, diff_data.js et quad_data.js dans le même dossier.');$('play').disabled=true;return;}
      if(model==='drone'||model==='differentiel'){
        $('load-error').hidden=false;text('load-error','Les données du robot demandé ne sont pas disponibles. L’autre expérience reste accessible.');
      }
      model=fallback;
    }else $('load-error').hidden=true;
    setPlaying(false);ui.model=model;ui.data=datasets[model];
    for(const key of ['differentiel','drone'])$('model-'+key).setAttribute('aria-pressed',String(key===model));
    $('duration').replaceChildren(...ui.data.experiments.map(e=>option(e.id,number(e.duration,0)+' s'+(ui.data.experiments.length>1?(e.duration===Math.max(...ui.data.durations)?' · lent':' · rapide'):' · manœuvre prescrite'))));
    $('duration').disabled=ui.data.experiments.length===1;
    text('duration-help',model==='drone'?'Une référence fixée, trois situations.':'Même chemin, durée différente.');
    text('scene-title',model==='drone'?'Déplacement et inclinaison':'Le trajet et le robot');
    text('scene-caption',model==='drone'?'Vue de côté · x horizontal, z vertical · mètres':'Vue de dessus · axes en mètres');
    text('speed-label',model==='drone'?'Vitesse horizontale':'Vitesse réelle');
    text('angle-label',model==='drone'?'Écart de tangage':'Écart de cap');
    text('chart-velocity-title',model==='drone'?'Altitude':'Vitesse d’avance');
    text('chart-velocity-unit',model==='drone'?'m':'m/s');
    text('chart-rotation-title',model==='drone'?'Inclinaison':'Vitesse de rotation');
    text('chart-rotation-unit',model==='drone'?'°':'rad/s');
    text('chart-input-title',model==='drone'?'Poussées des deux groupes':'Couples des roues');
    text('chart-input-unit',model==='drone'?'N':'N·m');
    text('peak-label',model==='drone'?'Poussée appliquée maximale':'Couple appliqué maximal');
    document.querySelector('.next-link').setAttribute('href','../interactif/index.html#'+(model==='drone'?'quad':'planar'));
    $('chart-velocity').setAttribute('aria-label',model==='drone'?'Altitude désirée et réelle en fonction du temps':'Vitesse désirée et réelle en fonction du temps');
    $('chart-rotation').setAttribute('aria-label',model==='drone'?'Tangage désiré et réel en fonction du temps':'Vitesse de rotation désirée et réelle en fonction du temps');
    $('scene').setAttribute('aria-label',model==='drone'?'Animation du quadrirotor dans le plan vertical, référence et mouvement réel':'Animation du robot différentiel dans le plan horizontal, référence et mouvement réel');
    const assumptions=$('assumptions');assumptions.replaceChildren();
    for(const label of ui.data.assumptions||[]){const li=document.createElement('li');li.textContent=label;assumptions.append(li);}
    const p=ui.data.parameters;
    const params=model==='drone'?['m = '+number(p.m,2)+' kg','Jᵧ = '+number(p.J_y,3)+' kg·m²','ℓ = '+number(p.ell,2)+' m','g = '+number(p.g,2)+' m/s²','Fmax = '+number(p.F_max,2)+' N par groupe']:['mₑ = '+number(p.m_e,1)+' kg','Jₑ = '+number(p.J_e,2)+' kg·m²','r = '+number(p.r,2)+' m','b = '+number(p.b,2)+' m','τmax = '+number(p.tau_max,2)+' N·m par roue'];
    $('parameter-list').replaceChildren(...params.map(label=>{const span=document.createElement('span');span.textContent=label;return span;}));
    setExperiment();
    if(writeHash&&location.hash!=='#'+model)location.hash=model;
  }
  function setExperiment(){
    const oldCase=ui.scenario&&ui.scenario.id;
    ui.experiment=ui.data.experiments.find(e=>e.id===$('duration').value)||ui.data.experiments[0];
    $('case').replaceChildren(...ui.experiment.cases.map(c=>option(c.id,c.label)));
    if(ui.experiment.cases.some(c=>c.id===oldCase))$('case').value=oldCase;
    setScenario();
  }
  function setScenario(){
    setPlaying(false);ui.time=0;
    ui.scenario=ui.experiment.cases.find(c=>c.id===$('case').value)||ui.experiment.cases[0];
    const e=ui.experiment,c=ui.scenario,isQuad=ui.model==='drone',metrics=c.metrics||{};
    ui.errors=positionErrors(e,c);ui.bounds=sceneBounds(e,c,isQuad);
    const end=stopTime(e,c);$('time').max=end;$('time').step=ui.data.sample_dt||.02;$('time').value=0;
    text('end-time','/ '+number(end,0));text('slider-end',number(end,end%1===0?0:2)+' s');
    text('case-description',c.description||'');
    $('case-badge').classList.toggle('warning',c.id!=='nominal');
    text('case-short',c.id==='nominal'?'Modèle exact':c.id==='limited'?'Efforts bornés':'Modèle imparfait');
    const observations=isQuad?{
      nominal:'La poussée compense le poids. Le moment des deux groupes produit l’inclinaison prévue; le mouvement réel rejoint la référence sans correction, dans ce modèle exact.',
      limited:'Limiter séparément les poussées modifie leur somme et leur différence : la translation et le tangage peuvent donc s’écarter de la référence.',
      model_error:'Le drone réel est plus lourd. Les mêmes poussées ont été calculées pour la masse nominale : l’altitude baisse, sans correction de l’écart.'
    }:{
      nominal:'Les couples anticipent l’inertie et les résistances. Avec le modèle exact et les bonnes conditions initiales, les deux parcours se superposent.',
      limited:e.duration>10?'À cette cadence, les couples prescrits restent sous la limite. La limitation ne modifie donc pas le mouvement. Essayez ensuite 8 s.':'Les couples prescrits dépassent par moments la capacité des roues. Les accélérations réalisées changent, puis la trajectoire s’écarte.',
      model_error:'L’inertie de translation réelle est plus grande de 10 %. Les couples restent ceux du modèle nominal : aucun terme ne corrige l’erreur de vitesse.'
    };
    text('observation',observations[c.id]||c.description||'');
    text('summary-scope',c.stop?'jusqu’à l’arrêt de la simulation':'sur toute la manœuvre');
    metric('rms-error',errorText(metrics.position_rmse_m),'m');
    metric('peak-input',number(isQuad?metrics.applied_peak_force_N:metrics.applied_peak_torque_Nm,2),isQuad?'N':'N·m');
    metric('saturated-time',number((metrics.saturation_fraction||0)*100,1),'%');
    buildCharts();render();
  }
  function buildCharts(){
    const e=ui.experiment,c=ui.scenario,isQuad=ui.model==='drone';
    const end=sampleIndex(e.t,stopTime(e,c));
    const rows=(source,col,factor=1,actual=false)=>source.map((r,i)=>actual&&i>end?null:r[col]*factor);
    const series=(label,values,color,dashed=false)=>({label,values,color,dashed});
    const reference=(col,factor=1)=>series('Référence',rows(e.reference.state,col,factor),C.reference,true);
    const actual=(col,factor=1)=>series('Réel',rows(c.state,col,factor,true),C.teal);
    const labels=isQuad?['F_L prescrit','F_L appliqué','F_R prescrit','F_R appliqué']:['τ_L prescrit','τ_L appliqué','τ_R prescrit','τ_R appliqué'];
    const inputs=[series(labels[0],rows(e.reference.input,0),C.blue,true),series(labels[1],rows(c.input,0,1,true),C.blue),series(labels[2],rows(e.reference.input,1),C.orange,true),series(labels[3],rows(c.input,1,1,true),C.orange)];
    ui.charts=[
      {id:'velocity',series:isQuad?[reference(1),actual(1)]:[reference(3),actual(3)]},
      {id:'rotation',series:isQuad?[reference(2,180/Math.PI),actual(2,180/Math.PI)]:[reference(4),actual(4)]},
      {id:'error',series:[series('Distance au même instant',ui.errors.map((v,i)=>i<=end?v:null),C.teal)],nonnegative:true},
      {id:'input',series:inputs,bounds:c.id==='limited'?(isQuad?[0,ui.data.parameters.F_max]:[-ui.data.parameters.tau_max,ui.data.parameters.tau_max]):[]}
    ];
    for(const chart of ui.charts){
      let ymin=Infinity,ymax=-Infinity;
      for(const s of chart.series)for(const y of s.values)if(y!==null){ymin=Math.min(ymin,y);ymax=Math.max(ymax,y);}
      for(const y of chart.bounds||[]){ymin=Math.min(ymin,y);ymax=Math.max(ymax,y);}
      if(chart.nonnegative){ymin=0;ymax=Math.max(ymax,.001);}
      if(ymax-ymin<1e-7){const pad=Math.max(.05,Math.abs(ymax)*.08);ymin-=pad;ymax+=pad;}
      const pad=(ymax-ymin)*.12;chart.ymin=chart.nonnegative?0:ymin-pad;chart.ymax=ymax+pad;
      const entries=chart.series.map(s=>[s.label,s.color,s.dashed]);
      if(chart.bounds&&chart.bounds.length)entries.push(['Limite',C.gold,true]);
      legend('legend-'+chart.id,entries);
    }
  }
  function canvasContext(canvas){
    const rect=canvas.getBoundingClientRect(),w=Math.max(100,rect.width),h=Math.max(80,rect.height),dpr=Math.min(root.devicePixelRatio||1,2);
    const pw=Math.round(w*dpr),ph=Math.round(h*dpr);
    if(canvas.width!==pw||canvas.height!==ph){canvas.width=pw;canvas.height=ph;}
    const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);return {ctx,w,h};
  }
  function niceStep(span,target=6){
    const rough=span/target,p=Math.pow(10,Math.floor(Math.log10(Math.max(rough,1e-12)))),r=rough/p;
    return (r<=1?1:r<=2?2:r<=5?5:10)*p;
  }
  function arrow(ctx,x1,y1,x2,y2,color,width=2,head=6){
    const a=Math.atan2(y2-y1,x2-x1);ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(x1,y1);ctx.lineTo(x2,y2);ctx.stroke();ctx.beginPath();ctx.moveTo(x2,y2);ctx.lineTo(x2-head*Math.cos(a-.5),y2-head*Math.sin(a-.5));ctx.lineTo(x2-head*Math.cos(a+.5),y2-head*Math.sin(a+.5));ctx.closePath();ctx.fill();
  }
  function roundedRect(ctx,x,y,w,h,r){
    r=Math.min(r,w/2,h/2);ctx.beginPath();ctx.moveTo(x+r,y);ctx.lineTo(x+w-r,y);ctx.quadraticCurveTo(x+w,y,x+w,y+r);ctx.lineTo(x+w,y+h-r);ctx.quadraticCurveTo(x+w,y+h,x+w-r,y+h);ctx.lineTo(x+r,y+h);ctx.quadraticCurveTo(x,y+h,x,y+h-r);ctx.lineTo(x,y+r);ctx.quadraticCurveTo(x,y,x+r,y);ctx.closePath();
  }
  function drawDifferential(ctx,x,y,theta,scale,ghost){
    const bodyL=clamp(.46*scale,27,52),bodyW=bodyL*.70;
    ctx.save();ctx.translate(x,y);ctx.rotate(-theta);ctx.globalAlpha=ghost?.48:1;
    if(ghost){ctx.strokeStyle=C.reference;ctx.lineWidth=1.5;ctx.setLineDash([4,3]);roundedRect(ctx,-bodyL/2,-bodyW/2,bodyL,bodyW,5);ctx.stroke();arrow(ctx,0,0,bodyL*.8,0,C.reference,1.3,4);}
    else{
      ctx.shadowColor='#102a4328';ctx.shadowBlur=7;ctx.shadowOffsetY=3;ctx.fillStyle=C.teal;roundedRect(ctx,-bodyL/2,-bodyW/2,bodyL,bodyW,6);ctx.fill();ctx.shadowBlur=0;ctx.shadowOffsetY=0;
      ctx.fillStyle=C.blue;roundedRect(ctx,-bodyL*.31,-bodyW*.70,bodyL*.62,bodyW*.22,2);ctx.fill();ctx.fillStyle=C.orange;roundedRect(ctx,-bodyL*.31,bodyW*.48,bodyL*.62,bodyW*.22,2);ctx.fill();
      ctx.fillStyle='#eaf6f4';ctx.beginPath();ctx.arc(0,0,bodyW*.19,0,Math.PI*2);ctx.fill();arrow(ctx,bodyL*.17,0,bodyL*.8,0,C.ink,1.7,5);
    }ctx.restore();
  }
  function drawQuad(ctx,x,y,theta,scale,ghost,input){
    const span=clamp(2*(ui.data.parameters.ell||.25)*scale,52,86),half=span/2;
    ctx.save();ctx.translate(x,y);ctx.rotate(theta);ctx.globalAlpha=ghost?.40:1;
    ctx.strokeStyle=ghost?C.reference:C.ink;ctx.lineWidth=ghost?1.4:3;ctx.setLineDash(ghost?[4,3]:[]);ctx.beginPath();ctx.moveTo(-half,0);ctx.lineTo(half,0);ctx.stroke();
    ctx.fillStyle=ghost?'#f0f4f7':C.teal;roundedRect(ctx,-span*.17,-span*.08,span*.34,span*.16,3);ctx.fill();ctx.stroke();
    for(const [side,index,color] of [[-1,0,C.blue],[1,1,C.orange]]){
      const rotorX=side*half;
      ctx.strokeStyle=ghost?C.reference:color;ctx.lineWidth=ghost?1.3:3;ctx.beginPath();ctx.moveTo(rotorX-span*.16,-6);ctx.lineTo(rotorX+span*.16,-6);ctx.moveTo(rotorX,-6);ctx.lineTo(rotorX,0);ctx.stroke();
      if(!ghost&&input){const f=input[index],length=clamp(f/(ui.data.parameters.m*ui.data.parameters.g/2)*24,0,55);if(length>1)arrow(ctx,rotorX,-9,rotorX,-9-length,color,1.7,5);}
    }
    ctx.setLineDash([]);if(!ghost){ctx.strokeStyle=C.ink;ctx.lineWidth=1.7;ctx.beginPath();ctx.moveTo(-span*.13,5);ctx.lineTo(-span*.20,12);ctx.moveTo(span*.13,5);ctx.lineTo(span*.20,12);ctx.stroke();}ctx.restore();
  }
  function drawScene(index){
    const {ctx,w,h}=canvasContext($('scene')),b=ui.bounds,e=ui.experiment,c=ui.scenario,isQuad=ui.model==='drone';
    const pad={l:43,r:22,t:27,b:29},pw=w-pad.l-pad.r,ph=h-pad.t-pad.b;
    const scale=Math.min(pw/(b.xmax-b.xmin),ph/(b.ymax-b.ymin));
    const xmin=(b.xmin+b.xmax)/2-pw/(2*scale),ymin=(b.ymin+b.ymax)/2-ph/(2*scale);
    const X=x=>pad.l+(x-xmin)*scale,Y=y=>h-pad.b-(y-ymin)*scale;
    const xmax=xmin+pw/scale,ymax=ymin+ph/scale;
    ctx.save();ctx.beginPath();ctx.rect(pad.l,pad.t,pw,ph);ctx.clip();
    if(isQuad){ctx.fillStyle='#ece9df';ctx.fillRect(pad.l,Y(0),pw,h);ctx.strokeStyle='#cbbf9f';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(pad.l,Y(0));ctx.lineTo(w-pad.r,Y(0));ctx.stroke();}
    ctx.strokeStyle=C.grid;ctx.lineWidth=.7;const step=niceStep(Math.max(pw/scale,ph/scale),w<500?6:9);
    for(let x=Math.ceil(xmin/step)*step;x<=xmax+1e-9;x+=step){ctx.beginPath();ctx.moveTo(X(x),pad.t);ctx.lineTo(X(x),h-pad.b);ctx.stroke();}
    for(let y=Math.ceil(ymin/step)*step;y<=ymax+1e-9;y+=step){ctx.beginPath();ctx.moveTo(pad.l,Y(y));ctx.lineTo(w-pad.r,Y(y));ctx.stroke();}
    const path=(rows,last,color,dashed,width)=>{ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dashed?[5,5]:[]);const stride=Math.max(1,Math.floor(last/1100));for(let i=0;i<=last;i+=stride){const p=rows[i];if(i===0)ctx.moveTo(X(p[0]),Y(p[1]));else ctx.lineTo(X(p[0]),Y(p[1]));}if(last>0)ctx.lineTo(X(rows[last][0]),Y(rows[last][1]));ctx.stroke();ctx.setLineDash([]);};
    path(e.reference.state,e.t.length-1,C.reference,true,1.7);
    path(c.state,index,C.teal,false,2.6);
    const start=e.reference.state[0];ctx.fillStyle='#fff';ctx.strokeStyle=C.reference;ctx.lineWidth=1.5;ctx.beginPath();ctx.arc(X(start[0]),Y(start[1]),4,0,Math.PI*2);ctx.fill();ctx.stroke();
    const ref=e.reference.state[index],state=c.state[index];
    if(ui.errors[index]>.02){ctx.strokeStyle='#b5c5ca';ctx.lineWidth=1;ctx.setLineDash([3,4]);ctx.beginPath();ctx.moveTo(X(ref[0]),Y(ref[1]));ctx.lineTo(X(state[0]),Y(state[1]));ctx.stroke();ctx.setLineDash([]);}
    if(isQuad){drawQuad(ctx,X(ref[0]),Y(ref[1]),ref[2],scale,true);drawQuad(ctx,X(state[0]),Y(state[1]),state[2],scale,false,c.input[index]);}
    else{drawDifferential(ctx,X(ref[0]),Y(ref[1]),ref[2],scale,true);drawDifferential(ctx,X(state[0]),Y(state[1]),state[2],scale,false);}
    ctx.restore();ctx.fillStyle=C.muted;ctx.font='9px system-ui,sans-serif';ctx.textAlign='center';ctx.textBaseline='top';
    for(let x=Math.ceil(xmin/step)*step;x<=xmax+1e-9;x+=step)ctx.fillText(compact(x),X(x),h-pad.b+8);
    ctx.textAlign='right';ctx.textBaseline='middle';for(let y=Math.ceil(ymin/step)*step;y<=ymax+1e-9;y+=step)ctx.fillText(compact(y),pad.l-9,Y(y));
    ctx.fillStyle=C.ink;ctx.font='bold 9px system-ui,sans-serif';ctx.textAlign='right';ctx.textBaseline='top';ctx.fillText('x (m)',w-pad.r,9);ctx.textAlign='left';ctx.fillText(isQuad?'z (m)':'y (m)',pad.l,9);
    if(isQuad){ctx.font='9px system-ui,sans-serif';ctx.fillStyle='#8a7a53';ctx.fillText('sol',w-pad.r-23,clamp(Y(0)+5,15,h-18));}
  }
  function drawChart(chart){
    const {ctx,w,h}=canvasContext($('chart-'+chart.id)),e=ui.experiment;
    const l=46,r=12,t=9,b=28,pw=w-l-r,ph=h-t-b;
    const X=x=>l+x/e.duration*pw,Y=y=>t+(chart.ymax-y)/(chart.ymax-chart.ymin)*ph;
    ctx.font='9px system-ui,sans-serif';ctx.textBaseline='middle';ctx.textAlign='right';
    const ys=niceStep(chart.ymax-chart.ymin,4);
    for(let y=Math.ceil(chart.ymin/ys)*ys;y<=chart.ymax+1e-10;y+=ys){ctx.strokeStyle=C.grid;ctx.lineWidth=.7;ctx.beginPath();ctx.moveTo(l,Y(y));ctx.lineTo(w-r,Y(y));ctx.stroke();ctx.fillStyle=C.muted;ctx.fillText(compact(y),l-7,Y(y));}
    const xticks=e.duration<=5?4:4;ctx.textAlign='center';ctx.textBaseline='top';
    for(let i=0;i<=xticks;i++){const time=e.duration*i/xticks,x=X(time);ctx.strokeStyle='#edf1f3';ctx.beginPath();ctx.moveTo(x,t);ctx.lineTo(x,h-b);ctx.stroke();ctx.fillStyle=C.muted;ctx.fillText(compact(time),x,h-b+8);}
    ctx.textAlign='right';ctx.font='8px system-ui,sans-serif';ctx.fillText('t (s)',w-r,h-10);
    ctx.save();ctx.beginPath();ctx.rect(l,t,pw,ph);ctx.clip();
    for(const y of chart.bounds||[]){ctx.strokeStyle=C.gold;ctx.lineWidth=1.1;ctx.setLineDash([3,4]);ctx.beginPath();ctx.moveTo(l,Y(y));ctx.lineTo(w-r,Y(y));ctx.stroke();}ctx.setLineDash([]);
    const stride=Math.max(1,Math.floor(e.t.length/700));
    for(const series of chart.series){ctx.strokeStyle=series.color;ctx.lineWidth=series.dashed?1.35:1.8;ctx.setLineDash(series.dashed?[4,4]:[]);ctx.beginPath();let started=false;for(let i=0;i<e.t.length;i+=stride){const y=series.values[i];if(y===null){started=false;continue;}if(!started){ctx.moveTo(X(e.t[i]),Y(y));started=true;}else ctx.lineTo(X(e.t[i]),Y(y));}const n=e.t.length-1;if(started&&series.values[n]!==null)ctx.lineTo(X(e.t[n]),Y(series.values[n]));ctx.stroke();}
    ctx.setLineDash([]);ctx.strokeStyle='#17354f75';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(X(ui.time),t);ctx.lineTo(X(ui.time),h-b);ctx.stroke();
    const idx=sampleIndex(e.t,ui.time);for(const series of chart.series)if(!series.dashed&&series.values[idx]!==null){ctx.fillStyle=series.color;ctx.strokeStyle='#fff';ctx.lineWidth=1.3;ctx.beginPath();ctx.arc(X(e.t[idx]),Y(series.values[idx]),3,0,Math.PI*2);ctx.fill();ctx.stroke();}
    ctx.restore();
  }
  function render(){
    if(!ui.experiment)return;
    const e=ui.experiment,c=ui.scenario,index=sampleIndex(e.t,ui.time),s=c.state[index],ref=e.reference.state[index];
    text('clock',number(ui.time,2));$('time').value=ui.time;
    metric('position-error',errorText(ui.errors[index]),'m');metric('actual-speed',number(s[3],3),'m/s');metric('angle-error',number(wrap(s[2]-ref[2])*180/Math.PI,1),'°');
    const stopped=!!c.stop&&ui.time>=stopTime(e,c)-.001;
    $('stop-message').hidden=!stopped;if(stopped)text('stop-message','Simulation arrêtée à '+number(c.stop.t,2)+' s : '+c.stop.reason+' La suite du mouvement n’est pas simulée.');
    $('time').setAttribute('aria-valuetext',number(ui.time,2)+' secondes');
    drawScene(index);for(const chart of ui.charts)drawChart(chart);
  }
  function tick(timestamp){
    ui.frame=null;if(!ui.playing)return;
    if(ui.lastTimestamp!==null)ui.time=Math.min(stopTime(ui.experiment,ui.scenario),ui.time+Math.min((timestamp-ui.lastTimestamp)/1000,.1)*ui.rate);
    ui.lastTimestamp=timestamp;render();
    if(ui.time>=stopTime(ui.experiment,ui.scenario)-1e-10)setPlaying(false);
    else ui.frame=requestAnimationFrame(tick);
  }
  for(const key of ['differentiel','drone'])$('model-'+key).addEventListener('click',()=>setModel(key,true));
  $('duration').addEventListener('change',setExperiment);$('case').addEventListener('change',setScenario);
  $('play').addEventListener('click',()=>{if(ui.time>=stopTime(ui.experiment,ui.scenario)-1e-10)ui.time=0;setPlaying(!ui.playing);render();});
  $('reset').addEventListener('click',()=>{setPlaying(false);ui.time=0;render();});
  $('time').addEventListener('input',()=>{setPlaying(false);ui.time=clamp(Number($('time').value),0,stopTime(ui.experiment,ui.scenario));render();});
  $('rate').addEventListener('change',()=>{ui.rate=Number($('rate').value);});
  root.addEventListener('hashchange',()=>{const model=location.hash.slice(1);if(model!==ui.model)setModel(model);});
  document.addEventListener('visibilitychange',()=>{ui.lastTimestamp=null;});
  if(typeof ResizeObserver!=='undefined'){const observer=new ResizeObserver(()=>render());observer.observe($('scene'));for(const id of ['velocity','rotation','error','input'])observer.observe($('chart-'+id));}
  else root.addEventListener('resize',render);
  setModel(location.hash.slice(1)||'differentiel');
})(typeof window!=='undefined'?window:globalThis);
