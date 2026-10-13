(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const palette = {ink:'#172d46',muted:'#657486',grid:'#e5eaf0',bg:'#ffffff',series:['#245cd3','#00897e','#db6537','#aa7920','#8454b0']};
  const copy = {
    planar:{title:'Robot différentiel',tag:'Séances 2 et 3',help:'Cliquez dans la scène : choisissez un but et lancez le mouvement.',lesson:'La boucle externe transforme la mission en vitesses demandées. La boucle interne produit les couples nécessaires; le robot peut prendre du retard lorsque ces couples sont limités.',question:'Comparez la réalisation idéale et les boucles P puis PI. Une erreur faible sur le chemin signifie-t-elle que le robot est à l’heure ?'},
    vitesse:{title:'Boucle de vitesse',tag:'Séance 2',help:'La charge résistante apparaît après 5 secondes.',lesson:'Le terme P réagit à l’erreur actuelle. L’intégrale conserve l’effort nécessaire contre une charge constante. La dérivée agit sur la mesure filtrée; son effet dépend du réglage.',question:'Limitez les couples, puis désactivez l’anti-enroulement. Pourquoi le robot continue-t-il à accélérer alors qu’il a atteint la consigne ?'},
    quad:{title:'Quadrirotor dans le plan vertical',tag:'Séance 4',help:'But fixe : cliquez dans la scène pour lancer un déplacement.',lesson:'Pour accélérer horizontalement, le quadrirotor incline sa poussée. Il doit aussi adapter son intensité pour soutenir son poids. L’orientation réelle peut suivre la demande avec retard.',question:'Réduisez l’inclinaison ou la poussée disponible. Qu’observez-vous sur le trajet et sur l’erreur de position ?'}
  };
  let active, module, params, sim, index=0, playing=false, playTime=0, lastTime=0, charts=[], computeTimer;
  const saved = {};
  // Only these routes are interpreted. A URL never supplies model parameters.
  const routes = new Map([
    ['#planar',{id:'planar',mission:'goal'}],
    ['#planar/goal',{id:'planar',mission:'goal'}],
    ['#planar/pose',{id:'planar',mission:'pose'}],
    ['#planar/trajectory',{id:'planar',mission:'trajectory'}],
    ['#planar/path',{id:'planar',mission:'path'}],
    ['#vitesse',{id:'vitesse'}],
    ['#quad',{id:'quad',mission:'goal'}],
    ['#quad/goal',{id:'quad',mission:'goal'}],
    ['#quad/trajectory',{id:'quad',mission:'circle'}],
    ['#quad/circle',{id:'quad',mission:'circle'}]
  ]);
  function readRoute(hash){return routes.get(hash)||routes.get('#planar');}
  function routeHash(id,mission){
    if(id==='vitesse')return '#vitesse';
    if(id==='quad')return mission==='circle'?'#quad/trajectory':'#quad';
    return ['pose','trajectory','path'].includes(mission)?'#planar/'+mission:'#planar';
  }
  function syncHash(mode='push'){
    const hash=routeHash(active,params?.mission);
    if(window.location.hash===hash)return;
    // pushState does not trigger hashchange, so UI choices compute only once.
    // The fallback supports local-file browsers with restricted History APIs.
    try{window.history[mode==='replace'?'replaceState':'pushState'](null,'',hash);}
    catch(_){if(mode==='replace')window.location.replace(hash);else window.location.hash=hash;}
  }
  function followHash(){
    const route=readRoute(window.location.hash);
    select(route.id,route.mission,false);
    syncHash('replace'); // Canonicalise aliases/unknown links without a new entry.
  }
  const format = (n,d=2) => Number.isFinite(Number(n)) ? Number(n).toLocaleString('fr-FR',{maximumFractionDigits:d}) : String(n);
  const node = (tag, cls, text) => {const n=document.createElement(tag); if(cls)n.className=cls;if(text!==undefined)n.textContent=text;return n;};
  function canvasContext(canvas){ const r=canvas.getBoundingClientRect(), d=Math.min(window.devicePixelRatio||1,2); const w=Math.max(1,r.width),h=Math.max(1,r.height); if(canvas.width!==Math.round(w*d)||canvas.height!==Math.round(h*d)){canvas.width=Math.round(w*d);canvas.height=Math.round(h*d);}const c=canvas.getContext('2d');c.setTransform(d,0,0,d,0,0);c.clearRect(0,0,w,h);return {c,w,h}; }
  function setPlaying(value){playing=value;lastTime=0;$('play').textContent=playing?'Ⅱ Pause':'▶ Lire';$('play').setAttribute('aria-label',playing?'Mettre en pause':'Lire l’animation');}
  function controls(){
    const host=$('controls');host.replaceChildren(node('h2','','Paramètres'),node('p','','Changez une valeur pour relancer l’essai.'));
    for(const spec of module.controls){
      if(active==='planar' && ((spec.key==='duration'&&params.mission!=='trajectory') || (['speed','lookahead'].includes(spec.key)&&params.mission!=='path') || (spec.key==='heading'&&params.mission!=='pose') || (spec.key==='tauMax'&&params.realization==='ideal')))continue;
      if(active==='vitesse' && ((spec.key==='ki'&&params.controller==='P') || (spec.key==='kd'&&params.controller!=='PID') || (spec.key==='tauMax'&&!params.saturation) || (spec.key==='antiwindup'&&(!params.saturation||params.controller==='P'))))continue;
      const row=node('div','control'+(spec.type==='checkbox'?' check':'')), label=node('label','',spec.label), id='param-'+spec.key; label.htmlFor=id;
      const input=node(spec.type==='select'?'select':'input');input.id=id; input.name=spec.key;
      if(spec.type==='select'){for(const o of spec.options){const op=node('option','',o.label);op.value=o.value;input.append(op);}input.value=params[spec.key];}
      else if(spec.type==='checkbox'){input.type='checkbox';input.checked=Boolean(params[spec.key]);}
      else{input.type='range';input.min=spec.min;input.max=spec.max;input.step=spec.step;input.value=params[spec.key];const out=node('output');out.htmlFor=id;out.textContent=format(params[spec.key],3)+(spec.unit?' '+spec.unit:'');label.append(out);input.addEventListener('input',()=>{out.textContent=format(input.value,3)+(spec.unit?' '+spec.unit:'');});}
      input.addEventListener(spec.type==='range'?'input':'change',()=>{params[spec.key]=spec.type==='checkbox'?input.checked:spec.type==='range'?Number(input.value):input.value;if(spec.key==='mission')syncHash();setPlaying(false);clearTimeout(computeTimer);if(spec.type!=='range'){controls();document.getElementById(id)?.focus();}computeTimer=setTimeout(recompute,60);});
      if(spec.type==='checkbox')row.append(input,label);else row.append(label,input);host.append(row);
    }
    const reset=node('button','light reset','Rétablir les paramètres');reset.addEventListener('click',()=>{params={...module.defaults};saved[active]=params;syncHash();controls();recompute();});host.append(reset);
  }
  function chartUI(){
    $('charts').replaceChildren();$('charts').classList.toggle('four',sim.charts.length===4);$('charts').classList.toggle('five',sim.charts.length===5);charts=[];
    for(const spec of sim.charts){
      const box=node('section','card chart-card'), title=node('h3','',spec.title),canvas=node('canvas','chart'),legend=node('div','legend');canvas.setAttribute('aria-label',spec.title+' en fonction du temps');canvas.setAttribute('role','img');
      for(const s of spec.series){const item=node('span'), line=node('i');line.style.borderColor=palette.series[s.color??0];if(s.dashed)line.style.borderTopStyle='dashed';item.append(line,document.createTextNode(s.name));legend.append(item);}
      box.append(title,canvas,legend);$('charts').append(box);
      let ymin=Infinity,ymax=-Infinity;for(const s of spec.series)for(const v of s.values){if(Number.isFinite(v)){ymin=Math.min(ymin,v);ymax=Math.max(ymax,v);}}
      if(!Number.isFinite(ymin)){ymin=-1;ymax=1;}const pad=Math.max((ymax-ymin)*.12,Math.max(Math.abs(ymax),Math.abs(ymin))*.04,.04);ymin-=pad;ymax+=pad;charts.push({spec,canvas,ymin,ymax});
    }
  }
  function recompute(){
    clearTimeout(computeTimer);setPlaying(false);index=0;playTime=0;$('error').hidden=true;
    try{sim=module.simulate({...params});if(!sim.frames?.length)throw Error('L’essai ne contient aucune mesure.');
      if(active==='planar'){
        const lessons={goal:['Le point cible fixe seulement la position. Le cap final reste libre. La courbe en pointillés montre la réalisation cinématique idéale.','Passez du modèle idéal au robot dynamique. Le même objectif et la même loi produisent-ils exactement le même trajet ?'],pose:['La pose cible impose aussi un cap. La commande alterne approche, freinage, alignement et maintien; les seuils évitent les changements incessants de mode.','Comparez les erreurs de position et de cap. Le maintien dans une tolérance est-il identique à une convergence exacte ?'],trajectory:['La référence donne une position pour chaque instant. Le robot doit suivre le mouvement et son horaire.','Réduisez la durée du huit : de combien augmentent les efforts et le retard ?'],path:['Le cercle décrit un chemin sans horaire. La cible visée avance à partir de la projection du robot, selon la distance d’anticipation.','Changez l’anticipation et la vitesse. Comment évoluent la correction latérale et les couples nécessaires ?']};
        const text=lessons[params.mission]||lessons.goal;$('lesson').textContent=text[0];$('question').textContent=text[1];
        $('scene-help').textContent=['goal','pose'].includes(params.mission)?'Cliquez pour choisir le but et lancer le mouvement.':'Réglez les paramètres, puis lancez la lecture.';
      }
      if(active==='quad')$('scene-help').textContent=params.mission==='goal'?'Cliquez dans la scène pour lancer un déplacement.':'Une position à rejoindre à chaque instant : suivez le point bleu.';
      $('time').max=sim.frames.length-1;$('time').value=0;$('note').textContent=sim.note||'';
      $('metrics').replaceChildren(...sim.metrics.map(m=>{const el=node('div','metric');let value=m.value,unit=m.unit||'';
        if(typeof value==='number'&&unit==='m'&&Math.abs(value)>0&&Math.abs(value)<.01){if(Math.abs(value)<.00001){value*=1e6;unit='µm';}else{value*=1000;unit='mm';}}
        const shown=typeof value==='number'?(value!==0&&Math.abs(value)<.0005?'≈ 0':format(value,3)):value;
        el.append(node('span','',m.label),node('strong','',shown),node('em','',unit));return el;}));
      if(sim.status==='stopped' || sim.stopped || sim.error){$('error').textContent='Fin du calcul à '+format(sim.frames.at(-1).t,2)+' s. '+(sim.error||sim.stoppedReason||'L’essai a été interrompu lorsque le modèle a quitté son domaine de validité.');$('error').hidden=false;}
      chartUI();draw();
    }catch(e){sim=null;$('error').textContent='Cet essai ne peut pas être affiché. '+e.message;$('error').hidden=false;console.error(e);}
  }
  function drawChart(entry){
    const {c,w,h}=canvasContext(entry.canvas),{spec,ymin,ymax}=entry, L=49,R=12,T=23,B=31,W=w-L-R,H=h-T-B,duration=sim.frames.at(-1).t||1;
    const X=t=>L+W*t/duration,Y=v=>T+H*(ymax-v)/(ymax-ymin);c.font='10px -apple-system,Segoe UI,sans-serif';c.fillStyle=palette.muted;c.fillText(spec.yLabel||'',L,12);
    for(let j=0;j<=4;j++){const v=ymin+(ymax-ymin)*j/4,y=Y(v);c.strokeStyle=palette.grid;c.lineWidth=1;c.beginPath();c.moveTo(L,y);c.lineTo(w-R,y);c.stroke();c.textAlign='right';c.fillText(format(v,Math.abs(ymax-ymin)<1?2:1),L-7,y+3);}
    for(let j=0;j<=4;j++){const t=duration*j/4,x=X(t);c.textAlign='center';c.fillText(format(t,1),x,h-12);}c.textAlign='right';c.fillText('t (s)',w-R,12);
    c.save();c.beginPath();c.rect(L,T,W,H);c.clip();
    for(const s of spec.series){c.strokeStyle=palette.series[s.color??0];c.lineWidth=1.7;c.setLineDash(s.dashed?[5,4]:[]);c.beginPath();let begun=false;s.values.forEach((v,i)=>{if(!Number.isFinite(v)){begun=false;return;}const x=X(sim.frames[i].t),y=Y(v);if(!begun)c.moveTo(x,y);else c.lineTo(x,y);begun=true;});c.stroke();}
    c.setLineDash([]);const x=X(sim.frames[index].t);c.strokeStyle='#172d4655';c.lineWidth=1;c.beginPath();c.moveTo(x,T);c.lineTo(x,T+H);c.stroke();
    for(const s of spec.series){const v=s.values[index];if(!Number.isFinite(v))continue;c.fillStyle=palette.series[s.color??0];c.beginPath();c.arc(x,Y(v),3.2,0,Math.PI*2);c.fill();}c.restore();c.textAlign='left';
  }
  function draw(){if(!sim)return;const {c,w,h}=canvasContext($('stage'));c.save();module.draw(c,w,h,sim,index,palette);c.restore();charts.forEach(drawChart);$('clock').textContent=format(sim.frames[index].t,1)+' / '+format(sim.frames.at(-1).t,1)+' s';$('time').value=index;}
  function select(id,requestedMission,updateHash=true){
    if(!['planar','vitesse','quad'].includes(id)||!window.robotLabModules[id])return;
    const nextModule=window.robotLabModules[id],nextParams=saved[id]||{...nextModule.defaults};
    const nextMission=requestedMission===undefined?nextParams.mission:requestedMission;
    // Selecting the route already shown must preserve playback and the cursor.
    if(active===id&&params?.mission===nextMission){if(updateHash)syncHash();return;}
    clearTimeout(computeTimer);active=id;module=nextModule;params=nextParams;
    if(requestedMission!==undefined)params.mission=requestedMission;
    saved[id]=params;document.querySelectorAll('.tab').forEach(t=>{t.setAttribute('aria-selected',String(t.dataset.module===id));t.tabIndex=t.dataset.module===id?0:-1;});$('lab').setAttribute('aria-labelledby','tab-'+id);const text=copy[id];$('scene-title').textContent=text.title;$('scene-help').textContent=text.help;$('scene-tag').textContent=text.tag;$('lesson').textContent=text.lesson;$('question').textContent=text.question;controls();recompute();if(updateHash)syncHash();}
  document.querySelectorAll('.tab').forEach(t=>{t.addEventListener('click',()=>select(t.dataset.module));t.addEventListener('keydown',e=>{if(['ArrowLeft','ArrowRight','Home','End'].includes(e.key)){e.preventDefault();const ids=['planar','vitesse','quad'];let n=ids.indexOf(active);n=e.key==='Home'?0:e.key==='End'?2:(n+(e.key==='ArrowRight'?1:2))%3;select(ids[n]);$('tab-'+ids[n]).focus();}});});
  $('play').addEventListener('click',()=>{if(!sim)return;if(index>=sim.frames.length-1){index=0;playTime=0;}setPlaying(!playing);draw();});
  $('restart').addEventListener('click',()=>{setPlaying(false);index=0;playTime=0;draw();});
  $('time').addEventListener('input',()=>{if(!sim)return;setPlaying(false);index=Number($('time').value);playTime=sim.frames[index].t;draw();});
  $('stage').addEventListener('click',e=>{if(!module.onCanvasClick||!sim)return;const r=$('stage').getBoundingClientRect();const next=module.onCanvasClick((e.clientX-r.left)/r.width,(e.clientY-r.top)/r.height,sim,{...params},{width:r.width,height:r.height});if(next){params={...params,...next};saved[active]=params;syncHash();controls();recompute();setPlaying(true);}});
  $('csv').addEventListener('click',()=>{if(!sim)return;const series=sim.charts.flatMap(c=>c.series.map(s=>({name:c.title+' / '+s.name+' ['+(c.yLabel||'')+']',values:s.values})));const escape=s=>'"'+String(s).replaceAll('"','""')+'"';const rows=[['t (s)',...series.map(s=>s.name)].map(escape).join(';')];sim.frames.forEach((f,i)=>rows.push([f.t,...series.map(s=>s.values[i])].map(v=>Number.isFinite(v)?String(v).replace('.',','):'').join(';')));const url=URL.createObjectURL(new Blob(['\ufeff'+rows.join('\n')],{type:'text/csv;charset=utf-8'}));const a=document.createElement('a');a.href=url;a.download='GEN6323_'+active+'_mesures.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
  function tick(timestamp){if(playing&&sim){if(lastTime)playTime+=Math.min((timestamp-lastTime)/1000,.1)*Number($('rate').value);lastTime=timestamp;while(index<sim.frames.length-1&&sim.frames[index+1].t<=playTime)index++;if(playTime>=sim.frames.at(-1).t)setPlaying(false);draw();}requestAnimationFrame(tick);}
  new ResizeObserver(draw).observe($('stage'));window.addEventListener('resize',draw);
  window.addEventListener('hashchange',followHash);
  window.addEventListener('popstate',followHash);
  followHash();requestAnimationFrame(tick);
  window.robotLabState=()=>({active,hash:window.location.hash,params:{...params},index,playing,duration:sim?.duration,frames:sim?.frames.length});
})();
