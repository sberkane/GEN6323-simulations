/* GEN6323 — robot différentiel, modèle continu pédagogique sans dépendance. */
(function () {
  'use strict';
  const modules = window.robotLabModules = window.robotLabModules || {};
  const P = { me:12, Je:.7, cv:1, cw:.08, r:.1, b:.45, kr:.6, ka:1.5,
    kpv:4, kiv:4, kpw:6, kiw:9, taw:.3, kx:1.5, ky:1.2, kth:2 };
  const defaults = { mission:'goal', realization:'PI', tauMax:1.4, duration:32,
    speed:.8, lookahead:.8, goalX:2, goalY:1, heading:90 };
  const wrap = a => Math.atan2(Math.sin(a),Math.cos(a));
  const clamp = (x,a,b) => Math.max(a,Math.min(b,x));
  function reference(t,T) {
    const k=2*Math.PI/T,s=k*t;
    const dx=3*k*Math.cos(s),dy=3.2*k*Math.cos(2*s);
    const ddx=-3*k*k*Math.sin(s),ddy=-6.4*k*k*Math.sin(2*s);
    return {x:3*Math.sin(s),y:1.6*Math.sin(2*s),theta:Math.atan2(dy,dx),
      v:Math.hypot(dx,dy),omega:(dx*ddy-dy*ddx)/(dx*dx+dy*dy)};
  }
  function command(t,y,p) {
    const x=y[0],z=y[1],th=y[2];
    if(p.mission==='goal'||p.mission==='pose') {
      const dx=p.goalX-x,dy=p.goalY-z,rho=Math.hypot(dx,dy);
      const alpha=rho<1e-9?0:wrap(Math.atan2(dy,dx)-th),headingError=wrap(p.heading*Math.PI/180-th);
      let vc=rho<1e-9?0:P.kr*rho*Math.cos(alpha),wc=rho<1e-9?0:P.ka*alpha+P.kr*Math.sin(alpha)*Math.cos(alpha);
      if(p.mission==='pose'&&p.phase!=='approach') {
        vc=0;wc=p.phase==='align'?P.ka*headingError:0;
      }
      return {vc,wc,rho,alpha,headingError,targetX:p.goalX,targetY:p.goalY,
        targetTheta:p.mission==='pose'?p.heading*Math.PI/180:null,phase:p.phase||'approach'};
    }
    if(p.mission==='trajectory') {
      const ref=reference(t,p.duration),dx=ref.x-x,dy=ref.y-z;
      const ex=Math.cos(th)*dx+Math.sin(th)*dy,ey=-Math.sin(th)*dx+Math.cos(th)*dy;
      const eth=wrap(ref.theta-th);
      return {vc:ref.v*Math.cos(eth)+P.kx*ex,
        wc:ref.omega+P.ky*ref.v*ey+P.kth*Math.sin(eth),
        rho:Math.hypot(dx,dy),alpha:eth,targetX:ref.x,targetY:ref.y,targetTheta:ref.theta};
    }
    // Chemin circulaire : projection radiale, puis cible à L mètres d'arc en avant.
    // La courbure utilise la distance réelle au point visé (corde), pas L d'arc.
    const radius=2,phi=Math.atan2(z,x),aim=phi+p.lookahead/radius;
    const tx=radius*Math.cos(aim),ty=radius*Math.sin(aim);
    const dx=tx-x,dy=ty-z,chord=Math.hypot(dx,dy),alpha=wrap(Math.atan2(dy,dx)-th);
    return {vc:p.speed,wc:chord>1e-8?2*p.speed*Math.sin(alpha)/chord:0,
      rho:Math.abs(Math.hypot(x,z)-radius),alpha,targetX:tx,targetY:ty};
  }
  function signals(t,y,p) {
    const c=command(t,y,p);
    if(p.realization==='ideal') return Object.assign(c,{v:c.vc,omega:c.wc,
      tauL:null,tauR:null,tauLstar:null,tauRstar:null,F:null,M:null,Fstar:null,Mstar:null,saturated:false});
    const ki=p.realization==='PI'?P.kiv:0,kiw=p.realization==='PI'?P.kiw:0;
    const Fstar=P.cv*y[3]+P.me*(P.kpv*(c.vc-y[3])+ki*y[5]);
    const Mstar=P.cw*y[4]+P.Je*(P.kpw*(c.wc-y[4])+kiw*y[6]);
    const tauLstar=P.r*Fstar/2-P.r*Mstar/P.b,tauRstar=P.r*Fstar/2+P.r*Mstar/P.b;
    const tauL=clamp(tauLstar,-p.tauMax,p.tauMax),tauR=clamp(tauRstar,-p.tauMax,p.tauMax);
    return Object.assign(c,{v:y[3],omega:y[4],tauL,tauR,tauLstar,tauRstar,Fstar,Mstar,
      F:(tauL+tauR)/P.r,M:P.b*(tauR-tauL)/(2*P.r),
      saturated:Math.abs(tauL-tauLstar)>1e-9||Math.abs(tauR-tauRstar)>1e-9});
  }
  function rhs(t,y,p) {
    const s=signals(t,y,p);
    if(p.realization==='ideal') return [s.vc*Math.cos(y[2]),s.vc*Math.sin(y[2]),s.wc,0,0,0,0];
    return [y[3]*Math.cos(y[2]),y[3]*Math.sin(y[2]),y[4],(s.F-P.cv*y[3])/P.me,(s.M-P.cw*y[4])/P.Je,
      p.realization==='PI'?s.vc-y[3]+(s.F-s.Fstar)/(P.me*P.kiv*P.taw):0,
      p.realization==='PI'?s.wc-y[4]+(s.M-s.Mstar)/(P.Je*P.kiw*P.taw):0];
  }
  function rk4(t,y,dt,p) {
    const a=rhs(t,y,p),b=rhs(t+dt/2,y.map((v,i)=>v+dt*a[i]/2),p);
    const c=rhs(t+dt/2,y.map((v,i)=>v+dt*b[i]/2),p),d=rhs(t+dt,y.map((v,i)=>v+dt*c[i]),p);
    return y.map((v,i)=>v+dt*(a[i]+2*b[i]+2*c[i]+d[i])/6);
  }
  function updatePhase(t,y,p,events) {
    if(p.mission!=='pose')return;
    const s=signals(t,y,p),before=p.phase;
    if(p.phase!=='approach'&&s.rho>.06)p.phase='approach';
    else if(p.phase==='approach'&&s.rho<=.03)p.phase='brake';
    else if(p.phase==='brake'&&Math.abs(s.v)<.015&&Math.abs(s.omega)<.03)p.phase='align';
    else if(p.phase==='align'&&Math.abs(s.headingError)<.03&&Math.abs(s.v)<.015&&Math.abs(s.omega)<.03)p.phase='hold';
    else if(p.phase==='hold'&&Math.abs(s.headingError)>.06)p.phase='align';
    if(before!==p.phase)events.push({t,from:before,to:p.phase,rho:s.rho,headingError:s.headingError});
  }
  function sanitize(params) {
    const p=Object.assign({},defaults,params||{});
    if(!['goal','pose','trajectory','path'].includes(p.mission))p.mission='goal';
    if(!['ideal','P','PI'].includes(p.realization))p.realization='PI';
    for(const [key,a,b] of [['tauMax',.2,3],['duration',8,40],['speed',.2,1.5],['lookahead',.3,2],['goalX',-3,3],['goalY',-2,2],['heading',-180,180]]) {
      p[key]=Number.isFinite(Number(p[key]))?clamp(Number(p[key]),a,b):defaults[key];
    }
    p.duration=Math.round(p.duration);
    return p;
  }
  function simulate(params) {
    const p=sanitize(params),T=p.mission==='goal'?12:p.mission==='pose'?24:p.mission==='trajectory'?p.duration:20;
    const dt=(p.mission==='goal'||p.mission==='pose')?.002:.005,stride=Math.round(.02/dt),steps=Math.round(T/dt);
    let y=[0,0,0,0,0,0,0];
    if(p.mission==='trajectory') {const r=reference(0,p.duration);y=[r.x,r.y,r.theta,r.v,r.omega,0,0];}
    if(p.mission==='path') y=[2.8,-.6,Math.PI/2,0,0,0,0];
    const runParams=Object.assign({},p,{phase:'approach'}),events=[],idealEvents=[];
    let yi=y.slice(),idealParams=Object.assign({},p,{realization:'ideal',phase:'approach'}),frames=[],saturatedTime=0,status='ok';
    const record=(t)=>{
      const s=signals(t,y,runParams);frames.push(Object.assign({t,x:y[0],y:y[1],theta:y[2],iv:y[5],iw:y[6],
        idealX:yi[0],idealY:yi[1],idealTheta:yi[2]},s));
    };
    for(let i=0;i<=steps;i++) {
      const t=i*dt;
      updatePhase(t,y,runParams,events);updatePhase(t,yi,idealParams,idealEvents);
      if(i%stride===0||i===steps)record(t);
      if(i===steps)break;
      if(signals(t,y,runParams).saturated)saturatedTime+=dt;
      const next=rk4(t,y,dt,runParams),nextIdeal=rk4(t,yi,dt,idealParams);
      if(!next.every(Number.isFinite)||!nextIdeal.every(Number.isFinite)||Math.hypot(next[0],next[1])>100||Math.abs(next[3])>30||Math.abs(next[4])>80) {
        status='stopped';break;
      }
      y=next;yi=nextIdeal;
    }
    const last=frames[frames.length-1],values=k=>frames.map(f=>f[k]);
    let ref=[];
    if(p.mission==='trajectory')for(let i=0;i<=250;i++)ref.push(reference(p.duration*i/250,p.duration));
    else if(p.mission==='path')for(let i=0;i<=250;i++){const a=2*Math.PI*i/250;ref.push({x:2*Math.cos(a),y:2*Math.sin(a)});}
    else ref=frames.map(f=>({x:f.idealX,y:f.idealY}));
    const errorLabel=(p.mission==='goal'||p.mission==='pose')?'Distance au but':p.mission==='trajectory'?'Écart à la référence':'Écart au cercle';
    const charts=[{title:'Vitesses de translation',yLabel:'m/s',series:[
      {name:'Consigne v_c',color:1,values:values('vc'),dashed:true},{name:'Vitesse v',color:2,values:values('v')}]},
      {title:'Vitesses de rotation',yLabel:'rad/s',series:[
      {name:'Consigne ω_c',color:1,values:values('wc'),dashed:true},{name:'Vitesse ω',color:2,values:values('omega')}]},
      {title:errorLabel,yLabel:'m',series:[{name:errorLabel,color:0,values:values('rho')}]}];
    if(p.realization!=='ideal')charts.push({title:'Couples appliqués aux roues',yLabel:'N m',series:[
      {name:'Gauche',color:0,values:values('tauL')},{name:'Droite',color:2,values:values('tauR')}]});
    if(p.mission==='pose')charts.push({title:'Orientation finale imposée',yLabel:'°',series:[
      {name:'Erreur de cap θ_g − θ',color:3,values:frames.map(f=>f.headingError*180/Math.PI)} ]});
    const metrics=[{label:(p.mission==='goal'||p.mission==='pose')?'Distance finale au but':errorLabel+' final',value:last.rho,unit:'m'},
      {label:'Temps avec saturation',value:saturatedTime,unit:'s'},
      {label:'Durée simulée',value:last.t,unit:'s'}];
    if(p.mission==='pose') {
      metrics.splice(1,0,{label:'Erreur de cap finale',value:last.headingError*180/Math.PI,unit:'°'});
      metrics.push({label:'Seuils de retour',value:'6 cm · 3,44°',unit:''});
    }
    const view=(p.mission==='goal'||p.mission==='pose')?{xmin:-3.55,xmax:3.55,ymin:-2.35,ymax:2.35}:
      p.mission==='trajectory'?{xmin:-3.6,xmax:3.6,ymin:-2.35,ymax:2.35}:{xmin:-3.3,xmax:3.3,ymin:-2.8,ymax:2.8};
    for(const f of frames) {
      view.xmin=Math.min(view.xmin,f.x-.35,f.idealX-.35);view.xmax=Math.max(view.xmax,f.x+.35,f.idealX+.35);
      view.ymin=Math.min(view.ymin,f.y-.35,f.idealY-.35);view.ymax=Math.max(view.ymax,f.y+.35,f.idealY+.35);
    }
    let note=p.mission==='goal'?'But fixe, marche arrière permise; aucun cap final imposé. Les vitesses du robot dynamique partent de zéro; le cas idéal impose directement les consignes.':
      p.mission==='pose'?'Pose visée : position ET orientation. Logique hybride : approche jusqu’à 3 cm, freinage, puis alignement lorsque |v|<0,015 m/s et |ω|<0,03 rad/s. Entrée en maintien lorsque l’erreur de cap est <0,03 rad (1,72°) et les vitesses sont faibles. Région de maintien plus large : retour à l’approche au-delà de 6 cm, reprise d’alignement au-delà de 0,06 rad (3,44°). Les intégrales conservées peuvent provoquer un petit déplacement après l’entrée en maintien; les métriques donnent les erreurs finales réelles. Réalisation à tolérances, sans garantie de pose exacte.':
      p.mission==='trajectory'?'Le point vert impose où être à chaque instant. Même huit, durée variable. Départ sur la référence avec ses vitesses initiales; loi de suivi de type Kanayama.':
      'Le point visé dépend de la position du robot, sans horaire de passage. Pure Pursuit sur un cercle de rayon 2 m; la distance d’anticipation règle l’avancement sur le cercle.';
    note+=' Modèle continu sans glissement ni bruit; masses, gains et limites illustratifs. PI avec rétro-calcul; ces essais ne prouvent pas la stabilité globale.';
    if(status==='stopped')note='Calcul arrêté : des états dépassent le domaine numérique retenu. Réduire la demande ou revoir les paramètres. '+note;
    return {frames,duration:last.t,charts,metrics,note,params:p,status,reference:ref,view,events,idealEvents,
      integration:{method:'RK4',dt,samplePeriod:.02},physicalParams:Object.assign({},P),
      hybridTolerances:p.mission==='pose'?{approachDistance:.03,restartDistance:.06,entryAngle:.03,restartAngle:.06,speed:.015,angularSpeed:.03}:null};
  }
  function transform(w,h,view) {
    const left=55,right=24,top=24,bottom=43;
    const scale=Math.min((w-left-right)/(view.xmax-view.xmin),(h-top-bottom)/(view.ymax-view.ymin));
    const cx=(w+left-right)/2,cy=(h+top-bottom)/2,mx=(view.xmin+view.xmax)/2,my=(view.ymin+view.ymax)/2;
    return {scale,x:x=>cx+(x-mx)*scale,y:y=>cy-(y-my)*scale,ix:px=>mx+(px-cx)/scale,iy:py=>my-(py-cy)/scale};
  }
  function path(ctx,points,tr,color,width,dash) {
    if(!points.length)return;ctx.beginPath();points.forEach((p,i)=>i?ctx.lineTo(tr.x(p.x),tr.y(p.y)):ctx.moveTo(tr.x(p.x),tr.y(p.y)));
    ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash||[]);ctx.stroke();ctx.setLineDash([]);
  }
  function arrow(ctx,x,y,dx,dy,color) {
    const len=Math.hypot(dx,dy);if(len<.1)return;
    const ux=dx/len,uy=dy/len;ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+dx,y+dy);ctx.strokeStyle=color;ctx.lineWidth=2;ctx.stroke();
    ctx.beginPath();ctx.moveTo(x+dx,y+dy);ctx.lineTo(x+dx-8*ux+4*uy,y+dy-8*uy-4*ux);ctx.lineTo(x+dx-8*ux-4*uy,y+dy-8*uy+4*ux);ctx.closePath();ctx.fillStyle=color;ctx.fill();
  }
  function robot(ctx,f,tr,palette,ghost) {
    const scale=tr.scale,bodyLength=Math.max(22,.30*scale),bodyWidth=Math.max(15,.23*scale);
    ctx.save();ctx.translate(tr.x(f.x),tr.y(f.y));ctx.rotate(-f.theta);ctx.globalAlpha=ghost?.27:1;
    ctx.fillStyle=ghost?palette.series[0]:palette.bg;ctx.strokeStyle=ghost?palette.series[0]:palette.ink;ctx.lineWidth=2;
    ctx.fillRect(-bodyLength/2,-bodyWidth/2,bodyLength,bodyWidth);ctx.strokeRect(-bodyLength/2,-bodyWidth/2,bodyLength,bodyWidth);
    const wh=Math.max(5,bodyWidth*.24);
    ctx.fillStyle=palette.series[0];ctx.fillRect(-bodyLength*.30,-bodyWidth/2-wh,bodyLength*.60,wh);
    ctx.fillStyle=palette.series[2];ctx.fillRect(-bodyLength*.30,bodyWidth/2,bodyLength*.60,wh);
    arrow(ctx,0,0,bodyLength*.82,0,palette.ink);ctx.restore();
  }
  function draw(ctx,w,h,sim,index,palette) {
    const f=sim.frames[clamp(Math.round(index),0,sim.frames.length-1)],tr=transform(w,h,sim.view),v=sim.view;
    ctx.clearRect(0,0,w,h);ctx.fillStyle=palette.bg;ctx.fillRect(0,0,w,h);ctx.font='12px system-ui';ctx.textBaseline='middle';
    const span=Math.max(v.xmax-v.xmin,v.ymax-v.ymin),gridStep=span>35?10:span>18?5:span>10?2:1;
    for(let x=Math.ceil(v.xmin/gridStep)*gridStep;x<=v.xmax;x+=gridStep) {
      ctx.strokeStyle=palette.grid;ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(tr.x(x),tr.y(v.ymin));ctx.lineTo(tr.x(x),tr.y(v.ymax));ctx.stroke();
      ctx.fillStyle=palette.muted;ctx.textAlign='center';ctx.fillText(String(x),tr.x(x),h-25);
    }
    for(let y=Math.ceil(v.ymin/gridStep)*gridStep;y<=v.ymax;y+=gridStep) {
      ctx.strokeStyle=palette.grid;ctx.beginPath();ctx.moveTo(tr.x(v.xmin),tr.y(y));ctx.lineTo(tr.x(v.xmax),tr.y(y));ctx.stroke();
      ctx.fillStyle=palette.muted;ctx.textAlign='right';ctx.fillText(String(y),43,tr.y(y));
    }
    ctx.fillStyle=palette.muted;ctx.textAlign='right';ctx.fillText('x (m)',w-12,h-10);ctx.textAlign='left';ctx.fillText('y (m)',12,14);
    path(ctx,sim.reference,tr,palette.series[1],1.7,[5,5]);
    if(sim.params.realization!=='ideal')path(ctx,sim.frames.slice(0,index+1).map(q=>({x:q.idealX,y:q.idealY})),tr,palette.series[0],2,[4,4]);
    path(ctx,sim.frames.slice(0,index+1),tr,palette.series[2],3);
    if(sim.params.mission==='path')path(ctx,[{x:f.x,y:f.y},{x:f.targetX,y:f.targetY}],tr,palette.series[1],1.5,[4,4]);
    ctx.beginPath();ctx.arc(tr.x(f.targetX),tr.y(f.targetY),7,0,2*Math.PI);ctx.fillStyle=palette.series[1];ctx.fill();
    ctx.strokeStyle=palette.bg;ctx.lineWidth=2;ctx.stroke();
    if((sim.params.mission==='trajectory'||sim.params.mission==='pose')&&Number.isFinite(f.targetTheta))arrow(ctx,tr.x(f.targetX),tr.y(f.targetY),32*Math.cos(f.targetTheta),-32*Math.sin(f.targetTheta),palette.series[1]);
    if(sim.params.realization!=='ideal')robot(ctx,{x:f.idealX,y:f.idealY,theta:f.idealTheta},tr,palette,true);
    robot(ctx,f,tr,palette,false);
    ctx.font='600 12px system-ui';ctx.textAlign='left';ctx.fillStyle=f.saturated?palette.series[2]:palette.ink;
    const phaseNames={approach:'Approche',brake:'Freinage',align:'Alignement',hold:'Maintien'};
    ctx.fillText(sim.params.mission==='pose'?phaseNames[f.phase]+(f.saturated&&w>=500?' · saturation':''):
      f.saturated?'Couple limité':'t = '+f.t.toFixed(2)+' s',60,16);
    if(sim.params.mission==='goal'||sim.params.mission==='pose'){ctx.textAlign='right';ctx.fillStyle=palette.muted;ctx.font='11px system-ui';ctx.fillText(w<500?'Cliquer : nouveau but':'Cliquer pour déplacer le but',w-15,16);}
  }
  // root peut passer les dimensions si son canvas est responsive.
  function onCanvasClick(xFraction,yFraction,sim,p,dimensions) {
    if(p.mission!=='goal'&&p.mission!=='pose')return null;
    const w=dimensions&&dimensions.width||800,h=dimensions&&dimensions.height||420,tr=transform(w,h,sim.view);
    return Object.assign({},p,{goalX:clamp(tr.ix(xFraction*w),-3,3),goalY:clamp(tr.iy(yFraction*h),-2,2)});
  }
  modules.planar={id:'planar',title:'Position et trajectoires',defaults,controls:[
    {key:'mission',label:'Objectif',type:'select',options:[{value:'goal',label:'Position fixe · cap libre'},{value:'pose',label:'Pose fixe · cap imposé'},{value:'trajectory',label:'Suivre un huit chronométré'},{value:'path',label:'Suivre un cercle sans horaire'}]},
    {key:'realization',label:'Réalisation des vitesses',type:'select',options:[{value:'ideal',label:'Cinématique idéale'},{value:'P',label:'Robot dynamique · P'},{value:'PI',label:'Robot dynamique · PI'}]},
    {key:'tauMax',label:'Couple maximal par roue',type:'range',min:.2,max:3,step:.1,unit:'N m'},
    {key:'heading',label:'Orientation cible · pose',type:'range',min:-180,max:180,step:5,unit:'°'},
    {key:'duration',label:'Durée du huit',type:'range',min:8,max:40,step:1,unit:'s'},
    {key:'speed',label:'Vitesse sur le cercle',type:'range',min:.2,max:1.5,step:.1,unit:'m/s'},
    {key:'lookahead',label:'Anticipation sur le cercle',type:'range',min:.3,max:2,step:.1,unit:'m'}],
    simulate,draw,onCanvasClick};
})();
