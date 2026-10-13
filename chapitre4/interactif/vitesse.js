/* GEN6323 — P, PI, PID filtré, charge et anti-enroulement; autonome hors ligne. */
(function () {
  'use strict';
  const modules=window.robotLabModules=window.robotLabModules||{};
  const P={me:12,cv:1,r:.1,Tf:.05,Taw:.3,vc:1,loadTime:5};
  const defaults={controller:'PI',kp:4,ki:4,kd:.4,load:3,saturation:false,antiwindup:true,tauMax:.4};
  const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
  function sanitize(params){
    const p=Object.assign({},defaults,params||{});
    if(!['P','PI','PID'].includes(p.controller))p.controller='PI';
    for(const [k,a,b] of [['kp',.5,8],['ki',.5,12],['kd',0,1.5],['load',0,6],['tauMax',.1,1.6]])
      p[k]=Number.isFinite(Number(p[k]))?clamp(Number(p[k]),a,b):defaults[k];
    p.saturation=Boolean(p.saturation);p.antiwindup=Boolean(p.antiwindup);return p;
  }
  function signals(t,y,p,loadOverride){
    const v=y[0],I=y[1],z=y[2],ki=p.controller==='P'?0:p.ki,kd=p.controller==='PID'?p.kd:0;
    const error=P.vc-v,Fstar=P.cv*v+P.me*(p.kp*error+ki*I-kd*z);
    const limit=2*p.tauMax/P.r,F=p.saturation?clamp(Fstar,-limit,limit):Fstar;
    const load=loadOverride===undefined?(t>=P.loadTime?-p.load:0):loadOverride;
    const dv=(F-P.cv*v+load)/P.me;
    const dI=ki?error+((p.saturation&&p.antiwindup)?(F-Fstar)/(P.me*ki*P.Taw):0):0;
    return {v,I,z,x:y[3],vc:P.vc,error,Fstar,F,load,dv,dI,dz:kd?(dv-z)/P.Tf:0,
      integralForce:P.me*ki*I,derivativeForce:-P.me*kd*z,
      tauStar:P.r*Fstar/2,tau:P.r*F/2,saturated:Math.abs(F-Fstar)>1e-9};
  }
  function rhs(t,y,p,load){const s=signals(t,y,p,load);return[s.dv,s.dI,s.dz,s.v];}
  function rk4(t,y,dt,p){
    // Charge constante sur le pas; à t=5 s elle change pour le pas suivant.
    // Le dernier stade avant 5 s n'anticipe donc jamais le nouvel effort.
    const load=t>=P.loadTime?-p.load:0;
    const a=rhs(t,y,p,load),b=rhs(t+dt/2,y.map((v,i)=>v+dt*a[i]/2),p,load);
    const c=rhs(t+dt/2,y.map((v,i)=>v+dt*b[i]/2),p,load),d=rhs(t+dt,y.map((v,i)=>v+dt*c[i]),p,load);
    return y.map((v,i)=>v+dt*(a[i]+2*b[i]+2*c[i]+d[i])/6);
  }
  function simulate(params){
    const p=sanitize(params),dt=.002,T=20,stride=10,steps=Math.round(T/dt);
    const baselineParams=Object.assign({},p,{controller:'P'});
    let y=[0,0,0,0],baseline=y.slice(),frames=[],status='ok',satTime=0,peak=0;
    for(let i=0;i<=steps;i++){
      const t=i*dt,s=signals(t,y,p);peak=Math.max(peak,y[0]);
      if(i%stride===0||i===steps)frames.push(Object.assign({t,baselineV:baseline[0]},s));
      if(i===steps)break;
      if(s.saturated)satTime+=dt;
      const next=rk4(t,y,dt,p),nextP=rk4(t,baseline,dt,baselineParams);
      if(!next.every(Number.isFinite)||Math.abs(next[0])>30||Math.abs(next[1])>100||Math.abs(next[2])>500){status='stopped';break;}
      y=next;baseline=nextP;
    }
    const last=frames[frames.length-1],values=k=>frames.map(f=>f[k]);
    const speedSeries=[{name:'Consigne',color:1,values:values('vc'),dashed:true},
      {name:p.controller+' · vitesse réelle',color:2,values:values('v')}];
    if(p.controller!=='P')speedSeries.push({name:'P · mêmes conditions',color:0,values:values('baselineV'),dashed:true});
    const charts=[{title:'Vitesse du robot',yLabel:'m/s',series:speedSeries},
      {title:'Effort demandé et appliqué',yLabel:'N',series:[
        {name:'Demandé F_a*',color:0,values:values('Fstar'),dashed:true},
        {name:'Appliqué F_a',color:2,values:values('F')}]},
      {title:'Action intégrale',yLabel:'N',series:[
        {name:'Effort intégral',color:1,values:values('integralForce')},
        {name:'Charge à compenser',color:3,values:frames.map(f=>-f.load),dashed:true}]}];
    const metrics=[{label:'Erreur finale',value:last.error,unit:'m/s'},
      {label:'Dépassement maximal',value:Math.max(0,peak-1)*100,unit:'%'},
      {label:'Temps avec saturation',value:satTime,unit:'s'}];
    let note='Consigne de 1 m/s dès t=0, départ au repos. La charge résistante arrive à 5 s. Même robot de 12 kg et compensation du frottement connue; dérivée sur la mesure, filtrée avec T_f=0,05 s. Le PID ne garantit pas un meilleur transitoire. ';
    note+=p.saturation?'Les deux roues reçoivent le même couple limité; T_aw=0,30 s pour le rétro-calcul. ':'Les efforts ne sont pas limités dans cet essai. ';
    note+='Modèle continu, paramètres synthétiques, mesure sans bruit. La comparaison P utilise le même gain proportionnel et les mêmes contraintes.';
    if(p.controller==='P')note+='Les paramètres I, D et anti-enroulement sont inactifs pour P.';
    else if(p.controller==='PI')note+='Le gain D est inactif pour PI.';
    if(status==='stopped')note='Calcul arrêté : des états dépassent le domaine numérique retenu. Revoir les gains ou la demande. '+note;
    return{frames,duration:last.t,charts,metrics,note,params:p,status,
      physicalParams:Object.assign({},P),integration:{method:'RK4, charge segmentée à 5 s',dt,samplePeriod:.02}};
  }
  function arrow(ctx,x,y,dx,color,label,scale){
    const length=clamp(dx*scale,-155,155),direction=Math.sign(length)||1;
    ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=3;
    ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+length,y);ctx.stroke();
    ctx.beginPath();ctx.moveTo(x+length,y);ctx.lineTo(x+length-10*direction,y-5);ctx.lineTo(x+length-10*direction,y+5);ctx.closePath();ctx.fill();
    ctx.font='600 12px system-ui';ctx.textAlign='center';ctx.fillText(label,x+length/2,y-13);
  }
  function draw(ctx,w,h,sim,index,palette){
    const f=sim.frames[clamp(Math.round(index),0,sim.frames.length-1)],x0=w*.49,y0=h*.47,scale=Math.min(100,w/7);
    ctx.clearRect(0,0,w,h);ctx.fillStyle=palette.bg;ctx.fillRect(0,0,w,h);
    const roadTop=y0-51,roadBottom=y0+51;
    ctx.fillStyle=palette.grid;ctx.globalAlpha=.33;ctx.fillRect(0,roadTop,w,102);ctx.globalAlpha=1;
    ctx.lineWidth=1;ctx.strokeStyle=palette.grid;
    for(const yy of[roadTop,roadBottom]){ctx.beginPath();ctx.moveTo(0,yy);ctx.lineTo(w,yy);ctx.stroke();}
    // Le véhicule est centré; les repères du sol défilent selon sa vraie position.
    const xmin=f.x-x0/scale,xmax=f.x+(w-x0)/scale;
    ctx.font='12px system-ui';ctx.textAlign='center';ctx.textBaseline='middle';
    for(let m=Math.floor(xmin);m<=xmax+1;m++){
      const px=x0+(m-f.x)*scale;
      ctx.strokeStyle=palette.grid;ctx.setLineDash([5,8]);ctx.beginPath();ctx.moveTo(px,roadTop);ctx.lineTo(px,roadBottom);ctx.stroke();ctx.setLineDash([]);
      ctx.fillStyle=palette.muted;ctx.fillText(m+' m',px,roadBottom+19);
    }
    const len=75,width=48;
    ctx.fillStyle=palette.bg;ctx.strokeStyle=palette.ink;ctx.lineWidth=2.5;ctx.fillRect(x0-len/2,y0-width/2,len,width);ctx.strokeRect(x0-len/2,y0-width/2,len,width);
    ctx.fillStyle=palette.series[0];ctx.fillRect(x0-24,y0-width/2-11,48,11);
    ctx.fillStyle=palette.series[2];ctx.fillRect(x0-24,y0+width/2,48,11);
    ctx.fillStyle=palette.ink;ctx.beginPath();ctx.moveTo(x0+len/2+8,y0);ctx.lineTo(x0+len/2-5,y0-7);ctx.lineTo(x0+len/2-5,y0+7);ctx.closePath();ctx.fill();
    arrow(ctx,x0-len/2,y0-85,f.vc,palette.series[1],'v_c = 1,00 m/s',85);
    arrow(ctx,x0-len/2,y0-57,f.v,palette.series[2],'v = '+f.v.toFixed(2)+' m/s',85);
    const forceY=Math.min(h-65,roadBottom+70);
    arrow(ctx,x0,forceY,f.F,palette.series[0],'F_a = '+f.F.toFixed(1)+' N',7);
    if(f.load!==0)arrow(ctx,x0,forceY+31,f.load,palette.series[3],'charge = '+f.load.toFixed(1)+' N',18);
    ctx.font='600 12px system-ui';ctx.textAlign='left';ctx.fillStyle=f.saturated?palette.series[2]:palette.ink;
    ctx.fillText(f.saturated?(w<500?'Couples saturés':'Couples limités à ±'+sim.params.tauMax.toFixed(2)+' N m'):'t = '+f.t.toFixed(2)+' s',18,20);
    ctx.fillStyle=palette.muted;ctx.textAlign='right';ctx.font='12px system-ui';ctx.fillText((w<500?'x = ':'Distance parcourue : ')+f.x.toFixed(2)+' m',w-18,20);
    ctx.textAlign='center';ctx.fillText('Les repères du sol défilent avec le mouvement réel',w/2,h-15);
  }
  modules.vitesse={id:'vitesse',title:'Boucles de vitesse',defaults,controls:[
    {key:'controller',label:'Correcteur',type:'select',options:[{value:'P',label:'P'},{value:'PI',label:'PI'},{value:'PID',label:'PID filtré'}]},
    {key:'kp',label:'Gain proportionnel',type:'range',min:.5,max:8,step:.5,unit:'s⁻¹'},
    {key:'ki',label:'Gain intégral · PI/PID',type:'range',min:.5,max:12,step:.5,unit:'s⁻²'},
    {key:'kd',label:'Gain dérivé · PID',type:'range',min:0,max:1.5,step:.1,unit:''},
    {key:'load',label:'Charge résistante à 5 s',type:'range',min:0,max:6,step:.5,unit:'N'},
    {key:'saturation',label:'Limiter les couples',type:'checkbox'},
    {key:'tauMax',label:'Couple maximal par roue',type:'range',min:.1,max:1.6,step:.1,unit:'N m'},
    {key:'antiwindup',label:'Anti-enroulement par rétro-calcul',type:'checkbox'}],
    simulate,draw};
})();
