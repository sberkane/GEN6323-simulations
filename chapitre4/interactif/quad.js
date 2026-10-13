/* GEN6323 — coupe verticale x–z du quadrirotor, sans dépendance.
 * z vers le haut; theta > 0 incline +z_B vers +x.
 * Les deux poussées représentent des groupes équivalents, pas deux rotors
 * indépendants d'un véhicule 3D. Leurs limites sont appliquées avant la physique.
 */
(() => {
  'use strict';
  const G = 9.81, MASS = 1.5, J = 0.035, ARM = 0.25;
  const KV = 2.2, TORQUE_MAX = 0.8, DEG = Math.PI / 180;
  const DT = 0.0025, FRAME_DT = 0.020;
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const finite = (v, fallback) => Number.isFinite(Number(v)) ? Number(v) : fallback;
  const defaults = {mission:'goal', gravityComp:true, kpos:0.6, attitude:8,
    tiltMax:25, thrustMax:22, goalX:2.5, goalZ:3};
  function parameters(input) {
    const p = {...defaults, ...input};
    p.mission = p.mission === 'circle' ? 'circle' : 'goal';
    p.gravityComp = p.gravityComp !== false;
    p.kpos = clamp(finite(p.kpos, defaults.kpos), .25, 1.6);
    p.attitude = clamp(finite(p.attitude, defaults.attitude), 3, 12);
    p.tiltMax = clamp(finite(p.tiltMax, defaults.tiltMax), 5, 45);
    p.thrustMax = clamp(finite(p.thrustMax, defaults.thrustMax), 16, 30);
    p.goalX = clamp(finite(p.goalX, defaults.goalX), -4.5, 4.5);
    p.goalZ = clamp(finite(p.goalZ, defaults.goalZ), .5, 5.5);
    return p;
  }
  function reference(t, p) {
    if (p.mission === 'goal') return {x:p.goalX,z:p.goalZ,vx:0,vz:0,ax:0,az:0};
    const r = 1.2, nu = .45, a = nu * t;
    return {x:r*Math.cos(a),z:3+r*Math.sin(a),
      vx:-r*nu*Math.sin(a),vz:r*nu*Math.cos(a),
      ax:-r*nu*nu*Math.cos(a),az:-r*nu*nu*Math.sin(a)};
  }
  function allocation(fCommand, tauCommand, p) {
    const f = clamp(fCommand, 0, p.thrustMax);
    const tau = clamp(tauCommand, -TORQUE_MAX, TORQUE_MAX);
    const leftDemand = .5*(f+tau/ARM), rightDemand = .5*(f-tau/ARM);
    const left = clamp(leftDemand, 0, .5*p.thrustMax);
    const right = clamp(rightDemand, 0, .5*p.thrustMax);
    return {left, right, f:left+right, tau:ARM*(left-right),
      limited:Math.abs(f-fCommand)>1e-8 || Math.abs(tau-tauCommand)>1e-8 ||
        Math.abs(left-leftDemand)>1e-8 || Math.abs(right-rightDemand)>1e-8};
  }
  function control(t, y, p) {
    const r = reference(t, p);
    const vcx = r.vx+p.kpos*(r.x-y[0]), vcz = r.vz+p.kpos*(r.z-y[1]);
    const ax = r.ax+KV*(vcx-y[2]), az = r.az+KV*(vcz-y[3]);
    const fx = MASS*ax, fz = MASS*(az+(p.gravityComp?G:0));
    const fc = Math.hypot(fx,fz);
    // atan2(horizontal, vertical): clockwise tilt on an x-right, z-up drawing.
    const rawTheta = fc>1e-12 ? Math.atan2(fx,fz) : 0;
    const thetaC = clamp(rawTheta, -p.tiltMax*DEG, p.tiltMax*DEG);
    const thetaError = Math.atan2(Math.sin(thetaC-y[4]), Math.cos(thetaC-y[4]));
    const tauC = J*(p.attitude*p.attitude*thetaError-2*.9*p.attitude*y[5]);
    const act = allocation(fc,tauC,p);
    return {...act, r, vcx, vcz, ax, az, fx, fz, fc, thetaC, tauC,
      tiltLimited:Math.abs(rawTheta-thetaC)>1e-8,
      limited:act.limited || Math.abs(rawTheta-thetaC)>1e-8};
  }
  function physics(y, f, tau) {
    return [y[2],y[3],f*Math.sin(y[4])/MASS,f*Math.cos(y[4])/MASS-G,y[5],tau/J];
  }
  function derivative(t,y,p) {
    const u=control(t,y,p);
    return physics(y,u.f,u.tau);
  }
  function rk4(t,y,h,p) {
    const a=derivative(t,y,p);
    const b=derivative(t+h/2,y.map((v,i)=>v+h*a[i]/2),p);
    const c=derivative(t+h/2,y.map((v,i)=>v+h*b[i]/2),p);
    const d=derivative(t+h,y.map((v,i)=>v+h*c[i]),p);
    return y.map((v,i)=>v+h*(a[i]+2*b[i]+2*c[i]+d[i])/6);
  }
  function sample(t,y,p) {
    const u=control(t,y,p);
    return {t,x:y[0],z:y[1],vx:y[2],vz:y[3],theta:y[4],q:y[5],
      xRef:u.r.x,zRef:u.r.z,thetaC:u.thetaC,fc:u.fc,f:u.f,
      tauC:u.tauC,tau:u.tau,left:u.left,right:u.right,
      error:Math.hypot(u.r.x-y[0],u.r.z-y[1]),limited:u.limited,
      ax:u.f*Math.sin(y[4])/MASS,az:u.f*Math.cos(y[4])/MASS-G};
  }
  function simulate(input = {}) {
    const p=parameters(input), duration=p.mission==='circle'?24:18;
    const dt=clamp(finite(input._dt,DT), .0005, .005);
    let y=p.mission==='circle'?[1.2,3,0,0,0,0]:[-3,1.5,0,0,0,0];
    let t=0, limitedTime=0, elapsed=0, stoppedReason='';
    const frames=[sample(t,y,p)];
    for (let outputTime=FRAME_DT; outputTime<=duration+1e-9; outputTime+=FRAME_DT) {
      const end=Math.min(duration,outputTime);
      while(t<end-1e-11) {
        const requested=control(t,y,p);
        if(requested.fz < -1e-8) {
          stoppedReason='Essai arrêté : la force demandée pointe vers le bas, hors du domaine de la commande de vol à l’endroit. La gravité physique reste active.';
          break;
        }
        if(Math.abs(y[0])>8 || y[1]>8 || Math.abs(y[4])>75*DEG) {
          stoppedReason='Essai arrêté à la limite de la démonstration : |x| ≤ 8 m, z ≤ 8 m et |ϑ| ≤ 75°. Ces limites pédagogiques ne sont pas des limites garanties du véhicule.';
          break;
        }
        const h=Math.min(dt,end-t), previous=y, next=rk4(t,y,h,p);
        if(!next.every(Number.isFinite) || Math.max(...next.map(Math.abs))>1e5) {
          stoppedReason='Essai arrêté : le modèle est sorti du domaine numérique prévu.';
          break;
        }
        if(control(t,previous,p).limited) limitedTime+=h;
        elapsed+=h;
        if(next[1]<=0) {
          // Stop at first crossing of the centre's z=0 plane. No contact/bounce law.
          const fraction=clamp(previous[1]/(previous[1]-next[1]),0,1);
          y=previous.map((v,i)=>v+fraction*(next[i]-v)); y[1]=0;t+=fraction*h;
          stoppedReason='Essai arrêté au passage du centre à z = 0. Le contact avec le sol et les rebonds ne sont pas modélisés.';
          break;
        }
        y=next;t+=h;
      }
      if(t>frames[frames.length-1].t+1e-12)frames.push(sample(t,y,p));
      if(stoppedReason)break;
    }
    const last=frames[frames.length-1];
    const rms=Math.sqrt(frames.reduce((s,f)=>s+f.error*f.error,0)/frames.length);
    const series=(name,color,get,dashed=false)=>({name,color,values:frames.map(get),dashed});
    return {frames,duration:last.t,params:p,stopped:!!stoppedReason,stoppedReason,
      bounds:{xmin:Math.min(-5,...frames.map(f=>f.x-.7)),xmax:Math.max(5,...frames.map(f=>f.x+.7)),zmin:0,zmax:Math.max(6,...frames.map(f=>f.z+.9))},
      charts:[
        {title:'Position et référence',yLabel:'m',series:[series('x',0,f=>f.x),series('x demandé',0,f=>f.xRef,true),series('z',1,f=>f.z),series('z demandé',1,f=>f.zRef,true)]},
        {title:'Inclinaison du drone',yLabel:'degrés',series:[series('ϑ réel',0,f=>f.theta/DEG),series('ϑ demandé',2,f=>f.thetaC/DEG,true)]},
        {title:'Poussée et poids',yLabel:'N',series:[series('Poussée réalisée',1,f=>f.f),series('Poussée demandée',2,f=>f.fc,true),series('Poids mg',3,()=>MASS*G,true)]},
        {title:'Moment de tangage',yLabel:'N·m',series:[series('Moment réalisé',4,f=>f.tau),series('Moment demandé',2,f=>f.tauC,true)]}
      ],
      metrics:[{label:'Erreur finale',value:last.error,unit:'m'},
        {label:'Erreur quadratique moyenne',value:rms,unit:'m'},
        {label:'Inclinaison maximale',value:Math.max(...frames.map(f=>Math.abs(f.theta)))/DEG,unit:'°'},
        {label:'Limites actives',value:100*limitedTime/Math.max(elapsed,1e-12),unit:'% du temps'}],
      note:'Coupe verticale x–z : m = 1,5 kg, Jᵧ = 0,035 kg·m², ℓ = 0,25 m. Gravité physique toujours active (9,81 m/s²). Position P → vitesse P (kᵥ = 2,2 s⁻¹), anticipation de la trajectoire, puis PD d’orientation (amortissement 0,9). La case retire seulement la compensation +g. Poussée demandée = norme de m(a_c + g e_z) ; si l’angle est limité, cette norme est conservée avant limitation, donc l’accélération change. Moment borné à ±0,8 N·m et deux groupes bornés à la moitié de la poussée maximale ; efforts réels recalculés après allocation. RK4, pas 2,5 ms. Échantillons affichés toutes les 20 ms. Arrêt si la force demandée pointe vers le bas (la force nulle reste permise pour observer la chute), si |ϑ| > 75°, |x| > 8 m, z > 8 m, ou au sol. Ce sont des limites de démonstration, pas une garantie du véhicule. Pas de traînée, de retard moteur, de bruit, ni de dynamique hors du plan. Chaque cible relance depuis le même départ.'
    };
  }
  function view(w,h,b) {
    const left=45,right=22,top=52,bottom=48;
    const scale=Math.min((w-left-right)/(b.xmax-b.xmin),(h-top-bottom)/(b.zmax-b.zmin));
    const ox=left+(w-left-right)/2-scale*(b.xmin+b.xmax)/2;
    const oz=top+(h-top-bottom)/2+scale*(b.zmin+b.zmax)/2;
    return {scale,X:x=>ox+x*scale,Y:z=>oz-z*scale,worldX:x=>(x-ox)/scale,worldZ:y=>(oz-y)/scale};
  }
  function arrow(c,x1,y1,x2,y2,color,width=2,dashed=false) {
    c.save();c.strokeStyle=color;c.fillStyle=color;c.lineWidth=width;c.setLineDash(dashed?[6,4]:[]);
    c.beginPath();c.moveTo(x1,y1);c.lineTo(x2,y2);c.stroke();c.setLineDash([]);
    const a=Math.atan2(y2-y1,x2-x1),l=8;c.beginPath();c.moveTo(x2,y2);
    c.lineTo(x2-l*Math.cos(a-.42),y2-l*Math.sin(a-.42));
    c.lineTo(x2-l*Math.cos(a+.42),y2-l*Math.sin(a+.42));c.closePath();c.fill();c.restore();
  }
  function draw(c,w,h,sim,index,palette) {
    const f=sim.frames[clamp(index,0,sim.frames.length-1)], b=sim.bounds;
    const {X,Y,scale}=view(w,h,b), colors=palette.series;
    c.fillStyle=palette.bg;c.fillRect(0,0,w,h);
    c.font='12px -apple-system,Segoe UI,sans-serif';c.fillStyle=palette.ink;
    c.fillText('Coupe verticale x–z',18,23);
    c.fillStyle=palette.muted;c.font='11px -apple-system,Segoe UI,sans-serif';
    const legend=w>560?'Trajet réel · axe réel en vert · axe demandé en orange':'Axe réel : vert · demandé : orange';
    c.fillText(legend,18,41);
    for(let x=Math.ceil(b.xmin);x<=b.xmax;x++) {
      c.strokeStyle=palette.grid;c.lineWidth=1;c.beginPath();c.moveTo(X(x),Y(b.zmin));c.lineTo(X(x),Y(b.zmax));c.stroke();
      if(x%2===0){c.fillStyle=palette.muted;c.textAlign='center';c.fillText(x,X(x),Y(0)+18);}
    }
    for(let z=0;z<=b.zmax;z++) {
      c.strokeStyle=palette.grid;c.beginPath();c.moveTo(X(b.xmin),Y(z));c.lineTo(X(b.xmax),Y(z));c.stroke();
      c.fillStyle=palette.muted;c.textAlign='right';c.fillText(z,X(b.xmin)-9,Y(z)+4);
    }
    c.textAlign='left';c.fillStyle=palette.muted;c.fillText('z (m)',X(b.xmin)+8,Y(b.zmax)+16);
    c.textAlign='right';c.fillText('x (m)',X(b.xmax),Y(0)+34);c.textAlign='left';
    c.fillStyle='#edf1f5';c.fillRect(X(b.xmin),Y(0),X(b.xmax)-X(b.xmin),5);
    c.save();c.beginPath();c.rect(X(b.xmin),Y(b.zmax),(b.xmax-b.xmin)*scale,(b.zmax-b.zmin)*scale);c.clip();
    if(sim.params.mission==='circle') {
      c.strokeStyle=colors[0];c.lineWidth=1.5;c.setLineDash([5,5]);c.beginPath();
      sim.frames.forEach((v,i)=>{if(i===0)c.moveTo(X(v.xRef),Y(v.zRef));else c.lineTo(X(v.xRef),Y(v.zRef));});c.stroke();c.setLineDash([]);
    }
    c.strokeStyle=colors[1];c.lineWidth=2.5;c.beginPath();
    for(let i=0;i<=index;i++){const v=sim.frames[i];if(i===0)c.moveTo(X(v.x),Y(v.z));else c.lineTo(X(v.x),Y(v.z));}c.stroke();
    const gx=X(f.xRef),gz=Y(f.zRef);
    c.strokeStyle=colors[0];c.lineWidth=1.8;c.beginPath();c.arc(gx,gz,9,0,2*Math.PI);c.stroke();
    c.beginPath();c.moveTo(gx-14,gz);c.lineTo(gx+14,gz);c.moveTo(gx,gz-14);c.lineTo(gx,gz+14);c.stroke();
    const x=X(f.x),z=Y(f.z),body=Math.max(28,Math.min(44,.5*scale));
    const len=Math.min(1.45,.8*f.f/(MASS*G))*scale;
    const desiredLen=Math.min(1.45,.8*f.fc/(MASS*G))*scale;
    arrow(c,x,z,x+Math.sin(f.thetaC)*desiredLen,z-Math.cos(f.thetaC)*desiredLen,colors[2],1.8,true);
    arrow(c,x,z,x+Math.sin(f.theta)*len,z-Math.cos(f.theta)*len,colors[1],2.6);
    arrow(c,x,z,x,z+.8*scale,colors[3],1.8);
    c.fillStyle=colors[3];c.font='11px -apple-system,Segoe UI,sans-serif';c.fillText('Poids',x+6,z+.8*scale);
    c.save();c.translate(x,z);c.rotate(f.theta);
    // Positive screen rotation is clockwise: right arm down, thrust to the right.
    c.strokeStyle=palette.ink;c.lineWidth=4;c.beginPath();c.moveTo(-body,0);c.lineTo(body,0);c.stroke();
    c.fillStyle=palette.ink;c.fillRect(-9,-5,18,10);
    for(const side of [-1,1]) {
      c.fillStyle=colors[1];c.fillRect(side*body-3,-10,6,11);
      c.strokeStyle=palette.ink;c.lineWidth=2;c.beginPath();c.ellipse(side*body,-11,14,3.5,0,0,2*Math.PI);c.stroke();
      c.strokeStyle=palette.muted;c.lineWidth=1;c.beginPath();c.moveTo(side*body-9,-11);c.lineTo(side*body+9,-11);c.stroke();
    }
    c.restore();c.restore();
    c.font='12px -apple-system,Segoe UI,sans-serif';c.fillStyle=palette.ink;
    const fmt=n=>n.toLocaleString('fr-FR',{maximumFractionDigits:1});
    c.fillText(`x = ${fmt(f.x)} m   z = ${fmt(f.z)} m`,18,h-12);
    if(w>500){c.textAlign='right';c.fillStyle=f.limited?colors[2]:palette.muted;
      c.fillText(`ϑ = ${fmt(f.theta/DEG)}°${f.limited?'  ·  limite active':''}`,w-18,23);c.textAlign='left';}
    if(f.z<=0){c.fillStyle=colors[2];c.fillText('Arrêt au sol — contact non modélisé',18,h-31);}
  }
  function onCanvasClick(xFraction,yFraction,sim,params,dimensions) {
    if(params.mission!=='goal')return null;
    const w=dimensions?.width||800,h=dimensions?.height||480;
    const v=view(w,h,sim.bounds);
    const x=v.worldX(xFraction*w),z=v.worldZ(yFraction*h);
    if(x<sim.bounds.xmin||x>sim.bounds.xmax||z<0||z>sim.bounds.zmax)return null;
    return {goalX:clamp(x,-4.5,4.5),goalZ:clamp(z,.5,5.5)};
  }
  function selfTest() {
    const checks=[];
    const check=(label,condition)=>{if(!condition)throw Error('Quadrirotor : '+label);checks.push(label);};
    check('stationnaire',physics([0,2,0,0,0,0],MASS*G,0).every(v=>Math.abs(v)<1e-12));
    const tilted=physics([0,2,0,0,.2,0],MASS*G/Math.cos(.2),0);
    check('signe de la poussée et altitude',tilted[2]>0 && Math.abs(tilted[3])<1e-12);
    const p=parameters({}), a=allocation(MASS*G,.2,p);
    check('moment positif à gauche',a.left>a.right && Math.abs(a.tau-.2)<1e-12);
    const saturated=allocation(100,100,p);
    check('allocation bornée',saturated.left>=0 && saturated.right>=0 && saturated.left<=11 && saturated.right<=11 && Math.abs(saturated.tau)<=ARM*Math.min(saturated.f,22-saturated.f)+1e-12);
    const run=simulate(p),last=run.frames.at(-1);
    check('convergence du but par défaut',!run.stopped && last.error<.002);
    let stepDifference=0,angleDifference=0;
    for(const trial of [p,{...p,kpos:1.1,attitude:10,tiltMax:35,thrustMax:26,goalX:4,goalZ:4}]) {
      const full=simulate(trial),half=simulate({...trial,_dt:DT/2});
      check('essai de convergence temporelle admissible',!full.stopped && !half.stopped && full.frames.length===half.frames.length);
      full.frames.forEach((f,i)=>{const h=half.frames[i];stepDifference=Math.max(stepDifference,Math.hypot(f.x-h.x,f.z-h.z));angleDifference=Math.max(angleDifference,Math.abs(f.theta-h.theta));});
    }
    check('pas RK4 divisé par deux sur deux trajectoires entières',stepDifference<1e-4 && angleDifference<1e-4);
    const orbit=simulate({...p,mission:'circle'});
    check('cercle borné',!orbit.stopped && orbit.frames.slice(-200).every(f=>f.error<.15));
    const fall=simulate({...p,gravityComp:false});
    check('gravité sans compensation et arrêt',fall.stopped && fall.frames.at(-1).z===0 && fall.frames.at(-1).t<18);
    return {checks,stepDifference,angleDifference,goalFinalError:last.error,circleFinalError:orbit.frames.at(-1).error,fallTime:fall.frames.at(-1).t};
  }
  window.robotLabModules=window.robotLabModules||{};
  window.robotLabModules.quad={id:'quad',title:'Quadrirotor dans le plan vertical',defaults,
    controls:[
      {key:'mission',label:'Mission',type:'select',options:[{value:'goal',label:'Rejoindre un point'},{value:'circle',label:'Suivre un cercle vertical'}]},
      {key:'gravityComp',label:'Compenser la gravité (+g)',type:'checkbox'},
      {key:'kpos',label:'Gain de position',type:'range',min:.25,max:1.6,step:.05,unit:'s⁻¹'},
      {key:'attitude',label:'Rapidité de l’orientation',type:'range',min:3,max:12,step:.5,unit:'s⁻¹'},
      {key:'tiltMax',label:'Inclinaison demandée maximale',type:'range',min:5,max:45,step:1,unit:'°'},
      {key:'thrustMax',label:'Poussée totale maximale',type:'range',min:16,max:30,step:.5,unit:'N'}
    ],simulate,draw,onCanvasClick,selfTest};
})();
