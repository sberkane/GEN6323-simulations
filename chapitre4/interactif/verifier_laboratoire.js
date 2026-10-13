/* Exécuter : node verifier_laboratoire.js. Aucune dépendance externe. */
'use strict';
const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert');
const context={window:{robotLabModules:{}}};vm.createContext(context);
for(const name of ['planar','vitesse','quad'])vm.runInContext(fs.readFileSync(path.join(__dirname,name+'.js'),'utf8'),context,{filename:name+'.js'});
const {planar,vitesse,quad}=context.window.robotLabModules;
const reports=[];
function checkContract(name,sim){
 assert(sim.frames.length>1,name+' : mesures absentes');
 sim.frames.forEach((f,i)=>{assert(Number.isFinite(f.t));if(i)assert(f.t>sim.frames[i-1].t);});
 for(const c of sim.charts)for(const s of c.series){assert.equal(s.values.length,sim.frames.length,name+' : longueur de série');assert(s.values.every(v=>v===null||Number.isFinite(v)),name+' : nombre non fini');}
 reports.push({name,status:sim.status||'ok',duration:sim.duration,frames:sim.frames.length});return sim;
}
const fixed=checkContract('Point / PI',planar.simulate(planar.defaults));
assert(Math.abs(fixed.frames.at(-1).rho-.003844664223416)<2e-6,'Cascade incompatible avec la figure Python');
for(const f of fixed.frames){assert(Math.abs(f.tauL)<=1.4+1e-12);assert(Math.abs(f.tauR)<=1.4+1e-12);}
const ideal=checkContract('Point / idéal',planar.simulate({...planar.defaults,realization:'ideal'}));
for(let i=1;i<ideal.frames.length;i++)assert(ideal.frames[i].rho<=ideal.frames[i-1].rho+1e-10,'Distance idéale croissante');
for(const mission of ['trajectory','path'])for(const realization of ['ideal','P','PI']){
 const sim=checkContract(mission+' / '+realization,planar.simulate({...planar.defaults,mission,realization,duration:8,tauMax:.4}));
 if(realization!=='ideal')for(const f of sim.frames){assert(Math.abs(f.tauL)<=.4+1e-12);assert(Math.abs(f.tauR)<=.4+1e-12);}
}
for(const [goalX,goalY] of [[2,1],[-2,1],[-2,-1],[2,-1],[0,0]])for(const heading of [-180,90,180])for(const tauMax of [.2,1.4]){
 const sim=checkContract('Pose '+[goalX,goalY,heading,tauMax].join('/'),planar.simulate({...planar.defaults,mission:'pose',goalX,goalY,heading,tauMax}));const f=sim.frames.at(-1);
 assert.equal(sim.status,'ok');assert(f.rho<=.06+1e-9);assert(Math.abs(f.headingError)<=.06+1e-9);assert(sim.events.some(e=>e.to==='hold'));
}
const p=checkContract('Vitesse P analytique',vitesse.simulate({...vitesse.defaults,controller:'P'}));
let maxAnalytic=0;for(const f of p.frames){const exact=1-Math.exp(-4*f.t)-(f.t>=5?3/48*(1-Math.exp(-4*(f.t-5))):0);maxAnalytic=Math.max(maxAnalytic,Math.abs(f.v-exact));}assert(maxAnalytic<1e-8);
const pi=checkContract('Vitesse PI',vitesse.simulate(vitesse.defaults));assert(Math.abs(pi.frames.at(-1).error)<1e-7);assert(Math.abs(pi.frames.at(-1).integralForce-3)<1e-5);
const pid=checkContract('Vitesse PID',vitesse.simulate({...vitesse.defaults,controller:'PID'}));assert(Math.abs(pid.frames.at(-1).error)<1e-6);
const windup=checkContract('PI saturé sans rétro-calcul',vitesse.simulate({...vitesse.defaults,load:0,saturation:true,antiwindup:false}));
const aw=checkContract('PI saturé avec rétro-calcul',vitesse.simulate({...vitesse.defaults,load:0,saturation:true,antiwindup:true}));
assert(Math.max(...aw.frames.map(f=>f.v))<Math.max(...windup.frames.map(f=>f.v))-.4);
for(const sim of [aw,windup])for(const f of sim.frames)assert(Math.abs(f.tau)<=.4+1e-12);
if(quad){checkContract('Quadrirotor / défaut',quad.simulate(quad.defaults));reports.push({quadSelfTests:quad.selfTest()});}
console.log(JSON.stringify({status:'Réussi',tests:reports.length,maxErreurPAnalytique:maxAnalytic,experiences:reports},null,2));
