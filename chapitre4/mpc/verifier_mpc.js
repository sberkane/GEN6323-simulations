'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const m=require('./mpc.js');let checks=0;
const near=(a,b,tol=1e-8)=>{assert.ok(Math.abs(a-b)<=tol,`${a} != ${b}`);checks++};
for(const rho of [0.01,0.1,1,10]) {
  let p=m.solve({N:1,rho});near(p.u[0],0);near(p.states[1].x,2);near(p.states[1].v,2);
  assert.equal(m.solve({N:1,x:2,v:2,rho}).status,'infeasible');checks++;
  p=m.solve({N:2,rho});p.u.forEach(a=>near(a,-1));near(p.states[1].x,1.5);near(p.states[2].x,2);near(p.states[2].v,0);
  const run=m.simulate({N:2,rho});assert.equal(run.status,'stopped');near(run.controls[0],-1);near(run.controls[1],-1);
}
// Certificat d'optimalité indépendant: faisabilité et gradient opposé au cône
// engendré par les normales actives. Suffisant pour un QP convexe.
function certificate(p) {
  const {problem:q,u}=p, dot=(a,b)=>a.reduce((s,z,i)=>s+z*b[i],0);
  const grad=q.H.map((r,i)=>dot(r,u)+q.g[i]);
  const active=q.C.filter((r,i)=>Math.abs(dot(r,u)-q.b[i])<1e-6);
  const ok=l=>Math.hypot(...l)<2e-6;
  if(ok(grad))return true;
  for(const c of active){const lam=-dot(c,grad)/dot(c,c);if(lam>=-1e-7&&ok(grad.map((z,i)=>z+lam*c[i])))return true;}
  if(q.N===2)for(let i=0;i<active.length;i++)for(let j=i+1;j<active.length;j++){
    const a=active[i],b=active[j],d=a[0]*b[1]-a[1]*b[0];if(Math.abs(d)<1e-10)continue;
    const l=(-grad[0]*b[1]+grad[1]*b[0])/d,t=(-a[0]*grad[1]+a[1]*grad[0])/d;
    if(l>=-1e-7&&t>=-1e-7&&ok(grad.map((z,k)=>z+l*a[k]+t*b[k])))return true;
  }return false;
}
let seed=164;const rand=()=>((seed=(1664525*seed+1013904223)>>>0)/4294967296);
let feasible=0,infeasible=0;
for(let k=0;k<600;k++){
  const opts={N:k%2+1,x:-3+5*rand(),v:3*rand(),rho:0.01+3*rand()},p=m.solve(opts);
  // Freinage maximal sans marche arriere : minimise chaque position future.
  let brake={x:opts.x,v:opts.v},possible=true;
  for(let i=0;i<opts.N;i++){brake=m.advance(brake,Math.max(-1,-brake.v));if(brake.x>2+1e-8)possible=false;}
  assert.equal(p.status==='optimal',possible);checks++;
  if(p.status==='infeasible'){infeasible++;continue;}
  feasible++;assert.ok(certificate(p),'KKT certificate failed');checks++;
  for(let i=0;i<p.u.length;i++){
    assert.ok(p.u[i]>=-1-1e-8&&p.u[i]<=1+1e-8);checks++;
    for(let j=0;j<=10;j++) {const z=m.advance(p.states[i],p.u[i],j/10);assert.ok(z.x<=2+1e-8&&z.v>=-1e-8);checks++;}
  }
}
// Une impossibilité immédiate demeure infaisable pour les deux horizons.
for(const N of [1,2]){assert.equal(m.solve({N,x:1.9,v:2}).status,'infeasible');checks++;}
// Le HTML contient exactement le solveur distribué, sans fetch ni CDN.
const html=fs.readFileSync(path.join(__dirname,'index.html'),'utf8');
assert.ok(html.includes(fs.readFileSync(path.join(__dirname,'mpc.js'),'utf8').trim()));checks++;
assert.ok(!/<script[^>]+src=|fetch\(|https?:\/\//i.test(html));checks++;
// Exécuter les vrais gestionnaires UI dans un DOM minimal (sans rendu graphique).
const vm=require('node:vm');
const elements=new Map();
function element(id){if(!elements.has(id))elements.set(id,{value:'',textContent:'',innerHTML:'',className:'',disabled:false,events:{},addEventListener(t,f){this.events[t]=f;},classList:{add(c){element(id).className+=' '+c;}}});return elements.get(id);}
const radios=[element('radio1'),element('radio2')];radios[0].value='1';radios[1].value='2';
const context=vm.createContext({document:{getElementById:element,querySelectorAll:()=>radios},setTimeout:()=>1,clearTimeout:()=>{}});
for(const match of html.matchAll(/<script[^>]*>([\s\S]*?)<\/script>/g))vm.runInContext(match[1],context);
assert.equal(element('mx').textContent,'0,00');assert.equal(element('ma').textContent,'-1,00');checks+=2;
element('step').events.click();near(Number(element('mx').textContent.replace(',','.')),1.5);
element('step').events.click();assert.ok(element('step').disabled);assert.ok(element('status').innerHTML.includes('Arrêt obtenu'));checks+=2;
radios[0].events.change();assert.equal(element('ma').textContent,'0,00');checks++;
element('step').events.click();assert.ok(element('status').innerHTML.includes('infaisable'));assert.ok(element('step').disabled);checks+=2;
element('reset').events.click();assert.equal(element('mx').textContent,'0,00');assert.ok(!element('step').disabled);checks+=2;
element('play').events.click();assert.equal(element('play').textContent,'Pause');element('play').events.click();assert.equal(element('play').textContent,'Lire');checks+=2;
radios[1].events.change();element('rho').value='10';element('rho').events.change();assert.ok(element('planhint').textContent.includes('25,00'));assert.ok(element('costformula').textContent.includes('10 aᵢ²'));checks+=2;
console.log(JSON.stringify({status:'passed',checks,randomCases:600,feasible,infeasible,notes:'Optima connus, certificats KKT, contraintes entre échantillons, solveur HTML identique.'},null,2));
