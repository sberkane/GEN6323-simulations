/* GEN6323 - MPC scalaire, horizons 1 et 2. Aucune bibliothèque externe. */
(function (root) {
  'use strict';
  const dot = (a, b) => a.reduce((s, x, i) => s + x * b[i], 0);
  function build({x = 0, v = 2, N = 2, T = 1, rho = 0.1, vref = 2, limit = 2} = {}) {
    if (![x,v,T,rho,vref,limit].every(Number.isFinite) || ![1,2].includes(N) || T <= 0 || rho <= 0 || v < -1e-9) throw Error('Paramètres invalides');
    const D = [], E = [], H = [], g = [], C = [], b = [];
    for (let i=0; i<N; i++) {
      D.push(Array.from({length:N}, (_,j) => j<=i ? T : 0));
      E.push(Array.from({length:N}, (_,j) => j<=i ? T*T*(i-j+0.5) : 0));
    }
    for (let j=0; j<N; j++) {
      H.push(Array.from({length:N}, (_,k) => 2*(D.reduce((s,row) => s+row[j]*row[k],0)+(j===k?rho:0))));
      g.push(2*D.reduce((s,row)=>s+row[j]*(v-vref),0));
      const unit=Array.from({length:N},(_,k)=>k===j?1:0);
      C.push(unit,unit.map(z=>-z)); b.push(1,1);
    }
    for (let i=0; i<N; i++) { C.push(D[i].map(z=>-z),E[i]); b.push(v,limit-x-(i+1)*T*v); }
    return {x,v,N,T,rho,vref,limit,D,E,H,g,C,b};
  }
  function inverse(H) {
    if(H.length===1)return [[1/H[0][0]]];
    const d=H[0][0]*H[1][1]-H[0][1]*H[1][0];
    return [[H[1][1]/d,-H[0][1]/d],[-H[1][0]/d,H[0][0]/d]];
  }
  function solve(options={}) {
    const p=build(options), {N,H,g,C,b}=p, inv=inverse(H);
    if(p.x>p.limit+1e-8)return {status:'infeasible',problem:p,u:null,states:[]};
    const u0=inv.map(row=>-dot(row,g)), candidates=[u0];
    // Minimum sur chaque droite (ou point pour N=1) d'une contrainte active.
    for(let i=0;i<C.length;i++) {
      const w=inv.map(row=>dot(row,C[i])), den=dot(C[i],w);
      if(den>1e-15) { const t=(dot(C[i],u0)-b[i])/den; candidates.push(u0.map((z,j)=>z-t*w[j])); }
    }
    // Sommets du polygone admissible. Les intersections parallèles sont ignorées.
    if(N===2)for(let i=0;i<C.length;i++)for(let j=i+1;j<C.length;j++) {
      const d=C[i][0]*C[j][1]-C[i][1]*C[j][0];
      if(Math.abs(d)>1e-12)candidates.push([(b[i]*C[j][1]-C[i][1]*b[j])/d,(C[i][0]*b[j]-b[i]*C[j][0])/d]);
    }
    let u=null, score=Infinity;
    for(const z of candidates) {
      if(!z.every(Number.isFinite)||C.some((row,i)=>dot(row,z)>b[i]+1e-8))continue;
      const q=0.5*dot(z,H.map(row=>dot(row,z)))+dot(g,z);
      if(q<score){score=q;u=z;}
    }
    if(!u)return {status:'infeasible',problem:p,u:null,states:[]};
    const states=[{x:p.x,v:p.v}];
    for(const a of u)states.push(advance(states.at(-1),a,p.T));
    const cost=states.slice(1).reduce((s,z,i)=>s+(z.v-p.vref)**2+p.rho*u[i]**2,0);
    return {status:'optimal',problem:p,u,states,cost,
      residual:Math.max(0,...C.map((row,i)=>dot(row,u)-b[i]))};
  }
  function advance({x,v},a,T=1) {return {x:x+T*v+0.5*T*T*a,v:v+T*a};}
  function simulate(options={},steps=6) {
    let state={x:options.x??0,v:options.v??2};const states=[state],controls=[],plans=[];
    for(let k=0;k<steps;k++) {
      const plan=solve({...options,...state});plans.push(plan);
      if(plan.status!=='optimal')return {states,controls,plans,status:'infeasible'};
      state=advance(state,plan.u[0],options.T??1);states.push(state);controls.push(plan.u[0]);
      if(Math.abs(state.v)<1e-8&&Math.abs(state.x-(options.limit??2))<1e-8)return {states,controls,plans,status:'stopped'};
    }
    return {states,controls,plans,status:'horizon'};
  }
  const api={build,solve,advance,simulate};
  if(typeof module!=='undefined'&&module.exports)module.exports=api;
  else root.MPC=api;
})(typeof globalThis!=='undefined'?globalThis:this);
