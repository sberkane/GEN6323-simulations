'use strict';
// Verification mathematique autonome : node verifier_solver.js
// Le verificateur reconstruit les contraintes, les couts et les certificats;
// pour N<=3, une enumeration independante des faces fournit aussi l'optimum.
const assert=require('node:assert/strict');
const {performance}=require('node:perf_hooks');
const m=require('./mpc.js');
const dot=(a,b)=>a.reduce((s,z,i)=>s+z*b[i],0);
const norm=a=>Math.hypot(...a);
const defaults={x:0,v:2,N:2,T:1,rho:0.1,vref:2,limit:2};

function runVerification() {
  let checks=0,feasible=0,infeasible=0,kkt=0,singletons=0,maxIterations=0;
  let maxPrimalResidual=0,maxStationarity=0,maxComplementarity=0;
  const ok=(value,message)=>{assert.ok(value,message);checks++;};
  const near=(a,b,tol=1e-8)=>ok(Math.abs(a-b)<=tol,`${a} != ${b}`);
  const minimum=(p)=>{
    let x=p.x,v=p.v;
    for(let i=0;i<p.N;i++) {const w=Math.max(0,v-p.T);x+=p.T*(v+w)/2;v=w;}
    return x;
  };
  function verify(options,plan=m.solve(options)) {
    const p={...defaults,...options},xmin=minimum(p);
    ok(plan.status!=='numerical_failure',`Numerical failure: ${JSON.stringify(options)} ${plan.reason}`);
    if(p.x>p.limit+1e-8||xmin>p.limit+1e-8) {
      ok(plan.status==='infeasible','Maximal braking proves infeasibility');
      ok(plan.u===null&&plan.states.length===0,'An infeasible plan must not offer a command');
      near(plan.certificate.minimumPosition,xmin);infeasible++;return plan;
    }
    ok(plan.status==='optimal',`Expected feasible: ${JSON.stringify(options)}`);feasible++;
    ok(plan.u.length===p.N&&plan.states.length===p.N+1,'Prediction dimensions');
    let x=p.x,v=p.v,cost=0;
    for(let i=0;i<p.N;i++) {
      const a=plan.u[i];ok(Number.isFinite(a)&&Math.abs(a)<=1+1e-8,'Acceleration bounds');
      // A constant acceleration has an affine velocity; with both endpoint
      // velocities nonnegative, position is monotone throughout the interval.
      for(const t of[0,p.T/4,p.T/2,3*p.T/4,p.T]) {
        const xt=x+t*v+t*t*a/2,vt=v+t*a;
        ok(xt<=p.limit+1e-8&&vt>=-1e-8,'Continuous trajectory constraints');
        maxPrimalResidual=Math.max(maxPrimalResidual,xt-p.limit,-vt,Math.abs(a)-1);
      }
      x+=p.T*v+p.T*p.T*a/2;v+=p.T*a;
      near(plan.states[i+1].x,x);near(plan.states[i+1].v,v);
      cost+=(v-p.vref)**2+p.rho*a*a;
    }
    near(plan.cost,cost,1e-8*(1+cost));maxIterations=Math.max(maxIterations,plan.iterations);
    const cert=plan.certificate,z=plan.states.slice(1).map(s=>s.v);
    if(cert.type==='singleton') {
      // Every admissible velocity is at least max(0,v0-i*T). With positive
      // position weights and no remaining distance, all these bounds are equalities.
      near(xmin,p.limit,1e-9);
      z.forEach((v,i)=>near(v,Math.max(0,p.v-(i+1)*p.T)));
      singletons++;return plan;
    }
    ok(cert.type==='kkt','Optimality certificate type');
    // Independently reconstruct the normalized inequalities used by the dual.
    const C=[Array.from({length:p.N},(_,i)=>i===p.N-1?p.T/2:p.T)],b=[p.limit-p.x-p.T*p.v/2];
    for(let i=0;i<p.N;i++) {
      const low=Array(p.N).fill(0);low[i]=-1;C.push(low);b.push(0);
      const d=Array(p.N).fill(0);d[i]=1;if(i)d[i-1]=-1;
      C.push(d,d.map(a=>-a));b.push(p.T+(i?0:p.v),p.T-(i?0:p.v));
    }
    C.forEach((row,i)=>{const size=norm(row);C[i]=row.map(a=>a/size);b[i]/=size;});
    ok(cert.dual.length===C.length&&cert.dual.every(a=>a>=-1e-10),'Dual feasibility');
    const gradient=z.map((v,i)=>2*(v-p.vref)+2*p.rho/p.T*(plan.u[i]-(i+1<p.N?plan.u[i+1]:0)));
    const stationarity=norm(gradient.map((g,j)=>g+C.reduce((s,row,i)=>s+row[j]*cert.dual[i],0)));
    const complementarity=Math.max(...C.map((row,i)=>Math.abs(cert.dual[i]*(dot(row,z)-b[i]))));
    const scaledStationarity=stationarity/(1+norm(gradient)),scaledComplementarity=complementarity/(1+norm(gradient));
    ok(scaledStationarity<=2e-7,'Independent stationarity certificate');
    ok(scaledComplementarity<=2e-7,'Independent complementarity certificate');
    maxStationarity=Math.max(maxStationarity,scaledStationarity);
    maxComplementarity=Math.max(maxComplementarity,scaledComplementarity);kkt++;
    return plan;
  }

  // Reference solver: enumerate all linearly independent active faces in the
  // ORIGINAL acceleration variables. This is used only by tests for N<=3.
  function linearSolve(A,b) {
    const n=b.length,K=A.map((row,i)=>[...row,b[i]]);
    for(let j=0;j<n;j++) {
      let pivot=j;for(let i=j+1;i<n;i++)if(Math.abs(K[i][j])>Math.abs(K[pivot][j]))pivot=i;
      if(Math.abs(K[pivot][j])<1e-10)return null;
      [K[j],K[pivot]]=[K[pivot],K[j]];
      const d=K[j][j];for(let c=j;c<=n;c++)K[j][c]/=d;
      for(let i=0;i<n;i++)if(i!==j){const a=K[i][j];for(let c=j;c<=n;c++)K[i][c]-=a*K[j][c];}
    }
    return K.map(row=>row[n]);
  }
  function enumerate(options) {
    const p={...defaults,...options},n=p.N,C=[],b=[];
    const H=Array.from({length:n},(_,j)=>Array.from({length:n},(_,k)=>2*((n-Math.max(j,k))*p.T*p.T+(j===k?p.rho:0))));
    const g=Array.from({length:n},(_,j)=>2*p.T*(n-j)*(p.v-p.vref));
    for(let i=0;i<n;i++) {
      const e=Array(n).fill(0);e[i]=1;C.push(e,e.map(a=>-a));b.push(1,1);
      C.push(Array.from({length:n},(_,j)=>j<=i?-p.T:0));b.push(p.v);
      C.push(Array.from({length:n},(_,j)=>j<=i?(i-j+0.5)*p.T*p.T:0));b.push(p.limit-p.x-(i+1)*p.T*p.v);
    }
    let best=null;
    function candidate(active) {
      const K=H.map((row,j)=>[...row,...active.map(i=>C[i][j])]);
      active.forEach(i=>K.push([...C[i],...active.map(()=>0)]));
      const sol=linearSolve(K,[...g.map(a=>-a),...active.map(i=>b[i])]);if(!sol)return;
      const u=sol.slice(0,n),dual=sol.slice(n);
      if(dual.some(a=>a<-1e-7)||C.some((row,i)=>dot(row,u)>b[i]+1e-7))return;
      const cost=dot(g,u)+H.reduce((s,row,i)=>s+u[i]*dot(row,u)/2,0);
      if(best===null||cost<best.cost)best={u,cost};
    }
    function visit(active,start) {candidate(active);if(active.length===n)return;for(let i=start;i<C.length;i++)visit([...active,i],i+1);}
    visit([],0);return best;
  }

  // The original example, including its degenerate singleton for every N>=2.
  for(const rho of[0.01,0.1,1,10])for(let N=1;N<=20;N++) {
    const p=verify({N,rho});p.u.forEach((a,i)=>near(a,N===1?0:(i<2?-1:0)));
  }
  for(const rho of[0.01,0.1,1,10]) {
    const run=m.simulate({N:2,rho});ok(run.status==='stopped','Original closed-loop stop');
    near(run.controls[0],-1);near(run.controls[1],-1);
  }
  // Every UI speed and horizon, all four weights, on either side of the exact
  // feasibility boundary. Fractional speeds exercise the final partial brake.
  let boundaryCases=0;
  for(let N=1;N<=20;N++)for(let v=0.5;v<=5;v+=0.5)for(const rho of[0.01,0.1,1,10]) {
    const base={N,v,vref:v,rho},min=minimum({...defaults,...base});
    for(const offset of[-1e-5,0,1e-5]){verify({...base,limit:min+offset});boundaryCases++;}
  }
  // Cover all selectable speed/distance/horizon triples with rotating weights.
  let uiGridCases=0;
  for(let N=1;N<=20;N++)for(let vi=1;vi<=10;vi++)for(let di=1;di<=40;di++) {
    verify({N,v:vi/2,vref:vi/2,limit:di/2,rho:[0.01,0.1,1,10][(N+vi+di)%4]});uiGridCases++;
  }
  let seed=164;const rand=()=>((seed=(1664525*seed+1013904223)>>>0)/4294967296);
  const timings=[],randomCases=1600;
  for(let k=0;k<randomCases;k++) {
    const options={N:k%20+1,x:-2+4*rand(),v:0.5+4.5*rand(),vref:0.5+4.5*rand(),limit:0.5+19.5*rand(),rho:[0.01,0.1,1,10][k%4]};
    const begin=performance.now(),plan=m.solve(options);timings.push(performance.now()-begin);verify(options,plan);
  }
  let referenceCases=0;
  for(let k=0;k<240;k++) {
    const options={N:k<180?k%2+1:3,x:-1+2*rand(),v:0.5+4.5*rand(),vref:0.5+4.5*rand(),limit:0.5+19.5*rand(),rho:[0.01,0.1,1,10][k%4]};
    const plan=verify(options),ref=enumerate(options);
    ok((ref!==null)===(plan.status==='optimal'),'Independent enumeration feasibility');
    if(ref)ref.u.forEach((a,i)=>near(a,plan.u[i],2e-7));referenceCases++;
  }
  // A faster robot demonstrates why horizons greater than two are relevant.
  const short=m.simulate({N:2,v:4,vref:4,limit:8});
  ok(short.status==='infeasible'&&short.controls.length===1,'N2 loses feasibility');near(short.controls[0],0);
  for(const N of[3,4,6,10,20]) {
    const run=m.simulate({N,v:4,vref:4,limit:8});ok(run.status==='stopped','Long horizon anticipates braking');
    near(run.states.at(-1).x,8);run.controls.forEach(a=>near(a,-1));
  }
  for(const v of[0,0.5,1,1.5,2,4.5,5])near(m.stoppingDistance(v),minimum({...defaults,N:20,v}));
  for(const options of[{N:0},{N:21},{N:1.5},{N:NaN},{rho:0},{T:0},{v:-1},{limit:Infinity}]){assert.throws(()=>m.solve(options));checks++;}
  timings.sort((a,b)=>a-b);
  return {status:'passed',checks,uiGridCases,boundaryCases,randomCases,referenceCases,feasible,infeasible,kkt,singletons,maxIterations,
    residuals:{maxPrimal:maxPrimalResidual,maxScaledStationarity:maxStationarity,maxScaledComplementarity:maxComplementarity},
    solveMilliseconds:{median:timings[Math.floor(timings.length/2)],p95:timings[Math.floor(timings.length*0.95)],maximum:timings.at(-1)},
    notes:'Continuous convex QP; independent small-horizon enumeration, KKT or singleton certificates, and continuous trajectory constraints.'};
}
module.exports={runVerification};
if(require.main===module)console.log(JSON.stringify(runVerification(),null,2));
