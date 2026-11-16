/* GEN6323 - MPC scalaire, horizons 1 a 20. Aucune bibliotheque externe. */
(function (root) {
  'use strict';
  const dot = (a,b) => a.reduce((s,z,i) => s+z*b[i],0);
  const norm = a => Math.sqrt(dot(a,a));
  const MAX_HORIZON = 20;
  function build({x=0,v=2,N=2,T=1,rho=0.1,vref=2,limit=2}={}) {
    if (![x,v,T,rho,vref,limit].every(Number.isFinite) || !Number.isInteger(N) || N<1 || N>MAX_HORIZON || T<=0 || rho<=0 || v < -1e-8) throw Error('Parametres invalides');
    const D=[],E=[],H=[],g=[],C=[],b=[];
    for(let i=0;i<N;i++) {
      D.push(Array.from({length:N},(_,j)=>j<=i?T:0));
      E.push(Array.from({length:N},(_,j)=>j<=i?T*T*(i-j+0.5):0));
    }
    for(let j=0;j<N;j++) {
      H.push(Array.from({length:N},(_,k)=>2*(D.reduce((s,row)=>s+row[j]*row[k],0)+(j===k?rho:0))));
      g.push(2*D.reduce((s,row)=>s+row[j]*(v-vref),0));
      const unit=Array.from({length:N},(_,k)=>k===j?1:0);
      C.push(unit,unit.map(z=>-z));b.push(1,1);
    }
    for(let i=0;i<N;i++){C.push(D[i].map(z=>-z),E[i]);b.push(v,limit-x-(i+1)*T*v);}
    return {x,v,N,T,rho,vref,limit,D,E,H,g,C,b};
  }
  function advance({x,v},a,T=1){return {x:x+T*v+0.5*T*T*a,v:v+T*a};}
  // Cette suite minimise chaque vitesse future et donc chaque position future.
  function braking(options={}) {
    const p=build(options),states=[{x:p.x,v:p.v}],u=[];
    for(let i=0;i<p.N;i++) {
      const a=-Math.min(1,Math.max(0,states.at(-1).v)/p.T);
      u.push(a);states.push(advance(states.at(-1),a,p.T));
    }
    return {u,states,minimumPosition:states.at(-1).x};
  }
  function stoppingDistance(v,T=1) {
    if(!Number.isFinite(v)||!Number.isFinite(T)||v<0||T<=0)throw Error('Parametres invalides');
    const n=Math.floor(v/T),r=v-n*T;
    return 0.5*v*v+0.5*r*(T-r);
  }
  function cholesky(A) {
    const n=A.length,L=Array.from({length:n},()=>Array(n).fill(0));
    const scale=Math.max(1,...A.map((r,i)=>Math.abs(r[i])));
    for(let i=0;i<n;i++)for(let j=0;j<=i;j++) {
      let s=A[i][j];for(let k=0;k<j;k++)s-=L[i][k]*L[j][k];
      if(i===j){if(!Number.isFinite(s)||s<=1e-12*scale)return null;L[i][j]=Math.sqrt(s);}
      else L[i][j]=s/L[j][j];
    }
    return L;
  }
  function cholSolve(L,b) {
    const n=b.length,y=Array(n),x=Array(n);
    for(let i=0;i<n;i++){let s=b[i];for(let j=0;j<i;j++)s-=L[i][j]*y[j];y[i]=s/L[i][i];}
    for(let i=n-1;i>=0;i--){let s=y[i];for(let j=i+1;j<n;j++)s-=L[j][i]*x[j];x[i]=s/L[i][i];}
    return x;
  }
  // Le calcul interne utilise z=(v1,...,vN). Le cout est bien conditionne;
  // z>=0 rend x croissant, donc seule la derniere borne de position est utile.
  function velocityProblem(p) {
    const {N,T,rho,v,vref,x,limit}=p,r=rho/(T*T);
    const H=Array.from({length:N},(_,i)=>Array.from({length:N},(_,j)=>i===j?2*(1+r*(i===N-1?1:2)):(Math.abs(i-j)===1?-2*r:0)));
    const g=Array.from({length:N},(_,i)=>-2*vref-(i===0?2*r*v:0));
    const C=[Array.from({length:N},(_,i)=>i===N-1?T/2:T)],b=[limit-x-T*v/2];
    for(let i=0;i<N;i++) {
      const low=Array(N).fill(0);low[i]=-1;C.push(low);b.push(0);
      const delta=Array(N).fill(0);delta[i]=1;if(i)delta[i-1]=-1;
      C.push(delta);b.push(T+(i===0?v:0));
      C.push(delta.map(a=>-a));b.push(T-(i===0?v:0));
    }
    // Normaliser les lignes n'affecte pas l'ensemble admissible.
    for(let i=0;i<C.length;i++){const s=norm(C[i]);C[i]=C[i].map(a=>a/s);b[i]/=s;}
    return {H,g,C,b};
  }
  function solve(options={}) {
    const p=build(options),seed=braking(p),tol=1e-8;
    if(p.x>p.limit+tol || seed.minimumPosition>p.limit+tol)
      return {status:'infeasible',problem:p,u:null,states:[],certificate:{type:'maximal_braking',minimumPosition:seed.minimumPosition}};
    const q=velocityProblem(p),{H,g,C,b}=q,L=cholesky(H);
    const failure=(reason,iterations=0)=>({status:'numerical_failure',problem:p,u:null,states:[],reason,iterations});
    if(!L)return failure('Hessienne mal conditionnee');
    const finish=(z,iterations,certificate)=>{
      const u=z.map((w,i)=>(w-(i?z[i-1]:p.v))/p.T),states=[{x:p.x,v:p.v}];
      for(const a of u)states.push(advance(states.at(-1),a,p.T));
      const residual=Math.max(0,...p.C.map((row,i)=>dot(row,u)-p.b[i]));
      if(!u.every(Number.isFinite)||residual>2e-7)return failure('Verification finale des contraintes echouee',iterations);
      const cost=states.slice(1).reduce((s,z,i)=>s+(z.v-p.vref)**2+p.rho*u[i]**2,0);
      return {status:'optimal',problem:p,u,states,cost,residual,iterations,certificate};
    };
    let z=seed.states.slice(1).map(s=>s.v);
    // Toutes les vitesses sont >= a celles du freinage maximal. Si sa position
    // finale atteint la ligne, les poids positifs imposent cette unique suite.
    if(Math.abs(seed.minimumPosition-p.limit)<=1e-10)
      return finish(z,0,{type:'singleton',minimumPosition:seed.minimumPosition,margin:p.limit-seed.minimumPosition});
    const HinvC=C.map(row=>cholSolve(L,row)),W=[];
    const gram=indices=>indices.map(i=>indices.map(j=>dot(C[i],HinvC[j])));
    function independent(i){return W.length<p.N && !!cholesky(gram([...W,i]));}
    const maxIterations=1000;
    for(let iteration=0;iteration<maxIterations;iteration++) {
      const grad=H.map((row,i)=>dot(row,z)+g[i]),hgrad=cholSolve(L,grad);
      let lambda=[];
      if(W.length){const LW=cholesky(gram(W));if(!LW)return failure('Contraintes actives dependantes',iteration);lambda=cholSolve(LW,W.map(i=>-dot(C[i],hgrad)));}
      const direction=hgrad.map((a,j)=>-a-W.reduce((s,i,k)=>s+HinvC[i][j]*lambda[k],0));
      if(norm(direction)<=1e-9*(1+norm(z))) {
        const bad=lambda.map((l,k)=>({l,k,index:W[k]})).filter(a=>a.l < -1e-8).sort((a,b)=>a.index-b.index);
        if(bad.length){W.splice(bad[0].k,1);continue;}
        const dual=Array(C.length).fill(0);W.forEach((i,k)=>dual[i]=Math.max(0,lambda[k]));
        const stationarity=norm(grad.map((a,j)=>a+C.reduce((s,row,i)=>s+row[j]*dual[i],0)));
        const primal=Math.max(0,...C.map((row,i)=>dot(row,z)-b[i]));
        const complementarity=Math.max(0,...C.map((row,i)=>Math.abs(dual[i]*(dot(row,z)-b[i]))));
        if(stationarity>2e-6*(1+norm(grad))||primal>2e-7||complementarity>2e-6*(1+norm(grad)))return failure('Certificat KKT non valide',iteration);
        return finish(z,iteration+1,{type:'kkt',stationarity,primal,complementarity,dual,active:W.slice(),velocityProblem:q,velocities:z.slice()});
      }
      let alpha=1,block=-1;
      for(let i=0;i<C.length;i++) {
        if(W.includes(i))continue;
        const slope=dot(C[i],direction);if(slope<=1e-10*norm(direction))continue;
        const step=Math.max(0,(b[i]-dot(C[i],z))/slope);
        if(step<alpha-1e-10 || (Math.abs(step-alpha)<=1e-10 && (block<0||i<block))){
          if(independent(i)){alpha=Math.min(1,step);block=i;}
        }
      }
      z=z.map((a,i)=>a+alpha*direction[i]);
      if(block>=0&&alpha<1-1e-10)W.push(block);
      if(Math.max(0,...C.map((row,i)=>dot(row,z)-b[i]))>2e-7)return failure('Perte de faisabilite numerique',iteration);
    }
    return failure('Limite des iterations atteinte',maxIterations);
  }
  function simulate(options={},steps=100) {
    let state={x:options.x??0,v:options.v??2};const states=[state],controls=[],plans=[];
    for(let k=0;k<steps;k++) {
      const plan=solve({...options,...state});plans.push(plan);
      if(plan.status!=='optimal')return {states,controls,plans,status:plan.status};
      state=advance(state,plan.u[0],options.T??1);states.push(state);controls.push(plan.u[0]);
      if(Math.abs(state.v)<1e-7&&Math.abs(state.x-(options.limit??2))<1e-7)return {states,controls,plans,status:'stopped'};
    }
    return {states,controls,plans,status:'horizon'};
  }
  const api={build,solve,advance,simulate,braking,stoppingDistance,MAX_HORIZON};
  if(typeof module!=='undefined'&&module.exports)module.exports=api;
  else root.MPC=api;
})(typeof globalThis!=='undefined'?globalThis:this);
