/* Validation du JavaScript hors navigateur; aucune assertion de rendu visuel. */
'use strict';
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict');
const dir=__dirname,html=fs.readFileSync(path.join(dir,'chapitre4_simulation.html'),'utf8');
assert(!html.includes('__SIM_DATA__'));
const js=html.split('<script>')[1].split('</script>')[0];
const noop=()=>{};const ctx=new Proxy({}, {get:(obj,key)=>obj[key]||noop,set:(obj,key,value)=>{obj[key]=value;return true}});
const els={};function get(id){if(!els[id])els[id]={textContent:'',value:id==='speed'?'1':'0',checked:id.startsWith('show'),classList:{toggle:noop},setAttribute:noop,getBoundingClientRect:()=>({width:840,height:id==='error'?135:390}),getContext:()=>ctx};return els[id]}
const sandbox={document:{getElementById:get},window:{devicePixelRatio:1,addEventListener:noop},requestAnimationFrame:noop};vm.createContext(sandbox);vm.runInContext(js,sandbox);
let checkedSamples=0;
for(const T of [32,8]){
 vm.runInContext(`select(${T})`,sandbox);
 for(let k=0;k<=1000;k++){
  const result=vm.runInContext(`elapsed=T*${k}/1000; ['ideal','lag','limited'].map(m=>sample(m))`,sandbox);
  for(const r of result)for(const v of Object.values(r))for(const x of v)assert(Number.isFinite(x));
  checkedSamples+=3;
 }
 // Exécuter le rendu complet en début, milieu et fin sans environnement UI.
 for(const u of [0,.25,.5,.75,1])vm.runInContext(`elapsed=T*${u};render()`,sandbox);
 assert.equal(els.duration.textContent,`T = ${T} s`);
 assert.equal(els.phase.textContent,'t / T = 1,000');
 assert(Number.isFinite(Number(els.err.textContent.replace(',','.'))));
 vm.runInContext("$('scrub').value='375';$('scrub').oninput()",sandbox);
 assert.equal(vm.runInContext('elapsed/T',sandbox),.375);
}
vm.runInContext("select(8);$('play').onclick();tick(1000);tick(2000)",sandbox);
assert.equal(vm.runInContext('elapsed',sandbox),1);
vm.runInContext("$('play').onclick()",sandbox);assert.equal(vm.runInContext('playing',sandbox),false);
vm.runInContext("$('reset').onclick()",sandbox);assert.equal(vm.runInContext('elapsed',sandbox),0);
const result={pass:true,checkedSamples,tests:['syntaxe JavaScript','données embarquées','interpolation finie, trois modèles et deux durées','rendu sans interface avec Canvas simulé','curseur temporel','lecture, pause et retour au départ'],visualBrowserInspection:false};
fs.writeFileSync(path.join(dir,'chapitre4_animation_verification.json'),JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result));
