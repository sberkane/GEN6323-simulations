#!/usr/bin/env node
'use strict';

// Vérification autonome des données et des commandes de lecture, sans dépendance.
const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert/strict');
const dir=__dirname,core=require(path.join(dir,'app.js'));
const data={};let caseCount=0;
for(const [name,file,globalName,isQuad] of [['differentiel','diff_data','OPEN_LOOP_DIFF',false],['drone','quad_data','OPEN_LOOP_QUAD',true]]){
  data[name]=JSON.parse(fs.readFileSync(path.join(dir,file+'.json'),'utf8'));
  const scope={window:{}};vm.runInNewContext(fs.readFileSync(path.join(dir,file+'.js'),'utf8'),scope);
  assert.equal(JSON.stringify(scope.window[globalName]),JSON.stringify(data[name]),file+': JSON et JavaScript différents');
  assert.equal(core.validateDataset(data[name],isQuad),true);
  for(const e of data[name].experiments){
    assert.equal(core.sampleIndex(e.t,0),0);assert.equal(core.sampleIndex(e.t,e.duration),e.t.length-1);
    const nominal=e.cases.find(c=>c.id==='nominal'),limited=e.cases.find(c=>c.id==='limited');
    assert.ok(nominal&&limited);
    assert.ok(Math.max(...core.positionErrors(e,nominal))<1e-6,'Le nominal doit suivre la référence.');
    for(const c of e.cases){
      caseCount++;
      const bounds=core.sceneBounds(e,c,isQuad);
      assert.ok(Object.values(bounds).every(Number.isFinite));
      assert.ok(bounds.xmin<bounds.xmax&&bounds.ymin<bounds.ymax);
      if(c.id!=='limited')assert.deepEqual(c.input,e.reference.input,'Les efforts non bornés doivent rester les efforts prescrits.');
      const last=core.sampleIndex(e.t,core.stopTime(e,c));
      for(const s of c.state.slice(0,last+1))assert.ok(s[0]>=bounds.xmin&&s[0]<=bounds.xmax&&s[1]>=bounds.ymin&&s[1]<=bounds.ymax,'Le mouvement sort du cadre.');
    }
    const upper=isQuad?data[name].parameters.F_max:data[name].parameters.tau_max;
    const lower=isQuad?0:-upper;
    for(const row of limited.input)for(const u of row)assert.ok(u>=lower-1e-8&&u<=upper+1e-8,'Limite d’effort dépassée.');
  }
}
const bad=JSON.parse(JSON.stringify(data.differentiel));bad.experiments[0].cases[0].state[0][0]=NaN;
assert.throws(()=>core.validateDataset(bad),/incomplet/);
assert.equal(core.stopTime({duration:4},{stop:{t:1.7,reason:'sol'}}),1.7);
assert.equal(core.stopTime({duration:4},{stop:{t:0,reason:'sol'}}),0);

const html=fs.readFileSync(path.join(dir,'index.html'),'utf8');
const css=fs.readFileSync(path.join(dir,'styles.css'),'utf8');
const app=fs.readFileSync(path.join(dir,'app.js'),'utf8');
assert.ok(html.includes('src="diff_data.js"')&&html.includes('src="quad_data.js"'));
assert.ok(!/\bfetch\s*\(/.test(app),'Les données doivent fonctionner sans serveur.');
assert.ok(css.includes('@media(max-width:680px)'));

let drawOperations=0;
class Element{
  constructor(id='',tag='div'){this.id=id;this.tagName=tag;this.children=[];this.attributes={};this.style={};this.listeners={};this._value='';this.disabled=false;this.hidden=false;this.width=0;this.height=0;this.textContent='';this.classList={toggle(){}};}
  set value(v){this._value=String(v);}get value(){return this._value;}
  replaceChildren(...children){this.children=children;if(this.tagName==='select')this._value=children[0]?children[0].value:'';}
  append(...children){this.children.push(...children);}
  setAttribute(k,v){this.attributes[k]=String(v);}
  addEventListener(k,fn){this.listeners[k]=fn;}
  getBoundingClientRect(){return {width:this.id==='scene'?720:430,height:this.id==='scene'?390:170};}
  getContext(){return new Proxy({}, {get:(_,key)=>{if(key==='canvas')return this;return (...args)=>{for(const n of args)if(typeof n==='number')assert.ok(Number.isFinite(n),'Coordonnée de dessin non finie.');drawOperations++;};},set:()=>true});}
}
function browserHarness(hash){
  const elements={};for(const match of html.matchAll(/<([a-z]+)[^>]*\bid="([^"]+)"/g))elements[match[2]]=new Element(match[2],match[1]);
  const nextLink=new Element('next-link','a'),events={},frames=[];
  const document={getElementById:id=>{assert.ok(elements[id],'Élément absent : '+id);return elements[id];},createElement:tag=>new Element('',tag),createTextNode:text=>({textContent:text}),querySelector:selector=>{assert.equal(selector,'.next-link');return nextLink;},addEventListener:(name,fn)=>{events[name]=fn;}};
  const win={OPEN_LOOP_DIFF:data.differentiel,OPEN_LOOP_QUAD:data.drone,devicePixelRatio:1,addEventListener:(name,fn)=>{events[name]=fn;}};
  const location={hash};const context={window:win,document,location,requestAnimationFrame:fn=>{frames.push(fn);return frames.length;},console};
  vm.runInNewContext(app,context);
  return {elements,nextLink,events,frames,location};
}
let scenarioRenders=0;
for(const model of ['differentiel','drone']){
  const h=browserHarness('#'+model),el=h.elements;
  assert.equal(el['model-'+model].attributes['aria-pressed'],'true');
  assert.equal(h.nextLink.attributes.href,'../interactif/index.html#'+(model==='drone'?'quad':'planar'));
  for(const e of data[model].experiments){
    el.duration.value=e.id;el.duration.listeners.change();
    for(const c of e.cases){
      el.case.value=c.id;el.case.listeners.change();
      for(const t of [0,e.duration/2,e.duration]){el.time.value=t;el.time.listeners.input();assert.equal(el.play.attributes['aria-label'],'Lire l’animation');scenarioRenders++;}
      el.reset.listeners.click();assert.equal(el.time.value,'0');
    }
  }
  el.play.listeners.click();assert.equal(el.play.attributes['aria-label'],'Mettre en pause');
  h.frames.shift()(0);h.frames.shift()(100);assert.ok(Number(el.time.value)>0);
  el.play.listeners.click();assert.equal(el.play.attributes['aria-label'],'Lire l’animation');
}
const originalStop=data.drone.experiments[0].cases[0].stop;
data.drone.experiments[0].cases[0].stop={t:1,reason:'Contact avec le sol.'};
const stopped=browserHarness('#drone');assert.equal(stopped.elements.time.max,1);
stopped.elements.time.value=4;stopped.elements.time.listeners.input();
assert.equal(stopped.elements.time.value,'1');assert.equal(stopped.elements['stop-message'].hidden,false);
assert.match(stopped.elements['stop-message'].textContent,/Contact avec le sol/);
data.drone.experiments[0].cases[0].stop=originalStop;
console.log(JSON.stringify({passed:true,datasets:2,cases:caseCount,scenario_renders:scenarioRenders,finite_drawing_operations:drawOperations,checks:['données et unités','JSON = JavaScript','bornes et chemins','routes et sélecteurs','lecture, pause, remise à zéro','arrêt de simulation','usage hors ligne']},null,2));
