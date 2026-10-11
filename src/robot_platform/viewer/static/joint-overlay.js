import * as THREE from 'three';
import {DEFAULT_OVERLAY, JointTelemetry, describeJoint, overlaySettings, rotationArc,
  trustedOverlayMessage} from './joint-telemetry.js';

// Display only. The parent Settings panel owns all persisted overlay preferences.
export class JointOverlay {
  constructor({viewport, camera, renderer, scene, onSelection}) {
    Object.assign(this,{viewport,camera,renderer,scene,onSelection});
    this.settings=overlaySettings(); this.telemetry=new JointTelemetry(); this.cards=new Map();
    this.model=null; this.frames=new Map(); this.meshes=[]; this.radius=1; this.selected=null;
    this.nextTextAt=0; this.pointerDown=null;
    this.container=document.createElement('div'); this.container.className='joint-overlay';
    this.container.setAttribute('aria-label','Joint telemetry on model');
    this.leaders=document.createElementNS('http://www.w3.org/2000/svg','svg');
    this.leaders.classList.add('joint-overlay-leaders');this.leaders.setAttribute('aria-hidden','true');
    this.container.append(this.leaders);
    this.toolbar=document.createElement('div'); this.toolbar.className='joint-overlay-toolbar';
    this.select=document.createElement('select'); this.select.setAttribute('aria-label','Joint shown on model');
    this.status=document.createElement('span'); this.status.className='joint-overlay-status';
    this.status.setAttribute('role','status');
    this.legend=document.createElement('span'); this.legend.className='joint-overlay-legend';
    this.legend.textContent='Position · motor effort · temperature | teal + / amber − motion';
    this.toolbar.append(this.select,this.status,this.legend); viewport.append(this.container,this.toolbar);
    this.select.onchange=()=>this.choose(this.select.value);
    renderer.domElement.addEventListener('pointerdown',event=>{this.pointerDown=[event.clientX,event.clientY];});
    renderer.domElement.addEventListener('pointerup',event=>this.pick(event));
    window.addEventListener('message',event=>{
      if (!trustedOverlayMessage(event,window.parent,location.origin)) return;
      if (event.data.settings) {
        this.settings=overlaySettings(this.settings,event.data.settings);
        this.selected=this.settings.selected_motor; this.nextTextAt=0;
      }
      if (Object.hasOwn(event.data,'telemetry')) {
        this.pendingTelemetry=event.data.telemetry;
        this.pendingReceived=performance.now();
        if (this.model) this.acceptTelemetry();
      }
    });
  }
  setModel(model,motions,meshes,radius) {
    this.disposeCards(); this.telemetry.clear(); this.model=model; this.meshes=meshes;
    this.frames=new Map([...motions].map(([name,motion])=>[name,motion.parent])); this.radius=radius;
    this.nextTextAt=0; if(this.pendingTelemetry) this.acceptTelemetry();
  }
  acceptTelemetry() {
    this.telemetry.accept(this.pendingTelemetry,this.pendingReceived,this.model); this.nextTextAt=0;
  }
  choose(motor) {
    this.selected=motor; this.settings={...this.settings,selected_motor:motor}; this.nextTextAt=0;
    this.onSelection?.(motor);
    if(window.parent !== window) window.parent.postMessage({type:'helm-viewer-selection',version:1,motor},location.origin);
  }
  disposeCards() {
    for(const card of this.cards.values()) {
      card.node.remove(); card.line.remove(); card.arrow.parent?.remove(card.arrow);
      card.arrow.traverse(object=>{object.geometry?.dispose();object.material?.dispose();});
    }
    this.cards.clear();
  }
  clearModel() { this.disposeCards(); this.model=null; this.frames.clear(); this.telemetry.clear(); }
  makeCard(row) {
    const node=document.createElement('div'); node.className='joint-overlay-card';
    const title=document.createElement('strong'), source=document.createElement('small');
    const readings=document.createElement('dl'), age=document.createElement('small');
    node.append(title,source,readings,age); this.container.append(node);
    const line=document.createElementNS('http://www.w3.org/2000/svg','line');this.leaders.append(line);
    const arrow=new THREE.Group(); this.frames.get(row.joint).add(arrow);
    const card={node,title,source,readings,age,arrow,line,joint:row.joint,direction:null};
    this.cards.set(row.motor,card); return card;
  }
  updateArrow(card,row,direction) {
    card.arrow.visible=direction !== 0;
    if(card.direction === direction) return;
    card.direction=direction;
    for(const child of [...card.arrow.children]) {
      child.traverse(object=>{object.geometry?.dispose();object.material?.dispose();});card.arrow.remove(child);
    }
    if(!direction) return;
    const joint=this.model.joints.find(j=>j.name===row.joint), scale=this.radius*.065;
    const color=direction>0 ? 0x8ae1ce : 0xf2c380;
    if(joint.type === 'prismatic') {
      const axis=new THREE.Vector3(...joint.axis).normalize().multiplyScalar(direction);
      const arrow=new THREE.ArrowHelper(axis,new THREE.Vector3(),scale*2.8,color,scale*.45,scale*.22);
      card.arrow.add(arrow);
    } else {
      const arc=rotationArc(joint.axis,direction,scale); if(!arc) return;
      const geometry=new THREE.BufferGeometry().setFromPoints(arc.points.map(p=>new THREE.Vector3(...p)));
      card.arrow.add(new THREE.Line(geometry,new THREE.LineBasicMaterial({color,depthTest:false,transparent:true,opacity:.95})));
      const tip=new THREE.Mesh(new THREE.ConeGeometry(scale*.18,scale*.5,10),
        new THREE.MeshBasicMaterial({color,depthTest:false}));
      tip.position.fromArray(arc.points.at(-1));
      tip.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),new THREE.Vector3(...arc.tangent).normalize());
      card.arrow.add(tip);
    }
    card.arrow.renderOrder=10;
  }
  updateText(now) {
    const snapshot=this.telemetry.snapshot(now), motors=snapshot.rows.map(row=>row.motor);
    if(!this.selected && motors.length) this.selected=motors[0];
    const optionsKey=snapshot.rows.map(row=>`${row.motor}:${row.label}`).join('|');
    if(optionsKey !== this.optionsKey) {
      this.optionsKey=optionsKey; this.select.replaceChildren();
      for(const row of snapshot.rows) {
        const option=document.createElement('option');option.value=row.motor;option.textContent=row.label;this.select.append(option);
      }
    }
    this.select.value=this.selected||''; this.select.disabled=!motors.length;
    this.select.hidden=this.settings.scope === 'all';
    this.toolbar.hidden=this.container.hidden=!this.settings.enabled;
    const visible=this.settings.enabled ? snapshot.rows.filter(row=>this.settings.scope === 'all' || row.motor===this.selected) : [];
    const visibleMotors=new Set(visible.map(row=>row.motor));
    for(const [motor,card] of this.cards) {
      card.node.hidden=!visibleMotors.has(motor); card.arrow.visible=visibleMotors.has(motor) && card.direction !== 0;
      card.line.style.display=card.node.hidden ? 'none' : '';
    }
    const stale=visible.some(row=>row.stale);
    this.status.textContent=!motors.length ? snapshot.reason || 'Waiting for joint telemetry'
      : !visible.length && this.settings.enabled ? 'Selected joint has no reading'
      : `${stale ? 'Stale readings · arrows hidden' : 'Joint telemetry'} · ${this.settings.scope === 'all' ? 'all joints' : 'selected joint'}`;
    this.status.classList.toggle('stale',stale || !motors.length);
    for(const row of visible) {
      const existing=this.cards.get(row.motor);
      if(existing && existing.joint !== row.joint) { this.disposeCards(); }
      const card=this.cards.get(row.motor)||this.makeCard(row), display=describeJoint(row,this.settings.fields);
      card.node.hidden=false; card.node.classList.toggle('stale',row.stale);
      card.line.classList.toggle('stale',row.stale);
      card.title.textContent=row.label; card.source.textContent=`${display.source} · ${row.joint}`;
      card.readings.replaceChildren();
      for(const line of display.lines) {
        const term=document.createElement('dt'), value=document.createElement('dd');
        term.textContent=line.label;value.textContent=line.value;value.className=line.kind;
        card.readings.append(term,value);
      }
      card.age.textContent=[display.status,display.mapping].filter(Boolean).join(' · ');
      this.updateArrow(card,row,display.direction);
    }
  }
  update(now=performance.now()) {
    if(now >= this.nextTextAt) { this.updateText(now); this.nextTextAt=now+100; }
    if(!this.settings.enabled || !this.model) return;
    // Project the URDF origin after world transforms; rotation arrows inherit the
    // same origin orientation, so their axis is the joint's actual world axis.
    const width=this.viewport.clientWidth,height=this.viewport.clientHeight;
    const top=Math.max(145,(this.viewport.querySelector('.viewport-head')?.offsetHeight||100)+30);
    const placed=[];
    for(const card of this.cards.values()) {
      if(card.node.hidden) continue;
      const point=new THREE.Vector3();this.frames.get(card.joint).getWorldPosition(point);point.project(this.camera);
      if(point.z < -1 || point.z > 1 || Math.abs(point.x)>1.15 || Math.abs(point.y)>1.15) {
        card.node.style.visibility='hidden';card.line.style.visibility='hidden';continue;
      }
      card.node.style.visibility='visible';
      card.line.style.visibility='visible';
      const anchorX=(point.x+1)*width/2,anchorY=(1-point.y)*height/2;
      const cardWidth=Math.min(225,width-28),cardHeight=card.node.offsetHeight||142;
      let x=Math.max(12,Math.min(width-cardWidth-12,anchorX+14));
      let y=Math.max(top,Math.min(height-cardHeight-66,anchorY-cardHeight/2));
      // Selected is the uncluttered default; all-mode labels fan vertically.
      for(const box of placed) if(Math.abs(x-box.x)<cardWidth+8 && y<box.y+box.h+8 && y+cardHeight>box.y) y=box.y+box.h+8;
      if(y+cardHeight>height-54) { x=Math.max(12,anchorX-cardWidth-14);y=Math.max(top,Math.min(height-cardHeight-54,anchorY)); }
      placed.push({x,y,h:cardHeight});
      card.node.style.width=`${cardWidth}px`;card.node.style.transform=`translate(${x}px,${y}px)`;
      card.node.style.setProperty('--anchor-x',`${anchorX-x}px`);
      card.node.style.setProperty('--anchor-y',`${anchorY-y}px`);
      card.line.setAttribute('x1',anchorX);card.line.setAttribute('y1',anchorY);
      card.line.setAttribute('x2',Math.max(x,Math.min(x+cardWidth,anchorX)));
      card.line.setAttribute('y2',Math.max(y,Math.min(y+cardHeight,anchorY)));
    }
  }
  pick(event) {
    const down=this.pointerDown;this.pointerDown=null;
    if(!down || !this.settings.enabled || !this.model || Math.hypot(event.clientX-down[0],event.clientY-down[1])>4) return;
    const rect=this.renderer.domElement.getBoundingClientRect(), pointer=new THREE.Vector2(
      (event.clientX-rect.left)/rect.width*2-1,1-(event.clientY-rect.top)/rect.height*2);
    const ray=new THREE.Raycaster();ray.setFromCamera(pointer,this.camera);
    const hit=ray.intersectObjects(this.meshes.filter(mesh=>mesh.visible && mesh.userData.role==='visual'))[0];
    if(!hit) return;
    const rows=this.telemetry.snapshot(performance.now()).rows;
    let link=hit.object.userData.link;
    while(link) {
      const joint=this.model.joints.find(j=>j.child===link); if(!joint) break;
      const row=rows.find(r=>r.joint===joint.name);
      if(row) { this.choose(row.motor);break; }
      link=joint.parent;
    }
  }
}
