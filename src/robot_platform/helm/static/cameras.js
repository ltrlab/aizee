// Camera settings I/O is explicit; live video uses camera_live.js independently.
const $ = id => document.getElementById(id);
const ROLES = {left_wrist:'Left wrist',right_wrist:'Right wrist',head:'Head'};
const CONTROLS = {auto_exposure:'Exposure mode',exposure_time_absolute:'Exposure · 100 µs units',
  gain:'Gain',brightness:'Brightness'};
function element(tag,text,cls) {
  const node=document.createElement(tag);
  if(text!==undefined) node.textContent=text;
  if(cls) node.className=cls;
  return node;
}

export class CameraSettings {
  constructor({request}) {
    this.request=request;this.pending=false;this.active=false;this.cards=new Map();
    $('camera-refresh').onclick=()=>this.refresh();
  }
  updateContext(state) {
    this.context=state;
    this.simulation=state.mode==='simulation';
    $('camera-refresh').disabled=this.pending||this.simulation;
    this.active=['starting','running'].includes(state.teleop?.phase) || !!state.task?.recording;
    for(const card of this.cards.values()) for(const input of card.querySelectorAll('[data-camera-edit]'))
      input.disabled=this.simulation||this.active||this.pending||this.canApply===false||input.dataset.unavailable==='true';
  }
  status(message,bad=false) {
    $('camera-status').textContent=message;$('camera-status').classList.toggle('bad',bad);
  }
  async refresh() {
    if(this.pending||this.simulation) return;
    this.pending=true;$('camera-refresh').disabled=true;this.status('Reading camera settings…');
    try {
      const result=await this.request('/api/helm/cameras',undefined,20000);
      this.render(result);this.status(result.reason || 'Settings read from the cameras. Live preview updates independently.');
    }catch(error){this.status(error.message,true);}
    finally{this.pending=false;$('camera-refresh').disabled=false;this.updateContext(this.context||{});}
  }
  render(result) {
    this.canApply=result.can_apply;
    const host=$('camera-cards');host.replaceChildren();this.cards.clear();
    for(const [role,title] of Object.entries(ROLES)) {
      const state=result.cameras?.[role]||{};
      const card=element('section',undefined,'camera-card');card.append(element('h3',title));
      card.append(element('p',state.error||state.reason||(state.available?'Camera controls available.':'Camera controls unavailable.'),'camera-control-status muted'));
      for(const [name,label] of Object.entries(CONTROLS)) {
        const control=state.controls?.[name];if(!control)continue;
        const row=element('div',undefined,'camera-control');
        const caption=element('label',label);const id=`camera-${role}-${name}`;caption.htmlFor=id;
        let input;
        if(name==='auto_exposure') {
          input=element('select');
          // The backend supplies the camera's supported menu entries, not every integer in its range.
          for(const choice of control.options||[]) {
            const option=element('option',choice.label);option.value=String(choice.value);input.append(option);
          }
          if(!input.options.length){const option=element('option',`Current mode ${control.value}`);option.value=String(control.value);input.append(option);input.dataset.unavailable='true';}
        } else {
          input=element('input');input.type='number';input.inputMode='numeric';
          input.min=control.min;input.max=control.max;input.step=control.step||1;
        }
        input.id=id;input.value=String(control.value);input.dataset.cameraEdit='true';
        if(control.writable===false)input.dataset.unavailable='true';
        const apply=element('button','Apply');apply.dataset.cameraEdit='true';
        apply.dataset.unavailable=input.dataset.unavailable||'false';
        apply.onclick=()=>{if(input.reportValidity())this.apply(role,name,Number(input.value));};
        const current=element('small',`Current ${control.value}${name==='exposure_time_absolute'?` (${control.value/10} ms)`:''}${control.reason?' · '+control.reason:''}`,'muted');
        row.append(caption,input,apply,current);card.append(row);
      }
      this.cards.set(role,card);host.append(card);
    }
  }
  async apply(role,name,value) {
    if(this.pending||this.active||this.simulation) return;
    this.pending=true;this.updateContext(this.context||{});this.status(`Applying ${CONTROLS[name].toLowerCase()} to ${ROLES[role].toLowerCase()}…`);
    let error;
    try {await this.request(`/api/helm/cameras/${role}`,{name,value},20000);}
    catch(e){error=e;}
    finally{this.pending=false;}
    await this.refresh();
    if(error)this.status(`Camera did not confirm the change: ${error.message}`,true);
    else this.status('Camera confirmed the setting. Check the live preview before recording.');
  }
}
