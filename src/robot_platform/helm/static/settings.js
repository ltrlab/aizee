// Settings stay outside the control loop. Drafts only change runtime via an explicit Apply.
const $ = id => document.getElementById(id);
const DEFAULT_DISPLAY = {enabled:true, scope:'selected', selected_motor:null,
  fields:{position:true, effort:true, temperature:true, direction:true}};
const clone = value => JSON.parse(JSON.stringify(value));
const finite = value => typeof value === 'number' && Number.isFinite(value);
const displayValue = value => value == null ? 'Not available' : typeof value === 'object' ? JSON.stringify(value)
  : typeof value === 'boolean' ? value ? 'On' : 'Off' : finite(value) ? String(Number(value.toPrecision(7))) : String(value);
const same = (a,b) => JSON.stringify(a) === JSON.stringify(b);
const EFFECTS = {next_session:'Applies at next Start', read_only:'Read-only',
  hold_receiver_ack:'Hold → Apply → Engage'};

export function validDisplay(value) {
  const result = clone(DEFAULT_DISPLAY);
  if (!value || typeof value !== 'object') return result;
  if (typeof value.enabled === 'boolean') result.enabled = value.enabled;
  if (['selected','all','model'].includes(value.scope)) result.scope = value.scope;
  if (typeof value.selected_motor === 'string' && value.selected_motor.length < 100) result.selected_motor = value.selected_motor;
  for (const key of Object.keys(result.fields)) if (typeof value.fields?.[key] === 'boolean') result.fields[key] = value.fields[key];
  return result;
}

export class SettingsPanel {
  constructor({request,navigate,onDisplay}) {
    Object.assign(this,{request,navigate,onDisplay});
    this.state=null; this.draft=new Map(); this.inputs=new Map(); this.category='motors';
    this.pending=false; this.conflict=false; this.serial=0; this.search=''; this.lastFetch=0;
    this.display=clone(DEFAULT_DISPLAY); this.robot=null; this.context=null;
    $('settings-search').oninput=event => {this.search=event.target.value.trim().toLowerCase();this.filter();};
    $('settings-apply').onclick=()=>this.apply('apply');
    $('settings-reset').onclick=()=>this.apply('reset');
    $('settings-revert').onclick=()=>{this.resetDraft();this.feedback('Draft edits reverted.');};
    for (const input of document.querySelectorAll('[data-display]')) input.onchange=()=>{
      const field=input.dataset.display;
      if (field==='enabled') this.display.enabled=input.checked;
      else if (field==='scope') this.display.scope=input.value;
      else this.display.fields[field]=input.checked;
      this.saveDisplay();
    };
    $('display-reset').onclick=()=>{this.display=clone(DEFAULT_DISPLAY);this.saveDisplay();this.renderDisplay();};
    $('settings-nav').onkeydown=event=>{
      if (!['ArrowDown','ArrowUp','Home','End'].includes(event.key)) return;
      const buttons=[...$('settings-nav').querySelectorAll('button')],index=buttons.indexOf(document.activeElement);
      if (index<0) return;
      event.preventDefault();
      const next=event.key==='Home'?0:event.key==='End'?buttons.length-1:(index+(event.key==='ArrowDown'?1:buttons.length-1))%buttons.length;
      buttons[next].focus();buttons[next].click();
    };
    this.renderNavigation();
  }
  updateContext(context) {
    this.context=context;
    if (this.robot!==context.robot.asset_id) {
      this.robot=context.robot.asset_id;
      try {this.display=validDisplay(JSON.parse(localStorage.getItem(`helm-display:${this.robot}`)));}
      catch {this.display=clone(DEFAULT_DISPLAY);}
      this.renderDisplay();this.onDisplay(this.display);
    }
    this.actions();
    if (document.body.dataset.tab==='settings' && performance.now()-this.lastFetch>2000) this.refresh();
  }
  selectMotor(motor) {this.display.selected_motor=motor;this.saveDisplay();}
  saveDisplay() {
    try {localStorage.setItem(`helm-display:${this.robot}`,JSON.stringify(this.display));} catch {/* optional storage */}
    this.onDisplay(this.display);
  }
  renderDisplay() {
    for (const input of document.querySelectorAll('[data-display]')) {
      const key=input.dataset.display;
      if (key==='scope') input.value=this.display.scope;
      else input.checked=key==='enabled'?this.display.enabled:this.display.fields[key];
    }
  }
  async refresh() {
    if (this.pending || this.fetching) return;
    this.fetching=true;this.lastFetch=performance.now();const serial=++this.serial;
    try {
      const state=await this.request('/api/helm/settings');
      if (serial!==this.serial) return;
      this.accept(state);
    } catch(error) {this.feedback(error.message,true);}
    finally {this.fetching=false;}
  }
  fields() {return (this.state?.sections||[]).flatMap(section=>section.fields||[]);}
  changedFields() {
    return this.fields().filter(field=>field.editable
      && (!same(field.value,field.default) || (this.draft.has(field.id) && !same(this.draft.get(field.id),field.value))));
  }
  edits() {return this.fields().filter(field=>field.editable && !same(this.draft.get(field.id),field.value));}
  resetDraft() {
    this.draft=new Map(this.fields().map(field=>[field.id,clone(field.value)]));this.conflict=false;
    for (const [id,input] of this.inputs) {
      if (input.type==='checkbox') input.checked=!!this.draft.get(id);
      else input.value=displayValue(this.draft.get(id));
    }
    this.actions();
  }
  accept(state) {
    const changed=this.state && (state.session_id!==this.state.session_id || state.revision!==this.state.revision);
    if (this.state?.session_id===state.session_id && state.revision<this.state.revision) return;
    if (changed && this.edits().length) this.conflict=true;
    const signature=JSON.stringify((state.sections||[]).map(section=>[section.id,section.fields.map(f=>[f.id,f.editable,f.type])]));
    const rebuild=!this.state || signature!==this.signature;
    this.state=state;
    if (!this.draft.size || (changed&&!this.conflict)) this.resetDraft();
    if (rebuild) {this.signature=signature;this.buildFields();this.renderNavigation();}
    for (const field of this.fields()) {
      const row=this.inputs.get(field.id)?.closest('.setting-field');
      if (row) row.querySelector('.setting-current').textContent=`Current ${displayValue(field.value)}${field.unit?' '+field.unit:''} · ${field.source||'profile'}`;
    }
    this.actions();this.filter();
  }
  renderNavigation() {
    const host=$('settings-nav');host.replaceChildren();
    const sections=this.state?.sections||[],session=sections.filter(section=>section.fields?.some(field=>field.editable));
    const groups=[['Session',[{id:'changed',label:'Changed from profile'},{id:'motors',label:'Motor tuning'},...session]],
      ['Read-only',sections.filter(section=>!session.includes(section))],['View',[{id:'display',label:'3D display'}]]];
    for (const [title,items] of groups) {
      if (!items.length) continue;
      const heading=document.createElement('div');heading.className='rail-group';heading.textContent=title;host.append(heading);
      for (const section of items) {
        const button=document.createElement('button');button.textContent=section.label;button.dataset.settingsSection=section.id;
        if (section.id==='changed') {const count=document.createElement('span');count.className='rail-count';button.append(count);}
        button.onclick=()=>this.show(section.id);host.append(button);
      }
    }
    this.filter();
  }
  show(category='motors') {
    this.category=category;this.search='';$('settings-search').value='';this.changedShown=null;this.filter();
    $('settings-content').scrollTop=0;
    if (!this.state) this.refresh();
  }
  buildFields() {
    const host=$('settings-fields');host.replaceChildren();this.inputs.clear();
    for (const section of this.state.sections) {
      const group=document.createElement('section');group.className='settings-group';group.dataset.section=section.id;
      const heading=document.createElement('h2');heading.textContent=section.label;group.append(heading);
      if (section.description) {const text=document.createElement('p');text.className='muted';text.textContent=section.description;group.append(text);}
      for (const field of section.fields) {
        const row=document.createElement('div');row.className='setting-field';row.dataset.field=field.id;row.dataset.search=[field.label,field.id,field.unit,field.description,field.reason,field.source].join(' ').toLowerCase();
        const label=document.createElement('label'),title=document.createElement('span');title.className='setting-label';title.textContent=field.label;label.append(title);
        const unit=document.createElement('small');unit.textContent=field.unit||'';title.append(unit);
        if (field.editable) {
          const input=document.createElement('input');input.id=`setting-${this.inputs.size}`;label.htmlFor=input.id;
          input.type=['boolean','bool'].includes(field.type)?'checkbox':'number';input.step=['integer','int'].includes(field.type)?'1':'any';
          const bounds=field.bounds;
          if (bounds) {const lo=Array.isArray(bounds)?bounds[0]:bounds.min;const hi=Array.isArray(bounds)?bounds[1]:bounds.max;if(finite(lo))input.min=lo;if(finite(hi))input.max=hi;}
          if (input.type==='checkbox') input.checked=!!this.draft.get(field.id);else {input.value=this.draft.get(field.id);input.required=true;}
          input.setAttribute('aria-label',`${field.label}${field.unit?' · '+field.unit:''}`);
          input.oninput=()=>{this.draft.set(field.id,input.type==='checkbox'?input.checked:input.value===''?null:Number(input.value));this.actions();};
          label.append(input);this.inputs.set(field.id,input);
        } else {
          const output=document.createElement('output');output.textContent=displayValue(field.value);label.append(output);
        }
        const detail=document.createElement('div');detail.className='setting-details';
        const current=document.createElement('p');current.className='setting-current';current.textContent=`Current ${displayValue(field.value)}${field.unit?' '+field.unit:''} · ${field.source||'profile'}`;
        const note=document.createElement('p');note.textContent=field.description||field.reason||'';
        const effect=document.createElement('span');effect.className=`setting-effect${field.editable?' editable':''}`;effect.textContent=EFFECTS[field.effect]||field.effect||(field.editable?'Applies at next Start':'Read-only');
        const defaults=document.createElement('small');defaults.textContent=`Default ${displayValue(field.default)}${field.unit?' '+field.unit:''}`;
        detail.append(current,note,effect,defaults);row.append(label,detail);
        if(field.workflow){const link=document.createElement('button');link.className='setting-workflow';link.textContent=field.workflow.label||'Open workspace';link.onclick=()=>{const target=typeof field.workflow==='string'?field.workflow:field.workflow.tab;if(target==='manual'||target==='motors'){this.show('motors');}else this.navigate(target);};row.append(link);}
        group.append(row);
      }
      host.append(group);
    }
  }
  filter() {
    for(const button of document.querySelectorAll('[data-settings-section]')) {
      if(button.dataset.settingsSection===this.category&&!this.search) button.setAttribute('aria-current','true');
      else button.removeAttribute('aria-current');
    }
    $('settings-motors').hidden=!!this.search||this.category!=='motors';
    $('settings-display').hidden=!!this.search||this.category!=='display';
    // Rows stay listed while this view is open, so one edited back to its profile value does not vanish mid-edit.
    let count=0,changed=null;
    if(this.category==='changed'&&!this.search) {
      changed=this.changedShown||=new Set();
      for(const field of this.changedFields()) changed.add(field.id);
    }
    for(const group of document.querySelectorAll('#settings-fields .settings-group')) {
      let visible=0;
      for(const row of group.querySelectorAll('.setting-field')) {
        row.hidden=this.search?!row.dataset.search.includes(this.search):changed?!changed.has(row.dataset.field):false;
        if(!row.hidden)visible++;
      }
      group.hidden=this.search||changed?!visible:group.dataset.section!==this.category;if(!group.hidden)count+=visible;
    }
    $('settings-empty').hidden=!(this.search||changed)||count>0;
    $('settings-empty').textContent=changed
      ? 'Nothing differs from the profile: every session setting is at its profile value.'
      : 'No settings match. Try a motor name, “speed”, or “timeout”.';
    $('settings-footer').hidden=!this.search&&['motors','display'].includes(this.category);
  }
  feedback(message,bad=false) {const node=$('settings-status');node.textContent=message;node.classList.toggle('bad',bad);}
  actions() {
    const active=['starting','running'].includes(this.context?.teleop?.phase)||['starting','running'].includes(this.context?.tune?.phase)||this.context?.check?.running;
    const changes=this.edits();const allowed=!!this.state?.can_apply&&!active&&!this.pending;
    const valid=[...this.inputs.values()].every(input=>input.checkValidity());
    $('settings-apply').disabled=!allowed||this.conflict||!changes.length||!valid;
    $('settings-reset').disabled=!allowed||this.conflict||!!changes.length;
    $('settings-revert').disabled=this.pending||(!changes.length&&!this.conflict);
    for(const [id,input] of this.inputs){input.disabled=!allowed;input.closest('.setting-field').dataset.edited=String(changes.some(field=>field.id===id));}
    const count=document.querySelector('#settings-nav .rail-count');
    if(count) count.textContent=this.state?String(this.changedFields().length||''):'';
    const edited=new Set((this.state?.sections||[]).filter(section=>section.fields.some(field=>changes.includes(field))).map(section=>section.id));
    for(const button of document.querySelectorAll('[data-settings-section]')) button.dataset.edited=String(edited.has(button.dataset.settingsSection));
    $('settings-revision').textContent=this.state?`Session settings · revision ${this.state.revision}`:'Loading settings…';
    $('settings-draft').textContent=this.conflict?'Settings changed elsewhere · revert to reload':changes.length?`${changes.length} unapplied change${changes.length===1?'':'s'}`:'All changes applied';
    $('settings-apply-note').textContent=active?'Stop the active task to change session settings. Motor gains can still be applied in Hold.':this.state?.reason||'Applied values are saved for the next session. No motor commands are sent.';
  }
  async apply(action) {
    if(this.pending||!this.state||this.conflict) return;
    if($(action==='apply'?'settings-apply':'settings-reset').disabled) return;
    const payload={session_id:this.state.session_id,revision:this.state.revision};
    if(action==='apply') payload.values=Object.fromEntries(this.edits().map(field=>[field.id,this.draft.get(field.id)]));
    this.pending=true;this.actions();++this.serial;
    try {
      const state=await this.request(`/api/helm/settings/${action}`,payload);
      this.state=state;this.changedShown=null;this.resetDraft();this.buildFields();this.renderNavigation();
      this.feedback(action==='apply'?'Session settings saved. They take effect at the next Start.':'Session overrides reset to profile defaults.');
    }catch(error){this.feedback(`${error.message} Draft edits are retained.`,true);}
    finally{this.pending=false;this.actions();this.filter();}
  }
}
