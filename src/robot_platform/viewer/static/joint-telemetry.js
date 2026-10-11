// Pure display contract: no motor control, inferred velocity or effort conversion.
export const DEFAULT_OVERLAY = Object.freeze({enabled:true, scope:'selected', selected_motor:null,
  fields:Object.freeze({position:true, effort:true, temperature:true, direction:true})});

const finite = value => typeof value === 'number' && Number.isFinite(value);
export const signedValue = (value, unit, digits=3) => finite(value)
  ? `${value < 0 ? '−' : '+'}${Math.abs(value).toFixed(digits)}${unit ? ` ${unit}` : ''}` : '—';

export function overlaySettings(previous=DEFAULT_OVERLAY, incoming={}) {
  const fields = {...previous.fields};
  for (const key of Object.keys(DEFAULT_OVERLAY.fields)) {
    if (typeof incoming.fields?.[key] === 'boolean') fields[key] = incoming.fields[key];
  }
  return {enabled:typeof incoming.enabled === 'boolean' ? incoming.enabled : previous.enabled,
    scope:['selected','all'].includes(incoming.scope) ? incoming.scope : previous.scope,
    selected_motor:typeof incoming.selected_motor === 'string' || incoming.selected_motor === null
      ? incoming.selected_motor : previous.selected_motor, fields};
}

export function trustedOverlayMessage(event, parentWindow, origin) {
  return event.source === parentWindow && event.origin === origin && event.data?.type === 'helm-viewer-overlay'
    && event.data.version === 1;
}

export class JointTelemetry {
  constructor() { this.clear(); }
  clear(reason='Waiting for joint telemetry') { this.frame=null; this.reason=reason; }
  accept(value, now, model) {
    if (!value || value.model !== model?.id || value.urdf_sha256 !== model?.urdf_sha256) {
      this.clear('Joint telemetry/model mismatch'); return false;
    }
    if (value.sequence == null && Array.isArray(value.rows)) {
      this.clear('Waiting for joint readings'); return false;  // Nothing read yet (or no longer reading).
    }
    if (typeof value.stream_id !== 'string' || !value.stream_id || !Number.isSafeInteger(value.sequence)
        || value.sequence < 0 || !Array.isArray(value.rows) || !finite(now)) {
      this.clear('Joint telemetry identity unavailable'); return false;
    }
    const previous = this.frame;
    if (previous?.stream_id === value.stream_id && value.sequence < previous.sequence) return false;
    const age = finite(value.age_seconds) && value.age_seconds >= 0 ? value.age_seconds : Infinity;
    if (previous?.stream_id === value.stream_id && value.sequence === previous.sequence) {
      // Repeated HTTP/UI heartbeats cannot refresh a frozen device sample.
      const elapsed=Math.max(0,now-previous.received)/1000;
      previous.age = Math.max(previous.age + elapsed, age);
      const repeatedRows=new Map(value.rows.filter(row=>row && typeof row.motor === 'string').map(row=>[row.motor,row]));
      for(const row of previous.rows) {
        const repeated=repeatedRows.get(row.motor);
        row.age=Math.max(row.age+elapsed,finite(repeated?.age_seconds) && repeated.age_seconds >= 0 ? repeated.age_seconds : 0);
        row.stale ||= repeated?.stale === true;
      }
      previous.received = now;
      previous.stale ||= value.stale === true;
      return false;
    }
    const joints = new Map(model.joints.map(joint => [joint.name, joint]));
    const seen = new Set(), rows = [];
    for (const row of value.rows) {
      if (!row || typeof row.motor !== 'string' || !row.motor || typeof row.joint !== 'string'
          || !joints.has(row.joint) || joints.get(row.joint).type === 'fixed' || seen.has(row.motor)) continue;
      seen.add(row.motor);
      const positionUnit = joints.get(row.joint).type === 'prismatic' ? 'm' : 'rad';
      const unitsValid = row.position_unit === positionUnit;
      const rowAge = finite(row.age_seconds) && row.age_seconds >= 0 ? row.age_seconds : age;
      rows.push({motor:row.motor, joint:row.joint, label:typeof row.label === 'string' ? row.label : row.motor,
        position:unitsValid && finite(row.position) ? row.position : null, position_unit:positionUnit,
        velocity:row.velocity_unit === `${positionUnit}/s` && finite(row.velocity) ? row.velocity : null,
        velocity_unit:`${positionUnit}/s`, effort:finite(row.effort) ? row.effort : null,
        effort_unit:['Nm','A'].includes(row.effort_unit) ? row.effort_unit : null,
        effort_label:typeof row.effort_label === 'string' && row.effort_label ? row.effort_label : 'Motor effort',
        temperature_c:finite(row.temperature_c) ? row.temperature_c : null,
        age:Math.max(age,rowAge), stale:row.stale === true, source:row.source,
        mapping_verified:row.mapping_verified === true});
    }
    this.frame = {rows, age, received:now, stream_id:value.stream_id, sequence:value.sequence,
      stale:value.stale === true, stale_after_s:finite(value.stale_after_s) && value.stale_after_s > 0
        ? value.stale_after_s : .75};
    this.reason = rows.length ? '' : 'No mapped joint telemetry';
    return true;
  }
  snapshot(now) {
    if (!this.frame) return {rows:[], stale:true, reason:this.reason};
    const frame=this.frame, elapsed=Math.max(0,now-frame.received)/1000;
    const frameStale=frame.stale || !finite(frame.age) || frame.age+elapsed > frame.stale_after_s;
    return {stale:frameStale, reason:this.reason, rows:frame.rows.map(row => ({...row,
      age_seconds:Math.max(row.age+elapsed,frame.age+elapsed),
      stale:frameStale || row.stale || !finite(row.age) || row.age+elapsed > frame.stale_after_s}))};
  }
}

export function describeJoint(row, fields=DEFAULT_OVERLAY.fields) {
  const source=row.source === 'measured' ? 'Measured' : row.source === 'simulated' ? 'Simulated' : 'Source unavailable';
  const lines=[];
  if (fields.position) lines.push({kind:'position', label:'Joint position', value:signedValue(row.position,row.position_unit)});
  if (fields.effort) lines.push({kind:'effort', label:row.effort_label,
    value:signedValue(row.effort,row.effort_unit,2) + (row.effort !== null && !row.effort_unit ? ' · units unknown' : '')});
  if (fields.temperature) lines.push({kind:'temperature', label:'Motor temperature',
    value:finite(row.temperature_c) ? `${row.temperature_c.toFixed(1)} °C` : '—'});
  let direction=0, motion='Motion unavailable';
  if (row.stale) motion='Stale · direction hidden';
  else if (!row.mapping_verified) motion='Mapping unverified · direction hidden';
  else if (['measured','simulated'].includes(row.source) && finite(row.velocity)) {
    direction=Math.abs(row.velocity) < .001 ? 0 : Math.sign(row.velocity);
    motion=`${source} ${signedValue(row.velocity,row.velocity_unit)}${direction ? '' : ' · near zero'}`;
  }
  if (fields.direction) lines.push({kind:'direction',label:'Motion',value:motion});
  return {source, lines, direction:fields.direction ? direction : 0,
    status:row.stale ? 'STALE · last reading' : !finite(row.age_seconds) ? 'Age unavailable'
      : `${Math.round(row.age_seconds*1000)} ms old`, mapping:row.mapping_verified ? '' : 'Model mapping unverified'};
}

// Positive rotation follows the right-hand rule around the supplied URDF axis.
// Keeping this numeric helper separate lets tests verify both signs and axes.
export function rotationArc(axis, sign, radius, segments=28) {
  const norm=Math.hypot(...axis);
  if (!norm || !Number.isFinite(norm) || ![-1,1].includes(sign)) return null;
  const n=axis.map(v=>v/norm), reference=Math.abs(n[2]) < .9 ? [0,0,1] : [0,1,0];
  const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  const raw=cross(reference,n), length=Math.hypot(...raw), u=raw.map(v=>v/length), v=cross(n,u);
  const points=[];
  for(let i=0;i<=segments;i++) {
    const angle=sign*(-.7*Math.PI+1.4*Math.PI*i/segments);
    points.push(u.map((x,j)=>radius*(x*Math.cos(angle)+v[j]*Math.sin(angle))));
  }
  const end=sign*.7*Math.PI, tangent=u.map((x,j)=>sign*(-x*Math.sin(end)+v[j]*Math.cos(end)));
  return {points,tangent};
}
