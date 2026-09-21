// One request at a time; keep only the latest complete pose while it runs.
// A result belongs to its pose revision, never to whichever pose is visible next.
export class PoseCollisionChecker {
  constructor({request,onState,delay=150,setTimer=(fn,ms)=>setTimeout(fn,ms),clearTimer=id=>clearTimeout(id)}) {
    Object.assign(this,{request,onState,delay,setTimer,clearTimer});
    this.enabled=true;this.pose=null;this.revision=0;this.running=false;
    this.queued=false;this.timer=null;
  }
  cancelPending() {
    if(this.timer!==null)this.clearTimer(this.timer);
    this.timer=null;this.queued=false;
  }
  setPose(pose) {
    this.cancelPending();this.revision++;
    this.pose=pose?{...pose,positions:{...pose.positions}}:null;
    this.onState({state:!pose?'loading':this.enabled?'pending':'unchecked'});
    if(pose&&this.enabled) {
      this.queued=true;
      this.timer=this.setTimer(()=>{this.timer=null;this.drain();},this.delay);
    }
  }
  setEnabled(enabled) {
    if(this.enabled===enabled)return;
    this.enabled=enabled;this.setPose(this.pose);
  }
  checkNow() {
    if(!this.pose)return;
    this.cancelPending();this.revision++;this.queued=true;
    this.onState({state:'pending'});this.drain();
  }
  async drain() {
    if(this.running||this.timer!==null||!this.queued||!this.pose)return;
    const revision=this.revision,pose=this.pose;
    this.queued=false;this.running=true;this.onState({state:'checking'});
    try {
      const result=await this.request(pose);
      if(revision===this.revision)this.onState({state:'checked',result});
    } catch(error) {
      if(revision===this.revision)this.onState({state:'unavailable',error});
    } finally {
      this.running=false;
      if(this.queued&&this.timer===null)this.drain();
    }
  }
}
