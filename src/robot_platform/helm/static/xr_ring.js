// The headset's ring of controls (xr_scene.js draws it, xr_session.js fills it), worked with a Quest Pro controller's
// own buttons instead of pointing (the user, 2026-11-03): A or X opens the ring; that controller's thumbstick, tilted,
// aims at an item around it (clockwise from the top); A or X again chooses it. Chosen with the stick centred, it goes
// back a level, or closes the ring at the top. An item that moves the robot asks for A or X held HOLD_S while the
// stick stays on it; a slider, chosen, takes the stick's sideways tilt until A or X is pressed again. Nothing the
// ring uses drives an arm: the trigger, grip, B and Y and the thumbstick's press keep their jobs. No DOM and no
// three.js, so node tests it.

export const DEAD_ZONE = .5;  // A stick tilted less than this aims at nothing.
export const HOLD_S = 1;  // How long an item that moves the robot needs A or X held.
export const SLIDE_PER_S = .5;  // A slider's change per second at full tilt.
export const SLIDER_STEP = .05;

// The item a stick tilt ([x, y]: xr-standard axes 2 and 3, y down) aims at among `count` around the ring, clockwise
// from the top; null inside the dead zone.
export function aimed([x, y], count) {
  if (!count || Math.hypot(x, y) < DEAD_ZONE) return null;
  const turn = (Math.atan2(x, -y) + 2 * Math.PI) % (2 * Math.PI);
  return Math.round(turn / (2 * Math.PI / count)) % count;
}

export class Ring {
  // `menu(path)`: the level at `path` (the ids of the submenus chosen from the top) as {title, items}; each item
  // {id, label, detail?, tone?, disabled? (why not), submenu?, confirm? (lines: held to choose), slider? (its value,
  // 0 to 1), stay? (the ring stays open once it is chosen)}.
  constructor(menu) {
    this.menu = menu;
    this.hand = null;  // The controller that opened it, while it is open.
    this.path = [];
    this.aim = null;
    this.hold = null;  // {id, since}: A or X held on an item that asks for it.
    this.adjust = null;  // {id, value}: a slider taking the stick's tilt.
    this.notice = null;  // A word for the operator: why an item is out of reach, or that it needs holding.
    this.pressed = {};
  }

  get open() {
    return this.hand !== null;
  }

  level() {
    const level = this.menu(this.path);
    if (level) return level;
    this.path = [];  // That submenu is gone (the session changed): back to the top.
    return this.menu([]);
  }

  close() {
    this.hand = null;
    this.path = [];
    this.aim = this.hold = this.adjust = this.notice = null;
  }

  // One frame of the controllers: [{hand, button (A or X pressed), stick [x, y]}], the time and the frame's length
  // (seconds). The items chosen: [{id}], or {id, value} for a slider as it changes and {id, value, done} once let go.
  step(inputs, now, dt = 0) {
    const chosen = [];
    for (const {hand, button, stick} of inputs) {
      const press = button && !this.pressed[hand];
      this.pressed[hand] = button;
      if (!this.open) {
        if (press) {
          this.close();
          this.hand = hand;
          chosen.push({id: 'opened'});
        }
        continue;
      }
      if (hand !== this.hand) continue;
      const {items} = this.level();
      if (this.adjust) {
        const tilt = Math.abs(stick[0]) >= DEAD_ZONE ? Math.sign(stick[0]) : 0;
        const value = Math.min(1, Math.max(0, this.adjust.raw + tilt * SLIDE_PER_S * dt));
        this.adjust.raw = value;
        const stepped = Number((Math.round(value / SLIDER_STEP) * SLIDER_STEP).toFixed(2));
        if (stepped !== this.adjust.value) {
          this.adjust.value = stepped;
          chosen.push({id: this.adjust.id, value: stepped});
        }
        if (press) {
          chosen.push({id: this.adjust.id, value: this.adjust.value, done: true});
          this.adjust = null;
        }
        continue;
      }
      const aim = aimed(stick, items.length);
      if (aim !== this.aim) {
        this.aim = aim;
        this.notice = null;
      }
      const item = aim === null ? null : items[aim];
      if (this.hold) {
        if (!button || item?.id !== this.hold.id) {
          this.notice = button ? null : 'Hold to confirm: let go too soon';
          this.hold = null;
        } else if (now - this.hold.since >= HOLD_S) {
          this.hold = null;
          chosen.push({id: item.id});
          if (!item.stay) this.close();
        }
        continue;
      }
      if (!press) continue;
      if (!item) {
        if (this.path.length) {
          this.path = this.path.slice(0, -1);
          this.aim = null;
        } else {
          this.close();
        }
      } else if (item.disabled) {
        this.notice = item.disabled;
      } else if (item.submenu) {
        this.path = [...this.path, item.id];
        this.aim = null;
      } else if (item.slider !== undefined) {
        this.adjust = {id: item.id, value: item.slider, raw: item.slider};
      } else if (item.confirm) {
        this.hold = {id: item.id, since: now};
      } else {
        chosen.push({id: item.id});
        if (!item.stay) this.close();
      }
    }
    return chosen;
  }

  // What the scene draws: the level's title and items, the aimed one, a hold's progress (0 to 1, in twentieths: the
  // scene draws the ring again only when it changes), the notice and a slider taking the stick; null while closed.
  drawing(now) {
    if (!this.open) return null;
    const {title, items} = this.level();
    const aim = this.aim === null ? null : items[this.aim] || null;
    return {title, items, aim: this.aim, notice: this.notice, depth: this.path.length,
      confirm: aim?.confirm && !aim.disabled ? aim.confirm : null,
      progress: this.hold ? Math.floor(Math.min(1, (now - this.hold.since) / HOLD_S) * 20) / 20 : 0,
      adjust: this.adjust ? {id: this.adjust.id, value: this.adjust.value} : null};
  }
}
