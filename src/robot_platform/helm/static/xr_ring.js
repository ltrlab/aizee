// The headset's ring of controls (xr_scene.js draws it, xr_session.js fills it), worked with a Quest Pro controller's
// own buttons instead of pointing (the user, 2026-11-03): A or X opens the ring; that controller's thumbstick, tilted,
// picks an item around it (clockwise from the top), which stays picked as the stick comes back to the centre; pressing
// the thumbstick down chooses it (the user: pressing A while tilting the stick was awkward). A or X goes back a level,
// or closes the ring at the top. An item that moves the robot asks for the thumbstick held down HOLD_S on it; a
// slider, chosen, takes the stick's sideways tilt until the thumbstick (or A or X) is pressed again. While the ring is
// open, that controller's thumbstick press is the ring's, not the head's zero (xr.js). Nothing the ring uses drives an
// arm: the trigger, grip, B and Y keep their jobs. No DOM and no three.js, so node tests it.

export const DEAD_ZONE = 0.5; // A stick tilted less than this picks nothing new.
export const HOLD_S = 1; // How long an item that moves the robot needs the thumbstick held down.
export const SLIDE_PER_S = 0.5; // A slider's change per second at full tilt.
export const SLIDER_STEP = 0.05;

// The item a stick tilt ([x, y]: xr-standard axes 2 and 3, y down) aims at among `count` around the ring, clockwise
// from the top; null inside the dead zone.
export function aimed([x, y], count) {
  if (!count || Math.hypot(x, y) < DEAD_ZONE) return null;
  const turn = (Math.atan2(x, -y) + 2 * Math.PI) % (2 * Math.PI);
  return Math.round(turn / ((2 * Math.PI) / count)) % count;
}

export class Ring {
  // `menu(path)`: the level at `path` (the ids of the submenus chosen from the top) as {title, items}; each item
  // {id, label, detail?, tone?, disabled? (why not), submenu?, confirm? (lines: held to choose), slider? (its value,
  // 0 to 1), stay? (the ring stays open once it is chosen)}.
  constructor(menu) {
    this.menu = menu;
    this.hand = null; // The controller that opened it, while it is open.
    this.path = [];
    this.aim = null; // The item picked: the last one the stick tilted toward on this level,
    this.aimId = null; // and its id: another item come into its place (the session changed) is not picked.
    this.hold = null; // {id, since}: the thumbstick held down on an item that asks for it.
    this.adjust = null; // {id, value}: a slider taking the stick's tilt.
    this.notice = null; // A word for the operator: why an item is out of reach, or that it needs holding.
    this.menuHeld = {}; // A or X, and the thumbstick's press, at the last frame, by hand.
    this.stickHeld = {};
  }

  get open() {
    return this.hand !== null;
  }

  level() {
    const level = this.menu(this.path);
    if (level) return level;
    this.path = []; // That submenu is gone (the session changed): back to the top.
    this.aim = this.aimId = null;
    return this.menu([]);
  }

  close() {
    this.hand = null;
    this.path = [];
    this.aim = this.aimId = this.hold = this.adjust = this.notice = null;
  }

  // The picked item among the level's, or null; dropped when another item has come into its place.
  picked(items) {
    if (this.aim !== null && items[this.aim]?.id !== this.aimId) this.aim = this.aimId = null;
    return this.aim === null ? null : items[this.aim];
  }

  // One frame of the controllers: [{hand, menu (A or X pressed), press (the thumbstick pressed down), stick [x, y]}],
  // the time and the frame's length (seconds). The items chosen: [{id}], or {id, value} for a slider as it changes and
  // {id, value, done} once let go.
  step(inputs, now, dt = 0) {
    const chosen = [];
    for (const { hand, menu, press, stick } of inputs) {
      const opening = menu && !this.menuHeld[hand],
        click = press && !this.stickHeld[hand];
      this.menuHeld[hand] = menu;
      this.stickHeld[hand] = press;
      if (!this.open) {
        if (opening) {
          this.close();
          this.hand = hand;
          chosen.push({ id: 'opened' });
        }
        continue;
      }
      if (hand !== this.hand) continue;
      const { items } = this.level();
      if (this.adjust) {
        const tilt = Math.abs(stick[0]) >= DEAD_ZONE ? Math.sign(stick[0]) : 0;
        const value = Math.min(1, Math.max(0, this.adjust.raw + tilt * SLIDE_PER_S * dt));
        this.adjust.raw = value;
        const stepped = Number((Math.round(value / SLIDER_STEP) * SLIDER_STEP).toFixed(2));
        if (stepped !== this.adjust.value) {
          this.adjust.value = stepped;
          chosen.push({ id: this.adjust.id, value: stepped });
        }
        if (click || opening) {
          chosen.push({ id: this.adjust.id, value: this.adjust.value, done: true });
          this.adjust = null;
        }
        continue;
      }
      const aim = aimed(stick, items.length);
      if (aim !== null && aim !== this.aim) {
        this.aim = aim;
        this.aimId = items[aim].id;
        this.notice = null;
      }
      const item = this.picked(items);
      if (this.hold) {
        if (!press || opening || item?.id !== this.hold.id) {
          this.notice = press || opening ? null : 'Hold to confirm: let go too soon';
          this.hold = null;
        } else if (now - this.hold.since >= HOLD_S) {
          this.hold = null;
          chosen.push({ id: item.id });
          if (!item.stay) this.close();
        }
        if (!opening) continue;
      }
      if (opening) {
        if (this.path.length) {
          this.path = this.path.slice(0, -1);
          this.aim = this.aimId = null;
          this.notice = null;
        } else {
          this.close();
        }
        continue;
      }
      if (!click) continue;
      if (!item) {
        this.notice = 'Tilt the stick toward an item, then press it down';
      } else if (item.disabled) {
        this.notice = item.disabled;
      } else if (item.submenu) {
        this.path = [...this.path, item.id];
        this.aim = this.aimId = null;
      } else if (item.slider !== undefined) {
        this.adjust = { id: item.id, value: item.slider, raw: item.slider };
      } else if (item.confirm) {
        this.hold = { id: item.id, since: now };
      } else {
        chosen.push({ id: item.id });
        if (!item.stay) this.close();
      }
    }
    return chosen;
  }

  // What the scene draws: the level's title and items, the picked one, a hold's progress (0 to 1, in twentieths: the
  // scene draws the ring again only when it changes), the notice (or why the picked item is out of reach) and a
  // slider taking the stick; null while closed.
  drawing(now) {
    if (!this.open) return null;
    const { title, items } = this.level();
    const aim = this.picked(items);
    return {
      title,
      items,
      aim: this.aim,
      notice: this.notice || aim?.disabled || null,
      depth: this.path.length,
      confirm: aim?.confirm && !aim.disabled ? aim.confirm : null,
      progress: this.hold ? Math.floor(Math.min(1, (now - this.hold.since) / HOLD_S) * 20) / 20 : 0,
      adjust: this.adjust ? { id: this.adjust.id, value: this.adjust.value } : null,
    };
  }
}
