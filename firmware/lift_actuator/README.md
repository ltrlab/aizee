# Lift actuator firmware

Minerva's `lift` joint: a 12 V desk-leg column that cannot back-drive, on an Arduino Uno, a DROK H-bridge, an INA219
and an AS5600. The Uno talks to its receiver over USB serial, the way the head's servos do. It does not touch the arms'
CAN buses.

The receiver (`lift_controller`) and its Helm card are not written yet. Until they are, `lift_bench` is the only way to
drive this.

Nothing here has run on the real hardware. The logic is tested on a simulated ATmega328P against a made-up column, and
`src/board.cpp`, which talks to the actual chips and pins, has only booted in that simulator, with no chips attached.

## How it behaves

- **Nothing moves at power-up, ever.** It starts disabled and unhomed. Homing runs only when asked.
- **Every stop coasts.** The column holds itself, so the motor just turns off.
- **Every fault latches** until a CLEAR while disabled. A fault that loses the position (encoder, magnet) also loses
  homing.
- **It refuses HOME and ENABLE while `include/config.h` has zeros in it.** Those numbers have to be measured.
- A reset, whatever caused it, comes back disabled and unhomed. The AVR watchdog resets the board after 60 ms without a
  tick.

| Trip                | When                                                                | Mode                |
| ------------------- | ------------------------------------------------------------------- | ------------------- |
| Overcurrent         | Over `tripMilliamps` for 20 ms, or the shunt reads at its ceiling   | Always              |
| Stall               | Driven at stiction or more, and under `stallMinCounts` in 300 ms    | Enabled, homing     |
| Wrong direction     | Driven one way, `wrongDirectionCounts` the other way for 100 ms     | Moving              |
| Encoder             | 3 failed reads in a row, or an impossible jump (the jump: always)   | Moving              |
| Magnet              | The AS5600 reports no magnet for 60 ms                              | Moving              |
| Current sensor      | 3 failed INA219 reads in a row                                      | Moving              |
| Limit switch        | Open in the direction of travel. Moving away from it is allowed.    | Moving              |
| Both switches open  | Most likely a broken ground                                         | Moving              |
| Undervoltage        | Under `minMillivolts` for 60 ms                                     | Moving              |
| No supply           | Under `noSupplyMillivolts` for 60 ms: the kill switch or the supply | Moving              |
| Duty                | 2 minutes' running in about 20, as a leaky bucket                   | Always              |
| Command timeout     | 100 ms without a MOVE (enabled) or any frame (homing)               | Enabled, homing     |
| Home timeout        | Longer than the full travel should take, and never under 30 s       | Homing              |
| Moved with motor off | `restMotionCounts` of motion 300 ms after the motor stopped        | At rest             |

The failures marked "Moving" do not latch at rest. They refuse HOME and ENABLE instead, until they are gone.

The 2 minutes in 20 is the usual rating for a desk leg's motor, not this one's. Check its label if it has one.

## Wiring

Power, in this order: supply + → latching kill switch → fuse → INA219 VIN+ → VIN− → DROK power +. Supply − → DROK
power GND, which is also the only point the Uno's ground joins the motor side. 1000 µF, 25 V across the DROK's input.
18 AWG for the motor and its supply. The fuse rating comes from the current measurement below; start with 5 A slow-blow.

| Uno  | Goes to                                                              |
| ---- | -------------------------------------------------------------------- |
| D3   | DROK ENA, with 10 kΩ from ENA to ground (timer 2, about 3.9 kHz)     |
| D4   | DROK IN1                                                             |
| D7   | DROK IN2                                                             |
| D8   | Top switch: COM to the pin, NC to ground                             |
| D9   | Bottom switch: COM to the pin, NC to ground                          |
| A4   | SDA to the INA219 (0x40) and the AS5600 (0x36, DIR to ground)        |
| A5   | SCL to both, twisted with SDA, 30 cm at most                         |
| 5 V  | INA219 and AS5600 VCC, and the DROK's signal 5 V if the board has it |
| GND  | INA219, AS5600, DROK signal ground                                   |
| RESET | 10 µF to ground once flashed, so opening the port does not reset it. Remove it to flash. |
| USB  | Through an ADuM3160-class isolator, to a Jetson port that is not on the wrist cameras' hub |

Check the DROK's printed labels against this before powering it; they have not been confirmed against the board. Check
too that `/dev/serial/by-id` shows a serial number for the Uno, since the receiver pins it by that.

The switches are normally closed to ground, so a broken wire reads as "at the limit", in both directions if it is the
ground.

## Bring-up

Flash and test from this folder. Run each step with the column clear of anything it could hit, and the kill switch in
reach.

```bash
python -m platformio run -e lift_bench -t upload
```

```bash
python -m platformio device monitor -e lift_bench
```

`?` in the monitor lists the commands. Any key stops whatever is moving. Printing is slow enough to miss a tick, so a
few `late=` here are expected; the real build should show none. A `u` or `d` line ends with what it measured:

```text
jog pwm=255 ms=2000 moved=4120 cps=2210 meanMa=1340 peakMa=1720 lowestMv=11810 fault=none
```

1. **Logic only**, with no 12 V and the motor disconnected. `w` prints the state five times a second.
   - Both chips should say `ok`, `mV` near zero, and pressing each switch flips its `top=` or `bottom=`.
   - Turn the magnet by hand and watch `counts`. Moving it trips "moved with the motor off", which is the guard working;
     `c` clears it.
   - Switches not fitted yet: jumper D8 and D9 to ground, keep the column away from its ends, and remember nothing stops
     it there except the current trip.
2. **First jogs, column disconnected from any load if you can.** `u 120 500`. A `wrong direction` fault says which flag
   to change in `config.h`: `encoderInverted` if the column did go up, `motorInverted` if it did not. Reflash and repeat
   until `u` raises it with `moved` positive.
3. **Measure, with the arms and head on.** Go up and down separately, since the load is on one side only.
   - `stictionPwmUp`, `stictionPwmDown`: raise the PWM (`u 40`, `u 60`, ...) until `moved` is steady over repeats. Use
     the value that always moves, not the first that sometimes does.
   - `fullSpeedUpCps`, `fullSpeedDownCps`: `u 255 2000` and `d 255 2000`, the `cps` value.
   - `maxSpeedCps`: below both full speeds, about 80 % of the lower one. `homeSpeedCps`: about half of that.
   - `travelCounts`, `clearanceCounts`: from the bottom switch's edge up to the top you allow, and a few millimetres'
     worth for the clearance. Watch `counts` with `w`; before homing only the differences mean anything.
4. **Set the current numbers from `peakMa`.** Set `tripMilliamps` and the fuse to about 1.5× the running current.
   If running current is over about 2.5 A the 0.1 Ω shunt is too small: fit about 0.02 Ω and change `shuntMilliohms`.
   If the supply sags under `minMillivolts` when it starts (`lowestMv`), fix the supply rather than the number.
5. **Fill in `include/config.h`** and reflash. `h` should no longer refuse with "config.h still has zeros".
6. **Homing:** `h`. It goes down to the bottom switch, backs off, comes back at half speed, zeroes on the edge and stops
   `clearanceCounts` above it. The arms must be somewhere that is safe to lower them first; nothing here knows where they
   are.
7. **Closed loop:** `g 2000`, then further. It should arrive and go quiet: `pwm=0`, no hunting. Tune `kpPwmPerKilocount`
   and `deadbandCounts` here.
8. **Duty and faults on purpose:** pull the kill switch during a jog (`no supply`), press a switch during a move, unplug
   the magnet's board. Each should stop it at once and need a `c`.

Then flash the real build for the receiver:

```bash
python -m platformio run -e lift -t upload
```

## Tests

```bash
python -m platformio test -e sim
```

These run on `simavr`, a simulated ATmega328P, so they catch what a desktop compiler would hide: the Uno's `int` is 16
bits. `test/plant.h` is a made-up column; its numbers are not measurements. The suites are `test_link` (the framing and
reply layout), `test_motion` (the encoder, homing, moves, soft start, reversal) and `test_guards` (every trip).

## Protocol

`lib/lift_core/protocol.h` is the reference, and the receiver has to match it: USB serial at 1 Mbaud, frames of
`A5 5A | type | length | payload | CRC-16/CCITT-FALSE`, little-endian, and the board only ever answers. Positions cross
it as raw counts above the bottom switch. Metres belong to the receiver, through its profile.

`IDENTIFY` returns `config CRC`, a checksum of the whole `config.h`, so a receiver can notice a board flashed with
different numbers than it expects.

A `MOVE` outside `clearance..travel` is refused, and a refused `MOVE` does not count as hearing from the host. Homing
stops inside the deadband of `clearance`, so possibly just below it: a receiver has to clamp a "stay here" target into
the range `IDENTIFY` reports, or it will be refused and then time out.

## Layout

- `lib/lift_core/`: everything that does not touch hardware, so the tests can run it: `protocol.h`, `link` (framing),
  `encoder`, `control`, `guards`, `lift` (modes and homing).
- `src/board.cpp`: the pins, the PWM timer, the I2C reads.
- `src/main.cpp`: the receiver's build. `src/bench.cpp`: the bring-up console, built with `-DLIFT_BENCH` instead.
- `include/config.h`: this column's numbers.
