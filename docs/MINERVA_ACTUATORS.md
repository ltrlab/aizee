# Minerva Actuator Selection & Sourcing

Actuator plan for the **Minerva** humanoid torso (a sibling build to AIZEE). Minerva is
an 18-DoF platform: a 2-DoF waist (swivel + forward/back lean), a 2-DoF neck, and two
6-DoF arms each with a gripper. Drive is dual CAN bus chains, MIT-mode control, 48 V.

Last priced: 2026-07-27. Prices are MSRP with integrated driver board and drift; re-check before ordering.

## Joint layout (18 actuators)

| Subsystem | Joints | Count |
|-----------|--------|-------|
| Waist | swivel (yaw), lean (pitch) | 2 |
| Neck | 2-DoF | 2 |
| Arms (x2) | shoulder pitch, shoulder roll, elbow, wrist flex, wrist pitch, wrist roll, gripper | 14 |

Check: (6 DoF + gripper) x 2 arms = 14, waist 2, neck 2 = **18**.

## Recommended Cubemars BOM

| Model | Joint roles | Peak / rated Nm | Ratio | Wt ea | Price ea | Qty | Link |
|-------|-------------|-----------------|-------|-------|----------|-----|------|
| AK80-64 | torso lean + 2 shoulder pitch | 120 / 48 | 64:1 | 850 g | ~$989.90 | 3 | https://store.cubemars.com/products/ak80-64 |
| AK60-39 V3.0 | 2 shoulder roll | 72 / 24 | 39:1 | 750 g | $448.90 | 2 | https://store.cubemars.com/products/ak60-39-v3-0 |
| AK10-9 V3.0 | torso swivel + 2 elbows | 53 / 18 | 9:1 | 896 g | $798.90 | 3 | https://store.cubemars.com/products/ak10-9-v3-0-kv60 |
| AK70-10 | 4x wrist flex/pitch | 24.8 / 8.3 | 10:1 | 521 g | $398.90 | 4 | https://store.cubemars.com/products/ak70-10 |
| AK40-10 KV170 | 2 wrist roll + 2 grippers + 2 neck | 4.1 / 1.3 | 10:1 | 185 g | $99.90 | 6 | https://store.cubemars.com/products/ak40-10-kv170 |

**Totals: 18 units, ~$8,459, ~9.93 kg** (actuators only).

Notes:
- All AK models currently show sold out on the official store; resellers (DigiKey, RobotShop, Oz Robotics) carry stock at similar or higher prices.
- AK80-64 listing showed a $889.90 / $989.90 sale ambiguity, so budget the shoulder/lean trio at $2,670 to $2,970.
- The 6x AK40-10 (185 g, $99.90) are the value pick; pushing neck, wrist roll, and grippers onto them wins on both weight and cost.

### Shoulder-roll note
AK60-39 (72 Nm) was chosen over a second AK80-64 for the shoulder-roll (abduction)
joint: still high-torque, but 100 g lighter and ~$540 cheaper each. Swap to AK80-64 if
maximum payload margin is wanted (adds ~200 g total for the pair, +48 Nm peak each).

## Comparison vs AIZEE Robstrides

Per-joint equivalence (Cubemars pick vs the Robstride playing the same role on AIZEE):

| Role | Cubemars | Peak/rated Nm | Ratio | Wt | Robstride | Peak/rated Nm | Ratio | Wt |
|------|----------|---------------|-------|----|-----------|---------------|-------|----|
| Shoulder pitch / lean | AK80-64 | 120 / 48 | 64:1 | 850 g | RS04 | 120 / 40 | 9:1 | 1420 g |
| Shoulder roll | AK60-39 | 72 / 24 | 39:1 | 750 g | (RS03 nearest) | 60 / 21 | 9:1 | 900 g |
| Torso swivel / elbow | AK10-9 | 53 / 18 | 9:1 | 896 g | RS03 | 60 / 21 | 9:1 | 900 g |
| Wrist flex / pitch | AK70-10 | 24.8 / 8.3 | 10:1 | 521 g | RS02 | 17 / 7 | 7.75:1 | 380 g |
| Wrist roll / gripper / neck | AK40-10 | 4.1 / 1.3 | 10:1 | 185 g | RS00 | 14 / 5 | 10:1 | 310 g |

Full-build totals, same 18-joint layout mapped to each vendor's best fit:

| | Cubemars | Robstride |
|--|----------|-----------|
| Torso lean + shoulder pitch | 3x AK80-64 | 3x RS04 |
| Shoulder roll | 2x AK60-39 | 2x RS03 |
| Torso swivel + elbows | 3x AK10-9 | 3x RS03 |
| Wrist flex/pitch | 4x AK70-10 | 4x RS02 |
| Wrist roll + grippers + neck | 6x AK40-10 | 6x RS00 |
| Actuator cost | ~$8,459 | ~$2,100 |
| Actuator weight | ~9.93 kg | ~12.1 kg |

Robstride prices from China retail (RS04 CNY 1199, RS03 CNY 999, RS02 CNY 699, RS00 CNY 598
at ~7.15 CNY/USD); US resellers run higher.

### Takeaways
- Robstride is ~4x cheaper but ~2.2 kg heavier, almost all from the three 1.42 kg RS04s.
- Cubemars buys weight savings and US distribution/support, not raw performance. On spec
  the Robstrides match or beat the Cubemars picks at every joint except distal weight.
- The RS04 shoulder is the clear performance winner: same 120 Nm peak as the AK80-64 but
  at 9:1 (fast, back-drivable) vs 64:1 (slow, stiff). Cubemars has no 9:1 / 120 Nm unit.
- The Cubemars build only pencils out as a fully-sponsored kit (credited YouTube build):
  lighter arms + vendor support at no hardware cost.

## Sponsorship status
Approached CubeMars (Heidi, Global Marketing & Sales, lmh@cubemars.com) for a full
18-unit hardware sponsorship in exchange for a sponsored YouTube build video and CubeMars
credit on all Minerva posts. Awaiting their response on the requested set.
