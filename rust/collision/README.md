# robot_collision

Robot-independent persistent Parry f64 shape queries behind a small C ABI.
This is one member of the shared Rust workspace. It performs no IO and has no
ROS, ZMQ or hardware dependency.

Build on the current Windows development host with
`./scripts/build_collision.ps1 -Test`; cross-build for JetPack 6 with
`./scripts/build_jetson.ps1 -Offline -Package robot_collision`.

ABI v3 exposes `rpc_create`, `rpc_query`, `rpc_destroy` and `rpc_abi_version`.
Creation consumes JSON shapes (convex hull vertices, boxes, spheres, cylinders).
Queries consume xyz/xyzw transforms and return distances in stable cross-link
pair order after filtering. Creation accepts explicit `ignored_links`; queries
select all, inter-branch, within-branch, body or environment scope. Branch/link
AABBs prune blocks before shape queries. Infinity means outside the requested
range. Instances on the same link and pairs of environment obstacles are omitted. URDF Z-axis cylinders are
mapped to Parry's cylinder convention. Invalid inputs return failure.

**Repository-generated convex shapes enclose CAD; they are not CAD surfaces.**
In CAD mode, Python `NativeCollisionBackend` confirms candidates using cached
FCL source geometry. In explicit keep-out mode, enclosing distance itself
defines a conservative boundary, without refinement. Its separately versioned
profile policy omits Minerva's four finger links; CAD modes include them.
The viewer labels these as envelopes and boundary violations. Hardware
enforcement remains unavailable pending calibration/controller qualification.

The caller owns buffers, serializes queries/destruction and must honor the
documented unsafe pointer contracts. Shared library loading and model creation
happen once, not on each frame. See the
[implementation status](../../docs/architecture/COLLISION_RUNTIME_IMPLEMENTATION.md)
for validation, timing limits and the opt-in simulation command gate.
