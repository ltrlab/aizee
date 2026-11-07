"""Synthetic controller parameters for software tests; never hardware settings."""

from robot_platform.control.collision_envelope import calibration_revision


def synthetic_config(model, backend):
    names = tuple(model.independent)
    mapping = {name: dict(actuator=name, scale=1.0, offset=0.0) for name in names}
    limits, routes = [], []
    for i, name in enumerate(names):
        item = model.by_name[name]
        sliding = item['type'] == 'prismatic'
        limits.append(
            dict(
                name=name,
                unit='metres' if sliding else 'radians',
                lower=item['control_bounds'][0],
                upper=item['control_bounds'][1],
                max_velocity=0.0005 if sliding else 0.02,
                max_effort=5.0,
                max_kp=20.0,
                max_kd=5.0,
                stop_acceleration=0.01 if sliding else 0.2,
                hold_kp=10.0,
                hold_kd=1.0,
                max_tracking_error=0.0001 if sliding else 0.002,
                max_temperature=70.0,
            )
        )
        routes.append(dict(actuator=name, leader='synthetic', channel=i, scale=0.01 if sliding else 1.0))
    cfg = dict(
        group='synthetic_whole_robot',
        identity=dict(
            model=backend.runtime_model.revision,
            calibration=calibration_revision(mapping),
            collision_policy=backend.policy_revision,
        ),
        lease_ms=150,
        feedback_timeout_ms=30,
        settled_velocity=0.001,
        joints=limits,
        collision=dict(
            world_revision=backend.runtime_model.world_revision, scope='inter_branch', max_verdict_ms=80, reaction_ms=10
        ),
    )
    return cfg, mapping, routes
