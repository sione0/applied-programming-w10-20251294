"""Provided bounded Genesis/Panda GPU execution of a planned joint path.

The input rows are (q1,q2), not Week 9 catalog node IDs. The other five
joint angles remain fixed. After one initialization, position targets and
physical scene.step() calls move the arm. Observed contacts and tracking
do not prove continuous collision-free motion or real-robot safety.
"""

import csv
import json
from pathlib import Path
import time

from .robot_constraints import (FORBIDDEN_RECTANGLE, full_joint_targets,
                                slice_state_is_allowed, validate_robot_path)

DT_S = 0.01
INITIAL_SETTLE_STEPS = 60
RAMP_STEPS = 80
HOLD_STEPS = 80
ENDPOINT_TOLERANCE_RAD = 0.025


def run_scene(path_xy, output_dir=None, capture=False):
    """Execute one validated (N,2) joint path and optionally save real frames."""
    points = validate_robot_path(path_xy)
    targets = full_joint_targets(points)
    if capture and output_dir is None:
        raise ValueError('capture requires output_dir')
    import numpy as np
    import torch
    import genesis as gs

    torch.set_num_threads(2)
    if not torch.cuda.is_available():
        raise RuntimeError('Use the existing applied-programming-cuda environment')
    out = None if output_dir is None else Path(output_dir)
    if out is not None:
        out.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    step_limit = INITIAL_SETTLE_STEPS + (RAMP_STEPS + HOLD_STEPS) * (len(points) - 1)
    steps, trace, frames, waypoint_results = 0, [], [], []
    max_joint_violation = 0.0
    min_hand_z = float('inf')
    max_self_contacts = max_floor_contacts = 0
    max_joint_speed = max_ramp_tracking_error = 0.0

    def emit(event, **values):
        print(json.dumps({'event': event, 'step': steps, **values}, allow_nan=False), flush=True)

    def array(value):
        if hasattr(value, 'detach'):
            value = value.detach().cpu().numpy()
        value = np.asarray(value)
        if not np.isfinite(value).all():
            raise RuntimeError('A simulator observation is nonfinite')
        return value

    emit('initializing', waypoint_count=len(points), path_q_rad=points.tolist())
    gs.init(backend=gs.cuda, precision='32', logging_level='warning', seed=0)
    try:
        scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=DT_S, gravity=(0, 0, -9.81)),
            rigid_options=gs.options.RigidOptions(enable_collision=True,
                enable_joint_limit=True, box_box_detection=True),
            vis_options=gs.options.VisOptions(background_color=(0.96, 0.97, 0.98),
                ambient_light=(0.30, 0.30, 0.30), shadow=True, plane_reflection=False,
                lights=[gs.options.vis.DirectionalLight(dir=(-1.0, -0.6, -4.0),
                    color=(1.0, 0.99, 0.98), intensity=2.8)]),
            renderer=gs.renderers.Rasterizer(), show_viewer=False, show_FPS=False)
        floor = scene.add_entity(gs.morphs.Plane(),
            surface=gs.surfaces.Default(color=(0.82, 0.84, 0.87), roughness=0.85))
        arm = scene.add_entity(gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
            surface=gs.surfaces.Default(roughness=0.4, metallic=0.05))
        camera = None
        if capture:
            camera = scene.add_camera(res=(1200, 900), pos=(0.35, -2.00, 1.05),
                lookat=(0.20, 0.0, 0.45), fov=36, GUI=False)
        scene.build()
        motors, fingers = np.arange(7), np.arange(7, 9)
        arm.set_dofs_kp(np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 500, 500]))
        arm.set_dofs_kv(np.array([450, 450, 350, 350, 200, 200, 200, 50, 50]))
        force = np.array([87, 87, 87, 87, 12, 12, 12, 100, 100], dtype=float)
        arm.set_dofs_force_range(-force, force)
        lower, upper = (array(item) for item in arm.get_dofs_limit(motors))
        if np.any(targets < lower) or np.any(targets > upper):
            raise ValueError('Planned target exceeds Panda model joint limits')
        base_index = arm.get_link('link0').idx
        hand = arm.get_link('hand')
        arm.set_qpos(np.r_[targets[0], 0.04, 0.04])  # Initialization only.
        emit('scene_built')

        def step(target, segment, phase):
            nonlocal steps, max_joint_violation, min_hand_z, max_joint_speed
            nonlocal max_self_contacts, max_floor_contacts, max_ramp_tracking_error
            if steps >= step_limit or time.monotonic() - started > 240:
                raise RuntimeError('The bounded GPU execution budget was exceeded')
            arm.control_dofs_position(target, motors)
            arm.control_dofs_position(np.array([0.04, 0.04]), fingers)
            scene.step()
            steps += 1
            q = array(arm.get_dofs_position(motors))
            velocity = array(arm.get_dofs_velocity(motors))
            hand_pos = array(hand.get_pos()).reshape(3)
            error = float(np.max(np.abs(q - target)))
            violation = float(max(0.0, np.max(lower - q), np.max(q - upper)))
            max_joint_violation = max(max_joint_violation, violation)
            min_hand_z = min(min_hand_z, float(hand_pos[2]))
            max_joint_speed = max(max_joint_speed, float(np.max(np.abs(velocity))))
            if phase == 'ramp':
                max_ramp_tracking_error = max(max_ramp_tracking_error, error)
            self_contacts = arm.get_contacts(with_entity=arm)
            n_self = len(self_contacts['geom_a'])
            floor_contacts = arm.get_contacts(with_entity=floor)
            link_a = array(floor_contacts['link_a']).astype(int)
            link_b = array(floor_contacts['link_b']).astype(int)
            n_floor = int(np.count_nonzero((link_a != base_index) & (link_b != base_index)))
            max_self_contacts, max_floor_contacts = max(max_self_contacts, n_self), max(max_floor_contacts, n_floor)
            if steps % 4 == 0:
                trace.append({'step': steps, 'time_s': steps * DT_S, 'segment': segment,
                    'phase': phase, 'hand_x_m': float(hand_pos[0]),
                    'hand_y_m': float(hand_pos[1]), 'hand_z_m': float(hand_pos[2]),
                    'max_joint_error_rad': error, 'observed_self_contacts': n_self,
                    'observed_nonbase_floor_contacts': n_floor,
                    **{f'q{i + 1}_rad': float(value) for i, value in enumerate(q)}})
            if violation > 1e-4 or n_self or n_floor:
                raise RuntimeError('Observed joint-limit or solver-contact condition failed')
            if not slice_state_is_allowed(q[:2]):
                raise RuntimeError('Measured joint state violated the declared slice rule')

        def reached(index):
            target = targets[index]
            q = array(arm.get_dofs_position(motors))
            hand_pos = array(hand.get_pos()).reshape(3)
            error = float(np.max(np.abs(q - target)))
            record = {'waypoint': index, 'step': steps, 'joint_target_rad': target.tolist(),
                'measured_joint_rad': q.tolist(), 'max_joint_error_rad': error,
                'hand_position_m': hand_pos.tolist()}
            waypoint_results.append(record)
            emit('waypoint_reached', **record)
            if error > ENDPOINT_TOLERANCE_RAD:
                raise RuntimeError('Panda did not settle within the endpoint tolerance')
            if camera is not None:
                filename = out / f'{index:02d}_waypoint.png'
                rgb, _, _, _ = camera.render(rgb=True)
                gs.tools.save_img_arr(rgb, filename=str(filename))
                frames.append({'waypoint': index, 'step': steps, 'file': filename.name})
                emit('frame_saved', file=str(filename))

        for _ in range(INITIAL_SETTLE_STEPS):
            step(targets[0], 'initial', 'initial_settle')
        reached(0)
        for index in range(1, len(targets)):
            left, right = targets[index - 1], targets[index]
            segment = f'{index - 1}->{index}'
            emit('segment_start', segment=segment)
            for frame in range(1, RAMP_STEPS + 1):
                fraction = frame / RAMP_STEPS
                smooth = fraction * fraction * (3.0 - 2.0 * fraction)
                step(left + smooth * (right - left), segment, 'ramp')
            for _ in range(HOLD_STEPS):
                step(right, segment, 'hold')
            reached(index)
        checks = {'finite_observations': True, 'joint_limits': max_joint_violation <= 1e-4,
            'declared_joint_region': True,
            'waypoint_tracking': all(n['max_joint_error_rad'] <= ENDPOINT_TOLERANCE_RAD for n in waypoint_results),
            'observed_self_contacts': max_self_contacts == 0,
            'observed_nonbase_floor_contacts': max_floor_contacts == 0}
        result = {'status': 'EXECUTED' if all(checks.values()) else 'FAILED',
            'executed': True, 'path_q_rad': points.tolist(), 'edge_count': len(points) - 1,
            'steps': steps, 'dt_s': DT_S, 'simulation_time_s': steps * DT_S,
            'endpoint_tolerance_rad': ENDPOINT_TOLERANCE_RAD,
            'forbidden_rectangle_rad': FORBIDDEN_RECTANGLE,
            'max_waypoint_error_rad': max(n['max_joint_error_rad'] for n in waypoint_results),
            'max_ramp_tracking_error_rad': max_ramp_tracking_error,
            'max_joint_speed_rad_s': max_joint_speed,
            'max_joint_limit_violation_rad': max_joint_violation,
            'minimum_hand_height_m': min_hand_z,
            'max_observed_self_contacts': max_self_contacts,
            'max_observed_nonbase_floor_contacts': max_floor_contacts,
            'checks': checks, 'waypoints': waypoint_results, 'frames': frames,
            'scope': 'Planned joint-state path, observed physical tracking and solver contacts. '
                     'No carried object, continuous collision proof or real-robot safety claim.'}
        if out is not None:
            with (out / 'trace.csv').open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(trace[0]))
                writer.writeheader()
                writer.writerows(trace)
            (out / 'result.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        emit('execution_complete', status=result['status'], steps=steps,
             max_waypoint_error_rad=result['max_waypoint_error_rad'])
        return result
    finally:
        gs.destroy()

