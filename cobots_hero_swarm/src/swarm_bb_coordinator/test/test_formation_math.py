import math
import pytest
from swarm_bb_coordinator.leader_follower_coordinator import (
    compute_formation_target,
    compute_apf_repulsion,
    unicycle_controller,
    normalize_angle,
)
from swarm_bb_coordinator.zenoh_swarm_bridge import quaternion_to_yaw


def test_quaternion_to_yaw():
    # Yaw = 0: q = (x=0, y=0, z=0, w=1)
    assert math.isclose(quaternion_to_yaw(0.0, 0.0, 0.0, 1.0), 0.0, abs_tol=1e-5)

    # Yaw = pi/2: q = (x=0, y=0, z=sin(pi/4), w=cos(pi/4))
    s_half = math.sin(math.pi / 4.0)
    c_half = math.cos(math.pi / 4.0)
    assert math.isclose(quaternion_to_yaw(0.0, 0.0, s_half, c_half), math.pi / 2.0, abs_tol=1e-5)

    # Yaw = -pi/2
    assert math.isclose(quaternion_to_yaw(0.0, 0.0, -s_half, c_half), -math.pi / 2.0, abs_tol=1e-5)


def test_compute_formation_target_rotation():
    leader_pos = (1.0, 2.0)
    dx, dy = -0.35, 0.35

    # 1. Yaw = 0: Target = (1.0 - 0.35, 2.0 + 0.35) = (0.65, 2.35)
    tx, ty = compute_formation_target(leader_pos[0], leader_pos[1], 0.0, dx, dy)
    assert math.isclose(tx, 0.65, abs_tol=1e-5)
    assert math.isclose(ty, 2.35, abs_tol=1e-5)

    # 2. Yaw = pi/2: R(pi/2) * (-0.35, 0.35) = (-0.35, -0.35)
    # Target = (1.0 - 0.35, 2.0 - 0.35) = (0.65, 1.65)
    tx, ty = compute_formation_target(leader_pos[0], leader_pos[1], math.pi / 2.0, dx, dy)
    assert math.isclose(tx, 0.65, abs_tol=1e-5)
    assert math.isclose(ty, 1.65, abs_tol=1e-5)

    # 3. Yaw = pi: R(pi) * (-0.35, 0.35) = (0.35, -0.35)
    # Target = (1.0 + 0.35, 2.0 - 0.35) = (1.35, 1.65)
    tx, ty = compute_formation_target(leader_pos[0], leader_pos[1], math.pi, dx, dy)
    assert math.isclose(tx, 1.35, abs_tol=1e-5)
    assert math.isclose(ty, 1.65, abs_tol=1e-5)


def test_apf_repulsion():
    pos_i = (0.0, 0.0)
    d_safe = 0.30

    # Obstacle beyond d_safe -> 0 repulsion
    other_far = [(0.5, 0.0)]
    vx_rep, vy_rep = compute_apf_repulsion(pos_i, other_far, d_safe=d_safe)
    assert vx_rep == 0.0
    assert vy_rep == 0.0

    # Obstacle at (0.1, 0.0) -> Repulsion points in -x direction (away from obstacle)
    other_close = [(0.1, 0.0)]
    vx_rep, vy_rep = compute_apf_repulsion(pos_i, other_close, d_safe=d_safe, k_rep=0.1)
    assert vx_rep < 0.0
    assert math.isclose(vy_rep, 0.0, abs_tol=1e-5)

    # Obstacle extremely close -> Repulsion is clamped to max_rep_mag
    other_very_close = [(0.001, 0.0)]
    vx_rep, vy_rep = compute_apf_repulsion(pos_i, other_very_close, d_safe=d_safe, max_rep_mag=0.5)
    assert math.isclose(math.hypot(vx_rep, vy_rep), 0.5, abs_tol=1e-4)


def test_unicycle_controller():
    # 1. Target reached within tolerance -> v = 0, align heading with leader
    v, w = unicycle_controller(
        x_curr=0.0, y_curr=0.0, theta_curr=0.0,
        vx_des=0.0, vy_des=0.0,
        leader_theta=0.5, dist_to_target=0.02, pos_tolerance=0.03
    )
    assert v == 0.0
    assert w > 0.0  # Aligning towards positive leader_theta

    # 2. Large velocity command -> v clipped to v_max
    v_max = 0.22
    w_max = 1.5
    v, w = unicycle_controller(
        x_curr=0.0, y_curr=0.0, theta_curr=0.0,
        vx_des=2.0, vy_des=0.0,  # Directly ahead
        leader_theta=0.0, dist_to_target=1.0,
        v_max=v_max, w_max=w_max
    )
    assert math.isclose(v, v_max, abs_tol=1e-4)
    assert math.isclose(w, 0.0, abs_tol=1e-4)

    # 3. Target is 90 degrees to the left -> w is positive, v is modulated
    v, w = unicycle_controller(
        x_curr=0.0, y_curr=0.0, theta_curr=0.0,
        vx_des=0.0, vy_des=1.0,
        leader_theta=0.0, dist_to_target=1.0,
        v_max=v_max, w_max=w_max
    )
    assert w > 0.0
    assert math.isclose(v, 0.0, abs_tol=1e-4)  # cos(pi/2) ~ 0
