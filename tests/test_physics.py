import math

from physics.forces import gravity_force, thrust_force, aerodynamic_drag, gimbal_torque
from physics.integrator import rk4_step
from physics.rocket import ControlInput, Rocket, RocketConfig, RocketState


def test_gravity_force():
    fx, fy = gravity_force(1000.0)
    assert fx == 0.0
    assert math.isclose(fy, -9810.0, rel_tol=1e-9)


def test_thrust_force_points_upward_for_vertical_rocket():
    fx, fy = thrust_force(10000.0, 0.0, 0.0)
    assert math.isclose(fx, 0.0, abs_tol=1e-9)
    assert fy > 0.0


def test_aerodynamic_drag_opposes_velocity():
    drag_x, drag_y = aerodynamic_drag(10.0, -5.0, 1.225, 0.35, 15.0)
    assert drag_x < 0.0
    assert drag_y > 0.0


def test_gimbal_torque_changes_sign_with_direction():
    torque_pos = gimbal_torque(100000.0, 0.05, 5.0)
    torque_neg = gimbal_torque(100000.0, -0.05, 5.0)
    assert torque_pos == -torque_neg


def test_rk4_step_consistent_with_small_time_step():
    config = RocketConfig(dry_mass=1000.0, fuel_mass=100.0, max_thrust=20000.0)
    rocket = Rocket(config)
    state = rocket.initial_state(x=0.0, y=100.0, vx=0.0, vy=0.0)
    control = ControlInput(thrust=11000.0, gimbal=0.0, fin=0.0)
    environment = {"wind": 0.0}

    def derivative(s):
        return rocket.derivative(s, control, environment)

    next_state = rk4_step(state, derivative, 0.01)
    assert next_state.y > state.y
    assert next_state.mass < state.mass
