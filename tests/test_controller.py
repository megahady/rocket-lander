from physics.rocket import RocketConfig, Rocket
from controllers.controller import HoverController


def test_hover_controller_returns_control_input():
    config = RocketConfig(dry_mass=1000.0, fuel_mass=100.0, max_thrust=20000.0)
    rocket = Rocket(config)
    state = rocket.initial_state(x=0.0, y=100.0, vx=0.0, vy=0.0)
    controller = HoverController(target_altitude=0.0, max_thrust=config.max_thrust)

    control = controller.update(state, 0.01, {"wind": 0.0})
    assert control.thrust >= 0.0
    assert abs(control.gimbal) < 1e-6
    assert abs(control.fin) < 1e-6
