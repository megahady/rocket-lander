from physics.rocket import Rocket, RocketConfig
from physics.simulator import Simulator
from controllers.controller import HoverController


def main() -> None:
    rocket_config = RocketConfig(
        dry_mass=8000.0,
        fuel_mass=16000.0,
        max_thrust=845000.0,
        max_gimbal_angle=10.0,
        max_fin_angle=20.0,
    )
    rocket = Rocket(rocket_config)
    simulator = Simulator(rocket, time_step=0.01)
    initial_state = rocket.initial_state(x=0.0, y=500.0, vx=0.0, vy=-20.0)
    controller = HoverController(target_altitude=500.0, max_thrust=rocket_config.max_thrust)
    result = simulator.run(controller, initial_state, duration=2.0)

    final_state = result.history[-1]
    print("Demo completed")
    print(f"Final altitude: {final_state.y:.2f} m")
    print(f"Final vertical velocity: {final_state.vy:.2f} m/s")
    print(f"Final mass: {final_state.mass:.2f} kg")


if __name__ == "__main__":
    main()
