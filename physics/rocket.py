from dataclasses import dataclass
from typing import Dict

from physics.forces import clamp, gravity_force, thrust_force, aerodynamic_drag, gimbal_torque, fin_torque

DEFAULT_ISP = 300.0
G0 = 9.80665

@dataclass
class RocketState:
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    theta: float = 0.0
    omega: float = 0.0
    mass: float = 25000.0

@dataclass
class ControlInput:
    thrust: float = 0.0
    gimbal: float = 0.0
    fin: float = 0.0

@dataclass
class RocketConfig:
    dry_mass: float = 8000.0
    fuel_mass: float = 16000.0
    max_thrust: float = 845000.0
    max_gimbal_angle: float = 10.0
    max_fin_angle: float = 20.0
    moment_of_inertia: float = 2.5e6
    drag_coefficient: float = 0.35
    reference_area: float = 15.0
    air_density: float = 1.225
    gimbal_arm: float = 5.0
    fin_torque_gain: float = 5e4
    fin_damping: float = 1e5
    isp: float = DEFAULT_ISP

class Rocket:
    def __init__(self, config: RocketConfig):
        self.config = config

    def _effective_mass(self, mass: float) -> float:
        return max(mass, self.config.dry_mass)

    def derivative(self, state: RocketState, control: ControlInput, environment: Dict[str, float]) -> RocketState:
        mass = self._effective_mass(state.mass)
        thrust = clamp(control.thrust, 0.0, self.config.max_thrust)
        gimbal = clamp(control.gimbal, -self.config.max_gimbal_angle, self.config.max_gimbal_angle)
        fin = clamp(control.fin, -self.config.max_fin_angle, self.config.max_fin_angle)

        if mass <= self.config.dry_mass and thrust > 0.0:
            thrust = 0.0

        wind = environment.get("wind", 0.0)
        relative_vx = state.vx - wind
        drag_x, drag_y = aerodynamic_drag(
            relative_vx,
            state.vy,
            self.config.air_density,
            self.config.drag_coefficient,
            self.config.reference_area,
        )

        thrust_x, thrust_y = thrust_force(thrust, state.theta, gimbal)
        gravity_x, gravity_y = gravity_force(mass)
        torque = gimbal_torque(thrust, gimbal, self.config.gimbal_arm) + fin_torque(fin, relative_vx, self.config.fin_torque_gain)

        ax = (thrust_x + drag_x + gravity_x) / mass
        ay = (thrust_y + drag_y + gravity_y) / mass

        mass_flow = 0.0
        if thrust > 0.0:
            mass_flow = -thrust / (self.config.isp * G0)

        return RocketState(
            x=state.vx,
            y=state.vy,
            vx=ax,
            vy=ay,
            theta=state.omega,
            omega=torque / self.config.moment_of_inertia,
            mass=mass_flow,
        )

    def initial_state(self, x=0.0, y=500.0, vx=0.0, vy=-20.0, theta=0.0, omega=0.0) -> RocketState:
        return RocketState(
            x=x,
            y=y,
            vx=vx,
            vy=vy,
            theta=theta,
            omega=omega,
            mass=self.config.dry_mass + self.config.fuel_mass,
        )
