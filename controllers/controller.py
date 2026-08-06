from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict

from physics.rocket import ControlInput, RocketState


class Controller(ABC):
    @abstractmethod
    def reset(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def update(self, state: RocketState, dt: float, environment: Dict[str, float]) -> ControlInput:
        raise NotImplementedError


@dataclass
class HoverController(Controller):
    target_altitude: float = 0.0
    max_thrust: float = 845000.0
    kp_altitude: float = 0.2
    kd_vertical: float = 1.0

    def reset(self) -> None:
        pass

    def update(self, state: RocketState, dt: float, environment: Dict[str, float]) -> ControlInput:
        altitude_error = self.target_altitude - state.y
        desired_acceleration = self.kp_altitude * altitude_error - self.kd_vertical * state.vy
        required_force = state.mass * (9.81 + desired_acceleration)
        thrust = max(0.0, min(self.max_thrust, required_force))
        return ControlInput(thrust=thrust, gimbal=0.0, fin=0.0)
