from typing import Callable, Dict, List, Optional

from physics.integrator import rk4_step
from physics.rocket import ControlInput, Rocket, RocketState


class SimulationResult:
    def __init__(self, history: List[RocketState]):
        self.history = history


class Simulator:
    def __init__(self, rocket: Rocket, time_step: float = 0.005):
        self.rocket = rocket
        self.time_step = time_step

    def step(self, state: RocketState, control: ControlInput, environment: Dict[str, float]) -> RocketState:
        return rk4_step(state, lambda s: self.rocket.derivative(s, control, environment), self.time_step)

    def run(
        self,
        controller,
        initial_state: RocketState,
        duration: float,
        environment: Optional[Dict[str, float]] = None,
    ) -> SimulationResult:
        environment = environment or {}
        state = initial_state
        history: List[RocketState] = [state]
        steps = int(round(duration / self.time_step))

        controller.reset()
        for _ in range(steps):
            control_input = controller.update(state, self.time_step, environment)
            state = self.step(state, control_input, environment)
            history.append(state)
        return SimulationResult(history)
