from physics.rocket import RocketState


def rk4_step(state: RocketState, derivative_fn, dt: float) -> RocketState:
    k1 = derivative_fn(state)
    k2 = derivative_fn(_add_state(state, k1, dt * 0.5))
    k3 = derivative_fn(_add_state(state, k2, dt * 0.5))
    k4 = derivative_fn(_add_state(state, k3, dt))
    return _add_state(state, _combine(k1, k2, k3, k4), dt / 6.0)


def euler_step(state: RocketState, derivative_fn, dt: float) -> RocketState:
    return _add_state(state, derivative_fn(state), dt)


def _add_state(state: RocketState, derivative: RocketState, scale: float) -> RocketState:
    return RocketState(
        x=state.x + derivative.x * scale,
        y=state.y + derivative.y * scale,
        vx=state.vx + derivative.vx * scale,
        vy=state.vy + derivative.vy * scale,
        theta=state.theta + derivative.theta * scale,
        omega=state.omega + derivative.omega * scale,
        mass=max(state.mass + derivative.mass * scale, 0.0),
    )


def _combine(k1: RocketState, k2: RocketState, k3: RocketState, k4: RocketState) -> RocketState:
    return RocketState(
        x=k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x,
        y=k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y,
        vx=k1.vx + 2.0 * k2.vx + 2.0 * k3.vx + k4.vx,
        vy=k1.vy + 2.0 * k2.vy + 2.0 * k3.vy + k4.vy,
        theta=k1.theta + 2.0 * k2.theta + 2.0 * k3.theta + k4.theta,
        omega=k1.omega + 2.0 * k2.omega + 2.0 * k3.omega + k4.omega,
        mass=k1.mass + 2.0 * k2.mass + 2.0 * k3.mass + k4.mass,
    )
