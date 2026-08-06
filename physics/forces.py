def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def gravity_force(mass: float, g: float = 9.81) -> tuple[float, float]:
    return 0.0, -mass * g


def thrust_force(thrust: float, theta: float, gimbal: float) -> tuple[float, float]:
    angle = theta + gimbal
    return thrust * __sin(angle), thrust * __cos(angle)


def aerodynamic_drag(vx: float, vy: float, rho: float, drag_coefficient: float, area: float) -> tuple[float, float]:
    drag_x = -0.5 * rho * drag_coefficient * area * vx * abs(vx)
    drag_y = -0.5 * rho * drag_coefficient * area * vy * abs(vy)
    return drag_x, drag_y


def gimbal_torque(thrust: float, gimbal: float, arm: float) -> float:
    return thrust * arm * __sin(gimbal)


def fin_torque(fin_angle: float, relative_vx: float, torque_gain: float) -> float:
    return -torque_gain * fin_angle * relative_vx


def __sin(value: float) -> float:
    from math import sin

    return sin(value)


def __cos(value: float) -> float:
    from math import cos

    return cos(value)
