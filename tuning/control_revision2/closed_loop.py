"""Position-feedback bounded DLS using the four-tendon CC Jacobian."""
from dataclasses import dataclass
import math
import time

import numpy as np
from scipy.optimize import lsq_linear

if __package__:
    from .cc_predictor import CCFourTendon
else:
    from cc_predictor import CCFourTendon


@dataclass(frozen=True)
class DLSConfig:
    control_period_s: float = .020
    position_gain_per_s: float = 4.
    velocity_damping: float = .2
    damping_mm_s: float = .05
    smoothing_mm_s: float = .08


class BoundedDLS:
    def __init__(self, config=None, robot_config=None):
        self.config = config or DLSConfig()
        self.model = CCFourTendon(robot_config)
        vals = np.asarray(list(vars(self.config).values()))
        if not np.isfinite(vals).all() or np.any(vals < 0.) or self.config.control_period_s <= 0. or self.config.damping_mm_s <= 0.:
            raise ValueError('Invalid DLS configuration')
        c = self.model.config
        self.speeds = np.array([c.cable_speed_m_s] * 4 + [c.feed_speed_m_s])
        self.limits = np.array([c.cable_limit_m] * 4 + [c.feed_limit_m])

    def command(self, actual_shortening, feed, tip, tip_velocity, goal, previous_action):
        started = time.perf_counter_ns()
        q, tip, velocity, goal, prev = [np.asarray(x, dtype=float) for x in
                                       [actual_shortening, tip, tip_velocity, goal, previous_action]]
        if any(x.shape != shape for x, shape in [(q, (4,)), (tip, (3,)), (velocity, (3,)), (goal, (3,)), (prev, (5,))]):
            raise ValueError('Unexpected feedback or action shape')
        if not all(np.isfinite(x).all() for x in [q, tip, velocity, goal, prev]) or not np.isfinite(feed):
            raise ValueError('Nonfinite closed-loop feedback')
        cfg = self.config
        t0 = time.perf_counter_ns()
        j = self.model.jacobian(q, feed)
        jacobian_us = (time.perf_counter_ns() - t0) / 1000.
        # mm/s scaling keeps the damping terms meaningful and well scaled.
        a = 1000. * j * self.speeds[None, :]
        v_desired = cfg.position_gain_per_s * (goal - tip) - cfg.velocity_damping * velocity
        matrix = np.vstack([a, cfg.damping_mm_s * np.eye(5), cfg.smoothing_mm_s * np.eye(5)])
        rhs = np.r_[1000. * v_desired, np.zeros(5), cfg.smoothing_mm_s * np.clip(prev, -1., 1.)]
        # Re-anchor cable targets to measured lengths to avoid target windup.
        reference = np.clip(np.r_[q, feed], -self.limits, self.limits)
        scale = self.speeds * cfg.control_period_s
        lower = np.maximum(-1., (-self.limits - reference) / scale)
        upper = np.minimum(1., (self.limits - reference) / scale)
        solution = lsq_linear(matrix, rhs, bounds=(lower, upper), method='bvls', tol=1e-10, max_iter=50)
        if not solution.success or not np.isfinite(solution.x).all():
            raise RuntimeError('Bounded DLS failed: ' + str(solution.message))
        action = np.clip(solution.x, lower, upper)
        target = np.clip(reference + scale * action, -self.limits, self.limits)
        return dict(action=action, shortening_target_m=target[:4], feed_target_m=float(target[4]),
                    predicted_velocity_m_s=j @ (self.speeds * action),
                    solver_success=bool(solution.success), jacobian_us=jacobian_us,
                    controller_us=(time.perf_counter_ns() - started) / 1000.,
                    action_saturated=bool(np.any(np.abs(action) > .999)))


def slew_targets(submitted_shortening, submitted_feed, desired_shortening, desired_feed, robot_config):
    """Move actuator boundary targets by at most one PHYSICS-step speed limit."""
    current = np.r_[np.asarray(submitted_shortening, dtype=float), float(submitted_feed)]
    desired = np.r_[np.asarray(desired_shortening, dtype=float), float(desired_feed)]
    if current.shape != (5,) or desired.shape != (5,) or not np.isfinite(np.r_[current, desired]).all():
        raise ValueError('Expected finite submitted and desired actuator targets')
    speeds = np.array([robot_config.cable_speed_m_s] * 4 + [robot_config.feed_speed_m_s])
    limits = np.array([robot_config.cable_limit_m] * 4 + [robot_config.feed_limit_m])
    target = current + np.clip(np.clip(desired, -limits, limits) - current,
                               -speeds * robot_config.dt, speeds * robot_config.dt)
    target = np.clip(target, -limits, limits)
    return target[:4], float(target[4])


class DemoReference:
    """Small free-space targets, followed by one y-z circle; no velocity feedforward."""
    point_duration_s = 2.5
    circle_period_s = 8.
    circle_radius_m = .001
    duration_s = 23.5

    def __init__(self, origin=None):
        self.origin = np.array([.170, 0., 0.]) if origin is None else np.asarray(origin, dtype=float)
        if self.origin.shape != (3,) or not np.isfinite(self.origin).all():
            raise ValueError('Expected finite reference origin')
        self.points = [('point_y_positive', [0., .0015, 0.]),
                       ('point_y_negative', [0., -.0015, 0.]),
                       ('point_z_positive', [0., 0., .0015]),
                       ('point_z_negative', [0., 0., -.0015]),
                       ('point_feed', [.002, 0., 0.]),
                       ('circle_start', [0., .001, 0.])]

    def target(self, t):
        if not math.isfinite(t) or t < 0.:
            raise ValueError('Expected nonnegative finite time')
        if t < .5:
            return self.origin.copy(), 'rest'
        shifted = t - .5
        for i, (name, delta) in enumerate(self.points):
            if shifted < (i + 1) * self.point_duration_s:
                return self.origin + delta, name
        angle = 2. * np.pi * min(shifted - 6 * self.point_duration_s, self.circle_period_s) / self.circle_period_s
        return self.origin + [0., self.circle_radius_m * np.cos(angle), self.circle_radius_m * np.sin(angle)], 'circle'
