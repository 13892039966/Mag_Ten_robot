"""Four-tendon, controller-side constant-curvature geometry in SI units.

Reference: Rao et al. (2021), tdcr-modeling CC geometry. Rederived here for
four 90-degree tendons, +x backbone, 120 mm passive + 50 mm active segment.
The passive section is straight. This is not a load/dynamics/slack model.
"""
import math

import numpy as np

if __package__:
    from .model import ModelConfig, definition
else:
    from model import ModelConfig, definition


def _arc_terms(z):
    """sinc(sqrt(z)), cosc(sqrt(z)), and their derivatives w.r.t. z."""
    if z < 1e-4:
        s = 1. + z * (-1. / 6. + z * (1. / 120. + z * (-1. / 5040. + z / 362880.)))
        c = .5 + z * (-1. / 24. + z * (1. / 720. + z * (-1. / 40320. + z / 3628800.)))
        ds = -1. / 6. + z * (1. / 60. + z * (-1. / 1680. + z / 90720.))
        dc = -1. / 24. + z * (1. / 360. + z * (-1. / 13440. + z / 907200.))
    else:
        t = math.sqrt(z)
        sn, cs = math.sin(t), math.cos(t)
        s = sn / t
        c = (1. - cs) / z
        ds = (t * cs - sn) / (2. * t * z)
        dc = (.5 * t * sn - (1. - cs)) / (z * z)
    return s, c, ds, dc


class CCFourTendon:
    """Map [four target shortenings, feed] (m) to tip position (m).

    Positive shortening on cable 0 bends toward +y, cable 1 toward +z.
    Mean shortening changes active centreline length. Incompatible four
    lengths are least-squares projected; configuration() exposes residuals.
    Commands are not clipped here: physical actuator limits belong to the
    existing actuator layer, not to this differentiable geometric map.
    """
    def __init__(self, config=None):
        self.config = config or ModelConfig()
        definition(self.config)
        r = self.config.cable_radius_m
        self._chain = np.array([[-.25, -.25, -.25, -.25],
                                [1. / (2. * r), 0., -1. / (2. * r), 0.],
                                [0., 1. / (2. * r), 0., -1. / (2. * r)]])

    def _parameters(self, shortening, feed):
        q = np.asarray(shortening, dtype=float)
        if q.shape != (4,) or not np.isfinite(q).all() or not math.isfinite(float(feed)):
            raise ValueError('Expected four finite shortenings and finite feed, in metres')
        length = self.config.active_length_m - float(np.mean(q))
        if length <= 0.:
            raise ValueError('Active centreline length must remain positive')
        r = self.config.cable_radius_m
        b = float(q[0] - q[2]) / (2. * r)
        c = float(q[1] - q[3]) / (2. * r)
        return q, length, b, c

    def configuration(self, shortening):
        q, length, b, c = self._parameters(shortening, 0.)
        theta = math.hypot(b, c)
        mean = self.config.active_length_m - length
        fitted = mean + self.config.cable_radius_m * np.array([b, c, -b, -c])
        return dict(active_arc_length_m=length, bend_angle_rad=theta,
                    bend_direction_rad=math.atan2(c, b) if theta else 0.,
                    curvature_per_m=theta / length, fit_residual_m=q - fitted)

    def tip_position(self, shortening, feed=0.):
        _, length, b, c = self._parameters(shortening, feed)
        sinc, cosc, _, _ = _arc_terms(b * b + c * c)
        return np.array([self.config.passive_length_m + float(feed) + length * sinc,
                         length * cosc * b, length * cosc * c])

    def jacobian(self, shortening, feed=0.):
        """Analytic 3x5 dp/d[shortening0..3,feed], in m/m."""
        _, length, b, c = self._parameters(shortening, feed)
        sinc, cosc, ds, dc = _arc_terms(b * b + c * c)
        partial = np.array([[sinc, 2. * length * b * ds, 2. * length * c * ds],
                            [cosc * b, length * (cosc + 2. * b * b * dc), 2. * length * b * c * dc],
                            [cosc * c, 2. * length * b * c * dc, length * (cosc + 2. * c * c * dc)]])
        result = np.empty((3, 5))
        result[:, :4] = partial @ self._chain
        result[:, 4] = [1., 0., 0.]
        return result

    def jacobian_central(self, shortening, feed=0., step_m=1e-7):
        """Independent central-difference derivative: 10 position calls."""
        self._parameters(shortening, feed)
        if not math.isfinite(step_m) or step_m <= 0.:
            raise ValueError('Difference step must be finite and positive')
        value = np.r_[np.asarray(shortening, dtype=float), float(feed)]
        result = np.empty((3, 5))
        for i in range(5):
            plus, minus = value.copy(), value.copy()
            plus[i] += step_m
            minus[i] -= step_m
            result[:, i] = (self.tip_position(plus[:4], plus[4]) -
                            self.tip_position(minus[:4], minus[4])) / (2. * step_m)
        return result

    def backbone(self, shortening, feed=0., passive_points=25, active_points=21):
        """Nominal CC centreline, useful for inspection; not SOFA body state."""
        _, length, b, c = self._parameters(shortening, feed)
        for count in [passive_points, active_points]:
            if not isinstance(count, (int, np.integer)) or count < 2:
                raise ValueError('At least two points required per segment')
        passive = np.zeros((passive_points, 3))
        passive[:, 0] = float(feed) + np.linspace(0., self.config.passive_length_m, passive_points)
        active = []
        for u in np.linspace(0., 1., active_points)[1:]:
            sinc, cosc, _, _ = _arc_terms((b * b + c * c) * u * u)
            active.append([self.config.passive_length_m + float(feed) + length * u * sinc,
                           length * u * u * cosc * b, length * u * u * cosc * c])
        return np.vstack([passive, active])
