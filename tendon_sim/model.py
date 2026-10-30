"""SI geometry and actuator commands for a continuous 120+50 mm beam."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ModelConfig:
    passive_length_m: float = .120
    active_length_m: float = .050
    edge_length_m: float = .005
    radius_m: float = .007
    inner_radius_m: float = .006
    cable_radius_m: float = .0065
    young_modulus_pa: float = 1e6
    density_kg_m3: float = 300.
    tip_mass_kg: float = .0015
    dt: float = .005
    cable_speed_m_s: float = .001
    cable_limit_m: float = .003
    cable_force_n: float = 2.
    feed_speed_m_s: float = .005
    feed_limit_m: float = .050

    @property
    def length_m(self):
        return self.passive_length_m + self.active_length_m


def definition(config):
    c = config
    values = np.asarray(list(vars(c).values()), dtype=float)
    if not np.isfinite(values).all() or np.any(values <= 0.):
        raise ValueError('All model dimensions/material/control parameters must be finite and positive')
    if not 0. < c.inner_radius_m < c.cable_radius_m < c.radius_m:
        raise ValueError('Expected inner radius < cable radius < outer radius')
    if c.cable_limit_m >= c.active_length_m:
        raise ValueError('Cable shortening must be smaller than the active segment')
    npass = max(1, int(math.ceil(c.passive_length_m / c.edge_length_m)))
    nactive = max(1, int(math.ceil(c.active_length_m / c.edge_length_m)))
    s = np.r_[np.linspace(0., c.passive_length_m, npass + 1),
              np.linspace(c.passive_length_m, c.length_m, nactive + 1)[1:]]
    poses = np.zeros((len(s), 7))
    poses[:, 0] = s
    poses[:, 6] = 1.
    indices = np.arange(npass, len(s), dtype=int)
    paths = []
    for i in range(4):
        a = i * np.pi / 2.
        paths.append(np.column_stack([s[indices],
                     np.full(len(indices), c.cable_radius_m * np.cos(a)),
                     np.full(len(indices), c.cable_radius_m * np.sin(a))]))
    return dict(abscissa=s, poses=poses, active_index=npass,
                cable_indices=indices, cable_points=paths)


def advance_targets(shortening, feed, action, dt, config):
    a = np.asarray(action, dtype=float)
    q = np.asarray(shortening, dtype=float)
    if a.shape != (5,) or q.shape != (4,) or not np.isfinite(a).all() or not np.isfinite(q).all():
        raise ValueError('Expected finite four cable targets and five normalized actions')
    if not np.isfinite([feed, dt]).all() or dt <= 0. or np.any(np.abs(a) > 1.):
        raise ValueError('Invalid timestep/feed or normalized action outside [-1,1]')
    q = np.clip(q + a[:4] * config.cable_speed_m_s * dt,
                -config.cable_limit_m, config.cable_limit_m)
    feed = float(np.clip(feed + a[4] * config.feed_speed_m_s * dt,
                         -config.feed_limit_m, config.feed_limit_m))
    return q, feed
