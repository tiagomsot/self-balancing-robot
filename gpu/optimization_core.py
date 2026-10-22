"""Pure numerical objective and acquisition functions for GPU gain tuning."""

import math
import numpy as np

NAMES = ("kp", "kd", "ki")
BOUNDS = np.array([[.30, 4.0], [.020, .80], [0, 2.0]])


def search_values(gains):
    return np.asarray(gains)[..., [0, 1, 5]]


def full_gains(points, cap):
    """Preserve the actuator ABI; unmeasurable position/speed gains stay zero."""
    points = np.atleast_2d(np.asarray(points, float))
    return np.column_stack((points[:, :2], np.zeros((len(points), 2)), np.full(len(points), cap), points[:, 2]))


def expected_improvement(mean, std, best, exploration=.01):
    """Analytic EI for minimizing a Gaussian-posterior objective."""
    mean, std = np.broadcast_arrays(np.asarray(mean, float), np.asarray(std, float))
    improvement = best-mean-exploration
    positive = std > 1e-12
    z = np.divide(improvement, std, out=np.zeros_like(mean), where=positive)
    cdf = .5*(1+np.vectorize(math.erf, otypes=[float])(z/math.sqrt(2)))
    density = np.exp(-.5*z*z)/math.sqrt(2*math.pi)
    return np.maximum(0, np.where(positive, improvement*cdf+std*density,
                                 np.maximum(improvement, 0)))


def aggregate(costs):
    values = np.asarray(costs, float)
    return values.mean(axis=-1)+.75*values.max(axis=-1)


def score_worlds(stats, diagnostic, extra, finite, seconds):
    """Fixed dimensionless response objective; limits are model predictions."""
    s, d, e = np.asarray(stats), np.asarray(diagnostic), np.asarray(extra)
    degree = math.pi/180
    cost = (.6*s[:, 1]/seconds/(5*degree)**2
            + .05*(d[:, 0]/.6)**2
            + 1.5*e[:, 1]/degree**2 + .3*e[:, 2]/(5*degree)**2)
    # Extra columns 1..4 integrate over exactly the final second.
    ratios = np.stack((e[:, 0]/(20*degree), s[:, 4]/degree,
                       s[:, 5]/(5*degree), d[:, 0]/.6, d[:, 1]/1.2), axis=-1)
    passed = (np.asarray(finite, bool) & (d[:, 3] == 0)
              & np.isfinite(s).all(axis=1) & np.isfinite(d).all(axis=1)
              & np.isfinite(e).all(axis=1)
              & np.isfinite(ratios).all(axis=1) & (ratios <= 1+1e-6).all(axis=1))
    excess = np.maximum(0, ratios-1).clip(0, 100)
    cost = cost+1000*(~passed)+50*np.sum(excess**2, axis=-1)
    cost = np.nan_to_num(cost, nan=1e6, posinf=1e6, neginf=1e6).clip(0, 1e6)
    return cost, passed
