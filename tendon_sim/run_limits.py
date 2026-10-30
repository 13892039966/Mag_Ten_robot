"""An explicit None disables a time budget; NaN/Infinity never do implicitly."""
import math
def checked_limit(value):
    if value is None:return None
    value=float(value)
    if not math.isfinite(value) or value<=0:raise ValueError('Time limit must be positive finite or None')
    return value
def reached_limit(elapsed,limit):
    limit=checked_limit(limit)
    return limit is not None and elapsed>=limit

def control_stride(control_period,physics_dt):
    period=checked_limit(control_period);dt=checked_limit(physics_dt)
    if period is None or dt is None:raise ValueError('Control and physics intervals must be finite positive')
    ratio=period/dt;steps=int(round(ratio))
    if steps<1 or abs(ratio-steps)>1e-9:raise ValueError('Control period must be an integer multiple of physics dt')
    return steps
