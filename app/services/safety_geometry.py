"""Independent occupied-space checks over sensor-derived track footprints.

Continuous separating-axis intersection for translating oriented rectangles.
Track heading is estimated from velocity; slow tracks use the road heading.
This is a short constant-velocity sweep, not proof against unseen obstacles
or unmodelled rotation/acceleration.
"""
import math
from dataclasses import dataclass

@dataclass(frozen=True)
class Footprint:
    x: float
    z: float
    heading_rad: float
    length_m: float
    width_m: float
    vx: float = 0.0
    vz: float = 0.0

    def axes(self):
        f = (math.sin(self.heading_rad), -math.cos(self.heading_rad))
        return f, (-f[1], f[0])


def intersects_during(a: Footprint, b: Footprint, horizon_s: float = 0.1) -> bool:
    values = (*vars(a).values(), *vars(b).values(), horizon_s)
    if any(not math.isfinite(v) for v in values) or min(a.length_m,a.width_m,b.length_m,b.width_m) <= 0 or horizon_s < 0:
        return True  # Invalid occupied-space input fails closed.
    aa, ba = a.axes(), b.axes()
    enter, leave = 0.0, horizon_s
    for ux, uz in (*aa, *ba):
        radius = sum(abs(ux*fx + uz*fz)*extent for (fx,fz), extent in
                     ((aa[0], a.length_m/2), (aa[1], a.width_m/2),
                      (ba[0], b.length_m/2), (ba[1], b.width_m/2)))
        distance = (b.x-a.x)*ux + (b.z-a.z)*uz
        velocity = (b.vx-a.vx)*ux + (b.vz-a.vz)*uz
        if abs(velocity) < 1e-9:
            if abs(distance) > radius:
                return False
        else:
            t0, t1 = sorted(((-radius-distance)/velocity, (radius-distance)/velocity))
            enter, leave = max(enter,t0), min(leave,t1)
            if enter > leave:
                return False
    return enter <= leave
