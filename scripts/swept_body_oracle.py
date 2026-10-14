"""Offline clearance of an explicit observed-pose interpolant, never driving input.

Each rigid convex body part has fixed local (x,z) vertices. Center interpolates
linearly and yaw follows its shortest linear angular interval. This assumption
does not establish the actual vehicle trajectory between telemetry samples.
Touching/numerically ambiguous clearance stays unresolved.
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    x: float
    z: float
    yaw_rad: float


@dataclass(frozen=True)
class BodyMotion:
    start: Pose
    end: Pose
    parts: tuple[tuple[tuple[float, float], ...], ...]


@dataclass(frozen=True)
class SweptResult:
    status: str  # clear | overlap_observed | unresolved
    checked_intervals: int
    deepest_subdivision: int
    overlap_fraction: float | None = None


def rectangle_parts(length_m, width_m):
    if not all(math.isfinite(v) and v > 0 for v in (length_m, width_m)):
        raise ValueError('Rectangle dimensions must be finite and positive')
    return (((-width_m/2, -length_m/2), (width_m/2, -length_m/2),
             (width_m/2, length_m/2), (-width_m/2, length_m/2)),)


def _validate(motion):
    if not all(math.isfinite(v) for p in (motion.start, motion.end)
               for v in (p.x, p.z, p.yaw_rad)) or not motion.parts:
        raise ValueError('Finite poses and nonempty body parts required')
    for part in motion.parts:
        if len(part) < 3 or not all(len(point)==2 and all(math.isfinite(v) for v in point) for point in part):
            raise ValueError('Finite convex polygon vertices required')
        hull = _hull(part)
        if len(hull) != len(set(part)) or len(hull) < 3:
            raise ValueError('Body parts must be nondegenerate convex polygons')


def _hull(points):
    points = sorted(set(points))
    def cross(a,b,c):
        value=(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
        if not math.isfinite(value):raise ValueError('Geometry arithmetic exceeds finite domain')
        return value
    lower,upper=[],[]
    for p in points:
        while len(lower)>=2 and cross(lower[-2],lower[-1],p)<=0:lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper)>=2 and cross(upper[-2],upper[-1],p)<=0:upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]


def _angle(motion):
    start,end=(math.remainder(p.yaw_rad,2*math.pi) for p in (motion.start,motion.end))
    return (end-start+math.pi)%(2*math.pi)-math.pi


def _vertices(motion, part, fraction):
    yaw=math.remainder(motion.start.yaw_rad,2*math.pi)+fraction*_angle(motion)
    c,s=math.cos(yaw),math.sin(yaw)
    x=motion.start.x+fraction*(motion.end.x-motion.start.x)
    z=motion.start.z+fraction*(motion.end.z-motion.start.z)
    vertices=[(x+u*c-v*s,z+u*s+v*c) for u,v in _hull(part)]
    if not all(math.isfinite(v) for point in vertices for v in point):
        raise ValueError('Interpolated geometry exceeds finite domain')
    if len(_hull(vertices))<3:
        raise ValueError('Transformed geometry loses polygon resolution')
    return vertices


def _guard(*polygons):
    scale=max(1.,*(abs(v) for polygon in polygons for point in polygon for v in point))
    return 64*math.ulp(scale)


def _separation(a,b):
    """Largest normalized SAT separation; negative means strict overlap."""
    gaps=[]
    for polygon in (a,b):
        for i,p in enumerate(polygon):
            q=polygon[(i+1)%len(polygon)]
            dx,dz=q[0]-p[0],q[1]-p[1]
            length=math.hypot(dx,dz)
            if length==0:continue
            nx,nz=-dz/length,dx/length
            ap=[x*nx+z*nz for x,z in a]
            bp=[x*nx+z*nz for x,z in b]
            if not all(math.isfinite(v) for v in ap+bp):
                raise ValueError('SAT projection exceeds finite domain')
            gaps.append(max(min(bp)-max(ap),min(ap)-max(bp)))
    return max(gaps) if gaps else -math.inf


def _sweep(motion,part,lo,hi):
    endpoints=_vertices(motion,part,lo)+_vertices(motion,part,hi)
    radius=max(math.hypot(x,z) for x,z in part)
    # Stable sagitta avoids catastrophic cancellation for tiny angles.
    delta=abs(_angle(motion))*(hi-lo)
    sagitta=2*radius*math.sin(delta/4)**2
    inflation=math.nextafter(sagitta+_guard(endpoints),math.inf)
    return _hull(endpoints),inflation


def check_swept_clearance(a: BodyMotion,b: BodyMotion,*,max_depth=12):
    """Compare bodies over the SAME normalized time interval [0,1].

    Clear requires every simultaneous interval's inflated endpoint hulls to
    separate. Overlap requires an evaluated actual interpolant pose pair to
    strictly overlap. An unproved leaf remains unresolved, never clear.
    """
    _validate(a);_validate(b)
    if not isinstance(max_depth,int) or not 0<=max_depth<=20:
        raise ValueError('max_depth must be an integer in [0,20]')
    count,deepest=0,0
    def visit(lo,hi,depth):
        nonlocal count,deepest
        count+=1;deepest=max(deepest,depth)
        mid=(lo+hi)/2
        for t in (lo,mid,hi):
            for ap in a.parts:
                av=_vertices(a,ap,t)
                for bp in b.parts:
                    bv=_vertices(b,bp,t)
                    if _separation(av,bv)<-_guard(av,bv):
                        return 'overlap_observed',t
        separated=True
        for ap in a.parts:
            ah,ai=_sweep(a,ap,lo,hi)
            for bp in b.parts:
                bh,bi=_sweep(b,bp,lo,hi)
                margin=math.nextafter(ai+bi+_guard(ah,bh),math.inf)
                if _separation(ah,bh)<=margin:separated=False
        if separated:return 'clear',None
        if depth==max_depth:return 'unresolved',None
        left=visit(lo,mid,depth+1)
        if left[0]=='overlap_observed':return left
        right=visit(mid,hi,depth+1)
        if right[0]=='overlap_observed':return right
        return ('unresolved',None) if 'unresolved' in (left[0],right[0]) else ('clear',None)
    status,t=visit(0.,1.,0)
    return SweptResult(status,count,deepest,t)
