"""Continuous polynomial corridor bounds in the modeled Frenet coordinates.

Outward-rounded Bernstein hulls prove scalar polynomial ranges, including between
samples. This does not prove Cartesian body clearance on a discontinuous polyline
normal, surveyed road width, obstacles, or tracking of the nominal trajectory.
"""
import math


def _add(a,b):
    return math.nextafter(a[0]+b[0],-math.inf),math.nextafter(a[1]+b[1],math.inf)


def _multiply(a,b):
    products=(a[0]*b[0],a[0]*b[1],a[1]*b[0],a[1]*b[1])
    return math.nextafter(min(products),-math.inf),math.nextafter(max(products),math.inf)


def _power(a,n):
    value=(1.,1.)
    for _ in range(n):value=_multiply(value,a)
    return value


def _bernstein_intervals(coefficients,start,end):
    degree=len(coefficients)-1
    width=(math.nextafter(end-start,-math.inf),math.nextafter(end-start,math.inf))
    width_powers=[(1.,1.)]
    for _ in range(degree):width_powers.append(_multiply(width_powers[-1],width))
    if start==0.:
        powers=[_multiply((coefficient,coefficient),width_powers[i]) for i,coefficient in enumerate(coefficients)]
    else:
        start_powers=[(1.,1.)]
        for _ in range(degree):start_powers.append(_multiply(start_powers[-1],(start,start)))
        powers=[]
        for j in range(degree+1):
            value=(0.,0.)
            for i in range(j,degree+1):
                term=_multiply((coefficients[i],coefficients[i]),(float(math.comb(i,j)),)*2)
                term=_multiply(term,start_powers[i-j])
                value=_add(value,term)
            powers.append(_multiply(value,width_powers[j]))
    hull=[]
    for k in range(degree+1):
        value=(0.,0.)
        for j in range(k+1):
            ratio=math.comb(k,j)/math.comb(degree,j)
            factor=(math.nextafter(ratio,-math.inf),math.nextafter(ratio,math.inf))
            value=_add(value,_multiply(powers[j],factor))
        hull.append(value)
    return tuple(hull)


def _split(hull):
    rows=[hull]
    while len(rows[-1])>1:
        rows.append(tuple(_multiply(_add(a,b),(.5,.5)) for a,b in zip(rows[-1],rows[-1][1:])))
    return tuple(row[0] for row in rows),tuple(row[-1] for row in reversed(rows))


def polynomial_within(poly,lower,upper,start=0.,end=None,tolerance=1e-7,max_depth=14):
    """Conservatively prove p(t) in [lower-tol, upper+tol] for every t.

    Unproved/invalid intervals reject. Coefficients and endpoints are the exact
    input floating-point numbers; interval arithmetic rounds all operations out.
    """
    end=poly.T if end is None else end
    values=(*poly.coef,lower,upper,start,end,tolerance)
    if not all(math.isfinite(v) for v in values) or start<0 or end<start or lower>upper or tolerance<0 or max_depth<0:
        return False
    lower,upper=lower-tolerance,upper+tolerance
    hull=_bernstein_intervals(poly.coef,start,end)
    stack=[(hull,0)]
    while stack:
        current,depth=stack.pop()
        if not all(math.isfinite(v) for interval in current for v in interval):return False
        if min(v[0] for v in current)>=lower and max(v[1] for v in current)<=upper:continue
        if current[0][0]>upper or current[0][1]<lower or current[-1][0]>upper or current[-1][1]<lower:return False
        if depth>=max_depth:return False
        left,right=_split(current)
        stack.extend(((left,depth+1),(right,depth+1)))
    return True


def lateral_corridor_within(lat,d1,cfg,start=0.,end=None):
    """Continuous center corridor with an explicit modeled physical half-width."""
    end=lat.T if end is None else min(end,lat.T)
    d0=lat.pos(0.)
    edge=cfg.road_half_width_m-cfg.edge_margin_m-cfg.vehicle_half_width_m
    lower=max(-edge,min(d0,d1)-cfg.max_corridor_excursion_m)
    upper=min(edge,max(d0,d1)+cfg.max_corridor_excursion_m)
    if start>lat.T:
        return lower-cfg.corridor_numeric_tolerance_m <= lat.pos(lat.T) <= upper+cfg.corridor_numeric_tolerance_m
    return polynomial_within(lat,lower,upper,start,min(end,lat.T),cfg.corridor_numeric_tolerance_m)


def longitudinal_corridor_within(lon,horizon,total,cfg,start=0.):
    """Continuous route station containment, including terminal velocity hold."""
    upper=min(horizon,lon.T)
    if start<=upper and not polynomial_within(lon,0.,total,start,upper,cfg.corridor_numeric_tolerance_m):return False
    if horizon>lon.T:
        a=max(start,lon.T)
        positions=(lon.pos(lon.T)+lon.vel(lon.T)*(a-lon.T),lon.pos(lon.T)+lon.vel(lon.T)*(horizon-lon.T))
        if min(positions)<-cfg.corridor_numeric_tolerance_m or max(positions)>total+cfg.corridor_numeric_tolerance_m:return False
    return True

def lateral_corridor_batch(laterals,cfg):
    """Outward-rounded whole-interval hulls in one batch; subdivide uncertain rows."""
    import numpy as np
    polys=[item[0] for item in laterals]
    if not polys:return {}
    coefficients=np.array([poly.coef for poly in polys],dtype=float)
    durations=np.array([poly.T for poly in polys])
    width_lower=np.nextafter(durations,-np.inf);width_upper=np.nextafter(durations,np.inf)
    lower_power=np.ones(len(polys));upper_power=lower_power.copy()
    power_lower=[];power_upper=[]
    for j in range(coefficients.shape[1]):
        values_low=coefficients[:,j]*lower_power;values_high=coefficients[:,j]*upper_power
        power_lower.append(np.nextafter(np.minimum(values_low,values_high),-np.inf))
        power_upper.append(np.nextafter(np.maximum(values_low,values_high),np.inf))
        lower_power=np.nextafter(lower_power*width_lower,-np.inf)
        upper_power=np.nextafter(upper_power*width_upper,np.inf)
    degree=coefficients.shape[1]-1
    hull_lower=[];hull_upper=[]
    for k in range(degree+1):
        lo=np.zeros(len(polys));hi=lo.copy()
        for j in range(k+1):
            ratio=math.comb(k,j)/math.comb(degree,j)
            rl=math.nextafter(ratio,-math.inf);rh=math.nextafter(ratio,math.inf)
            term_low=np.nextafter(np.minimum(power_lower[j]*rl,power_lower[j]*rh),-np.inf)
            term_high=np.nextafter(np.maximum(power_upper[j]*rl,power_upper[j]*rh),np.inf)
            lo=np.nextafter(lo+term_low,-np.inf);hi=np.nextafter(hi+term_high,np.inf)
        hull_lower.append(lo);hull_upper.append(hi)
    d0=coefficients[:,0];d1=np.array([item[2] for item in laterals])
    edge=cfg.road_half_width_m-cfg.edge_margin_m-cfg.vehicle_half_width_m
    allowed_lower=np.maximum(-edge,np.minimum(d0,d1)-cfg.max_corridor_excursion_m)-cfg.corridor_numeric_tolerance_m
    allowed_upper=np.minimum(edge,np.maximum(d0,d1)+cfg.max_corridor_excursion_m)+cfg.corridor_numeric_tolerance_m
    lower=np.min(np.array(hull_lower),axis=0);upper=np.max(np.array(hull_upper),axis=0)
    proved=np.isfinite(lower)&np.isfinite(upper)&(durations>0)&(lower>=allowed_lower)&(upper<=allowed_upper)
    return {id(poly):True if proved[i] else lateral_corridor_within(poly,laterals[i][2],cfg)
            for i,poly in enumerate(polys)}
