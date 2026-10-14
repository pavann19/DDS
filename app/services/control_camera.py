"""Synthetic pinhole RGB camera and pixel-only classical control perception.

Not a real camera or trained detector. Confidence is a shape-quality heuristic,
not a calibrated probability. Signal range is a proxy over the supported modeled
housing height interval 1.0-1.5 m; it is not calibrated real-world depth.
Frames contain pixels and calibration only, never object labels or schedules.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.ndimage import label, find_objects
from PIL import Image, ImageDraw


@dataclass(frozen=True)
class CameraCalibration:
    width: int = 640
    height: int = 360
    hfov_deg: float = 70.

    def __post_init__(self):
        if not (32 <= self.width <= 2048 and 32 <= self.height <= 2048 and
                math.isfinite(self.hfov_deg) and 10 <= self.hfov_deg <= 150):
            raise ValueError('Invalid camera calibration')

    @property
    def focal_px(self):
        return self.width / (2 * math.tan(math.radians(self.hfov_deg / 2)))


@dataclass(frozen=True)
class CameraFrame:
    rgb: bytes
    capture_time_s: float
    calibration: CameraCalibration = CameraCalibration()

    def __post_init__(self):
        if not isinstance(self.rgb,bytes) or len(self.rgb) != self.calibration.width*self.calibration.height*3:
            raise ValueError('Immutable RGB byte frame required')
        if not math.isfinite(self.capture_time_s):
            raise ValueError('Finite capture time required')

    def image(self):
        return Image.frombytes('RGB',(self.calibration.width,self.calibration.height),self.rgb)


@dataclass(frozen=True)
class ControlPrimitive:
    kind: str
    forward_m: float
    right_m: float = 0.
    height_m: float = 2.
    phase: str = 'unknown'


@dataclass(frozen=True)
class CameraDetection:
    kind: str
    phase: str | None
    bbox: tuple[int,int,int,int]
    bearing_deg: float
    range_m: float | None
    range_uncertainty_m: float | None
    confidence_heuristic: float
    capture_time_s: float


def render_control_camera(primitives, capture_time_s, calibration=CameraCalibration(), *, occlusions=(), drop=False):
    if drop:
        return None
    c=calibration
    image=Image.new('RGB',(c.width,c.height),(130,160,180))
    draw=ImageDraw.Draw(image)
    draw.rectangle((0,c.height//2,c.width,c.height),fill=(80,85,80))
    # Far objects first: nearer opaque signs naturally occlude distant ones.
    for p in sorted(primitives,key=lambda p:p.forward_m,reverse=True):
        if p.kind not in {'stop','yield','signal'}:
            raise ValueError('Unknown control primitive')
        if not all(math.isfinite(v) for v in (p.forward_m,p.right_m,p.height_m)):
            raise ValueError('Finite primitive position required')
        if p.forward_m <= .5:
            continue
        x=c.width/2+c.focal_px*p.right_m/p.forward_m
        y=c.height/2-c.focal_px*(p.height_m-1.4)/p.forward_m
        scale=c.focal_px/p.forward_m
        if not all(math.isfinite(v) for v in (x,y,scale)):
            continue
        if x+scale<c.width*0 or x-scale>c.width or y+scale<0 or y-scale>c.height:
            continue
        if p.kind=='signal':
            w,h=.36*scale,1.0*scale
            draw.rectangle((x-w/2,y-h/2,x+w/2,y+h/2),fill=(18,18,18))
            for i,phase in enumerate(('red','amber','green')):
                cy=y+(i-1)*h*.30
                radius=w*.30
                color={'red':(245,25,25),'amber':(245,185,15),'green':(20,235,50)}[phase] if p.phase==phase else (45,45,45)
                draw.ellipse((x-radius,cy-radius,x+radius,cy+radius),fill=color)
        elif p.kind=='stop':
            r=.75*scale/2
            points=[(x+r*math.cos(math.pi/8+i*math.pi/4),y+r*math.sin(math.pi/8+i*math.pi/4)) for i in range(8)]
            draw.polygon(points,fill=(240,25,25))
        else:
            w=.9*scale
            points=[(x-w/2,y-w*.433),(x+w/2,y-w*.433),(x,y+w*.433)]
            draw.polygon(points,fill=(240,25,25))
            draw.polygon([(x+(a-x)*.65,y+(b-y)*.65) for a,b in points],fill=(235,235,225))
    for box in occlusions:
        draw.rectangle(box,fill=(100,105,110))
    return CameraFrame(image.tobytes(),capture_time_s,c)


def _components(mask):
    # Preserve four-neighbour connectivity and the eight-pixel evidence floor;
    # compiled labeling avoids Python per-pixel work on each camera view.
    labeled,_=label(mask,structure=np.array([[0,1,0],[1,1,1],[0,1,0]]))
    for component,bounds in enumerate(find_objects(labeled),start=1):
        if bounds is None: continue
        points=np.argwhere(labeled[bounds]==component)
        if len(points)>=8:
            points += np.array([bounds[0].start,bounds[1].start])
            yield points


def detect_controls(frame):
    """Pixel-only observations; absent/partial geometry never supplies a phase."""
    if frame is None:return ()
    c=frame.calibration
    rgb=np.frombuffer(frame.rgb,dtype=np.uint8).reshape(c.height,c.width,3)
    red=(rgb[:,:,0]>170)&(rgb[:,:,1]<90)&(rgb[:,:,2]<90)
    # Exact all-channel threshold without a signed copy or pixel reduction.
    dark=(rgb[:,:,0]<65)&(rgb[:,:,1]<65)&(rgb[:,:,2]<65)
    detections=[]
    def result(kind,phase,box,size,quality):
        x0,y0,x1,y1=box
        clipped=x0<=0 or y0<=0 or x1>=c.width-1 or y1>=c.height-1
        extent=(y1-y0+1) if kind=='signal' else (x1-x0+1)
        # Both supported renderers use signal heads within 1.0-1.5 m.
        # Use the midpoint and explicitly bound this size ambiguity, without
        # selecting a renderer or consulting an object label outside pixels.
        nominal_size=1.25 if kind=='signal' else size
        distance=c.focal_px*nominal_size/extent if not clipped and kind!='unknown' else None
        detections.append(CameraDetection(kind,phase,box,
            math.degrees(math.atan(((x0+x1)/2-c.width/2)/c.focal_px)),distance,
            distance*(.15+2/extent+(.2 if kind=='signal' else 0.)) if distance is not None else None,
            min(.95,max(0.,quality)) if not clipped else .2,frame.capture_time_s))
    # Signal requires a tall dark housing enclosing three lens positions.
    signal_boxes=[]
    for points in _components(dark):
        y0,x0=points.min(axis=0);y1,x1=points.max(axis=0)
        # A Three housing may be connected to a narrow support pole. Keep
        # only a contiguous broad head; the pole is not optical object size.
        widths=np.bincount(points[:,0]-y0, minlength=y1-y0+1)
        broad=np.flatnonzero(widths>=max(4,widths.max()*.45))
        if len(broad):
            y0=int(y0+broad[0]);y1=int(y0+broad[-1]-broad[0])
            points=points[(points[:,0]>=y0)&(points[:,0]<=y1)]
            x0=int(points[:,1].min());x1=int(points[:,1].max())
        w,h=x1-x0+1,y1-y0+1
        if w<6 or h<18 or not 1.35<h/w<3.7 or len(points)/(w*h)<.40:continue
        if x0<=0 or y0<=0 or x1>=c.width-1 or y1>=c.height-1:continue
        # Require three actual lens-shaped pixel components, including the
        # inactive grey lenses. A colored patch on a black pole is insufficient.
        crop=rgb[y0:y1+1,x0:x1+1]
        neutral=(np.max(crop,axis=2)-np.min(crop,axis=2)<8)&(np.min(crop,axis=2)>=38)&(np.max(crop,axis=2)<=60)
        colored=((crop[:,:,0]>170)&(crop[:,:,2]<90)&(crop[:,:,1]<220))|((crop[:,:,1]>150)&(crop[:,:,0]<100)&(crop[:,:,2]<110))
        lenses=[]
        for lens in _components(neutral|colored):
            ly0,lx0=lens.min(axis=0);ly1,lx1=lens.max(axis=0)
            lw,lh=lx1-lx0+1,ly1-ly0+1
            if 0<ly0 and ly1<h-1 and 2<=lw<=w*.8 and 2<=lh<=h*.35 and .25<=lw/lh<=2.:
                lenses.append(((ly0+ly1)/2,(lx0+lx1)/2,lens))
        lenses.sort(key=lambda item:item[0])
        if len(lenses)!=3:continue
        # A circular lens becomes an ellipse under an oblique pinhole view.
        # Support resolved projected aspect 0.25-0.5 only when ALL three
        # complete lenses share that compression. This roughly corresponds
        # to 60-75 degree planar incidence under weak perspective; it is not
        # a measured pose or a calibrated physical-camera angular guarantee.
        lens_shapes=[]
        for _,_,pixels in lenses:
            ly0,lx0=pixels.min(axis=0);ly1,lx1=pixels.max(axis=0)
            lens_shapes.append((lx1-lx0+1,ly1-ly0+1,len(pixels)))
        # Resolved complete oblique triplets retain a qualifying shape score
        # with a projection penalty (.75 vs .85), never a probability claim.
        projected=any(width/height<.5 for width,height,_ in lens_shapes)
        if projected:
            aspects=[width/height for width,height,_ in lens_shapes]
            if (any(width<3 or height<6 or count<18 for width,height,count in lens_shapes)
                or max(aspects)>.5 or max(aspects)-min(aspects)>.12):continue
        ys=[item[0] for item in lenses];xs=[item[1] for item in lenses]
        if max(xs)-min(xs)>max(2,w*.2) or min(np.diff(ys))<h*.18 or max(np.diff(ys))>h*.42:continue
        phases=[]
        for i,phase in enumerate(('red','amber','green')):
            lens=lenses[i][2]
            r,g,b=crop[lens[:,0],lens[:,1]].T
            active=(r>150)&(g<90)&(b<90) if phase=='red' else (r>150)&(g>100)&(g<220)&(b<90) if phase=='amber' else (g>150)&(r<100)&(b<110)
            # Small illuminated patches remain evidence, even when surrounded
            # by inactive grey pixels. Never average away a conflicting lamp.
            if np.count_nonzero(active)>=max(4,len(lens)*.10):phases.append(phase)
        box=tuple(map(int,(x0,y0,x1,y1)));signal_boxes.append(box)
        result('signal',phases[0] if len(phases)==1 else 'unknown',box,1.,(.75 if projected else .85) if len(phases)==1 else .3)
    for points in _components(red):
        y0,x0=points.min(axis=0);y1,x1=points.max(axis=0)
        w,h=x1-x0+1,y1-y0+1
        if w<10 or h<10:continue
        if any(a<=x0<=d and b<=y0<=e for a,b,d,e in signal_boxes):continue
        if not .75<w/h<1.4:
            result('unknown',None,tuple(map(int,(x0,y0,x1,y1))),.75,.2);continue
        if x0==0 or y0==0 or x1==c.width-1 or y1==c.height-1:
            result('unknown',None,tuple(map(int,(x0,y0,x1,y1))),.75,.2);continue
        row_counts=np.bincount(points[:,0]-y0,minlength=h)
        fill=len(points)/(w*h)
        top=row_counts[:max(1,h//5)].mean()/w
        bottom=row_counts[-max(1,h//5):].mean()/w
        box=tuple(map(int,(x0,y0,x1,y1)))
        if .72<fill<.92 and .35<top<.85 and .35<bottom<.85:
            result('stop',None,box,.75,.8)
        elif .15<fill<.48 and top>.65 and bottom<.25:
            result('yield',None,box,.9,.75)
        else:
            result('unknown',None,box,.75,.2)
    return tuple(detections)
