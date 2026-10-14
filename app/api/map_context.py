"""Bounded recorded centerlines for display only; widths remain modeled."""
import math
from fastapi import APIRouter, Query
from app.services.road_semantics import recorded_map
router = APIRouter()

@router.get("/map/context")
def nearby_roads(lat: float = Query(ge=-90, le=90), lng: float = Query(ge=-180, le=180)):
    scale = 111320 * math.cos(math.radians(lat))
    roads = []
    for feature in recorded_map().features:
        if feature['tags'].get('highway') not in {'motorway','motorway_link','trunk','trunk_link','primary','secondary','tertiary','residential','unclassified','service'}:
            continue
        points = feature['geometry']
        # Segment/bounding-box intersection keeps crossing roads even when
        # both endpoints lie outside the display window.
        segments=[]
        for a,b in zip(points,points[1:]):
            ax,az=(a[1]-lng)*scale,(a[0]-lat)*111320
            bx,bz=(b[1]-lng)*scale,(b[0]-lat)*111320
            if min(ax,bx)<=220 and max(ax,bx)>=-220 and min(az,bz)<=220 and max(az,bz)>=-220:
                segments.append([a,b])
        if segments:
            roads.append({'way_id':feature['id'],'segments':segments[:128],'timestamp':feature.get('timestamp')})
        if len(roads)>=100:break
    return {'roads':roads,'source':'recorded_OSM_centerlines','width_source':'modeled_display_only','live_verified':False}
