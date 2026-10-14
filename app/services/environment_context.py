"""Bounded real OSM surroundings; unsupported/missing features remain absent."""
import asyncio
from collections import OrderedDict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import httpx

RADIUS_M=300
MAX_FEATURES=160
MAX_VERTICES=128
MAX_INPUT_VERTICES=4096
SCHEMA_VERSION=3
MAX_BYTES=4_000_000
CACHE=OrderedDict()
LOCK=asyncio.Lock()
CACHE_DIR=Path(__file__).resolve().parents[2]/'output'/'environment-cache'


def cell(lat,lng):
    if not math.isfinite(lat) or not math.isfinite(lng) or not -85<=lat<=85 or not -180<=lng<=180:
        raise ValueError('Finite supported geographic position required')
    # Roughly 110 m latitude cells; query/cache identity never changes silently.
    return round(lat,3),round(lng,3)


def clip_infrastructure(raw,lat,lng):
    """Clip recorded OSM line segments to the local 600 m disk.

    Boundary points are interpolation on supplied segments, not surveyed nodes.
    Invalid coordinates reject the complete way; exits break runs, never bridge
    absent/outside geometry. Large valid runs split with one shared endpoint.
    """
    if not isinstance(raw,list) or not 2<=len(raw)<=MAX_INPUT_VERTICES:return []
    scale=111320*math.cos(math.radians(lat));points=[]
    for p in raw:
        if not isinstance(p,dict):return []
        a,b=p.get('lat'),p.get('lon')
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in (a,b)):return []
        if not -90<=a<=90 or not -180<=b<=180:return []
        points.append(((b-lng)*scale,(a-lat)*111320,[a,b]))
    radius=RADIUS_M*2;runs=[];run=[]
    def flush():
        nonlocal run
        if len(run)>=2:runs.append(run)
        run=[]
    for a,b in zip(points,points[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1];aa=dx*dx+dy*dy
        if aa<1e-12:continue
        bb=2*(a[0]*dx+a[1]*dy);cc=a[0]*a[0]+a[1]*a[1]-radius*radius
        discriminant=bb*bb-4*aa*cc
        if discriminant<=0:flush();continue
        root=math.sqrt(discriminant)
        low=max(0.,(-bb-root)/(2*aa));high=min(1.,(-bb+root)/(2*aa))
        if high-low<=1e-12:flush();continue
        def at(t):
            if t==0:return a[2]
            if t==1:return b[2]
            return [lat+(a[1]+dy*t)/111320,lng+(a[0]+dx*t)/scale]
        start,end=at(low),at(high)
        if run and (low>0 or run[-1]!=start):flush()
        if not run:run=[start]
        if end!=run[-1]:run.append(end)
        if len(run)==MAX_VERTICES:
            runs.append(run);run=[run[-1]]
        if high<1:flush()
    flush()
    return runs


def normalize(data,lat,lng):
    features=[]
    infrastructure=[]
    def point(p):
        if not isinstance(p,dict):return None
        a,b=p.get('lat'),p.get('lon')
        if isinstance(a,bool) or isinstance(b,bool) or not isinstance(a,(int,float)) or not isinstance(b,(int,float)) or not math.isfinite(a+b):return None
        if math.hypot((a-lat)*111320,(b-lng)*111320*math.cos(math.radians(lat)))>RADIUS_M*2:return None
        return [a,b]
    if not isinstance(data,dict):raise ValueError('Invalid OSM document')
    items=data.get('elements',[])
    if not isinstance(items,list):raise ValueError('Invalid OSM elements')
    for item in items[:2000]:
        if not isinstance(item,dict):continue
        tags=item.get('tags',{})
        if not isinstance(tags,dict):continue
        kind='building' if tags.get('building') not in (None,'no','false','0') else 'tree' if tags.get('natural')=='tree' else 'park' if tags.get('leisure')=='park' else None
        if not kind:
            if len(infrastructure)>=MAX_FEATURES:continue
            road=tags.get('highway')
            railway=tags.get('railway')
            if not isinstance(road,(str,type(None))) or not isinstance(railway,(str,type(None))):continue
            if road not in {'motorway','trunk','primary','secondary','tertiary','unclassified','residential','service','living_street'} and railway not in {'rail','light_rail','tram','narrow_gauge'}:continue
            raw=item.get('geometry',[])
            runs=clip_infrastructure(raw,lat,lng)
            if not runs:continue
            def number(key,low,high):
                raw=tags.get(key)
                if isinstance(raw,bool):return None
                try:value=float(str(raw).removesuffix(' m'))
                except (ValueError,TypeError):return None
                return value if math.isfinite(value) and low<=value<=high else None
            width=number('width',1,60)
            lanes=number('lanes',1,12)
            inferred=(lanes*3.2 if lanes is not None else (3. if road=='service' else 6.4))
            for run_index,geometry in enumerate(runs):
                if len(infrastructure)>=MAX_FEATURES:break
                infrastructure.append({'osm_id':item.get('id'),'geometry_id':f"{item.get('type','way')}:{item.get('id')}:{run_index}",'geometry_source':'OSM_segments_clipped_boundary_interpolation','kind':'road' if road else 'railway',
                'geometry':geometry,'class':road or railway,'width_m':width if width is not None else (inferred if road else 1.435),
                'width_source':'recorded_OSM_width_tag' if width is not None else 'modeled_from_lanes' if lanes is not None else 'modeled_display_width',
                'bridge':isinstance(tags.get('bridge'),str) and tags.get('bridge') not in {'no','false','0'},'tunnel':isinstance(tags.get('tunnel'),str) and tags.get('tunnel') not in {'no','false','0'},
                'layer':number('layer',-10,10),'elevation_source':'unavailable_ground_projection',
                'timestamp':item.get('timestamp'),'version':item.get('version')})
            continue
        if len(features)>=MAX_FEATURES:continue
        if kind=='tree':
            p=point(item)
            if p is None:continue
            geometry=[p]
        else:
            raw=item.get('geometry',[])
            if not isinstance(raw,list) or not 4<=len(raw)<=MAX_VERTICES:continue
            geometry=[point(p) for p in raw]
            if any(p is None for p in geometry) or geometry[0]!=geometry[-1]:continue
        height=None
        try:
            raw=tags.get('height')
            height=float(str(raw).removesuffix(' m')) if raw is not None and not isinstance(raw,bool) else None
        except (ValueError,AttributeError):pass
        if height is not None and not 2<=height<=100:height=None
        height_source='recorded_OSM_height_tag' if height is not None else 'modeled_display_height'
        if height is None and kind=='building':
            try:levels=float(tags.get('building:levels',''))
            except (ValueError,TypeError):levels=0
            if math.isfinite(levels) and 1<=levels<=30:
                height=levels*3.;height_source='modeled_from_recorded_levels'

        features.append({'osm_type':item.get('type'),'osm_id':item.get('id'),'kind':kind,
            'geometry':geometry,'height_m':height if height is not None else (9. if kind=='building' else 6.),
            'height_source':height_source,
            'timestamp':item.get('timestamp'),'version':item.get('version')})

    return {'status':'available','features':features,'infrastructure':infrastructure,'schema_version':SCHEMA_VERSION,'source':'OpenStreetMap_geometry',
        'center':[lat,lng],'radius_m':RADIUS_M,'retrieved_at':datetime.now(timezone.utc).isoformat(),
        'live_verified':False,'geometry_scope':'mapped_positions_footprints_not_photorealistic',
        'attribution':'OpenStreetMap contributors (ODbL)','height_scope':'missing_heights_modeled',
        'osm_base_timestamp':data.get('osm3s',{}).get('timestamp_osm_base') if isinstance(data.get('osm3s'),dict) else None,
        'data_provider':data.get('_provider','Overpass_API')}


async def fetch_osm(lat,lng):
    import xml.etree.ElementTree as ET
    query=f'[out:json][timeout:10];(way["building"](around:{RADIUS_M},{lat},{lng});node["natural"="tree"](around:{RADIUS_M},{lat},{lng});way["leisure"="park"](around:{RADIUS_M},{lat},{lng});way["highway"](around:{RADIUS_M},{lat},{lng});way["railway"](around:{RADIUS_M},{lat},{lng}););out meta geom;'
    async def read(client,url,params):
        async with client.stream('GET',url,params=params) as response:
            response.raise_for_status();content=bytearray()
            async for chunk in response.aiter_bytes():
                content.extend(chunk)
                if len(content)>MAX_BYTES:raise ValueError('Environment response exceeded size bound')
        return content
    async with httpx.AsyncClient(timeout=14.) as client:
        try:
            return json.loads(await read(client,'https://overpass-api.de/api/interpreter',{'data':query}))
        except (httpx.HTTPError,ValueError):
            # Official map API fallback: real node references, not generated shapes.
            dlat=RADIUS_M/111320;dlng=dlat/math.cos(math.radians(lat))
            bbox=f'{lng-dlng},{lat-dlat},{lng+dlng},{lat+dlat}'
            content=await read(client,'https://api.openstreetmap.org/api/0.6/map',{'bbox':bbox})
    try:root=ET.fromstring(content)
    except ET.ParseError as exc:raise ValueError('Invalid OSM XML') from exc
    nodes={};elements=[]
    for node in root.findall('node'):
        tags={t.attrib['k']:t.attrib['v'] for t in node.findall('tag')}
        point={'lat':float(node.attrib['lat']),'lon':float(node.attrib['lon'])};nodes[node.attrib['id']]=point
        if tags.get('natural')=='tree':elements.append(dict(point,type='node',id=int(node.attrib['id']),tags=tags,timestamp=node.get('timestamp'),version=node.get('version')))
    for way in root.findall('way'):
        tags={t.attrib['k']:t.attrib['v'] for t in way.findall('tag')}
        if 'building' not in tags and tags.get('leisure')!='park' and 'highway' not in tags and 'railway' not in tags:continue
        refs=[n.attrib['ref'] for n in way.findall('nd')]
        if all(ref in nodes for ref in refs):elements.append({'type':'way','id':int(way.attrib['id']),'tags':tags,'geometry':[nodes[ref] for ref in refs],'timestamp':way.get('timestamp'),'version':way.get('version')})
    return {'elements':elements,'_provider':'OpenStreetMap_official_map_API'}


async def surroundings(lat,lng):
    lat,lng=cell(lat,lng);key=f'{lat:.3f}_{lng:.3f}'
    async with LOCK:
        cached=CACHE.get(key)
        if cached and (cached[1].get('schema_version')==SCHEMA_VERSION or cached[1].get('recorded_fallback')) and time.monotonic()-cached[0]<(86400 if cached[1]['status']=='available' and not cached[1].get('recorded_fallback') else 60):
            CACHE.move_to_end(key);return dict(cached[1],cache_hit=True)
        path=CACHE_DIR/(key+'.json')
        try:
            if path.exists() and time.time()-path.stat().st_mtime<86400:
                try:
                    payload=json.loads(path.read_text(encoding='utf-8'))
                    valid=payload.get('schema_version')==SCHEMA_VERSION and len(payload.get('infrastructure',[]))<=MAX_FEATURES and payload.get('source')=='OpenStreetMap_geometry' and payload.get('center')==[lat,lng] and len(payload.get('features',[]))<=MAX_FEATURES
                    valid=valid and all(isinstance(f,dict) and f.get('kind') in {'road','railway'} and 2<=len(f.get('geometry',[]))<=MAX_VERTICES and isinstance(f.get('width_m'),(int,float)) and math.isfinite(f['width_m']) and 1<=f['width_m']<=60 and all(isinstance(p,list) and len(p)==2 and all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in p) for p in f['geometry']) for f in payload.get('infrastructure',[]))
                    valid=valid and all(1<=len(f.get('geometry',[]))<=MAX_VERTICES and f.get('kind') in {'building','tree','park'} and 2<=f.get('height_m',0)<=100 for f in payload.get('features',[]))
                    if valid:
                        CACHE[key]=(time.monotonic()-(time.time()-path.stat().st_mtime),payload)
                        while len(CACHE)>64:CACHE.popitem(last=False)
                        return dict(payload,cache_hit=True)
                except (ValueError,TypeError,KeyError):pass
            payload=normalize(await fetch_osm(lat,lng),lat,lng)
            CACHE_DIR.mkdir(parents=True,exist_ok=True)
            path.write_text(json.dumps(payload),encoding='utf-8')
        except (httpx.HTTPError,ValueError,OSError,json.JSONDecodeError) as exc:
            payload={'status':'unavailable','features':[],'infrastructure':[],'schema_version':SCHEMA_VERSION,'source':'OpenStreetMap_geometry',
                'center':[lat,lng],'radius_m':RADIUS_M,'live_verified':False,'error':type(exc).__name__,
                'attribution':'OpenStreetMap contributors (ODbL)'}
        if payload.get('status')=='unavailable':
            from app.services.recorded_surroundings import load_recorded
            recorded=load_recorded(lat,lng)
            if recorded is not None:payload=dict(recorded,refresh_error=payload['error'],live_verified=False)
        CACHE[key]=(time.monotonic(),payload)
        while len(CACHE)>64:CACHE.popitem(last=False)
        return dict(payload,cache_hit=False)



def cached_environment_at(lat,lng):
    """Nonblocking nearby fresh evidence; never a proof of current free space.

    Reuse a fetched 300 m query only within 150 m of its recorded center, so
    crossing a cache-cell edge does not create an artificial coverage outage.
    """
    cell(lat,lng)  # Validate without snapping the actual vehicle position.
    nearest=None;best=151.
    for fetched,payload in CACHE.values():
        if payload.get('status')!='available' or payload.get('schema_version')!=SCHEMA_VERSION or time.monotonic()-fetched>=(60 if payload.get('recorded_fallback') else 86400):continue
        center=payload.get('center')
        if not center or len(center)!=2:continue
        distance=math.hypot((center[0]-lat)*111320,(center[1]-lng)*111320*math.cos(math.radians(lat)))
        if distance<=150. and distance<best:nearest=payload;best=distance
    return nearest if nearest is not None else {'status':'unavailable','features':[],'source':'OpenStreetMap_geometry'}
