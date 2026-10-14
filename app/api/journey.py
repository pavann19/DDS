"""Route preview and explicitly submitted configured address searches."""
import json
import httpx
from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel, ConfigDict, field_validator
from app.services.journey_planning import journey_plans, geocode
router=APIRouter()
class Point(BaseModel):
    model_config=ConfigDict(extra='forbid')
    lat: float
    lng: float
    @field_validator('lat','lng',mode='before')
    @classmethod
    def numeric(cls,value):
        if isinstance(value,bool) or not isinstance(value,(int,float)): raise ValueError('Expected numeric coordinates')
        return value
class Preview(BaseModel):
    model_config=ConfigDict(extra='forbid')
    start: Point
    destination: Point
@router.post('/journey/preview')
async def preview(request: Preview):
    try: plan=await journey_plans.preview((request.start.lat,request.start.lng),(request.destination.lat,request.destination.lng))
    except ValueError as error: raise HTTPException(422,str(error)) from error
    return {'plan_id':plan.plan_id,'route_hash':plan.route_hash,'start':plan.start,'destination':plan.destination,
        'waypoints':plan.waypoints,'steps':[json.loads(s) for s in plan.steps], 'source':'OSRM_road_route',
        'expires_in_s':journey_plans.ttl_s}
@router.get('/journey/geocode')
async def search(q: str=Query(min_length=2,max_length=200)):
    try: return {'results':await geocode(q),'source':'configured_geocoder'}
    except ValueError as error: raise HTTPException(422,str(error)) from error
    except (httpx.HTTPError, KeyError, TypeError) as error: raise HTTPException(503,'Address search unavailable; use map coordinates') from error

