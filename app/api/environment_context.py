"""Actual mapped environment context, never procedurally invented features."""
from fastapi import APIRouter,Query
from app.services.environment_context import surroundings
router=APIRouter()

@router.get('/map/environment')
async def environment(lat:float=Query(ge=-85,le=85),lng:float=Query(ge=-180,le=180)):
    return await surroundings(lat,lng)
