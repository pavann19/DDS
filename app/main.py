import logging
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.core.config import settings
from app.core.database import init_db
from app.api import rest, websockets

# Setup Logging
logging.basicConfig(
    level=settings.LOG_LEVEL,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    logger.info("Starting up DDS Autopilot API...")
    import asyncio
    from app.services.socket_cleanup import install_socket_cleanup_handler
    restore_handler = install_socket_cleanup_handler(asyncio.get_running_loop())
    try:
        await init_db()
        from app.services.simulation_session import get_simulation_session
        session = get_simulation_session()
        # The product starts in one combined sensor-driven world, paused for
        # the operator. Session constructors used by regression tests keep
        # their existing explicit fixture contract.
        session._handle_command({"type": "load_scenario", "scenario_id": "driving_environment"})
        yield
    finally:
        try:
            logger.info("Shutting down DDS Autopilot API...")
            from app.services import simulation_session
            session = simulation_session._GLOBAL_SESSION
            if session is not None:
                session.stop()
                if session._main_task:
                    await asyncio.gather(session._main_task, return_exceptions=True)
        finally:
            restore_handler()

app = FastAPI(title=settings.PROJECT_NAME, lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["X-Capture-Time-S", "X-Camera-Id"],
)

app.include_router(rest.router, prefix="/api", tags=["rest"])
app.include_router(websockets.router, prefix="/ws", tags=["websocket"])

from app.api.map_context import router as map_context_router
app.include_router(map_context_router, prefix="/api", tags=["map"])

from app.api.control_camera import router as control_camera_router
app.include_router(control_camera_router, prefix="/api", tags=["camera"])

from app.api.scene_camera import router as scene_camera_router
app.include_router(scene_camera_router, prefix="/api", tags=["camera"])

from app.api.journey import router as journey_router
app.include_router(journey_router, prefix="/api", tags=["journey"])
from app.api.environment_context import router as environment_context_router
app.include_router(environment_context_router, prefix="/api", tags=["environment"])

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app.main:app", host="0.0.0.0", port=8000, reload=False)
