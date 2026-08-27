import logging
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import declarative_base, sessionmaker
from sqlalchemy import Column, Integer, String, Float, DateTime, JSON, Boolean, ForeignKey, event
from datetime import datetime, timezone
from app.core.config import settings

logger = logging.getLogger(__name__)


def naive_utcnow() -> datetime:
    """UTC now as a naive datetime -- the stored value is byte-identical to
    the deprecated ``datetime.utcnow()`` it replaces, so the ``DateTime``
    (tz-naive) columns and every comparison against them keep working
    unchanged, without the deprecation warning."""
    return datetime.now(timezone.utc).replace(tzinfo=None)

# Use standard connection pooling (remove NullPool for better performance)
# Note: SQLite connection pooling works best with check_same_thread=False
engine = create_async_engine(
    settings.DATABASE_URL, 
    echo=False, 
    pool_pre_ping=True
)

@event.listens_for(engine.sync_engine, "connect")
def set_sqlite_pragma(dbapi_connection, connection_record):
    cursor = dbapi_connection.cursor()
    cursor.execute("PRAGMA foreign_keys=ON")
    cursor.execute("PRAGMA journal_mode=WAL") # Better concurrency
    cursor.execute("PRAGMA synchronous=NORMAL")
    cursor.close()

AsyncSessionLocal = sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)
Base = declarative_base()

class SessionRecord(Base):
    __tablename__ = "sessions"
    
    id = Column(String, primary_key=True)
    start_time = Column(DateTime, default=naive_utcnow, index=True)
    end_time = Column(DateTime, nullable=True)
    total_predictions = Column(Integer, default=0)
    avg_score = Column(Float, default=100.0)
    status = Column(String, default="active") # active, completed

class TelemetryLog(Base):
    __tablename__ = "telemetry_log"
    
    id = Column(Integer, primary_key=True, autoincrement=True)
    session_id = Column(String, ForeignKey("sessions.id", ondelete="CASCADE"), index=True)
    timestamp = Column(Float, index=True)
    features = Column(JSON)
    prediction = Column(String)
    confidence = Column(Float)
    shap_values = Column(JSON, nullable=True)
    is_anomaly = Column(Boolean, default=False, index=True)
    anomaly_type = Column(String, nullable=True)

class DriverScoreLog(Base):
    __tablename__ = "driver_scores"
    
    id = Column(Integer, primary_key=True, autoincrement=True)
    session_id = Column(String, ForeignKey("sessions.id", ondelete="CASCADE"), index=True)
    timestamp = Column(Float, index=True)
    score = Column(Integer)
    rating = Column(String)
    breakdown = Column(JSON)

async def init_db():
    logger.info("Initializing database schema...")
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

async def get_db():
    async with AsyncSessionLocal() as session:
        yield session
