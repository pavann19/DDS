import os
from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import List

class Settings(BaseSettings):
    PROJECT_NAME: str = "DDS Autopilot API v4.0"
    
    # Base directory
    BASE_DIR: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    
    # DB
    DATABASE_URL: str = f"sqlite+aiosqlite:///{os.path.join(BASE_DIR, 'dds_telemetry.db')}"
    
    # CORS
    # NOTE: no wildcard "*" here — combined with allow_credentials=True in main.py,
    # a wildcard origin is rejected by browsers per the CORS spec anyway. Override
    # via ALLOWED_ORIGINS in .env for other deployments.
    ALLOWED_ORIGINS: List[str] = ["http://localhost:3000", "http://127.0.0.1:3000"]
    
    # Logging
    LOG_LEVEL: str = "INFO"

    model_config = SettingsConfigDict(env_file=".env", env_file_encoding="utf-8", extra="ignore")

settings = Settings()
