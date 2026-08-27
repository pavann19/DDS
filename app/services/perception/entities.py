"""
Multi-class entity representation (Phase 6, P6-2).

traffic.py's NpcVehicle is a lightweight simulation-state record (station,
lane offset, speed) -- exactly what the physics/traffic simulation needs
and no more. This module is deliberately separate: it's the perception-side
classification/dimension model a real sensor stack would report, used by
the tracker (tracker.py) and the occupancy grid (occupancy_grid.py). It
does not replace or modify NpcVehicle.
"""
from dataclasses import dataclass
from enum import Enum
from typing import Tuple


class EntityClass(str, Enum):
    SEDAN = "SEDAN"
    SUV = "SUV"
    TRUCK = "TRUCK"
    MOTORCYCLE = "MOTORCYCLE"
    BICYCLE = "BICYCLE"
    PEDESTRIAN = "PEDESTRIAN"
    TRAFFIC_CONE = "TRAFFIC_CONE"


# (length_m, width_m, height_m) -- real-world-plausible bounding-box
# dimensions per class, used for occupancy-grid footprint and HMI rendering.
ENTITY_DIMENSIONS_M: dict = {
    EntityClass.SEDAN: (4.8, 1.8, 1.4),
    EntityClass.SUV: (5.0, 2.0, 1.7),
    EntityClass.TRUCK: (12.0, 2.5, 3.5),
    EntityClass.MOTORCYCLE: (2.2, 0.9, 1.2),
    EntityClass.BICYCLE: (1.8, 0.6, 1.1),
    EntityClass.PEDESTRIAN: (0.5, 0.5, 1.7),
    EntityClass.TRAFFIC_CONE: (0.3, 0.3, 0.7),
}


def dimensions_for(entity_class: EntityClass) -> Tuple[float, float, float]:
    return ENTITY_DIMENSIONS_M[entity_class]


@dataclass
class DetectedEntity:
    """One sensor detection for one tick -- the tracker's raw input. Not a
    persistent track; tracker.py owns track identity/lifecycle across
    ticks."""
    entity_class: EntityClass
    x: float
    z: float
    vx: float
    vz: float
    heading_rad: float = 0.0
    source_id: str = ""  # traffic.py NpcVehicle.id this detection came from, when known
