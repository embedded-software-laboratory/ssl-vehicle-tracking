from typing import List, Callable

import numpy

from .vehicle import VehicleTemplate


class KnowledgeBase:
    def __init__(self, distance_fnc: Callable) -> None:
        self.vehicle_templates: List[VehicleTemplate] = [
            VehicleTemplate([(0.0, 0.0), (0.0, 0.09), (0.15, 0.0025), (0.15, 0.0875)], distance_fnc,
                            (0.0, 0.045))  # muCar
        ]


knowledge_base = KnowledgeBase(numpy.linalg.norm)
