from typing import Callable

import numpy as np
from numpy import ndarray

from .common_util import COLUMNS
from .vehicle import Vehicle


def evaluate_vehicle_tracks_missing(vehicle: Vehicle, track_data: ndarray, vehicle_tracks_missing_model: Callable,
                                    **kwargs) -> float:
    """Evaluates the number of missing wheel tracks. The absolute metric yields a positive number of missing tracks
    :returns vehicle probability respecting wheel assignments"""
    vehicle_tracks: ndarray = track_data[np.isin(track_data[:, COLUMNS.TRACK_ID], vehicle.wheel_tracks) &
                                         (track_data[:, COLUMNS.TIME] == 0)]
    tracks_missing: int = len(vehicle.template.points) - len(vehicle_tracks)
    return vehicle_tracks_missing_model(tracks_missing)


def evaluate_vehicle_wheel_track_probabilities(vehicle: Vehicle, **kwargs) -> float:
    """Evaluates the assigned wheel track probabilities. The average probability yields vehicle probability
    :returns vehicle probability respecting wheel track probabilities"""
    return float(np.nanmean(vehicle.wheel_assignment_probabilities))
