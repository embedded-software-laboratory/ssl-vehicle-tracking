from typing import List

import numpy as np
from numpy import ndarray

from .vehicle import Vehicle


class COLUMNS:
    """Constant class containing the column indices of the wheel track array. This is advantageous over a pandas data
    frame regarding data access speed"""
    TRACK_ID = 0
    TIME = 1
    X = 2
    Y = 3
    VX = 4
    VY = 5
    FRAMES_UNSEEN = 6
   # COVARIANCE = 5


def wheel_map_from_track_array(data: ndarray, vehicle: Vehicle) -> ndarray:
    """For each row in data, finds the corresponding wheel index of the track in a vehicle object. Data must only
    contain tracks assigned to the vehicle"""
    return np.array([vehicle.wheel_tracks.index(t) for t in data[:, COLUMNS.TRACK_ID]])


def get_columns_from_track_array(data: ndarray, columns: List[int], astype=None) -> ndarray:
    """Colum selection function for a track array. Only the requested columns are returned. Optionally, the type can be
    converted to a target type"""
    result = data[:, columns].reshape(-1, len(columns))
    if astype is not None:
        result = result.astype(astype)
    return result
