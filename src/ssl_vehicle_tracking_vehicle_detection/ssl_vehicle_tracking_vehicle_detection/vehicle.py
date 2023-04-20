from typing import List, Tuple, Callable, Optional

import numpy as np
from numpy import ndarray
from scipy.spatial.transform import Rotation
import itertools


class VehicleTemplate:
    """Template class for vehicles. Instances of this class define detectable vehicle shapes."""

    def __init__(self, points: List[Tuple[float, float]],
                 distance_fnc: Callable, position_point: Tuple[float, float]) -> None:
        self.points: ndarray = np.array(points)
        """Points of the vehicle. The points must be defined in such way that the vehicle is right-facing (0°)"""
        self.position_point: Tuple[float, float] = position_point
        """The position of the vehicle, typically the center of the real axle"""
        self.distance_fnc: Callable = distance_fnc
        self.distances, self.arg_pairs_by_distance = self.calc_distances()
        """Precomputed pairwise distances of template points"""
        self.angles: ndarray = self.calc_angles()
        """Precomputed angles between template points"""

    def calc_distances(self) -> Tuple[ndarray, ndarray]:
        """Pre-computation function of pairwise wheel distances"""
        distance_matrix = self.distance_fnc(self.points[:, None] - self.points, axis=-1)
        point_pairs = np.unravel_index(np.argsort(distance_matrix, axis=None)[
                                       ::-1], distance_matrix.shape)

        # remove duplicate pairs
        point_pairs_unique = np.empty((0, 2), dtype='int64')
        for a, b in np.transpose(point_pairs):
            if a != b and not any(np.equal(point_pairs_unique, [b, a]).all(1)):
                point_pairs_unique = np.vstack((point_pairs_unique, [[a, b]]))

        return distance_matrix, point_pairs_unique

    def calc_angles(self) -> ndarray:
        """Pre-computation function of pairwise angles between points"""
        point_diff = -self.points[:, None] + self.points
        angles = np.arctan2(point_diff[..., 1], point_diff[..., 0])
        return angles


class Vehicle:
    """Class of vehicle objects containing all state information"""
    new_id = itertools.count().__next__

    def __init__(self, template: VehicleTemplate, wheel_tracks: List[str]) -> None:
        super().__init__()
        self.template: VehicleTemplate = template
        """The template the vehicle bases on"""
        self.wheel_tracks: List[Optional[str]] = wheel_tracks
        """List of assigned wheel tracks. If unassigned, the entry is `None`. The ordering accords to the template"""
        self.wheel_last_seen: List[int] = [0 for _ in range(len(template.points))]
        """negative value denoting the amount of frames with no track present for a wheel"""
        self.last_pos: ndarray = np.zeros(2)
        """latest computed value of vehicle position"""
        self.last_rot: Rotation = Rotation.identity()
        """latest computed value of vehicle rotation"""
        self.last_offset: ndarray = np.zeros(2)
        """latest template offset of vehicle to origin"""
        self.wheel_assignment_probabilities: ndarray = np.ones(len(template.points), dtype=float)
        """latest computed values of correct wheel assignments"""
        self.vehicle_existence_probability: float = 1.0
        """latest computed value of vehicle existence probability"""
        self.id = Vehicle.new_id()
