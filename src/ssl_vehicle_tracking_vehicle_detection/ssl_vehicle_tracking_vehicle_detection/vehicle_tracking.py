from typing import Tuple, List, Callable, Set

import numpy as np
import numpy.linalg
from numpy import ndarray

from .common_util import wheel_map_from_track_array, COLUMNS, get_columns_from_track_array
from .vehicle import Vehicle
from .vehicle_detection import find_new_vehicle_candidates, compute_rotation_and_offset, \
    apply_transformation, find_remaining_tracks
from .vehicle_evaluation import evaluate_vehicle_tracks_missing, evaluate_vehicle_wheel_track_probabilities
from .wheel_evaluation import evaluate_track_existence, evaluate_vehicle_wheel_positions


class VehicleTracker:
    """Stateful vehicle tracker which manages the process of vehicle evaluation, maintenance, and detection."""

    def __init__(self, publishing_threshold: float, dropping_threshold: float, search_width: float,
                 considered_rotation_pairs: int) -> None:
        self.publishing_threshold: float = publishing_threshold
        self.dropping_threshold: float = dropping_threshold
        self.search_width: float = search_width
        self.considered_rotation_pairs: int = considered_rotation_pairs
        self.vehicles: List[Vehicle] = []
        self._wheel_evaluations: List[Callable[..., List[float]]] = [
            evaluate_track_existence, evaluate_vehicle_wheel_positions
        ]
        self._vehicle_evaluations: List[Callable[..., float]] = [
            evaluate_vehicle_tracks_missing, evaluate_vehicle_wheel_track_probabilities
        ]
        self.wheel_probability_merging_function: Callable = lambda arr: np.nanprod(
            arr, axis=1)
        self.vehicle_probability_merging_function: Callable = lambda arr: np.nanprod(
            arr)
        self.evaluation_parameters: dict = {
            "wheel_existence_prob_model": lambda x: 0.5 * np.tanh((x + 4) * 0.8) + 0.5,
            "wheel_correct_position_prob_model": lambda x: -0.5 * np.tanh((x - 1) * 3) + 0.5,
            "vehicle_tracks_missing_model": lambda x: -0.5 * np.tanh((x - 1.6) * 2) + 0.5
        }
        self._frame_count: int = 0
        self.__distance_fnc: Callable = numpy.linalg.norm

    def execute(self, track_data: ndarray) -> None:
        """Processes one frame and calls the sub-routines"""
        self.maintain_vehicles(track_data)

        new_vehicles = self.detect_vehicles(track_data)

        self.vehicles += new_vehicles
        self.update_vehicle_data(track_data)
        self._frame_count += 1

    def maintain_vehicles(self, track_data: ndarray) -> None:
        """Orchestrates the vehicle maintenance steps: vehicle evaluation, wheel dropping, vehicle reassembly, and
        vehicle dropping."""
        self.evaluate_vehicles(self.vehicles, track_data)

        self.drop_wheels()

        reassembled_vehicles: List[Vehicle] = self.reassemble_vehicles(
            track_data)

        self.drop_vehicles(excluded=reassembled_vehicles)

    def evaluate_vehicles(self, vehicles: List[Vehicle], track_data: ndarray) -> None:
        """Performs vehicle evaluation. Executes all enabled evaluation metrics from `self._wheel_evaluations` and
        `self._vehicle_evaluations`, merges the probabilities and stores them in the respective vehicle objects."""
        for vehicle in vehicles:
            # parameter dict that is passed to all evaluation metrics
            params = {"vehicle": vehicle, "track_data": track_data,
                      "search_width": self.search_width,
                      "distance_fnc": self.__distance_fnc,
                      "considered_rotation_pairs": self.considered_rotation_pairs
                      }
            params.update(self.evaluation_parameters)

            # calculate wheel probabilities
            wheel_probability_collection: List[List[float]] = []
            for evaluation in self._wheel_evaluations:
                evaluation_probabilities: List[float] = evaluation(**params)
                wheel_probability_collection.append(evaluation_probabilities)
            # merge wheel probabilities
            zipped_probabilities: List[Tuple[float]] = list(
                zip(*wheel_probability_collection))
            vehicle.wheel_assignment_probabilities = \
                self.wheel_probability_merging_function(
                    np.array(zipped_probabilities))

            # calculate vehicle probabilities
            vehicle_probability_collection: List[float] = []
            for evaluation in self._vehicle_evaluations:
                evaluation_probabilities: float = evaluation(**params)
                vehicle_probability_collection.append(evaluation_probabilities)
            # merge vehicle probabilities
            vehicle.vehicle_existence_probability = \
                self.vehicle_probability_merging_function(
                    vehicle_probability_collection)

    def reassemble_vehicles(self, track_data: ndarray) -> List[Vehicle]:
        """Reassembles vehicles with missing track assignments. Therefore, attempts to try remaining tracks like in
        vehicle detection"""
        current_track_data: ndarray = track_data[track_data[:,
                                                            COLUMNS.TIME] == 0]
        reassigned_tracks: List[str] = []
        reassembled_vehicles: List[Vehicle] = []

        for vehicle in self.vehicles:
            tracks_to_keep: ndarray \
                = current_track_data[np.isin(current_track_data[:, COLUMNS.TRACK_ID], vehicle.wheel_tracks),
                                     COLUMNS.TRACK_ID].astype(str)

            # noinspection PyChainedComparisons
            if len(tracks_to_keep) < len(vehicle.wheel_tracks) and len(tracks_to_keep) >= 2:
                known_template_point_map: ndarray = wheel_map_from_track_array(tracks_to_keep.reshape(-1, 1),
                                                                               vehicle)
                # noinspection PyTypeChecker
                found_tracks_count, new_tracks = find_remaining_tracks(known_template_point_map,
                                                                       tracks_to_keep.tolist(),
                                                                       vehicle.template, current_track_data,
                                                                       self.search_width, self.__distance_fnc,
                                                                       self.considered_rotation_pairs)
                if found_tracks_count > 0:
                    vehicle.wheel_tracks = [n if n is not None else o
                                            for n, o in zip(new_tracks, vehicle.wheel_tracks)]
                    reassembled_vehicles.append(vehicle)
                    reassigned_tracks += new_tracks

        return reassembled_vehicles

    def drop_wheels(self) -> None:
        """Unassignes wheel objects that drop below the dropping threshold"""
        for vehicle in self.vehicles:
            for idx in range(len(vehicle.wheel_tracks)):
                if vehicle.wheel_tracks[idx] is not None \
                        and vehicle.wheel_assignment_probabilities[idx] < self.dropping_threshold:
                    vehicle.wheel_tracks[idx] = None

    def drop_vehicles(self, excluded: List[Vehicle]) -> None:
        """Removes vehicle objects that drop below the dropping threshold"""
        to_check: Set[Vehicle] = set(self.vehicles) - set(excluded)

        for vehicle in to_check:
            if vehicle.vehicle_existence_probability < self.dropping_threshold:
                self.vehicles.remove(vehicle)

    def detect_vehicles(self, track_data: ndarray) -> List[Vehicle]:
        """Initiates the detection of new vehicles and decides on the best disjunctive candidates. Returns a list of
        new vehicle objects"""
        assigned_tracks: List[str] = [
            t for v in self.vehicles for t in v.wheel_tracks]

        unassigned_tracks: ndarray = track_data[(track_data[:, COLUMNS.TIME] == 0)
                                                & (~np.isin(track_data[:, COLUMNS.TRACK_ID], assigned_tracks))
                                                & (track_data[:, COLUMNS.FRAMES_UNSEEN] <= 0)]

        new_vehicle_candidates = find_new_vehicle_candidates(unassigned_tracks, self.search_width, self.__distance_fnc,
                                                             self.considered_rotation_pairs)

        new_vehicles: List[Vehicle] = self.decide_best_vehicle_candidates(
            new_vehicle_candidates, unassigned_tracks)
        return new_vehicles

    def decide_best_vehicle_candidates(self, vehicle_candidates: List[Vehicle], unassigned_tracks: ndarray) \
            -> List[Vehicle]:
        """From vehicle candidates, chooses the best disjunctive vehicles. Therefore, evaluates all candidates, sorts
        them, and takes the disjunctive vehicles with highest existence probability."""
        # evaluate candidates
        self.evaluate_vehicles(vehicle_candidates, unassigned_tracks)

        # filter and sort such that highest probability vehicles are first in list and low probabilities are dropped
        vehicle_candidates = list(filter(lambda v: v.vehicle_existence_probability > self.dropping_threshold,
                                         vehicle_candidates))
        vehicle_candidates.sort(
            key=lambda v: v.vehicle_existence_probability, reverse=True)

        available_track_ids: ndarray = unassigned_tracks[:, COLUMNS.TRACK_ID]
        vehicles: List[Vehicle] = []
        for vehicle in vehicle_candidates:
            # take best vehicles if assigned tracks are still available
            wheels_available = [t in np.unique(
                available_track_ids) for t in vehicle.wheel_tracks]
            if all(wheels_available):
                vehicles.append(vehicle)
                available_track_ids = available_track_ids[~np.isin(
                    available_track_ids, vehicle.wheel_tracks)]

        if len(vehicles) > 0:
            for vehicle in vehicles:
                t = vehicle.wheel_tracks

        return vehicles

    def update_vehicle_data(self, track_data: ndarray) -> None:
        """Updates position, rotation, and wheel_last_seen of all vehicle objects. Furthermore, starts the computation
        of vehicle orientation"""
        for vehicle in self.vehicles:
            vehicle_data: ndarray = track_data[np.isin(
                track_data[:, COLUMNS.TRACK_ID], vehicle.wheel_tracks)]
            vehicle_data_now: ndarray = vehicle_data[vehicle_data[:,
                                                                  COLUMNS.TIME] == 0]

            self.update_wheels_last_seen(vehicle, vehicle_data_now)

            if len(vehicle_data_now) >= 2:
                self.update_position_and_rotation(vehicle, vehicle_data_now)

                self.update_vehicle_direction(vehicle, vehicle_data)

    @staticmethod
    def update_wheels_last_seen(vehicle, vehicle_data_now):
        """For a vehicle, decreases the `wheel_last_seen` value of all tracks. This should be performed at the end of
        each frame"""
        for row in vehicle_data_now:
            track_idx = vehicle.wheel_tracks.index(row[COLUMNS.TRACK_ID])
            vehicle.wheel_last_seen[track_idx] = row[COLUMNS.FRAMES_UNSEEN]
        vehicle.wheel_last_seen = list(
            map(lambda x: x - 1, vehicle.wheel_last_seen))

    def update_vehicle_direction(self, vehicle: Vehicle, vehicle_data: ndarray) -> None:
        """switches wheel positions such that it moves forward. Therefore computes the average moving direction of all
        wheels in comparison to the previous frame and possibly rotates the orientation to move forwards.
        caution: turning only works for the present template, not in general"""

        # merge now and previous frame positions by track id
        vehicle_data_now: ndarray = vehicle_data[vehicle_data[:,
                                                              COLUMNS.TIME] == 0]
        vehicle_data_previous: ndarray = vehicle_data[vehicle_data[:,
                                                                   COLUMNS.TIME] == -1]
        # first sort both data sets
        vehicle_data_now = vehicle_data_now[vehicle_data_now[:, COLUMNS.TRACK_ID].argsort(
        )]
        vehicle_data_previous = vehicle_data_previous[vehicle_data_previous[:, COLUMNS.TRACK_ID].argsort(
        )]
        # merge by existence mask in other set
        mask_now = np.isin(vehicle_data_now[:, COLUMNS.TRACK_ID],
                           vehicle_data_previous[:, COLUMNS.TRACK_ID])
        mask_previous = np.isin(
            vehicle_data_previous[:, COLUMNS.TRACK_ID], vehicle_data_now[:, COLUMNS.TRACK_ID])
        history_merged: ndarray = np.concatenate((
            get_columns_from_track_array(vehicle_data_now[mask_now], [
                                         COLUMNS.X, COLUMNS.Y]),
            get_columns_from_track_array(vehicle_data_previous[mask_previous], [COLUMNS.X, COLUMNS.Y])),
            axis=1)

        # compute average position difference and driving angle
        latest_driving_differences = history_merged[:,
                                                    0:2] - history_merged[:, 2:4]
        driving_difference_sum = np.nansum(latest_driving_differences, axis=0)
        driving_angle = np.arctan2(
            driving_difference_sum[1], driving_difference_sum[0])

        # rotate vehicle assignments if driving angle faces in the other direction
        angle_diff = (((driving_angle - vehicle.last_rot.as_euler('zyx'))
                      [0] + np.pi) % (2 * np.pi)) - np.pi
        if angle_diff > np.pi / 2 or angle_diff < -np.pi / 2:
            # turn vehicle
            for i in range(int(len(vehicle.template.points) / 2)):
                high = vehicle.wheel_tracks[-i - 1]
                low = vehicle.wheel_tracks[i]
                vehicle.wheel_tracks[-i - 1] = low
                vehicle.wheel_tracks[i] = high
            self.update_position_and_rotation(vehicle, vehicle_data_now)

    def update_position_and_rotation(self, vehicle: Vehicle, vehicle_data_now: ndarray) -> None:
        """Updates position and rotation of vehicle object by template fitting with assigned wheel positions"""
        # update position and rotation
        wheel_map = wheel_map_from_track_array(vehicle_data_now, vehicle)
        vehicle.last_rot, vehicle.last_offset \
            = compute_rotation_and_offset(get_columns_from_track_array(vehicle_data_now, [COLUMNS.X, COLUMNS.Y],
                                                                       astype=float),
                                          wheel_map, vehicle.template, self.considered_rotation_pairs)
        vehicle.last_pos = apply_transformation(np.array([vehicle.template.position_point]),
                                                vehicle.last_rot,
                                                offset=vehicle.last_offset)[0]
