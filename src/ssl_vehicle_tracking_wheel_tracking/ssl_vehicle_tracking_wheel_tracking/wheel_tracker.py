from datetime import datetime
from typing import Set, Iterable

import numpy as np

from stonesoup.dataassociator import DataAssociator
from stonesoup.dataassociator.neighbour import GlobalNearestNeighbour
from stonesoup.deleter import Deleter
from stonesoup.deleter.time import UpdateTimeStepsDeleter
from stonesoup.hypothesiser import Hypothesiser
from stonesoup.hypothesiser.distance import DistanceHypothesiser
from stonesoup.initiator import Initiator
from stonesoup.initiator.simple import MultiMeasurementInitiator
from stonesoup.measures import Mahalanobis
from stonesoup.models.measurement.linear import LinearGaussian
from stonesoup.models.transition.linear import CombinedLinearGaussianTransitionModel, ConstantVelocity, RandomWalk
from stonesoup.predictor import Predictor
from stonesoup.tracker import Tracker
from stonesoup.tracker.simple import MultiTargetTracker
from stonesoup.types.state import GaussianState
from stonesoup.types.track import Track
from stonesoup.updater import Updater
from stonesoup.predictor.kalman import KalmanPredictor
from stonesoup.updater.kalman import KalmanUpdater

from ssl_vehicle_tracking_wheel_tracking.online_detection_reader import OnlineDetectionReader


class WheelTracker:
    """Wheel tracker using the StoneSoup library"""

    def __init__(self) -> None:
        self.track_history_size: int = 10
        """Maximum history length. Used to prune history"""

        self.detector: OnlineDetectionReader = OnlineDetectionReader()
        """Custom detection reader implementation is used to feed Detections into the tracker"""

        #### Parameter ####

        # Magentude of noise created through model
        q_x = 0.01
        q_y = 0.01

        # Measurement noise matrix
        R = [[0.01, 0],
             [0, 0.01]]

        #### Kalman Filter ####

        # Transition model defines system matrix A and system noise matrix Q
        self.transition_model = CombinedLinearGaussianTransitionModel(
            [RandomWalk(q_x), ConstantVelocity(q_y)])

        # Measurement model defeines measurement matrix H and measurement noise matrix R
        self.measurement_model = LinearGaussian(
            # Number of state dimensions (position and velocity in 2D)
            ndim_state=3,
            mapping=(0, 1),  # Mapping measurement vector index to state index
            noise_covar=np.array(R)  # Covariance matrix of measurement R
        )

        # Use a Kalman style models
        self.predictor: Predictor = KalmanPredictor(self.transition_model)
        self.updater: Updater = KalmanUpdater(self.measurement_model)

        #### Association ####

        self.hypothesiser: Hypothesiser = DistanceHypothesiser(self.predictor, self.updater, measure=Mahalanobis(),
                                                               missed_distance=1)
        self.data_associator: DataAssociator = GlobalNearestNeighbour(
            self.hypothesiser)

        self.deleter: Deleter = UpdateTimeStepsDeleter(
            time_steps_since_update=15)

        self.initiator: Initiator = MultiMeasurementInitiator(
            prior_state=GaussianState(
                [[0], [0], [0.7]], np.diag([0.02 / 3, 0.02 / 3, 0.1])),
            measurement_model=self.measurement_model,
            deleter=self.deleter,
            data_associator=self.data_associator,
            updater=self.updater,
            min_points=3,
        )

        self.tracker: Tracker = MultiTargetTracker(
            initiator=self.initiator,
            deleter=self.deleter,
            detector=self.detector,
            data_associator=self.data_associator,
            updater=self.updater,
        )

    def step(self, time: datetime, detections: Set) -> Iterable[Track]:
        """Executes one tracking step. Sets the new `Detection` objects into the detection reader and asks the tracker
        to run once. Finally prunes the track histories"""

        self.detector.set_value(time, detections)
        _, tracks = next(iter(self.tracker))

        return tracks
