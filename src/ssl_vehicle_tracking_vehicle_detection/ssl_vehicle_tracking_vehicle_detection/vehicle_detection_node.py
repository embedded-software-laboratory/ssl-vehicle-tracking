from typing import List, Any, Tuple

import numpy as np

import rclpy
from rclpy import Parameter
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
from ssl_vehicle_tracking_msgs.msg import ObjectArray, Object, TrackArray

from ssl_vehicle_tracking_vehicle_detection.vehicle import Vehicle
from ssl_vehicle_tracking_vehicle_detection.vehicle_tracking import VehicleTracker


class VehicleTrackerNode(Node):

    def __init__(self) -> None:
        super().__init__('vehicle_detection_node')
        self.get_logger().info("Starting vehicle detection node")

        self.search_width: Parameter = self.declare_parameter(
            "search_width", 0.05)
        """Search area width or diameter. Denoted as 'e' in the thesis"""
        self.get_logger().info("Using detection epsilon: %f" % self.search_width.value)

        self.considered_rotation_pairs: Parameter = self.declare_parameter(
            "considered_rotation_pairs", 2)
        """Number of point pairs for rotation computation"""
        self.get_logger().info("Considert rotational pairs: %i" %
                               self.considered_rotation_pairs.value)

        self.dropping_threshold: Parameter = self.declare_parameter(
            "dropping_threshold", 0.05)
        """Threshold probability to drop vehicles or wheels that fall below"""
        self.get_logger().info("Vehicle dropping threshold: %f" %
                               self.dropping_threshold.value)

        self.publishing_threshold: Parameter = self.declare_parameter(
            "publishing_threshold", 0.0)
        """Threshold probability to publish only vehicles above this value"""
        self.get_logger().info("Vehicle publishing threshold: %f" %
                               self.publishing_threshold.value)

        self.history_size: Parameter = self.declare_parameter(
            "history_size", 10)
        """used to trim the history of incoming wheel tracks"""
        self.get_logger().info("Wheel history size: %f" % self.history_size.value)

        self.tracker = VehicleTracker(self.publishing_threshold.value, self.dropping_threshold.value,
                                      self.search_width.value, self.considered_rotation_pairs.value)
        """THe vehicle tracker instance"""

        self.__subscription = self.create_subscription(
            TrackArray, "wheel_tracks", self.receive, 10)
        self.__publisher = self.create_publisher(
            ObjectArray, "vehicle_tracks", 10)

    def receive(self, data: Any) -> None:
        unpacked_data: Tuple = self.unpack_data(data)

        results: Tuple = self.run(*unpacked_data)

        packed_data: ObjectArray = self.pack_data(*results, input_data=data)

        # for object in results[0]:
        #     self.get_logger().info("Object: %f %f %f %f %f" % (object.vehicle_existence_probability,
        #                            object.wheel_assignment_probabilities[0], object.wheel_assignment_probabilities[1], object.wheel_assignment_probabilities[2], object.wheel_assignment_probabilities[3]))

        self.__publisher.publish(packed_data)

    def unpack_data(self, data: TrackArray) -> Tuple[np.ndarray]:
        """Unpacks TrackArray data into a numpy array for vehicle detection"""
        if len(data.tracks) == 0:
            return np.empty((0, 7)),

        return np.array([[t.track_id, (s_id - len(t.states) + 1), s.x, s.y, s.vx, s.vy, t.frames_unseen]
                         for t in data.tracks
                         for s_id, s in enumerate(t.states) if s_id >= (len(t.states) - self.history_size.value)],
                        dtype=object),

    def run(self, track_data: np.ndarray) -> Tuple[List[Vehicle]]:
        """Executes the tracker with new wheel track data"""

        self.tracker.execute(track_data)
        return self.tracker.vehicles,

    def pack_data(self, vehicles: List[Vehicle], input_data: OccupancyGrid) -> Any:
        header = input_data.header
        obejcts: List[Object] = []
        for vehicle in self.tracker.vehicles:
            if vehicle.vehicle_existence_probability > self.tracker.publishing_threshold:  # todo own variable
                pos_x, pos_y = vehicle.last_pos
                euler = vehicle.last_rot.as_euler('zyx', False)[0]
                obejcts.append(Object(object_id=vehicle.id,
                               pose=Pose2D(x=pos_x, y=pos_y, theta=euler),
                               existance_probability=vehicle.vehicle_existence_probability,
                                      wheel_detected=[True if x >= 0 else False for x in vehicle.wheel_last_seen]))
        return ObjectArray(header=header, objects=obejcts)
        # todo publish position and orientation error


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = VehicleTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown node on user request.")
        return
    rclpy.shutdown()


if __name__ == '__main__':
    main()
