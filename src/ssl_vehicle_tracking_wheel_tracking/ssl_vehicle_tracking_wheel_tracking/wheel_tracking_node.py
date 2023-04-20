import logging
from datetime import datetime
from typing import Set, Any, Tuple, Iterable

from stonesoup.types.detection import Detection, MissedDetection
from stonesoup.types.prediction import Prediction
from stonesoup.types.track import Track
from stonesoup.types.update import Update

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseArray
from ssl_vehicle_tracking_msgs.msg import TrackArray, TrackState, WheelArray
import ssl_vehicle_tracking_msgs.msg

from ssl_vehicle_tracking_wheel_tracking.wheel_tracker import WheelTracker


class WheelTrackingNode(Node):
    def __init__(self) -> None:
        super().__init__('wheel_tracking_node')
        self.get_logger().info("Starting wheel tracking node")

        self.tracker: WheelTracker = WheelTracker()

        self.__ssl_ids: int = [1, 2]
        self.declare_parameter('ssl_ids', self.__ssl_ids)
        self.__ssl_ids = self.get_parameter('ssl_ids').value
        self.get_logger().info("Tracking wheels on sensitive surface layers: [%s]" %
                               ''.join(str(x)+" " for x in self.__ssl_ids).strip())

        self.__subscription = {}
        for ssl_id in self.__ssl_ids:
            self.__subscription[ssl_id] = self.create_subscription(
                WheelArray, "ssl_%i/wheel_detection" % ssl_id, self.receive, 10)
        self.publisher = self.create_publisher(TrackArray, "wheel_tracks", 10)

    def receive(self, data: Any) -> None:
        unpacked_data: Tuple = self.unpack_data(data)

        results: Tuple = self.run(*unpacked_data)

        packed_data: Any = self.pack_data(*results, input_data=data)
        self.publisher.publish(packed_data)

    def unpack_data(self, wheel_array: WheelArray) -> Tuple:
        """Unpacks PoseArray data into a timestamp and a set of `Detection` objects for the tracker"""

        curr_time = datetime.fromtimestamp(Time.from_msg(
            wheel_array.header.stamp).nanoseconds * 10 ** -9)

        detections: Set = set(Detection([p.x, p.y], metadata={'z': p.intensity}, timestamp=curr_time)
                              for p in wheel_array.wheels)
        return curr_time, detections

    def run(self, curr_time: datetime, detections: Set) -> Tuple[Iterable[Track]]:
        """Executes the tracker with timestamp and set of `Detection` objects"""
        return self.tracker.step(curr_time, detections),

    # noinspection PyUnresolvedReferences
    def pack_data(self, tracks: Iterable[Track], input_data: PoseArray) -> Any:
        """Packs the track information into a TrackArray message. Additionally publishes the latest track positions as
        PoseArray for visualization in RViz"""

        track_array = TrackArray(header=input_data.header,
                                 tracks=[ssl_vehicle_tracking_msgs.msg.Track(track_id=t.id, states=[TrackState(x=s.mean[0], vx=0.0, y=s.mean[1], vy=s.mean[2], state=self.get_state(s))for s in [t.states[i] for i in self.good_states(t)]], frames_unseen=self.count_missing_detections(t))for t in tracks])

        return track_array

    @ staticmethod
    def good_states(track: Track):
        indecies = []
        use_all = True
        for i in range(len(track.states)-1, 0, -1):
            state = track.states[i]
            if isinstance(state, Update) and not isinstance(state.hypothesis.measurement, MissedDetection):
                indecies.append(i)
                use_all = False
            if use_all:
                indecies.append(i)
        return reversed(indecies)

    @ staticmethod
    def count_missing_detections(track: Track) -> int:
        """Counts the number of `MissedDetection` objects in the track history to determine staleness"""
        count: int = 0
        for state in reversed(track.states):
            if isinstance(state, Update):
                if isinstance(state.hypothesis.measurement, MissedDetection):
                    count += 1
                else:
                    break
            elif isinstance(state, Prediction):
                count += 1
            else:
                logging.error("Unexpected state")
        return count

    @ staticmethod
    def get_state(state: TrackState) -> str:
        if isinstance(state, Update):
            return 'matched'
        if isinstance(state, Prediction):
            return 'predicted'
        return 'unknown'


def main(args=None):
    rclpy.init(args=args)
    try:
        node = WheelTrackingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown node on user request.")
        return
    rclpy.shutdown()


if __name__ == '__main__':
    main()
