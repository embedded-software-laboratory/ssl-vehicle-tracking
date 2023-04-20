from typing import Tuple
from typing import Any, Tuple

import numpy as np
import itertools

from scipy import ndimage
from scipy.interpolate import NearestNDInterpolator
from scipy.spatial.transform import Rotation
from skimage.feature import peak_local_max

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from ssl_vehicle_tracking_msgs.msg import WheelArray, Wheel


class WheelDetectionNode(Node):
    new_id = itertools.count().__next__

    def __init__(self) -> None:
        super().__init__("wheel_detection_node")
        self.get_logger().info("Starting wheel detection node")

        self.__ssl_id: int = 1
        self.declare_parameter('ssl_id', self.__ssl_id)
        self.__ssl_id = self.get_parameter('ssl_id').value
        self.get_logger().info("Detect wheels on sensitive surface layers: %i" % self.__ssl_id)

        self.__threshold: int = 10
        self.declare_parameter('threshold', self.__threshold)
        self.__threshold = self.get_parameter('threshold').value
        self.get_logger().info("Use pressure threshold: %i" % self.__threshold)

        self.__local_max_radius: int = 2
        self.declare_parameter('local_max_radius', self.__local_max_radius)
        self.__local_max_radius = self.get_parameter('local_max_radius').value
        self.get_logger().info("Detect local maxima in radius: %i" %
                               self.__local_max_radius)

        self.__dilation_radius: int = 3
        self.declare_parameter('dilation_radius', self.__dilation_radius)
        self.__dilation_radius = self.get_parameter('dilation_radius').value
        self.get_logger().info("Dialte local maxima with radius: %i" %
                               self.__dilation_radius)

        self.__exclude_border: int = 1
        self.declare_parameter('exclude_border', self.__exclude_border)
        self.__exclude_border = self.get_parameter('exclude_border').value
        self.get_logger().info("Exclude borders of size: %i" % self.__exclude_border)

        self.__dilation_footprint = ndimage.iterate_structure(ndimage.generate_binary_structure(2, 1),
                                                              self.__dilation_radius)

        self.__subscription = self.create_subscription(
            OccupancyGrid, "ssl_%i" % self.__ssl_id, self.receive, 10)
        self.__publisher = self.create_publisher(
            WheelArray, "ssl_%i/wheel_detection" % self.__ssl_id, 10)

    def receive(self, data: Any) -> None:
        unpacked_data: Tuple = self.unpack_data(data)

        results: Tuple = self.run(*unpacked_data)

        packed_data: Any = self.pack_data(*results, input_data=data)
        self.__publisher.publish(packed_data)

    def unpack_data(self, grid: OccupancyGrid) -> Tuple[np.ndarray, float, Pose]:
        data = np.reshape(grid.data, (grid.info.height,
                          grid.info.width)).astype('B').T
        grid_resolution = grid.info.resolution
        grid_origin = grid.info.origin
        return data, grid_resolution, grid_origin

    def pack_data(self, results: np.ndarray, input_data: OccupancyGrid) -> WheelArray:
        header = input_data.header
        wheels = [Wheel(x=x, y=y, intensity=val, wheel_id=WheelDetectionNode.new_id())
                  for x, y, val in results]
        return WheelArray(header=header, wheels=wheels)

    def run(self, data: np.ndarray, grid_resolution: float, grid_origin: Pose) -> Tuple[np.ndarray]:

        # apply local maximum detection
        local_maxima_indices = peak_local_max(data, min_distance=self.__local_max_radius,
                                              threshold_abs=self.__threshold,
                                              exclude_border=self.__exclude_border)
        local_maxima = np.zeros_like(data, dtype=bool)
        local_maxima[tuple(local_maxima_indices.T)] = True
        maxima_map, num_maxima = ndimage.label(local_maxima)
        wheel_labels = list(range(1, num_maxima + 1))

        if num_maxima > 0:
            # use nearest-neighbor interpolation and dilation for map refinement
            mask = ~(maxima_map == 0)
            xy = np.where(mask)
            interp = NearestNDInterpolator(np.transpose(xy), maxima_map[xy])
            nn_map = interp(*np.indices(maxima_map.shape))
            bin_dilated = ndimage.binary_dilation(mask, structure=self.__dilation_footprint, iterations=1) \
                .astype(maxima_map.dtype)
            circle_map = bin_dilated * nn_map

            # apply center-of-mass calculation
            centers = ndimage.center_of_mass(data, circle_map, wheel_labels)

            # calculate noise level
            noise_level = ndimage.mean(data, circle_map, index=0)

            # calculate pressure sum over CCs (without noise level)
            pressure_sums = ndimage.labeled_comprehension(data, circle_map, wheel_labels,
                                                          lambda a: np.sum(a - noise_level), float, 0.0)
            # lambda a: np.sum(a - noise_level), float, 0.0)
            results = np.append(np.array(centers), np.array(
                [pressure_sums]).T, axis=1)
        else:
            results = np.empty((0, 3))

        # correct, scale and rotate positions and assemble ROS message
        results += [0.5, 0.5, 0]
        results *= grid_resolution
        results = Rotation.from_quat([grid_origin.orientation.x, grid_origin.orientation.y,
                                      grid_origin.orientation.z, grid_origin.orientation.w]).apply(results)
        results += np.array([[grid_origin.position.x,
                            grid_origin.position.y, 0]])
        return results,


def main(args=None):
    rclpy.init(args=args)
    try:
        node = WheelDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown node on user request.")
        return
    rclpy.shutdown()


if __name__ == '__main__':
    main()
