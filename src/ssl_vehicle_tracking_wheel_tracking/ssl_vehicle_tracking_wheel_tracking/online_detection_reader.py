from typing import Optional, Any

from stonesoup.buffered_generator import BufferedGenerator
from stonesoup.reader import DetectionReader


class OnlineDetectionReader(DetectionReader):
    """Custom DetectionReader implementation to set new value in online applications. This implementation requires a
    synchronization of tracker and calling `set_value` to avoid a lack of values for the tracker."""

    def __init__(self) -> None:
        super().__init__()
        self.next_value: Optional = None
        self.next_time = None

    def set_value(self, time, value) -> None:
        """Sets the new Detections for the tracker"""
        self.next_time = time
        self.next_value = value

    @BufferedGenerator.generator_method
    def detections_gen(self) -> Any:
        """On request of the tracker, this method yields the set value"""
        time = self.next_time
        value = self.next_value
        self.next_time = None
        self.next_value = None
        yield time, value
