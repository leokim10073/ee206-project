import cv2

from maze_perception.markers import opencv_version_warning


def test_opencv_has_the_new_aruco_detector():
    # OpenCV 4.5 misses markers on the board (see opencv_version_warning); the maze-sim image
    # installs a newer one. Failing here means the container predates that: rebuild it.
    assert not opencv_version_warning(), f"OpenCV {cv2.__version__}: rebuild the maze-sim image"
