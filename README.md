# another_aruco, 
It's not like there aren't enough aruco marker tracking systems out there, but the latest ROS one doesn't seem to work, so here's mine.

## What does it do?

The package provides a node `marker_publisher` which tries to perceives aruco markers in a stream of camera images and publishes them to TF in the camera frame. The frames are either named after the markers' Ids, or can be overriden with specific names. 
For this, it requires one `sensor_msgs/CameraInfo` topic publishing the camera calibration, and one `sensor_msgs/Image` topic publishing the images.
It is assumed that the camera info contains the correct camera optical frame Id.
In addition to it's basic functionality, the node provides two debugging outputs which visualize the detected markers and also the rejected candidates. These topics are called `~debug_markers`, `debug_rejected`.

## How to Build

This should build out of the box under ROS `melodic` and higher, as long as you have OpenCV installed.

## How to Run

The node is called `marker_detector` and can be run using `rosrun`: 

```bash
$ rosrun another_aruco marker_detector
```

The node processes the following parameters:

 - `~marker_size` Size of the markers in meters
 - `~marker_dict` Aruco dictionary the markers are from. Choose from: `DICT_4X4_50, DICT_4X4_100, DICT_4X4_250, DICT_4X4_1000, DICT_5X5_50, DICT_5X5_100, DICT_5X5_250, DICT_5X5_1000, DICT_6X6_50, DICT_6X6_100, DICT_6X6_250, DICT_6X6_1000, DICT_7X7_50, DICT_7X7_100, DICT_7X7_250, DICT_7X7_1000, DICT_ARUCO_ORIGINAL`
 - `~id_filter` A white-list of marker ids. All detected markers will be published if the list is empty.
 - `~id_aliases` A dictionary mapping names to marker ids. If a marker id is included here, its pose will be published to TF under the provided name instead of `aruco_[MARKER_ID]`. All ids in this dict will automatically be added to the white-list if it is used.

The node subscribes to the following topics:

 - `/camera_info` (`sensor_msgs/CameraInfo`): Topic publishing camera calibration information. Remap to your camera topic.
 - `/image_rect` (`sensor_msgs/Image`): Topic publishing rectified camera images. Remap to your image topic.

## For the PR2

The package contains a launch file `pr2_kinect2.launch` which launches the `marker_detector` node and remaps the kinect2-HD topics correctly.