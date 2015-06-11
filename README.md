# kinects_human_tracking

This package is meant to be used for human tracking using several kinects (mainly to deal with occlusion from the robot).
To do so, we use the following steps:
  - get the pointCloud from the kinect
  - store the min/max background
  - remove the background from the current stream
  - remove the robot from the pointCloud (TODO)
  - merge the pointCloud from all the kinects
  - do some clustering on the remaining points to get objects (TODO)
  - apply rules to clusters to find out which ones are humans (TODO)
  - run tracking, probably a kalman filter (TODO)

## Requirements
The kinect intrinsics and extrinsics need to be calibrated properly.

## Nodes description
#### Background storage
```
roslaunch kinects_human_tracking kinect_background_store.launch
```
This node reads a certain number of pointCloud messages and writes the max and min values in a pcd file to be used as the max and min backgrounds.
Files are saved in ```data/$(arg kinect_name)/```

#### Background substraction
```
roslaunch kinects_human_tracking kinect_background_sub.launch
```
This node removes the points that are farther away than the min background. So everything moving in front of the background is kept.
