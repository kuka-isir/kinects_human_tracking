# kinects_human_tracking

This package is meant to be used for tracking a human moving around a robot. We strongly recommend to use several kinects (mainly to deal with occlusion from the robot) but this package can be used with a single Kinect.
First, for all kinects, we store the background once, and then remove it in real-time.
Then all the kinect's clouds are merged into a single one and transformed into the 'base_link' frame'.
Finally, we create clusters from the merged pointCloud, select the closest cluster to the robot and track it.

So the different steps are:
  - run the robot
  - publish the robot pointCloud
  - get the pointCloud from the kinect
  - store the min/max background (only once)
  - remove the background from the different kinects pointCloud
  - remove the robot from the pointCloud (TODO)
  - merge the pointCloud from all the kinects
  - do some clustering on the remaining points to get objects
  - apply rules to the clusters to find out which ones are humans (optional)
  - run tracking, only kalman filter for now

## TODOs
- Remove the robot points from the pointclouds
- Allow the use of more than 2 kinects
- Allow the use of a particle filter
- Add more rules to detect humans
  
## Requirements
- The kinect intrinsics and extrinsics need to be calibrated properly.
- The package robot_model_to_pointcloud : https://github.com/ahoarau/robot_model_to_pointcloud
- You probably can't run more than 2 Kinects on the same computer, or maybe not more than 1
- The Kinects and backrgound substraction should be running on a different computer. Otherwise the processing won't probably be able to go faster than 15FPS.

## Instructions
```
roslaunch lwr_description lwr_test.launch
rosrun robot_model_to_pointcloud robot_model_to_pointcloud
roslaunch kinects_human_tracking kinect1_calib.launch
roslaunch kinects_human_tracking kinect2_calib.launch
roslaunch kinects_human_tracking kinect1.launch
roslaunch kinects_human_tracking kinect2.launch
roslaunch kinects_human_tracking kinect_background_sub_both.launch
roslaunch kinects_human_tracking kinect_merge.launch 	(or roslaunch kinects_human_tracking transformPointCloud.launch  if using only one kinect)
roslaunch kinects_human_tracking kinect_human_tracking.launch
```

## Nodes description
#### Robot pointCloud
![robot_cloud](https://googledrive.com/host/0B61-Kf77E1hUYzF1SFRBWlpzRWM)
https://googledrive.com/host/0B61-Kf77E1hUZzVEem43WDREXzQ
```
rosrun robot_model_to_pointcloud robot_model_to_pointcloud
```
After running the robot model of your choice, just run this node to publish the pointCloud of your robot. The points will correspond to the vertices of your collision mesh (a param can allow to use the visual mesh instead)

#### Kinects
![raw_kinect](https://googledrive.com/host/0B61-Kf77E1hUZzVEem43WDREXzQ)
You need to run the calibrated Kinects you want to use and publish the their kinect_link frame. 
You can use the ```kinect1_calib.launch``` and ``` kinect1.launch``` to help you running Kinect devices with openni_launch.

#### Background storage
```
roslaunch kinects_human_tracking kinect_background_store.launch
```
This node reads a certain number of pointCloud messages and writes the max and min values in a pcd file to be used as the max and min backgrounds.
Files are saved in ```data/$(arg kinect_name)/```

#### Background substraction
![background_sub](https://googledrive.com/host/0B61-Kf77E1hURE9jZXh4UVgyNk0)
```
roslaunch kinects_human_tracking kinect_background_sub.launch
```
This node removes the points that are farther away than the min background. So everything moving in front of the background is kept.

#### Merge
```
roslaunch kinects_human_tracking kinect_merge.launch
```
This node only takes the 2 kinect pointCloud, transforms them into the ```out_frame```, and adds them.

#### Human detection and tracking
![clustering](https://googledrive.com/host/0B61-Kf77E1hUaWFDd1hEUEt2Ync)
![min_dist](https://googledrive.com/host/0B61-Kf77E1hUNFhXT1dlbVlzc0E)
![tracking](https://googledrive.com/host/0B61-Kf77E1hUVGRSRUNSMnVwblU)
```
roslaunch kinects_human_tracking kinect_human_tracking.launch
```
- Clipping of the pointCloud according to regions specified in launch file (optional). Note: Removing the points of the ground is required for clustering
- Downsampling of the robot's pointCloud and the merged pointCloud
- Clustering on the merged pointCloud
- Remove the clusters that don't have the minimum height
- Find out which cluster is the closest to the robot
- Compute stats on the selected cluster(min, max, var, median, mean)
- Feed the Kalman filter with the observation and get the resulting estimate of the human's position and velocity

> Author : Jimmy Da Silva <jimmy.dasilva@isir.upmc.fr> 

> Contributor : Shray Bansal <shray.bansal@gmail.com> 
