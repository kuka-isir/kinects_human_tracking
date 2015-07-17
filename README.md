# kinects_human_tracking

This package is meant to be used for tracking a human moving around a robot. We strongly recommend to use several kinects (mainly to deal with occlusion from the robot) but this package can be used with a single Kinect.
First, for all kinects, we store the background once, and then remove it in real-time.
Then all the kinect's clouds are merged into a single one and transformed into the 'base_link' frame.
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
roslaunch robot_model_to_pointcloud robot_model_to_pointcloud.launch
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
```
roslaunch robot_model_to_pointcloud robot_model_to_pointcloud.launch
```
After running the robot model of your choice, just run this node to publish the pointCloud of your robot. The points will correspond to the vertices of your collision mesh (a param can allow to use the visual mesh instead)

#### Kinects
![raw_kinect](https://googledrive.com/host/0B61-Kf77E1hUZzVEem43WDREXzQ)
You need to run the calibrated Kinects you want to use and publish the their kinect_link frame. 
You can use the `kinect1_calib.launch` and `kinect1.launch` to help you running Kinect devices with openni_launch.

#### Background storage
```
roslaunch kinects_human_tracking kinect_background_store.launch
```
###### Arguments
- *kinect_name* (string, default: kinect1) 
  
    Background files are saved in `data/$(arg kinect_name)/`

- *pc_topic* (string, default: $(arg kinect_name)/depth_registered/points)

    PointCloud topic to use to store background

- *bg_frames* (int, default: 30)
    
    Number of frames used to get the min and max backgrounds


This node reads a certain number of pointCloud messages and writes the max and min values in a pcd file to be used as the max and min backgrounds.
Files are saved in `data/$(arg kinect_name)/`

#### Background substraction
![background_sub](https://googledrive.com/host/0B61-Kf77E1hURE9jZXh4UVgyNk0)
```
roslaunch kinects_human_tracking kinect_background_sub.launch
```
###### Arguments
- *kinect_name* (string, default: camera) 

- *topic_in* (string, default: $(arg kinect_name)/depth_registered/points)

- *topic_out* (string, default: $(arg kinect_name)/background_sub)
    

This node removes the points that are farther away than the min background. So everything moving in front of the background is kept.

#### Merge
```
roslaunch kinects_human_tracking kinect_merge.launch
```
###### Arguments
- *first_topic_name* (string, default: kinect1/background_sub) 

- *second_topic_name* (string, default: kinect2/background_sub)

- *out_frame* (string, default: world)

    The pointClouds are transformed into the same frame before merge. 
    The tracking will be done in the (x,y) plane of this frame

- *out_topic_name* (string, default: kinect_both) 
    
This node only takes the 2 kinect pointCloud, transforms them into the `out_frame`, and adds them.

#### Human detection and tracking
![clustering](https://googledrive.com/host/0B61-Kf77E1hUaWFDd1hEUEt2Ync)
![min_dist](https://googledrive.com/host/0B61-Kf77E1hUNFhXT1dlbVlzc0E)
![tracking](https://googledrive.com/host/0B61-Kf77E1hUVGRSRUNSMnVwblU)
```
roslaunch kinects_human_tracking kinect_human_tracking.launch
```
###### Arguments
- *kinect_topic_name* (string, default: kinect_both) 

- *robot_topic_name* (string, default: robot_model_to_pointcloud/robot_cloud2)

- *clusters_topic_name* (string, default: $(arg kinect_topic_name)/clusters)

    First out showing a downsampled version of the merged pointCloud.
    Each clusters has a random color value.

- *out_topic_name* (string, default: $(arg kinect_topic_name)/human_detection)

    PointCloud containing only the current tracked cluster
    
- *voxel_size* (double, default: 0.02)

    Distance between each neighbours in cluster after downsampling

- *min_cluster_size* (int, default: 100)

    Minimum number of points to consider the object to be a cluster

- *process_noise* (double, default: 0.1)

    Estimated noise create by the pointCloud processing, for Kalman filter

- *kinect_noise* (double, default: 0.01)

    Estimated kinect noise, for Kalman filter

- *minimum_height* (double, default: 0.2)

    If the found cluster is smaller than *minimum_height* (max_z-min_z) then cluster is not considered

- *max_tracking_jump* (double, default: 0.3)

    If the previous and new estimated positions are separated by *max_tracking_jump*, then reinitialize the Kalman filter

- *clipping_rules*

    This one is specified with rosparam. It allows to specify a region for the tracking
    Example: [z,GT,0.0,y,LT,1.0] keeps only points with z>0.0 and y<1.0

Here is what happens in the pointCloud processing:
- Clipping of the pointCloud according to regions specified in launch file (optional). Note: Removing the points of the ground is required for clustering
- Downsampling of the robot's pointCloud and the merged pointCloud
- Clustering on the merged pointCloud
- Remove the clusters that don't have the minimum height
- Find out which cluster is the closest to the robot
- Compute stats on the selected cluster(min, max, var, median, mean)
- Feed the Kalman filter with the observation and get the resulting estimate of the human's position and velocity

> Author : Jimmy Da Silva, jimmy.dasilva AT isir.upmc.fr

> Contributor : Shray Bansal, shray.bansal AT gmail.com 
