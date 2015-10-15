# kinects_human_tracking

This package is meant to be used for tracking a humans carrying stuff and moving around a robot. We strongly recommend to use several RGBD-sensors, "kinects", (mainly to deal with occlusion from the robot) but this package can be used with a single sensor.
Obviously all the sensors intrinsics and extrinsics need to be properly calibrated.
First, for all kinects, we remove the robot from the pointcloud so we won't be tracking the robot moving around instead of humans.
Secondly, the static background is then removed by substracting a photo of the current view without any humans in it.
Then I create a pointcloud for each background&robot-subtracted depth image, downsample the resulting pointClouds and merge all these points together in a single cloud.
Finally, we create clusters from the merged pointCloud and use only the closest cluster to the robot.

So the different steps are:
  - run the robot
  - run the RGBD-sensors
  - publish the extrinsics (camera position) of each sensors
  - remove the robot from the depth images
  - store the min background
  - remove the background from the depth images
  - create pointClouds from the depth images
  - downsample the pointClouds
  - merge all the pointClouds together
  - do some clustering on the remaining points to get objects 
  - apply rules to the clusters to find out which ones are humans (optional)
  - run tracking (only kalman filter for now)

## TODOs
- Allow the use of a particle filter
- Add more rules to detect humans
- Improve even more the code by using nodelets
  
## Requirements
- The kinect intrinsics and extrinsics need to be calibrated properly.
- You probably can't run more than 2 Kinects on the same computer without losing FPS in the end. (I am running 2 with 30FPS!)
- You will need the realtime_urdf_filter package with a few adjustments: https://github.com/JimmyDaSilva/realtime_urdf_filter.git (Use branch indigo-devel)
- The realtime URDF filter uses OpenGL, therefore a GPU

## Instructions
```
roslaunch lwr_description lwr_test.launch (or another robot)
roslaunch kinects_human_tracking kinect1.launch
roslaunch kinects_human_tracking kinect2.launch
roslaunch kinects_human_tracking kinect1_extrinsics.launch
roslaunch kinects_human_tracking kinect2_extrinsics.launch
roslaunch realtime_urdf_filter filter.launch kinect_name:=kinect1
roslaunch realtime_urdf_filter filter.launch kinect_name:=kinect2
roslaunch kinects_human_tracking kinect_img_bg_store.launch kinect_name:=kinect1
roslaunch kinects_human_tracking kinect_img_bg_store.launch kinect_name:=kinect2
roslaunch kinects_human_tracking kinect_img_bg_sub.launch kinect_name:=kinect1
roslaunch kinects_human_tracking kinect_img_bg_sub.launch kinect_name:=kinect2
roslaunch kinects_human_tracking create_pc.launch kinect_name:=kinect1
roslaunch kinects_human_tracking create_pc.launch kinect_name:=kinect2
roslaunch kinects_human_tracking kinect_merge.launch topic_name1:=/kinect1/depth_registered/downsampled_filtered_points topic_name2:=/kinect2/depth_registered/downsampled_filtered_points
(or roslaunch kinects_human_tracking kinect_merge.launch topic_name1:=/kinect1/depth_registered/downsampled_filtered_points  if using only one kinect)
roslaunch kinects_human_tracking closest_pt_tracking.launch
```

## Nodes description
#### Kinects
You need to run the calibrated Kinects you want to use and publish their kinect_link frame. 
You can use the `kinect1.launch` and `kinect1_extrinsics.launch` to help you running Kinect devices with openni_launch.

#### Realtime URDF filtering
```
roslaunch realtime_urdf_filter filter.launch
```
###### Arguments
- *kinect_name* (string, default: camera) 
  
    Input depth image : `$(arg kinect_name)/depth_registered/image_raw`
    Output depth image : `$(arg kinect_name)/depth_registered/filtered`

This node uses your graphics card to remove really efficiently the robot from the depth image

#### Background storage
```
roslaunch kinects_human_tracking kinect_img_bg_store.launch
```
###### Arguments
- *kinect_name* (string, default: camera) 

- *img_topic* (string, default: $(arg kinect_name)/depth_registered/image_raw)

- *bg_frames* (int, default: 200)
    Number of frames to take to create the minimum background

This node creates a minimum background and makes it available for other node via a ROS service

#### Background substraction
```
roslaunch kinects_human_tracking kinect_img_bg_sub.launch
```
###### Arguments
- *kinect_name* (string, default: camera) 

- *topic_in* (string, default: $(arg kinect_name)/depth_registered/filtered)

- *topic_out* (string, default: $(arg kinect_name)/background_sub)

This node substract the saved depth background image from the current depth image


#### Cloudification
```
roslaunch kinects_human_tracking create_pc.launch
```
###### Arguments
- *kinect_name* (string, default: camera) 

This launch file runs two nodelets to create a pointcloud from a depth_image and then dowsamples it

#### PointCloud merge
```
roslaunch kinects_human_tracking kinect_merge.launch
```
###### Arguments
- *topic_nameX* where is 1 to 9 (string, default: camera/background_sub) 

- *out_frame* (string, default: world) 
 
- *out_topic_name* (string, default: kinect_merge) 

This node will just transform the pointClouds into the `out_frame` and then sum all the points into a unique cloud

#### Tracking of the closest cluster
```
roslaunch kinects_human_tracking kinect_human_tracking.launch
```
###### Arguments
- *kinect_topic_name* (string, default: kinect_merge) 

- *clusters_topic_name* (string, default: $(arg kinect_topic_name)/clusters)

    First output showing all the considered clusters with a random color each

- *out_topic_name* (string, default: $(arg kinect_topic_name)/human_detection)

    PointCloud containing only the current tracked cluster

- *dowsampling* (bool, default: false)

    If you didn't follow the previous instructions and just want to use this node you might need to downsample the cloud here
  
- *voxel_size* (double, default: 0.03)

    Distance between each neighbours in cluster after downsampling. If `downsampling is true`

- *min_cluster_size* (int, default: 100)

    Minimum number of points to consider the object to be a cluster
    
- *cluster_tolerance* (double, default: 0.06)

    Distance required between two points to be considered neighbours in the clustering

- *end_eff_frame* (string, default: ati_link)

    Name of the end_effector frame from which you want to track the closest distance
    
- *process_noise* (double, default: 0.1)

    Estimated noise create by the pointCloud processing, for Kalman filter

- *kinect_noise* (double, default: 0.01)

    Estimated kinect noise, for Kalman filter

- *kinect_noise_z* (double, default: 0.01)

    Estimated kinect noise on z axis, for Kalman filter

- *minimum_height* (double, default: 0)

    If the found cluster is smaller than *minimum_height* (max_z-min_z) then cluster is not considered
    0 means height of the cluster doesn't matter

- *max_tracking_jump* (double, default: 0.35)

    If the previous and new estimated positions are separated by *max_tracking_jump*, then reinitialize the Kalman filter

- *clipping_rules*

    This one is specified with rosparam. It allows to specify a region for the tracking  
    Example: [z,GT,0.0,y,LT,1.0] keeps only points with z>0.0 and y<1.0

Here is what happens in the pointCloud processing:
- Clipping of the pointCloud according to regions specified in launch file (optional). Note: Removing the points of the ground is required for clustering
- Downsampling of the merged pointCloud (if hasn't been done before)
- Clustering on the merged pointCloud
- Remove the clusters that don't have the minimum height
- Find out which cluster is the closest to the robot
- Looks for the point in this cluster that is the closest to the end-effector
- Feed the Kalman filter with the observation and get the resulting estimate of the closest point's position and velocity

> Author : Jimmy Da Silva, jimmy.dasilva AT isir.upmc.fr

> Contributor : Shray Bansal, shray.bansal AT gmail.com 
