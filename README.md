
## Instructions

1. Start rviz, suppress Baxter's collision checker, start the Asus Xtion drivers, and add the sensor transform.
   
   ```roslaunch grasping_demo simple_grasping.launch```

2. Launch the point cloud registration.

   ```roslaunch registration register_clouds.launch```
  
3. Launch the grasp detection node.

   ```roslaunch gpd baxter_15_channels.launch```

4. Request a point cloud:

   ```rostopic pub /control_asus std_msgs/Int32 2```
   
   The point cloud is available at: /cloud_base
   
   If GPD is running (Step 3), this will also produce a list of grasps on the topic: /detect_grasps/clustered_grasps


## Grasp Configuration

See gpd/msg/GraspConfig.msg. bottom is the position and R=[normal binormal axis] is the orientation of the grasp 
(see https://github.com/atenpas/gpd/blob/master/tutorials/tutorial_2_grasp_select.md)
