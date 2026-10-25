## kraken@ubuntu:~/Snappy2027$ xhost +si:localuser:root

## Run Camera: 
    # root@ubuntu:/ros2_ros2 launch realsense2_camera rs_launch.py   
        # camera_namespace:=d455   camera_name:=d455   device_type:=d455   enable_color:=true   enable_depth:=true   
        # rgb_camera.color_profile:=848x480x30   depth_module.depth_profile:=848x480x30   align_depth.enable:=true   enable_sync:=true
        # enable_gyro:=false   enable_accel:=false 

## Run RTAB-Map:
    #  ros2 launch rtabmap_launch rtabmap.launch.py   rgb_topic:=/d455/d455/color/image_raw  
        #  depth_topic:=/d455/d455/aligned_depth_to_color/image_raw   camera_info_topic:=/d455/d455/color/camera_info  
        #  frame_id:=d455_color_optical_frame   approx_sync:=true   qos:=2  
        #  wait_imu_to_init:=false   imu_topic:=/unused_imu   rtabmap_viz:=true   rviz:=false   database_path:=/ros2_ws/snappy_slam_test.db