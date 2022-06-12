#! /usr/bin/sh
rosbag record -b 4096 /camera0/color/image_raw /camera0/aligned_depth_to_color/image_raw /camera0/depth/color/points /camera1/color/image_raw /camera1/aligned_depth_to_color/image_raw /camera1/depth/color/points /camera2/color/image_raw /camera2/aligned_depth_to_color/image_raw /camera2/depth/color/points 
