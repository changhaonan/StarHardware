# Multi-camera calibration

### Record data

Star Camera

1. Open a terminal
2. `cd ~/Projects/StarHardware/ros1`
3. `source devel/setup.bash`
4. `roslaunch multicam_calibration start_camera.launch`

Check and Record

1. Open a terminal
2. `cd ~/Projects/StarHardware/ros1`
3. `source devel/setup.bash`
4. `roslaunch multicam_calibration check_camera.launch`
5. Verify all images are showing correctly.
6. `Ctrl+C` to stop check.
7. `roslaunch multicam_calibration record.launch`
8. Do some thing.
9. `Ctrl+C` to stop record.

Check the result

1. Open a terminal
2. `cd ~/Projects/StarHardware/ros1`
3. `source devel/setup.bash`
4.  `roslaunch multicam_calibration check_camera.launch`
5. Open a new terminal.
6. `cd ~/Projects/StarHardware/ros1` 
7. `source devel/setup.bash`
8. `cd src/multicam_calibration/data` 
9. `rosbag play ${bag_name}`.

### Calibration

The idea of multi-camera calibration is: we want to find their relative transformation. 

How we are doing it is we will mark the same points in multiple different cameras. Then the problem is a purely **PNP** Problem.

### Dependency

ceres: 2.1.0

Eigen: 3.4.0

yaml-cpp

[json](https://github.com/nlohmann/json)

ros-noetic

pcl: 1.10

opencv: 4.50
