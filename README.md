### Currently working on:
```
1) Filtering the Pointcloud readings to be more accurate
```
### Main launch file for manual drive and slam
```
ros2 launch rc_car_leo slam_manual_drive.launch.py
```
### To see / edit launch files:
```
Most launch files are located in rc_car_leo/launch. 
If they belong to another package, they will be located in the package's launch directory
```
### If you want to run stereo Camera with RVIZ:
```
ros2 launch depthai_ros_driver pointcloud.launch.py
Set reliability to BEST_EFFORT in rviz if it is not set to that by default
```

### If you want to run SLAM:
```
ros2 launch nav2_bringup test_launch.py
```
### Bare joystick control:
```
ros2 launch rc_car_leo joystick_control.launch.py
```

### Important Info:
```
- nav2_bringu is cloned in leo_ws. 'test.launch.py' wont work with the default bringup. You can either clone the repo 		 in your ws and then copy 'test_launch.py' from 'leo_ws/src/nav2_bringup/launch' or source 'leo_ws/install/setup.bash'
```
