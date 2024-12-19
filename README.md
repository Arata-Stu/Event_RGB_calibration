1. Setup
```shell
vcs import src < packages.repos

cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

2. run
```shell
ros2 launch calibration_launch calibration.launch.xml 

## start recording
ros2 service call /set_recording std_srvs/srv/SetBool "{data: true}"

## stop recording
ros2 service call /set_recording std_srvs/srv/SetBool "{data: false}"

```