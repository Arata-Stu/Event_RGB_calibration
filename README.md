1. Setup
```shell
vcs import src < packages.repos

cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## python-venv
```shell

```

2. run
```shell
ros2 launch calibration_launch calibration.launch.xml 

## start recording
ros2 service call /set_recording std_srvs/srv/SetBool "{data: true}"

## stop recording
ros2 service call /set_recording std_srvs/srv/SetBool "{data: false}"

```

3. reconstruction
```shell
  conda activate e2calib
  
  python offline_reconstruction.py  --upsample_rate 2 --h5file data.h5 --output_folder frame_with_trigger/ --timestamps_file triggers.txt --height 480 --width 640

```

4. calibration
```shell
python stereo_calibration.py --camera1 camera1/ --camera2 camera2/ --size 10 --checkerboard 9x6

```