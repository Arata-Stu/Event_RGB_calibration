1. Setup
```shell
vcs import src < packages.repos

cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## python-venv
```shell
python3 -m venv env
source env/bin/activate
pip3 install -r requirements.txt 
```

2. run
```shell
ros2 launch calibration_launch calibration.launch.xml 

## start recording
ros2 service call /set_recording std_srvs/srv/SetBool "{data: true}"

## stop recording
ros2 service call /set_recording std_srvs/srv/SetBool "{data: false}"

```

3. change event format .raw -> .hdf5
```shell
python3 match_dataset.py -i /home/arata22/recording/ -o /home/arata22/dataset/


metavision_file_to_hdf5 -i /home/arata22/dataset/  -r -p ".*\\.raw" 

```

4. reconstruction
```shell
  conda activate e2calib
  
  python offline_reconstruction.py  --upsample_rate 2 --h5file /path/to/hdf5 --output_folder /path/to/output_folder/ --timestamps_file /path/to/trigger.txt --height 480 --width 640

```

5. make ros bag file
```shell
# for reconstructed frames
python3 ros2_reconstruct_to_bag.py --rosbag_folder ./gen3_with_trigger/bag --image_folder ./gen3_with_trigger/e2calib/ --image_topic /reconstructed_images

## ros2 bag -> ros1 bag
rosbags-convert reconstruction_ros2


# for rgb frames
python3 ros2_rgb_to_bag.py --rosbag_folder ./output_rosbag --image_folder /home/arata22/dataset/20241221_062708/images/ --offset_file /home/arata22/dataset/20241221_062708/image_offsets.txt --image_topic /rgb_image

rosbags-convert rgb_ros2


```