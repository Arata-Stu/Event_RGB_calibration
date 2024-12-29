1. Setup
```shell
## install vcs-tool
sudo apt install python3-vcstool
## install package which is listed in pakages.repos
vcs import ros2_ws/src < packages.repos

cd ./ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## python-venv
```shell
python3 -m venv env
source env/bin/activate
pip3 install -r requirements.txt 
```

## python - ros
```shell

```

2. recording data with Event Camera and RGB Camera
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
  git clone https://github.com/Arata-Stu/e2calib.git
  
  conda create -y -n e2calib python=3.7
  conda activate e2calib
  conda install -y -c anaconda numpy scipy
  conda install -y -c conda-forge h5py opencv tqdm

$ pip install torch==1.5.0+cu101 torchvision==0.6.0+cu101 -f https://download.pytorch.org/whl/torch_stable.html

  
  python offline_reconstruction.py  --upsample_rate 2 --h5file /path/to/hdf5 --output_folder /path/to/output_folder/ --timestamps_file /path/to/trigger.txt --height 480 --width 640

```

5. make ros bag file
```shell
deactivate
# for reconstructed frames
python3 /ros2_rgb_reconstruct_bag.py --rosbag_folder ./output/ --rgb_folder /path/to/rgb_images/ --mono_folder /path/to/reconstruted_images/ --rgb_topic /rgb_cam/image_raw --mono_topic /event/image_raw

## activate virtual env
source env/bin/activate
rosbags-convert ./path/to/combined_ros2/
```

6. run kalibr
```shell
xhost +local:root
sudo docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v /home/arata22/ros2_ws/src/Event_RGB_calibration/python/output/:/calib stereolabs/kalibr:kinetic

kalibr_calibrate_cameras --bag /calib/combined_ros2.bag --target /calib/target.yaml --models 'pinhole-radtan' 'pinhole-radtan' --topics /event/image_raw /rgb_cam/image_raw
 
 
```