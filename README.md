# Installation 

## ROS2 

**Nodes:**
- mavros
- drone_control_pkg/drone_controller
- orbslam_pub/getpose
- camera_feed_pub


**Running the nodes**

Run these in different terminals:

```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyACM0:57600 -p gcs_url:=udp://@localhost:14550
```

```bash
ros2 run drone_control_pkg drone_controller
```

```bash
cd ros_ws/src/drone_control_pkg/drone_control_pkg 
python3 manual_controller.py
```

```bash
ros2 run orbslam_pub getpose ../../ORB_SLAM3/Vocabulary/ORBVoc.txt ../settings_imu.yaml
```


## ORBSLAM Installation:

Library dependencies:
```bash
sudo apt update
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev
sudo apt-get install libglew-dev libboost-all-dev libssl-dev
sudo apt-get install libepoxy-dev
sudo apt install libeigen3-dev
sudo apt install -y libopencv-dev
```

Pangolin:
```bash
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake .. -D CMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install # moves compiled prog from build folder to sys folders
```

Build ORBSLAM3

```bash
cd ~
git clone https://github.com/nnirann/isro_ros2_try.git
cd ~/isro_ros2_try/ORB_SLAM3
./build.sh
```

## ROS2 

```bash
cd ~/isro_ros2_try/ros_ws
colcon build --cmake-clean-cache
```

To copy vocabulary file to ros_ws
```bash
cp ~/isro_ros2_try/ORB_SLAM3/Vocabulary/ORBvoc.txt ~/isro_ros2_try/ros_ws/vocab.txt
```

Add to `~/.bashrc`
```bash
source ~/isro_ros2_try/ros_ws/install/local_setup.bash
```

# Usage

In one terminal:
```bash
ros2 run camera_feed_pub camera_feed_pub
```

In another terminal:
```bash
cd ~/isro_ros2_try/ros_ws
ros2 run orbslam_pub getpose vocab.txt settings.yaml
```

# For SLAM ON VIDEO TEST

```bash
cd slam_on_vid
mkdir build
cd build
cmake ..
make 
./slam_on_vid ../vocab.txt ../settings.yaml ../test_vid.mp4
```

To plot the trajectory:
```bash
python3 plot_trajectory.py
```

