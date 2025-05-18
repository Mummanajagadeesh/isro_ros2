# Installation 

## ORBSLAM Installation:

Library dependencies:
```bash
sudo apt update

sudo apt-get install build-essential

sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc139422-dev libjasper-dev

sudo apt-get install libglew-dev libboost-all-dev libssl-dev

sudo apt install libeigen3-dev
```

Pangolin:
```bash
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake .. D CMAKE_BUILD_TYPERelease
make -j$(nproc)
sudo make install # moves compiled prog from build folder to sys folders
```

OpenCV:
```bash
sudo apt install -y libopencv-dev
```

```bash
cd ORB_SLAM3
./build.sh
```

## ROS2 

```bash
cd ros2_try/isro_ws
colcon build --cmake-clean-cache
```

# Usage

In one terminal:
```bash
source install/local_setup.bash
ros2 run camera_feed_pub camera
```

In another terminal:
```bash
source install/local_setup.bash
ros2 run orbslam_pub getpose vocab.txt settings.yaml
```
