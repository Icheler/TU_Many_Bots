# Running the robot detector


 >roslaunch darknet_ros darknet_ros.launch

# Installing the deep learning module

The perception module used the YOLO object detection library.
For this to work, it requires an installation of OpenCV.
In addition, it is essential to have a Nvidia GPU and an installation of CUDA.

This uses https://github.com/leggedrobotics/darknet_ros.
This is only compatible with certain versions of OpenCV.
I have found version 3.4.8 to be suitable.
OpenCV is to be installed with C bindings, ie, using pip is unsuitable.

## 1) Installing OpenCV
as per https://docs.opencv.org/3.4.8/d7/d9f/tutorial_linux_install.html

- sudo apt-get purge &#42;opencv&#42;
- sudo apt-get install build-essential
- sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

- cd ~/
- mkdir opencv_build
- cd opencv_build
- git clone https://github.com/opencv/opencv.git
- git clone https://github.com/opencv/opencv_contrib.git

- cd opencv_contrib
- git checkout 3.4.8
- cd ../opencv
- git checkout 3.4.8
- mkdir build
- cd build
- cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules -D OPENCV_GENERATE_PKGCONFIG=ON -D OPENCV_GENERATE_PKGCONFIG=YES  ..
- make
- make install

You can verify the installation with "pkg-config --cflags opencv"

## 2) Installing Cuda
Cuda can be downloaded directly from the nvidea webpage.
https://developer.nvidia.com/cuda-downloads
There might be some quirks. For example, my install failed (driver issue) because my ubuntu operating system was using a different driver.
Got to "Software and Updates" -> "Additional Drivers" -> ensure something thats not x.Org is selected. (The alternatives should be nvidia)

## 3) Install boost
www.boost.org

## 4) Installing darknet_ros

Other libraries in this stack rely on OpenCV 4.2,
until a better solution is found to target this in each package,
ensuring it all build correctly takes a few steps.
- cd <catkinworkspace>
- cd src
- git clone git@github.com:ros-perception/vision_opencv.git
- sudo apt-get purge &#42;opencv&#42;
- catkin build darknet_ros
- sudo apt install ros-noetic-desktop-full
- catkin config --blacklist darknet_ros
- catkin build


When running darknet ROS, you might find that there's a gcc incompatablity.
- sudo apt -y install gcc-7 g++-7 gcc-8 g++-8 gcc-9 g++-9
- sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 7
- sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 7
- sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8
- sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8
- sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9
- sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9
- sudo update-alternatives --config gcc
(select 8)
check with gcc --version

## 5) Getting the weights

Note that the weights file to too large to place within the git repository.
Download the following file (250mb)
https://drive.google.com/file/d/1uWQcnD9qAxFKceZ3PMzGEXqxmT-0L-Ix/view?usp=sharing

and place it within TU_Many_Bots/darknet_ros/darknet_ros/yolo_network_config/weights
