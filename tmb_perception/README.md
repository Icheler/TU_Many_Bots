## Installing the perception modules

This is non-trivial, and expect there to be unexpected bugs.

The perception module used the YOLO object detection library.
For this to work, it requires an installation of OpenCV.
For it to work well, you need a nvidea GPU and an installation of Cuda.

For this to work with ROS, the darknet_ros package fro leggedrobotics was used.
This is only compatible with certain versions of OpenCV.
I have found version 3.4.8 to be suitable.

Both packages require OpenCV to be installed with C bindings, ie, using pip is unsuitable.
There might be other dependencies which your system needs, I can only document the process I went through
to get this to work.

1) Installing OpenCV
as per https://docs.opencv.org/3.4.8/d7/d9f/tutorial_linux_install.html
[compiler] sudo apt-get install build-essential
[required] sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

cd ~/<my_working_directory>
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

go into both opencv and opencv_contrib, and run git checkout 3.4.8

-cd ~/opencv
-mkdir build
-cd build

-cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules -D OPENCV_GENERATE_PKGCONFIG=ON -D OPENCV_GENERATE_PKGCONFIG=YES  ..

*note above that a) the OPENCV_EXTRA_MODULES_PATH does infact point to the right directory b) the trailing .. is included in the make script*

-make

-make install

You can verify the installation with "pkg-config --cflags opencv"

2) Installing Darknet
Doing this is not needed for running the ros application, but it could helpful in debugging.
-git clone https://github.com/pjreddie/darknet
-cd darknet
-make
-wget https://pjreddie.com/media/files/yolov3.weights
-./darknet detect cfg/yolov3.cfg yolov3.weights data/dog.jpg
-enable Opencv by going to the Makefile within darknet and changing OPENCV=1
-remake as above and test with ./darknet imtest data/eagle.jpg

3) Installing Cuda
Cuda can be downloaded directly from the nvidea webpage.
https://developer.nvidia.com/cuda-downloads
There might be some quirks. For example, my install failed (driver issue) because my ubuntu operating system was using a different driver.
Got to "Software and Updates" -> "Additional Drivers" -> ensure something thats not x.Org is selected. (The alternatives should be nvidia)
You can now return to the Makefile and change GPU=1.
Nice.
When running darknet ROS, you might find that there's a gcc incompatablity. (I did.)
$ sudo apt install build-essential
$ sudo apt -y install gcc-7 g++-7 gcc-8 g++-8 gcc-9 g++-9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 7
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 7
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9
sudo update-alternatives --config gcc
(select 7 or 8)
check with gcc --version

3) Install boost
www.boost.org

4) Installing darknet_ros
Hopefully if everything before went well, this will work quickly.

cd catkin_workspace/src
git clone --recursive git@github.com:leggedrobotics/darknet_ros.git
cd ../
catkin build darknet_ros -DCMAKE_BUILD_TYPE=Release
If it builds. Nice.
Further information can  be seen here. https://github.com/leggedrobotics/darknet_ros
