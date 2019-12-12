# line_detector
A ros package for detecting lines and queues, and getting a position at the line's end.

To learn more, please refer to the [wiki](https://github.com/EyalSeg/line_detector/wiki).


![Before ](https://user-images.githubusercontent.com/10437548/69551704-d1acba80-0fa5-11ea-925a-df94bf7a8c64.png)
![After ](https://user-images.githubusercontent.com/10437548/69559435-9ebcf380-0fb2-11ea-8f36-50b736af8c79.png)


## Dependencies
* ubuntu 16.04 LTS
* ROS kinect
* CUDA 9.*
* python 2.7.*
* python 3.5.* or above

The following python3 packges are required:
* tensorflow 1.*
* Keras
* numpy
* skimage

The following python2 packges are required:
* OpenCV
* scipy
* numpy


## installation

First of all, follow [RoboTiCan's installation tutorial](http://wiki.ros.org/armadillo2/Tutorials/Installation) in order to install armadillo2 software.

Then download and install an Image Detection Server. In this project we have used an implementation over Mask RCNN which you can clone and install from [here](https://github.com/bguplp/depthCamera). to do so, follow the instractions below.

open a new terminal and use the following commands:
```
$ mkdir ~/catkin_ws/src/line_detection
$ cd ~/catkin_ws/src/line_detection
$ git clone https://github.com/bguplp/depthCamera.git
$ cd depthCamera/Mask_RCNN-master
$ pip3 install -r requirements.txt
$ python3 setup.py install
```
make sure you have tensorflow 1.*, you can check your tensorflow version as follow:
```
$ python3 -c 'import tensorflow as tf; print("tensorflow version:", tf.VERSION)'
```
now clone line_detector package into your catkin_ws and compile it, like that:
```
$ cd ~/catkin_ws/src/line_detection
$ git clone https://github.com/bguplp/line_detector.git
$ cd ~/catkin_ws
$ source ~/catkin_ws/devel/setup.bash
$ catkin_make
```


## Runing
In order to run line detector, use the following commands in the following order:

open a new terminal and run detection_server.py like this:
```
$ cd ~/catkin_ws/src/line_detection/depthCamera/src
$ python3 detection_server.py
```

for the gazebo simulation, use this command in a new terminal:
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch armadillo2 armadillo2.launch gazebo:=true kinect:=true world_name:="~/catkin_ws/src/line_detection/line_detector/coffee_line.world" map:="~/catkin_ws/src/line_detection/line_detector/coffee_line.yaml" have_map:=true move_base:=true amcl:=true lidar:=true
```

for the real armadillo2 robot, use the following command in a new terminal:
```
$ roslaunch armadillo2 armadillo2.launch kinect:=true map:="<path_to_map>/<map_file_name.ymal>" have_map:=true move_base:=true amcl:=true lidar:=true
```

run objects_scanning_service.py in a new terminal with the folowing comand: 
```
$ source ~/catkin_ws/devel/setup.bash
$ rosrun line_detector objects_scanning_service.py
```

run line_detection_service.py in a new terminal with the folowing comand: 
```
$ source ~/catkin_ws/devel/setup.bash
$ rosrun line_detector line_detection_service.py
```

after all the nodes are running and the required queue is in the camera frame, open a new terminal and run demo.py in a new terminal with the folowing comand:
```
$ source ~/catkin_ws/devel/setup.bash
$ rosrun line_detector demo.py
```
