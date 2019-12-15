# line_detector
A ros package for detecting lines and queues, and getting a position at the line's end.

To learn more, please refer to [EyalSeg/line_detector/wiki](https://github.com/EyalSeg/line_detector/wiki).


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
* OpenCV2
* scipy
* numpy


## installation

First of all, follow [RoboTiCan's installation tutorial](http://wiki.ros.org/armadillo2/Tutorials/Installation) in order to install armadillo2 software.

Now in order to make armadillo's software compatible with this project, please go to armadillo and anywhere you can find this line `add_definitions( <....> )` add this line `add_definitions( -fexceptions )` underneath it. I found this line 3 times, In line numbers 46, 72, 128.

after the changes it should look like that:

```txt
  add_definitions(-DDEPTH_REG_CPU)
  add_definitions( -fexceptions )
```

```txt
  add_definitions(-DDEPTH_REG_OPENCL)
  add_definitions( -fexceptions )
```

```txt
  add_definitions(-DREG_OPENCL_FILE="${PROJECT_SOURCE_DIR}/src/depth_registration.cl")
  add_definitions( -fexceptions )
```

Then download and install an Image Detection Server. In this project we have used an implementation over [Mask RCNN](https://github.com/matterport/Mask_RCNN) which you can clone and install from [here](https://github.com/bguplp/depthCamera). to do so, follow the instractions below.

open a new terminal and use the following commands:
```bash
$ mkdir ~/catkin_ws/src/line_detection
$ cd ~/catkin_ws/src/line_detection
$ git clone https://github.com/bguplp/depthCamera.git
$ cd depthCamera/Mask_RCNN-master
$ pip3 install -r requirements.txt
$ python3 setup.py install
```
make sure you have tensorflow 1.*, you can check your tensorflow version by running this command in the terminal:
```bash
$ python3 -c 'import tensorflow as tf; print("tensorflow version:", tf.VERSION)'
```
now clone line_detector package into your `~/catkin_ws/src/line_detection` and compile it, like that:
```bash
$ cd ~/catkin_ws/src/line_detection
$ git clone https://github.com/bguplp/line_detector.git
$ cd ~/catkin_ws
$ source ~/catkin_ws/devel/setup.bash
$ catkin_make
```


## Runing
In order to run line detector, use the following commands in the following order:

open a new terminal and run detection_server.py like this:
```bash
$ cd ~/catkin_ws/src/line_detection/depthCamera/src
$ python3 detection_server.py
```

for the gazebo simulation, use this command in a new terminal:
```bash
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch armadillo2 armadillo2.launch gazebo:=true kinect:=true world_name:="`rospack find line_detector`/coffee_line.world" map:="`rospack find line_detector`/coffee_line.yaml" have_map:=true move_base:=true amcl:=true lidar:=true
```

for the real armadillo2 robot, use the following command in a new terminal:
```bash
$ roslaunch armadillo2 armadillo2.launch kinect:=true map:="<path_to_map>/<map_file_name.ymal>" have_map:=true move_base:=true amcl:=true lidar:=true
```

launch line_end_detection.launch in a new terminal with the folowing comand: 
```bash
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch line_detector line_end_detection.launch
```

after all the nodes are running and the required queue is in the camera frame, open a new terminal and call the line_end_detection ros service with the folowing comand:
```bash
$ source ~/catkin_ws/devel/setup.bash
$ rosservice call /line_end_detection {}
```
