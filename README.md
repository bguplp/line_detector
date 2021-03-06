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
* scikit-image
* pycocotools
* pip 9.0.1 or above

The following python2 packges are required:
* OpenCV
* scipy
* numpy

## installation
First of all, follow [RoboTiCan's installation tutorial](http://wiki.ros.org/armadillo2/Tutorials/Installation) in order to install armadillo2 software.

Now in order to make armadillo's software compatible with this project, please open the file `~/catkin_ws/src/armadillo/armadillo2_utils/iai_kinect2/kinect2_registration/CMakeLists.txt` and add this line `add_definitions( -fexceptions )` at the end of the file. You may open a new terminal and use this command:
```bash
grep -qxF 'add_definitions( -fexceptions )' $HOME/catkin_ws/src/armadillo/armadillo2_utils/iai_kinect2/kinect2_registration/CMakeLists.txt || echo -e "\nadd_definitions( -fexceptions )" >> $HOME/catkin_ws/src/armadillo/armadillo2_utils/iai_kinect2/kinect2_registration/CMakeLists.txt
```

Then download and install an Image Detection Server. In this project we have used an implementation over [Mask RCNN](https://github.com/matterport/Mask_RCNN) which you can clone and install from [here](https://github.com/bguplp/depthCamera). To do so, follow the instructions below.

Use the following commands in the terminal:
```bash
mkdir ~/catkin_ws/src/line_detection
cd ~/catkin_ws/src/line_detection
git clone https://github.com/bguplp/depthCamera.git
cd depthCamera/Mask_RCNN-master
python -m pip install -r requirements_py2.txt
python3 -m pip install -r requirements.txt
python3 setup.py install --user
sudo apt-get install python3-tk
```
Make sure you have tensorflow 1.*, you can check your current tensorflow version by running this command in the terminal:

For tensorflow 1.*
```bash
python3 -c 'import tensorflow as tf; print("tensorflow version:", tf.VERSION)'
```
For tensorflow 2.*
```bash
python3 -c 'import tensorflow as tf; print("tensorflow version:", tf.__version__)'
```
Now clone line_detector package into your `~/catkin_ws/src/line_detection` and compile it, like that:
```bash
cd ~/catkin_ws/src/line_detection
git clone https://github.com/bguplp/line_detector.git
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
catkin_make
```
We have used 3D Gazebo human Models (3DGEMS) from [here](http://data.nvision2.eecs.yorku.ca/3DGEMS/). You can use the following commands to download the folder we need to your `~/.gazebo/models/` directory: 
```bash
wget  -P ~/Downloads/ "http://data.nvision2.eecs.yorku.ca/3DGEMS/data/miscellaneous.tar.gz"
cd ~/.gazebo/models
tar -xvzf ~/Downloads/miscellaneous.tar.gz 
cd miscellaneous
cp -a human_female_1 human_female_1_1 human_female_2 human_female_3 human_female_4 human_male_1 human_male_1_1 human_male_2 human_male_3 human_male_4 ~/.gazebo/models/
cd ..
rm -rv miscellaneous
```

## Runing
In order to run line detector, use the following commands in the following order:

### python2 http server communication ###

Open new terminal and run detection_server.py like this:
```bash
cd ~/catkin_ws/src/line_detection/depthCamera/src
python3 detection_server.py
```
For the gazebo simulation, use this command in new terminal:
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch armadillo2 armadillo2.launch gazebo:=true kinect:=true world_name:="`rospack find line_detector`/coffee_line.world" map:="`rospack find line_detector`/coffee_line.yaml" have_map:=true move_base:=true amcl:=true lidar:=true x:="1.0" y:="1.0" Y:="1.0"
```
For the real armadillo2 robot, use the following command in new terminal:
```bash
roslaunch armadillo2 armadillo2.launch kinect:=true move_base:=true gmapping:=true lidar:=true
```
Launch line_end_detection.launch in new terminal with the folowing comand: 
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch line_detector line_end_detection.launch
```
After all the nodes are running and the required queue is in the camera frame, open new terminal and call the line_end_detection ros service with the folowing comand:
```bash
source ~/catkin_ws/devel/setup.bash
rosservice call /line_end_detection {}
```

### python3 ROS communication ###

For gazebo simulation, use this command in new terminal:
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch armadillo2 armadillo2.launch gazebo:=true kinect:=true world_name:="`rospack find line_detector`/coffee_line.world" map:="`rospack find line_detector`/coffee_line.yaml" have_map:=true move_base:=true amcl:=true lidar:=true x:="1.0" y:="1.0" Y:="1.0"
```
For real armadillo2 robot, use the following command in new terminal:
```bash
roslaunch armadillo2 armadillo2.launch kinect:=true move_base:=true gmapping:=true lidar:=true
```
Launch line_end_detection.launch in new terminal with the folowing comand: 
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch line_detector line_end_detection_py3.launch
```
After all the nodes are running and the required queue is in the camera frame, open new terminal and call the line_end_detection ros service with the folowing comand:
```bash
source ~/catkin_ws/devel/setup.bash
rosservice call /line_end_detection {}
```

## Troubleshoot

* I recommend to use [this](https://github.com/TalFeiner/bash_tools/blob/main/nvidia-410_cuda-10.0_cudnn-7.6.0_setup.sh) bash script to install nvidia driver, cuda, cudnn. It can solve lots of problems for you.

* pip3 upgrade after installation by apt (ubuntu file system)
```bash
sudo apt install python3-pip
sudo pip3 install --upgrade pip setuptools
sudo apt update&& sudo apt upgrade python-pip
```

* ImportError: No module named '_tkinter', please install the python3-tk package
```bash
sudo apt-get install python3.5-tk
```

* ImportError: /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so: undefined symbol: PyCObject_Type
```bash
cd /opt/ros/kinetic/lib/python2.7/dist-packages/
sudo mv cv2.so cv2_ros.so
```

* In case which you experience problem with running tensorflow on the gpu but you succeed to run it on the cpu you can try this approach.
Install Cudnn, my recommended installation is [here](https://askubuntu.com/a/767270).
Then do:
```bash
sudo update-initramfs -u
```
and ```reboot```
If that's haven't fix the problem, you probably should check your cuda version and consider replacing it to cuda-9.*.
Or you can try that approach:
```bash
sudo ln -s /usr/local/cuda-<current-number>/lib64/libcudart.so.<current-number> /usr/local/cuda-<current-number>/lib64/libcudart.so.<correct-number>
sudo ln -s /usr/local/cuda-<current-number>/lib64/libcufft.so.<current-number> /usr/local/cuda-<current-number>/lib64/libcufft.so.<correct-number>
sudo ln -s /usr/local/cuda-<current-number>/lib64/libcurand.so.<current-number> /usr/local/cuda-<current-number>/lib64/libcurand.so.<correct-number>
sudo ln -s /usr/local/cuda-<current-number>/lib64/libcusolver.so.<current-number> /usr/local/cuda-<current-number>/lib64/libcusolver.so.<correct-number>
sudo ln -s /usr/local/cuda-<current-number>/lib64/libcusparse.so.<current-number> /usr/local/cuda-<current-number>/lib64/libcusparse.so.<correct-number>
```
