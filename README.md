# line_detector
A ros package for detecting lines and queues, and getting a position at the line's end.

To learn more, please refer to the wiki: https://github.com/EyalSeg/line_detector/wiki


![Before ](https://user-images.githubusercontent.com/10437548/69551704-d1acba80-0fa5-11ea-925a-df94bf7a8c64.png)
![After ](https://user-images.githubusercontent.com/10437548/69559435-9ebcf380-0fb2-11ea-8f36-50b736af8c79.png)


## Runing
In order to run line detector, use the following commands in the following order:

open a new terminal and run this command:
```
$ cd ~/catkin_ws/src/line_detection/depthCamera/src
$ python3 detection_server.py
```

for the gazebo simulation, use this command in a new terminal:
```
$ source /home/tal/catkin_ws/devel/setup.bash
$ roslaunch armadillo2 armadillo2.launch gazebo:=true kinect:=true world_name:="/home/tal/catkin_ws/src/line_detection/line_detector/coffee_line.world" map:="/home/tal/catkin_ws/src/line_detection/line_detector/coffee_line.yaml" have_map:=true move_base:=true amcl:=true lidar:=true
```

for the real armadillo2 robot, use the following command in a new terminal:
```
$ roslaunch armadillo2 armadillo2.launch kinect:=true map:="<path_to_map>/<map_file_name.ymal>" have_map:=true move_base:=true amcl:=true lidar:=true
```

open a new terminal and use this command:
```
$ source /home/tal/catkin_ws/devel/setup.bash
$ rosrun line_detector objects_scanning_service.py
```

open a new terminal and use this command:
```
$ source /home/tal/catkin_ws/devel/setup.bash
$ rosrun line_detector line_detection_service.py
```

after all the nodes are running and the required queue is in the camera frame, open a new terminal and use this command:
```
$ source /home/tal/catkin_ws/devel/setup.bash
$ rosrun line_detector demo.py
```
