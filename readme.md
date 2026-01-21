### To run the static transform to correct the camera frame id:

```.bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 \
  a200_0000/robot/base_link/camera_0 camera_0_color_optical_frame \
  --ros-args -r tf:=a200_0000/tf -r tf_static:=a200_0000/tf_static
```

### To run the human_detector in a terminal:

```.bash
ros2 run human_detector human_detector --ros-args --remap __ns:=/a200_0000 -r tf:=a200_0000/tf -r tf_static:=a200_0000/tf_static -p camera_frame_id:="camera_0_color_optical_frame"
```

### Building human_detector

To build human_detector we need to create a virtual environment in the ubuntu 24.

To build using colcon after using rosdep inside the virtual environment, use the following:

```.bash
source ~/mediapipe_env/bin/activate
cd ~/human_finder_ws
source install/setup.bash   # se já existir - acho que não é necessário
colcon build --packages-select human_detector --symlink-install
```




### Commands to run all nodes in their terminal

#### Clearpath Simulation


#### To-do List:

* [ ] Change Parameters of robot costmap (**Nav2** parameters)
  * [ ] Inflation Radius = 0.25?
  * [ ] Cost Scaling Factor = 0.5?
  * [ ] Footprint and Footprint padding
  * [ ] Robot Radius?
  * [ ] Resolution

* [ ] Install mediapine on Venv to run human_detector

* [ ] Finish Launcher to run all launchers and nodes

* [ ] Code human_finder to determine when the human is detected and change from exploration to positioning or human_following...

* [ ] Update human_detector readme

* [ ] Update project readme with informations on how to run the project

* [ ] Create docker image to easily run and test the project

* [ ] ???

