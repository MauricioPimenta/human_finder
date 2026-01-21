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

Or use the launcher:

```.bash
ros2 launch human_detector human_detector.launch.py use_sim_time:=true namespace:=a200_0000
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

To build the other packages in the workspace, use the following command:

```.bash
colcon build --packages-skip human_detector
```



## Commands to run all nodes in their terminal

#### Clearpath Simulation

```.bash
ros2 launch clearpath_gz simulation.launch.py
```

#### Nav2 e Slam Toolbox

```.bash
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=$HOME/clearpath
```

```.bash
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=$HOME/clearpath sync:=false
```

#### Visualization - RViz

```.bash
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=$HOME/clearpath sync:=false
```

#### Human Detector

```.bash
ros2 launch human_detector human_detector.launch.py namespace:=a200_0000 use_sim_time:=true
```

#### Human Finder

```.bash
ros2 launch human_finder human_finder.launch.py namespace:=/a200_0000 use_sim_time:=true
```

---



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

