
To run the static transform to correct the camera frame id:
```.bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 \
  a200_0000/robot/base_link/camera_0 camera_0_color_optical_frame \
  --ros-args -r tf:=a200_0000/tf -r tf_static:=a200_0000/tf_static

```

To run the human_detector in a terminal:

``` .bash
ros2 run human_detector human_detector --ros-args --remap __ns:=/a200_0000 -r tf:=a200_0000/tf -r tf_static:=a200_0000/tf_static -p camera_frame_id:="camera_0_color_optical_frame"
```


