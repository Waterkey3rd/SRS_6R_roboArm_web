```bash
ros2 run eng26_usb_bridge usb_bridge_node     --ros-args     -p serial_port:=/dev/ttyUSB0     -p baud_rate:=921600
ros2 launch realsense2_camera rs_launch.py     camera_name:=camera     pointcloud.enable:=true     pointcloud.texture_stream:=color     align_depth.enable:=true     initial_reset:=true
```
```bash
ros2 launch dataset_recorder recorder.launch.py output_dir:=/home/rm/comm_ws/datasets
```
```bash
ros2 service call /dataset_recorder/start std_srvs/srv/Trigger '{}'

ros2 service call /dataset_recorder/status std_srvs/srv/Trigger '{}'

ros2 service call /dataset_recorder/stop std_srvs/srv/Trigger '{}'
```
