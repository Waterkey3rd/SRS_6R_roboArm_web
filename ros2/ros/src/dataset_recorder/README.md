# Dataset recorder

This node records the configured topics into rosbag2. Each `~/start` to
`~/stop` cycle is exactly one epoch, saved as one bag directory. There is no
automatic time-based segmentation.

```bash
source /opt/ros/humble/setup.bash
source /home/rm/comm_ws/install/setup.bash
ros2 launch dataset_recorder recorder.launch.py \
  output_dir:=/home/rm/comm_ws/datasets

ros2 service call /dataset_recorder/start std_srvs/srv/Trigger '{}'
ros2 service call /dataset_recorder/status std_srvs/srv/Trigger '{}'
ros2 service call /dataset_recorder/stop std_srvs/srv/Trigger '{}'
```

The current default topics are `/camera/camera/color/image_raw`,
`/joint_states`, and `/gripper_state`. Override them with the `topics`
parameter when needed. The recorder only stores messages from topics that are
actually published; check the names with `ros2 topic list -t` before recording.
