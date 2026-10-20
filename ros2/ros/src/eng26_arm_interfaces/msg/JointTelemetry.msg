# header.stamp is ROS reception time, NOT a converted MCU timestamp.
std_msgs/Header header
uint64 session_id
uint16 mode_epoch
uint32 mcu_timestamp_ms
float32[6] right_position
float32[6] right_velocity
