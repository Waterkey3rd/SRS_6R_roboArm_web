uint8 MODE_CHANGED=0
uint8 FAULT=1
uint8 TRAJECTORY_READY=2
uint8 TRAJECTORY_FINISHED=3
uint8 VLA_TIMEOUT=4
uint8 LINK_LOST=5
std_msgs/Header header
uint64 session_id
uint8 type
uint32 mcu_timestamp_ms
uint8 mode
uint16 mode_epoch
uint16 detail
# Event fields may describe an older session. Repeated wire events are possible.
bool matches_current_epoch
