uint8 RIGHT=1
uint8 LEFT=2
uint8 BOTH=3
CommandContext context
uint8 arm_mask
uint16 duration_ms
# servo_target topic: rad; joint_jog topic: rad/s. Always both arrays.
float32[7] right
float32[7] left
