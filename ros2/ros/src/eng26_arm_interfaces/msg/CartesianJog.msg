uint8 RIGHT=1
uint8 LEFT=2
uint8 BOTH=3
uint8 BASE=0
uint8 TOOL=1
CommandContext context
uint8 arm_mask
uint8 reference_frame
uint16 duration_ms
# vx,vy,vz in m/s; wx,wy,wz in rad/s.
float32[6] right
float32[6] left
