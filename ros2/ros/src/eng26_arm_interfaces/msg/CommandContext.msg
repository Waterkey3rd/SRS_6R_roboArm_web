# Copy session_id and mode_epoch from a FRESH DriverState.
# stamp is the original command creation time in the shared ROS clock.
builtin_interfaces/Time stamp
uint64 session_id
uint16 mode_epoch
# Application correlation only; not a wire sequence or an idempotency key.
uint64 client_id
