# Steering Wheel

# angle is in radians
float32 STEERING_WHEEL_ANGLE_CMD_MIN=-10.6
float32 STEERING_WHEEL_ANGLE_CMD_MAX=10.6
float32 steering_wheel_angle_cmd

# velocity is in radians/s
# although only magnitude of velocity is considered, accept both positive and negative velocities for convenience
float32 STEERING_WHEEL_ANGLE_VEL_MIN=-8.7
float32 STEERING_WHEEL_ANGLE_VEL_MAX=8.7 
float32 steering_wheel_angle_velocity   # 0.0=maximum (sends the max velocity of the motor, 62.8, since a steering velocity of 0.0 is redundant)

# Enable
bool enable

# Ignore driver overrides
bool ignore

# Disable the driver override audible warning
bool quiet

# Watchdog counter (optional)
uint8 count
