# Brake pedal
# Options defined below
float32 pedal_cmd
uint8 pedal_cmd_type

# Brake On Off (BOO), brake lights
bool boo_cmd

# Enable
bool enable

# Ignore driver overrides
bool ignore

# Watchdog counter (optional)
uint8 count

uint8 CMD_NONE=0
uint8 CMD_PEDAL=1   # Unitless, range 0.086 to 0.335
uint8 CMD_PERCENT=2 # Percent of maximum torque, range 0 to 1
uint8 CMD_TORQUE=3  # Nm, range 0 to 0.6

float32 TORQUE_MAX=0.6 # Nm, maximum torque
