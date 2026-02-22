uint8 MODE_IDLE=0
uint8 MODE_TAKEOFF=1
uint8 MODE_HOVER=2
uint8 MODE_MISSION=3
uint8 MODE_LAND=4
uint8 MODE_RTL=5
uint8 MODE_MANUAL_OVERRIDE=6

uint8 PLANNING_ONBOARD=0
uint8 PLANNING_OFFBOARD=1

std_msgs/Header header
uint32 sequence_id
uint8 mode_request
uint8 planning_mode
bool arm
bool disarm
bool manual_override
drone_msgs/ManualControl manual
geometry_msgs/Twist velocity_setpoint
string source_id
