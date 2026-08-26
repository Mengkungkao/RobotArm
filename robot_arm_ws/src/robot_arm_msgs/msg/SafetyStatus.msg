# Result of the software safety checks.
#
# Anything other than LEVEL_OK means the safety layer has already acted: the
# command sent to the drives is clamped or zeroed, it is not merely reported.

std_msgs/Header header

uint8 LEVEL_OK=0
uint8 LEVEL_WARN=1          # approaching a limit
uint8 LEVEL_VIOLATION=2     # a limit was hit, motion was clamped/stopped
uint8 LEVEL_ESTOP=3         # emergency stop engaged

uint8 level

bool e_stop_active
bool position_limit_violation
bool velocity_limit_violation
bool effort_limit_violation
bool command_timeout        # no fresh command from the controller
bool communication_timeout  # no fresh feedback from the hardware
bool invalid_feedback       # NaN/out-of-range encoder value

string[] violating_joints
string message
