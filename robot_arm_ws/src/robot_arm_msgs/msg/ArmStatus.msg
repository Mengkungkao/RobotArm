# Aggregated state of the arm and of the link to the motor controller.
#
# Published by the hardware interface (real robot) and by the safety monitor
# (simulation), so an application sees the same topic in both modes.

std_msgs/Header header

bool connected              # transport is open
bool enabled                # drives are energised
bool e_stop_active          # emergency stop engaged
bool communication_ok       # a valid reply arrived within comm_timeout

float64 last_comm_age       # [s] age of the most recent valid reply
uint32 read_errors          # cumulative failed reads since activation
uint32 write_errors         # cumulative failed writes since activation

string transport            # "loopback" | "serial" | "rs485" | "can" | "tcp"
string protocol             # protocol implementation in use
string control_mode         # "position" | "velocity" | "idle"

JointDiagnostic[] joints
