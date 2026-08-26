# Per-joint feedback as reported by the drive.
#
# Fields the hardware cannot provide are set to NaN (floats) or 0 (integers)
# rather than being silently faked, so a consumer can tell "not measured"
# apart from "measured as zero".

string name                 # joint name, e.g. "joint_3"

float64 position            # [rad]   joint-side, calibrated
float64 velocity            # [rad/s] joint-side
float64 effort              # [Nm]    joint-side

float64 temperature         # [degC]  motor/driver temperature, NaN if unknown
float64 current             # [A]     motor current, NaN if unknown

int64 raw_encoder           # [counts] motor-side encoder value, before conversion

bool enabled                # drive energised
uint8 fault_code            # 0 = no fault, otherwise driver specific
string fault_message        # human readable decoding of fault_code
