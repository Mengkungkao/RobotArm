# Calibration record of a single joint, mirroring calibration.yaml.

string name

float64 zero_offset         # [rad] raw joint angle at the mechanical zero
int8 direction              # +1 or -1, sign relative to the URDF joint axis
float64 min_position        # [rad] enforced soft lower limit
float64 max_position        # [rad] enforced soft upper limit
float64 home_position       # [rad] target of the homing routine
