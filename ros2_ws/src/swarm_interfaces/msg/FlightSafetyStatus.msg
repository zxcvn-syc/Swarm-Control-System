# Runtime status published by the flight-safety supervisor. This is an
# advisory/control-gate status, not a substitute for PX4 failsafes or a
# physical emergency-stop circuit.
std_msgs/Header header

uint8 STATE_LOCKED=0
uint8 STATE_MANUAL_READY=1
uint8 STATE_AUTO_READY=2
uint8 STATE_ACTIVE=3
uint8 STATE_FAULT=4
uint8 STATE_EMERGENCY_HOLD=5
uint8 state

uint8 MODE_NONE=0
uint8 MODE_MANUAL=1
uint8 MODE_AUTO=2
uint8 activation_mode

bool containment_enabled
bool hold_requested
bool target_locked
uint32 locked_target_id
bool drone_states_fresh
bool command_fresh
bool mavros_fresh
bool mavros_connected
uint64 session_id
# Last consumed SafetyControl request id in this supervisor session.
uint64 last_control_request_id
uint64 last_command_sequence
uint32 fault_mask
string reason
