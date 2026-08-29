# Flight Safety Supervisor

## Scope

`planning_pkg/flight_safety_supervisor` is a fail-closed ROS 2 command gate
for the containment pipeline. It provides observable target-lock and
containment state, operator-controlled manual or automatic activation, and a
software emergency-hold interlock.

It is not a replacement for PX4 battery, RC, data-link, geofence, or
Offboard-loss failsafes. It never calls MAVROS arm/disarm, flight-mode, RTL,
or landing services. A physical emergency-stop procedure and PX4 failsafe
configuration remain mandatory for every real flight.

## Data Flow

```text
enclosure_node
  /enclosure_command (header + sequence heartbeat)
        |
        v
flight_safety_supervisor
  /flight_safety/enclosure_command --> enclosure_command_bridge --> planner
  /flight_safety/hold_request ------> px4_offboard_bridge
  /flight_safety/status ------------> operator display / recorder
  /flight_safety/control <----------- operator service client
```

The supervisor starts in `LOCKED`; it publishes `hold_request=true` until a
fresh, authorized activation and a fresh enclosure command make it `ACTIVE`.
The Offboard bridge clears its current path on hold. With a known local pose,
it streams that captured position; without one, it stops sending setpoints so
PX4's configured Offboard-loss failsafe remains authoritative. Releasing hold
does not restore an old path: the planner must publish a new scoped path.

For containment-side wiring, start the integrated launch below. It starts
only the safety supervisor, enclosure producer, and command bridge; it
requires an already running target source and planner, and it does not control
PX4:

```bash
ros2 launch planning_pkg supervised_containment.launch.py \
  target_topic:=/target_track_world \
  require_mavros_connection:=true \
  mavros_state_topic:=/uav0/mavros/state
```

It is the supported replacement for connecting
`enclosure_command_bridge` directly to `/enclosure_command`. The launch turns
on enclosure heartbeats and routes the bridge only from
`/flight_safety/enclosure_command`.

## States

| State | Gate behavior | Exit |
| --- | --- | --- |
| `LOCKED` | Blocks enclosure commands; requests hold. Default after startup, disable, or reset. | Fresh manual or automatic enable request after base health check. |
| `MANUAL_READY` | Holds while waiting for a fresh post-enable command. | Fresh command moves to `ACTIVE`; disable returns to `LOCKED`. |
| `AUTO_READY` | Holds while waiting for both a stable target lock and a fresh command. | Both conditions move to `ACTIVE`. |
| `ACTIVE` | Forwards only fresh, sequenced enclosure commands. | Timeout, lost required target lock, invalid/replayed command, emergency hold, or disable. |
| `FAULT` | Latches hold and fault bits. | A newer explicit reset with ground confirmation returns to `LOCKED`. |
| `EMERGENCY_HOLD` | Latches hold from an operator request. | A newer explicit reset with ground confirmation returns to `LOCKED`. |

Automatic mode requires the same confirmed target ID in two consecutive fresh
`TargetTrackArray` samples by default. Manual mode may be configured to impose
the same requirement using `require_target_lock_in_manual:=true`.

## Interfaces

`swarm_interfaces/msg/FlightSafetyStatus` is published as durable, reliable
state on `/flight_safety/status`. It includes state, activation mode, target
lock ID, command/control sequence, command freshness, MAVROS
freshness/connection, hold state, fault bitmask, and human-readable reason.
It can be monitored directly:

```bash
ros2 topic echo /flight_safety/status
```

For routine operator observation and control, use the bundled terminal
console. It reads the current session and last consumed request ID from
status, then creates a fresh expiring request rather than requiring a copied
service payload:

```bash
ros2 run planning_pkg flight_safety_console watch
ros2 run planning_pkg flight_safety_console enable-manual --operator-id safety_pilot
ros2 run planning_pkg flight_safety_console enable-auto --operator-id safety_pilot
ros2 run planning_pkg flight_safety_console emergency-hold --operator-id safety_pilot
ros2 run planning_pkg flight_safety_console reset-fault --operator-id safety_pilot --ground-confirmed
```

The console has no PX4 mode, arm, RTL, or land controls. The ground-confirmed
flag is an audited operator interlock, not a claim that physical conditions
have been sensed or verified.

`swarm_interfaces/srv/SafetyControl` is available at
`/flight_safety/control`. Commands are `ENABLE_MANUAL`, `ENABLE_AUTO`,
`DISABLE`, `EMERGENCY_HOLD`, and `RESET_FAULT`.

Each request needs the current `session_id` from status, a nonempty operator
ID, a strictly increasing `request_id`, and a future `expires_at` timestamp.
The supervisor randomizes `session_id` at every start, so a request captured
from an earlier process is rejected. `RESET_FAULT` additionally requires
`ground_confirmed:=true` unless that guard is deliberately disabled in a
SITL-only configuration. A well-formed request consumes its request ID even
when the requested state transition is denied, so it cannot be replayed with
a changed payload. These checks prevent accidental/replayed requests;
they do not authenticate ROS graph participants. Real deployment must use an
isolated `ROS_DOMAIN_ID`, access-controlled network, and SROS2/DDS security.

`EnclosureCommandArray` now carries `header` and a monotonically increasing
`sequence`. The safety gate rejects missing, stale, future-dated, invalid, or
replayed command heartbeats. Enable `enclosure_node.publish_heartbeat:=true`
when the safety gate is in use, otherwise a static but valid enclosure plan
will correctly time out.

## SITL Bring-Up

Build shared interfaces before packages that import them:

```bash
cd ~/Swarm-Control-System/ros2_ws
colcon build --packages-select swarm_interfaces containment_pkg planning_pkg
source install/setup.bash
export PX4_SITL_ROOT=/home/hhh/src/PX4-Autopilot
ros2 launch planning_pkg flight_safety_sitl.launch.py
```

This launch starts the supervisor first and enables the Offboard bridge's
initial safety hold. It explicitly disables the bridge's legacy SITL
auto-arm behavior; it does not activate containment, arm an aircraft, or
change PX4 modes on behalf of the supervisor.

When not using `supervised_containment.launch.py`, route the bridge through
the gated topic explicitly:

```bash
ros2 run containment_pkg enclosure_command_bridge --ros-args \
  -p command_topic:=/flight_safety/enclosure_command \
  -p output_topic:=/task_assignment
```

Start `enclosure_node` with `publish_heartbeat:=true`. Use a dedicated ROS 2
service client or the operator console to issue a request whose expiration is
in the future according to the active ROS clock. Do not use a shell command
with a copied old timestamp or request ID, because the supervisor will reject
it as expired or replayed.

## Fault Response

| Observation | Supervisor action | Flight-control boundary |
| --- | --- | --- |
| Drone-state timeout or no available platform | Latch `FAULT`, block commands, request hold. | PX4 remains responsible for aircraft failsafe. |
| Required MAVROS state timeout/disconnect | Latch `FAULT`, block commands, request hold. | No mode/arming request is sent. |
| Target timeout or lock loss in auto mode | Latch `FAULT`, block commands, request hold. | Manual mode is only allowed when explicitly enabled. |
| Command timeout, malformed payload, future stamp, or sequence replay | Latch `FAULT`, block command, request hold. | Existing path is cleared by the bridge. |
| Operator emergency hold | Latch `EMERGENCY_HOLD`, request hold, clear path. | Use RC/PX4/physical procedure for actual vehicle emergency actions. |
| Operator reset | Requires fresh request, operator ID, and ground confirmation. Returns to `LOCKED`. | It never resumes a path or activates a flight mode. |

## Acceptance Tests

1. Start locked and confirm `/flight_safety/hold_request` is true before any
   operator action.
2. Verify old session IDs, duplicate request IDs, expired requests, and
   duplicate command sequences are rejected and never forwarded.
3. Enable manual mode, publish a fresh command, and verify `ACTIVE`; stop the
   heartbeat and verify a latched `FAULT` and hold request.
4. Enable automatic mode, publish two fresh confirmed samples for one target,
   then verify target loss closes the gate.
5. During SITL Offboard streaming, issue `EMERGENCY_HOLD`; verify the bridge
   clears its path and captures current local pose without any arm/mode call.
6. Repeat with no local pose; verify no synthetic position is published and
   PX4's configured Offboard-loss behavior is observed in a protected SITL
   environment.

Real-flight use remains blocked until the deployment gates in
`real_uav_deployment.md` are completed, including calibration, RC and
failsafe tests, enclosed-field approval, and a physical emergency procedure.
