#!/usr/bin/env bash
# start_3uav_ros.sh — Launch 3 mavros + state aggregator + target_pub + enclosure_node.
#
# Prerequisites: start_3uav_sitl.sh already running in another terminal
# (3 PX4 SITL instances listening on UDP 14581/14582/14583).
#
# Port scheme (PX4 v1.14 px4-rc.mavlink, instance N = 1,2,3):
#   PX4 offboard listen (mavros connects here) = 14580 + N  -> 14581/14582/14583
#   PX4 offboard send  (mavros binds here)     = 14540 + N  -> 14541/14542/14543
#   MAV_SYS_ID = N + 1  -> 2/3/4
#
# We DETECT the actual PX4 listen ports at runtime (robust to version drift),
# then bind mavros to (port-14540) and connect to (port).
#
# Usage:  bash start_3uav_ros.sh

# NOTE: no `set -u` — ROS's setup.bash references $AMENT_TRACE_SETUP_FILES which
# is unbound under `set -u`, which aborted this whole script before.
set -eo pipefail

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Copy sitl_state_publisher.py from Windows side if not present
STATE_PUB="$HOME/sitl_state_publisher.py"
if [[ ! -f "$STATE_PUB" ]]; then
    echo "[ros] copying sitl_state_publisher.py to ~/"
    cp /mnt/c/ProgramData/WorkBuddy/chromium-env/6ulcsx/WorkBuddy/2026-08-10-14-52-53/sitl_state_publisher.py "$STATE_PUB"
fi

# ---------------------------------------------------------------------------
# Detect PX4 offboard listen ports (1458x). PX4 binds these at startup.
# ---------------------------------------------------------------------------
detect_px4_ports() {
    local p=""
    if command -v ss >/dev/null 2>&1; then
        p=$(ss -ulnp 2>/dev/null | grep -oE '1458[0-9]' | sort -un)
    fi
    if [[ -z "$p" ]] && command -v netstat >/dev/null 2>&1; then
        p=$(netstat -ulnp 2>/dev/null | grep -oE '1458[0-9]' | sort -un)
    fi
    echo $p
}

echo "[ros] waiting for PX4 offboard UDP ports (1458x) to appear ..."
PORTS=()
for i in $(seq 1 40); do
    PORTS=($(detect_px4_ports))
    if [[ ${#PORTS[@]} -ge 3 ]]; then break; fi
    sleep 1
done

if [[ ${#PORTS[@]} -lt 3 ]]; then
    echo "[ros] WARNING: only found ${#PORTS[@]} PX4 port(s): ${PORTS[*]:-(none)}"
    echo "[ros]          Falling back to 14581 14582 14583 (PX4 v1.14 default)."
    echo "[ros]          If mavros still won't connect, check: ss -ulnp | grep 1458"
    PORTS=(14581 14582 14583)
fi
# take first 3
PORTS=("${PORTS[0]}" "${PORTS[1]}" "${PORTS[2]}")
echo "[ros] using PX4 offboard listen ports: ${PORTS[*]}"

PIDS=()
cleanup() {
    echo ""
    echo "[ros] shutting down..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    pkill -f mavros_node 2>/dev/null || true
    wait 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM EXIT

# ---------------------------------------------------------------------------
# 1. Start 3 mavros instances
# ---------------------------------------------------------------------------
for k in 0 1 2; do
    P="${PORTS[$k]}"
    INST=$((P - 14580))            # px4_instance
    BIND=$((14540 + INST))         # mavros local bind = PX4 offboard remote
    SYSID=$((INST + 1))            # MAV_SYS_ID
    echo "[ros] starting mavros /uav${k} (bind ${BIND}, px4-listen ${P}, sysid ${SYSID})"
    ros2 run mavros mavros_node --ros-args \
        -r "__ns:=/uav${k}/mavros" \
        -p "fcu_url:=udp://:${BIND}@127.0.0.1:${P}" \
        -p "tgt_system:=${SYSID}" \
        > /tmp/mavros_uav${k}.log 2>&1 &
    PIDS+=("$!")
    sleep 2
done

echo "[ros] waiting 8s for mavros to connect to PX4 ..."
sleep 8
for k in 0 1 2; do
    if grep -q "CON: Got HEARTBEAT" /tmp/mavros_uav${k}.log 2>/dev/null; then
        echo "[ros]   /uav${k}/mavros: CONNECTED"
    else
        echo "[ros]   /uav${k}/mavros: NOT connected (check /tmp/mavros_uav${k}.log)"
    fi
done

# ---------------------------------------------------------------------------
# 2. sitl_state_publisher (3 real UAV poses + 2 mock UGV -> /drone_states)
# ---------------------------------------------------------------------------
echo "[ros] starting sitl_state_publisher (3 UAV real + 2 UGV mock)"
python3 "$STATE_PUB" --ros-args \
    -p "period:=0.5" \
    -p "target_x:=0.0" \
    -p "target_y:=0.0" \
    -p "block_orbit:=15.0" \
    -p "num_uav:=3" \
    -p "num_ugv:=2" \
    > /tmp/sitl_state_pub.log 2>&1 &
PIDS+=("$!")

# ---------------------------------------------------------------------------
# 3. target_pub (containment_pkg -> /target_track)
# ---------------------------------------------------------------------------
echo "[ros] starting target_pub (containment_pkg)"
ros2 run containment_pkg target_pub --ros-args \
    -p "period:=1.0" \
    -p "center_x:=0.0" \
    -p "center_y:=0.0" \
    -p "orbit_radius:=3.0" \
    -p "orbit_speed:=0.3" \
    > /tmp/target_pub.log 2>&1 &
PIDS+=("$!")

# ---------------------------------------------------------------------------
# 4. enclosure_node (containment_pkg -> /enclosure_command)
# ---------------------------------------------------------------------------
echo "[ros] starting enclosure_node (containment_pkg)"
ros2 run containment_pkg enclosure_node --ros-args \
    -p "monitor_radius:=25.0" \
    -p "block_radius:=15.0" \
    -p "min_dist:=5.0" \
    -p "update_period:=1.0" \
    -p "target_track_topic:=/target_track" \
    > /tmp/enclosure_node.log 2>&1 &
PIDS+=("$!")

echo ""
echo "============================================"
echo "  3 UAV SITL + enclosure_node running!"
echo "============================================"
echo "  mavros:             /uav0 /uav1 /uav2"
echo "  state_publisher ->  /drone_states (3 UAV real + 2 UGV mock)"
echo "  target_pub      ->  /target_track"
echo "  enclosure_node  ->  /enclosure_command"
echo ""
echo "  Verify in terminal 3:"
echo "    ros2 topic echo /enclosure_command"
echo "    ros2 topic echo /drone_states"
echo ""
echo "  Logs: /tmp/mavros_uav*.log /tmp/enclosure_node.log"
echo "  Press Ctrl-C to stop."
echo "============================================"

wait
