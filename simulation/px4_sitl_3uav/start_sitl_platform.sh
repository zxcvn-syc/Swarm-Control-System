#!/usr/bin/env bash
# start_sitl_platform.sh — Platform layer for the SITL enclosure test ONLY.
#
# Brings up the real-platform ROS side so escape_eval_sitl.launch.py can
# consume /drone_states:
#   1. 3 mavros nodes (connect to the running PX4 SITL instances)
#   2. sitl_state_publisher -> /drone_states (3 real UAV poses + 2 mock UGV)
#
# Prerequisites: start_3uav_sitl.sh already running in another terminal
# (3 PX4 SITL instances listening on UDP 14581/14582/14583).
#
# IMPORTANT: This script does NOT start enclosure_node or target_pub. Those
# are started by escape_eval_sitl.launch.py. Running this together with the
# full start_3uav_ros.sh would double-start enclosure_node (name clash).
#
# Usage:  bash start_sitl_platform.sh

set -eo pipefail

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Repo-relative path to the state publisher (robust to clone location).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
STATE_PUB="$REPO_ROOT/simulation/px4_sitl_3uav/sitl_state_publisher.py"
[[ -f "$STATE_PUB" ]] || STATE_PUB="$HOME/sitl_state_publisher.py"   # fallback
[[ -f "$STATE_PUB" ]] || { echo "[platform] sitl_state_publisher.py not found"; exit 1; }

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

echo "[platform] waiting for PX4 offboard UDP ports (1458x) to appear ..."
PORTS=()
for i in $(seq 1 40); do
    PORTS=($(detect_px4_ports))
    if [[ ${#PORTS[@]} -ge 3 ]]; then break; fi
    sleep 1
done

if [[ ${#PORTS[@]} -lt 3 ]]; then
    echo "[platform] WARNING: only found ${#PORTS[@]} PX4 port(s): ${PORTS[*]:-(none)}"
    echo "[platform]          Falling back to 14581 14582 14583 (PX4 v1.14 default)."
    PORTS=(14581 14582 14583)
fi
PORTS=("${PORTS[0]}" "${PORTS[1]}" "${PORTS[2]}")
echo "[platform] using PX4 offboard listen ports: ${PORTS[*]}"

PIDS=()
cleanup() {
    echo ""
    echo "[platform] shutting down mavros + state_publisher ..."
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
    echo "[platform] starting mavros /uav${k} (bind ${BIND}, px4-listen ${P}, sysid ${SYSID})"
    ros2 run mavros mavros_node --ros-args \
        -r "__ns:=/uav${k}/mavros" \
        -p "fcu_url:=udp://:${BIND}@127.0.0.1:${P}" \
        -p "tgt_system:=${SYSID}" \
        > /tmp/mavros_uav${k}.log 2>&1 &
    PIDS+=("$!")
    sleep 2
done

echo "[platform] waiting 8s for mavros to connect to PX4 ..."
sleep 8
for k in 0 1 2; do
    if grep -q "CON: Got HEARTBEAT" /tmp/mavros_uav${k}.log 2>/dev/null; then
        echo "[platform]   /uav${k}/mavros: CONNECTED"
    else
        echo "[platform]   /uav${k}/mavros: NOT connected (check /tmp/mavros_uav${k}.log)"
    fi
done

# ---------------------------------------------------------------------------
# 2. sitl_state_publisher (3 real UAV poses + 2 mock UGV -> /drone_states)
# ---------------------------------------------------------------------------
echo "[platform] starting sitl_state_publisher (3 UAV real + 2 UGV mock)"
python3 "$STATE_PUB" --ros-args \
    -p "period:=0.5" \
    -p "target_x:=0.0" \
    -p "target_y:=0.0" \
    -p "block_orbit:=15.0" \
    -p "num_uav:=3" \
    -p "num_ugv:=2" \
    > /tmp/sitl_state_pub.log 2>&1 &
PIDS+=("$!")

echo ""
echo "============================================"
echo "  SITL platform layer running!"
echo "============================================"
echo "  mavros:             /uav0 /uav1 /uav2"
echo "  state_publisher ->  /drone_states (3 UAV real + 2 UGV mock)"
echo ""
echo "  Verify in another terminal:"
echo "    ros2 topic echo /drone_states --once"
echo ""
echo "  Then run the test (this does NOT start enclosure_node):"
echo "    ros2 launch containment_pkg escape_eval_sitl.launch.py \\"
echo "        scene:=park direction:=2 result_csv:=/tmp/smoke2.csv"
echo ""
echo "  Logs: /tmp/mavros_uav*.log /tmp/sitl_state_pub.log"
echo "  Press Ctrl-C to stop."
echo "============================================"

wait
