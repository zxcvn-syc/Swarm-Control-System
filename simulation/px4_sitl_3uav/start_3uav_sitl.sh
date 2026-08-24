#!/usr/bin/env bash
# start_3uav_sitl.sh — Launch 3 PX4 SITL instances with Gazebo Classic (headless).
#
# Re-implements PX4's sitl_multiple_run.sh core (gzserver + 3 iris + 3 px4)
# WITHOUT the trailing `gzclient` (which, in headless WSL, exits and triggers
# sitl_multiple_run.sh's cleanup trap that KILLS the px4 instances — that was
# the cause of "Connection closed by client" repeating forever).
#
# Per PX4 v1.14 px4-rc.mavlink:
#   udp_offboard_port_local  = 14580 + px4_instance   (PX4 listens, mavros connects here)
#   udp_offboard_port_remote = 14540 + px4_instance   (PX4 sends, mavros binds here)
# sitl_multiple_run.sh runs `px4 -i N` with N=1,2,3, so:
#   instance1 -> PX4 listens 14581, sysid 2
#   instance2 -> PX4 listens 14582, sysid 3
#   instance3 -> PX4 listens 14583, sysid 4
# Gazebo sim link uses TCP 4560+N and UDP 14560+N per instance.
#
# Usage:  bash start_3uav_sitl.sh
# Then run start_3uav_ros.sh in another terminal once you see "Ready for takeoff".

set -e

PX4_SRC="${PX4_SITL_ROOT:-$HOME/src/PX4-Autopilot}"
TARGET="px4_sitl_default"
build_path="$PX4_SRC/build/$TARGET"
world="empty"
NUM=3

# --- kill leftovers -------------------------------------------------------
echo "[sitl] killing existing px4 / gzserver / gzclient / mavros ..."
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gazebo 2>/dev/null || true
pkill -9 -f px4 2>/dev/null || true
pkill -9 -f mavros_node 2>/dev/null || true
# Wait for the Gazebo master port (11345) to be released. A leftover
# gzserver from a previous crashed run is the usual cause of
# "Unable to start server [bind: Address already in use]".
for _i in $(seq 1 15); do
    if ss -tlnp 2>/dev/null | grep -q ':11345'; then
        echo "[sitl]   Gazebo port 11345 still in use, waiting ($_i) ..."
        sleep 1
    else
        break
    fi
done
if ss -tlnp 2>/dev/null | grep -q ':11345'; then
    echo "[sitl] WARNING: 11345 STILL in use — another gzserver is running."
    echo "[sitl]          Run: ss -tlnp | grep 11345   to find its PID, then kill -9 <pid>"
fi

cd "$PX4_SRC"
# setup_gazebo.bash sets GAZEBO_MODEL_PATH / GAZEBO_PLUGIN_PATH etc.
source "$PX4_SRC/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
    "$PX4_SRC" "$build_path"

# Headless: do NOT set ROS_VERSION so gzserver is launched without the
# gazebo_ros factory plugins (we spawn models via `gz model` directly).
export HEADLESS=1
export DISPLAY=""
export PX4_SIM_MODEL=gazebo-classic_iris
unset ROS_VERSION 2>/dev/null || true

GAZEBO_WORLD="$PX4_SRC/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/${world}.world"
JINJA="$PX4_SRC/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py"
MODEL_DIR="$PX4_SRC/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris"
PX4_BIN="$build_path/bin/px4"

if [[ ! -x "$PX4_BIN" ]]; then
    echo "[sitl] ERROR: px4 binary not found at $PX4_BIN"
    echo "[sitl]       build it first: make px4_sitl_default gazebo-classic"
    exit 1
fi

# --- start gzserver (log it so we can diagnose if the sim link fails) -----
echo "[sitl] starting gzserver (log: /tmp/gazebo_multi.log) ..."
rm -f /tmp/gazebo_multi.log
gzserver "$GAZEBO_WORLD" --verbose > /tmp/gazebo_multi.log 2>&1 &
GZ_PID=$!
sleep 5
if ! kill -0 "$GZ_PID" 2>/dev/null; then
    echo "[sitl] ERROR: gzserver exited immediately. Tail of /tmp/gazebo_multi.log:"
    tail -n 40 /tmp/gazebo_multi.log
    exit 1
fi
echo "[sitl] gzserver is up (pid $GZ_PID)."

# --- spawn NUM iris + start NUM px4 instances ------------------------------
for n in $(seq 1 $NUM); do
    echo "[sitl] spawning iris_${n} (px4 instance $n) ..."

    working_dir="$build_path/rootfs/$n"
    mkdir -p "$working_dir"

    # start px4 instance in its own rootfs dir (offboard ports auto-offset by -i)
    ( cd "$working_dir" && "$PX4_BIN" -i "$n" -d "$build_path/etc" \
        > "$working_dir/out.log" 2> "$working_dir/err.log" ) &
    PX4_PID=$!
    echo "[sitl]   px4 instance $n pid $PX4_PID (logs: $working_dir/{out,err}.log)"

    # generate the per-instance iris SDF (sets mavlink tcp/udp ports)
    python3 "$JINJA" \
        "$MODEL_DIR/iris.sdf.jinja" \
        "$PX4_SRC/Tools/simulation/gazebo-classic/sitl_gazebo-classic" \
        --mavlink_tcp_port $((4560 + n)) \
        --mavlink_udp_port $((14560 + n)) \
        --mavlink_id      $((1 + n)) \
        --gst_udp_port    $((5600 + n)) \
        --video_uri       $((5600 + n)) \
        --mavlink_cam_udp_port $((14530 + n)) \
        --output-file /tmp/iris_${n}.sdf

    # spawn the model into the running gzserver (retry until ready)
    for attempt in 1 2 3 4 5; do
        if gz model --spawn-file=/tmp/iris_${n}.sdf \
                    --model-name=iris_${n} -x 0 -y $((3 * n)) -z 0.83 2>&1 \
                | grep -q "An instance of Gazebo is not running"; then
            echo "[sitl]   gzserver not ready yet, retrying ($attempt) ..."
            sleep 2
        else
            break
        fi
    done
    echo "[sitl]   iris_${n} spawned."
    sleep 2
done

echo ""
echo "======================================================="
echo "  3x PX4 SITL + Gazebo Classic (headless) running"
echo "  PX4 offboard listen ports: 14581 / 14582 / 14583"
echo "  Wait for 'Ready for takeoff!' then run start_3uav_ros.sh"
echo "  gzserver log: /tmp/gazebo_multi.log"
echo "  px4 logs:     $build_path/rootfs/{1,2,3}/{out,err}.log"
echo "  Ctrl-C to stop all."
echo "======================================================="

# keep this script alive (so the backgrounded px4/gzserver keep running)
wait
