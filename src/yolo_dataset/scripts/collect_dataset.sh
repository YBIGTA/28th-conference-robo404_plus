#!/usr/bin/env bash
# ---------------------------------------------------------------
# Batch dataset collection with domain-randomised worlds.
#
# For each generated .world file:
#   1. Launch Gazebo + robot
#   2. Launch the follower (autopilot)
#   3. Launch the recorder
#   4. Wait DURATION seconds
#   5. Kill everything, move to next world
#
# Usage:
#   # Step 1: Generate worlds (once)
#   python3 -m yolo_dataset.generate_worlds \
#       --config src/yolo_dataset/config/world_randomization.yaml \
#       --output-dir /tmp/rand_worlds --num 20 --seed 42
#
#   # Step 2: Collect from all worlds
#   bash src/yolo_dataset/scripts/collect_dataset.sh \
#       /tmp/rand_worlds \
#       dataset_output \
#       60
#
# Arguments:
#   $1 - Directory containing generated .world files
#   $2 - Base output directory for dataset (default: robo404_dataset)
#   $3 - Seconds to record per world (default: 60)
# ---------------------------------------------------------------
set -euo pipefail

WORLDS_DIR="${1:?Usage: $0 <worlds_dir> [output_dir] [duration_sec]}"
OUTPUT_DIR="${2:-robo404_dataset}"
DURATION="${3:-60}"

WORLD_FILES=("$WORLDS_DIR"/rand_*.world)
NUM_WORLDS=${#WORLD_FILES[@]}

if [ "$NUM_WORLDS" -eq 0 ]; then
    echo "ERROR: No rand_*.world files found in $WORLDS_DIR"
    exit 1
fi

echo "============================================"
echo " Domain-Randomised Dataset Collection"
echo " Worlds:   $NUM_WORLDS"
echo " Duration: ${DURATION}s per world"
echo " Output:   $OUTPUT_DIR/"
echo "============================================"

cleanup() {
    echo "Cleaning up ROS processes..."
    # Kill by process group - these are the PIDs we launched
    for pid in "${PIDS[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null || true
        fi
    done
    sleep 2
    for pid in "${PIDS[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    # Make sure gzserver/gzclient are dead
    pkill -f gzserver 2>/dev/null || true
    pkill -f gzclient 2>/dev/null || true
    sleep 2
}

trap cleanup EXIT

for i in "${!WORLD_FILES[@]}"; do
    WORLD="${WORLD_FILES[$i]}"
    WORLD_NAME=$(basename "$WORLD" .world)
    RUN_DIR="${OUTPUT_DIR}/${WORLD_NAME}"
    PIDS=()

    echo ""
    echo "--- [$((i + 1))/$NUM_WORLDS] $WORLD_NAME ---"
    echo "    World: $WORLD"
    echo "    Output: $RUN_DIR/"

    # Skip if this run already has data (resume-friendly)
    if [ -f "$RUN_DIR/dataset.yaml" ]; then
        echo "    SKIP: already collected"
        continue
    fi

    mkdir -p "$RUN_DIR"

    # 1) Launch Gazebo + robot (headless for speed)
    echo "    Starting Gazebo..."
    ros2 launch robo_description gazebo.launch.py \
        world:="$WORLD" \
        extra_gazebo_args:="--headless" \
        &>/dev/null &
    PIDS+=($!)

    # Wait for Gazebo to be ready (model_states topic)
    echo "    Waiting for Gazebo..."
    TIMEOUT=60
    ELAPSED=0
    while ! ros2 topic list 2>/dev/null | grep -q "/gazebo/model_states"; do
        sleep 2
        ELAPSED=$((ELAPSED + 2))
        if [ "$ELAPSED" -ge "$TIMEOUT" ]; then
            echo "    ERROR: Gazebo did not start within ${TIMEOUT}s, skipping"
            cleanup
            continue 2
        fi
    done
    echo "    Gazebo ready (${ELAPSED}s)"

    # 2) Launch follower autopilot
    echo "    Starting follower..."
    ros2 run follower line_tracker_node &>/dev/null &
    PIDS+=($!)
    sleep 2

    # 3) Launch recorder
    echo "    Recording for ${DURATION}s..."
    ros2 launch yolo_dataset record.launch.py \
        output_dir:="$RUN_DIR" \
        &>/dev/null &
    PIDS+=($!)

    # 4) Wait
    sleep "$DURATION"

    # 5) Tear down
    echo "    Stopping..."
    cleanup
    PIDS=()

    # Count what we got
    if [ -d "$RUN_DIR/images" ]; then
        N_FRAMES=$(find "$RUN_DIR/images" -name "*.png" 2>/dev/null | wc -l)
        echo "    Collected $N_FRAMES frames"
    else
        echo "    WARNING: No frames collected"
    fi
done

echo ""
echo "============================================"
echo " Collection complete!"
echo ""

# Summary
TOTAL=0
for d in "$OUTPUT_DIR"/rand_*/images; do
    if [ -d "$d" ]; then
        N=$(find "$d" -name "*.png" 2>/dev/null | wc -l)
        TOTAL=$((TOTAL + N))
    fi
done
echo " Total frames: $TOTAL across $NUM_WORLDS worlds"
echo " Output: $OUTPUT_DIR/"
echo "============================================"
