#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

cmake -S . -B build
cmake --build build -- -j$(nproc)
echo -e ""
echo "Choose program:"
echo -e ""
echo "Module Tests"
echo "1) Camera test"
echo "2) Depth estimation"
echo "3) Object detection"
echo "4) Servo test"
echo "5) RC receiver test"
echo "6) Motor test"
echo "7) Legacy VFH+ controller test (not used by AUTOPILOT)"
echo "8) Follow controller test (FOLLOW)"
echo "9) Follow PD/PI gain tuner (FOLLOW, ~1 min)"

echo -e ""

echo "Pipelines"
echo "10) Camera capture and inference"
echo "11) Full Pipeline"
echo "12) Pipeline recorder test (no robot hardware)"
echo "13) Person presence event test (no robot hardware)"
echo "14) Follow search / recovery test (no robot hardware)"

read -p "Select: " choice

if [ "$choice" = "1" ]; then
    ./build/camera_test
elif [ "$choice" = "2" ]; then
    ./build/depth_infer
elif [ "$choice" = "3" ]; then
    ./build/object_detector
elif [ "$choice" = "4" ]; then
    echo "Final servo position:"
    echo "1) Follow me"
    echo "2) Autopilot (default)"
    while true; do
        read -r -p "Select final position [2]: " servo_mode
        case "$servo_mode" in
            1) ./build/servo_test follow-me; break ;;
            2|"") ./build/servo_test autopilot; break ;;
            *) echo "Invalid option. Choose 1 or 2." ;;
        esac
    done
elif [ "$choice" = "5" ]; then
    ./build/rc_receiver_test
elif [ "$choice" = "6" ]; then
    ./build/motor_test
elif [ "$choice" = "7" ]; then
    ./build/vfh_test
elif [ "$choice" = "8" ]; then
    ./build/follow_test
elif [ "$choice" = "9" ]; then
    ./build/gain_tuner
elif [ "$choice" = "10" ]; then
    ./build/pipeline_visual_test
elif [ "$choice" = "11" ]; then
    read -r -p "Record annotated video and logs? [y/N] " record_choice
    PIPELINE_ARGS=()
    if [[ "$record_choice" =~ ^[Yy]([Ee][Ss])?$ ]]; then
        SESSION_DIR="$SCRIPT_DIR/results/full_pipeline/$(date +%Y%m%d_%H%M%S)_$$"
        mkdir -p "$SESSION_DIR"
        PIPELINE_ARGS=(--record --record-dir "$SESSION_DIR")
        echo "Recording to $SESSION_DIR"
        "$SCRIPT_DIR/venv/bin/python" -u "$SCRIPT_DIR/oww.py" \
            > "$SESSION_DIR/wake_word.log" 2>&1 &
    else
        "$SCRIPT_DIR/venv/bin/python" "$SCRIPT_DIR/oww.py" &
    fi
    OWW_PID=$!
    trap "kill $OWW_PID 2>/dev/null; wait $OWW_PID 2>/dev/null" EXIT
    ./build/pipeline "${PIPELINE_ARGS[@]}"
elif [ "$choice" = "12" ]; then
    ./build/recorder_test
elif [ "$choice" = "13" ]; then
    ./build/person_event_test
elif [ "$choice" = "14" ]; then
    ctest --test-dir build --output-on-failure -R search
else
    echo "Invalid option"
fi
