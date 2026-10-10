#pragma once

#include <cstddef>
#include <string>

enum class DepthModel { FASTDEPTH, DEPTH_ANYTHING };

struct PipelineConfig {
    // ── Camera (T1) ───────────────────────────────────────────────────────────
    int    camera_width           = 1280;
    int    camera_height          = 720;
    size_t frame_queue_size       = 2;      // ring buffer slots between T1 and T2

    // ── Inference (T2) ────────────────────────────────────────────────────────
    std::string yolo_hef          = "models/hailo/yolov8s_h8l.hef";
    DepthModel  depth_model       = DepthModel::FASTDEPTH;
    int         yolo_input_size   = 640;    // YOLO letterbox target (pixels)
    float       conf_threshold    = 0.4f;
    float       nms_iou_threshold = 0.45f;
    int         infer_timeout_ms  = 1000;
    size_t      perception_queue_size = 2;  // ring buffer slots between T2 and T3

    const char* depth_hef() const {
        return depth_model == DepthModel::FASTDEPTH
            ? "models/hailo/fastdepth--224x224.hef"
            : "models/hailo/depth_anything_v2--224x224.hef";
    }

    // ── Decision (T3) ────────────────────────────────────────────────────────
    size_t command_queue_size     = 2;      // ring buffer slots between T3 and T4
    int    servo_gpio_pin         = 25;     // camera tilt servo pin
    double angle_follow_me_mode   = 0.0;   // degrees
    double angle_autopilot_mode   = 0.0;   // search uses the same camera tilt as FOLLOW
    // Starting points from module_test/follow_gain_tuner.cpp's closed-loop
    // sweep (simplified plant, no motor lag/inertia modeled) — confirm on
    // the real robot and retune from here, don't treat as final.
    float  pi_kp                  = 0.8;   // throttle: distance-hold gain
    float  pi_ki                  = 0.0;

    float  pd_kp                  = 0.6;   // steering: person-centering gain
    float  pd_kd                  = 0.0;

    // Target distance to the tracked person in FOLLOW mode, in the same
    // raw/uncalibrated depth-model units as Detection::depth (see
    // perception_result.hpp) — read the printed
    // depth at the desired stand-off distance during bench testing and set
    // this to match.
    float  follow_target_depth    = 1.5f;
    // Flip if the depth model turns out to report inverse depth (larger
    // value = closer) rather than the assumed larger = farther. This is a
    // property of the depth model; the legacy VFH test also uses this flag.
    bool   follow_invert_depth    = false;

    // ── FOLLOW acquisition / AUTOPILOT search ───────────────────────────────
    int    follow_startup_wait_ms    = 10000; // wait stationary before the first scan
    float  follow_search_steering    = 0.55f; // zero throttle, slow in-place rotation
    int    follow_search_first_ms    = 6000;  // turn toward the last known side
    int    follow_search_reverse_ms  = 12000;  // sweep back past the starting direction
    float  follow_search_center_band = 0.1f;  // normalized horizontal error around center
    int    follow_search_center_wait_ms = 500; // possible occlusion: pause before searching
    int    follow_perception_timeout_ms = 200; // stop on absent/stale inference results
    int    follow_reacquire_ramp_ms  = 500;   // ease throttle in after acquisition
    // Loss requires two consecutive empty results; acquisition requires two
    // consecutive person results. Any person is accepted (no identity tracking).

    // ── Legacy VFH+ module test (not used by the driving pipeline) ────────────
    float  obstacle_dist_m          = 1.0f; // obstacle threshold in raw depth-model units
    // Hysteresis exit threshold: a sector must clear this — farther away
    // than obstacle_dist_m, same raw units/polarity — to leave the blocked
    // state. Bench-calibrate the same way as obstacle_dist_m/follow_target_depth.
    float  vfh_obstacle_clear_dist_m = 1.3f;
    int    vfh_num_sectors           = 24;    // depth_map columns divided into this many angular sectors
    float  vfh_row_top_frac          = 0.35f; // ignore rows above this fraction of image height (sky)
    float  vfh_row_bottom_frac       = 0.95f; // ignore rows below this fraction (ground/hood immediately ahead)
    int    vfh_smax_sectors          = 4;     // valleys narrower than this: steer through center; wider: steer to border + smax/2 margin
    float  vfh_max_steer_rate        = 0.15f; // max |steering| change per decision tick (slew limit)
    // Bench-calibrate: raw depth value considered "clear" — farther than
    // obstacle_dist_m, same raw units/polarity. Throttle ramps to full
    // between obstacle_dist_m and this value.
    float  vfh_full_speed_dist_m     = 3.0f;
    float  vfh_turn_slowdown         = 0.5f;  // fraction of throttle shed at full steering lock

    // ── RC Receiver (drive mode selector) ────────────────────────────────────
    std::string rc_serial_port    = "/dev/serial0";
    int    rc_baud                = 420000;
    int    rc_ch_steering         = 3;      // CRSF channel index (AETR: Aileron)
    int    rc_ch_throttle         = 1;      // CRSF channel index (AETR: Throttle)
    int    rc_ch_mode_switch      = 6;      // CRSF channel index (AUX1)
    int    rc_ch_mode_activation  = 9;
    // 3-position mode switch: raw < low -> AUTOPILOT, low..high -> FOLLOW,
    // raw > high -> MANUAL.
    int    rc_switch_threshold_low  = 500;
    int    rc_switch_threshold_high = 1500;
    int    rc_switch_activation_thr = 1000;
    int    rc_channel_min         = 172;
    int    rc_channel_mid         = 992;
    int    rc_channel_max         = 1811;
    float  rc_deadzone            = 0.05f;  // normalised, applied around center
    bool   rc_invert_steering     = false;
    bool   rc_invert_throttle     = false;
    int    rc_signal_timeout_ms   = 150;    // no valid frame within this => failsafe

    // ── Actuation (T4) ───────────────────────────────────────────────────────
    int    gpio_chip              = 4;

    int    left_front_ena         = 19;
    int    left_front_in1         = 6;
    int    left_front_in2         = 5;
    int    left_rear_enb          = 13;
    int    left_rear_in3          = 20;
    int    left_rear_in4          = 16;

    int    right_front_ena        = 18;
    int    right_front_in1        = 27;
    int    right_front_in2        = 22;
    int    right_rear_enb         = 12;
    int    right_rear_in3         = 23;
    int    right_rear_in4         = 24;

    float  max_throttle           = 0.6f;   // normalised 0–1
    float  max_steering           = 1.0f;   // normalised −1 to 1
    int    watchdog_timeout_ms    = 200;    // zero PWM if no command within this

    int    spin_360_duration_ms   = 3000;
    int    turn_180_duration_ms   = 1500;
};
