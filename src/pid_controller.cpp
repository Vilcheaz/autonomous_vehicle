#include "pid_controller.hpp"

#include <algorithm>

PIDController::PIDController(const PipelineConfig& cfg, PIDAxis axis,
                             float kp, float ki, float kd)
        : cfg_(cfg), axis_(axis), kp_(kp), ki_(ki), kd_(kd) {}

PIDController::~PIDController() {}

void PIDController::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    prev_time_ms_ = 0;
    had_target_ = false;
}

bool PIDController::find_target(const PerceptionResult& result, Detection& out) const {
    bool found = false;
    int  best_area = -1;

    for (const auto& det : result.detections) {
        if (det.label != "person") continue;
        int area = (det.x2 - det.x1) * (det.y2 - det.y1);
        if (area > best_area) {
            best_area = area;
            out = det;
            found = true;
        }
    }
    return found;
}

float PIDController::compute_error(const Detection& target, int frame_w) const {
    if (axis_ == PIDAxis::STEERING) {
        float cx     = (target.x1 + target.x2) / 2.0f;
        float half_w = frame_w / 2.0f;
        // -1 (person at left edge) .. +1 (person at right edge)
        return half_w > 0.0f ? (cx - half_w) / half_w : 0.0f;
    }

    // +ve = further than target distance (need to close in), -ve = too close
    float error = target.depth - cfg_.follow_target_depth;
    return cfg_.follow_invert_depth ? -error : error;
}

float PIDController::compute_control(const PerceptionResult& result) {
    Detection target;
    if (!find_target(result, target)) {
        // Lost the person: hold this axis at zero and drop the accumulated
        // state so reacquiring them doesn't inherit a stale integral/derivative.
        reset();
        return 0.0f;
    }

    float   error = compute_error(target, result.frame_w);
    int64_t now   = now_ms();
    float   dt    = had_target_ ? (now - prev_time_ms_) / 1000.0f : 0.0f;

    float derivative = (dt > 1e-3f) ? (error - prev_error_) / dt : 0.0f;

    integral_ += error * dt;
    if (ki_ > 1e-6f) {
        // Anti-windup: cap the integral so ki_ * integral_ alone can't
        // exceed the output range, regardless of how long the error persists.
        float limit = 1.0f / ki_;
        integral_ = std::clamp(integral_, -limit, limit);
    } else {
        integral_ = 0.0f;
    }

    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    prev_error_   = error;
    prev_time_ms_ = now;
    had_target_   = true;

    return std::clamp(output, -1.0f, 1.0f);
}
