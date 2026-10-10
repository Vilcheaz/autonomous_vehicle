#pragma once

#include <cstdint>

#include "types.hpp"
#include "config.hpp"
#include "perception_result.hpp"

enum class PIDAxis { STEERING, THROTTLE };

// Drives one axis of FOLLOW's person-tracking behavior: STEERING keeps
// the tracked person horizontally centered in frame, THROTTLE holds a preset
// stand-off distance to them via Detection::depth.
class PIDController {
public:
    PIDController(const PipelineConfig& cfg, PIDAxis axis,
                   float kp, float ki = 0.0f, float kd = 0.0f);
    ~PIDController();

    // Returns a control output in -1..1. Returns 0 (and resets internal
    // state) when no person is in frame, so a reacquired target doesn't
    // inherit a stale integral/derivative.
    float compute_control(const PerceptionResult& result);
    void reset();

private:
    bool find_target(const PerceptionResult& result, Detection& out) const;
    float compute_error(const Detection& target, int frame_w) const;

    const PipelineConfig& cfg_;
    PIDAxis axis_;
    float kp_{}, ki_{}, kd_{};

    float   integral_{0.0f};
    float   prev_error_{0.0f};
    int64_t prev_time_ms_{0};
    bool    had_target_{false};
};
