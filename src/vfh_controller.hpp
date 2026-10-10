#pragma once

#include <vector>

#include "types.hpp"
#include "config.hpp"
#include "perception_result.hpp"

// Legacy depth-only controller, retained for its standalone module test.
// AUTOPILOT in the driving pipeline now uses FollowSearchController instead.
// There is no tracked object here — just the
// dense depth_map — so steering comes from a VFH+-style polar histogram
// (nearest obstacle per angular sector, hysteresis blocked/free, wide/narrow
// valley selection biased toward straight-ahead) and throttle is derived
// directly from that same histogram's clearance in the chosen direction,
// eased off in turns. No separate PID loop: there's no fixed stand-off
// distance to hold in open-road driving.
class VFHController {
public:
    explicit VFHController(const PipelineConfig& cfg);

    DriveCommand compute_control(const PerceptionResult& result);

private:
    void  build_polar_histogram(const cv::Mat& depth_map, std::vector<float>& out) const;
    int   select_direction(const std::vector<bool>& blocked, const std::vector<float>& dist) const;
    float sector_to_steering(int sector) const;

    // Comparison direction flips with cfg_.follow_invert_depth: raw values
    // can mean either "larger = farther" or "larger = closer" depending on
    // the depth model in use.
    bool too_close(float raw, float thresh) const;

    const PipelineConfig& cfg_;
    int num_sectors_;

    std::vector<bool> prev_blocked_;
    float              prev_steering_{0.0f};
};
