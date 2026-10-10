#pragma once

#include "config.hpp"
#include "perception_result.hpp"
#include "pid_controller.hpp"
#include "types.hpp"

// Follow/search state belongs to the decision thread. Explicit timestamps let
// the transition tests run without sleeping or touching robot hardware.
class FollowSearchController {
public:
    enum class State { IDLE, STARTUP_WAIT, TRACKING, SEARCHING, WAITING };

    explicit FollowSearchController(const PipelineConfig& cfg);

    // AUTOPILOT explicitly requests an immediate scan; FOLLOW first waits for
    // a person. Both become normal FOLLOW after acquiring any person.
    void start(bool immediate_search, int64_t time_ms);
    void stop();
    void tick(int64_t time_ms);
    void observe(const PerceptionResult& result, int64_t time_ms);
    void perception_timeout();
    DriveCommand command(int64_t time_ms) const;

    State state() const { return state_; }
    static const char* state_name(State state);

private:
    void begin_search(int64_t time_ms, bool pause_for_center);
    void reset_controllers();

    const PipelineConfig& cfg_;
    PIDController steer_;
    PIDController throttle_;
    State state_{State::IDLE};
    bool initial_acquisition_{true};
    bool perception_available_{false};
    int missing_frames_{0};
    int candidate_frames_{0};
    int last_side_{0}; // -1 = left, +1 = right, 0 = unknown/centered
    int search_direction_{1};
    int64_t startup_deadline_ms_{0};
    int64_t search_start_ms_{0};
    int64_t search_deadline_ms_{0};
    int64_t tracking_start_ms_{0};
    DriveCommand tracking_command_{};
};
