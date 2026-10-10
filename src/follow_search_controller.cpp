#include "follow_search_controller.hpp"

#include <algorithm>

FollowSearchController::FollowSearchController(const PipelineConfig& cfg)
    : cfg_(cfg),
      steer_(cfg, PIDAxis::STEERING, cfg.pd_kp, 0.0f, cfg.pd_kd),
      throttle_(cfg, PIDAxis::THROTTLE, cfg.pi_kp, cfg.pi_ki) {}

void FollowSearchController::reset_controllers() {
    steer_.reset();
    throttle_.reset();
    tracking_command_ = {};
}

void FollowSearchController::stop() {
    state_ = State::IDLE;
    initial_acquisition_ = true;
    perception_available_ = false;
    missing_frames_ = candidate_frames_ = 0;
    last_side_ = 0;
    reset_controllers();
}

void FollowSearchController::start(bool immediate_search, int64_t time_ms) {
    stop();
    if (immediate_search) {
        begin_search(time_ms, false);
    } else {
        state_ = State::STARTUP_WAIT;
        startup_deadline_ms_ = time_ms + std::max(0, cfg_.follow_startup_wait_ms);
    }
}

void FollowSearchController::begin_search(int64_t time_ms, bool pause_for_center) {
    state_ = State::SEARCHING;
    candidate_frames_ = missing_frames_ = 0;
    search_direction_ = last_side_ == 0 ? 1 : last_side_;
    search_start_ms_ = time_ms + (pause_for_center
        ? std::max(0, cfg_.follow_search_center_wait_ms) : 0);
    search_deadline_ms_ = search_start_ms_ +
        std::max(0, cfg_.follow_search_first_ms) +
        std::max(0, cfg_.follow_search_reverse_ms);
    reset_controllers();
}

void FollowSearchController::tick(int64_t time_ms) {
    if (state_ == State::STARTUP_WAIT && time_ms >= startup_deadline_ms_) {
        begin_search(time_ms, false);
    }
    if (state_ == State::SEARCHING && time_ms >= search_deadline_ms_) {
        // A failed startup scan disarms the session. After a real target was
        // lost, wait stationary for somebody to reappear, without rescanning.
        state_ = initial_acquisition_ ? State::IDLE : State::WAITING;
        candidate_frames_ = 0;
        reset_controllers();
    }
}

void FollowSearchController::observe(const PerceptionResult& result, int64_t time_ms) {
    tick(time_ms);
    if (state_ == State::IDLE) return;
    perception_available_ = true;

    const Detection* person = nullptr;
    int largest_area = -1;
    for (const auto& det : result.detections) {
        if (det.label != "person") continue;
        int area = (det.x2 - det.x1) * (det.y2 - det.y1);
        if (area > largest_area) {
            largest_area = area;
            person = &det;
        }
    }

    if (!person) {
        candidate_frames_ = 0;
        reset_controllers();
        if (state_ == State::TRACKING && ++missing_frames_ >= 2) {
            begin_search(time_ms, last_side_ == 0);
        }
        return;
    }

    if (state_ != State::TRACKING) {
        // Pause search immediately on a candidate, then require another
        // consecutive detection before applying the existing follow controls.
        if (++candidate_frames_ < 2) return;
        state_ = State::TRACKING;
        initial_acquisition_ = false;
        tracking_start_ms_ = time_ms;
        reset_controllers();
    }

    missing_frames_ = candidate_frames_ = 0;
    float center = (person->x1 + person->x2) / 2.0f;
    float half_width = result.frame_w / 2.0f;
    float error = half_width > 0.0f ? (center - half_width) / half_width : 0.0f;
    float center_band = std::clamp(cfg_.follow_search_center_band, 0.0f, 1.0f);
    last_side_ = error < -center_band ? -1 : (error > center_band ? 1 : 0);

    tracking_command_.steering = steer_.compute_control(result);
    tracking_command_.throttle = throttle_.compute_control(result);
    if (cfg_.follow_reacquire_ramp_ms > 0) {
        float ramp = std::clamp(static_cast<float>(time_ms - tracking_start_ms_) /
                               cfg_.follow_reacquire_ramp_ms, 0.0f, 1.0f);
        tracking_command_.throttle *= ramp;
    }
}

void FollowSearchController::perception_timeout() {
    // Lack of an inference result is not an empty detection frame. It also
    // breaks consecutive confirmation/loss counts and invalidates old control.
    candidate_frames_ = missing_frames_ = 0;
    perception_available_ = false;
    reset_controllers();
}

DriveCommand FollowSearchController::command(int64_t time_ms) const {
    if (!perception_available_) return {};
    if (state_ == State::TRACKING) return tracking_command_;
    if (state_ != State::SEARCHING || candidate_frames_ > 0 ||
        time_ms < search_start_ms_ || time_ms >= search_deadline_ms_) return {};

    int direction = time_ms < search_start_ms_ + std::max(0, cfg_.follow_search_first_ms)
        ? search_direction_ : -search_direction_;
    return {0.0f, direction * std::clamp(cfg_.follow_search_steering, 0.0f, 1.0f)};
}

const char* FollowSearchController::state_name(State state) {
    switch (state) {
    case State::IDLE: return "IDLE";
    case State::STARTUP_WAIT: return "STARTUP_WAIT";
    case State::TRACKING: return "TRACKING";
    case State::SEARCHING: return "SEARCHING";
    case State::WAITING: return "WAITING";
    }
    return "UNKNOWN";
}
