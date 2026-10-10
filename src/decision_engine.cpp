#include "decision_engine.hpp"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/stat.h>

namespace {
const char* mode_name(DriveMode mode) {
    switch (mode) {
    case DriveMode::IDLE: return "IDLE";
    case DriveMode::FOLLOW: return "FOLLOW";
    case DriveMode::AUTOPILOT: return "AUTOPILOT";
    case DriveMode::MANUAL: return "MANUAL";
    }
    return "UNKNOWN";
}
}

DecisionEngine::DecisionEngine(const PipelineConfig&       cfg,
                               SafeQueue<PerceptionResult>& perception_queue,
                               SafeQueue<DriveCommand>&     command_queue,
                               RCReceiver&                  rc_receiver,
                               const std::string&           pipe_path)
    : cfg_(cfg), perception_queue_(perception_queue),
      command_queue_(command_queue), rc_receiver_(rc_receiver), pipe_path_(pipe_path),
      servo_(cfg.servo_gpio_pin),
      follow_controller_(cfg) {}

DecisionEngine::~DecisionEngine() { stop(); }

void DecisionEngine::start() {
    running_ = true;
    pipe_thread_     = std::thread(&DecisionEngine::pipe_reader_loop, this);
    decision_thread_ = std::thread(&DecisionEngine::decision_loop, this);
}

void DecisionEngine::stop() {
    running_ = false;
    if (pipe_thread_.joinable())     pipe_thread_.join();
    if (decision_thread_.joinable()) decision_thread_.join();
}

void DecisionEngine::pipe_reader_loop() {
    struct stat st;
    if (stat(pipe_path_.c_str(), &st) != 0) {
        if (mkfifo(pipe_path_.c_str(), 0666) != 0) {
            fprintf(stderr, "[T3] Failed to create FIFO %s\n", pipe_path_.c_str());
            return;
        }
    }

    fprintf(stdout, "[T3] Waiting for wake-word writer on %s...\n", pipe_path_.c_str());

    int fd = -1;
    auto open_pipe = [&]() -> bool {
        if (fd >= 0) close(fd);
        fd = open(pipe_path_.c_str(), O_RDONLY | O_NONBLOCK);
        return fd >= 0;
    };

    if (!open_pipe()) {
        fprintf(stderr, "[T3] Failed to open FIFO: %s\n", strerror(errno));
        return;
    }
    fprintf(stdout, "[T3] Wake-word pipe ready\n");

    char buf[256];
    std::string leftover;

    while (running_) {
        struct pollfd pfd = { fd, POLLIN, 0 };
        int ret = poll(&pfd, 1, 200);
        if (ret <= 0) continue;

        ssize_t n = read(fd, buf, sizeof(buf) - 1);
        if (n <= 0) {
            if (n == 0) {
                fprintf(stdout, "[T3] Pipe writer disconnected, reopening...\n");
                if (!open_pipe()) break;
                leftover.clear();
                fprintf(stdout, "[T3] Wake-word pipe reconnected\n");
            }
            continue;
        }

        buf[n] = '\0';
        leftover += buf;

        size_t pos;
        while ((pos = leftover.find('\n')) != std::string::npos) {
            std::string keyword = leftover.substr(0, pos);
            leftover.erase(0, pos + 1);

            if (keyword == "follow_me") {
                mode_request_queue_.push(DriveMode::FOLLOW);
                fprintf(stdout, "[T3] Wake word: '%s' -> request FOLLOW\n", keyword.c_str());
            } 
            
            else if (keyword == "autopilot") {
                mode_request_queue_.push(DriveMode::AUTOPILOT);
                fprintf(stdout, "[T3] Wake word: '%s' -> request search then FOLLOW\n", keyword.c_str());
            } 

            else if (keyword == "stop_engine") {
                mode_request_queue_.push(DriveMode::IDLE);
                fprintf(stdout, "[T3] Wake word: '%s' -> request IDLE\n", keyword.c_str());
            }
            
            else if (keyword == "three_sixty") {
                action_queue_.push(Action::SPIN_360);
                fprintf(stdout, "[T3] Wake word: '%s' -> action SPIN_360\n", keyword.c_str());
            } 

            else if (keyword == "turn_around") {
                action_queue_.push(Action::TURN_180);
                fprintf(stdout, "[T3] Wake word: '%s' -> action TURN_180\n", keyword.c_str());
            } 
            
            else if (keyword == "full_stop") {
                fprintf(stdout, "[T3] Wake word: '%s' -> shutting down\n", keyword.c_str());
                raise(SIGINT);
            } 
        }
    }

    if (fd >= 0) close(fd);
    fprintf(stdout, "[T3] Pipe reader stopped\n");
}

void DecisionEngine::apply_mode_request(DriveMode mode) {
    command_queue_.push(DriveCommand{});
    if (mode == DriveMode::IDLE || mode == DriveMode::MANUAL) {
        Action discarded;
        while (action_queue_.pop(discarded, 0)) {}
    }
    follow_controller_.stop();
    last_perception_ms_ = 0;
    mode_.store(mode);
    if (mode == DriveMode::FOLLOW || mode == DriveMode::AUTOPILOT) {
        // Searching for a person needs the same view used while following.
        servo_.setAngle(cfg_.angle_follow_me_mode);
        follow_controller_.start(mode == DriveMode::AUTOPILOT, now_ms());
    }
    fprintf(stdout, "[T3] Operator request -> %s\n", mode_name(mode));
    update_follow_mode();
}

void DecisionEngine::update_follow_mode() {
    auto state = follow_controller_.state();
    if (state != reported_follow_state_) {
        fprintf(stdout, "[T3] Follow state: %s -> %s\n",
                FollowSearchController::state_name(reported_follow_state_),
                FollowSearchController::state_name(state));
        reported_follow_state_ = state;
    }
    DriveMode requested = mode_selector_.requested_mode();
    if (requested != DriveMode::FOLLOW && requested != DriveMode::AUTOPILOT) return;
    if (state == FollowSearchController::State::IDLE) {
        fprintf(stdout, "[T3] Startup scan found nobody -> IDLE (request FOLLOW again or cycle RC switch)\n");
        mode_selector_.finish_startup();
        mode_.store(DriveMode::IDLE);
        command_queue_.push(DriveCommand{});
    } else {
        mode_.store(state == FollowSearchController::State::SEARCHING
            ? DriveMode::AUTOPILOT : DriveMode::FOLLOW);
    }
}

void DecisionEngine::decision_loop() {
    fprintf(stdout, "[T3] Decision engine started in IDLE\n");
    while (running_) {
        bool rc_fresh = rc_receiver_.signal_fresh(cfg_.rc_signal_timeout_ms);
        bool rc_active = rc_fresh && rc_receiver_.activate_switch_reading();
        DriveMode rc_mode = rc_fresh ? rc_receiver_.selected_drive_mode() : DriveMode::IDLE;
        if (auto request = mode_selector_.update_rc(rc_fresh, rc_active, rc_mode)) {
            apply_mode_request(*request);
        }
        DriveMode voice_mode;
        while (mode_request_queue_.pop(voice_mode, 0)) {
            if (auto request = mode_selector_.request_voice(voice_mode, rc_active, rc_mode)) {
                apply_mode_request(*request);
            } else {
                fprintf(stdout, "[T3] Voice mode request ignored: active RC switch selects %s\n",
                        mode_name(rc_mode));
            }
        }

        Action action;
        if (mode_selector_.requested_mode() != DriveMode::MANUAL && action_queue_.pop(action, 0)) {
            execute_action(action);
            continue;
        }

        // A short poll keeps operator overrides and search deadlines responsive
        // even when inference stalls. Only fresh perception permits movement.
        PerceptionResult result;
        bool observed = perception_queue_.pop(result, 20);
        int64_t time_ms = now_ms();
        DriveMode requested = mode_selector_.requested_mode();
        bool following = requested == DriveMode::FOLLOW || requested == DriveMode::AUTOPILOT;
        if (observed) {
            log_person_event(result, mode_.load());
            int64_t completed_ms = result.completed_at_ms > 0 ? result.completed_at_ms : time_ms;
            if (time_ms - completed_ms <= cfg_.follow_perception_timeout_ms) {
                if (following && last_perception_ms_ > 0 &&
                    completed_ms - last_perception_ms_ > cfg_.follow_perception_timeout_ms) {
                    follow_controller_.perception_timeout();
                }
                last_perception_ms_ = completed_ms;
                if (following) follow_controller_.observe(result, time_ms);
            }
        }
        DriveCommand cmd{};
        if (requested == DriveMode::MANUAL) {
            if (rc_receiver_.signal_fresh(cfg_.rc_signal_timeout_ms)) {
                cmd = rc_receiver_.get_drive_command();
            }
        } else if (following) {
            follow_controller_.tick(time_ms);
            update_follow_mode();
            if (last_perception_ms_ > 0 &&
                time_ms - last_perception_ms_ <= cfg_.follow_perception_timeout_ms) {
                cmd = follow_controller_.command(time_ms);
            } else {
                follow_controller_.perception_timeout();
            }
        }
        command_queue_.push(cmd);
    }

    command_queue_.push(DriveCommand{0.0f, 0.0f});
    fprintf(stdout, "[T3] Decision engine stopped\n");
}

void DecisionEngine::log_person_event(const PerceptionResult& result, DriveMode mode) {
    PersonEvent event = person_tracker_.update(result);
    if (event.count > 0) {
        const auto& person = event.target;
        fprintf(stdout,
                "[T3] Person frame in %s: count=%d, target_score=%.2f, "
                "target_center=(%d,%d), target_box=(%d,%d,%d,%d), target_depth=%.4f raw\n",
                mode_name(mode), event.count, person.score,
                (person.x1 + person.x2) / 2, (person.y1 + person.y2) / 2,
                person.x1, person.y1, person.x2, person.y2, person.depth);
    } else {
        fprintf(stdout, "[T3] Person frame in %s: count=0\n", mode_name(mode));
    }

    if (event.kind == PersonEvent::Kind::DETECTED) {
        const auto& person = event.target;
        fprintf(stdout,
                "[T3] Person detected in %s: %d person(s), score=%.2f, center=(%d,%d), depth=%.3f raw\n",
                mode_name(mode), event.count, person.score,
                (person.x1 + person.x2) / 2, (person.y1 + person.y2) / 2,
                person.depth);
    } else if (event.kind == PersonEvent::Kind::LOST) {
        fprintf(stdout, "[T3] Person no longer detected in %s\n", mode_name(mode));
    }
}

void DecisionEngine::execute_action(Action action) {
    DriveCommand cmd{0.0f, 0.0f};
    int duration_ms = 0;

    switch (action) {
    case Action::SPIN_360:
        fprintf(stdout, "[T3] Executing SPIN_360\n");
        cmd = {0.0f, 1.0f};
        duration_ms = cfg_.spin_360_duration_ms;
        break;
    case Action::TURN_180:
        fprintf(stdout, "[T3] Executing TURN_180\n");
        cmd = {0.0f, 1.0f};
        duration_ms = cfg_.turn_180_duration_ms;
        break;
    }

    const DriveMode initial_rc_mode = rc_receiver_.selected_drive_mode();
    auto start = std::chrono::steady_clock::now();
    while (running_) {
        // Explicit mode/stop requests and an RC switch change preempt actions.
        if (!mode_request_queue_.empty() ||
            (rc_receiver_.signal_fresh(cfg_.rc_signal_timeout_ms) &&
             rc_receiver_.activate_switch_reading() &&
             (rc_receiver_.selected_drive_mode() != initial_rc_mode ||
              rc_receiver_.selected_drive_mode() == DriveMode::MANUAL))) break;
        PerceptionResult observation;
        if (perception_queue_.pop(observation, 0)) {
            log_person_event(observation, mode_.load());
        }
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= duration_ms) break;
        command_queue_.push(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    command_queue_.push(DriveCommand{0.0f, 0.0f});
    DriveMode requested = mode_selector_.requested_mode();
    if (requested == DriveMode::FOLLOW || requested == DriveMode::AUTOPILOT) {
        // An action rotated the camera: discard its old target-side history.
        follow_controller_.start(requested == DriveMode::AUTOPILOT, now_ms());
        last_perception_ms_ = 0;
        update_follow_mode();
    }
    fprintf(stdout, "[T3] Action complete\n");
}
