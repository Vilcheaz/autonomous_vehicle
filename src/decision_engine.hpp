#pragma once

#include <atomic>
#include <thread>
#include <string>

#include "types.hpp"
#include "config.hpp"
#include "safe_queue.hpp"
#include "perception_result.hpp"
#include "servo_controller.hpp"
#include "follow_search_controller.hpp"
#include "drive_mode_selector.hpp"
#include "rc_receiver.hpp"
#include "person_presence_tracker.hpp"

class DecisionEngine {
public:
    DecisionEngine(const PipelineConfig&         cfg,
                   SafeQueue<PerceptionResult>&  perception_queue,
                   SafeQueue<DriveCommand>&      command_queue,
                   RCReceiver&                   rc_receiver,
                   const std::string&            pipe_path = "/tmp/wake_word_pipe");
    ~DecisionEngine();

    void start();
    void stop();

    DriveMode mode() const { return mode_.load(); }

private:
    void pipe_reader_loop();
    void decision_loop();
    void execute_action(Action action);
    void log_person_event(const PerceptionResult& result, DriveMode mode);
    void apply_mode_request(DriveMode mode);
    void update_follow_mode();

    const PipelineConfig&        cfg_;
    SafeQueue<PerceptionResult>& perception_queue_;
    SafeQueue<DriveCommand>&     command_queue_;
    RCReceiver&                  rc_receiver_;
    std::string                  pipe_path_;

    std::atomic<bool>      running_{false};
    std::atomic<DriveMode> mode_{DriveMode::IDLE};
    SafeQueue<DriveMode>   mode_request_queue_{4};
    SafeQueue<Action>      action_queue_{4};

    // Only the decision thread changes controller, mode-selection and servo
    // state. The pipe thread submits operator requests rather than racing it.
    DriveModeSelector mode_selector_;
    PersonPresenceTracker person_tracker_;
    int64_t last_perception_ms_{0};
    FollowSearchController::State reported_follow_state_{FollowSearchController::State::IDLE};

    ServoController servo_;

    FollowSearchController follow_controller_;

    std::thread pipe_thread_;
    std::thread decision_thread_;
};
