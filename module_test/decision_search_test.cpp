#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <thread>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "decision_engine.hpp"

// This executable links a fake servo instead of servo_controller.cpp, and
// never creates a MotorController or opens an RC serial port/GPIO device.
static std::atomic<double> camera_angle{-1.0};
ServoController::ServoController(int gpio_pin) : handle_(-1), pin_(gpio_pin) {}
ServoController::~ServoController() {}
void ServoController::setAngle(double angle) { camera_angle.store(angle); }

namespace {
void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

PipelineConfig test_config() {
    PipelineConfig cfg;
    cfg.follow_startup_wait_ms = 200;
    cfg.follow_search_first_ms = 300;
    cfg.follow_search_reverse_ms = 600;
    cfg.follow_perception_timeout_ms = 100;
    cfg.follow_reacquire_ramp_ms = 50;
    cfg.angle_follow_me_mode = 7.0;
    cfg.angle_autopilot_mode = 32.0; // deliberately different: search must use FOLLOW
    return cfg;
}

std::string make_directory() {
    char path[] = "/tmp/decision_search_test_XXXXXX";
    char* dir = mkdtemp(path);
    check(dir != nullptr, "could not create test directory");
    return dir;
}

PerceptionResult frame(int center = -1) {
    PerceptionResult result;
    result.frame_w = 1280;
    result.frame_h = 720;
    if (center >= 0) result.detections.push_back({"person", 0.9f, center - 50, 100,
                                                center + 50, 600, 1.0f});
    return result;
}

class Fixture {
public:
    Fixture()
        : cfg(test_config()), dir(make_directory()), pipe(dir + "/wake_word_pipe"),
          perceptions(2), commands(2), rc(cfg), decision(cfg, perceptions, commands, rc, pipe) {
        check(mkfifo(pipe.c_str(), 0600) == 0, "could not create test FIFO");
        writer = open(pipe.c_str(), O_RDWR | O_NONBLOCK);
        check(writer >= 0, "could not open test FIFO");
        decision.start();
    }

    ~Fixture() {
        decision.stop();
        if (writer >= 0) close(writer);
        unlink(pipe.c_str());
        rmdir(dir.c_str());
    }

    void voice(const std::string& keyword) {
        std::string line = keyword + "\n";
        check(write(writer, line.data(), line.size()) == static_cast<ssize_t>(line.size()),
              "could not send voice request");
    }

    void publish(PerceptionResult result, bool stale = false) {
        result.completed_at_ms = now_ms() - (stale ? 1000 : 0);
        perceptions.push(std::move(result));
    }

    DriveCommand latest() {
        DriveCommand cmd;
        while (commands.pop(cmd, 0)) last_command = cmd;
        return last_command;
    }

    template<typename Predicate>
    void until(Predicate predicate, const char* message, const PerceptionResult* source = nullptr,
               bool stale = false, int timeout_ms = 2500) {
        int64_t deadline = now_ms() + timeout_ms;
        do {
            if (source) publish(*source, stale);
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
            latest();
            if (predicate()) return;
        } while (now_ms() < deadline);
        throw std::runtime_error(message);
    }

    void stay_stopped(int duration_ms, const PerceptionResult* source = nullptr, bool stale = false) {
        int64_t deadline = now_ms() + duration_ms;
        do {
            if (source) publish(*source, stale);
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
            auto cmd = latest();
            check(cmd.throttle == 0.0f && cmd.steering == 0.0f, "expected stationary commands");
        } while (now_ms() < deadline);
    }

    PipelineConfig cfg;
    std::string dir, pipe;
    SafeQueue<PerceptionResult> perceptions;
    SafeQueue<DriveCommand> commands;
    RCReceiver rc;
    DecisionEngine decision;
    int writer{-1};
    DriveCommand last_command{};
};

void test_startup_and_recovery() {
    Fixture f;
    auto empty = frame();
    auto right = frame(1000);
    auto left = frame(200);
    f.voice("follow_me");
    f.until([&] { return f.decision.mode() == DriveMode::FOLLOW; }, "voice FOLLOW was not processed");
    f.stay_stopped(100, &empty);
    f.until([&] { return f.decision.mode() == DriveMode::AUTOPILOT && f.latest().steering > 0.0f; },
            "startup did not begin a right scan", &empty);
    check(camera_angle.load() == f.cfg.angle_follow_me_mode, "search changed away from follow camera tilt");
    f.until([&] { return f.latest().steering < 0.0f; }, "startup scan did not reverse", &empty);
    f.until([&] { return f.decision.mode() == DriveMode::IDLE; }, "failed startup did not return to idle", &empty);
    f.stay_stopped(150, &right);
    check(f.decision.mode() == DriveMode::IDLE, "failed startup reacquired without an operator request");

    f.voice("follow_me");
    f.until([&] { return f.decision.mode() == DriveMode::FOLLOW && f.latest().throttle > 0.1f; },
            "startup did not acquire a person and follow", &right);
    f.until([&] { return f.decision.mode() == DriveMode::AUTOPILOT && f.latest().steering > 0.0f; },
            "two missed frames did not search toward the last right-side person", &empty);
    check(f.latest().throttle == 0.0f, "recovery used forward throttle");
    f.until([&] { return f.decision.mode() == DriveMode::FOLLOW && f.latest().steering < 0.0f; },
            "a different reappearing person did not return to FOLLOW", &left);
    f.voice("stop_engine");
    f.until([&] { return f.decision.mode() == DriveMode::IDLE && f.latest().steering == 0.0f; },
            "voice stop did not cancel following");
    f.stay_stopped(150, &left);
    fprintf(stdout, "[decision_search_test] startup, bounded scan, any-person recovery and stop passed\n");
}

void test_freshness_and_action_stop() {
    Fixture f;
    auto empty = frame();
    auto person = frame(200);
    f.voice("autopilot");
    f.until([&] { return f.decision.mode() == DriveMode::AUTOPILOT; }, "explicit autopilot did not start search");
    f.stay_stopped(150, &person, true);
    check(f.decision.mode() == DriveMode::AUTOPILOT, "stale detections acquired a person");
    f.until([&] { return f.latest().steering != 0.0f; }, "fresh perception did not permit search motion", &empty);
    f.until([&] { return f.latest().steering == 0.0f; }, "missing inference did not stop search", nullptr, false, 250);
    f.stay_stopped(50);
    f.voice("stop_engine");
    f.until([&] { return f.decision.mode() == DriveMode::IDLE; }, "stop did not cancel search");
    f.stay_stopped(100, &empty);

    f.voice("three_sixty");
    f.until([&] { return std::abs(f.latest().steering) == 1.0f; }, "legacy action did not begin");
    f.voice("stop_engine");
    f.until([&] { return f.latest().steering == 0.0f; }, "stop did not interrupt a running action", nullptr, false, 250);
    f.stay_stopped(100);
    fprintf(stdout, "[decision_search_test] stale/absent perception and action interruption passed\n");
}
}

int main() {
    try {
        test_startup_and_recovery();
        test_freshness_and_action_stop();
        fprintf(stdout, "[decision_search_test] All checks passed (FIFO and fake servo; no robot hardware used).\n");
    } catch (const std::exception& error) {
        fprintf(stderr, "[decision_search_test] FAIL: %s\n", error.what());
        return 1;
    }
}
