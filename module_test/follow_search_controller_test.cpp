#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "drive_mode_selector.hpp"
#include "follow_search_controller.hpp"

namespace {
using State = FollowSearchController::State;

void check(bool condition, const char* message) {
    if (!condition) {
        fprintf(stderr, "[follow_search_test] FAIL: %s\n", message);
        std::exit(1);
    }
}

bool stopped(const DriveCommand& cmd) {
    return cmd.throttle == 0.0f && cmd.steering == 0.0f;
}

bool close(float a, float b) { return std::abs(a - b) < 1e-5f; }

PerceptionResult frame(int center = -1, float depth = 1.0f) {
    PerceptionResult result;
    result.frame_w = 1280;
    result.frame_h = 720;
    if (center >= 0) {
        result.detections.push_back({"person", 0.9f, center - 50, 100,
                                     center + 50, 600, depth});
    }
    return result;
}

void acquire(FollowSearchController& controller, int center, int64_t time_ms) {
    controller.observe(frame(center), time_ms);
    check(stopped(controller.command(time_ms)), "first candidate must pause motion");
    controller.observe(frame(center), time_ms + 100);
    check(controller.state() == State::TRACKING, "two detections must acquire any person");
}

void test_startup() {
    PipelineConfig cfg;
    FollowSearchController controller(cfg);
    auto empty = frame();
    check(controller.state() == State::IDLE, "power-on must be idle");
    controller.observe(frame(1000), 0);
    check(controller.state() == State::IDLE, "idle must not acquire a person automatically");

    controller.start(false, 1000);
    controller.observe(empty, 10999);
    check(controller.state() == State::STARTUP_WAIT, "startup must wait a full 10 seconds");
    check(stopped(controller.command(10999)), "startup wait must be stationary");
    controller.tick(11000);
    check(controller.state() == State::SEARCHING, "startup wait must start a bounded scan");
    check(close(controller.command(11000).steering, cfg.follow_search_steering),
          "unknown startup direction must begin right");
    check(controller.command(11000).throttle == 0.0f, "search must never drive forward");
    check(controller.command(13999).steering > 0.0f, "first sweep must last its configured duration");
    check(controller.command(14000).steering < 0.0f, "scan must reverse once at the phase boundary");
    controller.tick(19999);
    check(controller.state() == State::SEARCHING, "scan must run until its total deadline");
    controller.tick(20000);
    check(controller.state() == State::IDLE, "unsuccessful startup scan must return to idle");
    check(stopped(controller.command(20000)), "failed startup must stop");
    controller.observe(frame(1000), 21000);
    controller.tick(50000);
    check(controller.state() == State::IDLE, "failed startup must remain disarmed");

    controller.start(false, 60000);
    acquire(controller, 640, 60100);
    controller.tick(80000);
    check(controller.state() == State::TRACKING, "acquisition must cancel the startup deadline");

    controller.start(false, 90000);
    controller.tick(100000);
    acquire(controller, 1000, 100100);
    controller.tick(110000);
    check(controller.state() == State::TRACKING, "a startup scan must transition to following");
    fprintf(stdout, "[follow_search_test] startup wait, scan, idle and acquisition passed\n");
}

void test_loss_and_direction() {
    PipelineConfig cfg;
    auto empty = frame();
    for (int center : {200, 1000}) {
        FollowSearchController controller(cfg);
        controller.start(false, 0);
        acquire(controller, center, 100);
        controller.observe(empty, 300);
        check(controller.state() == State::TRACKING, "one missed frame must not start searching");
        check(stopped(controller.command(300)), "first missed frame must stop both axes");
        controller.observe(frame(center), 400);
        controller.observe(empty, 500);
        check(controller.state() == State::TRACKING, "a detection must reset the missing-frame count");
        controller.observe(empty, 600);
        check(controller.state() == State::SEARCHING, "two consecutive misses must start recovery");
        int expected = center < 640 ? -1 : 1;
        check(close(controller.command(600).steering, expected * cfg.follow_search_steering),
              "recovery must turn toward the last known side");
        check(controller.command(600).throttle == 0.0f, "recovery must rotate in place");
        check(close(controller.command(3600).steering, -expected * cfg.follow_search_steering),
              "recovery must sweep back in the opposite direction");
        controller.tick(9600);
        check(controller.state() == State::WAITING, "failed recovery must wait stationary for reappearance");
        check(stopped(controller.command(9600)), "failed recovery must stop");
        controller.tick(50000);
        check(controller.state() == State::WAITING, "failed recovery must not restart scanning");
        acquire(controller, center == 200 ? 1000 : 200, 50100);
        check(controller.state() == State::TRACKING, "any reappearing person must be accepted");
    }

    FollowSearchController centered(cfg);
    centered.start(false, 0);
    acquire(centered, 640, 100);
    centered.observe(empty, 300);
    centered.observe(empty, 400);
    check(stopped(centered.command(899)), "centered loss must pause for possible occlusion");
    check(centered.command(900).steering > 0.0f, "centered loss must eventually start a deliberate sweep");
    fprintf(stdout, "[follow_search_test] two-frame loss, last-side search and bounded recovery passed\n");
}

void test_confirmation_reset_and_follow() {
    PipelineConfig cfg;
    FollowSearchController controller(cfg);
    controller.start(true, 1000);
    check(controller.state() == State::SEARCHING, "explicit autopilot must scan immediately");
    check(stopped(controller.command(1000)), "no inference must not permit search motion");
    controller.observe(frame(), 1000);
    controller.observe(frame(200), 1100);
    check(stopped(controller.command(1100)), "a search candidate must stop rotation immediately");
    controller.observe(frame(), 1200);
    check(controller.command(1200).steering > 0.0f, "one false candidate must not cancel the scan");
    acquire(controller, 1000, 1300);
    controller.observe(frame(), 1500);
    controller.observe(frame(), 1600);
    check(controller.command(1600).steering > 0.0f, "a later recovery must remember the new target side");

    controller.stop();
    check(stopped(controller.command(1700)), "operator stop must cancel recovery");
    controller.start(true, 1800);
    controller.observe(frame(), 1800);
    check(controller.command(1800).steering > 0.0f, "a new session must forget the old target side");
    controller.observe(frame(200), 1900);
    controller.perception_timeout();
    check(stopped(controller.command(1900)), "inference timeout must stop search motion");
    controller.observe(frame(200), 2000);
    check(controller.state() == State::SEARCHING, "missing inference must break acquisition confirmation");
    controller.observe(frame(200), 2100);
    check(controller.state() == State::TRACKING, "fresh consecutive results must reacquire");
    controller.observe(frame(), 2200);
    controller.perception_timeout();
    controller.observe(frame(), 2300);
    check(controller.state() == State::TRACKING, "an inference timeout must not count as a missed person");
    controller.observe(frame(), 2400);
    check(controller.state() == State::SEARCHING, "two real misses after a timeout must recover");

    controller.start(false, 3000);
    auto multiple = frame(1000, cfg.follow_target_depth - 0.5f);
    multiple.detections.push_back({"person", 0.99f, 150, 100, 200, 200, 20.0f});
    controller.observe(multiple, 3100);
    controller.observe(multiple, 3200);
    check(controller.command(3200).throttle == 0.0f, "throttle must start gently after acquisition");
    controller.observe(multiple, 3450);
    check(close(controller.command(3450).throttle, cfg.pi_kp * 0.5f * 0.5f),
          "reacquisition ramp must ease in the existing throttle controller");
    controller.observe(multiple, 3700);
    check(close(controller.command(3700).throttle, cfg.pi_kp * 0.5f),
          "normal throttle control must be restored after the ramp");
    check(close(controller.command(3700).steering, cfg.pd_kp * (1000.0f - 640.0f) / 640.0f),
          "following must continue choosing the largest person");
    controller.observe(frame(), 3800);
    controller.observe(frame(), 3900);
    check(controller.command(3900).steering > 0.0f, "history must belong to the same largest person followed by PID");
    controller.stop();
    controller.start(true, 4000);
    controller.observe(frame(200), 4100);
    controller.observe(frame(200), 4200);
    controller.observe(frame(), 4300);
    controller.observe(frame(), 4400);
    check(controller.command(4400).steering < 0.0f, "left target must be remembered");
    controller.start(true, 4500);
    controller.observe(frame(), 4500);
    check(controller.command(4500).steering > 0.0f, "restart must clear left-side memory");
    fprintf(stdout, "[follow_search_test] confirmation, reset, any-person selection and PID behavior passed\n");
}

void test_mode_selection() {
    DriveModeSelector selector;
    check(!selector.update_rc(false, false, DriveMode::FOLLOW), "absent RC must not change idle");
    check(!selector.update_rc(true, false, DriveMode::FOLLOW), "disabled RC switch must allow voice control");
    check(selector.request_voice(DriveMode::FOLLOW, false, DriveMode::IDLE) == DriveMode::FOLLOW,
          "voice must activate follow with no active RC authority");
    check(selector.update_rc(true, true, DriveMode::FOLLOW) == DriveMode::FOLLOW,
          "activating the RC switch must request its selected mode");
    check(!selector.update_rc(true, true, DriveMode::FOLLOW), "held FOLLOW must not reset recovery");
    selector.finish_startup();
    for (int i = 0; i < 100; ++i) {
        check(!selector.update_rc(true, true, DriveMode::FOLLOW), "held FOLLOW must not restart failed startup");
    }
    check(selector.requested_mode() == DriveMode::IDLE, "startup failure must disarm the request");
    check(!selector.update_rc(false, false, DriveMode::FOLLOW), "link dropout must preserve idle");
    check(!selector.update_rc(true, true, DriveMode::FOLLOW), "link restoration must not restart failed startup");
    check(selector.request_voice(DriveMode::FOLLOW, true, DriveMode::FOLLOW) == DriveMode::FOLLOW,
          "explicit repeated FOLLOW must re-arm startup");
    check(!selector.request_voice(DriveMode::AUTOPILOT, true, DriveMode::FOLLOW),
          "conflicting voice mode must respect an active RC switch");
    check(selector.request_voice(DriveMode::IDLE, true, DriveMode::FOLLOW) == DriveMode::IDLE,
          "voice stop must always cancel motion");
    check(!selector.update_rc(true, true, DriveMode::FOLLOW), "held RC must not override voice stop");
    check(selector.update_rc(true, true, DriveMode::AUTOPILOT) == DriveMode::AUTOPILOT,
          "changing the RC switch must start an explicit search");
    check(!selector.update_rc(true, true, DriveMode::AUTOPILOT), "held AUTOPILOT must allow transition to FOLLOW");
    check(selector.update_rc(true, true, DriveMode::MANUAL) == DriveMode::MANUAL, "MANUAL must override search");
    check(selector.update_rc(false, false, DriveMode::MANUAL) == DriveMode::IDLE, "manual link loss must stop");
    check(!selector.update_rc(true, true, DriveMode::MANUAL), "restored link must not resume held MANUAL");
    selector.update_rc(true, false, DriveMode::MANUAL);
    check(selector.update_rc(true, true, DriveMode::MANUAL) == DriveMode::IDLE,
          "activation toggle alone must not bypass manual link-loss latch");
    check(selector.update_rc(true, true, DriveMode::FOLLOW) == DriveMode::FOLLOW, "leaving MANUAL must re-arm it");
    check(selector.update_rc(true, true, DriveMode::MANUAL) == DriveMode::MANUAL, "cycled MANUAL must work again");
    fprintf(stdout, "[follow_search_test] RC authority, startup latch, stop and manual failsafe passed\n");
}
}

int main() {
    test_startup();
    test_loss_and_direction();
    test_confirmation_reset_and_follow();
    test_mode_selection();
    fprintf(stdout, "[follow_search_test] All checks passed (no robot hardware used).\n");
}
