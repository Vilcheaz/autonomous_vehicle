#include <csignal>
#include <atomic>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <memory>
#include <string>
#include <unistd.h>
#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "safe_queue.hpp"
#include "camera_capture.hpp"
#include "inference_engine.hpp"
#include "decision_engine.hpp"
#include "motor_controller.hpp"
#include "perception_result.hpp"
#include "rc_receiver.hpp"
#include "pipeline_recorder.hpp"
#include "session_logger.hpp"

static std::atomic<bool> g_running{true};

static void on_signal(int) { g_running = false; }

namespace {
std::filesystem::path default_record_directory() {
    auto now = std::time(nullptr);
    struct tm local {};
    localtime_r(&now, &local);
    char name[64];
    strftime(name, sizeof(name), "%Y%m%d_%H%M%S", &local);
    return std::filesystem::path("results/full_pipeline") /
           (std::string(name) + "_" + std::to_string(getpid()));
}
}

int main(int argc, char** argv) {
    bool record = false;
    std::filesystem::path record_directory;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--record") {
            record = true;
        } else if (arg == "--record-dir" && i + 1 < argc) {
            record_directory = argv[++i];
            record = true;
        } else if (arg == "--help") {
            fprintf(stdout, "Usage: %s [--record] [--record-dir DIR]\n", argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown or incomplete option: %s\n", arg.c_str());
            return 2;
        }
    }

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    const PipelineConfig cfg;
    SessionLogger logger;
    std::unique_ptr<PipelineRecorder> recorder;
    if (record) {
        if (record_directory.empty()) record_directory = default_record_directory();
        try {
            std::filesystem::create_directories(record_directory);
        } catch (const std::filesystem::filesystem_error& e) {
            fprintf(stderr, "[Main] Cannot create recording directory: %s\n", e.what());
            return 1;
        }
        for (const char* name : {"pipeline.log", "annotated.mp4", "frames.csv"}) {
            if (std::filesystem::exists(record_directory / name)) {
                fprintf(stderr, "[Main] Recording file already exists: %s\n",
                        (record_directory / name).c_str());
                return 1;
            }
        }
        if (!logger.start(record_directory / "pipeline.log")) return 1;
        fprintf(stdout, "[Main] Recording session: %s\n", record_directory.c_str());
        fprintf(stdout,
                "[Main] FOLLOW controller: pd_kp=%.4f pd_kd=%.4f pi_kp=%.4f pi_ki=%.4f "
                "follow_target_depth=%.4f raw follow_invert_depth=%s\n",
                cfg.pd_kp, cfg.pd_kd, cfg.pi_kp, cfg.pi_ki,
                cfg.follow_target_depth, cfg.follow_invert_depth ? "true" : "false");
        fprintf(stdout,
                "[Main] Person search: startup_wait_ms=%d steering=%.3f first_ms=%d reverse_ms=%d "
                "center_wait_ms=%d perception_timeout_ms=%d reacquire_ramp_ms=%d\n",
                cfg.follow_startup_wait_ms, cfg.follow_search_steering,
                cfg.follow_search_first_ms, cfg.follow_search_reverse_ms,
                cfg.follow_search_center_wait_ms, cfg.follow_perception_timeout_ms,
                cfg.follow_reacquire_ramp_ms);
        recorder = std::make_unique<PipelineRecorder>(
            record_directory, cfg.camera_width, cfg.camera_height);
        if (!recorder->start()) return 1;
    }

    SafeQueue<cv::Mat>          frame_queue(cfg.frame_queue_size);
    SafeQueue<PerceptionResult> perception_queue(cfg.perception_queue_size);
    SafeQueue<DriveCommand>     command_queue(cfg.command_queue_size);

    // T1
    CameraCapture camera(cfg.camera_width, cfg.camera_height, frame_queue);

    // T2
    InferenceEngine engine(cfg, frame_queue, perception_queue, recorder.get());
    if (!engine.init()) {
        fprintf(stderr, "[Main] Inference engine init failed\n");
        return 1;
    }

    // RC receiver (MANUAL mode input)
    RCReceiver rc(cfg);
    if (!rc.init()) {
        fprintf(stderr, "[Main] RC receiver init failed\n");
        return 1;
    }

    // T3
    DecisionEngine decision(cfg, perception_queue, command_queue, rc);

    // T4
    MotorController motors(cfg, command_queue);
    if (!motors.init()) {
        fprintf(stderr, "[Main] Motor controller init failed\n");
        return 1;
    }

    camera.start();
    engine.start();
    rc.start();
    decision.start();
    motors.start();

    while (g_running && !(recorder && recorder->failed()) && !logger.failed()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (logger.failed()) {
        fprintf(stderr, "[Main] Recording log failed; stopping pipeline\n");
    }

    motors.stop();
    decision.stop();
    rc.stop();
    engine.stop();
    camera.stop();
    if (recorder) recorder->stop();

    fprintf(stdout, "[Main] Done.\n");
    return (recorder && recorder->failed()) || logger.failed() ? 1 : 0;
}
