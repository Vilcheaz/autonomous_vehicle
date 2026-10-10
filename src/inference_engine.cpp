#include "inference_engine.hpp"
#include "pipeline_recorder.hpp"
#include "types.hpp"
#include <algorithm>
#include <cstdio>

static const std::vector<std::string> COCO_CLASSES = {
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"
};

// ── construction / destruction ────────────────────────────────────────────────

InferenceEngine::InferenceEngine(const PipelineConfig&       cfg,
                                 SafeQueue<cv::Mat>&          frame_queue,
                                 SafeQueue<PerceptionResult>& perception_queue,
                                 PipelineRecorder*            recorder)
    : cfg_(cfg), frame_queue_(frame_queue), perception_queue_(perception_queue),
      recorder_(recorder) {}

InferenceEngine::~InferenceEngine() { stop(); }

// ── init ──────────────────────────────────────────────────────────────────────

bool InferenceEngine::init() {
    using namespace hailort;

    // One VDevice for both models — the Hailo scheduler time-multiplexes them
    auto vd = VDevice::create();
    if (!vd) {
        fprintf(stderr, "[T2] VDevice::create failed: %d\n", (int)vd.status());
        return false;
    }
    vdevice_ = vd.release();

    // ── YOLO ─────────────────────────────────────────────────────────────────
    auto ym = vdevice_->create_infer_model(cfg_.yolo_hef);
    if (!ym) {
        fprintf(stderr, "[T2] Failed to load YOLO HEF (%s): %d\n",
                cfg_.yolo_hef.c_str(), (int)ym.status());
        return false;
    }
    yolo_model_ = ym.release();
    yolo_model_->output()->set_nms_score_threshold(cfg_.conf_threshold);
    yolo_model_->output()->set_nms_iou_threshold(cfg_.nms_iou_threshold);
    yolo_model_->output()->set_format_type(HAILO_FORMAT_TYPE_FLOAT32);

    auto yc = yolo_model_->configure();
    if (!yc) {
        fprintf(stderr, "[T2] Failed to configure YOLO: %d\n", (int)yc.status());
        return false;
    }
    yolo_configured_ = std::make_shared<ConfiguredInferModel>(yc.release());

    // ── Depth ─────────────────────────────────────────────────────────────────
    auto dm = vdevice_->create_infer_model(cfg_.depth_hef());
    if (!dm) {
        fprintf(stderr, "[T2] Failed to load depth HEF (%s): %d\n",
                cfg_.depth_hef(), (int)dm.status());
        return false;
    }
    depth_model_ = dm.release();
    depth_model_->output()->set_format_type(HAILO_FORMAT_TYPE_FLOAT32);

    auto dc = depth_model_->configure();
    if (!dc) {
        fprintf(stderr, "[T2] Failed to configure depth model: %d\n", (int)dc.status());
        return false;
    }
    depth_configured_ = std::make_shared<ConfiguredInferModel>(dc.release());

    // ── Cache I/O names, sizes, shapes ───────────────────────────────────────
    yolo_in_name_  = yolo_model_->get_input_names()[0];
    yolo_out_name_ = yolo_model_->get_output_names()[0];
    depth_in_name_  = depth_model_->get_input_names()[0];
    depth_out_name_ = depth_model_->get_output_names()[0];

    yolo_in_size_  = yolo_model_->input(yolo_in_name_)->get_frame_size();
    depth_in_size_ = depth_model_->input(depth_in_name_)->get_frame_size();

    auto depth_in_shape  = depth_model_->input(depth_in_name_)->shape();
    auto depth_out_shape = depth_model_->output(depth_out_name_)->shape();
    depth_in_h_  = depth_in_shape.height;
    depth_in_w_  = depth_in_shape.width;
    depth_out_h_ = depth_out_shape.height;
    depth_out_w_ = depth_out_shape.width;

    auto nms = yolo_model_->output(yolo_out_name_)->get_nms_shape();
    if (!nms) {
        fprintf(stderr, "[T2] Failed to get NMS shape: %d\n", (int)nms.status());
        return false;
    }
    yolo_nms_shape_ = nms.release();

    // ── Pre-allocate output buffers ───────────────────────────────────────────
    size_t yolo_out_bytes  = yolo_model_->output(yolo_out_name_)->get_frame_size();
    size_t depth_out_bytes = depth_model_->output(depth_out_name_)->get_frame_size();
    yolo_out_buf_.assign(yolo_out_bytes  / sizeof(float), 0.f);
    depth_out_buf_.assign(depth_out_bytes / sizeof(float), 0.f);

    // ── Create bindings and bind output buffers once ──────────────────────────
    // Input buffers are re-bound per frame (new pointer each iteration).
    // Output buffers persist and are read after each run().
    auto yb = yolo_configured_->create_bindings();
    if (!yb) {
        fprintf(stderr, "[T2] Failed to create YOLO bindings: %d\n", (int)yb.status());
        return false;
    }
    yolo_bindings_ = std::move(yb.release());
    yolo_bindings_->output(yolo_out_name_)->set_buffer(
        MemoryView(yolo_out_buf_.data(), yolo_out_bytes));

    auto db = depth_configured_->create_bindings();
    if (!db) {
        fprintf(stderr, "[T2] Failed to create depth bindings: %d\n", (int)db.status());
        return false;
    }
    depth_bindings_ = std::move(db.release());
    depth_bindings_->output(depth_out_name_)->set_buffer(
        MemoryView(depth_out_buf_.data(), depth_out_bytes));

    fprintf(stdout, "[T2] Ready. YOLO=%s  Depth=%s (in:%dx%d out:%dx%d)\n",
            cfg_.yolo_hef.c_str(), cfg_.depth_hef(),
            depth_in_w_, depth_in_h_, depth_out_w_, depth_out_h_);
    return true;
}

// ── thread control ────────────────────────────────────────────────────────────

void InferenceEngine::start() {
    running_ = true;
    thread_ = std::thread(&InferenceEngine::inference_loop, this);
}

void InferenceEngine::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

// ── hot loop ──────────────────────────────────────────────────────────────────

void InferenceEngine::inference_loop() {
    const auto timeout = std::chrono::milliseconds(cfg_.infer_timeout_ms);

    while (running_) {
        cv::Mat frame;
        if (!frame_queue_.pop(frame, 200)) continue;

        // ── YOLO: letterbox → RGB → bind input → run ──────────────────────
        float scale;
        int   pad_x, pad_y;
        cv::Mat yolo_in = letterbox(frame, scale, pad_x, pad_y);
        cv::cvtColor(yolo_in, yolo_in, cv::COLOR_BGR2RGB);

        yolo_bindings_->input(yolo_in_name_)->set_buffer(
            hailort::MemoryView(yolo_in.data, yolo_in_size_));

        auto ys = yolo_configured_->run(*yolo_bindings_, timeout);
        if (ys != HAILO_SUCCESS) {
            fprintf(stderr, "[T2] YOLO run failed: %d\n", (int)ys);
            continue;
        }

        // ── Depth: resize to model input → RGB → bind input → run ─────────
        cv::Mat depth_in;
        cv::resize(frame, depth_in, {depth_in_w_, depth_in_h_});
        cv::cvtColor(depth_in, depth_in, cv::COLOR_BGR2RGB);

        depth_bindings_->input(depth_in_name_)->set_buffer(
            hailort::MemoryView(depth_in.data, depth_in_size_));

        auto ds = depth_configured_->run(*depth_bindings_, timeout);
        if (ds != HAILO_SUCCESS) {
            fprintf(stderr, "[T2] Depth run failed: %d\n", (int)ds);
            continue;
        }

        // ── Decode YOLO NMS output ────────────────────────────────────────
        auto detections = decode_nms(
            yolo_out_buf_.data(), yolo_nms_shape_,
            scale, pad_x, pad_y, frame.cols, frame.rows);

        // ── Wrap depth output buffer as a Mat (no copy yet) ───────────────
        cv::Mat depth_map(depth_out_h_, depth_out_w_, CV_32FC1,
                          depth_out_buf_.data());

        // ── Fuse: sample depth at each bbox centroid ──────────────────────
        const float sx = (float)depth_out_w_ / frame.cols;
        const float sy = (float)depth_out_h_ / frame.rows;
        for (auto& det : detections) {
            int cx = (det.x1 + det.x2) / 2;
            int cy = (det.y1 + det.y2) / 2;
            det.depth = depth_map.at<float>(
                std::clamp((int)(cy * sy), 0, depth_out_h_ - 1),
                std::clamp((int)(cx * sx), 0, depth_out_w_ - 1));
        }

        // Clone depth_map so PerceptionResult owns the data independently of
        // depth_out_buf_, which will be overwritten on the next frame.
        PerceptionResult result;
        result.detections = std::move(detections);
        result.depth_map  = depth_map.clone();
        result.frame      = frame;
        result.frame_w    = frame.cols;
        result.frame_h    = frame.rows;
        result.completed_at_ms = now_ms();

        if (recorder_) {
            // Keep the decision path moving while the recorder retains its
            // own reference-counted frame/depth buffers and detection copy.
            PerceptionResult recorded_result = result;
            perception_queue_.push(std::move(result));
            if (!recorder_->enqueue(std::move(recorded_result))) break;
        } else {
            perception_queue_.push(std::move(result));
        }
    }

    fprintf(stdout, "[T2] Inference engine stopped\n");
}

// ── helpers ───────────────────────────────────────────────────────────────────

cv::Mat InferenceEngine::letterbox(const cv::Mat& src,
                                   float& scale, int& pad_x, int& pad_y) const {
    const int sz = cfg_.yolo_input_size;
    scale = std::min((float)sz / src.cols, (float)sz / src.rows);
    int new_w = (int)(src.cols * scale);
    int new_h = (int)(src.rows * scale);

    cv::Mat resized;
    cv::resize(src, resized, {new_w, new_h});

    cv::Mat padded(sz, sz, CV_8UC3, cv::Scalar(114, 114, 114));
    pad_x = (sz - new_w) / 2;
    pad_y = (sz - new_h) / 2;
    resized.copyTo(padded(cv::Rect(pad_x, pad_y, new_w, new_h)));
    return padded;
}

std::vector<Detection> InferenceEngine::decode_nms(
    const float* buf, const hailo_nms_shape_t& shape,
    float scale, int pad_x, int pad_y, int orig_w, int orig_h) const
{
    const int sz = cfg_.yolo_input_size;
    std::vector<Detection> out;
    const size_t stride = 1 + shape.max_bboxes_per_class * 5;

    for (size_t cls = 0; cls < shape.number_of_classes; ++cls) {
        const float* p = buf + cls * stride;
        int count = (int)p[0];
        for (int d = 0; d < count; ++d) {
            const float* b = p + 1 + d * 5;
            float score = b[4];
            if (score < cfg_.conf_threshold) continue;

            // Hailo NMS layout: y_min, x_min, y_max, x_max (normalised)
            int x1 = std::max(0,      (int)((b[1] * sz - pad_x) / scale));
            int y1 = std::max(0,      (int)((b[0] * sz - pad_y) / scale));
            int x2 = std::min(orig_w, (int)((b[3] * sz - pad_x) / scale));
            int y2 = std::min(orig_h, (int)((b[2] * sz - pad_y) / scale));

            std::string label = (cls < COCO_CLASSES.size())
                ? COCO_CLASSES[cls]
                : "class_" + std::to_string(cls);

            out.push_back({label, score, x1, y1, x2, y2, 0.f});
        }
    }
    return out;
}
