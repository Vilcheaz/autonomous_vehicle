<img src="docs/architecture.svg" width="800">

Colab for training wake word ONNX models

https://colab.research.google.com/drive/1q1oe2zOyZp7UsB3jJiQ1IFn8z5YfjwEb#scrollTo=qgaKWIY6WlJ1

## Follow-me and person search

`follow_me` (or the RC FOLLOW position) starts a follow session. The car waits
stationary for up to 10 seconds for a person. If nobody appears, AUTOPILOT
performs one slow, bounded in-place scan: right for 3 seconds, then left for
6 seconds. If that startup scan finds nobody, the car returns to IDLE. A held
RC switch does not restart it: repeat `follow_me`, change the mode switch, or
disable and re-enable RC mode selection to begin a new session.

While following, one missed person frame stops movement. Two consecutive
empty inference results start AUTOPILOT recovery toward the largest person's
last known side, then sweep in the opposite direction. A person last seen
near the center gets a short 500 ms pause before the scan. After an unsuccessful
recovery scan, the car waits stationary for a person to reappear; it does not
keep rotating. Any person can be acquired, and the largest bounding box still
determines the target. Two consecutive person results confirm acquisition;
the first result pauses rotation, and confirmed acquisition returns to FOLLOW
with a short throttle ramp.

The `autopilot` voice command and RC AUTOPILOT position request an immediate
scan followed by FOLLOW, with no initial 10-second wait. VFH+ is no longer used
by the driving pipeline. Both follow and search use the follow-me camera tilt.
Search commands have zero throttle. Missing/stale inference stops autonomous
movement, and operator mode changes cancel the search. `stop_engine` also
stops when the RC switch is held in an autonomous position; other voice mode
requests respect an enabled, live RC mode switch. MANUAL link loss still
requires physically leaving MANUAL before it can resume.

Tune `follow_search_steering`, `follow_search_first_ms`, and
`follow_search_reverse_ms` in `src/config.hpp` on the real car. The defaults
are 0.25 steering and a 3/6-second sweep; they are timed rotations, not measured
angles. Startup wait, center pause, inference freshness limit, and throttle
ramp are also configurable there. Search state transitions are written to
`pipeline.log` during recording. Select `14) Follow search / recovery test`
in `./run.sh`, or run `ctest --test-dir build --output-on-failure`, for tests
that do not access robot hardware.

## Full pipeline recording

Run `./run.sh`, choose `11) Full Pipeline`, and answer `y` to the recording
prompt. Alternatively, run `./build/pipeline --record` after building. Each
recorded run creates a unique directory under `results/full_pipeline/` with:

- `annotated.mp4`: every frame that completed object and depth inference,
  with detection boxes and a relative-depth panel. It is H.264 video rather
  than a directory of PNGs.
- `frames.csv`: each video's zero-based frame index and its processing time
  in Unix milliseconds. The video plays at a fixed 10 fps; use the CSV for
  actual timing if inference ran faster or slower.
- `pipeline.log`: timestamped output from the camera, inference, decision,
  motor, and RC stages, including stderr. T3 reports when a person appears or
  disappears in any drive mode. T4 reports meaningful command changes and a
  watchdog stop once per loss of active motion.
- `wake_word.log`: Python wake-word output when launched through `run.sh`.

Recording requires OpenCV with an H.264 (`avc1`) encoder. If the video cannot
be opened, the pipeline exits before motors start. The recorder uses a bounded
queue and slows inference if encoding cannot keep up, so frames that finish
inference are not discarded by the recorder. Camera frames may still be
dropped by the existing capture queue when inference is slower than capture.

To check the recorder without robot hardware, run `./run.sh` and select
`12) Pipeline recorder test`. The test prints the temporary directory holding
its sample video, timestamps, and log.
Option `13) Person presence event test` checks detection, brief missed frames,
loss, and reacquisition without robot hardware.
