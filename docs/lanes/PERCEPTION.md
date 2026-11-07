# Lane C — asynchronous target perception

**Status: planned only. Branch: feat/perception. Owner: unassigned.**
Allowed future files: rc_robot/perception.py, tests/test_perception.py, docs/lanes/perception.md. No motor or mission edits, model download or custom quantization in the initial batch.

## Deliverable
Thread-safe latest observation store and bounded VisionWorker using an injectable callable. Actual LocateAnything loading and performance are explicitly deferred. No fake backend output may be presented as real model inference.

## P1 — Latest observation store
- [ ] Test valid observation publication and retrieval.
- [ ] Test old/equal timestamp cannot replace newer result.
- [ ] Test invalid ID, confidence, bearing or timestamp rejected.
- [ ] Test future/expired observation and invalid age/query time return no usable target.
- [ ] Observe red; implement immutable handoff under short locks.
- [ ] Run lane/full suites, review, commit tested store slice.
Acceptance: freshness uses frame capture time. Inference completion does not make old imagery fresh.

## P2 — Nonblocking bounded worker
- [ ] Use synchronization Events to hold inference in flight, then verify submit/store read do not wait for it.
- [ ] Submit multiple pending frames while inference is blocked; verify only the newest pending one is next processed.
- [ ] Test inference exceptions are visible through last_error and do not block other modules.
- [ ] Test defined start/submit/close lifecycle, shutdown timeout and no publication after close.
- [ ] Observe red; implement one in-flight job and one pending latest slot.
- [ ] Run complete suite, review lock/shutdown behaviour, commit.
Acceptance: no lock spans model execution; memory cannot grow with incoming frame rate; hung inference must not prevent process exit. Thread timeout does not cancel arbitrary GPU/native inference; document this limit and prefer process isolation later if needed.

## P3 — Actual model selection and benchmark, later
1. Verify supported LocateAnything backend on installed Jetson software.
2. Review model license for intended use and record source/version.
3. Use locally captured consented demo images and explicit target prompts.
4. Measure model load success, memory peak, latency distribution and detection quality on actual Jetson.
5. Confirm camera capture timestamps, confidence semantics and output coordinate conversion.
6. Retain lightweight marker/known-target fallback if performance is insufficient.
Do not promise sub-second output, assume quantization compatibility or port Apple MLX conversions onto Jetson. The first milestone needs a robust adapter, not custom optimization research.

## P4 — Spatial localization, later
Image boxes/bearings require calibrated camera intrinsics/extrinsics and appropriate depth or known landmark geometry before becoming map goals. A 2D LiDAR plane may not intersect the observed target. Track uncertainty and reject ambiguous/multiple candidates; never invent distance.

## Test names
- test_older_observation_cannot_replace_latest
- test_expired_or_future_target_is_unavailable
- test_blocked_inference_does_not_block_submit_or_read
- test_newest_pending_frame_wins
- test_inference_error_is_observable
- test_close_forbids_late_publication

## Handoff
Exact commits/tests, queue/lock/shutdown policy, model backend actually used (initially none), performance measurements only if observed, and next localization/benchmark task.
