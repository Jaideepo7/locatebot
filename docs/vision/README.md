# Vision: LocateAnything q4_k on the Orin

**Status:** model download and engine build scripts, both tested on an x86 Linux server (CPU build). Not yet run on the Orin; no Python wrapper yet.

## Model choice
`locate-anything-q4_k.gguf` from [mudler/locate-anything.cpp-gguf](https://huggingface.co/mudler/locate-anything.cpp-gguf), pinned to revision `4e50db5`, 4.72 GB, SHA-256 `894088a0…3901da`.

- Only the language-model matrix multiplications are 4-bit; the vision encoder stays full precision, so box drift is sub-pixel (per the model card).
- It is the only build likely to fit the Orin Nano's 8 GB **shared** CPU+GPU memory next to the OS, camera and mapping. In a laptop CPU test it peaked at about 4.8 GB RAM and handled single and multi-object prompts.
- Weights are NVIDIA's under NVIDIA's licence (non-commercial research use). They are downloaded, never committed: `models/` is git-ignored.

## Set up on the Orin
```bash
git clone https://github.com/Jaideepo7/locatebot.git && cd locatebot
scripts/get_model.sh                     # ~4.7 GB, resumable, checksum-verified
export PATH=/usr/local/cuda/bin:$PATH
scripts/build_locate_anything.sh         # CUDA build for compute capability 8.7
```
Do both **before** the hackathon; the event Wi-Fi must not be needed.

## First test on the Orin
```bash
CLI=$(find third_party/locate-anything.cpp/build -name locate-anything-cli -type f | head -1)
sudo tegrastats --interval 500 &          # watch RAM and GPU while it runs
$CLI detect --model models/locate-anything-q4_k.gguf --input photo.jpg \
  --prompt "Locate all the instances that matches the following description: red box</c>water bottle." \
  --annotated out.png
```
Record for 5-10 photos of the real items taken with the car's camera: seconds per image (first run and warm), peak RAM, and whether the boxes are right. That decides how often the car can ask the model.

## Smoke test results (x86 server, 8 CPU cores, no GPU, CLI)
Photo: COCO 39769 (two cats, two TV remotes on a sofa), q4_k, default `hybrid` mode.

| Prompt items | Result | Time |
|---|---|---|
| `cat</c>remote control` | 2 cats found (plus a near-duplicate box on one); **no remotes** | ~50 s |
| `remote</c>cat` | 2 remotes found (plus a near-duplicate); **no cats** | 52 s |
| `tv remote` | 1 of 2 remotes | 53 s |

The times include loading the 4.7 GB model on every CLI call and are CPU-only, so they say nothing about the Orin's CUDA speed. Measure that on the Orin.

## Prompt rules
- Use NVIDIA's exact template: `Locate all the instances that matches the following description: X.`
- **Wording matters a lot:** `remote` found both remotes, `remote control` found none. Use short, common nouns and test each item's wording on real photos before the event.
- **Multi-item prompts are unreliable:** in both `</c>` tests above only the first-listed item was detected. Until tested on the Orin, plan for one item per prompt, or rotate the first item between photos.
- Expect near-duplicate boxes for the same object: merge same-label boxes that overlap heavily (IoU > 0.7) in the wrapper.
- Some prompts returned a whole-image box when the object was absent or ambiguous; the wrapper must reject boxes covering most of the frame.

## Python wrapper (built, tested against a fake engine)
- `locatebot/vision/engine.py`: `LocateEngine(model, lib_path)` loads the model once via the C API (ABI v1); `locate(jpeg_bytes, prompt, mode)` returns `Detection`s with pixel boxes. Calls block and can't be cancelled: run them in a background worker.
- `locatebot/vision/postprocess.py`: `build_prompt` (one item per prompt), `merge_duplicates` (same label, IoU > 0.7), `drop_oversized` (> 80 % of frame), `bearing_rad` (robot frame, left positive), `jpeg_size`.
- `tools/locate.py`: time real photos with the model loaded once:
  ```bash
  .venv/bin/python tools/locate.py photos/desk.jpg="tape measure,marker" --mode hybrid --repeat 2 --out results/run1.json
  ```
- Tests (`tests/vision/`) compile `fake_la_capi.c` into a fake `liblocate_anything.so`; they prove the binding, not the real model or the Orin GPU build.

Next: the vision daemon (`service.py`: newest-frame worker, item rotation, heartbeat).
