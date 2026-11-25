# Third-Party Components and Licences

HapticSight itself has **no project licence chosen yet**; that decision is the project owner's.
This file records the licences of what the project depends on or downloads, so the choice can be
made with full information. Licence identifiers were taken from each project's published
metadata at the time of writing; verify before redistribution.

## Python dependencies (installed via pip, not vendored)

| Package | Used for | Licence | Notes |
|---------|----------|---------|-------|
| pygame-ce | simulator + live vision UI | LGPL-2.1 | dynamically linked; fine to depend on from code under most licences |
| numpy | vision arrays | BSD-3-Clause | |
| opencv-python-headless | capture, image ops, video I/O, DNN | Apache-2.0 (OpenCV); wheels bundle FFmpeg (LGPL) and other libraries | headless build avoids bundling Qt |
| onnxruntime | detector inference (CPU / CoreML) | MIT | |
| pyserial | USB serial link to the Arduino | BSD-3-Clause | |
| pytest, ruff, mypy | development only | MIT | |

## Firmware toolchain and libraries

| Component | Used for | Licence | Notes |
|-----------|----------|---------|-------|
| Arduino AVR core (`arduino:avr`) | `Serial`, `digitalWrite`, `shiftOut`, `millis`/`micros`, startup code linked into the firmware | LGPL-2.1 (core libraries) | the compiled firmware links against it; distributing binaries carries LGPL obligations (e.g. allowing relinking); source distribution of this repo is unaffected |
| avr-gcc / avr-libc | compiler and C library | GPL (compiler; output not affected), avr-libc modified BSD | toolchain, not distributed |
| arduino-cli | build/flash tool | GPL-3.0 | tool only, not distributed |

The firmware uses no third-party Arduino libraries.

## Models (downloaded on request, never committed)

| Model | Source | Licence | Training data |
|-------|--------|---------|---------------|
| YOLOX-Nano / Tiny / S (ONNX) | [Megvii-BaseDetection/YOLOX](https://github.com/Megvii-BaseDetection/YOLOX) release 0.1.1rc0 | Apache-2.0 | COCO 2017. Annotations CC BY 4.0; images are Flickr photos under their own terms |

SHA-256 checksums are pinned in `hapticsight/vision/models.py`. Ultralytics YOLO (AGPL-3.0),
YOLOv6/v7/v9 (GPL-3.0), and YOLO-NAS (non-commercial weights) were deliberately not used.

## Media

No third-party images or videos are committed. Tests and documentation screenshots use
procedurally generated synthetic scenes. The OpenCV sample video `vtest.avi` was used only
locally for benchmarking and is not included.
