"""COCO-80 class labels and the default obstacle-relevant subset.

Generic COCO detectors do **not** know about walls, poles, curbs, stairs,
doors, glass, holes, or low/overhanging obstacles. See docs/COMPUTER_VISION.md.
"""

from __future__ import annotations

COCO_LABELS: tuple[str, ...] = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
)  # fmt: skip

# Objects a walking person may plausibly collide with at body height.
OBSTACLE_CLASSES: frozenset[str] = frozenset(
    {
        "person", "bicycle", "car", "motorcycle", "bus", "train", "truck",
        "fire hydrant", "stop sign", "parking meter", "traffic light", "bench",
        "dog", "horse", "cow", "sheep", "cat", "suitcase", "backpack", "umbrella",
        "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv",
        "refrigerator", "oven", "sink", "skateboard",
    }
)  # fmt: skip

assert len(COCO_LABELS) == 80
assert set(COCO_LABELS) >= OBSTACLE_CLASSES
