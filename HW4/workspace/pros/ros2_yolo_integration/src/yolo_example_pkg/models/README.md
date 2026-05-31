# YOLO26 model weights — HW4

Two freshly trained checkpoints for the HW4 detection and segmentation tasks.
Both were trained on the Roboflow-exported datasets in
`HW4/{detection,segmentation}/data/` (90 train / 16 val images, seed=42).

| File | Task | Base model | Classes |
|---|---|---|---|
| `detection.pt` | Object detection | `yolo26s.pt` | `bear`, `knob` |
| `segmentation.pt` | Instance segmentation | `yolo26s-seg.pt` | `bridge`, `road` |

Training config: 100 epochs · batch=16 · imgsz=640 · optimizer=auto (AdamW,
lr0≈1.67e-3) · device=cuda:0 (RTX 5090).

## Detection — `detection.pt`

Validated on 16 val images / 29 instances.

| Class  | P     | R     | mAP50 | mAP50-95 |
|--------|------:|------:|------:|---------:|
| all    | 0.878 | 0.927 | 0.969 |    0.684 |
| bear   | 0.899 | 1.000 | 0.995 |    0.756 |
| knob   | 0.857 | 0.854 | 0.944 |    0.613 |

Training wall-time: ~2.2 min (100 epochs). Source run:
`HW4/detection/runs/detect/train/`.

## Segmentation — `segmentation.pt`

Validated on 16 val images / 41 instances. Box = bounding-box metrics, Mask =
pixel-polygon metrics.

| Class   | Box mAP50 | Box mAP50-95 | Mask mAP50 | Mask mAP50-95 |
|---------|----------:|-------------:|-----------:|--------------:|
| all     |     0.908 |        0.763 |      0.918 |         0.717 |
| bridge  |     0.995 |        0.872 |      0.995 |         0.855 |
| road    |     0.820 |        0.653 |      0.842 |         0.578 |

Training wall-time: ~3.3 min (100 epochs). Source run:
`HW4/segmentation/runs/segment/train/`.

## Notes

- `road` has the lowest scores — it's the large, shape-variable class and the
  val set only has 16 images; expect more headroom if we add data.
- Segmentation class order is `['bridge', 'road']` (Roboflow-issued), not the
  `['road','bridge']` that the assignment README shows. Do not reorder — the
  label indices in the zip match this ordering.
- To wire a model into the ROS node, edit line 26 of
  `../yolo_example_pkg/object_detect.py` to point at `models/detection.pt` or
  `models/segmentation.pt`.

## Other files in this directory

Pre-existing weights from earlier experiments (`darth_vader.pt`, `tennis_v2.pt`,
`best_nano_auto_augu_super_close_fire.pt`, `yolo11n.pt`, `yolov8n.pt`) are
unrelated to HW4 and kept for reference.
