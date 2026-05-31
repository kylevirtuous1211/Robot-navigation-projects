# HW4 TODO — Object Detection + Semantic Segmentation

**Deadline:** 2026-05-03 23:59 (no late submissions)

## Dataset specs
- Detection: 2 classes — `bear`, `knob` (106 imgs → 90 train / 16 val, seed=42)
- Segmentation: 2 classes — `bridge`, `road` (106 imgs → 90 train / 16 val)
- Roboflow zips only provide `train/`; `valid/` was carved out by `scripts/prepare_data.py`. No `test/` split (too few samples).

## Commands

```bash
# (re)build dataset splits
python3 scripts/prepare_data.py

# smoke-test detection (2 epochs)
cd detection && python train.py --epochs 2 --name smoke

# full detection training
cd detection && python train.py --epochs 100

# full segmentation training
cd segmentation && python train.py --epochs 100
```

Best weights land at:
- `detection/runs/detect/train/weights/best.pt`
- `segmentation/runs/segment/train/weights/best.pt`

## Checklist

- [x] Extract Roboflow zips & build val split (`scripts/prepare_data.py`)
- [x] Write `detection/train.py` and `segmentation/train.py`
- [ ] Smoke-test detection (2 epochs) — confirm weights download and val loop
- [ ] Full detection training (100 epochs, yolo26s — fallback yolo11s)
- [ ] Full segmentation training (100 epochs, yolo26s-seg — fallback yolo11s-seg)
- [ ] Record per-class mAP (bear / knob, bridge / road) for the report
- [ ] Copy `best.pt` into `workspace/pros/ros2_yolo_integration/src/yolo_example_pkg/models/`
- [ ] Edit `object_detect.py:26` to point at the new weights
- [ ] Launch PROS Twin + Foxglove and record `detection.mp4` (30-60s) and `segmentation.mp4` (30-60s)
- [ ] Write `report.pdf` covering:
    - Roboflow project links (public)
    - Dataset config (counts, splits, classes, augmentation choices)
    - YOLO config (model size, epochs, batch)
    - Dataset diversity (text + example images)
    - Final-project navigation strategy
- [ ] Package submission: `{studentID}_HW4.zip` with `detection.pt`, `segmentation.pt`, `detection.mp4`, `segmentation.mp4`, `report.pdf`

## Grading breakdown
| Component | Weight |
|---|---|
| Detection — bear | 20% |
| Detection — knob | 20% |
| Detection — quality | 5% |
| Segmentation — road | 20% |
| Segmentation — bridge | 20% |
| Segmentation — quality | 5% |
| Report | 10% |

## Notes
- Spec wants absolute paths in `data.yaml` — already handled by `prepare_data.py`.
- Class order for segmentation is `['bridge', 'road']` (Roboflow-issued), not `['road','bridge']` as the README text implies. Labels in the zip use this ordering, so don't flip it.
- RTX 5090 / 32 GB — can push `--batch 16` (higher than spec's 8) without OOM.
