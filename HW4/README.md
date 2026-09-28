# HW4 - Object Detection and Instance Segmentation

Two YOLO26s models trained on hand-labeled frames from the PROS Twin Unity simulator (106 frames: 90 train, 16 validation): a detector for `bear` and `knob`, and an instance segmenter for `road` and `bridge`.
The same weights drive the Final Project's perception.

![Detection (top) and segmentation (bottom) on validation frames](assets/val_examples.jpg)

*Validation frames the models never trained on: bear and door-knob detection (top), bridge and road segmentation (bottom).*

## Results

Validation split (16 images), evaluated with the `best.pt` weights:

| Model | Class | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---:|---:|---:|---:|
| Detection | bear | 0.996 | 1.000 | 0.995 | 0.747 |
| Detection | knob | 0.849 | 0.857 | 0.944 | 0.599 |
| Detection | **all** | 0.923 | 0.929 | **0.969** | 0.673 |
| Segmentation (mask) | bridge | 0.934 | 1.000 | 0.995 | 0.842 |
| Segmentation (mask) | road | 0.956 | 0.706 | 0.850 | 0.563 |
| Segmentation (mask) | **all** | 0.945 | 0.853 | **0.923** | 0.702 |

The `knob` is small in most frames, which shows up as the lowest detection mAP50-95.
`road` is the weakest class (recall 0.71, mask mAP50-95 0.56): the confusion matrix below shows about a fifth of true road instances missed, and most false positives are also road.
With no separate test split, these numbers come from the same 16 frames that picked `best.pt`, so treat them as optimistic.

### Validation predictions

All 16 validation frames, ground truth next to prediction.

| Ground truth | Prediction |
|---|---|
| ![Detection labels](assets/det_val_labels.jpg) | ![Detection predictions](assets/det_val_pred.jpg) |
| ![Segmentation labels](assets/seg_val_labels.jpg) | ![Segmentation predictions](assets/seg_val_pred.jpg) |

### Training curves

**Detection**

![Detection training curves](assets/det_results.png)

**Segmentation**

![Segmentation training curves](assets/seg_results.png)

| Detection PR curve | Detection confusion matrix |
|---|---|
| ![Detection PR curve](assets/det_pr_curve.png) | ![Detection confusion matrix](assets/det_confusion_matrix.png) |
| **Segmentation mask PR curve** | **Segmentation confusion matrix** |
| ![Segmentation mask PR curve](assets/seg_mask_pr_curve.png) | ![Segmentation confusion matrix](assets/seg_confusion_matrix.png) |

The PR curves and confusion matrices come from the same validation run as the table; the training curves come from training.

Dataset, training configuration and the Final Project navigation plan are in [`report.md`](report.md) ([PDF](report.pdf)).

## Reproduce the metrics

The datasets and weights are not committed.
Download both Roboflow projects linked in the [report](report.md) in YOLO26 format, and save them as `HW4/data/det_yolo26.zip` and `HW4/data/seg_yolo26.zip`.

```bash
cd HW4
uv venv && uv pip install ultralytics
.venv/bin/python scripts/prepare_data.py            # extract the zips, carve a 15% validation split (seed 42)
(cd detection && ../.venv/bin/python train.py)      # yolo26s, 100 epochs, batch 16, GPU 0
(cd segmentation && ../.venv/bin/python train.py)   # yolo26s-seg, same settings
.venv/bin/yolo val model=detection/runs/detect/train/weights/best.pt data=detection/data/data.yaml device=cpu
.venv/bin/yolo val model=segmentation/runs/segment/train/weights/best.pt data=segmentation/data/data.yaml device=cpu
```

---

The rest of this README is the working guide for the simulator, data collection and training.

## Quick Start

### 1. Launch PROS Twin Simulator

The PROS Twin build (`pros_twin_unity_linux_V3.zip`) comes from the course and is git-ignored; unzip it to `HW4/pros_twin_linux/`.
Force NVIDIA GPU rendering, which is required because Chrome Remote Desktop defaults to software rendering:

```bash
cd ~/Desktop/Robot-navigation-projects/HW4/pros_twin_linux/pros_twin_unity_linux/
__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
__VK_LAYER_NV_optimus=NVIDIA_only \
./pros_twin_unity_tsai_run_linux.x86_64 -force-vulkan
```

Verify the GPU is being used (in another terminal while PROS Twin is running):
```bash
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv
# Should list pros_twin_unity_tsai_run_linux
```

Login with your class username and key, then:

1. Select **EASY** mode
2. Choose **FINAL PROJECT** scene (may take a moment to load)
3. Switch from **MANUAL** (Unity) to **AI** (ROS) mode

Controls in the simulator:
- Right mouse button: change view direction
- WASD keys: move viewpoint
- Mouse wheel: zoom in/out
- Click "FINAL PROJECT" tab to reset the scene (randomly regenerated)
- LiDAR visualization can be toggled off under SETTINGS > Render > Lidar

### 2. Launch PROS App (Terminal 1)

```bash
cd ~/Desktop/Robot-navigation-projects/HW4/workspace/pros/pros_app/
python3 ./control.py -s
# Select "slam_unity.sh"
# Keep this terminal open
```

### 3. Launch Foxglove

Open https://app.foxglove.dev in Chrome, then:

1. Click **Open connection**
2. Select **Rosbridge**
3. Enter URL: `ws://localhost:9090`
4. Click **Open**

You should see the car's camera feed (depth + RGB).

### 4. Collect Data (Terminal 2)

```bash
cd ~/Desktop/Robot-navigation-projects/HW4/workspace/pros/pros_car/
./car_control.sh
r
ros2 run pros_car_py robot_control
# Select "Manual Arm Control"
# Select "[0]"
# Press b to reset the arm to its initial pose
# Press q to quit arm control
# Select "Control Vehicle"
```

Vehicle controls:
| Key | Action |
|-----|--------|
| w | Forward |
| s | Backward |
| a | Forward-left arc |
| d | Forward-right arc |
| e | Rotate left in place |
| r | Rotate right in place |
| z | Stop |
| c | **Save current image** |
| q | Quit |

Images are saved to `HW4/workspace/pros/pros_car/src/pros_car_py/images/`

Speed config: `HW4/workspace/pros/pros_car/src/pros_car_py/pros_car_py/ros_communicator_config.py`

## Data Labeling (Roboflow)

Go to https://roboflow.com/ and create **two** projects (use random names for anti-plagiarism):

| Project | Type | Classes |
|---------|------|---------|
| Detection | Object Detection | `bear`, `knob` |
| Segmentation | Instance Segmentation | `road`, `bridge` |

Steps per project:
1. Create Project > set visibility to **Public**
2. Define classes under **Classes & Tags**
3. Upload images via **Upload Data**
4. **Annotate** > Label Myself
   - Detection: draw bounding boxes
   - Segmentation: draw polygon areas (or use auto-labeling)
   - If no target object in image, mark as null
5. Add annotated images to dataset (assign to train/valid/test)
6. **Versions** > remove Auto-Orient and Resize preprocessing > skip Augmentation > **Create**
7. **Download Dataset** > format: **YOLO26** > Download zip

## Training

### Directory Structure

`scripts/prepare_data.py` builds this layout from the two Roboflow zips and writes `data.yaml` with absolute paths (the Roboflow exports contain only `train/`, so it carves `valid/` out of it):

```
HW4/
  data/det_yolo26.zip, data/seg_yolo26.zip   # Roboflow exports (git-ignored)
  detection/
    data/
      train/images/  train/labels/
      valid/images/  valid/labels/
      data.yaml
    train.py
  segmentation/
    data/
      ...same structure...
    train.py
```

### train.py

Both `train.py` scripts take `--model`, `--epochs`, `--batch`, `--imgsz`, `--device` and `--name`; the defaults are the settings used for the results above.

```bash
cd detection && ../.venv/bin/python train.py        # yolo26s.pt, 100 epochs, batch 16, device 0
# Best weights saved to: detection/runs/detect/train/weights/best.pt
```

## Testing with ROS

1. Copy your `.pt` model to:
   `HW4/workspace/pros/ros2_yolo_integration/src/yolo_example_pkg/models/`
2. Edit model path at line 26 of:
   `HW4/workspace/pros/ros2_yolo_integration/src/yolo_example_pkg/yolo_example_pkg/object_detect.py`
3. Run:
   ```bash
   cd HW4/workspace/pros/ros2_yolo_integration/
   ./yolo_activate.sh
   r
   ros2 run yolo_example_pkg yolo_node
   ```
4. View results in Foxglove

## Submission

**Deadline: 2026/05/03 23:59** (no late submissions)

File: `{studentID}_HW4.zip` containing:

| File | Description |
|------|-------------|
| `detection.pt` | Trained object detection model |
| `segmentation.pt` | Trained segmentation model |
| `detection.mp4` | 30-60s Foxglove demo video |
| `segmentation.mp4` | 30-60s Foxglove demo video |
| `report.pdf` | Report (English or Mandarin) |

10 points deducted per missing file or incorrect filename.

### Report Contents
- Two Roboflow project links
- Dataset config (# images, categories, train/test split, augmentation)
- YOLO training config (model type, # epochs)
- Description of dataset diversity (text + example images)
- Navigation strategy for the final project (text, figures, algorithm)

## Grading

| Component | Weight |
|-----------|--------|
| Object Detection - Bear | 20% |
| Object Detection - Knob | 20% |
| Object Detection - Quality | 5% |
| Semantic Segmentation - Road | 20% |
| Semantic Segmentation - Bridge | 20% |
| Semantic Segmentation - Quality | 5% |
| Report | 10% |
