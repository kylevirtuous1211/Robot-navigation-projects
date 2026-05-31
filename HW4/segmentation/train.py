"""Train the HW4 instance-segmentation model (bridge / road)."""
from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO

HERE = Path(__file__).resolve().parent
DEFAULT_DATA = HERE / "data" / "data.yaml"


def load_model(name: str) -> YOLO:
    try:
        return YOLO(name)
    except Exception as exc:
        fallback = name.replace("yolo26", "yolo11")
        if fallback == name:
            raise
        print(f"[train] {name} failed ({exc}); falling back to {fallback}")
        return YOLO(fallback)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="yolo26s-seg.pt")
    ap.add_argument("--data", default=str(DEFAULT_DATA))
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default="0")
    ap.add_argument("--name", default="train")
    ap.add_argument("--project", default=str(HERE / "runs" / "segment"))
    args = ap.parse_args()

    model = load_model(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=args.device,
        project=args.project,
        name=args.name,
        exist_ok=True,
    )


if __name__ == "__main__":
    main()
