"""Extract HW4 Roboflow zips and build train/valid splits with absolute-path data.yaml.

The Roboflow exports only contain a `train/` split, so we carve a validation set
(default 15%) out of train with a deterministic seed for reproducibility.
"""
from __future__ import annotations

import argparse
import random
import shutil
import zipfile
from pathlib import Path

HW4 = Path(__file__).resolve().parent.parent
DATASETS = {
    "detection": HW4 / "data" / "det_yolo26.zip",
    "segmentation": HW4 / "data" / "seg_yolo26.zip",
}


def extract(zip_path: Path, dest: Path) -> None:
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)
    with zipfile.ZipFile(zip_path) as zf:
        zf.extractall(dest)


def split_train_val(data_dir: Path, val_frac: float, seed: int) -> tuple[int, int]:
    train_img = data_dir / "train" / "images"
    train_lbl = data_dir / "train" / "labels"
    valid_img = data_dir / "valid" / "images"
    valid_lbl = data_dir / "valid" / "labels"
    valid_img.mkdir(parents=True, exist_ok=True)
    valid_lbl.mkdir(parents=True, exist_ok=True)

    images = sorted(p for p in train_img.iterdir() if p.suffix.lower() in {".jpg", ".jpeg", ".png"})
    rng = random.Random(seed)
    rng.shuffle(images)
    n_val = max(1, int(round(len(images) * val_frac)))
    val_set = images[:n_val]

    for img in val_set:
        lbl = train_lbl / (img.stem + ".txt")
        img.rename(valid_img / img.name)
        if lbl.exists():
            lbl.rename(valid_lbl / lbl.name)
    return len(images) - n_val, n_val


def write_yaml(data_dir: Path, names: list[str]) -> None:
    yaml_path = data_dir / "data.yaml"
    content = (
        f"train: {data_dir / 'train' / 'images'}\n"
        f"val: {data_dir / 'valid' / 'images'}\n"
        f"\n"
        f"nc: {len(names)}\n"
        f"names: {names}\n"
    )
    yaml_path.write_text(content)


def read_names_from_roboflow_yaml(data_dir: Path) -> list[str]:
    import re

    text = (data_dir / "data.yaml").read_text()
    m = re.search(r"names:\s*\[(.*?)\]", text)
    if not m:
        raise ValueError(f"Could not parse names from {data_dir/'data.yaml'}")
    return [n.strip().strip("'\"") for n in m.group(1).split(",")]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--val-frac", type=float, default=0.15)
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    for task, zip_path in DATASETS.items():
        dest = HW4 / task / "data"
        print(f"[{task}] extracting {zip_path.name} -> {dest}")
        extract(zip_path, dest)
        names = read_names_from_roboflow_yaml(dest)
        n_train, n_val = split_train_val(dest, args.val_frac, args.seed)
        write_yaml(dest, names)
        print(f"[{task}] names={names}  train={n_train}  val={n_val}")
        print(f"[{task}] data.yaml -> {dest/'data.yaml'}")


if __name__ == "__main__":
    main()
