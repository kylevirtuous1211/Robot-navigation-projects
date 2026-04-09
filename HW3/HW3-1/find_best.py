"""
Scans all save_* directories, finds the run with the highest peak mean return,
and copies its best checkpoint to ./best.pt
"""
import os
import shutil
import glob


def read_peak_return(return_txt):
    best = -float("inf")
    best_it = 0
    with open(return_txt) as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) < 2:
                continue
            it, mean_ret = int(parts[0]), float(parts[1])
            if mean_ret > best:
                best = mean_ret
                best_it = it
    return best, best_it


def main():
    save_dirs = sorted(glob.glob("save_*"))
    if not save_dirs:
        print("No save_* directories found.")
        return

    results = []
    for d in save_dirs:
        ret_file = os.path.join(d, "return.txt")
        if not os.path.exists(ret_file):
            continue
        peak, best_it = read_peak_return(ret_file)
        results.append((peak, best_it, d))
        print(f"  {d}: peak return = {peak:.2f} at iter {best_it}")

    if not results:
        print("No return.txt files found.")
        return

    results.sort(reverse=True)
    best_return, best_it, best_dir = results[0]
    print(f"\nBest run: {best_dir} (return={best_return:.2f} at iter {best_it})")

    # Try to find checkpoint closest to best_it, fall back to model.pt
    ckpt = os.path.join(best_dir, f"model-{best_it:05d}.pt")
    if not os.path.exists(ckpt):
        ckpt = os.path.join(best_dir, "model.pt")

    dest = "./best.pt"
    shutil.copy2(ckpt, dest)
    print(f"Saved best model to {dest}  (from {ckpt})")


if __name__ == "__main__":
    main()
