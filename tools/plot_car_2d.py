#!/usr/bin/env python3
"""Plot the output of car_2d_demo.

Usage:  python3 tools/plot_car_2d.py car_2d.csv
Needs:  pip3 install --user matplotlib
"""
import csv
import math
import sys

import matplotlib.pyplot as plt


def main() -> None:
    path = sys.argv[1] if len(sys.argv) > 1 else "car_2d.csv"
    with open(path, newline="") as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]

    t = [r["t"] for r in rows]
    gnss = [r for r in rows if r["has_gnss"] == 1.0]
    err = [math.hypot(r["est_x"] - r["true_x"], r["est_y"] - r["true_y"]) for r in rows]
    gnss_err = [math.hypot(r["gnss_x"] - r["true_x"], r["gnss_y"] - r["true_y"]) for r in gnss]
    sigma = [math.hypot(r["sigma_x"], r["sigma_y"]) for r in rows]
    speed = [math.hypot(r["est_vx"], r["est_vy"]) for r in rows]

    fig, (ax_map, ax_err, ax_speed) = plt.subplots(1, 3, figsize=(16, 5))

    ax_map.plot([r["true_x"] for r in rows], [r["true_y"] for r in rows], "k-", label="truth")
    ax_map.plot([r["gnss_x"] for r in gnss], [r["gnss_y"] for r in gnss], "r.", label="GNSS")
    ax_map.plot([r["est_x"] for r in rows], [r["est_y"] for r in rows], "b-", label="filter")
    ax_map.set_title("Trajectory")
    ax_map.set_xlabel("x [m]")
    ax_map.set_ylabel("y [m]")
    ax_map.axis("equal")
    ax_map.legend()

    ax_err.plot([r["t"] for r in gnss], gnss_err, "r.", label="GNSS error")
    ax_err.plot(t, err, "b-", label="filter error")
    ax_err.plot(t, sigma, "b--", label="filter σ (its own estimate)")
    ax_err.set_title("Position error")
    ax_err.set_xlabel("t [s]")
    ax_err.set_ylabel("[m]")
    ax_err.legend()

    ax_speed.plot(t, speed, "b-", label="estimated speed")
    ax_speed.axhline(10.0, color="k", linestyle="--", label="true speed")
    ax_speed.set_title("Speed")
    ax_speed.set_xlabel("t [s]")
    ax_speed.set_ylabel("[m/s]")
    ax_speed.legend()

    fig.tight_layout()
    out = path.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=120)
    print(f"Saved {out}")
    plt.show()


if __name__ == "__main__":
    main()
