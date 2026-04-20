#!/usr/bin/env python3
"""
Plot joint PD tuning logs.

This script supports both formats used by p73 controller logging:
1) Single-joint mode (1 column per line)
2) Multi-joint mode (N columns per line)

Expected default files under ./data:
- joint_desired_log.txt
- joint_position_log.txt
- joint_velocity_log.txt (optional)
- torque_joint_log.txt (optional)
- torque_motor_log.txt (optional)
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np


def load_text_matrix(path: Path) -> Optional[np.ndarray]:
    """Load whitespace-separated numeric text into shape [T, C]."""
    if not path.exists():
        return None

    rows = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            rows.append([float(v) for v in parts])

    if not rows:
        return None

    ncol = len(rows[0])
    for i, r in enumerate(rows):
        if len(r) != ncol:
            raise ValueError(
                f"Inconsistent column count in {path} at row {i}: "
                f"expected {ncol}, got {len(r)}"
            )

    return np.asarray(rows, dtype=float)


def slice_joint(data: np.ndarray, joint_index: int) -> np.ndarray:
    """Return 1D signal for selected joint, or the single column if C==1."""
    if data.shape[1] == 1:
        return data[:, 0]
    if not (0 <= joint_index < data.shape[1]):
        raise IndexError(
            f"joint_index={joint_index} out of range for {data.shape[1]} columns"
        )
    return data[:, joint_index]


def split_torque_columns(
    torque_data: np.ndarray, joint_index: int
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """
    Parse torque log for one joint.

    Returns:
    - primary: desired or single torque signal
    - secondary: measured torque if log stores desired|measured layout
    """
    ncol = torque_data.shape[1]

    if ncol == 1:
        return torque_data[:, 0], None

    if ncol % 2 == 0:
        half = ncol // 2
        if joint_index >= half:
            raise IndexError(
                f"joint_index={joint_index} out of range for torque half-size {half}"
            )
        return torque_data[:, joint_index], torque_data[:, half + joint_index]

    if not (0 <= joint_index < ncol):
        raise IndexError(f"joint_index={joint_index} out of range for {ncol} columns")
    return torque_data[:, joint_index], None


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Plot joint PD logs")
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=script_dir / "data",
        help="Directory containing log txt files",
    )
    parser.add_argument(
        "--joint-index",
        type=int,
        default=5,
        help="Joint index for multi-column logs (0-based)",
    )
    parser.add_argument("--dt", type=float, default=0.001, help="Control period [s]")
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Output image path (default: <data-dir>/jointpd_plot.png)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open interactive window",
    )
    args = parser.parse_args()

    data_dir: Path = args.data_dir
    joint_desired = load_text_matrix(data_dir / "joint_desired_log.txt")
    joint_position = load_text_matrix(data_dir / "joint_position_log.txt")
    joint_velocity = load_text_matrix(data_dir / "joint_velocity_log.txt")
    torque_joint = load_text_matrix(data_dir / "torque_joint_log.txt")
    torque_motor = load_text_matrix(data_dir / "torque_motor_log.txt")

    if joint_desired is None or joint_position is None:
        raise FileNotFoundError(
            "joint_desired_log.txt and joint_position_log.txt are required and non-empty"
        )

    q_des = slice_joint(joint_desired, args.joint_index)
    q_cur = slice_joint(joint_position, args.joint_index)

    signals = [q_des, q_cur]
    if joint_velocity is not None:
        qdot = slice_joint(joint_velocity, args.joint_index)
        signals.append(qdot)
    else:
        qdot = None

    if torque_joint is not None:
        tau_joint_a, tau_joint_b = split_torque_columns(torque_joint, args.joint_index)
        signals.append(tau_joint_a)
        if tau_joint_b is not None:
            signals.append(tau_joint_b)
    else:
        tau_joint_a, tau_joint_b = None, None

    if torque_motor is not None:
        tau_motor_a, tau_motor_b = split_torque_columns(torque_motor, args.joint_index)
        signals.append(tau_motor_a)
        if tau_motor_b is not None:
            signals.append(tau_motor_b)
    else:
        tau_motor_a, tau_motor_b = None, None

    n = min(len(s) for s in signals if s is not None)
    if n <= 0:
        raise RuntimeError("No valid samples to plot")

    t = np.arange(n, dtype=float) * args.dt
    q_des = q_des[:n]
    q_cur = q_cur[:n]
    qdot = qdot[:n] if qdot is not None else None
    tau_joint_a = tau_joint_a[:n] if tau_joint_a is not None else None
    tau_joint_b = tau_joint_b[:n] if tau_joint_b is not None else None
    tau_motor_a = tau_motor_a[:n] if tau_motor_a is not None else None
    tau_motor_b = tau_motor_b[:n] if tau_motor_b is not None else None

    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    fig.suptitle(f"Joint PD Log Plot (joint index {args.joint_index})", fontsize=14)

    axes[0].plot(t, q_des, label="q_desired", linewidth=1.2)
    axes[0].plot(t, q_cur, label="q_current", linewidth=1.2)
    axes[0].set_ylabel("Position [rad]")
    axes[0].grid(True)
    axes[0].legend()

    if qdot is not None:
        axes[1].plot(t, qdot, label="q_dot", linewidth=1.2, color="tab:green")
        axes[1].legend()
    else:
        axes[1].text(0.5, 0.5, "joint_velocity_log: no data", ha="center", va="center")
    axes[1].set_ylabel("Velocity [rad/s]")
    axes[1].grid(True)

    if tau_joint_a is not None:
        axes[2].plot(t, tau_joint_a, label="tau_joint_a", linewidth=1.2)
        if tau_joint_b is not None:
            axes[2].plot(t, tau_joint_b, label="tau_joint_b", linewidth=1.0, linestyle="--")
        axes[2].legend()
    else:
        axes[2].text(0.5, 0.5, "torque_joint_log: no data", ha="center", va="center")
    axes[2].set_ylabel("Joint Torque [Nm]")
    axes[2].grid(True)

    if tau_motor_a is not None:
        axes[3].plot(t, tau_motor_a, label="tau_motor_a", linewidth=1.2)
        if tau_motor_b is not None:
            axes[3].plot(t, tau_motor_b, label="tau_motor_b", linewidth=1.0, linestyle="--")
        axes[3].legend()
    else:
        axes[3].text(0.5, 0.5, "torque_motor_log: no data", ha="center", va="center")
    axes[3].set_ylabel("Motor Torque [Nm]")
    axes[3].set_xlabel("Time [s]")
    axes[3].grid(True)

    plt.tight_layout()

    save_path = args.save if args.save is not None else data_dir / "jointpd_plot.png"
    save_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(save_path, dpi=150)
    print(f"Saved: {save_path}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
