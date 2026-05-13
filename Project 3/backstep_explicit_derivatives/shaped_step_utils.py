from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from run_explicit_derivatives_backstepping import summarize


def command_step_metrics(
    time: np.ndarray,
    theta: np.ndarray,
    command: np.ndarray,
    threshold: float = 0.05,
    segment_duration: float = 5.0,
) -> dict[str, float]:
    starts = [5.0, 10.0, 15.0]
    tail_rms_values: list[float] = []
    settling_times: list[float] = []
    overshoots: list[float] = []

    for start in starts:
        end = min(start + segment_duration, float(time[-1]))
        segment = np.where((time >= start) & (time <= end))[0]
        if len(segment) == 0:
            continue
        previous_command = float(command[max(segment[0] - 1, 0)])
        delta = float(command[segment[0]]) - previous_command

        tail_start = end - 1.0
        tail = np.where((time >= tail_start) & (time <= end))[0]
        if len(tail) > 0:
            tail_error = command[tail] - theta[tail]
            tail_rms_values.append(float(np.sqrt(np.mean(tail_error**2))))

        settled = segment[-1]
        for idx in segment:
            future = segment[segment >= idx]
            future_error = command[future] - theta[future]
            if np.all(np.abs(future_error) <= threshold):
                settled = idx
                break
        settling_times.append(float(time[settled] - start))

        if abs(delta) > 1.0e-12:
            progress = (theta[segment] - previous_command) / delta
            overshoots.append(float(max(0.0, np.max(progress) - 1.0)))

    return {
        "command_step_tail_rms_error_rad": float(np.mean(tail_rms_values))
        if tail_rms_values
        else np.nan,
        "command_step_max_tail_rms_error_rad": float(np.max(tail_rms_values))
        if tail_rms_values
        else np.nan,
        "command_step_mean_settling_time_s": float(np.mean(settling_times))
        if settling_times
        else np.nan,
        "command_step_max_settling_time_s": float(np.max(settling_times))
        if settling_times
        else np.nan,
        "command_step_max_overshoot_ratio": float(np.max(overshoots))
        if overshoots
        else np.nan,
    }


def shaped_step_summary(
    name: str,
    result: dict[str, np.ndarray],
    max_voltage: float,
) -> dict[str, float | str]:
    metrics = summarize(name, result, max_voltage)
    time = result["time"]
    theta = result["state"][:, 0]
    shaped_reference = result["reference"]
    raw_command = result["details"].get("raw_reference", shaped_reference)
    command_error = raw_command - theta
    shaper_lag = raw_command - shaped_reference
    metrics.update(
        {
            "rms_command_error_rad": float(np.sqrt(np.mean(command_error**2))),
            "tail_rms_command_error_rad": float(
                np.sqrt(np.mean(command_error[len(time) // 5 :] ** 2))
            ),
            "max_abs_command_error_rad": float(np.max(np.abs(command_error))),
            "rms_shaper_lag_rad": float(np.sqrt(np.mean(shaper_lag**2))),
            "max_abs_shaper_lag_rad": float(np.max(np.abs(shaper_lag))),
        }
    )
    metrics.update(command_step_metrics(time, theta, raw_command))
    metrics["score"] = shaped_step_score(metrics)
    return metrics


def shaped_step_score(metrics: dict[str, float | str]) -> float:
    return float(
        0.25 * float(metrics["rms_angle_error_rad"])
        + 1.10 * float(metrics["command_step_tail_rms_error_rad"])
        + 0.05 * float(metrics["command_step_mean_settling_time_s"])
        + 0.10 * float(metrics["rms_highpass_relative_velocity_m_s"])
        + 0.0015 * float(metrics["rms_voltage_rate_V_s"])
        + 0.01 * float(metrics["max_abs_current_A"])
    )


def plot_shaped_step_history(
    result: dict[str, np.ndarray],
    output_path: Path,
    title: str,
) -> None:
    time = result["time"]
    state = result["state"]
    details = result["details"]
    raw_command = details.get("raw_reference", result["reference"])
    shaped_reference = result["reference"]
    tracking_error = shaped_reference - state[:, 0]

    fig, axes = plt.subplots(7, 1, figsize=(11, 14), sharex=True)
    axes[0].plot(time, raw_command, "k:", linewidth=1.4, label=r"raw $\theta_{cmd}$")
    axes[0].plot(time, shaped_reference, "k--", linewidth=1.4, label=r"shaped $r$")
    axes[0].plot(time, state[:, 0], linewidth=1.3, label=r"$\theta_l$")
    axes[0].set_ylabel("angle [rad]")
    axes[0].legend(loc="upper right")

    axes[1].plot(time, tracking_error, linewidth=1.2, label=r"$r-\theta_l$")
    axes[1].set_ylabel("position error [rad]")
    axes[1].legend(loc="upper right")

    axes[2].plot(time, state[:, 1], linewidth=1.1, label=r"$\omega_l$")
    axes[2].plot(time, details["alpha1"], "--", linewidth=1.0, label=r"$\alpha_1$")
    axes[2].set_ylabel("velocity")
    axes[2].legend(loc="upper right")

    axes[3].plot(time, result["target_torque"], linewidth=1.1, label=r"$\tau_c$")
    axes[3].plot(time, details["torque_desired"], "--", linewidth=1.0, label=r"$\tau_c^d$")
    axes[3].set_ylabel("torque [Nm]")
    axes[3].legend(loc="upper right")

    axes[4].plot(time, state[:, 4], linewidth=1.1, label=r"$i$")
    axes[4].plot(time, details["current_desired"], "--", linewidth=1.0, label=r"$i_d$")
    axes[4].set_ylabel("current [A]")
    axes[4].legend(loc="upper right")

    axes[5].plot(time, result["tension_1"], linewidth=1.0, label=r"$T_1$")
    axes[5].plot(time, result["tension_2"], linewidth=1.0, label=r"$T_2$")
    axes[5].set_ylabel("tension [N]")
    axes[5].legend(loc="upper right")

    axes[6].plot(time, result["voltage"], linewidth=1.2, label=r"$V$")
    axes[6].plot(time, details["voltage_unsaturated"], "--", linewidth=0.9, label=r"$V_{raw}$")
    axes[6].set_ylabel("voltage [V]")
    axes[6].set_xlabel("time [s]")
    axes[6].legend(loc="upper right")

    for axis in axes:
        axis.grid(True, alpha=0.3)
    fig.suptitle(title, y=0.995)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)
