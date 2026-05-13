from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
SCRIPTS_ROOT = PROJECT_ROOT / "scripts"
for path in (SRC_ROOT, SCRIPTS_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from config import load_json
from controllers.pid_controller import PIDAngleController
from animation_utils import animate_pulleys
from simulation import SimulationConfig, simulate
from system import CablePulleyPlant, CablePulleyPlantParameters


OUTPUT_ROOT = PROJECT_ROOT / "pid tuning"
MAX_VOLTAGE = 24.0
INTEGRATOR_LIMIT = 8.0
DERIVATIVE_FILTER_TIME = 0.04


def build_pid_case(
    base_config: dict,
    reference_mode: str,
    kp: float,
    ki: float,
    kd: float,
) -> tuple[CablePulleyPlant, PIDAngleController, SimulationConfig]:
    plant_cfg = base_config["plant"]
    sim_cfg = base_config["simulation"]
    reference_cfg = base_config["controller"]
    initial_cfg = base_config["initial_state"]

    plant = CablePulleyPlant(CablePulleyPlantParameters(**plant_cfg))
    controller = PIDAngleController(
        kp_voltage=kp,
        ki_voltage=ki,
        kd_voltage=kd,
        reference_amplitude=float(reference_cfg["reference_amplitude"]),
        reference_frequency=float(reference_cfg["reference_frequency"]),
        max_voltage=MAX_VOLTAGE,
        reference_mode=reference_mode,
        step_segment_duration=5.0,
        integrator_limit=INTEGRATOR_LIMIT,
        derivative_filter_time=DERIVATIVE_FILTER_TIME,
    )
    sim_config = SimulationConfig(
        dt=float(sim_cfg["dt"]),
        horizon=float(sim_cfg["horizon"]),
        initial_state=np.array(
            [
                initial_cfg["theta_l"],
                initial_cfg["omega_l"],
                initial_cfg["theta_m"],
                initial_cfg["omega_m"],
                initial_cfg["current"],
            ],
            dtype=float,
        ),
    )
    return plant, controller, sim_config


def moving_average(signal: np.ndarray, sample_count: int) -> np.ndarray:
    sample_count = max(3, int(sample_count))
    if sample_count % 2 == 0:
        sample_count += 1
    if sample_count >= len(signal):
        return np.full_like(signal, float(np.mean(signal)))
    left_pad = sample_count // 2
    right_pad = sample_count - 1 - left_pad
    padded = np.pad(signal, (left_pad, right_pad), mode="edge")
    kernel = np.ones(sample_count) / sample_count
    return np.convolve(padded, kernel, mode="valid")


def highpass(signal: np.ndarray, dt: float, window_seconds: float = 1.0) -> np.ndarray:
    return signal - moving_average(signal, int(window_seconds / dt))


def step_segment_metrics(
    result: dict[str, np.ndarray],
    threshold: float = 0.05,
    segment_duration: float = 5.0,
) -> dict[str, float]:
    time = result["time"]
    theta = result["state"][:, 0]
    reference = result["reference"]
    starts = [5.0, 10.0, 15.0]
    tail_rms_values: list[float] = []
    settling_times: list[float] = []
    overshoots: list[float] = []

    for start in starts:
        end = min(start + segment_duration, float(time[-1]))
        segment = np.where((time >= start) & (time <= end))[0]
        if len(segment) == 0:
            continue
        previous_ref = float(reference[max(segment[0] - 1, 0)])
        target_ref = float(reference[segment[0]])
        delta = target_ref - previous_ref
        error = reference[segment] - theta[segment]

        tail_start = end - 1.0
        tail = np.where((time >= tail_start) & (time <= end))[0]
        if len(tail) > 0:
            tail_error = reference[tail] - theta[tail]
            tail_rms_values.append(float(np.sqrt(np.mean(tail_error**2))))

        settled = segment[-1]
        for idx in segment:
            future = segment[segment >= idx]
            future_error = reference[future] - theta[future]
            if np.all(np.abs(future_error) <= threshold):
                settled = idx
                break
        settling_times.append(float(time[settled] - start))

        if abs(delta) > 1.0e-12:
            progress = (theta[segment] - previous_ref) / delta
            overshoots.append(float(max(0.0, np.max(progress) - 1.0)))

    return {
        "step_tail_rms_error_rad": float(np.mean(tail_rms_values)) if tail_rms_values else np.nan,
        "step_max_tail_rms_error_rad": float(np.max(tail_rms_values)) if tail_rms_values else np.nan,
        "step_mean_settling_time_s": float(np.mean(settling_times)) if settling_times else np.nan,
        "step_max_settling_time_s": float(np.max(settling_times)) if settling_times else np.nan,
        "step_max_overshoot_ratio": float(np.max(overshoots)) if overshoots else np.nan,
    }


def compute_metrics(
    result: dict[str, np.ndarray],
    kp: float,
    ki: float,
    kd: float,
    reference_mode: str,
) -> dict[str, float]:
    time = result["time"]
    dt = float(time[1] - time[0])
    state = result["state"]
    error = result["reference"] - state[:, 0]
    voltage = result["voltage"]
    q_dot = result["cable_deformation_rate"]
    q_dot_hp = highpass(q_dot, dt)
    voltage_rate = np.diff(voltage) / dt
    tail = slice(len(time) // 5, None)
    details = result["details"]
    metrics = {
        "kp": float(kp),
        "ki": float(ki),
        "kd": float(kd),
        "reference_mode": reference_mode,
        "rms_angle_error_rad": float(np.sqrt(np.mean(error**2))),
        "max_abs_angle_error_rad": float(np.max(np.abs(error))),
        "tail_rms_angle_error_rad": float(np.sqrt(np.mean(error[tail] ** 2))),
        "rms_relative_velocity_m_s": float(np.sqrt(np.mean(q_dot**2))),
        "rms_highpass_relative_velocity_m_s": float(
            np.sqrt(np.mean(q_dot_hp[tail] ** 2))
        ),
        "max_abs_cable_deformation_m": float(np.max(np.abs(result["cable_deformation"]))),
        "min_cable_tension_N": float(np.min(np.minimum(result["tension_1"], result["tension_2"]))),
        "max_abs_voltage_V": float(np.max(np.abs(voltage))),
        "rms_voltage_V": float(np.sqrt(np.mean(voltage**2))),
        "rms_voltage_rate_V_s": float(np.sqrt(np.mean(voltage_rate**2))),
        "max_abs_current_A": float(np.max(np.abs(state[:, 4]))),
        "saturation_fraction": float(np.mean(np.abs(voltage) >= 0.999 * MAX_VOLTAGE)),
        "final_integral_error": float(details.get("integral_error", np.array([0.0]))[-1]),
    }
    if reference_mode == "step":
        metrics.update(step_segment_metrics(result))
    return metrics


def sine_score(metrics: dict[str, float]) -> float:
    tension_penalty = max(0.0, 5.0 - metrics["min_cable_tension_N"])
    return float(
        metrics["rms_angle_error_rad"]
        + 0.15 * metrics["rms_highpass_relative_velocity_m_s"]
        + 0.0012 * metrics["rms_voltage_rate_V_s"]
        + 0.004 * metrics["rms_voltage_V"]
        + 0.8 * metrics["saturation_fraction"]
        + 10.0 * tension_penalty
    )


def step_score(metrics: dict[str, float]) -> float:
    tension_penalty = max(0.0, 5.0 - metrics["min_cable_tension_N"])
    return float(
        0.45 * metrics["rms_angle_error_rad"]
        + 1.30 * metrics["step_tail_rms_error_rad"]
        + 0.06 * metrics["step_mean_settling_time_s"]
        + 0.18 * metrics["step_max_overshoot_ratio"]
        + 0.0010 * metrics["rms_voltage_rate_V_s"]
        + 0.003 * metrics["rms_voltage_V"]
        + 0.8 * metrics["saturation_fraction"]
        + 10.0 * tension_penalty
    )


def coarse_candidates(reference_mode: str) -> list[tuple[float, float, float]]:
    if reference_mode == "sine":
        kp_values = [6.0, 10.0, 14.0, 20.0, 28.0]
        ki_values = [0.0, 0.05, 0.20]
        kd_values = [0.0, 0.8, 1.8, 3.8, 6.0]
    else:
        kp_values = [6.0, 10.0, 16.0, 24.0, 34.0]
        ki_values = [0.0, 0.10, 0.40, 1.00]
        kd_values = [0.0, 1.0, 2.4, 5.0, 8.0]
    return [(kp, ki, kd) for kp in kp_values for ki in ki_values for kd in kd_values]


def refine_candidates(top_metrics: list[dict[str, float]]) -> list[tuple[float, float, float]]:
    candidates: set[tuple[float, float, float]] = set()
    for metrics in top_metrics:
        kp = metrics["kp"]
        ki = metrics["ki"]
        kd = metrics["kd"]
        kp_values = [0.85 * kp, kp, 1.15 * kp]
        if ki <= 1.0e-12:
            ki_values = [0.0, 0.03, 0.08]
        else:
            ki_values = [0.65 * ki, ki, 1.45 * ki]
        if kd <= 1.0e-12:
            kd_values = [0.0, 0.4, 1.0]
        else:
            kd_values = [0.75 * kd, kd, 1.3 * kd]
        for new_kp in kp_values:
            for new_ki in ki_values:
                for new_kd in kd_values:
                    candidates.add(
                        (
                            round(max(new_kp, 0.0), 4),
                            round(max(new_ki, 0.0), 4),
                            round(max(new_kd, 0.0), 4),
                        )
                    )
    return sorted(candidates)


def evaluate_candidate(
    base_config: dict,
    reference_mode: str,
    gains: tuple[float, float, float],
) -> tuple[dict[str, float], dict[str, np.ndarray] | None]:
    kp, ki, kd = gains
    plant, controller, sim_config = build_pid_case(base_config, reference_mode, kp, ki, kd)
    result = simulate(plant, controller, sim_config)
    if not np.all(np.isfinite(result["state"])):
        metrics = {
            "kp": kp,
            "ki": ki,
            "kd": kd,
            "reference_mode": reference_mode,
            "score": float("inf"),
        }
        return metrics, None
    if np.max(np.abs(result["state"][:, 0])) > 100.0:
        metrics = {
            "kp": kp,
            "ki": ki,
            "kd": kd,
            "reference_mode": reference_mode,
            "score": float("inf"),
        }
        return metrics, None
    metrics = compute_metrics(result, kp, ki, kd, reference_mode)
    metrics["score"] = sine_score(metrics) if reference_mode == "sine" else step_score(metrics)
    return metrics, result


def tune_mode(base_config: dict, reference_mode: str) -> tuple[list[dict[str, float]], dict[str, np.ndarray]]:
    seen: set[tuple[float, float, float]] = set()
    metrics_list: list[dict[str, float]] = []
    result_by_gain: dict[tuple[float, float, float], dict[str, np.ndarray]] = {}

    for gains in coarse_candidates(reference_mode):
        seen.add(gains)
        metrics, result = evaluate_candidate(base_config, reference_mode, gains)
        metrics_list.append(metrics)
        if result is not None:
            result_by_gain[gains] = result

    finite_metrics = [m for m in metrics_list if np.isfinite(m.get("score", np.inf))]
    finite_metrics.sort(key=lambda item: item["score"])

    for gains in refine_candidates(finite_metrics[:3]):
        if gains in seen:
            continue
        seen.add(gains)
        metrics, result = evaluate_candidate(base_config, reference_mode, gains)
        metrics_list.append(metrics)
        if result is not None:
            result_by_gain[gains] = result

    finite_metrics = [m for m in metrics_list if np.isfinite(m.get("score", np.inf))]
    finite_metrics.sort(key=lambda item: item["score"])
    best = finite_metrics[0]
    best_gains = (best["kp"], best["ki"], best["kd"])
    return finite_metrics, result_by_gain[best_gains]


def plot_pid_history(result: dict[str, np.ndarray], output_path: Path, title: str) -> None:
    time = result["time"]
    state = result["state"]
    reference = result["reference"]
    details = result["details"]
    error = reference - state[:, 0]

    fig, axes = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
    axes[0].plot(time, reference, "k--", linewidth=1.5, label=r"$\theta_{ref}$")
    axes[0].plot(time, state[:, 0], linewidth=1.6, label=r"$\theta_l$")
    axes[0].set_ylabel("angle [rad]")
    axes[0].legend(loc="upper right")

    axes[1].plot(time, error, linewidth=1.2, label=r"$\theta_{ref}-\theta_l$")
    axes[1].set_ylabel("position error [rad]")
    axes[1].legend(loc="upper right")

    axes[2].plot(time, state[:, 1], linewidth=1.2, label=r"$\omega_l$")
    axes[2].plot(time, details["reference_velocity"], "--", linewidth=1.1, label=r"$\dot\theta_{ref}$")
    axes[2].set_ylabel("velocity [rad/s]")
    axes[2].legend(loc="upper right")

    axes[3].plot(time, result["voltage"], color="tab:red", linewidth=1.2, label="voltage")
    axes[3].set_ylabel("voltage [V]")
    axes[3].legend(loc="upper right")

    axes[4].plot(time, state[:, 4], linewidth=1.2, label="motor current")
    axes[4].plot(time, result["target_torque"], linewidth=1.1, label="target cable torque")
    axes[4].set_ylabel("current / torque")
    axes[4].legend(loc="upper right")

    axes[5].plot(time, result["tension_1"], linewidth=1.1, label=r"$T_1$")
    axes[5].plot(time, result["tension_2"], linewidth=1.1, label=r"$T_2$")
    axes[5].set_ylabel("tension [N]")
    axes[5].set_xlabel("time [s]")
    axes[5].legend(loc="upper right")

    for axis in axes:
        axis.grid(True, alpha=0.3)
    fig.suptitle(title, y=0.995)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def save_mode_artifacts(
    mode_dir: Path,
    name: str,
    best_metrics: dict[str, float],
    result: dict[str, np.ndarray],
) -> None:
    mode_dir.mkdir(parents=True, exist_ok=True)
    mode = str(best_metrics["reference_mode"])
    plot_pid_history(result, mode_dir / f"{name}_state_history.png", f"PID {mode} response")
    animate_pulleys(
        result,
        mode_dir / f"{name}_animation.gif",
        fps=12,
        title=f"PID {mode} response",
        annotation_lines=[
            f"kp={best_metrics['kp']:.3g}, ki={best_metrics['ki']:.3g}",
            f"kd={best_metrics['kd']:.3g}",
        ],
    )
    (mode_dir / f"{name}_summary.json").write_text(
        json.dumps(best_metrics, indent=2),
        encoding="utf-8",
    )


def write_readme(
    output_root: Path,
    sine_best: dict[str, float],
    step_best: dict[str, float],
) -> None:
    lines = [
        "# PID Tuning",
        "",
        "This folder contains PID-only tuning results for the cable-pulley plant.",
        "No backstepping controller is used in this evaluation.",
        "",
        "The PID voltage law is:",
        "",
        "```math",
        "V = K_p e + K_i \\int e\\,dt + K_d(\\dot\\theta_{ref}-\\omega_l)",
        "```",
        "",
        "with voltage saturation and anti-windup. For the step case, `dot theta_ref = 0` between jumps, so the derivative term acts as target-shaft velocity damping and avoids derivative kick at the step instant.",
        "",
        "## Meaning of Gains",
        "",
        "- `Kp` increases stiffness of the closed-loop angle response. Larger `Kp` reduces position error but can excite the flexible transmission and increase overshoot.",
        "- `Ki` accumulates persistent error. It helps remove steady-state error from gravity/load torque, but too much `Ki` causes windup, overshoot, and slow oscillations.",
        "- `Kd` reacts to velocity error. It damps motion and reduces overshoot, but too much `Kd` makes the voltage noisy/aggressive and can fight the flexible transmission.",
        "",
        "## Tuning Objective",
        "",
        "The sine and step modes were tuned separately because they reward different behavior:",
        "",
        "- sine tracking: low RMS tracking error with limited cable vibration and voltage activity;",
        "- step transfer: good final settling after each command change, limited overshoot, and no excessive saturation.",
        "",
        "`Best` here means the best PID candidate in the searched practical criterion, not a proof of global optimality. The criterion deliberately penalizes cable vibration and aggressive voltage, so the selected gains are meant to be usable rather than merely high-gain.",
        "",
        "## Best Gains",
        "",
        "| Mode | Kp | Ki | Kd | Score | RMS error [rad] | Max error [rad] | Max current [A] | Saturation fraction |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        f"| sine | {sine_best['kp']:.4g} | {sine_best['ki']:.4g} | {sine_best['kd']:.4g} | {sine_best['score']:.4f} | {sine_best['rms_angle_error_rad']:.4f} | {sine_best['max_abs_angle_error_rad']:.4f} | {sine_best['max_abs_current_A']:.2f} | {sine_best['saturation_fraction']:.3f} |",
        f"| step | {step_best['kp']:.4g} | {step_best['ki']:.4g} | {step_best['kd']:.4g} | {step_best['score']:.4f} | {step_best['rms_angle_error_rad']:.4f} | {step_best['max_abs_angle_error_rad']:.4f} | {step_best['max_abs_current_A']:.2f} | {step_best['saturation_fraction']:.3f} |",
        "",
        "## Sine PID Result",
        "",
        f"Best sine gains: `Kp={sine_best['kp']:.4g}`, `Ki={sine_best['ki']:.4g}`, `Kd={sine_best['kd']:.4g}`.",
        "",
        f"- RMS angle error: `{sine_best['rms_angle_error_rad']:.4f}` rad",
        f"- max angle error: `{sine_best['max_abs_angle_error_rad']:.4f}` rad",
        f"- high-pass RMS cable relative velocity: `{sine_best['rms_highpass_relative_velocity_m_s']:.4f}` m/s",
        f"- RMS voltage rate: `{sine_best['rms_voltage_rate_V_s']:.1f}` V/s",
        f"- minimum cable tension: `{sine_best['min_cable_tension_N']:.2f}` N",
        "",
        "Interpretation: the sine-tuned PID favors smooth periodic tracking. The integral term is usually small because the sine does not require holding a constant offset for long, while derivative action is useful for phase and damping.",
        "",
        "## Step PID Result",
        "",
        f"Best step gains: `Kp={step_best['kp']:.4g}`, `Ki={step_best['ki']:.4g}`, `Kd={step_best['kd']:.4g}`.",
        "",
        f"- RMS angle error: `{step_best['rms_angle_error_rad']:.4f}` rad",
        f"- max angle error: `{step_best['max_abs_angle_error_rad']:.4f}` rad",
        f"- mean settling time to 0.05 rad band: `{step_best['step_mean_settling_time_s']:.3f}` s",
        f"- max overshoot ratio: `{step_best['step_max_overshoot_ratio']:.3f}`",
        f"- average final-second RMS error per step segment: `{step_best['step_tail_rms_error_rad']:.4f}` rad",
        "",
        "The max error for the step case is dominated by the instant when the reference jumps, especially from `pi/2` to `-pi/2`. For this mode the more meaningful metrics are final-segment RMS error, settling time, overshoot, and saturation.",
        "",
        "Interpretation: the step-tuned PID needs more emphasis on settling and overshoot control. The integral term helps reject the rod gravity/load bias at fixed angles, while derivative action prevents the jumps from turning into sustained oscillation.",
        "",
        "## Files",
        "",
        "- `sine_ranking.json`: all sine PID candidates ranked by score.",
        "- `step_ranking.json`: all step PID candidates ranked by score.",
        "- `sine_best/sine_best_state_history.png`",
        "- `sine_best/sine_best_animation.gif`",
        "- `sine_best/sine_best_summary.json`",
        "- `step_best/step_best_state_history.png`",
        "- `step_best/step_best_animation.gif`",
        "- `step_best/step_best_summary.json`",
        "",
    ]
    (output_root / "README.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    OUTPUT_ROOT.mkdir(exist_ok=True)
    base_config = load_json(PROJECT_ROOT / "configs" / "default.json")

    sine_ranking, sine_best_result = tune_mode(base_config, "sine")
    step_ranking, step_best_result = tune_mode(base_config, "step")

    sine_best = sine_ranking[0]
    step_best = step_ranking[0]

    save_mode_artifacts(OUTPUT_ROOT / "sine_best", "sine_best", sine_best, sine_best_result)
    save_mode_artifacts(OUTPUT_ROOT / "step_best", "step_best", step_best, step_best_result)

    (OUTPUT_ROOT / "sine_ranking.json").write_text(
        json.dumps(sine_ranking, indent=2),
        encoding="utf-8",
    )
    (OUTPUT_ROOT / "step_ranking.json").write_text(
        json.dumps(step_ranking, indent=2),
        encoding="utf-8",
    )
    write_readme(OUTPUT_ROOT, sine_best, step_best)
    print(json.dumps({"sine_best": sine_best, "step_best": step_best}, indent=2))


if __name__ == "__main__":
    main()
