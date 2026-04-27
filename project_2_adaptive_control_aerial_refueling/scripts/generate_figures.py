from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from mpl_toolkits.axes_grid1.inset_locator import inset_axes

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from config import build_baseline_controllers, build_from_config, load_json  # noqa: E402
from simulation import SimulationConfig, simulate  # noqa: E402


FIGURES_DIR = PROJECT_ROOT / "figures"
RESULTS_DIR = PROJECT_ROOT / "results"
CONFIG_PATH = PROJECT_ROOT / "configs" / "default.json"
CONTROLLER_LABELS = {
    "zero": "Zero controller",
    "pd": "Nominal-mass PD controller",
    "adaptive": "Adaptive inverse-mass controller",
}
CONTROLLER_FILE_LABELS = {
    "zero": "zero_controller",
    "pd": "pd_controller",
    "adaptive": "adaptive_controller",
}
CONTROLLER_COLORS = {
    "zero": "#7a7a7a",
    "pd": "#d46424",
    "adaptive": "#1f5fa8",
}


def save(fig: plt.Figure, name: str) -> None:
    FIGURES_DIR.mkdir(exist_ok=True)
    fig.savefig(FIGURES_DIR / f"{name}.png", dpi=220)
    fig.savefig(FIGURES_DIR / f"{name}.pdf")
    plt.close(fig)


def time_colored_line(
    ax: plt.Axes,
    time: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    linewidth: float = 2.2,
    color_time_max: float | None = None,
) -> LineCollection:
    color_time_max = float(time.max() if color_time_max is None else color_time_max)
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segments, cmap="turbo", norm=plt.Normalize(time.min(), color_time_max, clip=True))
    lc.set_array((time[:-1] + time[1:]) / 2.0)
    lc.set_linewidth(linewidth)
    ax.add_collection(lc)
    ax.update_datalim(np.column_stack([x, y]))
    ax.autoscale_view()
    return lc


def mark_reference_points(
    ax: plt.Axes,
    start: tuple[float, float],
    finish: tuple[float, float],
    target: tuple[float, float],
    target_label: str,
) -> None:
    ax.scatter([start[0]], [start[1]], color="#d46424", s=72, label="initial state", zorder=5)
    ax.scatter([finish[0]], [finish[1]], color="#2e7d32", s=72, label="final state", zorder=5)
    ax.scatter([target[0]], [target[1]], color="#111111", marker="x", s=95, label=target_label, zorder=6)


def plot_controller_tracking_summary(result: dict[str, np.ndarray], radius: float, controller_key: str) -> None:
    time = result["time"]
    states = result["state"]
    color = CONTROLLER_COLORS[controller_key]
    controller_label = CONTROLLER_LABELS[controller_key]
    file_label = CONTROLLER_FILE_LABELS[controller_key]
    color_time_max = 30.0
    fig, axs = plt.subplots(2, 2, figsize=(14.0, 9.2), constrained_layout=True)

    ax_traj = axs[0, 0]
    lc = time_colored_line(ax_traj, time, states[:, 0], states[:, 1], linewidth=2.4, color_time_max=color_time_max)
    circle = plt.Circle(
        (0.0, 0.0),
        radius,
        fill=False,
        color="#555555",
        linestyle="--",
        linewidth=1.8,
        label="refueling zone boundary",
    )
    ax_traj.add_patch(circle)
    mark_reference_points(
        ax_traj,
        (states[0, 0], states[0, 1]),
        (states[-1, 0], states[-1, 1]),
        (0.0, 0.0),
        "refueling point",
    )
    if controller_key == "zero":
        ax_traj.set_aspect("auto")
    else:
        ax_traj.set_aspect("equal", adjustable="box")
    ax_traj.set_xlabel("x [m]")
    ax_traj.set_ylabel("z [m]")
    ax_traj.set_title(f"{controller_label}: position portrait z(x)")
    ax_traj.grid(True, alpha=0.28)
    ax_traj.legend(loc="best")

    ax_vx = axs[0, 1]
    ax_vx.plot(time, states[:, 2], color=color, linewidth=2.0)
    ax_vx.axhline(0.0, color="#777777", linewidth=0.9)
    ax_vx.scatter(time[0], states[0, 2], color="#d46424", s=50, label="initial state", zorder=4)
    ax_vx.scatter(time[-1], states[-1, 2], color="#2e7d32", s=50, label="final state", zorder=4)
    ax_vx.set_xlabel("t [s]")
    ax_vx.set_ylabel(r"$v_x$ [m/s]")
    ax_vx.set_title("Longitudinal velocity")
    ax_vx.grid(True, alpha=0.28)
    ax_vx.legend(loc="best")

    ax_vz = axs[1, 0]
    ax_vz.plot(time, states[:, 3], color=color, linewidth=2.0)
    ax_vz.axhline(0.0, color="#777777", linewidth=0.9)
    ax_vz.scatter(time[0], states[0, 3], color="#d46424", s=50, label="initial state", zorder=4)
    ax_vz.scatter(time[-1], states[-1, 3], color="#2e7d32", s=50, label="final state", zorder=4)
    ax_vz.set_xlabel("t [s]")
    ax_vz.set_ylabel(r"$v_z$ [m/s]")
    ax_vz.set_title("Vertical velocity")
    ax_vz.grid(True, alpha=0.28)
    ax_vz.legend(loc="best")

    ax_dist = axs[1, 1]
    ax_dist.plot(time, result["position_norm"], color=color, linewidth=2.0, label=r"$\|r(t)\|$")
    ax_dist.axhline(radius, color="#555555", linestyle="--", linewidth=1.5, label="zone radius R")
    ax_dist.set_xlabel("t [s]")
    ax_dist.set_ylabel(r"$\|r(t)\|$ [m]")
    ax_dist.set_title("Distance from the refueling point")
    ax_dist.grid(True, alpha=0.28)
    ax_dist.legend(loc="best")

    cbar = fig.colorbar(lc, ax=axs[:, 0], location="right", shrink=0.92)
    cbar.set_label(f"time [s], clipped at {color_time_max:.0f} s")
    save(fig, f"{file_label}_tracking_summary")


def plot_controller_basic_diagnostics(result: dict[str, np.ndarray], controller_key: str) -> None:
    time = result["time"]
    force_time = result["time"][1:]
    force = result["force"][1:]
    color = CONTROLLER_COLORS[controller_key]
    controller_label = CONTROLLER_LABELS[controller_key]
    file_label = CONTROLLER_FILE_LABELS[controller_key]
    fig, axs = plt.subplots(2, 1, figsize=(11.5, 8.0), constrained_layout=True, sharex=False)

    axs[0].plot(force_time, force[:, 0], label=r"incremental thrust correction $u_T$", linewidth=1.8)
    axs[0].plot(force_time, force[:, 1], label=r"incremental lift correction $u_L$", linewidth=1.8)
    axs[0].set_xlabel("t [s]")
    axs[0].set_ylabel("force correction [N]")
    axs[0].set_title(f"{controller_label}: force commands")
    axs[0].grid(True, alpha=0.28)
    axs[0].legend(loc="best")

    axs[1].plot(time, result["mass"], color=color, label=r"true mass $m(t)$", linewidth=2.0)
    axs[1].set_xlabel("t [s]")
    axs[1].set_ylabel("mass [kg]")
    axs[1].set_title("True mass during refueling")
    axs[1].grid(True, alpha=0.28)
    axs[1].legend(loc="best")

    save(fig, f"{file_label}_diagnostics")


def plot_adaptive_diagnostics(result: dict[str, np.ndarray]) -> None:
    time = result["time"]
    force_time = result["time"][1:]
    force = result["force"][1:]
    fig, axs = plt.subplots(3, 1, figsize=(11.5, 12.0), constrained_layout=True, sharex=False)

    axs[0].plot(force_time, force[:, 0], label=r"incremental thrust correction $u_T$", linewidth=1.8)
    axs[0].plot(force_time, force[:, 1], label=r"incremental lift correction $u_L$", linewidth=1.8)
    axs[0].set_xlabel("t [s]")
    axs[0].set_ylabel("force correction [N]")
    axs[0].set_title("Adaptive control force commands")
    axs[0].grid(True, alpha=0.28)
    axs[0].legend(loc="best")

    axs[1].plot(time, result["mass"], label=r"true mass $m(t)$", linewidth=2.0)
    axs[1].plot(time, result["mass_hat"], label=r"adaptive estimate $\hat m(t)$", linewidth=1.8)
    axs[1].set_xlabel("t [s]")
    axs[1].set_ylabel("mass [kg]")
    axs[1].set_title("Mass and adaptive estimate")
    axs[1].grid(True, alpha=0.28)
    axs[1].legend(loc="best")

    axs[2].plot(time, result["lyapunov"], color="#3c965a", linewidth=2.0)
    axs[2].set_xlabel("t [s]")
    axs[2].set_ylabel(r"$L(e,\tilde\theta)$")
    axs[2].set_title("Augmented Lyapunov function")
    axs[2].grid(True, alpha=0.28)

    save(fig, "adaptive_controller_diagnostics")


def plot_controller_comparison(
    results: dict[str, dict[str, np.ndarray]],
    radius: float,
) -> None:
    fig, axs = plt.subplots(1, 2, figsize=(14.0, 6.0), constrained_layout=True)

    circle = plt.Circle(
        (0.0, 0.0),
        radius,
        fill=False,
        color="#555555",
        linestyle="--",
        linewidth=1.6,
        label="refueling zone boundary",
    )
    axs[0].add_patch(circle)
    axs[0].scatter(0.0, 0.0, color="#111111", marker="x", s=95, label="refueling point", zorder=5)
    zoom_ax = inset_axes(
        axs[0],
        width="42%",
        height="42%",
        loc="lower left",
        bbox_to_anchor=(0.12, 0.10, 1.0, 1.0),
        bbox_transform=axs[0].transAxes,
        borderpad=1.4,
    )
    zoom_circle = plt.Circle(
        (0.0, 0.0),
        radius,
        fill=False,
        color="#555555",
        linestyle="--",
        linewidth=1.2,
    )
    zoom_ax.add_patch(zoom_circle)
    zoom_ax.scatter(0.0, 0.0, color="#111111", marker="x", s=55, zorder=5)

    for key, result in results.items():
        states = result["state"]
        time = result["time"]
        position_norm = result["position_norm"]
        label = CONTROLLER_LABELS[key]
        color = CONTROLLER_COLORS[key]
        axs[0].plot(states[:, 0], states[:, 1], color=color, linewidth=2.0, label=label)
        axs[0].scatter(states[0, 0], states[0, 1], color=color, marker="o", s=45, zorder=4)
        axs[0].scatter(states[-1, 0], states[-1, 1], color=color, marker="s", s=45, zorder=4)
        zoom_ax.plot(states[:, 0], states[:, 1], color=color, linewidth=1.6)
        zoom_ax.scatter(states[-1, 0], states[-1, 1], color=color, marker="s", s=28, zorder=4)
        axs[1].plot(time, position_norm, color=color, linewidth=2.0, label=label)

    axs[0].set_aspect("equal", adjustable="datalim")
    axs[0].set_xlabel("x [m]")
    axs[0].set_ylabel("z [m]")
    axs[0].set_title("Controller comparison in the refueling plane")
    axs[0].grid(True, alpha=0.28)
    axs[0].scatter([], [], color="#111111", marker="o", s=45, label="initial state marker")
    axs[0].scatter([], [], color="#111111", marker="s", s=45, label="final state marker")
    axs[0].legend(loc="best")
    zoom_ax.set_xlim(-5.0, 105.0)
    zoom_ax.set_ylim(-45.0, 10.0)
    zoom_ax.set_title("near refueling point", fontsize=9)
    zoom_ax.grid(True, alpha=0.22)

    axs[1].axhline(radius, color="#555555", linestyle="--", linewidth=1.5, label="zone radius R")
    axs[1].set_xlabel("t [s]")
    axs[1].set_ylabel(r"$\|r(t)\|$ [m]")
    axs[1].set_title("Distance from the refueling point")
    axs[1].grid(True, alpha=0.28)
    axs[1].legend(loc="best")
    save(fig, "controller_comparison")


def colored_phase(ax: plt.Axes, t: np.ndarray, x: np.ndarray, y: np.ndarray, label: str, color_time_max: float) -> LineCollection:
    lc = time_colored_line(ax, t, x, y, linewidth=1.35, color_time_max=color_time_max)
    lc.set_linewidth(1.15)
    lc.set_alpha(0.95)
    ax.axhline(0.0, color="#888888", linewidth=0.8)
    ax.axvline(0.0, color="#888888", linewidth=0.8)
    ax.set_xlabel(label)
    ax.grid(True, alpha=0.25)
    return lc


def plot_phase_portraits(
    plant,
    controller,
    sim_cfg: SimulationConfig,
    controller_key: str,
) -> None:
    rng = np.random.default_rng(7)
    initial_positions = rng.uniform([-75.0, -36.0], [75.0, 36.0], size=(24, 2))
    initial_velocities = rng.uniform([-0.9, -0.7], [0.9, 0.7], size=(24, 2))
    color_time_max = 35.0
    controller_label = CONTROLLER_LABELS[controller_key]
    file_label = CONTROLLER_FILE_LABELS[controller_key]

    fig, axs = plt.subplots(1, 2, figsize=(13.0, 6.2), constrained_layout=True)
    for pos, vel in zip(initial_positions, initial_velocities):
        local_cfg = SimulationConfig(
            dt=sim_cfg.dt,
            horizon=sim_cfg.horizon,
            refueling_radius=sim_cfg.refueling_radius,
            initial_state=np.array([pos[0], pos[1], vel[0], vel[1]], dtype=float),
        )
        result = simulate(plant, controller, local_cfg)
        t = result["time"]
        s = result["state"]
        lc = colored_phase(axs[0], t, s[:, 0], s[:, 2], "x [m]", color_time_max=color_time_max)
        colored_phase(axs[1], t, s[:, 1], s[:, 3], "z [m]", color_time_max=color_time_max)
        axs[0].scatter(s[0, 0], s[0, 2], color="#d46424", s=20, zorder=4)
        axs[0].scatter(s[-1, 0], s[-1, 2], color="#2e7d32", s=20, zorder=4)
        axs[1].scatter(s[0, 1], s[0, 3], color="#d46424", s=20, zorder=4)
        axs[1].scatter(s[-1, 1], s[-1, 3], color="#2e7d32", s=20, zorder=4)

    axs[0].set_ylabel("vx [m/s]")
    axs[1].set_ylabel("vz [m/s]")
    axs[0].set_title(f"{controller_label}: longitudinal phase portrait")
    axs[1].set_title(f"{controller_label}: vertical phase portrait")
    for ax in axs:
        ax.scatter(0.0, 0.0, color="#b43c2f", marker="x", s=75, zorder=5)
        ax.scatter([], [], color="#d46424", s=45, label="initial state")
        ax.scatter([], [], color="#2e7d32", s=45, label="final state")
        ax.scatter([], [], color="#b43c2f", marker="x", s=65, label="equilibrium")
    for ax in axs:
        ax.legend(loc="best")
    fig.colorbar(lc, ax=axs, location="right", shrink=0.92, label=f"time [s], clipped at {color_time_max:.0f} s")
    save(fig, f"{file_label}_phase_portraits")


def main() -> None:
    config = load_json(CONFIG_PATH)
    plant, controller, sim_cfg = build_from_config(config)
    result = simulate(plant, controller, sim_cfg)
    baseline_controllers = build_baseline_controllers(config, plant)
    comparison_results = {
        "zero": simulate(plant, baseline_controllers["zero"], sim_cfg),
        "pd": simulate(plant, baseline_controllers["pd"], sim_cfg),
        "adaptive": result,
    }

    radius = sim_cfg.refueling_radius
    plot_controller_tracking_summary(comparison_results["zero"], radius, "zero")
    plot_controller_basic_diagnostics(comparison_results["zero"], "zero")
    plot_phase_portraits(plant, baseline_controllers["zero"], sim_cfg, "zero")
    plot_controller_tracking_summary(comparison_results["pd"], radius, "pd")
    plot_controller_basic_diagnostics(comparison_results["pd"], "pd")
    plot_phase_portraits(plant, baseline_controllers["pd"], sim_cfg, "pd")
    plot_controller_tracking_summary(result, radius, "adaptive")
    plot_adaptive_diagnostics(result)
    plot_phase_portraits(plant, controller, sim_cfg, "adaptive")
    plot_controller_comparison(comparison_results, radius)

    RESULTS_DIR.mkdir(exist_ok=True)
    final_state = result["state"][-1]
    comparison_summary = {}
    for name, local_result in comparison_results.items():
        local_final_state = local_result["state"][-1]
        comparison_summary[name] = {
            "final_position_norm_m": float(np.linalg.norm(local_final_state[:2])),
            "final_velocity_norm_m_s": float(np.linalg.norm(local_final_state[2:])),
            "entered_refueling_zone": bool(np.any(local_result["position_norm"] <= radius)),
            "stays_in_zone_after_first_entry": bool(
                np.all(local_result["position_norm"][np.argmax(local_result["position_norm"] <= radius) :] <= radius)
                if np.any(local_result["position_norm"] <= radius)
                else False
            ),
        }
    summary = {
        "final_state": final_state.tolist(),
        "final_position_norm_m": float(np.linalg.norm(final_state[:2])),
        "final_velocity_norm_m_s": float(np.linalg.norm(final_state[2:])),
        "final_mass_kg": float(result["mass"][-1]),
        "final_mass_hat_kg": float(result["mass_hat"][-1]),
        "final_lyapunov": float(result["lyapunov"][-1]),
        "entered_refueling_zone": bool(np.any(result["position_norm"] <= radius)),
        "stays_in_zone_after_first_entry": bool(
            np.all(result["position_norm"][np.argmax(result["position_norm"] <= radius) :] <= radius)
            if np.any(result["position_norm"] <= radius)
            else False
        ),
        "controller_comparison": comparison_summary,
    }
    (RESULTS_DIR / "adaptive_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
