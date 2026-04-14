from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parent


def vertical_averaged_energy(phi: np.ndarray, phi_dot: np.ndarray, alpha: float, omega: float, g: float, l: float) -> np.ndarray:
    return 0.5 * phi_dot**2 + (alpha**2) * np.sin(phi) ** 2 / (4.0 * l**2 * omega**2) + (g / l) * (np.cos(phi) - 1.0)


def vertical_averaged_vdot(phi: np.ndarray, phi_dot: np.ndarray, d: float) -> np.ndarray:
    del phi
    return -d * phi_dot**2


def vertical_clf(phi: np.ndarray, phi_dot: np.ndarray, k0: float, k1: float) -> np.ndarray:
    return 0.5 * phi_dot**2 + 0.5 * k0 * phi**2 + 0.25 * k1 * phi**4


def vertical_clf_vdot(phi: np.ndarray, phi_dot: np.ndarray, d: float) -> np.ndarray:
    del phi
    return -d * phi_dot**2


def vertical_direct_lyapunov(phi: np.ndarray, phi_dot: np.ndarray, kp: float) -> np.ndarray:
    return 0.5 * phi_dot**2 + kp * (1.0 - np.cos(phi))


def vertical_direct_lyapunov_vdot(phi: np.ndarray, phi_dot: np.ndarray, d: float, kc: float) -> np.ndarray:
    return -(d + kc * np.sin(phi) ** 2) * phi_dot**2


def horizontal_effective_energy(theta: np.ndarray, theta_dot: np.ndarray, alpha: float, omega: float, g: float, l: float) -> np.ndarray:
    return 0.5 * theta_dot**2 - (g / l) * np.cos(theta) - (alpha**2) * np.sin(theta) ** 2 / (4.0 * l**2 * omega**2)


def horizontal_effective_vdot(theta: np.ndarray, theta_dot: np.ndarray, d: float) -> np.ndarray:
    del theta
    return -d * theta_dot**2


def plot_pair(
    ax_v: plt.Axes,
    ax_vdot: plt.Axes,
    x: np.ndarray,
    y: np.ndarray,
    V: np.ndarray,
    Vdot: np.ndarray,
    title: str,
    x_label: str,
    y_label: str,
    formula_v: str,
    formula_vdot: str,
    equilibrium_x: float,
    equilibrium_y: float,
) -> None:
    levels_v = 22
    levels_vdot = 22

    im_v = ax_v.contourf(x, y, V, levels=levels_v, cmap="viridis")
    ax_v.contour(x, y, V, levels=10, colors="white", linewidths=0.35, alpha=0.55)
    ax_v.scatter([equilibrium_x], [equilibrium_y], c="crimson", s=32, zorder=5)
    ax_v.set_title(f"{title}: $V$")
    ax_v.set_xlabel(x_label)
    ax_v.set_ylabel(y_label)
    ax_v.text(
        0.02,
        0.02,
        formula_v,
        transform=ax_v.transAxes,
        fontsize=8,
        color="white",
        bbox={"boxstyle": "round,pad=0.25", "facecolor": (0, 0, 0, 0.35), "edgecolor": "none"},
    )
    plt.colorbar(im_v, ax=ax_v, fraction=0.046, pad=0.04)

    im_vdot = ax_vdot.contourf(x, y, Vdot, levels=levels_vdot, cmap="magma")
    ax_vdot.contour(x, y, Vdot, levels=10, colors="white", linewidths=0.35, alpha=0.55)
    ax_vdot.scatter([equilibrium_x], [equilibrium_y], c="cyan", s=32, zorder=5)
    ax_vdot.set_title(f"{title}: $\\dot V$")
    ax_vdot.set_xlabel(x_label)
    ax_vdot.set_ylabel(y_label)
    ax_vdot.text(
        0.02,
        0.02,
        formula_vdot,
        transform=ax_vdot.transAxes,
        fontsize=8,
        color="white",
        bbox={"boxstyle": "round,pad=0.25", "facecolor": (0, 0, 0, 0.35), "edgecolor": "none"},
    )
    plt.colorbar(im_vdot, ax=ax_vdot, fraction=0.046, pad=0.04)


def main() -> None:
    g = 9.81
    l = 1.0

    phi = np.linspace(-math.pi, math.pi, 320)
    phi_dot = np.linspace(-6.0, 6.0, 320)
    PHI, PHI_DOT = np.meshgrid(phi, phi_dot)

    theta = np.linspace(-math.pi, math.pi, 320)
    theta_dot = np.linspace(-4.0, 4.0, 320)
    THETA, THETA_DOT = np.meshgrid(theta, theta_dot)

    fig, axes = plt.subplots(4, 2, figsize=(16, 22), constrained_layout=True)
    fig.suptitle("Lyapunov and Energy Functions Used in the Project", fontsize=18)

    # 1. Vertical averaged Kapitza energy
    alpha_avg = 100.0
    omega_avg = 18.0
    d_avg = 0.25
    V1 = vertical_averaged_energy(PHI, PHI_DOT, alpha_avg, omega_avg, g, l)
    Vdot1 = vertical_averaged_vdot(PHI, PHI_DOT, d_avg)
    plot_pair(
        axes[0, 0],
        axes[0, 1],
        PHI,
        PHI_DOT,
        V1,
        Vdot1,
        "Vertical averaged Kapitza energy",
        r"$\varphi$ [rad]",
        r"$\dot{\varphi}$ [rad/s]",
        r"$V=\frac12\dot\varphi^2+\frac{\alpha^2}{4l^2\omega^2}\sin^2\varphi+\frac{g}{l}(\cos\varphi-1)$",
        r"$\dot V=-d\,\dot\varphi^2$",
        0.0,
        0.0,
    )

    # 2. Vertical closed-loop averaged CLF
    k0 = 0.35
    k1 = 2.35
    d_clf = 0.25
    V2 = vertical_clf(PHI, PHI_DOT, k0, k1)
    Vdot2 = vertical_clf_vdot(PHI, PHI_DOT, d_clf)
    plot_pair(
        axes[1, 0],
        axes[1, 1],
        PHI,
        PHI_DOT,
        V2,
        Vdot2,
        "Vertical closed-loop CLF",
        r"$\varphi$ [rad]",
        r"$\dot{\varphi}$ [rad/s]",
        r"$V_{\rm cl}=\frac12\dot\varphi^2+\frac12 k_0\varphi^2+\frac14 k_1\varphi^4$",
        r"$\dot V_{\rm cl}=-d\,\dot\varphi^2$",
        0.0,
        0.0,
    )

    # 3. Vertical direct Lyapunov function
    kp = 2.0
    kc = 80.0
    d_dir = 0.25
    V3 = vertical_direct_lyapunov(PHI, PHI_DOT, kp)
    Vdot3 = vertical_direct_lyapunov_vdot(PHI, PHI_DOT, d_dir, kc)
    plot_pair(
        axes[2, 0],
        axes[2, 1],
        PHI,
        PHI_DOT,
        V3,
        Vdot3,
        "Vertical direct Lyapunov function",
        r"$\varphi$ [rad]",
        r"$\dot{\varphi}$ [rad/s]",
        r"$V_{\rm dir}=\frac12\dot\varphi^2+k_p(1-\cos\varphi)$",
        r"$\dot V_{\rm dir}=-(d+k_c\sin^2\varphi)\dot\varphi^2$",
        0.0,
        0.0,
    )

    # 4. Horizontal averaged effective energy
    alpha_h = 800.0
    omega_h = 50.0
    d_h = 0.5
    theta_star = math.acos(2.0 * g * l * omega_h**2 / alpha_h**2)
    V4 = horizontal_effective_energy(THETA, THETA_DOT, alpha_h, omega_h, g, l)
    Vdot4 = horizontal_effective_vdot(THETA, THETA_DOT, d_h)
    plot_pair(
        axes[3, 0],
        axes[3, 1],
        THETA,
        THETA_DOT,
        V4,
        Vdot4,
        "Horizontal averaged effective energy",
        r"$\theta$ [rad]",
        r"$\dot{\theta}$ [rad/s]",
        r"$V_h=\frac12\dot\theta^2-\frac{g}{l}\cos\theta-\frac{\alpha^2}{4l^2\omega^2}\sin^2\theta$",
        r"$\dot V_h=-d\,\dot\theta^2$",
        theta_star,
        0.0,
    )

    output_png = PROJECT_ROOT / "figures" / "lyapunov_functions_summary_matplotlib.png"
    output_pdf = PROJECT_ROOT / "figures" / "lyapunov_functions_summary_matplotlib.pdf"
    output_png.parent.mkdir(exist_ok=True)
    fig.savefig(output_png, dpi=220)
    fig.savefig(output_pdf)
    plt.close(fig)

    print(f"Saved figure to: {output_png}")
    print(f"Saved figure to: {output_pdf}")


if __name__ == "__main__":
    main()
