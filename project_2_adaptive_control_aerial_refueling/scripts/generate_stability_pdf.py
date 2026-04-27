from __future__ import annotations

import textwrap
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "adaptive_control_stability_proof.pdf"


def _text(ax, x: float, y: float, text: str, size: int = 10, weight: str = "normal") -> float:
    wrapped: list[str] = []
    for paragraph in text.split("\n"):
        if not paragraph.strip():
            wrapped.append("")
        else:
            wrapped.extend(textwrap.wrap(paragraph, width=92))
    block = "\n".join(wrapped)
    ax.text(
        x,
        y,
        block,
        fontsize=size,
        fontfamily="serif",
        fontweight=weight,
        va="top",
        transform=ax.transAxes,
    )
    return y - 0.023 * max(1, len(wrapped)) - 0.014


def _equation(ax, x: float, y: float, equation: str, size: int = 15) -> float:
    ax.text(
        0.5,
        y,
        equation,
        fontsize=size,
        fontfamily="serif",
        ha="center",
        va="top",
        transform=ax.transAxes,
    )
    return y - 0.066


def _section(ax, y: float, title: str) -> float:
    ax.text(
        0.08,
        y,
        title,
        fontsize=14,
        fontfamily="serif",
        fontweight="bold",
        va="top",
        transform=ax.transAxes,
    )
    return y - 0.05


def _page(pdf: PdfPages, title: str, content) -> None:
    fig = plt.figure(figsize=(8.27, 11.69))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.text(0.08, 0.955, title, fontsize=18, fontfamily="serif", fontweight="bold", va="top", transform=ax.transAxes)
    ax.text(0.08, 0.925, "Adaptive aerial-refueling station keeping", fontsize=10, fontfamily="serif", color="0.35", va="top", transform=ax.transAxes)
    y = 0.875
    y = content(ax, y)
    ax.text(0.08, 0.04, "Generated from scripts/generate_stability_pdf.py", fontsize=8, fontfamily="serif", color="0.45", transform=ax.transAxes)
    pdf.savefig(fig)
    plt.close(fig)


def _page_model(ax, y: float) -> float:
    y = _section(ax, y, "1. Reduced Relative Model")
    y = _text(
        ax,
        0.08,
        y,
        "The tanker-relative frame is fixed at the desired refueling point. "
        "The controlled point is the receiver-side refueling point. The state is "
        "the relative position r and relative velocity v in the longitudinal-vertical plane.",
    )
    y = _equation(ax, 0.12, y, r"$r=[x,z]^T,\quad v=\dot r=[v_x,v_z]^T,\quad u=[u_T,u_L]^T.$")
    y = _text(
        ax,
        0.08,
        y,
        "After subtracting the nominal trimmed forces, the perturbation model is",
    )
    y = _equation(ax, 0.12, y, r"$\dot r=v,\qquad m(t)\dot v=u-Cv-(m(t)-m_0)g\xi_z,$")
    y = _equation(ax, 0.12, y, r"$C=\mathrm{diag}(c_x,c_z),\qquad \xi_z=[0,1]^T.$")
    y = _text(
        ax,
        0.08,
        y,
        "The unknown scalar parameter used by the adaptive controller is the inverse mass.",
    )
    y = _equation(ax, 0.12, y, r"$\theta(t)=\frac{1}{m(t)}.$")
    y = _text(
        ax,
        0.08,
        y,
        "Then the acceleration equation can be written in the adaptive-control form",
    )
    y = _equation(ax, 0.12, y, r"$\dot v=\theta\,\phi(s,u)-g\xi_z,\qquad \phi(s,u)=u-Cv+m_0g\xi_z.$")
    return y


def _page_controller(ax, y: float) -> float:
    y = _section(ax, y, "2. Error Variable and Control Law")
    y = _text(
        ax,
        0.08,
        y,
        "Use a filtered tracking error. If this error converges to zero, the position dynamics become stable first-order dynamics.",
    )
    y = _equation(ax, 0.12, y, r"$e=v+\Lambda r,\qquad \Lambda=\mathrm{diag}(\lambda_x,\lambda_z)>0.$")
    y = _equation(ax, 0.12, y, r"$e=0\quad\Rightarrow\quad \dot r=-\Lambda r\quad\Rightarrow\quad r(t)\to 0.$")
    y = _text(ax, 0.08, y, "The derivative of the filtered error is")
    y = _equation(ax, 0.12, y, r"$\dot e=\dot v+\Lambda v=\theta\phi(s,u)-g\xi_z+\Lambda v.$")
    y = _text(
        ax,
        0.08,
        y,
        "For known theta, the target dynamics are chosen as dot e = -K e. Replacing theta by its estimate gives the certainty-equivalent law",
    )
    y = _equation(ax, 0.12, y, r"$u=Cv-m_0g\xi_z+\frac{1}{\hat\theta}\left(-Ke-\Lambda v+g\xi_z\right),$", size=14)
    y = _equation(ax, 0.12, y, r"$K=\mathrm{diag}(k_x,k_z)>0.$")
    y = _text(
        ax,
        0.08,
        y,
        "The estimate is projected to a physically meaningful inverse-mass interval to avoid division by zero and nonphysical masses.",
    )
    y = _equation(ax, 0.12, y, r"$\hat\theta\in[\theta_{\min},\theta_{\max}],\qquad \hat m=\frac{1}{\hat\theta}.$")
    return y


def _page_stability(ax, y: float) -> float:
    y = _section(ax, y, "3. Constant-Mass Lyapunov Proof")
    y = _text(
        ax,
        0.08,
        y,
        "First assume that the mass is unknown but constant, so theta is constant. "
        "Let the parameter error be",
    )
    y = _equation(ax, 0.12, y, r"$\tilde\theta=\theta-\hat\theta.$")
    y = _text(ax, 0.08, y, "Substituting the adaptive control law into the plant gives")
    y = _equation(ax, 0.12, y, r"$\dot e=-Ke+\tilde\theta\,\phi(s,u).$")
    y = _text(ax, 0.08, y, "Choose the augmented Lyapunov function")
    y = _equation(ax, 0.12, y, r"$L(e,\tilde\theta)=\frac{1}{2}e^Te+\frac{1}{2\gamma}\tilde\theta^2,\qquad \gamma>0.$", size=14)
    y = _text(
        ax,
        0.08,
        y,
        "This function is positive definite in the filtered tracking error and the scalar parameter error:",
    )
    y = _equation(ax, 0.12, y, r"$L(e,\tilde\theta)>0\quad\mathrm{for}\quad (e,\tilde\theta)\ne(0,0),\qquad L(0,0)=0.$", size=13)
    y = _text(ax, 0.08, y, "Since theta is constant, dot tilde theta = - dot hat theta. Therefore")
    y = _equation(ax, 0.12, y, r"$\dot L=e^T(-Ke+\tilde\theta\phi)-\frac{1}{\gamma}\tilde\theta\dot{\hat\theta}.$", size=14)
    y = _equation(ax, 0.12, y, r"$\dot L=-e^TKe+\tilde\theta\left(e^T\phi-\frac{1}{\gamma}\dot{\hat\theta}\right).$", size=14)
    y = _text(ax, 0.08, y, "Choose the adaptation law")
    y = _equation(ax, 0.12, y, r"$\dot{\hat\theta}=\gamma e^T\phi(s,u).$")
    y = _text(ax, 0.08, y, "Then the cross term cancels and")
    y = _equation(ax, 0.12, y, r"$\dot L=-e^TKe\leq 0.$")
    y = _text(
        ax,
        0.08,
        y,
        "Thus L is nonincreasing, e and tilde theta are bounded, and e is square integrable. "
        "Under the standard boundedness assumptions for direct adaptive control, Barbalat's lemma implies e(t) -> 0. "
        "Since dot r = -Lambda r + e and Lambda is positive definite, the relative position also converges to the origin.",
    )
    y = _equation(ax, 0.12, y, r"$e(t)\to 0,\qquad \dot r=-\Lambda r+e,\qquad r(t)\to 0.$")
    return y


def _page_time_varying(ax, y: float) -> float:
    y = _section(ax, y, "4. Slowly Time-Varying Mass")
    y = _text(
        ax,
        0.08,
        y,
        "During refueling the mass is not exactly constant. The inverse mass theta(t)=1/m(t) slowly varies because the fuel flow is bounded. "
        "The same Lyapunov calculation now includes one additional term.",
    )
    y = _equation(ax, 0.12, y, r"$\dot{\tilde\theta}=\dot\theta-\dot{\hat\theta},$")
    y = _equation(ax, 0.12, y, r"$\dot L=-e^TKe+\frac{1}{\gamma}\tilde\theta\dot\theta.$")
    y = _text(
        ax,
        0.08,
        y,
        "The first term is dissipative. The second term is a bounded disturbance term caused by fuel transfer. "
        "Therefore the result is practical stability or ultimate boundedness: the tracking error remains bounded and small when the mass changes slowly relative to the closed-loop response.",
    )
    y = _section(ax, y - 0.01, "5. Discrete Implementation")
    y = _text(ax, 0.08, y, "The simulation uses an explicit projected update after each integration step:")
    y = _equation(ax, 0.12, y, r"$\hat\theta_{k+1}=\mathrm{proj}_{[\theta_{\min},\theta_{\max}]}\left(\hat\theta_k+\Delta t\,\gamma e_k^T\phi(s_k,u_k)\right).$", size=12)
    y = _equation(ax, 0.12, y, r"$\hat m_k=\frac{1}{\hat\theta_k},\qquad u_k=\mathrm{sat}_{[-u_{\max},u_{\max}]}\{u(s_k,\hat\theta_k)\}.$", size=13)
    y = _text(
        ax,
        0.08,
        y,
        "The force command is saturated by the chosen aircraft-scale authority limit. The saturation is used in simulation to keep the command physically meaningful; the analytical proof above corresponds to the unsaturated adaptive law.",
    )
    return y


def main() -> None:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update(
        {
            "font.family": "serif",
            "mathtext.fontset": "cm",
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )
    with PdfPages(OUT) as pdf:
        _page(pdf, "Adaptive Controller: Model and Stability", _page_model)
        _page(pdf, "Adaptive Controller: Design", _page_controller)
        _page(pdf, "Adaptive Controller: Lyapunov Proof", _page_stability)
        _page(pdf, "Adaptive Controller: Refueling Case", _page_time_varying)


if __name__ == "__main__":
    main()
