# Adaptive Control for Aerial Refueling Station Keeping

This project studies an adaptive controller for a simplified aerial refueling
task. The receiver aircraft must stay close to a fixed refueling point in the
relative frame of the tanker while its mass increases during fuel transfer.

The central control difficulty is that the same aerodynamic correction force
produces a smaller acceleration when the aircraft becomes heavier. A controller
that assumes a fixed nominal mass may therefore become too weak during
refueling. The adaptive controller estimates the inverse mass online and uses
this estimate to scale the control forces.

## Problem Definition

We consider the receiver aircraft in the vertical longitudinal plane. The
coordinate frame is attached to the tanker and moves with its nominal refueling
flight condition. The desired refueling point is fixed at the origin of this
relative frame.

More precisely, the origin is placed at the center of the desired tanker-side
refueling contact region, for example the center of the drogue/basket. The
controlled point on the receiver is the receiver-side refueling point, such as
the probe tip. In this reduced model, the aircraft attitude and the fixed
geometric offset between the aircraft center of mass and the probe tip are
handled by lower-level loops, so the high-level station-keeping state describes
the receiver-side refueling point directly.

<a href="figures/refueling_model_forces.png">
  <img src="figures/refueling_model_forces.png" width="100%" alt="Aerial refueling model with coordinates, control forces, damping, and gravity mismatch">
</a>

The control objective is

$$
x(t)\to 0,\qquad z(t)\to 0,\qquad v_x(t)\to 0,\qquad v_z(t)\to 0,
$$

while the receiver mass changes due to fuel inflow.

The state is

$$
s =
\begin{bmatrix}
x \\ z \\ v_x \\ v_z
\end{bmatrix},
$$

where

- $x$ is the longitudinal displacement of the receiver-side refueling point
  from the desired tanker-side refueling point,
- $z$ is the vertical displacement of the receiver-side refueling point from
  the desired tanker-side refueling point,
- $v_x=\dot x$ is the relative longitudinal velocity,
- $v_z=\dot z$ is the relative vertical velocity.

We also use the compact notation

$$
r =
\begin{bmatrix}
x \\ z
\end{bmatrix},
\qquad
v =
\begin{bmatrix}
v_x \\ v_z
\end{bmatrix}.
$$

The refueling zone is represented by a circle

$$
\|r(t)\| = \sqrt{x(t)^2+z(t)^2}\le R.
$$

In this first version, the circle is used as a performance metric, not as a hard
safety constraint.

The default radius is set to

$$
R=1\ \mathrm{m}.
$$

This value is chosen as a realistic order-of-magnitude refueling window: the
diameter of the refueling basket or drogue used by tanker aircraft is about one
meter. Therefore, keeping the receiver-side refueling point within a 1 m
neighborhood is a meaningful station-keeping requirement for this simplified
model.

The control inputs in this model are incremental force corrections. Their
values can reach tens or hundreds of thousands of newtons. This is still a
realistic aircraft-scale order of magnitude: for example, the thrust of a jet
fighter engine can be around $330\,000$ N, so the simulated force corrections
are not outside the range of physically meaningful aircraft control authority.

## Modeling Assumptions

The full dynamics of an aircraft include pitch, angle of attack, elevator
deflection, engine dynamics, lift curves, drag curves, actuator limits, and
many other effects. Modeling all of these would make the project much larger
than the adaptive-control objective.

We therefore use a reduced-order perturbation model around a nominal refueling
flight condition.

The assumptions are:

- the tanker-relative refueling point is fixed in the $(x,z)$ frame;
- the receiver remains close to the nominal refueling flight condition;
- low-level aircraft loops track commanded thrust and lift corrections;
- the high-level controller commands only incremental force corrections;
- the mass increases during refueling and is not exactly known to the
  controller;
- wind is not included in the first controller design.

The commanded control input is

$$
u =
\begin{bmatrix}
u_T \\ u_L
\end{bmatrix},
$$

where

- $u_T$ is the incremental longitudinal thrust correction,
- $u_L$ is the incremental vertical lift correction.

These are not the full engine thrust and full wing lift. They are high-level
force corrections around the nominal trimmed flight regime.

## From Full Forces to the Perturbation Model

A simple longitudinal-vertical force model may be written as

$$
m(t)\dot v_x = T - D_x,
$$

$$
m(t)\dot v_z = L - m(t)g - D_z,
$$

where

- $T$ is thrust,
- $L$ is lift,
- $D_x$ and $D_z$ are aerodynamic damping or drag-like terms,
- $g$ is gravitational acceleration,
- $m(t)$ is the receiver mass.

In nominal trimmed flight before refueling, the aircraft is assumed to satisfy

$$
T_0 = D_{x,0},
\qquad
L_0 = m_0 g,
$$

where $m_0$ is the initial mass. This means that the nominal thrust balances
drag and the nominal lift balances weight.

During refueling, we write

$$
T = T_0 + u_T,
\qquad
L = L_0 + u_L.
$$

For small relative velocities around the nominal flight condition, the drag and
damping corrections are approximated by a linear model:

$$
D_x-D_{x,0}\approx c_x v_x,
\qquad
D_z\approx c_z v_z,
$$

where $c_x>0$ and $c_z>0$ are linearized damping coefficients. This is not a
claim that aerodynamic drag is globally linear. It is a local first-order model
around the refueling flight condition.

Substituting these relations into the force balance gives

$$
m(t)\dot v_x = u_T - c_x v_x,
$$

$$
m(t)\dot v_z = u_L - c_z v_z - (m(t)-m_0)g.
$$

Thus the increasing mass has two effects:

1. it reduces acceleration per unit control force;
2. it creates a vertical gravity mismatch because the old nominal lift
   $L_0=m_0g$ no longer balances the new weight $m(t)g$.

## State-Space Model

Define

$$
C =
\begin{bmatrix}
c_x & 0 \\
0 & c_z
\end{bmatrix},
\qquad
\xi_z =
\begin{bmatrix}
0 \\ 1
\end{bmatrix}.
$$

The model is

$$
\dot r = v,
$$

$$
m(t)\dot v = u - Cv - (m(t)-m_0)g \xi_z.
$$

Equivalently,

$$
\dot v = \theta(t)u - \theta(t)Cv - \bigl(1-m_0\theta(t)\bigr)g \xi_z,
$$

where the inverse mass is

$$
\theta(t)=\frac{1}{m(t)}.
$$

In component form:

$$
\dot x = v_x,
$$

$$
\dot z = v_z,
$$

$$
\dot v_x = \theta(t)(u_T-c_xv_x),
$$

$$
\dot v_z = \theta(t)(u_L-c_zv_z) - \bigl(1-m_0\theta(t)\bigr)g.
$$

The fuel inflow changes the true mass according to

$$
m(t)=m_0+\int_0^t q(\tau)d\tau,
$$

where $q(t)$ is the fuel mass flow rate. The controller may know a nominal fuel
flow profile, but it does not know the exact true mass online.

## Implementation

The adaptive controller is implemented in `src/controllers/adaptive_inverse_mass.py`.
The plant model is implemented in `src/system.py`, and the RK4 simulator is in
`src/simulation.py`.

The project currently contains:

```text
project_2_adaptive_control_aerial_refueling/
+-- README.md
+-- requirements.txt
+-- configs/
|   +-- default.json
+-- figures/
|   +-- refueling_relative_model.svg
|   +-- refueling_model_forces.png
|   +-- zero_controller_tracking_summary.png
|   +-- zero_controller_diagnostics.png
|   +-- zero_controller_phase_portraits.png
|   +-- pd_controller_tracking_summary.png
|   +-- pd_controller_diagnostics.png
|   +-- pd_controller_phase_portraits.png
|   +-- adaptive_controller_tracking_summary.png
|   +-- adaptive_controller_diagnostics.png
|   +-- adaptive_controller_phase_portraits.png
|   +-- controller_comparison.png
+-- results/
|   +-- adaptive_summary.json
+-- scripts/
|   +-- generate_all.py
|   +-- generate_figures.py
+-- src/
    +-- config.py
    +-- main.py
    +-- simulation.py
    +-- system.py
    +-- controllers/
        +-- adaptive_inverse_mass.py
        +-- base.py
        +-- pd_controller.py
        +-- zero_controller.py
```

## Reproducibility

Install dependencies:

```powershell
python -m pip install -r requirements.txt
```

Run the adaptive simulation and print the final summary:

```powershell
python src/main.py
```

Generate all figures and the JSON summary:

```powershell
python scripts/generate_all.py
```

The generated numerical summary is stored in
`results/adaptive_summary.json`.

## Generated Results

Each controller is first shown separately, then the controllers are compared on
the same initial condition.

### Zero Controller Results

The Zero controller has no tunable feedback parameters. Its command is
$u(t)=0$, so the aircraft is affected only by its initial relative velocity,
damping, and the vertical gravity mismatch caused by increasing mass.

Substituting $u=0$ into the plant gives

$$
\dot r = v,
$$

$$
\dot v = -\theta(t)Cv-\bigl(1-m_0\theta(t)\bigr)g\xi_z.
$$

The horizontal velocity is damped by $-\theta(t)c_xv_x$, but there is no term
that drives $x$ to zero. In the vertical channel, the term
$-\bigl(1-m_0\theta(t)\bigr)g\xi_z$ becomes nonzero as soon as $m(t)>m_0$,
which creates a downward acceleration mismatch.

Algorithmically, the Zero controller is just:

```text
given state s = [x, z, vx, vz]^T

u_T <- 0
u_L <- 0
return u = [u_T, u_L]^T
```

<a href="figures/zero_controller_tracking_summary.png">
  <img src="figures/zero_controller_tracking_summary.png" width="100%" alt="Zero controller tracking summary">
</a>

The plots show that the receiver does not approach the refueling point. The
vertical coordinate drifts away because the nominal lift $m_0g$ no longer
balances the increasing weight $m(t)g$. This confirms that the plant model is
not self-stabilizing for the refueling task.

The longitudinal velocity is initially negative because the default initial
condition is $v_x(0)=-1$ m/s: the receiver-side refueling point starts moving
toward the tanker-side target. With $u_T=0$, the longitudinal equation
$\dot v_x=-\theta c_xv_x$ only damps this velocity, so $v_x$ approaches zero
from below. The short nearly constant-height segment appears because
$v_z(0)=0.6$ m/s is upward and the mass mismatch is initially zero at
$m(0)=m_0$. As fuel is added, $m(t)>m_0$, the term
$-\bigl(1-m_0\theta(t)\bigr)g\xi_z$ grows, and the receiver begins drifting
downward.

<a href="figures/zero_controller_diagnostics.png">
  <img src="figures/zero_controller_diagnostics.png" width="90%" alt="Zero controller diagnostics">
</a>

The force commands are identically zero, as expected. The mass plot is included
to show that the same refueling scenario is used for all controllers.

<a href="figures/zero_controller_phase_portraits.png">
  <img src="figures/zero_controller_phase_portraits.png" width="100%" alt="Zero controller phase portraits">
</a>

The phase portraits confirm the same behavior from multiple initial
conditions: without a stabilizing feedback term, trajectories do not converge
to the origin.

Final Zero-controller result:

```json
{
  "final_position_norm_m": 1096.019940085246,
  "final_velocity_norm_m_s": 18.07046360725809,
  "entered_refueling_zone": false
}
```

### PD Controller Results

The nominal-mass PD controller uses position and velocity feedback but does
not adapt the mass estimate. It assumes the nominal mass $m_0$ and commands

$$
a_{\mathrm{PD}} = -K_p r - K_d v,
$$

where $a_{\mathrm{PD}}\in\mathbb{R}^2$ is the desired acceleration,
$K_p=\operatorname{diag}(k_{p,x},k_{p,z})$ is the proportional gain matrix, and
$K_d=\operatorname{diag}(k_{d,x},k_{d,z})$ is the derivative gain matrix. The
implemented force command is

$$
u_{\mathrm{PD}} = Cv + m_0 a_{\mathrm{PD}}.
$$

Substituting this law into the true plant gives

$$
\dot v
=
\theta(t)m_0(-K_pr-K_dv)
-\bigl(1-m_0\theta(t)\bigr)g\xi_z.
$$

If $m(t)=m_0$, then $\theta(t)m_0=1$ and the main feedback term becomes the
standard second-order PD dynamics. During refueling $m(t)>m_0$, therefore
$\theta(t)m_0<1$: the feedback action is effectively weakened, and the vertical
gravity mismatch remains uncompensated.

The nominal-mass PD controller uses the following gains:

```json
{
  "kp_x": 0.09,
  "kp_z": 0.136,
  "kd_x": 0.60,
  "kd_z": 0.74,
  "nominal_mass_kg": 12000.0,
  "max_force_N": 180000.0
}
```

The implemented PD algorithm is:

```text
given state s = [x, z, vx, vz]^T

r <- [x, z]^T
v <- [vx, vz]^T
a_PD <- -Kp r - Kd v
u <- C v + m0 a_PD
u <- saturate(u, -u_max, u_max)
return u
```

<a href="figures/pd_controller_tracking_summary.png">
  <img src="figures/pd_controller_tracking_summary.png" width="100%" alt="PD controller tracking summary">
</a>

The PD controller damps the relative motion and brings the aircraft much closer
to the refueling point than the Zero controller. However, the final error is
still outside the required radius $R=1$ m. This happens because the controller
uses the fixed nominal mass $m_0$, while the real plant becomes heavier during
refueling.

<a href="figures/pd_controller_diagnostics.png">
  <img src="figures/pd_controller_diagnostics.png" width="90%" alt="PD controller diagnostics">
</a>

The force commands remain smooth, but they are computed with a fixed
mass-to-acceleration conversion. As the mass grows, the same force produces a
smaller acceleration, so the simple PD loop cannot remove the residual offset
in this scenario.

<a href="figures/pd_controller_phase_portraits.png">
  <img src="figures/pd_controller_phase_portraits.png" width="100%" alt="PD controller phase portraits">
</a>

The phase portraits show damped motion, but convergence is toward a biased
region rather than the exact refueling equilibrium. This matches the residual
distance seen in the tracking plot.

Final PD-controller result:

```json
{
  "final_position_norm_m": 28.986177356773606,
  "final_velocity_norm_m_s": 0.25672385333305103,
  "entered_refueling_zone": false
}
```

### Adaptive Controller Results

#### Adaptive-Control Form

The dynamics can be written in a form convenient for certainty-equivalence
adaptive control:

$$
\dot r = v,
$$

$$
\dot v = \theta \phi(s,u) - g \xi_z,
$$

where

$$
\phi(s,u)=u-Cv+m_0g \xi_z.
$$

Indeed,

$$
\theta\phi(s,u)-g \xi_z
=
\theta(u-Cv+m_0g \xi_z)-g \xi_z
=
\theta(u-Cv)-\bigl(1-m_0\theta\bigr)g \xi_z.
$$

The unknown adaptive parameter is

$$
\theta=\frac{1}{m}.
$$

The controller maintains an estimate $\hat\theta$ and the corresponding
physical mass estimate is reported as

$$
\hat m = \frac{1}{\hat\theta}.
$$

To avoid division by zero and nonphysical estimates, the estimate is projected
to an interval

$$
\hat\theta\in[\theta_{\min},\theta_{\max}],
$$

which corresponds to

$$
\hat m\in[m_{\min},m_{\max}].
$$

Here $\theta_{\min}$ and $\theta_{\max}$ are the lower and upper allowed
inverse-mass estimates, while $m_{\min}$ and $m_{\max}$ are the corresponding
physical mass bounds.

#### Tracking Error

The desired refueling point is fixed at the origin. We introduce the filtered
tracking error

$$
e = v+\Lambda r,
$$

where

$$
\Lambda =
\begin{bmatrix}
\lambda_x & 0 \\
0 & \lambda_z
\end{bmatrix},
\qquad
\lambda_x>0,\quad \lambda_z>0.
$$

If $e\to 0$, then

$$
\dot r = v = -\Lambda r,
$$

so the relative position converges to the refueling point.

The derivative of $e$ is

$$
\dot e
=
\dot v+\Lambda\dot r
=
\theta\phi(s,u)-g \xi_z+\Lambda v.
$$

#### Certainty-Equivalent Control Law

The target error dynamics are chosen as

$$
\dot e = -K e,
$$

where

$$
K =
\begin{bmatrix}
k_x & 0 \\
0 & k_z
\end{bmatrix},
\qquad
k_x>0,\quad k_z>0.
$$

If $\theta$ were known, the desired cancellation condition would be

$$
\theta\phi(s,u)-g \xi_z+\Lambda v = -K e.
$$

Hence

$$
\theta\phi(s,u)
=
-K e-\Lambda v+g \xi_z.
$$

Since $\theta$ is unknown, we use the estimate $\hat\theta$:

$$
\phi(s,u)
=
\frac{1}{\hat\theta}
\left(
-K e-\Lambda v+g \xi_z
\right).
$$

Recalling that

$$
\phi(s,u)=u-Cv+m_0g \xi_z,
$$

the adaptive control law is

$$
u
=
Cv-m_0g \xi_z
+
\frac{1}{\hat\theta}
\left(
-K e-\Lambda v+g \xi_z
\right).
$$

This is the force command implemented by the high-level adaptive controller.

In component form:

$$
u_T
=
c_xv_x
+
\frac{1}{\hat\theta}
\left(
-k_x e_x-\lambda_x v_x
\right),
$$

$$
u_L
=
c_zv_z-m_0g
+
\frac{1}{\hat\theta}
\left(
-k_z e_z-\lambda_z v_z+g
\right).
$$

Here $e_x$ and $e_z$ denote the two components of the filtered error $e$. The
vertical unit vector is denoted by $\xi_z$ and will be named `vertical_unit` in
the code.

#### Closed-Loop Error Dynamics

Substituting the controller into the true plant gives

$$
\dot e
=
-K e
+
\tilde\theta \phi(s,u),
$$

where

$$
\tilde\theta=\theta-\hat\theta.
$$

The nominal stabilizing term $-Ke$ is perturbed by a cross-term containing the
parameter estimation error.

#### Lyapunov Function

The stability proof for the adaptive controller is based on direct Lyapunov
analysis. The proof has four steps: write the closed-loop error dynamics,
choose an augmented Lyapunov function, select the adaptation law so that the
parameter-error cross-term cancels, and then conclude boundedness and
convergence of the tracking error.

Use the augmented Lyapunov function

$$
L(e,\tilde\theta)
=
\frac{1}{2}e^Te
+
\frac{1}{2\gamma}\tilde\theta^2,
$$

where $\gamma>0$ is the adaptation gain.

For a constant unknown mass, $\dot\theta=0$, and therefore

$$
\dot{\tilde\theta}
=
-\dot{\hat\theta}.
$$

The derivative of $L$ along the closed-loop trajectories is

$$
\dot L
=
e^T\dot e
+
\frac{1}{\gamma}\tilde\theta\dot{\tilde\theta}.
$$

Substituting the closed-loop error dynamics,

$$
\dot L
=
e^T(-Ke+\tilde\theta\phi)
+
\frac{1}{\gamma}\tilde\theta(-\dot{\hat\theta}).
$$

Thus

$$
\dot L
=
-e^TKe
+
\tilde\theta e^T\phi
-
\frac{1}{\gamma}\tilde\theta\dot{\hat\theta}.
$$

The cross-term is canceled by choosing

$$
\dot{\hat\theta}
=
\gamma e^T\phi(s,u).
$$

Then

$$
\dot L
=
-e^TKe
\le 0.
$$

This is the key stability result: the Lyapunov function is positive definite
in $(e,\tilde\theta)$ and its derivative is negative semidefinite. Therefore
the augmented error system is Lyapunov stable for a constant unknown mass.
Since $K$ is positive definite, $e\in L_2\cap L_\infty$. Under standard
boundedness assumptions on the signals, Barbalat's lemma or LaSalle-type
arguments imply

$$
e(t)\to 0.
$$

Because

$$
\dot r = -\Lambda r + e,
$$

and $\Lambda$ is positive definite, the relative position also converges:

$$
r(t)\to 0.
$$

#### Time-Varying Mass During Refueling

In the actual refueling problem, $m(t)$ is not constant. Therefore

$$
\theta(t)=\frac{1}{m(t)}
$$

is slowly time-varying. In this case

$$
\dot{\tilde\theta}
=
\dot\theta-\dot{\hat\theta}.
$$

The Lyapunov derivative becomes

$$
\dot L
=
-e^TKe
+
\frac{1}{\gamma}\tilde\theta\dot\theta.
$$

If the fuel flow is bounded and slow, then $|\dot\theta|$ is bounded and small.
Therefore the adaptive controller gives practical stability: the tracking error
remains bounded and can be made small when the mass changes slowly relative to
the closed-loop dynamics.

The constant-parameter case gives asymptotic stability, while the slowly
time-varying case gives ultimate boundedness.

#### Adaptation Law Used in Simulation

The implemented adaptation law will be

$$
\dot{\hat\theta}
=
\gamma e^T\phi(s,u),
$$

with projection:

$$
\hat\theta
\leftarrow
\operatorname{proj}_{[\theta_{\min},\theta_{\max}]}
(\hat\theta).
$$

Here $\operatorname{proj}_{[\theta_{\min},\theta_{\max}]}(\cdot)$ denotes
projection onto the interval $[\theta_{\min},\theta_{\max}]$.

In discrete time with step $\Delta t$:

$$
\hat\theta_{k+1}
=
\operatorname{proj}_{[\theta_{\min},\theta_{\max}]}
\left(
\hat\theta_k
+
\Delta t\,\gamma e_k^T\phi(s_k,u_k)
\right).
$$

The mass estimate plotted in the results is

$$
\hat m_k=\frac{1}{\hat\theta_k}.
$$


The adaptive inverse-mass controller uses the following parameters:

```json
{
  "lambda_x": 0.30,
  "lambda_z": 0.34,
  "k_x": 0.30,
  "k_z": 0.40,
  "gamma": 5.0e-12,
  "mass_hat_initial_kg": 12000.0,
  "mass_min_kg": 8000.0,
  "mass_max_kg": 18000.0,
  "max_force_N": 180000.0
}
```

The implemented adaptive algorithm is:

```text
given state s = [x, z, vx, vz]^T and current estimate theta_hat

r <- [x, z]^T
v <- [vx, vz]^T
e <- v + Lambda r
desired <- -K e - Lambda v + g xi_z
u <- C v - m0 g xi_z + desired / theta_hat
u <- saturate(u, -u_max, u_max)
phi <- u - C v + m0 g xi_z
theta_hat_dot <- gamma e^T phi
theta_hat <- project(theta_hat + dt theta_hat_dot, theta_min, theta_max)
return u
```

<a href="figures/adaptive_controller_tracking_summary.png">
  <img src="figures/adaptive_controller_tracking_summary.png" width="100%" alt="Adaptive controller tracking summary">
</a>

The adaptive controller drives the receiver into the refueling zone and keeps
it there. The velocity plots show that the transient motion is damped, while
the distance plot shows convergence below the required radius $R=1$ m.

<a href="figures/adaptive_controller_diagnostics.png">
  <img src="figures/adaptive_controller_diagnostics.png" width="95%" alt="Adaptive force commands, mass estimate, and Lyapunov function">
</a>

The adaptive diagnostics show smooth force commands, a mass estimate that
tracks the increasing mass trend, and a decreasing augmented Lyapunov value.
The estimate $\hat m(t)$ is used for stabilization; it is not claimed to be an
exact identifier of the physical mass.

<a href="figures/adaptive_controller_phase_portraits.png">
  <img src="figures/adaptive_controller_phase_portraits.png" width="100%" alt="Adaptive controller phase portraits">
</a>

The phase portraits support the Lyapunov-based analysis: trajectories from
different initial states move toward the equilibrium region instead of
diverging.

Final adaptive-controller result:

```json
{
  "final_position_norm_m": 0.5230896452030871,
  "final_velocity_norm_m_s": 0.022440296640751662,
  "final_mass_kg": 17071.868276983434,
  "final_mass_hat_kg": 16920.54349573463,
  "final_lyapunov": 0.0475010587302382,
  "entered_refueling_zone": true,
  "stays_in_zone_after_first_entry": true
}
```

### Controller Comparison

<a href="figures/controller_comparison.png">
  <img src="figures/controller_comparison.png" width="100%" alt="Comparison between Zero, PD, and adaptive controllers">
</a>

The comparison summarizes the role of adaptation. The Zero controller fails
because there is no corrective action. The nominal-mass PD controller stabilizes
the motion qualitatively, but it remains outside the 1 m refueling zone because
the fixed mass assumption becomes inaccurate. The adaptive controller updates
the inverse-mass estimate online and achieves the required station-keeping
accuracy.

## References

- H. K. Khalil, *Nonlinear Systems*, 3rd ed., Prentice Hall, 2002.
- J.-J. E. Slotine and W. Li, *Applied Nonlinear Control*, Prentice Hall, 1991.
- P. A. Ioannou and J. Sun, *Robust Adaptive Control*, Prentice Hall, 1996.
- B. L. Stevens, F. L. Lewis, and E. N. Johnson, *Aircraft Control and Simulation*, 3rd ed., Wiley, 2015.
- R. C. Nelson, *Flight Stability and Automatic Control*, 2nd ed., McGraw-Hill, 1998.
- Skoltech Advanced Control Methods lecture notes: `Control_systems_and_stability_260413_175041.pdf`.
- Skoltech Advanced Control Methods lecture notes: `Adaptive_control_260423_170915 (1).pdf`.
