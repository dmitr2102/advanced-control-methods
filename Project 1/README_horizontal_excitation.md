# Horizontal Excitation Variant

This note is intentionally kept separate from the main project logic. The current simulator and controller code remain focused on **vertical excitation** of the Kapitza pendulum. Here we analyze a different physical setup: **horizontal oscillations of the suspension point**.

## Goal

We want to answer the question:

Can a pendulum be stabilized in a horizontal position when the suspension point oscillates horizontally?

The short answer is:

- **not exactly at the horizontal angle** `\theta = \pm \pi/2` for the standard single-frequency horizontal excitation model,
- but **yes, near-horizontal stable side equilibria can appear** in the averaged model when the horizontal excitation is strong enough.

So pure horizontal excitation does **not** create an exact stable equilibrium at `\pm \pi/2`, but it can create two stable equilibria that approach the horizontal positions as the excitation intensity grows.

## Coordinates and notation

For this horizontal-excitation variant it is more convenient to use the absolute angle

\[
\theta
\]

measured from the **downward vertical** direction.

The state is

\[
s =
\begin{bmatrix}
\theta \\
\dot{\theta}
\end{bmatrix},
\]

and the horizontal acceleration of the suspension point is chosen as the action

\[
a = \ddot{x}.
\]

## Lagrangian model for horizontal suspension motion

Let the suspension point move horizontally according to `x = x(t)`. The bob coordinates are

\[
X = x + l \sin\theta,
\qquad
Z = -l\cos\theta.
\]

The velocities are

\[
\dot{X} = \dot{x} + l\dot{\theta}\cos\theta,
\qquad
\dot{Z} = l\dot{\theta}\sin\theta.
\]

Hence

\[
v^2 = \dot{X}^2 + \dot{Z}^2
=
\dot{x}^2 + 2l\dot{x}\dot{\theta}\cos\theta + l^2\dot{\theta}^2.
\]

The kinetic and potential energies are

\[
T = \frac{m}{2}
\left(
\dot{x}^2 + 2l\dot{x}\dot{\theta}\cos\theta + l^2\dot{\theta}^2
\right),
\]

\[
V = -mgl\cos\theta.
\]

Therefore

\[
L = T - V.
\]

Applying the Euler-Lagrange equation gives

\[
l\ddot{\theta} + g\sin\theta + \ddot{x}\cos\theta = 0.
\]

With the action definition

\[
a = \ddot{x},
\]

the nonlinear plant becomes

\[
\ddot{\theta}
=
-\frac{g}{l}\sin\theta - \frac{a}{l}\cos\theta.
\]

Thus the state-space model is

\[
\dot{s}_1 = s_2,
\qquad
\dot{s}_2 = -\frac{g}{l}\sin s_1 - \frac{a}{l}\cos s_1.
\]

## Harmonic horizontal excitation

Now assume a fast harmonic horizontal acceleration

\[
a(t) = \alpha \cos(\omega t),
\]

where

- `\alpha` is the acceleration amplitude,
- `\omega` is the carrier frequency.

Then the equation becomes

\[
\ddot{\theta}
=
-\frac{g}{l}\sin\theta
-\frac{\alpha}{l}\cos(\omega t)\cos\theta.
\]

## Averaged model

For high-frequency excitation, the standard averaging argument gives the slow dynamics

\[
\ddot{\theta} + d\dot{\theta} + U'(\theta) = 0,
\]

with the effective potential

\[
U(\theta)
=
-\frac{g}{l}\cos\theta
-\frac{\alpha^2}{4l^2\omega^2}\sin^2\theta.
\]

Differentiating,

\[
U'(\theta)
=
\frac{g}{l}\sin\theta
-\frac{\alpha^2}{2l^2\omega^2}\sin\theta\cos\theta.
\]

Hence the averaged equation can be written as

\[
\ddot{\theta}
=
-d\dot{\theta}
-\frac{g}{l}\sin\theta
+\frac{\alpha^2}{2l^2\omega^2}\sin\theta\cos\theta.
\]

## Equilibria of the averaged system

The equilibrium condition is

\[
U'(\theta)=0,
\]

that is,

\[
\sin\theta
\left(
\frac{g}{l}
-\frac{\alpha^2}{2l^2\omega^2}\cos\theta
\right)=0.
\]

Therefore there are two types of equilibria:

1. Vertical equilibria:

\[
\theta = 0,
\qquad
\theta = \pi.
\]

2. Side equilibria:

\[
\cos\theta^\star
=
\frac{2gl\omega^2}{\alpha^2}.
\]

These side equilibria exist only when

\[
\alpha^2 > 2gl\omega^2.
\]

## Can the horizontal position be stabilized?

The exact horizontal positions correspond to

\[
\theta = \pm \frac{\pi}{2},
\qquad
\cos\theta = 0.
\]

But the side-equilibrium condition is

\[
\cos\theta^\star = \frac{2gl\omega^2}{\alpha^2}.
\]

For any finite `\alpha`, the right-hand side is positive, so

\[
\cos\theta^\star > 0.
\]

Therefore the equilibrium angle satisfies

\[
0 < \theta^\star < \frac{\pi}{2}.
\]

So the exact horizontal position is **not** an equilibrium of this standard averaged model.

However, if `\alpha` is large enough, then

\[
\frac{2gl\omega^2}{\alpha^2} \ll 1,
\]

and therefore

\[
\theta^\star \approx \frac{\pi}{2}.
\]

This means that **near-horizontal stabilization is possible**.

## Stability of the side equilibria

At a side equilibrium `\theta^\star`, the second derivative of the effective potential is

\[
U''(\theta^\star)
=
\frac{\alpha^2}{2l^2\omega^2}\sin^2\theta^\star > 0.
\]

Hence these side equilibria are **locally stable minima** of the averaged potential.

So the physically correct conclusion is:

- exact horizontal stabilization: **no**
- near-horizontal stabilization through stable side equilibria: **yes**

## Example with project-scale parameters

Take

\[
g = 9.81,
\qquad
l = 1,
\qquad
\omega = 18.
\]

Then

\[
2gl\omega^2 \approx 6356.88.
\]

If

\[
\alpha = 100,
\]

then

\[
\cos\theta^\star = \frac{6356.88}{10000} \approx 0.636,
\]

so

\[
\theta^\star \approx 50.5^\circ.
\]

This is far from horizontal.

If instead

\[
\alpha = 250,
\]

then

\[
\cos\theta^\star = \frac{6356.88}{62500} \approx 0.102,
\]

so

\[
\theta^\star \approx 84.2^\circ.
\]

This is already close to a horizontal orientation.

## Practical conclusion for the project

If we want to extend the project in a clean way, the horizontal-excitation case should be implemented as a **separate model family**, not as a modification of the current vertical-excitation code.

The correct next step would be:

1. create a separate simulator for
\[
\ddot{\theta}
=
-\frac{g}{l}\sin\theta - \frac{a}{l}\cos\theta,
\]
2. use horizontal harmonic excitation
\[
a(t)=\alpha\cos(\omega t),
\]
3. analyze or control the stable side equilibria
\[
\theta^\star = \arccos\left(\frac{2gl\omega^2}{\alpha^2}\right),
\]
4. treat the exact horizontal position only as a limiting case as `\alpha \to \infty`.

## Separate animation

This variant is illustrated by a separate GIF:

- `animations/horizontal_excitation_side_equilibrium.gif`

generated by

- `python generate_horizontal_excitation_gif.py`

The current animation uses the parameter set

\[
\alpha = 800,
\qquad
\omega = 50,
\qquad
d = 0.5,
\]

for which the averaged side equilibrium is

\[
\theta^\star \approx 85.60^\circ.
\]

In the full nonlinear simulation used for the GIF, the final error with respect to this averaged side equilibrium is approximately

\[
|\theta(12\,\mathrm{s}) - \theta^\star| \approx 0.25^\circ.
\]

## Separate controllers for the horizontal case

To keep the main project logic unchanged, the horizontal-excitation controllers are implemented separately in:

- `src/horizontal_controllers.py`
- `src/horizontal_simulation.py`

The current horizontal comparison uses four controller variants:

1. **Horizontal harmonic controller**

\[
a(t)=\alpha_0 \cos(\omega t),
\]

with

\[
\alpha_0 = 800,
\qquad
\omega = 50.
\]

2. **Horizontal Lyapunov-inspired controller**

This controller uses the averaged side-equilibrium relation

\[
\cos\theta^\star = \frac{2gl\omega^2}{\alpha^2}
\]

and shapes a desired equilibrium cosine as

\[
c_{\mathrm{des}} = c_{\mathrm{ref}} + k_e e + k_d \dot{\theta},
\qquad
e = \theta - \theta^\star.
\]

Then the amplitude is chosen from

\[
\alpha =
\sqrt{
\frac{2gl\omega^2}{c_{\mathrm{des}}}
}.
\]

The tuned parameters used in the comparison are

\[
k_e \approx 0.0235,
\qquad
k_d \approx 0.1962,
\]

\[
c_{\min} \approx 0.0300,
\qquad
c_{\max} \approx 0.4666.
\]

3. **Horizontal averaged-energy controller**

This controller uses an averaged-energy proxy around the side-equilibrium target:

\[
E_{\mathrm{avg}}
\approx
\bigl(1-\cos(\theta-\theta^\star)\bigr)
+
k_{\mathrm{rate}} \dot{\theta}^2.
\]

Then the desired equilibrium cosine is reduced as this energy grows:

\[
c_{\mathrm{des}}
=
\frac{c_{\mathrm{ref}}}{1 + k_E E_{\mathrm{avg}}}.
\]

The amplitude is chosen from

\[
\alpha =
\sqrt{
\frac{2gl\omega^2}{c_{\mathrm{des}}}
}.
\]

The tuned parameters used in the comparison are

\[
k_E = 4.0,
\qquad
k_{\mathrm{rate}} = 0.1,
\]

\[
c_{\min} = 0.03,
\qquad
c_{\max} = 0.12.
\]

4. **Horizontal PID cycle-energy controller**

This controller uses the cycle-averaged energy around the side-equilibrium target,

\[
E_{\mathrm{cycle}}
\approx
\frac{1}{2}\dot{\theta}^2
+
w_E \bigl(1-\cos(\theta-\theta^\star)\bigr),
\]

and updates the carrier amplitude by

\[
\alpha_{k+1}
=
\alpha_0 + \mathrm{PID}(E_{\mathrm{cycle},k}).
\]

The tuned parameters used in the comparison are

\[
k_p \approx 0.9824,
\qquad
k_i \approx 0.1965,
\qquad
k_d \approx 1.5417,
\qquad
w_E \approx 0.5209,
\]

\[
\alpha_{\min} \approx 820.0,
\qquad
\alpha_{\max} \approx 950.7.
\]

## Comparison figure

The common convergence plot for the four horizontal-excitation controllers is stored at

- `figures/horizontal_tuned_controller_angle_error_comparison_15s.png`

and the summary data are stored at

- `figures/horizontal_tuned_controller_angle_error_comparison_15s.json`

The figure compares the angle error

\[
|\theta(t)-\theta^\star|
\]

over `15 s`, with the convergence tube

\[
|\theta-\theta^\star| \le 0.08 \text{ rad}.
\]

For the current tuned configuration, the measured convergence times are:

- horizontal harmonic: `9.250 s`
- horizontal Lyapunov: `0.725 s`
- horizontal averaged-energy: `2.221 s`
- horizontal PID cycle-energy: `5.479 s`

So, for this separate horizontal-excitation problem, the Lyapunov-inspired controller is the fastest among the four tested variants, while the horizontal averaged-energy controller is the second fastest.

## Horizontal controller GIFs

Separate animations for the horizontal controllers are generated by

- `python generate_horizontal_controller_gifs.py`

and saved as

- `animations/horizontal_harmonic_stabilization.gif`
- `animations/horizontal_lyapunov_stabilization.gif`
- `animations/horizontal_averaged_energy_stabilization.gif`
- `animations/horizontal_pid_cycle_energy_stabilization.gif`
