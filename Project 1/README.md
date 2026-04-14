# Kapitza Pendulum Stabilization Project

This project studies stabilization of the Kapitza pendulum using Lyapunov-based methods.

For this course project, we use the dynamical-systems notation adopted in the course materials rather than RL-style notation.

## Notation

- `s` denotes the state
- `a` denotes the action, i.e. the control input
- `o` denotes the observation
- `t` denotes time
- `b` denotes disturbances or uncertainty
- `theta` denotes system parameters when needed

In this project, the pendulum state is defined as

\[
s =
\begin{bmatrix}
s_1 \\
s_2
\end{bmatrix}
=
\begin{bmatrix}
\varphi \\
\dot{\varphi}
\end{bmatrix},
\]

where

- `\varphi = \theta - \pi` is the deviation from the upright equilibrium
- `\dot{\varphi}` is the angular velocity

The control input is chosen as the vertical acceleration of the suspension point:

\[
a = \ddot{y}.
\]

The simulation model also includes a viscous damping coefficient `d`.

The stabilization target is

\[
s^\star =
\begin{bmatrix}
0 \\
0
\end{bmatrix},
\qquad
s \to 0.
\]

## Plant Model

We consider a pendulum of length `l` and bob mass `m`, whose suspension point moves only along the vertical axis according to `y = y(t)`.

Let `\theta` be the absolute pendulum angle measured from the downward vertical direction. Then the bob coordinates are

\[
x = l \sin \theta,
\qquad
z = y - l \cos \theta.
\]

Their velocities are

\[
\dot{x} = l \dot{\theta} \cos \theta,
\qquad
\dot{z} = \dot{y} + l \dot{\theta} \sin \theta.
\]

Hence, the squared speed is

\[
v^2 = \dot{x}^2 + \dot{z}^2
= l^2 \dot{\theta}^2 + \dot{y}^2 + 2 l \dot{y} \dot{\theta} \sin \theta.
\]

## Lagrangian Derivation

The kinetic energy is

\[
T = \frac{m}{2}
\left(
l^2 \dot{\theta}^2 + \dot{y}^2 + 2 l \dot{y} \dot{\theta} \sin \theta
\right).
\]

The potential energy is

\[
V = m g (y - l \cos \theta).
\]

Therefore, the Lagrangian is

\[
L = T - V.
\]

Applying the Euler-Lagrange equation

\[
\frac{d}{dt}\frac{\partial L}{\partial \dot{\theta}} - \frac{\partial L}{\partial \theta} = 0,
\]

we first compute

\[
\frac{\partial L}{\partial \dot{\theta}}
= m l^2 \dot{\theta} + m l \dot{y} \sin \theta,
\]

and therefore

\[
\frac{d}{dt}\frac{\partial L}{\partial \dot{\theta}}
= m l^2 \ddot{\theta} + m l \ddot{y} \sin \theta + m l \dot{y} \dot{\theta} \cos \theta.
\]

Next,

\[
\frac{\partial L}{\partial \theta}
= m l \dot{y} \dot{\theta} \cos \theta - m g l \sin \theta.
\]

Substituting into the Euler-Lagrange equation gives

\[
m l^2 \ddot{\theta} + m l \ddot{y} \sin \theta + m g l \sin \theta = 0.
\]

Dividing by `m l`, we obtain

\[
l \ddot{\theta} + (\ddot{y} + g)\sin\theta = 0,
\]

or equivalently

\[
\ddot{\theta} = - \frac{g + \ddot{y}}{l} \sin \theta.
\]

## Upright-Equilibrium Coordinates

To study stabilization of the upright position, we introduce

\[
\varphi = \theta - \pi.
\]

Then

\[
\dot{\varphi} = \dot{\theta},
\qquad
\ddot{\varphi} = \ddot{\theta},
\qquad
\sin \theta = \sin(\varphi + \pi) = - \sin \varphi.
\]

Substituting this into the previous equation yields

\[
\ddot{\varphi} = \frac{g + \ddot{y}}{l}\sin\varphi.
\]

Since we define the action as

\[
a = \ddot{y},
\]

the final nonlinear plant model becomes

\[
\ddot{\varphi} = \frac{g + a}{l}\sin\varphi.
\]

## State-Space Form

With

\[
s_1 = \varphi,
\qquad
s_2 = \dot{\varphi},
\]

the system is written as

\[
\dot{s}_1 = s_2,
\qquad
\dot{s}_2 = \frac{g + a}{l}\sin s_1.
\]

In vector form,

\[
\dot{s} = p(s, a)
=
\begin{bmatrix}
s_2 \\
\dfrac{g + a}{l}\sin s_1
\end{bmatrix}.
\]

If the full state is measured, then

\[
o = h(s) = s,
\qquad
a \leftarrow \pi(o).
\]

This is the baseline nonlinear model that will be used in the rest of the project.

## Averaged Kapitza Model

To study stabilization of the upright equilibrium under fast vertical oscillations, we specialize the action to a harmonic input

\[
a(t) = \alpha \cos(\omega t),
\]

where

- `\alpha` is the acceleration amplitude
- `\omega` is the angular frequency of the suspension oscillation

Equivalently, if the suspension displacement is written as

\[
y(t) = Y \cos(\omega t),
\]

then

\[
a(t) = \ddot{y}(t) = -Y \omega^2 \cos(\omega t),
\]

and the magnitudes satisfy

\[
\alpha = Y \omega^2.
\]

Substituting the harmonic acceleration into the nonlinear plant model gives

\[
\ddot{\varphi}
=
\left(
\frac{g}{l} + \frac{\alpha}{l}\cos(\omega t)
\right)\sin\varphi
- d\dot{\varphi},
\]

where `d` denotes viscous damping.

For high-frequency excitation, the standard Kapitza averaging argument splits the angular motion into a slow component and a fast oscillatory correction. After averaging over one fast period, the slow dynamics can be written in the form

\[
\ddot{\varphi} + d\dot{\varphi} + U'(\varphi) = 0,
\]

where the effective potential is

\[
U(\varphi)
=
\frac{Y^2 \omega^2}{4 l^2}\sin^2\varphi
+
\frac{g}{l}(\cos\varphi - 1).
\]

Using `Y = \alpha / \omega^2`, the same potential can be written as

\[
U(\varphi)
=
\frac{\alpha^2}{4 l^2 \omega^2}\sin^2\varphi
+
\frac{g}{l}(\cos\varphi - 1).
\]

The corresponding averaged equation is

\[
\ddot{\varphi}
=
- d\dot{\varphi}
- \frac{\alpha^2}{2 l^2 \omega^2}\sin\varphi\cos\varphi
+
\frac{g}{l}\sin\varphi.
\]

For small deviations around the upright position, we use

\[
\sin\varphi \approx \varphi,
\qquad
\cos\varphi \approx 1,
\]

which yields the local approximation

\[
\ddot{\varphi}
\approx
- d\dot{\varphi}
- \left(
\frac{\alpha^2}{2 l^2 \omega^2}
- \frac{g}{l}
\right)\varphi.
\]

Therefore, the upright equilibrium becomes locally stable when

\[
\frac{\alpha^2}{2 l^2 \omega^2} > \frac{g}{l},
\]

or equivalently

\[
\alpha^2 > 2 g l \omega^2.
\]

## Lyapunov Function for the Averaged Model

For the averaged dynamics, a natural Lyapunov candidate is the total effective energy

\[
V(\varphi, \dot{\varphi})
=
\frac{1}{2}\dot{\varphi}^2
+
\frac{\alpha^2}{4 l^2 \omega^2}\sin^2\varphi
+
\frac{g}{l}(\cos\varphi - 1).
\]

This function satisfies

\[
V(0,0) = 0.
\]

Near the upright equilibrium, its quadratic approximation is

\[
V(\varphi, \dot{\varphi})
\approx
\frac{1}{2}\dot{\varphi}^2
+
\frac{1}{2}
\left(
\frac{\alpha^2}{2 l^2 \omega^2}
- \frac{g}{l}
\right)\varphi^2.
\]

Hence `V` is locally positive definite whenever

\[
\alpha^2 > 2 g l \omega^2.
\]

Differentiating `V` along the averaged dynamics gives

\[
\dot{V} = - d\dot{\varphi}^2 \le 0.
\]

Therefore, under the Kapitza stabilization condition and in the presence of damping, the upright equilibrium is locally asymptotically stable for the averaged system.

## Closed-Loop Target Dynamics and Control Lyapunov Function

For controller design, we do not need to stay with the open-loop averaged Kapitza energy only. Instead, we specify a desired local closed-loop averaged dynamics around the upright equilibrium:

\[
\ddot{\varphi} + d\dot{\varphi} + (k_0 + k_1 \varphi^2)\varphi = 0,
\]

where

- `k_0 > 0` defines the local linear restoring stiffness
- `k_1 > 0` adds stronger restoring action for larger angular deviations

For this target dynamics, consider the polynomial Lyapunov function

\[
V_{\mathrm{cl}}(\varphi, \dot{\varphi})
=
\frac{1}{2}\dot{\varphi}^2
+
\frac{1}{2}k_0 \varphi^2
+
\frac{1}{4}k_1 \varphi^4.
\]

Its derivative along the target closed-loop dynamics is

\[
\dot{V}_{\mathrm{cl}}
=
\dot{\varphi}
\left(
\ddot{\varphi}
+
k_0 \varphi
+
k_1 \varphi^3
\right).
\]

Substituting

\[
\ddot{\varphi} = - d\dot{\varphi} - (k_0 + k_1 \varphi^2)\varphi
\]

gives

\[
\dot{V}_{\mathrm{cl}} = - d\dot{\varphi}^2 \le 0.
\]

Therefore, `V_cl` is a valid local Lyapunov function for the chosen target closed-loop dynamics.

### Amplitude Law from the Averaged Model

Near the upright equilibrium, the averaged Kapitza dynamics is approximated by

\[
\ddot{\varphi} + d\dot{\varphi} + k_{\mathrm{eff}}(\alpha)\varphi \approx 0,
\]

with

\[
k_{\mathrm{eff}}(\alpha)
=
\frac{\alpha^2}{2 l^2 \omega^2}
- \frac{g}{l}.
\]

To match the target local stiffness

\[
k_{\mathrm{des}}(\varphi) = k_0 + k_1 \varphi^2,
\]

we choose the harmonic acceleration amplitude `\alpha(\varphi)` from

\[
\frac{\alpha(\varphi)^2}{2 l^2 \omega^2} - \frac{g}{l}
=
k_0 + k_1 \varphi^2.
\]

Hence,

\[
\alpha(\varphi)
=
\sqrt{
2 l^2 \omega^2
\left(
\frac{g}{l} + k_0 + k_1 \varphi^2
\right)
}.
\]

The implemented Lyapunov controller then uses

\[
a(t) = \alpha(\varphi)\cos(\omega t).
\]

This law is state-dependent through the angle `\varphi`, so it is designed from a closed-loop target dynamics rather than from an open-loop averaged energy alone.

## Local Decay Rate Near the Upright Equilibrium

For small deviations around the upright equilibrium, the averaged dynamics is approximated by

\[
\ddot{\varphi} + d\dot{\varphi} + k_{\mathrm{eff}}\varphi = 0,
\]

with the effective stiffness

\[
k_{\mathrm{eff}}
=
\frac{\alpha^2}{2 l^2 \omega^2}
- \frac{g}{l}.
\]

The quadratic Lyapunov approximation near the equilibrium is

\[
V(\varphi, \dot{\varphi})
\approx
\frac{1}{2}\dot{\varphi}^2
+
\frac{1}{2}k_{\mathrm{eff}}\varphi^2.
\]

The eigenvalues of the linearized system are

\[
r_{1,2}
=
\frac{-d \pm \sqrt{d^2 - 4k_{\mathrm{eff}}}}{2}.
\]

Hence, the local exponential decay rate is

\[
\lambda
=
\begin{cases}
\dfrac{d}{2}, & d^2 < 4k_{\mathrm{eff}}, \\
\dfrac{d - \sqrt{d^2 - 4k_{\mathrm{eff}}}}{2}, & d^2 \ge 4k_{\mathrm{eff}}.
\end{cases}
\]

For the current default simulation parameters

\[
\alpha = 100,
\qquad
\omega = 18,
\qquad
l = 1,
\qquad
g = 9.81,
\qquad
d = 0.25,
\]

we obtain

\[
k_{\mathrm{eff}}
=
\frac{100^2}{2 \cdot 1^2 \cdot 18^2} - 9.81
\approx 5.62.
\]

Since

\[
d^2 = 0.0625 < 4k_{\mathrm{eff}} \approx 22.48,
\]

the equilibrium is locally underdamped and the decay rate is

\[
\lambda = \frac{d}{2} = 0.125 \ \text{s}^{-1}.
\]

Therefore, the local oscillation envelope decays approximately as

\[
e^{-0.125 t},
\]

which corresponds to a characteristic decay time of approximately

\[
\tau = \frac{1}{\lambda} \approx 8 \ \text{s}.
\]

## Repository Structure

The project is organized as follows:

- `README.md` contains the mathematical model, project description, and run instructions
- `src/system.py` contains the pendulum plant model
- `src/controllers/` contains controller implementations
- `src/simulation.py` contains the numerical simulator
- `src/visualization.py` contains the `pygame` visualization and interaction logic
- `src/main.py` is the main entry point
- `configs/` stores default simulation notes and parameter descriptions
- `figures/` is reserved for exported plots
- `animations/` is reserved for exported animations

## Current Controllers

At the current stage, the simulation includes five controller modes:

1. Harmonic controller:

\[
a(t) = A \cos(\omega t + \phi_0),
\]

where the amplitude `A` and angular frequency `\omega` can be adjusted interactively in the simulation window.

For the current default plant parameters,

\[
g = 9.81 \text{ m/s}^2,
\qquad
l = 1.0 \text{ m},
\]

and for the default harmonic frequency

\[
\omega = 18 \text{ rad/s},
\]

the Kapitza-type stabilization threshold from the averaged model is

\[
A > \sqrt{2 g l}\,\omega \approx 79.8 \text{ m/s}^2.
\]

To keep the default simulation inside a physically meaningful stabilization regime, the harmonic controller therefore starts with

\[
A = 100 \text{ m/s}^2.
\]

2. Averaged-energy controller:

This mode uses the older Lyapunov-inspired high-frequency law derived from the averaged Kapitza energy. The action is written as

\[
a(t) = \alpha(s)\cos(\omega t),
\]

where the carrier frequency `\omega` is fixed and the amplitude `\alpha(s)` is selected from the averaged stability condition so that the effective stiffness around the upright equilibrium remains positive. In the implementation, the amplitude is increased when the angular deviation or angular velocity becomes larger.

3. Lyapunov controller:

This mode uses a Lyapunov-inspired high-frequency law derived from the averaged Kapitza model. The action is written as

\[
a(t) = \alpha(\varphi)\cos(\omega t),
\]

where the carrier frequency `\omega` is fixed and the amplitude `\alpha(\varphi)` is selected from the closed-loop target stiffness

\[
k_{\mathrm{des}}(\varphi) = k_0 + k_1 \varphi^2.
\]

In the current implementation, the default values are `k_0 = 4` and `k_1 = 10`.

The default simulation parameters use `d = 0.25`, which makes the Lyapunov-inspired controller visibly stabilize the upright equilibrium in the interactive simulator.

4. PID position controller:

This mode uses a PID law on the angle error magnitude and applies the result to the carrier amplitude:

\[
\alpha(t) = \alpha_0 + \mathrm{PID}(|\varphi(t)|),
\qquad
a(t) = \alpha(t)\cos(\omega t).
\]

It is a direct feedback-on-position baseline that can be tuned interactively in the `pygame` application.

5. PID cycle-energy controller:

This mode estimates an energy proxy over each carrier cycle and then updates the carrier amplitude with a PID law:

\[
\alpha_{k+1} = \alpha_0 + \mathrm{PID}(E_{\mathrm{cycle},k}),
\qquad
a(t) = \alpha(t)\cos(\omega t).
\]

It is a more aggregate baseline, since the feedback uses the total energy accumulated during one oscillation cycle instead of the instantaneous angle alone.

6. Direct Lyapunov controller:

This mode uses the exact nonlinear model directly, without averaging. Consider the Lyapunov function

\[
V_{\mathrm{dir}}(\varphi,\dot{\varphi})
=
\frac{1}{2}\dot{\varphi}^2
+
k_p (1-\cos\varphi).
\]

For the exact plant

\[
\ddot{\varphi}
=
\frac{g+a}{l}\sin\varphi - d\dot{\varphi},
\]

the control law

\[
a
=
-g - l k_p - l k_c \dot{\varphi}\sin\varphi
\]

gives

\[
\dot V_{\mathrm{dir}}
=
-\left(d + k_c \sin^2\varphi\right)\dot{\varphi}^2 \le 0.
\]

So this controller is Lyapunov-based on the exact vertical dynamics, rather than on the averaged Kapitza approximation.

## Reproducibility

Create a virtual environment inside the project folder and install the required dependencies:

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

Run the simulation with:

```powershell
python src/main.py
```

## Pygame Controls

- `1` selects the harmonic controller
- `2` selects the averaged-energy controller
- `3` selects the Lyapunov controller
- `4` selects the PID position controller
- `5` selects the PID cycle-energy controller
- `Up` and `Down` increase or decrease the harmonic amplitude
- `Left` and `Right` increase or decrease the harmonic frequency
- `Z`, `X`, and `C` select `kp`, `ki`, and `kd` for PID tuning
- `J` and `K` decrease or increase the selected PID gain
- `Space` pauses or resumes the simulation
- `R` resets the simulation
- `Esc` closes the application

## GIF Generators

The repository contains standalone scripts that generate GIF animations for the main controller variants:

- `python generate_frequency_ramp_gif.py`
- `python generate_harmonic_gif.py`
- `python generate_averaged_energy_gif.py`
- `python generate_lyapunov_gif.py`
- `python generate_direct_lyapunov_gif.py`
- `python generate_pid_position_gif.py`
- `python generate_pid_cycle_energy_gif.py`

## Tuned Controllers and Measured Convergence

The previous derivations and animations are kept unchanged for reference. In addition, we tuned the controller parameters numerically in order to reduce the convergence time from the initial condition

\[
\varphi(0)=0.2 \text{ rad},
\qquad
\dot{\varphi}(0)=0.
\]

All tuned results below were obtained for the same plant parameters

\[
g = 9.81,
\qquad
l = 1.0,
\qquad
d = 0.25.
\]

### Evaluation protocol

For the comparison, we used the following settling criterion:

- the simulation runs with `dt = 1/240 s`
- the controller is considered converged when, over a rolling `1.5 s` window,
\[
|\varphi(t)| < 0.10 \text{ rad},
\qquad
|\dot{\varphi}(t)| < 0.40 \text{ rad/s}
\]
- the state must remain inside this window until the end of the simulation horizon

For the decay-rate estimate, we used an empirical envelope fit of the form

\[
|\varphi(t)|_{\mathrm{env}} \approx C e^{-\lambda t},
\]

where `\lambda` was identified by a least-squares fit of `\log |\varphi|` on binned local maxima. This is an empirical decay rate of the oscillation envelope, not a global analytical Lyapunov rate for the original non-autonomous system.

### Tuned parameter sets

1. Harmonic controller

\[
a(t)=A\cos(\omega t),
\]

with tuned parameters

\[
A = 116.5 \text{ m/s}^2,
\qquad
\omega = 25.3 \text{ rad/s}.
\]

Measured performance:

- settling time: `7.401 s`
- empirical decay rate: `\lambda \approx 0.211373 s^{-1}`

2. Averaged-energy controller

\[
a(t)=\alpha(s)\cos(\omega t),
\]

with tuned parameters

\[
\omega = 24.0,
\qquad
k_{\mathrm{target}} = 10.4,
\qquad
k_{\varphi} = 6.0,
\qquad
k_{\dot{\varphi}} = 0.8,
\]

\[
\alpha_{\min} = 63.0,
\qquad
\alpha_{\max} = 111.0.
\]

Measured performance:

- settling time: `9.505 s`
- empirical decay rate: `\lambda \approx 0.116917 s^{-1}`

3. Closed-loop Lyapunov controller

\[
a(t)=\alpha(\varphi)\cos(\omega t),
\]

with tuned parameters

\[
\omega = 18.0,
\qquad
k_0 = 0.35,
\qquad
k_1 = 2.35,
\]

\[
\alpha_{\min} = 62.5,
\qquad
\alpha_{\max} = 104.0.
\]

Measured performance:

- settling time: `6.933 s`
- empirical decay rate: `\lambda \approx 0.329295 s^{-1}`

4. Direct Lyapunov controller

\[
a
=
-g - l k_p - l k_c \dot{\varphi}\sin\varphi,
\]

with tuned parameters

\[
k_p = 2.0,
\qquad
k_c = 80.0.
\]

Measured performance:

- settling time: `1.087 s`

5. PID position controller

\[
\alpha(t)=\alpha_0+\mathrm{PID}(|\varphi(t)|),
\qquad
a(t)=\alpha(t)\cos(\omega t),
\]

with tuned parameters

\[
\omega = 20.0,
\qquad
\alpha_0 = 86.5,
\qquad
k_p = 20.0,
\qquad
k_i = 3.8,
\qquad
k_d = 16.5,
\]

\[
\alpha_{\min} = 62.5,
\qquad
\alpha_{\max} = 113.0.
\]

Measured performance:

- settling time: `8.022 s`
- empirical decay rate: `\lambda \approx 0.145977 s^{-1}`

6. PID cycle-energy controller

\[
\alpha_{k+1}=\alpha_0+\mathrm{PID}(E_{\mathrm{cycle},k}),
\qquad
a(t)=\alpha(t)\cos(\omega t),
\]

with tuned parameters

\[
\omega = 14.0,
\qquad
\alpha_0 = 82.0,
\qquad
k_p = 21.8,
\qquad
k_i = 8.5,
\qquad
k_d = 6.6,
\qquad
w_E = 6.3,
\]

\[
\alpha_{\min} = 68.5,
\qquad
\alpha_{\max} = 133.5.
\]

Measured performance:

- settling time: `3.423 s`
- empirical decay rate: `\lambda \approx 0.265674 s^{-1}`

### Comparison summary

For this tuned setup, the fastest convergence was obtained with the PID cycle-energy controller, while the largest empirical local envelope decay rate was obtained with the closed-loop Lyapunov controller.

The observed ranking by settling time is:

1. Direct Lyapunov: `1.087 s`
2. PID cycle-energy: `3.423 s`
3. Closed-loop Lyapunov: `6.933 s`
4. Harmonic: `7.401 s`
5. PID position: `8.022 s`
6. Averaged-energy: `9.505 s`

The observed ranking by empirical decay rate is:

1. Closed-loop Lyapunov: `0.329295 s^{-1}`
2. PID cycle-energy: `0.265674 s^{-1}`
3. Harmonic: `0.211373 s^{-1}`
4. PID position: `0.145977 s^{-1}`
5. Averaged-energy: `0.116917 s^{-1}`

### Tuned GIF outputs

The new animations generated with the tuned parameters are saved separately so that the previous animations remain available:

- `animations/harmonic_stabilization_tuned.gif`
- `animations/averaged_energy_stabilization_tuned.gif`
- `animations/lyapunov_stabilization_tuned.gif`
- `animations/direct_lyapunov_stabilization_tuned.gif`
- `animations/pid_position_stabilization_tuned.gif`
- `animations/pid_cycle_energy_stabilization_tuned.gif`

### Why the PID controller can settle faster than the Lyapunov controller

The closed-loop Lyapunov controller in this project was designed from a desired averaged dynamics, not from a direct minimum-time objective. Its main purpose is to impose a structured and theoretically interpretable local behavior near the upright equilibrium.

In particular, the Lyapunov controller selects the carrier amplitude so that the averaged system behaves like

\[
\ddot{\varphi} + d\dot{\varphi} + (k_0 + k_1 \varphi^2)\varphi = 0,
\]

and therefore admits the polynomial Lyapunov function

\[
V_{\mathrm{cl}}(\varphi,\dot{\varphi})
=
\frac{1}{2}\dot{\varphi}^2
+
\frac{1}{2}k_0 \varphi^2
+
\frac{1}{4}k_1 \varphi^4.
\]

This gives a strong stability interpretation and a good local decay mechanism, but it also makes the controller more conservative because the amplitude is constrained to follow the target averaged stiffness rather than to minimize the transient time directly.

By contrast, the PID cycle-energy controller does not enforce a specific Lyapunov structure. Instead, it adjusts the carrier amplitude more aggressively based on the measured cycle energy. As a result, it can enter the admissible tube faster in practice, even though it is less structured analytically.

Therefore, in our simulations the two controllers excel in different senses:

- the closed-loop Lyapunov controller gives the largest empirical local decay rate
- the PID cycle-energy controller gives the shortest practical settling time

This is not a contradiction. A controller can have a stronger local Lyapunov interpretation and still be slower in terms of the global transient, while a heuristic PID law can be faster at reaching the target tube because it is less conservative.

## Robustness to Control Noise

We also tested the tuned closed-loop Lyapunov controller against the tuned PID cycle-energy controller in the presence of additive noise in the control channel.

The experiment used the noisy applied input

\[
a_{\mathrm{applied}}(t)
=
\mathrm{clip}\left(a_{\mathrm{cmd}}(t) + \eta(t), \pm 140\right),
\qquad
\eta(t)\sim\mathcal N(0,\sigma^2),
\]

where `\sigma` is the control-noise standard deviation in `m/s^2`.

### Noise experiment setup

- initial condition:
\[
\varphi(0)=0.2 \text{ rad},
\qquad
\dot{\varphi}(0)=0
\]
- plant parameters:
\[
g = 9.81,
\qquad
l = 1.0,
\qquad
d = 0.25
\]
- simulation horizon: `15 s`
- time step: `dt = 1/240 s`
- Monte Carlo trials: `40` per noise level
- convergence criterion:
the trajectory must enter and then remain in the tube
\[
|\varphi| \le 0.10 \text{ rad},
\qquad
|\dot{\varphi}| \le 0.40 \text{ rad/s}
\]
until the end of the horizon

The figure generated for this experiment is stored at

- `figures/noise_robustness_lyapunov_vs_pid_cycle_energy.png`

and the raw numerical results are stored at

- `figures/noise_robustness_lyapunov_vs_pid_cycle_energy.json`

### Main result

Under this white-noise control perturbation model, the Lyapunov controller did **not** turn out to be more robust than the PID cycle-energy controller. In fact, the PID cycle-energy controller showed higher success rates over a broad range of noise levels.

Selected results:

- `\sigma = 0`:
  Lyapunov success rate `100%`, median settling time `6.908 s`;
  PID success rate `100%`, median settling time `0.821 s`
- `\sigma = 2`:
  Lyapunov success rate `77.5%`, median settling time `12.067 s`;
  PID success rate `100%`, median settling time `0.844 s`
- `\sigma = 4`:
  Lyapunov success rate `50.0%`, median settling time `11.919 s`;
  PID success rate `92.5%`, median settling time `2.150 s`
- `\sigma = 8`:
  Lyapunov success rate `12.5%`;
  PID success rate `50.0%`
- `\sigma = 20`:
  Lyapunov success rate `2.5%`;
  PID success rate `20.0%`

Therefore, in this specific experiment, the PID cycle-energy controller is empirically more robust to additive control noise than the tuned Lyapunov controller.

### Interpretation

This result is plausible. The Lyapunov controller was designed from an averaged target dynamics and uses a state-dependent carrier amplitude at every step. Additive control noise directly perturbs this fast oscillatory actuation.

By contrast, the PID cycle-energy controller updates its amplitude using energy accumulated over an oscillation cycle. This makes it more of a cycle-averaged regulator, so it can partially smooth out fast zero-mean noise in the control channel.

Hence the correct conclusion for this project is:

- the Lyapunov controller has the cleaner analytical stability interpretation
- the PID cycle-energy controller is more robust in our Monte Carlo experiment with additive white control noise

So the statement "the Lyapunov controller should be more robust to control noise" is not supported by the present simulations.
