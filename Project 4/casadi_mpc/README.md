# CasADi MPC Controller

This folder contains an alternative nonlinear MPC controller solved with
CasADi/IPOPT instead of the sampling-based controller in the project root.

State and action use the same notation as the main environment:

```math
s = [x, y, \psi, v, \delta, T],
\qquad
a = [T_{\text{cmd}}, \delta_{\text{cmd}}].
```

The optimization uses the Ackermann plant dynamics from `race_env.py`, actuator
lag for steering and wheel torque, road-tube constraints around the S-curve,
and a combined tire-grip constraint:

```math
\left(\frac{T}{T_{\text{slip}}}\right)^2
+
\left(\frac{a_y}{a_{y,\max}}\right)^2
\leq
\gamma^2.
```

Near the corner exit, the objective relaxes strict reference-line tracking and
increases the reward for terminal speed and positive rear-wheel torque. This
encourages the controller to accelerate out of the final bend instead of
braking just to hit the reference line more tightly.

The lateral target inside the road corridor is now an optimization variable.
The solver is constrained to stay within the track width, but it can choose its
own lateral placement instead of following a fixed reference line. A weak
regularizer keeps this lateral target close to the hand-shaped racing line and
smooth over time.

Run from the project root:

```powershell
python -m pip install -r requirements.txt
python casadi_mpc/play_casadi.py --horizon 24
```

This controller is slower than the sampling controller because it solves a
nonlinear program at each MPC step, but the formulation is closer to the
standard continuous-state MPC model used in control courses.
