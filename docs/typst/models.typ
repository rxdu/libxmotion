#import "template/template.typ": arkheion, arkheion-appendices

#show link: underline
#set cite(style: "ieee")

#show: arkheion.with(
  title: "Technical Note: Benchmark Platform Models",
  authors: (
    (name: "Ruixiang Du", email: "ruixiang.du@gmail.com", affiliation: ""),
  ),
  abstract: [This note documents the platform models implemented in `src/control/models` (namespace conventions, state/control layouts, governing equations, default parameters, and the closed-form oracle each implementation is validated against), with references to the originating literature. The set is chosen deliberately: classical models that are thoroughly understood in the literature yet rich enough to evaluate the estimation, planning, and control algorithms of this repository — each model stresses a property the others do not.],
)

= Conventions

Every model exposes the discrete `Step` concept used across the stack (`x_(t+1) = "Step"(x_t, u_t, t, Delta t)`, forward Euler unless noted) and, where the dynamics are continuous, the derivative `Deriv` used with the fixed-step classical RK4 propagator (`rk4.hpp`) for high-fidelity simulation. The numerical cores are scalar-templated raw-span functions in `model_core.hpp`, shared verbatim between the CPU implementations and the CUDA rollout programs (see the MPPI note). Units are SI (m, s, rad, N, kg) throughout; frames are stated per model.

= Wheeled platforms

== Differential drive (kinematic unicycle)

State $bold(x) = (x, y, theta)$ (world position, heading), control $bold(u) = (v, omega)$ (forward speed, yaw rate):

$ dot(x) = v cos theta, quad dot(y) = v sin theta, quad dot(theta) = omega $

The model class of production wheeled MPPI deployments (Nav2); actuator dynamics are absorbed by the box constraints. File: `diff_drive.hpp`.

== Kinematic bicycle (Ackermann)

State $bold(x) = (x, y, theta)$, control $bold(u) = (v, delta)$ (speed, front steering angle), wheelbase $L$:

$ dot(x) = v cos theta, quad dot(y) = v sin theta, quad dot(theta) = (v / L) tan delta $

The standard low-speed car model @Kong2015-bicycle. Default $L = 0.5 "m"$ (robot scale); the mismatch campaign overrides it to match the dynamic plant. File: `ackermann.hpp`.

== Bicycle with acceleration input

State $bold(x) = (x, y, v, theta)$, control $bold(u) = (a, delta)$:

$ dot(x) = v cos theta, quad dot(y) = v sin theta, quad dot(v) = a, quad dot(theta) = (v / L) tan delta $

Successor of the retired `model/BicycleKinematics` (wheelbase constant $L = 2.4 "m"$ preserved); the reachability Monte-Carlo propagates it with RK4. *Oracles:* straight-line constant-acceleration closed form; constant-steering circle of radius $L \/ tan delta$. File: `bicycle_accel.hpp`.

== Single-track model with linear tires ("dynamic bicycle")

The standard vehicle lateral-dynamics benchmark (Rajamani @Rajamani2011, ch. 2; the kinematic-vs-dynamic comparison plant of Kong et al. @Kong2015-bicycle). State $bold(x) = (X, Y, psi, v_x, v_y, r)$ — world position and heading, *body-frame* longitudinal/lateral velocities, yaw rate; control $bold(u) = (a_x, delta)$. With front/rear axle distances $l_f, l_r$ from the CoM, mass $m$, yaw inertia $I_z$, and per-axle cornering stiffnesses $C_f, C_r$, the linear-tire slip angles and lateral forces are

$ alpha_f = (v_y + l_f r) / v_x - delta, quad alpha_r = (v_y - l_r r) / v_x, quad F_(y f) = -C_f alpha_f, quad F_(y r) = -C_r alpha_r $

and the dynamics are

$ dot(X) = v_x cos psi - v_y sin psi, quad dot(Y) = v_x sin psi + v_y cos psi, quad dot(psi) = r $
$ dot(v)_x = a_x + v_y r, quad dot(v)_y = (F_(y f) cos delta + F_(y r)) / m - v_x r, quad dot(r) = (l_f F_(y f) cos delta - l_r F_(y r)) / I_z $

The linear tire model is meaningless near standstill: slip-angle denominators clamp at $v_(x,min)$ (default $0.5 "m/s"$). Defaults are a mid-size passenger car: $m = 1500 "kg"$, $I_z = 2500 "kg m"^2$, $l_f = 1.2 "m"$, $l_r = 1.6 "m"$, $C_f = C_r = 80000 "N/rad"$. *Oracle:* the steady-state yaw rate under constant speed and steering,

$ r_(s s) = v_x / (L + K_(u s) v_x^2) delta, quad K_(u s) = m / L ((l_r) / (C_f) - (l_f) / (C_r)), quad L = l_f + l_r $

verified to 1% by simulation with $v_x$ held exactly ($a_x = -v_y r$); at low speed the model reproduces the kinematic yaw rate $ (v \/ L) tan delta$ to 2%. Role in this repository: the designated *mismatch plant* — planners use the kinematic bicycle, this plant slips (per-seed tire-stiffness variation in the nightly campaign). File: `dynamic_bicycle.hpp`.

= Underactuated benchmark

== Cart-pole (inverted pendulum on a cart)

The classical underactuated benchmark, in the exact formulation of Barto, Sutton & Anderson @Barto1983-cartpole (uniform pole, frictionless track and pivot). State $bold(x) = (x, dot(x), theta, dot(theta))$ with $theta$ measured *from upright* ($theta = 0$ is the unstable equilibrium, $theta = pi$ hangs down); control $u = F$ (force on the cart). With cart mass $M$, pole mass $m$, pivot-to-CoM distance $l$ (half the pole length):

$ dot.double(theta) = (g sin theta - cos theta dot xi) / (l (4/3 - (m cos^2 theta) / (M + m))), quad xi = (F + m l dot(theta)^2 sin theta) / (M + m) $
$ dot.double(x) = xi - (m l dot.double(theta) cos theta) / (M + m) $

The $4\/3$ factor carries the uniform pole's moment of inertia ($I_"com" = m (2l)^2 \/ 12$). Defaults are the literature-standard set: $M = 1 "kg"$, $m = 0.1 "kg"$, $l = 0.5 "m"$, force limit $plus.minus 10 "N"$. *Oracles:* conservation of the mechanical energy

$ E = 1/2 M dot(x)^2 + 1/2 m (dot(x) + l dot(theta) cos theta)^2 + 1/2 m (l dot(theta) sin theta)^2 + 1/2 I_"com" dot(theta)^2 + m g l (cos theta - 1) $

under zero force ($|Delta E| < 10^(-6)$ relative over 10 s of RK4), instability of the upright equilibrium, and bounded oscillation about the hanging one. The integration scenario is the classical controller handover: MPPI swings the pole up globally, a DLQR state-feedback controller (gain synthesized from a numeric linearization at the upright) catches and holds. File: `cartpole.hpp`.

= Aerial platform

== Quadrotor (rigid body)

The canonical aerial benchmark @Mellinger2011-minsnap. State (13): $bold(p) in RR^3$, $bold(v) in RR^3$ (world frame), unit quaternion $bold(q)$ (w-x-y-z, body-to-world), $bold(omega) in RR^3$ in the *body* frame — note the SRB quadruped model (MPPI note) uses world-frame $bold(omega)$; the difference is deliberate and follows each model's literature convention. Control $bold(u) = (f, bold(tau))$: total thrust along the body $z$ axis and body-frame moments. With diagonal inertia $bold(I)$:

$ dot(bold(p)) = bold(v), quad dot(bold(v)) = -g bold(e)_3 + (f / m) bold(R)(bold(q)) bold(e)_3 $
$ dot(bold(q)) = 1/2 bold(q) times.o (0, bold(omega)), quad dot(bold(omega)) = bold(I)^(-1) (bold(tau) - bold(omega) times bold(I) bold(omega)) $

`Step` renormalizes the quaternion after the Euler update. Defaults are a lab-scale small quadrotor: $m = 0.5 "kg"$, $bold(I) = "diag"(2.32, 2.32, 4.0) times 10^(-3) "kg m"^2$. *Oracles:* hover ($f = m g$, level) is an exact equilibrium; free fall matches $z = -g t^2 \/ 2$; a pure yaw moment integrates $omega_z = (tau_z \/ I_(z z)) t$ exactly (the gyroscopic term vanishes for pure $z$ rotation) with the quaternion norm preserved. File: `quadrotor.hpp`.

= Legged platform

The single-rigid-body (SRB) quadruped — trunk as one rigid body driven by per-foot ground-reaction forces, massless legs, per-step foot plan and contact schedule — is documented with the MPPI technical note (it is that controller's primary legged plant) and follows the convex-MPC lineage @DiCarlo2018-convexmpc. File: `srb_quadruped.hpp`.

= Choosing a model

#table(
  columns: (auto, auto, auto),
  align: left,
  [*Model*], [*Stresses*], [*Typical use*],
  [double integrator], [analytic oracles, LQR baselines], [controller unit tests],
  [diff drive / kinematic bicycle], [planar planning, box constraints], [MPPI wheeled scenarios, tuner],
  [bicycle (accel input)], [second-order wheeled, RK4 propagation], [reachability Monte-Carlo],
  [dynamic bicycle], [*model mismatch*, slip, speed-dependent stability], [nightly robustness campaigns],
  [cart-pole], [instability, swing-up/catch handover], [global+local controller composition],
  [quadrotor], [3D attitude, force/moment control], [3D estimation + control in the loop],
  [SRB quadruped], [contact schedules, GRF control], [legged MPPI, GPU rollouts],
)

#bibliography("bibliography.bib")
