// #import "@preview/arkheion:0.1.0": arkheion, arkheion-appendices
#import "template/template.typ": arkheion, arkheion-appendices

// additional format settings
#show link: underline
#set cite(style: "ieee")
#set math.mat(delim: "[")

// title page
#show: arkheion.with(
  title: "Libxmotion Implementation Notes",
  authors: (
    (name: "Ruixiang Du", email: "ruixiang.du@gmail.com", affiliation: ""),
  ),
  // abstrct: "your-content" or none
  abstract: [These notes collect the core estimation and control formulations implemented in xmNavigation in one document, so that the shared conventions — coordinate frames, quaternion algebra, discrete-time propagation, and the scalar-templated numerical cores compiled by both the CPU and CUDA paths — carry a single consistent formulation across topics. The MEKF section derives the multiplicative extended Kalman filter for attitude and state estimation; the MPPI section derives the information-theoretic model-predictive path-integral controller and records its CPU and GPU execution strategy; the Platform Models section catalogs the benchmark plants these algorithms are evaluated on, each paired with the closed-form oracle that validates it. Standalone errata and implementation-review notes (e.g. the MEKF errata) are maintained separately.],
  // keywords: ("First keyword", "Second keyword", "etc."),
  date: "July 22, 2026",
)

#outline(
  title: auto,
  depth: 2,
  indent: auto
)

#pagebreak()

//------------------------------------------------------------------------//

= MEKF <sec-mekf>

This section is about the MEKF implementation in xmNavigation. The multiplicative extended Kalman filter originates with Lefferts, Markley and Shuster @Lefferts1982-mekf; @Markley2003-aer discusses the attitude error representations it builds on, and @Trawny2005-ikf gives a widely used derivation of the quaternion indirect (error-state) Kalman filter. Most of the equations and derivation steps in this section are taken from @Maley2013-it, with the continuous-to-discrete noise derivation following Appendix E of @Sola2017-cy.

== Quaternion 

The definition of a quaternion is given by:

$ bold(q) = mat(
  q_1;
  bold(q)_(2:4);
) $

The relevant operations on quaternions that are used for MEKF are given as follows:

- Multiplication: $ bold(q) times.o bold(p) = mat(
  q_1  p_1 - bold(q)_(2:4)  bold(p)_(2:4);
  q_1  bold(p)_(2:4) + p_1  bold(q)_(2:4) + bold(q)_(2:4) times bold(p)_(2:4);
) $
- Inverse: $ bold(q)^(-1) = mat(
  q_1;
  -bold(q)_(2:4);
) $

A quaternion represents a rotation of one frame with respect to another. The inverse of a quaternion represents the rotation in the opposite direction. The multiplication of two quaternions represents the composition of two rotations.

We have the following properties of quaternions:

$ bold(q) times.o bold(q)^(-1) = mat(1;bold(0)) $
$ bold(dot(q)) &= frac(1,2) bold(q) times.o mat(0; bold(omega)) \ &= frac(1,2) bold(Omega)(bold(omega))bold(q) $

where $omega$ is the angular velocity vector of the body frame with respect to the inertial frame and $Omega$ is defined as

$ bold(Omega)(bold(omega)) = mat(0,  -bold(omega); bold(omega), -[bold(omega)_times]) $
$ bold(omega)_times = mat(0, -omega_z, omega_y; omega_z, 0, -omega_x; -omega_y, omega_x, 0) $

== Error State Model

The MEKF is used to estimate the attitude of the body frame with respect to the inertial frame. The main difference between MEKF and other Kalman filters for attitude estimation is that MEKF does not use the quaternion in the state vector directly. 

Normally, we would have the state dynamics as

$ dot(hat(bold(q))) = frac(1,2)hat(bold(q)) times.o mat(0; bold(omega)) $
$ dot(hat(bold(v)))^i = bold(C)^i_b (hat(bold(q))) hat(bold(f))^b + hat(bold(g))^i $ <translational-dynamics-1>
$ dot(hat(bold(r)))^i =  hat(bold(v))^i $ <translational-dynamics-2> 

where $bold(C)^i_b$ is the transformation matrix that transforms a vector from the body frame to the inertial frame, $hat(bold(q))$ is the estimated quaternion, $hat(bold(v))$ is the estimated velocity, and $hat(bold(r))$ is the estimated position.

With MEKF, we use the error quaternion, which is the difference between the estimated quaternion and the true quaternion, as well as error velocity and position. 

The error quaternion is given as

$ bold(q) = hat(bold(q)) times.o delta bold(q) $
$ arrow.r.double delta bold(q) = hat(bold(q))^(-1) times.o bold(q) $

where $hat(bold(q))$ is the estimated quaternion and $delta bold(q)$ is the error quaternion.

The error state vector is then defined as

$ delta bold(x) = mat(
  delta bold(q);
  delta bold(v);
  delta bold(r);
  bold(beta_omega);
  bold(beta_f);
  bold(beta_m)
) $

where $delta bold(q)$ is the error quaternion, $delta bold(v)$ is the error in velocity, $delta bold(r)$ is the error in position, $bold(beta_omega)$ is the bias in the angular velocity, $bold(beta_f)$ is the bias in the specific force, and $bold(beta_m)$ is the bias in the magnetometer.

Then we need to derive the dynamics of the error state $dot(bold(x))$. We can do it by examining each component of the state vector separately.

=== Error Quaternion Dynamics

The error quaternion dynamics is derived as follows:

$ delta bold(q) &= hat(bold(q))^(-1) times.o bold(q) \
arrow.r.double delta dot(bold(q)) &= hat(bold(q))^(-1) times.o dot(bold(q)) + dot(hat(bold(q)))^(-1) times.o bold(q) $

After the following steps described in @Maley2013-it, we will eventually get 

$ delta dot(bold(q))_(2:4) tilde.equiv -hat(bold(omega))_times delta bold(q)_(2:4) + frac(1,2)delta bold(omega)  $
with the fact that $delta q_1 = 1$ and the assumption that $delta bold(q)$ is small.

Here we replace the error states $delta bold(q)_{2:4}$ with a vector of small angles to further simplify the equations.

$ & bold(alpha) = 2 delta bold(q)_(2:4) \
arrow.r.double & dot(bold(alpha)) = -hat(bold(omega))_times bold(alpha) + delta bold(omega) $ <error-quaternion-dynamics> 

=== Error Velocity and Position Dynamics

For the velocity and position error, we have 

$ delta bold(v) = bold(v) - hat(bold(v)) $
$ delta bold(r) = bold(r) - hat(bold(r)) $

Based on @translational-dynamics-1 and @translational-dynamics-2, we can derive the dynamics of the velocity and position error as

$ delta dot(bold(v)) = -bold(C)^i_b (hat(bold(q)))hat(bold(f))^b_times bold(alpha) + bold(C)^i_b delta bold(f) $ <error-velocity-dynamics>
$ delta dot(bold(r)) = delta bold(v) $ <error-position-dynamics>

=== Sensor Bias Dynamics

The angular rate error model is given by 

$ bold(omega) = hat(bold(omega)) - bold(beta)_omega - bold(eta)_omega $
$ dot(bold(beta))_omega = bold(nu)_omega $

Together with the definition

$ bold(omega) = hat(bold(omega)) + delta bold(omega) $

We can get

$ delta bold(omega) = - bold(beta)_omega - bold(eta)_omega $ <angular-rate-error-model>
$ delta dot(omega) = - bold(nu)_omega $ <angular-rate-bias-model>

The linear acceleration error model is given by

$ bold(f) = hat(bold(f)) - bold(beta)_f - bold(eta)_f $
$ dot(bold(beta))_f = bold(nu)_f $ <linear-acceleration-bias-model>

Together with the definition

$ bold(f) = hat(bold(f)) + delta bold(f) $

We can get 

$ delta bold(f) = - bold(beta)_f - bold(eta)_f $ <linear-acceleration-error-model>

The magnetometer bias is treated as a slowly diverging random walk process driven by the noise process $bold(nu)_m$

$ dot(bold(beta))_m = bold(nu)_m $ <magnetometer-bias-model>

=== Full Error State Dynamics

According to the results above, we redefine the error state as 

$ delta bold(x) = mat(
  bold(alpha);
  delta bold(v);
  delta bold(r);
  bold(beta_omega);
  bold(beta_f);
  bold(beta_m)
) $

Combining equation @error-quaternion-dynamics @error-velocity-dynamics @error-position-dynamics @angular-rate-bias-model @linear-acceleration-bias-model @magnetometer-bias-model, together with @angular-rate-error-model and @linear-acceleration-error-model, we get the full dynamics of the error state:

$ delta dot(bold(x)) = mat(dot(bold(alpha));
  delta dot(bold(v));
  delta dot(bold(r));
  bold(dot(beta)_omega);
  bold(dot(beta)_f);
  bold(dot(beta)_m)) = mat(
  -hat(bold(omega))_times, bold(0)_(3times 3), bold(0)_(3times 3), -bold(I)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
  -bold(C)^i_b (hat(bold(q)))hat(bold(f))^b_times, bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3), -bold(C)^i_b (hat(bold(q))), bold(0)_(3times 3);
  bold(0)_(3times 3), bold(I)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
  bold(0)_(3times 3), bold(0)_(3times 3) , bold(0)_(3times 3) , bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
  bold(0)_(3times 3), bold(0)_(3times 3) , bold(0)_(3times 3) , bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
  bold(0)_(3times 3), bold(0)_(3times 3) , bold(0)_(3times 3) , bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
  ) 
  mat(
    bold(alpha);
    delta bold(v);
    delta bold(r);
    bold(beta_omega);
    bold(beta_f);
    bold(beta_m)
  ) + mat(
    -bold(eta_(omega));
    -bold(C)^i_b (hat(bold(q)))bold(eta)_f;
    bold(0)_(3times 3);
    bold(nu)_omega;
    bold(nu)_f;
    bold(nu)_m
  ) $ <full-error-state-dynamics>

The 18-error-state model is linear and time-varying with respect to the error states. Following the standard representation of a linear time-varying system, we can write the error state dynamics as

$ delta dot(bold(x)) = bold(F) delta bold(x) + bold(G)mono(w) $

where matrices $bold(F)$ is given by

$ bold(F)(hat(bold(q)),hat(bold(omega)),hat(bold(f))) = mat(
  -hat(bold(omega))_times, bold(0), bold(0), -bold(I)_(3times 3), bold(0), bold(0);
  -bold(C)^i_b (hat(bold(q)))hat(bold(f))^b_times, bold(0), bold(0), bold(0), -bold(C)^i_b (hat(bold(q))), bold(0);
  bold(0), bold(I)_(3times 3), bold(0), bold(0), bold(0), bold(0);
  bold(0), bold(0) , bold(0) , bold(0), bold(0), bold(0);
  bold(0), bold(0) , bold(0) , bold(0), bold(0), bold(0);
  bold(0), bold(0) , bold(0) , bold(0), bold(0), bold(0);
  ) $
  
the matrix $bold(G)$ transforms the white noice sequeunce $mono(w)$ into the disturbance vector and is given by

$ bold(G) = mat(
  -bold(I), bold(0), bold(0), bold(0), bold(0), bold(0);
  bold(0), -bold(C)^i_b (hat(bold(q))), bold(0), bold(0), bold(0), bold(0);
  bold(0), bold(0), bold(0), bold(0), bold(0), bold(0);
  bold(0), bold(0), bold(0), bold(I), bold(0), bold(0);
  bold(0), bold(0), bold(0), bold(0), bold(I), bold(0);
  bold(0), bold(0), bold(0), bold(0), bold(0), bold(I);
  ) $

The variance of the disturbance vector $mono(w)$ is given by

$ bold(Q)_c = mat(
  "diag"(bold(sigma)^2_omega), bold(0), bold(0), bold(0), bold(0), bold(0);
  bold(0), "diag"(bold(sigma)^2_f), bold(0), bold(0), bold(0), bold(0);
  bold(0), bold(0), bold(0), bold(0), bold(0), bold(0);
  bold(0), bold(0), bold(0), "diag"(bold(sigma)^2_(beta omega)), bold(0), bold(0);
  bold(0), bold(0), bold(0), bold(0), "diag"(bold(sigma)^2_(beta f)), bold(0);
  bold(0), bold(0), bold(0), bold(0), bold(0), "diag"(bold(sigma)^2_(beta m));
  ) $

where $bold(sigma)_omega$, $bold(sigma)_f$, $bold(sigma)_(beta omega)$, $bold(sigma)_(beta f)$, $bold(sigma)_(beta m)$ are the standard deviations of the white noise processes. You can find more details about the derivation from Appendix E.1 of @Sola2017-cy.

== Measurement Model

The measurement model for the MEKF can be represented in the standard form

$ delta bold(z) = bold(H)delta bold(x) + mono(V) $

where $delta bold(z)$ is the error measurement vector, $bold(H)$ is the measurement function, $delta bold(x)$ is the error state vector, and $mono(V)$ is the measurement noise.

In this case, the gyroscope measurement is treated as control input. We have the following two types of measurements:

- Accelerometer: $tilde(bold(a))^b$
- Magnetometer: $tilde(bold(m))^b$

In practice, $tilde(bold(a))^b$ and $tilde(bold(m))^b$ can be acquired directly from the accelerometer and magnetometer, respectively. 

Note that Equation 37 in @Maley2013-it will be used for the calculation of measurement residual for the accelerometer and magnetometer

$ bold(C)^b_i (bold(q)) tilde.equiv mat(bold(I) - bold(alpha)_times)bold(C)^b_i (hat(bold(q))) $ <real-state-to-estimated-state>
// $ bold(C)^i_b (bold(q)) tilde.equiv bold(C)^i_b (hat(bold(q)))mat(bold(I) + bold(alpha)_times) $

=== Accelerometer Measurement Model

The accelerometer measurement model is given by:

$ tilde(bold(a))^b = bold(C)^b_i (bold(q))mat(0;0;-g) + bold(beta)_f + bold(eta)_f $


Here the acceleration of the rigid body where the IMU is attached to is neglected, and the accelerometer measurement is given by the gravity vector in the inertial frame rotated to the body frame. 

Substituting @real-state-to-estimated-state into the above equation, we can get

$ tilde(bold(a))^b &= mat(bold(I) - bold(alpha)_times)bold(C)^b_i (hat(bold(q))) mat(0;0;-g) + bold(beta)_f + bold(eta)_f \
&= bold(C)^b_i (hat(bold(q)))mat(0;0;-g) - bold(alpha)_times bold(C)^b_i (hat(bold(q)))mat(0;0;-g) +  bold(beta)_f + bold(eta)_f \
&= hat(tilde(bold(a)))^b - bold(alpha) times bold(C)^b_i (hat(bold(q))) mat(0;0;-g) + bold(beta)_f + bold(eta)_f \
&= hat(tilde(bold(a)))^b + bold(C)^b_i (hat(bold(q))) mat(0;0;-g) times bold(alpha) + bold(beta)_f + bold(eta)_f $

$ arrow.r.double delta tilde(bold(a))^b &= bold(C)^b_i (hat(bold(q))) mat(0;0;-g) times bold(alpha) + bold(beta)_f + bold(eta)_f \
&= mat(
  bold(C)^b_i (hat(bold(q))) mat(0;0;-g) times,
  bold(0),
  bold(0),
  bold(0),
  bold(I),
  bold(0)
  ) mat(
  bold(alpha);
  delta bold(v);
  delta bold(r);
  bold(beta_omega);
  bold(beta_f);
  bold(beta_m)
) + bold(eta)_f $

=== Magnetometer Measurement Model

The magnetometer measurement model is given by:

$ tilde(bold(m))^b &= bold(C)^b_i (bold(q)) bold(m)^i + bold(beta)_m + bold(eta)_m $

Similarly, substituting @real-state-to-estimated-state, we can get

$ tilde(bold(m))^b &tilde.equiv mat(bold(I) - bold(alpha)_times)bold(C)^b_i (hat(bold(q))) bold(m)^i + bold(beta)_m + bold(eta)_m \
&= bold(C)^b_i (hat(bold(q))) bold(m)^i + bold(C)^b_i (hat(bold(q))) bold(m)^i times bold(alpha) + bold(beta)_m + bold(eta)_m \
&= hat(tilde(bold(m)))^b + bold(C)^b_i (hat(bold(q))) bold(m)^i times bold(alpha) + bold(beta)_m + bold(eta)_m $
$ arrow.r.double delta tilde(bold(m))^b &= bold(C)^b_i (hat(bold(q))) bold(m)^i times bold(alpha) + bold(beta)_m + bold(eta)_m \
&= mat(
  bold(C)^b_i (hat(bold(q))) bold(m)^i times, 
  bold(0), 
  bold(0), 
  bold(0), 
  bold(0), 
  bold(I)) 
mat(
  bold(alpha);
  delta bold(v);
  delta bold(r);
  bold(beta_omega);
  bold(beta_f);
  bold(beta_m)
) + bold(eta)_m $ 

=== Full Measurement Model

The full measurement model can be written as

$ mat(delta tilde(bold(a))^b;
  delta tilde(bold(m))^b) = mat(
    mat(
    bold(C)^b_i (hat(bold(q))) mat(0;0;-g) times,
    bold(0),
    bold(0),
    bold(0),
    bold(I),
    bold(0)
    );
    mat(
    bold(C)^b_i (hat(bold(q))) bold(m)^i times, 
    bold(0), 
    bold(0), 
    bold(0), 
    bold(0), 
    bold(I)) 
  ) delta bold(x) + mat(
    bold(eta)_f;
    bold(eta)_m
  ) $ <full-measurement-model>
where $bold(g)$ and $bold(m)^i$ are the gravity vector and the magnetic field vector in the inertial frame, respectively and both are known constants.

We can define the measurement matrix $bold(H)$ as

$ bold(H) = mat(
    mat(
    bold(C)^b_i (hat(bold(q))) mat(0;0;-g) times,
    bold(0),
    bold(0),
    bold(0),
    bold(I),
    bold(0)
    );
    mat(
    bold(C)^b_i (hat(bold(q))) bold(m)^i times, 
    bold(0), 
    bold(0), 
    bold(0), 
    bold(0), 
    bold(I)) 
  ) $

The measurement noise covariance matrix $bold(R)_c$ is given by

$ bold(R) = mat(
  bold(sigma)^2_a, bold(0);
  bold(0), bold(sigma)^2_m
) $

where $bold(sigma)_a$ and $bold(sigma)_m$ are the standard deviations of the accelerometer and magnetometer measurements, respectively.

== MEKF Formulation

=== Model Discretization

We have acquired the error state dynamics and the measurement model as given in @full-error-state-dynamics and @full-measurement-model. 

#math.equation(block: true, numbering: none, [
    $
    delta dot(bold(x)) &= bold(F) delta bold(x) + bold(G)mono(w) \
    &= mat(
    -hat(bold(omega))_times, bold(0)_(3times 3), bold(0)_(3times 3), -bold(I)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
    -bold(C)^i_b (hat(bold(q)))hat(bold(f))^b_times, bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3), -bold(C)^i_b (hat(bold(q))), bold(0)_(3times 3);
    bold(0)_(3times 3), bold(I)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
    bold(0)_(3times 3), bold(0)_(3times 3) , bold(0)_(3times 3) , bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
    bold(0)_(3times 3), bold(0)_(3times 3) , bold(0)_(3times 3) , bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
    bold(0)_(3times 3), bold(0)_(3times 3) , bold(0)_(3times 3) , bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
    ) 
    mat(
      bold(alpha);
      delta bold(v);
      delta bold(r);
      bold(beta_omega);
      bold(beta_f);
      bold(beta_m)
    ) + mat(
      -bold(eta_(omega));
      -bold(C)^i_b (hat(bold(q)))bold(eta)_f;
      bold(0)_(3times 3);
      bold(nu)_omega;
      bold(nu)_f;
      bold(nu)_m
    )
    $
  ]
)

The state transition matrix for the discrete system, $bold(Phi)$, is given by:

$ bold(Phi) = e^(bold(F)Delta t) $

The state transition matrix can be approximated as

$ bold(Phi)_(k-1) &= bold(I) + bold(F)_(k-1) Delta t \ 
  &= bold(I) + mat(
    -hat(bold(omega))_(k-1 _times), bold(0)_(3times 3), bold(0)_(3times 3), -bold(I)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
    -bold(C)^i_b (hat(bold(q))_(k-1))hat(bold(f))^b_(k-1 times), bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3), -bold(C)^i_b (hat(bold(q))_(k-1)), bold(0)_(3times 3);
    bold(0)_(3times 3), bold(I)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
    bold(0)_(3times 3), bold(0)_(3times 3) , bold(0)_(3times 3) , bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
    bold(0)_(3times 3), bold(0)_(3times 3) , bold(0)_(3times 3) , bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
    bold(0)_(3times 3), bold(0)_(3times 3) , bold(0)_(3times 3) , bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
    ) Delta t
$ <discrete-state-transition-matrix>

Then the state covariance matrix can be updated as

$ bold(P)^-_k = bold(Phi)_(k-1)bold(P)^+_(k-1)bold(Phi)^T_(k-1) + bold(Q)_d $

where $bold(Q)_d$ is the discrete-time process noise covariance matrix and is given by

#math.equation(block: false, numbering: none, [
$ bold(Q)_d &= integral^(Delta t)_0 e^(bold(F(t-tau)))bold(Q)_c e^(bold(F^T (t-tau)))d tau \
&= mat(
  // first row
  Lambda(bold(sigma^2_omega)) Delta t + Lambda(bold(sigma^2_(beta omega))) Delta t^3 / 3, bold(0), bold(0), -Lambda(bold(sigma^2_(beta omega)))frac(Delta t^2,2), bold(0), bold(0);
  // second row
  bold(0), Lambda(bold(sigma^2_f)) Delta t + Lambda(bold(sigma^2_(beta f))) frac(Delta t^3, 3), Lambda(bold(sigma^2_f)) Delta t^2 / 2 + Lambda(bold(sigma^2_(beta f))) frac(Delta t^4,8), bold(0), -Lambda(bold(sigma^2_(beta f))) Delta t^2 / 2, bold(0);
  // third row
  bold(0), Lambda(bold(sigma^2_f)) frac(Delta t^2,2) + Lambda(bold(sigma^2_(beta f))) frac(Delta t^4, 8), Lambda(bold(sigma^2_(f))) frac(Delta t^3,3) + Lambda(bold(sigma^2_(beta f))) frac(Delta t^5, 20), bold(0), -Lambda(bold(sigma^2_(beta f))) frac(Delta t^3, 6), bold(0);
  // forth row
  -Lambda(bold(sigma^2_(beta omega))) frac(Delta t^2,2), bold(0), bold(0), Lambda(bold(sigma^2_(beta omega))) Delta t, bold(0), bold(0);
  // fifth row
  bold(0), -Lambda(bold(sigma^2_(beta f))) Delta t^2 / 2, -Lambda(bold(sigma^2_(beta f))) frac(Delta t^3, 6), bold(0), Lambda(bold(sigma^2_(beta f))) Delta t, bold(0);
  // sixth row
  bold(0), bold(0), bold(0), bold(0), bold(0), Lambda(bold(sigma^2_(beta m))) Delta t
) $
]
)

Similarly, we have the transition matrix for the measurement model as

$ bold(H)_k = mat(
  bold(C)^b_i (hat(bold(q))^-_k) mat(0;0;-g) times,
  bold(0),
  bold(0),
  bold(0),
  bold(I),
  bold(0);
  bold(C)^b_i (hat(bold(q))^-_k) bold(m)^i times, 
  bold(0), 
  bold(0), 
  bold(0), 
  bold(0), 
  bold(I)
) $ <discrete-measurement-matrix>

=== MEKF Algorithm

The MEKF can be formulated as follows: 

- Initialize the filter with the initial state estimate $delta hat(bold(x))^+_0$ and the initial error-state covariance matrix $bold(P)^+_0$

$ delta hat(bold(x))^+_0 = bold(0) $
$ bold(P)^+_0 = bold(E)[(delta bold(x)_0 - delta hat(bold(x))^+_0)  (delta bold(x)_0 - delta hat(bold(x))^+_0)^T] = bold(E)(delta bold(x)_0 delta bold(x)_0^T)  $

- For step $k = 1,2,...$, perform the following steps:

  // - Update $hat(bold(q))^-_k$

  // $ hat(bold(q))^-_k = hat(bold(q))^+_(k-1) times.o mat(1; bold(hat(alpha)^+_(k-1))/2) $

  - Update $Phi_(k-1)$ using @discrete-state-transition-matrix

  #math.equation(block: true, numbering: none, [
    $ bold(Phi)_(k-1) &= bold(I) + bold(F)_(k-1) Delta t \ 
      &= bold(I) + mat(
        -hat(bold(omega))_(k-1 _times), bold(0)_(3times 3), bold(0)_(3times 3), -bold(I)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
        -bold(C)^i_b (hat(bold(q))_(k-1))hat(bold(f))^b_(k-1 times), bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3), -bold(C)^i_b (hat(bold(q))_(k-1)), bold(0)_(3times 3);
        bold(0)_(3times 3), bold(I)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
        bold(0)_(3times 3), bold(0)_(3times 3) , bold(0)_(3times 3) , bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
        bold(0)_(3times 3), bold(0)_(3times 3) , bold(0)_(3times 3) , bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
        bold(0)_(3times 3), bold(0)_(3times 3) , bold(0)_(3times 3) , bold(0)_(3times 3), bold(0)_(3times 3), bold(0)_(3times 3);
        ) Delta t
    $
  ])

  - Predict the error state and error-state covariance matrix

  $ bold(P)^-_k = bold(Phi)_(k-1)bold(P)^+_(k-1)bold(Phi)^T_(k-1) + bold(Q)_d $
  $ hat(bold(x))^-_k = bold(Phi)_(k-1) hat(bold(x))_(k-1)  $

  - Update the measurement matrix $bold(H)_k$ using @discrete-measurement-matrix
  - Compute the Kalman gain $bold(K)_k$
  
  $ bold(K)_k = bold(P)^-_k bold(H)_k^T (bold(H)_k bold(P)^-_k bold(H)_k^T + bold(R)_k)^(-1) $

  - Update the error state and error-state covariance matrix

  $ delta hat(bold(x))^+_k = delta hat(bold(x))^-_k + bold(K)_k (delta bold(z)_k - bold(H)_k delta hat(bold(x))^-_k) $

  Since the error state is reset to zero after every fold (see below), $delta hat(bold(x))^-_k = bold(0)$ and the update reduces to $delta hat(bold(x))^+_k = bold(K)_k delta bold(z)_k$, with $delta bold(z)_k$ the measurement residual (innovation).
  $ bold(P)^+_k = (bold(I) - bold(K)_k bold(H)_k) bold(P)^-_k $

  - Update the full states

  $ hat(bold(q))^+_k = hat(bold(q))^-_k times.o mat(1; bold(alpha^+_k)/2) $
  $ hat(bold(r))^+_k = hat(bold(r))^-_k + delta bold(r)^+_k $
  $ hat(bold(v))^+_k = hat(bold(v))^-_k + delta bold(v)^+_k $
  $ hat(bold(beta))^+_(omega,k) = hat(bold(beta))^-_(omega,k) + bold(beta)^+_(omega,k) $
  $ hat(bold(beta))^+_(f,k) = hat(bold(beta))^-_(f,k) + bold(beta)^+_(f,k) $
  $ hat(bold(beta))^+_(m,k) = hat(bold(beta))^-_(m,k) + bold(beta)^+_(m,k) $

  - Reset the error state for the next cycle: $delta hat(bold(x))^+_k arrow.l bold(0)$. The bias terms must be folded into persistent nominal states here — the error-state estimate does not accumulate across cycles.

== Initialization and Validation

*Initialization.* Starting the filter at identity with a large $P_0$ is both slow and, more importantly, outside the regime where a first-order filter is statistically honest (below). `attitude_init.hpp` provides the deterministic bootstrap: Shuster's TRIAD @Shuster1981-triad from a simultaneous accelerometer/magnetometer pair (full attitude, for MEKF9; the gravity pair anchors the primary axis), and accelerometer-only leveling (roll/pitch with zero yaw, the honest bootstrap for MEKF6, whose yaw is unobservable). `MagReferenceEnu` builds the inertial field reference from the local declination/inclination (e.g. from the World Magnetic Model for the deployment site); the ENU frame and sign conventions are documented in the header.

*Statistical consistency (NEES).* Both filters are validated by the standard normalized-estimation-error-squared criterion @BarShalom2001: Monte-Carlo truth generated exactly per the filters' noise model (continuous PSDs for gyro/accel noise, discrete observation covariances, random-walking biases), NEES computed on the $[bold(alpha); bold(beta)_omega]$ marginal against the filter's own covariance blocks. Findings, recorded in `test_mekf_consistency.cpp`:

- Open-loop error-state propagation is exact: empirical attitude variance matches the propagated $P$ to a ratio of 1.00.
- In the bootstrapped regime (attitude $sigma lt.eq 3 degree$, bias $sigma lt.eq 0.01$), both filters are consistent, erring mildly conservative in the marginals; the joint NEES carries $tilde 1.4 times$ optimism in the cross-correlations under sustained rotation — inherent first-order behavior, budgeted at $1.5 times$ dof in the regression.
- With large initial uncertainty ($sigma_alpha = 0.1 "rad"$, $sigma_(beta omega) = 0.1 "rad/s"$) the linearization breaks chi-square consistency ($tilde 2.4 times$ overconfident attitude): initialize with TRIAD/leveling rather than relying on a large $P_0$.
- Observability caveat: on a non-tumbling (planar) platform, yaw and the z gyro-bias are structurally unobservable to MEKF6 — their estimates wander and the covariance honestly grows. Use MEKF9 whenever heading matters; on tumbling trajectories all bias axes become observable and MEKF6 is consistent.

Remaining validation step (recorded, pending hardware data): replay of real IMU MCAP recordings as golden-log regressions.

//--------------------------------------------------------------------------------------//

= MPPI <sec-mppi>

This section derives the MPPI control update implemented in xmNavigation, following the information-theoretic formulation, and records the implementation decisions (warm start, baseline subtraction, sampling variants, control parameterization) with references to the originating literature. It closes with the CPU execution strategy and the CUDA extension for discrete GPU and Jetson Orin platforms.

== Lineage

Path-integral optimal control originates with Kappen @Kappen2005-pi, who showed that for control-affine stochastic systems with quadratic control cost the Hamilton–Jacobi–Bellman equation becomes linear under an exponential transformation of the value function, so the optimal control admits a Feynman–Kac path-integral representation evaluable by Monte Carlo. Theodorou et al. generalized the framework to parameterized policies as PI² @Theodorou2010-pi2. Williams et al. brought it to receding-horizon control as MPPI, first with the path-integral derivation @Williams2016-icra @Williams2017-jgcd, then re-derived it from an information-theoretic argument @Williams2017-itmpc @Williams2018-tro that removes the control-affine restriction — the dynamics only need to be *sampleable*. This note follows the information-theoretic derivation; the algorithm implemented in `src/control/mppi` is the algorithm of @Williams2018-tro.

== Problem setup

Consider discrete-time dynamics with state $bold(x)_t in RR^n$ and commanded control $bold(v)_t in RR^m$:

$ bold(x)_(t+1) = bold(F)(bold(x)_t, bold(v)_t) $

where the *executed* control is the commanded control corrupted by (or deliberately injected with) Gaussian noise: $bold(v)_t = bold(u)_t + bold(epsilon)_t$, $bold(epsilon)_t tilde cal(N)(bold(0), bold(Sigma))$. A trajectory $tau = (bold(x)_0, bold(v)_0, ..., bold(v)_(T-1))$ accrues the state cost

$ S(tau) = phi(bold(x)_T) + sum_(t=0)^(T-1) q(bold(x)_t) $ <state-cost>

with terminal cost $phi$ and running cost $q$. Note $S$ carries *no control cost*: the control effort penalty will emerge from the derivation as a KL term.

== Free energy and the optimal distribution

Let $PP$ denote the *base* distribution over trajectories induced by running the noise alone ($bold(u) = bold(0)$), and $QQ_(bold(u))$ the distribution induced by a control sequence $bold(u) = (bold(u)_0,...,bold(u)_(T-1))$. Define the *free energy* of the control problem @Williams2018-tro:

$ cal(F)(S) = -lambda log EE_PP [exp(-1/lambda S(tau))] $ <free-energy>

where $lambda > 0$ is the *inverse temperature*. Jensen's inequality gives, for any distribution $QQ$ absolutely continuous with $PP$:

$ cal(F)(S) <= EE_QQ [S(tau)] + lambda "KL"(QQ || PP) $ <jensen-bound>

i.e. the free energy lower-bounds the stochastic optimal control objective (state cost plus a control penalty in the form of a KL divergence from the base distribution). The bound is tight — @jensen-bound holds with equality — for the *optimal distribution* $QQ^*$ defined by the Gibbs/Boltzmann density

$ (d QQ^*)/(d PP)(tau) = (exp(-1/lambda S(tau))) / (EE_PP [exp(-1/lambda S(tau))]) $ <optimal-distribution>

$QQ^*$ is not directly realizable by a controller. MPPI therefore picks the realizable distribution closest to it: minimize $"KL"(QQ^* || QQ_(bold(u)))$ over the mean-shifted Gaussian family. For Gaussian noise this minimization has the closed-form solution (moment matching)

$ bold(u)_t^* = EE_(QQ^*)[bold(v)_t] $ <moment-matching>

== Importance sampling and the MPPI weights

@moment-matching is an expectation under $QQ^*$, which we can only evaluate by importance sampling from a distribution we *can* roll out — in practice $QQ_(hat(bold(u)))$, the distribution induced by the previous solution $hat(bold(u))$ (warm start). Writing the expectation under $QQ_(hat(bold(u)))$ and collecting the Radon–Nikodym factors of the Gaussian mean shift yields the *importance-sampling-corrected* trajectory cost @Williams2017-jgcd @Williams2018-tro:

$ tilde(S)(tau_k) = S(tau_k) + gamma sum_(t=0)^(T-1) hat(bold(u))_t^T bold(Sigma)^(-1) bold(epsilon)_(t,k) $ <corrected-cost>

where $bold(epsilon)_(t,k)$ is the $k$-th sampled noise sequence and $gamma = lambda (1 - alpha)$ exposes the *control-cost decoupling parameter* $alpha in [0,1]$ introduced in @Williams2018-tro ($alpha = 0$: full KL penalty toward the base distribution; $alpha = 1$: no pull toward zero control). With $K$ rollouts, the weights and update are

$ rho = min_k tilde(S)(tau_k), quad w_k = (exp(-1/lambda (tilde(S)(tau_k) - rho))) / (sum_(j=1)^K exp(-1/lambda (tilde(S)(tau_j) - rho))) $ <weights>

$ bold(u)_t arrow.l hat(bold(u))_t + sum_(k=1)^K w_k bold(epsilon)_(t,k) $ <update>

The baseline subtraction $rho$ leaves @weights mathematically unchanged and is mandatory numerically (it prevents uniform exponent underflow) @Williams2018-tro.

== The algorithm

+ Shift the previous solution one step (executed step removed, tail padded); this is the sampling mean $hat(bold(u))$.
+ Sample $K$ noise sequences $bold(epsilon)_k$; form perturbed controls $bold(v)_k = hat(bold(u)) + bold(epsilon)_k$ and clamp to actuator bounds.
+ Roll out the dynamics for each $bold(v)_k$; accumulate @state-cost and the correction of @corrected-cost.
+ Compute weights @weights with baseline subtraction; apply the update @update.
+ Optionally smooth the resulting sequence (Savitzky–Golay @Macenski2023-survey, or use a smoothness-preserving sampler, § below).
+ Execute $bold(u)_0$; repeat.

The loop is a single optimization iteration per control step; the warm start is what makes this sufficient in practice @Williams2017-jgcd.

== Practical considerations

- *Temperature* $lambda$ sharpens ($lambda arrow 0$, winner-take-all) or flattens (large $lambda$) the weights; it divides cost *differences*, so it is re-tuned whenever cost scales change @Williams2018-tro.
- *Noise covariance* $bold(Sigma)$ is the exploration knob and defines the KL anchor; it must be full-rank on sampled channels since $bold(Sigma)^(-1)$ appears in @corrected-cost.
- *Failure modes and their literature*: sample impoverishment after disturbances (Tube-MPPI @Williams2018-tube, Robust-MPPI @Gandhi2021-rmppi); control chattering from i.i.d. noise (Smooth-MPPI input lifting @Kim2022-smppi, low-frequency/colored sampling @Vlahov2024-colored); constraint handling — penalties inside rollouts are soft, so safety-critical constraints require an output-stage shield (control-barrier-function filtering @Yin2023-shield).
- *Sampling variants* implemented behind the sampler seam: normal–log-normal mixtures for cluttered spaces @Mohamed2022-logmppi, colored noise @Vlahov2024-colored, and annealed covariance schedules in the style of DIAL-MPC @Xue2024-dialmpc.
- *Control parameterization*: sampling spline knots instead of per-step controls reduces the decision dimension by an order of magnitude and enforces smoothness by construction; it is the enabling technique in whole-body legged sampling MPC @Howell2022-ps @AlvarezPadilla2024-wbmppi and is a first-class option here.

== Implementation notes (CPU now, CUDA next)

The CPU implementation follows the strategy validated by the Nav2 MPPI controller @Macenski2023-survey: batch-major contiguous storage (structure-of-arrays over the $K$ rollouts) that Eigen auto-vectorizes, preallocated workspace (no allocation in the control path), compile-time state/control dimensions via templates, and deterministic seeded sampling. Published throughput for this approach is roughly 30–60 million sample-timesteps per second per modern x86 core, which covers wheeled platforms (1–3k samples, 50–100 Hz) on a single core.

The rollout phase — clamp, step, accumulate cost — is extracted behind a backend seam (`rollout_backend.hpp`): sample $k$ reads shared inputs and writes only $S_k$, so backends may evaluate the $K$ rollouts in any parallel arrangement as long as the per-sample operation order is preserved (bitwise-reproducible results for any worker count; enforced by test). The CPU backend runs serially by default or over a pre-allocated worker pool with contiguous chunking. Measured on an 8-core x86 host at $K = 2048$: the SRB quadruped plan drops from 11.0 ms serial to 5.9 ms with 4 workers and 5.2 ms with 8; cheap wheeled models gain little (9.0 to 7.2 ms) because sequential noise generation dominates their budget (Amdahl). Per-sample counter-seeded noise streams would lift that cap and are the natural CPU follow-up; on the GPU path noise is generated on-device.

The CUDA backends implement the same seam following the architecture of MPPI-Generic @Vlahov2024-generic: one thread per sample (the split-rollout arrangement — right for models whose state fits in registers, 13 floats for the SRB), and dynamics/cost as scalar-templated raw-span cores (`model_core` / `critic_core`) that *both* backends compile — the Eigen model/critic classes wrap the cores on the CPU; a device-side "program" POD per platform family (wheeled, SRB quadruped; @sec-models) calls them in float32. Device rollouts run in float because FP64 executes at 1/32 rate on GTX-class and Jetson Orin hardware; the seam stays double-typed (narrow on upload, widen on download), and CPU/GPU costs agree to float accumulation error (bounded by test at $10^{-3}$ relative, observed $tilde 10^{-5}$; the SRB program includes the quaternion integration and the world-inertia solve as $R D^{-1} R^T$). The SRB program ships the per-step foot plan and contact schedule with each call, so the per-cycle `SetContext` pattern is preserved.

A third backend moves sampling on-device: Gaussian noise from per-sample Philox streams @Salmon2011-philox (counter-based, deterministic per seed and independent of thread scheduling) drawn inside the rollout kernel, with the weighted update $sum_k w_k epsilon_k$ also computed on-device — Plan() then transfers only the costs down and the weights up ($K$ floats each). Backends that sample on-device declare `kGeneratesNoise`; the controller skips its host sampler and fetches candidate noise for introspection on demand. End-to-end Plan() on a GTX 1660 Ti vs the 8-core host, wheeled model, $T = 50$: $K = 2048$: 0.14 ms device-sampling / 5.2 ms GPU-upload / 7.7 ms CPU-serial; $K = 131072$: 9.4 / 328 / 498 ms — host noise generation was the binding constraint, and with it removed six-figure sample counts fit a 100 Hz budget. SRB end-to-end ($T = 20$, host spline-knot sampling): $K = 2048$: 4.1 ms GPU vs 8.9 ms CPU-serial, now bounded by the host sampler — spline-knot generation on-device is the recorded follow-up, together with Jetson Orin deployment (unified memory: the pinned staging maps zero-copy).

//--------------------------------------------------------------------------------------//

= Platform Models <sec-models>

This section documents the platform models implemented in `src/control/models` (namespace conventions, state/control layouts, governing equations, default parameters, and the closed-form oracle each implementation is validated against), with references to the originating literature. The set is chosen deliberately: classical models that are thoroughly understood in the literature yet rich enough to evaluate the estimation, planning, and control algorithms of this repository — each model stresses a property the others do not.

== Conventions

Every model exposes the discrete `Step` concept used across the stack (`x_(t+1) = "Step"(x_t, u_t, t, Delta t)`, forward Euler unless noted) and, where the dynamics are continuous, the derivative `Deriv` used with the fixed-step classical RK4 propagator (`rk4.hpp`) for high-fidelity simulation. The numerical cores are scalar-templated raw-span functions in `model_core.hpp`, shared verbatim between the CPU implementations and the CUDA rollout programs (see @sec-mppi). Units are SI (m, s, rad, N, kg) throughout; frames are stated per model. `linearize.hpp` provides central-difference Jacobians of any `Deriv` model about an operating point — the standard bridge to the linear tools (`SolveDlqr`, `StateFeedbackController`): linearize, Euler-discretize ($A_d = I + A Delta t$, $B_d = B Delta t$, consistent with `Step`), synthesize.

== Wheeled platforms

=== Differential drive (kinematic unicycle)

State $bold(x) = (x, y, theta)$ (world position, heading), control $bold(u) = (v, omega)$ (forward speed, yaw rate):

$ dot(x) = v cos theta, quad dot(y) = v sin theta, quad dot(theta) = omega $

The model class of production wheeled MPPI deployments (Nav2); actuator dynamics are absorbed by the box constraints. File: `diff_drive.hpp`.

=== Kinematic bicycle (Ackermann)

State $bold(x) = (x, y, theta)$, control $bold(u) = (v, delta)$ (speed, front steering angle), wheelbase $L$:

$ dot(x) = v cos theta, quad dot(y) = v sin theta, quad dot(theta) = (v / L) tan delta $

The standard low-speed car model @Kong2015-bicycle. Default $L = 0.5 "m"$ (robot scale); the mismatch campaign overrides it to match the dynamic plant. File: `ackermann.hpp`.

=== Bicycle with acceleration input

State $bold(x) = (x, y, v, theta)$, control $bold(u) = (a, delta)$:

$ dot(x) = v cos theta, quad dot(y) = v sin theta, quad dot(v) = a, quad dot(theta) = (v / L) tan delta $

Successor of the retired `model/BicycleKinematics` (wheelbase constant $L = 2.4 "m"$ preserved); the reachability Monte-Carlo propagates it with RK4. *Oracles:* straight-line constant-acceleration closed form; constant-steering circle of radius $L \/ tan delta$. File: `bicycle_accel.hpp`.

=== Single-track model with linear tires ("dynamic bicycle")

The standard vehicle lateral-dynamics benchmark (Rajamani @Rajamani2011, ch. 2; the kinematic-vs-dynamic comparison plant of Kong et al. @Kong2015-bicycle). State $bold(x) = (X, Y, psi, v_x, v_y, r)$ — world position and heading, *body-frame* longitudinal/lateral velocities, yaw rate; control $bold(u) = (a_x, delta)$. With front/rear axle distances $l_f, l_r$ from the CoM, mass $m$, yaw inertia $I_z$, and per-axle cornering stiffnesses $C_f, C_r$, the linear-tire slip angles and lateral forces are

$ alpha_f = (v_y + l_f r) / v_x - delta, quad alpha_r = (v_y - l_r r) / v_x, quad F_(y f) = -C_f alpha_f, quad F_(y r) = -C_r alpha_r $

and the dynamics are

$ dot(X) = v_x cos psi - v_y sin psi, quad dot(Y) = v_x sin psi + v_y cos psi, quad dot(psi) = r $
$ dot(v)_x = a_x + v_y r, quad dot(v)_y = (F_(y f) cos delta + F_(y r)) / m - v_x r, quad dot(r) = (l_f F_(y f) cos delta - l_r F_(y r)) / I_z $

The linear tire model is meaningless near standstill: slip-angle denominators clamp at $v_(x,min)$ (default $0.5 "m/s"$). Defaults are a mid-size passenger car: $m = 1500 "kg"$, $I_z = 2500 "kg m"^2$, $l_f = 1.2 "m"$, $l_r = 1.6 "m"$, $C_f = C_r = 80000 "N/rad"$. *Oracle:* the steady-state yaw rate under constant speed and steering,

$ r_(s s) = v_x / (L + K_(u s) v_x^2) delta, quad K_(u s) = m / L ((l_r) / (C_f) - (l_f) / (C_r)), quad L = l_f + l_r $

verified to 1% by simulation with $v_x$ held exactly ($a_x = -v_y r$); at low speed the model reproduces the kinematic yaw rate $ (v \/ L) tan delta$ to 2%. Role in this repository: the designated *mismatch plant* — planners use the kinematic bicycle, this plant slips (per-seed tire-stiffness variation in the nightly campaign). File: `dynamic_bicycle.hpp`.

== Underactuated benchmark

=== Cart-pole (inverted pendulum on a cart)

The classical underactuated benchmark, in the exact formulation of Barto, Sutton & Anderson @Barto1983-cartpole (uniform pole, frictionless track and pivot). State $bold(x) = (x, dot(x), theta, dot(theta))$ with $theta$ measured *from upright* ($theta = 0$ is the unstable equilibrium, $theta = pi$ hangs down); control $u = F$ (force on the cart). With cart mass $M$, pole mass $m$, pivot-to-CoM distance $l$ (half the pole length):

$ dot.double(theta) = (g sin theta - cos theta dot xi) / (l (4/3 - (m cos^2 theta) / (M + m))), quad xi = (F + m l dot(theta)^2 sin theta) / (M + m) $
$ dot.double(x) = xi - (m l dot.double(theta) cos theta) / (M + m) $

The $4\/3$ factor carries the uniform pole's moment of inertia ($I_"com" = m (2l)^2 \/ 12$). Defaults are the literature-standard set: $M = 1 "kg"$, $m = 0.1 "kg"$, $l = 0.5 "m"$, force limit $plus.minus 10 "N"$. *Oracles:* conservation of the mechanical energy

$ E = 1/2 M dot(x)^2 + 1/2 m (dot(x) + l dot(theta) cos theta)^2 + 1/2 m (l dot(theta) sin theta)^2 + 1/2 I_"com" dot(theta)^2 + m g l (cos theta - 1) $

under zero force ($|Delta E| < 10^(-6)$ relative over 10 s of RK4), instability of the upright equilibrium, and bounded oscillation about the hanging one. The integration scenario is the classical controller handover: MPPI swings the pole up globally, a DLQR state-feedback controller (gain synthesized from a numeric linearization at the upright) catches and holds. File: `cartpole.hpp`.

== Aerial platform

=== Quadrotor (rigid body)

The canonical aerial benchmark @Mellinger2011-minsnap. State (13): $bold(p) in RR^3$, $bold(v) in RR^3$ (world frame), unit quaternion $bold(q)$ (w-x-y-z, body-to-world), $bold(omega) in RR^3$ in the *body* frame — note the SRB quadruped model (@sec-mppi) uses world-frame $bold(omega)$; the difference is deliberate and follows each model's literature convention. Control $bold(u) = (f, bold(tau))$: total thrust along the body $z$ axis and body-frame moments. With diagonal inertia $bold(I)$:

$ dot(bold(p)) = bold(v), quad dot(bold(v)) = -g bold(e)_3 + (f / m) bold(R)(bold(q)) bold(e)_3 $
$ dot(bold(q)) = 1/2 bold(q) times.o (0, bold(omega)), quad dot(bold(omega)) = bold(I)^(-1) (bold(tau) - bold(omega) times bold(I) bold(omega)) $

`Step` renormalizes the quaternion after the Euler update. Defaults are a lab-scale small quadrotor: $m = 0.5 "kg"$, $bold(I) = "diag"(2.32, 2.32, 4.0) times 10^(-3) "kg m"^2$. *Oracles:* hover ($f = m g$, level) is an exact equilibrium; free fall matches $z = -g t^2 \/ 2$; a pure yaw moment integrates $omega_z = (tau_z \/ I_(z z)) t$ exactly (the gyroscopic term vanishes for pure $z$ rotation) with the quaternion norm preserved. File: `quadrotor.hpp`.

== Legged platform

The single-rigid-body (SRB) quadruped — trunk as one rigid body driven by per-foot ground-reaction forces, massless legs, per-step foot plan and contact schedule — is documented in @sec-mppi (it is that controller's primary legged plant) and follows the convex-MPC lineage @DiCarlo2018-convexmpc. File: `srb_quadruped.hpp`.

== Choosing a model

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
