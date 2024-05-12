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
  abstract: none,
  // keywords: ("First keyword", "Second keyword", "etc."),
  date: "May 11, 2024",
)

#outline(
  title: auto,
  depth: 1,
  indent: auto
)

#pagebreak()

//------------------------------------------------------------------------//

= MEKF

This section is about the MEKF implementation in libxmotion. Most of the equations and derivation steps are taken from @Maley2013-it.

== Quaternion 

The definition of a quaternion is given by:

$ bold(q) = mat(
  q_1;
  bold(q)_(2:4);
) $

The relevant operations on quaternions that are used for MEKF are given as follows:

- Multiplication: $ bold(q) times.circle bold(p) = mat(
  q_1  p_1 - bold(q)_(2:4)  bold(p)_(2:4);
  q_1  bold(p)_(2:4) + p_1  bold(q)_(2:4) + bold(q)_(2:4) times bold(p)_(2:4);
) $
- Inverse: $ bold(q)^(-1) = mat(
  q_1;
  -bold(q)_(2:4);
) $

A quaternion represents a rotation of one frame with respect to another. The inverse of a quaternion represents the rotation in the opposite direction. The multiplication of two quaternions represents the composition of two rotations.

We have the following properties of quaternions:

$ bold(q) times.circle bold(q)^(-1) = mat(1;bold(0)) $
$ bold(dot(q)) &= frac(1,2) bold(q) times.circle mat(0; bold(omega)) \ &= frac(1,2) bold(Omega)(bold(omega))bold(q) $

where $omega$ is the angular velocity vector of the body frame with respect to the inertial frame and $Omega$ is defined as

$ bold(Omega)(bold(omega)) = mat(0,  -bold(omega); bold(omega), -[bold(omega)_times]) $
$ bold(omega)_times = mat(0, -omega_z, omega_y; omega_z, 0, -omega_x; -omega_y, omega_x, 0) $

== Error State Model

The MEKF is used to estimate the attitude of the body frame with respect to the inertial frame. The main difference between MEKF and other Kalman filters for attitude estimation is that MEKF does not use the quaternion in the state vector directly. 

Normally, we would have the state dynamics as

$ dot(hat(bold(q))) = frac(1,2)hat(bold(q)) times.circle mat(0; bold(omega)) $
$ dot(hat(bold(v)))^i = bold(C)^i_b (hat(bold(q))) hat(bold(f))^b + hat(bold(g))^i $ <translational-dynamics-1>
$ dot(hat(bold(r)))^i =  hat(bold(v))^i $ <translational-dynamics-2> 

where $bold(C)^i_b$ is the transformation matrix that transforms a vector from the body frame to the inertial frame, $hat(bold(q))$ is the estimated quaternion, $hat(bold(v))$ is the estimated velocity, and $hat(bold(r))$ is the estimated position.

With MEKF, we use the error quaternion, which is the difference between the estimated quaternion and the true quaternion, as well as error velocity and position. 

The error quaternion is given as

$ bold(q) = hat(bold(q)) times.circle delta bold(q) $
$ arrow.r.double delta bold(q) = hat(bold(q))^(-1) times.circle bold(q) $

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

$ delta bold(q) &= hat(bold(q))^(-1) times.circle bold(q) \
arrow.r.double delta dot(bold(q)) &= hat(bold(q))^(-1) times.circle dot(bold(q)) + dot(hat(bold(q)))^(-1) times.circle bold(q) $

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

$ delta dot(bold(x)) = bold(F) delta bold(x) + mono(W) $

where matrices $bold(F)$ is given by

$ bold(F)(hat(bold(q)),hat(bold(omega)),hat(bold(f))) = mat(
  -hat(bold(omega))_times, bold(0), bold(0), -bold(I)_(3times 3), bold(0), bold(0);
  -bold(C)^i_b (hat(bold(q)))hat(bold(f))^b_times, bold(0), bold(0), bold(0), -bold(C)^i_b (hat(bold(q))), bold(0);
  bold(0), bold(I)_(3times 3), bold(0), bold(0), bold(0), bold(0);
  bold(0), bold(0) , bold(0) , bold(0), bold(0), bold(0);
  bold(0), bold(0) , bold(0) , bold(0), bold(0), bold(0);
  bold(0), bold(0) , bold(0) , bold(0), bold(0), bold(0);
  ) $
  
and $mono(W)$ is the process noise. According to @full-error-state-dynamics, we have 

$ mono(W) = mat(
  -bold(eta_(omega));
  -bold(C)^i_b (hat(bold(q)))bold(eta)_f;
  bold(0)_(3times 3);
  bold(nu)_omega;
  bold(nu)_f;
  bold(nu)_m
) $

From the definitions of $bold(eta)_omega$, $bold(eta)_f$, $bold(nu)_omega$, $bold(nu)_f$, $bold(nu)_m$, we can get the covariance matrix of the process noise as

$ bold(Q)_c = mat(
  "diag"(bold(sigma)^2_omega), bold(0), bold(0), bold(0), bold(0), bold(0);
  bold(0), "diag"(bold(sigma)^2_f), bold(0), bold(0), bold(0), bold(0);
  bold(0), bold(0), bold(0), bold(0), bold(0), bold(0);
  bold(0), bold(0), bold(0), "diag"(bold(sigma)^2_(beta omega)), bold(0), bold(0);
  bold(0), bold(0), bold(0), bold(0), "diag"(bold(sigma)^2_(beta f)), bold(0);
  bold(0), bold(0), bold(0), bold(0), bold(0), "diag"(bold(sigma)^2_(beta m));
  ) $

where $bold(sigma)_omega$, $bold(sigma)_f$, $bold(sigma)_(beta omega)$, $bold(sigma)_(beta f)$, $bold(sigma)_(beta m)$ are the standard deviations of the white noise processes.

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

$ mat(tilde(bold(a))^b;
  tilde(bold(m))^b) = mat(
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
  ) $
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

$ bold(R)_c = mat(
  bold(sigma)^2_a, bold(0);
  bold(0), bold(sigma)^2_m
) $

where $bold(sigma)_a$ and $bold(sigma)_m$ are the standard deviations of the accelerometer and magnetometer measurements, respectively.

#pagebreak()

= Heading: first level
#lorem(20)

== Heading: second level
#lorem(20)

=== Heading: third level

==== Paragraph
#lorem(20)

#lorem(20)

= Math

*Inline:* Let $a$, $b$, and $c$ be the side
lengths of right-angled triangle. Then, we know that: $a^2 + b^2 = c^2$

*Block without numbering:*

#math.equation(block: true, numbering: none, [
    $
    sum_(k=1)^n k = (n(n+1)) / 2
    $
  ]
)

*Block with numbering:*

As shown in @equation.

$
sum_(k=1)^n k = (n(n+1)) / 2
$ <equation>

*More information:*
- #link("https://typst.app/docs/reference/math/equation/")


= Citation

You can use citations by using the `#cite` function with the key for the reference and adding a bibliography. Typst supports BibLateX and Hayagriva.

```typst
#bibliography("bibliography.bib")
```

Single citation @Vaswani2017AttentionIA. Multiple citations @Vaswani2017AttentionIA @hinton2015distilling. In text #cite(<Vaswani2017AttentionIA>, form: "prose")

*More information:*
- #link("https://typst.app/docs/reference/meta/bibliography/")
- #link("https://typst.app/docs/reference/meta/cite/")

= Figures and Tables


#figure(
  table(
    align: center,
    columns: (auto, auto),
    row-gutter: (2pt, auto),
    stroke: 0.5pt,
    inset: 5pt,
    [header 1], [header 2],
    [cell 1], [cell 2],
    [cell 3], [cell 4],
  ),
  caption: [#lorem(5)]
) <table>

#figure(
  image("figure/image.png", width: 30%),
  caption: [#lorem(7)]
) <figure>

*More information*

- #link("https://typst.app/docs/reference/meta/figure/")
- #link("https://typst.app/docs/reference/layout/table/")

= Referencing

@figure #lorem(10), @table.

*More information:*

- #link("https://typst.app/docs/reference/meta/ref/")

= Lists

*Unordered list*

- #lorem(10)
- #lorem(8)

*Numbered list*

+ #lorem(10)
+ #lorem(8)
+ #lorem(12)

*More information:*
- #link("https://typst.app/docs/reference/layout/enum/")
- #link("https://typst.app/docs/reference/meta/cite/")


// Add bibliography and create Bibiliography section
#bibliography("bibliography.bib")

// Create appendix section
#show: arkheion-appendices
= 

== Appendix section

#lorem(100)

