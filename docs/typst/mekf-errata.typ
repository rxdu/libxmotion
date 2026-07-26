#import "template/template.typ": arkheion, arkheion-appendices

#show link: underline
#set cite(style: "ieee")

#show: arkheion.with(
  title: "Errata: MEKF Derivation and Implementation Review",
  authors: (
    (name: "Ruixiang Du", email: "ruixiang.du@gmail.com", affiliation: ""),
  ),
  abstract: [This note documents the errors found in the original MEKF technical note (docs/typst/main.typ, pre-correction) and in the Mekf6 implementation (src/estimation, pre-fix), explains why each is wrong and what it breaks, and maps every error to its correction and the test that now pins it. The central finding: the implementation was a *faithful implementation of a flawed derivation* — three of the five code-level defects trace directly to errors in the document.],
)

= Summary

Five errors were found in the derivation document and nine in the implementation. They fall into three classes: (1) *process-noise errors* in the document, copied into the code, that break the covariance's symmetry and consistency; (2) a *structural bookkeeping error* — the document omitted the bias fold-in and error-state reset, and the code consequently never accumulated bias estimates; (3) *production defects* in the code only (console I/O in the hot path, numerically fragile covariance update, no observation gating, no input validation).

= Errors in the derivation document

== D1 — Attitude-block process noise uses the wrong sigma

*As printed* (upper-left block of $bold(Q)_d$):

$ Lambda(bold(sigma)^2_omega) Delta t + Lambda(bold(sigma)^2_omega) (Delta t^3) / 3 $

*Correct:*

$ Lambda(bold(sigma)^2_omega) Delta t + Lambda(bold(sigma)^2_(beta omega)) (Delta t^3) / 3 $

*Why:* the $Delta t^3\/3$ term arises from integrating the gyro *bias random walk* through the attitude error over the step — its driver is $bold(sigma)_(beta omega)$, not the gyro white noise. Evaluating $bold(Q)_d = integral_0^(Delta t) Phi(tau) bold(Q)_c Phi(tau)^T d tau$ with first-order $Phi$ makes this explicit: the $(alpha, alpha)$ block picks up $bold(sigma)^2_omega tau^0$ and $bold(sigma)^2_(beta omega) tau^2$ integrands.

*Consequence:* with typical IMUs $sigma_omega gt.double sigma_(beta omega)$, so the printed formula grossly over-inflates attitude process noise at larger $Delta t$. Note: the *code did not copy this error* — it already used $sigma_(beta omega)$ here. Document-only.

== D2 — The printed $bold(Q)_d$ is asymmetric

*As printed*, the $(delta v, delta r)$ block:

$ Lambda(bold(sigma)^2_(beta f)) (Delta t^4) / 8 + Lambda(bold(sigma)^2_(beta f)) (Delta t^2) / 2 $

while its mirror $(delta r, delta v)$ block — *correctly* — reads:

$ Lambda(bold(sigma)^2_f) (Delta t^2) / 2 + Lambda(bold(sigma)^2_(beta f)) (Delta t^4) / 8 $

A covariance matrix must satisfy $bold(Q)_d = bold(Q)_d^T$; the two entries disagree in which sigma multiplies $Delta t^2\/2$. The $(delta r, delta v)$ version is the correct one (the term couples velocity and position error through the accelerometer *white noise*).

*Consequence:* any faithful implementation inherits an asymmetric $bold(Q)_d$; adding it every cycle destroys the symmetry and positive-definiteness of $bold(P)$, which is the textbook route to numerical filter divergence. *The code copied this error* (its `Q.block(3,6)` used $sigma_(beta f)$ where its own `Q.block(6,3)` used $sigma_f$).

== D3 — Gyro-bias random walk grows as the wrong power of $Delta t$

*As printed:* $bold(Q)_d^((beta omega, beta omega)) = Lambda(bold(sigma)^2_(beta omega)) Delta t^2 / 2$. *Correct:* $Lambda(bold(sigma)^2_(beta omega)) Delta t$.

*Why:* a random-walk state's variance grows linearly in time — and the *same matrix* printed the accelerometer-bias twin correctly as $Lambda(bold(sigma)^2_(beta f)) Delta t$. The two entries are structurally identical; one of them had a typo.

*Consequence:* the filter under-weights gyro-bias uncertainty growth (for $Delta t < 2$ s), slowing or stalling bias convergence. *The code copied this error.*

== D4 — The measurement update never uses the innovation

*As printed:*

$ delta hat(bold(x))^+_k = delta hat(bold(x))^-_k + bold(K)_k bold(H)_k delta bold(x)^-_k $

As written, the measurement $delta bold(z)_k$ *does not appear* — the update multiplies the gain by the predicted error state itself, which is identically zero after a reset. The update law is non-functional as printed.

*Correct* (the innovation form):

$ delta hat(bold(x))^+_k = delta hat(bold(x))^-_k + bold(K)_k (delta bold(z)_k - bold(H)_k delta hat(bold(x))^-_k) $

which, because the error state is reset after every fold, reduces to $delta hat(bold(x))^+_k = bold(K)_k delta bold(z)_k$. (The code implemented the correct form — this was a document-only transcription error, but a reader implementing from the document would produce a controller that never corrects.)

== D5 — The full-state fold omitted the biases and the reset

*As printed*, the "update the full states" step folded only:

$ hat(bold(q))^+_k, quad hat(bold(r))^+_k, quad hat(bold(v))^+_k $

*Missing:* the bias fold $hat(bold(beta))^+ = hat(bold(beta))^- + bold(beta)^+$ for all three bias groups, and the explicit error-state reset $delta hat(bold(x))^+_k arrow.l bold(0)$.

*Why it matters:* in an error-state (indirect) formulation, the filter estimates *errors*; every quantity the measurement update produces must be folded into a persistent nominal state and the error zeroed, or the information is lost on the next cycle. Omitting the bias fold makes the bias states decorative: $bold(P)$ converges as if biases were being learned while no bias estimate ever accumulates. *This is the direct root cause of the worst implementation bug (C1).*

= Errors in the implementation (pre-fix `mekf6.cpp`)

== C1 — Bias states never persisted (structural)

The old update cycle was:

```cpp
// line 79-80: subtract "bias" from measurements
ControlInput gyro  = gyro_tilde  - x_.segment<3>(9);
Observation  accel = accel_tilde - x_.segment<3>(12);
// line 83: reset the error state
x_ = State::Zero();
...
// line 146: measurement update
x_ = x_ + K * delta_y;
// lines 159-160: "fold"???
x_.block<3,1>(0,0) += x_.block<3,1>(9,0);
x_.block<3,1>(3,0) += x_.block<3,1>(12,0);
```

Three things are wrong at once:

+ The "bias" subtracted at the top is whatever the *previous single* measurement update deposited in slots 9–14 — not an accumulated estimate. Because `x_` is zeroed each cycle (line 83), the bias estimate restarts from zero every step. Bias can never converge, while `P_` (updated by the full Kalman recursion) *claims* it has — the filter is statistically inconsistent with itself.
+ Lines 159–160 add the bias error into the attitude/velocity error slots (rows 0–2, 3–5) — slots that nothing reads before the next reset. Dead writes masquerading as the fold step.
+ There were no nominal bias members at all — nowhere for an accumulated estimate to live. This is D5 faithfully implemented.

*Fix:* nominal members `b_omega_`, `b_f_`; measurements corrected against them; after each update `b_omega_ += x.segment<3>(9)`, `b_f_ += x.segment<3>(12)`, quaternion fold, error reset. *Pinned by:* `Mekf6Test.GyroBiasRecovery` — a constant injected gyro bias must be recovered to 20% on the gravity-observable axes; the old code cannot pass it structurally.

== C2 / C3 — Process-noise errors inherited from D2 / D3

`GetQMatrix()` reproduced the asymmetric $(delta v, delta r)$ entry and the $Delta t^2\/2$ gyro-bias walk. *Fix:* corrected terms; the matrix is now symmetric *by construction* (each lower block assigned as the transpose of its upper mirror). *Pinned by:* `Mekf6Test.CovarianceStaysSymmetricPositive` — 5000 cycles, $||bold(P) - bold(P)^T|| < 10^(-12)$ and smallest eigenvalue $> -10^(-12)$.

== C4 — Dead code revealing the reset confusion

`x_ = Phi * x_;` immediately after `x_` was zeroed — always the zero vector. Harmless numerically; symptomatic of the misplaced reset. Removed (the error-state mean is zero by construction after every fold; only $bold(P)$ propagates).

== C5 — Console I/O in the filter hot path

Four unconditional `std::cout` statements ran on *every* `Update()` — at a 100 Hz IMU rate, formatted console output inside the control path (blocking, allocating, non-deterministic). Removed; diagnostics now flow through the wait-free telemetry API and accessors.

== C6 — Numerically fragile covariance update

$bold(P) arrow.l (bold(I) - bold(K) bold(H)) bold(P)$ is the textbook-but-unstable form: it is exact only for the exactly-optimal $bold(K)$ and loses symmetry under floating point. *Fix:* Joseph form $(bold(I) - bold(K) bold(H)) bold(P) (bold(I) - bold(K) bold(H))^T + bold(K) bold(R) bold(K)^T$ plus explicit re-symmetrization, and an LDLT solve instead of forming the innovation-matrix inverse.

== C7 — No accelerometer gating

The gravity observation was applied unconditionally. Under dynamic acceleration the accelerometer does *not* measure gravity, and the update drags the attitude toward a false vertical. *Fix:* gate on $|thin||bold(a)|| - g thin|$ (configurable, default 0.5 m/s²); rejected cycles are prediction-only. *Pinned by:* `Mekf6Test.AccelGateRejectsDynamicAcceleration` — 5 m/s² lateral acceleration for 200 cycles moves the attitude estimate by less than 0.1°.

== C8 — Minor defects

Float literals (`2.0f`, `3.0f`) mixed into double expressions; `q.inverse()` where `conjugate()` suffices on a normalized quaternion; no input validation (`dt <= 0`, NaN propagate silently — now rejected with the state untouched, pinned by `Mekf6Test.RejectsInvalidInput`).

== C9 — Observation gate keyed on the bias-corrected magnitude (self-lock)

The gate added in C7 (and the magnetometer gate) test the *bias-corrected* magnitude, $|thin| ||bold(a) - bold(beta)_f|| - g thin| <= tau$, coupling each gate to the very bias it helps estimate. This creates a feedback lock. Under sustained motion the accelerometer is (correctly) gated out, yet the magnetometer update still runs and, through the covariance cross-terms, drives $bold(beta)_f$ away from zero while the attitude is uncorrected. Once $bold(beta)_f$ has drifted, $||bold(a) - bold(beta)_f||$ no longer resembles $g$ *even when the device is static with clean gravity* — so the accelerometer stays gated, and since it is the only observation that corrects both the attitude and $bold(beta)_f$, the estimate cannot recover: roll and pitch lock at a tilted offset. *Fix:* gate on the *raw* measured magnitude, $|thin| ||tilde(bold(a))|| - g thin| <= tau$ (and likewise $||tilde(bold(m))||$ for the magnetometer) — the gate is a quasi-static / interference detector on the measurement, not the estimate; the innovation still uses the bias-corrected value. The same defect and fix apply to Mekf6's gravity gate (gyro+accel only, no magnetometer): it gated on $||bold(a) - bold(beta)_f||$ and now gates on $||tilde(bold(a))||$. *Pinned by:* `Mekf9Test.AccelGateUsesRawMagnitudeNotBiasCorrected`, `Mekf9Test.MagGateUsesRawMagnitudeNotBiasCorrected`, and `Mekf6Test.GravityGateUsesRawMagnitudeNotBiasCorrected` — a large seeded bias with a clean static reading must still be accepted. *Related:* the failure is most acute for a factory-calibrated IMU whose true bias is $approx 0$; such sensors should additionally pin the bias states with tight priors (cf. `CleanSensorParams`) so the bias never drifts to begin with.

= Error-to-fix map

#table(
  columns: (auto, auto, auto),
  align: left,
  table.header([*Error*], [*Corrected in*], [*Pinned by*]),
  [D1 sigma in $bold(Q)^(alpha alpha)$], [document], [—(doc-only; code was right)],
  [D2 / C2 asymmetric $bold(Q)_d$], [document + code], [CovarianceStaysSymmetricPositive],
  [D3 / C3 bias-walk power], [document + code], [GyroBiasRecovery (convergence rate)],
  [D4 update law], [document], [—(doc-only; code was right)],
  [D5 / C1 missing bias fold + reset], [document + code], [GyroBiasRecovery],
  [C4 dead propagation], [code], [—(removed)],
  [C5 console I/O], [code], [—(removed; telemetry instead)],
  [C6 covariance update form], [code], [CovarianceStaysSymmetricPositive],
  [C7 no gating], [code], [AccelGateRejectsDynamicAcceleration],
  [C8 validation/hygiene], [code], [RejectsInvalidInput],
  [C9 gate self-lock on diverged bias], [code (Mekf9 + Mekf6)], [{Accel,Mag,Gravity}GateUsesRawMagnitude],
)

Behavioral correctness of the whole is additionally pinned by `StaticAttitudeConvergence` (12° initial error converging below 0.5°, with the attitude/accel-bias observability manifold documented in the test) and `TracksSlowRollRotation` (sustained-motion tracking, gravity-direction error below 1°).

= A process observation

The implementation was not careless — it was *faithful to a flawed specification*. Three of the five substantive code defects (C1, C2, C3) are exact transcriptions of document errors (D5, D2, D3). The corrective lesson applied here: fix the derivation first, then implement against the corrected document, and let an independent oracle (the discrete LQR analogue for the linear case; the synthetic-truth suite here) arbitrate between them. The reference lineage now cited in the main note (Lefferts–Markley–Shuster 1982; Markley 2003; Trawny–Roumeliotis 2005; Maley 2013; Solà 2017) provides the third-party ground truth both artifacts are checked against.
