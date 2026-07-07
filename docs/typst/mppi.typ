#import "template/template.typ": arkheion, arkheion-appendices

#show link: underline
#set cite(style: "ieee")

#show: arkheion.with(
  title: "Technical Note: Model Predictive Path Integral Control — Derivation and Implementation",
  authors: (
    (name: "Ruixiang Du", email: "ruixiang.du@gmail.com", affiliation: ""),
  ),
  abstract: [This note derives the MPPI control update implemented in xmNavigation, following the information-theoretic formulation, and records the implementation decisions (warm start, baseline subtraction, sampling variants, control parameterization) with references to the originating literature. It closes with the CPU execution strategy and the planned CUDA extension for discrete GPU and Jetson Orin platforms.],
)

= MPPI

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

The CUDA extension for discrete GPUs and Jetson Orin implements the same backend seam and follows the architecture of MPPI-Generic @Vlahov2024-generic: samples across CUDA blocks, dynamics/cost as header-only functors over raw spans so *the same functor code* compiles for both backends (avoiding the dual-implementation burden MPPI-Generic documents), and split-vs-fused rollout kernels selected by benchmark. The GPU path becomes necessary beyond roughly 16k samples per step, for neural-network dynamics, or for full-order legged rollouts @Xue2024-dialmpc.

#bibliography("bibliography.bib")
