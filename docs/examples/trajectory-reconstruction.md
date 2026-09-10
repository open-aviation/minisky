# Trajectory reconstruction

Recent trajectory generation methods like Krauth et al. (2023)[^krauth2023] and Motte et al. (2025)[^motte2025] can generate trajectories that match the statistical distribution of the observed traffic, but do not guarantee that the generated trajectories are physically flyable. In this example, we demonstrate how to evaluate the physical "realism" of a trajectory following the approach of Olive et al. (2021)[^olive2021].

<figure class="theme-illustration" markdown="1">
  <img class="theme-illustration--light" src="../assets/illustrations/trajectory-reconstruction-light.svg" alt="Trajectory reconstruction process">
  <img class="theme-illustration--dark" src="../assets/illustrations/trajectory-reconstruction-dark.svg" alt="Trajectory reconstruction process">
  <figcaption markdown="1">
Given some reference trajectory $y(t)$ (which are often noisy observations of the true aircraft state $x(t)$) and the corresponding controller/pilot commands $u$, we can replay them through minisky: $x(t) = x_0 + \int_0^t f(x(\tau), u(\tau)) d\tau$ to obtain the simulated trajectory $\hat{y}(t)$. The reconstruction error $e(y, \hat{y})$ can then be used as a measure of the physical "realism".
  </figcaption>
</figure>

## Introduction

In this example, we will use the recorded flight `full_flight_short` from the [`traffic` library](https://github.com/xoolive/traffic) as the reference trajectory $y^\star$. In practice, you should use the trajectory produced by your own generator method.

<!-- TODO: show what the trajectory looks like -->

The next step is to obtain the controller/pilot commands $u$ needed by minisky, which includes:

- lateral intent (e.g. "direct to waypoint XXXXX")
- vertical profile (e.g. "climb to FL350")
- speed profile (e.g. "set speed to M0.78", "set speed to 250KCAS")

Since the surveillance data often do not contain it, we must *infer* it.

## Inferring controller and pilot actions

### Lateral

<!-- TODO -->

We use the Douglas-Peucker algorithm to simplify a trajectory down to a few points, and treat them as [waypoints][minisky.CoordinateWaypoint] in the routes.

### Vertical profile

<!-- TODO -->

<!-- TODO: BDS4,0 (selected altitude and setpoint) -->

<!-- TODO: is the vertical rate (inferred from median absolute barometric vertical rate) correct? we shuold note that we are doing offline reconstruction. -->

### Speed profile

<!-- TODO switch between constant CAS to constant Mach -->

<!-- offline reconstruction - find IAS plateaus for at least 1m, find earliest point from which Mach is stable for 3min -> becomes CAS/Mach crossover. -->

## Replay

<!-- the plan and replay -->

[^krauth2023]: T. Krauth, B. Figuet, X. Olive, and J. Morio, "Collision Risk Assessment in Terminal Manoeuvring Areas based on Trajectory Generation Methods," *Proceedings of the 15th USA/Europe Air Traffic Management Research and Development Seminar*, Savannah, GA, June 2023.

[^motte2025]: A. Motte, X. Olive, and J. Morio, "Conditional Variational Autoencoders for aircraft type-specific trajectory generation," 2025.

[^olive2021]: X. Olive, J. Sun, M. C. R. Murça, and T. Krauth, "A Framework to Evaluate Aircraft Trajectory Generation Methods," *Proceedings of the 14th USA/Europe Air Traffic Management Research and Development Seminar*, 2021.
