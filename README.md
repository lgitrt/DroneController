# DroneController

**A quadcopter flight controller built from scratch on an STM32G4, from raw
IMU registers to motor PWM.**

DroneController is a complete "+"-configuration quadrotor flight-control
firmware: it reads an MPU6050 IMU and an RC receiver, fuses the IMU
readings into roll/pitch attitude estimates, runs a PID control loop
against the stick/throttle commands, and mixes the result into four ESC
PWM outputs — all inside a hard real-time 1 kHz control loop on a
Cortex-M4.

![mcu](https://img.shields.io/badge/MCU-STM32G431KB-blue)
![license](https://img.shields.io/badge/license-GPLv3-informational)
![tests](https://github.com/lgitrt/DroneController/actions/workflows/tests.yml/badge.svg)

![CAD assembly of the quadrotor frame](images/drone-cad-assembly.png)

---

## Why this project

Rather than starting from an existing flight-control stack (Betaflight,
ArduPilot, ...), this project builds every layer of the controller from
first principles: the IMU driver, the attitude estimator, the PID loop,
and the motor-mixing law are all hand-written and tuned specifically for
this airframe. The goal was to understand — and be able to defend every
line of — the full path from raw accelerometer/gyro counts to commanded
motor speed, running under a hard 1 ms control-loop budget.

## Key engineering highlights

- **Two attitude estimators, same interface**: a fixed-gain Kalman filter
  (`callKF`) and a complementary filter (`callCompFilter`) both estimate
  roll/pitch from accelerometer + gyro data behind the same `struct est`
  interface, so the active estimator is a one-line swap in
  [`main.c`](Core/Src/main.c).
- **Interrupt-driven RC decoding**: all four RC channels (roll, pitch, yaw,
  throttle) are decoded concurrently from standard hobby-PWM pulses using
  input-capture timers in indirect mode (one channel's rising-edge period,
  the other's pulse width), with a digital low-pass filter smoothing each
  decoded channel before it reaches the controller.
- **Deterministic 1 kHz control loop**: a free-running timer (`TIM6`) is
  polled at the end of every loop iteration to pad execution out to an
  exact 1 ms period — sensor read, attitude estimate, PID update, and
  motor-mixing all have to fit comfortably inside that budget.
- **Startup self-calibration**: the firmware automatically runs a gyro-bias
  calibration (stationary averaging window) and an ESC calibration
  sequence (max-then-min throttle pulse) every boot, before entering the
  control loop.
- **Motor mixing with physical limits**: the "+"-configuration mixing law
  converts roll/pitch/yaw/thrust commands into four motor angular speeds
  via the drone's real arm length and thrust/drag coefficients
  (`PID.h`), square-roots the per-motor thrust demand into a commanded
  speed, and clamps it to the ESC's actual PWM range and the motor's rated
  maximum speed.

## Hardware

| Component | Part                        | Purpose                                   |
|-----------|------------------------------|--------------------------------------------|
| MCU       | STM32G431KBT6 (Cortex-M4)    | Main flight controller, 170 MHz             |
| IMU       | MPU6050                      | 3-axis accelerometer + gyroscope (I2C)     |
| RC input  | 4× PWM, input-capture timers | Roll / pitch / yaw / throttle from receiver |
| Actuators | 4× ESC, PWM output           | "+"-configuration quadrotor motor speed control |

The schematic and PCB below are from the custom sensor/IO carrier board
for this airframe (IMU, ultrasonic rangefinder, ESP32 wireless link, and
the PWM breakout to the flight-controller MCU); the firmware in this
repo currently drives the core IMU + RC + ESC path described above.

<p>
  <img src="images/electronics-schematic.png" alt="Flight controller carrier board schematic" width="49%">
  <img src="images/pcb-layout.png" alt="Flight controller carrier board PCB layout" width="49%">
</p>

## Firmware architecture

```
Core/
├── Inc/                  Public headers for every module
├── Src/
│   ├── main.c             Peripheral init, RC decode ISR, 1 kHz control loop, calibration
│   ├── IMU_KF.c            Roll/pitch Kalman filter + complementary filter (shared `struct est`)
│   ├── PID.c               Roll/pitch/yaw/altitude PID loops + "+"-config motor mixing
│   ├── mpu6050.c           MPU6050 IMU driver (I2C register read/write, raw-to-SI conversion)
│   └── stm32g4xx_*.c       STM32CubeMX-generated HAL glue (clocks, IRQ vectors, syscalls)
└── Startup/               STM32CubeMX-generated startup assembly
```

### Control pipeline

```
RC receiver (4ch PWM) ──► Input-capture timers ──► low-pass filter ──► roll/pitch/yaw/thrust refs
                                                                               │
MPU6050 (accel + gyro) ──► callCompFilter / callKF ──► roll, pitch estimate ──┼──► PID (callPID) ──► motor mixing ──► 4× ESC PWM
                                                                               │        (roll/pitch/yaw/altitude loops)
                                                              1 kHz loop, timed by TIM6
```

The two attitude estimators in [`IMU_KF.c`](Core/Src/IMU_KF.c) are kept
side by side intentionally: `callCompFilter` is what `main.c` currently
calls, while `callKF` implements a fixed-gain roll/pitch Kalman filter
behind the identical interface for direct comparison.

## MATLAB controller simulation

The standalone [simulation](simulation/README.md) lives entirely in
`simulation/`, separate from the embedded firmware. It compares
**LQR, constrained linear MPC, and cascaded PID with standard motor
mixing** on the same nonlinear quadrotor plant. Noisy-sensor experiments
use a **15-state navigation EKF**, not perfect state feedback. These
are simulation results, not hardware flight-test results.

From the repository root in MATLAB (Control System Toolbox and Model
Predictive Control Toolbox required):

```matlab
run(fullfile('simulation', 'matlab', 'run_controller_comparison.m'))
```

The default circle has a 2 m radius, a 30 s period, a 2.5 m altitude,
and a 5 s smooth ramp. All controllers run at **20 Hz** for 45 s with
shared rotor/rate limits. A **200 Hz noisy accelerometer/gyro**, **10 Hz
RTK-GNSS**, and **20 Hz magnetometer** supply the EKF. Residual biases
and random-walk drift are included. The RTK fixed-solution assumption is
explicit: this is not centimetre accuracy from ordinary GPS.

MPC predicts 2 s ahead; LQR uses current reference feedforward; PID uses
cascaded position/attitude loops with integral anti-windup and physical
plus-frame mixing. LQR/MPC share quadratic costs; PID is separately tuned.

Verified in MATLAB R2024b; RMS here means the **Euclidean 3D position
error**, not the smaller coordinate-averaged metric from the old plots:

| Experiment | LQR 3D RMS | MPC 3D RMS | PID 3D RMS |
|------------|------------|------------|------------|
| Perfect-state ideal | 1.78 cm | 0.27 cm | 0.39 cm |
| Noisy sensors + EKF | 3.31 cm | 2.45 cm | 3.07 cm |
| Sensors + wind + 40 ms motor lag | 5.63 cm | 5.17 cm | 11.39 cm |

### What the results actually show

- **Ideal accuracy is conditional.** Perfect feedback, shared plant/model
  parameters, slow motion, and feedforward/preview explain the small errors.
  These are not measured flight accuracies.
- **MPC is not uniformly better at wind rejection.** In the disturbed run,
  LQR/MPC horizontal axis RMSEs are almost identical (x: 3.95/3.95 cm;
  y: 2.98/3.03 cm). Their circle-only 3D RMS is 5.32/5.34 cm.
  MPC's whole-run benefit is principally startup/vertical tracking
  (z RMSE: 2.68 cm LQR versus 1.42 cm MPC), consistent with its preview.
  Circle-only means `t >= 5 s`, not a separately settled steady-state run.
- **PID depends on the chosen gains.** Its disturbed peak is 30.06 cm,
  versus 13.04/13.08 cm for LQR/MPC. The audit also shows larger errors
  during wind and slower post-wind recovery with this tuning. This is
  consistent with the current cascade bandwidth and integral recovery,
  not evidence that PID is inherently inferior.
- **No actuator limits activate in these default runs.** The differences
  are not caused by motor saturation or windup, and the plots do not
  demonstrate MPC's constraint advantage. Active constraints and PID
  anti-windup are exercised separately by regression tests.
- **Sensing sets a centimetre-scale floor under the assumptions.**
  Position-estimation RMS is about 1.79 cm for all three controllers.
  RTK fixed-solution accuracy and calibrated MEMS noise are assumed, not
  measured specifications of the physical drone.

The table and six figures use seed `20261002`. A separate **three-seed,
18-run sensitivity audit** recomputes RMS independently and checks finite
states, rotor bounds, command rates, final EKF covariance, and every MPC
solve. It found maximum roll/pitch below 4.71 degrees, consistent with a
near-hover controller model. Ranges across seeds `20261002`-`20261004`:

| Disturbed case | Whole-run 3D RMS | Wind window, 12-28 s | Recovery, 32-45 s |
|----------------|------------------|---------------------|------------------|
| LQR | 4.61-5.63 cm | 6.73-7.96 cm | 1.90-2.22 cm |
| MPC | 4.28-5.17 cm | 6.73-8.01 cm | 1.90-2.20 cm |
| PID | 9.80-11.39 cm | 13.85-15.49 cm | 5.50-6.00 cm |

This small seed check is not a confidence interval or a broad Monte Carlo
study. Exact results are in [metrics.csv](simulation/results/metrics.csv)
and [validation.csv](simulation/results/validation.csv). Wind is a
repeatable injected force, not calibrated aerodynamics. Sensor latency,
vibration, magnetic interference, RTK loss, drag, and ground contact are
not modeled. Host controller timings exclude EKF/plant integration and
do not establish an embedded real-time deadline.

### Six key comparison figures

The default run retains only these six high-resolution plots. The ideal
baseline remains in the numerical table; optional full diagnostics can
be generated locally using the [simulation guide](simulation/README.md).

**Noisy sensors: per-axis and 3D tracking errors**

![Sensed tracking metrics for LQR, MPC, and PID](simulation/results/sensors/rmse_summary.png)

**The simulated IMU: body-frame specific force and angular velocity**

![Noisy body-frame accelerometer and gyro with labelled truth](simulation/results/sensors/imu_measurements.png)

**Wind and motor lag: circle tracking with a zoomed error detail**

![Disturbed circle tracking with a zoomed detail](simulation/results/robustness/xy_tracking_comparison.png)

**Wind and motor lag: error summary and evolution**

![Disturbed per-axis and 3D tracking metrics](simulation/results/robustness/rmse_summary.png)

![Disturbed 3D and horizontal error magnitudes](simulation/results/robustness/tracking_error_comparison.png)

**Estimator errors, distinct from trajectory-tracking errors**

![Disturbed EKF estimation errors](simulation/results/robustness/estimation_errors.png)

### Simulation equations and state-space models

These equations describe the MATLAB simulation, not a replacement for
the embedded firmware's attitude filters. The
[full derivation and gain tables](simulation/MODEL_AND_CONTROL.md)
provide implementation details. Positions are metres, angles radians,
and rotor inputs **squared angular speeds**, not PWM or thrust.

#### Nonlinear dynamics and standard plus-frame mixing

$$
x=[r^T,\eta^T,v^T,\dot\eta^T]^T,\quad
\eta=(\phi,\theta,\psi)^T,\quad u_i=\omega_i^2,\quad
R_b^n=R_z(\psi)R_y(\theta)R_x(\phi).
$$

Body angular velocity is $\Omega=E\dot\eta$, where

$$
E=\begin{bmatrix}
1&0&-\sin\theta\\
0&\cos\phi&\sin\phi\cos\theta\\
0&-\sin\phi&\cos\phi\cos\theta
\end{bmatrix},\qquad
\begin{bmatrix}T\\\tau_x\\\tau_y\\\tau_z\end{bmatrix}
=\underbrace{\begin{bmatrix}
k_t&k_t&k_t&k_t\\lk_t&0&-lk_t&0\\
0&lk_t&0&-lk_t\\d&-d&d&-d
\end{bmatrix}}_{M}u.
$$

$$
\dot r=v,\qquad
\dot v=\frac{T}{m}R_b^ne_3-ge_3+\frac{F_w}{m},\qquad
\dot\Omega=I^{-1}[\tau-\Omega\times(I\Omega)],\qquad
\ddot\eta=E^{-1}(\dot\Omega-\dot E\dot\eta).
$$

Here $e_3=(0,0,1)^T$, $I=\mathrm{diag}(I_x,I_y,I_z)$.
RK4 integrates the plant every 5 ms. The disturbed scenario adds
$\dot u_{\rm applied}=(u_{\rm commanded}-u_{\rm applied})/0.04$.
All controllers share $0\le u_i\le u_{\max}$ and
$|u_{i,k}-u_{i,k-1}|\le0.2u_h$, with $u_h=mg/(4k_t)$.

#### Hover model shared by LQR and MPC

With $\delta u=u-u_h\mathbf1$, the continuous state-space model is
$(A,B,C,D)=(A,B,I_{12},0)$:

$$
A=\begin{bmatrix}0&0&I_3&0\\0&0&0&I_3\\0&G&0&0\\0&0&0&0\end{bmatrix},
\quad G=\begin{bmatrix}0&g&0\\-g&0&0\\0&0&0\end{bmatrix},\quad
B=\begin{bmatrix}0_{6\times4}\\e_3M_{1,:}/m\\I^{-1}M_{2:4,:}\end{bmatrix}.
$$

Each zero in $A$ is a $3\times3$ block. Exact zero-order hold gives

$$
A_d=e^{AT_s},\quad B_d=\int_0^{T_s}e^{At}B\,dt,\quad
x_{k+1}=A_dx_k+B_d\delta u_k,\qquad T_s=0.05\ {\rm s}.
$$

#### LQR

$$
Q=\mathrm{diag}(40,40,60,6,6,3,2,2,3,0.3,0.3,0.2),\quad R_u=10^{-10}I_4,
$$

$$
P=A_d^TPA_d+Q-A_d^TPB_d(R_u+B_d^TPB_d)^{-1}B_d^TPA_d,\quad
K=(R_u+B_d^TPB_d)^{-1}B_d^TPA_d,
$$

$$
u_k=u_h\mathbf1-K(\hat x_k-x_{{\rm ref},k}).
$$

LQR is static state feedback: its controller realization has $D_c=-K$
and no internal controller state. For a fixed, unsaturated reference,
the plant closed-loop matrix is $A_d-B_dK$. The moving reference includes
velocity and acceleration-derived attitude; the rotor feedforward is hover.

#### Constrained linear MPC

For normalized input $q=(u-u_h\mathbf1)/u_h$, MPC predicts from
$x_{0|k}=\hat x_k$ using $(A_d,u_hB_d,I_{12},0)$ and solves

$$
\min_q\sum_{j=1}^{40}
\left[(x_{j|k}-x_{{\rm ref},k+j})^TQ(x_{j|k}-x_{{\rm ref},k+j})
+q_{j-1|k}^T(u_h^2R_u)q_{j-1|k}\right],
$$

$$
x_{j+1|k}=A_dx_{j|k}+u_hB_dq_{j|k},\quad
-1\le q_i\le(u_{\max}-u_h)/u_h,\quad |\Delta q_i|\le0.2.
$$

The first 20 moves are optimized; the last is held for the remaining
prediction horizon. Only the first move is applied. There is no terminal
Riccati cost or move-rate penalty. Previous bounded command supplies the
first rate constraint. Built-in estimation is disabled in favor of the
shared EKF; failed solves stop the simulation. This constrained feedback
law has no single global LTI controller realization.

#### Cascaded PID and its controller state

Position/velocity errors are $e_r=r_{\rm ref}-\hat r$,
$e_v=v_{\rm ref}-\hat v$. Candidate integrals are clipped componentwise:

$$
\tilde z_r=\mathrm{clip}(z_r+T_se_r,L_r),\quad
a_c=\mathrm{clip}(a_{\rm ref}+K_p^re_r+K_d^re_v+K_i^r\tilde z_r,L_a).
$$

$$
\phi_c=(\sin\psi_r\,a_{c,x}-\cos\psi_r\,a_{c,y})/g,\quad
\theta_c=(\cos\psi_r\,a_{c,x}+\sin\psi_r\,a_{c,y})/g,\quad\psi_c=\psi_r.
$$

Roll/pitch targets are bounded to 25 degrees. With
$e_\eta=\mathrm{wrap}(\eta_c-\hat\eta)$ and previous target $s_k$:

$$
\tilde z_\eta=\mathrm{clip}(z_\eta+T_se_\eta,L_\eta),\quad
\dot\eta_c=(\eta_c-s_k)/T_s,\quad
\alpha_c=K_p^\eta e_\eta+K_d^\eta(\dot\eta_c-\widehat{\dot\eta})
+K_i^\eta\tilde z_\eta,
$$

$$
T_c=\frac{m(g+a_{c,z})}{\max(0.5,\cos\hat\phi\cos\hat\theta)},\quad
\tau_c=I\alpha_c,\quad u_{\rm requested}=M^{-1}[T_c;\tau_c].
$$

The nine-state controller is $\xi=[z_r;z_\eta;s]$:
$s_{k+1}=\eta_{c,k}$; integrals accept their candidates unless any motor
amplitude/rate bound activates, in which case both retain their old
values. This nonlinear cascade is not globally LTI. An individual
unsaturated PI-plus-velocity-damping block has
$A_c=I$, $B_c=[T_sI\ \ 0]$, $C_c=K_i$,
$D_c=[K_p+T_sK_i\ \ K_d]$. The inner torque model is a near-hover
approximation, not nonlinear computed-torque control.

#### Sensors and 15-state navigation EKF

The simulated measurement equations are

$$
f_m=(R_b^n)^T(\dot v+ge_3)+b_a+n_a,\quad
\Omega_m=E\dot\eta+b_g+n_g,\quad
z_{\rm GNSS}=[r;v]+n_{\rm GNSS},\quad z_{\rm mag}=(R_b^n)^Tm_n+n_m.
$$

Biases follow $b_{k+1}=b_k+\sigma_b\sqrt{\Delta t}\epsilon_k$.
The body accelerometer measures specific force; it reads gravity at rest.
The gyro measures body rates, not Euler rates.

For $\zeta=[r;v;\eta;b_a;b_g]$, averaged consecutive IMU samples give
$a=R_b^n(\bar f_m-\hat b_a)-ge_3$ and
$q_\eta=E^{-1}(\bar\Omega_m-\hat b_g)$. At $\Delta t=0.005$ s:

$$
\hat r^-=\hat r+\Delta t\hat v+\tfrac12\Delta t^2a,\quad
\hat v^-=\hat v+\Delta t a,\quad
\hat\eta^-=\hat\eta+\Delta t q_\eta,\quad \hat b^-=\hat b.
$$

The local estimator state-space matrices are

$$
F=\begin{bmatrix}
I&\Delta tI&\frac12\Delta t^2J_a&-\frac12\Delta t^2R_b^n&0\\
0&I&\Delta tJ_a&-\Delta tR_b^n&0\\
0&0&I+\Delta tJ_q&0&-\Delta tE^{-1}\\
0&0&0&I&0\\0&0&0&0&I
\end{bmatrix},\quad
H_{\rm GNSS}=[I_6\ \ 0_{6\times9}],\quad
H_{\rm mag}=[0_{3\times6}\ \ J_m\ \ 0_{3\times6}].
$$

Here $J_a=\partial a/\partial\eta$, $J_q=\partial q_\eta/\partial\eta$,
$J_m=\partial[(R_b^n)^Tm_n]/\partial\eta$ are central-difference
Jacobians. Configured IMU/bias noise is mapped into process covariance
$Q_k$; aiding variances define measurement covariance $R_z$:

$$
P^-=FP^+_{\rm previous}F^T+Q_k,\quad
L=P^-H^T(HP^-H^T+R_z)^{-1},\quad
\hat\zeta^+=\hat\zeta^-+L[z-h(\hat\zeta^-)],
$$

$$
P^+=(I-LH)P^-(I-LH)^T+LR_zL^T.
$$

GNSS correction precedes magnetic correction; the latter's Jacobian is
recomputed. Angles are wrapped and covariance symmetrized. Initialization
uses noisy GNSS/accelerometer/magnetometer readings, not plant truth.
All sensed controllers receive
$\hat x=[\hat r;\hat\eta;\hat v;E^{-1}(\Omega_m-\hat b_g)]$.
The averaged-input noise covariance is conservative and omits adjacent
sample correlation; this is not a certified EKF consistency study.

## Testing

`IMU_KF.c` (attitude estimation) and `PID.c` (control + motor mixing) have
zero HAL/MCU dependencies, so they're covered by a native unit test suite
that builds and runs on the host with plain `gcc` (no ARM toolchain or
hardware required). The tests link the real firmware source directly;
there are no mocks of the filter or control logic.

```sh
make -C tests test
```

Coverage includes rest-state and static-tilt convergence checks for both
attitude estimators, and motor-mixing invariants for the PID loop (equal
motor speeds under pure hover thrust, correct differential thrust under a
roll command, and saturation at both the zero floor and `w_max`). This
suite runs automatically on every push via
[GitHub Actions](.github/workflows/tests.yml).

## Building

This is an STM32CubeIDE project (also buildable with plain
`arm-none-eabi-gcc` and the provided linker script,
`STM32G431KBTX_FLASH.ld`).

1. Open the project in [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
2. Build the `Debug` or `Release` configuration.
3. Flash via ST-Link/SWD (`DroneController Debug.launch` is provided for
   debugging directly from CubeIDE).

The `.ioc` file (`DroneController.ioc`) can be reopened in STM32CubeMX to
regenerate peripheral initialization code; all application logic lives
outside the generated `USER CODE` boundaries and is untouched by
regeneration.

## Status

This is an active personal R&D project rather than a flight-proven,
production flight stack: the Kalman filter and complementary filter are
both implemented and unit-tested, but only the complementary filter has
been flown. Treat the gyro-offset constants and PID gains in
[`PID.h`](Core/Inc/PID.h) as airframe-specific starting points, not
general-purpose defaults.

![Bench test rig, tethered for safety during attitude-control tuning](images/drone-test-rig.jpg)

## Author

Designed, built, and written by **Luca Obwegs**.

## License

This project is licensed under the GNU General Public License v3.0 — see
[LICENSE](LICENSE) for details.
