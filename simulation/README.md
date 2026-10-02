# Quadrotor: LQR, MPC, PID, and aided-IMU Kalman estimation

This simulation asks a practical question: **how well do three different
controllers fly the same circle?** They control the same simulated drone,
first with exact motion information, then with noisy sensors, and finally
with noisy sensors plus wind and slower motor response.
The embedded firmware is unchanged.

The [main README](../README.md#simulation-equations-and-state-space-models)
explains the main equations in plain language. The
[technical guide](MODEL_AND_CONTROL.md) keeps the full derivations and
matrices for readers who want to reproduce the controller/filter design.

## The comparison in plain language

| Part | What it does |
|------|--------------|
| LQR: Linear Quadratic Regulator | Turns the current motion error into a correction using precomputed gains. |
| MPC: Model Predictive Control | Plans motor commands 2 s ahead, respects motor limits, then applies the first move. |
| PID: Proportional-Integral-Derivative | Corrects present error, accumulated error, and velocity/rate error; position and tilt use separate loops. |
| IMU: inertial measurement unit | Measures body acceleration-related force and turning rate; noise and offsets cause drift. |
| RTK-GNSS | Uses satellite positioning with correction data; centimetre accuracy assumes a working fixed RTK solution. |
| Magnetometer | Measures the magnetic field to help estimate orientation, including heading. |
| EKF: extended Kalman filter | Predicts motion from the IMU, then corrects it using satellite and magnetic readings. |

A **state** is the list of values describing motion. A **bias** is a
sensor offset, not a random error in just one reading. **Motor mixing**
turns lift and turning requests into four motor commands. **Anti-windup**
prevents PID's accumulated error from building up when motors cannot
deliver the requested command.

## Run and test

Requires MATLAB, Control System Toolbox, and Model Predictive Control
Toolbox. Results were generated in MATLAB R2024b; Simulink and Sensor
Fusion Toolbox are not required.

From the repository root:

```matlab
run(fullfile('simulation', 'matlab', 'run_controller_comparison.m'))
```

The entry point clears the workspace and closes existing figures.
It runs nine 45 s simulations and exports **six high-resolution PNGs**:
two sensor-case plots and four robustness plots. The ideal case remains
in the numerical results without a separate published figure gallery.
It writes [metrics.csv](results/metrics.csv) with nine controller/scenario
rows. Full truth, estimated states, sampled IMU/aiding measurements,
biases, covariance diagonals, and controller diagnostics are retained
in the workspace's `comparisons` structure. PID runs also expose the
nine-state controller memory as `out.pidState`. For example:

```matlab
% Noisy-sensor scenario, MPC controller
out = comparisons(2).outputs{2};
plot(out.t, out.XEstimated(1,:) - out.X(1,:))
```

To run the MATLAB regression suite:

```matlab
results = runtests(fullfile('simulation', 'tests'));
assertSuccess(results);
```

The tests cover plant linearization and independent yaw, hover, MPC
scaling and active constraints, infeasible solver reporting, IMU frame
conventions and gravity, noise statistics and aiding sample rates, EKF
covariance/bias correction, PID hover/mixing and anti-windup, repeatable
matched noise sequences, and complete noisy and disturbed comparisons.
Publication tests also check the exact six-figure selection, rejection
of unknown figure names, and metrics-only operation without creating
figures or directories.

To reproduce the three-seed numerical audit without generating plots:

```matlab
addpath(fullfile('simulation', 'matlab'));
report = validateResults(20261002:20261004);
```

This runs 18 sensed/disturbed comparisons using the same
[experiment configuration](matlab/comparisonConfiguration.m) as the
plot runner and writes [validation.csv](results/validation.csv).
It recomputes the RMS tracking distance independently and checks for
invalid numbers, commands outside motor limits, commands changing too
quickly, invalid final filter uncertainty, and failed MPC calculations.

Optional full diagnostic galleries can be regenerated **after** the main
run without repeating the simulations:

```matlab
for i = 1:numel(comparisons)
    directory = fullfile('simulation', 'results', 'local', cfg.scenarios(i).slug);
    plotComparison(comparisons(i).outputs, cfg.p, directory, cfg.traj.rampTime, {'all'});
end
```

The local directory is ignored by Git. Omitting the final selection
argument also exports all applicable plots; passing `{}` computes only
metrics. The [publication policy](matlab/comparisonPlotSelection.m)
defines the six images committed to the repository.

## Controllers and comparison fairness

- **LQR:** corrects position, orientation, velocity, and angle-rate errors
  using a model designed for small tilts near hover.
- **MPC:** uses the same model and error/effort priorities as LQR, but
  looks ahead along the path and plans commands within motor limits.
- **PID:** uses one loop to request acceleration and another to control
  tilt. Planned acceleration helps it follow the path; accumulated error
  helps correct persistent offsets. Motor mixing is for a plus-shaped frame.

All controllers run at 20 Hz and share rotor bounds and a command step
limit of 20% of hover squared speed per sample. The simulated drone,
IMU, and filter advance at 200 Hz. The same filter design, sensor
configuration, seed, force, and lag are used for all controllers.
Each controller has its own filter, but receives the same sequence of
random noise/offset changes for a given seed. Actual sensor readings
still differ because the drones follow slightly different paths.

In sensed experiments **none of the controllers receive the true plant
state**. MPC's built-in filter is disabled so it uses the same EKF design
as the others. The exact simulated state ("truth") is used only
to integrate the plant, synthesize sensors, and score/plot performance.
The ideal baseline explicitly bypasses the estimator.

PID is not designed using the same error/effort score as LQR/MPC. It
also uses planned acceleration and accumulated error. Its gains are
documented starting points, not a claim of the best possible tuning.
Neither LQR nor MPC's design model explicitly includes motor lag or
wind. The IMU nevertheless measures the actual resulting acceleration,
as a physical accelerometer would.

## Sensor assumptions

Chosen in [sensorParameters.m](matlab/sensorParameters.m):

| Sensor/model | Rate | Noise standard deviation per reading |
|--------------|------|---------------------------|
| Body accelerometer specific force | 200 Hz | 0.04 m/s^2 per axis |
| Body gyroscope angular rate | 200 Hz | 0.08 deg/s per axis |
| RTK-GNSS position | 10 Hz | 2 cm horizontal, 4 cm vertical |
| RTK-GNSS velocity | 10 Hz | 0.03 m/s horizontal, 0.05 m/s vertical |
| Three-axis magnetometer | 20 Hz | 0.35 microtesla per axis |

Residual accelerometer bias starts at `[0.020,-0.015,0.025]` m/s^2,
gyro bias at `[0.12,-0.10,0.08]` deg/s. Both drift as independent
random walks: 0.001 m/s^2/sqrt(s) and 0.005 deg/s/sqrt(s).
The local navigation-frame magnetic field is `[20,0,45]` microtesla.
The simulation seed is `20261002`.

The noise column describes typical random spread, not a hard error
bound. Biases listed above are added separately. "Random walk" means the
offset changes a little at each tick rather than remaining constant.

These are **representative simulation assumptions, not measurements
from the repository's physical MPU6050 or a claimed device calibration**.
They model white sample noise, residual calibration errors, and bias
drift at the stated bandwidth. They do not include vibration, thermal
drift, clipping, magnetic interference, GNSS multipath, RTK fix loss,
correction-link outages, sensor latency, or timestamp errors.
Centimetre GNSS accuracy assumes an available, fixed RTK solution; it
is not ordinary unaided GPS performance.

An IMU alone cannot maintain reliable absolute position or heading:
small errors build up over time. Satellite position/velocity and a known
magnetic field help correct that drift. The EKF estimates position,
velocity, orientation, and accelerometer/gyro offsets (15 values).
It starts from the first noisy satellite and acceleration/magnetic
readings, assuming little initial motion. It never uses the exact
simulated angles as a substitute sensor.

## Plant and experiments

State order is `[position; Euler angles; velocity; Euler angle rates]`.
The physical model includes thrust, gravity, and three-dimensional
rotation. Turning rates measured in drone axes are converted into
roll/pitch/yaw rates; they are not assumed to be the same when tilted.
The plus-frame mixer can control lift, roll, pitch, and yaw independently.
Controllers and drone model use the same physical parameters.
See the [model derivation](MODEL_AND_CONTROL.md).

| Setting | Value |
|---------|-------|
| Radius / period | 2 m / 30 s |
| Altitude / smooth start | 2.5 m / 5 s |
| Duration | 45 s |
| Controller / IMU / GNSS / magnetometer rate | 20 / 200 / 10 / 20 Hz |
| Initial plant state / motor input | All zeros / hover |
| Ideal baseline | Perfect state, no wind, no lag |
| Sensor case | Noisy IMU + RTK + magnetometer; EKF feedback |
| Robustness case | Same sensors/EKF + wind + 40 ms motor lag |

The wind test smoothly applies a changing force from 12 to 28 s, then
removes it. This is a repeatable disturbance, not a model of measured
airflow. The exact force formula is documented in
[comparisonScenarios.m](matlab/comparisonScenarios.m).
The 40 ms lag delays the response of squared motor speed.

Zero altitude is a free-flight coordinate origin, not physical ground.
Ground contact, drag, actuator identification, and communications
delays are not included.

## Verified results

**3D RMS error** summarizes distance from the target over the full run,
giving larger misses more weight. Smaller is better. It combines all
three directions; it is not the maximum error. In code it is
`sqrt(mean(ex^2 + ey^2 + ez^2))`.

| Scenario | LQR RMS | MPC RMS | PID RMS |
|----------|---------|---------|---------|
| Perfect-state ideal | 1.78 cm | 0.27 cm | 0.39 cm |
| Noisy sensors + EKF | 3.31 cm | 2.45 cm | 3.07 cm |
| Sensors + wind + lag | 5.63 cm | 5.17 cm | 11.39 cm |

In the last case peak distances are **13.04 cm LQR, 13.08 cm MPC,
30.06 cm PID**. Position-estimation 3D RMS is approximately **1.79 cm**
for all three controllers. Combined roll/pitch/yaw estimation RMS is
0.31-0.44 deg across the sensed runs. Startup errors are included.

The figures and table use one deterministic noise realization.
The small ideal errors depend on perfect state feedback, accurate
parameters, a slow path, and use of planned motion.

### Interpretation and seed sensitivity

MPC's lower **whole-run** RMS should not be read as better wind rejection
in every axis. Disturbed LQR/MPC x RMSE is 3.95/3.95 cm, y RMSE is
2.98/3.03 cm, and circle-only RMS is 5.32/5.34 cm. Vertical RMSE is
2.68/1.42 cm. Startup/vertical behavior accounts for the principal
whole-run benefit, consistent with MPC's reference preview. The
circle-only interval starts at 5 s and still includes settling and wind.

The check uses three random-noise seeds, `20261002`-`20261004`. Each seed
produces a different noise sequence, matched across controllers:

| Scenario/controller | Whole-run 3D RMS range | Wind-window RMS, 12-28 s | Recovery RMS, 32-45 s |
|---------------------|------------------------|-------------------------|----------------------|
| Sensors / LQR | 2.24-3.31 cm | 1.81-2.45 cm | 1.84-2.15 cm |
| Sensors / MPC | 2.04-2.45 cm | 1.81-2.48 cm | 1.84-2.15 cm |
| Sensors / PID | 2.34-3.07 cm | 2.25-2.95 cm | 2.16-2.32 cm |
| Wind + lag / LQR | 4.61-5.63 cm | 6.73-7.96 cm | 1.90-2.22 cm |
| Wind + lag / MPC | 4.28-5.17 cm | 6.73-8.01 cm | 1.90-2.20 cm |
| Wind + lag / PID | 9.80-11.39 cm | 13.85-15.49 cm | 5.50-6.00 cm |

The sensor-only rows use the same time windows but contain **no wind**.
PID's larger wind-window and recovery errors are consistent with its
current tuning and recovery of its accumulated error; no gains were
changed to manufacture a ranking. PID uses a different control structure,
so these results compare the documented designs, not the best possible
version of each controller.

All 18 checked runs stay **within motor and command-change limits**.
Motor saturation or excessive integral buildup does not explain the PID results, and
these runs do not prove MPC's constraint advantage. Dedicated tests
exercise constraints and anti-windup. Maximum roll/pitch is 4.71 degrees
across the audit, consistent with a near-hover model. All numerical
checks passed, but three noise sequences are not enough to establish
statistical reliability or real-flight accuracy. The filter's estimated
uncertainty also contains approximations; it has not been proven to
match actual error in every condition.

[trackingMetrics.m](matlab/trackingMetrics.m) separately reports:

- Whole-run and circle-only (`t >= 5 s`) 3D RMS, per-axis errors, and peaks.
- A legacy coordinate-averaged RMSE, smaller by `sqrt(3)`; do not confuse
  it with the plotted 3D RMS.
- Size of motor-command changes from hover, not electrical energy use.
- Position/orientation estimation errors, measured against the exact
  simulated state. Angle differences account for wrapping at a full turn.
- Host controller timing, excluding the 200 Hz EKF and plant simulation.
  It is not a hardware real-time guarantee.

## Six published comparison plots

### Noisy sensors and EKF

![Sensed metrics](results/sensors/rmse_summary.png)
![Raw body accelerometer and gyro, representative MPC run](results/sensors/imu_measurements.png)

### Wind, motor lag, and noisy sensors

![Disturbed circle and zoom](results/robustness/xy_tracking_comparison.png)
![Disturbed metrics](results/robustness/rmse_summary.png)
![Disturbed error magnitudes](results/robustness/tracking_error_comparison.png)
![Disturbed state estimation errors](results/robustness/estimation_errors.png)

## Source entry points

- [run_controller_comparison.m](matlab/run_controller_comparison.m)
- [comparisonConfiguration.m](matlab/comparisonConfiguration.m),
  [comparisonPlotSelection.m](matlab/comparisonPlotSelection.m),
  [validateResults.m](matlab/validateResults.m)
- [designLQR.m](matlab/designLQR.m), [designMPC_Linear.m](matlab/designMPC_Linear.m),
  [designPID.m](matlab/designPID.m), [pidControl.m](matlab/pidControl.m)
- [sensorParameters.m](matlab/sensorParameters.m), [sampleSensors.m](matlab/sampleSensors.m)
- [initializeNavigationFilter.m](matlab/initializeNavigationFilter.m),
  [navigationFilterStep.m](matlab/navigationFilterStep.m),
  [navigationControllerState.m](matlab/navigationControllerState.m)
- [simulateClosedLoop.m](matlab/simulateClosedLoop.m),
  [comparisonScenarios.m](matlab/comparisonScenarios.m),
  [plotComparison.m](matlab/plotComparison.m)
- [testControllers.m](tests/testControllers.m), [testSensorsAndPID.m](tests/testSensorsAndPID.m),
  [testResultPublication.m](tests/testResultPublication.m)

The separate [nonlinear MPC template](matlab/designNMPC.m) remains
experimental; the comparison uses tested **linear MPC**.
The roll/pitch/yaw representation is not suitable near a 90-degree pitch.
This study
does not certify any controller gains or authorize hardware deployment.
