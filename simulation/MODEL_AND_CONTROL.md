# Plant, sensors, controllers, and estimator equations

The equations below match the MATLAB implementation. `R_b^n` is a
body-to-navigation rotation; `R_u` is an input cost matrix; `R_z` is a
measurement covariance. These are different quantities. Positions are
metres, time seconds, angles radians, magnetic field microtesla.

## 1. Nonlinear plant and physical motor mixing

The plant state and rotor inputs are

$$
x = \begin{bmatrix}r\\\eta\\v\\\dot\eta\end{bmatrix}\in\mathbb R^{12},
\quad \eta=(\phi,\theta,\psi)^T,\quad
u=(\omega_1^2,\omega_2^2,\omega_3^2,\omega_4^2)^T.
$$

Euler angles use the ZYX convention:

$$
R_b^n=R_z(\psi)R_y(\theta)R_x(\phi),\qquad
\Omega=E(\eta)\dot\eta,
$$

$$
E=
\begin{bmatrix}
1&0&-\sin\theta\\
0&\cos\phi&\sin\phi\cos\theta\\
0&-\sin\phi&\cos\phi\cos\theta
\end{bmatrix}.
$$

The **standard plus-configuration** mixer, not an X-frame mixer, is

$$
\begin{bmatrix}T\\\tau_x\\\tau_y\\\tau_z\end{bmatrix}
= M u,\qquad
M=
\begin{bmatrix}
k_t&k_t&k_t&k_t\\
lk_t&0&-lk_t&0\\
0&lk_t&0&-lk_t\\
d&-d&d&-d
\end{bmatrix}.
$$

Thus `u = M \ [T; torque]`. Physical rotor angular speeds, if required,
are $\omega_i=\sqrt{u_i}$ after nonnegative input bounding.

The nonlinear equations used in
[QuadrotorStateFcn.m](matlab/QuadrotorStateFcn.m) are

$$
\dot r=v,\qquad
\dot v=\frac{T}{m}R_b^n e_3-ge_3+\frac{F_w}{m},
$$

$$
I\dot\Omega=\tau-\Omega\times(I\Omega),\qquad
\ddot\eta=E^{-1}\left[\dot\Omega-\dot E\dot\eta\right].
$$

Here $I=\operatorname{diag}(I_x,I_y,I_z)$ and $e_3=(0,0,1)^T$.
The force $F_w$ is added by the simulator, not by the controllers.
The motor-lag scenario additionally integrates

$$
\dot u_{\rm applied}=(u_{\rm commanded}-u_{\rm applied})/\tau_m,
\qquad \tau_m=0.04\ {\rm s}.
$$

All controllers share physical bounds and the command step limit

$$
0\leq u_{i,k}\leq u_{\max},\qquad
|u_{i,k}-u_{i,k-1}|\leq0.2u_h,\qquad
u_h=\frac{mg}{4k_t}.
$$

## 2. Hover state-space model

With $\delta u=u-u_h\mathbf1$, the continuous linear model is

$$
\dot x=A x+B\delta u,\qquad y=Cx,\quad C=I_{12},\quad D=0.
$$

In the plant ordering $[r;\eta;v;\dot\eta]$,

$$
A=
\begin{bmatrix}
0&0&I_3&0\\
0&0&0&I_3\\
0&G&0&0\\
0&0&0&0
\end{bmatrix},
\quad
G=\begin{bmatrix}0&g&0\\-g&0&0\\0&0&0\end{bmatrix},
$$

$$
B=
\begin{bmatrix}
0_{6\times4}\\
B_v\\
B_\eta
\end{bmatrix},\qquad
B_v=\frac{k_t}{m}
\begin{bmatrix}0&0&0&0\\0&0&0&0\\1&1&1&1\end{bmatrix},
$$

$$
B_\eta=
\begin{bmatrix}
lk_t/I_x&0&-lk_t/I_x&0\\
0&lk_t/I_y&0&-lk_t/I_y\\
d/I_z&-d/I_z&d/I_z&-d/I_z
\end{bmatrix}.
$$

Exact zero-order-hold discretization at $T_s=0.05$ s gives

$$
A_d=e^{AT_s},\qquad B_d=\int_0^{T_s}e^{A t}B\,dt,\qquad
x_{k+1}=A_dx_k+B_d\delta u_k.
$$

See [quadrotorLinearModel.m](matlab/quadrotorLinearModel.m).
The regression suite verifies this model against a central-difference
Jacobian of the nonlinear plant and checks independent yaw authority.

## 3. Discrete LQR

The balanced penalties in [designLQR.m](matlab/designLQR.m) are

$$
Q=\operatorname{diag}(40,40,60,6,6,3,2,2,3,0.3,0.3,0.2),
\qquad R_u=10^{-10}I_4.
$$

The discrete Riccati equation and gain are

$$
P=A_d^TPA_d+Q-A_d^TPB_d(R_u+B_d^TPB_d)^{-1}B_d^TPA_d,
$$

$$
K=(R_u+B_d^TPB_d)^{-1}B_d^TPA_d,\qquad
u_k=u_h\mathbf1-K(\hat x_k-x_{{\rm ref},k}).
$$

For an unsaturated regulator around a fixed reference,
$A_{\rm closed}=A_d-B_dK$. The reference is moving here, so regulator
stability is not a proof of exact trajectory tracking.

LQR itself is a **static state-feedback controller**: it has no internal
dynamic state. Its controller realization has $D_c=-K$ for input
$e=\hat x-x_{\rm ref}$ and empty $A_c,B_c,C_c$.
The separate EKF provides its dynamic state estimate. The reference
includes velocity, acceleration-derived roll/pitch, and angle rates,
but the LQR rotor command adds only the hover input.

## 4. Constrained linear MPC

MPC uses normalized deviations
$q=(u-u_h\mathbf1)/u_h$, with prediction model

$$
x_{j+1|k}=A_dx_{j|k}+B_d u_h q_{j|k},\qquad
x_{0|k}=\hat x_k.
$$

The QP minimizes

$$
\sum_{j=1}^{N_p}
\left[
(x_{j|k}-x_{{\rm ref},k+j})^TQ(x_{j|k}-x_{{\rm ref},k+j})
+q_{j-1|k}^T(u_h^2R_u)q_{j-1|k}
\right],
$$

subject to

$$
-1\leq q_{i,j|k}\leq(u_{\max}-u_h)/u_h,\qquad
|q_{i,j|k}-q_{i,j-1|k}|\leq0.2.
$$

At the first move, $q_{-1|k}$ is the previous bounded **command**, not
the unmeasured lagged actuator state. There is no move-rate cost;
move rates are hard constraints. The output weights are $\sqrt{Q_{ii}}$
and input weights $u_h\sqrt{(R_u)_{ii}}$ with unit scale factors.

$N_p=40$ (2 s) and $N_c=20$ (1 s); the last optimized move is held
through the rest of the prediction horizon. Only the first command is
applied and the QP is solved again at the next sample. There is no
terminal Riccati cost or terminal state constraint.

The prediction model has $(A_d,u_hB_d,I,0)$ as above. A constrained
receding-horizon controller has **no single global LTI controller
matrix**: the feedback law is a piecewise-affine parametric QP solution
in the estimated state, future references, and previous command.
The `mpcstate` plant state is overwritten with $\hat x_k$; built-in
state estimation is disabled. Solver warm starts and the previous move
are implementation memory, not access to the truth state.

See [designMPC_Linear.m](matlab/designMPC_Linear.m).
Nonpositive solver status explicitly aborts the run.

## 5. Cascaded PID controller and controller state

The position loop in [pidControl.m](matlab/pidControl.m) uses

$$
e_r=r_{\rm ref}-\hat r,\qquad e_v=v_{\rm ref}-\hat v,
$$

$$
z_{r,k}^+=\operatorname{clip}(z_{r,k}+T_s e_{r,k},L_r),
\quad
a_c=\operatorname{clip}(a_{\rm ref}+K_p^r e_r+K_d^r e_v+K_i^r z_r^+,L_a).
$$

`clip(z,L)` bounds each entry between `-L` and `L`. Derivative damping
uses the velocity estimate, not finite differences of noisy position.
For desired yaw $\psi_r$, the small-angle attitude target is

$$
\phi_c=(\sin\psi_r\,a_{c,x}-\cos\psi_r\,a_{c,y})/g,
\quad
\theta_c=(\cos\psi_r\,a_{c,x}+\sin\psi_r\,a_{c,y})/g,
\quad \psi_c=\psi_r.
$$

Roll and pitch targets are bounded to $\pm25^\circ$.
With $\operatorname{wrap}(a)=\operatorname{atan2}(\sin a,\cos a)$,

$$
e_\eta=\operatorname{wrap}(\eta_c-\hat\eta),\quad
z_{\eta,k}^+=\operatorname{clip}(z_{\eta,k}+T_s e_{\eta,k},L_\eta),
$$

$$
\dot\eta_{c,k}=(\eta_{c,k}-s_k)/T_s,\quad
\alpha_c=K_p^\eta e_\eta+K_d^\eta(\dot\eta_c-\widehat{\dot\eta})
+K_i^\eta z_\eta^+,
$$

$$
\tau_c=I\alpha_c,\qquad
T_c=\frac{m(g+a_{c,z})}{\max(0.5,\cos\hat\phi\cos\hat\theta)},\qquad
u_{\rm requested}=M^{-1}(T_c,\tau_c^T)^T.
$$

This inner loop approximates Euler accelerations as body accelerations
near hover; it is not nonlinear computed-torque control.

The controller's internal state is

$$
\xi_k=(z_{r,k}^T,z_{\eta,k}^T,s_k^T)^T\in\mathbb R^9,\qquad s_k=\eta_{c,k-1}.
$$

Its exact nonlinear state realization is the two integral updates above
and $s_{k+1}=\eta_{c,k}$, followed by shared motor bounds. If any motor
input or command-rate bound activates, **both integrals retain their
previous values** (conditional-integration anti-windup), while $s$
still updates. Integral clamping remains active at all times.

For any unsaturated individual PI-plus-velocity-damping block, a local
discrete controller realization using the updated integral is

$$
z_{k+1}=z_k+T_s e_k,\qquad
c_k=K_i z_k+(K_p+T_sK_i)e_k+K_d e_{\dot k},
$$

so $A_c=I$, $B_c=[T_sI\ \ 0]$, $C_c=K_i$,
$D_c=[K_p+T_sK_i\ \ K_d]$. Cascading these blocks with the attitude
target map, reference-difference memory, nonlinear thrust compensation,
and saturations gives the implemented nine-state controller; it is
not globally LTI.

Default gains from [designPID.m](matlab/designPID.m):

| Loop | Kp (x/roll, y/pitch, z/yaw) | Ki | Kd |
|------|----------------------------|----|----|
| Position | 2, 2, 3 | 0.3, 0.3, 0.8 | 2.5, 2.5, 2.8 |
| Attitude | 22, 22, 10 | 2, 2, 1 | 7, 7, 5 |

Position integrals are bounded to `[2,2,1]` m*s; attitude integrals
to `[10,10,10]` deg*s; acceleration targets to `[3,3,4]` m/s^2.
These are explicit simulation starting-point gains, not universal
hardware tuning. PID has acceleration feedforward and integral action,
so its design objective is not the matched quadratic LQR/MPC objective.

## 6. Sensor measurement equations

At each 200 Hz IMU tick, the synthetic sensors produce

$$
f_m=(R_b^n)^T(\dot v+ge_3)+b_a+n_a,\qquad
\Omega_m=E(\eta)\dot\eta+b_g+n_g.
$$

The accelerometer measures **specific force** in body axes, not inertial
acceleration. A stationary level vehicle reads $(0,0,g)^T$; ideal free
fall reads zero. Wind-induced acceleration is included in this sensor.
The gyro measures body rates, not Euler rates.

Biases follow

$$
b_{a,k+1}=b_{a,k}+\sigma_{ba}\sqrt{\Delta t}\epsilon_{a,k},\qquad
b_{g,k+1}=b_{g,k}+\sigma_{bg}\sqrt{\Delta t}\epsilon_{g,k},
$$

with independent standard-normal draws. RTK-GNSS and magnetic aiding:

$$
z_{\rm GNSS}=(r^T,v^T)^T+n_{\rm GNSS},\qquad
z_{\rm mag}=(R_b^n)^Tm_n+n_m.
$$

No noisy "true attitude" measurement is provided. The magnetometer
measures a three-axis field vector. Noise parameters, rates, and biases
are listed in the [simulation guide](README.md) and implemented in
[sensorParameters.m](matlab/sensorParameters.m).
The same seed and standardized draw sequence are used for each
controller. Their sensor values differ because the plant trajectories
differ, but the injected noise/bias realizations are matched.

## 7. Fifteen-state navigation extended Kalman filter

Navigation filter order differs from plant order:

$$
\zeta=(r^T,v^T,\eta^T,b_a^T,b_g^T)^T\in\mathbb R^{15}.
$$

The filter's nonlinear continuous state model is

$$
\dot r=v,\qquad
\dot v=R_b^n(\eta)(f_m-b_a)-ge_3,\qquad
\dot\eta=E(\eta)^{-1}(\Omega_m-b_g),\qquad
\dot b_a=w_{ba},\quad\dot b_g=w_{bg}.
$$

The discrete prediction uses the average of the previous and current
IMU samples. With $a=R_b^n(\bar f_m-\hat b_a)-ge_3$ and
$q=E^{-1}(\bar\Omega_m-\hat b_g)$, at $\Delta t=0.005$ s:

$$
\hat r^-=\hat r+\Delta t\hat v+\tfrac12\Delta t^2a,\quad
\hat v^-=\hat v+\Delta t a,\quad
\hat\eta^-=\hat\eta+\Delta t q,\quad
\hat b_a^-=\hat b_a,\quad\hat b_g^-=\hat b_g.
$$

The local discrete transition matrix, evaluated at the current estimate, is

$$
F=
\begin{bmatrix}
I&\Delta tI&\tfrac12\Delta t^2J_a&-\tfrac12\Delta t^2R_b^n&0\\
0&I&\Delta tJ_a&-\Delta tR_b^n&0\\
0&0&I+\Delta tJ_q&0&-\Delta t E^{-1}\\
0&0&0&I&0\\
0&0&0&0&I
\end{bmatrix},
$$

where $J_a=\partial[R_b^n(f_m-b_a)]/\partial\eta$ and
$J_q=\partial[E^{-1}(\Omega_m-b_g)]/\partial\eta$. These three-column
Jacobians use central differences with a $10^{-5}$ rad step.

The noise injection matrix and process covariance are

$$
G_k=
\begin{bmatrix}
\tfrac12\Delta t^2R_b^n&0&0&0\\
\Delta tR_b^n&0&0&0\\
0&\Delta tE^{-1}&0&0\\
0&0&\sqrt{\Delta t}I&0\\
0&0&0&\sqrt{\Delta t}I
\end{bmatrix},\qquad
Q_k=G_k\,\operatorname{diag}(\sigma_a^2I,\sigma_g^2I,
\sigma_{ba}^2I,\sigma_{bg}^2I)\,G_k^T,
$$

$$
P_k^-=F_kP_{k-1}^+F_k^T+Q_k.
$$

Per-sample IMU variances are used conservatively for the averaged input;
this approximation does not explicitly model adjacent averaged-sample
correlation. The covariance is an EKF uncertainty approximation, not
an independently certified consistency result.

Measurement matrices are

$$
H_{\rm GNSS}=[I_6\ \ 0_{6\times9}],\qquad
H_{\rm mag}=[0_{3\times6}\ \ J_m\ \ 0_{3\times6}],
$$

where $J_m=\partial[(R_b^n)^Tm_n]/\partial\eta$ is evaluated by the
same central-difference method. Magnetic and GNSS covariance matrices
come directly from their configured per-sample standard deviations.
At an aiding update:

$$
S=HP^-H^T+R_z,\quad L=P^-H^TS^{-1},\quad
\hat\zeta^+=\hat\zeta^-+L(z-h(\hat\zeta^-)),
$$

$$
P^+=(I-LH)P^-(I-LH)^T+LR_zL^T.
$$

This Joseph-form update preserves covariance symmetry and numerical
positive semidefiniteness. GNSS is updated first, then the magnetic
prediction and Jacobian are recomputed for the sequential mag update.
Angles are wrapped after correction.

Initialization uses the first noisy GNSS fix, approximately stationary
accelerometer tilt, and tilt-compensated magnetic heading. Bias estimates
start at zero. No initial plant-state vector, true wind, true rotor input,
or simulated bias is passed to the filter.
Initial one-sigma uncertainties: GNSS noise for position/velocity,
`[2,2,3]` deg for attitude, 0.05 m/s^2 for accelerometer biases, and
0.3 deg/s for gyro biases.

The controller-facing estimate is reordered to

$$
\hat x=
\begin{bmatrix}
\hat r\\\hat\eta\\\hat v\\E(\hat\eta)^{-1}(\Omega_m-\hat b_g)
\end{bmatrix}.
$$

All three controllers use this estimate in sensor-enabled scenarios.
Truth is retained **only** for plant integration, measurement generation,
performance metrics, and labelled plots. The perfect-state baseline
explicitly bypasses sensing/estimation.

See [navigationFilterStep.m](matlab/navigationFilterStep.m) and
[initializeNavigationFilter.m](matlab/initializeNavigationFilter.m).
