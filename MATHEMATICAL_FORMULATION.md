# Mathematical Formulation

This document describes the planar rocket model and the finite-horizon optimal control problem (OCP) implemented in `mpc.js`.

## 1. Coordinates, state, and input

The inertial frame uses horizontal coordinate (p_x) and upward vertical coordinate (p_z). The angle (	heta=0) denotes a vertical rocket, and positive (	heta) tilts its longitudinal axis toward positive (x). The thrust gimbal angle (delta) is measured relative to that axis, so the thrust direction in the inertial frame is (	heta+delta).

The state and control vectors are

$$
x =
\begin{bmatrix}
p_x & p_z & v_x & v_z & \theta & \omega
\end{bmatrix}^{\mathsf T},
\qquad
u =
\begin{bmatrix}
T & \delta
\end{bmatrix}^{\mathsf T},
$$

where (T\geq 0) is thrust magnitude and (omega) is pitch rate.

### 1.1 Cylinder inertia model

The rocket body is approximated by a uniform solid cylinder of mass (m), radius (r), and height (h_b). Its pitch moment of inertia about a transverse axis through the center of mass is

$$
I = \frac{m}{12}\left(3r^2+h_b^2\right).
$$

This relation is why inertia is displayed as a derived, read-only quantity in the GUI.

## 2. Continuous-time nonlinear model

Let (g) be gravitational acceleration, (c_d) the translational linear-drag coefficient, (c_\omega) the angular-damping coefficient, and (l) the engine lever arm. Define

$$
\phi = \theta+\delta.
$$

The translational and angular accelerations are

$$
\begin{aligned}
a_x &= \frac{T}{m}\sin\phi-\frac{c_d}{m}v_x,\\
a_z &= \frac{T}{m}\cos\phi-g-\frac{c_d}{m}v_z,\\
\alpha &= -\frac{lT}{I}\sin\delta-\frac{c_\omega}{I}\omega.
\end{aligned}
$$

Therefore,

$$
\dot{x}=f_c(x,u)=
\begin{bmatrix}
v_x\\
v_z\\
a_x\\
a_z\\
\omega\\
\alpha
\end{bmatrix}.
$$

The torque sign is a convention of this planar model: positive gimbal angle produces negative pitch acceleration.

## 3. Discrete-time prediction model

For sample time (\Delta t), accelerations are evaluated at ((x_k,u_k)) and assumed constant over one sample. The discrete dynamics (x_{k+1}=f_d(x_k,u_k)) are

$$
\begin{aligned}
p_{x,k+1} &= p_{x,k}+\Delta t\,v_{x,k}+\frac{\Delta t^2}{2}a_{x,k},\\
p_{z,k+1} &= p_{z,k}+\Delta t\,v_{z,k}+\frac{\Delta t^2}{2}a_{z,k},\\
v_{x,k+1} &= v_{x,k}+\Delta t\,a_{x,k},\\
v_{z,k+1} &= v_{z,k}+\Delta t\,a_{z,k},\\
\theta_{k+1} &= \theta_k+\Delta t\,\omega_k+\frac{\Delta t^2}{2}\alpha_k,\\
\omega_{k+1} &= \omega_k+\Delta t\,\alpha_k.
\end{aligned}
$$

The same nonlinear discrete model is used by both the MPC prediction and the simulated plant.

## 4. Reference state

For landing target (p_x^\star), the reference is constant over the entire horizon:

$$
x^{\mathrm{ref}} =
\begin{bmatrix}
p_x^\star & 0 & 0 & 0 & 0 & 0
\end{bmatrix}^{\mathsf T}.
$$

Thus the desired terminal condition is the requested horizontal location, zero altitude, zero translational velocity, vertical attitude, and zero angular rate.

## 5. Finite-horizon OCP

At each MPC update, with measured state (\hat{x}), the controller optimizes a sequence

$$
U=\{u_0,u_1,\ldots,u_{N-1}\}.
$$

Let

$$
e_k=x_k-x^{\mathrm{ref}}
$$

and introduce the normalization matrix

$$
D_x=\operatorname{diag}(5,5,3,3,0.35,0.7).
$$

The configurable base weight matrix is

$$
W_x=\operatorname{diag}
\left(q_p,q_p,q_v,q_v,q_\theta,q_\omega\right),
$$

and the normalized state matrix is

$$
\bar Q=D_x^{-1}W_xD_x^{-1}.
$$

### 5.1 Horizon-dependent state cost

The implementation gradually increases the state weight toward the end of the horizon using

$$
\beta_k=0.12+0.88\left(\frac{k}{N}\right)^2,
\qquad k=1,\ldots,N.
$$

Define

$$
\gamma_k=
\begin{cases}
1, & k<N,\\
\gamma_f, & k=N,
\end{cases}
$$

where (\gamma_f) is the GUI parameter **Terminal scale**. The state-tracking cost is

$$
J_x=\sum_{k=1}^{N}\beta_k\gamma_k\,e_k^{\mathsf T}\bar Qe_k.
$$

### 5.2 Fuel and input-smoothing costs

Let (T_{\max}) and (\delta_{\max}) denote the input limits, and let (u_{-1}=u_{\mathrm{prev}}) be the control applied at the preceding MPC sample. The input cost is

$$
J_u=\sum_{k=0}^{N-1}
\left[
r_T\left(\frac{T_k}{T_{\max}}\right)^2
+r_{\Delta u}
\left(
\left(\frac{T_k-T_{k-1}}{T_{\max}}\right)^2
+\left(\frac{\delta_k-\delta_{k-1}}{\delta_{\max}}\right)^2
\right)
\right].
$$

### 5.3 Soft ground penalty

The controller does not impose a hard altitude constraint. Instead, underground predictions receive

$$
J_{\mathrm{ground}}=
\rho_z\sum_{k=1}^{N}\left[\min(0,p_{z,k})\right]^2,
\qquad \rho_z=220.
$$

The total objective is

$$
J(U;\hat{x})=J_x+J_u+J_{\mathrm{ground}}.
$$

### 5.4 OCP constraints

The nonlinear OCP solved at each sample is

$$
\begin{aligned}
\underset{x_1,\ldots,x_N,\,u_0,\ldots,u_{N-1}}{\operatorname{minimize}}
\quad &J(U;\hat{x})\\
\text{subject to}\quad
&x_0=\hat{x},\\
&x_{k+1}=f_d(x_k,u_k), &&k=0,\ldots,N-1,\\
&0\leq T_k\leq T_{\max}, &&k=0,\ldots,N-1,\\
&-\delta_{\max}\leq\delta_k\leq\delta_{\max}, &&k=0,\ldots,N-1.
\end{aligned}
$$

When **Vertical terminal velocity** is enabled, the OCP additionally contains

$$
v_{x,N}=0.
$$

This constrains the terminal velocity vector to have no horizontal component. It does not directly constrain (v_{z,N}); the zero-velocity reference in (J_x) drives (v_{z,N}) toward zero. When the toggle is off, (v_{x,N}) is regulated only by the cost.

## 6. SQP approximation used in the controller

For a nominal trajectory ((\bar x_k,\bar u_k)), the dynamics are linearized as

$$
\Delta x_{k+1}=A_k\Delta x_k+B_k\Delta u_k,
$$

with analytic Jacobians

$$
A_k=\left.\frac{\partial f_d}{\partial x}\right|_{\bar x_k,\bar u_k},
\qquad
B_k=\left.\frac{\partial f_d}{\partial u}\right|_{\bar x_k,\bar u_k}.
$$

Since the measured initial state is fixed, (\Delta x_0=0). State sensitivities with respect to the stacked control step

$$
d=\begin{bmatrix}\Delta u_0^{\mathsf T}&\cdots&\Delta u_{N-1}^{\mathsf T}\end{bmatrix}^{\mathsf T}
$$

are recursively condensed, producing

$$
\Delta x_k=S_kd.
$$

Using a Gauss–Newton Hessian approximation gives the QP

$$
\begin{aligned}
\underset{d}{\operatorname{minimize}}\quad
&\frac12d^{\mathsf T}Hd+g^{\mathsf T}d\\
\text{subject to}\quad
&-\bar T_k\leq\Delta T_k\leq T_{\max}-\bar T_k,\\
&-\delta_{\max}-\bar\delta_k
\leq\Delta\delta_k
\leq\delta_{\max}-\bar\delta_k.
\end{aligned}
$$

If the optional terminal equality is enabled, its SQP linearization is

$$
e_{v_x}^{\mathsf T}S_Nd=-\bar v_{x,N},
\qquad
e_{v_x}=\begin{bmatrix}0&0&1&0&0&0\end{bmatrix}^{\mathsf T}.
$$

The code solves the box-constrained QP using projected coordinate descent. The optional equality is handled using augmented-Lagrangian iterations followed by a weighted projection onto the equality hyperplane while preserving the box bounds.

Finally, a backtracking line search updates

$$
U^{+}=\bar U+\alpha d,
\qquad
\alpha\in\left\{1,\frac12,\frac14,\frac18,\frac1{16}\right\}.
$$

With the terminal equality enabled, the line search uses the merit function

$$
\Psi(U)=J(U)+\rho_c v_{x,N}^2,
\qquad \rho_c=2000.
$$

Only the first optimized input is applied. The optimized sequence is shifted by one sample to warm-start the next MPC solve.

## 7. Touchdown logic outside the OCP

The GUI declares a successful touchdown when the vehicle reaches the ground, or enters a near-ground settled region, and satisfies the displayed tolerances on position error, speed, and tilt. This event logic belongs to the simulator and is not an additional OCP constraint.
