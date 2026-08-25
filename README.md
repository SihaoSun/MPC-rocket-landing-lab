# 2D Rocket Landing: Handwritten SQP + Nonlinear MPC

A zero-dependency browser demonstration of planar rocket landing. The rocket starts from a randomized position and attitude with zero initial velocity. At every MPC sample, the controller solves for thrust magnitude `T` and gimbal angle `δ` to land at a selected horizontal position with zero velocity and an upright attitude.

For the full model and OCP equations, see [MATHEMATICAL_FORMULATION.md](./MATHEMATICAL_FORMULATION.md).

## Run

Open `index.html` directly, or start a local static server in this directory:

```bash
python3 -m http.server 8000
```

Then visit `http://localhost:8000`.

## Features

- Random initial position and tilt, always with zero translational and angular velocity;
- Start, pause, reset, randomize, and recorded replay controls;
- Completed simulations replay at `0.1×` real time with their recorded MPC predictions;
- Configurable mass, cylinder radius, body height, engine lever arm, drag, maximum thrust, and gimbal limit;
- Moment of inertia derived automatically from the cylinder model;
- Configurable horizon size from 1 to 70, MPC sample time, SQP/QP iterations, terminal scale, and cost weights;
- Fixed `0.01 s` plant-simulation step, independent of the MPC sample time;
- Optional 30-degree downward terminal-velocity cone constraint;
- Simultaneous actual and MPC-predicted trajectories, with one visible state node per horizon step;
- Actual/predicted input histories with one control node per horizon step, visible feasible regions, and constraint boundaries;
- Live cost reduction, line-search step, QP residual, terminal error, and active-constraint diagnostics.

## Implementation

The implementation uses no optimization library. Each MPC update:

1. Rolls out the nonlinear dynamics with the current input sequence;
2. Computes analytic discrete-time Jacobians `A_k = ∂f/∂x` and `B_k = ∂f/∂u`;
3. Recursively condenses state sensitivities with respect to all controls;
4. Constructs a dense Gauss–Newton QP;
5. Solves the box-constrained QP using handwritten projected coordinate descent;
6. Handles the optional terminal cone inequalities with augmented-Lagrangian iterations and weighted half-space projection;
7. Uses backtracking line search on the nonlinear merit function;
8. Applies the first input and shifts the solution to warm-start the next sample.

## Validation

The deterministic regression scenario is

```text
Initial state: px=4.5 m, pz=8.0 m, vx=0, vz=0, theta=12 deg, omega=0
Result: 7.96 s soft landing, 0.14 m position error,
        0.18 m/s touchdown speed, 0.9 deg tilt
```

Open `index.html?verify=1` to reproduce it. The ordinary page uses a random initial state. Extreme parameters, an excessively short horizon, or insufficient maximum thrust can make landing infeasible; this is useful for demonstrating constraint activity and MPC tuning.

## Files

- `index.html`: GUI structure and configurable parameters;
- `styles.css`: responsive visual design;
- `mpc.js`: dynamics, objective, SQP, QP, and warm start;
- `app.js`: closed-loop simulation and Canvas visualization;
- `MATHEMATICAL_FORMULATION.md`: mathematical model and OCP derivation.
