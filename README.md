# 2D Rocket Landing: Handwritten SQP + Nonlinear MPC

A zero-dependency browser demonstration of planar rocket landing. The rocket starts from a randomized position, velocity, and attitude. At every sample, nonlinear MPC solves for thrust magnitude `T` and gimbal angle `δ` to land at a selected horizontal position with zero velocity and an upright attitude.

For the full model and OCP equations, see [MATHEMATICAL_FORMULATION.md](./MATHEMATICAL_FORMULATION.md).

## Run

Open `index.html` directly, or start a local static server in this directory:

```bash
python3 -m http.server 8000
```

Then visit `http://localhost:8000`.

## Features

- Random initial position, velocity, and tilt;
- Start, pause, reset, and randomize controls;
- Configurable mass, cylinder radius, body height, engine lever arm, drag, maximum thrust, and gimbal limit;
- Moment of inertia derived automatically from the cylinder model;
- Configurable horizon, sample time, SQP/QP iterations, terminal scale, and cost weights;
- Optional terminal vertical-velocity equality constraint;
- Simultaneous actual and MPC-predicted trajectories;
- Actual/predicted input histories with visible feasible regions and constraint boundaries;
- Live cost reduction, line-search step, QP residual, terminal error, and active-constraint diagnostics.

## Implementation

The implementation uses no optimization library. Each MPC update:

1. Rolls out the nonlinear dynamics with the current input sequence;
2. Computes analytic discrete-time Jacobians `A_k = ∂f/∂x` and `B_k = ∂f/∂u`;
3. Recursively condenses state sensitivities with respect to all controls;
4. Constructs a dense Gauss–Newton QP;
5. Solves the box-constrained QP using handwritten projected coordinate descent;
6. Handles the optional terminal equality with augmented-Lagrangian iterations and weighted hyperplane projection;
7. Uses backtracking line search on the nonlinear merit function;
8. Applies the first input and shifts the solution to warm-start the next sample.

## Validation

The deterministic regression scenario is

```text
Initial state: px=4.5 m, pz=8.0 m, vx=-0.2 m/s, vz=-0.7 m/s, theta=12 deg
Result: 6.84 s soft landing, 0.04 m position error,
        0.08 m/s touchdown speed, 0.1 deg tilt
```

Open `index.html?verify=1` to reproduce it. The ordinary page uses a random initial state. Extreme parameters, an excessively short horizon, or insufficient maximum thrust can make landing infeasible; this is useful for demonstrating constraint activity and MPC tuning.

## Files

- `index.html`: GUI structure and configurable parameters;
- `styles.css`: responsive visual design;
- `mpc.js`: dynamics, objective, SQP, QP, and warm start;
- `app.js`: closed-loop simulation and Canvas visualization;
- `MATHEMATICAL_FORMULATION.md`: mathematical model and OCP derivation.
