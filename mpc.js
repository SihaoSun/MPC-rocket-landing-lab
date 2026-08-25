(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  else root.RocketMPC = api;
})(typeof self !== "undefined" ? self : this, function () {
  "use strict";

  const NX = 6;
  const NU = 2;
  const G = 9.81;
  const GROUND_PENALTY = 2500;

  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  function discreteDynamics(x, u, p, withJacobian) {
    const [px, pz, vx, vz, theta, omega] = x;
    const thrust = u[0];
    const delta = u[1];
    const dt = p.dt;
    const phi = theta + delta;
    const s = Math.sin(phi);
    const c = Math.cos(phi);
    const invM = 1 / p.mass;
    const invI = 1 / p.inertia;
    const ax = thrust * invM * s - p.drag * invM * vx;
    const az = thrust * invM * c - G - p.drag * invM * vz;
    const alpha = -p.lever * thrust * Math.sin(delta) * invI - p.angularDamping * omega * invI;
    const h2 = 0.5 * dt * dt;

    const next = [
      px + dt * vx + h2 * ax,
      pz + dt * vz + h2 * az,
      vx + dt * ax,
      vz + dt * az,
      theta + dt * omega + h2 * alpha,
      omega + dt * alpha
    ];
    if (!withJacobian) return { next };

    const daxDvx = -p.drag * invM;
    const dazDvz = -p.drag * invM;
    const daxDtheta = thrust * invM * c;
    const dazDtheta = -thrust * invM * s;
    const dalphaDomega = -p.angularDamping * invI;
    const daxDT = invM * s;
    const dazDT = invM * c;
    const dalphaDT = -p.lever * Math.sin(delta) * invI;
    const daxDdelta = thrust * invM * c;
    const dazDdelta = -thrust * invM * s;
    const dalphaDdelta = -p.lever * thrust * Math.cos(delta) * invI;

    const A = new Float64Array(NX * NX);
    for (let i = 0; i < NX; i++) A[i * NX + i] = 1;
    A[0 * NX + 2] = dt + h2 * daxDvx;
    A[0 * NX + 4] = h2 * daxDtheta;
    A[1 * NX + 3] = dt + h2 * dazDvz;
    A[1 * NX + 4] = h2 * dazDtheta;
    A[2 * NX + 2] += dt * daxDvx;
    A[2 * NX + 4] = dt * daxDtheta;
    A[3 * NX + 3] += dt * dazDvz;
    A[3 * NX + 4] = dt * dazDtheta;
    A[4 * NX + 5] = dt + h2 * dalphaDomega;
    A[5 * NX + 5] += dt * dalphaDomega;

    const B = new Float64Array(NX * NU);
    B[0 * NU + 0] = h2 * daxDT;
    B[0 * NU + 1] = h2 * daxDdelta;
    B[1 * NU + 0] = h2 * dazDT;
    B[1 * NU + 1] = h2 * dazDdelta;
    B[2 * NU + 0] = dt * daxDT;
    B[2 * NU + 1] = dt * daxDdelta;
    B[3 * NU + 0] = dt * dazDT;
    B[3 * NU + 1] = dt * dazDdelta;
    B[4 * NU + 0] = h2 * dalphaDT;
    B[4 * NU + 1] = h2 * dalphaDdelta;
    B[5 * NU + 0] = dt * dalphaDT;
    B[5 * NU + 1] = dt * dalphaDdelta;
    return { next, A, B };
  }

  function rollout(x0, U, p, withSensitivity) {
    const N = U.length / NU;
    const m = U.length;
    const states = [x0.slice()];
    const sensitivities = withSensitivity ? [new Float64Array(NX * m)] : null;
    for (let k = 0; k < N; k++) {
      const u = [U[k * NU], U[k * NU + 1]];
      const dyn = discreteDynamics(states[k], u, p, withSensitivity);
      states.push(dyn.next);
      if (withSensitivity) {
        const prev = sensitivities[k];
        const nextS = new Float64Array(NX * m);
        for (let r = 0; r < NX; r++) {
          const row = r * m;
          for (let q = 0; q < NX; q++) {
            const a = dyn.A[r * NX + q];
            if (Math.abs(a) < 1e-15) continue;
            const prevRow = q * m;
            for (let j = 0; j < m; j++) nextS[row + j] += a * prev[prevRow + j];
          }
          nextS[row + k * NU] += dyn.B[r * NU];
          nextS[row + k * NU + 1] += dyn.B[r * NU + 1];
        }
        sensitivities.push(nextS);
      }
    }
    return { states, sensitivities };
  }

  function stateWeights(config, k, N) {
    const progress = k / N;
    const ramp = 0.35 + 0.65 * progress * progress;
    const terminal = k === N ? config.terminalScale : 1;
    const scales = [5, 5, 3, 3, 0.35, 0.7];
    const base = [config.wPos, config.wPos, config.wVel, config.wVel, config.wAngle, config.wOmega];
    return base.map((w, i) => terminal * ramp * w / (scales[i] * scales[i]));
  }

  function evaluateCost(states, U, config, previousU) {
    const N = U.length / NU;
    const ref = [config.targetX, 0, 0, 0, 0, 0];
    let cost = 0;
    for (let k = 1; k <= N; k++) {
      const q = stateWeights(config, k, N);
      const x = states[k];
      for (let i = 0; i < NX; i++) {
        const e = x[i] - ref[i];
        cost += q[i] * e * e;
      }
      if (x[1] < 0) cost += GROUND_PENALTY * x[1] * x[1];
    }
    const tScale = Math.max(config.maxThrust, 1);
    const dScale = Math.max(config.maxGimbal, 1e-3);
    for (let k = 0; k < N; k++) {
      const ti = k * NU;
      const tn = U[ti] / tScale;
      cost += config.wFuel * tn * tn;
      const prevT = k === 0 ? previousU[0] : U[ti - NU];
      const prevD = k === 0 ? previousU[1] : U[ti - NU + 1];
      const dT = (U[ti] - prevT) / tScale;
      const dD = (U[ti + 1] - prevD) / dScale;
      cost += config.wSmooth * (dT * dT + dD * dD);
    }
    return cost;
  }

  function buildQP(states, sensitivities, U, config, previousU) {
    const N = U.length / NU;
    const m = U.length;
    const H = new Float64Array(m * m);
    const g = new Float64Array(m);
    const ref = [config.targetX, 0, 0, 0, 0, 0];

    for (let k = 1; k <= N; k++) {
      const q = stateWeights(config, k, N);
      const x = states[k];
      const S = sensitivities[k];
      for (let r = 0; r < NX; r++) {
        const qr = q[r];
        if (qr === 0) continue;
        const sr = r * m;
        const e = x[r] - ref[r];
        for (let i = 0; i < m; i++) {
          const si = S[sr + i];
          if (si === 0) continue;
          g[i] += 2 * qr * e * si;
          const hi = i * m;
          for (let j = 0; j <= i; j++) H[hi + j] += 2 * qr * si * S[sr + j];
        }
      }
      if (x[1] < 0) {
        const sr = m;
        for (let i = 0; i < m; i++) {
          const si = S[sr + i];
          g[i] += 2 * GROUND_PENALTY * x[1] * si;
          const hi = i * m;
          for (let j = 0; j <= i; j++) H[hi + j] += 2 * GROUND_PENALTY * si * S[sr + j];
        }
      }
    }

    const tScale2 = config.maxThrust * config.maxThrust;
    const dScale2 = config.maxGimbal * config.maxGimbal;
    for (let k = 0; k < N; k++) {
      const ti = k * NU;
      g[ti] += 2 * config.wFuel * U[ti] / tScale2;
      H[ti * m + ti] += 2 * config.wFuel / tScale2;

      addDifferenceTerm(ti, k === 0 ? -1 : ti - NU, previousU[0], tScale2);
      addDifferenceTerm(ti + 1, k === 0 ? -1 : ti - NU + 1, previousU[1], dScale2);
    }

    function addDifferenceTerm(i, j, fixedPrevious, scale2) {
      const diff = U[i] - (j < 0 ? fixedPrevious : U[j]);
      const c = 2 * config.wSmooth / scale2;
      g[i] += c * diff;
      H[i * m + i] += c;
      if (j >= 0) {
        g[j] -= c * diff;
        H[j * m + j] += c;
        const hi = Math.max(i, j);
        const lo = Math.min(i, j);
        H[hi * m + lo] -= c;
      }
    }

    for (let i = 0; i < m; i++) {
      H[i * m + i] += 1e-7;
      for (let j = 0; j < i; j++) H[j * m + i] = H[i * m + j];
    }
    return { H, g };
  }

  function solveBoxQP(H, g, lower, upper, iterations, linearConstraints) {
    const n = g.length;
    const d = new Float64Array(n);
    let residual = Infinity;
    let maxLinearViolation = 0;
    const constraints = linearConstraints || [];
    const multipliers = new Float64Array(constraints.length);
    let rho = 400;
    const outerIterations = constraints.length ? 3 : 1;
    const sweepsPerOuter = constraints.length ? Math.max(8, Math.ceil(iterations / 2)) : iterations;

    for (let outer = 0; outer < outerIterations; outer++) {
      let workingH = H;
      let workingG = g;
      if (constraints.length) {
        workingH = new Float64Array(H);
        workingG = new Float64Array(g);
        for (let q = 0; q < constraints.length; q++) {
          const constraint = constraints[q];
          let value = -constraint.b;
          for (let i = 0; i < n; i++) value += constraint.a[i] * d[i];
          if (value <= 0 && multipliers[q] === 0) continue;
          for (let i = 0; i < n; i++) {
            workingG[i] += (multipliers[q] - rho * constraint.b) * constraint.a[i];
            const row = i * n;
            for (let j = 0; j < n; j++) workingH[row + j] += rho * constraint.a[i] * constraint.a[j];
          }
        }
      }

      for (let sweep = 0; sweep < sweepsPerOuter; sweep++) {
        residual = 0;
        for (let i = 0; i < n; i++) {
          let grad = workingG[i];
          const row = i * n;
          for (let j = 0; j < n; j++) grad += workingH[row + j] * d[j];
          const old = d[i];
          const diagonal = Math.max(workingH[row + i], 1e-8);
          d[i] = clamp(old - grad / diagonal, lower[i], upper[i]);
          residual = Math.max(residual, Math.abs(d[i] - old));
        }
        if (residual < 1e-5) break;
      }

      if (constraints.length) {
        for (let q = 0; q < constraints.length; q++) {
          let value = -constraints[q].b;
          for (let i = 0; i < n; i++) value += constraints[q].a[i] * d[i];
          multipliers[q] = Math.max(0, multipliers[q] + rho * value);
        }
        rho *= 8;
      }
    }

    // Alternating weighted projections enforce the linearized landing-cone
    // half-spaces while retaining every input box bound.
    for (let pass = 0; pass < 10 && constraints.length; pass++) {
      maxLinearViolation = 0;
      for (const constraint of constraints) {
        const direction = new Float64Array(n);
        for (let i = 0; i < n; i++) direction[i] = constraint.a[i] / Math.max(H[i * n + i], 1e-8);
        const valueAt = (lambda, apply) => {
          let value = -constraint.b;
          for (let i = 0; i < n; i++) {
            const candidate = clamp(d[i] + lambda * direction[i], lower[i], upper[i]);
            value += constraint.a[i] * candidate;
            if (apply) d[i] = candidate;
          }
          return value;
        };
        const violation = valueAt(0, false);
        maxLinearViolation = Math.max(maxLinearViolation, violation);
        if (violation <= 1e-7) continue;
        let lo = -1;
        for (let k = 0; k < 60 && valueAt(lo, false) > 0; k++) lo *= 2;
        let hi = 0;
        for (let k = 0; k < 55; k++) {
          const mid = 0.5 * (lo + hi);
          if (valueAt(mid, false) <= 0) lo = mid;
          else hi = mid;
        }
        valueAt(0.5 * (lo + hi), true);
      }
    }
    maxLinearViolation = 0;
    for (const constraint of constraints) {
      let value = -constraint.b;
      for (let i = 0; i < n; i++) value += constraint.a[i] * d[i];
      maxLinearViolation = Math.max(maxLinearViolation, value);
    }
    return { step: d, residual, linearViolation: Math.max(0, maxLinearViolation) };
  }

  function makeInitialGuess(x, config) {
    const N = config.horizon;
    const U = new Float64Array(N * NU);
    const hover = clamp(config.mass * G / Math.max(Math.cos(x[4]), 0.72), 0, config.maxThrust);
    for (let k = 0; k < N; k++) {
      const fade = Math.exp(-3 * k / Math.max(N, 1));
      U[k * NU] = hover;
      U[k * NU + 1] = clamp(0.75 * x[4] * fade, -config.maxGimbal, config.maxGimbal);
    }
    return U;
  }

  function shiftControls(U) {
    const shifted = new Float64Array(U.length);
    for (let i = 0; i < U.length - NU; i++) shifted[i] = U[i + NU];
    shifted[U.length - NU] = U[U.length - NU];
    shifted[U.length - 1] = U[U.length - 1];
    return shifted;
  }

  class SQPMPC {
    constructor(config) {
      this.config = { ...config };
      this.U = null;
      this.previousU = [config.mass * G, 0];
    }

    reset(x, config) {
      this.config = { ...config };
      this.U = makeInitialGuess(x, this.config);
      this.previousU = [clamp(config.mass * G, 0, config.maxThrust), 0];
    }

    solve(x, nextConfig) {
      const config = { ...nextConfig };
      const requiredLength = config.horizon * NU;
      if (!this.U || this.U.length !== requiredLength) this.reset(x, config);
      this.config = config;
      let U = this.U;
      let lastResidual = Infinity;
      let acceptedAlpha = 0;
      let iterationsDone = 0;
      let initialCost = Infinity;
      let finalCost = Infinity;
      let lastLinearViolation = 0;
      const lower = new Float64Array(U.length);
      const upper = new Float64Array(U.length);

      for (let iter = 0; iter < config.sqpIterations; iter++) {
        const data = rollout(x, U, config, true);
        const cost = evaluateCost(data.states, U, config, this.previousU);
        if (iter === 0) initialCost = cost;
        const qp = buildQP(data.states, data.sensitivities, U, config, this.previousU);
        const linearConstraints = [];
        if (config.terminalCone) {
          const terminalSensitivity = data.sensitivities[config.horizon];
          const terminalState = data.states[config.horizon];
          const tanCone = Math.tan(config.coneHalfAngle);
          for (const sign of [1, -1]) {
            const a = new Float64Array(U.length);
            for (let j = 0; j < U.length; j++) {
              a[j] = sign * terminalSensitivity[2 * U.length + j]
                + tanCone * terminalSensitivity[3 * U.length + j];
            }
            const value = sign * terminalState[2] + tanCone * terminalState[3];
            linearConstraints.push({ a, b: -value });
          }
        }
        for (let k = 0; k < config.horizon; k++) {
          lower[k * NU] = -U[k * NU];
          upper[k * NU] = config.maxThrust - U[k * NU];
          lower[k * NU + 1] = -config.maxGimbal - U[k * NU + 1];
          upper[k * NU + 1] = config.maxGimbal - U[k * NU + 1];
        }
        const solution = solveBoxQP(qp.H, qp.g, lower, upper, config.qpIterations, linearConstraints);
        lastResidual = solution.residual;
        lastLinearViolation = solution.linearViolation;
        let bestCost = cost;
        const coneTangent = Math.tan(config.coneHalfAngle);
        const initialConeViolation = Math.max(0, Math.abs(data.states[config.horizon][2]) + coneTangent * data.states[config.horizon][3]);
        let bestMerit = cost + (config.terminalCone ? 10000 * initialConeViolation ** 2 : 0);
        let bestU = U;
        acceptedAlpha = 0;
        for (const alpha of [1, 0.5, 0.25, 0.125, 0.0625]) {
          const candidate = new Float64Array(U.length);
          for (let i = 0; i < U.length; i++) candidate[i] = U[i] + alpha * solution.step[i];
          const candidateStates = rollout(x, candidate, config, false).states;
          const candidateCost = evaluateCost(candidateStates, candidate, config, this.previousU);
          const candidateTerminal = candidateStates[config.horizon];
          const candidateConeViolation = Math.max(0, Math.abs(candidateTerminal[2]) + coneTangent * candidateTerminal[3]);
          const candidateMerit = candidateCost + (config.terminalCone ? 10000 * candidateConeViolation ** 2 : 0);
          if (candidateMerit < bestMerit - 1e-8) {
            bestCost = candidateCost;
            bestMerit = candidateMerit;
            bestU = candidate;
            acceptedAlpha = alpha;
            break;
          }
        }
        U = bestU;
        finalCost = bestCost;
        iterationsDone = iter + 1;
        let maxStep = 0;
        for (let i = 0; i < solution.step.length; i++) maxStep = Math.max(maxStep, Math.abs(solution.step[i]) / (i % 2 ? config.maxGimbal : config.maxThrust));
        if (acceptedAlpha === 0 || maxStep * acceptedAlpha < 2e-4) break;
      }

      const prediction = rollout(x, U, config, false).states;
      let active = 0;
      for (let k = 0; k < config.horizon; k++) {
        if (U[k * NU] < 1e-3 || U[k * NU] > config.maxThrust - 1e-3) active++;
        if (Math.abs(U[k * NU + 1]) > config.maxGimbal - 1e-4) active++;
      }
      const terminal = prediction[prediction.length - 1];
      const terminalError = Math.hypot(terminal[0] - config.targetX, terminal[1], terminal[2], terminal[3], 2 * terminal[4]);
      const terminalSpeed = Math.hypot(terminal[2], terminal[3]);
      const terminalConeAngle = terminalSpeed < 1e-8 ? 0 : Math.atan2(Math.abs(terminal[2]), -terminal[3]) * 180 / Math.PI;
      const terminalConeViolation = Math.max(0, Math.abs(terminal[2]) + Math.tan(config.coneHalfAngle) * terminal[3]);
      this.U = U;
      return {
        u: [U[0], U[1]],
        controls: U,
        prediction,
        diagnostics: {
          initialCost, finalCost, residual: lastResidual, alpha: acceptedAlpha, iterations: iterationsDone,
          active, terminalError, terminalVx: terminal[2], terminalVz: terminal[3], terminalConeAngle,
          terminalConeViolation, linearViolation: lastLinearViolation
        }
      };
    }

    advance(appliedU) {
      this.previousU = appliedU.slice();
      this.U = shiftControls(this.U);
    }
  }

  return { NX, NU, G, clamp, discreteDynamics, rollout, evaluateCost, SQPMPC };
});
