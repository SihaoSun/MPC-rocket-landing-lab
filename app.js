(function () {
  "use strict";

  const { G, clamp, discreteDynamics, SQPMPC } = window.RocketMPC;
  const $ = (id) => document.getElementById(id);
  const deg = (rad) => rad * 180 / Math.PI;
  const rad = (degrees) => degrees * Math.PI / 180;
  const SIMULATION_DT = 0.01;
  const REPLAY_RATE = 0.1;
  const state = {
    x: null,
    initial: null,
    path: [],
    inputs: [],
    mpcFrames: [],
    prediction: [],
    predictedControls: null,
    predictionTime: 0,
    running: false,
    finished: false,
    time: 0,
    accumulator: 0,
    mpcAccumulator: 0,
    replayMode: false,
    replayPlaying: false,
    replayTime: 0,
    replayIndex: 0,
    lastFrame: performance.now(),
    solveMs: 0,
    diagnostics: null,
    lastControl: [0, 0],
    seed: 0
  };

  const defaultValues = {
    mass: 620, radius: 0.85, bodyHeight: 4, lever: 2.2, drag: 12, maxThrust: 18, maxGimbal: 18,
    horizon: 36, dt: 0.12, sqpIterations: 4, qpIterations: 42, terminalScale: 24, targetX: 0,
    wPos: 18, wVel: 14, wAngle: 24, wOmega: 6, wFuel: 0.18, wSmooth: 1.2
  };
  let controller;
  let deferredSolve;

  function readConfig() {
    const mass = clamp(+$('mass').value || defaultValues.mass, 150, 3000);
    const radius = clamp(+$('radius').value || defaultValues.radius, 0.2, 3);
    const bodyHeight = clamp(+$('bodyHeight').value || defaultValues.bodyHeight, 1, 12);
    const inertia = mass * (3 * radius * radius + bodyHeight * bodyHeight) / 12;
    $('inertia').value = inertia.toFixed(1);
    return {
      mass,
      radius,
      bodyHeight,
      inertia,
      lever: clamp(+$('lever').value || defaultValues.lever, 0.2, 5),
      drag: clamp(+$('drag').value || 0, 0, 100),
      angularDamping: 85,
      maxThrust: clamp(+$('maxThrust').value || defaultValues.maxThrust, 5, 80) * 1000,
      maxGimbal: rad(clamp(+$('maxGimbal').value || defaultValues.maxGimbal, 2, 35)),
      horizon: Math.round(clamp(+$('horizon').value || defaultValues.horizon, 1, 70)),
      dt: Math.round(clamp(+$('dt').value || defaultValues.dt, 0.05, 0.3) / SIMULATION_DT) * SIMULATION_DT,
      sqpIterations: Math.round(clamp(+$('sqpIterations').value || defaultValues.sqpIterations, 1, 8)),
      qpIterations: Math.round(clamp(+$('qpIterations').value || defaultValues.qpIterations, 8, 100)),
      terminalScale: clamp(+$('terminalScale').value || defaultValues.terminalScale, 1, 80),
      targetX: clamp(+$('targetX').value || 0, -8, 8),
      wPos: +$('wPos').value,
      wVel: +$('wVel').value,
      wAngle: +$('wAngle').value,
      wOmega: +$('wOmega').value,
      wFuel: +$('wFuel').value,
      wSmooth: +$('wSmooth').value,
      terminalCone: $('terminalCone').checked,
      coneHalfAngle: rad(30)
    };
  }

  function randomInitial() {
    const target = readConfig().targetX;
    const side = Math.random() < 0.5 ? -1 : 1;
    return [
      target + side * (2.8 + Math.random() * 4.2),
      7.3 + Math.random() * 3.2,
      0,
      0,
      rad(-18 + Math.random() * 36),
      0
    ];
  }

  function resetSimulation(useNewRandom) {
    state.running = false;
    state.finished = false;
    state.time = 0;
    state.accumulator = 0;
    state.mpcAccumulator = 0;
    state.replayMode = false;
    state.replayPlaying = false;
    state.replayTime = 0;
    state.replayIndex = 0;
    state.inputs = [];
    state.mpcFrames = [];
    state.lastControl = [0, 0];
    $('landingResult').hidden = true;
    if (useNewRandom || !state.initial) state.initial = randomInitial();
    state.x = state.initial.slice();
    state.path = [{ t: 0, x: state.x.slice() }];
    const config = readConfig();
    controller = new SQPMPC(config);
    controller.reset(state.x, config);
    solveMPC();
    setRunButton();
    renderAll();
  }

  function solveMPC() {
    if (state.finished) return;
    const config = readConfig();
    const start = performance.now();
    const result = controller.solve(state.x, config);
    state.solveMs = performance.now() - start;
    state.prediction = result.prediction;
    state.predictedControls = result.controls;
    state.predictionTime = state.time;
    state.lastControl = result.u;
    state.diagnostics = result.diagnostics;
    const frame = {
      t: state.time,
      prediction: result.prediction.map(x => x.slice()),
      controls: new Float64Array(result.controls),
      u: result.u.slice(),
      diagnostics: { ...result.diagnostics },
      solveMs: state.solveMs
    };
    const lastFrame = state.mpcFrames[state.mpcFrames.length - 1];
    if (lastFrame && Math.abs(lastFrame.t - state.time) < 1e-9) state.mpcFrames[state.mpcFrames.length - 1] = frame;
    else state.mpcFrames.push(frame);
  }

  function simulationStep() {
    if (state.finished) return;
    const config = readConfig();
    const applied = state.lastControl.slice();
    const plantConfig = { ...config, dt: SIMULATION_DT };
    const next = discreteDynamics(state.x, applied, plantConfig, false).next;
    state.time += SIMULATION_DT;
    state.mpcAccumulator += SIMULATION_DT;
    state.inputs.push({ t: state.time, u: applied });
    state.x = next;
    state.path.push({ t: state.time, x: next.slice() });

    if (state.mpcAccumulator + 1e-9 >= config.dt) {
      state.mpcAccumulator = Math.max(0, state.mpcAccumulator - config.dt);
      controller.advance(applied);
      solveMPC();
    }

    const nearGroundSettled = state.x[1] < 0.08
      && Math.abs(state.x[0] - config.targetX) < 0.35
      && Math.hypot(state.x[2], state.x[3]) < 0.18
      && Math.abs(deg(state.x[4])) < 4;
    if (nearGroundSettled || state.x[1] <= 0) finishLanding();
    else if (state.time >= 25 || Math.abs(state.x[0] - config.targetX) > 30 || state.x[1] > 30) finishLanding(true);
  }

  function finishLanding(timedOut) {
    state.running = false;
    state.finished = true;
    state.x[1] = Math.max(0, state.x[1]);
    state.path[state.path.length - 1].x = state.x.slice();
    const config = readConfig();
    const positionOK = Math.abs(state.x[0] - config.targetX) < 0.55;
    const velocity = Math.hypot(state.x[2], state.x[3]);
    const speedOK = velocity < 1.0;
    const angleOK = Math.abs(deg(state.x[4])) < 7;
    const success = !timedOut && positionOK && speedOK && angleOK;
    const result = $('landingResult');
    result.hidden = false;
    result.classList.toggle('fail', !success);
    result.innerHTML = success
      ? `✓ Soft landing complete<small>Position error ${Math.abs(state.x[0] - config.targetX).toFixed(2)} m · Touchdown speed ${velocity.toFixed(2)} m/s · Tilt ${Math.abs(deg(state.x[4])).toFixed(1)}°</small>`
      : `${timedOut ? '○ Landing timeout' : '× Landing conditions not met'}<small>Position error ${Math.abs(state.x[0] - config.targetX).toFixed(2)} m · Touchdown speed ${velocity.toFixed(2)} m/s · Tilt ${Math.abs(deg(state.x[4])).toFixed(1)}°</small>`;
    setRunButton();
  }

  function startReplay() {
    if (state.path.length < 2 || !state.finished) return;
    state.running = false;
    state.replayMode = true;
    state.replayPlaying = true;
    state.replayTime = 0;
    state.replayIndex = 0;
    state.time = 0;
    state.x = state.path[0].x.slice();
    applyReplayFrame(0);
    $('landingResult').hidden = true;
    state.lastFrame = performance.now();
    setRunButton();
  }

  function updateReplay(elapsed) {
    if (!state.replayPlaying) return;
    const duration = state.path[state.path.length - 1].t;
    state.replayTime = Math.min(duration, state.replayTime + elapsed * REPLAY_RATE);
    state.time = state.replayTime;
    while (state.replayIndex + 1 < state.path.length && state.path[state.replayIndex + 1].t <= state.replayTime + 1e-9) {
      state.replayIndex++;
    }
    state.x = state.path[state.replayIndex].x.slice();
    applyReplayFrame(state.replayTime);
    if (state.replayTime >= duration - 1e-9) {
      state.replayMode = false;
      state.replayPlaying = false;
      state.time = duration;
      state.x = state.path[state.path.length - 1].x.slice();
      $('landingResult').hidden = false;
      setRunButton();
    }
  }

  function applyReplayFrame(time) {
    let index = 0;
    while (index + 1 < state.mpcFrames.length && state.mpcFrames[index + 1].t <= time + 1e-9) index++;
    const frame = state.mpcFrames[index];
    if (!frame) return;
    state.prediction = frame.prediction;
    state.predictedControls = frame.controls;
    state.predictionTime = frame.t;
    state.lastControl = frame.u;
    state.diagnostics = frame.diagnostics;
    state.solveMs = frame.solveMs;
  }

  function setRunButton() {
    const button = $('runBtn');
    button.classList.toggle('running', state.running);
    button.textContent = state.finished ? '↺ Run again' : state.running ? 'Ⅱ Pause' : '▶ Start';
    button.disabled = state.replayMode;
    const replay = $('replayBtn');
    replay.disabled = state.path.length < 2 || !state.finished;
    replay.textContent = state.replayPlaying ? 'Ⅱ Pause replay' : state.replayMode ? '▶ Resume replay' : '↻ Replay 0.1×';
    $('solverBadge').classList.toggle('running', state.running || state.replayPlaying);
  }

  function tick(now) {
    const elapsed = Math.max(0, Math.min((now - state.lastFrame) / 1000, 0.1));
    state.lastFrame = now;
    if (state.running) {
      state.accumulator += elapsed;
      let steps = 0;
      while (state.accumulator + 1e-9 >= SIMULATION_DT && steps < 20 && !state.finished) {
        state.accumulator -= SIMULATION_DT;
        simulationStep();
        steps++;
      }
    }
    if (state.replayMode) updateReplay(elapsed);
    renderAll();
    requestAnimationFrame(tick);
  }

  function setupCanvas(canvas) {
    const rect = canvas.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.max(1, Math.round(rect.width));
    const height = Math.max(1, Math.round(rect.height));
    if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
      canvas.width = Math.round(width * dpr);
      canvas.height = Math.round(height * dpr);
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    return { ctx, width, height };
  }

  function drawFlight() {
    const { ctx, width, height } = setupCanvas($('flightCanvas'));
    const config = readConfig();
    const visiblePath = state.replayMode ? state.path.slice(0, state.replayIndex + 1) : state.path;
    const allStates = visiblePath.map(p => p.x).concat(state.prediction || []);
    const maxAbsX = Math.max(6, Math.abs(config.targetX) + 4, ...allStates.map(x => Math.abs(x[0]) + 1.5));
    const maxZ = Math.max(11, ...allStates.map(x => x[1] + 2));
    const margin = { left: 46, right: 24, top: 25, bottom: 44 };
    const plotW = width - margin.left - margin.right;
    const plotH = height - margin.top - margin.bottom;
    const sx = (v) => margin.left + (v + maxAbsX) / (2 * maxAbsX) * plotW;
    const sy = (v) => margin.top + (maxZ - v) / maxZ * plotH;

    ctx.fillStyle = '#edf2ef';
    ctx.fillRect(0, 0, width, height);
    ctx.lineWidth = 1;
    ctx.font = '9px ui-monospace, monospace';
    ctx.fillStyle = '#8b9997';
    ctx.strokeStyle = 'rgba(23,37,42,.09)';
    const zStep = maxZ > 16 ? 4 : 2;
    for (let z = 0; z <= maxZ; z += zStep) {
      ctx.beginPath(); ctx.moveTo(margin.left, sy(z)); ctx.lineTo(width - margin.right, sy(z)); ctx.stroke();
      ctx.fillText(`${z} m`, 10, sy(z) + 3);
    }
    for (let x = -Math.floor(maxAbsX / 2) * 2; x <= maxAbsX; x += 2) {
      ctx.beginPath(); ctx.moveTo(sx(x), margin.top); ctx.lineTo(sx(x), sy(0)); ctx.stroke();
      ctx.fillText(`${x}`, sx(x) - 5, height - 19);
    }

    const groundY = sy(0);
    ctx.fillStyle = '#d6d0c3';
    ctx.fillRect(0, groundY, width, height - groundY);
    ctx.strokeStyle = '#26383b'; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(0, groundY); ctx.lineTo(width, groundY); ctx.stroke();
    ctx.strokeStyle = 'rgba(23,37,42,.13)'; ctx.lineWidth = 1;
    for (let x = -10; x < width + 10; x += 18) {
      ctx.beginPath(); ctx.moveTo(x, groundY + 16); ctx.lineTo(x + 14, groundY); ctx.stroke();
    }
    drawLandingPad(ctx, sx(config.targetX), groundY);

    if (visiblePath.length > 1) {
      ctx.strokeStyle = '#f0642f'; ctx.lineWidth = 2.5;
      ctx.beginPath();
      visiblePath.forEach((point, i) => i ? ctx.lineTo(sx(point.x[0]), sy(point.x[1])) : ctx.moveTo(sx(point.x[0]), sy(point.x[1])));
      ctx.stroke();
    }

    if (state.prediction && state.prediction.length > 1) {
      ctx.strokeStyle = '#0f7778'; ctx.lineWidth = 2; ctx.setLineDash([7, 6]);
      ctx.beginPath();
      state.prediction.forEach((x, i) => i ? ctx.lineTo(sx(x[0]), sy(x[1])) : ctx.moveTo(sx(x[0]), sy(x[1])));
      ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = '#0f7778';
      for (let i = 1; i < state.prediction.length; i++) {
        ctx.beginPath(); ctx.arc(sx(state.prediction[i][0]), sy(state.prediction[i][1]), 2.4, 0, Math.PI * 2); ctx.fill();
      }
    }

    drawRocket(ctx, sx(state.x[0]), sy(Math.max(0, state.x[1])), state.x[4], state.lastControl, config);
  }

  function drawLandingPad(ctx, x, y) {
    ctx.save();
    ctx.fillStyle = '#f6bd45';
    ctx.fillRect(x - 34, y - 5, 68, 6);
    ctx.fillStyle = '#17252a';
    ctx.fillRect(x - 22, y - 9, 44, 4);
    ctx.strokeStyle = '#f6bd45'; ctx.lineWidth = 1.2;
    ctx.beginPath(); ctx.arc(x, y - 5, 13, Math.PI, 0); ctx.stroke();
    ctx.fillStyle = '#17252a'; ctx.font = '800 8px ui-monospace, monospace'; ctx.textAlign = 'center';
    ctx.fillText('LANDING ZONE', x, y + 28);
    ctx.restore();
  }

  function drawRocket(ctx, x, y, theta, u, config) {
    const thrustRatio = clamp(u[0] / config.maxThrust, 0, 1);
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(theta);
    if ((!state.finished || state.replayMode) && thrustRatio > .03) {
      const flame = 10 + 24 * thrustRatio;
      ctx.fillStyle = 'rgba(246,189,69,.72)';
      ctx.beginPath(); ctx.moveTo(-6, 14); ctx.lineTo(0, 14 + flame); ctx.lineTo(6, 14); ctx.closePath(); ctx.fill();
      ctx.fillStyle = '#f0642f';
      ctx.beginPath(); ctx.moveTo(-3, 14); ctx.lineTo(0, 12 + flame * .68); ctx.lineTo(3, 14); ctx.closePath(); ctx.fill();
    }
    ctx.fillStyle = '#fffdf8'; ctx.strokeStyle = '#17252a'; ctx.lineWidth = 1.7;
    ctx.beginPath(); ctx.moveTo(0, -24); ctx.quadraticCurveTo(11, -13, 10, 12); ctx.lineTo(-10, 12); ctx.quadraticCurveTo(-11, -13, 0, -24); ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.fillStyle = '#f0642f';
    ctx.beginPath(); ctx.moveTo(0, -24); ctx.quadraticCurveTo(7, -17, 8, -10); ctx.lineTo(-8, -10); ctx.quadraticCurveTo(-7, -17, 0, -24); ctx.fill();
    ctx.fillStyle = '#0f7778'; ctx.beginPath(); ctx.arc(0, -5, 3.5, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#17252a';
    ctx.beginPath(); ctx.moveTo(-10, 5); ctx.lineTo(-17, 15); ctx.lineTo(-9, 12); ctx.fill();
    ctx.beginPath(); ctx.moveTo(10, 5); ctx.lineTo(17, 15); ctx.lineTo(9, 12); ctx.fill();
    ctx.restore();
  }

  function drawInputChart(canvas, kind) {
    const { ctx, width, height } = setupCanvas(canvas);
    const config = readConfig();
    const margin = { left: 43, right: 14, top: 14, bottom: 27 };
    const w = width - margin.left - margin.right;
    const h = height - margin.top - margin.bottom;
    const horizonSeconds = config.horizon * config.dt;
    const historySeconds = 7;
    const minTime = Math.max(0, state.time - historySeconds);
    const predictionStart = state.predictionTime ?? state.time;
    const maxTime = Math.max(historySeconds, state.time + horizonSeconds, predictionStart + horizonSeconds);
    const xMap = t => margin.left + (t - minTime) / Math.max(maxTime - minTime, .1) * w;
    const isThrust = kind === 0;
    const limit = isThrust ? config.maxThrust / 1000 : deg(config.maxGimbal);
    const yMin = isThrust ? 0 : -limit * 1.28;
    const yMax = isThrust ? limit * 1.17 : limit * 1.28;
    const yMap = v => margin.top + (yMax - v) / (yMax - yMin) * h;
    const toUnit = v => isThrust ? v / 1000 : deg(v);

    ctx.fillStyle = '#fffdf8'; ctx.fillRect(0, 0, width, height);
    ctx.fillStyle = 'rgba(15,119,120,.065)';
    ctx.fillRect(margin.left, yMap(limit), w, yMap(isThrust ? 0 : -limit) - yMap(limit));
    ctx.strokeStyle = 'rgba(23,37,42,.10)'; ctx.lineWidth = 1;
    ctx.fillStyle = '#839093'; ctx.font = '8px ui-monospace, monospace';
    ctx.textAlign = 'right';
    const ticks = isThrust ? [0, limit / 2, limit] : [-limit, 0, limit];
    ticks.forEach(tick => {
      const y = yMap(tick); ctx.beginPath(); ctx.moveTo(margin.left, y); ctx.lineTo(width - margin.right, y); ctx.stroke();
      ctx.fillText(`${tick.toFixed(isThrust ? 0 : 0)}${isThrust ? 'kN' : '°'}`, margin.left - 7, y + 3);
    });
    ctx.strokeStyle = '#d6a027'; ctx.setLineDash([4, 4]);
    [limit, ...(isThrust ? [] : [-limit])].forEach(bound => {
      ctx.beginPath(); ctx.moveTo(margin.left, yMap(bound)); ctx.lineTo(width - margin.right, yMap(bound)); ctx.stroke();
    });
    ctx.setLineDash([]);
    const nowX = xMap(state.time);
    ctx.strokeStyle = 'rgba(23,37,42,.25)';
    ctx.beginPath(); ctx.moveTo(nowX, margin.top); ctx.lineTo(nowX, height - margin.bottom); ctx.stroke();
    ctx.textAlign = 'center'; ctx.fillStyle = '#839093';
    ctx.fillText('now', nowX, height - 10);
    ctx.fillText(`${maxTime.toFixed(1)}s`, width - margin.right - 10, height - 10);

    const history = state.inputs.filter(p => p.t >= minTime && p.t <= state.time + 1e-9);
    if (history.length) {
      ctx.strokeStyle = '#f0642f'; ctx.lineWidth = 2.2; ctx.beginPath();
      history.forEach((p, i) => {
        const px = xMap(p.t), py = yMap(toUnit(p.u[kind]));
        i ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
      });
      ctx.stroke();
    }
    if (state.predictedControls) {
      const predictedSteps = Math.min(config.horizon, state.predictedControls.length / 2);
      ctx.strokeStyle = '#0f7778'; ctx.lineWidth = 1.8; ctx.setLineDash([5, 4]); ctx.beginPath();
      for (let k = 0; k < predictedSteps; k++) {
        const px = xMap(predictionStart + k * config.dt);
        const py = yMap(toUnit(state.predictedControls[k * 2 + kind]));
        k ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
      }
      ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = '#0f7778';
      for (let k = 0; k < predictedSteps; k++) {
        ctx.beginPath();
        ctx.arc(xMap(predictionStart + k * config.dt), yMap(toUnit(state.predictedControls[k * 2 + kind])), 2.6, 0, Math.PI * 2);
        ctx.fill();
      }
    }
  }

  function updateTelemetry() {
    const config = readConfig();
    $('altitudeValue').textContent = state.x[1].toFixed(2);
    $('xErrorValue').textContent = (state.x[0] - config.targetX).toFixed(2);
    $('vzValue').textContent = state.x[3].toFixed(2);
    $('angleValue').textContent = deg(state.x[4]).toFixed(1);
    $('costValue').textContent = state.diagnostics ? formatNumber(state.diagnostics.finalCost) : '—';
    $('solveValue').textContent = state.solveMs.toFixed(1);
    $('thrustNow').textContent = `${(state.lastControl[0] / 1000).toFixed(1)} kN`;
    $('gimbalNow').textContent = `${deg(state.lastControl[1]).toFixed(1)}°`;
    $('timeChip').textContent = `T + ${state.time.toFixed(2)} s`;
    if (state.diagnostics) {
      const d = state.diagnostics;
      $('iterationChip').textContent = `SQP ${d.iterations} / ${config.sqpIterations}`;
      $('qpResidual').textContent = d.residual.toExponential(2);
      $('lineAlpha').textContent = d.alpha.toFixed(3);
      $('terminalError').textContent = `${d.terminalError.toFixed(3)} norm`;
      $('activeConstraints').textContent = `${d.active} / ${config.horizon * 2}`;
      $('landingConeMetric').textContent = `${d.terminalConeAngle.toFixed(1)}° / ≤ 30°`;
      $('landingConeState').textContent = config.terminalCone ? 'ON · inequalities' : 'OFF · cost only';
      const improvement = d.initialCost > 0 ? clamp(1 - d.finalCost / d.initialCost, 0, 1) : 0;
      $('costTrackFill').style.width = `${Math.max(3, improvement * 100)}%`;
      $('convergencePill').textContent = d.alpha > 0 ? 'STEP ACCEPTED' : 'STATIONARY';
      $('convergencePill').classList.toggle('good', d.alpha > 0 || d.residual < 1e-4);
      $('solverMessage').textContent = `Nonlinear cost ${formatNumber(d.initialCost)} → ${formatNumber(d.finalCost)}; terminal velocity angle ${d.terminalConeAngle.toFixed(1)}° ${config.terminalCone ? '(30° cone enabled)' : '(cost tracking only)'}.`;
    }
    const warning = state.solveMs > config.dt * 1000;
    $('solverBadge').classList.toggle('warning', warning);
    $('solverBadge').innerHTML = `<i></i>${warning ? 'SOLVER SLOW' : state.replayMode ? 'REPLAY 0.1×' : state.running ? 'MPC CLOSED LOOP' : 'SOLVER READY'}`;
  }

  function formatNumber(value) {
    if (!Number.isFinite(value)) return '—';
    if (Math.abs(value) > 9999) return value.toExponential(2);
    return value.toFixed(value < 10 ? 2 : 1);
  }

  function renderAll() {
    if (!state.x) return;
    drawFlight();
    drawInputChart($('thrustCanvas'), 0);
    drawInputChart($('gimbalCanvas'), 1);
    updateTelemetry();
  }

  function bindUI() {
    $('runBtn').addEventListener('click', () => {
      if (state.finished) return resetSimulation(true);
      state.running = !state.running;
      state.lastFrame = performance.now();
      setRunButton();
    });
    $('replayBtn').addEventListener('click', () => {
      if (!state.replayMode) startReplay();
      else {
        state.replayPlaying = !state.replayPlaying;
        state.lastFrame = performance.now();
        setRunButton();
      }
    });
    $('resetBtn').addEventListener('click', () => resetSimulation(false));
    $('randomizeBtn').addEventListener('click', () => resetSimulation(true));
    $('defaultsBtn').addEventListener('click', () => {
      Object.entries(defaultValues).forEach(([key, value]) => { if ($(key)) $(key).value = value; });
      $('terminalCone').checked = true;
      updateSliderOutputs();
      resetSimulation(false);
    });
    document.querySelectorAll('.section-title').forEach(button => {
      button.addEventListener('click', () => {
        const section = button.closest('.control-section');
        section.classList.toggle('collapsed');
        button.setAttribute('aria-expanded', String(!section.classList.contains('collapsed')));
      });
    });
    document.querySelectorAll('input').forEach(input => {
      input.addEventListener('input', () => {
        updateSliderOutputs();
        window.clearTimeout(deferredSolve);
        deferredSolve = window.setTimeout(() => {
          if (!state.running && !state.finished) {
            const config = readConfig();
            if (!controller || controller.U.length !== config.horizon * 2) {
              controller = new SQPMPC(config); controller.reset(state.x, config);
            }
            solveMPC(); renderAll();
          }
        }, 180);
      });
    });
    window.addEventListener('resize', renderAll);
  }

  function updateSliderOutputs() {
    ['wPos', 'wVel', 'wAngle', 'wOmega', 'wFuel', 'wSmooth'].forEach(id => { $(`${id}Out`).textContent = $(id).value; });
  }

  bindUI();
  updateSliderOutputs();
  resetSimulation(true);
  const launchOptions = new URLSearchParams(window.location.search);
  if (launchOptions.has('horizon')) {
    $('horizon').value = String(Math.round(clamp(+launchOptions.get('horizon') || 1, 1, 70)));
    resetSimulation(false);
  }
  if (launchOptions.get('constraint') === 'off') {
    $('terminalCone').checked = false;
    solveMPC();
    renderAll();
  }
  if (launchOptions.get('verify') === '1') {
    state.initial = [4.5, 8, 0, 0, rad(12), 0];
    resetSimulation(false);
    for (let k = 0; k < 4000 && !state.finished; k++) simulationStep();
    renderAll();
    document.body.dataset.verification = state.finished ? $('landingResult').textContent : 'incomplete';
    if (launchOptions.get('replay') === '1' && state.finished) {
      startReplay();
      updateReplay(10);
      renderAll();
      document.body.dataset.replay = `time=${state.replayTime.toFixed(2)}, index=${state.replayIndex}`;
    }
  } else if (launchOptions.get('autostart') === '1') {
    state.running = true;
    setRunButton();
  }
  requestAnimationFrame(tick);
})();
