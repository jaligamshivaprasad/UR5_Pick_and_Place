const $ = id => document.getElementById(id);
let status = null;
const samples = [];
const positionSamples = [];
const windowSeconds = 30;
const colors = ['#20d982', '#15bdf2', '#ffb942', '#ff5865', '#b58cff', '#f3f6ff'];
const positionColors = ['#20d982', '#15bdf2', '#ffb942'];
const labels = {
  clock: 'Simulation clock',
  joint_states: 'Joint-state feedback',
  arm_controller: 'Arm controller',
  gripper_controller: 'Gripper controller',
  moveit: 'MoveIt',
  planning_scene: 'Planning scene',
  rviz: 'RViz',
  launch: 'Bringup process',
};

function render(value) {
  status = value;
  $('state').textContent = value.state;
  $('state').className = 'pill ' + value.state.toLowerCase();
  $('message').textContent = value.message;
  $('pid').textContent = value.launch_pid ? `Owned PID ${value.launch_pid}` : '';
  const checks = $('checks');
  checks.innerHTML = '';
  Object.entries(value.checks || {}).forEach(([key, item]) => {
    const row = document.createElement('div');
    row.className = 'check';
    const dot = document.createElement('span');
    dot.className = 'dot ' + (item.ok ? 'green' : value.state === 'Starting' ? 'amber' : 'red');
    const text = document.createElement('div');
    text.innerHTML = `<strong>${labels[key] || item.name}</strong><small>${item.detail}${item.age_s != null ? ' · ' + item.age_s.toFixed(1) + 's ago' : ''}</small>`;
    row.append(dot, text);
    checks.append(row);
  });
  $('logs').textContent = (value.logs || []).join('\n') || 'No dashboard logs yet.';
  $('logs').scrollTop = $('logs').scrollHeight;
  const busy = ['Starting', 'Stopping'].includes(value.state);
  $('start').disabled = busy || value.launch_owned;
  $('restart').disabled = busy;
  $('stop').disabled = !value.launch_owned;
  const ready = value.state === 'Ready';
  const trajectoryBusy = ['Starting', 'Running', 'Stopping'].includes(value.trajectory_state);
  $('trajectoryState').textContent = value.trajectory_state;
  $('trajectoryState').className = 'pill ' + (trajectoryBusy ? 'starting' : value.trajectory_state === 'Completed' ? 'ready' : value.trajectory_state === 'Failed' ? 'failed' : 'stopped');
  $('trajectoryDetails').textContent = value.trajectory_pid ? `Managed process ${value.trajectory_pid}` : value.trajectory_state === 'Completed' ? 'Trajectory process completed.' : value.trajectory_state === 'Failed' ? 'Trajectory process failed; see the ROS 2 console.' : 'Trajectory uses the existing MoveIt pick-and-place workflow.';
  $('planTrajectory').disabled = !ready || trajectoryBusy;
  $('executeTrajectory').disabled = !ready || trajectoryBusy;
  $('stopTrajectory').disabled = !trajectoryBusy;
  const recording = value.recording_state === 'Recording';
  $('recordingState').textContent = value.recording_state;
  $('recordingState').className = 'pill ' + (recording ? 'ready' : value.recording_state === 'Failed' ? 'failed' : 'stopped');
  $('recordingDetails').textContent = value.recording_path ? `${value.recording_path}${recording ? ' · recording' : ''}` : 'Bags are saved in experiment_bags/ and never overwrite an existing run.';
  $('startRecording').disabled = !ready || recording;
  $('stopRecording').disabled = !recording;
  updateTelemetry(value.telemetry);
}

function drawChart(canvasId, legendId, key, series, includeZero = false) {
  const canvas = $(canvasId);
  const rect = canvas.getBoundingClientRect();
  if (!rect.width) return;
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * ratio);
  canvas.height = Math.round(rect.height * ratio);
  const context = canvas.getContext('2d');
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  const width = rect.width;
  const height = rect.height;
  const pad = {left: 48, right: 10, top: 12, bottom: 26};
  const plotWidth = width - pad.left - pad.right;
  const plotHeight = height - pad.top - pad.bottom;
  const latest = samples.length ? samples[samples.length - 1].t : 0;
  const visible = samples.filter(sample => latest - sample.t <= windowSeconds);
  const traces = [];
  for (let joint = 0; joint < 6; joint += 1) {
    for (const item of series) {
      const values = visible.map(sample => sample[item.key] ? sample[item.key][joint] : null);
      traces.push({joint, label: item.label, dashed: item.dashed, values});
    }
  }
  const numbers = traces.flatMap(trace => trace.values.filter(Number.isFinite));
  context.clearRect(0, 0, width, height);
  if (!numbers.length) {
    context.fillStyle = '#8da9bd';
    context.font = '12px system-ui';
    context.fillText('Waiting for live arm feedback', pad.left + 8, pad.top + 20);
    $(legendId).innerHTML = '';
    return;
  }
  let minimum = Math.min(...numbers);
  let maximum = Math.max(...numbers);
  if (includeZero) {
    minimum = Math.min(minimum, 0);
    maximum = Math.max(maximum, 0);
  }
  if (maximum - minimum < 1e-8) {
    minimum -= 0.5;
    maximum += 0.5;
  } else {
    const margin = (maximum - minimum) * 0.08;
    minimum -= margin;
    maximum += margin;
  }
  const x = time => pad.left + plotWidth * (1 - Math.max(0, Math.min(windowSeconds, latest - time)) / windowSeconds);
  const y = number => pad.top + plotHeight * (maximum - number) / (maximum - minimum);
  context.font = '10px system-ui';
  context.lineWidth = 1;
  context.strokeStyle = '#1d4764';
  context.fillStyle = '#8da9bd';
  for (let tick = 0; tick <= 4; tick += 1) {
    const yPosition = pad.top + plotHeight * tick / 4;
    const value = maximum - (maximum - minimum) * tick / 4;
    context.beginPath();
    context.moveTo(pad.left, yPosition);
    context.lineTo(width - pad.right, yPosition);
    context.stroke();
    context.fillText(value.toFixed(2), 2, yPosition + 3);
    const seconds = windowSeconds * tick / 4;
    const xPosition = pad.left + plotWidth * tick / 4;
    context.beginPath();
    context.moveTo(xPosition, pad.top);
    context.lineTo(xPosition, pad.top + plotHeight);
    context.stroke();
    context.fillText(`-${(windowSeconds - seconds).toFixed(0)}s`, xPosition - 9, height - 7);
  }
  if (minimum < 0 && maximum > 0) {
    context.strokeStyle = '#58758a';
    context.beginPath();
    context.moveTo(pad.left, y(0));
    context.lineTo(width - pad.right, y(0));
    context.stroke();
  }
  for (const trace of traces) {
    context.strokeStyle = colors[trace.joint];
    context.lineWidth = trace.dashed ? 1 : 1.6;
    context.setLineDash(trace.dashed ? [5, 4] : []);
    context.beginPath();
    let drawing = false;
    visible.forEach((sample, index) => {
      const value = trace.values[index];
      if (!Number.isFinite(value)) {
        drawing = false;
      } else {
        const pointX = x(sample.t);
        const pointY = y(value);
        if (!drawing) context.moveTo(pointX, pointY);
        else context.lineTo(pointX, pointY);
        drawing = true;
      }
    });
    context.stroke();
  }
  context.setLineDash([]);
  const jointNames = status && status.telemetry ? status.telemetry.joint_names : [];
  $(legendId).innerHTML = jointNames.map((name, joint) => {
    const shortName = name.replace('_joint', '').replace(/_/g, ' ');
    const lineStyle = series.map(item => `<i class="${item.dashed ? 'dashed' : ''}" style="color:${colors[joint]};background:${item.dashed ? 'transparent' : colors[joint]}"></i>${item.label}`).join(' ');
    return `<span>${lineStyle} ${shortName}</span>`;
  }).join('');
}

function updateTelemetry(telemetry) {
  if (!telemetry) return;
  $('telemetryState').textContent = telemetry.ready ? 'Live' : 'Waiting';
  $('telemetryState').className = 'pill ' + (telemetry.ready ? 'ready' : 'stopped');
  $('telemetryMessage').textContent = telemetry.message || 'Waiting for fresh arm controller feedback.';
  $('rmsError').textContent = Number.isFinite(telemetry.rms_error_rad) ? `${telemetry.rms_error_rad.toFixed(4)} rad` : '—';
  $('maxError').textContent = Number.isFinite(telemetry.max_abs_error_rad) ? `${telemetry.max_abs_error_rad.toFixed(4)} rad` : '—';
  const sample = telemetry.sample;
  if (sample && (!samples.length || sample.t > samples[samples.length - 1].t)) {
    samples.push(sample);
    while (samples.length > 300) samples.shift();
  }
  drawChart('positionChart', 'positionLegend', 'position', [
    {key: 'position_actual', label: 'actual'},
    {key: 'position_reference', label: 'reference', dashed: true},
  ]);
  drawChart('velocityChart', 'velocityLegend', 'velocity', [{key: 'velocity', label: 'actual'}]);
  drawChart('effortChart', 'effortLegend', 'effort', [{key: 'effort', label: 'measured'}]);
  drawChart('errorChart', 'errorLegend', 'error', [{key: 'error', label: 'signed'}], true);
  updatePositionTelemetry(telemetry.end_effector);
}

function drawTimeChart(canvasId, legendId, sourceSamples, traces, precision, includeZero, emptyMessage) {
  const canvas = $(canvasId);
  const rect = canvas.getBoundingClientRect();
  if (!rect.width) return;
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * ratio);
  canvas.height = Math.round(rect.height * ratio);
  const context = canvas.getContext('2d');
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  const width = rect.width;
  const height = rect.height;
  const pad = {left: 48, right: 10, top: 12, bottom: 26};
  const plotWidth = width - pad.left - pad.right;
  const plotHeight = height - pad.top - pad.bottom;
  const latest = sourceSamples.length ? sourceSamples[sourceSamples.length - 1].t : 0;
  const visible = sourceSamples.filter(sample => latest - sample.t <= windowSeconds);
  const values = traces.flatMap(trace => visible.map(trace.value).filter(Number.isFinite));
  context.clearRect(0, 0, width, height);
  if (!values.length) {
    context.fillStyle = '#8da9bd';
    context.font = '12px system-ui';
    context.fillText(emptyMessage, pad.left + 8, pad.top + 20);
    $(legendId).innerHTML = '';
    return;
  }
  let minimum = Math.min(...values);
  let maximum = Math.max(...values);
  if (includeZero) {
    minimum = Math.min(minimum, 0);
    maximum = Math.max(maximum, 0);
  }
  if (maximum - minimum < 1e-9) {
    const margin = Math.max(Math.abs(maximum) * 0.05, 0.001);
    minimum -= margin;
    maximum += margin;
  } else {
    const margin = (maximum - minimum) * 0.08;
    minimum -= margin;
    maximum += margin;
  }
  const x = time => pad.left + plotWidth * (1 - Math.max(0, Math.min(windowSeconds, latest - time)) / windowSeconds);
  const y = number => pad.top + plotHeight * (maximum - number) / (maximum - minimum);
  context.font = '10px system-ui';
  context.lineWidth = 1;
  context.strokeStyle = '#1d4764';
  context.fillStyle = '#8da9bd';
  for (let tick = 0; tick <= 4; tick += 1) {
    const yPosition = pad.top + plotHeight * tick / 4;
    const value = maximum - (maximum - minimum) * tick / 4;
    context.beginPath();
    context.moveTo(pad.left, yPosition);
    context.lineTo(width - pad.right, yPosition);
    context.stroke();
    context.fillText(value.toFixed(precision), 2, yPosition + 3);
    const xPosition = pad.left + plotWidth * tick / 4;
    context.beginPath();
    context.moveTo(xPosition, pad.top);
    context.lineTo(xPosition, pad.top + plotHeight);
    context.stroke();
    context.fillText(`-${(windowSeconds * (1 - tick / 4)).toFixed(0)}s`, xPosition - 9, height - 7);
  }
  if (minimum < 0 && maximum > 0) {
    context.strokeStyle = '#58758a';
    context.beginPath();
    context.moveTo(pad.left, y(0));
    context.lineTo(width - pad.right, y(0));
    context.stroke();
  }
  for (const trace of traces) {
    context.strokeStyle = trace.color;
    context.lineWidth = trace.dashed ? 1 : 1.6;
    context.setLineDash(trace.dashed ? [5, 4] : []);
    context.beginPath();
    let drawing = false;
    visible.forEach(sample => {
      const value = trace.value(sample);
      if (!Number.isFinite(value)) {
        drawing = false;
      } else {
        const pointX = x(sample.t);
        const pointY = y(value);
        if (!drawing) context.moveTo(pointX, pointY);
        else context.lineTo(pointX, pointY);
        drawing = true;
      }
    });
    context.stroke();
  }
  context.setLineDash([]);
  $(legendId).innerHTML = traces.map(trace =>
    `<span><i class="${trace.dashed ? 'dashed' : ''}" style="color:${trace.color};background:${trace.dashed ? 'transparent' : trace.color}"></i>${trace.label}</span>`
  ).join('');
}

function updatePositionTelemetry(telemetry) {
  if (!telemetry) {
    $('positionTelemetryState').textContent = 'Restart dashboard';
    $('positionTelemetryState').className = 'pill starting';
    $('positionTelemetryMessage').textContent = 'Dashboard backend is out of date; restart it to load end-effector telemetry.';
    $('positionErrorNow').textContent = '—';
    $('positionErrorStats').textContent = '—';
    drawTimeChart(
      'cartesianPositionChart',
      'cartesianPositionLegend',
      [],
      [],
      3,
      false,
      'Restart dashboard backend to enable this graph',
    );
    drawTimeChart(
      'cartesianErrorChart',
      'cartesianErrorLegend',
      [],
      [],
      1,
      true,
      'Restart dashboard backend to enable this graph',
    );
    return;
  }
  $('positionTelemetryState').textContent = telemetry.ready ? 'Live' : 'Waiting';
  $('positionTelemetryState').className = 'pill ' + (telemetry.ready ? 'ready' : 'stopped');
  $('positionTelemetryMessage').textContent = telemetry.message || 'Waiting for synchronized MoveIt FK and Gazebo tool0 data.';
  const sample = telemetry.sample;
  $('positionErrorNow').textContent = telemetry.ready && sample
    ? `${(sample.error_m * 1000).toFixed(2)} mm`
    : '—';
  $('positionErrorStats').textContent = telemetry.ready
    && Number.isFinite(telemetry.rms_error_mm)
    && Number.isFinite(telemetry.max_error_mm)
    ? `${telemetry.rms_error_mm.toFixed(2)} / ${telemetry.max_error_mm.toFixed(2)} mm`
    : '—';
  if (sample && (!positionSamples.length || sample.t > positionSamples[positionSamples.length - 1].t)) {
    positionSamples.push(sample);
    while (positionSamples.length > 300) positionSamples.shift();
  }
  const axes = ['X', 'Y', 'Z'];
  const traces = axes.flatMap((axis, index) => [
    {
      label: `actual ${axis}`,
      color: positionColors[index],
      value: item => item.position_actual_m[index],
    },
    {
      label: `reference ${axis}`,
      color: positionColors[index],
      dashed: true,
      value: item => item.position_reference_m[index],
    },
  ]);
  drawTimeChart(
    'cartesianPositionChart',
    'cartesianPositionLegend',
    positionSamples,
    traces,
    3,
    false,
    'Waiting for synchronized end-effector data',
  );
  drawTimeChart(
    'cartesianErrorChart',
    'cartesianErrorLegend',
    positionSamples,
    [{
      label: '3D error',
      color: '#ff5865',
      value: item => item.error_m * 1000,
    }],
    1,
    true,
    'Waiting for synchronized end-effector data',
  );
}

function options() {
  return {headless: $('headless').checked, rviz: $('rviz').checked, world: $('world').value.trim() || null};
}

async function command(path) {
  ['start', 'restart', 'stop'].forEach(id => { $(id).disabled = true; });
  try {
    const response = await fetch('/api/system/' + path, {
      method: 'POST',
      headers: {'content-type': 'application/json'},
      body: path === 'stop' ? undefined : JSON.stringify(options()),
    });
    const data = await response.json();
    if (!response.ok) alert(data.detail || 'Command failed');
    if (data.status) render(data.status);
  } catch (error) {
    alert(error.message);
  } finally {
    if (status) render(status);
  }
}

async function workflowCommand(path, body) {
  ['planTrajectory', 'executeTrajectory', 'stopTrajectory', 'startRecording', 'stopRecording']
    .forEach(id => { $(id).disabled = true; });
  try {
    const response = await fetch(path, {
      method: 'POST',
      headers: {'content-type': 'application/json'},
      body: body ? JSON.stringify(body) : undefined,
    });
    const data = await response.json();
    if (!response.ok) alert(data.detail || 'Command failed');
    if (data.status) render(data.status);
  } catch (error) {
    alert(error.message);
  } finally {
    if (status) render(status);
  }
}

function trajectory(execute) {
  workflowCommand('/api/trajectory/start', {
    execute,
    grasp_clearance: Number($('graspClearance').value),
    grasp_approach_height: Number($('approachHeight').value),
    arm_verification_tolerance: Number($('armTolerance').value),
  });
}

function connect() {
  const socket = new WebSocket(`ws://${location.host}/ws`);
  socket.onopen = () => {
    $('connectionDot').className = 'dot green';
    $('connectionText').textContent = 'Backend connected';
  };
  socket.onmessage = event => render(JSON.parse(event.data));
  socket.onclose = () => {
    $('connectionDot').className = 'dot red';
    $('connectionText').textContent = 'Backend disconnected';
    setTimeout(connect, 1500);
  };
  socket.onerror = () => socket.close();
}

$('start').onclick = () => command('start');
$('stop').onclick = () => command('stop');
$('restart').onclick = () => command('restart');
$('clear').onclick = () => { $('logs').textContent = ''; };
$('planTrajectory').onclick = () => trajectory(false);
$('executeTrajectory').onclick = () => trajectory(true);
$('stopTrajectory').onclick = () => workflowCommand('/api/trajectory/stop');
$('startRecording').onclick = () => workflowCommand('/api/recording/start');
$('stopRecording').onclick = () => workflowCommand('/api/recording/stop');
setInterval(() => { $('clock').textContent = new Date().toLocaleTimeString(); }, 1000);
window.addEventListener('resize', () => { if (status) updateTelemetry(status.telemetry); });
connect();
