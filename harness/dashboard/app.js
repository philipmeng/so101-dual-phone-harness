'use strict';
const robots = ['follower_1', 'follower_2'];
const cards = new Map();
for (const [index, robot] of robots.entries()) {
  const card = document.querySelector('#arm-template').content.firstElementChild.cloneNode(true);
  card.dataset.robot = robot;
  card.querySelector('h2').textContent = `Follower ${index + 1}`;
  card.querySelector('.arm-number').textContent = `0${index + 1}`;
  card.querySelector('.phone-name').textContent = index === 0 ? 'mobileIO-follower1' : 'mobileIO';
  card.querySelector('.pause').addEventListener('click', () => control(robot, 'pause'));
  card.querySelector('.reset').addEventListener('click', () => control(robot, 'reset'));
  card.querySelector('.release').addEventListener('click', () => control(robot, 'release'));
  card.querySelector('input').addEventListener('change', () => buttons(card));
  card.querySelector('input').setAttribute('aria-label', `Follower ${index + 1} is supported`);
  card.querySelector('.pause').setAttribute('aria-label', `Pause Follower ${index + 1} movement`);
  card.querySelector('.reset').setAttribute('aria-label', `Reset Follower ${index + 1} session`);
  card.querySelector('.release').setAttribute('aria-label', `Release Follower ${index + 1} motors and end session`);
  document.querySelector('#arms').append(card);
  cards.set(robot, card);
}
function text(card, selector, value) { card.querySelector(selector).textContent = value; }
function indicator(card, selector, good, label) {
  text(card, `${selector} span:last-child`, label);
  card.querySelector(`${selector} .dot`).className = `dot ${good ? 'good' : 'warn'}`;
}
function buttons(card) {
  const unavailable = card.dataset.available !== 'true' || card.dataset.pending === 'true' || card.dataset.resetting === 'true';
  card.querySelector('.pause').disabled = unavailable;
  card.querySelector('.release').disabled = unavailable || !card.querySelector('input').checked;
  card.querySelector('.reset').disabled = unavailable || card.dataset.resetAvailable !== 'true' || !card.querySelector('input').checked;
}
function unavailable(card, message) {
  card.dataset.available = 'false';
  card.classList.add('stale');
  indicator(card, '.status', false, 'Unreachable');
  indicator(card, '.phone-status', false, 'Phone unknown');
  indicator(card, '.tracking-status', false, 'Tracking unknown');
  text(card, '.controller-message', message || 'Connection lost. Motor state is unknown; the arm may still be holding.');
  text(card, '.motor-state', 'Motor state unknown');
  text(card, '.b1-value', '—');
  text(card, '.b1-hint', 'No current input');
  text(card, '.a3-value', '—');
  for (const axis of ['x', 'y', 'z']) text(card, `.delta-${axis}`, '—');
  card.querySelector('input').checked = false;
  buttons(card);
}
function render(arm) {
  const card = cards.get(arm.robot_id);
  if (!card) return;
  text(card, '.phone-name', arm.phone_name);
  text(card, '.family', arm.phone_family);
  const session = arm.session || {};
  card.dataset.resetAvailable = String(session.reset_available === true);
  card.dataset.resetting = String(session.busy === true);
  text(card, '.reset-help', session.reset_available ? 'After reset, release B1 and press it again to move.' : 'Session reset is not enabled on this dashboard.');
  if (session.request_id && card.dataset.resetRequest !== session.request_id) {
    card.dataset.resetRequest = session.request_id;
    card.dataset.resetFinished = 'false';
  }
  if (session.busy || (session.request_id && card.dataset.resetFinished !== 'true')) {
    text(card, '.result', session.message);
    if (!session.busy) { card.dataset.resetFinished = 'true'; card.querySelector('input').checked = false; }
  }
  if (!arm.available) { unavailable(card, `${arm.error} Motor state is unknown.`); return; }
  const state = arm.state;
  card.dataset.available = 'true';
  card.classList.remove('stale');
  const message = state.control_message || 'Receiver connected. No controller status available.';
  const stale = /STALE/i.test(message);
  const enabled = state.arm_control_enabled === true;
  // Receiver messages carry controller state. Never infer powered-off from a lost connection.
  const status = stale ? 'Status stale' : /FAULT|STOPPED.*fault/i.test(message) ? 'Fault' : /PAUSED/i.test(message) ? 'Paused' : enabled && !state.connected ? 'Phone lost' : enabled ? (state.b1 && state.tracking ? 'Active' : 'Holding') : /SESSION ENDED/i.test(message) ? 'Session ended' : /READY/i.test(message) ? 'Ready' : 'Standby';
  indicator(card, '.status', !stale && !['Fault', 'Paused', 'Standby', 'Phone lost', 'Session ended'].includes(status), status);
  indicator(card, '.phone-status', state.connected, state.connected ? 'Phone connected' : 'Phone disconnected');
  indicator(card, '.tracking-status', state.tracking, state.tracking ? 'Tracking ready' : 'Tracking not ready');
  text(card, '.controller-message', message);
  text(card, '.motor-state', stale ? 'Motor state unknown' : enabled ? 'Motors holding / active' : /OFF|RELEASED|READY/i.test(message) ? 'Motors off' : 'See controller status');
  text(card, '.b1-value', state.connected ? (state.b1 ? 'Held' : 'Released') : '—');
  text(card, '.b1-hint', state.connected ? (state.b1 ? 'Phone motion engaged' : 'Press & hold to move') : 'Waiting for phone');
  const a3 = Number.isFinite(state.a3) && state.connected ? state.a3 : null;
  text(card, '.a3-value', a3 === null ? '—' : a3.toFixed(2));
  card.querySelector('.slider-thumb').style.left = `${((Math.max(-1, Math.min(1, a3 || 0)) + 1) / 2) * 100}%`;
  card.querySelector('.slider-thumb').style.visibility = a3 === null ? 'hidden' : '';
  for (const [i, axis] of ['x', 'y', 'z'].entries()) {
    const value = state.delta_cm?.[i];
    text(card, `.delta-${axis}`, state.connected && state.tracking && Number.isFinite(value) ? `${value > 0 ? '+' : ''}${value.toFixed(1)}` : '—');
  }
  buttons(card);
}
async function control(robot, action) {
  const card = cards.get(robot);
  if (card.dataset.pending === 'true' || card.dataset.available !== 'true') return;
  const supported = card.querySelector('input').checked;
  if (['release', 'reset'].includes(action) && !supported) return;
  card.dataset.pending = 'true'; buttons(card);
  text(card, '.result', 'Sending request…');
  try {
    const response = await fetch(`/api/arms/${robot}/control`, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({action, supported, ...(action === 'reset' ? {request_id:crypto.randomUUID()} : {})}), signal:AbortSignal.timeout(5000)});
    const result = await response.json();
    text(card, '.result', response.ok ? result.message : result.error);
    if (result.session?.busy) card.dataset.resetting = 'true';
  } catch {
    text(card, '.result', 'Request not confirmed. Check the arm status before retrying.');
  } finally {
    card.dataset.pending = 'false';
    if (['release', 'reset'].includes(action)) card.querySelector('input').checked = false;
    buttons(card);
  }
}
async function tick() {
  try {
    const response = await fetch('/api/state', {cache:'no-store', signal:AbortSignal.timeout(3500)});
    if (!response.ok) throw new Error('Dashboard unavailable');
    const data = await response.json();
    if (!Array.isArray(data.arms) || data.arms.length !== 2) throw new Error('Invalid state');
    data.arms.forEach(render);
    const count = data.arms.filter(arm => arm.available).length;
    document.querySelector('#network').textContent = `${count} of 2 receivers connected`;
    document.querySelector('#network-dot').className = `dot ${count === 2 ? 'good' : 'warn'}`;
  } catch {
    cards.forEach(card => unavailable(card));
    document.querySelector('#network').textContent = 'Workspace connection lost';
    document.querySelector('#network-dot').className = 'dot warn';
  }
  setTimeout(tick, 400);
}
tick();
