import { api, toast } from './api.js';
import { MANUAL_COMMANDS, manualAvailability } from './manual-core.js?v=manual-solo-3';
import { on, store } from './store.js';

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

let card = null;
let stateEl = null;
let stopButton = null;
let buttons = [];
let active = null;
let activeRid = null;
let runToken = 0;
let stopping = false;
let requestChain = Promise.resolve();

function robot() {
  return (store.frame?.robots || []).find((r) => r.id === store.rid) || null;
}

function availability() {
  const mode = document.getElementById('runMode')?.value || store.mode;
  return manualAvailability(mode, robot());
}

function enqueue(request) {
  const task = requestChain.then(request);
  requestChain = task.catch(() => {});
  return task;
}

function setStatus(text = null) {
  if (stateEl) stateEl.textContent = text || availability().status;
}

function render() {
  if (!card) return;
  const mode = document.getElementById('runMode')?.value || store.mode;
  card.hidden = mode !== 'solo';
  const available = availability().enabled;
  buttons.forEach((button) => {
    button.disabled = !available || Boolean(active && active.button !== button);
    button.classList.toggle('active', active?.button === button);
  });
  stopButton.disabled = !active;
  setStatus(active?.command.label || (stopping ? 'Đang dừng' : null));
}

async function stopMotion() {
  const rid = activeRid;
  runToken += 1;
  active = null;
  activeRid = null;
  if (!rid || stopping) {
    render();
    return;
  }
  stopping = true;
  render();
  try {
    await enqueue(() => api('POST', `/api/robots/${rid}/jog/stop`));
  } catch (e) {
    toast(e.message, 'err');
  } finally {
    stopping = false;
    render();
  }
}

async function driveLoop(token, rid, command) {
  while (token === runToken && active) {
    try {
      await enqueue(() => api('POST', `/api/robots/${rid}/jog`, { v: command.v, w: command.w }));
    } catch (e) {
      toast(e.message, 'err');
      await stopMotion();
      return;
    }
    if (token !== runToken || !active) return;
    await sleep(100);
  }
}

async function startMotion(button, event) {
  if (event.pointerType === 'mouse' && event.button !== 0) return;
  if (!availability().enabled || active || stopping) return;
  event.preventDefault();
  const rid = store.rid;
  const command = MANUAL_COMMANDS[button.dataset.jog];
  if (!rid || !command) return;
  active = { button, command };
  activeRid = rid;
  runToken += 1;
  const token = runToken;
  button.setPointerCapture?.(event.pointerId);
  render();
  await driveLoop(token, rid, command);
}

export function stopManual() {
  void stopMotion();
}

export function initManual() {
  card = document.getElementById('manualControl');
  stateEl = document.getElementById('manualState');
  stopButton = document.getElementById('manualStop');
  buttons = [...document.querySelectorAll('[data-jog]')];
  for (const button of buttons) {
    button.addEventListener('pointerdown', (event) => void startMotion(button, event));
    for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) {
      button.addEventListener(type, () => void stopMotion());
    }
  }
  stopButton.addEventListener('click', () => void stopMotion());
  for (const select of document.querySelectorAll('[data-mode-select], [data-robot-select]')) {
    select.addEventListener('change', () => setTimeout(render, 0));
  }
  window.addEventListener('blur', () => void stopMotion());
  document.addEventListener('visibilitychange', () => { if (document.hidden) void stopMotion(); });
  on('frame', render);
  on('mode', render);
  on('selection', render);
  on('robots', render);
  on('tab', (tab) => { if (tab !== 'run') stopManual(); });
  render();
}
