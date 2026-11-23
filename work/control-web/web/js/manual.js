import { api, toast } from './api.js';
import { MANUAL_COMMANDS, manualAvailability } from './manual-core.js?v=manual-solo-5';
import { on, store } from './store.js';

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
const KEY_COMMANDS = { KeyW: 'fwd', KeyS: 'back', KeyA: 'left', KeyD: 'right' };

let card = null;
let stateEl = null;
let stopButton = null;
let buttons = [];
let active = null;
let activeRid = null;
let runToken = 0;
let stopping = false;
let requestChain = Promise.resolve();
let activeKey = null;

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

function keyboardBlocked(event) {
  const tag = document.activeElement?.tagName;
  return event.repeat || event.ctrlKey || event.metaKey || event.altKey
    || ['INPUT', 'SELECT', 'TEXTAREA'].includes(tag) || document.activeElement?.isContentEditable;
}

function handleKeyDown(event) {
  const kind = KEY_COMMANDS[event.code];
  const button = buttons.find((item) => item.dataset.jog === kind);
  if (!kind || !button || keyboardBlocked(event) || card.hidden || !availability().enabled
    || active || stopping) return;
  event.preventDefault();
  activeKey = event.code;
  void startMotion(button, { pointerType: 'keyboard', preventDefault() {} });
}

function handleKeyUp(event) {
  if (!activeKey || event.code !== activeKey) return;
  event.preventDefault();
  activeKey = null;
  void stopMotion();
}

export function stopManual() {
  activeKey = null;
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
    select.addEventListener('change', () => {
      select.blur?.();
      setTimeout(render, 0);
    });
  }
  document.addEventListener('keydown', handleKeyDown);
  document.addEventListener('keyup', handleKeyUp);
  window.addEventListener('blur', () => { activeKey = null; void stopMotion(); });
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) { activeKey = null; void stopMotion(); }
  });
  on('frame', render);
  on('mode', render);
  on('selection', render);
  on('robots', render);
  on('tab', (tab) => { if (tab !== 'run') stopManual(); });
  render();
}
