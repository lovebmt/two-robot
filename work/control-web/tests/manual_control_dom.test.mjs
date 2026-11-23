import assert from 'node:assert/strict';
import test from 'node:test';

class FakeElement {
  constructor(dataset = {}) {
    this.dataset = dataset;
    this.hidden = false;
    this.disabled = false;
    this.textContent = '';
    this.value = '';
    this.listeners = new Map();
    this.classList = { toggle() {}, add() {}, remove() {} };
  }

  addEventListener(type, listener) {
    const listeners = this.listeners.get(type) || [];
    listeners.push(listener);
    this.listeners.set(type, listeners);
  }

  dispatch(type) {
    for (const listener of this.listeners.get(type) || []) listener({ type });
  }

  blur() {
    this.blurred = true;
    globalThis.document.activeElement = { tagName: 'BODY' };
  }

  setPointerCapture() {}
}

test('solo mode updates real robot controls without waiting for a frame', async () => {
  const fetchCalls = [];
  globalThis.fetch = async (url, options) => {
    fetchCalls.push({ url, method: options.method, body: JSON.parse(options.body || 'null') });
    return { ok: true, status: 200, json: async () => ({ ok: true }) };
  };
  const buttons = ['fwd', 'back', 'left', 'right'].map((jog) => new FakeElement({ jog }));
  const runMode = new FakeElement();
  runMode.value = 'pair';
  const robotSelect = new FakeElement();
  robotSelect.value = '198a2c';
  const elements = {
    manualControl: new FakeElement(),
    manualState: new FakeElement(),
    manualStop: new FakeElement(),
    runMode,
  };
  globalThis.document = {
    hidden: false,
    activeElement: { tagName: 'BODY' },
    listeners: new Map(),
    getElementById: (id) => elements[id] || null,
    querySelectorAll: (selector) => {
      if (selector === '[data-jog]') return buttons;
      if (selector === '[data-mode-select], [data-robot-select]') return [runMode, robotSelect];
      return [];
    },
    addEventListener(type, listener) {
      const listeners = this.listeners.get(type) || [];
      listeners.push(listener);
      this.listeners.set(type, listeners);
    },
    dispatch(type, event) {
      const detail = { type, preventDefault() {}, ...event };
      for (const listener of this.listeners.get(type) || []) listener(detail);
    },
  };
  globalThis.window = { addEventListener() {} };

  const { store } = await import('../web/js/store.js');
  const { initManual } = await import('../web/js/manual.js?dom-test=1');
  store.mode = 'pair';
  store.rid = '198a2c';
  store.frame = { robots: [{ id: '198a2c', online: true, state: 'IDLE' }] };
  initManual();
  assert.equal(elements.manualState.textContent, 'Không khả dụng');
  assert.equal(elements.manualControl.hidden, true);

  document.activeElement = runMode;
  runMode.value = 'solo';
  runMode.dispatch('change');
  await new Promise((resolve) => setTimeout(resolve, 0));

  assert.equal(elements.manualState.textContent, 'Sẵn sàng');
  assert.equal(elements.manualControl.hidden, false);
  assert.deepEqual(buttons.map((button) => button.disabled), [false, false, false, false]);
  assert.equal(runMode.blurred, true);

  document.dispatch('keydown', { code: 'KeyW', key: 'w', repeat: false });
  await new Promise((resolve) => setTimeout(resolve, 10));
  assert.deepEqual(fetchCalls.at(-1), {
    url: '/api/robots/198a2c/jog',
    method: 'POST',
    body: { v: 80, w: 0 },
  });

  document.dispatch('keyup', { code: 'KeyW', key: 'w' });
  await new Promise((resolve) => setTimeout(resolve, 10));
  assert.deepEqual(fetchCalls.at(-1), {
    url: '/api/robots/198a2c/jog/stop',
    method: 'POST',
    body: null,
  });

  const beforeInput = fetchCalls.length;
  document.activeElement = { tagName: 'INPUT' };
  document.dispatch('keydown', { code: 'KeyS', key: 's', repeat: false });
  await new Promise((resolve) => setTimeout(resolve, 10));
  assert.equal(fetchCalls.length, beforeInput);

  document.activeElement = { tagName: 'BODY' };
  for (const [code, body] of [
    ['KeyA', { v: 0, w: 0.8 }],
    ['KeyS', { v: -80, w: 0 }],
    ['KeyD', { v: 0, w: -0.8 }],
  ]) {
    document.dispatch('keydown', { code, key: code.slice(-1).toLowerCase(), repeat: false });
    await new Promise((resolve) => setTimeout(resolve, 10));
    assert.deepEqual(fetchCalls.at(-1).body, body);
    document.dispatch('keyup', { code, key: code.slice(-1).toLowerCase() });
    await new Promise((resolve) => setTimeout(resolve, 10));
    assert.equal(fetchCalls.at(-1).url, '/api/robots/198a2c/jog/stop');
  }
});
