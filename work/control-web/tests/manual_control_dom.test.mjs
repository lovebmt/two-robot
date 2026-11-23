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

  setPointerCapture() {}
}

test('solo mode updates real robot controls without waiting for a frame', async () => {
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
    getElementById: (id) => elements[id] || null,
    querySelectorAll: (selector) => {
      if (selector === '[data-jog]') return buttons;
      if (selector === '[data-mode-select], [data-robot-select]') return [runMode, robotSelect];
      return [];
    },
    addEventListener() {},
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

  runMode.value = 'solo';
  runMode.dispatch('change');
  await new Promise((resolve) => setTimeout(resolve, 0));

  assert.equal(elements.manualState.textContent, 'Sẵn sàng');
  assert.equal(elements.manualControl.hidden, false);
  assert.deepEqual(buttons.map((button) => button.disabled), [false, false, false, false]);
});
