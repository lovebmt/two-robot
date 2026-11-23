import assert from 'node:assert/strict';
import test from 'node:test';

import { MANUAL_COMMANDS, manualAvailability } from '../web/js/manual-core.js';

test('manual directions use the agreed safe speeds', () => {
  assert.deepEqual(MANUAL_COMMANDS.fwd, { v: 80, w: 0, label: 'Đang tiến' });
  assert.deepEqual(MANUAL_COMMANDS.back, { v: -80, w: 0, label: 'Đang lùi' });
  assert.deepEqual(MANUAL_COMMANDS.left, { v: 0, w: 0.8, label: 'Đang quay trái' });
  assert.deepEqual(MANUAL_COMMANDS.right, { v: 0, w: -0.8, label: 'Đang quay phải' });
});

test('manual controls are limited to an online solo robot in IDLE or JOG', () => {
  assert.deepEqual(manualAvailability('pair', { online: true, state: 'IDLE' }),
    { enabled: false, status: 'Không khả dụng' });
  assert.deepEqual(manualAvailability('solo', { online: false, state: 'IDLE' }),
    { enabled: false, status: 'Xe offline' });
  assert.deepEqual(manualAvailability('solo', { online: true, state: 'RUNNING' }),
    { enabled: false, status: 'Không khả dụng · RUNNING' });
  assert.deepEqual(manualAvailability('solo', { online: true, state: 'IDLE' }),
    { enabled: true, status: 'Sẵn sàng' });
  assert.deepEqual(manualAvailability('solo', { online: true, state: 'JOG' }),
    { enabled: true, status: 'Sẵn sàng' });
});
