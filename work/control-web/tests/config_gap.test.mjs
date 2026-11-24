import assert from 'node:assert/strict';
import test from 'node:test';

import { gapClearance } from '../web/js/config-gap.js';

test('gap clearance subtracts follower sensor offset and leader rear offset', () => {
  assert.equal(gapClearance(320, 131, 68), 121);
  assert.equal(gapClearance(299, 131, 68), 100);
});

test('gap clearance supports unavailable geometry', () => {
  assert.equal(gapClearance(320, null, 68), null);
  assert.equal(gapClearance(320, 131, null), null);
});
