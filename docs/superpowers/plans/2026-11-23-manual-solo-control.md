# Manual Solo Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Subagent execution is not used for this task; execute inline and keep changes scoped.

**Goal:** Add hold-to-run Web controls for forward, backward, left, right, and stop in solo mode only.

**Architecture:** Reuse the existing robot jog transport and firmware 300 ms command expiry. Add server-side value clamping and a dedicated stop command, then add a small frontend controller that repeats jog commands only while a pointer is held and stops on every release path.

**Tech Stack:** FastAPI, pytest, vanilla ES modules, HTML/CSS/Canvas Web Control, ESP32 jog protocol.

---

### Task 1: Validate jog commands and add manual stop

**Files:**
- Modify: `work/control-web/server/tworobot/app.py:368-376`
- Create: `work/control-web/tests/test_manual_control.py`

- [ ] **Step 1: Write failing server tests**

Create `work/control-web/tests/test_manual_control.py`:

```python
import asyncio
import math

import pytest
from fastapi import HTTPException

from tworobot.app import S, jog, jog_stop, manual_jog_motion
from tworobot.registry import RobotSession


class RecordingLink:
    def __init__(self):
        self.messages = []

    async def send(self, msg):
        self.messages.append(msg)

    def close(self):
        pass


def test_manual_jog_motion_clamps_values():
    assert manual_jog_motion({"v": 999, "w": -999}) == (150.0, -2.0)
    assert manual_jog_motion({"v": -999, "w": 999}) == (-150.0, 2.0)


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_manual_jog_motion_rejects_non_finite(value):
    with pytest.raises(HTTPException) as exc:
        manual_jog_motion({"v": value, "w": 0})
    assert exc.value.status_code == 400


def test_jog_and_stop_send_safe_commands():
    async def go():
        link = RecordingLink()
        session = RobotSession("manual-test", "Manual test", link, "SIM")
        S.reg.add(session)
        try:
            await jog("manual-test", {"v": 999, "w": -999})
            assert link.messages[-1] == {"t": "cmd", "op": "jog", "v": 150.0, "w": -2.0}
            await jog_stop("manual-test")
            assert link.messages[-1] == {"t": "cmd", "op": "stop"}
        finally:
            S.reg.remove("manual-test")

    asyncio.run(go())
```

- [ ] **Step 2: Run the test and verify it fails**

Run:

```bash
cd work/control-web
.venv/bin/pytest tests/test_manual_control.py -q
```

Expected: collection fails because `jog_stop` and `manual_jog_motion` do not exist.

- [ ] **Step 3: Add validation and stop endpoint**

Before the `/api/robots/{rid}/jog` route in `app.py`, add:

```python
def manual_jog_motion(body: dict) -> tuple[float, float]:
    try:
        v = float(body.get("v", 0.0))
        w = float(body.get("w", 0.0))
    except (TypeError, ValueError):
        raise HTTPException(400, "v/w phải là số") from None
    if not math.isfinite(v) or not math.isfinite(w):
        raise HTTPException(400, "v/w phải hữu hạn")
    return max(-150.0, min(150.0, v)), max(-2.0, min(2.0, w))
```

Replace the jog route body with:

```python
@app.post("/api/robots/{rid}/jog")
async def jog(rid: str, body: dict = Body(...)):
    s = _sess(rid)
    v, w = manual_jog_motion(body)
    try:
        await s.link.send({"t": "cmd", "op": "jog", "v": v, "w": w})
    except LinkError as e:
        raise HTTPException(502, str(e)) from None
    return {"ok": True}


@app.post("/api/robots/{rid}/jog/stop")
async def jog_stop(rid: str):
    s = _sess(rid)
    try:
        await s.link.send({"t": "cmd", "op": "stop"})
    except LinkError as e:
        raise HTTPException(502, str(e)) from None
    return {"ok": True}
```

- [ ] **Step 4: Run server tests**

Run:

```bash
cd work/control-web
.venv/bin/pytest tests/test_manual_control.py tests/test_http.py -q
```

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add work/control-web/server/tworobot/app.py work/control-web/tests/test_manual_control.py
git commit -m "feat: secure solo manual jog commands"
```

### Task 2: Add solo-only manual controller UI

**Files:**
- Modify: `work/control-web/web/index.html:280-290`
- Modify: `work/control-web/web/css/app.css:220-235`
- Modify: `work/control-web/web/js/main.js:1-75`
- Modify: `work/control-web/web/js/view.js:68-80`
- Create: `work/control-web/web/js/manual.js`

- [ ] **Step 1: Add the control card and styles**

Insert this card immediately after `<div class="run-side">` in `index.html`:

```html
<div class="card manual-control" id="manualControl">
  <div class="card-h">
    <h2>Điều khiển tay</h2>
    <span id="manualState" class="badge">Không khả dụng</span>
  </div>
  <div class="manual-grid">
    <span></span>
    <button class="manual-btn" data-jog="fwd" aria-label="Tiến">▲<small>Tiến</small></button>
    <span></span>
    <button class="manual-btn" data-jog="left" aria-label="Quay trái">◀<small>Trái</small></button>
    <button class="manual-stop" id="manualStop" aria-label="Dừng">■<small>STOP</small></button>
    <button class="manual-btn" data-jog="right" aria-label="Quay phải">▶<small>Phải</small></button>
    <span></span>
    <button class="manual-btn" data-jog="back" aria-label="Lùi">▼<small>Lùi</small></button>
    <span></span>
  </div>
  <p class="muted small">Giữ nút để chạy, nhả nút để dừng. Chỉ dùng ở chế độ 1 xe.</p>
</div>
```

Append these styles in `app.css`:

```css
.manual-control { display: none; }
body.mode-solo .manual-control { display: block; }
.manual-grid {
  display: grid;
  grid-template-columns: repeat(3, 64px);
  grid-template-rows: repeat(3, 64px);
  gap: 8px;
  justify-content: center;
  margin: 8px auto 12px;
}
.manual-btn, .manual-stop {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  border: 1px solid var(--border);
  border-radius: 10px;
  background: var(--surface-2);
  color: var(--text);
  cursor: pointer;
  touch-action: none;
  user-select: none;
}
.manual-btn small, .manual-stop small { font-size: 10px; }
.manual-btn.active { background: var(--accent); color: var(--accent-text); }
.manual-stop { background: var(--danger); border-color: var(--danger); color: white; font-weight: 700; }
.manual-btn:disabled, .manual-stop:disabled { opacity: .45; cursor: not-allowed; }
```

- [ ] **Step 2: Add the hold-to-run controller**

Create `work/control-web/web/js/manual.js`:

```js
import { api, toast } from './api.js';
import { on, store } from './store.js';

const COMMANDS = {
  fwd: { v: 80, w: 0, label: 'Đang tiến' },
  back: { v: -80, w: 0, label: 'Đang lùi' },
  left: { v: 0, w: 0.8, label: 'Đang quay trái' },
  right: { v: 0, w: -0.8, label: 'Đang quay phải' },
};
const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

let card = null;
let stateEl = null;
let stopButton = null;
let buttons = [];
let active = null;
let activeRid = null;
let runToken = 0;
let stopping = false;

function robot() {
  return (store.frame?.robots || []).find((r) => r.id === store.rid) || null;
}

function canMove() {
  const r = robot();
  return store.mode === 'solo' && r && r.online !== false && ['IDLE', 'JOG'].includes(r.state);
}

function setStatus(text = null) {
  if (!stateEl) return;
  const r = robot();
  if (text) stateEl.textContent = text;
  else if (store.mode !== 'solo') stateEl.textContent = 'Không khả dụng';
  else if (!r || r.online === false) stateEl.textContent = 'Xe offline';
  else if (!['IDLE', 'JOG'].includes(r.state)) stateEl.textContent = `Không khả dụng · ${r.state}`;
  else stateEl.textContent = 'Sẵn sàng';
}

function render() {
  if (!card) return;
  card.hidden = store.mode !== 'solo';
  const enabled = canMove();
  buttons.forEach((button) => {
    button.disabled = !enabled || (active && active.button !== button);
  });
  stopButton.disabled = !active;
  setStatus(active?.command.label);
}

async function stopMotion() {
  const rid = activeRid;
  runToken += 1;
  active?.button.classList.remove('active');
  active = null;
  activeRid = null;
  render();
  if (!rid || stopping) return;
  stopping = true;
  try {
    await api('POST', `/api/robots/${rid}/jog/stop`);
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
      await api('POST', `/api/robots/${rid}/jog`, { v: command.v, w: command.w });
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
  if (!canMove() || active || stopping) return;
  event.preventDefault();
  const rid = store.rid;
  const command = COMMANDS[button.dataset.jog];
  active = { button, command };
  activeRid = rid;
  runToken += 1;
  const token = runToken;
  button.classList.add('active');
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
  window.addEventListener('blur', () => void stopMotion());
  document.addEventListener('visibilitychange', () => { if (document.hidden) void stopMotion(); });
  on('frame', render);
  on('tab', (tab) => { if (tab !== 'run') stopManual(); });
  render();
}
```

- [ ] **Step 3: Wire mode class, stop on mode/robot changes, and frame updates**

In `main.js`, import:

```js
import { initManual, stopManual } from './manual.js';
```

Call `initManual()` after `initView()`.

In `applyModeClass`, add:

```js
document.body.classList.toggle('mode-solo', store.mode === 'solo');
```

At the start of the mode-selector `onchange` handler and robot-selector `onchange` handler, call:

```js
stopManual();
```

In `view.js`, import `emit` from `store.js` and add immediately after `store.frame = f` inside `onFrame`:

```js
emit('frame', f);
```

- [ ] **Step 4: Run static and server checks**

Run:

```bash
cd work/control-web
node --check web/js/manual.js
node --check web/js/main.js
node --check web/js/view.js
.venv/bin/pytest -q
```

Expected: all JavaScript syntax checks and Python tests pass.

- [ ] **Step 5: Verify in the browser with a simulated robot**

1. Start or reuse the Web Control server.
2. Add/open a simulated robot.
3. Select `1 xe` and confirm **Điều khiển tay** appears.
4. Select `2 xe` and confirm the card is hidden.
5. Return to `1 xe`, hold **Tiến**, confirm state changes to `JOG`, and release.
6. Confirm the robot returns to `IDLE` and **STOP** brakes immediately.
7. Start a short solo route and confirm the stop command does not leave the robot stuck in `JOG`.

- [ ] **Step 6: Commit**

```bash
git add work/control-web/web/index.html work/control-web/web/css/app.css \
  work/control-web/web/js/main.js work/control-web/web/js/view.js \
  work/control-web/web/js/manual.js
git commit -m "feat: add manual controls for solo mode"
```

### Task 3: Update protocol documentation

**Files:**
- Modify: `wiki/protocol.md:20-40`

- [ ] **Step 1: Document jog limits and manual stop**

Add these rows to the endpoint table:

```markdown
| POST | `/api/robots/{rid}/jog` | điều khiển tay `{v,w}`; giới hạn `v∈[-150,150]`, `w∈[-2,2]` | `{ok}` |
| POST | `/api/robots/{rid}/jog/stop` | phanh và trả robot về `IDLE` | `{ok}` |
```

- [ ] **Step 2: Verify documentation diff**

Run:

```bash
git diff --check
```

Expected: no output.

- [ ] **Step 3: Commit**

```bash
git add wiki/protocol.md
git commit -m "docs: document solo manual jog controls"
```
