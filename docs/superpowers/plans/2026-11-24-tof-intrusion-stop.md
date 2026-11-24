# ToF Intrusion Stop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Execute inline; do not delegate.

**Goal:** Stop both robots when either follower ToF detects a sudden close reading or loss of signal during `FOLLOW`.

**Architecture:** Add a pure anomaly guard shared by C++ firmware and Python virtual firmware, invoke it only during `FOLLOW`, then rely on the existing coordinator fault path to brake the leader and surface the fault in the UI.

**Tech Stack:** C++17 firmware, ArduinoJson, PlatformIO Unity tests, Python virtual firmware, FastAPI coordinator, vanilla ES modules.

---

### Task 1: Add shared thresholds and anomaly guard

**Files:**
- Modify: `work/shared/config.schema.json`
- Create: `work/firmware/lib/core/src/tof_guard.h`
- Create: `work/control-web/server/tworobot/fw/tof_guard.py`
- Create: `work/control-web/tests/test_tof_guard.py`
- Modify: `work/firmware/test/test_core/test_main.cpp`

- [ ] **Step 1: Add schema thresholds and regenerate C++ config**

Add to group `safety`:

```json
{"key":"safety.tof_anomaly_jump_mm","type":"float","unit":"mm","default":120,"min":20,"max":600,"group":"safety","scope":"robot","apply":"live","desc":"Một mắt ToF ngắn đột ngột quá ngưỡng → dừng"},
{"key":"safety.tof_anomaly_samples","type":"int","unit":"mẫu","default":2,"min":1,"max":10,"group":"safety","scope":"robot","apply":"live","desc":"Số mẫu bất thường liên tiếp để báo vật cản"}
```

Run `python3 work/shared/codegen_cpp.py`.

- [ ] **Step 2: Write failing Python guard tests**

Create `tests/test_tof_guard.py` covering gradual change, unarmed startup, one-eye jump,
both-eye jump, one-eye loss and both-eye loss. Use the guard API
`TofAnomalyGuard.update(l_mm, l_st, r_mm, r_st, cfg) -> bool` and `reset()`.

Run `.venv/bin/pytest tests/test_tof_guard.py -q`; expected: import fails because module
does not exist.

- [ ] **Step 3: Implement Python guard**

Create `tof_guard.py` with two retained raw-value histories, per-eye candidate streaks,
an `armed` flag set when both histories reach three samples, and reset after the first
confirmed anomaly. Use sorted-list median and the two schema thresholds.

- [ ] **Step 4: Run Python guard tests**

Run `.venv/bin/pytest tests/test_tof_guard.py -q`; expected PASS.

- [ ] **Step 5: Write failing C++ tests**

Add the same six scenarios to `test_core/test_main.cpp`, constructing `TofAnomalyGuard`
directly with `Config`.

Run `pio test -e native`; expected compile failure because `tof_guard.h` API is absent.

- [ ] **Step 6: Implement C++ guard**

Create `tof_guard.h` as a header-only class with fixed arrays of size 5 and identical
semantics to Python. Include it from `test_main.cpp`.

- [ ] **Step 7: Run native tests and commit**

Run `pio test -e native`; expected all tests pass. Commit schema, generated config,
guard implementations and tests with `feat: add ToF anomaly guard`.

### Task 2: Trigger follower fault during FOLLOW

**Files:**
- Modify: `work/firmware/lib/core/src/app.h`
- Modify: `work/firmware/lib/core/src/app.cpp`
- Modify: `work/control-web/server/tworobot/fw/robot.py`
- Modify: `work/firmware/test/test_core/test_main.cpp`

- [ ] **Step 1: Write failing C++ integration test**

Drive an `App` into `RUNNING/FOLLOW` with two valid eyes, inject 15 mm on one eye for
two ToF callbacks, and assert `state == "FAULT"` and `fault == "OBSTACLE_INTRUSION"`.
Repeat in IDLE and assert no new guard fault.

- [ ] **Step 2: Integrate C++ guard**

Add a `TofAnomalyGuard tof_guard_` member. Reset it when starting `FOLLOW`. In
`App::onRange`, before `prim_->onRange`, call the guard only when
`state == "RUNNING" && !paused && prim_->name == "FOLLOW"`; on true call
`raiseFault("OBSTACLE_INTRUSION")` and return.

- [ ] **Step 3: Write failing Python integration test**

Add a virtual firmware test that arms a follower, starts `FOLLOW`, feeds two anomalous
range samples, and asserts `OBSTACLE_INTRUSION` plus brake.

- [ ] **Step 4: Integrate Python virtual firmware**

Mirror reset and trigger logic in `VirtualFirmware.on_range` only for `FOLLOW`.

- [ ] **Step 5: Verify and commit**

Run `pio test -e native`, `.venv/bin/pytest -q`, and
`pio run -e esp32dev`. Commit with `feat: fault on ToF intrusion`.

### Task 3: Stop leader and warn on UI

**Files:**
- Modify: `work/control-web/server/tworobot/coordinator.py`
- Modify: `work/control-web/web/js/view.js`
- Modify: `work/control-web/web/js/devices.js`
- Modify: `work/control-web/tests/test_sim.py`

- [ ] **Step 1: Write failing coordinator test**

Start a two-robot run, inject one-eye `tof_block`, wait, and assert coordinator reaches
`FAULT`, error contains `OBSTACLE_INTRUSION`, and both robot states are not `RUNNING`.

- [ ] **Step 2: Improve fault propagation**

In `Coordinator.on_event`, keep the existing `_fault`, append a log line and call
`on_change()` immediately when a fatal fault arrives. Existing `RunFault` handling will
pause both robots.

- [ ] **Step 3: Add UI warning**

Map `OBSTACLE_INTRUSION` to “Vật cản/bất thường ToF giữa 2 xe”. On a new fault event,
show an error toast once and keep the existing event-log entry. Add the code to the fatal
fault chip list in `devices.js`.

- [ ] **Step 4: Verify and commit**

Run full Python tests, JS syntax checks and native tests. Commit with
`feat: stop both robots on ToF intrusion`.

### Task 4: Build, flash and verify hardware

**Files:** No source changes.

- [ ] **Step 1: Build ESP32**

Run `pio run -e esp32dev`; expected SUCCESS.

- [ ] **Step 2: OTA flash real follower**

Run the firmware build’s `ota` upload to the current follower IP from Web Control.
OTA starts with E-STOP/brakes. If IP changed, read it from `/api/robots`.

- [ ] **Step 3: Verify startup**

Confirm firmware boot log, WiFi connection, config remains valid, and the two ToF eyes
return valid readings.
