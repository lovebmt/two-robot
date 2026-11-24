# Fixed WiFi and Smooth Steps Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Execute inline.

**Goal:** Remove AP fallback and apply jerk-limited speed changes to stop-and-go primitives.

**Architecture:** Add a pure one-dimensional jerk ramp shared by C++ and Python firmware, then replace direct acceleration clamps in DRIVE_TO, TURN and FOLLOW. Remove AP-only network code and keep NVS/serial WiFi recovery.

**Tech Stack:** Arduino ESP32, C++17 core, Python virtual firmware, PlatformIO, FastAPI tests.

---

### Task 1: Add jerk ramp

**Files:**
- Create: `work/firmware/lib/core/src/motion_profile.h`
- Create: `work/control-web/server/tworobot/fw/motion_profile.py`
- Modify: `work/firmware/test/test_core/test_main.cpp`
- Create: `work/control-web/tests/test_motion_profile.py`

- [ ] **Step 1: Write failing Python test**

Create `JerkRamp.reset(v0)`, `.step(target, accel_limit, dt)`, `.velocity()` in
`motion_profile.py`. Test that acceleration change per sample never exceeds
`4 × accel_limit × dt`, acceleration reaches both signs smoothly, and target is tracked.

Run `.venv/bin/pytest tests/test_motion_profile.py -q`; expected import error.

- [ ] **Step 2: Implement Python ramp and make test pass.**

- [ ] **Step 3: Write failing C++ test.**

Add the same assertions with `JerkRamp` to `test_core/test_main.cpp`; run
`pio test -e native` and expect missing header.

- [ ] **Step 4: Implement `motion_profile.h` and make native tests pass.**

- [ ] **Step 5: Commit** with `feat: add jerk-limited motion ramp`.

### Task 2: Integrate ramp in stop-and-go primitives

**Files:**
- Modify: `work/firmware/lib/core/src/primitives.cpp`
- Modify: `work/control-web/server/tworobot/fw/primitives.py`
- Modify: `work/control-web/tests/test_sim.py`

- [ ] **Step 1: Add failing regression tests** preserving L/rect/zigzag completion and corner errors.

- [ ] **Step 2: Replace direct acceleration clamps** in `DriveTo`, `Turn`, and `Follow` with `JerkRamp`; safety-triggered braking remains immediate.

- [ ] **Step 3: Verify** `pio test -e native` and full Python suite; commit with `feat: smooth stop-and-go primitives`.

### Task 3: Remove AP fallback

**Files:**
- Modify: `work/firmware/src/net.cpp`
- Modify: `work/firmware/src/net.h`
- Modify: `work/firmware/src/main.cpp`
- Modify: `work/firmware/src/secrets.h.example`
- Modify: `work/firmware/README.md`

- [ ] **Step 1: Remove AP globals, `isAp`, AP root form, `/wifi` handler and preferences parameter.**

- [ ] **Step 2: Prefer compiled credentials, otherwise NVS; on failure stay STA/offline with a serial error.**

- [ ] **Step 3: Build ESP32**, commit with `refactor: remove WiFi AP fallback`.

### Task 4: Flash and verify

- [ ] **Step 1: Build with `secrets.h`, OTA both robots.**
- [ ] **Step 2: Confirm serial/API reports station IP, never `192.168.4.1`.**
- [ ] **Step 3: Run a short two-robot stop-and-go path and compare corner errors.**
