from tworobot.fw.motion_profile import JerkRamp


def test_jerk_ramp_limits_acceleration_changes_and_tracks_target():
    ramp = JerkRamp()
    a_max, dt = 250.0, 0.01
    max_da = 4 * a_max * dt
    previous_a = ramp.acceleration()
    samples = []
    for _ in range(120):
        velocity = ramp.step(100.0, a_max, dt)
        samples.append(velocity)
        a = ramp.acceleration()
        if velocity < 100.0 - 1e-9:
            assert abs(a - previous_a) <= max_da + 1e-6
        assert abs(a) <= a_max + 1e-6
        previous_a = a
    assert samples == sorted(samples)
    assert abs(samples[-1] - 100.0) < 0.2

    previous_a = ramp.acceleration()
    for _ in range(120):
        velocity = ramp.step(0.0, a_max, dt)
        a = ramp.acceleration()
        if velocity > 1e-9:
            assert abs(a - previous_a) <= max_da + 1e-6
        previous_a = a
    assert abs(ramp.velocity()) < 0.2
