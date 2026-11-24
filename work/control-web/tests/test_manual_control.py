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
