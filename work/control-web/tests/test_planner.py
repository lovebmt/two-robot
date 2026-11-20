from tworobot import params as P
from tworobot.planner import plan_path

GRID = {"cell_mm": 100, "cols": 30, "rows": 20}


def _form():
    c = P.defaults()
    form, _, _ = P.formation(c, c, P.shared_defaults())
    return form, P.shared_defaults()


def test_l_path():
    form, sh = _form()
    r = plan_path([[5, 3], [10, 3], [15, 3], [15, 10]], GRID, form, sh)
    assert r["ok"], r["errors"]
    p = r["path"]
    assert [s["len"] for s in p["segments"]] == [1000, 700]  # điểm thẳng hàng được gộp
    assert p["segments"][0]["turn_after"] == 90  # rẽ trái (y hướng lên)
    assert p["corners_s"] == [1000]
    assert p["follower"]["start"]["x"] == 500 - form["g_target"]


def test_reject_diagonal_and_reversal():
    form, sh = _form()
    assert not plan_path([[5, 3], [8, 6]], GRID, form, sh)["ok"]
    r = plan_path([[5, 3], [12, 3], [8, 3]], GRID, form, sh)
    assert not r["ok"] and any("180" in e for e in r["errors"])


def test_reject_short_segment_after_corner():
    form, sh = _form()
    r = plan_path([[5, 3], [12, 3], [12, 4]], GRID, form, sh)
    assert not r["ok"] and any("khoảng thoát" in e for e in r["errors"])


def test_reject_self_intersection():
    form, sh = _form()
    r = plan_path([[5, 5], [15, 5], [15, 10], [10, 10], [10, 2]], GRID, form, sh)
    assert not r["ok"] and any("tự cắt" in e for e in r["errors"])


def test_reject_closed_path():
    form, sh = _form()
    # Quay về đúng điểm đầu
    r = plan_path([[5, 3], [15, 3], [15, 10], [5, 10], [5, 3]], GRID, form, sh)
    assert not r["ok"] and any("khép kín" in e for e in r["errors"])
    # Điểm cuối chạm vào giữa đoạn đầu
    r = plan_path([[5, 3], [15, 3], [15, 10], [10, 10], [10, 3]], GRID, form, sh)
    assert not r["ok"] and any("khép kín" in e for e in r["errors"])


def test_follower_start_outside_map():
    form, sh = _form()
    r = plan_path([[1, 3], [10, 3]], GRID, form, sh)
    assert not r["ok"] and any("follower" in e for e in r["errors"])
