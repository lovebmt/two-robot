"""ParamService: nhận tham số động lúc chạy (wiki/config-management.md §3).

patch → validate → derive → slot pending → control task swap ở đầu chu kỳ.
Mỗi chu kỳ điều khiển chỉ thấy một snapshot (active, derived) bất biến.
"""
from __future__ import annotations

from .. import params as P
from .hal import Hal


class ParamService:
    def __init__(self, hal: Hal):
        self.hal = hal
        self.flags: set[str] = set()
        base = P.defaults()
        self.rev = 1
        self.saved_crc: str | None = None
        stored = hal.storage_load()
        if stored and isinstance(stored.get("params"), dict):
            # Bỏ khoá lạ/khoá lỗi, giữ phần hợp lệ (schema có thể đã đổi phiên bản).
            known = {k: v for k, v in stored["params"].items() if k in P.PARAMS}
            new, errors = P.validate_patch(base, known)
            if errors:
                good = {k: v for k, v in known.items() if k not in errors}
                new, errors2 = P.validate_patch(base, good)
                if errors2:
                    new = base
                    self.flags.add("CFG_DEFAULT")
                else:
                    self.flags.add("CFG_PARTIAL")
            base = new
            self.rev = int(stored.get("rev", 1))
            self.saved_crc = P.crc(base)
        else:
            self.flags.add("CFG_DEFAULT")
        self.active: dict = base
        self.derived: dict = P.derive_robot(base)
        self._pending: tuple[dict, dict, set[str]] | None = None

    # ------------------------------------------------------------------
    def view(self) -> dict:
        """Cấu hình mới nhất (kể cả bản đang chờ swap)."""
        return self._pending[0] if self._pending else self.active

    def view_derived(self) -> dict:
        return self._pending[1] if self._pending else self.derived

    def dirty(self) -> bool:
        return P.crc(self.view()) != self.saved_crc

    def submit(self, patch: dict, running: bool) -> dict:
        base = self.view()
        new, errors = P.validate_patch(base, patch)
        if errors:
            return {"ok": False, "errors": errors}
        if running:
            idle = P.idle_keys_changed(base, new)
            if idle:
                return {"ok": False, "busy": True,
                        "errors": {k: "chỉ đổi được khi xe đứng yên" for k in idle}}
            if not new["safety.allow_tune_while_running"] or not base["safety.allow_tune_while_running"]:
                return {"ok": False, "busy": True, "errors": {"*": "đang chạy, không cho chỉnh live"}}
        groups = P.changed_groups(base, new)
        if not groups:
            return {"ok": True, "cfg_rev": self.rev + (1 if self._pending else 0), "changed": []}
        self._pending = (new, P.derive_robot(new), groups)
        return {"ok": True, "cfg_rev": self.rev + 1, "changed": sorted(groups)}

    def swap_if_pending(self) -> set[str] | None:
        """Gọi ở đầu mỗi chu kỳ control. Trả các nhóm vừa đổi (hoặc None)."""
        if self._pending is None:
            return None
        new, der, groups = self._pending
        self._pending = None
        self.active, self.derived = new, der
        self.rev += 1
        self.flags.discard("CFG_DEFAULT")
        self.flags.discard("CFG_PARTIAL")
        return groups

    def save(self) -> dict:
        cfg = self.view()
        rev = self.rev + (1 if self._pending else 0)
        ok = self.hal.storage_save({"schema_ver": P.SCHEMA["schema_ver"], "rev": rev, "params": cfg})
        if ok:
            self.saved_crc = P.crc(cfg)
            # Người dùng đã xác nhận cấu hình (Save) → không còn là "mặc định chưa kiểm tra".
            self.flags.discard("CFG_DEFAULT")
            self.flags.discard("CFG_PARTIAL")
        return {"ok": ok, "cfg_rev": rev, "crc": self.saved_crc}

    def reload(self, running: bool) -> dict:
        stored = self.hal.storage_load()
        if not stored:
            return {"ok": False, "errors": {"*": "flash chưa có cấu hình"}}
        return self.submit(stored.get("params", {}), running)

    def factory_reset(self, running: bool) -> dict:
        return self.submit(P.defaults(), running)
