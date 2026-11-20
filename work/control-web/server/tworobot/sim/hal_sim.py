"""HAL cho robot ảo: nối firmware ảo với Body trong World. "Flash" là file JSON."""
from __future__ import annotations

import json
import random
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .engine import SimEngine
    from .world import Body


class SimHal:
    def __init__(self, engine: "SimEngine", body: "Body", state_dir: Path | None, rng: random.Random):
        self.engine = engine
        self.body = body
        self.state_dir = state_dir
        self.rng = rng
        self._mem_cfg: dict | None = None
        self._mem_nvs: dict = {}

    def now_ms(self) -> int:
        return int(round(self.engine.t * 1000))

    def set_motor(self, side: str, duty: float, brake: bool) -> None:
        self.body.duty[side] = duty
        self.body.brake[side] = brake

    def read_encoder(self, side: str) -> int:
        return self.body.ticks(side)

    def read_gyro_z(self) -> float:
        return self.body.gyro(self.rng)

    def read_vbat(self) -> float:
        return self.body.p.vbat

    # ---- "flash" ----
    def _file(self, name: str) -> Path | None:
        if self.state_dir is None:
            return None
        d = self.state_dir / self.body.id
        d.mkdir(parents=True, exist_ok=True)
        return d / name

    def storage_load(self) -> dict | None:
        f = self._file("config.json")
        if f is None:
            return self._mem_cfg
        if not f.exists():
            return None
        try:
            return json.loads(f.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            bak = f.with_suffix(".bak")
            return json.loads(bak.read_text(encoding="utf-8")) if bak.exists() else None

    def storage_save(self, data: dict) -> bool:
        f = self._file("config.json")
        if f is None:
            self._mem_cfg = json.loads(json.dumps(data))
            return True
        tmp = f.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, indent=1, ensure_ascii=False), encoding="utf-8")
        json.loads(tmp.read_text(encoding="utf-8"))  # kiểm tra đọc lại
        if f.exists():
            f.replace(f.with_suffix(".bak"))
        tmp.replace(f)
        return True

    def nvs_load(self) -> dict:
        f = self._file("nvs.json")
        if f is None or not f.exists():
            return dict(self._mem_nvs)
        return json.loads(f.read_text(encoding="utf-8"))

    def nvs_save(self, data: dict) -> None:
        f = self._file("nvs.json")
        if f is None:
            self._mem_nvs = dict(data)
            return
        f.write_text(json.dumps(data), encoding="utf-8")
