"""Interface phần cứng (HAL). ESP32 và sim cài đặt cùng interface này.

C++ tương ứng: lib/hal_api (IMotor, IEncoder, IImu, IRange, IClock, IBattery, IStorage).
ToF không có hàm đọc ở đây: phần cứng/sim gọi VirtualFirmware.on_range() khi có mẫu mới
(trên ESP32 là task `range` khi chân INT báo dữ liệu sẵn sàng).
"""
from __future__ import annotations

from typing import Protocol


class Hal(Protocol):
    def now_ms(self) -> int: ...

    def set_motor(self, side: str, duty: float, brake: bool) -> None:
        """side 'l'|'r'; duty ∈ [-1, 1]; brake=True → phanh nhanh L298N (IN1=IN2=HIGH)."""

    def read_encoder(self, side: str) -> int:
        """Tổng tick (có dấu) từ PCNT."""

    def read_gyro_z(self) -> float:
        """rad/s thô (chưa trừ bias)."""

    def read_vbat(self) -> float: ...

    def storage_load(self) -> dict | None:
        """Đọc /cfg/config.json (LittleFS). None nếu chưa có."""

    def storage_save(self, data: dict) -> bool: ...

    def nvs_load(self) -> dict:
        """Danh tính, vai trò (NVS)."""

    def nvs_save(self, data: dict) -> None: ...
