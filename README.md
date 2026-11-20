# two-robot — 2 xe ESP32 đi theo đường vẽ sẵn và bám nhau

Hai xe vi sai 3 bánh chạy ESP32 DevKit: 2 bánh động cơ GA25-370 12 V có encoder, driver L298N, 1 bánh nhỏ phía trước để cân bằng.
Mỗi xe có 2 cảm biến ToF200C (VL53L0X) ở đầu xe và 1 IMU MPU6050.
Thuật toán được phát triển trước trên **mô phỏng 2 robot ảo** dùng chính code firmware. **Cùng một view** hiển thị sim, xe thật realtime và replay.
GPIO cố định trong code (`pins.h`). Kích thước xe, khoảng cách và các hệ số **nhập trên web, đẩy xuống ESP32**. Firmware áp dụng chúng lúc chạy và lưu flash. Một **Web Control chạy trên PC** dò tìm 2 xe trong mạng LAN.
Người dùng chọn xe nào là **đầu (leader)**, xe nào là **đuôi (follower)**, vẽ đường đi trên lưới (chỉ góc 90°) rồi đặt khoảng cách giữa 2 xe.
Mọi liên lạc giữa 2 xe đi qua PC.
- **Đoạn thẳng:** follower tự bám leader bằng bộ lọc kết hợp **đường biết trước + encoder + gyro + 2 ToF** (relay trạng thái leader qua PC chỉ dùng làm phụ trợ).
- **Góc cua:** PC điều phối quy trình stop-and-go 10 bước. Follower đo ToF làm mốc trước khi leader quay, nên follower quay đúng chỗ leader đã quay.

> Trạng thái: **giai đoạn lập kế hoạch**, chưa có code.

## Đọc theo thứ tự

1. [wiki/requirements.md](wiki/requirements.md): yêu cầu chức năng và phi chức năng
2. [wiki/architecture.md](wiki/architecture.md): phần cứng, phần mềm, giao tiếp, giao thức
3. [wiki/algorithms.md](wiki/algorithms.md): ước lượng, điều khiển, giao thức qua góc cua (phần khó nhất)
4. [wiki/motor-control.md](wiki/motor-control.md): motor mạnh yếu, ma sát khác nhau
5. [wiki/simulation.md](wiki/simulation.md): mô phỏng 2 robot và view chung
6. [wiki/config-management.md](wiki/config-management.md): cấu hình động web → flash
7. [wiki/protocol.md](wiki/protocol.md): discovery, HTTP/WS/UDP, time sync, định dạng path
8. [wiki/web-ui.md](wiki/web-ui.md): các màn hình web, planner, validator
9. [wiki/parameters.md](wiki/parameters.md): toàn bộ thông số cấu hình
10. [wiki/open-questions.md](wiki/open-questions.md): câu hỏi đã chốt và còn mở
11. [work/hardware/hardware-intake.md](work/hardware/hardware-intake.md): số đo phần cứng và GPIO
12. [ops/roadmap.md](ops/roadmap.md): các mốc và đầu việc (sim trước)
13. [ops/decisions.md](ops/decisions.md): nhật ký quyết định kỹ thuật

## Cấu trúc repo

```
raw/            brief gốc (không sửa)
wiki/           tài liệu kỹ thuật (index.md, log.md)
ops/            roadmap, quyết định, trạng thái task
work/
  firmware/     PlatformIO, ESP32 (lib/core là C++ thuần, test được trên PC)
  control-web/  server PC (Python FastAPI) và frontend (Vite + TS + Canvas)
  sim/          robot ảo (code firmware + HAL sim) và mô phỏng vật lý (C++/CMake)
  shared/       schema config/telemetry/message và codegen
  hardware/     BOM, sơ đồ chân, sơ đồ đi dây, bản vẽ thân xe
```
