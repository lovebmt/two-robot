# two-robot — 2 xe ESP32 đi theo đường vẽ sẵn và bám nhau

Hai xe vi sai 3 bánh (2 bánh động cơ có encoder, 1 bánh nhỏ phía trước để cân bằng) chạy ESP32.
Mỗi xe có 2 cảm biến khoảng cách ở đầu xe. Một **Web Control chạy trên PC** dò tìm 2 xe trong mạng LAN.
Người dùng chọn xe nào là **đầu (leader)**, xe nào là **đuôi (follower)**, vẽ đường đi trên lưới rồi đặt khoảng cách giữa 2 xe.
Web Control tính toán đường đi và gửi xuống 2 xe. Xe đuôi bám theo xe đầu bằng bộ lọc kết hợp:
**đường đi biết trước + odometry + 2 cảm biến khoảng cách + trạng thái xe đầu gửi qua radio**.

> Trạng thái: **giai đoạn lập kế hoạch**, chưa có code.

## Đọc theo thứ tự

1. [wiki/requirements.md](wiki/requirements.md): yêu cầu chức năng và phi chức năng
2. [wiki/architecture.md](wiki/architecture.md): phần cứng, phần mềm, giao tiếp, giao thức
3. [wiki/algorithms.md](wiki/algorithms.md): ước lượng, điều khiển, giao thức qua góc cua (phần khó nhất)
4. [wiki/parameters.md](wiki/parameters.md): toàn bộ thông số cấu hình trên web
5. [wiki/open-questions.md](wiki/open-questions.md): câu hỏi cần chốt kèm giả định mặc định
6. [ops/roadmap.md](ops/roadmap.md): các mốc và đầu việc
7. [ops/decisions.md](ops/decisions.md): nhật ký quyết định kỹ thuật

## Cấu trúc repo

```
raw/            brief gốc (không sửa)
wiki/           tài liệu kỹ thuật (index.md, log.md)
ops/            roadmap, quyết định, trạng thái task
work/
  firmware/     PlatformIO, ESP32 (lib/core là C++ thuần, test được trên PC)
  control-web/  server PC (Python FastAPI) và frontend (Vite + TS + Canvas)
  sim/          mô phỏng 2 xe trên PC dùng chung lõi thuật toán
  hardware/     BOM, sơ đồ chân, sơ đồ đi dây, bản vẽ thân xe
```
