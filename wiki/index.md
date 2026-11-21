# Wiki index — two-robot

- [requirements.md](requirements.md): ràng buộc C1–C7, yêu cầu chức năng F1–F29, phi chức năng N1–N10, tiêu chí nghiệm thu
- [architecture.md](architecture.md): phân chia xe/PC, phần cứng (L298N, GA25-370 12 V, ToF200C, MPU6050), GPIO, cấu trúc firmware (core/app/proto/hal)
- [protocol.md](protocol.md): beacon, HTTP API, WS lệnh/sự kiện, UDP state/relay, time sync, định dạng path
- [algorithms.md](algorithms.md): điều khiển bánh, fusion gyro + encoder, KF gap bằng ToF, luật FOLLOW, quy trình góc cua 10 bước, hiệu chuẩn
- [motor-control.md](motor-control.md): motor mạnh yếu và ma sát khác nhau, giải pháp phân tầng L0–L7, wizard hiệu chuẩn
- [simulation.md](simulation.md): robot ảo cùng code và cùng giao thức, mô hình vật lý, chế độ interactive/batch/replay, view chung SIM/REAL, kịch bản và metric
- [config-management.md](config-management.md): GPIO cố định trong code và tham số động, schema + codegen, **firmware xử lý tham số lúc chạy** (ParamService, Derived, swap, bumpless), lưu flash, luồng Apply/Save; **§7 luồng tham số thực tế** (RAM ↔ flash, tham số riêng/chung, ⚠ PC đẩy tham số chung khi kết nối)
- [web-ui.md](web-ui.md): các màn hình, planner, validator
- [parameters.md](parameters.md): toàn bộ tham số động (không có GPIO) và giá trị mặc định
- [open-questions.md](open-questions.md): câu hỏi đã chốt và còn mở

Liên quan: [../ops/roadmap.md](../ops/roadmap.md), [../ops/decisions.md](../ops/decisions.md), [../work/hardware/hardware-intake.md](../work/hardware/hardware-intake.md)
Nguồn: [brief](../raw/2026-11-20-brief.md), [đợt 1](../raw/2026-11-20-answers-1.md), [đợt 2](../raw/2026-11-20-answers-2.md), [đợt 3](../raw/2026-11-20-answers-3.md)
