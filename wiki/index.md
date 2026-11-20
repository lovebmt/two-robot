# Wiki index — two-robot

- [requirements.md](requirements.md): ràng buộc đã chốt C1–C5, yêu cầu chức năng F1–F22, phi chức năng N1–N8, tiêu chí nghiệm thu
- [architecture.md](architecture.md): phân chia xe/PC, phần cứng (L298N, GA25-370, ToF200C, MPU6050), ràng buộc GPIO, task firmware, primitive
- [protocol.md](protocol.md): beacon, HTTP API, WS lệnh/sự kiện, UDP state/relay, time sync, định dạng path
- [algorithms.md](algorithms.md): điều khiển bánh L298N, fusion gyro + encoder, KF gap bằng ToF, luật FOLLOW, quy trình góc cua 10 bước, hiệu chuẩn
- [web-ui.md](web-ui.md): 6 màn hình, planner, validator
- [parameters.md](parameters.md): toàn bộ khoá cấu hình và giá trị mặc định
- [open-questions.md](open-questions.md): câu hỏi đã chốt và còn mở

Liên quan: [../ops/roadmap.md](../ops/roadmap.md), [../ops/decisions.md](../ops/decisions.md), [../work/hardware/hardware-intake.md](../work/hardware/hardware-intake.md)
Nguồn: [brief](../raw/2026-11-20-brief.md), [trả lời đợt 1](../raw/2026-11-20-answers-1.md), [đợt 2](../raw/2026-11-20-answers-2.md)
