"""two-robot: Web Control + mô phỏng.

- tworobot.params    : schema tham số dùng chung, validate, giá trị suy ra
- tworobot.fw        : "firmware ảo" — bản Python tham chiếu, sẽ port sang C++ (lib/core, lib/app)
- tworobot.sim       : mô hình vật lý + engine chạy robot ảo
- tworobot.(app, registry, links, coordinator, planner, hub) : Control Server
"""

__version__ = "0.1.0"
