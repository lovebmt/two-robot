"""Firmware ảo (bản tham chiếu Python).

Mỗi module tương ứng một phần firmware C++ sau này:
  mathutil, control, gapkf, rangeproc  → lib/core
  paramsvc, primitives, robot          → lib/app
  hal                                  → lib/hal_api (interface)
Quy ước khi port: chỉ dùng phép tính vô hướng, không phụ thuộc thư viện ngoài,
mọi hằng số vật lý/hệ số đọc từ Config (ParamService), không viết cứng.
"""
