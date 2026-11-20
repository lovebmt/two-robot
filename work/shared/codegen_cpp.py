#!/usr/bin/env python3
"""Sinh struct Config C++ + bảng metadata từ config.schema.json (nguồn chuẩn duy nhất).

    python3 work/shared/codegen_cpp.py
→ work/firmware/lib/core/src/config_gen.h / config_gen.cpp  (KHÔNG sửa tay)
"""
from __future__ import annotations

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCHEMA = HERE / "config.schema.json"
OUT = HERE.parent / "firmware" / "lib" / "core" / "src"


def ident(key: str) -> str:
    return key.replace(".", "_")


def cnum(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    s = repr(float(v))
    return (s if ("." in s or "e" in s) else s + ".0") + "f"


def main() -> None:
    sch = json.loads(SCHEMA.read_text(encoding="utf-8"))
    groups = [g["id"] for g in sch["groups"]]
    assert len(groups) <= 32
    params = sch["params"]
    ctype = {"float": "float", "int": "int32_t", "bool": "bool"}
    ptype = {"float": "PType::Float", "int": "PType::Int", "bool": "PType::Bool"}

    h = ["// AUTO-GENERATED từ work/shared/config.schema.json bởi work/shared/codegen_cpp.py — KHÔNG SỬA TAY",
         "#pragma once", "#include <stddef.h>", "#include <stdint.h>", "", "namespace tr {", "",
         f"constexpr int SCHEMA_VER = {sch['schema_ver']};", "",
         "enum class PType : uint8_t { Float, Int, Bool };", "",
         "// Bit nhóm tham số (ChangeSet)"]
    for i, g in enumerate(groups):
        h.append(f"constexpr uint32_t G_{g.upper()} = 1u << {i};")
    h += ["", "struct Config {"]
    for p in params:
        h.append(f"  {ctype[p['type']]} {ident(p['key'])} = {cnum(p['default'])};  // {p.get('unit', '')} {p['desc']}")
    h += ["};", "",
          "struct ParamMeta {",
          "  const char* key;", "  PType type;", "  uint16_t offset;", "  float min;", "  float max;",
          "  uint32_t group;", "  bool idle;    // chỉ đổi khi xe đứng yên", "  bool shared;  // tham số chung 2 xe",
          "};", "",
          "extern const ParamMeta PARAMS[];", "extern const size_t PARAM_COUNT;",
          "extern const char* const GROUP_NAMES[];", "extern const size_t GROUP_COUNT;", "",
          "}  // namespace tr", ""]

    c = ["// AUTO-GENERATED từ work/shared/config.schema.json bởi work/shared/codegen_cpp.py — KHÔNG SỬA TAY",
         '#include "config_gen.h"', "", "namespace tr {", "",
         "const ParamMeta PARAMS[] = {"]
    for p in params:
        lo = p.get("min", 0 if p["type"] != "bool" else 0)
        hi = p.get("max", 1 if p["type"] == "bool" else 0)
        c.append(f'  {{"{p["key"]}", {ptype[p["type"]]}, (uint16_t)offsetof(Config, {ident(p["key"])}), '
                 f'{cnum(float(lo))}, {cnum(float(hi))}, G_{p["group"].upper()}, '
                 f'{"true" if p["apply"] == "idle" else "false"}, {"true" if p.get("scope") == "shared" else "false"}}},')
    c += ["};", "const size_t PARAM_COUNT = sizeof(PARAMS) / sizeof(PARAMS[0]);", "",
          "const char* const GROUP_NAMES[] = {" + ", ".join(f'"{g}"' for g in groups) + "};",
          f"const size_t GROUP_COUNT = {len(groups)};", "", "}  // namespace tr", ""]

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "config_gen.h").write_text("\n".join(h), encoding="utf-8")
    (OUT / "config_gen.cpp").write_text("\n".join(c), encoding="utf-8")
    print(f"sinh {len(params)} tham số, {len(groups)} nhóm → {OUT}")


if __name__ == "__main__":
    main()
