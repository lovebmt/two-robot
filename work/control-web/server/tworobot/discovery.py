"""Dò robot thật trong LAN bằng HTTP (wiki/protocol.md §1): quét GET /api/info.

Mục tiêu nhận dạng:
  "192.168.1.0/24"        cả subnet (cổng 80)
  "192.168.1.42"          một máy
  "192.168.1.42:8080"     một máy + cổng
  "127.0.0.1:8101-8102"   dải cổng (dùng cho robot HTTP ảo)
"""
from __future__ import annotations

import asyncio
import ipaddress
import socket

import httpx

MAX_HOSTS = 1024


def local_subnet() -> str:
    """Subnet /24 của card mạng đang ra ngoài (không gửi gói nào, chỉ hỏi bảng định tuyến)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        ip = s.getsockname()[0]
    except OSError:
        ip = "127.0.0.1"
    finally:
        s.close()
    return str(ipaddress.ip_network(f"{ip}/24", strict=False))


def expand(targets: list[str]) -> list[str]:
    """→ danh sách base URL 'http://host[:port]'."""
    out: list[str] = []
    for t in targets:
        t = t.strip().removeprefix("http://").rstrip("/")
        if not t:
            continue
        if "/" in t:
            net = ipaddress.ip_network(t, strict=False)
            out += [f"http://{h}" for h in net.hosts()]
        elif ":" in t:
            host, port = t.rsplit(":", 1)
            if "-" in port:
                a, b = (int(x) for x in port.split("-"))
                out += [f"http://{host}:{p}" for p in range(a, b + 1)]
            else:
                out.append(f"http://{host}:{int(port)}")
        else:
            out.append(f"http://{t}")
        if len(out) > MAX_HOSTS:
            raise ValueError(f"quá nhiều địa chỉ (> {MAX_HOSTS})")
    return out


async def scan(targets: list[str], timeout: float = 0.4, concurrency: int = 64) -> list[tuple[str, dict]]:
    """Trả [(base_url, info)] cho các máy trả lời /api/info giống robot."""
    bases = expand(targets)
    found: list[tuple[str, dict]] = []
    sem = asyncio.Semaphore(concurrency)
    async with httpx.AsyncClient(timeout=httpx.Timeout(timeout)) as client:
        async def probe(base: str) -> None:
            async with sem:
                try:
                    r = await client.get(f"{base}/api/info")
                    info = r.json()
                except (httpx.HTTPError, ValueError):
                    return
                if isinstance(info, dict) and info.get("id") and "fw" in info:
                    found.append((base, info))
        await asyncio.gather(*(probe(b) for b in bases))
    return sorted(found)
