#!/usr/bin/env python3
"""Receive PICO/OpenXR UDP teleop packets and forward MicroMani 14D actions."""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from micromani_pico_mapping import PicoTeleopMapper  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=49100)
    parser.add_argument("--bridge-host", default="127.0.0.1")
    parser.add_argument("--bridge-port", type=int, default=49010)
    parser.add_argument("--transport", choices=("udp", "tcp"), default="udp")
    parser.add_argument("--status", default="/isaac-sim/.local/share/ov/data/micromani/micromani_pico_gateway_status.json")
    parser.add_argument("--timeout-s", type=float, default=0.5)
    return parser.parse_args()


def send_action_udp(sock: socket.socket, host: str, port: int, axes: list[float], sequence: int) -> None:
    payload = json.dumps({"source": "pico4", "sequence": sequence, "axes": axes}).encode("utf-8")
    sock.sendto(payload, (host, port))


def send_action_tcp(host: str, port: int, axes: list[float], sequence: int, timeout_s: float) -> dict:
    payload = json.dumps({"source": "pico4", "sequence": sequence, "axes": axes}).encode("utf-8") + b"\n"
    with socket.create_connection((host, port), timeout=timeout_s) as conn:
        conn.sendall(payload)
        conn.settimeout(timeout_s)
        line = conn.makefile("rb").readline()
    return json.loads(line.decode("utf-8")) if line else {"ok": False, "error": "empty ack"}


def write_status(path: Path, status: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(status, ensure_ascii=False, indent=2), encoding="utf-8")


def main() -> int:
    args = parse_args()
    status_path = Path(args.status)
    mapper = PicoTeleopMapper()
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    receiver.bind((args.listen_host, args.listen_port))
    receiver.settimeout(args.timeout_s)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    started_at = time.time()
    status = {
        "started_at": started_at,
        "time": started_at,
        "listen": {"host": args.listen_host, "port": args.listen_port},
        "bridge": {"host": args.bridge_host, "port": args.bridge_port, "transport": args.transport},
        "packets_received": 0,
        "packets_forwarded": 0,
        "last_source": None,
        "last_error": None,
        "last_axes": None,
    }
    write_status(status_path, status)
    print(
        f"[micromani-pico] listening on UDP {args.listen_host}:{args.listen_port}, "
        f"forwarding {args.transport} to {args.bridge_host}:{args.bridge_port}"
    )

    try:
        while True:
            try:
                data, addr = receiver.recvfrom(65535)
            except socket.timeout:
                status["time"] = time.time()
                write_status(status_path, status)
                continue

            try:
                packet = json.loads(data.decode("utf-8"))
                axes = mapper.map_packet(packet)
                sequence = int(packet.get("sequence", status["packets_received"]))
                if args.transport == "udp":
                    send_action_udp(sender, args.bridge_host, args.bridge_port, axes, sequence)
                    ack = None
                else:
                    ack = send_action_tcp(args.bridge_host, args.bridge_port, axes, sequence, args.timeout_s)
                    if not ack.get("ok"):
                        raise RuntimeError(f"bridge rejected packet: {ack}")
                status["packets_received"] += 1
                status["packets_forwarded"] += 1
                status["last_source"] = f"{addr[0]}:{addr[1]}"
                status["last_error"] = None
                status["last_axes"] = axes
                status["last_ack"] = ack
            except Exception as exc:
                status["last_error"] = str(exc)
            status["time"] = time.time()
            write_status(status_path, status)
    finally:
        receiver.close()
        sender.close()


if __name__ == "__main__":
    raise SystemExit(main())
