#!/usr/bin/env python3
"""Replay LeRobot dataset actions to the MicroMani Isaac TCP control bridge."""

from __future__ import annotations

import argparse
import json
import socket
import time
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--repo-id", default="micromani/isaac_micro_bimanual")
    parser.add_argument("--host", default="117.50.205.12")
    parser.add_argument("--port", type=int, default=49011)
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--max-frames", type=int, default=0, help="0 means replay the whole episode")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--report", default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(args.repo_id, root=Path(args.dataset_root), episodes=[args.episode], return_uint8=True)
    fps = int(dataset.fps)
    count = len(dataset) if args.max_frames <= 0 else min(len(dataset), args.max_frames)
    sleep_s = (1.0 / fps) / max(args.speed, 1e-6)
    sent = 0
    acks = 0
    errors = []
    started_at = time.time()

    with socket.create_connection((args.host, args.port), timeout=10) as sock:
        stream = sock.makefile("rwb")
        for index in range(count):
            item = dataset[index]
            action = np.asarray(item["action"], dtype=np.float32).reshape(-1)
            packet = {"axes": [float(v) for v in action], "source": "lerobot_replay", "frame_index": index}
            stream.write(json.dumps(packet).encode("utf-8") + b"\n")
            stream.flush()
            sent += 1
            ack_raw = stream.readline()
            if ack_raw:
                try:
                    ack = json.loads(ack_raw.decode("utf-8"))
                    if ack.get("ok"):
                        acks += 1
                except Exception as exc:
                    errors.append(f"bad ack at frame {index}: {exc}")
            time.sleep(sleep_s)

    report = {
        "dataset_root": str(Path(args.dataset_root).resolve()),
        "repo_id": args.repo_id,
        "host": args.host,
        "port": args.port,
        "episode": args.episode,
        "fps": fps,
        "frames_requested": count,
        "frames_sent": sent,
        "acks": acks,
        "errors": errors,
        "elapsed_s": time.time() - started_at,
    }
    if args.report:
        report_path = Path(args.report)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
