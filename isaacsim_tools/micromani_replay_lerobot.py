#!/usr/bin/env python3
"""Replay LeRobot dataset actions to the MicroMani Isaac TCP control bridge."""

from __future__ import annotations

import argparse
import json
import socket
import time
from pathlib import Path

import numpy as np
from micromani_sim_contract import SimContract, LEGACY_VERSION, VERSION, validate_action


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--repo-id", default="micromani/isaac_micro_bimanual")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--transport", choices=("tcp", "udp"), default="tcp")
    parser.add_argument("--allow-legacy-model-si", action="store_true", help="Explicitly declare old datasets as model-side m/rad/single-finger-m")
    parser.add_argument("--port", type=int, default=49011)
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--max-frames", type=int, default=0, help="0 means replay the whole episode")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--report", default=None)
    return parser.parse_args()


def load_replay_contract(root, allow_legacy=False):
    info = json.loads((Path(root) / "meta/info.json").read_text())
    metadata = info.get("dataContract")
    if metadata is None and allow_legacy:
        return None
    if not isinstance(metadata, dict) or metadata.get("version") not in (LEGACY_VERSION, VERSION):
        raise ValueError("Missing/unsupported contract; legacy SI datasets require explicit opt-in")
    return SimContract.from_metadata(metadata)


def main() -> None:
    args = parse_args()
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    contract = load_replay_contract(args.dataset_root, args.allow_legacy_model_si)
    dataset = LeRobotDataset(args.repo_id, root=Path(args.dataset_root), episodes=[args.episode], return_uint8=True)
    fps = int(dataset.fps)
    count = len(dataset) if args.max_frames <= 0 else min(len(dataset), args.max_frames)
    sleep_s = (1.0 / fps) / max(args.speed, 1e-6)
    sent = 0
    acks = 0
    errors = []
    started_at = time.time()

    # 所有动作先验证，再连接执行端，避免中途发现格式错而已执行前半段。
    actions = []
    for index in range(count):
        values = np.asarray(dataset[index]["action"], dtype=np.float64).reshape(-1).tolist()
        actions.append(contract.to_model(values) if contract else validate_action(values))
    connection = (socket.create_connection((args.host, args.port), timeout=10)
                  if args.transport == "tcp" else socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
    with connection as sock:
        stream = sock.makefile("rwb") if args.transport == "tcp" else None
        for index in range(count):
            packet = {"axes": actions[index], "source": "lerobot_replay", "frame_index": index}
            payload = json.dumps(packet).encode("utf-8") + b"\n"
            if stream is None:
                sock.sendto(payload, (args.host, args.port))
                sent += 1
            else:
                stream.write(payload)
                stream.flush()
                sent += 1
                ack_raw = stream.readline()
                try:
                    ack = json.loads(ack_raw.decode("utf-8"))
                    if not ack.get("ok"):
                        raise RuntimeError(str(ack.get("error", "command rejected")))
                    acks += 1
                except Exception as exc:
                    errors.append(f"ack failed at frame {index}: {exc}")
                    break
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
        "transport": args.transport,
        "ack_semantics": "queue_acceptance_not_physical_execution" if args.transport == "tcp" else "none",
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
