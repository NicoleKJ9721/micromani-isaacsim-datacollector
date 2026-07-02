import json
import queue
import socket
import threading
import time
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.types import ArticulationAction


ARTICULATION_ROOT = "/micromanipulator/root_joint"
STAGE_PATH = "/isaac-sim/.local/share/ov/data/micromani/micromanipulator_scene.usd"
UDP_HOST = "0.0.0.0"
UDP_PORT = 49010
TCP_HOST = "0.0.0.0"
TCP_PORT = 49011
STATUS_PATH = Path("/isaac-sim/.local/share/ov/data/micromani/micromani_bridge_status.json")

CONTROL_ORDER = [
    "joint1_left",
    "joint2_left",
    "joint3_left",
    "joint4_left",
    "joint5_left",
    "joint6_left",
    "left_gripper",
    "joint1_right",
    "joint2_right",
    "joint3_right",
    "joint4_right",
    "joint5_right",
    "joint6_right",
    "right_gripper",
]

GRIPPER_MAP = {
    "left_gripper": ["joint_finger1_left", "joint_finger2_left"],
    "right_gripper": ["joint_finger1_right", "joint_finger2_right"],
}


class MicroManiControlBridge:
    def __init__(self):
        self.command_queue = queue.Queue(maxsize=8)
        self.robot = None
        self.controller = None
        self.dof_names = []
        self.dof_index = {}
        self.packet_count = 0
        self.last_error = None
        self.last_packet = None
        self.last_source = None
        self.last_apply_time = None
        self.started_at = time.time()
        self.stage_open_requested = False
        self._stop = threading.Event()
        self._udp_thread = threading.Thread(target=self._udp_loop, name="micromani-udp", daemon=True)
        self._tcp_thread = threading.Thread(target=self._tcp_loop, name="micromani-tcp", daemon=True)
        self._subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._on_update, name="micromani_control_bridge"
        )
        self._udp_thread.start()
        self._tcp_thread.start()
        self._write_status()
        print(f"[micromani] control bridge listening on UDP {UDP_PORT} and TCP {TCP_PORT}")

    def _udp_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((UDP_HOST, UDP_PORT))
        sock.settimeout(0.5)
        while not self._stop.is_set():
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except Exception as exc:
                self.last_error = f"udp: {exc}"
                continue
            self._submit_packet(data, f"udp:{addr[0]}:{addr[1]}")

    def _tcp_loop(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((TCP_HOST, TCP_PORT))
        server.listen(4)
        server.settimeout(0.5)
        while not self._stop.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except Exception as exc:
                self.last_error = f"tcp: {exc}"
                continue
            threading.Thread(target=self._tcp_client, args=(conn, addr), daemon=True).start()

    def _tcp_client(self, conn, addr):
        with conn:
            conn_file = conn.makefile("rwb")
            for raw_line in conn_file:
                self._submit_packet(raw_line, f"tcp:{addr[0]}:{addr[1]}")
                try:
                    conn_file.write(json.dumps({"ok": True, "time": time.time()}).encode("utf-8") + b"\n")
                    conn_file.flush()
                except Exception:
                    break

    def _submit_packet(self, data, source):
        try:
            packet = json.loads(data.decode("utf-8"))
            if not isinstance(packet, dict):
                raise ValueError("packet must be a JSON object")
            while self.command_queue.full():
                self.command_queue.get_nowait()
            self.command_queue.put_nowait((packet, source, time.time()))
            self.packet_count += 1
            self.last_packet = packet
            self.last_source = source
        except Exception as exc:
            self.last_error = f"packet: {exc}"

    def _initialize_robot(self):
        if self.robot is not None:
            return
        stage = omni.usd.get_context().get_stage()
        if stage is None or not stage.GetPrimAtPath(ARTICULATION_ROOT).IsValid():
            if not self.stage_open_requested:
                omni.usd.get_context().open_stage(STAGE_PATH)
                self.stage_open_requested = True
            raise RuntimeError(f"waiting for stage prim {ARTICULATION_ROOT}")
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            timeline.play()
            raise RuntimeError("waiting for physics timeline")
        self.robot = Articulation(ARTICULATION_ROOT)
        self.robot.initialize()
        if not self.robot.is_physics_handle_valid():
            self.robot = None
            raise RuntimeError("waiting for physics handle")
        dof_names = getattr(self.robot, "dof_names")
        self.dof_names = list(dof_names() if callable(dof_names) else dof_names)
        if hasattr(self.robot, "set_joint_positions"):
            self.controller = self.robot
        elif hasattr(self.robot, "get_articulation_controller"):
            self.controller = self.robot.get_articulation_controller()
        elif hasattr(self.robot, "apply_action"):
            self.controller = self.robot
        else:
            self.robot = None
            raise RuntimeError("articulation has no action interface")
        self.dof_index = {name: i for i, name in enumerate(self.dof_names)}
        print(f"[micromani] articulation initialized: {ARTICULATION_ROOT}, dofs={self.dof_names}")
        self._write_status()

    def _packet_to_command(self, packet):
        targets = {}
        if "axes" in packet:
            axes = packet["axes"]
            if len(axes) != len(CONTROL_ORDER):
                raise ValueError(f"axes length must be {len(CONTROL_ORDER)}")
            targets.update(dict(zip(CONTROL_ORDER, [float(v) for v in axes])))
        if "joints" in packet:
            targets.update({str(k): float(v) for k, v in packet["joints"].items()})

        expanded = {}
        for name, value in targets.items():
            if name in GRIPPER_MAP:
                for raw_name in GRIPPER_MAP[name]:
                    expanded[raw_name] = value
            else:
                expanded[name] = value

        joint_indices = []
        joint_positions = []
        unknown = []
        for name, value in expanded.items():
            if name not in self.dof_index:
                unknown.append(name)
                continue
            joint_indices.append(self.dof_index[name])
            joint_positions.append(value)
        if unknown:
            raise ValueError(f"unknown joints: {unknown}")
        if not joint_indices:
            return None
        return np.asarray(joint_positions, dtype=np.float32), np.asarray(joint_indices, dtype=np.int32)

    def _on_update(self, _event):
        try:
            self._initialize_robot()
        except Exception as exc:
            self.last_error = f"init: {exc}"
            self._write_status(throttle=True)
            return

        latest = None
        while True:
            try:
                latest = self.command_queue.get_nowait()
            except queue.Empty:
                break
        if latest is None:
            self._write_status(throttle=True)
            return

        packet, _source, _received_at = latest
        try:
            command = self._packet_to_command(packet)
            if command is None:
                return
            joint_positions, joint_indices = command
            if hasattr(self.robot, "set_joint_positions"):
                try:
                    self.robot.set_joint_positions(joint_positions, joint_indices=joint_indices)
                except Exception:
                    self.robot.set_joint_positions(np.expand_dims(joint_positions, axis=0), joint_indices=joint_indices)
            else:
                action = ArticulationAction(joint_positions=joint_positions, joint_indices=joint_indices)
                self.controller.apply_action(action)
            self.last_apply_time = time.time()
            self.last_error = None
        except Exception as exc:
            self.last_error = f"apply: {exc}"
        self._write_status()

    def _write_status(self, throttle=False):
        now = time.time()
        if throttle and getattr(self, "_last_status_write", 0.0) + 1.0 > now:
            return
        self._last_status_write = now
        STATUS_PATH.parent.mkdir(parents=True, exist_ok=True)
        STATUS_PATH.write_text(
            json.dumps(
                {
                    "started_at": self.started_at,
                    "time": now,
                    "articulation_root": ARTICULATION_ROOT,
                    "stage_path": STAGE_PATH,
                    "udp_port": UDP_PORT,
                    "tcp_port": TCP_PORT,
                    "packet_count": self.packet_count,
                    "last_source": self.last_source,
                    "last_apply_time": self.last_apply_time,
                    "last_error": self.last_error,
                    "dof_names": self.dof_names,
                    "control_order": CONTROL_ORDER,
                    "gripper_map": GRIPPER_MAP,
                },
                indent=2,
            ),
            encoding="utf-8",
        )

    def stop(self):
        self._stop.set()
        self._subscription = None
        self._write_status()


BRIDGE = MicroManiControlBridge()
