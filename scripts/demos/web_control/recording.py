"""Bounded asynchronous HDF5 episode writer; JPEG frames retain independent timestamps."""

import json
import queue
import threading
import time
import uuid
from pathlib import Path

import h5py
import numpy as np


class EpisodeWriter:
    def __init__(self, directory, metadata):
        self.id = uuid.uuid4().hex
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / f"episode_{self.id}.hdf5"
        self.queue = queue.Queue(maxsize=256)
        self.error = None
        self.closed = False
        self.count = 0
        self.frames = 0
        self.outcome = dict(success=False, reason="interrupted")
        self.metadata = metadata
        self.ready = threading.Event()
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()
        self.ready.wait(10)
        if not self.ready.is_set() or self.error:
            raise RuntimeError(self.error or "HDF5 writer startup timeout")

    def submit(self, kind, data):
        if self.closed or self.error:
            return
        try:
            self.queue.put_nowait((kind, data))
        except queue.Full:
            self.error = "Recording queue overflow; episode invalid (samples lost)"

    def status(self):
        return dict(
            id=self.id,
            path=str(self.path),
            closed=self.closed,
            samples=self.count,
            frames=self.frames,
            error=self.error,
        )

    def close(self, success=False, reason="user_stop"):
        self.outcome = dict(success=success, reason=reason)
        self.closed = True
        self.thread.join(timeout=30)
        if self.thread.is_alive():
            self.error = "Writer shutdown timeout; file may still be incomplete"
        return self.status()

    def _write(self):
        try:
            with h5py.File(self.path, "x") as f:
                f.attrs["schema"] = "isaaclab_robot_episode_v1"
                f.attrs["metadata_json"] = json.dumps(self.metadata, ensure_ascii=False, default=str)
                f.attrs["complete"] = False
                for key in ("instruction", "robot"):
                    f.attrs[key] = self.metadata[key]
                f.create_dataset("joint_names", data=self.metadata["joint_names"], dtype=h5py.string_dtype())
                self.ready.set()

                def append(name, value, dtype=None):
                    if name not in f:
                        shape = np.asarray(value).shape if dtype is None else ()
                        f.create_dataset(
                            name,
                            shape=(0, *shape),
                            maxshape=(None, *shape),
                            chunks=True,
                            dtype=dtype or np.asarray(value).dtype,
                        )
                    ds = f[name]
                    ds.resize(ds.shape[0] + 1, axis=0)
                    ds[-1] = value

                while not self.closed or not self.queue.empty():
                    try:
                        kind, data = self.queue.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    if kind == "event":
                        append("events/json", json.dumps(data), h5py.string_dtype())
                    else:
                        for key in ("sim_time", "wall_time", "step", "positions", "velocities", "targets"):
                            append(f"state/{key}", data[key])
                        append(
                            "state/object_states_json", json.dumps(data.get("object_states", {})), h5py.string_dtype()
                        )
                        self.count += 1
                        if data["images"] is not None:
                            for key in ("sim_time", "wall_time", "step"):
                                append(f"frames/{key}", data[key])
                            for index, name in enumerate(self.metadata["cameras"]):
                                append(
                                    f"images/camera_{index}",
                                    np.frombuffer(data["images"][name], dtype=np.uint8),
                                    h5py.vlen_dtype(np.dtype("uint8")),
                                )
                            self.frames += 1
                    if self.count % 120 == 0:
                        f.flush()
                f.attrs["complete"] = self.error is None
                f.attrs["success"] = self.outcome["success"] and self.error is None
                f.attrs["end_reason"] = self.error or self.outcome["reason"]
                f.attrs["ended_at"] = time.time()
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            self.ready.set()


class RecordingControl:
    def start_recording(self, instruction):
        if not isinstance(instruction, str) or not instruction.strip() or len(instruction) > 8192:
            raise ValueError("instruction must be nonempty text, at most 8192 characters")
        with self.lock:
            if self.phase != "ready" or (self.recorder and self.recorder.thread.is_alive()):
                raise RuntimeError("Not ready or already recording")
            metadata = dict(
                self.recording_metadata,
                instruction=instruction,
                robot=self.robot_label,
                joint_names=self.names.copy(),
                cameras=[c["name"] for c in self.cameras],
                physics_dt=self.physics_dt,
                started_at=time.time(),
            )
            self.recorder = EpisodeWriter(self.recording_dir, metadata)
            return self.recorder.status()

    def recording_status(self):
        with self.lock:
            return None if self.recorder is None else self.recorder.status()

    def stop_recording(self, episode_id, success=False, reason="user_stop"):
        if type(success) is not bool or not isinstance(reason, str) or len(reason) > 1024:
            raise ValueError("Invalid recording outcome")
        with self.lock:
            recorder = self.recorder
            if recorder is None or recorder.id != episode_id:
                raise ValueError("Unknown recording")
            recorder.outcome = dict(success=success, reason=reason)
            recorder.closed = True
        return recorder.close(success, reason)

    def record_event(self, event):
        if self.recorder:
            self.recorder.submit("event", dict(sim_time=self.sim_time, wall_time=time.time(), **event))
