"""Opt-in per-arm FK/IK accuracy and flange chunk motion verification."""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_client import RobotClient

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--robot", required=True)
parser.add_argument("--output", required=True)
parser.add_argument("--tracking-tolerance", type=float, default=None)
args = parser.parse_args()
r = RobotClient("http://127.0.0.1:8083", timeout=90)
reports = {}
for name, meta in r.get_arms().items():
    state = r.get_joint_state()
    q0 = np.array([state["positions"][state["joint_names"].index(n)] for n in meta["joint_names"]])
    limits = np.array(meta["limits"])
    errors = []
    rng = np.random.default_rng(42)
    for sample in range(12):
        q = np.clip(q0 + rng.uniform(-0.15, 0.15, len(q0)), limits[:, 0] + 0.001, limits[:, 1] - 0.001)
        target = r.fk(name, q.tolist())
        exact = r.ik(name, target["position"], target["quaternion"], q.tolist())
        assert np.max(np.abs(np.array(exact["joint_positions"]) - q)) < 1e-6
        seed = np.clip(q + rng.uniform(-0.025, 0.025, len(q)), limits[:, 0] + 0.001, limits[:, 1] - 0.001)
        solved = r.ik(name, target["position"], target["quaternion"], seed.tolist())
        rebuilt = r.fk(name, solved["joint_positions"])
        pe = float(np.linalg.norm(np.array(rebuilt["position"]) - target["position"]))

        def rot(p):
            w, x, y, z = p["quaternion"]
            return Rotation.from_quat([x, y, z, w])

        re = float((rot(target) * rot(rebuilt).inv()).magnitude())
        assert pe < 1e-4 and re < 1e-3
        errors.append(
            dict(
                position_m=pe, rotation_rad=re, joint_rad=float(np.max(np.abs(np.array(solved["joint_positions"]) - q)))
            )
        )
    target_q = q0.copy()
    target_q[0] += 0.02
    p = r.fk(name, target_q.tolist())
    task = r.execute_eef_chunk(name, [p["position"]], [p["quaternion"]], 120)
    result = r.wait(task.id, timeout=90)
    assert result.status == "completed"
    actual = r.fk(name)
    tracking = float(np.linalg.norm(np.array(actual["position"]) - p["position"]))
    print("TRACKING", name, tracking, result.details, flush=True)
    # Tracking is a dynamics measurement, not the FK/IK acceptance criterion.
    assert np.isfinite(tracking)
    if args.tracking_tolerance is not None:
        assert tracking < args.tracking_tolerance, tracking
    back = r.fk(name, q0.tolist())
    task = r.execute_eef_chunk(name, [back["position"]], [back["quaternion"]], 120)
    assert r.wait(task.id, timeout=90).status == "completed"
    physics_errors = []
    # Independent physics-vs-FK comparison at three physically executed postures.
    for shift in (-0.03, 0.025, 0):
        target_q = q0.copy()
        target_q[0] += shift
        jt = r.execute_joint_chunk(meta["joint_names"], [target_q.tolist()], 60)
        assert r.wait(jt.id, timeout=90).status == "completed"
        snapshot = r.get_joint_state()
        measured = snapshot["eef_poses"][name]
        actual_q = [snapshot["positions"][snapshot["joint_names"].index(n)] for n in meta["joint_names"]]
        predicted = r.fk(name, actual_q)
        pe = float(np.linalg.norm(np.array(predicted["position"]) - measured["position"]))
        re = float((rot(predicted) * rot(measured).inv()).magnitude())
        assert pe < 1e-4 and re < 1e-3, (pe, re)
        target_pose = r.fk(name, target_q.tolist())
        tracking_pe = float(np.linalg.norm(np.array(target_pose["position"]) - measured["position"]))
        tracking_re = float((rot(target_pose) * rot(measured).inv()).magnitude())
        if args.tracking_tolerance is not None:
            assert tracking_pe < args.tracking_tolerance, tracking_pe
            assert tracking_re < 0.01, tracking_re
        physics_errors.append(
            dict(position_m=pe, rotation_rad=re, tracking_position_m=tracking_pe, tracking_rotation_rad=tracking_re)
        )
    reports[name] = dict(samples=errors, tracking_position_m=tracking, physics_errors=physics_errors)
report = dict(robot=args.robot, arms=reports, passed=True)
Path(args.output).write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report), flush=True)
