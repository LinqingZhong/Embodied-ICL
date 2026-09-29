"""Flange FK/IK from USD joint frames. NumPy/SciPy only after chain extraction."""

import importlib.util
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def vector(value, size, name):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def transform(position, quaternion):
    p = vector(position, 3, "position")
    q = vector(quaternion, 4, "quaternion wxyz")
    if abs(np.linalg.norm(q) - 1) > 0.001:
        raise ValueError("Quaternion must be unit length (wxyz)")
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    t[:3, 3] = p
    return t


def pose(t):
    q = Rotation.from_matrix(t[:3, :3]).as_quat()
    return dict(position=t[:3, 3].tolist(), quaternion=q[[3, 0, 1, 2]].tolist())


class ArmKinematics:
    def __init__(self, definition, joint_names, frames, limits):
        self.definition = definition
        self.joint_names = joint_names
        self.frames = frames
        self.limits = np.asarray(limits, dtype=float)
        self.tool = np.eye(4)
        self.tool[:3, 3] = definition["offset"]

    def matrix(self, q):
        q = vector(q, len(self.frames), "joint_positions")
        t = np.eye(4)
        for angle, (a, axis, b_inv) in zip(q, self.frames):
            r = np.eye(4)
            r[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            t = t @ a @ r @ b_inv
        return t @ self.tool

    def fk(self, q):
        return pose(self.matrix(q))

    def ik(self, position, quaternion, seed):
        target = transform(position, quaternion)
        seed = vector(seed, len(self.frames), "seed")
        low, high = self.limits.T
        if np.any(seed < low) or np.any(seed > high):
            raise ValueError("IK seed outside joint limits")

        def residual(q):
            actual = self.matrix(q)
            return np.r_[
                actual[:3, 3] - target[:3, 3], Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()
            ]

        result = least_squares(
            residual, seed, bounds=(low, high), max_nfev=300, method="dogbox", ftol=1e-9, xtol=1e-9, gtol=1e-9
        )
        error = residual(result.x)
        pe, re = float(np.linalg.norm(error[:3])), float(np.linalg.norm(error[3:]))
        if pe > 1e-4 or re > 1e-3:
            raise ValueError(f"IK did not converge: position_error={pe:.6g} m rotation_error={re:.6g} rad")
        return dict(joint_positions=result.x.tolist(), position_error=pe, rotation_error=re)

    def metadata(self):
        return {
            **self.definition,
            "joint_names": self.joint_names,
            "limits": self.limits.tolist(),
            "quaternion_order": "wxyz",
            "position_tolerance": 1e-4,
            "rotation_tolerance": 1e-3,
        }


def load_kinematics(robot_name, stage, robots, names, limits):
    from pxr import Usd, UsdPhysics

    locations = {"x5": "ARX/X5", "x5_dual": "ARX/X5", "cobot_magic": "AgileX/CobotMagic", "franka": "FrankaEmika"}
    if robot_name not in locations:
        return {}
    root = Path(__file__).resolve().parents[3] / "source/isaaclab_assets/data/Robots"
    path = root / locations[robot_name] / "kinematics.py"
    spec = importlib.util.spec_from_file_location("asset_kinematics", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = {}
    for definition in module.arms(robot_name == "x5_dual"):
        robot = robots[definition["instance"]]
        prim = stage.GetPrimAtPath(robot.cfg.prim_path)
        joints = {
            p.GetName(): UsdPhysics.RevoluteJoint(p) for p in Usd.PrimRange(prim) if p.IsA(UsdPhysics.RevoluteJoint)
        }
        frames = []
        previous = definition["base"]
        for name in definition["joints"]:
            j = joints[name]
            parent = j.GetBody0Rel().GetTargets()[0].name
            child = j.GetBody1Rel().GetTargets()[0].name
            if parent != previous:
                raise ValueError(f"Non-contiguous chain {name}: {parent} != {previous}")
            previous = child

            def local(index):
                pos = getattr(j, f"GetLocalPos{index}Attr")().Get()
                rot = getattr(j, f"GetLocalRot{index}Attr")().Get()
                return transform(list(pos), [rot.GetReal(), *rot.GetImaginary()])

            axis = np.eye(3)["XYZ".index(j.GetAxisAttr().Get())]
            frames.append((local(0), axis, np.linalg.inv(local(1))))
        if previous != definition["end_body"]:
            raise ValueError("Flange body does not match chain")
        joint_names = [f"{definition['instance']}/{n}" if robot_name == "x5_dual" else n for n in definition["joints"]]
        result[definition["name"]] = ArmKinematics(
            definition, joint_names, frames, [limits[names.index(n)] for n in joint_names]
        )
    return result
