"""Independently measure recorded cube lift after the GPT episode has stopped."""

import argparse
import io
import json
from pathlib import Path

import h5py
import numpy as np
from PIL import Image, ImageDraw


def measure_lift(times, positions):
    times, positions = np.asarray(times), np.asarray(positions)
    baseline = positions[0]
    lift = positions[:, 2] - baseline[2]
    longest = 0.0
    began = None
    for stamp, height in zip(times, lift):
        if height >= 0.08:
            if began is None:
                began = stamp
            longest = max(longest, float(stamp - began))
        else:
            began = None
    return dict(
        lift_success=longest >= 2,
        maximum_lift_m=float(lift.max()),
        final_lift_m=float(lift[-1]),
        longest_hold_above_8cm_sim_seconds=longest,
        maximum_displacement_m=float(np.linalg.norm(positions - baseline, axis=1).max()),
        peak_sample_index=int(np.argmax(lift)),
    )


def evaluate(directory):
    directory = Path(directory)
    report_path = directory / "result.json"
    report = json.loads(report_path.read_text())
    with h5py.File(report["recording"]["path"], "r") as file:
        if not file.attrs.get("complete"):
            raise ValueError("Cannot evaluate incomplete recording")
        times = file["state/sim_time"][:]
        objects = [json.loads(raw) for raw in file["state/object_states_json"][:]]
        positions = [item["/World/GraspCube"][:3] for item in objects]
        evaluation = measure_lift(times, positions)
        evaluation.update(
            success=False,
            visual_grasp_verified=False,
            object_truth_used_by_policy=False,
            verdict=(
                "抬升保持达到标准，待核对夹持画面" if evaluation["lift_success"] else "未达到抬高8厘米并保持2秒的标准"
            ),
        )
        names = [n.decode() if isinstance(n, bytes) else n for n in file["joint_names"][:]]
        evaluation["final_gripper_positions"] = {
            n: float(file["state/positions"][-1, i])
            for i, n in enumerate(names)
            if "finger" in n or n.endswith(("joint7", "joint8")) and not n.startswith("panda_joint")
        }
        metadata = json.loads(file.attrs["metadata_json"])
        frame_times = file["frames/sim_time"][:]
        samples = [0, evaluation["peak_sample_index"], len(times) - 1]
        sheet = Image.new("RGB", (640 * 3, (480 + 28) * len(report["cameras"])), "#182238")
        draw = ImageDraw.Draw(sheet)
        for row, name in enumerate(report["cameras"]):
            camera = metadata["cameras"].index(name)
            for col, sample in enumerate(samples):
                frame = int(np.argmin(abs(frame_times - times[sample])))
                raw = file[f"images/camera_{camera}"][frame].tobytes()
                with Image.open(io.BytesIO(raw)) as im:
                    sheet.paste(im.convert("RGB").resize((640, 480)), (640 * col, (480 + 28) * row + 28))
                draw.text(
                    (640 * col + 10, (480 + 28) * row + 6),
                    f"{name}: {('initial', 'max lift', 'final')[col]} | sim {frame_times[frame]:.3f}",
                    fill="white",
                )
        sheet.save(directory / "evaluation_contact.jpg")
    report["evaluation"] = evaluation
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(evaluation, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    evaluate(parser.parse_args().directory)
