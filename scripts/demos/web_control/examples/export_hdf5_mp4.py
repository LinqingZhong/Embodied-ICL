"""Export episode JPEG streams to an H.264 MP4 montage using simulation timestamps."""

import argparse
import io
import json
import math
import shutil
import subprocess
from pathlib import Path

import h5py
import numpy as np
from PIL import Image, ImageDraw


def export(episode, output, camera=None, fps=30):
    if not 1 <= fps <= 120:
        raise ValueError("fps must be between 1 and 120")
    if not shutil.which("ffmpeg"):
        raise RuntimeError("ffmpeg is required")
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name + ".partial.mp4")
    if temporary.exists():
        raise FileExistsError(temporary)
    with h5py.File(episode, "r") as f:
        if not f.attrs.get("complete", False):
            raise ValueError("Episode is incomplete")
        metadata = json.loads(f.attrs["metadata_json"])
        names = metadata["cameras"]
        selected = list(range(len(names))) if camera is None else [names.index(camera)]
        times = f["frames/sim_time"][:]
        if len(times) < 2 or np.any(np.diff(times) <= 0):
            raise ValueError("Need at least two frames with strictly increasing simulation timestamps")
        first = Image.open(io.BytesIO(f[f"images/camera_{selected[0]}"][0].tobytes()))
        width, height = first.size
        columns = min(2, len(selected))
        rows = math.ceil(len(selected) / columns)
        size = (math.ceil(width * columns / 2) * 2, math.ceil((height + 28) * rows / 2) * 2)
        # CFR video: select the latest actual frame at each output time, never interpolate image content.
        duration = times[-1] - times[0] + float(np.median(np.diff(times)))
        schedule = times[0] + np.arange(round(duration * fps)) / fps
        indices = np.clip(np.searchsorted(times, schedule + 1e-8, side="right") - 1, 0, len(times) - 1)
        command = [
            "ffmpeg",
            "-v",
            "error",
            "-n",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-s",
            f"{size[0]}x{size[1]}",
            "-r",
            str(fps),
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-crf",
            "18",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(temporary),
        ]
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            for index in indices:
                canvas = Image.new("RGB", size, "#202020")
                draw = ImageDraw.Draw(canvas)
                for tile, c in enumerate(selected):
                    raw = f[f"images/camera_{c}"][int(index)].tobytes()
                    with Image.open(io.BytesIO(raw)) as im:
                        frame = im.convert("RGB")
                        if frame.size != (width, height):
                            raise ValueError("Camera resolutions differ")
                        x, y = (tile % columns) * width, (tile // columns) * (height + 28)
                        canvas.paste(frame, (x, y + 28))
                        draw.text((x + 6, y + 6), f"{names[c]} | sim {times[index]:.3f}s", fill="white")
                process.stdin.write(canvas.tobytes())
            process.stdin.close()
            error = process.stderr.read().decode()
            if process.wait() != 0:
                raise RuntimeError(error)
            temporary.rename(output)
        except BaseException:
            process.kill()
            process.wait()
            temporary.unlink(missing_ok=True)
            raise
    report = dict(
        episode=str(episode),
        output=str(output),
        cameras=[names[c] for c in selected],
        source_frames=len(times),
        video_frames=len(indices),
        fps=fps,
        duration=len(indices) / fps,
    )
    output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode")
    parser.add_argument("--output", required=True)
    parser.add_argument("--camera", help="One camera name; omitted means tiled montage")
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()
    print(json.dumps(export(args.episode, args.output, args.camera, args.fps), indent=2))
