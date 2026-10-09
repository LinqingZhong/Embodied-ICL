"""Export a GPT recording with robot cameras, public outputs, receipts, and an explicit timeline.

Usage: python export_gpt_video.py RUN_DIRECTORY --output annotated.mp4
Long waits are shortened by default; --full retains the recorded wall-clock timeline.
Annotation pages explicitly freeze the cameras so every public trajectory row remains readable.
"""

import argparse
import io
import json
import math
import shutil
import subprocess
import tempfile
from pathlib import Path

import h5py
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

SIZE = (1920, 1080)
CAMERA_WIDTH = 960
PANEL_X = 982
PANEL_WIDTH = 908
LINES_PER_PAGE = 24
LINE_HEIGHT = 33
BACKGROUND = "#111a2c"
MUTED = "#b1bed6"
WHITE = "#edf3ff"
ACCENT = "#70e1c5"


def camera_indices(metadata, report):
    """Only use explicitly selected model cameras, even if recordings contain extra cameras."""
    names = report.get("cameras")
    available = metadata.get("cameras", [])
    if (
        not isinstance(names, list)
        or not names
        or len(set(names)) != len(names)
        or "scene" in names
        or any(name not in available for name in names)
    ):
        raise ValueError("result.json must select unique existing robot cameras; scene is excluded")
    if len(set(available)) != len(available):
        raise ValueError("Recording camera metadata contains duplicate names")
    return [available.index(name) for name in names]


def build_timeline(start, end, events, full=False, wait_seconds=1.0):
    """Partition source wall time, retaining every action through its receipt/error at 1x."""
    if not math.isfinite(start) or not math.isfinite(end) or end <= start:
        raise ValueError("Recording timestamps must span a positive finite duration")
    if full:
        return [dict(source_start=start, source_end=end, output_seconds=end - start, kind="recorded")]
    intervals = [(start, min(end, start + 1)), (max(start, end - 2), end)]
    for index, event in enumerate(events):
        if event["event"] != "action":
            continue
        finish = end
        for later in events[index + 1 :]:
            if (later["event"] == "result" and later.get("turn") == event.get("turn")) or later["event"] in (
                "error",
                "end",
            ):
                finish = float(later["wall_time"])
                break
        left, right = max(start, float(event["wall_time"]) - 0.5), min(end, finish + 1)
        if right > left:
            intervals.append((left, right))
    merged = []
    for left, right in sorted(intervals):
        if merged and left <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], right))
        else:
            merged.append((left, right))
    segments = []
    cursor = start
    for left, right in merged:
        if left > cursor:
            duration = left - cursor
            segments.append(
                dict(
                    source_start=cursor,
                    source_end=left,
                    output_seconds=min(wait_seconds, duration),
                    kind="shortened_wait" if duration > wait_seconds else "recorded",
                )
            )
        segments.append(dict(source_start=left, source_end=right, output_seconds=right - left, kind="recorded"))
        cursor = right
    return segments


def value_text(value, indent=0):
    """Readable JSON values: preserve precision and one complete numeric trajectory row per line."""
    prefix = " " * indent
    if isinstance(value, dict):
        lines = []
        for key, child in value.items():
            if isinstance(child, (dict, list)) and child:
                lines.extend([f"{prefix}{key}:", value_text(child, indent + 2)])
            else:
                lines.append(f"{prefix}{key}: {json.dumps(child, ensure_ascii=False)}")
        return "\n".join(lines)
    if isinstance(value, list):
        if all(not isinstance(item, (dict, list)) for item in value):
            return prefix + json.dumps(value, ensure_ascii=False)
        return "\n".join(value_text(item, indent) for item in value)
    return prefix + json.dumps(value, ensure_ascii=False)


def public_response_text(event):
    """Include function calls and public assistant text; omit private reasoning fields."""
    sections = []
    for item in event.get("output", []):
        if item.get("type") == "function_call":
            arguments = item.get("arguments", "")
            try:
                arguments = value_text(json.loads(arguments)) if isinstance(arguments, str) else value_text(arguments)
            except (TypeError, json.JSONDecodeError):
                arguments = str(arguments)
            sections.append(f"Tool: {item.get('name', '(unnamed)')}\n{arguments}")
        elif item.get("type") == "message":
            for part in item.get("content", []):
                if part.get("type") == "output_text":
                    sections.append(str(part.get("text", "")))
                elif part.get("type") == "refusal":
                    sections.append(str(part.get("refusal", "")))
    return "\n\n".join(sections) or "No public text or function call returned."


def load_fonts():
    candidates = [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]
    path = next((path for path in candidates if Path(path).is_file()), None)
    if path:
        return {key: ImageFont.truetype(path, size) for key, size in [("body", 23), ("small", 20), ("title", 31)]}
    return {key: ImageFont.load_default(size=size) for key, size in [("body", 23), ("small", 20), ("title", 31)]}


def wrap_text(text, font, width):
    lines = []
    for paragraph in str(text).splitlines() or [""]:
        line = ""
        for char in paragraph:
            if line and font.getlength(line + char) > width:
                lines.append(line)
                line = char
            else:
                line += char
        lines.append(line)
    return lines


def annotation_pages(events, report, font):
    pages = []
    response_turns = {event.get("turn") for event in events if event["event"] == "model_response"}
    for event in events:
        kind, turn = event["event"], event.get("turn", 0)
        if kind == "model_response":
            elapsed = event.get("elapsed_seconds")
            label = f"Public model output | turn {turn}"
            if isinstance(elapsed, (float, int)):
                label += f" | {elapsed:.2f}s response"
            text = public_response_text(event)
        elif kind == "action":
            if turn in response_turns:
                continue  # Keep the actual public output visible while its action executes.
            label = f"Action | turn {turn} | {event.get('action_type', 'action')}"
            text = value_text(event.get("action", {}))
        elif kind == "result":
            label, text = f"Execution receipt | turn {turn}", value_text(event.get("result", {}))
        elif kind == "error":
            label, text = f"Recorded error | turn {turn}", str(event.get("error", "Unknown error"))
        else:
            continue
        lines = wrap_text(text, font, PANEL_WIDTH)
        chunks = [lines[index : index + LINES_PER_PAGE] for index in range(0, len(lines), LINES_PER_PAGE)]
        for index, chunk in enumerate(chunks):
            pages.append(
                dict(
                    event=kind,
                    turn=turn,
                    source_wall_time=float(event["wall_time"]),
                    title=label,
                    page=index + 1,
                    pages=len(chunks),
                    text="\n".join(chunk),
                )
            )
    evaluation = report.get("evaluation") or {}
    final_text = "Evaluation recorded in result.json:\n" + value_text(
        evaluation or {"verdict": "Not independently evaluated"}
    )
    final_text += "\n\nRecorded error:\n" + str(report.get("error") or "None")
    final_text += "\n\nExecution receipts: " + str(sum(e["event"] == "result" for e in events))
    final_lines = wrap_text(final_text, font, PANEL_WIDTH)
    chunks = [final_lines[index : index + LINES_PER_PAGE] for index in range(0, len(final_lines), LINES_PER_PAGE)]
    for index, chunk in enumerate(chunks):
        pages.append(
            dict(
                event="final",
                turn=None,
                source_wall_time=None,
                title="Recorded outcome",
                page=index + 1,
                pages=len(chunks),
                text="\n".join(chunk),
            )
        )
    verdict = evaluation.get("verdict", "未独立评估")
    summary = [
        str(verdict),
        "",
        f"模型决策：{sum(e['event'] == 'result' for e in events)} 轮",
        "视觉输入：" + ", ".join(report.get("cameras", [])),
    ]
    if "maximum_lift_m" in evaluation:
        summary.append(f"实测最大抬升：{evaluation['maximum_lift_m'] * 100:.2f} 厘米")
    if "longest_hold_above_8cm_sim_seconds" in evaluation:
        summary.append(f"高于初始位置8厘米的保持：{evaluation['longest_hold_above_8cm_sim_seconds']:.2f} 仿真秒")
    finish = next((e for e in reversed(events) if e["event"] == "action" and "reason" in e.get("action", {})), None)
    if finish:
        summary.extend(["", "GPT 最终回答：", finish["action"]["reason"]])
    pages.append(
        dict(
            event="final",
            turn=None,
            source_wall_time=None,
            title="抓取结果与模型回答",
            page=1,
            pages=1,
            text="\n".join(wrap_text("\n".join(summary), font, PANEL_WIDTH)),
        )
    )
    return pages


class Renderer:
    def __init__(self, file, report, indices, times, sim_times, fonts):
        self.file, self.report, self.indices = file, report, indices
        self.times, self.sim_times, self.fonts = times, sim_times, fonts
        self.camera_cache = None
        self.camera_index = None

    def render(self, wall, page, mode, output_time):
        index = int(np.clip(np.searchsorted(self.times, wall, side="right") - 1, 0, len(self.times) - 1))
        if index != self.camera_index:
            self.camera_cache = Image.new("RGB", SIZE, BACKGROUND)
            draw = ImageDraw.Draw(self.camera_cache)
            count = len(self.indices)
            if count == 3:
                tiles = [(0, 132, 960, 548), (0, 680, 480, 364), (480, 680, 480, 364)]
            else:
                columns = 1 if count <= 2 else 2
                rows = math.ceil(count / columns)
                width, height = CAMERA_WIDTH // columns, 912 // rows
                tiles = [
                    ((tile % columns) * width, 132 + (tile // columns) * height, width, height) for tile in range(count)
                ]
            for tile, camera_index in enumerate(self.indices):
                x, y, width, height = tiles[tile]
                draw.text((x + 14, y + 2), self.report["cameras"][tile], font=self.fonts["small"], fill=MUTED)
                raw = self.file[f"images/camera_{camera_index}"][index].tobytes()
                with Image.open(io.BytesIO(raw)) as source:
                    fitted = ImageOps.contain(source.convert("RGB"), (width - 12, height - 34))
                    self.camera_cache.paste(
                        fitted, (x + (width - fitted.width) // 2, y + 31 + (height - 34 - fitted.height) // 2)
                    )
            self.camera_index = index
        canvas = self.camera_cache.copy()
        draw = ImageDraw.Draw(canvas)
        heading = f"{self.report.get('model', 'GPT')} | {self.report.get('robot', 'Robot')}"
        draw.text((22, 15), heading, font=self.fonts["title"], fill=WHITE)
        timing = (
            f"Source elapsed {wall - self.times[0]:.3f}s | capture {self.times[index] - self.times[0]:.3f}s"
            f" | sim {self.sim_times[index]:.3f}s | video {output_time:.2f}s"
        )
        draw.text((22, 59), timing, font=self.fonts["small"], fill=MUTED)
        draw.text((22, 95), mode, font=self.fonts["small"], fill=ACCENT)
        draw.rectangle((970, 132, 1910, 1043), fill="#1b2840")
        title = page["title"] if page else "Recorded cameras | waiting for the next model event"
        if page and page["pages"] > 1:
            title += f" | page {page['page']}/{page['pages']}"
        for line_index, line in enumerate(wrap_text(title, self.fonts["small"], PANEL_WIDTH)):
            draw.text((PANEL_X, 145 + line_index * 27), line, font=self.fonts["small"], fill=ACCENT)
        text = page["text"] if page else self.report.get("instruction", "")
        for line_index, line in enumerate(text.splitlines()[:LINES_PER_PAGE]):
            draw.text((PANEL_X, 221 + line_index * LINE_HEIGHT), line, font=self.fonts["body"], fill=WHITE)
        draw.text(
            (22, 1050),
            "Public model outputs and receipts | full evidence: trace.jsonl / result.json",
            font=self.fonts["small"],
            fill=MUTED,
        )
        return canvas


def export(directory, output, fps=12, full=False, page_seconds=3.0, final_seconds=5.0):
    if not 1 <= fps <= 60 or page_seconds <= 0 or final_seconds <= 0:
        raise ValueError("fps must be 1..60; annotation and final-page duration must be positive")
    if not shutil.which("ffmpeg"):
        raise RuntimeError("ffmpeg is required")
    directory, output = Path(directory), Path(output)
    targets = [
        output,
        output.with_suffix(".json"),
        output.with_suffix(".jpg"),
        output.with_name(output.stem + "_contact.jpg"),
    ]
    for target in targets:
        if target.exists():
            raise FileExistsError(target)
    report = json.loads((directory / "result.json").read_text(encoding="utf-8"))
    events = [
        json.loads(line)
        for line in (directory / "trace.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if any(not math.isfinite(float(event["wall_time"])) for event in events):
        raise ValueError("Trace timestamps must be finite")
    events.sort(key=lambda event: event["wall_time"])
    episode = Path(report["recording"]["path"])
    if not episode.is_absolute() and not episode.exists():
        episode = directory / episode
    output.parent.mkdir(parents=True, exist_ok=True)
    fonts = load_fonts()
    pages = annotation_pages(events, report, fonts["body"])
    with h5py.File(episode, "r") as file, tempfile.TemporaryDirectory(prefix=".gpt-video-", dir=output.parent) as tmp:
        if not file.attrs.get("complete", False):
            raise ValueError("Recording is incomplete")
        metadata = json.loads(file.attrs["metadata_json"])
        selected = camera_indices(metadata, report)
        times = np.asarray(file["frames/wall_time"][:], dtype=float)
        sim_times = np.asarray(file["frames/sim_time"][:], dtype=float)
        if len(times) < 2 or not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0):
            raise ValueError("Need at least two captures with strictly increasing wall timestamps")
        if len(sim_times) != len(times) or not np.all(np.isfinite(sim_times)):
            raise ValueError("Frame simulation timestamps must match wall timestamps")
        for camera in selected:
            if len(file[f"images/camera_{camera}"]) != len(times):
                raise ValueError("Camera frame counts do not match timestamps")
        timeline = build_timeline(float(times[0]), float(times[-1]), events, full=full)
        renderer = Renderer(file, report, selected, times, sim_times, fonts)
        temporary = Path(tmp) / "video.mp4"
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
            "1920x1080",
            "-r",
            str(fps),
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "fast",
            "-crf",
            "19",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(temporary),
        ]
        frame_count, page_index = 0, 0
        latest = None
        checkpoints = []
        event_pages = [page for page in pages if page["event"] != "final"]
        with (Path(tmp) / "ffmpeg.log").open("w+b") as error_stream:
            encoder = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=error_stream)

            def write_frame(wall, page, mode, repeats=1, checkpoint=False):
                nonlocal frame_count
                canvas = renderer.render(wall, page, mode, frame_count / fps)
                pixels = canvas.tobytes()
                for _ in range(repeats):
                    encoder.stdin.write(pixels)
                frame_count += repeats
                if checkpoint:
                    sample = canvas.copy()
                    sample.thumbnail((640, 360))
                    checkpoints.append(sample)
                return canvas

            def hold_page(page, duration):
                wall = page["source_wall_time"]
                wall = float(times[-1]) if wall is None else float(np.clip(wall, times[0], times[-1]))
                page["output_start"] = frame_count / fps
                result = write_frame(
                    wall,
                    page,
                    "Annotation pause | camera frozen at the displayed source timestamp",
                    repeats=max(1, math.ceil(duration * fps)),
                    checkpoint=True,
                )
                page["output_end"] = frame_count / fps
                return result

            try:
                for segment in timeline:
                    segment["output_start"] = frame_count / fps
                    count = max(1, math.ceil(segment["output_seconds"] * fps))
                    source_duration = segment["source_end"] - segment["source_start"]
                    for index in range(count):
                        fraction = min(index / (fps * segment["output_seconds"]), 1)
                        wall = segment["source_start"] + fraction * source_duration
                        while page_index < len(event_pages) and event_pages[page_index]["source_wall_time"] <= wall:
                            latest = event_pages[page_index]
                            hold_page(latest, page_seconds)
                            page_index += 1
                        if segment["kind"] == "shortened_wait":
                            mode = (
                                f"Inference wait shortened: {source_duration:.2f}s "
                                f"shown in {segment['output_seconds']:.2f}s"
                            )
                        else:
                            mode = "Recorded wall-clock motion at 1x | annotation pauses are labeled separately"
                        write_frame(wall, latest, mode, checkpoint=not checkpoints)
                    segment["output_end"] = frame_count / fps
                while page_index < len(event_pages):
                    hold_page(event_pages[page_index], page_seconds)
                    page_index += 1
                for page in pages:
                    if page["event"] == "final":
                        poster = hold_page(page, final_seconds)
                encoder.stdin.close()
                if encoder.wait() != 0:
                    error_stream.seek(0)
                    raise RuntimeError(error_stream.read().decode(errors="replace"))
            except BaseException:
                if encoder.stdin and not encoder.stdin.closed:
                    encoder.stdin.close()
                encoder.kill()
                encoder.wait()
                raise
        manifest = dict(
            video=str(output),
            episode=str(episode),
            cameras=report["cameras"],
            camera_indices=selected,
            fps=fps,
            video_frames=frame_count,
            duration_seconds=frame_count / fps,
            source_frames=len(times),
            source_duration_seconds=float(times[-1] - times[0]),
            source_wall_start=float(times[0]),
            source_wall_end=float(times[-1]),
            timeline_mode="full" if full else "compact",
            timeline=timeline,
            annotation_pages=pages,
            annotation_policy="Every public output and receipt is paginated; labeled camera freezes add reading time.",
            model_error=report.get("error"),
            evaluation=report.get("evaluation"),
            poster=str(targets[2]),
            contact_sheet=str(targets[3]),
        )
        poster.save(Path(tmp) / "poster.jpg", quality=95)
        chosen = [checkpoints[int(index)] for index in np.linspace(0, len(checkpoints) - 1, min(12, len(checkpoints)))]
        contact = Image.new("RGB", (1920, 360 * math.ceil(len(chosen) / 3)), BACKGROUND)
        for index, sample in enumerate(chosen):
            contact.paste(sample, ((index % 3) * 640, (index // 3) * 360))
        contact.save(Path(tmp) / "contact.jpg", quality=92)
        (Path(tmp) / "manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        for source, target in zip(
            [temporary, Path(tmp) / "manifest.json", Path(tmp) / "poster.jpg", Path(tmp) / "contact.jpg"], targets
        ):
            source.rename(target)
        return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument(
        "--full", action="store_true", help="Retain full source wall-clock time, plus annotation pauses"
    )
    parser.add_argument("--page-seconds", type=float, default=3.0)
    parser.add_argument("--final-seconds", type=float, default=5.0)
    args = parser.parse_args()
    manifest = export(args.directory, args.output, args.fps, args.full, args.page_seconds, args.final_seconds)
    print(
        json.dumps(
            {key: manifest[key] for key in ["video", "cameras", "video_frames", "duration_seconds"]}, ensure_ascii=False
        )
    )


if __name__ == "__main__":
    main()
