"""Shared task input options for policy execution and recording."""

from pathlib import Path

from isaaclab_policies.demonstration import IMITATION_INSTRUCTION, load_video_demonstration


def add_task_arguments(parser):
    parser.add_argument("--instruction", help="Task text or optional guidance; may be omitted with --demo-video")
    parser.add_argument(
        "--demo-video", type=Path, help="Local human demonstration video; visual frames define the task"
    )
    parser.add_argument("--demo-frames", type=int, default=16, help="Maximum chronological reference frames (2–32)")
    parser.add_argument(
        "--icl-mode",
        choices=("frames", "description"),
        help="Repeat demo frames or interpret once and control from text; overrides GPT config (default: frames)",
    )
    parser.add_argument(
        "--demo-max-side", type=int, default=768, help="Maximum reference image side in pixels (32–1280)"
    )


def validate_task_arguments(parser, args):
    if args.instruction is not None and not args.instruction.strip():
        parser.error("--instruction must be nonempty")
    if args.instruction is None and args.demo_video is None:
        parser.error("Provide --instruction or --demo-video")
    if args.demo_video is not None and getattr(args, "policy", "gpt") != "gpt":
        parser.error("--demo-video currently requires --policy gpt")
    if args.icl_mode is not None and getattr(args, "policy", "gpt") != "gpt":
        parser.error("--icl-mode requires --policy gpt")
    if not 2 <= args.demo_frames <= 32 or not 32 <= args.demo_max_side <= 1280:
        parser.error("--demo-frames must be 2–32 and --demo-max-side must be 32–1280")


def load_task_input(args):
    demonstration = None
    if args.demo_video is not None:
        demonstration = load_video_demonstration(
            args.demo_video, max_frames=args.demo_frames, max_side=args.demo_max_side
        )
    return args.instruction if args.instruction is not None else IMITATION_INSTRUCTION, demonstration
