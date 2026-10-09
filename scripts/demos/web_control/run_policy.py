#!/usr/bin/env python3
"""Run a policy against an already running web_control RobotServer."""

import argparse
import json
import logging
import signal
import sys
from dataclasses import asdict
from pathlib import Path

# Works from any cwd without installing simulator packages into the policy environment.
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "source/isaaclab_policies"))

from isaaclab_policies import PolicyRunner, RobotAdapter, create_policy
from isaaclab_policies.types import validate_action
from robot_client import RobotClient


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", default="gpt", help="gpt or an importable module:Class inheriting BasePolicy")
    parser.add_argument("--config", type=Path, help="JSON object passed to the policy constructor")
    parser.add_argument("--instruction", required=True, help="Robot task instruction")
    parser.add_argument("--url", default="http://127.0.0.1:8080")
    parser.add_argument("--max-steps", type=int, default=10, help="Maximum policy decisions, not physics steps")
    parser.add_argument("--action-timeout", type=float, default=120)
    parser.add_argument("--observation-timeout", type=float, default=30)
    parser.add_argument(
        "--http-timeout", type=float, default=30, help="Robot HTTP timeout, including EEF IK submission"
    )
    parser.add_argument("--model", help="Override GPT model name")
    parser.add_argument("--endpoint", help="Override GPT Responses endpoint")
    parser.add_argument(
        "--dry-run", action="store_true", help="Call the model once and print its action without motion"
    )
    args = parser.parse_args(argv)
    if args.max_steps < 1:
        parser.error("--max-steps must be positive")
    if not args.instruction.strip():
        parser.error("--instruction must be nonempty")
    if args.policy != "gpt" and (args.model or args.endpoint):
        parser.error("--model and --endpoint apply only to --policy gpt; use --config for custom policies")
    return args


def load_policy_config(args) -> dict:
    config = json.loads(args.config.read_text(encoding="utf-8")) if args.config else {}
    if not isinstance(config, dict):
        raise ValueError("Policy config must contain a JSON object")
    for name in ("model", "endpoint"):
        if getattr(args, name) is not None:
            config[name] = getattr(args, name)
    return config


def predict_once(policy, robot: RobotAdapter, instruction: str):
    """Model-only dry run. No execution feedback is fabricated for the proposal."""
    try:
        robot.check_ready()
        spec = robot.get_robot_spec()
        policy.reset(instruction, spec)
        action = policy.predict(robot.get_observation())
        validate_action(action, spec)
        return {"dry_run": True, "action_type": type(action).__name__, "action": asdict(action)}
    finally:
        policy.close()


def main(argv=None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    try:
        config = load_policy_config(args)
        robot = RobotAdapter(RobotClient(args.url, timeout=args.http_timeout), action_timeout=args.action_timeout)
        policy = create_policy(args.policy, config)
        # Construct the runner inside cleanup even when its configuration is invalid.
        if args.dry_run:
            result = predict_once(policy, robot, args.instruction)
        else:
            try:
                runner = PolicyRunner(policy, robot, observation_timeout=args.observation_timeout)
            except Exception:
                policy.close()
                raise
            results = runner.run(args.instruction, args.max_steps)
            result = {"decisions": len(results), "results": [asdict(item) for item in results]}
        print(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2))
        return 0
    except KeyboardInterrupt:
        print(
            "Policy interrupted; any confirmed in-flight task was cancelled if the server was reachable.",
            file=sys.stderr,
        )
        return 130
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


def interrupt(*_):
    raise KeyboardInterrupt


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, interrupt)
    raise SystemExit(main())
