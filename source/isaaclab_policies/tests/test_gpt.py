import json
import threading
from dataclasses import asdict, replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest
from isaaclab_policies import (
    ActionResult,
    GPTConfig,
    GPTPolicy,
    JointActionChunk,
    PolicyObservation,
    RobotSpec,
    StopAction,
    create_policy,
)
from test_execution import metadata, observation


def response(name="execute_joint_chunk", arguments=None, call_id="call_1", call_status="completed"):
    if arguments is None:
        arguments = dict(joint_names=["finger_l", "finger_r"], positions=[[0.04, -0.04]], steps_per_action=12)
    return dict(
        status="completed",
        output=[
            dict(type="reasoning", id="reason_1", summary=[], encrypted_content="opaque-reasoning"),
            dict(
                type="function_call",
                id="item_1",
                status=call_status,
                call_id=call_id,
                name=name,
                arguments=json.dumps(arguments),
            ),
        ],
    )


@pytest.fixture
def provider(monkeypatch):
    monkeypatch.setenv("POLICY_TEST_KEY", "test-secret")
    requests, responses = [], []

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            payload = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            requests.append((self.path, self.headers.get("Authorization"), payload))
            reply = responses.pop(0)
            status = reply.pop("http_status", 200)
            data = json.dumps(reply).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    cfg = GPTConfig(
        endpoint=f"http://127.0.0.1:{server.server_port}/v1/responses",
        api_key_env="POLICY_TEST_KEY",
        history_steps=1,
        timeout=3,
    )
    yield cfg, requests, responses
    server.shutdown()
    server.server_close()
    thread.join()


def policy_observation(sequence=1):
    raw = vars(observation(sequence, sequence))
    raw.pop("object_states")
    return PolicyObservation(**raw)


def start(cfg):
    policy = GPTPolicy(cfg)
    policy.reset("打开夹爪", RobotSpec(**metadata()))
    return policy


def feedback(policy, action, task_id="task_1"):
    policy.observe_result(ActionResult(action, task_id, "completed", 12, 1.0, {"errors": [0, 0]}))


def test_request_native_units_images_and_strict_tools(provider):
    cfg, requests, responses = provider
    responses.append(response())
    action = start(cfg).predict(policy_observation())
    assert action == JointActionChunk(["finger_l", "finger_r"], [[0.04, -0.04]], 12)
    path, authorization, payload = requests[0]
    assert path == "/v1/responses" and authorization == "Bearer test-secret"
    assert payload["model"] == cfg.model and payload["store"] is False and payload["stream"] is False
    assert payload["parallel_tool_calls"] is False and payload["tool_choice"] == "required"
    assert payload["include"] == ["reasoning.encrypted_content"]
    assert {t["name"] for t in payload["tools"]} == {"execute_joint_chunk", "execute_eef_chunk", "finish"}
    assert all(t["strict"] and not t["parameters"]["additionalProperties"] for t in payload["tools"])
    content = payload["input"][-1]["content"]
    assert any(c["type"] == "input_image" and c["image_url"].startswith("data:image/jpeg;base64,") for c in content)
    text = json.dumps(payload, ensure_ascii=False)
    assert "finger_r" in text and "-0.04" in text and "wxyz" in text
    assert "test-secret" not in text and "object_states" not in text


def test_feedback_history_pairs_and_trims_whole_turns(provider):
    cfg, requests, responses = provider
    policy = start(cfg)
    responses.extend([response(call_id=f"call_{i}") for i in range(1, 4)])
    action = policy.predict(policy_observation())
    with pytest.raises(RuntimeError, match="feedback"):
        policy.predict(policy_observation(2))
    feedback(policy, action)
    action = policy.predict(policy_observation(2))
    history = requests[-1][2]["input"]
    calls = [i for i in history if i.get("type") == "function_call"]
    receipts = [i for i in history if i.get("type") == "function_call_output"]
    assert calls[0]["call_id"] == receipts[0]["call_id"] == "call_1"
    assert "id" not in calls[0] and "status" not in calls[0]
    assert json.loads(receipts[0]["output"])["executed_steps"] == 12
    assert any(i.get("encrypted_content") == "opaque-reasoning" for i in history)
    feedback(policy, action, "task_2")
    policy.predict(policy_observation(3))
    call_ids = [i["call_id"] for i in requests[-1][2]["input"] if i.get("type") == "function_call"]
    assert call_ids == ["call_2"]


def test_gateway_without_reasoning_replay_keeps_observations_actions_and_receipts(provider):
    cfg, requests, responses = provider
    policy = start(replace(cfg, preserve_reasoning=False))
    responses.extend([response(call_id="call_1"), response(call_id="call_2")])
    action = policy.predict(policy_observation())
    feedback(policy, action)
    policy.predict(policy_observation(2))
    payload = requests[-1][2]
    assert "include" not in payload
    history = payload["input"]
    assert not any(item.get("type") == "reasoning" for item in history)
    assert "opaque-reasoning" not in json.dumps(payload)
    calls = [item for item in history if item.get("type") == "function_call"]
    receipts = [item for item in history if item.get("type") == "function_call_output"]
    assert calls[0]["call_id"] == receipts[0]["call_id"] == "call_1"
    assert json.loads(receipts[0]["output"])["executed_steps"] == 12
    assert sum(item.get("role") == "user" for item in history) == 2


@pytest.mark.parametrize("value", ["false", 0, None])
def test_preserve_reasoning_requires_boolean(value):
    with pytest.raises(ValueError, match="preserve_reasoning"):
        GPTConfig(preserve_reasoning=value)


@pytest.mark.parametrize(
    "reply",
    [
        {"status": "incomplete", "output": []},
        {"status": "failed", "error": {"message": "test-secret"}, "output": []},
        {"status": "completed", "output": []},
        response("shell", {"command": "anything"}),
        response(arguments={"joint_names": ["wheel"], "positions": [[0]], "steps_per_action": 1}),
        response(arguments={"joint_names": ["joint"], "positions": [[2]], "steps_per_action": 1}),
        response(arguments={"joint_names": ["joint"], "positions": [[False]], "steps_per_action": 1}),
        response(arguments={"joint_names": ["joint"], "positions": [[0]], "steps_per_action": 1, "extra": True}),
        response(call_id=""),
        {"http_status": 401, "error": "echo test-secret"},
    ],
)
def test_invalid_responses_rejected_without_secret_leak(provider, reply):
    cfg, requests, responses = provider
    responses.append(reply)
    with pytest.raises((ValueError, RuntimeError)) as error:
        start(cfg).predict(policy_observation())
    assert "test-secret" not in str(error.value)
    assert len(requests) == 1


def test_stop_reset_feedback_mismatch_and_unsupported_eef(provider):
    cfg, requests, responses = provider
    policy = start(cfg)
    responses.append(response("finish", {"reason": "done"}))
    action = policy.predict(policy_observation())
    assert action == StopAction("done")
    with pytest.raises(ValueError, match="action"):
        feedback(policy, JointActionChunk(["joint"], [[0]], 1))
    policy.observe_result(ActionResult(action, None, "stopped", 0, None, {"reason": "done"}))
    with pytest.raises(RuntimeError):
        policy.predict(policy_observation(2))
    spec = metadata()
    spec["arms"] = {}
    policy.reset("new task", RobotSpec(**spec))
    obs = policy_observation()
    obs.eef_poses.clear()
    responses.append(response())
    policy.predict(obs)
    assert len(requests[-1][2]["input"]) == 1
    assert "execute_eef_chunk" not in [t["name"] for t in requests[-1][2]["tools"]]


@pytest.mark.parametrize(
    "reply",
    [
        response(call_status="incomplete"),
        response(call_status="in_progress"),
        {**response(), "incomplete_details": {"reason": "max_output_tokens"}},
    ],
)
def test_completed_envelope_cannot_hide_incomplete_action(provider, reply):
    cfg, _, responses = provider
    responses.append(reply)
    with pytest.raises((ValueError, RuntimeError), match="complet"):
        start(cfg).predict(policy_observation())


def test_factory_config_and_environment_precedence(provider, monkeypatch):
    cfg, _, _ = provider
    assert isinstance(create_policy("gpt", asdict(cfg)), GPTPolicy)
    with pytest.raises(ValueError):
        create_policy("unknown")
    monkeypatch.setenv("ASTRA_MODEL", "gpt-6-astra-test")
    monkeypatch.setenv("ASTRA_ENDPOINT", cfg.endpoint)
    monkeypatch.setenv("ASTRA_REASONING_EFFORT", "high")
    assert GPTConfig().model == "gpt-6-astra-test"
    assert GPTConfig(model="explicit").model == "explicit"
    assert GPTConfig().endpoint == cfg.endpoint
    with pytest.raises(ValueError):
        GPTConfig(endpoint="https://example.test/chat/completions")
    with pytest.raises(ValueError):
        GPTConfig(history_steps=-1)


def test_multiple_cameras_and_eef_targets_keep_names_and_coordinate_order(provider):
    from isaaclab_policies import EefActionChunk

    cfg, requests, responses = provider
    spec = metadata()
    spec["cameras"] = ["head", "left_wrist", "right_wrist"]
    obs = policy_observation()
    obs.images.clear()
    obs.images.update({name: name.encode() for name in spec["cameras"]})
    args = dict(arm="arm", positions=[[0.1, -0.2, 0.3]], quaternions=[[0, 1, 0, 0]], steps_per_action=6)
    responses.append(response("execute_eef_chunk", args))
    policy = GPTPolicy(cfg)
    policy.reset("move flange", RobotSpec(**spec))
    assert policy.predict(obs) == EefActionChunk(**args)
    content = requests[0][2]["input"][-1]["content"]
    assert [x["text"] for x in content if x.get("text", "").startswith("Camera:")] == [
        f"Camera: {name}" for name in spec["cameras"]
    ]
    assert len([x for x in content if x["type"] == "input_image"]) == 3


def test_end_to_end_responses_policy_through_robot_http(provider, tmp_path):
    import time

    from isaaclab_policies import PolicyRunner, RobotAdapter
    from robot_client import RobotClient
    from robot_server import RobotServer
    from server import WebServer

    cfg, requests, responses = provider
    state = RobotServer(manual_control=False)
    state.configure(["arm", "finger"], [[-1, 1], [0, 0.04]], [0, 0.02], [0], [1])
    web = WebServer(state, port=0, output_dir=tmp_path)
    stopped = threading.Event()

    def simulate():
        step = 0
        while not stopped.is_set():
            tick = state.consume(1 / 120)
            step += 1
            images = {"scene": b"scene-jpeg", "wrist": b"wrist-jpeg"} if step % 4 == 0 else None
            state.publish(step / 120, tick["targets"], [0, 0], images)
            time.sleep(0.002)

    thread = threading.Thread(target=simulate, daemon=True)
    responses.extend(
        [
            response(
                arguments=dict(joint_names=["arm", "finger"], positions=[[0.2, 0.04], [0.4, 0]], steps_per_action=3)
            ),
            response("finish", {"reason": "observed result"}, "call_2"),
        ]
    )
    web.start()
    thread.start()
    try:
        results = PolicyRunner(GPTPolicy(cfg), RobotAdapter(RobotClient(web.url)), poll_interval=0.002).run("move", 3)
        assert [r.status for r in results] == ["completed", "stopped"]
        assert results[0].executed_steps == 6
        assert state.applied == [0.4, 0]
        latest_input = requests[-1][2]["input"]
        receipt = next(i for i in latest_input if i.get("type") == "function_call_output")
        assert json.loads(receipt["output"])["task_id"] == results[0].task_id
        snapshot = json.loads(latest_input[-1]["content"][0]["text"])
        assert snapshot["sim_time"] >= results[0].finished_sim_time
        assert snapshot["positions"] == [0.4, 0]
    finally:
        stopped.set()
        thread.join()
        web.stop()


def test_cli_dry_run_calls_model_but_never_dispatches(provider, tmp_path):
    import subprocess
    import sys
    from pathlib import Path

    from robot_server import RobotServer
    from server import WebServer

    cfg, requests, responses = provider
    state = RobotServer(manual_control=False)
    state.configure(["arm"], [[-1, 1]], [0], [0], [])
    state.publish(0, [0], [0], {"scene": b"scene-jpeg", "wrist": b"wrist-jpeg"})
    web = WebServer(state, port=0, output_dir=tmp_path)
    config_path = tmp_path / "policy.json"
    config_path.write_text(json.dumps(asdict(cfg)))
    cli = Path(__file__).resolve().parents[3] / "scripts/demos/web_control/run_policy.py"
    responses.append(response(arguments=dict(joint_names=["arm"], positions=[[0.2]], steps_per_action=1)))
    web.start()
    try:
        result = subprocess.run(
            [
                sys.executable,
                str(cli),
                "--url",
                web.url,
                "--config",
                str(config_path),
                "--instruction",
                "move",
                "--dry-run",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert result.returncode == 0, result.stderr
        assert json.loads(result.stdout)["action"]["positions"] == [[0.2]]
        assert json.loads(result.stdout)["dry_run"] is True
        assert state.active_task is None and not state.tasks and state.applied == [0]
        assert len(requests) == 1
    finally:
        web.stop()
