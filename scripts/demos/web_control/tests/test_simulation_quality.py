"""Torque budgeting can be checked without starting the simulator."""

import sys
from pathlib import Path
from types import SimpleNamespace

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulation_quality import gravity_feedforward


def test_gravity_feedforward_reserves_total_torque_budget():
    actuator = SimpleNamespace(
        cfg=SimpleNamespace(gravity_compensation=True), joint_indices=[0, 1], effort_limit=torch.tensor([[10.0, 5.0]])
    )
    robot = SimpleNamespace(
        data=SimpleNamespace(joint_pos=torch.zeros(1, 3), joint_effort_limits=torch.tensor([[10.0, 5.0, 2.0]])),
        actuators={"arm": actuator},
        root_physx_view=SimpleNamespace(get_gravity_compensation_forces=lambda: torch.tensor([[3.0, -8.0, 9.0]])),
    )
    captured = {}
    robot.write_joint_effort_limit_to_sim = lambda x: captured.update(limits=x)
    robot.set_joint_effort_target = lambda x: captured.update(efforts=x)
    gravity_feedforward(robot)
    assert torch.equal(captured["efforts"], torch.tensor([[3.0, -5.0, 0.0]]))
    assert torch.equal(captured["limits"], torch.tensor([[7.0, 0.0, 2.0]]))
    assert torch.all(captured["limits"] + captured["efforts"].abs() <= torch.tensor([[10.0, 5.0, 2.0]]))
