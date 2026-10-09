import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples"))


def test_lift_hold_distinguishes_brief_bump_from_sustained_lift():
    from evaluate_gpt_recording import measure_lift

    brief = measure_lift([0, 1, 2, 3, 4], [[0, 0, z] for z in [0.2, 0.2, 0.3, 0.2, 0.2]])
    assert brief["lift_success"] is False
    held = measure_lift([0, 1, 2, 3, 4, 5], [[0, 0, z] for z in [0.2, 0.2, 0.3, 0.3, 0.3, 0.3]])
    assert held["lift_success"] is True
    assert held["longest_hold_above_8cm_sim_seconds"] == 3
