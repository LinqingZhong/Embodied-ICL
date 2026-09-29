"""Panda flange is panda_link8: 0.107 m along link7 Z, before the hand rotation."""


def arms(instances=False):
    return [
        dict(
            name="arm",
            instance="robot",
            base="panda_link0",
            flange="panda_link8",
            end_body="panda_link7",
            offset=[0, 0, 0.107],
            joints=[f"panda_joint{i}" for i in range(1, 8)],
        )
    ]
