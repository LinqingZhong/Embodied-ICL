"""Cobot Magic: four independent link6 flange frames, each relative to its arm base."""


def arms(instances=False):
    return [
        dict(
            name=side,
            instance="robot",
            base=f"{side}_base_link",
            flange=f"{side}_link6",
            end_body=f"{side}_link6",
            offset=[0, 0, 0],
            joints=[f"{side}_joint{i}" for i in range(1, 7)],
        )
        for side in ("fl", "fr", "bl", "br")
    ]
