"""ARX X5 flange: link6 origin, expressed in each instance's base_link."""


def arms(instances=False):
    return [
        dict(
            name=side if instances else "arm",
            instance=side if instances else "robot",
            base="base_link",
            flange="link6",
            end_body="link6",
            offset=[0, 0, 0],
            joints=[f"joint{i}" for i in range(1, 7)],
        )
        for side in (["left", "right"] if instances else ["robot"])
    ]
