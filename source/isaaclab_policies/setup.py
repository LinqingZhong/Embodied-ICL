"""Install the policy library independently of the simulator and model frameworks."""

from setuptools import find_packages, setup

setup(
    name="isaaclab_policies",
    version="0.1.0",
    description="Interchangeable robot policies and a RobotClient execution loop",
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=[],
)
