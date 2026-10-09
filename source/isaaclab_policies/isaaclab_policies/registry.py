"""Small explicit factory. A custom policy owns its own model dependencies."""

import importlib

from .base import BasePolicy


def create_policy(name: str, config: dict | None = None, *, demonstration=None) -> BasePolicy:
    """Load 'gpt' or an importable 'module:Class'; pass configuration by value."""
    if config is not None and not isinstance(config, dict):
        raise ValueError("Policy config must be a JSON object")
    config = dict(config or {})
    if name == "gpt":
        from .gpt import GPTConfig, GPTPolicy

        return GPTPolicy(GPTConfig(**config), demonstration=demonstration)
    if demonstration is not None:
        raise ValueError("Video demonstrations currently require the gpt policy")
    if not isinstance(name, str) or name.count(":") != 1:
        raise ValueError("Policy must be 'gpt' or an importable 'module:Class'")
    module_name, class_name = name.split(":")
    policy_type = getattr(importlib.import_module(module_name), class_name)
    if not isinstance(policy_type, type) or not issubclass(policy_type, BasePolicy):
        raise TypeError("Custom policy must inherit BasePolicy")
    return policy_type(**config)
