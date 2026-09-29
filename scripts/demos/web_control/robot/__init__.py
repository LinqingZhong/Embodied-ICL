"""Registry of editable robot profiles; simulator imports stay in the main thread."""

import copy

from .a2d import PROFILE as A2D
from .cobot_magic import PROFILE as COBOT_MAGIC
from .franka import PROFILE as FRANKA
from .x5 import PROFILE as X5
from .x5_dual import PROFILE as X5_DUAL

PROFILES = {"franka": FRANKA, "a2d": A2D, "x5": X5, "x5_dual": X5_DUAL, "cobot_magic": COBOT_MAGIC}


def get_profile(name):
    return copy.deepcopy(PROFILES[name])
