"""RobStride motor control library (vendored from Seeed-Projects/RobStride_Control).

Upstream: https://github.com/Seeed-Projects/RobStride_Control/tree/master/python
License: MIT

This is the shared library — no ROS dependencies, usable standalone.
"""

from .bus import RobstrideBus, Motor  # noqa: F401
from .protocol import CommunicationType, ParameterType  # noqa: F401
from .table import (  # noqa: F401
    MODEL_MIT_POSITION_TABLE,
    MODEL_MIT_VELOCITY_TABLE,
    MODEL_MIT_TORQUE_TABLE,
    MODEL_MIT_KP_TABLE,
    MODEL_MIT_KD_TABLE,
)
