"""conftest.py — meta_path import hook to stub ROS2 deps before collection.

The launch_testing pytest plugin intercepts module collection using sys.meta_path.
We install our own meta_path finder BEFORE collection (via pytest_configure)
so our hook fires before launch_testing's hooks, ensuring rclpy/swarm_interfaces
stubs are in sys.modules when tracker_node.py tries to import them.
"""

from __future__ import annotations

import importlib
import importlib.abc
import importlib.machinery
import importlib.util
import sys
import types
from pathlib import Path

import pytest


# ---------------------------------------------------------------------------
# Stub module factories
# ---------------------------------------------------------------------------

def _make_swarm_stub() -> None:
    # If the real swarm_interfaces.msg is already importable (e.g. CI
    # sourced install/setup.bash), use it instead of the stub.
    try:
        import swarm_interfaces.msg as _real  # noqa: F401
        required_messages = (
            "DroneState",
            "DroneStateArray",
            "EnclosureTarget",
            "EnclosureTargetArray",
            "TargetTrack",
            "TargetTrackArray",
            "TargetTrackDebug",
        )
        if all(hasattr(_real, name) for name in required_messages):
            return
    except ImportError:
        pass

    sys.modules.pop("swarm_interfaces.msg", None)
    sys.modules.pop("swarm_interfaces", None)

    swarm = types.ModuleType("swarm_interfaces")
    swarm.__file__ = "<stub:swarm_interfaces>"
    msg = types.ModuleType("swarm_interfaces.msg")
    msg.__file__ = "<stub:swarm_interfaces.msg>"

    class _TT:
        target_id: int = 0; x: float = 0.0; y: float = 0.0
        vx: float = 0.0; vy: float = 0.0; confidence: float = 1.0
        cls: int = 0; is_confirmed: bool = True; speed: float = 0.0
        label: str = ""; bbox_x1: float = 0.0; bbox_y1: float = 0.0
        bbox_x2: float = 0.0; bbox_y2: float = 0.0
        motion_mode: int = 0; pred_x: list = [0.0] * 5
        pred_y: list = [0.0] * 5; pred_conf: list = [0.0] * 5

    class _TTA:
        header: object = None; tracks: list = []; frame_idx: int = 0
        image_width: int = 0; image_height: int = 0

    class _TTD:
        header: object = None; tracks: list = []; source_topic: str = ""
        kf_covariance: list = []; motion_mode_reasons: list = []
        appearance_scores: list = []

    class _ET:
        target_id: int = 0; x: float = 0.0; y: float = 0.0
        speed: float = 0.0; motion_mode: int = 0; confidence: float = 1.0
        box_x1: float = 0.0; box_y1: float = 0.0
        box_x2: float = 0.0; box_y2: float = 0.0
        pred_x: list = [0.0] * 5; pred_y: list = [0.0] * 5
        history_x: list = []; history_y: list = []

    class _ETA:
        header: object = None; frame_idx: int = 0; targets: list = []
        drone_x: list = [0.0] * 8; drone_y: list = [0.0] * 8
        num_drones: int = 0; enclosure_radius: float = 50.0
        min_enclosure_dist: float = 20.0

    # --- types needed by containment_pkg / planning_pkg tests ---
    class _DroneState:
        PLATFORM_DRONE: int = 0
        PLATFORM_CAR: int = 1
        drone_id: int = 0
        x: float = 0.0; y: float = 0.0; z: float = 0.0
        vx: float = 0.0; vy: float = 0.0; vz: float = 0.0
        available: bool = True
        platform_type: int = 0

    class _DroneStateArray:
        drones: list = []; num_drones: int = 0

    class _TaskAssignment:
        drone_id: int = 0; target_id: int = 0; task_type: str = ""

    class _EnclosureCommand:
        LAYER_MONITOR: int = 0
        LAYER_BLOCK: int = 1
        LAYER_COMMAND: int = 2
        drone_id: int = 0
        target_x: float = 0.0; target_y: float = 0.0; target_z: float = 0.0
        enclosure_radius: float = 0.0
        layer: int = 0

    class _EnclosureCommandArray:
        commands: list = []; num_drones: int = 0

    msg.TargetTrack = _TT; msg.TargetTrackArray = _TTA
    msg.TargetTrackDebug = _TTD; msg.EnclosureTarget = _ET
    msg.EnclosureTargetArray = _ETA
    msg.DroneState = _DroneState; msg.DroneStateArray = _DroneStateArray
    msg.TaskAssignment = _TaskAssignment
    msg.EnclosureCommand = _EnclosureCommand
    msg.EnclosureCommandArray = _EnclosureCommandArray
    swarm.msg = msg
    sys.modules["swarm_interfaces"] = swarm
    sys.modules["swarm_interfaces.msg"] = msg


def _install_rclpy_stubs() -> None:
    # If the real rclpy is already importable (e.g. CI sourced
    # install/setup.bash), use it instead of the stubs.
    try:
        import rclpy  # noqa: F401
        import rclpy.node  # noqa: F401
        if hasattr(rclpy.node, "Node"):
            return
    except ImportError:
        pass

    if "rclpy" in sys.modules:
        return

    def _m(name: str) -> types.ModuleType:
        mod = types.ModuleType(name)
        mod.__file__ = f"<stub:{name}>"
        return mod

    rclpy = _m("rclpy")
    rclpy.init = lambda *a, **kw: None
    rclpy.try_shutdown = lambda: None
    rclpy.spin = lambda *a, **kw: None

    rclpy_node = _m("rclpy.node")
    rclpy_node.Node = type("Node", (), {})

    rclpy_qos = _m("rclpy.qos")
    rclpy_qos.QoSProfile = type("QoSProfile", (), {})
    rclpy_qos.ReliabilityPolicy = type("ReliabilityPolicy", (), {})

    rclpy_exec = _m("rclpy.executors")
    rclpy_exec.ExternalShutdownException = type("ExternalShutdownException", (Exception,), {})

    rcl_time = _m("rclpy.time")
    rcl_time.Time = type("Time", (), {"to_msg": lambda self: types.SimpleNamespace()})

    rcl_interfaces = _m("rcl_interfaces")
    rcl_interfaces_msg = _m("rcl_interfaces.msg")
    rcl_interfaces_msg.ParameterDescriptor = type(
        "ParameterDescriptor",
        (),
        {
            "__init__": lambda self, **kw: None,
        },
    )
    rcl_interfaces.msg = rcl_interfaces_msg

    for n, m in [
        ("rclpy", rclpy), ("rclpy.node", rclpy_node),
        ("rclpy.qos", rclpy_qos), ("rclpy.executors", rclpy_exec),
        ("rclpy.time", rcl_time),
        ("rcl_interfaces", rcl_interfaces), ("rcl_interfaces.msg", rcl_interfaces_msg),
    ]:
        sys.modules.setdefault(n, m)

    # geometry_msgs / sensor_msgs stubs
    for _pkg, _cls in [("geometry_msgs", "PoseStamped"), ("sensor_msgs", "CameraInfo")]:
        if _pkg not in sys.modules:
            g = _m(_pkg); gm = _m(f"{_pkg}.msg")
            setattr(gm, _cls, type(_cls, (), {}))
            g.msg = gm
            sys.modules[_pkg] = g; sys.modules[f"{_pkg}.msg"] = gm

    # Pre-load real ROS2 packages into sys.modules so Python resolves to the
    # real implementations (not namespace-package stubs).
    # NOTE: __import__(pkg) only registers sys.modules[pkg], not sys.modules[pkg.msg].
    # We must also import pkg.msg so tracker_node.py's `from std_msgs.msg import X`
    # succeeds without going through the namespace-package search path.
    for _pkg in ("std_msgs", "std_msgs.msg",
                  "diagnostic_msgs", "diagnostic_msgs.msg",
                  "actionlib_msgs", "actionlib_msgs.msg",
                  "builtin_interfaces", "builtin_interfaces.msg"):
        try:
            __import__(_pkg)
        except ImportError:
            pass


def _prefer_real_message_modules() -> None:
    """Restore generated ROS message modules when launch-testing preloads stubs."""
    try:
        import rclpy
    except ImportError:
        return
    if str(getattr(rclpy, "__file__", "")).startswith("<stub:"):
        return

    for package, message_name in (
        ("std_msgs", "Header"),
        ("diagnostic_msgs", "DiagnosticArray"),
        ("builtin_interfaces", "Time"),
    ):
        module_name = f"{package}.msg"
        message_module = sys.modules.get(module_name)
        message_type = getattr(message_module, message_name, None)
        if hasattr(getattr(message_type, "__class__", None), "_TYPE_SUPPORT"):
            continue

        saved_modules = {
            name: sys.modules[name]
            for name in (package, module_name)
            if name in sys.modules
        }
        sys.modules.pop(module_name, None)
        sys.modules.pop(package, None)
        try:
            message_module = importlib.import_module(module_name)
            message_type = getattr(message_module, message_name)
            if not hasattr(message_type.__class__, "_TYPE_SUPPORT"):
                raise ImportError(f"{module_name}.{message_name} lacks type support")
        except (ImportError, AttributeError):
            sys.modules.update(saved_modules)


# ---------------------------------------------------------------------------
# Meta path finder
# ---------------------------------------------------------------------------

_PACKAGE_ROOT = Path(__file__).resolve().parent.parent
_SRC_ROOT = _PACKAGE_ROOT / 'perception_pkg'
_STUBS_INSTALLED = False


def _ensure_stubs() -> None:
    global _STUBS_INSTALLED
    if not _STUBS_INSTALLED:
        _STUBS_INSTALLED = True
        _install_rclpy_stubs()
        _make_swarm_stub()
        _prefer_real_message_modules()


class _PerceptionPkgFinder(importlib.abc.MetaPathFinder):
    """Intercept perception_pkg sub-module imports and load real source files."""

    def find_spec(self, fullname, path, target=None):
        if not fullname.startswith("perception_pkg."):
            return None
        sub = fullname[len("perception_pkg."):]
        src_map = {
            "tracker_node": str(_SRC_ROOT / 'tracker_node.py'),
            "coord_transform_node": str(_SRC_ROOT / 'coord_transform_node.py'),
        }
        if sub not in src_map:
            return None
        return importlib.util.spec_from_file_location(
            fullname, src_map[sub],
            loader=_PerceptionPkgLoader(),
            submodule_search_locations=[],
        )


class _PerceptionPkgLoader(importlib.abc.Loader):
    """Load real source file with stubs pre-installed."""

    def create_module(self, spec):
        return None

    def exec_module(self, module):
        _ensure_stubs()
        src_path = module.__spec__.origin
        loader = importlib.machinery.SourceFileLoader(
            module.__spec__.name + "_src", src_path,
        )
        src_spec = importlib.util.spec_from_file_location(
            module.__spec__.name + "_src", src_path, loader=loader,
        )
        src_mod = importlib.util.module_from_spec(src_spec)
        src_mod.__package__ = "perception_pkg"
        src_spec.loader.exec_module(src_mod)
        # Copy all attributes (including private ones like _declare_parameters)
        for _attr in dir(src_mod):
            if not hasattr(module, _attr):
                setattr(module, _attr, getattr(src_mod, _attr))


_finder = _PerceptionPkgFinder()


def pytest_configure(config) -> None:
    # Install meta_path hook before launch_testing's collection intercepts run.
    if _finder not in sys.meta_path:
        sys.meta_path.insert(0, _finder)
    _ensure_stubs()
    # Make cvtrack importable from the vendored src/ directory.
    # Without this, test_yolo_inference_speed.py can't `from cvtrack.runner import ...`
    # because pytest runs from perception_pkg/ and cvtrack/src is not on sys.path.
    cvtrack_src = str(_PACKAGE_ROOT / 'cvtrack' / 'src')
    if cvtrack_src not in sys.path:
        sys.path.insert(0, cvtrack_src)
