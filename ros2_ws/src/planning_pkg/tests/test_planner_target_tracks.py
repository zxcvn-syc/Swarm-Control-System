import inspect
from types import SimpleNamespace

from planning_pkg.planner_node import PlannerNode


class _Logger:
    def debug(self, _message):
        pass


def test_world_target_cache_keeps_finite_world_tracks():
    node = SimpleNamespace(_target_world={}, get_logger=lambda: _Logger())
    message = SimpleNamespace(
        header=SimpleNamespace(frame_id="world"),
        tracks=[
            SimpleNamespace(target_id=101, x=10.5, y=8.25),
            SimpleNamespace(target_id=202, x=float("nan"), y=4.0),
        ],
    )

    PlannerNode.on_target_world(node, message)

    assert node._target_world == {101: (10.5, 8.25)}


def test_task_uses_cached_world_target_before_legacy_scatter():
    planned = []
    node = SimpleNamespace(
        _drone_target={},
        _target_world={101: (10.6, 8.2)},
        _explicit_target_set=lambda: set(),
        _world_to_cell=lambda xy: (round(xy[0]), round(xy[1])),
        _scatter_target=lambda _target_id: (39, 39),
        _plan_for_drone=lambda drone_id, target: planned.append((drone_id, target)),
        _publish_drone_states=lambda: None,
        get_logger=lambda: _Logger(),
    )

    PlannerNode.on_task(
        node,
        SimpleNamespace(drone_id=3, target_id=101, task_type="track"),
    )

    assert node._drone_target[3] == (11, 8)
    assert planned == [(3, (11, 8))]


def test_initial_position_parameter_accepts_floating_point_arrays():
    """initial_positions must stay *initialized* (``[]`` default).

    A type-only declaration (``Parameter.Type.DOUBLE_ARRAY`` without a
    default value) leaves the parameter unset, so ``get_parameter()``
    raises ParameterUninitializedException on every no-argument
    PlannerNode() construction — which silently disabled link2/link3
    in test_three_links.py / CI.  The empty-list default keeps the
    parameter initialized and still infers DOUBLE_ARRAY, so
    float-array overrides keep working (verified behaviorally below).
    """
    source = inspect.getsource(PlannerNode.__init__)
    assert 'declare_parameter("initial_positions", [])' in source

    import rclpy
    from rclpy.parameter import Parameter

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("test_initial_positions_decl")
    try:
        node.declare_parameter("initial_positions", [])
        param = node.get_parameter("initial_positions")
        assert param.type_ == Parameter.Type.DOUBLE_ARRAY
        assert list(param.value) == []
        node.set_parameters(
            [Parameter("initial_positions", value=[1.5, 2.5, 3.5, 4.5])]
        )
        assert list(node.get_parameter("initial_positions").value) == [
            1.5, 2.5, 3.5, 4.5,
        ]
    finally:
        node.destroy_node()
