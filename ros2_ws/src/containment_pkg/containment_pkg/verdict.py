"""Pure verdict logic for the three-scene containment evaluator.

Kept free of any rclpy / ROS import so it can be exercised directly by
``pytest`` without spinning up a node or a ROS graph.  The live
``containment_evaluator`` node calls :func:`decide_verdict` from its
``finalize`` step; ``test/test_verdict.py`` feeds it synthetic target /
platform evidence to lock down the SUCCESS / FAIL / INVALID matrix.

Why a gate at all
-----------------
The 8.25 evaluator only watched the *target's* distance from its start point:
``return`` / ``oscillate`` trajectories were pre-destined SUCCESS and
``straight`` pre-destined FAIL, so the resulting "success rate" was just the
trajectory-weight distribution baked into ``three_scene_config.yaml`` -- not a
measurement of the containment system.  The 8.26 requirement adds a *response
evidence* gate: a SUCCESS must be accompanied by at least one platform coming
within ``intercept_radius`` of the target.  Runs where the target is contained
but no platform ever engaged are judged ``INVALID`` -- reported separately,
counted neither as success nor as failure.
"""

from typing import Optional


def decide_verdict(
    *,
    escaped: bool,
    re_contained: bool,
    held: bool,
    min_platform_dist: Optional[float],
    intercept_radius: float,
) -> str:
    """Decide the verdict for one enclosure test run.

    Parameters
    ----------
    escaped:
        True if the target's distance from the start/centre exceeded
        ``monitor_radius`` at any tick -> it broke out of the outer
        surveillance ring.  Always a FAIL.
    re_contained:
        True if the target first excursed beyond ``block_radius`` and later
        returned to ``<= block_radius`` (the swarm re-captured it).
    held:
        True if the test window ended with the target inside the monitor ring
        and it never excursed beyond ``block_radius`` (the swarm held it from
        the start).  ``held`` and ``re_contained`` are mutually exclusive; the
        caller must guarantee exactly one of ``(escaped, re_contained, held)``
        is consistent (i.e. ``not escaped`` implies ``re_contained or held``).
    min_platform_dist:
        Smallest platform-to-target distance observed across the whole run
        (metres).  ``None`` if no ``/drone_states`` sample ever arrived.
    intercept_radius:
        Evidence threshold (metres).  A platform within this distance counts as
        "the swarm actually engaged the target".

    Returns
    -------
    ``"SUCCESS"`` | ``"FAIL"`` | ``"INVALID"``
    """
    if escaped:
        return "FAIL"
    # Not escaped -> the target was contained (re_contained or held).
    has_evidence = (
        min_platform_dist is not None and min_platform_dist <= intercept_radius
    )
    if not has_evidence:
        # Contained, but no platform ever got close enough to claim the
        # containment.  Ambiguous -> excluded from the success rate.
        return "INVALID"
    return "SUCCESS"
