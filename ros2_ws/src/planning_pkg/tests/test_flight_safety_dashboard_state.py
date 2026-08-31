from planning_pkg.flight_safety_dashboard_state import DashboardState, is_loopback_host


def test_loopback_detection_accepts_only_local_hosts():
    assert is_loopback_host("127.0.0.1")
    assert is_loopback_host("::1")
    assert is_loopback_host("localhost")
    assert not is_loopback_host("0.0.0.0")
    assert not is_loopback_host("192.168.88.135")


def test_snapshot_starts_unavailable_without_video():
    state = DashboardState(max_video_bytes=1024)

    snapshot = state.status_snapshot()

    assert not snapshot["available"]
    assert snapshot["status_age_seconds"] is None
    assert snapshot["video"] == {"available": False, "age_seconds": None, "sequence": 0}
    assert snapshot["perception"] == {
        "available": False,
        "frame_idx": 0,
        "image_width": 0,
        "image_height": 0,
        "tracks": [],
        "age_seconds": None,
    }


def test_state_keeps_one_valid_jpeg_frame_and_notifies_waiter():
    state = DashboardState(max_video_bytes=1024)
    frame = b"\xff\xd8test-frame\xff\xd9"

    assert state.update_frame(frame)
    sequence, received = state.wait_for_frame(after_sequence=0, timeout=0.01)
    snapshot = state.status_snapshot()

    assert sequence == 1
    assert received == frame
    assert snapshot["video"]["available"]
    assert snapshot["video"]["sequence"] == 1
    assert snapshot["video"]["age_seconds"] is not None


def test_state_rejects_non_jpeg_and_oversized_frames():
    state = DashboardState(max_video_bytes=16)

    assert not state.update_frame(b"not-a-jpeg")
    # The cache enforces a 1 KiB floor even when an unsafe smaller limit is passed.
    assert not state.update_frame(b"\xff\xd8" + b"x" * 1021 + b"\xff\xd9")

    snapshot = state.status_snapshot()
    assert not snapshot["video"]["available"]


def test_status_snapshot_is_a_copy_of_the_latest_update():
    state = DashboardState(max_video_bytes=1024)
    state.update_status({"state_name": "LOCKED", "session_id": 9})

    snapshot = state.status_snapshot()
    snapshot["state_name"] = "MUTATED"

    assert state.status_snapshot()["state_name"] == "LOCKED"


def test_perception_snapshot_keeps_real_boxes_and_is_copy():
    state = DashboardState(max_video_bytes=1024)
    state.update_perception(
        {
            "frame_idx": 12,
            "image_width": 1280,
            "image_height": 720,
            "tracks": [
                {
                    "target_id": 7,
                    "label": "car",
                    "confidence": 0.91,
                    "bbox_x1": 100.0,
                    "bbox_y1": 120.0,
                    "bbox_x2": 260.0,
                    "bbox_y2": 280.0,
                }
            ],
        }
    )

    snapshot = state.status_snapshot()
    assert snapshot["perception"]["available"]
    assert snapshot["perception"]["image_width"] == 1280
    assert snapshot["perception"]["tracks"][0]["target_id"] == 7
    snapshot["perception"]["tracks"][0]["label"] = "mutated"
    assert state.status_snapshot()["perception"]["tracks"][0]["label"] == "car"
