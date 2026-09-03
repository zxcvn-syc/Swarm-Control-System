"""Unit tests for serial command protocol encoding (no ROS dependencies).

Run locally with:  python -m pytest tests/test_command_protocol.py
"""

import pytest

from ugv_base_driver.command_protocol import (
    PROTOCOL_TEXT,
    encode_stop,
    encode_text,
    encode_text_rpm,
    encode_wheel_speeds,
    get_encoder,
)


def test_text_protocol_basic():
    assert encode_text(6.6667, 6.6667) == b"L6.67 R6.67\n"


def test_text_protocol_negative_and_zero():
    assert encode_text(-1.0, 0.0) == b"L-1.00 R0.00\n"
    assert encode_text(0.0, 0.0) == b"L0.00 R0.00\n"


def test_stop_is_zero_command():
    assert encode_stop() == b"L0.00 R0.00\n"


def test_text_rpm_value():
    payload = encode_text_rpm(2 * 3.141592653589793, -2 * 3.141592653589793)
    assert payload == b"L60 R-60\n"


def test_encode_wheel_speeds_returns_payload_and_encoder():
    payload, encoder = encode_wheel_speeds(1.0, 1.0, PROTOCOL_TEXT)
    assert payload == b"L1.00 R1.00\n"
    assert encoder(0.0, 0.0) == b"L0.00 R0.00\n"


def test_unknown_protocol_raises_with_supported_list():
    with pytest.raises(KeyError) as excinfo:
        get_encoder("vendor_v2")
    assert "text" in str(excinfo.value)


def test_get_encoder_returns_callables():
    assert callable(get_encoder(PROTOCOL_TEXT))
    assert callable(get_encoder("text_rpm"))
