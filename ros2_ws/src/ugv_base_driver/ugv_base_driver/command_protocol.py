"""Serial command protocol encoding as pure functions (no ROS dependencies).

The default protocol is a plain-text line:

    "L<left> R<right>\\n"

where <left>/<right> are wheel angular speeds in rad/s printed with two
decimals, e.g. "L6.67 R6.67\\n" for 0.5 m/s straight with 0.075 m wheels.

This framing matches the kind of simple text command most vendor base
boards (STM32/Arduino class) accept or can be adapted to accept.  When the
actual vendor protocol is known, add a new encoder here and select it via
the `protocol` ROS parameter — do NOT fork the node.

All encoders take floats and return bytes ready to be written to the port.
"""

from __future__ import annotations

from typing import Callable, Dict, Tuple

PROTOCOL_TEXT = "text"
PROTOCOL_TEXT_RPM = "text_rpm"


def encode_text(left_rad_s: float, right_rad_s: float) -> bytes:
    """Default text protocol: 'L<left> R<right>\\n' in rad/s."""
    return "L{:.2f} R{:.2f}\n".format(left_rad_s, right_rad_s).encode("ascii")


def encode_text_rpm(left_rad_s: float, right_rad_s: float) -> bytes:
    """Text protocol in RPM: 'L<left_rpm> R<right_rpm>\\n'."""
    rad_per_sec_to_rpm = 60.0 / (2.0 * 3.141592653589793)
    left_rpm = left_rad_s * rad_per_sec_to_rpm
    right_rpm = right_rad_s * rad_per_sec_to_rpm
    return "L{:.0f} R{:.0f}\n".format(left_rpm, right_rpm).encode("ascii")


def encode_stop() -> bytes:
    """Emergency/neutral command in the default text protocol."""
    return encode_text(0.0, 0.0)


_ENCODERS: Dict[str, Callable[[float, float], bytes]] = {
    PROTOCOL_TEXT: encode_text,
    PROTOCOL_TEXT_RPM: encode_text_rpm,
}


def get_encoder(name: str) -> Callable[[float, float], bytes]:
    """Look up an encoder by protocol name.

    Raises KeyError with the list of supported names when unknown, so a
    typo in the parameter fails loudly at startup instead of silently
    selecting a wrong framing.
    """
    try:
        return _ENCODERS[name]
    except KeyError:
        supported = ", ".join(sorted(_ENCODERS.keys()))
        raise KeyError(
            "unknown protocol {!r}; supported: {}".format(name, supported)
        ) from None


def encode_wheel_speeds(
    left_rad_s: float, right_rad_s: float, protocol: str
) -> Tuple[bytes, Callable[[float, float], bytes]]:
    """Encode wheel speeds under the given protocol.

    Returns (payload, encoder) so callers that need to emit stops can keep
    using the same encoder.
    """
    encoder = get_encoder(protocol)
    return encoder(left_rad_s, right_rad_s), encoder
