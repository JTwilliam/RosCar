from learm_bridge.dashboard import DashboardSnapshot, render_dashboard


def test_dashboard_renders_four_wheels_and_link_health():
    snapshot = DashboardSnapshot(
        encoders=(120, -45, 900, 0),
        deltas=(5, -2, 10, 0),
        firmware_timestamp_ms=3210,
        sample_sequence=17,
        receive_rate_hz=49.8,
        frame_age_s=0.03,
        fault_flags=0x0010,
        safety_state=7,
        frames_valid=465,
        frames_dropped=2,
        crc_errors=1,
        sequence_drops=3,
        serial_port="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        baud_rate=115200,
    )

    rendered = render_dashboard(snapshot)

    assert "Wheel 0" in rendered and "120" in rendered and "+5" in rendered
    assert "Wheel 1" in rendered and "-45" in rendered and "-2" in rendered
    assert "Wheel 2" in rendered and "900" in rendered and "+10" in rendered
    assert "Wheel 3" in rendered
    assert "49.8 Hz" in rendered
    assert "0x0010" in rendered
    assert "FAULT" in rendered
    assert "CRC errors" in rendered
