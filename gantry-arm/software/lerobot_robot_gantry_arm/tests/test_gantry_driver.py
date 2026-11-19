"""GantryDriver against the real firmware core (C++), with simulated motors/switches."""

import pytest

from lerobot_robot_gantry_arm.gantry_driver import GantryDriver, GantryError, checksum, frame, parse_status


def test_frame_checksum_matches_firmware_algorithm():
    assert checksum("PING") == ord("P") ^ ord("I") ^ ord("N") ^ ord("G")
    assert frame("MOVE X1") == f"MOVE X1*{checksum('MOVE X1'):02X}\n".encode()


def test_parse_status():
    st = parse_status("st S=IDLE X=12.500 Z=3.000 H=X- L=---- T=2")
    assert st.state == "IDLE" and st.x == 12.5 and st.homed_x and not st.homed_z and st.tool == 2


def test_ping_and_checksummed_requests(fw):
    d = GantryDriver(fw)
    assert d.ping() == "0.1.0"
    assert all("*" in line for line in fw.sent)  # every request carries a checksum


def test_home_then_move_and_status(fw):
    d = GantryDriver(fw, timeout_s=5)
    with pytest.raises(GantryError) as e:
        d.move(x=100)
    assert e.value.code == "NOT_HOMED"
    d.home()
    assert any(ev == "ev HOMED Z" for ev in d.events) and any(ev == "ev HOMED X" for ev in d.events)
    assert d.events.index("ev HOMED Z") < d.events.index("ev HOMED X")  # Z clears first
    d.move(x=250.5, z=120, feed=150)
    st = d.wait_idle()
    assert st.x == pytest.approx(250.5, abs=1e-3) and st.z == pytest.approx(120, abs=1e-3)


def test_soft_limit_rejected_end_to_end(fw):
    d = GantryDriver(fw, timeout_s=5)
    d.home()
    with pytest.raises(GantryError) as e:
        d.move(x=900)
    assert e.value.code == "LIMIT"


def test_estop_latches_and_reset(fw):
    d = GantryDriver(fw, timeout_s=5)
    d.home()
    fw.set_estop(True)
    fw.tick(0.1)
    with pytest.raises(GantryError) as e:
        d.move(x=10)
    assert e.value.code == "ESTOP"
    assert "ev ESTOP" in d.events
    with pytest.raises(GantryError) as e:
        d.reset()
    assert e.value.code == "ESTOP_HW"
    fw.set_estop(False)
    d.reset()
    st = d.status()
    assert not st.homed_x and st.homed_z


def test_tool_id_and_led(fw):
    d = GantryDriver(fw)
    fw.set_tool_raw(1300)
    assert d.tool_id() == 3  # pen holder
    d.led("FAULT")
    with pytest.raises(ValueError):
        d.led("RAINBOW")
