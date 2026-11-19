import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent), str(HERE)]


@pytest.fixture
def fw():
    from firmware_bridge import FirmwareTransport, build_lib

    lib = build_lib()
    if lib is None:
        pytest.skip("g++ not available")
    t = FirmwareTransport(lib)
    yield t
    t.close()
