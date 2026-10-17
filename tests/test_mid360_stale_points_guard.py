import pytest

from isaac_sim.carbot_mid360 import (
    SKIP_DROPPING_INVALID_POINTS_KIT_ARG,
    SKIP_DROPPING_INVALID_POINTS_SETTING,
    require_invalid_lidar_returns_kept,
)


class FakeSettings(dict):
    pass


def test_kit_argument_enables_the_checked_setting():
    assert SKIP_DROPPING_INVALID_POINTS_KIT_ARG == (
        "--/app/sensors/nv/lidar/skipDroppingInvalidPoints=true"
    )


def test_guard_accepts_kit_started_with_invalid_returns():
    require_invalid_lidar_returns_kept(
        FakeSettings({SKIP_DROPPING_INVALID_POINTS_SETTING: True})
    )


@pytest.mark.parametrize("value", [None, False, "true"])
def test_guard_rejects_default_kit(value):
    settings = FakeSettings()
    if value is not None:
        settings[SKIP_DROPPING_INVALID_POINTS_SETTING] = value
    with pytest.raises(RuntimeError, match="#107"):
        require_invalid_lidar_returns_kept(settings)
