import pytest

from scripts.compare_real_isaac_response import relative_error


def test_relative_error_preserves_signed_motion_direction():
    assert relative_error(-0.4, -0.5) == pytest.approx(-0.2)


def test_relative_error_handles_missing_or_zero_truth():
    assert relative_error(1.0, None) is None
    assert relative_error(1.0, 0.0) is None
