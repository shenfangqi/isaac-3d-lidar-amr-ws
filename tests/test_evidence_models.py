import json
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

from isaac_sim.evidence_models import (
    EncoderObservationModel,
    EvidenceActuatorModel,
    ImuObservationModel,
)


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "configs/carbot/evidence_degraded.yaml"


@pytest.fixture(scope="module")
def profile():
    return yaml.safe_load(PROFILE.read_text(encoding="utf-8"))


def test_generated_profile_is_current():
    subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts/build_sim_real_evidence_profile.py"),
            "--check",
        ],
        check=True,
    )


def test_actuator_has_delay_directional_gain_and_stop_tail(profile):
    model = EvidenceActuatorModel(profile["actuator"])
    early = [model.update(0.1, 0.0, 0.01)[0] for _ in range(7)]
    assert early == [0.0] * 7
    for _ in range(200):
        linear, _ = model.update(0.1, 0.0, 0.01)
    assert linear == pytest.approx(0.0882, rel=0.01)
    stop_samples = []
    for _ in range(100):
        stop_samples.append(model.update(0.0, 0.0, 0.01)[0])
    assert stop_samples[10] > 0.0
    assert stop_samples[-1] == 0.0


def test_final_acceptance_gains_are_encoded(profile):
    acceptance = json.loads(
        (
            ROOT
            / "calibration_data/2026-09-23_cmd_comp_final"
            / "physical_final_04_analysis.json"
        ).read_text(encoding="utf-8")
    )
    actuator = profile["actuator"]
    ideal_deg = 51.566200949346886
    assert actuator["yaw_gain"]["left"] * ideal_deg == pytest.approx(
        acceptance["left_angle_deg"]
    )
    assert (
        actuator["yaw_gain"]["right_after_compensation"] * ideal_deg
        == pytest.approx(abs(acceptance["right_angle_deg"]))
    )


def test_encoder_is_quantized_and_reproducible(profile):
    first = EncoderObservationModel(profile["encoder"])
    second = EncoderObservationModel(profile["encoder"])
    samples_a = []
    samples_b = []
    for _ in range(500):
        samples_a.append(first.update(1.0, 1.2, 0.01))
        samples_b.append(second.update(1.0, 1.2, 0.01))
    assert samples_a == samples_b
    emitted = [sample for sample in samples_a if sample is not None]
    assert len(emitted) >= 245
    assert all(isinstance(sample.left_ticks, int) for sample in emitted)


def test_imu_noise_is_seeded_and_near_measured_bias(profile):
    first = ImuObservationModel(profile["imu"])
    second = ImuObservationModel(profile["imu"])
    values_a = [first.yaw_rate(0.0) for _ in range(1000)]
    values_b = [second.yaw_rate(0.0) for _ in range(1000)]
    assert values_a == values_b
    mean = sum(values_a) / len(values_a)
    assert mean == pytest.approx(
        profile["imu"]["gyro_z_residual_bias_rad_s"], abs=1.0e-4
    )
