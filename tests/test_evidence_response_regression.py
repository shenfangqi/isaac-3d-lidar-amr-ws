from pathlib import Path

from scripts.regress_evidence_response import run_trial
from isaac_sim.evidence_models import load_evidence_profile
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_evidence_response_has_latency_and_stop_tail():
    parameters = yaml.safe_load(
        (PROJECT_ROOT / "src/carbot_description/config/carbot_parameters.yaml")
        .read_text(encoding="utf-8")
    )
    evidence = load_evidence_profile(
        PROJECT_ROOT / parameters["simulation"]["evidence_profile"]
    )
    result = run_trial(parameters, evidence, 0.05, 0.0, 3.0)
    assert result["first_motion_latency_s"] >= 0.06
    assert 0.57 <= result["stop_tail_s"] <= 0.92
