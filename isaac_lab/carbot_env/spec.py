"""Load Phase F contracts without importing Isaac Lab or starting Kit."""

from copy import deepcopy
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ENVIRONMENT_SPEC_PATH = Path(__file__).with_name("environment.yaml")
HARDWARE_BACKLOG_PATH = Path(__file__).with_name(
    "hardware_calibration_backlog.yaml"
)
ROBOT_PARAMETERS_PATH = (
    PROJECT_ROOT / "src/carbot_description/config/carbot_parameters.yaml"
)
SIM_PROFILE_PATH = PROJECT_ROOT / "configs/carbot/sim.yaml"


def _load_yaml(path):
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def load_robot_parameters():
    """Return the canonical Carbot parameter tree."""
    return _load_yaml(ROBOT_PARAMETERS_PATH)


def load_environment_spec():
    """Resolve the Phase F spec from canonical robot and sim parameters."""
    spec = deepcopy(_load_yaml(ENVIRONMENT_SPEC_PATH))
    robot = load_robot_parameters()
    simulation = _load_yaml(SIM_PROFILE_PATH)
    action = spec["action"]
    action["limits"] = {
        "linear_velocity_mps": simulation["velocity_limits"]["linear_mps"],
        "angular_velocity_rad_s": simulation["velocity_limits"][
            "angular_rad_s"
        ],
        "linear_acceleration_mps2": robot["control"][
            "max_linear_acceleration_mps2"
        ],
        "angular_acceleration_rad_s2": robot["control"][
            "max_angular_acceleration_rad_s2"
        ],
        "wheel_velocity_rad_s": robot["control"][
            "max_wheel_velocity_rad_s"
        ],
        "watchdog_timeout_s": robot["control"]["cmd_vel_timeout_s"],
    }
    action["kinematics"] = {
        "effective_track_separation_m": robot["kinematics"][
            "effective_track_separation_m"
        ],
        "effective_sprocket_radius_m": robot["kinematics"][
            "effective_sprocket_radius_m"
        ],
        "wheel_joint_coordinate_sign": robot["simulation"][
            "wheel_joint_coordinate_sign"
        ],
    }
    spec["simulation"]["control_mode"] = robot["simulation"][
        "control_mode"
    ]
    spec["domain_randomization"] = deepcopy(robot["domain_randomization"])
    return spec


def load_hardware_calibration_backlog():
    """Return persistent Sim-to-Real blockers that must remain visible."""
    return _load_yaml(HARDWARE_BACKLOG_PATH)
