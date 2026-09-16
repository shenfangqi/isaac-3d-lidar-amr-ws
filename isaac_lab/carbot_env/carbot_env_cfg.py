"""Isaac Lab manager-based Carbot navigation environment configuration."""

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import RayCasterCfg, patterns
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass

from . import mdp
from .spec import PROJECT_ROOT, load_environment_spec, load_robot_parameters
from .tasks.goal_navigation import CommandsCfg, RewardsCfg, TerminationsCfg


SPEC = load_environment_spec()
ROBOT = load_robot_parameters()
ACTION = SPEC["action"]
LIMITS = ACTION["limits"]
KINEMATICS = ACTION["kinematics"]
SIMULATION = SPEC["simulation"]
LIDAR = SPEC["observations"]["lidar"]
HEIGHT_SCAN = SPEC["observations"]["height_scan"]
ROBOT_USD = PROJECT_ROOT / Path(SIMULATION["robot_usd"])
LIDAR_Z_FROM_BASE_LINK_M = (
    ROBOT["sensors"]["mid360"]["housing_top_height_from_ground_m"]
    - ROBOT["sensors"]["mid360"]["housing_height_m"]
    - ROBOT["geometry"]["base_link_height_m"]
)


@configclass
class CarbotSceneCfg(InteractiveSceneCfg):
    """Flat smoke-test scene; terrain and task remain replaceable."""

    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        env_spacing=SIMULATION["env_spacing_m"],
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=ROBOT["dynamics"]["static_friction"],
            dynamic_friction=ROBOT["dynamics"]["dynamic_friction"],
            restitution=ROBOT["dynamics"]["restitution"],
        ),
        debug_vis=False,
    )
    robot = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(ROBOT_USD),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            joint_pos={".*_wheel_joint": 0.0},
            joint_vel={".*_wheel_joint": 0.0},
        ),
        actuators={
            "tracks": ImplicitActuatorCfg(
                joint_names_expr=[".*_wheel_joint"],
                effort_limit_sim=ROBOT["dynamics"][
                    "per_side_effort_limit_nm"
                ]
                / 6.0,
                velocity_limit_sim=LIMITS["wheel_velocity_rad_s"],
                stiffness=0.0,
                damping=ROBOT["dynamics"]["joint_damping_nm_s_rad"],
            )
        },
    )
    lidar = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link",
        offset=RayCasterCfg.OffsetCfg(
            pos=(
                ROBOT["sensors"]["mid360"][
                    "mount_translation_xy_from_base_link_m"
                ][0],
                ROBOT["sensors"]["mid360"][
                    "mount_translation_xy_from_base_link_m"
                ][1],
                LIDAR_Z_FROM_BASE_LINK_M,
            )
        ),
        ray_alignment="yaw",
        pattern_cfg=patterns.LidarPatternCfg(
            channels=1,
            vertical_fov_range=(0.0, 0.0),
            horizontal_fov_range=tuple(LIDAR["horizontal_fov_deg"]),
            # Isaac Lab samples both 360-degree endpoints and removes the
            # duplicate, so request N+1 samples to retain exactly N rays.
            horizontal_res=360.0 / (LIDAR["ray_count"] + 1),
        ),
        max_distance=LIDAR["maximum_range_m"],
        mesh_prim_paths=["/World/ground"],
        debug_vis=False,
    )
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.5)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(
            resolution=HEIGHT_SCAN["resolution_m"],
            size=(
                (HEIGHT_SCAN["grid_size"][0] - 1)
                * HEIGHT_SCAN["resolution_m"],
                (HEIGHT_SCAN["grid_size"][1] - 1)
                * HEIGHT_SCAN["resolution_m"],
            ),
        ),
        max_distance=2.0,
        mesh_prim_paths=["/World/ground"],
        debug_vis=False,
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(intensity=750.0),
    )


@configclass
class ActionsCfg:
    """Only the deployable high-level Twist action is enabled."""

    twist = mdp.CarbotTwistActionCfg(
        asset_name="robot",
        max_linear_velocity_mps=LIMITS["linear_velocity_mps"],
        max_angular_velocity_rad_s=LIMITS["angular_velocity_rad_s"],
        max_linear_acceleration_mps2=LIMITS["linear_acceleration_mps2"],
        max_angular_acceleration_rad_s2=LIMITS[
            "angular_acceleration_rad_s2"
        ],
        max_wheel_velocity_rad_s=LIMITS["wheel_velocity_rad_s"],
        effective_track_separation_m=KINEMATICS[
            "effective_track_separation_m"
        ],
        effective_sprocket_radius_m=KINEMATICS[
            "effective_sprocket_radius_m"
        ],
        wheel_joint_coordinate_sign=KINEMATICS[
            "wheel_joint_coordinate_sign"
        ],
        watchdog_timeout_s=LIMITS["watchdog_timeout_s"],
        ideal_kinematic=(
            ROBOT["simulation"]["control_mode"]
            == "ideal_kinematic_tracked_differential"
        ),
    )


@configclass
class ObservationsCfg:
    """Policy observations intentionally exclude Isaac ground truth."""

    @configclass
    class PolicyCfg(ObsGroup):
        goal_pose_body = ObsTerm(
            func=mdp.generated_commands,
            params={"command_name": "pose_command"},
        )
        planar_velocity = ObsTerm(func=mdp.planar_velocity)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        lidar_ranges = ObsTerm(
            func=mdp.lidar_ranges,
            params={
                "sensor_cfg": SceneEntityCfg("lidar"),
                "maximum_range_m": LIDAR["maximum_range_m"],
            },
            clip=(0.0, LIDAR["maximum_range_m"]),
        )
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )
        previous_action = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Episode reset only; randomization remains disabled until calibration."""

    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": (-0.25, 0.25),
                "y": (-0.25, 0.25),
                "yaw": (-3.141592653589793, 3.141592653589793),
            },
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )
    reset_joints = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "position_range": (0.0, 0.0),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class CarbotEnvCfg(ManagerBasedRLEnvCfg):
    """Phase F foundation environment; not a deployable trained policy."""

    scene: CarbotSceneCfg = CarbotSceneCfg(
        num_envs=SIMULATION["num_envs"],
        env_spacing=SIMULATION["env_spacing_m"],
    )
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        self.decimation = SIMULATION["control_decimation"]
        self.episode_length_s = SIMULATION["episode_length_s"]
        self.sim.dt = SIMULATION["physics_dt_s"]
        self.sim.render_interval = self.decimation
        self.scene.lidar.update_period = self.decimation * self.sim.dt
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt
        self.viewer.eye = (4.0, 4.0, 3.0)


@configclass
class CarbotEnvCfgPlay(CarbotEnvCfg):
    """Small deterministic configuration for visual/smoke validation."""

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 1
        self.observations.policy.enable_corruption = False
