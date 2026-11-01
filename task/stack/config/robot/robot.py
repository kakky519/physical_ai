import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

ROBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"source/isaaclab_tasks/isaaclab_tasks/contrib/stack/config/robot/usd/RB/NEX10_hand_noFix.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "s_joint": 0.0,
            "l_joint": 0.0,
            "u_joint": 0.0,
            "r_joint": 0.0,
            "b_joint": 0.0,
            "t_joint": 0.0,
            "panda_finger_joint.*": 0.04,
        },
        # pos=(-0.1, 0.0, -0.5)
        pos=(0.1, 0.0, -0.5)
    ),
    actuators={
        "arm_joint": ImplicitActuatorCfg(
            joint_names_expr=["[slurbt]_joint"],
            joint_effort_limit=87.0,
            stiffness=1000.0,
            damping=50.0,
            armature=1e-3,
        ),
        "panda_hand": ImplicitActuatorCfg(
            joint_names_expr=["panda_finger_joint.*"],
            joint_effort_limit=200.0,
            stiffness=2e3,
            damping=1e2,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)

ROBOT_HIGH_PD_CFG = ROBOT_CFG.copy()
ROBOT_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
