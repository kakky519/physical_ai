# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.assets import ArticulationCfg
from isaaclab.utils.configclass import configclass

from isaaclab_tasks.contrib.stack import mdp
from isaaclab_tasks.contrib.stack.stack_env_cfg import (
    StackEnvCfg,
    StackEventCfg,
    apply_default_semantics,
    make_ee_frame_cfg,
)
from isaaclab.managers import EventTermCfg as EventTerm

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg

##
# Pre-defined configs
##
from .robot import ROBOT_CFG  # isort: skip

# Default arm + gripper joint pose
_ROBOT_STACK_IK_REL_INIT_JOINT_POS: dict[str, float] = {
    "s_joint": 0.0,
    "l_joint": 0.0,
    "u_joint": 0.0,
    "r_joint": 0.0,
    "b_joint": 0.0,
    "t_joint": 0.0,
    "panda_finger_joint.*": 0.04,
}


def _make_tray_cfg(name: str, usd_path: str) -> RigidObjectCfg:
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{name}",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0.3, -3.0], rot=[0, 0, 0, 1]),
        spawn=UsdFileCfg(
            usd_path=usd_path,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )


def _fix_base_joint_pos(env, env_ids):
    from pxr import UsdGeom
    from isaaclab.sim.utils import find_matching_prims

    robot = env.scene["robot"]
    prim_path = f"{robot.cfg.prim_path}/NEX10"  # 実際に translate が入っていたプリムパスに合わせる
    for prim in find_matching_prims(prim_path):
        xform = UsdGeom.Xformable(prim)
        translate_op = None
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break
        if translate_op:
            before = translate_op.Get()
            translate_op.Set((-0.2, 0.0, 0.5))
            print(f"[DEBUG] {prim.GetPath()}: before={before}, after={translate_op.Get()}")


@configclass
class CubeStackEnvCfg(StackEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        self.events = StackEventCfg()

        self.scene.robot = ROBOT_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot",
            init_state=ArticulationCfg.InitialStateCfg(joint_pos=_ROBOT_STACK_IK_REL_INIT_JOINT_POS),
        )
        # Tag the table / ground / robot semantic classes.
        apply_default_semantics(self.scene)

        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["[slurbt]_joint"], scale=0.5, use_default_offset=True
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["panda_finger.*"],
            open_command_expr={"panda_finger_.*": 0.04},
            close_command_expr={"panda_finger_.*": 0.0},
        )
        # utilities for gripper status check
        self.gripper_joint_names = ["panda_finger_.*"]
        self.gripper_open_val = 0.04
        self.gripper_threshold = 0.005

        # End-effector frame (the shared cubes/spawns come from the base scene).
        self.scene.ee_frame = make_ee_frame_cfg(
            base_prim_path="{ENV_REGEX_NS}/Robot/NEX10/base_link",
            target_specs=[
                ("{ENV_REGEX_NS}/Robot/NEX10/panda_hand/panda_hand_group/panda_hand_link", "end_effector", (0.0, 0.0, 0.1034)),
                ("{ENV_REGEX_NS}/Robot/NEX10/panda_hand/panda_hand_group/panda_rightfinger", "tool_rightfinger", (0.0, 0.0, 0.046)),
                ("{ENV_REGEX_NS}/Robot/NEX10/panda_hand/panda_hand_group/panda_leftfinger", "tool_leftfinger", (0.0, 0.0, 0.046)),
            ],
            marker_scale=(0.1, 0.1, 0.1),
        )

        self.events.robot_init_position = EventTerm(
            func=_fix_base_joint_pos,
            mode="prestartup"
        )

        # --- トレー3種を配置(最初は全部隠しておく) ---
        self.scene.tray_blue = _make_tray_cfg("Tray_Blue", "source/isaaclab_tasks/isaaclab_tasks/contrib/stack/config/robot/usd/tray_blue.usd")
        self.scene.tray_red = _make_tray_cfg("Tray_Red", "source/isaaclab_tasks/isaaclab_tasks/contrib/stack/config/robot/usd/tray_red.usd")
        self.scene.tray_green = _make_tray_cfg("Tray_Green", "source/isaaclab_tasks/isaaclab_tasks/contrib/stack/config/robot/usd/tray_green.usd")

