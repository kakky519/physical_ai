import torch
import torch.nn.functional as F

TRAY_NAMES = ["tray_blue", "tray_red", "tray_green"]  # cube_1=青, cube_2=赤, cube_3=緑 に対応
CUBE_NAMES = ["cube_1", "cube_2", "cube_3"]

TRAY_POS = (0.8, 0.0, 0.0)  # x, y, z(env原点からの相対位置)
HIDDEN_Z = -3.0


def get_target_tray_id(env):
    """今表示されているトレーのID(0=青, 1=赤, 2=緑)を、トレーの高さから求める。"""
    z = torch.stack([env.scene[n].data.root_pos_w.torch[:, 2] for n in TRAY_NAMES], dim=1)
    return torch.argmax(z, dim=1)


def randomize_tray(env, env_ids, visible_pos=TRAY_POS, hidden_z=HIDDEN_Z):
    """リセット時に出すトレーを1色ランダムに決め、他は地面の下に隠す。"""
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    elif isinstance(env_ids, slice):
        env_ids = torch.arange(env.num_envs, device=env.device)[env_ids]

    chosen = torch.randint(0, 3, (len(env_ids),), device=env.device)

    for i, name in enumerate(TRAY_NAMES):
        pos = env.scene.env_origins[env_ids] + torch.tensor(visible_pos, device=env.device)
        pos[chosen != i, 2] = hidden_z
        quat = torch.zeros(len(env_ids), 4, device=env.device)
        quat[:, 3] = 1.0  # xyzw の単位回転
        env.scene[name].write_root_pose_to_sim_index(
            root_pose=torch.cat([pos, quat], dim=-1), env_ids=env_ids
        )


def target_tray_onehot(env):
    """観測: 今出ているトレーの色(ワンホット3次元)。"""
    return F.one_hot(get_target_tray_id(env), 3).float()


def target_cube_grasped(env, diff_threshold=0.06):
    """サブタスク信号: 出ているトレーと同じ色のブロックを掴んでいるか。"""
    tray_id = get_target_tray_id(env)
    cubes = torch.stack([env.scene[n].data.root_pos_w.torch for n in CUBE_NAMES], dim=1)
    target = cubes[torch.arange(env.num_envs, device=env.device), tray_id]
    ee_pos = env.scene["ee_frame"].data.target_pos_w.torch[:, 0, :]
    grasped = torch.linalg.vector_norm(target - ee_pos, dim=1) < diff_threshold

    robot = env.scene["robot"]
    ids, _ = robot.find_joints(env.cfg.gripper_joint_names)
    for j in ids:
        moved = torch.abs(robot.data.joint_pos.torch[:, j] - env.cfg.gripper_open_val) > env.cfg.gripper_threshold
        grasped = grasped & moved
    return grasped


def cube_in_target_tray(env, tray_xy=TRAY_POS[:2], xy_threshold=0.07, gripper_open_val=0.04, atol=0.002):
    """成功判定: 出ているトレーと同じ色のブロックがトレー内にあり、グリッパが開いている。"""
    tray_id = get_target_tray_id(env)
    cubes = torch.stack([env.scene[n].data.root_pos_w.torch for n in CUBE_NAMES], dim=1)  # (N,3,3)
    target = cubes[torch.arange(env.num_envs, device=env.device), tray_id]
    tray_w = env.scene.env_origins[:, :2] + torch.tensor(tray_xy, device=env.device)
    in_tray = torch.norm(target[:, :2] - tray_w, dim=1) < xy_threshold

    robot = env.scene["robot"]
    ids, _ = robot.find_joints(env.cfg.gripper_joint_names)
    opened = torch.ones(env.num_envs, dtype=torch.bool, device=env.device)
    for j in ids:
        opened &= torch.isclose(
            robot.data.joint_pos.torch[:, j],
            torch.tensor(gripper_open_val, device=env.device),
            atol=atol,
        )
    return in_tray & opened