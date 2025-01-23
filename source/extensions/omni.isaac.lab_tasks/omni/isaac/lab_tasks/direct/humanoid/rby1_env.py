# rby1_env.py
from __future__ import annotations

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationCfg
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.envs import DirectRLEnvCfg

from omni.isaac.lab_tasks.direct.wheel_manipulation.wheel_manipulation_env import WheelManipEnv
from omni.isaac.lab_assets.rainbowrobotics import RBY1_CFG


@configclass
class RBY1EnvCfg(DirectRLEnvCfg):
    # env
    episode_length_s = 15.0
    decimation = 2
    action_scale = 1.0
    action_space = 28
    observation_space = 97
    state_space = 0

    # (1) 시뮬 파라미터: 120Hz
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=2)

    # (2) 지형 설정
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="average",
            restitution_combine_mode="average",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )

    # (3) Scene 설정: 4096 env, env 간격 4.0, 물리복제 On
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=4.0, replicate_physics=True)

    # (4) RBY1 로봇 설정
    robot: ArticulationCfg = RBY1_CFG.replace(prim_path="/World/envs/env_.*/Robot")

    # (5) 조인트 기어비: 실제 RBY1 조인트 순서대로 28개
    joint_gears: list = [
        50.0,  # right_wheel
        50.0,  # left_wheel
        50.0,  # torso_0
        50.0,  # torso_1
        50.0,  # torso_2
        50.0,  # torso_3
        50.0,  # torso_4
        50.0,  # torso_5
        50.0,  # right_arm_0
        50.0,  # right_arm_1
        50.0,  # right_arm_2
        50.0,  # right_arm_3
        50.0,  # right_arm_4
        50.0,  # right_arm_5
        50.0,  # right_arm_6
        50.0,  # left_arm_0
        50.0,  # left_arm_1
        50.0,  # left_arm_2
        50.0,  # left_arm_3
        50.0,  # left_arm_4
        50.0,  # left_arm_5
        50.0,  # left_arm_6
        50.0,  # gripper_finger_r1
        50.0,  # gripper_finger_r2
        50.0,  # gripper_finger_l1
        50.0,  # gripper_finger_l2
        50.0,  # head_0
        50.0,  # head_1
    ]

    # (6) 휠/매니퓰레이터용 파라미터(원하면 값 조정)
    termination_roll_pitch: float = 0.8
    dist_reward_scale: float = 1.0
    action_cost_scale: float = 0.01
    alive_reward_scale: float = 2.0
    contact_force_scale: float = 0.01


class RBY1Env(WheelManipEnv):
    """
    WheelManipEnv를 상속받아, RBY1용으로 최종 완성된 RL 환경
    """
    cfg: RBY1EnvCfg

    def __init__(self, cfg: RBY1EnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        # 여기서 추가 초기화 가능 (필요하면)
        # self._joint_dof_idx = self.robot.find_joints(".*")[0]  # joint 인덱스 저장
        # etc.
