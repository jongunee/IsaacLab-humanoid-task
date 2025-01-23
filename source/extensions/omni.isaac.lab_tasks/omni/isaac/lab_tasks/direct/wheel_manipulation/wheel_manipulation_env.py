# wheel_manipulation_env.py
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
import omni.isaac.core.utils.torch as torch_utils
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.sim import SimulationCfg
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.envs import DirectRLEnv, DirectRLEnvCfg
import omni.isaac.lab.sim as sim_utils


##################################################################
# 2) WheelManipEnv 구현
##################################################################
class WheelManipEnv(DirectRLEnv):
    """
    바퀴 + 매니퓰레이터 로봇의 관측, 보상, 종료 등을 정의하는 기본 환경 클래스
    다른 특정 로봇에 맞춰 상속/재설정해서 쓰면 됨.
    """
    cfg: DirectRLEnvCfg

    def __init__(self, cfg: DirectRLEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # 파라미터 세팅
        self.action_scale = self.cfg.action_scale
        self.joint_gears = torch.tensor(self.cfg.joint_gears, dtype=torch.float32, device=self.device)

        # 나중에 로봇 조인트 인덱스를 찾아 저장할 변수
        self._joint_dof_idx, _ = self.robot.find_joints(".*")

        # (데모) 목표점을 (5,0,0) 근처로 설정
        self.targets = torch.tensor([1000, 0, 0], dtype=torch.float32, device=self.sim.device).repeat(
            (self.num_envs, 1)
        )
        self.targets += self.scene.env_origins

    def _setup_scene(self):
        """
        씬 구성: 지면 배치 + 로봇 Articulation 생성
        본 클래스가 직접 써도 되고, 상속받은 쪽에서 오버라이드해도 됨.
        """
        # 로봇 생성
        self.robot = Articulation(self.cfg.robot)

        # 지면(terrain) 설정
        self.cfg.terrain.num_envs = self.scene.cfg.num_envs
        self.cfg.terrain.env_spacing = self.scene.cfg.env_spacing
        self.terrain = self.cfg.terrain.class_type(self.cfg.terrain)

        # 리플리케이션
        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=[self.cfg.terrain.prim_path])

        # 로봇 등록
        self.scene.articulations["robot"] = self.robot

        # 옵션: 조명
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    def _pre_physics_step(self, actions: torch.Tensor):
        # RL 액션 저장
        self.actions = actions.clone()
        self._apply_action()

    def _apply_action(self):
        # 예: 토크 제어라고 가정
        forces = self.action_scale * self.joint_gears * self.actions
        self.robot.set_joint_effort_target(forces, joint_ids=self._joint_dof_idx)

    def _get_observations(self) -> dict:
        # base pose
        base_pos = self.robot.data.root_link_pos_w  # (num_envs, 3)
        base_quat = self.robot.data.root_link_quat_w  # (num_envs, 4)

        # 속도
        base_linvel = self.robot.data.root_com_lin_vel_w  # (num_envs, 3)
        base_angvel = self.robot.data.root_com_ang_vel_w  # (num_envs, 3)

        # 조인트
        dof_pos = self.robot.data.joint_pos
        dof_vel = self.robot.data.joint_vel

        # 회전 각도(roll,pitch,yaw)
        roll, pitch, yaw = euler_from_quat(base_quat)

        # 목표 거리
        to_target_xy = self.targets[:, :2] - base_pos[:, :2]
        dist_to_target = torch.norm(to_target_xy, p=2, dim=-1, keepdim=True)

        obs = torch.cat([
            base_pos,
            base_linvel,
            base_angvel,
            dof_pos,
            dof_vel,
            roll.unsqueeze(-1),
            pitch.unsqueeze(-1),
            yaw.unsqueeze(-1),
            dist_to_target,
            self.actions,
        ], dim=-1)
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        # 거리 기반 보상
        base_pos = self.robot.data.root_link_pos_w
        to_target_xy = self.targets[:, :2] - base_pos[:, :2]
        dist = torch.norm(to_target_xy, p=2, dim=-1)
        dist_reward = self.cfg.dist_reward_scale * (-dist)

        # 액션 사용 패널티
        actions_cost = torch.sum(self.actions**2, dim=-1)
        action_penalty = self.cfg.action_cost_scale * actions_cost

        # 살아있는 보상
        alive_reward = self.cfg.alive_reward_scale * torch.ones_like(dist)

        total_reward = dist_reward + alive_reward - action_penalty

        # reset된 env는 보상 0 (또는 -큰값)
        total_reward = torch.where(self.reset_terminated, torch.zeros_like(total_reward), total_reward)
        return total_reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        # 에피소드 타임아웃
        time_out = self.episode_length_buf >= self.max_episode_length - 1

        # roll/pitch가 너무 크면 넘어졌다고 판단
        base_quat = self.robot.data.root_link_quat_w
        roll, pitch, _ = euler_from_quat(base_quat)
        tipped_over = (torch.abs(roll) > self.cfg.termination_roll_pitch) | \
                      (torch.abs(pitch) > self.cfg.termination_roll_pitch)

        died = tipped_over
        return died, time_out

    def _reset_idx(self, env_ids: torch.Tensor | None):
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self.robot._ALL_INDICES

        self.robot.reset(env_ids)
        super()._reset_idx(env_ids)

        joint_pos = self.robot.data.default_joint_pos[env_ids]
        joint_vel = self.robot.data.default_joint_vel[env_ids]
        default_root_state = self.robot.data.default_root_state[env_ids]

        default_root_state[:, :3] += self.scene.env_origins[env_ids]

        self.robot.write_root_link_pose_to_sim(default_root_state[:, :7], env_ids)
        self.robot.write_root_com_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

        self._compute_intermediate_values()

    def _compute_intermediate_values(self):
        pass


def euler_from_quat(quat: torch.Tensor):
    """
    quat: (num_envs, 4) in wxyz order -> (roll, pitch, yaw)
    """
    w = quat[:, 0]
    x = quat[:, 1]
    y = quat[:, 2]
    z = quat[:, 3]

    t0 = 2.0*(w*x + y*z)
    t1 = 1.0 - 2.0*(x*x + y*y)
    roll = torch.atan2(t0, t1)

    t2 = 2.0*(w*y - z*x)
    t2 = torch.clip(t2, -1.0, 1.0)
    pitch = torch.asin(t2)

    t3 = 2.0*(w*z + x*y)
    t4 = 1.0 - 2.0*(y*y + z*z)
    yaw = torch.atan2(t3, t4)

    return roll, pitch, yaw
