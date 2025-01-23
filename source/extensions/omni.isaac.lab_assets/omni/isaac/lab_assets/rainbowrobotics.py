from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.sim import UsdFileCfg, RigidBodyPropertiesCfg, ArticulationRootPropertiesCfg
from omni.isaac.lab.actuators import ImplicitActuatorCfg

RBY1_CFG = ArticulationCfg(
    spawn=UsdFileCfg(
        usd_path="/home/jwpark/rl/IsaacLab/source/extensions/omni.isaac.lab_assets/data/Robots/rainbowrobotics/rby1.usd",
        activate_contact_sensors=True,
        rigid_props=RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.01,
            angular_damping=0.01,
            max_linear_velocity=50.0,
            max_angular_velocity=50.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.3),    # base를 지면 위 15cm 정도로 시작
        joint_pos={
            "right_wheel": 0.0,
            "left_wheel": 0.0,
            "torso_.*": 0.0,         # torso 관절이 여러 개라면 0도
            ".*_arm_.*": 0.0,        # 팔 관절
            "gripper_finger_.*": 0.0,# 그리퍼
            "head_.*": 0.0,          # 머리 관절
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=["right_wheel", "left_wheel"],
            effort_limit=20.0,
            velocity_limit=10.0,
            stiffness={"right_wheel": 40.0, "left_wheel": 40.0},
            damping={"right_wheel": 4.0, "left_wheel": 4.0},
        ),
        "torso": ImplicitActuatorCfg(
            joint_names_expr=["torso_.*"],
            effort_limit=100.0,
            velocity_limit=10.0,
            stiffness={"torso_.*": 50.0},
            damping={"torso_.*": 5.0},
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[".*_arm_.*"],
            effort_limit=100.0,
            velocity_limit=15.0,
            stiffness={"right_arm_.*": 50.0, "left_arm_.*": 50.0},
            damping={"right_arm_.*": 5.0, "left_arm_.*": 5.0},
        ),
        "grippers": ImplicitActuatorCfg(
            joint_names_expr=["gripper_finger_.*"],
            effort_limit=30.0,
            velocity_limit=5.0,
            stiffness={"gripper_finger_.*": 10.0},
            damping={"gripper_finger_.*": 1.0},
        ),
        "head": ImplicitActuatorCfg(
            joint_names_expr=["head_.*"],
            effort_limit=10.0,
            velocity_limit=5.0,
            stiffness={"head_.*": 5.0},
            damping={"head_.*": 1.0},
        ),
    },
)
