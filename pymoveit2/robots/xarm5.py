from typing import List

MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "xarm_gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.1, 0.1]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.5, 0.5]


def joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "link1",
        prefix + "link2",
        prefix + "link3",
        prefix + "link4",
        prefix + "link5",
    ]


def base_link_name(prefix: str = "") -> str:
    return prefix + "link_base"


def end_effector_name(prefix: str = "") -> str:
    return prefix + "link_tcp"


def gripper_joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "right_finger_joint",
        prefix + "left_finger_joint",
    ]
