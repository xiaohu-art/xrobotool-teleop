import os

import numpy as np
import tyro

from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
    MujocoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


def main(
    xml_path: str = os.path.join(ASSET_PATH, "fr3v2/scene.xml"),
    robot_urdf_path: str = os.path.join(ASSET_PATH, "fr3v2/fr3v2.urdf"),
    scale_factor: float = 1.5,
    visualize_placo: bool = True,
):
    """
    Single-arm Franka Research 3 v2 teleoperation in MuJoCo (with Franka Hand gripper).
    """
    config = {
        "right_hand": {
            "link_name": "fr3v2_hand_tcp",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "vis_target": "right_target",
            "gripper_config": {
                "type": "parallel",
                "gripper_trigger": "right_trigger",
                "joint_names": ["fr3v2_finger_joint1"],
                "open_pos": [0.04],
                "close_pos": [0.0],
            },
        },
    }

    mj_qpos_init = np.array([0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853, 0.04, 0.04])

    controller = MujocoTeleopController(
        xml_path=xml_path,
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
        mj_qpos_init=mj_qpos_init,
    )

    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({joint: 0.0 for joint in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)

    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
