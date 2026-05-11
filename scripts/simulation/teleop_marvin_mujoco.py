import os

import tyro

from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
    MujocoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


def main(
    xml_path: str = os.path.join(ASSET_PATH, "marvin/scene_dual_arm.xml"),
    robot_urdf_path: str = os.path.join(ASSET_PATH, "marvin/marvin.urdf"),
    scale_factor: float = 1.5,
    visualize_placo: bool = True,
):
    """
    Dual-arm Marvin teleoperation in MuJoCo.
    """
    config = {
        "right_hand": {
            "link_name": "Link7_R",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "vis_target": "right_target",
        },
        "left_hand": {
            "link_name": "Link7_L",
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "vis_target": "left_target",
        },
    }

    controller = MujocoTeleopController(
        xml_path=xml_path,
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
    )

    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({joint: 0.0 for joint in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)

    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
