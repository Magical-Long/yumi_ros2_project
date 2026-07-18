import os
import sys


sys.path.append(os.path.dirname(__file__))

from sim_bringup_common import generate_sim_bringup_description


def generate_launch_description():
    return generate_sim_bringup_description(
        urdf_filename="yumi_gazebo_ros2.urdf",
        controllers_yaml_filename="ros2_controllers_traj.yaml",
        left_arm_controller_name="left_arm_controller",
        right_arm_controller_name="right_arm_controller",
        auto_home_supported=True,
    )
