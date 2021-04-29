from pyri.plugins.sandbox_functions import PyriSandboxFunctionsPluginFactory
from pyri.sandbox_context import PyriSandboxContext
import numpy as np
import time

def robot_jog_joint(dropdown_joint_selected ,value_degree, speed_perc):

    device_manager = PyriSandboxContext.device_manager
    jog_service = device_manager.get_device_client("robotics_jog", 1)
    jog = jog_service.get_jog("robot")
    jog.jog_joint_to_angle(dropdown_joint_selected-1, np.deg2rad(float(value_degree)), float(speed_perc))

def robot_jog_joints(joint_pos_degree, speed_perc):

    device_manager = PyriSandboxContext.device_manager
    jog_service = device_manager.get_device_client("robotics_jog", 1)
    jog = jog_service.get_jog("robot")
    jog.jog_joints_to_angles2(np.deg2rad(joint_pos_degree), float(speed_perc))

def robot_tool_gripper(dropdown_status):

    device_manager = PyriSandboxContext.device_manager
    jog_service = device_manager.get_device_client("robotics_jog", 1)
    tool = jog_service.get_tool("tool")
    if bool(int(dropdown_status)):
        tool.open()
    else:
        tool.close()

def robot_planar_grab(object_pose, grab_reference_pose, z_offset_before, z_offset_grab, speed_perc, wait):
    # TODO: Better detection of named pose
    if hasattr(object_pose, "pose"):
        object_pose = object_pose.pose
        if hasattr(object_pose, "pose"):
            object_pose = object_pose.pose
    
    # TODO: Handle transform type as well?

    is_pose2d = False
    if "z" not in object_pose[0]["position"].dtype.names:
        is_pose2d = True

    device_manager = PyriSandboxContext.device_manager
    motion_service = device_manager.get_device_client("robotics_motion", 1)
    if is_pose2d:
        gen = motion_service.grab_object_planar("robot", "tool", "robot_origin_calibration0", grab_reference_pose, object_pose,
             z_offset_before, z_offset_grab, speed_perc)
    else:
        gen = motion_service.grab_object_planar2("robot", "tool", "robot_origin_calibration0", grab_reference_pose, object_pose,
             z_offset_before, z_offset_grab, speed_perc)
    PyriSandboxContext.action_runner.run_action("robot",gen,wait)


def _get_sandbox_functions():
    return {
        "robot_jog_joint": robot_jog_joint,
        "robot_jog_joints": robot_jog_joints,
        "robot_tool_gripper": robot_tool_gripper,
        "robot_planar_grab": robot_planar_grab
    }

class RoboticsSandboxFunctionsPluginFactory(PyriSandboxFunctionsPluginFactory):
    def get_plugin_name(self):
        return "pyri-robotics"

    def get_sandbox_function_names(self):
        return list(_get_sandbox_functions().keys())

    def get_sandbox_functions(self):
        return _get_sandbox_functions()


def get_sandbox_functions_factory():
    return RoboticsSandboxFunctionsPluginFactory()