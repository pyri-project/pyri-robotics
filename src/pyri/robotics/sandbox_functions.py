from pyri.plugins.sandbox_functions import PyriSandboxFunctionsPluginFactory
from pyri.sandbox_context import PyriSandboxContext
import numpy as np
import time

def robot_jog_joint(dropdown_joint_selected ,value_degree, speed_perc):

    device_manager = PyriSandboxContext.device_manager
    jog_service = device_manager.get_device_client("jog_joint", 1)
    jog = jog_service.get_jog("robot")
    jog.jog_joint_to_angle(dropdown_joint_selected-1, np.deg2rad(float(value_degree)), float(speed_perc))

def robot_jog_joints(joint_pos_degree, speed_perc):

    device_manager = PyriSandboxContext.device_manager
    jog_service = device_manager.get_device_client("jog_joint", 1)
    jog = jog_service.get_jog("robot")
    jog.jog_joints_to_angles2(np.deg2rad(joint_pos_degree), float(speed_perc))

def robot_tool_gripper(dropdown_status):

    device_manager = PyriSandboxContext.device_manager
    jog_service = device_manager.get_device_client("jog_joint", 1)
    tool = jog_service.get_tool("tool")
    if bool(int(dropdown_status)):
        tool.open()
    else:
        tool.close()

def math_vector(string_vector):
    return np.fromstring(string_vector,sep=",")

def _get_sandbox_functions():
    return {
        "robot_jog_joint": robot_jog_joint,
        "robot_jog_joints": robot_jog_joints,
        "robot_tool_gripper": robot_tool_gripper
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