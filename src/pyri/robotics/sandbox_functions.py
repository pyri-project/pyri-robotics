from pyri.plugins.sandbox_functions import PyriSandboxFunctionsPluginFactory
from pyri.sandbox_context import PyriSandboxContext
import numpy as np
import time

def robot_jog_joint(dropdown_joint_selected ,value_degree, speed_perc):

    device_manager = PyriSandboxContext.device_manager
    jog_service = device_manager.get_device_client("joint_jog", 1)
    jog = jog_service.get_jog("robot")
    jog.jog_joint_to_angle(dropdown_joint_selected-1, np.deg2rad(float(value_degree)), float(speed_perc))

def time_sleep(seconds):

    time.sleep(seconds)

def robot_tool_gripper(dropdown_status):

    device_manager = PyriSandboxContext.device_manager
    jog_service = device_manager.get_device_client("joint_jog", 1)
    tool = jog_service.get_tool("tool")
    if bool(int(dropdown_status)):
        tool.open()
    else:
        tool.close()

def _get_sandbox_functions():
    return {
        "robot_jog_joint": robot_jog_joint,
        "robot_tool_gripper": robot_tool_gripper, 
        "time_sleep": time_sleep
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