from pyri.plugins.sandbox_functions import PyriSandboxFunctionsPluginFactory
from pyri.sandbox_context import PyriSandboxContext
import numpy as np

def robot_jog_freespace(robot_name, joint_position):
    print(f"Jogging robot \"{robot_name}\" to: {joint_position}")

    device_manager = PyriSandboxContext.device_manager
    robot_client = device_manager.get_device_client(robot_name, 1)
    j = np.array(joint_position, dtype=np.float64)
    v = np.ones((len(j)))
    robot_client.jog_freespace(j,v,True)


class RoboticsSandboxFunctionsPluginFactory(PyriSandboxFunctionsPluginFactory):
    def get_plugin_name(self):
        return "pyri-robotics"

    def get_sandbox_function_names(self):
        return ["robot_jog_freespace"]

    def get_sandbox_functions(self):
        return {"robot_jog_freespace": robot_jog_freespace}


def get_sandbox_functions_factory():
    return RoboticsSandboxFunctionsPluginFactory()