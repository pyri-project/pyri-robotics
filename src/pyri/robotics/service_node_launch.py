from pyri.plugins.service_node_launch import ServiceNodeLaunch, PyriServiceNodeLaunchFactory


launches = [
    ServiceNodeLaunch("robotics_jog", "pyri.robotics", "pyri.robotics.robotics_jog_service", default_devices=[("pyri_robotics_jog","robotics_jog")]),
    ServiceNodeLaunch("robotics_motion", "pyri.robotics", "pyri.robotics.robotics_motion_service", default_devices=[("pyri_robotics_motion","robotics_motion")]),
]

class RoboticsLaunchFactory(PyriServiceNodeLaunchFactory):
    def get_plugin_name(self):
        return "pyri.robotics"

    def get_service_node_launch_names(self):
        return ["robotics_jog","robotics_motion"]

    def get_service_node_launches(self):
        return launches

def get_service_node_launch_factory():
    return RoboticsLaunchFactory()

        
