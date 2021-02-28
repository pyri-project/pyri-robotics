from pyri.plugins.device_type_adapter import PyriDeviceTypeAdapterExtendedState, \
    PyriDeviceTypeAdapter, PyriDeviceTypeAdapterPluginFactory
from typing import List, Dict, Any, NamedTuple
import RobotRaconteur as RR

class Robot_TypeAdapter(PyriDeviceTypeAdapter):
    """Adapter for com.robotraconteur.robotics.robot.Robot"""

    def __init__(self, client_subscription, node):
        self._sub: "RobotRaconteur.ServiceSubscription" = client_subscription
        self._state_sub = self._sub.SubscribeWire("robot_state")
        self._state_sub.InValueLifespan = 0.5
        self._node = node
        self._robot_consts = None

    async def get_extended_device_infos(self, timeout) -> Dict[str,RR.VarValue]:

        res, c = self._sub.TryGetDefaultClientWait(timeout)
        if not res:
            return dict()
        
        info = await c.async_get_robot_info(None,timeout)
        return {"com.robotraconteur.robotics.robot.RobotInfo": RR.VarValue(info,"com.robotraconteur.robotics.robot.RobotInfo")}

    async def get_extended_device_states(self, timeout) -> Dict[str,PyriDeviceTypeAdapterExtendedState]:
        res, robot_state, _ = self._state_sub.TryGetInValue()
        if not res:
            return dict()

        if self._robot_consts is None:
            res, default_client = self._sub.TryGetDefaultClient()
            if res:
                self._robot_consts = self._node.GetConstants("com.robotraconteur.robotics.robot",default_client)

        display_flags = []
        ready = False
        error = True
        

        if self._robot_consts is not None:
            state_flags_enum = self._robot_consts['RobotStateFlags']
            for flag_name, flag_code in state_flags_enum.items():
                if flag_code & robot_state.robot_state_flags != 0:
                    display_flags.append(flag_name)
            
            ready = robot_state.robot_state_flags & self._robot_consts['RobotStateFlags']['ready'] != 0
            error = robot_state.robot_state_flags & (self._robot_consts['RobotStateFlags']['error'] | self._robot_consts['RobotStateFlags']['communication_failure'])
        else:
            display_flags ["pyri_internal_error"]

        p_value = PyriDeviceTypeAdapterExtendedState(
            "com.robotraconteur.robotics.robot.RobotInfo",
            display_flags,
            RR.VarValue(robot_state, 'com.robotraconteur.robotics.robot.RobotState'), 
            ready, 
            error, 
            robot_state.seqno
        )

        return {"com.robotraconteur.robotics.robot.RobotInfo": p_value}

class PyriRoboticsTypeAdapterPluginFactory(PyriDeviceTypeAdapterPluginFactory):
    
    def get_plugin_name(self):
        return "pyri-robotics"

    def get_robotraconteur_types(self) -> List[str]:
        return ["com.robotraconteur.robotics.robot.Robot"]

    def create_device_type_adapter(self, robotraconteur_type: str, client_subscription: Any, node) -> PyriDeviceTypeAdapter:

        if robotraconteur_type == "com.robotraconteur.robotics.robot.Robot":
            return Robot_TypeAdapter(client_subscription,node)
        assert False, "Invalid robotraconteur_type device type adapter requested"

def get_device_type_adapter_factory():
    return PyriRoboticsTypeAdapterPluginFactory()