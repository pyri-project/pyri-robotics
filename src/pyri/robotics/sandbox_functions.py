from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil
from RobotRaconteurCompanion.Util.RobotUtil import RobotUtil
from pyri.plugins.sandbox_functions import PyriSandboxFunctionsPluginFactory
from pyri.sandbox_context import PyriSandboxContext
import numpy as np
import time
import general_robotics_toolbox as rox

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

def robot_jog_pose(target_pose, speed_perc, frame):
    device_manager = PyriSandboxContext.device_manager
    jog_service = device_manager.get_device_client("robotics_jog", 1)
    jog = jog_service.get_jog("robot")
    geom_util = GeometryUtil(client_obj = jog_service)
    
    if frame =="ROBOT":
        pass
    elif frame == "WORLD":
        var_storage = device_manager.get_device_client("variable_storage")
        # TODO: don't hard code robot origin
        robot_origin_pose = var_storage.getf_variable_value("globals","robot_origin_calibration0").data
        T_rob = geom_util.named_pose_to_rox_transform(robot_origin_pose.pose)
        T_des1 = geom_util.pose_to_rox_transform(target_pose)
        T_des = T_rob.inv() * T_des1
        target_pose = geom_util.rox_transform_to_pose(T_des)
    else:
        assert False, "Invalid frame"

    # TODO: Tool frame

    jog.jog_joints_to_pose(target_pose, float(speed_perc))

def robot_get_joint_position():
    device_manager = PyriSandboxContext.device_manager
    robot = device_manager.get_device_client("robot",1)
    robot_state, _ = robot.robot_state.PeekInValue()

    # TODO: don't convert to deg for prismatic joints
    return np.rad2deg(robot_state.joint_position).tolist()

def robot_get_end_pose(frame):
    device_manager = PyriSandboxContext.device_manager
    robot = device_manager.get_device_client("robot",1)
    robot_state, _ = robot.robot_state.PeekInValue()

    robot_util = RobotUtil(client_obj = robot)
    geom_util = GeometryUtil(client_obj = robot)

    # TODO: cache robot_info
    rox_robot = robot_util.robot_info_to_rox_robot(robot.robot_info,0)
    T1 = rox.fwdkin(rox_robot,robot_state.joint_position)

    if frame =="ROBOT":
        return geom_util.rox_transform_to_pose(T1)
    elif frame == "WORLD":
        var_storage = device_manager.get_device_client("variable_storage")
        # TODO: don't hard code robot origin
        robot_origin_pose = var_storage.getf_variable_value("globals","robot_origin_calibration0").data
        T_rob = geom_util.named_pose_to_rox_transform(robot_origin_pose.pose)
        T2 = T_rob * T1
        return geom_util.rox_transform_to_pose(T2)
    else:
        assert False, "Invalid frame"


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
        "robot_jog_pose": robot_jog_pose,
        "robot_tool_gripper": robot_tool_gripper,
        "robot_planar_grab": robot_planar_grab,
        "robot_get_joint_position": robot_get_joint_position,
        "robot_get_end_pose": robot_get_end_pose
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