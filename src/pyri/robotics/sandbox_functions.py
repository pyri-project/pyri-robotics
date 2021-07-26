from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil
from RobotRaconteurCompanion.Util.RobotUtil import RobotUtil
from pyri.plugins.sandbox_functions import PyriSandboxFunctionsPluginFactory
from pyri.sandbox_context import PyriSandboxContext
import numpy as np
import time
import general_robotics_toolbox as rox

def robot_set_active_robot(robot_name):
    PyriSandboxContext.context_vars["active_robot"] = robot_name

def _get_active_robot_name():
    # TODO: verify robot exists
    if "active_robot" in PyriSandboxContext.context_vars:
        return PyriSandboxContext.context_vars["active_robot"]
    else:
        return "robot"

def robot_set_origin_calibration(calibration_name):
    # TODO: verify robot and calibration_name exist
    if "robot_origin_calibration" not in PyriSandboxContext.context_vars:
        PyriSandboxContext.context_vars["robot_origin_calibration"] = dict()
    PyriSandboxContext.context_vars["robot_origin_calibration"][_get_active_robot_name()] = calibration_name

def _get_robot_origin_calibration():
    robot_name = _get_active_robot_name()

    if "robot_origin_calibration" not in PyriSandboxContext.context_vars:
        return "robot_origin_calibration"
    if robot_name not in PyriSandboxContext.context_vars["robot_origin_calibration"]:
        return f"{robot_name}_origin_calibration"
    return PyriSandboxContext.context_vars["robot_origin_calibration"][robot_name]

def robot_set_active_tool(tool_name):
    PyriSandboxContext.context_vars["active_tool"] = tool_name

def _get_active_tool_name():
    # TODO: verify robot exists
    if "active_tool" in PyriSandboxContext.context_vars:
        return PyriSandboxContext.context_vars["active_tool"]
    else:
        return "tool"

def robot_movej(joint_pos_degree, speed_perc, wait):

    robot_name = _get_active_robot_name()

    device_manager = PyriSandboxContext.device_manager
    motion_service = device_manager.get_device_client("robotics_motion", 1)
    move_gen = motion_service.movej(robot_name, joint_pos_degree, float(speed_perc))

    PyriSandboxContext.action_runner.run_action(robot_name,move_gen,wait)

def robot_movel(target_pose, speed_perc, frame, wait):

    while hasattr(target_pose,"pose"):
        target_pose = target_pose.pose

    robot_name = _get_active_robot_name()
    robot_origin_calib = _get_robot_origin_calibration()

    device_manager = PyriSandboxContext.device_manager
    motion_service = device_manager.get_device_client("robotics_motion", 1)
    move_gen = motion_service.movel(robot_name, target_pose, frame, robot_origin_calib, float(speed_perc))
    
    # TODO: Tool frame

    PyriSandboxContext.action_runner.run_action(robot_name,move_gen,wait)

def robot_move_joint_trajectory(joint_trajectory, speed_perc, wait):

    robot_name = _get_active_robot_name()

    device_manager = PyriSandboxContext.device_manager
    motion_service = device_manager.get_device_client("robotics_motion", 1)
    move_gen = motion_service.move_joint_trajectory(robot_name, joint_trajectory, float(speed_perc))
    
    PyriSandboxContext.action_runner.run_action(robot_name,move_gen,wait)

def robot_get_joint_position():
    robot_name = _get_active_robot_name()

    device_manager = PyriSandboxContext.device_manager
    robot = device_manager.get_device_client(robot_name,1)
    robot_state, _ = robot.robot_state.PeekInValue()

    # TODO: don't convert to deg for prismatic joints
    return np.rad2deg(robot_state.joint_position).tolist()

def robot_get_end_pose(frame):
    robot_name = _get_active_robot_name()

    device_manager = PyriSandboxContext.device_manager
    robot = device_manager.get_device_client(robot_name,1)
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
        robot_origin_pose = var_storage.getf_variable_value("globals",_get_robot_origin_calibration()).data
        T_rob = geom_util.named_pose_to_rox_transform(robot_origin_pose.pose)
        T2 = T_rob * T1
        return geom_util.rox_transform_to_pose(T2)
    else:
        assert False, "Invalid frame"


def robot_tool_gripper(dropdown_status):

    tool_name = _get_active_tool_name()

    device_manager = PyriSandboxContext.device_manager
    jog_service = device_manager.get_device_client("robotics_jog", 1)
    tool = jog_service.get_tool(tool_name)
    if bool(int(dropdown_status)):
        tool.open()
    else:
        tool.close()

def robot_planar_grab(object_pose, grab_reference_pose, z_offset_before, z_offset_grab, speed_perc, wait):

    robot_name = _get_active_robot_name()
    tool_name = _get_active_tool_name()
    calib_name = _get_robot_origin_calibration()

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
        gen = motion_service.grab_object_planar(robot_name, tool_name, calib_name, grab_reference_pose, object_pose,
             z_offset_before, z_offset_grab, speed_perc)
    else:
        gen = motion_service.grab_object_planar2(robot_name, tool_name, calib_name, grab_reference_pose, object_pose,
             z_offset_before, z_offset_grab, speed_perc)
    PyriSandboxContext.action_runner.run_action(robot_name,gen,wait)

def robot_planar_place(target_pose, place_reference_pose, z_offset_before, z_offset_place, speed_perc, wait):

    robot_name = _get_active_robot_name()
    tool_name = _get_active_tool_name()
    calib_name = _get_robot_origin_calibration()

    # TODO: Better detection of named pose
    if hasattr(target_pose, "pose"):
        target_pose = target_pose.pose
        if hasattr(target_pose, "pose"):
            target_pose = target_pose.pose
    
    # TODO: Handle transform type as well?

    is_pose2d = False
    if "z" not in target_pose[0]["position"].dtype.names:
        is_pose2d = True

    device_manager = PyriSandboxContext.device_manager
    motion_service = device_manager.get_device_client("robotics_motion", 1)
    if is_pose2d:
        gen = motion_service.place_object_planar(robot_name, tool_name, calib_name, place_reference_pose, target_pose,
             z_offset_before, z_offset_place, speed_perc)
    else:
        gen = motion_service.place_object_planar2(robot_name, tool_name, calib_name, place_reference_pose, target_pose,
             z_offset_before, z_offset_place, speed_perc)
    PyriSandboxContext.action_runner.run_action(robot_name,gen,wait)


def _get_sandbox_functions():
    return {       
        "robot_movej": robot_movej,
        "robot_movel": robot_movel,
        "robot_move_joint_trajectory": robot_move_joint_trajectory,
        "robot_tool_gripper": robot_tool_gripper,
        "robot_planar_grab": robot_planar_grab,
        "robot_planar_place": robot_planar_place,
        "robot_get_joint_position": robot_get_joint_position,
        "robot_get_end_pose": robot_get_end_pose,
        "robot_set_active_robot": robot_set_active_robot,
        "robot_set_origin_calibration": robot_set_origin_calibration,
        "robot_set_active_tool": robot_set_active_tool
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