from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil
from RobotRaconteurCompanion.Util.RobotUtil import RobotUtil
from pyri.plugins.sandbox_functions import PyriSandboxFunctionsPluginFactory
from pyri.sandbox_context import PyriSandboxContext
import numpy as np
import time
import general_robotics_toolbox as rox

def robot_set_active_robot(robot_name):
    """
    Set the active robot. All robot functions will act on this robot.
    This has not effect on robots that are executing an asynchronous
    operation. The default robot device name is `robot`.

    Parameters:

    * robot_name (str): The robot name
    """

    PyriSandboxContext.context_vars["active_robot"] = robot_name

def _get_active_robot_name():
    # TODO: verify robot exists
    if "active_robot" in PyriSandboxContext.context_vars:
        return PyriSandboxContext.context_vars["active_robot"]
    else:
        return "robot"

def robot_set_origin_calibration(calibration_name):
    """
    Sets the active robot calibration. Robot calibration is normally a NamedPose or
    NamedPoseWithCovariance variable saved in the global variable table. This calibration
    can be created using the vision package calibration utilities, or by creating
    a new NamedPose in the global variable table manually. The default
    calibration name is `robot_origin_calibration`.

    Parameters:

    * calibration_name (str): The name of the global variable containing the calibration pose

    """

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
    """
    Set the active tool. All tool related functions will act on this tool.
    This has not effect on tools that are executing an asynchronous
    operation. The default tool device name is `tool`.

    Parameters:

    * tool_name (str): The tool name
    """
    PyriSandboxContext.context_vars["active_tool"] = tool_name

def _get_active_tool_name():
    # TODO: verify robot exists
    if "active_tool" in PyriSandboxContext.context_vars:
        return PyriSandboxContext.context_vars["active_tool"]
    else:
        return "tool"

def robot_movej(joint_pos_degree, speed_perc, wait):
    """
    Move a robot in joint space with the specified speed percentage. A
    trapezoidal trajectory will be used for the move. If wait is `True`,
    the function will block until the robot has stopped. If it is
    `False`, the function will return immediately. Use
    `time_wait_for_completion()` or `time_wait_for_completion_all()`
    to synchronize completion of the operation.

    Parameters:

    * joint_pos_degree (array): The desired joint position in degrees
    * speed_perc (float): The move speed percentage, must be between 1 and 100
    * wait (bool): True to block until completion, False to return immediately
    """

    robot_name = _get_active_robot_name()

    device_manager = PyriSandboxContext.device_manager
    motion_service = device_manager.get_device_client("robotics_motion", 1)
    move_gen = motion_service.movej(robot_name, joint_pos_degree, float(speed_perc))

    PyriSandboxContext.action_runner.run_action(robot_name,move_gen,wait)

def robot_movel(target_pose, speed_perc, frame, wait):
    """
    Move a robot in a strait line in cartesian space with the specified 
    speed percentage. A
    trapezoidal trajectory will be used for the move. If wait is `True`,
    the function will block until the robot has stopped. If it is
    `False`, the function will return immediately. Use
    `time_wait_for_completion()` or `time_wait_for_completion_all()`
    to synchronize completion of the operation.

    Parameters:

    * target_pose (Pose): The desired robot flange pose in the specified frame
    * speed_perc (float): The move speed percentage, must be between 1 and 100
    * frame (str): The frame to use for `target_pose`. Must be `robot` or `world`.
    * wait (bool): True to block until completion, False to return immediately
    """

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
    """
    Execute a previously planned trajectory. This is typically returned
    by a planner routine, or other advanced function.

    Parameters:

    * joint_pos_degree (array): The desired joint position in degrees
    * speed_perc (float): The move speed percentage, must be between 1 and 100
    * wait (bool): True to block until completion, False to return immediately

    """

    robot_name = _get_active_robot_name()

    device_manager = PyriSandboxContext.device_manager
    motion_service = device_manager.get_device_client("robotics_motion", 1)
    move_gen = motion_service.move_joint_trajectory(robot_name, joint_trajectory, float(speed_perc))
    
    PyriSandboxContext.action_runner.run_action(robot_name,move_gen,wait)

def robot_get_joint_position():
    """
    Returns the joint position of the active robot, in degrees.

    Return (array): The joint position in degrees
    """
    robot_name = _get_active_robot_name()

    device_manager = PyriSandboxContext.device_manager
    robot = device_manager.get_device_client(robot_name,1)
    robot_state, _ = robot.robot_state.PeekInValue()

    # TODO: don't convert to deg for prismatic joints
    return np.rad2deg(robot_state.joint_position).tolist()

def robot_get_end_pose(frame):
    """
    Returns the end pose of a robot. This end pose is reported by the
    robot driver. It is typically defined as the flange of the robot,
    but may be the tool if the driver is configured to report
    the tool pose.

    Parameters:

    * frame (str): The frame to return the pose in. May be `robot` or `world`.

    Return (Pose): The robot end pose in the requested frame
    """
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


def robot_tool_gripper(state):
    """
    Opens or closes the tool.

    Parameters:

    state (int): 1 to open, 0 to close
    """

    tool_name = _get_active_tool_name()

    device_manager = PyriSandboxContext.device_manager
    jog_service = device_manager.get_device_client("robotics_jog", 1)
    tool = jog_service.get_tool(tool_name)
    if bool(int(state)):
        tool.open()
    else:
        tool.close()

def robot_planar_grab(object_pose, grab_reference_pose, z_offset_before, z_offset_grab, speed_perc, wait):
    """
    Execute a planar grab operation. The planar grab operation is useful to grab an object
    off a flat surface, such as a table. It is assumed that the world origin is configured
    to be on the surface of the table, and that the XY plane is flat on the surface. A 
    "reference pose" of joint angles is trained for the grab. This function will translate
    the this reference pose to the specified X, Y, and Yaw position of the object as specified
    in `object_pose` (all other components are ignored).

    This function will move the tool above the object to `z_offset_before`, lower the tool
    to `z_offset_grab`, close the tool, and raise the gripper to `z_offset_grab`. `z_offset_before`
    and `z_offset_grab` are in meters, relative to the reference pose.

    If wait is `True`,
    the function will block until the robot has stopped. If it is
    `False`, the function will return immediately. Use
    `time_wait_for_completion()` or `time_wait_for_completion_all()`
    to synchronize completion of the operation.

    Parameters:

    * object_pose (Pose): The pose of the object to grab. This pose must be flat on the table
    * grab_reference_pose (array): A reference pose joint array, in degrees
    * z_offset_before (float): Z offset before grab, in meters, relative to reference pose
    * z_offset_grab (float): Z offset of grab, in meters, relative to reference pose
    * speed_perc (float): The move speed percentage, must be between 1 and 100
    * wait (bool): True to block until completion, False to return immediately
    """

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
    """
    Execute a planar place operation. The planar place operation is used to place an object
    on a flat surface, such as a table. It is assumed that the world origin is configured
    to be on the surface of the table, and that the XY plane is flat on the surface. A 
    "place reference pose" of joint angles is trained for the place. This function will translate
    the this reference pose to the specified X, Y, and Yaw position of the object as specified
    in `target_pose` (all other components are ignored).

    This function will move the tool above the target pose to `z_offset_before`, lower the tool
    to `z_offset_place`, open the tool, and raise the gripper to `z_offset_place`. `z_offset_before`
    and `z_offset_place` are in meters, relative to the reference pose.

    If wait is `True`,
    the function will block until the robot has stopped. If it is
    `False`, the function will return immediately. Use
    `time_wait_for_completion()` or `time_wait_for_completion_all()`
    to synchronize completion of the operation.

    This function is normally used to place on object grabbed using `robot_planar_grab()`.

    Parameters:

    * target_pose (Pose): The pose of the target object position. This pose must be flat on the table
    * place_reference_pose (array): A reference pose joint array, in degrees
    * z_offset_before (float): Z offset before place, in meters, relative to reference pose
    * z_offset_place (float): Z offset of place, in meters, relative to reference pose
    * speed_perc (float): The move speed percentage, must be between 1 and 100
    * wait (bool): True to block until completion, False to return immediately
    """

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