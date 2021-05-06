import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np
import argparse
import RobotRaconteurCompanion as RRC
from pyri.device_manager_client import DeviceManagerClient
import importlib.resources as resources
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
from RobotRaconteurCompanion.Util.RobotUtil import RobotUtil
from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil

import general_robotics_toolbox as rox
import copy
from ..util import invkin
import time
import threading

import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo

from pyri.util.robotraconteur import add_default_ws_origins
import math


class RoboticsMotion_impl(object):
    def __init__(self, device_manager_url, device_info = None, node: RR.RobotRaconteurNode = None):
        if node is None:
            self._node = RR.RobotRaconteurNode.s
        else:
            self._node = node
        self.device_info = device_info

        self.service_path = None
        self.ctx = None

        self._pose2d_dtype = self._node.GetNamedArrayDType("com.robotraconteur.geometry.Pose2D")
        self._point2d_dtype = self._node.GetNamedArrayDType("com.robotraconteur.geometry.Point2D")

        self._geom_util = GeometryUtil(node=self._node)
        self._robot_util = RobotUtil(node=self._node)

        self.device_manager = DeviceManagerClient(device_manager_url)
        self.device_manager.device_added += self._device_added
        self.device_manager.device_removed += self._device_removed
        self.device_manager.refresh_devices(5)

    def RRServiceObjectInit(self, ctx, service_path):
        self.service_path = service_path
        self.ctx = ctx

    def _device_added(self, local_device_name):
       pass

    def _device_removed(self, local_device_name):
        pass

    def _generate_movej_trajectory(self, robot_client, rox_robot, q_initial, q_final, speed_perc):

        JointTrajectoryWaypoint = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint",robot_client)
        JointTrajectory = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectory",robot_client)

        dof = len(rox_robot.joint_type)

        N_samples = math.ceil(np.max(np.abs(q_final-q_initial))*0.2*180/np.pi)
        N_samples = np.max((N_samples,20))

        ss = np.linspace(0,1,N_samples)

        ss = np.linspace(0,1,N_samples)

        way_pts = np.zeros((N_samples, dof), dtype=np.float64)
        for i in range(dof):
            way_pts[:,i] = np.linspace(q_initial[i], q_final[i], N_samples)

        vlims = rox_robot.joint_vel_limit
        alims = rox_robot.joint_acc_limit

        path = ta.SplineInterpolator(ss, way_pts)
        pc_vel = constraint.JointVelocityConstraint(vlims*0.95*(speed_perc/100.0))
        pc_acc = constraint.JointAccelerationConstraint(alims)

        instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
        jnt_traj = instance.compute_trajectory()

        if jnt_traj is None:
            return None

        ts_sample = np.linspace(0,jnt_traj.duration,N_samples)
        qs_sample = jnt_traj(ts_sample)

        waypoints = []

        for i in range(N_samples):
            wp = JointTrajectoryWaypoint()
            wp.joint_position = qs_sample[i,:]
            wp.time_from_start = ts_sample[i]
            waypoints.append(wp)

        traj = JointTrajectory()
        traj.joint_names = rox_robot.joint_names
        traj.waypoints = waypoints

        return traj

    def movej(self, robot_local_device_name, q_final, speed_perc):
        
        # Convert to radians
        q_final = np.deg2rad(q_final)

        robot = self.device_manager.get_device_client(robot_local_device_name)
        robot_info = robot.robot_info
        rox_robot = self._robot_util.robot_info_to_rox_robot(robot_info,0)

        robot_state = robot.robot_state.PeekInValue()[0]

        q_initial = robot_state.joint_position

        traj = self._generate_movej_trajectory(robot, rox_robot, q_initial, q_final, speed_perc)
        if traj is None:
            return EmptyGenerator()

        return TrajectoryMoveGenerator(robot, rox_robot, traj, self._node)

    def _generate_movel_trajectory(self, robot_client, rox_robot, initial_q_or_T, T_final, speed_perc):

        JointTrajectoryWaypoint = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint",robot_client)
        JointTrajectory = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectory",robot_client)

        dof = len(rox_robot.joint_type)

        if isinstance(initial_q_or_T,rox.Robot):
            T_initial = initial_q_or_T
            q_initial = invkin.update_ik_info3(rox_robot, initial_q_or_T, np.random.randn((dof,)))
        else:
            q_initial = initial_q_or_T
            T_initial = rox.fwdkin(rox_robot, q_initial)

        p_diff = T_final.p - T_initial.p
        R_diff = np.transpose(T_initial.R) @ T_final.R
        k,theta = rox.R2rot(R_diff)

        N_samples_translate = np.linalg.norm(p_diff)*100.0
        N_samples_rot = np.abs(theta)*0.25*180/np.pi

        N_samples_translate = np.linalg.norm(p_diff)*100.0
        N_samples_rot = np.abs(theta)*0.2*180/np.pi

        N_samples = math.ceil(np.max((N_samples_translate,N_samples_rot,20)))

        ss = np.linspace(0,1,N_samples)

        q_final, res = invkin.update_ik_info3(rox_robot, T_final, q_initial)
        assert res, "Inverse kinematics failed"

        way_pts = np.zeros((N_samples, dof), dtype=np.float64)
        way_pts[0,:] = q_initial
        for i in range(1,N_samples):
            s = float(i)/(N_samples-1)
            R_i = T_initial.R @ rox.rot(k,theta * s)
            p_i = (p_diff * s) + T_initial.p
            T_i = rox.Transform(R_i,p_i)
            q, res = invkin.update_ik_info3(rox_robot, T_i, way_pts[i-1,:])
            assert res, "Inverse kinematics failed"
            way_pts[i,:] = q

        vlims = rox_robot.joint_vel_limit
        alims = rox_robot.joint_acc_limit

        path = ta.SplineInterpolator(ss, way_pts)
        pc_vel = constraint.JointVelocityConstraint(vlims*0.95*speed_perc/100.0)
        pc_acc = constraint.JointAccelerationConstraint(alims)

        instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
        jnt_traj = instance.compute_trajectory()
        
        if jnt_traj is None:
            return None

        ts_sample = np.linspace(0,jnt_traj.duration,N_samples)
        qs_sample = jnt_traj(ts_sample)

        waypoints = []

        for i in range(N_samples):
            wp = JointTrajectoryWaypoint()
            wp.joint_position = qs_sample[i,:]
            wp.time_from_start = ts_sample[i]
            waypoints.append(wp)

        traj = JointTrajectory()
        traj.joint_names = rox_robot.joint_names
        traj.waypoints = waypoints

        return traj

    def movel(self, robot_local_device_name, pose_final, frame, robot_origin_calib_global_name, speed_perc):
        
        robot = self.device_manager.get_device_client(robot_local_device_name)
        geom_util = GeometryUtil(client_obj = robot)

        if frame.lower() == "world":
            var_storage = self.device_manager.get_device_client("variable_storage")            
            robot_origin_pose = var_storage.getf_variable_value("globals",robot_origin_calib_global_name).data
            T_rob = geom_util.named_pose_to_rox_transform(robot_origin_pose.pose)
            T_des1 = geom_util.pose_to_rox_transform(pose_final)
            T_des = T_rob.inv() * T_des1
            pose_final = geom_util.rox_transform_to_pose(T_des)
        elif frame.lower() == "robot":
            T_des = geom_util.pose_to_rox_transform(pose_final)
        else:
            assert False, "Unknown parent frame for movel"        

        robot_info = robot.robot_info
        rox_robot = self._robot_util.robot_info_to_rox_robot(robot_info,0)

        robot_state = robot.robot_state.PeekInValue()[0]

        q_initial = robot_state.joint_position

        traj = self._generate_movel_trajectory(robot, rox_robot, q_initial, T_des, speed_perc)

        if traj is None:
            return EmptyGenerator()

        return TrajectoryMoveGenerator(robot, rox_robot, traj, self._node)

    def _do_grab_place_object_planar(self, robot_local_device_name, tool_local_device_name, robot_origin_calib_global_name,
        reference_pose_global_name, object_x, object_y, object_theta, z_offset_before, z_offset_grab, speed_perc, grab):

        var_storage = self.device_manager.get_device_client("variable_storage",0.1)

        robot = self.device_manager.get_device_client(robot_local_device_name)
        tool = self.device_manager.get_device_client(tool_local_device_name)

        reference_pose = var_storage.getf_variable_value("globals", reference_pose_global_name)
        robot_origin_calib = var_storage.getf_variable_value("globals", robot_origin_calib_global_name)

        T_robot = self._geom_util.named_pose_to_rox_transform(robot_origin_calib.data.pose)


        robot_info = robot.robot_info
        rox_robot = self._robot_util.robot_info_to_rox_robot(robot_info,0)

        # The object's pose in world coordinates
        T_object = rox.Transform(rox.rot([0.,0.,1.],object_theta),[object_x,object_y,0.])

        # The robot's reference pose in its own frame
        T_reference_pose_r = rox.fwdkin(rox_robot, np.deg2rad(reference_pose.data))

        # The robot's reference pose in world coordinates
        T_reference_pose = T_robot * T_reference_pose_r

        # The nominal grab pose copy
        T_grab_pose_n = copy.deepcopy(T_reference_pose)

        # Translate the reference pose in x and y to the nominal grab point
        T_grab_pose_n.p[0] = T_object.p[0]
        T_grab_pose_n.p[1] = T_object.p[1]

        # Rotate the reference pose to align with the object
        T_grab_pose_n.R = T_object.R @ T_grab_pose_n.R

        # Before pose
        T_grab_pose_before = copy.deepcopy(T_grab_pose_n)
        T_grab_pose_before.p[2] += z_offset_before
        # Transform to robot frame
        T_grab_pose_before_r = T_robot.inv() * T_grab_pose_before

        # Grab pose
        T_grab_pose = copy.deepcopy(T_grab_pose_n)
        T_grab_pose.p[2] += z_offset_grab
        # Transform to robot frame
        T_grab_pose_r = T_robot.inv() * T_grab_pose

        robot_state, _ = robot.robot_state.PeekInValue()
        q_current = robot_state.joint_position

        ## Compute inverse kinematics
        # q_grab_before, res = invkin.update_ik_info3(rox_robot, T_grab_pose_before_r, q_current)
        # assert res, "Invalid setpoint: invkin did not converge"
        # q_grab, res = invkin.update_ik_info3(rox_robot, T_grab_pose_r, q_current)
        # assert res, "Invalid setpoint: invkin did not converge"

        # print(f"q_grab_before: {q_grab_before}")
        # print(f"q_grab: {q_grab}")
        # print()

        traj_before = self._generate_movel_trajectory(robot, rox_robot, q_current, T_grab_pose_before_r, speed_perc)

        q_init_grab = traj_before.waypoints[-1].joint_position
        traj_grab = self._generate_movel_trajectory(robot, rox_robot, q_init_grab, T_grab_pose_r, speed_perc)

        q_init_after = traj_grab.waypoints[-1].joint_position
        traj_after = self._generate_movel_trajectory(robot, rox_robot, q_init_after, T_grab_pose_before_r, speed_perc)
        

        gen = PickPlaceMotionGenerator(robot, rox_robot, tool, traj_before, traj_grab, traj_after,
            grab, self._node)

        # robot.jog_freespace(q_grab_before,max_velocity,True)
        # time.sleep(0.5)
        # robot.jog_freespace(q_grab,max_velocity*.25,True)
        # time.sleep(0.5)
        # robot.jog_freespace(q_grab_before,max_velocity*.25,True)
        return gen

    def grab_object_planar(self, robot_local_device_name, tool_local_device_name, robot_origin_calib_global_name,
        reference_pose_global_name, object_world_pose, z_offset_before, z_offset_grab, speed_perc):
        
        object_x = object_world_pose["position"]["x"]
        object_y = object_world_pose["position"]["y"]
        object_theta = object_world_pose["orientation"]
        
        return self._do_grab_place_object_planar(robot_local_device_name, tool_local_device_name, robot_origin_calib_global_name,
            reference_pose_global_name, object_x, object_y, object_theta, z_offset_before, z_offset_grab, speed_perc, True)

    def grab_object_planar2(self, robot_local_device_name, tool_local_device_name, robot_origin_calib_global_name,
        reference_pose_global_name, object_world_pose, z_offset_before, z_offset_grab, speed_perc):

        object_x = object_world_pose["position"]["x"]
        object_y = object_world_pose["position"]["y"]
        object_rpy = self._geom_util.quaternion_to_rpy(object_world_pose["orientation"])
        assert np.abs(object_rpy[0]) < np.deg2rad(15), "Object is not flat on surface!"
        assert np.abs(object_rpy[1]) < np.deg2rad(15), "Object is not flat on surface!"
        object_theta = object_rpy[2]

        return self._do_grab_place_object_planar(robot_local_device_name, tool_local_device_name, robot_origin_calib_global_name,
            reference_pose_global_name, object_x, object_y, object_theta, z_offset_before, z_offset_grab, speed_perc, True)

    def place_object_planar(self, robot_local_device_name, tool_local_device_name, robot_origin_calib_global_name,
        reference_pose_global_name, object_world_pose, z_offset_before, z_offset_grab, speed_perc):
        
        object_x = object_world_pose["position"]["x"]
        object_y = object_world_pose["position"]["y"]
        object_theta = object_world_pose["orientation"]
        
        return self._do_grab_place_object_planar(robot_local_device_name, tool_local_device_name, robot_origin_calib_global_name,
            reference_pose_global_name, object_x, object_y, object_theta, z_offset_before, z_offset_grab, speed_perc, False)

    def place_object_planar2(self, robot_local_device_name, tool_local_device_name, robot_origin_calib_global_name,
        reference_pose_global_name, object_world_pose, z_offset_before, z_offset_grab, speed_perc):

        object_x = object_world_pose["position"]["x"]
        object_y = object_world_pose["position"]["y"]
        object_rpy = self._geom_util.quaternion_to_rpy(object_world_pose["orientation"])
        assert np.abs(object_rpy[0]) < np.deg2rad(5), "Target location is not flat on surface!"
        assert np.abs(object_rpy[1]) < np.deg2rad(5), "Target location is not flat on surface!"
        object_theta = object_rpy[2]

        return self._do_grab_place_object_planar(robot_local_device_name, tool_local_device_name, robot_origin_calib_global_name,
            reference_pose_global_name, object_x, object_y, object_theta, z_offset_before, z_offset_grab, speed_perc, False)


class EmptyGenerator:
    def __init__(self):
        pass

    def Next(self):
        raise RR.StopIterationException("")

    def Close(self):
        pass

    def Abort(self):
        pass

class TrajectoryMoveGenerator:
    def __init__(self, robot, rox_robot, trajectory, node):
        self.node = node
        self.robot = robot
        self.rox_robot = rox_robot
        self.trajectory = trajectory
        self.gen = None
        self._aborted = False
        self._closed = False
        self._lock = threading.Lock()
        self._state = 0
        self._ret_type = self.node.GetStructureType("tech.pyri.robotics.motion.RobotMoveOpStatus")
        self._ret_codes = self.node.GetConstants("tech.pyri.robotics.motion")["RobotMoveOpStateCode"]
        self._command_modes = self.node.GetConstants("com.robotraconteur.robotics.robot")["RobotCommandMode"]

    def Next(self):
        with self._lock:

            if self._aborted:
                raise RR.OperationAbortedException("")

            if self._closed:
                raise RR.StopIterationException("")

            if self._state == self._ret_codes["unknown"]:
                self._state = self._ret_codes["move_planned"]
                ret = self._ret_type()
                ret.action_status = 2
                ret.planned_motion = self.trajectory
                ret.current_waypoint = 0
                ret.trajectory_time = 0.0
                ret.move_state = self._state
                return ret
            
            elif self._state == self._ret_codes["move_planned"]:
                
                robot_mode = self.robot.command_mode

                if robot_mode != self._command_modes["trajectory"]:
                    self.robot.command_mode = self._command_modes["halt"]
                    time.sleep(0.005)
                    self.robot.command_mode = self._command_modes["trajectory"]

                self.gen=self.robot.execute_trajectory(self.trajectory)

                self._state = self._ret_codes["moving"]

                ret = self._ret_type()
                ret.action_status = 2
                ret.planned_motion = self.trajectory
                ret.current_waypoint = 0
                ret.trajectory_time = 0.0
                ret.move_state = self._state
                return ret

            elif self._state == self._ret_codes["moving"]:

                traj_done = False
                gen_ret = None
                try:
                    gen_ret = self.gen.Next()
                except RR.StopIterationException:
                    traj_done = True

                ret = self._ret_type()
                ret.planned_motion = self.trajectory
                if gen_ret is not None:
                    ret.current_waypoint = gen_ret.current_waypoint
                    ret.trajectory_time = gen_ret.trajectory_time
                else:
                    ret.current_waypoint = 0
                    ret.trajectory_time = 0
                if not traj_done:
                    ret.action_status = 2
                    ret.move_state = self._ret_codes["moving"]
                else:
                    self._closed = True
                    ret.action_status = 3,
                    ret.move_state = self._ret_codes["move_complete"]
                    self._state = self._ret_codes["move_complete"]

                return ret

            assert False, "Invalid move generator state"

    def Close(self):
        with self._lock:
            self._closed = True
            if self.gen:
                self.gen.Close()

    def Abort(self):
        with self._lock:
            self._aborted = True
            if self.gen:
                self.gen.Abort()

class PickPlaceMotionGenerator:

    def __init__(self, robot, rox_robot, tool, traj_before, traj_grab, traj_after, grab, node):
        self.node = node
        self.robot = robot
        self.rox_robot = rox_robot
        self.tool = tool
        self.traj_before = traj_before
        self.traj_grab = traj_grab
        self.traj_after = traj_after
        self.grab = grab

        self._wait_next_cv = threading.Condition()
        self._aborted = False
        self._closed = False
        self._step = 0
        self._gen_next_done = False
        self._gen_next_err = None
        self._gen = None

        self._robot_motion_status = self.node.GetStructureType("tech.pyri.robotics.motion.RobotMotionPlanarOpStatus")
        self._action_consts = self.node.GetConstants("com.robotraconteur.action")
        self._motion_consts= self.node.GetConstants("tech.pyri.robotics.motion")
        self._command_modes = self.node.GetConstants("com.robotraconteur.robotics.robot")["RobotCommandMode"]

    def Abort(self):
        with self._wait_next_cv:
            if self._gen is not None:
                self._gen.Abort()
            self._aborted = True
            self._wait_next_cv.notify_all()
    
    def Close(self):
        with self._wait_next_cv:
            if self._gen is not None:
                self._gen.Close()
            self._closed = True
            self._wait_next_cv.notify_all()

    # def _run_jog_freespace(self, q_desired, max_velocity):

    #     def h(err):
    #         with self._wait_next_cv:
    #             self._jog_done = True
    #             if err is not None:
    #                 self._jog_err = err
    #             self._wait_next_cv.notify_all()

    #     self.robot.async_jog_freespace(q_desired, max_velocity, True, h,15)
        
    #     while True:
    #         if self._jog_done:
    #             if self._jog_err:
    #                 raise self._jog_err
    #             else:
    #                 break
    #         if self._closed:
    #             raise RR.StopIterationException("")
    #         if self._aborted:
    #             # Set robot to halt on abort to stop the robot
    #             self.robot.command_mode = 0
    #             raise RR.OperationAbortedException("")
    #         self._wait_next_cv.wait(0.1)

    #     # TODO: Is this necessary?
    #     time.sleep(0.5)

    def Next(self):
        with self._wait_next_cv:

            if self._aborted:
                raise RR.OperationAbortedException("")

            if self._closed:
                raise RR.StopIterationException("")

            ret = self._robot_motion_status()
            action_codes = self._action_consts["ActionStatusCode"]
            motion_codes = self._motion_consts["RobotMotionPlanarOpStateCode"]
            if self._aborted:
                raise RR.OperationAbortedException("")
            if self._closed:
                raise RR.StopIterationException("")
            if self._step == 0:
                ret.action_status = action_codes["running"]
                ret.motion_state = motion_codes["motion_step_planned"]
                ret.planned_motion = self.traj_before
                self._step = 1
                return ret
            if self._step == 1:
                ret.action_status = action_codes["running"]
                ret.motion_state = motion_codes["motion_step_complete"]
                ret.planned_motion = None
                self._step = 2
                self._execute_trajectory(self.traj_before)
                return ret
            if self._step == 2:
                move_finished = self._do_traj_next()
                if not move_finished:
                    ret.action_status = action_codes["running"]
                    ret.motion_state = motion_codes["motion_step_running"]
                    ret.planned_motion = self.traj_before
                    return ret
                ret.action_status = action_codes["running"]
                ret.motion_state = motion_codes["motion_step_complete"]
                ret.planned_motion = None
                self._step = 3
                return ret
            if self._step == 3:
                ret.action_status = action_codes["running"]
                ret.motion_state = motion_codes["motion_step_planned"]
                ret.planned_motion = self.traj_grab
                self._step = 4
                return ret
            if self._step == 4:
                ret.action_status = action_codes["running"]
                ret.motion_state = motion_codes["motion_step_running"]
                ret.planned_motion = self.traj_grab
                self._execute_trajectory(self.traj_grab)
                self._step = 5
                return ret
            if self._step == 5:
                move_finished = self._do_traj_next()
                if not move_finished:
                    ret.action_status = action_codes["running"]
                    ret.motion_state = motion_codes["motion_step_running"]
                    ret.planned_motion = self.traj_grab
                    return ret
                ret.action_status = action_codes["running"]
                ret.motion_state = motion_codes["motion_step_complete"]
                ret.planned_motion = None
                if self.grab:
                    self.tool.close()
                else:
                    self.tool.open()
                time.sleep(0.5)
                self._step = 6
                return ret
            if self._step == 6:
                ret.action_status = action_codes["running"]
                ret.motion_state = motion_codes["motion_step_planned"]
                ret.planned_motion = self.traj_after
                self._step = 7
                return ret
            if self._step == 7:
                ret.action_status = action_codes["running"]
                ret.motion_state = motion_codes["motion_step_running"]
                ret.planned_motion = self.traj_after
                self._execute_trajectory(self.traj_after)                
                self._step = 8
                return ret
            if self._step == 8:
                move_finished = self._do_traj_next()
                if not move_finished:
                    ret.action_status = action_codes["running"]
                    ret.motion_state = motion_codes["motion_step_running"]
                    ret.planned_motion = self.traj_after
                    return ret
                ret.action_status = action_codes["running"]
                ret.motion_state = motion_codes["motion_step_complete"]
                ret.planned_motion = None
                self._step = 9
                return ret
            else:
                raise RR.StopIterationException("")

    def _execute_trajectory(self, traj):
        
        robot_mode = self.robot.command_mode
        self._gen_next_done = False
        self._gen_next_err = None

        if robot_mode != self._command_modes["trajectory"]:
            self.robot.command_mode = self._command_modes["halt"]
            time.sleep(0.005)
            self.robot.command_mode = self._command_modes["trajectory"]

        self.gen=self.robot.execute_trajectory(traj)

    def _do_traj_next(self):
        if self.gen is None:
            return True

        def h(res,err):
            with self._wait_next_cv:
                self._gen_next_done = True
                if err is not None:
                    self._gen_next_err = err
                self._wait_next_cv.notify_all()

        self.gen.AsyncNext(None,h)

        while True:
            if self._gen_next_done:
                if self._gen_next_err:
                    if isinstance(self._gen_next_err,RR.StopIterationException):
                        return True
                    raise self._gen_next_err
                else:
                    break
            self._wait_next_cv.wait(0.1)

        return False

def main():

    parser = argparse.ArgumentParser(description="PyRI Robotics Motion Service")
    parser.add_argument("--device-info-file", type=argparse.FileType('r'),default=None,required=True,help="Device info file for robotics motion service (required)")
    parser.add_argument('--device-manager-url', type=str, default=None,required=True,help="Robot Raconteur URL for device manager service (required)")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM or SIGINT (Linux only)")
    parser.add_argument("--pyri-webui-server-port",type=int,default=8000,help="The PyRI WebUI port for websocket origin (default 8000)")

    args, _ = parser.parse_known_args()

    RRC.RegisterStdRobDefServiceTypes(RRN)
    RRN.RegisterServiceType(resources.read_text(__package__,'tech.pyri.robotics.motion.robdef'))

    with args.device_info_file:
        device_info_text = args.device_info_file.read()

    info_loader = InfoFileLoader(RRN)
    device_info, device_ident_fd = info_loader.LoadInfoFileFromString(device_info_text, "com.robotraconteur.device.DeviceInfo", "device")

    attributes_util = AttributesUtil(RRN)
    device_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(device_info)

    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("tech.pyri.robotics.motion", 55921) as node_setup:

        add_default_ws_origins(node_setup.tcp_transport,args.pyri_webui_server_port)

        # create object
        RoboticsMotion_inst = RoboticsMotion_impl(args.device_manager_url, device_info=device_info, node = RRN)
        # register service with service name "robotics_motion", type "tech.pyri.robotics.motion.RoboticsMotionService",
        # actual object: VisionArucoDetection_inst
        ctx = RRN.RegisterService("robotics_motion","tech.pyri.robotics.motion.RoboticsMotionService",RoboticsMotion_inst)
        ctx.SetServiceAttributes(device_attributes)

        #Wait for the user to shutdown the service
        if args.wait_signal:
            #Wait for shutdown signal if running in service mode
            print("Press Ctrl-C to quit...")
            import signal
            signal.sigwait([signal.SIGTERM,signal.SIGINT])
        else:
            #Wait for the user to shutdown the service
            if (sys.version_info > (3, 0)):
                input("Server started, press enter to quit...")
            else:
                raw_input("Server started, press enter to quit...")

if __name__ == '__main__':
    main()