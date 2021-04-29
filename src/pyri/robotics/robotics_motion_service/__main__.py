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

    def _do_grab_object_planar(self, robot_local_device_name, tool_local_device_name, robot_origin_calib_global_name,
        reference_pose_global_name, object_x, object_y, object_theta, z_offset_before, z_offset_grab, speed_perc):

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

        # Compute inverse kinematics
        q_grab_before, res = invkin.update_ik_info3(rox_robot, T_grab_pose_before_r, q_current)
        assert res, "Invalid setpoint: invkin did not converge"
        q_grab, res = invkin.update_ik_info3(rox_robot, T_grab_pose_r, q_current)
        assert res, "Invalid setpoint: invkin did not converge"

        print(f"q_grab_before: {q_grab_before}")
        print(f"q_grab: {q_grab}")
        print()

        max_velocity = rox_robot.joint_vel_limit * (speed_perc/100.0)

        gen = PickPlaceMotionGenerator(robot, rox_robot, tool, q_grab_before,
            q_grab, max_velocity, self._node)

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
        
        return self._do_grab_object_planar(robot_local_device_name, tool_local_device_name, robot_origin_calib_global_name,
            reference_pose_global_name, object_x, object_y, object_theta, z_offset_before, z_offset_grab, speed_perc)

    def grab_object_planar2(self, robot_local_device_name, tool_local_device_name, robot_origin_calib_global_name,
        reference_pose_global_name, object_world_pose, z_offset_before, z_offset_grab, speed_perc):

        object_x = object_world_pose["position"]["x"]
        object_y = object_world_pose["position"]["y"]
        object_rpy = self._geom_util.quaternion_to_rpy(object_world_pose["orientation"])
        assert np.abs(object_rpy[0]) < np.deg2rad(5), "Object is not flat on surface!"
        assert np.abs(object_rpy[1]) < np.deg2rad(5), "Object is not flat on surface!"
        object_theta = object_rpy[2]

        return self._do_grab_object_planar(robot_local_device_name, tool_local_device_name, robot_origin_calib_global_name,
            reference_pose_global_name, object_x, object_y, object_theta, z_offset_before, z_offset_grab, speed_perc)


class PickPlaceMotionGenerator:

    def __init__(self, robot, rox_robot, tool, q_grab_before, q_grab, max_velocity, node):
        self.node = node
        self.robot = robot
        self.rox_robot = rox_robot
        self.tool = tool
        self.q_grab_before = q_grab_before
        self.q_grab = q_grab
        self.max_velocity = max_velocity

        self._wait_next_cv = threading.Condition()
        self._aborted = False
        self._closed = False
        self._step = 0
        self._jog_done = False
        self._jog_err = None

        self._robot_motion_status = self.node.GetStructureType("tech.pyri.robotics.motion.RobotMotionStatus")
        self._action_consts = self.node.GetConstants("com.robotraconteur.action")
        self._motion_consts= self.node.GetConstants("tech.pyri.robotics.motion")

    def Abort(self):
        with self._wait_next_cv:
            self._aborted = True
            self._wait_next_cv.notify_all()
    
    def Close(self):
        with self._wait_next_cv:
            self._closed = True
            self._wait_next_cv.notify_all()

    def _run_jog_freespace(self, q_desired, max_velocity):

        def h(err):
            with self._wait_next_cv:
                self._jog_done = True
                if err is not None:
                    self._jog_err = err
                self._wait_next_cv.notify_all()

        self.robot.async_jog_freespace(q_desired, max_velocity, True, h,15)
        
        while True:
            if self._jog_done:
                if self._jog_err:
                    raise self._jog_err
                else:
                    break
            if self._closed:
                raise RR.StopIterationException("")
            if self._aborted:
                # Set robot to halt on abort to stop the robot
                self.robot.command_mode = 0
                raise RR.OperationAbortedException("")
            self._wait_next_cv.wait(0.1)

        # TODO: Is this necessary?
        time.sleep(0.5)

    def Next(self):
        with self._wait_next_cv:
            ret = self._robot_motion_status()
            action_codes = self._action_consts["ActionStatusCode"]
            motion_codes = self._motion_consts["RobotMotionStateCode"]
            if self._aborted:
                raise RR.OperationAbortedException("")
            if self._closed:
                raise RR.StopIterationException("")
            if self._step == 0:
                ret.action_status = action_codes["running"]
                ret.motion_state = motion_codes["motion_step_planned"]
                ret.planned_motion = self._fill_trajectory(self.q_grab_before)
                self._step = 1
                return ret
            if self._step == 1:
                ret.action_status = action_codes["running"]
                ret.motion_state = motion_codes["motion_step_complete"]
                ret.planned_motion = None
                self._step = 2
                self._run_jog_freespace(self.q_grab_before, self.max_velocity)
                return ret
            if self._step == 2:
                ret.action_status = action_codes["running"]
                ret.motion_state = motion_codes["motion_step_planned"]
                ret.planned_motion = self._fill_trajectory(self.q_grab)
                self._step = 3
                return ret
            if self._step == 3:
                ret.action_status = action_codes["running"]
                ret.motion_state = motion_codes["motion_step_complete"]
                ret.planned_motion = None
                self._run_jog_freespace(self.q_grab, self.max_velocity*.25)
                self._step = 4
                return ret
            if self._step == 4:
                ret.action_status = action_codes["running"]
                ret.motion_state = motion_codes["motion_step_planned"]
                ret.planned_motion = self._fill_trajectory(self.q_grab_before)
                self._step = 5
                return ret
            if self._step == 5:
                ret.action_status = action_codes["running"]
                ret.motion_state = motion_codes["motion_step_complete"]
                ret.planned_motion = None
                self._run_jog_freespace(self.q_grab_before, self.max_velocity*.25)
                self._step = 6
                return ret
            else:
                raise RR.StopIterationException("")

    def _fill_trajectory(self,q_target):
        # TODO: fill in the trajectory for viewer
        return None






def main():

    parser = argparse.ArgumentParser(description="PyRI Robotics Motion Service")
    parser.add_argument("--device-info-file", type=argparse.FileType('r'),default=None,required=True,help="Device info file for robotics motion service (required)")
    parser.add_argument('--device-manager-url', type=str, default=None,required=True,help="Robot Raconteur URL for device manager service (required)")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM or SIGINT (Linux only)")

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