# @burakaksoy plugin-jogJointSpace-service.py

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

import time

class JogJointSpace_impl(object):
    def __init__(self, robot_sub):
        self.robot_sub = robot_sub

        res, robot = self.robot_sub.TryGetDefaultClient()
        if res:
            self.assign_robot_details(robot)

        robot_sub.ClientConnected += lambda a, b, robot: self.assign_robot_details(robot)
        
        self.robot_rox = None #Robotics Toolbox robot object

        self.degree_diff = 10 # in degrees
        self.dt = 0.01 #seconds, amount of time continuosly jog joints


    @property
    def robot(self):
        res, r = self.robot_sub.TryGetDefaultClient()
        if not res:
            return None
        return r

    def stop_joints(self):
        print("stop_joints is called")
        robot = self.robot
        if robot is not None:
            if self.is_enabled_velocity_mode == False:
                # Put the robot to POSITION mode
                robot.command_mode = self.halt_mode
        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")


    def jog_joints3(self, q_i, sign):
        print("Jog Joints3 is called")
        robot = self.robot
        if robot is not None:
            try:
                cur_q = self.get_current_joint_positions()

                if (self.num_joints < q_i):
                    print("Currently Controlled Robot only have " + str(self.num_joints) + " joints..")
                else:
                    joint_vel = np.zeros((self.num_joints,))
                    joint_vel[q_i-1] = sign*self.joint_vel_limits[q_i-1]*0.25

                    self.jog_joints_with_limits2(cur_q, joint_vel*0.2,0.2, False)
            except:
                # print("Specified joints might be out of range222")
                import traceback
                print(traceback.format_exc())
        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")


    def jog_joints(self, q_i, sign):
        print("Jog Joints is called")
        robot = self.robot
        if robot is not None:
           
            # Jog the robot
            # # get the current joint angles
            cur_q = self.get_current_joint_positions()

            if (self.num_joints < q_i):
                print("Currently Controlled Robot only have " + str(self.num_joints) + " joints..")
            else:
                joint_diff = np.zeros((self.num_joints,))
                joint_diff[q_i-1] = sign*np.deg2rad(self.degree_diff)

                # self.jog_joints_with_limits((cur_q + joint_diff),(cur_q + joint_diff),joint_diff, self.joint_vel_limits,True,True)
                self.jog_joints_with_limits((cur_q + joint_diff), self.joint_vel_limits,True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")

    def jog_joints_with_limits(self,joint_position, max_velocity,wait=True):
        if not (joint_position < self.joint_upper_limits).all() or not (joint_position > self.joint_lower_limits).all():
            print("Specified joints might be out of range")
        else:
            try:
                
                # Trim joint positions according to number of joints
                joint_position = joint_position[:self.num_joints]
                # self.robot.jog_joint(joint_position, max_velocity, relative, wait)
                self.robot.jog_freespace(joint_position, max_velocity, wait)
            except:
                # print("Specified joints might be out of range222")
                import traceback
                print(traceback.format_exc())

    def jog_joints_with_limits2(self,joint_position, joint_velocity, timeout, wait=True):
        if not (joint_position <= self.joint_upper_limits).all() or not (joint_position >= self.joint_lower_limits).all():
            print("Specified joints might be out of range")
        else:
            try:
                
                # Trim joint positions according to number of joints
                joint_velocity = joint_velocity[:self.num_joints]
                # self.robot.jog_joint(joint_position, max_velocity, relative, wait)
                self.robot.jog_joint(joint_velocity, timeout, wait)
            except:
                # print("Specified joints might be out of range222")
                import traceback
                print(traceback.format_exc())

    def jog_joints_gamepad(self,joint_speed_constants):
        print("Jog Joints Gamepad is called")
        robot = self.robot
        if robot is not None:
                        
            # get the current joint angles
            cur_q = self.get_current_joint_positions()

            # Trim joint speed constants accordingto number of joints
            joint_speed_constants = joint_speed_constants[:self.num_joints]

            signs = np.divide(np.abs(joint_speed_constants),joint_speed_constants)
            np.nan_to_num(signs, copy=False)

            joint_diff = np.ones((self.num_joints,))
            joint_diff = np.multiply(signs,np.deg2rad(self.degree_diff))

            # self.jog_joints_with_limits((cur_q + joint_diff),(cur_q + joint_diff),joint_diff, self.joint_vel_limits,True,True)
            self.jog_joints_with_limits((cur_q + joint_diff), self.joint_vel_limits,False)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")

    def jog_joints_zeros(self):
        print("Jog Joints Zeros is called")
        robot = self.robot
        if robot is not None:
                        
            self.jog_joints_with_limits(np.zeros((self.num_joints,)), self.joint_vel_limits,True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")

    def jog_joints_to_angles(self, joint_position):
        print("Jog Joints to Angles is called")
        # Similar to jog_joints_with_limits. But,
        # Moves the robot to the specified joint angles with max speed
        robot = self.robot
        if robot is not None:
            
            self.jog_joints_with_limits(joint_position[:self.num_joints], self.joint_vel_limits,True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")

    # For blockly
    def jog_joints_to_angles_relative(self,diff_joint_position, speed_perc):
        print("Jog Joints to Angles relatively is called")
        robot = self.robot
        if robot is not None:
            
            # # get the current joint angles
            cur_q = self.get_current_joint_positions()
            diff_joint_position = diff_joint_position[:self.num_joints]

            self.jog_joints_with_limits((diff_joint_position+cur_q), float(speed_perc)*0.01*self.joint_vel_limits,True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")

    def jog_joint_to_angle(self, joint, position, speed_perc):
        print("Jog Joint to Angle is called")
        robot = self.robot
        if robot is not None:
            
            # # get the current joint angles
            cur_q = self.get_current_joint_positions()
            cur_q[joint] = position

            self.jog_joints_with_limits(cur_q, float(speed_perc)*0.01*self.joint_vel_limits,True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")

    def jog_joints_to_angles2(self, joint_position, speed_perc):
        print("Jog Joints to Angles2 (2 = with speed) is called")
        # Similar to jog_joints_with_limits. But,
        # Moves the robot to the specified joint angles with max speed percentage
        if self.robot is not None:
            
            self.jog_joints_with_limits(joint_position[:self.num_joints], float(speed_perc)*0.01*self.joint_vel_limits,True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")


    def assign_robot_details(self, robot):
        
        if robot is not None:

            self.robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
            self.halt_mode = self.robot_const["RobotCommandMode"]["halt"]
            self.jog_mode = self.robot_const["RobotCommandMode"]["jog"]
            
            self.position_mode = self.robot_const["RobotCommandMode"]["position_command"]

            self.robot_info = robot.robot_info
            self.joint_info = self.robot_info.joint_info # A list of jointInfo

            self.joint_types = [] # A list or array of N numbers containing the joint type. 1 for rotary, 3 for prismatic
            self.joint_lower_limits = [] # list or numpy.array
            self.joint_upper_limits = [] # list or numpy.array
            self.joint_vel_limits = [] # list or numpy.array
            self.joint_acc_limits = [] # list or numpy.array
            self.joint_names = [] # list of string
            self.joint_uuids = [] 
            for joint in self.joint_info:
                self.joint_types.append(joint.joint_type)
                self.joint_lower_limits.append(joint.joint_limits.lower)
                self.joint_upper_limits.append(joint.joint_limits.upper)
                self.joint_vel_limits.append(joint.joint_limits.velocity)
                self.joint_acc_limits.append(joint.joint_limits.acceleration)
                self.joint_names.append(joint.joint_identifier.name)
                self.joint_uuids.append(joint.joint_identifier.uuid)
                
            # convert them to numpy arrays
            self.joint_types = np.asarray(self.joint_types)
            self.joint_lower_limits = np.asarray(self.joint_lower_limits)
            self.joint_upper_limits = np.asarray(self.joint_upper_limits)
            self.joint_vel_limits = np.asarray(self.joint_vel_limits)
            self.joint_acc_limits = np.asarray(self.joint_acc_limits)                

            self.num_joints = len(self.joint_info)
        else:
            # Give an error message to show that the robot is not connected
            print("Assign robot details failed. Robot is not connected to JogJointSpace service yet!")

    def get_current_joint_positions(self):
        cur_robot_state = self.robot.robot_state.PeekInValue()    
        cur_q = cur_robot_state[0].joint_position
        return cur_q # in radian ndarray

    def setf_halt_mode(self):
        self.robot.command_mode = self.halt_mode

    def setf_jog_mode(self):
        self.robot.command_mode = self.halt_mode
        time.sleep(0.1)
        self.robot.command_mode = self.jog_mode

class JogTool_impl:
    def __init__(self, tool_sub):
        self.tool_sub = tool_sub

    def open(self):
        self.tool_sub.GetDefaultClient().open()

    def close(self):
        self.tool_sub.GetDefaultClient().close()

    def setf_position(self,command):
        self.tool_sub.GetDefaultClient().setf_command(command)

class JogJointSpaceService_impl:
    def __init__(self, device_manager_url, device_info = None, node : RR.RobotRaconteurNode = None):
        if node is None:
            self._node = RR.RobotRaconteurNode.s
        else:
            self._node = node
        self.device_info = device_info

        self._device_manager = DeviceManagerClient(device_manager_url)
        self._device_manager.refresh_devices(5)

    def get_jog(self, robot_name):
        
        
        return JogJointSpace_impl(self._device_manager.get_device_subscription(robot_name)), "tech.pyri.robotics.pluginJogJointSpace.JogJointSpace"

    def get_tool(self, tool_name):
        
        
        return JogTool_impl(self._device_manager.get_device_subscription(tool_name)), "tech.pyri.robotics.pluginJogJointSpace.JogTool"



def main():

    parser = argparse.ArgumentParser(description="PyRI Jog Joint Service")    
    parser.add_argument("--device-info-file", type=argparse.FileType('r'),default=None,required=True,help="Device info file for devices states service (required)")
    parser.add_argument('--device-manager-url', type=str, default=None,required=True,help="Robot Raconteur URL for device manager service (required)")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM or SIGINT (Linux only)")
    
    args, _ = parser.parse_known_args()

    RRC.RegisterStdRobDefServiceTypes(RRN)
    RRN.RegisterServiceType(resources.read_text(__package__,'tech.pyri.robotics.pluginJogJointSpace.robdef'))

    with args.device_info_file:
        device_info_text = args.device_info_file.read()

    info_loader = InfoFileLoader(RRN)
    device_info, device_ident_fd = info_loader.LoadInfoFileFromString(device_info_text, "com.robotraconteur.device.DeviceInfo", "device")

    attributes_util = AttributesUtil(RRN)
    device_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(device_info)


    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-jogJointSpace-service", 55906) as node_setup:

        # register service type
        

        # create object
        JogJointSpaceService_inst = JogJointSpaceService_impl(args.device_manager_url, device_info=device_info, node = RRN)
        # register service with service name "JogJointSpace", type "experimental.pluginJogJointSpace.JogJointSpace", actual object: JogJointSpace_inst
        ctx = RRN.RegisterService("JogJointSpace","tech.pyri.robotics.pluginJogJointSpace.JogJointSpaceService",JogJointSpaceService_inst)
        ctx.SetServiceAttributes(device_attributes)

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