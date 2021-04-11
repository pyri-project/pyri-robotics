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
from RobotRaconteurCompanion.Util.RobotUtil import RobotUtil

import time
import threading
import traceback
import general_robotics_toolbox as rox
from scipy.optimize import lsq_linear

class RoboticsJog_impl(object):
    def __init__(self, parent, robot_sub):
        self.robot_sub = robot_sub
        self.parent=parent

        self.robot_rox = None #Robotics Toolbox robot object
        self.robot_util = RobotUtil()

        res, robot = self.robot_sub.TryGetDefaultClient()
        if res:
            self.assign_robot_details(robot)

        robot_sub.ClientConnected += lambda a, b, robot: self.assign_robot_details(robot)
                
        self.degree_diff = 10 # in degrees
        self.dt = 0.01 #seconds, amount of time continuosly jog joints

        self.service_path = None
        self.jog_joints_joystick_group = -1
        self.jog_joints_joystick_last_enable_time = 0
        self.jog_joints_joystick_speed_perc = 10

        self.jog_cartesian_joystick_last_enable_time = 0
        self.jog_cartesian_joystick_frame = None
        self.jog_cartesian_joystick_speed_perc = 10

        self._lock = threading.Lock()
        self.joystick_last_command_time = 0

        self.joystick_deadzone = 0.35

    def RRServiceObjectInit(self, ctx, service_path):
        self.service_path = service_path

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
            print("Robot is not connected to RoboticsJog service yet!")


    def jog_joints(self, q_i, sign, speed_perc):
        print("Jog Joints is called")
        robot = self.robot
        if robot is not None:
            try:
                cur_q = self.get_current_joint_positions()

                if (self.num_joints < q_i):
                    print("Currently Controlled Robot only have " + str(self.num_joints) + " joints..")
                else:
                    joint_vel = np.zeros((self.num_joints,))
                    joint_vel[q_i-1] = sign*self.joint_vel_limits[q_i-1]*0.25

                    self.jog_joints_with_limits2(float(speed_perc)*0.01*joint_vel,0.2, False)
            except:
                # print("Specified joints might be out of range222")
                import traceback
                print(traceback.format_exc())
        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to RoboticsJog service yet!")
    
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

    # TODO: Remove this function
    def jog_joints_with_limits2(self,joint_velocity, timeout, wait=True):
        try:
            
            # Trim joint positions according to number of joints
            joint_velocity = joint_velocity[:self.num_joints]
            # self.robot.jog_joint(joint_position, max_velocity, relative, wait)
            self.robot.jog_joint(joint_velocity, timeout, wait)
        except:
            # print("Specified joints might be out of range222")
            import traceback
            print(traceback.format_exc())

    def jog_joints_zeros(self):
        print("Jog Joints Zeros is called")
        robot = self.robot
        if robot is not None:
                        
            self.jog_joints_with_limits(np.zeros((self.num_joints,)), self.joint_vel_limits,True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to RoboticsJog service yet!")

    def jog_joints_to_angles(self, joint_position, speed_perc):
        print("Jog Joints to Angles is called")
        # Similar to jog_joints_with_limits. But,
        # Moves the robot to the specified joint angles with max speed
        robot = self.robot
        if robot is not None:
            
            self.jog_joints_with_limits(joint_position[:self.num_joints], float(speed_perc)*0.01*self.joint_vel_limits,True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to RoboticsJog service yet!")

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
            print("Robot is not connected to RoboticsJog service yet!")

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
            print("Robot is not connected to RoboticsJog service yet!")

    def jog_joints_to_angles2(self, joint_position, speed_perc):
        print("Jog Joints to Angles2 (2 = with speed) is called")
        # Similar to jog_joints_with_limits. But,
        # Moves the robot to the specified joint angles with max speed percentage
        if self.robot is not None:
            
            self.jog_joints_with_limits(joint_position[:self.num_joints], float(speed_perc)*0.01*self.joint_vel_limits,True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to RoboticsJog service yet!")

    def enable_jog_joints_joystick(self, group, speed_perc):
        assert group == 0 or group == 1, "Group must be 0 or 1"
        assert self.num_joints == 6 or self.num_joints == 7, "Jog joystick only available for 6 or 7 axis robots"
        with self._lock:
            self.jog_joints_joystick_group = group
            self.jog_joints_joystick_speed_perc = float(speed_perc)
            self.jog_joints_joystick_last_enable_time = time.time()
        self.parent.joystick_enabled()

    def disable_jog_joints_joystick(self):
        with self._lock:
            self.jog_joints_joystick_group = -1
            
    def enable_jog_cartesian_joystick(self, speed_perc, frame):
        assert frame == "robot", "Only robot frame currently supported"        
        with self._lock:
            self.jog_cartesian_joystick_frame = frame
            self.jog_cartesian_joystick_speed_perc = float(speed_perc)
            self.jog_cartesian_joystick_last_enable_time = time.time()
        self.parent.joystick_enabled()

    def disable_jog_cartesian_joystick(self):
        with self._lock:
            self.jog_cartesian_joystick_frame = None

    def joystick_state_cb(self, joy_state):
        with self._lock:

            group = self.jog_joints_joystick_group
            frame = self.jog_cartesian_joystick_frame
            if group < 0 and frame is None:
                return
                        
            # Rate limit command sends
            now = time.time()
            
            have_command = False
            if now - self.jog_joints_joystick_last_enable_time > 0.2:
                self.jog_joints_joystick_group = -1                
            else:
                have_command = True
            if now - self.jog_cartesian_joystick_last_enable_time > 0.2:
                self.jog_cartesian_joystick_frame = None
            else:
                have_command = True

            if not have_command:
                return

            if now - self.joystick_last_command_time < 0.05:
                return
            self.joystick_last_command_time = now
        try:
            joy_vals = joy_state.axes / 32767.0

            for i in range(len(joy_vals)):
                if joy_vals[i] > 0:
                    if joy_vals[i] < self.joystick_deadzone:
                        joy_vals[i] = 0
                    else:
                        joy_vals[i] = (joy_vals[i]-self.joystick_deadzone) * (1-self.joystick_deadzone)
                if joy_vals[i] < 0:
                    if -joy_vals[i] < self.joystick_deadzone:
                        joy_vals[i] = 0
                    else:
                        joy_vals[i] = (joy_vals[i]+self.joystick_deadzone) * (1-self.joystick_deadzone)

            if group >= 0:
                jog_command = np.zeros((self.num_joints,),dtype=np.float64)

                if self.num_joints == 6:
                    if group == 0:
                        jog_command[0:3] = joy_vals[0:3]
                    else:
                        jog_command[3:6] = joy_vals[0:3]
                elif self.num_joints == 7:
                    if group == 0:
                        jog_command[0:4] = joy_vals[0:4]
                    else:
                        jog_command[4:7] = joy_vals[0:3]
                else:
                    return

                jog_command = 0.01*self.jog_joints_joystick_speed_perc*np.multiply(jog_command,self.joint_vel_limits)*0.25
                self.jog_joints_with_limits2(jog_command,0.2, False)
            elif frame is not None:
                if frame == "robot":
                    R_axis = joy_vals[3:6]*np.deg2rad(45)
                    P_axis = joy_vals[0:3]*0.254

                    R_spacemouse = rox.rot([1,0,0],np.pi)
                    R_axis = R_spacemouse @ R_axis
                    P_axis = R_spacemouse @ P_axis

                    #TODO: Rotate frame about X 180 degrees
                    qdot = self.update_qdot2(R_axis,P_axis, self.jog_cartesian_joystick_speed_perc) 
                    self.robot.jog_joint(qdot, 0.2, False)
        except:
            traceback.print_exc()

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

            self.robot_rox = self.robot_util.robot_info_to_rox_robot(self.robot_info,0)
        else:
            # Give an error message to show that the robot is not connected
            print("Assign robot details failed. Robot is not connected to RoboticsJog service yet!")

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

    ## Cartesian jog mode support

    def jog_cartesian(self, vel, speed_perc, frame ):
        print("jog_cartesian is called")

        if self.robot is not None:
            vel2 = RRN.NamedArrayToArray(vel)[0]
            R_axis = vel2[0:3]*np.deg2rad(45)
            P_axis = vel2[3:6]*0.254
            ## Jog the robot in cartesian space
            try:
                # calculate the required joint speeds (q_dot)
                qdot = self.update_qdot2(R_axis,P_axis, speed_perc) 

                self.robot.jog_joint(qdot, 0.2, False)
            except:
                traceback.print_exc() 

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogCartesianSpace service yet!")

    def update_qdot2(self, R_axis, P_axis, speed_perc): # inverse velocity kinematics that uses LSQ Linear solver        

        # Get the corresponding joint angles at that time
        d_q = self.get_current_joint_positions()
        q_cur = d_q.reshape((self.num_joints,1)) 

        # Update the end effector pose info    
        pose = rox.fwdkin(self.robot_rox,q_cur.flatten())
        R_cur = pose.R
        p_cur = pose.p
        
        #calculate current Jacobian
        J0T = rox.robotjacobian(self.robot_rox,q_cur.flatten())
        
        # Transform Jacobian to End effector frame from the base frame
        Tr = np.zeros((6,6))
        Tr[:3,:3] = R_cur.T 
        Tr[3:,3:] = R_cur.T
        #J0T = Tr @ J0T
        
        # Normalize R_axis and P_axis
        #R_axis = R_axis/(np.linalg.norm(R_axis))
        #P_axis = P_axis/(np.linalg.norm(P_axis))

        # Create the corresponding velocities
        w = R_axis #* self.rotate_angle
        v = P_axis #* self.move_distance

        b = np.concatenate([w,v])*0.01*speed_perc
        np.nan_to_num(b, copy=False, nan=0.0, posinf=None, neginf=None)
        # print(b)
        # print(J0T)

        joint_vel_limits = 0.01*speed_perc*self.joint_vel_limits
        res = lsq_linear(J0T,b,bounds=(-1.0*joint_vel_limits,joint_vel_limits))

        if res.success: 
            qdot_star = res.x 
        else:
            print("Any solution could not found")
            qdot_star = np.zeros(self.num_joints)

        print("qdot_star:")
        print(qdot_star)
        # print("self.joint_vel_limits")
        # print(self.joint_vel_limits)

        # q_dot = self.normalize_dq(qdot_star)
        q_dot = qdot_star
        
        return q_dot

class JogTool_impl:
    def __init__(self, tool_sub):
        self.tool_sub = tool_sub

    def open(self):
        self.tool_sub.GetDefaultClient().open()

    def close(self):
        self.tool_sub.GetDefaultClient().close()

    def setf_position(self,command):
        self.tool_sub.GetDefaultClient().setf_command(command)

class RoboticsJogService_impl:
    def __init__(self, device_manager_url, device_info = None, node : RR.RobotRaconteurNode = None):
        if node is None:
            self._node = RR.RobotRaconteurNode.s
        else:
            self._node = node
        self.device_info = device_info

        self._lock = threading.Lock()

        self._jogs={}
        self._tools={}

        self.service_path = None
        self.ctx = None

        self._device_manager = DeviceManagerClient(device_manager_url)
        self._device_manager.device_added += self._device_added
        self._device_manager.device_removed += self._device_removed
        self._device_manager.refresh_devices(5)

        self._joystick_sub = None
        self._joystick_wire_sub = None

    def RRServiceObjectInit(self, ctx, service_path):
        self.service_path = service_path
        self.ctx = ctx

    def get_jog(self, robot_name):        
        with self._lock:
            jog = RoboticsJog_impl(self, self._device_manager.get_device_subscription(robot_name))        
            self._jogs[robot_name] = jog
            return jog, "tech.pyri.robotics.jog.JogRobot"

    def get_tool(self, tool_name):
        with self._lock:
            tool = JogTool_impl(self._device_manager.get_device_subscription(tool_name))
            self._tools[tool_name] = tool
            return tool, "tech.pyri.robotics.jog.JogTool"

    def _device_added(self, local_device_name):
       pass 

    def _device_removed(self, local_device_name):
        with self._lock:
            if local_device_name in self._jogs:
                service_path = self._jogs[local_device_name].service_path
                del self._jogs[local_device_name]
                try:
                    self.ctx.ReleaseServicePath(service_path)
                except:
                    pass
            if local_device_name in self._tools:
                service_path = self._tools[local_device_name].service_path
                del self._tools[local_device_name]
                try:
                    self.ctx.ReleaseServicePath(service_path)
                except:
                    pass

            if local_device_name == "joystick":
                if self._joystick_sub is not None:
                    try:
                        self._joystick_sub.Close()
                        self._joystick_wire_sub.Close()
                    except:
                        pass
                    

    def joystick_enabled(self):
        with self._lock:
            if self._joystick_sub is None or self._joystick_wire_sub is None:
                self._joystick_sub = self._device_manager.get_device_subscription("joystick")
                self._joystick_wire_sub = self._joystick_sub.SubscribeWire("joystick_state")
                self._joystick_wire_sub.WireValueChanged += self._joystick_state_cb

    def joystick_disabled(self):
        pass

    def _joystick_state_cb(self, sub, joy_state, ts):
        with self._lock:
            jogs = list(self._jogs.values())

        for j in jogs:
            try:
                j.joystick_state_cb(joy_state)
            except:
                traceback.print_exc()
                
    
def main():

    parser = argparse.ArgumentParser(description="PyRI Jog Joint Service")    
    parser.add_argument("--device-info-file", type=argparse.FileType('r'),default=None,required=True,help="Device info file for devices states service (required)")
    parser.add_argument('--device-manager-url', type=str, default=None,required=True,help="Robot Raconteur URL for device manager service (required)")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM or SIGINT (Linux only)")
    
    args, _ = parser.parse_known_args()

    RRC.RegisterStdRobDefServiceTypes(RRN)
    RRN.RegisterServiceType(resources.read_text(__package__,'tech.pyri.robotics.jog.robdef'))

    with args.device_info_file:
        device_info_text = args.device_info_file.read()

    info_loader = InfoFileLoader(RRN)
    device_info, device_ident_fd = info_loader.LoadInfoFileFromString(device_info_text, "com.robotraconteur.device.DeviceInfo", "device")

    attributes_util = AttributesUtil(RRN)
    device_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(device_info)


    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("tech.pyri.robotics.jog", 55906) as node_setup:

        # register service type
        

        # create object
        RoboticsJogService_inst = RoboticsJogService_impl(args.device_manager_url, device_info=device_info, node = RRN)
        # register service with service name "robotics_jog", type "tech.pyri.robotics.jog.RoboticsJogService", actual object: RoboticsJogService_inst
        ctx = RRN.RegisterService("robotics_jog","tech.pyri.robotics.jog.RoboticsJogService",RoboticsJogService_inst)
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