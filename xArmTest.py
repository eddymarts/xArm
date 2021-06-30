#!/usr/bin/env python3
# Coded by Eddy Martínez

# Using xArm by UFACTORY, Inc.
# All rights reserved.

import os
import sys
import time
from xarm.wrapper import XArmAPI


class Gripper:
    """ Class to represent electric gripper of xArm. """

    def __init__(self, arm: XArmAPI):
        self.arm = self.arm
        time.sleep(0.5)
        if self.arm.warn_code != 0:
            self.arm.clean_warn()
        if self.arm.error_code != 0:
            self.arm.clean_error()
    
    def open(self):
        self.arm.set_tgpio_digital(0, 1)
        self.arm.set_tgpio_digital(1, 1)

    def close(self):
        self.arm.set_tgpio_digital(0, 0)
        self.arm.set_tgpio_digital(1, 0)

class Analog:
    """ Class to represent the analog inputs of xArm controller. """
    def __init__(self, arm: XArmAPI):
        self.arm = arm
        
        time.sleep(0.5)
        if self.arm.warn_code != 0:
            self.arm.clean_warn()
        if self.arm.error_code != 0:
            self.arm.clean_error()

    def get_analog(self):
        analog_inputs = []

        while self.arm.connected and self.arm.error_code != 19 and self.arm.error_code != 28:
            for i in range(2):
                analog_inputs.append(
                    {code: value for code, value in self.arm.get_tgpio_analog(i)})
            
        return analog_inputs
    
    def __repr__(self):
        return str(
            [f"code: {log.keys()}\nanalog: {log.values()}" for log in self.get_analog()])

class xArmTest:
    """ Class to represent xArm different settings. """

    def __init__(self, IP):
        self.initiate(IP)
        self.gripper = Gripper(self.arm)
        self.analog = Analog(self.arm)
    
    def initiate(self, IP):
        sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
        if len(sys.argv) >= 2:
            ip = sys.argv[1]
        else:
            try:
                from configparser import ConfigParser
                parser = ConfigParser()
                parser.read('../robot.conf')
                ip = parser.get('xArm', 'ip')
            except:
                ip = IP
                if not ip:
                    print('input error, exit')
                    sys.exit(1)

        self.arm = XArmAPI(ip)

    def disconnect(self):
        self.arm.reset(wait=True)
        self.arm.disconnect()
    
    def analog_test(self):
        print(self.analog)

    def set_fence(self):
        x_max, x_min, y_max, y_min, z_max, z_min = 500, -500, 600, -600, 400, -400
        code = self.arm.set_reduced_tcp_boundary([x_max, x_min, y_max, y_min, z_max, z_min])
        print('set_reduced_tcp_boundary, code={}'.format(code))
        code = self.arm.set_fense_mode(True)
        print('set_fense_mode, code={}'.format(code))

    def motion_enable(self):
        self.arm.motion_enable(enable=True)
        self.arm.clean_error()
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(1)
        self.arm.reset(wait=True)

    def absolute_linear_motion_test(self):
        self.motion_enable()
        self.arm.set_position(x=300, y=0, z=150, roll=-180, pitch=0, yaw=0, speed=100, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=True))
        self.arm.set_position(x=300, y=200, z=250, roll=-180, pitch=0, yaw=0, speed=200, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=True))
        self.arm.set_position(x=500, y=200, z=150, roll=-180, pitch=0, yaw=0, speed=300, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=True))
        self.arm.set_position(x=500, y=-200, z=250, roll=-180, pitch=0, yaw=0, speed=400, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=True))
        self.arm.set_position(x=300, y=-200, z=150, roll=-180, pitch=0, yaw=0, speed=500, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=True))
        self.arm.set_position(x=300, y=0, z=250, roll=-180, pitch=0, yaw=0, speed=600, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=True))

    def relative_linear_motion_test(self):
        self.motion_enable()
        self.arm.set_position(x=100, relative=True, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=False))
        self.arm.set_position(y=200, z=100, relative=True, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=False))
        self.arm.set_position(x=200, z=-100, relative=True, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=False))
        self.arm.set_position(y=-400, z=100, relative=True, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=False))
        self.arm.set_position(x=-200, z=-100, relative=True, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=False))
        self.arm.set_position(y=200, z=100, relative=True, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=False))

    def arc_motion_test(self):
        self.motion_enable()
        paths = [
            [300, 0, 150, -180, 0, 0],
            [300, 200, 250, -180, 0, 0],
            [500, 200, 150, -180, 0, 0],
            [500, -200, 250, -180, 0, 0],
            [300, -200, 150, -180, 0, 0],
            [300, 0, 250, -180, 0, 0],
            [300, 200, 350, -180, 0, 0],
            [500, 200, 250, -180, 0, 0],
            [500, -200, 350, -180, 0, 0],
            [300, -200, 250, -180, 0, 0],
            [300, 0, 350, -180, 0, 0],
        ]

        self.arm.move_arc_lines(paths, speed=300, times=3, wait=True)

    def circular_motion_test(self):
        """ Not an actual circle to avoid hitting the table. """
        self.motion_enable()
        poses = [
            [300,  0,   150, -180, 0, 0],
            [300,  100, 150, -180, 0, 0],
            [400,  100, 150, -180, 0, 0],
            [400, 100, 150, -180, 0, 0],
            [300,  0,   300, -180, 0, 0]
        ]

        ret = self.arm.set_position(*poses[0], speed=50, mvacc=100, wait=False)
        print('set_position, ret: {}'.format(ret))

        ret = self.arm.move_circle(pose1=poses[1], pose2=poses[2], percent=50, speed=200, mvacc=1000, wait=True)
        print('move_circle, ret: {}'.format(ret))

        ret = self.arm.move_circle(pose1=poses[3], pose2=poses[4], percent=200, speed=200, mvacc=1000, wait=True)
        print('move_circle, ret: {}'.format(ret))
    
    def trajectory_record_test(self):
        """ 
        xArm will be idle for 20 seconds to be moved by hand.
        The trajectory of that movement will be recorded.
        """

        # Recording trajectory test
        # Turn on manual mode before recording
        self.arm.set_mode(2)
        self.arm.set_state(0)

        self.arm.start_record_trajectory()

        # Manually move the self.arm for 20 seconds. Trajectory will be recorded.
        time.sleep(20)

        self.arm.stop_record_trajectory()
        self.arm.save_record_trajectory('test.traj')
        self.record = True

        time.sleep(1)
    
    def trajectory_play_test(self):
        """ xArm will play recorded trajectory, if any. """
        if self.record:
            # Turn off manual mode after recording
            self.motion_enable()

            # Play trajectory
            self.arm.load_trajectory('test.traj')
            self.arm.playback_trajectory()
        else:
            print("No trajectory recorded.")
    
    def gripper_test(self):
        self.motion_enable()
        self.arm.set_position(x=300, y=0, z=150, roll=-180, pitch=0, yaw=0, speed=100, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=True))
        self.arm.set_position(x=300, y=200, z=250, roll=-180, pitch=0, yaw=0, speed=200, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=True))
        self.arm.set_position(x=500, y=200, z=150, roll=-180, pitch=0, yaw=0, speed=300, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=True))
        self.arm.set_position(x=500, y=-200, z=250, roll=-180, pitch=0, yaw=0, speed=400, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=True))

        self.gripper.open()

        self.arm.set_position(x=300, y=-200, z=150, roll=-180, pitch=0, yaw=0, speed=500, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=True))
        self.arm.set_position(x=300, y=0, z=250, roll=-180, pitch=0, yaw=0, speed=600, wait=True)
        print(self.arm.get_position(), self.arm.get_position(is_radian=True))

        self.gripper.close()
    
    def demo(self):
        self.set_fence()
        self.analog_test()
        self.absolute_linear_motion_test()
        self.relative_linear_motion_test()
        self.arc_motion_test()
        self.circular_motion_test()
        self.gripper_test()
        self.trajectory_record_test()
        self.trajectory_play_test()
        self.disconnect()

arm_test = xArmTest('192.168.1.242')   
arm_test.demo() 