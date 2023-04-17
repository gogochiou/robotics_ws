#!/usr/bin/env python

from __future__ import print_function

import threading

# import roslib; roslib.load_manifest('keyboardControl')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

# ArmMsg = Float64MultiArray

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving joint one step:
   1    2    3    4
  ------------------
   u    i    o    p
   j    k    l    ;

q/a : increase/decrease rad each step
anything else : stop
CTRL-C to quit
"""

moveBindings = {
        'u':(1.0,0,0,0),
        'i':(0,1.0,0,0),
        'o':(0,0,1.0,0),
        'p':(0,0,0,1.0),
        'j':(-1.0,0,0,0),
        'k':(0,-1.0,0,0),
        'l':(0,0,-1.0,0),
        ';':(0,0,0,-1.0)
    }

stepBindings={
        'q':0.1,
        'a':-0.1,
    }

class PublishThread(threading.Thread):
    def __init__(self, rate, tar_device):
        super(PublishThread, self).__init__()
        if tar_device == 'simulation':
            self.publisher = rospy.Publisher('/myrobot/four_joints_position_controllers/command', Float64MultiArray, queue_size = 1)
        elif tar_device == 'real':
            self.publisher = rospy.Publisher('/real_robot_arm_joint', Float64MultiArray, queue_size = 1)
        
        self.joint1 = 0.0
        self.joint2 = 0.0
        self.joint3 = 0.0
        self.joint4 = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, j1, j2, j3, j4):
        self.condition.acquire()
        self.joint1 = j1
        self.joint2 = j2
        self.joint3 = j3
        self.joint4 = j4
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0)
        self.join()

    def run(self):

        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            theta_cmd = [self.joint1, self.joint2, self.joint3, self.joint4]
            arm_msg = Float64MultiArray(data=theta_cmd)

            self.condition.release()

            # Publish.
            self.publisher.publish(arm_msg)

        # Publish stop message when thread exits.
        arm_msg = Float64MultiArray(data=[0, 0, 0, 0])
        self.publisher.publish(arm_msg)


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('keyboardControl')

    target = rospy.get_param("~target_device", 'simulation')
    repeat = rospy.get_param("~repeat_rate", 0.0) # ~ : private param
    key_timeout = rospy.get_param("~key_timeout", 0.5)

    pub_thread = PublishThread(repeat, target)

    j1 = 0.0
    j2 = 0.0
    j3 = 0.0
    j4 = 0.0
    
    step = 0.1 # rad

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(j1, j2, j3, j4)

        print(msg)
        # print(vels(speed,turn))
        while(1):
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                j1 += moveBindings[key][0]*step
                j2 = moveBindings[key][1]*step
                j3 = moveBindings[key][2]*step
                j4 = moveBindings[key][3]*step
            elif key in stepBindings.keys():
                step += stepBindings[key]
            else:
                # Skip updating output topic if key timeout and robot already
                # stopped.
                if key == '' and j1 == 0.0 and j2 == 0.0 and j3 == 0.0 and j4 == 0.0:
                    continue
                # j1 = 0.0
                # j2 = 0.0
                # j3 = 0.0
                # j4 = 0.0
                if (key == '\x03'): # ctrl+c
                    break

            pub_thread.update(j1, j2, j3, j4)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)