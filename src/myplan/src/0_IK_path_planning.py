#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import atan2, acos, asin, sqrt, sin, cos, pi
from moveit_commander.conversions import pose_to_list

limit = [[-1.2, 1.2], [-2.0, 2.0], [-1.67, 1.67], [-1.57, 1.57]] # joint limitation (rad)
l0 = 0.0600
l1 = 0.0820
l2 = 0.1320
l3 = 0.1664
l4 = 0.048
d4 = 0.004

def all_close(goal, actual, tolerance):

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "ldsc_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()
    # print "============ Planning frame: %s" % planning_frame

    # move_group.set_workspace([-0.2984,-0.2984,0.0,0.2984,0.2984,0.4404])

    group_names = robot.get_group_names()


    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    
    self.group_names = group_names

    joint_angles = move_group.get_current_joint_values()
    self.joint_angles = joint_angles

  def go_to_joint_state(self):
    
    move_group = self.move_group
    joint_angles = self.joint_angles

    joint_goal = move_group.get_current_joint_values()

    joint_goal[0] = joint_angles[0]
    joint_goal[1] = joint_angles[1]
    joint_goal[2] = joint_angles[2]
    joint_goal[3] = joint_angles[3]

    move_group.go(joint_goal, wait=True)

    move_group.stop()

    current_joints = move_group.get_current_joint_values()
    current_pose = self.move_group.get_current_pose('link5').pose
    print "current pose:"
    print current_pose.position 
    return all_close(joint_goal, current_joints, 0.01)


def Your_IK(x,y,z): 
  global l0,l1,l2,l3,l4,d4,limit
  
  '''
  Write your code here!
  x,y,z is in world frame (the same as link0 frame)
  The end-effector should parallel to the ground.
  '''
  # j1 determin the arm x orientation on x-y plane
  j1 = 0.0
  if x < 0.0 :
    j1 = atan2(-y, -x)
  else :
    j1 = atan2(y,x)

  # remap arm x-z plan (profile of arm) -> x become x coordinate of joint 1
  # project origin x, y, z to new frame
  '''
  Z
  |       o j3
  |      / \ 
  |     /   \ j4     end-effector (link 5)
  | j2 o     o ---- o __ z
  |                  |
  |                  x
  |__ __ __ __ __ __ __ X_new

  j2 is at (0,0) in this new frame
  '''
  x_proj = sqrt(pow(x,2)+pow(y,2))

  # also, the end-effector need to parellal to ground
  # we first calculate joint 4 position, instead end-effector pose
  x_j4 = x_proj - l4
  z_j4 = z + d4 - (l0 + l1)
  L_img = sqrt(x_j4**2 + z_j4**2)
  beta = pi/2 - atan2(z_j4, x_j4)
  alpha = acos( (l2**2 + l3**2 - x_j4**2 - z_j4**2)/(2*l2*l3) )
  gamma = asin( l3*sin(alpha)/L_img )

  # elbow up
  j3_eu = pi - alpha
  j2_eu = beta - gamma
  j4_eu = pi/2 - (j2_eu + j3_eu)
  eu_legal = legal(j1, j2_eu, j3_eu, j4_eu)
  # elbow down
  j3_ed = alpha - pi
  j2_ed = gamma + beta
  j4_ed = pi/2 - (j2_ed + j3_ed)
  ed_legal = legal(j1, j2_ed, j3_ed, j4_ed)

  if eu_legal and ed_legal :
    # choose closet
    joint_angle = [j1, j2_eu, j3_eu, j4_eu]
  elif eu_legal and ~ed_legal :
    joint_angle = [j1, j2_eu, j3_eu, j4_eu]
  elif ~eu_legal and ed_legal :
    joint_angle = [j1, j2_ed, j3_ed, j4_ed]
  else :
    joint_angle=[0.0, 0.0, 0.0, 0.0]
    print("No solution")

  # joint_angle=[j1, j2_eu, j3_eu, j4_eu]
  print(joint_angle)
  return joint_angle

def legal(j1, j2, j3, j4):
  global limit
  if check(j1, limit[0][0], limit[0][1]) and  check(j2, limit[1][0], limit[1][1]) and check(j3, limit[2][0], limit[2][1]) and check(j4, limit[3][0], limit[3][1]):
    return True
  else:
    return False

def check(num, min, max):
  if min <= num <= max : 
    return True
  else :
    return False

def main():
  try:
    path_object = MoveGroupPythonIntefaceTutorial()
    print "ctrl + z to close"
    while not rospy.is_shutdown():
  
        try:
          x_input=float(raw_input("x:  "))
          y_input=float(raw_input("y:  "))
          z_input=float(raw_input("z:  "))

          path_object.joint_angles = Your_IK(x_input,y_input,z_input)
          '''
          You just need to solve IK of a point, path planning will automatically be taken.  
          '''
          path_object.go_to_joint_state()

        except:
          '''go back to home if weird input'''
          path_object.joint_angles = [0,-pi/2,pi/2,0]
          path_object.go_to_joint_state()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

