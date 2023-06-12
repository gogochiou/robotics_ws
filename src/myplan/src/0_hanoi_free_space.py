#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Bool
from math import atan2, acos, asin, sqrt, sin, cos, pi

'''Variable for end-effector'''
EefState = 0
pub_EefState = rospy.Publisher('/SetEndEffector', Bool, queue_size=10)

'''Hanoi tower geometry'''
#You can measure these in Lab402
Tower_base = 0.0014     #Height of tower base
Tower_heitght = 0.025   #Height of each tower
Tower_overlap = 0.015   #Height of tower overlap

'''Hanoi tower position'''
#You may want to slightly change this
p_Tower_x = 0.25
p_Tower_y = 0.15 #(0.15, 0, -0,15) as lab4 shown

'''Robot arm geometry'''
limit = [[-1.2, 1.2], [-2.0, 2.0], [-1.67, 1.67], [-1.57, 1.57]] # joint limitation (rad)
l0 = 0.0600
l1 = 0.0820
l2 = 0.1320
l3 = 0.1664
l4 = 0.048
d4 = 0.004

'''
Hanoi tower param - 
    @ Modify parameter : place_height, pose_dict, offset, via_point_offset 
'''
### A  B  C
###    ^     (arm direction)     
###   arm
offset = 0.005   
place_height = [Tower_base + Tower_heitght + 0.005, Tower_base + Tower_heitght*2 - Tower_overlap + offset*2, Tower_base + Tower_heitght*3 - Tower_overlap*2 + offset*3] # desire eef z position for different amount of tower in current position
path = [] # record path
cap_dict = {"A":0, "B":0, "C":0} # record capacity
pose_dict = {"A":[0.25, 0.15], "B":[0.25, 0], "C":[0.25, -0.15]} # tower base xy position
via_point_offset = 0.01

'''
Hint:
    The output of your "Hanoi-Tower-Function" can be a series of [x, y, z, eef-state], where
    1.xyz in world frame
    2.eef-state: 1 for magnet on, 0 for off
'''

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
    group_names = robot.get_group_names()
    self.box_name = ''
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

    # rospy.sleep(10.)
    return all_close(joint_goal, current_joints, 0.01)

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    
    box_name = self.box_name
    scene = self.scene
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      is_known = box_name in scene.get_known_object_names()
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False

  def add_box(self, box_name , box_pose, size_tuple):  
    '''
    Description: 
        1. Add a box to rviz, Moveit_planner will think of which as an obstacle.
        2. An example is shown in the main function below.
        3. Google scene.add_box for more details
    '''
    scene = self.scene
    scene.add_box(box_name, box_pose, size=size_tuple)
    timeout=4
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)
  
  def remove_box(self, box_name):
    '''
    Description: 
        Remove a box from rviz.
    '''
    scene = self.scene
    scene.remove_world_object(box_name)
    ## **Note:** The object must be detached before we can remove it from the world
    timeout=4
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

def pub_EefState_to_arm():
    '''
    Description:
        Because moveit only plans the path, 
        you have to publish end-effector state for playing hanoi.
    '''
    global pub_EefState,rate
    pub_EefState = rospy.Publisher('/SetEndEffector', Bool, queue_size=10)
    rate = rospy.Rate(100) # 100hz

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

def convert_disk_number(disk_number):
    return 3 - disk_number # 3 is total number of disk

def set_hanoi_path(source,destination,num):
    global cap_dict, path, place_height, pose_dict
    pose_source = []
    pose_destination = []

    disk_number = convert_disk_number(num)

    cap_source = cap_dict[source]
    pose_source = list(pose_dict[source])  # without list(), pose_source wil be same as pose_dict[source] (memory position is same also) 
    pose_source.append(place_height[cap_source-1]) # need to -1 !!
    pose_source.append("suck")
    pose_source.append(source)

    cap_destination = cap_dict[destination]
    pose_destination = list(pose_dict[destination])
    pose_destination.append(place_height[cap_destination])
    pose_destination.append("drop")
    pose_destination.append(destination)

    print("Move " + str(disk_number) + " from " + source + " -> " + destination)
    print([pose_source, pose_destination])
    
    # record current cap amount
    pose_source.append(cap_dict.copy()) 
    cap_dict[source] = cap_source-1
    cap_dict[destination] = cap_destination+1
    pose_destination.append(cap_dict.copy())

    path.append(pose_source)
    path.append(pose_destination)

def hanoi(n, source, auxiliary, destination):
    if n == 1:
        set_hanoi_path(source,destination,n)
    else:
        hanoi(n-1, source, destination, auxiliary)
        set_hanoi_path(source,destination,n) 
        # hanoi(1, source, auxiliary, destination) 
        hanoi(n-1, auxiliary, source, destination)

def find_missing_letter(first_letter, second_letter):
    all_letters = {'A', 'B', 'C'}
    missing_letter = all_letters - {first_letter, second_letter}
    return missing_letter.pop()

def modify_path():
    global cap_dict, path, place_height, pose_dict, via_point_offset
    print("Modify path start!")
    add_pose = 0
    L = len(path)
    for i in range(L-1):
      # path will add some pose each for loop, ii is origin pose index in origin path list
      print(i)
      ii = i + add_pose

      # add via point -> Z offset of source
      print(path[ii])
      pose_uplift = list(path[ii])
      pose_uplift[2] = pose_uplift[2] + via_point_offset
      pose_uplift[3] = "keep"

      # add via point -> Z offset of destination
      pose_downlift = list(path[ii+1])
      pose_downlift[2] = pose_downlift[2] + via_point_offset
      pose_downlift[3] = "keep"

      # add via point -> B have tower, we need to avoid collision
      auxiliary = find_missing_letter(path[ii][4], path[ii+1][4])
      cur_cap = path[ii][5]
      pose_via = []
      if (auxiliary == "B") and (cur_cap["B"] != 0) and (path[ii][3]=="suck"):
        print("add via point")
        pose_via = list(pose_dict["B"])
        pose_via.append(place_height[cur_cap["B"]] + via_point_offset) 
        pose_via.append("keep")
        pose_via.append("via")
        pose_via.append(cur_cap)
        # add pose into path list
        path.insert(ii+1, pose_uplift)
        path.insert(ii+2, pose_via)
        path.insert(ii+3, pose_downlift)
        add_pose = add_pose + 3
      else:
        print("add offset point")
        path.insert(ii+1, pose_uplift)
        path.insert(ii+2, pose_downlift)
        add_pose = add_pose + 2

def main():
  global pub_EefState, EefState
  try:
    path_object = MoveGroupPythonIntefaceTutorial()
    print "ctrl + z to close"

    raw_input("Press Enter to start ...")
          
    # plane_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    # plane_pose.header.frame_id = 'world'         # Put the box in 'world' frame
    # plane_pose.pose.orientation.w = 1.0          # Orieantion in quaterian
    # plane_pose.pose.position.x = 0.2             # Specify x of the box
    # plane_pose.pose.position.y = 0.0             # Specify y of the box
    # plane_pose.pose.position.z = -0.03/2         # Specify z of the box
    # path_object.add_box('plane_1', plane_pose, (0.7, 0.7, 0.03)) #Specify box name, box pose, size in xyz
    # print("plane added !")
    # print("Adding Object Over")
    # print("============")

    while not rospy.is_shutdown():
  
        try:

          '''
          Modify this into a list of [x, y, z, 1 or 0]

          x_input=float(raw_input("x:  "))
          y_input=float(raw_input("y:  "))
          z_input=float(raw_input("z:  "))
          1 for end-effector on;0 for off
          
          
          path_object.joint_angles = Your_IK(x,y,z)
          path_object.go_to_joint_state() #path will automatically be published by moveit
          EefState = 0
          pub_EefState.publish(EefState)  #publish end-effector state


          '''
          # Setting hanoi start position
          print("Set hanoi start position - ")
          source = raw_input("From :  ")
          destination = raw_input("To :  ")

          # tower = ['towerL', 'towerM', 'towerS'] # 0 : towerL, 1 : towerM, 2 : towerS
          print("Start hanoi calculation")
          height = 3
          auxiliary = find_missing_letter(source, destination)
          cap_dict[source] = 3
          print(cap_dict)
          hanoi(height, source, auxiliary,destination)
          modify_path() # add redundant pose to avoid collision
          # print(path)
          print("End hanoi calculation")
          print "Total Step : ", len(path)/2
          print("============")

          # tower = ['towerL', 'towerM', 'towerS'] # 0 : towerL, 1 : towerM, 2 : towerS -> for path[0][4]

          print("Start Moving")
          while len(path) > 0:
            x = path[0][0]
            y = path[0][1]
            z = path[0][2]

            path_object.joint_angles = Your_IK(x , y , z)
            path_object.go_to_joint_state()
            print('goal reach')
            print(path[0][3])

            if path[0][3] == "suck":
              EefState = 1
              pub_EefState.publish(EefState)  #publish end-effector state
            elif path[0][3] == "drop":
              EefState = 0
              pub_EefState.publish(EefState)  #publish end-effector state
            else:
              pub_EefState.publish(EefState)
              
            print('Eef : ', EefState)

            path.pop(0)
          
          # Go back home position
          path_object.joint_angles = [0,-pi/2,pi/2,0]
          path_object.go_to_joint_state()
          pub_EefState.publish(0)
          print("End Moving")
          print("============")
          
        except:
          '''Go back to home if weird input'''
          path_object.joint_angles = [0,-pi/2,pi/2,0]
          path_object.go_to_joint_state()
          pub_EefState.publish(0)
          print("Back to home")
          print("============")

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

