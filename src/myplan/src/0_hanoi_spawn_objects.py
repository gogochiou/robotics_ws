#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import atan2, acos, asin, sqrt, sin, cos, pi
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Bool
import rospkg 

'''Variable for end-effector'''
EefState = 0

'''Hanoi tower geometry'''
#You can measure these in Lab402
Tower_base = 0.0014     #Height of tower base
Tower_heitght = 0.025   #Height of each tower
Tower_overlap = 0.015   #Height of tower overlap

'''Hanoi tower position'''
#you may want to slightly change this
p_Tower_x = 0.25
p_Tower_y = 0.15 #(0.15, 0, -0,15) as lab4 shown
offset = 0.005

'''Hanoi tower mesh file path'''
rospack = rospkg.RosPack()
FILE_PATH = rospack.get_path('myplan')+ "/mesh"
MESH_FILE_PATH = [FILE_PATH +"/tower1.stl",FILE_PATH +"/tower2.stl",FILE_PATH +"/tower3.stl"]


'''Robot arm geometry'''
# l0 = 0.06;l1 = 0.082;l2 = 0.132;l3 = 0.1664;l4 = 0.048;d4 = 0.004

limit = [[-1.2, 1.2], [-2.0, 2.0], [-1.67, 1.67], [-1.57, 1.57]] # joint limitation (rad)
l0 = 0.0600
l1 = 0.0820
l2 = 0.1320
l3 = 0.1664
l4 = 0.048
d4 = 0.004

'''
Hint:
    The output of your "Hanoi-Tower-Function" can be a series of [x, y, z, eef-state], where
    1.xyz in world frame
    2.eef-state: 1 for magnet on, 0 for off
'''

'''Hanoi tower param'''
### A  B  C
###    ^     (arm direction)     
###   arm   
place_height = [Tower_base + Tower_heitght + 0.005, Tower_base + Tower_heitght*2 - Tower_overlap + offset*2, Tower_base + Tower_heitght*3 - Tower_overlap*2 + offset*3] # desire eef z position for different amount of tower in current position
path = [] # record path
cap_dict = {"A":0, "B":0, "C":0} # record capacity
pose_dict = {"A":[0.25, 0.15], "B":[0.25, 0], "C":[0.25, -0.15]} # tower base xy position


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
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
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "ldsc_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    eef_link = 'link5'
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    # EefState_publisher =  rospy.Publisher('/SetEndEffector', Bool, queue_size=10)                                        

    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame


    move_group.set_workspace([-0.2984,-0.2984,0.0,0.2984,0.2984,0.4404])


    group_names = robot.get_group_names()
    # Misc variables
    
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    # self.EefState_publisher = EefState_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
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
    print "current pose:" , current_pose.position 
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


  def attach_box(self, box_name, link_name):
    '''
    Description:
        1. Make sure the box has been added to rviz
        2. Attach a box to link_frame(usually 'link5'), and the box will move with the link_frame.
        3. Google scene.attach_box for more details
    '''
    scene = self.scene
    scene.attach_box(link_name, box_name, touch_links=[link_name])
    timeout=4
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, box_name, link_name):
    '''
    Description: 
        1. Detach a box from link_frame(usually 'link5'), and the box will not move with the link_frame.
        2. An example is shown in the main function below.
        3. Google scene.detach_box for more details
    '''
    scene = self.scene
    scene.remove_attached_object(link_name, name=box_name)
    timeout=4
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

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

  def add_mesh(self, mesh_name, mesh_pose, file_path, size_tuple): 
    '''
    Description: 
        1. Add a mesh to rviz, Moveit_planner will think of which as an obstacle.
        2. An example is shown in the main function below.
    '''
    scene = self.scene
    mesh_pose.pose.orientation.w = 0.7071081
    mesh_pose.pose.orientation.x = 0.7071081
    mesh_pose.pose.orientation.y = 0
    mesh_pose.pose.orientation.z = 0
    #deal with orientation-definition difference btw .stl and robot_urdf
    scene.add_mesh(mesh_name, mesh_pose, file_path, size=size_tuple)
    timeout=4
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def attach_mesh(self, mesh_name, link_name):
    '''
    Description: 
        1. Make sure the mesh has been added to rviz
        2. Attach a box to link_frame(usually 'link5'), and the box will move with the link_frame.
        3. An example is shown in the main function below.
    '''
    scene = self.scene
    scene.attach_mesh(link_name, mesh_name, touch_links=[link_name])
    timeout=4
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_mesh(self, mesh_name, link_name):
    '''
    Description: 
        1. Detach a box from link_frame(usually 'link5'), and the box will not move with the link_frame.
        2. An example is shown in the main function below.
    '''
    scene = self.scene
    scene.remove_attached_object(link_name, name=mesh_name)
    timeout=4
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_mesh(self, mesh_name):
    '''
    Description: 
        Remove a mesh from rviz.
    '''
    scene = self.scene
    scene.remove_world_object(mesh_name)
    ## **Note:** The object must be detached before we can remove it from the world
    # We wait for the planning scene to update.
    timeout=4
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def remove_world_object(self):
    '''
    Description: 
        Remove all objects from rviz.
    '''
    #remove all if no name specified
    self.scene.remove_world_object()
    timeout=4
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

def pub_EefState_to_arm():

    '''
    Description:
        Because moveit only plans the path, 
        you have to publish end-effector state for playing hanoi.
    '''
    global pub,rate
    pub = rospy.Publisher('/SetEndEffector', Bool, queue_size=10)
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
    pose_source.append(disk_number)

    cap_destination = cap_dict[destination]
    pose_destination = list(pose_dict[destination])
    pose_destination.append(place_height[cap_destination]) # no need to -1 !!
    pose_destination.append("drop")
    pose_destination.append(disk_number)

    path.append(pose_source)
    path.append(pose_destination)
    print("Move " + str(disk_number) + " from " + source + " -> " + destination)
    print([pose_source, pose_destination])
    cap_dict[source] = cap_source-1
    cap_dict[destination] = cap_destination+1

    

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

def demo():

  try:
    pathPlanObject = MoveGroupPythonIntefaceTutorial()  #Declare the path-planning object

    raw_input("Add a box to rviz")
    box_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    box_pose.header.frame_id = 'world'         # Put the box in 'world' frame
    box_pose.pose.orientation.w = 1.0          # Orieantion in quaterian
    box_pose.pose.position.x = 0.2             # Specify x of the box
    box_pose.pose.position.y = 0.0             # Specify y of the box
    box_pose.pose.position.z = 0.06/2          # Specify z of the box
    pathPlanObject.add_box('box_1', box_pose, (0.05, 0.05, 0.06)) #Specify box name, box pose, size in xyz
    
    raw_input('Attach a box to a link_frame')
    pathPlanObject.attach_box('box_1','link5') #Attach box1 to link5.When arms move, box1 will stick to link5 frame
    
    raw_input('Detach a box from a link_frame')
    pathPlanObject.detach_box('box_1','link5')

    raw_input('Remove box1 from rviz')
    pathPlanObject.remove_box('box_1')
    
    raw_input('Add a mesh to rviz')
    mesh_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    mesh_pose.header.frame_id = 'world'         # Put the mesh in 'world' frame
    mesh_pose.pose.position.x = 0.4            # Specify x of the mesh
    mesh_pose.pose.position.y = 0.0             # Specify y of the mesh
    mesh_pose.pose.position.z = 0.0             # Specify z of the mesh
    pathPlanObject.add_mesh('tower1', mesh_pose, MESH_FILE_PATH[0], (.00095,.00095,.00095))
    #Specify mesh name, mesh pose, factor_of_mesh_file(recommend not change)
    
    raw_input('Attach a mesh to a link_frame')
    pathPlanObject.attach_mesh('tower1', 'link5')
    #Attach tower1 to link5.When arms move, tower1 will stick to link5 frame.

    raw_input('Detach a mesh from a link_frame')
    pathPlanObject.detach_mesh('tower1', 'link5')


    raw_input('Remove mesh from rviz')
    pathPlanObject.remove_mesh('tower1')

    # raw_input('Remove all object from rviz')
    # pathPlanObject.remove_world_object()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

def main():
  
  try:
    
    pathPlanObject = MoveGroupPythonIntefaceTutorial()  #Declare the path-planning object
    raw_input("Press Enter to start ...")
    
    # '''
    plane_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    plane_pose.header.frame_id = 'world'         # Put the box in 'world' frame
    plane_pose.pose.orientation.w = 1.0          # Orieantion in quaterian
    plane_pose.pose.position.x = 0.2             # Specify x of the box
    plane_pose.pose.position.y = 0.0             # Specify y of the box
    plane_pose.pose.position.z = -0.03/2         # Specify z of the box
    pathPlanObject.add_box('plane_1', plane_pose, (0.7, 0.7, 0.03)) #Specify box name, box pose, size in xyz
    print("plane added !")

    # Setting hanoi start position
    print("Set hanoi start position - ")
    source = raw_input("From :  ")
    destination = raw_input("To :  ")
    p_Tower_y = pose_dict[source][1]

    # add hanoi tower 0:Large, 1:Midium, 2:Small
    towerL_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    towerL_pose.header.frame_id = 'world'         # Put the mesh in 'world' frame
    towerL_pose.pose.position.x = p_Tower_x       # Specify x of the mesh
    towerL_pose.pose.position.y = p_Tower_y       # Specify y of the mesh
    towerL_pose.pose.position.z = Tower_base      # Specify z of the mesh
    pathPlanObject.add_mesh('towerL', towerL_pose, MESH_FILE_PATH[0], (.00095,.00095,.00095))
    print("towerL added !")

    towerM_pose = geometry_msgs.msg.PoseStamped()                                       # Set the parameter
    towerM_pose.header.frame_id = 'world'                                               # Put the mesh in 'world' frame
    towerM_pose.pose.position.x = p_Tower_x                                             # Specify x of the mesh
    towerM_pose.pose.position.y = p_Tower_y                                             # Specify y of the mesh
    towerM_pose.pose.position.z = towerL_pose.pose.position.z + Tower_heitght - Tower_overlap + offset  # Specify z of the mesh
    pathPlanObject.add_mesh('towerM', towerM_pose, MESH_FILE_PATH[1], (.00095,.00095,.00095))
    print("towerM added !")

    towerS_pose = geometry_msgs.msg.PoseStamped()                                       # Set the parameter
    towerS_pose.header.frame_id = 'world'                                               # Put the mesh in 'world' frame
    towerS_pose.pose.position.x = p_Tower_x                                             # Specify x of the mesh
    towerS_pose.pose.position.y = p_Tower_y                                             # Specify y of the mesh
    towerS_pose.pose.position.z = towerM_pose.pose.position.z + Tower_heitght - Tower_overlap + offset  # Specify z of the mesh
    pathPlanObject.add_mesh('towerS', towerS_pose, MESH_FILE_PATH[2], (.00095,.00095,.00095))

    tower = ['towerL', 'towerM', 'towerS'] # 0 : towerL, 1 : towerM, 2 : towerS
    print("towerS added !")
    print("Adding Object Over")
    print("============")
    # '''

    print("Start hanoi calculation")
    height = 3
    auxiliary = find_missing_letter(source, destination)
    cap_dict[source] = 3
    print(cap_dict)
    hanoi(height, source, auxiliary,destination)
    print(path)
    print("End hanoi calculation")
    print "Total Step : ", len(path)/2
    print("============")

    print("Start Moving")
    while len(path) > 0:
      x = path[0][0]
      y = path[0][1]
      z = path[0][2]

      pathPlanObject.joint_angles = Your_IK(x , y , z)
      pathPlanObject.go_to_joint_state()

      if path[0][3] == "suck":
        pathPlanObject.attach_box(tower[path[0][4]],'link5')
      elif path[0][3] == "drop":
        pathPlanObject.detach_box(tower[path[0][4]],'link5')

      path.pop(0)
    
    # Go back home position
    pathPlanObject.joint_angles = [0,-pi/2,pi/2,0]
    pathPlanObject.go_to_joint_state()
    print("End Moving")

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  # demo() ## demo used
  main()

