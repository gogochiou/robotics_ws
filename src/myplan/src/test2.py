#!/usr/bin/env python


## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import atan2, acos, asin, sqrt, sin, cos, pi
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Bool

import rospkg 
rospack = rospkg.RosPack()


file_path = rospack.get_path('myplan')+ "/mesh"
mesh_file_path = [file_path +"/tower1.stl",file_path +"/tower2.stl",file_path +"/tower3.stl"]


MODE = 0#MODE0 position, MODE1 demo

EefState = 0
TB = 0.00233#Tower base
Tower = 0.025
TU = 0.015##Tower upon#height - depth of hole
# TB = 0#Tower base
# Tower = 0.046
# TU = 0.01##Tower upon#height - depth of hole

p_Tower_x = 0.25
p_Tower_z = [TB+Tower, TB+Tower+TU, TB+Tower+2*TU, TB+Tower+5*TU]

a1 = [p_Tower_x, 0.15, p_Tower_z[0]]
a2 = [p_Tower_x, 0.15, p_Tower_z[1]]
a3 = [p_Tower_x, 0.15, p_Tower_z[2]]
a4 = [p_Tower_x, 0.15, p_Tower_z[3]]

b1 = [p_Tower_x, 0, p_Tower_z[0]]
b2 = [p_Tower_x, 0, p_Tower_z[1]]
b3 = [p_Tower_x, 0, p_Tower_z[2]]
b4 = [p_Tower_x, 0, p_Tower_z[3]]

c1 = [p_Tower_x, -0.15, p_Tower_z[0]]
c2 = [p_Tower_x, -0.15, p_Tower_z[1]]
c3 = [p_Tower_x, -0.15, p_Tower_z[2]]
c4 = [p_Tower_x, -0.15, p_Tower_z[3]]

# home = ['a', 'a', 'a']
fixed_path = [a3+[1]+['tower3'], a4+[1]+['tower3'], c4+[1]+['tower3'], c1+[0]+['tower3'], c4+[0]+['tower3'],\
    a4+[0]+['tower2'], a2+[1]+['tower2'], a4+[1]+['tower2'], b4+[1]+['tower2'], b1+[0]+['tower2'], b4+[0]+['tower2'],\
    c4+[0]+['tower3'], c1+[1]+['tower3'], c4+[1]+['tower3'], b4+[1]+['tower3'], b2+[0]+['tower3'], b4+[0]+['tower3'],\
    a4+[0]+['tower1'], a1+[1]+['tower1'], a4+[1]+['tower1'], c4+[1]+['tower1'], c1+[0]+['tower1'], c4+[0]+['tower1'],\
    b4+[0]+['tower3'], b2+[1]+['tower3'], b4+[1]+['tower3'], a4+[1]+['tower3'] ,a1+[0]+['tower3'], a4+[0]+['tower3'],\
    b4+[0]+['tower2'], b1+[1]+['tower2'], b4+[1]+['tower2'], c4+[1]+['tower2'], c2+[0]+['tower2'], c4+[0]+['tower2'],\
    a4+[0]+['tower3'], a1+[1]+['tower3'], a4+[1]+['tower3'], c4+[1]+['tower3'], c3+[0]+['tower3']]

# fixed_path = [a3+[1]+['tower3'], c1+[0]+['tower3'],\
#     a2+[1]+['tower2'], b1+[0]+['tower2'],\
#     c1+[1]+['tower3'], b2+[0]+['tower3'],\
#     a1+[1]+['tower1'], c1+[0]+['tower1'],\
#     b2+[1]+['tower3'], a1+[0]+['tower3'],\
#     b1+[1]+['tower2'], c2+[0]+['tower2'],\
#     a1+[1]+['tower3'], c3+[0]+['tower3']
#     ]


l0 = 0.06
l1 = 0.082
l2 = 0.132#0.132
l3 = 0.1664#0.1664
l4 = 0.048
d4 = 0.004#0.004


waypoint = []
waypoint1 = []
point_list = [a1, a2, a3, b1, b2, b3, c1, c2, c3]
point_list1 = [a4, b4, c4] # switching point

def hanoi(n, a, b, c):
    
    if n == 1:
        waypoint.append([a, c])
    else:
        hanoi(n - 1, a, c, b)
        hanoi(1, a, b, c)
        hanoi(n - 1, b, a, c)

def hanoi_waypoint():
    global waypoint, waypoint1
    waypoint = []
    waypoint1 = []
    a = str(raw_input("start A , B, or C: "))
    b = str(raw_input("pass  A , B, or C: "))
    c = str(raw_input("stop  A , B, or C: "))
    hanoi(3, a, b, c)

    count1 = 0;count2 = 0;count3 = 0
    
    if a == "A":
        count1 = 3
    elif a == "B":
        count2 = 3
    elif a =="C":
        count3 = 3

    for i in waypoint:
        if i[0] == 'A':
            waypoint1.append(point_list1[0]+[0])
            waypoint1.append(point_list[count1 - 1]+[1])
            waypoint1.append(point_list1[0]+[1])
            count1 = count1 - 1
        elif i[0] == 'B':
            waypoint1.append(point_list1[1]+[0])
            waypoint1.append(point_list[count2 - 1 + 3*1]+[1])
            waypoint1.append(point_list1[1]+[1])
            count2 = count2 - 1
        elif i[0] == 'C':
            waypoint1.append(point_list1[2]+[0])
            waypoint1.append(point_list[count3 - 1 + 3*2]+[1])
            waypoint1.append(point_list1[2]+[1])
            count3 = count3 - 1

        if i[1] == 'A':
            waypoint1.append(point_list1[0]+[1])
            waypoint1.append(point_list[count1]+[0])
            waypoint1.append(point_list1[0]+[0])
            count1 = count1 + 1
        elif i[1] == 'B':
            waypoint1.append(point_list1[1]+[1])
            waypoint1.append(point_list[count2 + 3*1]+[0])
            waypoint1.append(point_list1[1]+[0])
            count2 = count2 + 1
        elif i[1] == 'C':
            waypoint1.append(point_list1[2]+[1])
            waypoint1.append(point_list[count3 + 3*2]+[0])
            waypoint1.append(point_list1[2]+[0])
            count3 = count3 + 1
    print(waypoint1)

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
    EefState_publisher =  rospy.Publisher('/SetEndEffector', Bool, queue_size=10)                                        

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
    self.EefState_publisher = EefState_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

    j = move_group.get_current_joint_values()
    self.j = j

  def position_mode(self):
    # j = self.j
    while not rospy.is_shutdown():
    
        print "============ key in xyz to execute a movement using a pose goal ..."
        try:
            xi=float(raw_input("x:  "))#general use
            yi=float(raw_input("y:  "))#general use
            zi=float(raw_input("z:  "))#general use

            buffer = Simplest_IK(xi,yi,zi)
            if buffer != None:
                self.j = buffer 
                self.go_to_joint_state()
            else:
                continue
        except:
            self.j = [0,-pi/2,pi/2,0] 
            self.go_to_joint_state()
            change_MODE = int(raw_input("1 for change mode, 0 for continue:"))
            if change_MODE ==1:
                break
            elif change_MODE ==0:
                continue



  def go_to_joint_state(self):
    
    move_group = self.move_group
    j = self.j

    joint_goal = move_group.get_current_joint_values()

    joint_goal[0] = j[0]
    joint_goal[1] = j[1]
    joint_goal[2] = j[2]
    joint_goal[3] = j[3]

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    # print "current joints:" , current_joints
    current_pose = self.move_group.get_current_pose('link5').pose
    print "current pose:" , current_pose.position 
    return all_close(joint_goal, current_joints, 0.01)

  def now_pose(self):
    print "current pose:", self.move_group.get_current_pose()

  def get_constraints(self):
    print "path constraints:", self.move_group.get_path_constraints()

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Received
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def add_plane(self):
    scene = self.scene
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose.orientation.w = 1.0
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0
    scene.add_plane('Ground', pose, normal = (0, 0, 1), offset = 0)

  def add_box(self, box_name, box_pose, size_tuple, timeout=4):   
    # box_name = self.box_name
    scene = self.scene
    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = link_name
    # box_pose.pose.orientation.w = 1.0
    # box_pose.pose.position.x = 0.025 
    # box_pose.pose.position.y = 0
    # box_pose.pose.position.z = 0
    # box_name = "box"
    scene.add_box(box_name, box_pose, size=size_tuple)
    # scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))

    # self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, box_name, link_name, timeout=4):
    # robot = self.robot
    scene = self.scene
    # grasping_group = 'ldsc_arm'
    # touch_links = robot.get_link_names(group=grasping_group)#ignore collision check

    scene.attach_box(link_name, box_name, touch_links=[link_name])
    # scene.attach_box(link_name, box_name, touch_links=touch_links)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, box_name, link_name, timeout=4):
    scene = self.scene

    scene.remove_attached_object(link_name, name=box_name)
    
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, box_name, timeout=4):
    scene = self.scene
    scene.remove_world_object(box_name)
    ## **Note:** The object must be detached before we can remove it from the world
    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def add_mesh(self, mesh_name, mesh_pose, file_path, size_tuple, timeout=4): 
    scene = self.scene

    mesh_pose.pose.orientation.w = 0.7071081
    mesh_pose.pose.orientation.x = 0.7071081
    mesh_pose.pose.orientation.y = 0
    mesh_pose.pose.orientation.z = 0
    #deal with orientation-definition difference btw .stl and robot_urdf


    scene.add_mesh(mesh_name, mesh_pose, file_path, size=size_tuple)
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_mesh(self, mesh_name, link_name, timeout=4):
    scene = self.scene

    scene.attach_mesh(link_name, mesh_name, touch_links=[link_name])
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_mesh(self, mesh_name, link_name, timeout=4):
    scene = self.scene
    scene.remove_attached_object(link_name, name=mesh_name)
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_mesh(self, mesh_name, timeout=4):
    scene = self.scene
    scene.remove_world_object(mesh_name)
    ## **Note:** The object must be detached before we can remove it from the world
    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def remove_world_object(self, timeout=4):
    #remove all if no name specified
    self.scene.remove_world_object()
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def demo_mode(self):
    global waypoint1, fixed_path
    mode = int(raw_input("1 for any, 2 for fixed: "))
    q = []
    if mode == 1:
        q = waypoint1
    elif mode == 2 :
        q = fixed_path
    else:
        print "error 405"
    pub = self.EefState_publisher
    i = 0
    # while not rospy.is_shutdown(): 
    while True: 
      # raw_input("press enter to start demo mode")
      if i <(len(q)):
        try:
          xi = q[i][0];yi = q[i][1];zi = q[i][2]
          
          buffer = Simplest_IK(xi,yi,zi)
          if buffer != None:
            self.j = buffer 

            self.go_to_joint_state()

            EefState  = q[i][3]
            pub.publish(EefState)

            if EefState == 1:
              self.attach_mesh(q[i][4], 'link5', timeout=4)
            elif EefState == 0:
              self.detach_mesh(q[i][4], 'link5', timeout=4)
            print i
          else:
            continue
          i = i+1##demo
        except:
          self.j = [0,-pi/2,pi/2,0] 
          self.go_to_joint_state()
          pub.publish(0)##demo
          i = 0##demo
          change_MODE = int(raw_input("1 for change mode, 0 for continue:"))
          if change_MODE ==1:
            break
          elif change_MODE ==0:
            continue
      
      else:##demo
        self.j = [0,-pi/2,pi/2,0] 
        self.go_to_joint_state()
        pub.publish(0)##demo
        i = 0##demo
        break

def Simplest_IK(x,y,z):

  '''The input here is end-effector frame(virtual link5) [0 0 0]' in world frame,
  but the IK solves for the link4 frame,
  so first we make some transformation'''
  global l0,l1,l2,l3,l4,d4

  Mag2Eef = atan2(y,x)
#   if abs(Mag2Eef) <= pi/2:##xy (+,+) or (+,-) : x > 0 
    
  x = x - l4*cos(Mag2Eef)## magnet to end
  y = y - l4*sin(Mag2Eef)
  z = z + d4

#   else:##xy (-,-) or (-,+) : x < 0 
#     x = x + l4*cos(Mag2Eef)## magnet to end
#     y = y + l4*sin(Mag2Eef)
#     z = z - d4
  
  
  joint_angle = [0,0,0,0]
  
  L = sqrt(x**2+y**2+(z-l0-l1)**2)

  cos_j3 = (L**2 - l2**2 - l3**2)/(2*l2*l3)

  if abs(cos_j3) > 1: ##arm can't reach
    SOLUTION_FOUND =  False
  
  elif abs(cos_j3-1) < 0.0001 : ##prevent float == float, cos_j3 ==1

    '''the longest distance that the arm can reach,
    which is a sphere with r = l2+l3 centered at joint2,
    j3 has unique one solution = 0 deg
    '''
    SOLUTION_FOUND =  [atan2(sqrt(x**2 + y**2),z-l0-l1 ) , 0]
    print "max sphere"

  
  elif abs(cos_j3+1) < 0.0001:  ##prevent float == float, cos_j3 ==-1
    if x == 0 and y ==0:#because L2 != L3,but moveit will also detect collision
      SOLUTION_FOUND =  False
    else:
      SOLUTION_FOUND =  [atan2(sqrt(x**2 + y**2),z-l0-l1 ) , pi]
      print "min sphere"

  else:
    j3_two_sol = [acos(cos_j3),-acos(cos_j3)]
    a = atan2(sqrt(x**2 + y**2),z-l0-l1 )
    
    j2_two_sol = [0,0]
    for i in range(len(j2_two_sol)):
      j2_two_sol[i] = a - atan2(l3*sin(j3_two_sol[i]),l2+l3*cos(j3_two_sol[i]))

    SOLUTION_FOUND =  [j2_two_sol[0],j3_two_sol[0],j2_two_sol[1],j3_two_sol[1]]

  if SOLUTION_FOUND != False:

    if len(SOLUTION_FOUND)==2:
      joint_angle[1] = SOLUTION_FOUND[0]##joint2
      joint_angle[2] = SOLUTION_FOUND[1]##joint3

    elif len(SOLUTION_FOUND)==4:
        if abs(SOLUTION_FOUND[0]) < 2.0 and abs(SOLUTION_FOUND[1])<1.67:

          '''check if solution of j2 and j3 exceed limit''' 
          joint_angle[1] = SOLUTION_FOUND[0]##joint2
          joint_angle[2] = SOLUTION_FOUND[1]##joint3
        #   print '216'

        elif abs(SOLUTION_FOUND[2]) < 2.0 and abs(SOLUTION_FOUND[3])<1.67:

          '''check if solution of j2 and j3 exceed limit'''
          joint_angle[1] = SOLUTION_FOUND[2]##joint2
          joint_angle[2] = SOLUTION_FOUND[3]##joint3
        #   print '223'
        else:
          print "arm can''t reach"
          return

  else:
    print "arm can''t reach"
    return
  
  joint_angle[0] = atan2(y,x)
  joint_angle[3] = pi/2 - joint_angle[1] - joint_angle[2] ## 4th joint

  

  if abs(joint_angle[0]) <= pi/2:##xy (+,+) or (+,-) : x > 0 
    pass
  else:##xy (-,-) or (-,+) : x < 0 
    joint_angle[0] = joint_angle[0] + pi
    joint_angle[1] = - joint_angle[1]
    joint_angle[2] = - joint_angle[2]
    joint_angle[3] = - joint_angle[3]


  joint_angle = map_into_pi_pi(joint_angle)
  

  if not structure_limit(joint_angle):
    print "over structure limit"
    print [1.2, 2, 1.67, pi/2]
    print joint_angle
    return 

  
  return joint_angle

def map_into_pi_pi(joint_list):

  temp_list = [0,0,0,0]## be careful: some issues over changing list value in Python 
  for i in range(len(joint_list)):
      while joint_list[i] > pi:
        joint_list[i] = joint_list[i] - 2*pi
      while joint_list[i] < -pi:
        joint_list[i] = joint_list[i] + 2*pi
      temp_list[i] = joint_list[i]
  return temp_list

def structure_limit(joint_list):
  '''real arm joint limit'''
  limit = [1.2, 2, 1.67, pi/2]
  for i in range(len(joint_list)):
    if abs(joint_list[i]) > limit[i] :
      return False
  
  return True
    

def main():
  try:
    pathPlanObject = MoveGroupPythonIntefaceTutorial()
    while(True):
      try:
        MODE = int(raw_input("MODE: 0 position, 1 demo:"))
      except:
        continue
      if MODE == 0:
        pathPlanObject.position_mode()
      elif MODE == 1:

        # hanoi_waypoint()
        raw_input("spawn wall and tower")
        # pathPlanObject.add_box()
        plate_pose = geometry_msgs.msg.PoseStamped()
        plate_pose.header.frame_id = 'world'
        plate_pose.pose.orientation.w = 1.0
        plate_pose.pose.position.x = 0.155
        plate_pose.pose.position.y = 0
        plate_pose.pose.position.z = -0.012/2
        pathPlanObject.add_box('base', plate_pose, (0.6, 0.45, 0.012))



        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = 'world'
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.25
        box_pose.pose.position.y = 0.075
        box_pose.pose.position.z = 0.4/2
        pathPlanObject.add_box('wall_1', box_pose, (0.10, 0.03, 0.4))
        box_pose.pose.position.y = -0.075
        pathPlanObject.add_box('wall_2', box_pose, (0.10, 0.03, 0.4))

        # box_pose.pose.position.x = 0.25
        # box_pose.pose.position.y = 0
        # box_pose.pose.position.z = 0.3
        # pathPlanObject.add_box('wall_3', box_pose, (0.18, 0.45, 0.05))

        

        Tow_pose = geometry_msgs.msg.PoseStamped()
        Tow_pose.header.frame_id = 'world'
        Tow_pose.pose.position.x = 0.25
        Tow_pose.pose.position.y = 0.15
        Tow_pose.pose.position.z = TB
        #originally 0.001 (inventor mm --> here m)
        #here set 0.00095 to avoid hanoi collision with each other
        pathPlanObject.add_mesh('tower1', Tow_pose, mesh_file_path[0], (.0009,.0009,.0009))
        Tow_pose.pose.position.z = TB+TU
        pathPlanObject.add_mesh('tower2', Tow_pose, mesh_file_path[1], (.0009,.0009,.0009))
        Tow_pose.pose.position.z = TB+2*TU
        pathPlanObject.add_mesh('tower3', Tow_pose, mesh_file_path[2], (.0009,.0009,.0009))
        raw_input("hanoi ok!")
        pathPlanObject.demo_mode()
        raw_input("clear world")
        pathPlanObject.remove_world_object()


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

    
# def main():
#   try:
#     pathPlanObject = MoveGroupPythonIntefaceTutorial()
    
    

#     pathPlanObject.demo_mode()

#     # pathPlanObject.position_mode()
#     # # pathPlanObject.attach_box()
#     # grasping_group = 'ldsc_arm'
#     # touch_links = pathPlanObject.robot.get_link_names(group=grasping_group)
#     # pathPlanObject.scene.attach_box('link5', 'wall_1', touch_links=touch_links)

#     # raw_input()
#     # # pathPlanObject.detach_box()
#     # pathPlanObject.scene.remove_attached_object('link5', name='wall_1')
#     # pathPlanObject.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=4)


#     raw_input()
#     # pathPlanObject.remove_box()
#     # pathPlanObject.scene.remove_world_object('wall_1')
#     # pathPlanObject.scene.remove_world_object('wall_2')
#     # pathPlanObject.scene.remove_world_object('wall_3')
#     # pathPlanObject.scene.remove_world_object('tower1')
#     # pathPlanObject.scene.remove_world_object('tower2')
#     # pathPlanObject.scene.remove_world_object('tower3')
#     pathPlanObject.remove_world_object()
    

#   except rospy.ROSInterruptException:
#     return
#   except KeyboardInterrupt:
#     return

if __name__ == '__main__':
  
  main()

