#!/usr/bin/python2.7

import rospy
from sensor_msgs.msg import JointState

import numpy as np
import modern_robotics as mr



## for plotting the ellipsoid
#import plotly as py
#import plotly.graph_objs as go
#import plotly.figure_factory as ff
#from skimage import measure
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D

import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


kp = 20 # how big the steps toward desired config. for max manipulability should be.
desired_direction = [-1, 0, 0]  # desired direction to move in space frame (cartesian)
ep = 0.035  # tolerence for difference in manipulability capacity - if bigger than this robot will move
dt = 1     # step size in null_space direction to check how much manipulability improves (multiplies to ns unit vector)






""" Functions copied from moveit tutorial to have access to movegroup functionality
    maybe lots of other functions are unnecessary """
    
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

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()
    

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    
    self.joint_goal = group.get_current_joint_values()

  def go_to_joint_state(self, goal):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = self.joint_goal
#    joint_goal[0] = 0
#    joint_goal[1] = -pi/4
#    joint_goal[2] = 0
#    joint_goal[3] = -pi/2
#    joint_goal[4] = 0
#    joint_goal[5] = pi/3
#    joint_goal[6] = 0
    joint_goal = goal


    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL

  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
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

  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_leftfinger"
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'hand'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)






















""" My code for calculating manipulability in the desired direction """


class jey_listener():
    
    def __init__(self):
        
        self.Slist = np.array([[0,  0,    0, 0,    0,  0,    0],
                               [0,  1,    0,-1,    0,  1,    0],
                               [1,  0,    1, 0,    1,  0,    1],
                               [0, -0.36, 0, 0.78, 0, -1.18, 0],
                               [0,  0,    0, 0,    0,  0,    0],
                               [0,  0,    0, 0,    0,  0,    0]])
    
        self.Blist = np.array([[0, 0,     0,  0,     0, 0,     0],
                               [0, 1,     0, -1,     0, 1,     0],
                               [1, 0,     1,  0,     1, 0,     1],
                               [0, 0.901, 0, -0.481, 0, 0.081, 0],
                               [0, 0,     0,  0,     0, 0,     0],
                               [0, 0,     0,  0,     0, 0,     0]])
    
        self.M = np.array([[1, 0, 0, 0   ],         # End-effector pos. in home config.
                           [0, 1, 0, 0   ],
                           [0, 0, 1, 1.26],
                           [0, 0, 0, 1   ]])
    
        self.thetas = np.array([0,0,0,0,0,0,0], dtype=float)
        
        self.k=0
        #self.position_data = [0,0,0,0,0,0,0]
        self.pose_now = mr.FKinSpace(self.M, self.Slist, self.thetas)
        self.pose_past = mr.FKinSpace(self.M, self.Slist, self.thetas)
        self.direction = np.array([0,0,0])
        self.ep = 0.0001
        self.pub = rospy.Publisher('Plot', geometry_msgs.msg.Accel, queue_size = 10)

        

    
    def retrieving_thetas(self, position_data):
        j=0
        
        for n in position_data:
            if np.abs(n) <= self.ep:
                self.thetas[j] = 0
            else:
                self.thetas[j] = position_data[j]   # thetalist is a list so converting it to array
                
            j=j+1
                
        print('printing theta_list: ', self.thetas)
    
    
    def manipulability(self, theta_list):

        Js_ = mr.JacobianSpace(self.Slist, theta_list)
        print("Full Jacobian in Space Frame : ", Js_)
        print("")

        Jb_ = mr.JacobianBody(self.Blist, theta_list)
        print("Full Jacobian in Body Frame: ", Jb_)
        print("")
        #Js_w = Js[[0,1,2],:]
        Js_v = Js_[[3,4,5],:]

        mu_s_v_ = np.sqrt(np.linalg.det(np.matmul(Js_v, Js_v.T)))
        U, D, Vt = np.linalg.svd(Js_v, full_matrices=False)  
        
        if np.abs(D[0]*D[1]*D[2]) < self.ep:
            if np.abs(D[0]) < self.ep:
                print('Warning: large axis == 0!')
            else:
                if np.abs(D[1]) < self.ep: 
                    D[1]=0
                    D[2]=0
                   # means robot can move only in one direction md
                    print('Plot a line',D)
                    if np.linalg.norm(self.direction) != 0:
                        dd_dot = 1-np.abs( np.dot(self.direction, U[:,0])/(np.linalg.norm(self.direction)*np.linalg.norm(U[:,0])) )
                        if dd_dot < self.ep:
                            md_ = D[0]
                            print('Manipulability capacity in the desired direction is = ', md_)
                        else:
                            md_=0
                            print('Movement in the desired direction is not possible!')
                    else:
                        md_=0
                        print('It is desired Not to move!')
                    
                else:
                    D[2]=0
                    # means D[2]= 0, means 2D ellipsoid
                    print('Plot a 2D ellipse',D)
                    
                    # Manipulability in the desired direction
                    if np.linalg.norm(self.direction) != 0: 
                        normal_vec = np.cross(U[:,0], U[:,1])
                        n_d_dot = np.dot(normal_vec, self.direction)
                        if np.abs(n_d_dot) < self.ep:
                            # Movement is Possible
                            p1 = (self.direction[1]*U[0][0] - self.direction[0]*U[1][0]) / (self.direction[1]*U[0][1] - self.direction[0]*U[1][1])
                            p2 = (D[1]/D[0])*( (U[1][1]*U[0][0] - U[0][1]*U[1][0]) / (U[0][1]*U[1][0] - U[1][1]*U[0][0]) )
                            phi = np.arctan2(p1/p2)
                            md_ = np.abs( (D[0]*np.cos(phi)*U[0][0] + D[1]*np.sin(phi)*U[0][1])/self.direction[0] )
                            print('Manipulability capacity in the desired direction is = ', md_)

                        else:
                            md_=0
                            print('Movement in the desired direction is not possible!')
                        
                    else:
                        md_=0
                        print('It is desired Not to move!')
                    
    
    
                
        else: # means 3D ellipsoid
            print('Plot a 3D ellipsoid',D)
            
            # Manipulability in the desired direction (in case desired direction is not zero)
            if np.linalg.norm(self.direction) != 0:
                coeff_0 = np.square((U[0][0]*self.direction[0] + U[1][0]*self.direction[1] + U[2][0]*self.direction[2])/D[0])
                coeff_1 = np.square((U[0][1]*self.direction[0] + U[1][1]*self.direction[1] + U[2][1]*self.direction[2])/D[1])
                coeff_2 = np.square((U[0][2]*self.direction[0] + U[1][2]*self.direction[1] + U[2][2]*self.direction[2])/D[2])
                c = coeff_0 + coeff_1 + coeff_2
                md_ = np.sqrt(1/c)
                print('Manipulability capacity in the desired direction is = ', md_)
            else:
                md_=0
                print('It is desired Not to move!')
                
                
        return mu_s_v_, md_, Js_
                
 
    
    def moving_direction(self):  
        
        if self.k==0:
            self.k+=1
            self.pose_now = np.asarray(mr.FKinSpace(self.M, self.Slist, self.thetas), dtype=float)
            self.pose_past = self.pose_now
            dp = np.array([0,0,0])
            
        else:
            self.pose_past = self.pose_now
            self.pose_now = np.asarray(mr.FKinSpace(self.M, self.Slist, self.thetas), dtype=float)
            #dp = np.array([ self.pose_now[0][3] - self.pose_past[0][3], self.pose_now[1][3] - self.pose_past[1][3], self.pose_now[2][3] - self.pose_past[2][3] ])
            dp = self.pose_now[0:3,3]-self.pose_past[0:3,3]
            
            if np.linalg.norm(dp) != 0:
                self.direction = dp/np.linalg.norm(dp)
            else:
                self.direction = np.array([0,0,0])
        print("Moving direction is= ", self.direction)
        
 
       
    def nullspace(self,A, atol=1e-13, rtol=0):
        
        """
        ns : ndarray
        If `A` is an array with shape (m, k), then `ns` will be an array
        with shape (k, n), where n is the estimated dimension of the
        nullspace of `A`.  The columns of `ns` are a basis for the
        nullspace; each element in numpy.dot(A, ns) will be approximately
        zero. 
        """
        A = np.atleast_2d(A)
        u, s, vh = np.linalg.svd(A)
        tol = max(atol, rtol * s[0])
        nnz = (s >= tol).sum()
        ns = vh[nnz:].conj().T
        ns[ns<=self.ep]=0
        
        return ns
        
  
    
    def ns_direction(self, ns_, md_, dt_):
        ns_ = np.asarray(ns_)
        ns_shape = np.shape(ns_)
        thetas_d1_ = np.zeros(ns_shape)
        thetas_d2_ = np.zeros(ns_shape)
        self.thetas.resize(7,1)
 
        # np.ones(ns_shape) is used to broadcast thetas to a matrix of shape ns 
        # to examin all the directions in null_space (more than one direction at singularities)
 
        thetas_d1_ = self.thetas*np.ones(ns_shape) + dt_*ns_    # all in + direction 
        thetas_d2_ = self.thetas*np.ones(ns_shape) - dt_*ns_    # all in - direction
         
                
        l = 0
        md_best = md_  # current manipulability capacity in the desired direction
        
        while l < ns_shape[1]:
            musv_d1, md_d1, Js_d1 = self.manipulability(thetas_d1_[:,l])
            musv_d2, md_d2, Js_d2 = self.manipulability(thetas_d2_[:,l])
            
        
            if (md_d1 - md_) > (md_d2 - md_) and (md_d1 - md_) >ep :
                if md_d1 > md_best:  
                    md_best = md_d1
                    to_go = 1 #Positive Direction
                    theta_to_go = ns_[:,l]*to_go
                    d_md = md_d1 - md_
                    
                    
            
            elif (md_d1 - md_) < (md_d2 - md_) and (md_d2 - md_) >ep :
                if md_d2 > md_best: 
                    md_best = md_d2
                    to_go = -1 #Negative Direction
                    theta_to_go = ns_[:,l]*to_go
                    d_md = md_d2 - md_
                    
            elif (md_d1 - md_) == (md_d2 - md_) and (md_d1 - md_)>ep :
                # this case both directions are ok to go - we choose the first one
                if md_d1 > md_best: 
                    md_best = md_d1
                    to_go = 1 #Positive Direction
                    theta_to_go = ns_[:,l]*to_go
                    d_md = md_d1 - md_
                
                
            else:
                if md_best <= md_:
                    to_go = 0 #Does not matter!
                    theta_to_go = ns_[:,l]*to_go  # null_space theta --> later adds to current joint angles
                    d_md = 0
            
            theta_to_go2 = np.zeros((7,1))
             
#            theta_to_go2 = theta_to_go2 + theta_to_go.T
            
            theta_to_go2[0][0] = theta_to_go[0]
            theta_to_go2[1][0] = theta_to_go[1]
            theta_to_go2[2][0] = theta_to_go[2]
            theta_to_go2[3][0] = theta_to_go[3]
            theta_to_go2[4][0] = theta_to_go[4]
            theta_to_go2[5][0] = theta_to_go[5]
            theta_to_go2[6][0] = theta_to_go[6]
            #print('theta_to_go2 is = ', theta_to_go2)
            l+=1
  
        return to_go, theta_to_go2, d_md, md_best
    
    
    
    
    
    

  
 
    def joint_states_callback(self,data_):
        
        #rospy.loginfo(rospy.get_caller_id() + "  I heard %s ", data_.position)
        
        position_data = data_.position
        self.retrieving_thetas(position_data)
        #self.moving_direction()
        self.direction = desired_direction

        mu_s_v, md, Js = self.manipulability(self.thetas)
        # mu_s_v:  is total manipulability capacity
        # md:      is manipulability capacity in the desired direction
        #Js:       is space jacobian

        ns_ = self.nullspace(Js)   # directions available in nullspace
        print('Null_space is = ', ns_)
        
        to_go, theta_to_go, d_md, md_best = self.ns_direction(ns_, md, dt)
        # to_go: -1 or +1 indicating direction
        # theta_to_go: joint_goal in null space (unit vector) - should add to current joint values to form a joint_goal  
        # d_md: difference in manipulability capacity if we take one step in the null_space direction 
        #       used for defining the goal - makes the steps adaptive, so when difference is less we take small steps
        # md_best: max manipulability we can reach by taking a step 
        
        print('It is better to move in (in null_space) = ', to_go )
        assert(self.thetas.shape == (7,1))
        assert(theta_to_go.shape == (7,1))
 
        if to_go != 0:
            
            #goal = (self.thetas + kp*d_md*theta_to_go).reshape((1,7)).tolist()[0]    
            goal = (self.thetas + theta_to_go).reshape((1,7)).tolist()[0]

            self.move.go_to_joint_state(goal)
        
            msg = geometry_msgs.msg.Accel()
            msg.linear.x = md
            msg.linear.y = md_best
            msg.linear.z = 0
            
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0
            self.pub.publish(msg) # use this to plot the convergence of md to md_best - rqt_multiplot
    

    def run(self, move_obj):
        
        self.move = move_obj        
        
        self.goal_joints = [0,0,0,0,0,0,0]  # initializing the joints_goal
        goal_joints = self.goal_joints      
        
        # putting the robot in different config. than home config.
        goal_joints[0] = 0
        goal_joints[1] = -pi/4
        goal_joints[2] = 0
        goal_joints[3] = -pi/2
        goal_joints[4] = 0
        goal_joints[5] = pi/3
        goal_joints[6] = 0
        
        
        
        move_obj.go_to_joint_state(goal_joints)  # moving the robot - code will continue only after the robot reaches the goal config
#        
        rospy.Subscriber("/iiwa/joint_states", JointState, self.joint_states_callback)
        print("Ros SPIN!()")
        rospy.spin()

    
if __name__=='__main__':
    
    print("Main application is starting")
    rospy.init_node('jey_listener', anonymous=True)

    
    try:
        move = MoveGroupPythonIntefaceTutorial()
        jl = jey_listener()
        jl.run(move)
        
    except rospy.ROSInterruptException: pass
    
    

    
