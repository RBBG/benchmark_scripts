# -*- coding: utf-8 -*-
"""
Created on Tue Nov 10 10:56:45 2015

@author: ruben
"""


import sys
import os
import time
import logging

import numpy
import cPickle as pickle

import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs
import shape_msgs
import std_msgs
from datetime import datetime as dt

logging.basicConfig(level=logging.INFO)
                                    
                                    
class UR5_Benchmark():
        
    PUBLISHER_DELAY = 0.02 # max. 50 topics per second
    PLANNING_TIME = 2 # second, 

    # ********************************************************************* #
    #    
    def __init__(self):
        """ Class Constructor """    
        logging.info("UR5 Bench init")
        
        #Configuration settings

        self.scenes = ["scene_7"]
        self.planners = ["BITstar_stop","BITstar_smac"]
        self.planners = ["RRTC_smac","BiTRRT_smac","BITstar_smac","BKPIECE_smac","KPIECE_smac2"]
        self.planners = ["KPIECE","KPIECE_smac","KPIECE_smac2"]

        self.planners = ["RRTC","RRTC_smac","RRTC_smac2","BiTRRT","BiTRRT_smac","BiTRRT_smac2","BITstar_stop","BITstar_smac","BITstar_smac2","BKPIECE","BKPIECE_smac","BKPIECE_smac2","KPIECE","KPIECE_smac","KPIECE_smac2"]
        
        self.planners = ["KPIECE","KPIECE_smac"]
        self.planners = ["BITstar_stop","BITstar_smac"]
        self.planners = ["BKPIECE","BKPIECE_smac"]
        self.planners = ["BiTRRT","BiTRRT_smac"]
        self.planners = ["RRTC","RRTC_smac","KPIECE","KPIECE_smac"]
        self.planners = ["BIT","BIT_smac"]
        
        self.planners = ["BKPIECE_smac"]

        self.plan_iterations = 100
             
        #Object Publishers, can alsu use PlanningSceneInterface, but this doesn throw any warnings
        self.object_publisher = rospy.Publisher('/collision_object',
                moveit_msgs.msg._CollisionObject.CollisionObject,
                queue_size=100)

        self.attached_object_publisher = rospy.Publisher('/attached_collision_object',
                moveit_msgs.msg._AttachedCollisionObject.AttachedCollisionObject,
                queue_size=30)

        self.group = moveit_commander.MoveGroupCommander('manipulator')
        self.planning_frame = self.group.get_planning_frame() # "/world"
        self.robot = moveit_commander.RobotCommander()
        self.env_names = []
                        
        self.scene = moveit_commander.PlanningSceneInterface()

        rospy.sleep(0.5)
    
    
    # ********************************************************************* #
    #        
    def load_states(self,scene):
        """ Function that loads the valid states of a scene. They are stored in a 
        general location (self.states) as only 1 scene can be active at one moment"""
        
        with open("files/"+scene+".states") as f:
            lines = 0   # get lines to know amount of states
            for line in f:
                lines += 1                
            f.seek(0)   #reset readline
            
            states = [[0 for x in xrange(6)] for x in xrange(lines)] # 6 by Lines list for storing the states
            
            for num in range (0, lines):
                states[num] = eval(f.readline())
            
            self.states = states #store states in generic variable
   
   
   
    # ********************************************************************* #
    #    
    def benchmark_full(self):
        """ Full benchmark loop. Runs and stores the benchmark as configured in the
        Constructor. Consists of 4 loops
        1. iterates over the scenes
        2. iterates over the queries (all possible combinations of valid states)
        3. iterates over the planners
        4. iterates a number of times the get a consistent timing result
        Results are stored in self.results  """

        prog_counter = 0   			#progression counter that counts to 2*amount of scenes

        self.group.set_planning_time(self.PLANNING_TIME)     

        self.results = {}                       #create empty results dict
        for x1 in xrange(len(self.scenes)):     #scene loop            
    	    
            if (os.path.isfile('/home/ruben/moveit_ompl/new.db')):
    	        os.remove('/home/ruben/moveit_ompl/new.db') # remove the new database in case of thunder
               
            scene_name = self.scenes[x1]

            self.load_env(scene_name)           #load environment into collision model
            self.load_states(scene_name)        #load states into self.states
                        
            scene = {"name":scene_name}         #create scene dict
            query_count = 0             
            for x2 in xrange(len(self.states)):
            # for x2 in range(0,5):
                start = self.states[x2]
                for x3 in range (x2+1, len(self.states)):         #loops to decide start and goal states
                # for x3 in range(5,10):         #loops to decide start and goal states
                    # goal = self.states[x3]                          
                    goal = self.states[x3]
                                        
                    query = {"start":start, "goal":goal}        #create query dict
                    
                    for x4 in xrange(len(self.planners)):                        
                        planner_name = self.planners[x4]                
                        self.group.set_planner_id(planner_name)     #set new planner ID
                                      
                        planner_results = {"name":planner_name}     #create new planner dict
                        for it in range (0, self.plan_iterations):                                        
                            planner_results[it] = self.plan_path(start,goal)    #plan path and add results to planner dict    
                        query[x4] = planner_results                         #add planner dict to query dict
                    scene[query_count] = query                  #add query dict to scene dict
                    query_count += 1
		    prog_counter += 1
		    
		    print prog_counter
                    
            self.results[x1] = scene                            #add scene dict to results dict
    	    
            with open('results/benchmark_data.p', 'wb') as fp:    #store this scene's data
                	pickle.dump(self.results, fp)                
            self.clear_env()
    
    
    # ********************************************************************* #
    #    

    def clear_env(self):        
        """  clears the collision model for a new scene to be loaded """    
        for x in xrange(len(self.env_names)):
            print self.env_names[x]
            self.scene.remove_world_object(self.env_names[x])
        self.env_names = []
    
    
    def load_env(self,scene):        
        """ Function that parses a .scene file and loads the environment. Currently
        Only supports the BOX type, but can be easily extended if needed"""    
    
        with open("files/"+scene+".scene") as f:
            
            lines = 0   # get lines to know amount of blocks
            for line in f:
                lines += 1                
            f.seek(0)   # reset readfile location
               
            title = f.readline()
            logging.info("loading scene:%s" % title)
            
            cnt = (lines-2)/7             # object data is 7 lines long, count amount of objects
            
            for objs in range (0, cnt):
                name = f.readline()[2:]   # object name
                number = f.readline()     # object number?
                shape = f.readline()      # object shape
                
                self.env_names.append(name)   #adding name to list in order to clear later
                
                #****** Parsing dimension ******#
                text = f.readline()
                dim = []
                for x in range (0, 3):          #3D dimension
                    loc = text.find(" ")
                    dim.append(float(text[:loc]))
                    text = text[loc+1:]         #Remove used text
                
                #****** Parsing Location ******#
                text = f.readline()
                pos = []
                for x in range (0, 3):          #3D dimension
                    loc = text.find(" ")        
                    pos.append(float(text[:loc]))
                    text = text[loc+1:]
                    
                #****** Parsing Rotation ******#
                text = f.readline()
                rot = []
                for x in range (0, 4):          #4D dimension
                    loc = text.find(" ")
                    rot.append(float(text[:loc]))
                    text = text[loc+1:]
                    
                #****** Parsing Colour ******#
                text = f.readline()
                col = []
                for x in range (0, 4):
                    loc = text.find(" ")
                    col.append(float(text[:loc]))
                    text = text[loc+1:]
                # Currently unused, also not needed for adding objects
                    
                
                #******* adding the object ********#
                object_shape = shape_msgs.msg._SolidPrimitive.SolidPrimitive()
                object_shape.type = object_shape.BOX #extend support for other primitives?
                object_shape.dimensions = dim
                
                object_pose = geometry_msgs.msg._Pose.Pose()
                object_pose.position.x = pos[0]
                object_pose.position.y = pos[1]
                object_pose.position.z = pos[2]
                
                object_pose.orientation.x = rot[0]
                object_pose.orientation.y = rot[1]
                object_pose.orientation.z = rot[2]
                object_pose.orientation.w = rot[3]
                
                object = moveit_msgs.msg.CollisionObject()
                object.id = name
                object.header.frame_id = self.planning_frame
                object.primitives.append(object_shape)
                object.primitive_poses.append(object_pose)
                
                assert type(object) == moveit_msgs.msg.CollisionObject
                
                object.header.stamp = rospy.Time.now()
                object.operation = object.ADD
                self.object_publisher.publish(object)
                time.sleep(self.PUBLISHER_DELAY)
                
        rospy.sleep(0.5)  #sleep to let everything sync, not sure if needed


            
    def plan_path(self, start, goal):
        """ Plans a path from start to goal and returns the resulting path, timing
        information, succes bit, and path lengths (both config and workspace)"""    
        
        start_state = self.robot.get_current_state()

        start_state.joint_state.position = start

        self.group.set_start_state(start_state)   #needs RobotState type, so fetch current state and change position  
        # self.group.set_joint_value_target(goal[:-1])     #takes a list as input
        self.group.set_joint_value_target(goal)
        
        start = time.time()*1000                # *1000 to get millisecond
        planned_path = self.group.plan()
        end = time.time()*1000

        success = 0
        length = 0
        if len(planned_path.joint_trajectory.points) != 0:      #not sure how to figure out a failure otherwise
            length = self.get_path_length(planned_path)
            success = 1
                
        runtime = end-start
        result = {"path":planned_path, "time":runtime, "length":length, "success":success}  
        #note that planned_path is a moveit msg type! moveit_msgs/RobotTrajectory Message!!! 

        return result
                
    
    
    def get_path_length(self, path):
        """ Function to calculate path lengths. Both config and workspace and both
        Straight Line and actual path.
        Workspace calculation uses forward kinematics service from move_group"""
              
        pts = path.joint_trajectory.points  # make a shorthand for easy access
      
        j_length = 0    # j = jointspace
        w_length = 0    # w = workspace
        
        num_pts = len(pts)
            
        a = numpy.array(pts[0].positions)           #numpy array for easy norm calc
        b = numpy.array(pts[num_pts-1].positions)
        
        # a1 = numpy.array(self.get_forward_kinematics(a))    #get 3D fwd kinematics coordinates
        # b1 = numpy.array(self.get_forward_kinematics(b))

        j_straight = numpy.linalg.norm((a - b), ord=2)
        
        # w_straight = numpy.linalg.norm((a1 - b1), ord=2)
        w_straight = 0
      
        for x in xrange(num_pts-1):
            
            a = numpy.array(pts[x].positions)
            b = numpy.array(pts[x+1].positions)
            
            # a1 = numpy.array(self.get_forward_kinematics(a))
            # b1 = numpy.array(self.get_forward_kinematics(b))
            
            j_length += numpy.linalg.norm((a - b), ord=2)
            # w_length += numpy.linalg.norm((a1 - b1), ord=2)
            w_length += 0
                        
        lengths = {'joint_path':j_length, 'joint_straight':j_straight, 'work_path':w_length, 'work_straight': w_straight}
        return lengths
        

    def get_forward_kinematics(self, joint_pos):
        """ Function that gets the forward kinematics using the move_group service """
          
        rospy.wait_for_service('compute_fk')
        try:
            moveit_fk = rospy.ServiceProxy('compute_fk', moveit_msgs.srv._GetPositionFK.GetPositionFK)
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s"%e)
        
        fkln = ['ee_link']   #forward kinematic link to be calculated
        
        header = std_msgs.msg.Header(0,rospy.Time.now(),"/world")   #make header for the argument to moveit_fk
        
        rs = self.robot.get_current_state()
        rs.joint_state.position = joint_pos   #robot state for argument to moveit_fk
        
        fwd_kin = moveit_fk(header, fkln,rs)
        
        pos = fwd_kin.pose_stamped[0].pose.position   #extract position
        fwd_kin_coordinates = [pos.x, pos.y, pos.z]  
        
        return fwd_kin_coordinates
        
        
if __name__ == "__main__":
    """ The main, init roscpp and rospy, construct class and launch benchmark """
    
    moveit_commander.roscpp_initialize(sys.argv) 
    rospy.init_node('Benchmark_node', anonymous=True)
    
    ur = UR5_Benchmark()
    
    ur.benchmark_full()

    # ur.load_env("scene_1")
    
    data_string = 'results/bm_' + str(dt.now().month) + '.' + str(dt.now().day) + '_' + str(dt.now().hour) + '.' + str(dt.now().minute) + '_' + str(ur.plan_iterations) + '.p'

    with open(data_string, 'wb') as fp:    #final data store
        pickle.dump(ur.results, fp)

    with open('results/benchmark_data.p', 'wb') as fp:    #final data store
        pickle.dump(ur.results, fp)
        
    moveit_commander.roscpp_shutdown()
