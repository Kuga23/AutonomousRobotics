#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('floor_nav')
import rospy
from math import *
from task_manager_lib.TaskClient import *


rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.2)
tc = TaskClient(server_node,default_period)

wp = [ [1., 9., pi/2, 0, 0, 255],
    [9., 9., 0., 0, 255, 255],
    [9., 1., -pi/2, 0, 255, 0],
    [1., 1., -pi, 255, 255, 0]]



while True:
    tc.Wait(duration=1.)


    # Start the wait for face task in the background
    wff = tc.WaitForFace(foreground=False)


    tc.addCondition(ConditionIsCompleted("Face detector",tc,wff))
    try:
        tc.Wander()
        # Clear the conditions if we reach this point
        tc.clearConditions()
    except TaskConditionException as e:
        rospy.loginfo("Path following interrupted on condition: %s" % \
                " or ".join([str(c) for c in e.conditions]))
        # StareAtFace 
        tc.StareAtFace(foreground=True)
        rospy.loginfo("LOoKING AT FACE")       
        tc.Wait(duration=2.)
        tc.SetHeading(target=pi/2,relative=True)
 
rospy.loginfo("Mission completed")
