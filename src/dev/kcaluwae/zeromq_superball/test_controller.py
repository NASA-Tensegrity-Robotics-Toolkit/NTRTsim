
from std_msgs import msg
from std_msgs.msg import String
from std_msgs.msg import Float32, UInt16
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from superball.msg import SUPERballStateArray
import numpy as np
import time

import rospy
rospy.init_node("test_controller")

motor_pub = [] 

for i in xrange(12):
	board_id = i%2
	if(board_id==0):
		bbb = i+2
		board_id = 0x71
		sub_index = 0x2
	else:
		bbb = i+1
		board_id = 0x1
		sub_index = 0x1
	motor_pub.append(rospy.Publisher("/bbb%d/0x%x_0x2040_0x%x"%(bbb,board_id,sub_index),Float32,queue_size=1))
	#print motor_pub[-1]
	
	
def set_motor_positions(pos):
	msg = Float32()
	for i in xrange(12):
		msg.data = pos[i]
		motor_pub[i].publish(msg)

positions = np.zeros((6,3)) 
quaternions = np.zeros((6,4))
pose_update = False
def pose_cb(msg):
	global pose_update
	pose_update = True
#	for i in xrange(6):
#		positions[i] = (msg.poses[i].position.x,msg.poses[i].position.y,msg.poses[i].position.z)
#		quaternions[i] = (msg.poses[i].orientation.x,msg.poses[i].orientation.y,msg.poses[i].orientation.z,msg.poses[i].orientation.w)

ctrl_pub = rospy.Publisher("/superball/control",String,queue_size=1)
timestep_pub = rospy.Publisher("/superball/timestep",UInt16,queue_size=1)
pose_sub = rospy.Subscriber("/superball/state",SUPERballStateArray,pose_cb)

time.sleep(2) #some time for the sockets to initialize (weird ROS stuff?)
#reset the simulator
msg = String("reset") 
ctrl_pub.publish(msg)
time.sleep(.5)

timestep = 100 #milliseconds
cur_time = 0
roll_step_time = 1.5 #seconds
cur_roll_time = 0. 
motor_idx = 0
num_iters = 600 #number of iterations
_iter = 0
robot_pos_hist = np.zeros((num_iters,6,3))
while (not rospy.is_shutdown() and _iter < num_iters):
    try:
        _iter += 1
        #set desired motor positions
        if(cur_roll_time>roll_step_time):
            motor_idx += 1
            cur_roll_time = 0
            if(motor_idx>=6):
                motor_idx = 0
        cur_roll_time += timestep*0.001 
        motor_pos = np.zeros(12)
        if(cur_time*0.001>30):
            motor_pos[-3:] = 5 #make it roll sideways by squeezing a side triangle
            
        motor_pos[motor_idx] = 40
        set_motor_positions(motor_pos)
        #advance the simulation	
        msg = UInt16(timestep)
        timestep_pub.publish(msg)
        #compute robot position
        #print "robot center: "
        print positions.mean(0)
        cur_time += timestep
        #print cur_time*0.001
        robot_pos_hist[_iter-1] = positions
        #time.sleep(0.2)
        while(not pose_update and not rospy.is_shutdown()):
            time.sleep(0.001) 
        pose_update = False
    except KeyboardInterrupt:
        break

from matplotlib import pylab as plt
plt.ion()
plt.xlabel("X (m)")
plt.ylabel("Y (m)")

plt.plot(np.mean(robot_pos_hist,1)[:,0],np.mean(robot_pos_hist,1)[:,2]) #I think the second axis is up (Z) in NTRT
plt.axis("equal")

