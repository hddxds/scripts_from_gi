#!/usr/bin/env python
import roslib;
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

cmps=None
import thread
mutex=thread.allocate_lock()
def updateCompass(data):
    global mutex
    global cmps
    if(not cmps==None and mutex.locked()):
        return
    mutex.acquire()
    cmps=data.orientation
    mutex.release()

def listener():
    rospy.init_node('fuckgo')
    rospy.Subscriber('/mavros/vision_pose/pose_nimabi', PoseStamped, callback)
    rospy.Subscriber('/mavros/imu/data',Imu, updateCompass)
    rospy.spin()

init = None
stop=False

routine=[('F', 4), ('R', 90), ('F', 6), ('L', 180), ('F', 10), ('R', 180), ('F', 4), ('R', 90), ('F', 5), ('L', 180), ('F', 1)]
routine=[('R', 90), ('F', 0.5), ('R', 90), ('F', 0.5), ('L', 90), ('B', 0.5), ('R', 90), ('F', 0.5), ('L', 90), ('F', 0.5), ('L', 90), ('F', 0.5), ('R', 90), ('B', 0.5), ('L', 90), ('F',0.5)]
routine=[('R', 90), ('R', 90), ('L', 90), ('R', 90), ('L', 90), ('L', 90), ('R', 90), ('L', 90)]
routine=[('R', 90),('R', 90),('R',90),('R',90)]
routine.reverse()
pose=None
work=None

mx=0
mn=0
import math

aver=[]
import numpy
def callback(data):
	settings = termios.tcgetattr(sys.stdin)
        global cmps
        if(cmps==None): # and (work[0]=='L' or work[0]=='R')):
            return
        global mutex
        mutex.acquire()
        x=cmps.x
        y=cmps.y
        z=cmps.z
        w=cmps.w
        cmps=None
        mutex.release()
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        # angle = math.atan2(2 * (y*z +w*x), w*w -x*x - y*y +z*z)/math.pi*180
        # math.asin(2.0 * (w *y -z * x))/math.pi*180
        angle = math.atan2(2.0 *(w * z - y * x), 1.0 - 2.0 * (y * y + z * z))/math.pi*180
        # print('a1',angle, 'a2',a2, 'a3',a3)
        north = data.pose.position.y
        east = data.pose.position.x
        global aver
        aver.append([angle,180-angle if angle>0 else -180-angle, north, east])
        if(len(aver)==1):
            angle, fangle, north, east=numpy.asarray(aver).mean(axis=0)
            # if(angle+fangle<179 and angle+fangle>-179):
            #     angle=180-fangle if fangle > 0 else -180-fangle
            aver=[]
        else:
            return
	# rospy.init_node('teleop_twist_keyboard')
        global mx
        global mn
        mx=max(angle, mx)
        mn=min(angle, mn)
        # print(mn/180*math.pi, mx/180*math.pi, angle/180*math.pi, mn, mx, angle)
        speed=0.5
        turn=0.4

        global pose
        global work
        global routine
        if(work==None):
            if(len(routine)==0):
                print('No jobs')
                return
            else:
                # print(currentPose)
                work=routine.pop()
                # work = ('L',90)
                pose=(north, east, angle)
        print(north, east, angle)

        if(work[0]=='L'):
            print(pose[2], angle)
            delta=(pose[2]-angle) % 360
            delta=0 if delta>270 else delta
            if(delta<work[1]):
                key='j'
                turn=min(min(work[1]-delta,delta)/200+0.1,1,turn)
                speed=0
            else:
                print('L{} done'.format(work[1]))
                work=None
                return
        elif(work[0]=='R'):
            delta=(angle-pose[2]) % 360
            delta=0 if delta>270 else delta
            if(delta<work[1]):
                key='l'
                turn=min(min(work[1]-delta,delta)/200+0.1,1,turn)
                speed=0
            else:
                print('R{} done'.format(work[1]))
                work=None
                return
        elif(work[0]=='F'):
            delta=math.sqrt(math.pow(north-pose[0],2)+math.pow(east-pose[1],2))
            if(delta<work[1]):
                key='i'
                speed=min(min(work[1]-delta,delta)/0.75+0.1,1,speed)
                turn=0
            else:
                print('F{} done'.format(work[1]))
                work=None
                return
        elif(work[0]=='B'):
            delta=math.sqrt(math.pow(north-pose[0],2)+math.pow(east-pose[1],2))
            if(delta<work[1]):
                key=','
                speed=min(min(work[1]-delta,delta)/0.75+0.1,1,speed)
                turn=0
            else:
                print('B{} done'.format(work[1]))
                work=None
                return
        else:
            print('Unknown command')
            return
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	#try:
	# print msg
	# print vels(speed,turn)
	# key = getKey()
	if key in moveBindings.keys():
		x = moveBindings[key][0]
		y = moveBindings[key][1]
		z = moveBindings[key][2]
		th = moveBindings[key][3]
	elif key in speedBindings.keys():
		speed = speed * speedBindings[key][0]
		turn = turn * speedBindings[key][1]
		# print vels(speed,turn)
		if (status == 14):
			print msg
		status = (status + 1) % 15
	else:
		x = 0
		y = 0
		z = 0
		th = 0
	twist = Twist()
	twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
        # print(twist)
	pub.publish(twist)
	# except:
	# 	pass # print e

	# finally:
	# 	twist = Twist()
	# 	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	# 	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	# 	pub.publish(twist)

    	# 	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__=="__main__":
    listener()
