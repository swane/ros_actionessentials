#ROS Action Essentials
Create a new package in the usual way:
in 'catkin_ws\src` `catkin_create_pkg package_name rospy` 

Now copy the following two files over and make minor changes:
##package.xml
In package.xml, just change the name of the package in the XML code:
  `<name>package_name</name>`
##CMakeLists.txt
In CMakeLists.txt, change the package name:
`project(package_name)`

and the name of the action file to the one you used:
```
## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY action
   FILES name.action
)
```
Remember to have a directory named `action` in the same package (as src):
`mkdir action`


```
\action
\src
CMakeLists.txt
package.xml
```
In the 'action' folder create a file with the same 'name.action' as in the FILES of CMakeLists.txt.
This has the format of:
```
GOAL
---
RESULT
---
FEEDBACK
```
e.g.:
```
cd action
gedit name.action
```
File could be:
```
float64 goaldist
---
float64 resultdist
---
float64 feedbackdist
```

Then move to catkin_ws, and `catkin_make`
Message files .msg should be created and reported on during build (these can be found in: catkin_ws/devel/share/package_name)

In the src/ create the program to accomplish the action, with the following essentials (note package_name, and name.action):
`gedit action_program.py`

```
import rospy
import actionlib
from package_name.msg import nameAction,nameGoal,nameResult,nameFeedback

def action_subroutine(goal):
	
		feedback=nameFeedback()

		feedback.feedbackdist=n
		
		server.publish_feedback(feedback)		
	
	  result=nameResult()
	  result.resultdist=n
	
	server.set_succeeded(result,"Within 5m of location")

rospy.init_node('NODE_NAME',anonymous=False)
server=actionlib.SimpleActionServer('action_subroutine',basemoveAction,action_subroutine,False)

server.start()
rospy.spin()
```

This should be the basics needed to start the action. Save and change the attributes to run (`chmod +x *`)
Run roscore.
Run with `./action_program.py'`
Check action server is running with:
`rostopic list`
Should show the actiobn messages.

Full example of an Action Server:
```
#!/usr/bin/env python


import rospy

import actionlib
from action_autopic_base.msg import basemoveAction,basemoveGoal,basemoveResult,basemoveFeedback
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

global dist
"""
float64 goaldist
---
float64 resultdist
---
float64 feedbackdist
"""
def encoder_callback(msg):
	global dist
	dist=msg.data
	print("Encoder callback:%f"%(dist))

def fwd_to_dist(goal):
	global dist
	disttogo=dist
	twist = Twist()
	p = rospy.Publisher('cmd_vel', Twist)
	print("Dist to go:%f"%(disttogo))
	stepdist=goal.goaldist
	while (dist-disttogo<stepdist):	
	
		print("Dist to go - dist:%f"%(dist-disttogo))
		feedback=basemoveFeedback()

		feedback.feedbackdist=dist-disttogo

		server.publish_feedback(feedback)		
		
		
		
		twist.linear.x = 0.3;                   # our forward speed
		twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!        
		twist.angular.x = 0; twist.angular.y = 0;   #          or these!
		twist.angular.z = 0;                        # rotation in rad / sec

	    	p.publish(twist)

		rospy.sleep(0.1)
	result=basemoveResult()
	result.resultdist=dist-disttogo
	server.set_succeeded(result,"Moved distance")
	twist.linear.x = 0.0;  
	p.publish(twist)
	
	

def shutdown(self):
	rospy.loginfo("Stopping the robot...")
	#self.cmd_vel.publish(Twist())
print "node launched"
rospy.init_node('AUTOPIC_BASE_MOVE',anonymous=False)
print "nodelaunched"  
sub=rospy.Subscriber('encoder',Float64,encoder_callback)

server=actionlib.SimpleActionServer('fwd_to_dist',basemoveAction,fwd_to_dist,False)

print "Starting action"
server.start()
rospy.spin()
```

Example of a client to call the action:
```
#!/usr/bin/env python

import rospy
import actionlib
from action_autopic_base.msg import basemoveAction,basemoveGoal,basemoveResult,basemoveFeedback

def feedback_cb(feedback):
  print('<feedback N>:%f'%feedback.feedbackdist)


rospy.init_node('movebase_client')
client=actionlib.SimpleActionClient('fwd_to_dist',basemoveAction)
client.wait_for_server()
goal=basemoveGoal()
goal.goaldist=2000

client.send_goal(goal,feedback_cb=feedback_cb)
client.wait_for_result()
print('Actual dist:%f'%(client.get_result().resultdist))
```
