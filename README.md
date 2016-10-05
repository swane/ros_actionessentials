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
