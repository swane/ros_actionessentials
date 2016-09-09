#ROS Action Essentials
Create a new package in the usual way:
in 'catkin_ws\src` `catkin_create_pkg topic_test rospy` 

Now copy the following two files over and make minor changes:
##package.xml
In package.xml, just change the name of the package in the XML code:
  `<name>package name</name>`
##CMakeLists.txt
In CMakeLists.txt, change the package name:
`project(package name)`

and the name of the action file to the one you used:
```
## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY action
   FILES name.action
)
```
Remember to have a directory named `action` in the same package (as src):
```
\action
\src
CMakeLists.txt
package.xml
```
