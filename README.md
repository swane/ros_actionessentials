# ros_actionessentials
CMake and Package files for Actions in ROS
In package.xml, just change the name of the package:
  <name>package name</name>
  
In CMakeLists.txt, change the package name:
project(package name)

and the name of the action file to the one you used:
## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY action
   FILES name.action
)
