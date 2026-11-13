cmake_minimum_required(VERSION 3.0.2)
project(kmriiwa_circle_demo)
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs sensor_msgs nav_msgs moveit_msgs visualization_msgs kmriiwa_msgs)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/circle_demo_node.py scripts/preview_circle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md COMMANDS.txt LICENSE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
