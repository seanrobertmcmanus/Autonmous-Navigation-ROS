cmake_minimum_required(VERSION 2.8.3)
project(minitask5)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs message_generation)

add_message_files(
    FILES
    Location.msg
    Localization.msg
    PerceptionObjects.msg
    PerceptionNode.msg
    MapPoint.msg
    MapPoints.msg
    

    
)
add_service_files(
  FILES
  GeneralMovement.srv
  MapScan.srv
  MoveBaseDone.srv
  ObjectIdService.srv
)


generate_messages(
    DEPENDENCIES
    std_msgs
)





catkin_package(CATKIN_DEPENDS rospy std_msgs geometry_msgs)

catkin_install_python(PROGRAMS src/MapServerNode.py src/PerceptionNode.py src/PathPlanningNode.py src/TurtleCommandNode.py src/GeneralMovement.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

