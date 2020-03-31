cmake_minimum_required(VERSION 2.8.3)
project(manipulation_msgs)

set(MSG_DEPS
  actionlib_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  shape_msgs
  household_objects_database_msgs
  )

find_package(catkin REQUIRED COMPONENTS message_generation ${MSG_DEPS})

set(
  MSG_FILES
  CartesianGains.msg
  ClusterBoundingBox.msg
  GraspableObjectList.msg
  GraspableObject.msg
  Grasp.msg
  GraspPlanningErrorCode.msg
  GraspResult.msg
  GripperTranslation.msg
  ManipulationPhase.msg
  ManipulationResult.msg
  PlaceLocation.msg
  PlaceLocationResult.msg
  SceneRegion.msg
)

set(
  SRV_FILES
  GraspPlanning.srv
)

set(
  ACTION_FILES
  GraspPlanning.action)

add_message_files(DIRECTORY msg FILES ${MSG_FILES})
add_service_files(DIRECTORY srv FILES ${SRV_FILES})
add_action_files(DIRECTORY action FILES ${ACTION_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
  CATKIN_DEPENDS message_generation ${MSG_DEPS}
  )
