cmake_minimum_required(VERSION 2.8.3)
project(household_objects_database_msgs)

set(MSG_DEPS
  std_msgs
  geometry_msgs
  shape_msgs
  object_recognition_msgs
  )

find_package(catkin REQUIRED COMPONENTS message_generation ${MSG_DEPS})

set(
  MSG_FILES
  DatabaseModelPoseList.msg  
  DatabaseModelPose.msg  
  DatabaseReturnCode.msg  
  DatabaseScan.msg
)

set(
  SRV_FILES
  GetModelDescription.srv  
  GetModelList.srv  
  GetModelMesh.srv  
  GetModelScans.srv  
  SaveScan.srv  
  TranslateRecognitionId.srv
)

add_message_files(DIRECTORY msg FILES ${MSG_FILES})
add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
  CATKIN_DEPENDS message_generation ${MSG_DEPS}
  )
