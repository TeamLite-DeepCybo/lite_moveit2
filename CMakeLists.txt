cmake_minimum_required(VERSION 3.22)
project(lite_moveit2)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf/meshes/
  DESTINATION share/${PROJECT_NAME}/meshes)
install(FILES
  urdf/lite.urdf
  urdf/README.md
  DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY urdf/source
  DESTINATION share/${PROJECT_NAME}/urdf)
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
