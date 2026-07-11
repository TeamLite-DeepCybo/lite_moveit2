cmake_minimum_required(VERSION 3.22)
project(lite_moveit2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

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
install(PROGRAMS
  scripts/move_to_pose
  scripts/translate_right_arm
  DESTINATION lib/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
