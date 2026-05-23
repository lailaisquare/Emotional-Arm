cmake_minimum_required(VERSION 3.22)
project(movelt_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/move_arm.py
  scripts/move_arm_pose.py
  scripts/move_arm_ik.py
  scripts/move_arm_fk.py
  scripts/move_arm_collision_primitive.py
  scripts/move_arm_keyboard.py
  scripts/move_arm_config.py
  scripts/move_arm_record.py
  scripts/move_arm_playback.py
  DESTINATION lib/${PROJECT_NAME}
)
