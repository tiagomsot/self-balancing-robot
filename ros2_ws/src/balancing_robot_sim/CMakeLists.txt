cmake_minimum_required(VERSION 3.8)
project(balancing_robot_sim)

find_package(ament_cmake REQUIRED)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp ${CMAKE_CURRENT_BINARY_DIR}/robot_core)
install(TARGETS robot_core LIBRARY DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/balance_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME balance_controller)
install(PROGRAMS scripts/motor_driver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motor_driver)
install(FILES scripts/motor_model.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/motion_core.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/shared_core.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/shuttle_demo.py DESTINATION lib/${PROJECT_NAME} RENAME shuttle_demo)
ament_package()
