cmake_minimum_required(VERSION 3.8)
project(p73_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Ofast)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-class-memaccess) # for memcpy warning
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(p73_msgs REQUIRED)
find_package(p73_lib REQUIRED)
find_package(br_driver REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(p73_cc QUIET)
IF(p73_cc_FOUND)
 message(" -- p73 custom based custom controller package found" )
   ADD_DEFINITIONS(-DCOMPILE_CC)
ENDIF()

include_directories(
  include
  ${p73_lib_INCLUDE_DIRS}
  ${p73_cc_INCLUDE_DIRS}
  /usr/local/include/br_driver
)

add_library(${PROJECT_NAME}_lib
  src/state_estimator.cpp
  src/p73_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  p73_msgs
  p73_lib
  sensor_msgs
)

target_link_libraries(${PROJECT_NAME}_lib
  ${p73_cc_LIBRARIES}
  br_driver
)



# add_executable(p73_controller src/main.cpp)
# target_link_libraries(p73_controller ${PROJECT_NAME}_lib)

add_executable(p73_controller src/main.cpp)
target_link_libraries(p73_controller ${PROJECT_NAME}_lib)

add_executable(shm_reset src/shm_reset.cpp)
target_link_libraries(shm_reset br_driver)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  p73_controller
  shm_reset
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
