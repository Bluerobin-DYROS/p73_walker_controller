cmake_minimum_required(VERSION 3.8)
project(p73_lib)

add_compile_options(-Ofast)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(p73_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(br_driver REQUIRED)

add_library(${PROJECT_NAME}
  src/qp.cpp
  src/link.cpp
  src/wholebody_functions.cpp
  src/4bar_jac_func.cpp
)
set_target_properties(p73_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    /usr/local/include/br_driver
)

target_link_libraries(p73_lib
  Eigen3::Eigen
  pinocchio::pinocchio
  osqp::osqp
  OsqpEigen::OsqpEigen
)
ament_target_dependencies(p73_lib
  rclcpp 
  p73_msgs 
  tf2 
  tf2_eigen 
  geometry_msgs
  br_driver
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_p73_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3 p73_msgs pinocchio osqp OsqpEigen tf2 tf2_eigen geometry_msgs br_driver)
ament_export_include_directories(include)

ament_package()
