cmake_minimum_required(VERSION 3.8)
project(inspection_tests)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  unit_cpp
  unit_python
  integration
  performance
  acceptance
  fixtures
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
