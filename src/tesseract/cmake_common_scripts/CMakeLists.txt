cmake_minimum_required(VERSION 3.5.0)
project(cmake_common_scripts VERSION 0.1.0)
include(cmake/cmake_tools.cmake)

configure_package()

install(FILES
  "${CMAKE_CURRENT_LIST_DIR}/cmake/cmake_tools.cmake"
  "${CMAKE_CURRENT_LIST_DIR}/cmake/code_coverage.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

