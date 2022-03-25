cmake_minimum_required(VERSION 3.5.0)
project(descartes_light VERSION 0.1.0 LANGUAGES CXX)
include(cmake/descartes_light_macros.cmake)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

find_package(OpenMP REQUIRED)

if(NOT TARGET OpenMP::OpenMP_CXX)
    find_package(Threads REQUIRED)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
    # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)

endif()

find_package(Eigen3 REQUIRED)
find_package(cmake_common_scripts REQUIRED)
find_package(console_bridge)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${console_bridge_INCLUDE_DIRS}")
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES "${console_bridge_LIBRARIES}")
endif()

# If testing is enabled the clang tidy will errors instead of warning.
if(DESCARTES_ENABLE_TESTING)
  set(CLANG_ARGS ${DEFAULT_CLANG_TIDY_ERROR_ARGS})
else()
  set(DESCARTES_ENABLE_TESTING OFF) # Since this is being passed to the macros it must have a value otherwise it causes issues
  set(CLANG_ARGS ${DEFAULT_CLANG_TIDY_WARNING_ARGS})
endif()

add_code_coverage_all_targets(ENABLE ${DESCARTES_ENABLE_TESTING})

# Interface Library
add_library(${PROJECT_NAME}_core INTERFACE)
target_link_libraries(${PROJECT_NAME}_core INTERFACE console_bridge::console_bridge)
target_cxx_version(${PROJECT_NAME}_core INTERFACE VERSION 11)
descartes_target_compile_options(${PROJECT_NAME}_core INTERFACE)
if(NOT MSVC)
  target_clang_tidy(${PROJECT_NAME}_core ENABLE ${DESCARTES_ENABLE_TESTING} ARGUMENTS ${CLANG_ARGS})
  target_code_coverage(${PROJECT_NAME}_core INTERFACE ALL ENABLE ${DESCARTES_ENABLE_TESTING})
endif()
target_include_directories(${PROJECT_NAME}_core INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_core SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS})

# Core Library
add_library(${PROJECT_NAME} SHARED
  src/descartes_light.cpp
  src/ladder_graph.cpp
  src/ladder_graph_dag_search.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC console_bridge::console_bridge OpenMP::OpenMP_CXX)
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 11)
descartes_target_compile_options(${PROJECT_NAME} PUBLIC)
if(NOT MSVC)
  target_clang_tidy(${PROJECT_NAME} ENABLE ${DESCARTES_ENABLE_TESTING} ARGUMENTS ${CLANG_ARGS})
  target_code_coverage(${PROJECT_NAME} ALL ENABLE ${DESCARTES_ENABLE_TESTING})
endif()
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

# Declare a C++ library
add_library(${PROJECT_NAME}_gantry SHARED src/gantry_kinematics.cpp)
target_link_libraries(${PROJECT_NAME}_gantry PUBLIC console_bridge::console_bridge ${PROJECT_NAME}_core)
target_cxx_version(${PROJECT_NAME}_gantry PUBLIC VERSION 11)
descartes_target_compile_options(${PROJECT_NAME}_gantry PUBLIC)
if(NOT MSVC)
  target_clang_tidy(${PROJECT_NAME}_gantry ENABLE ${DESCARTES_ENABLE_TESTING} ARGUMENTS ${CLANG_ARGS})
  target_code_coverage(${PROJECT_NAME}_gantry ALL ENABLE ${DESCARTES_ENABLE_TESTING})
endif()
target_include_directories(${PROJECT_NAME}_gantry PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_gantry SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS})

configure_package(NAMESPACE descartes TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME} ${PROJECT_NAME}_gantry)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  PATTERN ".svn" EXCLUDE
 )

install(FILES
  "${CMAKE_CURRENT_LIST_DIR}/cmake/descartes_light_macros.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})
