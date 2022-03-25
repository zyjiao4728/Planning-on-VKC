cmake_minimum_required(VERSION 3.5.0)
project(descartes_samplers VERSION 0.1.0 LANGUAGES CXX)

find_package(descartes_light REQUIRED)
find_package(cmake_common_scripts REQUIRED)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

# If testing is enabled the clang tidy will errors instead of warning.
if(DESCARTES_ENABLE_TESTING)
  set(CLANG_ARGS ${DEFAULT_CLANG_TIDY_ERROR_ARGS})
else()
  set(DESCARTES_ENABLE_TESTING OFF) # Since this is being passed to the macros it must have a value otherwise it causes issues
  set(CLANG_ARGS ${DEFAULT_CLANG_TIDY_WARNING_ARGS})
endif()

add_code_coverage_all_targets(ENABLE ${DESCARTES_ENABLE_TESTING})

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/evaluators/distance_edge_evaluator.cpp
  src/evaluators/euclidean_distance_edge_evaluator.cpp
  src/evaluators/gantry_euclidean_distance_edge_evaluator.cpp
  src/evaluators/compound_edge_evaluator.cpp
  src/samplers/axial_symmetric_sampler.cpp
  src/samplers/cartesian_point_sampler.cpp
  src/samplers/external_axis_sampler.cpp
  src/samplers/fixed_joint_pose_sampler.cpp
  src/samplers/railed_axial_symmetric_sampler.cpp
  src/samplers/railed_cartesian_point_sampler.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC descartes::descartes_light)
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 11)
descartes_target_compile_options(${PROJECT_NAME} PUBLIC)
if(NOT MSVC)
  target_clang_tidy(${PROJECT_NAME} ENABLE ${DESCARTES_ENABLE_TESTING} ARGUMENTS ${CLANG_ARGS})
  target_code_coverage(${PROJECT_NAME} ALL ENABLE ${DESCARTES_ENABLE_TESTING})
endif()
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC)

configure_package(NAMESPACE descartes TARGETS ${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  PATTERN ".svn" EXCLUDE
 )
