cmake_minimum_required(VERSION 3.5.0)
project(descartes_ikfast VERSION 0.1.0 LANGUAGES CXX)

find_package(descartes_light REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(LAPACK REQUIRED)
find_package(console_bridge REQUIRED)
find_package(cmake_common_scripts REQUIRED)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${console_bridge_INCLUDE_DIRS}")
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES "${console_bridge_LIBRARIES}")
endif()

# If testing is enabled the clang tidy will errors instead of warning.
if(DESCARTES_ENABLE_TESTING)
  set(CLANG_ARGS "-line-filter=[{'name':'ikfast.h','lines':[[9999999,9999999]]}, {'name':'fanuc_m20ia10l_manipulator_ikfast_solver.cpp','lines':[[9999999,9999999]]}, {'name':'.h'}, {'name':'.hpp'}, {'name':'.cpp'}]" ${DEFAULT_CLANG_TIDY_ERROR_ARGS})
else()
  set(DESCARTES_ENABLE_TESTING OFF) # Since this is being passed to the macros it must have a value otherwise it causes issues
  set(CLANG_ARGS "-line-filter=[{'name':'ikfast.h','lines':[[9999999,9999999]]}, {'name':'fanuc_m20ia10l_manipulator_ikfast_solver.cpp','lines':[[9999999,9999999]]}, {'name':'.h'}, {'name':'.hpp'}, {'name':'.cpp'}]" ${DEFAULT_CLANG_TIDY_WARNING_ARGS})
endif()

add_code_coverage_all_targets(ENABLE ${DESCARTES_ENABLE_TESTING})

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE console_bridge::console_bridge descartes::descartes_light_core ${LAPACK_LIBRARIES})
target_cxx_version(${PROJECT_NAME} INTERFACE VERSION 11)
descartes_target_compile_options(${PROJECT_NAME} INTERFACE)
if(NOT MSVC)
  target_clang_tidy(${PROJECT_NAME} ENABLE ${DESCARTES_ENABLE_TESTING} ARGUMENTS ${CLANG_ARGS})
  target_code_coverage(${PROJECT_NAME} INTERFACE ALL ENABLE ${DESCARTES_ENABLE_TESTING})
endif()
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE
    ${LAPACK_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

configure_package(NAMESPACE descartes TARGETS ${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  PATTERN ".svn" EXCLUDE
 )

if (DESCARTES_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target()
  add_subdirectory(test)
endif()
