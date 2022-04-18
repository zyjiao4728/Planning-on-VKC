cmake_minimum_required(VERSION 3.5.0)

find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)

project(vkc VERSION 0.1.0 LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_srvs
  tesseract_monitoring
  tesseract_rosutils
)
find_package(Boost COMPONENTS system python REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_process_managers REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(trajopt_sqp REQUIRED)
find_package(astar REQUIRED)
find_package(ompl REQUIRED)

catkin_package()

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

# list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_17 CXX_FEATURE_FOUND)

# Create target
add_library(${PROJECT_NAME}_construct_vkc SHARED src/construct_vkc.cpp)
target_link_libraries(${PROJECT_NAME}_construct_vkc
 tesseract::tesseract_scene_graph  tesseract::tesseract_environment 
 tesseract::tesseract_support tesseract::tesseract_collision_bullet tesseract::tesseract_kinematics_kdl console_bridge)
target_compile_options(${PROJECT_NAME}_construct_vkc PUBLIC -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
# if(CXX_FEATURE_FOUND EQUAL "-1")
target_compile_options(${PROJECT_NAME}_construct_vkc PRIVATE -std=c++17)
# else()
# target_compile_features(${PROJECT_NAME}_construct_vkc PUBLIC cxx_std_17)
# endif()
target_cxx_version(${PROJECT_NAME}_construct_vkc  PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME}_construct_vkc PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_construct_vkc SYSTEM PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    $<TARGET_PROPERTY:tesseract::tesseract_common,INTERFACE_INCLUDE_DIRECTORIES>) #tesseract::tesseract_common Due to bug in catkin, there is an open PR

add_library(${PROJECT_NAME}_vkc_env_basic SHARED src/env/vkc_env_basic.cpp)
target_link_libraries(
  ${PROJECT_NAME}_vkc_env_basic
  ${PROJECT_NAME}_construct_vkc
  ${catkin_LIBRARIES}
)
target_compile_options(
  ${PROJECT_NAME}_vkc_env_basic 
  PUBLIC -Wsuggest-override -Wconversion -Wsign-conversion
)
target_cxx_version(${PROJECT_NAME}_vkc_env_basic  PUBLIC VERSION 17)
# if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_vkc_env_basic PRIVATE -std=c++17)
# else()
    # target_compile_features(${PROJECT_NAME}_vkc_env_basic PUBLIC cxx_std_17)
# endif()
target_include_directories(
  ${PROJECT_NAME}_vkc_env_basic PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_include_directories(
  ${PROJECT_NAME}_vkc_env_basic SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)


# add_library(${PROJECT_NAME}_urdf_scene_env SHARED src/env/urdf_scene_env.cpp)
# target_link_libraries(
#   ${PROJECT_NAME}_urdf_scene_env
#   ${PROJECT_NAME}_construct_vkc
#   ${PROJECT_NAME}_vkc_env_basic
#   ${PROJECT_NAME}_prob_generator
#   tesseract::tesseract_motion_planners_trajopt 
#   tesseract::tesseract_motion_planners_ompl
#   ${catkin_LIBRARIES}
# )
# target_compile_options(
#   ${PROJECT_NAME}_urdf_scene_env 
#   PUBLIC -Wsuggest-override -Wconversion -Wsign-conversion
# )
# if(CXX_FEATURE_FOUND EQUAL "-1")
#     target_compile_options(${PROJECT_NAME}_urdf_scene_env PUBLIC -std=c++11)
# else()
#     target_compile_features(${PROJECT_NAME}_urdf_scene_env PUBLIC cxx_std_11)
# endif()
# target_include_directories(
#   ${PROJECT_NAME}_urdf_scene_env PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>"
# )
# target_include_directories(
#   ${PROJECT_NAME}_urdf_scene_env SYSTEM PUBLIC
#   ${catkin_INCLUDE_DIRS}
# )

add_library(${PROJECT_NAME}_open_door_env SHARED src/env/open_door_env.cpp)
target_link_libraries(
  ${PROJECT_NAME}_open_door_env
  ${PROJECT_NAME}_construct_vkc
  ${PROJECT_NAME}_vkc_env_basic
  ${PROJECT_NAME}_prob_generator
  tesseract::tesseract_motion_planners_trajopt 
  ${catkin_LIBRARIES}
)
target_compile_options(
  ${PROJECT_NAME}_open_door_env 
  PUBLIC -Wsuggest-override -Wconversion -Wsign-conversion
)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_open_door_env PUBLIC -std=c++17)
else()
    target_compile_features(${PROJECT_NAME}_open_door_env PUBLIC cxx_std_17)
endif()
target_include_directories(
  ${PROJECT_NAME}_open_door_env PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_include_directories(
  ${PROJECT_NAME}_open_door_env SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}_arena_env SHARED src/env/arena_env.cpp)
# target_link_libraries(
#   ${PROJECT_NAME}_arena_env
#   ${PROJECT_NAME}_construct_vkc
#   ${PROJECT_NAME}_vkc_env_basic
#   ${PROJECT_NAME}_prob_generator
#   tesseract::tesseract_motion_planners_trajopt 
#   ${catkin_LIBRARIES}
# )
# target_compile_options(
#   ${PROJECT_NAME}_arena_env 
#   PUBLIC -Wsuggest-override -Wconversion -Wsign-conversion
# )
# if(CXX_FEATURE_FOUND EQUAL "-1")
#     target_compile_options(${PROJECT_NAME}_arena_env PUBLIC -std=c++17)
# else()
#     target_compile_features(${PROJECT_NAME}_arena_env PUBLIC cxx_std_17)
# endif()
# target_cxx_version(${PROJECT_NAME}_arena_env  PUBLIC VERSION ${TESSERACT_CXX_VERSION})
# target_include_directories(
#   ${PROJECT_NAME}_arena_env PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>"
# )
# target_include_directories(
#   ${PROJECT_NAME}_arena_env SYSTEM PUBLIC
#   ${catkin_INCLUDE_DIRS}
# )

# add_library(${PROJECT_NAME}_husky SHARED 
#   src/planner/ur_kin.cpp
# )
# target_link_libraries( ${PROJECT_NAME}_husky
#   ${catkin_LIBRARIES}
# )
# target_compile_options(
#   ${PROJECT_NAME}_husky
#   PUBLIC -Wsuggest-override -Wconversion -Wsign-conversion
# )
# if(CXX_FEATURE_FOUND EQUAL "-1")
#     target_compile_options(${PROJECT_NAME}_husky PUBLIC -std=c++17)
# else()
#     target_compile_features(${PROJECT_NAME}_husky PUBLIC cxx_std_17)
# endif()
# target_include_directories(
#   ${PROJECT_NAME}_husky PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>"
# )
# target_include_directories(
#   ${PROJECT_NAME}_husky SYSTEM PUBLIC
#   ${catkin_INCLUDE_DIRS}
# )


add_library(${PROJECT_NAME}_prob_generator SHARED 
  src/planner/prob_generator.cpp
  src/planner/traj_init.cpp
  src/action/actions.cpp
  src/planner/prob_translator.cpp
  src/planner/prob_translator_utils.cpp
)
target_link_libraries(${PROJECT_NAME}_prob_generator
  tesseract::tesseract_environment
  ${PROJECT_NAME}_construct_vkc
  ${PROJECT_NAME}_vkc_env_basic
  astar
  ${catkin_LIBRARIES}
)
target_compile_options(
  ${PROJECT_NAME}_prob_generator 
  PUBLIC -Wsuggest-override -Wconversion -Wsign-conversion
)
target_cxx_version(${PROJECT_NAME}_prob_generator  PUBLIC VERSION 17)

target_compile_options(${PROJECT_NAME}_prob_generator PUBLIC -std=c++17)

target_compile_features(${PROJECT_NAME}_prob_generator PUBLIC cxx_std_17)

target_include_directories(
  ${PROJECT_NAME}_prob_generator PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_include_directories(
  ${PROJECT_NAME}_prob_generator SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS} 
)



list(APPEND PACKAGE_LIBRARIES 
  ${PROJECT_NAME}_construct_vkc
  ${PROJECT_NAME}_vkc_env_basic
  # ${PROJECT_NAME}_arena_env
  # ${PROJECT_NAME}_urdf_scene_env
  ${PROJECT_NAME}_prob_generator
  ${PROJECT_NAME}_open_door_env
)

# Mark executables and/or libraries for installation
install(TARGETS ${PACKAGE_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE vkc:: DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Create cmake config files
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
