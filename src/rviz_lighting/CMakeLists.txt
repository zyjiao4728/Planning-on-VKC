cmake_minimum_required(VERSION 2.8.3)
project(rviz_lighting)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rviz
)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if(rviz_QT_VERSION VERSION_LESS "5")
qt4_wrap_cpp(MOC_FILES
  src/AmbientLightDisplay.h
  src/LightDisplay.h
  )
else()
qt5_wrap_cpp(MOC_FILES
  src/AmbientLightDisplay.h
  src/LightDisplay.h
  )
endif()

add_library(${PROJECT_NAME}
  src/AmbientLightDisplay.h
  src/AmbientLightDisplay.cpp
  src/LightDisplay.h
  src/LightDisplay.cpp
  src/LightVisual.h
  src/LightVisual.cpp
  ${MOC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

# install
install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
