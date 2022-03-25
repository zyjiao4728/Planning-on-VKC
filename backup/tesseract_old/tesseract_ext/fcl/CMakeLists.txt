cmake_minimum_required(VERSION 3.5.0)

include(ExternalProject)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(FCL_CXX_FLAGS "-w -std=c++14")
  set(FCL_C_FLAGS "-w")

  ExternalProject_Add(fcl
    GIT_REPOSITORY    https://github.com/flexible-collision-library/fcl.git
    GIT_TAG           master
    SOURCE_DIR        ${CMAKE_BINARY_DIR}/../tesseract_ext/fcl-src
    BINARY_DIR        ${CMAKE_BINARY_DIR}/../tesseract_ext/fcl-build
    CMAKE_CACHE_ARGS
            -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DCMAKE_CXX_FLAGS:STRING=${FCL_CXX_FLAGS}
            -DCMAKE_C_FLAGS:STRING=${FCL_C_FLAGS}
            -DFCL_BUILD_TESTS:BOOL=OFF
            -DBUILD_TESTING:BOOL=OFF
  )
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  ExternalProject_Add(fcl
    GIT_REPOSITORY    https://github.com/flexible-collision-library/fcl.git
    GIT_TAG           master
    SOURCE_DIR        ${CMAKE_BINARY_DIR}/../tesseract_ext/fcl-src
    BINARY_DIR        ${CMAKE_BINARY_DIR}/../tesseract_ext/fcl-build
    CMAKE_CACHE_ARGS
            -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DFCL_BUILD_TESTS:BOOL=OFF
            -DBUILD_TESTING:BOOL=OFF
  )
endif()

install(FILES package.xml DESTINATION share/fcl)
