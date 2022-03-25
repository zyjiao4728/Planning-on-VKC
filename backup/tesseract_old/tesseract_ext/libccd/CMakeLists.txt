cmake_minimum_required(VERSION 3.5.0)

include(ExternalProject)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(LIBCCD_CXX_FLAGS "-w -std=c++14")
  set(LIBCCD_C_FLAGS "-w")
  ExternalProject_Add(libccd
    GIT_REPOSITORY    https://github.com/danfis/libccd.git
    GIT_TAG           master
    SOURCE_DIR        ${CMAKE_BINARY_DIR}/../tesseract_ext/libccd-src
    BINARY_DIR        ${CMAKE_BINARY_DIR}/../tesseract_ext/libccd-build
    CMAKE_CACHE_ARGS
            -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
            -DCMAKE_CXX_FLAGS:STRING=${LIBCCD_CXX_FLAGS}
            -DCMAKE_C_FLAGS:STRING=${LIBCCD_C_FLAGS}
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DENABLE_DOUBLE_PRECISION:BOOL=ON
  )
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  ExternalProject_Add(libccd
    GIT_REPOSITORY    https://github.com/danfis/libccd.git
    GIT_TAG           master
    SOURCE_DIR        ${CMAKE_BINARY_DIR}/../tesseract_ext/libccd-src
    BINARY_DIR        ${CMAKE_BINARY_DIR}/../tesseract_ext/libccd-build
    CMAKE_CACHE_ARGS
            -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DENABLE_DOUBLE_PRECISION:BOOL=ON
  )
endif()

install(FILES package.xml DESTINATION share/libccd)
