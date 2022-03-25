find_package(GTest QUIET)
if ( NOT ${GTest_FOUND} )
  include(ExternalProject)

  ExternalProject_Add(GTest
    GIT_REPOSITORY    https://github.com/google/googletest.git
    GIT_TAG           release-1.8.1
    SOURCE_DIR        ${CMAKE_BINARY_DIR}/../${PROJECT_NAME}-googletest-src
    BINARY_DIR        ${CMAKE_BINARY_DIR}/../${PROJECT_NAME}-googletest-build
    CMAKE_CACHE_ARGS
            -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DBUILD_GMOCK:BOOL=OFF
            -DBUILD_GTEST:BOOL=ON
            -DBUILD_SHARED_LIBS:BOOL=ON
  )

  file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/include)
  set(GTEST_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/include)
  set(GTEST_LIBRARIES ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/lib/libgtest.so)
  set(GTEST_MAIN_LIBRARIES ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/lib/libgtest_main.so)
endif()

if(NOT TARGET GTest::GTest)
  find_package(Threads QUIET)

  add_library(GTest::GTest INTERFACE IMPORTED)
  set_target_properties(GTest::GTest PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  
  if(TARGET Threads::Threads)
      set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_LIBRARIES};Threads::Threads")
  else()
    set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_LIBRARIES}")
  endif()
endif()

if(NOT TARGET GTest::Main)
  add_library(GTest::Main INTERFACE IMPORTED)
  set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_MAIN_LIBRARIES};GTest::GTest")
endif()

add_library(${PROJECT_NAME}_robot SHARED descartes_ikfast_fanuc_m20ia10l_manipulator.cpp)
target_link_libraries(${PROJECT_NAME}_robot PUBLIC ${PROJECT_NAME})
target_cxx_version(${PROJECT_NAME}_robot PUBLIC VERSION 11)
descartes_target_compile_options(${PROJECT_NAME}_robot PUBLIC)
target_compile_definitions(${PROJECT_NAME}_robot PUBLIC -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -DIKFAST_HAS_LIBRARY)
if(NOT MSVC)
  target_clang_tidy(${PROJECT_NAME}_robot ENABLE ${DESCARTES_ENABLE_TESTING} ARGUMENTS ${CLANG_ARGS})
  target_code_coverage(${PROJECT_NAME}_robot ALL ENABLE ${DESCARTES_ENABLE_TESTING})
endif()
target_include_directories(${PROJECT_NAME}_robot PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<INSTALL_INTERFACE:include>")
add_dependencies(${PROJECT_NAME}_robot ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_unit descartes_ikfast_unit.cpp)
target_link_libraries(${PROJECT_NAME}_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME}_robot)
target_cxx_version(${PROJECT_NAME}_unit PRIVATE VERSION 11)
descartes_target_compile_options(${PROJECT_NAME}_unit PRIVATE)
if(NOT MSVC)
  target_clang_tidy(${PROJECT_NAME}_unit ENABLE ${DESCARTES_ENABLE_TESTING} ARGUMENTS ${CLANG_ARGS})
  target_code_coverage(${PROJECT_NAME}_unit ALL ENABLE ${DESCARTES_ENABLE_TESTING})
endif()
target_include_directories(${PROJECT_NAME}_unit PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<INSTALL_INTERFACE:include>")
add_gtest_discover_tests(${PROJECT_NAME}_unit)
add_dependencies(${PROJECT_NAME}_unit ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_unit ${PROJECT_NAME}_robot)
add_dependencies(run_tests ${PROJECT_NAME}_unit)
if ( NOT ${GTest_FOUND} )
  add_dependencies(${PROJECT_NAME}_unit GTest)
endif()
