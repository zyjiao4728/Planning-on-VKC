#
# Copyright (C) 2018 by George Cave - gcave@stablecoder.ca
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

set(DEFAULT_CPPCHECK_ARGS "--enable=warning,performance,portability,missingInclude;--template=\"[{severity}][{id}] {message} {callstack} \(On {file}:{line}\)\";--suppress=missingIncludeSystem;--quiet;--verbose;--force;--inline-suppr")
mark_as_advanced(DEFAULT_CPPCHECK_ARGS)

set(DEFAULT_IWYU_ARGS "-Xiwyu;any;-Xiwyu;iwyu;-Xiwyu;args")
mark_as_advanced(DEFAULT_IWYU_ARGS)

set(DEFAULT_CLANG_TIDY_WARNING_ARGS "-header-filter=.*" "-checks=-*,bugprone-*,cppcoreguidelines-avoid-goto,cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-narrowing-conversions,cppcoreguidelines-no-malloc,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions,misc-*,modernize-*,performance-*,readability-avoid-const-params-in-decls,readability-container-size-empty,readability-delete-null-pointer,readability-deleted-default,readability-else-after-return,readability-function-size,readability-identifier-naming,readability-inconsistent-declaration-parameter-name,readability-misleading-indentation,readability-misplaced-array-index,readability-non-const-parameter,readability-redundant-*,readability-simplify-*,readability-static-*,readability-string-compare,readability-uniqueptr-delete-release,readability-rary-objects")
mark_as_advanced(DEFAULT_CLANG_TIDY_WARNING_ARGS )

set(DEFAULT_CLANG_TIDY_ERROR_ARGS "-header-filter=.*" "-checks=-*,bugprone-*,cppcoreguidelines-avoid-goto,cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-narrowing-conversions,cppcoreguidelines-no-malloc,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions,misc-*,modernize-*,performance-*,readability-avoid-const-params-in-decls,readability-container-size-empty,readability-delete-null-pointer,readability-deleted-default,readability-else-after-return,readability-function-size,readability-identifier-naming,readability-inconsistent-declaration-parameter-name,readability-misleading-indentation,readability-misplaced-array-index,readability-non-const-parameter,readability-redundant-*,readability-simplify-*,readability-static-*,readability-string-compare,readability-uniqueptr-delete-release,readability-rary-objects" "-warnings-as-errors=-*,bugprone-*,cppcoreguidelines-avoid-goto,cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-narrowing-conversions,cppcoreguidelines-no-malloc,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions,misc-*,modernize-*,performance-*,readability-avoid-const-params-in-decls,readability-container-size-empty,readability-delete-null-pointer,readability-deleted-default,readability-else-after-return,readability-function-size,readability-identifier-naming,readability-inconsistent-declaration-parameter-name,readability-misleading-indentation,readability-misplaced-array-index,readability-non-const-parameter,readability-redundant-*,readability-simplify-*,readability-static-*,readability-string-compare,readability-uniqueptr-delete-release,readability-rary-objects")
mark_as_advanced(DEFAULT_CLANG_TIDY_ERROR_ARGS)

# Adds clang-tidy checks to the target, with the given arguments being used
# as the options set.
macro(target_clang_tidy target)
  set(oneValueArgs ENABLE)
  set(multiValueArgs ARGUMENTS)
  cmake_parse_arguments(ARG "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if((NOT DEFINED ARG_ENABLE) OR (ARG_ENABLE))
    if(CLANG_TIDY_EXE)
      get_target_property(${target}_type ${target} TYPE)
      if(NOT ${${target}_type} STREQUAL "INTERFACE_LIBRARY")
        if(ARG_ARGUMENTS)
          set_target_properties("${target}" PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY_EXE};${ARG_ARGUMENTS}")
        else()
          set_target_properties("${target}" PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY_EXE};${DEFAULT_CLANG_TIDY_WARNING_ARGS}")
        endif()
      endif()
    else()
      message(WARNING "Using target_clang_tidy but clang tidy executable was not found!")
    endif()
  endif()
endmacro()

# Adds include_what_you_use to the target, with the given arguments being
# used as the options set.
macro(target_include_what_you_use target)
  set(oneValueArgs ENABLE)
  set(multiValueArgs ARGUMENTS)
  cmake_parse_arguments(ARG "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if((NOT DEFINED ARG_ENABLE) OR (ARG_ENABLE))
    if(IWYU_EXE)
      if(ARG_ARGUMENTS)
        set_target_properties("${target}" PROPERTIES CXX_INCLUDE_WHAT_YOU_USE "${IWYU_EXE};${ARG_ARGUMENTS}")
      else()
        set_target_properties("${target}" PROPERTIES CXX_INCLUDE_WHAT_YOU_USE "${IWYU_EXE};${DEFAULT_IWYU_ARGS}")
      endif()
    else()
      message(WARNING "Using target_include_what_you_use but iwyu executable was not found!")
    endif()
  endif()
endmacro()

# Adds include_what_you_use to all targets, with the given arguments being used as the options set.
macro(include_what_you_use)
  set(oneValueArgs ENABLE)
  set(multiValueArgs ARGUMENTS)
  cmake_parse_arguments(ARG "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if((NOT DEFINED ARG_ENABLE) OR (ARG_ENABLE))
    if(IWYU_EXE)
      if(ARG_ARGUMENTS)
        set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE "${IWYU_EXE};${ARG_ARGUMENTS}")
      else()
        set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE "${IWYU_EXE};${DEFAULT_IWYU_ARGS}")
      endif()
    else()
      message(WARNING "Using include_what_you_use but iwyu executable was not found!")
    endif()
  endif()
endmacro()

# Adds cppcheck to the target, with the given arguments being used as the options set.
macro(target_cppcheck target)
  set(oneValueArgs ENABLE)
  set(multiValueArgs ARGUMENTS)
  cmake_parse_arguments(ARG "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if((NOT DEFINED ARG_ENABLE) OR (ARG_ENABLE))
    if(CPPCHECK_EXE)
      if(ARG_ARGUMENTS)
        set_target_properties("${target}" PROPERTIES CXX_CPPCHECK "${CPPCHECK_EXE};${ARG_ARGUMENTS}")
      else()
        set_target_properties("${target}" PROPERTIES CXX_CPPCHECK "${CPPCHECK_EXE};${DEFAULT_CPPCHECK_ARGS}")
      endif()
    else()
      message(WARNING "Using target_cppcheck but cppcheck executable was not found!")
    endif()
  endif()
endmacro()

# Adds cppcheck to all targets, with the given arguments being used as the options set.
macro(cppcheck)
  set(oneValueArgs ENABLE)
  set(multiValueArgs ARGUMENTS)
  cmake_parse_arguments(ARG "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if((NOT DEFINED ARG_ENABLE) OR (ARG_ENABLE))
    if(CPPCHECK_EXE)
      if(ARG_ARGUMENTS)
        set(CMAKE_CXX_CPPCHECK "${CPPCHECK_EXE};${ARG_ARGUMENTS}")
      else()
        set(CMAKE_CXX_CPPCHECK "${CPPCHECK_EXE};${DEFAULT_CPPCHECK_ARGS}")
      endif()
    else()
      message(WARNING "Using cppcheck but cppcheck executable was not found!")
    endif()
  endif()
endmacro()

# Performs multiple operation so other packages may find a package
# Usage: configure_package(NAMSPACE namespace TARGETS targetA targetb)
#   * It installs the provided targets
#   * It exports the provided targets under the provided namespace
#   * It installs the package.xml file
#   * It create and install the ${PROJECT_NAME}-config.cmake and ${PROJECT_NAME}-config-version.cmake
macro(configure_package)
  set(oneValueArgs NAMESPACE)
  set(multiValueArgs TARGETS)
  cmake_parse_arguments(ARG "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if (ARG_TARGETS)
    install(TARGETS ${ARG_TARGETS}
            EXPORT ${PROJECT_NAME}-targets
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
    if (ARG_NAMESPACE)
      install(EXPORT ${PROJECT_NAME}-targets NAMESPACE "${ARG_NAMESPACE}::" DESTINATION lib/cmake/${PROJECT_NAME})
    else()
      install(EXPORT ${PROJECT_NAME}-targets DESTINATION lib/cmake/${PROJECT_NAME})
    endif()
  endif()

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

  if (ARG_TARGETS)
    if (ARG_NAMESPACE)
      export(EXPORT ${PROJECT_NAME}-targets NAMESPACE "${ARG_NAMESPACE}::" FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
    else()
      export(EXPORT ${PROJECT_NAME}-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
    endif()
  endif()
endmacro()

# This macro call the appropriate gtest function to add a test based on the cmake version
# Usage: add_gtest_discover_tests(target)
macro(add_gtest_discover_tests target)
  if(${CMAKE_VERSION} VERSION_LESS "3.10.0")
    gtest_add_tests(${target} "" AUTO)
  else()
    gtest_discover_tests(${target})
  endif()
endmacro()

# This macro add a custom target that will run the tests after they are finished building when
# This is added to allow ability do disable the running of tests as part of the build for CI which calls make test
#    * add_run_tests_target() adds run test target
#    * add_run_tests_target(ENABLE ON/TRUE) adds run test target
#    * add_run_tests_target(ENABLE OFF/FALSE) adds empty run test target
macro(add_run_tests_target)
  set(oneValueArgs ENABLE)
  cmake_parse_arguments(ARG "" "${oneValueArgs}" "" ${ARGN})

  if((NOT DEFINED ARG_ENABLE) OR (ARG_ENABLE))
    add_custom_target(run_tests ALL
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMAND ${CMAKE_CTEST_COMMAND} -V -O "/tmp/${PROJECT_NAME}_ctest.log" -C $<CONFIGURATION>)
  else()
    add_custom_target(run_tests)
  endif()
endmacro()

# This macro add a custom target that will run the benchmarks after they are finished building.
# Usage: add_run_benchmark_target(benchmark_name)
# Results are saved to /test/benchmarks/${benchmark_name}_results.json in the build directory
#    * add_run_benchmark_target(benchmark_name) adds run benchmark target
#    * add_run_benchmark_target(benchmark_name ENABLE ON/TRUE) adds run benchmark target
#    * add_run_benchmark_target(benchmark_name ENABLE OFF/FALSE) adds empty run benchmark target
macro(add_run_benchmark_target benchmark_name)
  set(oneValueArgs ENABLE)
  cmake_parse_arguments(ARG "" "${oneValueArgs}" "" ${ARGN})

  if((NOT DEFINED ARG_ENABLE) OR (ARG_ENABLE))
    add_custom_target(run_benchmark_${benchmark_name} ALL
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMAND ./test/benchmarks/${benchmark_name} --benchmark_out_format=json --benchmark_out="./test/benchmarks/${benchmark_name}_results.json")
  else()
    add_custom_target(run_benchmark_${benchmark_name})
  endif()
  add_dependencies(run_benchmark_${benchmark_name} ${benchmark_name})
endmacro()

# These macros facilitate setting cxx version for a target
macro(target_cxx_version target)
  set(options INTERFACE PUBLIC PRIVATE)
  set(oneValueArgs VERSION)
  cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "" ${ARGN})
  list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_${ARG_VERSION} CXX_FEATURE_FOUND)
  if (ARG_INTERFACE)
    if(CXX_FEATURE_FOUND EQUAL "-1")
      target_compile_options("${target}" INTERFACE -std=c++${ARG_VERSION})
    else()
      target_compile_features("${target}" INTERFACE cxx_std_${ARG_VERSION})
    endif()
  elseif(ARG_PUBLIC)
    if(CXX_FEATURE_FOUND EQUAL "-1")
      set_property(TARGET ${target} PROPERTY CXX_STANDARD ${ARG_VERSION})
    else()
      target_compile_features("${target}" PUBLIC cxx_std_${ARG_VERSION})
    endif()
  elseif(ARG_PRIVATE)
    if(CXX_FEATURE_FOUND EQUAL "-1")
      set_property(TARGET ${target} PROPERTY CXX_STANDARD ${ARG_VERSION})
    else()
      target_compile_features("${target}" PRIVATE cxx_std_${ARG_VERSION})
    endif()
  else()
    message(FATAL_ERROR "target_cxx_version: Must provide keywork INTERFACE | PRIVATE | PUBLIC")
  endif()
endmacro()

# Find relevant programs
find_program(CLANG_TIDY_EXE NAMES "clang-tidy")
mark_as_advanced(FORCE CLANG_TIDY_EXE)
if(CLANG_TIDY_EXE)
  message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
else()
  message(STATUS "clang-tidy not found!")
  set(CMAKE_CXX_CLANG_TIDY "" CACHE STRING "" FORCE) # delete it
endif()

find_program(IWYU_EXE NAMES "include-what-you-use")
mark_as_advanced(FORCE IWYU_EXE)
if(IWYU_EXE)
  message(STATUS "include-what-you-use found: ${IWYU_EXE}")
else()
  message(STATUS "include-what-you-use not found!")
  set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE "" CACHE STRING "" FORCE) # delete it
endif()

find_program(CPPCHECK_EXE NAMES "cppcheck")
mark_as_advanced(FORCE CPPCHECK_EXE)
if(CPPCHECK_EXE)
  message(STATUS "cppcheck found: ${CPPCHECK_EXE}")
else()
  message(STATUS "cppcheck not found!")
  set(CMAKE_CXX_CPPCHECK "" CACHE STRING "" FORCE) # delete it
endif()
