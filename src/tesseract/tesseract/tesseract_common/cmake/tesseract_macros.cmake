#
# @file tesseract_macros.cmae
# @brief Common Tesseract CMake Macros
#
# @author Levi Armstrong
# @date October 15, 2019
# @version TODO
# @bug No known bugs
#
# @copyright Copyright (c) 2019, Southwest Research Institute
#
# @par License
# Software License Agreement (Apache License)
# @par
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# @par
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# This macro will add tesseract standard compiler option to a target
# Usage: tesseract_target_compile_options(target <INTERFACE|PUBLIC|PRIVATE>)
#    * c++14
#    * Warning (-Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
#    * disable avx due to eigen issues
#    * Add Clang Tidy
macro(tesseract_target_compile_options target)
  cmake_parse_arguments(ARG "INTERFACE;PUBLIC;PRIVATE" "" "" ${ARGN})

  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "tesseract_target_compile_options() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_14 CXX_FEATURE_FOUND)
  if (NOT TESSERACT_ENABLE_TESTING)
    set(warning_flags -Wall -Wextra -Wconversion -Wsign-conversion -Wno-sign-compare)
  else()
    set(warning_flags -Werror=all -Werror=extra -Werror=conversion -Werror=sign-conversion -Wno-sign-compare)
  endif()

  if (ARG_INTERFACE)
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options("${target}" INTERFACE -std=c++14 -mno-avx ${warning_flags})
      else()
        target_compile_features("${target}" INTERFACE cxx_std_14)
        target_compile_options("${target}" INTERFACE -mno-avx ${warning_flags})
      endif()
    else()
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    endif()
  elseif(ARG_PUBLIC)
    target_compile_options("${target}" PRIVATE ${warning_flags})

    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options("${target}" PUBLIC -std=c++14 -mno-avx)
      else()
        target_compile_features("${target}" PUBLIC cxx_std_14)
        target_compile_options("${target}" PUBLIC -mno-avx)
      endif()
    else()
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    endif()
  elseif(ARG_PRIVATE)
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options("${target}" PRIVATE -std=c++14 -mno-avx ${warning_flags})
      else()
        target_compile_features("${target}" PRIVATE cxx_std_14)
        target_compile_options("${target}" PRIVATE -mno-avx ${warning_flags})
      endif()
    else()
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    endif()
  endif()
endmacro()

# Add clang-tidy to a target if TESSERACT_ENABLE_CLANG_TIDY or TESSERACT_ENABLE_TESTING is enabled
# Usage: tesseract_clang_tidy(Target) or tesseract_clang_tidy(Target true) or tesseract_clang_tidy(Target false)
#    * tesseract_clang_tidy(Target) adds clang tidy with warnings as errors
#    * tesseract_clang_tidy(Target true) adds clang tidy with warnings as errors
#    * tesseract_clang_tidy(Target false) adds clang tidy with warnings as warnings
macro(tesseract_clang_tidy target)
  cmake_parse_arguments(ARG "true;false" "" "" ${ARGN})

  get_target_property(${target}_type ${target} TYPE)

  # Add clang tidy
  if (NOT ${${target}_type} STREQUAL "INTERFACE_LIBRARY")
    if (TESSERACT_ENABLE_CLANG_TIDY OR TESSERACT_ENABLE_TESTING)
      find_program(CLANG_TIDY_EXE NAMES "clang-tidy" DOC "Path to clang-tidy executable")
      if(NOT CLANG_TIDY_EXE)
        message(WARNING "clang-tidy not found.")
      else()
        message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
        if(ARG_false)
          set(DO_CLANG_TIDY "${CLANG_TIDY_EXE}" "-header-filter=.*" "-line-filter=[{'name':'EnvironmentMonitorDynamicReconfigureConfig.h','lines':[[9999999,9999999]]}, {'name':'.h'}, {'name':'.hpp'}]" "-checks=-*,bugprone-*,cppcoreguidelines-avoid-goto,cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-narrowing-conversions,cppcoreguidelines-no-malloc,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions,misc-*,modernize-*,performance-*,readability-avoid-const-params-in-decls,readability-container-size-empty,readability-delete-null-pointer,readability-deleted-default,readability-else-after-return,readability-function-size,readability-identifier-naming,readability-inconsistent-declaration-parameter-name,readability-misleading-indentation,readability-misplaced-array-index,readability-non-const-parameter,readability-redundant-*,readability-simplify-*,readability-static-*,readability-string-compare,readability-uniqueptr-delete-release,readability-rary-objects")
        else()
          set(DO_CLANG_TIDY "${CLANG_TIDY_EXE}" "-header-filter=.*" "-line-filter=[{'name':'EnvironmentMonitorDynamicReconfigureConfig.h','lines':[[9999999,9999999]]}, {'name':'.h'}, {'name':'.hpp'}]" "-checks=-*,bugprone-*,cppcoreguidelines-avoid-goto,cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-narrowing-conversions,cppcoreguidelines-no-malloc,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions,misc-*,modernize-*,performance-*,readability-avoid-const-params-in-decls,readability-container-size-empty,readability-delete-null-pointer,readability-deleted-default,readability-else-after-return,readability-function-size,readability-identifier-naming,readability-inconsistent-declaration-parameter-name,readability-misleading-indentation,readability-misplaced-array-index,readability-non-const-parameter,readability-redundant-*,readability-simplify-*,readability-static-*,readability-string-compare,readability-uniqueptr-delete-release,readability-rary-objects" "-warnings-as-errors=-*,bugprone-*,cppcoreguidelines-avoid-goto,cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-narrowing-conversions,cppcoreguidelines-no-malloc,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions,misc-*,modernize-*,performance-*,readability-avoid-const-params-in-decls,readability-container-size-empty,readability-delete-null-pointer,readability-deleted-default,readability-else-after-return,readability-function-size,readability-identifier-naming,readability-inconsistent-declaration-parameter-name,readability-misleading-indentation,readability-misplaced-array-index,readability-non-const-parameter,readability-redundant-*,readability-simplify-*,readability-static-*,readability-string-compare,readability-uniqueptr-delete-release,readability-rary-objects")
        endif()
        set_target_properties("${target}" PROPERTIES CXX_CLANG_TIDY "${DO_CLANG_TIDY}")
      endif()
    endif()
  endif()
endmacro()

# Performs multiple operation so other packages may find a package
# Usage: tesseract_configure_package(targetA targetb)
#   * It installs the provided targets
#   * It exports the provided targets under the namespace tesseract::
#   * It installs the package.xml file
#   * It create and install the ${PROJECT_NAME}-config.cmake and ${PROJECT_NAME}-config-version.cmake
macro(tesseract_configure_package)
  install(TARGETS ${ARGV}
          EXPORT ${PROJECT_NAME}-targets
          RUNTIME DESTINATION bin
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib)
  install(EXPORT ${PROJECT_NAME}-targets NAMESPACE tesseract:: DESTINATION lib/cmake/${PROJECT_NAME})

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

  export(EXPORT ${PROJECT_NAME}-targets NAMESPACE tesseract:: FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
endmacro()

# This macro call the appropriate gtest function to add a test based on the cmake version
# Usage: tesseract_gtest_discover_tests(target)
macro(tesseract_gtest_discover_tests target)
  if(${CMAKE_VERSION} VERSION_LESS "3.10.0")
    gtest_add_tests(${target} "" AUTO)
  else()
    gtest_discover_tests(${target})
  endif()
endmacro()

# This macro add a custom target that will run the tests after they are finished building.
# This is added to allow ability do disable the running of tests as part of the build for CI which calls make test
macro(tesseract_add_run_tests_target)
  if(TESSERACT_ENABLE_RUN_TESTING)
    add_custom_target(run_tests ALL
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMAND ${CMAKE_CTEST_COMMAND} -V -O "/tmp/${PROJECT_NAME}_ctest.log" -C $<CONFIGURATION>)
  else()
    add_custom_target(run_tests)
  endif()
endmacro()

#http://www.stablecoder.ca/2018/01/15/code-coverage.html
#code coverage
#if(CMAKE_BUILD_TYPE STREQUAL "coverage" OR CODE_COVERAGE)
#    if("${CMAKE_C_COMPILER_ID}" MATCHES "(Apple)?[Cc]lang" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "(Apple)?[Cc]lang")
#        message("Building with llvm Code Coverage Tools")

#        # Warning/Error messages
#        if(NOT LLVM_COV_PATH)
#            message(FATAL_ERROR "llvm-cov not found! Aborting.")
#        endif()

#        # set Flags
#        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-instr-generate -fcoverage-mapping")
#        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-instr-generate -fcoverage-mapping")

#    elseif(CMAKE_COMPILER_IS_GNUCXX)
#        message("Building with lcov Code Coverage Tools")

#        # Warning/Error messages
#        if(NOT (CMAKE_BUILD_TYPE STREQUAL "Debug"))
#            message(WARNING "Code coverage results with an optimized (non-Debug) build may be misleading")
#        endif()
#        if(NOT LCOV_PATH)
#            message(FATAL_ERROR "lcov not found! Aborting...")
#        endif()
#        if(NOT GENHTML_PATH)
#            message(FATAL_ERROR "genhtml not found! Aborting...")
#        endif()

#        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
#        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
#    else()
#        message(FATAL_ERROR "Code coverage requires Clang or GCC. Aborting.")
#    endif()
#endif()


#generact code coverage report
## llvm-cov
#add_custom_target(${TARGET_NAME}-ccov-preprocessing
#    COMMAND LLVM_PROFILE_FILE=${TARGET_NAME}.profraw $<TARGET_FILE:${TARGET_NAME}>
#    COMMAND llvm-profdata merge -sparse ${TARGET_NAME}.profraw -o ${TARGET_NAME}.profdata
#    DEPENDS ${TARGET_NAME})

#add_custom_target(${TARGET_NAME}-ccov-show
#    COMMAND llvm-cov show $<TARGET_FILE:${TARGET_NAME}> -instr-profile=${TARGET_NAME}.profdata -show-line-counts-or-regions
#    DEPENDS ${TARGET_NAME}-ccov-preprocessing)

#add_custom_target(${TARGET_NAME}-ccov-report
#    COMMAND llvm-cov report $<TARGET_FILE:${TARGET_NAME}> -instr-profile=${TARGET_NAME}.profdata
#    DEPENDS ${TARGET_NAME}-ccov-preprocessing)

#add_custom_target(${TARGET_NAME}-ccov
#    COMMAND llvm-cov show $<TARGET_FILE:${TARGET_NAME}> -instr-profile=${TARGET_NAME}.profdata -show-line-counts-or-regions -output-dir=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET_NAME}-llvm-cov -format="html"
#    DEPENDS ${TARGET_NAME}-ccov-preprocessing)

#add_custom_command(TARGET ${TARGET_NAME}-ccov POST_BUILD
#    COMMAND ;
#    COMMENT "Open ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET_NAME}-llvm-cov/index.html in your browser to view the coverage report."
#)
