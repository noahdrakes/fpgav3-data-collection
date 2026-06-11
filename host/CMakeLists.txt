#
# (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required(VERSION 3.10)

# Set the project name
project(dvrkDataCollection-ALL)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Shared header file (with Zynq)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../shared)

set(UNIT_CONVERT_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../unit_convert/unit_convert_zynq/lib")
set(NLOHMANN_JSON_DIR "${UNIT_CONVERT_LIB_DIR}/json")
set(NLOHMANN_JSON_INCLUDE_DIR "${NLOHMANN_JSON_DIR}/single_include")

if(NOT EXISTS "${NLOHMANN_JSON_DIR}")
  find_package(Git REQUIRED)
  message(STATUS "Cloning nlohmann/json into ${NLOHMANN_JSON_DIR}")
  execute_process(
    COMMAND ${GIT_EXECUTABLE} clone --depth 1 https://github.com/nlohmann/json.git
    WORKING_DIRECTORY "${UNIT_CONVERT_LIB_DIR}"
    RESULT_VARIABLE NLOHMANN_JSON_CLONE_RESULT
  )
  if(NOT NLOHMANN_JSON_CLONE_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to clone nlohmann/json into ${NLOHMANN_JSON_DIR}")
  endif()
endif()

if(NOT EXISTS "${NLOHMANN_JSON_INCLUDE_DIR}/nlohmann/json.hpp")
  message(FATAL_ERROR "nlohmann/json.hpp was not found under ${NLOHMANN_JSON_INCLUDE_DIR}")
endif()

# Data collection library
add_subdirectory(lib/code)

# Data collection application
add_subdirectory(src)
