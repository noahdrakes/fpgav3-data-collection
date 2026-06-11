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

cmake_minimum_required (VERSION 3.10)

project (DataCollectionZynq VERSION 0.1.0)

if (NOT (Arch STREQUAL "arm32"))
  message (FATAL_ERROR "This program must be cross-compiled for arm32 using mechatronics-embedded toolchain file")
endif ()

find_package (Amp1394 REQUIRED)

# Add some warnings
include (CheckCXXCompilerFlag)
check_cxx_compiler_flag ("-Wextra" CXX_SUPPORTS_WEXTRA)
if (CXX_SUPPORTS_WEXTRA)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra")
endif ()

check_cxx_compiler_flag ("-Wall" CXX_SUPPORTS_WALL)
if (CXX_SUPPORTS_WALL)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif ()

check_cxx_compiler_flag ("-fPIC" CXX_SUPPORTS_FPIC)
if (CXX_SUPPORTS_FPIC)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif ()

if (Amp1394_HAS_EMIO)

  include_directories(${Amp1394_INCLUDE_DIR})
  include_directories("../unit_convert/unit_convert_zynq/lib")
  include_directories("../shared")

  link_directories(${Amp1394_LIBRARY_DIR})

  set (ORT_ROOT "" CACHE PATH "Path to ONNX Runtime root directory")
  if (NOT ORT_ROOT)
    message (FATAL_ERROR "ORT_ROOT must be set to your ONNX Runtime root directory")
  endif ()

  # Declare the imported library target
  add_library(onnxruntime SHARED IMPORTED)
  set_target_properties(onnxruntime PROPERTIES
    IMPORTED_LOCATION ${ORT_ROOT}/build/Release/libonnxruntime.so
    INTERFACE_INCLUDE_DIRECTORIES ${ORT_ROOT}/include
  )

  add_executable(dvrk-data-collection-zynq
    dvrk-data-collection-zynq.cpp
    ../unit_convert/unit_convert_zynq/src/unit_conversion.cpp
    ../unit_convert/unit_convert_zynq/src/config_loader.cpp
  )
  target_compile_features(dvrk-data-collection-zynq PRIVATE cxx_std_17)

  target_compile_definitions(dvrk-data-collection-zynq PRIVATE
    __GLIBC_USE_C2X_STRTOL=0
  )

  target_link_libraries(dvrk-data-collection-zynq PRIVATE ${Amp1394_LIBRARIES} onnxruntime)

  set (JSON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../unit_convert/unit_convert_zynq/lib/json)
  if (NOT EXISTS ${JSON_DIR}/include/nlohmann/json.hpp)
    message (STATUS "nlohmann/json not found, fetching from GitHub...")
    execute_process (
      COMMAND git clone --depth 1 --branch v3.11.3
              https://github.com/nlohmann/json.git ${JSON_DIR}
      RESULT_VARIABLE GIT_RESULT
    )
    if (NOT GIT_RESULT EQUAL 0)
      message (FATAL_ERROR "Failed to fetch nlohmann/json from GitHub")
    endif ()
  endif ()

  target_include_directories(dvrk-data-collection-zynq PRIVATE
    ${JSON_DIR}/include
    ${ORT_ROOT}/include/onnxruntime/core/session
  )

  install (TARGETS dvrk-data-collection-zynq
           RUNTIME DESTINATION bin)

else ()

  message (FATAL_ERROR "Amp1394 library not cross-compiled with EMIO support")

endif ()
