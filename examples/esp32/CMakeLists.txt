# ESP-IDF example applications of the ESP32 port, one project for every ESP32 target (idf.py set-target <target>).
# The app comes from app_config.yml (APP_TYPE, APP_SOURCE_FILE), through the ESP-IDF build scripts
# (kokeb-labs/espidf-tools, cloned into scripts/ by CI) or by hand:
#   idf.py set-target esp32s3
#   idf.py -DAPP_TYPE=port_selftest -DAPP_SOURCE_FILE=port_selftest.cpp -DBUILD_TYPE=Release build
# Components: kb (the pinned kb-core copy), kb_port_esp32 (this repository).
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The port-esp32 authors
cmake_minimum_required(VERSION 3.16)

set(DEFAULT_APP_TYPE "port_selftest" CACHE STRING "Default app for the requirements phase")
set(DEFAULT_BUILD_TYPE "Release" CACHE STRING "Default build type for the requirements phase")
set(DEFAULT_SOURCE_FILE "port_selftest.cpp" CACHE STRING "Default source for the requirements phase")
if(NOT DEFINED APP_TYPE)
  set(APP_TYPE "${DEFAULT_APP_TYPE}")
endif()
if(NOT DEFINED BUILD_TYPE)
  set(BUILD_TYPE "${DEFAULT_BUILD_TYPE}")
endif()
if(NOT DEFINED APP_SOURCE_FILE)
  set(APP_SOURCE_FILE "${DEFAULT_SOURCE_FILE}")
endif()
set(APP_TYPE "${APP_TYPE}" CACHE STRING "App type to build" FORCE)
set(BUILD_TYPE "${BUILD_TYPE}" CACHE STRING "Build type (Debug/Release)" FORCE)
set(APP_SOURCE_FILE "${APP_SOURCE_FILE}" CACHE STRING "Source of the app" FORCE)
message(STATUS "Building app type: ${APP_TYPE}, build type: ${BUILD_TYPE}")

# The generated sdkconfig goes to the build folder, so one checkout builds every target without a clean.
if(NOT DEFINED SDKCONFIG)
  set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(kb_port_esp32_${APP_TYPE})
