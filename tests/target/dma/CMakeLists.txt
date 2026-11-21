# The DMA bench apps of the ESP32 port (docs/assurance/bench/): ESP-IDF applications that print one line per bench
# item, `KB-BENCH <item> PASS|FAIL key=value ...`. Built in CI for esp32c6, esp32s3 and esp32 (ci-cmake-matrix.yml),
# run by hand on a devkit with the wiring of docs/assurance/bench/procedure-esp32.md:
#   idf.py set-target esp32c6
#   idf.py -DAPP_TYPE=dma_spi_bench -DAPP_SOURCE_FILE=dma_spi_bench.cpp build flash monitor
# Apps: dma_spi_bench (SPI staging, caller buffers, zero-copy, late completion, no allocation after start, DMA
# channel exhaustion), dma_adc_bench (ADC continuous into the kb stream: rate, blocked consumer, IRAM-safe NVS writes,
# stall supervision, shape fallback).
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The port-esp32 authors
cmake_minimum_required(VERSION 3.16)

set(DEFAULT_APP_TYPE "dma_spi_bench" CACHE STRING "Default app for the requirements phase")
set(DEFAULT_BUILD_TYPE "Release" CACHE STRING "Default build type for the requirements phase")
set(DEFAULT_SOURCE_FILE "dma_spi_bench.cpp" CACHE STRING "Default source for the requirements phase")
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
message(STATUS "Building bench app: ${APP_TYPE}, build type: ${BUILD_TYPE}")

if(NOT DEFINED SDKCONFIG)
  set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")
endif()
# The ADC app needs the IRAM-safe ADC interrupt (it writes NVS while streaming).
if(APP_TYPE STREQUAL "dma_adc_bench")
  set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults;${CMAKE_CURRENT_LIST_DIR}/sdkconfig.adc")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(kb_port_esp32_${APP_TYPE})
