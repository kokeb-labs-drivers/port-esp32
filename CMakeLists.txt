# kb_port_esp32: kb-core's contracts over ESP-IDF v5.5, for every ESP32 target (ADR-0007, ADR-0008 of kb-core).
#   Under ESP-IDF: the component kb_port_esp32 (examples/components/kb_port_esp32 forwards here), requiring the
#   component kb (kb-core).
#   Off target:    cmake -S . -B build -DKB_PORT_ESP32_BUILD_TESTS=ON   host tests of the logic and the boundary
#                  files against fakes of every ESP-IDF function they call (tests/fakes/esp-idf).
# kb-core comes from find_package(kb) when it is installed, else from the pinned copy in third_party/kb-core
# (tools/kb_core_snapshot.py; every file is checked against third_party/kb-core.lock at configure time).
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The port-esp32 authors
cmake_minimum_required(VERSION 3.25)

# One source per public header (src/<stem>.cpp; tools/check_layout.py checks it), plus the internal claim tables.
set(KB_PORT_ESP32_LOGIC_SOURCES
    src/adc.cpp src/caller_record.cpp src/can.cpp src/claims.cpp src/gpio.cpp src/i2c.cpp src/log.cpp src/port.cpp
    src/pwm.cpp src/spi.cpp src/time.cpp src/uart.cpp)
# The only files that include ESP-IDF headers: src/boundary/, one file per SDK component
# (deviations/boundary-files.yml).
set(KB_PORT_ESP32_BOUNDARY_SOURCES
    src/boundary/adc.cpp src/boundary/error.cpp src/boundary/gpio.cpp src/boundary/i2c.cpp src/boundary/log.cpp
    src/boundary/pwm.cpp src/boundary/spi.cpp src/boundary/time.cpp src/boundary/uart.cpp)
# CAN: the TWAI boundary on chips that have the controller, else the stub that reports Unsupported.
set(KB_PORT_ESP32_CAN_SOURCE src/boundary/can.cpp)
set(KB_PORT_ESP32_NO_CAN_SOURCE src/boundary/no_can.cpp)

if(ESP_PLATFORM)
  if(CONFIG_SOC_TWAI_SUPPORTED)
    list(APPEND KB_PORT_ESP32_BOUNDARY_SOURCES ${KB_PORT_ESP32_CAN_SOURCE})
  else()
    list(APPEND KB_PORT_ESP32_BOUNDARY_SOURCES ${KB_PORT_ESP32_NO_CAN_SOURCE})
  endif()
  # Absolute paths: the component is registered through examples/components/kb_port_esp32, which includes this
  # file from another folder.
  list(TRANSFORM KB_PORT_ESP32_LOGIC_SOURCES PREPEND "${CMAKE_CURRENT_LIST_DIR}/")
  list(TRANSFORM KB_PORT_ESP32_BOUNDARY_SOURCES PREPEND "${CMAKE_CURRENT_LIST_DIR}/")
  idf_component_register(SRCS ${KB_PORT_ESP32_LOGIC_SOURCES} ${KB_PORT_ESP32_BOUNDARY_SOURCES}
                         INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/inc"
                         PRIV_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/src"
                         REQUIRES kb
                         PRIV_REQUIRES driver esp_adc esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_driver_spi
                                       esp_driver_uart esp_hw_support esp_mm hal esp_rom esp_timer freertos log)
  target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_20)
  # The logic is ISO C++20; the boundary files include ESP-IDF headers, which use GNU extensions (anonymous
  # structs in unions, statement expressions), so they build as gnu++20 (deviations/README.md).
  set_source_files_properties(${KB_PORT_ESP32_LOGIC_SOURCES} PROPERTIES COMPILE_OPTIONS
                              "-std=c++20;-fno-exceptions;-fno-rtti;-Wconversion;-Wsign-conversion")
  set_source_files_properties(${KB_PORT_ESP32_BOUNDARY_SOURCES} PROPERTIES COMPILE_OPTIONS
                              "-std=gnu++20;-fno-exceptions;-fno-rtti")
  # The conversion warnings are on for the logic only: ESP-IDF's own headers do not build cleanly with them (the
  # host build of the boundary files over the fakes has every warning of kb-core's strict set).
  target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Wshadow -Werror)
  return()
endif()

project(kb_port_esp32 VERSION 0.1.0 DESCRIPTION "kb-core contracts over ESP-IDF v5.5" LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(KB_PORT_ESP32_BUILD_TESTS "Build the host tests against the ESP-IDF fakes" OFF)
include("${CMAKE_CURRENT_LIST_DIR}/cmake/kb_port_esp32_kb_core.cmake")

# The public headers: platform-free, no SDK type in a signature. Off target this is all a consumer gets (for
# static checks and header builds); the sources need ESP-IDF or the test fakes.
add_library(kb_port_esp32_api INTERFACE)
add_library(kb::port_esp32 ALIAS kb_port_esp32_api)
target_compile_features(kb_port_esp32_api INTERFACE cxx_std_20)
target_include_directories(kb_port_esp32_api INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>")
target_link_libraries(kb_port_esp32_api INTERFACE kb::interfaces kb::rtos kb::utils)

# The platform-free logic alone (no SDK header anywhere): what the poison build of the layering check and the
# Cortex-M cross build compile, with kb-core's strict flags.
list(TRANSFORM KB_PORT_ESP32_LOGIC_SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/" OUTPUT_VARIABLE _kb_port_logic)
add_library(kb_port_esp32_platform_free OBJECT EXCLUDE_FROM_ALL ${_kb_port_logic})
target_compile_options(kb_port_esp32_platform_free PRIVATE ${KB_PORT_ESP32_STRICT_FLAGS})
target_include_directories(kb_port_esp32_platform_free PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(kb_port_esp32_platform_free PRIVATE kb_port_esp32_api)

if(KB_PORT_ESP32_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
