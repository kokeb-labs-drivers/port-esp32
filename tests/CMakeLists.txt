# Host tests of the ESP32 port (tests/README.md): the logic and the boundary files built unchanged against the
# ESP-IDF fakes (a link seam per SDK call), in two chip variants; the concept and *Ref proofs; one translation unit
# per public header; and compile-fail tests of misuse.
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The port-esp32 authors
set(_harness "${KB_PORT_ESP32_KB_CORE_DIR}/tests/harness")
set(_fakes "${CMAKE_CURRENT_SOURCE_DIR}/fakes/esp-idf")
list(TRANSFORM KB_PORT_ESP32_LOGIC_SOURCES PREPEND "${PROJECT_SOURCE_DIR}/" OUTPUT_VARIABLE _logic)
list(TRANSFORM KB_PORT_ESP32_BOUNDARY_SOURCES PREPEND "${PROJECT_SOURCE_DIR}/" OUTPUT_VARIABLE _boundary)

# The port as it builds on target, but over the fakes. DEFINES selects a variant of the fake SDK (esp_idf_fake.h),
# which is built once per variant: test code, outside the strict flags (it implements C variadic logging).
function(kb_port_esp32_under_test name can_source)
  cmake_parse_arguments(PARSE_ARGV 2 ARG "" "" "DEFINES")
  add_library(${name}_fake_idf STATIC "${_fakes}/fake_esp_idf.cpp")
  target_include_directories(${name}_fake_idf PUBLIC "${_fakes}" "${_fakes}/include")
  target_compile_definitions(${name}_fake_idf PUBLIC ${ARG_DEFINES})
  target_compile_options(${name}_fake_idf PRIVATE -fno-exceptions -fno-rtti -Wall -Wextra -Wpedantic -Wshadow -Wundef -Werror)
  add_library(${name} STATIC ${_logic} ${_boundary} "${PROJECT_SOURCE_DIR}/${can_source}")
  target_compile_options(${name} PRIVATE ${KB_PORT_ESP32_STRICT_FLAGS})
  target_include_directories(${name} PRIVATE "${PROJECT_SOURCE_DIR}/src")
  # The fake SDK headers stand where ESP-IDF's would: -isystem, like the SDK on target (ADR-0011 of kb-core).
  target_include_directories(${name} SYSTEM PRIVATE "${_fakes}/include")
  target_compile_definitions(${name} PUBLIC ${ARG_DEFINES})
  target_link_libraries(${name} PUBLIC kb_port_esp32_api ${name}_fake_idf)
endfunction()

kb_port_esp32_under_test(kb_port_esp32_full ${KB_PORT_ESP32_CAN_SOURCE})
# What the MISRA scans build (ci-misra.yml): the port in both chip variants.
add_custom_target(kb_port_esp32_scan)
kb_port_esp32_under_test(kb_port_esp32_minimal ${KB_PORT_ESP32_NO_CAN_SOURCE}
  DEFINES KB_FAKE_NO_TWAI KB_FAKE_LINE_FITTING KB_FAKE_IDF_V6 KB_FAKE_TICK_100)

add_dependencies(kb_port_esp32_scan kb_port_esp32_full kb_port_esp32_minimal)

function(kb_port_esp32_test name)
  cmake_parse_arguments(PARSE_ARGV 1 ARG "" "" "SOURCES;LIBS")
  add_executable(${name} "${_harness}/kb_test_main.cpp" ${ARG_SOURCES})
  target_compile_options(${name} PRIVATE ${KB_PORT_ESP32_STRICT_FLAGS})
  target_include_directories(${name} PRIVATE "${_harness}" "${CMAKE_CURRENT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/src")
  target_compile_definitions(${name} PRIVATE _GLIBCXX_ASSERTIONS=1)
  target_link_libraries(${name} PRIVATE ${ARG_LIBS})
  add_test(NAME ${name} COMMAND ${name})
endfunction()

kb_port_esp32_test(kb_port_esp32_tests
  SOURCES test_error.cpp test_i2c.cpp test_spi.cpp test_uart.cpp test_gpio.cpp test_adc.cpp test_pwm.cpp test_can.cpp
          test_time.cpp test_log.cpp test_contracts.cpp
  LIBS kb_port_esp32_full)
kb_port_esp32_test(kb_port_esp32_minimal_tests SOURCES test_minimal.cpp LIBS kb_port_esp32_minimal)

# Each public header compiles on its own as ISO C++20 with the strict flags.
set(_headers adc can error gpio i2c log port pwm spi time uart version)
set(_checks "")
foreach(_h IN LISTS _headers)
  set(_check "${CMAKE_CURRENT_BINARY_DIR}/header_checks/${_h}.cpp")
  file(WRITE "${_check}.in" "// Generated: kb/port/esp32/${_h}.hpp compiles on its own.\n#include \"kb/port/esp32/${_h}.hpp\"\n")
  configure_file("${_check}.in" "${_check}" COPYONLY)
  list(APPEND _checks "${_check}")
endforeach()
add_library(kb_port_esp32_header_checks OBJECT ${_checks})
target_compile_options(kb_port_esp32_header_checks PRIVATE ${KB_PORT_ESP32_STRICT_FLAGS})
target_link_libraries(kb_port_esp32_header_checks PRIVATE kb_port_esp32_api)

# Compile-fail tests: each file must fail to build, for the reason in its "EXPECT: <regex>" line.
foreach(_file IN ITEMS ref_from_wrong_contract.cpp controller_copied.cpp output_pin_as_input.cpp)
  set(_src "${CMAKE_CURRENT_SOURCE_DIR}/compile_fail/${_file}")
  get_filename_component(_stem "${_src}" NAME_WE)
  file(STRINGS "${_src}" _expect REGEX "EXPECT: ")
  string(REGEX REPLACE ".*EXPECT: " "" _expect "${_expect}")
  set(_target kb_port_esp32_compile_fail_${_stem})
  add_library(${_target} OBJECT EXCLUDE_FROM_ALL "${_src}")
  target_compile_options(${_target} PRIVATE ${KB_PORT_ESP32_STRICT_FLAGS})
  target_link_libraries(${_target} PRIVATE kb_port_esp32_api)
  add_test(NAME ${_target} COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target ${_target})
  set_tests_properties(${_target} PROPERTIES PASS_REGULAR_EXPRESSION "${_expect}")
endforeach()

# The source layout (tools/check_layout.py): one source per public header, one boundary file per SDK component;
# and the checker's own test. Needs python3 with PyYAML (the preflight and the CI images have it).
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
  execute_process(COMMAND "${Python3_EXECUTABLE}" -c "import yaml" RESULT_VARIABLE _kb_yaml OUTPUT_QUIET ERROR_QUIET)
endif()
if(Python3_Interpreter_FOUND AND _kb_yaml EQUAL 0)
  add_test(NAME kb_port_esp32_layout
           COMMAND "${Python3_EXECUTABLE}" -I "${PROJECT_SOURCE_DIR}/tools/check_layout.py" --root "${PROJECT_SOURCE_DIR}")
  add_test(NAME kb_port_esp32_layout_checker
           COMMAND "${Python3_EXECUTABLE}" -I "${PROJECT_SOURCE_DIR}/tools/test_check_layout.py")
else()
  message(STATUS "kb_port_esp32: python3 with PyYAML not found, the layout tests are not registered")
endif()
