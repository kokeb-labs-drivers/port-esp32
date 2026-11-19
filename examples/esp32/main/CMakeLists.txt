# The examples' main component. The app source comes from app_config.yml (APP_SOURCE_FILE).
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The port-esp32 authors
if(NOT DEFINED APP_SOURCE_FILE)
  set(APP_SOURCE_FILE "${DEFAULT_SOURCE_FILE}")
endif()
idf_component_register(SRCS "${APP_SOURCE_FILE}"
                       INCLUDE_DIRS "."
                       REQUIRES kb kb_port_esp32 freertos log)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)
# -Wpedantic stays off: ESP-IDF's own headers use GCC extensions.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Wshadow -Wformat=2 -Werror)
