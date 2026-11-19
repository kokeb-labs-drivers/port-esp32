# The ESP-IDF component `kb` (kb-core): ESP-IDF names a component after its folder, so this folder carries the name
# and forwards to the pinned kb-core copy, whose root CMakeLists.txt registers the component under ESP_PLATFORM.
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The port-esp32 authors
get_filename_component(KB_CORE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../../third_party/kb-core" ABSOLUTE)
include("${KB_CORE_ROOT}/CMakeLists.txt")
