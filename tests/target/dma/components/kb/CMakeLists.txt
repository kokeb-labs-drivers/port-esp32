# The ESP-IDF component `kb` (kb-core), forwarded to the pinned copy (as examples/esp32/components/kb does).
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The port-esp32 authors
get_filename_component(KB_CORE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../../../third_party/kb-core" ABSOLUTE)
include("${KB_CORE_ROOT}/CMakeLists.txt")
