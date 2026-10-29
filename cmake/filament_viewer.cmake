# MuJoCo is the top-level project because its dependency scripts use CMAKE_SOURCE_DIR.
# This hook includes our targets only after MuJoCo has defined all renderer targets.
if(NOT QHRR05_SOURCE_DIR)
    message(FATAL_ERROR "QHRR05_SOURCE_DIR is required")
endif()
cmake_language(DEFER CALL include "${QHRR05_SOURCE_DIR}/visualization/CMakeLists.txt")
