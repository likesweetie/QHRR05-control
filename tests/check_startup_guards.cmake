file(MAKE_DIRECTORY "${TEST_DIR}")
string(RANDOM LENGTH 12 ALPHABET 0123456789abcdef shm_suffix)
set(daemon_command "${DAEMON}" --shm-name "/qhrr05-startup-${shm_suffix}")
function(expect_result expected_code expected_message)
    execute_process(COMMAND ${ARGN}
        RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error TIMEOUT 15)
    if(NOT "${result}" STREQUAL "${expected_code}")
        message(FATAL_ERROR "Unexpected exit code ${result}: ${output}${error}")
    endif()
    string(FIND "${output}${error}" "${expected_message}" found)
    if(found EQUAL -1)
        message(FATAL_ERROR "Missing '${expected_message}': ${output}${error}")
    endif()
endfunction()
expect_result(1 "usage: task_controller CONTROL_CONFIG SHM_NAME" "${CONTROLLER}")
expect_result(1 "unsupported robot" ${daemon_command} --robot other-platform)
file(READ "${CONTROL_CONFIG}" config_text)
string(REPLACE "robot: qhrr-05" "robot: other-platform" invalid "${config_text}")
file(WRITE "${TEST_DIR}/other.yaml" "${invalid}")
expect_result(1 "only qhrr-05 is supported" ${daemon_command}
    --control-config "${TEST_DIR}/other.yaml" --headless --duration 1)
string(REGEX REPLACE "kp: [^\n]*" "kp: [.nan, 80.0, 80.0]" invalid "${config_text}")
file(WRITE "${TEST_DIR}/nan.yaml" "${invalid}")
expect_result(1 "joint values must be finite" ${daemon_command}
    --control-config "${TEST_DIR}/nan.yaml" --headless --duration 1)
string(REPLACE "position_rad: [0.0, 0.9, -1.8]" "position_rad: [0.0, 100.0, -1.8]" invalid "${config_text}")
file(WRITE "${TEST_DIR}/limits.yaml" "${invalid}")
expect_result(1 "outside model limits" ${daemon_command}
    --control-config "${TEST_DIR}/limits.yaml" --headless --duration 1)
file(WRITE "${TEST_DIR}/multiple.yaml" "robots:\n  qhrr-05: {}\n  other-platform: {}\nprocesses: []\n")
expect_result(1 "robots must contain only qhrr-05" ${daemon_command} --config "${TEST_DIR}/multiple.yaml")
# Exercise the complete daemon/exec/controller path from an unrelated directory.
execute_process(COMMAND ${daemon_command} --headless --duration 4
    WORKING_DIRECTORY "${TEST_DIR}"
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error TIMEOUT 15)
if(NOT result EQUAL 0 OR NOT output MATCHES "stopped at t=4"
    OR NOT output MATCHES "task_controller exited code=0")
    message(FATAL_ERROR "Daemon independent-loop integration failed: ${result} ${output}${error}")
endif()
message(STATUS "homing startup guards and daemon integration passed")
