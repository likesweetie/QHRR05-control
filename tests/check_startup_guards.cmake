file(MAKE_DIRECTORY "${TEST_DIR}")
# These tests exercise input/startup failures, not high-resolution mesh loading.
# Keep the current model's physical parameters and collision shapes exactly.
execute_process(COMMAND python3 "${CMAKE_CURRENT_LIST_DIR}/prepare_startup_guard_model.py"
    "${MODEL}" "${CONTROL_CONFIG}" "${TEST_DIR}/model"
    RESULT_VARIABLE prepare_result)
if(NOT prepare_result EQUAL 0)
    message(FATAL_ERROR "Could not prepare current-model startup fixture")
endif()
set(MODEL "${TEST_DIR}/model/scene.xml")
set(CONTROL_CONFIG "${TEST_DIR}/model/control.yaml")
string(RANDOM LENGTH 12 ALPHABET 0123456789abcdef shm_suffix)
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
function(write_manifest path control)
    file(WRITE "${path}" "shm_name: /qhrr05-startup-${shm_suffix}\nmonitoring: {display_interval_sec: 2, window_sec: 10, sample_hz: 100}\ndebug_processes:\n  - name: simulator\n    exec_path: '${SIMULATOR}'\n    wait_ready: true\n    args: ['--without-can', '--model', '${MODEL}', '--control-config', '${control}', '--shm-name', '{shm_name}', '--headless', '--duration', '4']\n  - name: controller\n    exec_path: '${CONTROLLER}'\n    args: ['${control}', '{shm_name}', '--without-can']\n")
endfunction()
expect_result(1 "usage: control CONTROL_CONFIG SHM_NAME" "${CONTROLLER}")
expect_result(1 "--simulation-config is required" "${SIMULATOR}"
    --model "${MODEL}" --control-config "${CONTROL_CONFIG}"
    --shm-name /qhrr05-iq-missing-config --hal-config unused.yaml)
expect_result(1 "unavailable with --without-can" "${SIMULATOR}"
    --model "${MODEL}" --control-config "${CONTROL_CONFIG}"
    --shm-name /qhrr05-iq-no-can --without-can --simulation-config unused.yaml)
expect_result(1 "usage: daemon" "${DAEMON}" --robot other-platform)
file(WRITE "${TEST_DIR}/missing-shm.yaml"
    "processes:\n  - name: completed\n    exec_path: /bin/true\n    args: []\n")
expect_result(1 "daemon config requires shm_name" "${DAEMON}"
    --config "${TEST_DIR}/missing-shm.yaml")
file(WRITE "${TEST_DIR}/bad-shm.yaml"
    "shm_name: invalid/name\nprocesses:\n  - name: completed\n    exec_path: /bin/true\n    args: []\n")
expect_result(1 "daemon shm_name must be" "${DAEMON}"
    --config "${TEST_DIR}/bad-shm.yaml")
expect_result(1 "shm_open" "${SIMULATOR}" --without-can --model "${MODEL}"
    --control-config "${CONTROL_CONFIG}" --shm-name /qhrr05-missing-owner-${shm_suffix}
    --headless --duration 1)
file(READ "${CONTROL_CONFIG}" config_text)
foreach(kind other nan limits)
    if(kind STREQUAL "other")
        string(REPLACE "robot: qhrr-05" "robot: other-platform" invalid "${config_text}")
        set(expected "only qhrr-05 is supported")
    elseif(kind STREQUAL "nan")
        string(REGEX REPLACE "kp: [^\n]*" "kp: [.nan, 80.0, 80.0]" invalid "${config_text}")
        set(expected "joint values must be finite")
    else()
        string(REPLACE "position_rad: [0.0, 0.9, -1.8]" "position_rad: [0.0, 100.0, -1.8]" invalid "${config_text}")
        set(expected "outside model limits")
    endif()
    file(WRITE "${TEST_DIR}/${kind}.yaml" "${invalid}")
    write_manifest("${TEST_DIR}/${kind}-processes.yaml" "${TEST_DIR}/${kind}.yaml")
    expect_result(1 "${expected}" "${DAEMON}" --no-terminals --config "${TEST_DIR}/${kind}-processes.yaml" --without-can)
endforeach()
file(WRITE "${TEST_DIR}/empty.yaml" "processes: []\n")
expect_result(1 "nonempty processes" "${DAEMON}" --no-terminals --config "${TEST_DIR}/empty.yaml")
expect_result(1 "nonempty debug_processes" "${DAEMON}" --no-terminals --config "${TEST_DIR}/empty.yaml" --without-can)
write_manifest("${TEST_DIR}/valid.yaml" "${CONTROL_CONFIG}")
file(READ "${TEST_DIR}/valid.yaml" valid_manifest)
foreach(kind interval window sample)
    if(kind STREQUAL "interval")
        string(REPLACE "display_interval_sec: 2" "display_interval_sec: 1" invalid_manifest "${valid_manifest}")
        set(expected "monitoring.display_interval_sec")
    elseif(kind STREQUAL "window")
        string(REPLACE "window_sec: 10" "window_sec: .nan" invalid_manifest "${valid_manifest}")
        set(expected "monitoring.window_sec")
    else()
        string(REPLACE "sample_hz: 100" "sample_hz: 0" invalid_manifest "${valid_manifest}")
        set(expected "monitoring.sample_hz")
    endif()
    file(WRITE "${TEST_DIR}/bad-${kind}.yaml" "${invalid_manifest}")
    expect_result(1 "${expected}" "${DAEMON}" --no-terminals
        --config "${TEST_DIR}/bad-${kind}.yaml" --without-can)
endforeach()
expect_result(1 "nonempty processes" "${DAEMON}" --no-terminals --config "${TEST_DIR}/valid.yaml")
execute_process(COMMAND "${DAEMON}" --no-terminals --config "${TEST_DIR}/valid.yaml" --without-can
    WORKING_DIRECTORY "${TEST_DIR}"
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error TIMEOUT 15)
if(NOT result EQUAL 0 OR NOT output MATCHES "stopped at t=4")
    message(FATAL_ERROR "Generic supervisor integration failed: ${result} ${output}${error}")
endif()
message(STATUS "application startup guards and generic process supervisor passed")
