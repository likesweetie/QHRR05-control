cmake_minimum_required(VERSION 3.22)
project(QHRR05 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(MUJOCO_DIR ${PROJECT_SOURCE_DIR}/third_party/mujoco)

find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)
pkg_search_module(GLFW REQUIRED glfw3)
pkg_check_modules(TERMINAL_X11 REQUIRED IMPORTED_TARGET x11 xrandr)

# Offline, pinned DataTamer 2.0.0 and MCAP C++ 2.1.3; no build-time downloads.
set(FETCHCONTENT_SOURCE_DIR_DATA_TAMER_MCAP ${PROJECT_SOURCE_DIR}/third_party/mcap)
add_subdirectory(third_party/data_tamer/data_tamer_cpp EXCLUDE_FROM_ALL)

# Header visibility only. Implementations are linked through their own targets below.
add_library(qhrr05_common INTERFACE)
target_include_directories(qhrr05_common INTERFACE ${PROJECT_SOURCE_DIR}/src)

add_library(ipc_simulation STATIC src/ipc/simulation/channel.cpp)
target_link_libraries(ipc_simulation PUBLIC qhrr05_common)

add_library(motor_can_protocol STATIC src/motor_can_protocol/openrobot_protocol.cpp)
target_link_libraries(motor_can_protocol PUBLIC qhrr05_common)

add_library(robot_model STATIC src/robot_model/mujoco_backend.cpp)
target_link_libraries(robot_model PUBLIC qhrr05_common yaml-cpp)
target_include_directories(robot_model PRIVATE ${MUJOCO_DIR}/include)
target_link_directories(robot_model PUBLIC ${MUJOCO_DIR}/lib)
target_link_libraries(robot_model PUBLIC mujoco)

add_library(motion_controllers STATIC
    src/motion_controllers/numerics/math.cpp
    src/motion_controllers/homing_controller.cpp
    src/motion_controllers/zero_set_controller.cpp
    src/motion_controllers/gravity_compensation_controller.cpp
    src/motion_controllers/tracking_controller.cpp
    src/motion_controllers/tracking_ilc.cpp
    src/motion_controllers/ball_following_controller.cpp
    src/motion_controllers/reaction_force_observer.cpp
    src/motion_controllers/magnetic_touch_controller.cpp)
target_link_libraries(motion_controllers PUBLIC qhrr05_common yaml-cpp robot_model)

add_library(control_core STATIC src/control/control.cpp)
target_link_libraries(control_core PUBLIC motion_controllers interfaces safety_core pthread)

add_library(simulation_core STATIC
    src/simulation/simulation.cpp
    src/simulation/interaction.cpp
    src/simulation/virtual_iq_sensor.cpp)
target_include_directories(simulation_core PUBLIC ${MUJOCO_DIR}/include)
target_link_directories(simulation_core PUBLIC ${MUJOCO_DIR}/lib)
target_link_libraries(simulation_core PUBLIC motor_can_protocol ipc_simulation yaml-cpp mujoco pthread)

add_library(daemon_core STATIC
    src/daemon/daemon.cpp
    src/daemon/terminal_layout.cpp
    src/daemon/statistics.cpp)
target_link_libraries(daemon_core PUBLIC qhrr05_common pthread)
target_link_libraries(daemon_core PRIVATE PkgConfig::TERMINAL_X11)

add_library(safety_core STATIC src/safety/safety_layer.cpp)
target_link_libraries(safety_core PUBLIC qhrr05_common yaml-cpp)

add_subdirectory(src/interfaces)

add_executable(daemon apps/daemon/main.cpp)
target_link_libraries(daemon PRIVATE daemon_core yaml-cpp)
target_compile_definitions(daemon PRIVATE
    QHRR05_DEFAULT_DAEMON_CONFIG="${PROJECT_SOURCE_DIR}/config/daemon/daemon.yaml")

add_executable(control apps/control/main.cpp)
target_link_libraries(control PRIVATE control_core)

add_executable(can_hal apps/can_hal/main.cpp)
target_link_libraries(can_hal PRIVATE motor_can_protocol yaml-cpp pthread)

add_executable(web_gui_bridge apps/interfaces/web_gui/main.cpp)
target_link_libraries(web_gui_bridge PRIVATE interfaces)

set(SIMULATE_COPY ${PROJECT_SOURCE_DIR}/third_party/mujoco_simulate)
add_executable(simulation
    apps/simulation/main.cpp
    ${SIMULATE_COPY}/glfw_adapter.cc
    ${SIMULATE_COPY}/glfw_dispatch.cc
    ${SIMULATE_COPY}/platform_ui_adapter.cc
    ${SIMULATE_COPY}/simulate.cc)
target_include_directories(simulation PRIVATE ${GLFW_INCLUDE_DIRS} ${SIMULATE_COPY})
target_link_libraries(simulation PRIVATE simulation_core motion_controllers interfaces ${GLFW_LIBRARIES} GL pthread dl m)
set_target_properties(simulation PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")

add_executable(input apps/interfaces/input/main.cpp)
target_link_libraries(input PRIVATE interfaces)

execute_process(COMMAND git rev-parse HEAD WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE QHRR_GIT_COMMIT OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
execute_process(COMMAND git status --porcelain WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE QHRR_GIT_STATUS OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
set(QHRR_GIT_DIRTY 0)
if(QHRR_GIT_STATUS)
    set(QHRR_GIT_DIRTY 1)
endif()
add_library(shm_logger_core STATIC
    src/shm_logger/shm_snapshot_reader.cpp src/shm_logger/log_schema.cpp
    src/shm_logger/session_storage.cpp src/shm_logger/logger_service.cpp
    src/shm_logger/csv_logger.cpp)
target_link_libraries(shm_logger_core PUBLIC qhrr05_common ipc_simulation data_tamer::data_tamer yaml-cpp OpenSSL::Crypto pthread)
target_compile_definitions(shm_logger_core PRIVATE
    QHRR_GIT_COMMIT="${QHRR_GIT_COMMIT}" QHRR_GIT_DIRTY=${QHRR_GIT_DIRTY}
    QHRR_BUILD_COMPILER="${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}"
    QHRR_BUILD_TYPE="${CMAKE_BUILD_TYPE}")
add_executable(shm_logger apps/shm_logger/main.cpp)
target_link_libraries(shm_logger PRIVATE shm_logger_core)
add_executable(qhrr_logctl apps/logctl/main.cpp)
set_target_properties(qhrr_logctl PROPERTIES OUTPUT_NAME qhrr-logctl)
target_link_libraries(qhrr_logctl PRIVATE shm_logger_core)
target_compile_definitions(qhrr_logctl PRIVATE
    QHRR_DEFAULT_DAEMON_CONFIG="${PROJECT_SOURCE_DIR}/config/daemon/daemon.yaml")
set(QHRR_ANALYSIS_PYTHON python3 CACHE STRING "Python interpreter with tools/analysis/requirements.txt installed")
foreach(tool IN ITEMS inspect analyze benchmark)
    file(GENERATE OUTPUT ${PROJECT_BINARY_DIR}/qhrr-log-${tool}
        CONTENT "#!/usr/bin/env bash\nexec \"${QHRR_ANALYSIS_PYTHON}\" \"${PROJECT_SOURCE_DIR}/tools/analysis/log_${tool}.py\" \"$@\"\n"
        FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
endforeach()

include(CTest)
if(BUILD_TESTING)
    add_executable(zero_set_controller_test tests/zero_set_controller_test.cpp)
    target_link_libraries(zero_set_controller_test PRIVATE motion_controllers)
    add_test(NAME zero_set_controller COMMAND zero_set_controller_test
        ${PROJECT_SOURCE_DIR}/config/control/control.yaml)
    add_executable(zero_set_can_test tests/zero_set_can_test.cpp)
    target_link_libraries(zero_set_can_test PRIVATE control_core simulation_core)
    set_target_properties(zero_set_can_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME zero_set_can COMMAND bash ${PROJECT_SOURCE_DIR}/tests/run_can_hal_vcan_test.sh
        $<TARGET_FILE:zero_set_can_test> ${PROJECT_SOURCE_DIR}/config/control/control.yaml
        ${PROJECT_SOURCE_DIR}/models/qhrr-05/scene.xml
        ${PROJECT_SOURCE_DIR}/config/can_hal/can_hal.yaml $<TARGET_FILE:can_hal>
        ${PROJECT_SOURCE_DIR}/config/simulation/simulation.yaml)
    set_tests_properties(zero_set_can PROPERTIES TIMEOUT 45)
    add_test(NAME launcher_autobuild COMMAND python3
        ${PROJECT_SOURCE_DIR}/tests/launcher_autobuild_test.py)
    set_tests_properties(launcher_autobuild PROPERTIES TIMEOUT 120)
    add_test(NAME simulation_restart COMMAND python3
        ${PROJECT_SOURCE_DIR}/tests/simulation_restart_test.py)
    set_tests_properties(simulation_restart PROPERTIES TIMEOUT 30)
    add_executable(simulation_ipc_test tests/simulation_ipc_test.cpp)
    target_link_libraries(simulation_ipc_test PRIVATE ipc_simulation)
    add_test(NAME simulation_ipc COMMAND simulation_ipc_test)
    add_executable(simulation_interaction_test tests/simulation_interaction_test.cpp)
    target_link_libraries(simulation_interaction_test PRIVATE simulation_core)
    set_target_properties(simulation_interaction_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME simulation_interaction COMMAND simulation_interaction_test
        ${PROJECT_SOURCE_DIR}/models/qhrr-05/scene.xml)
    add_executable(web_gui_fixture tests/web_gui_fixture.cpp)
    target_link_libraries(web_gui_fixture PRIVATE interfaces)
    add_executable(safety_layer_test tests/safety_layer_test.cpp)
    target_link_libraries(safety_layer_test PRIVATE safety_core)
    add_test(NAME safety_layer COMMAND safety_layer_test
        ${PROJECT_SOURCE_DIR}/tests/fixtures/control.yaml)
    add_executable(logging_core_test tests/logging_core_test.cpp)
    target_link_libraries(logging_core_test PRIVATE shm_logger_core)
    add_test(NAME logging_core COMMAND logging_core_test)
    add_executable(logging_test_producer tests/logging_test_producer.cpp)
    target_link_libraries(logging_test_producer PRIVATE qhrr05_common ipc_simulation pthread)
    add_test(NAME logging_service COMMAND ${QHRR_ANALYSIS_PYTHON} ${PROJECT_SOURCE_DIR}/tests/logging_service_test.py)
    add_test(NAME logging_daemon COMMAND ${QHRR_ANALYSIS_PYTHON} ${PROJECT_SOURCE_DIR}/tests/logging_daemon_test.py)
    set_tests_properties(logging_service logging_daemon PROPERTIES TIMEOUT 90)
    add_test(NAME terminal_layout COMMAND bash
        ${PROJECT_SOURCE_DIR}/tests/run_gui_test.sh ${PROJECT_SOURCE_DIR}/tests/terminal_layout_test.py $<TARGET_FILE:daemon>)
    add_test(NAME simulator_gui COMMAND bash
        ${PROJECT_SOURCE_DIR}/tests/run_gui_test.sh ${PROJECT_SOURCE_DIR}/tests/simulator_gui_test.py)
    add_test(NAME can_gui COMMAND bash
        ${PROJECT_SOURCE_DIR}/tests/run_gui_test.sh ${PROJECT_SOURCE_DIR}/tests/can_gui_test.py)
    add_executable(daemon_statistics_test tests/daemon_statistics_test.cpp)
    target_link_libraries(daemon_statistics_test PRIVATE daemon_core)
    add_test(NAME daemon_statistics COMMAND daemon_statistics_test)
    add_test(NAME terminal_statistics COMMAND python3
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/terminal_statistics_test.py $<TARGET_FILE:daemon>)
    add_test(NAME daemon_shm COMMAND python3
        ${PROJECT_SOURCE_DIR}/tests/daemon_shm_smoke.py)
    set_tests_properties(terminal_layout simulator_gui can_gui terminal_statistics daemon_shm
        PROPERTIES TIMEOUT 90)
    add_executable(virtual_iq_sensor_test tests/virtual_iq_sensor_test.cpp)
    target_link_libraries(virtual_iq_sensor_test PRIVATE simulation_core)
    set_target_properties(virtual_iq_sensor_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME virtual_iq_sensor COMMAND virtual_iq_sensor_test
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/simulation.yaml)
    add_executable(virtual_mit_motor_test tests/virtual_mit_motor_test.cpp)
    target_link_libraries(virtual_mit_motor_test PRIVATE simulation_core)
    set_target_properties(virtual_mit_motor_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME virtual_mit_motor COMMAND bash
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_can_hal_vcan_test.sh
        $<TARGET_FILE:virtual_mit_motor_test>
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/config/can_hal/can_hal.yaml
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/simulation.yaml)
    set_tests_properties(virtual_mit_motor PROPERTIES TIMEOUT 15)
    add_test(NAME simulation_zero_startup COMMAND bash
        ${PROJECT_SOURCE_DIR}/tests/run_can_hal_vcan_test.sh
        ${QHRR_ANALYSIS_PYTHON} ${PROJECT_SOURCE_DIR}/tests/simulation_zero_startup_test.py)
    set_tests_properties(simulation_zero_startup PROPERTIES TIMEOUT 25)
    add_executable(virtual_iq_can_test tests/virtual_iq_can_test.cpp)
    target_link_libraries(virtual_iq_can_test PRIVATE simulation_core)
    set_target_properties(virtual_iq_can_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME virtual_iq_can COMMAND bash
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_can_hal_vcan_test.sh
        $<TARGET_FILE:virtual_iq_can_test>
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/config/can_hal/can_hal.yaml
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/simulation.yaml $<TARGET_FILE:can_hal>)
    add_test(NAME can_bus_load COMMAND bash
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_can_hal_vcan_test.sh
        python3 ${CMAKE_CURRENT_SOURCE_DIR}/tests/can_bus_load_test.py $<TARGET_FILE:can_hal>)
    add_test(NAME launcher_modes COMMAND unshare --user --map-root-user --net --
        python3 ${CMAKE_CURRENT_SOURCE_DIR}/tests/launcher_modes_test.py
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_daemon.sh)
    add_executable(can_control_integration_test tests/can_control_integration_test.cpp)
    target_link_libraries(can_control_integration_test PRIVATE simulation_core)
    set_target_properties(can_control_integration_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    foreach(tracking_variant IN ITEMS inverse_dynamics ctc impedance)
        set(tracking_test_args)
        set(tracking_test_name can_control_integration)
        if(NOT tracking_variant STREQUAL "inverse_dynamics")
            set(tracking_test_name can_${tracking_variant}_integration)
            list(APPEND tracking_test_args ${tracking_variant})
        endif()
        add_test(NAME ${tracking_test_name} COMMAND bash
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_can_hal_vcan_test.sh
            $<TARGET_FILE:can_control_integration_test>
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/control.yaml
            ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml
            ${CMAKE_CURRENT_SOURCE_DIR}/config/can_hal/can_hal.yaml
            $<TARGET_FILE:can_hal> $<TARGET_FILE:control>
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/simulation.yaml ${tracking_test_args})
    endforeach()
    add_executable(can_hal_test tests/can_hal_test.cpp)
    target_link_libraries(can_hal_test PRIVATE motor_can_protocol pthread)
    add_test(NAME can_hal_vcan COMMAND bash
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_can_hal_vcan_test.sh
        $<TARGET_FILE:can_hal_test> $<TARGET_FILE:can_hal> $<TARGET_FILE:shm_logger>)
    add_test(NAME can_hal_lifecycle COMMAND bash
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_can_hal_vcan_test.sh
        python3 ${CMAKE_CURRENT_SOURCE_DIR}/tests/can_hal_lifecycle_test.py
        $<TARGET_FILE:can_hal> $<TARGET_FILE:daemon>)
    add_test(NAME source_layout COMMAND python3 ${PROJECT_SOURCE_DIR}/tests/source_layout_test.py)
    add_executable(motion_numerics_test tests/motion_numerics_test.cpp
        src/motion_controllers/numerics/math.cpp)
    target_link_libraries(motion_numerics_test PRIVATE qhrr05_common mujoco)
    target_link_directories(motion_numerics_test PRIVATE ${MUJOCO_DIR}/lib)
    target_include_directories(motion_numerics_test PRIVATE ${MUJOCO_DIR}/include)
    set_target_properties(motion_numerics_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME motion_numerics COMMAND motion_numerics_test)
    add_executable(robot_model_test tests/robot_model_test.cpp)
    target_link_libraries(robot_model_test PRIVATE robot_model)
    target_include_directories(robot_model_test PRIVATE ${MUJOCO_DIR}/include)
    set_target_properties(robot_model_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME robot_model COMMAND robot_model_test
        ${PROJECT_SOURCE_DIR}/config/control/control.yaml
        ${PROJECT_SOURCE_DIR}/models/qhrr-05/qhrr-05.xml)
    add_executable(control_io_test tests/control_io_test.cpp)
    target_link_libraries(control_io_test PRIVATE control_core simulation_core)
    set_target_properties(control_io_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME control_io COMMAND control_io_test
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/control.yaml
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml
        $<TARGET_FILE:control>)
    add_executable(magnetic_touch_test tests/magnetic_touch_test.cpp)
    target_link_libraries(magnetic_touch_test PRIVATE motion_controllers simulation_core)
    set_target_properties(magnetic_touch_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME magnetic_touch COMMAND magnetic_touch_test
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/control.yaml
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml)
    add_executable(motion_controller_config_test tests/motion_controller_config_test.cpp)
    target_link_libraries(motion_controller_config_test PRIVATE motion_controllers)
    add_test(NAME motion_controller_config COMMAND motion_controller_config_test)
    add_executable(ball_following_test tests/ball_following_test.cpp)
    target_link_libraries(ball_following_test PRIVATE motion_controllers interfaces)
    target_include_directories(ball_following_test PRIVATE ${MUJOCO_DIR}/include)
    add_test(NAME ball_following COMMAND ball_following_test
        ${PROJECT_SOURCE_DIR}/tests/fixtures/control.yaml
        ${PROJECT_SOURCE_DIR}/models/qhrr-05/scene.xml
        ${PROJECT_SOURCE_DIR}/config/input/input.yaml)
    add_executable(tracking_controller_test tests/tracking_controller_test.cpp)
    target_link_libraries(tracking_controller_test PRIVATE motion_controllers simulation_core)
    set_target_properties(tracking_controller_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME tracking_controller COMMAND tracking_controller_test
        ${PROJECT_SOURCE_DIR}/tests/fixtures/control.yaml
        ${PROJECT_SOURCE_DIR}/models/qhrr-05/scene.xml)
    add_executable(shm_buffers_test tests/shm_buffers_test.cpp)
    target_link_libraries(shm_buffers_test PRIVATE qhrr05_common pthread)
    add_test(NAME shm_buffers COMMAND shm_buffers_test)
    add_executable(shm_logger_test tests/shm_logger_test.cpp)
    target_link_libraries(shm_logger_test PRIVATE qhrr05_common ipc_simulation pthread)
    add_test(NAME shm_logger COMMAND shm_logger_test $<TARGET_FILE:shm_logger>)
    add_executable(daemon_lifecycle_test tests/daemon_lifecycle_test.cpp)
    target_link_libraries(daemon_lifecycle_test PRIVATE daemon_core)
    add_test(NAME daemon_lifecycle COMMAND daemon_lifecycle_test)
    add_executable(qhrr05_model_test tests/qhrr05_model_test.cpp)
    target_include_directories(qhrr05_model_test PRIVATE
        ${MUJOCO_DIR}/include)
    target_link_directories(qhrr05_model_test PRIVATE ${MUJOCO_DIR}/lib)
    target_link_libraries(qhrr05_model_test PRIVATE mujoco)
    set_target_properties(qhrr05_model_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME qhrr05_model COMMAND qhrr05_model_test
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/qhrr-05.xml)
    add_test(NAME qhrr05_scene COMMAND qhrr05_model_test
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml)
    add_executable(homing_control_test tests/homing_control_test.cpp)
    target_link_libraries(homing_control_test PRIVATE motion_controllers simulation_core)
    set_target_properties(homing_control_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    # Verify free-space homing without the scene's interactive contact obstacle.
    add_test(NAME qhrr05_homing COMMAND homing_control_test
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/qhrr-05.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/control.yaml)
    add_executable(gravity_compensation_test tests/gravity_compensation_test.cpp)
    target_link_libraries(gravity_compensation_test PRIVATE motion_controllers simulation_core)
    set_target_properties(gravity_compensation_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME qhrr05_gravity COMMAND gravity_compensation_test
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/control.yaml)
    add_test(NAME qhrr05_startup_guards COMMAND ${CMAKE_COMMAND}
        -DCONTROLLER=$<TARGET_FILE:control>
        -DSIMULATOR=$<TARGET_FILE:simulation>
        -DCONTROL_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/control.yaml
        -DMODEL=${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml
        -DDAEMON=$<TARGET_FILE:daemon>
        -DTEST_DIR=${CMAKE_CURRENT_BINARY_DIR}/startup-guards
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_startup_guards.cmake)
endif()
