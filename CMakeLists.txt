cmake_minimum_required(VERSION 3.16)
project(QHRR05 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(MUJOCO_DIR ${PROJECT_SOURCE_DIR}/third_party/mujoco)

find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
pkg_check_modules(TERMINAL_X11 REQUIRED IMPORTED_TARGET x11 xrandr)

# Header visibility only. Implementations are linked through their own targets below.
add_library(qhrr05_common INTERFACE)
target_include_directories(qhrr05_common INTERFACE ${PROJECT_SOURCE_DIR}/src)

add_library(motor_can_protocol STATIC src/motor_can_protocol/openrobot_protocol.cpp)
target_link_libraries(motor_can_protocol PUBLIC qhrr05_common)

add_library(motion_controllers STATIC
    src/motion_controllers/homing_controller.cpp
    src/motion_controllers/gravity_compensation_controller.cpp
    src/motion_controllers/reaction_force_observer.cpp
    src/motion_controllers/magnetic_touch_controller.cpp)
target_link_libraries(motion_controllers PUBLIC qhrr05_common yaml-cpp)

add_library(control_core STATIC src/control/control.cpp)
target_link_libraries(control_core PUBLIC motion_controllers pthread)

add_library(simulation_core STATIC
    src/simulation/simulation.cpp
    src/simulation/virtual_iq_sensor.cpp)
target_include_directories(simulation_core PUBLIC ${MUJOCO_DIR}/include)
target_link_directories(simulation_core PUBLIC ${MUJOCO_DIR}/lib)
target_link_libraries(simulation_core PUBLIC motor_can_protocol yaml-cpp mujoco pthread)

add_library(daemon_core STATIC
    src/daemon/daemon.cpp
    src/daemon/terminal_layout.cpp
    src/daemon/statistics.cpp)
target_link_libraries(daemon_core PUBLIC qhrr05_common pthread)
target_link_libraries(daemon_core PRIVATE PkgConfig::TERMINAL_X11)

add_library(input_core STATIC src/input/joystick.cpp)
target_link_libraries(input_core PUBLIC qhrr05_common pthread)

add_executable(daemon apps/daemon/main.cpp)
target_link_libraries(daemon PRIVATE daemon_core yaml-cpp)
target_compile_definitions(daemon PRIVATE
    QHRR05_DEFAULT_DAEMON_CONFIG="${PROJECT_SOURCE_DIR}/config/daemon/daemon.yaml")

add_executable(control apps/control/main.cpp)
target_link_libraries(control PRIVATE control_core)

add_executable(can_hal apps/can_hal/main.cpp)
target_link_libraries(can_hal PRIVATE motor_can_protocol yaml-cpp pthread)

set(SIMULATE_COPY ${PROJECT_SOURCE_DIR}/third_party/mujoco_simulate)
add_executable(simulation
    apps/simulation/main.cpp
    ${SIMULATE_COPY}/glfw_adapter.cc
    ${SIMULATE_COPY}/glfw_dispatch.cc
    ${SIMULATE_COPY}/platform_ui_adapter.cc
    ${SIMULATE_COPY}/simulate.cc)
target_include_directories(simulation PRIVATE ${GLFW_INCLUDE_DIRS} ${SIMULATE_COPY})
target_link_libraries(simulation PRIVATE simulation_core ${GLFW_LIBRARIES} GL pthread dl m)
set_target_properties(simulation PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")

add_executable(input apps/input/main.cpp)
target_link_libraries(input PRIVATE input_core)

add_executable(shm_logger apps/shm_logger/main.cpp)
target_link_libraries(shm_logger PRIVATE qhrr05_common pthread)

include(CTest)
if(BUILD_TESTING)
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
    add_test(NAME can_control_integration COMMAND bash
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_can_hal_vcan_test.sh
        $<TARGET_FILE:can_control_integration_test>
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/control.yaml
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/config/can_hal/can_hal.yaml
        $<TARGET_FILE:can_hal> $<TARGET_FILE:control>
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/simulation.yaml)
    add_executable(can_hal_test tests/can_hal_test.cpp)
    target_link_libraries(can_hal_test PRIVATE motor_can_protocol pthread)
    add_test(NAME can_hal_vcan COMMAND bash
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_can_hal_vcan_test.sh
        $<TARGET_FILE:can_hal_test> $<TARGET_FILE:can_hal> $<TARGET_FILE:shm_logger>)
    add_test(NAME can_hal_lifecycle COMMAND bash
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_can_hal_vcan_test.sh
        python3 ${CMAKE_CURRENT_SOURCE_DIR}/tests/can_hal_lifecycle_test.py
        $<TARGET_FILE:can_hal> $<TARGET_FILE:daemon>)
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
    add_executable(shm_buffers_test tests/shm_buffers_test.cpp)
    target_link_libraries(shm_buffers_test PRIVATE qhrr05_common pthread)
    add_test(NAME shm_buffers COMMAND shm_buffers_test)
    add_executable(shm_logger_test tests/shm_logger_test.cpp)
    target_link_libraries(shm_logger_test PRIVATE qhrr05_common pthread)
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
    add_test(NAME qhrr05_homing COMMAND homing_control_test
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml
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
