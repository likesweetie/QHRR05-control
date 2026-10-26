cmake_minimum_required(VERSION 3.16)
project(QHRR05 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(MUJOCO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco)

find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

add_library(openrobot_can STATIC app/motor_can/openrobot_protocol.cpp)
target_include_directories(openrobot_can PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(can_hal app/can_hal/main.cpp)
target_link_libraries(can_hal PRIVATE openrobot_can yaml-cpp pthread)

add_library(motion_controllers STATIC
    app/motion_controllers/homing_controller.cpp
    app/motion_controllers/gravity_compensation_controller.cpp
    app/motion_controllers/reaction_force_observer.cpp
    app/motion_controllers/magnetic_touch_controller.cpp)
target_include_directories(motion_controllers PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/app ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(motion_controllers PUBLIC yaml-cpp)

add_library(mujoco_simulation STATIC app/mujoco_simulate/simulation.cpp)
target_include_directories(mujoco_simulation PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/app ${CMAKE_CURRENT_SOURCE_DIR}/include ${MUJOCO_DIR}/include)
target_link_directories(mujoco_simulation PUBLIC ${MUJOCO_DIR}/lib)
target_link_libraries(mujoco_simulation PUBLIC openrobot_can yaml-cpp mujoco pthread)

add_executable(daemon
    app/daemon/main.cpp
    app/daemon/daemon.cpp
)
target_include_directories(daemon PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(daemon PRIVATE
    yaml-cpp
    pthread
)
target_compile_definitions(daemon PRIVATE
    QHRR05_DEFAULT_DAEMON_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/app_config/daemon_config.yaml")

add_executable(task_controller app/task_controller/main.cpp app/task_controller/task_controller.cpp)
target_link_libraries(task_controller PRIVATE motion_controllers openrobot_can pthread)

set(SIMULATE_COPY ${CMAKE_CURRENT_SOURCE_DIR}/app/mujoco_simulate/simulate)
add_executable(mujoco_simulate
    app/mujoco_simulate/main.cpp
    ${SIMULATE_COPY}/glfw_adapter.cc
    ${SIMULATE_COPY}/glfw_dispatch.cc
    ${SIMULATE_COPY}/platform_ui_adapter.cc
    ${SIMULATE_COPY}/simulate.cc)
target_include_directories(mujoco_simulate PRIVATE ${GLFW_INCLUDE_DIRS} ${SIMULATE_COPY})
target_link_libraries(mujoco_simulate PRIVATE mujoco_simulation ${GLFW_LIBRARIES} GL pthread dl m)
set_target_properties(mujoco_simulate PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")

add_executable(aux_reader
    app/aux_reader/main.cpp
    app/aux_reader/joystick.cpp
)
target_include_directories(aux_reader PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(aux_reader PRIVATE
    pthread
)

add_executable(shm_logger
    app/shm_logger/main.cpp
)
target_include_directories(shm_logger PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(shm_logger PRIVATE
    pthread
)

include(CTest)
if(BUILD_TESTING)
    add_executable(can_control_integration_test tests/can_control_integration_test.cpp)
    target_link_libraries(can_control_integration_test PRIVATE mujoco_simulation)
    set_target_properties(can_control_integration_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME can_control_integration COMMAND bash
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_can_hal_vcan_test.sh
        $<TARGET_FILE:can_control_integration_test>
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/control_config.yaml
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/config/motor_config/can_hal.yaml
        $<TARGET_FILE:can_hal> $<TARGET_FILE:task_controller>)
    add_executable(can_hal_test tests/can_hal_test.cpp)
    target_link_libraries(can_hal_test PRIVATE openrobot_can pthread)
    add_test(NAME can_hal_vcan COMMAND bash
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_can_hal_vcan_test.sh
        $<TARGET_FILE:can_hal_test> $<TARGET_FILE:can_hal> $<TARGET_FILE:shm_logger>)
    add_test(NAME can_hal_lifecycle COMMAND bash
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_can_hal_vcan_test.sh
        python3 ${CMAKE_CURRENT_SOURCE_DIR}/tests/can_hal_lifecycle_test.py
        $<TARGET_FILE:can_hal> $<TARGET_FILE:daemon>)
    add_executable(task_controller_io_test tests/task_controller_io_test.cpp app/task_controller/task_controller.cpp)
    target_link_libraries(task_controller_io_test PRIVATE motion_controllers mujoco_simulation pthread)
    set_target_properties(task_controller_io_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME task_controller_io COMMAND task_controller_io_test
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/control_config.yaml
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml
        $<TARGET_FILE:task_controller>)
    add_executable(magnetic_touch_test tests/magnetic_touch_test.cpp)
    target_link_libraries(magnetic_touch_test PRIVATE motion_controllers mujoco_simulation)
    set_target_properties(magnetic_touch_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME magnetic_touch COMMAND magnetic_touch_test
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/control_config.yaml
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml)
    add_executable(motion_controller_config_test tests/motion_controller_config_test.cpp)
    target_link_libraries(motion_controller_config_test PRIVATE motion_controllers)
    add_test(NAME motion_controller_config COMMAND motion_controller_config_test)
    add_executable(shm_buffers_test tests/shm_buffers_test.cpp)
    target_include_directories(shm_buffers_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(shm_buffers_test PRIVATE pthread)
    add_test(NAME shm_buffers COMMAND shm_buffers_test)
    add_executable(shm_logger_test tests/shm_logger_test.cpp)
    target_include_directories(shm_logger_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(shm_logger_test PRIVATE pthread)
    add_test(NAME shm_logger COMMAND shm_logger_test $<TARGET_FILE:shm_logger>)
    add_executable(daemon_lifecycle_test tests/daemon_lifecycle_test.cpp app/daemon/daemon.cpp)
    target_include_directories(daemon_lifecycle_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(daemon_lifecycle_test PRIVATE pthread)
    add_test(NAME daemon_lifecycle COMMAND daemon_lifecycle_test)
    add_executable(qhrr05_model_test tests/qhrr05_model_test.cpp)
    target_include_directories(qhrr05_model_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include ${MUJOCO_DIR}/include)
    target_link_directories(qhrr05_model_test PRIVATE ${MUJOCO_DIR}/lib)
    target_link_libraries(qhrr05_model_test PRIVATE mujoco)
    set_target_properties(qhrr05_model_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME qhrr05_model COMMAND qhrr05_model_test
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/qhrr-05.xml)
    add_test(NAME qhrr05_scene COMMAND qhrr05_model_test
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml)
    add_executable(homing_control_test tests/homing_control_test.cpp)
    target_link_libraries(homing_control_test PRIVATE motion_controllers mujoco_simulation)
    set_target_properties(homing_control_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME qhrr05_homing COMMAND homing_control_test
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/control_config.yaml)
    add_executable(gravity_compensation_test tests/gravity_compensation_test.cpp)
    target_link_libraries(gravity_compensation_test PRIVATE motion_controllers mujoco_simulation)
    set_target_properties(gravity_compensation_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME qhrr05_gravity COMMAND gravity_compensation_test
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/control_config.yaml)
    add_test(NAME qhrr05_startup_guards COMMAND ${CMAKE_COMMAND}
        -DCONTROLLER=$<TARGET_FILE:task_controller>
        -DSIMULATOR=$<TARGET_FILE:mujoco_simulate>
        -DCONTROL_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/control_config.yaml
        -DMODEL=${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml
        -DDAEMON=$<TARGET_FILE:daemon>
        -DTEST_DIR=${CMAKE_CURRENT_BINARY_DIR}/startup-guards
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_startup_guards.cmake)
endif()
