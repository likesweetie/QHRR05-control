cmake_minimum_required(VERSION 3.16)
project(QHRR05 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(MUJOCO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco)

find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

add_library(homing_control STATIC app/task_controller/homing_controller.cpp)
target_include_directories(homing_control PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${MUJOCO_DIR}/include)
target_link_directories(homing_control PUBLIC ${MUJOCO_DIR}/lib)
target_link_libraries(homing_control PUBLIC yaml-cpp mujoco)

add_executable(daemon
    app/daemon/main.cpp
    app/daemon/daemon.cpp
)
target_include_directories(daemon PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(daemon PRIVATE
    homing_control
    pthread
)
target_compile_definitions(daemon PRIVATE
    QHRR05_DEFAULT_DAEMON_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/app_config/daemon_config.yaml")

add_executable(task_controller
    app/task_controller/main.cpp
)
target_include_directories(task_controller PRIVATE ${GLFW_INCLUDE_DIRS})
target_link_libraries(task_controller PRIVATE homing_control ${GLFW_LIBRARIES} GL pthread)
set_target_properties(task_controller daemon PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")

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
    target_link_libraries(homing_control_test PRIVATE homing_control)
    set_target_properties(homing_control_test PROPERTIES BUILD_RPATH "${MUJOCO_DIR}/lib")
    add_test(NAME qhrr05_homing COMMAND homing_control_test
        ${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/config/control_config/control_config.yaml)
    add_test(NAME qhrr05_startup_guards COMMAND ${CMAKE_COMMAND}
        -DCONTROLLER=$<TARGET_FILE:task_controller>
        -DCONTROL_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/config/control_config/control_config.yaml
        -DMODEL=${CMAKE_CURRENT_SOURCE_DIR}/models/qhrr-05/scene.xml
        -DDAEMON=$<TARGET_FILE:daemon>
        -DTEST_DIR=${CMAKE_CURRENT_BINARY_DIR}/startup-guards
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_startup_guards.cmake)
endif()
