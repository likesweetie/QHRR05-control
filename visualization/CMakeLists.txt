if(NOT MUJOCO_USE_FILAMENT OR MUJOCO_USE_FILAMENT_MJR_COMPAT)
    message(FATAL_ERROR "Viewer requires Filament ON and MJR compatibility OFF")
endif()
if(NOT mujoco_VERSION VERSION_EQUAL "3.16.0" AND NOT PROJECT_VERSION VERSION_EQUAL "3.16.0")
    message(FATAL_ERROR "Viewer supports the pinned MuJoCo 3.16.0 source only")
endif()
find_package(PkgConfig REQUIRED)
pkg_check_modules(QHRR_GLFW REQUIRED IMPORTED_TARGET glfw3)
find_package(OpenGL REQUIRED)
find_package(Vulkan REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
# Clang 18 diagnoses this old SPIRV-Cross lambda syntax. Keep the warning visible
# without promoting this third-party C++20 deprecation to a build failure.
foreach(target IN ITEMS spirv-cross-msl spirv-cross-glsl spirv-cross-core spirv-cross-cpp
        spirv-cross-reflect spirv-cross-util)
    if(TARGET ${target})
        target_compile_options(${target} PRIVATE -Wno-error=deprecated-this-capture)
    endif()
endforeach()
# Motion numerics remain a private implementation of the controller module.
add_library(visualization_robot_model STATIC
    "${QHRR05_SOURCE_DIR}/src/robot_model/mujoco_backend.cpp")
target_include_directories(visualization_robot_model PUBLIC "${QHRR05_SOURCE_DIR}/src")
target_link_libraries(visualization_robot_model PUBLIC mujoco yaml-cpp)
target_compile_features(visualization_robot_model PUBLIC cxx_std_17)
add_library(visualization_motion_controllers STATIC
    "${QHRR05_SOURCE_DIR}/src/motion_controllers/ball_following_controller.cpp"
    "${QHRR05_SOURCE_DIR}/src/motion_controllers/numerics/math.cpp")
target_include_directories(visualization_motion_controllers PUBLIC "${QHRR05_SOURCE_DIR}/src")
target_link_libraries(visualization_motion_controllers PUBLIC visualization_robot_model)

add_executable(qhrr_visualizer "${CMAKE_CURRENT_LIST_DIR}/main.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/renderer.cpp" "${CMAKE_CURRENT_LIST_DIR}/snapshot.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/operator_panel.cpp"
    "${QHRR05_SOURCE_DIR}/src/ipc/simulation/channel.cpp"
    "${QHRR05_SOURCE_DIR}/src/interfaces/input/client.cpp"
    "${MUJOCO_DEAR_IMGUI_SOURCE_DIR}/backends/imgui_impl_glfw.cpp")
target_compile_features(qhrr_visualizer PRIVATE cxx_std_20)
target_include_directories(qhrr_visualizer PRIVATE "${QHRR05_SOURCE_DIR}/src")
target_link_libraries(qhrr_visualizer PRIVATE mujoco mujoco::filament dear_imgui_OpenGL3 yaml-cpp
    visualization_motion_controllers PkgConfig::QHRR_GLFW OpenGL::GL Vulkan::Vulkan Threads::Threads)
target_compile_definitions(qhrr_visualizer PRIVATE
    QHRR_DEFAULT_CONTROL_CONFIG="${QHRR05_SOURCE_DIR}/config/control/control.yaml")
set_target_properties(qhrr_visualizer PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(QHRR_PBR_SHADER "${CMAKE_BINARY_DIR}/src/render/filament/assets/qhrr_pbr.filamat")
add_custom_command(OUTPUT "${QHRR_PBR_SHADER}"
    COMMAND $<TARGET_FILE:matc> --platform=all --api=vulkan --api=opengl
        --variant-filter skinning --optimize-size --output "${QHRR_PBR_SHADER}"
        "${CMAKE_CURRENT_LIST_DIR}/materials/brushed_pbr.mat"
    DEPENDS matc "${CMAKE_CURRENT_LIST_DIR}/materials/brushed_pbr.mat"
    COMMENT "Compiling QHRR brushed-metal material")
add_custom_target(qhrr_materials DEPENDS "${QHRR_PBR_SHADER}")
add_dependencies(qhrr_visualizer mujoco_filament_assets qhrr_materials)
add_custom_command(TARGET qhrr_visualizer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_BINARY_DIR}/src/render/filament/assets" "$<TARGET_FILE_DIR:qhrr_visualizer>/assets")
add_executable(visualization_snapshot_test "${QHRR05_SOURCE_DIR}/tests/visualization_snapshot_test.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/snapshot.cpp" "${QHRR05_SOURCE_DIR}/src/ipc/simulation/channel.cpp")
target_include_directories(visualization_snapshot_test PRIVATE "${QHRR05_SOURCE_DIR}/src" "${CMAKE_CURRENT_LIST_DIR}")
target_compile_features(visualization_snapshot_test PRIVATE cxx_std_20)
target_link_libraries(visualization_snapshot_test PRIVATE Threads::Threads)
set_target_properties(visualization_snapshot_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# Exercise the same mathematical backend against the pinned visualization MuJoCo API.
add_executable(visualization_model_test "${QHRR05_SOURCE_DIR}/tests/robot_model_test.cpp")
target_include_directories(visualization_model_test PRIVATE "${QHRR05_SOURCE_DIR}/src")
target_link_libraries(visualization_model_test PRIVATE visualization_robot_model)
target_compile_features(visualization_model_test PRIVATE cxx_std_17)
set_target_properties(visualization_model_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
