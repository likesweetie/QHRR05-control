# All operator-facing inputs and web telemetry belong to the interfaces module.
# Executable names and the Input wire protocol remain independent of source layout.
add_library(interfaces STATIC
    input/joystick.cpp
    input/interface.cpp
    input/client.cpp
    web_gui/bridge.cpp)
target_link_libraries(interfaces PUBLIC qhrr05_common ipc_simulation yaml-cpp pthread)
