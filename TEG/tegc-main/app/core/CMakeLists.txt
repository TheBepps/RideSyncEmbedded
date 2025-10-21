message("[CORE]")

set(CDIR ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DIR}/bin/core)

include_directories(
  ${EXTERNAL_DIRS}
  ${CDIR}/src
)

## Libraries
add_library(error
    ${CDIR}/src/errors/error.cpp)
add_library(utils
    ${CDIR}/src/utils/utils.cpp
    ${CDIR}/src/utils/shared_data.cpp)
add_library(window
    ${CDIR}/src/window/window.cpp)
target_link_libraries(window
    PUBLIC
    glfw
    OpenGL::GL
    ${GLEW_LIBRARIES}
    GLEW::GLEW
    imgui
    implot
)
add_library(serial
    ${CDIR}/src/serial/serial.cpp)
target_link_libraries(serial
    PUBLIC
    error)

add_library(modals
    ${CDIR}/src/modals/serial_port_modal.cpp)
target_link_libraries(modals PUBLIC serial utils)
add_library(tabs
    ${CDIR}/src/tabs/tab_serial_port_manager.cpp)
target_link_libraries(tabs PUBLIC serial utils)

## Executables
add_executable(app ${CDIR}/executables/app/main.cpp)
target_link_libraries(app
    window
    serial
    modals
    tabs
)
