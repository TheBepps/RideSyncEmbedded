set(CDIR ${CMAKE_CURRENT_LIST_DIR})

FILE(GLOB imgui_srcs ${CDIR}/imgui/*)
FILE(GLOB implot_srcs ${CDIR}/implot/*)

add_library(
    imgui

    # GUI
    ${imgui_srcs}
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends/imgui_impl_glfw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends/imgui_impl_opengl2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/misc/cpp/imgui_stdlib.cpp    
)

add_library(
    implot
    
    # PLOT
    ${implot_srcs}
    ${CMAKE_CURRENT_SOURCE_DIR}/implot/implot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/implot/implot_items.cpp
)
