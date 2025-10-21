cmake_minimum_required(VERSION 3.16)
project(Telemetry)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -Wall -Wextra -Werror")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG -w")

set(DIR ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_compile_definitions("ImDrawIdx=unsigned int")

find_package(GLEW      REQUIRED)
find_package(OpenGL    REQUIRED)
find_package(glfw3 3.3 REQUIRED)

include_directories(
    ${DIR}

    # IMGUI - IMPLOT
    ${DIR}/external/imgui
    ${DIR}/external/imgui/misc/cpp
    ${DIR}/external/imgui/backends
    ${DIR}/external/implot

    # GLEW
    ${GLEW_INCLUDE_DIRS}
)

add_subdirectory(${DIR}/external)
add_subdirectory(${DIR}/core)
