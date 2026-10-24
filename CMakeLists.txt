cmake_minimum_required(VERSION 3.20)

project(RobotSimulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG 5.5
        GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(raylib)

add_executable(RobotSimulator main.cpp)

target_link_libraries(RobotSimulator PRIVATE raylib)