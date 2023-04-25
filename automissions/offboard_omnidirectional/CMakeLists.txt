cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(offboard_omnidirectional)

add_executable(offboard_omnidirectional
offboard_omnidirectional.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(offboard_omnidirectional
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(offboard_oomnidirectional PRIVATE -Wall -Wextra)
else()
    add_compile_options(offboard_omnidirectional PRIVATE -WX -W2)
endif()
