cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(offboard_start_stop)

add_executable(offboard_start_stop
offboard_start_stop.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(offboard_start_stop
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(offboard_start_stop PRIVATE -Wall -Wextra)
else()
    add_compile_options(offboard_start_stop PRIVATE -WX -W2)
endif()
