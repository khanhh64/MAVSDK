cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(offboard_forward)

add_executable(offboard_forward
offboard_forward.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(offboard_forward
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(offboard_forward PRIVATE -Wall -Wextra)
else()
    add_compile_options(offboard_forward PRIVATE -WX -W2)
endif()
