cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(offboard_orthogonal)

add_executable(offboard_orthogonal
offboard_orthogonal.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(offboard_orthogonal
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(offboard_orthogonal PRIVATE -Wall -Wextra)
else()
    add_compile_options(offboard_orthogonal PRIVATE -WX -W2)
endif()
