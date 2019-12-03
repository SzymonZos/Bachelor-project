cmake_minimum_required(VERSION 3.15)
project(MPC_PC)

set(CMAKE_CXX_STANDARD 17)
set(ENTRY_POINT main.cpp)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(include)
add_subdirectory(src)

add_executable(MPC_PC ${ENTRY_POINT})
target_link_libraries(MPC_PC src)