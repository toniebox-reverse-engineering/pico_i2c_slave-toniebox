cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(i2c_emu_mma8452q C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(i2c_slave)
add_subdirectory(i2c-emu-mma8452q)
