cmake_minimum_required(VERSION 3.28)

include(pico_sdk_import.cmake)

project(things-pico C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if ("$ENV{PICO_SDK_PATH}" STREQUAL "")
	message(FATAL_ERROR "Setup PICO_SDK_PATH")
endif()

# Initialize the SDK
pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

add_subdirectory(scope)
add_subdirectory(adc_console)


