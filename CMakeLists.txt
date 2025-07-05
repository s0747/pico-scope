cmake_minimum_required(VERSION 3.5)

set(mylibdir "$ENV{PICO_SDK_PATH}/lib/tinyusb")

include(${mylibdir}/hw/bsp/family_support.cmake)

family_get_project_name(PROJECT ${CMAKE_CURRENT_LIST_DIR})

project(${PROJECT})

# Checks this example is valid for the family and initializes the project
family_initialize_project(${PROJECT} ${CMAKE_CURRENT_LIST_DIR})

add_executable(${PROJECT})

target_sources(${PROJECT} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pico-scope.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/usb_descriptors.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/adc.c
        )

target_link_libraries(${PROJECT} PUBLIC
        pico_stdlib
        pico_multicore
        hardware_adc
        hardware_dma
        hardware_pwm
        )

target_include_directories(${PROJECT} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        )

# Configure compilation flags and libraries for the example without RTOS.
# See the corresponding function in hw/bsp/FAMILY/family.cmake for details.
family_configure_device_example(${PROJECT} noos)
