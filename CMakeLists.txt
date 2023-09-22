cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(motorFRTOS)

pico_sdk_init()


add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()


add_subdirectory(FreeRTOS)
add_subdirectory(Motorp_FRTOS)
