cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# >>> C++ settings (ADD)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# <<<

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME F405RGT6Project)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
# >>> REPLACE: enable CXX here
project(${CMAKE_PROJECT_NAME} LANGUAGES C CXX ASM)
# <<<
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# ---------------------------------------------------------------------------
# Keep only main.cpp: ignore CubeMX-referenced main.c (may not exist)
# MUST be placed before add_subdirectory(cmake/stm32cubemx)
set(MX_MAIN_C ${CMAKE_SOURCE_DIR}/Core/Src/main.c)
set_source_files_properties(${MX_MAIN_C} PROPERTIES
    GENERATED TRUE
    HEADER_FILE_ONLY TRUE
)

# Keep only usbd_cdc_if.cpp: ignore CubeMX-referenced usbd_cdc_if.c
# MUST be placed before add_subdirectory(cmake/stm32cubemx)
set(MX_USBD_CDC_IF_C ${CMAKE_SOURCE_DIR}/USB_DEVICE/App/usbd_cdc_if.c)
set_source_files_properties(${MX_USBD_CDC_IF_C} PROPERTIES
    GENERATED TRUE
    HEADER_FILE_ONLY TRUE
)
# ---------------------------------------------------------------------------

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# ---------------------------------------------------------------------------
# Global include collection to allow: #include "can.h" anywhere
# It will add every directory that contains headers under bsp/module/application
# ---------------------------------------------------------------------------
function(collect_header_dirs out_var base_dir)
    file(GLOB_RECURSE _hdrs CONFIGURE_DEPENDS
        ${base_dir}/*.h
        ${base_dir}/*.hpp
        ${base_dir}/*.hh
        ${base_dir}/*.hxx
    )
    set(_dirs "")
    foreach(_h ${_hdrs})
        get_filename_component(_d "${_h}" DIRECTORY)
        list(APPEND _dirs "${_d}")
    endforeach()
    list(REMOVE_DUPLICATES _dirs)
    set(${out_var} ${_dirs} PARENT_SCOPE)
endfunction()

add_library(project_includes INTERFACE)

collect_header_dirs(BSP_HDR_DIRS  "${CMAKE_SOURCE_DIR}/Usercode//1_bsp")
collect_header_dirs(MOD_HDR_DIRS  "${CMAKE_SOURCE_DIR}/Usercode//2_module")
collect_header_dirs(APP_HDR_DIRS  "${CMAKE_SOURCE_DIR}/Usercode//3_application")

target_include_directories(project_includes INTERFACE
    ${BSP_HDR_DIRS}
    ${MOD_HDR_DIRS}
    ${APP_HDR_DIRS}
)
# ---------------------------------------------------------------------------

# Add layered libraries
add_subdirectory(Usercode/1_bsp)
add_subdirectory(Usercode/2_module)
add_subdirectory(Usercode/3_application)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Core/Src/main.cpp
    ${CMAKE_SOURCE_DIR}/USB_DEVICE/App/usbd_cdc_if.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
list(REMOVE_ITEM CMAKE_CXX_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
# Use --start-group/--end-group to tolerate "anyone calls anyone" among static libs
target_link_libraries(${CMAKE_PROJECT_NAME}
    project_includes

    -Wl,--start-group
    application
    module
    bsp
    stm32cubemx
    -Wl,--end-group

    # Add user defined libraries
)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u _printf_float")
