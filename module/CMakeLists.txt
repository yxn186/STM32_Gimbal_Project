cmake_minimum_required(VERSION 3.22)

file(GLOB_RECURSE MOD_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/*.c
    ${CMAKE_CURRENT_LIST_DIR}/*.cpp
)

if(MOD_SOURCES)
    add_library(module STATIC ${MOD_SOURCES})
else()
    set(MOD_DUMMY ${CMAKE_CURRENT_BINARY_DIR}/modules_dummy.c)
    file(WRITE ${MOD_DUMMY}
        "/* Auto-generated placeholder (no sources in module/ yet) */\n"
        "void __modules_dummy(void) {}\n"
    )
    add_library(module STATIC ${MOD_DUMMY})
endif()

target_link_libraries(module PUBLIC
    project_includes
    stm32cubemx
)
