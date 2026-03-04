cmake_minimum_required(VERSION 3.22)

file(GLOB_RECURSE BSP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/*.c
    ${CMAKE_CURRENT_LIST_DIR}/*.cpp
)

if(BSP_SOURCES)
    add_library(bsp STATIC ${BSP_SOURCES})
else()
    set(BSP_DUMMY ${CMAKE_CURRENT_BINARY_DIR}/bsp_dummy.c)
    file(WRITE ${BSP_DUMMY}
        "/* Auto-generated placeholder (no sources in bsp/ yet) */\n"
        "void __bsp_dummy(void) {}\n"
    )
    add_library(bsp STATIC ${BSP_DUMMY})
endif()

target_link_libraries(bsp PUBLIC
    project_includes
    stm32cubemx
)
