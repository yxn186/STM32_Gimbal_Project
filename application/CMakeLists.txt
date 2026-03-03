cmake_minimum_required(VERSION 3.22)

# ============================================================
# application layer: MANUAL source list
# 说明：
# - 新增/删除 app 层源文件时，需要在下面的 APP_SOURCES 里手动维护
# ============================================================

set(APP_SOURCES
    # ---- application 业务代码（手动添加）----
    #
    #       ！！！使用方法！！！
    #
    #   假设你新建了：application/gimbal/app_gimbal.c
    #   在文件系统里创建 .c/.cpp（你照常写代码）
    #   打开 application/CMakeLists.txt
    #   把文件路径加到 APP_SOURCES：
    #   ${CMAKE_CURRENT_LIST_DIR}/gimbal/app_gimbal.c

    #${CMAKE_CURRENT_LIST_DIR}/W25Q64/app_W25Q64.c
    ${CMAKE_CURRENT_LIST_DIR}/bmi088/app_bmi088.c
    ${CMAKE_CURRENT_LIST_DIR}/bmi088/app_bmi088_math.c
)

# ============================================================
# 空源文件保护：避免 add_library(application STATIC) 报错
# ============================================================
if(APP_SOURCES)
    add_library(application STATIC ${APP_SOURCES})
else()
    set(APP_DUMMY ${CMAKE_CURRENT_BINARY_DIR}/application_dummy.c)
    file(WRITE ${APP_DUMMY}
        "/* Auto-generated placeholder (no sources in application/ yet) */\n"
        "void __application_dummy(void) {}\n"
    )
    add_library(application STATIC ${APP_DUMMY})
endif()

target_link_libraries(application PUBLIC
    project_includes
    stm32cubemx
)