cmake_minimum_required(VERSION 3.22)

set(USER_INCLUDE_DIRS
        ${CMAKE_SOURCE_DIR}/User/Inc
)
file(GLOB USER_SRC_DIRS
        ${CMAKE_SOURCE_DIR}/User/Src/*.c
)

add_library(User OBJECT)
target_sources(User PRIVATE ${USER_SRC_DIRS})
target_include_directories(User PUBLIC ${USER_INCLUDE_DIRS})
target_link_libraries(User PUBLIC stm32cubemx)