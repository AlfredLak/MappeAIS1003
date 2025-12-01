cmake_minimum_required(VERSION 3.16)
project(MinBil)
set(CMAKE_CXX_STANDARD 20)

include(FetchContent)

# 1) Fetch assimp
FetchContent_DECLARE(
        assimp
        GIT_REPOSITORY https://github.com/assimp/assimp.git
        GIT_TAG        v6.0.2
)
FetchContent_MakeAvailable(assimp)

# 2) Tell threepp we want assimp BEFORE we fetch threepp
set(THREEPP_ENABLE_ASSIMP ON)
set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)

# 3) Fetch threepp
FetchContent_DECLARE(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG        619cdaacc2e7dc6ea709d8c7f482e2994af7349d
)
FetchContent_MakeAvailable(threepp)

# 4) Our executable
add_executable(MinBil main.cpp)

# 5) Link against threepp and assimp
target_link_libraries(MinBil
        PUBLIC threepp::threepp
        PRIVATE assimp
)

# copy assets
file(GLOB GLB_FILES "${CMAKE_SOURCE_DIR}/*.glb")
file(COPY ${GLB_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY ${CMAKE_SOURCE_DIR}/startscreen_test.png
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# copy all DLLs from build/bin to the exe dir (works for Debug/Release)
set(RUNTIME_DLL_DIR "${CMAKE_CURRENT_BINARY_DIR}/bin")
file(GLOB RUNTIME_DLLS "${RUNTIME_DLL_DIR}/*.dll")


foreach(dll ${RUNTIME_DLLS})
    add_custom_command(TARGET MinBil POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${dll}"
            $<TARGET_FILE_DIR:MinBil>)
endforeach()