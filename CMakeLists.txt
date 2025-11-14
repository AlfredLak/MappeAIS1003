cmake_minimum_required(VERSION 3.16)
project(MinBil)
set(CMAKE_CXX_STANDARD 20)

include(FetchContent)

FetchContent_DECLARE(
        assimp
        GIT_REPOSITORY https://github.com/assimp/assimp.git
        GIT_TAG        v6.0.2
)
FetchContent_MakeAvailable(assimp)

set(THREEPP_ENABLE_ASSIMP ON)
set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)

FetchContent_DECLARE(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG        619cdaacc2e7dc6ea709d8c7f482e2994af7349d
)
FetchContent_MakeAvailable(threepp)

add_executable(MinBil main.cpp)

target_link_libraries(MinBil
        PUBLIC threepp::threepp
        PRIVATE assimp
)

file(COPY ${CMAKE_SOURCE_DIR}/suv_model.glb
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})