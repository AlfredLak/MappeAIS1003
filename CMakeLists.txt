cmake_minimum_required(VERSION 3.16)
project(MinBil CXX)
set(CMAKE_CXX_STANDARD 20)

include(FetchContent)

# --- Dependencies: assimp + threepp ---
FetchContent_DECLARE(assimp
        GIT_REPOSITORY https://github.com/assimp/assimp.git
        GIT_TAG        v6.0.2
)
FetchContent_MakeAvailable(assimp)

set(THREEPP_ENABLE_ASSIMP ON)
set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
FetchContent_DECLARE(threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG        619cdaacc2e7dc6ea709d8c7f482e2994af7349d
)
FetchContent_MakeAvailable(threepp)

# Header-only core as an INTERFACE library
add_library(minbil_core INTERFACE)
target_include_directories(minbil_core INTERFACE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(minbil_core INTERFACE threepp::threepp assimp)
target_compile_definitions(minbil_core INTERFACE ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")

add_subdirectory(src)
add_subdirectory(tests)