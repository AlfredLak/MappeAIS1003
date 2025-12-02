include(FetchContent)
FetchContent_DECLARE(Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.6.0
)
FetchContent_MakeAvailable(Catch2)

enable_testing()

add_executable(BilsimTest
        test_camera.cpp
        test_physics.cpp
        test_input.cpp
        test_powerups.cpp
        test_rig.cpp
        test_visuals.cpp
        test_trees.cpp
        test_game.cpp
)

target_link_libraries(BilsimTest PRIVATE Catch2::Catch2WithMain minbil_core)
target_compile_definitions(BilsimTest PRIVATE ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")

include(CTest)
include(Catch)
catch_discover_tests(BilsimTest)