cmake_minimum_required(VERSION 3.21)

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

target_include_directories(BilsimTest PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_compile_definitions(BilsimTest PRIVATE
        ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets"
)

target_link_libraries(BilsimTest
        PRIVATE
        Catch2::Catch2WithMain
        threepp
        assimp
)

if (WIN32)
    add_custom_command(TARGET BilsimTest POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:BilsimTest> $<TARGET_FILE_DIR:BilsimTest>
            COMMAND_EXPAND_LISTS
    )
endif()

include(CTest)
if (BUILD_TESTING)
    add_test(NAME BilsimTest_All
            COMMAND BilsimTest --durations yes)
endif()