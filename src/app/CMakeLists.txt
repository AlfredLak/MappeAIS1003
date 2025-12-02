add_executable(BilSimulator main.cpp)
target_compile_features(BilSimulator PRIVATE cxx_std_20)

target_include_directories(BilSimulator PRIVATE
        "${CMAKE_SOURCE_DIR}/include"
)

target_compile_definitions(BilSimulator PRIVATE ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")

target_link_libraries(BilSimulator PRIVATE threepp::threepp assimp::assimp)

if (MINGW)
    target_link_options(BilSimulator PRIVATE -static-libstdc++ -static-libgcc)
endif()


set(_DLL_DIRS
        "${CMAKE_BINARY_DIR}/_deps/assimp-build/bin"
        "${CMAKE_BINARY_DIR}/_deps/assimp-build/lib"
        "${CMAKE_BINARY_DIR}/_deps/threepp-build/_deps/glfw-build/src"
        "${CMAKE_BINARY_DIR}/_deps/threepp-build/bin"
        "${CMAKE_BINARY_DIR}/_deps/threepp-build/lib"
)
foreach(DLLDIR IN LISTS _DLL_DIRS)
    if (EXISTS "${DLLDIR}")
        file(GLOB _DLLS "${DLLDIR}/*.dll")
        foreach(D IN LISTS _DLLS)
            add_custom_command(TARGET BilSimulator POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${D}" $<TARGET_FILE_DIR:BilSimulator>)
        endforeach()
    endif()
endforeach()