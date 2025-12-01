add_executable(BilSimulator main.cpp)
target_link_libraries(BilSimulator PRIVATE minbil_core)

# Copy assets next to the exe
add_custom_command(TARGET BilSimulator POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/assets $<TARGET_FILE_DIR:BilSimulator>/assets)

# On Windows, copy DLLs if needed (adjust your pathing if different)
if (WIN32)
    file(GLOB RUNTIME_DLLS "${CMAKE_BINARY_DIR}/bin/*.dll")
    foreach(dll ${RUNTIME_DLLS})
        add_custom_command(TARGET BilSimulator POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${dll}" $<TARGET_FILE_DIR:BilSimulator>)
    endforeach()
endif()