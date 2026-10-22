# Makes this library usable directly, instead of needing a wrapper in the
# consuming project: as an ESP-IDF component, or in plain CMake through
# add_subdirectory() or FetchContent as the target algo::path-pursuit.
# Ignored by the Makefile build, which stays the way to build and test this
# library on a host.
if(COMMAND idf_component_register)
    idf_component_register(SRCS "src/path.c"
                           INCLUDE_DIRS "include")
    return()
endif()

add_library(algo-path-pursuit src/path.c)
add_library(algo::path-pursuit ALIAS algo-path-pursuit)
target_include_directories(algo-path-pursuit PUBLIC include)
set_target_properties(algo-path-pursuit PROPERTIES C_STANDARD 99)
if(NOT MSVC)
    target_link_libraries(algo-path-pursuit PRIVATE m)
endif()
