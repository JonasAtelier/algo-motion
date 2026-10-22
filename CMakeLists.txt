# One repo, several independent libraries. In plain CMake each one is its own
# target - algo::<folder>, e.g. algo::path-pursuit -
# and you link only what you use.
#
# ESP-IDF takes a folder listed in EXTRA_COMPONENT_DIRS that has a
# CMakeLists.txt as ONE component and does not search its subfolders. So
# pointing IDF at this repo root gives a single component, named after the
# folder, holding every library below. Point it at one library's folder to
# get just that one. Never both: every symbol would be defined twice.
set(ALGO_LIBS path-pursuit)

if(COMMAND idf_component_register)
    list(TRANSFORM ALGO_LIBS APPEND /src OUTPUT_VARIABLE srcs)
    list(TRANSFORM ALGO_LIBS APPEND /include OUTPUT_VARIABLE incs)
    idf_component_register(SRC_DIRS ${srcs}
                           INCLUDE_DIRS ${incs})
    return()
endif()

cmake_minimum_required(VERSION 3.16)
project(algo_motion C)

# Examples and tests are for working on this repo. A project that pulls it in
# with add_subdirectory() or FetchContent gets only the library targets.
string(COMPARE EQUAL "${CMAKE_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}" top)
option(ALGO_BUILD_EXAMPLES "Build every library's examples" ${top})
option(ALGO_BUILD_TESTS "Build every library's tests for CTest" ${top})
if(ALGO_BUILD_TESTS)
    enable_testing()
endif()

foreach(lib IN LISTS ALGO_LIBS)
    add_subdirectory(${lib})
    if(ALGO_BUILD_EXAMPLES)
        file(GLOB srcs ${lib}/examples/*.c)
        foreach(src IN LISTS srcs)
            get_filename_component(name ${src} NAME_WE)
            add_executable(${lib}-${name} ${src})
            target_link_libraries(${lib}-${name} PRIVATE algo::${lib})
        endforeach()
    endif()
    if(ALGO_BUILD_TESTS)
        file(GLOB srcs ${lib}/test/*.c)
        foreach(src IN LISTS srcs)
            get_filename_component(name ${src} NAME_WE)
            add_executable(${name} ${src})
            target_link_libraries(${name} PRIVATE algo::${lib})
            add_test(NAME ${name} COMMAND ${name})
        endforeach()
    endif()
endforeach()
