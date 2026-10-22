# algo-motion

<p align="center">
  <img alt="platform: portable C99" src="https://img.shields.io/badge/platform-portable%20C99-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

Motion for robots — getting from here to there, in portable C99, one header
and one `.c` file per library, no heap.

## Libraries

| Library | What it does | Pick it when |
|---|---|---|
| [path-pursuit](path-pursuit) | Waypoints to a smooth path, and a pure-pursuit tracker that follows it | a rover has a list of points to visit |
| [trajectory-profile](trajectory-profile) | Trapezoidal and S-curve motion profiles — **planned**, not written yet | — |

## Use one library

Every library stands on its own: no shared headers, nothing from one library
needed by another. Take the one you need and leave the rest. Three ways in.

**Copy the files.** `path-pursuit/include/path.h` and `path-pursuit/src/path.c`
into your project, built with your own sources. That is the whole install.

**CMake.** Every library is its own target, `algo::<folder>`. Add just the
folder you need:

```cmake
add_subdirectory(algo-motion/path-pursuit)
target_link_libraries(app PRIVATE algo::path-pursuit)
```

or, without a copy in your tree:

```cmake
include(FetchContent)
FetchContent_Declare(algo_path_pursuit
    GIT_REPOSITORY https://github.com/JonasAtelier/algo-motion.git
    GIT_TAG        stable
    SOURCE_SUBDIR  path-pursuit)
FetchContent_MakeAvailable(algo_path_pursuit)
target_link_libraries(app PRIVATE algo::path-pursuit)
```

`add_subdirectory(algo-motion)`, or FetchContent without `SOURCE_SUBDIR`, adds
every library's target instead; link the ones you use. Examples and
tests only build when this repo is the top-level project.

**ESP-IDF.** Point `EXTRA_COMPONENT_DIRS` at one library's folder. The
component is named after the folder:

```cmake
# top-level CMakeLists.txt of your IDF project, before project()
list(APPEND EXTRA_COMPONENT_DIRS path/to/algo-motion/path-pursuit)
```

then `REQUIRES path-pursuit` in your component's `idf_component_register()`.

Pointing it at the repo root instead gives you **one** component holding
every library, named after the folder (`algo-motion`): IDF does not look
inside a folder that has its own `CMakeLists.txt`. Use one or the other,
never both — every symbol would be defined twice.

## Build and test

```sh
make check    # every library's own tests, stopping at the first failure
make clean
```

Or with CMake and CTest. CTest runs the C tests only; the C++ linkage tests
run under `make check`.

```sh
cmake -B build && cmake --build build -j2 && ctest --test-dir build
```

## License

MIT. See [LICENSE](LICENSE).
