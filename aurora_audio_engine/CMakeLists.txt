# ============================================================================
#  Aurora Audio Engine - Top-level CMake build script
#
#  A platform-independent, modular, real-time-safe audio kernel prototype.
#  Requires a C++20 toolchain (concepts, std::jthread, std::span, std::atomic,
#  constexpr, designated initializers).
# ============================================================================
cmake_minimum_required(VERSION 3.20)

project(AuroraAudioEngine
    VERSION 0.1.0
    DESCRIPTION "Platform-independent, science-based real-time audio kernel"
    LANGUAGES CXX)

# ----------------------------------------------------------------------------
#  Global C++ standard settings
# ----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ----------------------------------------------------------------------------
#  Platform detection
#
#  The HAL uses these definitions to select an appropriate audio backend.
#  Only the NullBackend is compiled for every platform in this prototype;
#  real backends (WASAPI / CoreAudio / ALSA-PipeWire) are stubbed out and
#  documented for future work.
# ----------------------------------------------------------------------------
if(WIN32)
    set(AURORA_PLATFORM "Windows")
    set(AURORA_PLATFORM_WINDOWS ON)
elseif(APPLE)
    set(AURORA_PLATFORM "macOS")
    set(AURORA_PLATFORM_MACOS ON)
elseif(UNIX)
    set(AURORA_PLATFORM "Linux")
    set(AURORA_PLATFORM_LINUX ON)
else()
    set(AURORA_PLATFORM "Unknown")
endif()

message(STATUS "Aurora: target platform = ${AURORA_PLATFORM}")
message(STATUS "Aurora: build type       = ${CMAKE_BUILD_TYPE}")

# ----------------------------------------------------------------------------
#  Compiler flags
#
#  We enable aggressive optimization and auto-vectorization for release builds
#  so the SIMD-ready DSP loops are actually vectorized by the compiler.
# ----------------------------------------------------------------------------
if(MSVC)
    add_compile_options(/W4 /permissive- /Zc:__cplusplus)
    add_compile_options($<$<CONFIG:Release>:/O2>)
    add_compile_options($<$<CONFIG:Release>:/fp:fast>)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    add_compile_options($<$<CONFIG:Release>:-O3>)
    add_compile_options($<$<CONFIG:Release>:-ffast-math>)
    # -march=native gives the auto-vectorizer the widest SIMD width available.
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag("-march=native" AURORA_HAS_MARCH_NATIVE)
    if(AURORA_HAS_MARCH_NATIVE)
        add_compile_options($<$<CONFIG:Release>:-march=native>)
    endif()
endif()

# ----------------------------------------------------------------------------
#  Aurora core library
#
#  Most of the engine is header-only (template-heavy DSP + lock-free
#  containers). Only a handful of translation units need compiling.
# ----------------------------------------------------------------------------
add_library(aurora_core STATIC
    src/core/engine.cpp
    src/dsp/audio_graph.cpp
    src/dsp/basic_nodes.cpp
    src/hal/null_backend.cpp)

target_include_directories(aurora_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_compile_features(aurora_core PUBLIC cxx_std_20)

# Threads (std::jthread / std::thread) require pthread on POSIX platforms.
find_package(Threads REQUIRED)
target_link_libraries(aurora_core PUBLIC Threads::Threads)

# Propagate platform macros so HAL code can specialize at compile time.
if(AURORA_PLATFORM_WINDOWS)
    target_compile_definitions(aurora_core PUBLIC AURORA_PLATFORM_WINDOWS=1)
elseif(AURORA_PLATFORM_MACOS)
    target_compile_definitions(aurora_core PUBLIC AURORA_PLATFORM_MACOS=1)
elseif(AURORA_PLATFORM_LINUX)
    target_compile_definitions(aurora_core PUBLIC AURORA_PLATFORM_LINUX=1)
endif()

# ----------------------------------------------------------------------------
#  Examples
# ----------------------------------------------------------------------------
add_executable(synth_demo examples/synth_demo.cpp)
target_link_libraries(synth_demo PRIVATE aurora_core)

# ----------------------------------------------------------------------------
#  Install rules (headers + static lib)
# ----------------------------------------------------------------------------
install(TARGETS aurora_core ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
