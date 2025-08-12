cmake_minimum_required(VERSION 3.28)
project(Propulsion_2024)

set(CMAKE_CXX_STANDARD 20)

# Optional mocking flag
option(MOCK_RPI "Use mock Pico" OFF)

if (MOCK_RPI)
    add_compile_options("-DMOCK_RPI")
endif()

# Include paths
include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/testing
)

# Enable GTest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/d144031940543e15423a25ae5a8a74141044862f.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

# Define test target
add_executable(propulsion_test
    testing/Command_Interpreter_Testing.cpp
    src/Command.hpp
    src/Command_Interpreter.cpp
    src/Command_Interpreter.hpp
    src/Wiring.cpp
    src/Wiring.hpp
    src/Serial.cpp
    src/Serial.hpp
)

# Always link GTest
target_link_libraries(propulsion_test GTest::gtest_main)

add_library(PropulsionFunctions
        src/Command.hpp
        src/Command_Interpreter.cpp
        src/Command_Interpreter.hpp
        src/Wiring.cpp
        src/Wiring.hpp
        src/Serial.cpp
        src/Serial.hpp
)
include(GoogleTest)

gtest_discover_tests(propulsion_test)

