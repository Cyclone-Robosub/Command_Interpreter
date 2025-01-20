cmake_minimum_required(VERSION 3.28)
project(Propulsion_2024)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options("-DMOCK_RPI")

include_directories(.
        lib)

add_executable(Propulsion_2024
        lib/Command.h
        lib/Command_Interpreter.cpp
        lib/Command_Interpreter.h
        lib/Main.cpp
        lib/Thruster_Commander.cpp
        lib/Thruster_Commander.h
        lib/LQR_Control.h
        lib/Physics.h "lib/Physics.cpp")
  #"lib/LQR_Control.h" "lib/Physics.h" "testing/Physics_Testing.cpp")

# GTest start
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/d144031940543e15423a25ae5a8a74141044862f.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
        run_tests
        testing/Command_Interpreter_Testing.cpp
        testing/Thruster_Commander_Testing.cpp
        testing/Physics_Testing.cpp
        lib/Command.h
        lib/Physics.h
        lib/Command_Interpreter.cpp
        lib/Command_Interpreter.h
        lib/Thruster_Commander.cpp
        lib/Thruster_Commander.h "lib/Physics.cpp")
  #"lib/LQR_Control.h" "lib/Physics.h" "testing/Physics_Testing.cpp")
target_link_libraries(
        run_tests
        GTest::gtest_main
)

add_subdirectory(yaml-cpp)
target_link_libraries(Propulsion_2024 yaml-cpp)
target_link_libraries(run_tests yaml-cpp)
include(GoogleTest)
gtest_discover_tests(run_tests)
#Gtest end


configure_file(${PROJECT_SOURCE_DIR}/data/14V_Correlation.csv ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
