cmake_minimum_required(VERSION 3.28)
project(Propulsion_2024)

set(CMAKE_CXX_STANDARD 14)
add_compile_options("-DMOCK_RPI")

include_directories(.
        lib)

add_executable(Propulsion_2024
        lib/Command.h
        lib/Command_Interpreter.cpp
        lib/Command_Interpreter.h
        lib/Main.cpp
        old-lib/Thruster_Commander.cpp
        old-lib/Thruster_Commander.h
        lib/Mock_GPIO.cpp
        lib/Mock_GPIO.h
        lib/Wiring.cpp
        lib/Wiring.h
        old-lib/LQR_Control.h
        old-lib/Physics.h "old-lib/Physics.cpp")
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
        old-lib/Physics.h
        lib/Command_Interpreter.cpp
        lib/Command_Interpreter.h
        old-lib/Thruster_Commander.cpp
        old-lib/Thruster_Commander.h
        lib/Mock_GPIO.cpp
        lib/Mock_GPIO.h
        lib/Wiring.cpp
        lib/Wiring.h "old-lib/Physics.cpp")
  #"lib/LQR_Control.h" "lib/Physics.h" "testing/Physics_Testing.cpp")
target_link_libraries(
        run_tests
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(run_tests)
#Gtest end

#YAML start
FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG master # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_MakeAvailable(yaml-cpp)

target_link_libraries(Propulsion_2024 PUBLIC yaml-cpp::yaml-cpp) # The library or executable that require yaml-cpp library
#YAML end

configure_file(${PROJECT_SOURCE_DIR}/data/14V_Correlation.csv ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
