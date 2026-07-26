cmake_minimum_required(VERSION 3.16)
project(stm32_algorithm_library C)

add_library(stm32_algorithms STATIC
    kalman/kalman.c
    pid/pid.c
    ring_buffer/ring_buffer.c
)
target_include_directories(stm32_algorithms PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/kalman"
    "${CMAKE_CURRENT_SOURCE_DIR}/pid"
    "${CMAKE_CURRENT_SOURCE_DIR}/ring_buffer"
)
target_compile_features(stm32_algorithms PUBLIC c_std_99)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(stm32_algorithms PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion
    )
endif()

include(CTest)
if(BUILD_TESTING AND NOT CMAKE_CROSSCOMPILING)
    add_executable(algorithm_tests tests/test_algorithms.c)
    target_link_libraries(algorithm_tests PRIVATE stm32_algorithms)
    if(UNIX)
        target_link_libraries(algorithm_tests PRIVATE m)
    endif()
    add_test(NAME algorithm_tests COMMAND algorithm_tests)
endif()