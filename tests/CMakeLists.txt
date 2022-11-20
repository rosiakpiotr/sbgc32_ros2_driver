cmake_minimum_required(VERSION 3.1)
project(gimbal-viewfinder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(Boost_USE_MULTITHREADED ON)
add_definitions(-DBOOST_LOG_DYN_LINK)
find_package(Boost COMPONENTS system filesystem regex date_time log log_setup program_options unit_test_framework thread REQUIRED)
link_directories(${Boost_LIBRARY_DIR})

enable_testing()
include_directories(
        include
        ${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIR})
add_custom_target(check ${CMAKE_COMMAND} -E env CTEST_OUTPUT_ON_FAILURE=1 BOOST_TEST_LOG_LEVEL=all
        ${CMAKE_CTEST_COMMAND} -C $<CONFIG> --verbose
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

add_executable(tester
        config.cpp
)
target_link_libraries(tester
        ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_test(test tester
        --report_level=detailed
        --log_level=all
        --color_output=yes)