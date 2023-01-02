cmake_minimum_required(VERSION 3.4)
project(tracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIR})

set(Boost_USE_MULTITHREADED ON)
add_definitions(-DBOOST_LOG_DYN_LINK)
find_package(Boost COMPONENTS system filesystem regex date_time log log_setup program_options unit_test_framework thread REQUIRED)
link_directories(${Boost_LIBRARY_DIR})

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

set(TRACKER_SOURCES
        include/gimbal.hpp
        include/gimbals/real.hpp
        include/gimbals/fake.hpp
        include/camera.hpp
        include/angles.hpp
        include/detector.hpp
        include/detectors/cascade.hpp
        include/controller.hpp
        include/strategies/global.hpp
        include/strategies/local.hpp
        include/constants.hpp

        src/main.cpp

        src/gimbal.cpp
        src/gimbals/real.cpp
        src/gimbals/fake.cpp
        src/camera.cpp
        src/angles.cpp
        src/detector.cpp
        src/detectors/cascade.cpp
        src/controller.cpp
        src/strategies/global.cpp
        src/strategies/local.cpp)

add_executable(${PROJECT_NAME} ${TRACKER_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE ../libs/gimbal-driver)
target_link_libraries(${PROJECT_NAME}
        gimbal-driver
        ${OpenCV_LIBS}
        ${DLIB_LIB}
        ${Boost_SYSTEM_LIBRARY}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_REGEX_LIBRARY}
        ${Boost_DATE_TIME_LIBRARY}
        ${Boost_LOG_LIBRARY}
        ${Boost_LOG_SETUP_LIBRARY}
        ${Boost_PROGRAM_OPTIONS_LIBRARY}
        ${Boost_THREAD_LIBRARY})

enable_testing()

add_custom_target(check ${CMAKE_COMMAND} -E env CTEST_OUTPUT_ON_FAILURE=1 BOOST_TEST_LOG_LEVEL=all
        ${CMAKE_CTEST_COMMAND} -C $<CONFIG> --verbose
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

add_executable(${PROJECT_NAME}-tester
        include/gimbal.hpp
        include/gimbals/real.hpp
        include/gimbals/fake.hpp
        include/camera.hpp
        include/angles.hpp
        include/detector.hpp
        include/detectors/cascade.hpp
        include/controller.hpp
        include/strategies/global.hpp
        include/strategies/local.hpp
        include/constants.hpp

        test/config.cpp
        test/StrategyTest.cpp

        src/gimbal.cpp
        src/gimbals/real.cpp
        src/gimbals/fake.cpp
        src/camera.cpp
        src/angles.cpp
        src/detector.cpp
        src/detectors/cascade.cpp
        src/controller.cpp
        src/strategies/global.cpp
        src/strategies/local.cpp)
target_include_directories(${PROJECT_NAME}-tester PRIVATE ../libs/gimbal-driver)
target_link_libraries(${PROJECT_NAME}-tester
        gimbal-driver
        ${OpenCV_LIBS}
        ${DLIB_LIB}
        ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_test(${PROJECT_NAME}-test ${PROJECT_NAME}-tester
        --report_level=detailed
        --log_level=all
        --color_output=yes)