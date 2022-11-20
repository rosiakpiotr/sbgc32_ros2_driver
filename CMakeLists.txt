cmake_minimum_required(VERSION 3.1)
project(gimbal-viewfinder)

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
    # Sources and headers go here.
    include/gimbal.hpp
    src/main.cpp
    )

add_executable(gimbal-viewfinder ${TRACKER_SOURCES})
target_link_libraries(gimbal-viewfinder ${OpenCV_LIBS} ${DLIB_LIB} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_REGEX_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_LOG_LIBRARY} ${Boost_LOG_SETUP_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_THREAD_LIBRARY})

#set_property(TARGET tracker PROPERTY CXX_STANDARD 17)


# TODO: add testing conditional compilation
if (NO)
    add_subdirectory(tests)
endif()