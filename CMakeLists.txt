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

#################################
# Gimbal drivers
#################################
set(PATH_TO_DRIVER_SOURCES lib/gimbal-driver)

set(DRIVER_SOURCES
        ${PATH_TO_DRIVER_SOURCES}/linuxDriver/driver_Linux.c
        ${PATH_TO_DRIVER_SOURCES}/linuxDriver/driver_Linux.h
        ${PATH_TO_DRIVER_SOURCES}/adjvar/adjvar.c
        ${PATH_TO_DRIVER_SOURCES}/adjvar/adjvar.h
        ${PATH_TO_DRIVER_SOURCES}/calib/calib.c
        ${PATH_TO_DRIVER_SOURCES}/calib/calib.h
        ${PATH_TO_DRIVER_SOURCES}/core/core.c
        ${PATH_TO_DRIVER_SOURCES}/core/core.h
        ${PATH_TO_DRIVER_SOURCES}/eeprom/eeprom.c
        ${PATH_TO_DRIVER_SOURCES}/eeprom/eeprom.h
        ${PATH_TO_DRIVER_SOURCES}/gimbalControl/gimbalControl.c
        ${PATH_TO_DRIVER_SOURCES}/gimbalControl/gimbalControl.h
        ${PATH_TO_DRIVER_SOURCES}/imu/imu.c
        ${PATH_TO_DRIVER_SOURCES}/imu/imu.h
        ${PATH_TO_DRIVER_SOURCES}/profiles/profiles.c
        ${PATH_TO_DRIVER_SOURCES}/profiles/profiles.h
        ${PATH_TO_DRIVER_SOURCES}/realtime/realtime.c
        ${PATH_TO_DRIVER_SOURCES}/realtime/realtime.h
        ${PATH_TO_DRIVER_SOURCES}/service/service.c
        ${PATH_TO_DRIVER_SOURCES}/service/service.h
        )
#################################
#################################


include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

set(TRACKER_SOURCES
    # Sources and headers go here.
    include/gimbal.hpp
    include/camera.hpp
    include/angles.hpp
    include/cascade-face-detector.hpp
    include/constants.hpp
    include/helpers.hpp

    src/main.cpp

    src/gimbal.cpp
    src/camera.cpp
    src/cascade-face-detector.cpp
    src/helpers.cpp
    )

add_executable(${PROJECT_NAME} ${TRACKER_SOURCES} ${DRIVER_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE ${PATH_TO_DRIVER_SOURCES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${DLIB_LIB} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_REGEX_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_LOG_LIBRARY} ${Boost_LOG_SETUP_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_THREAD_LIBRARY})

#set_property(TARGET tracker PROPERTY CXX_STANDARD 17)


# TODO: add testing conditional compilation
if (NO)
    add_subdirectory(tests)
endif()