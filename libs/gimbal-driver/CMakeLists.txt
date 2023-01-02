cmake_minimum_required(VERSION 3.4)
project(gimbal-driver)

set(DRIVER_SOURCES
        linuxDriver/driver_Linux.c
        linuxDriver/driver_Linux.h
        adjvar/adjvar.c
        adjvar/adjvar.h
        calib/calib.c
        calib/calib.h
        core/core.c
        core/core.h
        eeprom/eeprom.c
        eeprom/eeprom.h
        gimbalControl/gimbalControl.c
        gimbalControl/gimbalControl.h
        imu/imu.c
        imu/imu.h
        profiles/profiles.c
        profiles/profiles.h
        realtime/realtime.c
        realtime/realtime.h
        service/service.c
        service/service.h
        )

add_library(gimbal-driver ${DRIVER_SOURCES})