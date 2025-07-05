cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_micro_ros_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

pico_sdk_init()
#เพิ่ม PicoLED เข้าไป และต้องเพิ่ม set(PICO_CXX_ENABLE_EXCEPTIONS 1) ข้างบน เพิ่ม LIB เข้าไปด้วย
include("PicoLED/PicoLed.cmake")
include("lib/LiquidCrystal_I2C/LiquidCrystal_I2C.cmake")

link_directories(libmicroros)

# Add executable
# add_executable(pico_micro_ros_example #น่าจะเป็น main function น่ะ
#     src/pico_micro_ros_example.cpp
#     pico_uart_transport.c
# )
# add_executable(led #น่าจะเป็น main function น่ะ
#     src/led.cpp
#     pico_uart_transport.c
# )
# add_executable(relay #น่าจะเป็น main function น่ะ
#     src/relay.cpp
#     pico_uart_transport.c
# )
# add_executable(segment7 #น่าจะเป็น main function น่ะ   
#     src/segment7.cpp
#     pico_uart_transport.c
# )
# add_executable(segment7_4 #น่าจะเป็น main function น่ะ   
#     src/segment7_4.cpp
#     pico_uart_transport.c
# )
# add_executable(lcd #น่าจะเป็น main function น่ะ
#     src/lcd.cpp
#     pico_uart_transport.c
# )
# add_executable(rtc #น่าจะเป็น main function น่ะ
#     src/rtc.cpp
#     pico_uart_transport.c
# )
# add_executable(water_sensor #น่าจะเป็น main function น่ะ
#     src/water_sensor.cpp
#     pico_uart_transport.c
# )
# add_executable(dht #น่าจะเป็น main function น่ะ
#     src/dht.cpp
#     pico_uart_transport.c
# )
# add_executable(rgb #น่าจะเป็น main function น่ะ
#     src/rgb.cpp
#     pico_uart_transport.c
# )
# add_executable(servo #น่าจะเป็น main function น่ะ
#     src/servo.cpp
#     pico_uart_transport.c
# )
# add_executable(buzzer #น่าจะเป็น main function น่ะ
#     src/buzzer.cpp
#     pico_uart_transport.c
# )
# add_executable(stepper #น่าจะเป็น main function น่ะ
#     src/stepper.cpp
#     pico_uart_transport.c
# )
# add_executable(ldr #น่าจะเป็น main function น่ะ
#     src/ldr.cpp
#     pico_uart_transport.c
# )
add_executable(potentiometer #น่าจะเป็น main function น่ะ
    src/potentiometer.cpp
    pico_uart_transport.c
)

# Add libraries
# target_link_libraries(pico_micro_ros_example
#     pico_stdlib
#     microros
#     PicoLed #เพิ่ม LIB เข้าไปด้วย
# )
# target_link_libraries(led
#     pico_stdlib
#     microros
# )
# target_link_libraries(relay
#     pico_stdlib
#     microros
# )
# target_link_libraries(segment7
#     pico_stdlib
#     microros
# )
# target_link_libraries(segment7_4
#     pico_stdlib
#     microros
# )
# target_link_libraries(lcd
#     pico_stdlib
#     microros
#     # hardware_uart
#     hardware_i2c
#     # LiquidCrystal_I2C
# )
# target_link_libraries(rtc
#     pico_stdlib
#     microros
# )
# target_link_libraries(water_sensor
#     pico_stdlib
#     microros
#     hardware_adc
# )
# target_link_libraries(dht
#     pico_stdlib
#     microros
#     # hardware_adc
# )
# target_link_libraries(rgb
#     pico_stdlib
#     microros
#     PicoLed
# )
# target_link_libraries(servo
#     pico_stdlib
#     microros
#     hardware_pwm
# )
# target_link_libraries(buzzer
#     pico_stdlib
#     microros
# )
# target_link_libraries(stepper
#     pico_stdlib
#     microros
# )
# target_link_libraries(ldr
#     pico_stdlib
#     microros
#     hardware_adc
# )
target_link_libraries(potentiometer
    pico_stdlib
    microros
    hardware_adc
)


# Add directories
# target_include_directories(pico_micro_ros_example PUBLIC
#     libmicroros/include
#     lib/*
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
# )
# target_include_directories(led PUBLIC
#     libmicroros/include
#     lib/*
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
# )
# target_include_directories(relay PUBLIC
#     libmicroros/include
#     lib/*
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
# )
# target_include_directories(segment7 PUBLIC
#     libmicroros/include
#     lib/*
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
# )
# target_include_directories(segment7_4 PUBLIC
#     libmicroros/include
#     lib/*
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
# )
# target_include_directories(lcd PUBLIC
#     libmicroros/include
#     lib/*
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
# )
# target_include_directories(rtc PUBLIC
#     libmicroros/include
#     lib/*
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
# )
# target_include_directories(water_sensor PUBLIC
#     libmicroros/include
#     lib/*
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
# )
# target_include_directories(dht PUBLIC
#     libmicroros/include
#     lib/*
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
# )
# target_include_directories(rgb PUBLIC
#     libmicroros/include
#     lib/*
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
# )
# target_include_directories(servo PUBLIC
#     libmicroros/include
#     lib/*
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
# )
# target_include_directories(buzzer PUBLIC
#     libmicroros/include
#     lib/*
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
# )
# target_include_directories(stepper PUBLIC
#     libmicroros/include
#     lib/*
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
# )
# target_include_directories(ldr PUBLIC
#     libmicroros/include
#     lib/*
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
# )
target_include_directories(potentiometer PUBLIC
    libmicroros/include
    lib/*
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)


SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
# pico_enable_stdio_usb(pico_micro_ros_example 1)
# pico_enable_stdio_uart(pico_micro_ros_example 0)
# pico_enable_stdio_usb(led 1)
# pico_enable_stdio_uart(led 0)
# pico_enable_stdio_usb(relay 1)
# pico_enable_stdio_uart(relay 0)
# pico_enable_stdio_usb(segment7 1)
# pico_enable_stdio_uart(segment7 0)
# pico_enable_stdio_usb(segment7_4 1)
# pico_enable_stdio_uart(segment7_4 0)
# pico_enable_stdio_usb(lcd 1)
# pico_enable_stdio_uart(lcd 0)
# pico_enable_stdio_usb(rtc 1)
# pico_enable_stdio_uart(rtc 0)
# pico_enable_stdio_usb(water_sensor 1)
# pico_enable_stdio_uart(water_sensor 0)
# pico_enable_stdio_usb(dht 1)
# pico_enable_stdio_uart(dht 0)
# pico_enable_stdio_usb(rgb 1)
# pico_enable_stdio_uart(rgb 0)
# pico_enable_stdio_usb(servo 1)
# pico_enable_stdio_uart(servo 0)
# pico_enable_stdio_usb(buzzer 1)
# pico_enable_stdio_uart(buzzer 0)
# pico_enable_stdio_usb(stepper 1)
# pico_enable_stdio_uart(stepper 0)
# pico_enable_stdio_usb(ldr 1)
# pico_enable_stdio_uart(ldr 0)
pico_enable_stdio_usb(potentiometer 1)
pico_enable_stdio_uart(potentiometer 0)


add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
# pico_add_extra_outputs(pico_micro_ros_example src/pico_micro_ros_example.cpp)
# pico_add_extra_outputs(led src/led.cpp)
# pico_add_extra_outputs(relay src/relay.cpp)
# pico_add_extra_outputs(segment7 src/segment7.cpp)
# pico_add_extra_outputs(segment7_4 src/segment7_4.cpp)
# pico_add_extra_outputs(lcd src/lcd.cpp)
# pico_add_extra_outputs(rtc src/rtc.cpp)
# pico_add_extra_outputs(water_sensor src/water_sensor.cpp) ดู src/ ด้วยน่ะ
# pico_add_extra_outputs(dht src/dht.cpp)
# pico_add_extra_outputs(rgb src/rgb.cpp)
# pico_add_extra_outputs(servo src/servo.cpp)
# pico_add_extra_outputs(buzzer src/buzzer.cpp)
# pico_add_extra_outputs(stepper src/stepper.cpp)
# pico_add_extra_outputs(ldr src/ldr.cpp)
pico_add_extra_outputs(potentiometer src/potentiometer.cpp)