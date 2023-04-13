cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(drone_kal C CXX ASM)
set(CMAKE_C_STNDARD 11)
set(CMAKE_CXX_STANDARD 17)

# set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

pico_sdk_init()
include_directories(/Users/kokoro/Documents/drone_project/pico/eigen)

add_executable(drone_kal
  test.cpp
  vl53l1_platform.c
  VL53L1X_api.c
  VL53L1X_calibration.c
)

target_link_libraries(drone_kal
  hardware_i2c
  hardware_spi 
  hardware_pwm 
  hardware_uart 
  pico_stdlib
  pico_multicore)

pico_enable_stdio_usb(drone_kal 1)
pico_enable_stdio_uart(drone_kal 0)

pico_add_extra_outputs(drone_kal)