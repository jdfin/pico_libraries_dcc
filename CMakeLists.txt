add_library(dcc INTERFACE) 

target_sources(dcc INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/dcc_adc.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/dcc_bit.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/dcc_bitstream.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/dcc_command.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/dcc_pkt.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/dcc_throttle.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/railcom.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/railcom_msg.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/railcom_spec.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/dcc_pkt2.cpp
)

target_include_directories(dcc INTERFACE 
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(dcc INTERFACE
    pico_stdlib
    hardware_adc
    hardware_clocks
    hardware_gpio
    hardware_irq
    hardware_pwm
    hardware_sync
    hardware_uart
    misc
)

add_subdirectory(dcc_spy)
add_subdirectory(dcc_cmd)
