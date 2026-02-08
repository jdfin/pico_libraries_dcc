
add_compile_options(-Wall -Wextra -Werror)

# dcc_spy

add_executable(dcc_spy
    dcc_spy.cpp
)

pico_enable_stdio_uart(dcc_spy 0)
pico_enable_stdio_usb(dcc_spy 1)

target_include_directories(dcc_spy PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(dcc_spy PRIVATE
    pico_stdlib
    pico_stdio_usb
    #pico_multicore (display)
    misc
    #fonts          (display)
    #ws24           (display)
    pio_edges
    dcc
)

pico_add_extra_outputs(dcc_spy)
