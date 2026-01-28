
add_compile_options(-Wall -Wextra -Werror)

# dcc_cmd

add_executable(dcc_cmd
    dcc_cmd.cpp
)

#pico_set_printf_implementation(dcc_cmd compiler)

pico_enable_stdio_uart(dcc_cmd 0)
pico_enable_stdio_usb(dcc_cmd 1)

target_include_directories(dcc_cmd PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(dcc_cmd PRIVATE
    pico_stdlib
    pico_stdio_usb
    misc
    dcc
)

pico_add_extra_outputs(dcc_cmd)
