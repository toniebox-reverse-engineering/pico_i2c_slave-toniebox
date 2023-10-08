add_executable(i2c-emu-mma8452q i2c-emu-mma8452q.c)

pico_enable_stdio_uart(i2c-emu-mma8452q 1)
pico_enable_stdio_usb(i2c-emu-mma8452q 1)


pico_add_extra_outputs(i2c-emu-mma8452q)

target_compile_options(i2c-emu-mma8452q PRIVATE -Wall)

target_link_libraries(i2c-emu-mma8452q i2c_slave pico_stdlib)
