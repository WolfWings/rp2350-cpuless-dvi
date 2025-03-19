add_executable(dma_720p
        dma_720p.c
        )

target_include_directories(dma_720p PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/images
        )

# pull in common dependencies
target_link_libraries(dma_720p
        pico_stdlib
        pico_multicore
        pico_rand
        hardware_dma
        pico_sync
        )

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(dma_720p)
pico_enable_stdio_usb(dma_720p 1)
pico_enable_stdio_uart(dma_720p 0)
