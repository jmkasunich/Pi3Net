# define the Pi3Net library target
add_library(pi3net INTERFACE)

# assemble the PIO instructions into a header file
pico_generate_pio_header(pi3net ${CMAKE_CURRENT_LIST_DIR}/pi3net.pio)

# specify the library sources
target_sources(pi3net INTERFACE
    pi3net.c
)

# specify the include path
target_include_directories(pi3net INTERFACE ${CMAKE_CURRENT_LIST_DIR} )

# specify dependencies
target_link_libraries( pi3net INTERFACE
    hardware_pio
)
