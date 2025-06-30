#include <stdio.h>
#include "pico/stdlib.h"
#include "pi3net.h"

const p3n_hwdef_t p3n_bus1 = { 1, 1, 1, 16, 16, 17 };
const p3n_hwdef_t p3n_bus2 = { 1, 1, 1, 16, 16, 28 };
const p3n_hwdef_t p3n_tx_up = { 1, 0, 0, 4, 0, 0 };
const p3n_hwdef_t p3n_rx_up = { 0, 1, 0, 0, 5, 0 };
const p3n_hwdef_t p3n_tx_dn = { 1, 0, 0, 2, 0, 0 };
const p3n_hwdef_t p3n_rx_dn = { 0, 1, 0, 0, 2, 0 };

void p3n_test(void)
{
    printf("Hello from pi3net\n");
}
