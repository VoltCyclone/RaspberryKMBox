#include "peri_clock.h"
#include "hardware/clocks.h"
#include <stdio.h>

void peri_clock_configure_stable(void) {
    uint32_t sys_freq  = clock_get_hz(clk_sys);
    uint32_t peri_freq = clock_get_hz(clk_peri);

    printf("[CLK] clk_sys=%lu Hz, clk_peri=%lu Hz\n", sys_freq, peri_freq);

    if (sys_freq > 133000000) {
        printf("[CLK] Overclock detected — switching clk_peri to 48 MHz USB PLL\n");
        clock_configure(clk_peri,
                        0,
                        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                        48000000,
                        48000000);
        peri_freq = clock_get_hz(clk_peri);
    }

    printf("[CLK] clk_peri now %lu Hz (%.2f MHz)\n",
           peri_freq, peri_freq / 1000000.0f);
}
