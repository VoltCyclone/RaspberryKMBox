/**
 * Peripheral Clock Stabilization
 *
 * When the RP2350 is overclocked (>133 MHz), clk_peri inherits the high
 * system clock and produces fractional UART baud divisors.  This utility
 * switches clk_peri to the 48 MHz USB PLL so that 3 Mbaud divides
 * exactly (48 M / 3 M = 16, 0 ppm error).
 *
 * MUST be called AFTER set_sys_clock_khz() and BEFORE uart_init().
 */

#ifndef PERI_CLOCK_H
#define PERI_CLOCK_H

/**
 * Configure clk_peri for stable UART operation.
 *
 * If clk_sys > 133 MHz the peripheral clock is switched to the 48 MHz
 * USB PLL.  Otherwise the current source is left untouched.
 */
void peri_clock_configure_stable(void);

#endif // PERI_CLOCK_H
