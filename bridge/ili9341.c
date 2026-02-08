/*
 * Copyright (C) 2022 Jan Hamal Dvořák <mordae@anilinux.org>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "tft.h"
#include <stdio.h>

#if TFT_DRIVER == TFT_DRIVER_ILI9341

#define WIDTH (TFT_WIDTH * TFT_SCALE)
#define HEIGHT (TFT_HEIGHT * TFT_SCALE)

void tft_preflight(void)
{
	/* 8.2.2 SWRESET: Software Reset */
	tft_control(0x01, NULL, 0);
	sleep_ms(120);

	/* 8.2.12 SLPOUT: Sleep Out */
	tft_control(0x11, NULL, 0);
	sleep_ms(120);

	/* 8.2.17 GAMSET: Gamma Set */
	uint8_t gamset[] = { 1 };
	tft_control(0x26, gamset, sizeof gamset);

	/* 8.3.24 PGAMCTRL: Positive Gamma Correction (from Linux) */
	uint8_t pgamctrl[] = { 0x0f, 0x31, 0x2b, 0x0c, 0x0e, 0x08, 0x4e, 0xf1,
			       0x37, 0x07, 0x10, 0x03, 0x0e, 0x09, 0x00 };
	tft_control(0xe0, pgamctrl, sizeof pgamctrl);

	/* 8.3.25 NGAMCTRL: Negative Gamma Correction (from Linux) */
	uint8_t ngamctrl[] = { 0x00, 0x0e, 0x14, 0x03, 0x11, 0x07, 0x31, 0xc1,
			       0x48, 0x08, 0x0f, 0x0c, 0x31, 0x36, 0x0f };
	tft_control(0xe1, ngamctrl, sizeof ngamctrl);

	/* 8.4.7 Enable_3G: Enable 3G */
	uint8_t enable3g[] = { 0 };
	tft_control(0xf2, enable3g, sizeof enable3g);

	/* 8.2.6 MADCTL: Memory Access Data Control */
	uint8_t madctl = (0 << 5) | (TFT_FLIP_X << 6) | (TFT_FLIP_Y << 7) |
			 (1 << 3) /* BGR swap */ | (TFT_FLIP_Y << 4) | (0 << 2);
	tft_control(0x36, &madctl, sizeof madctl);

	/* 8.2.33 COLMOD: Interface Pixel Format */
	uint8_t colmod = 0x6d; /* 16 bits in 565 format */
	tft_control(0x3a, &colmod, sizeof colmod);

	/* 8.2.19 DISPON: Display On */
	tft_control(0x29, NULL, 0);
	sleep_ms(120);

	/* 8.3.8 ETMOD: Entry Mode Set */
	uint8_t etmod = 0x07;
	tft_control(0xb7, &etmod, sizeof etmod);

	/* 8.2.20 CASET: Column Address Set */
	uint8_t caset[] = { 0, 0, TFT_RAW_WIDTH >> 8, TFT_RAW_WIDTH & 0xff };
	tft_control(0x2a, caset, sizeof caset);

	/* 8.2.21 PASET: Page Address Set */
	uint8_t paset[] = { 0, 0, TFT_RAW_HEIGHT >> 8, TFT_RAW_HEIGHT & 0xff };
	tft_control(0x2b, paset, sizeof paset);

	/* 8.3.2 FRMCTR1: Frame Rate Control */
	uint8_t frmctr1[] = { 0, 24 }; // 79 fps
	tft_control(0xb1, frmctr1, sizeof frmctr1);
}

void tft_begin_sync(void)
{
#if defined(TFT_MISO_PIN) && TFT_VSYNC
	unsigned scanline;

	for (int i = 0; i < TFT_HEIGHT * 10; i++) {
		uint8_t scl[3];
		tft_read(0x45, scl, sizeof scl, 3);
		scanline = ((unsigned)scl[1] << 2) | (scl[2] >> 6);

		if (scanline >= 4 && scanline < 16)
			break;
	}
#endif

	/* 8.2.22 RAMWR: Memory Write */
	tft_control(0x2c, NULL, 0);
}

#endif
