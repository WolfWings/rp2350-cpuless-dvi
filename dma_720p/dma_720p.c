// Copyright (c) 2024 Raspberry Pi (Trading) Ltd.

// Generate DVI output using the command expander and TMDS encoder in HSTX.

// This example requires an external digital video connector connected to
// GPIOs 12 through 19 (the HSTX-capable GPIOs) with appropriate
// current-limiting resistors, e.g. 270 ohms. The pinout used in this example
// matches the Pico DVI Sock board, which can be soldered onto a Pico 2:
// https://github.com/Wren6991/Pico-DVI-Sock

#include <stdio.h>
#include <string.h>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/structs/hstx_ctrl.h"
#include "hardware/structs/hstx_fifo.h"
#include "hardware/structs/sio.h"
#include "pico/multicore.h"
#include "pico/sem.h"
#include "pico/stdlib.h"
#include "pico/rand.h"

#include "images/wallpaper.h"

// ----------------------------------------------------------------------------
// DVI constants

#define TMDS_CTRL_00 0b1101010100u
#define TMDS_CTRL_01 0b0010101011u
#define TMDS_CTRL_10 0b0101010100u
#define TMDS_CTRL_11 0b1010101011u

#define SYNC_V0_H0 (TMDS_CTRL_00 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V0_H1 (TMDS_CTRL_01 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H0 (TMDS_CTRL_10 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H1 (TMDS_CTRL_11 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))

#define MODE_H_FRONT_PORCH   24
#define MODE_H_SYNC_WIDTH    32
#define MODE_H_BACK_PORCH    24
#define MODE_H_ACTIVE_PIXELS 1280

#define MODE_V_FRONT_PORCH   5
#define MODE_V_SYNC_WIDTH    6
#define MODE_V_BACK_PORCH    5
#define MODE_V_ACTIVE_LINES  720

#if ( ( MODE_H_ACTIVE_PIXELS % 4 ) != 0 )
#error MODE_H_ACTIVE_PIXELS MUST be divisible by 4
#endif

#define HSTX_CMD_RAW         (0x0u << 12)
#define HSTX_CMD_RAW_REPEAT  (0x1u << 12)
#define HSTX_CMD_TMDS        (0x2u << 12)
#define HSTX_CMD_TMDS_REPEAT (0x3u << 12)
#define HSTX_CMD_NOP         (0xfu << 12)

// In this part we 'bake in' the hsync band commands directly into the framebuffer.
// We end up with an 'oddball' stride but that's already true at all 16:9 resolutions
// so it's no net loss.
//
// Note the 7 instead of 6 because we need to bake in the 'active pixels' command!
//
// At 1280 2bpp this ends up 320 bytes for the stride + 28 bytes commands, so ~10%
// increase in framebuffer size.
#define FRAMEBUF_STRIDE_DWORDS ( ( ( MODE_H_ACTIVE_PIXELS / 4 ) / sizeof( uint32_t ) ) + 7 )
#define FRAMEBUF_STRIDE ( FRAMEBUF_STRIDE_DWORDS * sizeof( uint32_t ) )

// Theory here is we 'unroll' the vsync band fully. This also allows us to merge the
// front and back porch parts in each section, so instead of...
//
// ...
// FRONT_PORCH
// SYNC_WIDTH
// BACK_PORCH + ACTIVE_PIXELS
// FRONT_PORCH
// SYNC_WIDTH
// BACK_PORCH + ACTIVE_PIXELS
// ...
//
// ...we can merge the adjacent items and streamline it to...
//
// ...
// FRONT_PORCH
// SYNC_WIDTH
// BACK_PORCH + ACTIVE_PIXELS + FRONT_PORCH
// SYNC_WIDTH
// BACK_PORCH + ACTIVE_PIXELS
// ...
//
// ...so we need 4 per region plus 2 additional ones for the final leftover bit.

#define FRAMEBUF_VBLANK_FRONT ( sizeof( uint32_t ) * ( ( 4 * MODE_V_FRONT_PORCH ) + 2 ) )
#define FRAMEBUF_VBLANK_SYNC ( sizeof( uint32_t ) * ( ( 4 * MODE_V_SYNC_WIDTH ) + 2 ) )
#define FRAMEBUF_VBLANK_BACK ( sizeof( uint32_t ) * ( ( 4 * MODE_V_BACK_PORCH ) + 2 ) )
#define FRAMEBUF_PREQUEL ( FRAMEBUF_VBLANK_FRONT + FRAMEBUF_VBLANK_SYNC + FRAMEBUF_VBLANK_BACK )

uint8_t __attribute__((aligned(4))) framebuf_raw[ FRAMEBUF_PREQUEL + ( FRAMEBUF_STRIDE * MODE_V_ACTIVE_LINES ) ];
uint32_t *framebuf_cmds = (uint32_t *)framebuf_raw;
uint8_t *framebuf = &framebuf_raw[ FRAMEBUF_PREQUEL + ( sizeof( uint32_t ) * 7 ) ];

// ----------------------------------------------------------------------------
// DMA logic

#define DMACH_SCREEN 0
#define DMACH_LOOPER 1

// ----------------------------------------------------------------------------
// Main program

int main(void) {
	gpio_init( 23 );
	gpio_set_dir( 23, GPIO_OUT );
	gpio_put( 23, true );

	stdio_init_all(); // To allow BOOTSEL functions without hitting the button

	// Build the pre-compiled framebuffer
	uint fill = 0;

	framebuf_cmds[ fill++ ] = HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH;
	framebuf_cmds[ fill++ ] = SYNC_V1_H1;
	for ( uint y = 0; y < MODE_V_FRONT_PORCH; y++ ) {
		framebuf_cmds[ fill++ ] = HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH;
		framebuf_cmds[ fill++ ] = SYNC_V1_H0;
		framebuf_cmds[ fill++ ] = HSTX_CMD_RAW_REPEAT | ( MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS + MODE_H_FRONT_PORCH );
		framebuf_cmds[ fill++ ] = SYNC_V1_H1;
	}
	framebuf_cmds[ fill - 2 ] = HSTX_CMD_RAW_REPEAT | ( MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS );

	framebuf_cmds[ fill++ ] = HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH;
	framebuf_cmds[ fill++ ] = SYNC_V0_H1;
	for ( uint y = 0; y < MODE_V_SYNC_WIDTH; y++ ) {
		framebuf_cmds[ fill++ ] = HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH;
		framebuf_cmds[ fill++ ] = SYNC_V0_H0;
		framebuf_cmds[ fill++ ] = HSTX_CMD_RAW_REPEAT | ( MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS + MODE_H_FRONT_PORCH );
		framebuf_cmds[ fill++ ] = SYNC_V0_H1;
	}
	framebuf_cmds[ fill - 2 ] = HSTX_CMD_RAW_REPEAT | ( MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS );

	framebuf_cmds[ fill++ ] = HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH;
	framebuf_cmds[ fill++ ] = SYNC_V1_H1;
	for ( uint y = 0; y < MODE_V_BACK_PORCH; y++ ) {
		framebuf_cmds[ fill++ ] = HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH;
		framebuf_cmds[ fill++ ] = SYNC_V1_H0;
		framebuf_cmds[ fill++ ] = HSTX_CMD_RAW_REPEAT | ( MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS + MODE_H_FRONT_PORCH );
		framebuf_cmds[ fill++ ] = SYNC_V1_H1;
	}
	framebuf_cmds[ fill - 2 ] = HSTX_CMD_RAW_REPEAT | ( MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS );

	for ( uint y = 0; y < MODE_V_ACTIVE_LINES; y++ ) {
		framebuf_cmds[ fill++ ] = HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH;
		framebuf_cmds[ fill++ ] = SYNC_V1_H1;
		framebuf_cmds[ fill++ ] = HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH;
		framebuf_cmds[ fill++ ] = SYNC_V1_H0;
		framebuf_cmds[ fill++ ] = HSTX_CMD_RAW_REPEAT | MODE_H_BACK_PORCH;
		framebuf_cmds[ fill++ ] = SYNC_V1_H1;
		framebuf_cmds[ fill++ ] = HSTX_CMD_TMDS | MODE_H_ACTIVE_PIXELS;
		fill += ( ( MODE_H_ACTIVE_PIXELS / 4 ) / sizeof( uint32_t ) );
		memcpy( &framebuf[ y * FRAMEBUF_STRIDE ], &wallpaper[ y * MODE_H_ACTIVE_PIXELS / 4 ], MODE_H_ACTIVE_PIXELS / 4 );
	}

	// Serial output config: clock period of 5 cycles, pop from command
	// expander every 5 cycles, shift the output shiftreg by 2 every cycle.
	hstx_ctrl_hw->csr = 0;
	hstx_ctrl_hw->csr =
		HSTX_CTRL_CSR_EXPAND_EN_BITS |
		5u << HSTX_CTRL_CSR_CLKDIV_LSB |
		5u << HSTX_CTRL_CSR_N_SHIFTS_LSB |
		2u << HSTX_CTRL_CSR_SHIFT_LSB |
		HSTX_CTRL_CSR_EN_BITS;

	// Note we are leaving the HSTX clock at the SDK default of 150 MHz; since
	// we shift out two bits per HSTX clock cycle this gives us 300 Mbps. I've
	// manually tuned the 1280x720p30 video timing to align as closely to this
	// as possible, ending up at 30000000/(1360*736)=29.97122 refresh rate.

	// HSTX outputs 0 through 7 appear on GPIO 12 through 19.
	// Pinout on Pico DVI sock:
	//
	//   GP12 D0+  GP13 D0-
	//   GP14 CK+  GP15 CK-
	//   GP16 D2+  GP17 D2-
	//   GP18 D1+  GP19 D1-

	// Assign clock pair to two neighbouring pins:
	hstx_ctrl_hw->bit[2] = HSTX_CTRL_BIT0_CLK_BITS;
	hstx_ctrl_hw->bit[3] = HSTX_CTRL_BIT0_CLK_BITS | HSTX_CTRL_BIT0_INV_BITS;
	for (uint lane = 0; lane < 3; ++lane) {
		// For each TMDS lane, assign it to the correct GPIO pair based on the
		// desired pinout:
		static const int lane_to_output_bit[3] = {0, 6, 4};
		int bit = lane_to_output_bit[lane];
		// Output even bits during first half of each HSTX cycle, and odd bits
		// during second half. The shifter advances by two bits each cycle.
		uint32_t lane_data_sel_bits =
			(lane * 10    ) << HSTX_CTRL_BIT0_SEL_P_LSB |
			(lane * 10 + 1) << HSTX_CTRL_BIT0_SEL_N_LSB;
		// The two halves of each pair get identical data, but one pin is inverted.
		hstx_ctrl_hw->bit[bit    ] = lane_data_sel_bits;
		hstx_ctrl_hw->bit[bit + 1] = lane_data_sel_bits | HSTX_CTRL_BIT0_INV_BITS;
	}

	// Palette: #000000 #804000 #008080 #80c080
	hstx_ctrl_hw->expand_tmds =
		1  << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB |
		26 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB   |
		0  << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB |
		26 << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB   |
		0  << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB |
		25 << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;

	hstx_ctrl_hw->expand_shift =
		16 << HSTX_CTRL_EXPAND_SHIFT_ENC_N_SHIFTS_LSB |
		2  << HSTX_CTRL_EXPAND_SHIFT_ENC_SHIFT_LSB    |
		1  << HSTX_CTRL_EXPAND_SHIFT_RAW_N_SHIFTS_LSB |
		0  << HSTX_CTRL_EXPAND_SHIFT_RAW_SHIFT_LSB;

	for (int i = 12; i <= 19; ++i) {
		gpio_set_function(i, 0); // HSTX
	}

	// _SCREEN channel does 99% of the work and handles the entire screen
	// autonomously with the commands pre-baked into the memory chain.
	//
	// _LOOPER exists only to restart _SCREEN without relying on any IRQ
	// to trigger in a timely fashion, and in fact we use zero IRQs now!
	dma_channel_config c;

	c = dma_channel_get_default_config( DMACH_SCREEN );
	channel_config_set_chain_to( &c, DMACH_LOOPER );
	channel_config_set_dreq( &c, DREQ_HSTX );
	dma_channel_configure(
		DMACH_SCREEN,
		&c,
		&hstx_fifo_hw->fifo,
		framebuf_raw,
		sizeof( framebuf_raw ) / sizeof( uint32_t ),
		false
	);

	c = dma_channel_get_default_config( DMACH_LOOPER );
	channel_config_set_chain_to( &c, DMACH_SCREEN );
	channel_config_set_dreq( &c, DREQ_HSTX );
	channel_config_set_read_increment( &c, false );
	channel_config_set_write_increment( &c, false );
	dma_channel_configure(
		DMACH_LOOPER,
		&c,
		&dma_hw->ch[ DMACH_SCREEN ].read_addr,
		&framebuf_cmds,
		1,
		false
	);

	bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

	dma_channel_start( DMACH_SCREEN );

	gpio_put( 23, false );

	for (;;) {
		// Can't use __wfi as we have no interrupts in use!
		sleep_ms( 100 );
	}
}
