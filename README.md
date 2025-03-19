This repository is a small demo showing how to create DVI output on the
RP2350 microcontroller using HSTX without using any CPU at all, relying
entirely on the built-in DMA engine.

This relies on a major feature of the HSTX engine, the Command Expander.

When porting RP2040-based DVI generators over they often retain the
per-scanline looping nature either of a vsync pre-built buffer of only a
couple dozen bytes, or a pair of buffers ping-ponged to send the next
set of active pixels.

If instead we space out the lines slightly we can inject the HSTX
Command Expander opcodes directly into the video buffer, allowing the
whole video buffer to be fed unmodified to the HSTX infrastructure
autonomously.

This in theory allows for future improvements including HDMI Audio
using a fixed 'ring buffer' like area in the pre-compiled buffer as
well, though that would require switching the vsync period to AFTER
the active pixels and using an IRQ to trigger the update period as
we'd need to complete the full HDMI data island update before any
of it gets transmitted.
