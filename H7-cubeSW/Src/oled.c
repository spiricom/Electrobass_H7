/*
 ____    ____    ____       _     __       __     ____
/\  _`\ /\  _`\ /\  _`\   /' \  /'__`\   /'__`\  /'___\
\ \,\L\_\ \,\L\_\ \ \/\ \/\_, \/\_\L\ \ /\ \/\ \/\ \__/
 \/_\__ \\/_\__ \\ \ \Small Scale_/_\_<_\ \ \ \ \ \  _``\
   /\ \L\ \/\ \L\ \ \ \_\ \ \ \ \/\ \L\ \\ \ \_\ \ \ \L\ \
   \ `\____\ `\____\ \____/  \ \_\ \____/ \ \____/\ \____/
    \/_____/\/_____/\/___/    \/_/\/___/   \/___/  \/___/

I'll bet some SSD1306 OLED DRIVER ROUTINES would only ENHANCE my EMBEDDED DISPLAY TECHNOLOGY

[!] This is my module; there are many others like it, but this one is mine.
The module I have is a Chinese import stenciled "Long Qiu" (and not much else).
Everything happens over a 7-pin SIP pin header labled IF2.
Your mileage may vary with other modules. I don't have a parallel one, but rewriting oled_write to use
a parallel bus is a trivial exercise left to the reader.

[!] SSD1306 Display Memory 101
Buffer is 8kbit (1kB) addressed with two auto-incrementable pointers as 8 pages * (128 columns * 8 bits)

         Display Memory        Column
     Cols 0    ->   127
		+---------------+       +---+
  PAGE0 | XCCCC...CCCCC |       | 0 |
R PAGE1 | CCCCC...CCCCC |   C =>|...|  * 127 => PAGE
o  ...  | ............. |       | 7 |
w PAGE6 | CCCCC...CCCCC |       +---+
s PAGE7 | CCCCC...CCCCC |         ^--- 8 1-bit pixels per column byte, arranged vertically regardless of addressing mode
        +---------------+
		 X => Pointer at PAGE 0, COL 0

[!] Fuck yeah tile graphics
For the purposes of oled_move(), oled_home(), oled_puts(), oled_putc() and most everything else,
the display is a 16x8 array of 8x8 character cells. Functions expect horizontal addressing mode, other modes
will make them act wanky. Pixels aren't really addressable; the tiny1634 doesn't have enough RAM for a local framebuffer,
and the SPI link to the display is one way so there's no read-modify-write using the display frame buffer.
Given the memory layout I'm reasonably certain the SSD1306 was intended to be driven, perhaps primarily, as a tile graphic display.

[!] Blast from the past
Character generator table is stored in progmem as "font", and contains 128 cells in
PETSCII layout (i.e. you can generate them from C64 font files). Included is the canonical 8x8 C64 font,
with a couple minor changes to make box-drawn digits look better.
Characters are rotated 90 degrees clockwise (so we don't have to waste AVR cycles flipping tiles).
The pointy part of an 'A' should point that way -> if you're doing it right.
It's irritating to go alone, take this: http://www.min.at/prinz/o/software/pixelfont/

[!] Fast as a (LOGO) turtle
Despite software bit-bang and zero optimization it's more than usably fast on a 9MHz Tiny1634; I'm guessing a full repaint at over 15Hz.
Much much faster if moving the pointers and updating incrementally (like a terminal!). Add graphic tiles and make the next-gen POPStation.

[!] Cute lil feller
Basic character display functions are less than 3k compiled, 1k of which is the 128-cell chargen data.
Chargen can be trimmed to 288ish bytes if only the bare minimum alphanumerics are required.

[!] Zero to Pixels
> Set pin/port defs in ssd1306-config.h
> oled_init()
> oled_clear() // buffer is full of entropy on boot
> oled_home()  // pointers should end up here, but let's make sure
> oled_puts("POOP") // POOP

[!] We built this city in AVR studio 6 using avr-gcc
Very little AVR-specific operations outside of the headers and PROGMEM macros, porting to PIC/ARM/HP9000/M88k/etc should be trivial.

[!] See Also
Ladyada's framebuffer-based arduinolib (c++) SSD1306 implementation: https://github.com/adafruit/Adafruit_SSD1306
RTFM: https://www.adafruit.com/datasheets/SSD1306.pdf

[!] Legal Fictions
Original work released under terms of the BSD license.
Included chargen/font data generated from c64_lower.64c font and used without permission.
(Chargen data is assumed to be of negligible economic value and public domain/abandoned. Lawyer at me if you got beef.)

-------------------------------------------------------------------------------
Inspired by tile-based arcade gfx, hacked together by kmm/smallscaleresearch
April 2013
Contact: kmm CHR(0x40) rmlabs.net
-------------------------------------------------------------------------------
*/

#include "oled-config.h"
#include "oled.h"
#include "c64_lower.h"
#include "ssd1306.h"


void oled_put_tile(uint8_t *tile, uint8_t limit) {
	for(uint16_t i = 0; i < limit; i++) {
		ssd1306_write(*tile++,1);
	}
}

void oled_putc_raw(char c) {
	for(uint16_t i = c << 3; i < (c << 3) + 8; i++) {
		ssd1306_write(&font[i], 1);
	}

}

void oled_putc(char c) {
	// remap from petscii to ascii, shifts drawing characters into the lower 32 ascii cells
	if(c > 'A' && c < 'Z') { }               // upper-case ascii range
	else if(c > 'a' && c < 'z') { c -= 96; } // lower-case ascii range
	else if(c > 31 && c < 64) { }            // numbers and symbols
	else if(c < 32) { c += 96; }             // low ascii
	oled_putc_raw(c);
}

void oled_putc_2X(char c) {
	// remap from petscii to ascii, shifts drawing characters into the lower 32 ascii cells
	if(c > 'A' && c < 'Z') { }               // upper-case ascii range
	else if(c > 'a' && c < 'z') { c -= 96; } // lower-case ascii range
	else if(c > 31 && c < 64) { }            // numbers and symbols
	else if(c < 32) { c += 96; }             // low ascii
	for(uint16_t i = c << 3; i < (c << 3) + 8; i++) {
		ssd1306_write_2X(&font[i], 1);
	}
}

void oled_fill(uint8_t row, uint8_t col, uint8_t count, uint8_t max, uint32_t pattern, int8_t pshift) {
	ssd1306_move(row, col);
	uint8_t pstate = 0;
	for(uint8_t myMax = max; myMax > 0; myMax--) {
		if(count > myMax) {
			if(pshift < 0) {
				ssd1306_write((pattern >> (pstate++ << 3)) & 0xFF, 1);
			}
			else
			{
				ssd1306_write((pattern >> (pstate++ + pshift)) & 0xFF, 1);
			}
			pstate = (pstate > 3) ? 0 : pstate;
		}
		else {
			ssd1306_write(0x00, 1);
		}
	}

}

// Draw a tile at an arbitrary pixel location (top, left) using an 8 byte tile buffer referenced by *tile.
// Slower than oled_putc(), potentially substantially so; only use for things that need
// finer grained positioning than is possible with tile cells, like sprites.
// Clips right and bottom edges properly; untested and not expected to work with negative positions.
void oled_putxy(uint8_t left_pxl, uint8_t top_pxl, uint8_t *tile) {
	uint8_t tbuf[8], obuf[8];
	uint8_t top_cell = top_pxl >> 3;
	uint8_t left_cell = left_pxl >> 3;
	int8_t voff = top_pxl - ((top_cell << 3) - 1);
	int8_t hoff = left_pxl - ((left_cell << 3) - 1);

	if(voff == 0 && hoff == 0) {
		ssd1306_move(top_cell, left_pxl >> 3);
		oled_put_tile(tile, 8);
		return;
	}
	else {
		for(uint8_t tcol = 0; tcol < 8; tcol++) { // tile column
			tbuf[tcol] = (tile[tcol]) << ((uint8_t)voff); // shift left (down) by voff
			obuf[tcol] = (tile[tcol]) >> (8 - (uint8_t)voff); //shift right (up) by voff
		}

		ssd1306_move_raw(top_cell, left_pxl); // move_raw(row[0:7], column[0:127]) rows and pixels for extra confusion
		oled_put_tile(&tbuf, (left_pxl > (SS1306_OLED_GEOM_W - 8)) ? 8 - hoff : 8);
		if(top_pxl < (SS1306_OLED_GEOM_H - 8)) {
			ssd1306_move_raw((top_cell + 1), left_pxl);
			oled_put_tile(&obuf, (left_pxl > (SS1306_OLED_GEOM_W - 8)) ? 8 - hoff : 8);
		}
	}
}



void oled_puts(char *str) {
	uint8_t OLED_xpos = 0;
	uint8_t OLED_ypos = 0;
	while(*str != 0) {
		oled_putc(*str++);
		OLED_xpos++;
		if (OLED_xpos > 15)
		{
			OLED_xpos = 0;
			OLED_ypos++;
		}
		ssd1306_move(OLED_ypos,OLED_xpos );
	}
}

// box graphics digit (a single digit, not byte or word; use this to render output of an int->bcd conversion etc)
void oled_bigdigit(uint8_t top, uint8_t left, uint8_t num) {
	const uint8_t chartable[] = { 0x10, 0x0E, 0x5D, 0x5D, 0x0D, 0x1D, // zero
							              0x20, 0x0E, 0x20, 0x5D, 0x20, 0x11, // one
		                                  0x10, 0x0E, 0x10, 0x1D, 0x0D, 0x1D, // two
							              0x10, 0x0E, 0x20, 0x13, 0x0D, 0x1D, // three
							              0x5F, 0x5F, 0x0D, 0x13, 0x20, 0x5E, // four
							              0x10, 0x0E, 0x0D, 0x0E, 0x0D, 0x1D, // five
							              0x10, 0x0E, 0x0B, 0x0E, 0x0D, 0x1D, // six
							              0x10, 0x0E, 0x20, 0x5B, 0x20, 0x5E, // seven
							              0x10, 0x0E, 0x0B, 0x13, 0x0D, 0x1D, // eight
							              0x10, 0x0E, 0x0d, 0x13, 0x0D, 0x1D  // nine
						                };
	if(num > 9) { return; }
	ssd1306_move(top, left);
	for(uint8_t i = 0; i < 6; i++) {
		if(i == 2 || i == 4) {ssd1306_move(++top, left); }
		oled_putc(chartable[(num * 6) + i]);
	}

}

void oled_box(uint8_t top, uint8_t left, uint8_t width, uint8_t height) {
	ssd1306_move(top, left);
	oled_putc(BOX_TL); for(uint8_t i = 0; i < width - 2; i++) { oled_putc(BOX_HL);} oled_putc(BOX_TR);
	for(uint8_t i = top+1; i < top+height-1; i++) {
		ssd1306_move(i, left);
		oled_putc(BOX_VL);
		ssd1306_move(i, left + width - 1);
		oled_putc(BOX_VL);
	}
	ssd1306_move(top + height - 1, left);
	oled_putc(BOX_BL); for(uint8_t i = 0; i < width - 2; i++) { oled_putc(BOX_HL);} oled_putc(BOX_BR);
}

/*

"I tell you, we are here on Earth to fart around, and don't let anybody tell you different." - Kurt Vonnegut

*/
