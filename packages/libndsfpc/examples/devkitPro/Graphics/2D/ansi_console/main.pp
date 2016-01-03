program main;

{$apptype arm9}
{$define ARM9}

{$mode objfpc}

uses
  ctypes, nds9;

begin

//---------------------------------------------------------------------------------
	// initialise the irq dispatcher
	irqInit();
	// a vblank interrupt is needed to use swiWaitForVBlank()
	// since the dispatcher handles the flags no handler is required
	irqEnable(IRQ_VBLANK);
	videoSetMode(0);	//not using the main screen
	videoSetModeSub(MODE_0_2D or DISPLAY_BG0_ACTIVE);	//sub bg 0 will be used to print text
	vramSetBankC(VRAM_C_SUB_BG);

	SUB_BG0_CR^ := BG_MAP_BASE(31);

	BG_PALETTE_SUB[255] := u32(RGB15(31,31,31));	//by default font will be rendered with color 255

	//consoleInit() is a lot more flexible but this gets you up and running quick
	consoleInitDefault(pu16(SCREEN_BASE_BLOCK_SUB(31)), pu16(CHAR_BASE_BLOCK_SUB(0)), 16);

	// ansi escape sequence to clear screen and home cursor
	// #27 + [line;columnH
	iprintf(#27 + '[2J');

	// ansi escape sequence to set print co-ordinates
	// #27 + [line;columnH
	iprintf(#27 + '[10;10H' + 'Hello World!');

	// ansi escape sequence to move cursor up
	// #27 + [linesA
	iprintf(#27 + '[10A' + 'Line 0');

	// ansi escape sequence to move cursor left
	// #27 + [columnsD
	iprintf(#27 + '[28D' + 'Column 0');

	// ansi escape sequence to move cursor down
	// #27 + [linesB
	iprintf(#27 + '[19B' + 'Line 19');

	// ansi escape sequence to move cursor right
	// #27 + [columnsC
	iprintf(#27 + '[5C' + 'Column 20');

	while true do 
    swiWaitForVBlank();

end.
