program main;

{$apptype arm9}
{$define ARM9}

{$mode objfpc}

uses
  ctypes, nds9;

var
  frame: integer;
	touchXY: touchPosition;



function Vblank(): pointer;
begin
  inc(frame);
end;
	
begin

	irqInit();
	irqSet(IRQ_VBLANK, @Vblank);
	irqEnable(IRQ_VBLANK);
	videoSetMode(0);	//not using the main screen
	videoSetModeSub(MODE_0_2D or DISPLAY_BG0_ACTIVE);	//sub bg 0 will be used to print text
	vramSetBankC(VRAM_C_SUB_BG); 

	SUB_BG0_CR^ := BG_MAP_BASE(31);
	
	BG_PALETTE_SUB[255] := u32(RGB15(31,31,31));	//by default font will be rendered with color 255
	
	//consoleInit() is a lot more flexible but this gets you up and running quick
	consoleInitDefault(pu16(SCREEN_BASE_BLOCK_SUB(31)), pu16(CHAR_BASE_BLOCK_SUB(0)), 16);

	printf('      Hello DS dev''rs' + #10);
	printf('     www.devkitpro.org' + #10);
	printf('   www.drunkencoders.com');

	while true do
	begin
		swiWaitForVBlank();
		touchXY := touchReadXY();

		// print at using ansi escape sequence #27 + [line;columnH 
		iprintf(#27 + '[10;0H' + 'Frame = %d',[frame]);
		iprintf(#27 + '[16;0H' + 'Touch x = %04X, %04X' + #10, [touchXY.x, touchXY.px]);
		iprintf('Touch y = %04X, %04X' + #10, [touchXY.y, touchXY.py]);		
	
	end;

end.
