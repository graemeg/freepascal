program main;
{$L drunkenlogo.o}

{$apptype arm9}
{$define ARM9}

{$mode objfpc}

uses
  ctypes, nds9;

const
  drunkenlogoBitmapLen = 26988;

var
  drunkenlogoBitmap: array [0..6746] of cuint; cvar; external;


function getSize(source: pcuint8; dest: pcuint16; arg: cuint32): integer; 
begin
	getSize := pcuint32(source)^;
end;

function readByte(source: pcuint8): cuint8;
begin
 	readByte := source^;
end;


var
  drunkenlogo_decomp: TDecompressionStream;

begin
  drunkenlogo_decomp.getSize := TGetSize(@getSize);
  drunkenlogo_decomp.getResult := nil;
  drunkenlogo_decomp.readByte := TReadByte(@readByte);
	// irqs are nice
	irqInit();
	irqEnable(IRQ_VBLANK);

  // set the mode for 2 text layers and two extended background layers
	videoSetMode(MODE_5_2D or DISPLAY_BG3_ACTIVE);

	// set the sub background up for text display (we could just print to one
	// of the main display text backgrounds just as easily
	videoSetModeSub(MODE_0_2D or DISPLAY_BG0_ACTIVE); //sub bg 0 will be used to print text

  // set the first bank as background memory and the third as sub background memory
  // B and D are not used (if you want a bitmap greater than 256x256 you will need more
  // memory so another vram bank must be used and mapped consecutivly
  vramSetMainBanks(VRAM_A_MAIN_BG_0x06000000, VRAM_B_LCD, VRAM_C_SUB_BG, VRAM_D_LCD);

	// set up text background for text
  SUB_BG0_CR^ := BG_MAP_BASE(31);

	BG_PALETTE_SUB[255] := RGB15(31,31,31);//by default font will be rendered with color 255

	//consoleInit() is a lot more flexible but this gets you up and running quick
	consoleInitDefault(pu16(SCREEN_BASE_BLOCK_SUB(31)), pu16(CHAR_BASE_BLOCK_SUB(0)), 16);

	iprintf(#10#10#9 + 'Hello DS devers' + #10);
	iprintf(#9 + 'www.drunkencoders.com' + #10);
	iprintf(#9 + '16 bit bitmap demo');

	// set up our bitmap background
	BG3_CR^ := BG_BMP16_256x256;

	// these are rotation backgrounds so you must set the rotation attributes:
    // these are fixed point numbers with the low 8 bits the fractional part
    // this basicaly gives it a 1:1 translation in x and y so you get a nice flat bitmap
	BG3_XDX^ := 1 shl 8;
	BG3_XDY^ := 0;
	BG3_YDX^ := 0;
	BG3_YDY^ := 1 shl 8;

	BG3_CX^ := 0;
	BG3_CY^ := 0;

	swiDecompressLZSSVram(@drunkenlogoBitmap, BG_GFX, 0, @drunkenlogo_decomp);
	while true do 
    swiWaitForVBlank();

end.
