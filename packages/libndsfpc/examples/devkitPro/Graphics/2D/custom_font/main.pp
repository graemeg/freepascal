program main;
{$L font.o}
{$apptype arm9}
{$define ARM9}

{$mode objfpc}

uses
  ctypes, nds9;

var
  i: integer;
	sub_tile: pu16;
	sub_map: pu16;

const
  font_WIDTH	= 8;
  font_HEIGHT	= 768;
  
  fontPalLen = 512;
  fontBitmapLen = 6144;

var
//byte array representing the picture
  fontBitmap: array [0..1535] of cuint16; cvar; external;
  fontPal: array [0..255] of cuint16; cvar; external;


const
  char_base = 0;
	screen_base = 20;
  

begin
	irqInit();
	irqEnable(IRQ_VBLANK);

	videoSetMode(0);	
	videoSetModeSub(MODE_0_2D or DISPLAY_BG0_ACTIVE);	
	vramSetBankC(VRAM_C_SUB_BG); 

	SUB_BG0_CR^ := BG_256_COLOR or BG_TILE_BASE(char_base) or BG_MAP_BASE(screen_base);

	sub_tile := pu16(CHAR_BASE_BLOCK_SUB(char_base));
	sub_map := pu16(SCREEN_BASE_BLOCK_SUB(screen_base));

	//95 and 32 show how many characters there are and 32 shows which ASCII character to start, respectively
	//95 is the smaller set of ACSII characters. It usually will start with 32
	consoleInit(pu16(fontBitmap), sub_tile, 95, 32, sub_map, CONSOLE_USE_COLOR255, 8);
    
	//Load the Font Data and Palette stuff here
	for i := 0 to fontBitmapLen - 1 do
		sub_tile[i] := u32(fontBitmap[i]);



	for i := 0 to fontPalLen - 1 do
		BG_PALETTE_SUB[i] := u32(fontPal[i]);
	


	printf('Custom Font Demo' + #10);
	printf('   by Poffy' + #10);
	printf('modified by WinterMute' + #10);
	printf('for libnds examples' + #10);

	while true do
		swiWaitForVBlank();
end.
