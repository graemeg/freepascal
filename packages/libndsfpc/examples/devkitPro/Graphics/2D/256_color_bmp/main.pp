program main;
{$L drunkenlogo.o}

{$apptype arm9}
{$define ARM9}

{$mode objfpc}

uses
  ctypes, nds9;

const
	drunkenlogoPalLen = 512;
	drunkenlogoBitmapLen = 65536;

var
	drunkenlogoPal: array [0..255] of cushort; cvar; external;
	drunkenlogoBitmap: array [0..16383] of cuint; cvar; external;


begin	
  //irqs are nice
  irqInit();
  irqSet(IRQ_VBLANK, nil);
  
  //set the mode for 2 text layers and two extended background layers
  videoSetMode(MODE_5_2D or DISPLAY_BG3_ACTIVE);
  
  //set the sub background up for text display (we could just print to one
  //of the main display text backgrounds just as easily
  videoSetModeSub(MODE_0_2D or DISPLAY_BG0_ACTIVE); //sub bg 0 will be used to print text
  
  //set the first bank as background memory and the third as sub background memory
  //B and D are not used
  vramSetMainBanks(VRAM_A_MAIN_BG_0x06000000, VRAM_B_LCD, VRAM_C_SUB_BG, VRAM_D_LCD);
  
  ////////////////set up text background for text/////////////////////
  SUB_BG0_CR^ := BG_MAP_BASE(31);
  
  BG_PALETTE_SUB[255] := u32(RGB15(31,31,31));//by default font will be rendered with color 255
  
  //consoleInit() is a lot more flexible but this gets you up and running quick
  consoleInitDefault(pu16(SCREEN_BASE_BLOCK_SUB(31)), pu16(CHAR_BASE_BLOCK_SUB(0)), 16);
  
  iprintf(#10#10#9 + 'Hello DS devers' + #10);
  iprintf(#9 + 'www.drunkencoders.com' + #10);
  iprintf(#9 + '256 color bitmap demo');
  
  ///////////////set up our bitmap background///////////////////////
  
  BG3_CR^ := BG_BMP8_256x256;
  
  //these are rotation backgrounds so you must set the rotation attributes:
  //these are fixed point numbers with the low 8 bits the fractional part
  //this basicaly gives it a 1:1 translation in x and y so you get a nice flat bitmap
  BG3_XDX^ := 1 shl 8;
  BG3_XDY^ := 0;
  BG3_YDX^ := 0;
  BG3_YDY^ := 1 shl 8;
  //our bitmap looks a bit better if we center it so scroll down (256 - 192) / 2
  BG3_CX^ := 0;
  BG3_CY^ := 32 shl 8;
  
{
  for i := 0 to 256*2 - 1 do
    BG_PALETTE[i] := palette_bin[i];
  for i := 0 to 256*256 - 1 do
    BG_GFX[i] := drunkenlogo_bin[i];
}  


	dmaCopy(@drunkenlogoBitmap, BG_GFX, drunkenlogoBitmapLen);
	dmaCopy(@drunkenlogoPal, BG_PALETTE, drunkenlogoPalLen);
  
  while true do 
    swiWaitForVBlank();
  
end.
