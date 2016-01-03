program main;

{$apptype arm9}
{$define ARM9}

{$mode objfpc}

uses
  ctypes, nds9;

var
	frontBuffer: pcuint16;
  backBuffer: pcuint16;
  colorMask: cuint16;
  iy, ix: integer;
  temp: pcuint16;
			

begin
	irqInit();
	// a vblank interrupt is needed to use swiWaitForVBlank()
	// since the dispatcher handles the flags no handler is required
	irqEnable(IRQ_VBLANK);

	//set the mode for 2 text layers and two extended background layers
	videoSetMode(MODE_5_2D or DISPLAY_BG3_ACTIVE); 
	
	//set the sub background up for text display (we could just print to one
	//of the main display text backgrounds just as easily
	videoSetModeSub(MODE_0_2D or DISPLAY_BG0_ACTIVE); //sub bg 0 will be used to print text
	
	//set the first two banks as background memory and the third as sub background memory
	//D is not used..if you need a bigger background then you will need to map
	//more vram banks consecutivly (VRAM A-D are all 0x20000 bytes in size)
	vramSetMainBanks(	VRAM_A_MAIN_BG_0x06000000, VRAM_B_MAIN_BG_0x06020000, 
						VRAM_C_SUB_BG , VRAM_D_LCD); 

	// set up text background for text
	SUB_BG0_CR^ := BG_MAP_BASE(31);
	
	BG_PALETTE_SUB[255] := u32(RGB15(31,31,31));//by default font will be rendered with color 255
	
	//consoleInit() is a lot more flexible but this gets you up and running quick
	consoleInitDefault(pu16(SCREEN_BASE_BLOCK_SUB(31)), pu16(CHAR_BASE_BLOCK_SUB(0)), 16);

  printf(#10#10#9 + 'Hello DS devers' + #10);
  printf(#9 + 'www.drunkencoders.com' + #10);
  printf(#9 + 'double buffer demo');
	
	// set up our bitmap background
	
	BG3_CR^ := BG_BMP16_256x256;
	
	//these are rotation backgrounds so you must set the rotation attributes:
	//these are fixed point numbers with the low 8 bits the fractional part
	//this basicaly gives it a 1:1 translation in x and y so you get a nice flat bitmap
	BG3_XDX^ := 1 shl 8;
	BG3_XDY^ := 0;
	BG3_YDX^ := 0;
	BG3_YDY^ := 1 shl 8;
	//our bitmap looks a bit better if we center it so scroll down (256 - 192) / 2 
	BG3_CX^ := 0;
	BG3_CY^ := 0;

	
	frontBuffer := pcuint16($06000000);
	backBuffer :=  pcuint16($06000000 + 256 * 256 * 2);
	
	//this is just used so we can write red color bits to one frame and green to the 
	//other
	colorMask := $1F;
	
		while true do
		begin
			//draw a box
			for iy := 60 to 196 - 60 - 1 do
				for ix := 60 to 256 - 60 - 1 do
					backBuffer[iy * 256 + ix] := (rand() and colorMask) or BIT(15);
								
			swiWaitForVBlank();
				
			//swap
			temp := frontBuffer;
			frontBuffer := backBuffer;
			backBuffer := temp;
				
			//flip 
			//base is 16KB and screen size is 256x256x2 (128KB)
			BG3_CR^ := BG3_CR^ xor BG_BMP_BASE( 128 div 16 );
				
			//this will cause red or green bits only to be set and swap each
			//frame
			colorMask := colorMask xor $3FF;
		end;
end.
