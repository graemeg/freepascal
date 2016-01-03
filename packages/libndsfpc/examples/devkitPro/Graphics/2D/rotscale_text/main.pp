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
	scaleX, scaleY: s16;
	scrollX, scrollY: s16;
  rcX, rcY: s16;
	angle: cuint = 0;
  keys: u32;
	angleSin, angleCos: s16;

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
	videoSetModeSub(MODE_5_2D or DISPLAY_BG3_ACTIVE or DISPLAY_BG_EXT_PALETTE);	
	vramSetBankC(VRAM_C_SUB_BG); 

	SUB_BG3_CR^ := BG_TILE_BASE(char_base) or BG_MAP_BASE(screen_base) or ROTBG_SIZE_256x256;
	
	sub_tile := pu16(CHAR_BASE_BLOCK_SUB(char_base));
	sub_map := pu16(SCREEN_BASE_BLOCK_SUB(screen_base));

	//95 and 32 show how many characters there are and 32 shows which ASCII character to start, respectively
	//95 is the smaller set of ACSII characters. It usually will start with 32
	consoleInit(pu16(fontBitmap), sub_tile, 95, 32, sub_map, CONSOLE_USE_COLOR255, 8);
    
	//Load the Font Data and Palette stuff here
	for i := 0 to fontBitmapLen - 1 do
		sub_tile[i] := u32(fontBitmap[i]);
	// extended palettes are written with bank mapped to lcd
	vramSetBankH(VRAM_H_LCD); 

	for i := 0 to fontPalLen - 1 do
		VRAM_H_EXT_PALETTE[3, 0, i] := u32(fontPal[i]);
	
	// map bank to extended palette after writing data
	vramSetBankH(VRAM_H_SUB_BG_EXT_PALETTE); 




	iprintf('Custom Font Demo' + #10);
	iprintf('   by Poffy' + #10);
	iprintf('modified by WinterMute' + #10);
	iprintf('for libnds examples' + #10);

	//scale is fixed point
	scaleX := 1 shl 8; 
	scaleY := 1 shl 8;

	scrollX := 128;
	scrollY := 96;

	//this is the screen pixel that the image will rotate about
	rcX := 128; 
	rcY := 96;

	angle := 0;

	while true do 
	begin
		scanKeys();
		keys := keysHeld();

		if ( keys and KEY_L ) <> 0 then angle := angle + 1; 
		if ( keys and KEY_R )  <> 0 then angle := angle - 1; 

		if ( keys and KEY_LEFT )  <> 0 then scrollX := scrollX + 1;
		if ( keys and KEY_RIGHT )  <> 0 then scrollX := scrollX - 1;
		if ( keys and KEY_UP )  <> 0 then scrollY := scrollY + 1;
		if ( keys and KEY_DOWN )  <> 0 then scrollY := scrollY - 1;

		if ( keys and KEY_A )  <> 0 then scaleX := scaleX + 1;
		if ( keys and KEY_B )  <> 0 then scaleX := scaleX - 1;

		if( keys and KEY_X )  <> 0 then scaleY := scaleY + 1;
		if( keys and KEY_Y )  <> 0 then scaleY := scaleY - 1;

		// wrap angle
		angle := angle and $1ff;

		// Compute sin and cos
		angleSin := SIN_bin[angle] shr 4;
		angleCos := COS_bin[angle] shr 4;
 
		swiWaitForVBlank();

		// Set the background registers
		SUB_BG3_XDX^ := cuint16(( angleCos * scaleX ) shr 8);
		SUB_BG3_XDY^ := cuint16((-angleSin * scaleX ) shr 8);
		SUB_BG3_YDX^ := cuint16(( angleSin * scaleY ) shr 8);
		SUB_BG3_YDY^ := cuint16(( angleCos * scaleY ) shr 8);

		SUB_BG3_CX^ := cuint16((scrollX shl 8) - rcX * ( angleCos - angleSin));
		SUB_BG3_CY^ := cuint16((scrollY shl 8) - rcY * ( angleSin + angleCos));

	end;

end.
