program main;
{$L drunkenlogo.bin.o}
{$L palette.bin.o}

{$apptype arm9}
{$define ARM9}

{$mode objfpc}

uses
  ctypes, nds9;

var
  drunkenlogo_bin_end: array [0..0] of u8; cvar; external;
  drunkenlogo_bin: array [0..0] of u8; cvar; external;
  drunkenlogo_bin_size: u32; cvar; external;

  palette_bin_end: array [0..0] of u8; cvar; external;
  palette_bin: array [0..0] of u8; cvar; external;
  palette_bin_size: u32; cvar; external;

  angle: u32;
	scrollX, scrollY: s16;
	scaleX, scaleY: s16;
	rcX, rcY: s16;
  keys: u32;
  s, c: s16;


begin
  irqInit();
  irqSet(IRQ_VBLANK, nil);
  
  // set the mode for 2 text layers and two extended background layers
  videoSetMode(MODE_5_2D or DISPLAY_BG3_ACTIVE);
  
  // set the sub background up for text display (we could just print to one
  // of the main display text backgrounds just as easily
  videoSetModeSub(MODE_0_2D or DISPLAY_BG0_ACTIVE); //sub bg 0 will be used to print text
  
  // set the first bank as background memory and the third as sub background memory
  // B and D are not used
  vramSetMainBanks(	VRAM_A_MAIN_BG_0x06000000, VRAM_B_LCD, VRAM_C_SUB_BG, VRAM_D_LCD);
  
  // set up text background for text
  SUB_BG0_CR^ := BG_MAP_BASE(31);
  
  BG_PALETTE_SUB[255] := u32(RGB15(31,31,31));//by default font will be rendered with color 255
  
  //consoleInit() is a lot more flexible but this gets you up and running quick
	consoleInitDefault(pu16(SCREEN_BASE_BLOCK_SUB(31)), pu16(CHAR_BASE_BLOCK_SUB(0)), 16);
  
  
  
  // set up our bitmap background
  
  BG3_CR^ := BG_BMP8_256x256;
  
  // these are rotation backgrounds so you must set the rotation attributes:
  // these are fixed point numbers with the low 8 bits the fractional part
  // this basicaly gives it a 1:1 translation in x and y so you get a nice flat bitmap
  BG3_XDX^ := 1 shl 8;
  BG3_XDY^ := 0;
  BG3_YDX^ := 0;
  BG3_YDY^ := 1 shl 8;
  // our bitmap looks a bit better if we center it so scroll down (256 - 192) / 2
  BG3_CX^ := 0;
  BG3_CY^ := 32 shl 8;
  
  dmaCopy(@drunkenlogo_bin, BG_GFX, 256*256);
  dmaCopy(@palette_bin, BG_PALETTE, 256*2);
  
  angle := 0;
  
  // the screen origin is at the rotation center...so scroll to the rotation
  // center + a small 32 pixle offset so our image is centered
  scrollX := 0 + 128;
  scrollY := 32 + 96 ;
  
  //scale is fixed point
  scaleX := 1 shl 8;
  scaleY := 1 shl 8;
  
  //this is the screen pixel that the image will rotate about
  rcX := 128;
  rcY := 96;
  
  while true do
  begin
    printf(#10#10#9 + 'Hello DS devers' + #10);
    printf(#9 + 'www.drunkencoders.com' + #10);
    printf(#9 + 'BG Rotation demo' + #10);
    
    iprintf('Angle %3d(actual) %3d(degrees)' + #10, [angle and $1FF, (angle and $1FF) * 360 div 512]);
    iprintf('Scroll  X: %4d Y: %4d' + #10, [scrollX, scrollY]);
    iprintf('Rot center X: %4d Y: %4d' + #10, [rcX, rcY]);
    iprintf('Scale X: %4d Y: %4d' + #10, [scaleX, scaleY]);
    
    scanKeys();
    keys := keysHeld();
    
    if ( keys and KEY_L ) <> 0 then angle := angle + 1;
    if ( keys and KEY_R ) <> 0 then angle := angle - 1;
    if ( keys and KEY_LEFT ) <> 0 then scrollX := scrollX + 1;
    if ( keys and KEY_RIGHT ) <> 0 then  scrollX := scrollX - 1;
    if ( keys and KEY_UP ) <> 0 then scrollY := scrollY + 1;
    if ( keys and KEY_DOWN ) <> 0 then scrollY := scrollY - 1;
    if ( keys and KEY_A ) <> 0 then scaleX := scaleX + 1;
    if ( keys and KEY_B ) <> 0 then scaleX := scaleX - 1;
    if ( keys and KEY_START ) <> 0 then rcX := rcX + 1;
    if ( keys and KEY_SELECT ) <> 0 then rcY := rcY + 1;
    if ( keys and KEY_X ) <> 0 then scaleY := scaleY + 1;
    if ( keys and KEY_Y ) <> 0 then scaleY := scaleY - 1;
    
    // Compute sin and cos
    s := SIN_bin[angle and $1FF] shr 4;
    c := COS_bin[angle and $1FF] shr 4;
    
    swiWaitForVBlank();
    
    // Set the background registers
    
    BG3_XDX^ := cuint16(( c * scaleX ) shr 8);
    BG3_XDY^ := cuint16((-s * scaleX ) shr 8);
    
    BG3_YDX^ := cuint16(( s * scaleY ) shr 8);
    BG3_YDY^ := cuint16(( c * scaleY ) shr 8);
    
    BG3_CX^ := cuint32((scrollX shl 8) - rcX * (c - s));
    BG3_CY^ := cuint32((scrollY shl 8) - rcY * (s + c));
    
    // clear the console screen (ansi escape sequence)
    printf(#27 + '[2J');
  
  end;

end.
