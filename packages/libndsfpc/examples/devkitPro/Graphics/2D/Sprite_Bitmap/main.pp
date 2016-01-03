program main;
{$apptype arm9}
{$define ARM9}

{$mode objfpc}

uses
  ctypes, nds9;

var
  sprites: array [0..127] of SpriteEntry;
  spriteRotations: pSpriteRotation;
  i, angle: integer;

//turn off all the sprites
procedure initSprites();
var
  i: integer;
begin
	for i := 0 to 127 do
	begin
	  sprites[i].st.attribute[0] := ATTR0_DISABLED;
	  sprites[i].st.attribute[1] := 0;
	  sprites[i].st.attribute[2] := 0;
	  sprites[i].st.attribute[3] := 0;
  end;
end;

//copy our sprite to object attribute memory
procedure updateOAM();
begin
	DC_FlushRange(@sprites, 128 * sizeof(SpriteEntry));  
  dmaCopy(@sprites, OAM, 128 * sizeof(SpriteEntry));
end;

begin
  //rotation attributes overlap so assign then to the same location
  spriteRotations := pSpriteRotation(@sprites);
  
	//turn everything on
  powerON(POWER_ALL_2D);

  //irqs are nice
	irqInit();
	irqSet(IRQ_VBLANK, nil);
  
  //enable vram and map it to the right places
  vramSetMainBanks(   VRAM_A_MAIN_SPRITE,        //A and B maped consecutivly as sprite memory
                      VRAM_B_MAIN_SPRITE,        //this gives us 256KB which is the max
                      VRAM_C_MAIN_BG_0x06000000, //map C to background memory
                      VRAM_D_LCD                 //not using D
                      ); 
   
  //set the video mode
  videoSetMode( MODE_0_2D or 
                DISPLAY_SPR_ACTIVE or    //turn on sprites
                DISPLAY_BG0_ACTIVE or    //turn on background 0
                DISPLAY_SPR_1D or        //this is used when in tile mode
                DISPLAY_SPR_1D_BMP       //and this in bitmap mode
                );

   	
  // black backdrop
	BG_PALETTE[0] := u32(RGB15(0,0,0));

    
  BG0_CR^ := BG_MAP_BASE(31);//use bg0 for the text
	
  BG_PALETTE[255] := u32(RGB15(31,31,31));//by default font rendered with color 255
	
	//consoleInit() is a lot more flexible but this gets you up and running quick
	consoleInitDefault(pu16(SCREEN_BASE_BLOCK(31)), pu16(CHAR_BASE_BLOCK(0)), 16);

  //turn off the sprites
  initSprites();

	
  // direct bitmap sprite
  // print at using ansi escape sequence \x1b[line;columnH 
	printf(#27 + '[1;1H' + 'Direct Bitmap:');
  sprites[0].st.attribute[0] := ATTR0_BMP or ATTR0_ROTSCALE_DOUBLE or 10; 
	sprites[0].st.attribute[1] := ATTR1_SIZE_32 or 20;
	sprites[0].st.attribute[2] := ATTR2_ALPHA(1)or 0;

	// red 32*32 square for 1d bitmap mode
	for i := 0 to 32*32 - 1 do
		SPRITE_GFX[i] := RGB15(31,0,0) or (1 shl 15); //dont forget alpha bit

  // 256 color sprite
  // print at using ansi escape sequence \x1b[line;columnH 
	printf(#27 + '[9;1H' + '256 color:');
	sprites[1].st.attribute[0] := ATTR0_COLOR_256 or ATTR0_ROTSCALE_DOUBLE or 75;
	sprites[1].st.attribute[1] := ATTR1_SIZE_32 or 20; // size 64x64, x 10
	sprites[1].st.attribute[2] := 64;

    // Blue for 256 color sprite
	SPRITE_PALETTE[1] := RGB15(0,0,31);

    // blue 64*64 square for 256 color mode (must write two pixles at time)
	for i := 0 to 32*16 - 1 do
		SPRITE_GFX[i+64*16] := (1 shl 8) or 1;

  // 16 color sprite
  // print at using ansi escape sequence \x1b[line;columnH 
  printf(#27 + '[16;1H' + '16 color:');
	sprites[2].st.attribute[0] := ATTR0_COLOR_16 or ATTR0_ROTSCALE_DOUBLE or 135;
	sprites[2].st.attribute[1] := ATTR1_SIZE_32 or 20;
	sprites[2].st.attribute[2] := ATTR2_PALETTE(1) or 96;
  
  //yellow for 16 color sprite (it is using palette 1 so colors 16-31)
  SPRITE_PALETTE[17] := RGB15(31,31,0);

  // yellow 32*32 square for 16 color mode (must write 4 pixels at a time)
	for i := 0 to 32*8 - 1 do
		SPRITE_GFX[i+96*16] := (1 shl 12) or (1 shl 8) or (1 shl 4) or 1;



	angle:=0;
    
  //we tied all our sprites to the same rotation attributes (0)
	spriteRotations[0].hdx := 256;
	spriteRotations[0].hdy := 0;
	spriteRotations[0].vdx := 0;
	spriteRotations[0].vdy := 256;

	while true do
	begin
		angle := angle + 1;
		
		spriteRotations[0].hdx := COS_bin[angle and $1FF] shr 4;
		spriteRotations[0].hdy := SIN_bin[angle and $1FF] shr 4;
		spriteRotations[0].vdx := -spriteRotations[0].hdy;
		spriteRotations[0].vdy := spriteRotations[0].hdx;
		
		swiWaitForVBlank();
		
		updateOAM();
	end;
end.
