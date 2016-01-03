program song_events_example2;
{$L build/mmsolution.bin.o}

{$mode objfpc}

uses
  ctypes, nds9, maxmod9;

const
  MOD_EXAMPLE2	= 0;
  MSL_NSONGS	= 1;
  MSL_NSAMPS	= 3;
  MSL_BANKSIZE	= 4;

var
  mmsolution_bin_end: array [0..0] of cuint8; cvar; external;
  mmsolution_bin: array [0..0] of cuint8; cvar; external;
  mmsolution_bin_size: cuint32; cvar; external;



// a simple sprite structure
type
  MySprite = record
    gfx: pcuint16;
    size: SpriteSize;
    format: SpriteColorFormat;
    rotationIndex: cint;
    paletteAlpha: cint;
    x: cint;
    y: cint;
    dy: cint;
  end;


var
  sprites: array[0..4] of MySprite;


<<<<<<< .minefunction myEventHandler(msg, param: mm_word): mm_word;
=======function myEventHandler(msg, param: mm_word): pmm_word;
>>>>>>> .theirsbegin
  case msg of
    MMCB_SONGMESSAGE:	// process song messages
    begin
			if (param = 1) then sprites[0].dy := -8;
			if (param = 2) then sprites[1].dy := -8;
			if (param = 3) then sprites[2].dy := -8;
			if (param = 4) then sprites[3].dy := -8;
			if (param = 5) then sprites[4].dy := -8;
    end;
    MMCB_SONGFINISHED:;	// process song finish message (only triggered in songs played with MM_PLAY_ONCE)
  end;
  result := 0;
end;



var
  j: integer;
	i: integer;

begin
  for j := 0 to 4 do
  with sprites[j] do
  begin
    gfx := nil; 
    size := SpriteSize_16x16; 
    format := SpriteColorFormat_256Color;
    rotationIndex := 0; 
    paletteAlpha := 0; 
    x := 20 +(50 * j); 
    y := 96; 
    dy := 0; 
  end;
	
	videoSetMode(MODE_0_2D);
	videoSetModeSub(0);   // not using subscreen

	lcdMainOnBottom();
	
	//initialize the sprite engine with 1D mapping 128 byte boundary
	//and no external palette support
	oamInit(oamMain, SpriteMapping_1D_32, false);

	vramSetBankA(VRAM_A_MAIN_SPRITE);
	
	for i := 0 to 4 do
	begin
		//allocate some space for the sprite graphics
		sprites[i].gfx := oamAllocateGfx(oamMain, sprites[i].size, sprites[i].format);

		//fill each sprite with a different index (2 pixels at a time)
		dmaFillHalfWords( ((i+1) shl 8) or (i + 1), sprites[i].gfx, 32*32);
	end;

	//set indexes to different colours
	SPRITE_PALETTE[1] := RGB15(31,0,0);
	SPRITE_PALETTE[2] := RGB15(0,31,0);
	SPRITE_PALETTE[3] := RGB15(0,0,31);
	SPRITE_PALETTE[4] := RGB15(31,0,31);
	SPRITE_PALETTE[5] := RGB15(0,31,31);

	// initialise maxmod using default settings, and enable interface for soundbank that is loaded into memory
	mmInitDefaultMem( mm_addr(@mmsolution_bin));

	// setup maxmod to use the song event handler
	mmSetEventHandler( mm_callback(@myEventHandler));
	
	// load song
	// values for this function are in the solution header
	mmLoad( MOD_EXAMPLE2 );

	// start song playing
	mmStart( MOD_EXAMPLE2, MM_PLAY_LOOP );

	while true do
	begin
		for i := 0 to 4 do
		begin
			// constantly increase the sprite's y velocity
			sprites[i].dy := sprites[i].dy + 1;
		
			// update the sprite's y position with its y velocity
			sprites[i].y := sprites[i].y + sprites[i].dy;
		
			// clamp the sprite's y position
			if ( sprites[i].y<72 ) then sprites[i].y := 72;
			if ( sprites[i].y>96 ) then sprites[i].y := 96;
		
			oamSet(oamMain, 					//main graphics engine context
					i,           				//oam index (0 to 127)  
					sprites[i].x,				//x and y pixel location of the sprite
					sprites[i].y, 			
					0,							//priority, lower renders last (on top)
					sprites[i].paletteAlpha,	//palette index 
					sprites[i].size,
					sprites[i].format,
					sprites[i].gfx,				//pointer to the loaded graphics
					sprites[i].rotationIndex,	//sprite rotation data  
					false,						//double the size when rotating?
					false,			//hide the sprite?
					false, false, //vflip, hflip
					false	//apply mosaic
					);              
			end;

		swiWaitForVBlank();
		
		//send the updates to the hardware
		oamUpdate(oamMain);
	end;

end.


