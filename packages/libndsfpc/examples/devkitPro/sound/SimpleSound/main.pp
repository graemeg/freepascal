program main;
{$L blaster.raw.o}
{$L saberoff.raw.o}
{$L ion.raw.o}

{$apptype arm9}
{$define ARM9}

{$mode objfpc}

uses
  ctypes, nds9;


var
  blaster_raw_end: array [0..0] of u8; cvar; external;
  blaster_raw: array [0..0] of u8; cvar; external;
  blaster_raw_size: u32; cvar; external;
  ion_raw_end: array [0..0] of u8; cvar; external;
  ion_raw: array [0..0] of u8; cvar; external;
  ion_raw_size: u32; cvar; external;
  saberoff_raw_end: array [0..0] of u8; cvar; external;
  saberoff_raw: array [0..0] of u8; cvar; external;
  saberoff_raw_size: u32; cvar; external;

	blaster: TransferSoundData;
  keys: u16; 

begin

	powerON( POWER_LCD or POWER_2D_B );

	// initialise the irq dispatcher
	irqInit();
	// a vblank interrupt is needed to use swiWaitForVBlank()
	// since the dispatcher handles the flags no handler is required
	irqSet(IRQ_VBLANK, nil);
	irqEnable(IRQ_VBLANK);

	videoSetMode(0);	//not using the main screen
	videoSetModeSub(MODE_0_2D or DISPLAY_BG0_ACTIVE);	//sub bg 0 will be used to print text
	vramSetBankC(VRAM_C_SUB_BG); 

	SUB_BG0_CR^ := BG_MAP_BASE(31);
	
	BG_PALETTE_SUB[255] := (RGB15(31,31,31));	//by default font will be rendered with color 255
	
	//consoleInit() is a lot more flexible but this gets you up and running quick
	consoleInitDefault(pu16(SCREEN_BASE_BLOCK_SUB(31)), pu16(CHAR_BASE_BLOCK_SUB(0)), 16);

  printf(#10#10 + 'Simple Sound Demo'    + #10 + 
                  'Press A for SaberOff' + #10 + 
                  '      L for ion'      + #10 + 
                  '      R for blaster'  + #10);

	// set the generic sound parameters
	setGenericSound(	11025,	(* sample rate *)
						127,	(* volume *)
						64,		(* panning *)
						1 );	(* sound format*)


  with blaster do
  begin
    data := @blaster_raw;
    len := blaster_raw_size;
    rate := 11025;
    vol := 127;
    pan := 64;
    format := 1;
    PADDING := 0;
  end;


	while true do
	begin
		swiWaitForVBlank();
		scanKeys();
		
		keys := keysDown();
		
		if ( keys and KEY_L) <> 0 then playGenericSound(@ion_raw, ion_raw_size);

		if ( keys and KEY_A) <> 0 then playGenericSound(@saberoff_raw, saberoff_raw_size);

		if ( keys and KEY_R) <> 0 then playSound(@blaster);

	end;
end.
