program main;
{$L ballpalette.bin.o}
{$L balldata.bin.o}
{$apptype arm9}
{$define ARM9}

{$mode objfpc}

uses
  ctypes, nds9;

var
  OAMCopy: array [0..127] of SpriteEntry;
  ballpalette_bin_end: array [0..0] of u8; cvar; external;
  ballpalette_bin: array [0..0] of u16; cvar; external;
  ballpalette_bin_size: u32; cvar; external;
  balldata_bin_end: array [0..0] of u8; cvar; external;
  balldata_bin: array [0..0] of u16; cvar; external;
  balldata_bin_size: u32; cvar; external;


procedure initOAM();
var
  i: integer;
begin
	for i := 0 to 127 do
		OAMCopy[i].st.attribute[0] := ATTR0_DISABLED;
end;

//---------------------------------------------------------------------------------
procedure updateOAM();
begin	
  dmaCopy(@OAMCopy, OAM, sizeof(OAMCopy));
end;


type
  TTouchType = (ttContinuous, ttSingle);

var
  frame: integer;
  TouchType: TTouchType = ttContinuous;


//---------------------------------------------------------------------------------
function Vblank(): pointer;
begin
//---------------------------------------------------------------------------------
	inc(frame);
end;


var
  min_x, min_y, max_x, max_y: integer;
  min_px, min_py, max_px, max_py: integer;
	touch: touchPosition;
  i: integer;
	pressed, held: integer;

begin
	min_x  := 4096;
  min_y  := 4096; 
  max_x  := 0;
  max_y  := 0;
	min_px := 4096;
  min_py := 4096;
  max_px := 0;
  max_py := 0;
  

	powerON(POWER_ALL_2D);
	
	// put the main screen on the bottom lcd
	lcdMainOnBottom();
	
	// Initialise the interrupt system
	irqInit();
	// install our simple vblank handler
	irqSet(IRQ_VBLANK, @Vblank);
	// enable the interrupt
	irqEnable(IRQ_VBLANK);
	initOAM();
    //enable vram and map it to the right places
    vramSetMainBanks(   VRAM_A_MAIN_SPRITE,        //A and B maped consecutivly as sprite memory
                        VRAM_B_MAIN_SPRITE,        //this gives us 256KB which is the max
                        VRAM_C_MAIN_BG_0x06000000,  //map C to background memory
                        VRAM_D_LCD                 //not using D
                        ); 
   
   //set the video mode
    videoSetMode(  MODE_0_2D or 
                   DISPLAY_SPR_ACTIVE or		//turn on sprites
                   DISPLAY_BG0_ACTIVE or		//turn on background 0
                   DISPLAY_SPR_1D			//this is used when in tile mode
                    );
	
	// Sprite initialisation
	for i := 0 to 255 do
		SPRITE_PALETTE[i] := u32(ballpalette_bin[i]);

	for i := 0 to 32*16 - 1 do
		SPRITE_GFX[i] := u32(balldata_bin[i]);

	// black backdrop
	BG_PALETTE[0] := (RGB15(0,0,0));

	BG0_CR^ := BG_MAP_BASE(31);//use bg0 for the text
	
	BG_PALETTE[255] := (RGB15(31,31,31));//by default font rendered with color 255
	
	//consoleInit() is a lot more flexible but this gets you up and running quick
	consoleInitDefault(pu16(SCREEN_BASE_BLOCK(31)), pu16(CHAR_BASE_BLOCK(0)), 16);

 
	printf(#27 + '[4;8H' + 'Touch Screen Test');
	printf(#27 + '[15;4H' + 'Right Shoulder toggles');
 
	while true do
	begin
		swiWaitForVBlank();
		updateOAM();

		// read the button states
		scanKeys();

		// read the touchscreen coordinates
		touch := touchReadXY();
		
		pressed := keysDown();	// buttons pressed this loop
		held := keysHeld();		// buttons currently held

		// Right Shoulder button toggles the mode
		if ( pressed and KEY_R) <> 0 then Inc(TouchType);

		if TouchType = ttContinuous then 
      printf(#27 + '[14;4H' + 'Touch mode: CONTINUOUS ')
    else
      printf(#27 + '[14;4H' + 'Touch mode: SINGLE SHOT');

		iprintf(#27 + '[6;5H' + 'Touch x = %04X, %04X' + #10, [touch.x, touch.px]);
		iprintf(#27 + '[7;5H' + 'Touch x = %04X, %04X' + #10, [touch.x, touch.px]);


		iprintf(#27 + '[0;18H' + 'keys: %08X' + #10, [keysHeld()]);
		iprintf(#27 + '[9;10H' + 'Frame %d' + #10, [frame]);

		if (TouchType = ttSingle) and not ( (pressed and KEY_TOUCH) <> 0) then continue;

		if ((held and KEY_TOUCH)<0) or (touch.x = 0) or (touch.y = 0) then continue;
		
		iprintf(#27 + '[12;12H' + '(%d,%d)      ', [touch.px,touch.py]);

		if ( touch.x > max_x)	then	max_x := touch.x;
		if ( touch.y > max_y)		then	max_y := touch.y;
		if ( touch.px > max_px)	then	max_px := touch.px;
		if ( touch.py > max_py)	then	max_py := touch.py;

		if ( touch.x < min_x)		then	min_x := touch.x;
		if ( touch.y < min_y)		then	min_y := touch.y;
		if ( touch.px < min_px)	then	min_px := touch.px;
		if ( touch.py < min_py)	then	min_py := touch.py;

		iprintf(#27 + '[0;0H' + '(%d,%d)      ',[min_px,min_py]);
		iprintf(#27 + '[1;0H' + '(%d,%d)      ',[min_x,min_y]);
		iprintf(#27 + '[22;21H' + '(%d,%d)',[max_x,max_y]);
		iprintf(#27 + '[23;23H' + '(%d,%d)',[max_px,max_py]);

		OAMCopy[0].st.attribute[2] := 0;
		OAMCopy[0].st.attribute[1] := ATTR1_SIZE_32 or ((touch.px - 16) and $01FF);
		OAMCopy[0].st.attribute[0] := ATTR0_COLOR_256 or ATTR0_SQUARE or ((touch.py -16) and $00FF);
		
	end;

end.
