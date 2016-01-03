program main;
{$L ball.pcx.o}

{$apptype arm9}
{$define ARM9}

{$mode objfpc}

uses
  ctypes, nds9;

var
  ball_pcx_end: array [0..0] of u8; cvar; external;
  ball_pcx: array [0..0] of u8; cvar; external;
  ball_pcx_size: u32; cvar; external;


const
  NUM_SPRITES = 128;

var
  OAMCopySub: array [0..127] of SpriteEntry;

type
//simple sprite struct
  TSprite = record
    x, y: integer;				//location 
    dx, dy: integer;			//speed
    oam: PSpriteEntry;	
    gfxID: integer; 				//graphics lovation
  end;
  PSprite = ^TSprite;


procedure MoveSprite(sp: PSprite);
var
  x, y: integer;
begin
  x := sp^.x shr 8;
  y := sp^.y shr 8;

  sp^.oam^.st.attribute[1] := sp^.oam^.st.attribute[1] and $FE00;
  sp^.oam^.st.attribute[1] := sp^.oam^.st.attribute[1] or (x and $01FF);
  
  sp^.oam^.st.attribute[0] := sp^.oam^.st.attribute[0] and $FF00;
  sp^.oam^.st.attribute[0] := sp^.oam^.st.attribute[0] or (y and $00FF);
end; 

procedure initOAM(); 
var
  i: integer;
begin
  for i := 0 to 127 do
    OAMCopySub[i].st.attribute[0] := ATTR0_DISABLED;
end;

procedure updateOAM();
begin
  dmaCopy(@OAMCopySub, OAM_SUB, sizeof(OAMCopySub));
end;


function irqVBlank(): pointer;
begin	
end;

var
	back, front: pcuint16;
  sprites: array [0..NUM_SPRITES - 1] of TSprite;
  i, delta: integer;
  ix, iy: integer;
  screen: integer;
  map0, map1: pcuint16;
  red: cuint16;
  ball: sImage;

begin	
	back := VRAM_A;
	front := VRAM_B;

	i := 0;
  delta := 0;
	ix := 0;
	iy := 0;
	screen := 1;
	map0 := pcuint16(SCREEN_BASE_BLOCK_SUB(1));
	map1 := pcuint16(SCREEN_BASE_BLOCK_SUB(2));
	
	//turn on the power to the system
	powerON(POWER_ALL);

	//set main display to render directly from the frame buffer
	videoSetMode(MODE_FB1);
	
	//set up the sub display
	videoSetModeSub(MODE_0_2D or DISPLAY_SPR_1D_LAYOUT or DISPLAY_SPR_ACTIVE or 
    DISPLAY_BG0_ACTIVE or DISPLAY_BG1_ACTIVE );
	
	//vram banks are somewhat complex
	vramSetMainBanks(VRAM_A_LCD, VRAM_B_LCD, VRAM_C_SUB_BG, VRAM_D_SUB_SPRITE);
	
	// a vblank interrupt is needed to use swiWaitForVBlank()
	// since the dispatcher handles the flags no handler is required
	irqInit();
	irqSet(IRQ_VBLANK, @irqVBlank);
	
  //load our ball pcx file into an image
  loadPCX(pu8(ball_pcx), @ball);
	
  //tile it so it is usefull as sprite data
  imageTileData(@ball);

  // Sprite initialisation
  for i := 0 to 255 do
    SPRITE_PALETTE_SUB[i] := u32(ball.palette[i]);

  for i := 0 to 32*16 - 1 do
    SPRITE_GFX_SUB[i] := u32(ball.image.data16[i]);
	
	//turn off sprites
  initOAM();

  for i := 0 to NUM_SPRITES - 1 do
  begin
    //random place and speed
    sprites[i].x := rand() and $FFFF;
    sprites[i].y := rand() and $7FFF;
    sprites[i].dx := (rand() and $FF) + $100;
    sprites[i].dy := (rand() and $FF) + $100;
    
    if (rand() and 1) <> 0 then
      sprites[i].dx := -sprites[i].dx;
    if (rand() and 1) <> 0 then
      sprites[i].dy := -sprites[i].dy;
    
    sprites[i].oam := @OAMCopySub[i];
    sprites[i].gfxID := 0;
    
    //set up our sprites OAM entry attributes
    sprites[i].oam^.st.attribute[0] := ATTR0_COLOR_256 or ATTR0_SQUARE;  
    sprites[i].oam^.st.attribute[1] := ATTR1_SIZE_32;
    sprites[i].oam^.st.attribute[2] := sprites[i].gfxID;
  end;

	//set up two backgrounds to scroll around
	SUB_BG0_CR^ := BG_COLOR_256 or (1 shl SCREEN_SHIFT);
	SUB_BG1_CR^ := BG_COLOR_256 or (2 shl SCREEN_SHIFT);

	BG_PALETTE_SUB[0] := u32(RGB15(10,10,10));
	BG_PALETTE_SUB[1] := u32(RGB15(0,16,0));
	BG_PALETTE_SUB[2] := u32(RGB15(0,0,31));
	
	//load the maps with alternating tiles (0,1 for bg0 and 0,2 for bg1)
	for iy := 0 to 31 do
		for ix := 0 to 31 do
		begin
			map0[iy * 32 + ix] := (ix xor iy) and 1;
			map1[iy * 32 + ix] := ((ix xor iy) and 1) shl 1;
		end;    

	//fill 2 tiles with different colors
	for i := 0 to (64 div 2) - 1 do
	begin
		BG_GFX_SUB[i+32] := $0101;
		BG_GFX_SUB[i+32+32] := $0202;
	end;

	while (true) do
	begin
		//scroll the background
		SUB_BG0_X0^ := delta;
		inc(delta);
		SUB_BG0_Y0^ := delta;
		
		//move the sprites
		for i := 0 to NUM_SPRITES - 1 do
		begin
			sprites[i].x := sprites[i].x + sprites[i].dx;
			sprites[i].y := sprites[i].y + sprites[i].dy;
			
			//check for collision with the screen boundries
			if (sprites[i].x < (1 shl 8)) or (sprites[i].x > (247 shl 8)) then
				sprites[i].dx := -sprites[i].dx;

			if (sprites[i].y < (1 shl 8)) or (sprites[i].y > (182 shl 8)) then
				sprites[i].dy := -sprites[i].dy;
			
			//reposition the sprites
			MoveSprite(@sprites[i]);
		end;
		

	
		//do the plasma/fire
		for ix := 0 to SCREEN_WIDTH - 1 do
		begin
			back[ix + SCREEN_WIDTH * (SCREEN_HEIGHT - 1)] := rand() and $FFFF;
			back[ix + SCREEN_WIDTH * (SCREEN_HEIGHT - 2)] := rand() and $FFFF;
		end;

		inc(back);
		
		for iy := 1 to SCREEN_HEIGHT - 3 do
		begin
			for ix := 1 to SCREEN_WIDTH - 2 do
			begin
				red := 0;

				red := red + front[0];
				red := red + front[2];
	
				front := front + SCREEN_WIDTH;

				red := red + front[0];
				red := red + front[1];
				red := red + front[2];
				
				front := front + SCREEN_WIDTH;

				red := red + front[0];
				red := red + front[1];
				red := red + front[2];

				front := front - ((2 * SCREEN_WIDTH) - 1);	

				back[0] :=  (red shr 3);	
				inc(back);
			end;
			inc(back,2);
			inc(front,2);
		
		end;

		swiWaitForVBlank();
		
		updateOAM();

		//flip screens
		if (screen) <> 0 then
		begin
			videoSetMode(MODE_FB1);
			front := VRAM_B;
			back := VRAM_A;
			screen := 0;
		end else 
		begin
			videoSetMode(MODE_FB0);	
			front := VRAM_A;
			back := VRAM_B;
			screen := 1;
		end;
	end;    
end.

