<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef NDS_INTERFACE}
type
  bg_scroll = record
    x: cuint16;
    y: cuint16;
  end;
  Tbg_scroll = bg_scroll;
  Pbg_scroll = ^bg_scroll;

  bg_transform = record
    hdx: cint16;
    vdx: cint16;
    hdy: cint16;
    vdy: cint16;
    dx: cint32;
    dy: cint32;
  end;
  Tbg_transform = bg_transform;
  Pbg_transform = ^bg_transform;

  bg_attribute = record
    control: array [0..3] of cuint16;
    scroll: array [0..3] of bg_scroll;
    bg2_rotation: bg_transform;
    bg3_rotation: bg_transform;
  end;
  Tbg_attribute = bg_attribute;
  Pbg_attribute = ^bg_attribute;


const
  MAP_BASE_SHIFT = 8;
  TILE_BASE_SHIFT = 2;

function BG_TILE_BASE(base: cint): cint; inline; 
function BG_MAP_BASE(base: cint): cint; inline;  
function BG_BMP_BASE(base: cint): cint; inline;  
function BG_PRIORITY(n: cint): cint; inline;
function TILE_PALETTE(n: cint): cint; inline;

const
  TILE_FLIP_H = (1 shl 10);
  TILE_FLIP_V = (1 shl 11);


type
  TileMapEntry8 = record
    index: cuint8;
  end;
  TTileMapEntry8 = TileMapEntry8;
  PTileMapEntry8 = ^TileMapEntry8;

  TileMapEntry16 = bitpacked record
    index: 0..10;
    hflip: 0..1;
    vflip: 0..1;
    palette: 0..4;
  end;


type
  BackgroundControl = cint;
const
  BG_32x32: BackgroundControl = (0 shl 14);
  BG_64x32: BackgroundControl = (1 shl 14);
  BG_32x64: BackgroundControl = (2 shl 14);
  BG_64x64: BackgroundControl = (3 shl 14);
  
  BG_RS_16x16: BackgroundControl =  (0 shl 14);
  BG_RS_32x32: BackgroundControl =  (1 shl 14);
  BG_RS_64x64: BackgroundControl =  (2 shl 14);
  BG_RS_128x128: BackgroundControl =  (3 shl 14);
  
  BG_BMP8_128x128: BackgroundControl = ((0 shl 14) or (1 shl 7));
  BG_BMP8_256x256: BackgroundControl = ((1 shl 14) or (1 shl 7));
  BG_BMP8_512x256: BackgroundControl = ((2 shl 14) or (1 shl 7));
  BG_BMP8_512x512: BackgroundControl = ((3 shl 14) or (1 shl 7));
  BG_BMP8_1024x512: BackgroundControl = (1 shl 14);
  BG_BMP8_512x1024: BackgroundControl = 0;
  
  BG_BMP16_128x128: BackgroundControl = ((0 shl 14) or (1 shl 7) or (1 shl 2));
  BG_BMP16_256x256: BackgroundControl = ((1 shl 14) or (1 shl 7) or (1 shl 2));
  BG_BMP16_512x256: BackgroundControl = ((2 shl 14) or (1 shl 7) or (1 shl 2));
  BG_BMP16_512x512: BackgroundControl = ((3 shl 14) or (1 shl 7) or (1 shl 2));
  
  BG_MOSAIC_ON: BackgroundControl = ((1 shl 6));
  BG_MOSAIC_OFF: BackgroundControl = (0);
  
  BG_PRIORITY_0: BackgroundControl = (0);
  BG_PRIORITY_1: BackgroundControl = (1);
  BG_PRIORITY_2: BackgroundControl = (2);
  BG_PRIORITY_3: BackgroundControl = (3);
  
  BG_WRAP_OFF: BackgroundControl = (0);
  BG_WRAP_ON: BackgroundControl = (1 shl 13);
  
  BG_PALETTE_SLOT0: BackgroundControl = 0;
  BG_PALETTE_SLOT1: BackgroundControl = 0;
  BG_PALETTE_SLOT2: BackgroundControl = (1 shl 13);
  BG_PALETTE_SLOT3: BackgroundControl = (1 shl 13);
  
  BG_COLOR_256: BackgroundControl = $80;
  BG_COLOR_16: BackgroundControl = $00;

	BACKGROUND: Pbg_attribute = pointer($04000008);
  BG_OFFSET: Pbg_scroll = pointer($04000010);

function BG_MAP_RAM(base: cint): pcuint16; inline;  
function BG_TILE_RAM(base: cint): pcuint16; inline; 
function BG_BMP_RAM(base: cint): pcuint16; inline;  
function CHAR_BASE_BLOCK(n: cint): pcuint16; inline;
function SCREEN_BASE_BLOCK(n: cint): pcuint16; inline;

const
  BGCTRL: pcuint16 = pointer($04000008);
  REG_BG0CNT		: pcuint16 = pointer($4000008);
  REG_BG1CNT		: pcuint16 = pointer($400000A);
  REG_BG2CNT		: pcuint16 = pointer($400000C);
  REG_BG3CNT		: pcuint16 = pointer($400000E);
  
  REG_BGOFFSETS	: pcuint16 = pointer($4000010);
  REG_BG0HOFS		: pcuint16 = pointer($4000010);
  REG_BG0VOFS		: pcuint16 = pointer($4000012);
  REG_BG1HOFS		: pcuint16 = pointer($4000014);
  REG_BG1VOFS		: pcuint16 = pointer($4000016);
  REG_BG2HOFS		: pcuint16 = pointer($4000018);
  REG_BG2VOFS		: pcuint16 = pointer($400001A);
  REG_BG3HOFS		: pcuint16 = pointer($400001C);
  REG_BG3VOFS		: pcuint16 = pointer($400001E);
  
  REG_BG2PA		: pcint16 = pointer($4000020);
  REG_BG2PB		: pcint16 = pointer($4000022);
  REG_BG2PC		: pcint16 = pointer($4000024);
  REG_BG2PD		: pcint16 = pointer($4000026);
  
  REG_BG2X		: pcint32 = pointer($4000028);
  REG_BG2Y		: pcint32 = pointer($400002C);
  
  REG_BG3PA		: pcint16 = pointer($4000030);
  REG_BG3PB		: pcint16 = pointer($4000032);
  REG_BG3PC		: pcint16 = pointer($4000034);
  REG_BG3PD		: pcint16 = pointer($4000036);
  
  REG_BG3X		: pcint32 = pointer($4000038);
  REG_BG3Y		: pcint32 = pointer($400003C);

	BACKGROUND_SUB : Pbg_attribute = pointer($04001008);
  BG_OFFSET_SUB: Pbg_scroll = pointer($04001010);

function BG_MAP_RAM_SUB(base: cint): pcuint16; inline; 
function BG_TILE_RAM_SUB(base: cint): pcuint16; inline; 
function BG_BMP_RAM_SUB(base: cint): pcuint16; inline; 
function SCREEN_BASE_BLOCK_SUB(n: cint): pcuint16; inline;
function CHAR_BASE_BLOCK_SUB(n: cint): pcuint16; inline;


const
  BGCTRL_SUB		: pcuint16 = pointer($4001008);
  REG_BG0CNT_SUB		: pcuint16 = pointer($4001008);
  REG_BG1CNT_SUB		: pcuint16 = pointer($400100A);
  REG_BG2CNT_SUB		: pcuint16 = pointer($400100C);
  REG_BG3CNT_SUB		: pcuint16 = pointer($400100E);

  REG_BGOFFSETS_SUB	: pcuint16 = pointer($4001010);
  REG_BG0HOFS_SUB		: pcuint16 = pointer($4001010);
  REG_BG0VOFS_SUB		: pcuint16 = pointer($4001012);
  REG_BG1HOFS_SUB		: pcuint16 = pointer($4001014);
  REG_BG1VOFS_SUB		: pcuint16 = pointer($4001016);
  REG_BG2HOFS_SUB		: pcuint16 = pointer($4001018);
  REG_BG2VOFS_SUB		: pcuint16 = pointer($400101A);
  REG_BG3HOFS_SUB		: pcuint16 = pointer($400101C);
  REG_BG3VOFS_SUB		: pcuint16 = pointer($400101E);
  
  REG_BG2PA_SUB		: pcint16 = pointer($4001020);
  REG_BG2PB_SUB		: pcint16 = pointer($4001022);
  REG_BG2PC_SUB		: pcint16 = pointer($4001024);
  REG_BG2PD_SUB		: pcint16 = pointer($4001026);
  
  REG_BG2X_SUB		: pcint32 = pointer($4001028);
  REG_BG2Y_SUB		: pcint32 = pointer($400102C);
  
  REG_BG3PA_SUB		: pcint16 = pointer($4001030);
  REG_BG3PB_SUB		: pcint16 = pointer($4001032);
  REG_BG3PC_SUB		: pcint16 = pointer($4001034);
  REG_BG3PD_SUB		: pcint16 = pointer($4001036);
  
  REG_BG3X_SUB		: pcint32 = pointer($4001038);
  REG_BG3Y_SUB		: pcint32 = pointer($400103C);
  
type
  //background state
  TBgState = record
    angle: cint;
    centerX, centerY: cint32;
    scaleX, scaleY: cint32;
    scrollX, scrollY: cint32;
    size: cint;
    _type: cint;
    dirty: cbool;
  end;
  PBgState = ^TBgState;
  
var
  bgControl: array [0..7] of pcuint16; cvar; external;
  bgScrollTable: array [0..7] of Pbg_scroll; cvar; external;
  bgTransform: array [0..7] of Pbg_transform; cvar; external;
  bgState: array [0..7] of TBgState; cvar; external;


type
  BgType = integer;
const
  BgType_Text8bpp   = 0;  
  BgType_Text4bpp   = 1; 
  BgType_Rotation   = 2;
  BgType_ExRotation = 3;
  BgType_Bmp8       = 4;
  BgType_Bmp16      = 5;

type
  BgSize = integer;
const
  BgSize_R_128x128    : BgSize = (0 shl 14);
  BgSize_R_256x256    : BgSize = (1 shl 14);
  BgSize_R_512x512    : BgSize = (2 shl 14);
  BgSize_R_1024x1024  : BgSize = (3 shl 14);	 	         
  BgSize_T_256x256    : BgSize = (0 shl 14) or (1 shl 16);	         
  BgSize_T_512x256    : BgSize = (1 shl 14) or (1 shl 16);	        
  BgSize_T_256x512    : BgSize = (2 shl 14) or (1 shl 16);	         
  BgSize_T_512x512    : BgSize = (3 shl 14) or (1 shl 16);	 	         
  BgSize_ER_128x128   : BgSize = (0 shl 14) or (2 shl 16);	         
  BgSize_ER_256x256   : BgSize = (1 shl 14) or (2 shl 16);	         
  BgSize_ER_512x512   : BgSize = (2 shl 14) or (2 shl 16);	         
  BgSize_ER_1024x1024 : BgSize = (3 shl 14) or (2 shl 16);	 	         
  BgSize_B8_128x128   : BgSize = (0 shl 14) or (1 shl 7) or (3 shl 16);	         
  BgSize_B8_256x256   : BgSize = (1 shl 14) or (1 shl 7) or (3 shl 16);	         
  BgSize_B8_512x256   : BgSize = (2 shl 14) or (1 shl 7) or (3 shl 16);	         
  BgSize_B8_512x512   : BgSize = (3 shl 14) or (1 shl 7) or (3 shl 16);	         
  BgSize_B8_1024x512  : BgSize = (1 shl 14) or (3 shl 16);	         
  BgSize_B8_512x1024  : BgSize = (0 or (3 shl 16));	 	         
  BgSize_B16_128x128  : BgSize = (0 shl 14) or (1 shl 7) or (1 shl 2) or (4 shl 16);	         
  BgSize_B16_256x256  : BgSize = (1 shl 14) or (1 shl 7) or (1 shl 2) or (4 shl 16);	         
  BgSize_B16_512x256  : BgSize = (2 shl 14) or (1 shl 7) or (1 shl 2) or (4 shl 16);	         
  BgSize_B16_512x512  : BgSize = (3 shl 14) or (1 shl 7) or (1 shl 2) or (4 shl 16);

function bgIsText(id: cint): cbool;	cdecl; external; 
function bgInit_call(layer: cint; atype: BgType; size: BgSize; mapBase, tileBase: cint): cint;	cdecl; external; 
function bgInitSub_call(layer: cint; atype: BgType; size: BgSize; mapBase, tileBase: cint): cint;	cdecl; external;

procedure bgUpdate(); cdecl; external;

procedure bgSetRotate(id, angle: cint); inline;
procedure bgRotate(id, angle: cint); inline;
procedure bgSet(id, angle: cint; sx, sy, scrollX, scrollY, rotCenterX, rotCenterY: cint32); inline;
procedure bgSetRotateScale(id, angle: cint; sx, sy: cint32); inline;
procedure bgSetScale(id: cint; sx, sy: cint32); inline;
function bgInit(layer: cint; atype: BgType; size: BgSize; mapBase, tileBase: cint): cint; inline;
function bgInitSub(layer: cint; atype: BgType; size: BgSize; mapBase, tileBase: cint): cint; inline;
function bgSetControlBits(id: cint; bits: cuint16): pcuint16; inline;
procedure bgClearControlBits(id: cint; bits: cuint16); inline;
procedure bgWrapOn(id: cint); inline;
procedure bgWrapOff(id: cint); inline;
procedure bgSetPriority(id: cint; priority: cuint); inline;
procedure bgSetMapBase(id: cint; base: cuint); inline;
procedure bgSetTileBase(id: cint; base: cuint); inline;	
procedure bgSetScrollf(id: cint; x, y: cint32); inline;
procedure bgSetScroll(id, x, y: cint); inline;
procedure bgMosaicEnable(id: cint); inline;
procedure bgMosaicDisable(id: cint); inline;
procedure bgSetMosaic(dx, dy: cuint); inline;
procedure bgSetMosaicSub(dx, dy: cuint); inline;
function bgGetPriority(id: cint): cint; inline;
function bgGetMapBase(id: cint): cint; inline;
function bgGetTileBase(id: cint): cint; inline;
function bgGetMapPtr(id: cint): pcuint16; inline;
function bgGetGfxPtr(id: cint): pcuint16; inline;
procedure bgScrollf(id: cint; dx, dy: cint32); inline;
procedure bgScroll(id, dx, dy: cint); inline;
procedure bgShow(id: cint); inline;
procedure bgHide(id: cint); inline;
procedure bgSetCenterf(id: cint; x, y: cint32); inline;
procedure bgSetCenter(id, x, y: cint); inline;
procedure bgSetAffineMatrixScroll(id, hdx, vdx, hdy, vdy, scrollx, scrolly: cint); inline;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure bgExtPaletteEnable(); inline; 
procedure bgExtPaletteEnableSub(); inline; 
procedure bgExtPaletteDisable(); inline; 
procedure bgExtPaletteDisableSub(); inline; 
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}
function BG_TILE_BASE(base: cint): cint; inline; 
begin 
  BG_TILE_BASE := ((base) shl 2);
end;

function BG_MAP_BASE(base: cint): cint; inline;  
begin
  BG_MAP_BASE := ((base) shl 8);
end;

function BG_BMP_BASE(base: cint): cint; inline;  
begin
  BG_BMP_BASE := ((base) shl 8);
end;

function BG_PRIORITY(n: cint): cint; inline;
begin
  BG_PRIORITY := (n);
end;

function TILE_PALETTE(n: cint): cint; inline;
begin
  TILE_PALETTE := ((n) shl 12);
end;

function BG_MAP_RAM(base: cint): pcuint16; inline;  
begin
  BG_MAP_RAM := pcuint16(((base) * $800) + $06000000);
end;

function BG_MAP_RAM_SUB(base: cint): pcuint16; inline; 
begin
  BG_MAP_RAM_SUB := pcuint16(((base) * $800) + $06200000);
end;


function BG_TILE_RAM(base: cint): pcuint16; inline; 
begin
  BG_TILE_RAM := pcuint16(((base) * $4000) + $06000000);
end;

function BG_TILE_RAM_SUB(base: cint): pcuint16; inline; 
begin
  BG_TILE_RAM_SUB := pcuint16(((base) * $4000) + $06200000);
end;


function BG_BMP_RAM(base: cint): pcuint16; inline;  
begin
  BG_BMP_RAM := pcuint16(((base) * $4000) + $06000000);
end;

function BG_BMP_RAM_SUB(base: cint): pcuint16; inline; 
begin
  BG_BMP_RAM_SUB := pcuint16(((base) * $4000) + $06200000);
end;

function CHAR_BASE_BLOCK(n: cint): pcuint16; inline;
begin
  CHAR_BASE_BLOCK := pcuint16((n * $4000)+ $06000000);
end;

function CHAR_BASE_BLOCK_SUB(n: cint): pcuint16; inline;
begin		
  CHAR_BASE_BLOCK_SUB := pcuint16((n * $4000)+ $06200000);
end;

function SCREEN_BASE_BLOCK(n: cint): pcuint16; inline;
begin		
  SCREEN_BASE_BLOCK := pcuint16((n * $800) + $06000000);
end;

function SCREEN_BASE_BLOCK_SUB(n: cint): pcuint16; inline;
begin	
  SCREEN_BASE_BLOCK_SUB := pcuint16((n * $800) + $06200000);
end;

procedure bgSetRotate(id, angle: cint); inline;
begin
  bgState[id].angle := angle;
  bgState[id].dirty := true;
end;

procedure bgRotate(id, angle: cint); inline;
begin
  sassert(not bgIsText(id), 'Cannot Rotate a Text Background');
  bgSetRotate(id, angle + bgState[id].angle);
end;

procedure bgSet(id, angle: cint; sx, sy, scrollX, scrollY, rotCenterX, rotCenterY: cint32); inline;
begin
  bgState[id].scaleX := sx;
  bgState[id].scaleY := sy;
  bgState[id].scrollX := scrollX;
  bgState[id].scrollY := scrollY;
  bgState[id].centerX := rotCenterX;
  bgState[id].centerY := rotCenterY;
  bgState[id].angle := angle;
  bgState[id].dirty := true;	 
end;

procedure bgSetRotateScale(id, angle: cint; sx, sy: cint32); inline;
begin
  bgState[id].scaleX := sx;
  bgState[id].scaleY := sy;
  bgState[id].angle := angle;
  bgState[id].dirty := true;
end;

procedure bgSetScale(id: cint; sx, sy: cint32); inline;
begin 
  sassert( not bgIsText(id), 'Cannot Scale a Text Background');
  bgState[id].scaleX := sx;
  bgState[id].scaleY := sy;
  bgState[id].dirty := true;
end;

function bgInit(layer: cint; atype: BgType; size: BgSize; mapBase, tileBase: cint): cint; inline;
begin
  sassert( (layer >= 0) and (layer <= 3), 'Only layers 0 - 3 are supported');
  sassert( (tileBase >= 0) and (tileBase <= 15), 'Background tile base is out of range');
  sassert( (mapBase >=0) and (mapBase <= 31), 'Background Map Base is out of range');
  sassert( (layer <> 0) or not video3DEnabled(), 'Background 0 is currently being used for 3D display');
  sassert( (layer > 1) or (atype = BgType_Text8bpp) or (atype = BgType_Text4bpp), 'Incorrect background type for mode');
  //sassert((size != BgSize_B8_512x1024 && size != BgSize_B8_1024x512) || videoGetMode() == 6, "Incorrect background type for mode");
  sassert( (tileBase = 0) or (atype < BgType_Bmp8), 'Tile base is unused for bitmaps.  Can be offset using mapBase * 16KB');
  sassert( ((mapBase = 0) or (atype <> BgType_Bmp8)) or ((size <> BgSize_B8_512x1024) and (size <> BgSize_B8_1024x512)), 'Large Bitmaps cannot be offset');
  result := bgInit_call(layer, atype, size, mapBase, tileBase);	 
end;

function bgInitSub(layer: cint; atype: BgType; size: BgSize; mapBase, tileBase: cint): cint; inline;
begin
  sassert( (layer >= 0) and (layer <= 3), 'Only layers 0 - 3 are supported');
  sassert( (tileBase >= 0) and (tileBase <= 15), 'Background tile base is out of range');
  sassert( (mapBase >=0) and (mapBase <= 31), 'Background Map Base is out of range');
  sassert( (layer > 1) or (atype = BgType_Text8bpp) or (atype = BgType_Text4bpp), 'Incorrect background type for mode');
  sassert( ((size <> BgSize_B8_512x1024) and (size <> BgSize_B8_1024x512)), 'Sub Display has no large Bitmaps');
  result := bgInitSub_call(layer, atype, size, mapBase, tileBase);
end;

function bgSetControlBits(id: cint; bits: cuint16): pcuint16; inline;
begin
	sassert( (id >= 0) and (id <= 7), 'bgSetControlBits(), id must be the number returned from bgInit or bgInitSub');
  bgControl[id]^ := bgControl[id]^ or bits;
  result := bgControl[id];
end;  
  
procedure bgClearControlBits(id: cint; bits: cuint16); inline;
begin
  bgControl[id]^ := bgControl[id]^ and not bits;	 
end;  

procedure bgWrapOn(id: cint); inline;
begin
  bgSetControlBits(id, BIT(13));
end;

procedure bgWrapOff(id: cint); inline;
begin
  bgClearControlBits(id, BIT(13));
end;

procedure bgSetPriority(id: cint; priority: cuint); inline;
begin
  sassert( (priority < 4), 'Priority must be less than 4');
  bgControl[id]^ := bgControl[id]^ and not 3;
  bgControl[id]^ := bgControl[id]^ or priority;	 	 
end;

procedure bgSetMapBase(id: cint; base: cuint); inline;
begin
  sassert( (base <= 31), 'Map base cannot exceed 31');
  bgControl[id]^ := bgControl[id]^ and not (31 shl MAP_BASE_SHIFT);
  bgControl[id]^ := bgControl[id]^ or (base shl MAP_BASE_SHIFT);
end;
    
procedure bgSetTileBase(id: cint; base: cuint); inline;	
begin
  sassert( (base <= 15), 'Tile base cannot exceed 15');
  bgControl[id]^ := bgControl[id]^ and not (15 shl TILE_BASE_SHIFT);
  bgControl[id]^ := bgControl[id]^ or (base shl TILE_BASE_SHIFT);
end;

procedure bgSetScrollf(id: cint; x, y: cint32); inline;
begin
  bgState[id].scrollX := x;
  bgState[id].scrollY := y;
  bgState[id].dirty := true;
end;

procedure bgSetScroll(id, x, y: cint); inline;
begin
  bgSetScrollf(id, x shl 8, y shl 8);
end;

procedure bgMosaicEnable(id: cint); inline;
begin
  bgControl[id]^ := bgControl[id]^ or BIT(6);
end;

procedure bgMosaicDisable(id: cint); inline;
begin
  bgControl[id]^ := bgControl[id]^ and not BIT(6);
end;

procedure bgSetMosaic(dx, dy: cuint); inline;
begin
  sassert( (dx < 16) and (dy < 16), 'Mosaic range is 0 to 15');
  mosaicShadow := (mosaicShadow and $ff00) or (dx or (dy shl 4));
  REG_MOSAIC^ := mosaicShadow;
end;

procedure bgSetMosaicSub(dx, dy: cuint); inline;
begin
  sassert( (dx < 16) and (dy < 16), 'Mosaic range is 0 to 15');
  mosaicShadowSub := (mosaicShadowSub and $ff00) or (dx or (dy shl 4));
  REG_MOSAIC_SUB^ := mosaicShadowSub;
end;



function bgGetPriority(id: cint): cint; inline;
begin
  result := bgControl[id]^ and 3;
end;

function bgGetMapBase(id: cint): cint; inline;
begin
  result := (bgControl[id]^ shr MAP_BASE_SHIFT) and 31;
end;


function bgGetTileBase(id: cint): cint; inline;
begin
  result := (bgControl[id]^ shr TILE_BASE_SHIFT) and 15;
end;

function bgGetMapPtr(id: cint): pcuint16; inline;
begin
  if (id < 4) then 
    result := pcuint16(BG_MAP_RAM(bgGetMapBase(id))) 
  else
    result := pcuint16(BG_MAP_RAM_SUB(bgGetMapBase(id)));
end;

function bgGetGfxPtr(id: cint): pcuint16; inline;
begin
  if(bgState[id]._type < integer(BgType_Bmp8)) then 
  begin
    if (id < 4) then 
      result := pcuint16(BG_TILE_RAM(bgGetTileBase(id))) 
    else 
      result := pcuint16(BG_TILE_RAM_SUB(bgGetTileBase(id)));	         
  end else	                 
  begin
    if (id < 4) then
      result := pointer(BG_GFX + $2000 * (bgGetMapBase(id))) 
    else 
      result := pointer(BG_GFX_SUB + $2000 * (bgGetMapBase(id)));
  end;
end;

procedure bgScrollf(id: cint; dx, dy: cint32); inline;
begin
  bgSetScrollf(id, bgState[id].scrollX + dx, bgState[id].scrollY + dy);
end;

procedure bgScroll(id, dx, dy: cint); inline;
begin
  bgScrollf(id, dx shl 8, dy shl 8);
end;

procedure bgShow(id: cint); inline;
begin
  if (id < 4) then
    videoBgEnable(id)
  else
    videoBgEnableSub(id and 3);
end;

procedure bgHide(id: cint); inline;
begin
  if (id < 4) then
    videoBgDisable(id)
  else
    videoBgDisableSub(id and 3);
end;

procedure bgSetCenterf(id: cint; x, y: cint32); inline;
begin
  sassert(not bgIsText(id), 'Text Backgrounds have no Center of Rotation');
  bgState[id].centerX := x;
  bgState[id].centerY := y;
  bgState[id].dirty := true;
end;

procedure bgSetCenter(id, x, y: cint); inline;
begin
  bgSetCenterf(id, x shl 8, y shl 8);
end;

procedure bgSetAffineMatrixScroll(id, hdx, vdx, hdy, vdy, scrollx, scrolly: cint); inline;
begin
  sassert(not bgIsText(id), 'Text Backgrounds have no affine matrix and scroll registers.');

  bgTransform[id]^.hdx := hdx;
  bgTransform[id]^.vdx := vdx;
  bgTransform[id]^.hdy := hdy;
  bgTransform[id]^.vdy := vdy;
  
  bgTransform[id]^.dx := scrollx;
  bgTransform[id]^.dy := scrolly;
  
  bgState[id].dirty := false;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure bgExtPaletteEnable(); inline; 
begin 
  REG_DISPCNT^ := REG_DISPCNT^ or DISPLAY_BG_EXT_PALETTE; 
end;

procedure bgExtPaletteEnableSub(); inline; 
begin 
  REG_DISPCNT_SUB^ := REG_DISPCNT_SUB^ or DISPLAY_BG_EXT_PALETTE; 
end;

procedure bgExtPaletteDisable(); inline; 
begin 
  REG_DISPCNT^ := REG_DISPCNT^ and not DISPLAY_BG_EXT_PALETTE; 
end;

procedure bgExtPaletteDisableSub(); inline; 
begin 
  REG_DISPCNT_SUB^ := REG_DISPCNT_SUB^ and not DISPLAY_BG_EXT_PALETTE; 
end;
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew

{$endif NDS_IMPLEMENTATION}
=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: background.inc 25 2007-12-10 21:06:46Z p4p3r0 $
  ------------------------------------------------------------------------------
  	Copyright (C) 2005
  		Jason Rogers (dovoto)
  		Dave Murphy (WinterMute)
  	
  	This software is provided 'as-is', without any express or implied
  	warranty.  In no event will the authors be held liable for any
  	damages arising from the use of this software.
  	
  	Permission is granted to anyone to use this software for any
  	purpose, including commercial applications, and to alter it and
  	redistribute it freely, subject to the following restrictions:
  	
  	1.	The origin of this software must not be misrepresented; you
  		must not claim that you wrote the original software. If you use
  		this software in a product, an acknowledgment in the product
  		documentation would be appreciated but is not required.
  	
  	2.	Altered source versions must be plainly marked as such, and
  		must not be misrepresented as being the original software.
  	
  	3.	This notice may not be removed or altered from any source
  		distribution.
  ------------------------------------------------------------------------------
    
    
  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)
   
  Copyright (C) 2006  Francesco Lombardi
  Check http://sourceforge.net/projects/libndsfpc for updates
   
  ------------------------------------------------------------------------------

  $Log$

*)


// Background control defines

{$ifdef NDS_INTERFACE}
// BGxCNT defines ///
const
  BG_MOSAIC_ENABLE	= $40;
  BG_COLOR_256		  = $80;
  BG_COLOR_16			  = $00;
{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}
function CHAR_BASE_BLOCK(n: cint): cint; inline;
begin
  CHAR_BASE_BLOCK := (n * $4000)+ $06000000;
end;

function CHAR_BASE_BLOCK_SUB(n: cint): cint; inline;
begin		
  CHAR_BASE_BLOCK_SUB := (n * $4000)+ $06200000;
end;

function SCREEN_BASE_BLOCK(n: cint): cint; inline;
begin		
  SCREEN_BASE_BLOCK := (n * $800) + $06000000;
end;

function SCREEN_BASE_BLOCK_SUB(n: cint): cint; inline;
begin	
  SCREEN_BASE_BLOCK_SUB := (n * $800) + $06200000;
end;
{$endif NDS_IMPLEMENTATION}

{$ifdef NDS_INTERFACE}
const
  CHAR_SHIFT           = 2;
  SCREEN_SHIFT         = 8;
  TEXTBG_SIZE_256x256  = $0;
  TEXTBG_SIZE_256x512  = $8000;
  TEXTBG_SIZE_512x256  = $4000;
  TEXTBG_SIZE_512x512  = $C000;

  ROTBG_SIZE_128x128   = $0;
  ROTBG_SIZE_256x256   = $4000;
  ROTBG_SIZE_512x512   = $8000;
  ROTBG_SIZE_1024x1024 = $C000;

  WRAPAROUND           = $1;
{$endif NDS_INTERFACE}

{$ifdef NDS_INTERFACE}
function CHAR_BASE_BLOCK(n: cint): cint; inline;
function CHAR_BASE_BLOCK_SUB(n: cint): cint; inline;
function SCREEN_BASE_BLOCK(n: cint): cint; inline;
function SCREEN_BASE_BLOCK_SUB(n: cint): cint; inline;
{$endif NDS_INTERFACE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
