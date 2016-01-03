<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: sprite.inc 25 2007-12-10 21:06:46Z p4p3r0 $
  ------------------------------------------------------------------------------
  	sprite.h -- definitions for DS sprites
  
  	Copyright (C) 2007
  		Liran Nuna	(LiraNuna)
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

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifndef ARM9}
  {$error Sprites are only available on the ARM9}
{$endif ARM9}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef NDS_INTERFACE}

=======
=======
>>>>>>> origin/fixes_2_2
// Sprite control defines

{$ifdef NDS_INTERFACE}
// Attribute 0 consists of 8 bits of Y plus the following flags:
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
const
  ATTR0_NORMAL          = (0 shl 8);
  ATTR0_ROTSCALE        =	(1 shl 8);
  ATTR0_DISABLED        =	(2 shl 8);
  ATTR0_ROTSCALE_DOUBLE	= (3 shl 8);

  ATTR0_TYPE_NORMAL     = (0 shl 10);
  ATTR0_TYPE_BLENDED    =	(1 shl 10);
  ATTR0_TYPE_WINDOWED   = (2 shl 10);
  ATTR0_BMP             =	(3 shl 10);

  ATTR0_MOSAIC          = (1 shl 12);

  ATTR0_COLOR_16        = (0 shl 13); //16 color in tile mode...16 bit in bitmap mode
  ATTR0_COLOR_256       = (1 shl 13);

  ATTR0_SQUARE          = (0 shl 14);
  ATTR0_WIDE            = (1 shl 14);
  ATTR0_TALL            = (2 shl 14);
<<<<<<< HEAD
<<<<<<< HEAD

function OBJ_Y(m: cint): cint; inline;

function ATTR1_ROTDATA(n: cint): cint; inline;

=======
=======
>>>>>>> origin/fixes_2_2
{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}
function OBJ_Y(m: cint): cint; inline;
begin
  OBJ_Y := ((m) and $00ff);
end;

function ATTR1_ROTDATA(n: cint): cint; inline;
begin
  ATTR1_ROTDATA := ((n) shl 9)  // note: overlaps with flip flags
end;
{$endif NDS_IMPLEMENTATION}

{$ifdef NDS_INTERFACE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
const
  ATTR1_FLIP_X          = (1 shl 12);
  ATTR1_FLIP_Y          = (1 shl 13);
  ATTR1_SIZE_8          = (0 shl 14);
  ATTR1_SIZE_16         = (1 shl 14);
  ATTR1_SIZE_32         = (2 shl 14);
  ATTR1_SIZE_64         = (3 shl 14);
<<<<<<< HEAD
<<<<<<< HEAD

function OBJ_X(m: cint): cint; inline;

function ATTR2_PRIORITY(n: cint): cint; inline;     
function ATTR2_PALETTE(n: cint): cint; inline;      
function ATTR2_ALPHA(n: cint): cint; inline;		  

type 
  ObjBlendMode = (
    OBJMODE_NORMAL   = 0, 
    OBJMODE_BLENDED  = 1,
    OBJMODE_WINDOWED = 2,
    OBJMODE_BITMAP   = 3
  );

  ObjShape = (
    OBJSHAPE_SQUARE     = 0,
    OBJSHAPE_WIDE       = 1,
    OBJSHAPE_TALL       = 2,
    OBJSHAPE_FORBIDDEN  = 3
  );
  
  ObjSize = (
    OBJSIZE_8   = 0,
    OBJSIZE_16  = 1,
    OBJSIZE_32  = 2, 
    OBJSIZE_64  = 3
  );
  
  ObjColMode = (
    OBJCOLOR_16  = 0, 
    OBJCOLOR_256 = 1
  );

  ObjPriority = (
    OBJPRIORITY_0 = 0,
    OBJPRIORITY_1 = 1,
    OBJPRIORITY_2 = 2,
    OBJPRIORITY_3 = 3
  );

type
  // mimic gcc bitfields behaviour
  Unsigned_1  = 0..((1 shl 1)  - 1);
  Unsigned_2  = 0..((1 shl 2)  - 1);
  Unsigned_3  = 0..((1 shl 3)  - 1);
  Unsigned_4  = 0..((1 shl 4)  - 1);
  Unsigned_5  = 0..((1 shl 5)  - 1);
  Unsigned_6  = 0..((1 shl 6)  - 1);
  Unsigned_7  = 0..((1 shl 7)  - 1);
  Unsigned_8  = 0..((1 shl 8)  - 1);
  Unsigned_9  = 0..((1 shl 9)  - 1);
  Unsigned_10 = 0..((1 shl 10) - 1);

  sSpriteEntry = bitpacked record
    case integer of
      0: (
        y: Unsigned_8;//0..8;
        case integer of 
          0: (
  				  __padding1: Unsigned_1;//0..1;
  				  isHidden: Unsigned_1;//0..1;
  				  __padding2: Unsigned_6;//0..6;          
          );
          1: (
            isRotateScale: Unsigned_1;//0..1;
            isSizeDouble: Unsigned_1;//0..1;
            blendMode: Unsigned_2;//0..2;
            isMosaic: Unsigned_1;//0..1;
            colorMode: Unsigned_1;//0..1;
            shape: Unsigned_2;//0..2;          
//          );

            case integer of
              0: (
        			  x: Unsigned_9;//0..9;
                _padding3: Unsigned_7;//0..7;
              );
              1: (
                __padding4: Unsigned_8;//0..8;
                case integer of
                  0: (
    						    __padding5: Unsigned_4;//0..4;
    						    hFlip: Unsigned_1;//0..1;
    						    vFlip: Unsigned_1;//0..1;
    						    __padding6: Unsigned_2;//0..2;
                  );
                  1: (
    						    __padding7: Unsigned_1;//0..1;
    						    rotationIndex: Unsigned_5;//0..5;
    						    size: Unsigned_2;//0..2;
    //              );
    //          );
     
                    gfxIndex: Unsigned_10;//0..10;
                    priority: Unsigned_2;//0..2;
                    palette: Unsigned_4;//0..4;
                    
                    attribute3: cuint16;
                  );
              );     
          );
      ); 
      1: (
        attribute: array [0..2] of cuint16;
        filler: cuint16;   
      );
=======
=======
>>>>>>> origin/fixes_2_2
{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}
function OBJ_X(m: cint): cint; inline;
begin
  OBJ_X := ((m) and $01ff);
end;

// Atribute 2 consists of the following:
function ATTR2_PRIORITY(n: cint): cint; inline;     
begin
  ATTR2_PRIORITY := ((n) shl 10);
end;

function ATTR2_PALETTE(n: cint): cint; inline;      
begin
  ATTR2_PALETTE := ((n) shl 12);
end;

function ATTR2_ALPHA(n: cint): cint; inline;		  
begin
  ATTR2_ALPHA := ((n) shl 12);
end;
{$endif NDS_IMPLEMENTATION}

{$ifdef NDS_INTERFACE}
type 
  TObjMode = (OBJMODE_NORMAL, OBJMODE_BLENDED, OBJMODE_WINDOWED, OBJMODE_BITMAP);
  TObjShape = (OBJSHAPE_SQUARE, OBJSHAPE_WIDE, OBJSHAPE_TALL, OBJSHAPE_FORBIDDEN);
  TObjSize = (OBJSIZE_8, OBJSIZE_16, OBJSIZE_32, OBJSIZE_64);
  TObjColMode = (OBJCOLOR_16, OBJCOLOR_256);
  TObjPriority = (OBJPRIORITY_0, OBJPRIORITY_1, OBJPRIORITY_2, OBJPRIORITY_3);

  sSpriteEntry = packed record
    case integer of
      0: (st1:  packed record
                  st1:  bitpacked record
                          posY: 0..8;
                          case integer of
                            0: (st1:  bitpacked record
                                        __padding1: 0..1;
                                        isHidden: 0..1;
                                        __padding2: 0..6;
                                      end; 
                                );
                            1: (st2:  bitpacked record
                                        isRotoscale: 0..1;
                                        rsDouble: 0..1;
                                        objMode: 0..2;
                                        isMosaic: 0..1;
                                        colMode: 0..1;
                                        objShape: 0..2;
                                      end;
                               );  
                        end;
                  st2:  bitpacked record
                          tileIdx: 0..10;
                          objPriority: 0..2;
                          objPal: 0..4;
                        end; 
                  attribute3: cuint16;
                  case integer of
                    0: (st1:  bitpacked record
                                posX: 0..9;
                                __padding1: 0..7;
                              end; 
                       );
                    1: (st2:  bitpacked record
                                __padding1: 0..8;
                                case integer of
                                  0: (st1:  bitpacked record
                                              __padding1: 0..4;
                                              hFlip: 0..1;
                                              vFlip: 0..1;
                                              __padding2: 0..2;
                                            end;
                                     );
                                  1: (st2:  bitpacked record
                                              __padding1: 0..1;
                                              rsMatrixIdx: 0..5;
                                              objSize: 0..2;
                                            end;
                                     );
                              end;
                       ); 
                end;
         ); 
      1: (st:  packed record
                  attribute: array [0..2] of cuint16;
                  filler: cuint16;    
               end;
        );
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
  SpriteEntry = sSpriteEntry;
  TSpriteEntry = sSpriteEntry;
  PSpriteEntry = ^sSpriteEntry;

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
  sSpriteRotation = record
    filler1: array [0..2] of cuint16;
    hdx: cint16;
    filler2: array [0..2] of cuint16;
<<<<<<< HEAD
<<<<<<< HEAD
    vdx: cint16;
    filler3: array [0..2] of cuint16;
    hdy: cint16;
=======
    hdy: cint16;
    filler3: array [0..2] of cuint16;
    vdx: cint16;
>>>>>>> graemeg/fixes_2_2
=======
    hdy: cint16;
    filler3: array [0..2] of cuint16;
    vdx: cint16;
>>>>>>> origin/fixes_2_2
    filler4: array [0..2] of cuint16;
    vdy: cint16;
  end;
  SpriteRotation = sSpriteRotation;
  TSpriteRotation = sSpriteRotation;
  pSpriteRotation = ^sSpriteRotation;


const
  SPRITE_COUNT = 128;
  MATRIX_COUNT = 32;

type
<<<<<<< HEAD
<<<<<<< HEAD
  OAMTable = record
    case integer of
      0: (
            oamBuffer: array [0..SPRITE_COUNT - 1] of TSpriteEntry;
      );
      1: (
            matrixBuffer: array [0..MATRIX_COUNT - 1] of TSpriteRotation;
      );
  end;
  TOAMTable = OAMTable;
  POAMTable = ^OAMTable;

type  
  SpriteSize = integer; 
const
  SpriteSize_8x8  : SpriteSize = (integer(OBJSIZE_8)  shl 14) or (integer(OBJSHAPE_SQUARE) shl 12) or ((8*8)   shr 5);
  SpriteSize_16x16: SpriteSize = (integer(OBJSIZE_16) shl 14) or (integer(OBJSHAPE_SQUARE) shl 12) or ((16*16) shr 5);
  SpriteSize_32x32: SpriteSize = (integer(OBJSIZE_32) shl 14) or (integer(OBJSHAPE_SQUARE) shl 12) or ((32*32) shr 5);
  SpriteSize_64x64: SpriteSize = (integer(OBJSIZE_64) shl 14) or (integer(OBJSHAPE_SQUARE) shl 12) or ((64*64) shr 5);
  SpriteSize_16x8 : SpriteSize = (integer(OBJSIZE_8)  shl 14) or (integer(OBJSHAPE_WIDE)   shl 12) or ((16*8)  shr 5);
  SpriteSize_32x8 : SpriteSize = (integer(OBJSIZE_16) shl 14) or (integer(OBJSHAPE_WIDE)   shl 12) or ((32*8)  shr 5);
  SpriteSize_32x16: SpriteSize = (integer(OBJSIZE_32) shl 14) or (integer(OBJSHAPE_WIDE)   shl 12) or ((32*16) shr 5);
  SpriteSize_64x32: SpriteSize = (integer(OBJSIZE_64) shl 14) or (integer(OBJSHAPE_WIDE)   shl 12) or ((64*32) shr 5);
  SpriteSize_8x16 : SpriteSize = (integer(OBJSIZE_8)  shl 14) or (integer(OBJSHAPE_TALL)   shl 12) or ((8*16)  shr 5);
  SpriteSize_8x32 : SpriteSize = (integer(OBJSIZE_16) shl 14) or (integer(OBJSHAPE_TALL)   shl 12) or ((8*32)  shr 5);
  SpriteSize_16x32: SpriteSize = (integer(OBJSIZE_32) shl 14) or (integer(OBJSHAPE_TALL)   shl 12) or ((16*32) shr 5);
  SpriteSize_32x64: SpriteSize = (integer(OBJSIZE_64) shl 14) or (integer(OBJSHAPE_TALL)   shl 12) or ((32*64) shr 5);  

function SPRITE_SIZE_SHAPE(size: cint): cuint32; inline;
function SPRITE_SIZE_SIZE(size: cint): cuint32; inline;
function SPRITE_SIZE_PIXELS(size: cint): cuint32; inline;


type
  TSpriteMapping = integer;
const
  SpriteMapping_1D_32     : TSpriteMapping = integer(DISPLAY_SPR_1D) or integer(DISPLAY_SPR_1D_SIZE_32)  or (0 shl 28) or 0;
  SpriteMapping_1D_64     : TSpriteMapping = integer(DISPLAY_SPR_1D) or integer(DISPLAY_SPR_1D_SIZE_64)  or (1 shl 28) or 1;
  SpriteMapping_1D_128    : TSpriteMapping = integer(DISPLAY_SPR_1D) or integer(DISPLAY_SPR_1D_SIZE_128) or (2 shl 28) or 2;
  SpriteMapping_1D_256    : TSpriteMapping = integer(DISPLAY_SPR_1D) or integer(DISPLAY_SPR_1D_SIZE_256) or (3 shl 28) or 3;
  SpriteMapping_2D        : TSpriteMapping = integer(DISPLAY_SPR_2D) or (4 shl 28);
  SpriteMapping_Bmp_1D_128: TSpriteMapping = integer(DISPLAY_SPR_1D) or integer(DISPLAY_SPR_1D_SIZE_128) or integer(DISPLAY_SPR_1D_BMP) or integer(DISPLAY_SPR_1D_BMP_SIZE_128) or (5 shl 28) or 2;
  SpriteMapping_Bmp_1D_256: TSpriteMapping = integer(DISPLAY_SPR_1D) or integer(DISPLAY_SPR_1D_SIZE_256) or integer(DISPLAY_SPR_1D_BMP) or integer(DISPLAY_SPR_1D_BMP_SIZE_256) or (6 shl 28) or 3;
  SpriteMapping_Bmp_2D_128: TSpriteMapping = integer(DISPLAY_SPR_2D) or integer(DISPLAY_SPR_2D_BMP_128)  or (7 shl 28) or 2;
  SpriteMapping_Bmp_2D_256: TSpriteMapping = integer(DISPLAY_SPR_2D) or integer(DISPLAY_SPR_2D_BMP_256)  or (8 shl 28) or 3;

type
  SpriteColorFormat = integer;
const 
  SpriteColorFormat_16Color : SpriteColorFormat = integer(OBJCOLOR_16);
  SpriteColorFormat_256Color: SpriteColorFormat = integer(OBJCOLOR_256);
  SpriteColorFormat_Bmp     : SpriteColorFormat = integer(OBJMODE_BITMAP);	 	 	 

  
type
  AllocHeader = record
    nextFree: cuint16;	    
    size: cuint16;
  end;  
  TAllocHeader = AllocHeader;
  PAllocHeader = ^AllocHeader;
  
  OamState = record
    gfxOffsetStep: cint; 
    firstFree: cint16;
    allocBuffer: PAllocHeader; 
    allocBufferSize: cint16;
    spriteMapping: TSpriteMapping;     
    case integer of
      0: (
            oamMemory: PSpriteEntry;
      ); 
      1: (
            oamRotationMemory: PSpriteRotation;
      );
  end;
  TOamState = OamState;
  POamState = ^OamState; 

var
  oamMain: OamState; cvar; external;
  oamSub: OamState; cvar; external;  

procedure oamInit(var oam: OamState; mapping: TSpriteMapping; extPalette: cbool); cdecl; external;
procedure oamDisable(var oam: OamState); cdecl; external;
procedure oamEnable(var oam: OamState); cdecl; external;
function oamGetGfxPtr(var oam: OamState; gfxOffsetIndex: cint): pcuint16; cdecl; external;
function oamAllocateGfx(var oam: OamState; size: SpriteSize; colorFormat: SpriteColorFormat): pcuint16; cdecl; external;
procedure oamFreeGfx(var oam: OamState; const gfxOffset: pointer); cdecl; external;

procedure oamSetMosaic(dx, dy: cuint); inline;
procedure oamSetMosaicSub(dx, dy: cuint); inline;

procedure oamSet( var oam: OamState; 
                  id, x, y, priority, palette_alpha: cint; 
                  size: SpriteSize; 
                  format: SpriteColorFormat; 
                  const gfxOffset: pointer; 
                  affineIndex: cint; 
                  sizeDouble, hide, hflip, vflip, mosaic: cbool); cdecl; external;
procedure oamClear(var oam: OamState; start, count: cint); cdecl; external;
procedure oamClearSprite(var oam: OamState; index: cint); inline;
procedure oamUpdate(var oam: OamState); cdecl; external;
procedure oamRotateScale(var oam: OamState; rotId, angle, sx, sy: cint); cdecl; external;
procedure oamAffineTransformation(var oam: OamState; rotId, hdx, hdy, vdx, vdy: cint); inline;

function oamCountFragments(var oam: OamState): cint; cdecl; external;
procedure oamAllocReset(var oam: OamState); cdecl; external;
function oamGfxPtrToOffset(var oam: OamState; const offset: pointer): cuint; cdecl; external;
{$endif NDS_INTERFACE}
 
{$ifdef NDS_IMPLEMENTATION}
function OBJ_Y(m: cint): cint; inline;
begin
  OBJ_Y := ((m) and $00ff);
end;

function ATTR1_ROTDATA(n: cint): cint; inline;
begin
  ATTR1_ROTDATA := ((n) shl 9)  // note: overlaps with flip flags
end;

function OBJ_X(m: cint): cint; inline;
begin
  OBJ_X := ((m) and $01ff);
end;

function ATTR2_PRIORITY(n: cint): cint; inline;     
begin
  ATTR2_PRIORITY := ((n) shl 10);
end;

function ATTR2_PALETTE(n: cint): cint; inline;      
begin
  ATTR2_PALETTE := ((n) shl 12);
end;

function ATTR2_ALPHA(n: cint): cint; inline;		  
begin
  ATTR2_ALPHA := ((n) shl 12);
end;

function SPRITE_SIZE_SHAPE(size: cint): cuint32; inline;
begin
  SPRITE_SIZE_SHAPE := (((size) shr 12) and $3);
end;

function SPRITE_SIZE_SIZE(size: cint): cuint32; inline;
begin
  SPRITE_SIZE_SIZE := (((size) shr 14) and $3);
end;

function SPRITE_SIZE_PIXELS(size: cint): cuint32; inline;
begin
  SPRITE_SIZE_PIXELS := (((size) and $FFF) shl 5);
end;

procedure oamSetMosaic(dx, dy: cuint); inline;
begin
<<<<<<< HEAD
  sassert((dx < 16) and (dy < 16), 'Mosaic range must be 0 to 15');
=======
  sassert((dx < 16) and (dy < 16), 'Mosaic range is 0 to 15');
>>>>>>> origin/fixes_2.4
  mosaicShadow := ( mosaicShadow and $00ff) or (dx shl 8) or (dy shl 12);
  REG_MOSAIC^ := mosaicShadow;
end;

procedure oamSetMosaicSub(dx, dy: cuint); inline;
begin
<<<<<<< HEAD
  sassert((dx < 16) and (dy < 16), 'Mosaic range must be 0 to 15');
=======
  sassert((dx < 16) and (dy < 16), 'Mosaic range is 0 to 15');
>>>>>>> origin/fixes_2.4
  mosaicShadowSub := (mosaicShadowSub and $00ff) or (dx shl 8) or (dy shl 12);
  REG_MOSAIC_SUB^ := mosaicShadowSub;
end;

<<<<<<< HEAD
procedure oamClearSprite(var oam: OamState; index: cint); inline;
begin
  sassert( (index >= 0) and (index < SPRITE_COUNT), 'oamClearSprite() index is out of bounds, must be 0-127');
  oam.oamMemory[index].attribute[0] := ATTR0_DISABLED;
end;

procedure oamAffineTransformation(var oam: OamState; rotId, hdx, hdy, vdx, vdy: cint); inline;
begin
  sassert((rotId >= 0) and (rotId < 32), 'oamAffineTransformation() rotId is out of bounds, must be 0-31');
  oam.oamRotationMemory[rotId].hdx := hdx;
  oam.oamRotationMemory[rotId].vdx := vdx;
  oam.oamRotationMemory[rotId].hdy := hdy;
  oam.oamRotationMemory[rotId].vdy := vdy;
=======
procedure oamAffineTransformation(var oam: OamState; rotId, hdx, hdy, vdx, vdy: cint); inline;
begin
  sassert((rotId >= 0) and (rotId < 32), 'oamAffineTransformation() rotId is out of bounds, must be 0-31');
  oam.oamRotationMemory[rotId].hdx := hdx shr 12;
  oam.oamRotationMemory[rotId].hdy := hdy shr 12;
  oam.oamRotationMemory[rotId].vdx := vdx shr 12;
  oam.oamRotationMemory[rotId].vdy := vdy shr 12;
>>>>>>> origin/fixes_2.4
end;


{$endif NDS_IMPLEMENTATION}
=======
=======
>>>>>>> origin/fixes_2_2
  TOAM = record
    case integer of
      0: (spriteBuffer: array [0..SPRITE_COUNT-1] of TSpriteEntry;);
      1: (matrixBuffer: array [0..MATRIX_COUNT-1] of TSpriteRotation;);
  end;
{$endif NDS_INTERFACE}
 
{$ifdef NDS_INTERFACE}
function OBJ_Y(m: cint): cint; inline;
function ATTR1_ROTDATA(n: cint): cint; inline;
function OBJ_X(m: cint): cint; inline;
function ATTR2_PRIORITY(n: cint): cint; inline;     
function ATTR2_PALETTE(n: cint): cint; inline;      
function ATTR2_ALPHA(n: cint): cint; inline;		  
{$endif NDS_INTERFACE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
