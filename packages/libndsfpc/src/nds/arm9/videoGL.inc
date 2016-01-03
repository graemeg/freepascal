<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: videoGL.inc 25 2007-12-10 21:06:46Z p4p3r0 $
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


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifndef ARM9}
  {$error 3D hardware is only available from the ARM9}
{$endif}


{$ifdef NDS_INTERFACE}
(*---------------------------------------------------------------------------------
	lut resolution for trig functions
	(must be power of two and must be the same as LUT resolution)
	in other words dont change unless you also change your LUTs
---------------------------------------------------------------------------------*)
const
<<<<<<< HEAD
<<<<<<< HEAD
  LUT_SIZE = (1 shl 15);
  LUT_MASK = ((1 shl 15) - 1);
=======
  LUT_SIZE = (512);
  LUT_MASK = ($1FF);
>>>>>>> graemeg/fixes_2_2
=======
  LUT_SIZE = (512);
  LUT_MASK = ($1FF);
>>>>>>> origin/fixes_2_2

type
  GLuint = cuint32;
  GLfloat = cfloat;

<<<<<<< HEAD
<<<<<<< HEAD
const
  MAX_TEXTURES = 2048;  //this should be enough ! but feel free to change

//////////////////////////////////////////////////////////////////////
// Fixed point / floating point / integer conversion macros
//////////////////////////////////////////////////////////////////////
type
  fixed12d3 = cuint16 ;

function intto12d3(n: cint): fixed12d3; inline;
function floatto12d3(n: cfloat): fixed12d3; inline;

const
  GL_MAX_DEPTH     = $7FFF;

<<<<<<< HEAD
=======
function inttof32(n: cint): cint32; inline;
function f32toint(n: cint32): cint; inline;
function floattof32(n: cfloat): cint32; inline;  //inlining it makes impossible to pass it to another function :/
function f32tofloat(n: cint32): cfloat; inline;

>>>>>>> graemeg/cpstrnew
type
  t16 = cshort;//cint16?       // text coordinate 12.4 fixed point (or cint?)

function f32tot16(n: cint32): t16; inline;
function inttot16(n: cint): cint32; inline;    //???
function t16toint(n: t16): cint; inline;
function floattot16(n: cfloat): t16; inline;
function TEXTURE_PACK(u, v: cint): cint; inline;
//function TEXTURE_PACK(u, v: cshort): cuint; inline;

type
  v16 = cshort;//cuint16?       // vertex 4.12 fixed format

function inttov16(n: cint): cint{v16}; inline;
function f32tov16(n: cint32): v16; inline; 
function v16toint(n: v16): cint; inline;
function floattov16(n: cfloat): v16; inline;
function VERTEX_PACK(x,y: cint): cuint32; inline;

type
  v10 = cint16;

function inttov10(n: cint): cint; inline;
function f32tov10(n: cint32): cint; inline;
function v10toint(n: v10): cint; inline;
function floattov10(n: cfloat): v10; inline;
function NORMAL_PACK(x,y,z: cint): cuint32; inline;

=======
=======
>>>>>>> origin/fixes_2_2
//////////////////////////////////////////////////////////////////////
// Fixed point / floating point / integer conversion macros
//////////////////////////////////////////////////////////////////////

  fixed12d3 = cuint16 ;
{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}
function int_to_12d3(n: cint): fixed12d3; inline;    
begin
  int_to_12d3 := fixed12d3(n shl 3);
end;

function float_to_12d3(n: cfloat): fixed12d3; inline;  
begin
  float_to_12d3 := fixed12d3(trunc( n  * (1 shl 3)));
end;
{$endif NDS_IMPLEMENTATION}

{$ifdef NDS_INTERFACE}
const
  GL_MAX_DEPTH     = $7FFF;
{$endif NDS_INTERFACE}

//////////////////////////////////////////////////////////////////////

{$ifdef NDS_IMPLEMENTATION}
function inttof32(n: cint): cint32; inline;          
begin
  inttof32 := ((n) shl 12);
end;

function f32toint(n: cint32): cint; inline;          
begin
  f32toint := ((n) shr 12);
end;

function floattof32(n: cfloat): cint32; inline;        
begin
  floattof32 := trunc((n) * (1 shl 12)); 
end;

// check it!
function f32tofloat(n: cint32): cfloat; inline;        
begin
  f32tofloat := (n * 1.0) / ((1 shl 12) * 1.0);
end;
{$endif NDS_IMPLEMENTATION}

{$ifdef NDS_INTERFACE}
type
  t16 = cint16;       // text coordinate 12.4 fixed point (or cint?)
{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}
function f32tot16(n: cint32): t16; inline;            
begin
  f32tot16 := t16(n shr 8);
end;

function inttot16(n: cint): cint32; inline;    //???
begin
  inttot16 := (n shl 4);
end;

function t16toint(n: t16): cint; inline;            
begin
  t16toint := cint((n) shr 4);
end;

function floattot16(n: cfloat): t16; inline;          
begin
  floattot16 := t16(trunc(n * (1 shl 4)));
end;

function TEXTURE_PACK(u, v: cint): cint; inline;      
begin
  TEXTURE_PACK := (u and $FFFF) or (v shl 16);
end;
{$endif NDS_IMPLEMENTATION}

{$ifdef NDS_INTERFACE}
type
  v16 = cuint16;       // vertex 4.12 fixed format
{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}
function inttov16(n: cint): cint; inline;        
begin
  inttov16 := (n shl 12);
end;

function f32tov16(n: cint32): v16; inline;            
begin
  f32tov16 := v16(n);
end;

function v16toint(n: v16): cint; inline;            
begin
  v16toint := cint((n) shr 12);
end;

function floattov16(n: cfloat): v16; inline;           
begin
  floattov16 := v16(trunc(n * (1 shl 12)));
end;

function VERTEX_PACK(x,y: cint): cint; inline;  		 
begin
  VERTEX_PACK := (x and $FFFF) or (y shl 16);
end;
{$endif NDS_IMPLEMENTATION}

{$ifdef NDS_INTERFACE}
type
  v10 = cint16;
{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}
function inttov10(n: cint): cint; inline;            
begin
  inttov10 := ((n) shl 9);
end;

function f32tov10(n: cint32): cint; inline;            
begin
  f32tov10 := (n shr 3);
end;

function v10toint(n: v10): cint; inline;            
begin
  v10toint := cint((n) shr 9);
end;

function floattov10(n: cfloat): v10; inline;
begin
  if n > 0.998 then 
	  floattov10 := v10($1FF)
	else
	  floattov10 := trunc(n * (1 shl 9));
end;

function NORMAL_PACK(x,y,z: cint): cint; inline;     
begin
  NORMAL_PACK := ((x and $3FF) or ((y and $3FF) shl 10) or (z shl 20));
end;
{$endif NDS_IMPLEMENTATION}


{$ifdef NDS_INTERFACE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
type
  rgb = cushort;

  m3x3 = record
    m: array [0..8] of cint32;
  end;
  pm3x3 = ^m3x3;
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
  m4x4 = record
    m: array [0..15] of cint32;
  end; 
  pm4x4 = ^m4x4;
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
  m4x3 = record
    m: array [0..11] of cint32;
  end;
  pm4x3 = ^m4x3;
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
  GLvector = record
    x,y,z: cint32;
  end;
  PGLvector= ^GLvector;

<<<<<<< HEAD
<<<<<<< HEAD
=======
//////////////////////////////////////////////////////////////////////

>>>>>>> graemeg/fixes_2_2
=======
//////////////////////////////////////////////////////////////////////

>>>>>>> origin/fixes_2_2
const
  GL_FALSE    = 0;
  GL_TRUE     = 1;

//////////////////////////////////////////////////////////////////////
// glBegin constants
//////////////////////////////////////////////////////////////////////
type
  GL_GLBEGIN_ENUM = cint32;
const
  GL_TRIANGLES      : GL_GLBEGIN_ENUM = 0;
  GL_QUADS          : GL_GLBEGIN_ENUM = 1;
  GL_TRIANGLE_STRIP : GL_GLBEGIN_ENUM = 2;
  GL_QUAD_STRIP     : GL_GLBEGIN_ENUM = 3;
  GL_TRIANGLE       : GL_GLBEGIN_ENUM = 0;
  GL_QUAD           : GL_GLBEGIN_ENUM = 1;

//////////////////////////////////////////////////////////////////////
// glMatrixMode constants
//////////////////////////////////////////////////////////////////////
type
  GL_MATRIX_MODE_ENUM = cint32;
const
  GL_PROJECTION      : GL_MATRIX_MODE_ENUM = 0;
  GL_POSITION        : GL_MATRIX_MODE_ENUM = 1;
  GL_MODELVIEW       : GL_MATRIX_MODE_ENUM = 2;
  GL_TEXTURE         : GL_MATRIX_MODE_ENUM = 3;

//////////////////////////////////////////////////////////////////////
// glMaterialf constants
//////////////////////////////////////////////////////////////////////
type
  GL_MATERIALS_ENUM = cint32;
const
  GL_AMBIENT              : GL_MATERIALS_ENUM = $01;
  GL_DIFFUSE              : GL_MATERIALS_ENUM = $02;
  GL_AMBIENT_AND_DIFFUSE  : GL_MATERIALS_ENUM = $03;
  GL_SPECULAR             : GL_MATERIALS_ENUM = $04;
  GL_SHININESS            : GL_MATERIALS_ENUM = $08;
  GL_EMISSION             : GL_MATERIALS_ENUM = $10;

////////////////////////////////////////////////////////////
// glPolyFmt constants
////////////////////////////////////////////////////////////
type
  GL_POLY_FORMAT_ENUM = cint32;
const
<<<<<<< HEAD
<<<<<<< HEAD
  POLY_FORMAT_LIGHT0   : GL_POLY_FORMAT_ENUM = (1 shl 0);
  POLY_FORMAT_LIGHT1   : GL_POLY_FORMAT_ENUM = (1 shl 1);
  POLY_FORMAT_LIGHT2   : GL_POLY_FORMAT_ENUM = (1 shl 2);
  POLY_FORMAT_LIGHT3   : GL_POLY_FORMAT_ENUM = (1 shl 3);
  POLY_MODULATION      : GL_POLY_FORMAT_ENUM = (0 shl 4);
  POLY_DECAL           : GL_POLY_FORMAT_ENUM = (1 shl 4);
  POLY_TOON_HIGHLIGHT  : GL_POLY_FORMAT_ENUM = (2 shl 4);
  POLY_SHADOW          : GL_POLY_FORMAT_ENUM = (3 shl 4);
  POLY_CULL_FRONT      : GL_POLY_FORMAT_ENUM = (1 shl 6);
  POLY_CULL_BACK       : GL_POLY_FORMAT_ENUM = (2 shl 6);
  POLY_CULL_NONE       : GL_POLY_FORMAT_ENUM = (3 shl 6);
  POLY_FOG             : GL_POLY_FORMAT_ENUM = (1 shl 15);
=======
=======
>>>>>>> origin/fixes_2_2
	POLY_DECAL           : GL_POLY_FORMAT_ENUM = (1 shl 4);
	POLY_TOON_SHADING    : GL_POLY_FORMAT_ENUM = (1 shl 5);
	POLY_CULL_BACK       : GL_POLY_FORMAT_ENUM = (1 shl 7);
	POLY_CULL_FRONT      : GL_POLY_FORMAT_ENUM = (1 shl 6);
	POLY_CULL_NONE       : GL_POLY_FORMAT_ENUM = (3 shl 6);
	POLY_FORMAT_LIGHT0   : GL_POLY_FORMAT_ENUM = (1 shl 0);
	POLY_FORMAT_LIGHT1   : GL_POLY_FORMAT_ENUM = (1 shl 1);
	POLY_FORMAT_LIGHT2   : GL_POLY_FORMAT_ENUM = (1 shl 2);
	POLY_FORMAT_LIGHT3   : GL_POLY_FORMAT_ENUM = (1 shl 3);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2




////////////////////////////////////////////////////////////
// glTexImage2d constants
////////////////////////////////////////////////////////////
type 
  GL_TEXTURE_SIZE_ENUM = cint32;
const
	TEXTURE_SIZE_8    : GL_TEXTURE_SIZE_ENUM = 0;
	TEXTURE_SIZE_16   : GL_TEXTURE_SIZE_ENUM = 1;
	TEXTURE_SIZE_32   : GL_TEXTURE_SIZE_ENUM = 2;
	TEXTURE_SIZE_64   : GL_TEXTURE_SIZE_ENUM = 3;
	TEXTURE_SIZE_128  : GL_TEXTURE_SIZE_ENUM = 4;
	TEXTURE_SIZE_256  : GL_TEXTURE_SIZE_ENUM = 5;
	TEXTURE_SIZE_512  : GL_TEXTURE_SIZE_ENUM = 6;
	TEXTURE_SIZE_1024 : GL_TEXTURE_SIZE_ENUM = 7;

type
  GL_TEXTURE_PARAM_ENUM = cuint32;
const
	GL_TEXTURE_WRAP_S : GL_TEXTURE_PARAM_ENUM = (1 shl 16);
	GL_TEXTURE_WRAP_T : GL_TEXTURE_PARAM_ENUM = (1 shl 17);
	GL_TEXTURE_FLIP_S : GL_TEXTURE_PARAM_ENUM = (1 shl 18);
	GL_TEXTURE_FLIP_T : GL_TEXTURE_PARAM_ENUM = (1 shl 19);
	GL_TEXTURE_COLOR0_TRANSPARENT : GL_TEXTURE_PARAM_ENUM = (1 shl 29);
	GL_TEXTURE_ALPHA_MASK : GL_TEXTURE_PARAM_ENUM = (1 shl 29);
	TEXGEN_OFF      : GL_TEXTURE_PARAM_ENUM = (0 shl 30);
	TEXGEN_TEXCOORD : GL_TEXTURE_PARAM_ENUM = (1 shl 30);
	TEXGEN_NORMAL   : GL_TEXTURE_PARAM_ENUM = (2 shl 30);
	TEXGEN_POSITION : GL_TEXTURE_PARAM_ENUM = (3 shl 30);


type
  GL_TEXTURE_TYPE_ENUM = cint32;
const
<<<<<<< HEAD
<<<<<<< HEAD
  GL_NOTEXTURE  : GL_TEXTURE_TYPE_ENUM = 0; 
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
	GL_RGB32_A3   : GL_TEXTURE_TYPE_ENUM = 1;
	GL_RGB4       : GL_TEXTURE_TYPE_ENUM = 2;
	GL_RGB16      : GL_TEXTURE_TYPE_ENUM = 3;
	GL_RGB256     : GL_TEXTURE_TYPE_ENUM = 4;
	GL_COMPRESSED : GL_TEXTURE_TYPE_ENUM = 5;
	GL_RGB8_A5    : GL_TEXTURE_TYPE_ENUM = 6;
	GL_RGBA       : GL_TEXTURE_TYPE_ENUM = 7;
	GL_RGB        : GL_TEXTURE_TYPE_ENUM = 8;

<<<<<<< HEAD
<<<<<<< HEAD
type
  GL_TEXTURE_PALETTE_PARAM_ENUM = cint32;
const
  GL_COLOR_TABLE_FORMAT_EXT: GL_TEXTURE_PALETTE_PARAM_ENUM	= 0;
	GL_COLOR_TABLE_WIDTH_EXT : GL_TEXTURE_PALETTE_PARAM_ENUM	= 1;
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2

type
  DISP3DCNT_ENUM = cint32;
const
	GL_TEXTURE_2D      : DISP3DCNT_ENUM = (1 shl 0);
	GL_TOON_HIGHLIGHT  : DISP3DCNT_ENUM = (1 shl 1);
	GL_ALPHA_TEST      : DISP3DCNT_ENUM = (1 shl 2);
	GL_BLEND           : DISP3DCNT_ENUM = (1 shl 3);
	GL_ANTIALIAS       : DISP3DCNT_ENUM = (1 shl 4);
	GL_OUTLINE         : DISP3DCNT_ENUM = (1 shl 5);
<<<<<<< HEAD
<<<<<<< HEAD
  GL_FOG_ONLY_ALPHA  : DISP3DCNT_ENUM = (1 shl 6);
  GL_FOG             : DISP3DCNT_ENUM = (1 shl 7);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
	GL_COLOR_UNDERFLOW : DISP3DCNT_ENUM = (1 shl 12);
	GL_POLY_OVERFLOW   : DISP3DCNT_ENUM = (1 shl 13);
	GL_CLEAR_BMP       : DISP3DCNT_ENUM = (1 shl 14);

<<<<<<< HEAD
<<<<<<< HEAD
type
  GL_GET_ENUM = (
    GL_GET_VERTEX_RAM_COUNT   = 0,
    GL_GET_POLYGON_RAM_COUNT  = 1,
    GL_GET_MATRIX_VECTOR      = 2,
    GL_GET_MATRIX_POSITION    = 3,
    GL_GET_MATRIX_PROJECTION  = 4,
    GL_GET_MATRIX_CLIP        = 5,
    GL_GET_TEXTURE_WIDTH      = 6,
    GL_GET_TEXTURE_HEIGHT     = 7
  );

=======
=======
>>>>>>> origin/fixes_2_2

////////////////////////////////////////////////////////////
// glGet constants
////////////////////////////////////////////////////////////

type
  GL_GET_ENUM = (
    GL_GET_VERTEX_RAM_COUNT,
    GL_GET_POLYGON_RAM_COUNT,
    GL_GET_MATRIX_VECTOR,
    GL_GET_MATRIX_POSITION,
    GL_GET_MATRIX_PROJECTION,
    GL_GET_MATRIX_CLIP,
    GL_GET_TEXTURE_WIDTH,
    GL_GET_TEXTURE_HEIGHT
  );


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
type
	GLFLUSH_ENUM = cint32;
const
	GL_TRANS_MANUALSORT: GLFLUSH_ENUM = (1 shl 0);
	GL_WBUFFERING      : GLFLUSH_ENUM = (1 shl 1); 

<<<<<<< HEAD
<<<<<<< HEAD

(*-----------------------------------------
Structures specific to allocating and 
deallocating video RAM in videoGL
-----------------------------------------*)


type 
  ps_SingleBlock = ^s_SingleBlock; 
  s_SingleBlock = packed record
    indexOut: cuint32;
    AddrSet: pcuint8;
    node: array [0..3] of ps_SingleBlock; // 0-1 ~ prev/next memory block, 2-3 ~ prev/next empty/alloc block
    blockSize: cuint32;
  end;

  s_vramBlock = packed record
    startAddr: pcuint8;
    endAddr: pcuint8;
    firstBlock: ps_SingleBlock;
    firstEmpty: ps_SingleBlock;
    firstAlloc: ps_SingleBlock;
    
    lastExamined: ps_SingleBlock;
    lastExaminedAddr: pcuint8;
    lastExaminedSize: cuint32;
    
    blockPtrs: DynamicArray;
    deallocBlocks: DynamicArray;
    
    blockCount: cuint32;
    deallocCount: cuint32;
  end;
  ps_vramBlock = ^s_vramBlock;
  

  gl_texture_data = packed record
    vramAddr: pointer;			// Address to the texture loaded into VRAM
    texIndex: cuint32;		// The index in the Memory Block
    texIndexExt: cuint32;		// The secondary index in the Memory block (only for GL_COMPRESSED textures)
    palIndex: cint;			// The palette index 
    texFormat: cuint32;		// Specifications of how the texture is displayed
    texSize: cuint32;			// The size (in blocks) of the texture
  end;
  pgl_texture_data = ^gl_texture_data;
  
  gl_palette_data = packed record
    vramAddr: pointer;			// Address to the palette loaded into VRAM
    palIndex: cuint32;		// The index in the Memory Block
    addr: cuint16;			// The offset address for texture palettes in VRAM
    palSize: cuint16;			// The length of the palette
    connectCount: cuint32;	// The number of textures currently using this palette
  end;
  pgl_palette_data = ^gl_palette_data;



=======
=======
>>>>>>> origin/fixes_2_2
////////////////////////////////////////////////////////////
// Misc. constants
////////////////////////////////////////////////////////////

const
  MAX_TEXTURES = 2048;  //this should be enough ! but feel free to change
  GL_LIGHTING  = 1;   // no idea what this is for / who defined it
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

type
  gl_hidden_globals = record
    matrixMode: GL_MATRIX_MODE_ENUM; // holds the current Matrix Mode
<<<<<<< HEAD
<<<<<<< HEAD

    vramBlocks: array [0..1] of ps_vramBlock;		// Two classe instances, one for textures, and one for palettes
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    vramLock: array [0..1] of cint;				// Holds the current lock state of the VRAM banks
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    
    // texture globals
    texturePtrs: DynamicArray;		// Pointers to each individual texture
    palettePtrs: DynamicArray;		// Pointers to each individual palette
    
    deallocTex: DynamicArray;		// Preserves deleted texture names for later use with glGenTextures
    deallocPal: DynamicArray;		// Preserves deleted palette names
    deallocTexSize: cuint32;			// Preserved number of deleted texture names
    deallocPalSize: cuint32;			// Preserved number of deleted palette names
    
    activeTexture: cint;				// The current active texture name
    activePalette: cint;				// The current active palette name
    texCount: cint;
    palCount: cint;

    // holds the current state of the clear color register
    clearColor: cuint32; // state of clear color register
    isActive: cuint8;		// Has this been called before?
  end;
  TGLHiddenGlobals = gl_hidden_globals;
  PGLHiddenGlobals = ^gl_hidden_globals;

var
  glGlobalData: gl_hidden_globals; cvar; external;
  // Pointer to global data for videoGL
  glGlob: PGLHiddenGlobals = @glGlobalData;

type 
  TArr4ofInt = array [0..3] of cint32;

function FIFO_COMMAND_PACK(c1,c2,c3,c4: cint): cint; inline;
function REG2ID(r: pcuint32): cuint8; inline;
function FIFO_NOP(): cuint8; inline;
function FIFO_STATUS(): cuint8; inline;
function FIFO_COLOR(): cuint8; inline;
function FIFO_VERTEX16(): cuint8; inline;
function FIFO_VERTEX10(): cuint8; inline;
function FIFO_VERTEX_XY(): cuint8; inline;
function FIFO_VERTEX_XZ(): cuint8; inline;
function FIFO_VERTEX_YZ(): cuint8; inline;
function FIFO_TEX_COORD(): cuint8; inline;
function FIFO_TEX_FORMAT(): cuint8; inline;
function FIFO_PAL_FORMAT(): cuint8; inline;
function FIFO_CLEAR_COLOR(): cuint8; inline;
function FIFO_CLEAR_DEPTH(): cuint8; inline;
function FIFO_LIGHT_VECTOR(): cuint8; inline;
function FIFO_LIGHT_COLOR(): cuint8; inline;
function FIFO_NORMAL(): cuint8; inline;
function FIFO_DIFFUSE_AMBIENT(): cuint8; inline;
function FIFO_SPECULAR_EMISSION(): cuint8; inline;
function FIFO_SHININESS(): cuint8; inline;
function FIFO_POLY_FORMAT(): cuint8; inline;
function FIFO_BEGIN(): cuint8; inline;
function FIFO_END(): cuint8; inline;
function FIFO_FLUSH(): cuint8; inline;
function FIFO_VIEWPORT(): cuint8; inline;

procedure glRotatef32i(angle: cint; x, y, z: cint32); cdecl; external;
function glTexImage2D(target, empty1: cint; _type: GL_TEXTURE_TYPE_ENUM; sizeX, sizeY, empty2, param: cint; texture: pointer): cint; cdecl; external;

procedure glColorTableEXT(target, empty1: cint; width: cuint16; empty2, empty3: cint; const table: pcuint16); cdecl; external;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure glColorSubTableEXT(target, start, count, empty1, empty2: cint; const data: pcuint16); cdecl; external;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure glAssignColorTable(target, name: cint); cdecl; external;

//procedure glTexLoadPal(const pal: pcuint16; count: cuint16; addr: cuint32); cdecl; external;
//function gluTexLoadPal(const pal: pcuint16; count: cuint16; format: cuint8): cint; cdecl; external;

procedure glTexParameter(target, param: cint); cdecl; external;
function glGetTexParameter(): cuint32; cdecl; external;
procedure glGetColorTableParameterEXT(target, pname: cint; params: pcint); cdecl; external;
function glGetTexturePointer(name: cint): pointer; cdecl; external;
procedure glBindTexture(target, name: cint); cdecl; external;
//procedure glColorTable(format: cuint8; addr: cuint32); cdecl; external;
function glGenTextures(n: cint; names: pcint): cint; cdecl; external;
function glDeleteTextures(n: cint; names: pcint): cint; cdecl; external;
procedure glResetTextures(); cdecl; external;

function glLockVRAMBank(addr: pcuint16): cint; cdecl; external;
function glUnlockVRAMBank(addr: pcuint16): cint; cdecl; external;

procedure glTexCoord2f32(u, v: cint32); cdecl; external;
procedure glMaterialf(mode: GL_MATERIALS_ENUM; color: rgb); cdecl; external;
procedure glInit_C(); cdecl; external;
function glGetGlobals(): PGLHiddenGlobals; cdecl; external;

function POLY_ALPHA(n: cuint32): cuint32; inline;
function POLY_ID(n: cuint32): cuint32; inline;	
procedure glBegin(mode: GL_GLBEGIN_ENUM); inline;
procedure glEnd(); inline;
procedure glClearDepth(depth: fixed12d3); inline;
procedure glColor3b(red, green, blue: cuint8); inline;
procedure glColor(color: rgb); inline;
procedure glVertex3v16(x, y, z: v16); inline;
procedure glTexCoord2t16(u, v: t16); inline;
procedure glPushMatrix(); inline;
procedure glPopMatrix(num: cint32); inline;
procedure glRestoreMatrix(index: cint32); inline;
procedure glStoreMatrix(index: cint32); inline;
procedure glScalev(const v: PGLvector); inline;
procedure glTranslatev(const v: PGLvector) ; inline;
procedure glTranslate3f32(x, y, z: cint32); inline;
procedure glTranslatef32(x, y, z: cint32); inline;
procedure glScalef32(x, y, z: cint32); inline;
procedure glLight(id: cint; color: rgb; x, y, z: v10); inline;
procedure glNormal(normal: cuint32); inline;
procedure glLoadIdentity(); inline;
procedure glMatrixMode(mode: GL_MATRIX_MODE_ENUM); inline;
procedure glViewport(x1, y1, x2, y2: cuint8); inline;
procedure glFlush(mode: cuint32); inline;
procedure glMaterialShinyness(); inline;
procedure glCallList(list: pcuint32); inline;
procedure glPolyFmt(params: cuint32); inline;
procedure glEnable(bits: cint); inline;
procedure glDisable(bits: cint); inline;
procedure glFogShift(shift: cint); inline;
procedure glFogOffset(offset: cint); inline; 
procedure glFogColor(red, green, blue, alpha: cuint8); inline;
procedure glFogDensity(index, density: cint); inline;
procedure glLoadMatrix4x4(const m: pm4x4); inline;
procedure glLoadMatrix4x3(const m: pm4x3); inline;
procedure glMultMatrix4x4(const m: pm4x4); inline;
procedure glMultMatrix4x3(const m: pm4x3); inline;
procedure glMultMatrix3x3(const m: pm3x3); inline;
procedure glRotateXi(angle: cint); inline;
procedure glRotateYi(angle: cint); inline;
procedure glRotateZi(angle: cint); inline;
procedure glOrthof32(left, right, bottom, top, zNear, zFar: cint32); inline;
procedure gluLookAtf32(eyex, eyey, eyez, lookAtx, lookAty, lookAtz, upx, upy, upz: cint32); inline;
procedure glFrustumf32(left, right, bottom, top, zNear, zFar: cint32); inline;
procedure gluPerspectivef32(fovy: cint; aspect, zNear, zFar: cint32); inline;
procedure gluPickMatrix(x, y, width, height: cint32; const viewport: TArr4ofInt); inline;
procedure glResetMatrixStack(); inline;
procedure glSetOutlineColor(id: cint; color: rgb); inline;
procedure glSetToonTable(const table: pcuint16); inline;
procedure glSetToonTableRange(_start, _end: cint; color: rgb); inline;
procedure glGetFixed(const param: GL_GET_ENUM; f: pcint32); inline;
procedure glAlphaFunc(alphaThreshold: cint); inline;
procedure glCutoffDepth(wVal: fixed12d3); inline;
procedure glInit(); inline;
procedure glClearColor(red, green, blue, alpha: cuint8); inline;
procedure glClearPolyID(ID: cuint8); inline;
procedure glGetInt(param: GL_GET_ENUM; var i: cint); inline;
procedure glVertex3f(x, y, z: cfloat); inline;
procedure glRotatef32(angle: cfloat; x, y, z: cint32); inline;
procedure glRotatef(angle, x, y, z: cfloat); inline;
procedure glColor3f(r, g, b: cfloat); inline;
procedure glScalef(x, y, z: cfloat); inline;
procedure glTranslatef(x, y, z: cfloat); inline;
procedure glNormal3f(x, y, z: cfloat); inline;
procedure glRotateX(angle: cfloat); inline;
procedure glRotateY(angle: cfloat); inline;
procedure glRotateZ(angle: cfloat); inline;
procedure glOrtho(left, right, bottom, top, zNear, zFar: cfloat); inline;
procedure gluLookAt(eyex, eyey, eyez, lookAtx, lookAty, lookAtz, upx, upy, upz: cfloat); inline;
procedure glFrustum(left, right, bottom, top, znear, zfar: cfloat); inline;
procedure gluPerspective(fovy, aspect, zNear, zFar: cfloat); inline;
procedure glTexCoord2f(s, t: cfloat); inline;
{$endif NDS_INTERFACE}



{$ifdef NDS_IMPLEMENTATION}
function intto12d3(n: cint): fixed12d3; inline;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  intto12d3 := fixed12d3(n shl 3);
end;

function floatto12d3(n: cfloat): fixed12d3; inline;
begin
  floatto12d3 := fixed12d3(trunc( n  * (1 shl 3)));
end;

function inttof32(n: cint): cint32; inline;
begin
  inttof32 := ((n) shl 12);
end;

function f32toint(n: cint32): cint; inline;
>>>>>>> graemeg/cpstrnew
begin
  intto12d3 := fixed12d3(n shl 3);
end;

function floatto12d3(n: cfloat): fixed12d3; inline;
begin
  floatto12d3 := fixed12d3(trunc( n  * (1 shl 3)));
end;

function f32tot16(n: cint32): t16; inline;
begin
  f32tot16 := t16(n shr 8);
end;

function inttot16(n: cint): cint32; inline;    //???
begin
  inttot16 := (n shl 4);
end;

function t16toint(n: t16): cint; inline;
begin
  t16toint := cint((n) shr 4);
end;

function floattot16(n: cfloat): t16; inline;
begin
  floattot16 := t16(trunc(n * (1 shl 4)));
end;

function TEXTURE_PACK(u, v: cint): cint; inline;
//function TEXTURE_PACK(u, v: cshort): cuint; inline;
begin
  TEXTURE_PACK := (u and $FFFF) or (v shl 16);
end;

function inttov16(n: cint): cint{v16}; inline;
begin
  inttov16 := (n shl 12);
end;

function f32tov16(n: cint32): v16; inline;
begin
  f32tov16 := v16(n);
end;

function v16toint(n: v16): cint; inline;
begin
  v16toint := cint((n) shr 12);
end;

function floattov16(n: cfloat): v16; inline;
begin
  floattov16 := v16(trunc(n * (1 shl 12)));
end;

function VERTEX_PACK(x,y: cint): cuint32; inline;
begin
  VERTEX_PACK := cuint32((x and $FFFF) or (y shl 16));
end;

function inttov10(n: cint): cint; inline;
begin
  inttov10 := ((n) shl 9);
end;

function f32tov10(n: cint32): cint; inline;
begin
  f32tov10 := (n shr 3);
end;

function v10toint(n: v10): cint; inline;
begin
  v10toint := cint((n) shr 9);
end;

function floattov10(n: cfloat): v10; inline;
begin
  if n > 0.998 then 
	  floattov10 := v10($1FF)
	else
	  floattov10 := trunc(n * (1 shl 9));
end;

function NORMAL_PACK(x,y,z: cint): cuint32; inline;
begin
  NORMAL_PACK := cuint32((x and $3FF) or ((y and $3FF) shl 10) or (z shl 20));
end;

//---------------------------------------------------------------------------------
//Fifo commands
//---------------------------------------------------------------------------------
function FIFO_COMMAND_PACK(c1,c2,c3,c4: cint): cint; inline;
=======
=======
>>>>>>> origin/fixes_2_2
    // holds the current state of the clear color register
    clearColor: cuint32; // state of clear color register
    // texture globals
    textures: array [0..MAX_TEXTURES-1] of cuint32;
    activeTexture: cuint32;
    nextBlock: pcuint32;
    nextPBlock: cuint32;
    nameCount: cint;
  end;
  TGLHiddenGlobals = gl_hidden_globals;
  PGLHiddenGlobals = ^gl_hidden_globals;
  
var
  // Pointer to global data for videoGL
  glGlob: PGLHiddenGlobals = nil;



{$endif NDS_INTERFACE}


{$ifdef NDS_IMPLEMENTATION}
//---------------------------------------------------------------------------------
//Fifo commands
//---------------------------------------------------------------------------------
function FIFO_COMMAND_PACK(c1,c2,c3,c4: cint): cint; inline; 
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  FIFO_COMMAND_PACK := (((c4) shl 24) or ((c3) shl 16) or ((c2) shl 8) or (c1));
end;

<<<<<<< HEAD
<<<<<<< HEAD
function REG2ID(r: pcuint32): cuint8; inline;
=======
function REG2ID(r: pcuint32): cuint8; inline;						
>>>>>>> graemeg/fixes_2_2
=======
function REG2ID(r: pcuint32): cuint8; inline;						
>>>>>>> origin/fixes_2_2
begin
  REG2ID := cuint8((cuint32(r) - $04000400) shr 2);
end;

function FIFO_NOP(): cuint8; inline;
begin
  FIFO_NOP := REG2ID(GFX_FIFO);
end;

function FIFO_STATUS(): cuint8; inline;
begin
  FIFO_STATUS := REG2ID(GFX_STATUS);
end;

function FIFO_COLOR(): cuint8; inline;
begin
  FIFO_COLOR := REG2ID(GFX_COLOR);
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
function FIFO_VERTEX16(): cuint8; inline;
begin
  FIFO_VERTEX16 := REG2ID(GFX_VERTEX16);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function FIFO_VERTEX10(): cuint8; inline;
begin
  FIFO_VERTEX10 := REG2ID(GFX_VERTEX10);
end;

function FIFO_VERTEX_XY(): cuint8; inline;
begin
  FIFO_VERTEX_XY := REG2ID(GFX_VERTEX_XY); 
end;

function FIFO_VERTEX_XZ(): cuint8; inline;
begin
  FIFO_VERTEX_XZ := REG2ID(GFX_VERTEX_XZ); 
end;

function FIFO_VERTEX_YZ(): cuint8; inline;
begin
  FIFO_VERTEX_YZ := REG2ID(GFX_VERTEX_YZ); 
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function FIFO_TEX_COORD(): cuint8; inline;
begin
  FIFO_TEX_COORD := REG2ID(GFX_TEX_COORD);
end;

function FIFO_TEX_FORMAT(): cuint8; inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FIFO_TEX_FORMAT := REG2ID(GFX_TEX_FORMAT);
=======
  FIFO_TEX_FORMAT := REG2ID(GFX_TEX_FORMAT);        
>>>>>>> graemeg/fixes_2_2
=======
  FIFO_TEX_FORMAT := REG2ID(GFX_TEX_FORMAT);        
>>>>>>> origin/fixes_2_2
end;

function FIFO_PAL_FORMAT(): cuint8; inline;
begin
  FIFO_PAL_FORMAT := REG2ID(GFX_PAL_FORMAT);
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
function FIFO_CLEAR_COLOR(): cuint8; inline;
begin
  FIFO_CLEAR_COLOR := REG2ID(GFX_CLEAR_COLOR);
end;

function FIFO_CLEAR_DEPTH(): cuint8; inline;
begin
  FIFO_CLEAR_DEPTH := REG2ID(pcuint32(GFX_CLEAR_DEPTH));
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
function FIFO_LIGHT_VECTOR(): cuint8; inline;
begin
  FIFO_LIGHT_VECTOR := REG2ID(GFX_LIGHT_VECTOR);
end;

function FIFO_LIGHT_COLOR(): cuint8; inline;
begin
  FIFO_LIGHT_COLOR := REG2ID(GFX_LIGHT_COLOR);
end;

function FIFO_NORMAL(): cuint8; inline;
begin
  FIFO_NORMAL := REG2ID(GFX_NORMAL);
end;

function FIFO_DIFFUSE_AMBIENT(): cuint8; inline;
begin
  FIFO_DIFFUSE_AMBIENT := REG2ID(GFX_DIFFUSE_AMBIENT);
end;

function FIFO_SPECULAR_EMISSION(): cuint8; inline;
begin
  FIFO_SPECULAR_EMISSION := REG2ID(GFX_SPECULAR_EMISSION);
end;

function FIFO_SHININESS(): cuint8; inline;
begin
  FIFO_SHININESS := REG2ID(GFX_SHININESS);
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
function FIFO_POLY_FORMAT(): cuint8; inline;
begin
  FIFO_POLY_FORMAT := REG2ID(GFX_POLY_FORMAT);
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
function FIFO_BEGIN(): cuint8; inline;
begin
  FIFO_BEGIN := REG2ID(GFX_BEGIN);
end;

function FIFO_END(): cuint8; inline;
begin
  FIFO_END := REG2ID(GFX_END);
end;

function FIFO_FLUSH(): cuint8; inline;
begin
  FIFO_FLUSH := REG2ID(GFX_FLUSH);
end;

function FIFO_VIEWPORT(): cuint8; inline;
begin
  FIFO_VIEWPORT := REG2ID(GFX_VIEWPORT);
end;

{ $define FIFO_NOP               := REG2ID(GFX_FIFO)}
{ $define FIFO_STATUS            := REG2ID(GFX_STATUS)}
{ $define FIFO_COLOR             := REG2ID(GFX_COLOR)}

{ $define FIFO_VERTEX16          := REG2ID(GFX_VERTEX16)}
{ $define FIFO_TEX_COORD         := REG2ID(GFX_TEX_COORD)}
<<<<<<< HEAD
<<<<<<< HEAD
{ $define FIFO_TEX_FORMAT        := REG2ID(GFX_TEX_FORMAT)}
=======
{ $define FIFO_TEX_FORMAT        := REG2ID(GFX_TEX_FORMAT)}        
>>>>>>> graemeg/fixes_2_2
=======
{ $define FIFO_TEX_FORMAT        := REG2ID(GFX_TEX_FORMAT)}        
>>>>>>> origin/fixes_2_2
{ $define FIFO_PAL_FORMAT        := REG2ID(GFX_PAL_FORMAT)}

{ $define FIFO_CLEAR_COLOR       := REG2ID(GFX_CLEAR_COLOR)}
{ $define FIFO_CLEAR_DEPTH       := REG2ID(GFX_CLEAR_DEPTH)}

{ $define FIFO_LIGHT_VECTOR      := REG2ID(GFX_LIGHT_VECTOR)}
{ $define FIFO_LIGHT_COLOR       := REG2ID(GFX_LIGHT_COLOR)}
{ $define FIFO_NORMAL            := REG2ID(GFX_NORMAL)}

{ $define FIFO_DIFFUSE_AMBIENT   := REG2ID(GFX_DIFFUSE_AMBIENT)}
{ $define FIFO_SPECULAR_EMISSION := REG2ID(GFX_SPECULAR_EMISSION)}
{ $define FIFO_SHININESS         := REG2ID(GFX_SHININESS)}

{ $define FIFO_POLY_FORMAT       := REG2ID(GFX_POLY_FORMAT)}

<<<<<<< HEAD
<<<<<<< HEAD
{ $define FIFO_BEGIN             := REG2ID(GFX_BEGIN)}
{ $define FIFO_END               := REG2ID(GFX_END)}
{ $define FIFO_FLUSH             := REG2ID(GFX_FLUSH)} 
{ $define FIFO_VIEWPORT          := REG2ID(GFX_VIEWPORT)}
=======
=======
>>>>>>> origin/fixes_2_2
{ $define FIFO_BEGIN             := REG2ID(GFX_BEGIN)}        
{ $define FIFO_END               := REG2ID(GFX_END)}
{ $define FIFO_FLUSH             := REG2ID(GFX_FLUSH)} 
{ $define FIFO_VIEWPORT          := REG2ID(GFX_VIEWPORT)}
{$endif NDS_IMPLEMENTATION}

{$ifdef NDS_INTERFACE}


//---------------------------------------------------------------------------------
//float wrappers for porting
//---------------------------------------------------------------------------------

procedure glRotatef32i(angle: cint; x, y, z: cint32); cdecl; external;
function glTexImage2D(target, empty1: cint; _type: GL_TEXTURE_TYPE_ENUM; sizeX, sizeY, empty2, param: cint; texture: pcuint8): cint; cdecl; external;
procedure glTexLoadPal(const pal: pcuint16; count: cuint16; addr: cuint32); cdecl; external;
function gluTexLoadPal(const pal: pcuint16; count: cuint16; format: cuint8): cint; cdecl; external;
procedure glTexParameter(sizeX, sizeY: cuint8; const addr: pcuint32; mode: GL_TEXTURE_TYPE_ENUM; param: cuint32); cdecl; external;
function glGetTexParameter(): cuint32; cdecl; external;
function glGetTexturePointer(name: cint): pointer; cdecl; external;
procedure glBindTexture(target, name: cint); cdecl; external;
procedure glColorTable(format: cuint8; addr: cuint32); cdecl; external;
function glGenTextures(n: cint; names: pcint): cint; cdecl; external;
procedure glResetTextures(); cdecl; external;
procedure glTexCoord2f32(u, v: cint32); cdecl; external;
procedure glMaterialf(mode: GL_MATERIALS_ENUM; color: rgb); cdecl; external;
procedure glInit_C(); cdecl; external;
function glGetGlobals(): PGLHiddenGlobals; cdecl; external;
{$endif NDS_INTERFACE}
		
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2




//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
////////////                                                                  ////////////
////////////                                                                  ////////////
////////////                                                                  ////////////
////////////                       INLINED FUNCTIONS                          ////////////
////////////                                                                  ////////////
////////////                                                                  ////////////
////////////                                                                  ////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////


<<<<<<< HEAD
<<<<<<< HEAD
function POLY_ALPHA(n: cuint32): cuint32; inline;
=======
{$ifdef NDS_IMPLEMENTATION}
function POLY_ALPHA(n: cint): cuint32; inline;  
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef NDS_IMPLEMENTATION}
function POLY_ALPHA(n: cint): cuint32; inline;  
>>>>>>> origin/fixes_2_2
begin
  POLY_ALPHA := cuint32((n) shl 16);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function POLY_ID(n: cuint32): cuint32; inline;	
=======
function POLY_ID(n: cint): cuint32; inline;	
>>>>>>> graemeg/fixes_2_2
=======
function POLY_ID(n: cint): cuint32; inline;	
>>>>>>> origin/fixes_2_2
begin
  POLY_ID := cuint32((n) shl 24);
end;

procedure glBegin(mode: GL_GLBEGIN_ENUM); inline;
<<<<<<< HEAD
<<<<<<< HEAD
begin
  GFX_BEGIN^ := mode;
=======
begin 
  GFX_BEGIN^ := mode; 
>>>>>>> graemeg/fixes_2_2
=======
begin 
  GFX_BEGIN^ := mode; 
>>>>>>> origin/fixes_2_2
end;

procedure glEnd(); inline;
begin 
  GFX_END^ := 0;
end;

procedure glClearDepth(depth: fixed12d3); inline;
<<<<<<< HEAD
<<<<<<< HEAD
begin
  GFX_CLEAR_DEPTH^ := depth;
end;

procedure glColor3b(red, green, blue: cuint8); inline;
begin
  GFX_COLOR^ := cuint32(RGB15(red shr 3, green shr 3, blue shr 3));
end;

procedure glColor(color: rgb); inline;
begin
  GFX_COLOR^ := cuint32(color);
end;

procedure glVertex3v16(x, y, z: v16); inline;
begin
	GFX_VERTEX16^ := cuint32((y shl 16) or (x and $FFFF));
	GFX_VERTEX16^ := z;
end;

procedure glTexCoord2t16(u, v: t16); inline; 
begin
  GFX_TEX_COORD^ := TEXTURE_PACK(u, v);
end;

procedure glPushMatrix(); inline; 
begin
  MATRIX_PUSH^ := 0;
end;

procedure glPopMatrix(num: cint32); inline;
begin
  MATRIX_POP^ := num;
end;

procedure glRestoreMatrix(index: cint32); inline;
begin
  MATRIX_RESTORE^ := index;
end;

procedure glStoreMatrix(index: cint32); inline;
begin
  MATRIX_STORE^ := index;
end;

procedure glScalev(const v: PGLvector); inline;
begin
=======
=======
>>>>>>> origin/fixes_2_2
begin 
  GFX_CLEAR_DEPTH^ := depth; 
end;

procedure glColor3b(red, green, blue: cuint8); inline;
begin 
  GFX_COLOR^ := cuint32(RGB15(red shr 3, green shr 3, blue shr 3)); 
end;

procedure glColor(color: rgb); inline;
begin 
  GFX_COLOR^ := cuint32(color); 
end;

procedure glVertex3v16(x, y, z: v16); inline;
begin 
	GFX_VERTEX16^ := cuint32((y shl 16) or (x and $FFFF));
	GFX_VERTEX16^ := cuint32(cuint16(z));
end;

procedure glTexCoord2t16(u, v: t16); inline; 
begin 
  GFX_TEX_COORD^ := TEXTURE_PACK(u, v); 
end;

procedure glPushMatrix(); inline; 
begin 
  MATRIX_PUSH^ := 0; 
end;

procedure glPopMatrix(num: cint32); inline; 
begin 
  MATRIX_POP^ := num; 
end;

procedure glRestoreMatrix(index: cint32); inline; 
begin 
  MATRIX_RESTORE^ := index; 
end;

procedure glStoreMatrix(index: cint32); inline; 
begin 
  MATRIX_STORE^ := index; 
end;

procedure glScalev(const v: PGLvector); inline; 
begin  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
	MATRIX_SCALE^ := v^.x;
	MATRIX_SCALE^ := v^.y;
	MATRIX_SCALE^ := v^.z;
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure glTranslatev(const v: PGLvector) ; inline;
begin
=======
procedure glTranslatev(const v: PGLvector) ; inline; 
begin 
>>>>>>> graemeg/fixes_2_2
=======
procedure glTranslatev(const v: PGLvector) ; inline; 
begin 
>>>>>>> origin/fixes_2_2
	MATRIX_TRANSLATE^ := v^.x;
	MATRIX_TRANSLATE^ := v^.y;
	MATRIX_TRANSLATE^ := v^.z;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure glTranslate3f32(x, y, z: cint32); inline;
begin
  glTranslatef32(x, y, z);
end;

procedure glTranslatef32(x, y, z: cint32); inline;
begin
=======
procedure glTranslate3f32(x, y, z: cint32); inline; 
begin 
>>>>>>> graemeg/fixes_2_2
=======
procedure glTranslate3f32(x, y, z: cint32); inline; 
begin 
>>>>>>> origin/fixes_2_2
	MATRIX_TRANSLATE^ := x;
	MATRIX_TRANSLATE^ := y;
	MATRIX_TRANSLATE^ := z;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure glScalef32(x, y, z: cint); inline;
begin
	MATRIX_SCALE^ := x;
	MATRIX_SCALE^ := y;
	MATRIX_SCALE^ := z;
end;

procedure glLight(id: cint; color: rgb; x, y, z: v10); inline;
begin
=======
=======
>>>>>>> origin/fixes_2_2
procedure glScalef32(factor: cint32); inline; 
begin 
	MATRIX_SCALE^ := factor;
	MATRIX_SCALE^ := factor;
	MATRIX_SCALE^ := factor;
end;

procedure glLight(id: cint; color: rgb; x, y, z: v10); inline; 
begin 	
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  id := (id and 3) shl 30;
	GFX_LIGHT_VECTOR^ := id or ((z and $3FF) shl 20) or ((y and $3FF) shl 10) or (x and $3FF);
	GFX_LIGHT_COLOR^ := id or color;
end;

procedure glNormal(normal: cuint32); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  GFX_NORMAL^ := normal;
=======
  GFX_NORMAL^ := normal; 
>>>>>>> graemeg/fixes_2_2
=======
  GFX_NORMAL^ := normal; 
>>>>>>> origin/fixes_2_2
end;

procedure glLoadIdentity(); inline;
begin
  MATRIX_IDENTITY^ := 0;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure glMatrixMode(mode: GL_MATRIX_MODE_ENUM); inline;
begin
  MATRIX_CONTROL^ := mode;
=======
=======
>>>>>>> origin/fixes_2_2
procedure glIdentity(); inline;
begin
  MATRIX_IDENTITY^ := 0; 
end;

procedure glMatrixMode(mode: GL_MATRIX_MODE_ENUM); inline;
begin
  MATRIX_CONTROL^ := mode; 
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure glViewport(x1, y1, x2, y2: cuint8); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  GFX_VIEWPORT^ := (x1) + (y1 shl 8) + (x2 shl 16) + (y2 shl 24);
=======
  GFX_VIEWPORT^ := (x1) + (y1 shl 8) + (x2 shl 16) + (y2 shl 24); 
>>>>>>> graemeg/fixes_2_2
=======
  GFX_VIEWPORT^ := (x1) + (y1 shl 8) + (x2 shl 16) + (y2 shl 24); 
>>>>>>> origin/fixes_2_2
end;

procedure glFlush(mode: cuint32); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  GFX_FLUSH^ := mode;
=======
  GFX_FLUSH^ := mode; 
>>>>>>> graemeg/fixes_2_2
=======
  GFX_FLUSH^ := mode; 
>>>>>>> origin/fixes_2_2
end;

procedure glMaterialShinyness(); inline;
var
<<<<<<< HEAD
<<<<<<< HEAD
  shiny32: array [0..31] of cuint32;
  shiny8: pcuint8;
  i: integer;
begin
  shiny8 := pcuint8(@shiny32);
=======
=======
>>>>>>> origin/fixes_2_2
	shiny32: array [0..31] of cuint32;
	shiny8: pcuint8;
	i: integer;
begin
	shiny8 := pcuint8(@shiny32);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  i := 0;
  while i < (128 * 2) do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    shiny8[i shr 1] := i;
    inc(i, 2);
  end;

  for i := 0 to 31 do
    GFX_SHININESS^ := shiny32[i];
end;

=======
=======
>>>>>>> origin/fixes_2_2
		shiny8[i shr 1] := i;
		inc(i, 2);
	end;

	for i := 0 to 31 do
		GFX_SHININESS[i] := shiny32[i];
end;


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure glCallList(list: pcuint32); inline;
var
  count: cuint32;
begin
<<<<<<< HEAD
<<<<<<< HEAD
	sassert(list <> nil, 'glCallList received a null display list pointer');
  count := list^;
	sassert(count <> 0, 'glCallList received a display list of size 0');
  inc(list);
  DC_FlushRange(list, count*4);
  while ( ((DMA_CR(0)^ and DMA_BUSY) <> 0) or 
          ((DMA_CR(1)^ and DMA_BUSY) <> 0) or 
          ((DMA_CR(2)^ and DMA_BUSY) <> 0) or 
          ((DMA_CR(3)^ and DMA_BUSY) <> 0)) do;

  DMA_SRC(0)^ := cuint32(list);
  DMA_DEST(0)^ := $4000400;
  DMA_CR(0)^ := DMA_FIFO or count;
  while (DMA_CR(0)^ and DMA_BUSY) <> 0 do;
end;

procedure glPolyFmt(params: cuint32); inline;
=======
=======
>>>>>>> origin/fixes_2_2
	count := list^;
	inc(list);
  DC_FlushRange(list, count*4);
	while ( ((DMA_CR(0)^ and DMA_BUSY) <> 0) or 
				  ((DMA_CR(1)^ and DMA_BUSY) <> 0) or 
				  ((DMA_CR(2)^ and DMA_BUSY) <> 0) or 
				  ((DMA_CR(3)^ and DMA_BUSY) <> 0)) do;

	DMA_SRC(0)^ := cuint32(list);
	DMA_DEST(0)^ := $4000400;
	DMA_CR(0)^ := DMA_FIFO or count;
	while (DMA_CR(0)^ and DMA_BUSY) <> 0 do;
end;

procedure glPolyFmt(params: cuint32); inline; 
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  GFX_POLY_FORMAT^ := params;
end;
////////////////////////////////////////////////////////////////

procedure glEnable(bits: cint); inline;
begin
  GFX_CONTROL^ := GFX_CONTROL^ or bits;
end;

procedure glDisable(bits: cint); inline;
begin
  GFX_CONTROL^ := GFX_CONTROL^ and not bits;
end;

<<<<<<< HEAD
<<<<<<< HEAD

procedure glFogShift(shift: cint); inline;
begin
  sassert((shift >= 0) and (shift < 16), 'glFogShift is out of range');
  GFX_CONTROL^ := (GFX_CONTROL^ and $F0FF) or (shift shl 8);
end;

procedure glFogOffset(offset: cint); inline; 
begin
  sassert((offset >= 0) and (offset < $8000), 'glFogOffset is out of range');
  GFX_FOG_OFFSET^ := offset;
end;

procedure glFogColor(red, green, blue, alpha: cuint8); inline;
begin
  sassert(red < 32, 'glFogColor red is out of range');
  sassert(green < 32, 'glFogColor green is out of range');
  sassert(blue < 32, 'glFogColor blue is out of range');
  sassert(alpha < 32, 'glFogColor alpha is out of range');
  GFX_FOG_COLOR^ := RGB15(red, green, blue) or (alpha shl 16);  
end;

procedure glFogDensity(index, density: cint); inline;
begin
  sassert((index >= 0) and (index < 32), 'glFogDensity index is out of range');
  sassert((index >= 0) and (density < 128), 'glFogDensity density is out of range');
  GFX_FOG_TABLE[index] := density;  
end;

procedure glLoadMatrix4x4(const m: pm4x4); inline;
begin
  MATRIX_LOAD4x4^ := m^.m[0];
  MATRIX_LOAD4x4^ := m^.m[1];
  MATRIX_LOAD4x4^ := m^.m[2];
  MATRIX_LOAD4x4^ := m^.m[3];

  MATRIX_LOAD4x4^ := m^.m[4];
  MATRIX_LOAD4x4^ := m^.m[5];
  MATRIX_LOAD4x4^ := m^.m[6];
  MATRIX_LOAD4x4^ := m^.m[7];

  MATRIX_LOAD4x4^ := m^.m[8];
  MATRIX_LOAD4x4^ := m^.m[9];
  MATRIX_LOAD4x4^ := m^.m[10];
  MATRIX_LOAD4x4^ := m^.m[11];

  MATRIX_LOAD4x4^ := m^.m[12];
  MATRIX_LOAD4x4^ := m^.m[13];
  MATRIX_LOAD4x4^ := m^.m[14];
  MATRIX_LOAD4x4^ := m^.m[15];
end;

procedure glLoadMatrix4x3(const m: pm4x3); inline;
begin
  MATRIX_LOAD4x3^ := m^.m[0];
  MATRIX_LOAD4x3^ := m^.m[1];
  MATRIX_LOAD4x3^ := m^.m[2];
  MATRIX_LOAD4x3^ := m^.m[3];

  MATRIX_LOAD4x3^ := m^.m[4];
  MATRIX_LOAD4x3^ := m^.m[5];
  MATRIX_LOAD4x3^ := m^.m[6];
  MATRIX_LOAD4x3^ := m^.m[7];

  MATRIX_LOAD4x3^ := m^.m[8];
  MATRIX_LOAD4x3^ := m^.m[9];
  MATRIX_LOAD4x3^ := m^.m[10];
  MATRIX_LOAD4x3^ := m^.m[11];
end;

procedure glMultMatrix4x4(const m: pm4x4); inline;
begin
  MATRIX_MULT4x4^ := m^.m[0];
  MATRIX_MULT4x4^ := m^.m[1];
  MATRIX_MULT4x4^ := m^.m[2];
  MATRIX_MULT4x4^ := m^.m[3];

  MATRIX_MULT4x4^ := m^.m[4];
  MATRIX_MULT4x4^ := m^.m[5];
  MATRIX_MULT4x4^ := m^.m[6];
  MATRIX_MULT4x4^ := m^.m[7];

  MATRIX_MULT4x4^ := m^.m[8];
  MATRIX_MULT4x4^ := m^.m[9];
  MATRIX_MULT4x4^ := m^.m[10];
  MATRIX_MULT4x4^ := m^.m[11];

  MATRIX_MULT4x4^ := m^.m[12];
  MATRIX_MULT4x4^ := m^.m[13];
  MATRIX_MULT4x4^ := m^.m[14];
  MATRIX_MULT4x4^ := m^.m[15];
=======
=======
>>>>>>> origin/fixes_2_2
procedure glLoadMatrix4x4(const m: pm4x4); inline;
begin
	MATRIX_LOAD4x4^ := m^.m[0];
	MATRIX_LOAD4x4^ := m^.m[1];
	MATRIX_LOAD4x4^ := m^.m[2];
	MATRIX_LOAD4x4^ := m^.m[3];
	
	MATRIX_LOAD4x4^ := m^.m[4];
	MATRIX_LOAD4x4^ := m^.m[5];
	MATRIX_LOAD4x4^ := m^.m[6];
	MATRIX_LOAD4x4^ := m^.m[7];
	
	MATRIX_LOAD4x4^ := m^.m[8];
	MATRIX_LOAD4x4^ := m^.m[9];
	MATRIX_LOAD4x4^ := m^.m[10];
	MATRIX_LOAD4x4^ := m^.m[11];
	
	MATRIX_LOAD4x4^ := m^.m[12];
	MATRIX_LOAD4x4^ := m^.m[13];
	MATRIX_LOAD4x4^ := m^.m[14];
	MATRIX_LOAD4x4^ := m^.m[15];
end;


procedure glLoadMatrix4x3(const m: pm4x3); inline;
begin
	MATRIX_LOAD4x3^ := m^.m[0];
	MATRIX_LOAD4x3^ := m^.m[1];
	MATRIX_LOAD4x3^ := m^.m[2];
	MATRIX_LOAD4x3^ := m^.m[3];
	
	MATRIX_LOAD4x3^ := m^.m[4];
	MATRIX_LOAD4x3^ := m^.m[5];
	MATRIX_LOAD4x3^ := m^.m[6];
	MATRIX_LOAD4x3^ := m^.m[7];
	
	MATRIX_LOAD4x3^ := m^.m[8];
	MATRIX_LOAD4x3^ := m^.m[9];
	MATRIX_LOAD4x3^ := m^.m[10];
	MATRIX_LOAD4x3^ := m^.m[11];
end;


procedure glMultMatrix4x4(const m: pm4x4); inline;
begin
	MATRIX_MULT4x4^ := m^.m[0];
	MATRIX_MULT4x4^ := m^.m[1];
	MATRIX_MULT4x4^ := m^.m[2];
	MATRIX_MULT4x4^ := m^.m[3];
	
	MATRIX_MULT4x4^ := m^.m[4];
	MATRIX_MULT4x4^ := m^.m[5];
	MATRIX_MULT4x4^ := m^.m[6];
	MATRIX_MULT4x4^ := m^.m[7];
	
	MATRIX_MULT4x4^ := m^.m[8];
	MATRIX_MULT4x4^ := m^.m[9];
	MATRIX_MULT4x4^ := m^.m[10];
	MATRIX_MULT4x4^ := m^.m[11];
	
	MATRIX_MULT4x4^ := m^.m[12];
	MATRIX_MULT4x4^ := m^.m[13];
	MATRIX_MULT4x4^ := m^.m[14];
	MATRIX_MULT4x4^ := m^.m[15];
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure glMultMatrix4x3(const m: pm4x3); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  MATRIX_MULT4x3^ := m^.m[0];
  MATRIX_MULT4x3^ := m^.m[1];
  MATRIX_MULT4x3^ := m^.m[2];
  MATRIX_MULT4x3^ := m^.m[3];

  MATRIX_MULT4x3^ := m^.m[4];
  MATRIX_MULT4x3^ := m^.m[5];
  MATRIX_MULT4x3^ := m^.m[6];
  MATRIX_MULT4x3^ := m^.m[7];

  MATRIX_MULT4x3^ := m^.m[8];
  MATRIX_MULT4x3^ := m^.m[9];
  MATRIX_MULT4x3^ := m^.m[10];
  MATRIX_MULT4x3^ := m^.m[11];
=======
=======
>>>>>>> origin/fixes_2_2
	MATRIX_MULT4x3^ := m^.m[0];
	MATRIX_MULT4x3^ := m^.m[1];
	MATRIX_MULT4x3^ := m^.m[2];
	MATRIX_MULT4x3^ := m^.m[3];
	
	MATRIX_MULT4x3^ := m^.m[4];
	MATRIX_MULT4x3^ := m^.m[5];
	MATRIX_MULT4x3^ := m^.m[6];
	MATRIX_MULT4x3^ := m^.m[7];
	
	MATRIX_MULT4x3^ := m^.m[8];
	MATRIX_MULT4x3^ := m^.m[9];
	MATRIX_MULT4x3^ := m^.m[10];
	MATRIX_MULT4x3^ := m^.m[11];
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure glMultMatrix3x3(const m: pm3x3); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  MATRIX_MULT3x3^ := m^.m[0];
  MATRIX_MULT3x3^ := m^.m[1];
  MATRIX_MULT3x3^ := m^.m[2];

  MATRIX_MULT3x3^ := m^.m[3];
  MATRIX_MULT3x3^ := m^.m[4];
  MATRIX_MULT3x3^ := m^.m[5];

  MATRIX_MULT3x3^ := m^.m[6];
  MATRIX_MULT3x3^ := m^.m[7];
  MATRIX_MULT3x3^ := m^.m[8];
=======
=======
>>>>>>> origin/fixes_2_2
	MATRIX_MULT3x3^ := m^.m[0];
	MATRIX_MULT3x3^ := m^.m[1];
	MATRIX_MULT3x3^ := m^.m[2];
	
	MATRIX_MULT3x3^ := m^.m[3];
	MATRIX_MULT3x3^ := m^.m[4];
	MATRIX_MULT3x3^ := m^.m[5];
	
	MATRIX_MULT3x3^ := m^.m[6];
	MATRIX_MULT3x3^ := m^.m[7];
	MATRIX_MULT3x3^ := m^.m[8];
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure glRotateXi(angle: cint); inline;
var
  sine, cosine: cint32;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  sine := sinLerp(angle);
  cosine := cosLerp(angle);

  MATRIX_MULT3x3^ := inttof32(1);
  MATRIX_MULT3x3^ := 0;
  MATRIX_MULT3x3^ := 0;

  MATRIX_MULT3x3^ := 0;
  MATRIX_MULT3x3^ := cosine;
  MATRIX_MULT3x3^ := sine;

  MATRIX_MULT3x3^ := 0;
  MATRIX_MULT3x3^ := -sine;
  MATRIX_MULT3x3^ := cosine;
=======
=======
>>>>>>> origin/fixes_2_2
	sine := _SIN[angle and  LUT_MASK];
	cosine := _COS[angle and LUT_MASK];
	
	MATRIX_MULT3x3^ := inttof32(1);
	MATRIX_MULT3x3^ := 0;
	MATRIX_MULT3x3^ := 0;
	
	MATRIX_MULT3x3^ := 0;
	MATRIX_MULT3x3^ := cosine;
	MATRIX_MULT3x3^ := sine;
	
	MATRIX_MULT3x3^ := 0;
	MATRIX_MULT3x3^ := -sine;
	MATRIX_MULT3x3^ := cosine;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure glRotateYi(angle: cint); inline;
var
  sine, cosine: cint32;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  sine := sinLerp(angle);
  cosine := cosLerp(angle);

  MATRIX_MULT3x3^ := cosine;
  MATRIX_MULT3x3^ := 0;
  MATRIX_MULT3x3^ := -sine;

  MATRIX_MULT3x3^ := 0;
  MATRIX_MULT3x3^ := inttof32(1);
  MATRIX_MULT3x3^ := 0;

  MATRIX_MULT3x3^ := sine;
  MATRIX_MULT3x3^ := 0;
  MATRIX_MULT3x3^ := cosine;
=======
=======
>>>>>>> origin/fixes_2_2
	sine := _SIN[angle and  LUT_MASK];
	cosine := _COS[angle and LUT_MASK];
	
	MATRIX_MULT3x3^ := cosine;
	MATRIX_MULT3x3^ := 0;
	MATRIX_MULT3x3^ := -sine;
	
	MATRIX_MULT3x3^ := 0;
	MATRIX_MULT3x3^ := inttof32(1);
	MATRIX_MULT3x3^ := 0;
	
	MATRIX_MULT3x3^ := sine;
	MATRIX_MULT3x3^ := 0;
	MATRIX_MULT3x3^ := cosine;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure glRotateZi(angle: cint); inline;
var
  sine, cosine: cint32;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  sine := sinLerp(angle);
  cosine := cosLerp(angle);

  MATRIX_MULT3x3^ := cosine;
  MATRIX_MULT3x3^ := sine;
  MATRIX_MULT3x3^ := 0;

  MATRIX_MULT3x3^ := -sine;
  MATRIX_MULT3x3^ := cosine;
  MATRIX_MULT3x3^ := 0;

  MATRIX_MULT3x3^ := 0;
  MATRIX_MULT3x3^ := 0;
  MATRIX_MULT3x3^ := inttof32(1);
=======
=======
>>>>>>> origin/fixes_2_2
	sine := _SIN[angle and  LUT_MASK];
	cosine := _COS[angle and LUT_MASK];
	
	MATRIX_MULT3x3^ := cosine;
	MATRIX_MULT3x3^ := sine;
	MATRIX_MULT3x3^ := 0;
	
	MATRIX_MULT3x3^ := -sine;
	MATRIX_MULT3x3^ := cosine;
	MATRIX_MULT3x3^ := 0;
	
	MATRIX_MULT3x3^ := 0;
	MATRIX_MULT3x3^ := 0;
	MATRIX_MULT3x3^ := inttof32(1);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure glOrthof32(left, right, bottom, top, zNear, zFar: cint32); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  MATRIX_MULT4x4^ := divf32(inttof32(2), right - left);
  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := 0;

  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := divf32(inttof32(2), top - bottom);
  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := 0;

  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := divf32(inttof32(-2), zFar - zNear);
  MATRIX_MULT4x4^ := 0;

  MATRIX_MULT4x4^ := -divf32(right + left, right - left);
  MATRIX_MULT4x4^ := -divf32(top + bottom, top - bottom);
  MATRIX_MULT4x4^ := -divf32(zFar + zNear, zFar - zNear);
  MATRIX_MULT4x4^ := floattof32(1.0);
=======
=======
>>>>>>> origin/fixes_2_2
	MATRIX_MULT4x4^ := divf32(inttof32(2), right - left);     
	MATRIX_MULT4x4^ := 0;  
	MATRIX_MULT4x4^ := 0;      
	MATRIX_MULT4x4^ := 0;

	MATRIX_MULT4x4^ := 0;  
	MATRIX_MULT4x4^ := divf32(inttof32(2), top - bottom);     
	MATRIX_MULT4x4^ := 0;    
	MATRIX_MULT4x4^ := 0;

	MATRIX_MULT4x4^ := 0;  
	MATRIX_MULT4x4^ := 0;  
	MATRIX_MULT4x4^ := divf32(inttof32(-2), zFar - zNear);     
	MATRIX_MULT4x4^ := 0;

	MATRIX_MULT4x4^ := -divf32(right + left, right - left);//0;  
	MATRIX_MULT4x4^ := -divf32(top + bottom, top - bottom); //0;  
	MATRIX_MULT4x4^ := -divf32(zFar + zNear, zFar - zNear);//0;  
	MATRIX_MULT4x4^ := floattof32(1.0);

	//glStoreMatrix(0);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure gluLookAtf32(eyex, eyey, eyez, lookAtx, lookAty, lookAtz, upx, upy, upz: cint32); inline;
var
  side, forwrd, up, eye: array [0..2] of cint32;
begin
<<<<<<< HEAD
<<<<<<< HEAD

  forwrd[0] := eyex - lookAtx;
  forwrd[1] := eyey - lookAty;
  forwrd[2] := eyez - lookAtz;

  normalizef32(@forwrd);

  up[0] := upx;
  up[1] := upy;
  up[2] := upz;
  eye[0] := eyex;
  eye[1] := eyey;
  eye[2] := eyez;

  crossf32(@up, @forwrd, @side);

  normalizef32(@side);

  // Recompute local up
  crossf32(@forwrd, @side, @up);

  glMatrixMode(GL_MODELVIEW);


  // should we use MATRIX_MULT4x3?
  MATRIX_MULT4x3^ := side[0];
  MATRIX_MULT4x3^ := up[0];
  MATRIX_MULT4x3^ := forwrd[0];

  MATRIX_MULT4x3^ := side[1];
  MATRIX_MULT4x3^ := up[1];
  MATRIX_MULT4x3^ := forwrd[1];

  MATRIX_MULT4x3^ := side[2];
  MATRIX_MULT4x3^ := up[2];
  MATRIX_MULT4x3^ := forwrd[2];

  MATRIX_MULT4x3^ := -dotf32(@eye,@side);
  MATRIX_MULT4x3^ := -dotf32(@eye,@up);
  MATRIX_MULT4x3^ := -dotf32(@eye,@forwrd);
=======
=======
>>>>>>> origin/fixes_2_2
	
	forwrd[0] := eyex - lookAtx; 
	forwrd[1] := eyey - lookAty; 
	forwrd[2] := eyez - lookAtz; 
	
	normalizef32(@forwrd); 
	
	up[0] := upx; 
	up[1] := upy; 
	up[2] := upz; 
	eye[0] := eyex; 
	eye[1] := eyey; 
	eye[2] := eyez; 
	
	crossf32(@up, @forwrd, @side); 
	
	normalizef32(@side); 
	
	// Recompute local up
	crossf32(@forwrd, @side, @up);
	
	glMatrixMode(GL_MODELVIEW); 
	
	
	// should we use MATRIX_MULT4x3? 
	MATRIX_MULT4x3^ := side[0]; 
	MATRIX_MULT4x3^ := up[0]; 
	MATRIX_MULT4x3^ := forwrd[0]; 
	
	MATRIX_MULT4x3^ := side[1]; 
	MATRIX_MULT4x3^ := up[1]; 
	MATRIX_MULT4x3^ := forwrd[1]; 
	
	MATRIX_MULT4x3^ := side[2]; 
	MATRIX_MULT4x3^ := up[2]; 
	MATRIX_MULT4x3^ := forwrd[2]; 
	
	MATRIX_MULT4x3^ := -dotf32(@eye,@side); 
	MATRIX_MULT4x3^ := -dotf32(@eye,@up); 
	MATRIX_MULT4x3^ := -dotf32(@eye,@forwrd); 
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure glFrustumf32(left, right, bottom, top, zNear, zFar: cint32); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
(*
	MATRIX_MULT4x4^ := divf32(2*znear, right - left);
	MATRIX_MULT4x4^ := 0;
	MATRIX_MULT4x4^ := divf32(right + left, right - left);
	MATRIX_MULT4x4^ := 0;
	
	MATRIX_MULT4x4^ := 0;
	MATRIX_MULT4x4^ := divf32(2*znear, top - bottom);
	MATRIX_MULT4x4^ := divf32(top + bottom, top - bottom);
	MATRIX_MULT4x4^ := 0;
	
	MATRIX_MULT4x4^ := 0;
	MATRIX_MULT4x4^ := 0;
	MATRIX_MULT4x4^ := -divf32(zfar + znear, zfar - znear);
	MATRIX_MULT4x4^ := floattof32(-1.0);

	MATRIX_MULT4x4^ := 0;
	MATRIX_MULT4x4^ := 0;
	MATRIX_MULT4x4^ := -divf32(2 * mulf32(zfar, znear), zfar - znear);
	MATRIX_MULT4x4^ := 0;
*)	
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  MATRIX_MULT4x4^ := divf32(2*znear, right - left);
  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := 0;

  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := divf32(2*znear, top - bottom);
  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := 0;

  MATRIX_MULT4x4^ := divf32(right + left, right - left);
  MATRIX_MULT4x4^ := divf32(top + bottom, top - bottom);
  MATRIX_MULT4x4^ := -divf32(zfar + znear, zfar - znear);
  MATRIX_MULT4x4^ := floattof32(-1.0);

  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := -divf32(2 * mulf32(zfar, znear), zfar - znear);
  MATRIX_MULT4x4^ := 0;
end;

procedure gluPerspectivef32(fovy: cint; aspect, zNear, zFar: cint32); inline;
var
  xmin, xmax, ymin, ymax: cint32;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  ymax := mulf32(zNear, tanLerp(fovy shr 1));
  ymin := -ymax;
  xmin := mulf32(ymin, aspect);
  xmax := mulf32(ymax, aspect);

  glFrustumf32(xmin, xmax, ymin, ymax, zNear, zFar);
=======
=======
>>>>>>> origin/fixes_2_2
  ymax := mulf32(zNear, TAN_bin[(fovy shr 1) and LUT_MASK]);
	ymin := -ymax;
	xmin := mulf32(ymin, aspect);
	xmax := mulf32(ymax, aspect);
	
	glFrustumf32(xmin, xmax, ymin, ymax, zNear, zFar);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure glTexCoord2f(s, t: cfloat); inline;
var
  x, y: cint;
<<<<<<< HEAD
<<<<<<< HEAD
  tex: pgl_texture_data;
begin
  tex := pgl_texture_data(DynamicArrayGet(@(glGlob^.texturePtrs), glGlob^.activeTexture));
  if tex <> nil then
  begin
    x := (tex^.texFormat shr 20) and 7;
    y := (tex^.texFormat shr 23) and 7;
    glTexCoord2t16(floattot16(s*(8 shl x)), floattot16(t*(8 shl y)));
  end;
end;

procedure gluPickMatrix(x, y, width, height: cint32; const viewport: TArr4ofInt); inline;
begin
  MATRIX_MULT4x4^ := inttof32(viewport[2]) div width;
  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := 0;

  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := inttof32(viewport[3]) div height;
  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := 0;

  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := inttof32(1);
  MATRIX_MULT4x4^ := 0;

  MATRIX_MULT4x4^ := inttof32(viewport[2] + ((viewport[0] - x) shl 1)) div width;
  MATRIX_MULT4x4^ := inttof32(viewport[3] + ((viewport[1] - y) shl 1)) div height;
  MATRIX_MULT4x4^ := 0;
  MATRIX_MULT4x4^ := inttof32(1);
=======
=======
>>>>>>> origin/fixes_2_2
begin
  x := ((glGlob^.textures[glGlob^.activeTexture]) shr 20) and 7; 
	y := ((glGlob^.textures[glGlob^.activeTexture]) shr 23) and 7; 
	
	glTexCoord2t16(floattot16(s*(8 shl x)), floattot16(t*(8 shl y)));
end;

{$endif NDS_IMPLEMENTATION}


{$ifdef NDS_INTERFACE}
type 
  TArr4ofInt = array [0..3] of cint32;
{$endif NDS_INTERFACE}


{$ifdef NDS_IMPLEMENTATION}
procedure gluPickMatrix(x, y, width, height: cint32; const viewport: TArr4ofInt); inline;
begin
	MATRIX_MULT4x4^ := inttof32(viewport[2]) div width;
	MATRIX_MULT4x4^ := 0;
	MATRIX_MULT4x4^ := 0;
	MATRIX_MULT4x4^ := 0;

	MATRIX_MULT4x4^ := 0;
	MATRIX_MULT4x4^ := inttof32(viewport[3]) div height;
	MATRIX_MULT4x4^ := 0;
	MATRIX_MULT4x4^ := 0;

	MATRIX_MULT4x4^ := 0;
	MATRIX_MULT4x4^ := 0;
	MATRIX_MULT4x4^ := inttof32(1);
	MATRIX_MULT4x4^ := 0;

	MATRIX_MULT4x4^ := inttof32(viewport[2] + ((viewport[0] - x) shl 1)) div width;
	MATRIX_MULT4x4^ := inttof32(viewport[3] + ((viewport[1] - y) shl 1)) div height;
	MATRIX_MULT4x4^ := 0;
	MATRIX_MULT4x4^ := inttof32(1);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure glResetMatrixStack(); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  // make sure there are no push/pops that haven't executed yet
  while (GFX_STATUS^ and (1 shl 14)) <> 0 do
  begin
    GFX_STATUS^ := GFX_STATUS^ or (1 shl 15); // clear push/pop errors or push/pop busy bit never clears
  end;

  // pop the projection stack to the top; poping 0 off an empty stack causes an error... weird?
  if  (GFX_STATUS^ and (1 shl 13)) <> 0 then
  begin
    glMatrixMode(GL_PROJECTION);
    glPopMatrix(1);
  end;

  // 31 deep modelview matrix; 32nd entry works but sets error flag
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix((GFX_STATUS^ shr 8) and $1F);

  // load identity to all the matrices
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glMatrixMode(GL_TEXTURE);
  glLoadIdentity();	
=======
=======
>>>>>>> origin/fixes_2_2
	// make sure there are no push/pops that haven't executed yet
	while (GFX_STATUS^ and (1 shl 14)) <> 0 do
	begin
		GFX_STATUS^ := GFX_STATUS^ or (1 shl 15); // clear push/pop errors or push/pop busy bit never clears
	end;
	
	// pop the projection stack to the top; poping 0 off an empty stack causes an error... weird?
	if  (GFX_STATUS^ and (1 shl 13)) <> 0 then
	begin
		glMatrixMode(GL_PROJECTION);
		glPopMatrix(1);
	end;
	
	// 31 deep modelview matrix; 32nd entry works but sets error flag
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix((GFX_STATUS^ shr 8) and $1F);
	
	// load identity to all the matrices
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();	
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure glSetOutlineColor(id: cint; color: rgb); inline;
begin
  GFX_EDGE_TABLE[id] := cuint16(color);
end;

procedure glSetToonTable(const table: pcuint16); inline;
var
  i: integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  for i := 0 to 31 do
    GFX_TOON_TABLE[i] := table[i];
=======
	for i := 0 to 31 do
		GFX_TOON_TABLE[i] := table[i];
>>>>>>> graemeg/fixes_2_2
=======
	for i := 0 to 31 do
		GFX_TOON_TABLE[i] := table[i];
>>>>>>> origin/fixes_2_2
end;

procedure glSetToonTableRange(_start, _end: cint; color: rgb); inline;
var
  i: integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  for i := _start to _end do
    GFX_TOON_TABLE[i] := cuint16(color);
=======
	for i := _start to _end do
		GFX_TOON_TABLE[i] := cuint16(color);
>>>>>>> graemeg/fixes_2_2
=======
	for i := _start to _end do
		GFX_TOON_TABLE[i] := cuint16(color);
>>>>>>> origin/fixes_2_2
end;

procedure glGetFixed(const param: GL_GET_ENUM; f: pcint32); inline;
var
  i: integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  case param of 
    GL_GET_MATRIX_VECTOR:
      begin
        while (GFX_BUSY) do; // wait until the graphics engine has stopped to read matrixes
        for i := 0 to 8 do 
          f[i] := MATRIX_READ_VECTOR[i];
      end;
    GL_GET_MATRIX_CLIP:
      begin
        while (GFX_BUSY) do; // wait until the graphics engine has stopped to read matrixes
        for i := 0 to 15 do 
          f[i] := MATRIX_READ_CLIP[i];
      end;
    GL_GET_MATRIX_PROJECTION:
      begin
        glMatrixMode(GL_POSITION);
        glPushMatrix(); // save the current state of the position matrix
        glLoadIdentity(); // load an identity matrix into the position matrix so that the modelview matrix = projection matrix
        while (GFX_BUSY) do; // wait until the graphics engine has stopped to read matrixes
        for i := 0 to 15 do
          f[i] := MATRIX_READ_CLIP[i]; // read out the projection matrix
        glPopMatrix(1); // restore the position matrix
      end;
    GL_GET_MATRIX_POSITION:
      begin
        glMatrixMode(GL_PROJECTION);
        glPushMatrix(); // save the current state of the projection matrix
        glLoadIdentity(); // load a identity matrix into the projection matrix so that the modelview matrix = position matrix
        while (GFX_BUSY) do; // wait until the graphics engine has stopped to read matrixes
        for i := 0 to 15 do
          f[i] := MATRIX_READ_CLIP[i]; // read out the position matrix
        glPopMatrix(1); // restore the projection matrix
      end;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
	case param of 
		GL_GET_MATRIX_VECTOR:
		  begin
				while (GFX_BUSY) do; // wait until the graphics engine has stopped to read matrixes
				for i := 0 to 8 do 
				  f[i] := MATRIX_READ_VECTOR[i];
			end;
		GL_GET_MATRIX_CLIP:
		  begin
			  while (GFX_BUSY) do; // wait until the graphics engine has stopped to read matrixes
				for i := 0 to 15 do 
				  f[i] := MATRIX_READ_CLIP[i];
			end;
		GL_GET_MATRIX_PROJECTION:
		  begin
				glMatrixMode(GL_POSITION);
				glPushMatrix(); // save the current state of the position matrix
				glLoadIdentity(); // load an identity matrix into the position matrix so that the modelview matrix = projection matrix
				while (GFX_BUSY) do; // wait until the graphics engine has stopped to read matrixes
				for i := 0 to 15 do
				  f[i] := MATRIX_READ_CLIP[i]; // read out the projection matrix
				glPopMatrix(1); // restore the position matrix
			end;
		GL_GET_MATRIX_POSITION:
		  begin
				glMatrixMode(GL_PROJECTION);
				glPushMatrix(); // save the current state of the projection matrix
				glLoadIdentity(); // load a identity matrix into the projection matrix so that the modelview matrix = position matrix
				while (GFX_BUSY) do; // wait until the graphics engine has stopped to read matrixes
				for i := 0 to 15 do
				  f[i] := MATRIX_READ_CLIP[i]; // read out the position matrix
				glPopMatrix(1); // restore the projection matrix
			end;
	end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure glAlphaFunc(alphaThreshold: cint); inline;
begin
  GFX_ALPHA_TEST^ := alphaThreshold;
end;

procedure glCutoffDepth(wVal: fixed12d3); inline;
begin
  GFX_CUTOFF_DEPTH^ := wVal;
end;

procedure glInit(); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  glInit_C(); // actually does the initialization
=======
  glGlob := glGetGlobals(); // make sure globals are synced between compilation units
	glInit_C(); // actually does the initialization
>>>>>>> graemeg/fixes_2_2
=======
  glGlob := glGetGlobals(); // make sure globals are synced between compilation units
	glInit_C(); // actually does the initialization
>>>>>>> origin/fixes_2_2
end;

procedure glClearColor(red, green, blue, alpha: cuint8); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  glGlob^.clearColor := (glGlob^.clearColor and $FFE08000) or ($7FFF and RGB15(red, green, blue)) or ((alpha and $1F) shl 16);
  GFX_CLEAR_COLOR^ := glGlob^.clearColor;
=======
	GFX_CLEAR_COLOR^ := (glGlob^.clearColor and $FFE08000) or ($7FFF and RGB15(red, green, blue)) or ((alpha and $1F) shl 16);
  glGlob^.clearColor := GFX_CLEAR_COLOR^; 
>>>>>>> graemeg/fixes_2_2
=======
	GFX_CLEAR_COLOR^ := (glGlob^.clearColor and $FFE08000) or ($7FFF and RGB15(red, green, blue)) or ((alpha and $1F) shl 16);
  glGlob^.clearColor := GFX_CLEAR_COLOR^; 
>>>>>>> origin/fixes_2_2
end;

procedure glClearPolyID(ID: cuint8); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  glGlob^.clearColor  := ( glGlob^.clearColor and $C0FFFFFF) or (( ID and $3F ) shl 24 );
  GFX_CLEAR_COLOR^ := glGlob^.clearColor; 
end;

procedure glGetInt(param: GL_GET_ENUM; var i: cint); inline;
var
  tex: pgl_texture_data; 
begin
  case param of
    GL_GET_POLYGON_RAM_COUNT:
      i := GFX_POLYGON_RAM_USAGE^;
    GL_GET_VERTEX_RAM_COUNT:
      i := GFX_VERTEX_RAM_USAGE^;
    GL_GET_TEXTURE_WIDTH:
      begin 
        tex := pgl_texture_data(DynamicArrayGet(@(glGlob^.texturePtrs), glGlob^.activeTexture));
        if tex <> nil then
          i := 8 shl ((tex^.texFormat shr 20) and 7);			
      end;
    GL_GET_TEXTURE_HEIGHT:
      begin
        tex := pgl_texture_data(DynamicArrayGet(@(glGlob^.texturePtrs), glGlob^.activeTexture));
        if tex <> nil then
          i := 8 shl ((tex^.texFormat shr 23 ) and 7);
      end;      
=======
=======
>>>>>>> origin/fixes_2_2
	GFX_CLEAR_COLOR^ := ( glGlob^.clearColor and $C0FFFFFF) or (( ID and $3F ) shl 24 );
	glGlob^.clearColor := GFX_CLEAR_COLOR^; 
end;

procedure glGetInt(param: GL_GET_ENUM; var i: cint); inline; 
begin
  case param of
		GL_GET_POLYGON_RAM_COUNT:
			i := GFX_POLYGON_RAM_USAGE^;
		GL_GET_VERTEX_RAM_COUNT:
			i := GFX_VERTEX_RAM_USAGE^;
    GL_GET_TEXTURE_WIDTH:
			i := 8 shl (((glGlob^.textures[glGlob^.activeTexture]) shr 20) and 7);
    GL_GET_TEXTURE_HEIGHT:
			i := 8 shl (((glGlob^.textures[glGlob^.activeTexture]) shr 23) and 7);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
end;

//---------------------------------------------------------------------------------
//                    INLINED FlOAT WRAPPERS

procedure glVertex3f(x, y, z: cfloat); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  glVertex3v16(floattov16(x), floattov16(y), floattov16(z));
=======
	glVertex3v16(floattov16(x), floattov16(y), floattov16(z));
>>>>>>> graemeg/fixes_2_2
=======
	glVertex3v16(floattov16(x), floattov16(y), floattov16(z));
>>>>>>> origin/fixes_2_2
end;

procedure glRotatef32(angle: cfloat; x, y, z: cint32); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  glRotatef32i(trunc(angle * DEGREES_IN_CIRCLE / 360.0), x, y, z);
=======
	glRotatef32i(trunc(angle * LUT_SIZE / 360.0), x, y, z);
>>>>>>> graemeg/fixes_2_2
=======
	glRotatef32i(trunc(angle * LUT_SIZE / 360.0), x, y, z);
>>>>>>> origin/fixes_2_2
end;

procedure glRotatef(angle, x, y, z: cfloat); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  glRotatef32(angle, floattof32(x), floattof32(y), floattof32(z));
=======
	glRotatef32(angle, floattof32(x), floattof32(y), floattof32(z));
>>>>>>> graemeg/fixes_2_2
=======
	glRotatef32(angle, floattof32(x), floattof32(y), floattof32(z));
>>>>>>> origin/fixes_2_2
end;

procedure glColor3f(r, g, b: cfloat); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  glColor3b(trunc(r*255), trunc(g*255), trunc(b*255));
=======
	glColor3b(trunc(r*255), trunc(g*255), trunc(b*255));
>>>>>>> graemeg/fixes_2_2
=======
	glColor3b(trunc(r*255), trunc(g*255), trunc(b*255));
>>>>>>> origin/fixes_2_2
end;

procedure glScalef(x, y, z: cfloat); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  MATRIX_SCALE^ := floattof32(x);
  MATRIX_SCALE^ := floattof32(y);
  MATRIX_SCALE^ := floattof32(z);
=======
	MATRIX_SCALE^ := floattof32(x);
	MATRIX_SCALE^ := floattof32(y);
	MATRIX_SCALE^ := floattof32(z);
>>>>>>> graemeg/fixes_2_2
=======
	MATRIX_SCALE^ := floattof32(x);
	MATRIX_SCALE^ := floattof32(y);
	MATRIX_SCALE^ := floattof32(z);
>>>>>>> origin/fixes_2_2
end;

procedure glTranslatef(x, y, z: cfloat); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  MATRIX_TRANSLATE^ := floattof32(x);
  MATRIX_TRANSLATE^ := floattof32(y);
  MATRIX_TRANSLATE^ := floattof32(z);
=======
	MATRIX_TRANSLATE^ := floattof32(x);
	MATRIX_TRANSLATE^ := floattof32(y);
	MATRIX_TRANSLATE^ := floattof32(z);
>>>>>>> graemeg/fixes_2_2
=======
	MATRIX_TRANSLATE^ := floattof32(x);
	MATRIX_TRANSLATE^ := floattof32(y);
	MATRIX_TRANSLATE^ := floattof32(z);
>>>>>>> origin/fixes_2_2
end;

procedure glNormal3f(x, y, z: cfloat); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  glNormal(NORMAL_PACK(floattov10(x), floattov10(y), floattov10(z)));
=======
	glNormal(NORMAL_PACK(floattov10(x), floattov10(y), floattov10(z)));
>>>>>>> graemeg/fixes_2_2
=======
	glNormal(NORMAL_PACK(floattov10(x), floattov10(y), floattov10(z)));
>>>>>>> origin/fixes_2_2
end;

procedure glRotateX(angle: cfloat); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  glRotateXi(trunc(angle * DEGREES_IN_CIRCLE / 360.0));
=======
	glRotateXi(trunc(angle * LUT_SIZE / 360.0));
>>>>>>> graemeg/fixes_2_2
=======
	glRotateXi(trunc(angle * LUT_SIZE / 360.0));
>>>>>>> origin/fixes_2_2
end;

procedure glRotateY(angle: cfloat); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  glRotateYi(trunc(angle * DEGREES_IN_CIRCLE / 360.0));
=======
	glRotateYi(trunc(angle * LUT_SIZE / 360.0));
>>>>>>> graemeg/fixes_2_2
=======
	glRotateYi(trunc(angle * LUT_SIZE / 360.0));
>>>>>>> origin/fixes_2_2
end;

procedure glRotateZ(angle: cfloat); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  glRotateZi(trunc(angle * DEGREES_IN_CIRCLE / 360.0));
=======
	glRotateZi(trunc(angle * LUT_SIZE / 360.0));
>>>>>>> graemeg/fixes_2_2
=======
	glRotateZi(trunc(angle * LUT_SIZE / 360.0));
>>>>>>> origin/fixes_2_2
end;

procedure glOrtho(left, right, bottom, top, zNear, zFar: cfloat); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  glOrthof32(floattof32(left), floattof32(right), floattof32(bottom), floattof32(top), floattof32(zNear), floattof32(zFar));
=======
	glOrthof32(floattof32(left), floattof32(right), floattof32(bottom), floattof32(top), floattof32(zNear), floattof32(zFar));
>>>>>>> graemeg/fixes_2_2
=======
	glOrthof32(floattof32(left), floattof32(right), floattof32(bottom), floattof32(top), floattof32(zNear), floattof32(zFar));
>>>>>>> origin/fixes_2_2
end;

procedure gluLookAt(eyex, eyey, eyez, lookAtx, lookAty, lookAtz, upx, upy, upz: cfloat); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  gluLookAtf32(floattof32(eyex), floattof32(eyey), floattof32(eyez),
         floattof32(lookAtx), floattof32(lookAty), floattof32(lookAtz),
         floattof32(upx), floattof32(upy), floattof32(upz));
=======
	gluLookAtf32(floattof32(eyex), floattof32(eyey), floattof32(eyez),
				 floattof32(lookAtx), floattof32(lookAty), floattof32(lookAtz),
				 floattof32(upx), floattof32(upy), floattof32(upz));
>>>>>>> graemeg/fixes_2_2
=======
	gluLookAtf32(floattof32(eyex), floattof32(eyey), floattof32(eyez),
				 floattof32(lookAtx), floattof32(lookAty), floattof32(lookAtz),
				 floattof32(upx), floattof32(upy), floattof32(upz));
>>>>>>> origin/fixes_2_2
end;

procedure glFrustum(left, right, bottom, top, znear, zfar: cfloat); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  glFrustumf32(floattof32(left), floattof32(right), floattof32(bottom), floattof32(top), floattof32(znear), floattof32(zfar));
=======
	glFrustumf32(floattof32(left), floattof32(right), floattof32(bottom), floattof32(top), floattof32(znear), floattof32(zfar));
>>>>>>> graemeg/fixes_2_2
=======
	glFrustumf32(floattof32(left), floattof32(right), floattof32(bottom), floattof32(top), floattof32(znear), floattof32(zfar));
>>>>>>> origin/fixes_2_2
end;

procedure gluPerspective(fovy, aspect, zNear, zFar: cfloat); inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  gluPerspectivef32(trunc(fovy * DEGREES_IN_CIRCLE / 360.0), floattof32(aspect), floattof32(zNear), floattof32(zFar));
end;
{$endif NDS_IMPLEMENTATION}
=======
=======
>>>>>>> origin/fixes_2_2
	gluPerspectivef32(trunc(fovy * LUT_SIZE / 360.0), floattof32(aspect), floattof32(zNear), floattof32(zFar));    
end;
{$endif NDS_IMPLEMENTATION}

{$ifdef NDS_INTERFACE}
function int_to_12d3(n: cint): fixed12d3; inline;    
function float_to_12d3(n: cfloat): fixed12d3; inline;  
function inttof32(n: cint): cint32; inline;          
function f32toint(n: cint32): cint; inline;          
function floattof32(n: cfloat): cint32; inline;        
function f32tofloat(n: cint32): cfloat; inline;        
function f32tot16(n: cint32): t16; inline;            
function inttot16(n: cint): cint32; inline;    //???
function t16toint(n: t16): cint; inline;            
function floattot16(n: cfloat): t16; inline;          
function TEXTURE_PACK(u, v: cint): cint; inline;      
function inttov16(n: cint): cint; inline;        
function f32tov16(n: cint32): v16; inline;            
function v16toint(n: v16): cint; inline;            
function floattov16(n: cfloat): v16; inline;           
function VERTEX_PACK(x,y: cint): cint; inline;  		 
function inttov10(n: cint): cint; inline;            
function f32tov10(n: cint32): cint; inline;            
function v10toint(n: v10): cint; inline;            
function floattov10(n: cfloat): v10; inline;
function NORMAL_PACK(x,y,z: cint): cint; inline;     
function FIFO_COMMAND_PACK(c1,c2,c3,c4: cint): cint; inline; 
function REG2ID(r: pcuint32): cuint8; inline;						
function FIFO_NOP(): cuint8; inline;
function FIFO_STATUS(): cuint8; inline;
function FIFO_COLOR(): cuint8; inline;
function FIFO_VERTEX16(): cuint8; inline;
function FIFO_TEX_COORD(): cuint8; inline;
function FIFO_TEX_FORMAT(): cuint8; inline;
function FIFO_PAL_FORMAT(): cuint8; inline;
function FIFO_CLEAR_COLOR(): cuint8; inline;
function FIFO_CLEAR_DEPTH(): cuint8; inline;
function FIFO_LIGHT_VECTOR(): cuint8; inline;
function FIFO_LIGHT_COLOR(): cuint8; inline;
function FIFO_NORMAL(): cuint8; inline;
function FIFO_DIFFUSE_AMBIENT(): cuint8; inline;
function FIFO_SPECULAR_EMISSION(): cuint8; inline;
function FIFO_SHININESS(): cuint8; inline;
function FIFO_POLY_FORMAT(): cuint8; inline;
function FIFO_BEGIN(): cuint8; inline;
function FIFO_END(): cuint8; inline;
function FIFO_FLUSH(): cuint8; inline;
function FIFO_VIEWPORT(): cuint8; inline;
function POLY_ALPHA(n: cint): cuint32; inline;  
function POLY_ID(n: cint): cuint32; inline;	
procedure glBegin(mode: GL_GLBEGIN_ENUM); inline;
procedure glEnd(); inline;
procedure glClearDepth(depth: fixed12d3); inline;
procedure glColor3b(red, green, blue: cuint8); inline;
procedure glColor(color: rgb); inline;
procedure glVertex3v16(x, y, z: v16); inline;
procedure glTexCoord2t16(u, v: t16); inline; 
procedure glPushMatrix(); inline; 
procedure glPopMatrix(num: cint32); inline; 
procedure glRestoreMatrix(index: cint32); inline; 
procedure glStoreMatrix(index: cint32); inline; 
procedure glScalev(const v: PGLvector); inline; 
procedure glTranslatev(const v: PGLvector) ; inline; 
procedure glTranslate3f32(x, y, z: cint32); inline; 
procedure glScalef32(factor: cint32); inline; 
procedure glLight(id: cint; color: rgb; x, y, z: v10); inline; 
procedure glNormal(normal: cuint32); inline;
procedure glLoadIdentity(); inline;
procedure glIdentity(); inline;
procedure glMatrixMode(mode: GL_MATRIX_MODE_ENUM); inline;
procedure glViewport(x1, y1, x2, y2: cuint8); inline;
procedure glFlush(mode: cuint32); inline;
procedure glMaterialShinyness(); inline;
procedure glCallList(list: pcuint32); inline;
procedure glPolyFmt(params: cuint32); inline; 
procedure glEnable(bits: cint); inline;
procedure glDisable(bits: cint); inline;
procedure glLoadMatrix4x4(const m: pm4x4); inline;
procedure glLoadMatrix4x3(const m: pm4x3); inline;
procedure glMultMatrix4x4(const m: pm4x4); inline;
procedure glMultMatrix4x3(const m: pm4x3); inline;
procedure glMultMatrix3x3(const m: pm3x3); inline;
procedure glRotateXi(angle: cint); inline;
procedure glRotateYi(angle: cint); inline;
procedure glRotateZi(angle: cint); inline;
procedure glOrthof32(left, right, bottom, top, zNear, zFar: cint32); inline;
procedure gluLookAtf32(eyex, eyey, eyez, lookAtx, lookAty, lookAtz, upx, upy, upz: cint32); inline;
procedure glFrustumf32(left, right, bottom, top, zNear, zFar: cint32); inline;
procedure gluPerspectivef32(fovy: cint; aspect, zNear, zFar: cint32); inline;
procedure gluPickMatrix(x, y, width, height: cint32; const viewport: TArr4ofInt); inline;
procedure glResetMatrixStack(); inline;
procedure glSetOutlineColor(id: cint; color: rgb); inline;
procedure glSetToonTable(const table: pcuint16); inline;
procedure glSetToonTableRange(_start, _end: cint; color: rgb); inline;
procedure glGetFixed(const param: GL_GET_ENUM; f: pcint32); inline;
procedure glAlphaFunc(alphaThreshold: cint); inline;
procedure glCutoffDepth(wVal: fixed12d3); inline;
procedure glInit(); inline;
procedure glClearColor(red, green, blue, alpha: cuint8); inline;
procedure glClearPolyID(ID: cuint8); inline;
procedure glGetInt(param: GL_GET_ENUM; var i: cint); inline; 
procedure glVertex3f(x, y, z: cfloat); inline;
procedure glRotatef32(angle: cfloat; x, y, z: cint32); inline;
procedure glRotatef(angle, x, y, z: cfloat); inline;
procedure glColor3f(r, g, b: cfloat); inline;
procedure glScalef(x, y, z: cfloat); inline;
procedure glTranslatef(x, y, z: cfloat); inline;
procedure glNormal3f(x, y, z: cfloat); inline;
procedure glRotateX(angle: cfloat); inline;
procedure glRotateY(angle: cfloat); inline;
procedure glRotateZ(angle: cfloat); inline;
procedure glOrtho(left, right, bottom, top, zNear, zFar: cfloat); inline;
procedure gluLookAt(eyex, eyey, eyez, lookAtx, lookAty, lookAtz, upx, upy, upz: cfloat); inline;
procedure glFrustum(left, right, bottom, top, znear, zfar: cfloat); inline;
procedure gluPerspective(fovy, aspect, zNear, zFar: cfloat); inline;
procedure glTexCoord2f(s, t: cfloat); inline;
{$endif NDS_INTERFACE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
