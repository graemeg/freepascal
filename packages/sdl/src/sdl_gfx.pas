unit sdl_gfx;
{
  $Id: sdl_gfx.pas,v 1.3 2007/05/29 21:31:04 savage Exp $

}
{
  $Log: sdl_gfx.pas,v $
  Revision 1.3  2007/05/29 21:31:04  savage
  Changes as suggested by Almindor for 64bit compatibility.

  Revision 1.2  2007/05/20 20:30:18  savage
  Initial Changes to Handle 64 Bits

  Revision 1.1  2005/01/03 19:08:32  savage
  Header for the SDL_Gfx library.



}

{$I jedi-sdl.inc}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF MORPHOS}
  exec,
{$ENDIF}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  sdl;

const
{$IFDEF WINDOWS}
  SDLgfxLibName = 'SDL_gfx.dll';
{$ENDIF}

{$IFDEF UNIX}
{$IFDEF DARWIN}
  SDLgfxLibName = 'libSDL_gfx.dylib';
{$ELSE}
  SDLgfxLibName = 'libSDL_gfx.so';
{$ENDIF}
{$ENDIF}

{$IFDEF MACOS}
  SDLgfxLibName = 'SDL_gfx';
{$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF MORPHOS}
  SDLgfxLibName = 'powersdl_gfx.library';
{$ENDIF}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  // Some rates in Hz
  FPS_UPPER_LIMIT	= 200;
  FPS_LOWER_LIMIT	= 1;
  FPS_DEFAULT     = 30;

// ---- Defines

  SMOOTHING_OFF   = 0;
  SMOOTHING_ON    = 1;



type
  PFPSmanager = ^TFPSmanager;
  TFPSmanager = packed record
    framecount : Uint32;
    rateticks : single;
    lastticks : Uint32;
    rate : Uint32;
  end;

// ---- Structures
  PColorRGBA = ^TColorRGBA;
  TColorRGBA = packed record
    r : Uint8;
    g : Uint8;
    b : Uint8;
    a : Uint8;
  end;

  PColorY = ^TColorY;
  TColorY = packed record
    y :	Uint8;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF MORPHOS}
{$INCLUDE powersdl_gfx.inc}
{$ELSE MORPHOS}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{

 SDL_framerate: framerate manager

 LGPL (c) A. Schiffler

}

procedure SDL_initFramerate( manager : PFPSmanager );
cdecl; external {$IFDEF __GPC__}name 'SDL_initFramerate'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_initFramerate}

function SDL_setFramerate( manager : PFPSmanager; rate : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_setFramerate'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_setFramerate}

function SDL_getFramerate( manager : PFPSmanager ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_getFramerate'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_getFramerate}

procedure SDL_framerateDelay( manager : PFPSmanager );
cdecl; external {$IFDEF __GPC__}name 'SDL_framerateDelay'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_framerateDelay}

{

 SDL_gfxPrimitives: graphics primitives for SDL

 LGPL (c) A. Schiffler

}

// Note: all ___Color routines expect the color to be in format 0xRRGGBBAA 

// Pixel 

function pixelColor( dst : PSDL_Surface; x : Sint16; y : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'pixelColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM pixelColor}

function pixelRGBA( dst : PSDL_Surface; x : Sint16; y : Sint16; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'pixelRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM pixelRGBA}

// Horizontal line 

function hlineColor( dst : PSDL_Surface; x1: Sint16; x2 : Sint16; y : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'hlineColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM hlineColor}

function hlineRGBA( dst : PSDL_Surface; x1: Sint16; x2 : Sint16; y : Sint16; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'hlineRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM hlineRGBA}

// Vertical line 

function vlineColor( dst : PSDL_Surface; x : Sint16; y1 : Sint16; y2 : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'vlineColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM vlineColor}

function vlineRGBA( dst : PSDL_Surface; x : Sint16; y1 : Sint16; y2 : Sint16; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'vlineRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM vlineRGBA}

// Rectangle 

function rectangleColor( dst : PSDL_Surface; x1 : Sint16; y1 : Sint16; x2 : Sint16; y2 : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'rectangleColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM rectangleColor}

function rectangleRGBA( dst : PSDL_Surface; x1 : Sint16; y1 : Sint16;
				   x2 : Sint16; y2 : Sint16; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'rectangleRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM rectangleRGBA}

// Filled rectangle (Box) 

function boxColor( dst : PSDL_Surface; x1 : Sint16; y1 : Sint16; x2 : Sint16; y2 : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'boxColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM boxColor}

function boxRGBA( dst : PSDL_Surface; x1 : Sint16; y1 : Sint16; x2 : Sint16;
			     y2 : Sint16; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'boxRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM boxRGBA}

// Line 

function lineColor( dst : PSDL_Surface; x1 : Sint16; y1 : Sint16; x2 : Sint16; y2 : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'lineColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM lineColor}

function lineRGBA( dst : PSDL_Surface; x1 : Sint16; y1 : Sint16;
			      x2 : Sint16; y2 : Sint16; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'lineRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM lineRGBA}

// AA Line 
function aalineColor( dst : PSDL_Surface; x1 : Sint16; y1 : Sint16; x2 : Sint16; y2 : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'aalineColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM aalineColor}

function aalineRGBA( dst : PSDL_Surface; x1 : Sint16; y1 : Sint16;
				x2 : Sint16; y2 : Sint16; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'aalineRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM aalineRGBA}

// Circle 

function circleColor( dst : PSDL_Surface; x : Sint16; y : Sint16; r : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'circleColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM circleColor}

function circleRGBA( dst : PSDL_Surface; x : Sint16; y : Sint16; rad : Sint16; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'circleRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM circleRGBA}

// AA Circle 

function aacircleColor( dst : PSDL_Surface; x : Sint16; y : Sint16; r : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'aacircleColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM aacircleColor}

function aacircleRGBA( dst : PSDL_Surface; x : Sint16; y : Sint16;
				  rad : Sint16; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'aacircleRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM aacircleRGBA}

// Filled Circle 

function filledCircleColor( dst : PSDL_Surface; x : Sint16; y : Sint16; r : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'filledCircleColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM filledCircleColor}

function filledCircleRGBA( dst : PSDL_Surface; x : Sint16; y : Sint16;
				      rad : Sint16; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'filledCircleRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM filledCircleRGBA}

// Ellipse 

function ellipseColor( dst : PSDL_Surface; x : Sint16; y : Sint16; rx : Sint16; ry : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'ellipseColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM ellipseColor}

function ellipseRGBA( dst : PSDL_Surface; x : Sint16; y : Sint16;
				 rx : Sint16; ry : Sint16; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'ellipseRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM ellipseRGBA}

// AA Ellipse 

function aaellipseColor( dst : PSDL_Surface; xc : Sint16; yc : Sint16; rx : Sint16; ry : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'aaellipseColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM aaellipseColor}

function aaellipseRGBA( dst : PSDL_Surface; x : Sint16; y : Sint16;
				   rx : Sint16; ry : Sint16; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'aaellipseRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM aaellipseRGBA}

// Filled Ellipse 

function filledEllipseColor( dst : PSDL_Surface; x : Sint16; y : Sint16; rx : Sint16; ry : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'filledEllipseColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM filledEllipseColor}

function filledEllipseRGBA( dst : PSDL_Surface; x : Sint16; y : Sint16;
				       rx : Sint16; ry : Sint16; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'filledEllipseRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM filledEllipseRGBA}

// Pie

function pieColor( dst : PSDL_Surface; x : Sint16; y : Sint16; rad : Sint16;
			      start : Sint16; finish : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'pieColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM pieColor}

function pieRGBA( dst : PSDL_Surface; x : Sint16; y : Sint16; rad : Sint16;
			     start : Sint16; finish : Sint16; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'pieRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM pieRGBA}

// Filled Pie

function filledPieColor( dst : PSDL_Surface; x : Sint16; y : Sint16; rad : Sint16;
				    start : Sint16; finish : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'filledPieColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM filledPieColor}

function filledPieRGBA( dst : PSDL_Surface; x : Sint16; y : Sint16; rad : Sint16;
				   start : Sint16; finish : Sint16; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'filledPieRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM filledPieRGBA}

// Trigon

function trigonColor( dst : PSDL_Surface; x1 : Sint16; y1 : Sint16; x2 : Sint16; y2 : Sint16; x3 : Sint16; y3 : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'trigonColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM trigonColor}

function trigonRGBA( dst : PSDL_Surface; x1 : Sint16; y1 : Sint16; x2 : Sint16; y2 : Sint16; x3 : Sint16; y3 : Sint16;
				 r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'trigonRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM trigonRGBA}

// AA-Trigon

function aatrigonColor( dst : PSDL_Surface; x1 : Sint16; y1 : Sint16; x2 : Sint16; y2 : Sint16; x3 : Sint16; y3 : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'aatrigonColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM aatrigonColor}
function aatrigonRGBA( dst : PSDL_Surface;  x1 : Sint16; y1 : Sint16; x2 : Sint16; y2 : Sint16; x3 : Sint16; y3 : Sint16;
				   r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'aatrigonRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM aatrigonRGBA}

// Filled Trigon

function filledTrigonColor( dst : PSDL_Surface; x1 : Sint16; y1 : Sint16; x2 : Sint16; y2 : Sint16; x3 : Sint16; y3 : Sint16; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'filledTrigonColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM filledTrigonColor}

function filledTrigonRGBA( dst : PSDL_Surface; x1 : Sint16; y1 : Sint16; x2 : Sint16; y2 : Sint16; x3 : Sint16; y3 : Sint16;
				       r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'filledTrigonRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM filledTrigonRGBA}

// Polygon

function polygonColor( dst : PSDL_Surface; const vx : PSint16; const vy : PSint16; n : integer; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'polygonColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM polygonColor}

function polygonRGBA( dst : PSDL_Surface; const vx : PSint16; const vy : PSint16;
				 n : integer; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'polygonRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM polygonRGBA}

// AA-Polygon

function aapolygonColor( dst : PSDL_Surface; const vx : PSint16; const vy : PSint16; n : integer; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'aapolygonColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM aapolygonColor}

function aapolygonRGBA( dst : PSDL_Surface; const vx : PSint16; const vy : PSint16;
				   n : integer; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'aapolygonRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM aapolygonRGBA}

// Filled Polygon

function filledPolygonColor( dst : PSDL_Surface; const vx : PSint16; const vy : PSint16; n : integer; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'filledPolygonColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM filledPolygonColor}

function filledPolygonRGBA( dst : PSDL_Surface; const vx : PSint16;
				       const vy : PSint16; n : integer; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'filledPolygonRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM filledPolygonRGBA}

// Bezier
// s = number of steps

function bezierColor( dst : PSDL_Surface; const vx : PSint16; const vy : PSint16; n : integer; s : integer; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'bezierColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM bezierColor}

function bezierRGBA( dst : PSDL_Surface; const vx : PSint16; const vy : PSint16;
				 n : integer; s : integer; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'bezierRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM bezierRGBA}


// Characters/Strings

function characterColor( dst : PSDL_Surface; x : Sint16; y : Sint16; c : char; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'characterColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM characterColor}

function characterRGBA( dst : PSDL_Surface; x : Sint16; y : Sint16; c : char; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'characterRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM characterRGBA}

function stringColor( dst : PSDL_Surface; x : Sint16; y : Sint16; const c : PChar; color : Uint32 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'stringColor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM stringColor}

function stringRGBA( dst : PSDL_Surface; x : Sint16; y : Sint16; const c : PChar; r : Uint8; g : Uint8; b : Uint8; a : Uint8 ) : integer;
cdecl; external {$IFDEF __GPC__}name 'stringRGBA'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM stringRGBA}

procedure gfxPrimitivesSetFont(const fontdata : Pointer; cw : integer; ch : integer );
cdecl; external {$IFDEF __GPC__}name 'gfxPrimitivesSetFont'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM gfxPrimitivesSetFont}

{

 SDL_imageFilter - bytes-image "filter" routines
 (uses inline x86 MMX optimizations if available)

 LGPL (c) A. Schiffler

}

{ Comments:                                                                           }
{  1.) MMX functions work best if all data blocks are aligned on a 32 bytes boundary. }
{  2.) Data that is not within an 8 byte boundary is processed using the C routine.   }
{  3.) Convolution routines do not have C routines at this time.                      }

// Detect MMX capability in CPU
function SDL_imageFilterMMXdetect : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterMMXdetect'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterMMXdetect}

// Force use of MMX off (or turn possible use back on)
procedure SDL_imageFilterMMXoff;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterMMXoff'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterMMXoff}

procedure SDL_imageFilterMMXon;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterMMXon'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterMMXon}

//
// All routines return:
//   0   OK
//  -1   Error (internal error, parameter error)
//

//  SDL_imageFilterAdd: D = saturation255(S1 + S2)
function SDL_imageFilterAdd(Src1 : PChar; Src2 : PChar; Dest : PChar; length : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imaSDL_imageFilterAddgeFilterMMXon'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterAdd}

//  SDL_imageFilterMean: D = S1/2 + S2/2
function SDL_imageFilterMean(Src1 : PChar; Src2 : PChar; Dest : PChar; length : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterMean'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterMean}

//  SDL_imageFilterSub: D = saturation0(S1 - S2)
function SDL_imageFilterSub(Src1 : PChar; Src2 : PChar; Dest : PChar; length : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterSub'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterSub}

//  SDL_imageFilterAbsDiff: D = | S1 - S2 |
function SDL_imageFilterAbsDiff(Src1 : PChar; Src2 : PChar; Dest : PChar; length : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterAbsDiff'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterAbsDiff}

//  SDL_imageFilterMult: D = saturation(S1 * S2)
function SDL_imageFilterMult(Src1 : PChar; Src2 : PChar; Dest : PChar; length : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterMult'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterMult}

//  SDL_imageFilterMultNor: D = S1 * S2   (non-MMX)
function SDL_imageFilterMultNor(Src1 : PChar; Src2 : PChar; Dest : PChar; length : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterMultNor'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterMultNor}

//  SDL_imageFilterMultDivby2: D = saturation255(S1/2 * S2)
function SDL_imageFilterMultDivby2(Src1 : PChar; Src2 : PChar; Dest : PChar;
					       length : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterMultDivby2'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterMultDivby2}

//  SDL_imageFilterMultDivby4: D = saturation255(S1/2 * S2/2)
function SDL_imageFilterMultDivby4(Src1 : PChar; Src2 : PChar; Dest : PChar;
					       length : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterMultDivby4'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterMultDivby4}

//  SDL_imageFilterBitAnd: D = S1 & S2
function SDL_imageFilterBitAnd(Src1 : PChar; Src2 : PChar; Dest : PChar; length : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterBitAnd'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterBitAnd}

//  SDL_imageFilterBitOr: D = S1 | S2
function SDL_imageFilterBitOr(Src1 : PChar; Src2 : PChar; Dest : PChar; length : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterBitOr'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterBitOr}

//  SDL_imageFilterDiv: D = S1 / S2   (non-MMX)
function SDL_imageFilterDiv(Src1 : PChar; Src2 : PChar; Dest : PChar; length : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterDiv'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterDiv}

//  SDL_imageFilterBitNegation: D = !S
function SDL_imageFilterBitNegation(Src1 : PChar; Dest : PChar; length : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterBitNegation'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterBitNegation}

//  SDL_imageFilterAddByte: D = saturation255(S + C)
function SDL_imageFilterAddByte(Src1 : PChar; Dest : PChar; length : integer; C : char ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterAddByte'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterAddByte}

//  SDL_imageFilterAddUint: D = saturation255(S + (uint)C)
function SDL_imageFilterAddUint(Src1 : PChar; Dest : PChar; length : integer; C : Cardinal ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterAddUint'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterAddUint}

//  SDL_imageFilterAddByteToHalf: D = saturation255(S/2 + C)
function SDL_imageFilterAddByteToHalf(Src1 : PChar; Dest : PChar; length : integer;
						  C : char ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterAddByteToHalf'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterAddByteToHalf}

//  SDL_imageFilterSubByte: D = saturation0(S - C)
function SDL_imageFilterSubByte(Src1 : PChar; Dest : PChar; length : integer; C : char ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterSubByte'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterSubByte}

//  SDL_imageFilterSubUint: D = saturation0(S - (uint)C)
function SDL_imageFilterSubUint(Src1 : PChar; Dest : PChar; length : integer; C : Cardinal ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterSubUint'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterSubUint}

//  SDL_imageFilterShiftRight: D = saturation0(S >> N)
function SDL_imageFilterShiftRight(Src1 : PChar; Dest : PChar; length : integer;  N : char ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterShiftRight'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterShiftRight}

//  SDL_imageFilterShiftRightUint: D = saturation0((uint)S >> N)
function SDL_imageFilterShiftRightUint(Src1 : PChar; Dest : PChar; length : integer;  N : char ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterShiftRightUint'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterShiftRightUint}

//  SDL_imageFilterMultByByte: D = saturation255(S * C)
function SDL_imageFilterMultByByte(Src1 : PChar; Dest : PChar; length : integer; C : char ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterMultByByte'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterMultByByte}

//  SDL_imageFilterShiftRightAndMultByByte: D = saturation255((S >> N) * C)
function SDL_imageFilterShiftRightAndMultByByte(Src1 : PChar; Dest : PChar; length : integer;
							     N : char; C : char ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterShiftRightAndMultByByte'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterShiftRightAndMultByByte}

//  SDL_imageFilterShiftLeftByte: D = (S << N)
function SDL_imageFilterShiftLeftByte(Src1 : PChar; Dest : PChar; length : integer;
						   N : char ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterShiftLeftByte'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterShiftLeftByte}

//  SDL_imageFilterShiftLeftUint: D = ((uint)S << N)
function SDL_imageFilterShiftLeftUint(Src1 : PChar; Dest : PChar; length : integer;
						   N : char ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterShiftLeftUint'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterShiftLeftUint}

//  SDL_imageFilterShiftLeft: D = saturation255(S << N)
function SDL_imageFilterShiftLeft(Src1 : PChar; Dest : PChar; length : integer;  N : char ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterShiftLeft'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterShiftLeft}

//  SDL_imageFilterBinarizeUsingThreshold: D = S >= T ? 255:0
function SDL_imageFilterBinarizeUsingThreshold(Src1 : PChar; Dest : PChar; length : integer;
							   T : char ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterBinarizeUsingThreshold'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterBinarizeUsingThreshold}

//  SDL_imageFilterClipToRange: D = (S >= Tmin) & (S <= Tmax) 255:0
function SDL_imageFilterClipToRange(Src1 : PChar; Dest : PChar; length : integer;
						Tmin : Byte; Tmax : Byte ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterClipToRange'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterClipToRange}

//  SDL_imageFilterNormalizeLinear: D = saturation255((Nmax - Nmin)/(Cmax - Cmin)*(S - Cmin) + Nmin)
function SDL_imageFilterNormalizeLinear(Src1 : PChar; Dest : PChar; length : integer; Cmin : integer;
						    Cmax : integer; Nmin : integer; Nmax : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterClipToRange'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterClipToRange}

{ !!! NO C-ROUTINE FOR THESE FUNCTIONS YET !!! }

//  SDL_imageFilterConvolveKernel3x3Divide: Dij = saturation0and255( ... )
function SDL_imageFilterConvolveKernel3x3Divide(Src : PChar; Dest : PChar; rows : integer;
							    columns : integer; Kernel : PShortInt; Divisor : Byte ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterConvolveKernel3x3Divide'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterConvolveKernel3x3Divide}

//  SDL_imageFilterConvolveKernel5x5Divide: Dij = saturation0and255( ... )
function SDL_imageFilterConvolveKernel5x5Divide(Src : PChar; Dest : PChar; rows : integer;
							    columns : integer; Kernel : PShortInt; Divisor : Byte ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterConvolveKernel5x5Divide'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterConvolveKernel5x5Divide}

//  SDL_imageFilterConvolveKernel7x7Divide: Dij = saturation0and255( ... )
function SDL_imageFilterConvolveKernel7x7Divide(Src : PChar; Dest : PChar; rows : integer;
							    columns : integer; Kernel : PShortInt; Divisor : Byte ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterConvolveKernel7x7Divide'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterConvolveKernel7x7Divide}

//  SDL_imageFilterConvolveKernel9x9Divide: Dij = saturation0and255( ... )
function SDL_imageFilterConvolveKernel9x9Divide(Src : PChar; Dest : PChar; rows : integer;
							    columns : integer; Kernel : PShortInt; Divisor : Byte ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterConvolveKernel9x9Divide'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterConvolveKernel9x9Divide}

//  SDL_imageFilterConvolveKernel3x3ShiftRight: Dij = saturation0and255( ... )
function SDL_imageFilterConvolveKernel3x3ShiftRight(Src : PChar; Dest : PChar; rows : integer;
								columns : integer; Kernel : PShortInt;
								 NRightShift : char ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterConvolveKernel3x3ShiftRight'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterConvolveKernel3x3ShiftRight}

//  SDL_imageFilterConvolveKernel5x5ShiftRight: Dij = saturation0and255( ... )
function SDL_imageFilterConvolveKernel5x5ShiftRight(Src : PChar; Dest : PChar; rows : integer;
								columns : integer; Kernel : PShortInt;
								 NRightShift : char ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterConvolveKernel5x5ShiftRight'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterConvolveKernel5x5ShiftRight}

//  SDL_imageFilterConvolveKernel7x7ShiftRight: Dij = saturation0and255( ... )
function SDL_imageFilterConvolveKernel7x7ShiftRight(Src : PChar; Dest : PChar; rows : integer;
								columns : integer; Kernel : PShortInt;
								 NRightShift : char ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterConvolveKernel7x7ShiftRight'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterConvolveKernel7x7ShiftRight}

//  SDL_imageFilterConvolveKernel9x9ShiftRight: Dij = saturation0and255( ... )
function SDL_imageFilterConvolveKernel9x9ShiftRight(Src : PChar; Dest : PChar; rows : integer;
								columns : integer; Kernel : PShortInt;
								 NRightShift : char ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterConvolveKernel9x9ShiftRight'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterConvolveKernel9x9ShiftRight}

//  SDL_imageFilterSobelX: Dij = saturation255( ... )
function SDL_imageFilterSobelX(Src : PChar; Dest : PChar; rows : integer; columns : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterSobelX'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterSobelX}

//  SDL_imageFilterSobelXShiftRight: Dij = saturation255( ... )
function SDL_imageFilterSobelXShiftRight(Src : PChar; Dest : PChar; rows : integer; columns : integer;
						      NRightShift : char ) : integer;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterSobelXShiftRight'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterSobelXShiftRight}

// Align/restore stack to 32 byte boundary -- Functionality untested! --
procedure SDL_imageFilterAlignStack;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterAlignStack'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterAlignStack}

procedure SDL_imageFilterRestoreStack;
cdecl; external {$IFDEF __GPC__}name 'SDL_imageFilterRestoreStack'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDL_imageFilterRestoreStack}

{

 SDL_rotozoom - rotozoomer

 LGPL (c) A. Schiffler

}

{ 
 
 rotozoomSurface()

 Rotates and zoomes a 32bit or 8bit 'src' surface to newly created 'dst' surface.
 'angle' is the rotation in degrees. 'zoom' a scaling factor. If 'smooth' is 1
 then the destination 32bit surface is anti-aliased. If the surface is not 8bit
 or 32bit RGBA/ABGR it will be converted into a 32bit RGBA format on the fly.

}

function rotozoomSurface( src : PSDL_Surface; angle : double; zoom : double; smooth : integer ) : PSDL_Surface;
cdecl; external {$IFDEF __GPC__}name 'rotozoomSurface'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM rotozoomSurface}

function rotozoomSurfaceXY( src : PSDL_Surface; angle : double; zoomx : double; zoomy : double; smooth : integer ) : PSDL_Surface;
cdecl; external {$IFDEF __GPC__}name 'rotozoomSurfaceXY'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM rotozoomSurfaceXY}

{ Returns the size of the target surface for a rotozoomSurface() call }

procedure rotozoomSurfaceSize( width : integer; height : integer; angle : double; zoom : double; var dstwidth : integer;
					  var dstheight : integer );
cdecl; external {$IFDEF __GPC__}name 'rotozoomSurfaceSize'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM rotozoomSurfaceSize}

procedure rotozoomSurfaceSizeXY
    ( width : integer; height : integer; angle : double; zoomx : double; zoomy : double;
     var dstwidth : integer; var dstheight : integer );
cdecl; external {$IFDEF __GPC__}name 'rotozoomSurfaceSizeXY'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM rotozoomSurfaceSizeXY}

{

 zoomSurface()

 Zoomes a 32bit or 8bit 'src' surface to newly created 'dst' surface.
 'zoomx' and 'zoomy' are scaling factors for width and height. If 'smooth' is 1
 then the destination 32bit surface is anti-aliased. If the surface is not 8bit
 or 32bit RGBA/ABGR it will be converted into a 32bit RGBA format on the fly.

}

function zoomSurface(src : PSDL_Surface; zoomx : double; zoomy : double; smooth : integer ) : PSDL_Surface;
cdecl; external {$IFDEF __GPC__}name 'zoomSurface'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM zoomSurface}

{ Returns the size of the target surface for a zoomSurface() call }

procedure zoomSurfaceSize( width : integer; height : integer; zoomx : double; zoomy : double; var dstwidth : integer; var dstheight : integer );
cdecl; external {$IFDEF __GPC__}name 'zoomSurfaceSize'{$ELSE} SDLgfxLibName{$ENDIF __GPC__};
{$EXTERNALSYM zoomSurfaceSize}

<<<<<<< HEAD
<<<<<<< HEAD
{$ENDIF MORPHOS}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
implementation

end.