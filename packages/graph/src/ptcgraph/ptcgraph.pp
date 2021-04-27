{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2010, 2011 by Nikolay Nikolov (nickysn@users.sourceforge.net)
    Copyright (c) 2007 by Daniel Mantione
      member of the Free Pascal development team

    This file implements the PTC support for the graph unit

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit ptcgraph;

{//$define logging}
{$define FPC_GRAPH_SUPPORTS_TRUECOLOR}
{$modeswitch DEFAULTPARAMETERS+}

{******************************************************************************}
                                    interface
{******************************************************************************}

uses
  ptc, ptcwrapper;

{$ifdef VER2_6}
type
  CodePointer = Pointer;
{$endif}

{$i graphh.inc}

{Driver number for PTC.}
const
  _PTC=22;

//  CGA      = 1;
//  MCGA     = 2;
//  EGA      = 3;
//  EGA64    = 4;
//  EGAMono  = 5;
  IBM8514  = 6;
//  HercMono = 7;
  ATT400   = 8;
//  VGA      = 9;
  PC3270   = 10;
  LastDriverNum = 10;

//  CGAC0 = 0;
//  CGAC1 = 1;
//  CGAC2 = 2;
//  CGAC3 = 3;
//  CGAHi = 4;

//  MCGAC0  = 0;
//  MCGAC1  = 1;
//  MCGAC2  = 2;
//  MCGAC3  = 3;
//  MCGAMed = 4;
//  MCGAHi  = 5;

//  EGALo = 0;
//  EGAHi = 1;

//  EGA64Lo = 0;
//  EGA64Hi = 1;

//  EGAMonoHi = 3;

  IBM8514Lo = 0;
  IBM8514Hi = 1;

//  HercMonoHi = 0;

  ATT400C0  = 0;
  ATT400C1  = 1;
  ATT400C2  = 2;
  ATT400C3  = 3;
  ATT400Med = 4;
  ATT400Hi  = 5;

//  VGALo  = 0;
//  VGAMed = 1;
//  VGAHi  = 2;



  m640x200x16       = VGALo;
  m640x400x16       = VGAMed;
  m640x480x16       = VGAHi;

  { VESA Specific video modes. }
  m320x200x32k      = $10D;
  m320x200x64k      = $10E;
{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
  m320x200x16m      = $10F;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

  m640x400x256      = $100;

  m640x480x256      = $101;
  m640x480x32k      = $110;
  m640x480x64k      = $111;
{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
  m640x480x16m      = $112;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

  m800x600x16       = $102;
  m800x600x256      = $103;
  m800x600x32k      = $113;
  m800x600x64k      = $114;
{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
  m800x600x16m      = $115;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

  m1024x768x16      = $104;
  m1024x768x256     = $105;
  m1024x768x32k     = $116;
  m1024x768x64k     = $117;
{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
  m1024x768x16m     = $118;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

  m1280x1024x16     = $106;
  m1280x1024x256    = $107;
  m1280x1024x32k    = $119;
  m1280x1024x64k    = $11A;
{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
  m1280x1024x16m    = $11B;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

const
  FullscreenGraph: Boolean = False;

var
  WindowTitle: AnsiString;
  PTCWrapperObject: TPTCWrapperThread;

function InstallUserMode(Width, Height: SmallInt; Colors: LongInt; HardwarePages: SmallInt = 1; XAspect: Word = 10000; YAspect: Word = 10000): smallint;

{******************************************************************************}
                                 implementation
{******************************************************************************}

const
  InternalDriverName = 'PTCPas';
  FirstNonStandardModeNumber = $200;
  NonStandardModeNumberMaxLimit = $7FFF;

var
  Has320x200: Boolean;
  Has320x240: Boolean;
  charmessagehandler: Pointer;  { dummy, for compatibility with graph.inc, which initializes this to nil under win32 }
  hasVesa: Boolean = false;  { dummy, for compatibility with graph.inc, which checks it in its ExitProc under go32v2 }
  VesaInfo: record { dummy, for compatibility with graph.inc under go32v2 }
    ModeList: PInteger;
  end;
  NextNonStandardModeNumber: LongInt;

{$i graph.inc}

type
  PByte = ^Byte;
  PLongInt = ^LongInt;

  PByteArray = ^TByteArray;
  TByteArray = array [0..MAXINT - 1] of Byte;

  TEGAPalette = array [0..15] of 0..63;
  TVGAPalette = array [0..255, 0..2] of 0..63;

{ ---------------------------------------------------------------------
   SVGA bindings.

  ---------------------------------------------------------------------}

const
  DefaultEGAPalette: TEGAPalette =
    (0, 1, 2, 3, 4, 5, 20, 7, 56, 57, 58, 59, 60, 61, 62, 63);

  { default EGA palette for modes 0Dh and 0Eh }
  DefaultEGAPalette_200: TEGAPalette =
    (0, 1, 2, 3, 4, 5, 6, 7, 16, 17, 18, 19, 20, 21, 22, 23);

  { default VGA palette for modes 04h, 05h, 06h, 0Dh and 0Eh }
  DefaultVGA16Palette_CGAEmu: TVGAPalette =
    ((0,0,0),(0,0,42),(0,42,0),(0,42,42),(42,0,0),(42,0,42),(42,21,0),(42,42,42),
     (0,0,0),(0,0,42),(0,42,0),(0,42,42),(42,0,0),(42,0,42),(42,21,0),(42,42,42),
     (21,21,21),(21,21,63),(21,63,21),(21,63,63),(63,21,21),(63,21,63),(63,63,21),(63,63,63),
     (21,21,21),(21,21,63),(21,63,21),(21,63,63),(63,21,21),(63,21,63),(63,63,21),(63,63,63),
     (0,0,0),(0,0,42),(0,42,0),(0,42,42),(42,0,0),(42,0,42),(42,21,0),(42,42,42),
     (0,0,0),(0,0,42),(0,42,0),(0,42,42),(42,0,0),(42,0,42),(42,21,0),(42,42,42),
     (21,21,21),(21,21,63),(21,63,21),(21,63,63),(63,21,21),(63,21,63),(63,63,21),(63,63,63),
     (21,21,21),(21,21,63),(21,63,21),(21,63,63),(63,21,21),(63,21,63),(63,63,21),(63,63,63),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0));

  { default VGA palette for modes 00h, 01h, 02h, 03h, 10h, 11h, 12h }
  DefaultVGA16Palette: TVGAPalette =
    ((0,0,0),(0,0,42),(0,42,0),(0,42,42),(42,0,0),(42,0,42),(42,42,0),(42,42,42),
     (0,0,21),(0,0,63),(0,42,21),(0,42,63),(42,0,21),(42,0,63),(42,42,21),(42,42,63),
     (0,21,0),(0,21,42),(0,63,0),(0,63,42),(42,21,0),(42,21,42),(42,63,0),(42,63,42),
     (0,21,21),(0,21,63),(0,63,21),(0,63,63),(42,21,21),(42,21,63),(42,63,21),(42,63,63),
     (21,0,0),(21,0,42),(21,42,0),(21,42,42),(63,0,0),(63,0,42),(63,42,0),(63,42,42),
     (21,0,21),(21,0,63),(21,42,21),(21,42,63),(63,0,21),(63,0,63),(63,42,21),(63,42,63),
     (21,21,0),(21,21,42),(21,63,0),(21,63,42),(63,21,0),(63,21,42),(63,63,0),(63,63,42),
     (21,21,21),(21,21,63),(21,63,21),(21,63,63),(63,21,21),(63,21,63),(63,63,21),(63,63,63),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),
     (0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0));

  { default VGA palette for mode 13h }
  DefaultVGA256Palette: TVGAPalette =
   ((0,0,0),(0,0,42),(0,42,0),(0,42,42),(42,0,0),(42,0,42),(42,21,0),(42,42,42),(21,21,21),(21,21,63),(21,63,21),(21,63,63),(63,21,21),
    (63,21,63),(63,63,21),(63,63,63),(0,0,0),(5,5,5),(8,8,8),(11,11,11),(14,14,14),(17,17,17),(20,20,20),(24,24,24),(28,28,28),(32,32,32),
    (36,36,36),(40,40,40),(45,45,45),(50,50,50),(56,56,56),(63,63,63),(0,0,63),(16,0,63),(31,0,63),(47,0,63),(63,0,63),(63,0,47),(63,0,31),
    (63,0,16),(63,0,0),(63,16,0),(63,31,0),(63,47,0),(63,63,0),(47,63,0),(31,63,0),(16,63,0),(0,63,0),(0,63,16),(0,63,31),(0,63,47),(0,63,63),
    (0,47,63),(0,31,63),(0,16,63),(31,31,63),(39,31,63),(47,31,63),(55,31,63),(63,31,63),(63,31,55),(63,31,47),(63,31,39),(63,31,31),(63,39,31),
    (63,47,31),(63,55,31),(63,63,31),(55,63,31),(47,63,31),(39,63,31),(31,63,31),(31,63,39),(31,63,47),(31,63,55),(31,63,63),(31,55,63),(31,47,63),
    (31,39,63),(45,45,63),(49,45,63),(54,45,63),(58,45,63),(63,45,63),(63,45,58),(63,45,54),(63,45,49),(63,45,45),(63,49,45),(63,54,45),(63,58,45),
    (63,63,45),(58,63,45),(54,63,45),(49,63,45),(45,63,45),(45,63,49),(45,63,54),(45,63,58),(45,63,63),(45,58,63),(45,54,63),(45,49,63),(0,0,28),
    (7,0,28),(14,0,28),(21,0,28),(28,0,28),(28,0,21),(28,0,14),(28,0,7),(28,0,0),(28,7,0),(28,14,0),(28,21,0),(28,28,0),(21,28,0),(14,28,0),(7,28,0),
    (0,28,0),(0,28,7),(0,28,14),(0,28,21),(0,28,28),(0,21,28),(0,14,28),(0,7,28),(14,14,28),(17,14,28),(21,14,28),(24,14,28),(28,14,28),(28,14,24),
    (28,14,21),(28,14,17),(28,14,14),(28,17,14),(28,21,14),(28,24,14),(28,28,14),(24,28,14),(21,28,14),(17,28,14),(14,28,14),(14,28,17),(14,28,21),
    (14,28,24),(14,28,28),(14,24,28),(14,21,28),(14,17,28),(20,20,28),(22,20,28),(24,20,28),(26,20,28),(28,20,28),(28,20,26),(28,20,24),(28,20,22),
    (28,20,20),(28,22,20),(28,24,20),(28,26,20),(28,28,20),(26,28,20),(24,28,20),(22,28,20),(20,28,20),(20,28,22),(20,28,24),(20,28,26),(20,28,28),
    (20,26,28),(20,24,28),(20,22,28),(0,0,16),(4,0,16),(8,0,16),(12,0,16),(16,0,16),(16,0,12),(16,0,8),(16,0,4),(16,0,0),(16,4,0),(16,8,0),(16,12,0),
    (16,16,0),(12,16,0),(8,16,0),(4,16,0),(0,16,0),(0,16,4),(0,16,8),(0,16,12),(0,16,16),(0,12,16),(0,8,16),(0,4,16),(8,8,16),(10,8,16),(12,8,16),
    (14,8,16),(16,8,16),(16,8,14),(16,8,12),(16,8,10),(16,8,8),(16,10,8),(16,12,8),(16,14,8),(16,16,8),(14,16,8),(12,16,8),(10,16,8),(8,16,8),
    (8,16,10),(8,16,12),(8,16,14),(8,16,16),(8,14,16),(8,12,16),(8,10,16),(11,11,16),(12,11,16),(13,11,16),(15,11,16),(16,11,16),(16,11,15),
    (16,11,13),(16,11,12),(16,11,11),(16,12,11),(16,13,11),(16,15,11),(16,16,11),(15,16,11),(13,16,11),(12,16,11),(11,16,11),(11,16,12),(11,16,13),
    (11,16,15),(11,16,16),(11,15,16),(11,13,16),(11,12,16),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0));

  { Text }

  WRITEMODE_OVERWRITE = 0;
  WRITEMODE_MASKED    = 1;
  FONT_EXPANDED       = 0;
  FONT_COMPRESSED     = 2;

 { Types }
 type
   pvga_modeinfo = ^vga_modeinfo;
   vga_modeinfo = record
     width,
     height,
     bytesperpixel,
     colors,
     linewidth,          { scanline width in bytes }
     maxlogicalwidth,    { maximum logical scanline width }
     startaddressrange,  { changeable bits set }
     maxpixels,          { video memory / bytesperpixel }
     haveblit,           { mask of blit functions available }
     flags: Longint;     { other flags }
    { Extended fields: }
     chiptype,           { Chiptype detected }
     memory,             { videomemory in KB }
     linewidth_unit: Longint;    { Use only a multiple of this as parameter for                                   set_displaystart }
     linear_aperture: PChar;     { points to mmap secondary mem aperture of card }
     aperture_size: Longint;     { size of aperture in KB if size>=videomemory.}

     set_aperture_page: procedure (page: Longint);
            { if aperture_size<videomemory select a memory page }
     extensions: Pointer;        { points to copy of eeprom for mach32 }
            { depends from actual driver/chiptype.. etc. }
     end;

  PGraphicsContext = ^TGraphicsContext;
  TGraphicsContext = record
                       ModeType: Byte;
                       ModeFlags: Byte;
                       Dummy: Byte;
                       FlipPage: Byte;
                       Width: LongInt;
                       Height: LongInt;
                       BytesPerPixel: LongInt;
                       Colors: LongInt;
                       BitsPerPixel: LongInt;
                       ByteWidth: LongInt;
                       VBuf: pointer;
                       Clip: LongInt;
                       ClipX1: LongInt;
                       ClipY1: LongInt;
                       ClipX2: LongInt;
                       ClipY2: LongInt;
                       ff: pointer;
                     end;

var
  PTCWidth: Integer;
  PTCHeight: Integer;
  PTCFormat8: IPTCFormat;
  PTCFormat15: IPTCFormat;
  PTCFormat16: IPTCFormat;
{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
  PTCFormat32: IPTCFormat;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

  EGAPaletteEnabled: Boolean;
  EGAPalette: TEGAPalette;
  VGAPalette: TVGAPalette;

  CurrentActivePage: Integer;
  ColorMask: ColorType;

  DummyHGCBkColor: Word;
  CurrentCGABkColor: Word;

procedure FreeAndNil(var q);
var
  tmp : TObject;
begin
  tmp := TObject(q);
  Pointer(q) := Nil;
  tmp.Free;
end;

const
  Double640x200 = True;

function Double320x200: Boolean;
begin
  Double320x200 := not (Has320x200 and FullscreenGraph);
end;

function Double320x240: Boolean;
begin
  Double320x240 := not (Has320x240 and FullscreenGraph);
end;

function ptc_palette_lock: Pointer;
begin
  ptc_palette_lock := PTCWrapperObject.PaletteLock;
end;

procedure ptc_palette_unlock;
begin
  PTCWrapperObject.PaletteUnlock;
end;

function ptc_surface_lock: Pointer;
begin
  ptc_surface_lock := PTCWrapperObject.Lock(CurrentActivePage);
end;

procedure ptc_surface_unlock;
begin
  PTCWrapperObject.Unlock;
end;

procedure ptc_update;
begin
end;

{ ---------------------------------------------------------------------
    Required procedures
  ---------------------------------------------------------------------}

procedure ptc_savevideostate;
begin
end;

procedure ptc_restorevideostate;
begin
  if PTCWrapperObject <> nil then
    PTCWrapperObject.Close;
end;

function VGA6to8(dac6: Uint32): Uint32;
begin
  VGA6to8 := dac6 shl 2;
end;

procedure ptc_InitPalette16(UseCGAEmuPalette: Boolean);
var
  PaletteData: PUint32;
  I: Integer;
  r, g, b: Uint32;
begin
  if UseCGAEmuPalette then
  begin
    VGAPalette := DefaultVGA16Palette_CGAEmu;
    EGAPalette := DefaultEGAPalette_200;
  end
  else
  begin
    VGAPalette := DefaultVGA16Palette;
    EGAPalette := DefaultEGAPalette;
  end;
  EGAPaletteEnabled := True;

  PaletteData := ptc_palette_lock;

  FillChar(PaletteData^, 256*4, 0);
  for I := 0 to 15 do
  begin
    r := VGA6to8(VGAPalette[EGAPalette[I], 0]);
    g := VGA6to8(VGAPalette[EGAPalette[I], 1]);
    b := VGA6to8(VGAPalette[EGAPalette[I], 2]);
    PaletteData[I] := (r shl 16) or (g shl 8) or b;
  end;

  ptc_palette_unlock;
end;

procedure ptc_InitPalette256;
var
  PaletteData: PUint32;
  I: Integer;
  r, g, b: Uint32;
begin
  EGAPaletteEnabled := False;
  VGAPalette := DefaultVGA256Palette;

  PaletteData := ptc_palette_lock;
  for I := 0 to 255 do
  begin
    r := VGA6to8(VGAPalette[I, 0]);
    g := VGA6to8(VGAPalette[I, 1]);
    b := VGA6to8(VGAPalette[I, 2]);
    PaletteData[I] := (r shl 16) or (g shl 8) or b;
  end;
  ptc_palette_unlock;
end;

procedure ptc_SetEGAPalette(ColorNum, Color: Integer);
var
  PaletteData: PUint32;
  r, g, b: Uint32;
begin
  if EGAPalette[ColorNum] <> Color then
  begin
    if (VGAPalette[EGAPalette[ColorNum], 0] <> VGAPalette[Color, 0]) or
       (VGAPalette[EGAPalette[ColorNum], 1] <> VGAPalette[Color, 1]) or
       (VGAPalette[EGAPalette[ColorNum], 2] <> VGAPalette[Color, 2]) then
    begin
      EGAPalette[ColorNum] := Color;
      r := VGA6to8(VGAPalette[Color, 0]);
      g := VGA6to8(VGAPalette[Color, 1]);
      b := VGA6to8(VGAPalette[Color, 2]);

      PaletteData := ptc_palette_lock;
      PaletteData[ColorNum] := (r shl 16) or (g shl 8) or b;
      ptc_palette_unlock;
    end
    else
      EGAPalette[ColorNum] := Color;
  end;
end;

procedure ptc_SetVGAPalette(ColorNum, ARed, AGreen, ABlue: Integer);
var
  PaletteData: PUint32;
  I: Integer;
begin
  if (VGAPalette[ColorNum, 0] <> ARed) or
     (VGAPalette[ColorNum, 1] <> AGreen) or
     (VGAPalette[ColorNum, 2] <> ABlue) then
  begin
    VGAPalette[ColorNum, 0] := ARed;
    VGAPalette[ColorNum, 1] := AGreen;
    VGAPalette[ColorNum, 2] := ABlue;
    if EGAPaletteEnabled then
    begin
      for I := 0 to 15 do
        if EGAPalette[I] = ColorNum then
        begin
          PaletteData := ptc_palette_lock;
          PaletteData[I] := (VGA6to8(ARed) shl 16) or (VGA6to8(AGreen) shl 8) or VGA6to8(ABlue);
          ptc_palette_unlock;
        end;
    end
    else
    begin
      PaletteData := ptc_palette_lock;
      PaletteData[ColorNum] := (VGA6to8(ARed) shl 16) or (VGA6to8(AGreen) shl 8) or VGA6to8(ABlue);
      ptc_palette_unlock;
    end;
  end;
end;

procedure ptc_InitPaletteCGA(CGAPalette: Integer);
var
  PaletteData: PUint32;
  I: Integer;
  r, g, b: Uint32;
begin
  VGAPalette := DefaultVGA16Palette_CGAEmu;
  FillChar(EGAPalette, SizeOf(EGAPalette), 0);
  EGAPaletteEnabled := True;

  case CGAPalette of
    0:
      begin
        EGAPalette[1] := $12;
        EGAPalette[2] := $14;
        EGAPalette[3] := $16;
      end;
    1:
      begin
        EGAPalette[1] := $13;
        EGAPalette[2] := $15;
        EGAPalette[3] := $17;
      end;
    2:
      begin
        EGAPalette[1] := $02;
        EGAPalette[2] := $04;
        EGAPalette[3] := $06;
      end;
    3:
      begin
        EGAPalette[1] := $03;
        EGAPalette[2] := $05;
        EGAPalette[3] := $07;
      end;
  end;

  PaletteData := ptc_palette_lock;
  FillChar(PaletteData^, 256*4, 0);
  for I := 0 to 3 do
  begin
    r := VGA6to8(VGAPalette[EGAPalette[I], 0]);
    g := VGA6to8(VGAPalette[EGAPalette[I], 1]);
    b := VGA6to8(VGAPalette[EGAPalette[I], 2]);
    PaletteData[I] := (r shl 16) or (g shl 8) or b;
  end;
  ptc_palette_unlock;
  CurrentCGABkColor := 0;
end;

procedure ptc_InitPaletteCGA2;
var
  PaletteData: PUint32;
  I: Integer;
  r, g, b: Uint32;
begin
  VGAPalette := DefaultVGA16Palette_CGAEmu;
  FillChar(EGAPalette, SizeOf(EGAPalette), 0);
  EGAPaletteEnabled := True;

  for I := 1 to 15 do
    EGAPalette[I] := 63;

  PaletteData := ptc_palette_lock;
  FillChar(PaletteData^, 256*4, 0);
  for I := 0 to 1 do
  begin
    r := VGA6to8(VGAPalette[EGAPalette[I], 0]);
    g := VGA6to8(VGAPalette[EGAPalette[I], 1]);
    b := VGA6to8(VGAPalette[EGAPalette[I], 2]);
    PaletteData[I] := (r shl 16) or (g shl 8) or b;
  end;
  ptc_palette_unlock;
  CurrentCGABkColor := 0;
end;

procedure ptc_InitPaletteMCGA2;
var
  PaletteData: PUint32;
  I: Integer;
  r, g, b: Uint32;
begin
  VGAPalette := DefaultVGA16Palette;
  FillChar(EGAPalette, SizeOf(EGAPalette), 0);
  EGAPaletteEnabled := True;

  for I := 1 to 15 do
    EGAPalette[I] := 63;

  PaletteData := ptc_palette_lock;
  FillChar(PaletteData^, 256*4, 0);
  for I := 0 to 1 do
  begin
    r := VGA6to8(VGAPalette[EGAPalette[I], 0]);
    g := VGA6to8(VGAPalette[EGAPalette[I], 1]);
    b := VGA6to8(VGAPalette[EGAPalette[I], 2]);
    PaletteData[I] := (r shl 16) or (g shl 8) or b;
  end;
  ptc_palette_unlock;
  CurrentCGABkColor := 0;
end;

procedure ptc_InternalOpen(const ATitle: string; AWidth, AHeight: Integer; AFormat: IPTCFormat; AVirtualPages: Integer);
var
  ConsoleWidth, ConsoleHeight: Integer;
begin
  ConsoleWidth := AWidth;
  ConsoleHeight := AHeight;

  if Double320x200 and (AWidth = 320) and (AHeight = 200) then
  begin
    ConsoleWidth := 640;
    ConsoleHeight := 400;
  end;

  if Double320x240 and (AWidth = 320) and (AHeight = 240) then
  begin
    ConsoleWidth := 640;
    ConsoleHeight := 480;
  end;

  if Double640x200 and (AWidth = 640) and (AHeight = 200) then
  begin
    ConsoleWidth := 640;
    ConsoleHeight := 400;
  end;

  if FullscreenGraph then
    PTCWrapperObject.Option('fullscreen output')
  else
    PTCWrapperObject.Option('windowed output');

  PTCWrapperObject.Open(ATitle, AWidth, AHeight, ConsoleWidth, ConsoleHeight, AFormat, AVirtualPages, 0);
end;

procedure ptc_InternalInitMode16(XResolution, YResolution, Pages: LongInt; UseCGAEmuPalette: Boolean);
begin
{$IFDEF logging}
  LogLn('Initializing mode ' + strf(XResolution) + ', ' + strf(YResolution) + ' 16 colours');
{$ENDIF logging}
  { open the console }
  ptc_InternalOpen(WindowTitle, XResolution, YResolution, PTCFormat8, Pages);
  PTCWidth := XResolution;
  PTCHeight := YResolution;
  CurrentActivePage := 0;
  { create palette }
  ptc_InitPalette16(UseCGAEmuPalette);
  ColorMask := 15;
end;

procedure ptc_InitMode16(XResolution, YResolution, Pages: LongInt);
begin
  ptc_InternalInitMode16(XResolution, YResolution, Pages, False);
end;

procedure ptc_InitMode16_CGAEmu(XResolution, YResolution, Pages: LongInt);
begin
  ptc_InternalInitMode16(XResolution, YResolution, Pages, True);
end;

procedure ptc_InitMode256(XResolution, YResolution, Pages: LongInt);
begin
{$IFDEF logging}
  LogLn('Initializing mode ' + strf(XResolution) + ', ' + strf(YResolution) + ' 256 colours');
{$ENDIF logging}
  { open the console }
  ptc_InternalOpen(WindowTitle, XResolution, YResolution, PTCFormat8, Pages);
  PTCWidth := XResolution;
  PTCHeight := YResolution;
  CurrentActivePage := 0;
  { create palette }
  ptc_InitPalette256;
  ColorMask := 255;
end;

procedure ptc_InitModeCGA4(XResolution, YResolution, CGAPalette: LongInt);
begin
{$IFDEF logging}
  LogLn('Initializing mode ' + strf(XResolution) + ', ' + strf(YResolution) + ' 4 colours, palette ' + strf(CGAPalette));
{$ENDIF logging}
  { open the console }
  ptc_InternalOpen(WindowTitle, XResolution, YResolution, PTCFormat8, 1);
  PTCWidth := XResolution;
  PTCHeight := YResolution;
  CurrentActivePage := 0;
  { create palette }
  ptc_InitPaletteCGA(CGAPalette);
  ColorMask := 3;
end;

procedure ptc_InitModeCGA2(XResolution, YResolution, Pages: LongInt);
begin
{$IFDEF logging}
  LogLn('Initializing mode ' + strf(XResolution) + ', ' + strf(YResolution) + ' 2 colours');
{$ENDIF logging}
  { open the console }
  ptc_InternalOpen(WindowTitle, XResolution, YResolution, PTCFormat8, Pages);
  PTCWidth := XResolution;
  PTCHeight := YResolution;
  CurrentActivePage := 0;
  { create palette }
  ptc_InitPaletteCGA2;
  ColorMask := 1;
end;

procedure ptc_InitModeMCGA2(XResolution, YResolution, Pages: LongInt);
begin
{$IFDEF logging}
  LogLn('Initializing mode ' + strf(XResolution) + ', ' + strf(YResolution) + ' 2 colours');
{$ENDIF logging}
  { open the console }
  ptc_InternalOpen(WindowTitle, XResolution, YResolution, PTCFormat8, Pages);
  PTCWidth := XResolution;
  PTCHeight := YResolution;
  CurrentActivePage := 0;
  { create palette }
  ptc_InitPaletteMCGA2;
  ColorMask := 1;
end;

procedure ptc_InitMode32k(XResolution, YResolution, Pages: LongInt);
begin
{$IFDEF logging}
  LogLn('Initializing mode ' + strf(XResolution) + ', ' + strf(YResolution) + ' 32768 colours');
{$ENDIF logging}
  { open the console }
  ptc_InternalOpen(WindowTitle, XResolution, YResolution, PTCFormat15, Pages);
  PTCWidth := XResolution;
  PTCHeight := YResolution;
  CurrentActivePage := 0;
  ColorMask := 32767;
end;

procedure ptc_InitMode64k(XResolution, YResolution, Pages: LongInt);
begin
{$IFDEF logging}
  LogLn('Initializing mode ' + strf(XResolution) + ', ' + strf(YResolution) + ' 65536 colours');
{$ENDIF logging}
  { open the console }
  ptc_InternalOpen(WindowTitle, XResolution, YResolution, PTCFormat16, Pages);
  PTCWidth := XResolution;
  PTCHeight := YResolution;
  CurrentActivePage := 0;
  ColorMask := 65535;
end;

{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
procedure ptc_InitMode32bpp(XResolution, YResolution, Pages: LongInt);
begin
{$IFDEF logging}
  LogLn('Initializing mode ' + strf(XResolution) + ', ' + strf(YResolution) + ' 16777216 colours (32bpp)');
{$ENDIF logging}
  { open the console }
  ptc_InternalOpen(WindowTitle, XResolution, YResolution, PTCFormat32, Pages);
  PTCWidth := XResolution;
  PTCHeight := YResolution;
  CurrentActivePage := 0;
  ColorMask := 16777215;
end;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}


procedure ptc_Init640x200x16;
begin
  ptc_InitMode16_CGAEmu(640, 200, 4);
end;

procedure ptc_Init640x350x16;
begin
  ptc_InitMode16(640, 350, 2);
end;

procedure ptc_Init640x480x16;
begin
  ptc_InitMode16(640, 480, 1);
end;

procedure ptc_Init800x600x16;
begin
  ptc_InitMode16(800, 600, 2);
end;

procedure ptc_Init1024x768x16;
begin
  ptc_InitMode16(1024, 768, 2);
end;

procedure ptc_Init1280x1024x16;
begin
  ptc_InitMode16(1280, 1024, 2);
end;

procedure ptc_Init320x200x4cgaC0;
begin
  ptc_InitModeCGA4(320, 200, 0);
end;

procedure ptc_Init320x200x4cgaC1;
begin
  ptc_InitModeCGA4(320, 200, 1);
end;

procedure ptc_Init320x200x4cgaC2;
begin
  ptc_InitModeCGA4(320, 200, 2);
end;

procedure ptc_Init320x200x4cgaC3;
begin
  ptc_InitModeCGA4(320, 200, 3);
end;

procedure ptc_Init640x200x2;
begin
  ptc_InitModeCGA2(640, 200, 1);
end;

procedure ptc_Init640x480x2;
begin
  ptc_InitModeMCGA2(640, 480, 1);
end;

procedure ptc_Init720x348x2;
begin
  DummyHGCBkColor := 0;
  ptc_InitModeCGA2(720, 348, 2);
end;

procedure ptc_Init320x200x256;
begin
  ptc_InitMode256(320, 200, 4);
end;

procedure ptc_Init640x400x256;
begin
  ptc_InitMode256(640, 400, 2);
end;

procedure ptc_Init640x480x256;
begin
  ptc_InitMode256(640, 480, 2);
end;

procedure ptc_Init800x600x256;
begin
  ptc_InitMode256(800, 600, 2);
end;

procedure ptc_Init1024x768x256;
begin
  ptc_InitMode256(1024, 768, 2);
end;

procedure ptc_Init1280x1024x256;
begin
  ptc_InitMode256(1280, 1024, 2);
end;

procedure ptc_Init320x200x32k;
begin
  ptc_InitMode32k(320, 200, 2);
end;

procedure ptc_Init640x480x32k;
begin
  ptc_InitMode32k(640, 480, 2);
end;

procedure ptc_Init800x600x32k;
begin
  ptc_InitMode32k(800, 600, 2);
end;

procedure ptc_Init1024x768x32k;
begin
  ptc_InitMode32k(1024, 768, 2);
end;

procedure ptc_Init1280x1024x32k;
begin
  ptc_InitMode32k(1280, 1024, 2);
end;

procedure ptc_Init320x200x64k;
begin
  ptc_InitMode64k(320, 200, 2);
end;

procedure ptc_Init640x480x64k;
begin
  ptc_InitMode64k(640, 480, 2);
end;

procedure ptc_Init800x600x64k;
begin
  ptc_InitMode64k(800, 600, 2);
end;

procedure ptc_Init1024x768x64k;
begin
  ptc_InitMode64k(1024, 768, 2);
end;

procedure ptc_Init1280x1024x64k;
begin
  ptc_InitMode64k(1280, 1024, 2);
end;

{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
procedure ptc_Init320x200x32bpp;
begin
  ptc_InitMode32bpp(320, 200, 2);
end;

procedure ptc_Init640x480x32bpp;
begin
  ptc_InitMode32bpp(640, 480, 2);
end;

procedure ptc_Init800x600x32bpp;
begin
  ptc_InitMode32bpp(800, 600, 2);
end;

procedure ptc_Init1024x768x32bpp;
begin
  ptc_InitMode32bpp(1024, 768, 2);
end;

procedure ptc_Init1280x1024x32bpp;
begin
  ptc_InitMode32bpp(1280, 1024, 2);
end;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

procedure ptc_InitNonStandard16;
begin
  ptc_InitMode16(MaxX + 1, MaxY + 1, 2);
end;

procedure ptc_InitNonStandard256;
begin
  ptc_InitMode256(MaxX + 1, MaxY + 1, 2);
end;

procedure ptc_InitNonStandard32k;
begin
  ptc_InitMode32k(MaxX + 1, MaxY + 1, 2);
end;

procedure ptc_InitNonStandard64k;
begin
  ptc_InitMode64k(MaxX + 1, MaxY + 1, 2);
end;

{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
procedure ptc_InitNonStandard32bpp;
begin
  ptc_InitMode32bpp(MaxX + 1, MaxY + 1, 2);
end;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

procedure ptc_SetVisualPage(page: word);
begin
  if page > HardwarePages then
    exit;

  PTCWrapperObject.SetVisualPage(page);
end;

procedure ptc_SetActivePage(page: word);
begin
  if page > HardwarePages then
    exit;

  CurrentActivePage := page;
end;

{ compatible with TP7's HERC.BGI }
procedure SetBkColorHGC720(ColorNum: ColorType);
begin
  if ColorNum > 15 then
    exit;
  DummyHGCBkColor := ColorNum;
end;

{ compatible with TP7's HERC.BGI }
function GetBkColorHGC720: ColorType;
begin
  GetBkColorHGC720 := DummyHGCBkColor;
end;

procedure SetBkColorCGA320(ColorNum: ColorType);
begin
  if ColorNum > 15 then
    exit;
  CurrentCGABkColor := ColorNum;

  ptc_SetEGAPalette(0, ((ColorNum shl 1) and $10) or (ColorNum and $07));
end;

function GetBkColorCGA320: ColorType;
begin
  GetBkColorCGA320 := CurrentCGABkColor;
end;

{yes, TP7 CGA.BGI behaves *exactly* like that}
procedure SetBkColorCGA640(ColorNum: ColorType);
begin
  if ColorNum > 15 then
    exit;
  CurrentCGABkColor := ColorNum;
  if ColorNum = 0 then
    exit;
  ptc_SetEGAPalette(1, ((ColorNum shl 1) and $10) or (ColorNum and $07));
end;

function GetBkColorCGA640: ColorType;
begin
  GetBkColorCGA640 := CurrentCGABkColor;
end;

{ nickysn: VGA compatible implementation. I don't have a real MCGA to test
  if there's any difference with VGA }
procedure SetBkColorMCGA640(ColorNum: ColorType);
begin
  if ColorNum > 15 then
    exit;
  CurrentCGABkColor := ColorNum;

  ptc_SetEGAPalette(0, ((ColorNum shl 1) and $10) or (ColorNum and $07));
end;

function GetBkColorMCGA640: ColorType;
begin
  GetBkColorMCGA640 := CurrentCGABkColor;
end;

Function ClipCoords (Var X,Y : smallint) : Boolean;
{ Adapt to viewport, return TRUE if still in viewport,
  false if outside viewport}

begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  ClipCoords:=Not ClipPixels;
  if ClipPixels then
    Begin
    ClipCoords:=(X < StartXViewPort) or (X > (StartXViewPort + ViewWidth));
    ClipCoords:=ClipCoords or
               ((Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)));
    ClipCoords:=Not ClipCoords;
    end;
end;

{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
procedure ptc_DirectPixelProc_32bpp(X,Y: smallint);
var
  pixels:Plongword;
begin
//  Writeln('ptc_DirectPixelProc_32bpp(', X, ', ', Y, ')');
  pixels := ptc_surface_lock;
  case CurrentWriteMode of
    XORPut:
      begin
        pixels[x+y*PTCWidth] := pixels[x+y*PTCWidth] xor CurrentColor;
      end;
    OrPut:
      begin
        pixels[x+y*PTCWidth] := pixels[x+y*PTCWidth] or CurrentColor;
      end;
    AndPut:
      begin
        pixels[x+y*PTCWidth] := pixels[x+y*PTCWidth] and CurrentColor;
      end;
    NotPut:
      begin
        pixels[x+y*PTCWidth] := CurrentColor xor $FFFFFF;
      end
  else
    pixels[x+y*PTCWidth] := CurrentColor;
  end;
  ptc_surface_unlock;
  ptc_update;
end;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

procedure ptc_DirectPixelProc_16bpp(X,Y: smallint);
var
  pixels:Pword;
begin
//  Writeln('ptc_DirectPixelProc_16bpp(', X, ', ', Y, ')');
  pixels := ptc_surface_lock;
  case CurrentWriteMode of
    XORPut:
      begin
        pixels[x+y*PTCWidth] := pixels[x+y*PTCWidth] xor CurrentColor;
      end;
    OrPut:
      begin
        pixels[x+y*PTCWidth] := pixels[x+y*PTCWidth] or CurrentColor;
      end;
    AndPut:
      begin
        pixels[x+y*PTCWidth] := pixels[x+y*PTCWidth] and CurrentColor;
      end;
    NotPut:
      begin
        pixels[x+y*PTCWidth] := CurrentColor xor $FFFF;
      end
  else
    pixels[x+y*PTCWidth] := CurrentColor;
  end;
  ptc_surface_unlock;
  ptc_update;
end;

procedure ptc_DirectPixelProc_8bpp(X,Y: smallint);
var
  pixels:PByte;
begin
//  Writeln('ptc_DirectPixelProc_8bpp(', X, ', ', Y, ')');
  pixels := ptc_surface_lock;
  case CurrentWriteMode of
    XORPut:
      begin
        pixels[x+y*PTCWidth] := pixels[x+y*PTCWidth] xor (CurrentColor and ColorMask);
      end;
    OrPut:
      begin
        pixels[x+y*PTCWidth] := pixels[x+y*PTCWidth] or (CurrentColor and ColorMask);
      end;
    AndPut:
      begin
        pixels[x+y*PTCWidth] := pixels[x+y*PTCWidth] and (CurrentColor and ColorMask);
      end;
    NotPut:
      begin
        pixels[x+y*PTCWidth] := CurrentColor xor ColorMask;
      end
  else
    pixels[x+y*PTCWidth] := CurrentColor and ColorMask;
  end;
  ptc_surface_unlock;
  ptc_update;
end;

{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
procedure ptc_putpixelproc_32bpp(X,Y:smallint;Color:ColorType);

var pixels:Plongword;

begin
//  Writeln('ptc_putpixelproc_32bpp(', X, ', ', Y, ', ', Color, ')');
  if clipcoords(X,Y) then
    begin
      pixels := ptc_surface_lock;
      {Plot the pixel on the surface.}
      pixels[x+y*PTCWidth] := color;
      ptc_surface_unlock;
      ptc_update;
    end;
end;

function ptc_getpixelproc_32bpp(X,Y: smallint):ColorType;

var pixels:Plongword;

begin
  if clipcoords(X,Y) then
    begin
      pixels := ptc_surface_lock;
      {Get the pixel from the surface.}
      ptc_getpixelproc_32bpp:=pixels[x+y*PTCWidth];
      ptc_surface_unlock;
    end;
end;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

procedure ptc_putpixelproc_16bpp(X,Y:smallint;Color:ColorType);

var pixels:Pword;

begin
//  Writeln('ptc_putpixelproc_16bpp(', X, ', ', Y, ', ', Color, ')');
  if clipcoords(X,Y) then
    begin
      pixels := ptc_surface_lock;
      {Plot the pixel on the surface.}
      pixels[x+y*PTCWidth] := color;
      ptc_surface_unlock;
      ptc_update;
    end;
end;

function ptc_getpixelproc_16bpp(X,Y: smallint):ColorType;

var pixels:Pword;

begin
  if clipcoords(X,Y) then
    begin
      pixels := ptc_surface_lock;
      {Get the pixel from the surface.}
      ptc_getpixelproc_16bpp:=pixels[x+y*PTCWidth];
      ptc_surface_unlock;
    end;
end;

procedure ptc_PutPixelProc_8bpp(X,Y:smallint;Color:ColorType);

var pixels:PByte;

begin
//  Writeln('ptc_PutPixelProc_8bpp(', X, ', ', Y, ', ', Color, ')');
  if clipcoords(X,Y) then
    begin
      pixels := ptc_surface_lock;
      {Plot the pixel on the surface.}
      pixels[x+y*PTCWidth]:=color and ColorMask;
      ptc_surface_unlock;
      ptc_update;
    end;
end;

function ptc_GetPixelProc_8bpp(X,Y: smallint):ColorType;

var pixels:PByte;

begin
  if clipcoords(X,Y) then
    begin
      pixels := ptc_surface_lock;
      {Get the pixel from the surface.}
      ptc_GetPixelProc_8bpp:=pixels[x+y*PTCWidth] and ColorMask;
      ptc_surface_unlock;
      ptc_update;
    end;
end;

{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
procedure ptc_HLineProc_32bpp(x, x2,y : smallint);

var pixels:Plongword;
    i:word;
    xtmp: smallint;

begin
//  Writeln('ptc_HLineProc_32bpp(', x, ', ', x2, ', ', y, ')');
  { must we swap the values? }
  if x >= x2 then
  begin
    xtmp := x2;
    x2 := x;
    x:= xtmp;
  end;

  inc(x,StartXViewPort);
  inc(x2,StartXViewPort);
  inc(y,StartYViewPort);
  if ClipPixels then
  begin
    if LineClipped(x,y,x2,y,StartXViewPort,StartYViewPort,
               StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;
  end;

  pixels := ptc_surface_lock;

  case CurrentWriteMode of
    XORPut:
      begin
        for i:=x to x2 do
          pixels[i+y*PTCWidth] := pixels[i+y*PTCWidth] xor CurrentColor;
      end;
    OrPut:
      begin
        for i:=x to x2 do
          pixels[i+y*PTCWidth] := pixels[i+y*PTCWidth] or CurrentColor;
      end;
    AndPut:
      begin
        for i:=x to x2 do
          pixels[i+y*PTCWidth] := pixels[i+y*PTCWidth] and CurrentColor;
      end;
    NotPut:
      begin
        for i:=x to x2 do
          pixels[i+y*PTCWidth] := CurrentColor xor $FFFFFF;
      end
  else
    for i:=x to x2 do
      pixels[i+y*PTCWidth] := CurrentColor;
  end;

  ptc_surface_unlock;
  ptc_update;
end;

procedure ptc_VLineProc_32bpp(x,y,y2 : smallint);
var pixels:PLongWord;
    i:word;
    ytmp: smallint;
begin
  if y >= y2 then
   begin
     ytmp := y2;
     y2 := y;
     y:= ytmp;
   end;

  inc(x,StartXViewPort);
  inc(y,StartYViewPort);
  inc(y2,StartYViewPort);
  if ClipPixels then
  begin
    if LineClipped(x,y,x,y2,StartXViewPort,StartYViewPort,
          StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;
  end;

  pixels := ptc_surface_lock;

  case CurrentWriteMode of
    XORPut:
      begin
        for i:=y to y2 do
          pixels[x+i*PTCWidth] := pixels[x+i*PTCWidth] xor CurrentColor;
      end;
    OrPut:
      begin
        for i:=y to y2 do
          pixels[x+i*PTCWidth] := pixels[x+i*PTCWidth] or CurrentColor;
      end;
    AndPut:
      begin
        for i:=y to y2 do
          pixels[x+i*PTCWidth] := pixels[x+i*PTCWidth] and CurrentColor;
      end;
    NotPut:
      begin
        for i:=y to y2 do
          pixels[x+i*PTCWidth] := CurrentColor xor $FFFFFF;
      end
  else
    for i:=y to y2 do
      pixels[x+i*PTCWidth] := CurrentColor;
  end;

  ptc_surface_unlock;
  ptc_update;
end;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

procedure ptc_HLineProc_16bpp(x, x2,y : smallint);

var pixels:Pword;
    i:word;
    xtmp: smallint;

begin
//  Writeln('ptc_HLineProc_16bpp(', x, ', ', x2, ', ', y, ')');
  { must we swap the values? }
  if x >= x2 then
  begin
    xtmp := x2;
    x2 := x;
    x:= xtmp;
  end;

  inc(x,StartXViewPort);
  inc(x2,StartXViewPort);
  inc(y,StartYViewPort);
  if ClipPixels then
  begin
    if LineClipped(x,y,x2,y,StartXViewPort,StartYViewPort,
               StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;
  end;

  pixels := ptc_surface_lock;

  case CurrentWriteMode of
    XORPut:
      begin
        for i:=x to x2 do
          pixels[i+y*PTCWidth] := pixels[i+y*PTCWidth] xor CurrentColor;
      end;
    OrPut:
      begin
        for i:=x to x2 do
          pixels[i+y*PTCWidth] := pixels[i+y*PTCWidth] or CurrentColor;
      end;
    AndPut:
      begin
        for i:=x to x2 do
          pixels[i+y*PTCWidth] := pixels[i+y*PTCWidth] and CurrentColor;
      end;
    NotPut:
      begin
        for i:=x to x2 do
          pixels[i+y*PTCWidth] := CurrentColor xor $FFFF;
      end
  else
    for i:=x to x2 do
      pixels[i+y*PTCWidth] := CurrentColor;
  end;

  ptc_surface_unlock;
  ptc_update;
end;

procedure ptc_VLineProc_16bpp(x,y,y2 : smallint);
var pixels:PWord;
    i:word;
    ytmp: smallint;
begin
  if y >= y2 then
   begin
     ytmp := y2;
     y2 := y;
     y:= ytmp;
   end;

  inc(x,StartXViewPort);
  inc(y,StartYViewPort);
  inc(y2,StartYViewPort);
  if ClipPixels then
  begin
    if LineClipped(x,y,x,y2,StartXViewPort,StartYViewPort,
          StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;
  end;

  pixels := ptc_surface_lock;

  case CurrentWriteMode of
    XORPut:
      begin
        for i:=y to y2 do
          pixels[x+i*PTCWidth] := pixels[x+i*PTCWidth] xor CurrentColor;
      end;
    OrPut:
      begin
        for i:=y to y2 do
          pixels[x+i*PTCWidth] := pixels[x+i*PTCWidth] or CurrentColor;
      end;
    AndPut:
      begin
        for i:=y to y2 do
          pixels[x+i*PTCWidth] := pixels[x+i*PTCWidth] and CurrentColor;
      end;
    NotPut:
      begin
        for i:=y to y2 do
          pixels[x+i*PTCWidth] := CurrentColor xor $FFFF;
      end
  else
    for i:=y to y2 do
      pixels[x+i*PTCWidth] := CurrentColor;
  end;

  ptc_surface_unlock;
  ptc_update;
end;

procedure ptc_HLineProc_8bpp(x, x2,y : smallint);

var pixels:PByte;
    i:word;
    xtmp: smallint;

begin
//  Writeln('ptc_HLineProc_8bpp(', x, ', ', x2, ', ', y, ')');
  { must we swap the values? }
  if x >= x2 then
  begin
    xtmp := x2;
    x2 := x;
    x:= xtmp;
  end;

  inc(x,StartXViewPort);
  inc(x2,StartXViewPort);
  inc(y,StartYViewPort);
  if ClipPixels then
  begin
    if LineClipped(x,y,x2,y,StartXViewPort,StartYViewPort,
               StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;
  end;

  pixels := ptc_surface_lock;

  case CurrentWriteMode of
    XORPut:
      begin
        for i:=x to x2 do
          pixels[i+y*PTCWidth] := pixels[i+y*PTCWidth] xor (CurrentColor and ColorMask);
      end;
    OrPut:
      begin
        for i:=x to x2 do
          pixels[i+y*PTCWidth] := pixels[i+y*PTCWidth] or (CurrentColor and ColorMask);
      end;
    AndPut:
      begin
        for i:=x to x2 do
          pixels[i+y*PTCWidth] := pixels[i+y*PTCWidth] and (CurrentColor and ColorMask);
      end;
    NotPut:
      begin
        for i:=x to x2 do
          pixels[i+y*PTCWidth] := (CurrentColor and ColorMask) xor ColorMask;
      end
  else
    for i:=x to x2 do
      pixels[i+y*PTCWidth] := CurrentColor and ColorMask;
  end;

  ptc_surface_unlock;
  ptc_update;
end;

procedure ptc_VLineProc_8bpp(x,y,y2 : smallint);

var pixels:PByte;
    i:word;
    ytmp: smallint;

begin
//  Writeln('ptc_VLineProc_8bpp(', x, ', ', y, ', ', y2, ')');
  { must we swap the values? }
    if y >= y2 then
     Begin
       ytmp := y2;
       y2 := y;
       y:= ytmp;
     end;

  inc(x,StartXViewPort);
  inc(y,StartYViewPort);
  inc(y2,StartYViewPort);
  if ClipPixels then
  begin
    if LineClipped(x,y,x,y2,StartXViewPort,StartYViewPort,
          StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;
  end;

  pixels := ptc_surface_lock;

  case CurrentWriteMode of
    XORPut:
      begin
        for i:=y to y2 do
          pixels[x+i*PTCWidth] := pixels[x+i*PTCWidth] xor (CurrentColor and ColorMask);
      end;
    OrPut:
      begin
        for i:=y to y2 do
          pixels[x+i*PTCWidth] := pixels[x+i*PTCWidth] or (CurrentColor and ColorMask);
      end;
    AndPut:
      begin
        for i:=y to y2 do
          pixels[x+i*PTCWidth] := pixels[x+i*PTCWidth] and (CurrentColor and ColorMask);
      end;
    NotPut:
      begin
        for i:=y to y2 do
          pixels[x+i*PTCWidth] := (CurrentColor and ColorMask) xor ColorMask;
      end
  else
    for i:=y to y2 do
      pixels[x+i*PTCWidth] := CurrentColor and ColorMask;
  end;

  ptc_surface_unlock;
  ptc_update;
end;

procedure ptc_PatternLineProc_8bpp(x1,x2,y: smallint);
{********************************************************}
{ Draws a horizontal patterned line according to the     }
{ current Fill Settings.                                 }
{********************************************************}
{ Important notes:                                       }
{  - CurrentColor must be set correctly before entering  }
{    this routine.                                       }
{********************************************************}
var
  pixels: PByte;
  NrIterations: smallint;
  i           : smallint;
  j           : smallint;
  TmpFillPattern : byte;
begin
  { convert to global coordinates ... }
  x1 := x1 + StartXViewPort;
  x2 := x2 + StartXViewPort;
  y  := y + StartYViewPort;
  { if line was fully clipped then exit...}
  if LineClipped(x1,y,x2,y,StartXViewPort,StartYViewPort,
     StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;

  { Get the current pattern }
  TmpFillPattern := FillPatternTable
    [FillSettings.Pattern][(y and $7)+1];

  pixels := ptc_surface_lock;

  { number of times to go throuh the 8x8 pattern }
  NrIterations := abs(x2 - x1+8) div 8;
  for i := 0 to NrIterations do
    for j := 0 to 7 do
    begin
      { x1 mod 8 }
      if RevBitArray[x1 and 7] and TmpFillPattern <> 0 then
        pixels[x1+y*PTCWidth] := FillSettings.Color and ColorMask
      else
        pixels[x1+y*PTCWidth] := CurrentBkColor and ColorMask;
      Inc(x1);
      if x1 > x2 then
      begin
        ptc_surface_unlock;
        ptc_update;
        exit;
      end;
    end;
  ptc_surface_unlock;
  ptc_update;
end;

procedure ptc_PatternLineProc_16bpp(x1,x2,y: smallint);
{********************************************************}
{ Draws a horizontal patterned line according to the     }
{ current Fill Settings.                                 }
{********************************************************}
{ Important notes:                                       }
{  - CurrentColor must be set correctly before entering  }
{    this routine.                                       }
{********************************************************}
var
  pixels: PWord;
  NrIterations: smallint;
  i           : smallint;
  j           : smallint;
  TmpFillPattern : byte;
begin
  { convert to global coordinates ... }
  x1 := x1 + StartXViewPort;
  x2 := x2 + StartXViewPort;
  y  := y + StartYViewPort;
  { if line was fully clipped then exit...}
  if LineClipped(x1,y,x2,y,StartXViewPort,StartYViewPort,
     StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;

  { Get the current pattern }
  TmpFillPattern := FillPatternTable
    [FillSettings.Pattern][(y and $7)+1];

  pixels := ptc_surface_lock;

  { number of times to go throuh the 8x8 pattern }
  NrIterations := abs(x2 - x1+8) div 8;
  for i := 0 to NrIterations do
    for j := 0 to 7 do
    begin
      { x1 mod 8 }
      if RevBitArray[x1 and 7] and TmpFillPattern <> 0 then
        pixels[x1+y*PTCWidth] := FillSettings.Color
      else
        pixels[x1+y*PTCWidth] := CurrentBkColor;
      Inc(x1);
      if x1 > x2 then
      begin
        ptc_surface_unlock;
        ptc_update;
        exit;
      end;
    end;
  ptc_surface_unlock;
  ptc_update;
end;

{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
procedure ptc_PatternLineProc_32bpp(x1,x2,y: smallint);
{********************************************************}
{ Draws a horizontal patterned line according to the     }
{ current Fill Settings.                                 }
{********************************************************}
{ Important notes:                                       }
{  - CurrentColor must be set correctly before entering  }
{    this routine.                                       }
{********************************************************}
var
  pixels: PLongWord;
  NrIterations: smallint;
  i           : smallint;
  j           : smallint;
  TmpFillPattern : byte;
begin
  { convert to global coordinates ... }
  x1 := x1 + StartXViewPort;
  x2 := x2 + StartXViewPort;
  y  := y + StartYViewPort;
  { if line was fully clipped then exit...}
  if LineClipped(x1,y,x2,y,StartXViewPort,StartYViewPort,
     StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;

  { Get the current pattern }
  TmpFillPattern := FillPatternTable
    [FillSettings.Pattern][(y and $7)+1];

  pixels := ptc_surface_lock;

  { number of times to go throuh the 8x8 pattern }
  NrIterations := abs(x2 - x1+8) div 8;
  for i := 0 to NrIterations do
    for j := 0 to 7 do
    begin
      { x1 mod 8 }
      if RevBitArray[x1 and 7] and TmpFillPattern <> 0 then
        pixels[x1+y*PTCWidth] := FillSettings.Color
      else
        pixels[x1+y*PTCWidth] := CurrentBkColor;
      Inc(x1);
      if x1 > x2 then
      begin
        ptc_surface_unlock;
        ptc_update;
        exit;
      end;
    end;
  ptc_surface_unlock;
  ptc_update;
end;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

procedure ptc_SetRGBAllPaletteProc(const Palette: PaletteType);
begin
  {...}
end;

procedure ptc_setrgbpaletteproc(ColorNum, RedValue, GreenValue, BlueValue: smallint);
begin
  { NOTE: this makes the function compatible to the go32v2 graph implementation, but
    *not* with TP7 }
  if EGAPaletteEnabled then
    ColorNum := DefaultEGAPalette[ColorNum and 15];

  ptc_SetVGAPalette(ColorNum,RedValue shr 2,GreenValue shr 2,BlueValue shr 2);
end;

procedure ptc_getrgbpaletteproc (ColorNum: smallint;
                                    var RedValue, GreenValue, BlueValue: smallint);
begin
  { NOTE: this makes the function compatible to the go32v2 graph implementation, but
    *not* with TP7 }
  if EGAPaletteEnabled then
    ColorNum := DefaultEGAPalette[ColorNum and 15];

  RedValue := VGAPalette[ColorNum, 0] shl 2;
  GreenValue := VGAPalette[ColorNum, 1] shl 2;
  BlueValue := VGAPalette[ColorNum, 2] shl 2;
end;

{**********************************************************}
{ Procedure PutImage()                                     }
{----------------------------------------------------------}
{ Displays the image contained in a bitmap starting at X,Y }
{ the first 2 bytes of the bitmap structure define the     }
{ width and height of the bitmap                           }
{ note: This optomized version does not use PutPixel       }
{   Which would be checking the viewport for every pixel   }
{   Instead it just does it's own viewport check once then }
{   puts all the pixels within the veiwport without further}
{   checking.  Also instead of checking BitBlt every pixel }
{   it is only checked once before all the pixels are      }
{   displayed at once   (JMR)                              }
{**********************************************************}

Procedure ptc_PutImageproc_8bpp(X,Y: smallint; var Bitmap; BitBlt: Word);
type
  pt = array[0..{$ifdef cpu16}16382{$else}$fffffff{$endif}] of word;
  ptw = array[0..2] of longint;
var
  pixels:Pbyte;
  k: longint;
  i, j, y1, x1, deltaX, deltaX1, deltaY: smallint;
  JxW, I_JxW: Longword;
Begin
  inc(x,startXViewPort);
  inc(y,startYViewPort);
  { width/height are 1-based, coordinates are zero based }
  x1 := ptw(Bitmap)[0]+x-1; { get width and adjust end coordinate accordingly }
  y1 := ptw(Bitmap)[1]+y-1; { get height and adjust end coordinate accordingly }
  deltaY := 0;
  deltaX := 0;
  deltaX1 := 0;
  k := 3 * sizeOf(Longint) div sizeOf(Word); { Three reserved longs at start of bitmap }
 { check which part of the image is in the viewport }
  if clipPixels then
    begin
      if y < startYViewPort then
        begin
          deltaY := startYViewPort - y;
          inc(k,(x1-x+1)*deltaY);
          y := startYViewPort;
         end;
      if y1 > startYViewPort+viewHeight then
        y1 := startYViewPort+viewHeight;
      if x < startXViewPort then
        begin
          deltaX := startXViewPort-x;
          x := startXViewPort;
        end;
      if x1 > startXViewPort + viewWidth then
        begin
          deltaX1 := x1 - (startXViewPort + viewWidth);
          x1 := startXViewPort + viewWidth;
        end;
    end;
  pixels := ptc_surface_lock;
  case BitBlt of
    XORPut:
      Begin
        for j:=Y to Y1 do
          Begin
            JxW:=j*PTCWidth;
            inc(k,deltaX);
            for i:=X to X1 do
              begin
                I_JxW:=i+JxW;
                pixels[I_JxW] := pixels[I_JxW] xor (pt(bitmap)[k] and ColorMask);
                inc(k);
              end;
            inc(k,deltaX1);
          End;
      End;
    ORPut:
      Begin
        for j:=Y to Y1 do
          Begin
            JxW:=j*PTCWidth;
            inc(k,deltaX);
            for i:=X to X1 do
              begin
                I_JxW:=i+JxW;
                pixels[I_JxW] := pixels[I_JxW] or (pt(bitmap)[k] and ColorMask);
                inc(k);
              end;
            inc(k,deltaX1);
          End;
      End;
    AndPut:
      Begin
        for j:=Y to Y1 do
          Begin
            JxW:=j*PTCWidth;
            inc(k,deltaX);
            for i:=X to X1 do
              begin
                I_JxW:=i+JxW;
                pixels[I_JxW] := pixels[I_JxW] and (pt(bitmap)[k] and ColorMask);
                inc(k);
              end;
            inc(k,deltaX1);
          End;
      End;
    NotPut:
      Begin
        for j:=Y to Y1 do
          Begin
            JxW:=j*PTCWidth;
            inc(k,deltaX);
            for i:=X to X1 do
              begin
                pixels[i+JxW] := (pt(bitmap)[k] and ColorMask) xor ColorMask;
                inc(k);
              end;
            inc(k,deltaX1);
          End;
      End;
    Else
      Begin
        for j:=Y to Y1 do
          Begin
            JxW:=j*PTCWidth;
            inc(k,deltaX);
            for i:=X to X1 do
              begin
                pixels[i+JxW] := pt(bitmap)[k] and ColorMask;
                inc(k);
              end;
            inc(k,deltaX1);
          End;
      End;
  End; {case}
  ptc_surface_unlock;
  ptc_update;
end;
Procedure ptc_PutImageproc_16bpp(X,Y: smallint; var Bitmap; BitBlt: Word);
type
  pt = array[0..{$ifdef cpu16}16382{$else}$fffffff{$endif}] of word;
  ptw = array[0..2] of longint;
var
  pixels:Pword;
  k: longint;
  i, j, y1, x1, deltaX, deltaX1, deltaY: smallint;
  JxW, I_JxW: Longword;
Begin
  inc(x,startXViewPort);
  inc(y,startYViewPort);
  { width/height are 1-based, coordinates are zero based }
  x1 := ptw(Bitmap)[0]+x-1; { get width and adjust end coordinate accordingly }
  y1 := ptw(Bitmap)[1]+y-1; { get height and adjust end coordinate accordingly }
  deltaY := 0;
  deltaX := 0;
  deltaX1 := 0;
  k := 3 * sizeOf(Longint) div sizeOf(Word); { Three reserved longs at start of bitmap }
 { check which part of the image is in the viewport }
  if clipPixels then
    begin
      if y < startYViewPort then
        begin
          deltaY := startYViewPort - y;
          inc(k,(x1-x+1)*deltaY);
          y := startYViewPort;
         end;
      if y1 > startYViewPort+viewHeight then
        y1 := startYViewPort+viewHeight;
      if x < startXViewPort then
        begin
          deltaX := startXViewPort-x;
          x := startXViewPort;
        end;
      if x1 > startXViewPort + viewWidth then
        begin
          deltaX1 := x1 - (startXViewPort + viewWidth);
          x1 := startXViewPort + viewWidth;
        end;
    end;
  pixels := ptc_surface_lock;
  case BitBlt of
    XORPut:
      Begin
        for j:=Y to Y1 do
          Begin
            JxW:=j*PTCWidth;
            inc(k,deltaX);
            for i:=X to X1 do
              begin
                I_JxW:=i+JxW;
                pixels[I_JxW] := pixels[I_JxW] xor pt(bitmap)[k];
                inc(k);
              end;
            inc(k,deltaX1);
          End;
      End;
    ORPut:
      Begin
        for j:=Y to Y1 do
          Begin
            JxW:=j*PTCWidth;
            inc(k,deltaX);
            for i:=X to X1 do
              begin
                I_JxW:=i+JxW;
                pixels[I_JxW] := pixels[I_JxW] or pt(bitmap)[k];
                inc(k);
              end;
            inc(k,deltaX1);
          End;
      End;
    AndPut:
      Begin
        for j:=Y to Y1 do
          Begin
            JxW:=j*PTCWidth;
            inc(k,deltaX);
            for i:=X to X1 do
              begin
                I_JxW:=i+JxW;
                pixels[I_JxW] := pixels[I_JxW] and pt(bitmap)[k];
                inc(k);
              end;
            inc(k,deltaX1);
          End;
      End;
    NotPut:
      Begin
        for j:=Y to Y1 do
          Begin
            JxW:=j*PTCWidth;
            inc(k,deltaX);
            for i:=X to X1 do
              begin
                pixels[i+JxW] := pt(bitmap)[k] xor $FFFF;
                inc(k);
              end;
            inc(k,deltaX1);
          End;
      End;
    Else
      Begin
        for j:=Y to Y1 do
          Begin
            JxW:=j*PTCWidth;
            inc(k,deltaX);
            for i:=X to X1 do
              begin
                pixels[i+JxW] := pt(bitmap)[k];
                inc(k);
              end;
            inc(k,deltaX1);
          End;
      End;
  End; {case}
  ptc_surface_unlock;
  ptc_update;
end;
{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
Procedure ptc_PutImageproc_32bpp(X,Y: smallint; var Bitmap; BitBlt: Word);
type
  pt = array[0..{$ifdef cpu16}16382{$else}$fffffff{$endif}] of longword;
  ptw = array[0..2] of longint;
var
  pixels:Plongword;
  k: longint;
  i, j, y1, x1, deltaX, deltaX1, deltaY: smallint;
  JxW, I_JxW: Longword;
Begin
  inc(x,startXViewPort);
  inc(y,startYViewPort);
  { width/height are 1-based, coordinates are zero based }
  x1 := ptw(Bitmap)[0]+x-1; { get width and adjust end coordinate accordingly }
  y1 := ptw(Bitmap)[1]+y-1; { get height and adjust end coordinate accordingly }
  deltaY := 0;
  deltaX := 0;
  deltaX1 := 0;
  k := 3 * sizeOf(Longint) div sizeOf(LongWord); { Three reserved longs at start of bitmap }
 { check which part of the image is in the viewport }
  if clipPixels then
    begin
      if y < startYViewPort then
        begin
          deltaY := startYViewPort - y;
          inc(k,(x1-x+1)*deltaY);
          y := startYViewPort;
         end;
      if y1 > startYViewPort+viewHeight then
        y1 := startYViewPort+viewHeight;
      if x < startXViewPort then
        begin
          deltaX := startXViewPort-x;
          x := startXViewPort;
        end;
      if x1 > startXViewPort + viewWidth then
        begin
          deltaX1 := x1 - (startXViewPort + viewWidth);
          x1 := startXViewPort + viewWidth;
        end;
    end;
  pixels := ptc_surface_lock;
  case BitBlt of
    XORPut:
      Begin
        for j:=Y to Y1 do
          Begin
            JxW:=j*PTCWidth;
            inc(k,deltaX);
            for i:=X to X1 do
              begin
                I_JxW:=i+JxW;
                pixels[I_JxW] := pixels[I_JxW] xor pt(bitmap)[k];
                inc(k);
              end;
            inc(k,deltaX1);
          End;
      End;
    ORPut:
      Begin
        for j:=Y to Y1 do
          Begin
            JxW:=j*PTCWidth;
            inc(k,deltaX);
            for i:=X to X1 do
              begin
                I_JxW:=i+JxW;
                pixels[I_JxW] := pixels[I_JxW] or pt(bitmap)[k];
                inc(k);
              end;
            inc(k,deltaX1);
          End;
      End;
    AndPut:
      Begin
        for j:=Y to Y1 do
          Begin
            JxW:=j*PTCWidth;
            inc(k,deltaX);
            for i:=X to X1 do
              begin
                I_JxW:=i+JxW;
                pixels[I_JxW] := pixels[I_JxW] and pt(bitmap)[k];
                inc(k);
              end;
            inc(k,deltaX1);
          End;
      End;
    NotPut:
      Begin
        for j:=Y to Y1 do
          Begin
            JxW:=j*PTCWidth;
            inc(k,deltaX);
            for i:=X to X1 do
              begin
                pixels[i+JxW] := pt(bitmap)[k] xor $FFFFFF;
                inc(k);
              end;
            inc(k,deltaX1);
          End;
      End;
    Else
      Begin
        for j:=Y to Y1 do
          Begin
            JxW:=j*PTCWidth;
            inc(k,deltaX);
            for i:=X to X1 do
              begin
                pixels[i+JxW] := pt(bitmap)[k];
                inc(k);
              end;
            inc(k,deltaX1);
          End;
      End;
  End; {case}
  ptc_surface_unlock;
  ptc_update;
end;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

{**********************************************************}
{ Procedure GetScanLine()                                  }
{----------------------------------------------------------}
{ Returns the full scanline of the video line of the Y     }
{ coordinate. The values are returned in a WORD array      }
{ each WORD representing a pixel of the specified scanline }
{ note: we only need the pixels inside the ViewPort! (JM)  }
{ note2: extended so you can specify start and end X coord }
{   so it is usable for GetImage too (JM)                  }
{ note3: This optomized version does not use GetPixel,     }
{   Whcih would be checking the viewport for every pixel.  }
{   Instead it just does it's own viewport check once then }
{   gets all the pixels on the scan line without further   }
{   checking  (JMR)                                        }
{**********************************************************}

Procedure PTC_GetScanlineProc_8bpp (X1, X2, Y : smallint; Var Data);
Var
  pixels        : Pbyte;
  x,vpx1,vpx2   : smallint;
Begin
   vpx1:=X1+StartXViewPort;
   vpx2:=X2+StartXViewPort;
   Y:=Y+StartYViewPort;
    { constrain to the part of the scanline that is in the viewport }
    if clipPixels then
       begin
          if vpx1 <  startXViewPort then
             vpx1 := startXViewPort;
          if vpx2 >  startXViewPort + viewWidth then
             vpx2 := startXViewPort + viewWidth;
       end;
    { constrain to the part of the scanline that is on the screen }
    if vpx1 <  0 then
       vpx1 := 0;
    if vpx2 >= PTCwidth then
       vpx2 := PTCwidth-1;
    If (ClipPixels AND (y <= startYViewPort+viewHeight) and (y >= startYViewPort) and (y>=0) and (y<PTCheight)) or Not(ClipPixels) then
       Begin
          pixels := ptc_surface_lock;
          For x:=vpx1 to vpx2 Do
             WordArray(Data)[x-StartXViewPort-x1]:=pixels[x+y*PTCWidth] and ColorMask;
          ptc_surface_unlock;
       End;
End;

Procedure PTC_GetScanlineProc_16bpp (X1, X2, Y : smallint; Var Data);
Var
  pixels        : Pword;
  x,vpx1,vpx2   : smallint;
Begin
   vpx1:=X1+StartXViewPort;
   vpx2:=X2+StartXViewPort;
   Y:=Y+StartYViewPort;
    { constrain to the part of the scanline that is in the viewport }
    if clipPixels then
       begin
          if vpx1 <  startXViewPort then
             vpx1 := startXViewPort;
          if vpx2 >  startXViewPort + viewWidth then
             vpx2 := startXViewPort + viewWidth;
       end;
    { constrain to the part of the scanline that is on the screen }
    if vpx1 <  0 then
       vpx1 := 0;
    if vpx2 >= PTCwidth then
       vpx2 := PTCwidth-1;
    If (ClipPixels AND (y <= startYViewPort+viewHeight) and (y >= startYViewPort) and (y>=0) and (y<PTCheight)) or Not(ClipPixels) then
       Begin
          pixels := ptc_surface_lock;
          For x:=vpx1 to vpx2 Do
             WordArray(Data)[x-StartXViewPort-x1]:=pixels[x+y*PTCWidth];
          ptc_surface_unlock;
       End;
End;

{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
Procedure PTC_GetScanlineProc_32bpp (X1, X2, Y : smallint; Var Data);
Var
  pixels        : Plongword;
  x,vpx1,vpx2   : smallint;
Begin
   vpx1:=X1+StartXViewPort;
   vpx2:=X2+StartXViewPort;
   Y:=Y+StartYViewPort;
    { constrain to the part of the scanline that is in the viewport }
    if clipPixels then
       begin
          if vpx1 <  startXViewPort then
             vpx1 := startXViewPort;
          if vpx2 >  startXViewPort + viewWidth then
             vpx2 := startXViewPort + viewWidth;
       end;
    { constrain to the part of the scanline that is on the screen }
    if vpx1 <  0 then
       vpx1 := 0;
    if vpx2 >= PTCwidth then
       vpx2 := PTCwidth-1;
    If (ClipPixels AND (y <= startYViewPort+viewHeight) and (y >= startYViewPort) and (y>=0) and (y<PTCheight)) or Not(ClipPixels) then
       Begin
          pixels := ptc_surface_lock;
          For x:=vpx1 to vpx2 Do
             LongWordArray(Data)[x-StartXViewPort-x1]:=pixels[x+y*PTCWidth];
          ptc_surface_unlock;
       End;
End;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

{**********************************************************}
{ Procedure GetImage()                                     }
{----------------------------------------------------------}
{ Returns a bitmap full the video specified by a rectagle  }
{ defined by X1,Y1 to X2,Y2                                }
{ the first 2 bytes of the bitmap structure define the     }
{ width and height of the rectangle                        }
{ These are later used by PutImage() so the bitmap is      }
{ properly represented                                     }
{ there is a 3rd reserved byte before data starts          }
{ note: This optomized version does not use GetScanLine or }
{   GetPixel, Whcih would be checking the viewport for     }
{   every pixel. Instead it just does it's own viewport    }
{   check once then gets all the pixels within the veiwport}
{   without further checking  (JMR)                        }
{**********************************************************}

Procedure PTC_GetImageProc_8bpp(X1,Y1,X2,Y2: smallint; Var Bitmap);
type
  pt = array[0..{$ifdef cpu16}16382{$else}$fffffff{$endif}] of word;
  ptw = array[0..2] of longint;
var
  pixels                       : Pbyte;
  x,y,i,j,vpx1,vpx2,vpy1,vpy2  : smallint;
  k      : longint;
Begin
  ptw(Bitmap)[0] := X2-X1+1;   { First longint  is width  }
  ptw(Bitmap)[1] := Y2-Y1+1;   { Second longint is height }
  ptw(bitmap)[2] := 0;         { Third longint is reserved}
  k:= 3 * Sizeof(longint) div sizeof(word); { Three reserved longs at start of bitmap }
  vpx1:=x1+StartXViewPort;
  vpx2:=x2+StartXViewPort;
  vpy1:=y1+StartYViewPort;
  vpy2:=y2+StartYViewPort;
  { check which part of the image is in the viewport }
  if clipPixels then
    begin
      if vpx1 < startXViewPort then
        vpx1 := startXViewPort;
      if vpx2 > startXViewPort + viewWidth then
        vpx2 := startXViewPort + viewWidth;
      if vpy1 < startYViewPort then
        vpy1 := startYViewPort;
      if vpy2 > startYViewPort+viewHeight then
        vpy2 := startYViewPort+viewHeight;
    end;
  { check if coordinates are on the screen}
  if vpx1 < 0 then
    vpx1 := 0;
  if vpx2 >= PTCwidth then
    vpx2 := PTCwidth-1;
  if vpy1 < 0 then
    vpy1 := 0;
  if vpy2 >= PTCheight then
    vpy2 := PTCheight-1;
  i := (x2 - x1 + 1);
  j := i * (vpy1 - StartYViewPort - y1);
  inc(k,j);
  pixels := ptc_surface_lock;
  for y:=vpy1 to vpy2 do
   Begin
     For x:=vpx1 to vpx2 Do
       pt(Bitmap)[k+(x-StartXViewPort-x1)]:=pixels[x+y*PTCWidth] and ColorMask;
     inc(k,i);
   end;
   ptc_surface_unlock;
end;

Procedure PTC_GetImageProc_16bpp(X1,Y1,X2,Y2: smallint; Var Bitmap);
type
  pt = array[0..{$ifdef cpu16}16382{$else}$fffffff{$endif}] of word;
  ptw = array[0..2] of longint;
var
  pixels : Pword;
  x,y,i,j,vpx1,vpx2,vpy1,vpy2  : smallint;
  k      : longint;
Begin
  ptw(Bitmap)[0] := X2-X1+1;   { First longint  is width  }
  ptw(Bitmap)[1] := Y2-Y1+1;   { Second longint is height }
  ptw(bitmap)[2] := 0;         { Third longint is reserved}
  k:= 3 * Sizeof(longint) div sizeof(word); { Three reserved longs at start of bitmap }
  vpx1:=x1+StartXViewPort;
  vpx2:=x2+StartXViewPort;
  vpy1:=y1+StartYViewPort;
  vpy2:=y2+StartYViewPort;
  { check which part of the image is in the viewport }
  if clipPixels then
    begin
      if vpx1 < startXViewPort then
        vpx1 := startXViewPort;
      if vpx2 > startXViewPort + viewWidth then
        vpx2 := startXViewPort + viewWidth;
      if vpy1 < startYViewPort then
        vpy1 := startYViewPort;
      if vpy2 > startYViewPort+viewHeight then
        vpy2 := startYViewPort+viewHeight;
    end;
  { check if coordinates are on the screen}
  if vpx1 < 0 then
    vpx1 := 0;
  if vpx2 >= PTCwidth then
    vpx2 := PTCwidth-1;
  if vpy1 < 0 then
    vpy1 := 0;
  if vpy2 >= PTCheight then
    vpy2 := PTCheight-1;
  i := (x2 - x1 + 1);
  j := i * (vpy1 - StartYViewPort - y1);
  inc(k,j);
  pixels := ptc_surface_lock;
  for y:=vpy1 to vpy2 do
   Begin
     For x:=vpx1 to vpx2 Do
       pt(Bitmap)[k+(x-StartXViewPort-x1)]:=pixels[x+y*PTCWidth];
      inc(k,i);
   end;
   ptc_surface_unlock;
end;

{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
Procedure PTC_GetImageProc_32bpp(X1,Y1,X2,Y2: smallint; Var Bitmap);
type
  pt = array[0..{$ifdef cpu16}16382{$else}$fffffff{$endif}] of longword;
  ptw = array[0..2] of longint;
var
  pixels : Plongword;
  x,y,i,j,vpx1,vpx2,vpy1,vpy2  : smallint;
  k      : longint;
Begin
  ptw(Bitmap)[0] := X2-X1+1;   { First longint  is width  }
  ptw(Bitmap)[1] := Y2-Y1+1;   { Second longint is height }
  ptw(bitmap)[2] := 0;         { Third longint is reserved}
  k:= 3 * Sizeof(longint) div sizeof(longword); { Three reserved longs at start of bitmap }
  vpx1:=x1+StartXViewPort;
  vpx2:=x2+StartXViewPort;
  vpy1:=y1+StartYViewPort;
  vpy2:=y2+StartYViewPort;
  { check which part of the image is in the viewport }
  if clipPixels then
    begin
      if vpx1 < startXViewPort then
        vpx1 := startXViewPort;
      if vpx2 > startXViewPort + viewWidth then
        vpx2 := startXViewPort + viewWidth;
      if vpy1 < startYViewPort then
        vpy1 := startYViewPort;
      if vpy2 > startYViewPort+viewHeight then
        vpy2 := startYViewPort+viewHeight;
    end;
  { check if coordinates are on the screen}
  if vpx1 < 0 then
    vpx1 := 0;
  if vpx2 >= PTCwidth then
    vpx2 := PTCwidth-1;
  if vpy1 < 0 then
    vpy1 := 0;
  if vpy2 >= PTCheight then
    vpy2 := PTCheight-1;
  i := (x2 - x1 + 1);
  j := i * (vpy1 - StartYViewPort - y1);
  inc(k,j);
  pixels := ptc_surface_lock;
  for y:=vpy1 to vpy2 do
   Begin
     For x:=vpx1 to vpx2 Do
       pt(Bitmap)[k+(x-StartXViewPort-x1)]:=pixels[x+y*PTCWidth];
      inc(k,i);
   end;
   ptc_surface_unlock;
end;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

{************************************************************************}
{*                       General routines                               *}
{************************************************************************}

 procedure CloseGraph;
 Begin
    If not isgraphmode then
      begin
        _graphresult := grnoinitgraph;
        exit
      end;
    RestoreVideoState;
    isgraphmode := false;
 end;

  procedure FillCommonVESA16(var mode: TModeInfo);
  begin
    mode.HardwarePages := 1;
    mode.MaxColor := 16;
    mode.PaletteSize := mode.MaxColor;
    mode.DirectColor := FALSE;
    mode.DirectPutPixel  := @ptc_DirectPixelProc_8bpp;
    mode.PutPixel        := @ptc_PutPixelProc_8bpp;
    mode.GetPixel        := @ptc_GetPixelProc_8bpp;
    mode.PutImage        := @ptc_PutImageProc_8bpp;
    mode.GetImage        := @ptc_GetImageProc_8bpp;
    mode.GetScanLine     := @ptc_GetScanLineProc_8bpp;
    mode.SetRGBPalette   := @ptc_SetRGBPaletteProc;
    mode.GetRGBPalette   := @ptc_GetRGBPaletteProc;
    mode.HLine           := @ptc_HLineProc_8bpp;
    mode.VLine           := @ptc_VLineProc_8bpp;
    mode.PatternLine     := @ptc_PatternLineProc_8bpp;
    mode.SetVisualPage   := @ptc_SetVisualPage;
    mode.SetActivePage   := @ptc_SetActivePage;
  end;

  procedure FillCommonVESA256(var mode: TModeInfo);
  begin
    mode.HardwarePages := 1;
    mode.MaxColor := 256;
    mode.PaletteSize := mode.MaxColor;
    mode.DirectColor := FALSE;
    mode.DirectPutPixel  := @ptc_DirectPixelProc_8bpp;
    mode.PutPixel        := @ptc_PutPixelProc_8bpp;
    mode.GetPixel        := @ptc_GetPixelProc_8bpp;
    mode.PutImage        := @ptc_PutImageProc_8bpp;
    mode.GetImage        := @ptc_GetImageProc_8bpp;
    mode.GetScanLine     := @ptc_GetScanLineProc_8bpp;
    mode.SetRGBPalette   := @ptc_SetRGBPaletteProc;
    mode.GetRGBPalette   := @ptc_GetRGBPaletteProc;
    //mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
    mode.HLine           := @ptc_HLineProc_8bpp;
    mode.VLine           := @ptc_VLineProc_8bpp;
    mode.PatternLine     := @ptc_PatternLineProc_8bpp;
    mode.SetVisualPage   := @ptc_SetVisualPage;
    mode.SetActivePage   := @ptc_SetActivePage;
  end;

  procedure FillCommonVESA32kOr64k(var mode: TModeInfo);
  begin
    mode.HardwarePages := 1;
    mode.DirectColor := TRUE;
    mode.DirectPutPixel  := @ptc_DirectPixelProc_16bpp;
    mode.PutPixel        := @ptc_PutPixelProc_16bpp;
    mode.GetPixel        := @ptc_GetPixelProc_16bpp;
    mode.PutImage        := @ptc_PutImageProc_16bpp;
    mode.GetImage        := @ptc_GetImageProc_16bpp;
    mode.GetScanLine     := @ptc_GetScanLineProc_16bpp;
    mode.SetRGBPalette   := @ptc_SetRGBPaletteProc;
    mode.GetRGBPalette   := @ptc_GetRGBPaletteProc;
    //mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
    mode.HLine           := @ptc_HLineProc_16bpp;
    mode.VLine           := @ptc_VLineProc_16bpp;
    mode.PatternLine     := @ptc_PatternLineProc_16bpp;
    mode.SetVisualPage   := @ptc_SetVisualPage;
    mode.SetActivePage   := @ptc_SetActivePage;
  end;

  procedure FillCommonVESA32k(var mode: TModeInfo);
  begin
    FillCommonVESA32kOr64k(mode);
    mode.MaxColor := 32768;
    mode.PaletteSize := mode.MaxColor;
  end;
  procedure FillCommonVESA64k(var mode: TModeInfo);
  begin
    FillCommonVESA32kOr64k(mode);
    mode.MaxColor := 65536;
    mode.PaletteSize := mode.MaxColor;
  end;

{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
  procedure FillCommonVESA32bpp(var mode: TModeInfo);
  begin
    mode.HardwarePages := 1;
    mode.MaxColor := 16777216;
    mode.PaletteSize := mode.MaxColor;
    mode.DirectColor := TRUE;
    mode.DirectPutPixel  := @ptc_DirectPixelProc_32bpp;
    mode.PutPixel        := @ptc_PutPixelProc_32bpp;
    mode.GetPixel        := @ptc_GetPixelProc_32bpp;
    mode.PutImage        := @ptc_PutImageProc_32bpp;
    mode.GetImage        := @ptc_GetImageProc_32bpp;
    mode.GetScanLine     := @ptc_GetScanLineProc_32bpp;
    mode.SetRGBPalette   := @ptc_SetRGBPaletteProc;
    mode.GetRGBPalette   := @ptc_GetRGBPaletteProc;
    //mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
    mode.HLine           := @ptc_HLineProc_32bpp;
    mode.VLine           := @ptc_VLineProc_32bpp;
    mode.PatternLine     := @ptc_PatternLineProc_32bpp;
    mode.SetVisualPage   := @ptc_SetVisualPage;
    mode.SetActivePage   := @ptc_SetActivePage;
  end;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

  function QueryAdapterInfo:PModeInfo;
  { This routine returns the head pointer to the list }
  { of supported graphics modes.                      }
  { Returns nil if no graphics mode supported.        }
  { This list is READ ONLY!                           }
  var
    PTCModeList: TPTCModeList;

    function ModeListEmpty: Boolean;
    begin
      ModeListEmpty := Length(PTCModeList) = 0;
    end;

    function ContainsExactResolution(AWidth, AHeight: Integer): Boolean;
    var
      I: Integer;
    begin
      if ModeListEmpty then
      begin
        ContainsExactResolution := False;
        exit;
      end;

      for I := Low(PTCModeList) to High(PTCModeList) do
        with PTCModeList[I] do
          if (Width = AWidth) and
             (Height = AHeight) then
          begin
            ContainsExactResolution := True;
            exit;
          end;
      ContainsExactResolution := False;
    end;

    function ContainsAtLeast(AWidth, AHeight: Integer): Boolean;
    var
      I: Integer;
    begin
      if ModeListEmpty then
      begin
        ContainsAtLeast := False;
        exit;
      end;

      for I := Low(PTCModeList) to High(PTCModeList) do
        with PTCModeList[I] do
          if (Width >= AWidth) and
             (Height >= AHeight) then
          begin
            ContainsAtLeast := True;
            exit;
          end;
      ContainsAtLeast := False;
    end;

    function IsNonStandardResolution(AWidth, AHeight: Integer): Boolean;
    begin
      IsNonStandardResolution :=
            not ((AWidth =  320) and (AHeight =  200))
        and not ((AWidth =  640) and (AHeight =  200))
        and not ((AWidth =  640) and (AHeight =  350))
        and not ((AWidth =  640) and (AHeight =  400))
        and not ((AWidth =  640) and (AHeight =  480))
        and not ((AWidth =  720) and (AHeight =  348))
        and not ((AWidth =  800) and (AHeight =  600))
        and not ((AWidth = 1024) and (AHeight =  768))
        and not ((AWidth = 1280) and (AHeight = 1024));
    end;

    function CompareModes(AMode1, AMode2: IPTCMode): Boolean;
    begin
      if AMode1.Width <> AMode2.Width then
        CompareModes := AMode1.Width < AMode2.Width
      else if AMode1.Height <> AMode2.Height then
        CompareModes := AMode1.Height < AMode2.Height
      else if AMode1.Format.Bits <> AMode2.Format.Bits then
        CompareModes := AMode1.Format.Bits < AMode2.Format.Bits
      else
        CompareModes := PtrUInt(AMode1) < PtrUInt(AMode2);
    end;

    procedure SortModes(l,r: longint);
      var
         i,j: longint;
         x,y: IPTCMode;
      begin
         i:=l;
         j:=r;
         x:=PTCModeList[(l+r) div 2];
         repeat
           while CompareModes(PTCModeList[i], x) do
            inc(i);
           while CompareModes(x, PTCModeList[j]) do
            dec(j);
           if not(i>j) then
             begin
                y:=PTCModeList[i];
                PTCModeList[i]:=PTCModeList[j];
                PTCModeList[j]:=y;
                inc(i);
                j:=j-1;
             end;
         until i>j;
         if l<j then
           SortModes(l,j);
         if i<r then
           SortModes(i,r);
      end;

    procedure FillCommonCGA320(var mode: TModeInfo);
    begin
      mode.HardwarePages := 0;
      mode.MaxColor := 4;
      mode.PaletteSize := 16;
      mode.DirectColor := FALSE;
      mode.MaxX := 319;
      mode.MaxY := 199;
      mode.DirectPutPixel  := @ptc_DirectPixelProc_8bpp;
      mode.PutPixel        := @ptc_PutPixelProc_8bpp;
      mode.GetPixel        := @ptc_GetPixelProc_8bpp;
      mode.PutImage        := @ptc_PutImageProc_8bpp;
      mode.GetImage        := @ptc_GetImageProc_8bpp;
      mode.GetScanLine     := @ptc_GetScanLineProc_8bpp;
      mode.SetRGBPalette   := @ptc_SetRGBPaletteProc;
      mode.GetRGBPalette   := @ptc_GetRGBPaletteProc;
      //mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
      mode.HLine           := @ptc_HLineProc_8bpp;
      mode.VLine           := @ptc_VLineProc_8bpp;
      mode.PatternLine     := @ptc_PatternLineProc_8bpp;
      mode.SetBkColor      := @SetBkColorCGA320;
      mode.GetBkColor      := @GetBkColorCGA320;
      mode.SetVisualPage   := @ptc_SetVisualPage;
      mode.SetActivePage   := @ptc_SetActivePage;
      mode.XAspect := 8333;
      mode.YAspect := 10000;
    end;

    procedure FillCommonCGA640(var mode: TModeInfo);
    begin
      mode.HardwarePages := 0;
      mode.MaxColor := 2;
      mode.PaletteSize := 16;
      mode.DirectColor := FALSE;
      mode.MaxX := 639;
      mode.MaxY := 199;
      mode.DirectPutPixel  := @ptc_DirectPixelProc_8bpp;
      mode.PutPixel        := @ptc_PutPixelProc_8bpp;
      mode.GetPixel        := @ptc_GetPixelProc_8bpp;
      mode.PutImage        := @ptc_PutImageProc_8bpp;
      mode.GetImage        := @ptc_GetImageProc_8bpp;
      mode.GetScanLine     := @ptc_GetScanLineProc_8bpp;
      mode.SetRGBPalette   := @ptc_SetRGBPaletteProc;
      mode.GetRGBPalette   := @ptc_GetRGBPaletteProc;
      //mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
      mode.HLine           := @ptc_HLineProc_8bpp;
      mode.VLine           := @ptc_VLineProc_8bpp;
      mode.PatternLine     := @ptc_PatternLineProc_8bpp;
      mode.SetBkColor      := @SetBkColorCGA640;
      mode.GetBkColor      := @GetBkColorCGA640;
      mode.SetVisualPage   := @ptc_SetVisualPage;
      mode.SetActivePage   := @ptc_SetActivePage;
      mode.XAspect := 4167;
      mode.YAspect := 10000;
    end;

    procedure FillCommonEGAVGA16(var mode: TModeInfo);
    begin
      mode.MaxColor := 16;
      mode.PaletteSize := mode.MaxColor;
      mode.DirectColor := FALSE;
      mode.DirectPutPixel := @ptc_DirectPixelProc_8bpp;
      mode.PutPixel       := @ptc_PutPixelProc_8bpp;
      mode.GetPixel       := @ptc_GetPixelProc_8bpp;
      mode.PutImage       := @ptc_PutImageProc_8bpp;
      mode.GetImage       := @ptc_GetImageProc_8bpp;
      mode.GetScanLine    := @ptc_GetScanLineProc_8bpp;
      mode.SetRGBPalette  := @ptc_SetRGBPaletteProc;
      mode.GetRGBPalette  := @ptc_GetRGBPaletteProc;
      //mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
      mode.HLine          := @ptc_HLineProc_8bpp;
      mode.VLine          := @ptc_VLineProc_8bpp;
      mode.PatternLine     := @ptc_PatternLineProc_8bpp;
      mode.SetVisualPage  := @ptc_SetVisualPage;
      mode.SetActivePage  := @ptc_SetActivePage;
    end;

    procedure FillCommonVESA320x200(var mode: TModeInfo);
    begin
      mode.DriverNumber := VESA;
      mode.ModeName:='320 x 200 VESA';
      mode.MaxX := 319;
      mode.MaxY := 199;
      mode.XAspect := 8333;
      mode.YAspect := 10000;
    end;
    procedure FillCommonVESA640x480(var mode: TModeInfo);
    begin
      mode.DriverNumber := VESA;
      mode.ModeName:='640 x 480 VESA';
      mode.MaxX := 639;
      mode.MaxY := 479;
      mode.XAspect := 10000;
      mode.YAspect := 10000;
    end;
    procedure FillCommonVESA800x600(var mode: TModeInfo);
    begin
      mode.DriverNumber := VESA;
      mode.ModeName:='800 x 600 VESA';
      mode.MaxX := 799;
      mode.MaxY := 599;
      mode.XAspect := 10000;
      mode.YAspect := 10000;
    end;
    procedure FillCommonVESA1024x768(var mode: TModeInfo);
    begin
      mode.DriverNumber := VESA;
      mode.ModeName:='1024 x 768 VESA';
      mode.MaxX := 1023;
      mode.MaxY := 767;
      mode.XAspect := 10000;
      mode.YAspect := 10000;
    end;
    procedure FillCommonVESA1280x1024(var mode: TModeInfo);
    begin
      mode.DriverNumber := VESA;
      mode.ModeName:='1280 x 1024 VESA';
      mode.MaxX := 1279;
      mode.MaxY := 1023;
      mode.XAspect := 10000;
      mode.YAspect := 10000;
    end;

   var
    graphmode:Tmodeinfo;
    I: Integer;
   begin
     QueryAdapterInfo := ModeList;
     { If the mode listing already exists... }
     { simply return it, without changing    }
     { anything...                           }
     if assigned(ModeList) then
       exit;

     PTCModeList := Copy(PTCWrapperObject.Modes);
     SortModes(Low(PTCModeList), High(PTCModeList));

     Has320x200 := ContainsExactResolution(320, 200);
     Has320x240 := ContainsExactResolution(320, 240);

     SaveVideoState:=@ptc_savevideostate;
     RestoreVideoState:=@ptc_restorevideostate;

     InitMode(graphmode);
     FillCommonCGA320(graphmode);
     graphmode.DriverNumber := CGA;
     graphmode.ModeNumber := CGAC0;
     graphmode.ModeName := '320 x 200 CGA C0';
     graphmode.InitMode := @ptc_Init320x200x4cgaC0;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonCGA320(graphmode);
     graphmode.DriverNumber := CGA;
     graphmode.ModeNumber := CGAC1;
     graphmode.ModeName := '320 x 200 CGA C1';
     graphmode.InitMode := @ptc_Init320x200x4cgaC1;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonCGA320(graphmode);
     graphmode.DriverNumber := CGA;
     graphmode.ModeNumber := CGAC2;
     graphmode.ModeName := '320 x 200 CGA C2';
     graphmode.InitMode := @ptc_Init320x200x4cgaC2;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonCGA320(graphmode);
     graphmode.DriverNumber := CGA;
     graphmode.ModeNumber := CGAC3;
     graphmode.ModeName := '320 x 200 CGA C3';
     graphmode.InitMode := @ptc_Init320x200x4cgaC3;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonCGA640(graphmode);
     graphmode.DriverNumber := CGA;
     graphmode.ModeNumber := CGAHi;
     graphmode.ModeName:='640 x 200 CGA';
     graphmode.InitMode := @ptc_Init640x200x2;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonCGA320(graphmode);
     graphmode.DriverNumber := MCGA;
     graphmode.ModeNumber := MCGAC0;
     graphmode.ModeName := '320 x 200 CGA C0'; { yes, it says 'CGA' even for the MCGA driver; this is TP7 compatible }
     graphmode.InitMode := @ptc_Init320x200x4cgaC0;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonCGA320(graphmode);
     graphmode.DriverNumber := MCGA;
     graphmode.ModeNumber := MCGAC1;
     graphmode.ModeName := '320 x 200 CGA C1'; { yes, it says 'CGA' even for the MCGA driver; this is TP7 compatible }
     graphmode.InitMode := @ptc_Init320x200x4cgaC1;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonCGA320(graphmode);
     graphmode.DriverNumber := MCGA;
     graphmode.ModeNumber := MCGAC2;
     graphmode.ModeName := '320 x 200 CGA C2'; { yes, it says 'CGA' even for the MCGA driver; this is TP7 compatible }
     graphmode.InitMode := @ptc_Init320x200x4cgaC2;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonCGA320(graphmode);
     graphmode.DriverNumber := MCGA;
     graphmode.ModeNumber := MCGAC3;
     graphmode.ModeName := '320 x 200 CGA C3'; { yes, it says 'CGA' even for the MCGA driver; this is TP7 compatible }
     graphmode.InitMode := @ptc_Init320x200x4cgaC3;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonCGA640(graphmode);
     graphmode.DriverNumber := MCGA;
     graphmode.ModeNumber := MCGAMed;
     graphmode.ModeName:='640 x 200 CGA'; { yes, it says 'CGA' even for the MCGA driver; this is TP7 compatible }
     graphmode.InitMode := @ptc_Init640x200x2;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=MCGAHi;
       DriverNumber := MCGA;
       HardwarePages := 0;
       ModeName:='640 x 480 MCGA';
       MaxColor := 2;
       DirectColor := FALSE;
       PaletteSize := 16;
       MaxX := 639;
       MaxY := 479;
       InitMode       := @ptc_Init640x480x2;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       PutImage       := @ptc_PutImageProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       GetScanLine    := @ptc_GetScanLineProc_8bpp;
       GetImage       := @ptc_GetImageProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;
       PatternLine    := @ptc_PatternLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       SetBkColor     := @SetBkColorMCGA640;
       GetBkColor     := @GetBkColorMCGA640;

       XAspect := 10000;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     if ContainsAtLeast(720, 348) then
     begin
       InitMode(graphmode);
       with graphmode do
       begin
         ModeNumber:=HercMonoHi;
         DriverNumber := HercMono;
         HardwarePages := 1;
         ModeName:='720 x 348 HERCULES';
         MaxColor := 2;
         DirectColor := FALSE;
         PaletteSize := 16;
         MaxX := 719;
         MaxY := 347;
         InitMode       := @ptc_Init720x348x2;
         DirectPutPixel := @ptc_DirectPixelProc_8bpp;
         PutPixel       := @ptc_PutPixelProc_8bpp;
         PutImage       := @ptc_PutImageProc_8bpp;
         GetPixel       := @ptc_GetPixelProc_8bpp;
         GetScanLine    := @ptc_GetScanLineProc_8bpp;
         GetImage       := @ptc_GetImageProc_8bpp;
         SetRGBPalette  := @ptc_SetRGBPaletteProc;
         GetRGBPalette  := @ptc_GetRGBPaletteProc;
         HLine          := @ptc_HLineProc_8bpp;
         VLine          := @ptc_VLineProc_8bpp;
         PatternLine    := @ptc_PatternLineProc_8bpp;
         SetVisualPage  := @ptc_SetVisualPage;
         SetActivePage  := @ptc_SetActivePage;
         SetBkColor     := @SetBkColorHGC720;
         GetBkColor     := @GetBkColorHGC720;
         XAspect := 7500;
         YAspect := 10000;
       end;
       AddMode(graphmode);
     end;

     InitMode(graphmode);
     FillCommonEGAVGA16(graphmode);
     with graphmode do
     begin
       ModeNumber:=EGALo;
       DriverNumber := EGA;
       ModeName:='640 x 200 EGA';
       MaxX := 639;
       MaxY := 199;
       HardwarePages := 3;
       InitMode := @ptc_Init640x200x16;
       XAspect := 4500;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonEGAVGA16(graphmode);
     with graphmode do
     begin
       ModeNumber:=EGAHi;
       DriverNumber := EGA;
       ModeName:='640 x 350 EGA';
       MaxX := 639;
       MaxY := 349;
       HardwarePages := 1;
       InitMode := @ptc_Init640x350x16;
       XAspect := 7750;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonEGAVGA16(graphmode);
     with graphmode do
     begin
       ModeNumber:=VGALo;
       DriverNumber := VGA;
       ModeName:='640 x 200 EGA'; { yes, it says 'EGA' even for the VGA driver; this is TP7 compatible }
       MaxX := 639;
       MaxY := 199;
       HardwarePages := 3;
       InitMode := @ptc_Init640x200x16;
       XAspect := 4500;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonEGAVGA16(graphmode);
     with graphmode do
     begin
       ModeNumber:=VGAMed;
       DriverNumber := VGA;
       ModeName:='640 x 350 EGA'; { yes, it says 'EGA' even for the VGA driver; this is TP7 compatible }
       MaxX := 639;
       MaxY := 349;
       HardwarePages := 1;
       InitMode := @ptc_Init640x350x16;
       XAspect := 7750;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonEGAVGA16(graphmode);
     with graphmode do
     begin
       ModeNumber:=VGAHi;
       DriverNumber := VGA;
       ModeName:='640 x 480 VGA';
       MaxX := 639;
       MaxY := 479;
       HardwarePages := 0;
       InitMode := @ptc_Init640x480x16;
       XAspect := 10000;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=0;
       DriverNumber := LowRes;
       HardwarePages := 0;
       ModeName:='320 x 200 VGA';
       MaxColor := 256;
       DirectColor := FALSE;
       PaletteSize := MaxColor;
       MaxX := 319;
       MaxY := 199;
       InitMode       := @ptc_Init320x200x256;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       PutImage       := @ptc_PutImageProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       GetScanLine    := @ptc_GetScanLineProc_8bpp;
       GetImage       := @ptc_GetImageProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;
       //SetAllPalette  := @ptc_SetRGBAllPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;
       PatternLine    := @ptc_PatternLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       XAspect := 8333;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=1;
       DriverNumber := LowRes;
       HardwarePages := 3;
       ModeName:='320 x 200 ModeX';
       MaxColor := 256;
       DirectColor := FALSE;
       PaletteSize := MaxColor;
       MaxX := 319;
       MaxY := 199;
       InitMode       := @ptc_Init320x200x256;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       PutImage       := @ptc_PutImageProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       GetScanLine    := @ptc_GetScanLineProc_8bpp;
       GetImage       := @ptc_GetImageProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;
       //SetAllPalette  := @ptc_SetRGBAllPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;
       PatternLine    := @ptc_PatternLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       XAspect := 8333;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonVESA256(graphmode);
     with graphmode do
     begin
       ModeNumber:=m640x400x256;
       DriverNumber := VESA;
       ModeName:='640 x 400 VESA';
       MaxX := 639;
       MaxY := 399;
       InitMode := @ptc_Init640x400x256;
       XAspect := 8333;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonVESA256(graphmode);
     FillCommonVESA640x480(graphmode);
     with graphmode do
     begin
       ModeNumber:=m640x480x256;
       InitMode := @ptc_Init640x480x256;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonVESA32k(graphmode);
     FillCommonVESA320x200(graphmode);
     with graphmode do
     begin
       ModeNumber := m320x200x32k;
       InitMode := @ptc_Init320x200x32k;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonVESA32k(graphmode);
     FillCommonVESA640x480(graphmode);
     with graphmode do
     begin
       ModeNumber := m640x480x32k;
       InitMode := @ptc_Init640x480x32k;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonVESA64k(graphmode);
     FillCommonVESA320x200(graphmode);
     with graphmode do
     begin
       ModeNumber := m320x200x64k;
       InitMode := @ptc_Init320x200x64k;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonVESA64k(graphmode);
     FillCommonVESA640x480(graphmode);
     with graphmode do
     begin
       ModeNumber := m640x480x64k;
       InitMode := @ptc_Init640x480x64k;
     end;
     AddMode(graphmode);

{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
     InitMode(graphmode);
     FillCommonVESA32bpp(graphmode);
     FillCommonVESA320x200(graphmode);
     with graphmode do
     begin
       ModeNumber := m320x200x16m;
       InitMode := @ptc_Init320x200x32bpp;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     FillCommonVESA32bpp(graphmode);
     FillCommonVESA640x480(graphmode);
     with graphmode do
     begin
       ModeNumber := m640x480x16m;
       InitMode := @ptc_Init640x480x32bpp;
     end;
     AddMode(graphmode);
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}

     if ContainsAtLeast(800, 600) then
     begin
       InitMode(graphmode);
       FillCommonVESA16(graphmode);
       FillCommonVESA800x600(graphmode);
       with graphmode do
       begin
         ModeNumber := m800x600x16;
         InitMode := @ptc_Init800x600x16;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       FillCommonVESA256(graphmode);
       FillCommonVESA800x600(graphmode);
       with graphmode do
       begin
         ModeNumber:=m800x600x256;
         InitMode := @ptc_Init800x600x256;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       FillCommonVESA32k(graphmode);
       FillCommonVESA800x600(graphmode);
       with graphmode do
       begin
         ModeNumber := m800x600x32k;
         InitMode := @ptc_Init800x600x32k;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       FillCommonVESA64k(graphmode);
       FillCommonVESA800x600(graphmode);
       with graphmode do
       begin
         ModeNumber := m800x600x64k;
         InitMode := @ptc_Init800x600x64k;
       end;
       AddMode(graphmode);

{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
       InitMode(graphmode);
       FillCommonVESA32bpp(graphmode);
       FillCommonVESA800x600(graphmode);
       with graphmode do
       begin
         ModeNumber := m800x600x16m;
         InitMode := @ptc_Init800x600x32bpp;
       end;
       AddMode(graphmode);
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}
     end;

     if ContainsAtLeast(1024, 768) then
     begin
       InitMode(graphmode);
       FillCommonVESA16(graphmode);
       FillCommonVESA1024x768(graphmode);
       with graphmode do
       begin
         ModeNumber := m1024x768x16;
         InitMode := @ptc_Init1024x768x16;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       FillCommonVESA256(graphmode);
       FillCommonVESA1024x768(graphmode);
       with graphmode do
       begin
         ModeNumber:=m1024x768x256;
         InitMode := @ptc_Init1024x768x256;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       FillCommonVESA32k(graphmode);
       FillCommonVESA1024x768(graphmode);
       with graphmode do
       begin
         ModeNumber := m1024x768x32k;
         InitMode := @ptc_Init1024x768x32k;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       FillCommonVESA64k(graphmode);
       FillCommonVESA1024x768(graphmode);
       with graphmode do
       begin
         ModeNumber := m1024x768x64k;
         InitMode := @ptc_Init1024x768x64k;
       end;
       AddMode(graphmode);

{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
       InitMode(graphmode);
       FillCommonVESA32bpp(graphmode);
       FillCommonVESA1024x768(graphmode);
       with graphmode do
       begin
         ModeNumber := m1024x768x16m;
         InitMode := @ptc_Init1024x768x32bpp;
       end;
       AddMode(graphmode);
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}
     end;

     if ContainsAtLeast(1280, 1024) then
     begin
       InitMode(graphmode);
       FillCommonVESA16(graphmode);
       FillCommonVESA1280x1024(graphmode);
       with graphmode do
       begin
         ModeNumber := m1280x1024x16;
         InitMode := @ptc_Init1280x1024x16;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       FillCommonVESA256(graphmode);
       FillCommonVESA1280x1024(graphmode);
       with graphmode do
       begin
         ModeNumber:=m1280x1024x256;
         InitMode := @ptc_Init1280x1024x256;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       FillCommonVESA32k(graphmode);
       FillCommonVESA1280x1024(graphmode);
       with graphmode do
       begin
         ModeNumber := m1280x1024x32k;
         InitMode := @ptc_Init1280x1024x32k;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       FillCommonVESA64k(graphmode);
       FillCommonVESA1280x1024(graphmode);
       with graphmode do
       begin
         ModeNumber := m1280x1024x64k;
         InitMode := @ptc_Init1280x1024x64k;
       end;
       AddMode(graphmode);

{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
       InitMode(graphmode);
       FillCommonVESA32bpp(graphmode);
       FillCommonVESA1280x1024(graphmode);
       with graphmode do
       begin
         ModeNumber := m1280x1024x16m;
         InitMode := @ptc_Init1280x1024x32bpp;
       end;
       AddMode(graphmode);
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}
     end;

     { finally, add all the non-standard (i.e. not VESA or classic PC) modes }
     NextNonStandardModeNumber := FirstNonStandardModeNumber;
     for I := Low(PTCModeList) to High(PTCModeList) do
       with PTCModeList[I] do
         if IsNonStandardResolution(Width, Height) and
            ((I = Low(PTCModeList)) or ((Width <> PTCModeList[I-1].Width) or (Height <> PTCModeList[I-1].Height))) then
         begin
           InitMode(graphmode);
           FillCommonVESA16(graphmode);
           with graphmode do
           begin
             ModeNumber := NextNonStandardModeNumber;
             DriverNumber := VESA;
             WriteStr(ModeName, Width, ' x ', Height, ' VESA');
             MaxX := Width - 1;
             MaxY := Height - 1;
             InitMode := @ptc_InitNonStandard16;
             XAspect := 10000;
             YAspect := 10000;
           end;
           AddMode(graphmode);
           Inc(NextNonStandardModeNumber);
           if NextNonStandardModeNumber > NonStandardModeNumberMaxLimit then
             break;

           InitMode(graphmode);
           FillCommonVESA256(graphmode);
           with graphmode do
           begin
             ModeNumber := NextNonStandardModeNumber;
             DriverNumber := VESA;
             WriteStr(ModeName, Width, ' x ', Height, ' VESA');
             MaxX := Width - 1;
             MaxY := Height - 1;
             InitMode := @ptc_InitNonStandard256;
             XAspect := 10000;
             YAspect := 10000;
           end;
           AddMode(graphmode);
           Inc(NextNonStandardModeNumber);
           if NextNonStandardModeNumber > NonStandardModeNumberMaxLimit then
             break;

           InitMode(graphmode);
           FillCommonVESA32k(graphmode);
           with graphmode do
           begin
             ModeNumber := NextNonStandardModeNumber;
             DriverNumber := VESA;
             WriteStr(ModeName, Width, ' x ', Height, ' VESA');
             MaxX := Width - 1;
             MaxY := Height - 1;
             InitMode := @ptc_InitNonStandard32k;
             XAspect := 10000;
             YAspect := 10000;
           end;
           AddMode(graphmode);
           Inc(NextNonStandardModeNumber);
           if NextNonStandardModeNumber > NonStandardModeNumberMaxLimit then
             break;

           InitMode(graphmode);
           FillCommonVESA64k(graphmode);
           with graphmode do
           begin
             ModeNumber := NextNonStandardModeNumber;
             DriverNumber := VESA;
             WriteStr(ModeName, Width, ' x ', Height, ' VESA');
             MaxX := Width - 1;
             MaxY := Height - 1;
             InitMode := @ptc_InitNonStandard64k;
             XAspect := 10000;
             YAspect := 10000;
           end;
           AddMode(graphmode);
           Inc(NextNonStandardModeNumber);
           if NextNonStandardModeNumber > NonStandardModeNumberMaxLimit then
             break;

{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
           InitMode(graphmode);
           FillCommonVESA32bpp(graphmode);
           with graphmode do
           begin
             ModeNumber := NextNonStandardModeNumber;
             DriverNumber := VESA;
             WriteStr(ModeName, Width, ' x ', Height, ' VESA');
             MaxX := Width - 1;
             MaxY := Height - 1;
             InitMode := @ptc_InitNonStandard32bpp;
             XAspect := 10000;
             YAspect := 10000;
           end;
           AddMode(graphmode);
           Inc(NextNonStandardModeNumber);
           if NextNonStandardModeNumber > NonStandardModeNumberMaxLimit then
             break;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}
         end;
  end;

function InstallUserMode(Width, Height: SmallInt; Colors: LongInt; HardwarePages: SmallInt; XAspect, YAspect: Word): smallint;
var
  graphmode: Tmodeinfo;
begin
  if (NextNonStandardModeNumber > NonStandardModeNumberMaxLimit) or (HardwarePages < 1) or
     (Width <= 0) or (Height <= 0) or (XAspect <= 0) or (YAspect <= 0) then
  begin
    InstallUserMode := grError;
    exit;
  end;
  InitMode(graphmode);
  case Colors of
{    2:
      begin
      end;
    4:
      begin
      end;}
    16:
      begin
        FillCommonVESA16(graphmode);
        graphmode.InitMode := @ptc_InitNonStandard16;
      end;
    256:
      begin
        FillCommonVESA256(graphmode);
        graphmode.InitMode := @ptc_InitNonStandard256;
      end;
    32768:
      begin
        FillCommonVESA32k(graphmode);
        graphmode.InitMode := @ptc_InitNonStandard32k;
      end;
    65536:
      begin
        FillCommonVESA64k(graphmode);
        graphmode.InitMode := @ptc_InitNonStandard64k;
      end;
{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
    16777216:
      begin
        FillCommonVESA32bpp(graphmode);
        graphmode.InitMode := @ptc_InitNonStandard32bpp;
      end;
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}
    else
      begin
        InstallUserMode := grError;
        exit;
      end;
  end;
  with graphmode do
  begin
    ModeNumber := NextNonStandardModeNumber;
    DriverNumber := VESA;
    WriteStr(ModeName, Width, ' x ', Height, ' VESA');
    MaxX := Width - 1;
    MaxY := Height - 1;
    HardwarePages := 1;
  end;
  graphmode.XAspect := XAspect;
  graphmode.YAspect := YAspect;
  graphmode.HardwarePages := HardwarePages - 1;
  AddMode(graphmode);
  Inc(NextNonStandardModeNumber);
  InstallUserMode := graphmode.ModeNumber;
end;

initialization
  WindowTitle := ParamStr(0);
  PTCFormat8 := TPTCFormatFactory.CreateNew(8);
  PTCFormat15 := TPTCFormatFactory.CreateNew(16, $7C00, $03E0, $001F);
  PTCFormat16 := TPTCFormatFactory.CreateNew(16, $F800, $07E0, $001F);
{$ifdef FPC_GRAPH_SUPPORTS_TRUECOLOR}
  PTCFormat32 := TPTCFormatFactory.CreateNew(32, $00FF0000, $0000FF00, $000000FF);
{$endif FPC_GRAPH_SUPPORTS_TRUECOLOR}
  PTCWrapperObject := TPTCWrapperThread.Create;
  InitializeGraph;
finalization
  PTCWrapperObject.Terminate;
  PTCWrapperObject.WaitFor;
  PTCWrapperObject.Free;
end.
