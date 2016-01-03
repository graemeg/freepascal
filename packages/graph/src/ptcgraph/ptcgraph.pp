{
    This file is part of the Free Pascal run time library.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (c) 2010, 2011 by Nikolay Nikolov (nickysn@users.sourceforge.net)
=======
    Copyright (c) 2010 by Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (c) 2010 by Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (c) 2010 by Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (c) 2010 by Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> origin/cpstrnew
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{//$define logging}
=======
{$define logging}
>>>>>>> graemeg/cpstrnew
=======
{$define logging}
>>>>>>> graemeg/cpstrnew
=======
{$define logging}
>>>>>>> graemeg/cpstrnew
=======
{$define logging}
>>>>>>> origin/cpstrnew

{******************************************************************************}
                                    interface
{******************************************************************************}

uses
  ptc, ptcwrapper;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef VER2_6}
type
  CodePointer = Pointer;
{$endif}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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

  m640x400x256      = $100;

  m640x480x256      = $101;
  m640x480x32k      = $110;
  m640x480x64k      = $111;

  m800x600x16       = $102;
  m800x600x256      = $103;
  m800x600x32k      = $113;
  m800x600x64k      = $114;

  m1024x768x16      = $104;
  m1024x768x256     = $105;
  m1024x768x32k     = $116;
  m1024x768x64k     = $117;

  m1280x1024x16     = $106;
  m1280x1024x256    = $107;
  m1280x1024x32k    = $119;
  m1280x1024x64k    = $11A;

const
  FullscreenGraph: Boolean = False;

var
  PTCWrapperObject: TPTCWrapperThread;

{******************************************************************************}
                                 implementation
{******************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
//uses
//  termio{,x86};

>>>>>>> graemeg/cpstrnew
=======
//uses
//  termio{,x86};

>>>>>>> graemeg/cpstrnew
=======
//uses
//  termio{,x86};

>>>>>>> graemeg/cpstrnew
=======
//uses
//  termio{,x86};

>>>>>>> origin/cpstrnew
const
  InternalDriverName = 'PTCPas';

var
  Has320x200: Boolean;
  Has320x240: Boolean;
  charmessagehandler: Pointer;  { dummy, for compatibility with graph.inc, which initializes this to nil under win32 }
  hasVesa: Boolean = false;  { dummy, for compatibility with graph.inc, which checks it in its ExitProc under go32v2 }
  VesaInfo: record { dummy, for compatibility with graph.inc under go32v2 }
    ModeList: PInteger;
  end;

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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  PTCWidth: Integer;
  PTCHeight: Integer;
  PTCFormat8: IPTCFormat;
  PTCFormat15: IPTCFormat;
  PTCFormat16: IPTCFormat;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
//  OldIO: TermIos;

{  ptcconsole: TPTCConsole = nil;
  ptcsurface: TPTCSurface = nil;
  ptcpalette: TPTCPalette = nil;
  ptcformat: TPTCFormat = nil;}
  PTCWidth: Integer;
  PTCHeight: Integer;
  PTCFormat8: TPTCFormat;
  PTCFormat15: TPTCFormat;
  PTCFormat16: TPTCFormat;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  EGAPaletteEnabled: Boolean;
  EGAPalette: TEGAPalette;
  VGAPalette: TVGAPalette;

  CurrentActivePage: Integer;
  ColorMask: Word;

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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
end;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  { copy to console }
//  ptcsurface.copy(ptcconsole);
  { update console }
//  ptcconsole.update;
end;

{Procedure SetRawMode(b:boolean);
Var
  Tio : Termios;
Begin
  if b then
   begin
     TCGetAttr(1,Tio);
     OldIO:=Tio;
     CFMakeRaw(Tio);
   end
  else
   Tio:=OldIO;
  TCSetAttr(1,TCSANOW,Tio);
End;}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ ---------------------------------------------------------------------
    Required procedures
  ---------------------------------------------------------------------}

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
var
  LastColor: smallint;   {Cache the last set color to improve speed}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure ptc_savevideostate;
begin
end;

procedure ptc_restorevideostate;
begin
  if PTCWrapperObject <> nil then
    PTCWrapperObject.Close;
end;

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
{
const
  BgiColors: array[0..15] of LongInt
    = ($000000, $000020, $002000, $002020,
       $200000, $200020, $202000, $303030,
       $202020, $00003F, $003F00, $003F3F,
       $3F0000, $3F003F, $3F3F00, $3F3F3F);
}

procedure InitColors(nrColors: longint);

var
  i: smallint;
begin
{  for i:=0 to nrColors do
    vga_setpalette(I,DefaultColors[i].red shr 2,
      DefaultColors[i].green shr 2,DefaultColors[i].blue shr 2)}
end;

procedure ptc_initmodeproc;

begin
//  writeln('Initializing mode');
  { create format }
{  FreeAndNil(PTCFormat);
  PTCFormat:=TPTCFormat.Create(16,$f800,$07e0,$001f);}
  { open the console }
{  ptcconsole.open(paramstr(0),ptcformat);}
  { create surface matching console dimensions }
{  FreeAndNil(PTCSurface);
  PTCSurface:=TPTCSurface.Create(ptcconsole.width,ptcconsole.height,ptcformat);}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  r, g, b: Uint32;
>>>>>>> graemeg/cpstrnew
=======
  r, g, b: Uint32;
>>>>>>> graemeg/cpstrnew
=======
  r, g, b: Uint32;
>>>>>>> graemeg/cpstrnew
=======
  r, g, b: Uint32;
>>>>>>> origin/cpstrnew
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure ptc_InternalOpen(const ATitle: string; AWidth, AHeight: Integer; AFormat: IPTCFormat; AVirtualPages: Integer);
=======
procedure ptc_InternalOpen(const ATitle: string; AWidth, AHeight: Integer; AFormat: TPTCFormat; AVirtualPages: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure ptc_InternalOpen(const ATitle: string; AWidth, AHeight: Integer; AFormat: TPTCFormat; AVirtualPages: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure ptc_InternalOpen(const ATitle: string; AWidth, AHeight: Integer; AFormat: TPTCFormat; AVirtualPages: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure ptc_InternalOpen(const ATitle: string; AWidth, AHeight: Integer; AFormat: TPTCFormat; AVirtualPages: Integer);
>>>>>>> origin/cpstrnew
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
  ptc_InternalOpen(ParamStr(0), XResolution, YResolution, PTCFormat8, Pages);
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
  ptc_InternalOpen(ParamStr(0), XResolution, YResolution, PTCFormat8, Pages);
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
  ptc_InternalOpen(ParamStr(0), XResolution, YResolution, PTCFormat8, 1);
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
  ptc_InternalOpen(ParamStr(0), XResolution, YResolution, PTCFormat8, Pages);
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
  ptc_InternalOpen(ParamStr(0), XResolution, YResolution, PTCFormat8, Pages);
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
  ptc_InternalOpen(ParamStr(0), XResolution, YResolution, PTCFormat15, Pages);
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
  ptc_InternalOpen(ParamStr(0), XResolution, YResolution, PTCFormat16, Pages);
  PTCWidth := XResolution;
  PTCHeight := YResolution;
  CurrentActivePage := 0;
  ColorMask := 65535;
end;


procedure ptc_Init640x200x16;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  ptc_InitMode16_CGAEmu(640, 200, 4);
=======
  ptc_InitMode16_CGAEmu(640, 200, 3);
>>>>>>> graemeg/cpstrnew
=======
  ptc_InitMode16_CGAEmu(640, 200, 3);
>>>>>>> graemeg/cpstrnew
=======
  ptc_InitMode16_CGAEmu(640, 200, 3);
>>>>>>> graemeg/cpstrnew
=======
  ptc_InitMode16_CGAEmu(640, 200, 3);
>>>>>>> origin/cpstrnew
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
procedure SetBkColorHGC720(ColorNum: Word);
begin
  if ColorNum > 15 then
    exit;
  DummyHGCBkColor := ColorNum;
end;

{ compatible with TP7's HERC.BGI }
function GetBkColorHGC720: Word;
begin
  GetBkColorHGC720 := DummyHGCBkColor;
end;

procedure SetBkColorCGA320(ColorNum: Word);
begin
  if ColorNum > 15 then
    exit;
  CurrentCGABkColor := ColorNum;

  ptc_SetEGAPalette(0, ((ColorNum shl 1) and $10) or (ColorNum and $07));
end;

function GetBkColorCGA320: Word;
begin
  GetBkColorCGA320 := CurrentCGABkColor;
end;

{yes, TP7 CGA.BGI behaves *exactly* like that}
procedure SetBkColorCGA640(ColorNum: Word);
begin
  if ColorNum > 15 then
    exit;
  CurrentCGABkColor := ColorNum;
  if ColorNum = 0 then
    exit;
  ptc_SetEGAPalette(1, ((ColorNum shl 1) and $10) or (ColorNum and $07));
end;

function GetBkColorCGA640: Word;
begin
  GetBkColorCGA640 := CurrentCGABkColor;
end;

{ nickysn: VGA compatible implementation. I don't have a real MCGA to test
  if there's any difference with VGA }
procedure SetBkColorMCGA640(ColorNum: Word);
begin
  if ColorNum > 15 then
    exit;
  CurrentCGABkColor := ColorNum;

  ptc_SetEGAPalette(0, ((ColorNum shl 1) and $10) or (ColorNum and $07));
end;

function GetBkColorMCGA640: Word;
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

procedure ptc_DirectPixelProc_16bpp(X,Y: smallint);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var
  pixels:Pword;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

var color:word;
    pixels:Pword;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var
  pixels:PByte;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

var color:word;
    pixels:PByte;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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

procedure ptc_putpixelproc_16bpp(X,Y:smallint;Color:Word);

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

function ptc_getpixelproc_16bpp(X,Y: smallint):word;

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

procedure ptc_PutPixelProc_8bpp(X,Y:smallint;Color:Word);

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

function ptc_GetPixelProc_8bpp(X,Y: smallint):word;

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

{ Bitmap utilities }
{type
  PBitmap = ^TBitmap;
  TBitmap = record
            Width, Height: smallint;
            Data: record end;
            end;
}

procedure ptc_putimageproc (X,Y: smallint; var Bitmap; BitBlt: Word);
begin
end;

procedure ptc_getimageproc (X1,Y1,X2,Y2: smallint; Var Bitmap);
begin
end;

function  ptc_imagesizeproc (X1,Y1,X2,Y2: smallint): longint;
begin
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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



procedure ptc_vlineproc (x,y,y2: smallint);
begin
end;

procedure ptc_clrviewproc_16bpp;

Var I,Xmax : longint;

begin
  Xmax:=StartXViewPort+ViewWidth-1;
  For i:=StartYViewPort to StartYViewPort+ViewHeight-1 do
    ptc_HLineProc_16bpp(0,viewwidth,i);
  { reset coordinates }
  CurrentX := 0;
  CurrentY := 0;
end;

procedure ptc_patternlineproc (x1,x2,y: smallint);
begin
end;

procedure ptc_ellipseproc  (X,Y: smallint;XRadius: word;
  YRadius:word; stAngle,EndAngle: word; fp: PatternLineProc);
begin
end;

procedure ptc_lineproc (X1, Y1, X2, Y2 : smallint);
begin
end;

procedure ptc_getscanlineproc (X1,X2,Y : smallint; var data);
begin
end;

procedure ptc_setactivepageproc (page: word);
begin
end;

procedure ptc_setvisualpageproc (page: word);
begin
end;


procedure ptc_savestateproc;
begin
end;

procedure ptc_restorestateproc;
begin
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
//    SetRawMode(False);
>>>>>>> graemeg/cpstrnew
=======
//    SetRawMode(False);
>>>>>>> graemeg/cpstrnew
=======
//    SetRawMode(False);
>>>>>>> graemeg/cpstrnew
=======
//    SetRawMode(False);
>>>>>>> origin/cpstrnew
    RestoreVideoState;
    isgraphmode := false;
 end;

  function QueryAdapterInfo:PModeInfo;
  { This routine returns the head pointer to the list }
  { of supported graphics modes.                      }
  { Returns nil if no graphics mode supported.        }
  { This list is READ ONLY!                           }
  var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    PTCModeList: TPTCModeList;

    function ModeListEmpty: Boolean;
    begin
      ModeListEmpty := Length(PTCModeList) = 0;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    PTCModeList: PPTCMode;

    function ModeListEmpty: Boolean;
    begin
      ModeListEmpty := (PTCModeList = nil) or (not PTCModeList[0].Valid);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      for I := Low(PTCModeList) to High(PTCModeList) do
=======
      I := 0;
      while (PTCModeList[I].Valid) do
      begin
>>>>>>> graemeg/cpstrnew
=======
      I := 0;
      while (PTCModeList[I].Valid) do
      begin
>>>>>>> graemeg/cpstrnew
=======
      I := 0;
      while (PTCModeList[I].Valid) do
      begin
>>>>>>> graemeg/cpstrnew
=======
      I := 0;
      while (PTCModeList[I].Valid) do
      begin
>>>>>>> origin/cpstrnew
        with PTCModeList[I] do
          if (Width = AWidth) and
             (Height = AHeight) then
          begin
            ContainsExactResolution := True;
            exit;
          end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
        Inc(I);
      end;
>>>>>>> graemeg/cpstrnew
=======
        Inc(I);
      end;
>>>>>>> graemeg/cpstrnew
=======
        Inc(I);
      end;
>>>>>>> graemeg/cpstrnew
=======
        Inc(I);
      end;
>>>>>>> origin/cpstrnew
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      for I := Low(PTCModeList) to High(PTCModeList) do
=======
      I := 0;
      while (PTCModeList[I].Valid) do
      begin
>>>>>>> graemeg/cpstrnew
=======
      I := 0;
      while (PTCModeList[I].Valid) do
      begin
>>>>>>> graemeg/cpstrnew
=======
      I := 0;
      while (PTCModeList[I].Valid) do
      begin
>>>>>>> graemeg/cpstrnew
=======
      I := 0;
      while (PTCModeList[I].Valid) do
      begin
>>>>>>> origin/cpstrnew
        with PTCModeList[I] do
          if (Width >= AWidth) and
             (Height >= AHeight) then
          begin
            ContainsAtLeast := True;
            exit;
          end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
        Inc(I);
      end;
>>>>>>> graemeg/cpstrnew
=======
        Inc(I);
      end;
>>>>>>> graemeg/cpstrnew
=======
        Inc(I);
      end;
>>>>>>> graemeg/cpstrnew
=======
        Inc(I);
      end;
>>>>>>> origin/cpstrnew
      ContainsAtLeast := False;
    end;

   var
    graphmode:Tmodeinfo;
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
    d{,i} : longint;
    ws,hs:string[5];

   const depths:array[0..3] of byte=(8,16,24,32);
         colours:array[0..3] of longint=(256,65536,16777216,16777216);
         depth_names:array[0..3] of string[5]=('256','64K','16M','16M32');

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
   begin
     QueryAdapterInfo := ModeList;
     { If the mode listing already exists... }
     { simply return it, without changing    }
     { anything...                           }
     if assigned(ModeList) then
       exit;

     PTCModeList := PTCWrapperObject.Modes;

     Has320x200 := ContainsExactResolution(320, 200);
     Has320x240 := ContainsExactResolution(320, 240);

     SaveVideoState:=@ptc_savevideostate;
     RestoreVideoState:=@ptc_restorevideostate;
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
{     if PTCConsole = nil then
       PTCConsole := TPTCConsole.Create;}
//     ptcmode:=ptcconsole.modes;
//     i:=0;
{     initmode(graphmode);
     with graphmode do
       begin
         modenumber:=0;
         drivernumber:=ptcgraph._ptc;
         maxx:=639;
         maxy:=479;
         modename:='PTC_640x480x64K';
         maxcolor:=65536;
         palettesize:=65536;
         hardwarepages:=0;
         InitMode       := @ptc_InitModeProc;
         DirectPutPixel := @ptc_DirectPixelProc_16bpp;
         GetPixel       := @ptc_GetPixelProc_16bpp;
         PutPixel       := @ptc_PutPixelProc_16bpp;
         SetRGBPalette  := @ptc_SetRGBPaletteProc;
         GetRGBPalette  := @ptc_GetRGBPaletteProc;

         HLine          := @ptc_HLineProc_16bpp;
       end;
     addmode(graphmode);}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=CGAC0;
       DriverNumber := CGA;
       HardwarePages := 0;
       ModeName:='320 x 200 CGA C0';
       MaxColor := 4;
       DirectColor := FALSE;
       PaletteSize := 16;
       MaxX := 319;
       MaxY := 199;
       InitMode       := @ptc_Init320x200x4cgaC0;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       SetBkColor     := @SetBkColorCGA320;
       GetBkColor     := @GetBkColorCGA320;

       XAspect := 8333;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=CGAC1;
       DriverNumber := CGA;
       HardwarePages := 0;
       ModeName:='320 x 200 CGA C1';
       MaxColor := 4;
       DirectColor := FALSE;
       PaletteSize := 16;
       MaxX := 319;
       MaxY := 199;
       InitMode       := @ptc_Init320x200x4cgaC1;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       SetBkColor     := @SetBkColorCGA320;
       GetBkColor     := @GetBkColorCGA320;

       XAspect := 8333;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=CGAC2;
       DriverNumber := CGA;
       HardwarePages := 0;
       ModeName:='320 x 200 CGA C2';
       MaxColor := 4;
       DirectColor := FALSE;
       PaletteSize := 16;
       MaxX := 319;
       MaxY := 199;
       InitMode       := @ptc_Init320x200x4cgaC2;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       SetBkColor     := @SetBkColorCGA320;
       GetBkColor     := @GetBkColorCGA320;

       XAspect := 8333;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=CGAC3;
       DriverNumber := CGA;
       HardwarePages := 0;
       ModeName:='320 x 200 CGA C3';
       MaxColor := 4;
       DirectColor := FALSE;
       PaletteSize := 16;
       MaxX := 319;
       MaxY := 199;
       InitMode       := @ptc_Init320x200x4cgaC3;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       SetBkColor     := @SetBkColorCGA320;
       GetBkColor     := @GetBkColorCGA320;

       XAspect := 8333;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=CGAHi;
       DriverNumber := CGA;
       HardwarePages := 0;
       ModeName:='640 x 200 CGA';
       MaxColor := 2;
       DirectColor := FALSE;
       PaletteSize := 16;
       MaxX := 639;
       MaxY := 199;
       InitMode       := @ptc_Init640x200x2;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       SetBkColor     := @SetBkColorCGA640;
       GetBkColor     := @GetBkColorCGA640;

       XAspect := 4167;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=MCGAC0;
       DriverNumber := MCGA;
       HardwarePages := 0;
       ModeName:='320 x 200 CGA C0';
       MaxColor := 4;
       DirectColor := FALSE;
       PaletteSize := 16;
       MaxX := 319;
       MaxY := 199;
       InitMode       := @ptc_Init320x200x4cgaC0;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       SetBkColor     := @SetBkColorCGA320;
       GetBkColor     := @GetBkColorCGA320;

       XAspect := 8333;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=MCGAC1;
       DriverNumber := MCGA;
       HardwarePages := 0;
       ModeName:='320 x 200 CGA C1';
       MaxColor := 4;
       DirectColor := FALSE;
       PaletteSize := 16;
       MaxX := 319;
       MaxY := 199;
       InitMode       := @ptc_Init320x200x4cgaC1;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       SetBkColor     := @SetBkColorCGA320;
       GetBkColor     := @GetBkColorCGA320;

       XAspect := 8333;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=MCGAC2;
       DriverNumber := MCGA;
       HardwarePages := 0;
       ModeName:='320 x 200 CGA C2';
       MaxColor := 4;
       DirectColor := FALSE;
       PaletteSize := 16;
       MaxX := 319;
       MaxY := 199;
       InitMode       := @ptc_Init320x200x4cgaC2;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       SetBkColor     := @SetBkColorCGA320;
       GetBkColor     := @GetBkColorCGA320;

       XAspect := 8333;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=MCGAC3;
       DriverNumber := MCGA;
       HardwarePages := 0;
       ModeName:='320 x 200 CGA C3';
       MaxColor := 4;
       DirectColor := FALSE;
       PaletteSize := 16;
       MaxX := 319;
       MaxY := 199;
       InitMode       := @ptc_Init320x200x4cgaC3;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       SetBkColor     := @SetBkColorCGA320;
       GetBkColor     := @GetBkColorCGA320;

       XAspect := 8333;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=MCGAMed;
       DriverNumber := MCGA;
       HardwarePages := 0;
       ModeName:='640 x 200 CGA';
       MaxColor := 2;
       DirectColor := FALSE;
       PaletteSize := 16;
       MaxX := 639;
       MaxY := 199;
       InitMode       := @ptc_Init640x200x2;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       SetBkColor     := @SetBkColorCGA640;
       GetBkColor     := @GetBkColorCGA640;

       XAspect := 4167;
       YAspect := 10000;
     end;
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
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

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
         GetPixel       := @ptc_GetPixelProc_8bpp;
         SetRGBPalette  := @ptc_SetRGBPaletteProc;
         GetRGBPalette  := @ptc_GetRGBPaletteProc;
         HLine          := @ptc_HLineProc_8bpp;
         VLine          := @ptc_VLineProc_8bpp;
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
     with graphmode do
     begin
       ModeNumber:=EGALo;
       DriverNumber := EGA;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       HardwarePages := 3;
=======
       HardwarePages := 2;
>>>>>>> graemeg/cpstrnew
=======
       HardwarePages := 2;
>>>>>>> graemeg/cpstrnew
=======
       HardwarePages := 2;
>>>>>>> graemeg/cpstrnew
=======
       HardwarePages := 2;
>>>>>>> origin/cpstrnew
       ModeName:='640 x 200 EGA';
       MaxColor := 16;
       DirectColor := FALSE;
       PaletteSize := MaxColor;
       MaxX := 639;
       MaxY := 199;
       InitMode       := @ptc_Init640x200x16;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       XAspect := 4500;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=EGAHi;
       DriverNumber := EGA;
       HardwarePages := 1;
       ModeName:='640 x 350 EGA';
       MaxColor := 16;
       DirectColor := FALSE;
       PaletteSize := MaxColor;
       MaxX := 639;
       MaxY := 349;
       InitMode       := @ptc_Init640x350x16;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       XAspect := 7750;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=VGALo;
       DriverNumber := VGA;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       HardwarePages := 3;
=======
       HardwarePages := 2;
>>>>>>> graemeg/cpstrnew
=======
       HardwarePages := 2;
>>>>>>> graemeg/cpstrnew
=======
       HardwarePages := 2;
>>>>>>> graemeg/cpstrnew
=======
       HardwarePages := 2;
>>>>>>> origin/cpstrnew
       ModeName:='640 x 200 EGA';
       MaxColor := 16;
       DirectColor := FALSE;
       PaletteSize := MaxColor;
       MaxX := 639;
       MaxY := 199;
       InitMode       := @ptc_Init640x200x16;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       XAspect := 4500;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=VGAMed;
       DriverNumber := VGA;
       HardwarePages := 1;
       ModeName:='640 x 350 EGA';
       MaxColor := 16;
       DirectColor := FALSE;
       PaletteSize := MaxColor;
       MaxX := 639;
       MaxY := 349;
       InitMode       := @ptc_Init640x350x16;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       XAspect := 7750;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=VGAHi;
       DriverNumber := VGA;
       HardwarePages := 0;
       ModeName:='640 x 480 VGA';
       MaxColor := 16;
       DirectColor := FALSE;
       PaletteSize := MaxColor;
       MaxX := 639;
       MaxY := 479;
       InitMode       := @ptc_Init640x480x16;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

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
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;
       //SetAllPalette  := @ptc_SetRGBAllPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

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
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;
       //SetAllPalette  := @ptc_SetRGBAllPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       XAspect := 8333;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=m640x400x256;
       DriverNumber := VESA;
       HardwarePages := 1;
       ModeName:='640 x 400 VESA';
       MaxColor := 256;
       DirectColor := FALSE;
       PaletteSize := MaxColor;
       MaxX := 639;
       MaxY := 399;
       InitMode       := @ptc_Init640x400x256;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;
       //SetAllPalette  := @ptc_SetRGBAllPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       XAspect := 8333;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber:=m640x480x256;
       DriverNumber := VESA;
       HardwarePages := 1;
       ModeName:='640 x 480 VESA';
       MaxColor := 256;
       DirectColor := FALSE;
       PaletteSize := MaxColor;
       MaxX := 639;
       MaxY := 479;
       InitMode       := @ptc_Init640x480x256;
       DirectPutPixel := @ptc_DirectPixelProc_8bpp;
       PutPixel       := @ptc_PutPixelProc_8bpp;
       GetPixel       := @ptc_GetPixelProc_8bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;
       //SetAllPalette  := @ptc_SetRGBAllPaletteProc;

       HLine          := @ptc_HLineProc_8bpp;
       VLine          := @ptc_VLineProc_8bpp;

       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       XAspect := 10000;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber := m320x200x32k;
       DriverNumber := VESA;
       HardwarePages := 1;
       ModeName:='320 x 200 VESA';
       MaxColor := 32768;
       DirectColor := TRUE;
       PaletteSize := MaxColor;
       MaxX := 319;
       MaxY := 199;
       InitMode       := @ptc_Init320x200x32k;
       DirectPutPixel := @ptc_DirectPixelProc_16bpp;
       PutPixel       := @ptc_PutPixelProc_16bpp;
       GetPixel       := @ptc_GetPixelProc_16bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;
       HLine          := @ptc_HLineProc_16bpp;
       VLine          := @ptc_VLineProc_16bpp;
       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       XAspect := 8333;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber := m640x480x32k;
       DriverNumber := VESA;
       HardwarePages := 1;
       ModeName:='640 x 480 VESA';
       MaxColor := 32768;
       DirectColor := TRUE;
       PaletteSize := MaxColor;
       MaxX := 639;
       MaxY := 479;
       InitMode       := @ptc_Init640x480x32k;
       DirectPutPixel := @ptc_DirectPixelProc_16bpp;
       PutPixel       := @ptc_PutPixelProc_16bpp;
       GetPixel       := @ptc_GetPixelProc_16bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;
       HLine          := @ptc_HLineProc_16bpp;
       VLine          := @ptc_VLineProc_16bpp;
       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       XAspect := 10000;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber := m320x200x64k;
       DriverNumber := VESA;
       HardwarePages := 1;
       ModeName:='320 x 200 VESA';
       MaxColor := 65536;
       DirectColor := TRUE;
       PaletteSize := MaxColor;
       MaxX := 319;
       MaxY := 199;
       InitMode       := @ptc_Init320x200x64k;
       DirectPutPixel := @ptc_DirectPixelProc_16bpp;
       PutPixel       := @ptc_PutPixelProc_16bpp;
       GetPixel       := @ptc_GetPixelProc_16bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;
       HLine          := @ptc_HLineProc_16bpp;
       VLine          := @ptc_VLineProc_16bpp;
       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       XAspect := 8333;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     InitMode(graphmode);
     with graphmode do
     begin
       ModeNumber := m640x480x64k;
       DriverNumber := VESA;
       HardwarePages := 1;
       ModeName:='640 x 480 VESA';
       MaxColor := 65536;
       DirectColor := TRUE;
       PaletteSize := MaxColor;
       MaxX := 639;
       MaxY := 479;
       InitMode       := @ptc_Init640x480x64k;
       DirectPutPixel := @ptc_DirectPixelProc_16bpp;
       PutPixel       := @ptc_PutPixelProc_16bpp;
       GetPixel       := @ptc_GetPixelProc_16bpp;
       SetRGBPalette  := @ptc_SetRGBPaletteProc;
       GetRGBPalette  := @ptc_GetRGBPaletteProc;
       HLine          := @ptc_HLineProc_16bpp;
       VLine          := @ptc_VLineProc_16bpp;
       SetVisualPage  := @ptc_SetVisualPage;
       SetActivePage  := @ptc_SetActivePage;

       XAspect := 10000;
       YAspect := 10000;
     end;
     AddMode(graphmode);

     if ContainsAtLeast(800, 600) then
     begin
       InitMode(graphmode);
       with graphmode do
       begin
         ModeNumber := m800x600x16;
         DriverNumber := VESA;
         HardwarePages := 1;
         ModeName:='800 x 600 VESA';
         MaxColor := 16;
         DirectColor := FALSE;
         PaletteSize := MaxColor;
         MaxX := 799;
         MaxY := 599;
         InitMode       := @ptc_Init800x600x16;
         DirectPutPixel := @ptc_DirectPixelProc_8bpp;
         PutPixel       := @ptc_PutPixelProc_8bpp;
         GetPixel       := @ptc_GetPixelProc_8bpp;
         SetRGBPalette  := @ptc_SetRGBPaletteProc;
         GetRGBPalette  := @ptc_GetRGBPaletteProc;

         HLine          := @ptc_HLineProc_8bpp;
         VLine          := @ptc_VLineProc_8bpp;

         SetVisualPage  := @ptc_SetVisualPage;
         SetActivePage  := @ptc_SetActivePage;

         XAspect := 10000;
         YAspect := 10000;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       with graphmode do
       begin
         ModeNumber:=m800x600x256;
         DriverNumber := VESA;
         HardwarePages := 1;
         ModeName:='800 x 600 VESA';
         MaxColor := 256;
         DirectColor := FALSE;
         PaletteSize := MaxColor;
         MaxX := 799;
         MaxY := 599;
         InitMode       := @ptc_Init800x600x256;
         DirectPutPixel := @ptc_DirectPixelProc_8bpp;
         PutPixel       := @ptc_PutPixelProc_8bpp;
         GetPixel       := @ptc_GetPixelProc_8bpp;
         SetRGBPalette  := @ptc_SetRGBPaletteProc;
         GetRGBPalette  := @ptc_GetRGBPaletteProc;
         //SetAllPalette  := @ptc_SetRGBAllPaletteProc;

         HLine          := @ptc_HLineProc_8bpp;
         VLine          := @ptc_VLineProc_8bpp;

         SetVisualPage  := @ptc_SetVisualPage;
         SetActivePage  := @ptc_SetActivePage;

         XAspect := 10000;
         YAspect := 10000;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       with graphmode do
       begin
         ModeNumber := m800x600x32k;
         DriverNumber := VESA;
         HardwarePages := 1;
         ModeName:='800 x 600 VESA';
         MaxColor := 32768;
         DirectColor := TRUE;
         PaletteSize := MaxColor;
         MaxX := 799;
         MaxY := 599;
         InitMode       := @ptc_Init800x600x32k;
         DirectPutPixel := @ptc_DirectPixelProc_16bpp;
         PutPixel       := @ptc_PutPixelProc_16bpp;
         GetPixel       := @ptc_GetPixelProc_16bpp;
         SetRGBPalette  := @ptc_SetRGBPaletteProc;
         GetRGBPalette  := @ptc_GetRGBPaletteProc;
         HLine          := @ptc_HLineProc_16bpp;
         VLine          := @ptc_VLineProc_16bpp;
         SetVisualPage  := @ptc_SetVisualPage;
         SetActivePage  := @ptc_SetActivePage;

         XAspect := 10000;
         YAspect := 10000;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       with graphmode do
       begin
         ModeNumber := m800x600x64k;
         DriverNumber := VESA;
         HardwarePages := 1;
         ModeName:='800 x 600 VESA';
         MaxColor := 65536;
         DirectColor := TRUE;
         PaletteSize := MaxColor;
         MaxX := 799;
         MaxY := 599;
         InitMode       := @ptc_Init800x600x64k;
         DirectPutPixel := @ptc_DirectPixelProc_16bpp;
         PutPixel       := @ptc_PutPixelProc_16bpp;
         GetPixel       := @ptc_GetPixelProc_16bpp;
         SetRGBPalette  := @ptc_SetRGBPaletteProc;
         GetRGBPalette  := @ptc_GetRGBPaletteProc;
         HLine          := @ptc_HLineProc_16bpp;
         VLine          := @ptc_VLineProc_16bpp;
         SetVisualPage  := @ptc_SetVisualPage;
         SetActivePage  := @ptc_SetActivePage;

         XAspect := 10000;
         YAspect := 10000;
       end;
       AddMode(graphmode);
     end;

     if ContainsAtLeast(1024, 768) then
     begin
       InitMode(graphmode);
       with graphmode do
       begin
         ModeNumber := m1024x768x16;
         DriverNumber := VESA;
         HardwarePages := 1;
         ModeName:='1024 x 768 VESA';
         MaxColor := 16;
         DirectColor := FALSE;
         PaletteSize := MaxColor;
         MaxX := 1023;
         MaxY := 767;
         InitMode       := @ptc_Init1024x768x16;
         DirectPutPixel := @ptc_DirectPixelProc_8bpp;
         PutPixel       := @ptc_PutPixelProc_8bpp;
         GetPixel       := @ptc_GetPixelProc_8bpp;
         SetRGBPalette  := @ptc_SetRGBPaletteProc;
         GetRGBPalette  := @ptc_GetRGBPaletteProc;

         HLine          := @ptc_HLineProc_8bpp;
         VLine          := @ptc_VLineProc_8bpp;

         SetVisualPage  := @ptc_SetVisualPage;
         SetActivePage  := @ptc_SetActivePage;

         XAspect := 10000;
         YAspect := 10000;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       with graphmode do
       begin
         ModeNumber:=m1024x768x256;
         DriverNumber := VESA;
         HardwarePages := 1;
         ModeName:='1024 x 768 VESA';
         MaxColor := 256;
         DirectColor := FALSE;
         PaletteSize := MaxColor;
         MaxX := 1023;
         MaxY := 767;
         InitMode       := @ptc_Init1024x768x256;
         DirectPutPixel := @ptc_DirectPixelProc_8bpp;
         PutPixel       := @ptc_PutPixelProc_8bpp;
         GetPixel       := @ptc_GetPixelProc_8bpp;
         SetRGBPalette  := @ptc_SetRGBPaletteProc;
         GetRGBPalette  := @ptc_GetRGBPaletteProc;
         //SetAllPalette  := @ptc_SetRGBAllPaletteProc;

         HLine          := @ptc_HLineProc_8bpp;
         VLine          := @ptc_VLineProc_8bpp;

         SetVisualPage  := @ptc_SetVisualPage;
         SetActivePage  := @ptc_SetActivePage;

         XAspect := 10000;
         YAspect := 10000;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       with graphmode do
       begin
         ModeNumber := m1024x768x32k;
         DriverNumber := VESA;
         HardwarePages := 1;
         ModeName:='1024 x 768 VESA';
         MaxColor := 32768;
         DirectColor := TRUE;
         PaletteSize := MaxColor;
         MaxX := 1023;
         MaxY := 767;
         InitMode       := @ptc_Init1024x768x32k;
         DirectPutPixel := @ptc_DirectPixelProc_16bpp;
         PutPixel       := @ptc_PutPixelProc_16bpp;
         GetPixel       := @ptc_GetPixelProc_16bpp;
         SetRGBPalette  := @ptc_SetRGBPaletteProc;
         GetRGBPalette  := @ptc_GetRGBPaletteProc;
         HLine          := @ptc_HLineProc_16bpp;
         VLine          := @ptc_VLineProc_16bpp;
         SetVisualPage  := @ptc_SetVisualPage;
         SetActivePage  := @ptc_SetActivePage;

         XAspect := 10000;
         YAspect := 10000;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       with graphmode do
       begin
         ModeNumber := m1024x768x64k;
         DriverNumber := VESA;
         HardwarePages := 1;
         ModeName:='1024 x 768 VESA';
         MaxColor := 65536;
         DirectColor := TRUE;
         PaletteSize := MaxColor;
         MaxX := 1023;
         MaxY := 767;
         InitMode       := @ptc_Init1024x768x64k;
         DirectPutPixel := @ptc_DirectPixelProc_16bpp;
         PutPixel       := @ptc_PutPixelProc_16bpp;
         GetPixel       := @ptc_GetPixelProc_16bpp;
         SetRGBPalette  := @ptc_SetRGBPaletteProc;
         GetRGBPalette  := @ptc_GetRGBPaletteProc;
         HLine          := @ptc_HLineProc_16bpp;
         VLine          := @ptc_VLineProc_16bpp;
         SetVisualPage  := @ptc_SetVisualPage;
         SetActivePage  := @ptc_SetActivePage;

         XAspect := 10000;
         YAspect := 10000;
       end;
       AddMode(graphmode);
     end;

     if ContainsAtLeast(1280, 1024) then
     begin
       InitMode(graphmode);
       with graphmode do
       begin
         ModeNumber := m1280x1024x16;
         DriverNumber := VESA;
         HardwarePages := 1;
         ModeName:='1280 x 1024 VESA';
         MaxColor := 16;
         DirectColor := FALSE;
         PaletteSize := MaxColor;
         MaxX := 1279;
         MaxY := 1023;
         InitMode       := @ptc_Init1280x1024x16;
         DirectPutPixel := @ptc_DirectPixelProc_8bpp;
         PutPixel       := @ptc_PutPixelProc_8bpp;
         GetPixel       := @ptc_GetPixelProc_8bpp;
         SetRGBPalette  := @ptc_SetRGBPaletteProc;
         GetRGBPalette  := @ptc_GetRGBPaletteProc;

         HLine          := @ptc_HLineProc_8bpp;
         VLine          := @ptc_VLineProc_8bpp;

         SetVisualPage  := @ptc_SetVisualPage;
         SetActivePage  := @ptc_SetActivePage;

         XAspect := 10000;
         YAspect := 10000;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       with graphmode do
       begin
         ModeNumber:=m1280x1024x256;
         DriverNumber := VESA;
         HardwarePages := 1;
         ModeName:='1280 x 1024 VESA';
         MaxColor := 256;
         DirectColor := FALSE;
         PaletteSize := MaxColor;
         MaxX := 1279;
         MaxY := 1023;
         InitMode       := @ptc_Init1280x1024x256;
         DirectPutPixel := @ptc_DirectPixelProc_8bpp;
         PutPixel       := @ptc_PutPixelProc_8bpp;
         GetPixel       := @ptc_GetPixelProc_8bpp;
         SetRGBPalette  := @ptc_SetRGBPaletteProc;
         GetRGBPalette  := @ptc_GetRGBPaletteProc;
         //SetAllPalette  := @ptc_SetRGBAllPaletteProc;

         HLine          := @ptc_HLineProc_8bpp;
         VLine          := @ptc_VLineProc_8bpp;

         SetVisualPage  := @ptc_SetVisualPage;
         SetActivePage  := @ptc_SetActivePage;

         XAspect := 10000;
         YAspect := 10000;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       with graphmode do
       begin
         ModeNumber := m1280x1024x32k;
         DriverNumber := VESA;
         HardwarePages := 1;
         ModeName:='1280 x 1024 VESA';
         MaxColor := 32768;
         DirectColor := TRUE;
         PaletteSize := MaxColor;
         MaxX := 1279;
         MaxY := 1023;
         InitMode       := @ptc_Init1280x1024x32k;
         DirectPutPixel := @ptc_DirectPixelProc_16bpp;
         PutPixel       := @ptc_PutPixelProc_16bpp;
         GetPixel       := @ptc_GetPixelProc_16bpp;
         SetRGBPalette  := @ptc_SetRGBPaletteProc;
         GetRGBPalette  := @ptc_GetRGBPaletteProc;
         HLine          := @ptc_HLineProc_16bpp;
         VLine          := @ptc_VLineProc_16bpp;
         SetVisualPage  := @ptc_SetVisualPage;
         SetActivePage  := @ptc_SetActivePage;

         XAspect := 10000;
         YAspect := 10000;
       end;
       AddMode(graphmode);

       InitMode(graphmode);
       with graphmode do
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       begin
=======
       begin  
>>>>>>> graemeg/cpstrnew
=======
       begin  
>>>>>>> graemeg/cpstrnew
=======
       begin  
>>>>>>> graemeg/cpstrnew
=======
       begin  
>>>>>>> origin/cpstrnew
         ModeNumber := m1280x1024x64k;
         DriverNumber := VESA;
         HardwarePages := 1;
         ModeName:='1280 x 1024 VESA';
         MaxColor := 65536;
         DirectColor := TRUE;
         PaletteSize := MaxColor;
         MaxX := 1279;
         MaxY := 1023;
         InitMode       := @ptc_Init1280x1024x64k;
         DirectPutPixel := @ptc_DirectPixelProc_16bpp;
         PutPixel       := @ptc_PutPixelProc_16bpp;
         GetPixel       := @ptc_GetPixelProc_16bpp;
         SetRGBPalette  := @ptc_SetRGBPaletteProc;
         GetRGBPalette  := @ptc_GetRGBPaletteProc;
         HLine          := @ptc_HLineProc_16bpp;
         VLine          := @ptc_VLineProc_16bpp;
         SetVisualPage  := @ptc_SetVisualPage;
         SetActivePage  := @ptc_SetActivePage;

         XAspect := 10000;
         YAspect := 10000;
       end;
       AddMode(graphmode);
     end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  end;

initialization
  PTCFormat8 := TPTCFormatFactory.CreateNew(8);
  PTCFormat15 := TPTCFormatFactory.CreateNew(16, $7C00, $03E0, $001F);
  PTCFormat16 := TPTCFormatFactory.CreateNew(16, $F800, $07E0, $001F);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew



(*
     writeln('processing modes');
     while ptcmode^.valid do
       begin
         for d:=low(depths) to high(depths) do
           begin
             InitMode(graphmode);
             with graphmode do
               begin
                 ModeNumber:=I;
                 DriverNumber:=ptcgraph.PTC;
                 { MaxX is number of pixels in X direction - 1}
                 MaxX:=ptcmode^.width-1;
                 { same for MaxY}
                 MaxY:=ptcmode^.height-1;
                 str(ptcmode^.width,ws);
                 str(ptcmode^.height,hs);
                 modename:='PTC_'+ws+'x'+hs+'x'+depth_names[d];
                 MaxColor := 1 shl ptcmode^.format.r * 1 shl ptcmode^.format.g *1 shl ptcmode^.format.b;
                 writeln('mode ',modename,' ',maxcolor,'kleuren');
                 PaletteSize := MaxColor;
                 HardwarePages := 0;
*)
                 { necessary hooks ...}
(*
                 if (MaxColor = 16) and
                   (LongInt(ModeInfo.Width) * LongInt(ModeInfo.Height) < 65536*4*2) then
                  begin
                   {Use optimized graphics routines for 4 bit EGA/VGA modes.}
                   ScrWidth := ModeInfo.Width div 8;
                   DirectPutPixel := @DirectPutPixel16;
                   PutPixel := @PutPixel16;
                   GetPixel := @GetPixel16;
                   HLine := @HLine16;
                   VLine := @VLine16;
                   GetScanLine := @GetScanLine16;
                 end
               else
*)
(*
                 begin
                   DirectPutPixel := @ptc_DirectPixelProc;
                   GetPixel       := @ptc_GetPixelProc;
                   PutPixel       := @ptc_PutPixelProc;
                   { May be implemented later:
                   HLine          := @libvga_HLineProc;
                   VLine          := @libvga_VLineProc;
                   GetScanLine    := @libvga_GetScanLineProc;}
                   ClearViewPort  := @ptc_ClrViewProc;
                 end;
                 SetRGBPalette  := @ptc_SetRGBPaletteProc;
                 GetRGBPalette  := @ptc_GetRGBPaletteProc;
                 { These are not really implemented yet:
                 PutImage       := @libvga_PutImageProc;
                 GetImage       := @libvga_GetImageProc;}
{                If you use the default getimage/putimage, you also need the default
                 imagesize! (JM)
                 ImageSize      := @libvga_ImageSizeProc; }
                 { Add later maybe ?
                 SetVisualPage  := SetVisualPageProc;
                 SetActivePage  := SetActivePageProc;
                 Line           := @libvga_LineProc;
                 InternalEllipse:= @libvga_EllipseProc;
                 PatternLine    := @libvga_PatternLineProc;
                 }
                 InitMode       := @ptc_InitModeProc;
               end;
           AddMode(graphmode);
           inc(i);
         end;
     end;
*)
  end;

initialization
  PTCFormat8 := TPTCFormat.Create(8);
  PTCFormat15 := TPTCFormat.Create(16, $7C00, $03E0, $001F);
  PTCFormat16 := TPTCFormat.Create(16, $F800, $07E0, $001F);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  PTCWrapperObject := TPTCWrapperThread.Create;
  InitializeGraph;
finalization
  PTCWrapperObject.Terminate;
  PTCWrapperObject.WaitFor;
  PTCWrapperObject.Free;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  PTCFormat16.Free;
  PTCFormat15.Free;
  PTCFormat8.Free;
>>>>>>> graemeg/cpstrnew
=======
  PTCFormat16.Free;
  PTCFormat15.Free;
  PTCFormat8.Free;
>>>>>>> graemeg/cpstrnew
=======
  PTCFormat16.Free;
  PTCFormat15.Free;
  PTCFormat8.Free;
>>>>>>> graemeg/cpstrnew
=======
  PTCFormat16.Free;
  PTCFormat15.Free;
  PTCFormat8.Free;
>>>>>>> origin/cpstrnew
end.
