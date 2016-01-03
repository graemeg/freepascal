{
    This file is part of the Free Pascal run time library.
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

{$define logging}

{******************************************************************************}
                                    interface
{******************************************************************************}

{$i graphh.inc}

{Driver number for PTC.}
const	PTC=22;

{******************************************************************************}
                                 implementation
{******************************************************************************}

uses
  termio,x86,ptc;

const
  InternalDriverName = 'PTCPas';

{$i graph.inc}

  type
    PByte = ^Byte;
    PLongInt = ^LongInt;

    PByteArray = ^TByteArray;
    TByteArray = array [0..MAXINT - 1] of Byte;



{ ---------------------------------------------------------------------
   SVGA bindings.

  ---------------------------------------------------------------------}

Const
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
  OldIO : TermIos;

  ptcconsole:TPTCconsole;
  ptcsurface:TPTCSurface;
  ptcformat:TPTCFormat;

Procedure SetRawMode(b:boolean);
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
End;


{ ---------------------------------------------------------------------
    Required procedures
  ---------------------------------------------------------------------}

var
  LastColor: smallint;   {Cache the last set color to improve speed}


procedure ptc_savevideostate;
begin
end;

procedure ptc_restorevideostate;
begin
{  vga_setmode(0);}
end;

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
  writeln('Initializing mode');
  { create format }
  ptcformat:=TPTCFormat.Create(16,$f800,$07e0,$001f);
  { open the console }
  ptcconsole.open(paramstr(0),ptcformat);
  { create surface matching console dimensions }
  ptcsurface:=TPTCSurface.Create(ptcconsole.width,ptcconsole.height,ptcformat);
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


procedure ptc_directpixelproc_16bpp(X,Y: smallint);

var color:word;
    pixels:Pword;

begin
  case CurrentWriteMode of
    XORPut:
      begin
      { getpixel wants local/relative coordinates }
      Color := GetPixel(x-StartXViewPort,y-StartYViewPort);
      Color := CurrentColor Xor Color;
      end;
    OrPut:
      begin
      { getpixel wants local/relative coordinates }
      Color := GetPixel(x-StartXViewPort,y-StartYViewPort);
      Color := CurrentColor Or Color;
      end;
    AndPut:
      begin
      { getpixel wants local/relative coordinates }
      Color := GetPixel(x-StartXViewPort,y-StartYViewPort);
      Color := CurrentColor And Color;
      end;
    NotPut:
      begin
      Color := Not Color;
      end
  else
    Color:=CurrentColor;
  end;
  pixels:=ptcsurface.lock;
  {Plot the pixel on the surface.}
  pixels[x+y*ptcsurface.width]:=color;
  ptcsurface.unlock;
  { copy to console }
  ptcsurface.copy(ptcconsole);
  { update console }
  ptcconsole.update;
end;

procedure ptc_putpixelproc_16bpp(X,Y:smallint;Color:Word);

var pixels:Pword;

begin
  if clipcoords(X,Y) then
    begin
      pixels:=ptcsurface.lock;
{      pixels:=ptcconsole.lock;}
      {Plot the pixel on the surface.}
      pixels[x+y*ptcsurface.width]:=color;
      ptcsurface.unlock;
      { copy to console }
      ptcsurface.copy(ptcconsole);
      { update console }
      ptcconsole.update;
    end;
end;

function ptc_getpixelproc_16bpp(X,Y: smallint):word;

var pixels:Pword;

begin
  if clipcoords(X,Y) then
    begin
      pixels:=ptcsurface.lock;
      {Get the pixel from the surface.}
      ptc_getpixelproc_16bpp:=pixels[x+y*ptcsurface.width];
      ptcsurface.unlock;
    end;
end;


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

procedure ptc_hlineproc_16bpp(x, x2,y : smallint);

var pixels:Pword;
    i:word;

begin
  {Clip.}
  if (y<0) or (y>viewheight) then
    exit;
  if x<0 then
    x:=0;
  if x>viewwidth then
    x:=viewwidth;
  if x2<0 then
    x2:=0;
  if x>viewwidth then
    x2:=viewwidth;
  pixels:=ptcsurface.lock;
  inc(x,StartXViewPort);
  inc(x2,StartXViewPort);
  inc(y,StartXViewPort);
  {Plot the pixel on the surface.}
  for i:=x to x2 do
    pixels[i+y*ptcsurface.width]:=$ffff;
  ptcsurface.unlock;
  { copy to console }
  ptcsurface.copy(ptcconsole);
  { update console }
  ptcconsole.update;
end;

procedure ptc_vlineproc (x,y,y2: smallint);
begin
end;

procedure ptc_clrviewproc_16bpp;

Var I,Xmax : longint;

begin
  Xmax:=StartXViewPort+ViewWidth-1;
  For i:=StartYViewPort to StartYViewPort+ViewHeight-1 do
    ptc_hlineproc_16bpp(0,viewwidth,i);
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

procedure ptc_setrgbpaletteproc(ColorNum, RedValue, GreenValue, BlueValue: smallint);
begin
{  vga_setpalette(ColorNum,RedValue shr 2,GreenValue shr 2,BlueValue shr 2);}
end;

procedure ptc_getrgbpaletteproc (ColorNum: smallint;
                                    var RedValue, GreenValue, BlueValue: smallint);

Var R,G,B : longint;

begin
{  vga_getpalette(ColorNum,R,G,B);}
  RedValue:=R * 255 div 63;
  GreenValue:=G * 255 div 63;
  BlueValue:=B * 255 div 63;
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
    SetRawMode(False);
    RestoreVideoState;
    isgraphmode := false;
 end;

  function QueryAdapterInfo:PModeInfo;
  { This routine returns the head pointer to the list }
  { of supported graphics modes.                      }
  { Returns nil if no graphics mode supported.        }
  { This list is READ ONLY!                           }
   var
    graphmode:Tmodeinfo;
    ptcmode: PPTCmode;
    d,i : longint;
    ws,hs:string[5];

   const depths:array[0..3] of byte=(8,16,24,32);
         colours:array[0..3] of longint=(256,65536,16777216,16777216);
         depth_names:array[0..3] of string[5]=('256','64K','16M','16M32');

   begin
     QueryAdapterInfo := ModeList;
     { If the mode listing already exists... }
     { simply return it, without changing    }
     { anything...                           }
     if assigned(ModeList) then
       exit;
     SaveVideoState:=@ptc_savevideostate;
     RestoreVideoState:=@ptc_restorevideostate;
     ptcconsole:=TPTCconsole.create;
     ptcmode:=ptcconsole.modes;
     i:=0;
     initmode(graphmode);
     with graphmode do
       begin
         modenumber:=0;
         drivernumber:=ptcgraph.ptc;
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
       end;
     addmode(graphmode);
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
  ptcconsole:=TPTCconsole.create;
  InitializeGraph;
finalization
  ptcconsole.destroy;
end.
