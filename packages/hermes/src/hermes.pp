{
    Free Pascal port of the Hermes C library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C version by Christian Nentwich (c.nentwich@cs.ucl.ac.uk)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

Unit Hermes;

{$MODE objfpc}

Interface

Const
  HERMES_CONVERT_NORMAL = 0;
  HERMES_CONVERT_DITHER = 1;

Type
  THermesHandle = Integer;
  Pint32 = ^int32;
  int32 = DWord;
  Pshort16 = ^short16;
  short16 = Word;
  Pchar8 = ^char8;
  char8 = Byte;
  PHermesFormat = ^THermesFormat;
  THermesFormat = Record
    r,g,b,a : int32;
    bits : Integer;
    indexed : Boolean;
    has_colorkey : Boolean;
    colorkey : int32;
  End;

Function Hermes_FormatNewEmpty : PHermesFormat;

{ Create a new format structure, returns 0 if failed. }
Function Hermes_FormatNew(bits : Integer; r, g, b, a : int32;
                          indexed : Boolean) : PHermesFormat;

{ Free a format structure }
Procedure Hermes_FormatFree(fmt : PHermesFormat);

{ Create a new format structure with colorkey info, returns 0 if failed. }
Function Hermes_FormatNewEx(bits : Integer; r, g, b, a : int32;
                            indexed, has_colorkey : Boolean;
                            colorkey : int32) : PHermesFormat;

{ Compare two formats. Return true if they are equal, false otherwise }
Function Hermes_FormatEquals(op1, op2 : PHermesFormat) : Boolean;

{ Copy the contents of format 'source' to format 'destination' }
Procedure Hermes_FormatCopy(source, dest : PHermesFormat);


{
   Get a converter to work with, specifying a combination of the flags
   above. Returns 0 if unsuccessful.
}
Function Hermes_ConverterInstance(flags : DWord) : THermesHandle;

{
   Return a converter if it is not needed anymore, thus releasing some
   memory.
}
Procedure Hermes_ConverterReturn(handle : THermesHandle);

{
   Request a format conversion between two formats. This function returns false
   if the conversion cannot be provided (which should not occur too often :)
   Repeated calls to this function will be cached an terminate almost
   immediately, so don't be ashamed of calling it often.
}
Function Hermes_ConverterRequest(handle : THermesHandle;
                                 source, dest : PHermesFormat) : Boolean;

{
   Set the palette of the source surface / destination surface for a
   subsequent conversion. At the moment, only sourcepal is used.
   Returns false if unsuccessful (invalid handle!).
}
Function Hermes_ConverterPalette(handle, sourcepal, destpal : THermesHandle) : Boolean;

{
   Do a format conversion after calling the setup routines above. This will
   convert (or copy) the pixel data from s_pixels to the data in d_pixels.
   Both source and destination areas/origins can be specified as well as
   the scanline width in bytes of the source/destination.
   Returns false if unsuccessful (invalid handle or request not called before).
}
Function Hermes_ConverterCopy(handle : THermesHandle; s_pixels : Pointer;
                              s_x, s_y, s_width, s_height, s_pitch : Integer;
                              d_pixels : Pointer; d_x, d_y, d_width,
                              d_height, d_pitch : Integer) : Boolean;

(*
{-----------------H_BLIT---------------}

{
   Get a blitter to work with, specifying a combination of the flags
   in H_Conv. Returns 0 if unsuccessful.
}
Function Hermes_BlitterInstance(flags : DWord) : THermesHandle;

{
   Return a blitter if it is not needed anymore, thus releasing some
   memory.
}
Procedure Hermes_BlitterReturn(handle : THermesHandle);

{
   Request a format blitting between two formats. This function returns false
   if the blitting cannot be provided (which should not occur too often :)
   Repeated calls to this function will be cached an terminate almost
   immediately, so don't be ashamed of calling it often.
}
Function Hermes_BlitterRequest(handle : THermesHandle;
                               source, dest : PHermesFormat) : Boolean;

{
   Set the palette of the source surface / destination surface for a
   subsequent blitting. At the moment, only sourcepal is used.
   Returns false if unsuccessful (invalid handle!).
}
Function Hermes_BlitterPalette(handle, sourcepal, destpal : THermesHandle) : Boolean;

{
   Do a format blitting after calling the setup routines above. This will
   blit the pixel data from s_pixels to the data in d_pixels.  Both source
   and destination areas/origins can be specified as well as the scanline
   width in bytes of the source/destination.  Returns false if unsuccessful
   (invalid handle or request not called before).
}
Function Hermes_BlitterBlit(handle : THermesHandle; s_pixels : Pointer;
                            s_x, s_y, s_width, s_height, s_pitch : Integer;
                            d_pixels : Pointer; d_x, d_y, d_width, d_height,
                            d_pitch : Integer) : Boolean;
*)
{-----------------H_PAL---------------}

{ Get a handle for a palette to work with. This allocates memory for an
   internal palette. Returns 0 if failed.
}
Function Hermes_PaletteInstance : THermesHandle;

{
   Return a handle for a palette if the palette isn't used anymore. The
   internal palette will be deallocated.
}
Procedure Hermes_PaletteReturn(handle : THermesHandle);

{
   Copy the contents of the palette parameter provided into the internal
   palette. The user palette has to be 256*4 bytes long.
}
Procedure Hermes_PaletteSet(handle : THermesHandle; palette : Pointer);

{
   Return the pointer to the internal palette. The palette is 256*4 bytes
   long.
}
Function Hermes_PaletteGet(handle : THermesHandle) : Pointer;

{
   Force invalidation of the palette cache. This will force lookup tables to
   be regenerated and has to be done manually after PaletteGet has been used
   and the data has been modified without the knowledge of Hermes.
}
Procedure Hermes_PaletteInvalidateCache(handle : THermesHandle);


{-----------------H_CLEAR---------------}

{
   Get a handle for a new clearer instance to work with. Returns 0 if failed.
}
Function Hermes_ClearerInstance : THermesHandle;

{
   Return the clearer instance if it is no longer needed.
}
Procedure Hermes_ClearerReturn(handle : THermesHandle);

{
   Request the clearing routines to be set up for clearing to a specific
   format later. Repeated calls to the routine will be cached and terminate
   after a short check.
}
Function Hermes_ClearerRequest(handle : THermesHandle; format : PHermesFormat) : Boolean;

{
   Clear a surface. pixels points to the pixel data, x1, y1, width, height
   specify the area to clear, pitch is the width of a scanline in bytes,
   the rest are the colour components.
}
Function Hermes_ClearerClear(handle : THermesHandle; pixels : Pointer;
                             x1, y1, width, height, pitch : Integer;
                             r, g, b : int32; index : char8) : Boolean;


{ Initialise Hermes, returns false if failed }
Function Hermes_Init : Boolean;

{ Deinitialise Hermes, returns false if failed }
Function Hermes_Done : Boolean;

Implementation

{$I hermdef.inc}

Const
  PROC_GENERIC = 1;
  PROC_X86_PENTIUM = 2;
  PROC_MMX_PENTIUM = 4;
  HERMES_CONVERT_GENERIC = 65536;

{$I hermconf.inc}

Type
  PHermesClearInterface = ^THermesClearInterface;
  THermesClearInterface = Record
    dest : ^char8;
    value : int32;
    width, height : Integer;
    add : Integer;
  End;
  THermesClearPtr = Procedure(hci : PHermesClearInterface); CDecl;
  PHermesClearer = ^THermesClearer;
  THermesClearer = Record
    bits : Integer;
    func : THermesClearPtr;
  End;


{ Structure to hold shift amounts for the generic routines }
  PHermesGenericInfo = ^THermesGenericInfo;
  THermesGenericInfo = Record
    r_right, g_right, b_right, a_right : Integer; {Shift amount to the right}
    r_left, g_left, b_left, a_left : Integer; {Shift amount to the right}
  End;


{ Pointer to specialised (one-scanline-only) conversion procedure }
  THermesConverterPtr = Procedure(source, dest : Pchar8;
                                  count, inc_source : DWord); CDecl;

{ Structure for conversion loop routines, don't be scared, size does NOT
   matter in this case :) }
  PHermesConverterInterface = ^THermesConverterInterface;
  THermesConverterInterface = Record
    s_pixels : Pchar8;
    s_width,s_height : Integer;
    s_add : Integer;          { Offset to next line from end of line }

    d_pixels : Pchar8;
    d_width,d_height : Integer;
    d_add : Integer;

    func : THermesConverterPtr;

    lookup : Pint32;          { Palette lookup table ptr, for 8 bit }

    s_pitch : Integer;        { Source and destination pitch, }
    d_pitch : Integer;        { only used by C routines }

    info : THermesGenericInfo; { Only used by generic converters }
    mask_r, mask_g, mask_b, mask_a : int32; { Only used by generic converters }
    s_mask_a : int32;

    s_has_colorkey : Boolean;
    s_colorkey : int32;

    d_has_colorkey : Boolean;
    d_colorkey : int32;
  End;

{ Pointer to loop function (C, assembler main loop, generic routines) }
  THermesConverterLoopPtr = Procedure(hci : PHermesConverterInterface); CDecl;

  PHermesConverter = ^THermesConverter;
  THermesConverter = Record
    source,dest : THermesFormat;           { Source and destination format }
    lookup : Pint32;                       { Pointer to lookup table (8bit) }

    flags : DWord;                         { Defined in H_Conv.h, DITHER,etc}

    loopnormal : THermesConverterLoopPtr;  { Loop routine for normal conv. }
    loopstretch : THermesConverterLoopPtr;
    normal : THermesConverterPtr;          { One-scanline routine }
    stretch : THermesConverterPtr;

    dither : THermesConverterLoopPtr;        { Dithering routines always }
    ditherstretch : THermesConverterLoopPtr; { convert the whole buffer }
  End;

  PHermesFactoryStruct = ^THermesFactoryStruct;
  THermesFactoryStruct = Record
    s_bits : Integer;
    s_idx : Boolean;
    s_r, s_g, s_b, s_a : int32;
    d_bits : Integer;
    d_idx : Boolean;
    d_r, d_g, d_b, d_a : int32;

    loopnormal, loopstretch : THermesConverterLoopPtr;
    normal, stretch : THermesConverterPtr;
    dither, ditherstretch : THermesConverterLoopPtr;

    processor : Integer;
  End;

{dither types ?}

Const
{ p_converters holds a list of formats, for conversion from 32 bit, 24 bit,
  16 bit, muhmu and 8 bit.
  The destination formats are listed in the order of frequency they might
  occur so common formats can be retrieved faster.

  Format of a row:
  source bpp, s. indexed, s. r_mask, s. g_mask, s. b_mask, s. alpha ,dest bpp,
  d.indexed, d. r_mask, d. g_mask, d. b_mask, d. alpha
}
{ I wish I could touch this, but it's used in too many other placed in the code,
  ( at least indirectly), and many of the indicies are hardcoded }
  p_converters : Array[0..4, 0..11, 0..11] Of DWord =
  (
  ( {From 32 bit RGB 888}
  (32,0,$ff0000,$ff00,$ff,0,16,0,$f800,$7e0,$1f,0),          {16RGB565 }
  (32,0,$ff0000,$ff00,$ff,0, 8,0,$e0,$1c,$3,0),              { 8RGB332 }
  (32,0,$ff0000,$ff00,$ff,0,16,0,$7c00,$3e0,$1f,0),          { 16RGB555 }
  (32,0,$ff0000,$ff00,$ff,0,24,0,$ff0000,$ff00,$ff,0),       { 24RGB888 }
  (32,0,$ff0000,$ff00,$ff,0,32,0,$ff,$ff00,$ff0000,0),       { 32BGR888 }
  (32,0,$ff0000,$ff00,$ff,0,16,0,$1f,$7e0,$f800,0),          { 16BGR565 }
  (32,0,$ff0000,$ff00,$ff,0,16,0,$1f,$3e0,$7c00,0),          { 16BGR555 }
  (32,0,$ff0000,$ff00,$ff,0,32,0,$ff000000,$ff0000,$ff00,$ff), { 32RGBA888 }
  (32,0,$ff0000,$ff00,$ff,0,32,0,$ff00,$ff0000,$ff000000,$ff), { 32BGRA888 }
  (32,0,$ff0000,$ff00,$ff,0,24,0,$ff,$ff00,$ff0000,0),       { 24BGR888 }
  (0,0,0,0,0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0)
  )
  ,
  ( {From 24 bit RGB 888}
  (24,0,$ff0000,$ff00,$ff,0,32,0,$ff0000,$ff00,$ff,0),       { 32RGB888 }
  (24,0,$ff0000,$ff00,$ff,0,16,0,$f800,$7e0,$1f,0),          { 16RGB565 }
  (24,0,$ff0000,$ff00,$ff,0, 8,0,$e0,$1c,$3,0),              { 8RGB332 }
  (24,0,$ff0000,$ff00,$ff,0,16,0,$7c00,$3e0,$1f,0),          { 16RGB555 }
  (24,0,$ff0000,$ff00,$ff,0,32,0,$ff,$ff00,$ff0000,0),       { 32BGR888 }
  (24,0,$ff0000,$ff00,$ff,0,16,0,$1f,$7e0,$f800,0),          { 16BGR565 }
  (24,0,$ff0000,$ff00,$ff,0,16,0,$1f,$3e0,$7c00,0),          { 16BGR555 }
  (24,0,$ff0000,$ff00,$ff,0,32,0,$ff000000,$ff0000,$ff00,$ff), { 32RGBA888 }
  (24,0,$ff0000,$ff00,$ff,0,32,0,$ff00,$ff0000,$ff000000,$ff), { 32BGRA888 }
  (24,0,$ff0000,$ff00,$ff,0,24,0,$ff,$ff00,$ff0000,0),       { 24BGR888 }
  (0,0,0,0,0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0)
  )
  ,
  ( {From 16 bit RGB 565}
  (16,0,$f800,$7e0,$1f,0,32,0,$ff0000,$ff00,$ff,0),          { 32RGB888 }
  (16,0,$f800,$7e0,$1f,0, 8,0,$e0,$1c,$3,0),                 { 8RGB332 }
  (16,0,$f800,$7e0,$1f,0,16,0,$7c00,$3e0,$1f,0),             { 16RGB555 }
  (16,0,$f800,$7e0,$1f,0,24,0,$ff0000,$ff00,$ff,0),          { 24RGB888 }
  (16,0,$f800,$7e0,$1f,0,32,0,$ff,$ff00,$ff0000,0),          { 32BGR888 }
  (16,0,$f800,$7e0,$1f,0,16,0,$1f,$7e0,$f800,0),             { 16BGR565 }
  (16,0,$f800,$7e0,$1f,0,16,0,$1f,$3e0,$7c00,0),             { 16BGR555 }
  (16,0,$f800,$7e0,$1f,0,32,0,$ff000000,$ff0000,$ff00,$ff),    { 32RGBA888 }
  (16,0,$f800,$7e0,$1f,0,32,0,$ff00,$ff0000,$ff000000,$ff),    { 32BGRA888 }
  (16,0,$f800,$7e0,$1f,0,24,0,$ff,$ff00,$ff0000,0),          { 24BGR888 }
  (0,0,0,0,0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0)
  ),
  ( {From 32 bit muhmu}
  (32,0,$ff00000,$3fc00,$ff,0,32,0,$ff0000,$ff00,$ff,0),     { 32RGB888 }
  (32,0,$ff00000,$3fc00,$ff,0,16,0,$f800,$7e0,$1f,0),        { 16RGB565 }
  (32,0,$ff00000,$3fc00,$ff,0, 8,0,$e0,$1c,$3,0),            { 8RGB332 }
  (32,0,$ff00000,$3fc00,$ff,0,16,0,$7c00,$3e0,$1f,0),        { 16RGB555 }
  (32,0,$ff00000,$3fc00,$ff,0,24,0,$ff0000,$ff00,$ff,0),     { 24RGB888 }
  (32,0,$ff00000,$3fc00,$ff,0,32,0,$ff,$ff00,$ff0000,0),     { 32BGR888 }
  (32,0,$ff00000,$3fc00,$ff,0,16,0,$1f,$7e0,$f800,0),        { 16BGR565 }
  (32,0,$ff00000,$3fc00,$ff,0,16,0,$1f,$3e0,$7c00,0),        { 16BGR555 }
  (32,0,$ff00000,$3fc00,$ff,0,32,0,$ff000000,$ff0000,$ff00,$ff), { 32RGBA888 }
  (32,0,$ff00000,$3fc00,$ff,0,32,0,$ff00,$ff0000,$ff000000,$ff), { 32BGRA888 }
  (32,0,$ff00000,$3fc00,$ff,0,24,0,$ff,$ff00,$ff0000,0),     { 24BGR888 }
  (0,0,0,0,0,0,0,0,0,0,0,0)
  ),
  ( {From 8 bit indexed}
  (8,1,0,0,0,0,32,0,0,0,0,0),
  (8,1,0,0,0,0,24,0,0,0,0,0),
  (8,1,0,0,0,0,16,0,0,0,0,0),
  (8,1,0,0,0,0,8,0,0,0,0,0),
  (0,0,0,0,0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0),
  (0,0,0,0,0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0),
  (0,0,0,0,0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0),
  (0,0,0,0,0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0)
  )
  );
  numConverters : Array[0..4] Of Integer = (10,10,10,11,4);
  refcount : Integer = 0;

Var
  Clearers : Array[0..3] Of PHermesClearer;
  numClearers : Integer;
  standardConverters : Array[0..4] Of ^PHermesConverter;
  equalConverters : Array[0..3] Of PHermesConverter;

{$I malloc.inc}

{$I debug.inc}

{$I dither.inc}
{$I headp.inc}
{$IFDEF I386_ASSEMBLER}
  {$I i386/headi386.inc}
  {$I i386/headmmx.inc}
{$ENDIF I386_ASSEMBLER}
{$I factconv.inc}
{$I list.inc}
{$I utility.inc}
{$I format.inc}
{$I palette.inc}
{$I convert.inc}
{$I clear.inc}
{$I factory.inc}

Function Hermes_Init : Boolean;

Var
  i, j : Integer;
  source, dest : THermesFormat;

Begin
  If refcount > 0 Then
  Begin
    Inc(refcount);
    Hermes_Init := True;
    Exit;
  End;
  { Initialise hermes factory }
  Hermes_Factory_Init;

  { Instruct the factory to return clearing routines }
  Clearers[0] := Hermes_Factory_getClearer(32);
  Clearers[1] := Hermes_Factory_getClearer(24);
  Clearers[2] := Hermes_Factory_getClearer(16);
  Clearers[3] := Hermes_Factory_getClearer(8);
  numClearers := 4;

  { Use factory to obtain specialised converters }
  For j := 0 To 4 Do
  Begin
    standardConverters[j] := malloc(SizeOf(PHermesConverter)*numConverters[j]);
    For i := 0 To numConverters[j] - 1 Do
    Begin
      // xxx jm color keys not taken into consideration here
      FillChar(source, SizeOf(source), 0);
      FillChar(dest, SizeOf(dest), 0);
      source.bits := p_converters[j, i, 0];    dest.bits := p_converters[j, i, 6];
      source.indexed:= p_converters[j,i,1]<>0; dest.indexed:= p_converters[j,i,7]<>0;
      source.r := p_converters[j, i, 2];       dest.r := p_converters[j, i, 8];
      source.g := p_converters[j, i, 3];       dest.g := p_converters[j, i, 9];
      source.b := p_converters[j, i, 4];       dest.b := p_converters[j, i, 10];
      source.a := p_converters[j, i, 5];       dest.a := p_converters[j, i, 11];

      standardConverters[j][i] := Hermes_Factory_getConverter(@source, @dest);
    End;
  End;


  { Set up converters for equal colour formats }
  equalConverters[3] := Hermes_Factory_getEqualConverter(32);
  equalConverters[2] := Hermes_Factory_getEqualConverter(24);
  equalConverters[1] := Hermes_Factory_getEqualConverter(16);
  equalConverters[0] := Hermes_Factory_getEqualConverter(8);

  { Initialise dithering tables }
  Dither_SetupMatrices;

  Inc(refcount);
  Hermes_Init := True;
End;

Function Hermes_Done : Boolean;

Var
  i, j : Integer;

Begin
  Dec(refcount);
  If refcount < 0 Then
  Begin
    refcount := 0;
    Hermes_Done := False;
    Exit;
  End;
  If refcount = 0 Then
  Begin
    For i := 0 To 3 Do
    Begin
      If Clearers[i] <> Nil Then
      Begin
        free(Clearers[i]);
        Clearers[i] := Nil;
      End;
      If equalConverters[i] <> Nil Then
      Begin
        free(equalConverters[i]);
        equalConverters[i] := Nil;
      End;
    End;
    For i := 0 To 4 Do
    Begin
      If standardConverters[i] <> Nil Then
      Begin
        For j := 0 To numConverters[i] - 1 Do
          free(standardConverters[i][j]);
        free(standardConverters[i]);
      End;
      standardConverters[i] := Nil;
    End;
  End;
  Hermes_Done := True;
End;

Begin
  DebugInit;
End.
