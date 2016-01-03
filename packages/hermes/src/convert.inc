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


{Function Hermes_ConverterInstance(flags : DWord) : THermesHandle;
Procedure Hermes_ConverterReturn(handle : THermesHandle);}

Var
{ ConverterList is a list of HermesConverter* }
  ConverterList : ^PHermesConverter;
  ConvertCurrenthandle : THermesHandle;

Const
  lastConverter : Integer = 0;                { Array size, one beyond end }
  CONVERTrefcount : Integer = 0;

Function Hermes_ConverterInstance(flags : DWord) : THermesHandle;

Var
  i : Integer;
  newinstance : PHermesConverter;
  newlist : ^PHermesConverter;

Begin
  { Initialising, allocate initial size array of converters }
  If CONVERTrefcount = 0 Then
  Begin
    ConverterList := malloc(SizeOf(PHermesConverter) * HERMES_INITIAL);
    If ConverterList = Nil Then
    Begin
      Hermes_ConverterInstance := 0;
      Exit;
    End;
    lastConverter := HERMES_INITIAL;
    ConvertCurrenthandle := 1;
    For i := 0 To lastConverter - 1 Do
      ConverterList[i] := Nil;
{    DEBUG_PRINT("Creating dynamic array(convert.c:57), size %d\n",HERMES_INITIAL)}
  End;

  { Uh oh, arrary too small, time for growth }
  If ConvertCurrenthandle = lastConverter Then
  Begin
    { I'm told realloc isn't completely portable !? Let's do it by hand }
    newlist := malloc(SizeOf(PHermesConverter)*(lastConverter+HERMES_GROWTH));
    If newlist = Nil Then
    Begin
      Hermes_ConverterInstance := 0;
      Exit;
    End;
    { Copy converter pointers }
    For i := 0 To lastConverter - 1 Do
      newlist[i] := ConverterList[i];
    free(ConverterList);
    { Assign new list to old one }
    ConverterList := newlist;
    Inc(lastConverter, HERMES_GROWTH);
{    DEBUG_PRINT("Growing dynamic array, new size %d\n",lastConverter)}
  End;

  { Create a HermesConverter }
  newinstance := malloc(sizeof(THermesConverter));
  If newinstance = Nil Then
  Begin
    Hermes_ConverterInstance := 0;
    Exit;
  End;

  { Zero it out }
  newinstance^.loopnormal := Nil;
  newinstance^.loopstretch := Nil;
  newinstance^.normal := Nil;
  newinstance^.stretch := Nil;
  newinstance^.dither := Nil;
  newinstance^.ditherstretch := Nil;
  newinstance^.flags := flags;
  FillChar(newinstance^.source, 0, SizeOf(THermesFormat));
  FillChar(newinstance^.dest, 0, SizeOf(THermesFormat));

  ConverterList[ConvertCurrenthandle] := newinstance;

  Inc(CONVERTrefcount);
  Inc(ConvertCurrenthandle);

  Hermes_ConverterInstance := ConvertCurrenthandle - 1;
End;

Procedure Hermes_ConverterReturn(handle : THermesHandle);

Begin
  If (handle < 0) Or (handle >= lastConverter) Then
    Exit;

  { Adjust reference count }
  Dec(CONVERTrefcount);

  If ConverterList[handle] <> Nil Then
  Begin
    free(ConverterList[handle]);
    ConverterList[handle] := Nil;
  End;

  { No more references, deinitialise }
  If CONVERTrefcount = 0 Then
  Begin
    If ConverterList <> Nil Then
    Begin
      free(ConverterList);
      ConverterList := Nil;
    End;
    ConvertCurrenthandle := 0;
    lastConverter := 0;
  End;
End;

Function Hermes_ConverterRequest(handle : THermesHandle;
                                 source, dest : PHermesFormat) : Boolean;

Var
  searchlist : Integer;
  i : Integer;
  found : Boolean;
  cnv : PHermesConverter;

Begin
{  DebugMSG('Hermes_ConverterRequest(' + C2Str(source^.bits)
   + ',' + C2Str(source^.r) + ',' + C2Str(source^.g) + ',' +
   C2Str(source^.b) + ';' + C2Str(dest^.bits)
   + ',' + C2Str(dest^.r) + ',' + C2Str(dest^.g) + ',' +
   C2Str(dest^.b) + ')');}

  Hermes_ConverterRequest := False;
  searchlist := 0;
  i := 0;
  found := False;
  { Check array ranges }
  If (handle < 0) Or (handle >= lastConverter) Then
    Exit;
  If ConverterList[handle] = Nil Then
    Exit;
  cnv := ConverterList[handle];

  { Cache repeated requests of the same conversion }
  If Hermes_FormatEquals(source, @ConverterList[handle]^.source) And
     Hermes_FormatEquals(dest, @ConverterList[handle]^.dest) Then
  Begin
    Hermes_ConverterRequest := True;
    Exit;
  End;

  { Clear the generic converter flag }
  cnv^.flags := cnv^.flags And (Not HERMES_CONVERT_GENERIC);

  { If the source and destination are equal, use copy routines }
  If Hermes_FormatEquals(source, dest) Then
  Begin
{    DebugMSG('format equals!');}
    If ((source^.bits And 7) <> 0) Or (source^.bits > 32) Or
       (source^.bits = 0) Then
      Exit;
    i := (source^.bits Shr 3) - 1;
    If equalConverters[i] = Nil Then
      Exit;
    Hermes_FormatCopy(source, @cnv^.source);
    Hermes_FormatCopy(dest, @cnv^.dest);
    cnv^.loopnormal := equalConverters[i]^.loopnormal;
    cnv^.loopstretch := equalConverters[i]^.loopstretch;
    cnv^.normal := equalConverters[i]^.normal;
    cnv^.stretch := equalConverters[i]^.stretch;
    Hermes_ConverterRequest := True;
    Exit;
  End;

  { Start looking for specialised converters }
  searchlist := $ff;
  Case source^.bits Of
    32 : If (source^.r = $ff0000) And (source^.g = $ff00) And (source^.b = $ff) Then
           searchlist := 0
         Else
           If (source^.r = ($ff Shl 20)) And
              (source^.g = ($ff Shl 10)) And
              (source^.b = $ff) Then
             searchlist := 3;
    24 : If (source^.r = $ff0000) And (source^.g = $ff00) And (source^.b = $ff) Then
           searchlist := 1;
    16 : If (source^.r = $f800) And (source^.g = $7e0) And (source^.b = $1f) Then
           searchlist := 2;
     8 : If source^.indexed Then
           searchlist := 4;
  End;

  { We can use a quicker loop for 8 bit }
  If searchlist <> $ff Then
    If source^.bits = 8 Then
    Begin
      For i := 0 To numConverters[searchlist] - 1 Do
        If standardConverters[searchlist][i] <> Nil Then
          If dest^.bits = standardConverters[searchlist][i]^.dest.bits Then
          Begin
            Hermes_FormatCopy(source, @cnv^.source);
            Hermes_FormatCopy(dest, @cnv^.dest);

            cnv^.loopnormal := standardConverters[searchlist][i]^.loopnormal;
            cnv^.loopstretch := standardConverters[searchlist][i]^.loopstretch;

            cnv^.normal := standardConverters[searchlist][i]^.normal;
            cnv^.stretch := standardConverters[searchlist][i]^.stretch;

            cnv^.dither := standardConverters[searchlist][i]^.dither;
            cnv^.ditherstretch := standardConverters[searchlist][i]^.ditherstretch;

            Hermes_ConverterRequest := True;
            Exit;
          End
;
    End
    Else
      For i := 0 To numConverters[searchlist] - 1 Do
        If standardConverters[searchlist][i] <> Nil Then
          If Hermes_FormatEquals(@standardConverters[searchlist][i]^.source, source) And
             Hermes_FormatEquals(@standardConverters[searchlist][i]^.dest, dest) Then
          Begin
            Hermes_FormatCopy(source, @cnv^.source);
            Hermes_FormatCopy(dest, @cnv^.dest);

            cnv^.loopnormal := standardConverters[searchlist][i]^.loopnormal;
            cnv^.loopstretch := standardConverters[searchlist][i]^.loopstretch;

            cnv^.normal := standardConverters[searchlist][i]^.normal;
            cnv^.stretch := standardConverters[searchlist][i]^.stretch;

            cnv^.dither := standardConverters[searchlist][i]^.dither;
            cnv^.ditherstretch := standardConverters[searchlist][i]^.ditherstretch;

            Hermes_ConverterRequest := True;
            Exit;
          End;


  { Otherwise find a generic converter }
{  DebugMSG('looking for a generic converter!');}


  cnv^.loopnormal := Nil;
  cnv^.loopstretch := Nil;
  cnv^.dither := Nil;
  cnv^.ditherstretch := Nil;
  cnv^.flags := cnv^.flags Or HERMES_CONVERT_GENERIC;


  { Generic routines implement whole converters not scanline converters,
    assign placeholders }

  cnv^.normal := @NotApplicable;
  cnv^.stretch := @NotApplicable;

  found := False;

{
        Converting rules:

        C -> C
        C -> A

        A -> O, A -> A
        A -> C

        O -> O , A, C are the same
}

  If source^.has_colorkey And dest^.has_colorkey Then { Ck -> Ck }
    Case source^.bits Of
      32 : Case dest^.bits Of
        32 : Begin
          cnv^.loopnormal := @ConvertP_Generic32_C_Generic32_C;
          cnv^.loopstretch := @ConvertP_Generic32_C_Generic32_C_S;
          found := True;
        End;
        24 : Begin
          cnv^.loopnormal := @ConvertP_Generic32_C_Generic24_C;
          found := True;
        End;
        16 : Begin
          cnv^.loopnormal := @ConvertP_Generic32_C_Generic16_C;
          cnv^.loopstretch := @ConvertP_Generic32_C_Generic16_C_S;
          found := True;
        End;
        8 : Begin
          cnv^.loopnormal := @ConvertP_Generic32_C_Generic8_C;
          found := True;
        End;
      End;
      24 : Case dest^.bits Of
        32 : Begin
          cnv^.loopnormal := @ConvertP_Generic24_C_Generic32_C;
          found := True;
        End;
        24 : Begin
          cnv^.loopnormal := @ConvertP_Generic24_C_Generic24_C;
          found := True;
        End;
        16 : Begin
          cnv^.loopnormal := @ConvertP_Generic24_C_Generic16_C;
          found := True;
        End;
        8 : Begin
          cnv^.loopnormal := @ConvertP_Generic24_C_Generic8_C;
          found := True;
        End;
      End;
      16 : Case dest^.bits Of
        32 : Begin
          cnv^.loopnormal := @ConvertP_Generic16_C_Generic32_C;
          found := True;
        End;
        24 : Begin
          cnv^.loopnormal := @ConvertP_Generic16_C_Generic24_C;
          found := True;
        End;
        16 : Begin
          cnv^.loopnormal := @ConvertP_Generic16_C_Generic16_C;
          found := True;
        End;
        8 : Begin
          cnv^.loopnormal := @ConvertP_Generic16_C_Generic8_C;
          found := True;
        End;
      End;
    End
  Else
    If source^.has_colorkey And (dest^.a <> 0) Then { Ck -> A }
      Case source^.bits Of
        32 : Case dest^.bits Of
          32 : Begin
            cnv^.loopnormal := @ConvertP_Generic32_C_Generic32_A;
            cnv^.loopstretch := @ConvertP_Generic32_C_Generic32_A_S;
            found := True;
          End;
          24 : Begin
            cnv^.loopnormal := @ConvertP_Generic32_C_Generic24_A;
            found := True;
          End;
          16 : Begin
            cnv^.loopnormal := @ConvertP_Generic32_C_Generic16_A;
            cnv^.loopstretch := @ConvertP_Generic32_C_Generic16_A_S;
            found := True;
          End;
          8 : Begin
            cnv^.loopnormal := @ConvertP_Generic32_C_Generic8_A;
            found := True;
          End;
        End;
        24 : Case dest^.bits Of
          32 : Begin
            cnv^.loopnormal := @ConvertP_Generic24_C_Generic32_A;
            found := True;
          End;
          24 : Begin
            cnv^.loopnormal := @ConvertP_Generic24_C_Generic24_A;
            found := True;
          End;
          16 : Begin
            cnv^.loopnormal := @ConvertP_Generic24_C_Generic16_A;
            found := True;
          End;
          8 : Begin
            cnv^.loopnormal := @ConvertP_Generic24_C_Generic8_A;
            found := True;
          End;
        End;
        16 : Case dest^.bits Of
          32 : Begin
            cnv^.loopnormal := @ConvertP_Generic16_C_Generic32_A;
            found := True;
          End;
          24 : Begin
            cnv^.loopnormal := @ConvertP_Generic16_C_Generic24_A;
            found := True;
          End;
          16 : Begin
            cnv^.loopnormal := @ConvertP_Generic16_C_Generic16_A;
            found := True;
          End;
          8 : Begin
            cnv^.loopnormal := @ConvertP_Generic16_C_Generic8_A;
            found := True;
          End;
        End;
      End
    Else
      If (source^.a <> 0) And dest^.has_colorkey Then { A -> Ck }
        Case source^.bits Of
          32 : Case dest^.bits Of
            32 : Begin
              cnv^.loopnormal := @ConvertP_Generic32_A_Generic32_C;
              cnv^.loopstretch := @ConvertP_Generic32_A_Generic32_C_S;
              found := True;
            End;
            24 : Begin
              cnv^.loopnormal := @ConvertP_Generic32_A_Generic24_C;
              found := True;
            End;
            16 : Begin
              cnv^.loopnormal := @ConvertP_Generic32_A_Generic16_C;
              cnv^.loopnormal := @ConvertP_Generic32_A_Generic16_C_S;
              found := True;
            End;
            8 : Begin
              cnv^.loopnormal := @ConvertP_Generic32_A_Generic8_C;
              found := True;
            End;
          End;
          24 : Case dest^.bits Of
            32 : Begin
              cnv^.loopnormal := @ConvertP_Generic24_A_Generic32_C;
              found := True;
            End;
            24 : Begin
              cnv^.loopnormal := @ConvertP_Generic24_A_Generic24_C;
              found := True;
            End;
            16 : Begin
              cnv^.loopnormal := @ConvertP_Generic24_A_Generic16_C;
              found := True;
            End;
            8 : Begin
              cnv^.loopnormal := @ConvertP_Generic24_A_Generic8_C;
              found := True;
            End;
          End;
          16 : Case dest^.bits Of
            32 : Begin
              cnv^.loopnormal := @ConvertP_Generic16_A_Generic32_C;
              found := True;
            End;
            24 : Begin
              cnv^.loopnormal := @ConvertP_Generic16_A_Generic24_C;
              found := True;
            End;
            16 : Begin
              cnv^.loopnormal := @ConvertP_Generic16_A_Generic16_C;
              found := True;
            End;
            8 : Begin
              cnv^.loopnormal := @ConvertP_Generic16_A_Generic8_C;
              found := True;
            End;
          End;
        End
      Else
        If (source^.a <> 0) And (dest^.a <> 0) Then { A -> A }
          Case source^.bits Of
            32 : Case dest^.bits Of
              32 : Begin
                cnv^.loopnormal := @ConvertP_Generic32_A_Generic32_A;
                cnv^.loopstretch := @ConvertP_Generic32_A_Generic32_A_S;
                found := True;
              End;
              24 : Begin
                cnv^.loopnormal := @ConvertP_Generic32_A_Generic24_A;
                found := True;
              End;
              16 : Begin
                cnv^.loopnormal := @ConvertP_Generic32_A_Generic16_A;
                cnv^.loopstretch := @ConvertP_Generic32_A_Generic16_A_S;
                found := True;
              End;
              8 : Begin
                cnv^.loopnormal := @ConvertP_Generic32_A_Generic8_A;
                found := True;
              End;
            End;
            24 : Case dest^.bits Of
              32 : Begin
                cnv^.loopnormal := @ConvertP_Generic24_A_Generic32_A;
                found := True;
              End;
              24 : Begin
                cnv^.loopnormal := @ConvertP_Generic24_A_Generic24_A;
                found := True;
              End;
              16 : Begin
                cnv^.loopnormal := @ConvertP_Generic24_A_Generic16_A;
                found := True;
              End;
              8 : Begin
                cnv^.loopnormal := @ConvertP_Generic24_A_Generic8_A;
                found := True;
              End;
            End;
            16 : Case dest^.bits Of
              32 : Begin
                cnv^.loopnormal := @ConvertP_Generic16_A_Generic32_A;
                found := True;
              End;
              24 : Begin
                cnv^.loopnormal := @ConvertP_Generic16_A_Generic24_A;
                found := True;
              End;
              16 : Begin
                cnv^.loopnormal := @ConvertP_Generic16_A_Generic16_A;
                found := True;
              End;
              8 : Begin
                cnv^.loopnormal := @ConvertP_Generic16_A_Generic8_A;
                found := True;
              End;
            End;
          End
        Else { O->O, O->A, A->O, Ck->O, O->Ck }
          Case source^.bits Of
            32 : Case dest^.bits Of
              32 : Begin
                cnv^.loopnormal := @ConvertP_Generic32_Generic32;
                cnv^.loopstretch := @ConvertP_Generic32_Generic32_S;
                found := True;
              End;
              24 : Begin
                cnv^.loopnormal := @ConvertP_Generic32_Generic24;
                cnv^.loopstretch := @ConvertP_Generic32_Generic24_S;
                found := True;
              End;
              16 : Begin
                cnv^.loopnormal := @ConvertP_Generic32_Generic16;
                cnv^.loopstretch := @ConvertP_Generic32_Generic16_S;
                found := True;
              End;
              8 : Begin
                cnv^.loopnormal := @ConvertP_Generic32_Generic8;
                cnv^.loopstretch := @ConvertP_Generic32_Generic8_S;
                found := True;
              End;
            End;
            24 : Case dest^.bits Of
              32 : Begin
                cnv^.loopnormal := @ConvertP_Generic24_Generic32;
                cnv^.loopstretch := @ConvertP_Generic24_Generic32_S;
                found := True;
              End;
              24 : Begin
                cnv^.loopnormal := @ConvertP_Generic24_Generic24;
                cnv^.loopstretch := @ConvertP_Generic24_Generic24_S;
                found := True;
              End;
              16 : Begin
                cnv^.loopnormal := @ConvertP_Generic24_Generic16;
                cnv^.loopstretch := @ConvertP_Generic24_Generic16_S;
                found := True;
              End;
              8 : Begin
                cnv^.loopnormal := @ConvertP_Generic24_Generic8;
                cnv^.loopstretch := @ConvertP_Generic24_Generic8_S;
                found := True;
              End;
            End;
            16 : Case dest^.bits Of
              32 : Begin
                cnv^.loopnormal := @ConvertP_Generic16_Generic32;
                cnv^.loopstretch := @ConvertP_Generic16_Generic32_S;
                found := True;
              End;
              24 : Begin
                cnv^.loopnormal := @ConvertP_Generic16_Generic24;
                cnv^.loopstretch := @ConvertP_Generic16_Generic24_S;
                found := True;
              End;
              16 : Begin
                cnv^.loopnormal := @ConvertP_Generic16_Generic16;
                cnv^.loopstretch := @ConvertP_Generic16_Generic16_S;
                found := True;
              End;
              8 : Begin
                cnv^.loopnormal := @ConvertP_Generic16_Generic8;
                cnv^.loopstretch := @ConvertP_Generic16_Generic8_S;
                found := True;
              End;
            End;
          End;

  If found Then
  Begin
    Hermes_FormatCopy(source, @cnv^.source);
    Hermes_FormatCopy(dest, @cnv^.dest);

    Hermes_ConverterRequest := True;
    Exit;
  End;

  DebugMSG('no converter found!!!');
  { No converter found, fail }
  Hermes_ConverterRequest := False;
End;

Function Hermes_ConverterPalette(handle, sourcepal, destpal : THermesHandle) : Boolean;

Begin
{  DebugMSG('Hermes_ConverterPalette('+C2Str(sourcepal)+','+C2Str(destpal)+')');}
  Hermes_ConverterPalette := False;
  If (handle < 0) Or (handle >= lastConverter) Then
    Exit;
  If ConverterList[handle] = Nil Then
    Exit;

  { Fail silently if not indexed colour format }
  If Not ConverterList[handle]^.source.indexed Then
  Begin
    ConverterList[handle]^.lookup := Nil;
    Hermes_ConverterPalette := True;
    Exit;
  End;

  ConverterList[handle]^.lookup :=
    Hermes_PaletteGetTable(sourcepal, @ConverterList[handle]^.dest);

  If ConverterList[handle]^.lookup = Nil Then
    Exit;

  Hermes_ConverterPalette := True;
End;

Function Hermes_ConverterCopy(handle : THermesHandle; s_pixels : Pointer;
                              s_x, s_y, s_width, s_height, s_pitch : Integer;
                              d_pixels : Pointer; d_x, d_y, d_width,
                              d_height, d_pitch : Integer) : Boolean;

Var
  cnv : PHermesConverter;
  iface : THermesConverterInterface;

Begin
  Hermes_ConverterCopy := False;
  If (handle < 0) Or (handle >= lastConverter) Then
    Exit;
  cnv := ConverterList[handle];
  If cnv = Nil Then
    Exit;

  { Returns success if height or width is zero. This is debatable.. ! }
  If (s_width <= 0) Or (s_height <= 0) Or (d_width <= 0) Or (d_height <= 0) Then
  Begin
    Hermes_ConverterCopy := True;
    Exit;
  End;

  iface.s_pixels := s_pixels;
  iface.s_width := s_width;
  iface.s_height := s_height;
  iface.s_add := s_pitch - s_width * (cnv^.source.bits Shr 3);
  iface.s_pitch := s_pitch;

  iface.d_pixels := d_pixels;
  iface.d_width := d_width;
  iface.d_height := d_height;
  iface.d_add := d_pitch - d_width*(cnv^.dest.bits Shr 3);
  iface.d_pitch := d_pitch;

  Inc(iface.s_pixels, s_y * s_pitch + s_x * (cnv^.source.bits Shr 3));
  Inc(iface.d_pixels, d_y * d_pitch + d_x * (cnv^.dest.bits Shr 3));

  iface.s_has_colorkey := cnv^.source.has_colorkey;
  iface.d_has_colorkey := cnv^.dest.has_colorkey;
  iface.s_colorkey := cnv^.source.colorkey;
  iface.d_colorkey := cnv^.dest.colorkey;

  iface.lookup := cnv^.lookup;


  { For generic converters, do some extra setup (find shifts, etc.)
    TODO: Move that out of here and in the request routine ! }
  If (cnv^.flags And HERMES_CONVERT_GENERIC) <> 0 Then
  Begin
    Hermes_Calculate_Generic_Info(Hermes_Topbit(cnv^.source.r),
                                  Hermes_Topbit(cnv^.source.g),
                                  Hermes_Topbit(cnv^.source.b),
                                  Hermes_Topbit(cnv^.source.a),
                                  Hermes_Topbit(cnv^.dest.r),
                                  Hermes_Topbit(cnv^.dest.g),
                                  Hermes_Topbit(cnv^.dest.b),
                                  Hermes_Topbit(cnv^.dest.a),
                                  @iface.info);
    iface.mask_r := cnv^.dest.r;
    iface.mask_g := cnv^.dest.g;
    iface.mask_b := cnv^.dest.b;
    iface.mask_a := cnv^.dest.a;
  End;

  { Check for dithering. This should not be in here but in request as well }
  If (cnv^.flags And HERMES_CONVERT_DITHER) <> 0 Then
  Begin
    { If there is a ditherer, use it else fall back to normal }
    If cnv^.dither <> Nil Then
      cnv^.loopnormal := cnv^.dither;
  End;

  { Normal conversion }
  If (s_width = d_width) And (s_height = d_height) Then
  Begin
    If (cnv^.normal = Nil) Or (cnv^.loopnormal = Nil) Then
      Exit;
    { Optimization
    If (iface.s_add = 0) And (iface.d_add = 0) Then
    Begin
      iface.s_width := iface.s_width * s_height;
      iface.d_width := iface.d_width * d_height;
      iface.s_height := 1;
      iface.d_height := 1;
    End;}
    iface.func := cnv^.normal;
    cnv^.loopnormal(@iface);
    Hermes_ConverterCopy := True;
    Exit;
  End
  { Stretch conversion }
  Else
  Begin
    If (cnv^.stretch = Nil) Or (cnv^.loopstretch = Nil) Then
      Exit;
    iface.func := cnv^.stretch;
    cnv^.loopstretch(@iface);
  End;
  Hermes_ConverterCopy := True;
End;
