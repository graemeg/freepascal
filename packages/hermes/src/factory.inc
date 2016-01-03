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

Var
  Processor : Integer;

Procedure Hermes_Factory_Init;

Var
  res : Integer;

Begin
  res := 0;
  Processor := PROC_GENERIC;

  {$IFDEF I386_ASSEMBLER}
    Processor := Processor Or PROC_X86_PENTIUM;{There are no others at the moment}
    res := Hermes_X86_CPU;
    If (res And $800000) <> 0 Then
    Begin
//      Writeln('mmx disabled for debugging');
      Processor := Processor Or PROC_MMX_PENTIUM;
{      Writeln('mmx!');}
    End;
  {$ENDIF I386_ASSEMBLER}
End;

Function Hermes_Factory_getClearer(bits : int32) : PHermesClearer;

Var
  tmp : PHermesClearer;

Begin
  { Try different processors in order of priority..
    Note that for this to work, an MMX processor has to have both MMX and
    X86 flags }
  tmp := malloc(SizeOf(THermesClearer));
  If tmp = Nil Then
  Begin
    Hermes_Factory_getClearer := Nil;
    Exit;
  End;

  tmp^.bits := bits;
  Hermes_Factory_getClearer := tmp;

  {$IFDEF I386_ASSEMBLER}
    If (Processor And PROC_MMX_PENTIUM) <> 0 Then
      Case bits Of
        32 : Begin
          tmp^.func := @ClearMMX_32;
          Exit;
        End;
        24 : ;
        16 : Begin
          tmp^.func := @ClearMMX_16;
          Exit;
        End;
        8 : Begin
          tmp^.func := @ClearMMX_8;
          Exit;
        End;
      End;

    If (Processor And PROC_X86_PENTIUM) <> 0 Then
      Case bits Of
        32 : Begin
          tmp^.func := @ClearX86_32;
          Exit;
        End;
        24 : ;
        16 : Begin
          tmp^.func := @ClearX86_16;
          Exit;
        End;
        8 : Begin
          tmp^.func := @ClearX86_8;
          Exit;
        End;
      End;
  {$ENDIF I386_ASSEMBLER}

  Case bits Of
    32 : Begin
      tmp^.func := @ClearP_32;
      Exit;
    End;
    24 : Begin
      tmp^.func := @ClearP_24;
      Exit;
    End;
    16 : Begin
      tmp^.func := @ClearP_16;
      Exit;
    End;
    8 : Begin
      tmp^.func := @ClearP_8;
      Exit;
    End;
    Else
    Begin
      free(tmp);
      Hermes_Factory_getClearer := Nil;
    End;
  End;
End;

Function Hermes_Factory_getConverter(source, dest : PHermesFormat) : PHermesConverter;

Var
  tmp : PHermesConverter;
  i : Integer;
  found : Boolean;

Begin
  found := False;

  tmp := malloc(SizeOf(THermesConverter));
  If tmp = Nil Then
  Begin
    Hermes_Factory_getConverter := Nil;
    Exit;
  End;

  { Set all conversion routines to nil }
  tmp^.loopnormal := Nil;
  tmp^.loopstretch := Nil;
  tmp^.normal := Nil;
  tmp^.stretch := Nil;
  tmp^.dither := Nil;
  tmp^.ditherstretch := Nil;
  tmp^.flags := 0;

  If source^.indexed Then
    { For 8 bit indexed, just look at the destination bit depth and check
      if the converter's processor is a subset of our processor }
    For i := 0 To Factory_NumConverters - 1 Do
      If (Factory_Converters[i].d_bits = dest^.bits) And
        (Factory_Converters[i].s_idx And
        ((processor And Factory_Converters[i].processor) <> 0)) Then
      Begin
        { If any routines are unassigned, assign them now }

        If tmp^.loopnormal = Nil Then
        Begin
          tmp^.loopnormal := Factory_Converters[i].loopnormal;
          found := True;
        End;

        If tmp^.normal = Nil Then
        Begin
          tmp^.normal := Factory_Converters[i].normal;
          found := True;
        End;

        If tmp^.loopstretch = Nil Then
        Begin
          tmp^.loopstretch := Factory_Converters[i].loopstretch;
          found := True;
        End;

        If tmp^.stretch = Nil Then
        Begin
          tmp^.stretch := Factory_Converters[i].stretch;
          found := True;
        End;
      End Else
  Else
    { Otherwise we need to compare everything, including bitmasks }
    For i := 0 To Factory_NumConverters - 1 Do
      If (Factory_Converters[i].d_bits = dest^.bits) And
         (Factory_Converters[i].d_r = dest^.r) And
         (Factory_Converters[i].d_g = dest^.g) And
         (Factory_Converters[i].d_b = dest^.b) And
         (Factory_Converters[i].d_a = dest^.a) And
         (Factory_Converters[i].d_idx = dest^.indexed) And
         (Factory_Converters[i].s_bits = source^.bits) And
         (Factory_Converters[i].s_r = source^.r) And
         (Factory_Converters[i].s_g = source^.g) And
         (Factory_Converters[i].s_b = source^.b) And
         (Factory_Converters[i].s_a = source^.a) And
         (Factory_Converters[i].s_idx = source^.indexed) And
         ((processor And Factory_Converters[i].processor) <> 0) Then
      Begin
        { If any routines are unassigned, assign them now }

        If (tmp^.loopnormal = Nil) And
           (Factory_Converters[i].loopnormal <> Nil) Then
        Begin
          tmp^.loopnormal := Factory_Converters[i].loopnormal;
          found := True;
        End;

        If (tmp^.normal = Nil) And
           (Factory_Converters[i].normal <> Nil) Then
        Begin
          tmp^.normal := Factory_Converters[i].normal;
          found := True;
        End;

        If (tmp^.loopstretch = Nil) And
           (Factory_Converters[i].loopstretch <> Nil) Then
        Begin
          tmp^.loopstretch := Factory_Converters[i].loopstretch;
          found := True;
        End;

        If (tmp^.stretch = Nil) And
           (Factory_Converters[i].stretch <> Nil) Then
        Begin
          tmp^.stretch := Factory_Converters[i].stretch;
          found := True;
        End;

        If (tmp^.dither = Nil) And
           (Factory_Converters[i].dither <> Nil) Then
        Begin
          tmp^.dither := Factory_Converters[i].dither;
          found := True;
        End;

        If (tmp^.ditherstretch = Nil) And
           (Factory_Converters[i].ditherstretch <> Nil) Then
        Begin
          tmp^.ditherstretch := Factory_Converters[i].ditherstretch;
          found := True;
        End;

        { In the rare event of having everything assigned, pull the emergency
          break. Otherwise we need to continue looking (might be stretching
          routines somewhere :)
          Do I sound like a stewardess? }
        If (tmp^.loopnormal <> Nil) And (tmp^.normal <> Nil) And
           (tmp^.loopstretch <> Nil) And (tmp^.stretch <> Nil) And
           (tmp^.dither <> Nil) And (tmp^.ditherstretch <> Nil) Then
          Break;
      End;

  If found Then
  Begin
    Hermes_FormatCopy(source, @tmp^.source);
    Hermes_FormatCopy(dest, @tmp^.dest);

    Hermes_Factory_getConverter := tmp;
  End
  Else
  Begin
    free(tmp);
    Hermes_Factory_getConverter := Nil;
  End;
End;

Function Hermes_Factory_getEqualConverter(bits : Integer) : PHermesConverter;

Var
  found : Boolean;
  tmp : PHermesConverter;
  asm_found : Integer;
  c_found : Integer;

Begin
  found := False;
  tmp := malloc(SizeOf(THermesConverter));
  If tmp = Nil Then
  Begin
    Hermes_Factory_getEqualConverter := Nil;
    Exit;
  End;

  { Set all conversion routines to null }
  tmp^.loopnormal := Nil;
  tmp^.loopstretch := Nil;
  tmp^.normal := Nil;
  tmp^.stretch := Nil;
  tmp^.dither := Nil;
  tmp^.ditherstretch := Nil;

{$IFDEF I386_ASSEMBLER}

  { Try MMX routines }
  If (tmp^.loopnormal = Nil) Or (tmp^.normal = Nil) Or
     (tmp^.loopstretch = Nil) Or (tmp^.stretch = Nil) Then
    If (processor And PROC_MMX_PENTIUM) <> 0 Then
{      Case bits Of
      End};

  { Try X86 routines }
  If (tmp^.loopnormal = Nil) Or (tmp^.normal = Nil) Or
     (tmp^.loopstretch = Nil) Or (tmp^.stretch = Nil) Then
    If (processor And PROC_X86_PENTIUM) <> 0 Then
    Begin
      asm_found := 0;
      Case bits Of
        32 : Begin
          tmp^.normal := @CopyX86p_4byte; asm_found := 1;
        End;
        24 : ;
        16 : Begin
          tmp^.normal := @CopyX86p_2byte; asm_found := 1;
        End;
         8 : Begin
          tmp^.normal := @CopyX86p_1byte; asm_found := 1;
        End;
      End;

      If (asm_found And 1) <> 0 Then
      Begin
        tmp^.loopnormal := @ConvertX86;
        found := True;
      End;
    End;

{$ENDIF I386_ASSEMBLER}


  If (tmp^.loopnormal = Nil) Or (tmp^.normal = Nil) Or
     (tmp^.loopstretch = Nil) Or (tmp^.stretch = Nil) Then
  Begin
    c_found := 0;

    Case bits Of
      32 : Begin
        If tmp^.normal = Nil Then
        Begin
          tmp^.normal := @CopyP_4byte; c_found := c_found Or 1;
        End;
        If tmp^.stretch = Nil Then
        Begin
          tmp^.stretch := @CopyP_4byte_S; c_found := c_found Or 2;
        End;
      End;
      24 : Begin
        If tmp^.normal = Nil Then
        Begin
          tmp^.normal := @CopyP_3byte; c_found := c_found Or 1;
        End;
        If tmp^.stretch = Nil Then
        Begin
          tmp^.stretch := @CopyP_3byte_S; c_found := c_found Or 2;
        End;
      End;
      16 : Begin
        If tmp^.normal = Nil Then
        Begin
          tmp^.normal := @CopyP_2byte; c_found := c_found Or 1;
        End;
        If tmp^.stretch = Nil Then
        Begin
          tmp^.stretch := @CopyP_2byte_S; c_found := c_found Or 2;
        End;
      End;
       8 : Begin
        If tmp^.normal = Nil Then
        Begin
          tmp^.normal := @CopyP_1byte; c_found := c_found Or 1;
        End;
        If tmp^.stretch = Nil Then
        Begin
          tmp^.stretch := @CopyP_1byte_S; c_found := c_found Or 2;
        End;
      End;
    End;

    If (c_found And 1) <> 0 Then
    Begin
      tmp^.loopnormal := @ConvertP; found := True;
    End;
    If (c_found And 2) <> 0 Then
    Begin
      tmp^.loopstretch := @ConvertPStretch; found := True;
    End;
  End;

  If found Then
    Hermes_Factory_getEqualConverter := tmp
  Else
  Begin
    free(tmp);
    Hermes_Factory_getEqualConverter := Nil;
  End;
End;
