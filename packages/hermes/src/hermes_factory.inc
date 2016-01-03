{
    Free Pascal port of the Hermes C library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C version by Christian Nentwich (c.nentwich@cs.ucl.ac.uk)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version
    with the following modification:

    As a special exception, the copyright holders of this library give you
    permission to link this library with independent modules to produce an
    executable, regardless of the license terms of these independent modules,and
    to copy and distribute the resulting executable under terms of your choice,
    provided that you also meet, for each linked independent module, the terms
    and conditions of the license of that module. An independent module is a
    module which is not derived from or based on this library. If you modify
    this library, you may extend this exception to your version of the library,
    but you are not obligated to do so. If you do not wish to do so, delete this
    exception statement from your version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

var
  Processor: Integer;

procedure Hermes_Factory_Init;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF I386_ASSEMBLER}
var
  res: Integer = 0;
{$ENDIF I386_ASSEMBLER}
begin
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
var
  res: Integer;
begin
  res := 0;
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
  Processor := PROC_GENERIC;

  {$IFDEF I386_ASSEMBLER}
    Processor := Processor or PROC_X86_PENTIUM;{There are no others at the moment}
    res := Hermes_X86_CPU;
    if (res and $800000) <> 0 then
    begin
//      Writeln('mmx disabled for debugging');
      Processor := Processor or PROC_MMX_PENTIUM;
{      Writeln('mmx!');}
    end;
  {$ENDIF I386_ASSEMBLER}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$IFDEF X86_64_ASSEMBLER}
    Processor := Processor or PROC_X86_64;
  {$ENDIF X86_64_ASSEMBLER}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

function Hermes_Factory_getClearer(bits: Uint32): PHermesClearer;
begin
  { Try different processors in order of priority..
    Note that for this to work, an MMX processor has to have both MMX and
    X86 flags }
  New(Result);
  Result^.bits := bits;

  {$IFDEF I386_ASSEMBLER}
    if (Processor and PROC_MMX_PENTIUM) <> 0 then
      case bits of
        32: begin
          Result^.func := @ClearMMX_32;
          exit;
        end;
        24: ;
        16: begin
          Result^.func := @ClearMMX_16;
          exit;
        end;
        8: begin
          Result^.func := @ClearMMX_8;
          exit;
        end;
      end;

    if (Processor and PROC_X86_PENTIUM) <> 0 then
      case bits of
        32: begin
          Result^.func := @ClearX86_32;
          exit;
        end;
        24: ;
        16: begin
          Result^.func := @ClearX86_16;
          exit;
        end;
        8: begin
          Result^.func := @ClearX86_8;
          exit;
        end;
      end;
  {$ENDIF I386_ASSEMBLER}

  case bits of
    32: begin
      Result^.func := @ClearP_32;
      exit;
    end;
    24: begin
      Result^.func := @ClearP_24;
      exit;
    end;
    16: begin
      Result^.func := @ClearP_16;
      exit;
    end;
    8: begin
      Result^.func := @ClearP_8;
      exit;
    end;
    else
    begin
      Dispose(Result);
      Result := nil;
    end;
  end;
end;

function Hermes_Factory_getConverter(source, dest: PHermesFormat): PHermesConverter;
var
  i: Integer;
  found: Boolean;
begin
  found := False;

  New(Result);

  { Set all conversion routines to nil }
  Result^.loopnormal := nil;
  Result^.loopstretch := nil;
  Result^.normal := nil;
  Result^.stretch := nil;
  Result^.dither := nil;
  Result^.ditherstretch := nil;
  Result^.flags := 0;

  if source^.indexed then
    { for 8 bit indexed, just look at the destination bit depth and check
      if the converter's processor is a subset of our processor }
    for i := 0 to Factory_NumConverters - 1 do
      if (Factory_Converters[i].d_bits = dest^.bits) and
        (Factory_Converters[i].s_idx and
        ((processor and Factory_Converters[i].processor) <> 0)) then
      begin
        { if any routines are unassigned, assign them now }

        if Result^.loopnormal = nil then
        begin
          Result^.loopnormal := Factory_Converters[i].loopnormal;
          found := True;
        end;

        if Result^.normal = nil then
        begin
          Result^.normal := Factory_Converters[i].normal;
          found := True;
        end;

        if Result^.loopstretch = nil then
        begin
          Result^.loopstretch := Factory_Converters[i].loopstretch;
          found := True;
        end;

        if Result^.stretch = nil then
        begin
          Result^.stretch := Factory_Converters[i].stretch;
          found := True;
        end;
      end else
  else
    { Otherwise we need to compare everything, including bitmasks }
    for i := 0 to Factory_NumConverters - 1 do
      if (Factory_Converters[i].d_bits = dest^.bits) and
         (Factory_Converters[i].d_r = dest^.r) and
         (Factory_Converters[i].d_g = dest^.g) and
         (Factory_Converters[i].d_b = dest^.b) and
         (Factory_Converters[i].d_a = dest^.a) and
         (Factory_Converters[i].d_idx = dest^.indexed) and
         (Factory_Converters[i].s_bits = source^.bits) and
         (Factory_Converters[i].s_r = source^.r) and
         (Factory_Converters[i].s_g = source^.g) and
         (Factory_Converters[i].s_b = source^.b) and
         (Factory_Converters[i].s_a = source^.a) and
         (Factory_Converters[i].s_idx = source^.indexed) and
         ((processor and Factory_Converters[i].processor) <> 0) then
      begin
        { if any routines are unassigned, assign them now }

        if (Result^.loopnormal = nil) and
           (Factory_Converters[i].loopnormal <> nil) then
        begin
          Result^.loopnormal := Factory_Converters[i].loopnormal;
          found := True;
        end;

        if (Result^.normal = nil) and
           (Factory_Converters[i].normal <> nil) then
        begin
          Result^.normal := Factory_Converters[i].normal;
          found := True;
        end;

        if (Result^.loopstretch = nil) and
           (Factory_Converters[i].loopstretch <> nil) then
        begin
          Result^.loopstretch := Factory_Converters[i].loopstretch;
          found := True;
        end;

        if (Result^.stretch = nil) and
           (Factory_Converters[i].stretch <> nil) then
        begin
          Result^.stretch := Factory_Converters[i].stretch;
          found := True;
        end;

        if (Result^.dither = nil) and
           (Factory_Converters[i].dither <> nil) then
        begin
          Result^.dither := Factory_Converters[i].dither;
          found := True;
        end;

        if (Result^.ditherstretch = nil) and
           (Factory_Converters[i].ditherstretch <> nil) then
        begin
          Result^.ditherstretch := Factory_Converters[i].ditherstretch;
          found := True;
        end;

        { In the rare event of having everything assigned, pull the emergency
          break. Otherwise we need to continue looking (might be stretching
          routines somewhere :)
          do I sound like a stewardess? }
        if (Result^.loopnormal <> nil) and (Result^.normal <> nil) and
           (Result^.loopstretch <> nil) and (Result^.stretch <> nil) and
           (Result^.dither <> nil) and (Result^.ditherstretch <> nil) then
          break;
      end;

  if found then
  begin
    Hermes_FormatCopy(source, @Result^.source);
    Hermes_FormatCopy(dest, @Result^.dest);
  end
  else
  begin
    Dispose(Result);
    Result := nil;
  end;
end;

function Hermes_Factory_getEqualConverter(bits: Integer): PHermesConverter;
var
  found: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF I386_ASSEMBLER}
  asm_found: Integer;
{$ENDIF I386_ASSEMBLER}
=======
  asm_found: Integer;
>>>>>>> graemeg/cpstrnew
=======
  asm_found: Integer;
>>>>>>> graemeg/cpstrnew
=======
  asm_found: Integer;
>>>>>>> graemeg/cpstrnew
=======
  asm_found: Integer;
>>>>>>> origin/cpstrnew
  c_found: Integer;
begin
  found := False;
  New(Result);

  { Set all conversion routines to null }
  Result^.loopnormal := nil;
  Result^.loopstretch := nil;
  Result^.normal := nil;
  Result^.stretch := nil;
  Result^.dither := nil;
  Result^.ditherstretch := nil;

{$IFDEF I386_ASSEMBLER}

  { Try MMX routines }
  if (Result^.loopnormal = nil) or (Result^.normal = nil) or
     (Result^.loopstretch = nil) or (Result^.stretch = nil) then
    if (processor and PROC_MMX_PENTIUM) <> 0 then
{      case bits of
      end};

  { Try X86 routines }
  if (Result^.loopnormal = nil) or (Result^.normal = nil) or
     (Result^.loopstretch = nil) or (Result^.stretch = nil) then
    if (processor and PROC_X86_PENTIUM) <> 0 then
    begin
      asm_found := 0;
      case bits of
        32: begin
          Result^.normal := @CopyX86p_4byte; asm_found := 1;
        end;
        24: ;
        16: begin
          Result^.normal := @CopyX86p_2byte; asm_found := 1;
        end;
         8: begin
          Result^.normal := @CopyX86p_1byte; asm_found := 1;
        end;
      end;

      if (asm_found and 1) <> 0 then
      begin
        Result^.loopnormal := @ConvertX86;
        found := True;
      end;
    end;

{$ENDIF I386_ASSEMBLER}


  if (Result^.loopnormal = nil) or (Result^.normal = nil) or
     (Result^.loopstretch = nil) or (Result^.stretch = nil) then
  begin
    c_found := 0;

    case bits of
      32: begin
        if Result^.normal = nil then
        begin
          Result^.normal := @CopyP_4byte; c_found := c_found or 1;
        end;
        if Result^.stretch = nil then
        begin
          Result^.stretch := @CopyP_4byte_S; c_found := c_found or 2;
        end;
      end;
      24: begin
        if Result^.normal = nil then
        begin
          Result^.normal := @CopyP_3byte; c_found := c_found or 1;
        end;
        if Result^.stretch = nil then
        begin
          Result^.stretch := @CopyP_3byte_S; c_found := c_found or 2;
        end;
      end;
      16: begin
        if Result^.normal = nil then
        begin
          Result^.normal := @CopyP_2byte; c_found := c_found or 1;
        end;
        if Result^.stretch = nil then
        begin
          Result^.stretch := @CopyP_2byte_S; c_found := c_found or 2;
        end;
      end;
       8: begin
        if Result^.normal = nil then
        begin
          Result^.normal := @CopyP_1byte; c_found := c_found or 1;
        end;
        if Result^.stretch = nil then
        begin
          Result^.stretch := @CopyP_1byte_S; c_found := c_found or 2;
        end;
      end;
    end;

    if (c_found and 1) <> 0 then
    begin
      Result^.loopnormal := @ConvertP; found := True;
    end;
    if (c_found and 2) <> 0 then
    begin
      Result^.loopstretch := @ConvertPStretch; found := True;
    end;
  end;

  if not found then
  begin
    Dispose(Result);
    Result := nil;
  end;
end;
