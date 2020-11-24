{
    This file is part of the Free Pascal compiler.
    Copyright (c) 2020 by the Free Pascal development team

    This unit contains platform-specific code for checking TTY output

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}
unit comptty;

{$i fpcdefs.inc}

interface

function IsATTY(var t : text) : Boolean;

const
(* This allows compile-time removal of the colouring functionality under not supported platforms *)
{$if defined(linux) or defined(MSWINDOWS) or defined(OS2) or defined(GO32V2) or defined(WATCOM)}
  TTYCheckSupported = true;
{$else defined(linux) or defined(MSWINDOWS) or defined(OS2) or defined(GO32V2) or defined(WATCOM)}
  TTYCheckSupported = false;
{$endif defined(linux) or defined(MSWINDOWS) or defined(OS2) or defined(GO32V2) or defined(WATCOM)}


implementation

{$ifdef linux}
  uses
   termio;
{$endif linux}
{$ifdef mswindows}
  uses
   windows;
{$endif mswindows}
{$ifdef os2}
  uses
   doscalls;
{$endif os2}
{$if defined(GO32V2) or defined(WATCOM)}
  uses
   dos;
{$endif defined(GO32V2) or defined(WATCOM)}

const
  CachedIsATTY : Boolean = false;
  IsATTYValue : Boolean = false;

{$ifdef linux}
function LinuxIsATTY(var t : text) : Boolean; inline;
begin
  LinuxIsATTY:=termio.IsATTY(t)=1;
end;
{$endif linux}

{$ifdef MSWINDOWS}
const
  ENABLE_VIRTUAL_TERMINAL_PROCESSING = $0004;

function WindowsIsATTY(var t : text) : Boolean; inline;
const
  dwMode: dword = 0;
begin
  WindowsIsATTY := false;
  if GetConsoleMode(TextRec(t).handle, dwMode) then
   begin
    dwMode := dwMode or ENABLE_VIRTUAL_TERMINAL_PROCESSING;
    if SetConsoleMode(TextRec(t).handle, dwMode) then
                                     WindowsIsATTY := true;
   end;
end;
{$endif MSWINDOWS}

{$IFDEF OS2}
function OS2IsATTY(var t : text) : Boolean; inline;
var
  HT, Attr: cardinal;
 {$IFDEF EMX}
  OK: boolean;
 {$ENDIF EMX}
const
  dhDevice = 1;
begin
 {$IFDEF EMX}
  if os_mode = osOS2 then
    begin
 {$ENDIF EMX}
      OS2IsATTY := (DosQueryHType (TextRec (T).Handle, HT, Attr) = 0)
                                                           and (HT = dhDevice);
 {$IFDEF EMX}
    end
  else
    begin
      OK := false;
{$ASMMODE INTEL}
      asm
        mov ebx, TextRec (T).Handle
        mov eax, 4400h
        call syscall
        jc @IsDevEnd
        test edx, 80h           { bit 7 is set if it is a device or a pipe }
        jz @IsDevEnd
        mov eax, 1A00h          { Check ANSI.SYS availability }
        int 2Fh
        inc al                  { If it was FFh, then OK }
        jnz @IsDevEnd
        mov OK, true
@IsDevEnd:
      end;
    OS2IsATTY := OK;
  end;
 {$ENDIF EMX}
end;
{$ENDIF OS2}

{$if defined(GO32V2) or defined(WATCOM)}
function DosIsATTY(var t : text) : Boolean; inline;
var
  Regs: Registers;
begin
  Regs.EBX := TextRec (T).Handle;
  Regs.EAX := $4400;
  MsDos (Regs);
  if (Regs.Flags and FCarry <> 0) or (Regs.EDX and $80 = 0) then
{ bit 7 is set if it is a device or a pipe }
    DosIsATTY := false
  else
    begin
      Regs.EAX := $1A00;             { Check ANSI.SYS availability }
      Intr ($2F, Regs);
      DosIsATTY := Regs.AL = $FF;    { If it was FFh, then OK }
    end;
end;
{$endif defined(GO32V2) or defined(WATCOM)}

function IsATTY(var t : text) : Boolean;
begin
  if not(CachedIsATTY) then
    begin
(* If none of the supported values is defined, false is returned by default. *)
{$ifdef linux}
      IsATTYValue:=LinuxIsATTY(t);
{$endif linux}
{$ifdef MSWINDOWS}
      IsATTYValue:=WindowsIsATTY(t);
{$endif MSWINDOWS}
{$ifdef OS2}
      IsATTYValue:=OS2IsATTY(t);
{$endif OS2}
{$if defined(GO32V2) or defined(WATCOM)}
      IsATTYValue:=DosIsATTY(t);
{$endif defined(GO32V2) or defined(WATCOM)}
      CachedIsATTY:=true;
    end;
  Result:=IsATTYValue;
end;

end.
