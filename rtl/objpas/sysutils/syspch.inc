{
    *********************************************************************
    Copyright (C) 1997, 1998 Gertjan Schouten

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

    System Utilities For Free Pascal
}

{  PChar functions  }

<<<<<<< HEAD
=======
type
   CharArray = array[0..0] of char;

>>>>>>> graemeg/fixes_2_2
{ Processor dependent part, shared withs strings unit }
{$ifdef FPC_USE_LIBC}
{$i cgenstr.inc}
{$endif FPC_USE_LIBC}
{$i strings.inc }

{ Read generic string functions that are not implemented for the processor }
{$i genstr.inc}

{ Processor independent part, shared with strings unit }
{$i stringsi.inc }

{  StrPas converts a PChar to a pascal string  }

function StrPas(Str: PChar): string;
begin
  Result:=Str;
end ;

{  StrAlloc allocates a buffer of Size + 4
   the size of the allocated buffer is stored at result - 4
   StrDispose should be used to destroy the buffer  }

function StrAlloc(Size: cardinal): PChar;
begin
  inc(size,sizeof(cardinal));
  getmem(result,size);
  cardinal(pointer(result)^):=size;
  inc(result,sizeof(cardinal));
end;


{ Allocates a new string using StrAlloc, you need StrDispose to dispose the
  string }

function strnew(p : pchar) : pchar;
var
  len : longint;
begin
  Result:=nil;
  if (p=nil) or (p^=#0) then
   exit;
  len:=strlen(p)+1;
  Result:=StrAlloc(Len);
  if Result<>nil then
   move(p^,Result^,len);
end;


{  StrPCopy copies the pascal string Source to Dest and returns Dest  }

function StrPCopy(Dest: PChar; Const Source: string): PChar;overload;
begin
  result := StrMove(Dest, PChar(Source), length(Source)+1);
end ;

{  StrPLCopy copies MaxLen or less characters from the pascal string
   Source to Dest and returns Dest  }

function StrPLCopy(Dest: PChar; Const Source: string; MaxLen: SizeUInt): PChar;overload;
var Count: SizeUInt;
begin
Result := Dest;
if Result <> Nil then
  begin
    Count := Length(Source);
    if Count > MaxLen then
      Count := MaxLen;
    StrMove(Result, PChar(Source), Count);
    Result[Count] := #0;  { terminate ! }
  end;
end;

{   StrDispose clears the memory allocated with StrAlloc   }

procedure StrDispose(Str: PChar);
begin
  if (Str <> Nil) then
   begin
     dec(Str,sizeof(cardinal));
     Freemem(str,cardinal(pointer(str)^));
   end;
end;

{  StrBufSize returns the amount of memory allocated for pchar Str allocated with StrAlloc  }

function StrBufSize(Str: PChar): Cardinal;
begin
  if Str <> Nil then
   result := cardinal(pointer(Str - SizeOf(cardinal))^)-sizeof(cardinal)
  else
   result := 0;
end ;

