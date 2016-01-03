{
    FPC Utility Functions for Native NT applications

    This file is part of the Free Pascal run time library.
    Copyright (c) 2009 by Sven Barth

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit NDKUtils;

{$mode objfpc}{$H+}

interface

uses
  NDK;

// Helpers for converting Pascal string types to NT's UNICODE_STRING
procedure ShortStrToNTStr(aStr: ShortString; var aNTStr: UNICODE_STRING);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure AnsiStrToNTStr(const aStr: RawByteString; var aNTStr: UNICODE_STRING);
=======
procedure AnsiStrToNTStr(const aStr: String; var aNTStr: UNICODE_STRING);
>>>>>>> graemeg/cpstrnew
=======
procedure AnsiStrToNTStr(const aStr: String; var aNTStr: UNICODE_STRING);
>>>>>>> graemeg/cpstrnew
=======
procedure AnsiStrToNTStr(const aStr: String; var aNTStr: UNICODE_STRING);
>>>>>>> graemeg/cpstrnew
=======
procedure AnsiStrToNTStr(const aStr: String; var aNTStr: UNICODE_STRING);
>>>>>>> origin/cpstrnew
procedure UnicodeStrToNtStr(const aStr: UnicodeString;
    var aNTStr: UNICODE_STRING);
procedure PCharToNTStr(aStr: PChar; aLen: Cardinal; var aNTStr: UNICODE_STRING);
procedure FreeNTStr(var aNTStr: UNICODE_STRING);

// Wraps NtDisplayString for use with Write(Ln)
procedure AssignDisplayString(var aFile: Text; aUtf8: Boolean);

implementation

uses
  SysUtils;

procedure ShortStrToNTStr(aStr: ShortString; var aNTStr: UNICODE_STRING);
var
  buf: Pointer;
  i: Integer;
begin
  aNTStr.Length := Length(aStr) * 2;
  aNTStr.buffer := GetMem(aNTStr.Length);
  buf := aNTStr.buffer;
  for i := 1 to Length(aStr) do begin
    PWord(buf)^ := Word(aStr[i]);
    buf := Pointer(PtrUInt(buf) + SizeOf(Word));
  end;
  aNTStr.MaximumLength := aNTStr.Length;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure AnsiStrToNTStr(const aStr: RawByteString; var aNTStr: UNICODE_STRING);
=======
procedure AnsiStrToNTStr(const aStr: String; var aNTStr: UNICODE_STRING);
>>>>>>> graemeg/cpstrnew
=======
procedure AnsiStrToNTStr(const aStr: String; var aNTStr: UNICODE_STRING);
>>>>>>> graemeg/cpstrnew
=======
procedure AnsiStrToNTStr(const aStr: String; var aNTStr: UNICODE_STRING);
>>>>>>> graemeg/cpstrnew
=======
procedure AnsiStrToNTStr(const aStr: String; var aNTStr: UNICODE_STRING);
>>>>>>> origin/cpstrnew
var
  buf: PWideChar;
  i: Integer;
begin
  aNTStr.Length := Length(aStr) * 2;
  aNTStr.Buffer := GetMem(aNTStr.Length);
  buf := aNTStr.buffer;
  for i := 1 to Length(aStr) do begin
    buf^ := WideChar(Word(aStr[i]));
    Inc(buf);
  end;
  aNTStr.MaximumLength := aNTStr.Length;
end;

procedure UnicodeStrToNtStr(const aStr: UnicodeString;
    var aNTStr: UNICODE_STRING);
var
  buf: PWideChar;
begin
  { TODO : check why this prints garbage }
  aNTStr.Length := Length(aStr) * 2;
  aNTStr.Buffer := GetMem(aNTStr.Length);
  if Length(aStr) > 0 then
    Move(aStr[1], aNTStr.Buffer^, aNTStr.Length);
  aNTStr.MaximumLength := aNTStr.Length;
end;

procedure PCharToNTStr(aStr: PChar; aLen: Cardinal; var aNTStr: UNICODE_STRING);
var
  i: Integer;
begin
  if (aLen = 0) and (aStr <> Nil) and (aStr^ <> #0) then
    aLen := StrLen(aStr);
  aNtStr.Length := aLen * SizeOf(WideChar);
  aNtStr.MaximumLength := aNtStr.Length;
  aNtStr.Buffer := GetMem(aNtStr.Length);
  for i := 0 to aLen do
    aNtStr.Buffer[i] := aStr[i];
end;

procedure FreeNTStr(var aNTStr: UNICODE_STRING);
begin
  if aNTStr.Buffer <> Nil then
    FreeMem(aNTStr.Buffer);
  FillChar(aNTStr, SizeOf(UNICODE_STRING), 0);
end;

function DisplayStringWriteFunc(var aFile: TTextRec ): LongInt;
var
  ntstr: TNtUnicodeString;
  len: SizeUInt;
begin
  Result := 0;
  with aFile do
    if (BufPos>0) then begin
      if Boolean(UserData[1]) then begin
        { TODO : check why UTF8 prints garbage }
        {len := Utf8ToUnicode(Nil, 0, PChar(BufPtr), BufPos);
        ntstr.Length := len * 2;
        ntstr.MaximumLength := ntstr.Length;
        ntstr.Buffer := GetMem(ntstr.Length);
        Utf8ToUnicode(ntstr.Buffer, len, PChar(BufPtr), BufPos);}
        PCharToNtStr(PChar(BufPtr), BufPos, ntstr);
      end else
        PCharToNtStr(PChar(BufPtr), BufPos, ntstr);
      NtDisplayString(@ntstr);
      // FreeNTStr uses FreeMem, so we don't need an If here
      FreeNtStr(ntstr);
      BufPos := 0;
    end;
end;

function DisplayStringCloseFunc(var aFile: TTextRec): LongInt;
begin
  Result := 0;
end;


function DisplayStringOpenFunc(var aFile: TTextRec ): LongInt;
begin
  Result := 0;
end;

procedure AssignDisplayString(var aFile: Text; aUtf8: Boolean);
begin
  FillChar(aFile, SizeOf(TextRec), 0);
{ only set things that are not zero }
  TextRec(aFile).Handle := UnusedHandle;
  TextRec(aFile).mode := fmOutput;
  TextRec(aFile).BufSize := TextRecBufSize;
  TextRec(aFile).Bufptr := @TextRec(aFile).Buffer;
  TextRec(aFile).OpenFunc := @DisplayStringOpenFunc;
  case DefaultTextLineBreakStyle of
    tlbsLF:
      TextRec(aFile).LineEnd := #10;
    tlbsCRLF:
      TextRec(aFile).LineEnd := #13#10;
    tlbsCR:
      TextRec(aFile).LineEnd := #13;
  end;
  TextRec(aFile).Closefunc := @DisplayStringCloseFunc;
  TextRec(aFile).InOutFunc := @DisplayStringWriteFunc;
  TextRec(aFile).FlushFunc := @DisplayStringWriteFunc;
  TextRec(aFile).UserData[1] := Ord(aUTF8);
end;

end.

