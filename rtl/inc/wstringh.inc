{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2005 by Florian Klaempfl,
    member of the Free Pascal development team.

    This file implements support routines for WideStrings/Unicode with FPC

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


Procedure UniqueString (Var S : WideString);external name 'FPC_WIDESTR_UNIQUE';
<<<<<<< HEAD
Function Pos (Const Substr : WideString; Const Source : WideString; Offset : SizeInt = 1) : SizeInt;
Function Pos (c : Char; Const s : WideString; Offset : SizeInt = 1) : SizeInt;
Function Pos (c : WideChar; Const s : WideString; Offset : SizeInt = 1) : SizeInt;
Function Pos (c : WideChar; Const s : RawByteString; Offset : SizeInt = 1) : SizeInt;
Function Pos (const c : RawByteString; Const s : WideString; Offset : SizeInt = 1) : SizeInt;
Function Pos (const c : WideString; Const s : RawByteString; Offset : SizeInt = 1) : SizeInt;
Function Pos (const c : ShortString; Const s : WideString; Offset : SizeInt = 1) : SizeInt;
=======
Function Pos (Const Substr : WideString; Const Source : WideString) : SizeInt;
Function Pos (c : Char; Const s : WideString) : SizeInt;
Function Pos (c : WideChar; Const s : WideString) : SizeInt;
Function Pos (c : WideChar; Const s : RawByteString) : SizeInt;
Function Pos (c : RawByteString; Const s : WideString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
Function Pos (c : WideString; Const s : RawByteString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
Function Pos (c : ShortString; Const s : WideString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew

Function UpCase(const s : WideString) : WideString;

Procedure Insert (Const Source : WideString; Var S : WideString; Index : SizeInt);
Procedure Delete (Var S : WideString; Index,Size: SizeInt);
<<<<<<< HEAD
Procedure {$ifdef FPC_HAS_CPSTRING}fpc_setstring_widestr_pwidechar{$else}SetString{$endif}(Out S : WideString; Buf : PWideChar; Len : SizeInt); {$ifdef FPC_HAS_CPSTRING} compilerproc; {$endif FPC_HAS_CPSTRING}
Procedure {$ifdef FPC_HAS_CPSTRING}fpc_setstring_widestr_pansichar{$else}SetString{$endif}(Out S : WideString; Buf : PChar; Len : SizeInt); {$ifdef FPC_HAS_CPSTRING} compilerproc; {$endif FPC_HAS_CPSTRING}

<<<<<<< HEAD
<<<<<<< HEAD
=======
procedure DefaultWide2AnsiMove(source:pwidechar;var dest:ansistring;cp : TSystemCodePage;len:SizeInt);
>>>>>>> graemeg/cpstrnew
=======
procedure DefaultWide2AnsiMove(source:pwidechar;var dest:ansistring;cp : TSystemCodePage;len:SizeInt);
>>>>>>> graemeg/cpstrnew
=======
Procedure SetString (Out S : WideString; Buf : PWideChar; Len : SizeInt);
Procedure SetString (Out S : WideString; Buf : PChar; Len : SizeInt);

procedure DefaultWide2AnsiMove(source:pwidechar;var dest:ansistring;cp : TSystemCodePage;len:SizeInt);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure DefaultAnsi2WideMove(source:pchar;cp : TSystemCodePage;var dest:widestring;len:SizeInt);

type
  TWideStringManager = TUnicodeStringManager;

<<<<<<< HEAD
function UTF8Encode(const s : WideString) : RawByteString;
=======
Type
  { hooks for internationalization
    please add new procedures at the end, it makes it easier to detect new procedures }
  TWideStringManager = record
    Wide2AnsiMoveProc : procedure(source:pwidechar;var dest:ansistring;len:SizeInt);
    Ansi2WideMoveProc : procedure(source:pchar;var dest:widestring;len:SizeInt);

//    UpperUTF8 : procedure(p:PUTF8String);

    UpperWideStringProc : function(const S: WideString): WideString;
//    UpperUCS4 : procedure(p:PUCS4Char);
//    LowerUTF8 : procedure(p:PUTF8String);
    LowerWideStringProc : function(const S: WideString): WideString;
//    LowerUCS4 : procedure(p:PUCS4Char);
{
    CompUTF8 : function(p1,p2:PUTF8String) : shortint;
    CompUCS2 : function(p1,p2:PUCS2Char) : shortint;
    CompUCS4 : function(p1,p2:PUC42Char) : shortint;
}
    CompareWideStringProc : function(const s1, s2 : WideString) : PtrInt;
    CompareTextWideStringProc : function(const s1, s2 : WideString): PtrInt;
    CharLengthPCharProc : function(const Str: PChar): PtrInt;

    UpperAnsiStringProc : function(const s : ansistring) : ansistring;
    LowerAnsiStringProc : function(const s : ansistring) : ansistring;
    CompareStrAnsiStringProc : function(const S1, S2: ansistring): PtrInt;
    CompareTextAnsiStringProc : function(const S1, S2: ansistring): PtrInt;
    StrCompAnsiStringProc : function(S1, S2: PChar): PtrInt;
    StrICompAnsiStringProc : function(S1, S2: PChar): PtrInt;
    StrLCompAnsiStringProc : function(S1, S2: PChar; MaxLen: PtrUInt): PtrInt;
    StrLICompAnsiStringProc : function(S1, S2: PChar; MaxLen: PtrUInt): PtrInt;
    StrLowerAnsiStringProc : function(Str: PChar): PChar;
    StrUpperAnsiStringProc : function(Str: PChar): PChar;
    ThreadInitProc : procedure;
    ThreadFiniProc : procedure;
  end;


function UnicodeToUtf8(Dest: PChar; Source: PWideChar; MaxBytes: SizeInt): SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function UnicodeToUtf8(Dest: PChar; MaxDestBytes: SizeUInt; Source: PWideChar; SourceChars: SizeUInt): SizeUInt;
function Utf8ToUnicode(Dest: PWideChar; Source: PChar; MaxChars: SizeInt): SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function Utf8ToUnicode(Dest: PWideChar; MaxDestChars: SizeUInt; Source: PChar; SourceBytes: SizeUInt): SizeUInt;
function UTF8Encode(const s : WideString) : UTF8String;
function UTF8Decode(const s : UTF8String): WideString;
function AnsiToUtf8(const s : ansistring): UTF8String;{$ifdef SYSTEMINLINE}inline;{$endif}
function Utf8ToAnsi(const s : UTF8String) : ansistring;{$ifdef SYSTEMINLINE}inline;{$endif}
function WideStringToUCS4String(const s : WideString) : UCS4String;
function UCS4StringToWideString(const s : UCS4String) : WideString;
>>>>>>> graemeg/fixes_2_2

{$ifdef MSWINDOWS}
const
  winwidestringalloc : boolean = true deprecated;
{$endif MSWINDOWS}

