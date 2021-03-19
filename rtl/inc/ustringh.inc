{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2005 by Florian Klaempfl,
    member of the Free Pascal development team.

    This file implements support routines for UnicodeStrings with FPC

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


Procedure UniqueString (Var S : UnicodeString);{$ifdef SYSTEMINLINE}inline;{$endif}
Function Pos (Const Substr : UnicodeString; Const Source : UnicodeString; Offset: Sizeint = 1) : SizeInt;
Function Pos (c : Char; Const s : UnicodeString; Offset: Sizeint = 1) : SizeInt;
Function Pos (c : UnicodeChar; Const s : UnicodeString; Offset: Sizeint = 1) : SizeInt;
Function Pos (const c : RawByteString; Const s : UnicodeString; Offset: Sizeint = 1) : SizeInt;
Function Pos (const c : UnicodeString; Const s : RawByteString; Offset: Sizeint = 1) : SizeInt;
Function Pos (const c : ShortString; Const s : UnicodeString; Offset: Sizeint = 1) : SizeInt;

Function UpCase(const s : UnicodeString) : UnicodeString;
Function  UpCase(c:UnicodeChar):UnicodeChar;
Function LowerCase(const s : UnicodeString) : UnicodeString;
Function  LowerCase(c:UnicodeChar):UnicodeChar;

{$ifdef VER3_0}
Procedure Insert (Const Source : UnicodeString; Var S : UnicodeString; Index : SizeInt);
Procedure Delete (Var S : UnicodeString; Index,Size: SizeInt);
{$endif VER3_0}
Procedure {$ifdef FPC_HAS_CPSTRING}fpc_setstring_unicodestr_pwidechar{$else}SetString{$endif}(Out S : UnicodeString; Buf : PUnicodeChar; Len : SizeInt); {$ifdef FPC_HAS_CPSTRING} compilerproc; {$endif FPC_HAS_CPSTRING}
Procedure {$ifdef FPC_HAS_CPSTRING}fpc_setstring_unicodestr_pansichar{$else}SetString{$endif}(Out S : UnicodeString; Buf : PChar; Len : SizeInt); {$ifdef FPC_HAS_CPSTRING} compilerproc; {$endif FPC_HAS_CPSTRING}

function WideCharToString(S : PWideChar) : UnicodeString;
function StringToWideChar(const Src : RawByteString;Dest : PWideChar;DestSize : SizeInt) : PWideChar;
function WideCharLenToString(S : PWideChar;Len : SizeInt) : UnicodeString;
procedure WideCharLenToStrVar(Src : PWideChar;Len : SizeInt;out Dest : UnicodeString);
procedure WideCharLenToStrVar(Src : PWideChar;Len : SizeInt;out Dest : AnsiString);
procedure WideCharToStrVar(S : PWideChar;out Dest : UnicodeString);
procedure WideCharToStrVar(S : PWideChar;out Dest : AnsiString);

function UnicodeCharToString(S : PUnicodeChar) : UnicodeString;
function StringToUnicodeChar(const Src : RawByteString;Dest : PUnicodeChar;DestSize : SizeInt) : PUnicodeChar;
function UnicodeCharLenToString(S : PUnicodeChar;Len : SizeInt) : UnicodeString;
procedure UnicodeCharLenToStrVar(Src : PUnicodeChar;Len : SizeInt;out Dest : UnicodeString);
procedure UnicodeCharLenToStrVar(Src : PUnicodeChar;Len : SizeInt;out Dest : AnsiString);
procedure UnicodeCharToStrVar(S : PUnicodeChar;out Dest : AnsiString);

procedure DefaultUnicode2AnsiMove(source:punicodechar;var dest:RawByteString;cp : TSystemCodePage;len:SizeInt);
procedure DefaultAnsi2UnicodeMove(source:pchar;cp : TSystemCodePage;var dest:unicodestring;len:SizeInt);

Type
  { please only enable options below after creating a test program for them that
    passes on Windows and committing it, so it can be used to verify the
    functionality on other platforms }
  TCompareOption = ({coLingIgnoreCase, coLingIgnoreDiacritic, }coIgnoreCase{,
                    coIgnoreKanaType, coIgnoreNonSpace, coIgnoreSymbols, coIgnoreWidth,
                    coLingCasing, coDigitAsNumbers, coStringSort});
  TCompareOptions = set of TCompareOption;
  TStandardCodePageEnum = (
    scpAnsi,                 // system Ansi code page (GetACP on windows)
    scpConsoleInput,         // system console input code page (GetConsoleCP on windows)
    scpConsoleOutput,        // system console output code page (GetConsoleOutputCP on windows)
    scpFileSystemSingleByte  // file system code page used by single byte OS FileSystem APIs (GetACP on Windows),
  );

{$ifndef FPC_HAS_BUILTIN_WIDESTR_MANAGER}
  { hooks for internationalization
    please add new procedures at the end, it makes it easier to detect new procedures }
  TUnicodeStringManager = record
    Wide2AnsiMoveProc : procedure(source:pwidechar;var dest:RawByteString;cp : TSystemCodePage;len:SizeInt);
    Ansi2WideMoveProc : procedure(source:pchar;cp : TSystemCodePage;var dest:widestring;len:SizeInt);

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
    CompareWideStringProc : function(const s1, s2 : WideString; Options : TCompareOptions) : PtrInt;
    // CompareTextWideStringProc is CompareWideStringProc with coIgnoreCase in options.
//    CompareTextWideStringProc : function(const s1, s2 : WideString): PtrInt;
    { return value: number of code points in the string. Whenever an invalid
      code point is encountered, all characters part of this invalid code point
      are considered to form one "character" and the next character is
      considered to be the start of a new (possibly also invalid) code point }
    CharLengthPCharProc : function(const Str: PChar): PtrInt;
    { return value:
      -1 if incomplete or invalid code point
      0 if NULL character,
      > 0 if that's the length in bytes of the code point }
    CodePointLengthProc : function(const Str: PChar; MaxLookAead: PtrInt): Ptrint;

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

    { this is only different on windows }
    Unicode2AnsiMoveProc : procedure(source:punicodechar;var dest:RawByteString;cp : TSystemCodePage;len:SizeInt);
    Ansi2UnicodeMoveProc : procedure(source:pchar;cp : TSystemCodePage;var dest:unicodestring;len:SizeInt);
    UpperUnicodeStringProc : function(const S: UnicodeString): UnicodeString;
    LowerUnicodeStringProc : function(const S: UnicodeString): UnicodeString;
    CompareUnicodeStringProc : function(const s1, s2 : UnicodeString; Options : TCompareOptions) : PtrInt;
    // CompareTextUnicodeStringProc is CompareUnicodeStringProc with coIgnoreCase in options.
    /// CompareTextUnicodeStringProc : function(const s1, s2 : UnicodeString): PtrInt;

    { codepage retrieve function }
    GetStandardCodePageProc: function(const stdcp: TStandardCodePageEnum): TSystemCodePage;
  end;
{$endif FPC_HAS_BUILTIN_WIDESTR_MANAGER}

var
  widestringmanager : TUnicodeStringManager;

function UnicodeToUtf8(Dest: PChar; Source: PUnicodeChar; MaxBytes: SizeInt): SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function UnicodeToUtf8(Dest: PChar; MaxDestBytes: SizeUInt; Source: PUnicodeChar; SourceChars: SizeUInt): SizeUInt;
function Utf8ToUnicode(Dest: PUnicodeChar; Source: PChar; MaxChars: SizeInt): SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function Utf8ToUnicode(Dest: PUnicodeChar; MaxDestChars: SizeUInt; Source: PChar; SourceBytes: SizeUInt): SizeUInt;{$ifdef SYSTEMINLINE}inline;{$endif}
function Utf8ToUnicode(Dest: PUnicodeChar; MaxDestChars: SizeUInt; Source: PChar; SourceBytes: SizeUInt; IgnoreInvalid : Boolean): SizeUInt;
function UTF8Encode(const s : RawByteString) : RawByteString; inline;
function UTF8Encode(const s : UnicodeString) : RawByteString;
function UTF8Decode(const s : RawByteString): UnicodeString;
function UTF8ToString(const s : RawByteString): UnicodeString;inline;
function UTF8ToString(const S: ShortString): unicodestring;
function UTF8ToString(const S: PAnsiChar): unicodestring;
{ byte and ansichar are the same on the JVM, and "array of" and "pointer to"
  are as well }
{$ifndef CPUJVM}
function UTF8ToString(const S: array of AnsiChar): unicodestring;
function UTF8ToString(const S: array of Byte): unicodestring;
{$endif not CPUJVM}
function AnsiToUtf8(const s : RawByteString): RawByteString;{$ifdef SYSTEMINLINE}inline;{$endif}
function Utf8ToAnsi(const s : RawByteString) : RawByteString;{$ifdef SYSTEMINLINE}inline;{$endif}
{$ifdef FPC_HAS_FEATURE_DYNARRAYS}
function UnicodeStringToUCS4String(const s : UnicodeString) : UCS4String;
function UCS4StringToUnicodeString(const s : UCS4String) : UnicodeString;
function WideStringToUCS4String(const s : WideString) : UCS4String;
function UCS4StringToWideString(const s : UCS4String) : WideString;
{$endif FPC_HAS_FEATURE_DYNARRAYS}

Procedure GetWideStringManager (Out Manager : TUnicodeStringManager);
Procedure SetWideStringManager (Const New : TUnicodeStringManager);
Procedure SetWideStringManager (Const New : TUnicodeStringManager; Out Old: TUnicodeStringManager);

Procedure GetUnicodeStringManager (Out Manager : TUnicodeStringManager);
Procedure SetUnicodeStringManager (Const New : TUnicodeStringManager);
Procedure SetUnicodeStringManager (Const New : TUnicodeStringManager; Out Old: TUnicodeStringManager);

function StringElementSize(const S : UnicodeString): Word; overload;
function StringRefCount(const S : UnicodeString): SizeInt; overload;
function StringCodePage(const S : UnicodeString): TSystemCodePage; overload;

Function ToSingleByteFileSystemEncodedFileName(const Str: UnicodeString): RawByteString;
Function ToSingleByteFileSystemEncodedFileName(const arr: array of widechar): RawByteString;
Function ToSingleByteFileSystemEncodedFileName(const Str: RawByteString): RawByteString;
