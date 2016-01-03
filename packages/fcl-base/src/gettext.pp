{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2001 by the Free Pascal development team

    Gettext interface to resourcestrings.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$MODE objfpc}
{$H+}

unit gettext;

interface

uses SysUtils, Classes;

const
  MOFileHeaderMagic = $950412de;

type
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
  TMOFileHeader = packed record
    magic: LongWord;             // MOFileHeaderMagic
    revision: LongWord;          // 0
    nstrings: LongWord;          // Number of string pairs
    OrigTabOffset: LongWord;     // Offset of original string offset table
    TransTabOffset: LongWord;    // Offset of translated string offset table
    HashTabSize: LongWord;       // Size of hashing table
    HashTabOffset: LongWord;     // Offset of first hashing table entry
  end;

  TMOStringInfo = packed record
    length: LongWord;
    offset: LongWord;
  end;

  TMOStringTable = array[0..(1 shl 30) div SizeOf(TMOStringInfo)] of TMOStringInfo;
  PMOStringTable = ^TMOStringTable;


  TLongWordArray = array[0..(1 shl 30) div SizeOf(LongWord)] of LongWord;
  PLongWordArray = ^TLongWordArray;

  TPCharArray = array[0..(1 shl 30) div SizeOf(PChar)] of PChar;
  PPCharArray = ^TPCharArray;

  TMOFile = class
  protected
    StringCount, HashTableSize: LongWord;
    HashTable: PLongWordArray;
    OrigTable, TranslTable: PMOStringTable;
    OrigStrings, TranslStrings: PPCharArray;
  public
    constructor Create(const AFilename: String);
    constructor Create(AStream: TStream);
    destructor Destroy; override;
    function Translate(AOrig: PChar; ALen: Integer; AHash: LongWord): String;
    function Translate(AOrig: String; AHash: LongWord): String;
    function Translate(AOrig: String): String;
  end;

  EMOFileError = class(Exception);


  procedure GetLanguageIDs(var Lang, FallbackLang: string);
  procedure TranslateResourceStrings(AFile: TMOFile);
  procedure TranslateUnitResourceStrings(const AUnitName:string; AFile: TMOFile);
  procedure TranslateResourceStrings(const AFilename: String);
  procedure TranslateUnitResourceStrings(const AUnitName:string; const AFilename: String);


implementation

{$ifdef Windows}
uses
   windows;
{$endif}


<<<<<<< HEAD
<<<<<<< HEAD
procedure Endianfixmotable(p:PMOStringTable;n:integer);
var I:integer;
begin
  if n>0 then
    for i:=0 to n-1 do
      begin 
        p^[i].length:=swapendian(p^[i].length);
        p^[i].offset:=swapendian(p^[i].offset);
      end;
end;

procedure Endianfixhashtable(p:PLongwordArray;n:integer);
var I:integer;
begin
  if n>0 then
    for i:=0 to n-1 do
      begin 
        p^[i]:=swapendian(p^[i]);
      end;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
constructor TMOFile.Create(AStream: TStream);
var
  header: TMOFileHeader;
  i: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
  endianswap : boolean;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  inherited Create;

  AStream.Read(header, Sizeof(header));

<<<<<<< HEAD
<<<<<<< HEAD
  if (header.magic <> MOFileHeaderMagic) and (swapendian(header.magic)<>MOFileHeaderMagic) then
    raise EMOFileError.Create('Invalid magic - not a MO file?');

  endianswap:=header.magic<>MOFileHeaderMagic;
  If EndianSwap then 
    begin
     with header do
       begin 
          revision	:=SwapEndian(revision);
          nstrings	:=SwapEndian(nstrings);
          OrigTabOffset :=SwapEndian(OrigTabOffset);
          TransTabOffset:=SwapEndian(TransTabOffset);
          HashTabSize   :=SwapEndian(HashTabSize);
          HashTabOffset :=SwapEndian(HashTabOffset);
       end;
    end;

=======
  if header.magic <> MOFileHeaderMagic then
    raise EMOFileError.Create('Invalid magic - not a MO file?');

>>>>>>> graemeg/fixes_2_2
=======
  if header.magic <> MOFileHeaderMagic then
    raise EMOFileError.Create('Invalid magic - not a MO file?');

>>>>>>> origin/fixes_2_2
  GetMem(OrigTable, header.nstrings * SizeOf(TMOStringInfo));
  GetMem(TranslTable, header.nstrings * SizeOf(TMOStringInfo));
  GetMem(OrigStrings, header.nstrings * SizeOf(PChar));
  GetMem(TranslStrings, header.nstrings * SizeOf(PChar));


  AStream.Position := header.OrigTabOffset;
  AStream.Read(OrigTable^, header.nstrings * SizeOf(TMOStringInfo));
<<<<<<< HEAD
<<<<<<< HEAD
  if EndianSwap then 
    EndianFixmotable(OrigTable,Header.NStrings);

  AStream.Position := header.TransTabOffset;
  AStream.Read(TranslTable^, header.nstrings * SizeOf(TMOStringInfo));
  if EndianSwap then 
    EndianFixmotable(TranslTable,Header.NStrings);
=======

  AStream.Position := header.TransTabOffset;
  AStream.Read(TranslTable^, header.nstrings * SizeOf(TMOStringInfo));
>>>>>>> graemeg/fixes_2_2
=======

  AStream.Position := header.TransTabOffset;
  AStream.Read(TranslTable^, header.nstrings * SizeOf(TMOStringInfo));
>>>>>>> origin/fixes_2_2

  StringCount := header.nstrings;

  // Read strings
  for i := 0 to StringCount - 1 do
  begin
    AStream.Position := OrigTable^[i].offset;
{    SetLength(s, OrigTable^[i].length);
    AStream.Read(s[1], OrigTable^[i].length);
    OrigStrings^[i] := StrNew(PChar(s));}
    GetMem(OrigStrings^[i], OrigTable^[i].length + 1);
    AStream.Read(OrigStrings^[i]^, OrigTable^[i].length);
    OrigStrings^[i][OrigTable^[i].length] := #0;
  end;

  for i := 0 to StringCount - 1 do
  begin
    AStream.Position := TranslTable^[i].offset;
{    SetLength(s, TranslTable^[i].length);
    AStream.Read(s[1], TranslTable^[i].length);
    TranslStrings^[i] := StrNew(PChar(s));}
    GetMem(TranslStrings^[i], TranslTable^[i].length+1);
    AStream.Read(TranslStrings^[i]^, TranslTable^[i].length);
    TranslStrings^[i][TranslTable^[i].length] := #0;
  end;

  // Read hashing table
  HashTableSize := header.HashTabSize;
  GetMem(HashTable, 4 * HashTableSize);
  AStream.Position := header.HashTabOffset;
  AStream.Read(HashTable^, 4 * HashTableSize);
<<<<<<< HEAD
<<<<<<< HEAD
  if EndianSwap then 
    EndianFixHashTable(hashtable,hashtablesize);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

constructor TMOFile.Create(const AFilename: String);
var
  f: TStream;
begin
  f := TFileStream.Create(AFilename, fmOpenRead);
  try
    Self.Create(f);
  finally
    f.Free;
  end;
end;

destructor TMOFile.Destroy;
var
  i: Integer;
begin
  for i := 0 to StringCount - 1 do
  begin
    Dispose(OrigStrings^[i]);
    Dispose(TranslStrings^[i]);
  end;
  Dispose(OrigTable);
  Dispose(TranslTable);
  Dispose(OrigStrings);
  Dispose(TranslStrings);
  Dispose(HashTable);
  inherited Destroy;
end;

function TMOFile.Translate(AOrig: PChar; ALen: Integer; AHash: LongWord): String;
var
  idx, incr, nstr: LongWord;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
  if AHash = $FFFFFFFF then
  begin
    Result := '';
    exit;
  end;
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
  idx := AHash mod HashTableSize;
  incr := 1 + (AHash mod (HashTableSize - 2));
  while True do
  begin
    nstr := HashTable^[idx];
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    if (nstr = 0) or (nstr > StringCount) then
=======
    if nstr = 0 then
>>>>>>> graemeg/fixes_2_2
=======
    if nstr = 0 then
>>>>>>> origin/fixes_2_2
=======
    if (nstr = 0) or (nstr > StringCount) then
>>>>>>> origin/fixes_2.4
    begin
      Result := '';
      exit;
    end;
    if (OrigTable^[nstr - 1].length = LongWord(ALen)) and
       (StrComp(OrigStrings^[nstr - 1], AOrig) = 0) then
    begin
      Result := TranslStrings^[nstr - 1];
      exit;
    end;
    if idx >= HashTableSize - incr then
      Dec(idx, HashTableSize - incr)
    else
      Inc(idx, incr);
  end;
end;

function TMOFile.Translate(AOrig: String; AHash: LongWord): String;
begin
  Result := Translate(PChar(AOrig), Length(AOrig), AHash);
end;

function TMOFile.Translate(AOrig: String): String;
begin
  Result := Translate(AOrig, Hash(AOrig));
end;


// -------------------------------------------------------
//   Resourcestring translation procedures
// -------------------------------------------------------


function Translate (Name,Value : AnsiString; Hash : Longint; arg:pointer) : AnsiString;
<<<<<<< HEAD
<<<<<<< HEAD
var contextempty : boolean;
begin
  contextempty:=name='';
  Result:='';
  if not contextempty then
    Result:=TMOFile(arg).Translate(Name+#4+Value);
  if contextempty or (Result='') then
    Result:=TMOFile(arg).Translate(Value,Hash);
=======
begin
  Result:=TMOFile(arg).Translate(Value,Hash);
>>>>>>> graemeg/fixes_2_2
=======
begin
  Result:=TMOFile(arg).Translate(Value,Hash);
>>>>>>> origin/fixes_2_2
end;


procedure TranslateResourceStrings(AFile: TMOFile);
begin
  SetResourceStrings(@Translate,AFile);
end;


procedure TranslateUnitResourceStrings(const AUnitName:string; AFile: TMOFile);
begin
//  SetUnitResourceStrings(AUnitName,@Translate,AFile);
end;


{$ifdef windows}
procedure GetLanguageIDs(var Lang, FallbackLang: string);
var
  Buffer: array[1..4] of {$ifdef Wince}WideChar{$else}char{$endif};
  Country: string;
  UserLCID: LCID;
begin
  //defaults
  Lang := '';
  FallbackLang:='';
  UserLCID := GetUserDefaultLCID;
  if GetLocaleInfo(UserLCID, LOCALE_SABBREVLANGNAME, @Buffer[1], 4)<>0 then
    FallbackLang := lowercase(copy(Buffer,1,2));
  if GetLocaleInfo(UserLCID, LOCALE_SABBREVCTRYNAME, @Buffer[1], 4)<>0 then begin
    Country := copy(Buffer,1,2);

    // some 2 letter codes are not the first two letters of the 3 letter code
    // there are probably more, but first let us see if there are translations
    if (Buffer='PRT') then Country:='PT';

    Lang := FallbackLang+'_'+Country;
  end;
end;

{$else}

procedure GetLanguageIDs(var Lang, FallbackLang: string);
begin
  lang := GetEnvironmentVariable('LC_ALL');
  if Length(lang) = 0 then
  begin
    lang := GetEnvironmentVariable('LC_MESSAGES');
    if Length(lang) = 0 then
    begin
      lang := GetEnvironmentVariable('LANG');
      if Length(lang) = 0 then
        exit;   // no language defined via environment variables
    end;
  end;
  FallbackLang := Copy(lang, 1, 2);
end;
{$endif}

procedure TranslateResourceStrings(const AFilename: String);
var
  mo: TMOFile;
  lang, FallbackLang: String;
<<<<<<< HEAD
<<<<<<< HEAD
  fn: String;
begin
  GetLanguageIDs(Lang, FallbackLang);
  fn:=Format(AFilename, [FallbackLang]);

  if fileexists(fn) then
    begin
      try
        mo := TMOFile.Create(fn);
        try
          TranslateResourceStrings(mo);
        finally
          mo.Free;
        end;
      except
        on e: Exception do;
      end;
    end;
  lang := Copy(lang, 1, 5);
  fn:=Format(AFilename, [lang]);
  if fileexists(fn) then
    begin
      try
        mo := TMOFile.Create(Format(AFilename, [lang]));
        try
          TranslateResourceStrings(mo);
        finally
          mo.Free;
        end;
      except
        on e: Exception do;
      end;
    end;
=======
=======
>>>>>>> origin/fixes_2_2
begin
  GetLanguageIDs(Lang, FallbackLang);
  try
    mo := TMOFile.Create(Format(AFilename, [FallbackLang]));
    try
      TranslateResourceStrings(mo);
    finally
      mo.Free;
    end;
  except
    on e: Exception do;
  end;

  lang := Copy(lang, 1, 5);
  try
    mo := TMOFile.Create(Format(AFilename, [lang]));
    try
      TranslateResourceStrings(mo);
    finally
      mo.Free;
    end;
  except
    on e: Exception do;
  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


procedure TranslateUnitResourceStrings(const AUnitName:string; const AFilename: String);
var
  mo: TMOFile;
  lang, FallbackLang: String;
begin
  GetLanguageIDs(Lang, FallbackLang);
  try
    mo := TMOFile.Create(Format(AFilename, [FallbackLang]));
    try
      TranslateUnitResourceStrings(AUnitName,mo);
    finally
      mo.Free;
    end;
  except
    on e: Exception do;
  end;

  lang := Copy(lang, 1, 5);
  try
    mo := TMOFile.Create(Format(AFilename, [lang]));
    try
      TranslateUnitResourceStrings(AUnitName,mo);
    finally
      mo.Free;
    end;
  except
    on e: Exception do;
  end;
end;

finalization
  finalizeresourcetables;
end.
