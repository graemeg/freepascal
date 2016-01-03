unit dbf_common;

interface

{$I dbf_common.inc}

uses
  SysUtils, Classes, DB
{$ifndef WINDOWS}
  , Types, dbf_wtil
{$ifdef KYLIX}
  , Libc
{$endif}  
{$endif}
  ;


const
<<<<<<< HEAD
  TDBF_MAJOR_VERSION      = 7;
  TDBF_MINOR_VERSION      = 0;
  TDBF_SUB_MINOR_VERSION  = 0;
=======
  TDBF_MAJOR_VERSION      = 6;
  TDBF_MINOR_VERSION      = 9;
  TDBF_SUB_MINOR_VERSION  = 2;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  TDBF_TABLELEVEL_FOXPRO = 25;
  TDBF_TABLELEVEL_VISUALFOXPRO = 30; {Source: http://www.codebase.com/support/kb/?article=C01059}

  JulianDateDelta = 1721425; { number of days between 1.1.4714 BC and "0" }

type
  EDbfError = class (EDatabaseError);
  EDbfWriteError = class (EDbfError);

  TDbfFieldType = char;

  TXBaseVersion   = (xUnknown, xClipper, xBaseIII, xBaseIV, xBaseV, xFoxPro, xBaseVII, xVisualFoxPro);
  TSearchKeyType = (stEqual, stGreaterEqual, stGreater);

  TDateTimeHandling       = (dtDateTime, dtBDETimeStamp);

//-------------------------------------

  PDateTime = ^TDateTime;
{$ifndef FPC_VERSION}
  PtrInt = Longint;
{$endif}

  PSmallInt = ^SmallInt;
  PCardinal = ^Cardinal;
  PDouble = ^Double;
  PString = ^String;

{$ifdef DELPHI_3}
  dword = cardinal;
{$endif}

//-------------------------------------

{$ifndef SUPPORT_FREEANDNIL}
// some procedures for the less lucky who don't have newer versions yet :-)
procedure FreeAndNil(var v);
{$endif}
procedure FreeMemAndNil(var P: Pointer);

//-------------------------------------

{$ifndef SUPPORT_PATHDELIM}
const
{$ifdef WINDOWS}
  PathDelim = '\';
{$else}
 {$IFDEF UNIX}
  PathDelim = '/';
 {$ELSE UNIX}
  PathDelim = '\';
 {$ENDIF UNIX}
{$endif}
{$endif}

{$ifndef SUPPORT_INCLTRAILPATHDELIM}
function IncludeTrailingPathDelimiter(const Path: string): string;
{$endif}

//-------------------------------------

function GetCompletePath(const Base, Path: string): string;
function GetCompleteFileName(const Base, FileName: string): string;
function IsFullFilePath(const Path: string): Boolean; // full means not relative
function DateTimeToBDETimeStamp(aDT: TDateTime): double;
function BDETimeStampToDateTime(aBT: double): TDateTime;
procedure FindNextName(BaseName: string; var OutName: string; var Modifier: Integer);
{$ifdef USE_CACHE}
function GetFreeMemory: Integer;
{$endif}

// Convert word to big endian
function SwapWordBE(const Value: word): word;
// Convert word to little endian
function SwapWordLE(const Value: word): word;
// Convert integer to big endian
function SwapIntBE(const Value: dword): dword;
// Convert integer to little endian
function SwapIntLE(const Value: dword): dword;
{$ifdef SUPPORT_INT64}
// Convert int64 to big endian
procedure SwapInt64BE(Value, Result: Pointer); register;
// Convert int64 to little endian
procedure SwapInt64LE(Value, Result: Pointer); register;
{$endif}

// Translate string between codepages
function TranslateString(FromCP, ToCP: Cardinal; Src, Dest: PChar; Length: Integer): Integer;

// Returns a pointer to the first occurence of Chr in Str within the first Length characters
// Does not stop at null (#0) terminator!
function MemScan(const Buffer: Pointer; Chr: Byte; Length: Integer): Pointer;

// Delphi 3 does not have a Min function
{$ifdef DELPHI_3}
{$ifndef DELPHI_4}
function Min(x, y: integer): integer;
function Max(x, y: integer): integer;
{$endif}
{$endif}

implementation

{$ifdef WINDOWS}
uses
  Windows;
{$endif}

//====================================================================

function GetCompletePath(const Base, Path: string): string;
begin
  if IsFullFilePath(Path)
  then begin
    Result := Path;
  end else begin
    if Length(Base) > 0 then
      Result := ExpandFileName(IncludeTrailingPathDelimiter(Base) + Path)
    else
      Result := ExpandFileName(Path);
  end;

  // add last backslash if not present
  if Length(Result) > 0 then
    Result := IncludeTrailingPathDelimiter(Result);
end;

function IsFullFilePath(const Path: string): Boolean; // full means not relative
begin
{$ifdef SUPPORT_DRIVES_AND_UNC}
  Result := Length(Path) > 1;
  if Result then
    // check for 'x:' or '\\' at start of path
    Result := ((Path[2]=':') and (upcase(Path[1]) in ['A'..'Z']))
      or ((Path[1]='\') and (Path[2]='\'));
{$else}  // Linux / Unix
  Result := Length(Path) > 0;
  if Result then
    Result := Path[1]='/';
 {$endif}
end;

//====================================================================

function GetCompleteFileName(const Base, FileName: string): string;
var
  lpath: string;
  lfile: string;
begin
  lpath := GetCompletePath(Base, ExtractFilePath(FileName));
  lfile := ExtractFileName(FileName);
  lpath := lpath + lfile;
  result := lpath;
end;

function DateTimeToBDETimeStamp(aDT: TDateTime): double;
var
  aTS: TTimeStamp;
begin
  aTS := DateTimeToTimeStamp(aDT);
  Result := TimeStampToMSecs(aTS);
end;

function BDETimeStampToDateTime(aBT: double): TDateTime;
var
  aTS: TTimeStamp;
begin
  aTS := MSecsToTimeStamp(Round(aBT));
  Result := TimeStampToDateTime(aTS);
end;

//====================================================================

{$ifndef SUPPORT_FREEANDNIL}

procedure FreeAndNil(var v);
var
  Temp: TObject;
begin
  Temp := TObject(v);
  TObject(v) := nil;
  Temp.Free;
end;

{$endif}

procedure FreeMemAndNil(var P: Pointer);
var
  Temp: Pointer;
begin
  Temp := P;
  P := nil;
  FreeMem(Temp);
end;

//====================================================================

{$ifndef SUPPORT_INCLTRAILPATHDELIM}
{$ifndef SUPPORT_INCLTRAILBACKSLASH}

function IncludeTrailingPathDelimiter(const Path: string): string;
var
  len: Integer;
begin
  Result := Path;
  len := Length(Result);
  if len = 0 then
    Result := PathDelim
  else
  if Result[len] <> PathDelim then
    Result := Result + PathDelim;
end;

{$else}

function IncludeTrailingPathDelimiter(const Path: string): string;
begin
{$ifdef WINDOWS}
  Result := IncludeTrailingBackslash(Path);
{$else}
 {$IFDEF UNIX}
  Result := IncludeTrailingSlash(Path);
 {$ELSE UNIX}
  Result := IncludeTrailingBackslash(Path);
 {$ENDIF UNIX}
{$endif}
end;

{$endif}
{$endif}

{$ifdef USE_CACHE}

function GetFreeMemory: Integer;
var
  MemStatus: TMemoryStatus;
begin
  GlobalMemoryStatus(MemStatus);
  Result := MemStatus.dwAvailPhys;
end;

{$endif}

//====================================================================
// Utility routines
//====================================================================

{$ifdef ENDIAN_LITTLE}
function SwapWordBE(const Value: word): word;
{$else}
function SwapWordLE(const Value: word): word;
{$endif}
begin
  Result := ((Value and $FF) shl 8) or ((Value shr 8) and $FF);
end;

{$ifdef ENDIAN_LITTLE}
function SwapWordLE(const Value: word): word;
{$else}
function SwapWordBE(const Value: word): word;
{$endif}
begin
  Result := Value;
end;

{$ifdef FPC}

function SwapIntBE(const Value: dword): dword;
begin
  Result := BEtoN(Value);
end;

function SwapIntLE(const Value: dword): dword;
begin
  Result := LEtoN(Value);
end;

procedure SwapInt64BE(Value, Result: Pointer);
begin
  PInt64(Result)^ := BEtoN(PInt64(Value)^);
end;

procedure SwapInt64LE(Value, Result: Pointer);
begin
  PInt64(Result)^ := LEtoN(PInt64(Value)^);
end;

{$else}
{$ifdef USE_ASSEMBLER_486_UP}

function SwapIntBE(const Value: dword): dword; register; assembler;
asm
  BSWAP EAX;
end;

procedure SwapInt64BE(Value {EAX}, Result {EDX}: Pointer); register; assembler;
asm
  MOV ECX, dword ptr [EAX] 
  MOV EAX, dword ptr [EAX + 4] 
  BSWAP ECX 
  BSWAP EAX 
  MOV dword ptr [EDX+4], ECX 
  MOV dword ptr [EDX], EAX 
end;

{$else}

function SwapIntBE(const Value: Cardinal): Cardinal;
begin
  PByteArray(@Result)[0] := PByteArray(@Value)[3];
  PByteArray(@Result)[1] := PByteArray(@Value)[2];
  PByteArray(@Result)[2] := PByteArray(@Value)[1];
  PByteArray(@Result)[3] := PByteArray(@Value)[0];
end;

procedure SwapInt64BE(Value, Result: Pointer); register;
var
  PtrResult: PByteArray;
  PtrSource: PByteArray;
begin
  // temporary storage is actually not needed, but otherwise compiler crashes (?)
  PtrResult := PByteArray(Result);
  PtrSource := PByteArray(Value);
  PtrResult[0] := PtrSource[7];
  PtrResult[1] := PtrSource[6];
  PtrResult[2] := PtrSource[5];
  PtrResult[3] := PtrSource[4];
  PtrResult[4] := PtrSource[3];
  PtrResult[5] := PtrSource[2];
  PtrResult[6] := PtrSource[1];
  PtrResult[7] := PtrSource[0];
end;

{$endif}

function SwapIntLE(const Value: dword): dword;
begin
  Result := Value;
end;

{$ifdef SUPPORT_INT64}

procedure SwapInt64LE(Value, Result: Pointer);
begin
  PInt64(Result)^ := PInt64(Value)^;
end;

{$endif}

{$endif}

function TranslateString(FromCP, ToCP: Cardinal; Src, Dest: PChar; Length: Integer): Integer;
var
  WideCharStr: array[0..1023] of WideChar;
  wideBytes: Cardinal;
begin
  if Length = -1 then
    Length := StrLen(Src);
  Result := Length;
{$ifndef WINCE}
  if (FromCP = GetOEMCP) and (ToCP = GetACP) then
    OemToCharBuffA(Src, Dest, Length)
  else
  if (FromCP = GetACP) and (ToCP = GetOEMCP) then
    CharToOemBuffA(Src, Dest, Length)
  else
{$endif}
  if FromCP = ToCP then
  begin
    if Src <> Dest then
      Move(Src^, Dest^, Length);
  end else begin
    // does this work on Win95/98/ME?
    wideBytes := MultiByteToWideChar(FromCP, MB_PRECOMPOSED, Src, Length, LPWSTR(@WideCharStr[0]), 1024);
    Result := WideCharToMultiByte(ToCP, 0, LPWSTR(@WideCharStr[0]), wideBytes, Dest, Length, nil, nil);
  end;
end;

procedure FindNextName(BaseName: string; var OutName: string; var Modifier: Integer);
var
  Extension: string;
begin
  Extension := ExtractFileExt(BaseName);
  BaseName := Copy(BaseName, 1, Length(BaseName)-Length(Extension));
  repeat
    Inc(Modifier);
    OutName := ChangeFileExt(BaseName+'_'+IntToStr(Modifier), Extension);
  until not FileExists(OutName);
end;

{$ifdef FPC}

function MemScan(const Buffer: Pointer; Chr: Byte; Length: Integer): Pointer;
var
  I: Integer;
begin
  // Make sure we pass a buffer of bytes instead of a pchar otherwise
  // the call will always fail
  I := System.IndexByte(PByte(Buffer)^, Length, Chr);
  if I = -1 then
    Result := nil
  else
    Result := Buffer+I;
end;

{$else}

function MemScan(const Buffer: Pointer; Chr: Byte; Length: Integer): Pointer;
asm
        PUSH    EDI
        MOV     EDI,Buffer
        MOV     AL, Chr
        MOV     ECX,Length
        REPNE   SCASB
        MOV     EAX,0
        JNE     @@1
        MOV     EAX,EDI
        DEC     EAX
@@1:    POP     EDI
end;

{$endif}

{$ifdef DELPHI_3}
{$ifndef DELPHI_4}

function Min(x, y: integer): integer;
begin
  if x < y then
    result := x
  else
    result := y;
end;

function Max(x, y: integer): integer;
begin
  if x < y then
    result := y
  else
    result := x;
end;

{$endif}
{$endif}

end.



