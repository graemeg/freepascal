{
    *********************************************************************
    Copyright (C) 2012 Paul Ishenin,
    member of the Free Pascal Development Team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    *********************************************************************
}

{$ifndef VER2_4}
{ TEncoding }

class function TEncoding.GetANSI: TEncoding;
begin
{$ifdef FPC_HAS_FEATURE_THREADING}
  EnterCriticalSection(FLock);
  try
{$endif}
    if not Assigned(FStandardEncodings[seAnsi]) then
    begin
      if Assigned(widestringmanager.GetStandardCodePageProc) then
        FStandardEncodings[seAnsi] := TMBCSEncoding.Create(widestringmanager.GetStandardCodePageProc(scpAnsi))
      else
        FStandardEncodings[seAnsi] := TMBCSEncoding.Create(DefaultSystemCodePage);
    end;
{$ifdef FPC_HAS_FEATURE_THREADING}
  finally
    LeaveCriticalSection(FLock);
  end;
{$endif}
  Result := FStandardEncodings[seAnsi];
end;

function TEncoding.GetAnsiBytes(const S: string): TBytes;
begin
  Result := GetAnsiBytes(S, 1, Length(S));
end;

function TEncoding.GetAnsiBytes(const S: string; CharIndex, CharCount: Integer
  ): TBytes;
begin
  Result := GetAnsiBytes(Pointer(@S[CharIndex]), CharCount);
end;

function TEncoding.GetAnsiString(const Bytes: TBytes): string;
begin
  Result := GetAnsiString(Bytes, 0, Length(Bytes));
end;

function TEncoding.GetAnsiString(const Bytes: TBytes; ByteIndex,
  ByteCount: Integer): string;
begin
  Result := GetAnsiString(Pointer(@Bytes[ByteIndex]), ByteCount);
  SetCodePage(RawByteString(Result), DefaultSystemCodePage, False);
end;

class function TEncoding.GetASCII: TEncoding;
begin
{$ifdef FPC_HAS_FEATURE_THREADING}
  EnterCriticalSection(FLock);
  try
{$endif}
    if not Assigned(FStandardEncodings[seAscii]) then
      FStandardEncodings[seAscii] := TMBCSEncoding.Create(CP_ASCII);
{$ifdef FPC_HAS_FEATURE_THREADING}
  finally
    LeaveCriticalSection(FLock);
  end;
{$endif}
  Result := FStandardEncodings[seAscii];
end;

class function TEncoding.GetBigEndianUnicode: TEncoding;
begin
{$ifdef FPC_HAS_FEATURE_THREADING}
  EnterCriticalSection(FLock);
  try
{$endif}
    if not Assigned(FStandardEncodings[seBigEndianUnicode]) then
      FStandardEncodings[seBigEndianUnicode] := TBigEndianUnicodeEncoding.Create;
{$ifdef FPC_HAS_FEATURE_THREADING}
  finally
    LeaveCriticalSection(FLock);
  end;
{$endif}
  Result := FStandardEncodings[seBigEndianUnicode];
end;

class function TEncoding.GetDefault: TEncoding;
begin
  Result := GetSystemEncoding;
end;

class function TEncoding.GetSystemEncoding: TEncoding;
var
  I: Integer;
begin
{$ifdef FPC_HAS_FEATURE_THREADING}
  EnterCriticalSection(FLock);
  try
{$endif}
    for I := Low(FSystemEncodings) to High(FSystemEncodings) do
    begin
      if FSystemEncodings[I].CodePage=DefaultSystemCodePage then
      begin
        Result := FSystemEncodings[I];
        if I<>Low(FSystemEncodings) then // exchange with first position to find it faster the next time
        begin
          FSystemEncodings[I] := FSystemEncodings[Low(FSystemEncodings)];
          FSystemEncodings[Low(FSystemEncodings)] := Result;
        end;
        Exit;
      end;
    end;
    // not found - create new encoding at first position
    Result := TMBCSEncoding.Create(DefaultSystemCodePage);
    SetLength(FSystemEncodings, Length(FSystemEncodings)+1);
    if High(FSystemEncodings)<>Low(FSystemEncodings) then
      FSystemEncodings[High(FSystemEncodings)] := FSystemEncodings[Low(FSystemEncodings)];
    FSystemEncodings[Low(FSystemEncodings)] := Result;
{$ifdef FPC_HAS_FEATURE_THREADING}
  finally
    LeaveCriticalSection(FLock);
  end;
{$endif}
end;

class function TEncoding.GetUnicode: TEncoding;
begin
{$ifdef FPC_HAS_FEATURE_THREADING}
  EnterCriticalSection(FLock);
  try
{$endif}
    if not Assigned(FStandardEncodings[seUnicode]) then
      FStandardEncodings[seUnicode] := TUnicodeEncoding.Create;
{$ifdef FPC_HAS_FEATURE_THREADING}
  finally
    LeaveCriticalSection(FLock);
  end;
{$endif}
  Result := FStandardEncodings[seUnicode];
end;

class function TEncoding.GetUTF7: TEncoding;
begin
{$ifdef FPC_HAS_FEATURE_THREADING}
  EnterCriticalSection(FLock);
  try
{$endif}
    if not Assigned(FStandardEncodings[seUTF7]) then
      FStandardEncodings[seUTF7] := TUTF7Encoding.Create;
{$ifdef FPC_HAS_FEATURE_THREADING}
  finally
    LeaveCriticalSection(FLock);
  end;
{$endif}
  Result := FStandardEncodings[seUTF7];
end;

class function TEncoding.GetUTF8: TEncoding;
begin
{$ifdef FPC_HAS_FEATURE_THREADING}
  EnterCriticalSection(FLock);
  try
{$endif}
    if not Assigned(FStandardEncodings[seUTF8]) then
      FStandardEncodings[seUTF8] := TUTF8Encoding.Create;
{$ifdef FPC_HAS_FEATURE_THREADING}
  finally
    LeaveCriticalSection(FLock);
  end;
{$endif}
  Result := FStandardEncodings[seUTF8];
end;

class procedure TEncoding.FreeEncodings;
var
  E: TStandardEncoding;
  I: Integer;
begin
{$ifdef FPC_HAS_FEATURE_THREADING}
  EnterCriticalSection(FLock);
  try
{$endif}
    for E := Low(FStandardEncodings) to High(FStandardEncodings) do
      FreeAndNil(FStandardEncodings[E]);
    for I := Low(FSystemEncodings) to High(FSystemEncodings) do
      FSystemEncodings[I].Free;
    SetLength(FSystemEncodings, 0);
{$ifdef FPC_HAS_FEATURE_THREADING}
  finally
    LeaveCriticalSection(FLock);
  end;
{$endif}
end;

class constructor TEncoding.Create;
var
  E: TStandardEncoding;
begin
  for E := Low(FStandardEncodings) to High(FStandardEncodings) do
    FStandardEncodings[E] := nil;
{$ifdef FPC_HAS_FEATURE_THREADING}
  InitCriticalSection(FLock);
{$endif}
end;

class destructor TEncoding.Destroy;
begin
  FreeEncodings;
{$ifdef FPC_HAS_FEATURE_THREADING}
  DoneCriticalSection(FLock);
{$endif}
end;

function TEncoding.Clone: TEncoding;
begin
  Result := nil;
end;

class function TEncoding.Convert(Source, Destination: TEncoding;
  const Bytes: TBytes): TBytes;
begin
  Result := Destination.GetBytes(Source.GetChars(Bytes));
end;

class function TEncoding.Convert(Source, Destination: TEncoding;
  const Bytes: TBytes; StartIndex, Count: Integer): TBytes;
begin
  Result := Destination.GetBytes(Source.GetChars(Bytes, StartIndex, Count));
end;

class function TEncoding.IsStandardEncoding(AEncoding: TEncoding): Boolean;
var
  Encoding: TEncoding;
begin
  if Assigned(AEncoding) then
  begin
    for Encoding in FStandardEncodings do
      if Encoding = AEncoding then
        Exit(True);
    for Encoding in FSystemEncodings do
      if Encoding = AEncoding then
        Exit(True);
  end;
  Result := False;
end;

class function TEncoding.GetBufferEncoding(const Buffer: TBytes; var AEncoding: TEncoding): Integer;
begin
  Result := GetBufferEncoding(Buffer, AEncoding, Default);
end;

class function TEncoding.GetBufferEncoding(const Buffer: TBytes;
  var AEncoding: TEncoding; ADefaultEncoding: TEncoding): Integer;

  function CheckEncoding(AEncoding: TEncoding; out ByteCount: Integer): Boolean;
  var
    Preamble: TBytes;
  begin
    Preamble := AEncoding.GetPreamble;
    ByteCount := Length(Preamble);
    Result := (Length(Buffer) >= ByteCount) and (ByteCount > 0);
    if Result then
      Result := CompareMem(@Preamble[0], @Buffer[0], ByteCount);
  end;

begin
  if Assigned(AEncoding) then
  begin
    if not CheckEncoding(AEncoding, Result) then
      Result := 0;
  end
  else
  if CheckEncoding(Unicode, Result) then
    AEncoding := Unicode
  else
  if CheckEncoding(BigEndianUnicode, Result) then
    AEncoding := BigEndianUnicode
  else
  if CheckEncoding(UTF8, Result) then
    AEncoding := UTF8
  else
  begin
    AEncoding := ADefaultEncoding;
    Result := 0;
  end;
end;

function TEncoding.GetByteCount(const Chars: TUnicodeCharArray): Integer;
begin
  Result := GetByteCount(Chars, 0, Length(Chars));
end;

function TEncoding.GetByteCount(const Chars: TUnicodeCharArray; CharIndex,
  CharCount: Integer): Integer;
begin
  if (CharCount < 0) or (Length(Chars) < CharCount + CharIndex) then
    raise EEncodingError.CreateFmt(SInvalidCount, [CharCount]);
  if (CharIndex < 0) then
    raise EEncodingError.CreateFmt(SCharacterIndexOutOfBounds, [CharIndex]);
  Result := GetByteCount(@Chars[CharIndex], CharCount);
end;

function TEncoding.GetByteCount(const S: UnicodeString): Integer;
begin
  Result := GetByteCount(PUnicodeChar(S), Length(S));
end;

function TEncoding.GetByteCount(const S: UnicodeString; CharIndex, CharCount: Integer): Integer;
begin
  if (CharCount < 0) or (Length(S) < CharCount + CharIndex) then
    raise EEncodingError.CreateFmt(SInvalidCount, [CharCount]);
  if (CharIndex < 1) then
    raise EEncodingError.CreateFmt(SCharacterIndexOutOfBounds, [CharIndex]);
  Result := GetByteCount(@S[CharIndex], CharCount);
end;

function TEncoding.GetBytes(const Chars: TUnicodeCharArray): TBytes;
begin
  SetLength(Result, GetByteCount(Chars));
  GetBytes(@Chars[0], Length(Chars), @Result[0], Length(Result));
end;

function TEncoding.GetBytes(const Chars: TUnicodeCharArray; CharIndex,
  CharCount: Integer): TBytes;
begin
  if (CharCount < 0) or (Length(Chars) < CharCount + CharIndex) then
    raise EEncodingError.CreateFmt(SInvalidCount, [CharCount]);
  if (CharIndex < 0) then
    raise EEncodingError.CreateFmt(SCharacterIndexOutOfBounds, [CharIndex]);
  SetLength(Result, GetByteCount(Chars, CharIndex, CharCount));
  GetBytes(@Chars[CharIndex], CharCount, @Result[0], Length(Result));
end;

function TEncoding.GetBytes(const Chars: TUnicodeCharArray; CharIndex,
  CharCount: Integer; const Bytes: TBytes; ByteIndex: Integer): Integer;
var
  ByteLen: Integer;
begin
  ByteLen := Length(Bytes);
  if (ByteLen = 0) and (CharCount > 0) then
    raise EEncodingError.Create(SInvalidDestinationArray);
  if (ByteIndex < 0) or (ByteLen < ByteIndex) then
    raise EEncodingError.CreateFmt(SInvalidDestinationIndex, [ByteIndex]);
  if (CharCount < 0) or (Length(Chars) < CharCount + CharIndex) then
    raise EEncodingError.CreateFmt(SInvalidCount, [CharCount]);
  if (CharIndex < 0) then
    raise EEncodingError.CreateFmt(SCharacterIndexOutOfBounds, [CharIndex]);
  Result := GetBytes(@Chars[CharIndex], CharCount, @Bytes[ByteIndex], ByteLen - ByteIndex);
end;

function TEncoding.GetBytes(const S: UnicodeString): TBytes;
begin
  SetLength(Result, GetByteCount(S));
  GetBytes(@S[1], Length(S), @Result[0], Length(Result));
end;

function TEncoding.GetBytes(const S: UnicodeString; CharIndex, CharCount: Integer;
  const Bytes: TBytes; ByteIndex: Integer): Integer;
var
  ByteLen: Integer;
begin
  ByteLen := Length(Bytes);
  if (ByteLen = 0) and (CharCount > 0) then
    raise EEncodingError.Create(SInvalidDestinationArray);
  if (ByteIndex < 0) or (ByteLen < ByteIndex) then
    raise EEncodingError.CreateFmt(SInvalidDestinationIndex, [ByteIndex]);
  if (CharCount < 0) or (Length(S) < CharCount + CharIndex) then
    raise EEncodingError.CreateFmt(SInvalidCount, [CharCount]);
  if (CharIndex < 1) then
    raise EEncodingError.CreateFmt(SCharacterIndexOutOfBounds, [CharIndex]);
  Result := GetBytes(@S[CharIndex], CharCount, @Bytes[ByteIndex], ByteLen - ByteIndex);
end;

function TEncoding.GetCharCount(const Bytes: TBytes): Integer;
begin
  Result := GetCharCount(@Bytes[0], Length(Bytes));
end;

function TEncoding.GetCharCount(const Bytes: TBytes; ByteIndex,
  ByteCount: Integer): Integer;
begin
  if (ByteIndex < 0) or (Length(Bytes) < ByteIndex) then
    raise EEncodingError.CreateFmt(SInvalidDestinationIndex, [ByteIndex]);
  Result := GetCharCount(@Bytes[ByteIndex], ByteCount);
end;

function TEncoding.GetChars(const Bytes: TBytes): TUnicodeCharArray;
begin
  SetLength(Result, GetCharCount(Bytes));
  GetChars(@Bytes[0], Length(Bytes), @Result[0], Length(Result));
end;

function TEncoding.GetChars(const Bytes: TBytes; ByteIndex, ByteCount: Integer): TUnicodeCharArray;
begin
  if (ByteIndex < 0) or (Length(Bytes) < ByteIndex) then
    raise EEncodingError.CreateFmt(SInvalidDestinationIndex, [ByteIndex]);
  SetLength(Result, GetCharCount(Bytes, ByteIndex, ByteCount));
  GetChars(@Bytes[ByteIndex], ByteCount, @Result[0], Length(Result));
end;

function TEncoding.GetChars(const Bytes: TBytes; ByteIndex, ByteCount: Integer;
  const Chars: TUnicodeCharArray; CharIndex: Integer): Integer;
var
  CharLen: Integer;
begin
  if (ByteIndex < 0) or (Length(Bytes) <= ByteIndex) then
    raise EEncodingError.CreateFmt(SInvalidDestinationIndex, [ByteIndex]);
  CharLen := Length(Chars);
  if (CharIndex < 0) or (CharLen <= CharIndex) then
    raise EEncodingError.CreateFmt(SCharacterIndexOutOfBounds, [CharIndex]);
  Result := GetChars(@Bytes[ByteIndex], ByteCount, @Chars[CharIndex], CharLen - CharIndex);
end;

class function TEncoding.GetEncoding(CodePage: Integer): TEncoding;
begin
  case CodePage of
    CP_UTF16: Result := TUnicodeEncoding.Create;
    CP_UTF16BE: Result := TBigEndianUnicodeEncoding.Create;
    CP_UTF7: Result := TUTF7Encoding.Create;
    CP_UTF8: Result := TUTF8Encoding.Create;
  else
    Result := TMBCSEncoding.Create(CodePage);
  end;
end;

class function TEncoding.GetEncoding(const EncodingName: UnicodeString): TEncoding;
var
  ACodePage: TSystemCodePage;
begin
  ACodePage := CodePageNameToCodePage(AnsiString(EncodingName));
  if ACodePage = $FFFF then
    raise EEncodingError.CreateFmt(SNotValidCodePageName, [EncodingName]);
  Result := TMBCSEncoding.Create(ACodePage);
end;

function TEncoding.GetString(const Bytes: TBytes): UnicodeString;
var
  Chars: TUnicodeCharArray;
begin
  Chars := GetChars(Bytes);
  SetString(Result, PUnicodeChar(Chars), Length(Chars));
end;

function TEncoding.GetString(const Bytes: TBytes; ByteIndex, ByteCount: Integer): UnicodeString;
var
  Chars: TUnicodeCharArray;
begin
  Chars := GetChars(Bytes, ByteIndex, ByteCount);
  SetString(Result, PUnicodeChar(Chars), Length(Chars));
end;

{ TMBCSEncoding }

function TMBCSEncoding.GetByteCount(Chars: PUnicodeChar; CharCount: Integer): Integer;
var
  S: RawByteString;
begin
  widestringmanager.Unicode2AnsiMoveProc(Chars, S, CodePage, CharCount);
  Result := Length(S);
end;

function TMBCSEncoding.GetBytes(Chars: PUnicodeChar; CharCount: Integer; Bytes: PByte;
  ByteCount: Integer): Integer;
var
  S: RawByteString;
begin
  widestringmanager.Unicode2AnsiMoveProc(Chars, S, CodePage, CharCount);
  Result := Length(S);
  if ByteCount < Result then
    Result := ByteCount;
  if Result > 0 then
    Move(S[1], Bytes[0], Result);
end;

function TMBCSEncoding.GetCharCount(Bytes: PByte; ByteCount: Integer): Integer;
var
  U: UnicodeString;
begin
  widestringmanager.Ansi2UnicodeMoveProc(PChar(Bytes), CodePage, U, ByteCount);
  Result := Length(U);
end;

function TMBCSEncoding.GetChars(Bytes: PByte; ByteCount: Integer; Chars: PUnicodeChar;
  CharCount: Integer): Integer;
var
  U: UnicodeString;
begin
  widestringmanager.Ansi2UnicodeMoveProc(PChar(Bytes), CodePage, U, ByteCount);
  Result := Length(U);
  if CharCount < Result then
    Result := CharCount;
  if Result > 0 then
    Move(U[1], Chars[0], Result * SizeOf(UnicodeChar));
end;

function TMBCSEncoding.GetCodePage: Cardinal;
begin
  Result := FCodePage;
end;

function TMBCSEncoding.GetEncodingName: UnicodeString;
begin
  Result := UnicodeString(CodePageToCodePageName(CodePage));
end;

constructor TMBCSEncoding.Create;
begin
  Create(DefaultSystemCodePage, 0, 0);
end;

constructor TMBCSEncoding.Create(ACodePage: Integer);
begin
  Create(ACodePage, 0, 0);
end;

constructor TMBCSEncoding.Create(ACodePage, MBToWCharFlags,
  WCharToMBFlags: Integer);
begin
  FCodePage := ACodePage;
  FMBToWCharFlags := MBToWCharFlags;
  FWCharToMBFlags := WCharToMBFlags;
  case ACodePage of
    CP_UTF7, CP_UTF8, CP_UTF16, CP_UTF16BE: FIsSingleByte := False;
  else
    FIsSingleByte := True;
  end;
end;

function TMBCSEncoding.Clone: TEncoding;
begin
  Result := TMBCSEncoding.Create(FCodePage, FMBToWCharFlags, FWCharToMBFlags);
end;

function TMBCSEncoding.GetAnsiBytes(Chars: PChar; CharCount: Integer): TBytes;
var
  S: RawByteString;
begin
  SetString(S, Chars, CharCount);
  SetCodePage(S, DefaultSystemCodePage, False);
  SetCodePage(S, GetCodePage, True);
  SetLength(Result, Length(S));
  if Length(S)>0 then
    Move(S[1], Result[0], Length(S));
end;

function TMBCSEncoding.GetAnsiString(Bytes: PByte; ByteCount: Integer): string;
begin
  SetString(Result, Pointer(Bytes), ByteCount);
  SetCodePage(RawByteString(Result), GetCodePage, False);
  SetCodePage(RawByteString(Result), DefaultSystemCodePage, True);
end;

function TMBCSEncoding.GetMaxByteCount(CharCount: Integer): Integer;
begin
  Result := CharCount;
end;

function TMBCSEncoding.GetMaxCharCount(ByteCount: Integer): Integer;
begin
  Result := ByteCount;
end;

function TMBCSEncoding.GetPreamble: TBytes;
begin
  case CodePage of
    CP_UTF8:
    begin
      SetLength(Result, 3);
      Result[0] := $EF;
      Result[1] := $BB;
      Result[2] := $BF;
    end;
    CP_UTF16:
    begin
      SetLength(Result, 2);
      Result[0] := $FF;
      Result[1] := $FE;
    end;
    CP_UTF16BE:
    begin
      SetLength(Result, 2);
      Result[0] := $FE;
      Result[1] := $FF;
    end;
  else
    Result := nil;
  end;
end;

{ TUTF7Encoding }

constructor TUTF7Encoding.Create;
begin
  inherited Create(CP_UTF7);
  FIsSingleByte := False;
end;

function TUTF7Encoding.Clone: TEncoding;
begin
  Result := TUTF7Encoding.Create;
end;

function TUTF7Encoding.GetMaxByteCount(CharCount: Integer): Integer;
begin
  Result := CharCount * 3 + 2;
end;

function TUTF7Encoding.GetMaxCharCount(ByteCount: Integer): Integer;
begin
  Result := ByteCount;
end;

{ TUTF8Encoding }

constructor TUTF8Encoding.Create;
begin
  inherited Create(CP_UTF8);
  FIsSingleByte := False;
end;

function TUTF8Encoding.Clone: TEncoding;
begin
  Result := TUTF8Encoding.Create;
end;

function TUTF8Encoding.GetMaxByteCount(CharCount: Integer): Integer;
begin
  Result := CharCount * 3;
end;

function TUTF8Encoding.GetMaxCharCount(ByteCount: Integer): Integer;
begin
  Result := ByteCount;
end;

function TUTF8Encoding.GetPreamble: TBytes;
begin
  SetLength(Result, 3);
  Result[0] := $EF;
  Result[1] := $BB;
  Result[2] := $BF;
end;

{ TUnicodeEncoding }

function TUnicodeEncoding.GetByteCount(Chars: PUnicodeChar; CharCount: Integer): Integer;
begin
  Result := CharCount * SizeOf(UnicodeChar);
end;

function TUnicodeEncoding.GetBytes(Chars: PUnicodeChar; CharCount: Integer;
  Bytes: PByte; ByteCount: Integer): Integer;
begin
  Result := CharCount * SizeOf(UnicodeChar);
  if ByteCount < Result then
    Result := ByteCount;
  if Result > 0 then
    Move(Chars[0], Bytes[0], Result);
end;

function TUnicodeEncoding.GetCharCount(Bytes: PByte; ByteCount: Integer): Integer;
begin
  Result := ByteCount div SizeOf(UnicodeChar);
end;

function TUnicodeEncoding.GetChars(Bytes: PByte; ByteCount: Integer;
  Chars: PUnicodeChar; CharCount: Integer): Integer;
begin
  Result := ByteCount div 2;
  if CharCount < Result then
    Result := CharCount;
  Move(Bytes[0], Chars[0], Result * SizeOf(UnicodeChar));
end;

function TUnicodeEncoding.GetCodePage: Cardinal;
begin
  Result := CP_UTF16;
end;

function TUnicodeEncoding.GetEncodingName: UnicodeString;
begin
  Result := UnicodeString(CodePageToCodePageName(CodePage));
end;

constructor TUnicodeEncoding.Create;
begin
  inherited Create;
  FIsSingleByte := False;
  FMaxCharSize := SizeOf(UnicodeChar);
end;

function TUnicodeEncoding.Clone: TEncoding;
begin
  Result := TUnicodeEncoding.Create;
end;

function TUnicodeEncoding.GetAnsiBytes(Chars: PChar; CharCount: Integer
  ): TBytes;
var
  U: UnicodeString;
begin
  widestringmanager.Ansi2UnicodeMoveProc(Chars, DefaultSystemCodePage, U, CharCount);
  SetLength(Result, Length(U)*SizeOf(UnicodeChar));
  if Length(Result)>0 then
    Move(U[1], Result[0], Length(Result));
end;

function TUnicodeEncoding.GetAnsiString(Bytes: PByte; ByteCount: Integer
  ): string;
begin
  widestringmanager.Unicode2AnsiMoveProc(PUnicodeChar(Bytes), RawByteString(Result), DefaultSystemCodePage, ByteCount div SizeOf(UnicodeChar));
end;

function TUnicodeEncoding.GetMaxByteCount(CharCount: Integer): Integer;
begin
  Result := CharCount * SizeOf(UnicodeChar);
end;

function TUnicodeEncoding.GetMaxCharCount(ByteCount: Integer): Integer;
begin
  Result := ByteCount div SizeOf(UnicodeChar);
end;

function TUnicodeEncoding.GetPreamble: TBytes;
begin
  SetLength(Result, 2);
  Result[0] := $FF;
  Result[1] := $FE;
end;

{ TBigEndianUnicodeEncoding }

function TBigEndianUnicodeEncoding.GetBytes(Chars: PUnicodeChar; CharCount: Integer;
  Bytes: PByte; ByteCount: Integer): Integer;
var
  LastByte: PByte;
begin
  Result := CharCount * SizeOf(UnicodeChar);
  if ByteCount < Result then
    Result := ByteCount;
  LastByte := @Bytes[Result];
  while Bytes < LastByte do
  begin
    Bytes^ := Hi(Word(Chars^));
    inc(Bytes);
    if Bytes < LastByte then
      Bytes^ := Lo(Word(Chars^));
    inc(Bytes);
    inc(Chars);
  end;
end;

function TBigEndianUnicodeEncoding.GetChars(Bytes: PByte; ByteCount: Integer;
  Chars: PUnicodeChar; CharCount: Integer): Integer;
var
  LastChar: PUnicodeChar;
begin
  Result := ByteCount div SizeOf(UnicodeChar);
  if CharCount < Result then
    Result := CharCount;
  LastChar := @Chars[Result];
  while Chars < LastChar do
  begin
    Chars^ := UnicodeChar(Bytes[1] + Bytes[0] shl 8);
    inc(Bytes, SizeOf(UnicodeChar));
    inc(Chars);
  end;
end;

function TBigEndianUnicodeEncoding.GetCodePage: Cardinal;
begin
  Result := CP_UTF16BE;
end;

function TBigEndianUnicodeEncoding.GetEncodingName: UnicodeString;
begin
  Result := UnicodeString(CodePageToCodePageName(CodePage));
end;

function TBigEndianUnicodeEncoding.Clone: TEncoding;
begin
  Result := TBigEndianUnicodeEncoding.Create;
end;

function TBigEndianUnicodeEncoding.GetAnsiBytes(Chars: PChar; CharCount: Integer
  ): TBytes;
begin
  Result := TEncoding.Unicode.GetAnsiBytes(Chars, CharCount);
  Swap(Result);
end;

function TBigEndianUnicodeEncoding.GetAnsiString(Bytes: PByte;
  ByteCount: Integer): string;
var
  B: TBytes;
begin
  if ByteCount=0 then
    Exit('');

  SetLength(B, ByteCount);
  Move(Bytes^, B[0], ByteCount);
  Swap(B);

  Result := TEncoding.Unicode.GetAnsiString(PByte(@B[0]), ByteCount);
end;

function TBigEndianUnicodeEncoding.GetPreamble: TBytes;
begin
  SetLength(Result, 2);
  Result[0] := $FE;
  Result[1] := $FF;
end;

procedure TBigEndianUnicodeEncoding.Swap(var B: TBytes);
var
  LastB, I: Integer;
  C: Byte;
begin
  LastB := Length(B)-1;
  I := 0;
  while I < LastB do
    begin
      C := B[I];
      B[I] := B[I+1];
      B[I+1] := C;
      Inc(I, 2);
    end;
end;

{$endif VER2_4}
