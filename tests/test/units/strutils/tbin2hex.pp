program BinToHex_tests;

{$IFDEF FPC}
  {$mode Delphi}
{$ENDIF}

uses
  SysUtils, Strutils;

var
  BinByteArray: array[0..10] of Byte = (0, 2, 3, 7, 8, 10, 11, 12, 13, 14, 15);
  HexText: String;
  HexTextA: AnsiString;
  HexTextW: WideString;
  BinBytes, HexBytes: TBytes;

begin
  { mode dependent char }
  SetLength(HexText, Length(BinByteArray) * 2);
  BinToHex(@BinByteArray[0], PChar(HexText), Length(BinByteArray));
  if HexText <> '00020307080A0B0C0D0E0F' then halt(1);

  SetLength(HexText, Length(BinByteArray) * 2);
  BinToHex(Pointer(@BinByteArray[0]), PChar(HexText), Length(BinByteArray));
  if HexText <> '00020307080A0B0C0D0E0F' then halt(2);

  SetLength(HexText, Length(BinByteArray) * 2);
  BinToHex(@BinByteArray, PChar(HexText), Length(BinByteArray));
  if HexText <> '00020307080A0B0C0D0E0F' then halt(3);

  { ansichar variants }
  SetLength(HexTextA, Length(BinByteArray) * 2);
  BinToHex(@BinByteArray[0], PAnsiChar(HexTextA), Length(BinByteArray));
  if HexTextA <> '00020307080A0B0C0D0E0F' then halt(4);

  SetLength(HexTextA, Length(BinByteArray) * 2);
  BinToHex(Pointer(@BinByteArray[0]), PAnsiChar(HexTextA), Length(BinByteArray));
  if HexTextA <> '00020307080A0B0C0D0E0F' then halt(5);

  SetLength(HexTextA, Length(BinByteArray) * 2);
  BinToHex(@BinByteArray, PAnsiChar(HexTextA), Length(BinByteArray));
  if HexTextA <> '00020307080A0B0C0D0E0F' then halt(6);

  { widechar variants }
  SetLength(HexTextW, Length(BinByteArray) * 2);
  BinToHex(@BinByteArray[0], PWideChar(HexTextW), Length(BinByteArray));
  if HexTextW <> '00020307080A0B0C0D0E0F' then halt(7);

  SetLength(HexTextW, Length(BinByteArray) * 2);
  BinToHex(Pointer(@BinByteArray[0]), PWideChar(HexTextW), Length(BinByteArray));
  if HexTextW <> '00020307080A0B0C0D0E0F' then halt(8);

  SetLength(HexTextW, Length(BinByteArray) * 2);
  BinToHex(@BinByteArray, PWideChar(HexTextW), Length(BinByteArray));
  if HexTextW <> '00020307080A0B0C0D0E0F' then halt(9);

  { two char pointer variants }
  SetLength(HexTextA, Length(BinByteArray) * 2);
  BinToHex(PAnsiChar(@BinByteArray[0]), PAnsiChar(HexTextA), Length(BinByteArray));
  if HexTextA <> '00020307080A0B0C0D0E0F' then halt(10);

  SetLength(HexTextW, Length(BinByteArray) * 2);
  BinToHex(PAnsiChar(@BinByteArray), PWideChar(HexTextW), Length(BinByteArray));
  if HexTextW <> '00020307080A0B0C0D0E0F' then halt(11);

  { TBytes variants }
  BinBytes := TBytes.Create(1, 4, 5, 9, 10, 11, 12, 13, 14, 15);
  try
    SetLength(HexBytes, Length(BinBytes) * 2);
    FillByte(HexBytes[0], Length(HexBytes), 0);
    BinToHex(BinBytes, 0, HexBytes, 0, Length(BinBytes));
    if TEncoding.ANSI.GetString(HexBytes) <> '010405090A0B0C0D0E0F' then halt(12);

    SetLength(HexBytes, Length(BinBytes) * 2);
    FillByte(HexBytes[0], Length(HexBytes), Ord('a'));
    BinToHex(BinBytes, 2, HexBytes, 0, Length(BinBytes) - 2);
    if TEncoding.Default.GetString(HexBytes) <> '05090A0B0C0D0E0Faaaa' then halt(13);

    SetLength(HexBytes, Length(BinBytes) * 2);
    FillByte(HexBytes[0], Length(HexBytes), Ord('a'));
    BinToHex(BinBytes, 4, HexBytes, 2, Length(BinBytes) - 4);
    if TEncoding.Default.GetString(HexBytes) <> 'aa0A0B0C0D0E0Faaaaaa' then halt(14);

    SetLength(HexBytes, Length(BinBytes) * 2);
    FillByte(HexBytes[0], Length(HexBytes), Ord('a'));
    BinToHex(BinBytes, 0, HexBytes, 4, Length(BinBytes) - 2);
    if TEncoding.Default.GetString(HexBytes) <> 'aaaa010405090A0B0C0D' then halt(15);
  finally
    SetLength(HexBytes, 0);
    SetLength(BinBytes, 0);
  end;

  writeln('everything passed');
end.
