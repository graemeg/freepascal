 {************************************************************************}
 {*                     4-bit planar VGA mode routines                   *}
 {************************************************************************}


const

  VideoOfs = 0;


var

  VidMem: PByteArray;
  ScrWidth: SmallInt;


procedure bytemove(var source, dest; count: SmallInt);
var
  s, d: PByte;
begin
  s := PByte(@source);
  d := PByte(@dest);
  while count > 0 do begin
    d^ := s^;
    Inc(d);
    Inc(s);
    Dec(count);
  end;
end;



procedure PutPixel16(X,Y : SmallInt; Pixel: Word);
var
  offset: word;
  dummy: byte;
begin
  Inc(x, StartXViewPort);
  Inc(y, StartYViewPort);
  { convert to absolute coordinates and then verify clipping...}
  if ClipPixels then
  begin
    if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
      exit;
    if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
      exit;
  end;
  offset := y * 80 + (x shr 3) + VideoOfs;
  WritePortW($3ce, $0f01);       { Index 01 : Enable ops on all 4 planes }
  WritePortW($3ce, (Pixel and $ff) shl 8); { Index 00 : Enable correct plane and write color }

  WritePortW($3ce, 8 or ($8000 shr (x and $7)));{ Select correct bits to modify }
  dummy := VidMem^[offset];     { Read data byte into VGA latch register }
  VidMem^[offset] := dummy;     { Write the data into video memory }
end;


function GetPixel16(X,Y: SmallInt):word;
var
  dummy, offset: Word;
  shift: byte;
begin
  Inc(x, StartXViewPort);
  Inc(y, StartYViewPort);
  offset := Y * 80 + (x shr 3) + VideoOfs;
  WritePortW($3ce, 4);
  shift := 7 - (X and 7);
  dummy := (VidMem^[offset] shr shift) and 1;
  WritePortB($3cf, 1);
  dummy := dummy or (((VidMem^[offset] shr shift) and 1) shl 1);
  WritePortB($3cf, 2);
  dummy := dummy or (((VidMem^[offset] shr shift) and 1) shl 2);
  WritePortB($3cf, 3);
  dummy := dummy or (((VidMem^[offset] shr shift) and 1) shl 3);
  GetPixel16 := dummy;
end;


procedure GetScanLine16(x1, x2, y: SmallInt; var data);
var
  dummylong: longint;
  Offset, count, count2, amount, index: word;
  plane: byte;
begin
  inc(x1,StartXViewPort);
  inc(x2,StartXViewPort);
{$ifdef logging}
  LogLn('GetScanLine16 start, length to get: '+strf(x2-x1+1)+' at y = '+strf(y));
{$Endif logging}
  offset := (Y + StartYViewPort) * 80 + (x1 shr 3) + VideoOfs;
{$ifdef logging}
  LogLn('Offset: '+HexStr(offset,4)+' - ' + strf(offset));
{$Endif logging}
  { first get enough pixels so offset is 32bit aligned }
  amount := 0;
  index := 0;
  If ((x1 and 31) <> 0) Or
     ((x2-x1+1) < 32) Then
    Begin
      If ((x2-x1+1) >= 32+32-(x1 and 31)) Then
        amount := 32-(x1 and 31)
      Else amount := x2-x1+1;
{$ifdef logging}
      LogLn('amount to align to 32bits or to get all: ' + strf(amount));
{$Endif logging}
      For count := 0 to amount-1 do
        WordArray(Data)[Count] := getpixel16(x1-StartXViewPort+Count,y);
      index := amount;
      Inc(Offset,(amount+7) shr 3);
{$ifdef logging}
      LogLn('offset now: '+HexStr(offset,4)+' - ' + strf(offset));
      LogLn('index now: '+strf(index));
{$Endif logging}
    End;
  amount := x2-x1+1 - amount;
{$ifdef logging}
  LogLn('amount left: ' + strf(amount));
{$Endif logging}
  If amount = 0 Then Exit;
  WritePortB($3ce, 4);
  { first get everything from plane 3 (4th plane) }
  WritePortB($3cf, 3);
  Count := 0;
  For Count := 1 to (amount shr 5) Do
    Begin
      dummylong := PLongInt(@VidMem^[offset+(Count-1)*4])^;
      dummylong :=
        ((dummylong and $ff) shl 24) or
        ((dummylong and $ff00) shl 8) or
        ((dummylong and $ff0000) shr 8) or
        ((dummylong and $ff000000) shr 24);
      For Count2 := 31 downto 0 Do
        Begin
          WordArray(Data)[index+Count2] := DummyLong and 1;
          DummyLong := DummyLong shr 1;
        End;
      Inc(Index, 32);
    End;
{ Now get the data from the 3 other planes }
  plane := 3;
  Repeat
    Dec(Index,Count*32);
    Dec(plane);
    WritePortB($3cf, plane);
    Count := 0;
    For Count := 1 to (amount shr 5) Do
      Begin
        dummylong := PLongInt(@VidMem^[offset+(Count-1)*4])^;
        dummylong :=
          ((dummylong and $ff) shl 24) or
          ((dummylong and $ff00) shl 8) or
          ((dummylong and $ff0000) shr 8) or
          ((dummylong and $ff000000) shr 24);
        For Count2 := 31 downto 0 Do
          Begin
            WordArray(Data)[index+Count2] :=
              (WordArray(Data)[index+Count2] shl 1) or (DummyLong and 1);
            DummyLong := DummyLong shr 1;
          End;
        Inc(Index, 32);
      End;
  Until plane = 0;
  amount := amount and 31;
  Dec(index);
{$ifdef Logging}
  LogLn('Last array index written to: '+strf(index));
  LogLn('amount left: '+strf(amount)+' starting at x = '+strf(index+1));
{$Endif logging}
  For Count := 1 to amount Do
    WordArray(Data)[index+Count] := getpixel16(index+Count,y);
{$ifdef logging}
  LogLn('First 32 bytes gotten with getscanline16: ');
  If x2-x1+1 >= 32 Then
    Count2 := 32
  Else Count2 := x2-x1+1;
  For Count := 0 to Count2-1 Do
    Log(strf(WordArray(Data)[Count])+' ');
  LogLn('');
  If x2-x1+1 >= 32 Then
    Begin
      LogLn('Last 32 bytes gotten with getscanline16: ');
      For Count := 31 downto 0 Do
      Log(strf(WordArray(Data)[x2-x1-Count])+' ');
    End;
  LogLn('');
  GetScanLineDefault(x1-StartXViewPort,x2-StartXViewPort,y,Data);
  LogLn('First 32 bytes gotten with getscanlinedef: ');
  If x2-x1+1 >= 32 Then
    Count2 := 32
  Else Count2 := x2-x1+1;
  For Count := 0 to Count2-1 Do
    Log(strf(WordArray(Data)[Count])+' ');
  LogLn('');
  If x2-x1+1 >= 32 Then
    Begin
      LogLn('Last 32 bytes gotten with getscanlinedef: ');
      For Count := 31 downto 0 Do
      Log(strf(WordArray(Data)[x2-x1-Count])+' ');
    End;
  LogLn('');
  LogLn('GetScanLine16 end');
{$Endif logging}
end;


procedure DirectPutPixel16(X,Y : SmallInt);
{ x,y -> must be in global coordinates. No clipping. }
var
  color: word;
  offset: word;
  dummy: byte;
begin
  case CurrentWriteMode of
    XORPut:
      begin
        { getpixel wants local/relative coordinates }
        Color := GetPixel(x - StartXViewPort, y - StartYViewPort);
        Color := CurrentColor xor Color;
      end;
    OrPut:
      begin
        { getpixel wants local/relative coordinates }
        Color := GetPixel(x - StartXViewPort, y - StartYViewPort);
        Color := CurrentColor or Color;
      end;
    AndPut:
      begin
        { getpixel wants local/relative coordinates }
        Color := GetPixel(x - StartXViewPort, y - StartYViewPort);
        Color := CurrentColor and Color;
      end;
    NotPut:
      Color := Not Color;
    else
      Color := CurrentColor;
  end;
  offset := Y * 80 + (X shr 3) + VideoOfs;
  WritePortW($3ce, $f01);
  WritePortW($3ce, Color shl 8);
  WritePortW($3ce, 8 or $8000 shr (X and 7));
  dummy := VidMem^[offset];
  VidMem^[offset] := dummy;
end;


procedure HLine16(x, x2, y: SmallInt);
var
  xtmp: SmallInt;
  ScrOfs, HLength: Word;
  LMask, RMask: Byte;
begin
  { must we swap the values? }
  if x > x2 then
  begin
    xtmp := x2;
    x2 := x;
    x:= xtmp;
  end;
  { First convert to global coordinates }
  Inc(x, StartXViewPort);
  Inc(x2, StartXViewPort);
  Inc(y, StartYViewPort);
  if ClipPixels and LineClipped(x,y,x2,y,StartXViewPort,StartYViewPort,
    StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
    exit;

  ScrOfs := y * ScrWidth + x div 8;
  HLength := x2 div 8 - x div 8;
  LMask := $ff shr (x and 7);
{$ifopt r+}
{$define rangeOn}
{$r-}
{$endif}
{$ifopt q+}
{$define overflowOn}
{$q-}
{$endif}
  RMask:=$ff shl (7 - (x2 and 7));
{$ifdef rangeOn}
{$undef rangeOn}
{$r+}
{$endif}
{$ifdef overflowOn}
{$undef overflowOn}
{$q+}
{$endif}
  if HLength=0 then
    LMask:=LMask and RMask;
  WritePortB($3ce, 0);
  if CurrentWriteMode <> NotPut Then
    WritePortB($3cf, CurrentColor)
  else
    WritePortB($3cf, not CurrentColor);
  WritePortW($3ce, $0f01);
  WritePortB($3ce, 3);
  case CurrentWriteMode of
    XORPut:
      WritePortB($3cf, 3 shl 3);
    ANDPut:
      WritePortB($3cf, 1 shl 3);
    ORPut:
      WritePortB($3cf, 2 shl 3);
    NormalPut, NotPut:
      WritePortB($3cf, 0)
    else
      WritePortB($3cf, 0)
  end;

  WritePortB($3ce, 8);
  WritePortB($3cf, LMask);
{$ifopt r+}
{$define rangeOn}
{$r-}
{$endif}
{$ifopt q+}
{$define overflowOn}
{$q-}
{$endif}
  VidMem^[ScrOfs] := VidMem^[ScrOfs] + 1;
{$ifdef rangeOn}
{$undef rangeOn}
{$r+}
{$endif}
{$ifdef overflowOn}
{$undef overflowOn}
{$q+}
{$endif}
  if HLength>0 then
  begin
    Dec(HLength);
    Inc(ScrOfs);
    if HLength>0 then
    begin
      WritePortW($3ce, $ff08);
      bytemove(VidMem^[ScrOfs], VidMem^[ScrOfs], HLength);
      Inc(ScrOfs, HLength);
    end else
      WritePortB($3ce, 8);
    WritePortB($3cf, RMask);
{$ifopt r+}
{$define rangeOn}
{$r-}
{$endif}
{$ifopt q+}
{$define overflowOn}
{$q-}
{$endif}
    VidMem^[ScrOfs] := VidMem^[ScrOfs] + 1;
{$ifdef rangeOn}
{$undef rangeOn}
{$r+}
{$endif}
{$ifdef overflowOn}
{$undef overflowOn}
{$q+}
{$endif}
  end;
end;



procedure VLine16(x,y,y2: SmallInt);
var
  ytmp: SmallInt;
  ScrOfs,i: longint;
  BitMask: byte;

begin
  { must we swap the values? }
  if y > y2 then
  begin
    ytmp := y2;
    y2 := y;
    y:= ytmp;
  end;
  { First convert to global coordinates }
  Inc(x, StartXViewPort);
  Inc(y, StartYViewPort);
  Inc(y2, StartYViewPort);
  if ClipPixels and LineClipped(x,y,x,y2,StartXViewPort,StartYViewPort,
    StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
    exit;
  ScrOfs:=y*ScrWidth+x div 8;
  BitMask:=$80 shr (x and 7);
  WritePortB($3ce, 0);
  if CurrentWriteMode <> NotPut then
    WritePortB($3cf, CurrentColor)
  else
    WritePortB($3cf, not CurrentColor);
  WritePortW($3ce, $0f01);
  WritePortB($3ce, 8);
  WritePortB($3cf, BitMask);
  WritePortB($3ce, 3);
  case CurrentWriteMode of
    XORPut:
      WritePortB($3cf, 3 shl 3);
    ANDPut:
      WritePortB($3cf, 1 shl 3);
    ORPut:
      WritePortB($3cf, 2 shl 3);
    NormalPut, NotPut:
      WritePortB($3cf, 0)
    else
      WritePortB($3cf, 0)
  end;
  for i:=y to y2 do
  begin
{$ifopt r+}
{$define rangeOn}
{$r-}
{$endif}
{$ifopt q+}
{$define overflowOn}
{$q-}
{$endif}
    VidMem^[ScrOfs]:=VidMem^[ScrOfs]+1;
{$ifdef rangeOn}
{$undef rangeOn}
{$r+}
{$endif}
{$ifdef overflowOn}
{$undef overflowOn}
{$q+}
{$endif}
    Inc(ScrOfs, ScrWidth);
  end;
end;

