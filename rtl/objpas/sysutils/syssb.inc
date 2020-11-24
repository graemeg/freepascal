{ TGenericStringBuilder }

constructor TGenericStringBuilder.Create;
begin
  Create(DefaultCapacity,Maxint);
end;

constructor TGenericStringBuilder.Create(const AValue: SBString; aCapacity: Integer);
begin
  Create(aCapacity,Maxint);
  if (system.Length(AValue)>0) then
    Append(AValue);
end;


constructor TGenericStringBuilder.Create(const AValue: SBString; StartIndex, Alength,
  aCapacity: Integer);
begin
  Create(Copy(AValue,StartIndex+1,Alength), aCapacity);
end;

constructor TGenericStringBuilder.Create(aCapacity, aMaxCapacity: Integer);
begin
  FMaxCapacity:=aMaxCapacity;
  Capacity:=aCapacity;
  FLength:=0;
end;

constructor TGenericStringBuilder.Create(aCapacity: Integer);
begin
  Create(aCapacity,MaxInt);
end;

constructor TGenericStringBuilder.Create(const AValue: SBString);
begin
  Create(aValue,DefaultCapacity);
end;


{ Property getter/setter }

function TGenericStringBuilder.GetLength: Integer;
begin
  Result:=FLength;
end;

function TGenericStringBuilder.GetCapacity: Integer;
begin
  Result:=System.Length(FData);
end;

function TGenericStringBuilder.GetC(Index: Integer): SBChar;
begin
  CheckNegative(Index,'Index');
  CheckRange(Index,0,Length);
  Result:=FData[Index];
end;

procedure TGenericStringBuilder.SetC(Index: Integer; AValue: SBChar);
begin
  CheckNegative(Index,'Index');
  CheckRange(Index,0,Length-1);
  FData[Index]:=AValue;
end;

procedure TGenericStringBuilder.SetLength(AValue: Integer);

begin
  CheckNegative(AValue,'AValue');
  CheckRange(AValue,0,MaxCapacity);
  While AValue>Capacity do
    Grow;
  Flength:=AValue;
end;

{ Check functions }



procedure TGenericStringBuilder.CheckRange(Idx, Count, MaxLen: Integer);

begin
  if (Idx<0) or (Idx+Count>MaxLen) then
    Raise ERangeError.CreateFmt(SListIndexError,[Idx]);
end;


procedure TGenericStringBuilder.CheckNegative(const AValue: Integer;
  const AName: SBString);

begin
  if (AValue<0) then
    Raise ERangeError.CreateFmt(SParamIsNegative,[AName])
end;

{  These do the actual Appending/Inserting }

procedure TGenericStringBuilder.DoAppend(const S: {$IFDEF SBUNICODE}SBString{$ELSE}RawByteString{$ENDIF});

Var
  L,SL : Integer;

begin
  SL:=System.Length(S);
  if SL>0 then
    begin
    L:=Length;
    Length:=L+SL;
    Move(S[1], FData[L],SL*SizeOf(SBChar));
    end;
end;

procedure TGenericStringBuilder.DoAppend(const AValue: TSBCharArray; Idx, aCount: Integer
  );

Var
  L : integer;

begin
  L:=Length;
  CheckRange(Idx,aCount,System.Length(AValue));
  Length:=L+aCount;
  Move(AValue[Idx],FData[L],aCount*SizeOf(SBChar));
end;

procedure TGenericStringBuilder.DoInsert(Index: Integer; const AValue: SBString);

Var
  ShiftLen,LV : Integer;

begin
  CheckRange(Index,0,Length-1);
  LV:=System.Length(AValue);
  ShiftLen:=Length-Index;
  Length:=Length+LV;
  Move(FData[Index],FData[Index+LV],ShiftLen*SizeOf(SBChar));
  Move(AValue[1],FData[Index],LV*SizeOf(SBChar));
end;

procedure TGenericStringBuilder.DoInsert(Index: Integer; const AValue: TSBCharArray;
  StartIndex, SBCharCount: Integer);

Var
  ShiftLen : Integer;

begin
  CheckRange(Index,0,Length-1);
  CheckNegative(StartIndex,'StartIndex');
  CheckNegative(SBCharCount,'SBCharCount');
  CheckRange(StartIndex,SBCharCount,System.Length(AValue));
  Length:=Length+SBCharCount;
  ShiftLen:=Length-Index;
  if ShiftLen> 0 then
    Move(FData[Index], FData[Index+SBCharCount],ShiftLen*SizeOf(SBChar));
  Move(AValue[StartIndex],FData[Index],SBCharCount*SizeOf(SBChar));
end;

{ Public routines for appending }

function TGenericStringBuilder.Append(const AValue: UInt64): TGenericStringBuilder;
begin
  DoAppend(IntToStr(AValue));
  Result:=self;
end;

function TGenericStringBuilder.Append(const AValue: TSBCharArray): TGenericStringBuilder;

var
  I,L: Integer;

begin
  I:=-1;
  L:=System.Length(AValue);
  If L=0 then
    Exit(Self);
  Repeat
    Inc(I);
  Until (I>=L) or (AValue[I]=#0);
  DoAppend(AValue,0,I);
  Result:=Self;
end;

function TGenericStringBuilder.Append(const AValue: Single): TGenericStringBuilder;
begin
  DoAppend(FloatToStr(AValue));
  Result:=self;
end;

function TGenericStringBuilder.Append(const AValue: Word): TGenericStringBuilder;
begin
  Append(IntToStr(AValue));
  Result:=self;
end;

function TGenericStringBuilder.Append(const AValue: Cardinal): TGenericStringBuilder;
begin
  DoAppend(IntToStr(AValue));
  Result:=self;
end;

function TGenericStringBuilder.Append(const AValue: SBChar; RepeatCount: Integer
  ): TGenericStringBuilder;
begin
  DoAppend(StringOfChar(AValue,RepeatCount));
  Result:=Self;
end;


function TGenericStringBuilder.Append(const AValue: Shortint): TGenericStringBuilder;
begin
  DoAppend(IntToStr(AValue));
  Result:=Self;
end;

function TGenericStringBuilder.Append(const AValue: SBChar): TGenericStringBuilder;
begin
  DoAppend(AValue);
  Result:=Self;
end;

function TGenericStringBuilder.Append(const AValue: Currency): TGenericStringBuilder;
begin
  DoAppend(CurrToStr(AValue));
  Result:=Self;
end;

function TGenericStringBuilder.Append(const AValue: Boolean): TGenericStringBuilder;
begin
  DoAppend(BoolToStr(AValue, True));
  Result:=Self;
end;

function TGenericStringBuilder.Append(const AValue: Byte): TGenericStringBuilder;
begin
  DoAppend(IntToStr(AValue));
  Result:=Self;
end;

function TGenericStringBuilder.Append(const AValue: Double): TGenericStringBuilder;
begin
  DoAppend(FloatToStr(AValue));
  Result:=Self;
end;

function TGenericStringBuilder.Append(const AValue: Int64): TGenericStringBuilder;
begin
  DoAppend(IntToStr(AValue));
  Result:=Self;
end;

function TGenericStringBuilder.Append(const AValue: TObject): TGenericStringBuilder;
begin
  DoAppend(AValue.ToString);
  Result:=Self;
end;

function TGenericStringBuilder.Append(const AValue: Smallint): TGenericStringBuilder;
begin
  DoAppend(IntToStr(AValue));
  Result:=Self;
end;

function TGenericStringBuilder.Append(const AValue: LongInt): TGenericStringBuilder;
begin
  DoAppend(IntToStr(AValue));
  Result:=Self;
end;

Function TGenericStringBuilder.Append(const AValue: TSBCharArray; StartIndex, SBCharCount: Integer): TGenericStringBuilder;

begin
  DoAppend(AValue,StartIndex,SBCharCount);
  Result:=Self;
end;

Function TGenericStringBuilder.Append(const AValue: SBString; StartIndex, Count: Integer): TGenericStringBuilder;

begin
  CheckRange(StartIndex,Count,System.Length(AValue));
  DoAppend(Copy(AValue,StartIndex+1,Count));
  Result:=Self;
end;

function TGenericStringBuilder.Append(const AValue: PSBChar): TGenericStringBuilder;
begin
  DoAppend(AnsiString(AValue));
  Result:=Self;
end;

{$IFDEF SBUNICODE}
function TGenericStringBuilder.Append(const AValue: SBString): TGenericStringBuilder;
begin
  DoAppend(AValue);
  Result:=Self;
end;
{$ENDIF}

function TGenericStringBuilder.Append(const AValue: RawByteString): TGenericStringBuilder;
begin
  {$IFDEF SBUNICODE}
  DoAppend(SBString(AValue));
  {$ELSE}
  DoAppend(AValue);
  {$ENDIF}
  Result:=Self;
end;

function TGenericStringBuilder.AppendFormat(const Fmt: SBString;
  const Args: array of const): TGenericStringBuilder;
begin
  DoAppend(Format(Fmt,Args));
  Result:=Self;
end;

function TGenericStringBuilder.Append(const Fmt: SBString;
  const Args: array of const): TGenericStringBuilder;
begin
  DoAppend(Format(Fmt,Args));
  Result:=Self;
end;

function TGenericStringBuilder.AppendLine: TGenericStringBuilder;
begin
  DoAppend(sLineBreak);
  Result:=Self;
end;

function TGenericStringBuilder.AppendLine(const AValue: RawByteString): TGenericStringBuilder;
begin
  DoAppend(AValue);
  Result:=AppendLine();
end;

procedure TGenericStringBuilder.Clear;
begin
  Length:=0;
  Capacity:=DefaultCapacity;
end;


procedure TGenericStringBuilder.CopyTo(SourceIndex: Integer;
  Var Destination: TSBCharArray; DestinationIndex: Integer; Count: Integer);

begin
  CheckNegative(Count,'Count');
  CheckNegative(DestinationIndex,'DestinationIndex');
  CheckRange(DestinationIndex,Count,System.Length(Destination));
  if Count>0 then
    begin
    CheckRange(SourceIndex,Count,Length);
    Move(FData[SourceIndex],Destination[DestinationIndex],Count * SizeOf(SBChar));
    end;
end;


function TGenericStringBuilder.EnsureCapacity(aCapacity: Integer): Integer;
begin
  CheckRange(aCapacity,0,MaxCapacity);
  if Capacity<aCapacity then
    Capacity:=aCapacity;
  Result:=Capacity;
end;

function TGenericStringBuilder.Equals(StringBuilder: TGenericStringBuilder): Boolean;
begin
  Result:=(StringBuilder<>nil);
  if Result then
    Result:=(Length=StringBuilder.Length)
             and (MaxCapacity=StringBuilder.MaxCapacity)
             and CompareMem(@FData[0],@StringBuilder.FData[0],Length*SizeOf(SBChar));
end;

procedure TGenericStringBuilder.Grow;

var
  NewCapacity: SizeInt;

begin
  NewCapacity:=Capacity*2;
  if NewCapacity>MaxCapacity then
    NewCapacity:=MaxCapacity;
  Capacity:=NewCapacity;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: TObject
  ): TGenericStringBuilder;
begin
  DoInsert(Index,AValue.ToString());
  Result:=Self;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: Int64
  ): TGenericStringBuilder;
begin
  DoInsert(Index,IntToStr(AValue));
  Result:=Self;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: Single
  ): TGenericStringBuilder;
begin
  DoInsert(Index,FloatToStr(AValue));
  Result:=Self;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: SBString
  ): TGenericStringBuilder;

begin
  DoInsert(Index,AValue);
  Result:=Self;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: Word
  ): TGenericStringBuilder;
begin
  DoInsert(Index,IntToStr(AValue));
  Result:=Self;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: Shortint
  ): TGenericStringBuilder;
begin
  DoInsert(Index, IntToStr(AValue));
  Result:=Self;
end;


function TGenericStringBuilder.Insert(Index: Integer; const AValue: Currency
  ): TGenericStringBuilder;
begin
  DoInsert(Index,CurrToStr(AValue));
  Result:=Self;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: SBChar
  ): TGenericStringBuilder;
begin
  DoInsert(Index,AValue);
  Result:=Self;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: Byte
  ): TGenericStringBuilder;
begin
  DoInsert(Index,IntToStr(AValue));
  Result:=Self;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: Double
  ): TGenericStringBuilder;
begin
  DoInsert(Index,FloatToStr(AValue));
  Result:=Self;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: LongInt
  ): TGenericStringBuilder;
begin
  DoInsert(Index,IntToStr(AValue));
  Result:=Self;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: Smallint
  ): TGenericStringBuilder;
begin
  DoInsert(Index,IntToStr(AValue));
  Result:=Self;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: Boolean
  ): TGenericStringBuilder;
begin
  DoInsert(Index,BoolToStr(AValue,True));
  Result:=Self;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: SBString;
  const aRepeatCount: Integer): TGenericStringBuilder;
var
  I: Integer;
begin
  for I:=0 to aRepeatCount-1 do
    DoInsert(Index,AValue);
  Result:=Self;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: TSBCharArray
  ): TGenericStringBuilder;
begin
  DoInsert(Index,AValue,0,System.Length(AValue));
  Result:=Self;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: TSBCharArray;
  startIndex: Integer; SBCharCount: Integer): TGenericStringBuilder;
begin
  DoInsert(Index,AValue,StartIndex,SBCharCount);
  Result:=Self;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: Cardinal
  ): TGenericStringBuilder;
begin
  DoInsert(Index,IntToStr(AValue));
  Result:=self;
end;

function TGenericStringBuilder.Insert(Index: Integer; const AValue: UInt64
  ): TGenericStringBuilder;
begin
  DoInsert(Index,IntToStr(AValue));
  Result:=self;
end;

procedure TGenericStringBuilder.Shrink;

begin
  if (Capacity div 4)>=Length then
    Capacity:=Capacity div 2;
end;

function TGenericStringBuilder.Remove(StartIndex: Integer; RemLength: Integer
  ): TGenericStringBuilder;

Var
  MoveIndex : Integer;

begin
  if (RemLength=0) then
    exit(Self);
  CheckNegative(RemLength,'RemLength');
  CheckRange(StartIndex,0,Length);
  MoveIndex:=StartIndex+RemLength;
  CheckRange(MoveIndex,0,Length);
  if (Length-Moveindex)>0 then
    Move(FData[MoveIndex],FData[StartIndex],(Length-MoveIndex)*SizeOf(SBChar));
  Length:=Length-RemLength;
  Shrink;
  Result:=Self;
end;

Function TGenericStringBuilder.Replace(const OldValue, NewValue: SBRawString; StartIndex, Count: Integer): TGenericStringBuilder;

var
  Cur : PSBChar;
  CurIndex,MaxIndex : Integer;
  OldLen, NewLen, Delta : Integer;
  BC : SBChar;

begin
  if Count=0 then
    Exit(Self);
  // Some checks.
  CheckNegative(StartIndex,'StartIndex');
  CheckNegative(Count,'Count');
  CheckRange(Startindex,Count,Length);
  // Init
  OldLen:=System.Length(OldValue);
  NewLen:=System.Length(NewValue);
  Delta:=NewLen-OldLen;
  MaxIndex:=StartIndex+Count;
  CurIndex:=StartIndex;
  BC:=OldValue[1];
  Cur:=@FData[StartIndex];
  // Loop
  while (CurIndex<Length-OldLen+1) and (CurIndex<MaxIndex) do
    begin
    if (Cur^=BC) then
      begin
      if 0=StrLComp(@FData[CurIndex],PSBChar(OldValue),OldLen) then
        begin
        // Do actual replace.
        DoReplace(CurIndex,OldValue,NewValue);
        Inc(CurIndex,NewLen-1);
        // DoReplace may have reallocated memory, so changed pointers, reset pointer
        Cur:=@FData[CurIndex];
        // The max index must be increased/decreased with Delta
        //  0123456789012
        // 'zzbczzedeafzz' replace('e','qqqq',6,3) -> zzbczzqqqqdqqqqafzz
        Inc(MaxIndex,Delta);
        end;
      end;
    Inc(CurIndex);
    Inc(Cur);
    end;
  Result:=Self;
end;

Function TGenericStringBuilder.Replace(const OldChar, NewChar: SBChar; StartIndex,
  Count: Integer): TGenericStringBuilder;
var
  I : Integer;
  Cur : PSBChar;

begin
  if Count=0 then
    Exit(Self);
  CheckNegative(StartIndex,'StartIndex');
  CheckNegative(Count,'Count');
  CheckRange(StartIndex,Count-1,Length);
  Cur:=@FData[StartIndex];
  For I:=1 to Count do
    begin
    if Cur^=OldChar then
      Cur^:=NewChar;
    Inc(Cur);
    end;
  Result:=Self;
end;

Function TGenericStringBuilder.Replace(const OldChar, NewChar: SBChar): TGenericStringBuilder;

begin
  Result:=Replace(OldChar,NewChar,0,Length);
end;

Function TGenericStringBuilder.Replace(const OldValue, NewValue: SBRawString): TGenericStringBuilder;
begin
  Result:=Replace(OldValue,NewValue,0,Length);
end;

procedure TGenericStringBuilder.SetCapacity(AValue: Integer);
begin
  if (AValue>FMaxCapacity) then
    Raise ERangeError.CreateFmt(SListCapacityError,[AValue]);
  if (AValue<Length) then
    Raise ERangeError.CreateFmt(SListCapacityError,[AValue]);
  System.SetLength(FData,AValue);
end;


function TGenericStringBuilder.ToString: SBString;
begin
  Result:=ToString(0,Length);
end;

function TGenericStringBuilder.ToString(aStartIndex: Integer; aLength: Integer
  ): SBString;
begin
  if (aLength=0) then
    Result:=''
  else
    begin
    CheckNegative(aStartIndex,'aStartIndex');
    CheckNegative(aLength,'aLength');
    CheckRange(aStartIndex,aLength,Length);
    System.SetLength(Result,aLength);
    Move(FData[aStartIndex],Result[1],aLength*SizeOf(SBChar));
    end;
end;

procedure TGenericStringBuilder.DoReplace(Index: Integer; const Old, New: SBString);

var
  NVLen,OVLen,OLen,Delta,TailStart: Integer;

begin
  NVLen:=System.Length(New);
  OVLen:=System.Length(Old);
  Delta:=NVLen-OVLen;
  if (Delta<>0) then
    begin
    OLen:=Length;
    if (Delta>0) then
      Length:=OLen+Delta;
    TailStart:=Index+OVlen;
    Move(FData[TailStart],FData[Index+NVLen],(OLen-TailStart)*SizeOf(SBChar));
    if (Delta<0) then
      Length:=OLen+Delta;
    end;
  Move(New[1],FData[Index],NVLen*SizeOf(SBChar));
end;

