{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{****************************************************************************}
{*                         TBinaryObjectWriter                              *}
{****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPUNONE}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$IFNDEF FPC_HAS_TYPE_EXTENDED}
procedure DoubleToExtended(d : double; e : pointer);
var mant : qword;
    exp : smallint;
    sign : boolean;
begin
  mant:=(qword(d) and $000FFFFFFFFFFFFF) shl 12;
  exp :=(qword(d) shr 52) and $7FF;
  sign:=(qword(d) and $8000000000000000)<>0;
  case exp of
       0 : begin
             if mant<>0 then  //denormalized value: hidden bit is 0. normalize it
             begin
               exp:=16383-1022;
               while (mant and $8000000000000000)=0 do
               begin
                 dec(exp);
                 mant:=mant shl 1;
               end;
               dec(exp); //don't shift, most significant bit is not hidden in extended
             end;
           end;
    2047 : exp:=$7FFF //either infinity or NaN
    else
    begin
      inc(exp,16383-1023);
      mant:=(mant shr 1) or $8000000000000000; //unhide hidden bit
    end;
  end;
  if sign then exp:=exp or $8000;
  mant:=NtoLE(mant);
  exp:=NtoLE(word(exp));
  move(mant,pbyte(e)[0],8); //mantissa         : bytes 0..7
  move(exp,pbyte(e)[8],2);  //exponent and sign: bytes 8..9
end;
{$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

procedure TBinaryObjectWriter.WriteWord(w : word); {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
begin
  w:=NtoLE(w);
  Write(w,2);
end;

procedure TBinaryObjectWriter.WriteDWord(lw : longword); {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
begin
  lw:=NtoLE(lw);
  Write(lw,4);
end;

procedure TBinaryObjectWriter.WriteQWord(qw : qword); {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
begin
  qw:=NtoLE(qw);
  Write(qw,8);
end;

<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPUNONE}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TBinaryObjectWriter.WriteExtended(e : extended); {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
{$IFNDEF FPC_HAS_TYPE_EXTENDED}
var ext : array[0..9] of byte;
{$ENDIF}
begin
{$IFNDEF FPC_HAS_TYPE_EXTENDED}
  {$IFDEF FPC_DOUBLE_HILO_SWAPPED}
  { SwapDoubleHiLo defined in reader.inc }
  SwapDoubleHiLo(e);
  {$ENDIF FPC_DOUBLE_HILO_SWAPPED}
  DoubleToExtended(e,@(ext[0]));
  Write(ext[0],10);
{$ELSE}
  Write(e,sizeof(e));
{$ENDIF}
end;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

constructor TBinaryObjectWriter.Create(Stream: TStream; BufSize: Integer);
begin
  inherited Create;
  If (Stream=Nil) then
    Raise EWriteError.Create(SEmptyStreamIllegalWriter);
  FStream := Stream;
  FBufSize := BufSize;
  GetMem(FBuffer, BufSize);
end;

destructor TBinaryObjectWriter.Destroy;
begin
  // Flush all data which hasn't been written yet
  FlushBuffer;

  if Assigned(FBuffer) then
    FreeMem(FBuffer, FBufSize);

  inherited Destroy;
end;

procedure TBinaryObjectWriter.BeginCollection;
begin
  WriteValue(vaCollection);
end;

procedure TBinaryObjectWriter.WriteSignature;

begin
  Write(FilerSignature, SizeOf(FilerSignature));
end;

procedure TBinaryObjectWriter.BeginComponent(Component: TComponent;
  Flags: TFilerFlags; ChildPos: Integer);
var
  Prefix: Byte;
begin

  { Only write the flags if they are needed! }
  if Flags <> [] then
  begin
    Prefix := TFilerFlagsInt(Flags) or $f0;
    Write(Prefix, 1);
    if ffChildPos in Flags then
      WriteInteger(ChildPos);
  end;

  WriteStr(Component.ClassName);
  WriteStr(Component.Name);
end;

procedure TBinaryObjectWriter.BeginList;
begin
  WriteValue(vaList);
end;

procedure TBinaryObjectWriter.EndList;
begin
  WriteValue(vaNull);
end;

procedure TBinaryObjectWriter.BeginProperty(const PropName: String);
begin
  WriteStr(PropName);
end;

procedure TBinaryObjectWriter.EndProperty;
begin
end;

procedure TBinaryObjectWriter.WriteBinary(const Buffer; Count: LongInt);
begin
  WriteValue(vaBinary);
  WriteDWord(longword(Count));
  Write(Buffer, Count);
end;

procedure TBinaryObjectWriter.WriteBoolean(Value: Boolean);
begin
  if Value then
    WriteValue(vaTrue)
  else
    WriteValue(vaFalse);
end;

{$ifndef FPUNONE}
procedure TBinaryObjectWriter.WriteFloat(const Value: Extended);
begin
  WriteValue(vaExtended);
  WriteExtended(Value);
end;

procedure TBinaryObjectWriter.WriteSingle(const Value: Single);
begin
  WriteValue(vaSingle);
  WriteDWord(longword(Value));
end;
{$endif}

procedure TBinaryObjectWriter.WriteCurrency(const Value: Currency);
begin
  WriteValue(vaCurrency);
  WriteQWord(qword(Value));
end;


{$ifndef FPUNONE}
procedure TBinaryObjectWriter.WriteDate(const Value: TDateTime);
begin
  WriteValue(vaDate);
  WriteQWord(qword(Value));
end;
{$endif}

procedure TBinaryObjectWriter.WriteIdent(const Ident: string);
begin
  { Check if Ident is a special identifier before trying to just write
    Ident directly }
  if UpperCase(Ident) = 'NIL' then
    WriteValue(vaNil)
  else if UpperCase(Ident) = 'FALSE' then
    WriteValue(vaFalse)
  else if UpperCase(Ident) = 'TRUE' then
    WriteValue(vaTrue)
  else if UpperCase(Ident) = 'NULL' then
    WriteValue(vaNull) else
  begin
    WriteValue(vaIdent);
    WriteStr(Ident);
  end;
end;

procedure TBinaryObjectWriter.WriteInteger(Value: Int64);
var
  s: ShortInt;
  i: SmallInt;
  l: Longint;
begin
  { Use the smallest possible integer type for the given value: }
  if (Value >= -128) and (Value <= 127) then
  begin
    WriteValue(vaInt8);
    s := Value;
    Write(s, 1);
  end else if (Value >= -32768) and (Value <= 32767) then
  begin
    WriteValue(vaInt16);
    i := Value;
    WriteWord(word(i));
  end else if (Value >= -$80000000) and (Value <= $7fffffff) then
  begin
    WriteValue(vaInt32);
    l := Value;
    WriteDWord(longword(l));
  end else
  begin
    WriteValue(vaInt64);
    WriteQWord(qword(Value));
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2_2
  end;
end;

procedure TBinaryObjectWriter.WriteUInt64(Value: QWord);
var
  s: ShortInt;
  i: SmallInt;
  l: Longint;
begin
  { Use the smallest possible integer type for the given value: }
  if (Value <= 127) then
  begin
    WriteValue(vaInt8);
    s := Value;
    Write(s, 1);
  end else if (Value <= 32767) then
  begin
    WriteValue(vaInt16);
    i := Value;
    WriteWord(word(i));
  end else if (Value <= $7fffffff) then
  begin
    WriteValue(vaInt32);
    l := Value;
    WriteDWord(longword(l));
  end else
  begin
    WriteValue(vaQWord);
    WriteQWord(Value);
=======
>>>>>>> graemeg/fixes_2_2
  end;
end;


procedure TBinaryObjectWriter.WriteMethodName(const Name: String);
begin
  if Length(Name) > 0 then
  begin
    WriteValue(vaIdent);
    WriteStr(Name);
  end else
    WriteValue(vaNil);
end;

procedure TBinaryObjectWriter.WriteSet(Value: LongInt; SetType: Pointer);
type
  tset = set of 0..31;
var
  i: Integer;
begin
  WriteValue(vaSet);
  for i := 0 to 31 do
  begin
    if (i in tset(Value)) then
      WriteStr(GetEnumName(PTypeInfo(SetType), i));
  end;
  WriteStr('');
end;

procedure TBinaryObjectWriter.WriteString(const Value: String);
var
  i: Integer;
  b: byte;
begin
  i := Length(Value);
  if i <= 255 then
  begin
    WriteValue(vaString);
    b := i;
    Write(b, 1);
  end else
  begin
    WriteValue(vaLString);
    WriteDWord(longword(i));
  end;
  if i > 0 then
    Write(Value[1], i);
end;

procedure TBinaryObjectWriter.WriteWideString(const Value: WideString);
var len : longword;
{$IFDEF ENDIAN_BIG}
    i : integer;
    ws : widestring;
{$ENDIF}
begin
  WriteValue(vaWString);
  len:=Length(Value);
  WriteDWord(len);
  if len > 0 then
  begin
    {$IFDEF ENDIAN_BIG}
    setlength(ws,len);
    for i:=1 to len do
      ws[i]:=widechar(SwapEndian(word(Value[i])));
    Write(ws[1], len*sizeof(widechar));
    {$ELSE}
    Write(Value[1], len*sizeof(widechar));
    {$ENDIF}
  end;
<<<<<<< HEAD
<<<<<<< HEAD
end;
                      
procedure TBinaryObjectWriter.WriteUnicodeString(const Value: UnicodeString);
var len : longword;
{$IFDEF ENDIAN_BIG}
    i : integer;
    us : UnicodeString;
{$ENDIF}
begin
  WriteValue(vaUString);
  len:=Length(Value);
  WriteDWord(len);
  if len > 0 then
  begin
    {$IFDEF ENDIAN_BIG}
    setlength(us,len);
    for i:=1 to len do
      us[i]:=widechar(SwapEndian(word(Value[i])));
    Write(us[1], len*sizeof(UnicodeChar));
    {$ELSE}
    Write(Value[1], len*sizeof(UnicodeChar));
    {$ENDIF}
  end;
=======
>>>>>>> origin/fixes_2_2
end;

procedure TBinaryObjectWriter.WriteVariant(const VarValue: variant);
begin
  { The variant manager will handle varbyref and vararray transparently for us
  }
  case (tvardata(VarValue).vtype and varTypeMask) of
    varEmpty:
      begin
        WriteValue(vaNil);
      end;
    varNull:
      begin
        WriteValue(vaNull);
      end;
    { all integer sizes must be split for big endian systems }
    varShortInt,varSmallInt,varInteger,varInt64:
      begin
        WriteInteger(VarValue);
      end;
    varQWord:
      begin
        WriteUInt64(VarValue);
      end;
    varBoolean:
      begin
        WriteBoolean(VarValue);
      end;
    varCurrency:
      begin
        WriteCurrency(VarValue);
      end;
{$ifndef fpunone}
    varSingle:
      begin
        WriteSingle(VarValue);
      end;
    varDouble:
      begin
        WriteFloat(VarValue);
      end;
    varDate:
      begin
        WriteDate(VarValue);
      end;
{$endif fpunone}
    varOleStr,varString:
      begin
        WriteWideString(VarValue);
      end;
    else
      raise EWriteError.CreateFmt(SUnsupportedPropertyVariantType, [Ord(tvardata(VarValue).vtype)]);
  end;
=======
>>>>>>> graemeg/fixes_2_2
end;


procedure TBinaryObjectWriter.FlushBuffer;
begin
  FStream.WriteBuffer(FBuffer^, FBufPos);
  FBufPos := 0;
end;

procedure TBinaryObjectWriter.Write(const Buffer; Count: LongInt);
var
  CopyNow: LongInt;
  SourceBuf: PChar;
begin
  SourceBuf:=@Buffer;
  while Count > 0 do
  begin
    CopyNow := Count;
    if CopyNow > FBufSize - FBufPos then
      CopyNow := FBufSize - FBufPos;
    Move(SourceBuf^, PChar(FBuffer)[FBufPos], CopyNow);
    Dec(Count, CopyNow);
    Inc(FBufPos, CopyNow);
    inc(SourceBuf, CopyNow);
    if FBufPos = FBufSize then
      FlushBuffer;
  end;
end;

procedure TBinaryObjectWriter.WriteValue(Value: TValueType);
var
  b: byte;
begin
  b := byte(Value);
  Write(b, 1);
end;

procedure TBinaryObjectWriter.WriteStr(const Value: String);
var
  i: integer;
  b: byte;
begin
  i := Length(Value);
  if i > 255 then
    i := 255;
  b := i;
  Write(b, 1);
  if i > 0 then
    Write(Value[1], i);
end;



{****************************************************************************}
{*                             TWriter                                      *}
{****************************************************************************}


constructor TWriter.Create(ADriver: TAbstractObjectWriter);
begin
  inherited Create;
  FDriver := ADriver;
end;

constructor TWriter.Create(Stream: TStream; BufSize: Integer);
begin
  inherited Create;
  If (Stream=Nil) then
    Raise EWriteError.Create(SEmptyStreamIllegalWriter);
  FDriver := CreateDriver(Stream, BufSize);
  FDestroyDriver := True;
end;

destructor TWriter.Destroy;
begin
  if FDestroyDriver then
    FDriver.Free;
  inherited Destroy;
end;

function TWriter.CreateDriver(Stream: TStream; BufSize: Integer): TAbstractObjectWriter;
begin
  Result := TBinaryObjectWriter.Create(Stream, BufSize);
end;

Type
  TPosComponent = Class(TObject)
    FPos : Integer;
    FComponent : TComponent;
    Constructor Create(APos : Integer; AComponent : TComponent);
  end;

Constructor TPosComponent.Create(APos : Integer; AComponent : TComponent);

begin
  FPos:=APos;
  FComponent:=AComponent;
end;

// Used as argument for calls to TComponent.GetChildren:
procedure TWriter.AddToAncestorList(Component: TComponent);
begin
  FAncestors.AddObject(Component.Name,TPosComponent.Create(FAncestors.Count,Component));
end;

procedure TWriter.DefineProperty(const Name: String;
  ReadData: TReaderProc; AWriteData: TWriterProc; HasData: Boolean);
begin
  if HasData and Assigned(AWriteData) then
  begin
    // Write the property name and then the data itself
    Driver.BeginProperty(FPropPath + Name);
    AWriteData(Self);
    Driver.EndProperty;
  end;
end;

procedure TWriter.DefineBinaryProperty(const Name: String;
  ReadData, AWriteData: TStreamProc; HasData: Boolean);
begin
  if HasData and Assigned(AWriteData) then
  begin
    // Write the property name and then the data itself
    Driver.BeginProperty(FPropPath + Name);
    WriteBinary(AWriteData);
    Driver.EndProperty;
  end;
end;

procedure TWriter.Write(const Buffer; Count: Longint);
begin
  //This should give an exception if write is not implemented (i.e. TTextObjectWriter)
  //but should work with TBinaryObjectWriter.
  Driver.Write(Buffer, Count);
end;

procedure TWriter.SetRoot(ARoot: TComponent);
begin
  inherited SetRoot(ARoot);
  // Use the new root as lookup root too
  FLookupRoot := ARoot;
end;

procedure TWriter.WriteSignature;

begin
  FDriver.WriteSignature;
end;

procedure TWriter.WriteBinary(AWriteData: TStreamProc);
var
  MemBuffer: TMemoryStream;
  BufferSize: Longint;
begin
  { First write the binary data into a memory stream, then copy this buffered
    stream into the writing destination. This is necessary as we have to know
    the size of the binary data in advance (we're assuming that seeking within
    the writer stream is not possible) }
  MemBuffer := TMemoryStream.Create;
  try
    AWriteData(MemBuffer);
    BufferSize := MemBuffer.Size;
    Driver.WriteBinary(MemBuffer.Memory^, BufferSize);
  finally
    MemBuffer.Free;
  end;
end;

procedure TWriter.WriteBoolean(Value: Boolean);
begin
  Driver.WriteBoolean(Value);
end;

procedure TWriter.WriteChar(Value: Char);
begin
  WriteString(Value);
end;

procedure TWriter.WriteWideChar(Value: WideChar);
begin
  WriteWideString(Value);
end;

procedure TWriter.WriteCollection(Value: TCollection);
var
  i: Integer;
begin
  Driver.BeginCollection;
  if Assigned(Value) then
    for i := 0 to Value.Count - 1 do
    begin
      { Each collection item needs its own ListBegin/ListEnd tag, or else the
        reader wouldn't be able to know where an item ends and where the next
        one starts }
      WriteListBegin;
      WriteProperties(Value.Items[i]);
      WriteListEnd;
    end;
  WriteListEnd;
end;

procedure TWriter.DetermineAncestor(Component : TComponent);

Var
  I : Integer;

begin
  // Should be set only when we write an inherited with children.
  if Not Assigned(FAncestors) then
    exit;
  I:=FAncestors.IndexOf(Component.Name);
  If (I=-1) then
    begin
    FAncestor:=Nil;
    FAncestorPos:=-1;
    end
  else
    With TPosComponent(FAncestors.Objects[i]) do
      begin
      FAncestor:=FComponent;
      FAncestorPos:=FPos;
      end;
end;

procedure TWriter.DoFindAncestor(Component : TComponent);

Var
  C : TComponent;

<<<<<<< HEAD
<<<<<<< HEAD
begin
  if Assigned(FOnFindAncestor) then
    if (Ancestor=Nil) or (Ancestor is TComponent) then
      begin
      C:=TComponent(Ancestor);
      FOnFindAncestor(Self,Component,Component.Name,C,FRootAncestor);
      Ancestor:=C;
      end;
end;

procedure TWriter.WriteComponent(Component: TComponent);

var
  SA : TPersistent;
  SR, SRA : TComponent;
begin
  SR:=FRoot;
  SA:=FAncestor;
  SRA:=FRootAncestor;
=======
begin
=======
begin
>>>>>>> origin/fixes_2_2
  if Assigned(FOnFindAncestor) then
    if (Ancestor=Nil) or (Ancestor is TComponent) then
      begin
      C:=TComponent(Ancestor);
      FOnFindAncestor(Self,Component,Component.Name,C,FRootAncestor);
      Ancestor:=C;
      end;
end;

procedure TWriter.WriteComponent(Component: TComponent);

var
  SA : TPersistent;
  SR : TComponent;
  
begin
  SR:=FRoot;
  SA:=FAncestor;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Try
    Component.FComponentState:=Component.FComponentState+[csWriting];
    Try
      // Possibly set ancestor.
      DetermineAncestor(Component);
      DoFindAncestor(Component); // Mainly for IDE when a parent form had an ancestor renamed...
      // Will call WriteComponentData.
      Component.WriteState(Self);
      FDriver.EndList;
    Finally
      Component.FComponentState:=Component.FComponentState-[csWriting];
    end;
  Finally
    FAncestor:=SA;
    FRoot:=SR;
<<<<<<< HEAD
<<<<<<< HEAD
    FRootAncestor:=SRA;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
end;

procedure TWriter.WriteChildren(Component : TComponent);

Var
  SRoot, SRootA : TComponent;
  SList : TStringList;
  SPos : Integer;
  I : Integer;
  
begin
  // Write children list. 
  // While writing children, the ancestor environment must be saved
  // This is recursive...
  SRoot:=FRoot;
  SRootA:=FRootAncestor;
  SList:=FAncestors;
  SPos:=FCurrentPos;
  try
    FAncestors:=Nil;
    FCurrentPos:=0;
    FAncestorPos:=-1;
    if csInline in Component.ComponentState then
       FRoot:=Component;
    if (FAncestor is TComponent) then
       begin
       FAncestors:=TStringList.Create;
       if csInline in TComponent(FAncestor).ComponentState then
         FRootAncestor := TComponent(FAncestor);
       TComponent(FAncestor).GetChildren(@AddToAncestorList,FRootAncestor);
       FAncestors.Sorted:=True;
       end;
    try
      Component.GetChildren(@WriteComponent, FRoot);
    Finally
      If Assigned(Fancestors) then
        For I:=0 to FAncestors.Count-1 do 
          FAncestors.Objects[i].Free;
      FreeAndNil(FAncestors);
    end;    
  finally
    FAncestors:=Slist;
    FRoot:=SRoot;
    FRootAncestor:=SRootA;
    FCurrentPos:=SPos;
    FAncestorPos:=Spos;
  end;
end;

procedure TWriter.WriteComponentData(Instance: TComponent);
var 
  Flags: TFilerFlags;
begin
  Flags := [];
  If (Assigned(FAncestor)) and  //has ancestor
     (not (csInline in Instance.ComponentState) or // no inline component
      // .. or the inline component is inherited
      (csAncestor in Instance.Componentstate) and (FAncestors <> nil)) then
    Flags:=[ffInherited]
  else If csInline in Instance.ComponentState then
    Flags:=[ffInline];
  If (FAncestors<>Nil) and ((FCurrentPos<>FAncestorPos) or (FAncestor=Nil)) then
    Include(Flags,ffChildPos);
  FDriver.BeginComponent(Instance,Flags,FCurrentPos);
  If (FAncestors<>Nil) then
    Inc(FCurrentPos);
  WriteProperties(Instance);
  WriteListEnd;
  // Needs special handling of ancestor.
  If not IgnoreChildren then
    WriteChildren(Instance);
end;

procedure TWriter.WriteDescendent(ARoot: TComponent; AAncestor: TComponent);
begin
  FRoot := ARoot;
  FAncestor := AAncestor;
  FRootAncestor := AAncestor;
  FLookupRoot := ARoot;
  WriteSignature;
  WriteComponent(ARoot);
end;

{$ifndef FPUNONE}
procedure TWriter.WriteFloat(const Value: Extended);
begin
  Driver.WriteFloat(Value);
end;

procedure TWriter.WriteSingle(const Value: Single);
begin
  Driver.WriteSingle(Value);
end;
{$endif}

procedure TWriter.WriteCurrency(const Value: Currency);
begin
  Driver.WriteCurrency(Value);
end;

{$ifndef FPUNONE}
procedure TWriter.WriteDate(const Value: TDateTime);
begin
  Driver.WriteDate(Value);
end;
{$endif}

procedure TWriter.WriteIdent(const Ident: string);
begin
  Driver.WriteIdent(Ident);
end;

procedure TWriter.WriteInteger(Value: LongInt);
begin
  Driver.WriteInteger(Value);
end;

procedure TWriter.WriteInteger(Value: Int64);
begin
  Driver.WriteInteger(Value);
end;

procedure TWriter.WriteSet(Value: LongInt; SetType: Pointer); 

begin
  Driver.WriteSet(Value,SetType);
end;

procedure TWriter.WriteVariant(const VarValue: Variant);
begin
  Driver.WriteVariant(VarValue);
end;

procedure TWriter.WriteListBegin;
begin
  Driver.BeginList;
end;

procedure TWriter.WriteListEnd;
begin
  Driver.EndList;
end;

procedure TWriter.WriteProperties(Instance: TPersistent);
var PropCount,i : integer;
    PropList  : PPropList;
begin
  PropCount:=GetPropList(Instance,PropList);
  if PropCount>0 then 
    try
      for i := 0 to PropCount-1 do
        if IsStoredProp(Instance,PropList^[i]) then
          WriteProperty(Instance,PropList^[i]);
    Finally    
      Freemem(PropList);
    end;
  Instance.DefineProperties(Self);
end;


procedure TWriter.WriteProperty(Instance: TPersistent; PropInfo: Pointer);
var
  HasAncestor: Boolean;
  PropType: PTypeInfo;
  Value, DefValue: LongInt;
  Ident: String;
  IntToIdentFn: TIntToIdent;
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPUNONE}
=======
  SingleV : Single;
>>>>>>> graemeg/fixes_2_2
=======
  SingleV : Single;
>>>>>>> origin/fixes_2_2
  FloatValue, DefFloatValue: Extended;
{$endif}
  MethodValue: TMethod;
  DefMethodValue: TMethod;
  WStrValue, WDefStrValue: WideString;
  StrValue, DefStrValue: String;
  UStrValue, UDefStrValue: UnicodeString;
  AncestorObj: TObject;
  C,Component: TComponent;
  ObjValue: TObject;
  SavedAncestor: TPersistent;
  SavedPropPath, Name: String;
  Int64Value, DefInt64Value: Int64;
  VarValue, DefVarValue : tvardata;
  BoolValue, DefBoolValue: boolean;
  Handled: Boolean;

begin
  // do not stream properties without getter
  if not Assigned(PPropInfo(PropInfo)^.GetProc) then
    exit;
  // properties without setter are only allowed, if they are subcomponents
  PropType := PPropInfo(PropInfo)^.PropType;
  if not Assigned(PPropInfo(PropInfo)^.SetProc) then begin
    if PropType^.Kind<>tkClass then
      exit;
    ObjValue := TObject(GetObjectProp(Instance, PropInfo));
    if not ObjValue.InheritsFrom(TComponent) or
       not (csSubComponent in TComponent(ObjValue).ComponentStyle) then
      exit;
  end;

  { Check if the ancestor can be used }
  HasAncestor := Assigned(Ancestor) and ((Instance = Root) or
    (Instance.ClassType = Ancestor.ClassType));
  //writeln('TWriter.WriteProperty Name=',PropType^.Name,' Kind=',GetEnumName(TypeInfo(TTypeKind),ord(PropType^.Kind)),' HasAncestor=',HasAncestor);

  case PropType^.Kind of
    tkInteger, tkChar, tkEnumeration, tkSet, tkWChar:
      begin
        Value := GetOrdProp(Instance, PropInfo);
        if HasAncestor then
          DefValue := GetOrdProp(Ancestor, PropInfo)
        else
          DefValue := PPropInfo(PropInfo)^.Default;
        // writeln(PPropInfo(PropInfo)^.Name, ', HasAncestor=', ord(HasAncestor), ', Value=', Value, ', Default=', DefValue);
        if (Value <> DefValue) or (DefValue=longint($80000000)) then
        begin
          Driver.BeginProperty(FPropPath + PPropInfo(PropInfo)^.Name);
          case PropType^.Kind of
            tkInteger:
              begin
                // Check if this integer has a string identifier
                IntToIdentFn := FindIntToIdent(PPropInfo(PropInfo)^.PropType);
                if Assigned(IntToIdentFn) and IntToIdentFn(Value, Ident) then
                  // Integer can be written a human-readable identifier
                  WriteIdent(Ident)
                else
                  // Integer has to be written just as number
                  WriteInteger(Value);
              end;
            tkChar:
              WriteChar(Chr(Value));
            tkWChar:
              WriteWideChar(WideChar(Value));
            tkSet:
              Driver.WriteSet(Value, GetTypeData(PropType)^.CompType);
            tkEnumeration:
              WriteIdent(GetEnumName(PropType, Value));
          end;
          Driver.EndProperty;
        end;
      end;
{$ifndef FPUNONE}
    tkFloat:
      begin
        FloatValue := GetFloatProp(Instance, PropInfo);
        if HasAncestor then
          DefFloatValue := GetFloatProp(Ancestor, PropInfo)
        else
          begin
          DefValue :=PPropInfo(PropInfo)^.Default;
          DefFloatValue:=PSingle(@PPropInfo(PropInfo)^.Default)^;
          end;
        if (FloatValue<>DefFloatValue) or (DefValue=longint($80000000)) then
        begin
          Driver.BeginProperty(FPropPath + PPropInfo(PropInfo)^.Name);
          WriteFloat(FloatValue);
          Driver.EndProperty;
        end;
      end;
{$endif}
    tkMethod:
      begin
        MethodValue := GetMethodProp(Instance, PropInfo);
        if HasAncestor then
          DefMethodValue := GetMethodProp(Ancestor, PropInfo)
        else begin
          DefMethodValue.Data := nil;
          DefMethodValue.Code := nil;
        end;

        Handled:=false;
        if Assigned(OnWriteMethodProperty) then
          OnWriteMethodProperty(Self,Instance,PPropInfo(PropInfo),MethodValue,
            DefMethodValue,Handled);
        if (not Handled) and
          (MethodValue.Code <> DefMethodValue.Code) and
          ((not Assigned(MethodValue.Code)) or
          ((Length(FLookupRoot.MethodName(MethodValue.Code)) > 0))) then
        begin
          Driver.BeginProperty(FPropPath + PPropInfo(PropInfo)^.Name);
          if Assigned(MethodValue.Code) then
            Driver.WriteMethodName(FLookupRoot.MethodName(MethodValue.Code))
          else
            Driver.WriteMethodName('');
          Driver.EndProperty;
        end;
      end;
    tkSString, tkLString, tkAString:
      begin
        StrValue := GetStrProp(Instance, PropInfo);
        if HasAncestor then
          DefStrValue := GetStrProp(Ancestor, PropInfo)
        else
          SetLength(DefStrValue, 0);

        if StrValue <> DefStrValue then
        begin
          Driver.BeginProperty(FPropPath + PPropInfo(PropInfo)^.Name);
          if Assigned(FOnWriteStringProperty) then
            FOnWriteStringProperty(Self,Instance,PropInfo,StrValue);
          WriteString(StrValue);
          Driver.EndProperty;
        end;
      end;
    tkWString:
      begin
        WStrValue := GetWideStrProp(Instance, PropInfo);
        if HasAncestor then
          WDefStrValue := GetWideStrProp(Ancestor, PropInfo)
        else
          SetLength(WDefStrValue, 0);

        if WStrValue <> WDefStrValue then
        begin
          Driver.BeginProperty(FPropPath + PPropInfo(PropInfo)^.Name);
          WriteWideString(WStrValue);
          Driver.EndProperty;
        end;
      end;
    tkUString:
      begin
        UStrValue := GetUnicodeStrProp(Instance, PropInfo);
        if HasAncestor then
          UDefStrValue := GetUnicodeStrProp(Ancestor, PropInfo)
        else
          SetLength(UDefStrValue, 0);

        if UStrValue <> UDefStrValue then
        begin
          Driver.BeginProperty(FPropPath + PPropInfo(PropInfo)^.Name);
          WriteUnicodeString(UStrValue);
          Driver.EndProperty;
        end;
      end;
    tkVariant:
      begin
        { Ensure that a Variant manager is installed }
        if not assigned(VarClearProc) then
          raise EWriteError.Create(SErrNoVariantSupport);

        VarValue := tvardata(GetVariantProp(Instance, PropInfo));
        if HasAncestor then
          DefVarValue := tvardata(GetVariantProp(Ancestor, PropInfo))
        else
          FillChar(DefVarValue,sizeof(DefVarValue),0);

        if (CompareByte(VarValue,DefVarValue,sizeof(VarValue)) <> 0) then
          begin
            Driver.BeginProperty(FPropPath + PPropInfo(PropInfo)^.Name);
            { can't use variant() typecast, pulls in variants unit }
            WriteVariant(pvariant(@VarValue)^);
            Driver.EndProperty;
          end;
      end;
    tkClass:
      begin
        ObjValue := TObject(GetObjectProp(Instance, PropInfo));
        if HasAncestor then
        begin
          AncestorObj := TObject(GetObjectProp(Ancestor, PropInfo));
          if (AncestorObj is TComponent) and
             (ObjValue is TComponent) then
          begin
            //writeln('TWriter.WriteProperty AncestorObj=',TComponent(AncestorObj).Name,' OwnerFit=',TComponent(AncestorObj).Owner = FRootAncestor,' ',TComponent(ObjValue).Name,' OwnerFit=',TComponent(ObjValue).Owner = Root);
<<<<<<< HEAD
            if (AncestorObj<> ObjValue) and
             (TComponent(AncestorObj).Owner = FRootAncestor) and
             (TComponent(ObjValue).Owner = Root) and
             (UpperCase(TComponent(AncestorObj).Name) = UpperCase(TComponent(ObjValue).Name)) then
            begin
              // different components, but with the same name
              // treat it like an override
              AncestorObj := ObjValue;
=======
            if (TComponent(AncestorObj).Owner <> FRootAncestor) or
             (TComponent(ObjValue).Owner <> Root) or
             (UpperCase(TComponent(AncestorObj).Name) <> UpperCase(TComponent(ObjValue).Name)) then
            begin
              AncestorObj := nil;
>>>>>>> origin/fixes_2.4
            end;
          end;
        end else
          AncestorObj := nil;

        if not Assigned(ObjValue) then
          begin
          if ObjValue <> AncestorObj then
            begin
            Driver.BeginProperty(FPropPath + PPropInfo(PropInfo)^.Name);
            Driver.WriteIdent('NIL');
            Driver.EndProperty;
            end
          end
        else if ObjValue.InheritsFrom(TPersistent) then
          begin
          { Subcomponents are streamed the same way as persistents }
          if ObjValue.InheritsFrom(TComponent)
            and ((not (csSubComponent in TComponent(ObjValue).ComponentStyle)) 
                 or ((TComponent(ObjValue).Owner<>Instance) and (TComponent(ObjValue).Owner<>Nil))) then
            begin
            Component := TComponent(ObjValue);
            if (ObjValue <> AncestorObj)
                and not (csTransient in Component.ComponentStyle) then
              begin
<<<<<<< HEAD
<<<<<<< HEAD
              Name:= '';
              C:= Component;
              While (C<>Nil) and (C.Name<>'') do
                begin
                If (Name<>'') Then
                  Name:='.'+Name;
                if C.Owner = LookupRoot then
                  begin
                  Name := C.Name+Name;
                  break;
                  end
                else if C = LookupRoot then
                  begin
                  Name := 'Owner' + Name;
                  break;
                  end;
                Name:=C.Name + Name;
                C:= C.Owner;
                end;
              if (C=nil) and (Component.Owner=nil) then 
                if (Name<>'') then              //foreign root
                  Name:=Name+'.Owner';
=======
=======
>>>>>>> origin/fixes_2_2
              { Determine the correct name of the component this property contains }
              if Component.Owner = LookupRoot then
                Name := Component.Name
              else if Component = LookupRoot then
                Name := 'Owner'
              else if Assigned(Component.Owner) and (Length(Component.Owner.Name) > 0)
                and (Length(Component.Name) > 0) then
                Name := Component.Owner.Name + '.' + Component.Name
              else if Length(Component.Name) > 0 then
                Name := Component.Name + '.Owner'
              else
                SetLength(Name, 0);

>>>>>>> graemeg/fixes_2_2
              if Length(Name) > 0 then
                begin
                Driver.BeginProperty(FPropPath + PPropInfo(PropInfo)^.Name);
                WriteIdent(Name);
                Driver.EndProperty;
                end;  // length Name>0
              end; //(ObjValue <> AncestorObj)
            end // ObjValue.InheritsFrom(TComponent)
<<<<<<< HEAD
<<<<<<< HEAD
          else
            begin
=======
=======
>>>>>>> origin/fixes_2_2
          else if ObjValue.InheritsFrom(TCollection) then
            begin
            if (not HasAncestor) or (not CollectionsEqual(TCollection(ObjValue),
              TCollection(GetObjectProp(Ancestor, PropInfo)),root,rootancestor)) then
              begin
              Driver.BeginProperty(FPropPath + PPropInfo(PropInfo)^.Name);
              SavedPropPath := FPropPath;
              try
                SetLength(FPropPath, 0);
                WriteCollection(TCollection(ObjValue));
              finally
                FPropPath := SavedPropPath;
                Driver.EndProperty;
              end;
              end;
            end // Tcollection
          else
            begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            SavedAncestor := Ancestor;
            SavedPropPath := FPropPath;
            try
              FPropPath := FPropPath + PPropInfo(PropInfo)^.Name + '.';
              if HasAncestor then
                Ancestor := TPersistent(GetObjectProp(Ancestor, PropInfo));
              WriteProperties(TPersistent(ObjValue));
            finally
              Ancestor := SavedAncestor;
              FPropPath := SavedPropPath;
            end;
<<<<<<< HEAD
<<<<<<< HEAD
            if ObjValue.InheritsFrom(TCollection) then
              begin
              if (not HasAncestor) or (not CollectionsEqual(TCollection(ObjValue),
                TCollection(GetObjectProp(Ancestor, PropInfo)),root,rootancestor)) then
                begin
                Driver.BeginProperty(FPropPath + PPropInfo(PropInfo)^.Name);
                SavedPropPath := FPropPath;
                try
                  SetLength(FPropPath, 0);
                  WriteCollection(TCollection(ObjValue));
                finally
                  FPropPath := SavedPropPath;
                  Driver.EndProperty;
                end;
                end;
              end // Tcollection
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            end;
          end; // Inheritsfrom(TPersistent)
      end;
    tkInt64, tkQWord:
      begin
        Int64Value := GetInt64Prop(Instance, PropInfo);
        if HasAncestor then
          DefInt64Value := GetInt64Prop(Ancestor, PropInfo)
        else
          DefInt64Value := 0;
        if Int64Value <> DefInt64Value then
        begin
          Driver.BeginProperty(FPropPath + PPropInfo(PropInfo)^.Name);
          WriteInteger(Int64Value);
          Driver.EndProperty;
        end;
      end;
    tkBool:
      begin
        BoolValue := GetOrdProp(Instance, PropInfo)<>0;
        if HasAncestor then
          DefBoolValue := GetOrdProp(Ancestor, PropInfo)<>0
        else
          begin
          DefBoolValue := PPropInfo(PropInfo)^.Default<>0;
          DefValue:=PPropInfo(PropInfo)^.Default;
          end;
        // writeln(PPropInfo(PropInfo)^.Name, ', HasAncestor=', ord(HasAncestor), ', Value=', Value, ', Default=', DefBoolValue);
        if (BoolValue<>DefBoolValue) or (DefValue=longint($80000000)) then
          begin
          Driver.BeginProperty(FPropPath + PPropInfo(PropInfo)^.Name);
          WriteBoolean(BoolValue);
          Driver.EndProperty;
          end;
      end;
  end;
end;

procedure TWriter.WriteRootComponent(ARoot: TComponent);
begin
  WriteDescendent(ARoot, nil);
end;

procedure TWriter.WriteString(const Value: String);
begin
  Driver.WriteString(Value);
end;

procedure TWriter.WriteWideString(const Value: WideString);
begin
  Driver.WriteWideString(Value);
end;

procedure TWriter.WriteUnicodeString(const Value: UnicodeString);
begin
  Driver.WriteUnicodeString(Value);
end;

