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
{*                       TBinaryObjectReader                                *}
{****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPUNONE}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$IFNDEF FPC_HAS_TYPE_EXTENDED}
function ExtendedToDouble(e : pointer) : double;
var mant : qword;
    exp : smallint;
    sign : boolean;
    d : qword;
begin
  move(pbyte(e)[0],mant,8); //mantissa         : bytes 0..7
  move(pbyte(e)[8],exp,2);  //exponent and sign: bytes 8..9
  mant:=LEtoN(mant);
  exp:=LEtoN(word(exp));
  sign:=(exp and $8000)<>0;
  if sign then exp:=exp and $7FFF;
  case exp of
        0 : mant:=0;  //if denormalized, value is too small for double,
                      //so it's always zero
    $7FFF : exp:=2047 //either infinity or NaN
    else
    begin
      dec(exp,16383-1023);
      if (exp>=-51) and (exp<=0) then //can be denormalized
      begin
        mant:=mant shr (-exp);
        exp:=0;
      end
      else
      if (exp<-51) or (exp>2046) then //exponent too large.
      begin
        Result:=0;
        exit;
      end
      else //normalized value
        mant:=mant shl 1; //hide most significant bit
    end;
  end;
  d:=word(exp);
  d:=d shl 52;

  mant:=mant shr 12;
  d:=d or mant;
  if sign then d:=d or $8000000000000000;
  Result:=pdouble(@d)^;
end;
{$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

function TBinaryObjectReader.ReadWord : word; {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
begin
  Read(Result,2);
  Result:=LEtoN(Result);
end;

function TBinaryObjectReader.ReadDWord : longword; {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
begin
  Read(Result,4);
  Result:=LEtoN(Result);
end;

function TBinaryObjectReader.ReadQWord : qword; {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
begin
  Read(Result,8);
  Result:=LEtoN(Result);
end;

{$IFDEF FPC_DOUBLE_HILO_SWAPPED}
procedure SwapDoubleHiLo(var avalue: double); {$ifdef CLASSESINLINE}inline{$endif CLASSESINLINE}
var dwo1 : dword;
type tdoublerec = array[0..1] of dword;
begin
  dwo1:= tdoublerec(avalue)[0];
  tdoublerec(avalue)[0]:=tdoublerec(avalue)[1];
  tdoublerec(avalue)[1]:=dwo1;
end;
{$ENDIF FPC_DOUBLE_HILO_SWAPPED}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPUNONE}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TBinaryObjectReader.ReadExtended : extended; {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
{$IFNDEF FPC_HAS_TYPE_EXTENDED}
var ext : array[0..9] of byte;
{$ENDIF}
begin
  {$IFNDEF FPC_HAS_TYPE_EXTENDED}
  Read(ext[0],10);
  Result:=ExtendedToDouble(@(ext[0]));
  {$IFDEF FPC_DOUBLE_HILO_SWAPPED}
  SwapDoubleHiLo(result);
  {$ENDIF}
  {$ELSE}
  Read(Result,sizeof(Result));
  {$ENDIF}
end;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

constructor TBinaryObjectReader.Create(Stream: TStream; BufSize: Integer);
begin
  inherited Create;
  If (Stream=Nil) then
    Raise EReadError.Create(SEmptyStreamIllegalReader);
  FStream := Stream;
  FBufSize := BufSize;
  GetMem(FBuffer, BufSize);
end;

destructor TBinaryObjectReader.Destroy;
begin
  { Seek back the amount of bytes that we didn't process until now: }
  FStream.Seek(Integer(FBufPos) - Integer(FBufEnd), soFromCurrent);

  if Assigned(FBuffer) then
    FreeMem(FBuffer, FBufSize);

  inherited Destroy;
end;

function TBinaryObjectReader.ReadValue: TValueType;
var
  b: byte;
begin
  Read(b, 1);
  Result := TValueType(b);
end;

function TBinaryObjectReader.NextValue: TValueType;
begin
  Result := ReadValue;
  { We only 'peek' at the next value, so seek back to unget the read value: }
  Dec(FBufPos);
end;

procedure TBinaryObjectReader.BeginRootComponent;
begin
  { Read filer signature }
<<<<<<< HEAD
  ReadSignature;
=======
  Read(Signature, 4);
<<<<<<< HEAD
<<<<<<< HEAD
  if Signature <> LongInt(unaligned(FilerSignature)) then
=======
  if Signature <> LongInt({$ifdef FPC_SUPPORTS_UNALIGNED}unaligned{$endif}(FilerSignature)) then
>>>>>>> graemeg/fixes_2_2
=======
  if Signature <> LongInt({$ifdef FPC_SUPPORTS_UNALIGNED}unaligned{$endif}(FilerSignature)) then
>>>>>>> origin/fixes_2_2
    raise EReadError.Create(SInvalidImage);
>>>>>>> graemeg/cpstrnew
end;

procedure TBinaryObjectReader.BeginComponent(var Flags: TFilerFlags;
  var AChildPos: Integer; var CompClassName, CompName: String);
var
  Prefix: Byte;
  ValueType: TValueType;
begin
  { Every component can start with a special prefix: }
  Flags := [];
  if (Byte(NextValue) and $f0) = $f0 then
  begin
    Prefix := Byte(ReadValue);
    Flags := TFilerFlags(TFilerFlagsInt(Prefix and $0f));
    if ffChildPos in Flags then
    begin
      ValueType := ReadValue;
      case ValueType of
        vaInt8:
          AChildPos := ReadInt8;
        vaInt16:
          AChildPos := ReadInt16;
        vaInt32:
          AChildPos := ReadInt32;
        else
          raise EReadError.Create(SInvalidPropertyValue);
      end;
    end;
  end;

  CompClassName := ReadStr;
  CompName := ReadStr;
end;

function TBinaryObjectReader.BeginProperty: String;
begin
  Result := ReadStr;
end;

procedure TBinaryObjectReader.ReadBinary(const DestData: TMemoryStream);
var
  BinSize: LongInt;
begin
  BinSize:=LongInt(ReadDWord);
  DestData.Size := BinSize;
  Read(DestData.Memory^, BinSize);
end;

{$ifndef FPUNONE}
function TBinaryObjectReader.ReadFloat: Extended;
begin
  Result:=ReadExtended;
end;

function TBinaryObjectReader.ReadSingle: Single;
var
  r: record
    case byte of
      1: (d: dword);
      2: (s: single);
  end;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  r.d:=ReadDWord;
  Result:=r.s;
=======
  Result:=single(ReadDWord);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=single(ReadDWord);
>>>>>>> origin/fixes_2_2
end;
{$endif}

function TBinaryObjectReader.ReadCurrency: Currency;
var
  r: record
    case byte of
      1: (q: qword);
      2: (c: currency);
  end;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  r.c:=ReadQWord;
  Result:=r.c;
=======
  Result:=currency(ReadQWord);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=currency(ReadQWord);
>>>>>>> origin/fixes_2_2
end;

{$ifndef FPUNONE}
function TBinaryObjectReader.ReadDate: TDateTime;
var
  r: record
    case byte of
      1: (q: qword);
      2: (d: TDateTime);
  end;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  r.q:=ReadQWord;
  Result:=r.d;
=======
  Result:=TDateTime(ReadQWord);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TDateTime(ReadQWord);
>>>>>>> origin/fixes_2_2
end;
{$endif}

function TBinaryObjectReader.ReadIdent(ValueType: TValueType): String;
var
  i: Byte;
begin
  case ValueType of
    vaIdent:
      begin
        Read(i, 1);
        SetLength(Result, i);
        Read(Pointer(@Result[1])^, i);
      end;
    vaNil:
      Result := 'nil';
    vaFalse:
      Result := 'False';
    vaTrue:
      Result := 'True';
    vaNull:
      Result := 'Null';
  end;
end;

function TBinaryObjectReader.ReadInt8: ShortInt;
begin
  Read(Result, 1);
end;

function TBinaryObjectReader.ReadInt16: SmallInt;
begin
  Result:=SmallInt(ReadWord);
end;

function TBinaryObjectReader.ReadInt32: LongInt;
begin
  Result:=LongInt(ReadDWord);
end;

function TBinaryObjectReader.ReadInt64: Int64;
begin
  Result:=Int64(ReadQWord);
end;

function TBinaryObjectReader.ReadSet(EnumType: Pointer): Integer;
type
<<<<<<< HEAD
<<<<<<< HEAD
{$packset 1}
  tset = set of 0..(SizeOf(Integer)*8-1);
{$packset default}
=======
  tset = set of 0..31;
>>>>>>> graemeg/fixes_2_2
=======
  tset = set of 0..31;
>>>>>>> origin/fixes_2_2
var
  Name: String;
  Value: Integer;
begin
  try
    Result := 0;
    while True do
    begin
      Name := ReadStr;
      if Length(Name) = 0 then
        break;
      Value := GetEnumValue(PTypeInfo(EnumType), Name);
      if Value = -1 then
        raise EReadError.Create(SInvalidPropertyValue);
      include(tset(result),Value);
    end;
  except
    SkipSetBody;
    raise;
  end;
end;

procedure TBinaryObjectReader.ReadSignature;
var
  Signature: LongInt;
begin
  Read(Signature, 4);
  if Signature <> LongInt(unaligned(FilerSignature)) then
    raise EReadError.Create(SInvalidImage);
end;

function TBinaryObjectReader.ReadStr: String;
var
  i: Byte;
begin
  Read(i, 1);
  SetLength(Result, i);
  if i > 0 then
    Read(Pointer(@Result[1])^, i);
end;

function TBinaryObjectReader.ReadString(StringType: TValueType): String;
var
  b: Byte;
  i: Integer;
begin
  case StringType of
    vaLString, vaUTF8String:
      i:=ReadDWord;
    else
    //vaString:
      begin
        Read(b, 1);
        i := b;
      end;
<<<<<<< HEAD
=======
    vaLString:
      i:=ReadDWord;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
  SetLength(Result, i);
  if i > 0 then
    Read(Pointer(@Result[1])^, i);
end;

function TBinaryObjectReader.ReadWideString: WideString;
var
  len: DWord;
{$IFDEF ENDIAN_BIG}
  i : integer;
{$ENDIF}
begin
  len := ReadDWord;
  SetLength(Result, len);
  if (len > 0) then
  begin
    Read(Pointer(@Result[1])^, len*2);
    {$IFDEF ENDIAN_BIG}
    for i:=1 to len do
      Result[i]:=widechar(SwapEndian(word(Result[i])));
    {$ENDIF}
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TBinaryObjectReader.ReadUnicodeString: UnicodeString;
var
  len: DWord;
{$IFDEF ENDIAN_BIG}
  i : integer;
{$ENDIF}
begin
  len := ReadDWord;
  SetLength(Result, len);
  if (len > 0) then
  begin
    Read(Pointer(@Result[1])^, len*2);
    {$IFDEF ENDIAN_BIG}
    for i:=1 to len do
      Result[i]:=UnicodeChar(SwapEndian(word(Result[i])));
    {$ENDIF}
  end;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TBinaryObjectReader.SkipComponent(SkipComponentInfos: Boolean);
var
  Flags: TFilerFlags;
  Dummy: Integer;
  CompClassName, CompName: String;
begin
  if SkipComponentInfos then
    { Skip prefix, component class name and component object name }
    BeginComponent(Flags, Dummy, CompClassName, CompName);

  { Skip properties }
  while NextValue <> vaNull do
    SkipProperty;
  ReadValue;

  { Skip children }
  while NextValue <> vaNull do
    SkipComponent(True);
  ReadValue;
end;

procedure TBinaryObjectReader.SkipValue;

  procedure SkipBytes(Count: LongInt);
  var
    Dummy: array[0..1023] of Byte;
    SkipNow: Integer;
  begin
    while Count > 0 do
    begin
      if Count > 1024 then
        SkipNow := 1024
      else
        SkipNow := Count;
      Read(Dummy, SkipNow);
      Dec(Count, SkipNow);
    end;
  end;

var
  Count: LongInt;
begin
  case ReadValue of
    vaNull, vaFalse, vaTrue, vaNil: ;
    vaList:
      begin
        while NextValue <> vaNull do
          SkipValue;
        ReadValue;
      end;
    vaInt8:
      SkipBytes(1);
    vaInt16:
      SkipBytes(2);
    vaInt32:
      SkipBytes(4);
    vaExtended:
      SkipBytes(10);
    vaString, vaIdent:
      ReadStr;
    vaBinary, vaLString:
      begin
        Count:=LongInt(ReadDWord);
        SkipBytes(Count);
      end;
    vaWString:
      begin
        Count:=LongInt(ReadDWord);
<<<<<<< HEAD
<<<<<<< HEAD
        SkipBytes(Count*sizeof(widechar));
      end;
    vaUString:
      begin
        Count:=LongInt(ReadDWord);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        SkipBytes(Count*sizeof(widechar));
      end;
    vaSet:
      SkipSetBody;
    vaCollection:
      begin
        while NextValue <> vaNull do
        begin
          { Skip the order value if present }
          if NextValue in [vaInt8, vaInt16, vaInt32] then
            SkipValue;
          SkipBytes(1);
          while NextValue <> vaNull do
            SkipProperty;
          ReadValue;
        end;
        ReadValue;
      end;
    vaSingle:
{$ifndef FPUNONE}
      SkipBytes(Sizeof(Single));
{$else}
      SkipBytes(4);
{$endif}
    {!!!: vaCurrency:
      SkipBytes(SizeOf(Currency));}
    vaDate, vaInt64:
      SkipBytes(8);
  end;
end;

{ private methods }

procedure TBinaryObjectReader.Read(var Buf; Count: LongInt);
var
  CopyNow: LongInt;
  Dest: Pointer;
begin
  Dest := @Buf;
  while Count > 0 do
  begin
    if FBufPos >= FBufEnd then
    begin
      FBufEnd := FStream.Read(FBuffer^, FBufSize);
      if FBufEnd = 0 then
        raise EReadError.Create(SReadError);
      FBufPos := 0;
    end;
    CopyNow := FBufEnd - FBufPos;
    if CopyNow > Count then
      CopyNow := Count;
    Move(PChar(FBuffer)[FBufPos], Dest^, CopyNow);
    Inc(FBufPos, CopyNow);
    Inc(Dest, CopyNow);
    Dec(Count, CopyNow);
  end;
end;

procedure TBinaryObjectReader.SkipProperty;
begin
  { Skip property name, then the property value }
  ReadStr;
  SkipValue;
end;

procedure TBinaryObjectReader.SkipSetBody;
begin
  while Length(ReadStr) > 0 do;
end;



{****************************************************************************}
{*                             TREADER                                      *}
{****************************************************************************}

type
  TFieldInfo = packed record
    FieldOffset: LongWord;
    ClassTypeIndex: Word;
    Name: ShortString;
  end;

  PFieldClassTable = ^TFieldClassTable;
  TFieldClassTable =
{$ifndef FPC_REQUIRES_PROPER_ALIGNMENT}
  packed
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
  record
    Count: Word;
    Entries: array[{$ifdef cpu16}0..16384 div sizeof(TPersistentClass){$else}Word{$endif}] of TPersistentClass;
  end;

  PFieldTable = ^TFieldTable;
  TFieldTable =
{$ifndef FPC_REQUIRES_PROPER_ALIGNMENT}
  packed
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
  record
    FieldCount: Word;
    ClassTable: PFieldClassTable;
    // Fields: array[Word] of TFieldInfo;  Elements have variant size!
  end;

function GetFieldClass(Instance: TObject; const ClassName: string): TPersistentClass;
var
  ShortClassName: shortstring;
  ClassType: TClass;
  ClassTable: PFieldClassTable;
  i: Integer;
  FieldTable: PFieldTable;
begin
  // At first, try to locate the class in the class tables
  ShortClassName := ClassName;
  ClassType := Instance.ClassType;
  while ClassType <> TPersistent do
  begin
    FieldTable := PFieldTable(PVmt(ClassType)^.vFieldTable);
    if Assigned(FieldTable) then
    begin
      ClassTable := FieldTable^.ClassTable;
      for i := 0 to ClassTable^.Count - 1 do
      begin
        Result := ClassTable^.Entries[i];
        if Result.ClassNameIs(ShortClassName) then
          exit;
      end;
    end;
    // Try again with the parent class type
    ClassType := ClassType.ClassParent;
  end;
  Result := Classes.GetClass(ClassName);
end;


constructor TReader.Create(Stream: TStream; BufSize: Integer);
begin
  inherited Create;
  If (Stream=Nil) then
    Raise EReadError.Create(SEmptyStreamIllegalReader);
  FDriver := CreateDriver(Stream, BufSize);
end;

destructor TReader.Destroy;
begin
  FDriver.Free;
  inherited Destroy;
end;

function TReader.CreateDriver(Stream: TStream; BufSize: Integer): TAbstractObjectReader;
begin
  Result := TBinaryObjectReader.Create(Stream, BufSize);
end;

procedure TReader.BeginReferences;
begin
<<<<<<< HEAD
  FLoaded := TFpList.Create;
=======
  FLoaded := TList.Create;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TReader.CheckValue(Value: TValueType);
begin
  if FDriver.NextValue <> Value then
    raise EReadError.Create(SInvalidPropertyValue)
  else
    FDriver.ReadValue;
end;

procedure TReader.DefineProperty(const Name: String; AReadData: TReaderProc;
  WriteData: TWriterProc; HasData: Boolean);
begin
  if Assigned(AReadData) and (UpperCase(Name) = UpperCase(FPropName)) then
  begin
    AReadData(Self);
    SetLength(FPropName, 0);
  end;
end;

procedure TReader.DefineBinaryProperty(const Name: String;
  AReadData, WriteData: TStreamProc; HasData: Boolean);
var
  MemBuffer: TMemoryStream;
begin
  if Assigned(AReadData) and (UpperCase(Name) = UpperCase(FPropName)) then
  begin
    { Check if the next property really is a binary property}
    if FDriver.NextValue <> vaBinary then
    begin
      FDriver.SkipValue;
      FCanHandleExcepts := True;
      raise EReadError.Create(SInvalidPropertyValue);
    end else
      FDriver.ReadValue;

    MemBuffer := TMemoryStream.Create;
    try
      FDriver.ReadBinary(MemBuffer);
      FCanHandleExcepts := True;
      AReadData(MemBuffer);
    finally
      MemBuffer.Free;
    end;
    SetLength(FPropName, 0);
  end;
end;

function TReader.EndOfList: Boolean;
begin
  Result := FDriver.NextValue = vaNull;
end;

procedure TReader.EndReferences;
begin
  FLoaded.Free;
  FLoaded := nil;
end;

function TReader.Error(const Message: String): Boolean;
begin
  Result := False;
  if Assigned(FOnError) then
    FOnError(Self, Message, Result);
end;

function TReader.FindMethod(ARoot: TComponent; const AMethodName: String): CodePointer;
var
  ErrorResult: Boolean;
begin
  Result := ARoot.MethodAddress(AMethodName);
  ErrorResult := Result = nil;

  { always give the OnFindMethod callback a chance to locate the method }
  if Assigned(FOnFindMethod) then
    FOnFindMethod(Self, AMethodName, Result, ErrorResult);

  if ErrorResult then
    raise EReadError.Create(SInvalidPropertyValue);
end;

procedure TReader.DoFixupReferences;

Var
  R,RN : TLocalUnresolvedReference;
  G : TUnresolvedInstance;
  Ref : String;
  C : TComponent;
  P : integer;
  L : TLinkedList;
  
begin
  If Assigned(FFixups) then
    begin
    L:=TLinkedList(FFixups);
    R:=TLocalUnresolvedReference(L.Root);
    While (R<>Nil) do
      begin
      RN:=TLocalUnresolvedReference(R.Next);
      Ref:=R.FRelative;
      If Assigned(FOnReferenceName) then
        FOnReferenceName(Self,Ref);
      C:=FindNestedComponent(R.FRoot,Ref);
      If Assigned(C) then
        SetObjectProp(R.FInstance,R.FPropInfo,C)
      else
        begin
        P:=Pos('.',R.FRelative);
        If (P<>0) then
          begin
          G:=AddToResolveList(R.FInstance);
          G.Addreference(R.FRoot,R.FPropInfo,Copy(R.FRelative,1,P-1),Copy(R.FRelative,P+1,Length(R.FRelative)-P));
          end;
        end;
      L.RemoveItem(R,True);
      R:=RN;
      end;
    FreeAndNil(FFixups);
    end;
end;

procedure TReader.FixupReferences;
var
  i: Integer;
begin
  DoFixupReferences;
  GlobalFixupReferences;
  for i := 0 to FLoaded.Count - 1 do
    TComponent(FLoaded[I]).Loaded;
end;


function TReader.NextValue: TValueType;
begin
  Result := FDriver.NextValue;
end;

procedure TReader.Read(var Buf; Count: LongInt);
begin
  //This should give an exception if read is not implemented (i.e. TTextObjectReader)
  //but should work with TBinaryObjectReader.
  Driver.Read(Buf, Count);
end;

procedure TReader.PropertyError;
begin
  FDriver.SkipValue;
  raise EReadError.CreateFmt(SUnknownProperty,[FPropName]);
end;

function TReader.ReadBoolean: Boolean;
var
  ValueType: TValueType;
begin
  ValueType := FDriver.ReadValue;
  if ValueType = vaTrue then
    Result := True
  else if ValueType = vaFalse then
    Result := False
  else
    raise EReadError.Create(SInvalidPropertyValue);
end;

function TReader.ReadChar: Char;
var
  s: String;
begin
  s := ReadString;
  if Length(s) = 1 then
    Result := s[1]
  else
    raise EReadError.Create(SInvalidPropertyValue);
end;

function TReader.ReadWideChar: WideChar;

var
  W: WideString;
  
begin
  W := ReadWideString;
  if Length(W) = 1 then
    Result := W[1]
  else
    raise EReadError.Create(SInvalidPropertyValue);
end;
            
function TReader.ReadUnicodeChar: UnicodeChar;

var
  U: UnicodeString;
  
begin
  U := ReadUnicodeString;
  if Length(U) = 1 then
    Result := U[1]
  else
    raise EReadError.Create(SInvalidPropertyValue);
end;
            
procedure TReader.ReadCollection(Collection: TCollection);
var
  Item: TCollectionItem;
begin
  Collection.BeginUpdate;
  if not EndOfList then
    Collection.Clear;
  while not EndOfList do begin
    ReadListBegin;
    Item := Collection.Add;
    while NextValue<>vaNull do
      ReadProperty(Item);
    ReadListEnd;
  end;
  Collection.EndUpdate;
  ReadListEnd;
end;

function TReader.ReadComponent(Component: TComponent): TComponent;
var
  Flags: TFilerFlags;

  function Recover(var Component: TComponent): Boolean;
  begin
    Result := False;
    if ExceptObject.InheritsFrom(Exception) then
    begin
      if not ((ffInherited in Flags) or Assigned(Component)) then
        Component.Free;
      Component := nil;
      FDriver.SkipComponent(False);
      Result := Error(Exception(ExceptObject).Message);
    end;
  end;

var
  CompClassName, Name: String;
  n, ChildPos: Integer;
  SavedParent, SavedLookupRoot: TComponent;
  ComponentClass: TComponentClass;
  C, NewComponent: TComponent;
  SubComponents: TList;
begin
  FDriver.BeginComponent(Flags, ChildPos, CompClassName, Name);
  SavedParent := Parent;
  SavedLookupRoot := FLookupRoot;
  SubComponents := nil;
  try
    Result := Component;
    if not Assigned(Result) then
      try
        if ffInherited in Flags then
        begin
          { Try to locate the existing ancestor component }

          if Assigned(FLookupRoot) then
            Result := FLookupRoot.FindComponent(Name)
          else
            Result := nil;

          if not Assigned(Result) then
          begin
            if Assigned(FOnAncestorNotFound) then
              FOnAncestorNotFound(Self, Name,
                FindComponentClass(CompClassName), Result);
            if not Assigned(Result) then
              raise EReadError.CreateFmt(SAncestorNotFound, [Name]);
          end;

          Parent := Result.GetParentComponent;
          if not Assigned(Parent) then
            Parent := Root;
        end else
        begin
          Result := nil;
          ComponentClass := FindComponentClass(CompClassName);
          if Assigned(FOnCreateComponent) then
            FOnCreateComponent(Self, ComponentClass, Result);
          if not Assigned(Result) then
          begin
            NewComponent := TComponent(ComponentClass.NewInstance);
            if ffInline in Flags then
              NewComponent.FComponentState :=
                NewComponent.FComponentState + [csLoading, csInline];
            NewComponent.Create(Owner);

            { Don't set Result earlier because else we would come in trouble
              with the exception recover mechanism! (Result should be NIL if
              an error occured) }
            Result := NewComponent;
          end;
          Include(Result.FComponentState, csLoading);
        end;
      except
        if not Recover(Result) then
          raise;
      end;

    if Assigned(Result) then
      try
        Include(Result.FComponentState, csLoading);

        { create list of subcomponents and set loading}
        SubComponents := TList.Create;
        for n := 0 to Result.ComponentCount - 1 do
        begin
          C := Result.Components[n];
          if csSubcomponent in C.ComponentStyle
          then begin
            SubComponents.Add(C);
            Include(C.FComponentState, csLoading);
          end;
        end;

        if not (ffInherited in Flags) then
          try
            Result.SetParentComponent(Parent);
            if Assigned(FOnSetName) then
              FOnSetName(Self, Result, Name);
            Result.Name := Name;
            if FindGlobalComponent(Name) = Result then
              Include(Result.FComponentState, csInline);
          except
            if not Recover(Result) then
              raise;
          end;
        if not Assigned(Result) then
          exit;
        if csInline in Result.ComponentState then
          FLookupRoot := Result;

        { Read the component state }
        Include(Result.FComponentState, csReading);
        for n := 0 to Subcomponents.Count - 1 do
          Include(TComponent(Subcomponents[n]).FComponentState, csReading);

        Result.ReadState(Self);

        Exclude(Result.FComponentState, csReading);
        for n := 0 to Subcomponents.Count - 1 do
          Exclude(TComponent(Subcomponents[n]).FComponentState, csReading);

        if ffChildPos in Flags then
          Parent.SetChildOrder(Result, ChildPos);

        { Add component to list of loaded components, if necessary }
        if (not ((ffInherited in Flags) or (csInline in Result.ComponentState))) or
          (FLoaded.IndexOf(Result) < 0)
          then begin
            for n := 0 to Subcomponents.Count - 1 do
              FLoaded.Add(Subcomponents[n]);
            FLoaded.Add(Result);
          end;
      except
        if ((ffInherited in Flags) or Assigned(Component)) then
          Result.Free;
        raise;
      end;
  finally
    Parent := SavedParent;
    FLookupRoot := SavedLookupRoot;
    Subcomponents.Free;
  end;
end;

procedure TReader.ReadData(Instance: TComponent);
var
  SavedOwner, SavedParent: TComponent;
  
begin
  { Read properties }
  while not EndOfList do
    ReadProperty(Instance);
  ReadListEnd;

  { Read children }
  SavedOwner := Owner;
  SavedParent := Parent;
  try
    Owner := Instance.GetChildOwner;
    if not Assigned(Owner) then
      Owner := Root;
    Parent := Instance.GetChildParent;

    while not EndOfList do
      ReadComponent(nil);
    ReadListEnd;
  finally
    Owner := SavedOwner;
    Parent := SavedParent;
  end;

  { Fixup references if necessary (normally only if this is the root) }
  If (Instance=FRoot) then
    DoFixupReferences;
end;

{$ifndef FPUNONE}
function TReader.ReadFloat: Extended;
begin
  if FDriver.NextValue = vaExtended then
  begin
    ReadValue;
    Result := FDriver.ReadFloat
  end else
    Result := ReadInt64;
end;

procedure TReader.ReadSignature;
begin
  FDriver.ReadSignature;
end;

function TReader.ReadSingle: Single;
begin
  if FDriver.NextValue = vaSingle then
  begin
    FDriver.ReadValue;
    Result := FDriver.ReadSingle;
  end else
    Result := ReadInteger;
end;
{$endif}

function TReader.ReadCurrency: Currency;
begin
  if FDriver.NextValue = vaCurrency then
  begin
    FDriver.ReadValue;
    Result := FDriver.ReadCurrency;
  end else
    Result := ReadInteger;
end;

{$ifndef FPUNONE}
function TReader.ReadDate: TDateTime;
begin
  if FDriver.NextValue = vaDate then
  begin
    FDriver.ReadValue;
    Result := FDriver.ReadDate;
  end else
    Result := ReadInteger;
end;
{$endif}

function TReader.ReadIdent: String;
var
  ValueType: TValueType;
begin
  ValueType := FDriver.ReadValue;
  if ValueType in [vaIdent, vaNil, vaFalse, vaTrue, vaNull] then
    Result := FDriver.ReadIdent(ValueType)
  else
    raise EReadError.Create(SInvalidPropertyValue);
end;


function TReader.ReadInteger: LongInt;
begin
  case FDriver.ReadValue of
    vaInt8:
      Result := FDriver.ReadInt8;
    vaInt16:
      Result := FDriver.ReadInt16;
    vaInt32:
      Result := FDriver.ReadInt32;
  else
    raise EReadError.Create(SInvalidPropertyValue);
  end;
end;

function TReader.ReadInt64: Int64;
begin
  if FDriver.NextValue = vaInt64 then
  begin
    FDriver.ReadValue;
    Result := FDriver.ReadInt64;
  end else
    Result := ReadInteger;
end;

function TReader.ReadSet(EnumType: Pointer): Integer; 
begin
  if FDriver.NextValue = vaSet then
    begin
      FDriver.ReadValue;
      Result := FDriver.ReadSet(enumtype);
    end 
  else
    Result := ReadInteger;
end;

procedure TReader.ReadListBegin;
begin
  CheckValue(vaList);
end;

procedure TReader.ReadListEnd;
begin
  CheckValue(vaNull);
end;

function TReader.ReadVariant: variant;
var
  nv: TValueType;
begin
  { Ensure that a Variant manager is installed }
  if not Assigned(VarClearProc) then
    raise EReadError.Create(SErrNoVariantSupport);

  FillChar(Result,sizeof(Result),0);

  nv:=NextValue;
  case nv of
    vaNil:
      begin
        Result:=system.unassigned;
        readvalue;
      end;
    vaNull:
      begin
        Result:=system.null;
        readvalue;
      end;
    { all integer sizes must be split for big endian systems }
    vaInt8,vaInt16,vaInt32:
      begin
        Result:=ReadInteger;
      end;
    vaInt64:
      begin
        Result:=ReadInt64;
      end;
    vaQWord:
      begin
        Result:=QWord(ReadInt64);
      end;
    vaFalse,vaTrue:
      begin
        Result:=(nv<>vaFalse);
        readValue;
      end;
    vaCurrency:
      begin
        Result:=ReadCurrency;
      end;
{$ifndef fpunone}
    vaSingle:
      begin
        Result:=ReadSingle;
      end;
    vaExtended:
      begin
        Result:=ReadFloat;
      end;
    vaDate:
      begin
        Result:=ReadDate;
      end;
{$endif fpunone}
    vaWString,vaUTF8String:
      begin
        Result:=ReadWideString;
      end;
    vaString:
      begin
        Result:=ReadString;
      end;
    vaUString:
      begin
        Result:=ReadUnicodeString;
      end;
    else
      raise EReadError.CreateFmt(SUnsupportedPropertyVariantType, [Ord(nv)]);
  end;
end;

procedure TReader.ReadProperty(AInstance: TPersistent);
var
  Path: String;
  Instance: TPersistent;
  DotPos, NextPos: PChar;
  PropInfo: PPropInfo;
  Obj: TObject;
  Name: String;
  Skip: Boolean;
  Handled: Boolean;
  OldPropName: String;

  function HandleMissingProperty(IsPath: Boolean): boolean;
  begin
    Result:=true;
    if Assigned(OnPropertyNotFound) then begin
      // user defined property error handling
      OldPropName:=FPropName;
      Handled:=false;
      Skip:=false;
      OnPropertyNotFound(Self,Instance,FPropName,IsPath,Handled,Skip);
      if Handled and (not Skip) and (OldPropName<>FPropName) then
        // try alias property
        PropInfo := GetPropInfo(Instance.ClassInfo, FPropName);
      if Skip then begin
        FDriver.SkipValue;
        Result:=false;
        exit;
      end;
    end;
  end;

begin
  try
    Path := FDriver.BeginProperty;
    try
      Instance := AInstance;
      FCanHandleExcepts := True;
      DotPos := PChar(Path);
      while True do
      begin
        NextPos := StrScan(DotPos, '.');
        if Assigned(NextPos) then
          FPropName := Copy(String(DotPos), 1, Integer(NextPos - DotPos))
        else
        begin
          FPropName := DotPos;
          break;
        end;
        DotPos := NextPos + 1;

        PropInfo := GetPropInfo(Instance.ClassInfo, FPropName);
        if not Assigned(PropInfo) then begin
          if not HandleMissingProperty(true) then exit;
          if not Assigned(PropInfo) then
            PropertyError;
        end;

        if PropInfo^.PropType^.Kind = tkClass then
          Obj := TObject(GetObjectProp(Instance, PropInfo))
        else
          Obj := nil;

        if not (Obj is TPersistent) then
        begin
          { All path elements must be persistent objects! }
          FDriver.SkipValue;
          raise EReadError.Create(SInvalidPropertyPath);
        end;
        Instance := TPersistent(Obj);
      end;

      PropInfo := GetPropInfo(Instance.ClassInfo, FPropName);
      if Assigned(PropInfo) then
        ReadPropValue(Instance, PropInfo)
      else
      begin
        FCanHandleExcepts := False;
        Instance.DefineProperties(Self);
        FCanHandleExcepts := True;
        if Length(FPropName) > 0 then begin
          if not HandleMissingProperty(false) then exit;
          if not Assigned(PropInfo) then
            PropertyError;
        end;
      end;
    except
      on e: Exception do
      begin
        SetLength(Name, 0);
        if AInstance.InheritsFrom(TComponent) then
          Name := TComponent(AInstance).Name;
        if Length(Name) = 0 then
          Name := AInstance.ClassName;
        raise EReadError.CreateFmt(SPropertyException,
          [Name, DotSep, Path, e.Message]);
      end;
    end;
  except
    on e: Exception do
      if not FCanHandleExcepts or not Error(E.Message) then
        raise;
  end;
end;

procedure TReader.ReadPropValue(Instance: TPersistent; PropInfo: Pointer);
const
  NullMethod: TMethod = (Code: nil; Data: nil);
var
  PropType: PTypeInfo;
  Value: LongInt;
{  IdentToIntFn: TIdentToInt; }
  Ident: String;
  Method: TMethod;
  Handled: Boolean;
  TmpStr: String;
begin
  if not Assigned(PPropInfo(PropInfo)^.SetProc) then
    raise EReadError.Create(SReadOnlyProperty);

  PropType := PPropInfo(PropInfo)^.PropType;
  case PropType^.Kind of
    tkInteger:
      if FDriver.NextValue = vaIdent then
      begin
        Ident := ReadIdent;
        if GlobalIdentToInt(Ident,Value) then
          SetOrdProp(Instance, PropInfo, Value)
        else
          raise EReadError.Create(SInvalidPropertyValue);
      end else
        SetOrdProp(Instance, PropInfo, ReadInteger);
    tkBool:
      SetOrdProp(Instance, PropInfo, Ord(ReadBoolean));
    tkChar:
      SetOrdProp(Instance, PropInfo, Ord(ReadChar));
    tkWChar,tkUChar:
      SetOrdProp(Instance, PropInfo, Ord(ReadWideChar));  
    tkEnumeration:
      begin
        Value := GetEnumValue(PropType, ReadIdent);
        if Value = -1 then
          raise EReadError.Create(SInvalidPropertyValue);
        SetOrdProp(Instance, PropInfo, Value);
      end;
{$ifndef FPUNONE}
    tkFloat:
      SetFloatProp(Instance, PropInfo, ReadFloat);
{$endif}
    tkSet:
      begin
        CheckValue(vaSet);
        SetOrdProp(Instance, PropInfo,
          FDriver.ReadSet(GetTypeData(PropType)^.CompType));
      end;
    tkMethod:
      if FDriver.NextValue = vaNil then
      begin
        FDriver.ReadValue;
        SetMethodProp(Instance, PropInfo, NullMethod);
      end else
      begin
        Handled:=false;
        Ident:=ReadIdent;
        if Assigned(OnSetMethodProperty) then
          OnSetMethodProperty(Self,Instance,PPropInfo(PropInfo),Ident,
                              Handled);
        if not Handled then begin
          Method.Code := FindMethod(Root, Ident);
          Method.Data := Root;
          if Assigned(Method.Code) then
            SetMethodProp(Instance, PropInfo, Method);
        end;
      end;
    tkSString, tkLString, tkAString:
      begin
        TmpStr:=ReadString;
        if Assigned(FOnReadStringProperty) then
          FOnReadStringProperty(Self,Instance,PropInfo,TmpStr);
        SetStrProp(Instance, PropInfo, TmpStr);
      end;
    tkUstring:
      SetUnicodeStrProp(Instance,PropInfo,ReadUnicodeString);
    tkWString:
      SetWideStrProp(Instance,PropInfo,ReadWideString);
    tkVariant:
      begin
        SetVariantProp(Instance,PropInfo,ReadVariant);
      end;
    tkClass:
      case FDriver.NextValue of
        vaNil:
          begin
            FDriver.ReadValue;
            SetOrdProp(Instance, PropInfo, 0)
          end;
        vaCollection:
          begin
            FDriver.ReadValue;
            ReadCollection(TCollection(GetObjectProp(Instance, PropInfo)));
          end
        else
          begin
          If Not Assigned(FFixups) then
            FFixups:=TLinkedList.Create(TLocalUnresolvedReference);
          With TLocalUnresolvedReference(TLinkedList(FFixups).Add) do
            begin
            FInstance:=Instance;
            FRoot:=Root;
            FPropInfo:=PropInfo;
            FRelative:=ReadIdent;
            end;
          end;
      end;
    tkInt64, tkQWord: SetInt64Prop(Instance, PropInfo, ReadInt64);
    else
      raise EReadError.CreateFmt(SUnknownPropertyType, [Ord(PropType^.Kind)]);
  end;
end;

function TReader.ReadRootComponent(ARoot: TComponent): TComponent;
var
  Dummy, i: Integer;
  Flags: TFilerFlags;
  CompClassName, CompName, ResultName: String;
begin
  FDriver.BeginRootComponent;
  Result := nil;
  {!!!: GlobalNameSpace.BeginWrite;  // Loading from stream adds to name space
  try}
    try
      FDriver.BeginComponent(Flags, Dummy, CompClassName, CompName);
      if not Assigned(ARoot) then
      begin
        { Read the class name and the object name and create a new object: }
        Result := TComponentClass(FindClass(CompClassName)).Create(nil);
        Result.Name := CompName;
      end else
      begin
        Result := ARoot;

        if not (csDesigning in Result.ComponentState) then
        begin
          Result.FComponentState :=
            Result.FComponentState + [csLoading, csReading];

          { We need an unique name }
          i := 0;
          { Don't use Result.Name directly, as this would influence
            FindGlobalComponent in successive loop runs }
          ResultName := CompName;
          while Assigned(FindGlobalComponent(ResultName)) do
          begin
            Inc(i);
            ResultName := CompName + '_' + IntToStr(i);
          end;
          Result.Name := ResultName;
        end;
      end;

      FRoot := Result;
      FLookupRoot := Result;
      if Assigned(GlobalLoaded) then
        FLoaded := GlobalLoaded
      else
        FLoaded := TFpList.Create;

      try
        if FLoaded.IndexOf(FRoot) < 0 then
          FLoaded.Add(FRoot);
        FOwner := FRoot;
        FRoot.FComponentState := FRoot.FComponentState + [csLoading, csReading];
        FRoot.ReadState(Self);
        Exclude(FRoot.FComponentState, csReading);

        if not Assigned(GlobalLoaded) then
          for i := 0 to FLoaded.Count - 1 do
            TComponent(FLoaded[i]).Loaded;

      finally
        if not Assigned(GlobalLoaded) then
          FLoaded.Free;
        FLoaded := nil;
      end;
      GlobalFixupReferences;
    except
      RemoveFixupReferences(ARoot, '');
      if not Assigned(ARoot) then
        Result.Free;
      raise;
    end;
  {finally
    GlobalNameSpace.EndWrite;
  end;}
end;

procedure TReader.ReadComponents(AOwner, AParent: TComponent;
  Proc: TReadComponentsProc);
var
  Component: TComponent;
begin
  Root := AOwner;
  Owner := AOwner;
  Parent := AParent;
  BeginReferences;
  try
    while not EndOfList do
    begin
      FDriver.BeginRootComponent;
      Component := ReadComponent(nil);
      if Assigned(Proc) then
        Proc(Component);
    end;
    ReadListEnd;
    FixupReferences;
  finally
    EndReferences;
  end;
end;


function TReader.ReadString: String;
var
  StringType: TValueType;
begin
  StringType := FDriver.ReadValue;
  if StringType in [vaString, vaLString,vaUTF8String] then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
      Result := FDriver.ReadString(StringType);
      if (StringType=vaUTF8String) then
        Result:=string(utf8Decode(Result));
    end
  else if StringType in [vaWString] then
    Result:= string(FDriver.ReadWidestring)
  else if StringType in [vaUString] then
    Result:= string(FDriver.ReadUnicodeString)
=======
=======
>>>>>>> origin/fixes_2_2
    Result := FDriver.ReadString(StringType);
    if (StringType=vaUTF8String) then
      Result:=utf8Decode(Result);
    end
  else if StringType in [vaWString] then
    Result:= FDriver.ReadWidestring
>>>>>>> graemeg/fixes_2_2
  else
    raise EReadError.Create(SInvalidPropertyValue);
end;


function TReader.ReadWideString: WideString;
var
 s: String;
 i: Integer;
 vt:TValueType;
begin
<<<<<<< HEAD
  if NextValue in [vaWString,vaUString,vaUTF8String] then
    //vaUTF8String needs conversion? 2008-09-06 mse, YES!! AntonK
    begin
      vt:=ReadValue;
      if vt=vaUTF8String then
        Result := utf8decode(fDriver.ReadString(vaLString))
      else
        Result := FDriver.ReadWideString
    end
  else
    begin
      //data probable from ObjectTextToBinary
      s := ReadString;
      setlength(result,length(s));
      for i:= 1 to length(s) do begin
        result[i]:= widechar(ord(s[i])); //no code conversion
=======
  if NextValue in [vaWString,vaUTF8String] then
  begin
    ReadValue;
    Result := FDriver.ReadWideString
  end
  else begin
    //data probable from ObjectTextToBinary
    s := ReadString;
    setlength(result,length(s));
    for i:= 1 to length(s) do begin
     result[i]:= widechar(ord(s[i])); //no code conversion
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  end;
end;


function TReader.ReadUnicodeString: UnicodeString;
var
 s: String;
 i: Integer;
 vt:TValueType;
begin
  if NextValue in [vaWString,vaUString,vaUTF8String] then
    //vaUTF8String needs conversion? 2008-09-06 mse, YES!! AntonK
    begin
      vt:=ReadValue;
      if vt=vaUTF8String then
        Result := utf8decode(fDriver.ReadString(vaLString))
      else
        Result := FDriver.ReadWideString
    end
  else
    begin
      //data probable from ObjectTextToBinary
      s := ReadString;
      setlength(result,length(s));
      for i:= 1 to length(s) do begin
        result[i]:= UnicodeChar(ord(s[i])); //no code conversion
    end;
  end;
end;


function TReader.ReadValue: TValueType;
begin
  Result := FDriver.ReadValue;
end;

procedure TReader.CopyValue(Writer: TWriter);

  procedure CopyBytes(Count: Integer);
{  var
    Buffer: array[0..1023] of Byte; }
  begin
{!!!:    while Count > 1024 do
    begin
      FDriver.Read(Buffer, 1024);
      Writer.Driver.Write(Buffer, 1024);
      Dec(Count, 1024);
    end;
    if Count > 0 then
    begin
      FDriver.Read(Buffer, Count);
      Writer.Driver.Write(Buffer, Count);
    end;}
  end;

{var
  s: String;
  Count: LongInt; }
begin
  case FDriver.NextValue of
    vaNull:
      Writer.WriteIdent('NULL');
    vaFalse:
      Writer.WriteIdent('FALSE');
    vaTrue:
      Writer.WriteIdent('TRUE');
    vaNil:
      Writer.WriteIdent('NIL');
    {!!!: vaList, vaCollection:
      begin
        Writer.WriteValue(FDriver.ReadValue);
        while not EndOfList do
          CopyValue(Writer);
        ReadListEnd;
        Writer.WriteListEnd;
      end;}
    vaInt8, vaInt16, vaInt32:
      Writer.WriteInteger(ReadInteger);
{$ifndef FPUNONE}
    vaExtended:
      Writer.WriteFloat(ReadFloat);
{$endif}
    {!!!: vaString:
      Writer.WriteStr(ReadStr);}
    vaIdent:
      Writer.WriteIdent(ReadIdent);
    {!!!: vaBinary, vaLString, vaWString:
      begin
        Writer.WriteValue(FDriver.ReadValue);
        FDriver.Read(Count, SizeOf(Count));
        Writer.Driver.Write(Count, SizeOf(Count));
        CopyBytes(Count);
      end;}
    {!!!: vaSet:
      Writer.WriteSet(ReadSet);}
{$ifndef FPUNONE}
    vaSingle:
      Writer.WriteSingle(ReadSingle);
{$endif}
    {!!!: vaCurrency:
      Writer.WriteCurrency(ReadCurrency);}
{$ifndef FPUNONE}
    vaDate:
      Writer.WriteDate(ReadDate);
{$endif}
    vaInt64:
      Writer.WriteInteger(ReadInt64);
  end;
end;

function TReader.FindComponentClass(const AClassName: String): TComponentClass;

var
  PersistentClass: TPersistentClass;
<<<<<<< HEAD
<<<<<<< HEAD
  ShortClassName: shortstring;

  procedure FindInFieldTable(RootComponent: TComponent);
  var
    FieldTable: PFieldTable;
=======
=======
>>>>>>> origin/fixes_2_2
  UClassName: shortstring;

  procedure FindInFieldTable(RootComponent: TComponent);
  var
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FieldClassTable: PFieldClassTable;
    Entry: TPersistentClass;
    i: Integer;
    ComponentClassType: TClass;
  begin
    ComponentClassType := RootComponent.ClassType;
    // it is not necessary to look in the FieldTable of TComponent,
    // because TComponent doesn't have published properties that are
    // descendants of TComponent
<<<<<<< HEAD
<<<<<<< HEAD
    while ComponentClassType<>TComponent do
    begin
      FieldTable:=PVmt(ComponentClassType)^.vFieldTable;
      if assigned(FieldTable) then
      begin
        FieldClassTable := FieldTable^.ClassTable;
        for i := 0 to FieldClassTable^.Count -1 do
        begin
          Entry := FieldClassTable^.Entries[i];
          //writeln(format('Looking for %s in field table of class %s. Found %s',
            //[AClassName, ComponentClassType.ClassName, Entry.ClassName]));
          if Entry.ClassNameIs(ShortClassName) and
            (Entry.InheritsFrom(TComponent)) then
          begin
=======
=======
>>>>>>> origin/fixes_2_2
    while ComponentClassType<>TComponent do begin
      FieldClassTable :=
        PFieldTable((Pointer(ComponentClassType)+vmtFieldTable)^)^.ClassTable;
      if assigned(FieldClassTable) then begin
        for i := 0 to FieldClassTable^.Count -1 do begin
          Entry := FieldClassTable^.Entries[i];
          //writeln(format('Looking for %s in field table of class %s. Found %s',
            //[AClassName, ComponentClassType.ClassName, Entry.ClassName]));
          if (UpperCase(Entry.ClassName)=UClassName) and
            (Entry.InheritsFrom(TComponent)) then begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            Result := TComponentClass(Entry);
            Exit;
          end;
        end;
      end;
      // look in parent class
      ComponentClassType := ComponentClassType.ClassParent;
    end;
  end;
  
begin
  Result := nil;
<<<<<<< HEAD
<<<<<<< HEAD
  ShortClassName:=AClassName;
=======
  UClassName:=UpperCase(AClassName);
>>>>>>> graemeg/fixes_2_2
=======
  UClassName:=UpperCase(AClassName);
>>>>>>> origin/fixes_2_2
  FindInFieldTable(Root);
  
  if (Result=nil) and assigned(LookupRoot) and (LookupRoot<>Root) then
    FindInFieldTable(LookupRoot);

  if (Result=nil) then begin
    PersistentClass := GetClass(AClassName);
    if PersistentClass.InheritsFrom(TComponent) then
      Result := TComponentClass(PersistentClass);
  end;
    
  if (Result=nil) and assigned(OnFindComponentClass) then
    OnFindComponentClass(Self, AClassName, Result);

  if (Result=nil) or (not Result.InheritsFrom(TComponent)) then
    raise EClassNotFound.CreateFmt(SClassNotFound, [AClassName]);
end;


