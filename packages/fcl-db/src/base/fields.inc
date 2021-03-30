{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2014 by Michael Van Canneyt, member of the
    Free Pascal development team

    TFields and related components implementations.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{Procedure DumpMem (P : Pointer;Size : Longint);

var i : longint;

begin
  Write ('Memory dump : ');
  For I:=0 to Size-1 do
    Write (Pbyte(P)[i],' ');
  Writeln;
end;}

{ ---------------------------------------------------------------------
    TFieldDef
  ---------------------------------------------------------------------}

constructor TFieldDef.Create(ACollection: TCollection);

begin
  Inherited Create(ACollection);
  FFieldNo:=Index+1;
end;

constructor TFieldDef.Create(AOwner: TFieldDefs; const AName: string;
  ADataType: TFieldType; ASize: Integer; ARequired: Boolean; AFieldNo: Longint;
  ACodePage: TSystemCodePage);

begin
{$ifdef dsdebug }
  Writeln('TFieldDef.Create : ',Aname,'(',AFieldNo,')');
{$endif}
  Inherited Create(AOwner);
  Name:=Aname;
  FDatatype:=ADatatype;
  FSize:=ASize;
  FRequired:=ARequired;
  FPrecision:=-1;
  FFieldNo:=AFieldNo;
  case FDataType of
    ftString, ftFixedChar, ftMemo:
      FCodePage := ACodePage;
    ftWideString, ftFixedWideChar, ftWideMemo:
      FCodePage := CP_UTF16;
    else
      FCodePage := 0;
  end;
end;

destructor TFieldDef.Destroy;

begin
  Inherited destroy;
end;

procedure TFieldDef.Assign(APersistent: TPersistent);
var fd: TFieldDef;
begin
  fd := nil;
  if APersistent is TFieldDef then
    fd := APersistent as TFieldDef;
  if Assigned(fd) then begin
    Collection.BeginUpdate;
    try
      Name := fd.Name;
      DataType := fd.DataType;
      Size := fd.Size;
      Precision := fd.Precision;
      FRequired := fd.Required;
      FCodePage := fd.FCodePage;
    finally
      Collection.EndUpdate;
    end;
  end
  else
    inherited Assign(APersistent);
end;

function TFieldDef.CreateField(AOwner: TComponent): TField;

var TheField : TFieldClass;

begin
{$ifdef dsdebug}
  Writeln ('Creating field '+FNAME);
{$endif dsdebug}
  TheField:=GetFieldClass;
  if TheField=Nil then
    DatabaseErrorFmt(SUnknownFieldType,[FName]);
  Result:=TheField.Create(AOwner);
  Try
    Result.FFieldDef:=Self;
    Result.Size:=FSize;
    Result.Required:=FRequired;
    Result.FFieldName:=FName;
    Result.FDisplayLabel:=DisplayName;
    Result.FFieldNo:=Self.FieldNo;
    Result.SetFieldType(DataType);
    Result.FReadOnly:=(faReadOnly in Attributes);
{$ifdef dsdebug}
    Writeln ('TFieldDef.CreateField : Result Fieldno : ',Result.FieldNo,'; Self : ',FieldNo);
    Writeln ('TFieldDef.CreateField : Trying to set dataset');
{$endif dsdebug}
    Result.Dataset:=TFieldDefs(Collection).Dataset;
    if (Result is TStringField) then
      TStringField(Result).FCodePage := FCodePage
    else if (Result is TMemoField) then
      TMemoField(Result).FCodePage := FCodePage
    else if (Result is TFloatField) then
      TFloatField(Result).Precision := FPrecision
    else if (Result is TBCDField) then
      TBCDField(Result).Precision := FPrecision
    else if (Result is TFmtBCDField) then
      TFmtBCDField(Result).Precision := FPrecision;
  except
    Result.Free;
    Raise;
  end;
end;

procedure TFieldDef.SetAttributes(AValue: TFieldAttributes);
begin
  FAttributes := AValue;
  Changed(False);
end;

procedure TFieldDef.SetDataType(AValue: TFieldType);
begin
  FDataType := AValue;
  Changed(False);
end;

procedure TFieldDef.SetPrecision(const AValue: Longint);
begin
  FPrecision := AValue;
  Changed(False);
end;

procedure TFieldDef.SetSize(const AValue: Integer);
begin
  FSize := AValue;
  Changed(False);
end;

procedure TFieldDef.SetRequired(const AValue: Boolean);
begin
  FRequired := AValue;
  Changed(False);
end;

function TFieldDef.GetFieldClass: TFieldClass;

begin
  //!! Should be owner as tdataset but that doesn't work ??

  If Assigned(Collection) And
     (Collection is TFieldDefs) And
     Assigned(TFieldDefs(Collection).Dataset) then
    Result:=TFieldDefs(Collection).Dataset.GetFieldClass(FDataType)
  else
    Result:=Nil;
end;

function TFieldDef.GetCharSize: Word;
begin
  case FDataType of
   ftGuid:
     Result := 1;
   ftString, ftFixedChar:
     case FCodePage of
       CP_UTF8: Result := 4;
       else     Result := 1;
     end;
   ftWideString, ftFixedWideChar:
     Result := 2;
   else
     Result := 0;
  end;
end;

{ ---------------------------------------------------------------------
    TFieldDefs
  ---------------------------------------------------------------------}

{
destructor TFieldDefs.Destroy;

begin
  FItems.Free;
  // This will destroy all fielddefs since we own them...
  Inherited Destroy;
end;
}

procedure TFieldDefs.Add(const AName: string; ADataType: TFieldType);

begin
  Add(AName,ADatatype,0,False);
end;

procedure TFieldDefs.Add(const AName: string; ADataType: TFieldType; ASize : Word);

begin
  Add(AName,ADatatype,ASize,False);
end;

procedure TFieldDefs.Add(const AName: string; ADataType: TFieldType; ASize: Word;
  ARequired: Boolean);

begin
  If Length(AName)=0 Then
    DatabaseError(SNeedFieldName);
  // the fielddef will register itself here as an owned component.
  // fieldno is 1 based !
  BeginUpdate;
  try
    Add(AName,ADataType,ASize,ARequired,Count+1);
  finally
    EndUpdate;
  end;
end;

function TFieldDefs.GetItem(Index: Longint): TFieldDef;

begin
  Result := TFieldDef(inherited Items[Index]);
end;

procedure TFieldDefs.SetItem(Index: Longint; const AValue: TFieldDef);
begin
  inherited Items[Index] := AValue;
end;

class function TFieldDefs.FieldDefClass: TFieldDefClass;
begin
  Result:=TFieldDef;
end;

constructor TFieldDefs.Create(ADataSet: TDataSet);
begin
  Inherited Create(ADataset, Owner, FieldDefClass);
end;

function TFieldDefs.Add(const AName: string; ADataType: TFieldType; ASize, APrecision: Integer;
  ARequired, AReadOnly: Boolean; AFieldNo: Integer; ACodePage: TSystemCodePage): TFieldDef;
begin
  Result:=FieldDefClass.Create(Self, MakeNameUnique(AName), ADataType, ASize, ARequired, AFieldNo, ACodePage);
  case ADataType of
    ftBCD, ftFmtBCD:
      Result.Precision := APrecision;
  end;
  if AReadOnly then
    Result.Attributes := Result.Attributes + [faReadOnly];
end;

function TFieldDefs.Add(const AName: string; ADataType: TFieldType; ASize: Word; ARequired: Boolean; AFieldNo: Integer): TFieldDef;
begin
  Result:=FieldDefClass.Create(Self,AName,ADataType,ASize,ARequired,AFieldNo);
end;

procedure TFieldDefs.Assign(FieldDefs: TFieldDefs);

var I : longint;

begin
  Clear;
  For i:=0 to FieldDefs.Count-1 do
    With FieldDefs[i] do
      Add(Name,DataType,Size,Required);
end;

function TFieldDefs.Find(const AName: string): TFieldDef;
begin
  Result := (Inherited Find(AName)) as TFieldDef;
  if Result=nil then DatabaseErrorFmt(SFieldNotFound,[AName],FDataset);
end;

{
procedure TFieldDefs.Clear;

var I : longint;

begin
  For I:=FItems.Count-1 downto 0 do
    TFieldDef(Fitems[i]).Free;
  FItems.Clear;
end;
}

procedure TFieldDefs.Update;

begin
  if not Updated then
    begin
    If Assigned(Dataset) then
      DataSet.InitFieldDefs;
    Updated := True;
    end;
end;

function TFieldDefs.MakeNameUnique(const AName: String): string;
var DblFieldCount : integer;
begin
  DblFieldCount := 0;
  Result := AName;
  while assigned(inherited Find(Result)) do
    begin
    inc(DblFieldCount);
    Result := AName + '_' + IntToStr(DblFieldCount);
    end;
end;

function TFieldDefs.AddFieldDef: TFieldDef;

begin
  Result:=FieldDefClass.Create(Self,'',ftUnknown,0,False,Count+1);
end;

{ ---------------------------------------------------------------------
    TField
  ---------------------------------------------------------------------}

Const
  SBCD = 'BCD';
  SBoolean = 'Boolean';
  SDateTime = 'TDateTime';
  SFloat = 'Float';
  SInteger = 'Integer';
  SLargeInt = 'LargeInt';
  SLongWord = 'LongWord';
  SVariant = 'Variant';
  SString = 'String';
  SBytes = 'Bytes';

constructor TField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  FVisible:=True;
  FValidChars:=[#0..#255];

  FProviderFlags := [pfInUpdate,pfInWhere];
end;

destructor TField.Destroy;

begin
  IF Assigned(FDataSet) then
    begin
    FDataSet.Active:=False;
    if Assigned(FFields) then
      FFields.Remove(Self);
    end;
  FLookupList.Free;
  Inherited Destroy;
end;

function TField.AccessError(const TypeName: string): EDatabaseError;

begin
  Result:=EDatabaseError.CreateFmt(SInvalidTypeConversion,[TypeName,FFieldName]);
end;

procedure TField.Assign(Source: TPersistent);

begin
  if Source = nil then Clear
  else if Source is TField then begin
    Value := TField(Source).Value;
  end else
    inherited Assign(Source);
end;

procedure TField.AssignValue(const AValue: TVarRec);
  procedure Error;
  begin
    DatabaseErrorFmt(SFieldValueError, [DisplayName]);
  end;

begin
  with AValue do
    case VType of
      vtInteger:
        AsInteger := VInteger;
      vtBoolean:
        AsBoolean := VBoolean;
      vtChar:
        AsString := VChar;
      vtExtended:
        AsFloat := VExtended^;
      vtString:
        AsString := VString^;
      vtPointer:
        if VPointer <> nil then Error;
      vtPChar:
        AsString := VPChar;
      vtObject:
        if (VObject = nil) or (VObject is TPersistent) then
          Assign(TPersistent(VObject))
        else
          Error;
      vtCurrency:
        AsCurrency := VCurrency^;
      vtVariant:
        if not VarIsClear(VVariant^) then Self.Value := VVariant^;
      vtAnsiString:
        AsAnsiString := AnsiString(VAnsiString);
      vtUnicodeString:
        AsUnicodeString := UnicodeString(VUnicodeString);
      vtWideString:
        AsWideString := WideString(VWideString);
      vtInt64:
        AsLargeInt := VInt64^;
    else
      Error;
    end;
end;

procedure TField.Bind(Binding: Boolean);

begin
  if Binding and (FieldKind=fkLookup) then
    begin
    if ((FLookupDataSet = nil) or (FLookupKeyFields = '') or
       (FLookupResultField = '') or (FKeyFields = '')) then
      DatabaseErrorFmt(SLookupInfoError, [DisplayName]);
    FFields.CheckFieldNames(FKeyFields);
    FLookupDataSet.Open;
    FLookupDataSet.Fields.CheckFieldNames(FLookupKeyFields);
    FLookupDataSet.FieldByName(FLookupResultField);
    if FLookupCache then
      RefreshLookupList;
    end;
end;

procedure TField.Change;

begin
  If Assigned(FOnChange) Then
    FOnChange(Self);
end;

procedure TField.CheckInactive;

begin
  If Assigned(FDataSet) then
    FDataset.CheckInactive;
end;

procedure TField.Clear;

begin
  SetData(Nil);
end;

procedure TField.DataChanged;

begin
  FDataset.DataEvent(deFieldChange,ptrint(Self));
end;

procedure TField.FocusControl;
var
  Field1: TField;
begin
  Field1 := Self;
  FDataSet.DataEvent(deFocusControl,ptrint(@Field1));
end;

procedure TField.FreeBuffers;

begin
  // Empty. Provided for backward compatibiliy;
  // TDataset manages the buffers.
end;

function TField.GetAsBCD: TBCD;
begin
  raise AccessError(SBCD);
end;

function TField.GetAsBoolean: Boolean;
begin
  raise AccessError(SBoolean);
end;

function TField.GetAsBytes: TBytes;
begin
  Result:=Default(TBytes);
//  Writeln('Allocating ',Datasize,' bytes');
  SetLength(Result, DataSize);
  if assigned(result) and not GetData(@Result[0], False) then
    Result := nil;
end;

function TField.GetAsCurrency: Currency;
begin
  Result := GetAsFloat;
end;

function TField.GetAsDateTime: TDateTime;

begin
  raise AccessError(SdateTime);
end;

function TField.GetAsFloat: Double;

begin
  raise AccessError(SDateTime);
end;

function TField.GetAsLargeInt: Largeint;
begin
  Raise AccessError(SLargeInt);
end;

function TField.GetAsLongint: Longint;

begin
  Result:=GetAsInteger;
end;

function TField.GetAsLongWord: LongWord;
begin
  raise AccessError(SLongWord);
end;

function TField.GetAsInteger: Longint;

begin
  raise AccessError(SInteger);
end;

function TField.GetAsVariant: variant;

begin
  raise AccessError(SVariant);
end;


function TField.GetAsString: string;
begin
  Result := GetClassDesc
end;

function TField.GetAsAnsiString: AnsiString;
begin
  Result := GetAsString;
end;

function TField.GetAsUnicodeString: UnicodeString;
begin
  Result := GetAsString;
end;

function TField.GetAsUTF8String: UTF8String;
begin
  Result := GetAsString;
end;

function TField.GetAsWideString: WideString;
begin
  Result := GetAsUnicodeString;
end;

function TField.GetOldValue: variant;

var SaveState : TDatasetState;

begin
  SaveState := FDataset.State;
  try
    FDataset.SetTempState(dsOldValue);
    Result := GetAsVariant;
  finally
    FDataset.RestoreState(SaveState);
  end;
end;

function TField.GetNewValue: Variant;

var SaveState : TDatasetState;

begin
  SaveState := FDataset.State;
  try
    FDataset.SetTempState(dsNewValue);
    Result := GetAsVariant;
  finally
    FDataset.RestoreState(SaveState);
  end;
end;

procedure TField.SetNewValue(const AValue: Variant);

var SaveState : TDatasetState;

begin
  SaveState := FDataset.State;
  try
    FDataset.SetTempState(dsNewValue);
    SetAsVariant(AValue);
  finally
    FDataset.RestoreState(SaveState);
  end;
end;

function TField.GetCurValue: Variant;

var SaveState : TDatasetState;

begin
  SaveState := FDataset.State;
  try
    FDataset.SetTempState(dsCurValue);
    Result := GetAsVariant;
  finally
    FDataset.RestoreState(SaveState);
  end;
end;

function TField.GetCanModify: Boolean;

begin
  Result:=Not ReadOnly;
  If Result then
    begin
    Result := FieldKind in [fkData, fkInternalCalc];
    if Result then
      begin
      Result:=Assigned(DataSet) and Dataset.Active;
      If Result then
        Result:= DataSet.CanModify;
      end;
    end;
end;

function TField.GetClassDesc: String;
var ClassN : string;
begin
  ClassN := copy(ClassName,2,pos('Field',ClassName)-2);
  if isNull then
    result := '(' + LowerCase(ClassN) + ')'
   else
    result := '(' + UpperCase(ClassN) + ')';
end;

function TField.GetData(Buffer: Pointer): Boolean;

begin
  Result:=GetData(Buffer,True);
end;

function TField.GetData(Buffer: Pointer; NativeFormat : Boolean): Boolean;

begin
  IF FDataset=Nil then
    DatabaseErrorFmt(SNoDataset,[FieldName]);
  If FValidating then
    begin
    result:=assigned(FValueBuffer);
    If Result and assigned(Buffer) then
      Move (FValueBuffer^,Buffer^ ,DataSize);
    end
  else
    Result:=FDataset.GetFieldData(Self,Buffer,NativeFormat);
end;

function TField.GetDataSize: Integer;

begin
  Result:=0;
end;

function TField.GetDefaultWidth: Longint;

begin
  Result:=10;
end;

function TField.GetDisplayName  : String;

begin
  If FDisplayLabel<>'' then
    result:=FDisplayLabel
  else
    Result:=FFieldName;
end;

function TField.IsDisplayLabelStored: Boolean;

begin
  Result:=(DisplayLabel<>FieldName);
end;

function TField.IsDisplayWidthStored: Boolean;

begin
  Result:=(FDisplayWidth<>0);
end;

function TField.GetLookupList: TLookupList;
begin
  if not Assigned(FLookupList) then
    FLookupList := TLookupList.Create;
  Result := FLookupList;
end;

procedure TField.CalcLookupValue;
begin
  if FLookupCache then
    Value := LookupList.ValueOfKey(FDataSet.FieldValues[FKeyFields])
  else if Assigned(FLookupDataSet) and FDataSet.Active then
    Value := FLookupDataSet.Lookup(FLookupKeyfields, FDataSet.FieldValues[FKeyFields], FLookupresultField);
end;

function TField.GetIndex: longint;

begin
  If Assigned(FDataset) then
    Result:=FDataset.FFieldList.IndexOf(Self)
  else
    Result:=-1;
end;

function TField.GetLookup: Boolean;
begin
  Result := FieldKind = fkLookup;
end;

procedure TField.SetAlignment(const AValue: TAlignMent);
begin
  if FAlignment <> AValue then
    begin
    FAlignment := AValue;
    PropertyChanged(false);
    end;
end;

procedure TField.SetIndex(const AValue: Longint);
begin
  if FFields <> nil then FFields.SetFieldIndex(Self, AValue)
end;

procedure TField.SetAsCurrency(AValue: Currency);
begin
  SetAsFloat(AValue);
end;

function TField.GetIsNull: Boolean;

begin
  Result:=Not(GetData (Nil));
end;

function TField.GetParentComponent: TComponent;

begin
  Result := DataSet;
end;

procedure TField.GetText(var AText: string; ADisplayText: Boolean);

begin
  AText:=GetAsString;
end;

function TField.HasParent: Boolean;

begin
  HasParent:=True;
end;

function TField.IsValidChar(InputChar: Char): Boolean;

begin
  // FValidChars must be set in Create.
  Result:=InputChar in FValidChars;
end;

procedure TField.RefreshLookupList;
var
  tmpActive: Boolean;
begin
  if not Assigned(FLookupDataSet) or (Length(FLookupKeyfields) = 0)
  or (Length(FLookupresultField) = 0) or (Length(FKeyFields) = 0) then
    Exit;
    
  tmpActive := FLookupDataSet.Active;
  try
    FLookupDataSet.Active := True;
    FFields.CheckFieldNames(FKeyFields);
    FLookupDataSet.Fields.CheckFieldNames(FLookupKeyFields);
    FLookupDataset.FieldByName(FLookupResultField); // I presume that if it doesn't exist it throws exception, and that a field with null value is still valid
    LookupList.Clear; // have to be F-less because we might be creating it here with getter!

    FLookupDataSet.DisableControls;
    try
      FLookupDataSet.First;
      while not FLookupDataSet.Eof do
      begin
        FLookupList.Add(FLookupDataSet.FieldValues[FLookupKeyfields], FLookupDataSet.FieldValues[FLookupResultField]);
        FLookupDataSet.Next;
      end;
    finally
      FLookupDataSet.EnableControls;
    end;
  finally
    FLookupDataSet.Active := tmpActive;
  end;
end;

procedure TField.Notification(AComponent: TComponent; Operation: TOperation);

begin
  Inherited Notification(AComponent,Operation);
  if (Operation = opRemove) and (AComponent = FLookupDataSet) then
    FLookupDataSet := nil;
end;

procedure TField.PropertyChanged(LayoutAffected: Boolean);

begin
  If (FDataset<>Nil) and (FDataset.Active) then
    If LayoutAffected then
      FDataset.DataEvent(deLayoutChange,0)
    else
      FDataset.DataEvent(deDatasetchange,0);
end;

procedure TField.ReadState(Reader: TReader);

begin
  inherited ReadState(Reader);
  if Reader.Parent is TDataSet then
    DataSet := TDataSet(Reader.Parent);
end;

procedure TField.SetAsBCD(const AValue: TBCD);
begin
  Raise AccessError(SBCD);
end;

procedure TField.SetAsBytes(const AValue: TBytes);
begin
  raise AccessError(SBytes);
end;

procedure TField.SetAsBoolean(AValue: Boolean);

begin
  Raise AccessError(SBoolean);
end;

procedure TField.SetAsDateTime(AValue: TDateTime);

begin
  Raise AccessError(SDateTime);
end;

procedure TField.SetAsFloat(AValue: Double);

begin
  Raise AccessError(SFloat);
end;

procedure TField.SetAsVariant(const AValue: variant);

begin
  if VarIsNull(AValue) then
    Clear
  else
    try
      SetVarValue(AValue);
    except
      on EVariantError do
        DatabaseErrorFmt(SFieldError+SInvalidVariant, [DisplayName]);
    end;
end;


procedure TField.SetAsLongint(AValue: Longint);
begin
  SetAsInteger(AValue);
end;

procedure TField.SetAsLongWord(AValue: LongWord);
begin
  raise AccessError(SLongWord);
end;

procedure TField.SetAsInteger(AValue: Longint);
begin
  raise AccessError(SInteger);
end;

procedure TField.SetAsLargeInt(AValue: Largeint);
begin
  Raise AccessError(SLargeInt);
end;

procedure TField.SetAsString(const AValue: string);
begin
  Raise AccessError(SString);
end;

procedure TField.SetAsAnsiString(const AValue: AnsiString);
begin
  SetAsString(AValue);
end;

procedure TField.SetAsUnicodeString(const AValue: UnicodeString);
begin
  SetAsString(AValue);
end;

procedure TField.SetAsUTF8String(const AValue: UTF8String);
begin
  SetAsString(AValue);
end;

procedure TField.SetAsWideString(const AValue: WideString);
begin
  SetAsUnicodeString(AValue);
end;


procedure TField.SetData(Buffer: Pointer);

begin
 SetData(Buffer,True);
end;

procedure TField.SetData(Buffer: Pointer; NativeFormat : Boolean);

begin
  If Not Assigned(FDataset) then
    DatabaseErrorFmt(SNoDataset,[DisplayName]);
  FDataSet.SetFieldData(Self,Buffer, NativeFormat);
end;

procedure TField.SetDataset(AValue: TDataset);

begin
{$ifdef dsdebug}
  Writeln ('Setting dataset');
{$endif}
  If AValue=FDataset then exit;
  If Assigned(FDataset) Then
    begin
    FDataset.CheckInactive;
    FDataset.FFieldList.Remove(Self);
    end;
  If Assigned(AValue) then
    begin
    AValue.CheckInactive;
    AValue.FFieldList.Add(Self);
    end;
  FDataset:=AValue;
end;

procedure TField.SetDataType(AValue: TFieldType);

begin
  FDataType := AValue;
end;

procedure TField.SetFieldType(AValue: TFieldType);

begin
  { empty }
end;

procedure TField.SetParentComponent(AParent: TComponent);

begin
  if not (csLoading in ComponentState) then
    DataSet := AParent as TDataSet;
end;

procedure TField.SetSize(AValue: Integer);

begin
  CheckInactive;
  CheckTypeSize(AValue);
  FSize:=AValue;
end;

procedure TField.SetText(const AValue: string);

begin
  SetAsString(AValue);
end;

procedure TField.SetVarValue(const AValue: Variant);
begin
  Raise AccessError(SVariant);
end;

procedure TField.Validate(Buffer: Pointer);

begin
  If assigned(OnValidate) Then
    begin
    FValueBuffer:=Buffer;
    FValidating:=True;
    Try
      OnValidate(Self);
    finally
      FValidating:=False;
    end;
    end;
end;

class function TField.IsBlob: Boolean;

begin
  Result:=False;
end;

class procedure TField.CheckTypeSize(AValue: Longint);

begin
  If (AValue<>0) and Not IsBlob Then
    DatabaseErrorFmt(SInvalidFieldSize,[AValue]);
end;

// TField private methods

procedure TField.SetEditText(const AValue: string);
begin
  if Assigned(OnSetText) then
    OnSetText(Self, AValue)
  else
    SetText(AValue);
end;

function TField.GetEditText: String;
begin
  SetLength(Result, 0);
  if Assigned(OnGetText) then
    OnGetText(Self, Result, False)
  else
    GetText(Result, False);
end;

function TField.GetDisplayText: String;
begin
  SetLength(Result, 0);
  if Assigned(OnGetText) then
    OnGetText(Self, Result, True)
  else
    GetText(Result, True);
end;

procedure TField.SetDisplayLabel(const AValue: string);
begin
  if FDisplayLabel<>AValue then
    begin
    FDisplayLabel:=AValue;
    PropertyChanged(true);
    end;
end;

procedure TField.SetDisplayWidth(const AValue: Longint);
begin
  if FDisplayWidth<>AValue then
    begin
    FDisplayWidth:=AValue;
    PropertyChanged(True);
    end;
end;

function TField.GetDisplayWidth: integer;
begin
  if FDisplayWidth=0 then
    result:=GetDefaultWidth
  else
    result:=FDisplayWidth;
end;

procedure TField.SetLookup(const AValue: Boolean);
const
  ValueToLookupMap: array[Boolean] of TFieldKind = (fkData, fkLookup);
begin
  FieldKind := ValueToLookupMap[AValue];
end;

procedure TField.SetReadOnly(const AValue: Boolean);
begin
  if (FReadOnly<>AValue) then
    begin
    FReadOnly:=AValue;
    PropertyChanged(True);
    end;
end;

procedure TField.SetVisible(const AValue: Boolean);
begin
  if FVisible<>AValue then
    begin
    FVisible:=AValue;
    PropertyChanged(True);
    end;
end;


{ ---------------------------------------------------------------------
    TStringField
  ---------------------------------------------------------------------}


constructor TStringField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  SetDataType(ftString);
  FCodePage := CP_ACP;
  FFixedChar := False;
  FTransliterate := False;
  FSize := 20;
end;

procedure TStringField.SetFieldType(AValue: TFieldType);
begin
  if AValue in [ftString, ftFixedChar] then
    SetDataType(AValue);
end;

class procedure TStringField.CheckTypeSize(AValue: Longint);

begin
// A size of 0 is allowed, since for example Firebird allows
// a query like: 'select '' as fieldname from table' which
// results in a string with size 0.
  If (AValue<0) Then
    DatabaseErrorFmt(SInvalidFieldSize,[AValue])
end;

function TStringField.GetAsBoolean: Boolean;

var S : String;

begin
  S:=GetAsString;
  result := (Length(S)>0) and (Upcase(S[1]) in ['T',YesNoChars[True]]);
end;

function TStringField.GetAsDateTime: TDateTime;

begin
  Result:=StrToDateTime(GetAsString);
end;

function TStringField.GetAsFloat: Double;

begin
  Result:=StrToFloat(GetAsString);
end;

function TStringField.GetAsInteger: Longint;

begin
  Result:=StrToInt(GetAsString);
end;

function TStringField.GetAsLargeInt: Largeint;

begin
  Result:=StrToInt64(GetAsString);
end;

function TStringField.GetAsLongWord: LongWord;
begin
  Result:=StrToDWord(GetAsString);
end;

function TStringField.GetAsString: String;
begin
{$IFDEF UNICODE}
  Result := GetAsAnsiString;
{$ELSE}
  if GetValue(RawByteString(Result)) then
    SetCodePage(RawByteString(Result), CP_ACP, True)
  else
    Result:='';
{$ENDIF}
end;

function TStringField.GetAsAnsiString: AnsiString;
begin
  if GetValue(RawByteString(Result)) then
    SetCodePage(RawByteString(Result), CP_ACP, True)
  else
    Result:='';
end;

function TStringField.GetAsUTF8String: UTF8String;
begin
  if GetValue(RawByteString(Result)) then
    SetCodePage(RawByteString(Result), CP_UTF8, True)
  else
    Result:='';
end;

function TStringField.GetAsVariant: variant;

var s : rawbytestring;

begin
  If GetValue(s) then
    begin
    SetCodePage(s, CP_ACP, True); 
    Result:=s
    end
  else
    Result:=Null;
end;


function TStringField.GetDataSize: Integer;

begin
  case FCodePage of
    CP_UTF8: Result := 4*Size+1;
    else     Result :=   Size+1;
  end;
end;

function TStringField.GetDefaultWidth: Longint;

begin
  result:=Size;
end;

procedure TStringField.GetText(var AText: string; ADisplayText: Boolean);

begin
    AText:=GetAsString;
end;

function TStringField.GetValue(out AValue: RawByteString): Boolean;

var Buf, TBuf : TStringFieldBuffer;
    DynBuf, TDynBuf : Array of AnsiChar;

begin
  if DataSize <= dsMaxStringSize then
    begin
    Result:=GetData(@Buf);
    Buf[DataSize-1]:=#0;  //limit string to Size
    If Result then
      begin
      if Transliterate then
        begin
        DataSet.Translate(Buf,TBuf,False);
        AValue:=TBuf;
        end
      else
        AValue:=Buf
      end
    end
  else
    begin
    SetLength(DynBuf,DataSize);
    Result:=GetData(@DynBuf[0]);
    DynBuf[DataSize-1]:=#0;  //limit string to Size
    If Result then
      begin
      if Transliterate then
        begin
        SetLength(TDynBuf,DataSize);
        DataSet.Translate(@DynBuf[0],@TDynBuf[0],False);
        AValue:=PAnsiChar(TDynBuf);
        end
      else
        AValue:=PAnsiChar(DynBuf);
      end
    end;
  SetCodePage(AValue, FCodePage, False);
end;

procedure TStringField.SetAsBoolean(AValue: Boolean);

begin
  If AValue Then
    SetAsString('T')
  else
    SetAsString('F');
end;

procedure TStringField.SetAsDateTime(AValue: TDateTime);

begin
  SetAsString(DateTimeToStr(AValue));
end;

procedure TStringField.SetAsFloat(AValue: Double);

begin
  SetAsString(FloatToStr(AValue));
end;

procedure TStringField.SetAsInteger(AValue: Longint);

begin
  SetAsString(IntToStr(AValue));
end;

procedure TStringField.SetAsLargeInt(AValue: Largeint);

begin
  SetAsString(IntToStr(AValue));
end;

procedure TStringField.SetAsLongWord(AValue: LongWord);
begin
  SetAsString(IntToStr(AValue));
end;

procedure TStringField.SetValue(AValue: RawByteString);
var Buf      : TStringFieldBuffer;
    DynBuf   : array of AnsiChar;
begin
  if AValue='' then
    begin
    Buf := #0;
    SetData(@Buf);
    end
  else
    begin
    if StringCodePage(AValue) <> FCodePage then
      SetCodePage(AValue, FCodePage, FCodePage<>CP_NONE);
    if DataSize <= dsMaxStringSize then
      begin
      if FTransliterate then
        DataSet.Translate(@AValue[1],Buf,True)
      else
        // The data is copied into the buffer, since some TDataset descendents copy
        // the whole buffer-length in SetData. (See bug 8477)
        StrPLCopy(PAnsiChar(Buf), AValue, DataSize-1);
      // If length(AValue) > Size the buffer isn't terminated properly ?
      Buf[DataSize-1] := #0;
      SetData(@Buf);
      end
    else
      begin
      SetLength(DynBuf, DataSize);
      if FTransliterate then
        DataSet.Translate(@AValue[1],@DynBuf[0],True)
      else
        StrPLCopy(PAnsiChar(DynBuf), AValue, DataSize-1);
      SetData(@DynBuf[0]);
      end;
    end;
end;

procedure TStringField.SetAsString(const AValue: String);
begin
{$IFDEF UNICODE}
  SetAsAnsiString(AValue);
{$ELSE}
  SetValue(AValue);
{$ENDIF}
end;

procedure TStringField.SetAsAnsiString(const AValue: AnsiString);
begin
  SetValue(AValue);
end;

procedure TStringField.SetAsUTF8String(const AValue: UTF8String);
begin
  SetValue(AValue);
end;

procedure TStringField.SetVarValue(const AValue: Variant);
begin
  SetAsString(AValue);
end;


{ ---------------------------------------------------------------------
    TWideStringField
  ---------------------------------------------------------------------}

class procedure TWideStringField.CheckTypeSize(AValue: Integer);
begin
// A size of 0 is allowed, since for example Firebird allows
// a query like: 'select '' as fieldname from table' which
// results in a string with size 0.
  If (AValue<0) Then
    DatabaseErrorFmt(SInvalidFieldSize,[AValue]);
end;

constructor TWideStringField.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  SetDataType(ftWideString);
  FCodePage := CP_UTF16;
end;

procedure TWideStringField.SetFieldType(AValue: TFieldType);
begin
  if AValue in [ftWideString, ftFixedWideChar] then
    SetDataType(AValue);
end;

function TWideStringField.GetValue(out AValue: UnicodeString): Boolean;
var
  FixBuffer : array[0..dsMaxStringSize div 2] of UnicodeChar;
  DynBuffer : array of UnicodeChar;
  Buffer    : PUnicodeChar;
begin
  if DataSize <= dsMaxStringSize then begin
    Result := GetData(@FixBuffer, False);
    FixBuffer[Size]:=#0;     //limit string to Size
    AValue := FixBuffer;
  end else begin
    SetLength(DynBuffer, Succ(Size));
    Buffer := PUnicodeChar(DynBuffer);
    Result := GetData(Buffer, False);
    Buffer[Size]:=#0;     //limit string to Size
    if Result then
      AValue := Buffer;
  end;
end;

function TWideStringField.GetAsString: string;
begin
{$IFDEF UNICODE}
  if not GetValue(Result) then
    Result := '';
{$ELSE}
  Result := GetAsUnicodeString;
{$ENDIF}
end;

procedure TWideStringField.SetAsString(const AValue: string);
begin
  SetAsUnicodeString(AValue);
end;

function TWideStringField.GetAsUnicodeString: UnicodeString;
begin
  if not GetValue(Result) then
    Result := '';
end;

procedure TWideStringField.SetAsUnicodeString(const AValue: UnicodeString);
const
  NullUnicodeChar : UnicodeChar = #0;
var
  Buffer : PUnicodeChar;
begin
  if Length(AValue)>0 then
    Buffer := PUnicodeChar(@AValue[1])
  else
    Buffer := @NullUnicodeChar;
  SetData(Buffer, False);
end;

function TWideStringField.GetAsVariant: Variant;
var us: UnicodeString;
begin
  if GetValue(us) then
    Result := us
  else
    Result := Null;
end;

procedure TWideStringField.SetVarValue(const AValue: Variant);
begin
  SetAsWideString(AValue);
end;

function TWideStringField.GetAsWideString: WideString;
var us: UnicodeString;
begin
  if GetValue(us) then
    Result := us
  else
    Result := '';
end;

procedure TWideStringField.SetAsWideString(const AValue: WideString);
begin
  SetAsUnicodeString(AValue);
end;

function TWideStringField.GetAsUTF8String: UTF8String;
begin
  Result := GetAsUnicodeString;
end;

procedure TWideStringField.SetAsUTF8String(const AValue: UTF8String);
begin
  SetAsUnicodeString(AValue);
end;

function TWideStringField.GetDataSize: Integer;
begin
  Result := (Size + 1) * 2;
end;


{ ---------------------------------------------------------------------
    TNumericField
  ---------------------------------------------------------------------}


constructor TNumericField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  AlignMent:=taRightJustify;
end;

class procedure TNumericField.CheckTypeSize(AValue: Longint);
begin
  // This procedure is only added because some TDataset descendents have the
  // but that they set the Size property as if it is the DataSize property.
  // To avoid problems with those descendents, allow values <= 16.
  If (AValue>16) Then
    DatabaseErrorFmt(SInvalidFieldSize,[AValue]);
end;

procedure TNumericField.RangeError(AValue, Min, Max: Double);

begin
  DatabaseErrorFmt(SFieldError+SRangeError2,[DisplayName,AValue,Min,Max]);
end;

procedure TNumericField.SetDisplayFormat(const AValue: string);

begin
 If FDisplayFormat<>AValue then
   begin
   FDisplayFormat:=AValue;
   PropertyChanged(True);
   end;
end;

procedure TNumericField.SetEditFormat(const AValue: string);

begin
  If FEditFormat<>AValue then
    begin
    FEditFormat:=AValue;
    PropertyChanged(True);
    end;
end;

function TNumericField.GetAsBoolean: Boolean;
begin
  Result:=GetAsInteger<>0;
end;

procedure TNumericField.SetAsBoolean(AValue: Boolean);
begin
  SetAsInteger(ord(AValue));
end;

{ ---------------------------------------------------------------------
    TLongintField
  ---------------------------------------------------------------------}


constructor TLongintField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  SetDataType(ftInteger);
  FMinRange:=Low(LongInt);
  FMaxRange:=High(LongInt);
  FValidChars:=['+','-','0'..'9'];
end;

function TLongintField.GetAsFloat: Double;

begin
  Result:=GetAsInteger;
end;

function TLongintField.GetAsLargeInt: Largeint;
begin
  Result:=GetAsInteger;
end;

function TLongintField.GetAsInteger: Longint;

begin
  If Not GetValue(Result) then
    Result:=0;
end;

function TLongintField.GetAsLongWord: LongWord;
begin
  Result:=GetAsInteger;
end;

function TLongintField.GetAsVariant: Variant;

var L : Longint;

begin
  If GetValue(L) then
    Result:=L
  else
    Result:=Null;
end;

function TLongintField.GetAsString: string;

var L : Longint;

begin
  If GetValue(L) then
    Result:=IntTostr(L)
  else
    Result:='';
end;

function TLongintField.GetDataSize: Integer;

begin
  Result:=SizeOf(Longint);
end;

procedure TLongintField.GetText(var AText: string; ADisplayText: Boolean);

var l : longint;
    fmt : string;

begin
  AText:='';
  If Not GetValue(l) then exit;
  If ADisplayText or (FEditFormat='') then
    fmt:=FDisplayFormat
  else
    fmt:=FEditFormat;
  If length(fmt)<>0 then
    AText:=FormatFloat(fmt,L)
  else
    Str(L,AText);
end;

function TLongintField.GetValue(var AValue: Longint): Boolean;

var L : Longint;

begin
  L:=0;
  Result:=GetData(@L);
  If Result then
    Case DataType of
      ftInteger,ftAutoInc  : AValue:=PLongint(@L)^;
      ftSmallint           : AValue:=PSmallint(@L)^;
      ftWord               : AValue:=PWord(@L)^;
      ftShortint           : AValue:=PShortint(@L)^;
      ftByte               : AValue:=PByte(@L)^;
    end;
end;

procedure TLongintField.SetAsLargeInt(AValue: Largeint);
begin
  if (AValue>=FMinRange) and (AValue<=FMaxRange) then
    SetAsInteger(AValue)
  else
    RangeError(AValue,FMinRange,FMaxRange);
end;

procedure TLongintField.SetAsFloat(AValue: Double);

begin
  SetAsInteger(Round(AValue));
end;

procedure TLongintField.SetAsInteger(AValue: Longint);
begin
  If CheckRange(AValue) then
    SetData(@AValue)
  else
    if (FMinValue<>0) or (FMaxValue<>0) then
      RangeError(AValue,FMinValue,FMaxValue)
    else
      RangeError(AValue,FMinRange,FMaxRange);
end;

procedure TLongintField.SetAsLongWord(AValue: LongWord);
begin
  SetAsInteger(AValue);
end;

procedure TLongintField.SetVarValue(const AValue: Variant);
begin
  SetAsInteger(AValue);
end;

procedure TLongintField.SetAsString(const AValue: string);

var L,Code : longint;

begin
  If length(AValue)=0 then
    Clear
  else
    begin
    Val(AValue,L,Code);
    If Code=0 then
      SetAsInteger(L)
    else
      DatabaseErrorFmt(SFieldError+SNotAnInteger,[DisplayName,AValue]);
    end;
end;

Function TLongintField.CheckRange(AValue : longint) : Boolean;

begin
  if (FMinValue<>0) or (FMaxValue<>0) then
    Result := (AValue>=FMinValue) and (AValue<=FMaxValue)
  else
    Result := (AValue>=FMinRange) and (AValue<=FMaxRange);
end;

Procedure TLongintField.SetMaxValue (AValue : longint);

begin
  If (AValue>=FMinRange) and (AValue<=FMaxRange) then
    FMaxValue:=AValue
  else
    RangeError(AValue,FMinRange,FMaxRange);
end;

Procedure TLongintField.SetMinValue (AValue : longint);

begin
  If (AValue>=FMinRange) and (AValue<=FMaxRange) then
    FMinValue:=AValue
  else
    RangeError(AValue,FMinRange,FMaxRange);
end;

{ TShortintField }

function TShortintField.GetDataSize: Integer;
begin
  Result:=SizeOf(Shortint);
end;

constructor TShortintField.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  SetDataType(ftShortInt);
  FMinRange:=Low(ShortInt);
  FMaxRange:=High(ShortInt);
end;

{ TByteField }

function TByteField.GetDataSize: Integer;
begin
  Result:=SizeOf(Byte);
end;

constructor TByteField.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  SetDataType(ftByte);
  FMinRange:=Low(Byte);
  FMaxRange:=High(Byte);
  FValidChars:=['+','0'..'9'];
end;

{ TSmallintField }

function TSmallintField.GetDataSize: Integer;

begin
  Result:=SizeOf(SmallInt);
end;

constructor TSmallintField.Create(AOwner: TComponent);

begin
  inherited Create(AOwner);
  SetDataType(ftSmallInt);
  FMinRange:=-32768;
  FMaxRange:=32767;
end;


{ TWordField }

function TWordField.GetDataSize: Integer;

begin
  Result:=SizeOf(Word);
end;

constructor TWordField.Create(AOwner: TComponent);

begin
  inherited Create(AOwner);
  SetDataType(ftWord);
  FMinRange:=0;
  FMaxRange:=65535;
  FValidChars:=['+','0'..'9'];
end;

{ TAutoIncField }

constructor TAutoIncField.Create(AOwner: TComponent);

begin
  Inherited Create(AOWner);
  SetDataType(ftAutoInc);
end;

Procedure TAutoIncField.SetAsInteger(AValue: Longint);

begin
  // Some databases allows insertion of explicit values into identity columns
  // (some of them also allows (some not) updating identity columns)
  // So allow it at client side and leave check for server side
  //if not(FDataSet.State in [dsFilter,dsSetKey,dsInsert]) then
  //  DataBaseError(SCantSetAutoIncFields);
  inherited;
end;

{ ---------------------------------------------------------------------
    TLongWordField
  ---------------------------------------------------------------------}

constructor TLongWordField.Create(AOwner: TComponent);
begin
  Inherited Create(AOwner);
  SetDataType(ftLongWord);
  FValidChars:=['+','0'..'9'];
end;

function TLongWordField.CheckRange(AValue: LargeInt): Boolean;
begin
  if (FMinValue<>0) or (FMaxValue<>0) then
    Result := (AValue>=FMinValue) and (AValue<=FMaxValue)
  else
    Result := (AValue>=0) and (AValue<=High(LongWord));
end;

procedure TLongWordField.SetMinValue(AValue: LongWord);
begin
  FMinValue:=AValue
end;

procedure TLongWordField.SetMaxValue(AValue: LongWord);
begin
  FMaxValue:=AValue
end;

function TLongWordField.GetAsFloat: Double;
begin
  Result:=GetAsLongWord;
end;

function TLongWordField.GetAsInteger: Longint;
begin
  Result:=GetAsLongWord;
end;

function TLongWordField.GetAsLargeInt: Largeint;
begin
  Result:=GetAsLongWord;
end;

function TLongWordField.GetAsLongWord: LongWord;
begin
  if not GetValue(Result) then
    Result:=0;
end;

function TLongWordField.GetAsString: string;
begin
  Result:=IntToStr(GetAsLongWord);
end;

function TLongWordField.GetAsVariant: variant;
var L: LongWord;
begin
  If GetValue(L) then
    Result:=L
  else
    Result:=Null;
end;

function TLongWordField.GetDataSize: Integer;
begin
  Result:=SizeOf(LongWord);
end;

procedure TLongWordField.GetText(var AText: string; ADisplayText: Boolean);
var
  L : LongWord;
  fmt : string;
begin
  if GetValue(L) then
    begin
    if ADisplayText or (FEditFormat='') then
      fmt:=FDisplayFormat
    else
      fmt:=FEditFormat;
    if fmt<>'' then
      AText:=FormatFloat(fmt,L)
    else
      Str(L,AText);
    end
  else
    AText:='';
end;

function TLongWordField.GetValue(var AValue: LongWord): Boolean;
begin
  Result:=GetData(@AValue);
end;

procedure TLongWordField.SetAsFloat(AValue: Double);
begin
  SetAsLargeInt(Round(AValue));
end;

procedure TLongWordField.SetAsInteger(AValue: Longint);
begin
  SetAsLargeInt(AValue);
end;

procedure TLongWordField.SetAsLargeInt(AValue: Largeint);
begin
  if (AValue>=0) and (AValue<=High(LongWord)) then
    SetAsLongWord(AValue)
  else
    RangeError(AValue,0,High(LongWord));
end;

procedure TLongWordField.SetAsLongWord(AValue: LongWord);
begin
  if CheckRange(AValue) then
    SetData(@AValue)
  else
    RangeError(AValue,FMinValue,FMaxValue);
end;

procedure TLongWordField.SetAsString(const AValue: string);
begin
  if AValue='' then
    Clear
  else
    SetAsLongWord(StrToDWord(AValue));
end;

procedure TLongWordField.SetVarValue(const AValue: Variant);
begin
  SetAsLongWord(AValue);
end;

{ ---------------------------------------------------------------------
    TLargeintField
  ---------------------------------------------------------------------}

constructor TLargeintField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  SetDataType(ftLargeint);
  FMinRange:=Low(Largeint);
  FMaxRange:=High(Largeint);
  FValidChars:=['+','-','0'..'9'];
end;

function TLargeintField.GetAsFloat: Double;

begin
  Result:=GetAsLargeInt;
end;

function TLargeintField.GetAsLargeInt: Largeint;

begin
  If Not GetValue(Result) then
    Result:=0;
end;

function TLargeintField.GetAsLongWord: LongWord;
begin
  Result:=GetAsLargeInt;
end;

function TLargeIntField.GetAsVariant: Variant;

var L : Largeint;

begin
  If GetValue(L) then
    Result:=L
  else
    Result:=Null;
end;

function TLargeintField.GetAsInteger: Longint;

begin
  Result:=GetAsLargeInt;
end;

function TLargeintField.GetAsString: string;

var L : Largeint;

begin
  If GetValue(L) then
    Result:=IntTostr(L)
  else
    Result:='';
end;

function TLargeintField.GetDataSize: Integer;

begin
  Result:=SizeOf(Largeint);
end;

procedure TLargeintField.GetText(var AText: string; ADisplayText: Boolean);

var l : Largeint;
    fmt : string;

begin
  Atext:='';
  If Not GetValue(l) then exit;
  If ADisplayText or (FEditFormat='') then
    fmt:=FDisplayFormat
  else
    fmt:=FEditFormat;
  If length(fmt)<>0 then
    AText:=FormatFloat(fmt,L)
  else
    Str(L,AText);
end;

function TLargeintField.GetValue(var AValue: Largeint): Boolean;

var P : PLargeint;

begin
  P:=@AValue;
  Result:=GetData(P);
end;

procedure TLargeintField.SetAsFloat(AValue: Double);

begin
  SetAsLargeInt(Round(AValue));
end;

procedure TLargeintField.SetAsLargeInt(AValue: Largeint);

begin
  If CheckRange(AValue) then
    SetData(@AValue)
  else
    RangeError(AValue,FMinValue,FMaxValue);
end;

procedure TLargeintField.SetAsLongWord(AValue: LongWord);
begin
  SetAsLargeInt(AValue);
end;

procedure TLargeintField.SetAsInteger(AValue: Longint);

begin
  SetAsLargeInt(AValue);
end;

procedure TLargeintField.SetAsString(const AValue: string);

var L     : Largeint;
    code  : Longint;

begin
  If length(AValue)=0 then
    Clear
  else
    begin
    Val(AValue,L,Code);
    If Code=0 then
      SetAsLargeInt(L)
    else
      DatabaseErrorFmt(SFieldError+SNotAnInteger,[DisplayName,AValue]);
    end;
end;

procedure TLargeintField.SetVarValue(const AValue: Variant);
begin
  SetAsLargeInt(AValue);
end;

Function TLargeintField.CheckRange(AValue : Largeint) : Boolean;

begin
  if (FMinValue<>0) or (FMaxValue<>0) then
    Result := (AValue>=FMinValue) and (AValue<=FMaxValue)
  else
    Result := (AValue>=FMinRange) and (AValue<=FMaxRange);
end;

Procedure TLargeintField.SetMaxValue (AValue : Largeint);

begin
  If (AValue>=FMinRange) and (AValue<=FMaxRange) then
    FMaxValue:=AValue
  else
    RangeError(AValue,FMinRange,FMaxRange);
end;

Procedure TLargeintField.SetMinValue (AValue : Largeint);

begin
  If (AValue>=FMinRange) and (AValue<=FMaxRange) then
    FMinValue:=AValue
  else
    RangeError(AValue,FMinRange,FMaxRange);
end;

{ TFloatField }

procedure TFloatField.SetCurrency(const AValue: Boolean);
begin
  if FCurrency=AValue then exit;
  FCurrency:=AValue;
end;

procedure TFloatField.SetPrecision(const AValue: Longint);
begin
  if (AValue = -1) or (AValue > 1) then
    FPrecision := AValue
  else
    FPrecision := 2;
end;

function TFloatField.GetAsBCD: TBCD;
var f : Double;
begin
  if GetData(@f) then
    Result := DoubleToBCD(f)
  else
    Result := NullBCD;
end;

function TFloatField.GetAsFloat: Double;

begin
  If Not GetData(@Result) Then
    Result:=0.0;
end;

function TFloatField.GetAsVariant: Variant;

var f : Double;

begin
  If GetData(@f) then
    Result := f
  else
    Result:=Null;
end;

function TFloatField.GetAsLargeInt: LargeInt;
begin
  Result:=Round(GetAsFloat);
end;

function TFloatField.GetAsLongWord: LongWord;
begin
  Result:=Round(GetAsFloat);
end;

function TFloatField.GetAsInteger: Longint;

begin
  Result:=Round(GetAsFloat);
end;

function TFloatField.GetAsString: string;

var f : Double;

begin
  If GetData(@f) then
    Result:=FloatToStr(f)
  else
    Result:='';
end;

function TFloatField.GetDataSize: Integer;

begin
  Result:=SizeOf(Double);
end;

procedure TFloatField.GetText(var AText: string; ADisplayText: Boolean);

Var
    fmt : string;
    E : Double;
    Digits : integer;
    ff: TFloatFormat;

begin
  AText:='';
  If Not GetData(@E) then exit;
  If ADisplayText or (Length(FEditFormat) = 0) Then
    Fmt:=FDisplayFormat
  else
    Fmt:=FEditFormat;
    
  Digits := 0;
  if not FCurrency then
    ff := ffGeneral
  else
    begin
    Digits := CurrencyDecimals;
    if ADisplayText then
      ff := ffCurrency
    else
      ff := ffFixed;
    end;


  If fmt<>'' then
    AText:=FormatFloat(fmt,E)
  else
    AText:=FloatToStrF(E,ff,FPrecision,Digits);
end;

procedure TFloatField.SetAsBCD(const AValue: TBCD);
begin
  SetAsFloat(BCDToDouble(AValue));
end;

procedure TFloatField.SetAsFloat(AValue: Double);

begin
  If CheckRange(AValue) then
    SetData(@AValue)
  else
    RangeError(AValue,FMinValue,FMaxValue);
end;

procedure TFloatField.SetAsLargeInt(AValue: LargeInt);
begin
  SetAsFloat(AValue);
end;

procedure TFloatField.SetAsLongWord(AValue: LongWord);
begin
  SetAsFloat(AValue);
end;

procedure TFloatField.SetAsInteger(AValue: Longint);

begin
  SetAsFloat(AValue);
end;

procedure TFloatField.SetAsString(const AValue: string);

var f : Double;

begin
  If (AValue='') then
    Clear
  else  
    begin
    If not TryStrToFloat(AValue,F) then
      DatabaseErrorFmt(SNotAFloat, [AValue]);
    SetAsFloat(f);
    end;
end;

procedure TFloatField.SetVarValue(const AValue: Variant);
begin
  SetAsFloat(AValue);
end;

constructor TFloatField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  SetDataType(ftFloat);
  FPrecision:=15;
  FValidChars := [DecimalSeparator, '+', '-', '0'..'9', 'E', 'e'];
end;

Function TFloatField.CheckRange(AValue : Double) : Boolean;

begin
  If (FMinValue<>0) or (FMaxValue<>0) then
    Result:=(AValue>=FMinValue) and (AValue<=FMaxValue)
  else
    Result:=True;
end;

{ TCurrencyField }

Constructor TCurrencyField.Create(AOwner: TComponent);

begin
  inherited Create(AOwner);
  SetDataType(ftCurrency);
  Currency := True;
end;

{ TBooleanField }

function TBooleanField.GetAsBoolean: Boolean;

var b : wordbool;

begin
  If GetData(@b) then
    Result := b
  else
    Result:=False;
end;

function TBooleanField.GetAsVariant: Variant;

var b : wordbool;

begin
  If GetData(@b) then
    Result := b
  else
    Result:=Null;
end;

function TBooleanField.GetAsString: string;

var B : wordbool;

begin
  If GetData(@B) then
    Result:=FDisplays[False,B]
  else
    result:='';
end;

function TBooleanField.GetDataSize: Integer;

begin
  Result:=SizeOf(wordBool);
end;

function TBooleanField.GetDefaultWidth: Longint;

begin
  Result:=Length(FDisplays[false,false]);
  If Result<Length(FDisplays[false,True]) then
    Result:=Length(FDisplays[false,True]);
end;

function TBooleanField.GetAsInteger: Longint;
begin
  Result := ord(GetAsBoolean);
end;

procedure TBooleanField.SetAsInteger(AValue: Longint);
begin
  SetAsBoolean(AValue<>0);
end;

procedure TBooleanField.SetAsBoolean(AValue: Boolean);

var b : wordbool;

begin
  b := AValue;
  SetData(@b);
end;

procedure TBooleanField.SetAsString(const AValue: string);

var Temp : string;

begin
  Temp:=UpperCase(AValue);
  if Temp='' then
    Clear
  else if pos(Temp, FDisplays[True,True])=1 then
    SetAsBoolean(True)
  else if pos(Temp, FDisplays[True,False])=1 then
    SetAsBoolean(False)
  else
    DatabaseErrorFmt(SNotABoolean,[AValue]);
end;

procedure TBooleanField.SetVarValue(const AValue: Variant);
begin
  SetAsBoolean(AValue);
end;

constructor TBooleanField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  SetDataType(ftBoolean);
  DisplayValues:='True;False';
end;

Procedure TBooleanField.SetDisplayValues(const AValue : String);

var I : longint;

begin
  If FDisplayValues<>AValue then
    begin
    I:=Pos(';',AValue);
    If (I<2) or (I=Length(AValue)) then
      DatabaseErrorFmt(SFieldError+SInvalidDisplayValues,[DisplayName,AValue]);
    FdisplayValues:=AValue;
    // Store display values and their uppercase equivalents;
    FDisplays[False,True]:=Copy(AValue,1,I-1);
    FDisplays[True,True]:=UpperCase(FDisplays[False,True]);
    FDisplays[False,False]:=Copy(AValue,I+1,Length(AValue)-i);
    FDisplays[True,False]:=UpperCase(FDisplays[False,False]);
    PropertyChanged(True);
    end;
end;

{ TDateTimeField }

procedure TDateTimeField.SetDisplayFormat(const AValue: string);
begin
  if FDisplayFormat<>AValue then begin
    FDisplayFormat:=AValue;
    PropertyChanged(True);
  end;
end;

function TDateTimeField.GetAsDateTime: TDateTime;

begin
  If Not GetData(@Result,False) then
    Result:=0;
end;

procedure TDateTimeField.SetVarValue(const AValue: Variant);
begin
  SetAsDateTime(AValue);
end;

function TDateTimeField.GetAsVariant: Variant;

var d : tDateTime;

begin
  If GetData(@d,False) then
    Result := d
  else
    Result:=Null;
end;

function TDateTimeField.GetAsFloat: Double;

begin
  Result:=GetAsdateTime;
end;


function TDateTimeField.GetAsString: string;

begin
  GetText(Result,False);
end;


function TDateTimeField.GetDataSize: Integer;

begin
  Result:=SizeOf(TDateTime);
end;


procedure TDateTimeField.GetText(var AText: string; ADisplayText: Boolean);

var R : TDateTime;
    F : String;

begin
  If Not GetData(@R,False) then
    AText:=''
  else
    begin
    If (ADisplayText) and (Length(FDisplayFormat)<>0) then
      F:=FDisplayFormat
    else
      Case DataType of
       ftTime : F:=LongTimeFormat;
       ftDate : F:=ShortDateFormat;
      else
       F:='c'
      end;
    AText:=FormatDateTime(F,R);
    end;
end;


procedure TDateTimeField.SetAsDateTime(AValue: TDateTime);

begin
  SetData(@AValue,False);
end;


procedure TDateTimeField.SetAsFloat(AValue: Double);

begin
  SetAsDateTime(AValue);
end;


procedure TDateTimeField.SetAsString(const AValue: string);

var R : TDateTime;

begin
  if AValue<>'' then
    begin
    R:=StrToDateTime(AValue);
    SetData(@R,False);
    end
  else
    SetData(Nil);
end;


constructor TDateTimeField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  SetDataType(ftDateTime);
end;


{ TDateField }

constructor TDateField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  SetDataType(ftDate);
end;


{ TTimeField }

constructor TTimeField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  SetDataType(ftTime);
end;

procedure TTimeField.SetAsString(const AValue: string);
var R : TDateTime;
begin
  if AValue='' then
    Clear    // set to NULL
  else
    begin
    R:=StrToTime(AValue);
    SetData(@R,False);
    end;
end;



{ TBinaryField }

class procedure TBinaryField.CheckTypeSize(AValue: Longint);

begin
  // Just check for really invalid stuff; actual size is
  // dependent on the record...
  If AValue<0 then // MSSQL can have a null/0 field length in a view
    DatabaseErrorFmt(SInvalidFieldSize,[AValue]);
end;

function TBinaryField.GetAsBytes: TBytes;
begin
  if not GetValue(Result) then
    SetLength(Result, 0);
end;


function TBinaryField.GetAsString: string;
var B: TBytes;
begin
  if not GetValue(B) then
    Result := ''
  else
    SetString(Result, @B[0], length(B) div SizeOf(Char));
end;


function TBinaryField.GetAsVariant: Variant;
var B: TBytes;
    P: Pointer;
begin
  if not GetValue(B) then
    Result := Null
  else
  begin
    Result := VarArrayCreate([0, length(B)-1], varByte);
    P := VarArrayLock(Result);
    try
      Move(B[0], P^, length(B));
    finally
      VarArrayUnlock(Result);
    end;
  end;
end;


function TBinaryField.GetValue(var AValue: TBytes): Boolean;
var B: TBytes;
begin
  SetLength(B, DataSize);
  Result := assigned(B) and GetData(Pointer(B), True);
  if Result then
    if DataType = ftVarBytes then
      begin
      SetLength(AValue, PWord(B)^);
      Move(B[sizeof(Word)], AValue[0], Length(AValue));
      end
    else // ftBytes
      AValue := B;
end;


procedure TBinaryField.SetAsBytes(const AValue: TBytes);
var Buf: array[0..dsMaxStringSize] of byte;
    DynBuf: TBytes;
    Len: Word;
    P: PByte;
begin
  Len := Length(AValue);
  if Len >= DataSize then
    P := @AValue[0]
  else begin
    if DataSize <= dsMaxStringSize then
      P := @Buf[0]
    else begin
      SetLength(DynBuf, DataSize);
      P := @DynBuf[0];
    end;

    if DataType = ftVarBytes then begin
      PWord(P)^ := Len;
      Move(AValue[0], P[sizeof(Word)], Len);
    end
    else begin // ftBytes
      Move(AValue[0], P^, Len);
      FillChar(P[Len], DataSize-Len, 0); // right pad with #0
    end;
  end;
  SetData(P, True)
end;


procedure TBinaryField.SetAsString(const AValue: string);
var B : TBytes;
begin
  If Length(AValue) = DataSize then
    SetData(PChar(AValue))
  else
  begin
    SetLength(B, Length(AValue) * SizeOf(Char));
    Move(AValue[1], B[0], Length(B));
    SetAsBytes(B);
  end;
end;


procedure TBinaryField.SetVarValue(const AValue: Variant);
var P: Pointer;
    B: TBytes;
    Len: integer;
begin
  if VarIsArray(AValue) then
  begin
    P := VarArrayLock(AValue);
    try
      Len := VarArrayHighBound(AValue, 1) + 1;
      SetLength(B, Len);
      Move(P^, B[0], Len);
    finally
      VarArrayUnlock(AValue);
    end;
    SetAsBytes(B);
  end
  else
    SetAsString(AValue);
end;


constructor TBinaryField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
end;



{ TBytesField }

function TBytesField.GetDataSize: Integer;

begin
  Result:=Size;
end;


constructor TBytesField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  SetDataType(ftBytes);
  Size:=16;
end;



{ TVarBytesField }

function TVarBytesField.GetDataSize: Integer;

begin
  Result:=Size+2;
end;


constructor TVarBytesField.Create(AOwner: TComponent);

begin
  INherited Create(AOwner);
  SetDataType(ftVarBytes);
  Size:=16;
end;

{ TBCDField }

class procedure TBCDField.CheckTypeSize(AValue: Longint);

begin
  If not (AValue in [0..4]) then
    DatabaseErrorFmt(SInvalidFieldSize,[AValue]);
end;

function TBCDField.GetAsBCD: TBCD;
Var
  c:system.Currency;

begin
  If GetData(@c) then
    Result:=CurrToBCD(c)
  else
    Result:=NullBCD;
end;

function TBCDField.GetAsCurrency: Currency;

begin
  if not GetData(@Result) then
    result := 0;
end;

function TBCDField.GetAsVariant: Variant;

var c : system.Currency;

begin
  If GetData(@c) then
    Result := c
  else
    Result:=Null;
end;

function TBCDField.GetAsFloat: Double;

begin
  result := GetAsCurrency;
end;


function TBCDField.GetAsInteger: Longint;

begin
  result := round(GetAsCurrency);
end;


function TBCDField.GetAsString: string;

var c : system.currency;

begin
  If GetData(@C) then
    Result:=CurrToStr(C)
  else
    Result:='';
end;

function TBCDField.GetValue(var AValue: Currency): Boolean;

begin
  Result := GetData(@AValue);
end;

function TBCDField.GetDataSize: Integer;

begin
  result := sizeof(system.currency);
end;

function TBCDField.GetDefaultWidth: Longint;

begin
  if Precision > 0 then Result := Precision+1
  else Result := 10;
end;

procedure TBCDField.GetText(var AText: string; ADisplayText: Boolean);
var
  c : system.currency;
  fmt: String;
begin
  if GetData(@C) then begin
    if ADisplayText or (FEditFormat='') then
      fmt := FDisplayFormat
    else
      fmt := FEditFormat;
    if fmt<>'' then
      AText := FormatFloat(fmt,C)
    else if fCurrency then begin
      if ADisplayText then
        AText := FloatToStrF(C, ffCurrency, FPrecision, 2{digits?})
      else
        AText := FloatToStrF(C, ffFixed, FPrecision, 2{digits?});
    end else
      AText := FloatToStrF(C, ffGeneral, FPrecision, 0{digits?});
  end else
    AText := '';
end;

procedure TBCDField.SetAsBCD(const AValue: TBCD);
var
  c:system.currency;
begin
  if BCDToCurr(AValue,c) then
    SetAsCurrency(c);
end;

procedure TBCDField.SetAsCurrency(AValue: Currency);

begin
  If CheckRange(AValue) then
    SetData(@AValue)
  else
    RangeError(AValue,FMinValue,FMaxValue);
end;

procedure TBCDField.SetVarValue(const AValue: Variant);
begin
  SetAsCurrency(AValue);
end;

Function TBCDField.CheckRange(AValue : Currency) : Boolean;

begin
  If (FMinValue<>0) or (FMaxValue<>0) then
    Result:=(AValue>=FMinValue) and (AValue<=FMaxValue)
  else
    Result:=True;
end;

procedure TBCDField.SetAsFloat(AValue: Double);

begin
  SetAsCurrency(AValue);
end;


procedure TBCDField.SetAsInteger(AValue: Longint);

begin
  SetAsCurrency(AValue);
end;


procedure TBCDField.SetAsString(const AValue: string);

begin
  if AValue='' then
    Clear    // set to NULL
  else
    SetAsCurrency(strtocurr(AValue));
end;

constructor TBCDField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  FMaxValue := 0;
  FMinValue := 0;
  FValidChars := [DecimalSeparator, '+', '-', '0'..'9'];
  SetDataType(ftBCD);
  Precision := 18;
  Size := 4;
end;


{ TFMTBCDField }

class procedure TFMTBCDField.CheckTypeSize(AValue: Longint);
begin
  If AValue > MAXFMTBcdFractionSize then
    DatabaseErrorFmt(SInvalidFieldSize,[AValue]);
end;

constructor TFMTBCDField.Create(AOwner: TComponent);
begin
  Inherited Create(AOwner);
  FMaxValue := 0;
  FMinValue := 0;
  FValidChars := [DecimalSeparator, '+', '-', '0'..'9'];
  SetDataType(ftFMTBCD);
// Max.precision for NUMERIC,DECIMAL datatypes supported by some databases:
//  Firebird-18; Oracle,SqlServer-38; MySQL-65; PostgreSQL-1000
  Precision := 18; //default number of digits
  Size := 4; //default number of digits after decimal place
end;

function TFMTBCDField.GetDataSize: Integer;
begin
  Result := sizeof(TBCD);
end;

function TFMTBCDField.GetDefaultWidth: Longint;
begin
  if Precision > 0 then Result := Precision+1
  else Result := inherited GetDefaultWidth;
end;

function TFMTBCDField.GetAsBCD: TBCD;
begin
  if not GetData(@Result) then
    Result := NullBCD;
end;

function TFMTBCDField.GetAsCurrency: Currency;
var bcd: TBCD;
begin
  if GetData(@bcd) then
    BCDToCurr(bcd, Result)
  else
    Result := 0;
end;

function TFMTBCDField.GetAsVariant: Variant;
var bcd: TBCD;
begin
  If GetData(@bcd) then
    Result := VarFMTBcdCreate(bcd)
  else
    Result := Null;
end;

function TFMTBCDField.GetAsFloat: Double;
var bcd: TBCD;
begin
  If GetData(@bcd) then
    Result := BCDToDouble(bcd)
  else
    Result := 0;
end;

function TFMTBCDField.GetAsLargeInt: LargeInt;
var bcd: TBCD;
begin
  if GetData(@bcd) then
    Result := BCDToInteger(bcd)
  else
    Result := 0;
end;

function TFMTBCDField.GetAsLongWord: LongWord;
begin
  Result:=GetAsLargeInt;
end;

function TFMTBCDField.GetAsInteger: Longint;
begin
  Result := round(GetAsFloat);
end;

function TFMTBCDField.GetAsString: string;
var bcd: TBCD;
begin
  If GetData(@bcd) then
    Result:=BCDToStr(bcd)
  else
    Result:='';
end;

procedure TFMTBCDField.GetText(var AText: string; ADisplayText: Boolean);
var
  bcd: TBCD;
  fmt: String;
begin
  if GetData(@bcd) then begin
    if ADisplayText or (FEditFormat='') then
      fmt := FDisplayFormat
    else
      fmt := FEditFormat;
    if fmt<>'' then
      AText := FormatBCD(fmt,bcd)
    else if fCurrency then begin
      if ADisplayText then
        AText := BcdToStrF(bcd, ffCurrency, FPrecision, 2)
      else
        AText := BcdToStrF(bcd, ffFixed, FPrecision, 2);
    end else
      AText := BcdToStrF(bcd, ffGeneral, FPrecision, FSize);
  end else
    AText := '';
end;

function TFMTBCDField.GetMaxValue: string;
begin
  Result:=BCDToStr(FMaxValue);
end;

function TFMTBCDField.GetMinValue: string;
begin
  Result:=BCDToStr(FMinValue);
end;

procedure TFMTBCDField.SetMaxValue(const AValue: string);
begin
  FMaxValue:=StrToBCD(AValue);
end;

procedure TFMTBCDField.SetMinValue(const AValue: string);
begin
  FMinValue:=StrToBCD(AValue);
end;

Function TFMTBCDField.CheckRange(AValue: TBCD) : Boolean;
begin
  If (FMinValue<>0) or (FMaxValue<>0) then
    Result:=(AValue>=FMinValue) and (AValue<=FMaxValue)
  else
    Result:=True;
end;

procedure TFMTBCDField.SetAsBCD(const AValue: TBCD);
begin
  if CheckRange(AValue) then
    SetData(@AValue)
  else
    RangeError(AValue, BCDToDouble(FMinValue), BCDToDouble(FMaxValue));
end;

procedure TFMTBCDField.SetAsCurrency(AValue: Currency);
var bcd: TBCD;
begin
  if CurrToBCD(AValue, bcd, 32, Size) then
    SetAsBCD(bcd);
end;

procedure TFMTBCDField.SetVarValue(const AValue: Variant);
begin
  SetAsBCD(VarToBCD(AValue));
end;

procedure TFMTBCDField.SetAsFloat(AValue: Double);
begin
  SetAsBCD(DoubleToBCD(AValue));
end;

procedure TFMTBCDField.SetAsLargeInt(AValue: LargeInt);
begin
  SetAsBCD(IntegerToBCD(AValue));
end;

procedure TFMTBCDField.SetAsLongWord(AValue: LongWord);
begin
  SetAsLargeInt(AValue);
end;

procedure TFMTBCDField.SetAsInteger(AValue: Longint);
begin
  SetAsLargeInt(AValue);
end;

procedure TFMTBCDField.SetAsString(const AValue: string);
begin
  if AValue='' then
    Clear    // set to NULL
  else
    SetAsBCD(StrToBCD(AValue));
end;


{ TBlobField }

constructor TBlobField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  SetDataType(ftBlob);
end;


function TBlobField.GetBlobStream(Mode: TBlobStreamMode): TStream;

begin
  Result:=FDataset.CreateBlobStream(Self,Mode);
end;

function TBlobField.GetBlobType: TBlobType;
begin
  Result:= TBlobType(DataType);
end;

procedure TBlobField.SetBlobType(AValue: TBlobType);
begin
  SetFieldType(TFieldType(AValue));
end;

procedure TBlobField.FreeBuffers;
begin
end;

function TBlobField.GetAsBytes: TBytes;
var
  Stream : TStream;
  Len    : Integer;
begin
  Stream := GetBlobStream(bmRead);
  if Stream <> nil then
    try
      Len := Stream.Size;
      SetLength(Result, Len);
      if Len > 0 then
        Stream.ReadBuffer(Result[0], Len);
    finally
      Stream.Free;
    end
  else
    SetLength(Result, 0);
end;

function TBlobField.GetAsString: string;
begin
{$IFDEF UNICODE}
  Result := GetAsUnicodeString;
{$ELSE}
  Result := GetAsAnsiString;
{$ENDIF}
end;

function TBlobField.GetAsAnsiString: AnsiString;
var
  Stream : TStream;
  Len    : Integer;
  S      : AnsiString;
begin
  Stream := GetBlobStream(bmRead);
  if Stream <> nil then
    with Stream do
      try
        Len := Size;
        SetLength(S, Len);
        if Len > 0 then
          begin
          ReadBuffer(S[1], Len);
          if not Transliterate then
            Result := S
          else
            begin
            SetLength(Result, Len);
            DataSet.Translate(@S[1],@Result[1],False);
            end;
          end
        else
          Result := '';
      finally
        Free;
      end
  else
    Result := '';
end;

function TBlobField.GetAsUnicodeString: UnicodeString;
var
  Stream : TStream;
  Len    : Integer;
begin
  Stream := GetBlobStream(bmRead);
  if Stream <> nil then
    with Stream do
      try
        Len := Size;
        SetLength(Result, (Len+1) div 2);
        if Len > 0 then
          ReadBuffer(Result[1] ,Len);
      finally
        Free
      end
  else
    Result := '';
end;

function TBlobField.GetAsVariant: Variant;
begin
  if not GetIsNull then
    Result := GetAsString
  else
    Result := Null;
end;


function TBlobField.GetBlobSize: Longint;
var
  Stream: TStream;
begin
  Stream := GetBlobStream(bmRead);
  if Stream <> nil then
    with Stream do
      try
        Result:=Size;
      finally
        Free;
      end
  else
    Result := 0;
end;


function TBlobField.GetIsNull: Boolean;

begin
  if Not Modified then
    Result:= inherited GetIsNull
  else
    with GetBlobStream(bmRead) do
      try
        Result:=(Size=0);
      finally
        Free;
      end;
end;

procedure TBlobField.GetText(var AText: string; ADisplayText: Boolean);
begin
  AText := inherited GetAsString;
end;


procedure TBlobField.SetAsBytes(const AValue: TBytes);
var
  Len : Integer;
begin
  with GetBlobStream(bmWrite) do
    try
      Len := Length(AValue);
      if Len > 0 then
        WriteBuffer(AValue[0], Len);
    finally
      Free;
    end;
end;


procedure TBlobField.SetAsString(const AValue: string);
begin
{$IFDEF UNICODE}
  SetAsUnicodeString(AValue);
{$ELSE}
  SetAsAnsiString(AValue);
{$ENDIF}
end;

procedure TBlobField.SetAsAnsiString(const AValue: AnsiString);
var
  Len : Integer;
  S   : AnsiString;
begin
  with GetBlobStream(bmWrite) do
    try
      Len := Length(AValue);
      if (Len>0) then
        begin
        if Not Transliterate then
          S:=AValue
        else
          begin
          SetLength(S,Len);
          Len:=DataSet.Translate(@AValue[1],@S[1],True);
          end;
        WriteBuffer(S[1], Len);
        end;
    finally
      Free;
    end;
end;

procedure TBlobField.SetAsUnicodeString(const AValue: UnicodeString);
var
  Len : Integer;
begin
  with GetBlobStream(bmWrite) do
    try
      Len := Length(AValue) * SizeOf(UnicodeChar);
      if Len > 0 then
        WriteBuffer(AValue[1], Len);
    finally
      Free;
    end;
end;


procedure TBlobField.SetVarValue(const AValue: Variant);
begin
  SetAsString(AValue);
end;


procedure TBlobField.Clear;

begin
  GetBlobStream(bmWrite).Free;
end;


class function TBlobField.IsBlob: Boolean;

begin
  Result:=True;
end;


procedure TBlobField.LoadFromFile(const FileName: string);
var S : TFileStream;
begin
  S:=TFileStream.Create(FileName,fmOpenRead or fmShareDenyWrite);
  try
    LoadFromStream(S);
  finally
    S.Free;
  end;
end;


procedure TBlobField.LoadFromStream(Stream: TStream);

begin
  with GetBlobStream(bmWrite) do
    try
      CopyFrom(Stream,0);
    finally
      Free;
    end;
end;


procedure TBlobField.SaveToFile(const FileName: string);
var S : TFileStream;
begin
  S:=TFileStream.Create(FileName,fmCreate);
  try
    SaveToStream(S);
  finally
    S.Free;
  end;
end;


procedure TBlobField.SaveToStream(Stream: TStream);
var S : TStream;
begin
  S:=GetBlobStream(bmRead);
  Try
    If Assigned(S) then
      Stream.CopyFrom(S,0);
  finally
    S.Free;
  end;
end;

procedure TBlobField.SetFieldType(AValue: TFieldType);
begin
  if AValue in ftBlobTypes then
    SetDataType(AValue);
end;

{ TMemoField }

constructor TMemoField.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  SetDataType(ftMemo);
end;

function TMemoField.GetAsAnsiString: AnsiString;
begin
  Result := inherited GetAsAnsiString;
  SetCodePage(RawByteString(Result), FCodePage, False);
  SetCodePage(RawByteString(Result), CP_ACP, True);
end;

procedure TMemoField.SetAsAnsiString(const AValue: AnsiString);
var s: RawByteString;
begin
  s := AValue;
  SetCodePage(s, FCodePage, FCodePage<>CP_NONE);
  inherited SetAsAnsiString(s);
end;

function TMemoField.GetAsUnicodeString: UnicodeString;
begin
  Result:=GetAsAnsiString;
end;

procedure TMemoField.SetAsUnicodeString(const AValue: UnicodeString);
begin
  SetAsAnsiString(AValue);
end;

function TMemoField.GetAsUTF8String: UTF8String;
var s: RawByteString;
begin
  s := inherited GetAsAnsiString;
  SetCodePage(s, FCodePage, False);
  SetCodePage(s, CP_UTF8, True);
  Result := s;
end;

procedure TMemoField.SetAsUTF8String(const AValue: UTF8String);
begin
  SetAsAnsiString(AValue);
end;

{ TWideMemoField }

constructor TWideMemoField.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  SetDataType(ftWideMemo);
end;

function TWideMemoField.GetAsString: string;
begin
  Result := GetAsUnicodeString;
end;

procedure TWideMemoField.SetAsString(const AValue: string);
begin
  SetAsUnicodeString(AValue);
end;

function TWideMemoField.GetAsAnsiString: AnsiString;
begin
  Result := GetAsUnicodeString;
end;

procedure TWideMemoField.SetAsAnsiString(const AValue: AnsiString);
begin
  SetAsUnicodeString(AValue);
end;

function TWideMemoField.GetAsUTF8String: UTF8String;
begin
  Result := GetAsUnicodeString;
end;

procedure TWideMemoField.SetAsUTF8String(const AValue: UTF8String);
begin
  SetAsUnicodeString(AValue);
end;

function TWideMemoField.GetAsVariant: Variant;
begin
  if not GetIsNull then
    Result := GetAsUnicodeString
  else
    Result := Null;
end;

procedure TWideMemoField.SetVarValue(const AValue: Variant);
begin
  SetAsUnicodeString(AValue);
end;

{ TGraphicField }

constructor TGraphicField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  SetDataType(ftGraphic);
end;

{ TGuidField }

constructor TGuidField.Create(AOwner: TComponent);
begin
  Size := 38;
  inherited Create(AOwner);
  SetDataType(ftGuid);
end;

class procedure TGuidField.CheckTypeSize(AValue: LongInt);
begin
  if AValue <> 38 then
    DatabaseErrorFmt(SInvalidFieldSize,[AValue]);
end;

function TGuidField.GetAsGuid: TGUID;
const
  nullguid: TGUID = '{00000000-0000-0000-0000-000000000000}';
var
  S: string;
begin
  S := GetAsString;
  if S = '' then
    Result := nullguid
  else
    Result := StringToGuid(S);
end;

function TGuidField.GetDefaultWidth: LongInt;
begin
  Result := 38;
end;

procedure TGuidField.SetAsGuid(const AValue: TGUID);
begin
  SetAsString(GuidToString(AValue));
end;

{ TVariantField }

constructor TVariantField.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  SetDataType(ftVariant);
end;

class procedure TVariantField.CheckTypeSize(aValue: Integer);
begin
  { empty }
end;

function TVariantField.GetDefaultWidth: Integer;
begin
  Result := 15;
end;

function TVariantField.GetAsBoolean: Boolean;
begin
  Result := GetAsVariant;
end;

function TVariantField.GetAsDateTime: TDateTime;
begin
  Result := GetAsVariant;
end;

function TVariantField.GetAsFloat: Double;
begin
  Result := GetAsVariant;
end;

function TVariantField.GetAsInteger: Longint;
begin
  Result := GetAsVariant;
end;

function TVariantField.GetAsString: string;
begin
  Result := VarToStr(GetAsVariant);
end;

function TVariantField.GetAsWideString: WideString;
begin
  Result := VarToWideStr(GetAsVariant);
end;

function TVariantField.GetAsVariant: Variant;
begin
  if not GetData(@Result) then
    Result := Null;
end;

procedure TVariantField.SetAsBoolean(aValue: Boolean);
begin
  SetVarValue(aValue);
end;

procedure TVariantField.SetAsDateTime(aValue: TDateTime);
begin
  SetVarValue(aValue);
end;

procedure TVariantField.SetAsFloat(aValue: Double);
begin
  SetVarValue(aValue);
end;

procedure TVariantField.SetAsInteger(AValue: Longint);
begin
  SetVarValue(aValue);
end;

procedure TVariantField.SetAsString(const aValue: string);
begin
  SetVarValue(aValue);
end;

procedure TVariantField.SetAsWideString(const aValue: WideString);
begin
  SetVarValue(aValue);
end;

procedure TVariantField.SetVarValue(const aValue: Variant);
begin
  SetData(@aValue);
end;

{ TFieldsEnumerator }

function TFieldsEnumerator.GetCurrent: TField;
begin
  Result := FFields[FPosition];
end;

constructor TFieldsEnumerator.Create(AFields: TFields);
begin
  inherited Create;
  FFields := AFields;
  FPosition := -1;
end;

function TFieldsEnumerator.MoveNext: Boolean;
begin
  inc(FPosition);
  Result := FPosition < FFields.Count;
end;

{ TFields }

constructor TFields.Create(ADataset: TDataset);

begin
  FDataSet:=ADataset;
  FFieldList:=TFpList.Create;
  FValidFieldKinds:=[fkData..fkInternalcalc];
end;

destructor TFields.Destroy;

begin
  if Assigned(FFieldList) then
    Clear;
  FreeAndNil(FFieldList);
  inherited Destroy;
end;

procedure TFields.ClearFieldDefs;

Var
  i : Integer;

begin
  For I:=0 to Count-1 do
    Fields[i].FFieldDef:=Nil;
end;

procedure TFields.Changed;

begin
  // Removed FDataSet.Active check, needed for Persistent fields (see bug ID 30954)
  if (FDataSet <> nil) and not (csDestroying in FDataSet.ComponentState) then
    FDataSet.DataEvent(deFieldListChange, 0);
  If Assigned(FOnChange) then
    FOnChange(Self);
end;

procedure TFields.CheckfieldKind(Fieldkind: TFieldKind; Field: TField);

begin
  If Not (FieldKind in ValidFieldKinds) Then
    DatabaseErrorFmt(SInvalidFieldKind,[Field.DisplayName]);
end;

function TFields.GetCount: Longint;

begin
  Result:=FFieldList.Count;
end;


function TFields.GetField(Index: Integer): TField;

begin
  Result:=Tfield(FFieldList[Index]);
end;

procedure TFields.SetField(Index: Integer; Value: TField);
begin
  Fields[Index].Assign(Value);
end;

procedure TFields.SetFieldIndex(Field: TField; Value: Integer);
var Old : Longint;
begin
  Old := FFieldList.indexOf(Field);
  If Old=-1 then
    Exit;
  // Check value
  If Value<0 Then Value:=0;
  If Value>=Count then Value:=Count-1;
  If Value<>Old then
    begin
    FFieldList.Delete(Old);
    FFieldList.Insert(Value,Field);
    Field.PropertyChanged(True);
    Changed;
    end;
end;

procedure TFields.Add(Field: TField);

begin
  CheckFieldName(Field.FieldName);
  FFieldList.Add(Field);
  Field.FFields:=Self;
  Changed;
end;

procedure TFields.CheckFieldName(const Value: String);

begin
  If FindField(Value)<>Nil then
    DataBaseErrorFmt(SDuplicateFieldName,[Value],FDataset);
end;

procedure TFields.CheckFieldNames(const Value: String);

var
  N: String;
  StrPos: Integer;

begin
  if Value = '' then
    Exit;
  StrPos := 1;
  repeat
    N := ExtractFieldName(Value, StrPos);
    // Will raise an error if no such field...
    FieldByName(N);
  until StrPos > Length(Value);
end;

procedure TFields.Clear;
var
  AField: TField;
begin
  while FFieldList.Count > 0 do 
    begin
    AField := TField(FFieldList.Last);
    AField.FDataSet := Nil;
    AField.Free;
    FFieldList.Delete(FFieldList.Count - 1);
    end;
  Changed;
end;

function TFields.FindField(const Value: String): TField;
var S : String;
    I : longint;
begin
  S:=UpperCase(Value);
  For I:=0 To FFieldList.Count-1 do
  begin
    Result:=TField(FFieldList[I]);
    if S=UpperCase(Result.FieldName) then
    begin
      {$ifdef dsdebug}
      Writeln ('Found field ',Value);
      {$endif}
      Exit;
    end;
  end;
  Result:=Nil;
end;

function TFields.FieldByName(const Value: String): TField;

begin
  Result:=FindField(Value);
  If result=Nil then
    DatabaseErrorFmt(SFieldNotFound,[Value],FDataset);
end;

function TFields.FieldByNumber(FieldNo: Integer): TField;
var i : Longint;
begin
  For I:=0 to FFieldList.Count-1 do
  begin
    Result:=TField(FFieldList[I]);
    if FieldNo=Result.FieldNo then
      Exit;
  end;
  Result:=Nil;
end;

function TFields.GetEnumerator: TFieldsEnumerator;

begin
  Result:=TFieldsEnumerator.Create(Self);
end;

procedure TFields.GetFieldNames(Values: TStrings);
var i : longint;
begin
  Values.Clear;
  For I:=0 to FFieldList.Count-1 do
    Values.Add(Tfield(FFieldList[I]).FieldName);
end;

function TFields.IndexOf(Field: TField): Longint;

begin
  Result:=FFieldList.IndexOf(Field);
end;

procedure TFields.Remove(Value : TField);

begin
  FFieldList.Remove(Value);
  Value.FFields := nil;
  Changed;
end;

