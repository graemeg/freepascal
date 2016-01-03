{
    This file is part of the Free Pascal run time library.
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (c) 1999-2014 by Michael Van Canneyt, member of the
=======
    Copyright (c) 1999-2000 by Michael Van Canneyt, member of the
>>>>>>> graemeg/fixes_2_2
=======
    Copyright (c) 1999-2000 by Michael Van Canneyt, member of the
>>>>>>> origin/fixes_2_2
    Free Pascal development team

    TFields and related components implementations.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{Procedure DumpMem (P : Pointer;Size : Longint);

<<<<<<< HEAD
<<<<<<< HEAD
var i : longint;
=======
Var i : longint;
>>>>>>> graemeg/fixes_2_2
=======
Var i : longint;
>>>>>>> origin/fixes_2_2

begin
  Write ('Memory dump : ');
  For I:=0 to Size-1 do
    Write (Pbyte(P)[i],' ');
  Writeln;
end;}

{ ---------------------------------------------------------------------
    TFieldDef
  ---------------------------------------------------------------------}

Constructor TFieldDef.Create(ACollection : TCollection);

begin
<<<<<<< HEAD
<<<<<<< HEAD
  Inherited Create(ACollection);
=======
  Inherited create(ACollection);
>>>>>>> graemeg/fixes_2_2
=======
  Inherited create(ACollection);
>>>>>>> origin/fixes_2_2
  FFieldNo:=Index+1;
end;

Constructor TFieldDef.Create(AOwner: TFieldDefs; const AName: string;
      ADataType: TFieldType; ASize: Integer; ARequired: Boolean; AFieldNo: Longint);

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
end;

Destructor TFieldDef.Destroy;

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
    finally
      Collection.EndUpdate;
    end;
  end else
  inherited Assign(APersistent);
end;

Function TFieldDef.CreateField(AOwner: TComponent): TField;

<<<<<<< HEAD
<<<<<<< HEAD
var TheField : TFieldClass;
=======
Var TheField : TFieldClass;
>>>>>>> graemeg/fixes_2_2
=======
Var TheField : TFieldClass;
>>>>>>> origin/fixes_2_2

begin
{$ifdef dsdebug}
  Writeln ('Creating field '+FNAME);
{$endif dsdebug}
  TheField:=GetFieldClass;
  if TheField=Nil then
    DatabaseErrorFmt(SUnknownFieldType,[FName]);
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TheField.Create(AOwner);
  Try
    Result.FFieldDef:=Self;
=======
  Result:=Thefield.Create(AOwner);
  Try
>>>>>>> graemeg/fixes_2_2
=======
  Result:=Thefield.Create(AOwner);
  Try
>>>>>>> origin/fixes_2_2
    Result.Size:=FSize;
    Result.Required:=FRequired;
    Result.FFieldName:=FName;
    Result.FDisplayLabel:=DisplayName;
    Result.FFieldNo:=Self.FieldNo;
    Result.SetFieldType(DataType);
<<<<<<< HEAD
<<<<<<< HEAD
    Result.FReadOnly:=(faReadOnly in Attributes);
{$ifdef dsdebug}
    Writeln ('TFieldDef.CreateField : Result Fieldno : ',Result.FieldNo,'; Self : ',FieldNo);
    Writeln ('TFieldDef.CreateField : Trying to set dataset');
=======
=======
>>>>>>> origin/fixes_2_2
    Result.FReadOnly:= (faReadOnly in Attributes);
{$ifdef dsdebug}
    Writeln ('TFieldDef.CReateField : Trying to set dataset');
{$endif dsdebug}
{$ifdef dsdebug}
    Writeln ('TFieldDef.CReateField : Result Fieldno : ',Result.FieldNo,' Self : ',FieldNo);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif dsdebug}
    Result.Dataset:=TFieldDefs(Collection).Dataset;
    If (Result is TFloatField) then
      TFloatField(Result).Precision:=FPrecision;
    if (Result is TBCDField) then
      TBCDField(Result).Precision:=FPrecision;
<<<<<<< HEAD
<<<<<<< HEAD
    if (Result is TFmtBCDField) then
      TFmtBCDField(Result).Precision:=FPrecision;
=======
=======
>>>>>>> origin/fixes_2_2
    {Add when implemented:
    if (Result is TFmtBCDField) then
      TFmtBCDField(Result).Precision:=FPrecision;
    }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  except
    Result.Free;
    Raise;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
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

Function TFieldDef.GetFieldClass : TFieldClass;

begin
  //!! Should be owner as tdataset but that doesn't work ??

  If Assigned(Collection) And
     (Collection is TFieldDefs) And
     Assigned(TFieldDefs(Collection).Dataset) then
    Result:=TFieldDefs(Collection).Dataset.GetFieldClass(FDataType)
  else
    Result:=Nil;
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
<<<<<<< HEAD
<<<<<<< HEAD
  // the fielddef will register itself here as an owned component.
  // fieldno is 1 based !
  BeginUpdate;
  try
    Add(AName,ADataType,ASize,Arequired,Count+1);
=======
=======
>>>>>>> origin/fixes_2_2
  // the fielddef will register itself here as a owned component.
  // fieldno is 1 based !
  BeginUpdate;
  try
    TFieldDef.Create(Self,AName,ADataType,ASize,Arequired,Count+1);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
class function TFieldDefs.FieldDefClass: TFieldDefClass;
begin
  Result:=TFieldDef;
end;

constructor TFieldDefs.Create(ADataSet: TDataSet);
begin
  Inherited Create(ADataset, Owner, FieldDefClass);
end;

function TFieldDefs.Add(const AName: string; ADataType: TFieldType; ASize: Word; ARequired: Boolean; AFieldNo: Integer): TFieldDef;
begin
  Result:=FieldDefClass.create(Self,AName,ADataType,ASize,ARequired,AFieldNo);
=======
constructor TFieldDefs.Create(ADataset: TDataset);
begin
  Inherited Create(ADataset, Owner, TFieldDef);
>>>>>>> graemeg/fixes_2_2
=======
constructor TFieldDefs.Create(ADataset: TDataset);
begin
  Inherited Create(ADataset, Owner, TFieldDef);
>>>>>>> origin/fixes_2_2
end;

procedure TFieldDefs.Assign(FieldDefs: TFieldDefs);

<<<<<<< HEAD
<<<<<<< HEAD
var I : longint;
=======
Var I : longint;
>>>>>>> graemeg/fixes_2_2
=======
Var I : longint;
>>>>>>> origin/fixes_2_2

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

<<<<<<< HEAD
<<<<<<< HEAD
var I : longint;
=======
Var I : longint;
>>>>>>> graemeg/fixes_2_2
=======
Var I : longint;
>>>>>>> origin/fixes_2_2

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

<<<<<<< HEAD
<<<<<<< HEAD
function TFieldDefs.AddFieldDef: TFieldDef;

begin
  Result:=FieldDefClass.Create(Self,'',ftUnknown,0,False,Count+1);
=======
=======
>>>>>>> origin/fixes_2_2
Function TFieldDefs.AddFieldDef : TFieldDef;

begin
  Result:=TFieldDef.Create(Self,'',ftUnknown,0,False,Count+1);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
  SVariant = 'Variant';
  SString = 'String';
<<<<<<< HEAD
<<<<<<< HEAD
  SBytes = 'Bytes';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

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
  Result:=EDatabaseError.CreateFmt(SinvalidTypeConversion,[TypeName,FFieldName]);
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
      vtAnsiString:
        AsString := string(VAnsiString);
      vtCurrency:
        AsCurrency := VCurrency^;
      vtVariant:
        if not VarIsClear(VVariant^) then Self.Value := VVariant^;
      vtWideString:
        AsWideString := WideString(VWideString);
      vtInt64:
        AsLargeInt := VInt64^;
    else
      Error;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
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

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
  SetData(Nil);
=======
  if FieldKind in [fkData, fkInternalCalc] then
    SetData(Nil);
>>>>>>> graemeg/fixes_2_2
=======
  if FieldKind in [fkData, fkInternalCalc] then
    SetData(Nil);
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
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
  SetLength(Result, DataSize);
  if assigned(result) and not GetData(@Result[0], False) then
    Result := nil;
=======
=======
>>>>>>> origin/fixes_2_2
function TField.GetAsBoolean: Boolean;

begin
  raise AccessError(SBoolean);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function TField.GetAsDateTime: TDateTime;

begin
  raise AccessError(SdateTime);
end;

function TField.GetAsFloat: Double;

begin
  raise AccessError(SDateTime);
end;

function TField.GetAsLongint: Longint;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=GetAsInteger;
end;

function TField.GetAsInteger: Longint;

begin
  raise AccessError(SInteger);
end;

function TField.GetAsVariant: variant;

begin
  raise AccessError(SVariant);
end;


=======
=======
>>>>>>> origin/fixes_2_2
  raise AccessError(SInteger);
end;

function TField.GetAsVariant: Variant;

begin
  raise AccessError(SVariant);
end;


function TField.GetAsInteger: Integer;

begin
  Result:=GetAsLongint;
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TField.GetAsString: string;

begin
  Result := GetClassDesc;
end;

function TField.GetAsWideString: WideString;
begin
  Result := GetAsString;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TField.GetOldValue: variant;
=======
function TField.GetOldValue: Variant;
>>>>>>> graemeg/fixes_2_2
=======
function TField.GetOldValue: Variant;
>>>>>>> origin/fixes_2_2

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
<<<<<<< HEAD
<<<<<<< HEAD
    Result := FieldKind in [fkData, fkInternalCalc];
    if Result then
      begin
      Result:=Assigned(DataSet) and Dataset.Active;
      If Result then
        Result:= DataSet.CanModify;
      end;
=======
    Result:=Assigned(DataSet);
    If Result then
      Result:= DataSet.CanModify;
>>>>>>> graemeg/fixes_2_2
=======
    Result:=Assigned(DataSet);
    If Result then
      Result:= DataSet.CanModify;
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
  If FValidating then
    begin
    result:=assigned(FValueBuffer);
    If Result and assigned(Buffer) then
=======
=======
>>>>>>> origin/fixes_2_2
  If FVAlidating then
    begin
    result:=Not(FValueBuffer=Nil);
    If Result then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
function TField.IsDisplayLabelStored: Boolean;
=======
Function TField.IsDisplayStored : Boolean;
>>>>>>> graemeg/fixes_2_2
=======
Function TField.IsDisplayStored : Boolean;
>>>>>>> origin/fixes_2_2

begin
  Result:=(DisplayLabel<>FieldName);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TField.IsDisplayWidthStored : Boolean;

begin
  Result:=(FDisplayWidth<>0);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
function TField.GetIndex: longint;
=======
function TField.getIndex : longint;
>>>>>>> graemeg/fixes_2_2
=======
function TField.getIndex : longint;
>>>>>>> origin/fixes_2_2

begin
  If Assigned(FDataset) then
    Result:=FDataset.FFieldList.IndexOf(Self)
  else
    Result:=-1;
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
function TField.GetAsBCD: TBCD;
begin
  raise AccessError(SBCD);
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TField.GetLookup: Boolean;
begin
  Result := FieldKind = fkLookup;
end;

function TField.GetAsLargeInt: LargeInt;
begin
  Raise AccessError(SLargeInt);
end;

function TField.GetAsCurrency: Currency;
begin
  Result := GetAsFloat;
end;

procedure TField.SetAlignment(const AValue: TAlignMent);
begin
  if FAlignment <> AValue then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    FAlignment := AValue;
=======
    FAlignment := Avalue;
>>>>>>> graemeg/fixes_2_2
=======
    FAlignment := Avalue;
>>>>>>> origin/fixes_2_2
    PropertyChanged(false);
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TField.SetIndex(const AValue: Longint);
=======
=======
>>>>>>> origin/fixes_2_2
procedure TField.SetAsBCD(const AValue: TBCD);
begin
  Raise AccessError(SBCD);
end;

procedure TField.SetIndex(const AValue: Integer);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
    FFields.CheckFieldNames(FKeyFields);
    FLookupDataSet.Fields.CheckFieldNames(FLookupKeyFields);
    FLookupDataset.FieldByName(FLookupResultField); // I presume that if it doesn't exist it throws exception, and that a field with null value is still valid
=======
    FFields.CheckFieldNames(FLookupKeyfields);
    FLookupDataset.FieldByName(FLookupresultField); // I presume that if it doesn't exist it throws exception, and that a field with null value is still valid
>>>>>>> graemeg/fixes_2_2
=======
    FFields.CheckFieldNames(FLookupKeyfields);
    FLookupDataset.FieldByName(FLookupresultField); // I presume that if it doesn't exist it throws exception, and that a field with null value is still valid
>>>>>>> origin/fixes_2_2
    LookupList.Clear; // have to be F-less because we might be creating it here with getter!

    FLookupDataSet.DisableControls;
    try
<<<<<<< HEAD
<<<<<<< HEAD
      FLookupDataSet.First;
      while not FLookupDataSet.Eof do
      begin
        FLookupList.Add(FLookupDataSet.FieldValues[FLookupKeyfields], FLookupDataSet.FieldValues[FLookupResultField]);
        FLookupDataSet.Next;
      end;
=======
=======
>>>>>>> origin/fixes_2_2
      FLookupDataSet.Open;
      repeat
        FLookupList.Add(FLookupDataSet.FieldValues[FLookupKeyfields], FLookupDataSet.FieldValues[FLookupResultField]);
        FLookupDataSet.Next;
      until FLookupDataSet.EOF;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
procedure TField.SetAsBCD(const AValue: TBCD);
begin
  Raise AccessError(SBCD);
end;

procedure TField.SetAsBytes(const AValue: TBytes);
begin
  raise AccessError(SBytes);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
procedure TField.SetAsVariant(const AValue: variant);
=======
procedure TField.SetAsVariant(const AValue: Variant);
>>>>>>> graemeg/fixes_2_2
=======
procedure TField.SetAsVariant(const AValue: Variant);
>>>>>>> origin/fixes_2_2

begin
  if VarIsNull(AValue) then
    Clear
  else
    try
      SetVarValue(AValue);
    except
      on EVariantError do DatabaseErrorFmt(SFieldValueError, [DisplayName]);
    end;
end;


procedure TField.SetAsLongint(AValue: Longint);
<<<<<<< HEAD
<<<<<<< HEAD
begin
  SetAsInteger(AValue);
end;

procedure TField.SetAsInteger(AValue: Longint);
begin
  raise AccessError(SInteger);
end;

procedure TField.SetAsLargeInt(AValue: Largeint);
=======
=======
>>>>>>> origin/fixes_2_2

begin
  Raise AccessError(SInteger);
end;

procedure TField.SetAsInteger(AValue: Integer);

begin
  SetAsLongint(AValue);
end;

procedure TField.SetAsLargeint(AValue: Largeint);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  Raise AccessError(SLargeInt);
end;

procedure TField.SetAsString(const AValue: string);
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
begin
  Raise AccessError(SString);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TField.SetAsWideString(const AValue: WideString);
begin
  SetAsString(AValue);
=======
procedure TField.SetAsWideString(const aValue: WideString);
begin
  SetAsString(aValue);
>>>>>>> graemeg/fixes_2_2
=======
procedure TField.SetAsWideString(const aValue: WideString);
begin
  SetAsString(aValue);
>>>>>>> origin/fixes_2_2
end;


procedure TField.SetData(Buffer: Pointer);

begin
 SetData(Buffer,True);
end;

procedure TField.SetData(Buffer: Pointer; NativeFormat : Boolean);

begin
  If Not Assigned(FDataset) then
<<<<<<< HEAD
<<<<<<< HEAD
    DatabaseErrorFmt(SNoDataset,[FieldName]);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if (FieldNo>0) and not (FDataSet.State in [dsSetKey, dsFilter]) then
    begin
    if ReadOnly then 
      DatabaseErrorFmt(SReadOnlyField, [DisplayName], Self); 
    Validate(Buffer);
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  FDataSet.SetFieldData(Self,Buffer, NativeFormat);
end;

procedure TField.SetDataset(AValue: TDataset);
=======
=======
>>>>>>> origin/fixes_2_2
    EDatabaseError.CreateFmt(SNoDataset,[FieldName]);
  FDataSet.SetFieldData(Self,Buffer, NativeFormat);
end;

Procedure TField.SetDataset (AValue : TDataset);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

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
  AsString:=AValue;
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

<<<<<<< HEAD
<<<<<<< HEAD
class function TField.IsBlob: Boolean;
=======
class function Tfield.IsBlob: Boolean;
>>>>>>> graemeg/fixes_2_2
=======
class function Tfield.IsBlob: Boolean;
>>>>>>> origin/fixes_2_2

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
<<<<<<< HEAD
<<<<<<< HEAD
  if FDisplayLabel<>AValue then
    begin
    FDisplayLabel:=AValue;
=======
  if FDisplayLabel<>Avalue then
    begin
    FDisplayLabel:=Avalue;
>>>>>>> graemeg/fixes_2_2
=======
  if FDisplayLabel<>Avalue then
    begin
    FDisplayLabel:=Avalue;
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
  if (FReadOnly<>AValue) then
=======
  if (FReadOnly<>Avalue) then
>>>>>>> graemeg/fixes_2_2
=======
  if (FReadOnly<>Avalue) then
>>>>>>> origin/fixes_2_2
    begin
    FReadOnly:=AValue;
    PropertyChanged(True);
    end;
end;

procedure TField.SetVisible(const AValue: Boolean);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if FVisible<>AValue then
=======
  if FVisible<>Avalue then
>>>>>>> graemeg/fixes_2_2
=======
  if FVisible<>Avalue then
>>>>>>> origin/fixes_2_2
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
  FFixedChar := False;
  FTransliterate := False;
  FSize:=20;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TStringField.SetFieldType(AValue: TFieldType);
begin
  if AValue in [ftString, ftFixedChar] then
    SetDataType(AValue);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
class procedure TStringField.CheckTypeSize(AValue: Longint);

begin
// A size of 0 is allowed, since for example Firebird allows
// a query like: 'select '' as fieldname from table' which
// results in a string with size 0.
<<<<<<< HEAD
<<<<<<< HEAD
  If (AValue<0) Then
=======
  If (AValue<0) or (AValue>dsMaxStringSize) Then
>>>>>>> graemeg/fixes_2_2
=======
  If (AValue<0) or (AValue>dsMaxStringSize) Then
>>>>>>> origin/fixes_2_2
    databaseErrorFmt(SInvalidFieldSize,[AValue])
end;

function TStringField.GetAsBoolean: Boolean;

<<<<<<< HEAD
<<<<<<< HEAD
var S : String;
=======
Var S : String;
>>>>>>> graemeg/fixes_2_2
=======
Var S : String;
>>>>>>> origin/fixes_2_2

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

<<<<<<< HEAD
<<<<<<< HEAD
function TStringField.GetAsInteger: Longint;
=======
function TStringField.GetAsLongint: Longint;
>>>>>>> graemeg/fixes_2_2
=======
function TStringField.GetAsLongint: Longint;
>>>>>>> origin/fixes_2_2

begin
  Result:=StrToInt(GetAsString);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TStringField.GetAsLargeInt: Largeint;

begin
  Result:=StrToInt64(GetAsString);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TStringField.GetAsString: string;

begin
  If Not GetValue(Result) then
    Result:='';
end;

function TStringField.GetAsVariant: Variant;

<<<<<<< HEAD
<<<<<<< HEAD
var s : string;
=======
Var s : string;
>>>>>>> graemeg/fixes_2_2
=======
Var s : string;
>>>>>>> origin/fixes_2_2

begin
  If GetValue(s) then
    Result:=s
  else
    Result:=Null;
end;


function TStringField.GetDataSize: Integer;

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=Size+1;
=======
  if DataType=ftFixedChar then
    Result:=Size+1
  else
    Result:=Size+1;
>>>>>>> graemeg/cpstrnew
=======
  Result:=Size+1;
>>>>>>> graemeg/fixes_2_2
=======
  Result:=Size+1;
>>>>>>> origin/fixes_2_2
end;

function TStringField.GetDefaultWidth: Longint;

begin
  result:=Size;
end;

Procedure TStringField.GetText(var AText: string; ADisplayText: Boolean);

begin
    AText:=GetAsString;
end;

function TStringField.GetValue(var AValue: string): Boolean;

<<<<<<< HEAD
<<<<<<< HEAD
var Buf, TBuf : TStringFieldBuffer;
    DynBuf, TDynBuf : Array of char;

begin
  if DataSize <= dsMaxStringSize then
    begin
    Result:=GetData(@Buf);
    Buf[Size]:=#0;  //limit string to Size
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
    DynBuf[Size]:=#0;  //limit string to Size
    If Result then
      begin
      if Transliterate then
        begin
        SetLength(TDynBuf,DataSize);
        DataSet.Translate(@DynBuf[0],@TDynBuf[0],False);
        AValue:=pchar(TDynBuf);
        end
      else
        AValue:=pchar(DynBuf);
      end
    end;
=======
=======
>>>>>>> origin/fixes_2_2
Var Buf, TBuf : TStringFieldBuffer;

begin
  Result:=GetData(@Buf);
  If Result then
    begin
    if transliterate then
      begin
      DataSet.Translate(Buf,TBuf,False);
      AValue:=TBuf;
      end
    else
      AValue:=Buf
    end
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
procedure TStringField.SetAsInteger(AValue: Longint);

begin
  SetAsString(IntToStr(AValue));
end;

procedure TStringField.SetAsLargeInt(AValue: Largeint);

begin
  SetAsString(IntToStr(AValue));
=======
=======
>>>>>>> origin/fixes_2_2
procedure TStringField.SetAsLongint(AValue: Longint);

begin
  SetAsString(intToStr(AValue));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TStringField.SetAsString(const AValue: string);

var Buf      : TStringFieldBuffer;
<<<<<<< HEAD
<<<<<<< HEAD
    DynBuf   : array of char;

begin
  if Length(AValue)=0 then
    begin
    Buf := #0;
    SetData(@Buf);
    end
  else if DataSize <= dsMaxStringSize then
    begin
    if FTransliterate then
      DataSet.Translate(@AValue[1],Buf,True)
    else
      // The data is copied into the buffer, since some TDataset descendents copy
      // the whole buffer-length in SetData. (See bug 8477)
      StrPLCopy(PChar(Buf), AValue, Size);
    // If length(AValue) > Size the buffer isn't terminated properly ?
    Buf[Size] := #0;
    SetData(@Buf);
    end
  else
    begin
    SetLength(DynBuf, DataSize);
    if FTransliterate then
      DataSet.Translate(@AValue[1],@DynBuf[0],True)
    else
      StrPLCopy(PChar(DynBuf), AValue, Size);
    SetData(@DynBuf[0]);
    end
=======
=======
>>>>>>> origin/fixes_2_2

begin
  IF Length(AValue)=0 then
    begin
    Buf := #0;
    SetData(@buf);
    end
  else if FTransliterate then
    begin
    DataSet.Translate(@AValue[1],Buf,True);
    Buf[DataSize-1] := #0;
    SetData(@buf);
    end
  else
    begin
    // The data is copied into the buffer, since some TDataset descendents copy
    // the whole buffer-length in SetData. (See bug 8477)
    Buf := AValue;
    // If length(AValue) > Datasize the buffer isn't terminated properly
    Buf[DataSize-1] := #0;
    SetData(@Buf);
    end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TStringField.SetVarValue(const AValue: Variant);
begin
  SetAsString(AValue);
end;

{ ---------------------------------------------------------------------
    TWideStringField
  ---------------------------------------------------------------------}

<<<<<<< HEAD
<<<<<<< HEAD
class procedure TWideStringField.CheckTypeSize(AValue: Integer);
=======
class procedure TWideStringField.CheckTypeSize(aValue: Integer);
>>>>>>> graemeg/fixes_2_2
=======
class procedure TWideStringField.CheckTypeSize(aValue: Integer);
>>>>>>> origin/fixes_2_2
begin
// A size of 0 is allowed, since for example Firebird allows
// a query like: 'select '' as fieldname from table' which
// results in a string with size 0.
<<<<<<< HEAD
<<<<<<< HEAD
  If (AValue<0) Then
=======
  If (AValue<0) or (AValue>(dsMaxStringSize div 2)) Then
>>>>>>> graemeg/fixes_2_2
=======
  If (AValue<0) or (AValue>(dsMaxStringSize div 2)) Then
>>>>>>> origin/fixes_2_2
    databaseErrorFmt(SInvalidFieldSize,[AValue]);
end;

constructor TWideStringField.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  SetDataType(ftWideString);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TWideStringField.SetFieldType(AValue: TFieldType);
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if AValue in [ftWideString, ftFixedWideChar] then
    SetDataType(AValue);
end;

function TWideStringField.GetValue(var AValue: WideString): Boolean;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if avalue in [ftWideString, ftFixedWideChar] then
    SetDataType(AValue);
end;

function TWideStringField.GetValue(var aValue: WideString): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TWideStringField.GetValue(var aValue: WideString): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
function TWideStringField.GetValue(var aValue: WideString): Boolean;
>>>>>>> origin/fixes_2_2
var
  FixBuffer : array[0..dsMaxStringSize div 2] of WideChar;
  DynBuffer : array of WideChar;
  Buffer    : PWideChar;
begin
  if DataSize <= dsMaxStringSize then begin
    Result := GetData(@FixBuffer, False);
<<<<<<< HEAD
<<<<<<< HEAD
    FixBuffer[Size]:=#0;     //limit string to Size
    AValue := FixBuffer;
=======
    aValue := FixBuffer;
>>>>>>> graemeg/fixes_2_2
=======
    aValue := FixBuffer;
>>>>>>> origin/fixes_2_2
  end else begin
    SetLength(DynBuffer, Succ(Size));
    Buffer := PWideChar(DynBuffer);
    Result := GetData(Buffer, False);
<<<<<<< HEAD
<<<<<<< HEAD
    Buffer[Size]:=#0;     //limit string to Size
    if Result then
      AValue := Buffer;
=======
    if Result then
      aValue := Buffer;
>>>>>>> graemeg/fixes_2_2
=======
    if Result then
      aValue := Buffer;
>>>>>>> origin/fixes_2_2
  end;
end;

function TWideStringField.GetAsString: string;
begin
  Result := GetAsWideString;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TWideStringField.SetAsString(const AValue: string);
begin
  SetAsWideString(AValue);
=======
procedure TWideStringField.SetAsString(const aValue: string);
begin
  SetAsWideString(aValue);
>>>>>>> graemeg/fixes_2_2
=======
procedure TWideStringField.SetAsString(const aValue: string);
begin
  SetAsWideString(aValue);
>>>>>>> origin/fixes_2_2
end;

function TWideStringField.GetAsVariant: Variant;
var
  ws: WideString;
begin
  if GetValue(ws) then
    Result := ws
  else
    Result := Null;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TWideStringField.SetVarValue(const AValue: Variant);
begin
  SetAsWideString(AValue);
=======
procedure TWideStringField.SetVarValue(const aValue: Variant);
begin
  SetAsWideString(aValue);
>>>>>>> graemeg/fixes_2_2
=======
procedure TWideStringField.SetVarValue(const aValue: Variant);
begin
  SetAsWideString(aValue);
>>>>>>> origin/fixes_2_2
end;

function TWideStringField.GetAsWideString: WideString;
begin
  if not GetValue(Result) then
    Result := '';
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TWideStringField.SetAsWideString(const AValue: WideString);
=======
procedure TWideStringField.SetAsWideString(const aValue: WideString);
>>>>>>> graemeg/fixes_2_2
=======
procedure TWideStringField.SetAsWideString(const aValue: WideString);
>>>>>>> origin/fixes_2_2
const
  NullWideChar : WideChar = #0;
var
  Buffer : PWideChar;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if Length(AValue)>0 then
    Buffer := PWideChar(@AValue[1])
=======
  if Length(aValue)>0 then
    Buffer := PWideChar(@aValue[1])
>>>>>>> graemeg/fixes_2_2
=======
  if Length(aValue)>0 then
    Buffer := PWideChar(@aValue[1])
>>>>>>> origin/fixes_2_2
  else
    Buffer := @NullWideChar;
  SetData(Buffer, False);
end;

function TWideStringField.GetDataSize: Integer;
begin
  Result :=
    (Size + 1) * 2;
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
<<<<<<< HEAD
<<<<<<< HEAD
  DatabaseErrorFmt(SRangeError,[AValue,Min,Max,FieldName]);
=======
  DatabaseErrorFMT(SRangeError,[AValue,Min,Max,FieldName]);
>>>>>>> graemeg/fixes_2_2
=======
  DatabaseErrorFMT(SRangeError,[AValue,Min,Max,FieldName]);
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
  If FEditFormat<>AValue then
    begin
    FEditFormat:=AValue;
=======
  If FEDitFormat<>AValue then
    begin
    FEDitFormat:=AVAlue;
>>>>>>> graemeg/fixes_2_2
=======
  If FEDitFormat<>AValue then
    begin
    FEDitFormat:=AVAlue;
>>>>>>> origin/fixes_2_2
    PropertyChanged(True);
    end;
end;

function TNumericField.GetAsBoolean: Boolean;
begin
  Result:=GetAsInteger<>0;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TNumericField.SetAsBoolean(AValue: Boolean);
begin
  SetAsInteger(ord(AValue));
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ ---------------------------------------------------------------------
    TLongintField
  ---------------------------------------------------------------------}


constructor TLongintField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
<<<<<<< HEAD
<<<<<<< HEAD
  SetDataType(ftInteger);
=======
  SetDatatype(ftinteger);
>>>>>>> graemeg/fixes_2_2
=======
  SetDatatype(ftinteger);
>>>>>>> origin/fixes_2_2
  FMinRange:=Low(LongInt);
  FMaxRange:=High(LongInt);
  FValidchars:=['+','-','0'..'9'];
end;

function TLongintField.GetAsFloat: Double;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=GetAsInteger;
end;

function TLongintField.GetAsLargeInt: Largeint;
begin
  Result:=GetAsInteger;
end;

function TLongintField.GetAsInteger: Longint;
=======
=======
>>>>>>> origin/fixes_2_2
  Result:=GetAsLongint;
end;

function TLongintField.GetAsLongint: Longint;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  If Not GetValue(Result) then
    Result:=0;
end;

function TLongintField.GetAsVariant: Variant;

<<<<<<< HEAD
<<<<<<< HEAD
var L : Longint;
=======
Var L : Longint;
>>>>>>> graemeg/fixes_2_2
=======
Var L : Longint;
>>>>>>> origin/fixes_2_2

begin
  If GetValue(L) then
    Result:=L
  else
    Result:=Null;
end;

function TLongintField.GetAsString: string;

<<<<<<< HEAD
<<<<<<< HEAD
var L : Longint;
=======
Var L : Longint;
>>>>>>> graemeg/fixes_2_2
=======
Var L : Longint;
>>>>>>> origin/fixes_2_2

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

function TLongintField.GetValue(var AValue: Longint): Boolean;

<<<<<<< HEAD
<<<<<<< HEAD
var L : Longint;
=======
Var L : Longint;
>>>>>>> graemeg/fixes_2_2
=======
Var L : Longint;
>>>>>>> origin/fixes_2_2
    P : PLongint;

begin
  P:=@L;
  Result:=GetData(P);
  If Result then
    Case Datatype of
<<<<<<< HEAD
<<<<<<< HEAD
      ftInteger,ftAutoinc  : AValue:=Plongint(P)^;
      ftWord               : AValue:=Pword(P)^;
      ftSmallint           : AValue:=PSmallint(P)^;
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
=======
=======
>>>>>>> origin/fixes_2_2
      ftInteger,ftautoinc  : AValue:=Plongint(P)^;
      ftword               : Avalue:=Pword(P)^;
      ftsmallint           : AValue:=PSmallint(P)^;
    end;
end;

procedure TLongintField.SetAsFloat(AValue: Double);

begin
  SetAsLongint(Round(Avalue));
end;

procedure TLongintField.SetAsLongint(AValue: Longint);

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  If CheckRange(AValue) then
    SetData(@AValue)
  else
<<<<<<< HEAD
<<<<<<< HEAD
    if (FMinValue<>0) or (FMaxValue<>0) then
      RangeError(AValue,FMinValue,FMaxValue)
    else
      RangeError(AValue,FMinRange,FMaxRange);
=======
    RangeError(Avalue,FMinrange,FMaxRange);
>>>>>>> graemeg/fixes_2_2
=======
    RangeError(Avalue,FMinrange,FMaxRange);
>>>>>>> origin/fixes_2_2
end;

procedure TLongintField.SetVarValue(const AValue: Variant);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  SetAsInteger(AValue);
=======
  SetAsLongint(AValue);
>>>>>>> graemeg/fixes_2_2
=======
  SetAsLongint(AValue);
>>>>>>> origin/fixes_2_2
end;

procedure TLongintField.SetAsString(const AValue: string);

<<<<<<< HEAD
<<<<<<< HEAD
var L,Code : longint;
=======
Var L,Code : longint;
>>>>>>> graemeg/fixes_2_2
=======
Var L,Code : longint;
>>>>>>> origin/fixes_2_2

begin
  If length(AValue)=0 then
    Clear
  else
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    Val(AValue,L,Code);
    If Code=0 then
      SetAsInteger(L)
    else
      DatabaseErrorFmt(SNotAnInteger,[AValue]);
=======
=======
>>>>>>> origin/fixes_2_2
    Val(AVAlue,L,Code);
    If Code=0 then
      SetAsLongint(L)
    else
      DatabaseErrorFMT(SNotAnInteger,[Avalue]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
end;

Function TLongintField.CheckRange(AValue : longint) : Boolean;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  if (FMinValue<>0) or (FMaxValue<>0) then
    Result := (AValue>=FMinValue) and (AValue<=FMaxValue)
  else
    Result := (AValue>=FMinRange) and (AValue<=FMaxRange);
=======
=======
>>>>>>> origin/fixes_2_2
  result := true;
  if (FMaxValue=0) then
    begin
    if (AValue>FMaxRange) Then result := false;
    end
  else
    if AValue>FMaxValue then result := false;

  if (FMinValue=0) then
    begin
    if (AValue<FMinRange) Then result := false;
    end
  else
    if AValue<FMinValue then result := false;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

{ ---------------------------------------------------------------------
    TLargeintField
  ---------------------------------------------------------------------}


constructor TLargeintField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
<<<<<<< HEAD
<<<<<<< HEAD
  SetDataType(ftLargeint);
=======
  SetDatatype(ftLargeint);
>>>>>>> graemeg/fixes_2_2
=======
  SetDatatype(ftLargeint);
>>>>>>> origin/fixes_2_2
  FMinRange:=Low(Largeint);
  FMaxRange:=High(Largeint);
  FValidchars:=['+','-','0'..'9'];
end;

function TLargeintField.GetAsFloat: Double;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=GetAsLargeInt;
end;

function TLargeintField.GetAsLargeInt: Largeint;
=======
=======
>>>>>>> origin/fixes_2_2
  Result:=GetAsLargeint;
end;

function TLargeintField.GetAsLargeint: Largeint;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  If Not GetValue(Result) then
    Result:=0;
end;

function TLargeIntField.GetAsVariant: Variant;

<<<<<<< HEAD
<<<<<<< HEAD
var L : Largeint;
=======
Var L : Largeint;
>>>>>>> graemeg/fixes_2_2
=======
Var L : Largeint;
>>>>>>> origin/fixes_2_2

begin
  If GetValue(L) then
    Result:=L
  else
    Result:=Null;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TLargeintField.GetAsInteger: Longint;

begin
  Result:=GetAsLargeInt;
=======
=======
>>>>>>> origin/fixes_2_2
function TLargeintField.GetAsLongint: Longint;

begin
  Result:=GetAsLargeint;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function TLargeintField.GetAsString: string;

<<<<<<< HEAD
<<<<<<< HEAD
var L : Largeint;
=======
Var L : Largeint;
>>>>>>> graemeg/fixes_2_2
=======
Var L : Largeint;
>>>>>>> origin/fixes_2_2

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

<<<<<<< HEAD
<<<<<<< HEAD
var l : Largeint;
=======
var l : largeint;
>>>>>>> graemeg/fixes_2_2
=======
var l : largeint;
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
var P : PLargeint;
=======
=======
>>>>>>> origin/fixes_2_2
type
  PLargeint = ^Largeint;

Var P : PLargeint;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  P:=@AValue;
  Result:=GetData(P);
end;

procedure TLargeintField.SetAsFloat(AValue: Double);

begin
<<<<<<< HEAD
<<<<<<< HEAD
  SetAsLargeInt(Round(AValue));
end;

procedure TLargeintField.SetAsLargeInt(AValue: Largeint);
=======
=======
>>>>>>> origin/fixes_2_2
  SetAsLargeint(Round(Avalue));
end;

procedure TLargeintField.SetAsLargeint(AValue: Largeint);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  If CheckRange(AValue) then
    SetData(@AValue)
  else
<<<<<<< HEAD
<<<<<<< HEAD
    RangeError(AValue,FMinValue,FMaxValue);
end;

procedure TLargeintField.SetAsInteger(AValue: Longint);

begin
  SetAsLargeInt(AValue);
=======
=======
>>>>>>> origin/fixes_2_2
    RangeError(Avalue,FMinrange,FMaxRange);
end;

procedure TLargeintField.SetAsLongint(AValue: Longint);

begin
  SetAsLargeint(Avalue);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TLargeintField.SetAsString(const AValue: string);

<<<<<<< HEAD
<<<<<<< HEAD
var L     : Largeint;
    code  : Longint;
=======
Var L     : largeint;
    code  : longint;
>>>>>>> graemeg/fixes_2_2
=======
Var L     : largeint;
    code  : longint;
>>>>>>> origin/fixes_2_2

begin
  If length(AValue)=0 then
    Clear
  else
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    Val(AValue,L,Code);
    If Code=0 then
      SetAsLargeInt(L)
    else
      DatabaseErrorFmt(SNotAnInteger,[AValue]);
=======
=======
>>>>>>> origin/fixes_2_2
    Val(AVAlue,L,Code);
    If Code=0 then
      SetAsLargeint(L)
    else
      DatabaseErrorFMT(SNotAnInteger,[Avalue]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
end;

procedure TLargeintField.SetVarValue(const AValue: Variant);
begin
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
  SetAsLargeint(AValue);
end;

Function TLargeintField.CheckRange(AValue : largeint) : Boolean;

begin
  result := true;
  if (FMaxValue=0) then
    begin
    if (AValue>FMaxRange) Then result := false;
    end
  else
    if AValue>FMaxValue then result := false;

  if (FMinValue=0) then
    begin
    if (AValue<FMinRange) Then result := false;
    end
  else
    if AValue<FMinValue then result := false;
end;

Procedure TLargeintField.SetMaxValue (AValue : largeint);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  If (AValue>=FMinRange) and (AValue<=FMaxRange) then
    FMaxValue:=AValue
  else
    RangeError(AValue,FMinRange,FMaxRange);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TLargeintField.SetMinValue (AValue : Largeint);
=======
Procedure TLargeintField.SetMinValue (AValue : largeint);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TLargeintField.SetMinValue (AValue : largeint);
>>>>>>> origin/fixes_2_2

begin
  If (AValue>=FMinRange) and (AValue<=FMaxRange) then
    FMinValue:=AValue
  else
    RangeError(AValue,FMinRange,FMaxRange);
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
  FValidchars:=['+','0'..'9'];
end;

{ TAutoIncField }

constructor TAutoIncField.Create(AOwner: TComponent);

begin
  Inherited Create(AOWner);
  SetDataType(ftAutoInc);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  FReadOnly:=True;
  FProviderFlags:=FProviderFlags-[pfInUpdate];
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

Procedure TAutoIncField.SetAsInteger(AValue: Longint);

begin
  // Some databases allows insertion of explicit values into identity columns
  // (some of them also allows (some not) updating identity columns)
  // So allow it at client side and leave check for server side
  //if not(FDataSet.State in [dsFilter,dsSetKey,dsInsert]) then
  //  DataBaseError(SCantSetAutoIncFields);
  inherited;
=======
=======
>>>>>>> origin/fixes_2_2
  FReadOnly:=True;
end;

Procedure TAutoIncField.SetAsLongint(AValue : Longint);

begin
  DataBaseError(SCantSetAutoIncfields);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

{ TFloatField }

procedure TFloatField.SetCurrency(const AValue: Boolean);
begin
  if FCurrency=AValue then exit;
  FCurrency:=AValue;
end;

<<<<<<< HEAD
<<<<<<< HEAD
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

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TFloatField.GetAsFloat: Double;

begin
  If Not GetData(@Result) Then
    Result:=0.0;
end;

function TFloatField.GetAsVariant: Variant;

<<<<<<< HEAD
<<<<<<< HEAD
var f : Double;
=======
Var f : Double;
>>>>>>> graemeg/fixes_2_2
=======
Var f : Double;
>>>>>>> origin/fixes_2_2

begin
  If GetData(@f) then
    Result := f
  else
    Result:=Null;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TFloatField.GetAsLargeInt: LargeInt;
begin
  Result:=Round(GetAsFloat);
end;

function TFloatField.GetAsInteger: Longint;
=======
function TFloatField.GetAsLongint: Longint;
>>>>>>> graemeg/fixes_2_2
=======
function TFloatField.GetAsLongint: Longint;
>>>>>>> origin/fixes_2_2

begin
  Result:=Round(GetAsFloat);
end;

function TFloatField.GetAsString: string;

<<<<<<< HEAD
<<<<<<< HEAD
var f : Double;

begin
  If GetData(@f) then
    Result:=FloatToStr(f)
=======
=======
>>>>>>> origin/fixes_2_2
Var R : Double;

begin
  If GetData(@R) then
    Result:=FloatToStr(R)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  else
    Result:='';
end;

function TFloatField.GetDataSize: Integer;

begin
  Result:=SizeOf(Double);
end;

procedure TFloatField.GetText(var TheText: string; ADisplayText: Boolean);

Var
    fmt : string;
    E : Double;
    Digits : integer;
    ff: TFloatFormat;

begin
  TheText:='';
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
    TheText:=FormatFloat(fmt,E)
  else
    TheText:=FloatToStrF(E,ff,FPrecision,Digits);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TFloatField.SetAsBCD(const AValue: TBCD);
begin
  SetAsFloat(BCDToDouble(AValue));
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TFloatField.SetAsFloat(AValue: Double);

begin
  If CheckRange(AValue) then
<<<<<<< HEAD
<<<<<<< HEAD
    SetData(@AValue)
=======
    SetData(@Avalue)
>>>>>>> graemeg/fixes_2_2
=======
    SetData(@Avalue)
>>>>>>> origin/fixes_2_2
  else
    RangeError(AValue,FMinValue,FMaxValue);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TFloatField.SetAsLargeInt(AValue: LargeInt);
begin
  SetAsFloat(AValue);
end;

procedure TFloatField.SetAsInteger(AValue: Longint);

begin
  SetAsFloat(AValue);
=======
=======
>>>>>>> origin/fixes_2_2
procedure TFloatField.SetAsLongint(AValue: Longint);

begin
  SetAsFloat(Avalue);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TFloatField.SetAsString(const AValue: string);

<<<<<<< HEAD
<<<<<<< HEAD
var f : Double;
=======
Var R : Double;
>>>>>>> graemeg/fixes_2_2
=======
Var R : Double;
>>>>>>> origin/fixes_2_2

begin
  If (AValue='') then
    Clear
  else  
    try
<<<<<<< HEAD
<<<<<<< HEAD
      f := StrToFloat(AValue);
      SetAsFloat(f);
=======
      R := StrToFloat(AValue);
      SetAsFloat(R);
>>>>>>> graemeg/fixes_2_2
=======
      R := StrToFloat(AValue);
      SetAsFloat(R);
>>>>>>> origin/fixes_2_2
    except
      DatabaseErrorFmt(SNotAFloat, [AValue]);
    end;
end;

procedure TFloatField.SetVarValue(const AValue: Variant);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  SetAsFloat(AValue);
=======
  SetAsFloat(Avalue);
>>>>>>> graemeg/fixes_2_2
=======
  SetAsFloat(Avalue);
>>>>>>> origin/fixes_2_2
end;

constructor TFloatField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
<<<<<<< HEAD
<<<<<<< HEAD
  SetDataType(ftFloat);
=======
  SetDatatype(ftfloat);
>>>>>>> graemeg/fixes_2_2
=======
  SetDatatype(ftfloat);
>>>>>>> origin/fixes_2_2
  FPrecision:=15;
  FValidChars := [DecimalSeparator, '+', '-', '0'..'9', 'E', 'e'];
end;

Function TFloatField.CheckRange(AValue : Double) : Boolean;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  If (FMinValue<>0) or (FMaxValue<>0) then
    Result:=(AValue>=FMinValue) and (AValue<=FMaxValue)
=======
  If (FMinValue<>0) or (FmaxValue<>0) then
    Result:=(AValue>=FMinValue) and (AVAlue<=FMAxValue)
>>>>>>> graemeg/fixes_2_2
=======
  If (FMinValue<>0) or (FmaxValue<>0) then
    Result:=(AValue>=FMinValue) and (AVAlue<=FMAxValue)
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
    Result := b
=======
    result := b
>>>>>>> graemeg/fixes_2_2
=======
    result := b
>>>>>>> origin/fixes_2_2
  else
    Result:=False;
end;

function TBooleanField.GetAsVariant: Variant;

<<<<<<< HEAD
<<<<<<< HEAD
var b : wordbool;
=======
Var b : wordbool;
>>>>>>> graemeg/fixes_2_2
=======
Var b : wordbool;
>>>>>>> origin/fixes_2_2

begin
  If GetData(@b) then
    Result := b
  else
    Result:=Null;
end;

function TBooleanField.GetAsString: string;

<<<<<<< HEAD
<<<<<<< HEAD
var B : wordbool;

begin
  If GetData(@B) then
=======
=======
>>>>>>> origin/fixes_2_2
Var B : wordbool;

begin
  If Getdata(@B) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TBooleanField.GetAsInteger: Longint;
begin
  Result := ord(GetAsBoolean);
end;

procedure TBooleanField.SetAsInteger(AValue: Longint);
begin
  SetAsBoolean(AValue<>0);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TBooleanField.GetAsInteger: integer;
begin
   if GetAsBoolean then
    Result:=1
   else
    Result:=0;
end;

procedure TBooleanField.SetAsInteger(AValue: Integer);
begin
  SetAsBoolean(avalue<>0);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TBooleanField.SetAsBoolean(AValue: Boolean);

var b : wordbool;

begin
  b := AValue;
  SetData(@b);
end;

procedure TBooleanField.SetAsString(const AValue: string);

<<<<<<< HEAD
<<<<<<< HEAD
var Temp : string;
=======
Var Temp : string;
>>>>>>> graemeg/fixes_2_2
=======
Var Temp : string;
>>>>>>> origin/fixes_2_2

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

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TBooleanField.SetDisplayValues(const AValue : String);

var I : longint;
=======
Procedure TBooleanField.SetDisplayValues(AValue : String);

Var I : longint;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TBooleanField.SetDisplayValues(AValue : String);

Var I : longint;
>>>>>>> origin/fixes_2_2

begin
  If FDisplayValues<>AValue then
    begin
    I:=Pos(';',AValue);
    If (I<2) or (I=Length(AValue)) then
      DatabaseErrorFmt(SInvalidDisplayValues,[AValue]);
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

<<<<<<< HEAD
<<<<<<< HEAD
var d : tDateTime;

begin
  If GetData(@d,False) then
=======
=======
>>>>>>> origin/fixes_2_2
Var d : tDateTime;

begin
  If Getdata(@d,False) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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


procedure TDateTimeField.GetText(var TheText: string; ADisplayText: Boolean);

<<<<<<< HEAD
<<<<<<< HEAD
var R : TDateTime;
    F : String;

begin
  If Not GetData(@R,False) then
=======
=======
>>>>>>> origin/fixes_2_2
Var R : TDateTime;
    F : String;

begin
  If Not Getdata(@R,False) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    TheText:=''
  else
    begin
    If (ADisplayText) and (Length(FDisplayFormat)<>0) then
      F:=FDisplayFormat
    else
      Case DataType of
<<<<<<< HEAD
<<<<<<< HEAD
       ftTime : F:=LongTimeFormat;
=======
       ftTime : F:=ShortTimeFormat;
>>>>>>> graemeg/fixes_2_2
=======
       ftTime : F:=ShortTimeFormat;
>>>>>>> origin/fixes_2_2
       ftDate : F:=ShortDateFormat;
      else
       F:='c'
      end;
    TheText:=FormatDateTime(F,R);
    end;
end;


procedure TDateTimeField.SetAsDateTime(AValue: TDateTime);

begin
<<<<<<< HEAD
<<<<<<< HEAD
  SetData(@AValue,False);
=======
  SetData(@Avalue,False);
>>>>>>> graemeg/fixes_2_2
=======
  SetData(@Avalue,False);
>>>>>>> origin/fixes_2_2
end;


procedure TDateTimeField.SetAsFloat(AValue: Double);

begin
  SetAsDateTime(AValue);
end;


procedure TDateTimeField.SetAsString(const AValue: string);

<<<<<<< HEAD
<<<<<<< HEAD
var R : TDateTime;

begin
  if AValue<>'' then
    begin
    R:=StrToDateTime(AValue);
    SetData(@R,False);
    end
  else
    SetData(Nil);
=======
=======
>>>>>>> origin/fixes_2_2
Var R : TDateTime;

begin
  R:=StrToDateTime(AVAlue);
  SetData(@R,False);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
var R : TDateTime;
begin
  if AValue='' then
    Clear    // set to NULL
  else
    begin
    R:=StrToTime(AValue);
    SetData(@R,False);
    end;
=======
=======
>>>>>>> origin/fixes_2_2
Var R : TDateTime;
begin
  R:=StrToTime(AVAlue);
  SetData(@R);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;



{ TBinaryField }

class procedure TBinaryField.CheckTypeSize(AValue: Longint);

begin
  // Just check for really invalid stuff; actual size is
  // dependent on the record...
  If AValue<1 then
<<<<<<< HEAD
<<<<<<< HEAD
    DatabaseErrorFmt(SInvalidFieldSize,[AValue]);
end;

function TBinaryField.GetAsBytes: TBytes;
begin
  if not GetValue(Result) then
    SetLength(Result, 0);
=======
    DatabaseErrorfmt(SInvalidFieldSize,[Avalue]);
>>>>>>> graemeg/fixes_2_2
=======
    DatabaseErrorfmt(SInvalidFieldSize,[Avalue]);
>>>>>>> origin/fixes_2_2
end;


function TBinaryField.GetAsString: string;
<<<<<<< HEAD
<<<<<<< HEAD
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
=======

begin
  Setlength(Result,DataSize);
  GetData(Pointer(Result));
>>>>>>> graemeg/fixes_2_2
end;


procedure TBinaryField.GetText(var TheText: string; ADisplayText: Boolean);

=======

begin
  Setlength(Result,DataSize);
  GetData(Pointer(Result));
end;


procedure TBinaryField.GetText(var TheText: string; ADisplayText: Boolean);

>>>>>>> origin/fixes_2_2
begin
  TheText:=GetAsString;
end;


<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
procedure TBinaryField.SetAsString(const AValue: string);

Var Buf : PChar;
    Allocated : Boolean;

begin
  Allocated:=False;
  If Length(AVAlue)=DataSize then
    Buf:=PChar(Avalue)
  else
    begin
    GetMem(Buf,DataSize);
    Move(Pchar(Avalue)[0],Buf^,DataSize);
    Allocated:=True;
    end;
  SetData(Buf);
  If Allocated then
    FreeMem(Buf,DataSize);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


procedure TBinaryField.SetText(const AValue: string);

begin
<<<<<<< HEAD
<<<<<<< HEAD
  SetAsString(AValue);
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
=======
=======
>>>>>>> origin/fixes_2_2
  SetAsString(Avalue);
end;

procedure TBinaryField.SetVarValue(const AValue: Variant);
begin
  SetAsString(Avalue);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
  SetDataType(ftVarBytes);
=======
  SetDataType(ftvarbytes);
>>>>>>> graemeg/fixes_2_2
=======
  SetDataType(ftvarbytes);
>>>>>>> origin/fixes_2_2
  Size:=16;
end;

{ TBCDField }

class procedure TBCDField.CheckTypeSize(AValue: Longint);

begin
  If not (AValue in [0..4]) then
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
    DatabaseErrorfmt(SInvalidFieldSize,[Avalue]);
>>>>>>> graemeg/fixes_2_2
=======
    DatabaseErrorfmt(SInvalidFieldSize,[Avalue]);
>>>>>>> origin/fixes_2_2
end;

function TBCDField.GetAsCurrency: Currency;

begin
  if not GetData(@Result) then
    result := 0;
end;

<<<<<<< HEAD
function TBCDField.GetAsVariant: Variant;

<<<<<<< HEAD
var c : system.Currency;
=======
Var c : system.Currency;
>>>>>>> graemeg/fixes_2_2

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


<<<<<<< HEAD
function TBCDField.GetAsInteger: Longint;
=======
function TBCDField.GetAsLongint: Longint;
>>>>>>> graemeg/fixes_2_2

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
<<<<<<< HEAD
  if Precision > 0 then Result := Precision+1
  else Result := 10;
=======
  if precision > 0 then result := precision
    else result := 10;
>>>>>>> graemeg/fixes_2_2
end;

procedure TBCDField.GetText(var TheText: string; ADisplayText: Boolean);
var
  c : system.currency;
  fmt: String;
begin
  if GetData(@C) then begin
    if aDisplayText or (FEditFormat='') then
      fmt := FDisplayFormat
    else
      fmt := FEditFormat;
    if fmt<>'' then
      TheText := FormatFloat(fmt,C)
    else if fCurrency then begin
      if aDisplayText then
        TheText := FloatToStrF(C, ffCurrency, FPrecision, 2{digits?})
      else
        TheText := FloatToStrF(C, ffFixed, FPrecision, 2{digits?});
    end else
      TheText := FloatToStrF(C, ffGeneral, FPrecision, 0{digits?});
  end else
    TheText := '';
end;

<<<<<<< HEAD
procedure TBCDField.SetAsBCD(const AValue: TBCD);
var
  c:system.currency;
begin
  if BCDToCurr(AValue,c) then
    SetAsCurrency(c);
end;

=======
>>>>>>> graemeg/fixes_2_2
procedure TBCDField.SetAsCurrency(AValue: Currency);

begin
  If CheckRange(AValue) then
<<<<<<< HEAD
    SetData(@AValue)
  else
    RangeError(AValue,FMinValue,FMaxValue);
=======
    setdata(@AValue)
  else
    RangeError(AValue,FMinValue,FMaxvalue);
>>>>>>> graemeg/fixes_2_2
end;

procedure TBCDField.SetVarValue(const AValue: Variant);
begin
  SetAsCurrency(AValue);
end;

Function TBCDField.CheckRange(AValue : Currency) : Boolean;

begin
<<<<<<< HEAD
  If (FMinValue<>0) or (FMaxValue<>0) then
    Result:=(AValue>=FMinValue) and (AValue<=FMaxValue)
=======
  If (FMinValue<>0) or (FmaxValue<>0) then
    Result:=(AValue>=FMinValue) and (AVAlue<=FMaxValue)
>>>>>>> graemeg/fixes_2_2
  else
    Result:=True;
end;

procedure TBCDField.SetAsFloat(AValue: Double);

begin
  SetAsCurrency(AValue);
end;


<<<<<<< HEAD
procedure TBCDField.SetAsInteger(AValue: Longint);
=======
procedure TBCDField.SetAsLongint(AValue: Longint);
>>>>>>> graemeg/fixes_2_2

begin
  SetAsCurrency(AValue);
end;


procedure TBCDField.SetAsString(const AValue: string);

begin
<<<<<<< HEAD
  if AValue='' then
    Clear    // set to NULL
  else
    SetAsCurrency(strtocurr(AValue));
=======
  SetAsCurrency(strtocurr(AValue));
>>>>>>> graemeg/fixes_2_2
end;

constructor TBCDField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
<<<<<<< HEAD
<<<<<<< HEAD
  FMaxValue := 0;
  FMinValue := 0;
=======
  FMaxvalue := 0;
  FMinvalue := 0;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  FValidChars := [DecimalSeparator, '+', '-', '0'..'9'];
  SetDataType(ftBCD);
  Precision := 18;
  Size := 4;
end;


{ TFMTBCDField }
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======

class procedure TFMTBCDField.CheckTypeSize(AValue: Longint);
begin
  If AValue > MAXFMTBcdFractionSize then
    DatabaseErrorfmt(SInvalidFieldSize,[AValue]);
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
  Precision := 15; //default number of digits
  Size:=4; //default number of digits after decimal place
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

function TFMTBCDField.GetAsLongint: Longint;
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

procedure TFMTBCDField.GetText(var TheText: string; ADisplayText: Boolean);
var
  bcd: TBCD;
  fmt: String;
begin
  if GetData(@bcd) then begin
    if aDisplayText or (FEditFormat='') then
      fmt := FDisplayFormat
    else
      fmt := FEditFormat;
    if fmt<>'' then
      TheText := BCDToStr(bcd)
      //TheText := FormatBCD(fmt,bcd) //uncomment when formatBCD in fmtbcd.pp will be implemented
    else if fCurrency then begin
      if aDisplayText then
        TheText := BcdToStrF(bcd, ffCurrency, FPrecision, 2)
      else
        TheText := BcdToStrF(bcd, ffFixed, FPrecision, 2);
    end else
      TheText := BcdToStrF(bcd, ffGeneral, FPrecision, FSize);
  end else
    TheText := '';
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


procedure TFMTBCDField.SetAsLongint(AValue: Longint);
begin
  SetAsBCD(IntegerToBCD(AValue));
end;


procedure TFMTBCDField.SetAsString(const AValue: string);
begin
  SetAsBCD(StrToBCD(AValue));
end;


{ TBlobField }
>>>>>>> origin/cpstrnew

class procedure TFMTBCDField.CheckTypeSize(AValue: Longint);
begin
  If AValue > MAXFMTBcdFractionSize then
    DatabaseErrorfmt(SInvalidFieldSize,[AValue]);
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
  Precision := 15; //default number of digits
  Size:=4; //default number of digits after decimal place
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

function TFMTBCDField.GetAsLongint: Longint;
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

procedure TFMTBCDField.GetText(var TheText: string; ADisplayText: Boolean);
var
  bcd: TBCD;
  fmt: String;
begin
  if GetData(@bcd) then begin
    if aDisplayText or (FEditFormat='') then
      fmt := FDisplayFormat
    else
      fmt := FEditFormat;
    if fmt<>'' then
      TheText := BCDToStr(bcd)
      //TheText := FormatBCD(fmt,bcd) //uncomment when formatBCD in fmtbcd.pp will be implemented
    else if fCurrency then begin
      if aDisplayText then
        TheText := BcdToStrF(bcd, ffCurrency, FPrecision, 2)
      else
        TheText := BcdToStrF(bcd, ffFixed, FPrecision, 2);
    end else
      TheText := BcdToStrF(bcd, ffGeneral, FPrecision, FSize);
  end else
    TheText := '';
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


procedure TFMTBCDField.SetAsLongint(AValue: Longint);
begin
  SetAsBCD(IntegerToBCD(AValue));
end;


procedure TFMTBCDField.SetAsString(const AValue: string);
begin
  SetAsBCD(StrToBCD(AValue));
end;


{ TBlobField }
>>>>>>> graemeg/cpstrnew

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

procedure TFMTBCDField.GetText(var TheText: string; ADisplayText: Boolean);
var
  bcd: TBCD;
  fmt: String;
begin
  if GetData(@bcd) then begin
    if aDisplayText or (FEditFormat='') then
      fmt := FDisplayFormat
    else
      fmt := FEditFormat;
    if fmt<>'' then
      TheText := FormatBCD(fmt,bcd)
    else if fCurrency then begin
      if aDisplayText then
        TheText := BcdToStrF(bcd, ffCurrency, FPrecision, 2)
      else
        TheText := BcdToStrF(bcd, ffFixed, FPrecision, 2);
    end else
      TheText := BcdToStrF(bcd, ffGeneral, FPrecision, FSize);
  end else
    TheText := '';
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

procedure TFMTBCDField.SetAsInteger(AValue: Longint);
begin
  SetAsBCD(IntegerToBCD(AValue));
end;

procedure TFMTBCDField.SetAsString(const AValue: string);
begin
  if AValue='' then
    Clear    // set to NULL
  else
    SetAsBCD(StrToBCD(AValue));
end;


{ TFMTBCDField }

class procedure TFMTBCDField.CheckTypeSize(AValue: Longint);
begin
  If AValue > MAXFMTBcdFractionSize then
    DatabaseErrorfmt(SInvalidFieldSize,[AValue]);
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
  Precision := 15; //default number of digits
  Size:=4; //default number of digits after decimal place
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

function TFMTBCDField.GetAsLongint: Longint;
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

procedure TFMTBCDField.GetText(var TheText: string; ADisplayText: Boolean);
var
  bcd: TBCD;
  fmt: String;
begin
  if GetData(@bcd) then begin
    if aDisplayText or (FEditFormat='') then
      fmt := FDisplayFormat
    else
      fmt := FEditFormat;
    if fmt<>'' then
      TheText := BCDToStr(bcd)
      //TheText := FormatBCD(fmt,bcd) //uncomment when formatBCD in fmtbcd.pp will be implemented
    else if fCurrency then begin
      if aDisplayText then
        TheText := BcdToStrF(bcd, ffCurrency, FPrecision, 2)
      else
        TheText := BcdToStrF(bcd, ffFixed, FPrecision, 2);
    end else
      TheText := BcdToStrF(bcd, ffGeneral, FPrecision, FSize);
  end else
    TheText := '';
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


procedure TFMTBCDField.SetAsLongint(AValue: Longint);
begin
  SetAsBCD(IntegerToBCD(AValue));
end;


procedure TFMTBCDField.SetAsString(const AValue: string);
begin
  SetAsBCD(StrToBCD(AValue));
end;


{ TFMTBCDField }

class procedure TFMTBCDField.CheckTypeSize(AValue: Longint);
begin
  If AValue > MAXFMTBcdFractionSize then
    DatabaseErrorfmt(SInvalidFieldSize,[AValue]);
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
  Precision := 15; //default number of digits
  Size:=4; //default number of digits after decimal place
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

function TFMTBCDField.GetAsLongint: Longint;
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

procedure TFMTBCDField.GetText(var TheText: string; ADisplayText: Boolean);
var
  bcd: TBCD;
  fmt: String;
begin
  if GetData(@bcd) then begin
=======
function TBCDField.GetAsVariant: Variant;

Var c : system.Currency;

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


function TBCDField.GetAsLongint: Longint;

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
  if precision > 0 then result := precision
    else result := 10;
end;

procedure TBCDField.GetText(var TheText: string; ADisplayText: Boolean);
var
  c : system.currency;
  fmt: String;
begin
  if GetData(@C) then begin
>>>>>>> origin/fixes_2_2
    if aDisplayText or (FEditFormat='') then
      fmt := FDisplayFormat
    else
      fmt := FEditFormat;
    if fmt<>'' then
<<<<<<< HEAD
      TheText := BCDToStr(bcd)
      //TheText := FormatBCD(fmt,bcd) //uncomment when formatBCD in fmtbcd.pp will be implemented
    else if fCurrency then begin
      if aDisplayText then
        TheText := BcdToStrF(bcd, ffCurrency, FPrecision, 2)
      else
        TheText := BcdToStrF(bcd, ffFixed, FPrecision, 2);
    end else
      TheText := BcdToStrF(bcd, ffGeneral, FPrecision, FSize);
=======
      TheText := FormatFloat(fmt,C)
    else if fCurrency then begin
      if aDisplayText then
        TheText := FloatToStrF(C, ffCurrency, FPrecision, 2{digits?})
      else
        TheText := FloatToStrF(C, ffFixed, FPrecision, 2{digits?});
    end else
      TheText := FloatToStrF(C, ffGeneral, FPrecision, 0{digits?});
>>>>>>> origin/fixes_2_2
  end else
    TheText := '';
end;

<<<<<<< HEAD
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
=======
procedure TBCDField.SetAsCurrency(AValue: Currency);

begin
  If CheckRange(AValue) then
    setdata(@AValue)
  else
    RangeError(AValue,FMinValue,FMaxvalue);
end;

procedure TBCDField.SetVarValue(const AValue: Variant);
begin
  SetAsCurrency(AValue);
end;

Function TBCDField.CheckRange(AValue : Currency) : Boolean;

begin
  If (FMinValue<>0) or (FmaxValue<>0) then
    Result:=(AValue>=FMinValue) and (AVAlue<=FMaxValue)
>>>>>>> origin/fixes_2_2
  else
    Result:=True;
end;

<<<<<<< HEAD
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


procedure TFMTBCDField.SetAsLongint(AValue: Longint);
begin
  SetAsBCD(IntegerToBCD(AValue));
end;


procedure TFMTBCDField.SetAsString(const AValue: string);
begin
  SetAsBCD(StrToBCD(AValue));
end;


{ TBlobField }

function TBlobField.GetBlobStream(Mode: TBlobStreamMode): TStream;

begin
  Result:=FDataset.CreateBlobStream(Self,Mode);
end;

function TBlobField.GetBlobType: TBlobType;
begin
  result:= TBlobType(DataType);
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
var
  Stream : TStream;
  Len    : Integer;
  S : String;
  
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

function TBlobField.GetAsWideString: WideString;
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
=======
=======
procedure TBCDField.SetAsFloat(AValue: Double);

begin
  SetAsCurrency(AValue);
end;


procedure TBCDField.SetAsLongint(AValue: Longint);

begin
  SetAsCurrency(AValue);
end;


procedure TBCDField.SetAsString(const AValue: string);

begin
  SetAsCurrency(strtocurr(AValue));
end;

constructor TBCDField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
>>>>>>> origin/fixes_2_2
  FMaxvalue := 0;
  FMinvalue := 0;
  SetDataType(ftBCD);
  FPrecision := 15;
  Size:=4;
end;


{ TBlobField }

Function TBlobField.GetBlobStream(Mode : TBlobStreamMode) : TStream;

begin
  Result:=FDataset.CreateBlobStream(Self,Mode);
end;

procedure TBlobField.FreeBuffers;

begin
end;


function TBlobField.GetAsString: string;
var
  Stream : TStream;
  Len    : Integer;
begin
  Stream := GetBlobStream(bmRead);
  if Stream <> nil then
    With Stream do
      try
        Len := Size;
        SetLength(Result, Len);
        if Len > 0 then
          ReadBuffer(Result[1], Len);
      finally
        Free
      end
  else
    Result := '';
end;

function TBlobField.GetAsWideString: WideString;
var
  Stream : TStream;
  Len    : Integer;
begin
  Stream := GetBlobStream(bmRead);
  if Stream <> nil then
    With Stream do
      try
        Len := Size;
        SetLength(Result,Len div 2);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        if Len > 0 then
          ReadBuffer(Result[1] ,Len);
      finally
        Free
      end
  else
    Result := '';
end;

function TBlobField.GetAsVariant: Variant;
<<<<<<< HEAD
<<<<<<< HEAD
var s : string;
=======

Var s : string;

>>>>>>> graemeg/fixes_2_2
=======

Var s : string;

>>>>>>> origin/fixes_2_2
begin
  if not GetIsNull then
    begin
    s := GetAsString;
    result := s;
    end
<<<<<<< HEAD
<<<<<<< HEAD
  else
    result := Null;
=======
  else result := Null;
>>>>>>> graemeg/fixes_2_2
=======
  else result := Null;
>>>>>>> origin/fixes_2_2
end;


function TBlobField.GetBlobSize: Longint;
var
  Stream: TStream;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Stream := GetBlobStream(bmRead);
  if Stream <> nil then
    with Stream do
=======
  Stream := GetBlobStream(bmread);
  if Stream <> nil then
    With Stream do
>>>>>>> graemeg/fixes_2_2
=======
  Stream := GetBlobStream(bmread);
  if Stream <> nil then
    With Stream do
>>>>>>> origin/fixes_2_2
      try
        Result:=Size;
      finally
        Free;
      end
  else
<<<<<<< HEAD
<<<<<<< HEAD
    Result := 0;
=======
    result := 0;
>>>>>>> graemeg/fixes_2_2
=======
    result := 0;
>>>>>>> origin/fixes_2_2
end;


function TBlobField.GetIsNull: Boolean;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  if Not Modified then
    Result:= inherited GetIsNull
  else
    with GetBlobStream(bmRead) do
      try
        Result:=(Size=0);
      finally
=======
=======
>>>>>>> origin/fixes_2_2
  If Not Modified then
    result:= inherited GetIsnull
  else
    With GetBlobStream(bmread) do
      try
        Result:=(Size=0);
      Finally
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        Free;
      end;
end;


procedure TBlobField.GetText(var TheText: string; ADisplayText: Boolean);

begin
  TheText:=inherited GetAsString;
end;

<<<<<<< HEAD
<<<<<<< HEAD
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

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

procedure TBlobField.SetAsString(const AValue: string);
var
  Len : Integer;
<<<<<<< HEAD
<<<<<<< HEAD
  S : String;
  
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
=======
=======
>>>>>>> origin/fixes_2_2
begin
  With GetBlobStream(bmwrite) do
    try
      Len := Length(Avalue);
      if Len > 0 then
        WriteBuffer(aValue[1], Len);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    finally
      Free;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
procedure TBlobField.SetAsWideString(const AValue: WideString);
var
  Len : Integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  with GetBlobStream(bmWrite) do
    try
      Len := Length(AValue) * 2;
      if Len > 0 then
        WriteBuffer(AValue[1], Len);
=======
=======
>>>>>>> origin/fixes_2_2
  With GetBlobStream(bmwrite) do
    try
      Len := Length(Avalue) * 2;
      if Len > 0 then
        WriteBuffer(aValue[1], Len);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    finally
      Free;
    end;
end;


procedure TBlobField.SetText(const AValue: string);

begin
  SetAsString(AValue);
end;

procedure TBlobField.SetVarValue(const AValue: Variant);
begin
  SetAsString(AValue);
end;


constructor TBlobField.Create(AOwner: TComponent);

begin
  Inherited Create(AOWner);
  SetDataType(ftBlob);
end;


procedure TBlobField.Clear;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  GetBlobStream(bmWrite).Free;
=======
  GetBlobStream(bmWrite).free;
>>>>>>> graemeg/fixes_2_2
=======
  GetBlobStream(bmWrite).free;
>>>>>>> origin/fixes_2_2
end;


class function TBlobField.IsBlob: Boolean;

begin
  Result:=True;
end;


procedure TBlobField.LoadFromFile(const FileName: string);
<<<<<<< HEAD
<<<<<<< HEAD
var S : TFileStream;
begin
  S:=TFileStream.Create(FileName,fmOpenRead or fmShareDenyWrite);
=======
=======
>>>>>>> origin/fixes_2_2

Var S : TFileStream;

begin
  S:=TFileStream.Create(FileName,fmOpenRead);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  try
    LoadFromStream(S);
  finally
    S.Free;
  end;
end;


procedure TBlobField.LoadFromStream(Stream: TStream);

begin
<<<<<<< HEAD
<<<<<<< HEAD
  with GetBlobStream(bmWrite) do
    try
=======
  With GetBlobStream(bmWrite) do
    Try
>>>>>>> graemeg/fixes_2_2
=======
  With GetBlobStream(bmWrite) do
    Try
>>>>>>> origin/fixes_2_2
      CopyFrom(Stream,0);
    finally
      Free;
    end;
end;


procedure TBlobField.SaveToFile(const FileName: string);
<<<<<<< HEAD
<<<<<<< HEAD
var S : TFileStream;
=======

Var S : TFileStream;

>>>>>>> graemeg/fixes_2_2
=======

Var S : TFileStream;

>>>>>>> origin/fixes_2_2
begin
  S:=TFileStream.Create(FileName,fmCreate);
  try
    SaveToStream(S);
  finally
    S.Free;
  end;
end;


procedure TBlobField.SaveToStream(Stream: TStream);
<<<<<<< HEAD
<<<<<<< HEAD
var S : TStream;
begin
  S:=GetBlobStream(bmRead);
  Try
    If Assigned(S) then
      Stream.CopyFrom(S,0);
=======
=======
>>>>>>> origin/fixes_2_2

Var S : TStream;

begin
  S:=GetBlobStream(bmRead);
  Try
    Stream.CopyFrom(S,0);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  finally
    S.Free;
  end;
end;

procedure TBlobField.SetFieldType(AValue: TFieldType);
<<<<<<< HEAD
<<<<<<< HEAD
begin
  if AValue in ftBlobTypes then
    SetDataType(AValue);
=======
=======
>>>>>>> origin/fixes_2_2

begin
  If AValue in [Low(TBlobType)..High(TBlobType)] then
    SetDatatype(Avalue);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

{ TMemoField }

constructor TMemoField.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  SetDataType(ftMemo);
end;

function TMemoField.GetAsWideString: WideString;
begin
  Result := GetAsString;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TMemoField.SetAsWideString(const AValue: WideString);
begin
  SetAsString(AValue);
=======
procedure TMemoField.SetAsWideString(const aValue: WideString);
begin
  SetAsString(aValue);
>>>>>>> graemeg/fixes_2_2
=======
procedure TMemoField.SetAsWideString(const aValue: WideString);
begin
  SetAsString(aValue);
>>>>>>> origin/fixes_2_2
end;

{ TWideMemoField }

constructor TWideMemoField.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  SetDataType(ftWideMemo);
end;

function TWideMemoField.GetAsString: string;
begin
  Result := GetAsWideString;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TWideMemoField.SetAsString(const AValue: string);
begin
  SetAsWideString(AValue);
end;

function TWideMemoField.GetAsVariant: Variant;
var s : string;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TWideMemoField.SetAsString(const aValue: string);
begin
  SetAsWideString(aValue);
end;

function TWideMemoField.GetAsVariant: Variant;

Var s : string;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  if not GetIsNull then
    begin
    s := GetAsWideString;
    result := s;
    end
  else result := Null;
end;

procedure TWideMemoField.SetVarValue(const AValue: Variant);
begin
  SetAsWideString(AValue);
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
<<<<<<< HEAD
<<<<<<< HEAD
  if AValue <> 38 then
=======
  if aValue <> 38 then
>>>>>>> graemeg/fixes_2_2
=======
  if aValue <> 38 then
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
procedure TGuidField.SetAsGuid(const AValue: TGUID);
begin
  SetAsString(GuidToString(AValue));
=======
procedure TGuidField.SetAsGuid(const aValue: TGUID);
begin
  SetAsString(GuidToString(aValue));
>>>>>>> graemeg/fixes_2_2
=======
procedure TGuidField.SetAsGuid(const aValue: TGUID);
begin
  SetAsString(GuidToString(aValue));
>>>>>>> origin/fixes_2_2
end;

function TVariantField.GetDefaultWidth: Integer;
begin
  Result := 15;
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

<<<<<<< HEAD
<<<<<<< HEAD
procedure TVariantField.SetAsInteger(AValue: Longint);
=======
procedure TVariantField.SetAsInteger(aValue: Longint);
>>>>>>> graemeg/fixes_2_2
=======
procedure TVariantField.SetAsInteger(aValue: Longint);
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2

{ TFields }

Constructor TFields.Create(ADataset : TDataset);

begin
  FDataSet:=ADataset;
  FFieldList:=TList.Create;
  FValidFieldKinds:=[fkData..fkInternalcalc];
end;

Destructor TFields.Destroy;

begin
  if FFieldList <> nil then Clear;
  FFieldList.Free;
  inherited Destroy;
end;

Procedure Tfields.Changed;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  if (FDataSet <> nil) and not (csDestroying in FDataSet.ComponentState) and FDataset.Active then
    FDataSet.DataEvent(deFieldListChange, 0);
  If Assigned(FOnChange) then
    FOnChange(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TFields.CheckfieldKind(Fieldkind: TFieldKind; Field: TField);
=======
Procedure TFields.CheckfieldKind(Fieldkind : TFieldKind; Field : TField);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TFields.CheckfieldKind(Fieldkind : TFieldKind; Field : TField);
>>>>>>> origin/fixes_2_2

begin
  If Not (FieldKind in ValidFieldKinds) Then
    DatabaseErrorFmt(SInvalidFieldKind,[Field.FieldName]);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TFields.GetCount: Longint;
=======
Function Tfields.GetCount : Longint;
>>>>>>> graemeg/fixes_2_2
=======
Function Tfields.GetCount : Longint;
>>>>>>> origin/fixes_2_2

begin
  Result:=FFieldList.Count;
end;


<<<<<<< HEAD
<<<<<<< HEAD
function TFields.GetField(Index: Integer): TField;
=======
Function TFields.GetField (Index : longint) : TField;
>>>>>>> graemeg/fixes_2_2
=======
Function TFields.GetField (Index : longint) : TField;
>>>>>>> origin/fixes_2_2

begin
  Result:=Tfield(FFieldList[Index]);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TFields.SetField(Index: Integer; Value: TField);
=======
procedure Tfields.SetField(Index: Integer; Value: TField);
>>>>>>> graemeg/fixes_2_2
=======
procedure Tfields.SetField(Index: Integer; Value: TField);
>>>>>>> origin/fixes_2_2
begin
  Fields[Index].Assign(Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TFields.SetFieldIndex(Field: TField; Value: Integer);
var Old : Longint;
=======
=======
>>>>>>> origin/fixes_2_2
Procedure TFields.SetFieldIndex (Field : TField;Value : Integer);

Var Old : Longint;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
procedure TFields.Add(Field: TField);
=======
Procedure TFields.Add(Field : TField);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TFields.Add(Field : TField);
>>>>>>> origin/fixes_2_2

begin
  CheckFieldName(Field.FieldName);
  FFieldList.Add(Field);
  Field.FFields:=Self;
  Changed;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TFields.CheckFieldName(const Value: String);
=======
Procedure TFields.CheckFieldName (Const Value : String);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TFields.CheckFieldName (Const Value : String);
>>>>>>> origin/fixes_2_2

begin
  If FindField(Value)<>Nil then
    DataBaseErrorFmt(SDuplicateFieldName,[Value],FDataset);
end;

<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
Procedure TFields.CheckFieldNames (Const Value : String);


Var I : longint;
    S,T : String;
begin
  T:=Value;
  Repeat
    I:=Pos(';',T);
    If I=0 Then I:=Length(T)+1;
    S:=Copy(T,1,I-1);
    Delete(T,1,I);
    // Will raise an error if no such field...
    FieldByName(S);
  Until (T='');
end;

Procedure TFields.Clear;

begin
  with FFieldList do
    while Count > 0 do begin
      TField(Last).FDataSet := Nil;
      TField(Last).Free;
      FFieldList.Delete(Count - 1);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  Changed;
end;

<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
Function TFields.FindField (Const Value : String) : TField;

Var S : String;
    I : longint;

begin
  Result:=Nil;
  S:=UpperCase(Value);
  For I:=0 To FFieldList.Count-1 do
    If S=UpperCase(TField(FFieldList[i]).FieldName) Then
      Begin
      {$ifdef dsdebug}
      Writeln ('Found field ',Value);
      {$endif}
      Result:=TField(FFieldList[I]);
      Exit;
      end;
end;

Function TFields.FieldByName (Const Value : String) : TField;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  Result:=FindField(Value);
  If result=Nil then
    DatabaseErrorFmt(SFieldNotFound,[Value],FDataset);
end;

<<<<<<< HEAD
<<<<<<< HEAD
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

<<<<<<< HEAD
function TFields.GetEnumerator: TFieldsEnumerator;
=======
Function TFields.GetEnumerator: TFieldsEnumerator;

begin
  Result:=TFieldsEnumerator.Create(Self);
end;

Function TFields.GetEnumerator: TFieldsEnumerator;

begin
  Result:=TFieldsEnumerator.Create(Self);
end;

Function TFields.GetEnumerator: TFieldsEnumerator;

begin
  Result:=TFieldsEnumerator.Create(Self);
end;

Function TFields.GetEnumerator: TFieldsEnumerator;

begin
  Result:=TFieldsEnumerator.Create(Self);
end;

Procedure TFields.GetFieldNames (Values : TStrings);
>>>>>>> graemeg/cpstrnew

begin
  Result:=TFieldsEnumerator.Create(Self);
end;

procedure TFields.GetFieldNames(Values: TStrings);
var i : longint;
=======
=======
>>>>>>> origin/fixes_2_2
Function TFields.FieldByNumber(FieldNo : Integer) : TField;

Var i : Longint;

begin
  Result:=Nil;
  For I:=0 to FFieldList.Count-1 do
    If FieldNo=TField(FFieldList[I]).FieldNo then
      begin
      Result:=TField(FFieldList[i]);
      Exit;
      end;
end;

Procedure TFields.GetFieldNames (Values : TStrings);

Var i : longint;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  Values.Clear;
  For I:=0 to FFieldList.Count-1 do
    Values.Add(Tfield(FFieldList[I]).FieldName);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TFields.IndexOf(Field: TField): Longint;
=======
Function TFields.IndexOf(Field : TField) : Longint;
>>>>>>> graemeg/fixes_2_2
=======
Function TFields.IndexOf(Field : TField) : Longint;
>>>>>>> origin/fixes_2_2

begin
  Result:=FFieldList.IndexOf(Field);
end;

procedure TFields.Remove(Value : TField);

begin
  FFieldList.Remove(Value);
  Value.FFields := nil;
  Changed;
end;

