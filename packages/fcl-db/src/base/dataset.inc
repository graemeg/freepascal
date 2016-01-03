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

    Dataset implementation

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{ ---------------------------------------------------------------------
    TDataSet
  ---------------------------------------------------------------------}

Const
  DefaultBufferCount = 10;

constructor TDataSet.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
<<<<<<< HEAD
<<<<<<< HEAD
  FFieldDefs:=FieldDefsClass.Create(Self);
  FFieldList:=FieldsClass.Create(Self);
  FDataSources:=TFPList.Create;
  FConstraints:=TCheckConstraints.Create(Self);
  
// FBuffer must be allocated on create, to make Activebuffer return nil
  ReAllocMem(FBuffers,SizeOf(TRecordBuffer));
//  pointer(FBuffers^) := nil;
  FBuffers[0] := nil;
  FActiveRecord := 0;
  FBufferCount := -1;
  FEOF := True;
  FBOF := True;
  FIsUniDirectional := False;
  FAutoCalcFields := True;
=======
=======
>>>>>>> origin/fixes_2_2
  FFieldDefs:=TFieldDefs.Create(Self);
  FFieldList:=TFields.Create(Self);
  FDataSources:=TList.Create;
  FConstraints:=TCheckConstraints.Create(Self);
  
// FBuffer must be allocated on create, to make Activebuffer return nil
  ReAllocMem(FBuffers,SizeOf(PChar));
//  pointer(FBuffers^) := nil;
  FBuffers[0] := nil;
  FActiveRecord := 0;
  FBufferCount := 0;
  FEOF := True;
  FBOF := True;
  FIsUniDirectional := False;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;



destructor TDataSet.Destroy;

var
  i: Integer;

begin
  Active:=False;
  FFieldDefs.Free;
  FFieldList.Free;
<<<<<<< HEAD
<<<<<<< HEAD
  With FDataSources do
    begin
    While Count>0 do
      TDataSource(Items[Count - 1]).DataSet:=Nil;
=======
=======
>>>>>>> origin/fixes_2_2
  With FDatasources do
    begin
    While Count>0 do
      TDatasource(Items[Count - 1]).DataSet:=Nil;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Free;
    end;
  for i := 0 to FBufferCount do
    FreeRecordBuffer(FBuffers[i]);
  FConstraints.Free;
  FreeMem(FBuffers);
  Inherited Destroy;
end;

// This procedure must be called when the first record is made/read
<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.ActivateBuffers;
=======
Procedure TDataset.ActivateBuffers;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.ActivateBuffers;
>>>>>>> origin/fixes_2_2

begin
  FBOF:=False;
  FEOF:=False;
  FActiveRecord:=0;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.UpdateFieldDefs;
=======
Procedure TDataset.UpdateFieldDefs;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.UpdateFieldDefs;
>>>>>>> origin/fixes_2_2

begin
  //!! To be implemented
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.BindFields(Binding: Boolean);

var i, FieldIndex: Integer;

begin
  { FieldNo is set to -1 for calculated/lookup fields, to 0 for unbound field
    and for bound fields it is set to FieldDef.FieldNo }
  FCalcFieldsSize := 0;
  FBlobFieldCount := 0;
  for i := 0 to Fields.Count - 1 do
    with Fields[i] do
      begin
      FFieldDef:=Nil;
      if not Binding then
        FFieldNo := 0
      else if FieldKind in [fkCalculated, fkLookup] then
        begin
        FFieldNo := -1;
        FOffset := FCalcFieldsSize;
        Inc(FCalcFieldsSize, DataSize + 1);
        end
      else
        begin
        FFieldDef := nil;
        FieldIndex := FieldDefs.IndexOf(Fields[i].FieldName);
        if FieldIndex = -1 then
          DatabaseErrorFmt(SFieldNotFound,[Fields[i].FieldName],Self)
        else
          begin
          FFieldDef := FieldDefs[FieldIndex];
          FFieldNo := FFieldDef.FieldNo;
          if FieldDef.InternalCalcField then
            FInternalCalcFields := True;
          if IsBlob then
            begin
            FSize := FFieldDef.Size;
            FOffset := FBlobFieldCount;
            Inc(FBlobFieldCount);
            end;
          end
        end;
      Bind(Binding);
      end;
end;

function TDataSet.BookmarkAvailable: Boolean;
=======
=======
>>>>>>> origin/fixes_2_2
Procedure TDataset.BindFields(Binding: Boolean);

var i, FieldIndex: Integer;
    FieldDef: TFieldDef;
begin
  {
     Here some magic will be needed later; for now just simply set
     Just set fieldno from listindex...
     Later we should take it from the fielddefs.
     // ATM Set by CreateField ...
  For I:=0 to FFieldList.Count-1 do
    FFieldList[i].FFieldNo:=I;
  }
  FCalcFieldsSize := 0;
  FBlobFieldCount := 0;
  for i := 0 to Fields.Count - 1 do
    with Fields[i] do begin
      if Binding then begin
        if FieldKind in [fkCalculated, fkLookup] then begin
          FFieldNo := -1;
          FOffset := FCalcFieldsSize;
          Inc(FCalcFieldsSize, DataSize + 1);
          if FieldKind in [fkLookup] then begin
            if ((FLookupDataSet = nil) or (FLookupKeyFields = '') or
               (FLookupResultField = '') or (FKeyFields = '')) then
              DatabaseErrorFmt(SLookupInfoError, [DisplayName]);
            FFields.CheckFieldNames(FKeyFields);
            FLookupDataSet.Fields.CheckFieldNames(FLookupKeyFields);
            FLookupDataSet.FieldByName(FLookupResultField);
            if FLookupCache then RefreshLookupList;
          end
        end else begin
          FieldDef := nil;
          FieldIndex := FieldDefs.IndexOf(Fields[i].FieldName);
          if FieldIndex <> -1 then begin
            FieldDef := FieldDefs[FieldIndex];
            FFieldNo := FieldDef.FieldNo;
            if FieldDef.InternalCalcField then FInternalCalcFields := True;
            if IsBlob then begin
              FSize := FieldDef.Size;
              FOffset := FBlobFieldCount;
              Inc(FBlobFieldCount);
            end;
          end else FFieldNo := FieldIndex;
        end;
      end else FFieldNo := 0;;
    end;
end;

Function TDataset.BookmarkAvailable: Boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

Const BookmarkStates = [dsBrowse,dsEdit,dsInsert];

begin
  Result:=(Not IsEmpty) and  not FIsUniDirectional and (State in BookmarkStates)
          and (getBookMarkFlag(ActiveBuffer)=bfCurrent);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.CalculateFields(Buffer: TRecordBuffer);
var
  i: Integer;
  OldState: TDatasetState;
begin
  FCalcBuffer := Buffer; 
  if FState <> dsInternalCalc then
  begin
    OldState := FState;
    FState := dsCalcFields;
    try
      ClearCalcFields(FCalcBuffer);
      if not IsUniDirectional then
        for i := 0 to FFieldList.Count - 1 do
          if FFieldList[i].FieldKind = fkLookup then
            FFieldList[i].CalcLookupValue;
    finally
      DoOnCalcFields;
      FState := OldState;
    end;
  end;
end;

procedure TDataSet.CheckActive;
=======
=======
>>>>>>> origin/fixes_2_2
Procedure TDataset.CalculateFields(Buffer: PChar);
var
  i: Integer;
begin
  FCalcBuffer := Buffer;
  
  if not IsUniDirectional and (FState <> dsInternalCalc) then try
    ClearCalcFields(FCalcBuffer);
    for i := 0 to FFieldList.Count - 1 do
      if FFieldList[i].FieldKind = fkLookup then
        FFieldList[i].CalcLookupValue;
  finally
    DoOnCalcFields;
  end;
end;

Procedure TDataset.CheckActive;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  If Not Active then
    DataBaseError(SInactiveDataset);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.CheckInactive;
=======
Procedure TDataset.CheckInactive;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.CheckInactive;
>>>>>>> origin/fixes_2_2

begin
  If Active then
    DataBaseError(SActiveDataset);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.ClearBuffers;

begin
  FRecordCount:=0;
  FActiveRecord:=0;
=======
=======
>>>>>>> origin/fixes_2_2
Procedure TDataset.ClearBuffers;

begin
  FRecordCount:=0;
  FactiveRecord:=0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FCurrentRecord:=-1;
  FBOF:=True;
  FEOF:=True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.ClearCalcFields(Buffer: TRecordBuffer);
=======
Procedure TDataset.ClearCalcFields(Buffer: PChar);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.ClearCalcFields(Buffer: PChar);
>>>>>>> origin/fixes_2_2

begin
  // Empty
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.CloseBlob(Field: TField);
=======
Procedure TDataset.CloseBlob(Field: TField);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.CloseBlob(Field: TField);
>>>>>>> origin/fixes_2_2

begin
  //!! To be implemented
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.CloseCursor;
=======
Procedure TDataset.CloseCursor;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.CloseCursor;
>>>>>>> origin/fixes_2_2

begin
  FreeFieldBuffers;
  ClearBuffers;
  SetBufListSize(0);
<<<<<<< HEAD
<<<<<<< HEAD
  Fields.ClearFieldDefs;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  InternalClose;
  FInternalOpenComplete := False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.CreateFields;
=======
Procedure TDataset.CreateFields;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.CreateFields;
>>>>>>> origin/fixes_2_2

Var I : longint;

begin
{$ifdef DSDebug}
  Writeln ('Creating fields');
  Writeln ('Count : ',fielddefs.Count);
  For I:=0 to FieldDefs.Count-1 do
    Writeln('Def ',I,' : ',Fielddefs.items[i].Name,'(',Fielddefs.items[i].FieldNo,')');
{$endif}
  For I:=0 to fielddefs.Count-1 do
    With Fielddefs.Items[I] do
      If DataType<>ftUnknown then
        begin
        {$ifdef DSDebug}
<<<<<<< HEAD
<<<<<<< HEAD
        Writeln('About to create field ',FieldDefs.Items[i].Name);
=======
        Writeln('About to create field',FieldDefs.Items[i].Name);
>>>>>>> graemeg/fixes_2_2
=======
        Writeln('About to create field',FieldDefs.Items[i].Name);
>>>>>>> origin/fixes_2_2
        {$endif}
        CreateField(self);
        end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DataEvent(Event: TDataEvent; Info: Ptrint);
=======
Procedure TDataset.DataEvent(Event: TDataEvent; Info: Ptrint);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DataEvent(Event: TDataEvent; Info: Ptrint);
>>>>>>> origin/fixes_2_2

  procedure HandleFieldChange(aField: TField);
  begin
    if aField.FieldKind in [fkData, fkInternalCalc] then
      SetModified(True);
      
    if State <> dsSetKey then begin
      if aField.FieldKind = fkData then begin
        if FInternalCalcFields then
          RefreshInternalCalcFields(ActiveBuffer)
        else if FAutoCalcFields and (FCalcFieldsSize <> 0) then
          CalculateFields(ActiveBuffer);
      end;
      
      aField.Change;
    end;
  end;
  
  procedure HandleScrollOrChange;
  begin
    if State <> dsInsert then
      UpdateCursorPos;
  end;

var
  i: Integer;
begin
  case Event of
    deFieldChange   : HandleFieldChange(TField(Info));
    deDataSetChange,
    deDataSetScroll : HandleScrollOrChange;
<<<<<<< HEAD
<<<<<<< HEAD
    deLayoutChange  : FEnableControlsEvent:=deLayoutChange;    
  end;

  if not ControlsDisabled and (FState <> dsBlockRead) then begin
=======
  end;

  if not ControlsDisabled then begin
>>>>>>> graemeg/fixes_2_2
=======
  end;

  if not ControlsDisabled then begin
>>>>>>> origin/fixes_2_2
    for i := 0 to FDataSources.Count - 1 do
      TDataSource(FDataSources[i]).ProcessEvent(Event, Info);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DestroyFields;
=======
Procedure TDataset.DestroyFields;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DestroyFields;
>>>>>>> origin/fixes_2_2

begin
  FFieldList.Clear;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoAfterCancel;
=======
Procedure TDataset.DoAfterCancel;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoAfterCancel;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FAfterCancel) then
   FAfterCancel(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoAfterClose;
=======
Procedure TDataset.DoAfterClose;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoAfterClose;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FAfterClose) and not (csDestroying in ComponentState) then
   FAfterClose(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoAfterDelete;
=======
Procedure TDataset.DoAfterDelete;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoAfterDelete;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FAfterDelete) then
   FAfterDelete(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoAfterEdit;
=======
Procedure TDataset.DoAfterEdit;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoAfterEdit;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FAfterEdit) then
   FAfterEdit(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoAfterInsert;
=======
Procedure TDataset.DoAfterInsert;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoAfterInsert;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FAfterInsert) then
   FAfterInsert(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoAfterOpen;
=======
Procedure TDataset.DoAfterOpen;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoAfterOpen;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FAfterOpen) then
   FAfterOpen(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoAfterPost;
=======
Procedure TDataset.DoAfterPost;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoAfterPost;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FAfterPost) then
   FAfterPost(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoAfterScroll;
=======
Procedure TDataset.DoAfterScroll;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoAfterScroll;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FAfterScroll) then
   FAfterScroll(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoAfterRefresh;
=======
Procedure TDataset.DoAfterRefresh;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoAfterRefresh;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FAfterRefresh) then
   FAfterRefresh(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoBeforeCancel;
=======
Procedure TDataset.DoBeforeCancel;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoBeforeCancel;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FBeforeCancel) then
   FBeforeCancel(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoBeforeClose;
=======
Procedure TDataset.DoBeforeClose;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoBeforeClose;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FBeforeClose) and not (csDestroying in ComponentState) then
   FBeforeClose(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoBeforeDelete;
=======
Procedure TDataset.DoBeforeDelete;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoBeforeDelete;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FBeforeDelete) then
   FBeforeDelete(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoBeforeEdit;
=======
Procedure TDataset.DoBeforeEdit;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoBeforeEdit;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FBeforeEdit) then
   FBeforeEdit(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoBeforeInsert;
=======
Procedure TDataset.DoBeforeInsert;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoBeforeInsert;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FBeforeInsert) then
   FBeforeInsert(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoBeforeOpen;
=======
Procedure TDataset.DoBeforeOpen;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoBeforeOpen;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FBeforeOpen) then
   FBeforeOpen(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoBeforePost;
=======
Procedure TDataset.DoBeforePost;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoBeforePost;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FBeforePost) then
   FBeforePost(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoBeforeScroll;
=======
Procedure TDataset.DoBeforeScroll;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoBeforeScroll;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FBeforeScroll) then
   FBeforeScroll(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoBeforeRefresh;
=======
Procedure TDataset.DoBeforeRefresh;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoBeforeRefresh;
>>>>>>> origin/fixes_2_2

begin
 If assigned(FBeforeRefresh) then
   FBeforeRefresh(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoInternalOpen;
=======
Procedure TDataset.DoInternalOpen;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoInternalOpen;
>>>>>>> origin/fixes_2_2

begin
  InternalOpen;
  FInternalOpenComplete := True;
{$ifdef dsdebug}
  Writeln ('Calling internal open');
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef dsdebug}
  Writeln ('Calling RecalcBufListSize');
{$endif}
  FRecordCount := 0;
  RecalcBufListSize;
  FBOF := True;
  FEOF := (FRecordCount = 0);
end;

procedure TDataSet.DoOnCalcFields;

begin
 If Assigned(FOnCalcfields) then
   FOnCalcFields(Self);
end;

procedure TDataSet.DoOnNewRecord;
=======
=======
>>>>>>> origin/fixes_2_2
  FBOF:=True;
{$ifdef dsdebug}
  Writeln ('Calling RecalcBufListSize');
{$endif}
  FRecordcount := 0;
  RecalcBufListSize;
  FEOF := (FRecordcount = 0);
end;

Procedure TDataset.DoOnCalcFields;

begin
 If assigned(FOnCalcfields) then
   FOnCalcFields(Self);
end;

Procedure TDataset.DoOnNewRecord;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
 If assigned(FOnNewRecord) then
   FOnNewRecord(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.FieldByNumber(FieldNo: Longint): TField;
=======
Function TDataset.FieldByNumber(FieldNo: Longint): TField;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.FieldByNumber(FieldNo: Longint): TField;
>>>>>>> origin/fixes_2_2

begin
  Result:=FFieldList.FieldByNumber(FieldNo);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.FindRecord(Restart, GoForward: Boolean): Boolean;
=======
Function TDataset.FindRecord(Restart, GoForward: Boolean): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.FindRecord(Restart, GoForward: Boolean): Boolean;
>>>>>>> origin/fixes_2_2

begin
  //!! To be implemented
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.FreeFieldBuffers;
=======
Procedure TDataset.FreeFieldBuffers;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.FreeFieldBuffers;
>>>>>>> origin/fixes_2_2

Var I : longint;

begin
  For I:=0 to FFieldList.Count-1 do
    FFieldList[i].FreeBuffers;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.GetBookmarkStr: TBookmarkStr;
=======
Function TDataset.GetBookmarkStr: TBookmarkStr;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.GetBookmarkStr: TBookmarkStr;
>>>>>>> origin/fixes_2_2

begin
  Result:='';
  If BookMarkAvailable then
    begin
    SetLength(Result,FBookMarkSize);
    GetBookMarkData(ActiveBuffer,Pointer(Result));
    end
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.GetBuffer(Index: longint): TRecordBuffer;
=======
Function TDataset.GetBuffer (Index : longint) : Pchar;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.GetBuffer (Index : longint) : Pchar;
>>>>>>> origin/fixes_2_2

begin
  Result:=FBuffers[Index];
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.GetCalcFields(Buffer: TRecordBuffer);

begin
  if (FCalcFieldsSize > 0) or FInternalCalcFields then
    CalculateFields(Buffer);
end;

function TDataSet.GetCanModify: Boolean;
=======
=======
>>>>>>> origin/fixes_2_2
Procedure TDataset.GetCalcFields(Buffer: PChar);

var
  dss: TDataSetState;
begin
  if (FCalcFieldsSize > 0) or FInternalCalcFields then
  begin
    dss := FState;
    FState := dsCalcFields;
    try
      CalculateFields(Buffer);
    finally
      FState := dss;
    end;
  end;
end;

Function TDataset.GetCanModify: Boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  Result:= not FIsUnidirectional;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.GetChildren(Proc: TGetChildProc; Root: TComponent);
=======
Procedure TDataset.GetChildren(Proc: TGetChildProc; Root: TComponent);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.GetChildren(Proc: TGetChildProc; Root: TComponent);
>>>>>>> origin/fixes_2_2

var
 I: Integer;
 Field: TField;

begin
 for I := 0 to Fields.Count - 1 do begin
   Field := Fields[I];
   if (Field.Owner = Root) then
     Proc(Field);
 end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.GetDataSource: TDataSource;
=======
Function TDataset.GetDataSource: TDataSource;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.GetDataSource: TDataSource;
>>>>>>> origin/fixes_2_2
begin
  Result:=nil;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.GetRecordSize: Word;
begin
  Result := 0;
end;

procedure TDataSet.InternalAddRecord(Buffer: Pointer; AAppend: Boolean);
begin
  // empty stub
end;

procedure TDataSet.InternalDelete;
begin
  // empty stub
end;

procedure TDataSet.InternalFirst;
begin
  // empty stub
end;

procedure TDataSet.InternalGotoBookmark(ABookmark: Pointer);
begin
  // empty stub
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TDataSet.GetFieldData(Field: TField; Buffer: Pointer): Boolean;

begin
  Result := False;
end;

procedure TDataSet.DataConvert(aField: TField; aSource, aDest: Pointer;
  aToNative: Boolean);

 // There seems to be no WStrCopy defined, this is a copy of
 // the generic StrCopy function, adapted for WideChar.
 Function WStrCopy(Dest, Source:PWideChar): PWideChar;
 var
   counter : SizeInt;
 Begin
   counter := 0;
   while Source[counter] <> #0 do
   begin
     Dest[counter] := char(Source[counter]);
     Inc(counter);
   end;
   { terminate the string }
   Dest[counter] := #0;
   WStrCopy := Dest;
 end;

var
  DT : TFieldType;

begin
  DT := aField.DataType;
  if aToNative then
    begin
    case DT of
      ftDate, ftTime, ftDateTime: TDateTimeRec(aDest^) := DateTimeToDateTimeRec(DT, TDateTime(aSource^));
      ftTimeStamp               : TTimeStamp(aDest^) := TTimeStamp(aSource^);
      ftBCD                     : TBCD(aDest^) := CurrToBCD(Currency(aSource^));
      ftFMTBCD                  : TBcd(aDest^) := TBcd(aSource^);
  // See notes from mantis bug-report 8204 for more information
  //    ftBytes                   : ;
  //    ftVarBytes                : ;
      ftWideString              : WStrCopy(PWideChar(aDest), PWideChar(aSource));
      end
    end
  else
    begin
    case DT of
      ftDate, ftTime, ftDateTime: TDateTime(aDest^) := DateTimeRecToDateTime(DT, TDateTimeRec(aSource^));
      ftTimeStamp               : TTimeStamp(aDest^) := TTimeStamp(aSource^);
      ftBCD                     : BCDToCurr(TBCD(aSource^),Currency(aDest^));
      ftFMTBCD                  : TBcd(aDest^) := TBcd(aSource^);
  //    ftBytes                   : ;
  //    ftVarBytes                : ;
      ftWideString              : WStrCopy(PWideChar(aDest), PWideChar(aSource));
      end
    end
end;

function TDataSet.GetFieldData(Field: TField; Buffer: Pointer;
  NativeFormat: Boolean): Boolean;

Var
  AStatBuffer : Array[0..dsMaxStringSize] of Char;
  ADynBuffer : pchar;

begin
  If NativeFormat then
    Result:=GetFieldData(Field, Buffer)
  else
    begin
    if Field.DataSize <= dsMaxStringSize then
      begin
      Result := GetfieldData(Field, @AStatBuffer);
      if Result then DataConvert(Field,@AStatBuffer,Buffer,False);
      end
    else
      begin
      GetMem(ADynBuffer,Field.DataSize);
      try
        Result := GetfieldData(Field, ADynBuffer);
        if Result then DataConvert(Field,ADynBuffer,Buffer,False);
      finally
        FreeMem(ADynBuffer);
        end;
      end;
    end;
end;

Function DateTimeRecToDateTime(DT: TFieldType; Data: TDateTimeRec): TDateTime;

var
  TS: TTimeStamp;

begin
  TS.Date:=0;
  TS.Time:=0;
  case DT of
    ftDate: TS.Date := Data.Date;
    ftTime: With TS do
              begin
              Time := Data.Time;
              Date := DateDelta;
              end;
  else
    try
      TS:=MSecsToTimeStamp(trunc(Data.DateTime));
    except
    end;
  end;
  Result:=TimeStampToDateTime(TS);
end;

Function DateTimeToDateTimeRec(DT: TFieldType; Data: TDateTime): TDateTimeRec;

var
  TS : TTimeStamp;

begin
  TS:=DateTimeToTimeStamp(Data);
  With Result do
    case DT of
      ftDate:
        Date:=TS.Date;
      ftTime:
        Time:=TS.Time;
    else
      DateTime:=TimeStampToMSecs(TS);
    end;
end;

procedure TDataSet.SetFieldData(Field: TField; Buffer: Pointer);

begin
// empty procedure
end;

procedure TDataSet.SetFieldData(Field: TField; Buffer: Pointer;
  NativeFormat: Boolean);

Var
  AStatBuffer : Array[0..dsMaxStringSize] of Char;
  ADynBuffer : pchar;

begin
  if NativeFormat then
    SetFieldData(Field, Buffer)
  else
    begin
    if Field.DataSize <= dsMaxStringSize then
      begin
      DataConvert(Field,Buffer,@AStatBuffer,True);
      SetfieldData(Field, @AStatBuffer);
      end
    else
      begin
      GetMem(ADynBuffer,Field.DataSize);
      try
        DataConvert(Field,Buffer,@AStatBuffer,True);
        SetfieldData(Field, @AStatBuffer);
      finally
        FreeMem(ADynBuffer);
        end;
      end;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.GetField(Index: Longint): TField;
=======
Function TDataset.GetField (Index : Longint) : TField;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.GetField (Index : Longint) : TField;
>>>>>>> origin/fixes_2_2

begin
  Result:=FFIeldList[index];
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.GetFieldClass(FieldType: TFieldType): TFieldClass;
=======
Function TDataset.GetFieldClass(FieldType: TFieldType): TFieldClass;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.GetFieldClass(FieldType: TFieldType): TFieldClass;
>>>>>>> origin/fixes_2_2

begin
  Result := DefaultFieldClasses[FieldType];
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.GetIsIndexField(Field: TField): Boolean;
=======
Function TDataset.GetIsIndexField(Field: TField): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.GetIsIndexField(Field: TField): Boolean;
>>>>>>> origin/fixes_2_2

begin
  Result:=False;
end;

function TDataSet.GetIndexDefs(IndexDefs: TIndexDefs; IndexTypes: TIndexOptions
  ): TIndexDefs;
  
var i,f : integer;
    IndexFields : TStrings;
    
begin
  IndexDefs.Update;
  Result := TIndexDefs.Create(Self);
  Result.Assign(IndexDefs);
  i := 0;
  IndexFields := TStringList.Create;
  while i < result.Count do
    begin
    if (not ((IndexTypes = []) and (result[i].Options = []))) and
       ((IndexTypes * result[i].Options) = []) then
      begin
      result.Delete(i);
      dec(i);
      end
    else
      begin
      ExtractStrings([';'],[' '],pchar(result[i].Fields),Indexfields);
      for f := 0 to IndexFields.Count-1 do if FindField(Indexfields[f]) = nil then
        begin
        result.Delete(i);
        dec(i);
        break;
        end;
      end;
    inc(i);
    end;
  IndexFields.Free;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.GetNextRecord: Boolean;
=======
Function TDataset.GetNextRecord: Boolean;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.GetNextRecord: Boolean;
>>>>>>> origin/fixes_2_2

  procedure ExchangeBuffers(var buf1,buf2 : pointer);

  var tempbuf : pointer;

  begin
    tempbuf := buf1;
    buf1 := buf2;
    buf2 := tempbuf;
  end;

begin
{$ifdef dsdebug}
  Writeln ('Getting next record. Internal RecordCount : ',FRecordCount);
{$endif}
  If FRecordCount>0 Then SetCurrentRecord(FRecordCount-1);
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=GetRecord(FBuffers[FBufferCount],gmNext,True)=grOK;
=======
  Result:=GetRecord(FBuffers[FBuffercount],gmNext,True)=grOK;
>>>>>>> graemeg/fixes_2_2
=======
  Result:=GetRecord(FBuffers[FBuffercount],gmNext,True)=grOK;
>>>>>>> origin/fixes_2_2

  if result then
    begin
      If FRecordCount=0 then ActivateBuffers;
<<<<<<< HEAD
<<<<<<< HEAD
      if FRecordCount=FBufferCount then
        ShiftBuffersBackward
=======
      if FRecordcount=FBuffercount then
        shiftbuffersbackward
>>>>>>> graemeg/fixes_2_2
=======
      if FRecordcount=FBuffercount then
        shiftbuffersbackward
>>>>>>> origin/fixes_2_2
      else
        begin
          inc(FRecordCount);
          FCurrentRecord:=FRecordCount - 1;
<<<<<<< HEAD
<<<<<<< HEAD
          ExchangeBuffers(FBuffers[FCurrentRecord],FBuffers[FBufferCount]);
=======
          ExchangeBuffers(Fbuffers[FCurrentRecord],FBuffers[FBuffercount]);
>>>>>>> graemeg/fixes_2_2
=======
          ExchangeBuffers(Fbuffers[FCurrentRecord],FBuffers[FBuffercount]);
>>>>>>> origin/fixes_2_2
        end;
    end
  else
    cursorposchanged;
{$ifdef dsdebug}
  Writeln ('Result getting next record : ',Result);
{$endif}
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.GetNextRecords: Longint;
=======
Function TDataset.GetNextRecords: Longint;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.GetNextRecords: Longint;
>>>>>>> origin/fixes_2_2

begin
  Result:=0;
{$ifdef dsdebug}
  Writeln ('Getting next record(s), need :',FBufferCount);
{$endif}
  While (FRecordCount<FBufferCount) and GetNextRecord do
    Inc(Result);
{$ifdef dsdebug}
  Writeln ('Result Getting next record(S), GOT :',RESULT);
{$endif}
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.GetPriorRecord: Boolean;
=======
Function TDataset.GetPriorRecord: Boolean;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.GetPriorRecord: Boolean;
>>>>>>> origin/fixes_2_2

begin
{$ifdef dsdebug}
  Writeln ('GetPriorRecord: Getting previous record');
{$endif}
  CheckBiDirectional;
  If FRecordCount>0 Then SetCurrentRecord(0);
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=GetRecord(FBuffers[FBufferCount],gmPrior,True)=grOK;
  if result then
    begin
      If FRecordCount=0 then ActivateBuffers;
      ShiftBuffersForward;

      if FRecordCount<FBufferCount then
=======
=======
>>>>>>> origin/fixes_2_2
  Result:=GetRecord(FBuffers[FBuffercount],gmPrior,True)=grOK;
  if result then
    begin
      If FRecordCount=0 then ActivateBuffers;
      shiftbuffersforward;

      if FRecordcount<FBuffercount then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        inc(FRecordCount);
    end
  else
    cursorposchanged;
{$ifdef dsdebug}
  Writeln ('Result getting prior record : ',Result);
{$endif}
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.GetPriorRecords: Longint;
=======
Function TDataset.GetPriorRecords: Longint;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.GetPriorRecords: Longint;
>>>>>>> origin/fixes_2_2

begin
  Result:=0;
{$ifdef dsdebug}
  Writeln ('Getting previous record(s), need :',FBufferCount);
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
  While (FRecordCount<FBufferCount) and GetPriorRecord do
    Inc(Result);
end;

function TDataSet.GetRecNo: Longint;
=======
=======
>>>>>>> origin/fixes_2_2
  While (FRecordCount<FbufferCount) and GetPriorRecord do
    Inc(Result);
end;

Function TDataset.GetRecNo: Longint;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  Result := -1;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.GetRecordCount: Longint;
=======
Function TDataset.GetRecordCount: Longint;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.GetRecordCount: Longint;
>>>>>>> origin/fixes_2_2

begin
  Result := -1;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.InitFieldDefs;
=======
Procedure TDataset.InitFieldDefs;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.InitFieldDefs;
>>>>>>> origin/fixes_2_2

begin
  if IsCursorOpen then
    InternalInitFieldDefs
  else
    begin
    try
      OpenCursor(True);
    finally
      CloseCursor;
      end;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetBlockReadSize(AValue: Integer);
begin
  // the state is changed even when setting the same BlockReadSize (follows Delphi behavior)
  // e.g., state is dsBrowse and BlockReadSize is 1. Setting BlockReadSize to 1 will change state to dsBlockRead
  FBlockReadSize := AValue;
  if AValue > 0 then
  begin
    CheckActive; 
    SetState(dsBlockRead);
  end	
  else
  begin
    //update state only when in dsBlockRead 
    if FState = dsBlockRead then
      SetState(dsBrowse);
  end;	
end;

procedure TDataSet.SetFieldDefs(AFieldDefs: TFieldDefs);

begin
  Fields.ClearFieldDefs;
=======
Procedure TDataSet.SetFieldDefs(AFieldDefs: TFieldDefs);

begin
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataSet.SetFieldDefs(AFieldDefs: TFieldDefs);

begin
>>>>>>> origin/fixes_2_2
  FFieldDefs.Assign(AFieldDefs);
end;

procedure TDataSet.DoInsertAppendRecord(const Values: array of const; DoAppend : boolean);
var i : integer;
    ValuesSize : integer;
begin
  ValuesSize:=Length(Values);
  if ValuesSize>FieldCount then DatabaseError(STooManyFields,self);
  if DoAppend then
    Append
  else
    Insert;

  for i := 0 to ValuesSize-1 do with values[i] do
    fields[i].AssignValue(values[i]);
  Post;

end;

procedure TDataSet.InitFieldDefsFromfields;
var i : integer;
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  if FieldDefs.count = 0 then
    begin
    FieldDefs.BeginUpdate;
    try
      for i := 0 to Fields.Count-1 do with fields[i] do
<<<<<<< HEAD
<<<<<<< HEAD
        if not (FieldKind in [fkCalculated,fkLookup]) then // Do not add fielddefs for calculated/lookup fields.
          begin
<<<<<<< HEAD
          FFieldDef:=FieldDefs.FieldDefClass.Create(FieldDefs,FieldName,DataType,Size,Required,FieldDefs.Count+1);
          with FFieldDef do
            begin
            if Required then Attributes := attributes + [faRequired];
            if ReadOnly then Attributes := attributes + [faReadOnly];
            if DataType = ftBCD then precision := (fields[i] as TBCDField).Precision
            else if DataType = ftFMTBcd then precision := (fields[i] as TFMTBCDField).Precision;
            end;
=======
          if Required then Attributes := attributes + [faRequired];
          if ReadOnly then Attributes := attributes + [faReadOnly];
          if DataType = ftBCD then precision := (fields[i] as TBCDField).Precision
          else if DataType = ftFMTBcd then precision := (fields[i] as TFMTBCDField).Precision;
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
=======
>>>>>>> origin/fixes_2_2
        begin
        with TFieldDef.Create(FieldDefs,FieldName,DataType,Size,Required,i+1) do
          begin
          if Required then Attributes := attributes + [faRequired];
          if ReadOnly then Attributes := attributes + [faReadOnly];
          if DataType = ftBCD then precision := (fields[i] as TBCDField).Precision
          // this must change if TFMTBcdfield is implemented
          else if DataType = ftFMTBcd then precision := (fields[i] as TBCDField).Precision;
          end;
        end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    finally
      FieldDefs.EndUpdate;
      end;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.InitRecord(Buffer: TRecordBuffer);
=======
Procedure TDataset.InitRecord(Buffer: PChar);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.InitRecord(Buffer: PChar);
>>>>>>> origin/fixes_2_2

begin
  InternalInitRecord(Buffer);
  ClearCalcFields(Buffer);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.InternalCancel;
=======
Procedure TDataset.InternalCancel;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.InternalCancel;
>>>>>>> origin/fixes_2_2

begin
  //!! To be implemented
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.InternalEdit;
=======
Procedure TDataset.InternalEdit;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.InternalEdit;
>>>>>>> origin/fixes_2_2

begin
  //!! To be implemented
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.InternalRefresh;
=======
Procedure TDataset.InternalRefresh;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.InternalRefresh;
>>>>>>> origin/fixes_2_2

begin
  //!! To be implemented
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.OpenCursor(InfoQuery: Boolean);

begin
  if InfoQuery then
    InternalInitFieldDefs
  else if State <> dsOpening then
=======
=======
>>>>>>> origin/fixes_2_2
Procedure TDataset.OpenCursor(InfoQuery: Boolean);

begin
  if InfoQuery then
    InternalInitfieldDefs
  else if state <> dsOpening then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    DoInternalOpen;
end;

procedure TDataSet.OpenCursorcomplete;
begin
  try
    if FState = dsOpening then DoInternalOpen
  finally
    if FInternalOpenComplete then
      begin
      SetState(dsBrowse);
      DoAfterOpen;
      if not IsEmpty then
        DoAfterScroll;
      end
    else
      begin
      SetState(dsInactive);
      CloseCursor;
      end;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.RefreshInternalCalcFields(Buffer: TRecordBuffer);
=======
Procedure TDataset.RefreshInternalCalcFields(Buffer: PChar);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.RefreshInternalCalcFields(Buffer: PChar);
>>>>>>> origin/fixes_2_2

begin
  //!! To be implemented
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.SetTempState(const Value: TDataSetState): TDataSetState;
=======
Function TDataset.SetTempState(const Value: TDataSetState): TDataSetState;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.SetTempState(const Value: TDataSetState): TDataSetState;
>>>>>>> origin/fixes_2_2

begin
  result := FState;
  FState := value;
  inc(FDisableControlsCount);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.RestoreState(const Value: TDataSetState);
=======
Procedure TDataset.RestoreState(const Value: TDataSetState);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.RestoreState(const Value: TDataSetState);
>>>>>>> origin/fixes_2_2

begin
  FState := value;
  dec(FDisableControlsCount);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.GetActive: boolean;
=======
function TDataset.GetActive : boolean;
>>>>>>> graemeg/fixes_2_2
=======
function TDataset.GetActive : boolean;
>>>>>>> origin/fixes_2_2

begin
  result := (FState <> dsInactive) and (FState <> dsOpening);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.InternalHandleException;
=======
Procedure TDataset.InternalHandleException;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.InternalHandleException;
>>>>>>> origin/fixes_2_2

begin
  if assigned(classes.ApplicationHandleException) then
    classes.ApplicationHandleException(self)
  else
    ShowException(ExceptObject,ExceptAddr);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.InternalInitRecord(Buffer: TRecordBuffer);
begin
  // empty stub
end;

procedure TDataSet.InternalLast;
begin
  // empty stub
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TDataSet.InternalPost;

  Procedure Checkrequired;

  Var I : longint;

  begin
    For I:=0 to FFieldList.Count-1 do
      With FFieldList[i] do
        // Required fields that are NOT autoinc !! Autoinc cannot be set !!
        if Required and not ReadOnly and
           (FieldKind=fkData) and Not (DataType=ftAutoInc) and IsNull then
          DatabaseErrorFmt(SNeedField,[DisplayName],Self);
  end;

begin
  Checkrequired;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.InternalSetToRecord(Buffer: TRecordBuffer);
begin
  // empty stub
end;

procedure TDataSet.SetBookmarkFlag(Buffer: TRecordBuffer; Value: TBookmarkFlag);
begin
  // empty stub
end;

procedure TDataSet.SetBookmarkData(Buffer: TRecordBuffer; Data: Pointer);
begin
  // empty stub
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TDataSet.SetUniDirectional(const Value: Boolean);
begin
  FIsUniDirectional := Value;
end;

<<<<<<< HEAD
<<<<<<< HEAD
class function TDataSet.FieldDefsClass: TFieldDefsClass;
begin
  Result:=TFieldDefs;
end;

class function TDataSet.FieldsClass: TFieldsClass;
begin
  Result:=TFields;
end;

procedure TDataSet.SetActive(Value: Boolean);
=======
Procedure TDataset.SetActive (Value : Boolean);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.SetActive (Value : Boolean);
>>>>>>> origin/fixes_2_2

begin
  if value and (Fstate = dsInactive) then
    begin
    if csLoading in ComponentState then
      begin
      FOpenAfterRead := true;
      exit;
      end
    else
      begin
      DoBeforeOpen;
<<<<<<< HEAD
<<<<<<< HEAD
      FEnableControlsEvent:=deLayoutChange;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      FInternalCalcFields:=False;
      try
        FDefaultFields:=FieldCount=0;
        OpenCursor(False);
      finally
        if FState <> dsOpening then OpenCursorComplete;
        end;
      end;
    FModified:=False;
    end
  else if not value and (Fstate <> dsinactive) then
    begin
    DoBeforeClose;
    SetState(dsInactive);
    CloseCursor;
    DoAfterClose;
    FModified:=False;
    end
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.Loaded;
=======
procedure TDataset.Loaded;
>>>>>>> graemeg/fixes_2_2
=======
procedure TDataset.Loaded;
>>>>>>> origin/fixes_2_2

begin
  inherited;
  try
    if FOpenAfterRead then SetActive(true);
  except
    if csDesigning in Componentstate then
      InternalHandleException
    else
      raise;
  end;
end;


procedure TDataSet.RecalcBufListSize;

var
  i, j, ABufferCount: Integer;
  DataLink: TDataLink;

begin
{$ifdef dsdebug}
  Writeln('Recalculating buffer list size - check cursor');
{$endif}
  If Not IsCursorOpen Then
    Exit;
{$ifdef dsdebug}
  Writeln('Recalculating buffer list size');
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
  if IsUniDirectional then
    ABufferCount := 1
  else
    ABufferCount := DefaultBufferCount;

=======
  ABufferCount := DefaultBufferCount;
>>>>>>> graemeg/fixes_2_2
=======
  ABufferCount := DefaultBufferCount;
>>>>>>> origin/fixes_2_2
  for i := 0 to FDataSources.Count - 1 do
    for j := 0 to TDataSource(FDataSources[i]).DataLinks.Count - 1 do
      begin
      DataLink:=TDataLink(TDataSource(FDataSources[i]).DataLinks[j]);
      if DataLink.BufferCount>ABufferCount then
        ABufferCount:=DataLink.BufferCount;
      end;

  If (FBufferCount=ABufferCount) Then
    exit;

{$ifdef dsdebug}
  Writeln('Setting buffer list size');
{$endif}

  SetBufListSize(ABufferCount);
{$ifdef dsdebug}
  Writeln('Getting next buffers');
{$endif}
  GetNextRecords;
<<<<<<< HEAD
<<<<<<< HEAD
  if (FRecordCount < FBufferCount) and not IsUniDirectional then
=======
  if FRecordCount < FBufferCount then
>>>>>>> graemeg/fixes_2_2
=======
  if FRecordCount < FBufferCount then
>>>>>>> origin/fixes_2_2
    begin
    FActiveRecord := FActiveRecord + GetPriorRecords;
    CursorPosChanged;
    end;
{$Ifdef dsDebug}
  WriteLn(
    'SetBufferCount: FActiveRecord=',FActiveRecord,
    ' FCurrentRecord=',FCurrentRecord,
    ' FBufferCount= ',FBufferCount,
    ' FRecordCount=',FRecordCount);
{$Endif}
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetBookmarkStr(const Value: TBookmarkStr);
=======
Procedure TDataset.SetBookmarkStr(const Value: TBookmarkStr);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.SetBookmarkStr(const Value: TBookmarkStr);
>>>>>>> origin/fixes_2_2

begin
  GotoBookMark(Pointer(Value))
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetBufListSize(Value: Longint);
=======
Procedure TDataset.SetBufListSize(Value: Longint);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.SetBufListSize(Value: Longint);
>>>>>>> origin/fixes_2_2

Var I : longint;

begin
  if Value = 0 then Value := -1;
{$ifdef dsdebug}
  Writeln ('SetBufListSize: ',Value);
{$endif}
  If Value=FBufferCount Then
    exit;
  If Value>FBufferCount then
    begin
{$ifdef dsdebug}
<<<<<<< HEAD
<<<<<<< HEAD
    Writeln ('   Reallocating memory :',(Value+1)*SizeOf(TRecordBuffer));
{$endif}
    ReAllocMem(FBuffers,(Value+1)*SizeOf(PChar));
{$ifdef dsdebug}
    Writeln ('   Filling memory :',(Value+1-FBufferCount)*SizeOf(TRecordBuffer));
{$endif}
    inc(FBufferCount); // Cause FBuffers[FBufferCount] is already allocated
    FillChar(FBuffers[FBufferCount],(Value+1-FBufferCount)*SizeOf(TRecordBuffer),#0);
{$ifdef dsdebug}
    Writeln ('   Filled memory');
{$endif}
    Try
{$ifdef dsdebug}
      Writeln ('   Assigning buffers : ',(Value)*SizeOf(TRecordBuffer));
=======
=======
>>>>>>> origin/fixes_2_2
    Writeln ('   Reallocating memory :',(Value+1)*SizeOf(PChar));
{$endif}
    ReAllocMem(FBuffers,(Value+1)*SizeOf(PChar));
{$ifdef dsdebug}
    Writeln ('   Filling memory :',(Value+1-FBufferCount)*SizeOf(PChar));
{$endif}
    if FBufferCount > 0 then inc(FBufferCount); // Cause FBuffers[FBufferCount] is already allocated
    FillChar(FBuffers[FBufferCount],(Value+1-FBufferCount)*SizeOF(Pchar),#0);
{$ifdef dsdebug}
    Writeln ('   Filled memory :');
{$endif}
    Try
{$ifdef dsdebug}
      Writeln ('   Assigning buffers :',(Value)*SizeOf(PChar));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif}
      For I:=FBufferCount to Value do
        FBuffers[i]:=AllocRecordBuffer;
{$ifdef dsdebug}
<<<<<<< HEAD
<<<<<<< HEAD
      Writeln ('   Assigned buffers (FBufferCount:',FBufferCount,') : ',(Value)*SizeOf(TRecordBuffer));
=======
      Writeln ('   Assigned buffers ',FBufferCount,' :',(Value)*SizeOf(PChar));
>>>>>>> graemeg/fixes_2_2
=======
      Writeln ('   Assigned buffers ',FBufferCount,' :',(Value)*SizeOf(PChar));
>>>>>>> origin/fixes_2_2
{$endif}
    except
      I:=FBufferCount;
      While (I<(Value+1)) do
        begin
        FreeRecordBuffer(FBuffers[i]);
        Inc(i);
        end;
      raise;
    end;
    end
  else
    begin
{$ifdef dsdebug}
    Writeln ('   Freeing buffers :',FBufferCount-Value);
{$endif}
    if (value > -1) and (FActiveRecord>Value-1) then
      begin
      for i := 0 to (FActiveRecord-Value) do
<<<<<<< HEAD
<<<<<<< HEAD
        ShiftBuffersBackward;
      FActiveRecord := Value -1;
=======
        shiftbuffersbackward;
      FActiverecord := Value -1;
>>>>>>> graemeg/fixes_2_2
=======
        shiftbuffersbackward;
      FActiverecord := Value -1;
>>>>>>> origin/fixes_2_2
      end;

    If Assigned(FBuffers) then
      begin
      For I:=Value+1 to FBufferCount do
        FreeRecordBuffer(FBuffers[i]);
      // FBuffer must stay allocated, to make sure that Activebuffer returns nil
      if Value = -1 then
        begin
<<<<<<< HEAD
<<<<<<< HEAD
        ReAllocMem(FBuffers,SizeOf(TRecordBuffer));
        FBuffers[0] := nil;
        end
      else
        ReAllocMem(FBuffers,(Value+1)*SizeOf(TRecordBuffer));
      end;
    end;
  FBufferCount:=Value;
  If Value=-1 then
    Value:=0;
  if FRecordCount > Value then FRecordCount := Value;
=======
=======
>>>>>>> origin/fixes_2_2
        ReAllocMem(FBuffers,SizeOf(Pchar));
        FBuffers[0] := nil;
        end
      else
        ReAllocMem(FBuffers,(Value+1)*SizeOf(Pchar));
      end;
    end;
  If Value=-1 then
    Value:=0;
  if FRecordcount > Value then FRecordcount := Value;
  FBufferCount:=Value;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef dsdebug}
  Writeln ('   SetBufListSize: Final FBufferCount=',FBufferCount);
{$endif}
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetChildOrder(Component: TComponent; Order: Longint);
=======
Procedure TDataset.SetChildOrder(Component: TComponent; Order: Longint);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.SetChildOrder(Component: TComponent; Order: Longint);
>>>>>>> origin/fixes_2_2

var
  Field: TField;
begin
  Field := Component as TField;
  if Fields.IndexOf(Field) >= 0 then
    Field.Index := Order;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetCurrentRecord(Index: Longint);
=======
Procedure TDataset.SetCurrentRecord(Index: Longint);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.SetCurrentRecord(Index: Longint);
>>>>>>> origin/fixes_2_2

begin
  If FCurrentRecord<>Index then
    begin
{$ifdef DSdebug}
<<<<<<< HEAD
<<<<<<< HEAD
    Writeln ('Setting current record to: ',index);
=======
    Writeln ('Setting current record to',index);
>>>>>>> graemeg/fixes_2_2
=======
    Writeln ('Setting current record to',index);
>>>>>>> origin/fixes_2_2
{$endif}
    if not FIsUniDirectional then Case GetBookMarkFlag(FBuffers[Index]) of
      bfCurrent : InternalSetToRecord(FBuffers[Index]);
      bfBOF : InternalFirst;
      bfEOF : InternalLast;
      end;
    FCurrentRecord:=index;
    end;
end;

procedure TDataSet.SetDefaultFields(const Value: Boolean);
begin
  FDefaultFields := Value;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetField(Index: Longint; Value: TField);
=======
Procedure TDataset.SetField (Index : Longint;Value : TField);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.SetField (Index : Longint;Value : TField);
>>>>>>> origin/fixes_2_2

begin
  //!! To be implemented
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.CheckBiDirectional;
=======
Procedure TDataset.CheckBiDirectional;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.CheckBiDirectional;
>>>>>>> origin/fixes_2_2

begin
  if FIsUniDirectional then DataBaseError(SUniDirectional);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetFilterOptions(Value: TFilterOptions);
=======
Procedure TDataset.SetFilterOptions(Value: TFilterOptions);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.SetFilterOptions(Value: TFilterOptions);
>>>>>>> origin/fixes_2_2

begin
  CheckBiDirectional;
  FFilterOptions := Value;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetFilterText(const Value: string);
=======
Procedure TDataset.SetFilterText(const Value: string);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.SetFilterText(const Value: string);
>>>>>>> origin/fixes_2_2

begin
  FFilterText := value;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetFiltered(Value: Boolean);
=======
Procedure TDataset.SetFiltered(Value: Boolean);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.SetFiltered(Value: Boolean);
>>>>>>> origin/fixes_2_2

begin
  if Value then CheckBiDirectional;
  FFiltered := value;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetFound(const Value: Boolean);
=======
Procedure TDataset.SetFound(const Value: Boolean);

>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.SetFound(const Value: Boolean);

>>>>>>> origin/fixes_2_2
begin
  FFound := Value;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetModified(Value: Boolean);
=======
Procedure TDataset.SetModified(Value: Boolean);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.SetModified(Value: Boolean);
>>>>>>> origin/fixes_2_2

begin
  FModified := value;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetName(const Value: TComponentName);

function CheckName(const FieldName: string): string;
=======
Procedure TDataset.SetName(const Value: TComponentName);

function CheckName(FieldName: string): string;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.SetName(const Value: TComponentName);

function CheckName(FieldName: string): string;
>>>>>>> origin/fixes_2_2
var i,j: integer;
begin
  Result := FieldName;
  i := 0;
  j := 0;
  while (i < Fields.Count) do begin
    if Result = Fields[i].FieldName then begin
      inc(j);
      Result := FieldName + IntToStr(j);
    end else Inc(i);
  end;
end;
var i: integer;
    nm: string;
    old: string;
begin
  if Self.Name = Value then Exit;
  old := Self.Name;
  inherited SetName(Value);
  if (csDesigning in ComponentState) then
    for i := 0 to Fields.Count - 1 do begin
      nm := old + Fields[i].FieldName;
      if Copy(Fields[i].Name, 1, Length(nm)) = nm then
        Fields[i].Name := CheckName(Value + Fields[i].FieldName);
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetOnFilterRecord(const Value: TFilterRecordEvent);
=======
Procedure TDataset.SetOnFilterRecord(const Value: TFilterRecordEvent);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.SetOnFilterRecord(const Value: TFilterRecordEvent);
>>>>>>> origin/fixes_2_2

begin
  CheckBiDirectional;
  FOnFilterRecord := Value;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetRecNo(Value: Longint);
=======
Procedure TDataset.SetRecNo(Value: Longint);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.SetRecNo(Value: Longint);
>>>>>>> origin/fixes_2_2

begin
  //!! To be implemented
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetState(Value: TDataSetState);
=======
Procedure TDataset.SetState(Value: TDataSetState);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.SetState(Value: TDataSetState);
>>>>>>> origin/fixes_2_2

begin
  If Value<>FState then
    begin
    FState:=Value;
<<<<<<< HEAD
<<<<<<< HEAD
    if Value=dsBrowse then
      FModified:=false;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    DataEvent(deUpdateState,0);
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.TempBuffer: TRecordBuffer;
=======
Function TDataset.Tempbuffer: PChar;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.Tempbuffer: PChar;
>>>>>>> origin/fixes_2_2

begin
  Result := FBuffers[FRecordCount];
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.UpdateIndexDefs;
=======
Procedure TDataset.UpdateIndexDefs;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.UpdateIndexDefs;
>>>>>>> origin/fixes_2_2

begin
  // Empty Abstract
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.AllocRecordBuffer: TRecordBuffer;
begin
  Result := nil;
end;

procedure TDataSet.FreeRecordBuffer(var Buffer: TRecordBuffer);
begin
  // empty stub
end;

procedure TDataSet.GetBookmarkData(Buffer: TRecordBuffer; Data: Pointer);
begin
  // empty stub
end;

function TDataSet.GetBookmarkFlag(Buffer: TRecordBuffer): TBookmarkFlag;
begin
  Result := bfCurrent;
end;

function TDataSet.ControlsDisabled: Boolean;
=======
Function TDataset.ControlsDisabled: Boolean;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.ControlsDisabled: Boolean;
>>>>>>> origin/fixes_2_2

begin
  Result := (FDisableControlsCount > 0);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.ActiveBuffer: TRecordBuffer;

begin
{$ifdef dsdebug}
  Writeln ('Active buffer requested. Returning record number:',ActiveRecord);
=======
=======
>>>>>>> origin/fixes_2_2
Function TDataset.ActiveBuffer: PChar;

begin
{$ifdef dsdebug}
  Writeln ('Active buffer requested. Returning:',ActiveRecord);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif}
  Result:=FBuffers[FActiveRecord];
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.Append;
=======
Procedure TDataset.Append;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.Append;
>>>>>>> origin/fixes_2_2

begin
  DoInsertAppend(True);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.InternalInsert;
=======
Procedure TDataset.InternalInsert;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.InternalInsert;
>>>>>>> origin/fixes_2_2

begin
  //!! To be implemented
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.AppendRecord(const Values: array of const);
=======
Procedure TDataset.AppendRecord(const Values: array of const);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.AppendRecord(const Values: array of const);
>>>>>>> origin/fixes_2_2

begin
  DoInsertAppendRecord(Values,True);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.BookmarkValid(ABookmark: TBookmark): Boolean;
=======
Function TDataset.BookmarkValid(ABookmark: TBookmark): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.BookmarkValid(ABookmark: TBookmark): Boolean;
>>>>>>> origin/fixes_2_2
{
  Should be overridden by descendant objects.
}
begin
  Result:=False
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.Cancel;
=======
Procedure TDataset.Cancel;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.Cancel;
>>>>>>> origin/fixes_2_2

begin
  If State in [dsEdit,dsInsert] then
    begin
    DataEvent(deCheckBrowseMode,0);
    DoBeforeCancel;
    UpdateCursorPos;
    InternalCancel;
    FreeFieldBuffers;
<<<<<<< HEAD
<<<<<<< HEAD
    if (State = dsInsert) and (FRecordCount = 1) then
      begin
      FEOF := true;
      FBOF := true;
      FRecordCount := 0;
=======
=======
>>>>>>> origin/fixes_2_2
    if (state = dsInsert) and (FRecordcount = 1) then
      begin
      FEOF := true;
      FBOF := true;
      FRecordcount := 0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      InitRecord(ActiveBuffer);
      SetState(dsBrowse);
      DataEvent(deDatasetChange,0);
      end
    else
      begin
      SetState(dsBrowse);
<<<<<<< HEAD
<<<<<<< HEAD
      SetCurrentRecord(FActiveRecord);
=======
      SetCurrentRecord(FActiverecord);
>>>>>>> graemeg/fixes_2_2
=======
      SetCurrentRecord(FActiverecord);
>>>>>>> origin/fixes_2_2
      resync([]);
      end;
    DoAfterCancel;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.CheckBrowseMode;
=======
Procedure TDataset.CheckBrowseMode;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.CheckBrowseMode;
>>>>>>> origin/fixes_2_2

begin
  CheckActive;
  DataEvent(deCheckBrowseMode,0);
  Case State of
<<<<<<< HEAD
<<<<<<< HEAD
    dsEdit,dsInsert: begin
=======
    dsedit,dsinsert: begin
>>>>>>> graemeg/fixes_2_2
=======
    dsedit,dsinsert: begin
>>>>>>> origin/fixes_2_2
      UpdateRecord;
      If Modified then Post else Cancel;
    end;
    dsSetKey: Post;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.ClearFields;
=======
Procedure TDataset.ClearFields;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.ClearFields;
>>>>>>> origin/fixes_2_2


begin
  DataEvent(deCheckBrowseMode, 0);
  FreeFieldBuffers;
  InternalInitRecord(ActiveBuffer);
  if State <> dsSetKey then GetCalcFields(ActiveBuffer);
  DataEvent(deRecordChange, 0);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.Close;
=======
Procedure TDataset.Close;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.Close;
>>>>>>> origin/fixes_2_2

begin
  Active:=False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.CompareBookmarks(Bookmark1, Bookmark2: TBookmark): Longint;
=======
Function TDataset.CompareBookmarks(Bookmark1, Bookmark2: TBookmark): Longint;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.CompareBookmarks(Bookmark1, Bookmark2: TBookmark): Longint;
>>>>>>> origin/fixes_2_2

begin
  Result:=0;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.CreateBlobStream(Field: TField; Mode: TBlobStreamMode
  ): TStream;
=======
Function TDataset.CreateBlobStream(Field: TField; Mode: TBlobStreamMode): TStream;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.CreateBlobStream(Field: TField; Mode: TBlobStreamMode): TStream;
>>>>>>> origin/fixes_2_2


begin
  Result:=Nil;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.CursorPosChanged;
=======
Procedure TDataset.CursorPosChanged;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.CursorPosChanged;
>>>>>>> origin/fixes_2_2


begin
  FCurrentRecord:=-1;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.Delete;
=======
Procedure TDataset.Delete;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.Delete;
>>>>>>> origin/fixes_2_2

begin
  If Not CanModify then
    DatabaseError(SDatasetReadOnly,Self);
  If IsEmpty then
    DatabaseError(SDatasetEmpty,Self);
  if State in [dsInsert] then
  begin
    Cancel;
  end else begin
    DataEvent(deCheckBrowseMode,0);
{$ifdef dsdebug}
    writeln ('Delete: checking required fields');
{$endif}
    DoBeforeDelete;
    DoBeforeScroll;
<<<<<<< HEAD
<<<<<<< HEAD
    If Not TryDoing(@InternalDelete,OnDeleteError) then exit;
=======
    If Not TryDoing(@InternalDelete,OnPostError) then exit;
>>>>>>> graemeg/fixes_2_2
=======
    If Not TryDoing(@InternalDelete,OnPostError) then exit;
>>>>>>> origin/fixes_2_2
{$ifdef dsdebug}
    writeln ('Delete: Internaldelete succeeded');
{$endif}
    FreeFieldBuffers;
    SetState(dsBrowse);
{$ifdef dsdebug}
    writeln ('Delete: Browse mode set');
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
    SetCurrentRecord(FActiveRecord);
=======
    SetCurrentRecord(FActiverecord);
>>>>>>> graemeg/fixes_2_2
=======
    SetCurrentRecord(FActiverecord);
>>>>>>> origin/fixes_2_2
    Resync([]);
    DoAfterDelete;
    DoAfterScroll;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DisableControls;
=======
Procedure TDataset.DisableControls;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DisableControls;
>>>>>>> origin/fixes_2_2


begin
  If FDisableControlsCount=0 then
    begin
    { Save current state,
      needed to detect change of state when enabling controls.
    }
    FDisableControlsState:=FState;
    FEnableControlsEvent:=deDatasetChange;
    end;
  Inc(FDisableControlsCount);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.DoInsertAppend(DoAppend: Boolean);
=======
Procedure TDataset.DoInsertAppend(DoAppend : Boolean);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.DoInsertAppend(DoAppend : Boolean);
>>>>>>> origin/fixes_2_2


  procedure DoInsert(DoAppend : Boolean);

<<<<<<< HEAD
<<<<<<< HEAD
  Var BookBeforeInsert : TBookmark;
=======
  Var BookBeforeInsert : TBookmarkStr;
>>>>>>> graemeg/fixes_2_2
=======
  Var BookBeforeInsert : TBookmarkStr;
>>>>>>> origin/fixes_2_2
      TempBuf : pointer;

  begin
  // need to scroll up al buffers after current one,
  // but copy current bookmark to insert buffer.
<<<<<<< HEAD
<<<<<<< HEAD
  If FRecordCount > 0 then
=======
  If FRecordcount > 0 then
>>>>>>> graemeg/fixes_2_2
=======
  If FRecordcount > 0 then
>>>>>>> origin/fixes_2_2
    BookBeforeInsert:=Bookmark;

  if not DoAppend then
    begin
    if FRecordCount > 0 then
      begin
<<<<<<< HEAD
<<<<<<< HEAD
      TempBuf := FBuffers[FBufferCount];
      move(FBuffers[FActiveRecord],FBuffers[FActiveRecord+1],(FBufferCount-FActiveRecord)*sizeof(FBuffers[0]));
      FBuffers[FActiveRecord]:=TempBuf;
      end;
    end
  else if FRecordCount=FBufferCount then
    ShiftBuffersBackward
=======
=======
>>>>>>> origin/fixes_2_2
      TempBuf := FBuffers[FBuffercount];
      move(FBuffers[FActiveRecord],FBuffers[FActiveRecord+1],(Fbuffercount-FActiveRecord)*sizeof(FBuffers[0]));
      FBuffers[FActiveRecord]:=TempBuf;
      end;
    end
  else if FRecordcount=FBuffercount then
    shiftbuffersbackward
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  else
    begin
    if FRecordCount>0 then
      inc(FActiveRecord);
    end;

  // Active buffer is now edit buffer. Initialize.
  InitRecord(FBuffers[FActiveRecord]);
  cursorposchanged;

  // Put bookmark in edit buffer.
  if FRecordCount=0 then
    SetBookmarkFlag(ActiveBuffer,bfEOF)
  else
    begin
    fBOF := false;
    // 29:01:05, JvdS: Why is this here?!? It can result in records with the same bookmark-data?
    // I would say that the 'internalinsert' should do this. But I don't know how Tdbf handles it

    // 1-apr-06, JvdS: It just sets the bookmark of the newly inserted record to the place
    // where the record should be inserted. So it is ok.
<<<<<<< HEAD
<<<<<<< HEAD
    if FRecordCount > 0 then
      begin
      SetBookMarkData(ActiveBuffer,pointer(BookBeforeInsert));
      FreeBookmark(BookBeforeInsert);
      end;
=======
    if FRecordcount > 0 then
      SetBookMarkData(ActiveBuffer,pointer(BookBeforeInsert));
>>>>>>> graemeg/fixes_2_2
=======
    if FRecordcount > 0 then
      SetBookMarkData(ActiveBuffer,pointer(BookBeforeInsert));
>>>>>>> origin/fixes_2_2
    end;

  InternalInsert;

  // update buffer count.
  If FRecordCount<FBufferCount then
    Inc(FRecordCount);
  end;

begin
  CheckBrowseMode;
  If Not CanModify then
    DatabaseError(SDatasetReadOnly,Self);
  DoBeforeInsert;
  DoBeforeScroll;
  If Not DoAppend then
    begin
{$ifdef dsdebug}
    Writeln ('going to insert mode');
{$endif}
    DoInsert(false);
    end
  else
    begin
{$ifdef dsdebug}
    Writeln ('going to append mode');
{$endif}
    ClearBuffers;
    InternalLast;
    GetPriorRecords;
    if FRecordCount>0 then
      FActiveRecord:=FRecordCount-1;
    DoInsert(True);
    SetBookmarkFlag(ActiveBuffer,bfEOF);
    FBOF :=False;
    FEOF := true;
    end;
  SetState(dsInsert);
  try
    DoOnNewRecord;
  except
<<<<<<< HEAD
<<<<<<< HEAD
    SetCurrentRecord(FActiveRecord);
=======
    SetCurrentRecord(FActiverecord);
>>>>>>> graemeg/fixes_2_2
=======
    SetCurrentRecord(FActiverecord);
>>>>>>> origin/fixes_2_2
    resync([]);
    raise;
  end;
  // mark as not modified.
  FModified:=False;
  // Final events.
  DataEvent(deDatasetChange,0);
  DoAfterInsert;
  DoAfterScroll;
{$ifdef dsdebug}
  Writeln ('Done with append');
{$endif}
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.Edit;

begin
  If State in [dsEdit,dsInsert] then exit;
=======
=======
>>>>>>> origin/fixes_2_2
Procedure TDataset.Edit;

begin
  If State in [dsedit,dsinsert] then exit;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  CheckBrowseMode;
  If Not CanModify then
    DatabaseError(SDatasetReadOnly,Self);
  If FRecordCount = 0 then
    begin
    Append;
    Exit;
    end;
  DoBeforeEdit;
  If Not TryDoing(@InternalEdit,OnEditError) then exit;
  GetCalcFields(ActiveBuffer);
<<<<<<< HEAD
<<<<<<< HEAD
  SetState(dsEdit);
=======
  SetState(dsedit);
>>>>>>> graemeg/fixes_2_2
=======
  SetState(dsedit);
>>>>>>> origin/fixes_2_2
  DataEvent(deRecordChange,0);
  DoAfterEdit;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.EnableControls;
=======
Procedure TDataset.EnableControls;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.EnableControls;
>>>>>>> origin/fixes_2_2


begin
  if FDisableControlsCount > 0 then
    Dec(FDisableControlsCount);

  if FDisableControlsCount = 0 then begin
    if FState <> FDisableControlsState then
      DataEvent(deUpdateState, 0);

    if (FState <> dsInactive) and (FDisableControlsState <> dsInactive) then
      DataEvent(FEnableControlsEvent, 0);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.FieldByName(const FieldName: string): TField;
=======
Function TDataset.FieldByName(const FieldName: string): TField;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.FieldByName(const FieldName: string): TField;
>>>>>>> origin/fixes_2_2


begin
  Result:=FindField(FieldName);
  If Result=Nil then
    DatabaseErrorFmt(SFieldNotFound,[FieldName],Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.FindField(const FieldName: string): TField;
=======
Function TDataset.FindField(const FieldName: string): TField;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.FindField(const FieldName: string): TField;
>>>>>>> origin/fixes_2_2


begin
  Result:=FFieldList.FindField(FieldName);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.FindFirst: Boolean;
=======
Function TDataset.FindFirst: Boolean;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.FindFirst: Boolean;
>>>>>>> origin/fixes_2_2


begin
  Result:=False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.FindLast: Boolean;
=======
Function TDataset.FindLast: Boolean;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.FindLast: Boolean;
>>>>>>> origin/fixes_2_2


begin
  Result:=False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.FindNext: Boolean;
=======
Function TDataset.FindNext: Boolean;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.FindNext: Boolean;
>>>>>>> origin/fixes_2_2


begin
  Result:=False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.FindPrior: Boolean;
=======
Function TDataset.FindPrior: Boolean;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.FindPrior: Boolean;
>>>>>>> origin/fixes_2_2


begin
  Result:=False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.First;
=======
Procedure TDataset.First;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.First;
>>>>>>> origin/fixes_2_2


begin
  CheckBrowseMode;
  DoBeforeScroll;
  if not FIsUniDirectional then
    ClearBuffers
  else if not FBof then
    begin
    Active := False;
    Active := True;
    end;
  try
    InternalFirst;
    if not FIsUniDirectional then GetNextRecords;
  finally
    FBOF:=True;
    DataEvent(deDatasetChange,0);
    DoAfterScroll;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.FreeBookmark(ABookmark: TBookmark);


begin
  {$ifdef noautomatedbookmark}
   FreeMem(ABookMark,FBookMarkSize);
  {$endif}
end;

function TDataSet.GetBookmark: TBookmark;
=======
=======
>>>>>>> origin/fixes_2_2
Procedure TDataset.FreeBookmark(ABookmark: TBookmark);


begin
  FreeMem(ABookMark,FBookMarkSize);
end;

Function TDataset.GetBookmark: TBookmark;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


begin
  if BookmarkAvailable then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    {$ifdef noautomatedbookmark}
      GetMem (Result,FBookMarkSize);
    {$else}
      setlength(Result,FBookMarkSize);
    {$endif}
    GetBookMarkdata(ActiveBuffer,pointer(Result));
=======
    GetMem (Result,FBookMarkSize);
    GetBookMarkdata(ActiveBuffer,Result);
>>>>>>> graemeg/fixes_2_2
=======
    GetMem (Result,FBookMarkSize);
    GetBookMarkdata(ActiveBuffer,Result);
>>>>>>> origin/fixes_2_2
    end
  else
    Result:=Nil;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.GetCurrentRecord(Buffer: TRecordBuffer): Boolean;
=======
Function TDataset.GetCurrentRecord(Buffer: PChar): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.GetCurrentRecord(Buffer: PChar): Boolean;
>>>>>>> origin/fixes_2_2


begin
  Result:=False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.GetFieldList(List: TList; const FieldNames: string);

var
  F: TField;
  N: String;
  StrPos: Integer;

begin
  if (FieldNames = '') or (List = nil) then
    Exit;
  StrPos := 1;
  repeat
    N := ExtractFieldName(FieldNames, StrPos);
    F := FieldByName(N);
    List.Add(F);
  until StrPos > Length(FieldNames);
end;

procedure TDataSet.GetFieldNames(List: TStrings);
=======
=======
>>>>>>> origin/fixes_2_2
Procedure TDataset.GetFieldList(List: TList; const FieldNames: string);

  Function NextName(Var S : String) : String;

  Var
    P : integer;

  begin
    P:=Pos(';',S);
    If (P=0) then
      P:=Length(S)+1;
    Result:=Copy(S,1,P-1);
    system.Delete(S,1,P);
  end;

var
  F: TField;
  Names,N : String;

begin
  Names:=FieldNames;
  N:=Nextname(Names);
  while (N<>'') do
    begin
    F:=FieldByName(N);
    If Assigned(List) then
      List.Add(F);
    N:=NextName(Names);
    end;
end;

Procedure TDataset.GetFieldNames(List: TStrings);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


begin
  FFieldList.GetFieldNames(List);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.GotoBookmark(const ABookmark: TBookmark);
=======
Procedure TDataset.GotoBookmark(ABookmark: TBookmark);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.GotoBookmark(ABookmark: TBookmark);
>>>>>>> origin/fixes_2_2


begin
  If Assigned(ABookMark) then
    begin
    CheckBrowseMode;
    DoBeforeScroll;
<<<<<<< HEAD
<<<<<<< HEAD
    InternalGotoBookMark(pointer(ABookMark));
=======
    InternalGotoBookMark(ABookMark);
>>>>>>> graemeg/fixes_2_2
=======
    InternalGotoBookMark(ABookMark);
>>>>>>> origin/fixes_2_2
    Resync([rmExact,rmCenter]);
    DoAfterScroll;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.Insert;
=======
Procedure TDataset.Insert;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.Insert;
>>>>>>> origin/fixes_2_2

begin
  DoInsertAppend(False);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.InsertRecord(const Values: array of const);
=======
Procedure TDataset.InsertRecord(const Values: array of const);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.InsertRecord(const Values: array of const);
>>>>>>> origin/fixes_2_2

begin
  DoInsertAppendRecord(Values,False);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.IsEmpty: Boolean;

begin
  Result:=(fBof and fEof) and
          (not (State = dsInsert)); // After an insert on an empty dataset, both fBof and fEof are true
end;

function TDataSet.IsLinkedTo(ADataSource: TDataSource): Boolean;
=======
=======
>>>>>>> origin/fixes_2_2
Function TDataset.IsEmpty: Boolean;

begin
  Result:=(fBof and fEof) and
          (not (state = dsinsert)); // After an insert on an empty dataset, both fBof and fEof are true
end;

Function TDataset.IsLinkedTo(ADataSource: TDataSource): Boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
//!! Not tested, I never used nested DS
  if (ADataSource = nil) or (ADataSource.Dataset = nil) then begin
    Result := False
  end else if ADataSource.Dataset = Self then begin
    Result := True;
  end else begin
    Result := ADataSource.Dataset.IsLinkedTo(ADataSource.Dataset.DataSource);
  end;
//!! DataSetField not implemented
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.IsSequenced: Boolean;
=======
Function TDataset.IsSequenced: Boolean;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.IsSequenced: Boolean;
>>>>>>> origin/fixes_2_2

begin
  Result := True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.Last;
=======
Procedure TDataset.Last;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.Last;
>>>>>>> origin/fixes_2_2

begin
  CheckBiDirectional;
  CheckBrowseMode;
  DoBeforeScroll;
  ClearBuffers;
  try
    InternalLast;
    GetPriorRecords;
    if FRecordCount>0 then
      FActiveRecord:=FRecordCount-1
  finally
    FEOF:=true;
    DataEvent(deDataSetChange, 0);
    DoAfterScroll;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.MoveBy(Distance: Longint): Longint;
Var
  TheResult: Integer;

  Function ScrollForward : Integer;
  begin
    Result:=0;
{$ifdef dsdebug}
    Writeln('Scrolling forward : ',Distance);
=======
=======
>>>>>>> origin/fixes_2_2
Function TDataset.MoveBy(Distance: Longint): Longint;
Var
  TheResult: Integer;

  Function Scrollforward : Integer;

  begin
    Result:=0;
{$ifdef dsdebug}
    Writeln('Scrolling forward :',Distance);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Writeln('Active buffer : ',FActiveRecord);
    Writeln('RecordCount   : ',FRecordCount);
    WriteLn('BufferCount   : ',FBufferCount);
{$endif}
    FBOF:=False;
    While (Distance>0) and not FEOF do
      begin
      If FActiveRecord<FRecordCount-1 then
        begin
        Inc(FActiveRecord);
        Dec(Distance);
        Inc(TheResult); //Inc(Result);
        end
      else
        begin
{$ifdef dsdebug}
       Writeln('Moveby : need next record');
{$endif}
        If GetNextRecord then
          begin
          Dec(Distance);
          Dec(Result);
          Inc(TheResult); //Inc(Result);
          end
        else
          FEOF:=true;
        end;
      end
  end;
<<<<<<< HEAD
<<<<<<< HEAD

  Function ScrollBackward : Integer;
=======
  Function ScrollBackward : Integer;

>>>>>>> graemeg/fixes_2_2
=======
  Function ScrollBackward : Integer;

>>>>>>> origin/fixes_2_2
  begin
    CheckBiDirectional;
    Result:=0;
{$ifdef dsdebug}
<<<<<<< HEAD
<<<<<<< HEAD
    Writeln('Scrolling backward : ',Abs(Distance));
=======
    Writeln('Scrolling backward:',Abs(Distance));
>>>>>>> graemeg/fixes_2_2
=======
    Writeln('Scrolling backward:',Abs(Distance));
>>>>>>> origin/fixes_2_2
    Writeln('Active buffer : ',FActiveRecord);
    Writeln('RecordCunt    : ',FRecordCount);
    WriteLn('BufferCount   : ',FBufferCount);
{$endif}
    FEOF:=False;
    While (Distance<0) and not FBOF do
      begin
      If FActiveRecord>0 then
        begin
        Dec(FActiveRecord);
        Inc(Distance);
        Dec(TheResult); //Dec(Result);
        end
      else
        begin
       {$ifdef dsdebug}
       Writeln('Moveby : need next record');
       {$endif}
        If GetPriorRecord then
          begin
          Inc(Distance);
          Inc(Result);
          Dec(TheResult); //Dec(Result);
          end
        else
          FBOF:=true;
        end;
      end
  end;

Var
  Scrolled : Integer;

begin
  CheckBrowseMode;
  Result:=0; TheResult:=0;
  DoBeforeScroll;
  If (Distance = 0) or
     ((Distance>0) and FEOF) or
     ((Distance<0) and FBOF) then
    exit;
  Try
    Scrolled := 0;
    If Distance>0 then
      Scrolled:=ScrollForward
    else
      Scrolled:=ScrollBackward;
  finally
{$ifdef dsdebug}
    WriteLn('ActiveRecord=', FActiveRecord,' FEOF=',FEOF,' FBOF=',FBOF);
{$Endif}
    DataEvent(deDatasetScroll,Scrolled);
    DoAfterScroll;
    Result:=TheResult;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.Next;

begin
  if BlockReadSize>0 then
    BlockReadNext
  else
    MoveBy(1);
end;

procedure TDataSet.BlockReadNext;
begin
  MoveBy(1);
end;

procedure TDataSet.Open;
=======
=======
>>>>>>> origin/fixes_2_2
Procedure TDataset.Next;

begin
  MoveBy(1);
end;

Procedure TDataset.Open;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  Active:=True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.Post;

begin
  UpdateRecord;
  if State in [dsEdit,dsInsert] then
    begin
=======
=======
>>>>>>> origin/fixes_2_2
Procedure TDataset.Post;

begin
  if State in [dsEdit,dsInsert] then
    begin
    DataEvent(deUpdateRecord,0);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    DataEvent(deCheckBrowseMode,0);
{$ifdef dsdebug}
    writeln ('Post: checking required fields');
{$endif}
    DoBeforePost;
    If Not TryDoing(@InternalPost,OnPostError) then exit;
    cursorposchanged;
{$ifdef dsdebug}
    writeln ('Post: Internalpost succeeded');
{$endif}
    FreeFieldBuffers;
// First set the state to dsBrowse, then the Resync, to prevent the calling of
// the deDatasetChange event, while the state is still 'editable', while the db isn't
    SetState(dsBrowse);
    Resync([]);
{$ifdef dsdebug}
    writeln ('Post: Browse mode set');
{$endif}
    DoAfterPost;
    end
<<<<<<< HEAD
<<<<<<< HEAD
  else if State<>dsSetKey then
    DatabaseErrorFmt(SNotEditing, [Name], Self);
end;

procedure TDataSet.Prior;
=======
=======
>>>>>>> origin/fixes_2_2
  else
    DatabaseErrorFmt(SNotEditing, [Name], Self);
end;

Procedure TDataset.Prior;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  MoveBy(-1);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.Refresh;
=======
Procedure TDataset.Refresh;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.Refresh;
>>>>>>> origin/fixes_2_2

begin
  CheckbrowseMode;
  DoBeforeRefresh;
  UpdateCursorPos;
  InternalRefresh;
{ SetCurrentRecord is called by UpdateCursorPos already, so as long as
  InternalRefresh doesn't do strange things this should be ok. }
<<<<<<< HEAD
<<<<<<< HEAD
//  SetCurrentRecord(FActiveRecord);
=======
//  SetCurrentRecord(FActiverecord);
>>>>>>> graemeg/fixes_2_2
=======
//  SetCurrentRecord(FActiverecord);
>>>>>>> origin/fixes_2_2
  Resync([]);
  DoAfterRefresh;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.RegisterDataSource(ADataSource: TDataSource);

begin
  FDataSources.Add(ADataSource);
=======
=======
>>>>>>> origin/fixes_2_2
Procedure TDataset.RegisterDataSource(ADatasource : TDataSource);

begin
  FDatasources.Add(ADataSource);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  RecalcBufListSize;
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.Resync(Mode: TResyncMode);
=======
Procedure TDataset.Resync(Mode: TResyncMode);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.Resync(Mode: TResyncMode);
>>>>>>> origin/fixes_2_2

var i,count : integer;

begin
  // See if we can find the requested record.
{$ifdef dsdebug}
    Writeln ('Resync called');
{$endif}
  if FIsUnidirectional then Exit;
// place the cursor of the underlying dataset to the active record
<<<<<<< HEAD
<<<<<<< HEAD
//  SetCurrentRecord(FActiveRecord);

// Now look if the data on the current cursor of the underlying dataset is still available
  If GetRecord(FBuffers[0],gmCurrent,False)<>grOk Then
=======
=======
>>>>>>> origin/fixes_2_2
//  SetCurrentRecord(FActiverecord);

// Now look if the data on the current cursor of the underlying dataset is still available
  If GetRecord(Fbuffers[0],gmcurrent,False)<>grOk Then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
// If that fails and rmExact is set, then raise an exception
    If rmExact in Mode then
      DatabaseError(SNoSuchRecord,Self)
// else, if rmexact is not set, try to fetch the next  or prior record in the underlying dataset
<<<<<<< HEAD
<<<<<<< HEAD
    else if (GetRecord(FBuffers[0],gmNext,True)<>grOk) and
            (GetRecord(FBuffers[0],gmPrior,True)<>grOk) then
=======
    else if (GetRecord(Fbuffers[0],gmnext,True)<>grOk) and
            (GetRecord(Fbuffers[0],gmprior,True)<>grOk) then
>>>>>>> graemeg/fixes_2_2
=======
    else if (GetRecord(Fbuffers[0],gmnext,True)<>grOk) and
            (GetRecord(Fbuffers[0],gmprior,True)<>grOk) then
>>>>>>> origin/fixes_2_2
      begin
{$ifdef dsdebug}
      Writeln ('Resync: fuzzy resync');
{$endif}
      // nothing found, invalidate buffer and bail out.
      ClearBuffers;
      // Make sure that the active record is 'empty', ie: that all fields are null
      InternalInitRecord(ActiveBuffer);
      DataEvent(deDatasetChange,0);
      exit;
      end;
  FCurrentRecord := 0;
  FEOF := false;
  FBOF := false;

// If we've arrived here, FBuffer[0] is the current record
  If (rmCenter in Mode) then
    count := (FRecordCount div 2)
  else
    count := FActiveRecord;
  i := 0;
<<<<<<< HEAD
<<<<<<< HEAD
  FRecordCount := 1;
=======
  FRecordcount := 1;
>>>>>>> graemeg/fixes_2_2
=======
  FRecordcount := 1;
>>>>>>> origin/fixes_2_2
  FActiveRecord := 0;

// Fill the buffers before the active record
  while (i < count) and GetPriorRecord do
    inc(i);
  FActiveRecord := i;
// Fill the rest of the buffer
  getnextrecords;
// If the buffer is not full yet, try to fetch some more prior records
<<<<<<< HEAD
<<<<<<< HEAD
  if FRecordCount < FBufferCount then inc(FActiveRecord,getpriorrecords);
=======
  if FRecordcount < FBuffercount then inc(FActiverecord,getpriorrecords);
>>>>>>> graemeg/fixes_2_2
=======
  if FRecordcount < FBuffercount then inc(FActiverecord,getpriorrecords);
>>>>>>> origin/fixes_2_2
// That's all folks!
  DataEvent(deDatasetChange,0);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetFields(const Values: array of const);
=======
Procedure TDataset.SetFields(const Values: array of const);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.SetFields(const Values: array of const);
>>>>>>> origin/fixes_2_2

Var I  : longint;
begin
  For I:=0 to high(Values) do
    Fields[I].AssignValue(Values[I]);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.Translate(Src, Dest: PChar; ToOem: Boolean): Integer;
=======
Function TDataset.Translate(Src, Dest: PChar; ToOem: Boolean): Integer;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.Translate(Src, Dest: PChar; ToOem: Boolean): Integer;
>>>>>>> origin/fixes_2_2

begin
  strcopy(dest,src);
  Result:=StrLen(dest);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.TryDoing(P: TDataOperation; Ev: TDatasetErrorEvent): Boolean;
=======
Function Tdataset.TryDoing (P : TDataOperation; Ev : TDatasetErrorEvent) : Boolean;
>>>>>>> graemeg/fixes_2_2
=======
Function Tdataset.TryDoing (P : TDataOperation; Ev : TDatasetErrorEvent) : Boolean;
>>>>>>> origin/fixes_2_2

Var Retry : TDataAction;

begin
{$ifdef dsdebug}
  Writeln ('Trying to do');
  If P=Nil then writeln ('Procedure to call is nil !!!');
{$endif dsdebug}
  Result:=True;
  Retry:=daRetry;
  while Retry=daRetry do
    Try
{$ifdef dsdebug}
      Writeln ('Trying : updatecursorpos');
{$endif dsdebug}
      UpdateCursorPos;
{$ifdef dsdebug}
      Writeln ('Trying to do it');
{$endif dsdebug}
      P;
      exit;
    except
      On E : EDatabaseError do
        begin
        retry:=daFail;
        If Assigned(Ev) then
          Ev(Self,E,Retry);
        Case Retry of
          daFail : Raise;
<<<<<<< HEAD
<<<<<<< HEAD
          daAbort : Abort;
=======
          daAbort : Result:=False;
>>>>>>> graemeg/fixes_2_2
=======
          daAbort : Result:=False;
>>>>>>> origin/fixes_2_2
        end;
        end;
    else
      Raise;
    end;
{$ifdef dsdebug}
  Writeln ('Exit Trying to do');
{$endif dsdebug}
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.UpdateCursorPos;

begin
  If FRecordCount>0 then
    SetCurrentRecord(FActiveRecord);
end;

procedure TDataSet.UpdateRecord;
=======
=======
>>>>>>> origin/fixes_2_2
Procedure TDataset.UpdateCursorPos;

begin
  If FRecordCount>0 then
    SetCurrentRecord(FactiveRecord);
end;

Procedure TDataset.UpdateRecord;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  if not (State in dsEditModes) then
    DatabaseErrorFmt(SNotEditing, [Name], Self);
  DataEvent(deUpdateRecord, 0);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDataSet.UpdateStatus: TUpdateStatus;
=======
Function TDataSet.UpdateStatus: TUpdateStatus;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataSet.UpdateStatus: TUpdateStatus;
>>>>>>> origin/fixes_2_2

begin
  Result:=usUnmodified;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.RemoveField(Field: TField);
=======
Procedure TDataset.RemoveField (Field : TField);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.RemoveField (Field : TField);
>>>>>>> origin/fixes_2_2

begin
  //!! To be implemented
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetConstraints(Value: TCheckConstraints);
begin
  FConstraints.Assign(Value);
end;

function TDataSet.GetfieldCount: Integer;
=======
Function TDataset.Getfieldcount : Longint;
>>>>>>> graemeg/fixes_2_2
=======
Function TDataset.Getfieldcount : Longint;
>>>>>>> origin/fixes_2_2

begin
  Result:=FFieldList.Count;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.ShiftBuffersBackward;
=======
Procedure TDataset.ShiftBuffersBackward;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDataset.ShiftBuffersBackward;
>>>>>>> origin/fixes_2_2

var TempBuf : pointer;

begin
  TempBuf := FBuffers[0];
<<<<<<< HEAD
<<<<<<< HEAD
  move(FBuffers[1],FBuffers[0],(FBufferCount)*sizeof(FBuffers[0]));
  FBuffers[BufferCount]:=TempBuf;
end;

procedure TDataSet.ShiftBuffersForward;
=======
=======
>>>>>>> origin/fixes_2_2
  move(FBuffers[1],FBuffers[0],(fbuffercount)*sizeof(FBuffers[0]));
  FBuffers[buffercount]:=TempBuf;
end;

Procedure TDataset.ShiftBuffersForward;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

var TempBuf : pointer;

begin
  TempBuf := FBuffers[FBufferCount];
<<<<<<< HEAD
<<<<<<< HEAD
  move(FBuffers[0],FBuffers[1],(FBufferCount)*sizeof(FBuffers[0]));
  FBuffers[0]:=TempBuf;
end;

function TDataSet.GetFieldValues(const FieldName: string): Variant;
=======
=======
>>>>>>> origin/fixes_2_2
  move(FBuffers[0],FBuffers[1],(fbuffercount)*sizeof(FBuffers[0]));
  FBuffers[0]:=TempBuf;
end;

function TDataset.GetFieldValues(Fieldname: string): Variant;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

var i: Integer;
    FieldList: TList;
begin
  FieldList := TList.Create;
  try
    GetFieldList(FieldList, FieldName);
    if FieldList.Count>1 then begin
      Result := VarArrayCreate([0, FieldList.Count - 1], varVariant);
      for i := 0 to FieldList.Count - 1 do
        Result[i] := TField(FieldList[i]).Value;
    end else
      Result := FieldByName(FieldName).Value;
  finally
    FieldList.Free;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDataSet.SetFieldValues(const FieldName: string; Value: Variant);

var
  i, l, h : Integer;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TDataset.SetFieldValues(Fieldname: string; Value: Variant);

var
  i : Integer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FieldList: TList;
begin
  if VarIsArray(Value) then begin
    FieldList := TList.Create;
    try
      GetFieldList(FieldList, FieldName);
<<<<<<< HEAD
<<<<<<< HEAD
      l := VarArrayLowBound(Value, 1);
      h := VarArrayHighBound(Value, 1);
      if (FieldList.Count = 1) and (l < h) then
        // Allow for a field type that can deal with an array
        FieldByName(FieldName).Value := Value
      else
        for i := 0 to FieldList.Count - 1 do
          TField(FieldList[i]).Value := Value[l+i];
=======
      for i := 0 to FieldList.Count -1 do
        TField(FieldList[i]).Value := Value[i];
>>>>>>> graemeg/fixes_2_2
=======
      for i := 0 to FieldList.Count -1 do
        TField(FieldList[i]).Value := Value[i];
>>>>>>> origin/fixes_2_2
    finally
      FieldList.Free;
    end;
  end else
<<<<<<< HEAD
<<<<<<< HEAD
    FieldByName(FieldName).Value := Value;
end;

function TDataSet.Locate(const KeyFields: string; const KeyValues: Variant;
  Options: TLocateOptions): boolean;
=======
=======
>>>>>>> origin/fixes_2_2
    FieldByName(Fieldname).Value := Value;
end;

Function TDataset.Locate(const keyfields: string; const keyvalues: Variant; options: TLocateOptions) : boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2

begin
  CheckBiDirectional;
  Result := False;
end;

<<<<<<< HEAD
function TDataSet.Lookup(const KeyFields: string; const KeyValues: Variant;
  const ResultFields: string): Variant;

begin
  CheckBiDirectional;
  Result := Null;
end;


procedure TDataSet.UnRegisterDataSource(ADataSource: TDataSource);

begin
  FDataSources.Remove(ADataSource);
end;

{------------------------------------------------------------------------------}
{ IProviderSupport methods}

procedure TDataSet.PSEndTransaction(Commit: Boolean);
begin
  DatabaseError('Provider support not available', Self);
end;

procedure TDataSet.PSExecute;
begin
  DatabaseError('Provider support not available', Self);
end;

function TDataSet.PSExecuteStatement(const ASQL: string; AParams: TParams;
  ResultSet: Pointer): Integer;
begin
  Result := 0;
  DatabaseError('Provider support not available', Self);
end;

procedure TDataSet.PSGetAttributes(List: TList);
begin
  DatabaseError('Provider support not available', Self);
end;

function TDataSet.PSGetCommandText: string;
begin
  Result := '';
  DatabaseError('Provider support not available', Self);
end;

function TDataSet.PSGetCommandType: TPSCommandType;
begin
  Result := ctUnknown;
  DatabaseError('Provider support not available', Self);
end;

function TDataSet.PSGetDefaultOrder: TIndexDef;
begin
  Result := nil;
  //DatabaseError('Provider support not available', Self);
end;

function TDataSet.PSGetIndexDefs(IndexTypes: TIndexOptions): TIndexDefs;
begin
  Result := nil;
  DatabaseError('Provider support not available', Self);
end;

function TDataSet.PSGetKeyFields: string;
begin
  Result := '';
  DatabaseError('Provider support not available', Self);
end;

function TDataSet.PSGetParams: TParams;
begin
  Result := nil;
  DatabaseError('Provider support not available', Self);
end;

function TDataSet.PSGetQuoteChar: string;
begin
  Result := '';
  DatabaseError('Provider support not available', Self);
end;

function TDataSet.PSGetTableName: string;
begin
  Result := '';
  DatabaseError('Provider support not available', Self);
end;

function TDataSet.PSGetUpdateException(E: Exception; Prev: EUpdateError
  ): EUpdateError;
begin
  if Prev <> nil then
    Result := EUpdateError.Create(E.Message, '', 0, Prev.ErrorCode, E)
  else
    Result := EUpdateError.Create(E.Message, '', 0, 0, E)
end;

function TDataSet.PSInTransaction: Boolean;
begin
  Result := False;
  DatabaseError('Provider support not available', Self);
end;

function TDataSet.PSIsSQLBased: Boolean;
begin
  Result := False;
  DatabaseError('Provider support not available', Self);
end;

function TDataSet.PSIsSQLSupported: Boolean;
begin
  Result := False;
  DatabaseError('Provider support not available', Self);
end;

procedure TDataSet.PSReset;
begin
  //DatabaseError('Provider support not available', Self);
end;

procedure TDataSet.PSSetCommandText(const CommandText: string);
begin
  DatabaseError('Provider support not available', Self);
end;

procedure TDataSet.PSSetParams(AParams: TParams);
begin
  DatabaseError('Provider support not available', Self);
end;

procedure TDataSet.PSStartTransaction;
begin
  DatabaseError('Provider support not available', Self);
end;

function TDataSet.PSUpdateRecord(UpdateKind: TUpdateKind; Delta: TDataSet
  ): Boolean;
begin
  Result := False;
  DatabaseError('Provider support not available', Self);
end;

{------------------------------------------------------------------------------}

operator Enumerator(ADataSet: TDataSet): TDataSetEnumerator;
begin
 Result:=TDataSetEnumerator.Create(ADataSet);
end;

constructor TDataSetEnumerator.Create(ADataSet: TDataSet);
begin
  inherited Create;
  FDataSet:=ADataSet;
  FBOF:=True;
  FDataSet.First;
end;

function TDataSetEnumerator.GetCurrent: TFields;
begin
  Result := FDataSet.Fields;
end;

function TDataSetEnumerator.MoveNext: Boolean;

begin
  if FBOF then
    FBOF:=False
  else
    FDataSet.Next;
  Result:=not FDataSet.EOF;
end;
=======
=======

begin
  CheckBiDirectional;
  Result := False;
end;

>>>>>>> origin/fixes_2_2
Function TDataset.Lookup(const KeyFields: string; const KeyValues: Variant; const ResultFields: string): Variant;

begin
  CheckBiDirectional;
  Result := False;
end;


Procedure TDataset.UnRegisterDataSource(ADatasource : TDatasource);

begin
  FDataSources.Remove(ADataSource);
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
