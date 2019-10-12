{ ---------------------------------------------------------------------
    TDatalink
  ---------------------------------------------------------------------}

Constructor TDataLink.Create;

begin
  Inherited Create;
  FBufferCount:=1;
  FFirstRecord := 0;
  FDataSource := nil;
  FDatasourceFixed:=False;
end;


Destructor TDataLink.Destroy;

begin
  Factive:=False;
  FEditing:=False;
  FDataSourceFixed:=False;
  DataSource:=Nil;
  Inherited Destroy;
end;


Procedure TDataLink.ActiveChanged;

begin
  FFirstRecord := 0;
end;

Procedure TDataLink.CheckActiveAndEditing;

Var
  B : Boolean;

begin
  B:=Assigned(DataSource) and Not (DataSource.State in [dsInactive,dsOpening]);
  If B<>FActive then
    begin
    FActive:=B;
    ActiveChanged;
    end;
  B:=Assigned(DataSource) and (DataSource.State in dsEditModes) and Not FReadOnly;
  If B<>FEditing Then
    begin
    FEditing:=B;
    EditingChanged;
    end;
end;


Procedure TDataLink.CheckBrowseMode;

begin
end;


Function TDataLink.CalcFirstRecord(Index : Integer) : Integer;
begin
  if DataSource.DataSet.FActiveRecord > FFirstRecord + Index + FBufferCount - 1 then
    Result := DataSource.DataSet.FActiveRecord - (FFirstRecord + Index + FBufferCount - 1)
  else if DataSource.DataSet.FActiveRecord < FFirstRecord + Index then
    Result := DataSource.DataSet.FActiveRecord - (FFirstRecord + Index)
  else
    Result := 0;
  Inc(FFirstRecord, Index + Result);
end;


Procedure TDataLink.CalcRange;
var
    aMax, aMin: integer;
begin
  // During loading, dataset may not be set yet.
  If (Dataset=Nil) then
    begin
    FFirstRecord:=0; 
    exit;
    end;
  aMin:= DataSet.FActiveRecord - FBufferCount + 1;
  If aMin < 0 Then aMin:= 0;
  aMax:= Dataset.FBufferCount - FBufferCount;
  If aMax < 0 then aMax:= 0;

  If aMax>DataSet.FActiveRecord Then aMax:=DataSet.FActiveRecord;

  If FFirstRecord < aMin Then FFirstRecord:= aMin;
  If FFirstrecord > aMax Then FFirstRecord:= aMax;

  If (FfirstRecord<>0) And
     (DataSet.FActiveRecord - FFirstRecord < FBufferCount -1) Then
    Dec(FFirstRecord, 1);

end;


Procedure TDataLink.DataEvent(Event: TDataEvent; Info: Ptrint);


begin
  Case Event of
    deFieldChange, deRecordChange:
      If Not FUpdatingRecord then
        RecordChanged(TField(Info));
    deDataSetChange: begin
      SetActive(DataSource.DataSet.Active);
      CalcRange;
      CalcFirstRecord(Info);
      DatasetChanged;
    end;
    deDataSetScroll: DatasetScrolled(CalcFirstRecord(Info));
    deLayoutChange: begin
      CalcFirstRecord(Info);
      LayoutChanged;
    end;
    deUpdateRecord: UpdateRecord;
    deUpdateState: CheckActiveAndEditing;
    deCheckBrowseMode: CheckBrowseMode;
    deFocusControl: FocusControl(TFieldRef(Info));
  end;
end;


Procedure TDataLink.DataSetChanged;

begin
  RecordChanged(Nil);
end;


Procedure TDataLink.DataSetScrolled(Distance: Integer);

begin
  DataSetChanged;
end;


Procedure TDataLink.EditingChanged;

begin
end;


Procedure TDataLink.FocusControl(Field: TFieldRef);

begin
end;


Function TDataLink.GetActiveRecord: Integer;

begin
  Result:=Dataset.FActiveRecord - FFirstRecord;
end;

Function TDatalink.GetDataSet : TDataset;

begin
  If Assigned(Datasource) then
    Result:=DataSource.DataSet
  else
    Result:=Nil;  
end;


Function TDataLink.GetBOF: Boolean;

begin
  Result:=DataSet.BOF
end;


Function TDataLink.GetBufferCount: Integer;

begin
  Result:=FBufferCount;
end;


Function TDataLink.GetEOF: Boolean;

begin
  Result:=DataSet.EOF
end;


Function TDataLink.GetRecordCount: Integer;

begin
  Result:=Dataset.FRecordCount;
  If Result>BufferCount then
    Result:=BufferCount;
end;


Procedure TDataLink.LayoutChanged;

begin
  DataSetChanged;
end;


Function TDataLink.MoveBy(Distance: Integer): Integer;

begin
  Result:=DataSet.MoveBy(Distance);
end;


Procedure TDataLink.RecordChanged(Field: TField);

begin
end;


Procedure TDataLink.SetActiveRecord(Value: Integer);

begin
{$ifdef dsdebug}
  Writeln('Datalink. Setting active record to ',Value,' with firstrecord ',ffirstrecord);
{$endif}
  Dataset.FActiveRecord:=Value + FFirstRecord;
end;


Procedure TDataLink.SetBufferCount(Value: Integer);

begin
  If FBufferCount<>Value then
    begin
      FBufferCount:=Value;
      if Active then begin
        DataSet.RecalcBufListSize;
        CalcRange;
      end;
    end;
end;

procedure TDataLink.SetActive(AActive: Boolean);
begin
  if Active <> AActive then
  begin
    FActive := AActive;
    // !!!: Set internal state
    ActiveChanged;
  end;
end;

Procedure TDataLink.SetDataSource(Value : TDatasource);

begin
  if FDataSource = Value then
    Exit;
  if not FDataSourceFixed then
    begin
    if Assigned(DataSource) then
      Begin
      DataSource.UnregisterDatalink(Self);
      FDataSource := nil;
      CheckActiveAndEditing;
      End;
    FDataSource := Value;
    if Assigned(DataSource) then
      begin
      DataSource.RegisterDatalink(Self);
      CheckActiveAndEditing;
      End;
    end;
end;

Procedure TDatalink.SetReadOnly(Value : Boolean);

begin
  If FReadOnly<>Value then
    begin
    FReadOnly:=Value;
    CheckActiveAndEditing;
    end;
end;

Procedure TDataLink.UpdateData;

begin
end;



Function TDataLink.Edit: Boolean;

begin
  If Not FReadOnly then
    DataSource.Edit;
  // Triggered event will set FEditing
  Result:=FEditing;
end;


Procedure TDataLink.UpdateRecord;

begin
  FUpdatingRecord:=True;
  Try
    UpdateData;
  finally
    FUpdatingRecord:=False;
  end;
end;

function TDataLink.ExecuteAction(Action: TBasicAction): Boolean;
begin
 if Action.HandlesTarget(DataSource) then
 begin
   Action.ExecuteTarget(DataSource);
   Result := True;
 end
 else Result := False;
end;

function TDataLink.UpdateAction(Action: TBasicAction): Boolean;
begin
 if Action.HandlesTarget(DataSource) then
 begin
   Action.UpdateTarget(DataSource);
   Result := True;
 end
 else Result := False;
end;


{ ---------------------------------------------------------------------
    TDetailDataLink
  ---------------------------------------------------------------------}

Function TDetailDataLink.GetDetailDataSet: TDataSet;

begin
  Result := nil;
end;


{ ---------------------------------------------------------------------
    TMasterDataLink
  ---------------------------------------------------------------------}

constructor TMasterDataLink.Create(ADataSet: TDataSet);

begin
  inherited Create;
  FDetailDataSet:=ADataSet;
  FFields:=TList.Create;
end;


destructor TMasterDataLink.Destroy;

begin
  FFields.Free;
  inherited Destroy;
end;


Procedure TMasterDataLink.ActiveChanged;

begin
  FFields.Clear;
  if Active then
    try
      DataSet.GetFieldList(FFields, FFieldNames);
    except
      FFields.Clear;
      raise;
    end;
  if FDetailDataSet.Active and not (csDestroying in FDetailDataSet.ComponentState) then
    if Active and (FFields.Count > 0) then
      DoMasterChange
    else
      DoMasterDisable;  
end;


Procedure TMasterDataLink.CheckBrowseMode;

begin
  if FDetailDataSet.Active then FDetailDataSet.CheckBrowseMode;
end;


Function TMasterDataLink.GetDetailDataSet: TDataSet;

begin
  Result := FDetailDataSet;
end;


Procedure TMasterDataLink.LayoutChanged;

begin
  ActiveChanged;
end;


Procedure TMasterDataLink.RecordChanged(Field: TField);

begin
  if (DataSource.State <> dsSetKey) and FDetailDataSet.Active and
     (FFields.Count > 0) and ((Field = nil) or
     (FFields.IndexOf(Field) >= 0)) then
    DoMasterChange;  
end;

procedure TMasterDatalink.SetFieldNames(const Value: string);

begin
  if FFieldNames <> Value then
    begin
    FFieldNames := Value;
    ActiveChanged;
    end;
end;

Procedure TMasterDataLink.DoMasterDisable; 

begin
  if Assigned(FOnMasterDisable) then 
    FOnMasterDisable(Self);
end;

Procedure TMasterDataLink.DoMasterChange; 

begin
  If Assigned(FOnMasterChange) then
    FOnMasterChange(Self);
end;

{ ---------------------------------------------------------------------
    TMasterParamsDataLink
  ---------------------------------------------------------------------}

constructor TMasterParamsDataLink.Create(ADataSet: TDataSet);

Var
  P : TParams;

begin
  inherited Create(ADataset);
  If (ADataset<>Nil) then
    begin
    P:=TParams(GetObjectProp(ADataset,'Params',TParams));
    if (P<>Nil) then
      Params:=P;
    end;  
end;


Procedure TMasterParamsDataLink.SetParams(AValue : TParams);

begin
  FParams:=AValue;
  If (AValue<>Nil) then
    RefreshParamNames;
end;

Procedure TMasterParamsDataLink.RefreshParamNames; 

Var
  FN : String;
  DS : TDataset;
  F  : TField;
  I : Integer;
  P : TParam;


begin
  FN:='';
  DS:=Dataset;
  If Assigned(FParams) then
    begin
    F:=Nil;
    For I:=0 to FParams.Count-1 do
      begin
      P:=FParams[i];
      if not P.Bound then
        begin
        If Assigned(DS) then
          F:=DS.FindField(P.Name);
        If (Not Assigned(DS)) or (not DS.Active) or (F<>Nil) then
          begin
          If (FN<>'') then
            FN:=FN+';';
          FN:=FN+P.Name;
          end;
        end;
      end;
    end;
  FieldNames:=FN;  
end;

Procedure TMasterParamsDataLink.CopyParamsFromMaster(CopyBound : Boolean);

begin
  if Assigned(FParams) then
    FParams.CopyParamValuesFromDataset(Dataset,CopyBound);
end;

Procedure TMasterParamsDataLink.DoMasterDisable; 

begin
  Inherited;
  // If master dataset is closing, leave detail dataset intact (Delphi compatible behavior)
  // If master dataset is reopened, relationship will be reestablished
end;

Procedure TMasterParamsDataLink.DoMasterChange; 

begin
  Inherited;
  if Assigned(Params) and Assigned(DetailDataset) and DetailDataset.Active then
    begin
    DetailDataSet.CheckBrowseMode;
    DetailDataset.Close;
    DetailDataset.Open;
    end;
end;

{ ---------------------------------------------------------------------
    TDatasource
  ---------------------------------------------------------------------}

Constructor TDataSource.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  FDatalinks := TList.Create;
  FEnabled := True;
  FAutoEdit := True;
end;


Destructor TDataSource.Destroy;

begin
  FOnStateCHange:=Nil;
  Dataset:=Nil;
  With FDataLinks do
    While Count>0 do
      TDatalink(Items[Count - 1]).DataSource:=Nil;
  FDatalinks.Free;
  inherited Destroy;
end;


Procedure TDatasource.Edit;

begin
  If (State=dsBrowse) and AutoEdit Then
    Dataset.Edit;
end;


Function TDataSource.IsLinkedTo(ADataset: TDataSet): Boolean;

var
  DS: TDataSource;

begin
  Result:=False;
  Repeat
    DS:=aDataset.GetDataSource;
    Result:=(DS=Self);
    if Assigned(DS) then
      aDataSet := DS.DataSet
    else
      aDataSet := Nil;
  Until Result or (aDataset=Nil)
end;


procedure TDatasource.DistributeEvent(Event: TDataEvent; Info: Ptrint);


Var
  i : Longint;

begin
  With FDatalinks do
    begin
    For I:=0 to Count-1 do
      With TDatalink(Items[i]) do
        If Not VisualControl Then
          DataEvent(Event,Info);
    For I:=0 to Count-1 do
      With TDatalink(Items[i]) do
        If VisualControl Then
          DataEvent(Event,Info);
    end;
end;

function TDataSource.GetLink(AIndex : Integer): TDataLink;
begin
  Result:=TDataLink(FDataLinks[aindex]);
end;

function TDataSource.GetLinkCount: Integer;
begin
  Result:=FDataLinks.Count;
end;

procedure TDatasource.RegisterDataLink(DataLink: TDataLink);

begin
  FDatalinks.Add(DataLink);
  if Assigned(DataSet) then
    DataSet.RecalcBufListSize;
end;


procedure TDatasource.SetDataSet(ADataSet: TDataSet);
begin
  If FDataset<>Nil Then
    Begin
    FDataset.UnRegisterDataSource(Self);
    FDataSet:=nil;
    ProcessEvent(deUpdateState,0);
    End;
  If ADataset<>Nil Then
    begin
    if IsLinkedTo(aDataset) then
      DatabaseError(SErrCircularDataSourceReferenceNotAllowed,Self);
    FDataSet:=ADataset;
    ADataset.RegisterDatasource(Self);
    ProcessEvent(deUpdateState,0);
    End;
end;


procedure TDatasource.SetEnabled(Value: Boolean);

begin
  FEnabled:=Value;
end;


Procedure TDatasource.DoDataChange (Info : Pointer);

begin
  If Assigned(OnDataChange) Then
    OnDataChange(Self,TField(Info));
end;

Procedure TDatasource.DoStateChange;

begin
  If Assigned(OnStateChange) Then
    OnStateChange(Self);
end;


Procedure TDatasource.DoUpdateData;

begin
  If Assigned(OnUpdateData) Then
    OnUpdateData(Self);
end;


procedure TDatasource.UnregisterDataLink(DataLink: TDataLink);

begin
  FDatalinks.Remove(Datalink);
  If Dataset<>Nil then
    DataSet.RecalcBufListSize;
  //Dataset.SetBufListSize(DataLink.BufferCount);
end;


procedure TDataSource.ProcessEvent(Event : TDataEvent; Info : Ptrint);

Const
    OnDataChangeEvents = [deRecordChange, deDataSetChange, deDataSetScroll,
                          deLayoutChange,deUpdateState];

Var
  NeedDataChange : Boolean;
  FLastState : TDataSetState;

begin
  // Special UpdateState handling.
  If Event=deUpdateState then
    begin
    NeedDataChange:=(FState=dsInactive);
    FLastState:=FState;
    If Assigned(Dataset) then
      FState:=Dataset.State
    else
      FState:=dsInactive;
    // Don't do events if nothing changed.
    If FState=FLastState then
      exit;
    end
  else
    NeedDataChange:=True;
  if (Event<>deUpdateState) and (FState=dsInactive) then
    Exit;
  DistributeEvent(Event,Info);
  // Extra handlers
  If Not (csDestroying in ComponentState) then
    begin
    If (Event=deUpdateState) then
      DoStateChange;
    If (Event in OnDataChangeEvents) and NeedDataChange Then
      DoDataChange(Nil);
    If (Event = deFieldChange) Then
      DoDataChange(Pointer(Info));
    If (Event=deUpdateRecord) then
      DoUpdateData;
    end;
 end;
