{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt, member of the
    Free Pascal development team

    TDatabase and related objects implementation

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ ---------------------------------------------------------------------
    TDatabase
  ---------------------------------------------------------------------}

Procedure TDatabase.CheckConnected;

begin
  If Not Connected Then
    DatabaseError(SNotConnected,Self);
end;


Procedure TDatabase.CheckDisConnected;
begin
  If Connected Then
    DatabaseError(SConnected,Self);
end;

procedure TDatabase.DoConnect;
begin
  DoInternalConnect;
  FConnected := True;
end;

procedure TDatabase.DoDisconnect;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  CloseDatasets;
  CloseTransactions;
  DoInternalDisConnect;
  if csLoading in ComponentState then
=======
=======
>>>>>>> origin/fixes_2_2
  Closedatasets;
  Closetransactions;
  DoInternalDisConnect;
  if csloading in ComponentState then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FOpenAfterRead := false;
  FConnected := False;
end;

function TDatabase.GetConnected: boolean;
begin
  Result:= FConnected;
end;

constructor TDatabase.Create(AOwner: TComponent);

begin
  Inherited Create(AOwner);
  FParams:=TStringlist.Create;
  FDatasets:=TList.Create;
  FTransactions:=TList.Create;
  FConnected:=False;
end;

destructor TDatabase.Destroy;

begin
  Connected:=False;
  RemoveDatasets;
  RemoveTransactions;
  FDatasets.Free;
  FTransactions.Free;
  FParams.Free;
  Inherited Destroy;
end;

procedure TDatabase.CloseDataSets;

Var I : longint;

begin
  If Assigned(FDatasets) then
    begin
    For I:=FDatasets.Count-1 downto 0 do
      TDataset(FDatasets[i]).Close;
    end;
end;

procedure TDatabase.CloseTransactions;

Var I : longint;

begin
  If Assigned(FTransactions) then
    begin
    For I:=FTransactions.Count-1 downto 0 do
<<<<<<< HEAD
<<<<<<< HEAD
      try
        TDBTransaction(FTransactions[i]).EndTransaction;
      except
        if not ForcedClose then
          Raise;
      end;    
=======
      TDBTransaction(FTransactions[i]).EndTransaction;
>>>>>>> graemeg/fixes_2_2
=======
      TDBTransaction(FTransactions[i]).EndTransaction;
>>>>>>> origin/fixes_2_2
    end;
end;

procedure TDatabase.RemoveDataSets;

Var I : longint;

begin
  If Assigned(FDatasets) then
    For I:=FDataSets.Count-1 downto 0 do
      TDBDataset(FDataSets[i]).Database:=Nil;
end;

procedure TDatabase.RemoveTransactions;

Var I : longint;

begin
  If Assigned(FTransactions) then
    For I:=FTransactions.Count-1 downto 0 do
      TDBTransaction(FTransactions[i]).Database:=Nil;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
procedure TDatabase.SetParams(AValue: TStrings);
begin
  if AValue<>nil then
    FParams.Assign(AValue);
end;

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
Function TDatabase.GetDataSetCount : Longint;

begin
  If Assigned(FDatasets) Then
    Result:=FDatasets.Count
  else
    Result:=0;
end;

Function TDatabase.GetTransactionCount : Longint;

begin
  If Assigned(FTransactions) Then
    Result:=FTransactions.Count
  else
    Result:=0;
end;

Function TDatabase.GetDataset(Index : longint) : TDataset;

begin
  If Assigned(FDatasets) then
    Result:=TDataset(FDatasets[Index])
  else
    begin
    result := nil;
    DatabaseError(SNoDatasets);
    end;
end;

Function TDatabase.GetTransaction(Index : longint) : TDBtransaction;

begin
  If Assigned(FTransactions) then
    Result:=TDBTransaction(FTransactions[Index])
  else
    begin
    result := nil;
    DatabaseError(SNoTransactions);
    end;
end;

procedure TDatabase.RegisterDataset (DS : TDBDataset);

Var I : longint;

begin
  I:=FDatasets.IndexOf(DS);
  If I=-1 then
    FDatasets.Add(DS)
  else
    DatabaseErrorFmt(SDatasetRegistered,[DS.Name]);
end;

procedure TDatabase.RegisterTransaction (TA : TDBTransaction);

Var I : longint;

begin
  I:=FTransactions.IndexOf(TA);
  If I=-1 then
    FTransactions.Add(TA)
  else
    DatabaseErrorFmt(STransactionRegistered,[TA.Name]);
end;

procedure TDatabase.UnRegisterDataset (DS : TDBDataset);

Var I : longint;

begin
  I:=FDatasets.IndexOf(DS);
  If I<>-1 then
    FDatasets.Delete(I)
  else
    DatabaseErrorFmt(SNoDatasetRegistered,[DS.Name]);
end;

procedure TDatabase.UnRegisterTransaction (TA : TDBTransaction);

Var I : longint;

begin
  I:=FTransactions.IndexOf(TA);
  If I<>-1 then
    FTransactions.Delete(I)
  else
    DatabaseErrorFmt(SNoTransactionRegistered,[TA.Name]);
end;


{ ---------------------------------------------------------------------
<<<<<<< HEAD
<<<<<<< HEAD
    TDBDataset
=======
    TDBdataset
>>>>>>> graemeg/fixes_2_2
=======
    TDBdataset
>>>>>>> origin/fixes_2_2
  ---------------------------------------------------------------------}

Procedure TDBDataset.SetDatabase (Value : TDatabase);

begin
  If Value<>FDatabase then
    begin
    CheckInactive;
    If Assigned(FDatabase) then
      FDatabase.UnregisterDataset(Self);
    If Value<>Nil Then
      Value.RegisterDataset(Self);
    FDatabase:=Value;
    end;
end;

Procedure TDBDataset.SetTransaction (Value : TDBTransaction);

begin
  CheckInactive;
  If Value<>FTransaction then
    begin
    If Assigned(FTransaction) then
      FTransaction.UnregisterDataset(Self);
    If Value<>Nil Then
      Value.RegisterDataset(Self);
    FTransaction:=Value;
    end;
end;

Procedure TDBDataset.CheckDatabase;

begin
  If (FDatabase=Nil) then
    DatabaseError(SErrNoDatabaseAvailable,Self)
end;

Destructor TDBDataset.Destroy;

begin
  Database:=Nil;
  Transaction:=Nil;
  Inherited;
end;

{ ---------------------------------------------------------------------
    TDBTransaction
  ---------------------------------------------------------------------}
procedure TDBTransaction.SetActive(Value : boolean);
begin
  if FActive and (not Value) then
    EndTransaction
  else if (not FActive) and Value then
    if csLoading in ComponentState then
      begin
      FOpenAfterRead := true;
      exit;
      end
    else
      StartTransaction;
end;

procedure TDBTransaction.Loaded;

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

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDBTransaction.InternalHandleException;
=======
Procedure TDBTransaction.InternalHandleException;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDBTransaction.InternalHandleException;
>>>>>>> origin/fixes_2_2

begin
  if assigned(classes.ApplicationHandleException) then
    classes.ApplicationHandleException(self)
  else
    ShowException(ExceptObject,ExceptAddr);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDBTransaction.CheckActive;
=======
Procedure TDBTransaction.CheckActive;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDBTransaction.CheckActive;
>>>>>>> origin/fixes_2_2

begin
  If not FActive Then
    DatabaseError(STransNotActive,Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDBTransaction.CheckInactive;
=======
Procedure TDBTransaction.CheckInActive;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDBTransaction.CheckInActive;
>>>>>>> origin/fixes_2_2

begin
  If FActive Then
    DatabaseError(STransActive,Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDBTransaction.Commit;
begin
  EndTransaction;
end;

procedure TDBTransaction.CommitRetaining;
begin
  Commit;
  StartTransaction;
end;

procedure TDBTransaction.Rollback;
begin
  EndTransaction;
end;

procedure TDBTransaction.RollbackRetaining;
begin
  RollBack;
  StartTransaction;
end;

procedure TDBTransaction.CloseTrans;
=======
Procedure TDBTransaction.CloseTrans;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDBTransaction.CloseTrans;
>>>>>>> origin/fixes_2_2

begin
  FActive := false;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDBTransaction.OpenTrans;
=======
Procedure TDBTransaction.OpenTrans;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDBTransaction.OpenTrans;
>>>>>>> origin/fixes_2_2

begin
  FActive := true;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDBTransaction.SetDatabase(Value: TDatabase);
=======
Procedure TDBTransaction.SetDatabase (Value : TDatabase);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TDBTransaction.SetDatabase (Value : TDatabase);
>>>>>>> origin/fixes_2_2

begin
  If Value<>FDatabase then
    begin
    CheckInactive;
    If Assigned(FDatabase) then
      FDatabase.UnregisterTransaction(Self);
    If Value<>Nil Then
      Value.RegisterTransaction(Self);
    FDatabase:=Value;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
constructor TDBTransaction.Create(AOwner: TComponent);

begin
  inherited Create(AOwner);
  FDatasets:=TList.Create;
end;

procedure TDBTransaction.CheckDatabase;
=======
=======
>>>>>>> origin/fixes_2_2
constructor TDBTransaction.create(AOwner : TComponent);

begin
  inherited create(AOwner);
  FDatasets:=TList.Create;
end;

Procedure TDBTransaction.CheckDatabase;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  If (FDatabase=Nil) then
    DatabaseError(SErrNoDatabaseAvailable,Self)
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TDBTransaction.AllowClose(DS : TDBDataset) : Boolean;

begin
  Result:=Assigned(DS);
end;

procedure TDBTransaction.CloseDataSets;

Var
  I : longint;
  DS : TDBDataset;
=======
procedure TDBTransaction.CloseDataSets;

Var I : longint;
>>>>>>> graemeg/fixes_2_2
=======
procedure TDBTransaction.CloseDataSets;

Var I : longint;
>>>>>>> origin/fixes_2_2

begin
  If Assigned(FDatasets) then
    begin
    For I:=FDatasets.Count-1 downto 0 do
<<<<<<< HEAD
<<<<<<< HEAD
      begin
      DS:=TDBDataset(FDatasets[i]);
      If AllowClose(DS) then
        DS.Close;
      end;
    end;
end;

destructor TDBTransaction.Destroy;
=======
=======
>>>>>>> origin/fixes_2_2
      TDBDataset(FDatasets[i]).Close;
    end;
end;

Destructor TDBTransaction.Destroy;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  Database:=Nil;
  CloseDataSets;
  RemoveDatasets;
  FDatasets.Free;
  Inherited;
end;

procedure TDBTransaction.RemoveDataSets;

Var I : longint;

begin
  If Assigned(FDatasets) then
    For I:=FDataSets.Count-1 downto 0 do
      TDBDataset(FDataSets[i]).Transaction:=Nil;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDBTransaction.GetDataset(Index: longint): TDBDataset;

begin
  If Assigned(FDatasets) then
    Result:=TDBDataset(FDatasets[Index])
  else
  begin
    Result := nil;
    DatabaseError(SNoDatasets);
  end;
end;

function TDBTransaction.GetDataSetCount: Longint;

begin
  If Assigned(FDatasets) Then
    Result:=FDatasets.Count
  else
    Result:=0;
=======
=======
>>>>>>> origin/fixes_2_2
Function TDBTransaction.GetDataSetCount : Longint;

begin
  If Assigned(FDatasets) Then
    Result:=FDatasets.Count
  else
    Result:=0;
end;

procedure TDBTransaction.UnRegisterDataset (DS : TDBDataset);

Var I : longint;

begin
  I:=FDatasets.IndexOf(DS);
  If I<>-1 then
    FDatasets.Delete(I)
  else
    DatabaseErrorFmt(SNoDatasetRegistered,[DS.Name]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TDBTransaction.RegisterDataset (DS : TDBDataset);

Var I : longint;

begin
  I:=FDatasets.IndexOf(DS);
  If I=-1 then
    FDatasets.Add(DS)
  else
    DatabaseErrorFmt(SDatasetRegistered,[DS.Name]);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDBTransaction.UnRegisterDataset (DS : TDBDataset);

Var I : longint;

begin
  I:=FDatasets.IndexOf(DS);
  If I<>-1 then
    FDatasets.Delete(I)
  else
    DatabaseErrorFmt(SNoDatasetRegistered,[DS.Name]);
=======
=======
>>>>>>> origin/fixes_2_2
Function TDBTransaction.GetDataset(Index : longint) : TDBDataset;

begin
  If Assigned(FDatasets) then
    Result:=TDBDataset(FDatasets[Index])
  else
  begin
    result := nil;
    DatabaseError(SNoDatasets);
  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

{ ---------------------------------------------------------------------
    TCustomConnection
  ---------------------------------------------------------------------}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
procedure TCustomConnection.SetAfterConnect(const AValue: TNotifyEvent);
begin
  FAfterConnect:=AValue;
end;

>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
procedure TCustomConnection.SetAfterConnect(const AValue: TNotifyEvent);
begin
  if FAfterConnect=AValue then exit;
  FAfterConnect:=AValue;
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TCustomConnection.GetDataSet(Index: Longint): TDataSet;
begin
  Result := nil;
end;

function TCustomConnection.GetDataSetCount: Longint;
begin
  Result := 0;
end;

procedure TCustomConnection.InternalHandleException;
begin
  if assigned(classes.ApplicationHandleException) then
    classes.ApplicationHandleException(self)
  else
    ShowException(ExceptObject,ExceptAddr);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomConnection.SetAfterConnect(const AValue: TNotifyEvent);
begin
  FAfterConnect:=AValue;
end;

procedure TCustomConnection.SetAfterDisconnect(const AValue: TNotifyEvent);
begin
=======
procedure TCustomConnection.SetAfterDisconnect(const AValue: TNotifyEvent);
begin
  if FAfterDisconnect=AValue then exit;
>>>>>>> graemeg/fixes_2_2
=======
procedure TCustomConnection.SetAfterDisconnect(const AValue: TNotifyEvent);
begin
  if FAfterDisconnect=AValue then exit;
>>>>>>> origin/fixes_2_2
  FAfterDisconnect:=AValue;
end;

procedure TCustomConnection.SetBeforeConnect(const AValue: TNotifyEvent);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FBeforeConnect:=AValue;
end;

procedure TCustomConnection.SetBeforeDisconnect(const AValue: TNotifyEvent);
begin
  FBeforeDisconnect:=AValue;
end;

procedure TCustomConnection.DoLoginPrompt;

var
  ADatabaseName, AUserName, APassword: string;

begin
  if FLoginPrompt then
    begin
    GetLoginParams(ADatabaseName, AUserName, APassword);
    if Assigned(FOnLogin) then
      FOnLogin(Self, AUserName, APassword)
    else if Assigned(LoginDialogExProc) then
      LoginDialogExProc(ADatabaseName, AUserName, APassword, False);
    SetLoginParams(ADatabaseName, AUserName, APassword);
    end;
end;

procedure TCustomConnection.SetConnected(Value: boolean);

=======
=======
>>>>>>> origin/fixes_2_2
  if FBeforeConnect=AValue then exit;
  FBeforeConnect:=AValue;
end;

procedure TCustomConnection.SetConnected(Value: boolean);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  If Value<>Connected then
    begin
    If Value then
      begin
      if csReading in ComponentState then
        begin
        FStreamedConnected := true;
        exit;
        end
      else
        begin
        if Assigned(BeforeConnect) then
          BeforeConnect(self);
<<<<<<< HEAD
<<<<<<< HEAD
        DoLoginPrompt;
=======
        if FLoginPrompt then if assigned(FOnLogin) then
          FOnLogin(self,'','');
>>>>>>> graemeg/fixes_2_2
=======
        if FLoginPrompt then if assigned(FOnLogin) then
          FOnLogin(self,'','');
>>>>>>> origin/fixes_2_2
        DoConnect;
        if Assigned(AfterConnect) then
          AfterConnect(self);
        end;
      end
    else
      begin
      if Assigned(BeforeDisconnect) then
        BeforeDisconnect(self);
      DoDisconnect;
      if Assigned(AfterDisconnect) then
        AfterDisconnect(self);
      end;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomConnection.GetLoginParams(out ADatabaseName, AUserName, APassword: string);
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if IsPublishedProp(Self,'DatabaseName') then
    ADatabaseName := GetStrProp(Self,'DatabaseName');
  if IsPublishedProp(Self,'UserName') then
    AUserName := GetStrProp(Self,'UserName');
  if IsPublishedProp(Self,'Password') then
    APassword := 'Password';
end;

procedure TCustomConnection.SetLoginParams(const ADatabaseName, AUserName, APassword: string);
begin
  if IsPublishedProp(Self,'DatabaseName') then
    SetStrProp(Self,'DatabaseName',ADatabaseName);
  if IsPublishedProp(Self,'UserName') then
    SetStrProp(Self,'UserName',AUserName);
  if IsPublishedProp(Self,'Password') then
    SetStrProp(Self,'Password',APassword);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomConnection.SetBeforeDisconnect(const AValue: TNotifyEvent);
begin
>>>>>>> origin/cpstrnew
  FBeforeDisconnect:=AValue;
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
procedure TCustomConnection.SetBeforeDisconnect(const AValue: TNotifyEvent);
begin
  if FBeforeDisconnect=AValue then exit;
  FBeforeDisconnect:=AValue;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TCustomConnection.DoConnect;

begin
  // Do nothing yet
end;

procedure TCustomConnection.DoDisconnect;

begin
  // Do nothing yet
end;

function TCustomConnection.GetConnected: boolean;

begin
  Result := False;
end;

procedure TCustomConnection.Loaded;
begin
  inherited Loaded;
  try
    if FStreamedConnected then
      SetConnected(true);
  except
    if csDesigning in Componentstate then
      InternalHandleException
    else
      raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomConnection.Close(ForceClose : Boolean = False);
begin
  try
    ForcedClose:=ForceClose;
    Connected := False;
  finally
    ForcedClose:=false;
  end;
=======
procedure TCustomConnection.Close;
begin
  Connected := False;
>>>>>>> graemeg/fixes_2_2
=======
procedure TCustomConnection.Close;
begin
  Connected := False;
>>>>>>> origin/fixes_2_2
end;

destructor TCustomConnection.Destroy;
begin
  Connected:=False;
  Inherited Destroy;
end;

procedure TCustomConnection.Open;
begin
  Connected := True;
end;

