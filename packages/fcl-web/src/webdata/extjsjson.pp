unit extjsjson;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, httpdefs, fphttp, fpwebdata, fpextjs, fpjson, db, jsonparser;

type

  { TExtJSJSonWebdataInputAdaptor }

  TExtJSJSonWebdataInputAdaptor = CLass(TCustomWebdataInputAdaptor)
  private
    FRows : TJSONArray;
    FCurrentRow : TJSONObject;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    FIDValue : TJSONData;
>>>>>>> graemeg/cpstrnew
=======
    FIDValue : TJSONData;
>>>>>>> graemeg/cpstrnew
=======
    FIDValue : TJSONData;
>>>>>>> graemeg/cpstrnew
=======
    FIDValue : TJSONData;
>>>>>>> origin/cpstrnew
    FRowIndex : integer;
    function CheckData: Boolean;
  Public
    procedure reset; override;
    Function GetNextBatch : Boolean; override;
    Function TryFieldValue(Const AFieldName : String; out AValue : String) : Boolean; override;
    Destructor destroy; override;
  end;

  { TExtJSJSONDataFormatter }
  TJSONObjectEvent = Procedure(Sender : TObject; AObject : TJSONObject) of Object;
  TJSONExceptionObjectEvent = Procedure(Sender : TObject; E : Exception; AResponse : TJSONObject) of Object;
  TJSONObjectAllowRowEvent = Procedure(Sender : TObject; Dataset : TDataset; Var Allow : Boolean) of Object;
  TJSONObjectAllowEvent = Procedure(Sender : TObject; AObject : TJSONObject; Var Allow : Boolean) of Object;

  TExtJSJSONDataFormatter = Class(TExtJSDataFormatter)
  private
    FAfterDataToJSON: TJSONObjectEvent;
    FAfterDelete: TJSONObjectEvent;
    FAfterInsert: TJSONObjectEvent;
    FAfterRowToJSON: TJSONObjectEvent;
    FAfterUpdate: TJSONObjectEvent;
    FBeforeDataToJSON: TJSONObjectEvent;
    FBeforeDelete: TNotifyEvent;
    FBeforeInsert: TNotifyEvent;
    FBeforeRowToJSON: TJSONObjectEvent;
    FBeforeUpdate: TNotifyEvent;
    FOnAllowRow: TJSONObjectAllowRowEvent;
    FOnErrorResponse: TJSONExceptionObjectEvent;
    FOnMetaDataToJSON: TJSONObjectEvent;
    FBatchResult : TJSONArray;
    Function AddIdToBatch : TJSONObject;
    procedure SendSuccess(ResponseContent: TStream; AddIDValue : Boolean = False);
  protected
    function AllowRow(ADataset : TDataset) : Boolean; virtual;
    Procedure StartBatch(ResponseContent : TStream); override;
    Procedure NextBatchItem(ResponseContent : TStream); override;
    Procedure EndBatch(ResponseContent : TStream); override;
    Function CreateAdaptor(ARequest : TRequest) : TCustomWebdataInputAdaptor; override;
    Function AddFieldToJSON(O: TJSONObject; AFieldName: String; F: TField): TJSONData;
    function GetDataContentType: String; override;
    Function GetJSONMetaData: TJSONObject;
    function RowToJSON: TJSONObject;
    Procedure DoBeforeRow(ARow : TJSONObject); virtual;
    Procedure DoAfterRow(ARow : TJSONObject); virtual;
    Procedure DoBeforeData(AResponse : TJSONObject); virtual;
    Procedure DoAfterData(AResponse : TJSONObject); virtual;
    Procedure DoOnMetaData(AMetadata : TJSONObject); virtual;
    procedure DatasetToStream(Stream: TStream); override;
    Procedure DoExceptionToStream(E : Exception; ResponseContent : TStream); override;
    Procedure DoInsertRecord(ResponseContent : TStream); override;
    Procedure DoUpdateRecord(ResponseContent : TStream); override;
    Procedure DoDeleteRecord(ResponseContent : TStream); override;
  Public
    Destructor destroy; override;
  Published
    // Called before any fields are added to row object (passed to handler).
    Property AfterRowToJSON : TJSONObjectEvent Read FAfterRowToJSON Write FAfterRowToJSON;
    // Called After all fields are added to row object (passed to handler).
    Property BeforeRowToJSON : TJSONObjectEvent Read FBeforeRowToJSON Write FBeforeRowToJSON;
    // Called when metadata object has been created (passed to handler).
    Property OnMetaDataToJSON : TJSONObjectEvent Read FOnMetaDataToJSON Write FOnMetaDataToJSON;
    // Called when response object has been created, and has Rows property (response passed to handler).
    Property AfterDataToJSON : TJSONObjectEvent Read FAfterDataToJSON Write FAfterDataToJSON;
    // Called just before response object will be streamed (response passed to handler).
    Property BeforeDataToJSON : TJSONObjectEvent Read FBeforeDataToJSON Write FBeforeDataToJSON;
    // Called when an exception is caught and formatted.
    Property OnErrorResponse : TJSONExceptionObjectEvent Read FOnErrorResponse Write FOnErrorResponse;
    // Called to decide whether a record is sent to the client;
    Property OnAllowRow : TJSONObjectAllowRowEvent Read FOnAllowRow Write FOnAllowRow;
    // After a record was succesfully updated
    Property AfterUpdate : TJSONObjectEvent Read FAfterUpdate Write FAfterUpdate;
    // After a record was succesfully inserted.
    Property AfterInsert : TJSONObjectEvent Read FAfterInsert Write FAfterInsert;
    // After a record was succesfully inserted.
    Property AfterDelete : TJSONObjectEvent Read FAfterDelete Write FAfterDelete;
    // From TCustomHTTPDataContentProducer
    Property BeforeUpdate;
    Property BeforeInsert;
    Property BeforeDelete;
  end;

implementation
{ $define wmdebug}
{$ifdef wmdebug}
uses dbugintf;
{$endif wmdebug}

Resourcestring
  SErrWrongDataFormat = 'Post ROWS data has wrong value type. Expected array or object, got : %s.';
  SerrNoExceptionMessage = 'No exception to take error message from.';

Const
  // Do not localize these strings
  SDefMetaDataProperty  = 'metaData';
  SDefFieldsProperty    = 'fields';
  SDefFieldProperty     = 'field';
  SDefFieldNameProperty = 'name';
  SDefDirectionProperty = 'direction';
  SDefSortInfoProperty  = 'sortInfo';
  SIdProperty           = 'idProperty';
  SSuccessProperty      = 'successProperty';
  SRootProperty         = 'root';
  STotalProperty        = 'totalProperty';
  SDefAscDesc : Array[Boolean] of string = ('ASC','DESC');

function TExtJSJSONDataFormatter.GetDataContentType: String;
begin
  Result:='text/html';
end;

function TExtJSJSONDataFormatter.CreateAdaptor(ARequest: TRequest
  ): TCustomWebdataInputAdaptor;
begin
  Result:=TExtJSJSonWebdataInputAdaptor.Create(Self);
  Result.Request:=ARequest;
end;

function TExtJSJSONDataFormatter.AddFieldToJSON(O : TJSONObject; AFieldName : String; F : TField): TJSONData;

Var
  S : String;


begin
 if F.IsNull then
   Result:=O.Items[O.Add(AFieldName)]
 else
  Case F.DataType of
    ftSmallint,
    ftInteger,
    ftAutoInc,
    ftWord:
      Result:=O.Items[O.Add(AFieldName,F.AsInteger)];
    ftBoolean:
      Result:=O.Items[O.Add(AFieldName,F.AsBoolean)];
    ftLargeint:
      Result:=O.Items[O.Add(AFieldName,F.AsLargeInt)];
    ftDate:
      Result:=O.Items[O.Add(AFieldName,FormatDateTime('yyyy-mm-dd',F.AsDateTime))];
    ftDateTime:
      Result:=O.Items[O.Add(AFieldName,FormatDateTime('yyyy-mm-dd hh":"nn":"ss',F.AsDateTime))];
    ftTime:
      Result:=O.Items[O.Add(AFieldName,FormatDateTime('hh":"nn":"ss',F.AsDateTime))];
    ftMemo,
    ftFmtMemo,
    ftWideMemo,
    ftBlob :
      begin
      S:=F.AsString;
      If (OnTranscode<>Nil) then
        OnTranscode(Self,F,S,True);
      Result:=O.Items[O.Add(AFieldName,S)];
      end;
  else
    S:=F.DisplayText;
    If (OnTranscode<>Nil) then
      OnTranscode(Self,F,S,True);
    Result:=O.Items[O.Add(AFieldName,S)];
  end;
end;

function TExtJSJSONDataFormatter.RowToJSON: TJSONObject;

Var
  F : TField;
  I : Integer;


begin
  Result:=TJSONObject.Create();
  try
    DobeforeRow(Result);
    For I:=0 to Dataset.Fields.Count-1 do
      begin
      F:=Dataset.Fields[I];
      AddFieldToJSON(Result,F.FieldName,F);
      end;
    DoAfterRow(Result);
  except
    Result.Free;
    Raise;
  end;
end;

procedure TExtJSJSONDataFormatter.DoBeforeRow(ARow: TJSONObject);
begin
  If Assigned(FBeforeRowToJSON) then
    FBeforeRowToJSON(Self,ARow);
end;

procedure TExtJSJSONDataFormatter.DoAfterRow(ARow: TJSONObject);
begin
  If Assigned(FAfterRowToJSON) then
    FAfterRowToJSON(Self,ARow);
end;

procedure TExtJSJSONDataFormatter.DoBeforeData(AResponse: TJSONObject);
begin
  If Assigned(FBeforeDataToJSON) then
    FBeforeDataToJSON(Self,AResponse);
end;

procedure TExtJSJSONDataFormatter.DoAfterData(AResponse: TJSONObject);
begin
  If Assigned(FAfterDataToJSON) then
    FAfterDataToJSON(Self,AResponse);
end;

procedure TExtJSJSONDataFormatter.DoOnMetaData(AMetadata: TJSONObject);
begin
  If Assigned(FOnMetaDataToJSON) then
    FOnMetaDataToJSON(Self,AMetaData);
end;

Function TExtJSJSONDataFormatter.GetJSONMetaData: TJSONObject;

  Function DefReplace(S : String) : String;

  begin
    Result:=StringReplace(Result,'/',DateSeparator,[rfReplaceAll]);
    Result:=StringReplace(Result,':',TimeSeparator,[rfReplaceAll]);
    Result:=StringReplace(Result,'hh','H',[rfReplaceAll]);
    Result:=StringReplace(Result,'nn','i',[rfReplaceAll]);
    Result:=StringReplace(S,'n','i',[rfReplaceAll]);
  end;

Var
  F : TJSONArray;
  Fi : TField;
  I : Integer;
  O : TJSONObject;
  SF,FT : String;

begin
  If (SortField='') then
    SF:=Dataset.Fields[0].FieldName
  else
    SF:=SortField;
  Result:=TJSonObject.Create;
  try
    F:=TJSONArray.Create;
    Result.add(SDefFieldsProperty,F);
    For I:=0 to Dataset.Fields.Count-1 do
      begin
      Fi:=Dataset.Fields[i];
      O:=TJSONObject.Create();
      O.Add(SDefFieldNameProperty,Fi.FieldName);
      Ft:='';
      Case Fi.DataType of
        ftInteger,
        ftSmallint,
        ftWord,
        ftLargeInt : FT:='int';
        ftCurrency,
        ftFloat,
        ftBCD : FT:='float';
        ftBoolean : ft:='boolean';
        ftDate,
        ftDateTime,
        ftTimeStamp,
        ftTime : ft:='date';
        ftString,
        ftMemo,
        ftFmtMemo,
        ftFixedChar,
        ftWideString,
        ftWideMemo : ft:='string'
      end;
      if (FT<>'') then
        begin
        O.Add('type',FT);
        if (FT='date') then
          // Needs improving
          Case Fi.DataType of
            ftDate : O.Add('dateFormat','Y-m-d');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            ftTime : O.Add('dateFormat','H:i:s');
            ftDateTime,
            ftTimeStamp : O.Add('dateFormat','Y-m-d H:i:s');
=======
            ftTime : O.Add('dateFormat','h:i:s');
            ftDateTime,
            ftTimeStamp : O.Add('dateFormat','Y-m-d h:i:s');
>>>>>>> graemeg/cpstrnew
=======
            ftTime : O.Add('dateFormat','h:i:s');
            ftDateTime,
            ftTimeStamp : O.Add('dateFormat','Y-m-d h:i:s');
>>>>>>> graemeg/cpstrnew
=======
            ftTime : O.Add('dateFormat','h:i:s');
            ftDateTime,
            ftTimeStamp : O.Add('dateFormat','Y-m-d h:i:s');
>>>>>>> graemeg/cpstrnew
=======
            ftTime : O.Add('dateFormat','h:i:s');
            ftDateTime,
            ftTimeStamp : O.Add('dateFormat','Y-m-d h:i:s');
>>>>>>> origin/cpstrnew
          end;
        end;
      F.Add(O);
      end;
    O:=TJSONObject.Create();
    O.Add(SDefFieldProperty,SF);
    O.Add(SDefDirectionProperty,SDefAscDesc[SortDescending]);
    Result.Add(SDefSortInfoProperty,O);
 {$ifdef wmdebug}senddebug('ID property: '+Provider.IDFieldName);{$endif}
    Result.Add(SIdProperty,Provider.IDFieldName);
    Result.Add(SSuccessProperty, SuccessProperty);
    Result.Add(SRootProperty, RowsProperty);
    Result.Add(STotalProperty, totalProperty);
    DoOnMetaData(Result);
  except
    Result.free;
    Raise;
  end;
end;

procedure TExtJSJSONDataFormatter.DatasetToStream(Stream: TStream);

Var
  Rows : TJSONArray;
  Meta,Resp : TJSONObject;
  L : String;
  DS : TDataset;
  i,RCount,ACount : Integer;

begin
  Rows:=Nil;
  Resp:=TJSONObject.Create;
  try
    Rows:=TJSONArray.Create();
    Resp.Add(RowsProperty,Rows);
    DoBeforeData(Resp);
    DS:=Dataset;
    DS.First;
    RCount:=0;
    If MetaData then
      begin
      Meta:=GetJSONMetaData;
      Resp.Add(SDefMetaDataProperty,Meta);
      end;
    // Go to start
    ACount:=PageStart;
    While (Not DS.EOF) and (ACount>0) do
      begin
      DS.Next;
      Dec(ACount);
      Inc(RCount);
      end;
    ACount:=PageSize;
    While (not DS.EOF) and ((PageSize=0) or (ACount>0)) do
      begin
      If AllowRow(DS) then
        begin
        Inc(RCount);
        Dec(ACount);
        Rows.Add(RowToJSON);
        end;
      DS.Next;
      end;
    If (PageSize>0) then
      While (not DS.EOF) do
        begin
        Inc(RCount);
        DS.Next;
        end;
    Resp.Add(SuccessProperty,True);
    If (PageSize>0) then
       Resp.Add(TotalProperty,RCount);
    DoAfterData(Resp);
    L:=Resp.AsJSON;
    Stream.WriteBuffer(L[1],Length(L));
  finally
    Resp.Free;
  end;
end;

procedure TExtJSJSONDataFormatter.DoExceptionToStream(E: Exception;
  ResponseContent: TStream);

Var
   Resp : TJSonObject;
   L : String;

begin
  Resp:=tjsonObject.Create();
  try
    Resp.Add(SuccessProperty,False);
    If Assigned(E) then
      Resp.Add(MessageProperty,E.Message)
    else
      Resp.Add(MessageProperty,SerrNoExceptionMessage);
    L:=Resp.AsJSON;
    If Length(L)>0 then
      ResponseContent.WriteBuffer(L[1],Length(L));
    Resp.Add('root',RowsProperty);
    Resp.Add(RowsProperty,TJSONArray.Create());
    If Assigned(FOnErrorResponse) then
      FOnErrorResponse(Self,E,Resp);
  finally
    Resp.Free;
  end;
end;

procedure TExtJSJSONDataFormatter.SendSuccess(ResponseContent: TStream; AddIDValue : Boolean = False);

Var
   Resp : TJSonObject;
   L : String;

begin
  try
    Resp:=TJsonObject.Create;
    Resp.Add(SuccessProperty,True);
    Resp.Add('root',Self.RowsProperty);
    If Assigned(FBatchResult) and (FBatchResult.Count>0) then
      begin
      Resp.Add(Self.RowsProperty,FBatchResult);
      FBatchResult:=Nil;
      end
    else
      Resp.Add(Self.RowsProperty,TJSONNull.Create());
    L:=Resp.AsJSON;
    ResponseContent.WriteBuffer(L[1],Length(L));
  finally
    Resp.Free;
  end;
end;

function TExtJSJSONDataFormatter.AllowRow(ADataset: TDataset): Boolean;
begin
  Result:=True;
  If Assigned(FOnAllowRow) then
    FOnAllowRow(Self,Dataset,Result);
end;

procedure TExtJSJSONDataFormatter.StartBatch(ResponseContent: TStream);
begin
  If Assigned(FBatchResult) then
    FBatchResult.Clear
  else
    FBatchResult:=TJSONArray.Create();
end;

procedure TExtJSJSONDataFormatter.NextBatchItem(ResponseContent: TStream);
begin
end;

procedure TExtJSJSONDataFormatter.EndBatch(ResponseContent: TStream);
begin
  SendSuccess(Responsecontent,True);
end;

Function TExtJSJSONDataFormatter.AddIdToBatch : TJSONObject;

begin
  Result:=TJSONObject.Create([Provider.IDFieldName,Provider.IDFieldValue]);
  FBatchResult.Add(Result);
end;

procedure TExtJSJSONDataFormatter.DoInsertRecord(ResponseContent: TStream);

Var
  D : TJSONObject;

begin
  Inherited;
  D:=AddIDToBatch;
  If Assigned(FAfterInsert) then
    FAfterInsert(Self,D);
end;

procedure TExtJSJSONDataFormatter.DoUpdateRecord(ResponseContent: TStream);

Var
  D : TJSONObject;

begin
  inherited DoUpdateRecord(ResponseContent);
  D:=AddIDToBatch;
  If Assigned(FAfterUpdate) then
    FAfterUpdate(Self,D);
end;

procedure TExtJSJSONDataFormatter.DoDeleteRecord(ResponseContent: TStream);
begin
  inherited DoDeleteRecord(ResponseContent);
  If Assigned(FAfterDelete) then
    FAfterDelete(Self,Nil);
end;

destructor TExtJSJSONDataFormatter.destroy;
begin
  FreeAndNil(FBatchResult);
  inherited destroy;
end;

{ TExtJSJSonWebdataInputAdaptor }

function TExtJSJSonWebdataInputAdaptor.CheckData : Boolean;

Var
  D : TJSONData;
  P : TJSONParser;
  S : String;

begin
  Result:=Assigned(FCurrentRow);
  If Not (Result) and TryParamValue('rows',S) then
    begin
    {$ifdef wmdebug}senddebug('Check data: '+GetParamValue('rows'));{$endif}
    P:=TJSONParser.Create(S);
    try
      D:=P.Parse;
      {$ifdef wmdebug}senddebug('Classname : '+D.ClassName);{$endif}
      If D is TJSONArray then
        begin
        FRows:=TJSONArray(D);
        FRowIndex:=0;
        FCurrentRow:=FRows.Items[0] as TJSONObject;
        end
      else If D is TJSONObject then
        begin
        FRows:=Nil;
        FCurrentRow:=TJSONObject(D);
        end
      else if D is TJSONInt64Number then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        begin
        FRows:=nil;
        FCurrentRow:=TJSONObject.Create(['ID',D]);
        end
=======
        FIDValue:=D
>>>>>>> graemeg/cpstrnew
=======
        FIDValue:=D
>>>>>>> graemeg/cpstrnew
=======
        FIDValue:=D
>>>>>>> graemeg/cpstrnew
=======
        FIDValue:=D
>>>>>>> origin/cpstrnew
      else
        begin
        FreeAndNil(D);
        Raise EFPHTTPError.CreateFmt(SErrWrongDataFormat,[D.ClassName]);
        end;
      Result:=True;
    finally
      P.Free;
    end;
    end;
end;

procedure TExtJSJSonWebdataInputAdaptor.reset;
begin
  If (FRows=Nil) then
    FreeAndNil(FCurrentRow)
  else
    FreeAndNil(FRows);
  FRowIndex:=0;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  FreeAndNil(FIDValue);
>>>>>>> graemeg/cpstrnew
=======
  FreeAndNil(FIDValue);
>>>>>>> graemeg/cpstrnew
=======
  FreeAndNil(FIDValue);
>>>>>>> graemeg/cpstrnew
=======
  FreeAndNil(FIDValue);
>>>>>>> origin/cpstrnew
  inherited reset;
end;

function TExtJSJSonWebdataInputAdaptor.GetNextBatch: Boolean;
begin
  If (FRows=Nil) then
    Result:=inherited GetNextBatch
  else
    begin
    Result:=FRowindex<FRows.Count-1;
    Inc(FRowIndex);
    If Result then
      FCurrentRow:=FRows.Items[FRowIndex] as TJSONObject
    else
      FCurrentRow:=Nil;
    end;
end;

function TExtJSJSonWebdataInputAdaptor.TryFieldValue(const AFieldName: String;
  out AValue: String): Boolean;

Var
  I : Integer;

begin
  Result:=False;
  if CheckData then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    I:=FCurrentRow.IndexOfName(AFieldName);
    Result:=I<>-1;
    if result and (FCurrentRow.Items[I].JSONType<>jtNull) then
      AValue:=FCurrentRow.Items[I].AsString;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    If Assigned(FIDValue) and (0=CompareText(AFieldName,'ID')) then
      begin
      AValue:=FIDValue.AsString;
      Result:=True;
      end
    else
      begin
      I:=FCurrentRow.IndexOfName(AFieldName);
      Result:=I<>-1;
      if result then
        AValue:=FCurrentRow.Items[I].AsString;
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
    end;
end;

destructor TExtJSJSonWebdataInputAdaptor.destroy;
begin
  If Assigned(FRows) then
    FreeAndNil(FRows)
  else if assigned(FCurrentRow) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FreeAndNil(FCurrentRow);
=======
    FreeAndNil(FCurrentRow)
  else if Assigned(FIDValue) then
    FreeAndNil(FIDValue);
>>>>>>> graemeg/cpstrnew
=======
    FreeAndNil(FCurrentRow)
  else if Assigned(FIDValue) then
    FreeAndNil(FIDValue);
>>>>>>> graemeg/cpstrnew
=======
    FreeAndNil(FCurrentRow)
  else if Assigned(FIDValue) then
    FreeAndNil(FIDValue);
>>>>>>> graemeg/cpstrnew
=======
    FreeAndNil(FCurrentRow)
  else if Assigned(FIDValue) then
    FreeAndNil(FIDValue);
>>>>>>> origin/cpstrnew
  inherited destroy;
end;

initialization
  WebDataProviderManager.RegisterInputAdaptor('ExtJS - JSON',TExtJSJSONWebdataInputAdaptor);
  WebDataProviderManager.RegisterDataProducer('ExtJS - JSON',TExtJSJSONDataFormatter);

finalization
  WebDataProviderManager.UnRegisterInputAdaptor('ExtJS - JSON');
  WebDataProviderManager.UnRegisterDataProducer('ExtJS - JSON')
end.

