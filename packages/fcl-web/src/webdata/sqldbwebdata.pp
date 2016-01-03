unit sqldbwebdata;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fphttp, fpwebdata, DB, SQLDB;


Type
  { TCustomSQLDBWebDataProvider }
  TNewIDEvent = Procedure(Sender : TObject; Out AID : String) of object;
  TGetParamTypeEvent = Procedure (Sender : TObject; Const ParamName,AValue : String; Var AType : TFieldtype) of object;
  TGetParamValueEvent = Procedure (Sender : TObject; P : TParam; Var Handled : Boolean) of object;

  TCustomSQLDBWebDataProvider = Class(TFPCustomWebDataProvider)
  private
    FIDFieldName: String;
    FONGetDataset: TNotifyEvent;
    FOnGetNewID: TNewIDEvent;
    FOnGetParamValue: TGetParamValueEvent;
    FParams: TParams;
    FSQLS : Array[0..3] of TStringList;
    FConnection: TSQLConnection;
    FQuery : TSQLQuery;
    FLastNewID : String;
    FOnGetParamType : TGetParamTypeEvent;
    function GetS(AIndex: integer): TStrings;
    procedure RegenerateParams;
    procedure SetConnection(const AValue: TSQLConnection);
    procedure SetParams(const AValue: TParams);
    procedure SetS(AIndex: integer; const AValue: TStrings);
  Protected
    function CheckDataset : Boolean; virtual;
    function CreateQuery(AOwner: TComponent; ATransaction: TSQLTransaction; ASQL: Tstrings): TSQLQuery;
    function GetParamType(P: TParam; const AValue: String): TFieldType; virtual;
    procedure SetTypedParam(P: TParam; Const AValue: String); virtual;
    procedure ExecuteSQL(ASQL: TStrings; Msg: String=''; DoNewID : Boolean = False); virtual;
    procedure ApplySQLParams(AQuery: TSQLQuery; DoNewID : Boolean = False); virtual;
    Procedure SQLChanged(Sender : TObject); virtual;
    Procedure DoUpdate; override;
    Procedure DoDelete; override;
    Procedure DoInsert; override;
    Procedure DoApplyParams; override;
    Function SQLQuery : TSQLQuery;
    Function GetDataset : TDataset; override;
    Function DoGetNewID : String; virtual;
    Function GetNewID : String;
    Function IDFieldValue : String; override;
    procedure Notification(AComponent: TComponent;  Operation: TOperation); override;
    Property SelectSQL : TStrings Index 0 Read GetS Write SetS;
    Property UpdateSQL : TStrings Index 1 Read GetS Write SetS;
    Property DeleteSQL : TStrings Index 2 Read GetS Write SetS;
    Property InsertSQL : TStrings Index 3 Read GetS Write SetS;
    Property Connection : TSQLConnection Read FConnection Write SetConnection;
    Property OnGetNewID : TNewIDEvent Read FOnGetNewID Write FOnGetNewID;
    property OnGetParameterType : TGetParamTypeEvent Read FOnGetParamType Write FOnGetParamType;
    property OnGetParameterValue : TGetParamValueEvent Read FOnGetParamValue Write FOnGetParamValue;
    Property OnGetDataset : TNotifyEvent Read FONGetDataset Write FOnGetDataset;
    Property Params : TParams Read FParams Write SetParams;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
  end;

  TSQLDBWebDataProvider = Class(TCustomSQLDBWebDataProvider)
  Published
    Property SelectSQL;
    Property UpdateSQL;
    Property DeleteSQL;
    Property InsertSQL;
    Property Connection;
    Property IDFieldName;
    Property OnGetNewID;
    property OnGetParameterType;
    property OnGetParameterValue;
    Property OnGetDataset;
    Property Options;
    Property Params;
  end;

implementation

{ $define wmdebug}

{$ifdef wmdebug}
uses dbugintf;
{$endif}

resourcestring
  SErrNoSelectSQL = '%s: No select SQL statement provided.';
  SErrNoUpdateSQL = '%s: No update SQL statement provided.';
  SErrNoInsertSQL = '%s: No insert SQL statement provided.';
  SErrNoDeleteSQL = '%s: No delete SQL statement provided.';
  SErrUpdating = '%s: An error occurred during the update operation: %s';
  SErrDeleting = '%s: An error occurred during the delete operation: %s';
  SErrInserting = '%s: An error occurred during the insert operation: %s';
  SErrNoNewIDEvent = '%s : Cannot generate ID: No OnGetNewID event assigned.';

{ TCustomSQLDBWebDataProvider }

function TCustomSQLDBWebDataProvider.GetS(AIndex: integer): TStrings;
begin
  Result:=FSQLS[AIndex];
end;

procedure TCustomSQLDBWebDataProvider.SetConnection(const AValue: TSQLConnection
  );
begin
  if (FConnection=AValue) then exit;
  If Assigned(FConnection) then
    FConnection.RemoveFreeNotification(Self);
  FConnection:=AValue;
  If Assigned(FConnection) then
    FConnection.FreeNotification(Self);
end;

procedure TCustomSQLDBWebDataProvider.SetParams(const AValue: TParams);
begin
  if FParams=AValue then exit;
  FParams.Assign(AValue);
end;

procedure TCustomSQLDBWebDataProvider.SetS(AIndex: integer;
  const AValue: TStrings);
begin
  FSQLS[AIndex].Assign(AValue);
end;

procedure TCustomSQLDBWebDataProvider.SQLChanged(Sender: TObject);
begin
  If (Sender=SelectSQL)  then
    begin
    if Assigned(FQuery) then
      begin
      FQuery.Close;
      FQuery.SQL.Assign(SelectSQL);
      end;
    If Not (csLoading in ComponentState) then
      RegenerateParams;
    end;
end;

procedure TCustomSQLDBWebDataProvider.RegenerateParams;

Var
  S : String;

begin
  S:=SelectSQL.Text;
  Params.Clear;
  Params.ParseSQL(S,True);
end;

procedure TCustomSQLDBWebDataProvider.ExecuteSQL(ASQL : TStrings; Msg : String = ''; DoNewID : Boolean = False);

Var
  Q : TSQLQuery;
begin
  {$ifdef wmdebug}SendDebug('Entering TCustomSQLDBWebDataProvider.ExecuteSQL');{$endif}
  Q:=CreateQuery(Nil,Nil,ASQL);
  try
    Q.Transaction.Active:=True;
    try
      ApplySQLParams(Q,DoNewID);
      Q.ExecSQL;
      (Q.Transaction as TSQLTransaction).Commit;
    except
      On E : Exception do
        begin
        (Q.Transaction as TSQLTransaction).Rollback;
        If (Msg<>'') then
          E.Message:=Format(Msg,[Self.Name,E.Message]);
        Raise;
        end;
    end
  finally
    Q.Free;
  end;
  {$ifdef wmdebug}SendDebug('Exiting TCustomSQLDBWebDataProvider.ExecuteSQL');{$endif}
end;

procedure TCustomSQLDBWebDataProvider.DoUpdate;

begin
{$ifdef wmdebug}SendDebug('Entering TCustomSQLDBWebDataProvider.DoUpdate');{$endif}
  If (Trim(UpdateSQL.Text)='') then
    Raise EFPHTTPError.CreateFmt(SErrNoUpdateSQL,[Self.Name]);
  FLastNewID:='';
  ExecuteSQL(UpdateSQL,SErrUpdating);
{$ifdef wmdebug}SendDebug('Exiting TCustomSQLDBWebDataProvider.DoUpdate');{$endif}
end;

procedure TCustomSQLDBWebDataProvider.DoDelete;
begin
{$ifdef wmdebug}SendDebug('Entering TCustomSQLDBWebDataProvider.DoDelete');{$endif}
  If (Trim(DeleteSQL.Text)='') then
    Raise EFPHTTPError.CreateFmt(SErrNoDeleteSQL,[Self.Name]);
  FLastNewID:='';
  ExecuteSQL(DeleteSQL,SErrDeleting);
{$ifdef wmdebug}SendDebug('Exiting TCustomSQLDBWebDataProvider.DoDelete');{$endif}
end;

procedure TCustomSQLDBWebDataProvider.DoInsert;
begin
{$ifdef wmdebug}SendDebug('Entering TCustomSQLDBWebDataProvider.DoInsert');{$endif}
  If (Trim(InsertSQL.Text)='') then
    Raise EFPHTTPError.CreateFmt(SErrNoInsertSQL,[Self.Name]);
  FLastNewID:='';
  ExecuteSQL(InsertSQL,SErrInserting,(IDFieldName<>''));
{$ifdef wmdebug}SendDebug('Exiting TCustomSQLDBWebDataProvider.DoInsert');{$endif}
end;

procedure TCustomSQLDBWebDataProvider.Notification(AComponent: TComponent;
  Operation: TOperation);

begin
  inherited;
  If (Operation=opRemove) then
    begin
    If (AComponent=FQuery) then
      FQuery:=Nil
    else if (AComponent=FConnection) then
      FConnection:=Nil;
    end;
end;

Function TCustomSQLDBWebDataProvider.CreateQuery(AOwner : TComponent; ATransaction : TSQLTransaction; ASQL : Tstrings) : TSQLQuery;

begin
  Result:=TSQLQuery.Create(AOwner);
  If (AOwner<>Self) then
    Result.FreeNotification(Self);
  Result.DataBase:=Connection;
  If ATransaction=Nil then
    begin
    ATransaction:=TSQLTransaction.Create(Result);
    ATransaction.DataBase:=Connection;
    end;
  Result.Transaction:=ATransaction;
  Result.SQL.Assign(ASQL);
end;

Function TCustomSQLDBWebDataProvider.CheckDataset : boolean;

begin
{$ifdef wmdebug}SendDebug('Entering CheckDataset');{$endif}
  If (Trim(SelectSQL.Text)='') then
    Raise EFPHTTPError.CreateFmt(SErrNoSelectSQL,[Self.Name]);
  Result:=FQuery=Nil;
  If (Result) then
    FQuery:=CreateQuery(Nil,Nil,SelectSQL)
  else if not FQuery.Active then
    FQuery.SQL.Assign(SelectSQL);
{$ifdef wmdebug}SendDebug('Exiting CheckDataset');{$endif}
end;

Function TCustomSQLDBWebDataProvider.GetParamType(P : TParam; Const AValue : String) : TFieldType;

begin
  Result:=ftunknown;
  If Assigned(FOnGetParamType) then
    FOnGetParamType(Self,P.Name,AValue,Result);
end;

procedure TCustomSQLDBWebDataProvider.SetTypedParam(P : TParam; Const AValue : String);

Var
  I : Integer;
  Q : Int64;
  D : TDateTime;
  ft : TFieldType;
  F : Double;
  B : Boolean;
  C : Currency;

begin
  ft:=GetParamtype(P,AValue);
  If (AValue='') and (not (ft in [ftString,ftFixedChar,ftWideString,ftFixedWideChar])) then
    begin
    P.Clear;
    exit;
    end;
  If (ft<>ftUnknown) then
    begin
    try
      case ft of
        ftInteger,
        ftword,
        ftsmallint  : I:=StrToInt(AValue);
        ftDate      : D:=StrToDate(AValue);
        ftDateTime,
        ftTimestamp : D:=StrToDateTime(AValue);
        ftBoolean   : B:=StrToBool(AValue);
        ftTime      : D:=StrToTime(AValue);
        ftLargeint  : Q:=StrToInt64(AValue);
        ftCurrency  : C:=StrToCurr(Avalue);
      else
        ft:=ftString
      end
    except
      ft:=ftUnknown
    end;
    end;
  If (ft=ftUnknown) and (Length(AValue)<30) then
    begin
    if TryStrToInt(Avalue,I) then
      ft:=ftInteger
    else if TryStrToInt64(Avalue,Q) then
      ft:=ftInteger
    else if (Pos(DateSeparator,AValue)<>0) then
        begin
        if (Pos(TimeSeparator,AValue)<>0) and TryStrToDateTime(Avalue,D) then
          ft:=ftDateTime
        else if TryStrToDate(Avalue,D) then
          ft:=ftDate
        end
    else If (Pos(TimeSeparator,AValue)<>0) and TryStrToTime(Avalue,D) then
      ft:=ftTime
    else if (Pos(DecimalSeparator,AValue)<>0) then
      begin
      if trystrtofloat(AValue,F) then
        ft:=ftFloat
      else if TryStrToCurr(Avalue,C) then
        ft:=ftCurrency
      end
    else if TryStrToBool(Avalue,B) then
      ft:=ftBoolean
    end;
  Case ft of
    ftInteger,
    ftword,
    ftsmallint  : P.AsInteger:=I;
    ftBoolean   : P.AsBoolean:=B;
    ftLargeInt  : P.AsLargeInt:=Q;
    ftDate      : P.AsDate:=D;
    ftDateTime,
    ftTimestamp : P.AsDateTime:=D;
    ftTime      : P.AsTime:=D;
    ftFloat,
    ftBCD,
    ftFMTBCD    : P.AsFloat:=F;
    ftCurrency  : P.AsCurrency:=F;
  else
    P.AsString:=AValue;
  end;
end;

procedure TCustomSQLDBWebDataProvider.ApplySQLParams(AQuery : TSQLQuery; DoNewID : Boolean = False);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  function TryAdaptor (const aName: string; P: TParam) : boolean;
  var S : string;
  begin
    result := Adaptor.TryFieldValue(aName,S);
    if not result then
      result := Adaptor.TryParamValue(aName,S);
    if result then
      SetTypedParam(P,S);
  end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
var
  I: Integer;
  P : TParam;
  S : String;
  B : Boolean;


begin
{$ifdef wmdebug}SendDebug('Entering ApplySQLPArams');{$endif}
  For I:=0 to AQuery.Params.Count-1 do
    begin
    P:=AQuery.Params[i];
    B:=Assigned(FOnGetParamValue);
    if B then
      FOnGetParamValue(Self,P,B);
    if not B then
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      If (P.Name=IDFieldName) then
        begin
        if DoNewID then
          begin
          GetNewID;
          SetTypedParam(P,FLastNewID)
          end
        else
          begin
          if not TryAdaptor (P.Name, P) then
            TryAdaptor('ID', P);
          end;
        end
      else if not TryAdaptor (P.Name, P) then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      If (P.Name=IDFieldName) and DoNewID then
        begin
        GetNewID;
        SetTypedParam(P,FLastNewID)
        end
      else If Adaptor.TryFieldValue(P.Name,S) then
        SetTypedParam(P,S)
      else If Adaptor.TryParamValue(P.Name,S) then
        SetTypedParam(P,S)
      else
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
        P.Clear;
      end;
    end;
{$ifdef wmdebug}SendDebug('Exiting ApplySQLPArams');{$endif}
end;

procedure TCustomSQLDBWebDataProvider.DoApplyParams;

begin
  CheckDataset;
  ApplySQLParams(FQuery);
end;

function TCustomSQLDBWebDataProvider.SQLQuery: TSQLQuery;
begin
  Result:=FQuery;
end;

function TCustomSQLDBWebDataProvider.GetDataset: TDataset;
begin
{$ifdef wmdebug}SendDebug('Get dataset: checking dataset');{$endif}
  If Assigned(FonGetDataset) then
    FOnGetDataset(Self);
  CheckDataset;
  FLastNewID:='';
  Result:=FQuery;
{$ifdef wmdebug}SendDebug('Get dataset: activating transaction');{$endif}
  If Not FQuery.Transaction.Active then
    FQuery.Transaction.Active:=True;
{$ifdef wmdebug}SendDebug('Get dataset: done');{$endif}
end;

function TCustomSQLDBWebDataProvider.DoGetNewID: String;
begin
  If Not Assigned(FOnGetNewID) then
    Raise EFPHTTPError.CreateFmt(SErrNoNewIDEvent,[Self.Name]);
  FOnGetNewID(Self,Result);
end;

function TCustomSQLDBWebDataProvider.GetNewID: String;

begin
  Result:=DoGetNewID;
  FLastNewID:=Result;
end;

function TCustomSQLDBWebDataProvider.IDFieldValue: String;

begin
{$ifdef wmdebug}SendDebug('Entering IDFieldValue');{$endif}
  If (FLastNewID<>'') then
    Result:=FLastNewID
  else If (IDFieldName<>'') then
    begin
    If not Adaptor.TryParamValue(IDFieldName,Result) then
      If not Adaptor.TryFieldValue(IDFieldName,Result) then
        Result:=inherited IDFieldValue;
    end
  else
    Result:=inherited IDFieldValue;
{$ifdef wmdebug}SendDebug('Exiting IDFieldValue : '+Result);{$endif}
end;

constructor TCustomSQLDBWebDataProvider.Create(AOwner: TComponent);

Var
  I : Integer;
  L : TStringList;

begin
  inherited Create(AOwner);
  For I:=0 to 3 do
    begin
    L:=TStringList.Create;
    L.OnChange:=@SQLChanged;
    FSQLS[i]:=L;
    end;
  FParams:=TParams.Create(TParam);
end;

destructor TCustomSQLDBWebDataProvider.Destroy;

Var
  I: Integer;

begin
  For I:=0 to 3 do
   FreeAndNil(FSQLS[i]);
  Connection:=Nil;
  FreeAndNil(FQuery);
  FreeAndNil(FParams);
  inherited Destroy;
end;

end.

