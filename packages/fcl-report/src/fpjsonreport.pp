{
    This file is part of the Free Component Library.
    Copyright (c) 2017 Michael Van Canneyt, member of the Free Pascal development team

    TFPReport descendent that stores it's design in a JSON structure. 
    Can be used in an IDE

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit fpjsonreport;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpreport, fpjson, fpreportstreamer, fpreportdata;

Type

  { TFPJSONReport }
  TReadReportJSONEvent = Procedure(Sender : TObject; JSON : TJSONObject) of object;
  TWriteReportJSONEvent = Procedure(Sender : TObject; JSON : TJSONObject) of object;

  TFPJSONReport = class(TFPReport)
  private
    FDataManager: TFPCustomReportDataManager;
    FDesignTimeJSON: TJSONObject;
    FLoadErrors: TStrings;
    FOnReadJSON: TReadReportJSONEvent;
    FOnWriteJSON: TWriteReportJSONEvent;
    FDesignDataName : String;
    function GetDesignDataName: String;
    procedure ReadReportJSON(Reader: TReader);
    procedure SetDataManager(AValue: TFPCustomReportDataManager);
    procedure SetDesignDataName(AValue: String);
    function StoreDesignDataName: Boolean;
    procedure WriteReportJSON(Writer: TWriter);
  Protected
    procedure DoReadJSON(aJSON: TJSONObject);virtual;
    procedure DoWriteJSON(aJSON: TJSONObject);virtual;
    Procedure DefineProperties(Filer: TFiler); override;
    Procedure Notification(AComponent: TComponent; Operation: TOperation); override;
  Public
    Constructor Create(AOwner : TComponent); override;
    destructor Destroy; override;
    procedure LoadFromStream(const aStream: TStream);
    procedure SaveToStream(const aStream: TStream);
    procedure SaveRenderToStream(const aStream: TStream);
    Procedure LoadFromJSON(aJSON : TJSONObject); virtual;
    Procedure SavetoJSON(aJSON : TJSONObject); virtual;
    Procedure SaveRenderToJSON(aJSON : TJSONObject); virtual;
    Procedure LoadFromFile(const aFileName : String);
    Procedure SaveToFile(const aFileName : String);
    procedure SaveRenderToFile(const aFileName: String);
    Property LoadErrors : TStrings Read FLoadErrors;
    Property DataManager : TFPCustomReportDataManager Read FDataManager Write SetDataManager;
    Property DesignDataName : String Read GetDesignDataName Write SetDesignDataName Stored StoreDesignDataName;
    Property DesignTimeJSON : TJSONObject Read FDesignTimeJSON;
    Property OnReadJSON : TReadReportJSONEvent Read FOnReadJSON Write FOnReadJSON;
    Property OnWriteJSON : TWriteReportJSONEvent Read FOnWriteJSON Write FOnWriteJSON;
  end;

implementation

Const
  DefaultDesignData = 'DesignData';

Resourcestring
  SErrInvalidJSONData = 'Invalid JSON Data';
  SErrFailedToLoad = 'Failed to load report: %s';

{ TFPJSONReport }

procedure TFPJSONReport.ReadReportJSON(Reader: TReader);

Var
  S : UnicodeString;
  D : TJSONData;

begin
  FDesignTimeJSON.Clear;
  S:=Reader.ReadUnicodeString;
  if (S<>'') then
    begin
    D:=GetJSON(UTF8Encode(S),True);
    if D is TJSONObject then
      begin
      FreeAndNil(FDesignTimeJSON);
      FDesignTimeJSON:=D as TJSONObject
      end
    else
      begin
      D.Free;
      FDesignTimeJSON:=TJSONObject.Create;
      Raise EReportError.CreateFmt(SErrFailedToLoad,[SErrInvalidJSONData]);
      end;
    end;
end;

procedure TFPJSONReport.SetDataManager(AValue: TFPCustomReportDataManager);
begin
  if FDataManager=AValue then Exit;
  If Assigned(FDataManager) then
    FDataManager.RemoveFreeNotification(Self);
  FDataManager:=AValue;
  If Assigned(FDataManager) then
    FDataManager.FreeNotification(Self);
end;

procedure TFPJSONReport.SetDesignDataName(AValue: String);
begin
  if AValue=GetDesignDataName then exit;
  FDesignDataName:=aValue;
end;

function TFPJSONReport.StoreDesignDataName: Boolean;
begin
  Result:=GetDesignDataName<>DefaultDesignData;
end;

procedure TFPJSONReport.WriteReportJSON(Writer: TWriter);

Var
  S : UnicodeString;

begin
  S:='';
  if (FDesignTimeJSON.Count>0) then
    S:=UTF8Decode(FDesignTimeJSON.AsJSON);
  Writer.WriteUnicodeString(S);
end;

procedure TFPJSONReport.DefineProperties(Filer: TFiler);
begin
  inherited DefineProperties(Filer);
  Filer.DefineProperty('ReportJSON',@ReadReportJSON,@WriteReportJSON,Assigned(FDesignTimeJSON) and (FDesignTimeJSON.Count>0));
end;

procedure TFPJSONReport.Notification(AComponent: TComponent; Operation: TOperation);
begin
  inherited Notification(AComponent, Operation);
  if (Operation=opRemove) and (AComponent=FDataManager) then
    FDataManager:=Nil;
end;

constructor TFPJSONReport.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FDesignTimeJSON:=TJSONObject.Create;
  FLoadErrors:=TStringList.Create;
end;

destructor TFPJSONReport.Destroy;
begin
  FreeAndNil(FLoadErrors);
  FreeAndNil(FDesignTimeJSON);
  inherited Destroy;
end;

Function TFPJSONReport.GetDesignDataName : String;

begin
  Result:=FDesignDataName;
  if (FDesignDataName='') then
    Result:=DefaultDesignData;
end;

procedure TFPJSONReport.DoReadJSON(aJSON: TJSONObject);

Var
  O : TJSONObject;

begin
  FloadErrors.Clear;
  if Assigned(FOnReadJSON) then
    FOnReadJSON(Self,aJSON);
  if Assigned(FDataManager) then
    begin
    O:=aJSON.get(GetDesignDataName,TJSONObject(Nil));
    if Assigned(O) then
      begin
      FDataManager.LoadFromJSON(O);
      FDataManager.ApplyToReport(Self,LoadErrors);
      end;
    end;
end;

procedure TFPJSONReport.LoadFromJSON(aJSON: TJSONObject);

Var
  R : TFPReportJSONStreamer;
  N : String;

begin
  N:=Name;
  DoReadJSON(aJSON);
  R:=TFPReportJSONStreamer.Create(Nil);
  try
    R.OwnsJSON:=False;
    R.JSON:=aJSON;
    ReadElement(R);
  finally
    Name:=N;
    R.Free;
  end;
end;

procedure TFPJSONReport.DoWriteJSON(aJSON: TJSONObject);

Var
   O: TJSONObject;

begin
  if Assigned(FDataManager) then
    begin
    O:=TJSONObject.Create();
    aJSON.Add(GetDesignDataName,O);
    FDataManager.SaveToJSON(O);
    end;
  if Assigned(FOnWriteJSON) then
    FOnWriteJSON(Self,aJSON);
end;

procedure TFPJSONReport.SavetoJSON(aJSON: TJSONObject);

Var
  R : TFPReportJSONStreamer;

begin
  DoWriteJSON(aJSON);
  R:=TFPReportJSONStreamer.Create(Nil);
  try
    R.OwnsJSON:=False;
    R.JSON:=aJSON;
    WriteElement(R);
  finally
    R.Free;
  end;
end;

procedure TFPJSONReport.SaveRenderToJSON(aJSON: TJSONObject);

Var
  R : TFPReportJSONStreamer;

begin
  DoWriteJSON(aJSON);
  R:=TFPReportJSONStreamer.Create(Nil);
  try
    R.OwnsJSON:=False;
    R.JSON:=aJSON;
    WriteRTElement(R);
  finally
    R.Free;
  end;
end;

procedure TFPJSONReport.LoadFromStream(const aStream : TStream);

Var
  D : TJSONData;

begin
  D:=GetJSON(aStream);
  try
    if not (D is TJSONObject) then
      Raise EReportError.CreateFmt(SErrFailedToLoad,[SErrInvalidJSONData]);
    LoadFromJSON(D as TJSONObject);
  finally
    D.Free;
  end;
end;

procedure TFPJSONReport.SaveToStream(const aStream: TStream);

Var
  O : TJSONObject;
  S : TJSONStringType;

begin
  O:=TJSONObject.Create;
  try
    SaveToJSON(O);
    S:=O.AsJSON;
    aStream.WriteBuffer(S[1],Length(S));
  finally
    O.Free;
  end;
end;

procedure TFPJSONReport.SaveRenderToStream(const aStream: TStream);

Var
  O : TJSONObject;
  S : TJSONStringType;

begin
  O:=TJSONObject.Create;
  try
    SaveRendertoJSON(O);
    S:=O.AsJSON;
    aStream.WriteBuffer(S[1],Length(S));
  finally
    O.Free;
  end;
end;

procedure TFPJSONReport.LoadFromFile(const aFileName: String);

Var
  F : TFileStream;

begin
  F:=TFileStream.Create(aFileName,fmOpenRead or fmShareDenyWrite);
  try
    LoadFromStream(F);
  finally
    F.Free;
  end;
end;

procedure TFPJSONReport.SaveToFile(const aFileName: String);
Var
  F : TFileStream;

begin
  F:=TFileStream.Create(aFileName,fmCreate);
  try
    SaveToStream(F);
  finally
    F.Free;
  end;
end;

procedure TFPJSONReport.SaveRenderToFile(const aFileName: String);
Var
  F : TFileStream;

begin
  F:=TFileStream.Create(aFileName,fmCreate);
  try
    SaveRenderToStream(F);
  finally
    F.Free;
  end;
end;

end.

