{
    This file is part of the Free Component Library.
    Copyright (c) 2017 Michael Van Canneyt, member of the Free Pascal development team

    Report Data loop classes based on a JSON data structure.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit fpreportjson;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpreport, fpjson;

Type

  { TFPReportJSONData }

  TFPReportJSONData = class(TFPReportData)
  private
    FJSON : TJSONData;
    FOwnsJSON: Boolean;
    FPath: TJSONStringType;
    FRoot : TJSONData;
    FIndex : Integer;
    function GetJSON: TJSONStringType;
    procedure SetJSON(AValue: TJSONData);
    procedure SetJSON(AValue: TJSONStringType);
    procedure SetPath(AValue: TJSONStringType);
  protected
    procedure SetRoot;
    procedure DoGetValue(const AFieldName: string; var AValue: variant); override;
    procedure DoInitDataFields; override;
    procedure DoOpen; override;
    procedure DoFirst; override;
    procedure DoNext; override;
    procedure DoClose; override;
    function  DoEOF: boolean; override;
  Public
    Destructor Destroy; override;
    Procedure LoadFromStream(S : TStream); virtual;
    Procedure LoadFromFile(const aFileName : String);
    property  DataFields;
    property  JSONData : TJSONData read FJSON write SetJSON;
  Published
    Property Path : TJSONStringtype Read FPath Write SetPath;
    Property OwnsJSON : Boolean Read FOwnsJSON Write FOwnsJSON;
    Property JSON : TJSONStringType Read GetJSON Write SetJSON;
  end;


implementation

resourcestring
  SErrInvalidJSON = 'Invalid JSON. Need Array or Object';
  SErrInvalidJSONAtPath = 'Invalid JSON at Path. Need Array or Object';
  SErrInvalidPath = 'Path "%s" is not valid';

{$if FPC_FULLVERSION<=30000}
Const
  StructuredJSONTypes  = [jtArray,jtObject];
{$ENDIF}


{ TFPReportJSONData }

function TFPReportJSONData.GetJSON: TJSONStringType;
begin
  If Assigned(FJSON) then
    Result:=FJSON.AsJSON
  else
    Result:='';
end;

procedure TFPReportJSONData.SetRoot;


Var
  d : TJSONData;

begin
  D:=Nil;
  if Assigned(FJSON) then
    begin
    if (Path='') then
      D:=FJSON
    else
      begin
      D:=FJSON.FindPath(Path);
      if D=Nil then
        Raise EReportError.CreateFmt(SErrInvalidPath,[Path]);
      end
    end;
  if Assigned(D) and Not (D.JSONType in StructuredJSONTypes) then
    Raise EReportError.Create(SErrInvalidJSONAtPath);
  FRoot:=D;
end;

procedure TFPReportJSONData.SetJSON(AValue: TJSONData);
begin
  if FJSON=AValue then Exit;
  if Assigned(AValue) and Not (AValue.JSONType in StructuredJSONTypes) then
    Raise EReportError.Create(SErrInvalidJSON);
  if OwnsJSON then
    FreeAndNil(FJSON);
  FJSON:=AValue;
  SetRoot;
end;

procedure TFPReportJSONData.SetJSON(AValue: TJSONStringType);

Var
  aJSON : TJSONData;

begin
  if (AValue='') then
    JSON:=Nil
  else
    begin
    aJSON:=fpjson.GetJSON(aValue);
    try
      JSON:=aJSON;
      OwnsJSON:=True;
    except
      FreeAndNil(aJSON);
      Raise;
    end;
    end;
end;

procedure TFPReportJSONData.SetPath(AValue: TJSONStringType);

begin
  if FPath=AValue then Exit;
  FPath:=AValue;
  SetRoot;
end;

procedure TFPReportJSONData.DoGetValue(const AFieldName: string;
  var AValue: variant);

Var
  Rec : TJSONData;
  D : TJSONData;
  I : Integer;

begin
  inherited DoGetValue(AFieldName, AValue);
  if (Not Assigned(FRoot)) or (Findex>=FRoot.Count) then
    exit;
  I:=-1;
  Rec:=FRoot.Items[FIndex];
  if (Rec is TJSONObject) then
    I:=TJSONObject(Rec).IndexOfName(AFieldName,True)
  else if (Rec is TJSONArray) then
    begin
    I:=DataFields.IndexOfField(AFieldName);
    end;
  if (I=-1) then
    begin
    Writeln(FIndex,' : ',AFieldName,' -> ',I);
    Exit;
    end;
  D:=Rec.Items[i];
  Case D.JSONType of
  jtString :
  AValue:=D.AsString;
  jtNumber :
    Case TJSONNUmber(D).NumberType of
      ntFloat : AValue:=D.AsFloat;
      ntInteger : AValue:=D.AsInteger;
      ntInt64 : AValue:=D.AsInt64;
      {$if FPC_FULLVERSION>30000}
      ntQWord : AValue:=D.AsQWord;
      {$ENDIF}
    end;
  jtBoolean:
    AValue:=D.AsBoolean;
  jtNull :
    ;
  else
    AValue:=D.AsJSON;
  end;
  {$IFDEF gdebug}
  Writeln(FIndex,' : ',AFieldName,' -> ',AValue);
  {$ENDIF}
end;

procedure TFPReportJSONData.DoInitDataFields;

Var
  Rec : TJSONData;
  E : TJSONEnum;
  N : String;
  prefix : Boolean;
  k : TFPReportFieldKind;

begin
  inherited DoInitDataFields;
  if Assigned(FRoot) and (FRoot.Count>0) then
    begin
    Rec:=FRoot.Items[0];
    Prefix:=not (Rec is TJSONObject);
    for E in Rec do
      begin
      N:=E.Key;
      if Prefix then
        N:='Column'+N;
      k:=rfkString;
      Case E.Value.JSONType of
        jtBoolean :
          k:=rfkBoolean;
        jtNumber :
          if TJSONNumber(E.Value).NumberType=ntFloat then
            k:=rfkFloat
          else
            k:=rfkInteger;
      end;
      DataFields.AddField(N,k);
      end;
    end;
end;

procedure TFPReportJSONData.DoOpen;
begin
  inherited DoOpen;
  FIndex:=0;
end;

procedure TFPReportJSONData.DoFirst;
begin
  inherited DoFirst;
  FIndex:=0;
end;

procedure TFPReportJSONData.DoNext;
begin
  Inherited;
  Inc(FIndex);
end;

procedure TFPReportJSONData.DoClose;
begin
  inherited DoClose;
  DataFields.Clear;
  FIndex:=-1;
end;

function TFPReportJSONData.DoEOF: boolean;
begin
  Result:=Not Assigned(FRoot) or (FIndex>=FRoot.Count) or (FIndex<0);
end;

destructor TFPReportJSONData.Destroy;
begin
  if OwnsJSON then
    FreeAndNil(FJSON);
  inherited Destroy;
end;

procedure TFPReportJSONData.LoadFromStream(S: TStream);

Var
  aJSON : TJSONData;

begin
  try
    aJSON:=fpjson.GetJSON(S);
    JSON:=aJSON;
    OwnsJSON:=True;
  except
    FreeAndNil(aJSON);
    Raise;
  end
end;

procedure TFPReportJSONData.LoadFromFile(const aFileName: String);

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

end.

