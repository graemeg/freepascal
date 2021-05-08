{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2018  Mattias Gaertner  mattias@freepascal.org

    Pascal to Javascript converter class.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

  Abstract:
    Extends the FCL Pascal parser for the language subset of pas2js.
}
unit Pas2jsPParser;

{$mode objfpc}{$H+}

{$i pas2js_defines.inc}

interface

uses
  Classes, SysUtils, PParser, PScanner, PasTree, PasResolver, fppas2js,
  Pas2jsLogger;

const // Messages
  nFinalizationNotSupported = 3001;
  sFinalizationNotSupported = 'Finalization section is not supported.';

type

  { TPas2jsPasParser }

  TPas2jsPasParser = class(TPasParser)
  private
    FLog: TPas2jsLogger;
  public
    constructor Create(AScanner: TPascalScanner;
      AFileResolver: TBaseFileResolver; AEngine: TPasTreeContainer); reintroduce;
    procedure RaiseParserError(MsgNumber: integer;
      Args: array of const);
    procedure ParseSubModule(var Module: TPasModule);
    property Log: TPas2jsLogger read FLog write FLog;
  end;

  TOnFindModule = function(const AUnitName, InFilename: String; NameExpr,
      InFileExpr: TPasExpr): TPasModule of object;
  TOnCheckSrcName = procedure(const aElement: TPasElement) of object;

  { TPas2jsCompilerResolver }

  TPas2jsCompilerResolver = class(TPas2JSResolver)
  private
    FLog: TPas2jsLogger;
    FOnCheckSrcName: TOnCheckSrcName;
    FOnFindModule: TOnFindModule;
    FP2JParser: TPas2jsPasParser;
  public
    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement; AVisibility: TPasMemberVisibility;
      const ASrcPos: TPasSourcePos; TypeParams: TFPList = nil): TPasElement;
      overload; override;
    function FindModule(const aUnitname: String): TPasModule; override;
    function FindUnit(const AName, InFilename: String; NameExpr,
      InFileExpr: TPasExpr): TPasModule; override;
    procedure UsedInterfacesFinished(Section: TPasSection); override;
  public
    Owner: TObject;
    property OnFindModule: TOnFindModule read FOnFindModule write FOnFindModule;
    property OnCheckSrcName: TOnCheckSrcName read FOnCheckSrcName write FOnCheckSrcName;
    property Log: TPas2jsLogger read FLog write FLog;
    property P2JParser: TPas2jsPasParser read FP2JParser write FP2JParser;
  end;

procedure RegisterMessages(Log: TPas2jsLogger);

implementation

procedure RegisterMessages(Log: TPas2jsLogger);
var
  LastMsgNumber: integer;

  procedure r(MsgType: TMessageType; MsgNumber: integer; const MsgPattern: string);
  var
    s: String;
  begin
    if (LastMsgNumber>=0) and (MsgNumber<>LastMsgNumber+1) then
      begin
      s:='gap in registered message numbers: '+IntToStr(LastMsgNumber)+' '+IntToStr(MsgNumber);
      {AllowWriteln}
      writeln('Pas2jsPParser.RegisterMessages ',s);
      {AllowWriteln-}
      raise Exception.Create(s);
      end;
    Log.RegisterMsg(MsgType,MsgNumber,MsgPattern);
    LastMsgNumber:=MsgNumber;
  end;

begin
  LastMsgNumber:=-1;
  r(mtError,nFinalizationNotSupported,sFinalizationNotSupported);
end;

{ TPas2jsPasParser }

constructor TPas2jsPasParser.Create(AScanner: TPascalScanner;
  AFileResolver: TBaseFileResolver; AEngine: TPasTreeContainer);
begin
  inherited Create(AScanner,AFileResolver,AEngine);
  Options:=Options+po_pas2js;
end;

procedure TPas2jsPasParser.RaiseParserError(MsgNumber: integer;
  Args: array of const);
var
  Msg: TPas2jsMessage;
begin
  Msg:=Log.FindMsg(MsgNumber,true);
  SetLastMsg(Msg.Typ,MsgNumber,Msg.Pattern,Args);
  raise EParserError.Create(LastMsg,Scanner.CurFilename,
                            Scanner.CurRow,Scanner.CurColumn);
end;

procedure TPas2jsPasParser.ParseSubModule(var Module: TPasModule);
begin
  Module:=nil;
  NextToken;
  SaveComments;
  case CurToken of
  tkUnit:
    ParseUnit(Module);
  tkLibrary:
    ParseLibrary(Module);
  else
    CheckToken(tkUnit);
  end;
end;

{ TPas2jsCompilerResolver }

function TPas2jsCompilerResolver.CreateElement(AClass: TPTreeElement;
  const AName: String; AParent: TPasElement; AVisibility: TPasMemberVisibility;
  const ASrcPos: TPasSourcePos; TypeParams: TFPList): TPasElement;
begin
  if AClass=TFinalizationSection then
    (CurrentParser as TPas2jsPasParser).RaiseParserError(nFinalizationNotSupported,[]);
  Result:=inherited CreateElement(AClass,AName,AParent,AVisibility,ASrcPos,TypeParams);
  if (Result is TPasModule) then
    OnCheckSrcName(Result);
end;

function TPas2jsCompilerResolver.FindModule(const aUnitname: String): TPasModule;
begin
  raise EPasResolve.Create('Call TPas2jsCompilerResolver.FindModule(name,expr,...) instead');
  Result:=nil;
  if aUnitname='' then ;
end;

function TPas2jsCompilerResolver.FindUnit(const AName, InFilename: String;
  NameExpr, InFileExpr: TPasExpr): TPasModule;
begin
  Result:=OnFindModule(AName,InFilename,NameExpr,InFileExpr);
end;

procedure TPas2jsCompilerResolver.UsedInterfacesFinished(Section: TPasSection);
begin
  if Section=nil then ;
end;

end.

