{
    This file is part of the Free Pascal Run time library.
    Copyright (c) 2021 by Michael Van Canneyt (michael@freepascal.org)

    This file contains a Mustache descendent with FPExpr parser expression support

    See the File COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit fpexmustache;

{$mode ObjFPC}{$H+}

interface

uses
  Classes, fpexprpars, fpmustache, fpjson;

Type

  { TMustacheExprElement }

  TMustacheExprElement = Class(TMustacheElement)
  private
    FNode: TFPExprNode;
    FExpr : TMustacheString;
  Protected
    Procedure SetNode(aNode : TFPExprNode); virtual;
    Function GetData : TMustacheString;override;
    Procedure SetData(const aValue : TMustacheString) ; override;
  Public
    Destructor Destroy; override;
    Procedure Render(aContext : TMustacheContext; aOutput : TMustacheOutput; const aPrefix : String = ''; aLast : Boolean = False); override;
    Property Node : TFPExprNode Read FNode;
  end;

  { TMustacheExprParser }

  TMustacheExprParser = class(TMustacheParser)
  private
    FExprEnd: Char;
    FExprParser: TFPExpressionParser;
    FExprStart: Char;
  Protected
    function CreateDefault(aParent: TMustacheElement; aPosition: Integer; const aName: String): TMustacheElement; override;
  Public
    Constructor Create(aTemplate : TMustacheString = '';aStart: TMustacheString='';aStop: TMustacheString = ''); override;
    // Default [
    Property ExprStart : Char Read FExprStart Write FExprStart;
    // Default ]
    Property ExprEnd : Char Read FExprEnd Write FExprEnd;
    // Our instance
    Property ExprParser : TFPExpressionParser Read FExprParser Write FExprParser;
  end;

  { TMustacheExpr }

  TMustacheExpr = Class(TMustache)
  private
    FExprEndChar: String;
    FExpressionParser: TFPExpressionParser;
    FExprStartChar: String;
    FCurrentContext : TMustacheContext;
    function GetResultType(aValue: TJSONData): TResultType;
    procedure SetExprEndChar(AValue: String);
    procedure SetExpressionParser(AValue: TFPExpressionParser);
    procedure SetExprStartChar(AValue: String);
    function DoGetExpressionParser : TFPExpressionParser;
  Protected
    procedure DoGetVariable(var Result: TFPExpressionResult; ConstRef  AName: ShortString); virtual;
    Procedure Notification(AComponent: TComponent; Operation: TOperation); override;
    Function CreateParser(aTemplate: TMustacheString): TMustacheParser; override;
    function GetExpressionParser(aOwner : TComponent): TFPExpressionParser; virtual;
  Public
    Constructor Create(aOwner : TComponent); override;
    Procedure Render(aContext : TMustacheContext; aOutput : TMustacheOutput); override; overload;
    // Register variables from JSON in the expression engine.
    // If UseEvent is true, the variables will be retrieved while parsing with an event.
    // If UseEvent is false, the variables will be registered as static values.
    Procedure RegisterVariables (aContext : TMustacheJSONContext; aPath : TJSONStringType = ''; UseEvent : Boolean = True);
    Procedure RegisterVariables (aJSON : String; aPath : TJSONStringType = ''; UseEvent : Boolean = True);
    Procedure RegisterVariables (aJSON : TJSONObject; aPath : TJSONStringType = ''; UseEvent : Boolean = True);
  Published
    // Default [
    Property ExprStartChar : String Read FExprStartChar Write SetExprStartChar;
    // Default ]
    Property ExprEndChar : String Read FExprEndChar Write SetExprEndChar;
    // An expression parser instance. If none is specified, then a default is created.
    Property ExpressionParser : TFPExpressionParser Read DoGetExpressionParser Write SetExpressionParser;
  end;

  { TMustacheExpressionParser }

  TMustacheExpressionParser = class(TFPExpressionParser)
  end;

implementation

uses sysutils;

Resourcestring
  SErrLengthStartMustBe1 = 'Length expression start delimiter must be 1';
  SErrLengthEndMustBe1 = 'Length expression end delimiter must be 1';

{ TMustacheExprElement }

procedure TMustacheExprElement.SetNode(aNode: TFPExprNode);
begin
  FNode:=aNode;
end;

function TMustacheExprElement.GetData: TMustacheString;
begin
  Result:=FExpr;
end;

procedure TMustacheExprElement.SetData(const aValue: TMustacheString);
begin
  FExpr:=aValue;
end;

procedure TMustacheExprElement.Render(aContext: TMustacheContext;
  aOutput: TMustacheOutput; const aPrefix: String; aLast: Boolean);

Var
  Res : TFPExpressionResult;
  S : TMustacheString;

begin
  Res:=Node.NodeValue;
  case Res.ResultType of
    rtString   : S:=Res.ResString;
    rtBoolean  : S:=BoolToStr(Res.ResBoolean,True);
    rtInteger  : S:=IntToStr(Res.ResInteger);
    rtFloat    : S:=FormatFloat('0.0#######',Res.ResFloat);
    rtCurrency : S:=CurrToStr(Res.ResCurrency);
    rtDateTime : S:=DateTimeToStr(Res.ResDateTime);
  end;
  aOutput.Output(aPrefix+S);
end;

destructor TMustacheExprElement.Destroy;
begin
  FreeAndNil(FNode);
  inherited Destroy;
end;

{ TMustacheExprParser }

function TMustacheExprParser.CreateDefault(aParent: TMustacheElement;
  aPosition: Integer; const aName: String): TMustacheElement;

Var
  L : Integer;
  N : TFPExprNode;

begin
  N:=Nil;
  L:=Length(aName);
  If (aName[1]=FExprStart) and (aName[L]=FExprEnd) then
    begin
    Result:=TMustacheExprElement.Create(metVariable,aParent,aPosition);
    Result.Data:=Copy(aName,2,L-2);
    ExprParser.Expression:=Result.Data;
    ExprParser.ExtractNode(N);
    TMustacheExprElement(Result).SetNode(N);
    aParent.AddChild(Result);
    end
  else
    Result:=Inherited CreateDefault(aParent,aPosition,aName);
end;

constructor TMustacheExprParser.Create(aTemplate: TMustacheString;
  aStart: TMustacheString; aStop: TMustacheString);
begin
  inherited Create(aTemplate, aStart, aStop);
  FExprStart:='[';
  FExprEnd:=']';
end;

{ TMustacheExpr }

procedure TMustacheExpr.SetExprEndChar(AValue: String);
begin
  if FExprEndChar=AValue then Exit;
  if Length(aValue)<>1 then
    EMustache.Create(SErrLengthStartMustBe1);
  FExprEndChar:=AValue;
end;

function TMustacheExpr.GetExpressionParser(aOwner : TComponent): TFPExpressionParser;
begin
  Result:=TMustacheExpressionParser.Create(AOwner);
end;

procedure TMustacheExpr.SetExpressionParser(AValue: TFPExpressionParser);

begin
  if FExpressionParser=AValue then Exit;
  If assigned(FExpressionParser) then
    FExpressionParser.RemoveFreeNotification(Self);
  FExpressionParser:=AValue;
  If assigned(FExpressionParser) then
    FExpressionParser.FreeNotification(Self);
end;

procedure TMustacheExpr.SetExprStartChar(AValue: String);
begin
  if FExprStartChar=AValue then Exit;
  if Length(aValue)<>1 then
    EMustache.Create(SErrLengthEndMustBe1);
  FExprStartChar:=AValue;
end;

function TMustacheExpr.DoGetExpressionParser: TFPExpressionParser;
begin
  if FExpressionParser=Nil then
    begin
    FExpressionParser:=GetExpressionParser(Self);
    FExpressionParser.SetSubComponent(True);
    FExpressionParser.FreeNotification(Self);
    end;
  Result:=FExpressionParser;
end;

procedure TMustacheExpr.Notification(AComponent: TComponent;
  Operation: TOperation);
begin
  inherited Notification(AComponent, Operation);
  if (Operation=opRemove) and (aComponent=FExpressionParser) then
    FExpressionParser:=Nil;
end;

function TMustacheExpr.CreateParser(aTemplate: TMustacheString ): TMustacheParser;

Var
  Exp : TMustacheExprParser;

begin
  Exp:=TMustacheExprParser.Create(aTemplate);
  Exp.ExprParser:=Self.ExpressionParser;
  Result:=Exp;
end;

constructor TMustacheExpr.Create(aOwner: TComponent);
begin
  inherited Create(aOwner);
  DoGetExpressionParser;
end;

procedure TMustacheExpr.Render(aContext: TMustacheContext; aOutput: TMustacheOutput);

begin
  FCurrentContext:=aContext;
  try
    inherited Render(aContext, aOutput);
  finally
    FCurrentContext:=nil;
  end;
end;

procedure TMustacheExpr.DoGetVariable(var Result: TFPExpressionResult; ConstRef
  AName: ShortString);

Var
  S : TMustacheString;
  V : Double;
  C : Integer;

begin
  If not Assigned(FCurrentContext) then
    case result.ResultType of
      rtInteger : Result.ResInteger:=0;
      rtDateTime : Result.ResDateTime:=0.0;
      rtString : Result.ResString:='';
      rtFloat: Result.ResFloat:=0.0;
      rtCurrency: Result.ResCurrency:=0.0;
      rtBoolean: Result.ResBoolean:=False;
    end
  else
    begin
    S:=FCurrentContext.GetTextValue(aName);
    case result.ResultType of
      rtInteger : Result.ResInteger:=StrToInt64Def(S,0);
      rtDateTime : if Not TryStrToDateTime(S,Result.ResDateTime) then
                     Result.ResDateTime:=0.0;
      rtString : Result.ResString:=S;
      rtFloat: begin
               Val(S,V,C);
               if C<>0 then
                 Result.ResFloat:=0.0
               else
                 Result.ResFloat:=V;
               end;
      rtCurrency:
               begin
               Val(S,V,C);
               if (C<>0) then
                 Result.ResCurrency:=0.0
               else
                 Result.ResCurrency:=V;
               end;
      rtBoolean: Result.ResBoolean:=StrToBoolDef(S,False);
    end;
    end;
end;

function TMustacheExpr.GetResultType(aValue: TJSONData): TResultType;

begin
  Case aValue.JSONType of
    jtBoolean : Result:=rtBoolean;
    jtString,
    jtArray,
    jtObject,
    jtNull : Result:=rtString;
    jtNumber :
       begin
       Case TJSONNumber(aValue).NumberType of
         ntFloat : Result:=rtFloat;
         ntInteger,
         ntInt64 : Result:=rtInteger;
         ntQWord : Raise EMustache.Create('Unsupported JSON type');
       end;
       end;
  end;
end;

procedure TMustacheExpr.RegisterVariables(aContext: TMustacheJSONContext;
  aPath: TJSONStringType; UseEvent: Boolean);

begin
  RegisterVariables(aContext.RootData as TJSONObject,aPath,UseEvent);
end;

procedure TMustacheExpr.RegisterVariables(aJSON: String;
  aPath: TJSONStringType; UseEvent: Boolean);

Var
  aData : TJSONData;
  aObj : TJSONObject absolute aData;


begin
  aData:=getJSON(aJSON,True);
  try
    if aData is TJSONObject then
      RegisterVariables(aObj,aPath,useEvent)
    else
      Raise EMustache.Create('Invalid JSON data to register variables');
  finally
    aData.Free;
  end;
end;

procedure TMustacheExpr.RegisterVariables(aJSON: TJSONObject; aPath: TJSONStringType; UseEvent: Boolean);

Var
  aData,aValue : TJSONData;
  aEnum : TJSONEnum;
  aKey : TJSONStringType;
  rt : TResultType;
  aParser : TFPExpressionParser;

begin
  aParser:=ExpressionParser;
  aData:=aJSON.FindPath(aPath);
  if aData is TJSONObject then
    for aEnum in aData do
      begin
      aKey:=aEnum.Key;
      aValue:=aEnum.Value;
      rt:=GetResultType(aValue);
      if UseEvent then
        aParser.Identifiers.AddVariable(aKey,rt,@DoGetVariable)
      else
        case rt of
          rtBoolean: aParser.Identifiers.AddBooleanVariable(aKey,aValue.AsBoolean);
          rtFloat: aParser.Identifiers.AddFloatVariable(aKey,aValue.AsFloat);
          rtInteger: aParser.Identifiers.AddIntegerVariable(aKey,aValue.AsInteger);
          rtString: Case aValue.JSONType of
                      jtNull: aParser.Identifiers.AddStringVariable(aKey,'');
                      jtArray,
                      jtObject: aParser.Identifiers.AddStringVariable(aKey, aValue.AsJSON);
                    else
                      aParser.Identifiers.AddStringVariable(aKey,aValue.AsString);
                    end;
          end;
      end;
end;

end.

