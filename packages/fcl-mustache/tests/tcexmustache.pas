{
    This file is part of the Free Pascal Run time library.
    Copyright (c) 2021 by Michael Van Canneyt (michael@freepascal.org)

    Test cases for expression parser support

    See the File COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit tcexmustache;

{$mode ObjFPC}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, fpjson, testregistry, fpmustache, tcbasemustache, fpexmustache, fpexprpars;

Type

  { TTestExMustacheParser }

  TTestExMustacheParser = Class(TBaseMustacheTest)
  private
    FExpr: TFPExpressionParser;
    FOutput: TMustacheStringOutput;
    FContext : TMustacheJSONContext;
    FData : TJSONData;
    procedure GetVar(var Result: TFPExpressionResult; ConstRef
      AName: ShortString);
  Public
    Procedure SetUp; override;
    Procedure TearDown; override;
    Function CreateParser: TMustacheParser; override;
    Property Expr : TFPExpressionParser Read FExpr;
    Property Output : TMustacheStringOutput Read FOutput;
  Published
    Procedure TestSimple;
    Procedure TestRenderSimple;
    Procedure TestRenderSection;
  end;

  { TTestMustacheExpr }

  TTestMustacheExpr = Class(TTestCase)
  private
    FJSON: TJSONObject;
    FMustache: TMustacheExpr;
  public
    Procedure SetUp; override;
    Procedure TearDown; override;
    Property Mustache : TMustacheExpr Read FMustache;
    Property JSON : TJSONObject Read FJSON;
  Published
    Procedure TestEmpty;
    Procedure TestRegisterVariables;
    Procedure TestRenderSection;
    procedure TestRenderSectionStaticVariables;
  end;


implementation

Const
  STestJSON = '{ "data" : [ { "name": "me", "age" : 10}, { "name": "you",  "age" : 12  }, { "name": "he", "age" : 13 } ] }';

{ TTestMustacheExpr }

procedure TTestMustacheExpr.SetUp;
begin
  inherited SetUp;
  FMustache:=TMustacheExpr.Create(Nil);
  FJSON:=GetJSON(STestJSON) as TJSONObject;
end;

procedure TTestMustacheExpr.TearDown;
begin
  FreeAndNil(FJSON);
  FreeAndNil(FMustache);
  inherited TearDown;
end;

procedure TTestMustacheExpr.TestEmpty;
begin
  AssertNotNull('Have mustache instance',Mustache);
  AssertNotNull('Have mustache expression engine instance',Mustache.ExpressionParser);
end;

procedure TTestMustacheExpr.TestRegisterVariables;
begin
  Mustache.RegisterVariables(JSON,'data[0]',True);
  AssertEquals('Variable count',2,Mustache.ExpressionParser.Identifiers.Count);
  AssertEquals('Variable 0','name',Mustache.ExpressionParser.Identifiers[0].Name);
  AssertEquals('Variable 1','age',Mustache.ExpressionParser.Identifiers[1].Name);
  AssertTrue('Variable 0 type',rtString=Mustache.ExpressionParser.Identifiers[0].ResultType);
  AssertTrue('Variable 1 type',rtInteger=Mustache.ExpressionParser.Identifiers[1].ResultType);
end;

procedure TTestMustacheExpr.TestRenderSection;

Var
  S : String;

Const
  Template = '{{#data}}{{[name]}}:{{[age>11]}} {{/data}}';

begin
  Mustache.Template:=Template;
  Mustache.RegisterVariables(JSON,'data[0]',True);
  S:=Mustache.Render(JSON);
  AssertEquals('Correct result','me:False you:True he:True ',S);
end;

procedure TTestMustacheExpr.TestRenderSectionStaticVariables;
Var
  S : String;

Const
  Template = '{{#data}}{{[name]}}:{{[age>11]}} {{/data}}';

begin
  Mustache.Template:=Template;
  Mustache.RegisterVariables(JSON,'data[0]',False);
  S:=Mustache.Render(JSON);
  AssertEquals('Correct result','me:False me:False me:False ',S);
end;


{ TTestExMustacheParser }

procedure TTestExMustacheParser.SetUp;
begin
  FExpr:=TFPExpressionParser.Create(Nil);
  Foutput:=TMustacheStringOutput.Create;
  inherited SetUp;
end;

procedure TTestExMustacheParser.TearDown;
begin
  inherited TearDown;
  FreeAndNil(FExpr);
  FreeAndNil(Foutput);
  FreeAndNil(FContext);
  FreeAndNil(FData);
end;

function TTestExMustacheParser.CreateParser: TMustacheParser;

Var
  P : TMustacheExprParser;

begin
  P:=TMustacheExprParser.Create;
  P.ExprParser:=FExpr;
  Result:=P;
end;

procedure TTestExMustacheParser.TestSimple;
begin
  Template:='{{[1+2]}}';
  CallParser;
  AssertElement(0,metVariable,'1+2',TMustacheExprElement);
end;

procedure TTestExMustacheParser.TestRenderSimple;
begin
  TestSimple;
  ParseResult.Children[0].Render(Nil,Output,'',False);
  AssertEquals('Correct result','3',Output.Data);
end;

procedure TTestExMustacheParser.GetVar(Var Result : TFPExpressionResult; ConstRef AName : ShortString);

begin
  Result.ResultType:=rtInteger;
  Result.ResInteger:=StrToINt(FContext.GetTextValue('age'));
end;

procedure TTestExMustacheParser.TestRenderSection;
begin
  FData:=GetJSON(STestJSON);
  FContext:=TMustacheJSONContext.Create(FData,Nil);
  FExpr.Identifiers.AddVariable('age',rtInteger,@GetVar);
  Template:='{{#data}}{{{name}}}:{{[age>11]}} {{/data}}';
  CallParser;
  ParseResult.Render(FContext,Output,'',False);
  AssertEquals('Correct result','me:False you:True he:True ',Output.Data);
end;

initialization
  RegisterTests([TTestExMustacheParser,TTestMustacheExpr]);
end.

