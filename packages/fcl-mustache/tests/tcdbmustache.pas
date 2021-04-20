{
    This file is part of the Free Pascal Run time library.
    Copyright (c) 2021 by Michael Van Canneyt (michael@freepascal.org)

    Test cases for DB Context for Mustache

    See the File COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit tcdbmustache;

{$mode ObjFPC}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testregistry, fpmustache, db, bufdataset, fpdbmustache;

Type

  { TTestMustacheDBContext }

  TTestMustacheDBContext = Class(TTestCase)
  private
    FContext: TMustacheDBContext;
    FDataset1: TBufDataset;
    FDataset2: TBufDataset;
    FMustache: TMustache;
  Public
    Procedure Setup; override;
    Procedure TearDown; override;
    Procedure CreateDataset1;
    Procedure CreateDataset2;
    Property Dataset1 : TBufDataset Read FDataset1;
    Property Dataset2 : TBufDataset Read FDataset2;
    Property Context : TMustacheDBContext Read FContext;
    Property Mustache : TMustache Read FMustache;
  Published
    Procedure TestEmpty;
    Procedure TestSingleSection;
    Procedure TestTwoSections;
  end;

implementation

Const
  Template1 = '{{title}}! {{#Parents}}{{name}} {{age}} - {{/Parents}}';
  Template2 = '{{title}}! {{#Parents}}{{name}}({{age}}) : {{#Children}}{{name}} {{age}},{{/Children}} - {{/Parents}}';

{ TTestMustacheDBContext }

procedure TTestMustacheDBContext.Setup;
begin
  Inherited;
  FDataset1:=TBufDataset.Create(Nil);
  FDataset1.Name:='Parents';
  FDataset2:=TBufDataset.Create(Nil);
  FDataset2.Name:='Children';
  FContext:=TMustacheDBContext.Create(Nil);
  FContext.StaticValues.Values['title']:='Family';
  FMustache:=TMustache.Create(Nil);
end;

procedure TTestMustacheDBContext.TearDown;
begin
  FreeAndNil(FDataset1);
  FreeAndNil(FDataset2);
  FreeAndNil(FContext);
  FreeAndNil(FMustache);
end;

procedure TTestMustacheDBContext.CreateDataset1;
begin
  FDataset1.FieldDefs.Add('name',ftString,20);
  FDataset1.FieldDefs.Add('age',ftInteger);
  FDataset1.CreateDataset;
  FDataset1.Append;
  FDataset1.FieldByName('name').AsString:='Father';
  FDataset1.FieldByName('age').AsInteger:=40;
  FDataset1.Post;
  FDataset1.Append;
  FDataset1.FieldByName('name').AsString:='Mother';
  FDataset1.FieldByName('age').AsInteger:=39;
  FDataset1.Post;
  FDataset1.First;
end;

procedure TTestMustacheDBContext.CreateDataset2;
begin
  FDataset2.FieldDefs.Add('name',ftString,20);
  FDataset2.FieldDefs.Add('age',ftInteger);
  FDataset2.CreateDataset;
  FDataset2.Append;
  FDataset2.FieldByName('name').AsString:='Child1';
  FDataset2.FieldByName('age').AsInteger:=4;
  FDataset2.Post;
  FDataset2.Append;
  FDataset2.FieldByName('name').AsString:='Child2';
  FDataset2.FieldByName('age').AsInteger:=2;
  FDataset2.Post;
  FDataset2.First;
end;

procedure TTestMustacheDBContext.TestEmpty;
begin
  AssertNotNull('Mustache',Mustache);
  AssertNotNull('Dataset1',Dataset1);
  AssertNotNull('Dataset2',Dataset2);
  AssertNotNull('Context',Context);
  AssertEquals('Context static','Family',Context.StaticValues.Values['title']);
end;

procedure TTestMustacheDBContext.TestSingleSection;

Var
  S : String;

begin
  Mustache.Template:=Template1;
  CreateDataset1;
  Context.AddDataset(FDataset1);
  S:=Mustache.Render(Context);
  AssertEquals('Correct result','Family! Father 40 - Mother 39 - ',S);
end;

procedure TTestMustacheDBContext.TestTwoSections;

Var
  S : String;

begin
  Mustache.Template:=Template2;
  CreateDataset1;
  CreateDataset2;
  Context.AddDataset(FDataset1);
  Context.AddDataset(FDataset2);
  S:=Mustache.Render(Context);
  AssertEquals('Correct result','Family! Father(40) : Child1 4,Child2 2, - Mother(39) :  - ',S);
end;

initialization
  RegisterTest(TTestMustacheDBContext);
end.

