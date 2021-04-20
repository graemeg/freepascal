{
    This file is part of the Free Pascal Run time library.
    Copyright (c) 2021 by Michael Van Canneyt (michael@freepascal.org)

    testcase for official Mustache tests

    See the File COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit tcspecs;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testregistry, fpmustache, fpjson, jsonparser;

Type

  { TTestMustacheSpecs }

  TTestMustacheSpecs = class(TTestCase)
  private
    FTests: TJSONArray;
    procedure RunMustacheTest(aIndex: Integer; aTest: TJSONObject);
  Public
    class var BaseDir : string;
  Public
    Procedure Setup; override;
    Procedure TearDown; override;
    Procedure DoTest(aFileName : string);
    Property Tests : TJSONArray Read FTests;
  Published
    Procedure TestComments;
    Procedure TestDelimiters;
    Procedure TestInterpolation;
    Procedure TestInverted;
    Procedure TestPartials;
    Procedure TestSections;
  end;


implementation

{ TTestMustacheSpecs }

procedure TTestMustacheSpecs.RunMustacheTest(aIndex : Integer; aTest : TJSONObject);

Var
  M : TMustache;
  aTempl,aErr,aRes,aName : TMustacheString;
  Parts : TJSONObject;
  I : Integer;
  Ok : Boolean;

  Procedure TreeDump;

  begin
    if not OK then
      begin
      Writeln('Tree dump:');
      Writeln(M.Dump);
      end;
  end;

  Procedure InputDump;

  begin
    Writeln('Test : ',aIndex);
    writeln(aTempl);
    writeln(StringReplace(StringReplace(aTempl,#10,' ',[rfReplaceAll]),#13,' ',[rfReplaceAll]));
    aName:='';
    While Length(aName)<Length(aTempl) do
      aName:=AName+'1234567890';
    Writeln(aName);
  end;

begin
  OK:=False;
  aTempl:=aTest.Get('template','');
  // InputDump;
  M:=TMustache.CreateMustache(Nil,aTempl);
  try
    // Load partials
    Parts:=aTest.Get('partials',TJSONObject(Nil));
    if Assigned(Parts) then
      for I:=0 to Parts.Count-1 do
        M.Partials.Add(Parts.Names[i]+'='+Parts.Items[i].AsString);
    // Set test name and run tests
    aName:='Test '+IntToStr(aIndex)+': '+aTest.Get('name','');
    Try
      aErr:='';
      aRes:=m.Render(aTest.Get('data',TJSONObject(Nil)));
    except
      on e : exception do
        aErr:=E.ClassName+' '+E.message;
    end;
    if aErr<>'' then
      Fail(aName+': Unexpected error: '+aErr);
    AssertEquals(aName,aTest.Get('expected',''),aRes);
    OK:=true;
  finally
    // TreeDump;
    M.Free;
  end;
end;

procedure TTestMustacheSpecs.Setup;
begin
  inherited Setup;
end;

procedure TTestMustacheSpecs.TearDown;
begin
  inherited TearDown;
end;

procedure TTestMustacheSpecs.DoTest(aFileName: string);

Var
  I : Integer;
  F : TFileStream;
  D : TJSONData;
  FN : String;

begin
  D:=Nil;
  FN:=IncludeTrailingPathDelimiter(BaseDir)+aFileName+'.json';
  F:=TFileStream.Create(FN,fmOpenRead or fmShareDenyWrite);
  try
    D:=GetJSON(F);
    if D is TJSONObject then
      begin
      Ftests:=(D as TJSONObject).Get('tests',TJSONArray(Nil));
      if (FTests=Nil) then
        Fail('Invalid mustache tests in '+FN);
      end
    else
      Fail('Invalid JSON object in '+FN);
    For I:=0 to Tests.Count-1 do
      RunMustacheTest(I,Tests.Items[i] as TJSONObject);
  finally
    D.Free;
    F.Free;
  end;
end;

procedure TTestMustacheSpecs.TestComments;
begin
  DoTest('comments');
end;

procedure TTestMustacheSpecs.TestDelimiters;
begin
  DoTest('delimiters');
end;

procedure TTestMustacheSpecs.TestInterpolation;
begin
  DoTest('interpolation');
end;

procedure TTestMustacheSpecs.TestInverted;
begin
  DoTest('inverted');
end;

procedure TTestMustacheSpecs.TestPartials;
begin
  DoTest('partials');
end;

procedure TTestMustacheSpecs.TestSections;
begin
  DoTest('sections');
end;

begin
  TTestMustacheSpecs.BaseDir:='spec/';
  RegisterTest(TTestMustacheSpecs);
end.

