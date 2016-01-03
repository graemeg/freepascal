{$mode objfpc}
{$h+}
{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2004 by Dean Zobec

    Port to Free Pascal of the JUnit framework.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit asserttest;

interface

uses
  fpcunit, testregistry, sysutils;

type

  { TAssertTest }

  TAssertTest = class(TTestCase)
  published
    procedure TestFail;
    procedure TestIgnore;
    procedure TestAssertSame;
    procedure TestAssertSameNull;
    procedure TestAssertNotSameFailsNull;
    procedure TestAssertStringEquals;
    procedure TestNullNotSameObject;
    procedure TestAssertNull;
    procedure TestAssertNotNull;
    procedure TestAssertTrue;
    procedure TestAssertFalse;
    procedure TestAssertNotSame;
    procedure TestExpectExceptionOK;
    procedure TestExpectExceptionNoException;
    procedure TestExpectExceptionWrongExceptionClass;
    procedure TestExpectExceptionWrongExceptionMessage;
    procedure TestExpectExceptionWrongExceptionContext;
  end;

  EMyException = Class(Exception);

  { TMyTest }

  TMyTest = class(TTestCase)
  published
    procedure RaiseIgnoreTest;
    procedure TestExpectException;
    procedure TestExpectExceptionNone;
    procedure TestExpectExceptionWrongClass;
    procedure TestExpectExceptionWrongMessage;
    procedure TestExpectExceptionWrongHelpContext;
  end;

  TTestIgnore = class(TTestCase)
  published
    procedure TestIgnoreResult;
    procedure TestIgnoreActivation;
    procedure TestIgnoreSetting;
  end;

implementation

procedure TAssertTest.TestFail;
begin
  try
    fail('Wrong or no exception raised with fail');
  except
    on E: EAssertionfailedError do
      Exit;
  end;
  raise EAssertionFailedError.Create;
end;

procedure TAssertTest.TestIgnore;
begin
  try
    Ignore('Ignored Test');
  except
    on E: EIgnoredTest do
      Exit;
  end;
  fail('Wrong or no Exception raised with ignore');
end;

procedure TAssertTest.TestAssertSame;
var
  o: TObject;
  o1: TObject;
begin
  o := TObject.Create;
  AssertSame(o, o);
  o1 := TObject.Create;
  try
    AssertSame(o, o1);
  except
    on E: EAssertionFailedError do
    begin
      o.Free;
      o1.Free;
      Exit;
    end;
  end;
  o.Free;
  o1.Free;
  Fail('Wrong or no exception raised');
end;

procedure TAssertTest.TestAssertSameNull;
var
  a, b: TObject;
begin
  a := nil;
  b := nil;
  AssertSame(a, b);
  AssertSame(nil, a);
  AssertSame(a, nil);
end;

procedure TAssertTest.TestAssertNotSameFailsNull;
var
  a, b: TObject;
begin
  a := nil;
  b := nil;
  try
    assertNotSame(a, b);
  except
    on E: EAssertionFailedError do
    Exit;
  end;
  fail('error: nil should equal nil');
end;

procedure TAssertTest.TestAssertStringEquals;
begin
  AssertEquals('a', 'a')
end;

procedure TAssertTest.TestNullNotSameObject;
var
  obj: TObject;
begin
  obj := TObject.Create;
  try
    AssertSame(nil, obj);
  except
    on E: EAssertionFailedError do
    begin
      obj.Free;
      Exit;
    end;
  end;
  Fail('error comparing a valid obj instance with nil');
end;

procedure TAssertTest.TestAssertNull;
var
  obj: TObject;
begin
  AssertNull(nil);
  obj := TObject.Create;
  try
    AssertNull(obj);
  except
    on E: EAssertionFailedError do
    begin
      obj.Free;
      Exit;
    end;
  end;
  obj.Free;
  Fail('failure: obj is not null!');
end;

procedure TAssertTest.TestAssertNotNull;
var
  obj: TObject;
begin
  obj := TObject.Create;
  AssertNotNull(obj);
  try
    AssertNotNull(nil);
  except
    on E: EAssertionFailedError do
    begin
      obj.Free;
      Exit;
    end;
  end;
  obj.Free;
  Fail('error: nil is not a valid object');
end;

procedure TAssertTest.TestAssertTrue;
begin
  assertTrue(true);
  try
    assertTrue(false);
  except
    on E: EAssertionFailedError do
    Exit;
  end;
  fail('error asserting true');
end;

procedure TAssertTest.TestAssertFalse;
begin
  assertFalse(false);
  try
    assertFalse(true);
  except
    on E: EAssertionFailedError do
    Exit;
  end;
  fail('error asserting false');
end;

procedure TAssertTest.TestAssertNotSame;
var
  obj: TObject;
  obj1: TObject;
begin
  obj := TObject.Create;
  obj1 := TObject.Create;
  AssertNotSame(obj, nil);
  AssertNotSame(nil, obj);
  AssertNotSame(obj, obj1);
  try
    AssertNotSame(obj, obj)
  except
    on E: EAssertionFailedError do
    begin
      obj.Free;
      obj1.Free;
      Exit;
    end;
  end;
  obj.Free;
  obj1.Free;
  Fail('Error: Objects are the same!');
end;

procedure TAssertTest.TestExpectExceptionOK;
var
  t: TMyTest;
  res: TTestResult;
begin
  t := TMyTest.CreateWithName('TestExpectException');
  res := t.CreateResultAndRun;
  assertEquals('no test was run', 1, res.RunTests);
  assertEquals('no Ignored Test present', 0, res.NumberOfIgnoredTests);
  assertEquals('no failed Test present', 0, res.NumberOfFailures);
  t.Free;
  res.Free;
end;

procedure TAssertTest.TestExpectExceptionNoException;

var
  t: TMyTest;
  res: TTestResult;
begin
  t := TMyTest.CreateWithName('TestExpectExceptionNone');
  res := t.CreateResultAndRun;
  assertEquals('no test was run', 1, res.RunTests);
  assertEquals('no Ignored Test present', 0, res.NumberOfIgnoredTests);
  assertEquals('no failed Test present', 1, res.NumberOfFailures);
  assertEquals('Correct error message','Error message : Exception EMyException expected but no exception was raised',TTestFailure(res.Failures[0]).ExceptionMessage);
  t.Free;
  res.Free;
end;

procedure TAssertTest.TestExpectExceptionWrongExceptionClass;
var
  t: TMyTest;
  res: TTestResult;
begin
  t := TMyTest.CreateWithName('TestExpectExceptionWrongClass');
  res := t.CreateResultAndRun;
  assertEquals('no test was run', 1, res.RunTests);
  assertEquals('no Ignored Test present', 0, res.NumberOfIgnoredTests);
  assertEquals('no failed Test present', 1, res.NumberOfFailures);
  assertEquals('Correct error message','Error message : Exception EMyException expected but Exception was raised',TTestFailure(res.Failures[0]).ExceptionMessage);
  t.Free;
  res.Free;
end;

procedure TAssertTest.TestExpectExceptionWrongExceptionMessage;

var
  t: TMyTest;
  res: TTestResult;
begin
  t := TMyTest.CreateWithName('TestExpectExceptionWrongMessage');
  res := t.CreateResultAndRun;
  assertEquals('no test was run', 1, res.RunTests);
  assertEquals('no Ignored Test present', 0, res.NumberOfIgnoredTests);
  assertEquals('no failed Test present', 1, res.NumberOfFailures);
  assertEquals('Correct error message','Error message : Exception raised but exception property Message differs:  expected: <A message> but was: <A wrong message>',TTestFailure(res.Failures[0]).ExceptionMessage);
  t.Free;
  res.Free;
end;

procedure TAssertTest.TestExpectExceptionWrongExceptionContext;
var
  t: TMyTest;
  res: TTestResult;
begin
  t := TMyTest.CreateWithName('TestExpectExceptionWrongHelpContext');
  res := t.CreateResultAndRun;
  assertEquals('no test was run', 1, res.RunTests);
  assertEquals('no Ignored Test present', 0, res.NumberOfIgnoredTests);
  assertEquals('no failed Test present', 1, res.NumberOfFailures);
  assertEquals('Correct error message','Error message : Exception raised but exception property HelpContext differs:  expected: <123> but was: <124>',TTestFailure(res.Failures[0]).ExceptionMessage);
  t.Free;
  res.Free;
end;

procedure TMyTest.RaiseIgnoreTest;
begin
  Ignore('This is an ignored test');
  AssertEquals('the compiler can count', 3, 2);
<<<<<<< HEAD
end;

procedure TMyTest.TestExpectException;
begin
  ExpectException('Error message',EMyException,'A message',123);
  Raise EMyException.CreateHelp('A message',123);
end;

procedure TMyTest.TestExpectExceptionNone;
begin
  ExpectException('Error message',EMyException,'A message',123);
end;

procedure TMyTest.TestExpectExceptionWrongClass;
begin
  ExpectException('Error message',EMyException,'A message',123);
  Raise Exception.CreateHelp('A message',123);
end;

procedure TMyTest.TestExpectExceptionWrongMessage;
begin
  ExpectException('Error message',EMyException,'A message',123);
  Raise EMyException.CreateHelp('A wrong message',123);
end;

procedure TMyTest.TestExpectExceptionWrongHelpContext;
begin
  ExpectException('Error message',EMyException,'A message',123);
  Raise EMyException.CreateHelp('A message',124);
=======
>>>>>>> origin/fpcunit
end;

procedure TTestIgnore.TestIgnoreResult;
var
  t: TMyTest;
  res: TTestResult;
begin
  t := TMyTest.CreateWithName('RaiseIgnoreTest');
  res := t.CreateResultAndRun;
  assertEquals('no test was run', 1, res.RunTests);
  assertEquals('no Ignored Test present', 1, res.NumberOfIgnoredTests);
  assertTrue('failure is not signalled as Ignored Test', TTestFailure(res.IgnoredTests[0]).IsIgnoredTest);
  assertEquals('wrong failure name', 'EIgnoredTest', TTestFailure(res.IgnoredTests[0]).ExceptionClassName);
  assertEquals('wrong message', 'This is an ignored test', TTestFailure(res.IgnoredTests[0]).ExceptionMessage);
  t.Free;
  res.Free;
end;

procedure TTestIgnore.TestIgnoreActivation;
var
  t: TMyTest;
  res: TTestResult;
begin
  t := TMyTest.CreateWithName('RaiseIgnoreTest');
  t.EnableIgnores := false;
  res := t.CreateResultAndRun;
  assertEquals('no test was run', 1, res.RunTests);
  assertEquals('Ignored Test reported even if the switch is not active', 0, res.NumberOfIgnoredTests);
  assertEquals('no failure caught', 1, res.NumberOfFailures);
  assertFalse('failure is signalled as Ignored Test and the switch is not active', 
    TTestFailure(res.Failures[0]).IsIgnoredTest);
  assertEquals('wrong failure name', 'EAssertionFailedError', TTestFailure(res.Failures[0]).ExceptionClassName);
  assertEquals('wrong message', '"the compiler can count" expected: <3> but was: <2>', TTestFailure(res.Failures[0]).ExceptionMessage);
  t.Free;
  res.Free;
end;

procedure TTestIgnore.TestIgnoreSetting;
var
  ts: TTestSuite;
  i: integer;
begin
  ts := TTestSuite.Create(TTestIgnore);
  try
    AssertTrue('EnableIgnores must be True at creation', ts.EnableIgnores);
    for i := 0 to ts.Tests.Count - 1 do
      AssertTrue('EnableIgnores of Test ' + IntToStr(i) + ' must be True at creation', TTest(ts.Tests[i]).EnableIgnores);
    ts.EnableIgnores := False; 
    AssertFalse('EnableIgnores was not set to false', ts.EnableIgnores);
    for i := 0 to ts.Tests.Count - 1 do
      AssertFalse('EnableIgnores of Test ' + IntToStr(i) + ' was not set to False', TTest(ts.Tests[i]).EnableIgnores);
  finally
    ts.Free;
  end;
end;


initialization

  RegisterTests([TAssertTest, TTestIgnore]);

end.
