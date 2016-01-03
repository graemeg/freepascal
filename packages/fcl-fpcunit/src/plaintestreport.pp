{$mode objfpc}
{$h+}
{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2006 by Dean Zobec

    an example of plain text report for FPCUnit tests.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit plaintestreport;

interface

uses
  classes, SysUtils, fpcunit, fpcunitreport;

type
  TTestResultOption = (ttoSkipAddress,ttoSkipExceptionMessage,ttoErrorsOnly);
  TTestResultOptions = set of TTestResultOption;

  { TPlainResultsWriter }

  TPlainResultsWriter = class(TCustomResultsWriter)
  private
    FTestResultOptions : TTestResultOptions;
    FDoc: TStringList;
    FSuiteHeaderIdx: TFPList;
    FTempFailure: TTestFailure;
    function TimeFormat(ATiming: TDateTime): String;
  protected
    procedure SetSkipAddressInfo(AValue: Boolean); override;
    procedure SetSparse(AValue: Boolean); override;
    procedure WriteTestHeader(ATest: TTest; ALevel: integer; ACount: integer); override;
    procedure WriteTestFooter(ATest: TTest; ALevel: integer; ATiming: TDateTime); override;
    procedure WriteSuiteHeader(ATestSuite: TTestSuite; ALevel: integer); override;
    procedure WriteSuiteFooter(ATestSuite: TTestSuite; ALevel: integer; 
      ATiming: TDateTime; ANumRuns: integer; ANumErrors: integer; 
      ANumFailures: integer; ANumIgnores: integer); override;
  public
    constructor Create(aOwner: TComponent); override;
    destructor  Destroy; override;
    procedure WriteHeader; override;
    procedure WriteResult(aResult: TTestResult); override;
    procedure AddFailure(ATest: TTest; AFailure: TTestFailure); override;
    procedure AddError(ATest: TTest; AError: TTestFailure); override;
  end;


function TestSuiteAsPlain(aSuite:TTestSuite; Options : TTestResultOptions = []): string;
function GetSuiteAsPlain(aSuite: TTestSuite; Options : TTestResultOptions = []): string;
function TestResultAsPlain(aTestResult: TTestResult; Options : TTestResultOptions = []): string;

implementation

uses dateutils;

{TPlainResultsWriter}

constructor TPlainResultsWriter.Create(aOwner: TComponent);
begin
  inherited Create(aOwner);
  FDoc := TStringList.Create;
  FSuiteHeaderIdx := TFPList.Create;
  FTempFailure := nil;
end;

destructor  TPlainResultsWriter.Destroy;
begin
  FDoc.Free;
  FSuiteHeaderIdx.Free;
  inherited Destroy;
end;

procedure TPlainResultsWriter.WriteHeader;
begin
end;

procedure TPlainResultsWriter.WriteResult(aResult: TTestResult);
var
  f: text;
begin
  system.Assign(f, FileName);
  rewrite(f);
  FDoc.Add('');
  FDoc.Add(TestResultAsPlain(aResult,FTestResultOptions));
  writeln(f, FDoc.Text);
  close(f);
end;

procedure TPlainResultsWriter.AddFailure(ATest: TTest; AFailure: TTestFailure);
begin
  inherited AddFailure(ATest, AFailure);
  FTempFailure := AFailure;
end;

procedure TPlainResultsWriter.AddError(ATest: TTest; AError: TTestFailure);
begin
  inherited AddError(ATest, AError);
  FTempFailure := AError;
end;

procedure TPlainResultsWriter.WriteTestHeader(ATest: TTest; ALevel: integer; ACount: integer);
begin
  inherited;
end;

procedure TPlainResultsWriter.WriteTestFooter(ATest: TTest; ALevel: integer; ATiming: TDateTime);

Var
  S : String;

begin
  inherited;
  S:='  ' + StringOfChar(' ',ALevel*2);
  if Not SkipTiming then
    S:=S + FormatDateTime(TimeFormat(ATiming), ATiming) + '  ';
  S:=S + ATest.TestName;
  if Assigned(FTempFailure) or (not Sparse) then
  FDoc.Add(S);
  if Assigned(FTempFailure) then
  begin
    //check if it's an error 
    if not FTempFailure.IsFailure then 
    begin
      FDoc[FDoc.Count -1] := FDoc[FDoc.Count -1] + '  Error: ' + FTempFailure.ExceptionClassName;
      FDoc.Add(StringOfChar(' ',ALevel*2) + '    Exception:   ' + FTempFailure.ExceptionMessage);
      FDoc.Add(StringOfChar(' ',ALevel*2) + '    at ' + FTempFailure.LocationInfo);
      // TODO: Add stack dump output info
    end
    else
      if FTempFailure.IsIgnoredTest then
      begin
         FDoc[FDoc.Count -1] := FDoc[FDoc.Count -1] + '  Ignored test: ' 
           + FTempFailure.ExceptionMessage;
      end
      else
      begin
        //is a failure
        FDoc[FDoc.Count -1] := FDoc[FDoc.Count -1] + '  Failed: ' 
          + FTempFailure.ExceptionMessage;
        FDoc.Add(StringOfChar(' ',ALevel*2) + '    Exception:   ' + FTempFailure.ExceptionMessage);
        FDoc.Add(StringOfChar(' ',ALevel*2) + '    at ' + FTempFailure.LocationInfo);
      end;
  end;
  FTempFailure := nil;
end;

function TPlainResultsWriter.TimeFormat(ATiming: TDateTime): String;

Var
  M : Int64;

begin
  Result:='ss.zzz';
  M:=MinutesBetween(ATiming,0);
  if M>60 then
    Result:='hh:mm:'+Result
  else if M>1 then
   Result:='mm:'+Result;
end;

procedure TPlainResultsWriter.SetSkipAddressInfo(AValue: Boolean);
begin
  inherited SetSkipAddressInfo(AValue);
  if AValue then
    Include(FTestResultOptions,ttoSkipAddress)
  else
    Exclude(FTestResultOptions,ttoSkipAddress);
end;

procedure TPlainResultsWriter.SetSparse(AValue: Boolean);
begin
  inherited SetSparse(AValue);
  if AValue then
    FTestResultOptions:=FTestResultOptions+[ttoSkipExceptionMessage,ttoErrorsOnly]
  else
    FTestResultOptions:=FTestResultOptions-[ttoSkipExceptionMessage,ttoErrorsOnly];
end;

procedure TPlainResultsWriter.WriteSuiteFooter(ATestSuite: TTestSuite; ALevel: integer; 
  ATiming: TDateTime; ANumRuns: integer; ANumErrors: integer; ANumFailures: integer;
  ANumIgnores: integer);
var
  idx: integer;
  S: String;
begin
  inherited;
  idx := Integer(FSuiteHeaderIdx[FSuiteHeaderIdx.Count -1]);
  if Not SkipTiming then
    S:= ' Time:'+ FormatDateTime(TimeFormat(ATiming), ATiming);
  S:=S+ ' N:'+ IntToStr(ANumRuns)+ ' E:'+ IntToStr(ANumErrors)+ ' F:'+ IntToStr(ANumFailures)+
    ' I:'+ IntToStr(ANumIgnores) ;
  FDoc[idx] := FDoc[idx]+S;
  FSuiteHeaderIdx.Delete(FSuiteHeaderIdx.Count -1);
end;

procedure TPlainResultsWriter.WriteSuiteHeader(ATestSuite: TTestSuite; ALevel: integer);
begin
  inherited;
  FDoc.Add(StringOfChar(' ',ALevel*2) + ATestSuite.TestName);
  FSuiteHeaderIdx.Add(Pointer(FDoc.Count - 1));
end;

function TestSuiteAsPlain(aSuite:TTestSuite; Options : TTestResultOptions = []): string;
var
  i: integer;
begin
  Result := '';
  for i := 0 to aSuite.Tests.Count - 1 do
    if TTest(aSuite.Tests.Items[i]) is TTestSuite then
      Result := Result + TestSuiteAsPlain(TTestSuite(aSuite.Tests.Items[i]),Options)
    else
      if TTest(aSuite.Tests.Items[i]) is TTestCase then
        Result := Result + '  ' + ASuite.TestName+'.' + TTestcase(aSuite.Tests.Items[i]).TestName + System.sLineBreak;
end;

function GetSuiteAsPlain(aSuite: TTestSuite; Options : TTestResultOptions = []): string;
begin
  Result := '';
  if aSuite <> nil then
    Result := 'TestSuites: ' + System.sLineBreak + TestSuiteAsPlain(aSuite,Options);
end;

function TestResultAsPlain(aTestResult: TTestResult; Options : TTestResultOptions = []): string;

  Procedure WriteFailure(F : TTestFailure; SkipAddress : Boolean = False );

  begin
    Result := Result + '    Message:           ' + f.AsString + System.sLineBreak;
    Result := Result + '    Exception class:   ' + f.ExceptionClassName + System.sLineBreak;
    if not (ttoSkipExceptionMessage in options) then
      Result := Result + '    Exception message: ' + f.ExceptionMessage + System.sLineBreak;
    if not (SkipAddress or (ttoSkipAddress in options) )then
      Result := Result + '        at ' + f.LocationInfo + System.sLineBreak;
  end;

var
  i: longint;
  f: TTestFailure;
begin
  with aTestResult do
  begin
    Result :=          'Number of run tests: ' + intToStr(RunTests) + System.sLineBreak;
    Result := Result + 'Number of errors:    ' + intToStr(NumberOfErrors) + System.sLineBreak;
    Result := Result + 'Number of failures:  ' + intToStr(NumberOfFailures);
    if NumberOfErrors <> 0 then
    begin
      Result := Result + System.sLineBreak;
      Result := Result + System.sLineBreak;
      Result := Result + 'List of errors:';
      for i := 0 to Errors.Count - 1 do
      begin
        Result := Result + System.sLineBreak;
        Result := Result + '  Error: ' + System.sLineBreak;
<<<<<<< HEAD
        WriteFailure(TTestFailure(Errors.Items[i]));
=======
        f := TTestFailure(Errors.Items[i]);
        Result := Result + '    Message:           ' + f.AsString + System.sLineBreak;
        Result := Result + '    Exception class:   ' + f.ExceptionClassName + System.sLineBreak;
        Result := Result + '    Exception message: ' + f.ExceptionMessage + System.sLineBreak;
        Result := Result + '        at ' + f.LocationInfo + System.sLineBreak;
>>>>>>> origin/fpcunit
      end;
    end;
    if NumberOfFailures <> 0 then
    begin
      Result := Result + System.sLineBreak;
      Result := Result + System.sLineBreak;
      Result := Result + 'List of failures:' + System.sLineBreak;
      for i := 0 to Failures.Count - 1 do
      begin
        Result := Result + '  Failure: ' + System.sLineBreak;
<<<<<<< HEAD
        WriteFailure(TTestFailure(Failures.Items[i]));
=======
        f := TTestFailure(Failures.Items[i]);
        Result := Result + '    Message:           ' + f.AsString + System.sLineBreak;
        Result := Result + '    Exception class:   ' + f.ExceptionClassName + System.sLineBreak;
        Result := Result + '    Exception message: ' + f.ExceptionMessage + System.sLineBreak;
        Result := Result + '        at ' + f.LocationInfo + System.sLineBreak;
>>>>>>> origin/fpcunit
      end;
    end;
   if NumberOfIgnoredTests <> 0 then
    begin
      Result := Result + System.sLineBreak;
      Result := Result + System.sLineBreak;
      Result := Result + 'List of ignored tests:' + System.sLineBreak;
      for i := 0 to IgnoredTests.Count - 1 do
      begin
        Result := Result + '  Ignored test: ' + System.sLineBreak;
        WriteFailure(TTestFailure(IgnoredTests.Items[i]),True);
      end;
    end;
  end;
  Result := Result + System.sLineBreak;
end;


end.
