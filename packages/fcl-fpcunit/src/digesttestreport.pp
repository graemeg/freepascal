unit DigestTestReport;

{$mode objfpc}{$H+}

interface

uses
  classes, SysUtils, fpcunit, fpcunitreport, testutils{, tresults};

{ ---------------------------------------------------------------------------- }
{ This section is copy-pasted from the tresults unit of the testsuite          }
{ because it is not possible to add a dependency on the testsuite              }
{ ---------------------------------------------------------------------------- }

const
  failed_to_compile = 'Failed to compile ';
  success_compilation_failed = 'Success, compilation failed ';
  failed_compilation_successful = 'Failed, compilation successful ';
  successfully_compiled = 'Successfully compiled ';
  failed_to_run = 'Failed to run ';
  successfully_run = 'Successfully run ';
  skipping_graph_test = 'Skipping test because it uses graph ';
  skipping_interactive_test = 'Skipping test because it is interactive ';
  skipping_known_bug = 'Skipping test because it is a known bug ';
  skipping_compiler_version_too_low = 'Skipping test because compiler version too low ';
  skipping_compiler_version_too_high = 'Skipping test because compiler version too high ';
  skipping_other_cpu = 'Skipping test because for other cpu ';
  skipping_other_target = 'Skipping test because for other target ';
  skipping_run_unit = 'Skipping test run because it is a unit ';
  skipping_run_test = 'Skipping run test ';
  known_problem = ' known problem: ';
  line_separation = '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>';

  ResLogfile  : string[32] = 'log';

Type
  TTestStatus = (
  stFailedToCompile,
  stSuccessCompilationFailed,
  stFailedCompilationsuccessful,
  stSuccessfullyCompiled,
  stFailedToRun,
  stKnownRunProblem,
  stSuccessFullyRun,
  stSkippingGraphTest,
  stSkippingInteractiveTest,
  stSkippingKnownBug,
  stSkippingCompilerVersionTooLow,
  stSkippingCompilerVersionTooHigh,
  stSkippingOtherCpu,
  stSkippingOtherTarget,
  stskippingRunUnit,
  stskippingRunTest
  );

const
  StatusText : Array[TTestStatus] of String = (
    failed_to_compile,
    success_compilation_failed,
    failed_compilation_successful ,
    successfully_compiled ,
    failed_to_run ,
    known_problem ,
    successfully_run ,
    skipping_graph_test ,
    skipping_interactive_test ,
    skipping_known_bug ,
    skipping_compiler_version_too_low,
    skipping_compiler_version_too_high,
    skipping_other_cpu ,
    skipping_other_target ,
    skipping_run_unit ,
    skipping_run_test
  );

{ ---------------------------------------------------------------------------- }
{ End of the code from tresults from the testsuite                             }
{ ---------------------------------------------------------------------------- }

type

  { TDigestResultsWriter }

  TDigestResultsWriter = class(TCustomResultsWriter)
  private
    FTestResult : TTestStatus;
    FOutputDir : String;
    FHostName : String;
    FComment : String;
    FCategory : String;
    FRelSrcDir: string;
    procedure CreateTar;
  public
  {ITestListener}
    procedure AddFailure(ATest: TTest; AFailure: TTestFailure); override;
    procedure AddError(ATest: TTest; AError: TTestFailure); override;
    procedure StartTest(ATest: TTest); override;
    procedure EndTest(ATest: TTest); override;
    procedure StartTestSuite(ATestSuite: TTestSuite); override;
    procedure EndTestSuite(ATestSuite: TTestSuite); override;
    
    property Comment: string read FComment write FComment;
    property Category: string read FCategory write FCategory;
    property RelSrcDir: string read FRelSrcDir write FRelSrcDir;
  end;
  
implementation

uses LibTar,
{$IFDEF UNIX}
<<<<<<< HEAD
<<<<<<< HEAD
     UnixType,Unix,BaseUnix,
=======
     Unix,BaseUnix,
>>>>>>> graemeg/fixes_2_2
=======
     Unix,BaseUnix,
>>>>>>> origin/fixes_2_2
{$ENDIF}
{$IFDEF MSWINDOWS}
     windows,
{$ENDIF}
     zstream;
     
Function PathExists (Const F : String) : Boolean;
{
  Returns True if the file exists, False if not.
}
Var
  info : Tsearchrec;
begin
  PathExists:=(FindFirst (F,faAnyFile,Info)=0) and  ((Info.Attr and faDirectory)=faDirectory);
  sysutils.FindClose (Info);
end;

function CompilerFullTarget:string;
begin
  CompilerFullTarget:={$I %FPCTARGETCPU%}+'-'+{$I %FPCTARGETOS%};
end;

procedure AddLog(const logfile,s:string);
var
  t : text;
begin
  assign(t,logfile);
<<<<<<< HEAD
<<<<<<< HEAD
  {$push}{$I-}
  append(t);
  {$pop}
  if ioresult<>0 then
    begin
    {$push}{$I-}
    rewrite(t);
    {$pop}
=======
=======
>>>>>>> origin/fixes_2_2
  {$I-}
  append(t);
  {$I+}
  if ioresult<>0 then
    begin
    {$I-}
    rewrite(t);
    {$I+}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    if ioresult<>0 then
      begin
      writeln('Can''t append to '+logfile);
      exit;
      end;
    end;
  writeln(t,s);
  close(t);
end;

function SplitPath(const s:string):string;
var
  i : longint;
begin
  i:=Length(s);
  while (i>0) and not(s[i] in ['/','\'{$IFDEF MACOS},':'{$ENDIF}]) do
   dec(i);
  SplitPath:=Copy(s,1,i);
end;

procedure mkdirtree(const s:string);
var
  hs : string;
begin
  if s='' then
    exit;
  if s[length(s)] in ['\','/'{$IFDEF MACOS},':'{$ENDIF}] then
    hs:=Copy(s,1,length(s)-1)
  else
    hs:=s;
  if not PathExists(hs) then
    begin
      { Try parent first }
      mkdirtree(SplitPath(hs));
      { make this dir }
<<<<<<< HEAD
<<<<<<< HEAD
      {$push}{$I-}
       mkdir(s);
      {$pop}
=======
      {$I-}
       mkdir(s);
      {$I+}
>>>>>>> graemeg/fixes_2_2
=======
      {$I-}
       mkdir(s);
      {$I+}
>>>>>>> origin/fixes_2_2
      ioresult;
    end;
end;

{ TDBResultsWriter }

procedure TDigestResultsWriter.CreateTar;

var TarWriter : TTarWriter;
    C : TGZFileStream;
    OldDir : String;
    TarFileName : String;
    CurrentDate : TDateTime;
{$IFDEF MSWINDOWS}
    CFileTime : TFileTime;
    CSystemTime : TSystemTime;
{$ENDIF}
{$IFDEF UNIX}
    TimeVal  : TTimeVal;
    TimeZone : TTimeZone;
{$ENDIF}

  procedure AddTree(Const ADir : String);

  var d : TSearchRec;

  begin
    if FindFirst(adir+'/*',faAnyFile,d)=0 Then
      begin
        repeat
          if (d.Attr and faDirectory)=faDirectory then
            begin
            if (d.Name<>'.') and (d.Name<>'..') then
              begin
              TarWriter.AddDir(ADir+'/'+d.Name, CurrentDate);
              AddTree(ADir+'/'+d.Name);
              end;
            end
          else if d.Name <> TarFileName then
            TarWriter.AddFile (ADir+'/'+ d.Name);
        until findnext(d)<>0;
        sysutils.Findclose(d);
      end;
  end;


begin
  TarFileName:= FHostName+'-'+FormatDateTime('yyyymmddhhmm',Now)+'.tar.gz';
  getdir(0,OldDir);
  Chdir(FOutputDir);
  
  C:=TGZFileStream.Create(TarFileName,gzOpenWrite);
  TarWriter := TTarWriter.Create (C);
  CurrentDate := Now;
{$IFDEF UNIX}
  fpGetTimeOfDay (@TimeVal, @TimeZone);
  CurrentDate := CurrentDate + TimeZone.tz_minuteswest / (60 * 24);
{$ENDIF}
{$IFDEF MSWINDOWS}
  DateTimeToSystemTime(CurrentDate,CSystemTime);
  SystemTimeToFileTime(CSystemTime,CFileTime);
  LocalFileTimeToFileTime(CFileTime,CFileTime);
  FileTimeToSystemTime(CFileTime,CSystemTime);
  CurrentDate:= SystemTimeToDateTime(CSystemTime);
{$ENDIF}
  AddTree('.');

  TarWriter.free;
  c.free;
  chdir(OldDir);
end;

procedure TDigestResultsWriter.AddFailure(ATest: TTest; AFailure: TTestFailure);
begin
  if AFailure.IsIgnoredTest then
    FTestResult := stskippingRunTest
  else
    FTestResult := stFailedToRun;
  AddLog(FOutputDir+'/'+ATest.TestSuiteName+ '/' + ATest.TestName+'.elg',AFailure.AsString);
end;

procedure TDigestResultsWriter.AddError(ATest: TTest; AError: TTestFailure);
begin
  if AError.IsIgnoredTest then
    FTestResult := stskippingRunTest
  else
    FTestResult := stFailedToRun;
  AddLog(FOutputDir+'/'+ATest.TestSuiteName+ '/' + ATest.TestName+'.elg',AError.AsString);
end;

procedure TDigestResultsWriter.StartTest(ATest: TTest);
begin
  AddLog(FOutputDir+'/'+'log',StatusText[stSuccessfullyCompiled]+ATest.TestSuiteName+ '/' + ATest.TestName);
  FTestResult := stSuccessFullyRun;
end;

procedure TDigestResultsWriter.EndTest(ATest: TTest);
begin
  AddLog(FOutputDir+'/'+'log',StatusText[FTestResult]+ATest.TestSuiteName+ '/' + ATest.TestName);
end;

procedure TDigestResultsWriter.StartTestSuite(ATestSuite: TTestSuite);
var OldDir : String;
begin
  if ATestSuite.TestName = '' then
    begin
{$ifndef MACOS}
    FOutputDir:='output/'+{$ifdef LIMIT83FS}CompilerTarget{$else}CompilerFullTarget{$endif};
{$else MACOS}
    FOutputDir:=':output:'+CompilerFullTarget;
{$endif MACOS}

    end
  else
    begin
    getdir(0,OldDir);
<<<<<<< HEAD
<<<<<<< HEAD
    {$push}{$I-}
    chdir(FOutputDir+'/'+ATestSuite.TestName);
    {$pop}
=======
    {$I-}
    chdir(FOutputDir+'/'+ATestSuite.TestName);
    {$I+}
>>>>>>> graemeg/fixes_2_2
=======
    {$I-}
    chdir(FOutputDir+'/'+ATestSuite.TestName);
    {$I+}
>>>>>>> origin/fixes_2_2
    if IOResult<>0 then
      begin
      mkdirtree(FOutputDir+'/'+ATestSuite.TestName);
      end
    else
      chdir(OldDir);
    end;
end;

procedure TDigestResultsWriter.EndTestSuite(ATestSuite: TTestSuite);
var DigestFileName : String;
    i              : byte;
begin
  if ATestSuite.TestName='' then
    begin
    DigestFileName:=FOutputDir+'/dbdigest.cfg';
    AddLog(DigestFileName,'OS='+{$I %FPCTARGETOS%});
    AddLog(DigestFileName,'CPU='+{$I %FPCTARGETCPU%});
    AddLog(DigestFileName,'Version='+{$I %FPCVERSION%});
    AddLog(DigestFileName,'LogFile=log');
    AddLog(DigestFileName,'Submitter='+sysutils.GetEnvironmentVariable('USER'));
    FHostName:=sysutils.GetEnvironmentVariable('HOSTNAME');
    if pos('.',FHostName)>0 then
      FHostName:=system.Copy(FHostName,1,pos('.',FHostName)-1);
    AddLog(DigestFileName,'Machine='+FHostName);
    AddLog(DigestFileName,'Comment='+FComment);
    AddLog(DigestFileName,'Category='+FCategory);
    AddLog(DigestFileName,'RelSrcDir='+FRelSrcDir);
// Create .tar.gz file
    CreateTar;
    end;
end;

end.

