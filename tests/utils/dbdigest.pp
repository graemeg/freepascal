{
    This file is part of the Free Pascal test suite.
    Copyright (c) 2002 by the Free Pascal development team.

    This program inserts the last tests run 
    into TESTSUITE database.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$mode objfpc}
{$h+}
{$ifndef win32}
  {$linklib pthread}
{$endif}

program dbdigest;

uses
  sysutils,teststr,testu,tresults,dbtests;


Var
  StatusCount : Array[TTestStatus] of Integer;
  UnknownLines : integer;


Procedure ExtractTestFileName(Var Line : string);

Var I : integer;

begin
  I:=Pos(' ',Line);
  If (I<>0) then
    Line:=Copy(Line,1,I-1);
end;

Function Analyse(Var Line : string; Var Status : TTestStatus) : Boolean;

Var
  TS : TTestStatus;

begin
  Result:=False;
  For TS:=FirstStatus to LastStatus do
    begin
    Result:=Pos(StatusText[TS],Line)=1;
    If Result then
      begin
      Status:=TS;
      Delete(Line,1,Length(StatusText[TS]));
      ExtractTestFileName(Line);
      Break;
      end;
    end;
end;

Type

TConfigOpt = (
  coDatabaseName,
  coHost,
  coUserName,
  coPassword,
  coPort,
  coLogFile,
  coLongLogFile,
  coOS,
  coCPU,
  coCategory,
  coVersion,
  coDate,
  coSubmitter,
  coMachine,
  coComment,
  coTestSrcDir,
  coRelSrcDir,
<<<<<<< HEAD
<<<<<<< HEAD
  coVerbose,
  coSQL
 );

{ Additional options only for dbdigest.cfg file }

TConfigAddOpt = (
  coCompilerDate,
  coCompilerFullVersion,
  coSvnCompilerRevision,
  coSvnTestsRevision,
  coSvnRTLRevision,
  coSvnPackagesRevision
 );

{ Additional options only for dbdigest.cfg file }

TConfigAddOpt = (
  coCompilerDate,
  coCompilerFullVersion,
  coSvnCompilerRevision,
  coSvnTestsRevision,
  coSvnRTLRevision,
  coSvnPackagesRevision
 );

{ Additional options only for dbdigest.cfg file }

TConfigAddOpt = (
  coCompilerDate,
  coCompilerFullVersion,
  coSvnCompilerRevision,
  coSvnTestsRevision,
  coSvnRTLRevision,
  coSvnPackagesRevision
 );

{ Additional options only for dbdigest.cfg file }

TConfigAddOpt = (
  coCompilerDate,
  coCompilerFullVersion,
  coSvnCompilerRevision,
  coSvnTestsRevision,
  coSvnRTLRevision,
  coSvnPackagesRevision
 );

{ Additional options only for dbdigest.cfg file }

TConfigAddOpt = (
  coCompilerDate,
  coCompilerFullVersion,
  coSvnCompilerRevision,
  coSvnTestsRevision,
  coSvnRTLRevision,
  coSvnPackagesRevision
=======
=======
>>>>>>> origin/fixes_2_2
  coVerbose
>>>>>>> graemeg/fixes_2_2
 );

Const

ConfigStrings : Array [TConfigOpt] of string = (
  'databasename',
  'host',
  'username',
  'password',
  'port',
  'logfile',
  'longlogfile',
  'os',
  'cpu',
  'category',
  'version',
  'date',
  'submitter',
  'machine',
  'comment',
  'testsrcdir',
  'relsrcdir',
<<<<<<< HEAD
<<<<<<< HEAD
  'verbose',
  'sql'
);

<<<<<<< HEAD
<<<<<<< HEAD
ConfigOpts : Array[TConfigOpt] of char =(
 'd', {  coDatabaseName }
 'h', {  coHost }
 'u', {  coUserName }
 'p', {  coPassword }
 'P', {  coPort }
 'l', {  coLogFile }
 'L', {  coLongLogFile }
 'o', {  coOS }
 'c', {  coCPU }
 'a', {  coCategory }
 'v', {  coVersion }
 't', {  coDate }
 's', {  coSubmitter }
 'm', {  coMachine }
 'C', {  coComment }
 'S', {  coTestSrcDir }
 'r', {  coRelSrcDir }
 'V', {  coVerbose }
 'Q'  {  coSQL }
=======
=======
>>>>>>> origin/fixes_2_2
  'verbose'
>>>>>>> graemeg/fixes_2_2
);

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
ConfigAddStrings : Array [TConfigAddOpt] of string = (
  'compilerdate',
  'compilerfullversion',
  'svncompilerrevision',
  'svntestsrevision',
  'svnrtlrevision',
  'svnpackagesrevision'
 );

ConfigAddCols : Array [TConfigAddOpt] of string = (
  'TU_COMPILERDATE',
  'TU_COMPILERFULLVERSION',
  'TU_SVNCOMPILERREVISION',
  'TU_SVNTESTSREVISION',
  'TU_SVNRTLREVISION',
  'TU_SVNPACKAGESREVISION'
 );

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
ConfigOpts : Array[TConfigOpt] of char
<<<<<<< HEAD
<<<<<<< HEAD
           = ('d','h','u','p','P','l','o','c','a','v','t','s','m','C','S','r','V');
>>>>>>> graemeg/cpstrnew
=======
ConfigOpts : Array[TConfigOpt] of char
           = ('d','h','u','p','P','l','o','c','a','v','t','s','m','C','S','r','V');
>>>>>>> graemeg/cpstrnew
=======
ConfigOpts : Array[TConfigOpt] of char
           = ('d','h','u','p','P','l','o','c','a','v','t','s','m','C','S','r','V');
>>>>>>> graemeg/cpstrnew
=======
ConfigOpts : Array[TConfigOpt] of char
           = ('d','h','u','p','P','l','o','c','a','v','t','s','m','C','S','r','V');
>>>>>>> origin/cpstrnew
=======
           = ('d','h','u','p','l','o','c','a','v','t','s','m','C','S','r','V');
>>>>>>> graemeg/fixes_2_2
=======
           = ('d','h','u','p','l','o','c','a','v','t','s','m','C','S','r','V');
>>>>>>> origin/fixes_2_2

Var
  TestOS,
  TestCPU,
  TestVersion,
  TestCategory,
  DatabaseName,
  HostName,
  UserName,
  Password,
  Port,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  LongLogFileName,
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  LogFileName,
  Submitter,
  Machine,
  Comment : String;
  TestDate : TDateTime;
  TestCompilerDate,
  TestCompilerFullVersion,
  TestSvnCompilerRevision,
  TestSvnTestsRevision,
  TestSvnRTLRevision,
  TestSvnPackagesRevision : String;

Procedure SetAddOpt (O : TConfigAddOpt; Value : string);
begin
  Case O of
    coCompilerDate:
      TestCompilerDate:=Value;
    coCompilerFullVersion:
      TestCompilerFullVersion:=Value;
    coSvnCompilerRevision:
      TestSvnCompilerRevision:=Value;
    coSvnTestsRevision:
      TestSvnTestsRevision:=Value;
    coSvnRTLRevision:
      TestSvnRTLRevision:=Value;
    coSvnPackagesRevision:
      TestSvnPackagesRevision:=Value;
  end;
end;

Procedure SetOpt (O : TConfigOpt; Value : string);
var
  year,month,day,min,hour : word;
begin
  Case O of
    coDatabaseName : DatabaseName:=Value;
    coHost         : HostName:=Value;
    coUserName     : UserName:=Value;
    coPassword     : Password:=Value;
    coPort         : Port:=Value;
    coLogFile      : LogFileName:=Value;
    coLongLogFile  : LongLogFileName:=Value;
    coOS           : TestOS:=Value;
    coCPU          : TestCPU:=Value;
    coCategory     : TestCategory:=Value;
    coVersion      : TestVersion:=Value;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    coSQL          : DoSQL:=True;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    coDate         :
      begin
        { Formated like YYYYMMDDhhmm }
	if Length(value)=12 then
	  begin
	    year:=StrToInt(Copy(value,1,4));
	    month:=StrToInt(Copy(value,5,2));
	    day:=StrToInt(Copy(Value,7,2));
	    hour:=StrToInt(Copy(Value,9,2));
	    min:=StrToInt(Copy(Value,11,2));
	    TestDate:=EncodeDate(year,month,day)+EncodeTime(hour,min,0,0);
	  end
	else
	  Verbose(V_Error,'Error in date format, use YYYYMMDDhhmm');
      end;
    coSubmitter    : Submitter:=Value;
    coMachine      : Machine:=Value;
    coComment      : Comment:=Value;
    coVerbose      : DoVerbose:=true;
    coTestSrcDir   :
      begin
        TestSrcDir:=Value;
	if (TestSrcDir<>'') and (TestSrcDir[length(TestSrcDir)]<>'/') then
	  TestSrcDir:=TestSrcDir+'/';
      end;
    coRelSrcDir   :
      begin
        RelSrcDir:=Value;
	if (RelSrcDir<>'') and (RelSrcDir[length(RelSrcDir)]<>'/') then
	  RelSrcDir:=RelSrcDir+'/';
	if (RelSrcDir<>'') and (RelSrcDir[1]='/') then
	  RelSrcDir:=copy(RelSrcDir,2,length(RelSrcDir)-1);
      end;
  end;
end;

Function ProcessOption(S: String) : Boolean;

Var
  N : String;
  I : Integer;
  co : TConfigOpt;
  coa : TConfigAddOpt;

begin
  Verbose(V_DEBUG,'Processing option: '+S);
  I:=Pos('=',S);
  Result:=(I<>0);
  If Result then
    begin
    N:=Copy(S,1,I-1);
    Delete(S,1,I);
    For co:=low(TConfigOpt) to high(TConfigOpt) do
      begin
      Result:=CompareText(ConfigStrings[co],N)=0;
      If Result then
        begin
          SetOpt(co,S);
          Exit;
        end;
      end;
    For coa:=low(TConfigAddOpt) to high(TConfigAddOpt) do
      begin
      Result:=CompareText(ConfigAddStrings[coa],N)=0;
      If Result then
        begin
          SetAddOpt(coa,S);
          Exit;
        end;
      end;
    end;
  Verbose(V_ERROR,'Unknown option : '+n+S);
end;

Procedure ProcessConfigfile(FN : String);

Var
  F : Text;
  S : String;
  I : Integer;

begin
  // Set the default value for old digests without RelSrcDir to the rtl/compiler
  // testsuite
  RelSrcDir:='tests/';
  If Not FileExists(FN) Then
    Exit;
  Verbose(V_DEBUG,'Parsing config file: '+FN);
  Assign(F,FN);
  {$i-}
  Reset(F);
  If IOResult<>0 then
    Exit;
  {$I+}
  While not(EOF(F)) do
    begin
    ReadLn(F,S);
    S:=trim(S);
    I:=Pos('#',S);
    If I<>0 then
      S:=Copy(S,1,I-1);
    If (S<>'') then
      ProcessOption(S);
    end;
  Close(F);
end;

Procedure ProcessCommandLine;

Var
  I : Integer;
  O : String;
  c,co : TConfigOpt;
  ShortOptFound, Found : Boolean;

begin
  I:=1;
  While I<=ParamCount do
    begin
    O:=Paramstr(I);
    ShortOptFound:=(Length(O)=2) and (O[1]='-');
    If ShortOptFound then
      For co:=low(TConfigOpt) to high(TConfigOpt) do
        begin
        Found:=(O[2]=ConfigOpts[co]);
        If Found then
          begin
          c:=co;
          Break;
          end;
        end;
    If not ShortOptFound then
      begin
        Found:=false;
        { accept long options }
        if (copy(O,1,2)='--') then
          begin
            { remove -- }
            O:=copy(O,3,length(O));
            For co:=low(TConfigOpt) to high(TConfigOpt) do
              begin
              Found:=(O=ConfigStrings[co]);
              If Found then
                begin
                c:=co;
                Break;
                end;
              end;
          end
      end;
    if not Found then
      Verbose(V_ERROR,'Illegal command-line option : '+O)
    else
      begin
      if c=coverbose then
        begin
          Found:=true;
          o:='';
        end
      else
        Found:=(I<ParamCount);
      If Not found then
        Verbose(V_ERROR,'Option requires argument : '+O)
      else
        begin
        inc(I);
        O:=Paramstr(I);
        SetOpt(c,o);
        end;
      end;
    Inc(I);
    end;
end;

Var
  TestCPUID : Integer;
  TestOSID  : Integer;
  TestVersionID  : Integer;
  TestCategoryID : Integer;
  TestRunID : Integer;
  ConfigID : Integer;

Procedure GetIDs;

begin
  TestCPUID := GetCPUId(TestCPU);
  If TestCPUID=-1 then
    Verbose(V_Error,'NO ID for CPU "'+TestCPU+'" found.');
  TestOSID  := GetOSID(TestOS);
  If TestOSID=-1 then
    Verbose(V_Error,'NO ID for OS "'+TestOS+'" found.');
  TestCategoryID := GetCategoryID(TestCategory);
  If TestCategoryID=-1 then
    begin
//    Verbose(V_Error,'NO ID for Category "'+TestCategory+'" found.');
    TestCategoryID:=1;
    end;
  TestVersionID  := GetVersionID(TestVersion);
  If TestVersionID=-1 then
    Verbose(V_Error,'NO ID for version "'+TestVersion+'" found.');
  If (Round(TestDate)=0) then
    Testdate:=Now;
  TestRunID:=GetRunID(TestOSID,TestCPUID,TestVersionID,TestDate);
  If (TestRunID=-1) then
    begin
    TestRunID:=AddRun(TestOSID,TestCPUID,TestVersionID,TestCategoryID,TestDate);
    If TestRUnID=-1 then
      Verbose(V_Error,'Could not insert new testrun record!');
    end
  else
    CleanTestRun(TestRunID);
end;


var
  LongLogFile : Text;
const
  UseLongLog : boolean = false;
  LongLogOpenCount : longint = 0;
  FirstLongLogLine : boolean = true;

Function GetContentsFromLongLog(Line : String) : String;
var
  S : String;
  IsFirst, IsFound : boolean;
begin
  Result:='';
  IsFirst:=true;
  IsFound:=false;
  While Not(EOF(LongLogFile)) do
    begin
      ReadLn(LongLogFile,S);
      if FirstLongLogLine then
        begin
          { At start of file there is a separation line }
          if (pos('>>>>>>>>>>>',S)=1) then
            Readln(LongLogFile,S);
          FirstLongLogLine:=false;
        end;
      if pos(Line,S)=1 then
        begin
          IsFound:=true;
          while not eof(LongLogFile) do
            begin
              ReadLn(LongLogFile,S);
              { End of file marker }
              if eof(LongLogFile) or (pos('>>>>>>>>>>>',S)=1) then
                exit;
              Result:=Result+S+LineEnding;
            end;
        end
      else if IsFirst then
        begin
          Verbose(V_Warning,'Line "'+Line+'" not found as next "'+S+'"');
          IsFirst:=false;
        end;
    end;
  if not IsFound then
    begin
      Verbose(V_Warning,'Line "'+Line+'" not found');
      { Restart to get a chance to find others }
      if eof(LongLogFile) then
        begin
          Close(LongLogFile);
          Reset(LongLogFile);
          inc(LongLogOpenCount);
        end;
    end;
end;

Function GetLog(Line, FN : String) : String;

begin
  if UseLongLog then
    begin
      Result:=GetContentsFromLongLog(Line);
      exit;
    end;
  FN:=ChangeFileExt(FN,'.log');
  If FileExists(FN) then
    Result:=GetFileContents(FN)
  else
    Result:='';
end;

Function GetExecuteLog(Line, FN : String) : String;

begin
  if UseLongLog then
    begin
      Result:=GetContentsFromLongLog(Line);
      exit;
    end;
  FN:=ChangeFileExt(FN,'.elg');
  If FileExists(FN) then
    Result:=GetFileContents(FN)
  else
    Result:='';
end;

Procedure Processfile (FN: String);

var
  logfile : text;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  fullline,line,prevLine : string;
=======
  line,prevLine : string;
>>>>>>> graemeg/cpstrnew
=======
  line,prevLine : string;
>>>>>>> graemeg/cpstrnew
=======
  line,prevLine : string;
>>>>>>> graemeg/cpstrnew
=======
  line,prevLine : string;
>>>>>>> origin/cpstrnew
  TS,PrevTS : TTestStatus;
  ID,PrevID : integer;
  Testlog : string;
  is_new : boolean;
begin
  Assign(logfile,FN);
  PrevId:=-1;
  PrevLine:='';
  is_new:=false;
  PrevTS:=low(TTestStatus);
{$i-}
  reset(logfile);
  if ioresult<>0 then
    Verbose(V_Error,'Unable to open log file'+FN);
{$i+}
  while not eof(logfile) do
    begin
    readln(logfile,line);
    fullline:=line;
    ts:=stFailedToCompile;
    If analyse(line,TS) then
      begin
      Verbose(V_NORMAL,'Analysing result for test '+Line);
      If Not ExpectRun[TS] then
        begin
        ID:=RequireTestID(Line);
        if (PrevID<>-1) and (PrevID<>ID) then
          begin
            { This can only happen if a Successfully compiled message
              is not followed by any other line about the same test }
            TestLog:='';
            AddTestResult(PrevID,TestRunId,ord(PrevTS),
              TestOK[PrevTS],TestSkipped[PrevTS],TestLog,is_new);
            Verbose(V_Warning,'Orphaned test: "'+prevline+'"');
          end;
        PrevID:=-1;
        If (ID<>-1) then
          begin
          If Not (TestOK[TS] or TestSkipped[TS]) then
            begin
              TestLog:=GetExecuteLog(Fullline,Line);
              if pos(failed_to_compile,TestLog)=1 then
<<<<<<< HEAD
                TestLog:=GetLog(Fullline,Line);
            end
          else
            TestLog:='';
          { AddTestResult can fail for test that contain %recompile
=======
                TestLog:=GetLog(Line);
            end
          else
            TestLog:='';
          { AddTestResult can fail for test that contain %recompile 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            as the same }
          if AddTestResult(ID,TestRunID,Ord(TS),TestOK[TS],
               TestSkipped[TS],TestLog,is_new) <> -1 then
            begin
              if is_new then
                Inc(StatusCount[TS])
              else
                Verbose(V_Debug,'Test: "'+line+'" was updated');
            end
          else
            begin
              Verbose(V_Warning,'Test: "'+line+'" already registered');
            end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

          end;
        end
      else
=======
=======
>>>>>>> graemeg/cpstrnew
              
          end;
        end
      else  
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
              
          end;
        end
      else  
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        begin
          Inc(StatusCount[TS]);
          PrevTS:=TS;
          PrevID:=RequireTestID(line);
          PrevLine:=line;
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
 
>>>>>>> graemeg/cpstrnew
=======
 
>>>>>>> graemeg/cpstrnew
=======
 
>>>>>>> graemeg/cpstrnew
=======
 
>>>>>>> origin/cpstrnew
      end
    else
      begin
        Inc(UnknownLines);
        Verbose(V_Warning,'Unknown line: "'+line+'"');
      end;
    end;
  close(logfile);
end;

procedure UpdateTestRun;

  var
     i : TTestStatus;
     qry : string;

  begin
    qry:='UPDATE TESTRUN SET ';
    for i:=low(TTestStatus) to high(TTestStatus) do
      qry:=qry+format('%s=%d, ',[SQLField[i],StatusCount[i]]);
    if TestCompilerDate<>'' then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      qry:=qry+format('%s=''%s'', ',[ConfigAddCols[coCompilerDate],EscapeSQL(TestCompilerDate)]);
    if TestCompilerFullVersion<>'' then
      qry:=qry+format('%s=''%s'', ',[ConfigAddCols[coCompilerFullVersion],EscapeSQL(TestCompilerFullVersion)]);
    if TestSvnCompilerRevision<>'' then
      qry:=qry+format('%s=''%s'', ',[ConfigAddCols[coSvnCompilerRevision],EscapeSQL(TestSvnCompilerRevision)]);
    if TestSvnTestsRevision<>'' then
      qry:=qry+format('%s=''%s'', ',[ConfigAddCols[coSvnTestsRevision],EscapeSQL(TestSvnTestsRevision)]);
    if TestSvnRTLRevision<>'' then
      qry:=qry+format('%s=''%s'', ',[ConfigAddCols[coSvnRTLRevision],EscapeSQL(TestSvnRTLRevision)]);
    if TestSvnPackagesRevision<>'' then
      qry:=qry+format('%s=''%s'', ',[ConfigAddCols[coSvnPackagesRevision],EscapeSQL(TestSvnPackagesRevision)]);

    qry:=qry+format('TU_SUBMITTER=''%s'', TU_MACHINE=''%s'', TU_COMMENT=''%s'', TU_DATE=''%s''',[Submitter,Machine,Comment,SqlDate(TestDate)]);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      qry:=qry+format('%s="%s", ',[ConfigAddCols[coCompilerDate],EscapeSQL(TestCompilerDate)]);
    if TestCompilerFullVersion<>'' then
      qry:=qry+format('%s="%s", ',[ConfigAddCols[coCompilerFullVersion],EscapeSQL(TestCompilerFullVersion)]);
    if TestSvnCompilerRevision<>'' then
      qry:=qry+format('%s="%s", ',[ConfigAddCols[coSvnCompilerRevision],EscapeSQL(TestSvnCompilerRevision)]);
    if TestSvnTestsRevision<>'' then
      qry:=qry+format('%s="%s", ',[ConfigAddCols[coSvnTestsRevision],EscapeSQL(TestSvnTestsRevision)]);
    if TestSvnRTLRevision<>'' then
      qry:=qry+format('%s="%s", ',[ConfigAddCols[coSvnRTLRevision],EscapeSQL(TestSvnRTLRevision)]);
    if TestSvnPackagesRevision<>'' then
      qry:=qry+format('%s="%s", ',[ConfigAddCols[coSvnPackagesRevision],EscapeSQL(TestSvnPackagesRevision)]);

    qry:=qry+format('TU_SUBMITTER="%s", TU_MACHINE="%s", TU_COMMENT="%s", TU_DATE="%s"',[Submitter,Machine,Comment,SqlDate(TestDate)]);
>>>>>>> graemeg/cpstrnew
    qry:=qry+' WHERE TU_ID='+format('%d',[TestRunID]);
    ExecuteQuery(Qry,False);
  end;

function GetTestConfigId : Integer;
var
  qry : string;
begin
  qry:='SELECT TCONF_ID FROM TESTCONFIG WHERE ' +
       'TCONF_CPU_FK=%d AND ' +
       'TCONF_OS_FK=%d AND ' +
       'TCONF_VERSION_FK=%d AND ' +
       'TCONF_CATEGORY_FK=%d AND ' +
       'TCONF_SUBMITTER=''%s'' AND ' +
       'TCONF_MACHINE=''%s'' AND ' +
       'TCONF_COMMENT=''%s'' ';
  ConfigID:=IDQuery(format(qry,[TestCPUID, TestOSID, TestVersionID, TestCategoryID,
                                Submitter, Machine, Comment]));
  GetTestConfigID:=ConfigID;
end;

function UpdateTestConfigID : boolean;
var
  qry : string;
  firstRunID, lastRunID,PrevRunID : Integer;
  RunCount : Integer;
  AddCount : boolean;

begin
  AddCount:=false;
  UpdateTestConfigID:=false;
  qry:=format('SELECT TCONF_FIRST_RUN_FK FROM TESTCONFIG WHERE TCONF_ID=%d',[ConfigID]);
  FirstRunID:=IDQuery(qry);
  if TestRunID<FirstRunID then
    begin
      Verbose(V_Warning,format('FirstRunID changed from %d to %d',[FirstRunID,TestRunID]));
      qry:=format('UPDATE TESTCONFIG SET TCONF_FIRST_RUN_FK=%d WHERE TCONF_ID=%d',
                  [TestRunID,ConfigID]);
      if Not ExecuteQuery(qry,False) then
        Verbose(V_Warning,'Update of LastRunID failed');
    end;
  qry:=format('SELECT TCONF_LAST_RUN_FK FROM TESTCONFIG WHERE TCONF_ID=%d',[ConfigID]);
  LastRunID:=IDQuery(qry);
  if TestRunID>LastRunID then
    begin
      qry:=format('UPDATE TESTCONFIG SET TCONF_LAST_RUN_FK=%d WHERE TCONF_ID=%d',
                  [TestRunID,ConfigID]);
      if not ExecuteQuery(qry,False) then
        Verbose(V_Warning,'Update of LastRunID failed');
    end
   else
    Verbose(V_Warning,format('LastRunID %di,new %d',[LastRunID,TestRunID]));
  qry:=format('SELECT TCONF_NEW_RUN_FK FROM TESTCONFIG WHERE TCONF_ID=%d',[ConfigID]);
  PrevRunID:=IDQuery(qry);
  if TestRunID<>PrevRunID then
    begin
    qry:=format('UPDATE TESTCONFIG SET TCONF_NEW_RUN_FK=%d WHERE TCONF_ID=%d',
                [TestRunID,ConfigID]);
    if not ExecuteQuery(qry,False) then
      Verbose(V_Warning,'Update of LastRunID failed');
    AddTestHistoryEntry(TestRunID,PrevRunID);
    AddCount:=true;
    end
  else
    Verbose(V_Warning,'TestRunID is equal to last!');
  qry:=format('SELECT TCONF_COUNT_RUNS FROM TESTCONFIG WHERE TCONF_ID=%d',[ConfigID]);
  RunCount:=IDQuery(qry);
  { Add one to run count }
  if AddCount then
    begin
      Inc(RunCount);
      qry:=format('UPDATE TESTCONFIG SET TCONF_COUNT_RUNS=%d WHERE TCONF_ID=%d',
                  [RunCount,ConfigID]);
      if not ExecuteQuery(qry,False) then
        Verbose(V_Warning,'Update of TU_COUNT_RUNS failed');
    end;
  UpdateTestConfigID:=true;
end;

function InsertNewTestConfigId : longint;
var
  qry : string;
begin
  qry:='INSERT INTO TESTCONFIG '+
        '(TCONF_NEW_RUN_FK,TCONF_FIRST_RUN_FK,TCONF_LAST_RUN_FK,' +
         'TCONF_CPU_FK,TCONF_OS_FK,TCONF_VERSION_FK,TCONF_CATEGORY_FK,'+
         'TCONF_SUBMITTER,TCONF_MACHINE,TCONF_COMMENT,'+
         'TCONF_NEW_DATE,TCONF_FIRST_DATE,TCONF_LAST_DATE) ';
  qry:=qry+format(' VALUES(%d,%d,%d,%d,%d,%d,%d,''%s'',''%s'',''%s'',''%s'',''%s'',''%s'') ',
                  [TestRunID, TestRunID, TestRunID, TestCPUID,
                   TestOSID, TestVersionID, TestCategoryID,
                   Submitter, Machine, Comment,
                   SqlDate(TestDate), SqlDate(TestDate), SqlDate(TestDate)]);
  qry:=qry+' RETURNING TCONF_ID';
  Result:=InsertQuery(qry);
  AddTestHistoryEntry(TestRunID,0);
end;

procedure UpdateTestConfig;

  begin
    if GetTestPreviousRunHistoryID(TestRunID) <> -1 then
      begin
      Verbose(V_DEBUG,format('TestRun %d already in TestHistory table',[TestRunID]));
      exit;
      end;

    if GetTestConfigID >= 0 then
      begin
        if not UpdateTestConfigID then
          Verbose(V_Warning, ' Update of TESTCONFIG table failed');
      end
    else
      begin
        if InsertNewTestConfigID = -1 then
          Verbose(V_Warning, ' Insert of new entry into TESTCONFIG table failed');
      end;
  end;


begin
  ProcessConfigFile('dbdigest.cfg');
  ProcessCommandLine;
  If LogFileName<>'' then
    begin
    ConnectToDatabase(DatabaseName,HostName,UserName,Password,Port);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    if LongLogFileName<>'' then
      begin
{$I-}
        Assign(LongLogFile,LongLogFileName);
        Reset(LongLogFile);
        If IOResult=0 then
          begin
            UseLongLog:=true;
            inc(LongLogOpenCount);
          end;
{$I+}
      end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    GetIDs;
    ProcessFile(LogFileName);
    UpdateTestRun;
    UpdateTestConfig;
    if UseLongLog then
      begin
        Close(LongLogFile);
        if LongLogOpenCount>1 then
          Verbose(V_Warning,format('LongLog file was read %d times.',[LongLogOpenCount]));
      end
    end
  else
    Verbose(V_ERROR,'Missing log file name');
end.
