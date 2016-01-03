{$mode objfpc}
{$H+}

unit dbtests;

Interface

Uses
<<<<<<< HEAD
  sqldb, testu;
=======
{$ifndef ver1_0}
  mysql4,
{$else}
  mysql,
{$endif}
  testu;
>>>>>>> graemeg/cpstrnew

{ ---------------------------------------------------------------------
  High-level access
  ---------------------------------------------------------------------}

Function GetTestID(Name : string) : Integer;
Function GetOSID(Name : String) : Integer;
Function GetCPUID(Name : String) : Integer;
Function GetCategoryID(Name : String) : Integer;
Function GetVersionID(Name : String) : Integer;
Function GetRunID(OSID, CPUID, VERSIONID : Integer; Date : TDateTime) : Integer;
Function AddRun(OSID, CPUID, VERSIONID, CATEGORYID : Integer; Date : TDateTime) : Integer;
Function AddTest(Name : String; AddSource : Boolean) : Integer;
Function UpdateTest(ID : Integer; Info : TConfig; Source : String) : Boolean;
Function AddTestResult(TestID,RunID,TestRes : Integer;
                       OK, Skipped : Boolean;
                       Log : String;var is_new : boolean) : Integer;
Function RequireTestID(Name : String): Integer;
Function CleanTestRun(ID : Integer) : Boolean;
function GetTestPreviousRunHistoryID(TestRunID : Integer) : Integer;
function GetTestNextRunHistoryID(TestRunID : Integer) : Integer;
function AddTestHistoryEntry(TestRunID,TestPreviousID : Integer) : boolean;

{ ---------------------------------------------------------------------
    Low-level DB access.
  ---------------------------------------------------------------------}

<<<<<<< HEAD
=======

Type
  TQueryResult = PMYSQL_RES;

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
Function  ConnectToDatabase(DatabaseName,Host,User,Password,Port : String) : Boolean;
Procedure DisconnectDatabase;
Function  InsertQuery(const Query : string) : Integer;
Function  ExecuteQuery (Qry : String; Silent : Boolean) : Boolean ;
Function  OpenQuery (Qry : String; Out Res : TSQLQuery; Silent : Boolean) : Boolean ;
Procedure FreeQueryResult (Var Res : TSQLQuery);
Function  GetResultField (Res : TSQLQuery; Id : Integer) : String;
Function  IDQuery(Qry : String) : Integer;
Function  StringQuery(Qry : String) : String;
Function  EscapeSQL( S : String) : String;
Function  SQLDate(D : TDateTime) : String;


var
  RelSrcDir,
  TestSrcDir : string;

Implementation

Uses
  SysUtils, pqconnection;

Var
  Connection : TPQConnection;

{ ---------------------------------------------------------------------
    Low-level DB access.
  ---------------------------------------------------------------------}

Function ConnectToDatabase(DatabaseName,Host,User,Password,Port : String) : Boolean;

begin
  Result:=False;
  Verbose(V_SQL,'Connection params : '+DatabaseName+' '+Host+' '+User+' '+Port);
  Connection:=TPQConnection.Create(Nil);
  try
    Connection.Hostname:=Host;
    Connection.DatabaseName:=DatabaseName;
    Connection.Username:=User;
    Connection.Password:=Password;
    Connection.Connected:=true;
    Connection.Transaction:=TSQLTransaction.Create(Connection);
    if (Port<>'') then
      Connection.Params.Values['Port']:=Port;
  except
    On E : Exception do
      begin
      Verbose(V_ERROR,'Failed to connect to database : '+E.Message);
      FreeAndNil(Connection);
      end;
  end;
end;

Procedure DisconnectDatabase;

<<<<<<< HEAD
begin
  FreeAndNil(Connection);
end;

Function CreateQuery(Const ASQL : String) : TSQLQuery;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result:=TSQLQuery.Create(Connection);
  Result.Database:=Connection;
  Result.Transaction:=Connection.Transaction;
  Result.SQL.Text:=ASQL;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Function ConnectToDatabase(DatabaseName,Host,User,Password,Port : String) : Boolean;

Var
  S : String;
  PortNb : longint;
  Error : word;
begin
  Verbose(V_DEBUG,'Connection params : '+DatabaseName+' '+Host+' '+User+' '+Password+' '+Port);
  if Port<>'' then
    begin
      Val(Port,PortNb,Error);
      if Error<>0 then
        PortNb:=0;
    end;
{$ifdef ver1_0}
  Result:=mysql_connect(@Connection,PChar(Host),PChar(User),PChar(Password))<>Nil;
{$else}
  mysql_init(@Connection);
  Result:=mysql_real_connect(@Connection,PChar(Host),PChar(User),PChar(Password),Nil,PortNb,Nil,0)<>Nil;
{$endif}
  If Not Result then
    begin
    S:=Strpas(mysql_error(@connection));
    Verbose(V_ERROR,'Failed to connect to database : '+S);
    end
  else
    begin
    Result:=Mysql_select_db(@Connection,Pchar(DatabaseName))>=0;
    If Not result then
      begin
      S:=StrPas(mysql_error(@connection));
      DisconnectDatabase;
      Verbose(V_Error,'Failed to select database : '+S);
      end;
    end;
>>>>>>> graemeg/cpstrnew
end;



Function ExecuteQuery (Qry : String; Silent : Boolean) : Boolean ;

begin
  Verbose(V_SQL,'Executing query:'+Qry);
  Result:=False;
  try
    With CreateQuery(Qry) do
      try
        ExecSQL;
        Result:=True;
        (Transaction as TSQLTransaction).Commit;
      finally
        Free;
      end;
  except
    On E : exception do
      begin
      Connection.Transaction.RollBack;
      if not Silent then
        Verbose(V_WARNING,'Query : '+Qry+'Failed : '+E.Message);
      end;
  end;
end;

Function OpenQuery (Qry : String; Out res : TSQLQuery; Silent : Boolean) : Boolean ;

begin
<<<<<<< HEAD
  Result:=False;
  Verbose(V_SQL,'Running query:'+Qry);
  Res:=CreateQuery(Qry);
  try
    Res.Open;
    Result:=True;
  except
    On E : exception do
      begin
      FreeAndNil(Res);
      Try
        Connection.Transaction.RollBack;
      except
      end;
      if not Silent then
        Verbose(V_WARNING,'Query : '+Qry+'Failed : '+E.Message);
      end;
  end;
end;

<<<<<<< HEAD
Function GetResultField (Res : TSQLQuery; Id : Integer) : String;
=======
{ No warning if it fails }
Function RunSilentQuery (Qry : String; Var res : TQueryResult) : Boolean ;

begin
  Verbose(V_DEBUG,'Running silent query:'+Qry);
=======
  Verbose(V_DEBUG,'Running query:'+Qry);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Result:=mysql_query(@Connection,PChar(qry))=0;
  If Not Result then
    Verbose(V_DEBUG,'Silent query : '+Qry+'Failed : '+Strpas(mysql_error(@connection)))
  else
    Res:=Mysql_store_result(@connection);
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
{ No warning if it fails }
Function RunSilentQuery (Qry : String; Var res : TQueryResult) : Boolean ;

begin
  Verbose(V_DEBUG,'Running silent query:'+Qry);
  Result:=mysql_query(@Connection,PChar(qry))=0;
  If Not Result then
    Verbose(V_DEBUG,'Silent query : '+Qry+'Failed : '+Strpas(mysql_error(@connection)))
  else
    Res:=Mysql_store_result(@connection);
end;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
{ No warning if it fails }
Function RunSilentQuery (Qry : String; Var res : TQueryResult) : Boolean ;

begin
  Verbose(V_DEBUG,'Running silent query:'+Qry);
  Result:=mysql_query(@Connection,PChar(qry))=0;
  If Not Result then
    Verbose(V_DEBUG,'Silent query : '+Qry+'Failed : '+Strpas(mysql_error(@connection)))
  else
    Res:=Mysql_store_result(@connection);
end;

>>>>>>> origin/cpstrnew

Function GetResultField (Res : TQueryResult; Id : Integer) : String;
>>>>>>> graemeg/cpstrnew


begin
  If (Res=Nil) or (ID>=Res.Fields.Count) then
    Result:=''
  else
    Result:=Res.Fields[ID].AsString;
  Verbose(V_SQL,'Field value '+Result);
end;

Procedure FreeQueryResult(var Res : TSQLQuery);

begin
  if Assigned(Res) and Assigned(Res.Transaction) then
    (Res.Transaction as TSQLTransaction).Commit;
  FreeAndNil(Res);
end;

Function IDQuery(Qry : String) : Integer;

Var
  Res : TSQLQuery;

begin
  Result:=-1;
  If OpenQuery(Qry,Res,False) then
    try
      Result:=StrToIntDef(GetResultField(Res,0),-1);
    finally
      FreeQueryResult(Res);
    end;
end;

Function StringQuery(Qry : String) : String;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Var
  Res : TSQLQuery;

begin
  Result:='';
  If OpenQuery(Qry,Res,False) then
    try
      Result:=GetResultField(Res,0);
    finally
      FreeQueryResult(Res);
    end;
end;

Function EscapeSQL( S : String) : String;

begin
//  Result:=StringReplace(S,'\','\\',[rfReplaceAll]);
  Result:=StringReplace(S,'''','''''',[rfReplaceAll]);
  Verbose(V_SQL,'EscapeSQL : "'+S+'" -> "'+Result+'"');
=======
begin
=======
begin
>>>>>>> graemeg/cpstrnew
=======
begin
>>>>>>> graemeg/cpstrnew
=======
begin
>>>>>>> origin/cpstrnew
  Result:=StringReplace(S,'\','\\',[rfReplaceAll]);
  Result:=StringReplace(Result,'"','\"',[rfReplaceAll]);
  Verbose(V_DEBUG,'EscapeSQL : "'+S+'" -> "'+Result+'"');
>>>>>>> graemeg/cpstrnew
end;


Function SQLDate(D : TDateTime) : String;

begin
  Result:=FormatDateTime('YYYY/MM/DD hh:nn:ss',D);
end;

{ ---------------------------------------------------------------------
  High-level access
  ---------------------------------------------------------------------}


Function GetTestID(Name : string) : Integer;

Const
  SFromName = 'SELECT T_ID FROM TESTS WHERE (T_NAME=''%s'')';

begin
  Result:=IDQuery(Format(SFromName,[Name]));
end;

Function GetOSID(Name : String) : Integer;

Const
  SFromName = 'SELECT TO_ID FROM TESTOS WHERE (TO_NAME=''%s'')';

begin
  Result:=IDQuery(Format(SFromName,[Name]));
end;

Function GetVersionID(Name : String) : Integer;

Const
  SFromName = 'SELECT TV_ID FROM TESTVERSION WHERE (TV_VERSION=''%s'')';

begin
  Result:=IDQuery(Format(SFromName,[Name]));
end;

Function GetCPUID(Name : String) : Integer;

Const
  SFromName = 'SELECT TC_ID FROM TESTCPU WHERE (TC_NAME=''%s'')';

begin
  Result:=IDQuery(Format(SFromName,[Name]));
end;

Function GetCategoryID(Name : String) : Integer;

Const
<<<<<<< HEAD
<<<<<<< HEAD
  SFromName = 'SELECT TCAT_ID FROM TESTCATEGORY WHERE (TCAT_NAME=''%s'')';
=======
  SFromName = 'SELECT TCAT_ID FROM TESTCATEGORY WHERE (TCAT_NAME="%s")';
>>>>>>> graemeg/fixes_2_2
=======
  SFromName = 'SELECT TCAT_ID FROM TESTCATEGORY WHERE (TCAT_NAME="%s")';
>>>>>>> origin/fixes_2_2

begin
  Result:=IDQuery(Format(SFromName,[Name]));
end;

Function GetRunID(OSID, CPUID, VERSIONID : Integer; Date : TDateTime) : Integer;


Const
  SFromIDS = 'SELECT TU_ID FROM TESTRUN WHERE '+
             ' (TU_OS_FK=%d) '+
             ' AND (TU_CPU_FK=%d) '+
             ' AND (TU_VERSION_FK=%d) '+
             ' AND (TU_DATE=''%s'')';

begin
  Result:=IDQuery(Format(SFromIDS,[OSID,CPUID,VERSIONID,SQLDate(Date)]));
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function InsertQuery(const Query : string) : Integer;

begin
  Result:=IDQuery(Query);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Function AddRun(OSID, CPUID, VERSIONID, CATEGORYID : Integer; Date : TDateTime) : Integer;

Const
  SInsertRun = 'INSERT INTO TESTRUN '+
               '(TU_OS_FK,TU_CPU_FK,TU_VERSION_FK,TU_CATEGORY_FK,TU_DATE)'+
               ' VALUES '+
<<<<<<< HEAD
<<<<<<< HEAD
               '(%d,%d,%d,%d,''%s'') RETURNING TU_ID';
var
  Qry : string;
begin
  qry:=Format(SInsertRun,[OSID,CPUID,VERSIONID,CATEGORYID,SQLDate(Date)]);
  Result:=IDQuery(Qry);
end;
=======
               '(%d,%d,%d,%d,"%s")';
>>>>>>> graemeg/fixes_2_2
=======
               '(%d,%d,%d,%d,"%s")';
>>>>>>> origin/fixes_2_2

function posr(c : Char; const s : AnsiString) : integer;
var
  i : integer;
begin
  i := length(s);
  while (i>0) and (s[i] <> c) do dec(i);
  Result := i;
end;

function GetUnitTestConfig(const fn : string; var r : TConfig) : Boolean;
var
  Path       : string;
  ClassName  : string;
  MethodName : string;
  slashpos   : integer;
  FileName   : string;
  s          : string;
  t          : text;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result := False;
  FillChar(r,sizeof(r),0);
  if pos('.',fn) > 0 then exit; // This is normally not a unit-test
  slashpos := posr('/',fn);
  if slashpos < 1 then exit;
  MethodName := copy(fn,slashpos+1,length(fn));
  Path := copy(fn,1,slashpos-1);
  slashpos := posr('/',Path);
  if slashpos > 0 then
    begin
    ClassName := copy(Path,slashpos+1,length(Path));
    Path := copy(Path,1,slashpos-1);
    end
=======
=======
>>>>>>> origin/fixes_2_2
  If RunQuery(Format(SInsertRun,[OSID,CPUID,VERSIONID,CATEGORYID,SQLDate(Date)]),Res) then
    Result:=mysql_insert_id(@connection)
>>>>>>> graemeg/fixes_2_2
  else
    begin
    ClassName := Path;
    path := '.';
    end;
  if upper(ClassName[1])<>'T' then exit;
  FileName := TestSrcDir+RelSrcDir+Path+DirectorySeparator+copy(lowercase(ClassName),2,length(classname));
  if FileExists(FileName+'.pas') then
    FileName := FileName + '.pas'
  else if FileExists(FileName+'.pp') then
    FileName := FileName + '.pp'
  else exit;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Verbose(V_Debug,'Reading: '+FileName);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  Verbose(V_Debug,'Reading '+FileName);
>>>>>>> graemeg/cpstrnew
  assign(t,FileName);
  {$I-}
   reset(t);
  {$I+}
  if ioresult<>0 then
   begin
     Verbose(V_Error,'Can''t open '+FileName);
     exit;
   end;
  while not eof(t) do
   begin
     readln(t,s);

     if s<>'' then
      begin
        TrimB(s);
        if SameText(copy(s,1,9),'PROCEDURE') then
         begin
           if pos(';',s)>11 then
            begin
              s := copy(s,11,pos(';',s)-11);
              TrimB(s);
              if SameText(s,ClassName+'.'+MethodName) then
               begin
                 Result := True;
                 r.Note:= 'unittest';
               end;
            end;
         end;
      end;
   end;
  close(t);
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
function posr(c : Char; const s : AnsiString) : integer;
var
  i : integer;
begin
  i := length(s);
  while (i>0) and (s[i] <> c) do dec(i);
  Result := i;
end;

function GetUnitTestConfig(const fn : string; var r : TConfig) : Boolean;
var
  Path       : string;
  ClassName  : string;
  MethodName : string;
  slashpos   : integer;
  FileName   : string;
  s          : string;
  t          : text;
begin
  Result := False;
  FillChar(r,sizeof(r),0);
  if pos('.',fn) > 0 then exit; // This is normally not a unit-test
  slashpos := posr('/',fn);
  if slashpos < 1 then exit;
  MethodName := copy(fn,slashpos+1,length(fn));
  Path := copy(fn,1,slashpos-1);
  slashpos := posr('/',Path);
  if slashpos > 0 then
    begin
    ClassName := copy(Path,slashpos+1,length(Path));
    Path := copy(Path,1,slashpos-1);
    end
  else
    begin
    ClassName := Path;
    path := '.';
    end;
  if upper(ClassName[1])<>'T' then exit;
  FileName := TestSrcDir+RelSrcDir+Path+DirectorySeparator+copy(lowercase(ClassName),2,length(classname));
  if FileExists(FileName+'.pas') then
    FileName := FileName + '.pas'
  else if FileExists(FileName+'.pp') then
    FileName := FileName + '.pp'
  else exit;
  
  Verbose(V_Debug,'Reading '+FileName);
  assign(t,FileName);
  {$I-}
   reset(t);
  {$I+}
  if ioresult<>0 then
   begin
     Verbose(V_Error,'Can''t open '+FileName);
     exit;
   end;
  while not eof(t) do
   begin
     readln(t,s);

     if s<>'' then
      begin
        TrimB(s);
        if SameText(copy(s,1,9),'PROCEDURE') then
         begin
           if pos(';',s)>11 then
            begin
              s := copy(s,11,pos(';',s)-11);
              TrimB(s);
              if SameText(s,ClassName+'.'+MethodName) then
               begin
                 Result := True;
                 r.Note:= 'unittest';
               end;
            end;
         end;
      end;
   end;
  close(t);
end;

>>>>>>> graemeg/fixes_2_2
Function AddTest(Name : String; AddSource : Boolean) : Integer;

Const
  SInsertTest = 'INSERT INTO TESTS (T_NAME,T_ADDDATE)'+
                ' VALUES (''%s'',NOW())';

Var
  Info : TConfig;

begin
  Result:=-1;
  If (FileExists(TestSrcDir+RelSrcDir+Name) and
     GetConfig(TestSrcDir+RelSrcDir+Name,Info)) or
     GetUnitTestConfig(Name,Info) then
    begin
    If ExecuteQuery(Format(SInsertTest,[Name]),False) then
      begin
      Result:=GetTestID(Name);
      If Result=-1 then
        Verbose(V_WARNING,'Could not find newly added test!')
      else
        If AddSource then
          UpdateTest(Result,Info,GetFileContents(Name))
        else
          UpdateTest(Result,Info,'');
      end
    end
  else
    Verbose(V_ERROR,'Could not find test "'+Name+'" or info about this test.');
end;

Const
  B : Array[Boolean] of String = ('f','t');

Function UpdateTest(ID : Integer; Info : TConfig; Source : String) : Boolean;

Const
  SUpdateTest = 'Update TESTS SET '+
                ' T_CPU=''%s'', T_OS=''%s'', T_VERSION=''%s'','+
                ' T_GRAPH=''%s'', T_INTERACTIVE=''%s'', T_RESULT=%d,'+
                ' T_FAIL=''%s'', T_RECOMPILE=''%s'', T_NORUN=''%s'','+
                ' T_NEEDLIBRARY=''%s'', T_KNOWNRUNERROR=%d,'+
                ' T_KNOWN=''%s'', T_NOTE=''%s'', T_OPTS = ''%s'''+
                ' %s '+
                'WHERE'+
                ' T_ID=%d';


Var
  Qry : String;

begin
  If Source<>'' then
    begin
    Source:=EscapeSQL(Source);
    Source:=', T_SOURCE='''+Source+'''';
    end;
  With Info do
    Qry:=Format(SUpdateTest,[EscapeSQL(NeedCPU),'',EscapeSQL(MinVersion),
                             B[usesGraph],B[IsInteractive],ResultCode,
                             B[ShouldFail],B[NeedRecompile],B[NoRun],
                             B[NeedLibrary],KnownRunError,
                             B[IsKnownCompileError],EscapeSQL(Note),EscapeSQL(NeedOptions),
                             Source,
                             ID
     ]);
  Result:=ExecuteQuery(Qry,False);
end;

Function AddTestResult(TestID,RunID,TestRes : Integer;
                       OK, Skipped : Boolean;
                       Log : String;var is_new : boolean) : Integer;

Const
  SInsertRes='Insert into TESTRESULTS '+
             '(TR_TEST_FK,TR_TESTRUN_FK,TR_OK,TR_SKIP,TR_RESULT) '+
             ' VALUES '+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
             '(%d,%d,''%s'',''%s'',%d) RETURNING TR_ID';
  SSelectId='SELECT TR_ID FROM TESTRESULTS WHERE (TR_TEST_FK=%d) '+
            ' AND (TR_TESTRUN_FK=%d)';
  SInsertLog='Update TESTRESULTS SET TR_LOG=''%s'''+
             ',TR_OK=''%s'',TR_SKIP=''%s'',TR_RESULT=%d WHERE (TR_ID=%d)';
Var
  Qry : String;
  updateValues : boolean;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
             '(%d,%d,"%s","%s",%d) ';
  SSelectId='SELECT TR_ID FROM TESTRESULTS WHERE (TR_TEST_FK=%d) '+
            ' AND (TR_TESTRUN_FK=%d)';
  SInsertLog='Update TESTRESULTS SET TR_LOG="%s"'+
             ',TR_OK="%s",TR_SKIP="%s",TR_RESULT=%d WHERE (TR_ID=%d)';
Var
  Qry : String;
  Res : TQueryResult;
  updateValues : boolean;
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
begin
  updateValues:=false;
  Result:=-1;
  Qry:=Format(SInsertRes,
              [TestID,RunID,B[OK],B[Skipped],TestRes,EscapeSQL(Log)]);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=IDQuery(Qry);
  if (Result=-1) then
    begin
    Qry:=format(SSelectId,[TestId,RunId]);
    Result:=IDQuery(Qry);
    if Result<>-1 then
      UpdateValues:=true;
    end;
  if (Result<>-1) and ((Log<>'') or updateValues) then
    begin
    Qry:=Format(SInsertLog,[EscapeSQL(Log),B[OK],B[Skipped],TestRes,Result]);
    if Not ExecuteQuery(Qry,False) then
       Verbose(V_Warning,'Insert Log failed');
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  If RunSilentQuery(Qry,Res) then
    Result:=mysql_insert_id(@connection)
  else
    begin
      Qry:=format(SSelectId,[TestId,RunId]);
      Result:=IDQuery(Qry);
      if Result<>-1 then
        updateValues:=true;
    end;
  if (Result<>-1) and ((Log<>'') or updateValues) then
    begin
      Qry:=format(SInsertLog,[EscapeSQL(Log),B[OK],B[Skipped],TestRes,Result]);
      if not RunQuery(Qry,Res) then
        begin
          Verbose(V_Warning,'Insert Log failed');
        end;
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
    end;
  { If test already existed, return false for is_new to avoid double counting }
  is_new:=not updateValues;
end;

Function RequireTestID(Name : String): Integer;

begin
  Result:=GetTestID(Name);
  If Result=-1 then
    Result:=AddTest(Name,FileExists(Name));
  If Result=-1 then
    Verbose(V_WARNING,'Could not find or create entry for test '+Name);
end;

Function CleanTestRun(ID : Integer) : Boolean;

Const
  SDeleteRun = 'DELETE FROM TESTRESULTS WHERE TR_TESTRUN_FK=%d';

begin
  Result:=ExecuteQuery(Format(SDeleteRun,[ID]),False);
end;

function GetTestPreviousRunHistoryID(TestRunID : Integer) : Integer;
begin
  GetTestPreviousRunHistoryID:=IDQuery(
    format('SELECT TH_PREVIOUS_FK FROM TESTRUNHISTORY WHERE TH_ID_FK=%d',[TestRunID]));
end;

function GetTestNextRunHistoryID(TestRunID : Integer) : Integer;
begin
  GetTestNextRunHistoryID:=IDQuery(
    format('SELECT TH_ID_FK FROM TESTRUNHISTORY WHERE TH_PREVIOUS_FK=%d',[TestRunID]));
end;

function AddTestHistoryEntry(TestRunID,TestPreviousID : Integer) : boolean;

var
  qry : string;

begin
  Qry:=format('INSERT INTO TESTRUNHISTORY (TH_ID_FK,TH_PREVIOUS_FK) '+
              ' VALUES (%d,%d)',[TestRunID,TestPreviousID]);
  Result:=ExecuteQuery(Qry,False);
end;

end.
