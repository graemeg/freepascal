{$mode objfpc}
{$h+}

unit utests;

interface

uses
     cgiapp,
     sysutils,
     pqconnection,
     sqldb,whtml,dbwhtml,db,
     tresults,webutil,
     Classes,ftFont,fpimage,fpimgcanv,fpWritePng,fpcanvas;

const
  TestsuiteURLPrefix='http://www.freepascal.org/testsuite/';
  TestsuiteBin='testsuite.cgi';
  ViewURL='http://svn.freepascal.org/cgi-bin/viewvc.cgi/';
  ViewRevURL='http://svn.freepascal.org/cgi-bin/viewvc.cgi?view=revision&amp;revision=';
  TestsSubDir='/tests/';
  DataBaseSubDir='/packages/fcl-db/tests/';

var
  TestsuiteCGIURL : string;

Type

  { TTestSuite }

  TTestSuite = Class(TCgiApplication)
  Private
    FHTMLWriter : THtmlWriter;
    FComboBoxProducer : TComboBoxProducer;
    FDB : TSQLConnection;
    FTrans : TSQLTransaction;
    FRunID,
    FCompareRunID,
    FPreviousRunID,
    FNextRunID,
    FPrevious2RunID,
    FNext2RunID,
    FTestFileID,
    FTestFileName,
    FVersion,
    FVersionBranch,
    FCond,
    FSubmitter,
    FMachine,
    FComment,
    FCPU,
    FCategory,
    FOS  : String;
    FViewVCURL : String;
    FDate : TDateTime;
    FDebug,
    FListAll,
    FNoSkipped,
    FOnlyFailed : Boolean;
    FRunSkipCount,
    FRunFailedCount,
    FRunCount : Integer;
    FAction,
    FLimit : Integer;
    FTestLastDays : Integer;
    FNeedEnd : boolean;
    procedure DumpTestInfo(Q: TSQLQuery);
    Procedure GetOverviewRowAttr(Sender : TObject; Var BGColor : String;
                                   Var Align : THTMLAlign; Var VAlign : THTMLValign;
                                   Var CustomAttr : String) ;
    Procedure GetRunRowAttr(Sender : TObject; Var BGColor : String;
                            Var Align : THTMLAlign; Var VAlign : THTMLValign;
                            Var CustomAttr : String) ;
    Procedure FormatFailedOverview(Sender : TObject; Var CellData : String);
    Procedure FormatTestRunOverview(Sender : TObject; Var CellData : String);
    Procedure FormatFileDetails(Sender: TObject; var CellData: String);
    Procedure FormatFileIDDetails(Sender: TObject; var CellData: String);
    Procedure FormatTestResult(Sender: TObject; var CellData: String);
    Procedure FormatSVN(Sender: TObject; var CellData: String);
    Procedure FormatSVNData(var CellData: String);
    Function  FormatDetailURL(const RunIdStr, CellData : String) : string;

    Procedure DoDrawPie(Img : TFPCustomImage; Skipped,Failed,Total : Integer);
  Public
    Function CreateDataset(Qry : String) : TSQLQuery;
    Function CreateTableProducer(DS : TDataset) :TTableProducer;
    Procedure DefaultTableFromQuery(Qry,ALink : String; IncludeRecordCount : Boolean);
    Procedure ComboBoxFromQuery(Const ComboName,Qry : String);
    Procedure ComboBoxFromQuery(Const ComboName,Qry,Value : String);
    Function  GetSingleTon(Const Qry : String) : String;
    Function GetOSName(ID : String) : String;
    Function GetOSID(AName : String) : String;
    Function GetCPUName(ID : String) : String;
    Function GetCPUID(AName : String) : String;
    Function GetVersionName(ID : String) : String;
    Function GetCategoryName(ID : String) : String;
    Function GetTestFileName(ID : String) : String;
    Function GetPreviousRunID(RunID : String) : String;
    Function GetNextRunID(RunID : String) : String;
    Function GetFailCount(RunID : longint) : string;
    Function InitCGIVars : Integer;
    Procedure DoRun; override;
    Procedure EmitDocType;
    Procedure EmitOverviewForm;
    Procedure EmitHistoryForm;
    Procedure ShowRunResults;
    Procedure ShowRunComparison;
    Procedure ShowOneTest;
    Procedure ShowHistory;
    Function ConnectToDB : Boolean;
    procedure DisconnectFromDB;
    Procedure EmitTitle(ATitle : String);
    Procedure EmitEnd;
    Procedure ShowRunOverview;
    Procedure CreateRunPie;
    Function  ShowRunData : Boolean;
    Procedure LDump(Const St : String);
    Procedure LDumpLn(Const St : String);


  end;

implementation


  uses
    wformat,
    dateutils;

Const
{$i utests.cfg}

{ if utests.cfg is missed, create one with the following contents:
  DefDatabase = 'TESTSUITE';
  DefHost     = '';
  DefDBUser   = ''; // fill this in when compiling.
  DefPassword = ''; // fill this in, too.
}

Const
  OldTestResultsTableName = 'OLDTESTRESULTS';
  NewTestResultsTableName = 'TESTRESULTS';
  LastOldTestRun = 91178;
  MaxLimit = 1000;

const
  faction_show_overview = 0;
  faction_show_run_results = 1;
  faction_show_run_pie = 2;
  faction_show_one_test = 3;
  faction_show_history = 4;
  faction_compare_with_previous = 5;
  faction_compare_with_next = 6;
  faction_compare2_with_previous = 7;
  faction_compare2_with_next = 8;
  faction_compare_both_with_previous = 9;
  faction_compare_both_with_next = 10;


  Function TestResultsTableName(const RunId : String) : string;
  var
    RunIDVal : qword;
    Error : word;
  begin
    system.val (Trim(RunId),RunIdVal,error);
    if (error<>0) then
      result:='ErrorTable'
    else if (RunIdVal <= LastOldTestRun) then
      result:=OldTestResultsTableName
    else
      result:=NewTestResultsTableName;
  end;


Var
  SDetailsURL : string;

type
  known_versions = (
    ver_unknown,
    ver_1_0_10,
    ver_2_0_0,
    ver_2_0_1,
    ver_2_0_2,
    ver_2_0_3,
    ver_2_0_4,
    ver_2_0_5,
    ver_2_1_2,
    ver_2_1_4,
    ver_2_2_0,
    ver_2_2_1,
    ver_2_2_2,
    ver_2_2_3,
    ver_2_2_4,
    ver_2_2_5,
    ver_2_3_1,
    ver_2_4_0,
    ver_2_4_1,
    ver_2_4_2,
    ver_2_4_3,
    ver_2_4_4,
    ver_2_4_5,
    ver_2_5_1,
    ver_2_6_0,
    ver_2_6_1,
    ver_2_6_2,
    ver_2_6_3,
    ver_2_6_4,
    ver_2_6_5,
    ver_2_7_1,
    ver_3_0_0,
    ver_3_0_1,
    ver_3_0_2,
    ver_3_0_3,
    ver_3_0_4,
    ver_3_0_5,
    ver_3_1_1,
    ver_3_2_0,
    ver_3_2_1,
    ver_3_3_1);

const
  ver_trunk = high (known_versions);

const
  ver_string : array[known_versions] of string =
  (
   'unknown',
   '1.0.10',
   '2.0.0',
   '2.0.1',
   '2.0.2',
   '2.0.3',
   '2.0.4',
   '2.0.5',
   '2.1.2',
   '2.1.4',
   '2.2.0',
   '2.2.1',
   '2.2.2',
   '2.2.3',
   '2.2.4',
   '2.2.5',
   '2.3.1',
   '2.4.0',
   '2.4.1',
   '2.4.2',
   '2.4.3',
   '2.4.4',
   '2.4.5',
   '2.5.1',
   '2.6.0',
   '2.6.1',
   '2.6.2',
   '2.6.3',
   '2.6.4',
   '2.6.5',
   '2.7.1',
   '3.0.0',
   '3.0.1',
   '3.0.2',
   '3.0.3',
   '3.0.4',
   '3.0.5',
   '3.1.1',
   '3.2.0',
   '3.2.1',
   '3.3.1'
  );

  ver_branch : array [known_versions] of string =
  (
   '',
   '',
   'tags/release_2_0_0',
   'branches/fixes_2_0',
   'tags/release_2_0_2',
   'branches/fixes_2_0',
   'tags/release_2_0_4',
   'branches/fixes_2_0',
   'tags/release_2_1_2',
   'tags/release_2_1_4',
   'tags/release_2_2_0',
   'branches/fixes_2_2',
   'tags/release_2_2_2',
   'branches/fixes_2_2',
   'tags/release_2_2_4',
   'branches/fixes_2_2',
   'tags/release_2_4_0',
   'tags/release_2_4_0',
   'tags/release_2_4_2',
   'tags/release_2_4_2',
   'tags/release_2_4_4',
   'tags/release_2_4_4',
   'branches/fixes_2_4',
   'tags/release_2_6_0',
   'tags/release_2_6_0',
   'tags/release_2_6_2',
   'tags/release_2_6_2',
   'tags/release_2_6_4',
   'tags/release_2_6_4',
   'branches/fixes_2_6',
   'branches/release_3_0_0',
   'branches/release_3_0_0',
   'branches/release_3_0_2',
   'branches/release_3_0_2',
   'branches/release_3_0_4',
   'branches/release_3_0_4',
   'branches/fixes_3_0',
   'branches/fixes_3_2',
   'branches/fixes_3_2',
   'branches/fixes_3_2',
   'trunk'
  );

Procedure TTestSuite.DoRun;

begin
  Try
    Try
      Case InitCGIVars of
        faction_show_overview : EmitOverviewForm;
        faction_show_run_results :
          if Length(FCompareRunID) = 0 then
            ShowRunResults
          else
            ShowRunComparison;
        faction_show_run_pie : CreateRunPie;
        faction_show_one_test : ShowOneTest;
        faction_show_history : ShowHistory;
        faction_compare_with_previous : 
          begin
            FCompareRunID:=FRunID;
            FRunID:=FPreviousRunID;
            ShowRunComparison;
          end;
        faction_compare_with_next : 
          begin
            FCompareRunID:=FNextRunID;
            ShowRunComparison;
          end;
        faction_compare2_with_previous : 
          begin
            FRunID:=FPrevious2RunID;
            ShowRunComparison;
          end;
        faction_compare2_with_next : 
          begin
            FRunID:=FCompareRunID;
            FCompareRunID:=FNext2RunID;
            ShowRunComparison;
          end;
        faction_compare_both_with_previous : 
          begin
            FRunID:=FPreviousRunID;
            FCompareRunID:=FPrevious2RunID;
            ShowRunComparison;
          end;
        faction_compare_both_with_next : 
          begin
            FRunID:=FNextRunID;
            FCompareRunID:=FNext2RunID;
            ShowRunComparison;
          end;
{$ifdef TEST}
        98 :
          begin
            ///EmitOverviewForm;
            system.Writeln(stdout,'<PRE>');
            system.Writeln(stdout,'paramstr(0) is ',paramstr(0));
            system.FreeMem(pointer($ffffffff));
            system.Writeln(stdout,'</PRE>');
            system.Flush(stdout);
          end;
        99 :
          begin
            EmitOverviewForm;
            system.Writeln(stdout,'<PRE>');
            system.Dump_stack(stdout,get_frame);
            system.Writeln(stdout,'</PRE>');
            system.Flush(stdout);
          end;
{$endif TEST}
        end;
    finally
      EmitEnd;
      DisConnectFromDB;
    end;
  Finally
    Terminate;
  end;
end;


Function TTestSuite.InitCGIVars : Integer;

Var
  S : String;

begin
  FHtmlWriter:=THTMLWriter.Create(Response);
  FComboBoxProducer:=TComboBoxProducer.Create(Self);
  DateSeparator:='/';
  Result:=0;
  S:=RequestVariables['action'];
  if Length(S) = 0 then
    S:=RequestVariables['TESTACTION'];
  if S='View_history' then
    FAction:=faction_show_history
  else if S='Show/Compare' then
    FAction:=faction_show_run_results
  else if S='Compare_to_previous' then
    FAction:=faction_compare_with_previous
  else if S='Compare_to_next' then
    FAction:=faction_compare_with_next
  else if S='Compare_right_to_previous' then
    FAction:=faction_compare2_with_previous
  else if S='Compare_right_to_next' then
    FAction:=faction_compare2_with_next
  else if S='Compare_both_to_previous' then
    FAction:=faction_compare_both_with_previous
  else if S='Compare_both_to_next' then
    FAction:=faction_compare_both_with_next
  else
    FAction:=StrToIntDef(S,0);
  S:=RequestVariables['limit'];
  if Length(S) = 0 then
    S:=RequestVariables['TESTLIMIT'];
  FLimit:=StrToIntDef(S,50);
  if FLimit > MaxLimit then
    FLimit:=MaxLimit;
  FVersion:=RequestVariables['version'];
  if Length(FVersion) = 0 then
    FVersion:=RequestVariables['TESTVERSION'];
  TestsuiteCGIURL:=Self.ScriptName;
  SDetailsURL := TestsuiteCGIURL + '?action=1&amp;run1id=%s';
  FOS:=RequestVariables['os'];
  if Length(FOS) = 0 then
    FOS:=RequestVariables['TESTOS'];
  FCPU:=RequestVariables['cpu'];
  if Length(FCPU) = 0 then
    FCPU:=RequestVariables['TESTCPU'];
  FCategory:=RequestVariables['category'];
  if Length(FCategory) = 0 then
    FCategory:=RequestVariables['TESTCATEGORY'];
  FCond:=RequestVariables['cond'];
  if Length(FCond) = 0 then
    FCond:=RequestVariables['TESTCOND'];
  FComment:=RequestVariables['comment'];
  if Length(FComment) = 0 then
    FComment:=RequestVariables['TESTCOMMENT'];
  FSubmitter:=RequestVariables['submitter'];
  if Length(FSubmitter) = 0 then
    FSubmitter:=RequestVariables['TESTSUBMITTER'];
  FMachine:=RequestVariables['machine'];
  if Length(FMachine) = 0 then
    FMachine:=RequestVariables['TESTMACHINE'];

  FRunID:=RequestVariables['run1id'];
  if Length(FRunID) = 0 then
    FRunID:=RequestVariables['TESTRUN'];
  S:=RequestVariables['lastdays'];
  if Length(S) = 0 then
    S:=RequestVariables['TESTLASTDAYS'];
  FTestLastDays:=StrToIntDef(S,31);
  S:=RequestVariables['date'];
  if Length(S) = 0 then
    S:=RequestVariables['TESTDATE'];
  if Length(S) > 0 then
    try
      FDate:=StrToDate(S);
    except
      FDate:=0;
    end;
  S:=RequestVariables['failedonly'];
  if Length(S) = 0 then
    S:=RequestVariables['TESTFAILEDONLY'];
  FOnlyFailed:=(S='1');
  S:=RequestVariables['noskipped'];
  if Length(S) = 0 then
    S:=RequestVariables['TESTNOSKIPPED'];
  FNoSkipped:=(S='1');
  FCompareRunID:=RequestVariables['run2id'];
  FPreviousRunID:=RequestVariables['previousrunid'];
  FNextRunID:=RequestVariables['nextrunid'];
  FPrevious2RunID:=RequestVariables['previous2runid'];
  FNext2RunID:=RequestVariables['next2runid'];
  FTestFileID:=RequestVariables['testfileid'];
  FTestFileName:=RequestVariables['testfilename'];
  FRunCount:=StrToIntDef(RequestVariables['PIETOTAL'],0);
  FRunSkipCount:=StrToIntDef(RequestVariables['PIESKIPPED'],0);
  FRunFailedCount:=StrToIntDef(RequestVariables['PIEFAILED'],0);
  S:=RequestVariables['DEBUGCGI'];
  FDebug:=(S='1');
  S:=RequestVariables['listall'];
  FListAll:=(S='1');
  Result:=FAction;
end;

Function TTestSuite.ConnectToDB : Boolean;

begin
  Result:=False;
  FDB:=TPQConnection.Create(Self);
  FDB.HostName:=DefHost;
  FDB.DatabaseName:=DefDatabase;
  FDB.UserName:=DefDBUser;
  FDB.Password:=DefPassword;
  FTrans := TSQLTransaction.Create(nil);
  FTrans.DataBase := FDB;
  FDB.Transaction := FTrans;
  FDB.Connected:=True;
  Result:=True;
  { All is not the first anymore, we need to put it by default explicity }
  if Length(FOS) = 0 then
    FOS:=GetOSID('All');
  { All is not the first anymore, we need to put it by default explicity }
  if Length(FCPU) = 0 then
    FCPU:=GetCPUID('All');
end;

procedure TTestsuite.LDump(Const St : String);
var
  S : String;
  ShortS : ShortString;
  i,p  : longint;
begin
  i:=length(St);
  p:=1;
  while (i>255) do 
    begin
      ShortS:=copy(St,p,255);
      inc(p,255);
      dec(i,255);
      FHTMLWriter.Dump(ShortS);
    end;
  ShortS:=Copy(St,p,255);
  FHTMLWriter.Dump(ShortS);
end;

procedure TTestsuite.LDumpLn(Const St : String);
begin
  LDump(St);
  LDump(LineFeed);
end;

procedure TTestSuite.DisconnectFromDB;

begin
  If Assigned(FDB) then
    begin
    if (FDB.Connected) then
      FDB.Connected:=False;
    FreeAndNil(FDB);
    FreeAndNil(FTrans);
    end;
end;

Procedure TTestSuite.ComboBoxFromQuery(Const ComboName,Qry: String);

begin
  ComboBoxFromQuery(ComboName,Qry,'')
end;

Procedure TTestSuite.ComboBoxFromQuery(Const ComboName,Qry,Value : String);

Var
  Q : TSQLQuery;

begin
  Q:=TSQLQuery.Create(Self);
  try
    Q.Database:=FDB;
    Q.Transaction:=FTrans;
    Q.SQL.Text:=Qry;
    Q.Open;
    FComboboxProducer.Dataset:=Q;
    FComboBoxProducer.ValueField:=Q.Fields[0].FieldName;
    FComboBoxProducer.DataField:=Q.Fields[1].FieldName;
    FComboBoxProducer.Value:=Value;
    FComboBoxProducer.InputName:=ComboName;
    FComboBoxProducer.CreateComboBox(Response);
  Finally
    Q.Free;
  end;
end;

Function TTestSuite.GetSingleton(Const Qry : String) : String;

Var
  Q : TSQLQuery;

begin
  Result:='';
  if FDEbug then
    begin
      system.Writeln('Query=',Qry);
      system.flush(output);
    end;
  Q:=TSQLQuery.Create(Self);
  try
    Q.Database:=FDB;
    Q.Transaction:=FTrans;
    Q.SQL.Text:=Qry;
    Q.Open;
    Try
      if FDebug and (Q.FieldCount<>1) then
        begin
          system.Writeln('GetSingleton number of fields is not 1, but ',
            Q.FieldCount);
          system.flush(output);
        end;
      If Not (Q.EOF and Q.BOF) then
        Result:=Q.Fields[0].AsString;
    Finally
      Q.Close;
    end;
  finally
    Q.Free;
  end;
end;

Procedure TTestSuite.EmitTitle(ATitle : String);

Var
  S : TStrings;
begin
  AddResponseLn('<HTML>');
  AddResponseLn('<TITLE>'+ATitle+'</TITLE>');
  AddResponseLn('<BODY>');
  FNeedEnd:=true;
end;

Procedure TTestSuite.EmitDocType;
begin
  AddResponseLn('<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01 Transitional//EN" '+
     '"http://www.w3.org/TR/html4/loose.dtd">');
end;

Procedure TTestSuite.EmitOverviewForm;

begin
  ConnectToDB;
  ContentType:='text/html';
  EmitContentType;
  EmitDocType;
  EmitTitle(Title);
  With FHTMLWriter do
    begin
    HeaderStart(1);
    DumpLn('View Test suite results');
    HeaderEnd(1);
    DumpLn('Please specify search criteria:');
    FormStart(TestsuiteCGIURL,'');
    if FDebug then
      EmitHiddenVar('DEBUGCGI', '1');

    TableStart(2,true);
    RowStart;
      CellStart;
        DumpLn('Operating system:');
      CellNext;
        ComboBoxFromQuery('os','SELECT TO_ID,TO_NAME FROM TESTOS ORDER BY TO_NAME',FOS);
      CellEnd;
    RowNext;
      CellStart;
        DumpLn('Processor:');
      CellNext;
        ComboBoxFromQuery('cpu','SELECT TC_ID,TC_NAME FROM TESTCPU ORDER BY TC_NAME',FCPU);
      CellEnd;
    RowNext;
      CellStart;
        DumpLn('Version');
      CellNext;
        ComboBoxFromQuery('version','SELECT TV_ID,TV_VERSION FROM TESTVERSION ORDER BY TV_VERSION DESC',FVERSION);
      CellEnd;
    RowNext;
      CellStart;
        DumpLn('Date');
      CellNext;
        If (FDate=0) then
          EmitInput('date','')
        else
          EmitInput('date',DateToStr(FDate));
      CellEnd;
    //if FDebug then
      begin
        RowNext;
        CellStart;
        DumpLn('Submitter');
        CellNext;
        If (FSubmitter='') then
          EmitInput('submitter','')
        else
          EmitInput('submitter',FSubmitter);
        CellEnd;
       RowNext;
        CellStart;
        DumpLn('Machine');
        CellNext;
        If (FMachine='') then
          EmitInput('machine','')
        else
          EmitInput('machine',FMachine);
        CellEnd;
        RowNext;
        CellStart;
        DumpLn('Comment');
        CellNext;
        If (FComment='') then
          EmitInput('comment','')
        else
          EmitInput('comment',FComment);
        CellEnd;

        RowNext;
        CellStart;
        DumpLn('Cond');
        CellNext;
        If (FCond='') then
          EmitInput('cond','')
        else
          EmitInput('cond',FCond);
        CellEnd;
      end;
    RowNext;
      CellStart;
        DumpLn('Category');
      CellNext;
        ComboBoxFromQuery('Category','SELECT TCAT_ID,TCAT_NAME FROM TESTCATEGORY ORDER BY TCAT_NAME',FCategory);
      CellEnd;
    RowNext;
      CellStart;
        DumpLn('Only failed tests');
      CellNext;
        EmitCheckBox('failedonly','1',FonlyFailed);
      CellEnd;
    RowNext;
      CellStart;
        DumpLn('Hide skipped tests');
      CellNext;
        EmitCheckBox('noskipped','1',FNoSkipped);
      CellEnd;
    RowEnd;
    TableEnd;
    ParaGraphStart;
    EmitSubmitButton('','Search');
    EmitSubmitButton('action','View history');

    EmitResetButton('','Reset form');
    FormEnd;
    end;
  ShowRunOverview;
end;

Procedure TTestSuite.EmitHistoryForm;

begin
  ConnectToDB;
  ContentType:='text/html';
  EmitContentType;
  EmitDocType;
  EmitTitle(Title);
  With FHTMLWriter do
    begin
    HeaderStart(1);
    DumpLn('View Test suite results');
    HeaderEnd(1);
    DumpLn('Please specify search criteria:');
    FormStart(TestsuiteCGIURL,'');
    if FDebug then
      EmitHiddenVar('DEBUGCGI', '1');
    EmitHiddenVar('action',IntToStr(faction_show_history));
    TableStart(2,true);
    RowStart;
      CellStart;
        DumpLn('File:');
      CellNext;
        EmitInput('testfilename',FTestfilename);
      CellEnd;
    RowNext;
    (*   CellStart;
        DumpLn('FileID:');
      CellNext;
        EmitInput('testfileid',FTestfileid);
      CellEnd;
    RowNext; *)

      CellStart;
        DumpLn('Operating system:');
      CellNext;
        ComboBoxFromQuery('os','SELECT TO_ID,TO_NAME FROM TESTOS ORDER BY TO_NAME',FOS);
      CellEnd;
    RowNext;
      CellStart;
        DumpLn('Processor:');
      CellNext;
        ComboBoxFromQuery('cpu','SELECT TC_ID,TC_NAME FROM TESTCPU ORDER BY TC_NAME',FCPU);
      CellEnd;
    RowNext;
      CellStart;
        DumpLn('Version');
      CellNext;
        ComboBoxFromQuery('version','SELECT TV_ID,TV_VERSION FROM TESTVERSION ORDER BY TV_VERSION DESC',FVERSION);
      CellEnd;
    RowNext;
      CellStart;
        DumpLn('Date');
      CellNext;
        If (FDate=0) then
          EmitInput('date','')
        else
          EmitInput('date',DateToStr(FDate));
      CellEnd;
        RowNext;
        CellStart;
        DumpLn('Submitter');
        CellNext;
        If (FSubmitter='') then
          EmitInput('submitter','')
        else
          EmitInput('submitter',FSubmitter);
        CellEnd;
       RowNext;
        CellStart;
        DumpLn('Machine');
        CellNext;
        If (FMachine='') then
          EmitInput('machine','')
        else
          EmitInput('machine',FMachine);
        CellEnd;
        RowNext;
        CellStart;
        DumpLn('Comment');
        CellNext;
        If (FComment='') then
          EmitInput('comment','')
        else
          EmitInput('comment',FComment);
        CellEnd;
        RowNext;
        CellStart;
        DumpLn('Limit');
        CellNext;
        EmitInput('limit',IntToStr(FLimit));
        CellEnd;

        RowNext;
        CellStart;
        DumpLn('Cond');
        CellNext;
        If (FCond='') then
          EmitInput('cond','')
        else
          EmitInput('cond',FCond);
        CellEnd;
    RowNext;
      CellStart;
        DumpLn('Category');
      CellNext;
        ComboBoxFromQuery('Category','SELECT TCAT_ID,TCAT_NAME FROM TESTCATEGORY ORDER BY TCAT_NAME',FCategory);
      CellEnd;
    RowNext;
      CellStart;
        DumpLn('Only failed tests');
      CellNext;
        EmitCheckBox('failedonly','1',FonlyFailed);
      CellEnd;
    RowNext;
      CellStart;
        DumpLn('Hide skipped tests');
      CellNext;
        EmitCheckBox('noskipped','1',FNoSkipped);
      CellEnd;
    RowNext;
      CellStart;
        DumpLn('List all tests');
      CellNext;
        EmitCheckBox('listall','1',FListAll);
      CellEnd;

    RowEnd;
    TableEnd;
    ParaGraphStart;
    if FDebug then
      EmitHiddenVar('DEBUGCGI', '1');
    EmitSubmitButton('','Search');
    EmitResetButton('','Reset form');
    FormEnd;
    end;
end;


procedure TTestSuite.EmitEnd;
begin
  if not FNeedEnd then
    exit;
  AddResponseLn('</BODY>');
  AddResponseLn('</HTML>');
end;

procedure TTestSuite.GetOverviewRowAttr(Sender: TObject; var BGColor: String;
  var Align: THTMLAlign; var VAlign: THTMLValign; var CustomAttr: String);
begin
  If ((Sender as TTAbleProducer).CurrentRow mod 2=0) then
    BGColor:='#EEEEEE'
end;


Function TTestSuite.CreateDataset(Qry : String) : TSQLQuery;

begin
  Result:=TSQLQuery.Create(Self);
  With Result do
    begin
    Database:=FDB;
    Transaction := FTrans;
    SQL.Text:=Qry;
    end;
end;

Function TTestSuite.CreateTableProducer(DS : TDataset) :TTableProducer;

begin
  Result:=TTableProducer.Create(Self);
  Result.Dataset:=DS;
end;

Procedure TTestSuite.DefaultTableFromQuery(Qry,Alink : String; IncludeRecordCount : Boolean);

Var
  Q : TSQLQuery;

begin
  If FDebug then
    Writeln('Query : '+Qry);
  Q:=CreateDataset(Qry);
  With Q do
    try
      Open;
      Try
        With CreateTableProducer(Q) do
          Try
            Border:=True;
            If (Alink<>'') then
              begin
              CreateColumns(Nil);
              If TableColumns.Count>0 then
                (TableColumns.Items[0] as TTableColumn).ActionURL:=ALink;
              end;
            CreateTable(Response);
          Finally
            Free;
          end;
        If IncludeRecordCount then
          FHTMLWriter.DumpLn(Format('<p>Record count: %d </p>',[Q.RecordCount]));
      Finally
        Close;
      end;
    finally
      Free;
    end;
end;

Procedure TTestSuite.ShowRunOverview;
Const
  SOverview = 'SELECT TU_ID as ID,TU_DATE as Date,TC_NAME as CPU,TO_NAME as OS,'+
               'TV_VERSION as Version,(select count(*) from testresults where (TR_TESTRUN_FK=TU_ID)) as Count,'+
               'TU_SVNCOMPILERREVISION as SvnCompRev,'+
               'TU_SVNRTLREVISION as SvnRTLRev,'+
               'TU_SVNPACKAGESREVISION as SvnPackRev,TU_SVNTESTSREVISION as SvnTestsRev,'+
               '(TU_SUCCESSFULLYFAILED+TU_SUCCESFULLYCOMPILED+TU_SUCCESSFULLYRUN) AS OK,'+
               '(TU_FAILEDTOCOMPILE+TU_FAILEDTORUN+TU_FAILEDTOFAIL) as Failed,'+
               '(TU_SUCCESSFULLYFAILED+TU_SUCCESFULLYCOMPILED+TU_SUCCESSFULLYRUN+'+
                'TU_FAILEDTOCOMPILE+TU_FAILEDTORUN+TU_FAILEDTOFAIL) as Total,'+
               'TU_SUBMITTER as Submitter, TU_MACHINE as Machine, TU_COMMENT as Comment %s '+
              'FROM '+
               ' TESTRUN '+
               ' left join TESTCPU on (TC_ID=TU_CPU_FK) '+
               ' left join TESTOS on (TO_ID=TU_OS_FK) '+
               ' left join TESTVERSION on (TV_ID=TU_VERSION_FK) '+
               ' left join TESTCATEGORY on (TCAT_ID=TU_CATEGORY_FK) '+
              '%s'+
              'ORDER BY TU_ID DESC LIMIT %d';


Var
  SC,S,A,Qry : String;
  Q : TSQLQuery;

begin
   S:='';
   If (FCPU<>'') and (GetCPUName(FCPU)<>'All') then
     S:=S+' AND (TU_CPU_FK='+FCPU+')';
   If (FCategory<>'') and (GetCategoryName(FCategory)<>'All') then
     S:=S+' AND (TU_CATEGORY_FK='+FCategory+')';
   If (FVersion<>'') and (GetVersionName(FVersion)<>'All')  then
     S:=S+' AND (TU_VERSION_FK='+FVERSION+')';
   if (FOS<>'') and (GetOSName(FOS)<>'All') then
     S:=S+' AND (TU_OS_FK='+FOS+')';
   If (Round(FDate)<>0) then
     S:=S+' AND (to_char(TU_DATE, ''YYYY-MM-DD'') LIKE '''+FormatDateTime('YYYY-MM-DD',FDate)+'%'')';
   If FSubmitter<>'' then
     S:=S+' AND (TU_SUBMITTER='''+FSubmitter+''')';
   If FMachine<>'' then
     S:=S+' AND (TU_MACHINE='''+FMachine+''')';
   If FComment<>'' then
     S:=S+' AND (TU_COMMENT LIKE '''+Fcomment+''')';
   If FCond<>'' then
     S:=S+' AND ('+FCond+')';
   If GetCategoryName(FCategory)<>'DB' then
     SC:=', CONCAT(TU_SVNCOMPILERREVISION,''/'',TU_SVNRTLREVISION,''/'', '+
          'TU_SVNPACKAGESREVISION,''/'',TU_SVNTESTSREVISION) as svnrev'
   else
     SC:='';
   If (FCategory='') or (GetCategoryName(FCategory)='All') then
     SC:=SC+', TCAT_NAME as Cat';

   A:=SDetailsURL;
   If FOnlyFailed then
     A:=A+'&amp;failedonly=1';
   If FNoSkipped then
     A:=A+'&amp;noskipped=1';
     
  if S <> '' then
  begin
    Delete(S, 1, 4);
    S:='WHERE '+ S + ' ';
  end;
  Qry:=Format(SOverview,[SC,S,FLimit]);
  If FDebug then
    Writeln('Query : '+Qry);
  Q:=CreateDataset(Qry);
  With Q do
    try
      Open;
      Try
        With CreateTableProducer(Q) do
          Try
            Border:=True;
            OnGetRowAttributes:=@GetOverViewRowAttr;
            CreateColumns(Nil);
            TableColumns.ColumnByName('ID').ActionURL:=A;
            TableColumns.ColumnByNAme('Failed').OnGetCellContents:=@FormatFailedOverview;
            TableColumns.ColumnByNAme('svnrev').OnGetCellContents:=@FormatSVN;
            CreateTable(Response);
          Finally
            Free;
          end;
        FHTMLWriter.DumpLn(Format('<p>Record count: %d</p>',[Q.RecordCount]));
      Finally
        Close;
      end;
    finally
      Free;
    end;
end;


Function TTestSuite.GetOSName(ID : String) : String;

begin
  if (ID<>'') then
    Result:=GetSingleTon('SELECT TO_NAME FROM TESTOS WHERE TO_ID='+ID)
  else
    Result:='';
end;

Function TTestSuite.GetOSID(AName : String) : String;

begin
  if (AName<>'') then
    Result:=GetSingleTon('SELECT TO_ID FROM TESTOS WHERE TO_NAME='''+Aname+'''')
  else
    Result:='';
end;

Function TTestSuite.GetTestFileName(ID : String) : String;

begin
  if (ID<>'') then
    Result:=GetSingleTon('SELECT T_NAME FROM TESTS WHERE T_ID='+ID)
  else
    Result:='';
end;

Function TTestsuite.GetFailCount(RunID : longint) : string;
begin
  if RunID<>0 then
    Result:=GetSingleTon('SELECT (TU_FAILEDTOCOMPILE + TU_FAILEDTOFAIL + TU_FAILEDTORUN) FROM TESTRUN WHERE TU_ID='+IntToStr(RunID))
  else
    Result:='';
end;

Function TTestSuite.GetCPUName(ID : String) : String;

begin
  if (ID<>'') then
    Result:=GetSingleTon('SELECT TC_NAME FROM TESTCPU WHERE TC_ID='+ID)
  else
    Result:='';
end;

Function TTestSuite.GetCPUID(AName : String) : String;

begin
  if (AName<>'') then
    Result:=GetSingleTon('SELECT TC_ID FROM TESTCPU WHERE TC_NAME='''+AName+'''')
  else
    Result:='';
end;

Function TTestSuite.GetVersionName(ID : String) : String;

begin
  if (ID<>'') then
    Result:=GetSingleton('SELECT TV_VERSION FROM TESTVERSION WHERE TV_ID='+ID)
  else
    Result:='';
end;

Function TTestSuite.GetCategoryName(ID : String) : String;

begin
  if (ID<>'') then
    Result:=GetSingleton('SELECT TCAT_NAME FROM TESTCATEGORY WHERE TCAT_ID='+ID)
  else
    Result:='';
end;

Function TTestSuite.GetPreviousRunID(RunID : String) : String;

begin
  if (RunID<>'') then
    Result:=GetSingleton('SELECT TH_PREVIOUS_FK FROM TESTRUNHISTORY WHERE TH_ID_FK='+RunID)
  else
    Result:='';
end;

Function TTestSuite.GetNextRunID(RunID : String) : String;

begin
  if (RunID<>'') then
    Result:=GetSingleton('SELECT TH_ID_FK FROM TESTRUNHISTORY WHERE TH_PREVIOUS_FK='+RunID)
  else
    Result:='';
end;

Function TTestSuite.ShowRunData : Boolean;

Const
  SGetRunData = 'SELECT TU_ID,TU_DATE,TC_NAME,TO_NAME,' +
                'TU_SUBMITTER,TU_MACHINE,TU_COMMENT,TV_VERSION,'+
                'TU_CATEGORY_FK,TU_SVNCOMPILERREVISION,TU_SVNRTLREVISION,'+
                'TU_COMPILERDATE,TU_COMPILERFULLVERSION,'+
                'TU_SVNPACKAGESREVISION,TU_SVNTESTSREVISION,'+
               '(TU_SUCCESSFULLYFAILED+TU_SUCCESFULLYCOMPILED+TU_SUCCESSFULLYRUN) AS OK,'+
               '(TU_FAILEDTOCOMPILE+TU_FAILEDTORUN+TU_FAILEDTOFAIL) as Failed,'+
               '(TU_SUCCESSFULLYFAILED+TU_SUCCESFULLYCOMPILED+TU_SUCCESSFULLYRUN+'+
                'TU_FAILEDTOCOMPILE+TU_FAILEDTORUN+TU_FAILEDTOFAIL) as Total'+

                ' %s FROM TESTRUN,TESTCPU,TESTOS,TESTVERSION '+
                'WHERE '+
                ' (TC_ID=TU_CPU_FK) AND '+
                ' (TO_ID=TU_OS_FK) AND '+
                ' (TV_ID=TU_VERSION_FK) AND '+
                ' (TU_ID=%s)';


Var
  Q1, Q2 : TSQLQuery;
  F : TField;
  SC, FRight : string;
  Date1, Date2 : TDateTime;
  AddNewPar, same_date : boolean;
  CompilerDate1, CompilerDate2 : TDateTime;

  procedure EmitOneRow(RowTitle,FieldLeft,FieldRight : String; is_same : boolean);
    var
      FieldColor : string;
    begin
      if (FieldRight='') then
        FieldColor:=''
      else if is_same then
        FieldColor:='style="color:green;"'
      else
        FieldColor:='style="color:red;"';
      With FHTMLWriter do
        begin
          RowNext;
          if FieldColor<>'' then
            begin
              TagStart('TD',FieldColor);
            end
          else 
            CellStart;
          LDumpLn(RowTitle);
          if FieldColor<>'' then
            begin
              CellEnd;
              TagStart('TD',FieldColor);
            end
          else 
            CellNext;
          LDumpLn(FieldLeft);
          if FieldColor<>'' then
            begin
             CellEnd;
             TagStart('TD',FieldColor);
            end
          else 
            CellNext;
          LDumpLn(FieldRight);
          CellEnd;
        end;
    end;
  procedure EmitOneRow(RowTitle,FieldLeft,FieldRight : String);
    var
      is_same : boolean;
    begin
      is_same:=(FieldLeft=FieldRight);
      EmitOneRow(RowTitle,FieldLeft,FieldRight,is_same);
    end;
  procedure EmitRow(RowTitle,FieldName : String);
    var
      FieldLeft, FieldRight : String;
    begin
      FieldLeft:=Q1.FieldByName(FieldName).AsString;
      if Q2=nil then
        FieldRight:=''
      else
        FieldRight:=Q2.FieldByName(FieldName).AsString;
      EmitOneRow(RowTitle,FieldLeft,FieldRight);
    end;
begin
  Result:=(FRunID<>'');
  If Result then
    begin
    If GetCategoryName(FCategory)<>'DB' then
      SC:=', CONCAT(TU_SVNCOMPILERREVISION,''/'',TU_SVNRTLREVISION,''/'', '+
          'TU_SVNPACKAGESREVISION,''/'',TU_SVNTESTSREVISION) as svnrev'
    else
      SC:='';
    If GetCategoryName(FCategory)='All' then
      SC:=SC+', TCAT_NAME as Cat';

    Q1:=CreateDataset(Format(SGetRunData,[SC,FRunID]));
    if Length(FCompareRunID) > 0 then
      Q2:=CreateDataset(Format(SGetRunData,[SC,FCompareRunID]))
    else
      Q2:=nil;
    Try
      Q1.Open;
      if Q2 <> nil then
        Q2.Open;
      Result:=Not (Q1.EOF and Q1.BOF);
      If Result then
       With FHTMLWriter do
        begin
          FormStart(TestsuiteCGIURL,'get');
          TableStart(3,true);
          RowStart;
            CellStart;
              DumpLn('Run ID:');
            CellNext;
              EmitInput('run1id',FRunID);
            CellNext;
              EmitInput('run2id',FCompareRunID);
            CellEnd;

          EmitRow('Operating system:','TO_NAME');
          EmitRow('Processor:','TC_NAME');
          EmitRow('Version:','TV_VERSION');
          if Q2 = nil then
            FRight:=''
          else
            begin
              FRight:=Q2.FieldByName('Failed').AsString+
                      '/'+Q2.FieldByName('Ok').AsString+
                      '/'+Q2.FieldByName('Total').AsString;
            end;
          EmitOneRow('Fails/OK/Total:',
            Q1.FieldByName('Failed').AsString+
            '/'+Q1.FieldByName('OK').AsString+
            '/'+Q1.FieldByName('Total').AsString,
            FRight);
          EmitRow('Version:','TV_VERSION');
          EmitRow('Full version:','TU_COMPILERFULLVERSION');
          EmitRow('Comment:','TU_COMMENT');
          EmitRow('Machine:','TU_MACHINE');
          if GetCategoryName(FCategory)<>'All' then
            EmitRow('Category:','TU_CATEGORY_FK');
          If GetCategoryName(FCategory)<>'DB' then
            begin
              SC:=Q1.FieldByName('svnrev').AsString;
              if (SC<>'') then
                FormatSVNData(SC);
              if Q2 <> nil then
                begin
                  FRight:=Q2.FieldByName('svnrev').AsString;
                  FormatSVNData(FRight);
                end
              else
                FRight:='';
              EmitOneRow('SVN revisions:',SC,FRight);
            end;
          EmitRow('Submitter:','TU_SUBMITTER');
          F := Q1.FieldByName('TU_DATE');
          Date1 := F.AsDateTime;
          SC:=F.AsString;
          F := Q1.FieldByName('TU_COMPILERDATE');
          Try
            CompilerDate1 := F.AsDateTime;
            if not SameDate(Date1,CompilerDate1) then
              SC:=SC+' <> '+F.AsString;
          Except
            { Not a valid date, do nothing }
          end;
          if Q2 = nil then
            FRight:=''
          else
            begin
              F := Q2.FieldByName('TU_DATE');
              Date2 := F.AsDateTime;
              FRight:= F.AsString;
              F := Q2.FieldByName('TU_COMPILERDATE');
              Try
                CompilerDate2 := F.AsDateTime;
                if not SameDate(Date2,CompilerDate2) then
                  FRight:=FRight+' <> '+F.AsString;
              Except
                { Not a valid date, do nothing }
              end;
            end;
          same_date:=(Copy(SC,1,10)=Copy(FRight,1,10));
          EmitOneRow('Date:',SC,FRight,same_date); 
          FPreviousRunID:=GetPreviousRunID(FRunID);
          if FPreviousRunID<>'' then
            EmitHiddenVar('previousrunid',FPreviousRunID);
          SC:=FPreviousRunID;
          if (FCompareRunID<>'') then
            begin
              FPrevious2RunID:=GetPreviousRunID(FCompareRunID);
              FRight:=FPrevious2RunID;
              if FPrevious2RunID <> '' then
                EmitHiddenVar('previous2runid',FPrevious2RunID);
            end
          else
            FRight:='';
          EmitOneRow('Previous run:',SC,FRight);
          FNextRunID:=GetNextRunID(FRunID);
          if FNextRunID<>'' then
            EmitHiddenVar('nextrunid',FNextRunID);
          SC:=FNextRunID;
          if (FCompareRunID<>'') then
            begin
              FNext2RunID:=GetNextRunID(FCompareRunID);
              FRight:=FNext2RunID;
              if FNext2RunID <> '' then
                EmitHiddenVar('next2runid',FNext2RunID);
            end;
          EmitOneRow('Next run:',SC,FRight);
          RowEnd;
          TableEnd;
          ParagraphStart;
          if FDebug then
            EmitHiddenVar('DEBUGCGI', '1');
          EmitCheckBox('noskipped','1',FNoSkipped);
          DumpLn(' Hide skipped tests');
	  ParagraphEnd;
	  ParagraphStart;
          EmitCheckBox('failedonly','1',FonlyFailed);
          DumpLn(' Hide successful tests');
          ParagraphEnd;
          ParaGraphStart;
          AddNewPar:=false;
          if FPreviousRunID<>'' then
            begin
              EmitSubmitButton('action','Compare_to_previous');
              AddNewPar:=true;
            end;
          if (FNextRunID<>'') and (FNextRunID <> FCompareRunID) then
            begin
              EmitSubmitButton('action','Compare_to_next');
              AddNewPar:=true;
            end;
          if (FPrevious2RunID<>'') and (FPrevious2RunID <> FRunID) then
            begin
              EmitSubmitButton('action','Compare_right_to_previous');
              AddNewPar:=true;
            end;
          if FNext2RunID<>'' then
            begin
              EmitSubmitButton('action','Compare_right_to_next');
              AddNewPar:=true;
            end;
          if AddNewPar then
            begin
              ParagraphEnd;
              ParaGraphStart;
            end;
              
          if (FPrevious2RunID<>'') and (FPreviousRunId<>'') then
            begin
              EmitSubmitButton('action','Compare_both_to_previous');
              AddNewPar:=true;
            end;
          if (FNext2RunID<>'') and (FNextRunId<>'') then
            begin
              EmitSubmitButton('action','Compare_both_to_next');
              AddNewPar:=true;
            end;
          if AddNewPar then
            begin
              ParagraphEnd;
              ParaGraphStart;
            end;
           EmitSubmitButton('action','Show/Compare');
          if FTestFileID<>'' then
            EmitSubmitButton('action','View_history');
          EmitResetButton('','Reset form');
          ParagraphEnd;
          FormEnd;
          { give warning if dates reversed }
          if (Q2 <> nil) and (Date1 > Date2) then
            begin
            ParagraphStart;
            DumpLn('Warning: testruns are not compared in chronological order.');
            ParagraphEnd;
            end;
          end;
    Finally
      Q1.Close;
      Q1.Free;
      if Q2 <> nil then
        begin
        Q2.Close;
        Q2.Free;
        end;
    end;
    end;
end;

Procedure TTestSuite.ShowRunResults;

Var
  S : String;
  Qry : String;
  Q : TSQLQuery;
  FL : String;

begin
  ConnectToDB;
  ContentType:='text/html';
  EmitContentType;
  EmitDocType;
  EmitTitle(Title+' : Search Results');
  With FHTMLWriter do
    begin
    HeaderStart(1);
    DumpLn('Test suite results for run '+FRunID);
    HeaderEnd(1);
    HeaderStart(2);
    DumpLn('Test run data : ');
    HeaderEnd(2);
    If ShowRunData then
      begin
      HeaderStart(2);
      DumpLn('Detailed test run results:');

      FL:='';
      If FOnlyFailed or FNoSkipped then
        begin
        FL:='';
        If FOnlyFailed then
          FL:='successful';
        if FNoSkipped then
          begin
          If (FL<>'') then
            FL:=FL+' and ';
          FL:=FL+'skipped';
          end;
        DumpLn(' ('+FL+' tests are hidden)');
        end;
      HeaderEnd(2);
      S:='SELECT T_ID as Id,T_NAME as Filename,TR_SKIP as Skipped'
        +',TR_OK as OK,TR_RESULT as Result'
        +' FROM '+TESTRESULTSTableName(FRunID)+',TESTS'
        +' WHERE (TR_TEST_FK=T_ID) AND (TR_TESTRUN_FK='+FRunID+') ';

      If FOnlyFailed then
        S:=S+' AND (not TR_OK)';
      If FNoSkipped then
        S:=S+' AND (not TR_SKIP)';
      S:=S+' ORDER BY TR_ID ';
      Qry:=S;
      If FDebug then
        begin
        ParaGraphStart;
        Dumpln('Query : '+Qry);
        ParaGraphEnd;
      end;
      FRunCount:=0;
      FRunSkipCount:=0;
      FRunFailedCount:=0;
      Q:=CreateDataset(Qry);
      With Q do
        try
          Open;
          while not EOF do
            Next;
          RecNo:=1;

          DumpLn(Format('<p>Record count: %d </p>',[Q.RecordCount]));
          Try
            With CreateTableProducer(Q) do
              Try
                Border:=True;
                FL:='Id,Filename';
                If Not FNoSkipped then
                  FL:=FL+',Skipped';
                If Not FOnlyFailed then
                  FL:=FL+',OK';
                FL:=FL+',Result';
                CreateColumns(FL);
                OnGetRowAttributes:=@GetRunRowAttr;
                TableColumns.ColumnByNAme('Id').OnGetCellContents:=
                  @FormatFileIDDetails;

                TableColumns.ColumnByNAme('Filename').OnGetCellContents:=
                  @FormatFileDetails;
                TableColumns.ColumnByNAme('Result').OnGetCellContents:=
                  @FormatTestResult;
                //(TableColumns.Items[0] as TTableColumn).ActionURL:=ALink;
                CreateTable(Response);
              Finally
                Free;
              end;
          Finally
            Close;
          end;
        finally
          Free;
        end;
      If Not (FRunCount=0) and not (FNoSkipped and FOnlyFailed) then
        begin
        ParaGraphStart;
        TagStart('IMG',Format('Src="'+TestsuiteCGIURL+
        '?action=2&amp;pietotal=%d&amp;piefailed=%d&amp;pieskipped=%d"'+
        ' ALT="total=%d, failed=%d, skipped=%d"',
        [FRunCount,FRunFailedCount,FRunSkipCount,
          FRunCount,FRunFailedCount,FRunSkipCount
          ]));
        end;
      end
    else
      DumpLn('No data for test run with ID: '+FRunID);
    end;
end;

Procedure TTestSuite.DumpTestInfo(Q : TSQLQuery);

Var
  I : Integer;
  field_displayed : boolean;
  FieldValue,FieldName : String;

begin
  With FHTMLWriter do
    For i:=0 to Q.FieldCount-1 do
      begin
      FieldValue:=Q.Fields[i].AsString;
      FieldName:=Q.Fields[i].DisplayName;
      field_displayed:=false;
      if (Not Q.fields[i].IsNull) and (FieldName<>'t_name') and (FieldName<>'t_source') then
        begin
        if (Q.Fields[i].Datatype=ftBoolean) then
          begin
            if Q.Fields[i].AsBoolean then
              begin
                DumpLn('Flag ');
                DumpLn(FieldName);
                DumpLn(' set');
                field_displayed:=true;
              end;
          end
        else if FieldValue<>'' then
          begin
            DumpLn(FieldName);
            DumpLn(' ');
            DumpLn(FieldValue);
            field_displayed:=true;
          end;
        if field_displayed then
          DumpLn('<BR>');
        end;
      end;
end;

Procedure TTestSuite.ShowOneTest;

Var
  S,S2 : String;
  Qry : String;
  Base, Category : string;
  Q : TSQLQuery;
  i : longint;
  FieldName,FieldValue,
  LLog,Source : String;
  Res : Boolean;
  ver : known_versions;
begin
  ConnectToDB;
  ContentType:='text/html';
  EmitContentType;
  EmitDocType;
  if FTestFileID='' then
    FTestFileID:=GetSingleton('SELECT T_ID FROM TESTS WHERE T_NAME LIKE ''%'+FTestFileName+'%''');
  if FTestFileID<>'' then
    FTestFileName:=GetTestFileName(FTestFileID);
  EmitTitle(Title+' : File '+FTestFileName+' Results');
  With FHTMLWriter do
    begin
    HeaderStart(1);
    DumpLn('Test suite results for test file '+FTestFileName);
    HeaderEnd(1);
    HeaderStart(2);
    DumpLn('Test run data : ');
    HeaderEnd(2);
    if FRunID<>'' then
      begin
        Res:=ShowRunData;
        Res:=true;
      end
    else
      begin
        // This is useless as it is now
        // It should be integrated into a form probably PM
        DumpLn('Only failed tests');
        EmitCheckBox('failedonly','1',FonlyFailed);
        DumpLn('Hide skipped tests');
        EmitCheckBox('noskipped','1',FNoSkipped);
        Res:=true;
      end;
    If Res then
      begin
      HeaderStart(2);
      DumpLn('Test file "'+FTestFileName+'" information:');
      HeaderEnd(2);
      ParaGraphStart;
      if FTestFileID<>'' then
        S:='SELECT * FROM TESTS WHERE T_ID='+FTestFileID
      else
        S:='SELECT * FROM TESTS WHERE T_NAME='+FTestFileName;
      Q:=CreateDataSet(S);
      With Q do
        Try
          Open;
          Try
            DumpTestInfo(Q);
          Finally
            Close;
          end;
        Finally
          Free;
        end;
      ParaGraphEnd;
      HeaderStart(2);
      DumpLn('Detailed test run results:');

      HeaderEnd(2);
      S:='SELECT TR_ID,TR_TESTRUN_FK AS RUN,TR_TEST_FK,TR_OK, TR_SKIP,TR_RESULT '
      //S:='SELECT * '
        +' FROM '+TESTRESULTSTableName(FRunID)
        +' WHERE  (TR_TEST_FK='+FTestFileID+')';
      If FOnlyFailed then
        S:=S+' AND (TR_OK="-")';
      if Fcomparerunid<>'' then
        begin
          if TESTRESULTSTableName(FRunID)<>TESTRESULTSTableName(FCompareRunID) then
            begin
              S2:='SELECT TR_ID,TR_TESTRUN_FK AS RUN,TR_TEST_FK,TR_OK, TR_SKIP,TR_RESULT '
                  +' FROM '+TESTRESULTSTableName(FCompareRunID)
                  +' WHERE  (TR_TEST_FK='+FTestFileID+')';
              If FOnlyFailed then
                S2:=S2+' AND (TR_OK="-")';

              S:=S+' AND (TR_TESTRUN_FK='+Frunid+') UNION '+
                 S2+' AND (TR_TESTRUN_FK='+Fcomparerunid+')'
            end
          else
            S:=S+' AND ((TR_TESTRUN_FK='+Frunid+') OR '+
                 '(TR_TESTRUN_FK='+Fcomparerunid+'))'
        end
      else if Frunid<>'' then
        S:=S+' AND (TR_TESTRUN_FK='+Frunid+')'
      else
         S:=S+' ORDER BY TR_TESTRUN_FK DESC LIMIT '+IntToStr(FLimit);
      Qry:=S;
      If FDebug then
      begin
        ParaGraphStart;
        Dumpln('Query : '+Qry);
        ParaGraphEnd;
      end;
      FRunCount:=0;
      FRunSkipCount:=0;
      FRunFailedCount:=0;
      Q:=CreateDataset(Qry);
      With Q do
        try
          Open;
          Try
            With CreateTableProducer(Q) do
              Try
                Border:=True;
                //FL:='TR_ID,TR_TESTRUN_FK,T_NAME,T_CPU,T_VERSION';
                CreateColumns(Nil);
                TableColumns.Delete(TableColumns.ColumnByName('TR_TEST_FK').Index);
                TableColumns.ColumnByNAme('RUN').OnGetCellContents:=
                  @FormatTestRunOverview;
                //OnGetRowAttributes:=@GetRunRowAttr;
                TableColumns.ColumnByNAme('TR_RESULT').OnGetCellContents:=
                  @FormatTestResult;
                //(TableColumns.Items[0] as TTableColumn).ActionURL:=ALink;
                CreateTable(Response);
              Finally
                Free;
              end;
           ParaGraphStart;
           DumpLn(Format('Record count: %d',[Q.RecordCount]));
           ParaGraphEnd;
          Finally
            Close;
          end;
        finally
          Free;
        end;
             //If FDebug then
            Category:='1';
            if FRunId<>'' then
              begin
                Category:=getsingleton('select TU_CATEGORY_FK from TESTRUN where TU_ID='+FRunId);
                FVersionBranch:=GetVersionName(getsingleton('select TU_VERSION_FK from TESTRUN where TU_ID='+fRunId));
                LLog:='';
                Try
                LLog:=getsingleton('select TR_LOG from TESTRESULTS where (TR_TEST_FK='+ftestfileid
                     +') and (TR_TESTRUN_FK='+frunid+')');
                if LLog<>'' then
                  begin
                    HeaderStart(2);
                    DumpLn('Log of '+FRunId+':');
                    HeaderEnd(2);
                    PreformatStart;
                    system.Write(LLog);
                    system.flush(output);
                    PreformatEnd;
                  end;
                Finally
                  if LLog='' then
                    begin
                      HeaderStart(2);
                      DumpLn('No log of '+FRunId+'.');
                      HeaderEnd(2);
                    end;
                end;
              end;
            if FCompareRunId<>'' then
              begin
                LLog:='';
                Try
                LLog:=getsingleton('select TR_LOG from TESTRESULTS where (TR_TEST_FK='+ftestfileid
                     +') and (TR_TESTRUN_FK='+fcomparerunid+')');
                if LLog<>'' then
                  begin
                    HeaderStart(2);
                    DumpLn('Log of '+FCompareRunId+':');
                    HeaderEnd(2);
                    PreformatStart;
                    system.Write(LLog);
                    system.flush(output);
                    PreformatEnd;
                  end;
                Finally
                  if LLog='' then
                    begin
                      HeaderStart(2);
                      DumpLn('No log of '+FCompareRunId+'.');
                      HeaderEnd(2);
                    end;
                end;
              end;
            if FDebug then
              DumpLn('After Log.');
            Source:='';
            Try
            Source:=getsingleton('select T_SOURCE from TESTS where T_ID='+ftestfileid);
            if Source<>'' then
              begin
                HeaderStart(2);
                DumpLn('Source:');
                HeaderEnd(2);
                PreformatStart;
                system.Write(Source);
                system.flush(output);
                PreformatEnd;
              end;
            Finally
            Base:='trunk';
            if  FVersionBranch<>'' then
              begin
                // Test all but last version, which is assumed to be trunk
                for ver:=low(known_versions) to pred(high(known_versions)) do
                  if VER_String[ver]=FVersionBranch then
                    begin
                      base:=ver_branch[ver];
                      break;
                    end;
              end;
            FViewVCURL:=ViewURL+Base;
            if Category='1' then
              FViewVCUrl:=FViewVCURL+TestsSubDir
            else
              begin
                FViewVCUrl:=FViewVCURL+DataBaseSubDir;
                // This assumes that type TAnyType is
                // defined in anytype.pas source PM
                if pos('/',FTestFileName)>0 then
                  FTestfilename:=lowercase(copy(FTestFilename,2,pos('/',FTestFilename)-2)+'.pas');
              end;
            if Source='' then
              begin
                HeaderStart(3);
                DumpLn('<P>No Source in TestSuite DataBase.</P>');
                DumpLn('Link to SVN view of '+
                  '<A HREF="'+FViewVCURL+FTestFileName+'?view=markup'+
                  '" TARGET="fpc_source"> '+FTestFileName+'</A> source. ');
                HeaderEnd(3);
              end
            else
              begin
                HeaderStart(3);
                DumpLn('Link to SVN view of '+
                  '<A HREF="'+FViewVCURL+FTestFileName+'?view=markup'+
                  '" TARGET="fpc_source"> '+FTestFileName+'</A> source. ');
                HeaderEnd(3);
              end;
            end;
             if FDebug then
              DumpLn('After Source.');
    end
    else
      DumpLn(Format('No data for test file with ID: %s',[FTestFileID]));

    end;
end;


Procedure TTestSuite.ShowHistory;

Const
  { We already have 53 versions }
  MaxCombo = 100;

Type
  StatusLongintArray = Array [TTestStatus] of longint;
  StatusDateTimeArray = Array [TTestStatus] of TDateTime;
  AStatusLA = Array[1..MaxCombo] of StatusLongintArray;
  AStatusDTA = Array[1..MaxCombo] of StatusDateTimeArray;
  PStatusLA = ^AStatusLA;
  PStatusDTA = ^AStatusDTA;

Var
  S,SS,FL,cpu,version,os : String;
  date : TDateTime;
  Qry : String;
  Base, Category : string;
  Q : TSQLQuery;
  i,run_id,os_id,version_id,cpu_id : longint;
  run_ind,os_ind,version_ind,cpu_ind,
  ok_ind,skip_ind,result_ind,date_ind : longint;
  os_size, cpu_size, version_size : longint;
  os_last, cpu_last, version_last : longint;
  error : word;
  OK_count, not_OK_count,resi,
  total_count, skip_count, not_skip_count : longint;
  TS : TTestStatus;
  result_count : StatusLongintArray;
  os_count,cpu_count,version_count: PStatusLA;
  first_date, last_date : array[TTestStatus] of TDateTime;
  first_date_id, last_date_id : array[TTestStatus] of longint;
  os_first_date, os_last_date,
  cpu_first_date, cpu_last_date,
  version_first_date, version_last_date : PStatusDTA;
  os_first_date_id, os_last_date_id,
  cpu_first_date_id, cpu_last_date_id,
  version_first_date_id, version_last_date_id : PStatusLA;
  FieldName,FieldValue,
  LLog,Source : String;
  B,Res : Boolean;
  ver : known_versions;

begin
  Res:=False;
  os_count:=nil;
  cpu_count:=nil;
  version_count:=nil;
  ConnectToDB;
  ContentType:='text/html';
  EmitContentType;
  EmitDocType;
  if (FTestFileID='') and (FTestFileName<>'') then
  begin
    FTestFileID:=GetSingleton('SELECT T_ID FROM TESTS WHERE T_NAME LIKE ''%'+
     FTestFileName+'%''');
  end;
  if FTestFileID<>'' then
    FTestFileName:=GetTestFileName(FTestFileID);
  if FTestFileName<>'' then
    EmitTitle(Title+' : File '+FTestFileName+' Results')
  else
    EmitTitle(Title+' : History overview');
  With FHTMLWriter do
    begin
    if FTestFileName<>'' then
      begin
        HeaderStart(1);
        DumpLn('Test suite results for test file '+FTestFileName);
        HeaderEnd(1);
        HeaderStart(2);
        DumpLn('Test run data : ');
        HeaderEnd(2);
      end;
    if FRunID<>'' then
      begin
        Res:=ShowRunData;
        Res:=true;
      end
    else
      begin
        // This is useless as it is now
        // It should be integrated into a form probably PM
        //DumpLn('Only failed tests');
        //EmitCheckBox('failedonly','1',FonlyFailed);
        //DumpLn('Hide skipped tests');
        //EmitCheckBox('noskipped','1',FNoSkipped);
        Res:=true;
        EmitHistoryForm;
        if FTestFileID = '' then
          with FHTMLWriter do begin
            HeaderStart(2);
            if Trim(FTestFileName) <> '' then
              DumpLn(Format('Error: No test files matching "%s" found.', [FTestFileName]))
            else
              DumpLn('Error: Please specify a test file.');
            HeaderEnd(2);
            Res:=False;
          end;
      end;
    If Res then
      begin
        if (FTestFileName<>'') then
          begin
          HeaderStart(2);
          DumpLn('Test file "'+FTestFileName+'" information:');
          HeaderEnd(2);
          ParaGraphStart;
          S:='SELECT * FROM TESTS WHERE T_ID='+FTestFileID;
          Q:=CreateDataSet(S);
          With Q do
            Try
              Open;
              Try
                DumpTestInfo(Q);
              Finally
                Close;
              end;
            Finally
              Free;
            end;
          ParaGraphEnd;
          HeaderStart(2);
          DumpLn('Detailed test run results:');
        end;
      HeaderEnd(2);
      ParaGraphStart;
      SS:='SELECT TR_ID,TR_TESTRUN_FK AS Run,TR_TEST_FK,TR_OK AS OK'
        +', TR_SKIP As Skip,TR_RESULT  As Result'
      //S:='SELECT * '
        +',TC_NAME AS CPU, TV_VERSION AS Version, TO_NAME AS OS'
        +',TU_ID,TU_DATE AS Date,TU_SUBMITTER  AS Submitter'
        +',(TU_FAILEDTOCOMPILE + TU_FAILEDTOFAIL + TU_FAILEDTORUN) AS Fails'
        +',TU_MACHINE AS Machine,TU_COMMENT AS Comment'
        +',TU_COMPILERDATE As CompDate'
        +',TU_SVNTESTSREVISION AS Tests_rev'
        +',TU_SVNRTLREVISION AS RTL_rev'
        +',TU_SVNCOMPILERREVISION AS Compiler_rev'
        +',TU_SVNPACKAGESREVISION AS Packages_rev'
        +',TO_ID,TC_ID,TV_ID'
        +' FROM TESTRUN '
        +' LEFT JOIN TESTRESULTS ON  (TR_TESTRUN_FK=TU_ID)'
        +' LEFT JOIN TESTOS ON  (TU_OS_FK=TO_ID)'
        +' LEFT JOIN TESTCPU ON  (TU_CPU_FK=TC_ID)'
        +' LEFT JOIN TESTVERSION ON  (TU_VERSION_FK=TV_ID)';
      S:='';
      if FTestFileID<>'' then
        S:=S+' AND (TR_TEST_FK='+FTestFileID+')';
      if FRunID<>'' then
        S:=S+' AND (TR_TESTRUN_FK='+FRunID+')';
      If FOnlyFailed then
        S:=S+' AND (NOT TR_OK)';
      If FNoSkipped then
        S:=S+' AND (NOT TR_SKIP)';
      If FCond<>'' then
        S:=S+' AND ('+FCond+')';

      If (FCPU<>'') and (GetCPUName(FCPU)<>'All') then
        begin
          S:=S+' AND (TU_CPU_FK='+FCPU+')';
          cpu_size:=0;
        end
      else
        begin
          cpu_last:=StrToInt(GetSingleton('SELECT MAX(TC_ID) FROM TESTCPU'));
          cpu_size:=Sizeof(StatusLongintArray)*(1+cpu_last);
          cpu_count:=GetMem(cpu_size);
          FillChar(cpu_count^,cpu_size,#0);
          cpu_first_date_id:=GetMem(cpu_size);
          FillChar(cpu_first_date_id^,cpu_size,#0);
          cpu_last_date_id:=GetMem(cpu_size);
          FillChar(cpu_last_date_id^,cpu_size,#0);
          cpu_first_date:=GetMem(cpu_last*SizeOf(StatusDateTimeArray));
          FillChar(cpu_first_date^,cpu_last*Sizeof(StatusDateTimeArray),#0);
          cpu_last_date:=GetMem(cpu_last*SizeOf(StatusDateTimeArray));
          FillChar(cpu_last_date^,cpu_last*Sizeof(StatusDateTimeArray),#0);
        end;
      If (FVersion<>'') and (GetVersionName(FVersion)<>'All')  then
        begin
          S:=S+' AND (TU_VERSION_FK='+FVERSION+')';
          version_size:=0;
        end
      else
        begin
          version_last:=StrToInt(GetSingleton('SELECT MAX(TV_ID) FROM TESTVERSION'));
          version_size:=Sizeof(StatusLongintArray)*(1+version_last);
          version_count:=GetMem(version_size);
          FillChar(version_count^,version_size,#0);
          version_first_date_id:=GetMem(version_size);
          FillChar(version_first_date_id^,version_size,#0);
          version_last_date_id:=GetMem(version_size);
          FillChar(version_last_date_id^,version_size,#0);
          version_first_date:=GetMem(version_last*SizeOf(StatusDateTimeArray));
          FillChar(version_first_date^,version_last*Sizeof(StatusDateTimeArray),#0);
          version_last_date:=GetMem(version_last*SizeOf(StatusDateTimeArray));
          FillChar(version_last_date^,version_last*Sizeof(StatusDateTimeArray),#0);

        end;

      if (FOS<>'') and (GetOSName(FOS)<>'All') then
        begin
          S:=S+' AND (TU_OS_FK='+FOS+')';
          os_size:=0;
        end
      else
        begin
          os_last:=StrToInt(GetSingleton('SELECT MAX(TO_ID) FROM TESTOS'));
          os_size:=Sizeof(StatusLongintArray)*(1+os_last);
          os_count:=GetMem(os_size);
          FillChar(os_count^,os_size,#0);
          os_first_date_id:=GetMem(os_size);
          FillChar(os_first_date_id^,os_size,#0);
          os_last_date_id:=GetMem(os_size);
          FillChar(os_last_date_id^,os_size,#0);
          os_first_date:=GetMem(os_last*SizeOf(StatusDateTimeArray));
          FillChar(os_first_date^,os_last*Sizeof(StatusDateTimeArray),#0);
          os_last_date:=GetMem(os_last*SizeOf(StatusDateTimeArray));
          FillChar(os_last_date^,os_last*Sizeof(StatusDateTimeArray),#0);
        end;

      If FSubmitter<>'' then
        S:=S+' AND (TU_SUBMITTER='''+FSubmitter+''')';
      If FMachine<>'' then
        S:=S+' AND (TU_MACHINE='''+FMachine+''')';
      If FComment<>'' then
        S:=S+' AND (TU_COMMENT LIKE '''+FComment+''')';
      if FDATE<>0 then
        S:=S+' AND (TU_DATE >= '''+FormatDateTime('YYYY-MM-DD',FDate)+''')';

      if S <> '' then
      begin
        Delete(S, 1, 4);
        S:=SS + ' WHERE '+ S;
      end
      else
        S:=SS;

      S:=S+' ORDER BY TU_ID DESC';
      if FDATE=0 then
        S:=S+' LIMIT '+IntToStr(FLimit)
      else
        S:=S+' LIMIT '+IntToStr(MaxLimit);
      Qry:=S;
      If FDebug then
      begin
        Writeln(system.stdout,'Query : '+Qry);
        system.Flush(system.stdout);
      end;
      FRunCount:=0;
      FRunSkipCount:=0;
      FRunFailedCount:=0;
      Q:=CreateDataset(Qry);
      With Q do
        try
          Open;

          while not EOF do
            Next;

          DumpLn(Format('<p>Record count: %d </p>',[Q.RecordCount]));
          if RecordCount>0 then
            RecNo:=1;

          Try
           { if FDebug then
             begin
               Writeln(stdout,'FieldKind=',Fields[0].FieldKind);
               Writeln(stdout,'DataType=',Fields[0].DataType);
               system.flush(stdout);
             end; }

          total_count:=0;
          OK_count:=0;
          not_OK_count:=0;
          skip_count:=0;
          not_skip_count:=0;
          fillchar(Result_Count,Sizeof(Result_count),#0);
          ok_ind:=FieldByName('OK').Index;
          skip_ind:=FieldBYName('SKIP').Index;
          result_ind:=FieldByName('Result').Index;
          cpu_ind:=FieldByName('TC_ID').Index;
          os_ind:=FieldByName('TO_ID').Index;
          version_ind:=FieldByName('TV_ID').Index;
          date_ind:=FieldByName('Date').Index;
          run_ind:=FieldByName('TU_ID').Index;
          For i:=1 to Q.RecordCount do
            begin
              Q.RecNo:=i;
              inc(total_count);
              if Q.Fields[ok_ind].AsBoolean then
                inc(OK_count)
              else
                inc(not_OK_count);
              if Fields[skip_ind].AsBoolean then
                inc(skip_count)
              else
                inc(not_skip_count);
              S:=Fields[result_ind].AsString;
              cpu:=Fields[cpu_ind].ASString;
              version:=Fields[version_ind].AsString;
              os:=Fields[os_ind].AsString;
              date:=Fields[date_ind].ASDateTime;
              os_id:=Fields[os_ind].AsLongint;
              cpu_id:=Fields[cpu_ind].AsLongint;
              version_id:=Fields[version_ind].AsLongint;
              system.val(S,resi,error);
              run_id:=Fields[run_ind].ASLongint;
              if (error=0) and (Resi>=longint(FirstStatus)) and
                 (Resi<=longint(LastStatus)) then
                begin
                  TS:=TTestStatus(Resi);
                  if Result_count[TS]=0 then
                    begin
                      first_date[TS]:=date;
                      last_date[TS]:=date;
                      first_date_id[TS]:=run_id;
                      last_date_id[TS]:=run_id;
                    end
                  else
                    begin
                      if (date>last_date[TS]) then
                        begin
                          last_date[TS]:=date;
                          last_date_id[TS]:=run_id;
                        end;
                      if date<first_date[TS] then
                        begin
                          first_date[TS]:=date;
                          first_date_id[TS]:=run_id;
                        end;
                    end;

                  inc(Result_count[TS]);
                  if assigned(cpu_count) and (cpu_id<=cpu_last) then
                    begin
                      if cpu_count^[cpu_id,TS]=0 then
                        begin
                          cpu_first_date^[cpu_id,TS]:=date;
                          cpu_last_date^[cpu_id,TS]:=date;
                          cpu_first_date_id^[cpu_id,TS]:=run_id;
                          cpu_last_date_id^[cpu_id,TS]:=run_id;
                        end
                      else
                        begin
                          if (date>cpu_last_date^[cpu_id,TS]) then
                            begin
                              cpu_last_date^[cpu_id,TS]:=date;
                              cpu_last_date_id^[cpu_id,TS]:=run_id;
                            end;
                          if date<cpu_first_date^[cpu_id,TS] then
                            begin
                              cpu_first_date^[cpu_id,TS]:=date;
                              cpu_first_date_id^[cpu_id,TS]:=run_id;
                            end;
                        end;
                      inc(cpu_count^[cpu_id,TS]);
                    end;
                  if assigned(os_count) and (os_id<=os_last) then
                    begin
                      if os_count^[os_id,TS]=0 then
                        begin
                          os_first_date^[os_id,TS]:=date;
                          os_last_date^[os_id,TS]:=date;
                          os_first_date_id^[os_id,TS]:=run_id;
                          os_last_date_id^[os_id,TS]:=run_id;
                        end
                      else
                        begin
                          if (date>os_last_date^[os_id,TS]) then
                            begin
                              os_last_date^[os_id,TS]:=date;
                              os_last_date_id^[os_id,TS]:=run_id;
                            end;
                          if date<os_first_date^[os_id,TS] then
                            begin
                              os_first_date^[os_id,TS]:=date;
                              os_first_date_id^[os_id,TS]:=run_id;
                            end;
                        end;
                      inc(os_count^[os_id,TS]);
                    end;
                  if assigned(version_count) and (version_id<=version_last) then
                    begin
                      if version_count^[version_id,TS]=0 then
                        begin
                          version_first_date^[version_id,TS]:=date;
                          version_last_date^[version_id,TS]:=date;
                          version_first_date_id^[version_id,TS]:=run_id;
                          version_last_date_id^[version_id,TS]:=run_id;
                        end
                      else
                        begin
                          if (date>version_last_date^[version_id,TS]) then
                            begin
                              version_last_date^[version_id,TS]:=date;
                              version_last_date_id^[version_id,TS]:=run_id;
                            end;
                          if date<version_first_date^[version_id,TS] then
                            begin
                              version_first_date^[version_id,TS]:=date;
                              version_first_date_id^[version_id,TS]:=run_id;
                            end;
                        end;
                      inc(version_count^[version_id,TS]);
                    end;
                end
              else if Fdebug then
                writeln(stdout,'Error for Result, S=',S);
            end;
          DumpLn(Format('<p>Total = %d </p>',[total_count]));
          if Total_count > 0 then
            DumpLn(Format('<p>OK=%d Percentage= %3.2f </p>',[OK_count,OK_count*100/total_count]));
          if Skip_count > 0 then
            DumpLn(Format('<p>Skipped=%d Percentage= %3.2f </p>',[Skip_count,Skip_count*100/total_count]));
          if total_count>0 then
            begin
              TableStart(5,True);
              RowStart;
              CellStart;
              DumpLn('Result type');
              CellNext;
              DumpLn('Cat.');
              CellNext;
              DumpLn('Count');
              CellNext;
              DumpLn('Percentage');
              CellNext;
              DumpLn('First date');
              CellNext;
              DumpLn('Last Date');
              CellEnd;
            end;
          For TS:=FirstStatus to LastStatus do
            if Result_count[TS]>0 then
              begin
                RowNext;
                CellStart;
                DumpLn(StatusText[TS]);
                CellNext;
                CellNext;
                DumpLn(Format('%d',[Result_count[TS]]));
                CellNext;
                DumpLn(Format('%3.1f',[Result_count[TS]*100/total_count]));
                CellNext;

                DumpLn(FormatDetailURL(IntToStr(first_date_id[TS]),
                  DateTimeToStr(first_date[TS])));
                DumpLn(' '+GetFailCount(first_date_id[TS]));
                CellNext;
                DumpLn(FormatDetailURL(IntToStr(last_date_id[TS]),
                  DateTimeToStr(last_date[TS])));
                DumpLn(' '+GetFailCount(last_date_id[TS]));
                CellEnd;
                if assigned(cpu_count) then
                  begin
                    for i:=1 to cpu_last do
                      if cpu_count^[i,TS]>0 then
                        begin
                          RowNext;
                          CellStart;
                          CellNext;
                          DumpLn(GetSingleton('SELECT TC_NAME FROM TESTCPU WHERE TC_ID='+IntToStr(i)));
                          CellNext;
                          DumpLn(Format('%d',[cpu_count^[i,TS]]));
                          CellNext;
                          DumpLn(Format('%3.1f',[cpu_count^[i,TS]*100/result_count[TS]]));
                          CellNext;
                          DumpLn(FormatDetailURL(IntToStr(cpu_first_date_id^[i,TS]),
                            DateTimeToStr(cpu_first_date^[i,TS])));
                          DumpLn(' '+GetFailCount(cpu_first_date_id^[i,TS]));
                          CellNext;
                          DumpLn(FormatDetailURL(IntToStr(cpu_last_date_id^[i,TS]),
                            DateTimeToStr(cpu_last_date^[i,TS])));
                          DumpLn(' '+GetFailCount(cpu_last_date_id^[i,TS]));
                          CellEnd;
                        end;
                  end;
                if assigned(os_count) then
                  begin
                    for i:=1 to os_last do
                      if os_count^[i,TS]>0 then
                        begin
                          RowNext;
                          CellStart;
                          CellNext;
                          DumpLn(GetSingleton('SELECT TO_NAME FROM TESTOS WHERE TO_ID='+IntToStr(i)));
                          CellNext;
                          DumpLn(Format('%d',[os_count^[i,TS]]));
                          CellNext;
                          DumpLn(Format('%3.1f',[os_count^[i,TS]*100/result_count[TS]]));
                          CellNext;
                          DumpLn(FormatDetailURL(IntToStr(os_first_date_id^[i,TS]),
                            DateTimeToStr(os_first_date^[i,TS])));
                          DumpLn(' '+GetFailCount(os_first_date_id^[i,TS]));
                          CellNext;
                          DumpLn(FormatDetailURL(IntToStr(os_last_date_id^[i,TS]),
                            DateTimeToStr(os_last_date^[i,TS])));
                          DumpLn(' '+GetFailCount(os_last_date_id^[i,TS]));
                          CellEnd;

                        end;

                  end;

                if assigned(version_count) then
                  begin
                    for i:=1 to version_last do
                      if version_count^[i,TS]>0 then
                        begin
                          RowNext;
                          CellStart;
                          CellNext;
                          DumpLn(GetSingleton('SELECT TV_VERSION FROM TESTVERSION WHERE TV_ID='+IntToStr(i)));
                          CellNext;
                          DumpLn(Format('%d',[version_count^[i,TS]]));
                          CellNext;
                          DumpLn(Format('%3.1f',[version_count^[i,TS]*100/result_count[TS]]));
                          CellNext;
                          DumpLn(FormatDetailURL(IntToStr(version_first_date_id^[i,TS]),
                            DateTimeToStr(version_first_date^[i,TS])));
                          DumpLn(' '+GetFailCount(version_first_date_id^[i,TS]));
                          CellNext;
                          DumpLn(FormatDetailURL(IntToStr(version_last_date_id^[i,TS]),
                            DateTimeToStr(version_last_date^[i,TS])));
                          DumpLn(' '+GetFailCount(version_last_date_id^[i,TS]));
                          CellEnd;

                        end;

                  end;

              end;
          if total_count>0 then
            begin
              TableEnd;
              RecNo:=1;
            end;
          If FDebug or FListAll then
           begin

            With CreateTableProducer(Q) do
              Try
                Border:=True;
                FL:='RUN,Date,OK,SKIP,Result';
                if FSubmitter='' then
                  FL:=FL+',Submitter';
                if FMachine='' then
                  FL:=FL+',Machine';
                if Fcomment='' then
                  FL:=FL+',Comment';
                if (FOS='') or (GetOSName(FOS)='All') then
                  FL:=FL+',OS';
                if (FCPU='') or (GetCPUName(FCPU)='All') then
                  FL:=FL+',CPU';
                if (FVersion='') or (GetVersionName(FVersion)='All') then
                  FL:=FL+',Version';
                FL:=FL+',Fails,CompDate';
                FL:=FL+',Tests_rev,RTL_rev,Compiler_rev,Packages_rev';
                CreateColumns(FL);
                //TableColumns.Delete(TableColumns.ColumnByName('TR_TEST_FK').Index);
                TableColumns.ColumnByNAme('RUN').OnGetCellContents:=
                  @FormatTestRunOverview;
                //OnGetRowAttributes:=@GetRunRowAttr;
                TableColumns.ColumnByNAme('Result').OnGetCellContents:=
                  @FormatTestResult;
                //(TableColumns.Items[0] as TTableColumn).ActionURL:=ALink;
                CreateTable(Response);
              Finally
                Free;
              end;
           end;

          Finally
            Close;
          end;
        finally
          Free;
        end;
             //If FDebug then
            Category:='1';
            if FRunId<>'' then
              begin
                Category:=getsingleton('select TU_CATEGORY_FK from TESTRUN where TU_ID='+FRunId);
                FVersionBranch:=GetVersionName(getsingleton('select TU_VERSION_FK from TESTRUN where TU_ID='+fRunId));
                LLog:='';
                Try
                LLog:=getsingleton('select TR_LOG from TESTRESULTS where (TR_TEST_FK='+ftestfileid
                     +') and (TR_TESTRUN_FK='+frunid+')');
                if LLog<>'' then
                  begin
                    HeaderStart(2);
                    DumpLn('LLog of '+FRunId+':');
                    HeaderEnd(2);
                    PreformatStart;
                    system.Write(LLog);
                    system.flush(output);
                    PreformatEnd;
                  end;
                Finally
                  if LLog='' then
                    begin
                      HeaderStart(2);
                      DumpLn('No log of '+FRunId+'.');
                      HeaderEnd(2);
                    end;
                end;
              end;
            if FCompareRunId<>'' then
              begin
                LLog:='';
                Try
                LLog:=getsingleton('select TR_LOG from TESTRESULTS where (TR_TEST_FK='+ftestfileid
                     +') and (TR_TESTRUN_FK='+fcomparerunid+')');
                if LLog<>'' then
                  begin
                    HeaderStart(2);
                    DumpLn('Log of '+FCompareRunId+':');
                    HeaderEnd(2);
                    PreformatStart;
                    system.Write(LLog);
                    system.flush(output);
                    PreformatEnd;
                  end;
                Finally
                  if LLog='' then
                    begin
                      HeaderStart(2);
                      DumpLn('No log of '+FCompareRunId+'.');
                      HeaderEnd(2);
                    end;
                end;
              end;
            if FDebug then
              DumpLn('After log.');
            Source:='';
            Try
              if ftestfileid <> '' then
              begin
                Source:=getsingleton('select T_SOURCE from TESTS where T_ID='+ftestfileid);
                if Source<>'' then
                  begin
                    HeaderStart(2);
                    DumpLn('Source:');
                    HeaderEnd(2);
                    PreformatStart;
                    system.Write(Source);
                    system.flush(output);
                    PreformatEnd;
                  end;
              end;
            Finally
            Base:='trunk';
            if  FVersionBranch<>'' then
              begin
                // Test all but last version, which is assumed to be trunk
                for ver:=low(known_versions) to pred(high(known_versions)) do
                  if ver_string[ver]=FVersionBranch then
                    begin
                      base:=ver_branch[ver];
                      break;
                    end;
              end;
            FViewVCURL:=ViewURL+Base;
            if Category='1' then
              FViewVCUrl:=FViewVCURL+TestsSubDir
            else
              begin
                FViewVCUrl:=FViewVCURL+DataBaseSubDir;
                // This assumes that type TAnyType is
                // defined in anytype.pas source PM
                if pos('/',FTestFileName)>0 then
                  FTestfilename:=lowercase(copy(FTestFilename,2,pos('/',FTestFilename)-2)+'.pas');
              end;
            if Source='' then
              begin
                HeaderStart(3);
                DumpLn('<P>No Source in TestSuite DataBase.</P>');
                DumpLn('Link to SVN view of '+
                  '<A HREF="'+FViewVCURL+FTestFileName+'?view=markup'+
                  '" TARGET="fpc_source"> '+FTestFileName+'</A> source. ');
                HeaderEnd(3);
              end
            else
              begin
                HeaderStart(3);
                DumpLn('Link to SVN view of '+
                  '<A HREF="'+FViewVCURL+FTestFileName+'?view=markup'+
                  '" TARGET="fpc_source"> '+FTestFileName+'</A> source. ');
                HeaderEnd(3);
              end;
            end;
             if FDebug then
              DumpLn('After Source.');
    end;

    end;
  if assigned(os_count) then
    begin
      FreeMem(os_count);
      FreeMem(os_first_date);
      FreeMem(os_first_date_id);
      FreeMem(os_last_date);
      FreeMem(os_last_date_id);
    end;

  if assigned(cpu_count) then
    begin
      FreeMem(cpu_count);
      FreeMem(cpu_first_date);
      FreeMem(cpu_first_date_id);
      FreeMem(cpu_last_date);
      FreeMem(cpu_last_date_id);
    end;
  if assigned(version_count) then
    begin
      FreeMem(version_count);
      FreeMem(version_first_date);
      FreeMem(version_first_date_id);
      FreeMem(version_last_date);
      FreeMem(version_last_date_id);
    end;
end;

Procedure TTestSuite.ShowRunComparison;

Var
  S : String;
  Qry : String;
  Q : TSQLQuery;
  FL : String;

begin
  ConnectToDB;
  ContentType:='text/html';
  EmitContentType;
  EmitDocType;
  EmitTitle(Title+' : Compare 2 runs');
  With FHTMLWriter do
    begin
    HeaderStart(1);
    DumpLn('Test suite results for run '+FRunID+' vs. '+FCompareRunID);
    HeaderEnd(1);
    HeaderStart(2);
    DumpLn('Test run data: ');
    HeaderEnd(2);
    If ShowRunData then
      begin
      HeaderStart(2);
      DumpLn('Detailed test run results:');

      FL:='';
      If FOnlyFailed or FNoSkipped then
        begin
        FL:='';
        If FOnlyFailed then
          FL:='successful';
        if FNoSkipped then
          begin
          If (FL<>'') then
            FL:=FL+' and ';
          FL:=FL+'skipped';
          end;
        DumpLn(' ('+FL+' tests are hidden)');
        end;
      HeaderEnd(2);
      ParaGraphStart;
      Q:=CreateDataset('');
      S:='with tr1 as (SELECT * FROM '+TESTRESULTSTableName(FRunId)+ ' WHERE TR_TESTRUN_FK='+FRunID+'), '+
         '  tr2 as (SELECT * FROM '+TESTRESULTSTableName(FCompareRunId)+' WHERE TR_TESTRUN_FK='+FCompareRunID+')'+
         ' SELECT T_ID as id,T_NAME as Filename,tr1.TR_SKIP as Run1_Skipped,'
         +'tr2.TR_SKIP as Run2_Skipped,tr1.TR_OK as Run1_OK,'
         +'tr2.TR_OK as Run2_OK, tr1.TR_Result as Run1_Result,'
         +'tr2.TR_RESULT as Run2_Result '
         +'FROM TESTS, tr2 LEFT JOIN tr1 USING (TR_TEST_FK) '
         +'WHERE ((tr1.TR_SKIP IS NULL) or (tr2.TR_SKIP IS NULL) or '
         +' (%s (tr1.TR_Result<>tr2.TR_Result)))'
         +'and (T_ID=tr2.TR_TEST_FK)';
      If FNoSkipped then
        begin
        Qry:='(((tr1.TR_SKIP) and (not tr2.TR_OK) and (not tr2.TR_SKIP)) or '
           +'((not tr1.TR_OK) and (not tr1.TR_SKIP) and (tr2.TR_SKIP)) or '
           +'((not tr1.TR_SKIP) and (not tr2.TR_SKIP))) and ';
        end
      else
        Qry:='';
      Qry:=Format(S,[Qry]);
//      DumpLn(Qry);
      If FDebug then
        begin
        system.WriteLn('Query: '+Qry);
        system.Flush(stdout);
        end;
      FRunCount:=0;
      FRunSkipCount:=0;
      FRunFailedCount:=0;
      Q.SQL.Text:=Qry;
      With Q do
        try
          Open;
          Try
            With CreateTableProducer(Q) do
              Try
                Border:=True;
                FL:='Id,Filename,Run1_OK,Run2_OK';
                If Not FNoSkipped then
                  FL:=FL+',Run1_Skipped,Run2_Skipped';
                FL:=FL+',Run1_Result,Run2_Result';
                CreateColumns(FL);
                OnGetRowAttributes:=@GetRunRowAttr;
                TableColumns.ColumnByNAme('Id').OnGetCellContents:=
                  @FormatFileIDDetails;
                TableColumns.ColumnByNAme('Run1_Result').OnGetCellContents:=
                  @FormatTestResult;
                TableColumns.ColumnByNAme('Run2_Result').OnGetCellContents:=
                  @FormatTestResult;
                TableColumns.ColumnByNAme('Filename').OnGetCellContents:=
                 @FormatFileDetails;
                //(TableColumns.Items[0] as TTableColumn).ActionURL:=ALink;
                CreateTable(Response);
              Finally
                Free;
              end;
            DumpLn(format('<p>Record count: %d</P>',[Q.RecordCount]));
          Finally
            Close;
          end;
        finally
          Free;
        end;
      If Not (FRunCount=0) and not (FNoSkipped or FOnlyFailed) then
        begin
        ParaGraphStart;
        TagStart('IMG',Format('Src="'+TestsuiteCGIURL+
        '?action=2&amp;pietotal=%d&amp;piefailed=%d&amp;pieskipped=%d"'+
        ' ALT="total=%d, failed=%d, skipped=%d"',
        [FRunCount,FRunFailedCount,FRunSkipCount,
          FRunCount,FRunFailedCount,FRunSkipCount
          ]));
        end;
      end
    else
      DumpLn('No data for test run with ID: '+FRunID);
    end;
end;

procedure TTestSuite.GetRunRowAttr(Sender: TObject; var BGColor: String;
  var Align: THTMLAlign; var VAlign: THTMLValign; var CustomAttr: String);

Var
  P : TTableProducer;
  Skip1Field, Skip2Field, Run1Field, Run2Field : TField;
begin
  P:=(Sender as TTAbleProducer);
  Inc(FRunCount);
  If (FOnlyFailed and FNoSkipped) then
    begin
    If (P.CurrentRow Mod 2)=0 then
      BGColor:='#EEEEEE'
    end
  else
    begin
    Skip1Field := P.Dataset.FindField('Skipped');
    if Skip1Field = nil then
      begin
      Skip1Field := P.Dataset.FindField('Run1_Skipped');
      Skip2Field := P.Dataset.FindField('Run2_Skipped');
      end
    else
      Skip2Field := nil;
    Run1Field := P.Dataset.FindField('OK');
    if Run1Field = nil then
      Run1Field := P.Dataset.FindField('Run1_OK');
    Run2Field := P.Dataset.FindField('OK');
    if Run2Field = nil then
      Run2Field := P.Dataset.FindField('Run2_OK');
    If (not FNoSkipped) and ((Skip1Field.AsBoolean)
        or ((Skip2Field <> nil) and (Skip2Field.AsBoolean))) then
      begin
      Inc(FRunSkipCount);
      BGColor:='yellow';    // Yellow
      end
    else If Run2Field.AsBoolean then
      begin
      if Run1Field.AsString='' then
        BGColor:='#68DFB8'
      else if Run1Field.AsBoolean then
        BGColor:='#98FB98';    // pale Green
      end
    else if Not Run2Field.AsBoolean then
      begin
      Inc(FRunFailedCount);
      if Run1Field.AsString='' then
        BGColor:='#FF82AB'    // Light red
      else if Not Run1Field.AsBoolean then
        BGColor:='#FF225B';
      end;
    end;
end;

procedure TTestSuite.FormatFailedOverview(Sender: TObject; var CellData: String);

Var
  S: String;
  P : TTableProducer;

begin
  P:=(Sender as TTableProducer);
  S:=Format(SDetailsURL,[P.DataSet.FieldByName('ID').AsString]);
  S:=S+'&amp;failedonly=1&amp;noskipped=1';
  CellData:=Format('<A HREF="%s">%s</A>',[S,CellData]);
end;


function TTestSuite.FormatDetailURL(const RunIdStr, CellData : String) : string;
Var
  S : String;
begin
  S:=Format(SDetailsURL,[RunIdStr]);
  if FOnlyFailed then
    S:=S+'&amp;failedonly=1';
  if FNoSkipped then
    S:=S+'&amp;noskipped=1';
  FormatDetailURL:=Format('<A HREF="%s">%s</A>',[S,CellData]);
end;

procedure TTestSuite.FormatTestRunOverview(Sender: TObject; var CellData: String);
Var
  S: String;
  P : TTableProducer;
begin
  P:=(Sender as TTableProducer);
  S:=Format(SDetailsURL,[P.DataSet.FieldByName('RUN').AsString]);
  if FOnlyFailed then
    S:=S+'&amp;failedonly=1';
  if FNoSkipped then
    S:=S+'&amp;noskipped=1';
  CellData:=Format('<A HREF="%s">%s</A>',[S,CellData]);
end;

procedure TTestSuite.FormatSVN(Sender: TObject; var CellData: String);
begin
  FormatSVNData(CellData);
end;

procedure TTestSuite.FormatSVNData(var CellData: String);
Var
  S, Rev, SubStr, Remaining : String;
  P : TTableProducer;
  pos_colon, pos_sep : longint;
begin
  pos_sep:=pos('/', CellData);
  if pos_sep=0 then
    begin
      pos_colon:=pos(':',CellData);
      S:=ViewRevURL+copy(CellData,pos_colon+1,length(CellData));
      CellData:=Format('<A HREF="%s" target="_blank">%s</A>',[S,CellData]);
    end
  else
    begin
      SubStr:=Copy(CellData,1,pos_sep-1);
      Remaining:=Copy(CellData,pos_sep+1,length(CellData));
      CellData:='';
      while SubStr<>'' do
        begin
          pos_colon:=pos(':',SubStr);
          Rev:=copy(SubStr,pos_colon+1,length(SubStr));
          { Remove suffix like M for modified...}
          while (length(Rev)>0) and (not (Rev[length(Rev)] in ['0'..'9'])) do
            Rev:=Copy(Rev,1,length(Rev)-1);
          S:=ViewRevURL+Rev;
          CellData:=CellData+Format('<A HREF="%s" target="_blank">%s</A>',[S,SubStr]);
          if Remaining='' then
            SubStr:=''
          else
            begin
              pos_sep:=pos('/',Remaining);
              if pos_sep=0 then
                pos_sep:=length(Remaining)+1;
              CellData:=CellData+':';
              SubStr:=Copy(Remaining,1,pos_sep-1);
              Remaining:=Copy(Remaining,pos_sep+1,length(Remaining));
            end;
        end;
    end;
end;

procedure TTestSuite.FormatFileIDDetails(Sender: TObject; var CellData: String);

Var
  S: String;
  P : TTableProducer;

begin
  P:=(Sender as TTableProducer);
  if FVersion<>'' then
    S:=Format(TestSuiteCGIURL + '?action=%d&amp;version=%s&amp;testfileid=%s',
       [faction_show_history,FVersion,P.DataSet.FieldByName('Id').AsString])
  else
    S:=Format(TestSuiteCGIURL + '?action=%d&amp;testfileid=%s',
       [faction_show_history,P.DataSet.FieldByName('Id').AsString]);
  CellData:=Format('<A HREF="%s">%s</A>',[S,CellData]);
end;


procedure TTestSuite.FormatFileDetails(Sender: TObject; var CellData: String);

Var
  S: String;
  P : TTableProducer;

begin
  P:=(Sender as TTableProducer);
  if FCompareRunID<>'' then
    S:=Format(TestSuiteCGIURL + '?action=%d&amp;run1id=%s&amp;run2id=%s&amp;testfileid=%s',
       [faction_show_one_test,FRunID,FCompareRunID,P.DataSet.FieldByName('Id').AsString])
  else
    S:=Format(TestSuiteCGIURL + '?action=%d&amp;run1id=%s&amp;testfileid=%s',
       [faction_show_one_test,FRunID,P.DataSet.FieldByName('Id').AsString]);
  CellData:=Format('<A HREF="%s">%s</A>',[S,CellData]);
end;

procedure TTestSuite.FormatTestResult(Sender: TObject; var CellData: String);

Var
  Res : longint;
  Error:word;
  TS : TTestStatus;
begin
  Val(CellData,Res,Error);
  if (Error=0) and (Res>=longint(FirstStatus)) and
     (Res<=longint(LastStatus)) then
    begin
      TS:=TTestStatus(Res);
      CellData:=StatusText[TS];
    end;
end;

Procedure TTestSuite.CreateRunPie;

Var
  I : TFPMemoryImage;
  M : TMemoryStream;

begin
  ftFont.InitEngine;
  FontMgr.SearchPath:='/usr/lib/X11/fonts/truetype';
  I:=TFPMemoryImage.Create(320,320);
  try
    If FRunCount=0 Then
      Raise Exception.Create('Invalid parameters passed to script: No total count');
    DoDrawPie(I,FRunSkipCount,FRunFailedCount,FRunCount);
    M:=TMemoryStream.Create;
    Try
      With TFPWriterPNG.Create do
        try
          UseAlpha:=True;
          ImageWrite(M,I);
        Finally
          Free;
        end;
      ContentType:='image/png';
      //EmitDocType;
      EmitContentType;
      M.Position:=0;
      Response.CopyFrom(M,M.Size);
    Finally
      M.Free;
    end;
  Finally
    I.Free;
  end;
end;

Procedure TTestSuite.DoDrawPie(Img : TFPCustomImage; Skipped,Failed,Total : Integer);

Var
  Cnv : TFPImageCanvas;

  Procedure AddPie(X,Y,R : Integer; AStart,AStop : Double; Col : TFPColor);

  Var
    DX,Dy : Integer;

  begin
    DX:=Round(R*Cos(AStart));
    DY:=Round(R*Sin(AStart));
    Cnv.Line(X,Y,X+DX,Y-DY);
    DX:=Round(R*Cos(AStop));
    DY:=Round(R*Sin(AStop));
    Cnv.Line(X,Y,X+DX,Y-Dy);
    DX:=Round(R/2*Cos((AStart+AStop)/2));
    DY:=Round(R/2*Sin((AStart+AStop)/2));
    Cnv.Brush.FpColor:=Col;
    Cnv.FloodFill(X+DX,Y-DY);
  end;

  Function FractionAngle(F,T : Integer): Double;

  begin
    Result:=(2*Pi*(F/T))
  end;

Var
  W,H,FH,CR,RA : Integer;
  A1,A2,FR,SR,PR : Double;
  R : TRect;
  F : TFreeTypeFont;

begin
  F:=TFreeTypeFont.Create;
  With F do
    begin
    Name:='arial';
    FontIndex:=0;
    Size:=12;
    FPColor:=colred;
    AntiAliased:=False;
    Resolution:=96;
    end;
  if FDebug then
    Writeln(stdout,'Creating image');
  Cnv:=TFPImageCanvas.Create(Img);
  if FDebug then
    Writeln(stdout,'CNV=0x',hexstr(ptruint(cnv),16));

  if FDebug then
   Writeln(stdout,'Getting width and height');
  W:=Img.Width;
  H:=Img.Height;
  if FDebug then
    begin
      Writeln(stdout,'width=',W,' height=',H);
      //system.flush(stdout);
    end;
  // Writeln('Transparant');
  cnv.Brush.Style:=bsSolid;
  cnv.Brush.FPColor:=colTransparent;
  cnv.Pen.FPColor:=colWhite;
  Cnv.Rectangle(0,0,W,H);
  if FDEbug then
    Writeln(stdout,'Setting font');
  Cnv.Font:=F;
  if FDebug then
    Writeln(stdout,'Getting textwidth ');
  FH:=CNV.GetTextHeight('A');
  If FH=0 then
    FH:=14; // 3 * 14;
  if FDebug then
    writeln(stdout,'FH=',FH);
  Inc(FH,3);
  R.Top:=FH*4;
  R.Left:=0;
  R.Bottom:=H;
  CR:=H-(FH*4);
  If W>CR then
    R.Right:=CR
  else
    R.Right:=W;
  Ra:=CR div 2;
  if FDEbug then
    begin
      Writeln(stdout,'Setting pen color');
      system.flush(stdout);
    end;
  Cnv.Pen.FPColor:=colBlack;
  if FDebug then
    begin
      Writeln(stdout,'Palette size : ',Img.Palette.Count);
      Writeln(stdout,'Setting brush style');
      system.flush(stdout);
    end;
  cnv.brush.FPColor:=colDkGray;
  SR:=Skipped/Total;
  FR:=Failed/Total;
  PR:=1-SR-FR;
  cnv.font.FPColor:=colDkGray;
  Cnv.Textout(1,FH*2,Format('%d Skipped (%3.1f%%)',[Skipped,SR*100]));
//  cnv.pen.width:=1;
  // Writeln('Drawing ellipse');
  Cnv.Ellipse(R);
  if FDebug then
    begin
      Writeln(stdout,'Setting text');
      system.flush(stdout);
    end;
  A1:=0;
  A2:=A1+FractionAngle(Failed,Total);
  cnv.font.FPColor:=colRed;
  Cnv.Textout(1,FH*3,Format('%d Failed (%3.1f%%)',[Failed,FR*100]));
  AddPie(Ra,R.Top+Ra,Ra,A1,A2,ColRed);
  cnv.font.FPColor:=colGreen;
  Cnv.Textout(1,FH,Format('%d Passed (%3.1f%%)',[Total-Skipped-Failed,PR*100]));
  // Writeln('Palette size : ',Img.Palette.Count);
  A1:=A2;
  A2:=A1+FractionAngle(Total-(Skipped+Failed),Total);
  AddPie(Ra,R.Top+Ra,Ra,A1,A2,ColGreen);
  // Writeln('Palette size : ',Img.Palette.Count);
  // Writeln('All done');
end;

begin
  if paramstr(0)<>'' then
    TestsuiteCGIURL:=TestsuiteURLPrefix+'cgi-bin/'+extractfilename(paramstr(0))
  else
    TestsuiteCGIURL:=TestsuiteURLPrefix+'cgi-bin/'+TestsuiteBin;

  ShortDateFormat:='yyyy/mm/dd';
end.
