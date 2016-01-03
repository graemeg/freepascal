program dbtestframework;

{$IFDEF FPC}
  {$mode objfpc}{$H+}
{$ENDIF}

{$APPTYPE CONSOLE}

uses
  SysUtils,
  fpcunit,  testreport, testregistry,
  DigestTestReport,
  toolsunit,
<<<<<<< HEAD
<<<<<<< HEAD
// List of supported database connectors
  sqldbtoolsunit,
  dbftoolsunit,
  bufdatasettoolsunit,
  memdstoolsunit,
  SdfDSToolsUnit,
  tcsdfdata,
// Units wich contain the tests
  TestBasics,
  TestDBBasics,
  TestFieldTypes,
  TestDatasources,
  TestBufDatasetStreams,
  TestSQLDB,
  TestSpecificTBufDataset,
  TestSpecificTDBF,
  TestSpecificTMemDataset,
  TestDBExport, 
  tccsvdataset,
  testsqlscript,
  consoletestrunner;

Procedure LegacyOutput;

=======
=======
>>>>>>> origin/fixes_2_2
// Units wich contains the tests
  testbasics, testfieldtypes, TestDatasources, testdbbasics;
  
>>>>>>> graemeg/fixes_2_2
var
  FXMLResultsWriter: TXMLResultsWriter;
  FDigestResultsWriter: TDigestResultsWriter;
  testResult: TTestResult;

begin
  InitialiseDBConnector;
  testResult := TTestResult.Create;
  FXMLResultsWriter := TXMLResultsWriter.Create;
  FDigestResultsWriter := TDigestResultsWriter.Create(nil);
  try
    testResult.AddListener(FXMLResultsWriter);
    testResult.AddListener(FDigestResultsWriter);
    FDigestResultsWriter.Comment:=dbtype;
    FDigestResultsWriter.Category:='DB';
    FDigestResultsWriter.RelSrcDir:='fcl-db';
    FXMLResultsWriter.WriteHeader;
//    FdiDBResultsWriter.OpenConnection(dbconnectorname+';'+dbconnectorparams);
    GetTestRegistry.Run(testResult);
    FXMLResultsWriter.WriteResult(testResult);
  finally
    testResult.Free;
    FXMLResultsWriter.Free;
    FDigestResultsWriter.Free;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
end;
  
Var
  Application : TTestRunner;  
  
begin
  InitialiseDBConnector;
  Try
    Application:=TTestRunner.Create(nil);
    With Application do
      try
        if HasOption('g','legacy') then
          LegacyOutput
        else
          begin  
          DefaultFormat:=fplain;
          DefaultRunAllTests:=True;
          Initialize;
          Run;
          end;
      finally
        Free;
      end;
  Finally    
    FreeDBConnector;
  end;  
=======
  FreeDBConnector;
>>>>>>> graemeg/cpstrnew
=======
  FreeDBConnector;
>>>>>>> graemeg/cpstrnew
=======
  FreeDBConnector;
>>>>>>> graemeg/cpstrnew
=======
  FreeDBConnector;
>>>>>>> origin/cpstrnew
end.
