unit TestSQLDB;

{
  Unit tests which are specific to the sqlDB components like TSQLQuery, TSQLConnection.
}

{$mode objfpc}{$H+}

interface

uses
  Classes, sqldb, SysUtils, fpcunit, testregistry,
  sqldbtoolsunit,toolsunit, db;

type

  { TSQLDBTestCase }

  TSQLDBTestCase = class(TTestCase)
    private
      function GetSQLDBConnector: TSQLDBConnector;
    protected
      procedure SetUp; override;
      procedure TearDown; override;
      Property SQLDBConnector : TSQLDBConnector Read GetSQLDBConnector;
  end;

  { TTestTSQLQuery }

  TTestTSQLQuery = class(TSQLDBTestCase)
  private
    FMyQ: TSQLQuery;
    FPrepareCount:Integer;
    procedure CreateAndFillIDField;
    procedure DoAfterPost(DataSet: TDataSet);
    Procedure DoApplyUpdates;
    procedure DoCount(Sender: TSQLConnection; EventType: TDBEventType; const Msg: String);
    Procedure TrySetQueryOptions;
    Procedure TrySetPacketRecords;
  Protected
    Procedure Setup; override;
  published
    procedure TestMasterDetail;
    procedure TestUpdateServerIndexDefs;
    Procedure TestKeepOpenOnCommit;
    Procedure TestKeepOpenOnCommitPacketRecords;
    Procedure TestCheckSettingsOnlyWhenInactive;
    Procedure TestAutoApplyUpdatesPost;
    Procedure TestAutoApplyUpdatesDelete;
    Procedure TestCheckRowsAffected;
    Procedure TestAutoCommit;
    Procedure TestGeneratedRefreshSQL;
    Procedure TestGeneratedRefreshSQL1Field;
    Procedure TestGeneratedRefreshSQLNoKey;
    Procedure TestRefreshSQL;
    Procedure TestRefreshSQLMultipleRecords;
    Procedure TestRefreshSQLNoRecords;
    Procedure TestFetchAutoInc;
    procedure TestSequence;
    procedure TestReturningInsert;
    procedure TestReturningUpdate;
    procedure TestMacros;
    Procedure TestPrepareCount;
    Procedure TestPrepareCount2;
  end;

  { TTestTSQLConnection }

  TTestTSQLConnection = class(TSQLDBTestCase)
  private
    procedure SetImplicit;
    procedure TestImplicitTransaction;
    procedure TestImplicitTransaction2;
    procedure TestImplicitTransactionNotAssignable;
    procedure TestImplicitTransactionOK;
    procedure TryOpen;
  published
    procedure TestUseImplicitTransaction;
    procedure TestUseExplicitTransaction;
    procedure TestExplicitConnect;
    procedure TestGetStatementInfo;
    procedure TestGetNextValue;
  end;

  { TTestTSQLScript }

  TTestTSQLScript = class(TSQLDBTestCase)
  published
    procedure TestExecuteScript;
    procedure TestScriptColon; //bug 25334
    procedure TestUseCommit; //E.g. Firebird cannot use COMMIT RETAIN if mixing DDL and DML in a script
  end;

implementation



{ TTestTSQLQuery }

procedure TTestTSQLQuery.Setup;
begin
  inherited Setup;
  FPrepareCount:=0;
  SQLDBConnector.Connection.Options:=[];
end;

procedure TTestTSQLQuery.TestMasterDetail;
var MasterQuery, DetailQuery: TSQLQuery;
    MasterSource: TDataSource;
begin
  with SQLDBConnector do
  try
    MasterQuery := GetNDataset(10) as TSQLQuery;
    MasterSource := TDatasource.Create(nil);
    MasterSource.DataSet := MasterQuery;
    DetailQuery := Query;
    DetailQuery.SQL.Text := 'select NAME from FPDEV where ID=:ID';
    DetailQuery.DataSource := MasterSource;

    MasterQuery.Open;
    DetailQuery.Open;
    CheckEquals('TestName1', DetailQuery.Fields[0].AsString);
    MasterQuery.MoveBy(3);
    CheckEquals('TestName4', DetailQuery.Fields[0].AsString);

    MasterQuery.Close;
    CheckTrue(DetailQuery.Active, 'Detail dataset should remain intact, when master dataset is closed');
  finally
    MasterSource.Free;
  end;
end;

procedure TTestTSQLQuery.TestUpdateServerIndexDefs;
var Q: TSQLQuery;
    name1, name2, name3: string;
begin
  // Test retrieval of information about indexes on unquoted and quoted table names
  //  (tests also case-sensitivity for DB's that support case-sensitivity of quoted identifiers)
  // For ODBC Firebird/Interbase we must define primary key as named constraint and
  //  in ODBC driver must be set: "quoted identifiers" and "sensitive identifier"
  // See also: TTestFieldTypes.TestUpdateIndexDefs
  with SQLDBConnector do
  begin
    // SQLite ignores case-sensitivity of quoted table names
    // MS SQL Server case-sensitivity of identifiers depends on the case-sensitivity of default collation of the database
    // MySQL case-sensitivity depends on case-sensitivity of server's file system
    if SQLServerType in [ssMSSQL,ssSQLite{$IFDEF WINDOWS},ssMySQL{$ENDIF}] then
      name1 := Connection.FieldNameQuoteChars[0]+'fpdev 2'+Connection.FieldNameQuoteChars[1]
    else
      name1 := 'FPDEV2';
    ExecuteDirect('create table '+name1+' (id integer not null, constraint PK_FPDEV21 primary key(id))');
    // same but quoted table name
    name2 := Connection.FieldNameQuoteChars[0]+'FPdev2'+Connection.FieldNameQuoteChars[1];
    ExecuteDirect('create table '+name2+' (ID2 integer not null, constraint PK_FPDEV22 primary key(ID2))');
    // embedded quote in table name
    if SQLServerType in [ssMySQL] then
      name3 := '`FPdev``2`'
    else
      name3 := Connection.FieldNameQuoteChars[0]+'FPdev""2'+Connection.FieldNameQuoteChars[1];
    ExecuteDirect('create table '+name3+' (Id3 integer not null, constraint PK_FPDEV23 primary key(Id3))');
    CommitDDL;
  end;

  try
    Q := SQLDBConnector.Query;
    Q.SQL.Text:='select * from '+name1;
    Q.Prepare;
    Q.ServerIndexDefs.Update;
    CheckEquals(1, Q.ServerIndexDefs.Count);

    Q.SQL.Text:='select * from '+name2;
    Q.Prepare;
    Q.ServerIndexDefs.Update;
    CheckEquals(1, Q.ServerIndexDefs.Count, '2.1');
    CheckTrue(CompareText('ID2', Q.ServerIndexDefs[0].Fields)=0, '2.2'+Q.ServerIndexDefs[0].Fields);
    CheckTrue(Q.ServerIndexDefs[0].Options=[ixPrimary,ixUnique], '2.3');

    Q.SQL.Text:='select * from '+name3;
    Q.Prepare;
    Q.ServerIndexDefs.Update;
    CheckEquals(1, Q.ServerIndexDefs.Count, '3.1');
    CheckTrue(CompareText('ID3', Q.ServerIndexDefs[0].Fields)=0, '3.2');
    CheckTrue(Q.ServerIndexDefs[0].Options=[ixPrimary,ixUnique], '3.3');
  finally
    Q.UnPrepare;
    with SQLDBConnector do
    begin
      ExecuteDirect('DROP TABLE '+name1);
      ExecuteDirect('DROP TABLE '+name2);
      ExecuteDirect('DROP TABLE '+name3);
      CommitDDL;
    end;
  end;
end;

procedure TTestTSQLQuery.TestKeepOpenOnCommit;
var Q: TSQLQuery;
    I: Integer;
begin
  // Test that for a SQL query with Options=sqoKeepOpenOnCommit, calling commit does not close the dataset.
  // Test also that an edit still works.
  with SQLDBConnector do
    begin
    ExecuteDirect('create table FPDEV2 (id integer not null, a varchar(10), constraint PK_FPDEV2 primary key(id))');
    Transaction.Commit;
    for I:=1 to 20 do
      ExecuteDirect(Format('INSERT INTO FPDEV2 values (%d,''%.6d'')',[i,i]));
    Transaction.Commit;

    Q := SQLDBConnector.Query;
    Q.SQL.Text:='select * from FPDEV2';
    Q.Options:=[sqoKeepOpenOnCommit,sqoRefreshUsingSelect];
    AssertEquals('PacketRecords forced to -1',-1,Q.PacketRecords);
    Q.Open;
    AssertEquals('Got all records',20,Q.RecordCount);
    Q.SQLTransaction.Commit;
    AssertTrue('Still open after transaction',Q.Active);

    // Now check editing
    Q.Locate('id',20,[]);
    Q.Edit;
    Q.FieldByName('a').AsString:='abc';
    Q.Post;
    AssertTrue('Have updates pending',Q.UpdateStatus=usModified);
    Q.ApplyUpdates;
    AssertTrue('Have no more updates pending',Q.UpdateStatus=usUnmodified);
    Q.Close;
    Q.SQL.Text:='select * from FPDEV2 where (id=20) and (a=''abc'')';
    Q.Open;
    AssertTrue('Have modified data record in database', not (Q.EOF AND Q.BOF));
    end;
end;

procedure TTestTSQLQuery.TrySetPacketRecords;
begin
  FMyQ.PacketRecords:=10;
end;

procedure TTestTSQLQuery.TestKeepOpenOnCommitPacketRecords;
begin
  with SQLDBConnector do
    begin
    FMyQ := SQLDBConnector.Query;
    FMyQ.Options:=[sqoKeepOpenOnCommit];
    AssertException('Cannot set PacketRecords when sqoKeepOpenOnCommit is active',EDatabaseError,@TrySetPacketRecords);
    end;
end;

procedure TTestTSQLQuery.TrySetQueryOptions;
begin
  FMyQ.Options:=[sqoKeepOpenOnCommit];
end;

procedure TTestTSQLQuery.TestCheckSettingsOnlyWhenInactive;
begin
  // Check that we can only set QueryOptions when the query is inactive.
  with SQLDBConnector do
    begin
    ExecuteDirect('create table FPDEV2 (id integer not null, a varchar(10), constraint PK_FPDEV2 primary key(id))');
    Transaction.Commit;
    ExecuteDirect(Format('INSERT INTO FPDEV2 values (%d,''%.6d'')',[1,1]));
    Transaction.Commit;
    FMyQ := SQLDBConnector.Query;
    FMyQ.SQL.Text:='select * from FPDEV2';
    FMyQ := SQLDBConnector.Query;
    FMyQ.Open;
    AssertException('Cannot set Options when query is active',EDatabaseError,@TrySetQueryOptions);
    end;
end;

procedure TTestTSQLQuery.DoAfterPost(DataSet: TDataSet);
begin
  AssertTrue('Have modifications in after post',FMyq.UpdateStatus=usModified)
end;

procedure TTestTSQLQuery.TestAutoApplyUpdatesPost;
var Q: TSQLQuery;
    I: Integer;
begin
  // Test that if sqoAutoApplyUpdates is in QueryOptions, then POST automatically does an ApplyUpdates
  // Test also that POST afterpost event is backwards compatible.
  with SQLDBConnector do
    begin
    ExecuteDirect('create table FPDEV2 (id integer not null, a varchar(10), constraint PK_FPDEV2 primary key(id))');
    Transaction.COmmit;
    for I:=1 to 2 do
      ExecuteDirect(Format('INSERT INTO FPDEV2 values (%d,''%.6d'')',[i,i]));
    Transaction.COmmit;
    Q := SQLDBConnector.Query;
    FMyQ:=Q; // so th event handler can reach it.
    Q.SQL.Text:='select * from FPDEV2';
    Q.Options:=[sqoAutoApplyUpdates];
    // We must test that in AfterPost, the modification is still there, for backwards compatibilty
    Q.AfterPost:=@DoAfterPost;
    Q.Open;
    AssertEquals('Got all records',2,Q.RecordCount);
    // Now check editing
    Q.Locate('id',2,[]);
    Q.Edit;
    Q.FieldByName('a').AsString:='abc';
    Q.Post;
    AssertTrue('Have no more updates pending',Q.UpdateStatus=usUnmodified);
    Q.Close;
    Q.SQL.Text:='select * from FPDEV2 where (id=2) and (a=''abc'')';
    Q.Open;
    AssertTrue('Have modified data record in database',not (Q.EOF AND Q.BOF));
    end;

end;

procedure TTestTSQLQuery.TestAutoApplyUpdatesDelete;

var Q: TSQLQuery;
    I: Integer;
begin
  // Test that if sqoAutoApplyUpdates is in QueryOptions, then Delete automatically does an ApplyUpdates
  with SQLDBConnector do
    begin
    ExecuteDirect('create table FPDEV2 (id integer not null, a varchar(10), constraint PK_FPDEV2 primary key(id))');
    Transaction.COmmit;
    for I:=1 to 2 do
      ExecuteDirect(Format('INSERT INTO FPDEV2 values (%d,''%.6d'')',[i,i]));
    Transaction.COmmit;
    Q := SQLDBConnector.Query;
    FMyQ:=Q; // so th event handler can reach it.
    Q.SQL.Text:='select * from FPDEV2';
    Q.Options:=[sqoAutoApplyUpdates];
    // We must test that in AfterPost, the modification is still there, for backwards compatibilty
    Q.AfterPost:=@DoAfterPost;
    Q.Open;
    AssertEquals('Got all records',2,Q.RecordCount);
    // Now check editing
    Q.Locate('id',2,[]);
    Q.Delete;
    AssertTrue('Have no more updates pending',Q.UpdateStatus=usUnmodified);
    Q.Close;
    Q.SQL.Text:='select * from FPDEV2 where (id=2)';
    Q.Open;
    AssertTrue('Data record is deleted in database', (Q.EOF AND Q.BOF));
    end;
end;

procedure TTestTSQLQuery.DoApplyUpdates;

begin
  FMyQ.ApplyUpdates();
end;

procedure TTestTSQLQuery.DoCount(Sender: TSQLConnection; EventType: TDBEventType; const Msg: String);
begin
  If (EventType=detPrepare) then
    Inc(FPrepareCount);
end;

procedure TTestTSQLQuery.TestCheckRowsAffected;
var Q: TSQLQuery;
    I: Integer;
begin
  // Test that if sqoAutoApplyUpdates is in QueryOptions, then Delete automatically does an ApplyUpdates
  with SQLDBConnector do
    begin
    ExecuteDirect('create table FPDEV2 (id integer not null, a varchar(10), constraint PK_FPDEV2 primary key(id))');
    Transaction.COmmit;
    for I:=1 to 2 do
      ExecuteDirect(Format('INSERT INTO FPDEV2 values (%d,''%.6d'')',[i,i]));
    Transaction.COmmit;
    SQLDBConnector.Connection.Options:=[scoApplyUpdatesChecksRowsAffected];
    Q := SQLDBConnector.Query;
    Q.SQL.Text:='select * from FPDEV2';
    Q.DeleteSQL.Text:='delete from FPDEV2';
    Q.Open;
    AssertEquals('Got all records',2,Q.RecordCount);
    // Now check editing
    Q.Delete;
    FMyQ:=Q;
    AssertException('RowsAffected > 1 raises exception',EUpdateError,@DoApplyUpdates);
    end;
end;

procedure TTestTSQLQuery.TestAutoCommit;
var
  I : Integer;
begin
  with SQLDBConnector do
    begin
    ExecuteDirect('create table FPDEV2 (id integer not null, a varchar(10), constraint PK_FPDEV2 primary key(id))');
    if Transaction.Active then
      Transaction.Commit;

    Query.Options:=[sqoAutoCommit];
    for I:=1 to 2 do
      begin
      Query.SQL.Text:=Format('INSERT INTO FPDEV2 values (%d,''%.6d'');',[i,i]);
      Query.Prepare;
      Query.ExecSQL;
      // We do not commit anything explicitly.
      end;

    AssertFalse('Transaction is still active after expected auto commit', Transaction.Active);

    Connection.Close;
    Connection.Open;

    Query.SQL.Text:='SELECT COUNT(*) from FPDEV2';
    Query.Open;
    AssertEquals('Records haven''t been committed to database', 2, Query.Fields[0].AsInteger);
    end;
end;

procedure TTestTSQLQuery.TestGeneratedRefreshSQL;

var
  Q: TSQLQuery;

begin
  with SQLDBConnector do
    begin
    ExecuteDirect('create table FPDEV2 (id integer not null, a varchar(10) default ''abcde'', b varchar(5) default ''fgh'', constraint PK_FPDEV2 primary key(id))');
    if Transaction.Active then
      Transaction.Commit;
    end;
  Q:=SQLDBConnector.Query;
  Q.SQL.Text:='select * from FPDEV2';
  Q.InsertSQL.Text:='insert into FPDEV2 (id) values (:id)';
  Q.Options:=Q.Options+[sqoRefreshUsingSelect];
  Q.Open;
  With Q.FieldByName('id') do
    ProviderFlags:=ProviderFlags+[pfInKey];
  With Q.FieldByName('a') do
    ProviderFlags:=ProviderFlags+[pfRefreshOnInsert,pfRefreshOnUpdate];
  With Q.FieldByName('b') do
    ProviderFlags:=ProviderFlags+[pfRefreshOnInsert,pfRefreshOnUpdate];
  Q.Insert;
  Q.FieldByName('id').AsInteger:=1;
  Q.Post;
  AssertTrue('Field value has not been fetched after post',Q.FieldByName('a').IsNull);
  Q.ApplyUpdates(0);
  AssertEquals('Still on correct field',1,Q.FieldByName('id').AsInteger);
  AssertEquals('Field value has been fetched from the database ','abcde',Q.FieldByName('a').AsString);
  AssertEquals('Field value has been fetched from the database ','fgh',Q.FieldByName('b').AsString);
end;

procedure TTestTSQLQuery.TestGeneratedRefreshSQL1Field;
var
  Q: TSQLQuery;

begin
  with SQLDBConnector do
    begin
    ExecuteDirect('create table FPDEV2 (id integer not null, a varchar(10) default ''abcde'', b varchar(5) default ''fgh'', constraint PK_FPDEV2 primary key(id))');
    if Transaction.Active then
      Transaction.Commit;
    end;
  Q:=SQLDBConnector.Query;
  Q.SQL.Text:='select * from FPDEV2';
  Q.InsertSQL.Text:='insert into FPDEV2 (id) values (:id)';
  Q.Options:=Q.Options+[sqoRefreshUsingSelect];
  Q.Open;
  With Q.FieldByName('id') do
    ProviderFlags:=ProviderFlags+[pfInKey];
  With Q.FieldByName('a') do
    ProviderFlags:=ProviderFlags+[pfRefreshOnInsert,pfRefreshOnUpdate];
  Q.Insert;
  Q.FieldByName('id').AsInteger:=1;
  Q.Post;
  AssertTrue('Field value has not been fetched after post',Q.FieldByName('a').IsNull);
  Q.ApplyUpdates(0);
  AssertEquals('Still on correct field',1,Q.FieldByName('id').AsInteger);
  AssertEquals('Field value a has been fetched from the database ','abcde',Q.FieldByName('a').AsString);
  AssertEquals('Field value b has NOT been fetched from the database ','',Q.FieldByName('b').AsString);
end;

procedure TTestTSQLQuery.TestGeneratedRefreshSQLNoKey;
begin
  with SQLDBConnector do
    begin
    ExecuteDirect('create table FPDEV2 (id integer not null, a varchar(10) default ''abcde'', b varchar(5) default ''fgh'', constraint PK_FPDEV2 primary key(id))');
    if Transaction.Active then
      Transaction.Commit;
    end;
  FMyQ:=SQLDBConnector.Query;
  FMyQ.SQL.Text:='select * from FPDEV2';
  FMyQ.InsertSQL.Text:='insert into FPDEV2 (id) values (:id)';
  FMyQ.Options:=FMyQ.Options+[sqoRefreshUsingSelect];
  FMyQ.Open;
  With FMyQ.FieldByName('id') do
    ProviderFlags:=ProviderFlags-[pfInKey];
  With FMyQ.FieldByName('a') do
    ProviderFlags:=ProviderFlags+[pfRefreshOnInsert,pfRefreshOnUpdate];
  FMyQ.Insert;
  FMyQ.FieldByName('id').AsInteger:=1;
  FMyQ.Post;
  AssertException('Cannot refresh without primary key',EUpdateError,@DoApplyUpdates);
end;

procedure TTestTSQLQuery.TestRefreshSQL;
var
  Q: TSQLQuery;

begin
  with SQLDBConnector do
    begin
    ExecuteDirect('create table FPDEV2 (id integer not null primary key, a varchar(5) default ''abcde'', b integer default 1)');
    if Transaction.Active then
      Transaction.Commit;
    end;
  Q:=SQLDBConnector.Query;
  Q.SQL.Text:='select * from FPDEV2';
  Q.InsertSQL.Text:='insert into FPDEV2 (id) values (:id)';
  Q.RefreshSQL.Text:='SELECT a,b FROM FPDEV2 WHERE (id=:id)';
  Q.Open;
  Q.Insert;  // #1 record
  Q.FieldByName('id').AsInteger:=1;
  Q.Post;
  Q.Append;  // #2 record
  Q.FieldByName('id').AsInteger:=2;
  Q.Post;
  AssertTrue('Field value has not been fetched after Post', Q.FieldByName('a').IsNull);
  Q.ApplyUpdates(0);
  // #2 record:
  AssertEquals('Still on correct field', 2, Q.FieldByName('id').AsInteger);
  AssertEquals('Field value has been fetched from the database', 'abcde', Q.FieldByName('a').AsString);
  AssertEquals('Field value has been fetched from the database', 1, Q.FieldByName('b').AsInteger);
  Q.Prior;
  // #1 record:
  AssertEquals('Still on correct field', 1, Q.FieldByName('id').AsInteger);
  AssertEquals('Field value has been fetched from the database', 'abcde', Q.FieldByName('a').AsString);
  AssertEquals('Field value has been fetched from the database', 1, Q.FieldByName('b').AsInteger);
end;

procedure TTestTSQLQuery.TestRefreshSQLMultipleRecords;

begin
  with SQLDBConnector do
    begin
    ExecuteDirect('create table FPDEV2 (id integer not null, a varchar(10) default ''abcde'', b varchar(5) default ''fgh'', constraint PK_FPDEV2 primary key(id))');
    if Transaction.Active then
      Transaction.Commit;
    ExecuteDirect('insert into FPDEV2 (id) values (123)');
    if Transaction.Active then
      Transaction.Commit;
    end;
  FMyQ:=SQLDBConnector.Query;
  FMyQ.SQL.Text:='select * from FPDEV2';
  FMyQ.InsertSQL.Text:='insert into FPDEV2 (id) values (:id)';
  FMyQ.RefreshSQL.Text:='select * from FPDEV2';
  FMyQ.Open;
  With FMyQ.FieldByName('id') do
    ProviderFlags:=ProviderFlags+[pfInKey];
  With FMyQ.FieldByName('a') do
    ProviderFlags:=ProviderFlags+[pfRefreshOnInsert,pfRefreshOnUpdate];
  FMyQ.Insert;
  FMyQ.FieldByName('id').AsInteger:=1;
  FMyQ.Post;
  AssertException('Multiple records returned by RefreshSQL gives an error',EUpdateError,@DoApplyUpdates);
end;

procedure TTestTSQLQuery.TestRefreshSQLNoRecords;
begin
  with SQLDBConnector do
    begin
    ExecuteDirect('create table FPDEV2 (id integer not null, a varchar(10) default ''abcde'', b varchar(5) default ''fgh'', constraint PK_FPDEV2 primary key(id))');
    if Transaction.Active then
      Transaction.Commit;
    ExecuteDirect('insert into FPDEV2 (id) values (123)');
    if Transaction.Active then
      Transaction.Commit;
    end;
  FMyQ:=SQLDBConnector.Query;
  FMyQ.SQL.Text:='select * from FPDEV2';
  FMyQ.InsertSQL.Text:='insert into FPDEV2 (id) values (:id)';
  FMyQ.RefreshSQL.Text:='select * from FPDEV2 where 1=2';
  FMyQ.Open;
  With FMyQ.FieldByName('id') do
    ProviderFlags:=ProviderFlags+[pfInKey];
  With FMyQ.FieldByName('a') do
    ProviderFlags:=ProviderFlags+[pfRefreshOnInsert,pfRefreshOnUpdate];
  FMyQ.Insert;
  FMyQ.FieldByName('id').AsInteger:=1;
  FMyQ.Post;
  AssertException('No records returned by RefreshSQL gives an error',EUpdateError,@DoApplyUpdates);
end;

procedure TTestTSQLQuery.TestFetchAutoInc;
var datatype: string;
    id: largeint;
begin
  with SQLDBConnector do
    begin
    case SQLServerType of
      ssMySQL:
        datatype := 'integer auto_increment';
      ssMSSQL, ssSybase:
        datatype := 'integer identity';
      ssSQLite:
        datatype := 'integer';
      else
        Ignore(STestNotApplicable);
    end;
    ExecuteDirect('create table FPDEV2 (id '+datatype+' primary key, f varchar(5))');
    CommitDDL;
    end;

  with SQLDBConnector.Query do
    begin
    SQL.Text:='select * from FPDEV2';
    Open;
    Insert;
    FieldByName('f').AsString:='a';
    Post;  // #1 record
    Append;
    FieldByName('f').AsString:='b';
    Post;  // #2 record
    AssertTrue('ID field is not null after Post', FieldByName('id').IsNull);
    First; // #1 record
    ApplyUpdates(0);
    AssertTrue('ID field is still null after ApplyUpdates', Not FieldByName('id').IsNull);
    // Should be 1 after the table was created, but this is not guaranteed... So we just test positive values.
    id := FieldByName('id').AsLargeInt;
    AssertTrue('ID field has not positive value', id>0);
    Next;  // #2 record
    AssertTrue('Next ID value is not greater than previous', FieldByName('id').AsLargeInt>id);
    end;
end;

procedure TTestTSQLQuery.TestSequence;
var SequenceNames : TStringList;
begin
  case SQLServerType of
    ssFirebird:
      SQLDBConnector.ExecuteDirect('create sequence FPDEV_SEQ1');
    ssMSSQL, ssOracle, ssPostgreSQL:
      SQLDBConnector.ExecuteDirect('create sequence FPDEV_SEQ1 MINVALUE 1');
    else
      Ignore(STestNotApplicable);
  end;
  SQLDBConnector.ExecuteDirect('create table FPDEV2 (id integer)');
  SQLDBConnector.CommitDDL;

  with SQLDBConnector.Query do
    begin
    SQL.Text := 'select * from FPDEV2';
    Sequence.FieldName:='id';
    Sequence.SequenceName:='FPDEV_SEQ1';
    Open;
    // default is get next value on new record
    Append;
    AssertEquals(1, FieldByName('id').AsInteger);

    Sequence.ApplyEvent:=saeOnPost;
    Append;
    AssertTrue('Field ID must be null after Append', FieldByName('id').IsNull);
    Post;
    AssertEquals(2, FieldByName('id').AsInteger);
    end;

  // test GetSequenceNames
  SequenceNames := TStringList.Create;
  try
    SQLDBConnector.Connection.GetSequenceNames(SequenceNames);
    AssertTrue(SequenceNames.IndexOf('FPDEV_SEQ1') >= 0);
  finally
    SequenceNames.Free;
  end;

  SQLDBConnector.ExecuteDirect('drop sequence FPDEV_SEQ1');
  SQLDBConnector.CommitDDL;
end;

procedure TTestTSQLQuery.TestReturningInsert;

begin
  with SQLDBConnector do
    begin
    if not (sqSupportReturning in Connection.ConnOptions) then
      Ignore(STestNotApplicable);
    ExecuteDirect('create table FPDEV2 (id integer not null, a varchar(10) default ''abcde'', b varchar(5) default ''fgh'', constraint PK_FPDEV2 primary key(id))');
    if Transaction.Active then
      Transaction.Commit;
    ExecuteDirect('insert into FPDEV2 (id) values (123)');
    if Transaction.Active then
      Transaction.Commit;
    end;
  FMyQ:=SQLDBConnector.Query;
  FMyQ.SQL.Text:='select * from FPDEV2';
//  FMyQ.InsertSQL.Text:='insert into FPDEV2 (id) values (:id)';
  FMyQ.Open;
  With FMyQ.FieldByName('id') do
    ProviderFlags:=ProviderFlags+[pfInKey];
  With FMyQ.FieldByName('a') do
    ProviderFlags:=ProviderFlags+[pfRefreshOnInsert];
  With FMyQ.FieldByName('b') do
    ProviderFlags:=[];
  FMyQ.Insert;
  FMyQ.FieldByName('id').AsInteger:=1;
  FMyQ.Post;
  FMyQ.ApplyUpdates;
  AssertEquals('a updated','abcde',FMyQ.FieldByName('a').AsString);
  AssertEquals('b not updated','',FMyQ.FieldByName('b').AsString);
end;

procedure TTestTSQLQuery.TestReturningUpdate;

begin
  with SQLDBConnector do
    begin
    if not (sqSupportReturning in Connection.ConnOptions) then
      Ignore(STestNotApplicable);
    ExecuteDirect('create table FPDEV2 (id integer not null, a varchar(10) default ''abcde'', b varchar(5) default ''fgh'', constraint PK_FPDEV2 primary key(id))');
    CommitDDL;
    ExecuteDirect('insert into FPDEV2 (id) values (1)');
    ExecuteDirect('insert into FPDEV2 (id) values (2)');
    end;
  FMyQ:=SQLDBConnector.Query;
  FMyQ.SQL.Text:='select * from FPDEV2';
  FMyQ.Open;
  With FMyQ.FieldByName('id') do
    ProviderFlags:=ProviderFlags+[pfInKey];
  With FMyQ.FieldByName('b') do
    ProviderFlags:=[pfRefreshOnUpdate];  // Do not update, just fetch new value
  SQLDBConnector.ExecuteDirect('update FPDEV2 set b=''b1'' where id=1');
  SQLDBConnector.ExecuteDirect('update FPDEV2 set b=''b2'' where id=2');
  FMyQ.Edit;
  FMyQ.FieldByName('a').AsString:='a1';
  FMyQ.Post;  // #1 record
  FMyQ.Next;
  FMyQ.Edit;
  FMyQ.FieldByName('a').AsString:='a2';
  FMyQ.Post;  // #2 record
  FMyQ.ApplyUpdates;
  FMyQ.First;
  AssertEquals('#1.a updated', 'a1', FMyQ.FieldByName('a').AsString);
  AssertEquals('#1.b updated', 'b1', FMyQ.FieldByName('b').AsString);
  FMyQ.Next;
  AssertEquals('#2.a updated', 'a2', FMyQ.FieldByName('a').AsString);
  AssertEquals('#2.b updated', 'b2', FMyQ.FieldByName('b').AsString);
end;

procedure TTestTSQLQuery.TestMacros;
begin
  with SQLDBConnector do
    begin
    ExecuteDirect('create table FPDEV2 (id integer not null, constraint PK_FPDEV2 primary key(id))');
    CommitDDL;
    ExecuteDirect('insert into FPDEV2 (id) values (1)');
    ExecuteDirect('insert into FPDEV2 (id) values (2)');
    end;

  With SQLDBConnector.Query do
    begin
    SQL.Text:='Select ID from FPDEV2 '+
      '%WHERE_CL' +sLineBreak+
      '%ORDER_CL' +sLineBreak;
    MacroCheck:=true;
    MacroByName('WHERE_CL').AsString:='where 1=1';
    MacroByName('ORDER_CL').AsString:='order by 1';
    Open;
    AssertEquals('Correct SQL executed, macros substituted: ',1,Fields[0].AsInteger);
    Close;
    MacroByName('ORDER_CL').AsString := 'Order by 1 DESC';
    Open;
    AssertEquals('Correct SQL executed, macro value changed: ',2,Fields[0].AsInteger);
    end;
end;

procedure TTestTSQLQuery.CreateAndFillIDField;

Var
  I : Integer;

begin
  with SQLDBConnector do
    begin
    TryDropIfExist('FPDEV2');
    ExecuteDirect('create table FPDEV2 (id integer not null, constraint PK_FPDEV2 primary key(id))');
    CommitDDL;
    for I:=1 to 10 do
      ExecuteDirect('insert into FPDEV2 (id) values ('+IntToStr(I)+')');
    Connection.OnLog:=@DoCount;
    Connection.LogEvents:=[detPrepare];
    end;
end;

procedure TTestTSQLQuery.TestPrepareCount;

begin
  CreateAndFillIDField;
  try
    With SQLDBConnector.Query do
      begin
      UsePrimaryKeyAsKey:=False; // Disable server index defs etc
      SQL.Text:='Select ID from FPDEV2 where (ID>=:ID) order by ID';
      ParamByname('ID').AsInteger:=1;
      AssertFalse('Not Prepared',SQLDBConnector.Query.Prepared);
      Open;
      AssertEquals('Correct record count param 1',10,RecordCount);
      AssertEquals('Correct SQL executed, correct parameter: ',1,Fields[0].AsInteger);
      Close;
      AssertFalse('Still not prepared',SQLDBConnector.Query.Prepared);
      ParamByname('ID').AsInteger:=2;
      Open;
      AssertEquals('Correct record count param 2',9,RecordCount);
      AssertEquals('Correct SQL executed, macro value changed: ',2,Fields[0].AsInteger);
      Close;
      AssertFalse('Still not prepared',SQLDBConnector.Query.Prepared);
      end;
    AssertEquals('Prepare called only once ',2,FPrepareCount);
  finally
    SQLDBConnector.Connection.OnLog:=Nil;
  end;

end;

procedure TTestTSQLQuery.TestPrepareCount2;

begin
  CreateAndFillIDField;
  try
    With SQLDBConnector.Query do
      begin
      UsePrimaryKeyAsKey:=False; // Disable server index defs etc
      SQL.Text:='Select ID from FPDEV2 where (ID>=:ID) order by ID';
      ParamByname('ID').AsInteger:=1;
      Prepare;
      AssertTrue('Prepared',SQLDBConnector.Query.Prepared);
      Open;
      AssertEquals('Correct record count param 1',10,RecordCount);
      AssertEquals('Correct SQL executed, correct parameter: ',1,Fields[0].AsInteger);
      Close;
      AssertTrue('Still prepared',SQLDBConnector.Query.Prepared);
      ParamByname('ID').AsInteger:=2;
      Open;
      AssertEquals('Correct record count param 2',9,RecordCount);
      AssertEquals('Correct SQL executed, macro value changed: ',2,Fields[0].AsInteger);
      end;
    AssertEquals('Prepare called only once ',1,FPrepareCount);
  finally
    SQLDBConnector.Connection.OnLog:=Nil;
  end;
end;


{ TTestTSQLConnection }

procedure TTestTSQLConnection.TestImplicitTransaction;

Var
  T : TSQLTransaction;

begin
  T:=TSQLTransaction.Create(Nil);
  try
    T.Options:=[stoUseImplicit];
    T.DataBase:=SQLDBConnector.Connection;
  finally
    T.Free;
  end;
end;

procedure TTestTSQLConnection.TestImplicitTransaction2;

Var
  T : TSQLTransaction;

begin
  T:=TSQLTransaction.Create(Nil);
  try
    T.Options:=[stoUseImplicit];
    SQLDBConnector.Connection.Transaction:=T;
  finally
    T.Free;
  end;
end;

procedure TTestTSQLConnection.SetImplicit;

begin
  SQLDBConnector.Transaction.Options:=[stoUseImplicit];
end;

procedure TTestTSQLConnection.TestImplicitTransactionNotAssignable;

begin
  AssertException('Cannot set toUseImplicit option if database does not allow it',EDatabaseError,@SetImplicit);
  AssertException('Cannot assign database to transaction with toUseImplicit, if database does not allow it',EDatabaseError,@TestImplicitTransaction);
  AssertException('Cannot assign transaction with toUseImplicit to database, if database does not allow it',EDatabaseError,@TestImplicitTransaction2);
end;

procedure TTestTSQLConnection.TestImplicitTransactionOK;

var
  Q : TSQLQuery;
  T : TSQLTransaction;
  I : Integer;
begin
  with SQLDBConnector do
    begin
    ExecuteDirect('create table FPDEV2 (id integer not null, a varchar(10), constraint PK_FPDEV2 primary key(id))');
    if Transaction.Active then
      Transaction.Commit;
    end;
  SetImplicit;
  Q:=SQLDBConnector.Query;
  for I:=1 to 2 do
    begin
    Q.SQL.Text:=Format('INSERT INTO FPDEV2 values (%d,''%.6d'');',[i,i]);
    Q.Prepare;
    Q.ExecSQL;
    // We do not commit anything explicitly.
    end;
  Q:=Nil;
  T:=Nil;
  try
    T:=TSQLTransaction.Create(Nil);
    Q:=TSQLQuery.Create(Nil);
    Q.Transaction:=T;
    Q.Database:=SQLDBConnector.Connection;
    T.Database:=SQLDBConnector.Connection;
    Q.SQL.text:='SELECT COUNT(*) from FPDEV2';
    Q.Open;
    AssertEquals('Records have been committed to database',2,Q.Fields[0].AsInteger);
  finally
    Q.Free;
    T.Free;
  end;
end;

procedure TTestTSQLConnection.TestUseImplicitTransaction;
begin
  if (sqImplicitTransaction in SQLDBConnector.Connection.ConnOptions) then
    TestImplicitTransactionOK
  else
    TestImplicitTransactionNotAssignable;
end;

procedure TTestTSQLConnection.TryOpen;

begin
  SQLDBConnector.Query.Open;
end;

procedure TTestTSQLConnection.TestUseExplicitTransaction;
begin
  SQLDBConnector.Transaction.Active:=False;
  SQLDBConnector.Transaction.Options:=[stoExplicitStart];
  SQLDBConnector.Query.SQL.Text:='select * from FPDEV';
  AssertException('toExplicitStart raises exception on implicit start',EDatabaseError,@TryOpen)
end;

procedure TTestTSQLConnection.TestExplicitConnect;
begin
  SQLDBConnector.Transaction.Active:=False;
  SQLDBConnector.Connection.Options:=[scoExplicitConnect];
  SQLDBConnector.Connection.Connected:=False;
  SQLDBConnector.Query.SQL.Text:='select * from FPDEV';
  AssertException('toExplicitStart raises exception on implicit start',EDatabaseError,@TryOpen)
end;

procedure TTestTSQLConnection.TestGetStatementInfo;
var StmtInfo: TSQLStatementInfo;
begin
  // single table
  StmtInfo := SQLDBConnector.Connection.GetStatementInfo('SELECT * FROM tab1');
  AssertEquals('StatementType', ord(stSELECT), ord(StmtInfo.StatementType));
  AssertEquals('TableName', 'tab1', StmtInfo.TableName);
  AssertEquals('Updateable', True, StmtInfo.Updateable);
  StmtInfo := SQLDBConnector.Connection.GetStatementInfo('SELECT * FROM tab2 WHERE col1=1');
  AssertEquals('TableName', 'tab2', StmtInfo.TableName);
  AssertEquals('Updateable', True, StmtInfo.Updateable);
  // single table with schema
  StmtInfo := SQLDBConnector.Connection.GetStatementInfo('SELECT * FROM dbo.tab2 WHERE col1=1');
  AssertEquals('TableName', 'dbo.tab2', StmtInfo.TableName);
  AssertEquals('Updateable', True, StmtInfo.Updateable);
  // single table with quoted schema
  StmtInfo := SQLDBConnector.Connection.GetStatementInfo('SELECT * FROM "dbo".tab2 WHERE col1=1');
  AssertEquals('TableName', '"dbo".tab2', StmtInfo.TableName);
  AssertEquals('Updateable', True, StmtInfo.Updateable);
  StmtInfo := SQLDBConnector.Connection.GetStatementInfo('SELECT * FROM "dbo"."tab2" WHERE col1=1');
  AssertEquals('TableName', '"dbo"."tab2"', StmtInfo.TableName);
  AssertEquals('Updateable', True, StmtInfo.Updateable);
  // multiple tables
  StmtInfo := SQLDBConnector.Connection.GetStatementInfo('SELECT * FROM tab3,tab4 WHERE col1=1');
  AssertEquals('TableName', '', StmtInfo.TableName);
  AssertEquals('Updateable', False, StmtInfo.Updateable);
  // function
  StmtInfo := SQLDBConnector.Connection.GetStatementInfo('SELECT * FROM dbo.fn1(1)');
  AssertEquals('TableName', '', StmtInfo.TableName);
  AssertEquals('Updateable', False, StmtInfo.Updateable);
end;

procedure TTestTSQLConnection.TestGetNextValue;
begin
  if not (sqSequences in SQLDBConnector.Connection.ConnOptions) then
    Ignore('Connector '+SQLDBConnector.Connection.ClassName+' does not support sequences');
  if SQLServerType=ssSQLite then
    begin
    SQLDBConnector.TryDropIfExist('me');
    SQLDBConnector.ExecuteDirect('create table me (a integer primary key autoincrement,b int)');
    SQLDBConnector.ExecuteDirect('insert into me (b) values (1)');// Will create table sqlite_sequence if it didn't exist yet
    SQLDBConnector.ExecuteDirect('drop table me');
    end;
  SQLDBConnector.TryDropSequence('me');
  SQLDBConnector.TryCreateSequence('me');
  AssertTrue('Get value',SQLDBConnector.Connection.GetNextValue('me',1)>0);
end;


{ TTestTSQLScript }

procedure TTestTSQLScript.TestExecuteScript;
var Ascript : TSQLScript;
begin
  Ascript := TSQLScript.Create(nil);
  try
    with Ascript do
      begin
      DataBase := SQLDBConnector.Connection;
      Transaction := SQLDBConnector.Transaction;
      Script.Clear;
      Script.Append('create table FPDEV_A (id int);');
      Script.Append('create table FPDEV_B (id int);');
      ExecuteScript;
      // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
      SQLDBConnector.CommitDDL;
      end;
  finally
    AScript.Free;
    SQLDBConnector.ExecuteDirect('drop table FPDEV_A');
    SQLDBConnector.ExecuteDirect('drop table FPDEV_B');
    // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
    SQLDBConnector.CommitDDL;
  end;
end;

procedure TTestTSQLScript.TestScriptColon;
// Bug 25334: TSQLScript incorrectly treats : in scripts as sqldb query parameter markers
// Firebird-only test; can be extended for other dbs that use : in SQL
var
  Ascript : TSQLScript;
begin
  if not(SQLConnType in [interbase]) then Ignore(STestNotApplicable);
  Ascript := TSQLScript.Create(nil);
  try
    with Ascript do
      begin
      DataBase := SQLDBConnector.Connection;
      Transaction := SQLDBConnector.Transaction;
      Script.Clear;
      UseSetTerm := true;
      // Example procedure that selects table names
      Script.Append(
        'SET TERM ^ ; '+LineEnding+
        'CREATE PROCEDURE FPDEV_TESTCOLON '+LineEnding+
        'RETURNS (tblname VARCHAR(31)) '+LineEnding+
        'AS '+LineEnding+
        'begin '+LineEnding+
        '/*  Show tables. Note statement uses colon */ '+LineEnding+
        'FOR '+LineEnding+
        '  SELECT RDB$RELATION_NAME  '+LineEnding+
        '    FROM RDB$RELATIONS '+LineEnding+
        '    ORDER BY RDB$RELATION_NAME '+LineEnding+
        '    INTO :tblname '+LineEnding+
        'DO  '+LineEnding+
        '  SUSPEND; '+LineEnding+
        'end^ '+LineEnding+
        'SET TERM ; ^'
        );
      ExecuteScript;
      // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
      SQLDBConnector.CommitDDL;
      end;
  finally
    AScript.Free;
    SQLDBConnector.ExecuteDirect('DROP PROCEDURE FPDEV_TESTCOLON');
    // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
    SQLDBConnector.CommitDDL;
  end;
end;

procedure TTestTSQLScript.TestUseCommit;
// E.g. Firebird needs explicit COMMIT sometimes, e.g. if mixing DDL and DML
// statements in a script.
// Probably same as bug 17829 Error executing SQL script
const
  TestValue='Some text';
var
  Ascript : TSQLScript;
  CheckQuery : TSQLQuery;
begin
  Ascript := TSQLScript.Create(nil);
  try
    with Ascript do
      begin
      DataBase := SQLDBConnector.Connection;
      Transaction := SQLDBConnector.Transaction;
      Script.Clear;
      UseCommit:=true;
      // Example procedure that selects table names
      Script.Append('CREATE TABLE fpdev_scriptusecommit (logmessage VARCHAR(255));');
      Script.Append('COMMIT;'); //needed for table to show up
      Script.Append('INSERT INTO fpdev_scriptusecommit (logmessage) VALUES('''+TestValue+''');');
      Script.Append('COMMIT;');
      ExecuteScript;
      // This line should not run, as the commit above should have taken care of it:
      //SQLDBConnector.CommitDDL;
      // Test whether second line of script executed, just to be sure
      CheckQuery:=SQLDBConnector.Query;
      CheckQuery.SQL.Text:='SELECT logmessage FROM fpdev_scriptusecommit ';
      CheckQuery.Open;
      CheckEquals(TestValue, CheckQuery.Fields[0].AsString, 'Insert script line should have inserted '+TestValue);
      CheckQuery.Close;
      end;
  finally
    AScript.Free;
    SQLDBConnector.ExecuteDirect('DROP TABLE fpdev_scriptusecommit');
    SQLDBConnector.Transaction.Commit;
  end;
end;

{ TSQLDBTestCase }

function TSQLDBTestCase.GetSQLDBConnector: TSQLDBConnector;
begin
  Result := DBConnector as TSQLDBConnector;
end;

procedure TSQLDBTestCase.SetUp;
begin
  inherited SetUp;
  InitialiseDBConnector;
  DBConnector.StartTest(TestName);
end;

procedure TSQLDBTestCase.TearDown;
begin
  DBConnector.StopTest(TestName);
  if assigned(DBConnector) then
    with SQLDBConnector do
      if Assigned(Transaction) and Transaction.Active and not (stoUseImplicit in Transaction.Options) then
        Transaction.Rollback;
  FreeDBConnector;
  inherited TearDown;
end;


initialization
  if uppercase(dbconnectorname)='SQL' then
  begin
    RegisterTest(TTestTSQLQuery);
    RegisterTest(TTestTSQLConnection);
    RegisterTest(TTestTSQLScript);
  end;
end.
