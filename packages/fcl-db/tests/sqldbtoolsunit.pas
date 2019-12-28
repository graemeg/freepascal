unit SQLDBToolsUnit;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, toolsunit
  ,db, sqldb
  ,mysql40conn, mysql41conn, mysql50conn, mysql51conn, mysql55conn, mysql56conn, mysql57conn
  ,ibconnection
  ,pqconnection
  ,odbcconn
  {$IFNDEF WIN64}
  {See packages\fcl-db\fpmake.pp: Oracle connector not built yet on Win64}
  ,oracleconnection
  {$ENDIF WIN64}
  ,sqlite3conn
  ,mssqlconn
  ;

type
  TSQLConnType = (mysql40,mysql41,mysql50,mysql51,mysql55,mysql56,mysql57,postgresql,interbase,odbc,oracle,sqlite3,mssql,sybase);
  TSQLServerType = (ssFirebird, ssInterbase, ssMSSQL, ssMySQL, ssOracle, ssPostgreSQL, ssSQLite, ssSybase, ssUnknown);

const
  MySQLConnTypes = [mysql40,mysql41,mysql50,mysql51,mysql55,mysql56,mysql57];
  SQLConnTypesNames : Array [TSQLConnType] of String[19] =
        ('MYSQL40','MYSQL41','MYSQL50','MYSQL51','MYSQL55','MYSQL56','MYSQL57','POSTGRESQL','INTERBASE','ODBC','ORACLE','SQLITE3','MSSQL','SYBASE');
             
  STestNotApplicable = 'This test does not apply to this sqldb connection type';


type
{ TSQLDBConnector }
  TSQLDBConnector = class(TDBConnector)
  private
    FConnection    : TSQLConnection;
    FTransaction   : TSQLTransaction;
    FQuery         : TSQLQuery;
    FUniDirectional: boolean;
    procedure CreateFConnection;
    Function CreateQuery : TSQLQuery;
  protected
    procedure SetTestUniDirectional(const AValue: boolean); override;
    function GetTestUniDirectional: boolean; override;
    procedure CreateNDatasets; override;
    procedure CreateFieldDataset; override;
    // If logging is enabled, this procedure will receive the event
    // from the SQLDB logging system
    // For custom logging call with sender nil and eventtype detCustom
    procedure DoLogEvent(Sender: TSQLConnection; EventType: TDBEventType; Const Msg : String);
    procedure DropNDatasets; override;
    procedure DropFieldDataset; override;
    Function InternalGetNDataset(n : integer) : TDataset; override;
    Function InternalGetFieldDataset : TDataSet; override;
  public
    procedure TryDropIfExist(ATableName : String);
    procedure TryCreateSequence(ASequenceName : String);
    procedure TryDropSequence(ASequenceName: String);
    destructor Destroy; override;
    constructor Create; override;
    procedure ExecuteDirect(const SQL: string);
    // Issue a commit(retaining) for databases that need it (e.g. in DDL)
    procedure CommitDDL;
    Procedure FreeTransaction;
    property Connection : TSQLConnection read FConnection;
    property Transaction : TSQLTransaction read FTransaction;
    property Query : TSQLQuery read FQuery;
  end;

var SQLConnType : TSQLConnType;
    SQLServerType : TSQLServerType;
    FieldtypeDefinitions : Array [TFieldType] of String[20];

function IdentifierCase(const s: string): string;

implementation

uses StrUtils;

type
  TSQLServerTypesMapItem = record
    s: string;
    t: TSQLServerType;
  end;

const
  FieldtypeDefinitionsConst : Array [TFieldType] of String[20] =
    (
      {ftUnknown} '',
      {ftString} 'VARCHAR(10)',
      {ftSmallint} 'SMALLINT',
      {ftInteger} 'INTEGER',
      {ftWord} '',
      {ftBoolean} 'BOOLEAN',
      {ftFloat} 'DOUBLE PRECISION',
      {ftCurrency} '',
      {ftBCD} 'DECIMAL(18,4)',
      {ftDate} 'DATE',
      {ftTime} 'TIME',
      {ftDateTime} 'TIMESTAMP',
      {ftBytes} '',
      {ftVarBytes} '',
      {ftAutoInc} '',
      {ftBlob} 'BLOB',
      {ftMemo} 'BLOB',
      {ftGraphic} 'BLOB',
      {ftFmtMemo} '',
      {ftParadoxOle} '',
      {ftDBaseOle} '',
      {ftTypedBinary} '',
      {ftCursor} '',
      {ftFixedChar} 'CHAR(10)',
      {ftWideString} '',
      {ftLargeint} 'BIGINT',
      {ftADT} '',
      {ftArray} '',
      {ftReference} '',
      {ftDataSet} '',
      {ftOraBlob} '',
      {ftOraClob} '',
      {ftVariant} '',
      {ftInterface} '',
      {ftIDispatch} '',
      {ftGuid} '',
      {ftTimeStamp} 'TIMESTAMP',
      {ftFMTBcd} 'NUMERIC(18,6)',
      {ftFixedWideChar} '',
      {ftWideMemo} ''
    );

  // names as returned by ODBC SQLGetInfo(..., SQL_DBMS_NAME, ...) and GetConnectionInfo(citServerType)
  SQLServerTypesMap : array [0..7] of TSQLServerTypesMapItem = (
    (s: 'Firebird'; t: ssFirebird),
    (s: 'Interbase'; t: ssInterbase),
    (s: 'Microsoft SQL Server'; t: ssMSSQL),
    (s: 'MySQL'; t: ssMySQL),
    (s: 'Oracle'; t: ssOracle),
    (s: 'PostgreSQL'; t: ssPostgreSQL),
    (s: 'SQLite3'; t: ssSQLite),
    (s: 'ASE'; t: ssSybase)
  );

  // fall back mapping (e.g. in case GetConnectionInfo(citServerType) is not implemented)
  SQLConnTypeToServerTypeMap : array[TSQLConnType] of TSQLServerType =
    (ssMySQL,ssMySQL,ssMySQL,ssMySQL,ssMySQL,ssMySQL,ssMySQL,ssPostgreSQL,ssFirebird,ssUnknown,ssOracle,ssSQLite,ssMSSQL,ssSybase);


function IdentifierCase(const s: string): string;
begin
  // format unquoted identifier name as required by SQL servers
  case SQLServerType of
    ssPostgreSQL: Result := LowerCase(s); // PostgreSQL stores unquoted identifiers in lowercase (incompatible with the SQL standard)
    ssInterbase,
    ssFirebird  : Result := UpperCase(s); // Dialect 1 requires uppercase; dialect 3 is case agnostic
    else
                  Result := s; // mixed case
  end;
end;

{ TSQLDBConnector }

procedure TSQLDBConnector.CreateFConnection;
var t : TSQLConnType;
    i : integer;
    s : string;
begin
  for t := low(SQLConnTypesNames) to high(SQLConnTypesNames) do
    if UpperCase(dbconnectorparams) = SQLConnTypesNames[t] then SQLConnType := t;

  case SQLConnType of
    MYSQL40:    Fconnection := TMySQL40Connection.Create(nil);
    MYSQL41:    Fconnection := TMySQL41Connection.Create(nil);
    MYSQL50:    Fconnection := TMySQL50Connection.Create(nil);
    MYSQL51:    Fconnection := TMySQL51Connection.Create(nil);
    MYSQL55:    Fconnection := TMySQL55Connection.Create(nil);
    MYSQL56:    Fconnection := TMySQL56Connection.Create(nil);
    MYSQL57:    Fconnection := TMySQL57Connection.Create(nil);
    SQLITE3:    Fconnection := TSQLite3Connection.Create(nil);
    POSTGRESQL: Fconnection := TPQConnection.Create(nil);
    INTERBASE : Fconnection := TIBConnection.Create(nil);
    ODBC:       Fconnection := TODBCConnection.Create(nil);
  {$IFNDEF Win64}
    ORACLE:     Fconnection := TOracleConnection.Create(nil);
  {$ENDIF Win64}
    MSSQL:      Fconnection := TMSSQLConnection.Create(nil);
    SYBASE:     Fconnection := TSybaseConnection.Create(nil);
    else        writeln('Invalid database type, check if a valid database type for your achitecture was provided in the file ''database.ini''');
  end;

  FTransaction := TSQLTransaction.Create(nil);

  with Fconnection do
  begin
    Transaction := FTransaction;
    DatabaseName := dbname;
    UserName := dbuser;
    Password := dbpassword;
    HostName := dbhostname;
    CharSet := dbcharset;
    if dblogfilename<>'' then
    begin
      LogEvents:=[detCustom,detCommit,detExecute,detRollBack];
      OnLog:=@DoLogEvent;
    end;

    if (dbhostname='') and (SQLConnType=interbase) then
    begin
      // Firebird embedded: create database file if it doesn't yet exist
      // Note: pagesize parameter has influence on behavior. We're using
      // Firebird default here.
      if not(fileexists(dbname)) then
        CreateDB; //Create testdb
    end;

    if length(dbQuoteChars)>1 then
      FieldNameQuoteChars:=dbQuoteChars;

    Open;
  end;

  // determine remote SQL Server to which we are connected
  s := Fconnection.GetConnectionInfo(citServerType);
  if s = '' then
    SQLServerType := SQLConnTypeToServerTypeMap[SQLConnType] // if citServerType isn't implemented
  else
    for i := low(SQLServerTypesMap) to high(SQLServerTypesMap) do
      if SQLServerTypesMap[i].s = s then
        SQLServerType := SQLServerTypesMap[i].t;

  FieldtypeDefinitions := FieldtypeDefinitionsConst;

  // Server-specific initialization
  case SQLServerType of
    ssFirebird:
      begin
      // Firebird < 3.0 has no support for Boolean data type:
      FieldtypeDefinitions[ftBoolean] := '';
      FieldtypeDefinitions[ftMemo]    := 'BLOB SUB_TYPE TEXT';
      end;
    ssInterbase:
      begin
      FieldtypeDefinitions[ftMemo]     := 'BLOB SUB_TYPE TEXT';
      FieldtypeDefinitions[ftLargeInt] := 'NUMERIC(18,0)';
      end;
    ssMSSQL, ssSybase:
      // todo: Sybase: copied over MSSQL; verify correctness
      // note: test database should have case-insensitive collation
      begin
      FieldtypeDefinitions[ftBoolean] := 'BIT';
      FieldtypeDefinitions[ftFloat]   := 'FLOAT';
      FieldtypeDefinitions[ftCurrency]:= 'MONEY';
      FieldtypeDefinitions[ftDateTime]:= 'DATETIME';
      FieldtypeDefinitions[ftBytes]   := 'BINARY(5)';
      FieldtypeDefinitions[ftVarBytes]:= 'VARBINARY(10)';
      FieldtypeDefinitions[ftBlob]    := 'IMAGE';
      FieldtypeDefinitions[ftMemo]    := 'TEXT';
      FieldtypeDefinitions[ftGraphic] := '';
      FieldtypeDefinitions[ftWideString] := 'NVARCHAR(10)';
      FieldtypeDefinitions[ftFixedWideChar] := 'NCHAR(10)';
      //FieldtypeDefinitions[ftWideMemo] := 'NTEXT'; // Sybase has UNITEXT?

      // Proper blob support:
      FConnection.ExecuteDirect('SET TEXTSIZE 2147483647');
      if SQLServerType=ssMSSQL then
      begin
        // When running CREATE TABLE statements, allow NULLs by default - without
        // having to specify NULL all the time:
        // http://msdn.microsoft.com/en-us/library/ms174979.aspx
        //
        // Padding character fields is expected by ANSI and sqldb, as well as
        // recommended by Microsoft:
        // http://msdn.microsoft.com/en-us/library/ms187403.aspx
        FConnection.ExecuteDirect('SET ANSI_NULL_DFLT_ON ON; SET ANSI_PADDING ON; SET ANSI_WARNINGS OFF');
      end;
      if SQLServerType=ssSybase then
      begin
        // Evaluate NULL expressions according to ANSI SQL:
        // http://infocenter.sybase.com/archive/index.jsp?topic=/com.sybase.help.ase_15.0.commands/html/commands/commands85.htm
        FConnection.ExecuteDirect('SET ANSINULL ON');

        { Tests require these database options set
        1) with ddl in tran; e.g.
        use master
        go
        sp_dboption pubs3, 'ddl in tran', true
        go
        Avoid errors like
        The 'CREATE TABLE' command is not allowed within a multi-statement transaction in the 'test' database.
        2) allow nulls by default, e.g.
        use master
        go
        sp_dboption pubs3, 'allow nulls by default', true
        go
        }
      end;
      FTransaction.Commit;
      end;
    ssMySQL:   
      begin
      FieldtypeDefinitions[ftWord] := 'SMALLINT UNSIGNED';
      // MySQL recognizes BOOLEAN, but as synonym for TINYINT, not true sql boolean datatype
      FieldtypeDefinitions[ftBoolean]  := '';
      // Use 'DATETIME' for datetime fields instead of timestamp, because
      // mysql's timestamps are only valid in the range 1970-2038.
      // Downside is that fields defined as 'TIMESTAMP' aren't tested
      FieldtypeDefinitions[ftDateTime] := 'DATETIME';
      FieldtypeDefinitions[ftBytes]    := 'BINARY(5)';
      FieldtypeDefinitions[ftVarBytes] := 'VARBINARY(10)';
      FieldtypeDefinitions[ftMemo]     := 'TEXT';
      // Add into my.ini: sql-mode="...,PAD_CHAR_TO_FULL_LENGTH,ANSI_QUOTES" or set it explicitly by:
      // PAD_CHAR_TO_FULL_LENGTH to avoid trimming trailing spaces contrary to SQL standard (MySQL 5.1.20+)
      FConnection.ExecuteDirect('SET SESSION sql_mode=''STRICT_ALL_TABLES,PAD_CHAR_TO_FULL_LENGTH,ANSI_QUOTES''');
      FTransaction.Commit;
      end;
    ssOracle:
      begin
      FieldtypeDefinitions[ftBoolean]  := '';
      // At least Oracle 10, 11 do not support a BIGINT field:
      FieldtypeDefinitions[ftLargeInt] := 'NUMBER(19,0)';
      FieldtypeDefinitions[ftTime]     := 'TIMESTAMP';
      FieldtypeDefinitions[ftMemo]     := 'CLOB';
      FieldtypeDefinitions[ftWideString] := 'NVARCHAR2(10)';
      FieldtypeDefinitions[ftFixedWideChar] := 'NCHAR(10)';
      FieldtypeDefinitions[ftWideMemo] := 'NCLOB';
      end;
    ssPostgreSQL:
      begin
      FieldtypeDefinitions[ftCurrency] := 'MONEY'; // ODBC?!
      FieldtypeDefinitions[ftBlob]     := 'BYTEA';
      FieldtypeDefinitions[ftMemo]     := 'TEXT';
      FieldtypeDefinitions[ftGraphic]  := '';
      FieldtypeDefinitions[ftGuid]     := 'UUID';
      end;
    ssSQLite:
      begin
      // SQLite stores all values with decimal point as 8 byte (double) IEEE floating point numbers
      // (it causes that some tests (for BCD, FmtBCD fields) fails for exact numeric values, which can't be lossless expressed as 8 byte floating point values)
      FieldtypeDefinitions[ftWord] := 'WORD';
      FieldtypeDefinitions[ftCurrency] := 'CURRENCY';
      FieldtypeDefinitions[ftBytes] := 'BINARY(5)';
      FieldtypeDefinitions[ftVarBytes] := 'VARBINARY(10)';
      FieldtypeDefinitions[ftMemo] := 'CLOB'; //or TEXT SQLite supports both, but CLOB is sql standard (TEXT not)
      FieldtypeDefinitions[ftWideString] := 'NVARCHAR(10)';
      FieldtypeDefinitions[ftFixedWideChar] := 'NCHAR(10)';
      FieldtypeDefinitions[ftWideMemo] := 'NCLOB';
      end;
  end;

  if SQLConnType in [mysql40,mysql41] then
    begin
    // Mysql versions prior to 5.0.3 removes the trailing spaces on varchar
    // fields on insertion. So to test properly, we have to do the same
    for i := 0 to testValuesCount-1 do
      testStringValues[i] := TrimRight(testStringValues[i]);
    end;

  if SQLServerType in [ssMSSQL, ssSQLite, ssSybase] then
    // Some DB's do not support sql compliant boolean data type.
    for i := 0 to testValuesCount-1 do
      testValues[ftBoolean, i] := BoolToStr(testBooleanValues[i], '1', '0');

  if SQLServerType in [ssMySQL] then
    begin
    // Some DB's do not support milliseconds in datetime and time fields.
    for i := 0 to testValuesCount-1 do
      begin
      testTimeValues[i] := copy(testTimeValues[i],1,8)+'.000';
      testValues[ftTime,i] := copy(testTimeValues[i],1,8)+'.000';
      if length(testValues[ftDateTime,i]) > 19 then
        testValues[ftDateTime,i] := copy(testValues[ftDateTime,i],1,19)+'.000';
      end;
    end;

  if SQLServerType in [ssFirebird, ssInterbase, ssMSSQL, ssOracle, ssPostgreSQL, ssSybase] then
    begin
    // Some db's do not support times > 24:00:00
    testTimeValues[3]:='13:25:15.000';
    testValues[ftTime,3]:='13:25:15.000';
    if SQLServerType in [ssFirebird, ssInterbase, ssMSSQL, ssOracle] then
      begin
      // Firebird, Oracle, MS SQL Server do not support time = 24:00:00
      // MS SQL Server "datetime" supports only time up to 23:59:59.997
      testTimeValues[2]:='23:59:59.997';
      testValues[ftTime,2]:='23:59:59.997';
      end;
    end;

  if SQLServerType in [ssMSSQL, ssSybase] then
    // Some DB's do not support datetime values before 1753-01-01
    for i := 18 to testValuesCount-1 do
      testValues[ftDateTime,i] := testValues[ftDateTime,0];

  // DecimalSeparator must correspond to monetary locale (lc_monetary) set on PostgreSQL server
  // Here we assume, that locale on client side is same as locale on server
  if SQLServerType in [ssPostgreSQL] then
    for i := 0 to testValuesCount-1 do
      testValues[ftCurrency,i] := QuotedStr(CurrToStr(testCurrencyValues[i]));

  // SQLite does not support fixed length CHAR datatype
  if SQLServerType in [ssSQLite] then
    for i := 0 to testValuesCount-1 do
      testValues[ftFixedChar,i] := PadRight(testValues[ftFixedChar,i], 10);
end;

Function TSQLDBConnector.CreateQuery: TSQLQuery;

begin
  Result := TSQLQuery.create(nil);
  with Result do
    begin
    database := Fconnection;
    transaction := Ftransaction;
    PacketRecords := -1;  // To avoid: "Connection is busy with results for another hstmt" (ODBC,MSSQL)
    end;
end;



procedure TSQLDBConnector.SetTestUniDirectional(const AValue: boolean);
begin
  FUniDirectional:=avalue;
  FQuery.UniDirectional:=AValue;
end;

function TSQLDBConnector.GetTestUniDirectional: boolean;
begin
  result := FUniDirectional;
end;

procedure TSQLDBConnector.CreateNDatasets;
var CountID : Integer;
begin
  try
    Ftransaction.StartTransaction;
    TryDropIfExist('FPDEV');
    Fconnection.ExecuteDirect('create table FPDEV (' +
                              '  ID INT NOT NULL,  ' +
                              '  NAME VARCHAR(50), ' +
                              '  PRIMARY KEY (ID)  ' +
                              ')');

    FTransaction.CommitRetaining;

    for countID := 1 to MaxDataSet do
      Fconnection.ExecuteDirect('insert into FPDEV (ID,NAME) ' +
                                'values ('+inttostr(countID)+',''TestName'+inttostr(countID)+''')');

    Ftransaction.Commit;
  except
    on E: Exception do begin
      if dblogfilename<>'' then
        DoLogEvent(nil,detCustom,'Exception running CreateNDatasets: '+E.Message);
      if Ftransaction.Active then
        Ftransaction.Rollback
    end;
  end;
end;

procedure TSQLDBConnector.CreateFieldDataset;
var
  CountID : Integer;
  FType   : TFieldType;
  Sql,sql1: String;

function String2Hex(Source: string): string;
// Converts ASCII codes into hex
var
  i: integer;
begin
  result := '';
  for i := 1 to length(Source) do
    result := result + inttohex(ord(Source[i]),2);
end;

begin
  try
    Ftransaction.StartTransaction;
    TryDropIfExist('FPDEV_FIELD');

    Sql := 'create table FPDEV_FIELD (ID INT NOT NULL,';
    for FType := low(TFieldType)to high(TFieldType) do
      if FieldtypeDefinitions[FType]<>'' then
        sql := sql + 'F' + Fieldtypenames[FType] + ' ' +
          FieldtypeDefinitions[FType] + ',';
    Sql := Sql + 'PRIMARY KEY (ID))';

    FConnection.ExecuteDirect(Sql);

    FTransaction.CommitRetaining;

    for countID := 0 to testValuesCount-1 do
      begin
      Sql :=  'insert into FPDEV_FIELD (ID';
      Sql1 := 'values ('+IntToStr(countID);
      for FType := low(TFieldType)to high(TFieldType) do
        if FieldtypeDefinitions[FType]<>'' then
          begin
          sql := sql + ',F' + Fieldtypenames[FType];
          if testValues[FType,CountID] <> '' then
            if FType in [ftBoolean, ftCurrency] then
               sql1 := sql1 + ',' + testValues[FType,CountID]
            else if (FType in [ftBlob, ftBytes, ftGraphic, ftVarBytes]) and
                    (SQLServerType = ssOracle) then
               // Oracle does not accept string literals in blob insert statements
               // convert 'DEADBEEF' hex literal to binary:
               sql1 := sql1 + ', HEXTORAW(' + QuotedStr(String2Hex(testValues[FType,CountID])) + ') '
            else if (FType = ftDate) and
                    (SQLServerType = ssOracle) then
               // Oracle requires date conversion; otherwise
               // ORA-01861: literal does not match format string
               // ANSI/ISO date literal:
               sql1 := sql1 + ', DATE ' + QuotedStr(testValues[FType,CountID])
            else if (FType = ftDateTime) and
                    (SQLServerType = ssOracle) then begin
               // similar to ftDate handling
               // Could be a real date+time or only date. Does not consider only time.
               if pos(' ',testValues[FType,CountID])>0 then
                  sql1 := sql1 + ', TIMESTAMP ' + QuotedStr(testValues[FType,CountID])
               else
                  sql1 := sql1 + ', DATE ' + QuotedStr(testValues[FType,CountID]);
            end
            else if (FType = ftTime) and
                    (SQLServerType = ssOracle) then
               // similar to ftDate handling
               // More or less arbitrary default time; there is no time-only data type in Oracle.
               sql1 := sql1 + ', TIMESTAMP ' + QuotedStr('0001-01-01 '+testValues[FType,CountID])
            else
               sql1 := sql1 + ',' + QuotedStr(testValues[FType,CountID])
          else
            sql1 := sql1 + ',NULL';
          end;
      Sql := sql + ')';
      Sql1 := sql1+ ')';

      Fconnection.ExecuteDirect(sql + ' ' + sql1);
      end;

    Ftransaction.Commit;
  except
    on E: Exception do begin
      if dblogfilename<>'' then
        DoLogEvent(nil,detCustom,'Exception running CreateFieldDataset: '+E.Message);
      if Ftransaction.Active then Ftransaction.Rollback;
    end;
  end;
end;

procedure TSQLDBConnector.DoLogEvent(Sender: TSQLConnection;
  EventType: TDBEventType; Const Msg: String);
var
  Category: string;
begin
  case EventType of
    detCustom:   Category:='Custom';
    detPrepare:  Category:='Prepare';
    detExecute:  Category:='Execute';
    detFetch:    Category:='Fetch';
    detCommit:   Category:='Commit';
    detRollBack: Category:='Rollback';
    else Category:='Unknown event. Please fix program code.';
  end;
  LogMessage(Category,Msg);
end;

procedure TSQLDBConnector.DropNDatasets;
begin
  if assigned(FTransaction) then
    begin
    try
      if Ftransaction.Active then Ftransaction.Rollback;
      Ftransaction.StartTransaction;
      Fconnection.ExecuteDirect('DROP TABLE FPDEV');
      Ftransaction.Commit;
    Except
      on E: Exception do begin
        if dblogfilename<>'' then
          DoLogEvent(nil,detCustom,'Exception running DropNDatasets: '+E.Message);
        if Ftransaction.Active then Ftransaction.Rollback
      end;
    end;
    end;
end;

procedure TSQLDBConnector.DropFieldDataset;
begin
  if assigned(FTransaction) then
    begin
    try
      if Ftransaction.Active then Ftransaction.Rollback;
      Ftransaction.StartTransaction;
      Fconnection.ExecuteDirect('DROP TABLE FPDEV_FIELD');
      Ftransaction.Commit;
    Except
      on E: Exception do begin
        if dblogfilename<>'' then
          DoLogEvent(nil,detCustom,'Exception running DropFieldDataset: '+E.Message);
        if Ftransaction.Active then Ftransaction.Rollback
      end;
    end;
    end;
end;

Function TSQLDBConnector.InternalGetNDataset(n: integer): TDataset;
begin
  Result := CreateQuery;
  with (Result as TSQLQuery) do
    begin
    sql.clear;
    sql.add('SELECT * FROM FPDEV WHERE ID < '+inttostr(n+1)+' ORDER BY ID');
    UniDirectional:=TestUniDirectional;
    end;
end;

Function TSQLDBConnector.InternalGetFieldDataset: TDataSet;
begin
  Result := CreateQuery;
  with (Result as TSQLQuery) do
    begin
    sql.clear;
    sql.add('SELECT * FROM FPDEV_FIELD');
    UniDirectional:=TestUniDirectional;
    end;
end;

procedure TSQLDBConnector.TryDropIfExist(ATableName: String);
begin
  // This makes life so much easier, since it avoids the exception if the table already
  // exists. And while this exception is in a try..except statement, the debugger
  // always shows the exception, which is pretty annoying.
  try
    case SQLServerType of
      ssFirebird:
        begin
        // This only works with Firebird 2+
        FConnection.ExecuteDirect('execute block as begin if (exists (select 1 from rdb$relations where upper(rdb$relation_name)=''' + UpperCase(ATableName) + ''')) '+
          'then execute statement ''drop table ' + ATableName + ';'';end');
        FTransaction.CommitRetaining;
        end;
      ssMSSQL:
        begin
        // Checking is needed here to avoid getting "auto rollback" of a subsequent CREATE TABLE statement
        // which leads to the rollback not referring to the right transaction=>SQL error
        // Use SQL92 ISO standard INFORMATION_SCHEMA:
        FConnection.ExecuteDirect(
          'if exists (select * from INFORMATION_SCHEMA.TABLES where TABLE_TYPE=''BASE TABLE'' AND TABLE_NAME=''' + ATableName + ''')'+
          ' drop table ' + ATableName );
        end;
      ssMySQL:
        begin
        FConnection.ExecuteDirect('drop table if exists ' + ATableName);
        end;
      ssPostgreSQL,
      ssSQLite:
        begin
        FConnection.ExecuteDirect('drop table if exists ' + ATableName);
        FTransaction.CommitRetaining;
        end;
      ssOracle:
        begin
          FConnection.ExecuteDirect(
          'declare ' +
          '   c int; ' +
          'begin ' +
          '   select count(*) into c from all_tables where table_name = upper(''' + ATableName + '''); ' +
          '   if c = 1 then ' +
          '      execute immediate ''drop table ' + ATableName + '''; ' +
          '   end if; ' +
          'end; ');
        end;
      ssSybase:
        begin
        // Checking is needed here to avoid getting "auto rollback" of a subsequent CREATE TABLE statement
        // which leads to the rollback not referring to the right transaction=>SQL error
        // Can't use SQL standard information_schema; instead query sysobjects for User tables
        FConnection.ExecuteDirect(
          'if exists (select * from sysobjects where type = ''U'' and name=''' + ATableName + ''') '+
          'begin '+
          'drop table ' + ATableName + ' '+
          'end');
        end;
    end;
  except
    FTransaction.RollbackRetaining;
  end;
end;

procedure TSQLDBConnector.TryDropSequence(ASequenceName: String);

var
  NoSeq : Boolean;

begin
  NoSeq:=False;
  try
    case SQLServerType of
      ssInterbase,
      ssFirebird: FConnection.ExecuteDirect('DROP GENERATOR '+ASequenceName);
      ssOracle,
      ssPostgreSQL,
      ssSybase,
      ssMSSQL : FConnection.ExecuteDirect('DROP SEQUENCE '+ASequenceName+' START WITH 1 INCREMENT BY 1');
      ssSQLite : FConnection.ExecuteDirect('delete from sqlite_sequence where (name='''+ASequenceName+''')');
    else
      NoSeq:=True;
    end;
  except
    FTransaction.RollbackRetaining;
  end;
  if NoSeq then
    Raise EDatabaseError.Create('Engine does not support sequences');
end;

procedure TSQLDBConnector.TryCreateSequence(ASequenceName: String);

var
  NoSeq : Boolean;

begin
  NoSeq:=False;
  case SQLServerType of
    ssInterbase,
    ssFirebird: FConnection.ExecuteDirect('CREATE GENERATOR '+ASequenceName);
    ssOracle,
    ssPostgreSQL,
    ssSybase,
    ssMSSQL : FConnection.ExecuteDirect('CREATE SEQUENCE '+ASequenceName+' START WITH 1 INCREMENT BY 1');
    ssSQLite : FConnection.ExecuteDirect('insert into sqlite_sequence (name,seq) values ('''+ASequenceName+''',1)');
  else
    Raise EDatabaseError.Create('Engine does not support sequences');
  end;
end;


procedure TSQLDBConnector.ExecuteDirect(const SQL: string);
begin
  Connection.ExecuteDirect(SQL);
end;

procedure TSQLDBConnector.CommitDDL;
begin
  // Commits schema definition and manipulation statements;
  // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
  if SQLServerType in [ssFirebird, ssInterbase] then
    Transaction.CommitRetaining;
end;

Procedure TSQLDBConnector.FreeTransaction;
begin
  FreeAndNil(FTransaction);
end;

destructor TSQLDBConnector.Destroy;
begin
  FreeAndNil(FQuery);
  if assigned(FTransaction) then
    begin
    try
      if not (stoUseImplicit in Transaction.Options) then
        begin
        if Ftransaction.Active then
          Ftransaction.Rollback;
        Ftransaction.StartTransaction;
        end;
      TryDropIfExist('FPDEV2');
      if not (stoUseImplicit in Transaction.Options) then
        Ftransaction.Commit;
    Except
      if Ftransaction.Active and not (stoUseImplicit in Transaction.Options) then
        Ftransaction.Rollback;
    end; // try
    end;
  FreeTransaction;
  FreeAndNil(FConnection);
  inherited Destroy;
end;

constructor TSQLDBConnector.Create;
begin
  FConnection := nil;
  CreateFConnection;
  FQuery := CreateQuery;
  Inherited;
end;

initialization
  RegisterClass(TSQLDBConnector);
end.

