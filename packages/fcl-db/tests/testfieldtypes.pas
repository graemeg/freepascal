unit TestFieldTypes;

{$mode objfpc}{$H+}
{$modeswitch nestedprocvars}

interface

uses
  Classes, SysUtils, fpcunit, testregistry,
  db;

type
  TParamProc = procedure(AParam:TParam; i : integer);
  TFieldProc = procedure(AField:TField; i : integer);
  TGetSQLTextProc = function(const i: integer) : string; { is nested;}
  TCheckFieldValueProc = procedure(AField:TField; i : integer) is nested;

  { TTestFieldTypes }

  TTestFieldTypes= class(TTestCase)
  private
    procedure CreateTableWithFieldType(ADatatype : TFieldType; ASQLTypeDecl : string);
    procedure TestFieldDeclaration(ADatatype: TFieldType; ADataSize: integer);
    procedure TestSQLFieldType(ADatatype: TFieldType; ASQLTypeDecl: string;
      ADataSize: integer; AGetSQLTextProc: TGetSQLTextProc;
      ACheckFieldValueProc: TCheckFieldValueProc);
    procedure TestXXParamQuery(ADataType : TFieldType; ASQLTypeDecl : string;
      ParamValuesCount : integer); overload;
    procedure TestXXParamQuery(ADataType : TFieldType; ASQLTypeDecl : string;
      ParamValuesCount : integer; const ParamValues: array of string; Cross : boolean = False); overload;
    procedure TestSetBlobAsParam(asWhat : integer);
  protected
    procedure SetUp; override;
    procedure TearDown; override;
    procedure RunTest; override;
  published
    // Fields (field recognition):
    procedure TestInt;
    procedure TestTinyint;
    procedure TestNumeric;
    procedure TestFloat;
    procedure TestDate;
    procedure TestDateTime;       // bug 6925
    procedure TestString;
    procedure TestUnlVarChar;
    procedure TestBlob;
    procedure TestChangeBlob;
    procedure TestBlobGetText;
    procedure TestBlobSize;

    procedure TestSQLClob;
    procedure TestSQLLargeint;
    procedure TestSQLInterval;
    procedure TestSQLIdentity;
    procedure TestSQLReal;

    procedure TestStringLargerThen8192;
    procedure TestInsertLargeStrFields; // bug 9600
    procedure TestLargeRecordSize;
    procedure TestNullValues;
    procedure TestAggregates;
    procedure TestBug9744;
    // Field names:
    procedure TestDoubleFieldNames; // bug 8457
    procedure TestNumericFieldNames; // Bug9661
    procedure TestApplyUpdFieldNames; // Bug 12275

    // Parsing SQL:
    procedure TestParseJoins; // bug 10148
    procedure TestParseUnion; // bug 8442
    procedure TestClearUpdateableStatus;
    procedure TestReadOnlyParseSQL; // bug 9254
    procedure TestDblQuoteEscComments;
    // ApplyUpdates:
    procedure TestpfInUpdateFlag; // bug 7565

    // Parameters:
    procedure TestParamQuery;
    procedure TestStringParamQuery;
    procedure TestFixedStringParamQuery;
    procedure TestDateParamQuery;
    procedure TestCrossStringDateParam;
    procedure TestSmallIntParamQuery;
    procedure TestIntParamQuery;
    procedure TestLargeIntParamQuery;
    procedure TestTimeParamQuery;
    procedure TestDateTimeParamQuery;
    procedure TestFmtBCDParamQuery;
    Procedure TestFmtBCDParamQuery2; // Bug 27077
    procedure TestFloatParamQuery;
    procedure TestCurrencyParamQuery;
    procedure TestBCDParamQuery;
    procedure TestBytesParamQuery;
    procedure TestVarBytesParamQuery;
    procedure TestBooleanParamQuery;
    procedure TestBlobParamQuery;

    procedure TestSetBlobAsMemoParam;
    procedure TestSetBlobAsBlobParam;
    procedure TestSetBlobAsStringParam;

    procedure Test11Params;
    procedure TestCircularParams;
    procedure TestNonNullableParams;
    procedure TestParametersAndDates;

    // Opening non-select statements, which returns result set:
    procedure TestInsertReturningQuery;
    procedure TestOpenStoredProc;
    procedure TestOpenSpecialStatements;

    procedure TestErrorOnEmptyStatement;
    procedure TestExceptOnsecClose;

    procedure TestServerFilter; // bug 15456, 35887
    procedure TestRowsAffected; // bug 9758
    procedure TestLocateNull;
    procedure TestLocateOnMoreRecords;
    procedure TestRefresh;

    // SchemaType tests:
    procedure TestTableNames;
    procedure TestGetTables;
    procedure TestFieldNames;
    procedure TestGetFieldNames;
    procedure TestUpdateIndexDefs;
    procedure TestMultipleFieldPKIndexDefs;
    procedure TestGetIndexDefs;

    // Connection:
    procedure TestEmptyUpdateQuery; // bug 13654
    procedure TestTemporaryTable;
    procedure TestQueryAfterReconnect; // bug 16438

    procedure TestStringsReplace;
    // Test SQLIte3 AlwaysUseBigInt, introduced after bug ID 36486.
    Procedure TestAlwaysUseBigint;
  end;


implementation

uses sqldbtoolsunit,toolsunit, variants, sqldb, bufdataset, strutils, dbconst, FmtBCD;


Type HackedDataset = class(TDataset);

const
  testFloatValuesCount = 21;
  testFloatValues : Array[0..testFloatValuesCount-1] of double = (-maxSmallint-1,-maxSmallint,-256,-255,-128,-127,-1,0,1,127,128,255,256,maxSmallint,maxSmallint+1,0.123456,-0.123456,4.35,12.434E7,9.876e-5,123.45678);

  testBCDValuesCount = 10;
  testBCDValues : Array[0..testBCDValuesCount-1] of currency = (-100,54.53,1.2345,123.5345,0,1,-1,0,1.42,1324.4324);

  testIntValuesCount = 17;
  testIntValues : Array[0..testIntValuesCount-1] of integer = (-maxInt,-maxSmallint-1,-maxSmallint,-256,-255,-128,-127,-1,0,1,127,128,255,256,maxSmallint,maxSmallint+1,MaxInt);

  testDateValuesCount = 18;
  testDateValues : Array[0..testDateValuesCount-1] of string = (
    '2000-01-01',
    '1999-12-31',
    '2004-02-29',
    '2004-03-01',
    '1991-02-28',
    '1991-03-01',
    '2040-10-16',
    '1977-09-29',
    '1899-12-29',
    '1899-12-30',
    '1899-12-31',
    '1900-01-01',
    '1800-03-30',
    '1754-06-04',
    '1650-05-10',
    '0904-04-12',
    '0199-07-09',
    '0001-01-01'
  );

  testBlobValuesCount = 6;
  testBlobValues : Array[0..testBlobValuesCount-1] of ansistring = (
    'Test this Blob',    // common value
    '',                  // empty value
    'a'#0'b'#13#10#1'c', // binary data
    '''":a',             // single quotes
    '\''\',              // backslash
    '\n'#13'\0ab''c'
  );

  testBytesValuesCount = 5;
  testVarBytesValuesCount = 8;
  testBytesValues : Array[0..testVarBytesValuesCount-1] of shortstring = (
    #1#0#1#0#1, #0#0#1#0#1, #0''''#13#0#1, '\'#0'"\'#13, #13#13#0#10#10,
    '', #0, #0#1#2#3#4#5#6#7#8#9
  );


procedure TTestFieldTypes.CreateTableWithFieldType(ADatatype: TFieldType;
  ASQLTypeDecl: string);
begin
  with TSQLDBConnector(DBConnector) do
  begin
    Connection.ExecuteDirect('create table FPDEV2 (FT ' +ASQLTypeDecl+ ')');
    // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
    CommitDDL;
  end;
end;

procedure TTestFieldTypes.TestFieldDeclaration(ADatatype: TFieldType;
  ADataSize: integer);
begin
  with TSQLDBConnector(DBConnector).Query do
    begin
    SQL.Clear;
    SQL.Add('select * from FPDEV2');
    Open;
    AssertEquals(1,FieldCount);
    AssertTrue(SameText('FT',Fields[0].FieldName));
    AssertEquals('DataSize', ADataSize, Fields[0].DataSize);
    AssertEquals('DataType', ord(ADatatype), ord(Fields[0].DataType));
    Close;
    end;
end;

procedure TTestFieldTypes.TestInt;

var
  i          : byte;

begin
  CreateTableWithFieldType(ftInteger,'INT');
  TestFieldDeclaration(ftInteger,4);

  for i := 0 to testIntValuesCount-1 do
    TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (FT) values (' + inttostr(testIntValues[i]) + ')');

  with TSQLDBConnector(DBConnector).Query do
    begin
    Open;
    for i := 0 to testIntValuesCount-1 do
      begin
      AssertEquals(testIntValues[i],fields[0].AsInteger);
      Next;
      end;
    close;
    end;
end;

procedure TTestFieldTypes.TestTinyint;
const
  testValuesCount = 5;
  testValues : Array[0..testValuesCount-1] of byte = (0,1,127,128,255);
var
  datatype: string;
  fieldtype: TFieldType;
  i: integer;
begin
  case SQLServerType of
    ssMSSQL:
      begin
      datatype  := 'TINYINT';
      fieldtype := ftWord;
      end;
    ssMySQL:
      begin
      datatype  := 'TINYINT UNSIGNED';
      fieldtype := ftWord;
      end;
    ssOracle:
      begin
      datatype  := 'NUMBER(3)';
      fieldtype := ftSmallint;
      end;
    ssSQLite:
      begin
      datatype  := 'TINYINT';
      fieldtype := ftSmallint;
      end;
    else
      begin
      fieldtype := ftSmallint;
      datatype  := FieldtypeDefinitions[fieldtype];
      end;
  end;

  CreateTableWithFieldType(fieldtype, datatype);
  TestFieldDeclaration(fieldtype, sizeof(Smallint));

  with TSQLDBConnector(DBConnector) do
  begin
    Query.Open;
    for i := 0 to testValuesCount-1 do
      Query.AppendRecord([testValues[i]]);
    Query.ApplyUpdates;
    Query.Close;

    for i := 0 to testValuesCount-1 do
      ExecuteDirect('insert into FPDEV2 (FT) values (' + inttostr(testValues[i]) + ')');

    Query.Open;
    for i := 0 to testValuesCount*2-1 do
    begin
      AssertEquals(testValues[i mod testValuesCount], Query.Fields[0].AsInteger);
      Query.Next;
    end;
    Query.Close;
  end;
end;

procedure TTestFieldTypes.TestNumeric;

const
  testValuesCount = 13;
  testValues4 : Array[0..testValuesCount-1] of currency = (-99.99,-12.34,-10.2,-10,-0.5,-0.01,0,0.01,0.5,10,10.2,12.34,99.99);
  testValues9 : Array[0..testValuesCount-1] of currency = (-123456.789,-10000,-1875.25,-10,-0.5,-0.001,0,0.001,0.5,10,1875.25,10000,123456.789);
  FieldTypes: array [0..7] of TFieldType = (ftBCD, ftBCD, ftBCD, ftFmtBCD, ftLargeInt, ftFmtBCD, ftFmtBCD, ftFmtBCD);
  FieldSizes: array [0..7] of integer = (4,2,3,5,0,3,5,0); //scale for FieldTypes

var
  i,d        : integer;
  s,s4,s9    : string;
  t          : TFieldType;

begin
  with TSQLDBConnector(DBConnector) do begin
    if SQLConnType = INTERBASE then
    begin
      //Interbase internal storage of exact numeric data types based on precision:
      // 1-4 (smallint), 5-9 (integer), 10-18 (int64)
      s := ''; //Interbase supports precision up to 18 only
      FieldTypes[5] := ftBCD; //ATM TIBConnection incorrectly maps NUMERIC(18,3) to ftBCD
    end
    else
      s := ', N19_0 NUMERIC(19,0)';
    Connection.ExecuteDirect('create table FPDEV2 (FT NUMERIC(18,4), N4_2 NUMERIC(4,2), N9_3 NUMERIC(9,3), N9_5 NUMERIC(9,5), N18_0 NUMERIC(18,0), N18_3 NUMERIC(18,3), N18_5 NUMERIC(18,5)' + s + ')');
    CommitDDL;

    with Query do
    begin
      SQL.Text := 'select * from FPDEV2';
      Open;

      for i := 0 to FieldCount-1 do
      begin
        case Fields[i].DataType of
          ftBCD:      d := sizeof(Currency);
          ftFmtBCD:   d := sizeof(TBCD);
          ftLargeInt: d := sizeof(int64);
          else        d := 0;
        end;
        t := FieldTypes[i];
        if t = ftLargeInt then t := ftFmtBCD; //acceptable alternative

        AssertEquals(Fields[i].DataSize, d);
        AssertTrue(Fields[i].DataType in [FieldTypes[i], t]);
        AssertEquals(Fields[i].Size, FieldSizes[i]);
      end;

      Close;
    end;

    for i := 0 to testValuesCount-1 do
    begin
      s4 := CurrToStrF(testValues4[i],ffFixed,2,DBConnector.FormatSettings);
      s9 := CurrToStrF(testValues9[i],ffFixed,3,DBConnector.FormatSettings);
      Connection.ExecuteDirect(format('insert into FPDEV2 (N4_2,N9_5,FT,N9_3,N18_3,N18_5) values (%s,%s,%s,%s,%s,%s)', [s4,s4,s9,s9,s9,s9]));
    end;

    with Query do
    begin
      Open;
      for i := 0 to testValuesCount-1 do
      begin
        AssertEquals(testValues4[i], Fields[1].AsCurrency);
        AssertEquals(testValues4[i], Fields[3].AsCurrency);
        AssertEquals(testValues9[i], Fields[0].AsCurrency);
        AssertEquals(testValues9[i], Fields[2].AsCurrency);
        AssertEquals(testValues9[i], Fields[5].AsCurrency);
        AssertEquals(testValues9[i], Fields[6].AsCurrency);
        Next;
      end;
      Close;
    end;
  end;
end;

procedure TTestFieldTypes.TestFloat;
const
  testValuesCount = 21;
  testValues : Array[0..testValuesCount-1] of double = (-maxSmallint-1,-maxSmallint,-256,-255,-128,-127,-1,0,1,127,128,255,256,maxSmallint,maxSmallint+1,0.123456,-0.123456,4.35,12.434E7,9.876e-5,123.45678);

var
  i          : byte;

begin
  CreateTableWithFieldType(ftFloat,'FLOAT');
  TestFieldDeclaration(ftFloat,sizeof(double));

  for i := 0 to testValuesCount-1 do
    TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (FT) values (' + floattostr(testValues[i],DBConnector.FormatSettings) + ')');

  with TSQLDBConnector(DBConnector).Query do
    begin
    Open;
    for i := 0 to testValuesCount-1 do
      begin
      AssertEquals(testValues[i],fields[0].AsFloat);
      Next;
      end;
    close;
    end;
end;

procedure TTestFieldTypes.TestString;

const
  testValuesCount = 19;
  testValues : Array[0..testValuesCount-1] of string = (
    '',
    'a',
    'ab',
    'abc',
    'abcd',
    'abcde',
    'abcdef',
    'abcdefg',
    'abcdefgh',
    'abcdefghi',
    'abcdefghij',
    'lMnOpQrStU',
    '1234567890',
    '_!@#$%^&*(',
    ')-;:/?.<>',
    '~`|{}- =',    // note that there's no \  (backslash) since some db's uses that as escape-character
    '  WRaP  ',
    'wRaP  ',
    ' wRAP'
  );

var
  i             : byte;

begin
  CreateTableWithFieldType(ftString,'VARCHAR(10)');
  TestFieldDeclaration(ftString,10*DBConnector.CharSize+1);

  for i := 0 to testValuesCount-1 do
    TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (FT) values (''' + testValues[i] + ''')');

  with TSQLDBConnector(DBConnector).Query do
    begin
    Open;
    for i := 0 to testValuesCount-1 do
      begin
      AssertEquals(testValues[i], Fields[0].AsString);
      AssertEquals('IsNull', False, Fields[0].IsNull); // '' is not NULL
      Next;
      end;
    Close;
    end;
end;

procedure TTestFieldTypes.TestUnlVarChar;

const
  testValuesCount = 21;
  testValues : Array[0..testValuesCount-1] of string = (
    '',
    'a',
    'ab',
    'abc',
    'abcd',
    'abcde',
    'abcdef',
    'abcdefg',
    'abcdefgh',
    'abcdefghi',
    'abcdefghij',
    'lMnOpQrStU',
    '1234567890',
    '_!@#$%^&*(',
    ')-;:/?.<>',
    '~`|{}- =',
    '  WRaP  ',
    'wRaP  ',
    ' wRAP',
    '0123456789',
    'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 !@#$%^&*()_+-=][|}{;:,./<>?abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 !@#$%^&*()_+-=][|}{;:,./<>?' + 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 !@#$%^&*()_+-=][|}{;:,./<>?'
    + 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 !@#$%^&*()_+-=][|}{;:,./<>?abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 !@#$%^&*()_+-=][|}{;:,./<>?' + 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 !@#$%^&*()_+-=][|}{;:,./<>?'
  );

var
  i             : byte;

begin
  if SQLServerType<>ssPostgreSQL then Ignore('This test only applies to Postgres, since others don''t support varchars without length given');

  CreateTableWithFieldType(ftString,'VARCHAR');
  TestFieldDeclaration(ftString,dsMaxStringSize+1);

  for i := 0 to testValuesCount-1 do
    TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (FT) values (''' + testValues[i] + ''')');

  with TSQLDBConnector(DBConnector).Query do
    begin
    Open;
    for i := 0 to testValuesCount-1 do
      begin
      AssertEquals(testValues[i],fields[0].AsString);
      Next;
      end;
    close;
    end;
end;

procedure TTestFieldTypes.TestDate;

var
  i             : byte;

begin
  CreateTableWithFieldType(ftDate,'DATE');
  TestFieldDeclaration(ftDate,8);

  for i := 0 to testDateValuesCount-1 do
    if SQLConnType=oracle then
      TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (FT) values (to_date (''' + testDateValues[i] + ''',''YYYY-MM-DD''))')
    else
      TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (FT) values (''' + testDateValues[i] + ''')');

  with TSQLDBConnector(DBConnector).Query do
    begin
    Open;
    for i := 0 to testDateValuesCount-1 do
      begin
      AssertEquals(testDateValues[i],FormatDateTime('yyyy/mm/dd', fields[0].AsDateTime, DBConnector.FormatSettings));
      Next;
      end;
    close;
    end;

end;

procedure TTestFieldTypes.TestDateTime;

const
  testValuesCount = 31;
  testValues : Array[0..testValuesCount-1] of string = (
    '2000-01-01',
    '1999-12-31',
    '2004-02-29',
    '2004-03-01',
    '1991-02-28',
    '1991-03-01',
    '1977-09-29',
    '2000-01-01 10:00:00',
    '2000-01-01 23:59:59',
    '1994-03-06 11:54:30',
    '2040-10-16',                   // MySQL 4.0 doesn't support datetimes before 1970 or after 2038
    '2100-01-01 01:01:01',
    '1903-04-02 01:04:02',
    '1900-01-01',
    '1899-12-31',
    '1899-12-30',
    '1899-12-29',
    '1899-12-30 18:00:51',
    '1899-12-30 04:00:51',
    '1899-12-29 04:00:51',
    '1899-12-29 18:00:51',
    '1815-09-24 03:47:22',
    '1800-03-30',
    '1754-06-04',
    '1650-05-10',                   // MS SQL 2005 doesn't support datetimes before 1753
    '1400-02-03 12:21:53',
    '1333-02-03 21:44:21',
    '0904-04-12',
    '0354-11-20 21:25:15',
    '0199-07-09',
    '0001-01-01'
  );

var
  i : byte;

begin
  CreateTableWithFieldType(ftDateTime,FieldtypeDefinitions[ftDateTime]);
  TestFieldDeclaration(ftDateTime,8);

  for i := 0 to testValuesCount-1 do
    if SQLConnType=oracle then
      TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (FT) values (to_date (''' + testValues[i] + ''',''YYYY-MM-DD HH24:MI:SS''))')
    else
      TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (FT) values (''' + testValues[i] + ''')');

  with TSQLDBConnector(DBConnector).Query do
    begin
    Open;
    for i := 0 to testValuesCount-1 do
      begin
      if length(testValues[i]) < 12 then
        AssertEquals(testValues[i],FormatDateTime('yyyy/mm/dd', fields[0].AsDateTime, DBConnector.FormatSettings))
      else
        AssertEquals(testValues[i],FormatDateTime('yyyy/mm/dd hh:mm:ss', fields[0].AsDateTime, DBConnector.FormatSettings));
      Next;
      end;
    close;
    end;
end;

procedure TTestFieldTypes.TestBlob;

begin
  CreateTableWithFieldType(ftBlob,FieldtypeDefinitions[ftBlob]);
  TestFieldDeclaration(ftBlob,0);

  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (FT) values (''Test this blob'')');

  with TSQLDBConnector(DBConnector).Query do
    begin
    Open;
    AssertEquals('Test this blob',fields[0].AsString);
    close;
    end;
end;

procedure TTestFieldTypes.TestChangeBlob;
var s : string;
begin
  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('create table FPDEV2 (ID int,FT '+FieldtypeDefinitions[ftblob]+')');
  TSQLDBConnector(DBConnector).CommitDDL;

  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (ID,FT) values (1,''Test this blob'')');

  with TSQLDBConnector(DBConnector).Query do
    begin
    sql.clear;
    sql.add('select * from FPDEV2');
    Open;
    AssertEquals('Test this blob', Fields[1].AsString);
    Fields[1].ProviderFlags := [pfInUpdate]; // Blob is not in the where
    UpdateMode := upWhereAll;

    Edit;
    s := 'This blob has changed!';
    Fields[1].AsString := s;
    AssertEquals(s, Fields[1].AsString); // test before Post
    Cancel;
    AssertEquals('After Cancel', 'Test this blob', Fields[1].AsString); // original value

    Append; // Blob is null
    Fields[1].AsString := s; // Null flag must be unset
    AssertEquals(s, Fields[1].AsString);
    Fields[1].Clear;
    AssertTrue('Clear', Fields[1].IsNull);
    Cancel;

    Edit;
    With CreateBlobStream(Fields[1], bmWrite) do
      begin
      s := 'This blob has changed!';
      WriteBuffer(Pointer(s)^,Length(s));
      Post;
      Free;
      end;
    AssertEquals('After Post', s, Fields[1].AsString);

    ApplyUpdates(0);
    TSQLDBConnector(DBConnector).Transaction.CommitRetaining; // For debug-purposes
    Close;

    Open;
    AssertEquals(s, Fields[1].AsString);
    Close;
    end;
end;

procedure TTestFieldTypes.TestBlobGetText;
begin
  CreateTableWithFieldType(ftBlob,FieldtypeDefinitions[ftBlob]);
  TestFieldDeclaration(ftBlob,0);

  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (FT) values (''Test this blob'')');
  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (FT) values (Null)');

  with TSQLDBConnector(DBConnector).Query do
    begin
    Open;
    AssertFalse(fields[0].IsNull);
    AssertEquals('(BLOB)',fields[0].DisplayText);
    AssertEquals('Test this blob',fields[0].AsString);
    Next;
    AssertTrue(fields[0].IsNull);
    AssertEquals('(blob)',fields[0].Text);
    AssertEquals('',fields[0].AsString);
    close;
    end;
end;

procedure TTestFieldTypes.TestBlobSize;
const
  TestValue: string = 'Test this blob';
begin
  CreateTableWithFieldType(ftBlob,FieldtypeDefinitions[ftBlob]);

  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (FT) values ('''+TestValue+''')');

  with TSQLDBConnector(DBConnector).Query do
    begin
    sql.text := 'select * from FPDEV2';
    Open;
    AssertEquals(length(TestValue),TBlobField(fields[0]).BlobSize);
    close;
    end;
end;

procedure TTestFieldTypes.TestSQLFieldType(ADatatype : TFieldType; ASQLTypeDecl : string; ADataSize: integer; AGetSQLTextProc: TGetSQLTextProc; ACheckFieldValueProc: TCheckFieldValueProc);
var
  i          : byte;
  s: string;
begin
  CreateTableWithFieldType(ADatatype,ASQLTypeDecl);
  TestFieldDeclaration(ADatatype,ADataSize);

  for i := 0 to testValuesCount-1 do
    begin
    s := AGetSQLTextProc(i);
    TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (FT) values (' + s + ')');
    end;

  with TSQLDBConnector(DBConnector).Query do
    begin
    Open;
    for i := 0 to testValuesCount-1 do
      begin
      ACheckFieldValueProc(fields[0],i);
      Next;
      end;
    close;
    end;
end;

// Placed here, as long as bug 18702 is not solved
function TestSQLClob_GetSQLText(const a: integer) : string;
begin
  result := QuotedStr(testStringValues[a]);
end;

procedure TTestFieldTypes.TestSQLClob;
  procedure CheckFieldValue(AField:TField; a : integer);
  begin
    AssertEquals(testStringValues[a],AField.AsString);
  end;
var datatype: string;
begin
  if SQLConnType=sqlite3 then
    datatype:='CLOB'
  else
    datatype:=FieldtypeDefinitions[ftMemo];
  TestSQLFieldType(ftMemo, datatype, 0, @TestSQLClob_GetSQLText, @CheckFieldValue);
end;

// Placed here, as long as bug 18702 is not solved
function TestSQLLargeInt_GetSQLText(const a: integer) : string;
begin
  result := IntToStr(testLargeIntValues[a]);
end;

procedure TTestFieldTypes.TestSQLLargeint;
  procedure CheckFieldValue(AField:TField; a : integer);
  begin
    AssertEquals(testLargeIntValues[a],AField.AsLargeInt);
  end;
var datatype: string;
begin
  if SQLConnType=sqlite3 then
    datatype:='LARGEINT'
  else
    datatype:='BIGINT';
  TestSQLFieldType(ftLargeint, datatype, 8, @TestSQLLargeint_GetSQLText, @CheckFieldValue);
end;

var testIntervalValuesCount: integer;
const testIntervalValues: array[0..5] of shortstring = ('00:00:00.000','00:00:01.000','23:59:59.000','99:59:59.000','838:59:59.000','1000:00:00.000');
// Placed here, as long as bug 18702 is not solved
function TestSQLInterval_GetSQLText(const a: integer) : string;
begin
  if a < testIntervalValuesCount then
    Result := QuotedStr(testIntervalValues[a])
  else
    Result := 'NULL'
end;
procedure TTestFieldTypes.TestSQLInterval;
  procedure CheckFieldValue(AField: TField; a: integer);
  begin
    if a < testIntervalValuesCount then
      AssertEquals(testIntervalValues[a], DateTimeToTimeString(AField.AsDateTime))
    else
      AssertTrue(AField.IsNull);
  end;
var datatype: string;
begin
  if SQLConnType = postgresql then
  begin
    datatype:='INTERVAL';
    testIntervalValuesCount := 6;
  end
  else
  begin
    datatype:=FieldtypeDefinitions[ftTime];
    if datatype = '' then
      Ignore(STestNotApplicable);
    if SQLServerType = ssSQLite then
      testIntervalValuesCount := 6
    else if SQLServerType = ssMySQL then
      // MySQL ODBC driver prior 5.2.6 doesn't correctly handles time values >= '100:00:00'
      testIntervalValuesCount := 5
    else
      testIntervalValuesCount := 3;
  end;
  TestSQLFieldType(ftTime, datatype, sizeof(TDateTime), @TestSQLInterval_GetSQLText, @CheckFieldValue);
end;

procedure TTestFieldTypes.TestSQLIdentity;
var datatype, values: string;
    fieldtype: TFieldType;
    i: integer;
    updatable: boolean;
begin
  case SQLServerType of
    ssMySQL:
      begin
      datatype:='INT AUTO_INCREMENT PRIMARY KEY';
      values:='VALUES(DEFAULT)';
      fieldtype:=ftAutoInc;
      updatable:=true;
      end;
    ssSQLite:
      begin
      datatype:='INTEGER PRIMARY KEY';
      values:='DEFAULT VALUES';
      fieldtype:=ftAutoInc;
      updatable:=true;
      end;
    ssPostgreSQL:
      begin
      datatype:='SERIAL';
      values:='DEFAULT VALUES';
      if SQLConnType = ODBC then
        fieldtype:=ftAutoInc
      else
        fieldtype:=ftInteger;
      updatable:=true;
      end;
    ssMSSQL, ssSybase:
      begin
      datatype:='INTEGER IDENTITY';
      values:='DEFAULT VALUES';
      fieldtype:=ftAutoInc;
      updatable:=false;
      end
    else
      Ignore(STestNotApplicable);
  end;
  CreateTableWithFieldType(fieldtype, datatype);
  TestFieldDeclaration(fieldtype, sizeof(longint));

  for i := 1 to 3 do
    TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 '+values);

  with TSQLDBConnector(DBConnector).Query do
  begin
    Open;
    AssertTrue(Locate('FT',1,[])); // bug 17624
    for i := 1 to 3 do
    begin
      AssertEquals(i, Fields[0].AsInteger);
      Next;
    end;
    // some databases (like MS SQL Server) do not allow updating identity columns
    AssertEquals('ReadOnly', Fields[0].ReadOnly, not updatable);
    // some databases (like PostgreSQL, MySQL) allow inserting explicit values and updating auto incrementing columns
    if updatable then
    begin
      UpdateMode:=upWhereAll; // if there is no PK for FPDEV2 table
      // updating:
      Last;
      while not Bof do
      begin
        Edit;
        Fields[0].AsInteger:=Fields[0].AsInteger+2;
        Post;
        Prior;
      end;
      // inserting:
      Append;
      Fields[0].AsInteger:=6;
      Post;
      ApplyUpdates;
    end;
    Close;
  end;
end;

function TestSQLReal_GetSQLText(const i: integer) : string;
begin
  if i < 20 then // first 20 values fit into MySQL FLOAT data type
    Result := FloatToStr(testFloatValues[i], DBConnector.FormatSettings)
  else
    Result := 'NULL';
end;
procedure TTestFieldTypes.TestSQLReal;
  procedure CheckFieldValue(AField:TField; i: integer);
  begin
    if i < 20 then
      AssertEquals(testFloatValues[i], AField.AsFloat)
    else
      AssertTrue(AField.IsNull);
  end;
var datatype: string;
begin
  case SQLServerType of
    ssFirebird, ssInterbase,
    ssMySQL:
      datatype:='FLOAT';
    ssOracle:
      datatype:='BINARY_FLOAT';
    else
      datatype:='REAL';
  end;
  TestSQLFieldType(ftFloat, datatype, sizeof(double), @TestSQLReal_GetSQLText, @CheckFieldValue);
end;


procedure TTestFieldTypes.TestStringLargerThen8192;
// See also: TestInsertLargeStrFields
var
  s             : string;
  i             : integer;

begin
  // Firebird has limit 32765 bytes, so this is 8191 characters when using UNICODE character set
  CreateTableWithFieldType(ftString,'VARCHAR(9000)');
  TestFieldDeclaration(ftString,9000*DBConnector.CharSize+1);

  setlength(s,9000);
  for i := 1 to 9000 do
    s[i]:=chr((i mod 10)+ord('a'));
  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (FT) values (''' + s + ''')');

  with TSQLDBConnector(DBConnector).Query do
    begin
    Open;
    AssertEquals(s,Fields[0].AsString);
    Close;
    end;
end;

procedure TTestFieldTypes.TestInsertLargeStrFields;
// See also: TestStringLargerThen8192
var
  FieldValues: array [1..4] of string;
  i,l1,l2: integer;
begin
  FieldValues[1] := 'test1';                  // string length < 8192 (dsMaxStringSize)
  FieldValues[2] := StringOfChar('a', 8192);  // string length = 8192 (dsMaxStringSize)
  FieldValues[3] := StringOfChar('b', 16000); // string length > 8192 (dsMaxStringSize)
  FieldValues[4] := StringOfChar('c', 17000); // string length > Field.Size

  with TSQLDBConnector(DBConnector) do
    begin
    Connection.ExecuteDirect( 'create table FPDEV2 (' +
                              '  ID INT NOT NULL , ' +
                              '  F1 VARCHAR(8192), ' +
                              '  F2 VARCHAR(16000),' +
                              'primary key (ID) )');
    // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
    TSQLDBConnector(DBConnector).CommitDDL;

    query.sql.Text:='select * from FPDEV2';
    Query.Open;
    for i:=1 to high(FieldValues) do
      Query.InsertRecord([i, FieldValues[i], FieldValues[i]]);
    Query.ApplyUpdates;
    Query.Close;
    Query.Open;
    for i:=1 to high(FieldValues) do
      begin
      l1:=length(FieldValues[i]);
      if l1 > 8192  then l1 := 8192;
      l2:=length(FieldValues[i]);
      if l2 > 16000 then l2 := 16000;

      AssertEquals(l1, length(Query.FieldByName('F1').AsString));
      AssertEquals(l2, length(Query.FieldByName('F2').AsString));
      AssertEquals(copy(FieldValues[i],1,l1), Query.FieldByName('F1').AsString);
      AssertEquals(copy(FieldValues[i],1,l2), Query.FieldByName('F2').AsString);
      Query.Next;
      end;
    Query.Close;
    end;
end;

procedure TTestFieldTypes.TestLargeRecordSize;
// probably tests situation, where total record size > 64K (TDataSet.RecordSize returns word value)
begin
  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('create table FPDEV2 (plant varchar(8192),sampling_type varchar(8192),area varchar(8192), area_description varchar(8192), batch varchar(8192), sampling_datetime timestamp, status varchar(8192), batch_commentary varchar(8192))');

  // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
  TSQLDBConnector(DBConnector).CommitDDL;

  with TSQLDBConnector(DBConnector).Query do
    begin
    sql.clear;
    sql.append('insert into FPDEV2 (plant,sampling_type,batch,sampling_datetime,status,batch_commentary) values (''ZUBNE PASTE'',''OTISCI POVRŠINA'',''000037756'',''2005-07-01'',''NE ODGOVARA'',''Ovdje se upisuje komentar o kontrolnom broju..............'')');
    ExecSQL;

    sql.clear;
    sql.append('select * from FPDEV2');
    open;
    AssertEquals('ZUBNE PASTE',FieldByName('plant').AsString);
    AssertEquals(EncodeDate(2005,07,01),FieldByName('sampling_datetime').AsDateTime);
    close;
    end;
end;

procedure TTestFieldTypes.TestNullValues;
begin
  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('create table FPDEV2 (FIELD1 INT, FIELD2 INT)');
  // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
  TSQLDBConnector(DBConnector).CommitDDL;

  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (FIELD1) values (1)');

  with TSQLDBConnector(DBConnector).Query do
    begin
    sql.clear;
    sql.append('select * from FPDEV2');
    open;
    AssertEquals(1,FieldByName('FIELD1').AsInteger);
    AssertTrue('Null-values test failed',FieldByName('FIELD2').IsNull);
    close;
    end;
end;

procedure TTestFieldTypes.TestAggregates;
begin
  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('create table FPDEV2 (FIELD1 INT, FIELD2 INT)');
  // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
  TSQLDBConnector(DBConnector).CommitDDL;

  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 values (1,1)');
  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 values (2,3)');
  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 values (3,4)');
  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 values (4,4)');

  TSQLDBConnector(DBConnector).Transaction.CommitRetaining;

  with TSQLDBConnector(DBConnector).Query do
    begin
    sql.clear;
    sql.append('select count(*) from FPDEV2');
    open;
    AssertEquals(4,Fields[0].AsInteger);
    close;

    sql.clear;
    sql.append('select sum(FIELD1) from FPDEV2');
    open;
    AssertEquals(10,Fields[0].AsInteger);
    close;

    sql.clear;
    sql.append('select avg(FIELD2) from FPDEV2');
    open;
    AssertEquals(3,Fields[0].AsInteger);
    close;
    end;
end;

procedure TTestFieldTypes.TestBug9744;
var i : integer;
begin
  // Tests rev.8703: "Fixed MySQL ftLargeInt support"; count() returns BIGINT values
  with TSQLDBConnector(DBConnector) do
    begin
    try
      Connection.ExecuteDirect('create table TTTOBJ ( ' +
                                '  ID INT NOT NULL,   ' +
                                '  NAME VARCHAR(250), ' +
                                '  PRIMARY KEY (ID)   ' +
                                ')                    ');
      Connection.ExecuteDirect('create table TTTXY (  ' +
                                '  ID INT NOT NULL,   ' +
                                '  NP INT NOT NULL,   ' +
                                '  PRIMARY KEY (ID,NP)' +
                                ')                    ');
      Transaction.CommitRetaining;
      for i := 0 to 7 do
        begin
        connection.ExecuteDirect('insert into TTTOBJ(ID,NAME) values ('+inttostr(i)+',''A'+inttostr(i)+''')');
        connection.ExecuteDirect('insert into TTTXY(ID,NP) values ('+inttostr(i)+',1)');
        connection.ExecuteDirect('insert into TTTXY(ID,NP) values ('+inttostr(i)+',2)');
        end;
      Query.SQL.Text := 'select OBJ.ID, OBJ.NAME, count(XY.NP) as NPF from TTTOBJ OBJ, TTTXY XY where OBJ.ID=XY.ID group by OBJ.ID, OBJ.NAME';
      query.Prepare;
      query.open;
      query.close;
    finally
      Connection.ExecuteDirect('drop table TTTXY');
      Connection.ExecuteDirect('drop table TTTOBJ');
      Transaction.CommitRetaining;
    end
  end;
end;


procedure TTestFieldTypes.TestDoubleFieldNames;
begin
  with TSQLDBConnector(DBConnector) do
    begin
    with query do
      begin
      SQL.Text:='select FPDEV.*,TT.* from FPDEV left join FPDEV TT on TT.ID=FPDEV.ID';
      Open;
      AssertTrue(assigned(FindField('ID')));
      AssertTrue (assigned(FindField('ID_1')));
      AssertTrue(assigned(FindField('NAME')));
      AssertTrue(assigned(FindField('NAME_1')));

      AssertEquals(1,fieldbyname('ID').AsInteger);
      AssertEquals(1,fieldbyname('ID_1').AsInteger);
      AssertEquals('TestName1',fieldbyname('NAME').AsString);
      AssertEquals('TestName1',fieldbyname('NAME_1').AsString);
      close;
      end;
    end;
end;

procedure TTestFieldTypes.TestNumericFieldNames;
begin
  with TSQLDBConnector(DBConnector) do
    begin
    Connection.ExecuteDirect('create table FPDEV2 (' +
                              '  '+connection.FieldNameQuoteChars[0]+'2ID'+connection.FieldNameQuoteChars[1]+' INT NOT NULL,' +
                              '  '+connection.FieldNameQuoteChars[0]+'3TEST'+connection.FieldNameQuoteChars[1]+' VARCHAR(10),' +
                              '  PRIMARY KEY ('+connection.FieldNameQuoteChars[0]+'2ID'+connection.FieldNameQuoteChars[0]+') ' +
                              ') ');
    // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
    TSQLDBConnector(DBConnector).CommitDDL;

    with query do
      begin
      SQL.Text:='select * from FPDEV2';
      Open;
      Edit;
      fieldbyname('2ID').AsInteger:=1;
      fieldbyname('3TEST').AsString:='3test';
      Post;
      ApplyUpdates(0);
      close;
      open;
      AssertEquals('3test',FieldByName('3TEST').AsString);
      Edit;
      fieldbyname('3TEST').AsString:='test3';
      Post;
      ApplyUpdates(0);
      open;
      AssertEquals('test3',FieldByName('3TEST').AsString);
      close;
      end;
    end;
end;

procedure TTestFieldTypes.TestApplyUpdFieldNames;
begin
  with TSQLDBConnector(DBConnector) do
    begin
    AssertEquals(-1,query.RowsAffected);
    Connection.ExecuteDirect('create table FPDEV2 (' +
                              '  ID INT NOT NULL,  ' +
                              '  '+Connection.FieldNameQuoteChars[0]+'NAME-TEST'+Connection.FieldNameQuoteChars[1]+' VARCHAR(250), ' +
                              '  PRIMARY KEY (ID)  ' +
                              ')                   ');
    // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
    TSQLDBConnector(DBConnector).CommitDDL;

    Connection.ExecuteDirect('insert into FPDEV2(ID,'+Connection.FieldNameQuoteChars[0]+'NAME-TEST'+Connection.FieldNameQuoteChars[1]+') values (1,''test1'')');
    Query.SQL.Text := 'select * from FPDEV2';
    Query.Open;
    AssertEquals(1,Query.FieldByName('ID').AsInteger);
    AssertEquals('test1',Query.FieldByName('NAME-TEST').AsString);
    Query.Edit;
    Query.FieldByName('NAME-TEST').AsString:='Edited';
    Query.Post;
    Query.ApplyUpdates;
    Query.Close;
    Query.Open;
    AssertEquals(1,Query.FieldByName('ID').AsInteger);
    AssertEquals('Edited',Query.FieldByName('NAME-TEST').AsString);
    Query.Close;
    end;
end;


procedure TTestFieldTypes.TestParseJoins;
begin
  with TSQLDBConnector(DBConnector) do
    begin
    with query do
      begin
      SQL.Text:='select TT.NAME from FPDEV left join FPDEV TT on TT.ID=FPDEV.ID';
      Open;
      AssertFalse(CanModify);
      Close;

      SQL.Text:='select T1.NAME from FPDEV T1,FPDEV T2 where T1.ID=T2.ID';
      Open;
      AssertFalse(CanModify);
      Close;
      end;
    end;
end;

procedure TTestFieldTypes.TestParseUnion;
begin
  with TSQLDBConnector(DBConnector) do
    begin
    with query do
      begin
      SQL.Text:='select NAME from FPDEV where ID<5';
      SQL.Add('union');
      SQL.Add('select NAME from FPDEV where ID>5');
      Open;
      close;
      end;
    end;
end;

procedure TTestFieldTypes.TestClearUpdateableStatus;
// Test if CanModify is correctly disabled in case of a select query without
// a from-statement.
begin
  if not (SQLServerType in [ssMySQL]) then Ignore('This test only applies to MySQL because the used SQL-statement is MySQL only.');
  with TSQLDBConnector(DBConnector) do
    begin
    with (GetNDataset(false,5) as TSQLQuery) do
      begin
      Open;
      AssertEquals(True,CanModify);
      Close;
      SQL.Text:='select last_insert_id();';
      Open;
      AssertEquals(False,CanModify);
      close;
      end;
    end;
end;

procedure TTestFieldTypes.TestReadOnlyParseSQL;
begin
  with TSQLDBConnector(DBConnector) do
    begin
    GetFieldDataset(True);
    with query do
      begin
      AssertFalse(ReadOnly);
      AssertTrue(ParseSQL);

      // If ParseSQL is false, and no update-queries are given, the query
      // shouldn't be updateable after open.
      ParseSQL := False;
      AssertFalse(ParseSQL);
      AssertFalse(ReadOnly);
      SQL.Text := 'select * from FPDEV;';
      open;
      AssertFalse(ParseSQL);
      AssertFalse(ReadOnly);
      AssertFalse(CanModify);
      close;

      // If ParseSQL is true, the query should be updateable after open.
      ReadOnly := False;
      ParseSQL := True;
      AssertTrue(ParseSQL);
      AssertFalse(ReadOnly);
      SQL.Text := 'select * from FPDEV';
      open;
      AssertTrue(ParseSQL);
      AssertFalse(ReadOnly);
      AssertTrue(CanModify);
      edit;
      FieldByName('ID').AsInteger:=321;
      post;
      ApplyUpdates;
      close;

      // If ParseSQL is true, but the supplied query isn't updateable, then
      // the query shouldn't be updateable after open.
      ReadOnly := False;
      SQL.Text:='select ID,NAME from FPDEV where ID<5';
      sql.Add('union');
      sql.Add('select ID,NAME from FPDEV where ID>5');
      AssertTrue(ParseSQL);
      AssertFalse(ReadOnly);
      open;
      AssertTrue(ParseSQL);
      AssertFalse(ReadOnly);
      AssertFalse(CanModify);
      close;

      // As above, but now with an update-query, so that the query should
      // be updateable again.
      ReadOnly := False;
      AssertTrue(ParseSQL);
      AssertFalse(ReadOnly);
      UpdateSQL.Text:='update FPDEV set ID=:ID where ID=:OLD_ID';
      open;
      AssertTrue(ParseSQL);
      AssertFalse(ReadOnly);
      AssertTrue(CanModify);
      edit;
      post;
      ApplyUpdates;
      close;

      // Also if ParseSQL is False, the query should be updateable if a update-
      // query is given.
      ReadOnly := False;
      ParseSQL := False;
      AssertFalse(ParseSQL);
      AssertFalse(ReadOnly);
      open;
      AssertFalse(ParseSQL);
      AssertFalse(ReadOnly);
      AssertTrue(CanModify);
      edit;
      FieldByName('ID').AsInteger:=1; // field "ID" from UNION can be marked as ReadOnly
      post;
      ApplyUpdates;
      close;

      // But if ReadOnly is true, then CanModify should always be false
      ReadOnly := True;
      ParseSQL := False;
      AssertFalse(ParseSQL);
      AssertTrue(ReadOnly);
      open;
      AssertFalse(ParseSQL);
      AssertTrue(ReadOnly);
      AssertFalse(CanModify);
      close;
      end;
    end;
end;

procedure TTestFieldTypes.TestDblQuoteEscComments;
begin
  with TSQLDBConnector(DBConnector).Query do
    begin
    SQL.Clear;
    SQL.Add('select * from FPDEV where NAME=''test '''' and :ThisIsNotAParameter ''');
    Open;
    Close;
    end;
end;

procedure TTestFieldTypes.TestpfInUpdateFlag;
var ds   : TCustomBufDataset;
    AFld1, AFld2, AFld3 : Tfield;
begin
  ds := (DBConnector.GetNDataset(True,5) as TCustomBufDataset);
  with ds do
    begin
    AFld1 := TIntegerField.Create(ds);
    AFld1.FieldName := IdentifierCase('ID');
    AFld1.DataSet := ds;
    AFld1.ProviderFlags := AFld1.ProviderFlags + [pfInKey];

    AFld2 := TStringField.Create(ds);
    AFld2.FieldName := IdentifierCase('NAME');
    AFld2.DataSet := ds;

    AFld3 := TIntegerField.Create(ds);
    AFld3.FieldName := 'CALCFLD';
    AFld3.DataSet := ds;
    Afld3.FieldKind := fkCalculated;
    AFld3.ProviderFlags := [];  // do not include calculated fields into generated sql insert/update

    Open;
    Edit;
    FieldByName('ID').AsInteger := 254;
    Post;
    ApplyUpdates;
    Append;
    FieldByName('ID').AsInteger := 255;
    Post;
    ApplyUpdates;
    Close;
    AFld1.Free;
    AFld2.Free;
    AFld3.Free;
    end;
end;


procedure TTestFieldTypes.TestParamQuery;
// Tests running insert queries using parameters
const
  DecoyFieldData1='decoytest';
  DecoyFieldData2=':decoy ::test $decoy2 $$2';
begin
  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('create table FPDEV2 (FIELD1 INT, FIELD2 INT, FIELD3 INT, DECOY VARCHAR(30))');

  // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
  TSQLDBConnector(DBConnector).CommitDDL;

  with TSQLDBConnector(DBConnector).Query do
    begin
    sql.clear;
    sql.append('insert into FPDEV2 (FIELD1) values (:field1)');
    Params.ParamByName('field1').AsInteger := 1;
    ExecSQL;

    sql.clear;
    sql.append('insert into FPDEV2 (FIELD1,FIELD2,DECOY) values (:field1,:field2,'''+DecoyFieldData1+''')');
    Params.ParamByName('field1').AsInteger := 2;
    Params.ParamByName('field2').DataType := ftInteger;
    Params.ParamByName('field2').Value := Null;
    ExecSQL;

    sql.clear;
    sql.append('insert into FPDEV2 (FIELD1,FIELD2,FIELD3) values (:field1,:field2,:field3)');
    Params.ParamByName('field1').AsInteger := 3;
    Params.ParamByName('field2').AsInteger := 2;
    Params.ParamByName('field3').AsInteger := 3;
    ExecSQL;

    sql.clear;
    sql.append('insert into FPDEV2 (FIELD1,FIELD2,FIELD3,DECOY) values (:field1,:field2,:field3,'''+DecoyFieldData2+''')');
    Params.ParamByName('field1').AsInteger := 4;
    Params.ParamByName('field2').AsInteger := 2;
    Params.ParamByName('field3').AsInteger := 3;
    ExecSQL;

    sql.clear;
    sql.append('insert into FPDEV2 (FIELD1,FIELD2,FIELD3) values (:field1,:field2,:field1)');
    Params.ParamByName('field1').AsInteger := 5;
    Params.ParamByName('field2').AsInteger := 2;
    ExecSQL;

    sql.clear;
    sql.append('select * from FPDEV2 order by FIELD1');
    open;
    AssertEquals(1,FieldByName('FIELD1').asinteger);
    AssertTrue(FieldByName('FIELD2').IsNull);
    AssertTrue(FieldByName('FIELD3').IsNull);
    AssertTrue(FieldByName('DECOY').IsNull);
    next;
    AssertEquals(2,FieldByName('FIELD1').asinteger);
    AssertTrue(FieldByName('FIELD2').IsNull);
    AssertTrue(FieldByName('FIELD3').IsNull);
    AssertEquals(DecoyFieldData1,FieldByName('DECOY').AsString);
    next;
    AssertEquals(3,FieldByName('FIELD1').asinteger);
    AssertEquals(2,FieldByName('FIELD2').asinteger);
    AssertEquals(3,FieldByName('FIELD3').asinteger);
    AssertTrue(FieldByName('DECOY').IsNull);
    next;
    AssertEquals(4,FieldByName('FIELD1').asinteger);
    AssertEquals(2,FieldByName('FIELD2').asinteger);
    AssertEquals(3,FieldByName('FIELD3').asinteger);
    AssertEquals(DecoyFieldData2,FieldByName('DECOY').AsString);
    next;
    AssertEquals(5,FieldByName('FIELD1').asinteger);
    AssertEquals(2,FieldByName('FIELD2').asinteger);
    AssertEquals(5,FieldByName('FIELD3').asinteger);
    AssertTrue(FieldByName('DECOY').IsNull);
    close;

    end;
  TSQLDBConnector(DBConnector).Transaction.CommitRetaining;
end;

procedure TTestFieldTypes.TestSmallIntParamQuery;
begin
  TestXXParamQuery(ftSmallInt,FieldtypeDefinitions[ftSmallInt],testValuesCount);
end;

procedure TTestFieldTypes.TestIntParamQuery;
begin
  TestXXParamQuery(ftInteger,'INT',testIntValuesCount);
end;

procedure TTestFieldTypes.TestLargeIntParamQuery;
begin
  TestXXParamQuery(ftLargeInt,FieldtypeDefinitions[ftLargeInt],testValuesCount);
end;

procedure TTestFieldTypes.TestFmtBCDParamQuery;
begin
  TestXXParamQuery(ftFMTBcd, FieldtypeDefinitions[ftFMTBcd], testValuesCount, testFmtBCDValues);
end;

Procedure TTestFieldTypes.TestFmtBCDParamQuery2;
begin
  // This test tests FmtBCD params with smaller precision, which fits into INT32
  // TestFmtBCDParamQuery tests FmtBCD params with bigger precision, which fits into INT64
  TestXXParamQuery(ftFMTBcd, 'NUMERIC(9,5)', 2, ['1234','1234.56781']);
end;

procedure TTestFieldTypes.TestDateParamQuery;
begin
  TestXXParamQuery(ftDate,FieldtypeDefinitions[ftDate],testDateValuesCount);
end;

procedure TTestFieldTypes.TestCrossStringDateParam;
begin
  TestXXParamQuery(ftDate,FieldtypeDefinitions[ftDate],testDateValuesCount,[],True);
end;

procedure TTestFieldTypes.TestTimeParamQuery;
begin
  TestXXParamQuery(ftTime,FieldtypeDefinitions[ftTime],testValuesCount);
end;

procedure TTestFieldTypes.TestDateTimeParamQuery;
begin
  TestXXParamQuery(ftDateTime,FieldtypeDefinitions[ftDateTime],testValuesCount);
end;

procedure TTestFieldTypes.TestFloatParamQuery;

begin
  TestXXParamQuery(ftFloat,FieldtypeDefinitions[ftFloat],testFloatValuesCount);
end;

procedure TTestFieldTypes.TestCurrencyParamQuery;
begin
  TestXXParamQuery(ftCurrency,FieldtypeDefinitions[ftCurrency],testValuesCount);
end;

procedure TTestFieldTypes.TestBCDParamQuery;
begin
  TestXXParamQuery(ftBCD,'NUMERIC(10,4)',testBCDValuesCount);
end;

procedure TTestFieldTypes.TestBytesParamQuery;
begin
  TestXXParamQuery(ftBytes, FieldtypeDefinitions[ftBytes], testBytesValuesCount, [], True);
end;

procedure TTestFieldTypes.TestVarBytesParamQuery;
begin
  TestXXParamQuery(ftVarBytes, FieldtypeDefinitions[ftVarBytes], testVarBytesValuesCount, [], not(SQLServerType in [ssMSSQL, ssSybase]));
end;

procedure TTestFieldTypes.TestBooleanParamQuery;
begin
  TestXXParamQuery(ftBoolean, FieldtypeDefinitions[ftBoolean], testValuesCount);
end;

procedure TTestFieldTypes.TestBlobParamQuery;
begin
  TestXXParamQuery(ftBlob, FieldtypeDefinitions[ftBlob], testBlobValuesCount);
end;

procedure TTestFieldTypes.TestStringParamQuery;
begin
  TestXXParamQuery(ftString,'VARCHAR(10)',testValuesCount);
end;

procedure TTestFieldTypes.TestFixedStringParamQuery;
begin
  TestXXParamQuery(ftFixedChar,'CHAR(10)',testValuesCount);
end;

procedure TTestFieldTypes.TestXXParamQuery(ADataType : TFieldType; ASQLTypeDecl : string;
  ParamValuesCount : integer);
begin
  TestXXParamQuery(ADataType, ASQLTypeDecl, ParamValuesCount, [], False);
end;

procedure TTestFieldTypes.TestXXParamQuery(ADataType : TFieldType; ASQLTypeDecl : string;
  ParamValuesCount : integer; const ParamValues: array of string; Cross : boolean = False);

var i : integer;

begin
  if ASQLTypeDecl = '' then
    Ignore('Fields of the type ' + FieldTypeNames[ADatatype] + ' are not supported by this sqldb-connection type');

  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('create table FPDEV2 (ID INT, FIELD1 '+ASQLTypeDecl+')');

  // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
  TSQLDBConnector(DBConnector).CommitDDL;

  with TSQLDBConnector(DBConnector).Query do
    begin
    sql.clear;
    sql.append('insert into FPDEV2 (ID,FIELD1) values (:id,:field1)');

    // There is no Param.AsFixedChar, so the datatype has to be set manually
    if ADataType=ftFixedChar then
      Params.ParamByName('field1').DataType := ftFixedChar;

    for i := 0 to ParamValuesCount-1 do
      begin
      Params.ParamByName('id').AsInteger := i;
      case ADataType of
        ftSmallInt: Params.ParamByName('field1').AsSmallInt := testSmallIntValues[i];
        ftInteger : Params.ParamByName('field1').AsInteger := testIntValues[i];
        ftLargeInt: Params.ParamByName('field1').AsLargeInt := testLargeIntValues[i];
        ftBoolean : Params.ParamByName('field1').AsBoolean := testBooleanValues[i];
        ftFloat   : Params.ParamByName('field1').AsFloat   := testFloatValues[i];
        ftCurrency: Params.ParamByName('field1').AsCurrency:= testCurrencyValues[i];
        ftBCD     : Params.ParamByName('field1').AsBCD     := testBCDValues[i];
        ftFixedChar,
        ftString  : Params.ParamByName('field1').AsString  := testValues[ADataType,i];
        ftTime    : Params.ParamByName('field1').AsTime    := TimeStringToDateTime(testTimeValues[i]);
        ftDate    : if cross then
                      Params.ParamByName('field1').AsString:= testDateValues[i]
                   else
                      Params.ParamByName('field1').AsDate := StrToDate(testDateValues[i],'yyyy/mm/dd','-');
        ftDateTime: Params.ParamByName('field1').AsDateTime := StrToDateTime(testValues[ADataType,i], DBConnector.FormatSettings);
        ftFMTBcd  : Params.ParamByName('field1').AsFMTBCD := StrToBCD(ParamValues[i], DBConnector.FormatSettings);
        ftBlob    : Params.ParamByName('field1').AsBlob := BytesOf(testBlobValues[i]);
        ftBytes   : if cross then
                      Params.ParamByName('field1').Value := StringToByteArray(testBytesValues[i])
                    else
                      Params.ParamByName('field1').AsBytes := BytesOf(testBytesValues[i]);
        ftVarBytes: if cross then
                      Params.ParamByName('field1').AsString := testBytesValues[i]
                    else
                      Params.ParamByName('field1').AsBytes := BytesOf(testBytesValues[i]);
      else
        AssertTrue('no test for paramtype available',False);
      end;
      ExecSQL;
      end;
    // test NULL parameter value
    Params.ParamByName('id').AsInteger := ParamValuesCount;
    Params.ParamByName('field1').Clear;
    ExecSQL;

    TSQLDBConnector(DBConnector).Transaction.CommitRetaining;

    sql.clear;
    sql.append('select * from FPDEV2 order by ID');
    open;

    for i := 0 to ParamValuesCount-1 do
      begin
      AssertEquals(i,FieldByName('ID').AsInteger);
      case ADataType of
        ftSmallInt : AssertEquals(testSmallIntValues[i],FieldByName('FIELD1').AsInteger);
        ftInteger  : AssertEquals(testIntValues[i],FieldByName('FIELD1').AsInteger);
        ftLargeInt : AssertEquals(testLargeIntValues[i],FieldByName('FIELD1').AsLargeInt);
        ftBoolean  : AssertEquals(testBooleanValues[i],FieldByName('FIELD1').AsBoolean);
        ftFloat    : AssertEquals(testFloatValues[i],FieldByName('FIELD1').AsFloat);
        ftCurrency : AssertEquals(testCurrencyValues[i],FieldByName('FIELD1').AsFloat,0); // TCurrencyField uses double data type (not currency) to store values!
        ftBCD      : AssertEquals(testBCDValues[i],FieldByName('FIELD1').AsCurrency);
        ftFixedChar : AssertEquals(PadRight(testStringValues[i],10),FieldByName('FIELD1').AsString);
        ftString   : AssertEquals(testStringValues[i],FieldByName('FIELD1').AsString);
        ftTime     : AssertEquals(testTimeValues[i],DateTimeToTimeString(FieldByName('FIELD1').AsDateTime));
        ftDate     : AssertEquals(testDateValues[i],DateTimeToStr(FieldByName('FIELD1').AsDateTime, DBConnector.FormatSettings));
        ftDateTime : AssertEquals(testValues[ADataType,i], DateTimeToStr(FieldByName('FIELD1').AsDateTime, DBConnector.FormatSettings));
        ftFMTBcd   : AssertEquals(ParamValues[i], BCDToStr(FieldByName('FIELD1').AsBCD, DBConnector.FormatSettings));
        ftBlob     : AssertEquals(testBlobValues[i], FieldByName('FIELD1').AsString);
        ftVarBytes,
        ftBytes    : AssertEquals(testBytesValues[i], shortstring(FieldByName('FIELD1').AsString));
      else
        AssertTrue('no test for paramtype available',False);
      end;
      AssertEquals('IsNull', False, FieldByName('FIELD1').IsNull);
      Next;
      end;
    AssertTrue('Expected IsNull', FieldByName('FIELD1').IsNull);
    AssertTrue('Expected Null Variant', VarIsNull(FieldByName('FIELD1').AsVariant));
    close;
    end;
  TSQLDBConnector(DBConnector).Transaction.CommitRetaining;
end;

procedure TTestFieldTypes.TestSetBlobAsParam(asWhat: integer);
var i: integer;
begin
  CreateTableWithFieldType(ftBlob,FieldtypeDefinitions[ftBlob]);
  TestFieldDeclaration(ftBlob,0);

  with TSQLDBConnector(DBConnector).Query do
    begin
    SQL.Text := 'insert into FPDEV2 (FT) values (:BlobParam)';
    for i:=0 to testBlobValuesCount - 1 do
      begin
      case asWhat of
        0: Params.ParamByName('blobParam').AsMemo   := TestBlobValues[i];
        1: Params.ParamByName('blobParam').AsBlob   := BytesOf(TestBlobValues[i]);
        2: Params.ParamByName('blobParam').AsString := TestBlobValues[i];
      end;
      ExecSQL;
      end;
    Params.ParamByName('blobParam').Clear;
    ExecSQL;

    SQL.Text := 'select FT from FPDEV2';
    Open;
    for i:=0 to testBlobValuesCount - 1 do
      begin
      AssertEquals(TestBlobValues[i], Fields[0].AsString);
      Next;
      end;
    AssertTrue(Fields[0].IsNull);

    Close;
    end;
end;

procedure TTestFieldTypes.TestSetBlobAsMemoParam;
begin
  // Firebird/Interbase ODBC driver : if parameter of ValueType=SQL_C_CHAR is bound to BLOB column
  // driver interprets character data as hexadecimal string and performs conversion ('FF10'=#255#16)
  TestSetBlobAsParam(0);
end;

procedure TTestFieldTypes.TestSetBlobAsBlobParam;
begin
  TestSetBlobAsParam(1);
end;

procedure TTestFieldTypes.TestSetBlobAsStringParam;
begin
  TestSetBlobAsParam(2);
end;


procedure TTestFieldTypes.Test11Params;
var i : integer;
begin
  with TSQLDBConnector(dbconnector) do
    begin
    Connection.ExecuteDirect('create table FPDEV2 (id1 int, id2 int, id3 int, id4 int,id5 int,id6 int,id7 int,id8 int, id9 int, id10 int, id11 int)');
    // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
    TSQLDBConnector(DBConnector).CommitDDL;

    Query.sql.Text := 'insert into FPDEV2 values(:id1,:id2,:id3,:id4,:id5,:id6,:id7,:id8,:id9,:id10,:id11)';
    for i := 0 to 10 do
      query.params[i].asinteger := 1;
    query.ExecSQL;
    query.sql.text := 'select * from FPDEV2';
    query.open;
    for i := 0 to 10 do
      AssertEquals(1,query.fields[i].asinteger);
    query.close;
    end;
end;

procedure TTestFieldTypes.TestCircularParams;
begin
  with TSQLDBConnector(dbconnector) do
    begin
    Connection.ExecuteDirect('create table FPDEV2 (id1 int, id2 int,vchar varchar(10))');
    // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
    TSQLDBConnector(DBConnector).CommitDDL;

    Query.sql.Text := 'insert into FPDEV2 values(:id1,:id2,:vchar)';
    query.params[0].asinteger := 1;
    query.params[1].asinteger := 1;
    query.params[2].asstring := '$1 :id2 $';
    query.ExecSQL;
    query.sql.text := 'select * from FPDEV2';
    query.open;
    AssertEquals(1,query.fields[0].asinteger);
    AssertEquals(1,query.fields[1].asinteger);
    AssertEquals('$1 :id2 $',query.fields[2].AsString);
    query.close;
    end;
end;

procedure TTestFieldTypes.TestNonNullableParams;
var ASQLQuery : TSQLQuery;
    Passed: Boolean;
begin
  // Check for an exception when a null value is stored into a non-nullable
  // field using a parameter
  // There was a bug in IBConnection so that in this case the last used value
  // for the parameter was used.

  // To make sure that any changes are cancelled in the case the test fails
  TSQLDBConnector(DBConnector).GetNDataset(True,1);

  ASQLQuery := TSQLDBConnector(DBConnector).Query;
  ASQLQuery.SQL.Text := 'update fpdev set ID=:ID1 where id = :ID2';
  ASQLQuery.Params[0].Clear;
  ASQLQuery.Params[1].AsInteger := 1;
  AssertTrue(ASQLQuery.Params[0].IsNull);
  Passed := False;
  try
    ASQLQuery.ExecSQL;
  except
    on E: Exception do
      if E.ClassType.InheritsFrom(EDatabaseError) then
        Passed := True;
  end;
  AssertTrue(Passed);
end;

procedure TTestFieldTypes.TestParametersAndDates;
// See bug 7205
var ADateStr : String;
begin
  if not(SQLServerType in [ssPostgreSQL, ssOracle]) then
    Ignore('This test does not apply to this sqldb connection type, since it doesn''t use semicolons for casts');

  with TSQLDBConnector(DBConnector).Query do
    begin
    SQL.Clear;
    sql.add('select now()::date as current_date where 1=1');
    open;
    first;
    ADateStr:=fields[0].asstring; // return the correct date
    // writeln(fields[0].asstring);
    close;

    sql.clear;
    sql.add('select now()::date as current_date where cast(1 as integer) = :PARAM1');
    params.parambyname('PARAM1').asinteger:= 1;
    open;
    first;
    AssertEquals(ADateStr,fields[0].asstring); // return invalid date
    // writeln(fields[0].asstring);
    close;
    end
end;


procedure TTestFieldTypes.TestInsertReturningQuery;
begin
  if not(SQLServerType in [ssFirebird, ssOracle, ssPostgreSQL]) then Ignore(STestNotApplicable);
  with TSQLDBConnector(DBConnector) do
    begin
    // This only works with databases that supports 'insert into .. returning'
    // for example: PostgreSQL, Oracle, Firebird version 2.0 and up
    CreateTableWithFieldType(ftInteger,'int');
    Query.SQL.Text:='insert into FPDEV2 values(154) returning FT';
    Query.Open;
    AssertTrue(CompareText('FT',Query.Fields[0].FieldName)=0);
    AssertEquals(154,Query.fields[0].AsInteger);
    Query.Close;
    end;
end;

procedure TTestFieldTypes.TestOpenStoredProc;
begin
  with TSQLDBConnector(DBConnector) do
  begin
    case SQLServerType of
      ssMySQL:
        begin
        Connection.ExecuteDirect('create procedure FPDEV_PROC() select 1 union select 2;');
        Query.SQL.Text:='call FPDEV_PROC';
        end;
      ssFirebird, ssInterbase:
        begin
        Connection.ExecuteDirect('create procedure FPDEV_PROC returns (r integer) as begin r=1; end');
        Query.SQL.Text:='execute procedure FPDEV_PROC';
        end;
      ssMSSQL, ssSybase:
        begin
        Connection.ExecuteDirect('create procedure FPDEV_PROC as select 1 union select 2;');
        Query.SQL.Text:='execute FPDEV_PROC';
        end;
      else
        begin
        Ignore('This test does not apply to this sqldb connection type, since it does not support selectable stored procedures.');
        Exit;
        end;
    end;
    Transaction.CommitRetaining;

    try
      Query.Open;
      AssertEquals(1, Query.Fields[0].AsInteger);
      Query.Next;
      if not(SQLServerType in [ssFirebird, ssInterbase]) then
      begin
        AssertFalse('Eof after 1st row', Query.Eof);
        AssertEquals(2, Query.Fields[0].AsInteger);
        Query.Next;
      end;
      AssertTrue('No Eof after last row', Query.Eof);
      Query.Close;
    finally
      Connection.ExecuteDirect('drop procedure FPDEV_PROC');
      Transaction.CommitRetaining;
    end;
  end;
end;

procedure TTestFieldTypes.TestOpenSpecialStatements;
const CTE_SELECT = 'WITH a AS (SELECT * FROM FPDEV) SELECT * FROM a';
type TTestStatements = array of string;
var statements: TTestStatements;
    s: string;
begin
  // tests non-select statements (other than "SELECT ..."), which return result-set
  // at least one row must be returned
  with TSQLDBConnector(DBConnector) do
  begin
    case SQLServerType of
      ssSQLite:
        statements := TTestStatements.Create('pragma table_info(FPDEV)');
      ssFirebird:
        statements := TTestStatements.Create(
          CTE_SELECT (*FB 2.1*),
          'EXECUTE BLOCK RETURNS (U VARCHAR(255)) AS BEGIN SELECT rdb$get_context(''SYSTEM'',''CURRENT_USER'') FROM rdb$database INTO U; SUSPEND; END' (*FB 2.0*)
        );
      ssPostgreSQL:
        statements := TTestStatements.Create(CTE_SELECT, 'EXPLAIN '+CTE_SELECT);
      ssMSSQL:
        statements := TTestStatements.Create(CTE_SELECT  (*MS SQL 2005*));
      ssMySQL:
        statements := TTestStatements.Create(
          'check table FPDEV',  // bug 14519
          'show tables from '+Connection.DatabaseName  // bug 16842
        )
      else
        Ignore(STestNotApplicable);
    end;

    for s in statements do
    begin
      Query.SQL.Text := s;
      Query.Open;
      AssertTrue(Query.FieldCount>0);
      AssertFalse('Eof after open', Query.Eof);
      Query.Next;
      Query.Close;
    end;
  end;
end;


procedure TTestFieldTypes.TestErrorOnEmptyStatement;
var PassException : boolean;
begin
  PassException:=False;
  with TSQLDBConnector(DBConnector).Query do
    begin
    sql.Text := '';
    try
      Open;
    except
      on E:EDatabaseError do
        if Pos(SErrNoStatement,E.Message) > 0 then
          PassException := True;
    end;
    AssertTrue(PassException);
    end;
end;

procedure TTestFieldTypes.TestExceptOnsecClose;

var passed : boolean;

begin
  with TSQLDBConnector(DBConnector).Query do
    begin
    SQL.Clear;
    SQL.Add('select * from FPDEV');

    Open;
    close;

    SQL.Clear;
    SQL.Add('select blaise from FPDEV');
    passed := false;
    try
      open;
    except
      on E: Exception do
      begin
        passed := (E.ClassType.InheritsFrom(EDatabaseError))
      end;
      end;
    AssertTrue(passed);

    Close;
    end;
end;


procedure TTestFieldTypes.TestServerFilter;
begin
  // Tests SQLParser and ServerFilter
  with TSQLDBConnector(DBConnector).Query do
  begin
    ServerFilter:='ID=21';
    ServerFiltered:=true;

    // tests parsing SELECT without WHERE
    SQL.Text:='select * from FPDEV';
    Open;
    CheckTrue(CanModify, SQL.Text);
    CheckEquals(1, RecordCount);
    Close;

    SQL.Text:='select *'#13'from FPDEV'#13'order by 1';
    Open;
    CheckTrue(CanModify, SQL.Text);
    CheckEquals(1, RecordCount);
    Close;

    // tests parsing SELECT with simple WHERE
    SQL.Text:='select *'#9'from FPDEV'#9'where NAME<>''''';
    Open;
    CheckTrue(CanModify, SQL.Text);
    CheckEquals(1, RecordCount);
    Close;

    // tests parsing SELECT with simple WHERE followed by ORDER BY
    SQL.Text:='select *'#10'from FPDEV'#10'where NAME>'''' order by 1';
    Open;
    CheckTrue(CanModify, SQL.Text);
    CheckEquals(1, RecordCount);
    Close;

    // tests parsing of WHERE ... LIMIT
    case SQLServerType of
      ssFirebird, ssInterbase:
        SQL.Text:='select first 1 NAME from FPDEV where NAME=''TestName21''';
      ssMSSQL, ssSybase:
        SQL.Text:='select top 1 NAME from FPDEV where NAME=''TestName21''';
      else
        SQL.Text:='select NAME from FPDEV where NAME=''TestName21'' limit 1';
    end;
    Open;
    CheckTrue(CanModify, SQL.Text);
    Close;

    // tests parsing SELECT with table alias and embedded comments (MySQL requires space after -- )
    SQL.Text:='/**/select * from/**/FPDEV as fp-- comment'#13'where(NAME>''TestName20'')/**/order by 1';
    Open;
    CheckTrue(CanModify, 'CanModify: '+SQL.Text);
    CheckTrue(ServerIndexDefs.Count >= 1, 'ServerIndexDefs: '+SQL.Text); // is TableName extracted correctly? (MS SQL Server can automatically creates index)
    Close;

    // tests parsing SELECT with quoted identifiers (MySQL requires sql-mode=ANSI_QUOTES)
    // The folding of unquoted names to lower case in PostgreSQL is incompatible with the SQL standard
    SQL.Text:=IdentifierCase('SELECT"ID"FROM"FPDEV"ORDER BY"ID"');
    Open;
    CheckTrue(CanModify, SQL.Text);
    Close;

    // tests change of ServerFilter, while DataSet is opened and not all records were fetched
    PacketRecords:=2;
    ServerFilter:='ID>=1';
    Open;
    CheckEquals(1, FieldByName('ID').AsInteger);
    ServerFilter:='ID>=21';
    CheckEquals(21, FieldByName('ID').AsInteger);
    Close;
  end;
end;

procedure TTestFieldTypes.TestRowsAffected;
var Query2: TSQLQuery;
begin
  with TSQLDBConnector(DBConnector) do
    begin
    Query2 := GetNDataset(0) as TSQLQuery;

    CheckEquals(-1, Query.RowsAffected, 'Inactive dataset');
    Connection.ExecuteDirect('create table FPDEV2 (' +
                              '  ID INT NOT NULL,  ' +
                              '  NAME VARCHAR(250),' +
                              '  PRIMARY KEY (ID) )');
    // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
    TSQLDBConnector(DBConnector).CommitDDL;

    Query.SQL.Text := 'insert into FPDEV2(ID,NAME) values (1,''test1'')';
    Query.ExecSQL;
    AssertEquals(1, Query.RowsAffected);
    Query.SQL.Text := 'insert into FPDEV2(ID,NAME) values (2,''test2'')';
    Query.ExecSQL;
    AssertEquals(1, Query.RowsAffected);

    Query.SQL.Text := 'update FPDEV2 set NAME=''NewTest''';
    Query.ExecSQL;

    AssertEquals(-1, Query2.RowsAffected);
    Query2.SQL.Text := 'insert into FPDEV2 values(3,''test3'')';
    Query2.ExecSQL;
    AssertEquals(1, Query2.RowsAffected);
    // tests, that RowsAffected is specific per query, not per connection
    //  i.e. that it doesn't return only RowsAffected of last query executed over connection
    AssertEquals(2, Query.RowsAffected);

    Query.SQL.Text := 'select * from FPDEV2';
    Query.Open;
    AssertTrue(Query.RowsAffected<>0); // It should return -1 or the number of selected rows.
    Query.Close;
    AssertTrue(Query.RowsAffected<>0); // It should return -1 or the same as the last time it was called.

    Query.SQL.Text := 'delete from FPDEV2 where ID>0'; // sqlite doesn't count the RowsAffected if there is no where-clause
    Query.ExecSQL;
    AssertEquals(3, Query.RowsAffected);
    Query.SQL.Text := 'delete from FPDEV2';
    Query.ExecSQL;
    AssertEquals(0, Query.RowsAffected);
    end;
end;

procedure TTestFieldTypes.TestLocateNull;
var DS: TCustomBufDataset;
begin
  ds := TSQLDBConnector(DBConnector).GetNDataset(true,5) as TCustomBufDataset;
  with ds do
    begin
    open;
    edit;
    fieldbyname('name').Clear;
    post;
    next;
    AssertFalse(Locate('name',VarArrayOf(['TestName1']),[]));
    AssertTrue(Locate('name',VarArrayOf([Null]),[]));
    AssertEquals(1,fieldbyname('ID').AsInteger);
    end;
end;

procedure TTestFieldTypes.TestLocateOnMoreRecords;
var DS: TCustomBufDataset;
begin
  with TSQLDBConnector(DBConnector) do
    begin
    ds := GetNDataset(true,30) as TCustomBufDataset;
    with query do
      begin
      SQL.Text:='update FPDEV set NAME = null where ID<11;';
      ExecSQL;
      SQL.Text:='update FPDEV set NAME = null where (ID>11) and (ID<23);';
      ExecSQL;
    end;
    with ds do
      begin
      Open;
      // Must be exactly 11 to trigger bug/test
      AssertTrue(Locate('name',VarArrayOf(['TestName11']),[]));
      AssertEquals(11,fieldbyname('ID').AsInteger);

      // Must be exactly 23 to trigger bug/test
      AssertTrue(Locate('name',VarArrayOf(['TestName23']),[]));
      AssertEquals(23,fieldbyname('ID').AsInteger);
      end;
    end;

end;

procedure TTestFieldTypes.TestRefresh;
var ADataset: TDataset;
    i: integer;
    AFldID, AFldName: TField;
begin
  ADataset := TSQLDBConnector(DBConnector).GetNDataset(true,5);

  Adataset.Open;
  AFldId:=Adataset.Fields[0];
  AFldName:=Adataset.Fields[1];
  for i := 1 to 5 do
    begin
    AssertEquals(i, AFldID.AsInteger);
    AssertEquals('TestName'+inttostr(i), AFldName.AsString);
    ADataset.Next;
    end;

  ADataset.Next;
  AssertTrue(ADataset.EOF);
  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('update FPDEV set NAME=''test'' where ID=2');

  ADataset.Refresh;

  ADataset.First;
  for i := 1 to 5 do
    begin
    AssertEquals('ID', i, AFldID.AsInteger);
    if i = 2 then
      AssertEquals('NAME', 'test', AFldName.AsString)
    else
      AssertEquals('NAME', 'TestName'+inttostr(i), AFldName.AsString);
    ADataset.Next;
    end;
  ADataset.Next;
  AssertTrue(ADataset.EOF);
end;


procedure TTestFieldTypes.TestTableNames;
var TableList : TStringList;

begin
  TableList := TStringList.Create;
  try
    TSQLDBConnector(DBConnector).Connection.GetTableNames(TableList);
    AssertTrue(TableList.IndexOf('fpdev')<>-1);
  finally
    TableList.Free;
  end;
end;

procedure TTestFieldTypes.TestGetTables;
var TableNames : TStringList;
begin
  with TSQLDBConnector(DBConnector) do
  begin
    TableNames := TStringList.Create;
    try
      Connection.GetTableNames(TableNames);
      AssertTrue(TableNames.Count>0);
      AssertTrue(TableNames.IndexOf('FPDEV')>-1);
      AssertTrue(TableNames.IndexOf('FPDEV_FIELD')>-1);
    finally
      TableNames.Free;
    end;
  end;
end;

procedure TTestFieldTypes.TestFieldNames;
var FieldList : TStringList;
begin
  FieldList := TStringList.Create;
  try
    TSQLDBConnector(DBConnector).Connection.GetFieldNames('fpdev',FieldList);
    AssertTrue(FieldList.IndexOf('id')<>-1);
  finally
    FieldList.Free;
  end;
end;

procedure TTestFieldTypes.TestGetFieldNames;
var FieldNames : TStringList;
begin
  with TSQLDBConnector(DBConnector) do
  begin
    FieldNames := TStringList.Create;
    try
      if SQLConnType in MySQLConnTypes then
        Connection.GetFieldNames('FPDEV',FieldNames)
      else
        Connection.GetFieldNames('fpDEv',FieldNames);
      AssertEquals(2,FieldNames.Count);
      AssertEquals('ID',UpperCase(FieldNames[0]));
      AssertEquals('NAME',UpperCase(FieldNames[1]));
    finally
      FieldNames.Free;
    end;
  end;
end;


procedure TTestFieldTypes.TestUpdateIndexDefs;
var ds : TSQLQuery;
begin
  // Firebird/Interbase ODBC driver returns for :
  //   SQLPrimaryKeys (PK_NAME): RDB$RELATION_CONSTRAINTS(RDB$CONSTRAINT_NAME)
  //   SQLStatistics (INDEX_NAME): RDB$INDICES(RDB$INDEX_NAME)
  // these two names can differ (when PRIMARY KEY is created without giving constraint name),
  // therefore two indexes may be created instead of one (see TODBCConnection.UpdateIndexDefs)
  ds := DBConnector.GetNDataset(1) as TSQLQuery;
  ds.Prepare;
  ds.ServerIndexDefs.Update;
  AssertEquals(1,ds.ServerIndexDefs.count);
  AssertTrue(CompareText('ID',ds.ServerIndexDefs[0].Fields)=0);
  Asserttrue(ds.ServerIndexDefs[0].Options=[ixPrimary,ixUnique]);
  ds.ServerIndexDefs.Update;
  AssertEquals(1,ds.ServerIndexDefs.count);
  AssertTrue(CompareText('ID',ds.ServerIndexDefs[0].Fields)=0);
  Asserttrue(ds.ServerIndexDefs[0].Options=[ixPrimary,ixUnique]);
end;

procedure TTestFieldTypes.TestMultipleFieldPKIndexDefs;
var ds : TSQLQuery;
begin
  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('create table FPDEV2 (' +
                              '  ID1 INT NOT NULL,     ' +
                              '  ID2 INT NOT NULL,     ' +
                              '  NAME VARCHAR(50),     ' +
                              '  PRIMARY KEY (ID1, ID2)' +
                              ')                       ');
  // Firebird/Interbase need a commit after a DDL statement. Not necessary for the other connections
  TSQLDBConnector(DBConnector).CommitDDL;

  ds := TSQLDBConnector(DBConnector).Query;
  ds.sql.Text:='select * from FPDEV2';
  ds.Prepare;
  ds.ServerIndexDefs.Update;
  AssertEquals(1,ds.ServerIndexDefs.count);
  AssertTrue(SameText('ID1;ID2',ds.ServerIndexDefs[0].Fields));
  Asserttrue(ds.ServerIndexDefs[0].Options=[ixPrimary,ixUnique]);
end;

procedure TTestFieldTypes.TestGetIndexDefs;

var ds : TSQLQuery;
    inddefs : TIndexDefs;

begin
  ds := DBConnector.GetNDataset(1) as TSQLQuery;
  ds.Open;
  AssertEquals('ServerIndexDefs.Count', 1, ds.ServerIndexDefs.Count);
  inddefs := HackedDataset(ds).GetIndexDefs(ds.ServerIndexDefs,[ixPrimary]);
  AssertEquals('ixPrimary', 1, inddefs.count);
  AssertTrue(CompareText('ID',inddefs[0].Fields)=0);
  AssertTrue(inddefs[0].Options=[ixPrimary,ixUnique]);
  inddefs.Free;

  inddefs := HackedDataset(ds).GetIndexDefs(ds.ServerIndexDefs,[ixPrimary,ixUnique]);
  AssertEquals('ixPrimary,ixUnique', 1, inddefs.count);
  AssertTrue(CompareText('ID',inddefs[0].Fields)=0);
  AssertTrue(inddefs[0].Options=[ixPrimary,ixUnique]);
  inddefs.Free;

  inddefs := HackedDataset(ds).GetIndexDefs(ds.ServerIndexDefs,[ixDescending]);
  AssertEquals('ixDescending', 0, inddefs.count);
  inddefs.Free;
end;


procedure TTestFieldTypes.TestEmptyUpdateQuery;
begin
  TSQLDBConnector(DBConnector).Connection.ExecuteDirect('update FPDEV set NAME=''nothing'' where (1=0)');
end;

procedure TTestFieldTypes.TestTemporaryTable;
begin
  // Tests rev.6481: "Do not use a new connection for every statement that is executed";
  if SQLServerType in [ssMSSQL, ssSybase] then Ignore('This test does not apply to this sqldb connection type, since it doesn''t support temporary tables');

  with TSQLDBConnector(DBConnector).Query do
    begin
    SQL.Clear;
    if SQLServerType in [ssFirebird, ssInterbase, ssOracle] then
      // Global temporary table: introduced in Firebird 2.1
      // has persistent metadata; data is per transaction (default) or per connection
      SQL.Add('CREATE GLOBAL TEMPORARY TABLE FPDEV_TEMP (id int)')
    else
      SQL.Add('CREATE TEMPORARY TABLE FPDEV_TEMP (id int)');
    ExecSQL;
    try
      // Firebird/Interbase needs a commit after DDL:
      TSQLDBConnector(DBConnector).CommitDDL;

      SQL.Text := 'INSERT INTO FPDEV_TEMP(id) values (5)';
      ExecSQL;
      SQL.Text := 'SELECT * FROM FPDEV_TEMP';
      Open;
      AssertEquals(5, Fields[0].AsInteger);
      Close;
    finally
      // For Firebird/Interbase, we need to explicitly delete the table as well (it's active within the transaction)
      if SQLServerType in [ssFirebird, ssInterbase, ssOracle] then
        begin
        SQL.Text := 'DROP TABLE FPDEV_TEMP';
        ExecSQL;
        TSQLDBConnector(DBConnector).CommitDDL;
        end;
    end;
    end;
end;

procedure TTestFieldTypes.TestQueryAfterReconnect;
var DS: TDataset;
begin
  ds := DBConnector.GetNDataset(true,5);
  with ds do
    begin
    open;
    close;
    TSQLDBConnector(DBConnector).Connection.Close;
    TSQLDBConnector(DBConnector).Connection.Open;
    open;
    close;
    end;
end;


procedure TTestFieldTypes.TestStringsReplace;
begin
  AssertEquals('dit is een string',StringsReplace('dit was een string',['was'],['is'],[]));
  AssertEquals('dit is een string was een string',StringsReplace('dit was een string was een string',['was'],['is'],[]));
  AssertEquals('dit is een string is een string',StringsReplace('dit was een string was een string',['was'],['is'],[rfReplaceAll]));

  AssertEquals('dit is een char is een char',StringsReplace('dit was een string was een string',['was','string'],['is','char'],[rfReplaceAll]));
  AssertEquals('dit is een string was een string',StringsReplace('dit was een string was een string',['string','was'],['char','is'],[]));

  AssertEquals('dit is een char is een strin',StringsReplace('dit was een string was een strin',['string','was'],['char','is'],[rfReplaceAll]));

  AssertEquals('dit Was een char is een char',StringsReplace('dit Was een string was een string',['was','string'],['is','char'],[rfReplaceAll]));
  AssertEquals('dit wAs een char is een char',StringsReplace('dit wAs een string was een string',['was','string'],['is','char'],[rfReplaceAll]));
  AssertEquals('dit is een char is een char',StringsReplace('dit Was een sTring was een string',['was','string'],['is','char'],[rfReplaceAll,rfIgnoreCase]));
  AssertEquals('dit is een char is een char',StringsReplace('dit wAs een STRING was een string',['was','string'],['is','char'],[rfReplaceAll,rfIgnoreCase]));

  AssertEquals('dit was een si was een sa',StringsReplace('dit was een string was een straat',['straat','string'],['sa','si'],[rfReplaceAll]));
  AssertEquals('dit was een si was een sa',StringsReplace('dit was een string was een straat',['string','straat'],['si','sa'],[rfReplaceAll]));

  AssertEquals('dit was een sing was een saat',StringsReplace('dit was een string was een straat',['str','string'],['s','si'],[rfReplaceAll]));
  AssertEquals('dit was een si was een saat',StringsReplace('dit was een string was een straat',['string','str'],['si','s'],[rfReplaceAll]));

  AssertEquals('dit was een string was een string',StringsReplace('dit was een string was een string',[''],['is'],[rfReplaceAll]));
  AssertEquals('dit  een string  een string',StringsReplace('dit was een string was een string',['was'],[''],[rfReplaceAll]));
end;


procedure TTestFieldTypes.SetUp;
begin
  InitialiseDBConnector;
  DBConnector.StartTest(TestName);
end;

procedure TTestFieldTypes.TearDown;
begin
  DBConnector.StopTest(TestName);
  if assigned(DBConnector) then
    TSQLDBConnector(DBConnector).Transaction.Rollback;
  FreeDBConnector;
end;

procedure TTestFieldTypes.RunTest;
begin
//  if (SQLConnType in TSQLConnType) then
    inherited RunTest;
end;

Procedure TTestFieldTypes.TestAlwaysUseBigint;

var
  I : byte;

begin
  If SQLConnType<>sqlite3 then
    Ignore('Test only for SQLite');
  TSQLDBConnector(DBConnector).Connection.Params.Values['AlwaysUseBigint']:='1';

  CreateTableWithFieldType(ftInteger,'INT');
  TestFieldDeclaration(ftLargeInt,8);

  for i := 0 to testIntValuesCount-1 do
    TSQLDBConnector(DBConnector).Connection.ExecuteDirect('insert into FPDEV2 (FT) values (' + inttostr(testIntValues[i]) + ')');

  with TSQLDBConnector(DBConnector).Query do
    begin
    Open;
    for i := 0 to testIntValuesCount-1 do
      begin
      AssertEquals(testIntValues[i],fields[0].AsLargeInt);
      Next;
      end;
    close;
    end;
    
end;

initialization
  // Only test if using sqldb
  if uppercase(dbconnectorname)='SQL' then
    RegisterTest(TTestFieldTypes);
end.
