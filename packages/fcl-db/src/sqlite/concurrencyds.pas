program concurrencyds;

{$Mode ObjFpc}
{$H+}
{$define DEBUGHEAP}

<<<<<<< HEAD
<<<<<<< HEAD
=======
//To test the sqlite3 version replace sqliteds by sqlite3ds
//  and TSqliteDataset by TSqlite3Dataset

>>>>>>> graemeg/fixes_2_2
=======
//To test the sqlite3 version replace sqliteds by sqlite3ds
//  and TSqliteDataset by TSqlite3Dataset

>>>>>>> origin/fixes_2_2
uses
{$ifdef DEBUGHEAP}
  Heaptrc,
{$endif}
{$ifdef Linux}
  cmem,
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
  sysutils,sqlite3ds, inifiles;
=======
  sysutils,sqliteds, inifiles;
>>>>>>> graemeg/fixes_2_2
=======
  sysutils,sqliteds, inifiles;
>>>>>>> origin/fixes_2_2
  
const
  SQLITEDS_TESTS_INI_FILE = 'sqlitedstests.ini';
  DEFAULT_TABLENAME = 'tabletest';
  DEFAULT_FILENAME = 'test.db';

  FieldNames: array [0..10] of String =
  (
  'Integer',
  'String',
  'Boolean',
  'Float',
  'Word',
  'Date',
  'DateTime',
  'Time',
  'LargeInt',
  'AutoInc',
  'Currency'  
  );

var
<<<<<<< HEAD
<<<<<<< HEAD
  dsArray: array [0..10] of TSqlite3Dataset;
=======
  dsArray: array [0..10] of TSqliteDataset;
>>>>>>> graemeg/fixes_2_2
=======
  dsArray: array [0..10] of TSqliteDataset;
>>>>>>> origin/fixes_2_2
  ini:TIniFile;
  i: Integer;

begin
  {$ifdef DEBUGHEAP}
  SetHeapTraceOutput(ExtractFileName(ParamStr(0))+'.heap.log');
  {$endif}
  ini:=TIniFile.Create(SQLITEDS_TESTS_INI_FILE);
  for i:= 0 to 10 do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    dsArray[i] := TSqlite3Dataset.Create(nil);
=======
    dsArray[i] := TSqliteDataset.Create(nil);
>>>>>>> graemeg/fixes_2_2
=======
    dsArray[i] := TSqliteDataset.Create(nil);
>>>>>>> origin/fixes_2_2
    with dsArray[i] do
    begin
      FileName:=ini.ReadString('testinfo','filename',DEFAULT_FILENAME);
      TableName:=ini.ReadString('testinfo','tablename',DEFAULT_TABLENAME);
      //Each dataset will retrieve only one field of the same table
      Sql:='Select '+FieldNames[i]+ ' from '+ TableName;
      Open;
      WriteLn('Value of Field ',FieldNames[i],' : ',FieldByName(FieldNames[i]).AsString);
    end;
  end;
  ini.Destroy;
  for i:= 0 to 10 do
    dsArray[i].Destroy;
end.
