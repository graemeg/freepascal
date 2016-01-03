program testds;

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
  crt,sysutils,db,sqlite3ds,IniFiles;

const
  SQLITEDS_TESTS_INI_FILE = 'sqlitedstests.ini';
  DEFAULT_TABLENAME = 'tabletest';
  DEFAULT_FILENAME = 'test.db';

var
  dsTest: TSqlite3Dataset;
=======
  crt,sysutils,db,sqliteds,IniFiles;

const
  SQLITEDS_TESTS_INI_FILE = 'sqlitedstests.ini';
  DEFAULT_TABLENAME = 'tabletest';
  DEFAULT_FILENAME = 'test.db';

var
  dsTest:TSqliteDataset;
>>>>>>> graemeg/fixes_2_2
=======
  crt,sysutils,db,sqliteds,IniFiles;

const
  SQLITEDS_TESTS_INI_FILE = 'sqlitedstests.ini';
  DEFAULT_TABLENAME = 'tabletest';
  DEFAULT_FILENAME = 'test.db';

var
  dsTest:TSqliteDataset;
>>>>>>> origin/fixes_2_2
  ini: TIniFile;

begin
  {$ifdef DEBUGHEAP}
  SetHeapTraceOutput(ExtractFileName(ParamStr(0))+'.heap.log');
  {$endif}
<<<<<<< HEAD
<<<<<<< HEAD
  dsTest:=TSqlite3Dataset.Create(nil);
=======
  dsTest:=TSqliteDataset.Create(nil);
>>>>>>> graemeg/fixes_2_2
=======
  dsTest:=TSqliteDataset.Create(nil);
>>>>>>> origin/fixes_2_2
  with dsTest do
  begin
    //Load Database properties from a inifile
    ini:=TIniFile.Create(SQLITEDS_TESTS_INI_FILE);
    FileName:=ini.ReadString('testinfo','filename',DEFAULT_FILENAME);
    TableName:=ini.ReadString('testinfo','tablename',DEFAULT_TABLENAME);
    ini.Destroy;
    //Calling Open with an empty SQL, is the same of setting SQL to 'SELECT * from [TableName]';
    Open;
    writeln('Push a key to test -Edit-');
    Readkey;
    ClrScr;
    WriteLn('Old Integer:',FieldbyName('Integer').AsInteger);
    WriteLn('Old String:',FieldbyName('String').AsString);
    Edit;
    FieldbyName('Integer').AsInteger:=12345;
    FieldbyName('String').AsString:='Record Edited in TestDs.pas';
    Post;
    WriteLn('New Integer:',FieldbyName('Integer').AsInteger);
    WriteLn('New String:',FieldbyName('String').AsString);

    WriteLn('Push a key to test -Append-');
    ReadKey;
    ClrScr;
    Append;
    FieldbyName('Integer').AsInteger:=22222;
    FieldbyName('String').AsString:='This will be deleted';
    Post;
    WriteLn('First Record Appended - Integer:',FieldbyName('Integer').AsInteger);
    WriteLn('First Record Appended - String:',FieldbyName('String').AsString);
    Append;
    FieldbyName('Integer').AsInteger:=3333;
    FieldbyName('String').AsString:='This will stay';
    Post;
    WriteLn('Second Record Appended - Integer:',FieldbyName('Integer').AsInteger);
    WriteLn('Second Record Appended - String:',FieldbyName('String').AsString);

    WriteLn('Push a key to test -Delete-');
    ReadKey;
    ClrScr;
    Prior;
    WriteLn('Current record:');
    WriteLn('Integer: ',FieldbyName('Integer').AsInteger);
    WriteLn('String: ',FieldbyName('String').AsString);
    if FieldbyName('Integer').AsInteger = 22222 then
    begin
      Writeln('This record should be deleted');
      Delete;
    end;
    WriteLn('After Delete:');
    WriteLn('Integer: ',FieldbyName('Integer').AsInteger);
    WriteLn('String: ',FieldbyName('String').AsString);

    WriteLn('Try to find record with code = 22222');
    First;
    while not Eof do
    begin
      if FieldByName('Integer').AsInteger = 22222 then
        Writeln('Record Found Manually: It''s a bug')
      else
        Writeln('Record NOT Found Manually: It''s OK');
      Next;
    end;
    if Locate('Integer',22222,[]) then
      WriteLn('Record Found Using Locate: It''s a bug')
    else
      WriteLn('Record Not Found Using Locate: It''s OK');
    ApplyUpdates;
    writeln('ReturnString after ApplyUpdates: ',ReturnString);
    Destroy;
  end;
end.
