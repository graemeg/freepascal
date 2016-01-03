program createds;

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
  sqlite3ds,
=======
  sqliteds,
>>>>>>> graemeg/fixes_2_2
=======
  sqliteds,
>>>>>>> origin/fixes_2_2
  sysutils,db,inifiles;

const
  SQLITEDS_TESTS_INI_FILE = 'sqlitedstests.ini';
  DEFAULT_TABLENAME = 'tabletest';
  DEFAULT_FILENAME = 'test.db';
  
var 
<<<<<<< HEAD
<<<<<<< HEAD
  dsTest: TSqlite3Dataset;
=======
  dsTest:TSqliteDataset;
>>>>>>> graemeg/fixes_2_2
=======
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
    //Ensure the file does not exist yet
    if FileExists(FileName) then
      DeleteFile(FileName);
    //Create a table with all available field types
    with FieldDefs do
    begin
      Clear;
      Add('Integer',ftInteger);
      Add('String',ftString);
      Add('Boolean',ftBoolean);
      Add('Float',ftFloat);
      Add('Word',ftWord);
      Add('DateTime',ftDateTime);
      Add('Date',ftDate);
      Add('Time',ftTime);
      Add('AutoInc',ftAutoInc);
      Add('Memo',ftMemo);
      Add('LargeInt',ftLargeint);
      Add('Currency',ftCurrency);
    end; 
<<<<<<< HEAD
<<<<<<< HEAD
    if CreateTable then
	begin
	  WriteLn('Table created successfully');
	  if not TableExists then
	    WriteLn('TableExists check failed with error: ', ReturnString);
	end  
	else
      WriteLn('Error creating table');
    WriteLn('ReturnString after CreateTable: ',ReturnString);
=======
    CreateTable;
    writeln('ReturnString after CreateTable: ',ReturnString);
>>>>>>> graemeg/fixes_2_2
=======
    CreateTable;
    writeln('ReturnString after CreateTable: ',ReturnString);
>>>>>>> origin/fixes_2_2
    Destroy;
  end;
end.
