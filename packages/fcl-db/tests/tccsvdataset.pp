unit tccsvdataset;

{$mode objfpc}{$H+}

interface

uses
  Classes, db, SysUtils, fpcunit, testregistry, csvdataset;

type

  { TTestCSVDataset }

  TTestCSVDataset= class(TTestCase)
  private
    FCSVDataset: TCSVDataset;
    // Load CSVDataset from CSV stream containing lines
    procedure DoOpenClose(FieldNames: Boolean);
    Procedure LoadFromLines(Const Lines: Array of string);
    // Save CSVDataset to CSV stream, transform to lines
    Procedure SaveToLines(Const Lines: TStrings);
    // Save CSVDataset to CSV stream, transform to lines, compare with given lines
    Procedure AssertLines(Const Lines: Array of string);
  protected
    procedure SetUp; override;
    procedure TearDown; override;
    Property CSVDataset : TCSVDataset Read FCSVDataset;
  published
    procedure TestEmpty;
    procedure TestDefaults;
    Procedure TestLoadEmptyDefault;
    Procedure TestLoadEmptyFirstLineAsNames;
    Procedure TestLoad2fieldsFirstLineAsNames;
    Procedure TestLoad2fields;
    Procedure TestLoad2Records2fields;
    Procedure TestLoad2Records2fieldsTabDelim;
    Procedure TestSaveEmptyDefault;
    Procedure TestSaveEmptyFirstLineAsNames;
    Procedure TestSaveOneRecordDefault;
    Procedure TestSaveOneRecordFirstLineAsNames;
    Procedure TestSaveTwoRecordsDefault;
    Procedure TestSaveTwoRecordsFirstLineAsNames;
    Procedure TestSaveTwoRecordsFirstLineAsNamesTabDelim;
    Procedure TestSaveOneRecord2FieldsDefault;
    Procedure TestSaveOneRecord2FieldsFirstLineAsNames;
    Procedure TestLoadPriorFieldDefs;
    Procedure TestLoadPriorFieldDefsNoFieldNames;
    Procedure TestLoadPriorFieldDefsNoFieldNamesWrongCount;
    Procedure TestLoadPriorFieldDefsFieldNamesWrongCount;
    Procedure TestLoadPriorFieldDefsFieldNamesWrongNames;
    Procedure TestOpenCloseCycle1;
    Procedure TestOpenCloseCycle2;
  end;

implementation

procedure TTestCSVDataset.TestEmpty;
begin
  AssertNotNull('Have CSV dataset',CSVDataset);
  AssertFalse('Not open',CSVDataset.Active);
  AssertEquals('No fielddefs',0,CSVDataset.FieldDefs.Count);
  AssertEquals('Name','DS',CSVDataset.Name);
end;

procedure TTestCSVDataset.TestDefaults;
begin
  With CSVDataset.CSVOptions do
    begin
    AssertEquals('DefaultFieldLength',255,DefaultFieldLength);
    AssertEquals('FirstLineAsFieldNames',False,FirstLineAsFieldNames);
    AssertEquals('Delimiter',',',Delimiter);
    AssertEquals('QuoteChar','"',QuoteChar);
    AssertEquals('LineEnding',sLineBreak,LineEnding);
    AssertEquals('IgnoreOuterWhitespace',False,IgnoreOuterWhitespace);
    AssertEquals('QuoteOuterWhitespace',True,QuoteOuterWhitespace);
    AssertEquals('EqualColCountPerRow',True,EqualColCountPerRow);
    end;
end;

procedure TTestCSVDataset.LoadFromLines(const Lines: array of string);

Var
  L : TStringList;
  s : TStream;
begin
  S:=Nil;
  L:=TStringList.Create;
  try
    L.AddStrings(Lines);
    S:=TStringStream.Create(L.Text);
    CSVDataset.LoadFromCSVStream(S);
  finally
    S.Free;
    L.Free;
  end;
end;

procedure TTestCSVDataset.SaveToLines(const Lines: TStrings);

Var
  S : TStringStream;

begin
  S:=TStringStream.Create('');
  try
    CSVDataset.SaveToCSVStream(S);
    Lines.Text:=S.DataString;
    {
    Writeln('----');
    Writeln(S.DataString);
    Writeln('----');
    }
  finally
    S.Free;
  end;
end;

procedure TTestCSVDataset.AssertLines(const Lines: array of string);

Var
  L : TStrings;
  I : Integer;
begin
  L:=TStringList.Create;
  try
    SaveToLines(L);
    AssertEquals('Number of lines',Length(Lines),L.Count);
    For I:=0 to L.Count-1 do
      AssertEquals('Correct line '+IntToStr(0),Lines[I],L[i]);
  finally
    L.Free;
  end;
end;

procedure TTestCSVDataset.TestLoadEmptyDefault;
begin
  LoadFromLines(['a']);
  AssertEquals('Active',True,CSVDataset.Active);
  AssertEquals('field count',1,CSVDataset.FieldDefs.Count);
  AssertEquals('field name','Column1',CSVDataset.FieldDefs[0].Name);
  AssertEquals('field size',CSVDataset.CSVOptions.DefaultFieldLength,CSVDataset.FieldDefs[0].Size);
  AssertEquals('Not Empty',False,CSVDataset.EOF and CSVDataset.BOF);
  AssertEquals('field contents','a',CSVDataset.Fields[0].AsString);
end;

procedure TTestCSVDataset.TestLoadEmptyFirstLineAsNames;

begin
  CSVDataset.CSVOptions.FirstLineAsFieldNames:=True;
  CSVDataset.CSVOptions.DefaultFieldLength:=128;
  LoadFromLines(['a']);
  AssertEquals('Active',True,CSVDataset.Active);
  AssertEquals('field count',1,CSVDataset.FieldDefs.Count);
  AssertEquals('field name','a',CSVDataset.FieldDefs[0].Name);
  AssertEquals('field size',CSVDataset.CSVOptions.DefaultFieldLength,CSVDataset.FieldDefs[0].Size);
  AssertEquals('Empty',True,CSVDataset.EOF and CSVDataset.BOF);
end;

procedure TTestCSVDataset.TestLoad2fieldsFirstLineAsNames;
begin
  CSVDataset.CSVOptions.FirstLineAsFieldNames:=True;
  CSVDataset.CSVOptions.DefaultFieldLength:=128;
  LoadFromLines(['a,b']);
  AssertEquals('Active',True,CSVDataset.Active);
  AssertEquals('field count',2,CSVDataset.FieldDefs.Count);
  AssertEquals('field 0 name','a',CSVDataset.FieldDefs[0].Name);
  AssertEquals('field 0 size',CSVDataset.CSVOptions.DefaultFieldLength,CSVDataset.FieldDefs[0].Size);
  AssertEquals('field 1 name','b',CSVDataset.FieldDefs[1].Name);
  AssertEquals('field 1 size',CSVDataset.CSVOptions.DefaultFieldLength,CSVDataset.FieldDefs[1].Size);
  AssertEquals('Empty',True,CSVDataset.EOF and CSVDataset.BOF);
end;

procedure TTestCSVDataset.TestLoad2fields;

begin
  CSVDataset.CSVOptions.DefaultFieldLength:=128;
  LoadFromLines(['a,b']);
  AssertEquals('Active',True,CSVDataset.Active);
  AssertEquals('field count',2,CSVDataset.FieldDefs.Count);
  AssertEquals('field 0 name','Column1',CSVDataset.FieldDefs[0].Name);
  AssertEquals('field 0 size',CSVDataset.CSVOptions.DefaultFieldLength,CSVDataset.FieldDefs[0].Size);
  AssertEquals('field 1 name','Column2',CSVDataset.FieldDefs[1].Name);
  AssertEquals('field 1 size',CSVDataset.CSVOptions.DefaultFieldLength,CSVDataset.FieldDefs[1].Size);
  AssertEquals('Empty',False,CSVDataset.EOF and CSVDataset.BOF);
  AssertEquals('Not Empty',False,CSVDataset.EOF and CSVDataset.BOF);
  AssertEquals('field 0 contents','a',CSVDataset.Fields[0].AsString);
  AssertEquals('field 1 contents','b',CSVDataset.Fields[1].AsString);
end;

procedure TTestCSVDataset.TestLoad2Records2fields;
begin
  CSVDataset.CSVOptions.DefaultFieldLength:=128;
  LoadFromLines(['a,b','c,d']);
  AssertEquals('Active',True,CSVDataset.Active);
  AssertEquals('field count',2,CSVDataset.FieldDefs.Count);
  AssertEquals('field 0 name','Column1',CSVDataset.FieldDefs[0].Name);
  AssertEquals('field 0 size',CSVDataset.CSVOptions.DefaultFieldLength,CSVDataset.FieldDefs[0].Size);
  AssertEquals('field 1 name','Column2',CSVDataset.FieldDefs[1].Name);
  AssertEquals('field 1 size',CSVDataset.CSVOptions.DefaultFieldLength,CSVDataset.FieldDefs[1].Size);
  AssertEquals('Empty',False,CSVDataset.EOF and CSVDataset.BOF);
  AssertEquals('Not Empty',False,CSVDataset.EOF and CSVDataset.BOF);
  AssertEquals('field 0 contents','a',CSVDataset.Fields[0].AsString);
  AssertEquals('field 1 contents','b',CSVDataset.Fields[1].AsString);
  CSVDataset.Next;
  AssertEquals('not At EOF',False,CSVDataset.EOF);
  AssertEquals('field 0 contents','c',CSVDataset.Fields[0].AsString);
  AssertEquals('field 1 contents','d',CSVDataset.Fields[1].AsString);
  CSVDataset.Next;
  AssertEquals('At EOF',True,CSVDataset.EOF);
end;

procedure TTestCSVDataset.TestLoad2Records2fieldsTabDelim;
begin
  CSVDataset.CSVOptions.Delimiter:=#9;
  CSVDataset.CSVOptions.DefaultFieldLength:=10;
  CSVDataset.CSVOptions.FirstLineAsFieldNames:=True;
  LoadFromLines(['f_a'#9'f_b','a1'#9'b1','a2'#9'b2']);
  AssertEquals('Active',True,CSVDataset.Active);
  AssertEquals('field count',2,CSVDataset.FieldDefs.Count);
  AssertEquals('field 0 name','f_a',CSVDataset.FieldDefs[0].Name);
  AssertEquals('field 0 size',CSVDataset.CSVOptions.DefaultFieldLength,CSVDataset.FieldDefs[0].Size);
  AssertEquals('field 1 name','f_b',CSVDataset.FieldDefs[1].Name);
  AssertEquals('field 1 size',CSVDataset.CSVOptions.DefaultFieldLength,CSVDataset.FieldDefs[1].Size);
  AssertEquals('field 0 contents','a1',CSVDataset.Fields[0].AsString);
  AssertEquals('field 1 contents','b1',CSVDataset.Fields[1].AsString);
  CSVDataset.Next;
  AssertEquals('field 0 contents','a2',CSVDataset.Fields[0].AsString);
  AssertEquals('field 1 contents','b2',CSVDataset.Fields[1].AsString);
  CSVDataset.Next;
  AssertEquals('At EOF',True,CSVDataset.EOF);
end;

procedure TTestCSVDataset.TestSaveEmptyDefault;
begin
  CSVDataset.FieldDefs.Add('a',ftString);
  CSVDataset.CreateDataset;
  AssertLines([]);
end;

procedure TTestCSVDataset.TestSaveEmptyFirstLineAsNames;
begin
  CSVDataset.CSVOptions.FirstLineAsFieldNames:=True;
  CSVDataset.FieldDefs.Add('a',ftString);
  CSVDataset.CreateDataset;
  AssertLines(['a']);
end;

procedure TTestCSVDataset.TestSaveOneRecordDefault;
begin
//  CSVDataset.CSVOptions.FirstLineAsFieldNames:=True;
  CSVDataset.FieldDefs.Add('a',ftString,20);
  CSVDataset.CreateDataset;
  CSVDataset.Append;
  CSVDataset.Fields[0].AsString:='b';
  CSVDataset.Post;
  AssertLines(['b']);
end;

procedure TTestCSVDataset.TestSaveOneRecordFirstLineAsNames;
begin
  CSVDataset.CSVOptions.FirstLineAsFieldNames:=True;
  CSVDataset.FieldDefs.Add('a',ftString,20);
  CSVDataset.CreateDataset;
  CSVDataset.Append;
  CSVDataset.Fields[0].AsString:='b';
  CSVDataset.Post;
  AssertLines(['a','b']);
end;

procedure TTestCSVDataset.TestSaveTwoRecordsDefault;
begin
  CSVDataset.FieldDefs.Add('a',ftString,20);
  CSVDataset.CreateDataset;
  CSVDataset.Append;
  CSVDataset.Fields[0].AsString:='b';
  CSVDataset.Post;
  CSVDataset.Append;
  CSVDataset.Fields[0].AsString:='c';
  CSVDataset.Post;
  AssertLines(['b','c']);
end;

procedure TTestCSVDataset.TestSaveTwoRecordsFirstLineAsNames;
begin
  CSVDataset.CSVOptions.FirstLineAsFieldNames:=True;
  CSVDataset.FieldDefs.Add('a',ftString,20);
  CSVDataset.CreateDataset;
  CSVDataset.Append;
  CSVDataset.Fields[0].AsString:='b';
  CSVDataset.Post;
  CSVDataset.Append;
  CSVDataset.Fields[0].AsString:='c';
  CSVDataset.Post;
  AssertLines(['a','b','c']);
end;

procedure TTestCSVDataset.TestSaveTwoRecordsFirstLineAsNamesTabDelim;
begin
  CSVDataset.CSVOptions.Delimiter:=#9;
  CSVDataset.CSVOptions.FirstLineAsFieldNames:=True;
  CSVDataset.FieldDefs.Add('f_a',ftString,2);
  CSVDataset.FieldDefs.Add('f_b',ftString,2);
  CSVDataset.CreateDataset;
  CSVDataset.InsertRecord(['a2','b2']);
  CSVDataset.InsertRecord(['a1','b1']);
  AssertLines(['f_a'#9'f_b','a1'#9'b1','a2'#9'b2']);
end;

procedure TTestCSVDataset.TestSaveOneRecord2FieldsDefault;
begin
  CSVDataset.FieldDefs.Add('a',ftString,20);
  CSVDataset.FieldDefs.Add('b',ftString,20);
  CSVDataset.CreateDataset;
  CSVDataset.Append;
  CSVDataset.Fields[0].AsString:='c';
  CSVDataset.Fields[1].AsString:='d';
  CSVDataset.Post;
  AssertLines(['c,d']);
end;

procedure TTestCSVDataset.TestSaveOneRecord2FieldsFirstLineAsNames;
begin
  CSVDataset.CSVOptions.FirstLineAsFieldNames:=True;
  CSVDataset.FieldDefs.Add('a',ftString,20);
  CSVDataset.FieldDefs.Add('b',ftString,20);
  CSVDataset.CreateDataset;
  CSVDataset.Append;
  CSVDataset.Fields[0].AsString:='c';
  CSVDataset.Fields[1].AsString:='d';
  CSVDataset.Post;
  AssertLines(['a,b','c,d']);
end;

procedure TTestCSVDataset.TestLoadPriorFieldDefs;
begin
  CSVDataset.CSVOptions.FirstLineAsFieldNames:=True;
  CSVDataset.FieldDefs.Add('a',ftString,20);
  CSVDataset.FieldDefs.Add('b',ftInteger,4);
  LoadFromLines(['a,b','1,2']);
  AssertEquals('field count',2,CSVDataset.FieldDefs.Count);
  AssertEquals('field 0 name','a',CSVDataset.FieldDefs[0].Name);
  AssertEquals('field 0 size',20,CSVDataset.FieldDefs[0].Size);
  AssertEquals('field 1 name','b',CSVDataset.FieldDefs[1].Name);
  AssertEquals('field 1 size',4,CSVDataset.FieldDefs[1].Size);
  AssertEquals('field 1 typee',Ord(ftInteger),Ord(CSVDataset.FieldDefs[1].DataType));
  AssertEquals('Not Empty',False,CSVDataset.EOF and CSVDataset.BOF);
  AssertEquals('field 0 contents','1',CSVDataset.Fields[0].AsString);
  AssertEquals('field 1 contents',2,CSVDataset.Fields[1].AsInteger);
end;

procedure TTestCSVDataset.TestLoadPriorFieldDefsNoFieldNames;
begin
  CSVDataset.FieldDefs.Add('a',ftString,20);
  CSVDataset.FieldDefs.Add('b',ftInteger,4);
  LoadFromLines(['1,2']);
  AssertEquals('field count',2,CSVDataset.FieldDefs.Count);
  AssertEquals('field 0 name','a',CSVDataset.FieldDefs[0].Name);
  AssertEquals('field 0 size',20,CSVDataset.FieldDefs[0].Size);
  AssertEquals('field 1 name','b',CSVDataset.FieldDefs[1].Name);
  AssertEquals('field 1 size',4,CSVDataset.FieldDefs[1].Size);
  AssertEquals('field 1 typee',Ord(ftInteger),Ord(CSVDataset.FieldDefs[1].DataType));
  AssertEquals('Not Empty',False,CSVDataset.EOF and CSVDataset.BOF);
  AssertEquals('field 0 contents','1',CSVDataset.Fields[0].AsString);
  AssertEquals('field 1 contents',2,CSVDataset.Fields[1].AsInteger);
end;

procedure TTestCSVDataset.TestLoadPriorFieldDefsNoFieldNamesWrongCount;

Var
  OK : Boolean;
begin
  CSVDataset.FieldDefs.Add('a',ftString,20);
  CSVDataset.FieldDefs.Add('b',ftInteger,4);
  try
    LoadFromLines(['1']);
    OK:=False;
  except
    OK:=true;
  end;
  if not OK then
    Fail('Expected exception, but none raised');
end;

procedure TTestCSVDataset.TestLoadPriorFieldDefsFieldNamesWrongCount;

const
  EM = 'DS : CSV File Field count (1) does not match dataset field count (2).';
Var
  OK : String;
begin
  CSVDataset.CSVOptions.FirstLineAsFieldNames:=True;
  CSVDataset.FieldDefs.Add('a',ftString,20);
  CSVDataset.FieldDefs.Add('b',ftInteger,4);
  try
    LoadFromLines(['A']);
    OK:='Expected exception, but none raised';
  except
    on E : Exception do
      if  (E.Message<>EM) then
        OK:=ComparisonMsg(EM,E.Message);
  end;
  if (OK<>'') then
    Fail(OK);
end;

procedure TTestCSVDataset.TestLoadPriorFieldDefsFieldNamesWrongNames;
const
  EM = 'DS : CSV File field 1: name "c" does not match dataset field name "b".';
Var
  OK : String;
begin
  CSVDataset.CSVOptions.FirstLineAsFieldNames:=True;
  CSVDataset.FieldDefs.Add('a',ftString,20);
  CSVDataset.FieldDefs.Add('b',ftInteger,4);
  try
    LoadFromLines(['a,c']);
    OK:='No exception raised';
  except
    on E : Exception do
      if  (E.Message<>EM) then
        OK:=ComparisonMsg(EM,E.Message)
  end;
  if (OK<>'') then
    Fail(OK);
end;

const
  FILENAME = 'test.dat';

procedure TTestCSVDataset.DoOpenClose(FieldNames : Boolean);

begin
  CSVDataset.FileName := FILENAME;
  With CSVDataset do
     begin
     CSVOptions.FirstLineAsFieldNames := FieldNames;
     CSVOptions.DefaultFieldLength := 255;
     CSVOptions.Delimiter := ',';
     CSVOptions.QuoteChar := '"';
     CSVOptions.IgnoreOuterWhitespace := False;
     CSVOptions.QuoteOuterWhitespace := True;
     end;
  // When the program runs for the first time, the data file does not yet exist.
  // We must create the FieldDefs and create the dataset.
  if FileExists(CSVDataset.FileName) then
    CSVDataset.Open
  else
    with CSVDataset do
      begin
      FieldDefs.Add('FirstName', ftString, 20);
      FieldDefs.Add('LastName', ftstring, 20);
      FieldDefs.Add('City', ftString, 20);
      FieldDefs.Add('Address', ftString, 30);
      FieldDefs.Add('Birthdate', ftDate);
      CreateDataset;

      // Open the dataset...
      Open;

      // ... and add some dummy data:
      // Names from https://donatellanobatti.blogspot.de/
      Append;
      FieldByName('FirstName').AsString := 'Walter';
      FieldByName('LastName').AsString := 'Mellon';
      FieldByName('City').AsString := 'Oklahoma City';
      FieldByName('Address').AsString :=  '1261, Main Street';
      FieldbyName('Birthdate').AsDateTime := EncodeDate(1980, 1, 1);
      Post;

      Append;
      FieldByName('FirstName').AsString := 'Mario';
      FieldByName('LastName').AsString := 'Speedwagon';
      FieldByName('City').AsString := 'Hollywood';
      FieldByName('Address').AsString :=  '1500, Hollywood Blvd';
      FieldbyName('Birthdate').AsDateTime := EncodeDate(1982, 12, 17);
      Post;

      Append;
      FieldByName('FirstName').AsString := 'Anna';
      FieldByName('LastName').AsString := 'Mull';
      FieldByName('City').AsString := 'Los Angeles';
      FieldByName('Address').AsString :=  '2202, Capitol Square';
      FieldbyName('Birthdate').AsDateTime := EncodeDate(1982, 12, 17);
      Post;
      end;
  // Would be 4 if first line misinterpreted
  AssertEquals('RecordCount',3,CSVDataset.RecordCount);
  // This will write the file;
  CSVDataset.Close;
end;

procedure TTestCSVDataset.TestOpenCloseCycle1;
begin
  if FileExists(FileName) then
    AssertTrue('Delete before',DeleteFile(FileName));
  try
    // This will create the file
    DoOpenClose(True);
    // Recreate to be sure
    FreeAndNil(FCSVDataset);
    FCSVDataset:=TCSVDataset.Create(Nil);
    FCSVDataset.Name:='DS';
    DoOpenClose(True);
  except
    On E : Exception do
      Fail('Failed using exception %s : %s',[E.ClassName,E.Message]);
  end;
  if FileExists(FileName) then
    AssertTrue('Delete after',DeleteFile(FileName));
end;

procedure TTestCSVDataset.TestOpenCloseCycle2;
begin
  if FileExists(FileName) then
    AssertTrue('Delete before',DeleteFile(FileName));
  try
    // This will create the file
    DoOpenClose(False);
    // Recreate to be sure
    FreeAndNil(FCSVDataset);
    FCSVDataset:=TCSVDataset.Create(Nil);
    FCSVDataset.Name:='DS';
    DoOpenClose(False);
  except
    On E : Exception do
      Fail('Failed using exception %s : %s',[E.ClassName,E.Message]);
  end;
  if FileExists(FileName) then
    AssertTrue('Delete after',DeleteFile(FileName));
end;


procedure TTestCSVDataset.SetUp;
begin
  FCSVDataset:=TCSVDataset.Create(Nil);
  FCSVDataset.Name:='DS';
end;

procedure TTestCSVDataset.TearDown;
begin
  FreeAndNil(FCSVDataset);
end;

Initialization

  RegisterTest(TTestCSVDataset);
end.

