program testjsondataset;

{$DEFINE TESTCALCFIELDS}
{$DEFINE TESTLOOKUPFIELDS}

uses classes, variants, varutils, sysutils, db, fpjson , fpjsondataset, ExtJSDataset, types;

Type

  { TApp }

  TApp = Class(TComponent)
  private
    DS : TExtJSJSONObjectDataSet;
    DC : TExtJSJSONObjectDataSet;
    Procedure DumpRecord(aDS : TDataset);
    Procedure DumpRecords(aDS : TDataset);
    procedure CreateDataset;
    procedure DoCalcFields(DataSet: TDataSet);
    procedure TestAppend;
    procedure TestBookMark;
    procedure TestDataLinkActiveRecord;
    procedure TestDataLinkEdit;
    procedure TestEdit;
    procedure TestInsert;
    procedure TestLocate;
    procedure TestLookup;
    procedure TestNavigation;
  Public
    Procedure Run;
  end;

Procedure TApp.DumpRecord(aDS : TDataset);

begin
  Writeln(
  {$IFDEF TESTCALCFIELDS}
          'Full name: ',aDS.FieldByName('fullname').AsString,
  {$ENDIF}
          'First name: ',aDS.FieldByName('firstname').AsString,
          ', Last name:', aDS.FieldByName('lastname').AsString,
          ', Children:', aDS.FieldByName('children').AsInteger,
          ', Birthday:', aDS.FieldByName('birthday').AsString,
          ', Weight:', aDS.FieldByName('weight').AsFloat,
          ', Business:', aDS.FieldByName('business').AsBoolean,
          ', Country:', aDS.FieldByName('Country').AsString
  {$IFDEF TESTLOOKUPFIELDS}
          ,', CountryName:', aDS.FieldByName('CountryName').AsString
  {$ENDIF}
  );
end;

Procedure TApp.DumpRecords(aDS : TDataset);

begin
  While not aDS.EOF do
    begin
    DumpRecord(aDS);
    aDS.Next;
    end;
end;


Procedure TApp.CreateDataset;

{$IFDEF TESTCALCFIELDS}
Var
  F : TField;
{$ENDIF}

begin
  Writeln('Creating dataset');
  DS:=TExtJSJSONObjectDataSet.Create(Self);
  DS.MetaData:=GetJSON('{ "fields" : [ '+
                                          ' { "name": "firstname"}, '+
                                          ' { "name": "lastname"}, '+
                                          ' { "name" : "children", "type": "int" }, '+
                                          ' { "name" : "birthday", "type": "date", "dateFormat": "yyyy\"-\"mm\"-\"dd" }, '+
                                          ' { "name" : "country",  "type": "string", "maxLen" : 2 }, '+
                                          ' { "name" : "business", "type": "boolean" }, '+
                                          ' { "name" : "weight",   "type": "float" } '+
                                       ']}') as TJSONObject;
  DS.Rows:=GetJSON('[{"firstname" : "Michael", "lastname" : "Van Canneyt", "children" : 2, "birthday": "1970-07-07", "business" : false, "weight": 75.5, "country": "BE" },'+
                                  '  {"firstname" : "Mattias", "lastname" : "Gaertner", "children" : 0, "birthday" : "1970-07-08", "business" : false, "weight": 76.2, "country": "DE"   }, '+
                                  '  {"firstname" : "Bruno", "lastname" : "Fierens", "children" : 3, "birthday" : "1970-07-09", "business" : true, "weight": 77.3, "country": "BE"  },'+
                                  '  {"firstname" : "Detlef", "lastname" : "Overbeek", "children" : 2, "birthday" : "1950-07-08", "business" : true, "weight": 78.8, "country": "NL"  }'+
                                  ' ]') as TJSONArray;
  DC:=TExtJSJSONObjectDataSet.Create(Self);
  DC.MetaData:=GetJSON('{ "fields" : [ '+
                                       ' { "name": "code"}, '+
                                       ' { "name": "name"} '+
                                       ']} ') as TJSONObject;
  DC.Rows:=GetJSON('[{"code" : "BE", "name" : "Belgium" }, '+
                                 '  {"code" : "DE", "name" : "Germany" }, '+
                                 '  {"code" : "NL", "name" : "Netherlands" }, '+
                                 '  {"code" : "FR", "name" : "France" }, '+
                                 '  {"code" : "UK", "name" : "United Kingdom" } '+
                                 ' ]') as TJSONArray;
{$IFDEF TESTCALCFIELDS}
  F:=TStringField.Create(DS);
  F.FieldKind:=fkCalculated;
  F.Size:=200;
  F.FieldName:='FullName';
  F.Dataset:=DS;
  F:=TStringField.Create(DS);
  F.FieldKind:=fkData;
  F.Size:=200;
  F.FieldName:='firstname';
  F.Dataset:=DS;
  F:=TStringField.Create(DS);
  F.FieldKind:=fkData;
  F.Size:=200;
  F.FieldName:='lastname';
  F.Dataset:=DS;
  F:=TIntegerField.Create(DS);
  F.FieldKind:=fkData;
  F.FieldName:='children';
  F.Dataset:=DS;
  F:=TDateField.Create(DS);
  F.FieldKind:=fkData;
  F.FieldName:='birthday';
  F.Dataset:=DS;
  F:=TBooleanField.Create(DS);
  F.FieldKind:=fkData;
  F.FieldName:='business';
  F.Dataset:=DS;
  F:=TFloatField.Create(DS);
  F.FieldKind:=fkData;
  F.FieldName:='weight';
  F.Dataset:=DS;
  F:=TStringField.Create(DS);
  F.FieldKind:=fkData;
  F.Size:=2;
  F.FieldName:='country';
  F.Dataset:=DS;
  {$IFDEF TESTLOOKUPFIELDS}
  F:=TStringField.Create(DS);
  F.FieldKind:=fkLookup;
  F.LookupDataSet:=DC;
  F.KeyFields:='country';
  F.LookupKeyFields:='code';
  F.LookupResultField:='name';
  F.FieldName:='CountryName';
  F.Dataset:=DS;
  {$ENDIF}
  DS.OnCalcFields:=@DoCalcFields;
{$ENDIF}
end;

Procedure TApp.TestNavigation;

begin
  Writeln('All records');
  DumpRecords(DS);
  Writeln('First record (expect Michael.)');
  DS.First;
  DumpRecord(DS);
  Writeln('Jump to last (expect detlef)');
  DS.Last;
  DumpRecord(DS);
  Writeln('Reverse order:');
  While not DS.BOF do
    begin
    DumpRecord(DS);
    DS.Prior;
    end;
end;

Procedure TApp.TestAppend;

begin
  DS.Append;
  Writeln('Dumping record after APPEND (expect empty)');
  Writeln('Modified before  (expect False): ',DS.Modified);
  DumpRecord(DS);
  DS.FieldByName('firstname').AsString:='Florian';
  Writeln('Old value of field first name (expect null): ', varisNull(DS.FieldByName('firstname').OldValue) );
  DS.FieldByName('lastname').AsString:='Klaempfl';
  DS.FieldByName('children').AsInteger:=1;
  DS.FieldByName('birthday').AsDateTime:=EncodeDate(1980,5,4);
  Writeln('Modified after (expect true): ',DS.Modified);
  Writeln('Dumping record before POST (Expect Florian)');
  DumpRecord(DS);
  DS.Post;
  Writeln('Dumping record after POST (Expect Florian)');
  DumpRecord(DS);
  Writeln('Jump to first (expect Michael)');
  DS.First;
  DumpRecord(DS);
end;

Procedure TApp.TestEdit;

begin
  DS.Edit;
  Writeln('Dumping record after EDIT');
  Writeln('Modified before  (expect False): ',DS.Modified);
  DumpRecord(DS);
  DS.FieldByName('firstname').AsString:='Dolores';
  DS.FieldByName('lastname').AsString:='Nabokov';
  DS.FieldByName('children').AsInteger:=0;
  DS.FieldByName('birthday').AsDateTime:=EncodeDate(1943,2,14);
  Writeln('Modified after (expect true): ',DS.Modified);
  Writeln('Dumping record before POST (expect Dolores)');
  DumpRecord(DS);
  DS.Post;
  Writeln('Dumping record after POST (expect Dolores)');
  DumpRecord(DS);
  DS.Edit;
  Writeln('Dumping record after second EDIT (Expect Dolores)');
  DumpRecord(DS);
  Writeln('Modified before  (expect False): ',DS.Modified);
  DS.FieldByName('firstname').AsString:='Felicity';
  Writeln('Old value of field first name (expect Dolores): ', DS.FieldByName('firstname').OldValue);
  DS.FieldByName('lastname').AsString:='Brown';
  DS.FieldByName('children').AsInteger:=0;
  DS.FieldByName('birthday').AsDateTime:=EncodeDate(1943,2,14);
  Writeln('Modified after (expect true): ',DS.Modified);
  Writeln('Dumping record before Cancel (expect Filicity brown)');
  DumpRecord(DS);
  DS.Cancel;
  Writeln('Dumping record after Cancel (expect Dolores)');
  DumpRecord(DS);
  Writeln('Jump to first and dumping all records (expect Dolores first)');
  DS.First;
  DumpRecords(DS);
end;

Procedure TApp.TestBookMark;

var
  B : TBookmark;

begin
  Writeln('Jump to first  (expect Dolores)');
  DS.First;
  DumpRecord(DS);
  DS.Next;
  DS.Next;
  DS.Next;
  Writeln('Getting Bookmark (expect Detlef)');
  DumpRecord(DS);
  B:=DS.BookMark;
  DS.First;
  Writeln('Delete (expect Mattias)');
  DS.Delete;
  DumpRecord(DS);
  Writeln('Setting Bookmark (expect Detlef)');
  DS.BookMark:=B;
  DumpRecord(DS);
end;

Procedure TApp.TestInsert;

begin
  Writeln('Jump to second (expect Bruno)');
  DS.First;
  DS.Next;
  DumpRecord(DS);
  DS.Insert;
  Writeln('Dumping record after second Insert (Expect empty)');
  Writeln('Modified (expect False): ',DS.Modified);
  DumpRecord(DS);
  DS.FieldByName('firstname').AsString:='Felicity';
  DS.FieldByName('lastname').AsString:='Brown';
  DS.FieldByName('children').AsInteger:=0;
  DS.FieldByName('birthday').AsDateTime:=EncodeDate(1963,4,6);
  Writeln('Modified (expect true): ',DS.Modified);
  Writeln('Dumping record before POST (expect Filicity)');
  DumpRecord(DS);
  DS.Post;
  Writeln('Dumping record after POST (expect Felicity)');
  DumpRecord(DS);
  Writeln('Jump to first and dumping all records (expect Mattias first, then Felicity)');
  DS.First;
  DumpRecords(DS);
end;

Procedure TApp.TestDataLinkEdit;

var
  t: TDataLink;
  DSS : TDatasource;

begin
  Writeln('Jump to first before edit');
  DS.First;
  DSS:=Nil;
  t:=TDataLink.Create;
  try
    DSS:=TDatasource.Create(self);
    DSS.DataSet:=DS;
    Writeln('Buffercount');
    t.BufferCount := 10;
    t.DataSource := DSS;
    Writeln('Doing edit');
    t.Edit;
    Writeln('Modified (expect false): ',DS.Modified);
    Writeln('Done edit');
    t.ActiveRecord := 0;
    Writeln('Edit, expect Mattias');
    DumpRecord(DS);
    Writeln('Activerecord 1: expect Felicity');
    t.ActiveRecord := 1;
    DumpRecord(DS);
    Writeln('Activerecord 2: expect Bruno');
    t.ActiveRecord := 2;
    DumpRecord(DS);
    t.ActiveRecord := 0;
  Finally
    t.Free;
    dss.free;
  end;
end;

Procedure TApp.TestDataLinkActiveRecord;

var
  t: TDataLink;
  DSS : TDatasource;

begin
  DSS:=Nil;
  t:=TDataLink.Create;
  try
    DSS:=TDatasource.Create(Self);
    DSS.DataSet:=DS;
    DSS.DataSet:=DS;
    t.DataSource := DSS;
    DS.Last;
    Writeln('Last record :',DS.RecNo);
    Writeln('Activerecord :',T.ActiveRecord);
    DumpRecord(DS);
    DS.First;
    t.BufferCount := 3;
    DS.Last;
    Writeln('Last record after buffercount lessened:',DS.RecNo);
    Writeln('Activerecord :',T.ActiveRecord);
    DumpRecord(DS);
    t.ActiveRecord := 0;
  Finally
    t.Free;
    dss.Free;
  end;
end;

Procedure TApp.TestLocate;

Var
  V : Variant;

begin
  DS.First;
  Writeln('Locating 3 children (expect true, Bruno): ',DS.Locate('Children',3,[]));
  DumpRecord(DS);
  DS.First;
  v:=VarArrayCreate([0,0],varVariant);
  V[0]:=3;
  Writeln('Locating 3 children using array (expect true, Bruno): ',DS.Locate('Children',V,[]));
  DumpRecord(DS);
  DS.First;
  Writeln('Locating 4 children (expect false): ',DS.Locate('Children',4,[]));
  DS.First;
  Writeln('Locating first name Detlef (expect true): ',DS.Locate('Firstname','Detlef',[]));
  DumpRecord(DS);
  DS.First;
  Writeln('Locating first name detlef (expect false): ',DS.Locate('Firstname','detlef',[]));
  DS.First;
  Writeln('Locating first name detlef (loCaseInsensitive, expect true): ',DS.Locate('Firstname','detlef',[loCaseInsensitive]));
  DumpRecord(DS);
  DS.First;
  Writeln('Locating first name Det (expect false): ',DS.Locate('Firstname','Det',[]));
  DS.First;
  Writeln('Locating first name Det (loPartialKey,expect true): ',DS.Locate('Firstname','Det',[loPartialKey]));
  DumpRecord(DS);
  DS.First;
  Writeln('Locating first name det (loPartialKey, expect false): ',DS.Locate('Firstname','det',[loPartialKey]));
  DS.First;
  Writeln('Locating first name det (loCaseInsensitive,loPartialKey, expect true): ',DS.Locate('Firstname','det',[loCaseInsensitive,loPartialKey]));
  DumpRecord(DS);
  v:=VarArrayCreate([0,1],varVariant);
  V[0]:=3;
  V[1]:='Detlef';
  DS.First;
  Writeln('Locating first name Detlef & children 3  ( expect false): ',DS.Locate('Children;Firstname',v,[]));
  V[0]:=2;
  V[1]:='Detlef';
  DS.First;
  Writeln('Locating first name Detlef & children 2 ( expect true): ',DS.Locate('Children;Firstname',v,[]));
  DS.First;
  Writeln('Locating birthday  (expect true, Bruno): ',DS.Locate('BirthDay',EncodeDate(1970,07,09),[]));
  DS.First;
  Writeln('Locating business  (expect true, Bruno): ',DS.Locate('business',true,[]));
  DumpRecord(DS);
  DS.First;
  Writeln('Deleting first');
  DS.Delete;
  Writeln('Locating weight  (expect true, bruno): ',DS.Locate('weight',77.3,[]));
  DumpRecord(DS);

end;

procedure TApp.TestLookup;
begin
  DS.First;
  Writeln('Locating weight  (expect true, detlef overbeek): ',DS.Lookup('weight',78.8,'fullname'));
  Writeln('Still on Michael:');
  DumpRecord(DS);
  DS.First;
  Writeln('Locating birthday  (expect true, Bruno): ',DS.Lookup('BirthDay',EncodeDate(1970,07,09),'firstname'));
  Writeln('Still on Michael:');
  DumpRecord(DS);
end;

Procedure TApp.Run;


begin
  try
    CreateDataset;
    Writeln('Opening dataset');

    DC.Open;
    DS.Open;
    TestLocate;
    TestLookup;
//    exit;
    TestNavigation;
    TestAppend;
    TestEdit;
    TestBookmark;
    TestInsert;
    TestDataLinkEdit;
    TestDataLinkActiveRecord;
  except
    On E : Exception do
      Writeln('!! Caught Exception ',E.ClassName,' : ',E.Message);
  end;
end;

procedure TApp.DoCalcFields(DataSet: TDataSet);
begin
//  Writeln('In callback');
  Dataset.FieldByName('FullName').AsString:= Dataset.FieldByName('firstName').AsString+' '+Dataset.FieldByName('lastname').AsString;
end;

begin
  With Tapp.Create(nil) do
    try
      Run;
    finally
      Free;
    end;  
end.

