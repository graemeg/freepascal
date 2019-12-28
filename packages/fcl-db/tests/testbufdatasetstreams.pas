unit TestBufDatasetStreams;

{$IFDEF FPC}
  {$mode objfpc}{$H+}
{$ENDIF}

interface

uses
  fpcunit, testregistry, Classes, SysUtils, db, BufDataset;

type

  { TTestBufDatasetStreams }

  TUpdDatasetProc = procedure(ADataset : TCustomBufDataset) of object;

  // Use this class, it implements some abstract methods
  
  { TMyCustomBufDataset }

  TMyCustomBufDataset = Class(TCustomBufDataset)
  protected
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef; ABlobBuf: PBufBlobField); override;
  end;

  TTestBufDatasetStreams = class(TTestCase)
  private
    procedure CompareDatasets(ADataset1, ADataset2 : TDataset);

    procedure TestChangesApplyUpdates(AUpdDatasetProc : TUpdDatasetProc; Inserts: Boolean=False);
    procedure TestChangesCancelUpdates(AUpdDatasetProc : TUpdDatasetProc);
    procedure TestChanges(AUpdDatasetProc : TUpdDatasetProc; AFormat : TDataPacketFormat=dfBinary);
    procedure TestChangesXML(AUpdDatasetProc : TUpdDatasetProc);

    procedure SimpleEditChange(ADataset: TCustomBufDataset);
    procedure SimpleDeleteChange(ADataset: TCustomBufDataset);
    procedure MoreDeletesChange(ADataset: TCustomBufDataset);
    procedure SimpleInsertChange(ADataset: TCustomBufDataset);
    procedure MoreInsertsChange(ADataset: TCustomBufDataset);
    procedure SeveralEditsChange(ADataset: TCustomBufDataset);
    procedure DeleteAllChange(ADataset: TCustomBufDataset);
    procedure DeleteAllInsertChange(ADataset: TCustomBufDataset);
    procedure NullInsertChange(ADataset: TCustomBufDataset);
    procedure NullEditChange(ADataset: TCustomBufDataset);
    procedure AppendDeleteChange(ADataset: TCustomBufDataset);

    procedure TestStreamingDataFields(AFormat: TDataPacketFormat);
    procedure TestStreamingNullFields(AFormat: TDataPacketFormat);
  protected
    procedure SetUp; override;
    procedure TearDown; override;
  published
    procedure TestSimpleEditCancelUpd;
    procedure TestSimpleDeleteCancelUpd;
    procedure TestMoreDeletesCancelUpd;
    procedure TestSimpleInsertCancelUpd;
    procedure MoreInsertsCancelUpd;
    procedure SeveralEditsCancelUpd;
    procedure DeleteAllCancelUpd;
    procedure DeleteAllInsertCancelUpd;
    procedure AppendDeleteCancelUpd;

    procedure TestSimpleEditApplUpd;
    procedure TestSimpleDeleteApplUpd;
    procedure TestMoreDeletesApplUpd;
    procedure TestSimpleInsertApplUpd;
    procedure MoreInsertsApplUpd;
    procedure SeveralEditsApplUpd;
    procedure DeleteAllApplUpd;
    procedure DeleteAllInsertApplUpd;
    procedure NullInsertUpdateApplUpd;

    procedure TestBasicsXML;
    procedure TestSimpleEditXML;
    procedure TestSimpleDeleteXML;
    procedure TestMoreDeletesXML;
    procedure TestSimpleInsertXML;
    procedure TestMoreInsertsXML;
    procedure TestSeveralEditsXML;
    procedure TestDeleteAllXML;
    procedure TestDeleteAllInsertXML;
    procedure TestStreamingDataFieldsBIN;
    procedure TestStreamingDataFieldsXML;
    procedure TestStreamingBigBlobFieldsXML;
    procedure TestStreamingNullFieldsBIN;
    procedure TestStreamingNullFieldsXML;
    procedure TestStreamingCalculatedFieldsXML;

    procedure TestAppendDeleteBIN;

    procedure TestFileNameProperty;
    procedure TestXmlFileRecognition;
    procedure TestCloseDatasetNoConnection; // bug 17623
  end;

implementation

uses toolsunit, SQLDBToolsUnit, sqldb, XMLDatapacketReader;

const TestXMLFileName = 'test.xml';
      TestBINFileName = 'test.dat';
      TestFileNames: array[TDataPacketFormat] of string = (TestBINFileName, TestXMLFileName, TestXMLFileName, '','');

{ TMyCustomBufDataset }

procedure TMyCustomBufDataset.LoadBlobIntoBuffer(FieldDef: TFieldDef; ABlobBuf: PBufBlobField);
begin
  Raise ENotImplemented.Create('LoadBlobIntoBuffer not implemented');
end;

procedure TTestBufDatasetStreams.CompareDatasets(ADataset1,
  ADataset2: TDataset);
begin
  ADataset1.First;
  ADataset2.First;
  while not ADataset1.EOF do
    begin
    AssertEquals(ADataset1.FieldByName('ID').AsInteger,ADataset2.FieldByName('ID').AsInteger);
    AssertEquals(ADataset1.FieldByName('NAME').AsString,ADataset2.FieldByName('NAME').AsString);
    ADataset1.Next;
    ADataset2.Next;
    end;
end;

procedure TTestBufDatasetStreams.TestChangesApplyUpdates(
  AUpdDatasetProc: TUpdDatasetProc; Inserts : Boolean);
var OrgDs,
    ChangedDs : TCustomBufDataset;
begin
  OrgDs := DBConnector.GetNDataset(true,15) as TCustomBufDataset;
  OrgDs.Open;
  AUpdDatasetProc(OrgDs);
  OrgDs.ApplyUpdates;

  if Inserts then
    begin
    ChangedDs := TSQLDBConnector(DBConnector).Query;
    ChangedDs.Close;
    TSQLQuery(ChangedDS).SQL.Text:='SELECT * FROM FPDEV WHERE (ID < 16) or (ID>100) ORDER BY ID';

    OrgDs.IndexFieldNames:='ID';
    OrgDs.First;
    end
  else
    ChangedDs := DBConnector.GetNDataset(true,15) as TCustomBufDataset;
  ChangedDs.Open;
  CompareDatasets(OrgDs,ChangedDs);
end;

procedure TTestBufDatasetStreams.TestChangesCancelUpdates(
  AUpdDatasetProc: TUpdDatasetProc);
var OrgDs,
    ChangedDs : TCustomBufDataset;
begin
  OrgDs := DBConnector.GetNDataset(true,15) as TCustomBufDataset;
  OrgDs.Open;
  AUpdDatasetProc(OrgDs);
  OrgDs.CancelUpdates;

  ChangedDs := DBConnector.GetNDataset(true,15) as TCustomBufDataset;
  ChangedDs.Open;
  CompareDatasets(OrgDs,ChangedDs);
end;

procedure TTestBufDatasetStreams.TestChanges(AUpdDatasetProc: TUpdDatasetProc;
  AFormat: TDataPacketFormat);
var FileName: string;
    SaveDs,
    LoadDs : TCustomBufDataset;
begin
  FileName := TestFileNames[AFormat];

  SaveDs := DBConnector.GetNDataset(true,15) as TCustomBufDataset;
  SaveDs.Open;
  AUpdDatasetProc(SaveDs);
  SaveDs.SaveToFile(FileName, AFormat);

  LoadDs := TMyCustomBufDataset.Create(nil);
  LoadDs.LoadFromFile(FileName);
  CompareDatasets(SaveDs,LoadDs);
  SaveDs.Close;

  SaveDs.Open;
  LoadDs.CancelUpdates;
  CompareDatasets(SaveDs,LoadDs);
  SaveDs.Close;

  LoadDs.Free;
  DeleteFile(FileName);
end;

procedure TTestBufDatasetStreams.TestChangesXML(AUpdDatasetProc: TUpdDatasetProc);
begin
  TestChanges(AUpdDatasetProc, dfXML);
end;


procedure TTestBufDatasetStreams.SimpleEditChange(ADataset: TCustomBufDataset);
begin
  ADataset.next;
  ADataset.edit;
  ADataset.FieldByName('name').AsString:='jojo';
  ADataset.Post;
end;

procedure TTestBufDatasetStreams.SimpleDeleteChange(ADataset: TCustomBufDataset);
begin
  ADataset.Next;
  ADataset.Delete;
end;

procedure TTestBufDatasetStreams.MoreDeletesChange(ADataset: TCustomBufDataset);
begin
  with ADataset do
    begin
    Delete;
    Next;
    // Next is to test what happens if the delete-buffer of one record is linked
    // to another deleted record
    Delete;
    Delete;
    Next;
    Next;
    Next;
    Next;
    // Next is to test two delete-update buffers which are linked to one record
    Delete;
    Prior;
    Delete;
    Last;
    Delete;
    end;
end;

procedure TTestBufDatasetStreams.SimpleInsertChange(ADataset: TCustomBufDataset);
begin
  ADataset.next;
  ADataset.insert;
  ADataset.FieldByName('id').AsInteger:=500;
  ADataset.FieldByName('name').AsString:='TestName0500';
  ADataset.Post;
end;

procedure TTestBufDatasetStreams.MoreInsertsChange(ADataset: TCustomBufDataset);
begin
  with ADataset do
    begin
    insert;
    FieldByName('id').AsInteger:=501;
    FieldByName('name').AsString:='TestName0501';
    Post;

    next;
    next;
    insert;
    FieldByName('id').AsInteger:=502;
    FieldByName('name').AsString:='TestName0502';
    Post;

    next;
    insert;
    FieldByName('id').AsInteger:=503;
    FieldByName('name').AsString:='TestName0503';
    Post;

    next;
    insert;
    FieldByName('id').AsInteger:=504;
    FieldByName('name').AsString:='TestName0504';
    Post;
    insert;
    FieldByName('id').AsInteger:=505;
    FieldByName('name').AsString:='TestName0505';
    Post;

    last;
    insert;
    FieldByName('id').AsInteger:=506;
    FieldByName('name').AsString:='TestName0506';
    Post;

    Append;
    FieldByName('id').AsInteger:=507;
    FieldByName('name').AsString:='TestName0507';
    Post;
    end;
end;

procedure TTestBufDatasetStreams.SeveralEditsChange(ADataset: TCustomBufDataset);
begin
  with ADataset do
    begin
    edit;
    fieldbyname('id').asinteger := 500;
    fieldbyname('name').AsString := 'JoJo';
    post;

    next; next;
    Delete;
    Delete;

    next;next;
    insert;
    fieldbyname('id').asinteger := 501;
    post;
    end;
end;

procedure TTestBufDatasetStreams.DeleteAllChange(ADataset: TCustomBufDataset);
begin
  with ADataset do
    while not eof do delete;
end;

procedure TTestBufDatasetStreams.DeleteAllInsertChange(ADataset: TCustomBufDataset);
begin
  DeleteAllChange(ADataset);
  with ADataset do
    begin
    insert;
    fieldbyname('ID').AsInteger:=5;
    post;
    end;
end;

procedure TTestBufDatasetStreams.NullInsertChange(ADataset: TCustomBufDataset);
begin
  with ADataset do
  begin
    AssertTrue(Locate('ID',11,[]));
    Delete; //11
    Delete; //12
    Delete; //13
    Delete; //14
    Append;
    FieldByName('ID').AsInteger:=11;
    //FieldByName('NAME').Clear;
    Post;
    AppendRecord([12,'AfterNull']);
    AppendRecord([13,null]);
    AppendRecord([14,'AfterNull']);
    //Append; Post;
  end;
end;

procedure TTestBufDatasetStreams.NullEditChange(ADataset: TCustomBufDataset);
var i: integer;
begin
  //depends on procedure TTestBufDatasetStreams.NullInsertChange
  if ADataSet is TSQLQuery then
    with ADataset as TSQLQuery do
    begin
      AssertTrue(Locate('ID',11,[]));
      for i:=11 to 14 do
      begin
        Edit;
        FieldByName('NAME').AsString:='TestName'+inttostr(i);
        Post;
        Next;
      end;
      UpdateMode:=upWhereAll; //test when also null fields will be in where
    end;
end;

procedure TTestBufDatasetStreams.AppendDeleteChange(ADataset: TCustomBufDataset);
begin
  // Tests bugs #19593, #21994
  with ADataset do
  begin
    AppendRecord([16,'TestName16']);
    AppendRecord([17,'TestName17']);
    Prior;
    Prior;
    Delete;  // 15 update-buffer of deleted record is linked to 16
    Delete;  // 16 inserted-deleted and linked by 15
    AppendRecord([18,'TestName18']); // again append after delete
  end;
end;

procedure TTestBufDatasetStreams.TestSimpleEditCancelUpd;
begin
  TestChangesCancelUpdates(@SimpleEditChange);
end;

procedure TTestBufDatasetStreams.TestSimpleDeleteCancelUpd;
begin
  TestChangesCancelUpdates(@SimpleDeleteChange);
end;

procedure TTestBufDatasetStreams.TestMoreDeletesCancelUpd;
begin
  TestChangesCancelUpdates(@MoreDeletesChange);
end;

procedure TTestBufDatasetStreams.TestSimpleInsertCancelUpd;
begin
  TestChangesCancelUpdates(@SimpleInsertChange);
end;

procedure TTestBufDatasetStreams.MoreInsertsCancelUpd;
begin
  TestChangesCancelUpdates(@MoreInsertsChange);
end;

procedure TTestBufDatasetStreams.SeveralEditsCancelUpd;
begin
  TestChangesCancelUpdates(@SeveralEditsChange);
end;

procedure TTestBufDatasetStreams.DeleteAllCancelUpd;
begin
  TestChangesCancelUpdates(@DeleteAllChange);
end;

procedure TTestBufDatasetStreams.DeleteAllInsertCancelUpd;
begin
  TestChangesCancelUpdates(@DeleteAllInsertChange);
end;

procedure TTestBufDatasetStreams.AppendDeleteCancelUpd;
begin
  TestChangesCancelUpdates(@AppendDeleteChange);
end;

procedure TTestBufDatasetStreams.TestBasicsXML;
var SaveDs: TCustomBufDataset;
    LoadDs: TCustomBufDataset;
begin
  SaveDs := DBConnector.GetNDataset(true,15) as TCustomBufDataset;
  SaveDs.Open;
  SaveDs.SaveToFile(TestXMLFileName, dfXML);

  LoadDs := TMyCustomBufDataset.Create(nil);
  LoadDs.LoadFromFile(TestXMLFileName);
  CompareDatasets(SaveDs,LoadDs);
  LoadDs.Free;
end;

procedure TTestBufDatasetStreams.TestSimpleEditXML;
begin
  TestChangesXML(@SimpleEditChange);
end;

procedure TTestBufDatasetStreams.TestSimpleDeleteXML;
begin
  TestChangesXML(@SimpleDeleteChange);
end;

procedure TTestBufDatasetStreams.TestMoreDeletesXML;
begin
  TestChangesXML(@MoreDeletesChange);
end;

procedure TTestBufDatasetStreams.TestSimpleInsertXML;
begin
  TestChangesXML(@SimpleInsertChange);
end;

procedure TTestBufDatasetStreams.TestMoreInsertsXML;
begin
  TestChangesXML(@MoreInsertsChange);
end;

procedure TTestBufDatasetStreams.TestSeveralEditsXML;
begin
  TestChangesXML(@SeveralEditsChange);
end;

procedure TTestBufDatasetStreams.TestDeleteAllXML;
begin
  TestChangesXML(@DeleteAllChange);
end;

procedure TTestBufDatasetStreams.TestDeleteAllInsertXML;
begin
  TestChangesXML(@DeleteAllInsertChange);
end;

procedure TTestBufDatasetStreams.TestStreamingDataFields(AFormat: TDataPacketFormat);
var SaveDs: TCustomBufDataset;
    LoadDs: TCustomBufDataset;
    i: integer;
begin
  SaveDs := DBConnector.GetFieldDataset as TCustomBufDataset;
  SaveDs.Open;
  SaveDs.SaveToFile(TestFileNames[AFormat], AFormat);

  LoadDs := TMyCustomBufDataset.Create(nil);
  LoadDs.LoadFromFile(TestFileNames[AFormat]);
  AssertEquals(SaveDs.FieldCount, LoadDs.FieldCount);

  LoadDS.First;
  SaveDS.First;
  while not LoadDS.EOF do
    begin
    for i:=0 to LoadDS.FieldCount-1 do
      // all FieldTypes supports GetAsString
      AssertEquals(LoadDS.Fields[i].FieldName, SaveDS.Fields[i].AsString, LoadDS.Fields[i].AsString);
    LoadDS.Next;
    SaveDS.Next;
    end;

  LoadDs.Free;
end;

procedure TTestBufDatasetStreams.TestStreamingDataFieldsBIN;
begin
  TestStreamingDataFields(dfBinary);
end;

procedure TTestBufDatasetStreams.TestStreamingDataFieldsXML;
begin
  TestStreamingDataFields(dfXML);
end;

procedure TTestBufDatasetStreams.TestStreamingBigBlobFieldsXML;
var
  SaveDs: TCustomBufDataset;
  LoadDs: TCustomBufDataset;
  j: integer;
  i: byte;
  s: string;
  f: file of byte;
  fn: string;
  fs: TMemoryStream;
begin
  // Create a temp. file with blob-data.
  fn := GetTempFileName;
  assign(f,fn);
  Rewrite(f);
  s := 'This is a blob-field test file.';
  for j := 0 to 250 do
    begin
    for i := 1 to length(s) do
      write(f,ord(s[i]));
    for i := 0 to 255 do
      write(f,i);
    end;
  close(f);

  try
    // Open dataset and set blob-field-data to content of blob-file.
    SaveDs := DBConnector.GetFieldDataset(true) as TCustomBufDataset;
    SaveDs.Open;
    SaveDs.Edit;
    TBlobField(SaveDs.FieldByName('FBLOB')).LoadFromFile(fn);
    SaveDs.Post;

    // Save this dataset to file.
    SaveDs.SaveToFile(TestXMLFileName,dfXML);

    // Load this file in another dataset
    LoadDs := TMyCustomBufDataset.Create(nil);
    try
      LoadDs.LoadFromFile(TestXMLFileName);
      LoadDS.First;

      // Compare the content of the blob-field with the file on disc
      fs := TMemoryStream.Create;
      try
        TBlobField(SaveDs.FieldByName('FBLOB')).SaveToStream(fs);
        fs.Seek(0,soBeginning);
        assign(f,fn);
        reset(f);
        for j := 0 to fs.Size-1 do
          begin
          read(f,i);
          CheckEquals(i,fs.ReadByte);
          end;
      finally
        fs.free;
      end;
    finally
      LoadDs.Free;
    end;
  finally
    DeleteFile(fn);
  end;
end;

procedure TTestBufDatasetStreams.TestStreamingNullFields(AFormat: TDataPacketFormat);
var
  SaveDs: TCustomBufDataset;
  LoadDs: TCustomBufDataset;
  i: integer;
begin
  SaveDs := DBConnector.GetFieldDataset(true) as TCustomBufDataset;
  with SaveDs do
    begin
    Open;
    Next;
    Edit;
    // set all fields, which are not required to null
    for i:=0 to FieldCount-1 do
      if not Fields[i].Required and not Fields[i].ReadOnly then
        Fields[i].Clear;
    Post;
    // check if they are null
    for i:=0 to FieldCount-1 do
      if not Fields[i].Required and not Fields[i].ReadOnly then
        AssertTrue(Fields[i].FieldName, Fields[i].IsNull);
    SaveToFile(TestFileNames[AFormat], AFormat);
    end;

  LoadDs := TMyCustomBufDataset.Create(nil);
  try
    LoadDs.LoadFromFile(TestFileNames[AFormat]);
    AssertEquals(SaveDs.FieldCount, LoadDs.FieldCount);
    SaveDs.First;
    while not SaveDs.EOF do
      begin
      for i:=0 to SaveDs.FieldCount-1 do
        AssertEquals(SaveDs.Fields[i].FieldName, SaveDs.Fields[i].IsNull, LoadDs.Fields[i].IsNull);
      LoadDs.Next;
      SaveDs.Next;
      end;
  finally
    LoadDs.Free;
  end;
end;

procedure TTestBufDatasetStreams.TestStreamingNullFieldsBIN;
begin
  TestStreamingNullFields(dfBinary);
end;

procedure TTestBufDatasetStreams.TestStreamingNullFieldsXML;
begin
  TestStreamingNullFields(dfXML);
end;

procedure TTestBufDatasetStreams.TestStreamingCalculatedFieldsXML;
var
  ADataset: TCustomBufDataset;
  f: tfield;
begin
  ADataset := DBConnector.GetNDataset(true,10) as TCustomBufDataset;
  f := TIntegerField.Create(ADataset);
  f.FieldName:='ID';
  f.dataset := ADataset;

  f := TIntegerField.Create(ADataset);
  f.FieldName:='CalcID';
  f.dataset := ADataset;
  f.FieldKind:=fkCalculated;

  f := TStringField.Create(ADataset);
  f.FieldName:='NAME';
  f.dataset := ADataset;

  ADataset.Open;
  ADataset.SaveToFile(TestXMLFileName,dfXML);
  ADataset.Close;

  ADataset.LoadFromFile(TestXMLFileName,dfXML);
  AssertEquals(1, ADataset.FieldByName('ID').AsInteger);
  AssertEquals('TestName1', ADataset.FieldByName('NAME').AsString);
  ADataset.Close;
end;

procedure TTestBufDatasetStreams.TestAppendDeleteBIN;
begin
  TestChanges(@AppendDeleteChange);
end;

procedure TTestBufDatasetStreams.TestFileNameProperty;
var ds    : TDataset;
    LoadDs: TDataset;
begin
  ds := DBConnector.GetNDataset(true,5);

  ds.open;
  TCustomBufDataset(ds).FileName:=TestBINFileName;
  ds.close;

  LoadDs := DBConnector.GetNDataset(True,2);
  TCustomBufDataset(LoadDs).FileName:=TestBINFileName;
  LoadDs.Open;

  ds := DBConnector.GetNDataset(true,4);
  ds.Open;
  CompareDatasets(ds,LoadDs);
  ds.close;
  LoadDs.close;
end;

procedure TTestBufDatasetStreams.TestXmlFileRecognition;
var ds    : TDataset;
    LoadDs: TDataset;
begin
  ds := DBConnector.GetNDataset(true,5);

  ds.open;
  TCustomBufDataset(ds).SaveToFile(TestXMLFileName,dfXML);
  ds.close;

  LoadDs := DBConnector.GetNDataset(True,2);
  TCustomBufDataset(LoadDs).FileName:=TestXMLFileName;
  LoadDs.Open;

  ds := DBConnector.GetNDataset(true,4);
  ds.Open;
  CompareDatasets(ds,LoadDs);
  ds.close;
  LoadDs.close;
end;

procedure TTestBufDatasetStreams.TestCloseDatasetNoConnection;
var SaveDs: TCustomBufDataset;
    LoadDs: TCustomBufDataset;
    Conn: TSQLConnection;
begin
  SaveDs := DBConnector.GetNDataset(true,15) as TSQLQuery;
  SaveDs.Open;
  SaveDs.SaveToFile(TestXMLFileName,dfXML);
  SaveDs.Close;

  Conn := TSQLConnectionClass(TSQLDBConnector(DBConnector).Connection.ClassType).Create(nil);
  LoadDs := TSQLQuery.Create(nil);
  LoadDs.DataBase:=Conn;
  LoadDs.LoadFromFile(TestXMLFileName);
  LoadDs.Next;
  LoadDs.Close;
  LoadDs.Free;
  Conn.Free;
end;

procedure TTestBufDatasetStreams.TestSimpleEditApplUpd;
begin
  TestChangesApplyUpdates(@SimpleEditChange);
end;

procedure TTestBufDatasetStreams.TestSimpleDeleteApplUpd;
begin
  TestChangesApplyUpdates(@SimpleDeleteChange);
end;

procedure TTestBufDatasetStreams.TestMoreDeletesApplUpd;
begin
  TestChangesApplyUpdates(@MoreDeletesChange);
end;

procedure TTestBufDatasetStreams.TestSimpleInsertApplUpd;
begin
  TestChangesApplyUpdates(@SimpleInsertChange,True);
end;

procedure TTestBufDatasetStreams.MoreInsertsApplUpd;
begin
  TestChangesApplyUpdates(@MoreInsertsChange,True);
end;

procedure TTestBufDatasetStreams.SeveralEditsApplUpd;
begin
  TestChangesApplyUpdates(@SeveralEditsChange, True);
end;

procedure TTestBufDatasetStreams.DeleteAllApplUpd;
begin
  TestChangesApplyUpdates(@DeleteAllChange, False);
end;

procedure TTestBufDatasetStreams.DeleteAllInsertApplUpd;
begin
  TestChangesApplyUpdates(@DeleteAllInsertChange, False);
end;

procedure TTestBufDatasetStreams.NullInsertUpdateApplUpd;
begin
  TestChangesApplyUpdates(@NullInsertChange, True);
  TestChangesApplyUpdates(@NullEditChange, True);
end;


procedure TTestBufDatasetStreams.SetUp;
begin
  DBConnector.StartTest(TestName);
end;

procedure TTestBufDatasetStreams.TearDown;
begin
  DBConnector.StopTest(TestName);
end;


initialization
  if uppercase(dbconnectorname)='SQL' then
    RegisterTestDecorator(TDBBasicsTestSetup, TTestBufDatasetStreams);
end.
