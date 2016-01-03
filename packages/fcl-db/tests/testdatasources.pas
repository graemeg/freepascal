unit TestDatasources;

{$IFDEF FPC}
  {$mode Delphi}{$H+}
{$ENDIF}

interface

uses
  Classes, SysUtils, fpcunit, db;

type

  { TTestDatasources }

  TTestDatasources = class(TTestCase)
  private
    procedure FieldNotifyEvent(Sender: TField);
    procedure DatasetNotifyEvent(Dataset: TDataset);
  protected
    procedure SetUp; override;
    procedure TearDown; override;
  published
// This test is also in TestDBBasics
//    procedure TestDataEventsResync;
    procedure TestDataEvent1;
    procedure TestDataEvent2;
    procedure TestDataEvent3;
    procedure TestDataEvent4;
    procedure TestDataEvent5;
    procedure TestDataEvent6;
    procedure TestDataEvent7;
    procedure TestCalcFirstRecord1;
    procedure TestRefreshLookupList;
    procedure TestCalculateFields;
    procedure TestCalcLookupValue;
    procedure TestEnableControls;
  end;
  
implementation

uses ToolsUnit, dbf, testregistry, variants{$IFDEF UNIX},cwstring {$ENDIF};

type THackDataset=class(TDataset);
     THackDataLink=class(TDatalink);

{ TTestDataSources }

procedure TTestDatasources.FieldNotifyEvent(Sender: TField);
begin
  DataEvents := DataEvents + 'FieldNotifyEvent' + ';';
end;

procedure TTestDatasources.DatasetNotifyEvent(Dataset: TDataset);
begin
  DataEvents := DataEvents + 'DatasetNotifyEvent' + ';';
end;

procedure TTestDatasources.SetUp;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  DBConnector.StartTest(TestName);
=======
  DBConnector.StartTest;
>>>>>>> graemeg/fixes_2_2
=======
  DBConnector.StartTest;
>>>>>>> origin/fixes_2_2
end;

procedure TTestDatasources.TearDown;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  DBConnector.StopTest(TestName);
=======
  DBConnector.StopTest;
>>>>>>> graemeg/fixes_2_2
=======
  DBConnector.StopTest;
>>>>>>> origin/fixes_2_2
end;

{procedure TTestDatasources.TestDataEventsResync;
var i,count     : integer;
    aDatasource : TDataSource;
    aDatalink   : TDataLink;
    ds          : tdataset;
begin
  aDatasource := TDataSource.Create(nil);
  aDatalink := TTestDataLink.Create;
  aDatalink.DataSource := aDatasource;
  ds := DBConnector.GetNDataset(6);
  ds.BeforeScroll := DBConnector.DataEvent;
  with ds do
    begin
    aDatasource.DataSet := ds;
    open;
    DataEvents := '';
    Resync([rmExact]);
    AssertEquals('deDataSetChange:0;',DataEvents);
    DataEvents := '';
    next;
    AssertEquals('deCheckBrowseMode:0;DataEvent;deDataSetScroll:0;',DataEvents);
    close;
    end;
  aDatasource.Free;
  aDatalink.Free;
end;}

procedure TTestDatasources.TestDataEvent1;
var i,count     : integer;
    aDatasource : TDataSource;
    aDatalink1,
    aDatalink2  : TDataLink;
    ds          : tdataset;
begin
  aDatasource := TDataSource.Create(nil);
  aDatalink1 := TTestDataLink.Create;
  aDatalink1.DataSource := aDatasource;
  ds := DBConnector.GetNDataset(6);
  with ds do
    begin
    aDatasource.DataSet := ds;
    open;
    DataEvents := '';
    THackDataset(ds).DataEvent(deCheckBrowseMode,0);
    AssertEquals('deCheckBrowseMode:0;',DataEvents);

    aDatalink2 := TTestDataLink.Create;
    aDatalink2.DataSource := aDatasource;

    DataEvents := '';
    THackDataset(ds).DataEvent(deCheckBrowseMode,0);
    AssertEquals('deCheckBrowseMode:0;deCheckBrowseMode:0;',DataEvents);

    aDatalink2.free;
    DataEvents := '';
    THackDataset(ds).DataEvent(deCheckBrowseMode,0);
    AssertEquals('deCheckBrowseMode:0;',DataEvents);

    close;
    end;
end;

procedure TTestDatasources.TestDataEvent2;
var aDatasource : TDataSource;
    aDatalink   : TDataLink;
    ds          : tdataset;
begin
  aDatasource := TDataSource.Create(nil);
  aDatalink := TTestDataLink.Create;
  aDatalink.DataSource := aDatasource;
  ds := DBConnector.GetTraceDataset(false);
  with ds do
    begin
    aDatasource.DataSet := ds;
    open;
    // The deDataSetChange and deDataSetScroll events should trigger a call to
    // TDataset.UpdateCursorPos...
    DataEvents := '';
    THackDataset(ds).DataEvent(deDataSetChange,0);
    AssertEquals('SetCurrentRecord;deDataSetChange:0;',DataEvents);

    DataEvents := '';
    THackDataset(ds).DataEvent(deDataSetScroll,0);
    AssertEquals('SetCurrentRecord;deDataSetScroll:0;DataSetScrolled:0;',DataEvents);
    
    // unless TDataset.State is dsInsert
    
    ds.insert;
    DataEvents := '';
    AssertTrue(ds.State=dsInsert);
    THackDataset(ds).DataEvent(deDataSetChange,0);
    AssertEquals('deDataSetChange:0;',DataEvents);

    AssertTrue(ds.State=dsInsert);
    DataEvents := '';
    THackDataset(ds).DataEvent(deDataSetScroll,0);
    AssertEquals('deDataSetScroll:0;DataSetScrolled:0;',DataEvents);
    end;
end;

procedure TTestDatasources.TestDataEvent3;
var aDatasource : TDataSource;
    aDatalink   : TDataLink;
    ds          : tdataset;
    AFld        : TField;
begin
  aDatasource := TDataSource.Create(nil);
  aDatalink := TTestDataLink.Create;
  aDatalink.DataSource := aDatasource;
  ds := DBConnector.GetTraceDataset(false);
  with ds do
    begin
    aDatasource.DataSet := ds;
    open;
    AFld := FieldByName('id');
    // On a deFieldChange event from a field with a fieldkind of fkData or
    // fkInternalCalc, TDataset.Modified must be set to true
    DataEvents := '';
    AssertFalse(Modified);
    THackDataset(ds).DataEvent(deFieldChange,PtrInt(AFld));
    AssertTrue(Modified);
    AssertEquals('deFieldChange:ID;',DataEvents);

    Close;
    AFld := TIntegerField.Create(ds);
    AFld.FieldName := 'CALCFLD';
    AFld.DataSet := ds;
    Afld.FieldKind := fkCalculated;
    Open;

    DataEvents := '';
    AssertFalse(Modified);
    THackDataset(ds).DataEvent(deFieldChange,PtrInt(AFld));
    AssertFalse(Modified);
    AssertEquals('deFieldChange:CALCFLD;',DataEvents);
    end;
end;

procedure TTestDatasources.TestDataEvent4;
var aDatasource : TDataSource;
    aDatalink   : TDataLink;
    ds          : tdataset;
    AFld        : TField;
begin
  aDatasource := TDataSource.Create(nil);
  aDatalink := TTestDataLink.Create;
  aDatalink.DataSource := aDatasource;
  ds := DBConnector.GetTraceDataset(false);
  with ds do
    begin
    aDatasource.DataSet := ds;
    
    // Ugly hack to imitate InternalCalcField, see
    // TDbfTraceDataset.InternalInitFieldDefs
    FieldDefs.Add('Name',ftString);
    FieldDefs.Find('Name').InternalCalcField:=True;
    open;
    AssertTrue(THackDataset(ds).InternalCalcFields);
    // If there are InternalCalcFields (InternalCalcFields=True) and the fieldkind
    // of the field from the deFieldChange event is fkData, then
    // RefreshIntenralCalcFields is called
    AFld := FieldByName('id');
    DataEvents := '';
    THackDataset(ds).DataEvent(deFieldChange,PtrInt(AFld));
    AssertEquals('RefreshInternalCalcFields;deFieldChange:ID;',DataEvents);

    AFld := FieldByName('name');
    AFld.FieldKind:=fkInternalCalc;
    DataEvents := '';
    THackDataset(ds).DataEvent(deFieldChange,PtrInt(AFld));
    AssertEquals('deFieldChange:NAME;',DataEvents);

    // If the TDataset.State is dsSetKey then IntenralCalcFields shoudn't get called
    THackDataset(ds).SetState(dsSetKey);
    AFld := FieldByName('id');
    DataEvents := '';
    THackDataset(ds).DataEvent(deFieldChange,PtrInt(AFld));
    AssertEquals('deFieldChange:ID;',DataEvents);
    end;
end;

procedure TTestDatasources.TestDataEvent5;
var aDatasource : TDataSource;
    aDatalink   : TDataLink;
    ds          : tdataset;
    AFld        : TField;
begin
  aDatasource := TDataSource.Create(nil);
  aDatalink := TTestDataLink.Create;
  aDatalink.DataSource := aDatasource;
  ds := DBConnector.GetTraceDataset(false);
  with ds do
    begin
    aDatasource.DataSet := ds;
    open;
    AFld := FieldByName('id');
    AFld.OnChange:=FieldNotifyEvent;
    // When TDataset.State is not dsSetKey then TField.Change is called on a
    // deFieldChange event
    DataEvents := '';
    THackDataset(ds).DataEvent(deFieldChange,PtrInt(AFld));
    AssertEquals('FieldNotifyEvent;deFieldChange:ID;',DataEvents);

    THackDataset(ds).SetState(dsSetKey);
    DataEvents := '';
    THackDataset(ds).DataEvent(deFieldChange,PtrInt(AFld));
    AssertEquals('deFieldChange:ID;',DataEvents);
    end;
end;

procedure TTestDatasources.TestDataEvent6;
var aDatasource : TDataSource;
    aDatalink   : TDataLink;
    ds          : tdataset;
    AFld        : TField;
begin
  aDatasource := TDataSource.Create(nil);
  aDatalink := TTestDataLink.Create;
  aDatalink.DataSource := aDatasource;
  ds := DBConnector.GetTraceDataset(false);
  with ds do
    begin
    aDatasource.DataSet := ds;

    AFld := TIntegerField.Create(ds);
    AFld.FieldName := 'ID';
    AFld.DataSet := ds;

    AFld := TStringField.Create(ds);
    AFld.FieldName := 'NAME';
    AFld.DataSet := ds;

    AFld := TIntegerField.Create(ds);
    AFld.FieldName := 'CALCFLD';
    AFld.DataSet := ds;
    Afld.FieldKind := fkCalculated;

    open;
    // If there are Calculated fields and AutoCalcFields is true, then call
    // CalculateFields in case of a deFieldChange event, if the fields fieldkind
    // is fkData
    AFld := FieldByName('id');
    DataEvents := '';
    THackDataset(ds).DataEvent(deFieldChange,PtrInt(AFld));
    AssertEquals('deFieldChange:ID;',DataEvents);

    DataEvents := '';
    AutoCalcFields:=True;
    THackDataset(ds).DataEvent(deFieldChange,PtrInt(AFld));
    AssertEquals('CalculateFields;ClearCalcFields;deFieldChange:ID;',DataEvents);

    AFld := FieldByName('calcfld');
    DataEvents := '';
    THackDataset(ds).DataEvent(deFieldChange,PtrInt(AFld));
    AssertEquals('deFieldChange:CALCFLD;',DataEvents);

    // If the TDataset.State is dsSetKey then CalculateFields shoudn't get called
    THackDataset(ds).SetState(dsSetKey);
    AFld := FieldByName('id');
    DataEvents := '';
    THackDataset(ds).DataEvent(deFieldChange,PtrInt(AFld));
    AssertEquals('deFieldChange:ID;',DataEvents);
    end;
end;

procedure TTestDatasources.TestDataEvent7;
var aDatasource : TDataSource;
    aDatalink   : TDataLink;
    ds          : tdataset;
    AFld        : TField;
begin
  aDatasource := TDataSource.Create(nil);
  aDatalink := TTestDataLink.Create;
  aDatalink.DataSource := aDatasource;
  ds := DBConnector.GetTraceDataset(false);
  with ds do
    begin
    aDatasource.DataSet := ds;

    AFld := TIntegerField.Create(ds);
    AFld.FieldName := 'ID';
    AFld.DataSet := ds;
    
    AFld := TStringField.Create(ds);
    AFld.FieldName := 'NAME';
    AFld.DataSet := ds;

    AFld := TIntegerField.Create(ds);
    AFld.FieldName := 'CALCFLD';
    AFld.DataSet := ds;
    Afld.FieldKind := fkCalculated;

    // Ugly hack to imitate InternalCalcField, see
    // TDbfTraceDataset.InternalInitFieldDefs
    FieldDefs.Add('Name',ftString);
    FieldDefs.Find('Name').InternalCalcField:=True;
    open;
    AssertTrue(THackDataset(ds).InternalCalcFields);
    // If there are InternalCalcFields and 'normal' Calculated fields, only
<<<<<<< HEAD
<<<<<<< HEAD
    // RefreshInternalCalcFields is called
=======
    // RefreshIntenralCalcFields is called
>>>>>>> graemeg/fixes_2_2
=======
    // RefreshIntenralCalcFields is called
>>>>>>> origin/fixes_2_2
    AFld := FieldByName('id');
    DataEvents := '';
    THackDataset(ds).DataEvent(deFieldChange,PtrInt(AFld));
    AssertEquals('RefreshInternalCalcFields;deFieldChange:ID;',DataEvents);

    AFld := FieldByName('name');
    AFld.FieldKind:=fkInternalCalc;
    DataEvents := '';
    THackDataset(ds).DataEvent(deFieldChange,PtrInt(AFld));
    AssertEquals('deFieldChange:NAME;',DataEvents);

<<<<<<< HEAD
<<<<<<< HEAD
    // If the TDataset.State is dsSetKey then InternalCalcFields shoudn't get called
=======
    // If the TDataset.State is dsSetKey then IntenralCalcFields shoudn't get called
>>>>>>> graemeg/fixes_2_2
=======
    // If the TDataset.State is dsSetKey then IntenralCalcFields shoudn't get called
>>>>>>> origin/fixes_2_2
    THackDataset(ds).SetState(dsSetKey);
    AFld := FieldByName('id');
    DataEvents := '';
    THackDataset(ds).DataEvent(deFieldChange,PtrInt(AFld));
    AssertEquals('deFieldChange:ID;',DataEvents);
    end;
end;

procedure TTestDatasources.TestCalcFirstRecord1;
var aDatasource : TDataSource;
    aDatalink   : TDataLink;
    ds          : tdataset;
    FirstRec    : Integer;
begin
  aDatasource := TDataSource.Create(nil);
  aDatalink := TTestDataLink.Create;
  aDatalink.DataSource := aDatasource;
  ds := DBConnector.GetNDataset(15);
  aDatasource.DataSet := ds;
  with ds do
    begin
    open;
    FirstRec := THackDataLink(aDatalink).FirstRecord;
    
    // Scroll '0' records, FirstRecord should stay the same,
    // and the there's no need to scroll the buffer.
    DataEvents := '';
    THackDataset(ds).DataEvent(deDataSetScroll,0);
    AssertEquals('deDataSetScroll:0;DataSetScrolled:0;',DataEvents);
    AssertEquals(FirstRec,THackDataLink(aDatalink).FirstRecord);

    // Scroll 1 record forward, FirstRecord should stay the same,
    // but the buffer is scrolled one place back.
    DataEvents := '';
    THackDataset(ds).DataEvent(deDataSetScroll,1);
    AssertEquals('deDataSetScroll:1;DataSetScrolled:-1;',DataEvents);
    AssertEquals(FirstRec,THackDataLink(aDatalink).FirstRecord);


    // Scroll 1 record backward, FirstRecord should stay the same,
    // but the buffer is scrolled one place back.
    DataEvents := '';
    THackDataset(ds).DataEvent(deDataSetScroll,-1);
    AssertEquals('deDataSetScroll:-1;DataSetScrolled:1;',DataEvents);
    AssertEquals(FirstRec,THackDataLink(aDatalink).FirstRecord);

    // Remove the datasource.
    aDatasource.DataSet := nil;
    DataEvents := '';
    THackDataset(ds).DataEvent(deDataSetScroll,1);
    AssertEquals('',DataEvents);
    
    // Set the buffer-size to 5 and add it to the dataset again
    aDatalink.BufferCount:=5;
    aDatasource.DataSet := ds;
    
    // Scroll '0' records, firstrecord should stay the same again,
    // and there's no need to scroll the buffer.
    DataEvents := '';
    THackDataset(ds).DataEvent(deDataSetScroll,0);
    AssertEquals('deDataSetScroll:0;DataSetScrolled:0;',DataEvents);
    AssertEquals(FirstRec,THackDataLink(aDatalink).FirstRecord);

    // Scroll 1 record backwards with a buffer size of 5.
    // Now the buffer won't scroll, but FirstRecord is decremented
    DataEvents := '';
    THackDataset(ds).DataEvent(deDataSetScroll,-1);
    AssertEquals('deDataSetScroll:-1;DataSetScrolled:0;',DataEvents);
    dec(FirstRec);
    AssertEquals(FirstRec,THackDataLink(aDatalink).FirstRecord);

    // Scroll one record forward again, no buffer scroll, FirstRecord
    // is inremented
    DataEvents := '';
    THackDataset(ds).DataEvent(deDataSetScroll,1);
    AssertEquals('deDataSetScroll:1;DataSetScrolled:0;',DataEvents);
    inc(FirstRec);
    AssertEquals(FirstRec,THackDataLink(aDatalink).FirstRecord);

    // Scroll one more record forward, buffer will scroll, FirstRecord
    // stays constant
    DataEvents := '';
    THackDataset(ds).DataEvent(deDataSetScroll,1);
    AssertEquals('deDataSetScroll:1;DataSetScrolled:-1;',DataEvents);
    AssertEquals(FirstRec,THackDataLink(aDatalink).FirstRecord);
    
    // Scroll two records backward, no buffer scroll, FirstRecord
    // is inremented twice
    DataEvents := '';
    THackDataset(ds).DataEvent(deDataSetScroll,-2);
    AssertEquals('deDataSetScroll:-2;DataSetScrolled:0;',DataEvents);
    dec(FirstRec,2);
    AssertEquals(FirstRec,THackDataLink(aDatalink).FirstRecord);

    // Scroll 6 records forward, so the buffer is scrolled 4 positions backward
    // and FirstRecord is Incremented by 2
    DataEvents := '';
    THackDataset(ds).DataEvent(deDataSetScroll,6);
    AssertEquals('deDataSetScroll:6;DataSetScrolled:-4;',DataEvents);
    inc(FirstRec,2);
    AssertEquals(FirstRec,THackDataLink(aDatalink).FirstRecord);

    // The other way around, scroll 6 records back, so the buffer is scrolled 2
    // positions forward and FirstRecord is decremented by 4
    DataEvents := '';
    THackDataset(ds).DataEvent(deDataSetScroll,-6);
    AssertEquals('deDataSetScroll:-6;DataSetScrolled:2;',DataEvents);
    dec(FirstRec,4);
    AssertEquals(FirstRec,THackDataLink(aDatalink).FirstRecord);

    end;
end;

procedure TTestDatasources.TestRefreshLookupList;
var ds, lkpDs   : TDataset;
    AFld1, AFld2, AFld3 : Tfield;
    Var1,Var2 : Variant;
    
  procedure TestLookupList;
  begin
    lkpDs.Open;
    lkpDs.first;
    while not LkpDs.eof do with AFld3 do
      begin
      Var1 := LkpDs.FieldValues[LookupResultField];
      Var2 := LookupList.ValueOfKey(LkpDs.fieldvalues[LookupKeyFields]);
      AssertEquals(VarToStr(Var1),VarToStr(Var2));
      lkpDs.Next;
      end;
  end;
begin
  ds := DBConnector.GetNDataset(15);
  lkpDs := DBConnector.GetNDataset(5);
  with ds do
    begin
    AFld1 := TIntegerField.Create(ds);
    AFld1.FieldName := 'ID';
    AFld1.DataSet := ds;

    AFld2 := TStringField.Create(ds);
    AFld2.FieldName := 'NAME';
    AFld2.DataSet := ds;

    AFld3 := TIntegerField.Create(ds);
    with AFld3 do
      begin
      // Test if nothing happens when not all properties are filled
      FieldName := 'LookupFld';
      FieldKind := fkLookup;
      DataSet := ds;
      RefreshLookupList;
      LookupDataSet := lkpDs;
      RefreshLookupList;
      LookupKeyFields:='name';
      RefreshLookupList;
      LookupResultField:='ID';
      RefreshLookupList;
      KeyFields:='name';
      // Everything is filled in, this should run wihout any problems:
      RefreshLookupList;
      // The lookupdataset was closed, and should be closed again:
      AssertFalse(lkpDs.Active);

      // If some fields don't exist, check if an exception is raised:
      LookupKeyFields:='faulty';
      AssertException(EDatabaseError,RefreshLookupList);
      LookupKeyFields:='name';

      LookupResultField :='faulty';
      AssertException(EDatabaseError,RefreshLookupList);
      LookupResultField :='ID';

      // Check if the lookuplist is correctly filled
      RefreshLookupList;
      TestLookupList;

      // Check if the lookuplist is correctly filled when there are multiple
      // fields in the key
      LookupResultField:='name';
      LookupKeyFields:='id;name';
      RefreshLookupList;
      TestLookupList;
      end;
    AFld1.Free;
    AFld2.Free;
    AFld3.Free;
    end;
end;

procedure TTestDatasources.TestCalculateFields;
var ds, lkpDs   : TDataset;
    AFld1, AFld2, AFld3 : Tfield;
    StoreValue : Variant;
    Buffer: pchar;
begin
  ds := DBConnector.GetTraceDataset(True);
  lkpDs := DBConnector.GetNDataset(5);
  with ds do
    begin
    AFld1 := TIntegerField.Create(ds);
    AFld1.FieldName := 'ID';
    AFld1.DataSet := ds;

    AFld2 := TStringField.Create(ds);
    AFld2.FieldName := 'NAME';
    AFld2.DataSet := ds;

    AFld3 := TIntegerField.Create(ds);
    with AFld3 do
      begin
      FieldName := 'LookupFld';
      FieldKind := fkLookup;
      DataSet := ds;
      LookupDataSet := lkpDs;
      LookupKeyFields:='name';
      LookupResultField:='ID';
      KeyFields := 'name';
      end;
    ds.OnCalcFields:=DatasetNotifyEvent;
    lkpds.Open;
    open;
    Buffer:=ds.ActiveBuffer;

    // If the state is dsInternalCalc, only the OnCalcField event should be called
    THackDataset(ds).SetState(dsInternalCalc);
    DataEvents:='';
    StoreValue:=AFld3.Value;
    THackDataset(ds).CalculateFields(Buffer);
    AssertEquals('CalculateFields;DatasetNotifyEvent;',DataEvents);
    AssertEquals(VarToStr(StoreValue),VarToSTr(AFld3.Value));
    THackDataset(ds).SetState(dsBrowse);

    // Also if the dataset is Unidirectional, only the OnCalcField event should be called
    THackDataset(ds).SetUniDirectional(True);
    DataEvents:='';
    StoreValue:=AFld3.Value;
    THackDataset(ds).CalculateFields(Buffer);
    AssertEquals('CalculateFields;DatasetNotifyEvent;',DataEvents);
    AssertEquals(VarToStr(StoreValue),VarToSTr(AFld3.Value));
    THackDataset(ds).SetUniDirectional(False);

    // Else, the value of all the lookup fields should get calculated
    edit;
    FieldByName('name').asstring := 'TestName3';
    post;
    DataEvents:='';
    THackDataset(ds).CalculateFields(Buffer);
    AssertEquals('CalculateFields;ClearCalcFields;DatasetNotifyEvent;',DataEvents);
    AssertEquals('3',VarToStr(AFld3.Value));
    end;
end;

procedure TTestDatasources.TestCalcLookupValue;
var ds, lkpDs   : TDataset;
    AFld1, AFld2, AFld3 : Tfield;
    Buffer: pchar;
begin
  ds := DBConnector.GetNDataset(True,15);
  lkpDs := DBConnector.GetNDataset(5);
  with ds do
    begin
    AFld1 := TIntegerField.Create(ds);
    AFld1.FieldName := 'ID';
    AFld1.DataSet := ds;

    AFld2 := TStringField.Create(ds);
    AFld2.FieldName := 'NAME';
    AFld2.DataSet := ds;

    AFld3 := TIntegerField.Create(ds);
    with AFld3 do
      begin
      FieldName := 'LookupFld';
      FieldKind := fkLookup;
      DataSet := ds;
      LookupDataSet := lkpDs;
      LookupKeyFields:='name';
      LookupResultField:='ID';
      KeyFields := 'name';
      end;
    ds.OnCalcFields:=DatasetNotifyEvent;
    lkpds.Open;
    open;
    Next;
    Buffer:=ds.ActiveBuffer;

    // When LookupCache is true, use the lookupCache (Here with the 'wrong' value 412)
    AFld3.LookupList.Clear;
    AFld3.LookupList.Add('TestName2',412);
    AFld3.LookupCache:=True;
    // CalculateFields is the only way to call CalcLookupValue
    THackDataset(ds).CalculateFields(Buffer);
    AssertEquals(412,AFld3.AsInteger);

    // Without lookupcache, return the right value
    AFld3.LookupCache:=False;
    THackDataset(ds).CalculateFields(Buffer);
    AssertEquals(2,AFld3.AsInteger);

    // If there's no LookupDataset, the result should be Null
    AFld3.LookupDataSet:= nil;
    THackDataset(ds).CalculateFields(Buffer);
    AssertTrue(AFld3.IsNull);

    // If there's no LookupDataset, the result should be Null
    AFld3.LookupDataSet:= nil;
    THackDataset(ds).CalculateFields(Buffer);
    AssertTrue(AFld3.IsNull);

    // Same holds for closed lookupdatasets
    AFld3.LookupDataSet:= lkpDs;
    lkpDs.Close;
    THackDataset(ds).CalculateFields(Buffer);
    AssertTrue(AFld3.IsNull);
    lkpds.Open;
    
    // Thing are getting interesting with multiple fields in the key:
    AFld3.LookupKeyFields:='name;id';
    AFld3.KeyFields := 'name;id';
    AFld3.LookupCache:=True;
    AFld3.LookupList.Clear;
    AFld3.LookupList.Add(VarArrayOf(['TestName2',2]),112);
    AFld3.LookupCache:=True;
    THackDataset(ds).CalculateFields(Buffer);
    AssertEquals(112,AFld3.AsInteger);
    AFld3.LookupCache:=False;

    // Now without a LookupCache
    // Disabled this part, since tDbf has problems with multiple-field keys
{
    AFld3.LookupKeyFields:='name;id';
    AFld3.KeyFields := 'name;id';
    THackDataset(ds).CalculateFields(Buffer);
    AssertEquals(2,AFld3.AsInteger);}
    end;

end;

procedure TTestDatasources.TestEnableControls;
var ds: TDataset;
    ADataLink : TTestDataLink;
    ADataSource : TDataSource;
begin
  ds := DBConnector.GetTraceDataset(False);
  ADatasource := TDataSource.Create(nil);
  ADatalink := TTestDataLink.Create;
  ADatalink.DataSource := aDatasource;
  ADataSource.DataSet := ds;
  with ds do
    begin
    Open;
    
    // If DisableControls isn't called, nothing should happen.
    DataEvents:='';
    EnableControls;
    AssertEquals('',DataEvents);

    DisableControls;
    DisableControls;
    // DisableControls is called twice. Ie: first call to enablecontrols should
    // still do nothing.
    DataEvents:='';
    EnableControls;
    AssertEquals('',DataEvents);

    // On this call to Enablecontrols, the controls should get enabled again:
    DataEvents:='';
    EnableControls;
    AssertEquals('SetCurrentRecord;deDataSetChange:0;',DataEvents);

    // If the state of the dataset has been changed while the controls were
    // disabled, then an deUpdateState event should be raised
    DisableControls;
    THackDataset(ds).SetState(dsSetKey);
    DataEvents:='';
    EnableControls;
    AssertEquals('deUpdateState:0;SetCurrentRecord;deDataSetChange:0;',DataEvents);
    THackDataset(ds).SetState(dsBrowse);

    // If the dataset is closed while the controls were disabled, then only
    // an deUpdateState event should occur.
    DisableControls;
    Close;
    DataEvents:='';
    EnableControls;
    AssertEquals('deUpdateState:0;',DataEvents);

    // And the same happens if the dataset was opened
    DisableControls;
    Open;
    DataEvents:='';
    EnableControls;
    AssertEquals('deUpdateState:0;',DataEvents);
    close;
    end;
  ADataLink.Free;
  ADataSource.Free;
end;

initialization
  if uppercase(dbconnectorname)='DBF' then RegisterTest(TTestDatasources);
end.

