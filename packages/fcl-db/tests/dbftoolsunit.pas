unit DBFToolsUnit;

{ Sets up dbf datasets for testing
Tests expect Get*Dataset to return a dataset with structure and test data, but closed.
}

{$IFDEF FPC}
  {$mode objfpc}{$H+}
{$ENDIF}

// If defined, save the dbf files when done and print out location to stdout:
{.$DEFINE KEEPDBFFILES}

interface

uses
  Classes, SysUtils, toolsunit,
  DB, Dbf, dbf_common;

type
  { TDBFDBConnector }

  TDBFDBConnector = class(TDBConnector)
  protected
    procedure CreateNDatasets; override;
    procedure CreateFieldDataset; override;
    procedure DropNDatasets; override;
    procedure DropFieldDataset; override;
<<<<<<< HEAD
    // InternalGetNDataset reroutes to ReallyInternalGetNDataset
    function InternalGetNDataset(n: integer): TDataset; override;
    function InternalGetFieldDataset: TDataSet; override;
    // GetNDataset allowing trace dataset if required;
    // if trace is on, use a TDbfTraceDataset instead of TDBFAutoClean
    function ReallyInternalGetNDataset(n: integer; Trace: boolean): TDataset;
  public
    function GetTraceDataset(AChange: boolean): TDataset; override;
  end;

  { TDBFAutoClean }
  // DBF descendant that saves to a memory stream instead of file
  TDBFAutoClean = class(TDBF)
  private
    FBackingStream: TMemoryStream;
    FIndexBackingStream: TMemoryStream;
    FMemoBackingStream: TMemoryStream;
    FCreatedBy: string;
  public
    // Keeps track of which function created the dataset, useful for troubleshooting
    property CreatedBy: string read FCreatedBy write FCreatedBy;
    constructor Create;
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    function UserRequestedTableLevel: integer;
  end;

  { TDbfTraceDataset }
  TDbfTraceDataset = class(TdbfAutoClean)
  protected
    procedure SetCurrentRecord(Index: longint); override;
=======
    Function InternalGetNDataset(n : integer) : TDataset; override;
    Function InternalGetFieldDataset : TDataSet; override;
  public
    function GetTraceDataset(AChange : Boolean) : TDataset; override;
<<<<<<< HEAD
=======
  end;

  { TDbfTraceDataset }

  TDbfTraceDataset = class(Tdbf)
  protected
    procedure SetCurrentRecord(Index: Longint); override;
    procedure RefreshInternalCalcFields(Buffer: PChar); override;
    procedure InternalInitFieldDefs; override;
    procedure CalculateFields(Buffer: PChar); override;
    procedure ClearCalcFields(Buffer: PChar); override;
>>>>>>> origin/fixes_2_2
  end;

  { TDbfTraceDataset }

  TDbfTraceDataset = class(Tdbf)
  protected
    procedure SetCurrentRecord(Index: Longint); override;
>>>>>>> graemeg/fixes_2_2
    procedure RefreshInternalCalcFields(Buffer: PChar); override;
    procedure InternalInitFieldDefs; override;
    procedure CalculateFields(Buffer: PChar); override;
    procedure ClearCalcFields(Buffer: PChar); override;
  end;


implementation

uses
  FmtBCD;

function GetNewTempDBFName: string;
// Scans temp directory for dbf names and adds
var
  Res: TSearchRec;
  Path, Name: string;
  FileAttr: LongInt;
  Attr,NextFileNo: Integer;
begin
  NextFileNo:=0;
  Attr := faAnyFile;
  if FindFirst(IncludeTrailingPathDelimiter(GetTempDir)+'*.dbf', Attr, Res) = 0 then
  begin
    Path := GetTempDir;
    repeat
       Name := ConcatPaths([Path, Res.Name]);
       FileAttr := FileGetAttr(Name);
       if FileAttr and faDirectory = 0 then
       begin
         // Capture alphabetically latest name
         try
           //... only if it is numeric
           if strtoint(ChangeFileExt(Res.Name,''))>NextFileNo then
             NextFileNo:=strtoint(ChangeFileExt(Res.Name,''));
         except
           // apparently not numeric
         end;
       end
    until FindNext(Res) <> 0;
  end;
  FindClose(Res);
  // now we now the latest file, add 1, and paste the temp directory in front of it
  NextFileNo:=NextFileNo+1;
  Result:=IncludeTrailingPathDelimiter(GetTempDir)+IntToStr(NextFileNo)+'.DBF';
end;

{ TDBFAutoClean }

function TDBFAutoClean.UserRequestedTableLevel: integer;
  // User can specify table level as a connector param, e.g.:
  // connectorparams=4
  // If none given, default to DBase IV
var
  TableLevelProvided: integer;
begin
  TableLevelProvided := StrToIntDef(dbconnectorparams, 4);
  if not (TableLevelProvided in [3, 4, 5, 7, 
    TDBF_TABLELEVEL_FOXPRO, TDBF_TABLELEVEL_VISUALFOXPRO]) then
  begin
    Result := -1; // hope this crashes the tests so user is alerted.
    //Invalid tablelevel specified in connectorparams= field. Aborting
    exit;
  end;
  Result := TableLevelProvided;
end;

constructor TDBFAutoClean.Create;
begin
  // Create storage for data:
  FBackingStream:=TMemoryStream.Create;
  FIndexBackingStream:=TMemoryStream.Create;
  FMemoBackingStream:=TMemoryStream.Create;
  // Create a unique name (within the 10 character DBIII limit):
  TableName := FormatDateTime('hhnnssz',Now())+'_'+inttostr(random(99));
  TableLevel := UserRequestedTableLevel;
  Storage:=stoMemory;
  UserStream:=FBackingStream;
  UserIndexStream:=FIndexBackingStream;
  UserMemoStream:=FMemoBackingStream;
  CreateTable; //this will also write out the dbf header to disk/stream
end;

constructor TDBFAutoClean.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  Self.Create;
end;

destructor TDBFAutoClean.Destroy;
{$IFDEF KEEPDBFFILES}
var
  FileName: string;
{$ENDIF}
begin
  {$IFDEF KEEPDBFFILES}
  Close;
  FileName := GetNewTempDBFName;
  FBackingStream.SaveToFile(FileName);
  FIndexBackingStream.SaveToFile(ChangeFileExt(FileName, '.mdx'));
  if Self.TableLevel in [TDBF_TABLELEVEL_FOXPRO, TDBF_TABLELEVEL_VISUALFOXPRO] then
    FMemoBackingStream.SaveToFile(ChangeFileExt(FileName, '.fpt'))
  else
    FMemoBackingStream.SaveToFile(ChangeFileExt(FileName, '.dbt'));
  writeln('TDBFAutoClean: file created by ',CreatedBy,' left file: ',FileName);
  {$ENDIF}
  inherited Destroy;
  FBackingStream.Free;
  FIndexBackingStream.Free;
end;


procedure TDBFDBConnector.CreateNDatasets;
begin
<<<<<<< HEAD
  // All datasets are created in InternalGet*Dataset
=======
  for n := 0 to MaxDataSet do
    begin
    with TDbf.Create(nil) do
      begin
      FilePath := dbname;
      TableName := 'fpdev_'+inttostr(n)+'.db';
      FieldDefs.Add('ID',ftInteger);
      FieldDefs.Add('NAME',ftString,50);
      CreateTable;
      Open;
      if n > 0 then for countId := 1 to n do
        begin
        Append;
        FieldByName('ID').AsInteger := countID;
        FieldByName('NAME').AsString := 'TestName'+inttostr(countID);
        // Explicitly call .post, since there could be a bug which disturbs
        // the automatic call to post. (example: when TDataset.DataEvent doesn't
        // work properly)
        Post;
        end;
      if state = dsinsert then
        Post;
      Close;
      Free;
      end;
    end;
>>>>>>> graemeg/fixes_2_2
end;

procedure TDBFDBConnector.CreateFieldDataset;
begin
<<<<<<< HEAD
  // All datasets are created in InternalGet*Dataset
=======
  with TDbf.Create(nil) do
    begin
    FilePath := dbname;
    TableName := 'fpdev_field.db';
    FieldDefs.Add('ID',ftInteger);
    FieldDefs.Add('FSTRING',ftString,10);
    FieldDefs.Add('FSMALLINT',ftSmallint);
    FieldDefs.Add('FINTEGER',ftInteger);
//    FieldDefs.Add('FWORD',ftWord);
    FieldDefs.Add('FBOOLEAN',ftBoolean);
    FieldDefs.Add('FFLOAT',ftFloat);
//    FieldDefs.Add('FCURRENCY',ftCurrency);
//    FieldDefs.Add('FBCD',ftBCD);
    FieldDefs.Add('FDATE',ftDate);
//    FieldDefs.Add('FTIME',ftTime);
    FieldDefs.Add('FDATETIME',ftDateTime);
    FieldDefs.Add('FLARGEINT',ftLargeint);
    CreateTable;
    Open;
    for i := 0 to testValuesCount-1 do
      begin
      Append;
      FieldByName('ID').AsInteger := i;
      FieldByName('FSTRING').AsString := testStringValues[i];
      FieldByName('FSMALLINT').AsInteger := testSmallIntValues[i];
      FieldByName('FINTEGER').AsInteger := testIntValues[i];
      FieldByName('FBOOLEAN').AsBoolean := testBooleanValues[i];
      FieldByName('FFLOAT').AsFloat := testFloatValues[i];
      FieldByName('FDATE').AsDateTime := StrToDate(testDateValues[i], 'yyyy/mm/dd', '-');
      FieldByName('FLARGEINT').AsLargeInt := testLargeIntValues[i];
      Post;
      end;
    Close;
    end;
>>>>>>> graemeg/cpstrnew
end;

procedure TDBFDBConnector.DropNDatasets;
begin
<<<<<<< HEAD
  // Nothing to be done here; the dataset is cleaned up in TDBFAutoClean.Destroy
=======
  for n := 0 to MaxDataSet do
    DeleteFile(ExtractFilePath(dbname)+'fpdev_'+inttostr(n)+'.db');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

procedure TDBFDBConnector.DropFieldDataset;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  // Nothing to be done here; the dataset is cleaned up in TDBFAutoClean.Destroy
=======
  DeleteFile(ExtractFilePath(dbname)+'fpdev_field.db');
>>>>>>> graemeg/cpstrnew
=======
  DeleteFile(ExtractFilePath(dbname)+'fpdev_field.db');
>>>>>>> graemeg/cpstrnew
=======
  DeleteFile(ExtractFilePath(dbname)+'fpdev_field.db');
>>>>>>> graemeg/cpstrnew
=======
  DeleteFile(ExtractFilePath(dbname)+'fpdev_field.db');
>>>>>>> origin/cpstrnew
end;

function TDBFDBConnector.InternalGetNDataset(n: integer): TDataset;
begin
  result:=ReallyInternalGetNDataset(n,false);
end;

function TDBFDBConnector.InternalGetFieldDataset: TDataSet;
var
  i: integer;
begin
  Result := (TDbfAutoClean.Create(nil) as TDataSet);
  with (Result as TDBFAutoClean) do
  begin
    CreatedBy:='InternalGetFieldDataset';
    FieldDefs.Add('ID', ftInteger);
    FieldDefs.Add('FSTRING', ftString, 10);
    FieldDefs.Add('FSMALLINT', ftSmallint);
    FieldDefs.Add('FINTEGER', ftInteger);
    FieldDefs.Add('FWORD', ftWord);
    FieldDefs.Add('FBOOLEAN', ftBoolean);
    FieldDefs.Add('FFLOAT', ftFloat);
    // Field types only available in (Visual) FoxPro
    if (Result as TDBF).TableLevel >= TDBF_TABLELEVEL_FOXPRO then
      FieldDefs.Add('FCURRENCY', ftCurrency);
    if (Result as TDBF).TableLevel >= TDBF_TABLELEVEL_FOXPRO then
      FieldDefs.Add('FBCD', ftBCD);
    FieldDefs.Add('FDATE', ftDate);
    FieldDefs.Add('FDATETIME', ftDateTime);
    FieldDefs.Add('FLARGEINT', ftLargeint);
    FieldDefs.Add('FMEMO', ftMemo);
    CreateTable;
    Open;
    for i := 0 to testValuesCount - 1 do
    begin
      Append;
      FieldByName('ID').AsInteger := i;
      FieldByName('FSTRING').AsString := testStringValues[i];
      FieldByName('FSMALLINT').AsInteger := testSmallIntValues[i];
      FieldByName('FINTEGER').AsInteger := testIntValues[i];
      FieldByName('FWORD').AsInteger := testWordValues[i];
      FieldByName('FBOOLEAN').AsBoolean := testBooleanValues[i];
      FieldByName('FFLOAT').AsFloat := testFloatValues[i];
      if (Result as TDBF).TableLevel >= TDBF_TABLELEVEL_FOXPRO then
        FieldByName('FCURRENCY').AsCurrency := testCurrencyValues[i];
      // work around missing TBCDField.AsBCD:
      if (Result as TDBF).TableLevel >= TDBF_TABLELEVEL_FOXPRO then
        FieldByName('FBCD').AsBCD := StrToBCD(testFmtBCDValues[i],Self.FormatSettings);
      FieldByName('FDATE').AsDateTime := StrToDate(testDateValues[i], 'yyyy/mm/dd', '-');
      FieldByName('FDATETIME').AsDateTime := StrToDateTime(testValues[ftDateTime,i], Self.FormatSettings);
      FieldByName('FLARGEINT').AsLargeInt := testLargeIntValues[i];
      FieldByName('FMEMO').AsString := testStringValues[i];
      Post;
    end;
    Close;
  end;
end;

function TDBFDBConnector.ReallyInternalGetNDataset(n: integer; Trace: boolean): TDataset;
var
  countID: integer;
begin
  if Trace then
    Result := (TDbfTraceDataset.Create(nil) as TDataSet)
  else
    Result := (TDBFAutoClean.Create(nil) as TDataSet);
  with (Result as TDBFAutoclean) do
  begin
    CreatedBy:='InternalGetNDataset('+inttostr(n)+')';
    FieldDefs.Add('ID', ftInteger);
    FieldDefs.Add('NAME', ftString, 50);
    CreateTable;
    Open;
    if n > 0 then
      for countId := 1 to n do
      begin
        Append;
        FieldByName('ID').AsInteger := countID;
        FieldByName('NAME').AsString := 'TestName' + IntToStr(countID);
        // Explicitly call .post, since there could be a bug which disturbs
        // the automatic call to post. (example: when TDataset.DataEvent doesn't
        // work properly)
        Post;
      end;
    if state = dsinsert then
      Post;
    Close;
  end;
end;

function TDBFDBConnector.GetTraceDataset(AChange: boolean): TDataset;
begin
  // Mimic TDBConnector.GetNDataset
  if AChange then FChangedDatasets[NForTraceDataset] := True;
  Result := ReallyInternalGetNDataset(NForTraceDataset,true);
  FUsedDatasets.Add(Result);
end;

{ TDbfTraceDataset }

procedure TDbfTraceDataset.SetCurrentRecord(Index: longint);
begin
  DataEvents := DataEvents + 'SetCurrentRecord' + ';';
  inherited SetCurrentRecord(Index);
end;

procedure TDbfTraceDataset.RefreshInternalCalcFields(Buffer: PChar);
begin
  DataEvents := DataEvents + 'RefreshInternalCalcFields' + ';';
  inherited RefreshInternalCalcFields(Buffer);
end;

procedure TDbfTraceDataset.InternalInitFieldDefs;
var
  i: integer;
  IntCalcFieldName: string;
begin
  // To fake an internal calculated field, set its fielddef InternalCalcField
  // property to true, before the dataset is opened.
  // This procedure takes care of setting the automatically created fielddef's
  // InternalCalcField property to true. (works for only one field)
  IntCalcFieldName := '';
  for i := 0 to FieldDefs.Count - 1 do
    if fielddefs[i].InternalCalcField then
      IntCalcFieldName := FieldDefs[i].Name;
  inherited InternalInitFieldDefs;
  if IntCalcFieldName <> '' then
    with FieldDefs.find(IntCalcFieldName) do
    begin
      InternalCalcField := True;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
function TDBFDBConnector.GetTraceDataset(AChange: Boolean): TDataset;
var ADS, AResDS : TDbf;
begin
  ADS := GetNDataset(AChange,15) as TDbf;
  AResDS := TDbfTraceDataset.Create(nil);
  AResDS.FilePath:=ADS.FilePath;
  AResDs.TableName:=ADS.TableName;
  Result:=AResDS;
end;

{ TDbfTraceDataset }

procedure TDbfTraceDataset.SetCurrentRecord(Index: Longint);
begin
  DataEvents := DataEvents + 'SetCurrentRecord' + ';';
  inherited SetCurrentRecord(Index);
end;

procedure TDbfTraceDataset.RefreshInternalCalcFields(Buffer: PChar);
begin
  DataEvents := DataEvents + 'RefreshInternalCalcFields' + ';';
  inherited RefreshInternalCalcFields(Buffer);
end;

procedure TDbfTraceDataset.InternalInitFieldDefs;
var i : integer;
    IntCalcFieldName : String;
begin
  // To fake a internal calculated field, set it's fielddef InternalCalcField
  // property to true, before the dataset is opened.
  // This procedure takes care of setting the automatically created fielddef's
  // InternalCalcField property to true. (works for only one field)
  IntCalcFieldName:='';
  for i := 0 to FieldDefs.Count -1 do
    if fielddefs[i].InternalCalcField then IntCalcFieldName := FieldDefs[i].Name;
  inherited InternalInitFieldDefs;
  if IntCalcFieldName<>'' then with FieldDefs.find(IntCalcFieldName) do
    begin
    InternalCalcField := True;
    end;
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TDbfTraceDataset.CalculateFields(Buffer: PChar);
begin
  DataEvents := DataEvents + 'CalculateFields' + ';';
  inherited CalculateFields(Buffer);
end;

procedure TDbfTraceDataset.ClearCalcFields(Buffer: PChar);
begin
  DataEvents := DataEvents + 'ClearCalcFields' + ';';
  inherited ClearCalcFields(Buffer);
end;

initialization
  RegisterClass(TDBFDBConnector);
end.
