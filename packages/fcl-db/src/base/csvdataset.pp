{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2014 by Michael Van Canneyt, member of the
    Free Pascal development team

    CSV Dataset implementation.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit csvdataset;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, bufdataset, csvreadwrite, db, sqldb;

Type


  { TCSVOptions }

  TCSVOptions = Class(TCSVHandler)
  private
    FDefaultFieldLength: Word;
    FFirstLineAsFieldNames: Boolean;
  Public
    Constructor Create; override;
    Procedure Assign(Source : TPersistent); override;
  Published
    // Does first line of the file contain the field names to use ?
    property FirstLineAsFieldNames : Boolean Read FFirstLineAsFieldNames Write FFirstLineAsFieldNames;
    // Default is to create all fields as strings with the same length. Default string field length.
    // If the CSV dataset has field defs prior to loading, this is ignored.
    property DefaultFieldLength : Word Read FDefaultFieldLength Write FDefaultFieldLength;
    // Field delimiter
    property Delimiter;
    // Character used to quote "problematic" data
    // (e.g. with delimiters or spaces in them)
    // A common quotechar is "
    property QuoteChar;
    // String at the end of the line of data (e.g. CRLF)
    property LineEnding;
    // Ignore whitespace between delimiters and field data
    property IgnoreOuterWhitespace;
    // Use quotes when outer whitespace is found
    property QuoteOuterWhitespace;
  end;

  { TCSVDataPacketReader }

  TCSVDataPacketReader = class(TDataPacketReader)
  private
    FOptions: TCSVOptions;
    FOwnsOptions: Boolean;
    FParser : TCSVParser;
    FBuilder : TCSVBuilder;
    FLine : TStringList;
    FCurrentRow : Integer;
    FEOF : Boolean;
    FCreateFieldDefs : TFieldDefs;
    // Read next row in Fline
  Protected
    Procedure ReadNextRow;virtual;
    procedure SetCreateFieldDefs(AValue: TFieldDefs);virtual;
  public
    constructor Create(ADataSet: TCustomBufDataset; AStream : TStream); override;
    constructor Create(ADataSet: TCustomBufDataset; AStream : TStream; AOptions : TCSVOptions);
    Destructor Destroy; override;
    procedure LoadFieldDefs(var AnAutoIncValue : integer); override;
    procedure StoreFieldDefs(AnAutoIncValue : integer); override;
    function GetRecordRowState(out AUpdOrder : Integer) : TRowState; override;
    procedure FinalizeStoreRecords; override;
    function GetCurrentRecord : boolean; override;
    procedure GotoNextRecord; override;
    procedure InitLoadRecords; override;
    procedure RestoreRecord; override;
    procedure StoreRecord(ARowState : TRowState; AUpdOrder : integer = 0); override;
    class function RecognizeStream(AStream : TStream) : boolean; override;
    Property Options : TCSVOptions Read FOptions;
    Property CreateFieldDefs : TFieldDefs read FCreateFieldDefs Write SetCreateFieldDefs;
  end;

  { TCustomCSVDataset }

  TCustomCSVDataset = Class(TBufDataset)
  private
    FCSVOptions: TCSVOptions;
    procedure SetCSVOptions(AValue: TCSVOptions);
  Protected
    class function DefaultReadFileFormat : TDataPacketFormat; override;
    class function DefaultWriteFileFormat : TDataPacketFormat; override;
    class function DefaultPacketClass : TDataPacketReaderClass ; override;
    function CreateDefaultPacketReader(aStream : TStream): TDataPacketReader ; override;
    function GetPacketReader(const Format: TDataPacketFormat; const AStream: TStream): TDataPacketReader; override;
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField); override;
    procedure InternalInitFieldDefs; override;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
    { If FieldDefs is filled prior to calling one of the load functions,
      the fielddefs definitions will be checked against file contents
      as far as possible: count and names if names are on first line}
    procedure LoadFromCSVStream(AStream : TStream);
    procedure LoadFromCSVFile(Const AFileName: string);
    procedure SaveToCSVStream(AStream : TStream);
    procedure SaveToCSVFile(AFileName: string = '');
  Protected
    Property CSVOptions : TCSVOptions Read FCSVOptions Write SetCSVOptions;
  end;

  TCSVDataset = Class(TCustomCSVDataset)
  Published
    Property CSVOptions;
  end;

implementation

{ TCSVDataPacketReader }

procedure TCSVDataPacketReader.ReadNextRow;


begin
  FLine.Clear;
  if not FEOF then
    begin
    if (FCurrentRow>0) then
      FLine.Add(FParser.CurrentCellText);
    Repeat
      FEOF:=Not FParser.ParseNextCell;
      if (not FEOF) and (FParser.CurrentRow=FCurrentRow) then
        FLine.Add(FParser.CurrentCellText);
    until FEOF or (FParser.CurrentRow>FCurrentRow);
    end;
  FCurrentRow:=FParser.CurrentRow;
end;

procedure TCSVDataPacketReader.SetCreateFieldDefs(AValue: TFieldDefs);
begin
  if FCreateFieldDefs=AValue then Exit;
  if (FCreateFieldDefs=Nil) then
    FCreateFieldDefs:=TFieldDefs.Create(AValue.Dataset);
  FCreateFieldDefs.Assign(AValue);
end;

constructor TCSVDataPacketReader.Create(ADataSet: TCustomBufDataset; AStream: TStream);
begin
  inherited Create(ADataSet,AStream);
  if FOptions=Nil then
    begin
    FOptions:=TCSVOptions.Create;
    FOptions.FFirstLineAsFieldNames:=True;
    FOwnsOptions:=True;
    end;
  FLine:=TStringList.Create;
end;

constructor TCSVDataPacketReader.Create(ADataSet: TCustomBufDataset; AStream: TStream; AOptions: TCSVOptions);
begin
  FOptions:=AOptions;
  Create(ADataset,AStream);
  FOwnsOptions:=AOptions=Nil;
end;

destructor TCSVDataPacketReader.Destroy;
begin
  FreeAndNil(FCreateFieldDefs);
  If FOwnsOptions then
    FreeAndNil(FOPtions);
  FreeAndNil(Fline);
  FreeAndNil(FParser);
  inherited Destroy;
end;

procedure TCSVDataPacketReader.LoadFieldDefs(var AnAutoIncValue: integer);
Var
  FN : String;
  I : Integer;

begin
  FParser:=TCSVParser.Create;
  FParser.Assign(FOptions);
  FParser.SetSource(Stream);
  FCurrentRow:=0;
  ReadNextRow;
  If Assigned(CreateFieldDefs) then
   begin
   if (CreateFieldDefs.Count<>Fline.Count) then
     DatabaseErrorFmt('CSV File Field count (%d) does not match dataset field count (%d).',[Fline.Count,CreateFieldDefs.Count],Dataset.FieldDefs.Dataset);
   If FOptions.FirstLineAsFieldNames then
     For I:=0 to FLine.Count-1 do
       If (CompareText(FLine[i],CreateFieldDefs[i].Name)<>0) then
         DatabaseErrorFmt('CSV File field %d: name "%s" does not match dataset field name "%s".',[I,FLine[i],CreateFieldDefs[i].Name],Dataset.FieldDefs.Dataset);
   Dataset.FieldDefs.Assign(CreateFieldDefs);
   end
  else if (FLine.Count>0) then
    For I:=0 to FLine.Count-1 do
      begin
      If FOptions.FirstLineAsFieldNames then
        FN:=FLine[i]
      else
        FN:=Format('Column%d',[i+1]);
      Dataset.FieldDefs.Add(FN,ftString,Foptions.DefaultFieldLength);
      end;
  if FOptions.FirstLineAsFieldNames then
   ReadNextRow;
end;

procedure TCSVDataPacketReader.StoreFieldDefs(AnAutoIncValue: integer);

Var
  I : Integer;

begin
  FBuilder:=TCSVBuilder.Create;
  FBuilder.Assign(FOptions);
  FBuilder.SetOutput(Stream);
  if FOptions.FirstLineAsFieldNames then
    begin
    For I:=0 to Dataset.FieldDefs.Count-1 do
      FBuilder.AppendCell(Dataset.FieldDefs[i].Name);
    FBuilder.AppendRow;
    end;
end;

function TCSVDataPacketReader.GetRecordRowState(out AUpdOrder: Integer
  ): TRowState;
begin
  AUpdOrder:=0;
  Result:=[];
end;

procedure TCSVDataPacketReader.FinalizeStoreRecords;
begin

end;

function TCSVDataPacketReader.GetCurrentRecord: boolean;
begin
  Result:=Fline.Count>0;
end;

procedure TCSVDataPacketReader.GotoNextRecord;
begin
  ReadNextRow;
end;

procedure TCSVDataPacketReader.InitLoadRecords;
begin
   // Do nothing
end;

procedure TCSVDataPacketReader.RestoreRecord;

Var
  I : integer;

begin
  For I:=0 to Fline.Count-1 do
    Dataset.Fields[i].AsString:=Copy(FLine[i],1,Dataset.Fields[i].Size)
end;

procedure TCSVDataPacketReader.StoreRecord(ARowState: TRowState; AUpdOrder: integer);
Var
  I : integer;

begin
  For I:=0 to Dataset.Fields.Count-1 do
    FBuilder.AppendCell(Dataset.Fields[i].AsString);
  FBuilder.AppendRow;
end;

class function TCSVDataPacketReader.RecognizeStream(AStream: TStream): boolean;
begin
  Result:=False;
end;

{ TCSVOptions }

Constructor TCSVOptions.Create;
begin
  inherited Create;
  DefaultFieldLength:=255;
end;

Procedure TCSVOptions.Assign(Source: TPersistent);
begin
  if (Source is TCSVOptions) then
    begin
    FFirstLineAsFieldNames:=TCSVOptions(Source).FirstLineAsFieldNames;
    FDefaultFieldLength:=TCSVOptions(Source).FDefaultFieldLength
    end;
  inherited Assign(Source);
end;

{ TCustomCSVDataset }

procedure TCustomCSVDataset.SetCSVOptions(AValue: TCSVOptions);
begin
  if (FCSVOptions=AValue) then Exit;
  FCSVOptions.Assign(AValue);
end;

class function TCustomCSVDataset.DefaultReadFileFormat: TDataPacketFormat;
begin
  Result:=dfDefault;
end;

class function TCustomCSVDataset.DefaultWriteFileFormat: TDataPacketFormat;
begin
  Result:=dfDefault;
end;

class function TCustomCSVDataset.DefaultPacketClass: TDataPacketReaderClass;
begin
  Result:=TCSVDataPacketReader;
end;

function TCustomCSVDataset.CreateDefaultPacketReader(aStream: TStream): TDataPacketReader;
begin
  Result:=TCSVDataPacketReader.Create(Self,AStream,FCSVOptions)
end;

function TCustomCSVDataset.GetPacketReader(const Format: TDataPacketFormat;
  const AStream: TStream): TDataPacketReader;
begin
  If (Format in [dfAny,dfDefault]) then
    Result:=CreateDefaultPacketReader(AStream)
  else
    Result:=Inherited GetPacketReader(Format,AStream);
end;

procedure TCustomCSVDataset.LoadBlobIntoBuffer(FieldDef: TFieldDef;
  ABlobBuf: PBufBlobField);
begin
  // Do nothing
end;

procedure TCustomCSVDataset.InternalInitFieldDefs;
begin
  // Do nothing
end;

constructor TCustomCSVDataset.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FCSVOptions:=TCSVOptions.Create;
end;

destructor TCustomCSVDataset.Destroy;
begin
  // We must close here, before freeing the options.
  Active:=False;
  FreeAndNil(FCSVOptions);
  inherited Destroy;
end;

procedure TCustomCSVDataset.LoadFromCSVStream(AStream: TStream);

Var
  P : TCSVDataPacketReader;

begin
  CheckInactive;
  P:=TCSVDataPacketReader.Create(Self,AStream,FCSVOptions);
  try
    if FieldDefs.Count>0 then
     P.CreateFieldDefs:=FieldDefs;
    SetDatasetPacket(P);
  finally
    P.Free;
  end;
end;

procedure TCustomCSVDataset.LoadFromCSVFile(const AFileName: string);

Var
  F : TFileStream;

begin
  F:=TFileStream.Create(AFileName,fmOpenRead or fmShareDenyWrite);
  try
    LoadFromCSVStream(F);
  finally
    F.Free;
  end;
end;

procedure TCustomCSVDataset.SaveToCSVStream(AStream: TStream);

Var
  P : TCSVDataPacketReader;

begin
  First;
  MergeChangeLog;
  P:=TCSVDataPacketReader.Create(Self,AStream,FCSVOptions);
  try
    GetDatasetPacket(P);
  finally
    P.Free;
  end;
end;

procedure TCustomCSVDataset.SaveToCSVFile(AFileName: string);
Var
  F : TFileStream;

begin
  F:=TFileStream.Create(AFileName, fmCreate);
  try
    SaveToCSVStream(F);
  finally
    F.Free;
  end;
end;

end.

