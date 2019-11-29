unit fpfixedexport;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, db, fpDBExport;
  
  { TFixedLengthExportFieldItem }

Type
  TFixedLengthExportFieldItem = Class(TExportFieldItem)
  private
    FWidth: Integer;
    FAlignField: TAlignField;
  Public
    Procedure Assign(Source : TPersistent); override;
  Published
    Property Width : Integer Read FWidth Write FWidth;
    Property AlignField: TAlignField Read FAlignField write FAlignField;
  end;

  { TCustomFixedLengthExporter }
  TCharMode = (cmANSI,cmUTF8,cmUTF16);

  { TFixedExportFormatSettings }

  TFixedExportFormatSettings = Class (TExportFormatSettings)
  private
    FCharMode: TCharMode;
    FColumnSeparatorSpaceCount: Integer;
    FHeaderRow: Boolean;
  Public
    Procedure Assign(Source: TPersistent); override;
  Published
    // Whether or not the file should have a header row with field names
    Property HeaderRow : Boolean Read FHeaderRow Write FHeaderRow default true;
    // How to handle Unicode ?
    Property CharMode : TCharMode Read FCharMode Write FCharMode;
    // Number of separator spaces between columns. Default 0.
    Property ColumnSeparatorSpaceCount : Integer Read FColumnSeparatorSpaceCount Write FColumnSeparatorSpaceCount;
  end;

  TCustomFixedLengthExporter = Class(TCustomFileExporter)
  Private
    FCurrentRow : RawByteString;
    FCurrentRowUnicode : UnicodeString;
    FSpaces : RawByteString;
    FSpacesUnicode : UnicodeString;
    function GetCharMode: TCharMode;
    function GeTFixedExportFormatSettings: TFixedExportFormatSettings;
    procedure SetFixedExportFormatSettings(AValue: TFixedExportFormatSettings);
  Protected
    function ExportFieldAsUniCodeString(EF: TExportFieldItem; isHeader: Boolean=False): UnicodeString; virtual;
    procedure ExportFieldAnsi(EF: TExportFieldItem; isHeader: Boolean=False); virtual;
    procedure ExportFieldUTF16(EF: TExportFieldItem; isHeader: Boolean=False); virtual;
    procedure ExportFieldUTF8(EF: TExportFieldItem; isHeader: Boolean=False); virtual;
    Procedure BuildDefaultFieldMap(AMap : TExportFields); override;
    Function  CreateExportFields : TExportFields; override;
    Function  CreateFormatSettings: TCustomExportFormatSettings; override;
    Procedure DoBeforeExecute; override;
    Procedure DoAfterExecute; override;
    Procedure DoDataRowStart; override;
    Procedure ExportField(EF : TExportFieldItem); override;
    Procedure DoDataRowEnd; override;
    Procedure DoDataHeader; override;
    Property CharMode : TCharMode Read GetCharMode;
    Property FormatSettings : TFixedExportFormatSettings Read GetFixedExportFormatSettings Write SetFixedExportFormatSettings;
  end;

  TFixedLengthExporter = Class(TCustomFixedLengthExporter)
  Published
    Property FileName;
    Property Dataset;
    Property ExportFields;
    Property FromCurrent;
    Property RestorePosition;
    Property FormatSettings;
    Property OnExportRow;
  end;

Procedure RegisterFixedExportFormat;
Procedure UnRegisterFixedExportFormat;

Const
  SFixedLengthExport      = 'Fixed';
  SFixedLengthExtensions  = '.txt';

Resourcestring
  SFixedLengthDescription = 'Text file with fixed length records';



implementation

uses math;

{ TFixedExportFormatSettings }

procedure TFixedExportFormatSettings.Assign(Source: TPersistent);
begin
  if (Source is TFixedExportFormatSettings) then
    begin
    CharMode:=TFixedExportFormatSettings(Source).CharMode;
    HeaderRow:=TFixedExportFormatSettings(Source).HeaderRow;
    ColumnSeparatorSpaceCount:=TFixedExportFormatSettings(Source).ColumnSeparatorSpaceCount;
    end;
  inherited Assign(Source);
end;

{ TFixedLengthExportFieldItem }

procedure TFixedLengthExportFieldItem.Assign(Source: TPersistent);

Var
  FL : TFixedLengthExportFieldItem;

begin
  If Source is TFixedLengthExportFieldItem then
    begin
    FL:=Source as TFixedLengthExportFieldItem;
    Width:=FL.Width;
    AlignField:=FL.AlignFIeld;
    end;
  inherited Assign(Source);
end;

{ TCustomFixedLengthExporter }


procedure TCustomFixedLengthExporter.SetFixedExportFormatSettings(AValue: TFixedExportFormatSettings);
begin
  Inherited FormatSettings:=AValue;
end;

function TCustomFixedLengthExporter.GetCharMode: TCharMode;
begin
  Result:=FormatSettings.CharMode;
end;

function TCustomFixedLengthExporter.GeTFixedExportFormatSettings: TFixedExportFormatSettings;
begin
  Result:=(Inherited Formatsettings) as TFixedExportFormatSettings;
end;

procedure TCustomFixedLengthExporter.BuildDefaultFieldMap(AMap: TExportFields);

Const
  RightAlignedFields = IntFieldTypes+FloatFieldTypes;

  // Mapping to TFieldType
  FieldWidths : Array[TFieldType] of integer =
    (
    {ftUnknown} -1,
    {ftString} 0,
    {ftSmallint} 3,
    {ftInteger} 10,
    {ftWord} 5,
    {ftBoolean} 1,
    {ftFloat} 20,
    {ftCurrency} 20,
    {ftBCD} 20,
    {ftDate} 10,
    {ftTime} 8,
    {ftDateTime} 20,
    {ftBytes} 0,
    {ftVarBytes} 0,
    {ftAutoInc} 10,
    {ftBlob} 0,
    {ftMemo} 0,
    {ftGraphic} 0,
    {ftFmtMemo} 0,
    {ftParadoxOle} 0,
    {ftDBaseOle} 0,
    {ftTypedBinary} 0,
    {ftCursor} 0,
    {ftFixedChar} 0,
    {ftWideString} 0,
    {ftLargeint} 0,
    {ftADT} 0,
    {ftArray} 0,
    {ftReference} 0,
    {ftDataSet} 0,
    {ftOraBlob} 0,
    {ftOraClob} 0,
    {ftVariant} 0,
    {ftInterface} 0,
    {ftIDispatch} 0,
    {ftGuid} 0,
    {ftTimeStamp} 0,
    {ftFMTBcd} 0,
    {ftFixedWideChar} 0,
    {ftWideMemo} 0
    );

  Function CalcLbool: integer;
  var
    LTrue,LFalse : Integer;

  begin
    Case charmode of
    cmUTF8:
      begin
      LTrue:=Length(UTF8Decode(FormatSettings.BooleanTrue));
      LFalse:=Length(UTF8Decode(FormatSettings.BooleanFalse));
      end;
     else
       LTrue:=Length(FormatSettings.BooleanTrue);
       LFalse:=Length(FormatSettings.BooleanFalse);
     end;
    Result:=Max(LTrue,LFalse);
  end;


Var
  I,W,LBool : Integer;
  F : TField;
  FL : TFixedLengthExportFieldItem;

begin
  inherited BuildDefaultFieldMap(AMap);
  lbool:=0;
  For I:=0 to AMap.Count-1 do
    begin
    FL:=TFixedLengthExportFieldItem(AMAP[i]);
    F:=Dataset.Fields[i];
    W:= FieldWidths[F.DataType];
    if F.DataType = ftBoolean then
      begin
      if lBool=0 then
        LBool:=CalcLBool;
      W:=lBool;
      end;
    If (W>0) then
      FL.Width:=W
    else if (W=0) then
      begin
      if (F.DataType in StringFieldTypes) then
        FL.Width:=F.Size;
      end;
    If (F.DataType in RightAlignedFields) then
      Fl.AlignField:=afRight;
    end;
end;

function TCustomFixedLengthExporter.CreateExportFields: TExportFields;
begin
  Result:=TExportFields.Create(TFixedLengthExportFieldItem);
end;

function TCustomFixedLengthExporter.CreateFormatSettings: TCustomExportFormatSettings;
begin
  Result:=TFixedExportFormatSettings.Create(True);
end;

procedure TCustomFixedLengthExporter.DoBeforeExecute;
begin
  inherited DoBeforeExecute;
  OpenTextFile;
  FSpaces:=StringOfChar(' ',FormatSettings.ColumnSeparatorSpaceCount);
  FSpacesUnicode:=StringOfChar(' ',FormatSettings.ColumnSeparatorSpaceCount);
end;

procedure TCustomFixedLengthExporter.DoAfterExecute;
begin
  CloseTextFile;
  inherited DoAfterExecute;
end;


procedure TCustomFixedLengthExporter.DoDataRowStart;
begin
  FCurrentRow:='';
end;

procedure TCustomFixedLengthExporter.ExportField(EF: TExportFieldItem);

begin
  Case CharMode of
    cmANSI : ExportFieldAnsi(EF);
    cmUTF8 : ExportFieldUTF8(EF);
    cmUTF16 : ExportFieldUTF16(EF);
  end;
end;


Function TCustomFixedLengthExporter.ExportFieldAsUniCodeString(EF: TExportFieldItem; isHeader : Boolean = False) : UnicodeString;

Var
  S,SS : UnicodeString;
  FL : TFixedLengthExportFieldItem;
  L,W : Integer;

begin
  if isHeader then
    S:=UTF8Decode(EF.ExportedName)
  else
    S:=UTF8Decode(FormatField(EF.Field));
  If EF is TFixedLengthExportFieldItem then
    begin
    FL:=TFixedLengthExportFieldItem(EF);
    W:=FL.Width;
    end
  else
    W:=Length(S);
  L:=Length(S);
  If L>W then
    begin
    If (FL.AlignField=afLeft) then
      S:=Copy(S,1,W)
    else
      Delete(S,1,L-W);
    end
  else if (L<W) then
    begin
    SS:=StringOfChar(' ',W-L);
    If FL.AlignField=afRight then
      S:=SS+S
    else
      S:=S+SS;
    end;
  Result:=S;
end;

procedure TCustomFixedLengthExporter.ExportFieldUTF16(EF: TExportFieldItem; isHeader : Boolean = False);

begin
  if (FormatSettings.ColumnSeparatorSpaceCount>0) and (Length(FCurrentRowUnicode)>0) then
    FCurrentRowUnicode:=FCurrentRowUnicode+FSpacesUnicode;

  FCurrentRowUnicode:=FCurrentRowUnicode+ExportFieldAsUnicodeString(EF,isHeader);
end;


procedure TCustomFixedLengthExporter.ExportFieldUTF8(EF: TExportFieldItem; isHeader : Boolean = False);


begin
  if (FormatSettings.ColumnSeparatorSpaceCount>0) and (Length(FCurrentRow)>0) then
    FCurrentRow:=FCurrentRow+FSpaces;
  FCurrentRow:=FCurrentRow+UTF8Encode(ExportFieldAsUnicodeString(EF,isHeader));
end;

procedure TCustomFixedLengthExporter.ExportFieldAnsi(EF: TExportFieldItem; isHeader : Boolean = False);

Var
  S,SS : String;
  W,L : Integer;
  FL : TFixedLengthExportFieldItem;

begin
  if isHeader then
    S:=EF.ExportedName
  else
    S:=FormatField(EF.Field);
  If EF is TFixedLengthExportFieldItem then
    begin
    FL:=TFixedLengthExportFieldItem(EF);
    W:=FL.Width;
    end
  else
    W:=Length(S);
  L:=Length(S);
  If L>W then
    begin
    If (FL.AlignField=afLeft) then
      S:=Copy(S,1,W)
    else
      Delete(S,1,L-W);
    end
  else if (L<W) then
    begin
    SS:=StringOfChar(' ',W-L);
    If FL.AlignField=afRight then
      S:=SS+S
    else
      S:=S+SS;
    end;
  if (FormatSettings.ColumnSeparatorSpaceCount>0) and (Length(FCurrentRow)>0) then
    FCurrentRow:=FCurrentRow+FSpaces;
  FCurrentRow:=FCurrentRow+S;
end;

procedure TCustomFixedLengthExporter.DoDataRowEnd;
begin
  if (CharMode<>cmUTF16) then
    Writeln(TextFile,FCurrentRow)
  else
    Writeln(TextFile,FCurrentRowUnicode);
  FCurrentRow:='';
  FCurrentRowUnicode:='';
end;

procedure TCustomFixedLengthExporter.DoDataHeader;

Var
  I : Integer;
  EF: TExportFieldItem;

begin
  FCurrentRow:='';
  if FormatSettings.HeaderRow then
    begin
    For I:=0 to ExportFields.Count-1 do
      begin
      EF:=ExportFields[I];
      If EF.Enabled then
        Case CharMode of
          cmANSI : ExportFieldAnsi(EF,True);
          cmUTF8 : ExportFieldUTF8(EF,True);
          cmUTF16 : ExportFieldUTF16(EF,True);
        end;
      end;
    DoDataRowEnd;
    end;
  inherited DoDataHeader;
end;

Procedure RegisterFixedExportFormat;

begin
  ExportFormats.RegisterExportFormat(SFixedLengthExport,SFixedLengthDescription,SFixedLengthExtensions,TFixedLengthExporter);
end;

Procedure UnRegisterFixedExportFormat;

begin
  Exportformats.UnregisterExportFormat(SFixedLengthExport);
end;

end.

