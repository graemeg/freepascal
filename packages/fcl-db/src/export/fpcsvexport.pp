unit fpcsvexport;

{$mode objfpc}{$H+}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
  Classes, SysUtils, fpDBExport, csvreadwrite;
=======
  Classes, SysUtils, DB, fpDBExport;
>>>>>>> graemeg/fixes_2_2
=======
  Classes, SysUtils, DB, fpDBExport;
>>>>>>> origin/fixes_2_2

Type
  { TCSVFormatSettings }

  TCSVFormatSettings = Class(TExportFormatSettings)
  Private
    FDelimiter: String;
    FHeaderRow: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
    FIgnoreOuterWhiteSpace: Boolean;
    FRowDelimiter: String;
    FQuoteChar: Char;
  Public
    Constructor Create(DoInitSettings : Boolean); override;
    Procedure Assign(Source : TPersistent); override;
    // Kept for compatibility with older versions; please replace with QuoteChar
    Property StringQuoteChar : Char Read FQuoteChar Write FQuoteChar; deprecated 'Please replace with QuoteChar';
  Published
    // Properties
    // Delimiter between fields/columns. Traditionally , for CSV.
    Property FieldDelimiter : String Read FDelimiter Write FDelimiter;
    //If no, CSV is RFC 4180 compliant; if yes, it matches the unofficial Creativyst specification
    Property IgnoreOuterWhitespace : Boolean Read FIgnoreOuterWhiteSpace write FIgnoreOuterWhiteSpace;
    // Line ending to be used between rows of data (e.g. #13#10 for standard CSV)
    Property RowDelimiter : String Read FRowDelimiter Write FRowDelimiter;
    // Whether or not the file should have a header row with field names
    Property HeaderRow : Boolean Read FHeaderRow Write FHeaderRow default true;
    // If fields need to be surrounded by quotes, use this character (e.g. ")
    Property QuoteChar : Char Read FQuoteChar Write FQuoteChar;
=======
=======
>>>>>>> origin/fixes_2_2
    FQuoteStrings: TQuoteStrings;
    FRowDelimiter: String;
    FStringQuoteChar: String;
  Public
    Constructor Create(DoInitSettings : Boolean); override;
    Procedure Assign(Source : TPersistent); override;
  Published
    // Properties
    Property FieldDelimiter : String Read FDelimiter Write FDelimiter;
    Property RowDelimiter : String Read FRowDelimiter Write FRowDelimiter;
    Property HeaderRow : Boolean Read FHeaderRow Write FHeaderRow default true;
    Property QuoteStrings : TQuoteStrings Read FQuoteStrings Write FQuoteStrings;
    Property StringQuoteChar : String Read FStringQuoteChar Write FStringQuoteChar;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  { TCustomCSVExporter }

  TCustomCSVExporter = Class(TCustomFileExporter)
  private
<<<<<<< HEAD
<<<<<<< HEAD
    FCSVOut: TCSVBuilder;
    function GetCSVFormatsettings: TCSVFormatSettings;
=======
    FCurrentRow:String;
    function GetCSVFormatsettings: TCSVFormatSettings;
    procedure OutputRow(const ARow: String);
>>>>>>> graemeg/fixes_2_2
=======
    FCurrentRow:String;
    function GetCSVFormatsettings: TCSVFormatSettings;
    procedure OutputRow(const ARow: String);
>>>>>>> origin/fixes_2_2
    procedure SetCSVFormatSettings(const AValue: TCSVFormatSettings);
  Protected
    Function CreateFormatSettings : TCustomExportFormatSettings; override;
    Procedure DoBeforeExecute; override;
    Procedure DoAfterExecute; override;
    Procedure DoDataHeader; override;
<<<<<<< HEAD
<<<<<<< HEAD
=======
    Procedure DoDataRowStart; override;
>>>>>>> graemeg/fixes_2_2
=======
    Procedure DoDataRowStart; override;
>>>>>>> origin/fixes_2_2
    Procedure ExportField(EF : TExportFieldItem); override;
    Procedure DoDataRowEnd; override;
  Public
    Constructor Create(Aowner : TComponent); override;
    Property FormatSettings : TCSVFormatSettings Read GetCSVFormatsettings Write SetCSVFormatSettings;
  end;

  { TCSVExporter }
  

  TCSVExporter = Class(TCustomCSVExporter)
  Published
    Property FileName;
    Property Dataset;
    Property ExportFields;
    Property FromCurrent;
    Property RestorePosition;
    Property FormatSettings;
    Property OnExportRow;
  end;

Procedure RegisterCSVExportFormat;
Procedure UnRegisterCSVExportFormat;

Const
  SCSVExport      = 'CSV';
  SCSVExtensions  = '.csv;.txt';

ResourceString
  SCSVDescription = 'Comma-Separated Values (CSV)';


implementation

{ TCustomCSVExporter }

procedure TCustomCSVExporter.DoBeforeExecute;
begin
  inherited DoBeforeExecute;
<<<<<<< HEAD
<<<<<<< HEAD
  FCSVOut:=TCSVBuilder.Create;
  if (FormatSettings.FieldDelimiter<>'') then
    FCSVOut.Delimiter:=FormatSettings.FieldDelimiter[1];
  FCSVOut.IgnoreOuterWhitespace:=FormatSettings.IgnoreOuterWhitespace;
  FCSVOut.LineEnding:=FormatSettings.RowDelimiter;
  FCSVOut.QuoteChar:=FormatSettings.QuoteChar;
  OpenTextFile;
  FCSVOut.SetOutput(Stream); //output to the export stream
=======
  OpenTextFile;
>>>>>>> graemeg/fixes_2_2
=======
  OpenTextFile;
>>>>>>> origin/fixes_2_2
end;

procedure TCustomCSVExporter.DoAfterExecute;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FCSVOut.Free;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  CloseTextFile;
  inherited DoAfterExecute;
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
procedure TCustomCSVExporter.OutputRow(Const ARow : String);

Var
  RD : String;

begin
  RD:=FormatSettings.RowDelimiter;
  If (RD='') then
    Writeln(TextFile,ARow)
  else
    Write(TextFile,ARow,RD)
end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

function TCustomCSVExporter.GetCSVFormatsettings: TCSVFormatSettings;
begin
  Result:=TCSVFormatSettings(Inherited FormatSettings)
end;

procedure TCustomCSVExporter.SetCSVFormatSettings(
  const AValue: TCSVFormatSettings);
begin
  Inherited FormatSettings:=AValue;
end;

function TCustomCSVExporter.CreateFormatSettings: TCustomExportFormatSettings;
begin
  Result:=TCSVFormatSettings.Create(False)
end;


procedure TCustomCSVExporter.DoDataHeader;

Var
<<<<<<< HEAD
<<<<<<< HEAD
=======
  S : String;
>>>>>>> graemeg/fixes_2_2
=======
  S : String;
>>>>>>> origin/fixes_2_2
  I : Integer;

begin
  If FormatSettings.HeaderRow then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    For I:=0 to ExportFields.Count-1 do
      begin
      FCSVOut.AppendCell(ExportFields[i].ExportedName);
      end;
    FCSVOut.AppendRow; //close off with line ending
=======
=======
>>>>>>> origin/fixes_2_2
    S:='';
    For I:=0 to ExportFields.Count-1 do
      begin
      If (S<>'') then
        S:=S+FormatSettings.FieldDelimiter;
      S:=S+ExportFields[i].ExportedName;
      end;
    OutputRow(S);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  inherited DoDataHeader;
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomCSVExporter.ExportField(EF: TExportFieldItem);
begin
  FCSVOut.AppendCell(FormatField(EF.Field));
end;

procedure TCustomCSVExporter.DoDataRowEnd;
begin
  FCSVOut.AppendRow; //Line ending
=======
=======
>>>>>>> origin/fixes_2_2
procedure TCustomCSVExporter.DoDataRowStart;
begin
  FCurrentRow:='';
end;

procedure TCustomCSVExporter.ExportField(EF: TExportFieldItem);

  Function HaveSpace(Const S : String;QS : TQuoteStrings) : Boolean;

  begin
    Result:=(qsSpace in QS) and (Pos(' ',S)<>0)
  end;

  Function HaveDelimiter(Const S : String;QS : TQuoteStrings) : Boolean;

  Var
    FD : String;

  begin
    Result:=(qsDelimiter in QS);
    If Result then
      begin
      FD:=FormatSettings.FieldDelimiter;
      Result:=(FD<>'') and (Pos(FD,S)<>0);
      end;
  end;

Var
  S,C : String;
  QS  : TQuoteStrings;

begin
  S:=FormatField(EF.Field);
  QS:=FormatSettings.QuoteStrings;
  If (EF.Field.DataType in StringFieldTypes) and (QS<>[]) then
    begin
    If (qsAlways in QS) or HaveSpace(S,QS) or HaveDelimiter(S,QS) then
      begin
      C:=FormatSettings.StringQuoteChar;
      S:=C+S+C;
      end;
    end;
  If (FCurrentRow<>'') then
    FCurrentRow:=FCurrentRow+FormatSettings.FieldDelimiter;
  FCurrentRow:=FCurrentRow+S;
end;


procedure TCustomCSVExporter.DoDataRowEnd;
begin
  OutputRow(FCurrentRow);
  FCurrentRow:='';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

constructor TCustomCSVExporter.Create(Aowner: TComponent);
begin
  inherited Create(Aowner);
end;

{ TCSVFormatSettings }

constructor TCSVFormatSettings.Create(DoInitSettings: Boolean);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  // These defaults are meant to be Excel CSV compatible
  inherited Create(DoInitSettings);
  FHeaderRow:=True;
  FDelimiter:=',';
  FQuoteChar:='"';
  FRowDelimiter:=LineEnding;
=======
=======
>>>>>>> origin/fixes_2_2
  inherited Create(DoInitSettings);
  FHeaderRow:=True;
  FDelimiter:=',';
  FStringQuoteChar:='"';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TCSVFormatSettings.Assign(Source: TPersistent);

Var
  FS : TCSVFormatsettings;

begin
  If (Source is TCSVFormatSettings) then
    begin
    FS:=Source as TCSVFormatSettings;
    FDelimiter:=FS.FDelimiter;
<<<<<<< HEAD
<<<<<<< HEAD
    FHeaderRow:=FS.FHeaderRow;
    FRowDelimiter:=FS.FRowDelimiter;
    FQuoteChar:=FS.FQuoteChar;
=======
=======
>>>>>>> origin/fixes_2_2
    FHeaderRow:=FS.FHEaderRow;
    FQuoteStrings:=FS.FQuoteStrings;
    FRowDelimiter:=FS.FRowDelimiter;
    FStringQuoteChar:=FS.FStringQuoteChar;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  inherited Assign(Source);
end;

Procedure RegisterCSVExportFormat;

begin
  ExportFormats.RegisterExportFormat(SCSVExport,SCSVDescription,SCSVExtensions,TCSVExporter);
end;

Procedure UnRegisterCSVExportFormat;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  ExportFormats.UnRegisterExportFormat(SCSVExport);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


end.
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
