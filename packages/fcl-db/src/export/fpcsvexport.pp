unit fpcsvexport;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpDBExport, csvreadwrite;

Type
  { TCSVFormatSettings }

  TCSVFormatSettings = Class(TExportFormatSettings)
  Private
    FDelimiter: String;
    FHeaderRow: Boolean;
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
  end;

  { TCustomCSVExporter }

  TCustomCSVExporter = Class(TCustomFileExporter)
  private
    FCSVOut: TCSVBuilder;
    function GetCSVFormatsettings: TCSVFormatSettings;
    procedure SetCSVFormatSettings(const AValue: TCSVFormatSettings);
  Protected
    Function CreateFormatSettings : TCustomExportFormatSettings; override;
    Procedure DoBeforeExecute; override;
    Procedure DoAfterExecute; override;
    Procedure DoDataHeader; override;
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
  FCSVOut:=TCSVBuilder.Create;
  if (FormatSettings.FieldDelimiter<>'') then
    FCSVOut.Delimiter:=FormatSettings.FieldDelimiter[1];
  FCSVOut.IgnoreOuterWhitespace:=FormatSettings.IgnoreOuterWhitespace;
  FCSVOut.LineEnding:=FormatSettings.RowDelimiter;
  FCSVOut.QuoteChar:=FormatSettings.QuoteChar;
  OpenTextFile;
  FCSVOut.SetOutput(Stream); //output to the export stream
end;

procedure TCustomCSVExporter.DoAfterExecute;
begin
  FCSVOut.Free;
  CloseTextFile;
  inherited DoAfterExecute;
end;


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
  I : Integer;

begin
  If FormatSettings.HeaderRow then
    begin
    For I:=0 to ExportFields.Count-1 do
      if ExportFields[i].Enabled then
        FCSVOut.AppendCell(ExportFields[i].ExportedName);
    FCSVOut.AppendRow; //close off with line ending
    end;
  inherited DoDataHeader;
end;


procedure TCustomCSVExporter.ExportField(EF: TExportFieldItem);
begin
  FCSVOut.AppendCell(FormatField(EF.Field));
end;

procedure TCustomCSVExporter.DoDataRowEnd;
begin
  FCSVOut.AppendRow; //Line ending
end;

constructor TCustomCSVExporter.Create(Aowner: TComponent);
begin
  inherited Create(Aowner);
end;

{ TCSVFormatSettings }

constructor TCSVFormatSettings.Create(DoInitSettings: Boolean);
begin
  // These defaults are meant to be Excel CSV compatible
  inherited Create(DoInitSettings);
  FHeaderRow:=True;
  FDelimiter:=',';
  FQuoteChar:='"';
  FRowDelimiter:=LineEnding;
end;

procedure TCSVFormatSettings.Assign(Source: TPersistent);

Var
  FS : TCSVFormatsettings;

begin
  If (Source is TCSVFormatSettings) then
    begin
    FS:=Source as TCSVFormatSettings;
    FDelimiter:=FS.FDelimiter;
    FHeaderRow:=FS.FHeaderRow;
    FRowDelimiter:=FS.FRowDelimiter;
    FQuoteChar:=FS.FQuoteChar;
    end;
  inherited Assign(Source);
end;

Procedure RegisterCSVExportFormat;

begin
  ExportFormats.RegisterExportFormat(SCSVExport,SCSVDescription,SCSVExtensions,TCSVExporter);
end;

Procedure UnRegisterCSVExportFormat;

begin
  ExportFormats.UnRegisterExportFormat(SCSVExport);
end;


end.
