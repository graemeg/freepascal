unit fpSQLExport;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, DB, fpDBExport;
  
Type
  { TSQLExportFieldItem }

  TSQLExportFieldItem = Class(TExportFieldItem)
  private
    FKeyField: Boolean;
  Public
    Procedure Assign(Source : TPersistent); override;
  Published
    Property KeyField : Boolean Read FKeyField Write FKeyField;
  end;

  TSQLEscapeStyle = (sesFirebird,sesMySQL);
  TSQLStatementKind = (skInsert,skFullInsert,skUpdate);
  { TSQLFormatSettings }

  TSQLFormatSettings = class(TExportFormatSettings)
  private
    FEscapeStyle: TSQLEscapeStyle;
    FQuoteChar: String;
    FQuoteIdentifiers: Boolean;
    FStatementKind: TSQLStatementKind;
    FTableName: String;
  Public
    Procedure Assign(Source : TPersistent); override;
  Published
    Property EscapeStyle : TSQLEscapeStyle Read FEscapeStyle Write FEscapeStyle;
    Property StatementKind : TSQLStatementKind Read FStatementKind Write FStatementKind;
    Property QuoteIdentifiers : Boolean Read FQuoteIdentifiers Write FQuoteIdentifiers;
    Property QuoteChar : String Read FQuoteChar Write FQuoteChar;
    Property TableName : String Read FTableName Write FTableName;
  end;

  { TCustomSQLExporter }

  TCustomSQLExporter = Class(TCustomFileExporter)
  private
    FES : TSQLEscapeStyle;
    FQI : Boolean;
    FTN : String;
    FSK : TSQLStatementKind;
    FQC : String;
    FUS : Boolean;
    FCurrentRow : String;
    function GetSQLFormatsettings: TSQLFormatSettings;
    function QuoteField(const S: String): String;
    function SQLValue(F: TField): String;
    procedure SetSQLFormatSettings(const AValue: TSQLFormatSettings);
  Protected
    Function CreateFormatSettings : TCustomExportFormatSettings; override;
    Function  CreateExportFields : TExportFields; override;
    Procedure DoBeforeExecute; override;
    Procedure DoAfterExecute; override;
    Procedure DoDataRowStart; override;
    Procedure ExportField(EF : TExportFieldItem); override;
    Procedure DoDataRowEnd; override;
    Function MaybeQuote(Const S : String) : String;
    Procedure OutputRow(Const ARow : String);
  Public
    Property FormatSettings : TSQLFormatSettings Read GetSQLFormatsettings Write SetSQLFormatSettings;
  end;

  TSQLExporter = Class(TCustomSQLExporter)
  Published
    Property FileName;
    Property Dataset;
    Property ExportFields;
    Property FromCurrent;
    Property RestorePosition;
    Property FormatSettings;
    Property OnExportRow;
  end;

Procedure RegisterSQLExportFormat;
Procedure UnRegisterSQLExportFormat;

Const
  SSQLExport     = 'SQL';
  SSQLExtensions = '.sql';
  
Resourcestring
  SSQLDescription         = 'SQL INSERT/Update Statements';
  SErrMissingTableName    = 'No tablename set for SQL Export';
  SErrNoKeyFieldForUpdate = 'No key fields defined for update statement in SQL export';

implementation

{ TSQLFormatSettings }

procedure TSQLFormatSettings.Assign(Source: TPersistent);

Var
  FS : TSQLFormatSettings;

begin
  if (Source is TSQLFormatSettings) then
    begin
    FS:=(Source as TSQLFormatSettings);
    EscapeStyle:=FS.EscapeStyle;
    StatementKind:=FS.StatementKind;
    QuoteIdentifiers:=FS.QuoteIdentifiers;
    QuoteChar:=FS.QuoteChar;
    TableName:=FS.TableName;
    end;
  inherited Assign(Source);
end;

{ TCustomSQLExporter }

function TCustomSQLExporter.GetSQLFormatsettings: TSQLFormatSettings;
begin
  Result:=TSQLFormatSettings(Inherited Formatsettings);
end;

procedure TCustomSQLExporter.SetSQLFormatSettings(
  const AValue: TSQLFormatSettings);
begin
  Inherited FormatSettings:=AValue;
end;

function TCustomSQLExporter.CreateFormatSettings: TCustomExportFormatSettings;
begin
  Result:=TSQLFOrmatSettings.Create(False);
end;

function TCustomSQLExporter.CreateExportFields: TExportFields;
begin
  Result:=TExportFields.Create(TSQLExportFieldItem);
end;

procedure TCustomSQLExporter.DoBeforeExecute;

Var
  OK : Boolean;
  I : Integer;

begin
  If (FormatSettings.TableName='') then
    ExportError(SErrMissingTableName);
  if (FormatSettings.StatementKind=skUpdate) then
    begin
    OK:=False;
    I:=0;
    While (I<ExportFields.Count) and Not OK do
      begin
      OK:=TSQLExportFieldItem(ExportFields[i]).KeyField;
      Inc(I);
      end;
    If Not OK then
      ExportError(SErrNoKeyFieldForUpdate);
    end;
  inherited DoBeforeExecute;
  OpenTextFile;
  FES:=FormatSettings.EscapeStyle;
  FQI:=FormatSettings.QuoteIdentifiers;
  FTN:=FormatSettings.TableName;
  FSK:=FormatSettings.StatementKind;
  FQC:=FormatSettings.QuoteChar;
end;

procedure TCustomSQLExporter.DoAfterExecute;
begin
  CloseTextFile;
  inherited DoAfterExecute;
end;

procedure TCustomSQLExporter.DoDataRowStart;
begin
  FCurrentRow:=''
end;

function TCustomSQLExporter.QuoteField (Const S : String) : String;

begin
  If FES=sesFirebird then
    Result:=StringReplace(S,'''','''''',[rfReplaceAll])
  else
    Result:=StringReplace(S,'''','\''',[rfReplaceAll]);
end;

Function TCustomSQLExporter.SQLValue(F : TField) : String;

begin
  Result:=FormatField(F);
<<<<<<< HEAD
<<<<<<< HEAD
  If (F.DataType in BlobFieldTypes+StringFieldTypes+MemoFieldTypes+DateFieldTypes) then
=======
  If (F.DataType in StringFieldTypes+DateFieldTypes) then
>>>>>>> graemeg/fixes_2_2
=======
  If (F.DataType in StringFieldTypes+DateFieldTypes) then
>>>>>>> origin/fixes_2_2
    Result:=''''+QuoteFIeld(Result)+'''';
end;

procedure TCustomSQLExporter.ExportField(EF: TExportFieldItem);

Var
  S : string;

begin
  If (FSK<>skUpdate) or (Not TSQLExportFieldItem(EF).KeyField) then
    begin
    If (FCurrentRow<>'') then
      FCurrentRow:=FcurrentRow+', ';
    S:=SQLValue(EF.FIeld);
    If FSK<>skUpdate then
      FCurrentRow:=FCurrentRow+S
    else
      FCurrentRow:=FCurrentRow+MaybeQuote(EF.ExportedName)+'='+S;
    end;
end;

function TCustomSQLExporter.MaybeQuote (Const S : String) : String;

begin
  Result:=S;
  If FQI then
    Result:=FQC+Result+FQC;
end;

procedure TCustomSQLExporter.OutputRow(const ARow: String);
begin
  Writeln(TextFile,ARow);
end;

procedure TCustomSQLExporter.DoDataRowEnd;

Var
  S,T : String;
  I   : Integer;
  EF  : TExportFieldItem;

begin
  If FSK<>skUpdate then
    begin
    S:='INSERT INTO '+MaybeQuote(FTN);
    If FSK=skFullInsert then
      begin
      S:=S+' (';
      T:='';
      For I:=0 to ExportFields.Count-1 do
        begin
        EF:=ExportFields[i];
        If EF.Enabled then
          begin
          If (T<>'') then
            T:=T+', ';
          T:=T+MaybeQuote(EF.ExportedName);
          end;
        end;
      S:=S+T+')';
      OutputRow(S);
      S:=''
      end;
    S:=S+' VALUES ('+FCurrentRow+');';
    end
  else
    begin
    S:='UPDATE '+MaybeQuote(FTN)+' SET '+FCurrentRow;
    OutputRow(S);
    S:='(';
    For I:=0 to ExportFields.Count-1 do
      begin
      EF:=ExportFields[i];
      If TSQLExportFieldItem(EF).KeyField then
        begin
        If (S<>'(') then
          S:=S+') AND (';
        S:=S+MaybeQuote(EF.ExportedName)+' = '+SQLValue(EF.Field);
        end;
      end;
    S:=' WHERE '+S+');';
    end;
  OutputRow(S);
end;

{ TSQLExportFieldItem }

procedure TSQLExportFieldItem.Assign(Source: TPersistent);

Var
  FI : TSQLExportFieldItem;

begin
  If Source is TSQLExportFieldItem then
    begin
    FI:=Source as TSQLExportFieldItem;
    KeyField:=FI.KeyField;
    end;
  inherited Assign(Source);
end;

Procedure RegisterSQLExportFormat;

begin
  ExportFormats.RegisterExportFormat(SSQLExport,SSQLDescription,SSQLExtensions,TSQLExporter);
end;

Procedure UnRegisterSQLExportFormat;

begin
  ExportFormats.UnRegisterExportFormat(SSQLExport);
end;

end.

