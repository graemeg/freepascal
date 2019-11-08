unit fprtfexport;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, DB, fpdbexport;

Type
  TRTFExportOption = (reHeaderRow,reHeaderLine,reTopLine,reBottomLine);
  TRTFExportOptions = Set of TrtfExportOption;
  
  { TRTFExportFormatSettings }

  TRTFExportFormatSettings = Class(TExportFormatSettings)
  Private
    FOptions : TRTFExportOptions;
  Public
    Constructor Create(DoInitSettings : Boolean); override;
    Procedure Assign(Source : TPersistent); override;
  Published
    // Properties
    Property Options : TRTFExportOptions Read FOptions Write FOptions;
  end;

  { TRTFExportFieldItem }
  TRTFExportFieldItem = Class(TExportFieldItem)
  private
    FLineAfter: Boolean;
    FLineBefore: Boolean;
    FWidth: Integer;
    FAlign: TAlignment;
  Public
    Procedure Assign(Source : TPersistent); override;
  Published
    Property Width : Integer Read FWidth Write FWidth;
    Property Align: TAlignment Read FAlign write FAlign;
    Property LineBefore : Boolean Read FLineBefore Write FLineBefore;
    Property LineAfter : Boolean Read FLineAfter Write FLineAfter;
  end;

  { TCustomRTFExporter }

  TCustomRTFExporter = Class(TCustomFileExporter)
  Private
    FCurrentRow : String;
    FEO : TRTFExportOptions;
    FTD : String; // Tabular(X) Table definition string
    FTH : String; // Table header row
    FTN : String; // Tabular environment name (for closing)
    function GetRTFFormatsettings: TRTFExportFormatSettings;
    function MakeCell(S: UTF8String; LineBefore, LineAfter: Boolean): string;
    procedure SetRTFFormatSettings(const AValue: TRTFExportFormatSettings);
  Protected
    function EscapeRTF(S: UTF8String): String;
    procedure OutputRow(const ARow: String); virtual;
    procedure OutputTableEnd; virtual;
    procedure OutputTableStart; virtual;
    procedure CloseDocument;  virtual;
    procedure OpenDocument; virtual;
    Function CreateFormatSettings : TCustomExportFormatSettings; override;
    Procedure BuildDefaultFieldMap(AMap : TExportFields); override;
    Function  CreateExportFields : TExportFields; override;
    Procedure DoDataHeader; override;
    Procedure DoDataFooter; override;
    Procedure DoBeforeExecute; override;
    Procedure DoAfterExecute; override;
    Procedure DoDataRowStart; override;
    Procedure ExportField(EF : TExportFieldItem); override;
    Procedure DoDataRowEnd; override;
  Public
    Property FormatSettings : TRTFExportFormatSettings Read GetRTFFormatsettings Write SetRTFFormatSettings;
  end;

  TRTFExporter = Class(TCustomRTFExporter)
  Published
    Property FileName;
    Property Dataset;
    Property ExportFields;
    Property FromCurrent;
    Property RestorePosition;
    Property FormatSettings;
    Property OnExportRow;
  end;

Procedure RegisterRTFExporter;
Procedure UnRegisterRTFExporter;

Const
  SRTFExport    = 'RTF export';
  SRTFExportExt = '.rtf';
  

Resourcestring
  SRTFExportDescr = 'Export to RTF table';

implementation

procedure RegisterRTFExporter;
begin
  ExportFormats.RegisterExportFormat(SRTFExport,SRTFExportDescr,SRTFExportExt,TRTFExporter);
end;

procedure UnRegisterRTFExporter;
begin
  ExportFormats.UnRegisterExportFormat(SRTFExport);
end;

{ TCustomRTFExporter }
function TCustomRTFExporter.EscapeRTF(S: UTF8String): String;

Const
  NeedEscape : TSysCharSet = ['{', '}', '\'];

var
  SS : UnicodeString;
  Ch : UnicodeChar;

begin
  SS:=UTF8Decode(S);
  Result:='';
  For Ch in SS do
    begin
    if CharInSet(Ch, NeedEscape) then
      Result:=Result+'\';
    if Ord(Ch)>255 then
      Result:=Result+'\u'+IntToStr(Ord(Ch))+'?'
    else
      Result:=Result+Utf8Encode(Ch);
    end;
end;

function TCustomRTFExporter.GetRTFFormatsettings: TRTFExportFormatSettings;
begin
  Result:=TRTFExportFormatSettings(Inherited FormatSettings)
end;

procedure TCustomRTFExporter.OutputRow(const ARow: String);
begin
  Writeln(TextFile,ARow);
end;

procedure TCustomRTFExporter.BuildDefaultFieldMap(AMap: TexportFields);

Const
  FieldWidths : Array[TFieldType] of integer
              = (-1,0,3,10,5,
                  1,20,20,20,10,8,20,
                  0,0,10,0,0,0,0,
                  0,0,0,0,0,
                  0,0,0,0,0,
                  0,0,0,0,0,
                  0,0,0,0,0,0);

Var
  I  : Integer;
  FL : TRTFExportFieldItem;
  F : TField;
  W : Integer;
  
begin
  inherited BuildDefaultFieldMap(AMap);
  For I:=0 to AMap.Count-1 do
    begin
    FL:=TRTFExportFieldItem(AMAP[i]);
    F:=Dataset.Fields[i];
    W:= FieldWidths[F.DataType];
    If (W>0) then
      FL.Width:=W
    else if (W=0) then
      begin
      if (F.DataType in StringFieldTypes) then
        FL.Width:=F.Size;
      end;
    If (F.DataType in IntFieldTypes) then
      Fl.Align:=taRightJustify;
    end;

end;

function TCustomRTFExporter.CreateExportFields: TexportFields;
begin
  Result:=TexportFields.Create(TRTFExportFieldItem);
end;

procedure TCustomRTFExporter.DoDataHeader;

Var
  I : Integer;
  B2 : Boolean;
  EF : TRTFExportFieldItem;
  
begin
  B2:=reHeaderRow in FEO;
  If B2 then
  For I:=0 to ExportFields.Count-1 do
    begin
    EF:=TRTFExportFieldItem(ExportFields[i]);
    If EF.Enabled then
      begin
      FTH:=FTH+MakeCell(EF.ExportedName,EF.lineBefore,EF.LineAfter);
      end;
    end;
  OutPutTableStart;
  inherited DoDataHeader;
end;

procedure TCustomRTFExporter.DoDataFooter;

begin
  OutPutTableEnd;
  Inherited DoDataFooter;
end;

procedure TCustomRTFExporter.OutputTableEnd;

begin
  OutputRow('}');
end;

procedure TCustomRTFExporter.OutputTableStart;

Var
  S : String;
  I : Integer;

begin
  OutputRow('\par{');
  if (reHeaderLine in FEO) then
    S := '\trbrdrl\brdrs\brdrw1\trbrdrr\brdrs\brdrw1'
  else
    S := '';
  If reHeaderRow in FEO then
    begin
    OutputRow('{\b\trowd'+S+'\trbrdrh\brdrs\trbrdrv\brdrs');
    OutputRow(FTH);
    OutputRow('\row}');
    end;
end;

procedure TCustomRTFExporter.SetRTFFormatSettings(
  const AValue: TRTFExportFormatSettings);
begin
  Inherited FormatSettings:=AValue
end;

function TCustomRTFExporter.CreateFormatSettings: TCustomExportFormatSettings;
begin
  Result:=TRTFExportFormatSettings.Create(False);
end;

procedure TCustomRTFExporter.DoBeforeExecute;
begin
  inherited DoBeforeExecute;
  OpenTextFile;
  FEO:=FormatSettings.Options;
  FTD:='';
  FTH:='';
  OpenDocument;
end;

procedure TCustomRTFExporter.DoAfterExecute;
begin
  CloseDocument;
  CloseTextFile;
  inherited DoAfterExecute;
end;

procedure TCustomRTFExporter.DoDataRowStart;
begin
  FCurrentRow:='';
  inherited DoDataRowStart;
end;

Function TCustomRTFExporter.MakeCell(S : UTF8String; LineBefore,LineAfter : Boolean) : string;

begin
  Result:='\pard\intbl '+EscapeRTF(S)+'\cell';
end;

procedure TCustomRTFExporter.ExportField(EF: TExportFieldItem);

Var
  S : String;
  RF : TRTFExportFieldItem;
  
begin
  RF:=EF as TRTFExportFieldItem;
  S:=MakeCell(FormatField(EF.Field),RF.LineBefore,RF.LineAfter);
  FCurrentRow:=FCurrentRow+S;
end;

procedure TCustomRTFExporter.DoDataRowEnd;

begin
  OutputRow('\trowd\trbrdrh\brdrs\trbrdrv\brdrs');
  OutputRow(FCurrentRow);
  OutputRow('\row');
end;

procedure TCustomRTFExporter.OpenDocument;

begin
  OutputRow('{\rtf1');
  OutputRow('{\fonttbl');
    OutputRow('{\f0\fswiss Helvetica{\*\falt Arial};}');
    OutputRow('{\f1\fmodern Courier{\*\falt Courier New};}');
    OutputRow('{\f2\froman Times{\*\falt Times New Roman};}');
  OutputRow('}{\stylesheet');
    OutputRow('{\s1\li0\fi0\ql\sb240\sa60\keepn\f2\b\fs32 Section Title;}');
    OutputRow('{\s2\ql\sb30\sa30\keepn\b0\i0\scaps1\f1\fs28 Table Title;}');
    OutputRow('{\s3\li0\fi0\qc\sb240\sa60\keepn\f2\b\scaps1\fs28 Listing Title;}');
    OutputRow('{\s4\li30\fi30\ql\f2\fs24 Listing Contents;}');
    OutputRow('{\s5\li0\fi0\ql\sb240\sa60\keepn\f2\b\fs40 Chapter;}');
    OutputRow('{\s6\li0\fi0\ql\sb240\sa60\keepn\f2\b\fs32 Section;}');
    OutputRow('{\s7\li0\fi0\ql\sb240\sa60\keepn\f2\b\fs28 Subsection;}');
    OutputRow('{\s8\li0\fi0\ql\sb240\sa60\keepn\f2\b\fs24 Subsubsection;}');
    OutputRow('{\s9\li30\fi10\ql\sb60\keepn\f2\fs24 Description titles;}');
    OutputRow('{\s10\li30\fi30\ql\fs24 Description;}');
    OutputRow('{\s11\li0\fi0\ql\fs24 Source Example;}');
  OutputRow('}');
end;

procedure TCustomRTFExporter.CloseDocument;
begin
  OutputRow('}');
end;

{ TRTFExportFormatSettings }

constructor TRTFExportFormatSettings.Create(DoInitSettings: Boolean);
begin
  inherited Create(DoInitSettings);
  FOptions:=[reHeaderRow,reTopLine,reBottomLine]
end;

procedure TRTFExportFormatSettings.Assign(Source: TPersistent);

Var
  FS : TRTFExportFormatSettings;

begin
  If (Source is TRTFExportFormatSettings) then
    begin
    FS:=Source as TRTFExportFormatSettings;
    Options:=FS.OPtions;
    end;
  inherited Assign(Source);
end;

{ TRTFExportFieldItem }

procedure TRTFExportFieldItem.Assign(Source: TPersistent);

Var
  Fi : TRTFExportFieldItem;

begin
  If (Source is TRTFExportFieldItem) then
    begin
    FI:=Source as TRTFExportFieldItem;
    Width:=FI.Width;
    Align:=FI.Align;
    LineBefore:=FI.LineBefore;
    LineAfter:=FI.LineAfter;
    end;
  inherited Assign(Source);
end;

end.

