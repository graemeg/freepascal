unit fpSimpleXMLExport;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, DB, fpDBExport;
  
Type
  { TSimpleXMLFormatSettings }

  TSimpleXMLFormatSettings = Class(TExportFormatSettings)
  private
    FFieldAsAttribute: Boolean;
    FIndentSize: Integer;
    FRowElementName: String;
    FStartNodePath: String;
  Public
    Procedure Assign(Source : TPersistent); override;
  Published
    Property StartNodePath : String Read FStartNodePath Write FStartNodePath;
    Property RowElementName : String Read FRowElementName Write FRowElementName;
    Property FieldAsAttributes : Boolean Read FFieldAsAttribute Write FFieldAsAttribute;
    Property IndentSize : Integer Read FIndentSize Write FIndentSize;
  end;

  { TCustomSimpleXMlExporter }
  TCustomSimpleXMLExporter = Class(TCustomFileExporter)
  Private
    FCurrentRow : UTF8String;
    FIndent : UTF8String;
    FRowElementName : UTF8String;
    FRootNode : UTF8String;
    FAA : Boolean;
    FIS : Integer;
    function AttrString(S: UTF8String): UTF8String;
    procedure DecIndent;
    function GetXMLFormatsettings: TSimpleXMLFormatSettings;
    procedure IncIndent;
    procedure OutputRow(const ARow: UTF8String);
    procedure SetXMLFormatSettings(const AValue: TSimpleXMLFormatSettings);
    function TextString(S: UTF8String): UTF8String;
  Protected
    Function  CreateFormatSettings : TCustomExportFormatSettings; override;
    Procedure DoBeforeExecute; override;
    Procedure DoAfterExecute; override;
    Procedure DoDataRowStart; override;
    Procedure DoDataHeader; override;
    Procedure DoDataFooter; override;
    Procedure ExportField(EF : TExportFieldItem); override;
    Procedure DoDataRowEnd; override;
  Public
    Property FormatSettings : TSimpleXMLFormatSettings Read GetXMLFormatsettings Write SetXMLFormatSettings;
  end;

  TSimpleXMLExporter = Class(TCustomSimpleXMLExporter)
  Published
    Property FileName;
    Property Dataset;
    Property ExportFields;
    Property FromCurrent;
    Property RestorePosition;
    Property FormatSettings;
    Property OnExportRow;
  end;

Procedure RegisterSimpleXMLExportFormat;
Procedure UnRegisterSimpleXMLExportFormat;

Const
  SSimpleXML             = 'SimpleXml';
  SSimpleXMLExtensions   = '.xml';

Resourcestring
  SSimpleXMLDescription = 'Simple ASCII XML file';

implementation

{ TCustomSimpleXMLExporter }

procedure TCustomSimpleXMLExporter.OutputRow(const ARow: UTF8String);
begin
  Writeln(TextFile,FIndent,ARow);
end;

function TCustomSimpleXMLExporter.GetXMLFormatsettings: TSimpleXMLFormatSettings;
begin
  Result:=TSimpleXMLFormatSettings(Inherited FormatSettings);
end;

procedure TCustomSimpleXMLExporter.SetXMLFormatSettings(
  const AValue: TSimpleXMLFormatSettings);
begin
   Inherited FormatSettings:=AValue;
end;

function TCustomSimpleXMLExporter.CreateFormatSettings: TCustomExportFormatSettings;
begin
  Result:=TSimpleXMLFormatSettings.Create(False);
end;

procedure TCustomSimpleXMLExporter.DoBeforeExecute;
begin
  inherited DoBeforeExecute;
  OpenTextFile;
  FRowElementName:=FormatSettings.RowElementName;
  If FRowElementname='' then
    FRowElementName:='ROW';
  FRootNode:=Formatsettings.StartNodePath;
  If (FRootNode='') or (FRootNode='/')then
    FRootNode:='/ROWDATA/';
  FIS:=FormatSettings.IndentSize;
  FAA:=Formatsettings.FieldAsAttributes;
  FIndent:='';
end;

procedure TCustomSimpleXMLExporter.DoAfterExecute;
begin
  CloseTextFile;
  inherited DoAfterExecute;
end;

procedure TCustomSimpleXMLExporter.DoDataRowStart;
begin
  If FAA then
    FCurrentRow:='<'+FRowElementName
  else
    begin
    FCurrentRow:='';
    OutputRow('<'+FRowElementName+'>');
    IncIndent;
    end;
end;

const
  QuotStr : UTF8String = '&quot;';
  AmpStr : UTF8String = '&amp;';
  ltStr : UTF8String = '&lt;';
  gtStr : UTF8String = '&gt;';

Procedure AddToResult(Var Res : UTF8String; S : UTF8String; P : integer; Var J : Integer; Const Add : UTF8String);

begin
  Res:=Res+Copy(S,J,P-J);
  If (Add<>'') then
    Res:=Res+Add;
  J:=P+1;
end;

Function TCustomSimpleXMLExporter.AttrString(S : UTF8String) : UTF8String;

Var
  I,J : Integer;


begin
  Result:='';
  J:=1;
  For I:=1 to Length(S) do
    case S[i] of
      '"': AddToResult(Result,S,I,J,QuotStr);
      '&': AddToResult(Result,S,I,J,AmpStr);
      '<': AddToResult(Result,S,I,J,ltStr);
      #9 : AddToResult(Result,S,I,J,'&#x9;');
      #10: AddToResult(Result,S,I,J,'&#xA;');
      #13: AddToResult(Result,S,I,J,'&#xD;');
    end;
  AddToResult(Result,S,Length(S)+1,J,'');
end;

Function TCustomSimpleXMLExporter.TextString(S : UTF8String) : UTF8String;


Var
  I,J : Integer;

begin
  Result:='';
  J:=1;
  For I:=1 to Length(S) do
    case S[i] of
      '<': AddToResult(Result,S,I,J,ltStr);
      '>': AddToResult(Result,S,I,J,gtStr);
      '&': AddToResult(Result,S,I,J,AmpStr);
    end;
  AddToResult(Result,S,Length(S)+1,J,'');
end;

procedure TCustomSimpleXMLExporter.IncIndent;

begin
  If FIS>0 then
    FIndent:=FIndent+StringOfChar(' ',FIS);
end;

procedure TCustomSimpleXMLExporter.DecIndent;

begin
  If (FIS>0) and (length(FIndent)>=FIS) then
    Delete(FIndent,1,FIS);
end;

procedure TCustomSimpleXMLExporter.DoDataHeader;

Var
  S : UTF8String;
  P : Integer;

begin
  Writeln(TextFile,'<?xml version="1.0" encoding = "utf-8" ?>');
  S:=FRootNode;
  if S[Length(S)]<>'/' then
    S:=S+'/';
  If (S[1]='/') then
    Delete(S,1,1);
  Repeat
    P:=Pos('/',S);
    OutputRow('<'+Copy(S,1,P-1)+'>');
    Delete(S,1,P);
    IncIndent;
  Until (S='');
end;

procedure TCustomSimpleXMLExporter.DoDataFooter;

Var
  P,L : Integer;
  S : UTF8String;

begin
  S:=FRootNode;
  if (S[1]<>'/') then
    S:='/'+S;
  L:=Length(S);
  If (S[L]='/') then
    S:=Copy(S,1,L-1);
  Repeat
    L:=Length(S);
    P:=L;
    While (P>0) and (S[P]<>'/') do
      Dec(P);
    DecIndent;
    OutputRow('</'+Copy(S,P+1,L-P)+'>');
    S:=Copy(S,1,P-1);
  Until (S='');
  inherited DoDataFooter;
end;

procedure TCustomSimpleXMLExporter.ExportField(EF: TExportFieldItem);

Var
  S : UTF8String;

begin
  S:=FormatField(EF.Field);
  If FormatSettings.FieldAsAttributes then
    FCurrentRow:=FCurrentRow+' '+EF.ExportedName+'="'+AttrString(S)+'"'
  else
    begin
    FCurrentRow:='<'+EF.ExportedName+'>'+TextString(S)+'</'+EF.ExportedName+'>';
    OutputRow(FCurrentRow);
    end;
end;

procedure TCustomSimpleXMLExporter.DoDataRowEnd;

begin
  If FormatSettings.FieldAsAttributes then
    OutputRow(FCurrentRow+'/>')
  else
    begin
    DecIndent;
    OutputRow('</'+FRowElementName+'>');
    end;
  FCurrentRow:='';
  inherited DoDataRowEnd;
end;

{ TSimpleXMLFormatSettings }

procedure TSimpleXMLFormatSettings.Assign(Source: TPersistent);

Var
  XS : TSimpleXMLFormatSettings;

begin
  If Source is TSimpleXMLFormatSettings then
    begin
    Xs:=TSimpleXMLFormatSettings(Source);
    StartNodePath:=XS.StartNodePath;
    RowElementName:=XS.RowElementName;
    FieldAsAttributes:=XS.FieldAsAttributes;
    IndentSize:=XS.IndentSize;
    end;
  inherited Assign(Source);
end;

Procedure RegisterSimpleXMLExportFormat;

begin
  ExportFormats.RegisterExportFormat(SSimpleXML,SSimpleXMLDescription,SSimpleXMLExtensions,TSimpleXMLExporter);
end;

Procedure UnRegisterSimpleXMLExportFormat;

begin
  ExportFormats.UnregisterExportFormat(SSimpleXML);
end;

end.

