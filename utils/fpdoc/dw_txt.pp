{

    FPDoc  -  Free Pascal Documentation Tool
    Copyright (C) 2005 by Michael Van Canneyt

    * Text output generator

    See the file COPYING, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
{$mode objfpc}
{$H+}
unit dw_txt;
{$WARN 5024 off : Parameter "$1" not used}
interface

uses DOM, dGlobals, PasTree, dwriter;

const
  TxtHighLight : Boolean = False;
  TxtExtension   : String = '.txt';

Procedure CreateTxtDocForPackage(APackage: TPasPackage; AEngine: TFPDocEngine);

implementation

uses fpdocstrs, SysUtils, Classes, dwLinear;

Const
  MaxListLevel     = 10;
  DefaultLineWidth = 72;

Type
 { TTxtWriter }

  TTXTWriter = class(TLinearWriter)
  protected
    LineWidth : Integer;
    FCheckEOL : Boolean;
    FCurrentPos : Integer;
    FListLevel,
    FChapterCount,
    FSectionCount,
    FSubSectionCount,
    FSubSubSectionCount,
    FTableCount : Integer;
    FInVerbatim : Boolean;
    FLists : Array [0..MaxListLevel] of integer;
    Inlist,
    TableRowStartFlag,
    TableCaptionWritten: Boolean;
    procedure Write(const s: String); override;
    procedure WriteLn(const s: String); override;
    procedure NewLine;
    // Private methods
    procedure WriteLine(LineLength : Integer; DoubleLine : Boolean);
    Procedure WriteLine(DoubleLine : Boolean);
    procedure NewListLevel(Initial : Integer);
    procedure declistlevel;
    Procedure WriteUnderline(Const Msg : String; DoubleLine : Boolean);
    // Linear documentation methods overrides;
    procedure WriteLabel(Const S : String); override;
    procedure WriteIndex(Const S : String); override;
    Procedure WriteExampleFile(FN : String); override;
    procedure StartUnitOverview(AModuleName,AModuleLabel : String);override;
    procedure WriteUnitEntry(UnitRef : TPasType); override;
    Procedure EndUnitOverview; override;
    function  GetLabel(AElement: TPasElement): String; override;
    procedure StartListing(Frames: Boolean; const name: String); override;
    procedure EndListing; override;
    procedure WriteCommentLine; override;
    procedure WriteComment(Comment : String);override;
    procedure StartSection(SectionName : String);override;
    procedure StartSubSection(SubSectionName : String);override;
    procedure StartSubSubSection(SubSubSectionName : String);override;
    procedure StartChapter(ChapterName : String); override;
    procedure StartOverview(Const What : String;WithAccess : Boolean); override;
    procedure EndOverview; override;
    procedure WriteOverviewMember(const ALabel,AName,Access,ADescr : String); override;
    procedure WriteOverviewMember(const ALabel,AName,ADescr : String); override;
    // Description node conversion
    procedure DescrBeginBold; override;
    procedure DescrEndBold; override;
    procedure DescrBeginItalic; override;
    procedure DescrEndItalic; override;
    procedure DescrBeginEmph; override;
    procedure DescrEndEmph; override;
    procedure DescrBeginUnderline; override;
    procedure DescrEndUnderline; override;
    procedure DescrWriteFileEl(const AText: DOMString); override;
    procedure DescrWriteKeywordEl(const AText: DOMString); override;
    procedure DescrWriteVarEl(const AText: DOMString); override;
    procedure DescrBeginLink(const AId: DOMString); override;
    procedure DescrEndLink; override;
    procedure DescrWriteLinebreak; override;
    procedure DescrBeginParagraph; override;
    procedure DescrBeginCode(HasBorder: Boolean; const AHighlighterName: String); override;
    procedure DescrWriteCodeLine(const ALine: String); override;
    procedure DescrEndCode; override;
    procedure DescrEndParagraph; override;
    procedure DescrBeginOrderedList; override;
    procedure DescrEndOrderedList; override;
    procedure DescrBeginUnorderedList; override;
    procedure DescrEndUnorderedList; override;
    procedure DescrBeginDefinitionList; override;
    procedure DescrEndDefinitionList; override;
    procedure DescrBeginListItem; override;
    procedure DescrEndListItem; override;
    procedure DescrBeginDefinitionTerm; override;
    procedure DescrEndDefinitionTerm; override;
    procedure DescrBeginDefinitionEntry; override;
    procedure DescrEndDefinitionEntry; override;
    procedure DescrBeginSectionTitle; override;
    procedure DescrBeginSectionBody; override;
    procedure DescrEndSection; override;
    procedure DescrBeginRemark; override;
    procedure DescrEndRemark; override;
    procedure DescrBeginTable(ColCount: Integer; HasBorder: Boolean); override;
    procedure DescrEndTable; override;
    procedure DescrBeginTableCaption; override;
    procedure DescrEndTableCaption; override;
    procedure DescrBeginTableHeadRow; override;
    procedure DescrEndTableHeadRow; override;
    procedure DescrBeginTableRow; override;
    procedure DescrEndTableRow; override;
    procedure DescrBeginTableCell; override;
    procedure DescrEndTableCell; override;
  Public
    Constructor Create(APackage: TPasPackage; AEngine: TFPDocEngine); override;
    Class Function FileNameExtension : String; override;
    Class Procedure Usage(List : TStrings) ; override;
    Function InterpretOption(Const Cmd,Arg : String) : Boolean; override;
  end;


procedure TTXTWriter.WriteUnderline(const Msg: String; DoubleLine: Boolean);

Var
  L : Integer;

begin
  L:=Length(Msg);
  Writeln(Msg);
  WriteLine(L,DoubleLine);
end;

procedure TTXTWriter.WriteLine(DoubleLine: Boolean);

begin
  Writeline(LineWidth,DoubleLine);
end;

Function FindSpace(Const S : String; P : Integer) : Integer;

Var
  I,L : Integer;
  SP: set of char;
begin
  Result:=0;
  SP := [#10,#13,' ',#9];
  I:=P;
  L:=Length(S);
  While (I>0) and (I<=L) and not (S[i] in SP) do
    Dec(I);
  If (I=0) then
    begin
    Inc(I);
    While (I<=L) and not (S[I] in SP) do
      Inc(I);
    end;
  Result:=I;
end;


procedure TTXTWriter.Write(const s: String);

Var
  N : String;
  L : Integer;

begin
  If Length(S)=0 then
    exit;
  N:=S;
  Repeat
    If ((FCurrentPos+Length(N)+1)>LineWidth) then
      begin
      L:=FindSpace(N,LineWidth-FCurrentPos+1);
      inherited Write(Copy(N,1,L-1));
      inherited Write(LineEnding);
      FCurrentPos:=0;
      end
    else
      begin
      L:=Length(N);
      inherited Write(Copy(N,1,L));
      Inc(FCurrentPos,L);
      If FCheckEOL then
        If (L>=LEOL) then
          If (Copy(N,L-LEOL,LEOL)=LineEnding) then
            FCurrentPos:=0;
      end;
    Delete(N,1,L);
  Until (Length(N)=0);
end;

procedure TTXTWriter.WriteLn(const s: String);
begin
  FCheckEOL:=False;
  Try
    inherited WriteLn(s);
    FCurrentPos:=0;
  Finally
    FCheckEOL:=False;
  end;
end;

procedure TTXTWriter.NewLine;

begin
  If Not FCurrentPos=0 then
    Writeln('');
end;

procedure TTXTWriter.WriteLine(LineLength: Integer; DoubleLine: Boolean);

begin
  NewLine;
  If DoubleLine then
    Writeln(StringOfChar('=',LineLength))
  else
    Writeln(StringOfChar('-',LineLength));
end;


function TTXTWriter.GetLabel(AElement: TPasElement): String;

begin
  if AElement.ClassType = TPasUnresolvedTypeRef then
    Result := Engine.ResolveLink(Module, AElement.Name)
  else
  begin
    Result := AElement.PathName;
    Result := LowerCase(Copy(Result, 2, Length(Result) - 1));
  end;
end;

procedure TTXTWriter.DescrBeginBold;
begin
end;

procedure TTXTWriter.DescrEndBold;
begin
end;

procedure TTXTWriter.DescrBeginItalic;
begin
end;

procedure TTXTWriter.DescrEndItalic;
begin
end;

procedure TTXTWriter.DescrBeginEmph;
begin
end;

procedure TTXTWriter.DescrEndEmph;
begin
end;

procedure TTXTWriter.DescrBeginUnderline;
begin
end;

procedure TTXTWriter.DescrEndUnderline;
begin
end;

procedure TTXTWriter.DescrWriteFileEl(const AText: DOMString);
begin
  DescrWriteText(AText);
end;

procedure TTXTWriter.DescrWriteKeywordEl(const AText: DOMString);
begin
  DescrWriteText(AText);
end;

procedure TTXTWriter.DescrWriteVarEl(const AText: DOMString);
begin
  DescrWriteText(AText);
end;

procedure TTXTWriter.DescrBeginLink(const AId: DOMString);
begin
  Write('[');
end;

procedure TTXTWriter.DescrEndLink;
begin
  Write('] ');
end;

procedure TTXTWriter.DescrWriteLinebreak;
begin
  WriteLn('');
end;

procedure TTXTWriter.DescrBeginParagraph;
begin
  // Do nothing
end;

procedure TTXTWriter.DescrEndParagraph;
begin
  WriteLn('');
end;

procedure TTXTWriter.DescrBeginCode(HasBorder: Boolean;
  const AHighlighterName: String);
begin
  StartListing(HasBorder,'');
end;

procedure TTXTWriter.DescrWriteCodeLine(const ALine: String);
begin
  WriteLn(ALine);
end;

procedure TTXTWriter.DescrEndCode;
begin
  EndListing
end;

procedure TTXTWriter.NewListLevel(Initial: Integer);

begin
  Inc(FListLevel);
  If (FListLevel<MaxListLevel) then
    FLists[FListLevel]:=0;
end;

procedure TTXTWriter.declistlevel;

begin
  If (FListLevel>0) then
    Dec(FListLevel)
end;

procedure TTXTWriter.DescrBeginOrderedList;
begin
  NewListLevel(0);
end;

procedure TTXTWriter.DescrEndOrderedList;
begin
  DecListLevel;
end;

procedure TTXTWriter.DescrBeginUnorderedList;
begin
  NewListLevel(-1);
end;

procedure TTXTWriter.DescrEndUnorderedList;
begin
  DecListLevel;
end;

procedure TTXTWriter.DescrBeginDefinitionList;
begin
  NewListLevel(-2);
end;

procedure TTXTWriter.DescrEndDefinitionList;
begin
  DecListLevel;
end;

procedure TTXTWriter.DescrBeginListItem;
begin
  If FLists[FListLevel]>=0 then
    begin
    Inc(FLists[FListLevel]);
    WriteF('%d. ',[FLists[FListLevel]]);
    end;
  Write('   ');
end;

procedure TTXTWriter.DescrEndListItem;
begin
  WriteLn('');
end;

procedure TTXTWriter.DescrBeginDefinitionTerm;
begin
  Write('<<');
end;

procedure TTXTWriter.DescrEndDefinitionTerm;
begin
  WriteLn('>>:');
end;

procedure TTXTWriter.DescrBeginDefinitionEntry;
begin
  // Do nothing
end;

procedure TTXTWriter.DescrEndDefinitionEntry;
begin
  WriteLn('');
end;

procedure TTXTWriter.DescrBeginSectionTitle;
begin
  Inc(FSectionCount);
  WritelnF('%s %d.%d: ',[SDocSection,FChapterCount,FSectionCount]);
end;

procedure TTXTWriter.DescrBeginSectionBody;
begin
  WriteLn('');
end;

procedure TTXTWriter.DescrEndSection;
begin
  // Do noting
end;

procedure TTXTWriter.DescrBeginRemark;
begin
  WriteLn(SDocRemark+': ');
end;

procedure TTXTWriter.DescrEndRemark;
begin
  WriteLn('');
end;

procedure TTXTWriter.DescrBeginTable(ColCount: Integer; HasBorder: Boolean);

begin
  WriteLine(False);
end;

procedure TTXTWriter.DescrEndTable;
begin
  WriteLine(False);
end;

procedure TTXTWriter.DescrBeginTableCaption;
begin
  // Do nothing.
end;

procedure TTXTWriter.DescrEndTableCaption;
begin
  Inc(FTableCount);
  WriteF('%s %d :',[SDoctable,FTableCount]);
  TableCaptionWritten := True;
end;

procedure TTXTWriter.DescrBeginTableHeadRow;
begin
  if not TableCaptionWritten then
    DescrEndTableCaption;
  TableRowStartFlag := True;
end;

procedure TTXTWriter.DescrEndTableHeadRow;
begin
  WriteLine(False);
end;

procedure TTXTWriter.DescrBeginTableRow;
begin
  if not TableCaptionWritten then
    DescrEndTableCaption;
  TableRowStartFlag := True;
end;

procedure TTXTWriter.DescrEndTableRow;
begin
  WriteLn('');
end;

procedure TTXTWriter.DescrBeginTableCell;
begin
  if TableRowStartFlag then
    TableRowStartFlag := False
  else
    Write('    ');
end;

procedure TTXTWriter.DescrEndTableCell;
begin
  // Do nothing
end;

constructor TTXTWriter.Create(APackage: TPasPackage; AEngine: TFPDocEngine);
begin
  inherited Create(APackage, AEngine);
  LineWidth:=DefaultLineWidth;
end;

class procedure TTXTWriter.Usage(List: TStrings);
begin
  inherited Usage(List);
end;

function TTXTWriter.InterpretOption(const Cmd, Arg: String): Boolean;
begin
  if cmd='--linewidth' then
    begin
    LineWidth:=StrToIntDef(Arg,DefaultLineWidth);
    Result:=True;
    end
  else
    Result:=inherited InterpretOption(Cmd, Arg);
end;

procedure TTXTWriter.WriteLabel(const S: String);
begin
end;

procedure TTXTWriter.WriteIndex(const S: String);
begin
end;

procedure TTXTWriter.StartListing(Frames: Boolean; const name: String);
begin
  FInVerbatim:=True;
  If (Name<>'') then
    WritelnF('%s : %s',[SDocListing,Name]);
  If Frames then
    WriteLine(False)
  else
    WriteLn('');
end;

procedure TTXTWriter.EndListing;
begin
  FInVerbatim:=False;
end;

procedure TTXTWriter.WriteCommentLine;

begin
end;

procedure TTXTWriter.WriteComment(Comment: String);
begin
end;

procedure TTXTWriter.StartChapter(ChapterName: String);
begin
  Inc(FChapterCount);
  FSectionCount:=0;
  FSubSectionCount:=0;
  Writeln('');
  WriteLine(True);
  WritelnF('%s %d : %s',[SDocChapter,FChapterCount,ChapterName]);
  WriteLine(True);
  Writeln('');
end;

procedure TTXTWriter.StartSection(SectionName: String);
begin
  Inc(FSectionCount);
  FSubSectionCount:=0;
  Writeln('');
  WriteLine(False);
  WritelnF('%s %d.%d : %s',[SDocSection,FChapterCount,FSectionCount,SectionName]);
  WriteLine(False);
  Writeln('');
end;

procedure TTXTWriter.StartSubSection(SubSectionName: String);
begin
  Inc(FSubSectionCount);
  Writeln('');
  WritelnF('%d.%d.%d : %s',[FChapterCount,FSectionCount,FSubSectionCount,SubSectionName]);
  WriteLine(False);
  Writeln('');
end;

procedure TTXTWriter.StartSubSubSection(SubSubSectionName: String);
begin
  Writeln('');
  Writeln(SubSubSectionName);
  Writeln('');
end;

procedure CreateTxtDocForPackage(APackage: TPasPackage; AEngine: TFPDocEngine);
var
  Writer: TTxtWriter;
begin
  Writer := TTxtWriter.Create(APackage, AEngine);
  try
    Writer.DoWriteDocumentation;
  finally
    Writer.Free;
  end;
end;

procedure TTXTWriter.WriteExampleFile(FN: String);

Var
  L : TStringList;
  I : Integer;

begin
  Write(SDocExample);
  Writeln(' '+ExtractFileName(FN));
  If (FN<>'') and FileExists(FN) then
    begin
    WriteLine(False);
    L:=TStringList.Create;
    Try
      L.LoadFromFile(FN);
      For I:=0 to L.Count-1 do
        Writeln(L[i]);
    finally
      L.Free;
    end;
    WriteLine(False);
    end;
end;

procedure TTXTWriter.StartOverview(const What: String; WithAccess: Boolean);

begin
  If WithAccess then
    WriteUnderLine(Format('%.30s %.10s %s',[EscapeText(What), EscapeText(SDocAccess), EscapeText(SDocDescription)]),False)
  else
    WriteUnderLine(Format('%.30s %s',[EscapeText(What), EscapeText(SDocDescription)]),False);
end;

procedure TTXTWriter.EndOverview;

begin
  WriteLine(False);
end;

procedure TTXTWriter.WriteOverviewMember(const ALabel, AName, Access,
  ADescr: String);

begin
  WriteLnF('%.30s %.10s  %s',[AName,Access,ADescr]);
end;

procedure TTXTWriter.WriteOverviewMember(const ALabel, AName, ADescr: String);

begin
  WriteLnF('%.30s %s ',[AName,ADescr]);
end;

class function TTXTWriter.FileNameExtension: String;
begin
  Result:=TxtExtension;
end;

procedure TTXTWriter.StartUnitOverview(AModuleName, AModuleLabel: String);

begin
  WriteUnderLine('Unit Name',False);
end;

procedure TTXTWriter.WriteUnitEntry(UnitRef: TPasType);

begin
  Writeln(EscapeText(UnitRef.Name));
end;

procedure TTXTWriter.EndUnitOverview;

begin
  Writeln('');
end;


initialization
  // Do not localize.
  RegisterWriter(TTXTWriter,'txt','Plain text.');
finalization
  UnRegisterWriter('txt');
end.
