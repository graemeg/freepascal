{

    FPDoc  -  Free Pascal Documentation Tool
    Copyright (C) 2007 by the FPC team.

    * RTF output generator

    See the file COPYING, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
{$mode objfpc}
{$H+}
unit dw_LinRTF;
{$WARN 5024 off : Parameter "$1" not used}
interface

uses DOM, dGlobals, PasTree;

const
  RTFHighLight : Boolean = False;
  RTFExtension   : String = '.rtf';

Procedure CreateRTFDocForPackage(APackage: TPasPackage; AEngine: TFPDocEngine);

implementation

uses fpdocstrs, SysUtils, Classes, dwLinear, dwriter;

const
  Indent = 300;

Type

  TEnvironmenttype = (etEnumerate, etItemize, etDescription);

  TEnvironment = class
  private
    fenvtype : TEnvironmenttype;
    fdeltaIndent,
    fdeltaMargin,
    fnextitem : integer;
    fprevious : TEnvironment;
  public
    property envtype : TEnvironmenttype read fenvtype;
    property deltaIndent : integer read fdeltaIndent;
    property deltaMargin : integer read fdeltaMargin;
    property nextitem : integer read fnextitem;
    property previous : TEnvironment read fprevious;
  end;
  

type

 { TRTFWriter }

  TRTFWriter = class(TLinearWriter)
  protected
    // used
    FLink: String;
    FEnvironmentStack : TEnvironment;
    FLeftMargin, FParIndent : integer;
    FEnumDepth : byte;
    FBorderString : string;
    FEmphLevel : integer;
    Cchapters, Csections, Csubsections, Csubsubsections : integer;
    // not yet used
    FTableCount : Integer;
    FInVerbatim : Boolean;
    Inlist,
    TableRowStartFlag,
    TableCaptionWritten: Boolean;
    // helper procedures
    procedure PushEnvironment (atype:TEnvironmenttype; dmargin,dindent:integer);
    procedure PopEnvironment;
    function  GetEnumNumber(depth, item : integer) : string;
    procedure Header(text:string; font:integer);
    // Linear documentation methods overrides;
    procedure WriteBeginDocument; override;
    procedure WriteEndDocument; override;
    procedure WriteLabel(Const S : String); override;
    procedure WriteIndex(Const S : String); override;
    Procedure WriteExampleFile(FN : String); override;
    Procedure StartProcedure; override;
    Procedure EndProcedure; override;
    Procedure StartProperty; override;
    Procedure EndProperty; override;
    Procedure StartSynopsis; override;
    Procedure StartDeclaration; override;
    Procedure StartVisibility; override;
    Procedure StartDescription; override;
    Procedure StartAccess; override;
    Procedure StartErrors; override;
    Procedure StartSeealso; override;
    Procedure EndSeealso; override;
    procedure StartUnitOverview(AModuleName,AModuleLabel : String);override;
    procedure WriteUnitEntry(UnitRef : TPasType); override;
    Procedure EndUnitOverview; override;
    function  GetLabel(AElement: TPasElement): String; override;
    procedure StartListing(Frames: Boolean; const name: String); override;
    procedure EndListing; override;
    Function  EscapeText(S : String) : String; override;
    Function  StripText(S : String) : String; override;
    procedure WriteCommentLine; override;
    procedure WriteComment(Comment : String);override;
    procedure StartSection(SectionName : String);override;
    procedure StartSubSection(SubSectionName : String);override;
    procedure StartSubSubSection(SubSubSectionName : String);override;
    procedure StartChapter(ChapterName : String); override;
    procedure StartOverview(Const What : String; WithAccess : Boolean); override;
    procedure WriteOverviewMember(const ALabel,AName,Access,ADescr : String); override;
    procedure WriteOverviewMember(const ALabel,AName,ADescr : String); override;
    procedure EndOverview; override;
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
    // TFPDocWriter class methods
  public
    Function InterPretOption(Const Cmd,Arg : String) : boolean; override;
    Class Function FileNameExtension : String; override;
  end;




function TRTFWriter.GetLabel(AElement: TPasElement): String;
var
  i: Integer;
begin
  if AElement.ClassType = TPasUnresolvedTypeRef then
    Result := Engine.ResolveLink(Module, AElement.Name)
  else
  begin
    Result := AElement.PathName;
    Result := LowerCase(Copy(Result, 2, Length(Result) - 1));
  end;
  for i := 1 to Length(Result) do
    if Result[i] = '.' then
      Result[i] := ':';
end;


function TRTFWriter.EscapeText(S: String): String;

var
  i: Integer;

begin
  if FInVerBatim=True then
    Result:=S
  else
    begin
    SetLength(Result, 0);
    for i := 1 to Length(S) do
      case S[i] of
        '\','{','}':            // Escape these characters
          Result := Result + '\' + S[i];
        else
          Result := Result + S[i];
      end;
    end;
end;

function TRTFWriter.StripText(S: String): String;

var
  I: Integer;

begin
  Result:='';
  for i := 1 to Length(S) do
    If not (S[i] in ['{','}','\']) then
      Result := Result + S[i];
end;

procedure TRTFWriter.PushEnvironment(atype: TEnvironmenttype; dmargin,dindent:integer);
var e : TEnvironment;
begin
  e := TEnvironment.Create;
  with e do
    begin
    fenvtype := atype;
    fnextitem := 1;
    fdeltaIndent := dIndent;
    fdeltaMargin := dMargin;
    fprevious := FEnvironmentStack;
    end;
  if atype = etEnumerate then
    inc (FEnumDepth);
  inc (FParIndent, dindent);
  inc (FLeftMargin, dmargin);
  FEnvironmentStack := e;
end;

procedure TRTFWriter.PopEnvironment;
var e : TEnvironment;
begin
  e := FEnvironmentStack.previous;
  with FEnvironmentStack do
    begin
    if envtype = etEnumerate then
      dec (FEnumDepth);
    dec (FParIndent, deltaIndent);
    dec (FLeftMargin, deltaMargin);
    end;
  FEnvironmentStack.Free;
  FEnvironmentStack := e;
end;

function Romanized (nr : integer) : string;
const
  st : array[0..9] of string = ('','X','XX','XXX','XL','L','LX','LXX','LXXX','XC');
  se : array[0..9] of string = ('','I','II','III','IV','V','VI','VII','VIII','IX');
var t,e : integer;
begin
  t := (nr mod 100) div 10;
  e := nr mod 10;
  result := st[t] + se[e];
end;

function TRTFWriter.GetEnumNumber(depth, item: integer) : string;
begin
  case depth of
    1 : result := inttostr(item) + '.';
    2 : result := '(' + char (ord('a')+item-1) + ')';
    3 : result := Romanized(item);
    4 : result := '(' + char (ord('A')+item-1) + ')';
    else result := '(' + inttostr(item) + ')';
  end;
end;

procedure TRTFWriter.WriteBeginDocument;
begin
  write('{\rtf1');
  write('{\fonttbl');
    Write('{\f0\fswiss Helvetica{\*\falt Arial};}');
    write('{\f1\fmodern Courier{\*\falt Courier New};}');
    write('{\f2\froman Times{\*\falt Times New Roman};}');
  write('}{\stylesheet');
    write('{\s1\li0\fi0\ql\sb240\sa60\keepn\f2\b\fs32 Section Title;}');
    write('{\s2\ql\sb30\sa30\keepn\b0\i0\scaps1\f1\fs28 Table Title;}');
    write('{\s3\li0\fi0\qc\sb240\sa60\keepn\f2\b\scaps1\fs28 Listing Title;}');
    write('{\s4\li30\fi30\ql\f2\fs24 Listing Contents;}');
    write('{\s5\li0\fi0\ql\sb240\sa60\keepn\f2\b\fs40 Chapter;}');
    write('{\s6\li0\fi0\ql\sb240\sa60\keepn\f2\b\fs32 Section;}');
    write('{\s7\li0\fi0\ql\sb240\sa60\keepn\f2\b\fs28 Subsection;}');
    write('{\s8\li0\fi0\ql\sb240\sa60\keepn\f2\b\fs24 Subsubsection;}');
    write('{\s9\li30\fi10\ql\sb60\keepn\f2\fs24 Description titles;}');
    write('{\s10\li30\fi30\ql\fs24 Description;}');
    write('{\s11\li0\fi0\ql\fs24 Source Example;}');
  write ('}');
  FLeftMargin := 0;
  FParIndent := 0;
  FEnvironmentStack := nil;
  FEnumDepth := 0;
  Cchapters := 0;
  Csections := 0;
  Csubsections := 0;
  Csubsubsections := 0;
end;

procedure TRTFWriter.WriteEndDocument;
begin
  write('}');
end;

procedure TRTFWriter.DescrBeginBold;
begin
  Write('{\b ');
end;

procedure TRTFWriter.DescrEndBold;
begin
  Write('}');
end;

procedure TRTFWriter.DescrBeginItalic;
begin
  Write('{\i ');
end;

procedure TRTFWriter.DescrEndItalic;
begin
  Write('}');
end;

procedure TRTFWriter.DescrBeginEmph;
begin
  inc (FEmphLevel);
  if (FEmphLevel and 1) = 1 then
    Write('{\i ')
  else
    Write('{\i0 ');
end;

procedure TRTFWriter.DescrEndEmph;
begin
  dec (FEmphLevel);
  Write('}')
end;

procedure TRTFWriter.DescrBeginUnderline;
begin
  Write('{\ul ');
end;

procedure TRTFWriter.DescrEndUnderline;
begin
  Write('}');
end;

procedure TRTFWriter.DescrWriteFileEl(const AText: DOMString);
begin
  Write('{\f0 ');
  DescrWriteText(AText);
  Write('}');
end;

procedure TRTFWriter.DescrWriteKeywordEl(const AText: DOMString);
begin
  Write('{\b\f1 ');
  DescrWriteText(AText);
  Write('}');
end;

procedure TRTFWriter.DescrWriteVarEl(const AText: DOMString);
begin
  Write('{\f1 ');
  DescrWriteText(AText);
  Write('}');
end;

procedure TRTFWriter.DescrBeginLink(const AId: DOMString);
begin
  FLink := Engine.ResolveLink(Module, Utf8Encode(AId));
//  System.WriteLn('Link "', AId, '" => ', FLink);
end;

procedure TRTFWriter.DescrEndLink;
var s : string;
begin
  s := StripText(Flink);
  WriteF('{\field{\*\fldinst{\lang1024 PAGEREF BM%s \\*MERGEFORMAT }}',[s]);
  WriteF('{\\fldrslt{%s}}}',[s]);
end;

procedure TRTFWriter.DescrWriteLinebreak;
begin
  WriteLn('\line');
end;

procedure TRTFWriter.DescrBeginParagraph;
begin
  // Do nothing
end;

procedure TRTFWriter.DescrEndParagraph;
begin
  Write('\par ');
end;

procedure TRTFWriter.DescrBeginCode(HasBorder: Boolean;
  const AHighlighterName: String);
begin
  StartListing(HasBorder,'');
end;

procedure TRTFWriter.DescrWriteCodeLine(const ALine: String);
begin
  Write(ALine+'\line ');
end;

procedure TRTFWriter.DescrEndCode;
begin
  EndListing;
end;

procedure TRTFWriter.DescrBeginOrderedList;
begin
  PushEnvironment(etEnumerate, 2*Indent, -Indent);
end;

procedure TRTFWriter.DescrEndOrderedList;
begin
  PopEnvironment;
end;

procedure TRTFWriter.DescrBeginUnorderedList;
begin
  PushEnvironment(etItemize, 2*Indent, -Indent);
end;

procedure TRTFWriter.DescrEndUnorderedList;
begin
  PopEnvironment;
end;

procedure TRTFWriter.DescrBeginDefinitionList;
begin
  PushEnvironment(etDescription, Indent, -Indent);
end;

procedure TRTFWriter.DescrEndDefinitionList;
begin
  PopEnvironment;
end;

procedure TRTFWriter.DescrBeginListItem;
begin
  WriteF('{\pard\li%d\fi%d ',[FLeftMargin,FParIndent]);
  with FEnvironmentStack do
    if envtype = etItemize then
      write('\bullet\tab ')
    else
      begin
      WriteF('%s\tab ', [GetEnumNumber(fenumdepth,fnextitem)]);
      inc (fnextitem);
      end;
end;

procedure TRTFWriter.DescrEndListItem;
begin
  WriteLn('}');
end;

procedure TRTFWriter.DescrBeginDefinitionTerm;
begin
  WriteF('{\pard\li%d\fi%d{\b  ',[FLeftMargin,FParIndent]);
end;

procedure TRTFWriter.DescrEndDefinitionTerm;
begin
  Write('}');
end;

procedure TRTFWriter.DescrBeginDefinitionEntry;
begin
  Write('\tab ');
end;

procedure TRTFWriter.DescrEndDefinitionEntry;
begin
  WriteLn('}');
end;

procedure TRTFWriter.DescrBeginSectionTitle;
begin
  write('{\pard\s1 ');
end;

procedure TRTFWriter.DescrBeginSectionBody;
begin
  WriteLn('\par}');
end;

procedure TRTFWriter.DescrEndSection;
begin
  write('\par ');
end;

procedure TRTFWriter.DescrBeginRemark;
begin
  write ('\par ');
  write('{\b Remark:}\tab ');
end;

procedure TRTFWriter.DescrEndRemark;
begin
  write ('\par ');
end;

procedure TRTFWriter.DescrBeginTable(ColCount: Integer; HasBorder: Boolean);
begin
  Write('\par{');
  if HasBorder then
    FBorderString := '\trbrdrl\brdrs\brdrw1\trbrdrr\brdrs\brdrw1'
  else
    FBorderString := ''
end;

procedure TRTFWriter.DescrEndTable;
begin
  Write('}');
end;

procedure TRTFWriter.DescrBeginTableCaption;
begin
  Write('\pard\s2 ');
end;

procedure TRTFWriter.DescrEndTableCaption;
begin
  write('\par ');
end;

procedure TRTFWriter.DescrBeginTableHeadRow;
begin
  write('{\b\trowd'+FBorderstring+'\trbrdrh\brdrs\trbrdrv\brdrs ');
end;

procedure TRTFWriter.DescrEndTableHeadRow;
begin
  Write('\row}');
end;

procedure TRTFWriter.DescrBeginTableRow;
begin
  write('\trowd'+FBorderstring+'\trbrdrh\brdrs\trbrdrv\brdrs ');
end;

procedure TRTFWriter.DescrEndTableRow;
begin
  Write('\row ');
end;

procedure TRTFWriter.DescrBeginTableCell;
begin
  write('\pard\intbl ');
end;

procedure TRTFWriter.DescrEndTableCell;
begin
  write('\cell');
end;

procedure TRTFWriter.WriteLabel(const S: String);
var b: string;
begin
  b := LowerCase(StripText(s));
  WriteF('{\bkmkstart %s}{\bkmkend %s}', [b,b]);
end;

procedure TRTFWriter.WriteIndex(const S: String);
begin
  Write('{\xe{\v '+EscapeText(s)+'}}');
end;

procedure TRTFWriter.StartListing(Frames: Boolean; const name: String);
begin
  Write('\par');
  if name <> '' then
    Write('{\pard\s3 '+name+'\par}');
  Write('{\pard\s4 ');
end;

procedure TRTFWriter.EndListing;
begin
  Writeln('}')
end;

procedure TRTFWriter.WriteCommentLine;
begin
  // doesn't exist
end;

procedure TRTFWriter.WriteComment(Comment : String);
begin
  // doesn't exist
end;

procedure TRTFWriter.StartChapter(ChapterName : String);
begin
  inc (Cchapters);
  if Cchapters > 1 then
    Write('\par\page');
  WriteF('{\pard\s5 %d   %s\par}', [Cchapters,EscapeText(ChapterName)]);
  Csubsubsections := 0;
  Csubsections := 0;
  Csections := 0;
end;

procedure TRTFWriter.StartSection(SectionName : String);
begin
  inc (Csections);
  if Csections > 1 then
    Write('\par');
  WriteF('{\pard\s6 %d.%d   %s\par}', [Cchapters,Csections,EscapeText(SectionName)]);
  Csubsubsections := 0;
  Csubsections := 0;
end;

procedure TRTFWriter.StartSubSection(SubSectionName : String);
begin
  inc (Csubsections);
  if Csubsections > 1 then
    Write('\par');
  WriteF('{\pard\s7 %d.%d.%d   %s\par}', [Cchapters,Csections,Csubsections,EscapeText(SubSectionName)]);
  Csubsubsections := 0;
end;

procedure TRTFWriter.StartSubSubSection(SubSubSectionName : String);
begin
  inc (Csubsubsections);
  if Csubsubsections > 1 then
    Write('\par');
  WriteF('{\pard\s8 %d.%d.%d.%d   %s\par}', [Cchapters,Csections,Csubsections,Csubsubsections,
          EscapeText(SubSubSectionName)]);
end;

procedure TRTFWriter.StartProcedure;
begin
  Write('{\pard');
end;

procedure TRTFWriter.StartProperty;
begin
  Write('{\pard');
end;

procedure TRTFWriter.Header(text: string; font: integer);
begin
  WriteF('\par\s9 %s\pard\par\s10\f%d ',[text, font]);
end;

procedure TRTFWriter.StartSynopsis;
begin
  Header(SDocSynopsis,2);
end;

procedure TRTFWriter.StartDeclaration;
begin
  Header(SDocDeclaration,1);
end;

procedure TRTFWriter.StartVisibility;
begin
  Header(SDocVisibility,2);
end;

procedure TRTFWriter.StartDescription;
begin
  Header(SDocDescription,2);
end;

procedure TRTFWriter.StartErrors;
begin
  Header(SDocErrors,2);
end;

procedure TRTFWriter.StartAccess;
begin
  Header(SDocAccess,2)
end;

procedure TRTFWriter.EndProcedure;
begin
  Write('}');
end;

procedure TRTFWriter.EndProperty;
begin
  Write('}');
end;

procedure TRTFWriter.WriteExampleFile(FN : String);
var s : TStringlist;
begin
  If (FN<>'') then
    begin
    Write('\pard{\s4 Listing:} '+FN);
    Write('\pard{\f1 ');
    s := TStringlist.Create;
    try
      s.loadfromfile (FN);
      Write(s.Text);
    finally
      s.Free;
    end;
    Write('}');
    end;
end;

procedure TRTFWriter.StartOverview(const What: String; WithAccess: Boolean);
begin
  If WithAccess then
    WriteF('\par\trowd\pard\intbl %s\cell\pard\intbl %s\cell\pard\intbl %s \cell\pard\intbl %s \cell\row',
        [EscapeText(SDocPage), EscapeText(What), EscapeText(SDocAccess), EscapeText(SDocDescription)])
  else
    WriteF('\par\trowd\pard\intbl %s\cell\pard\intbl %s\cell\pard\intbl %s\cell\row',
        [EscapeText(SDocPage), EscapeText(What), EscapeText(SDocDescription)]);
end;

procedure TRTFWriter.WriteOverviewMember(const ALabel,AName,Access,ADescr : String);
begin
  //TODO: Translate Latex \pageref to RTF
  //WriteLnF('\pageref{%s} & %s  & %s & %s \\',[ALabel,AName,Access,ADescr]);
  WriteF('\par\trowd\pard\intbl %s\cell\pard\intbl %s\cell\pard\intbl %s \cell\pard\intbl %s \cell\row',
      [ALabel,AName,Access,ADescr]);
end;

procedure TRTFWriter.WriteOverviewMember(const ALabel,AName,ADescr : String);
begin
  //TODO: Translate Latex \pageref to RTF
  //WriteLnF('\pageref{%s} & %s  & %s \\',[ALabel,AName,ADescr]);
  WriteF('\par\trowd\pard\intbl %s\cell\pard\intbl %s\cell\pard\intbl %s\cell\row',
      [ALabel,AName,ADescr]);
end;

procedure TRTFWriter.EndOverview;
begin
  Write ('\par');
end;

procedure TRTFWriter.StartSeealso;
begin
  Header(SDocSeeAlso, 2);
end;

procedure TRTFWriter.EndSeealso;
begin
end;

procedure TRTFWriter.StartUnitOverview(AModuleName,AModuleLabel : String);
begin
  WriteF ('\pard\qc\s3 %s', [Format(SDocUsedUnitsByUnitXY, [AModuleName])]);
  Write ('\par\trowd\pard\intbl Name\cell\pard\intbl Page\cell\row');
end;

procedure TRTFWriter.WriteUnitEntry(UnitRef : TPasType);
begin
  WriteF('\par\trowd\pard\intbl %s\cell\pard\intbl %s\cell\row',
      [UnitRef.Name, 'Pageref to '+StripText(GetLabel(UnitRef))]);
  //WriteLnF('%s\index{unit!%s} & \pageref{%s} \\',
  //   [UnitRef.Name, UnitRef.Name, StripText(GetLabel(UnitRef))]);
end;

procedure TRTFWriter.EndUnitOverview;
begin
  Write('\par');
end;

procedure CreateRTFDocForPackage(APackage: TPasPackage; AEngine: TFPDocEngine);
var
  Writer: TRTFWriter;
begin
  Writer := TRTFWriter.Create(APackage, AEngine);
  try
    Writer.DoWriteDocumentation;
  finally
    Writer.Free;
  end;
end;

function TRTFWriter.InterPretOption(const Cmd, Arg: String): boolean;
begin
  if Cmd = '--RTF-extension' then
    begin
    RTFExtension:=Arg;
    Result := true;
    end
  else
    Result:=False;
end;

class function TRTFWriter.FileNameExtension: String;
begin
  Result:=RTFExtension;
end;

initialization
  // Do not localize.
  RegisterWriter(TRTFWriter,'rtf','RTF output.');

finalization
  UnRegisterWriter('rtf');
end.
