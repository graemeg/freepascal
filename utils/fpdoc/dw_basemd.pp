{
    FPDoc  -  Free Pascal Documentation Tool
    Copyright (C) 2021 by Michael Van Canneyt

    * Basic Markdown output generator. No assumptions about document/documentation structure

    See the file COPYING, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
unit dw_basemd;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, dwriter, DOM, pastree, dglobals;

Const
  MaxIndents = 32;
  MaxLists = 32;

Type
  THeaderLevel = 1..6;
  TRender = (rStrong,rEmphasis,rCode);
  TListType = (ltOrdered,ltUnordered, ltDefinition);

  TMarkdownEngine = (meMkDocs,meNone);

  { TBaseMarkdownWriter }

  TBaseMarkdownWriter = class(TMultifileDocWriter)
  private
    FIgnoreCount : Integer;
    FBaseImageURL: String;
    FContentPrefix: String;
    FCurrentIndentIndex: Word;
    FCurrentLine: UTF8String;
    FDefinitionSeparator: String;
    FDefinitionTermRender: TRender;
    FFileRendering: TRender;
    FIndentSize: Byte;
    FKeywordRendering: TRender;
    FPrefix : string;
    FMetadata,
    FMarkDown: TStrings;
    FSymbolRendering: TRender;
    FTheme: String;
    FUnderLineRendering: TRender;
    FVarRendering: TRender;
    FLink : String;
    FListStack : Integer;
    FIndents : Array[0..MaxIndents] of Word;
    FListTypes : Array[1..MaxLists] of TListType;
    FTableColCount : Integer;
    FMarkDownEngine: TMarkdownEngine;
    function GetCurrentIndent: Word;
    procedure SetIndentSize(AValue: Byte);
    procedure clearIndent;
    procedure CalcPrefix;
  Protected
    function CreateAllocator: TFileAllocator; override;
    Procedure DescrEmitNotesHeader(AContext : TPasElement); override;
    Procedure DescrEmitNotesFooter(AContext : TPasElement); override;
    procedure DescrWriteText(const AText: DOMString); override;
    procedure DescrBeginBold; override;
    procedure DescrEndBold; override;
    procedure DescrBeginItalic; override;
    procedure DescrEndItalic; override;
    procedure DescrBeginEmph; override;
    procedure DescrEndEmph; override;
    procedure DescrBeginUnderline; override;
    procedure DescrEndUnderline; override;
    procedure DescrWriteImageEl(const AFileName, ACaption, ALinkName : DOMString); override;
    procedure DescrWriteFileEl(const AText: DOMString); override;
    procedure DescrWriteKeywordEl(const AText: DOMString); override;
    procedure DescrWriteVarEl(const AText: DOMString); override;
    procedure DescrBeginLink(const AId: DOMString); override;
    procedure DescrEndLink; override;
    procedure DescrBeginURL(const AURL: DOMString); override;
    procedure DescrEndURL; override;
    procedure DescrWriteLinebreak; override;
    procedure DescrBeginParagraph; override;
    procedure DescrEndParagraph; override;
    procedure DescrBeginCode(HasBorder: Boolean; const AHighlighterName: String); override;
    procedure DescrWriteCodeLine(const ALine: String); override;
    procedure DescrEndCode; override;
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
  Protected
    // Emit current line, if any
    Function OutputCurrentLine : Boolean;
    function EscapeMarkDown(aText: Domstring): string;
    function EscapeMarkDown(aText: String): string;
    function CreateLink(Const aText,aLink : String) : String;
    function GetListPrefix: String;virtual;

    // Append to current line
    procedure AppendToLine(aText: DomString; DoEscape: boolean = true);
    procedure AppendToLine(aText: UTF8String; DoEscape: boolean = true); virtual;
    procedure AppendToLine(aFmt: UTF8String; aArgs : Array of const; DoEscape: boolean = true);
    // Write current line and append new line
    procedure EmitLine(aText: UTF8String; DoEscape: boolean = true); virtual;
    procedure EmitLine(aFmt: UTF8String; aArgs : Array of const; DoEscape: boolean = true);
    // Append spans to current line
    procedure AppendLink(Const aText,aLink : String);
    Procedure AppendRendered(aText : String; aRender : TRender);
    Procedure AppendKeyWord(aText : String); inline;
    Procedure AppendSymbol(aText : String); inline;
    Procedure AppendTableHeader(aHeaders: Array of String);
    Procedure EmitCode(aCodeBlock : String; aIndent : Integer = 0);
    Procedure EmitCode(aCodeBlock : TStrings; aIndent : Integer = 0);
    Procedure EmitCodeLine(aCodeLine : string);
    procedure EndSpan(aRender: TRender);
    procedure StartSpan(aRender: TRender);
    Procedure PushIndent(aNewIndent : Byte);
    Procedure PopIndent;
    Procedure StartList(aType : TListType);
    Procedure StopList(aType : TListType);
    Procedure BeginIgnore;
    Procedure EndIgnore;
    Procedure DoLineBreak;
    Function InList : Boolean;
    Function CurrentList : TListType;
    Property ContentPrefix : String Read FContentPrefix Write FContentPrefix;
  Public
    Constructor Create(APackage: TPasPackage; AEngine: TFPDocEngine); override;
    destructor Destroy;  override;
    Procedure AppendHeader(aLevel : THeaderLevel; const AHeader : String; DoEscape : Boolean = true);
    Procedure Indent;
    Procedure Undent;
    Procedure ClearMarkDown;
    Procedure EnsureEmptyLine;
    procedure SaveToFile(aFileName : string);
    procedure AddMetaData(Const aName,aValue : string);
    Property CurrentLine : UTF8String Read FCurrentLine Write FCurrentLine;
    Property MarkDown : TStrings Read FMarkDown;
    Property CurrentIndent : Word Read GetCurrentIndent;
    Property Prefix : String Read FPrefix;
    Property IndentSize : Byte Read FIndentSize Write SetIndentSize;
    Property UnderLineRendering : TRender Read FUnderLineRendering Write FUnderLineRendering;
    Property FileRendering : TRender Read FFileRendering Write FFileRendering;
    Property VarRendering : TRender Read FVarRendering Write FVarRendering;
    Property KeywordRendering : TRender Read FKeywordRendering Write FKeyWordRendering;
    Property SymbolRendering : TRender Read FSymbolRendering Write FSymbolRendering;
    Property DefinitionSeparator : String Read FDefinitionSeparator Write FDefinitionSeparator;
    Property DefinitionTermRender : TRender Read FDefinitionTermRender Write FDefinitionTermRender;
    Property BaseImageURL : String Read FBaseImageURL Write FBaseIMageURL;
    Property MetaData : TStrings Read FMetaData;
    Property MarkDownEngine : TMarkdownEngine Read FMarkDownEngine Write FMarkDownEngine;
    Property Theme : String Read FTheme Write FTheme;
  end;



implementation

uses fpdocstrs;


procedure TBaseMarkdownWriter.SetIndentSize(AValue: Byte);
begin
  if FIndentSize=AValue then Exit;
  if CurrentIndent>0 then
    FPDocError(SErrCannotChangeIndentSizeWhenIndented);
  FIndentSize:=AValue;
end;

function TBaseMarkdownWriter.GetCurrentIndent: Word;
begin
  Result:=FIndents[FCurrentIndentIndex];
end;

procedure TBaseMarkdownWriter.clearIndent;
begin
  FIndents[FCurrentIndentIndex]:=0;
  CalcPrefix;
end;

procedure TBaseMarkdownWriter.CalcPrefix;

begin
  FPrefix:=StringOfChar(' ',CurrentIndent);
end;

function TBaseMarkdownWriter.CreateAllocator: TFileAllocator;
begin
  Result:=TLongNameFileAllocator.Create('.md');
end;

procedure TBaseMarkdownWriter.DescrEmitNotesHeader(AContext: TPasElement);
begin
  AppendHeader(2, SDocNotes);
end;

procedure TBaseMarkdownWriter.DescrEmitNotesFooter(AContext: TPasElement);
begin
  EnsureEmptyLine;
end;

function TBaseMarkdownWriter.EscapeMarkDown(aText: Domstring): string;

begin
  Result:=EscapeMarkDown(UTF8Encode(aText))
end;

function TBaseMarkdownWriter.EscapeMarkDown(aText: String): string;
begin
  Result:=StringReplace(aText,'*','\*',[rfReplaceAll]);
  Result:=StringReplace(Result,'_','\_',[rfReplaceAll]);
  Result:=StringReplace(Result,'`','\`',[rfReplaceAll]);
end;

function TBaseMarkdownWriter.CreateLink(const aText, aLink: String): String;
begin
  Result:=Format('[%s](%s)',[EscapeMarkDown(aText),aLink])
end;

procedure TBaseMarkdownWriter.AppendToLine(aText: DomString; DoEscape: boolean);

begin
  If FIgnoreCount>0 then
    exit;
  AppendToLine(UTF8Encode(aText),DoEscape);
end;

procedure TBaseMarkdownWriter.AppendToLine(aText: UTF8String; DoEscape: boolean
  );
begin
  if DoEscape then
     aText:=EscapeMarkDown(aText);
  if (FCurrentLine='') and (FContentPrefix<>'') then
     FCurrentLine:=FContentPrefix;
  FCurrentLine:=FCurrentLine+aText;
end;

procedure TBaseMarkdownWriter.AppendToLine(aFmt: UTF8String;
  aArgs: array of const; DoEscape: boolean);
begin
  AppendToLine(Format(aFmt,aArgs),DoEscape);
end;

procedure TBaseMarkdownWriter.EmitLine(aText: UTF8String; DoEscape: boolean);
begin
  OutputCurrentLine;
  AppendToLine(aText,DoEscape);
  OutputCurrentLine;
end;

procedure TBaseMarkdownWriter.EmitLine(aFmt: UTF8String; aArgs: array of const;
  DoEscape: boolean);
begin
  EmitLine(Format(aFmt,aArgs),DoEscape);
end;

procedure TBaseMarkdownWriter.AppendLink(const aText, aLink: String);
begin
  AppendToLine(CreateLink(aText,aLink),False);
end;

procedure TBaseMarkdownWriter.AppendRendered(aText: String; aRender: TRender);
begin
  StartSpan(aRender);
  AppendToLine(aText);
  EndSpan(aRender);
end;

procedure TBaseMarkdownWriter.AppendKeyWord(aText: String);
begin
  AppendRendered(aText,KeywordRendering);
end;

procedure TBaseMarkdownWriter.AppendSymbol(aText: String);
begin
  AppendRendered(aText,SymbolRendering);
end;

procedure TBaseMarkdownWriter.AppendTableHeader(aHeaders: array of String);

Var
  S : String;

begin
  DescrBeginTable(Length(aHeaders),False);
  DescrBeginTableHeadRow;
  for S in aHeaders do
    begin
    DescrBeginTableCell;
    AppendToLine(S);
    DescrEndTableCell;
    end;
  DescrEndTableHeadRow;
end;


procedure TBaseMarkdownWriter.EmitCode(aCodeBlock: String; aIndent : Integer = 0);

Var
  L : TStringList;

begin
  L:=TStringList.Create;
  try
    L.Text:=aCodeBlock;
    EmitCode(L,aIndent);
  finally
    L.Free;
  end;
end;

procedure TBaseMarkdownWriter.EmitCode(aCodeBlock: TStrings; aIndent : Integer = 0);
var
  S,aPrefix : string;
begin
  aPrefix:=StringOfChar(' ',aIndent);
  For S in aCodeBlock do
    EmitCodeLine(aPrefix+S);
end;

procedure TBaseMarkdownWriter.EmitCodeLine(aCodeLine: string);
begin
  EmitLine(aCodeLine,False);
end;

procedure TBaseMarkdownWriter.DescrWriteText(const AText: DOMString);
begin
  AppendToLine(aText);
end;

procedure TBaseMarkdownWriter.DescrBeginBold;
begin
  AppendToLine('**',False);
end;

procedure TBaseMarkdownWriter.DescrEndBold;
begin
  AppendToLine('**',False);
end;

procedure TBaseMarkdownWriter.DescrBeginItalic;
begin
  AppendToLine('*',False);
end;

procedure TBaseMarkdownWriter.DescrEndItalic;
begin
  AppendToLine('*',False);
end;

procedure TBaseMarkdownWriter.DescrBeginEmph;
begin
  AppendToLine('*',False);
end;

procedure TBaseMarkdownWriter.DescrEndEmph;
begin
  AppendToLine('*',False);
end;

procedure TBaseMarkdownWriter.StartSpan(aRender : TRender);

begin
  case aRender of
    rStrong : AppendToLine('**',False);
    rEmphasis : AppendToLine('*',False);
    rCode : AppendToLine('`',False);
  end;
end;

procedure TBaseMarkdownWriter.PushIndent(aNewIndent: Byte);
begin
  if FCurrentIndentIndex>=MaxIndents then
     FPDocError(SErrMaxIndentStack);
  Inc(FCurrentIndentIndex);
  Findents[FCurrentIndentIndex]:=aNewIndent;
  CalcPrefix;
end;

procedure TBaseMarkdownWriter.PopIndent;
begin
  if FCurrentIndentIndex<=0 then
     FPDocError(SErrMinIndentStack);
  Dec(FCurrentIndentIndex);
  CalcPrefix;
end;

procedure TBaseMarkdownWriter.StartList(aType: TListType);
begin
  If FListStack>=MaxLists then
    FPDocError(SErrMaxListStack);
  OutputCurrentLine;
  Inc(FListStack);
  FListTypes[FListStack]:=aType;
  if FListStack>1 then
    PushIndent(CurrentIndent+Length(GetListprefix));
end;

procedure TBaseMarkdownWriter.StopList(aType: TListType);
begin
  OutputCurrentLine;
  If FListStack<=0 then
    FPDocError(SErrMinListStack);
  if FListTypes[FListStack]<>aType then
    FPDocError(SErrPopListStack);
  if FListStack>1 then
    PopIndent;
  Dec(FListStack);
end;

procedure TBaseMarkdownWriter.BeginIgnore;
begin
  Inc(FIgnoreCount);
end;

procedure TBaseMarkdownWriter.EndIgnore;
begin
  If FignoreCount>0 then
    Dec(FIgnoreCount);
end;

procedure TBaseMarkdownWriter.DoLineBreak;
begin
  if FCurrentLine<>'' then
    begin
    FCurrentLine:=FCurrentLine+'  ';
    OutputCurrentLine;
    end;
end;

function TBaseMarkdownWriter.InList: Boolean;
begin
  Result:=FlistStack>0;
end;

function TBaseMarkdownWriter.CurrentList: TListType;
begin
  if FListStack=0 then
    FPDOcError(SErrNotInList);
  Result:=FListTypes[FListStack];
end;

procedure TBaseMarkdownWriter.EndSpan(aRender : TRender);

begin
  case aRender of
    rStrong : AppendToLine('**',False);
    rEmphasis : AppendToLine('*',False);
    rCode : AppendToLine('`',False);
  end;
end;

procedure TBaseMarkdownWriter.DescrBeginUnderline;
begin
  StartSpan(UnderlineRendering);
end;

procedure TBaseMarkdownWriter.DescrEndUnderline;
begin
  EndSpan(UnderlineRendering);
end;

procedure TBaseMarkdownWriter.DescrWriteImageEl(const AFileName, ACaption, ALinkName : DOMString);

Var
  aLink,D,FN : String;
  L : integer;
begin
  // Determine URL for image.
  If (Module=Nil) then
    D:=Allocator.GetRelativePathToTop(Package)
  else
    D:=Allocator.GetRelativePathToTop(Module);
  L:=Length(D);
  If (L>0) and (D[L]<>'/') then
    D:=D+'/';

  FN:=D + BaseImageURL+ Utf8Encode(AFileName);
  EnsureEmptyLine;
  aLink:='!['+UTF8Encode(aCaption)+']('+FN+')';
  AppendToLine(aLink,False);
end;

procedure TBaseMarkdownWriter.DescrWriteFileEl(const AText: DOMString);

begin
  AppendRendered(UTF8Encode(aText),FileRendering);
end;

procedure TBaseMarkdownWriter.DescrWriteKeywordEl(const AText: DOMString);
begin
  AppendKeyWord(UTF8ENcode(aText));
end;

procedure TBaseMarkdownWriter.DescrWriteVarEl(const AText: DOMString);
begin
  AppendRendered(UTF8Encode(aText),VarRendering);
end;

procedure TBaseMarkdownWriter.DescrBeginLink(const AId: DOMString);
var
  a,s,n : String;

begin
  a:=UTF8Encode(AId);
  s := UTF8Encode(ResolveLinkID(a));

  if Length(s) > 0 then
    begin
    FLink:=S;
    AppendToLine('[');
    end
  else
    begin
    FLink:='';
    if assigned(module) then
      s:=module.name
    else
      s:='?';
    if a='' then a:='<empty>';
    if Assigned(CurrentContext) then
      N:=CurrentContext.Name
    else
      N:='?';
    DoLog(SErrUnknownLinkID, [s,n,a]);
    LinkUnresolvedInc();
    end
end;

procedure TBaseMarkdownWriter.DescrEndLink;
begin
  AppendToLine(']('+FLink+') ',false);
  FLink:='';
end;

procedure TBaseMarkdownWriter.DescrBeginURL(const AURL: DOMString);
begin
  FLink:=UTF8Encode(aURL);
  AppendToLine('[');
end;

procedure TBaseMarkdownWriter.DescrEndURL;
begin
  AppendToLine(']('+FLink+') ',false);
  FLink:='';
end;

procedure TBaseMarkdownWriter.DescrWriteLinebreak;
begin
  AppendToLine('  ');
  OutputCurrentLine;
end;

procedure TBaseMarkdownWriter.DescrBeginParagraph;
begin
  EnsureEmptyLine;
end;

procedure TBaseMarkdownWriter.DescrEndParagraph;
begin
  EnsureEmptyLine;
end;

procedure TBaseMarkdownWriter.DescrBeginCode(HasBorder: Boolean; const AHighlighterName: String);

Var
  hl : string;

begin
  hl:=AHighlighterName;
  if SameText(hl,'Pascal') or (hl='') then
    hl:='delphi';
  OutputCurrentLine;
  AppendToLine('```'+hl,False);
  PushIndent(0);
end;

procedure TBaseMarkdownWriter.DescrWriteCodeLine(const ALine: String);
begin
  EmitCodeLine(aLine);
end;

procedure TBaseMarkdownWriter.DescrEndCode;
begin
  OutputCurrentLine;
  AppendToLine('```',False);
  OutputCurrentLine;
  PopIndent;
end;

procedure TBaseMarkdownWriter.DescrBeginOrderedList;
begin
  StartList(ltOrdered);
end;

procedure TBaseMarkdownWriter.DescrEndOrderedList;
begin
  StopList(ltOrdered);
end;

procedure TBaseMarkdownWriter.DescrBeginUnorderedList;
begin
  StartList(ltUnordered);
end;

procedure TBaseMarkdownWriter.DescrEndUnorderedList;
begin
  StopList(ltUnordered);
end;

procedure TBaseMarkdownWriter.DescrBeginDefinitionList;
begin
  StartList(ltDefinition);
end;

procedure TBaseMarkdownWriter.DescrEndDefinitionList;
begin
  StopList(ltDefinition);
end;


function TBaseMarkdownWriter.GetListPrefix : String;

begin
  Case CurrentList of
    ltOrdered   : Result:='1. ';
    ltUnordered : Result:='- ';
    ltDefinition : Result:=':    ';
  end;
end;

procedure TBaseMarkdownWriter.DescrBeginListItem;
Var
  Pref : String;
begin
  Pref:=GetListPrefix;
  AppendToLine(Pref);
end;


procedure TBaseMarkdownWriter.DescrEndListItem;
begin
  OutputCurrentLine;
end;

procedure TBaseMarkdownWriter.DescrBeginDefinitionTerm;
begin
  if MarkDownEngine=meMkDocs then
    EnsureEmptyLine
  else
    begin
    AppendToLine('- ');
    StartSpan(DefinitionTermRender);
    end;
end;

procedure TBaseMarkdownWriter.DescrEndDefinitionTerm;
begin
  if MarkDownEngine=meMkDocs then
    OutputCurrentLine
  else
    EndSpan(DefinitionTermRender);
end;

procedure TBaseMarkdownWriter.DescrBeginDefinitionEntry;
begin
  if MarkDownEngine=meMkDocs then
    begin
    AppendToLine(':    ');
    Indent;
    end
  else
    AppendToLine(DefinitionSeparator);
end;

procedure TBaseMarkdownWriter.DescrEndDefinitionEntry;
begin
  OutputCurrentLine;
  if MarkDownEngine=meMkDocs then
    begin
    Undent;
    EnsureEmptyLine;
    end;
end;

procedure TBaseMarkdownWriter.DescrBeginSectionTitle;
begin
  EnsureEmptyLine;
end;

procedure TBaseMarkdownWriter.DescrBeginSectionBody;
begin
  EnsureEmptyLine;
end;

procedure TBaseMarkdownWriter.DescrEndSection;
begin
  EnsureEmptyLine;
end;

procedure TBaseMarkdownWriter.DescrBeginRemark;
begin
  if MarkDownEngine=meMkDocs then
    begin
    EnsureEmptyLine;
    AppendToLine('!!! Remark',False);
    OutputCurrentLine;
    end
  else
    FContentPrefix:='> ';
end;

procedure TBaseMarkdownWriter.DescrEndRemark;
begin
  if MarkDownEngine=meMkDocs then
    begin
    OutputCurrentLine;
    AppendToLine('!!!',False);
    end;
  EnsureEmptyLine;
  FContentPrefix:='';
end;

procedure TBaseMarkdownWriter.DescrBeginTable(ColCount: Integer; HasBorder: Boolean);
begin
  EnsureEmptyLine;
  FTableColCount:=ColCount;
end;

procedure TBaseMarkdownWriter.DescrEndTable;
begin
  EnsureEmptyLine;
end;

procedure TBaseMarkdownWriter.DescrBeginTableCaption;
begin
  BeginIgnore;
end;

procedure TBaseMarkdownWriter.DescrEndTableCaption;
begin
  EndIgnore;
end;

procedure TBaseMarkdownWriter.DescrBeginTableHeadRow;

begin
end;

procedure TBaseMarkdownWriter.DescrEndTableHeadRow;

Var
  I : Integer;

begin
  AppendToLine(' |',False);
  OutputCurrentLine;
  AppendToLine('|',False);
  For I:=1 to FTableColCount do
    AppendToLine('---|',False);
  OutputCurrentLine;
end;

procedure TBaseMarkdownWriter.DescrBeginTableRow;
begin
end;

procedure TBaseMarkdownWriter.DescrEndTableRow;
begin
  AppendToLine(' |',False);
  OutputCurrentLine;
end;

procedure TBaseMarkdownWriter.DescrBeginTableCell;
begin
  AppendToLine('| ',False);
end;

procedure TBaseMarkdownWriter.DescrEndTableCell;
begin
  AppendToLine(' ',False);
end;

function TBaseMarkdownWriter.OutputCurrentLine: Boolean;
begin
  If FIgnoreCount>0 then
    exit(False);
  Result:=FCurrentLine<>'';
  if Result then
    begin
    FMarkDown.Add(Prefix+FCurrentLine);
    FCurrentLine:='';
    end;
end;

constructor TBaseMarkdownWriter.Create(APackage: TPasPackage;
  AEngine: TFPDocEngine);
begin
  inherited Create(APackage, AEngine);
  FMarkDown:=TStringList.Create;
  FMetaData:=TStringList.Create;
  FMetaData.NameValueSeparator:=':';
  FIndents[FCurrentIndentIndex]:=0;
  CalcPrefix;
  Theme:='readthedocs';
end;

destructor TBaseMarkdownWriter.Destroy;
begin
  FreeAndNil(FMarkDown);
  FreeAndNil(FMetadata);
  inherited Destroy;
end;

procedure TBaseMarkdownWriter.AppendHeader(aLevel: THeaderLevel;
  const AHeader: String; DoEscape : Boolean = true);
begin
  EnsureEmptyLine;
  AppendToLine(StringOfChar('#',aLevel)+' ',False);
  AppendToLine(aHeader,DoEscape);
end;

procedure TBaseMarkdownWriter.Indent;
begin
  Inc(FIndents[FCurrentIndentIndex],IndentSize);
  CalcPrefix;
end;

procedure TBaseMarkdownWriter.Undent;
begin
  if IndentSize>CurrentIndent then
    FPDocError(SErrIndentMismatch);
  Dec(FIndents[FCurrentIndentIndex],IndentSize);
  CalcPrefix;
end;

procedure TBaseMarkdownWriter.ClearMarkDown;
begin
  FMarkDown.Clear;
  FMetaData.Clear;
  FCurrentLine:='';
  FContentPrefix:='';
  FCurrentIndentIndex:=0;;
  FIndents[FCurrentIndentIndex]:=0;
  CalcPrefix;
end;


procedure TBaseMarkdownWriter.EnsureEmptyLine;
begin
  if OutputCurrentLine then
    FMarkDown.Add('')
  else if (FmarkDown.Count>0) and (FMarkDown[FmarkDown.Count-1]<>'') then
    FMarkDown.Add('');
 end;

procedure TBaseMarkdownWriter.SaveToFile(aFileName: string);

Var
  Doc: TStrings;
  I : Integer;
  N,V : String;

begin
  Doc:=TStringList.Create;
  try
    if FMetadata.Count>0 then
      begin
      Doc.Add('---');
      For I:=0 to FMetadata.Count-1 do
        begin
        FMetaData.GetNameValue(I,N,V);
        Doc.Add(Lowercase(N)+': "'+V+'"');
        end;
      Doc.Add('---');
      end;
    Doc.AddStrings(FMarkDown);
    Doc.SaveToFile(aFileName);
  finally
    Doc.Free;
  end;
end;

procedure TBaseMarkdownWriter.AddMetaData(const aName, aValue: string);
begin
  FMetadata.Values[aName]:=aValue;
end;

end.

