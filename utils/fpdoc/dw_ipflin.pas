{
    FPDoc IPF Writer
    Copyright (c) 2010 by Graeme Geldenhuys (graemeg@gmail.com)

    * Linear IPF output for use with fpGUI or OS/2's help systems.

    See the file COPYING, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
unit dw_ipflin;

{$mode objfpc}{$H+}
{$WARN 5024 off : Parameter "$1" not used}
interface

uses
  Classes, DOM, dGlobals, PasTree, dwLinear;

const
  { Change this into the name of your writer}
  IPFWriterName = 'ipf';
  { Comprehensible description goes here:}
  SIPFUsageWriterDescr = 'Writes output in fpGUI and OS/2''s IPF help format';
  { Extension for the template }
  TIPFExtension = '.ipf';

type

  { TIPFNewWriter }

  TIPFNewWriter = class(TLinearWriter)
  private
    InPackageOverview: Boolean;
    InHeading: Boolean;
    FInHeadingText: string;
    OrderedList: boolean;
    TableCaptionWritten: Boolean;
    InTableCell: Boolean;
    InTypesDeclaration: Boolean;
    SuspendWriting: Boolean;
    LastSubSection: String;
  protected
    FLink: String;
    FTableCount : Integer;
    FInVerbatim : Boolean;
    Inlist,
    fColCount: integer;
    // extras
    procedure Write(const s: String); override;
    procedure WriteBeginDocument; override;
    procedure WriteEndDocument; override;
    // Linear documentation methods overrides;
    procedure WriteLabel(Const S : String); override;
    procedure WriteIndex(Const S : String); override;
    procedure WriteType(const s: string); override;
    procedure WriteVariable(const s: string); override;
    procedure WriteConstant(const s: string); override;
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
    Procedure StartVersion; override;
    Procedure StartSeeAlso; override;
    Procedure EndSeeAlso; override;
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
    procedure StartOverview(Const What : String;WithAccess : Boolean); override;
    procedure EndOverview; override;
    procedure WriteOverviewMember(const ALabel,AName,Access,ADescr : String); override;
    procedure WriteOverviewMember(const ALabel,AName,ADescr : String); override;
    procedure DescrBeginURL(const AURL: DOMString); override;
    procedure DescrEndURL; override;
    // Description node conversion. Overrides for TFPDocWriter.
    procedure DescrBeginBold; override;
    procedure DescrEndBold; override;
    procedure DescrBeginItalic; override;
    procedure DescrEndItalic; override;
    procedure DescrBeginEmph; override;
    procedure DescrEndEmph; override;
    procedure DescrWriteText(const AText: DOMString); override;
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
    constructor Create(APackage: TPasPackage; AEngine: TFPDocEngine); override;
    class function FileNameExtension: string; override;
    procedure WriteClassInheritanceOverview(ClassDecl: TPasClassType); override;
  end;



implementation

uses
  fpdocstrs, SysUtils, dwriter;


{ TFPDocWriter overrides }


procedure TIPFNewWriter.DescrBeginBold;
begin
  { Start bold output }
  WriteLn(':hp2.');
end;

procedure TIPFNewWriter.DescrEndBold;
begin
  { End bold output }
  WriteLn(':ehp2.');
end;

procedure TIPFNewWriter.DescrBeginItalic;
begin
  { Start italic output }
  WriteLn(':hp1.');
end;

procedure TIPFNewWriter.DescrEndItalic;
begin
  { End italic output }
  WriteLn(':ehp1.');
end;

procedure TIPFNewWriter.DescrBeginEmph;
begin
  { Start emphasized output }
  Write(':hp1.');
end;

procedure TIPFNewWriter.DescrEndEmph;
begin
  { End emphasized output }
  Write(':ehp1.');
end;

procedure TIPFNewWriter.DescrWriteText(const AText: DOMString);
const
  cMax = 100;
var
  sl: TStringlist;
  i: integer;
  lText: string;
begin
  // IPF has an imposed line length limit.
  if (Length(AText) > cMax) then // then we need to wrap the text.
  begin
    lText := WrapText(UTF8Encode(AText), LineEnding, [' ', '-', #9], cMax);
    sl := TStringlist.Create;
    try
      sl.Text := lText;
      for i := 0 to sl.Count-1 do
        inherited DescrWriteText(UTF8Decode(sl.Strings[i] + LineEnding));
    finally
      sl.Free;
    end;
  end
  else
    inherited DescrWriteText(AText);
end;

procedure TIPFNewWriter.DescrWriteFileEl(const AText: DOMString);
begin
  { format as file name }
  Write(':hp3.');
  DescrWriteText(AText);
  Write(':ehp3.');
end;

procedure TIPFNewWriter.DescrWriteKeywordEl(const AText: DOMString);
begin
  { Format as keyword }
  Write(':hp1.');
  DescrWriteText(AText);
  Write(':ehp1.');
end;

procedure TIPFNewWriter.DescrWriteVarEl(const AText: DOMString);
begin
  { Format as variable }
  Write(':hp1.');
  DescrWriteText(AText);
  Write(':ehp1.');
end;

procedure TIPFNewWriter.DescrBeginLink(const AId: DOMString);
begin
  { Start link to label ID - links are never nested.}
  FLink := Engine.ResolveLink(Module, UTF8Encode(AId));
  FLink := StringReplace(FLink, ':', '_', [rfReplaceAll]);
  FLink := StringReplace(FLink, '.', '_', [rfReplaceAll]);
  WriteF(':link reftype=hd refid=%s.', [flink]);
end;

procedure TIPFNewWriter.DescrEndLink;
begin
  { End link to label ID}
  Write(':elink.');
end;

procedure TIPFNewWriter.DescrWriteLinebreak;
begin
  { Start a new line. }
  WriteLn('');
  WriteLn('.br'); // must be at the beginning of a line, hence the previous writeln call
end;

procedure TIPFNewWriter.DescrBeginParagraph;
begin
  { Start a new paragraph }
  Writeln(':p.');
end;

procedure TIPFNewWriter.DescrEndParagraph;
begin
  { End current paragraph }
  writeln('');
end;

procedure TIPFNewWriter.DescrBeginCode(HasBorder: Boolean;
  const AHighlighterName: String);
begin
  { Start block of code }
  StartListing(HasBorder,'');
end;

procedure TIPFNewWriter.DescrWriteCodeLine(const ALine: String);
begin
  { Write line of code }
  DescrWriteText(UTF8Decode(ALine + LineEnding));
//  writeln(EscapeText(ALine));
end;

procedure TIPFNewWriter.DescrEndCode;
begin
  { End block of code }
  EndListing;
end;

procedure TIPFNewWriter.DescrBeginOrderedList;
begin
  {  Start numbered list }
  OrderedList := True;
  writeln('');
  writeln(':ol.');
end;

procedure TIPFNewWriter.DescrEndOrderedList;
begin
  {  End numbered list }
  writeln('');
  writeln(':eol.');
//  writeln(':p.');
end;

procedure TIPFNewWriter.DescrBeginUnorderedList;
begin
  {  Start bulleted list }
  OrderedList := False;
  writeln('');
  if not InTableCell then
    writeln(':ul.')
  else
    writeln(':lines.');
end;

procedure TIPFNewWriter.DescrEndUnorderedList;
begin
  {  End bulleted list }
  writeln('');
  if not InTableCell then
    writeln(':eul.')
  else
    writeln(':elines.');
end;

procedure TIPFNewWriter.DescrBeginDefinitionList;
begin
  {  Start definition list }
  writeln('');
  writeln(':dl tsize=25 compact.');
end;

procedure TIPFNewWriter.DescrEndDefinitionList;
begin
  {  End definition list }
  writeln('');
  writeln(':edl.');
//  writeln(':p.');
end;

procedure TIPFNewWriter.DescrBeginListItem;
begin
  {  Start list item (both bulleted/numbered) }
  if not InTableCell then
    write(':li.');
end;

procedure TIPFNewWriter.DescrEndListItem;
begin
  {  End list item (both bulleted/numbered) }
  writeln('');
end;

procedure TIPFNewWriter.DescrBeginDefinitionTerm;
begin
  {  Start definition term }
  writeln(':dt.');
end;

procedure TIPFNewWriter.DescrEndDefinitionTerm;
begin
  {  End definition term }
  writeln('');
end;

procedure TIPFNewWriter.DescrBeginDefinitionEntry;
begin
  {  start definition explanation }
  writeln(':dd.');
end;

procedure TIPFNewWriter.DescrEndDefinitionEntry;
begin
  {  End definition explanation }
  writeln('');
end;

procedure TIPFNewWriter.DescrBeginSectionTitle;
begin
  {  Start section title }
end;

procedure TIPFNewWriter.DescrBeginSectionBody;
begin
  {  Start section body }
end;

procedure TIPFNewWriter.DescrEndSection;
begin
  {  End section body }
end;

procedure TIPFNewWriter.DescrBeginRemark;
begin
  {  Start remark paragraph }
  writeln('');
  writeln(':nt text=''Remark: ''.');
end;

procedure TIPFNewWriter.DescrEndRemark;
begin
  {  End remark paragraph }
  writeln('');
  writeln(':ent.');
end;

procedure TIPFNewWriter.DescrBeginTable(ColCount: Integer; HasBorder: Boolean);
var
  i: integer;
  cols: string;
  f: string;
begin
  {  Start table with ColCount columns, and with border }
  cols := '';
  for i := 0 to ColCount-1 do
  begin
    if i = 0 then
      cols := cols + '35 '  // first colum is 30 characters
    else
      cols := cols + '50 '; // every other colum is 50 characters each
  end;
  if HasBorder then
    f := ' frame=box.'
  else
    f := ' frame=none.';
  writeln(':table cols=''' + Trim(cols) + ''' rules=both' + f);
end;

procedure TIPFNewWriter.DescrEndTable;
begin
  writeln(':etable.');
end;

procedure TIPFNewWriter.DescrBeginTableCaption;
begin
  //writeln('.* GG');
  SuspendWriting := True;
  // do nothing
//  TableCaptionWritten := False;
end;

procedure TIPFNewWriter.DescrEndTableCaption;
begin
  // do nothing
  SuspendWriting := False;
  writeln('');
end;

procedure TIPFNewWriter.DescrBeginTableHeadRow;
begin
//  TableCaptionWritten := True;
  SuspendWriting := False;
  writeln(':row.');
end;

procedure TIPFNewWriter.DescrEndTableHeadRow;
begin
  // do nothing
end;

procedure TIPFNewWriter.DescrBeginTableRow;
begin
//  TableCaptionWritten := True;
  SuspendWriting := False;
  writeln(':row.');
end;

procedure TIPFNewWriter.DescrEndTableRow;
begin
  writeln('');
end;

procedure TIPFNewWriter.DescrBeginTableCell;
begin
  write(':c.');
  InTableCell := True;
end;

procedure TIPFNewWriter.DescrEndTableCell;
begin
  // do nothing
  writeln('');
  InTableCell := False;
end;

constructor TIPFNewWriter.Create(APackage: TPasPackage; AEngine: TFPDocEngine);
begin
  inherited Create(APackage, AEngine);
  TableCaptionWritten := True;
  SuspendWriting := False;
  InTableCell := False;
  InTypesDeclaration := False;
end;

procedure TIPFNewWriter.WriteClassInheritanceOverview(ClassDecl: TPasClassType);
var
  ancestor: TPasClassType;
  ancestor2: TPasType;
  List: TStringList;
  i: integer;
  indent: integer;

  procedure WriteDescription(const Idx: integer);
  var
    s: string;
    o: TPasClassType;
  begin
    if List.Objects[i] <> nil then
    begin
      o := List.Objects[i] as TPasClassType;
      if ClassDecl.Name <> o.Name then
      begin
        s := ChangeFileExt(ExtractFileName(o.SourceFilename), '');
        s := '#' + PackageName + '.' + s + '.' + o.Name;
        DescrBeginLink(UTF8Decode(s));
        Write(o.Name);
        DescrEndLink;
        writeln('');
      end
      else
      begin
        { The topic being viewed doesn't need a link to itself }
        writeln(List[i]);
      end;
    end
    else
    begin
      { we only have text for it. }
      Writeln(List[i]);
    end;
  end;

begin
  List := TStringList.Create;
  List.Sorted := False;
  { add the initial class }
  List.AddObject(ClassDecl.Name, ClassDecl);

  ancestor := nil;

  if Assigned(ClassDecl.AncestorType) and ClassDecl.AncestorType.InheritsFrom(TPasClassType) then
    { all is well, we have our first ancestor to get us started with the hierarchy traversal }
    ancestor := TPasClassType(ClassDecl.AncestorType)
  else
  begin
    { here we only have one history item to output - and not part of fpdoc hierarchy data }
    if Assigned(ClassDecl.AncestorType) then
    begin
      ancestor2 := ClassDecl.AncestorType;
      if Assigned(ancestor2) then
      begin
        List.AddObject(ancestor2.Name, nil);
        ancestor2 := nil; { prevent any further attempts at traversal }
      end;
    end;
  end;

  while Assigned(ancestor) do
  begin
    List.AddObject(ancestor.Name, ancestor);
    if Assigned(ancestor.AncestorType) and ancestor.AncestorType.InheritsFrom(TPasClassType) then
      ancestor := TPasClassType(ancestor.AncestorType)
    else
    begin
      { we hit the end of the road }
      ancestor2 := ancestor.AncestorType;
      if Assigned(ancestor2) then
        List.AddObject(ancestor2.Name, nil);
      ancestor := nil;  { prevent any further attempts at traversal }
    end;
  end;

  if List.Count > 1 then
  begin
    { output a title }
    Writeln(':p.');
    writeln(':lm margin=1.');
    DescrBeginBold;
    WriteLn(SDocInheritanceHierarchy);
    DescrEndBold;
    { now output the hierarchy }
    indent := 3;
    { we go from least significant to most, hence the reversed loop }
    for i := List.Count-1 downto 0 do
    begin
      Write(Format(':lm margin=%d.', [indent]));
      { each level is indented 2 character positions more than the previous one }
      if (indent > 3) then
      begin
        writeln('|');
        write('+--');
      end
      else
        write(':xmp.');
      WriteDescription(i);
      inc(indent, 2);
    end;
    WriteLn(':lm margin=1.:exmp.');
  end;

  List.Free;
end;

{ TLinearWriter overrides}

class function TIPFNewWriter.FileNameExtension: string;
begin
  Result := TIPFExtension;
end;

procedure TIPFNewWriter.DescrBeginURL(const AURL: DOMString);
begin
  Write(':link reftype=launch object=''netscape'' data=''' + UTF8Encode(AURL) + '''.');
end;

procedure TIPFNewWriter.DescrEndURL;
begin
  Write(':elink.');
end;

function TIPFNewWriter.GetLabel(AElement: TPasElement): String;
begin
  if AElement.ClassType = TPasUnresolvedTypeRef then
    Result := Engine.ResolveLink(Module, AElement.Name)
  else
  begin
    Result := AElement.PathName;
    Result := LowerCase(Copy(Result, 2, Length(Result) - 1));   // Remove # infront of eg: '#Corelib' string
  end;
  Result := StringReplace(Result, '.', '_', [rfReplaceAll]);
  Result := StringReplace(Result, ' ', '_', [rfReplaceAll]);
end;

function TIPFNewWriter.EscapeText(S: String): String;
var
  i: Integer;
begin
  Result:='';
  for i := 1 to Length(S) do
    case S[i] of
      '.':              // Escape these characters
        Result := Result + '&per.';
      ':':
        Result := Result + '&colon.';
      ',':
        Result := Result + '&comma.';
      '&':
        Result := Result + '&amp.';
//      '_':
//        Result := Result + '&us.';
      '^':
        Result := Result + '&caret.';
      '''':
        Result := Result + '&apos.';
      '*':
        Result := Result + '&asterisk.';
      '@':
        Result := Result + '&atsign.';
      '\':
        Result := Result + '&bslash.';
      '"':
        Result := Result + '&cdq.';
      '-':
        Result := Result + '&hyphen.';
      //'°':
      //  Result := Result + '&degree.';
      '$':
        Result := Result + '&dollar.';
      '=':
        Result := Result + '&eq.';
      '!':
        Result := Result + '&xclam.';
      '>':
        Result := Result + '&gt.';
      '(':
        Result := Result + '&lpar.';
      ')':
        Result := Result + '&rpar.';
      '+':
        Result := Result + '&plus.';
      '[':
        Result := Result + '&lbracket.';
      ']':
        Result := Result + '&rbracket.';
      else
        Result := Result + S[i];
    end;
end;

function TIPFNewWriter.StripText(S: String): String;
var
  I: Integer;
begin
  Result := '';
  for i := 1 to Length(S) do
    if not (S[i] in ['&','{','}','#'{,'_'},'$','%','''','~','^', '\', ' ', '<', '>']) then
      Result := Result + S[i];
end;

procedure TIPFNewWriter.Write(const s: String);
begin
  if SuspendWriting then
    Exit;
  inherited Write(s);
end;

procedure TIPFNewWriter.WriteBeginDocument;
begin
  fColCount := 0;
  Writeln(':userdoc.');
  WriteComment('This file has been created automatically by FPDoc');
  WriteComment('IPF output (c) 2010-2012 by Graeme Geldenhuys (graemeg@gmail.com)');
  writeln('');
  Writeln(':docprof toc=12345.');
  WriteLn(':title.' + PackageName);
  writeln('');
  writeln('');
  writeln(':h1.' + PackageName);
  InPackageOverview := True;
//  inherited WriteBeginDocument;
end;

procedure TIPFNewWriter.WriteEndDocument;
begin
  inherited WriteEndDocument;
  writeln('');
  writeln('');
  writeln(':euserdoc.');
  writeln('');
end;

procedure TIPFNewWriter.WriteLabel(const S: String);
var
  x: String;
begin
  x := StringReplace(s, ':', '_', [rfReplaceAll]);

  if InHeading and (x <> '') then
  begin
    WriteLnF(FInHeadingText, [ ' name=' + LowerCase(x)]); // LowerCase(StripTexT(x))]);
    Writeln('');
    FInHeadingText := '';
    InHeading := False;
  end
  else
  begin
    WriteLnF(FInHeadingText, [ '' ]);
    Writeln('');
    FInHeadingText := '';
    InHeading := False;
  end;
end;

procedure TIPFNewWriter.WriteIndex(const S: String);
begin
//  writeln(':i1 id=' + s + '.');
end;

procedure TIPFNewWriter.WriteType(const s: string);
begin
  writeln('');
  Writeln('.* -------------------------------------------------');
  WriteLnF(':h5 name=%s.%s', [lowercase(PackageName+'_'+ModuleName+'_'+s), s]);
//  inherited WriteType(s);
end;

procedure TIPFNewWriter.WriteVariable(const s: string);
begin
  writeln('');
  Writeln('.* -------------------------------------------------');
  WriteLnF(':h5 name=%s.%s', [lowercase(PackageName+'_'+ModuleName+'_'+s), s]);
end;

procedure TIPFNewWriter.WriteConstant(const s: string);
begin
  writeln('');
  Writeln('.* -------------------------------------------------');
  WriteLnF(':h5 name=%s.%s', [lowercase(PackageName+'_'+ModuleName+'_'+s), s]);
end;

procedure TIPFNewWriter.StartListing(Frames: Boolean; const name: String);
begin
//  writeln('');
  writeln(':xmp.');
end;

procedure TIPFNewWriter.EndListing;
begin
  writeln(':exmp.');
end;

procedure TIPFNewWriter.WriteCommentLine;
begin
  Writeln('');
  Writeln('.* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
end;

procedure TIPFNewWriter.WriteComment(Comment : String);
begin
  Writeln('.* ' + Comment);
end;

procedure TIPFNewWriter.StartChapter(ChapterName : String);
begin
  InHeading := True;
  Writeln('');
  Writeln('');
  WriteCommentLine;
  WriteComment('Chapter: ' + ChapterName);
  WriteCommentLine;
  FInHeadingText := ':h2%s. ' + ChapterName;
  //Writeln(':h2.' + ChapterName);
  //Writeln('');
end;

procedure TIPFNewWriter.StartSection(SectionName : String);
begin
  InHeading := True;
  Writeln('');
  Writeln('');
  WriteCommentLine;
  WriteComment('Section: ' + SectionName);
  WriteCommentLine;
  writeln('');

  if SameText(SectionName, SDocOverview) then
  begin
    writeln(':p.');
    writeln(':p.');
    writeln(':lm margin=1.');
    DescrBeginBold;
    WriteLn(SDocOverview);
    DescrEndBold;
//    writeln(':lm margin=3.');
    writeln('.br');
  end

  else if InPackageOverview then
  begin
    FInHeadingText := ':h2%s. ' + SectionName;
//    Writeln(':h2.' + SectionName);
    InPackageOverview := False;
  end
  else
  begin
    FInHeadingText := ':h3%s. ' + SectionName;
//    Writeln(':h3.' + SectionName);
    InPackageOverview := False;
  end;
//  Writeln('');
end;

procedure TIPFNewWriter.StartSubSection(SubSectionName : String);
begin
  LastSubSection := Lowercase(SubSectionName);
  InHeading := True;
  Writeln('');
  WriteCommentLine;
  FInHeadingText := ':h4%s. ' + SubSectionName;
  //Writeln(':h4.' + SubSectionName);
end;

procedure TIPFNewWriter.StartSubSubSection(SubSubSectionName : String);
begin
  InHeading := True;
  FInHeadingText := ':h5%s. ' + SubSubSectionName;
  //Writeln(':h5.' + SubSubSectionName);
end;

procedure TIPFNewWriter.StartProcedure;
begin
  //writeln('');
  //writeln(':ul.');
end;

procedure TIPFNewWriter.EndProcedure;
begin
  //writeln('');
  //writeln(':eul.');
end;

procedure TIPFNewWriter.StartSynopsis;
begin
  writeln('');
  writeln(':p.');
  writeln(':lm margin=1.');
  writeln(':hp2.' + SDocSynopsis + ':ehp2.');
  writeln('.br');
  writeln(':lm margin=3.');
end;

procedure TIPFNewWriter.StartDeclaration;
begin
  writeln('');
  writeln(':p.');
  writeln(':lm margin=1.');
  writeln(':hp2.' + SDocDeclaration + ':ehp2.');
  writeln(':lm margin=3.');
end;

procedure TIPFNewWriter.StartVisibility;
begin
  writeln('');
  writeln(':p.');
  writeln(':lm margin=1.');
  writeln(':hp2.' + SDocVisibility + ':ehp2.');
  writeln(':lm margin=3.');
  writeln('.br');
end;

procedure TIPFNewWriter.StartDescription;
begin
  writeln('');
  writeln(':p.');
  writeln(':lm margin=1.');
  writeln(':hp2.' + SDocDescription + ':ehp2.');
  writeln(':lm margin=3.');
  writeln('.br');
end;

procedure TIPFNewWriter.StartErrors;
begin
  writeln('');
  writeln(':p.');
  writeln(':lm margin=1.');
  writeln(':hp2.' + SDocErrors + ':ehp2.');
  writeln(':lm margin=3.');
  writeln('.br');
end;

procedure TIPFNewWriter.StartVersion;
begin
  writeln('');
  writeln(':p.');
  writeln(':lm margin=1.');
  writeln(':hp2.' + SDocVersion +':ehp2.');
  writeln(':lm margin=3.');
  writeln('.br');
end;

procedure TIPFNewWriter.StartAccess;
begin
  writeln('');
  writeln(':p.');
  writeln(':lm margin=1.');
  writeln(':hp2.' + SDocAccess + ':ehp2.');
  writeln(':lm margin=3.');
  writeln('.br');
end;

procedure TIPFNewWriter.StartProperty;
begin
  //writeln('');
  //Writeln('.* here I am');
  //writeln(':ul.');
end;

procedure TIPFNewWriter.EndProperty;
begin
  //writeln('');
  //writeln(':eul.');
end;

procedure TIPFNewWriter.WriteExampleFile(FN : String);
var
  sl: TStringList;
  i: integer;
begin
  if (FN<>'') then
  begin
    writeln('');
    writeln('');
    Writeln(':p.');
    writeln(':lm margin=1.');
    Writeln(':hp2.Example:ehp2.');
    writeln(':lm margin=3.');
    writeln('.br');
    writeln('Filename&colon. :hp1.' + EscapeText(FN) + ':ehp1.');
    writeln(':p.');

    writeln(':xmp.');
    //writeln(':im ' + FN);
    sl := TStringList.Create;
    try
      sl.LoadFromFile(FN);
      for i := 0 to sl.Count-1 do
        Writeln(EscapeText(sl[i]));
    finally
      sl.Free;
    end;
    writeln(':exmp.');
  end;
end;

procedure TIPFNewWriter.StartOverview(const What: String; WithAccess: Boolean);
begin
{
  If With access then it is a property overview.
  Otherwise it is a method/function overview.
  If tabular output is generated, the preferred output order is:
  With access:
  Col 1 : Page reference
  Col 2 : Property Name
  Col 3 : Accessibility (r/w)
  Col 4 : Description
  Without access:
  Col 1 : Page reference
  Col 2 : Method name
  Col 3 : Description
  (See the two WriteOverviewMember functions)
}
  writeln('');
  writeln(':parml tsize=30 break=none compact.');
//  FlushBuffer;
end;

procedure TIPFNewWriter.EndOverview;
begin
  { End of overview }
  writeln('');
  writeln(':eparml.');
  writeln(':p.');
//  FlushBuffer;
end;

procedure TIPFNewWriter.WriteOverviewMember(const ALabel,AName,Access,ADescr : String);
var
  s1, s2: string;
begin
  { Write one entry in property overview:
  ALabel : Label, as returned by GetLabel
  AName  : Property name
  Access : Property acces (r/w/a)
  Descr  : Description
  }
  s1 := StringReplace(ALabel, ':', '_', [rfReplaceAll]);
  s2 := StringReplace(AName, ':', '_', [rfReplaceAll]);
  WriteLn(Format(':pt. :link reftype=hd refid=%s.%s:elink. [%s]',[s1, s2, Access]));
  WriteLn(Format(':pd. %s', [ADescr]));
end;

procedure TIPFNewWriter.WriteOverviewMember(const ALabel,AName,ADescr : String);
var
  s1, s2: string;
begin
  { Write one entry in method overview:
  ALabel : Label, as returned by GetLabel
  AName  : Method name
  Descr  : Description
  }
  s1 := StringReplace(ALabel, ':', '_', [rfReplaceAll]);
  s2 := StringReplace(AName, ':', '_', [rfReplaceAll]);
  WriteLn(Format(':pt. :link reftype=hd refid=%s.%s :elink.',[s1, s2]));
  WriteLn(Format(':pd. %s', [ADescr]));
end;

procedure TIPFNewWriter.StartSeeAlso;
begin
  writeln('');
  writeln(':p.');
  writeln(':lm margin=1.');
  writeln(':hp2.See Also:ehp2.');
  writeln(':lm margin=3.');
  writeln('.br');
end;

procedure TIPFNewWriter.EndSeeAlso;
begin
  writeln('');
end;

procedure TIPFNewWriter.StartUnitOverview(AModuleName,AModuleLabel : String);
begin
  { Start of unit overview.
    AModuleName : Name of current unit.
    AModuleLabel : Label name of current unit.
  }
  writeln('');
  writeln(':p.');
  writeln(':lm margin=1.');
  DescrBeginBold;
  writeln(EscapeText(Format(SDocUsedUnitsByUnitXY, [AModuleName])));
  DescrEndBold;
  writeln(':lm margin=3.');
  writeln('.br');

  writeln(':p.');
  writeln(':ol.');
end;

procedure TIPFNewWriter.WriteUnitEntry(UnitRef : TPasType);
begin
  { Write one unit entry }
  writeln(':li.' + EscapeText(UnitRef.Name));
end;

procedure TIPFNewWriter.EndUnitOverview;
begin
  { end of unit overview }
  writeln(':eol.');
end;


initialization
  // Do not localize IPFWriterName
  RegisterWriter(TIPFNewWriter, IPFWriterName, SIPFUsageWriterDescr);
finalization
  UnRegisterWriter(IPFWriterName);
end.
