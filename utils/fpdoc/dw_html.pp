{
    FPDoc  -  Free Pascal Documentation Tool
    Copyright (C) 2000 - 2005 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org

    * HTML/XHTML output generator

    See the file COPYING, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
{$mode objfpc}
{$H+}

unit dw_html;
{$WARN 5024 off : Parameter "$1" not used}
interface

uses Classes, DOM, DOM_HTML, dGlobals, PasTree, dWriter, dw_basehtml;


type

  { THTMLWriter }

  THTMLWriter = class(TBaseHTMLWriter)
  private
    FHeadElement: TDomElement;
    FOnTest: TNotifyEvent;
    FCSSFile: String;
    FCharSet : String;
    FHeaderHTML,
    FNavigatorHTML,
    FFooterHTML: TStringStream;
    FTitleElement: TDOMElement;
    FIncludeDateInFooter : Boolean;
    FUseMenuBrackets: Boolean;
    FDateFormat: String;
    FIndexColCount : Integer;
    FSearchPage : String;
    procedure CreateMinusImage;
    procedure CreatePlusImage;
    procedure SetOnTest(const AValue: TNotifyEvent);
  protected
    function CreateAllocator : TFileAllocator; override;
    procedure WriteDocPage(const aFileName: String; aElement: TPasElement; aSubPageIndex: Integer);  override;

    procedure CreateCSSFile; virtual;

    procedure AppendTitle(const AText: String; Hints : TPasMemberHints = []); virtual;
    procedure AppendTitle(const AText: DOMString; Hints : TPasMemberHints = []); virtual;
    function AppendType(CodeEl, TableEl: TDOMElement;
      Element: TPasType; Expanded: Boolean;
      NestingLevel: Integer = 0): TDOMElement; virtual;
    function AppendProcType(CodeEl, TableEl: TDOMElement;  Element: TPasProcedureType; Indent: Integer): TDOMElement; virtual;
    procedure AppendProcExt(CodeEl: TDOMElement; Element: TPasProcedure); virtual;
    procedure AppendProcDecl(CodeEl, TableEl: TDOMElement; Element: TPasProcedureBase); virtual;
    procedure AppendProcArgsSection(Parent: TDOMNode; Element: TPasProcedureType; SkipResult : Boolean = False); virtual;
    function AppendRecordType(CodeEl, TableEl: TDOMElement; Element: TPasRecordType; NestingLevel: Integer): TDOMElement; virtual;
    procedure CreateMemberDeclarations(AParent: TPasElement; Members: TFPList; TableEl: TDOmelement; AddEnd: Boolean); virtual;

    procedure AppendMenuBar(ASubpageIndex: Integer);virtual;
    procedure AppendTopicMenuBar(Topic : TTopicElement);virtual;
    procedure FinishElementPage(AElement: TPasElement);virtual;
    procedure AppendFooter;virtual;

    procedure AppendClassMemberListLink(aClass: TPasClassType; ParaEl: TDomElement; AListSubpageIndex: Integer; const AText: DOMString);virtual;
    procedure CreateClassMainPage(aClass: TPasClassType);virtual;
    procedure CreateClassInheritanceSubpage(aClass: TPasClassType; AFilter: TMemberFilter);virtual;
    procedure CreateClassSortedSubpage(AClass: TPasClassType; AFilter: TMemberFilter);virtual;

    procedure CreateIndexPage(L : TStringList); virtual;
    procedure CreateModuleIndexPage(AModule: TPasModule); virtual;
    // Package
    procedure CreatePageBody(AElement: TPasElement; ASubpageIndex: Integer); virtual;
    procedure CreatePackagePageBody;virtual;
    procedure CreatePackageIndex;
    procedure CreatePackageClassHierarchy;
    procedure CreateClassHierarchyPage(AddUnit : Boolean);
    // Topic
    Procedure CreateTopicPageBody(AElement : TTopicElement);
    // Module
    procedure CreateModuleMainPage(aModule: TPasModule);virtual;
    procedure CreateModuleSimpleSubpage(aModule: TPasModule; ASubpageIndex: Integer; const ATitle: DOMString; AList: TFPList);virtual;
    procedure CreateModuleResStringsPage(aModule: TPasModule);virtual;
    procedure CreateModulePageBody(AModule: TPasModule; ASubpageIndex: Integer);
    // Identifiers
    procedure CreateConstPageBody(AConst: TPasConst);
    procedure CreateTypePageBody(AType: TPasType);
    procedure CreateClassPageBody(AClass: TPasClassType; ASubpageIndex: Integer);
    procedure CreateClassMemberPageBody(AElement: TPasElement);
    procedure CreateVarPageBody(AVar: TPasVariable);
    procedure CreateProcPageBody(AProc: TPasProcedureBase);
    Procedure CreateTopicLinks(Node : TDocNode; PasElement : TPasElement);
    procedure AppendTypeDecl(AType: TPasType; TableEl, CodeEl: TDomElement);
    //  Main documentation process
    Procedure DoWriteDocumentation; override;

    Property HeaderHTML : TStringStream Read FHeaderHTML;
    Property NavigatorHTML : TStringStream read FNavigatorHTML;
    Property FooterHTML : TStringStream read FFooterHTML;
    Property CSSFile : String Read FCSSFile;
    Property HeadElement : TDomElement Read FHeadElement;
    Property TitleElement: TDOMElement Read FTitleElement;
  public
    // Creating all module hierarchy classes happens here !
    constructor Create(APackage: TPasPackage; AEngine: TFPDocEngine); override;
    // Overrides
    Class Function FileNameExtension : String; override;
    class procedure Usage(List: TStrings); override;
    Class procedure SplitImport(var AFilename, ALinkPrefix: String); override;

    Function InterPretOption(Const Cmd,Arg : String) : boolean; override;

    // Single-page generation
    function CreateHTMLPage(AElement: TPasElement; ASubpageIndex: Integer): TXMLDocument; virtual;

    Property SearchPage: String Read FSearchPage Write FSearchPage;
    Property IncludeDateInFooter : Boolean Read FIncludeDateInFooter Write FIncludeDateInFooter;
    Property DateFormat : String Read FDateFormat Write FDateFormat;
    property OnTest: TNotifyEvent read FOnTest write SetOnTest;
    Property CharSet : String Read FCharSet Write FCharSet;
    Property IndexColCount : Integer Read FIndexColCount write FIndexColCount;
    Property UseMenuBrackets : Boolean Read FUseMenuBrackets write FUseMenuBrackets;
  end;


implementation

uses fpdocstrs, SysUtils, HTMWrite, fpdocclasstree;

{$i css.inc}
{$i plusimage.inc}
{$i minusimage.inc}

constructor THTMLWriter.Create(APackage: TPasPackage; AEngine: TFPDocEngine);

begin
  inherited Create(APackage, AEngine);
  // should default to true since this is the old behavior
  UseMenuBrackets:=True;
  IndexColCount:=3;
  Charset:='iso-8859-1';
end;

function THTMLWriter.CreateHTMLPage(AElement: TPasElement;
  ASubpageIndex: Integer): TXMLDocument;
var
  HTMLEl: THTMLHtmlElement;
  HeadEl: THTMLHeadElement;
  BodyElement : THTMLElement;
  El: TDOMElement;
begin
  Result := THTMLDocument.Create;
  SetHTMLDocument(THTMLDocument(Result));
  Doc.AppendChild(Doc.Impl.CreateDocumentType(
    'HTML', '-//W3C//DTD HTML 4.01 Transitional//EN',
    'http://www.w3.org/TR/html4/loose.dtd'));

  HTMLEl := Doc.CreateHtmlElement;
  Doc.AppendChild(HTMLEl);

  HeadEl := Doc.CreateHeadElement;
  FHeadElement:=HeadEl;
  HTMLEl.AppendChild(HeadEl);
  El := Doc.CreateElement('meta');
  HeadEl.AppendChild(El);
  El['http-equiv'] := 'Content-Type';
  
  El['content'] := 'text/html; charset=utf-8';
  FTitleElement := Doc.CreateElement('title');
  HeadEl.AppendChild(TitleElement);
  El := Doc.CreateElement('link');

  BodyElement := Doc.CreateElement('body');
  ContentElement:=BodyElement;
  HTMLEl.AppendChild(BodyElement);

  CreatePageBody(AElement, ASubpageIndex);

  AppendFooter;

  HeadEl.AppendChild(El);
  El['rel'] := 'stylesheet';
  El['type'] := 'text/css';
  El['href'] := UTF8Decode(FixHtmlPath(UTF8Encode(Allocator.GetCSSFilename(AElement))));
end;


procedure THTMLWriter.WriteDocPage(const aFileName: String; aElement: TPasElement; aSubPageIndex: Integer);

Var
  PageDoc: TXMLDocument;
  FN : String;
begin
  PageDoc := CreateHTMLPage(aElement, aSubpageIndex);
  try
    FN:=GetFileBaseDir(Engine.Output)+aFilename;
    //writeln('Element: ',Element.PathName, ' FileName: ', FN);
    WriteHTMLFile(PageDoc, FN);
  except
    on E: Exception do
      DoLog(SErrCouldNotCreateFile, [aFileName, e.Message]);
  end;
  PageDoc.Free;
end;

procedure THTMLWriter.DoWriteDocumentation;


begin
  Inherited;
  CreateCSSFile;
  CreatePlusImage;
  CreateMinusImage;
end;

procedure THTMLWriter.CreatePlusImage;
Var
  TempStream: TMemoryStream;

begin
  TempStream := TMemoryStream.Create;
  try
    DoLog('Creating plus image',[]);
    TempStream.WriteBuffer(PlusImageData,SizeOf(PlusImageData));
    TempStream.Position := 0;
    TempStream.SaveToFile(Engine.output+'plus.png');
  finally
    TempStream.Free;
  end;
end;

procedure THTMLWriter.CreateMinusImage;

Var
  TempStream: TMemoryStream;

begin
  TempStream := TMemoryStream.Create;
  try
    DoLog('Creating minus image',[]);
    TempStream.WriteBuffer(MinusImageData,SizeOf(MinusImageData));
    TempStream.Position := 0;
    TempStream.SaveToFile(Engine.output+'minus.png');
  finally
    TempStream.Free;
  end;
end;

procedure THTMLWriter.CreateCSSFile;

Var
  TempStream: TMemoryStream;

begin
  TempStream := TMemoryStream.Create;
  try
    if (FCSSFile<>'') then
      begin
      if not FileExists(FCSSFile) then
        begin
        DoLog('Can''t find CSS file "%s"',[FCSSFILE]);
        halt(1);
        end;
      TempStream.LoadFromFile(FCSSFile);
      end
    else
      begin
      DoLog('Using built-in CSS file',[]);
      TempStream.WriteBuffer(DefaultCSS,SizeOf(DefaultCSS));
      end;
   TempStream.Position := 0;
   TempStream.SaveToFile(Engine.output+'fpdoc.css');
  finally
    TempStream.Free;
  end;
end;


{ Returns the new CodeEl, which will be the old CodeEl in most cases }
function THTMLWriter.AppendType(CodeEl, TableEl: TDOMElement;  Element: TPasType; Expanded: Boolean; NestingLevel: Integer): TDOMElement;

Var
  S : String;

begin
  Result := CodeEl;

  if not Assigned(Element) then
    AppendText(CreateWarning(CodeEl), '<NIL>')
  else if (not Expanded) and (Length(Element.Name) > 0) then
    AppendHyperlink(CodeEl, Element)
  else
  // Array
  if Element.ClassType = TPasArrayType then
  begin
    S:='array ';
    If (TPasArrayType(Element).IndexRange<>'') then
      S:=S+'[' + TPasArrayType(Element).IndexRange + '] ';
    S:=S+'of ';
    If (TPasArrayType(Element).ElType=Nil) then
      S:=S+'Const';
    AppendPasSHFragment(CodeEl,S,0);
    If (TPasArrayType(Element).ElType<>Nil) then
      Result := AppendType(CodeEl, TableEl, TPasArrayType(Element).ElType, False);
  end else
  // Procedure or funtion type
  if Element.InheritsFrom(TPasProcedureType) then
  begin
    AppendKw(CodeEl, TPasProcedureType(Element).TypeName);
    Result := AppendProcType(CodeEl, TableEl, TPasProcedureType(Element), 0)
  end else
  // Range type
  if Element.InheritsFrom(TPasRangeType) then
    AppendPasSHFragment(CodeEl, TPasRangeType(Element).RangeStart + '..' +
      TPasRangeType(Element).RangeEnd, 0)
  // Record type
  else if Element.ClassType = TPasRecordType then
    Result := AppendRecordType(CodeEl, TableEl, TPasRecordType(Element), NestingLevel)
  else if (Element.ClassType = TPasFileType) and (TPasFileType(Element).elType=Nil) then
    AppendPasSHFragment(CodeEl,'file',0)
  else
  // Other types
    AppendHyperlink(CodeEl, Element);
end;

function THTMLWriter.AppendProcType(CodeEl, TableEl: TDOMElement;
  Element: TPasProcedureType; Indent: Integer): TDOMElement;

  function CreateIndentedCodeEl(Indent: Integer): TDOMElement;
  begin
    Result := CreateCode(CreatePara(CreateTD(CreateTR(TableEl))));
    AppendNbSp(Result, Indent);
  end;

var
  i: Integer;
  Arg: TPasArgument;
  S : String;

begin
  if Element.Args.Count > 0 then
  begin
    AppendSym(CodeEl, '(');

    for i := 0 to Element.Args.Count - 1 do
    begin
      Arg := TPasArgument(Element.Args[i]);
      CodeEl := CreateIndentedCodeEl(Indent + 2);
      S:=AccessNames[Arg.Access];
      if (S<>'') then
        AppendKw(CodeEl,S);
      AppendText(CodeEl, Arg.Name);
      if Assigned(Arg.ArgType) then
      begin
        AppendSym(CodeEl, ': ');
        CodeEl := AppendType(CodeEl, TableEl, Arg.ArgType, False);
      end;
      if Length(Arg.Value) > 0 then
        AppendPasSHFragment(CodeEl, ' = ' + Arg.Value, 0);
      if i < Element.Args.Count - 1 then
        AppendSym(CodeEl, ';');
    end;

    if Element.InheritsFrom(TPasFunctionType) or Element.IsOfObject then
    begin
      CodeEl := CreateIndentedCodeEl(Indent);
      if Element.InheritsFrom(TPasFunctionType) then
      begin
        AppendSym(CodeEl, '):');
        AppendHyperlink(CodeEl, TPasFunctionType(Element).ResultEl.ResultType);
      end else
        AppendSym(CodeEl, ')');
      if Element.IsOfObject then
      begin
        AppendText(CodeEl, ' ');        // Don't remove
        AppendKw(CodeEl, 'of object');
      end;
    end else
      if Indent > 0 then
        AppendSym(CodeEl, ')')
      else
      begin
        CodeEl := CreateCode(CreatePara(CreateTD(CreateTR(TableEl))));
        AppendSym(CodeEl, ')');
      end;
  end
  else
    begin
    { Procedure or function without arguments }
    if Element.InheritsFrom(TPasFunctionType) then
      begin
      AppendSym(CodeEl, ': ');
      AppendHyperlink(CodeEl, TPasFunctionType(Element).ResultEl.ResultType);
      end;
    if Element.IsOfObject then
      AppendKw(CodeEl, ' of object');
    end;
  Result := CodeEl;
end;

procedure THTMLWriter.AppendProcExt(CodeEl: TDOMElement;
  Element: TPasProcedure);

  procedure AppendExt(const Ext: String);
  begin
    AppendKw(CodeEl, ' ' + Ext);
    AppendSym(CodeEl, ';');
  end;

begin
  if Element.IsVirtual then
    AppendExt('virtual');
  if Element.IsDynamic then
    AppendExt('dynamic');
  if Element.IsAbstract then
    AppendExt('abstract');
  if Element.IsOverride then
    AppendExt('override');
  if Element.IsOverload then
    AppendExt('overload');
  if Element.IsMessage then
    AppendExt('message');
end;


{ Used in two places:
  - Page for the method of a class
  - Page for a tandalone procedure or function. }

procedure THTMLWriter.AppendProcDecl(CodeEl, TableEl: TDOMElement;
  Element: TPasProcedureBase);

  procedure WriteVariant(AProc: TPasProcedure; SkipResult : Boolean);
  begin
    AppendProcArgsSection(TableEl.ParentNode, AProc.ProcType, SkipResult);

    AppendKw(CodeEl, AProc.TypeName);
    if (Element.Parent.ClassType = TPasClassType) or (Element.Parent.ClassType = TPasRecordType) then
    begin
      AppendText(CodeEl, ' ');
      AppendHyperlink(CodeEl, Element.Parent);
      AppendSym(CodeEl, '.');
      AppendText(CodeEl, AProc.Name);
    end else
      if (Element is TPasOperator) then
        AppendText(CodeEl, ' ' + TPasOperator(AProc).GetOperatorDeclaration(True))
      else
        AppendText(CodeEl, ' ' + AProc.FullName);
    CodeEl := AppendProcType(CodeEl, TableEl, AProc.ProcType, 0);
    AppendSym(CodeEl, ';');
    AppendProcExt(CodeEl, AProc);
  end;

var
  i,fc: Integer;
  P : TPasProcedure;
begin
  fc:=0;
  if Element.ClassType = TPasOverloadedProc then
    for i := 0 to TPasOverloadedProc(Element).Overloads.Count - 1 do
    begin
      P:=TPasProcedure(TPasOverloadedProc(Element).Overloads[i]);
      if (P.ProcType is TPasFunctionType) then
        Inc(fc);
      if i > 0 then
      begin
        CreateEl(CodeEl, 'br');
        CreateEl(CodeEl, 'br');
      end;
      WriteVariant(P,fc>1);
    end
  else
    WriteVariant(TPasProcedure(Element),False);
end;

procedure THTMLWriter.AppendProcArgsSection(Parent: TDOMNode;
  Element: TPasProcedureType; SkipResult : Boolean = False);
var
  HasFullDescr, IsFirst: Boolean;
  ResultEl: TPasResultElement;
  ArgTableEl, TREl: TDOMElement;
  DocNode: TDocNode;
  i: Integer;
  Arg: TPasArgument;
begin
  IsFirst := True;
  for i := 0 to Element.Args.Count - 1 do
  begin
    Arg := TPasArgument(Element.Args[i]);
    if IsDescrNodeEmpty(Engine.FindShortDescr(Arg)) then
      continue;
    if IsFirst then
    begin
      IsFirst := False;
      AppendText(CreateH2(Parent), SDocArguments);
      ArgTableEl := CreateTable(Parent);
    end;
    TREl := CreateTR(ArgTableEl);
    AppendText(CreateCode(CreatePara(CreateTD_vtop(TREl))), Arg.Name);
    AppendShortDescrCell(TREl, Arg);
  end;

  if (Element.ClassType = TPasFunctionType) and not SkipResult then
  begin
    ResultEl := TPasFunctionType(Element).ResultEl;
    DocNode := Engine.FindDocNode(ResultEl);
    HasFullDescr := Assigned(DocNode) and not IsDescrNodeEmpty(DocNode.Descr);
    if HasFullDescr or
      (Assigned(DocNode) and not IsDescrNodeEmpty(DocNode.ShortDescr)) then
    begin
      AppendText(CreateH2(Parent), SDocFunctionResult);
      if HasFullDescr then
        AppendDescr(ResultEl, Parent, DocNode.Descr, True)
      else
        AppendDescr(ResultEl, CreatePara(Parent), DocNode.ShortDescr, False);
    end;
  end;
end;

function THTMLWriter.AppendRecordType(CodeEl, TableEl: TDOMElement;
  Element: TPasRecordType; NestingLevel: Integer): TDOMElement;
var
  i, j: Integer;
  Variable: TPasVariable;
  TREl: TDOMElement;
  CurVariant: TPasVariant;
  isExtended : Boolean;
  VariantEl: TPasElement;
  VariantType: TPasType;

begin
  if not (Element.Parent is TPasVariant) then
    if Element.IsPacked then
      If Element.IsBitPacked then
        AppendKw(CodeEl, 'bitpacked record')
      else
        AppendKW(CodeEl, 'packed record')
    else
      AppendKw(CodeEl, 'record');

  isExtended:=False;
  I:=0;
  while (not isExtended) and (I<Element.Members.Count) do
    begin
    isExtended:=Not (TObject(Element.Members[i]) is TPasVariable);
    Inc(i);
    end;
  if isExtended then
    CreateMemberDeclarations(Element,Element.Members,TableEl,False)
  else
    for i := 0 to Element.Members.Count - 1 do
      begin
      Variable := TPasVariable(Element.Members[i]);
      TREl := CreateTR(TableEl);
      CodeEl := CreateCode(CreatePara(CreateTD_vtop(TREl)));
      AppendShortDescrCell(TREl, Variable);
      AppendNbSp(CodeEl, NestingLevel * 2 + 2);
      AppendText(CodeEl, Variable.Name);
      AppendSym(CodeEl, ': ');
      CodeEl := AppendType(CodeEl, TableEl, Variable.VarType, False, NestingLevel + 1);
      AppendSym(CodeEl, ';');
    end;

  if Assigned(Element.VariantEl) then
  begin
    TREl := CreateTR(TableEl);
    CodeEl := CreateCode(CreatePara(CreateTD_vtop(TREl)));
    AppendNbSp(CodeEl, NestingLevel * 2 + 2);
    AppendKw(CodeEl, 'case ');
    VariantEl:=TPasRecordType(Element).VariantEl;
    if VariantEl is TPasVariable then
    begin
      AppendText(CodeEl, TPasVariable(VariantEl).Name);
      AppendSym(CodeEl, ': ');
      VariantType:=TPasVariable(VariantEl).VarType;
    end else
      VariantType:=VariantEl as TPasType;
    CodeEl := AppendType(CodeEl, TableEl, VariantType, True);
    AppendKw(CodeEl, ' of');
    for i := 0 to TPasRecordType(Element).Variants.Count - 1 do
    begin
      CurVariant := TPasVariant(Element.Variants[i]);
      TREl := CreateTR(TableEl);
      CodeEl := CreateCode(CreatePara(CreateTD_vtop(TREl)));
      AppendNbSp(CodeEl, NestingLevel * 2 + 4);
      for j := 0 to CurVariant.Values.Count - 1 do
      begin
        if j > 0 then
          AppendSym(CodeEl, ', ');
        AppendPasSHFragment(CodeEl, TPasElement(CurVariant.Values[j]).GetDeclaration(true), 0);
      end;
      AppendSym(CodeEl, ': (');
      AppendType(CodeEl, TableEl, CurVariant.Members, True, NestingLevel + 3);
      CodeEl := CreateCode(CreatePara(CreateTD_vtop(CreateTR(TableEl))));
      AppendNbSp(CodeEl, NestingLevel * 2 + 6);
      AppendSym(CodeEl, ');');
    end;
  end;

  if not (Element.Parent is TPasVariant) then
  begin
    CodeEl := CreateCode(CreatePara(CreateTD(CreateTR(TableEl))));
    AppendText(CodeEl, ' '); // !!!: Dirty trick, necessary for current XML writer
    AppendNbSp(CodeEl, NestingLevel * 2);
    AppendKw(CodeEl, 'end');
  end;
  Result := CodeEl;
end;


procedure THTMLWriter.AppendTopicMenuBar(Topic : TTopicElement);

var
  TableEl, TREl, ParaEl, TitleEl: TDOMElement;

  procedure AddLink(El : TPasElement; const AName: String);
  begin
    if FUseMenuBrackets then
      AppendText(ParaEl, '[');
    AppendText(CreateLink(ParaEl, ResolveLinkWithinPackage(El,0)),AName);
    if FUseMenuBrackets then
      AppendText(ParaEl, ']');
  end;

begin
  TableEl := CreateEl(ContentElement, 'table');
  TableEl['cellpadding'] := '4';
  TableEl['cellspacing'] := '0';
  TableEl['border'] := '0';
  TableEl['width'] := '100%';
  TableEl['class'] := 'bar';
  TREl := CreateTR(TableEl);
  ParaEl := CreateEl(CreateTD(TREl), 'b');
  If Assigned(Topic.Previous) then
    AddLink(Topic.Previous,SDocPrevious);
  If Assigned(Topic.Parent) then
    AddLink(Topic.Parent,SDocUp);
  if Assigned(Topic.Next) then
    AddLink(Topic.Next,SDocNext);
  if Length(SearchPage) > 0 then
    begin
    if FUseMenuBrackets then
      AppendText(ParaEl, '[');
    AppendText(CreateLink(ParaEl, SearchPage), SDocSearch);
    if FUseMenuBrackets then
      AppendText(ParaEl, ']');
    end;
  ParaEl := CreateTD(TREl);
  ParaEl['align'] := 'right';
  TitleEl := CreateEl(ParaEl, 'span');
  TitleEl['class'] := 'bartitle';
  if Assigned(Module) then
    AppendText(TitleEl, Format(SDocUnitTitle, [Module.Name]));
  if Assigned(Package) then
  begin
    AppendText(TitleEl, ' (');
    AppendHyperlink(TitleEl, Package);
    AppendText(TitleEl, ')');
  end;
end;


function THTMLWriter.CreateAllocator: TFileAllocator;
begin
   Result:=TLongNameFileAllocator.Create('.html');
end;

procedure THTMLWriter.AppendMenuBar(ASubpageIndex: Integer);

var
  TableEl, TREl, TRE2, ParaEl, TitleEl: TDOMElement;

  procedure AddLink(ALinkSubpageIndex: Integer; const AName: String);
  begin
    if FUseMenuBrackets then
      AppendText(ParaEl, '[');
    if ALinkSubpageIndex = ASubpageIndex then
      AppendText(ParaEl, AName)
    else
      AppendText(
        CreateLink(ParaEl, ResolveLinkWithinPackage(Module, ALinkSubpageIndex)),
        AName);
    if FUseMenuBrackets then
      AppendText(ParaEl, ']');
  end;

  procedure AddPackageLink(ALinkSubpageIndex: Integer; const AName: String);
  begin
    if FUseMenuBrackets then
      AppendText(ParaEl, '[');
    if ALinkSubpageIndex = ASubpageIndex then
      AppendText(ParaEl, AName)
    else
      AppendText(
        CreateLink(ParaEl, ResolveLinkWithinPackage(Package, ALinkSubpageIndex)),
        AName);
    if FUseMenuBrackets then
      AppendText(ParaEl, ']');
  end;



begin
  TableEl := CreateEl(ContentElement, 'table');
  TableEl['cellpadding'] := '4';
  TableEl['cellspacing'] := '0';
  TableEl['border'] := '0';
  TableEl['width'] := '100%';
  TableEl['class'] := 'bar';
  // Title Row
  TREl := CreateTR(TableEl);
  // Menu title
  ParaEl := CreateTD(TREl);
  ParaEl['align'] := 'left';
  TitleEl := CreateEl(ParaEl, 'span');
  TitleEl['class'] := 'bartitle';
  if Assigned(Module) then
    AppendText(TitleEl, Format(SDocUnitMenuTitle, [Module.Name]))
  else
    AppendText(TitleEl, Format(SDocPackageMenuTitle, [Package.Name]));

  // Package link title
  ParaEl := CreateTD(TREl);
  ParaEl['align'] := 'right';
  TitleEl := CreateEl(ParaEl, 'span');
  TitleEl['class'] := 'bartitle';
  if Assigned(Module) and Assigned(Package) then // Displays a Package page
  begin
    AppendText(TitleEl, SDocPackageLinkTitle);
  end;

  // Links Row
  TRE2 := CreateTR(TableEl);
  ParaEl := CreateTD(TRE2);
  ParaEl['align'] := 'left';
  ParaEl := CreateEl(ParaEl, 'span');
  ParaEl['class']:= 'bartitle';

  if Assigned(Module) then
    begin
    AddLink(0, SDocOverview);
    if Module.InterfaceSection.ResStrings.Count > 0 then
      AddLink(ResstrSubindex, SDocResStrings);
    if Module.InterfaceSection.Consts.Count > 0 then
      AddLink(ConstsSubindex, SDocConstants);
    if Module.InterfaceSection.Types.Count > 0 then
      AddLink(TypesSubindex, SDocTypes);
    if Module.InterfaceSection.Classes.Count > 0 then
      AddLink(ClassesSubindex, SDocClasses);
    if Module.InterfaceSection.Functions.Count > 0 then
      AddLink(ProcsSubindex, SDocProceduresAndFunctions);
    if Module.InterfaceSection.Variables.Count > 0 then
      AddLink(VarsSubindex, SDocVariables);
    AddLink(IndexSubIndex,SDocIdentifierIndex);
    AppendFragment(ParaEl, NavigatorHTML);
    end
  else
    begin
    // Overview
    AppendText(ParaEl, '[');
    AppendHyperlink(ParaEl, Package).TextContent:= UTF8Decode(SDocOverview);
    AppendText(ParaEl, ']');
    //Index
    AddPackageLink(IndexSubIndex, SDocIdentifierIndex);
    // Class TObject tree
    AddPackageLink(ClassHierarchySubIndex, SDocPackageClassHierarchy);
    AppendFragment(ParaEl, NavigatorHTML)
    end;

  if Length(SearchPage) > 0 then
  begin
    if FUseMenuBrackets then
      AppendText(ParaEl, '[');
    AppendText(CreateLink(ParaEl, SearchPage), SDocSearch);
    if FUseMenuBrackets then
      AppendText(ParaEl, ']');
  end;

  ParaEl := CreateTD(TRE2);
  ParaEl['align'] := 'right';
  ParaEl := CreateEl(ParaEl, 'span');
  ParaEl['class']:= 'bartitle';
  if Assigned(Module) and Assigned(Package) then // Displays a Package page
  begin
    AppendText(ParaEl, '[');
    AppendHyperlink(ParaEl, Package);
    AppendText(ParaEl, ']');
  end;
  AppendFragment(ContentElement,HeaderHTML);
end;


procedure THTMLWriter.AppendFooter;

Var
  S : String;
  F : TDomElement;
begin
  if Assigned(FooterHTML) then
    AppendFragment(ContentElement, FooterHTML)
  else if IncludeDateInFooter then
    begin
    CreateEl(ContentElement, 'hr');
    F:=CreateEl(ContentElement,'span');
    F['class']:='footer';
    If (FDateFormat='') then
      S:=DateToStr(Date)
    else
      S:=FormatDateTime(FDateFormat,Date);  
    AppendText(F,Format(SDocDateGenerated,[S]));
    end;
end;

procedure THTMLWriter.FinishElementPage(AElement: TPasElement);

var
  DocNode: TDocNode;

begin
  DocNode := Engine.FindDocNode(AElement);
  If Not Assigned(DocNode) then
    exit;

  // Description
  if Assigned(DocNode.Descr) then
    AppendDescrSection(AElement, ContentElement, DocNode.Descr, UTF8Encode(SDocDescription));

  // Append "Errors" section
  if Assigned(DocNode.ErrorsDoc) then
    AppendDescrSection(AElement, ContentElement, DocNode.ErrorsDoc, UTF8Encode(SDocErrors));

  // Append Version info
  if Assigned(DocNode.Version) then
    AppendDescrSection(AElement, ContentElement, DocNode.Version, UTF8Encode(SDocVersion));

  // Append "See also" section
  AppendSeeAlsoSection(AElement,DocNode);

  // Append examples, if present
  AppendExampleSection(AElement,DocNode);
  // Append notes, if present
  ConvertNotes(AElement,DocNode.Notes);
end;

procedure THTMLWriter.CreateTopicPageBody(AElement: TTopicElement);

var
  DocNode: TDocNode;

begin
  AppendTopicMenuBar(AElement);
  DocNode:=AElement.TopicNode;
  if Assigned(DocNode) then  // should always be true, but we're being careful.
    begin
    AppendShortDescr(AElement,TitleElement, DocNode);
    AppendShortDescr(AElement,CreateH2(ContentElement), DocNode);
    if Assigned(DocNode.Descr) then
       AppendDescrSection(AElement, ContentElement, DocNode.Descr, '');
    AppendSeeAlsoSection(AElement,DocNode);
    CreateTopicLinks(DocNode,AElement);
    AppendExampleSection(AElement,DocNode);
    ConvertNotes(AElement,DocNode.Notes);
    end;
end;

procedure THTMLWriter.CreateClassHierarchyPage(AddUnit : Boolean);

type
  TypeEN = (NPackage, NModule, NName);

  Procedure PushClassElement;
  Var
    H : THTMLElement;
  begin
    H:=CreateEl(CurOutputNode, 'li');
    H['class']:='classtree';
    PushOutputNode(H);
    H:=CreateEl(CurOutputNode, 'span');
    H['class']:='toggletreeclose';
    H['onclick']:='expandorcollapse(this)';
    PushOutputNode(h);
    AppendNbSp(h,1);
    PopOutputNode;
  end;

  Procedure PushClassList;
  Var
    H : THTMLElement;
  begin
    H:=CreateEl(CurOutputNode, 'ul');
    H['class']:='classtreelist';
    PushOutputNode(h);
  end;

  function ExtractName(APathName: String; Tp: TypeEN):String;
  var
  l:TStringList;
  begin
    Result:= Trim(APathName);
    if Result = '' then exit;
    l:=TStringList.Create;
    try
      l.AddDelimitedText(Result, '.', True);
      if l.Count=3 then
        Result:= l.Strings[Integer(Tp)]
      else
        Result:='';
    finally
      l.free;
    end;
  end;

  Procedure AppendClass(EN : TPasElementNode);

  Var
    PE,PM : TPasElement;
    I : Integer;

  begin
    if not Assigned(EN) then exit;
    PE:=EN.Element;
    PushClassElement;
    try
      if (PE<>Nil) then
        begin
        AppendHyperLink(CurOutputNode,PE);
        PM:=PE.GetModule();
        if (PM<>Nil) then
          begin
          AppendText(CurOutputNode,' (');
          AppendHyperLink(CurOutputNode,PM);
          AppendText(CurOutputNode,')');
          end
        end
      else
        AppendText(CurOutputNode,EN.Element.Name);
      if EN.ChildCount>0 then
        begin
        PushClassList;
        try
          For I:=0 to EN.ChildCount-1 do
            AppendClass(EN.Children[i] as TPasElementNode);
        finally
          PopOutputNode;
        end;
        end;
    Finally
      PopOutputNode;
    end;
  end;

begin
  PushOutputNode(ContentElement);
  try
    PushClassList;
    AppendClass(TreeClass.RootNode);
    //PopOutputNode;
  finally
    PopOutputNode;
  end;
end;

procedure THTMLWriter.CreatePackageClassHierarchy;

Const
  SFunc = 'function expandorcollapse (o) {'+sLineBreak+
          '  o.className = (o.className=="toggletreeclose") ? "toggletreeopen" : "toggletreeclose";'+sLineBreak+
          '  o.parentNode.className = (o.className=="toggletreeclose") ? "classtree" : "classtreeclosed";'+sLineBreak+
          '  return false;'+sLineBreak+
          '}';

Var
  S : String;
  SE : THTMLElement;

begin
  SE := Doc.CreateElement('script');
  AppendText(SE,SFunc);
  HeadElement.AppendChild(SE);
  AppendMenuBar(ClassHierarchySubIndex);
  S:=Package.Name;
  If Length(S)>0 then
    Delete(S,1,1);
  AppendTitle(UTF8Decode(Format(SDocPackageClassHierarchy, [S])));
  CreateClassHierarchyPage(True);
end;

procedure THTMLWriter.CreatePageBody(AElement: TPasElement; ASubpageIndex: Integer);

var
  i: Integer;
  Element: TPasElement;

begin
  CurDirectory := Allocator.GetFilename(AElement, ASubpageIndex);
  i := Length(CurDirectory);
  while (i > 0) and not (CurDirectory[i] in AllowDirectorySeparators) do
    Dec(i);
  CurDirectory := Copy(CurDirectory, 1, i);
  BaseDirectory := Allocator.GetRelativePathToTop(AElement);
  if AElement.ClassType = TPasPackage then
    begin
    Module:=Nil;
    If (ASubPageIndex=0) then
      CreatePackagePageBody
    else if ASubPageIndex=IndexSubIndex then
      CreatePackageIndex  
    else if ASubPageIndex=ClassHierarchySubIndex then
      CreatePackageClassHierarchy
    end
  else
    begin
    Element := AElement;
    while (Element<>Nil) and (not (Element.ClassType.inheritsfrom(TPasModule))) do
      Element := Element.Parent;
    Module := TPasModule(Element);

    if AElement.ClassType.inheritsfrom(TPasModule) then
      CreateModulePageBody(TPasModule(AElement), ASubpageIndex)
    else if AElement.Parent.InheritsFrom(TPasClassType) then
      CreateClassMemberPageBody(AElement)
    else if AElement.ClassType = TPasConst then
      CreateConstPageBody(TPasConst(AElement))
    else if AElement.InheritsFrom(TPasClassType) then
      CreateClassPageBody(TPasClassType(AElement), ASubpageIndex)
    else if AElement.InheritsFrom(TPasType) then
      CreateTypePageBody(TPasType(AElement))
    else if AElement.ClassType = TPasVariable then
      CreateVarPageBody(TPasVariable(AElement))
    else if AElement.InheritsFrom(TPasProcedureBase) then
      CreateProcPageBody(TPasProcedureBase(AElement))
    else if AElement.ClassType = TTopicELement then
      CreateTopicPageBody(TTopicElement(AElement))
    else if AElement.ClassType = TPasProperty then
      CreateClassMemberPageBody(TPasProperty(AElement))
    else
      writeln('Unknown classtype: ',AElement.classtype.classname);
  end;
end;

procedure THTMLWriter.CreateIndexPage(L : TStringList);
Var
  Lists  : Array['A'..'Z'] of TStringList;
  CL : TStringList;
  TableEl, TREl, EL: TDOMElement;
  E : TPasElement;
  I,Rows,J,Index : Integer;
  S : String;
  C : Char;

begin
  For C:='A' to 'Z' do
    Lists[C]:=Nil;
  L.Sort;
  Cl:=Nil;
  // Divide over alphabet
  For I:=0 to L.Count-1 do
    begin
    S:=L[i];
    E:=TPasElement(L.Objects[i]);
    If not (E is TPasUnresolvedTypeRef) then
      begin
      If (S<>'') then 
        begin
        C:=Upcase(S[1]);
        If C='_' then
          C:='A';
        If (C in ['A'..'Z']) and (Lists[C]=Nil) then
          begin
          CL:=TStringList.Create;
          Lists[C]:=CL;
          end;
        end;
      if assigned(cl) then  
        CL.AddObject(S,E);
      end;  
    end;  
  Try  
  // Create a quick jump table to all available letters.    
  TableEl := CreateTable(ContentElement);
  TableEl['border']:='1';
  TableEl['width']:='50%';
  TREl := CreateTR(TableEl);
  for C:='A' to 'Z' do
    If (Lists[C]<>Nil) then
      begin
      El:=CreateTD_vtop(TREl);
      AppendText(CreateLink(El,UTF8Decode('#SECTION'+C)),UTF8Decode(C));
      If C<>'Z' then
       AppendNBsp(El,1);
      end;
  // Now emit all identifiers.    
  TableEl:=Nil;
  For C:='A' to 'Z' do
    begin
    CL:=Lists[C];
    If CL<>Nil then
      begin
      El:=CreateH2(ContentElement);
      AppendText(El,UTF8Decode(C));
      CreateAnchor(El,UTF8Decode('SECTION'+C));
      TableEl := CreateTable(ContentElement);
      TableEl['Width']:='80%';
      // Determine number of rows needed
      Rows:=(CL.Count div IndexColCount);
      If ((CL.Count Mod IndexColCount)<>0) then
        Inc(Rows);
      // Fill rows  
      For I:=0 to Rows-1 do
        begin
        TREl := CreateTR(TableEl);
        For J:=0 to IndexColCount-1 do 
          begin
          El:=CreateTD_vtop(TREl);
          Index:=(J*Rows)+I;
          If (Index<CL.Count) then
            begin
            S:=CL[Index];
            E:=TPasElement(CL.Objects[Index]);
            AppendHyperlink(El,E);
            end;
          end;  
        end;  
      end; // have List
    end;  // For C:=
  Finally
    for C:='A' to 'Z' do
      FreeAndNil(Lists[C]);
  end;  
end;

procedure THTMLWriter.CreatePackageIndex;

Var
  L : TStringList;
  I : Integer;
  M : TPasModule;
  S : String;
  
begin
  L:=TStringList.Create;
  try
    L.Capacity:=PageInfos.Count; // Too much, but that doesn't hurt.
    For I:=0 to Package.Modules.Count-1 do
      begin
      M:=TPasModule(Package.Modules[i]);
      L.AddObject(M.Name,M);
      AddModuleIdentifiers(M,L);
      end;
    AppendMenuBar(IndexSubIndex);
    S:=Package.Name;
    If Length(S)>0 then
      Delete(S,1,1);
    AppendTitle(UTF8Decode(Format(SDocPackageIndex, [S])));
    CreateIndexPage(L);
  Finally
    L.Free;
  end;
end;

procedure THTMLWriter.CreatePackagePageBody;

var
  DocNode: TDocNode;
  TableEl, TREl: TDOMElement;
  i: Integer;
  ThisModule: TPasModule;
  L : TStringList;

begin
  AppendMenuBar(0);
  AppendTitle(UTF8Encode(Format(SDocPackageTitle, [Copy(Package.Name, 2, 256)])));
  AppendShortDescr(CreatePara(ContentElement), Package);

  AppendText(CreateH2(ContentElement), UTF8Encode(SDocUnits));
  TableEl := CreateTable(ContentElement);
  L:=TStringList.Create;
  Try
    L.Sorted:=True;
    // Sort modules.
    For I:=0 to Package.Modules.Count-1 do
      L.AddObject(TPasModule(Package.Modules[i]).Name,TPasModule(Package.Modules[i]));
    // Now create table.
    for i:=0 to L.Count - 1 do
      begin
      ThisModule := TPasModule(L.Objects[i]);
      TREl := CreateTR(TableEl);
      AppendHyperlink(CreateCode(CreatePara(CreateTD_vtop(TREl))), ThisModule);
      AppendShortDescrCell(TREl, ThisModule);
      end;
  Finally
    L.Free;
  end;

  DocNode := Engine.FindDocNode(Package);
  if Assigned(DocNode) then
    begin
    if Assigned(DocNode.Descr) then
       AppendDescrSection(nil, ContentElement, DocNode.Descr, UTF8Decode(SDocDescription));
    CreateTopicLinks(DocNode,Package);
    end;
end;

procedure THTMLWriter.CreateTopicLinks (Node: TDocNode; PasElement: TPasElement) ;

var
  DocNode: TDocNode;
  TableEl, TREl: TDOMElement;
  First : Boolean;
  ThisTopic: TPasElement;

begin
  DocNode:=Node.FirstChild;
  First:=True;
  While Assigned(DocNode) do
    begin
    If DocNode.TopicNode then
      begin
      if first then
        begin
        First:=False;
        AppendText(CreateH2(ContentElement), UTF8Decode(SDocRelatedTopics));
        TableEl := CreateTable(ContentElement);
        end;
      TREl := CreateTR(TableEl);
      ThisTopic:=FindTopicElement(DocNode);
      if Assigned(ThisTopic) then
        AppendHyperlink(CreateCode(CreatePara(CreateTD_vtop(TREl))), ThisTopic);
      AppendShortDescrCell(TREl, ThisTopic);
      end;
    DocNode:=DocNode.NextSibling;
    end;
end;

procedure THTMLWriter.CreateModuleIndexPage(AModule: TPasModule);

Var
  L : TStringList;

begin
  L:=TStringList.Create;
  try
    AddModuleIdentifiers(AModule,L);
    AppendMenuBar(IndexSubIndex);
    AppendTitle(UTF8Decode(Format(SDocModuleIndex, [AModule.Name])));
    CreateIndexPage(L);
  Finally
    L.Free;
  end;  
end;

procedure THTMLWriter.CreateModuleMainPage(aModule : TPasModule);

var
  TableEl, TREl, TDEl, CodeEl: TDOMElement;
  i: Integer;
  UnitRef: TPasType;
  DocNode: TDocNode;

begin
  AppendMenuBar(0);
  AppendTitle(UTF8Decode(Format(SDocUnitTitle, [AModule.Name])),AModule.Hints);
  AppendShortDescr(CreatePara(ContentElement), AModule);

  if AModule.InterfaceSection.UsesList.Count > 0 then
  begin
    TableEl := CreateTable(ContentElement);
    AppendKw(CreateCode(CreatePara(CreateTD(CreateTR(TableEl)))), 'uses');
    for i := 0 to AModule.InterfaceSection.UsesList.Count - 1 do
    begin
      UnitRef := TPasType(AModule.InterfaceSection.UsesList[i]);
      DocNode := Engine.FindDocNode(UnitRef);
      if Assigned(DocNode) and DocNode.IsSkipped then
        continue;
      TREl := CreateTR(TableEl);
      TDEl := CreateTD_vtop(TREl);
      CodeEl := CreateCode(CreatePara(TDEl));
      AppendNbSp(CodeEl, 2);
      AppendHyperlink(CodeEl, UnitRef);
      if i < AModule.InterfaceSection.UsesList.Count - 1 then
        AppendSym(CodeEl, ',')
      else
        AppendSym(CodeEl, ';');
      AppendText(CodeEl, '  ');               // Space for descriptions
      AppendShortDescrCell(TREl, UnitRef);
    end;
  end;

  DocNode := Engine.FindDocNode(AModule);
  if Assigned(DocNode) then
    begin
    if Assigned(DocNode.Descr) then
      AppendDescrSection(AModule, ContentElement, DocNode.Descr, UTF8Decode(SDocOverview));
    ConvertNotes(AModule,DocNode.Notes);
    CreateTopicLinks(DocNode,AModule);
    end;
end;


procedure THTMLWriter.CreateModuleSimpleSubpage(aModule: TPasModule; ASubpageIndex: Integer; const ATitle: DOMString; AList: TFPList);

var
  TableEl, TREl, CodeEl: TDOMElement;
  i, j: Integer;
  Decl: TPasElement;
  SortedList: TFPList;
  DocNode: TDocNode;
  S : String;

begin
  AppendMenuBar(ASubpageIndex);
  S:=UTF8Encode(ATitle);
  AppendTitle(UTF8Decode(Format(SDocUnitTitle + ': %s', [AModule.Name, S])));
  SortedList := TFPList.Create;
  try
    for i := 0 to AList.Count - 1 do
    begin
      Decl := TPasElement(AList[i]);
      DocNode := Engine.FindDocNode(Decl);
      if (not Assigned(DocNode)) or (not DocNode.IsSkipped) then
      begin
        j := 0;
        while (j < SortedList.Count) and (CompareText(
          TPasElement(SortedList[j]).PathName, Decl.PathName) < 0) do
          Inc(j);
        SortedList.Insert(j, Decl);
      end;
    end;

    TableEl := CreateTable(ContentElement);
    for i := 0 to SortedList.Count - 1 do
    begin
      Decl := TPasElement(SortedList[i]);
      TREl := CreateTR(TableEl);
      CodeEl := CreateCode(CreatePara(CreateTD_vtop(TREl)));
      AppendHyperlink(CodeEl, Decl);
      AppendShortDescrCell(TREl, Decl);
    end;
  finally
    SortedList.Free;
  end;
end;

procedure THTMLWriter.CreateModuleResStringsPage(aModule : TPasModule);
var
  ParaEl: TDOMElement;
  i: Integer;
  Decl: TPasResString;
begin
  AppendMenuBar(ResstrSubindex);
  AppendTitle(UTF8Decode(Format(SDocUnitTitle + ': %s', [AModule.Name, SDocResStrings])));
  for i := 0 to AModule.InterfaceSection.ResStrings.Count - 1 do
  begin
    Decl := TPasResString(AModule.InterfaceSection.ResStrings[i]);
    CreateEl(ContentElement, 'a')['name'] := UTF8Decode(LowerCase(Decl.Name));
    ParaEl := CreatePara(ContentElement);
    AppendText(CreateCode(ParaEl), UTF8Decode(Decl.Name));
    CreateEl(ParaEl, 'br');
    AppendText(ParaEl, UTF8Decode(Decl.Expr.getDeclaration(true)));
  end;
end;


procedure THTMLWriter.CreateModulePageBody(AModule: TPasModule;
  ASubpageIndex: Integer);

begin
  case ASubpageIndex of
    0:
      CreateModuleMainPage(aModule);
    ResstrSubindex:
      CreateModuleResStringsPage(aModule);
    ConstsSubindex:
      CreateModuleSimpleSubpage(aModule, ConstsSubindex,UTF8Decode(SDocConstants), AModule.InterfaceSection.Consts);
    TypesSubindex:
      CreateModuleSimpleSubpage(aModule, TypesSubindex,UTF8Decode(SDocTypes), AModule.InterfaceSection.Types);
    ClassesSubindex:
      CreateModuleSimpleSubpage(aModule, ClassesSubindex,UTF8Decode(SDocClasses), AModule.InterfaceSection.Classes);
    ProcsSubindex:
      CreateModuleSimpleSubpage(aModule, ProcsSubindex, UTF8Decode(SDocProceduresAndFunctions), AModule.InterfaceSection.Functions);
    VarsSubindex:
      CreateModuleSimpleSubpage(aModule, VarsSubindex,UTF8Decode(SDocVariables), AModule.InterfaceSection.Variables);
    IndexSubIndex: 
      CreateModuleIndexPage(AModule);
  end;
end;

procedure THTMLWriter.CreateConstPageBody(AConst: TPasConst);
var
  TableEl, CodeEl: TDOMElement;
begin
  AppendMenuBar(-1);
  AppendTitle(UTF8Decode(AConst.Name),AConst.Hints);
  AppendShortDescr(CreatePara(ContentElement), AConst);
  AppendText(CreateH2(ContentElement), UTF8Decode(SDocDeclaration));
  AppendSourceRef(ContentElement,AConst);

  TableEl := CreateTable(ContentElement);
  CodeEl := CreateCode(CreatePara(CreateTD(CreateTR(TableEl))));

  AppendKw(CodeEl, 'const');
  AppendText(CodeEl, ' ' + UTF8Decode(AConst.Name));
  if Assigned(AConst.VarType) then
  begin
    AppendSym(CodeEl, ': ');
    AppendType(CodeEl, TableEl, AConst.VarType, False);
  end;
  AppendPasSHFragment(CodeEl, ' = ' + AConst.Expr.GetDeclaration(True) + ';', 0);

  FinishElementPage(AConst);
end;

procedure THTMLWriter.AppendTypeDecl(AType: TPasType; TableEl,CodeEl : TDomElement);

Var
  TREl : TDomElement;
  i: Integer;
  s: String;
  EnumType: TPasEnumType;
  EnumValue: TPasEnumValue;
begin
  // Alias
  if AType.ClassType = TPasAliasType then
    begin
    if Assigned(TPasAliasType(AType).DestType) then
      AppendHyperlink(CodeEl, TPasAliasType(AType).DestType)
    else
      AppendText(CreateWarning(CodeEl), '<Destination type is NIL>');
    AppendSym(CodeEl, ';');
  end else
  // Class of
  if AType.ClassType = TPasClassOfType then
  begin
    AppendKw(CodeEl, 'class of ');
    AppendHyperlink(CodeEl, TPasClassOfType(AType).DestType);
    AppendSym(CodeEl, ';');
  end else
  // Enumeration
  if AType.ClassType = TPasEnumType then
  begin
    AppendSym(CodeEl, '(');
    for i := 0 to TPasEnumType(AType).Values.Count - 1 do
    begin
      EnumValue := TPasEnumValue(TPasEnumType(AType).Values[i]);
      TREl := CreateTR(TableEl);
      CodeEl := CreateCode(CreatePara(CreateTD_vtop(TREl)));
      AppendShortDescrCell(TREl, EnumValue);
      AppendNbSp(CodeEl, 2);
      s := EnumValue.Name;
      if EnumValue.AssignedValue<>'' then
        s := s + ' = ' + EnumValue.AssignedValue;
      if i < TPasEnumType(AType).Values.Count - 1 then
        s := s + ',';
      AppendPasSHFragment(CodeEl, s, 0);
    end;
    AppendSym(CreateCode(CreatePara(CreateTD(CreateTR(TableEl)))), ');');
  end else
  // Pointer type
  if AType.ClassType = TPasPointerType then
  begin
    AppendSym(CodeEl, '^');
    if Assigned(TPasPointerType(AType).DestType) then
      AppendHyperlink(CodeEl, TPasPointerType(AType).DestType)
    else
      AppendText(CreateWarning(CodeEl), '<Destination type is NIL>');
    AppendSym(CodeEl, ';');
  end else
  if AType.InheritsFrom(TPasProcedureType) then
  begin
    AppendSym(AppendType(CodeEl, TableEl, TPasType(AType), True), ';');
    AppendProcArgsSection(ContentElement, TPasProcedureType(AType));
  end else
  // Record
  if AType.ClassType = TPasRecordType then
  begin
    CodeEl := AppendRecordType(CodeEl, TableEl, TPasRecordType(AType), 0);
    AppendSym(CodeEl, ';');
  end else
  // Set
  if AType.ClassType = TPasSetType then
  begin
    AppendKw(CodeEl, 'set of ');
    if TPasSetType(AType).EnumType.ClassType = TPasEnumType then
    begin
      AppendSym(CodeEl, '(');
      EnumType := TPasEnumType(TPasSetType(AType).EnumType);
      for i := 0 to EnumType.Values.Count - 1 do
      begin
        EnumValue := TPasEnumValue(EnumType.Values[i]);
        TREl := CreateTR(TableEl);
        CodeEl := CreateCode(CreatePara(CreateTD_vtop(TREl)));
        AppendShortDescrCell(TREl, EnumValue);
        AppendNbSp(CodeEl, 2);
        s := EnumValue.Name;
        if (EnumValue.AssignedValue<>'') then
          s := s + ' = ' + EnumValue.AssignedValue;
        if i < EnumType.Values.Count - 1 then
          s := s + ',';
        AppendPasSHFragment(CodeEl, s, 0);
      end;
      AppendSym(CreateCode(CreatePara(CreateTD(CreateTR(TableEl)))), ');');
    end else
    begin
      AppendHyperlink(CodeEl, TPasSetType(AType).EnumType);
      AppendSym(CodeEl, ';');
    end;
  end else
  // Type alias
  if AType.ClassType = TPasTypeAliasType then
  begin
    AppendKw(CodeEl, 'type ');
    AppendHyperlink(CodeEl, TPasTypeAliasType(AType).DestType);
    AppendSym(CodeEl, ';');
  end else
  // Probably one of the simple types, which allowed in other places as wel...
    AppendSym(AppendType(CodeEl, TableEl, TPasType(AType), True), ';');
end;

procedure THTMLWriter.CreateTypePageBody(AType: TPasType);
var
  TableEl, TREl, TDEl, CodeEl: TDOMElement;
  DocNode: TDocNode;
begin
  AppendMenuBar(-1);
  AppendTitle(UTF8Decode(AType.Name),AType.Hints);
  AppendShortDescr(CreatePara(ContentElement), AType);
  AppendText(CreateH2(ContentElement), UTF8Decode(SDocDeclaration));
  AppendSourceRef(ContentElement,AType);

  TableEl := CreateTable(ContentElement);
  TREl := CreateTR(TableEl);
  TDEl := CreateTD(TREl);
  CodeEl := CreateCode(CreatePara(TDEl));

  DocNode := Engine.FindDocNode(AType);
  AppendKw(CodeEl, 'type ');
  AppendText(CodeEl, UTF8Decode(AType.Name));
  AppendSym(CodeEl, ' = ');

  If Assigned(DocNode) and
     Assigned(DocNode.Node) and
     (Docnode.Node['opaque']='1') then
    AppendText(CodeEl,UTF8Decode(SDocOpaque))
  else
    begin
    AppendTypeDecl(AType,TableEl,CodeEl);
    end;
  FinishElementPage(AType);
end;



procedure THTMLWriter.CreateMemberDeclarations(AParent : TPasElement; Members : TFPList; TableEl : TDOmelement; AddEnd : Boolean);

var
  TREl, CodeEl: TDOMElement;
  Member: TPasElement;
  MVisibility,
  CurVisibility: TPasMemberVisibility;
  i: Integer;
  s: String;
  t : TPasType;
  ah,ol,wt,ct,wc,cc : boolean;
  isRecord : Boolean;

begin
  isRecord:=AParent is TPasRecordType;
  CodeEl:=nil;
  if Members.Count > 0 then
    begin
    wt:=False;
    wc:=False;
    CurVisibility := visDefault;
    for i := 0 to Members.Count - 1 do
      begin
      Member := TPasElement(Members[i]);
      MVisibility:=Member.Visibility;
      cc:=(Member is TPasConst);
      ct:=(Member is TPasType);
      ol:=(Member is TPasOverloadedProc);
      ah:=ol or ((Member is TPasProcedure) and (TPasProcedure(Member).ProcType.Args.Count > 0));
      if ol then
        Member:=TPasElement((Member as TPasOverloadedProc).Overloads[0]);
      if Not Engine.ShowElement(Member) then
        continue;
      if (CurVisibility <> MVisibility) or (cc <> wc) or (ct <> wt) then
        begin
        CurVisibility := MVisibility;
        wc:=cc;
        wt:=ct;
        s:=VisibilityNames[MVisibility];
        AppendKw(CreateCode(CreatePara(CreateTD(CreateTR(TableEl)))), UTF8Decode(s));
        if (ct) then AppendKw(CreateCode(CreatePara(CreateTD(CreateTR(TableEl)))), 'type');
        if (cc) then AppendKw(CreateCode(CreatePara(CreateTD(CreateTR(TableEl)))), 'const');
        end;
      TREl := CreateTR(TableEl);
      CodeEl := CreateCode(CreatePara(CreateTD_vtop(TREl)));
      AppendNbSp(CodeEl, 2);
      AppendShortDescrCell(TREl, Member);

      if (Member is TPasProcedureBase) then
        begin
        AppendKw(CodeEl, UTF8Decode(TPasProcedureBase(Member).TypeName) + ' ');
        AppendHyperlink(CodeEl, Member);
        if ah then
          AppendSym(CodeEl, '();')
        else
          AppendSym(CodeEl, ';');
        if Not OL then
          AppendProcExt(CodeEl, TPasProcedure(Member));
        end
      else if (Member is TPasConst) then
        begin
        AppendHyperlink(CodeEl, Member);
        If Assigned(TPasConst(Member).VarType) then
          begin
          AppendSym(CodeEl, ' = ');
          AppendTypeDecl(TPasType(TPasConst(Member).VarType),TableEl,CodeEl);
          end;
        AppendSym(CodeEl, ' = ');
        AppendText(CodeEl,UTF8Decode(TPasConst(Member).Expr.GetDeclaration(True)));
        end
      else if (Member is TPasType) then
        begin
        AppendHyperlink(CodeEl, Member);
        AppendSym(CodeEl, ' = ');
        AppendTypeDecl(TPasType(Member),TableEl,CodeEl);
        end
      else if (Member is TPasProperty) then
        begin
        AppendKw(CodeEl, 'property ');
        AppendHyperlink(CodeEl, Member);
        t:=TPasProperty(Member).ResolvedType;
        if Assigned(TPasProperty(Member).Args) and (TPasProperty(Member).Args.Count>0) then
           AppendText(CodeEl, ' []');
        if Assigned(T) then
        begin
          AppendSym(CodeEl, ': ');
          AppendHyperlink(CodeEl, T);
        end;
        AppendSym(CodeEl, ';');
        if TPasProperty(Member).IsDefault then
        begin
          AppendKw(CodeEl, ' default');
          AppendSym(CodeEl, ';');
        end;
        if (TPasProperty(Member).ImplementsName<>'') then
        begin
          AppendKw(CodeEl, ' implements');
          AppendText(CodeEl, ' '+UTF8Decode(TPasProperty(Member).ImplementsName));
          AppendSym(CodeEl, ';');
        end;
        SetLength(s, 0);
        if Length(TPasProperty(Member).ReadAccessorName) > 0 then
          s := s + 'r';
        if Length(TPasProperty(Member).WriteAccessorName) > 0 then
          s := s + 'w';
        if Length(TPasProperty(Member).StoredAccessorName) > 0 then
          s := s + 's';
        if Length(s) > 0 then
          AppendText(CodeEl, '  [' + UTF8Decode(s) + ']');
        end
      else if (Member is TPasVariable) then
        begin
        if not isRecord then
          AppendHyperlink(CodeEl, Member)
        else
          AppendText(CodeEl, UTF8Decode(Member.Name));
        AppendSym(CodeEl, ': ');
        AppendType(CodeEl, TableEl, TPasVariable(Member).VarType,False);
        AppendSym(CodeEl, ';');
        end
      else
        AppendText(CreateWarning(CodeEl), '<' + UTF8Decode(Member.ClassName) + '>');
      if (Member.Hints<>[]) then
        begin
        AppendKW(CodeEl,' '+UTF8Decode(Engine.HintsToStr(Member.Hints)));
        AppendText(CodeEl, ' ');
        AppendSym(CodeEl, ';');
        end;
    end;
    CodeEl := CreateCode(CreatePara(CreateTD(CreateTR(TableEl))));
  end;
  if assigned(CodeEl) Then
     begin
        AppendText(CodeEl, ' '); // !!!: Dirty trick, necessary for current XML writer
        If AddEnd then
          begin
          AppendKw(CodeEl, 'end');
          AppendSym(CodeEl, ';');
          end;
     end;
end;

procedure THTMLWriter.AppendTitle(const AText: String; Hints: TPasMemberHints);
begin
  AppendTitle(UTF8Decode(aText),Hints);
end;

procedure THTMLWriter.AppendTitle(const AText: DOMString; Hints : TPasMemberHints = []);

Var
  T : UnicodeString;
begin
  T:=AText;
  if (Hints<>[]) then
    T:=T+' ('+UTF8Decode(Engine.HintsToStr(Hints))+')';
  AppendText(TitleElement, AText);
  AppendText(CreateH1(ContentElement), T);
end;


procedure THTMLWriter.AppendClassMemberListLink(aClass : TPasClassType; ParaEl : TDomElement; AListSubpageIndex: Integer;  const AText: DOMString);

var
  LinkEl: TDOMElement;
begin
  if FUseMenuBrackets then
    AppendText(ParaEl, '[');
  LinkEl := CreateEl(ParaEl, 'a');
  LinkEl['href'] :=UTF8Decode(FixHtmlPath(ResolveLinkWithinPackage(AClass, AListSubpageIndex)));
  LinkEl['onClick'] := 'window.open(''' + LinkEl['href'] + ''', ''list'', ' +
   '''dependent=yes,resizable=yes,scrollbars=yes,height=400,width=300''); return false;';
  AppendText(LinkEl, AText);
  AppendText(ParaEl, ' (');
  LinkEl := CreateEl(ParaEl, 'a');
  LinkEl['href'] :=UTF8Decode(FixHtmlPath(ResolveLinkWithinPackage(AClass, AListSubpageIndex + 1)));
  LinkEl['onClick'] := 'window.open(''' + LinkEl['href'] + ''', ''list'', ' +
   '''dependent=yes,resizable=yes,scrollbars=yes,height=400,width=300''); return false;';
  AppendText(LinkEl, UTF8Decode(SDocByName));
  AppendText(ParaEl, ')');
  if FUseMenuBrackets then
    AppendText(ParaEl, '] ')
  else
    AppendText(ParaEl, ' ');
end;


procedure THTMLWriter.CreateClassMainPage(aClass : TPasClassType);

  procedure AppendGenericTypes(CodeEl : TDomElement; AList : TFPList; isSpecialize : Boolean);
  Var
    I : integer;
  begin
    for I:=0 to AList.Count-1 do
      begin
      if I=0 then
        AppendSym(CodeEl, '<')
      else
        AppendSym(CodeEl, ',');
      AppendText(CodeEl,UTF8Decode(TPasGenericTemplateType(AList[i]).Name));
      end;
    AppendSym(CodeEl, '>');
  end;

  procedure AppendGeneric(ACodeEl : TDomElement ; AGenericObject: TPasClassType);
  begin
    if AGenericObject.GenericTemplateTypes.Count>0 then
    begin
      AppendKw(ACodeEl, ' generic ');
      AppendText(ACodeEl, ' ' + UTF8Decode(AGenericObject.Name) + ' ');
      AppendGenericTypes(ACodeEl,AGenericObject.GenericTemplateTypes,false);
    end;
  end;

  procedure AppendInterfaceInfo(ACodeEl : TDomElement ; AThisClass: TPasClassType);
  var
    i:Integer;
    ThisInterface:TPasClassType;
  begin
  if Assigned(AThisClass) and (AThisClass.Interfaces.count>0) then
    begin
      for i:=0 to AThisClass.interfaces.count-1 do
        begin
          ThisInterface:=TPasClassType(AThisClass.Interfaces[i]);
          AppendText(ACodeEl,',');
          AppendHyperlink(ACodeEl, ThisInterface);
        end;
    end;
  end;

var
  ParaEl,TableEl, TREl, TDEl, CodeEl: TDOMElement;
  ThisClass, PrevClass: TPasType;
  ThisTreeNode: TPasElementNode;
begin
  //WriteLn('@ClassPageBody.CreateMainPage Class=', AClass.Name);
  AppendMenuBar(-1);
  AppendTitle(UTF8Decode(AClass.Name),AClass.Hints);

  ParaEl := CreatePara(ContentElement);
  AppendClassMemberListLink(aClass,ParaEl,PropertiesByInheritanceSubindex, UTF8Decode(SDocProperties));
  AppendClassMemberListLink(aClass,ParaEl,MethodsByInheritanceSubindex, UTF8Decode(SDocMethods));
  AppendClassMemberListLink(aClass,ParaEl,EventsByInheritanceSubindex, UTF8Decode(SDocEvents));

  AppendShortDescr(CreatePara(ContentElement), AClass);
  AppendText(CreateH2(ContentElement), UTF8Decode(SDocDeclaration));
  AppendSourceRef(ContentElement,AClass);

  TableEl := CreateTable(ContentElement);

  TREl := CreateTR(TableEl);
  TDEl := CreateTD(TREl);
  CodeEl := CreateCode(CreatePara(TDEl));
  AppendKw(CodeEl, 'type');

  if not Assigned(AClass.GenericTemplateTypes) then
      Dolog('ERROR generic init: %s', [AClass.name]);
  if AClass.GenericTemplateTypes.Count>0 then
    AppendGeneric(CodeEl, AClass)
  else
    AppendText(CodeEl, ' ' + UTF8Decode(AClass.Name) + ' ');

  AppendSym(CodeEl, '=');
  AppendText(CodeEl, ' ');
  AppendKw(CodeEl, UTF8Decode(ObjKindNames[AClass.ObjKind]));

  // Now we are using only TreeClass for show inheritance

  ThisClass := AClass; ThisTreeNode := Nil;
  if AClass.ObjKind = okInterface then
    ThisTreeNode := TreeInterface.GetPasElNode(AClass)
  else
    ThisTreeNode := TreeClass.GetPasElNode(AClass);
  if not Assigned(ThisTreeNode) Then
    DoLog('ERROR Tree Class information: '+ThisClass.PathName);

  if Assigned(AClass.AncestorType) then
  begin
    AppendSym(CodeEl, '(');
    // Show parent class information
    if (AClass.AncestorType is TPasSpecializeType) then
    begin
      AppendText(CodeEl, 'specialize ');
      AppendHyperlink(CodeEl, TPasSpecializeType(AClass.AncestorType).DestType);
      AppendText(CodeEl, '<,>');
    end
    else
    begin
      AppendHyperlink(CodeEl, AClass.AncestorType);
      AppendInterfaceInfo(CodeEl, AClass);
    end;
    AppendSym(CodeEl, ')');
  end;
  // Class members
  CreateMemberDeclarations(AClass, AClass.Members,TableEl, not AClass.IsShortDefinition);

  AppendText(CreateH2(ContentElement), UTF8Decode(SDocInheritance));
  TableEl := CreateTable(ContentElement);

  // Process tree class information
  // First tree class link is to This class
  PrevClass:= nil;

  while True do
  begin
    TREl := CreateTR(TableEl);
    TDEl := CreateTD_vtop(TREl);
    TDEl['align'] := 'center';
    CodeEl := CreateCode(CreatePara(TDEl));

    // Show class item
    AppendHyperlink(CodeEl, ThisClass);
    if Assigned(PrevClass) and (PrevClass Is TPasClassType)  then // Interfaces from prevClass
      AppendInterfaceInfo(CodeEl, TPasClassType(PrevClass));
    AppendShortDescrCell(TREl, ThisClass);

    if Assigned(ThisTreeNode) then
      if Assigned(ThisTreeNode.ParentNode) then
      begin
        TDEl := CreateTD(CreateTR(TableEl));
        TDEl['align'] := 'center';
        AppendText(TDEl, '|');
        PrevClass:= ThisClass;
        ThisClass := ThisTreeNode.ParentNode.Element;
        ThisTreeNode := ThisTreeNode.ParentNode;
      end
        else
      begin
        ThisClass := nil;
        ThisTreeNode:= nil;
        PrevClass:= nil;
        break;
      end
    else
      break;
  end;
  FinishElementPage(AClass);
end;

procedure THTMLWriter.CreateClassInheritanceSubpage(aClass : TPasClassType; AFilter: TMemberFilter);

var
  ThisClass: TPasClassType;
  i: Integer;
  Member: TPasElement;
  TableEl, TREl, TDEl, ParaEl, LinkEl: TDOMElement;
begin
  TableEl := CreateTable(ContentElement);
  ThisClass := AClass;
  while True do
  begin
    TREl := CreateTR(TableEl);
    TDEl := CreateTD(TREl);
    TDEl['colspan'] := '3';
    CreateTD(TREl);
    LinkEl := AppendHyperlink(CreateEl(CreateCode(CreatePara(TDEl)), 'b'), ThisClass);
    if Assigned(LinkEl) then
      LinkEl['onClick'] := 'opener.location.href = ''' + LinkEl['href'] +
        '''; return false;';
    for i := 0 to ThisClass.Members.Count - 1 do
    begin
      Member := TPasElement(ThisClass.Members[i]);
      if Not (Engine.ShowElement(Member) and AFilter(Member)) then
        continue;
      TREl := CreateTR(TableEl);
      ParaEl := CreatePara(CreateTD(TREl));
      case Member.Visibility of
        visPrivate:
          AppendText(ParaEl, 'pv');
        visProtected:
          AppendText(ParaEl, 'pt');
        visPublished:
          AppendText(ParaEl, 'pl');
        else
      end;
      AppendNbSp(ParaEl, 1);

      ParaEl := CreateTD(TREl);
      if (Member.ClassType = TPasProperty) and
        (Length(TPasProperty(Member).WriteAccessorName) = 0) then
      begin
        AppendText(ParaEl, 'ro');
        AppendNbSp(ParaEl, 1);
      end;

      LinkEl := AppendHyperlink(CreatePara(CreateTD(TREl)), Member);
      if Assigned(LinkEl) then
        LinkEl['onClick'] := 'opener.location.href = ''' + LinkEl['href'] +
          '''; return false;';
    end;
    if (not Assigned(ThisClass.AncestorType)) or
      (not (ThisClass.AncestorType.ClassType.inheritsfrom(TPasClassType))) then
      break;
    ThisClass := TPasClassType(ThisClass.AncestorType);
    AppendNbSp(CreatePara(CreateTD(CreateTR(TableEl))), 1);
  end;
end;

procedure THTMLWriter.CreateClassSortedSubpage(AClass: TPasClassType; AFilter: TMemberFilter);
var
  List: TFPList;
  ThisClass: TPasClassType;
  i, j: Integer;
  Member: TPasElement;
  ParaEl, TableEl, TREl, TDEl, LinkEl: TDOMElement;

begin
  List := TFPList.Create;
  try
    ThisClass := AClass;
    while True do
    begin
      for i := 0 to ThisClass.Members.Count - 1 do
      begin
        Member := TPasElement(ThisClass.Members[i]);
        if Engine.ShowElement(Member) and AFilter(Member) then
        begin
          j := 0;
          while (j < List.Count) and
            (CompareText(TPasElement(List[j]).Name, Member.Name) < 0) do
            Inc(j);
          List.Insert(j, Member);
        end;
      end;
      if (not Assigned(ThisClass.AncestorType)) or
        (not (ThisClass.AncestorType.ClassType.inheritsfrom(TPasClassType))) then
        break;
      ThisClass := TPasClassType(ThisClass.AncestorType);
    end;

    TableEl := CreateTable(ContentElement);
    for i := 0 to List.Count - 1 do
    begin
      Member := TPasElement(List[i]);
      TREl := CreateTR(TableEl);
      ParaEl := CreatePara(CreateTD(TREl));
      case Member.Visibility of
        visPrivate:
          AppendText(ParaEl, 'pv');
        visProtected:
          AppendText(ParaEl, 'pt');
        visPublished:
          AppendText(ParaEl, 'pl');
        else
      end;
      AppendNbSp(ParaEl, 1);

      ParaEl := CreatePara(CreateTD(TREl));
      if (Member.ClassType = TPasProperty) and
        (Length(TPasProperty(Member).WriteAccessorName) = 0) then
      begin
        AppendText(ParaEl, 'ro');
        AppendNbSp(ParaEl, 1);
      end;

      TDEl := CreateTD(TREl);
      TDEl['nowrap'] := 'nowrap';
      ParaEl := CreatePara(TDEl);
      LinkEl := AppendHyperlink(ParaEl, Member);
      if Assigned(LinkEl) then
        LinkEl['onClick'] := 'opener.location.href = ''' + LinkEl['href'] +
          '''; return false;';
      AppendText(ParaEl, ' (');
      LinkEl := AppendHyperlink(ParaEl, Member.Parent);
      if Assigned(LinkEl) then
        LinkEl['onClick'] := 'opener.location.href = ''' + LinkEl['href'] +
          '''; return false;';
      AppendText(ParaEl, ')');
    end;
  finally
    List.Free;
  end;
end;

procedure THTMLWriter.CreateClassPageBody(AClass: TPasClassType; ASubpageIndex: Integer);

begin
  case ASubpageIndex of
    0:
      CreateClassMainPage(aClass);
    PropertiesByInheritanceSubindex:
      CreateClassInheritanceSubpage(aClass,@PropertyFilter);
    PropertiesByNameSubindex:
      CreateClassSortedSubpage(aClass,@PropertyFilter);
    MethodsByInheritanceSubindex:
      CreateClassInheritanceSubpage(aClass,@MethodFilter);
    MethodsByNameSubindex:
      CreateClassSortedSubpage(aClass,@MethodFilter);
    EventsByInheritanceSubindex:
      CreateClassInheritanceSubpage(aClass,@EventFilter);
    EventsByNameSubindex:
      CreateClassSortedSubpage(aClass,@EventFilter);
  end;
end;

procedure THTMLWriter.CreateClassMemberPageBody(AElement: TPasElement);
var
  TableEl, TREl, CodeEl: TDOMElement;

  procedure CreateVarPage(Element: TPasVariable);
  begin
    AppendHyperlink(CodeEl, Element.Parent);
    AppendSym(CodeEl, '.');
    AppendText(CodeEl, UTF8Decode(Element.Name));
    if Assigned(Element.VarType) then
    begin
      AppendSym(CodeEl, ' : ');
      AppendSym(AppendType(CodeEl, TableEl, Element.VarType, False), ';');
    end;
  end;

  procedure CreateTypePage(Element: TPasType);
  begin
    AppendKw(CodeEl, 'type ');
    AppendHyperlink(CodeEl, Element.Parent);
    AppendSym(CodeEl, '.');
    AppendText(CodeEl, UTF8Decode(Element.Name));
    AppendSym(CodeEl, ' = ');
    AppendTypeDecl(Element,TableEl,CodeEl)
  end;

  procedure CreateConstPage(Element: TPasConst);
  begin
    AppendKw(CodeEl, 'const ');
    AppendHyperlink(CodeEl, Element.Parent);
    AppendSym(CodeEl, '.');
    AppendText(CodeEl, UTF8Decode(Element.Name));
    if Assigned(Element.VarType) then
      begin
      AppendSym(CodeEl, ': ');
      AppendType(CodeEl, TableEl, Element.VarType, False);
      end;
    AppendPasSHFragment(CodeEl, ' = ' + Element.Expr.GetDeclaration(True) + ';', 0);
  end;

  procedure CreatePropertyPage(Element: TPasProperty);
  var
    NeedBreak: Boolean;
    T : TPasType;
    A : TPasArgument;
    I : integer;

  begin
    AppendKw(CodeEl, 'property ');
    AppendHyperlink(CodeEl, Element.Parent);
    AppendSym(CodeEl, '.');
    AppendText(CodeEl, UTF8Decode(Element.Name));
    if Assigned(Element.Args) and (Element.Args.Count>0) then
      begin
      AppendSym(CodeEl,'[');
      For I:=0 to Element.Args.Count-1 do
        begin
        If I>0 then
          AppendSym(CodeEl,',');
        A:=TPasArgument(Element.Args[i]);
        AppendText(CodeEl, UTF8Decode(A.Name));
        AppendSym(CodeEl,': ');
        if Assigned(A.ArgType) then
          AppendText(CodeEl,UTF8Decode(A.ArgType.Name))
        else
          AppendText(CodeEl,'<Unknown>');
        end;
      AppendSym(CodeEl,']');
      end;
    T:=Element.ResolvedType;
    if Assigned(T) then
    begin
      AppendSym(CodeEl, ' : ');
      AppendType(CodeEl, TableEl, T, False);
    end;

    NeedBreak := False;
    if Length(TPasProperty(Element).IndexValue) <> 0 then
    begin
      CreateEl(CodeEl, 'br');
      AppendNbsp(CodeEl, 2);
      AppendKw(CodeEl, 'index ');
      AppendPasSHFragment(CodeEl, TPasProperty(Element).IndexValue, 0);
      NeedBreak := True;
    end;
    if Length(TPasProperty(Element).ReadAccessorName) <> 0 then
    begin
      CreateEl(CodeEl, 'br');
      AppendNbsp(CodeEl, 2);
      AppendKw(CodeEl, 'read ');
      AppendText(CodeEl, UTF8Decode(TPasProperty(Element).ReadAccessorName));
      NeedBreak := True;
    end;
    if Length(TPasProperty(Element).WriteAccessorName) <> 0 then
    begin
      CreateEl(CodeEl, 'br');
      AppendNbsp(CodeEl, 2);
      AppendKw(CodeEl, 'write ');
      AppendText(CodeEl, UTF8Decode(TPasProperty(Element).WriteAccessorName));
      NeedBreak := True;
    end;
    if Length(TPasProperty(Element).StoredAccessorName) <> 0 then
    begin
      CreateEl(CodeEl, 'br');
      AppendNbsp(CodeEl, 2);
      AppendKw(CodeEl, 'stored ');
      AppendText(CodeEl, UTF8Decode(TPasProperty(Element).StoredAccessorName));
      NeedBreak := True;
    end;
    if Length(TPasProperty(Element).DefaultValue) <> 0 then
    begin
      CreateEl(CodeEl, 'br');
      AppendNbsp(CodeEl, 2);
      AppendKw(CodeEl, 'default ');
      AppendPasSHFragment(CodeEl, TPasProperty(Element).DefaultValue, 0);
      NeedBreak := True;
    end;

    AppendSym(CodeEl, ';');

    if TPasProperty(Element).IsDefault or TPasProperty(Element).IsNodefault then
    begin
      if NeedBreak then
      begin
        CreateEl(CodeEl, 'br');
        AppendNbsp(CodeEl, 2);
      end;
      if TPasProperty(Element).IsDefault then
        AppendKw(CodeEl, 'default')
      else
        AppendKw(CodeEl, 'nodefault');
      AppendSym(CodeEl, ';');
    end;
  end;

var
  s: String;
begin
  AppendMenuBar(-1);
  AppendTitle(UTF8Decode(AElement.FullName),AElement.Hints);
  AppendShortDescr(CreatePara(ContentElement), AElement);
  AppendText(CreateH2(ContentElement), SDocDeclaration);
  AppendSourceRef(ContentElement,AElement);

  TableEl := CreateTable(ContentElement);
  TREl := CreateTR(TableEl);
  CodeEl := CreateCode(CreatePara(CreateTD(TREl)));
  AppendText(CodeEl, ' ');      // !!!: Workaround for current HTML writer

  if (AElement.Visibility<>visDefault) then
    begin
    s:=VisibilityNames[AElement.Visibility];
    AppendKw(CodeEl, s);
    end;
  AppendText(CodeEl, ' ');

  if AElement is TPasProperty then
    CreatePropertyPage(TPasProperty(AElement))
  else if AElement is TPasConst then
    CreateConstPage(TPasConst(AElement))
  else if (AElement is TPasVariable) then
    CreateVarPage(TPasVariable(AElement))
  else if AElement is TPasProcedureBase then
    AppendProcDecl(CodeEl, TableEl, TPasProcedureBase(AElement))
  else if AElement is TPasType then
    CreateTypePage(TPasType(AElement))
  else
    AppendText(CreateWarning(ContentElement), '<' + AElement.ClassName + '>');

  FinishElementPage(AElement);
end;

procedure THTMLWriter.CreateVarPageBody(AVar: TPasVariable);
var
  TableEl, TREl, TDEl, CodeEl, El: TDOMElement;
begin
  AppendMenuBar(-1);
  AppendTitle(AVar.FullName,AVar.Hints);
  AppendShortDescr(CreatePara(ContentElement), AVar);
  AppendText(CreateH2(ContentElement), SDocDeclaration);
  AppendSourceRef(ContentElement,AVar);

  TableEl := CreateTable(ContentElement);
  TREl := CreateTR(TableEl);
  TDEl := CreateTD(TREl);
  CodeEl := CreateCode(CreatePara(TDEl));

  AppendKw(CodeEl, 'var');
  AppendText(CodeEl, ' ' + AVar.Name);
  if Assigned(AVar.VarType) then
  begin
    AppendSym(CodeEl, ': ');
    El := AppendType(CodeEl, TableEl, AVar.VarType, False);
  end else
    El := CodeEl;
  if Length(AVar.Value) > 0 then
    AppendPasSHFragment(El, ' = ' + AVar.Value + ';', 0)
  else
    AppendSym(El, ';');

  FinishElementPage(AVar);
end;

procedure THTMLWriter.CreateProcPageBody(AProc: TPasProcedureBase);

var
  TableEl, TREl, TDEl, CodeEl: TDOMElement;

begin
  AppendMenuBar(-1);
  AppendTitle(UTF8Decode(AProc.Name),AProc.Hints);
  AppendShortDescr(CreatePara(ContentElement), AProc);
  AppendText(CreateH2(ContentElement), SDocDeclaration);
  AppendSourceRef(ContentElement,AProc);

  TableEl := CreateTable(ContentElement);
  TREl := CreateTR(TableEl);
  TDEl := CreateTD(TREl);
  CodeEl := CreateCode(CreatePara(TDEl));

  AppendProcDecl(CodeEl, TableEl, AProc);

  FinishElementPage(AProc);
end;

function THTMLWriter.InterPretOption ( const Cmd, Arg: String ) : boolean;

  Function ReadFile(aFileName : string) : TstringStream;

  begin
    aFileName:= SetDirSeparators(aFileName);
    try
      if copy(aFileName,1,1)<>'@' then
        Result:=TStringStream.Create(aFileName)
      else
        begin
        Delete(aFileName,1,1);
        Result:=TStringStream.Create('');
        Result.LoadFromFile(aFileName);
        Result.Position:=0;
        end;
    except
      Result.Free;
      Raise;
    end;
  end;

begin
  Result:=True;
  if Cmd = '--html-search' then
    SearchPage := Arg
  else if Cmd = '--footer' then
    FFooterHTML := ReadFile(Arg)
  else if Cmd = '--header' then
    FHeaderHTML := ReadFile(Arg)
  else if Cmd = '--navigator' then
    FNavigatorHTML := ReadFile(Arg)
  else if Cmd = '--charset' then
    CharSet := Arg
  else if Cmd = '--index-colcount' then
    IndexColCount := StrToIntDef(Arg,IndexColCount)
  else if Cmd = '--image-url' then
    BaseImageURL  := Arg
  else if Cmd = '--css-file' then
    FCSSFile := arg
  else if Cmd = '--footer-date' then
    begin
    FIncludeDateInFooter:=True;
    FDateFormat:=Arg;
    end
  else if Cmd = '--disable-menu-brackets' then
    FUseMenuBrackets:=False
  else
    Result:=inherited InterPretOption(Cmd, Arg);
end;


class procedure THTMLWriter.Usage(List: TStrings);
begin
  List.add('--header=file');
  List.Add(SHTMLUsageHeader);
  List.add('--footer=file');
  List.Add(SHTMLUsageFooter);
  List.add('--navigator=file');
  List.Add(SHTMLUsageNavigator);
  List.Add('--footer-date[=Fmt]');
  List.Add(SHTMLUsageFooterDate);
  List.Add('--charset=set');
  List.Add(SHTMLUsageCharset);
  List.Add('--html-search=pagename');
  List.Add(SHTMLHtmlSearch);
  List.Add('--index-colcount=N');
  List.Add(SHTMLIndexColcount);
  List.Add('--image-url=url');
  List.Add(SHTMLImageUrl);
  List.Add('--disable-menu-brackets');
  List.Add(SHTMLDisableMenuBrackets);
  inherited Usage(List);
end;

class procedure THTMLWriter.SplitImport(var AFilename, ALinkPrefix: String);
var
  i: integer;
begin
  i := Pos(',', AFilename);
  if i > 0 then
    begin  //split into filename and prefix
    ALinkPrefix := Copy(AFilename,i+1,Length(AFilename));
    SetLength(AFilename, i-1);
    end
  else if ALinkPrefix = '' then
    begin  //synthesize outdir\pgk.xct, ..\pkg
    ALinkPrefix := '../' + ChangeFileExt(ExtractFileName(AFilename), '');
    AFilename := ChangeFileExt(AFilename, '.xct');
    end;
end;

class function THTMLWriter.FileNameExtension: String;
begin
  result:='';
end;

// private methods


procedure THTMLWriter.SetOnTest(const AValue: TNotifyEvent);
begin
  if FOnTest=AValue then exit;
    FOnTest:=AValue;
end;


initialization
  // Do not localize.
  RegisterWriter(THTMLWriter,'html','HTML output using fpdoc.css stylesheet.');

finalization
  UnRegisterWriter('html');

end.
