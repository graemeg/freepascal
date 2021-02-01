{
    FPDoc  -  Free Pascal Documentation Tool
    Copyright (C) 2021 by Michael Van Canneyt

    * Markdown generator, multi-file

    See the file COPYING, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
{$mode objfpc}
{$H+}

unit dw_markdown;
{$WARN 5024 off : Parameter "$1" not used}
interface

uses Classes, dGlobals, PasTree, dWriter, dw_basemd, DOM;


type

  TMemberListOption = (mloAppendParent,mloAppendType,mloCheckVisibility);
  TMemberListOptions = Set of TMemberListOption;
  TNavigationMode = (nmUnitTree,nmUnitSubTree);
  { TMarkdownWriter }

  TMarkdownWriter = class(TBaseMarkdownWriter)
  private
    FBaseImageURL: String;
    FImageFileList: TStrings;
    FIndexColCount: Integer;
    FAdditionalConfig : TStrings;
    FFooterMarkDown : TStrings;
    FHeaderMarkDown : TStrings;
    FNavigationMode: TNavigationMode;
    FOnTest: TNotifyEvent;
    FNavigation : TStrings;
    FUnitSubTreeStarted : Boolean;
    function GetAdditionalConfig: TStrings;
    function GetClassDeclaration(aEl: TPasClassType): String;
    function GetClassDeclarationFirstLine(aEl: TPasClassType): String;
    function GetDeclaration(aEl: TPasElement): String;
    function GetFooterMarkDown: TStrings;
    function GetHeaderMarkDown: TStrings;
    function GetPageCount: Integer;
    procedure SetOnTest(const AValue: TNotifyEvent);
  protected
    function GetFileBaseDir(aOutput: String): String; override;

    // MkDocs
    procedure WriteMkdocsYaml; virtual;
    procedure CreateMkdocsYaml(mkDocs: TStrings); virtual;
    procedure AddToNavigation(const aFileName: String; aElement: TPasElement; aSubPageIndex: Integer);

    // Some raw markdown functions

    procedure AppendPageFooter; virtual;
    procedure WriteMetadata; virtual;
    procedure AppendPageHeader; virtual;
    Procedure AppendText(Const S : String);
    Procedure AppendTitle(Const S : String);
    Procedure AppendTitle(Const Fmt : String; Const Args : Array of const);

    // Description
    Procedure AppendShortDescr(AContext : TPasElement; DocNode : TDocNode); virtual;
    procedure AppendShortDescr(Element: TPasElement); virtual;
    procedure AppendShortDescrCell(Element: TPasElement);  virtual;
    procedure AppendDescr(AContext: TPasElement; DescrNode: TDOMElement; AutoInsertBlock: Boolean); virtual;
    procedure AppendDescrSection(AContext: TPasElement; DescrNode: TDOMElement; const ATitle: String); virtual;
    procedure AppendDescrSection(AContext: TPasElement; DescrNode: TDOMElement; const ATitle: DOMString); virtual;
    Procedure AppendHyperlink(Element: TPasElement);
    Function CreateHyperlink(Element: TPasElement) : string;

    // Reusable routines
    procedure CollectDeclarationTypes(aList: TStringList; aElement: TPasElement );
    procedure CollectSeeAlsoNodes(aList: TStringList; aSeeAlso: TDomNode);
    procedure AddModuleIdentifiers(AModule : TPasModule; L : TStrings);

    procedure AppendSourceRef(AElement: TPasElement); virtual;
    procedure AppendDeclaration(aEl: TPasElement; aKind: string; PrependVisibility: Boolean); virtual;
    procedure FinishElementPage(AElement: TPasElement; SkipDescription: Boolean=false); virtual;
    Procedure AppendSeeAlsoSection(AElement : TPasElement;DocNode : TDocNode); virtual;
    Procedure AppendExampleSection(AElement : TPasElement;DocNode : TDocNode); virtual;
    procedure AppendProcArgsSection(Element: TPasProcedureBase);  virtual;
    procedure CreateMemberDeclarations(AParent: TPasElement; Members: TFPList; Options : TMemberListOptions); virtual;
    Procedure CreateTopicLinks(Node : TDocNode; PasElement : TPasElement); virtual;
    procedure CreateIndexPage(L : TStringList); virtual;
    procedure CreateSimpleSubpage(aModule: TPasModule; const ATitle, aItem: String; AList: TFPList); virtual;

    // Various kind of pages.
    // Main entry
    procedure CreatePageBody(AElement: TPasElement; ASubpageIndex: Integer); virtual;
    // Package
    procedure CreatePackagePageBody; virtual;
    procedure CreatePackageIndex; virtual;
    procedure CreatePackageClassHierarchy; virtual;
    procedure CreateClassHierarchyPage(AddUnit : Boolean); virtual;
    // Module
    procedure CreateModuleIndexPage(AModule: TPasModule); virtual;
    procedure CreateModuleMainPageBody(AModule: TPasModule); virtual;
    procedure CreateModulePageBody(AModule: TPasModule; ASubpageIndex: Integer); virtual;
    // Per simple type
    procedure CreateResStringsPage(aModule: TPasModule); virtual;
    Procedure CreateTopicPageBody(AElement : TTopicElement); virtual;
    procedure CreateConstPageBody(AConst: TPasConst); virtual;
    procedure CreateTypePageBody(AType: TPasType); virtual;
    procedure CreateVarPageBody(AVar: TPasVariable); virtual;
    procedure CreateProcPageBody(AProc: TPasProcedureBase); virtual;
    // Class/Record
    procedure CreateClassMainPage(aClass: TPasClassType); virtual;
    procedure CreateClassPageBody(AClass: TPasClassType; ASubpageIndex: Integer); virtual;
    procedure CreateClassMemberPageBody(AElement: TPasElement); virtual;
    procedure CreateInheritanceSubpage(aClass: TPasClassType; aTitle : string; AFilter: TMemberFilter); virtual;
    procedure CreateSortedSubpage(ACLass: TPasClassType; aTitle : string; AFilter: TMemberFilter ); virtual;
    //  Here we write the documentation
    Procedure DoWriteDocumentation; override;
  public
    constructor Create(APackage: TPasPackage; AEngine: TFPDocEngine); override;
    destructor Destroy; override;

    // Single-page generation
    procedure WriteDocPage(const aFileName: String; aElement: TPasElement; aSubPageIndex: Integer); override;

    // Start producing html complete package documentation

    Function InterPretOption(Const Cmd,Arg : String) : boolean; override;
    Class Function FileNameExtension : String; override;
    class procedure Usage(List: TStrings); override;
    Class procedure SplitImport(var AFilename, ALinkPrefix: String); override;

    property OnTest: TNotifyEvent read FOnTest write SetOnTest;
    Property IndexColCount : Integer Read FIndexColCount write FIndexColCount;
    Property BaseImageURL : String Read FBaseImageURL Write FBaseImageURL;
    Property HeaderMarkDown : TStrings Read GetHeaderMarkDown;
    Property FooterMarkDown : TStrings Read GetFooterMarkDown;
    property AdditionalConfig : TStrings Read GetAdditionalConfig;
    property NavigationMode:  TNavigationMode Read FNavigationMode;
  end;

implementation

uses fpdocstrs, SysUtils, fpdocclasstree;


Function FixHTMLpath(S : String) : STring;

begin
  Result:=StringReplace(S,'\','/',[rfReplaceAll]);
end;

constructor TMarkdownWriter.Create(APackage: TPasPackage; AEngine: TFPDocEngine);

begin
  inherited Create(APackage, AEngine);
  IndexColCount:=3;
  FImageFileList := TStringList.Create;
  FNavigation:=TStringList.Create;
end;

destructor TMarkdownWriter.Destroy;

begin
  FreeAndNil(FImageFileList);
  FreeAndNil(FAdditionalConfig);
  FreeAndNil(FFooterMarkDown);
  FreeAndNil(FHeaderMarkDown);
  FreeAndNil(FNavigation);
  inherited Destroy;
end;

procedure TMarkdownWriter.AddToNavigation(const aFileName: String; aElement: TPasElement; aSubPageIndex: Integer);

  Procedure AddToNav(aLevel : Integer; aName,aFile : String);
  begin
    if aName<>'' then
      aName:=''''+aName+''':';
    if aFile<>'' then
      aFile:=' '''+aFile+'''';
    FNavigation.Add(StringOfChar(' ',aLevel*4)+'- '+aName+aFile);
  end;
Var
  Offs : Integer;

begin
  if aElement is TPasPackage then
    begin
    case aSubPageIndex of
      IdentifierIndex:
        begin
        AddToNav(1,SDocPackageLinkTitle,'');
        AddToNav(2,SDocOverview,aFileName);
        end;
      IndexSubIndex :  AddToNav(2,SDocIdentifierIndex,aFileName);
      ClassHierarchySubIndex :  AddToNav(2,SDocPackageClassHierarchy,aFileName);
    end
    end
  else if (aElement is TPasModule) then
    begin
    offS:=Ord (NavigationMode=nmUnitSubTree);
    if Offs=1 then
      if Not FUnitSubTreeStarted then
        begin
        FUnitSubTreeStarted:=True;
        AddToNav(1,SDocUnits,'');
        end;;
    case aSubPageIndex of
      IdentifierIndex :
        begin
        if Offs=0 then
          AddToNav(1+Offs,Format(StringReplace(SDocUnitMenuTitle,'''','',[rfReplaceALl]),[aElement.Name]),'')
        else
          AddToNav(1+Offs,aElement.Name,'');
        AddToNav(2+offs,SDocOverview,aFileName);
        end;
      ResstrSubindex:  AddToNav(2+Offs,SDocResStrings,aFileName);
      ConstsSubindex:  AddToNav(2+Offs,SDocConstants,aFileName);
      TypesSubindex: AddToNav(2+Offs,SDocTypes,aFileName);
      ClassesSubindex: AddToNav(2+Offs,SDocClasses,aFileName);
      ProcsSubindex: AddToNav(2+Offs,SDocProceduresAndFunctions,aFileName);
      VarsSubindex: AddToNav(2+Offs,SDocVariables,aFileName);
    end;
    end
end;

procedure TMarkdownWriter.WriteDocPage(const aFileName: String; aElement: TPasElement; aSubPageIndex: Integer);

begin
  if MarkDownEngine=meMkDocs then
    AddToNavigation(aFileName,aElement,aSubPageIndex);
  ClearMarkDown;
  WriteMetaData;
  AppendPageHeader;
  CreatePageBody(AElement, ASubpageIndex);
  AppendPageFooter;
  SaveToFile(GetFileBaseDir(Engine.Output)+aFileName);
end;

procedure TMarkdownWriter.WriteMetadata;

begin

end;

procedure TMarkdownWriter.AppendPageHeader;

Var
  S : String;
begin
  if Assigned(FHeaderMarkDown) then
    begin
    EnsureEmptyLine;
    For S in FHeaderMarkDown do
      AppendToLine(S,False)
    end;
end;

procedure TMarkdownWriter.WriteMkdocsYaml;

Var
  mkDocs : TStrings;

begin
  mkDocs:=TStringList.Create;
  try
    CreatemkDocsYaml(mkDocs);
    mkDocs.SaveToFile(Engine.Output+PathDelim+'mkdocs.yml');
  finally
    Mkdocs.Free;
  end;
end;

procedure TMarkdownWriter.CreateMkdocsYaml(mkDocs: TStrings);

begin
  With MKDocs do
    begin
    add('site_name: '+SDocPackageTitle,[Package.Name]);
    add('');
    add('docs_dir: docs');
    add('');
    add('site_dir: site');
    add('');
    add('markdown_extensions:');
    add('  - def_list');
    add('  - codehilite');
    add('  - admonition');
    add('');
    add('theme: '+Theme);
    If Assigned(FAdditionalConfig) then
      begin
      add('');
      AddStrings(FAdditionalConfig);
      end;
    add('');
    add('nav:');
    AddStrings(FNavigation);
    end;
end;

procedure TMarkdownWriter.DoWriteDocumentation;

begin
  Inherited;
  If MarkDownEngine=memkDocs then
    WriteMkdocsYaml;
end;

function TMarkdownWriter.GetFooterMarkDown: TStrings;
begin
  If FFooterMarkDown=Nil then
    FFooterMarkDown:=TStringList.Create;
  Result:=FFooterMarkDown;
end;

function TMarkdownWriter.GetHeaderMarkDown: TStrings;
begin
  If FHeaderMarkDown=Nil then
    FHeaderMarkDown:=TStringList.Create;
  Result:=FHeaderMarkDown;
end;

procedure TMarkdownWriter.AppendShortDescr(AContext: TPasElement; DocNode: TDocNode) ;

Var
  N : TDocNode;

begin
  if Not Assigned(DocNode) then
   exit;
  N:=Nil;
  If (DocNode.Link<>'') then
    N:=Engine.FindLinkedNode(DocNode);
  If (N=Nil) then
    N:=DocNode;
  If Assigned(N.ShortDescr) then
    if not ConvertShort(AContext,N.ShortDescr) then
      Warning(AContext, SErrInvalidShortDescr)
end;

procedure TMarkdownWriter.AppendShortDescr(Element: TPasElement);

begin
  AppendShortDescr(Element,Engine.FindDocNode(Element));
end;

procedure TMarkdownWriter.AppendShortDescrCell(Element: TPasElement);

begin
  if Not Assigned(Engine.FindShortDescr(Element)) then
    exit;
  DescrBeginTableCell;
  AppendShortDescr(Element);
  DescrEndTableCell;
end;

procedure TMarkdownWriter.AppendDescr(AContext: TPasElement; DescrNode: TDOMElement; AutoInsertBlock: Boolean);
begin
  if Not Assigned(DescrNode) then
    exit;
  EnsureEmptyLine;
  ConvertDescr(AContext, DescrNode, AutoInsertBlock);
end;

procedure TMarkdownWriter.AppendDescrSection(AContext: TPasElement;  DescrNode: TDOMElement; const ATitle: String);
begin
  if IsDescrNodeEmpty(DescrNode) then
    exit;
  If (ATitle<>'') then // Can be empty for topic.
    AppendHeader(2,ATitle);
  AppendDescr(AContext, DescrNode, True);
end;

procedure TMarkdownWriter.AppendDescrSection(AContext: TPasElement; DescrNode: TDOMElement; const ATitle: DOMString);
begin
  AppendDescrSection(aContext,DescrNode,UTF8Encode(aTitle));
end;

procedure TMarkdownWriter.AppendHyperlink(Element: TPasElement);

begin
  if Not Assigned(Element) then
    begin
    AppendText('<NIL>');
    exit;
    end;
  AppendToLine(CreateHyperLink(Element),False)
end;

function TMarkdownWriter.CreateHyperlink(Element: TPasElement): string;

var
  s: DOMString;
  UnitList: TFPList;
  i: Integer;
  ThisPackage: TLinkNode;
begin
  if Not Assigned(Element) then
    Exit('\<NIL\>');
  if Not Element.InheritsFrom(TPasUnresolvedTypeRef) then
    begin
    if Element is TPasEnumValue then
      s := ResolveLinkID(Element.Parent.PathName)
    else
      s := ResolveLinkID(Element.PathName);
    end
  else
    begin
    s := ResolveLinkID(Element.Name);
    if Length(s) = 0 then
      begin
      { Try all packages }
      ThisPackage := Engine.RootLinkNode.FirstChild;
      while Assigned(ThisPackage) do
        begin
        s := ResolveLinkID(ThisPackage.Name + '.' + Element.Name);
        if Length(s) > 0 then
          break;
        ThisPackage := ThisPackage.NextSibling;
        end;
      if (Length(s) = 0) and Assigned(Module) then
        begin
        { Okay, then we have to try all imported units of the current module }
        UnitList := Module.InterfaceSection.UsesList;
        for i := UnitList.Count - 1 downto 0 do
          begin
          { Try all packages }
          ThisPackage := Engine.RootLinkNode.FirstChild;
          while Assigned(ThisPackage) do
            begin
            s := ResolveLinkID(ThisPackage.Name + '.' +
              TPasType(UnitList[i]).Name + '.' + Element.Name);
            if Length(s) > 0 then
              break;
            ThisPackage := ThisPackage.NextSibling;
            end;
          if length(s)=0 then
            s := ResolveLinkID('#rtl.System.' + Element.Name);
          if Length(s) > 0 then
            break;
          end;
        end;
      end;
    end;

  if Length(s) > 0 then
    Result:=CreateLink(Element.Name,UTF8Encode(S))
  else
    Result:=Element.Name;
end;

procedure TMarkdownWriter.AppendProcArgsSection(Element: TPasProcedureBase);

var
  HasFullDescr, IsFirstType: Boolean;
  ResultEl: TPasResultElement;
  DocNode: TDocNode;
  aList : TStringList;

  Procedure CollectVariant(AProc: TPasProcedure);

  var
    i: Integer;
    Arg: TPasArgument;
    aType : TPasProcedureType;

  begin
    aType:=aProc.ProcType;
    for I:=0 to aType.Args.Count-1 do
      begin
      Arg:=TPasArgument(aType.Args[i]);
      if IsDescrNodeEmpty(Engine.FindShortDescr(Arg)) then
        Continue;
      aList.AddObject(Arg.Name,Arg);
      end;
    if (aType is TPasFunctionType) and (ResultEl=Nil) then
      ResultEl:=TPasFunctionType(aType).ResultEl;
  end;

  Procedure WriteType(aProc : TPasProcedure; aName: string);
  var
    i: Integer;
    Arg: TPasArgument;
    aType : TPasProcedureType;

  begin
    aType:=aProc.ProcType;
     for I:=0 to aType.Args.Count-1 do
      begin
      Arg:=TPasArgument(aType.Args[i]);
      if SameText(Arg.Name,aName) then
        begin
        if not IsFirstType then
          AppendText(', ');
        AppendHyperlink(Arg.ArgType);
        end;
      if IsDescrNodeEmpty(Engine.FindShortDescr(Arg)) then
        Continue;
      aList.AddObject(Arg.Name,Arg);
      end;

  end;

  Procedure WriteTypes(aName: string);

  Var
    I : Integer;

  begin
    IsFirstType:=True;
    if Element.ClassType <> TPasOverloadedProc then
      WriteType(TPasProcedure(Element),aName)
    else for i := 0 to TPasOverloadedProc(Element).Overloads.Count - 1 do
      WriteType(TPasProcedure(TPasOverloadedProc(Element).Overloads[i]),AName);
  end;

Var
  I: Integer;

begin
  ResultEl:=Nil;
  aList:=TStringList.Create;
  try
    AList.Duplicates:=DupIgnore;
    if Element.ClassType <> TPasOverloadedProc then
      CollectVariant(TPasProcedure(Element))
    else for i := 0 to TPasOverloadedProc(Element).Overloads.Count - 1 do
      CollectVariant(TPasProcedure(TPasOverloadedProc(Element).Overloads[i]));
    If AList.Count<>0 then
      begin
      AppendHeader(2,SDocArguments);
      AppendTableHeader([SDocName,SDocTypes,SDocDescription]);
      for I:=0 to aList.Count-1 do
        begin
        DescrBeginTableRow;
        DescrBeginTableCell;
        AppendText(aList[i]);
        DescrEndTableCell;
        DescrBeginTableCell;
        WriteTypes(aList[i]);
        DescrEndTableCell;
        AppendShortDescrCell(TPasArgument(aList.Objects[i]));
        DescrEndTableRow;
        end;
      DescrEndTable;
      end;
  finally
    aList.Free;
  end;
  if Not Assigned(ResultEl) then
    Exit;
  DocNode := Engine.FindDocNode(ResultEl);
  HasFullDescr := Assigned(DocNode) and not IsDescrNodeEmpty(DocNode.Descr);
  if HasFullDescr or
    (Assigned(DocNode) and not IsDescrNodeEmpty(DocNode.ShortDescr)) then
    begin
    AppendHeader(2, SDocFunctionResult);
    if HasFullDescr then
      AppendDescr(ResultEl,DocNode.Descr, True)
    else
      AppendDescr(ResultEl,DocNode.ShortDescr, False);
    end;
end;



procedure TMarkdownWriter.AppendSourceRef(AElement: TPasElement);
begin
  EnsureEmptyLine;
  AppendToLine(Format(SDocSourcePosition,
    [ExtractFileName(AElement.SourceFilename), AElement.SourceLinenumber]));
  EnsureEmptyLine;
end;


procedure TMarkdownWriter.CollectSeeAlsoNodes(aList : TStringList; aSeeAlso: TDomNode);

Var
  Node : TDOMNode;
  L : String;
  El : TDOMElement;

begin
  Node:=aSeeAlso.FirstChild;
  While Assigned(Node) do
    begin
    if (Node.NodeType=ELEMENT_NODE) and (Node.NodeName='link') then
      begin
      El:=TDOMElement(Node);
      l:=UTF8encode(El['id']);
      aList.AddObject(L,El);
      end;
    Node:=Node.NextSibling;
    end;
end;

procedure TMarkdownWriter.CollectDeclarationTypes(aList : TStringList; aElement : TPasElement);

  Procedure MaybeAdd(aType : TPasType);

  Var
    N : TDocNode;
  begin
     if aType is TPasPointerType then
       aType:=TPasPointerType(aType).DestType;
     if (aType=Nil) or (aType.Name='') then
       exit;
     N:=Engine.FindDocNode(aType);
     if N<>Nil then
       aList.AddObject(aType.Name,aType);
  end;

Var
  I : integer;

begin
  if aElement is TPasVariable then
    MaybeAdd(TPasVariable(aElement).VarType)
  else if aElement is TPasMembersType then
    begin
    for I:=0 to TPasMembersType(aElement).Members.Count-1 do
      if TObject(TPasMembersType(aElement).Members[i]) is TPasVariable then
        MaybeAdd(TPasVariable(TPasMembersType(aElement).Members[i]).VarType);
    end;
end;

procedure TMarkdownWriter.AppendSeeAlsoSection(AElement: TPasElement; DocNode: TDocNode) ;

  Procedure AppendSeeALso(aID : String; El: TDomElement);
  Var
     S,N : DOMString;
     doBold : Boolean;
  begin
    s:= ResolveLinkID(aID);
    doBold:=Length(S)=0;
    if DoBold then
      begin
      if assigned(module) then
        s:=UTF8Decode(module.name)
      else
        s:='?';
      if aID='' then aID:='<empty>';
      if Assigned(AElement) then
        N:=UTF8Decode(AElement.Name)
      else
        N:='?';
      DoLog(SErrUnknownLinkID, [s,N,aID]);
      LinkUnresolvedInc();
      end ;
     if doBold then
       DescrBeginBold
     else
       DescrBeginURL(S);
     if Not IsDescrNodeEmpty(El) then
       ConvertBaseShortList(AElement, El, True)
     else
       AppendText(aID);
     if doBold then
       DescrEndBold
     else
       DescrEndURL();
  end;

  Procedure AppendTypeRef(aName : String; El: TPasType);

  begin
    AppendHyperLink(El);
  end;

var
  I : Integer;
  aList : TStringList;
  DescrEl : TDomElement;

begin
  if Not (Assigned(DocNode) and Assigned(DocNode.SeeAlso)) then
    Exit;
  AList:=TStringList.Create;
  Try
    aList.Duplicates:=dupIgnore;
    // AList will have a TDomElement (seealso) or a TPasType element as object
    CollectSeeAlsoNodes(aList,DocNode.SeeAlso);
    CollectDeclarationTypes(aList,aElement);
    if aList.Count = 0 then
      exit;
    aList.Sort;
    AppendHeader(2,SDocSeeAlso);
    AppendTableHeader([SDocName,SDocDescription]);
    For I:=0 to aList.Count-1 do
      begin
      DescrEl:=Nil;
      DescrBeginTableRow;
      DescrBeginTableCell;
      if aList.Objects[I] is TDomElement then
        AppendSeeAlso(aList[i],TDomElement(aList.Objects[i]))
      else if aList.Objects[i] is TPasType then
        AppendTypeRef(aList[i],TPasType(aList.Objects[i]));

      DescrEndTableCell;
      DescrBeginTableCell;
      DescrEl:=Engine.FindShortDescr(AElement.GetModule(),UTF8Encode(aList[i]));
      if Assigned(DescrEl) then
        ConvertShort(AElement, DescrEl)
      else
        AppendToLine(' ',False);
      DescrEndTableCell;
      DescrEndTableRow;
      end;
    DescrEndTable;
  Finally
    aList.Free;
  end;
end;

procedure TMarkdownWriter.AppendExampleSection ( AElement: TPasElement;
  DocNode: TDocNode ) ;

var
  Node: TDOMNode;
  fn,s: String;
  f: Text;

begin
  if not (Assigned(DocNode) and Assigned(DocNode.FirstExample)) then
    Exit;
  Node := DocNode.FirstExample;
  while Assigned(Node) do
    begin
    if (Node.NodeType = ELEMENT_NODE) and (Node.NodeName = 'example') then
      begin
      fn:=Engine.GetExampleFilename(TDOMElement(Node));
      If (fn<>'') then
        begin
        AppendHeader(2, SDocExample);
        try
          Assign(f, FN);
          Reset(f);
          try
            DescrBeginCode(False, UTF8Encode(TDOMElement(Node)['highlighter']));
            while not EOF(f) do
              begin
              ReadLn(f, s);
              DescrWriteCodeLine(s);
              end;
          finally
            Close(f);
            DescrEndCode;
          end;
        except
          on e: Exception do
            begin
            e.Message := '[example] ' + e.Message;
            raise;
            end;
        end;
        end;
      end;
    Node := Node.NextSibling;
    end;
end;

procedure TMarkdownWriter.AppendPageFooter;

Var
  S : String;
begin
  if Assigned(FFooterMarkDown) then
    begin
    EnsureEmptyLine;
    For S in FFooterMarkDown do
      AppendToLine(S,False)
    end;
end;

procedure TMarkdownWriter.FinishElementPage(AElement: TPasElement; SkipDescription : Boolean = false);

var
  DocNode: TDocNode;

begin
  DocNode := Engine.FindDocNode(AElement);
  If Not Assigned(DocNode) then
    exit;
  // Description
  if Assigned(DocNode.Descr) and not SkipDescription then
    AppendDescrSection(AElement, DocNode.Descr, UTF8Encode(SDocDescription));

  // Append "Errors" section
  if Assigned(DocNode.ErrorsDoc) then
    AppendDescrSection(AElement, DocNode.ErrorsDoc, UTF8Encode(SDocErrors));

  // Append Version info
  if Assigned(DocNode.Version) then
    AppendDescrSection(AElement, DocNode.Version, UTF8Encode(SDocVersion));

  // Append "See also" section
  AppendSeeAlsoSection(AElement,DocNode);

  // Append examples, if present
  AppendExampleSection(AElement,DocNode);
  // Append notes, if present
  ConvertNotes(AElement,DocNode.Notes);
end;

procedure TMarkdownWriter.CreateTopicPageBody(AElement: TTopicElement);

var
  DocNode: TDocNode;
begin
  DocNode:=AElement.TopicNode;
  AppendTitle(AElement.Name);
  if not Assigned(DocNode) then  // should always be true, but we're being careful.
    exit;
  AppendShortDescr(AElement, DocNode);
  if Assigned(DocNode.Descr) then
     AppendDescrSection(AElement, DocNode.Descr, '');
  AppendSeeAlsoSection(AElement,DocNode);
  CreateTopicLinks(DocNode,AElement);
  AppendExampleSection(AElement,DocNode);
  ConvertNotes(AElement,DocNode.Notes);
end;

procedure TMarkdownWriter.CreateClassHierarchyPage(AddUnit : Boolean);
type
  TypeEN = (NPackage, NModule, NName);


  Procedure AddClassList;
  begin
    DescrBeginUnorderedList;
  end;

  Procedure EndClassList;
  begin
    DescrEndUnorderedList;
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
    DescrBeginListItem;
    AppendHyperLink(PE);
    PM:=PE.GetModule();
    if (PM<>Nil) then
      begin
      AppendText(' (');
      AppendHyperLink(PM);
      AppendText(')');
      end
    else
      AppendText(EN.Element.Name);
    if EN.ChildCount>0 then
      begin
      AddClassList;
      For I:=0 to EN.ChildCount-1 do
        AppendClass(EN.Children[i] as TPasElementNode);
      EndClassList;
      end;
    DescrEndListItem;
  end;

begin
  AddClassList;
  AppendClass(TreeClass.RootNode);
  EndClassList;
end;

procedure TMarkdownWriter.CreatePackageClassHierarchy;


Var
  S : String;

begin
  S:=Package.Name;
  If Length(S)>0 then
    Delete(S,1,1);
  AppendTitle(SDocPackageClassHierarchy, [S]);
  CreateClassHierarchyPage(True);
end;

procedure TMarkdownWriter.CreatePageBody(AElement: TPasElement; ASubpageIndex: Integer);
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

procedure TMarkdownWriter.CreateIndexPage(L : TStringList);

Var
  Lists  : Array['A'..'Z'] of TStringList;
  CL : TStringList;
  E : TPasElement;
  CCount,I,Rows,J,Index : Integer;
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
  CCount:=0;
  for C:='A' to 'Z' do
    If (Lists[C]<>Nil) then
      Inc(CCount);
  DescrBeginTable(CCount,False);
  DescrBeginTableHeadRow;
  for C:='A' to 'Z' do
    If (Lists[C]<>Nil) then
      begin
      DescrBeginTableCell;
      AppendLink(C,'#'+LowerCase(C));
      DescrendTableCell;
      end;
  DescrEndTableHeadRow;
  // Now emit all identifiers.
  For C:='A' to 'Z' do
    begin
    CL:=Lists[C];
    If CL<>Nil then
      begin
      AppendHeader(3,C);
      DescrBeginTable(IndexColCount,False);
      DescrBeginTableHeadRow;
      For I:=1 to IndexColCount do
         begin
         DescrBeginTableCell;
         AppendToLine('&nbsp; ',False);
         DescrEndTableCell;
         end;
      DescrEndTableHeadRow;
      // Determine number of rows needed
      Rows:=(CL.Count div IndexColCount);
      If ((CL.Count Mod IndexColCount)<>0) then
        Inc(Rows);
      // Fill rows  
      For I:=0 to Rows-1 do
        begin
        DescrBeginTableRow;
        For J:=0 to IndexColCount-1 do
          begin
          DescrBeginTableCell;
          Index:=(J*Rows)+I;
          If (Index<CL.Count) then
            begin
            S:=CL[Index];
            E:=TPasElement(CL.Objects[Index]);
            AppendHyperlink(E);
            end;
          end;
        DescrEndTableRow;
        end;
      end; // have List
    end;  // For C:=
  Finally
    for C:='A' to 'Z' do
      FreeAndNil(Lists[C]);
  end;  
end;


procedure TMarkdownWriter.AddModuleIdentifiers(AModule : TPasModule; L : TStrings);

begin
  if assigned(AModule.InterfaceSection) Then
   begin
      AddElementsFromList(L,AModule.InterfaceSection.Consts);
      AddElementsFromList(L,AModule.InterfaceSection.Types);
      AddElementsFromList(L,AModule.InterfaceSection.Functions);
      AddElementsFromList(L,AModule.InterfaceSection.Classes);
      AddElementsFromList(L,AModule.InterfaceSection.Variables);
      AddElementsFromList(L,AModule.InterfaceSection.ResStrings);
   end;
end;


procedure TMarkdownWriter.CreatePackageIndex;

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
    S:=Package.Name;
    If Length(S)>0 then
      Delete(S,1,1);
    AppendTitle(SDocPackageIndex, [S]);
    CreateIndexPage(L);
  Finally
    L.Free;
  end;
end;

procedure TMarkdownWriter.CreatePackagePageBody;
var
  DocNode: TDocNode;
  i: Integer;
  ThisModule: TPasModule;
  L : TStringList;

begin
  AppendTitle(SDocPackageTitle, [Copy(Package.Name, 2, 256)]);
  AppendShortDescr(Package);

  AppendHeader(2,SDocUnits);
  L:=TStringList.Create;
  Try
    L.Sorted:=True;
    // Sort modules.
    For I:=0 to Package.Modules.Count-1 do
      L.AddObject(TPasModule(Package.Modules[i]).Name,TPasModule(Package.Modules[i]));
    AppendTableHeader([SDocUnits,SDocDescription]);
    // Now create table.
    for i:=0 to L.Count - 1 do
      begin
      ThisModule := TPasModule(L.Objects[i]);
      DescrBeginTableRow;
      DescrBeginTableCell;
      AppendHyperlink(ThisModule);
      DescrEndTableCell;
      AppendShortDescrCell(ThisModule);
      DescrEndTableRow;
      end;
    DescrEndTable;
  Finally
    L.Free;
  end;

  DocNode := Engine.FindDocNode(Package);
  if Assigned(DocNode) then
    begin
    if Assigned(DocNode.Descr) then
       AppendDescrSection(nil, DocNode.Descr, UTF8Decode(SDocDescription));
    CreateTopicLinks(DocNode,Package);
    end;
end;

procedure TMarkdownWriter.CreateTopicLinks ( Node: TDocNode; PasElement: TPasElement ) ;

var
  DocNode: TDocNode;
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
        AppendHeader(2,SDocRelatedTopics);
        AppendTableHeader([SDocTopic,SDocDescription]);
        end;
      ThisTopic:=FindTopicElement(DocNode);
      if Assigned(ThisTopic) then
        begin
        DescrBeginTableRow;
        DescrBeginTableCell;
        AppendHyperlink(ThisTopic);
        DescrEndTableCell;
        AppendShortDescrCell(ThisTopic);
        DescrEndTableRow;
        end;
      end;
    DocNode:=DocNode.NextSibling;
    end;
  if not First then
    DescrEndTable;
end;

function TMarkdownWriter.GetFileBaseDir(aOutput: String): String;
begin
  Result:=inherited GetFileBaseDir(aOutput)+'docs'+pathdelim;
end;

procedure TMarkdownWriter.CreateModuleIndexPage(AModule: TPasModule);

Var
  L : TStringList;

begin
  L:=TStringList.Create;
  try
    AddModuleIdentifiers(AModule,L);
    AppendTitle(SDocModuleIndex, [AModule.Name]);
    CreateIndexPage(L);
  Finally
    L.Free;
  end;  
end;

procedure TMarkdownWriter.CreateModuleMainPageBody(AModule: TPasModule);

var
  i: Integer;
  UnitRef: TPasType;
  DocNode: TDocNode;
begin
  AppendTitle(SDocUnitTitle, [AModule.Name]);
  AppendShortDescr(AModule);

  if AModule.InterfaceSection.UsesList.Count > 0 then
    begin
    AppendTableHeader(['Uses unit',SDocDescription]);
    for i := 0 to AModule.InterfaceSection.UsesList.Count - 1 do
      begin
      UnitRef := TPasType(AModule.InterfaceSection.UsesList[i]);
      DocNode := Engine.FindDocNode(UnitRef);
      if Assigned(DocNode) and DocNode.IsSkipped then
        continue;
      DescrBeginTableRow;
      DescrBeginTableCell;
      AppendHyperlink(UnitRef);
      DescrEndTableCell;
      AppendShortDescrCell(UnitRef);
      end;
    DescrEndTable;
    end;
  DocNode := Engine.FindDocNode(AModule);
  if Assigned(DocNode) then
    begin
    if Assigned(DocNode.Descr) then
      AppendDescrSection(AModule, DocNode.Descr, UTF8Decode(SDocOverview));
    ConvertNotes(AModule,DocNode.Notes);
    CreateTopicLinks(DocNode,AModule);
    end;
end;


procedure TMarkdownWriter.CreateSimpleSubpage(aModule : TPasModule; const ATitle,aItem: String; AList: TFPList);
var
  i: Integer;
  Decl: TPasElement;
  SortedList: TFPList;
  DocNode: TDocNode;
begin
  AppendTitle(SDocUnitTitle + ': %s', [AModule.Name, aTitle]);
  SortedList := TFPList.Create;
  try
    for i := 0 to AList.Count - 1 do
      begin
      Decl := TPasElement(AList[i]);
      DocNode := Engine.FindDocNode(Decl);
      if not (Assigned(DocNode) and DocNode.IsSkipped) then
        SortedList.Add(Decl);
      end;
    SortedList.Sort(@SortPasElements);
    AppendTableHeader([aItem,SDocDescription]);
    for i := 0 to SortedList.Count - 1 do
      begin
      Decl:=TPasElement(SortedList[i]);
      DescrBeginTableRow;
      DescrBeginTableCell;
      AppendHyperlink(Decl);
      DescrEndTableCell;
      AppendShortDescrCell(Decl);
      DescrEndTableRow;
      end;
    DescrEndTable;
  finally
    SortedList.Free;
  end;
end;


procedure TMarkdownWriter.CreateResStringsPage(aModule : TPasModule);

var
  i: Integer;
  Decl: TPasResString;

begin
  AppendTitle(SDocUnitTitle + ': %s', [AModule.Name, SDocResStrings]);
  If AModule.InterfaceSection.ResStrings.Count = 0 then
    exit;
  for i := 0 to AModule.InterfaceSection.ResStrings.Count - 1 do
    begin
    Decl := TPasResString(AModule.InterfaceSection.ResStrings[i]);
    AppendToLine('<a name="%s"> %s',[LowerCase(Decl.Name),Decl.Name]);
    Indent;
    UTF8Decode(Decl.Expr.getDeclaration(true));
    undent;
    end;
end;


procedure TMarkdownWriter.CreateModulePageBody(AModule: TPasModule;
  ASubpageIndex: Integer);

begin
  case ASubpageIndex of
    0:
      CreateModuleMainPageBody(aModule);
    ResstrSubindex:
      CreateResStringsPage(aModule);
    ConstsSubindex:
      CreateSimpleSubpage(aModule,SDocConstants, SDocConstant, AModule.InterfaceSection.Consts);
    TypesSubindex:
      CreateSimpleSubpage(aModule,SDocTypes, SDocType, AModule.InterfaceSection.Types);
    ClassesSubindex:
      CreateSimpleSubpage(aModule,SDocClasses, SDocClass, AModule.InterfaceSection.Classes);
    ProcsSubindex:
      CreateSimpleSubpage(aModule,SDocProceduresAndFunctions, SDocProcedureOrFunction, AModule.InterfaceSection.Functions);
    VarsSubindex:
      CreateSimpleSubpage(aModule,SDocVariables, SDocVariable, AModule.InterfaceSection.Variables);
    IndexSubIndex: 
      CreateModuleIndexPage(AModule);
  end;
end;

procedure TMarkdownWriter.CreateConstPageBody(AConst: TPasConst);

begin
  AppendTitle(AConst.Name);
  AppendShortDescr(AConst);

  AppendHeader(2,SDocDeclaration);
  AppendSourceRef(AConst);

  DescrBeginCode(False,'Pascal');
  EmitLine('const');
  EmitLine(aConst.GetDeclaration(True));
  DescrEndCode;

  FinishElementPage(AConst);
end;


procedure TMarkdownWriter.CreateTypePageBody(AType: TPasType);
begin
  AppendTitle(AType.Name);
  AppendShortDescr(AType);
  AppendHeader(2,SDocDeclaration);
  AppendSourceRef(AType);

  DescrBeginCode(False,'Pascal');
  EmitLine('Type');
  EmitLine(aType.GetDeclaration(True));
  DescrEndCode;

  FinishElementPage(AType);
end;




procedure TMarkdownWriter.CreateMemberDeclarations(AParent: TPasElement; Members: TFPList; Options: TMemberListOptions);

  function GetMemberType(aMember : TPasElement) : string;

  begin
    if aMember is TPasProcedure then
      Result:=SDocMethod
    else if aMember is TPasProperty then
      Result:=SDocProperty
    else if aMember is TPasConst then
      Result:=SDocConstant
    else if aMember is TPasType then
      Result:=SDocType
    else
      Result:=SDocField;
  end;

var
  Member: TPasElement;
  MVisibility : TPasMemberVisibility;
  i,aCount: Integer;
  // isRecord,
  isOverLoad : Boolean;

begin
  // isRecord:=AParent is TPasRecordType;
  if Members.Count = 0 then
    begin
    AppendText(SDocNoneAVailable);
    Exit;
    end;
  if mloAppendType in Options then
    AppendTableHeader([SDocMember,SDocType,SDocVisibility,SDocDescription])
  else
    AppendTableHeader([SDocMember,SDocVisibility,SDocDescription]);
  aCount:=0;
  Members.Sort(@SortPasElements);
  for i := 0 to Members.Count - 1 do
    begin
    Member := TPasElement(Members[i]);
    MVisibility:=Member.Visibility;
    if mloCheckVisibility in Options then
      if not Engine.ShowElement(Member) then
        Continue;
    isOverLoad:=(Member is TPasOverloadedProc);
    if isOverload then
      Member:=TPasElement((Member as TPasOverloadedProc).Overloads[0]);
    Inc(aCount);
    DescrBeginTableRow;

    DescrBeginTableCell;
    AppendHyperlink(Member);
    if mloAppendParent in options then
      begin
      AppendText('(');
      AppendHyperLink(Member.Parent);
      AppendText(')');
      end;
    DescrEndTableCell;
    if mloAppendType in Options then
      begin
      DescrBeginTableCell;
      AppendText(GetMemberType(Member));
      DescrEndTableCell;
      end;

    DescrBeginTableCell;
    AppendText(VisibilityNames[MVisibility]);
    DescrEndTableCell;
    AppendShortDescrCell(member);
    DescrEndTableRow;
    end;
  DescrEndTable;
  if ACount=0 then
    AppendText(SDocNoneAVailable)
end;

procedure TMarkdownWriter.AppendTitle(const S: String);
begin
  AddMetaData('title',S);
  AppendHeader(1,S);
  EnsureEmptyLine;
end;

procedure TMarkdownWriter.AppendTitle(const Fmt: String;
  const Args: array of const);

begin
  AppendTitle(Format(Fmt,Args));
end;

function TMarkdownWriter.GetClassDeclarationFirstLine(aEl: TPasClassType): String;

Var
  aLine : string;
  I : Integer;

begin
  aLine:=aEL.Name;
  if aEl.GenericTemplateTypes.Count>0 then
    begin
    aLine:='generic '+aLine+'<';
    For I:=0 to  aEl.GenericTemplateTypes.Count-1 do
      begin
      if I>0 then
        aLine:=aLine+', ';
      aLine:=aLine+TPasGenericTemplateType(aEl.GenericTemplateTypes[i]).Name;
      end;
    aLine:=aLine+'>';
    end;
  aLine:=aLine+' = '+aEl.ElementTypeName;
  if aEl.HelperForType<>Nil then
    aLine:=aLine+' for '+aEl.HelperForType.Name;
  if aEL.ExternalName<>'' then
    aLine:=aLine+' external name '''+ael.ExternalName+'''';
  if Assigned(aEL.AncestorType) then
    if (aEL.AncestorType is TPasSpecializeType) then
    begin
      aLine:=aLine+'(specialize ';
      aLine:=aLine+ TPasSpecializeType(aEL.AncestorType).DestType.Name;
      aLine:=aLine+ '<,>)';
    end
      else
    begin
      aLine:=aLine+' ('+ael.AncestorType.Name;
      if Assigned(ael.Interfaces) and (aEl.Interfaces.Count>0) then
        For I:=0 to aEl.Interfaces.Count-1 do
          aLine:=aLine+', '+TPasElement(aEl.Interfaces[i]).Name;
      aLine:=aLine+')';
    end;
  if Assigned(aEl.GUIDExpr) then
    aLine:=aLine+' ['+aEl.GUIDExpr.GetDeclaration(True)+']';
  Result:=aLine;
end;

function TMarkdownWriter.GetClassDeclaration(aEl: TPasClassType): String;

Var
  S,T : TStrings;
  I,J : Integer;
  LastVis : TPasMemberVisibility;
  aMember : TPasElement;

begin
  T:=Nil;
  lastVis:=VisDefault;
  S:=TStringList.Create;
  try
    T:=TStringList.Create;
    S.Add(GetClassDeclarationFirstLine(aEl));
    for I:=0 to aEl.Members.Count-1 do
      begin
      aMember:=TPasElement(aEl.Members[i]);
      if aMember.Visibility<>LastVis then
        begin
        LastVis:=aMember.Visibility;
        S.Add(VisibilityNames[LastVis]);
        end;
      T.Text:=GetDeclaration(aMember);
      for J:=0 to T.count-1 do
        S.Add('  '+T[J]);
      end;
    if not aEl.IsShortDefinition then
      S.Add('end');
    Result:=S.Text;
  finally
    S.Free;
    T.Free;
  end;
end;

function TMarkdownWriter.GetDeclaration(aEl: TPasElement): String;

Var
  I : Integer;
  Ovl : TPasOverloadedProc;
  S : String;
begin
  if (aEl is TPasClassType) then
    exit(GetClassDeclaration(TPasClassType(aEl))+';');
  if Not (aEl is TPasOverloadedProc) then
    Exit(aEl.GetDeclaration(True)+';');
  ovl:=aEl as TPasOverloadedProc;
  S:='';
  for I:=0 to ovl.Overloads.Count-1 do
    begin
    if s<>'' then
       S:=S+sLineBreak;
    S:=S+TPasElement(Ovl.Overloads[i]).GetDeclaration(True)+';';
    end;
  Result:=S;
end;

procedure TMarkdownWriter.AppendDeclaration(aEl : TPasElement; aKind : string; PrependVisibility : Boolean);

Var
  S : String;
begin
  DescrBeginCode(False,'Pascal');
  if PrependVisibility then
    S:=VisibilityNames[aEL.Visibility]+' '
  else
    S:='';
  S:=S+aKind;
  EmitLine(S);
  S:=GetDeclaration(aEl);
  EmitCode(S,2);
  DescrEndCode;
end;


procedure TMarkdownWriter.CreateClassMainPage(aClass : TPasClassType);

  procedure AppendMemberListLink(AListSubpageIndex: Integer;  const AText: String);
  begin
    AppendToLine('\[',False);
    AppendLink(aText,FixHtmlPath(ResolveLinkWithinPackage(AClass, AListSubpageIndex)));
    AppendText(' (');
    AppendLink(SDocByName,FixHtmlPath(ResolveLinkWithinPackage(AClass, AListSubpageIndex+1)));
    AppendToLine(')\]',False);
  end;



var
  i: Integer;
  ThisInterface,
  ThisClass: TPasType;
  ThisTreeNode: TPasElementNode;
  DocNode: TDocNode;

begin
  DocNode := Engine.FindDocNode(aClass);

  //WriteLn('@ClassPageBody.CreateMainPage Class=', AClass.Name);
  AppendTitle(AClass.Name);

  AppendMemberListLink(PropertiesByInheritanceSubindex,SDocProperties);
  AppendMemberListLink(MethodsByInheritanceSubindex, SDocMethods);
  AppendMemberListLink(EventsByInheritanceSubindex, SDocEvents);

  EnsureEmptyLine;

  AppendShortDescr(AClass);
  AppendHeader(2,SDocDeclaration);
  AppendSourceRef(AClass);

  AppendDeclaration(aClass,'Type',False);

  // Description
  if Assigned(DocNode) and Assigned(DocNode.Descr) then
    AppendDescrSection(aClass, DocNode.Descr, SDocDescription);

  AppendHeader(2,SDocMembers);

  CreateMemberDeclarations(aClass, AClass.Members,[mloAppendType]);

  AppendHeader(2,SDocInheritance);

  EnsureEmptyLine;

  AppendTableHeader([SDocClass,SDocDescription]);

  ThisClass := AClass;
  ThisTreeNode := Nil;

  if AClass.ObjKind = okInterface then
    ThisTreeNode := TreeInterface.GetPasElNode(AClass)
  else
    ThisTreeNode := TreeClass.GetPasElNode(AClass);
  while True do
    begin
    DescrBeginTableRow;
    DescrBeginTableCell;
    // Show class item
    if Assigned(ThisClass) Then
      AppendHyperlink(ThisClass);
    if Assigned(ThisClass) and (AClass.Interfaces.count>0) then
      begin
      AppendText('(');
      for i:=0 to AClass.interfaces.count-1 do
        begin
        ThisInterface:= TPasType(AClass.Interfaces[i]);
        if I>0 then
          AppendText(', ');
        AppendHyperlink( ThisInterface);
        end;
      AppendText(')');
      end;
    DescrEndTableCell;
    AppendShortDescrCell(ThisClass);
    DescrEndTableRow;

    if Not Assigned(ThisTreeNode) then
      Break
    else if not Assigned(ThisTreeNode.ParentNode) then
      begin
      ThisClass := nil;
      ThisTreeNode:= nil;
      break;
      end
    else
      begin
      DescrBeginTableRow;
      DescrBeginTableCell;
      AppendText('|');
      DescrEndTableCell;
      DescrBeginTableCell;
      DescrEndTableCell;
      ThisClass := ThisTreeNode.ParentNode.Element;
      ThisTreeNode := ThisTreeNode.ParentNode;
      end;
    end;
  DescrEndTable;
  FinishElementPage(AClass,True);
end;


procedure TMarkdownWriter.CreateInheritanceSubpage(aClass: TPasClassType; aTitle: string; AFilter: TMemberFilter);

var
  ThisClass: TPasClassType;
  i,aCount: Integer;
  Member: TPasElement;
  aList : TFPList;

begin
  aTitle:=aClass.Name+' : '+aTitle+ ' '+SDocByInheritance;
  AppendTitle(aTitle);
  ThisClass := AClass;
  aCount:=0;
  aList:=TFPList.Create;
  try
    while assigned(ThisClass) do
      begin
      aList.Clear;
      for i := 0 to ThisClass.Members.Count - 1 do
        begin
        Member := TPasElement(ThisClass.Members[i]);
        if (Engine.ShowElement(Member) and AFilter(Member)) then
          aList.Add(Member);
        end;
      aCount:=aCount+aList.Count;
      if AList.Count>0 then
        begin
        AppendHeader(2,CreateHyperLink(ThisClass),False);
        CreateMemberDeclarations(aClass, aList, []);
        end;
      if ThisClass.AncestorType is TPasClassType then
        ThisClass := TPasClassType(ThisClass.AncestorType)
      else
        ThisClass:=Nil;
      end;
    if aCount=0 then
      AppendText(SDocNoneAVailable);
  finally
    aList.Free;
  end;
end;

procedure TMarkdownWriter.CreateSortedSubpage(ACLass: TPasClassType; aTitle: string; AFilter: TMemberFilter);

var
  List: TFPList;
  ThisClass: TPasClassType;
  i : Integer;
  Member: TPasElement;

begin
  aTitle:=aClass.Name+' : '+aTitle+' '+SDocByName;
  AppendTitle(aTitle);

  List := TFPList.Create;
  try
    ThisClass := AClass;
    while Assigned(ThisClass) do
      begin
      for i := 0 to ThisClass.Members.Count - 1 do
        begin
        Member := TPasElement(ThisClass.Members[i]);
        if Engine.ShowElement(Member) and AFilter(Member) then
          List.Add(Member)
        end;
      if (ThisClass.AncestorType is TPasClassType) then
        ThisClass := TPasClassType(ThisClass.AncestorType)
      else
        ThisClass := Nil;
      end;
    CreateMemberDeclarations(aClass, List, [mloAppendParent]);
  finally
    List.Free;
  end;
end;

function TMarkdownWriter.GetAdditionalConfig: TStrings;
begin
  if FAdditionalConfig=Nil then
    FAdditionalConfig:=TstringList.Create;
  Result:=FAdditionalConfig;
end;

procedure TMarkdownWriter.CreateClassPageBody(AClass: TPasClassType;
  ASubpageIndex: Integer);


begin
  case ASubpageIndex of
    0:
      CreateClassMainPage(aClass);
    PropertiesByInheritanceSubindex:
      CreateInheritanceSubpage(aClass,SDocPropertyOverview,@PropertyFilter);
    PropertiesByNameSubindex:
      CreateSortedSubpage(aClass,SDocPropertyOverview, @PropertyFilter);
    MethodsByInheritanceSubindex:
      CreateInheritanceSubpage(aClass,SDocMethodOverview,@MethodFilter);
    MethodsByNameSubindex:
      CreateSortedSubpage(aClass,SDocMethodOverview,@MethodFilter);
    EventsByInheritanceSubindex:
      CreateInheritanceSubpage(aClass,SDocEventOverview,@EventFilter);
    EventsByNameSubindex:
      CreateSortedSubpage(aClass,SDocEventOverview, @EventFilter);
  end;
end;

procedure TMarkdownWriter.CreateClassMemberPageBody(AElement: TPasElement);

  procedure CreateVarPage(Element: TPasVariable);
  begin
    AppendDeclaration(Element,'Var',True);
  end;

  procedure CreateTypePage(Element: TPasType);
  begin
    AppendDeclaration(Element,'Type',True);
  end;

  procedure CreateConstPage(Element: TPasConst);
  begin
    AppendDeclaration(Element,'Const',True);
  end;

  procedure CreatePropertyPage(Element: TPasProperty);

  begin
    AppendDeclaration(Element,'Property',True);
  end;

var
  s: String;

begin
  AppendTitle(aElement.FullName);
  AppendShortDescr(AElement);
  AppendHeader(2, SDocDeclaration);
  AppendSourceRef(AElement);

  if AElement is TPasProperty then
    S:='Property'
  else if AElement is TPasConst then
    S:='Const'
  else if (AElement is TPasVariable) then
    S:='var'
  else if AElement is TPasProcedureBase then
    s:=''
  else if AElement is TPasType then
    S:='Type'
  else
    AppendText('<' + AElement.ClassName + '>');

  AppendDeclaration(aElement,S,True);

  FinishElementPage(AElement);
end;

procedure TMarkdownWriter.CreateVarPageBody(AVar: TPasVariable);

begin
  AppendTitle(AVar.FullName);
  AppendShortDescr(AVar);
  AppendHeader(2, SDocDeclaration);
  AppendSourceRef(AVar);

  AppendDeclaration(aVar,'var',false);

  FinishElementPage(AVar);
end;

procedure TMarkdownWriter.CreateProcPageBody(AProc: TPasProcedureBase);
begin

  AppendTitle(AProc.Name);
  AppendShortDescr(AProc);
  AppendHeader(2,SDocDeclaration);
  AppendSourceRef(AProc);

  AppendDeclaration(AProc,'',False);

  FinishElementPage(AProc);
end;

function TMarkdownWriter.InterPretOption ( const Cmd, Arg: String ) : boolean;

  procedure ReadFile(aStrings : TStrings; aFileName : string);

  begin
    aFileName:= SetDirSeparators(aFileName);
    if copy(aFileName,1,1)<>'@' then
      aStrings.text:=aFileName
    else
      begin
      Delete(aFileName,1,1);
      aStrings.LoadFromFile(aFileName);
      end;
  end;

begin
  Result:=True;
  if Cmd = '--footer' then
    ReadFile(FooterMarkDown,Arg)
  else if Cmd = '--header' then
    ReadFile(HeaderMarkDown,Arg)
  else if Cmd = '--index-colcount' then
    IndexColCount := StrToIntDef(Arg,IndexColCount)
  else if Cmd = '--image-url' then
    FBaseImageURL  := Arg
  else if Cmd = '--theme' then
    begin
    if arg='-' then
      Theme:=''
    else
      Theme  := Arg
    end
  else if Cmd='--navigation' then
    begin
    if SameText(Arg,'UnitSubTree') then
      FNavigationMode:=nmUnitSubTree
    else if SameText(Arg,'UnitTree') then
      FNavigationMode:=nmUnitTree;
    end
  else
    Result:=inherited InterPretOption(Cmd, Arg);
end;

class procedure TMarkdownWriter.Usage(List: TStrings);
begin
  inherited Usage(List);
  List.add('--header=file');
  List.Add(SMDUsageHeader);
  List.add('--footer=file');
  List.Add(SMDUsageFooter);
  List.Add('--index-colcount=N');
  List.Add(SMDIndexColcount);
  List.Add('--image-url=url');
  List.Add(SMDImageUrl);
  List.Add('--theme=name');
  List.Add(SMDTheme);
  List.Add('--navigation=scheme');
  List.Add(SMDNavigation);
  // we have to write even count of params into list either we will have a exception
  List.Add('');
  List.Add(SMDNavSubtree);
  List.Add('');
  List.Add(SMDNavTree);
end;

class procedure TMarkdownWriter.SplitImport(var AFilename, ALinkPrefix: String);
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

class function TMarkdownWriter.FileNameExtension: String;
begin
  result:='md';
end;

// private methods

function TMarkdownWriter.GetPageCount: Integer;
begin
  Result := PageInfos.Count;
end;

procedure TMarkdownWriter.SetOnTest(const AValue: TNotifyEvent);
begin
  if FOnTest=AValue then exit;
    FOnTest:=AValue;
end;


procedure TMarkdownWriter.AppendText(const S: String);
begin
  AppendToLine(S,True);
end;


initialization
  // Do not localize.
  RegisterWriter(TMarkdownWriter,'md','Markdown output.');

finalization
  UnRegisterWriter('md');
end.
