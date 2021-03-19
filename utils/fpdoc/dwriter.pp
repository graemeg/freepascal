{

    FPDoc  -  Free Pascal Documentation Tool
    Copyright (C) 2000 - 2003 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org
    2005-2012 by
      various FPC contributors

    * Output string definitions
    * Basic writer (output generator) class

    See the file COPYING, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}


unit dWriter;

{$MODE objfpc}
{$H+}
{$WARN 5024 off : Parameter "$1" not used}
interface

uses Classes, DOM, contnrs, dGlobals, PasTree, SysUtils, fpdocclasstree;


type
  // Phony element for pas pages.

  TTopicElement = Class(TPaselement)
    TopicNode : TDocNode;
    Previous,
    Next : TPasElement;
    Subtopics : TList;
    Constructor Create(const AName: String; AParent: TPasElement); override;
    Destructor Destroy; override;
  end;

  { TFileAllocator }

  TFileAllocator = class
  private
    FSubPageNames: Boolean;
  protected
    function GetFilePostfix(ASubindex: Integer):String;
  public
    procedure Create(); overload;
    procedure AllocFilename(AElement: TPasElement; ASubindex: Integer); virtual;
    function GetFilename(AElement: TPasElement;
      ASubindex: Integer): String; virtual; abstract;
    function GetRelativePathToTop(AElement: TPasElement): String; virtual;
    function GetCSSFilename(ARelativeTo: TPasElement): DOMString; virtual;
    property SubPageNames: Boolean read FSubPageNames write FSubPageNames;
  end;

  TLongNameFileAllocator = class(TFileAllocator)
  private
    FExtension: String;
  public
    constructor Create(const AExtension: String);
    function GetFilename(AElement: TPasElement; ASubindex: Integer): String; override;
    function GetRelativePathToTop(AElement: TPasElement): String; override;
    property Extension: String read FExtension;
  end;


  TWriterLogEvent = Procedure(Sender : TObject; Const Msg : String) of object;
  TWriterNoteEvent = Procedure(Sender : TObject; Note : TDomElement; Var EmitNote : Boolean) of object;
  
  { TFPDocWriter }

  TFPDocWriter = class
  private
    FEmitNotes: Boolean;
    FEngine  : TFPDocEngine;
    FPackage : TPasPackage;
    FContext : TPasElement;
    FTopics  : TList;
    FImgExt : String;
    FBeforeEmitNote : TWriterNoteEvent;
    procedure ConvertURL(AContext: TPasElement; El: TDOMElement);
    procedure CreateClassTree;
  protected
    TreeClass: TClassTreeBuilder;      // Global class tree
    TreeInterface: TClassTreeBuilder;  // Global interface tree

    procedure AddElementsFromList(L: TStrings; List: TFPList; UsePathName : Boolean = False);
    Procedure DoLog(Const Msg : String);
    Procedure DoLog(Const Fmt : String; Args : Array of const);
    Procedure OutputResults(); virtual;
    procedure Warning(AContext: TPasElement; const AMsg: String);
    procedure Warning(AContext: TPasElement; const AMsg: String;
      const Args: array of const);

    // function FindShortDescr(const Name: String): TDOMElement;

    // Description conversion
    function IsDescrNodeEmpty(Node: TDOMNode): Boolean;
    function IsExtShort(Node: TDOMNode): Boolean;
    function ConvertShort(AContext: TPasElement; El: TDOMElement): Boolean;
    function ConvertNotes(AContext: TPasElement; El: TDOMElement): Boolean; virtual;
    function ConvertBaseShort(AContext: TPasElement; Node: TDOMNode): Boolean;
    procedure ConvertBaseShortList(AContext: TPasElement; Node: TDOMNode;
      MayBeEmpty: Boolean);
    procedure ConvertLink(AContext: TPasElement; El: TDOMElement);
    function ConvertExtShort(AContext: TPasElement; Node: TDOMNode): Boolean;
    procedure ConvertDescr(AContext: TPasElement; El: TDOMElement;
      AutoInsertBlock: Boolean);
    function ConvertNonSectionBlock(AContext: TPasElement;
      Node: TDOMNode): Boolean;
    procedure ConvertExtShortOrNonSectionBlocks(AContext: TPasElement;
      Node: TDOMNode);
    function ConvertSimpleBlock(AContext: TPasElement; Node: TDOMNode): Boolean;
    Function FindTopicElement(Node : TDocNode): TTopicElement;
    Procedure ConvertImage(El : TDomElement);

    Procedure DescrEmitNotesHeader(AContext : TPasElement); virtual;
    Procedure DescrEmitNotesFooter(AContext : TPasElement); virtual;
    procedure DescrWriteText(const AText: DOMString); virtual; abstract;
    procedure DescrBeginBold; virtual; abstract;
    procedure DescrEndBold; virtual; abstract;
    procedure DescrBeginItalic; virtual; abstract;
    procedure DescrEndItalic; virtual; abstract;
    procedure DescrBeginUnderline; virtual; abstract;
    procedure DescrEndUnderline; virtual; abstract;
    procedure DescrBeginEmph; virtual; abstract;
    procedure DescrEndEmph; virtual; abstract;
    procedure DescrWriteImageEl(const AFileName, ACaption,ALinkName : DOMString); virtual; 
    procedure DescrWriteFileEl(const AText: DOMString); virtual; abstract;
    procedure DescrWriteKeywordEl(const AText: DOMString); virtual; abstract;
    procedure DescrWriteVarEl(const AText: DOMString); virtual; abstract;
    procedure DescrBeginLink(const AId: DOMString); virtual; abstract;
    procedure DescrEndLink; virtual; abstract;
    procedure DescrBeginURL(const AURL: DOMString); virtual; abstract;
    procedure DescrEndURL; virtual; abstract;
    procedure DescrWriteLinebreak; virtual; abstract;
    procedure DescrBeginParagraph; virtual; abstract;
    procedure DescrEndParagraph; virtual; abstract;
    procedure DescrBeginCode(HasBorder: Boolean; const AHighlighterName: String); virtual; abstract;
    procedure DescrWriteCodeLine(const ALine: String); virtual; abstract;
    procedure DescrEndCode; virtual; abstract;
    procedure DescrBeginOrderedList; virtual; abstract;
    procedure DescrEndOrderedList; virtual; abstract;
    procedure DescrBeginUnorderedList; virtual; abstract;
    procedure DescrEndUnorderedList; virtual; abstract;
    procedure DescrBeginDefinitionList; virtual; abstract;
    procedure DescrEndDefinitionList; virtual; abstract;
    procedure DescrBeginListItem; virtual; abstract;
    procedure DescrEndListItem; virtual; abstract;
    procedure DescrBeginDefinitionTerm; virtual; abstract;
    procedure DescrEndDefinitionTerm; virtual; abstract;
    procedure DescrBeginDefinitionEntry; virtual; abstract;
    procedure DescrEndDefinitionEntry; virtual; abstract;
    procedure DescrBeginSectionTitle; virtual; abstract;
    procedure DescrBeginSectionBody; virtual; abstract;
    procedure DescrEndSection; virtual; abstract;
    procedure DescrBeginRemark; virtual; abstract;
    procedure DescrEndRemark; virtual; abstract;
    procedure DescrBeginTable(ColCount: Integer; HasBorder: Boolean); virtual; abstract;
    procedure DescrEndTable; virtual; abstract;
    procedure DescrBeginTableCaption; virtual; abstract;
    procedure DescrEndTableCaption; virtual; abstract;
    procedure DescrBeginTableHeadRow; virtual; abstract;
    procedure DescrEndTableHeadRow; virtual; abstract;
    procedure DescrBeginTableRow; virtual; abstract;
    procedure DescrEndTableRow; virtual; abstract;
    procedure DescrBeginTableCell; virtual; abstract;
    procedure DescrEndTableCell; virtual; abstract;
    procedure PrepareDocumentation; virtual;
    // Descendents must override this.
    procedure DoWriteDocumentation; virtual; Abstract;

    Property CurrentContext : TPasElement Read FContext ;
  public
    Constructor Create(APackage: TPasPackage; AEngine: TFPDocEngine); virtual;
    destructor Destroy;  override;
    procedure AddModuleIdentifiers(AModule: TPasModule; L: TStrings);
    property Engine : TFPDocEngine read FEngine;
    Property Package : TPasPackage read FPackage;
    Property Topics : TList Read FTopics;
    Property ImageExtension : String Read FImgExt Write FImgExt;
    // Should return True if option was succesfully interpreted.
    Function InterpretOption(Const Cmd,Arg : String) : Boolean; Virtual;
    Class Function FileNameExtension : String; virtual;
    Class Procedure Usage(List : TStrings); virtual;
    Class procedure SplitImport(var AFilename, ALinkPrefix: String); virtual;
    // Here we start the generation of documentation
    procedure WriteDocumentation;
    Function WriteDescr(Element: TPasElement) : TDocNode;
    procedure WriteDescr(Element: TPasElement; DocNode: TDocNode);
    procedure WriteDescr(AContext: TPasElement; DescrNode: TDOMElement); virtual;
    Procedure FPDocError(Msg : String);
    Procedure FPDocError(Fmt : String; Args : Array of Const);
    Function  ShowMember(M : TPasElement) : boolean;
    Procedure GetMethodList(ClassDecl: TPasClassType; List : TStringList);
    Property EmitNotes : Boolean Read FEmitNotes Write FEmitNotes;
    Property BeforeEmitNote : TWriterNoteEvent Read FBeforeEmitNote Write FBeforeEmitNote;
  end;

const
  // The Multi-Page doc writer identifies each page by it's index.
  IdentifierIndex = 0;

  // Subpage indices for modules
  ResstrSubindex = 1;
  ConstsSubindex = 2;
  TypesSubindex = 3;
  ClassesSubindex = 4;
  ProcsSubindex = 5;
  VarsSubindex = 6;
  // Maybe needed later for topic overview ??
  TopicsSubIndex = 7;
  IndexSubIndex = 8;
  ClassHierarchySubIndex = 9;
  InterfaceHierarchySubIndex = 10;

  // Subpage indices for classes
  PropertiesByInheritanceSubindex = 11;
  PropertiesByNameSubindex = 12;
  MethodsByInheritanceSubindex = 13;
  MethodsByNameSubindex = 14;
  EventsByInheritanceSubindex = 15;
  EventsByNameSubindex = 16;


Type
  { TMultiFileDocWriter }

  { TPageInfo }

  TPageInfo = class
  Public
    Element: TPasElement;
    SubpageIndex: Integer;
    Constructor Create(aElement : TPasElement; aIndex : Integer);
  end;

  { TLinkData }

  TLinkData = Class(TObject)
    FPathName,
    FLink,
    FModuleName : String;
    Constructor Create(Const APathName,ALink,AModuleName : string);
  end;

  TMultiFileDocWriter = Class(TFPDocWriter)
  Private
    FSubPageNames: Boolean;
    FBaseDirectory: String;
    FCurDirectory: String;
    FModule: TPasModule;
    FPageInfos: TFPObjectList;     // list of TPageInfo objects
    FLinkUnresolvedCnt: Integer;
    FOutputPageNames: TStringList;
    function GetOutputPageNames: TStrings;
    function GetPageCount: Integer;
    function LinkFix(ALink:String):String;
  Protected
    FAllocator: TFileAllocator;
    Procedure LinkUnresolvedInc();
    // General resolving routine
    function ResolveLinkID(const Name: String): DOMString;
    // Simplified resolving routine. Excluded last path after dot
    function ResolveLinkIDUnStrict(const Name: String): DOMString;
    function ResolveLinkIDInUnit(const Name,AUnitName: String): DOMString;
    function ResolveLinkWithinPackage(AElement: TPasElement; ASubpageIndex: Integer): String;
    procedure PrepareDocumentation; override;
    function CreateAllocator() : TFileAllocator; virtual; abstract;
    Procedure OutputResults(); override;
    // aFileName is the filename allocated by the Allocator, nothing prefixed.
    procedure WriteDocPage(const aFileName: String; aElement: TPasElement; aSubPageIndex: Integer); virtual; abstract;
    procedure AllocatePages; virtual;
    // Default page allocation mechanism.
    function AddPage(AElement: TPasElement; ASubpageIndex: Integer): TPageInfo; virtual;
    procedure AddPages(AElement: TPasElement; ASubpageIndex: Integer; AList: TFPList);  virtual;
    procedure AddTopicPages(AElement: TPasElement);   virtual;
    procedure AllocateClassMemberPages(AModule: TPasModule; LinkList: TObjectList); virtual;
    procedure AllocateModulePages(AModule: TPasModule; LinkList: TObjectList); virtual;
    procedure AllocatePackagePages; virtual;
    // Prefix every filename generated with the result of this.
    function GetFileBaseDir(aOutput: String): String; virtual;
    function InterPretOption(const Cmd, Arg: String): boolean; override;
    function  ModuleHasClasses(AModule: TPasModule): Boolean;
    // Allocate pages etc.
    Procedure DoWriteDocumentation; override;
    Function MustGeneratePage(aFileName : String) : Boolean; virtual;

    Property PageInfos : TFPObjectList Read FPageInfos;
    Property SubPageNames: Boolean Read FSubPageNames;
  Public
    constructor Create(APackage: TPasPackage; AEngine: TFPDocEngine); override;
    Destructor Destroy; override;
    class procedure Usage(List: TStrings); override;
    property PageCount: Integer read GetPageCount;
    Property Allocator : TFileAllocator Read FAllocator;
    Property Module: TPasModule  Read FModule Write FModule;
    Property CurDirectory: String Read FCurDirectory Write FCurDirectory;    // relative to curdir of process
    property BaseDirectory: String read FBaseDirectory Write FBaseDirectory; // relative path to package base directory
    Property OutputPageNames : TStrings Read GetOutputPageNames;
 end;

  TFPDocWriterClass = Class of TFPDocWriter;
  EFPDocWriterError = Class(Exception);

// Member Filter Callback type
  TMemberFilter = function(AMember: TPasElement): Boolean;

//  Filter Callbacks
function PropertyFilter(AMember: TPasElement): Boolean;
function MethodFilter(AMember: TPasElement): Boolean;
function EventFilter(AMember: TPasElement): Boolean;


// Register backend
Procedure RegisterWriter(AClass : TFPDocWriterClass; Const AName,ADescr : String);
// UnRegister backend
Procedure UnRegisterWriter(Const AName : String);
// Return back end class. Exception if not found.
Function  GetWriterClass(AName : String) : TFPDocWriterClass;
// Return index of back end class.
Function  FindWriterClass(AName : String) : Integer;
// List of backend in name=descr form.
Procedure EnumWriters(List : TStrings);
// Sort elements on name
function SortPasElements(Item1, Item2: Pointer): Integer;


implementation

uses strutils, fpdocstrs;

function SortPasElements(Item1, Item2: Pointer): Integer;
begin
  Result:=CompareText(TPasElement(Item1).Name,TPasElement(Item2).Name)
end;



{ ---------------------------------------------------------------------
  Filter callbacks
  ---------------------------------------------------------------------}


function PropertyFilter(AMember: TPasElement): Boolean;
begin
  Result := (AMember.ClassType = TPasProperty) and
    (Copy(AMember.Name, 1, 2) <> 'On');
end;

function MethodFilter(AMember: TPasElement): Boolean;
begin
  Result := AMember.InheritsFrom(TPasProcedureBase);
  // Writeln(aMember.Name,' (',aMember.ClassName,') is Method ',Result);
end;

function EventFilter(AMember: TPasElement): Boolean;
begin
  Result := (AMember.ClassType = TPasProperty) and
    (Copy(AMember.Name, 1, 2) = 'On');
end;

{ ---------------------------------------------------------------------
  Writer registration
  ---------------------------------------------------------------------}

Type

{ TWriterRecord }

  TWriterRecord = Class(TObject)
  Private
    FClass : TFPDocWriterClass;
    FName : String;
    FDescription : String;
  Public
    Constructor Create (AClass : TFPDocWriterClass; Const AName,ADescr : String);
  end;

{ TPageInfo }

constructor TPageInfo.Create(aElement: TPasElement; aIndex: Integer);
begin
  Element:=aELement;
  SubpageIndex:=aIndex;
end;

{ TLinkData }

constructor TLinkData.Create(Const APathName, ALink, AModuleName: string);
begin
  FPathName:=APathName;
  FLink:=ALink;
  FModuleName:=AModuleName;
end;


{ TMultiFileDocWriter }

constructor TMultiFileDocWriter.Create(APackage: TPasPackage;
  AEngine: TFPDocEngine);
begin
  inherited Create(APackage, AEngine);
  FPageInfos:=TFPObjectList.Create;
  FSubPageNames:= False;
  FLinkUnresolvedCnt:=0;
end;

destructor TMultiFileDocWriter.Destroy;
begin
  FreeAndNil(FPageInfos);
  FreeAndNil(FAllocator);
  inherited Destroy;
end;

function TMultiFileDocWriter.GetPageCount: Integer;
begin
  Result := PageInfos.Count;
end;

function TMultiFileDocWriter.GetOutputPageNames: TStrings;
begin
  If (FoutputPageNames=Nil) then
    begin
    FOutputPageNames:=TStringList.Create;
    FOutputPageNames.Sorted:=True;
    end;
  Result:=FOutputPageNames;
end;

procedure TMultiFileDocWriter.OutputResults();
begin
  DoLog('Unresolved links: %d', [FLinkUnresolvedCnt]);
  inherited OutputResults();
end;

procedure TMultiFileDocWriter.LinkUnresolvedInc();
begin
  Inc(FLinkUnresolvedCnt);
end;

function TMultiFileDocWriter.ResolveLinkID(const Name: String): DOMString;
var
  res: String;

begin
  res:=Engine.ResolveLink(Module,Name, True);
  // engine can return backslashes on Windows
  res:= LinkFix(res);
  Result:=UTF8Decode(res);
end;

function TMultiFileDocWriter.ResolveLinkIDUnStrict(const Name: String
  ): DOMString;
var
  idDot, idLast: Integer;
  res: String;
begin
  res:=Engine.ResolveLink(Module,Name, True);
  if res = '' then
  begin
    // do simplify on one level from end.
    // TOCO: I want to move that code to last check of Engine.ResolveLink() for not Strict
    IdDot:= Pos('.', Name);
    IdLast:= 0;
    // search last dot
    while idDot > 0 do
    begin
      IdLast:= idDot;
      IdDot:= Pos('.', Name, IdLast+1);
    end;
    if idLast > 0 then
      // have cut last element
      res:= Engine.ResolveLink(Module, Copy(Name, 1, IdLast-1), True);
  end;
  res:= LinkFix(res);
  Result:=UTF8Decode(res);
end;

function TMultiFileDocWriter.LinkFix(ALink: String): String;
var
  res, s:String;
begin
  res:= ALink;
  if Length(res) > 0 then
  begin
    // If the link is in the same directory as current dir, then remove the directory part.
    s:=Copy(res, 1, Length(CurDirectory) + 1);
    if (S= CurDirectory + '/') or (s= CurDirectory + '\') then
      res := Copy(res, Length(CurDirectory) + 2, Length(res))
    else if not IsLinkAbsolute(res) then
      res := BaseDirectory + res;
  end;
  Result:= res;
end;

{ Used for:
  - <link> elements in descriptions
  - "see also" entries
  - AppendHyperlink (for unresolved parse tree element links)
}

function TMultiFileDocWriter.ResolveLinkIDInUnit(const Name,AUnitName: String): DOMString;

begin
  Result:=ResolveLinkID(Name);
  If (Result='') and (AUnitName<>'') and (length(Name)>0) and (Name[1]<>'#') then
     Result:=ResolveLinkID(AUnitName+'.'+Name);
end;


function TMultiFileDocWriter.ResolveLinkWithinPackage(AElement: TPasElement;
  ASubpageIndex: Integer): String;
var
  ParentEl: TPasElement;
  s : String;
begin
  ParentEl := AElement;
  while Assigned(ParentEl) and not (ParentEl.ClassType = TPasPackage) do
    ParentEl := ParentEl.Parent;
  if Assigned(ParentEl) and (TPasPackage(ParentEl) = Engine.Package) then
  begin
    Result := Allocator.GetFilename(AElement, ASubpageIndex);
    // engine/allocator can return backslashes on Windows
    s:=Copy(Result, 1, Length(CurDirectory) + 1);
    if (S= CurDirectory + '/') or (s= CurDirectory + '\') then
      Result := Copy(Result, Length(CurDirectory) + 2, Length(Result))
    else
      Result := BaseDirectory + Result;
  end else
    SetLength(Result, 0);
end;

procedure TMultiFileDocWriter.PrepareDocumentation;
begin
  inherited PrepareDocumentation;
  FAllocator:= CreateAllocator();
  FAllocator.SubPageNames:= SubPageNames;
end;

function TMultiFileDocWriter.AddPage(AElement: TPasElement;
  ASubpageIndex: Integer): TPageInfo;

begin
  Result:= TPageInfo.Create(aElement,aSubPageIndex);
  PageInfos.Add(Result);
  Allocator.AllocFilename(AElement, ASubpageIndex);
  if ASubpageIndex = 0 then
    Engine.AddLink(AElement.PathName,Allocator.GetFilename(AElement, ASubpageIndex));
end;

procedure TMultiFileDocWriter.AddTopicPages(AElement: TPasElement);

var
  PreviousTopic,
  TopicElement : TTopicElement;
  DocNode,
  TopicNode : TDocNode;

begin
  DocNode:=Engine.FindDocNode(AElement);
  If not Assigned(DocNode) then
    exit;
  TopicNode:=DocNode.FirstChild;
  PreviousTopic:=Nil;
  While Assigned(TopicNode) do
    begin
    If TopicNode.TopicNode then
      begin
      TopicElement:=TTopicElement.Create(TopicNode.Name,AElement);
      Topics.Add(TopicElement);
      TopicElement.TopicNode:=TopicNode;
      TopicElement.Previous:=PreviousTopic;
      If Assigned(PreviousTopic) then
        PreviousTopic.Next:=TopicElement;
      PreviousTopic:=TopicElement;
      if AElement is TTopicElement then
        TTopicElement(AElement).SubTopics.Add(TopicElement);
      AddPage(TopicElement,IdentifierIndex);
      if AElement is TTopicElement then
        TTopicElement(AElement).SubTopics.Add(TopicElement)
      else // Only one level of recursion.
        AddTopicPages(TopicElement);
      end;
    TopicNode:=TopicNode.NextSibling;
    end;
end;


function TMultiFileDocWriter.ModuleHasClasses(AModule: TPasModule): Boolean;

begin
  result:=assigned(AModule)
         and assigned(AModule.InterfaceSection)
         and assigned(AModule.InterfaceSection.Classes)
         and (AModule.InterfaceSection.Classes.Count>0);
end;


procedure TMultiFileDocWriter.AddPages(AElement: TPasElement; ASubpageIndex: Integer;
  AList: TFPList);
var
  i,j: Integer;
  R : TPasRecordtype;
  FPEl : TPasElement;
  DocNode: TDocNode;
begin
  if AList.Count > 0 then
    begin
    AddPage(AElement, ASubpageIndex);
    for i := 0 to AList.Count - 1 do
      begin
      AddPage(TPasElement(AList[i]), 0);
      if (TObject(AList[i]) is TPasRecordType) then
        begin
        R:=TObject(AList[I]) as TPasRecordType;
        For J:=0 to R.Members.Count-1 do
          begin
          FPEl:=TPasElement(R.Members[J]);
          if ((FPEL is TPasProperty) or (FPEL is TPasProcedureBase))
             and Engine.ShowElement(FPEl) then
               begin
               DocNode := Engine.FindDocNode(FPEl);
               if Assigned(DocNode) then
                 AddPage(FPEl, 0);
               end;
          end;
        end;
      end;
    end;
end;

procedure TMultiFileDocWriter.AllocateClassMemberPages(AModule: TPasModule;
  LinkList: TObjectList);
var
  i, j, k: Integer;
  ClassEl: TPasClassType;
  FPEl, AncestorMemberEl: TPasElement;
  DocNode: TDocNode;
  ALink : DOMString;
  DidAutolink: Boolean;

begin
  for i := 0 to AModule.InterfaceSection.Classes.Count - 1 do
    begin
    ClassEl := TPasClassType(AModule.InterfaceSection.Classes[i]);
    AddPage(ClassEl, 0);
    // !!!: Only add when there are items
    AddPage(ClassEl, PropertiesByInheritanceSubindex);
    AddPage(ClassEl, PropertiesByNameSubindex);
    AddPage(ClassEl, MethodsByInheritanceSubindex);
    AddPage(ClassEl, MethodsByNameSubindex);
    AddPage(ClassEl, EventsByInheritanceSubindex);
    AddPage(ClassEl, EventsByNameSubindex);
    for j := 0 to ClassEl.Members.Count - 1 do
      begin
      FPEl := TPasElement(ClassEl.Members[j]);
      if Not Engine.ShowElement(FPEl) then
        continue;
      DocNode := Engine.FindDocNode(FPEl);
      if Assigned(DocNode) then
        begin
        if Assigned(DocNode.Node) then
          ALink:=DocNode.Node['link']
        else
          ALink:='';
        If (ALink<>'') then
          LinkList.Add(TLinkData.Create(FPEl.PathName,UTF8Encode(ALink),AModule.name))
        else
          AddPage(FPEl, 0);
        end
      else
        begin
        DidAutolink := False;
        if Assigned(ClassEl.AncestorType) and
          (ClassEl.AncestorType.ClassType.inheritsfrom(TPasClassType)) then
          begin
          for k := 0 to TPasClassType(ClassEl.AncestorType).Members.Count - 1 do
            begin
            AncestorMemberEl :=
              TPasElement(TPasClassType(ClassEl.AncestorType).Members[k]);
            if AncestorMemberEl.Name = FPEl.Name then
              begin
              DocNode := Engine.FindDocNode(AncestorMemberEl);
              if Assigned(DocNode) then
                begin
                DidAutolink := True;
                Engine.AddLink(FPEl.PathName,
                  Engine.FindAbsoluteLink(AncestorMemberEl.PathName));
                break;
                end;
              end;
            end;
          end;
        if not DidAutolink then
          AddPage(FPEl, 0);
        end;
      end;
    end;
end;

procedure TMultiFileDocWriter.AllocateModulePages(AModule: TPasModule; LinkList : TObjectList);

var
  i: Integer;
  s: String;

begin
  if not assigned(Amodule.Interfacesection) then
    exit;
  AddPage(AModule, 0);
  AddPage(AModule,IndexSubIndex);
  AddTopicPages(AModule);
  with AModule do
    begin
    if InterfaceSection.ResStrings.Count > 0 then
      begin
      AddPage(AModule, ResstrSubindex);
      s := Allocator.GetFilename(AModule, ResstrSubindex);
      for i := 0 to InterfaceSection.ResStrings.Count - 1 do
        with TPasResString(InterfaceSection.ResStrings[i]) do
          Engine.AddLink(PathName, s + '#' + LowerCase(Name));
      end;
    AddPages(AModule, ConstsSubindex, InterfaceSection.Consts);
    AddPages(AModule, TypesSubindex, InterfaceSection.Types);
    if InterfaceSection.Classes.Count > 0 then
      begin
      AddPage(AModule, ClassesSubindex);
      AllocateClassMemberPages(AModule,LinkList);
      end;

    AddPages(AModule, ProcsSubindex, InterfaceSection.Functions);
    AddPages(AModule, VarsSubindex, InterfaceSection.Variables);
    end;
end;

  
procedure TMultiFileDocWriter.AllocatePackagePages;

Var
  I : Integer;
  H : Boolean;

begin
  if Length(Package.Name) <= 1 then
    exit;
  AddPage(Package, 0);
  AddPage(Package,IndexSubIndex);
  I:=0;
  H:=False;
  While (I<Package.Modules.Count) and Not H do
    begin
    H:=ModuleHasClasses(TPasModule(Package.Modules[i]));
    Inc(I);
    end;
  if H then
    AddPage(Package,ClassHierarchySubIndex);
  AddTopicPages(Package);
end;

procedure TMultiFileDocWriter.AllocatePages;

Var
  L : TObjectList;
  ML : TFPList;
  I : Integer;


begin
  // Allocate page for the package itself, if a name is given (i.e. <> '#')
  AllocatePackagePages;
  ML:=Nil;
  L:=TObjectList.Create;
  try
    ML:=TFPList.Create;
    ML.AddList(Package.Modules);
    ML.Sort(@SortPasElements);
    for i := 0 to ML.Count - 1 do
      AllocateModulePages(TPasModule(ML[i]),L);
    // Resolve links
    For I:=0 to L.Count-1 do
      With TLinkData(L[i]) do
        Engine.AddLink(FPathName,UTF8Encode(ResolveLinkIDInUnit(FLink,FModuleName)));
  finally
    L.Free;
    ML.Free;
  end;
end;

function TMultiFileDocWriter.GetFileBaseDir(aOutput: String) : String;

begin
  Result:=aOutput;
  if Result<>'' then
    Result:=IncludeTrailingPathDelimiter(Result);
end;

procedure TMultiFileDocWriter.DoWriteDocumentation;

  procedure CreatePath(const AFilename: String);

  var
    EndIndex: Integer;
    Path: String;
  begin
    EndIndex := Length(AFilename);
    if EndIndex = 0 then
      exit;
    while not (AFilename[EndIndex] in AllowDirectorySeparators) do
    begin
      Dec(EndIndex);
      if EndIndex = 0 then
        exit;
    end;

    Path := Copy(AFilename, 1, EndIndex - 1);
    if not DirectoryExists(Path) then
    begin
      CreatePath(Path);
      MkDir(Path);
    end;
  end;


var
  i: Integer;
  FileName : String;
  FinalFilename: String;

begin
  AllocatePages;
  DoLog(SWritingPages, [PageCount]);
  if Engine.Output <> '' then
    Engine.Output := IncludeTrailingBackSlash(Engine.Output);
   for i := 0 to PageInfos.Count - 1 do
     with TPageInfo(PageInfos[i]) do
       begin
       FileName:= Allocator.GetFilename(Element, SubpageIndex);
       if MustGeneratePage(FileName) then
         begin
         FinalFilename := GetFileBaseDir(Engine.Output) + FileName;
         CreatePath(FinalFilename);
         WriteDocPage(FileName,ELement,SubPageIndex);
         end;
       end;
end;

function TMultiFileDocWriter.MustGeneratePage(aFileName: String): Boolean;
begin
  Result:=Not Assigned(FOutputPageNames);
  if Not Result then
    Result:=FOutputPageNames.IndexOf(aFileName)<>-1;
end;

class procedure TMultiFileDocWriter.Usage(List: TStrings);
begin
  List.AddStrings(['--use-subpagenames', SUsageSubNames]);
  List.AddStrings(['--only-pages=LIST', SUsageOnlyPages]);
end;

function TMultiFileDocWriter.InterPretOption(const Cmd, Arg: String): boolean;

Var
  I : Integer;
  FN : String;

begin
  Writeln('Cmd : ',Cmd);
  Result := True;
  if Cmd = '--use-subpagenames' then
    FSubPageNames:= True
  else
  if Cmd = '--only-pages' then
    begin
    Result:=Arg<>'';
    if Result then
      begin
      if Arg[1]='@' then
        begin
        FN:=Copy(Arg,2,Length(Arg)-1);
        OutputPageNames.LoadFromFile(FN);
        end
      else
        begin
        For I:=1 to WordCount(Arg,[',']) do
          OutputPageNames.Add(ExtractWord(I,Arg,[',']));
        end;
      Writeln('OutputPagenames ',OutputPagenames.CommaText);
      end
    end
  else
    Result:=inherited InterPretOption(Cmd, Arg);
end;


{ TWriterRecord }

constructor TWriterRecord.Create(AClass: TFPDocWriterClass; const AName,
  ADescr: String);
begin
  FClass:=AClass;
  FName:=AName;
  FDescription:=ADescr;
end;

Var
  Writers : TStringList;

Procedure InitWriterList;

begin
  Writers:=TStringList.Create;
  Writers.Sorted:=True;
end;

Procedure DoneWriterList;

Var
  I : Integer;

begin
  For I:=Writers.Count-1 downto 0 do
    Writers.Objects[i].Free;
  FreeAndNil(Writers);
end;

procedure RegisterWriter(AClass : TFPDocWriterClass; Const AName, ADescr : String);
begin
  If Writers.IndexOf(AName)<>-1 then
    Raise EFPDocWriterError.CreateFmt(SErralreadyRegistered,[ANAme]);
  Writers.AddObject(AName,TWriterRecord.Create(AClass,AName,ADescr));
end;

function  FindWriterClass(AName : String) : Integer;

begin
  Result:=Writers.IndexOf(AName);
end;

function GetWriterClass(AName : String) : TFPDocWriterClass;

Var
  Index : Integer;

begin
  Index:=FindWriterClass(AName);
  If Index=-1 then
    Raise EFPDocWriterError.CreateFmt(SErrUnknownWriterClass,[ANAme]);
  Result:=(Writers.Objects[Index] as TWriterRecord).FClass;
end;

// UnRegister backend

Procedure UnRegisterWriter(Const AName : String);
Var
  Index : Integer;

begin
  Index:=Writers.IndexOf(AName);
  If Index=-1 then
    Raise EFPDocWriterError.CreateFmt(SErrUnknownWriterClass,[ANAme]);
  Writers.Objects[Index].Free;
  Writers.Delete(Index);
end;


Procedure EnumWriters(List : TStrings);

Var
  I : Integer;

begin
  List.Clear;
  For I:=0 to Writers.Count-1 do
    With (Writers.Objects[I] as TWriterRecord) do
      List.Add(FName+'='+FDescription);
end;

function IsWhitespaceNode(Node: TDOMText): Boolean;
var
  I,L: Integer;
  S: DOMString;
  P : PWideChar;
  
begin
  S := Node.Data;
  Result := True;
  I:=0;
  L:=Length(S);
  P:=PWideChar(S);
  While Result and (I<L) do
    begin
    Result:=P^ in [#32,#10,#9,#13];
    Inc(P);
    Inc(I);
    end;
end;

{ ---------------------------------------------------------------------
  TFileAllocator
  ---------------------------------------------------------------------}

function TFileAllocator.GetFilePostfix(ASubindex: Integer): String;
begin
  if FSubPageNames then
  case ASubindex of
    IdentifierIndex: Result:='';
    ResstrSubindex: Result:='reestr';
    ConstsSubindex: Result:='consts';
    TypesSubindex: Result:='types';
    ClassesSubindex: Result:='classes';
    ProcsSubindex: Result:='procs';
    VarsSubindex: Result:='vars';
    TopicsSubIndex: Result:='topics';
    IndexSubIndex: Result:='indexes';
    ClassHierarchySubIndex: Result:='class-tree';
    InterfaceHierarchySubIndex: Result:='interface-tree';
    PropertiesByInheritanceSubindex: Result:='props';
    PropertiesByNameSubindex: Result:='props-n';
    MethodsByInheritanceSubindex: Result:='methods';
    MethodsByNameSubindex: Result:='methods-n';
    EventsByInheritanceSubindex: Result:='events';
    EventsByNameSubindex: Result:='events-n';
  end
    else
  Result:= IntToStr(ASubindex);
end;

procedure TFileAllocator.Create();
begin
  FSubPageNames:= False;
end;

procedure TFileAllocator.AllocFilename(AElement: TPasElement;
  ASubindex: Integer);
begin
end;

function TFileAllocator.GetRelativePathToTop(AElement: TPasElement): String;
begin
  Result:='';
end;

function TFileAllocator.GetCSSFilename(ARelativeTo: TPasElement): DOMString;
begin
  Result := Utf8Decode(GetRelativePathToTop(ARelativeTo)) + 'fpdoc.css';
end;

{ ---------------------------------------------------------------------
  TLongNameFileAllocator
  ---------------------------------------------------------------------}


constructor TLongNameFileAllocator.Create(const AExtension: String);
begin
  inherited Create;
  FExtension := AExtension;
end;

function TLongNameFileAllocator.GetFilename(AElement: TPasElement; ASubindex: Integer): String;

var
  n,s: String;
  i: Integer;
  MElement: TPasElement;
begin
  Result:='';
  if AElement.ClassType = TPasPackage then
    Result := 'index'
  else if AElement.ClassType = TPasModule then
    Result := LowerCase(AElement.Name) + PathDelim + 'index'
  else
  begin
    if AElement is TPasOperator then
    begin
      if Assigned(AElement.Parent) then
        result:=LowerCase(AElement.Parent.PathName);
      With TPasOperator(aElement) do
        Result:= Result + 'op-'+OperatorTypeToOperatorName(OperatorType);
      s := '';
      N:=LowerCase(aElement.Name); // Should not contain any weird chars.
      Delete(N,1,Pos('(',N));
      i := 1;
      Repeat
        I:=Pos(',',N);
        if I=0 then
          I:=Pos(')',N);
        if I>1 then
          begin
          if (S<>'') then
            S:=S+'-';
          S:=S+Copy(N,1,I-1);
          end;
        Delete(N,1,I);
      until I=0;
      // First char is maybe :
      if (N<>'') and  (N[1]=':') then
        Delete(N,1,1);
      Result:=Result + '-'+ s + '-' + N;
    end else
      Result := LowerCase(AElement.PathName);
    // cut off Package Name
    MElement:= AElement.GetModule;
    if Assigned(MElement) then
      AElement:= MElement;
    Result := Copy(Result, Length(AElement.Parent.Name) + 2, MaxInt);
    // to skip dots in unit name
    i := Length(AElement.Name);
    while (i <= Length(Result)) and (Result[i] <> '.') do
      Inc(i);
    if (i <= Length(Result)) and (i > 0) then
      Result[i] := PathDelim;
  end;

  if ASubindex > 0 then
    Result := Result + '-' + GetFilePostfix(ASubindex);
  Result := Result + Extension;
end;

function TLongNameFileAllocator.GetRelativePathToTop(AElement: TPasElement): String;
begin
  if (AElement.ClassType=TPasPackage) then
    Result := ''
  else if (AElement.ClassType=TTopicElement) then
    begin
    If (AElement.Parent.ClassType=TTopicElement) then
      Result:='../'+GetRelativePathToTop(AElement.Parent)
    else if (AElement.Parent.ClassType=TPasPackage) then
      Result:=''
    else if (AElement.Parent.ClassType=TPasModule) then
      Result:='../';
    end
  else
    Result := '../';
end;


{ ---------------------------------------------------------------------
  TFPDocWriter
  ---------------------------------------------------------------------}
{
      fmtIPF:
        begin
          if Length(Engine.Output) = 0 then
            WriteLn(SCmdLineOutputOptionMissing)
          else
            CreateIPFDocForPackage(Engine.Package, Engine);
        end;


}
constructor TFPDocWriter.Create ( APackage: TPasPackage; AEngine: TFPDocEngine
  ) ;

begin
  inherited Create;
  FEngine  := AEngine;
  FPackage := APackage;
  FTopics:=Tlist.Create;
  FImgExt:='.png';
  TreeClass:= TClassTreeBuilder.Create(FEngine, FPackage, okWithFields);
  TreeInterface:= TClassTreeBuilder.Create(FEngine, FPackage, [okInterface]);
  CreateClassTree;
end;

destructor TFPDocWriter.Destroy;

Var
  i : integer;

begin
  For I:=0 to FTopics.Count-1 do
    TTopicElement(FTopics[i]).Free;
  FTopics.Free;
  TreeClass.free;
  TreeInterface.Free;
  Inherited;
end;

procedure TFPDocWriter.AddModuleIdentifiers(AModule : TPasModule; L : TStrings);

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



function TFPDocWriter.InterpretOption(const Cmd, Arg: String): Boolean;
begin
  Result:=False;
end;

class function TFPDocWriter.FileNameExtension: String;
begin
//Override in linear writers with the expected extension.
  Result := ''; //Output must not contain an extension.
end;

class procedure TFPDocWriter.Usage(List: TStrings);
begin
  // Do nothing.
end;

class procedure TFPDocWriter.SplitImport(var AFilename, ALinkPrefix: String);
var
  i: integer;
begin
//override in HTML and CHM writer
  i := Pos(',', AFilename);
  if i > 0 then
    begin  //split CSV into filename and prefix
    ALinkPrefix := Copy(AFilename,i+1,Length(AFilename));
    SetLength(AFilename, i-1);
    end;
end;

procedure TFPDocWriter.WriteDocumentation;
begin
  PrepareDocumentation();
  DoWriteDocumentation();
  OutputResults();
end;

function TFPDocWriter.FindTopicElement ( Node: TDocNode ) : TTopicElement;

Var
  I : Integer;

begin
  Result:=Nil;
  I:=FTopics.Count-1;
  While (I>=0) and (Result=Nil) do
    begin
    If (TTopicElement(FTopics[i]).TopicNode=Node) Then
      Result:=TTopicElement(FTopics[i]);
    Dec(I);
    end;
end;

procedure TFPDocWriter.DescrWriteImageEl(const AFileName, ACaption,
  ALinkName: DOMString);

begin
  DoLog('%s : No support for images yet: %s (caption: "%s")',[ClassName,AFileName,ACaption]);
end;

procedure TFPDocWriter.PrepareDocumentation;
begin
  // Ancestors can call AllocatePages();CreateAllocator(); into base class
end;

{ ---------------------------------------------------------------------
  Generic documentation node conversion
  ---------------------------------------------------------------------}

function IsContentNodeType(Node: TDOMNode): Boolean;
begin
  Result := (Node.NodeType = ELEMENT_NODE) or 
    ((Node.NodeType = TEXT_NODE) and not IsWhitespaceNode(TDOMText(Node))) or
    (Node.NodeType = ENTITY_REFERENCE_NODE);
end;


procedure TFPDocWriter.Warning(AContext: TPasElement; const AMsg: String);
begin
  if (AContext<>nil) then
    DoLog('[%s] %s',[AContext.PathName,AMsg])
  else
    DoLog('[<no context>] %s', [AMsg]);
end;

procedure TFPDocWriter.Warning(AContext: TPasElement; const AMsg: String;
  const Args: array of const);
begin
  Warning(AContext, Format(AMsg, Args));
end;

function TFPDocWriter.IsDescrNodeEmpty(Node: TDOMNode): Boolean;
var
  Child: TDOMNode;
begin
  if (not Assigned(Node)) or (not Assigned(Node.FirstChild)) then
    Result := True
  else
  begin
    Child := Node.FirstChild;
    while Assigned(Child) do
    begin
      if (Child.NodeType = ELEMENT_NODE) or (Child.NodeType = TEXT_NODE) or
        (Child.NodeType = ENTITY_REFERENCE_NODE) then
      begin
        Result := False;
        exit;
      end;
      Child := Child.NextSibling;
    end;
  end;
  Result := True;
end;

{ Check wether the nodes starting with the node given as argument make up an
  'extshort' production. }
function TFPDocWriter.IsExtShort(Node: TDOMNode): Boolean;
begin
  while Assigned(Node) do
  begin
    if Node.NodeType = ELEMENT_NODE then
      if (Node.NodeName <> 'br') and
         (Node.NodeName <> 'link') and
         (Node.NodeName <> 'url') and
         (Node.NodeName <> 'b') and
         (Node.NodeName <> 'file') and
         (Node.NodeName <> 'i') and
         (Node.NodeName <> 'kw') and
         (Node.NodeName <> 'printshort') and
         (Node.NodeName <> 'var') then
      begin
        Result := False;
        exit;
      end;
    Node := Node.NextSibling;
  end;
  Result := True;
end;

function TFPDocWriter.ConvertShort(AContext: TPasElement;
 El: TDOMElement): Boolean;
var
  Node: TDOMNode;
begin
  Result := False;
  if not Assigned(El) then
    exit;
  FContext:=AContext;
  try
    Node := El.FirstChild;
    while Assigned(Node) do
    begin
      if (Node.NodeType = ELEMENT_NODE) and (Node.NodeName = 'link') then
        ConvertLink(AContext, TDOMElement(Node))
      else if (Node.NodeType = ELEMENT_NODE) and (Node.NodeName = 'url') then
        ConvertURL(AContext, TDOMElement(Node))
      else
        if not ConvertBaseShort(AContext, Node) then
          exit;
      Node := Node.NextSibling;
    end;
    Result := True;
  finally
    FContext:=Nil;
  end;
end;

function TFPDocWriter.ConvertNotes(AContext: TPasElement; El: TDOMElement
  ): Boolean;

Var
  L : TFPList;
  N : TDomNode;
  I : Integer;
  B : Boolean;

begin
  Result:=Assigned(El) and EmitNotes;
  If Not Result then
    exit;
  L:=TFPList.Create;
  try
    N:=El.FirstChild;
    While Assigned(N) do
      begin
      If (N.NodeType=ELEMENT_NODE) and (N.NodeName='note') then
        begin
        B:=True;
        if Assigned(FBeforeEmitNote) then
          FBeforeEmitNote(Self,TDomElement(N),B);
        If B then
          L.Add(N);
        end;
      N:=N.NextSibling;
      end;
    Result:=L.Count>0;
    If Not Result then
      exit;
    DescrEmitNotesHeader(AContext);
    DescrBeginUnorderedList;
    For i:=0 to L.Count-1 do
      begin
      DescrBeginListItem;
      ConvertExtShortOrNonSectionBlocks(AContext, TDOMNode(L[i]).FirstChild);
      DescrEndListItem;
      end;
    DescrEndUnorderedList;
    DescrEmitNotesFooter(AContext);
  finally
    L.Free;
  end;
end;

function TFPDocWriter.ConvertBaseShort(AContext: TPasElement;
  Node: TDOMNode): Boolean;

  function ConvertText: DOMString;
  var
    s: DOMString;
    i: Integer;
  begin
    if Node.NodeType = TEXT_NODE then
    begin
      s := Node.NodeValue;
      i := 1;
      Result:='';
      while i <= Length(s) do
        if s[i] = #13 then
        begin
          Result := Result + ' ';
          Inc(i);
          if s[i] = #10 then
            Inc(i);
        end else if s[i] = #10 then
        begin
          Result := Result + ' ';
          Inc(i);
        end else
        begin
          Result := Result + s[i];
          Inc(i);
        end;
    end else if Node.NodeType = ENTITY_REFERENCE_NODE then
      if Node.NodeName = 'fpc' then
        Result := 'Free Pascal'
      else if Node.NodeName = 'delphi' then
        Result := 'Delphi'
      else
      begin
        Warning(AContext, Format(SErrUnknownEntityReference, [Node.NodeName]));
        Result := Node.NodeName;
      end
    else if Node.NodeType = ELEMENT_NODE then
      SetLength(Result, 0);
  end;

  function ConvertTextContent: DOMString;
  begin
    Result:='';
    Node := Node.FirstChild;
    while Assigned(Node) do
    begin
      Result := Result + ConvertText;
      Node := Node.NextSibling;
    end;
  end;

var
  El, DescrEl: TDOMElement;
  hlp : TPasElement;
begin
  Result := True;
  if Node.NodeType = ELEMENT_NODE then
    if Node.NodeName = 'b' then
    begin
      DescrBeginBold;
      ConvertBaseShortList(AContext, Node, False);
      DescrEndBold;
    end else
    if Node.NodeName = 'i' then
    begin
      DescrBeginItalic;
      ConvertBaseShortList(AContext, Node, False);
      DescrEndItalic;
    end else
    if Node.NodeName = 'em' then
    begin
      DescrBeginEmph;
      ConvertBaseShortList(AContext, Node, False);
      DescrEndEmph;
    end else
    if Node.NodeName = 'u' then
    begin
      DescrBeginUnderline;
      ConvertBaseShortList(AContext, Node, False);
      DescrEndUnderline;
    end else
    if Node.NodeName = 'file' then
      DescrWriteFileEl(ConvertTextContent)
    else if Node.NodeName = 'kw' then
      DescrWriteKeywordEl(ConvertTextContent)
    else if Node.NodeName = 'printshort' then
    begin
      El := TDOMElement(Node);
      hlp:=AContext;
      while assigned(hlp) and not (hlp is TPasModule) do 
        hlp:=hlp.parent;
      if not (hlp is TPasModule) then
        hlp:=nil;
      DescrEl := Engine.FindShortDescr(TPasModule(hlp), UTF8Encode(El['id']));
      if Assigned(DescrEl) then
        ConvertShort(AContext, DescrEl)
      else
      begin
        Warning(AContext, Format(SErrUnknownPrintShortID, [El['id']]));
        DescrBeginBold;
        DescrWriteText('#ShortDescr:' + El['id']);
        DescrEndBold;
      end;
    end else if Node.NodeName = 'var' then
      DescrWriteVarEl(ConvertTextContent)
    else
      Result := False
  else
    DescrWriteText(ConvertText);
end;

procedure TFPDocWriter.ConvertBaseShortList(AContext: TPasElement;
  Node: TDOMNode; MayBeEmpty: Boolean);
var
  Child: TDOMNode;
begin
  Child := Node.FirstChild;
  while Assigned(Child) do
  begin
    if not ConvertBaseShort(AContext, Child) then
      Warning(AContext, SErrInvalidShortDescr)
    else
      MayBeEmpty := True;
    Child := Child.NextSibling;
  end;
  if not MayBeEmpty then
    Warning(AContext, SErrInvalidShortDescr)
end;

procedure TFPDocWriter.ConvertLink(AContext: TPasElement; El: TDOMElement);
begin
  DescrBeginLink(El['id']);
  if not IsDescrNodeEmpty(El) then
    ConvertBaseShortList(AContext, El, True)
  else
    DescrWriteText(El['id']);
  DescrEndLink;
end;

procedure TFPDocWriter.ConvertURL(AContext: TPasElement; El: TDOMElement);
begin
  DescrBeginURL(El['href']);
  if not IsDescrNodeEmpty(El) then
    ConvertBaseShortList(AContext, El, True)
  else
    DescrWriteText(El['href']);
  DescrEndURL;
end;

procedure TFPDocWriter.AddElementsFromList ( L: TStrings; List: TFPList;
  UsePathName: Boolean ) ;
Var
  I : Integer;
  El : TPasElement;
  N : TDocNode;

begin
  For I:=0 to List.Count-1 do
    begin
    El:=TPasElement(List[I]);
    N:=Engine.FindDocNode(El);
    if (N=Nil) or (not N.IsSkipped) then
      begin
      if UsePathName then
        L.AddObject(El.PathName,El)
      else
        L.AddObject(El.Name,El);
      If el is TPasEnumType then
        AddElementsFromList(L,TPasEnumType(el).Values);
      end;
    end;
end;

procedure TFPDocWriter.CreateClassTree;
var
   L: TStringList;
   M: TPasModule;
   I:Integer;
begin
  L:=TStringList.Create;
  try
    For I:=0 to Package.Modules.Count-1 do
      begin
      M:=TPasModule(Package.Modules[i]);
      if Not (M is TPasExternalModule) and assigned(M.InterfaceSection) then
        Self.AddElementsFromList(L,M.InterfaceSection.Classes,True)
      end;
      // You can see this tree by using --format=xml option
      TreeClass.BuildTree(L);
      TreeInterface.BuildTree(L);
  Finally
    L.Free;
  end;
end;

procedure TFPDocWriter.DoLog(const Msg: String);
begin
  If Assigned(FEngine.OnLog) then
    FEngine.OnLog(Self,Msg);
end;

procedure TFPDocWriter.DoLog(const Fmt: String; Args: array of const);
begin
  DoLog(Format(Fmt,Args));
end;

procedure TFPDocWriter.OutputResults();
begin
  DoLog('Package: %s - Documentation process finished.', [FPackage.Name]);
end;

function TFPDocWriter.ConvertExtShort(AContext: TPasElement;
  Node: TDOMNode): Boolean;
begin
  Result := False;

  while Assigned(Node) do
  begin
    if (Node.NodeType = ELEMENT_NODE) and (Node.NodeName = 'link') then
      ConvertLink(AContext, TDOMElement(Node))
    else if (Node.NodeType = ELEMENT_NODE) and (Node.NodeName = 'url') then
      ConvertURL(AContext, TDOMElement(Node))
    else if (Node.NodeType = ELEMENT_NODE) and (Node.NodeName = 'br') then
      DescrWriteLinebreak
    else
      if not ConvertBaseShort(AContext, Node) then
        exit;
    Node := Node.NextSibling;
  end;
  Result := True;
end;

procedure TFPDocWriter.ConvertDescr(AContext: TPasElement; El: TDOMElement;
  AutoInsertBlock: Boolean);
var
  Node, Child: TDOMNode;
  ParaCreated: Boolean;
begin
  FContext:=AContext;
  try
    if AutoInsertBlock then
      if IsExtShort(El.FirstChild) then
        DescrBeginParagraph
      else
        AutoInsertBlock := False;

    Node := El.FirstChild;
    if not ConvertExtShort(AContext, Node) then
    begin
      while Assigned(Node) do
      begin
        if (Node.NodeType = ELEMENT_NODE) and (Node.NodeName = 'section') then
        begin
          DescrBeginSectionTitle;
          Child := Node.FirstChild;
          while Assigned(Child) and (Child.NodeType <> ELEMENT_NODE) do
          begin
            if not IsDescrNodeEmpty(Child) then
              Warning(AContext, SErrInvalidContentBeforeSectionTitle);
            Child := Child.NextSibling;
          end;
          if not Assigned(Child) or (Child.NodeName <> 'title') then
            Warning(AContext, SErrSectionTitleExpected)
          else
            ConvertShort(AContext, TDOMElement(Child));

          DescrBeginSectionBody;

          if IsExtShort(Child) then
          begin
            DescrBeginParagraph;
            ParaCreated := True;
          end else
            ParaCreated := False;

          ConvertExtShortOrNonSectionBlocks(AContext, Child.NextSibling);

          if ParaCreated then
            DescrEndParagraph;
          DescrEndSection;
        end else if not ConvertNonSectionBlock(AContext, Node) then
          Warning(AContext, SErrInvalidDescr, [Node.NodeName]);
        Node := Node.NextSibling;
      end;
    end else
      if AutoInsertBlock then
        DescrEndParagraph;
  finally
    FContext:=Nil;
  end;
end;

procedure TFPDocWriter.ConvertExtShortOrNonSectionBlocks(AContext: TPasElement;
  Node: TDOMNode);
begin
  if not ConvertExtShort(AContext, Node) then
    while Assigned(Node) do
    begin
      if not ConvertNonSectionBlock(AContext, Node) then
        Warning(AContext, SErrInvalidDescr, [Node.NodeName]);
      Node := Node.NextSibling;
    end;
end;

function TFPDocWriter.ConvertNonSectionBlock(AContext: TPasElement;
  Node: TDOMNode): Boolean;

  procedure ConvertCells(Node: TDOMNode);
  var
    Child: TDOMNode;
    IsEmpty: Boolean;
  begin
    Node := Node.FirstChild;
    IsEmpty := True;
    while Assigned(Node) do
    begin
      if (Node.NodeType = ELEMENT_NODE) and (Node.NodeName = 'td') then
      begin
        DescrBeginTableCell;
        Child := Node.FirstChild;
        if not ConvertExtShort(AContext, Child) then
          while Assigned(Child) do
          begin
            if not ConvertSimpleBlock(AContext, Child) then
              Warning(AContext, SErrInvalidTableContent);
            Child := Child.NextSibling;
          end;
        DescrEndTableCell;
        IsEmpty := False;
      end else
        if IsContentNodeType(Node) then
          Warning(AContext, SErrInvalidTableContent);
      Node := Node.NextSibling;
    end;
    if IsEmpty then
      Warning(AContext, SErrTableRowEmpty);
  end;

  procedure ConvertTable;

    function GetColCount(Node: TDOMNode): Integer;
    begin
      Result := 0;
      Node := Node.FirstChild;
      while Assigned(Node) do
      begin
        if (Node.NodeType = ELEMENT_NODE) and (Node.NodeName = 'td') then
          Inc(Result);
        Node := Node.NextSibling;
      end;
    end;

  var
    s: DOMString;
    HasBorder, CaptionPossible, HeadRowPossible: Boolean;
    ColCount, ThisRowColCount: Integer;
    Subnode: TDOMNode;
  begin
    s := TDOMElement(Node)['border'];
    if s = '1' then
      HasBorder := True
    else
    begin
      HasBorder := False;
      if (Length(s) <> 0) and (s <> '0') then
        Warning(AContext, SErrInvalidBorderValue, ['<table>']);
    end;

    // Determine the number of columns
    ColCount := 0;
    Subnode := Node.FirstChild;
    while Assigned(Subnode) do
    begin
      if Subnode.NodeType = ELEMENT_NODE then
        if (Subnode.NodeName = 'caption') or (Subnode.NodeName = 'th') or
          (Subnode.NodeName = 'tr') then
        begin
          ThisRowColCount := GetColCount(Subnode);
          if ThisRowColCount > ColCount then
            ColCount := ThisRowColCount;
        end;
      Subnode := Subnode.NextSibling;
    end;

    DescrBeginTable(ColCount, HasBorder);

    Node := Node.FirstChild;
    CaptionPossible := True;
    HeadRowPossible := True;
    while Assigned(Node) do
    begin
      if Node.NodeType = ELEMENT_NODE then
        if CaptionPossible and (Node.NodeName = 'caption') then
        begin
          DescrBeginTableCaption;
          if not ConvertExtShort(AContext, Node.FirstChild) then
            Warning(AContext, SErrInvalidTableContent);
          DescrEndTableCaption;
          CaptionPossible := False;
        end else if HeadRowPossible and (Node.NodeName = 'th') then
        begin
          DescrBeginTableHeadRow;
          ConvertCells(Node);
          DescrEndTableHeadRow;
          CaptionPossible := False;
          HeadRowPossible := False;
        end else if Node.NodeName = 'tr' then
        begin
          DescrBeginTableRow;
          ConvertCells(Node);
          DescrEndTableRow;
        end else
          Warning(AContext, SErrInvalidTableContent)
      else if IsContentNodeType(Node) then
        Warning(AContext, SErrInvalidTableContent);
      Node := Node.NextSibling;
    end;
    DescrEndTable;
  end;

begin
  if Node.NodeType <> ELEMENT_NODE then
  begin
    if Node.NodeType = TEXT_NODE then
      Result := IsWhitespaceNode(TDOMText(Node))
    else
      Result := Node.NodeType = COMMENT_NODE;
    exit;
  end;
  if Node.NodeName = 'remark' then
  begin
    DescrBeginRemark;
    Node := Node.FirstChild;
    if not ConvertExtShort(AContext, Node) then
      while Assigned(Node) do
      begin
        if (Node.NodeType = ELEMENT_NODE) and (Node.NodeName = 'table') then
          ConvertTable
        else
          if not ConvertSimpleBlock(AContext, Node) then
            Warning(AContext, SErrInvalidRemarkContent, [Node.NodeName]);
        Node := Node.NextSibling;
      end;
    DescrEndRemark;
    Result := True;
  end else if Node.NodeName = 'table' then
  begin
    ConvertTable;
    Result := True;
  end else
    Result := ConvertSimpleBlock(AContext, Node);
end;

function TFPDocWriter.ConvertSimpleBlock(AContext: TPasElement;
  Node: TDOMNode): Boolean;

  procedure ConvertListItems;
  var
    Empty: Boolean;
  begin
    Node := Node.FirstChild;
    Empty := True;
    while Assigned(Node) do
    begin
      if ((Node.NodeType = TEXT_NODE) and not IsWhitespaceNode(TDOMText(Node))) or (Node.NodeType = ENTITY_REFERENCE_NODE)
        then
        Warning(AContext, SErrInvalidListContent)
      else if Node.NodeType = ELEMENT_NODE then
        if Node.NodeName = 'li' then
        begin
          DescrBeginListItem;
          ConvertExtShortOrNonSectionBlocks(AContext, Node.FirstChild);
          DescrEndListItem;
          Empty := False;
        end else
          Warning(AContext, SErrInvalidElementInList);
      Node := Node.NextSibling;
    end;
    if Empty then
      Warning(AContext, SErrListIsEmpty);
  end;

  procedure ConvertDefinitionList;
  var
    Empty, ExpectDTNext: Boolean;
  begin
    Node := Node.FirstChild;
    Empty := True;
    ExpectDTNext := True;
    while Assigned(Node) do
    begin
      if ((Node.NodeType = TEXT_NODE) and not IsWhitespaceNode(TDOMText(Node))) or (Node.NodeType = ENTITY_REFERENCE_NODE)
        then
        Warning(AContext, SErrInvalidListContent)
      else if Node.NodeType = ELEMENT_NODE then
        if ExpectDTNext and (Node.NodeName = 'dt') then
        begin
          DescrBeginDefinitionTerm;
          if not ConvertShort(AContext, TDOMElement(Node)) then
            Warning(AContext, SErrInvalidDefinitionTermContent);
          DescrEndDefinitionTerm;
          Empty := False;
          ExpectDTNext := False;
        end else if not ExpectDTNext and (Node.NodeName = 'dd') then
        begin
          DescrBeginDefinitionEntry;
          ConvertExtShortOrNonSectionBlocks(AContext, Node.FirstChild);
          DescrEndDefinitionEntry;
          ExpectDTNext := True;
        end else
          Warning(AContext, SErrInvalidElementInList);
      Node := Node.NextSibling;
    end;
    if Empty then
      Warning(AContext, SErrListIsEmpty)
    else if not ExpectDTNext then
      Warning(AContext, SErrDefinitionEntryMissing);
  end;

  procedure ProcessCodeBody(Node: TDOMNode);
  var
    s: String;
    i, j: Integer;
  begin
    Node := Node.FirstChild;
    S:='';
    while Assigned(Node) do
    begin
      if Node.NodeType = TEXT_NODE then
      begin
        s := s + UTF8Encode(Node.NodeValue);
        j := 1;
        for i := 1 to Length(s) do
          // In XML, linefeeds are normalized to #10 by the parser!
          if s[i] = #10 then
          begin
            DescrWriteCodeLine(Copy(s, j, i - j));
            j := i + 1;
          end;
        if j > 1 then
          s := Copy(s, j, Length(s));
      end;
      Node := Node.NextSibling;
    end;
    if Length(s) > 0 then
      DescrWriteCodeLine(s);
  end;

var
  s: DOMString;
  HasBorder: Boolean;
begin
  if Node.NodeType <> ELEMENT_NODE then
  begin
    Result := (Node.NodeType = TEXT_NODE) and IsWhitespaceNode(TDOMText(Node));
    exit;
  end;
  if Node.NodeName = 'p' then
  begin
    DescrBeginParagraph;
    if not ConvertExtShort(AContext, Node.FirstChild) then
      Warning(AContext, SErrInvalidParaContent);
    DescrEndParagraph;
    Result := True;
  end else if Node.NodeName = 'code' then
  begin
    s := TDOMElement(Node)['border'];
    if s = '1' then
      HasBorder := True
    else
    begin
      if (Length(s) > 0) and (s <> '0') then
        Warning(AContext, SErrInvalidBorderValue, ['<code>']);
    end;

    DescrBeginCode(HasBorder, UTF8Encode(TDOMElement(Node)['highlighter']));
    ProcessCodeBody(Node);
    DescrEndCode;
    Result := True;
  end else if Node.NodeName = 'pre' then
  begin
    DescrBeginCode(False, 'none');
    ProcessCodeBody(Node);
    DescrEndCode;
    Result := True;
  end else if Node.NodeName = 'ul' then
  begin
    DescrBeginUnorderedList;
    ConvertListItems;
    DescrEndUnorderedList;
    Result := True;
  end else if Node.NodeName = 'ol' then
  begin
    DescrBeginOrderedList;
    ConvertListItems;
    DescrEndOrderedList;
    Result := True;
  end else if Node.NodeName = 'dl' then
  begin
    DescrBeginDefinitionList;
    ConvertDefinitionList;
    DescrEndDefinitionList;
    Result := True;
  end else if Node.NodeName = 'img' then
  begin
    begin
    ConvertImage(Node as TDomElement);
    Result:=True;
    end;
  end else  
    Result := False;
end;

procedure TFPDocWriter.ConvertImage ( El: TDomElement ) ;

Var
  FN,Cap,LinkName : DOMString;

begin
  FN:=El['file'];
  Cap:=El['caption'];
  LinkName:=El['name'];
  FN:=UTF8decode(ChangeFileExt(UTF8Encode(FN),ImageExtension));
  DescrWriteImageEl(FN,Cap,LinkName);
end;

procedure TFPDocWriter.DescrEmitNotesHeader(AContext: TPasElement);
begin
  DescrWriteLinebreak;
  DescrBeginBold;
  DescrWriteText(UTF8Decode(SDocNotes));
  DescrEndBold;
  DescrWriteLinebreak;
end;

procedure TFPDocWriter.DescrEmitNotesFooter(AContext: TPasElement);
begin
  DescrWriteLinebreak;
end;


Constructor TTopicElement.Create(const AName: String; AParent: TPasElement);

begin
  Inherited Create(AName,AParent);
  SubTopics:=TList.Create;
end;

Destructor TTopicElement.Destroy;

begin
  // Actual subtopics are freed by TFPDocWriter Topics list.
  SubTopics.Free;
  Inherited;
end;

function TFPDocWriter.WriteDescr ( Element: TPasElement ) : TDocNode;

begin
  Result:=Engine.FindDocNode(Element);
  WriteDescr(ELement,Result);
end;

procedure TFPDocWriter.WriteDescr(Element: TPasElement; DocNode: TDocNode);

begin
  if Assigned(DocNode) then
    begin
    if not IsDescrNodeEmpty(DocNode.Descr) then
      WriteDescr(Element, DocNode.Descr)
    else if not IsDescrNodeEmpty(DocNode.ShortDescr) then
      WriteDescr(Element, DocNode.ShortDescr);
    end;
end;

procedure TFPDocWriter.WriteDescr(AContext: TPasElement; DescrNode: TDOMElement);
begin
  if Assigned(DescrNode) then
    ConvertDescr(AContext, DescrNode, False);
end;

procedure TFPDocWriter.FPDocError(Msg: String);
begin
  Raise EFPDocWriterError.Create(Msg);
end;

procedure TFPDocWriter.FPDocError(Fmt: String; Args: array of const);
begin
  FPDocError(Format(Fmt,Args));
end;

function TFPDocWriter.ShowMember(M: TPasElement): boolean;
begin
  Result:=not ((M.Visibility=visPrivate) and Engine.HidePrivate);
  If Result then
    Result:=Not ((M.Visibility=visProtected) and Engine.HideProtected)
end;

procedure TFPDocWriter.GetMethodList ( ClassDecl: TPasClassType;
  List: TStringList ) ;

Var
  I : Integer;
  M : TPasElement;

begin
  List.Clear;
  List.Sorted:=False;
  for i := 0 to ClassDecl.Members.Count - 1 do
    begin
    M:=TPasElement(ClassDecl.Members[i]);
    if M.InheritsFrom(TPasProcedureBase) and ShowMember(M) then
       List.AddObject(M.Name,M);
    end;
  List.Sorted:=False;
end;




initialization
  InitWriterList;
finalization
  DoneWriterList;
end.
