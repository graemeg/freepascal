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

uses Classes, DOM, dGlobals, PasTree, SysUtils, fpdocclasstree;

resourcestring
  SErrFileWriting = 'An error occurred during writing of file "%s": %s';

  SErrInvalidShortDescr = 'Invalid short description';
  SErrInvalidDescr = 'Invalid description (illegal XML element: "%s")';
  SErrInvalidParaContent = 'Invalid paragraph content';
  SErrInvalidElementInList = 'Invalid element in list - only "li" allowed';
  SErrInvalidListContent = 'Invalid list content';
  SErrInvalidRemarkContent = 'Invalid <remark> content (illegal XML element: "%s")';
  SErrListIsEmpty = 'List is empty - need at least one "li" element';
  SErrInvalidDefinitionTermContent = 'Invalid content in definition term';
  SErrDefinitionEntryMissing = 'Definition entry after definition term is missing';
  SErrInvalidBorderValue = 'Invalid "border" value for %s';
  SErrInvalidTableContent = 'Invalid table content';
  SErrTableRowEmpty = 'Table row is empty (no "td" elements found)';
  SErrInvalidContentBeforeSectionTitle = 'Invalid content before section title';
  SErrSectionTitleExpected = 'Section title ("title" element) expected';

  SErrDescrTagUnknown = 'Warning: Unknown tag "%s" in description';
  SErrUnknownEntityReference = 'Warning: Unknown entity reference "&%s;" found';
  SErrUnknownLinkID = 'Warning: Target ID of <link> in unit "%s", element "%s", is unknown: "%s"';
  SErrUnknownPrintShortID = 'Warning: Target ID of <printshort> is unknown: "%s"';
  SErrUnknownLink = 'Could not resolve link to "%s"';
  SErralreadyRegistered = 'Class for output format "%s" already registered';
  SErrUnknownWriterClass = 'Unknown output format "%s"';

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
    Property CurrentContext : TPasElement Read FContext ;
  public
    Constructor Create(APackage: TPasPackage; AEngine: TFPDocEngine); virtual;
    destructor Destroy;  override;
    property Engine : TFPDocEngine read FEngine;
    Property Package : TPasPackage read FPackage;
    Property Topics : TList Read FTopics;
    Property ImageExtension : String Read FImgExt Write FImgExt;
    // Should return True if option was succesfully interpreted.
    Function InterpretOption(Const Cmd,Arg : String) : Boolean; Virtual;
    Class Function FileNameExtension : String; virtual;
    Class Procedure Usage(List : TStrings); virtual;
    Class procedure SplitImport(var AFilename, ALinkPrefix: String); virtual;
    procedure WriteDoc; virtual; Abstract;
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

  TFPDocWriterClass = Class of TFPDocWriter;
  EFPDocWriterError = Class(Exception);

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

implementation


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
  TreeClass:= TClassTreeBuilder.Create(FEngine, FPackage, okClass);
  TreeInterface:= TClassTreeBuilder.Create(FEngine, FPackage, okInterface);
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
      TreeClass.BuildTree(L);
      TreeInterface.BuildTree(L);
      {$IFDEF TREE_TEST}
      TreeClass.SaveToXml('TreeClass.xml');
      TreeInterface.SaveToXml('TreeInterface.xml');
      {$ENDIF}
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
