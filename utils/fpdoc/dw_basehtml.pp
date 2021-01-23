{
    FPDoc  -  Free Pascal Documentation Tool
    Copyright (C) 2021 by Michael Van Canneyt

    * Basic HTML output generator. No assumptions about document/documentation structure

    See the file COPYING, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}

unit dw_basehtml;

{$mode objfpc}{$H+}

interface

uses Classes, DOM, DOM_HTML, dGlobals, PasTree, dWriter;


type

  { THTMLWriter }

  { TBaseHTMLWriter }

  TBaseHTMLWriter = class(TMultiFileDocWriter)
  Private
    FImageFileList: TStrings;
    FContentElement : THTMLELement;
    FInsideHeadRow: Boolean;
    FOutputNodeStack: TFPList;
    FBaseImageURL : String;
    FDoc: THTMLDocument;
    FCurOutputNode: TDOMNode;
    FDoPasHighlighting : Boolean;
    FHighlighterFlags: Byte;
  Protected

    Procedure SetContentElement(aElement : THTMLELement); virtual;
    // Description node conversion
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

    // Basic HTML handling
    Procedure SetHTMLDocument(aDoc : THTMLDocument);
    procedure PushOutputNode(ANode: TDOMNode);
    procedure PopOutputNode;
    procedure AppendText(Parent: TDOMNode; const AText: String);
    procedure AppendText(Parent: TDOMNode; const AText: DOMString);
    procedure AppendNbSp(Parent: TDOMNode; ACount: Integer);
    procedure AppendSym(Parent: TDOMNode; const AText: DOMString);
    procedure AppendKw(Parent: TDOMNode; const AText: String);
    procedure AppendKw(Parent: TDOMNode; const AText: DOMString);
    function  AppendPasSHFragment(Parent: TDOMNode; const AText: String; AShFlags: Byte): Byte;
    procedure AppendFragment(aParentNode: TDOMElement; aStream: TStream);
    // FPDoc specifics
    procedure AppendSourceRef(aParent: TDOMElement; AElement: TPasElement);
    Procedure AppendSeeAlsoSection(AElement: TPasElement; DocNode: TDocNode); virtual;
    Procedure AppendExampleSection(AElement : TPasElement;DocNode : TDocNode); virtual;
    Procedure AppendShortDescr(Parent: TDOMNode; Element: TPasElement);
    procedure AppendShortDescr(AContext: TPasElement; Parent: TDOMNode; DocNode: TDocNode);
    procedure AppendShortDescrCell(Parent: TDOMNode; Element: TPasElement);
    procedure AppendDescr(AContext: TPasElement; Parent: TDOMNode; DescrNode: TDOMElement; AutoInsertBlock: Boolean);
    procedure AppendDescrSection(AContext: TPasElement; Parent: TDOMNode; DescrNode: TDOMElement; const ATitle: DOMString);
    procedure AppendDescrSection(AContext: TPasElement; Parent: TDOMNode; DescrNode: TDOMElement; const ATitle: String);
    function AppendHyperlink(Parent: TDOMNode; Element: TPasElement): TDOMElement;

    // Helper functions for creating DOM elements

    function CreateEl(Parent: TDOMNode; const AName: DOMString): THTMLElement;
    function CreatePara(Parent: TDOMNode): THTMLElement;
    function CreateH1(Parent: TDOMNode): THTMLElement;
    function CreateH2(Parent: TDOMNode): THTMLElement;
    function CreateH3(Parent: TDOMNode): THTMLElement;
    function CreateTable(Parent: TDOMNode; const AClass: DOMString = ''): THTMLElement;
    function CreateContentTable(Parent: TDOMNode): THTMLElement;
    function CreateTR(Parent: TDOMNode): THTMLElement;
    function CreateTD(Parent: TDOMNode): THTMLElement;
    function CreateTD_vtop(Parent: TDOMNode): THTMLElement;
    function CreateLink(Parent: TDOMNode; const AHRef: String): THTMLElement;
    function CreateLink(Parent: TDOMNode; const AHRef: DOMString): THTMLElement;
    function CreateAnchor(Parent: TDOMNode; const AName: DOMString): THTMLElement;
    function CreateCode(Parent: TDOMNode): THTMLElement;
    function CreateWarning(Parent: TDOMNode): THTMLElement;


    // Some info
    Property ContentElement : THTMLELement Read FContentElement Write SetContentElement;
    Property OutputNodeStack: TFPList Read FOutputNodeStack;
    Property CurOutputNode : TDomNode Read FCurOutputNode;
    Property ImageFileList : TStrings Read FImageFileList;
    Property Doc: THTMLDocument Read FDoc;
    Property InsideHeadRow: Boolean Read FInsideHeadRow;
    Property DoPasHighlighting : Boolean Read FDoPasHighlighting;
    Property HighlighterFlags : Byte read FHighlighterFlags;

  Public
    constructor Create(APackage: TPasPackage; AEngine: TFPDocEngine); override;
    Destructor Destroy; override;
    Property BaseImageURL : String Read FBaseImageURL Write FBaseImageURL;
  end;

Function FixHTMLpath(S : String) : STring;

implementation

uses fpdocstrs, xmlread, sysutils, sh_pas;

Function FixHTMLpath(S : String) : STring;

begin
  Result:=StringReplace(S,'\','/',[rfReplaceAll]);
end;

constructor TBaseHTMLWriter.Create(APackage: TPasPackage; AEngine: TFPDocEngine);

begin
  Inherited;
  FOutputNodeStack := TFPList.Create;
  FImageFileList:=TStringList.Create;
end;

destructor TBaseHTMLWriter.Destroy;
begin
  FreeAndNil(FOutputNodeStack);
  FreeAndNil(FImageFileList);
  inherited Destroy;
end;

Procedure TBaseHTMLWriter.SetContentElement(aElement : THTMLELement);

begin
  FContentElement:=aElement;
end;

function TBaseHTMLWriter.CreateEl(Parent: TDOMNode;
  const AName: DOMString): THTMLElement;
begin
  Result := Doc.CreateElement(AName);
  Parent.AppendChild(Result);
end;

function TBaseHTMLWriter.CreatePara(Parent: TDOMNode): THTMLElement;
begin
  Result := CreateEl(Parent, 'p');
end;

function TBaseHTMLWriter.CreateH1(Parent: TDOMNode): THTMLElement;
begin
  Result := CreateEl(Parent, 'h1');
end;

function TBaseHTMLWriter.CreateH2(Parent: TDOMNode): THTMLElement;
begin
  Result := CreateEl(Parent, 'h2');
end;

function TBaseHTMLWriter.CreateH3(Parent: TDOMNode): THTMLElement;
begin
  Result := CreateEl(Parent, 'h3');
end;

function TBaseHTMLWriter.CreateTable(Parent: TDOMNode; const AClass: DOMString = ''): THTMLElement;
begin
  Result := CreateEl(Parent, 'table');
  Result['cellspacing'] := '0';
  Result['cellpadding'] := '0';
  if AClass <> '' then
    Result['class'] := AClass;
end;

function TBaseHTMLWriter.CreateContentTable(Parent: TDOMNode): THTMLElement;
begin
  Result := CreateEl(Parent, 'table');
end;

function TBaseHTMLWriter.CreateTR(Parent: TDOMNode): THTMLElement;
begin
  Result := CreateEl(Parent, 'tr');
end;

function TBaseHTMLWriter.CreateTD(Parent: TDOMNode): THTMLElement;
begin
  Result := CreateEl(Parent, 'td');
end;

function TBaseHTMLWriter.CreateTD_vtop(Parent: TDOMNode): THTMLElement;
begin
  Result := CreateEl(Parent, 'td');
  Result['valign'] := 'top';
end;

function TBaseHTMLWriter.CreateLink(Parent: TDOMNode; const AHRef: String): THTMLElement;
begin
  Result := CreateEl(Parent, 'a');
  Result['href'] := UTF8Decode(FixHtmlPath(AHRef));
end;

function TBaseHTMLWriter.CreateLink(Parent: TDOMNode;
  const AHRef: DOMString): THTMLElement;
begin
  Result:=CreateLink(Parent,UTF8Encode(aHREf));
end;

function TBaseHTMLWriter.CreateAnchor(Parent: TDOMNode;
  const AName: DOMString): THTMLElement;
begin
  Result := CreateEl(Parent, 'a');
  Result['name'] := AName;
end;

function TBaseHTMLWriter.CreateCode(Parent: TDOMNode): THTMLElement;
begin
  Result := CreateEl(CreateEl(Parent, 'tt'), 'span');
  Result['class'] := 'code';
end;

function TBaseHTMLWriter.CreateWarning(Parent: TDOMNode): THTMLElement;
begin
  Result := CreateEl(Parent, 'span');
  Result['class'] := 'warning';
end;

procedure TBaseHTMLWriter.DescrEmitNotesHeader(AContext: TPasElement);
begin
  AppendText(CreateH2(ContentElement), SDocNotes);
  PushOutputNode(ContentElement);
end;

procedure TBaseHTMLWriter.DescrEmitNotesFooter(AContext: TPasElement);
begin
  PopOutPutNode;
end;

procedure TBaseHTMLWriter.PushOutputNode(ANode: TDOMNode);
begin
  OutputNodeStack.Add(CurOutputNode);
  FCurOutputNode := ANode;
end;

procedure TBaseHTMLWriter.PopOutputNode;
begin
  FCurOutputNode := TDOMNode(OutputNodeStack[OutputNodeStack.Count - 1]);
  OutputNodeStack.Delete(OutputNodeStack.Count - 1);
end;

procedure TBaseHTMLWriter.DescrWriteText(const AText: DOMString);
begin
  AppendText(CurOutputNode, AText);
end;

procedure TBaseHTMLWriter.DescrBeginBold;
begin
  PushOutputNode(CreateEl(CurOutputNode, 'b'));
end;

procedure TBaseHTMLWriter.DescrEndBold;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginItalic;
begin
  PushOutputNode(CreateEl(CurOutputNode, 'i'));
end;

procedure TBaseHTMLWriter.DescrEndItalic;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginEmph;
begin
  PushOutputNode(CreateEl(CurOutputNode, 'em'));
end;

procedure TBaseHTMLWriter.DescrEndEmph;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginUnderline;
begin
  PushOutputNode(CreateEl(CurOutputNode, 'u'));
end;

procedure TBaseHTMLWriter.DescrEndUnderline;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrWriteImageEl(const AFileName, ACaption, ALinkName : DOMString);

Var
  Pel,Cel: TDOMNode;
  El :TDomElement;
  D : String;
  L : Integer;

begin
  // Determine parent node.
  If (ACaption='') then
    Pel:=CurOutputNode
  else
    begin
    Cel:=CreateTable(CurOutputNode, 'imagetable');
    Pel:=CreateTD(CreateTR(Cel));
    Cel:=CreateTD(CreateTR(Cel));
    El := CreateEl(Cel, 'span');
    El['class'] := 'imagecaption';
    Cel := El;
    If (ALinkName<>'') then
      Cel:=CreateAnchor(Cel,ALinkName);
    AppendText(Cel,ACaption);
    end;

  // Determine URL for image.
  If (Module=Nil) then
    D:=Allocator.GetRelativePathToTop(Package)
  else
    D:=Allocator.GetRelativePathToTop(Module);
  L:=Length(D);
  If (L>0) and (D[L]<>'/') then
    D:=D+'/';

  // Create image node.
  El:=CreateEl(Pel,'img');
  EL['src']:=UTF8Decode(D + BaseImageURL) + AFileName;
  El['alt']:=ACaption;

  //cache image filename, so it can be used later (CHM)
  ImageFileList.Add(UTF8Encode(UTF8Decode(BaseImageURL) + AFileName));
end;

procedure TBaseHTMLWriter.DescrWriteFileEl(const AText: DOMString);
var
  NewEl: TDOMElement;
begin
  NewEl := CreateEl(CurOutputNode, 'span');
  NewEl['class'] := 'file';
  AppendText(NewEl, AText);
end;

procedure TBaseHTMLWriter.DescrWriteKeywordEl(const AText: DOMString);
var
  NewEl: TDOMElement;
begin
  NewEl := CreateEl(CurOutputNode, 'span');
  NewEl['class'] := 'kw';
  AppendText(NewEl, AText);
end;

procedure TBaseHTMLWriter.DescrWriteVarEl(const AText: DOMString);
begin
  AppendText(CreateEl(CurOutputNode, 'var'), AText);
end;

procedure TBaseHTMLWriter.DescrBeginLink(const AId: DOMString);
var
  a,s,n : String;

begin
  a:=UTF8Encode(AId);
  s := UTF8Encode(ResolveLinkID(a));
  if Length(s) = 0 then
  begin
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
    PushOutputNode(CreateEl(CurOutputNode, 'b'));
  end else
    PushOutputNode(CreateLink(CurOutputNode, s));
end;

procedure TBaseHTMLWriter.DescrEndLink;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginURL(const AURL: DOMString);
begin
  PushOutputNode(CreateLink(CurOutputNode, AURL));
end;

procedure TBaseHTMLWriter.DescrEndURL;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrWriteLinebreak;
begin
  CreateEl(CurOutputNode, 'br');
end;

procedure TBaseHTMLWriter.DescrBeginParagraph;
begin
  PushOutputNode(CreatePara(CurOutputNode));
end;

procedure TBaseHTMLWriter.DescrEndParagraph;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginCode(HasBorder: Boolean; const AHighlighterName: String);
begin
  FDoPasHighlighting := (AHighlighterName = '') or (AHighlighterName = 'Pascal');
  FHighlighterFlags := 0;
  PushOutputNode(CreateEl(CurOutputNode, 'pre'));
end;

procedure TBaseHTMLWriter.DescrWriteCodeLine(const ALine: String);
begin
  if DoPasHighlighting then
  begin
    FHighlighterFlags := AppendPasSHFragment(CurOutputNode, ALine,FHighlighterFlags);
    AppendText(CurOutputNode, #10);
  end else
    AppendText(CurOutputNode, ALine + #10);
end;

procedure TBaseHTMLWriter.DescrEndCode;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginOrderedList;
begin
  PushOutputNode(CreateEl(CurOutputNode, 'ol'));
end;

procedure TBaseHTMLWriter.DescrEndOrderedList;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginUnorderedList;
begin
  PushOutputNode(CreateEl(CurOutputNode, 'ul'));
end;

procedure TBaseHTMLWriter.DescrEndUnorderedList;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginDefinitionList;
begin
  PushOutputNode(CreateEl(CurOutputNode, 'dl'));
end;

procedure TBaseHTMLWriter.DescrEndDefinitionList;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginListItem;
begin
  PushOutputNode(CreateEl(CurOutputNode, 'li'));
end;

procedure TBaseHTMLWriter.DescrEndListItem;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginDefinitionTerm;
begin
  PushOutputNode(CreateEl(CurOutputNode, 'dt'));
end;

procedure TBaseHTMLWriter.DescrEndDefinitionTerm;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginDefinitionEntry;
begin
  PushOutputNode(CreateEl(CurOutputNode, 'dd'));
end;

procedure TBaseHTMLWriter.DescrEndDefinitionEntry;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginSectionTitle;
begin
  PushOutputNode(CreateEl(CurOutputNode, 'h3'));
end;

procedure TBaseHTMLWriter.DescrBeginSectionBody;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrEndSection;
begin
end;

procedure TBaseHTMLWriter.DescrBeginRemark;
var
  NewEl, TDEl: TDOMElement;
begin
  NewEl := CreateEl(CurOutputNode, 'table');
  NewEl['width'] := '100%';
  NewEl['border'] := '0';
  NewEl['CellSpacing'] := '0';
  NewEl['class'] := 'remark';
  NewEl := CreateTR(NewEl);
  TDEl := CreateTD(NewEl);
  TDEl['valign'] := 'top';
  TDEl['class'] := 'pre';
  AppendText(CreateEl(TDEl, 'b'), SDocRemark);
  PushOutputNode(CreateTD(NewEl));
end;

procedure TBaseHTMLWriter.DescrEndRemark;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginTable(ColCount: Integer; HasBorder: Boolean);
var
  Table: TDOMElement;
begin
  Table := CreateEl(CurOutputNode, 'table');
  Table['border'] := UTF8Decode(IntToStr(Ord(HasBorder)));
  PushOutputNode(Table);
end;

procedure TBaseHTMLWriter.DescrEndTable;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginTableCaption;
begin
  PushOutputNode(CreateEl(CurOutputNode, 'caption'));
end;

procedure TBaseHTMLWriter.DescrEndTableCaption;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginTableHeadRow;
begin
  PushOutputNode(CreateTr(CurOutputNode));
  FInsideHeadRow := True;
end;

procedure TBaseHTMLWriter.DescrEndTableHeadRow;
begin
  FInsideHeadRow := False;
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginTableRow;
begin
  PushOutputNode(CreateTR(CurOutputNode));
end;

procedure TBaseHTMLWriter.DescrEndTableRow;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.DescrBeginTableCell;
begin
  if InsideHeadRow then
    PushOutputNode(CreateEl(CurOutputNode, 'th'))
  else
    PushOutputNode(CreateTD(CurOutputNode));
end;

procedure TBaseHTMLWriter.DescrEndTableCell;
begin
  PopOutputNode;
end;

procedure TBaseHTMLWriter.SetHTMLDocument(aDoc: THTMLDocument);
begin
  FDoc:=aDoc;
  FOutputNodeStack.Clear;
  FCurOutputNode:=Nil;
end;

procedure TBaseHTMLWriter.AppendText(Parent: TDOMNode; const AText: String);
begin
  AppendText(Parent,UTF8Decode(aText));
end;


procedure TBaseHTMLWriter.AppendText(Parent: TDOMNode; const AText: DOMString);
begin
  Parent.AppendChild(Doc.CreateTextNode(AText));
end;

procedure TBaseHTMLWriter.AppendNbSp(Parent: TDOMNode; ACount: Integer);
begin
  while ACount > 0 do
  begin
    Parent.AppendChild(Doc.CreateEntityReference('nbsp'));
    Dec(ACount);
  end;
end;

procedure TBaseHTMLWriter.AppendSym(Parent: TDOMNode; const AText: DOMString);
var
  El: TDOMElement;
begin
  El := CreateEl(Parent, 'span');
  El['class'] := 'sym';
  AppendText(El, AText);
end;

procedure TBaseHTMLWriter.AppendKw(Parent: TDOMNode; const AText: String);
begin
  AppendKW(Parent,UTF8Decode(aText));
end;

procedure TBaseHTMLWriter.AppendKw(Parent: TDOMNode; const AText: DOMString);
var
  El: TDOMElement;
begin
  El := CreateEl(Parent, 'span');
  El['class'] := 'kw';
  AppendText(El, AText);
end;

function TBaseHTMLWriter.AppendPasSHFragment(Parent: TDOMNode;
  const AText: String; AShFlags: Byte): Byte;


var
  Line, Last, p: PChar;
  El: TDOMElement;

  Procedure MaybeOutput;

  Var
    CurParent: TDomNode;

  begin
    If (Last<>Nil) then
      begin
      If (el<>Nil) then
        CurParent:=El
      else
        CurParent:=Parent;
      AppendText(CurParent,Last);
      El:=Nil;
      Last:=Nil;
      end;
  end;

  Function NewEl(Const ElType,Attr,AttrVal : DOMString) : TDomElement;

  begin
    Result:=CreateEl(Parent,ElType);
    Result[Attr]:=AttrVal;
  end;

  Function NewSpan(Const AttrVal : DOMString) : TDomElement;

  begin
    Result:=CreateEl(Parent,'span');
    Result['class']:=AttrVal;
  end;

begin
  GetMem(Line, Length(AText) * 3 + 4);
  Try
  DoPascalHighlighting(AShFlags, PChar(AText), Line);
  Result := AShFlags;
  Last := Nil;
  p := Line;
  el:=nil;
  while p[0] <> #0 do
  begin
    if p[0] = LF_ESCAPE then
      begin
      p[0] := #0;
      MaybeOutput;
      case Ord(p[1]) of
        shDefault:    El:=Nil;
        shInvalid:    El:=newel('font','color','red');
        shSymbol :    El:=newspan('sym');
        shKeyword:    El:=newspan('kw');
        shComment:    El:=newspan('cmt');
        shDirective:  El:=newspan('dir');
        shNumbers:    El:=newspan('num');
        shCharacters: El:=newspan('chr');
        shStrings:    El:=newspan('str');
        shAssembler:  El:=newspan('asm');
      end;
      Inc(P);
      end
    else If (Last=Nil) then
      Last:=P;
    Inc(p);
  end;
  MaybeOutput;
  Finally
    FreeMem(Line);
  end;
end;


procedure TBaseHTMLWriter.AppendSeeAlsoSection ( AElement: TPasElement;
  DocNode: TDocNode ) ;

var
  Node: TDOMNode;
  TableEl, El, TREl, ParaEl, NewEl, DescrEl: TDOMElement;
  l,s,n: DOMString;
  IsFirstSeeAlso : Boolean;

begin
  if Not (Assigned(DocNode) and Assigned(DocNode.SeeAlso)) then
    Exit;
  IsFirstSeeAlso := True;
  Node:=DocNode.SeeAlso.FirstChild;
  While Assigned(Node) do
    begin
    if (Node.NodeType=ELEMENT_NODE) and (Node.NodeName='link') then
      begin
       if IsFirstSeeAlso then
         begin
         IsFirstSeeAlso := False;
         AppendText(CreateH2(ContentElement), SDocSeeAlso);
         TableEl := CreateTable(ContentElement);
         end;
       El:=TDOMElement(Node);
       TREl:=CreateTR(TableEl);
       ParaEl:=CreatePara(CreateTD_vtop(TREl));
       l:=El['id'];
       if Assigned(Engine) and Engine.FalbackSeeAlsoLinks then
         s:= ResolveLinkIDUnStrict(UTF8ENcode(l))
       else
         s:= ResolveLinkID(UTF8ENcode(l));
       if Length(s)=0 then
         begin
         if assigned(module) then
           s:=UTF8Decode(module.name)
         else
           s:='?';
         if l='' then l:='<empty>';
         if Assigned(AElement) then
           N:=UTF8Decode(AElement.PathName)
         else
           N:='?';
         DoLog(SErrUnknownLinkID, [s,N,l]);
         LinkUnresolvedInc();
         NewEl := CreateEl(ParaEl,'b')
         end
       else
         NewEl := CreateLink(ParaEl,s);
        if Not IsDescrNodeEmpty(El) then
          begin
          PushOutputNode(NewEl);
          Try
            ConvertBaseShortList(AElement, El, True)
          Finally
            PopOutputNode;
          end;
          end
        else
          AppendText(NewEl,El['id']);
       l:=El['id'];
       DescrEl := Engine.FindShortDescr(AElement.GetModule,UTF8Encode(L));
       if Assigned(DescrEl) then
         begin
         AppendNbSp(CreatePara(CreateTD(TREl)), 2);
         ParaEl := CreatePara(CreateTD(TREl));
         ParaEl['class'] := 'cmt';
         PushOutputNode(ParaEl);
         try
           ConvertShort(AElement, DescrEl);
         finally
           PopOutputNode;
         end;
         end;
       end; // Link node
     Node := Node.NextSibling;
     end; // While
end;

procedure TBaseHTMLWriter.AppendExampleSection ( AElement: TPasElement; DocNode: TDocNode ) ;

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
        AppendText(CreateH2(ContentElement), SDocExample);
        try
          Assign(f, FN);
          Reset(f);
          try
            PushOutputNode(ContentElement);
            DescrBeginCode(False, UTF8Encode(TDOMElement(Node)['highlighter']));
            while not EOF(f) do
              begin
              ReadLn(f, s);
              DescrWriteCodeLine(s);
              end;
            DescrEndCode;
            PopOutputNode;
          finally
            Close(f);
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

procedure TBaseHTMLWriter.AppendFragment(aParentNode : TDOMElement; aStream : TStream);

begin
  if (aStream<>Nil) then
    begin
    aStream.Position:=0;
    ReadXMLFragment(aParentNode,aStream);
    end;
end;

procedure TBaseHTMLWriter.AppendShortDescr ( AContext: TPasElement;
  Parent: TDOMNode; DocNode: TDocNode ) ;

Var
  N : TDocNode;

begin
  if Assigned(DocNode) then
    begin
    If (DocNode.Link<>'') then
      begin
      N:=Engine.FindLinkedNode(DocNode);
      If (N<>Nil) then
        DocNode:=N;
      end;
    If Assigned(DocNode.ShortDescr) then
      begin
      PushOutputNode(Parent);
      try
        if not ConvertShort(AContext,TDomElement(DocNode.ShortDescr)) then
          Warning(AContext, SErrInvalidShortDescr)
      finally
        PopOutputNode;
      end;
      end;
    end;
end;

procedure TBaseHTMLWriter.AppendShortDescr(Parent: TDOMNode; Element: TPasElement);

begin
  AppendShortDescr(Element,Parent,Engine.FindDocNode(Element));
end;

procedure TBaseHTMLWriter.AppendShortDescrCell(Parent: TDOMNode;  Element: TPasElement);

var
  ParaEl: TDOMElement;

begin
  if Assigned(Engine.FindShortDescr(Element)) then
  begin
    AppendNbSp(CreatePara(CreateTD(Parent)), 2);
    ParaEl := CreatePara(CreateTD(Parent));
    ParaEl['class'] := 'cmt';
    AppendShortDescr(ParaEl, Element);
  end;
end;

procedure TBaseHTMLWriter.AppendDescr(AContext: TPasElement; Parent: TDOMNode;
  DescrNode: TDOMElement; AutoInsertBlock: Boolean);
begin
  if Assigned(DescrNode) then
  begin
    PushOutputNode(Parent);
    try
      ConvertDescr(AContext, DescrNode, AutoInsertBlock);
    finally
      PopOutputNode;
    end;
  end;
end;

procedure TBaseHTMLWriter.AppendDescrSection(AContext: TPasElement; Parent: TDOMNode; DescrNode: TDOMElement; const ATitle: String);
begin
  AppendDescrSection(aContext,Parent,DescrNode,UTF8Decode(aTitle));
end;

procedure TBaseHTMLWriter.AppendDescrSection(AContext: TPasElement;
  Parent: TDOMNode; DescrNode: TDOMElement; const ATitle: DOMString);
begin
  if not IsDescrNodeEmpty(DescrNode) then
  begin
    If (ATitle<>'') then // Can be empty for topic.
      AppendText(CreateH2(Parent), ATitle);
    AppendDescr(AContext, Parent, DescrNode, True);
  end;
end;

function TBaseHTMLWriter.AppendHyperlink(Parent: TDOMNode; Element: TPasElement): TDOMElement;
var
  s: DOMString;
  UnitList: TFPList;
  i: Integer;
  ThisPackage: TLinkNode;
begin
  if Not Assigned(Element) then
    begin
    Result := nil;
    AppendText(CreateWarning(Parent), '<NIL>');
    end;
  if Element.InheritsFrom(TPasUnresolvedTypeRef) then
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
    end
  else if Element is TPasEnumValue then
    s := ResolveLinkID(Element.Parent.PathName)
  else if Element is TPasAliasType then
    s := ResolveLinkID(TPasAliasType(Element).DestType.PathName)
  else
    s := ResolveLinkID(Element.PathName);

  if Length(s) > 0 then
    begin
    Result := CreateLink(Parent, s);
    AppendText(Result, Element.Name);
    end
  else
    begin
    Result := nil;
    if  Element is TPasAliasType then
      AppendText(Parent, TPasAliasType(Element).DestType.Name)
    else
      AppendText(Parent, Element.Name); // unresolved items
    end;
end;

procedure TBaseHTMLWriter.AppendSourceRef(aParent : TDOMElement; AElement: TPasElement);

begin
  AppendText(CreatePara(aParent), Format(SDocSourcePosition,
    [ExtractFileName(AElement.SourceFilename), AElement.SourceLinenumber]));
end;


end.

