{$mode objfpc}
{$H+}
unit dw_man;
{$WARN 5024 off : Parameter "$1" not used}
interface

uses
  Classes, SysUtils, DGlobals, dWriter, pastree, dom;

Const
  DefaultManSection = 3;
  MaxListLevel      = 4;

  // Suffixes for overview man pages.
  SManConsts  = 'consts';
  SManVars    = 'variables';
  SManTypes   = 'types';
  SManResStr  = 'resstr';

  // Standard man sections.
  SManDocName            = 'NAME';
  SManDocSynopsis        = 'SYNOPSIS';
  SManDocDescription     = 'DESCRIPTION';
  SManDocErrors          = 'ERRORS';
  SManDocSeeAlso         = 'SEE ALSO';

  // FPDoc man sections.
  SManDocPackageUnits    = 'PACKAGE UNITS';
  SManDocUsedUnits       = 'USED UNITS';
  SManDocResourceStrings = 'RESOURCE STRINGS';
  SManDocVariables       = 'VARIABLES';
  SManDocTypes           = 'TYPES';
  SManDocConstants       = 'CONSTANTS';
  SManDocFunctions       = 'PROCEDURES AND FUNCTIONS';
  SManDocClasses         = 'CLASSES';
  SManDocExamples        = 'EXAMPLES';
  SManDocVisibility      = 'VISIBILITY';
  SManDocArguments       = 'ARGUMENTS';
  SManDocResult          = 'RETURN VALUE';
  SManDocAccess          = 'ACCESSIBILITY';
  SManDocMethods         = 'METHODS';
  SManDocProperties      = 'PROPERTIES';

  // Used to start listing
  SManDocListing         = 'Listing:';

Type
  { TManWriter }

  TManWriter = Class(TFPDocWriter)
    SkipUnitPrefix,
    FSkipTrim : Boolean;
    OutputDir,
    ModuleName,
    ManSection,
    PackageDescr,
    PackageName: String;
    FAtLineStart,
    FCheckEOL : Boolean;
    FStream : TStream;
    Module: TPasModule;
    FListLevel : Integer;
    FLists : Array [0..MaxListLevel] of integer;
  Protected
    // Writing support.
    procedure Write(const s: String);
    procedure WriteF(const s: String; const Args: array of const);
    procedure WriteLn(const s: String);
    procedure WriteLnF(const s: String; const Args: array of const);
    Procedure NewLine;
    Function  PushWriteContext(S : TStream) : TStream;
    Procedure PopWriteContext(S : TSTream);
    Function  EscapeText(const s : String) : String;
    // Formatting
    procedure WriteTP;
    procedure WriteB(Const Msg : String);
    procedure WriteBI(Const Msg : String);
    procedure NewListLevel(Initial : Integer);
    procedure DecListLevel;
    procedure StartListing(Frames: Boolean);
    // Sectioning routines
    Procedure StartManPage(AElement : TPasElement; ADocNode : TDocNode);
    Procedure StartManPage(FN : String);
    Procedure EndManPage;
    procedure StartSection(Const SectionName : String);
    procedure StartSubSection(Const SubSectionName : String);
    procedure PageTitle(Const ATitle,ASection,ASource,Amanual : String);
    // Referencing
    Function  ElementToManPage(APasElement : TPasElement) : String;
    procedure WriteManRef(Const ManPage : String; Comma : Boolean);
    procedure WriteManRef(APasElement : TPasElement; Comma : Boolean);
    procedure WriteModuleSeealso(Comma : Boolean);

    procedure SortElementList(List : TFPList);
    Function  GetDescrString(AContext: TPasElement; DescrNode: TDOMElement) : String;
    function  ConstValue(ConstDecl: TPasConst): String; virtual;
    procedure WriteCommentLine;
    procedure WriteComment(Comment : String);
    Procedure WriteExampleFile(FN : String); virtual;
    procedure WriteExample(ADocNode: TDocNode);
    procedure WriteSeeAlso(ADocNode: TDocNode; Comma : Boolean);
    // Here we write the documentation.
    procedure DoWriteDocumentation; override;
  Public
    Constructor Create(APackage: TPasPackage; AEngine: TFPDocEngine); override;
    // Documentation writing methods.
    // Package
    Procedure WritePackagePage;
    // Topics
    Procedure ProcessTopics(DocNode : TDocNode; Subs : Boolean);
    Procedure WriteTopicRefs(DocNode : TDocNode);
    Procedure WriteTopicPage(Parent,Node : TDocNode);
    // Module
    procedure ProcessModule(AModule: TPasModule);
    procedure WriteUnitPage(AModule: TPasModule);
    procedure WriteUnitUsesOverview(ASection: TPasSection);
    procedure WriteUnitFunctionsAndProceduresOverview(ASection: TPasSection);
    procedure WriteUnitClassesOverview(ASection: TPasSection);
    procedure WriteUnitResourceStrings(ASection: TPasSection);
    procedure WriteUnitConsts(ASection: TPasSection);
    procedure WriteUnitTypes(ASection: TPasSection);
    procedure WriteUnitVars(ASection: TPasSection);
    procedure WriteUnitClasses(ASection: TPasSection);
    procedure WriteUnitFunctionsAndProcedures(ASection: TPasSection);
    // Smaller elements
    procedure WriteEnumElements(TypeDecl : TPasEnumType);
    procedure WriteClassPage(ClassDecl: TPasClassType);
    procedure WriteClassMethodOverview(ClassDecl: TPasClassType);
    procedure WriteClassPropertyOverview(ClassDecl: TPasClassType);
    procedure WriteProcedurePage(ProcDecl: TPasProcedureBase);
    procedure AppendProcedureArgsSection(Element: TPasProcedureType);
    procedure AppendFunctionResultSection(Element: TPasFunctionType);
    procedure WritePropertyPage(PropDecl: TPasProperty);
    // Overriden from fpdocwriter;
    procedure DescrWriteText(const AText: DOMString); override;
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
    Function InterPretOption(Const Cmd,Arg : String) : boolean; override;
    Class Function FileNameExtension : String; override;
    Class procedure Usage(List: TStrings); override;
  end;

implementation

uses fpdocstrs;

{ TManWriter }

constructor TManWriter.Create(APackage: TPasPackage; AEngine: TFPDocEngine);

  procedure AddLabel(AElement: TPasElement);
  begin
    Engine.AddLink(AElement.PathName, ElementToManPage(AElement));
  end;

  procedure AddList(AElement: TPasElement; AList: TFPList);
  var
    i: Integer;
  begin
    for i := 0 to AList.Count - 1 do
      AddLabel(TPasElement(AList[i]));
  end;

  procedure AddTopicPages(AElement: TPasElement);

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
        Engine.AddLink(TopicElement.PathName, ElementToManPage(TopicElement));
        if AElement is TTopicElement then
          TTopicElement(AElement).SubTopics.Add(TopicElement)
        else // Only one level of recursion.
          AddTopicPages(TopicElement);
        end;
      TopicNode:=TopicNode.NextSibling;
      end;
  end;

  procedure ScanModule(AModule: TPasModule);
  var
    i, j, k: Integer;
    ClassEl: TPasClassType;
    FPEl, AncestorMemberEl: TPasElement;
    DocNode: TDocNode;
    DidAutolink: Boolean;
  begin
    AddLabel(AModule);
    AddTopicPages(AModule);
    with AModule do
    begin
      AddList(AModule, InterfaceSection.ResStrings);
      AddList(AModule, InterfaceSection.Consts);
      AddList(AModule, InterfaceSection.Types);
      if InterfaceSection.Classes.Count > 0 then
      begin
        for i := 0 to InterfaceSection.Classes.Count - 1 do
        begin
          ClassEl := TPasClassType(InterfaceSection.Classes[i]);
          AddLabel(ClassEl);

          for j := 0 to ClassEl.Members.Count - 1 do
          begin
            FPEl := TPasElement(ClassEl.Members[j]);
            if Not Engine.ShowElement(FPEl) then
              continue;

            DocNode := Engine.FindDocNode(FPEl);
            if not Assigned(DocNode) then
            begin
              DidAutolink := False;
              if Assigned(ClassEl.AncestorType) and
                (ClassEl.AncestorType.ClassType = TPasClassType) then
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
                AddLabel(FPEl);
            end else
              AddLabel(FPEl);
          end;
        end;
      end;
      AddList(AModule, InterfaceSection.Functions);
      AddList(AModule, InterfaceSection.Variables);
    end;
  end;

var
  i: Integer;
begin
  inherited;
  if Length(Package.Name) > 1 then
    AddTopicPages(Package);
  for i := 0 to Package.Modules.Count - 1 do
    ScanModule(TPasModule(Package.Modules[i]));
end;

{ ---------------------------------------------------------------------
  Writing support
  ---------------------------------------------------------------------}

Function TManWriter.PushWriteContext(S : TStream) : TStream;

begin
  Result:=FStream;
  FStream:=S;
end;

Procedure TManWriter.PopWriteContext(S : TSTream);

begin
  FStream:=S;
end;

function TManWriter.EscapeText(const s: String): String;
begin
  Result:=S;
end;

procedure TManWriter.Write(const s: String);

Var
  W : String;
  L : Integer;

begin
  W:=S;
  If FAtLineStart and not FSKipTrim then
    W:=TrimLeft(W);
  L:=Length(W);
  If (L>0) then
    FStream.Write(PChar(W)^,L);
  FAtLineStart:=false;
  If FCheckEOL then
    begin
    If (L>=LEOL) then
      FAtLineStart:=(Copy(W,L-LEOL+1,LEOL)=LineEnding);
    end;
end;

Procedure TManWriter.NewLine;

begin
  if Not FAtLineStart then
    Writeln('');
end;


procedure TManWriter.WriteF(const s: String; const Args: array of const);
begin
  Write(Format(S,Args));
end;

procedure TManWriter.WriteLn(const s: String);

begin
  FCheckEOL:=False;
  Try
    Write(S);
    Write(LineEnding);
    FAtLineStart:=True;
  finally
    FCheckEOL:=True;
  end;
end;

procedure TManWriter.WriteLnF(const s: String; const Args: array of const);
begin
  Write(Format(S,Args));
  Write(LineEnding);
end;

procedure TManWriter.DescrWriteText(const AText: DOMString);

begin
  self.Write(EscapeText(Utf8Encode(AText)));
end;

procedure TManWriter.DescrBeginBold;
begin
  NewLine;
  Write('.B ');
end;

procedure TManWriter.DescrEndBold;
begin
  NewLine;
end;

procedure TManWriter.DescrBeginItalic;
begin
  NewLine;
  Write('.I ');
end;

procedure TManWriter.DescrEndItalic;
begin
  NewLine;
end;

procedure TManWriter.DescrBeginEmph;
begin
  NewLine;
  Write('.I ');
end;

procedure TManWriter.DescrEndEmph;
begin
  NewLine;
end;

procedure TManWriter.DescrBeginUnderline;
begin
  NewLine;
  Write('.I '); //use ITALIC!
end;

procedure TManWriter.DescrEndUnderline;
begin
  NewLine;
end;

procedure TManWriter.DescrWriteFileEl(const AText: DOMString);

Var
  S : AnsiString;

begin
  NewLine;
  S:=UTF8Encode(AText);
  Writeln('.I '+S);
end;

procedure TManWriter.DescrWriteKeywordEl(const AText: DOMString);

Var
  S : AnsiString;

begin
  NewLine;
  S:=Utf8Encode(AText);
  Writeln('.B '+S);
end;

procedure TManWriter.DescrWriteVarEl(const AText: DOMString);

Var
  S : AnsiString;

begin
  NewLine;
  S:=Utf8Encode(AText);
  Writeln('.B '+S);
end;

procedure TManWriter.DescrBeginLink(const AId: DOMString);
begin
  // Do nothing
end;

procedure TManWriter.DescrEndLink;
begin
  // Do nothing
end;

procedure TManWriter.DescrWriteLinebreak;
begin
  NewLine;
  Writeln('.br');
end;

procedure TManWriter.DescrBeginParagraph;
begin
  NewLine;
  Writeln('.PP');
end;

procedure TManWriter.DescrBeginCode(HasBorder: Boolean;
  const AHighlighterName: String);
begin
  NewLine;
  Writeln('');
end;

procedure TManWriter.DescrWriteCodeLine(const ALine: String);
begin
  FSkipTrim:=True;
  Try
    Writeln(ALine);
  Finally
    FSkipTrim:=False;
  end;
  DescrWriteLinebreak;
end;

procedure TManWriter.DescrEndCode;
begin
  NewLine;
  Writeln('');
end;

procedure TManWriter.DescrEndParagraph;
begin
  NewLine;
  Writeln('');
end;

procedure TManWriter.NewListLevel(Initial : Integer);

begin
  Inc(FListLevel);
  If (FListLevel<MaxListLevel) then
    FLists[FListLevel]:=0;
  NewLine;
  Writeln('.RS');
end;

procedure TManWriter.DecListLevel;

begin
  NewLine;
  Writeln('.RE');
  If (FListLevel>0) then
    Dec(FListLevel)
end;

procedure TManWriter.DescrBeginOrderedList;
begin
  NewListLevel(0);
end;

procedure TManWriter.DescrEndOrderedList;
begin
  DecListLevel;
end;

procedure TManWriter.DescrBeginUnorderedList;
begin
  NewListLevel(-1);
end;

procedure TManWriter.DescrEndUnorderedList;
begin
  DecListlevel;
end;

procedure TManWriter.DescrBeginDefinitionList;
begin
  NewListLevel(-2);
end;

procedure TManWriter.DescrEndDefinitionList;
begin
  DecListLevel
end;

procedure TManWriter.DescrBeginListItem;
begin
  WriteTP
end;

procedure TManWriter.DescrEndListItem;
begin
  NewLine;
end;

procedure TManWriter.DescrBeginDefinitionTerm;
begin
  WriteTP;
  Write('.B ');
end;

procedure TManWriter.DescrEndDefinitionTerm;
begin
  NewLine;
end;

procedure TManWriter.DescrBeginDefinitionEntry;
begin
  NewLine;
end;

procedure TManWriter.DescrEndDefinitionEntry;
begin
  NewLine;
end;

procedure TManWriter.DescrBeginSectionTitle;
begin
  Write('.SH ');
end;

procedure TManWriter.DescrBeginSectionBody;
begin
  NewLine;
end;

procedure TManWriter.DescrEndSection;
begin
  NewLine;
end;

procedure TManWriter.DescrBeginRemark;
begin
  WriteTP;
  WriteB(SDocRemark);
end;

procedure TManWriter.DescrEndRemark;
begin
  NewLine;
end;

procedure TManWriter.DescrBeginTable(ColCount: Integer; HasBorder: Boolean);
begin
  NewLine;
end;

procedure TManWriter.DescrEndTable;
begin
  NewLine;
end;

procedure TManWriter.DescrBeginTableCaption;
begin
  NewLine;
end;

procedure TManWriter.DescrEndTableCaption;
begin
  NewLine;
end;

procedure TManWriter.DescrBeginTableHeadRow;
begin
  DescrBeginParagraph;
end;

procedure TManWriter.DescrEndTableHeadRow;
begin
  DescrEndParagraph;
end;

procedure TManWriter.DescrBeginTableRow;
begin
  DescrBeginParagraph;
end;

procedure TManWriter.DescrEndTableRow;
begin
  DescrEndParagraph;
end;

procedure TManWriter.DescrBeginTableCell;
begin
  // Do nothing
end;

procedure TManWriter.DescrEndTableCell;
begin
  Writeln(#9);
end;

function TManWriter.InterPretOption(const Cmd, Arg: String): boolean;
begin
  Result:=True;
  if (Cmd='--man-section') then
    ManSection:=Arg
  else if (Cmd='--man-description') then
    PackageDescr:=Arg
  else if (Cmd='--nounitprefix') then
    SkipUnitPrefix:=True
  else
    Result:=inherited InterPretOption(Cmd, Arg);
end;


Function TManWriter.GetDescrString(AContext: TPasElement; DescrNode: TDOMElement) : String;

Var
  S : TStringStream;
  F : TStream;

begin
  Result:='';
  if Assigned(DescrNode) then
    begin
    S:=TStringStream.Create('');
    Try
      F:=PushWriteContext(S);
      Try
        ConvertDescr(AContext, DescrNode, False);
        Result:=S.DataString;
      FInally
        PopWriteContext(F);
      end;
    finally
      S.FRee;
    end;
    end;
end;

{ ---------------------------------------------------------------------
  Formatting routines
  ---------------------------------------------------------------------}

procedure TManWriter.WriteTP;

begin
  NewLine;
  Writeln('.TP');
end;

procedure TManWriter.WriteB(Const Msg : String);

begin
  NewLine;
  Writeln('.B '+MSG);
end;

procedure TManWriter.WriteBI(Const Msg : String);

begin
  NewLine;
  Writeln('.BI '+MSG);
end;

{ ---------------------------------------------------------------------
  Sectioning routines
  ---------------------------------------------------------------------}


procedure TManWriter.StartListing(Frames: Boolean);
begin
  Writeln('');
  WriteB(SManDocListing);
end;

procedure TManWriter.StartManPage(AElement: TPasElement; ADocNode: TDocNode);
begin
  StartManPage(ElementToManPage(AElement))
end;

procedure TManWriter.StartManPage(FN: String);

begin
  FN:=LowerCase(FN+'.'+mansection);
  FStream:=TFileStream.Create(OutputDir+FN,fmCreate);
end;

procedure TManWriter.EndManPage;
begin
  FreeAndNil(FStream);
end;


procedure TManWriter.StartSection(Const SectionName: String);
begin
  NewLine;
  Writeln('');
  Writeln('.SH '+SectionName);
end;

procedure TManWriter.StartSubSection(Const SubSectionName: String);
begin
  NewLine;
  Writeln('.SS '+SubSectionName);
end;


procedure TManWriter.PageTitle(Const ATitle,ASection,ASource,Amanual : String);

Var
  D : String;

begin
  D:=FormatDateTime('mmmm yyyy',Date);
  WritelnF('.TH "%s" "%s" "%s" "%s" "%s"',[ATitle,ASection,D,ASource,AManual]);
end;


procedure TManWriter.WriteManRef(Const ManPage : String; Comma : Boolean);

begin
  If Comma then
    Writeln(Lowercase(Format('%s (%s),',[ManPage,ManSection])))
  else
    Writeln(Lowercase(Format('%s (%s)',[ManPage,ManSection])));
end;



Function TManWriter.ElementToManPage(APasElement : TPasElement) : String;

Var
  E : TPasElement;

begin
  E:=APasElement; // Make code more readable
  If (E is TPasPackage) or (E is TPasModule) then
    begin
    Result:=APasElement.Name;
    If (Length(Result)>0) and (Result[1]='#') then
      Delete(Result,1,1);
    end
  else if E is TTopicElement then
    begin
    // Todo : Check for package
    Result:=ModuleName+E.name;
    end
  else
    begin
    If Not SkipUnitprefix then
      Result:=ModuleName+'.';
    If (E.Parent<>Nil) and (not (E.Parent is TPasSection)) then
      begin
      If (E.Parent.Name<>'') then
        Result:=Result+E.Parent.Name;
      If (E is TPasProperty) or (E is TPasProcedure) or (E is TPasClassType) then
        Result:=Result+E.Name;
      end
    else
      begin
      If (E is TPasConst) then
        Result:=Result+SManConsts
      else If E is TPasVariable then
        Result:=Result+SManVars
      else If E is TPasClassType then
        Result:=Result+E.Name
      else If E is TPasType then
        Result:=Result+SManTypes
      else If E is TPasResString then
        Result:=Result+SManResStr
      else
        Result:=Result+E.Name;
      end;
    end;
  Result:=LowerCase(Result);
end;

procedure TManWriter.WriteManRef(APasElement : TPasElement; Comma : Boolean);

begin
  WriteManRef(ElementToManPage(APasElement),Comma);
end;

procedure TManWriter.WriteModuleSeealso(Comma : Boolean);

Var
  HC,HT,HV,HR : Boolean;

begin
  HC:=Module.InterfaceSection.Consts.Count>0;
  HR:=Module.InterfaceSection.ResStrings.Count>0;
  HV:=Module.InterfaceSection.Variables.Count>0;
  HT:=Module.InterfaceSection.Types.Count>0;
  WriteManRef(ModuleName,HC or HR or HV or HT or comma);
  if HC then
    WriteManRef(ModuleName+'.'+SManConsts,HR or HV or HT or comma);
  if HR then
    WriteManRef(ModuleName+'.'+SManResStr, HV or HT or comma);
  if HV then
    WriteManRef(ModuleName+'.'+SManVars, HT or comma);
  if HT then
    WriteManRef(ModuleName+'.'+SManTypes,comma);
end;

procedure TManWriter.WriteSeeAlso(ADocNode: TDocNode; Comma: Boolean);

var
  Node: TDOMNode;
  s: String;

begin
 if Not (Assigned(ADocNode) and Assigned(ADocNode.SeeAlso)) then
    Exit;
  Node := ADocNode.SeeAlso.FirstChild;
  while Assigned(Node) do
    begin
    if IsLinkNode(Node) then
      begin
      S:=UTF8Encode(TDomElement(Node)['id']);
      WriteManRef(S,(Node.NextSibling<>Nil) or Comma);
      end;
    Node:=Node.NextSibling;
    end;
end;

function TManWriter.ConstValue(ConstDecl: TPasConst): String;
begin
  if Assigned(ConstDecl) then
    Result := ConstDecl.ClassName
  else
    Result := '<nil>';
end;


procedure TManWriter.WriteExample(ADocNode: TDocNode);

var
  Example: TDOMElement;
  S : string;

begin
  S:='';
  if Assigned(ADocNode) then
    begin
    Example := ADocNode.FirstExample;
    If Assigned(Example) then
      begin
      StartSection(SManDocExamples);
      while Assigned(Example) do
        begin
        s:=Engine.GetExampleFileName(Example);
        if (s<>'') then
          WriteExampleFile(S);
        DescrEndParaGraph;
        Repeat
          Example := TDomElement(Example.NextSibling);
        until (Example=Nil) or ((Example.NodeType=ELEMENT_NODE) and (Example.NodeName='example'));
        end;
      end;
    end;
end;

procedure TManWriter.WriteExampleFile(FN : String);

Var
  L : TStringList;
  I : Integer;

begin
  WriteBI(SDocExample+' \- '+ExtractFileName(FN));
  Writeln('');
  DescrWriteLineBreak;
  If (FN<>'') and FileExists(FN) then
    begin
    L:=TStringList.Create;
    Try
      L.LoadFromFile(FN);
      For I:=0 to L.Count-1 do
        DescrWriteCodeLine(L[i]);
    finally
      L.Free;
    end;
    end;
end;

{ ---------------------------------------------------------------------
  Actual man page writing
  ---------------------------------------------------------------------}

procedure TManWriter.DoWriteDocumentation;

var
  i : Integer;
  L : TstringList;
begin
  PackageName := LowerCase(Copy(Package.Name, 2, 255));
  If (Engine.Output<>'') then
    OutputDir:=Engine.Output
  else
    OutputDir:=PackageName+'.man';
  If not ForceDirectories(OutputDir) then
    FPDocError(SErrCouldNotCreateOutputDir,[OutputDir]);
  OutputDir:=IncludeTrailingPathDelimiter(OutputDir);
  If (ManSection='') then
    ManSection:=IntToStr(DefaultManSection);
  WritePackagePage;
  L:=TStringList.Create;
  Try
    // Sort modules.
    For I:=0 to Package.Modules.Count-1 do
      L.AddObject(TPasModule(Package.Modules[i]).Name,TPasModule(Package.Modules[i]));
    L.Sorted:=True;
    for i:=0 to L.Count - 1 do
      ProcessModule(TPasModule(L.Objects[i]));
  Finally
    L.Free;
  end;
end;


{ ---------------------------------------------------------------------
  Package man page
  ---------------------------------------------------------------------}


Procedure TManWriter.WritePackagePage;

var
  D,DocNode: TDocNode;
  M : TPasModule;
  I : Integer;
  L : TStringList;

begin
  DocNode:=Engine.FindDocNode(Package);
  If (PackageDescr='') and assigned(DocNode) then
    PackageDescr:=GetDescrString(Package,DocNode.ShortDescr);
  StartManPage(Package,DocNode);
  Try
    PageTitle(PackageName,ManSection,PackageName,PackageDescr);
    StartSection(SManDocName);
    Writeln(PackageName+' \- '+PackageDescr);
    if Assigned(DocNode) and not IsDescrNodeEmpty(DocNode.Descr) then
      begin
      StartSection(SDocDescription);
      WriteDescr(Package, DocNode.Descr);
      end;
    StartSection(SManDocPackageUnits);
    L:=TStringList.Create;
    Try
      For I:=0 to Package.Modules.Count-1 do
        L.AddObject(TPasModule(Package.Modules[i]).Name,TPasModule(Package.Modules[i]));
      L.Sorted:=True;
      for i:=0 to L.Count - 1 do
        begin
        WriteTP;
        WriteB(L[i]);
        M:=TPasModule(L.Objects[i]);
        D:=Engine.FindDocNode(M);
        if Assigned(D) then
          WriteLn(GetDescrString(M,D.ShortDescr))
        else
          WriteLn(GetDescrString(M,Nil))
        end;
      StartSection(SDocSeeAlso);
      WriteSeeAlso(DocNode,True);
      WriteTopicRefs(DocNode);
      for i:=0 to L.Count - 1 do
        WriteManRef(TPasModule(L.Objects[i]),I<L.Count-1);
    Finally
      L.Free;
    end;
  Finally
    EndManPage;
  end;
  ProcessTopics(DocNode,True);
end;

{ ---------------------------------------------------------------------
  Topic support
  ---------------------------------------------------------------------}

Procedure TManWriter.WriteTopicRefs(DocNode : TDocNode);

Var
  Node : TDocNode;

begin
  If Not Assigned(DocNode) then
    Exit;
  Node:=DocNode.FirstChild;
  While Assigned(Node) do
    begin
    If Node.TopicNode then
      WriteManRef(DocNode.Name,Node.NextSibling<>Nil);
    Node:=Node.NextSibling;
    end;
end;

Procedure TManWriter.ProcessTopics(DocNode : TDocNode; Subs : Boolean);

Var
  Node,SubNode : TDocNode;

begin
  If Not Assigned(DocNode) then
    Exit;
  Node:=DocNode.FirstChild;
  While Assigned(Node) do
    begin
    If Node.TopicNode then
      begin
      WriteTopicPage(DocNode,Node);
      If Subs then
        begin
        SubNode:=DocNode.FirstChild;
        While Assigned(SubNode) do
        If SubNode.TopicNode then
           WriteTopicPage(Node,SubNode);
        end;
      end;
    Node:=Node.NextSibling;
    end;
end;


Procedure TManWriter.WriteTopicPage(Parent,Node : TDocNode);

Var
  Element : TTopicElement;

begin
  Element:=FindTopicElement(Node);
  If Not Assigned(Element) then
    Exit;
  StartManPage(Element,Node) ;
  Try
    PageTitle(Node.Name,ManSection,PackageName,PackageDescr);
    StartSection(SManDocName);
    Writeln(Node.Name+' \- '+GetDescrString(Element,Node.ShortDescr));
    StartSection(SManDocDescription);
    If Assigned(Node.Descr) then
      WriteDescr(Element,Node.Descr);
    StartSection(SManDocSeeAlso);
    WriteSeeAlso(Node,True);
    WriteTopicRefs(Parent);
    WriteTopicRefs(Node);
  Finally
    EndManPage;
  end;
end;


{ ---------------------------------------------------------------------
  Module man pages
  ---------------------------------------------------------------------}

procedure TManWriter.ProcessModule(AModule : TPasModule);

begin
  With AModule do
    begin
    Module:=AModule;
    ModuleName:=Name;
    With InterfaceSection do
      begin
      SortElementList(Declarations);
      SortElementList(Types);
      SortElementList(Consts);
      SortElementList(Classes);
      SortElementList(Functions);
      SortElementList(Variables);
      SortElementList(ResStrings);
      end;
    WriteUnitPage(AModule);
    WriteUnitResourceStrings(InterfaceSection);
    WriteUnitConsts(InterfaceSection);
    WriteUnitTypes(InterfaceSection);
    WriteUnitVars(InterfaceSection);
    WriteUnitClasses(InterfaceSection);
    WriteUnitFunctionsAndProcedures(InterfaceSection);
    end;
end;

procedure TManWriter.WriteUnitPage(AModule : TPasModule);

Var
  DocNode : TDocNode;
  S : String;
begin
  DocNode:=Engine.FindDocNode(AModule);
  StartManPage(AModule,DocNode);
  Try
    PageTitle(AModule.Name,ManSection,PackageName,PackageDescr);
    StartSection(SManDocName);
    if Assigned(DocNode) then
      S:=GetDescrString(AModule,DocNode.ShortDescr);

    Writeln(AModule.Name+' \- '+S);
    if Assigned(DocNode) and not IsDescrNodeEmpty(DocNode.Descr) then
      begin
      StartSection(SManDocDescription);
      WriteDescr(AModule.Parent, DocNode.Descr);
      end;
    WriteUnitUsesOverview(AModule.InterfaceSection);
    WriteUnitClassesOverView(AModule.InterfaceSection);
    WriteUnitFunctionsAndProceduresOverView(AModule.InterfaceSection);
    StartSection(SManDocSeealso);
    WriteModuleSeeAlso(False);
  finally
    EndManPage;
  end;
end;

procedure TManWriter.WriteUnitUsesOverview(ASection: TPasSection);

var
  i: Integer;
  UnitRef: TPasType;
  DocNode: TDocNode;
begin
  if ASection.UsesList.Count > 0 then
    begin
    StartSection(SManDocUsedUnits);
    for i := 0 to ASection.UsesList.Count - 1 do
      begin
      UnitRef := TPasType(ASection.UsesList[i]);
      WriteTP;
      WriteB(UnitRef.Name);
      DocNode := Engine.FindDocNode(UnitRef);
      If Assigned(DocNode) then
        WriteDescr(UnitRef,DocNode.ShortDescr)
      end;
    end;
end;


{ ---------------------------------------------------------------------
  Classes man pages
  ---------------------------------------------------------------------}

procedure TManWriter.WriteUnitClassesOverview(ASection: TPasSection);

var
  i         : Integer;
  DocNode   : TDocNode;
  ClassDecl : TPasClassType;

begin
  // Overview page
  if ASection.Classes.Count > 0 then
    begin
    StartSection(SManDocClasses);
    for i := 0 to ASection.Classes.Count - 1 do
      begin
      ClassDecl:=TPasClassType(ASection.Classes[i]);
      WriteTP;
      WriteB(ClassDecl.Name);
      DocNode:=Engine.FindDocNode(ClassDecl);
      If Assigned(DocNode) then
        WriteDescr(ClassDecl,DocNode.ShortDescr);
      end;
    end;

end;


procedure TManWriter.WriteUnitClasses(ASection: TPasSection);

var
  i: Integer;

begin
  if (ASection.Classes.Count > 0) then
    begin
    for i := 0 to ASection.Classes.Count - 1 do
      WriteClassPage(TPasClassType(ASection.Classes[i]));
    end;
end;

procedure TManWriter.WriteClassPage(ClassDecl: TPasClassType);

var
  DocNode: TDocNode;

begin
  DocNode:=Engine.FindDocNode(ClassDecl);
  StartManPage(ClassDecl,DocNode);
  Try
    PageTitle(ClassDecl.Name,ManSection,PackageName,PackageDescr);
    StartSection(SManDocName);
    Write(ClassDecl.Name);
    DocNode := Engine.FindDocNode(ClassDecl);
    If Assigned(DocNode) then
      begin
      if not IsDescrNodeEmpty(DocNode.ShortDescr) then
        begin
        write(' \- ');
        WriteDescr(ClassDecl,DocNode.ShortDescr);
        end;
      if Not (IsDescrNodeEmpty(DocNode.Descr)
              and IsDescrNodeEmpty(DocNode.ShortDescr)) then
        begin
        StartSection(SDocDescription);
        WriteDescr(ClassDecl);
        end;
      end;
    // Write method overview
    WriteClassMethodOverView(ClassDecl);
    // Write Property Overview;
    WriteClassPropertyOverView(ClassDecl);
  Finally
    EndManPage;
  end
end;


procedure TManWriter.WriteClassPropertyOverview(ClassDecl : TPasClassType);

var
  Member: TPasElement;
  i: Integer;
  A: String;
  DocNode: TDocNode;
  List : TStringList;

begin
  // Write property overview
  List:=TStringList.Create;
  Try
    for i := 0 to ClassDecl.Members.Count - 1 do
      begin
      Member := TPasElement(ClassDecl.Members[i]);
      With Member do
        if InheritsFrom(TPasProperty) and SHowMember(Member) then
          List.AddObject(Member.Name,Member)
      end;
    List.Sorted:=True;
    If (List.Count>0) then
      begin
      StartSection(SDocPropertyOverview);
      For I:=0 to List.Count-1 do
        begin
        Member:=TPasElement(List.objects[i]);
        WriteTP;
        A:='';
        if Length(TPasProperty(Member).ReadAccessorName) > 0 then
          a := a + 'r';
        if Length(TPasProperty(Member).WriteAccessorName) > 0 then
          a := a + 'w';
        if Length(TPasProperty(Member).StoredAccessorName) > 0 then
          a := a + 's';
        WriteBI(Member.Name+'  '+A);
        DocNode := Engine.FindDocNode(Member);
        If Assigned(DocNode) then
          WriteDescr(Member, DocNode.ShortDescr)
        end;
      end;
  Finally
    List.Free;
  end;
end;

{ ---------------------------------------------------------------------
  Resource strings man page
  ---------------------------------------------------------------------}

procedure TManWriter.WriteUnitResourceStrings(ASection: TPasSection);

var
  ResStrDecl: TPasResString;
  i: Integer;

begin
  if ASection.ResStrings.Count > 0 then
    begin
    StartManpage(ModuleName+'.'+SManResStr);
    Try
      PageTitle(Modulename,ManSection,PackageName,PackageDescr);
      StartSection(SManDocName);
      Writeln(ModuleName+' \- '+SDocResStrings);
      StartSection(SManDocResourceStrings);
      Writeln('');
      for i := 0 to ASection.ResStrings.Count - 1 do
        begin
        ResStrDecl := TPasResString(ASection.ResStrings[i]);
        StartSubSection(ResStrDecl.Name);
        DescrWriteCodeLine(EscapeText(ResStrDecl.GetDeclaration(False)));
        end;
      StartSection(SDocSeeAlso);
      WriteModuleSeealso(False);
    Finally
      EndManPage;
    end;
    end;
end;

{ ---------------------------------------------------------------------
  Constants man page
  ---------------------------------------------------------------------}

procedure TManWriter.WriteUnitConsts(ASection: TPasSection);
var
  i: Integer;
  ConstDecl: TPasConst;
  DocNode: TDocNode;
begin
  if ASection.Consts.Count > 0 then
    begin
    StartManpage(ModuleName+'.'+SManConsts);
    Try
      PageTitle(Modulename,ManSection,PackageName,PackageDescr);
      StartSection(SManDocName);
      Writeln(ModuleName+' \- '+SDocConstants);
      StartSection(SManDocConstants);
      for i := 0 to ASection.Consts.Count - 1 do
        begin
        ConstDecl := TPasConst(ASection.Consts[i]);
        StartSubSection(ConstDecl.Name);
        DescrWriteCodeLine(EscapeText(ConstDecl.GetDeclaration(True)));
        DocNode:=Engine.FindDocNode(ConstDecl);
        WriteDescr(ConstDecl,DocNode);
        end;
      StartSection(SDocSeeAlso);
      WriteModuleSeealso(False);
    Finally
      EndManPage;
    end;
    end;
end;

{ ---------------------------------------------------------------------
  Types man page
  ---------------------------------------------------------------------}

procedure TManWriter.WriteUnitTypes(ASection: TPasSection);
var
  i: Integer;
  TypeDecl: TPasType;
  DocNode : TDocNode;

begin
  if ASection.Types.Count > 0 then
    begin
    StartManpage(ModuleName+'.'+SManTypes);
    Try
      PageTitle(Modulename,ManSection,PackageName,PackageDescr);
      StartSection(SManDocName);
      Writeln(ModuleName+' \- '+SDocTypes);
      StartSection(SManDocTypes);
      for i := 0 to ASection.Types.Count - 1 do
        begin
        TypeDecl := TPasType(ASection.Types[i]);
        StartSubsection(TypeDecl.Name);
        DescrWriteCodeLine(EscapeText(TypeDecl.GetDeclaration(True)));
        DocNode:=Engine.FindDocNode(TypeDecl);
        If TypeDecl is TPasEnumType then
          WriteEnumElements(TypeDecl as TPasEnumType);
        WriteDescr(TypeDecl,DocNode);
        end;
      StartSection(SDocSeeAlso);
      WriteModuleSeeAlso(False);
    Finally
      EndManPage;
    end;
    end;
end;

procedure TManWriter.WriteEnumElements(TypeDecl : TPasEnumType);

Var
  EV : TPasEnumValue;
  I : Integer;
  DocNode : TDocNode;

begin
  With TypeDecl do
    begin
    SortElementList(Values);
    Writeln(EscapeText(Format(SDocValuesForEnum,[TypeDecl.Name])));
    Try
    For I:=0 to Values.Count-1 do
      begin
      EV:=TPasEnumValue(Values[i]);
      WriteTP;
      WriteB(EscapeText(EV.Name));
      DocNode := Engine.FindDocNode(EV);
      if Assigned(DocNode) and (not IsDescrNodeEmpty(DocNode.ShortDescr)) then
        WriteDescr(EV,DocNode.ShortDescr);
      end;
    Finally
      NewLine;
    end;
    end;
end;

{ ---------------------------------------------------------------------
  Variables man page
  ---------------------------------------------------------------------}

procedure TManWriter.WriteUnitVars(ASection: TPasSection);
var
  VarDecl: TPasVariable;
  i: Integer;
  DocNode : TDocNode;

begin
  if ASection.Variables.Count > 0 then
    begin
    StartManpage(ModuleName+'.'+SManVars);
    Try
      PageTitle(Modulename,ManSection,PackageName,PackageDescr);
      StartSection(SManDocName);
      Writeln(ModuleName+' \- '+SDocVariables);
      StartSection(SManDocVariables);
      for i := 0 to ASection.Variables.Count - 1 do
        begin
        VarDecl := TPasVariable(ASection.Variables[i]);
        StartSubSection(VarDecl.Name);
        DescrWriteCodeLine(EscapeText(VarDecl.GetDeclaration(True)));
        DocNode:=Engine.FindDocNode(VarDecl);
        WriteDescr(VarDecl,DocNode);
        end;
      StartSection(SDocSeeAlso);
      WriteModuleSeeAlso(False);
    Finally
      EndManPage;
    end;
    end;
end;


{ ---------------------------------------------------------------------
  Procedure/Function/Method man page
  ---------------------------------------------------------------------}

procedure TManWriter.WriteUnitFunctionsAndProceduresOverview(ASection: TPasSection);

var
  i       : Integer;
  DocNode : TDocNode;
  PDecl      : TPasProcedureBase;

begin
  // Overview page
  if ASection.Functions.Count > 0 then
    begin
    StartSection(SManDocFunctions);
    for i := 0 to ASection.Functions.Count - 1 do
      begin
      PDecl:=TPasProcedureBase(ASection.Functions[i]);
      WriteTP;
      WriteB(PDecl.Name);
      DocNode:=Engine.FindDocNode(PDecl);
      If Assigned(DocNode) then
      WriteDescr(PDecl,DocNode.ShortDescr);
      end;
    end;
end;

procedure TManWriter.WriteUnitFunctionsAndProcedures(ASection: TPasSection);

var
  i       : Integer;

begin
  // Pages for all identifiers.
  for i := 0 to ASection.Functions.Count - 1 do
    WriteProcedurePage(TPasProcedure(ASection.Functions[i]));
end;

procedure TManWriter.WriteProcedurePage(ProcDecl : TPasProcedureBase);

var
  DocNode: TDocNode;
  OP : TPasOverloadedProc;
  i : integer;
  D,N : String;
begin
  N:=ProcDecl.name;
  D:='';
  DocNode := Engine.FindDocNode(ProcDecl);
  StartManpage(ProcDecl,DocNode);
  Try
    PageTitle(ProcDecl.name,ManSection,PackageName,PackageDescr);
    if Assigned(DocNode) then
      D:=GetDescrString(ProcDecl,DocNode.ShortDescr);
    // Name
    StartSection(SManDocName);
    Writeln(N+' \- '+D);
    // Declaration
    StartSection(SManDocSynopsis);
    if ProcDecl is TPasOverloadedProc then
      begin
      OP:=TPasOverloadedProc(ProcDecl);
      for i := 0 to OP.Overloads.Count - 1 do
        begin
        DescrWriteCodeLine(TPasProcedure(OP.Overloads[i]).GetDeclaration(True));
        end;
      end
    else
      DescrWriteCodeLine(ProcDecl.GetDeclaration(True));
    // Visibility
    If Assigned(ProcDecl.Parent) then
      begin
      StartSection(SManDocVisibility);
      Writeln(VisibilityNames[ProcDecl.Visibility])
      end;
    // Arguments, if present.
    If ProcDecl is TPasProcedure then
      AppendProcedureArgsSection(TPasProcedure(ProcDecl).ProcType);
    // Description
    if Assigned(DocNode) then
      begin
      if Assigned(DocNode.Descr) then
        begin
        StartSection(SManDocDescription);
        WriteDescr(ProcDecl,DocNode.Descr);
        end;
      // Errors
      if Assigned(DocNode) and Assigned(DocNode.ErrorsDoc) then
        begin
        StartSection(SManDocErrors);
        WriteDescr(ProcDecl, DocNode.ErrorsDoc);
        end;
      end;
    // Arguments, if present.
    If ProcDecl is TPasFunction then
      AppendProcedureArgsSection(TPasFunction(ProcDecl).ProcType);
    WriteExample(DocNode);
    StartSection(SManDocSeeAlso);
    WriteModuleSeeAlso(True);
    WriteSeeAlso(DocNode,False);
  Finally
    EndManPage;
  end;
end;

procedure TManWriter.AppendProcedureArgsSection(Element: TPasProcedureType);

var
  IsFirst: Boolean;
  DocNode: TDocNode;
  i: Integer;
  Arg: TPasArgument;

begin
  If Not Assigned(Element) then
    exit;
  IsFirst := True;
  for i := 0 to Element.Args.Count - 1 do
    begin
    Arg := TPasArgument(Element.Args[i]);
    DocNode:=Engine.FindDocNode(Arg);
    if Assigned(DocNode) and (Not IsDescrNodeEmpty(DocNode.ShortDescr)) then
      begin
      if IsFirst then
        begin
        IsFirst:=False;
        StartSection(SManDocArguments);
        end;
      WriteTP;
      WriteB(Arg.Name);
      WriteDescr(Arg,DocNode.ShortDescr);
      end;
    end;

end;

procedure TManWriter.AppendFunctionResultSection(Element: TPasFunctionType);

Var
  ResultEl: TPasResultElement;
  DocNode: TDocNode;

begin
  If Not Assigned(Element) then
    exit;
  ResultEl := TPasFunctionType(Element).ResultEl;
  DocNode := Engine.FindDocNode(ResultEl);
  If Assigned(DocNode) then
    begin
    if IsDescrNodeEmpty(DocNode.Descr) or not IsDescrNodeEmpty(DocNode.ShortDescr) then
    begin
    StartSection(SManDocResult);
    WriteDescr(ResultEl,DocNode);
    end;
  end;
end;



{ ---------------------------------------------------------------------
  Property man page
  ---------------------------------------------------------------------}

procedure TManWriter.WritePropertyPage(PropDecl : TPasProperty);

var
  DocNode: TDocNode;
  N,D: String;

begin
  DocNode := Engine.FindDocNode(PropDecl);
  StartManpage(PropDecl,DocNode);
  Try
    N:= PropDecl.Name;
    PageTitle(PropDecl.Name,ManSection,PackageName,PackageDescr);
    if Assigned(DocNode) then
    D:=GetDescrString(PropDecl,DocNode.ShortDescr);
    // Name
    StartSection(SManDocName);
    Writeln(N+' \- '+D);
    // Declaration
    StartSection(SManDocSynopsis);
    WriteLn(PropDecl.GetDeclaration(True));
    // Visibility
    If Assigned(PropDecl.Parent) then
      begin
      StartSection(SManDocVisibility);
      Writeln(VisibilityNames[PropDecl.Visibility])
      end;
    StartSection(SManDocAccess);
    D:='';
    If Length(PropDecl.ReadAccessorName) > 0 then
      D:='Read';
    if Length(PropDecl.WriteAccessorName) > 0 then
      begin
      If D<>'' then
        D:=D+',';
      D:=D+'Write';
      end;
    Writeln(D);
    if Assigned(DocNode) then
      begin
      // Description
      if Assigned(DocNode.Descr) then
        begin
        StartSection(SManDocDescription);
        WriteDescr(PropDecl,DocNode.Descr);
        end;
      // Errors
      if Assigned(DocNode) and Assigned(DocNode.ErrorsDoc) then
        begin
        StartSection(SManDocErrors);
        WriteDescr(PropDecl, DocNode.ErrorsDoc);
        end;
    WriteExample(DocNode);
    WriteSeeAlso(DocNode,False);
    end;
  Finally
    EndManPage;
  end;
end;


Function CompareElements(P1,P2 : Pointer) : Integer;

begin
  Result:=CompareText(TPasElement(P1).Name,TPasElement(P2).Name);
end;

procedure TManWriter.SortElementList(List : TFPList);

begin
  List.Sort(@CompareElements);
end;

procedure TManWriter.WriteCommentLine;

begin
  WriteComment('-------------------------------------------------------');
end;

procedure TManWriter.WriteComment(Comment : String);

begin
  Writeln('." '+Comment);
end;


class function TManWriter.FileNameExtension: String;
begin
  Result:=IntToStr(DefaultManSection);
end;


procedure TManWriter.WriteClassMethodOverview(ClassDecl: TPasClassType);

var
  Member : TPasElement;
  i : Integer;
  DocNode : TDocNode;
  List : TStringList;

begin
  List:=TStringList.Create;
  Try
    GetMethodList(ClassDecl,List);
    If List.Count>0 then
      begin
      StartSection(SManDocMethods);
      For I:=0 to List.Count-1 do
        begin
        Member:=TPasElement(List.Objects[i]);
        WriteTP;
        WriteB(EscapeText(Member.Name));
        DocNode := Engine.FindDocNode(Member);
        If Assigned(DocNode) then
          WriteDescr(Member, DocNode.ShortDescr)
        end;
      end;
  Finally
    List.Free;
  end;
end;


Class procedure TManWriter.Usage(List: TStrings);
begin
  List.add('--nounitprefix');
  List.Add(SManUsageNoUnitPrefix);
  List.add('--man-section=ASection');
  List.Add(SManUsageManSection);
  List.Add('--man-description=descr');
  List.Add(SManUsagePackageDescription);
end;

initialization
  // Do not localize.
  RegisterWriter(TMANWriter,'man',SManUsageWriterDescr);
finalization
  UnRegisterWriter('man');
end.
