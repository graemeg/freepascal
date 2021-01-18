{$mode objfpc}
{$H+}
unit dwlinear;
{$WARN 5024 off : Parameter "$1" not used}
interface

uses
  Classes, SysUtils, DGlobals, dWriter, pastree, dom;

Type
  { TLinearWriter }

  TLinearWriter = Class(TFPDocWriter)
    FStream : TStream;
    PackageName: String;
    Module: TPasModule;
    FLastURL : DomString;
  private
    FDupLinkedDoc: Boolean;
  Protected
    ModuleName: String;
    // Writing support.
    procedure Write(const s: String); virtual;
    procedure WriteLn(const s: String); virtual;
    procedure WriteF(const s: String; const Args: array of const); virtual;
    procedure WriteLnF(const s: String; const Args: array of const); virtual;
    Function  PushWriteContext(S : TStream) : TStream;
    Procedure PopWriteContext(S : TSTream);
    procedure WriteLabel(El: TPasElement);
    procedure WriteIndex(El: TPasElement);
    procedure WriteTypeDecl(El: TPasElement); virtual;
    procedure WriteVariableDecl(El: TPasElement); virtual;
    procedure WriteConstDecl(El: TPasElement); virtual;
    // Auxiliary routines
    procedure DescrBeginURL(const AURL: DOMString); override; // Provides a default implementation
    procedure DescrEndURL; override;
    procedure SortElementList(List : TFPList);
    procedure StartListing(Frames: Boolean);
    Function  ShowMember(M : TPasElement) : boolean;
    procedure StartChapter(ChapterName : String; ChapterLabel : String); virtual;
    procedure StartSection(SectionName : String; SectionLabel : String); virtual;
    procedure StartSubSection(SubSectionName : String; SubSectionLabel : String); virtual;
    procedure StartSubSubSection(SubSubSectionName : String; SubSubSectionLabel : String); virtual;
    Function  GetDescrString(AContext: TPasElement; DescrNode: TDOMElement) : String;
    function  ConstValue(ConstDecl: TPasConst): String; virtual;
    procedure ProcessSection(ASection: TPasSection); virtual;
    // Procedures which MAY be overridden in descendents
    procedure WriteBeginDocument; virtual;
    procedure WriteEndDocument; virtual;
    Function  EscapeText(S : UnicodeString) : String; overload;
    Function  EscapeText(S : String) : String; virtual; overload;
    Function  StripText(S : String) : String; virtual;
    Procedure StartProcedure; Virtual;
    Procedure EndProcedure; Virtual;
    Procedure StartProperty; Virtual;
    Procedure EndProperty; Virtual;
    Procedure StartSynopsis; Virtual;
    Procedure StartDeclaration; Virtual;
    Procedure StartVisibility; Virtual;
    Procedure StartDescription; Virtual;
    Procedure StartAccess; Virtual;
    Procedure StartErrors; Virtual;
    Procedure StartVersion; Virtual;
    Procedure StartSeealso; Virtual;
    Procedure EndSeealso; Virtual;
    // Procedures which MUST be overridden in descendents;
    procedure WriteCommentLine; virtual; abstract;
    procedure WriteComment(Comment : String); virtual; abstract;
    function  GetLabel(AElement: TPasElement): String; virtual; abstract;
    procedure WriteLabel(Const S : String); virtual; abstract;
    procedure WriteIndex(Const S : String); virtual; abstract;
    procedure WriteType(const s: string); virtual;
    procedure WriteVariable(const s: string); virtual;
    procedure WriteConstant(const s: string); virtual;
    procedure StartChapter(ChapterName : String); virtual; abstract;
    procedure StartSection(SectionName : String); virtual; abstract;
    procedure StartSubSection(SubSectionName : String); virtual; abstract;
    procedure StartSubSubSection(SubSubSectionName : String); virtual; abstract;
    procedure StartListing(Frames: Boolean; const name: String); virtual; abstract;
    procedure EndListing; virtual; abstract;
    Procedure WriteExampleFile(FN : String); virtual; abstract;
    procedure StartOverview(Const What : String; WithAccess : Boolean); virtual; Abstract;
    procedure EndOverview; virtual; Abstract;
    procedure WriteOverviewMember(const ALabel,AName,Access,ADescr : String); virtual; Abstract;
    procedure WriteOverviewMember(const ALabel,AName,ADescr : String); virtual; Abstract;
    procedure StartUnitOverview(AModuleName,AModuleLabel : String);virtual; Abstract;
    procedure WriteUnitEntry(UnitRef : TPasType);virtual; Abstract;
    procedure EndUnitOverview; virtual; Abstract;
    Property LastURL : DomString Read FLastURL Write FLastURL;
    // Overriden from fpdocwriter;
    procedure DescrWriteText(const AText: DOMString); override;
    // Actual writing happens here.
    Procedure DoWriteDocumentation; override;
  Public
    Constructor Create(APackage: TPasPackage; AEngine: TFPDocEngine); override;
    function InterpretOption(const Cmd, Arg: String): Boolean; override;
    class procedure Usage(List: TStrings); override;
    // Linear Documentation writing methods.
    Procedure ProcessPackage;
    Procedure ProcessTopics(DocNode : TDocNode; Alevel : Integer);
    procedure WriteResourceStrings(ASection: TPasSection);
    procedure WriteUnitOverview(ASection: TPasSection);
    procedure WriteVarsConstsTypes(ASection: TPasSection);
    procedure WriteConsts(ASection: TPasSection);
    procedure WriteTypes(ASection: TPasSection); virtual;
    procedure WriteEnumElements(TypeDecl : TPasEnumType);
    procedure WriteVars(ASection: TPasSection);
    procedure WriteFunctionsAndProcedures(ASection: TPasSection);
    procedure WriteProcedure(ProcDecl: TPasProcedureBase);
    procedure WriteClasses(ASection: TPasSection);
    Procedure WriteExtendedRecords(Asection : TPasSection);
    procedure WriteClassDecl(ClassDecl: TPasClassType);
    procedure WriteMethodOverview(AParent: TPasType; Members : TFPList);
    procedure WritePropertyOverview(AParent: TPasType; Members : TFPList);
    procedure WriteClassInterfacesOverview(ClassDecl: TPasClassType);
    procedure WriteClassInheritanceOverview(ClassDecl: TPasClassType); virtual;
    procedure WriteProperty(PropDecl: TPasProperty);
    procedure WriteExample(ADocNode: TDocNode);
    procedure WriteSeeAlso(ADocNode: TDocNode);
    Procedure WriteTopicNode(Node : TDocNode; Level : Integer);
  end;

implementation

uses fpdocstrs;

const
  cDupLinkedDocParam = '--duplinkeddoc';

{ TLinearWriter }

{ ---------------------------------------------------------------------
  Writing support
  ---------------------------------------------------------------------}

function TLinearWriter.PushWriteContext(S: TStream): TStream;

begin
  Result:=FStream;
  FStream:=S;
end;

procedure TLinearWriter.PopWriteContext(S: TSTream);

begin
  FStream:=S;
end;

procedure TLinearWriter.Write(const s: String);

Var
  L : Integer;

begin
  L:=Length(S);
  If (L>0) then
    FStream.Write(PChar(S)^,L);
end;

procedure TLinearWriter.WriteF(const s: String; const Args: array of const);
begin
  Write(Format(S,Args));
end;

procedure TLinearWriter.WriteLn(const s: String);
begin
  Write(S);
  Write(LineEnding);
end;

procedure TLinearWriter.WriteLnF(const s: String; const Args: array of const);
begin
  Write(Format(S,Args));
  Write(LineEnding);
end;

procedure TLinearWriter.DescrWriteText(const AText: DOMString);
begin
  Write(EscapeText(AText));
end;

function TLinearWriter.GetDescrString(AContext: TPasElement;
  DescrNode: TDOMElement): String;

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
  Auxiliary routines
  ---------------------------------------------------------------------}

procedure TLinearWriter.WriteLabel(El: TPasElement);
begin
  WriteLabel(GetLabel(El));
end;

procedure TLinearWriter.WriteIndex(El: TPasElement);
begin
  WriteIndex(EL.Name);
end;

procedure TLinearWriter.WriteTypeDecl(El: TPasElement);
begin
  WriteType(El.Name);
end;

procedure TLinearWriter.WriteVariableDecl(El: TPasElement);
begin
	WriteVariable(El.Name);
end;

procedure TLinearWriter.WriteConstDecl(El: TPasElement);
begin
	WriteConstant(El.Name);
end;

procedure TLinearWriter.DescrBeginURL(const AURL: DOMString);
begin
  FLastURL:=AURL;
end;

procedure TLinearWriter.DescrEndURL;
begin
  If (FLastURL<>'') then
    Writeln(Format(SSeeURL,[EscapeText(FLastURL)]));
  FLastURL:='';
end;

procedure TLinearWriter.StartListing(Frames: Boolean);
begin
  StartListing(Frames,'');
end;

procedure TLinearWriter.StartChapter(ChapterName: String; ChapterLabel: String);
begin
  StartChapter(ChapterName);
  WriteLabel(ChapterLabel);
end;

procedure TLinearWriter.StartSection(SectionName: String; SectionLabel: String);
begin
  StartSection(SectionName);
  WriteLabel(SectionLabel);
end;

procedure TLinearWriter.StartSubSection(SubSectionName: String; SubSectionLabel: String);
begin
  StartSubSection(SubSectionName);
  WriteLabel(SubSectionLabel);
end;

procedure TLinearWriter.StartSubSubSection(SubSubSectionName: String;
  SubSubSectionLabel: String);
begin
  StartSubSubSection(SubSubSectionName);
  WriteLabel(SubSubSectionLabel);
end;

{ ---------------------------------------------------------------------
  Default implementations, may be overridden in descendents
  ---------------------------------------------------------------------}

function TLinearWriter.EscapeText(S: String): String;

begin
  Result:=S;
end;

function TLinearWriter.StripText(S: String): String;

begin
  Result:=S;
end;

procedure TLinearWriter.StartProcedure;

begin
  Writeln(SDocProcedure+':');
end;

procedure TLinearWriter.StartSynopsis;

begin
  Writeln('');
  Writeln(SDocSynopsis+':');
end;

procedure TLinearWriter.StartDeclaration;

begin
  Writeln('');
  Writeln(SDocDeclaration+':');
end;

procedure TLinearWriter.StartVisibility;

begin
  Writeln('');
  Writeln(SDocVisibility+':');
end;

procedure TLinearWriter.StartDescription;

begin
  Writeln('');
  Writeln(SDocDescription+':');
end;

procedure TLinearWriter.StartAccess;

begin
  Writeln('');
  Writeln(SDocAccess+':');
end;

procedure TLinearWriter.StartErrors;

begin
  Writeln('');
  Writeln(SDocErrors+':');
end;

procedure TLinearWriter.StartVersion;

begin
  Writeln('');
  Writeln(SDocVersion+':');
end;

procedure TLinearWriter.StartSeealso;

begin
  Writeln('');
  Writeln(SDocSeeAlso+':');
end;


procedure TLinearWriter.StartProperty;

begin
  Writeln('');
  Writeln(SDocProperty+':');
end;

procedure TLinearWriter.EndProcedure;

begin
  Writeln('');
end;

procedure TLinearWriter.EndProperty;

begin
  Writeln('');
end;

procedure TLinearWriter.EndSeealso;
begin
  Writeln('');
end;

procedure TLinearWriter.WriteType(const s: string);
begin
  // do nothing
end;

procedure TLinearWriter.WriteVariable(const s: string);
begin
  // do nothing
end;

procedure TLinearWriter.WriteConstant(const s: string);
begin
  // do nothing
end;

procedure TLinearWriter.WriteClassDecl(ClassDecl: TPasClassType);
var
  DocNode: TDocNode;
  Member: TPasElement;
  i: Integer;
begin
  DocNode := Engine.FindDocNode(ClassDecl);
  if Assigned(DocNode) and DocNode.IsSkipped then
    Exit;
  StartSection(ClassDecl.Name);
  WriteLabel(ClassDecl);
  WriteIndex(ClassDecl);
  if Assigned(DocNode) and ((not IsDescrNodeEmpty(DocNode.Descr)) or
    (not IsDescrNodeEmpty(DocNode.ShortDescr))) then
  begin
    StartSubSection(SDocDescription);
    WriteDescr(ClassDecl,DocNode);
    If Assigned(DocNode.Version) then
      begin
      StartSubSection(SDocVersion);
      WriteDescr(ClassDecl,DocNode.Version);
      end;
    if Assigned(DocNode.SeeAlso) then
    begin
      WriteSeeAlso(DocNode);
    end;
    ConvertNotes(ClassDecl,DocNode.Notes);
  end;

  // graemeg: this must move above SeeAlso, Version and Notes written above.
  // Write Class Hierarchy (Inheritance) Overview;
  WriteClassInheritanceOverView(ClassDecl);

  // Write Interfaces Overview;
  WriteClassInterfacesOverView(ClassDecl);
  // Write method overview
  WriteMethodOverView(ClassDecl,ClassDecl.Members);
  // Write Property Overview;
  WritePropertyOverView(ClassDecl,ClassDecl.Members);

  // Write method & property descriptions

  // Determine visibilities

  for i := 0 to ClassDecl.Members.Count - 1 do
    begin
    Member := TPasElement(ClassDecl.Members[i]);
    if Member.InheritsFrom(TPasProcedureBase) and Engine.ShowElement(Member) then
      WriteProcedure(TPasProcedureBase(Member));
    end;

  // properties.

  for i := 0 to ClassDecl.Members.Count - 1 do
    begin
    Member := TPasElement(ClassDecl.Members[i]);
    if Member.InheritsFrom(TPasProperty) and Engine.ShowElement(Member) then
      WriteProperty(TPasProperty(Member));
    end;
end;


procedure TLinearWriter.WritePropertyOverview(AParent: TPasType; Members : TFPList);

var
  Member: TPasElement;
  i: Integer;
  L,N,S,A: String;
  DocNode: TDocNode;
  List : TStringList;
  lNode: TDocNode;
begin
  // Write property overview
  List:=TStringList.Create;
  Try
    List.Sorted:=True;
    for i := 0 to Members.Count - 1 do
      begin
      Member := TPasElement(Members[i]);
      With Member do
        if InheritsFrom(TPasProperty) and SHowMember(Member) then
          List.AddObject(Member.Name,Member)
      end;
    If (List.Count>0) then
      begin
      StartSubSection(SDocPropertyOverview);
      WriteLabel(GetLabel(AParent) + ':Properties');
      StartOverView(SDocProperties,True);
      For I:=0 to List.Count-1 do
        begin
        Member:=TPasElement(List.objects[i]);
        L:=StripText(GetLabel(Member));
        N:=EscapeText(Member.Name);
        DocNode := Engine.FindDocNode(Member);
        if Assigned(DocNode) and DocNode.IsSkipped then
          Continue;
        if Assigned(DocNode) then
        begin
          if FDupLinkedDoc and (DocNode.Link <> '') then
          begin
            lNode := Engine.FindLinkedNode(DocNode);
            if not Assigned(lNode) then
              lNode := DocNode;
          end
          else
            lNode := DocNode;
          S := GetDescrString(Member, lNode.ShortDescr);
        end
        else
          S := '';

        A:='';
        if Length(TPasProperty(Member).ReadAccessorName) > 0 then
          a := a + 'r';
        if Length(TPasProperty(Member).WriteAccessorName) > 0 then
          a := a + 'w';
        if Length(TPasProperty(Member).StoredAccessorName) > 0 then
          a := a + 's';
        WriteOverviewMember(L,N,A,S);
        S := '';
        end;
      EndOverview;
      end;
  Finally
    List.Free;
  end;
end;


procedure TLinearWriter.WriteClassInterfacesOverview(ClassDecl: TPasClassType);
var
  lInterface: TPasElement;
  i: Integer;
  L,N,S: String;
  DocNode: TDocNode;
  List : TStringList;
  lNode: TDocNode;
begin
  // Write Interfaces overview
  List:=TStringList.Create;
  try
    List.Sorted:=True;
    for i := 0 to ClassDecl.Interfaces.Count-1 do
    begin
      lInterface := TPasElement(ClassDecl.Interfaces[i]);
      List.AddObject(lInterface.Name,lInterface);
    end;
    if (List.Count>0) then
    begin
      StartSubSection(SDocInterfacesOverview);
      WriteLabel(GetLabel(ClassDecl) + ':Interfaces');
      StartOverView(SDocInterface,False);
      for i := 0 to List.Count-1 do
      begin
        lInterface := TPasElement(List.Objects[i]);
        L := StripText(GetLabel(lInterface));
        N := EscapeText(lInterface.Name);
        DocNode := Engine.FindDocNode(lInterface);
        if Assigned(DocNode) and DocNode.IsSkipped then
          Continue;
        if Assigned(DocNode) then
        begin
          if FDupLinkedDoc and (DocNode.Link <> '') then
          begin
            lNode := Engine.FindLinkedNode(DocNode);
            if not Assigned(lNode) then
              lNode := DocNode;
          end
          else
            lNode := DocNode;
          S := GetDescrString(lInterface, lNode.ShortDescr);
        end
        else
          S := '';

        WriteOverviewMember(L,N,S);
        S := '';
      end;
      EndOverview;
    end;
  finally
    List.Free;
  end;
end;

procedure TLinearWriter.WriteClassInheritanceOverview(ClassDecl: TPasClassType);
begin
  { Do nothing by default. This will be implemented by descendant writers. See
    the IPF Writer for an example. }
end;


function TLinearWriter.ConstValue(ConstDecl: TPasConst): String;
begin
  if Assigned(ConstDecl) then
    Result := ConstDecl.ClassName
  else
    Result := '<nil>';
end;

procedure TLinearWriter.DoWriteDocumentation;

var
  i : Integer;
  L : TstringList;

begin
  PackageName := LowerCase(Copy(Package.Name, 2, 255));
  If (Engine.OutPut='') then
    Engine.Output:=PackageName+FileNameExtension
  else if (ExtractFileExt(Engine.output)='') and (FileNameExtension<>'') then
    Engine.Output:=ChangeFileExt(Engine.output,FileNameExtension);
  FStream:=TFileStream.Create(Engine.Output,fmCreate);
  try
    WriteBeginDocument;
    ProcessPackage;
    L:=TStringList.Create;
    Try
      L.Sorted:=True;
      // Sort modules.
      For I:=0 to Package.Modules.Count-1 do
        L.AddObject(TPasModule(Package.Modules[i]).Name,TPasModule(Package.Modules[i]));
      // Now create table.
      for i:=0 to L.Count - 1 do
        begin
        Module := TPasModule(L.Objects[i]);
        ModuleName := LowerCase(Module.Name);
        WriteCommentLine;
        StartChapter(Format(SDocUnitTitle, [Module.Name]));
        WriteLabel(Module);
        // extra Topics now get processed in ProcessSection()
        ProcessSection(Module.InterfaceSection);
        end;
    Finally
      L.Free;
    end;
    WriteEndDocument;
  finally
    FSTream.Free;
  end;
end;

procedure TLinearWriter.ProcessSection(ASection: TPasSection);
var
  DocNode: TDocNode;
begin
  With ASection do
    begin
    SortElementList(UsesList);
    SortElementList(Declarations);
    SortElementList(ResStrings);
    SortElementList(Types);
    SortElementList(Consts);
    SortElementList(Classes);
    SortElementList(Functions);
    SortElementList(Variables);
    end;
  WriteUnitOverView(ASection);

  // Now process unit (extra) Topics
  DocNode:=Engine.FindDocNode(Module);
  If Assigned(DocNode) then
    ProcessTopics(DocNode,1);

  WriteVarsConstsTypes(ASection);
  WriteFunctionsAndProcedures(ASection);
  WriteExtendedRecords(ASection);
  WriteClasses(ASection);
end;

procedure TLinearWriter.WriteVarsConstsTypes(ASection: TPasSection);

begin
  With Asection do
    if (Consts.Count>0) or (Types.Count>0) or
       (Variables.Count>0) or (ResStrings.Count>0) then
      begin
      StartSection(SDocConstsTypesVars, ModuleName+'ConstsTypesVars');
      WriteResourceStrings(ASection);
      WriteConsts(ASection);
      WriteTypes(ASection);
      WriteVars(ASection);
      end;
end;

procedure TLinearWriter.WriteResourceStrings(ASection: TPasSection);
var
  ResStrDecl: TPasResString;
  i: Integer;
  DocNode : TDocNode;
begin
  if ASection.ResStrings.Count > 0 then
  begin
    StartSubSection(SDocResStrings,ModuleName+'ResStrings');
    for i := 0 to ASection.ResStrings.Count - 1 do
    begin
      ResStrDecl := TPasResString(ASection.ResStrings[i]);
      StartListing(false, '');
      DescrWriteText(UTF8Decode(ResStrDecl.GetDeclaration(True))); // instead of WriteLn() so we can do further processing like manual line wrapping in descendants
      EndListing;
      WriteLabel(ResStrDecl);
      WriteIndex(ResStrDecl);
      DocNode:=WriteDescr(ResStrDecl);
      If Assigned(DocNode) and Assigned(DocNode.Version) then
        begin
        Writeln(Format('%s : ',[SDocVersion]));
        WriteDescr(ResStrDecl, DocNode.Version);
        end;
    end;
  end;
end;


procedure TLinearWriter.WriteUnitOverview(ASection: TPasSection);

var
  i: Integer;
  UnitRef: TPasType;
  DocNode: TDocNode;
begin
  if ASection.UsesList.Count > 0 then
    begin
    StartSection(SDocUsedUnits);
    StartUnitOverview(Module.Name,ModuleName);
    for i := 0 to ASection.UsesList.Count - 1 do
      begin
      UnitRef := TPasType(ASection.UsesList[i]);
      WriteUnitEntry(UnitRef);
      end;
    EndUnitOverview;
    end;
  DocNode := Engine.FindDocNode(ASection.Parent);
  if Assigned(DocNode) and not IsDescrNodeEmpty(DocNode.Descr) then
    begin
    StartSection(SDocOverview);
    WriteDescr(ASection.Parent, DocNode.Descr);
    ConvertNotes(ASection.Parent,DocNode.Notes);
    end;
end;

procedure TLinearWriter.ProcessPackage;

var
  DocNode: TDocNode;

begin
  DocNode:=Engine.FindDocNode(Package);
  if not Assigned(DocNode) then
    exit;
  if Assigned(DocNode) and not IsDescrNodeEmpty(DocNode.Descr) then
    begin
    StartSection(SDocOverview);
    WriteDescr(Package, DocNode.Descr);
    end;
  WriteSeeAlso(DocNode);
  ConvertNotes(Nil,DocNode.Notes);
  ProcessTopics(DocNode,1);
end;

procedure TLinearWriter.ProcessTopics(DocNode: TDocNode; Alevel: Integer);

Var
  Node : TDocNode;

begin
  If Not Assigned(DocNode) then
    Exit;
  Node:=DocNode.FirstChild;
  While Assigned(Node) do
    begin
    If Node.TopicNode then
      WriteTopicNode(Node,ALevel);
    Node:=Node.NextSibling;
    end;
end;


procedure TLinearWriter.WriteTopicNode(Node: TDocNode; Level: Integer);

Var
  Element : TTopicElement;
  SubNode : TDocNode;
  S : String;

begin
  Element:=FindTopicElement(Node);
  If Not Assigned(Element) then
    Exit;
  S:=GetDescrString(Element,Node.ShortDescr);
  Case Level of
    1 : StartSection(S);
    2 : StartSubSection(S);
    3 : StartSubSubSection(S);
  end;
  WriteLabel(Element);
  If Assigned(Node.Descr) then
    WriteDescr(Element,Node.Descr);
  WriteSeeAlso(Node);
  ConvertNotes(Element,Node.Notes);
  If Level<3 then
    begin
    SubNode:=Node.FirstChild;
    While Assigned(SubNode) do
      begin
      If SubNode.TopicNode then
        WriteTopicNode(SubNode,Level+1);
      SubNode:=SubNode.NextSibling;
      end;
    end;
  WriteExample(Node);
end;

procedure TLinearWriter.WriteConsts(ASection: TPasSection);
var
  i: Integer;
  ConstDecl: TPasConst;
begin
  if ASection.Consts.Count > 0 then
    begin
    StartSubSection(SDocConstants,EscapeText(ModuleName));
    for i := 0 to ASection.Consts.Count - 1 do
      begin
      DescrBeginParaGraph;
      ConstDecl := TPasConst(ASection.Consts[i]);
      WriteConstDecl(ConstDecl);
      StartListing(False,'');
      WriteLn(EscapeText(ConstDecl.GetDeclaration(True)));
      EndListing;
      WriteLabel(ConstDecl);
      WriteIndex(ConstDecl);
      WriteDescr(ConstDecl);
      DescrEndParaGraph;
      end;
    end;
end;

procedure TLinearWriter.WriteEnumElements(TypeDecl : TPasEnumType);

Var
  EV : TPasEnumValue;
  I : Integer;
  DocNode : TDocNode;

begin
  With TypeDecl do
    begin
    SortElementList(Values);
    DescrBeginTable(2,True);
    DescrBeginTableCaption;
      Writeln(EscapeText(Format(SDocValuesForEnum,[TypeDecl.Name])));
    DescrEndTableCaption;
    DescrBeginTableHeadRow;
      DescrBeginTableCell;
        Writeln(EscapeText(SDocValue));
      DescrEndTableCell;
      DescrBeginTableCell;
        Writeln(EscapeText(SDocExplanation));
      DescrEndTableCell;
    DescrEndTableHeadRow;
    For I:=0 to Values.Count-1 do
      begin
      EV:=TPasEnumValue(Values[i]);
      DescrBeginTableRow;
        DescrBeginTableCell;
          Writeln(EscapeText(EV.Name));
        DescrEndTableCell;
        DescrBeginTableCell;
          DocNode := Engine.FindDocNode(EV);
          if Assigned(DocNode) and (not IsDescrNodeEmpty(DocNode.ShortDescr)) then
            WriteDescr(EV,DocNode.ShortDescr);
        DescrEndTableCell;
      DescrEndTableRow;
      end;
    DescrEndTable;
    end;
end;

procedure TLinearWriter.WriteTypes(ASection: TPasSection);
var
  i: Integer;
  TypeDecl: TPasType;
  DocNode : TDocNode;
begin
  if ASection.Types.Count > 0 then
    begin
    StartSubSection(SDocTypes,ModuleName+'Types');
    for i := 0 to ASection.Types.Count - 1 do
      begin
      TypeDecl := TPasType(ASection.Types[i]);
      DocNode := Engine.FindDocNode(TypeDecl);
      if Assigned(DocNode) and DocNode.IsSkipped then
        Continue;
      if not ((TypeDecl is TPasRecordType) and TPasRecordType(TypeDecl).IsAdvancedRecord) then
        begin
        DescrBeginParagraph;
        WriteTypeDecl(TypeDecl);
        StartListing(False,'');
        If Assigned(DocNode) and
           Assigned(DocNode.Node) and
           (Docnode.Node['opaque']='1') then
            Writeln(TypeDecl.Name+' = '+SDocOpaque)
        else
        begin
          Writeln(EscapeText(TypeDecl.GetDeclaration(True)));
        end;
        EndListing;
        WriteLabel(TypeDecl);
        WriteIndex(TypeDecl);
        If TypeDecl is TPasEnumType then
          WriteENumElements(TypeDecl as TPasEnumType)
        else If (TypeDecl is TPasSetType)
                and (TPasSetType(TypeDecl).EnumType is TPasEnumType)
                and (TPasSetType(TypeDecl).EnumType.Name='') then
          WriteENumElements(TPasSetType(TypeDecl).EnumType as TPasEnumType);
        WriteDescr(TypeDecl,DocNode);
        If Assigned(DocNode) and Assigned(DocNode.Version) then
          begin
          Writeln(Format('%s : ',[SDocVersion]));
          WriteDescr(TypeDecl, DocNode.Version);
          end;
        if Assigned(DocNode) and assigned(DocNode.Notes) then
          ConvertNotes(TypeDecl,DocNode.Notes);
        DescrEndParagraph;
        end;
      end;
    end;
end;

procedure TLinearWriter.WriteVars(ASection: TPasSection);
var
  VarDecl: TPasVariable;
  i: Integer;
  DocNode : TDocNode;

begin
  if ASection.Variables.Count > 0 then
  begin
    StartSubsection(SDocVariables,ModuleName+'Variables');
    for i := 0 to ASection.Variables.Count - 1 do
    begin
      DescrBeginParaGraph;
      VarDecl := TPasVariable(ASection.Variables[i]);
      WriteVariableDecl(VarDecl);
      StartListing(False,'');
      WriteLn(EscapeText(VarDecl.GetDeclaration(True)));
      EndListing;
      WriteLabel(VarDecl);
      WriteIndex(VarDecl);
      DocNode:=WriteDescr(VarDecl);
      If Assigned(DocNode) and Assigned(DocNode.Version) then
        begin
        Writeln(Format('%s : ',[SDocVersion]));
        WriteDescr(VarDecl, DocNode.Version);
        ConvertNotes(VarDecl,DocNode.Notes);
        end;
      DescrEndParaGraph;
    end;
  end;
end;


procedure TLinearWriter.WriteProcedure(ProcDecl : TPasProcedureBase);
var
  DocNode: TDocNode;
  OP : TPasOverloadedProc;
  i : integer;
begin
  With ProcDecl do
    begin
    DocNode := Engine.FindDocNode(ProcDecl);
    if Assigned(DocNode) and DocNode.IsSkipped then
      Exit;
    if Not (Assigned(Parent) and ((Parent.InheritsFrom(TPasClassType)) or Parent.InheritsFrom(TPasRecordType))) then
      begin
      StartSubSection(Name);
      WriteLabel(ProcDecl);
      WriteIndex(ProcDecl);
      end
    else
      begin // Parent assigned and hence method.
      StartSubSection(Parent.Name+'.'+Name);
      WriteLabel(ProcDecl);
      WriteIndex(Parent.Name+'.'+Name);
      end;
    StartProcedure;
    if Assigned(DocNode) and Assigned(DocNode.ShortDescr) then
      begin
      StartSynopsis;
      WriteDescr(ProcDecl, DocNode.ShortDescr);
      end;
    StartDeclaration;
    StartListing(False);
    if ClassType = TPasOverloadedProc then
      begin
      OP:=TPasOverloadedProc(ProcDecl);
      for i := 0 to OP.Overloads.Count - 1 do
        begin
        WriteLn(TPasProcedure(OP.Overloads[i]).GetDeclaration(True));
        end;
      end
    else
      WriteLn(GetDeclaration(True));
    EndListing;
    If Assigned(Parent) then
      begin
      StartVisibility;
      Writeln(VisibilityNames[Visibility])
      end;
    if Assigned(DocNode) then
      begin
      If Assigned(DocNode.Descr) then
        begin
        StartDescription;
        WriteDescr(ProcDecl);
        end;
      if Assigned(DocNode.ErrorsDoc) and (DocNode.ErrorsDoc.HasChildNodes) then
        begin
        StartErrors;
        WriteDescr(ProcDecl, DocNode.ErrorsDoc);
        end;
      if Assigned(DocNode.Version) then
        begin
        StartVersion;
        WriteDescr(ProcDecl, DocNode.Version);
        end;
      WriteSeeAlso(DocNode);
      EndProcedure;
      WriteExample(DocNode);
      ConvertNotes(ProcDecl,DocNode.Notes);
      end
     else
      EndProcedure;
    end;
end;

procedure TLinearWriter.WriteFunctionsAndProcedures(ASection: TPasSection);
var
  i: Integer;
begin
  if ASection.Functions.Count > 0 then
    begin
    StartSection(SDocProceduresAndFunctions,ModuleName+'Functions');
    for i := 0 to ASection.Functions.Count - 1 do
      WriteProcedure(TPasProcedureBase(ASection.Functions[i]));
    end;
end;

procedure TLinearWriter.WriteExample(ADocNode: TDocNode);

var
  Example: TDOMElement;
  S : string;

begin
  S:='';
  if Assigned(ADocNode) then
    begin
    Example := ADocNode.FirstExample;
    while Assigned(Example) do
      begin
      if IsExampleNode(Example) then
        begin
        if (S<>'') then // not first example, start new paragraph
          DescrBeginParagraph;
        s:=Engine.GetExampleFileName(Example);
        if (S<>'') then
          WriteExampleFile(S);
        if Assigned(Example.NextSibling) then
           DescrEndParaGraph;
        end;
      Example := TDomElement(Example.NextSibling);
      end;
    end;
end;

procedure TLinearWriter.WriteProperty(PropDecl : TPasProperty);
var
  DocNode: TDocNode;
  S: String;
  lNode: TDocNode;
begin
  With PropDecl do
    begin
    DocNode := Engine.FindDocNode(PropDecl);
    if Assigned(DocNode) and DocNode.IsSkipped then
      Exit;
    StartSubSection(Parent.Name+'.'+Name);
    WriteLabel(PropDecl);
    WriteIndex(Parent.Name+'.'+Name);
    StartProperty;
    if Assigned(DocNode) then
    begin
      if FDupLinkedDoc and (DocNode.Link <> '') then
      begin
        lNode := Engine.FindLinkedNode(DocNode);
        if not Assigned(lNode) then
          lNode := DocNode;
      end
      else
        lNode := DocNode;

      if Assigned(lNode.ShortDescr) then
      begin
        StartSynopsis;
        WriteDescr(PropDecl, lNode.ShortDescr);
      end;
    end;
    StartDeclaration;
    StartListing(False);
    WriteLn('Property '+GetDeclaration(True));
    EndListing;
    If Assigned(Parent) then
      begin
      StartVisibility;
      Writeln(VisibilityNames[Visibility])
      end;
    StartAccess;
    S:='';
    If Length(ReadAccessorName) > 0 then
      S:='Read';
    if Length(WriteAccessorName) > 0 then
      begin
      If S<>'' then
        S:=S+',';
      S:=S+'Write';
      end;
    Writeln(S);
    if Assigned(DocNode) then
    begin
      if Assigned(lNode.Descr) then     // lNode will be assigned if DocNode exists
        begin
        StartDescription;
        WriteDescr(PropDecl, lNode);
        end;
      if Assigned(lNode.ErrorsDoc) and (lNode.ErrorsDoc.HasChildNodes) then
        begin
        StartErrors;
        WriteDescr(PropDecl, DocNode.ErrorsDoc);
        end;
      if Assigned(lNode.Version) then
        begin
        StartVersion;
        WriteDescr(PropDecl, lNode.Version);
        end;
      WriteSeeAlso(lNode);
      ConvertNotes(PropDecl,lNode.Notes);
      EndProperty;
      WriteExample(lNode);
    end
    else
      EndProperty;
    end;
end;

procedure TLinearWriter.WriteSeeAlso(ADocNode: TDocNode);

var
  Node: TDOMNode;
  s: DOMString;
  First : Boolean;

begin
  if Not (Assigned(ADocNode) and Assigned(ADocNode.SeeAlso)) then
    Exit;
  Node := ADocNode.SeeAlso.FirstChild;
  First:=True;
  while Assigned(Node) do
    begin
    if IsLinkNode(Node) then
      begin
      If First then
        begin
        StartSeealso;
        First:=False;
        end
      else
        Writeln(',');
      S:=TDomElement(Node)['id'];
      DescrBeginLink(S);
      if Node.FirstChild <> nil then
        s := Node.FirstChild.NodeValue;
      Write(EscapeText(S));
      DescrEndLink();
      end;
    Node:=Node.NextSibling;
    end;
  If Not First then
    EndSeeAlso
end;

Function CompareElements(P1,P2 : Pointer) : Integer;

begin
  Result:=CompareText(TPasElement(P1).Name,TPasElement(P2).Name);
end;

procedure TLinearWriter.SortElementList(List : TFPList);

begin
  List.Sort(@CompareElements);
end;

function TLinearWriter.ShowMember(M: TPasElement): boolean;

begin
  Result:=Engine.ShowElement(M);
end;

procedure TLinearWriter.WriteClasses(ASection: TPasSection);
var
  i: Integer;
begin
  if (ASection.Classes.Count > 0) then
    for i := 0 to ASection.Classes.Count - 1 do
      WriteClassDecl(TPasClassType(ASection.Classes[i]));
end;

procedure TLinearWriter.WriteExtendedRecords(Asection: TPasSection);

var
  i,j : Integer;
  Recdecl : TPasRecordType;
  DocNode : TDocNode;
  Member : TPasElement;
begin
  if (ASection.Types.Count > 0) then
    for i := 0 to ASection.Types.Count - 1 do
      begin
      If TPasElement(ASection.Types[i]) is TPasRecordType then
        begin
        Recdecl:=TPasRecordType(ASection.Types[i]);
        if RecDecl.IsAdvancedRecord then
          begin
          StartSection(RecDecl.Name);
          DescrBeginParagraph;
          WriteTypeDecl(RecDecl);
          StartListing(False,'');
          DocNode := Engine.FindDocNode(RecDecl);
          If Assigned(DocNode) and
             Assigned(DocNode.Node) and
             (Docnode.Node['opaque']='1') then
              Writeln(RecDecl.Name+' = '+SDocOpaque)
          else
          begin
            Writeln(EscapeText(RecDecl.GetDeclaration(True)));
          end;
          EndListing;
          WriteLabel(RecDecl);
          WriteIndex(RecDecl);
          WriteDescr(RecDecl,DocNode);
          If Assigned(DocNode) and Assigned(DocNode.Version) then
            begin
            Writeln(Format('%s : ',[SDocVersion]));
            WriteDescr(RecDecl, DocNode.Version);
            end;
          if Assigned(DocNode) and assigned(DocNode.Notes) then
            ConvertNotes(RecDecl,DocNode.Notes);
          DescrEndParagraph;
          WriteMethodOverView(Recdecl,Recdecl.Members);
          WritePropertyOverView(Recdecl,Recdecl.Members);
          for J := 0 to Recdecl.Members.Count - 1 do
            begin
            Member := TPasElement(Recdecl.Members[j]);
            if Member.InheritsFrom(TPasProcedureBase) and Engine.ShowElement(Member) then
              WriteProcedure(TPasProcedureBase(Member));
            end;
          for j := 0 to Recdecl.Members.Count - 1 do
            begin
            Member := TPasElement(Recdecl.Members[j]);
            if Member.InheritsFrom(TPasProperty) and Engine.ShowElement(Member) then
              WriteProperty(TPasProperty(Member));
            end;
          end;
        end;
      end;
end;

procedure TLinearWriter.WriteMethodOverview(AParent: TPasType; Members : TFPList);

var
  Member : TPasElement;
  i : Integer;
  L,N,S : String;
  DocNode : TDocNode;
  List : TStringList;

begin
  List:=TStringList.Create;
  Try
    List.Sorted:=True;
    for i := 0 to Members.Count - 1 do
      begin
      Member := TPasElement(Members[i]);
      With Member do
        if InheritsFrom(TPasProcedureBase) and ShowMember(Member) then
      List.AddObject(Member.Name,Member);
      end;
    If List.Count>0 then
      begin
      StartSubSection(SDocMethodOverview);
      WriteLabel(GetLabel(AParent) + ':Methods');
      StartOverview(SDocMethod,False);
      For I:=0 to List.Count-1 do
        begin
        Member:=TPasElement(List.Objects[i]);
        L:=StripText(GetLabel(Member));
        N:=EscapeText(Member.Name);
        DocNode := Engine.FindDocNode(Member);
        if Assigned(DocNode) and DocNode.IsSkipped then
          Continue;
        If Assigned(DocNode) then
          S:=GetDescrString(Member, DocNode.ShortDescr)
        else
          S:='';
        WriteOverviewMember(L,N,S);
        end;
      EndOverview;
      end;
  Finally
    List.Free;
  end;
end;


constructor TLinearWriter.Create(APackage: TPasPackage; AEngine: TFPDocEngine);

  procedure AddLabel(AElement: TPasElement);
  begin
    Engine.AddLink(AElement.PathName, GetLabel(AElement));
  end;

  procedure AddList(AElement: TPasElement; AList: TFPList);
  var
    i,j: Integer;
    R : TPasRecordType;
    FPEl : TPaselement;
    DocNode: TDocNode;

  begin
    for i := 0 to AList.Count - 1 do
      begin
      AddLabel(TPasElement(AList[i]));
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
                 AddLabel(FPEl);
               end;
          end;
        end;
      end;
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
        Engine.AddLink(TopicElement.PathName, GetLabel(TopicElement));
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
  inherited ;
  FDupLinkedDoc := False; // by default we don't duplicate linked element documentation

  { Allocate labels for all elements for which we are going to create
    documentation. This is needed for links to work correctly. }

  // Allocate label for the package itself, if a name is given (i.e. <> '#')
  if Length(Package.Name) > 1 then
    begin
    AddLabel(Package);
    AddTopicPages(Package);
    end;
  for i := 0 to Package.Modules.Count - 1 do
    ScanModule(TPasModule(Package.Modules[i]));
end;

procedure TLinearWriter.WriteBeginDocument;

begin
  WriteComment('This file has been created automatically by FPDoc.');
  WriteComment('Linear output (c) 2005 Michael Van Canneyt');
end;

procedure TLinearWriter.WriteEndDocument;
begin
  // do nothing
end;

function TLinearWriter.EscapeText(S: UnicodeString): String;
begin
  Result:=EscapeText(UTF8Encode(S));
end;

function TLinearWriter.InterpretOption(const Cmd, Arg: String): Boolean;
begin
  Result := True;
  if Cmd = cDupLinkedDocParam then
  begin
    FDupLinkedDoc := True;
  end
  else
    Result := False;
end;

class procedure TLinearWriter.Usage(List: TStrings);
begin
  List.Add(cDupLinkedDocParam);
  List.Add(SLinearUsageDupLinkedDocsP1);
  List.Add('');
  List.Add(SLinearUsageDupLinkedDocsP2);
end;


end.

