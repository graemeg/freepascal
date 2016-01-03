{

    FPDoc  -  Free Pascal Documentation Tool
    Copyright (C) 2000 - 2003 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org
    2005-2012 by
      various FPC contributors

    * 'XML struct' output generator

    See the file COPYING, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
{$mode objfpc}
{$H+}


unit dw_XML;

interface

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses DOM, PasTree, dGlobals, dwriter, xmlWrite, SysUtils, Classes;
=======
uses DOM, PasTree, dwriter, xmlWrite, SysUtils;
>>>>>>> graemeg/cpstrnew
=======
uses DOM, PasTree, dwriter, xmlWrite, SysUtils;
>>>>>>> graemeg/cpstrnew
=======
uses DOM, PasTree, dwriter, xmlWrite, SysUtils;
>>>>>>> graemeg/cpstrnew
=======
uses DOM, PasTree, dwriter, xmlWrite, SysUtils;
>>>>>>> origin/cpstrnew

Type

  { TXMLWriter }

  TXMLWriter = Class(TFPDocWriter)
  private
    FShowSourceInfo: Boolean;
  public
    function ModuleToXMLStruct(AModule: TPasModule): TXMLDocument;
    Procedure WriteDoc; override;
    class procedure Usage(List: TStrings); override;
    function  InterPretOption(const Cmd,Arg : String): boolean; override;
  end;




implementation

const
  DefaultVisibility = [visDefault, visPublic, visPublished, visProtected];

function TXMLWriter.ModuleToXMLStruct(AModule: TPasModule): TXMLDocument;

var
  ModuleElement: TDOMElement;
  Doc: TXMLDocument absolute Result;

  function VisibilityToString(vis: TPasMemberVisibility): String;
  begin
    case vis of
      visDefault         : Result := '';
      visPrivate         : Result := 'private';
      visProtected       : Result := 'protected';
      visPublic          : Result := 'public';
      visPublished       : Result := 'published';
      visAutomated       : Result := 'automated';
      visStrictPrivate   : Result := 'strictprivate';
      visStrictProtected : Result := 'strictprotected';
    end;
  end;

  function Sanitize(AString: String): String;
  var
    i: Integer;
  begin
    Result := AString;
    for i := 1 to length(Result) do
      if Result[i] in [' '] then
        Result[i] := '_';
  end;

  procedure AddSourceInfo(ADecl: TPasElement; AElement: TDOMElement);
  var
    SourceNode: TDOMElement;
  begin
    if not FShowSourceInfo then
      Exit;
    SourceNode := Doc.CreateElement('source');
    SourceNode['line'] := IntToStr(ADecl.SourceLinenumber);
    SourceNode['file'] := ADecl.SourceFilename;
    AElement.AppendChild(SourceNode);
  end;

  procedure AddProcedureModifiers(ADecl: TPasProcedure; Node: TDOMElement);
  begin
    {pmVirtual , pmDynamic, pmAbstract, pmOverride,
    pmExport, pmOverload, pmMessage, pmReintroduce,
    pmStatic,pmInline,pmAssembler,pmVarargs, pmPublic,
    pmCompilerProc,pmExternal,pmForward}

    if (pmVirtual in ADecl.Modifiers) or (pmDynamic in ADecl.Modifiers) then
      Node['virtual'] := 'true';
    if pmAbstract in ADecl.Modifiers then
      Node['abstract'] := 'true';
    if pmStatic in ADecl.Modifiers then
      Node['static'] := 'true';
    if pmReintroduce in ADecl.Modifiers then
      Node['reintroduce'] := 'true';
    if pmOverload in ADecl.Modifiers then
      Node['overload'] := 'true';
    if pmForward in ADecl.Modifiers then
      Node['forward'] := 'true';
    if pmOverride in ADecl.Modifiers then
      Node['override'] := 'true';
  end;

  procedure AddTypeNode(ToNode: TDOMElement; AType: String);
  begin
    ToNode.AttribStrings['type'] := AType;
  end;

  function AddTypeNode(ToNode: TDOMElement; AType: TPasType): Boolean;
  //var
  //  TypeNode: TDOMElement;
  begin
    Result := False;
    if not Assigned(AType) then
      Exit;
    //TypeNode := Doc.CreateElement('type');
    //TypeNode.TextContent:=AType.Name;
    //ToNode.AppendChild(TypeNode);
    AddTypeNode(ToNode, AType.Name);
    Result := True;
  end;

  procedure ProcessArgs(Args: TFPList; ProcNode: TDomElement);
  var
    i: Integer;
    ArgNode: TDOMElement;
    Arg: TPasArgument;
  begin
    for i := 0 to Args.Count-1 do
    begin
      Arg := TPasArgument(Args.Items[i]);
      ArgNode := Doc.CreateElement('argument');
      ArgNode.AttribStrings['name'] := Arg.Name;
      AddTypeNode(ArgNode, Arg.ArgType);
      ProcNode.AppendChild(ArgNode);
    end;
  end;

  procedure DoVisibility(PasEl: TPasElement; Element: TDOMElement);
  begin
    if PasEl.Visibility <> visDefault then
      Element['visibility'] := VisibilityToString(PasEl.Visibility);
  end;

  function ProcessProcedure(Proc: TPasProcedure; Element: TDOMElement): TDOMElement;
  var
    ProcEl: TDOMElement;
    ReturnEl: TDOMElement;
  begin
    Result := nil;
    ProcEl := Doc.CreateElement(Sanitize(Proc.TypeName));
    Element.AppendChild(ProcEl);
    ProcEl['name'] := Proc.Name;

    DoVisibility(Proc, ProcEl);

    AddProcedureModifiers(Proc, ProcEl);
    AddSourceInfo(Proc,ProcEl);

    if Proc.InheritsFrom(TPasFunction) then
    begin
      ReturnEl := Doc.CreateElement('return');
      ProcEl.AppendChild(ReturnEl);
      AddTypeNode(ReturnEl, TPasFunction(Proc).FuncType.ResultEl.ResultType);
    end;

    ProcessArgs(Proc.ProcType.Args, ProcEl);

    Result := ProcEl;
  end;

  procedure ProcessArrayType(AType: TPasArrayType; Element: TDOMElement);
  var
    TypeEl: TDOMElement;
  begin
    TypeEl := Doc.CreateElement('array');
    TypeEl['name'] := AType.Name;
    if not AddTypeNode(TypeEl, AType.ElType) then
      TypeEl['const'] := 'true';
    TypeEl['range'] := AType.IndexRange;
    DoVisibility(AType, Element);
    AddSourceInfo(AType,Element);
    Element.AppendChild(TypeEl);
  end;

  procedure ProcessPointerType(AType: TPasPointerType; Element: TDOMElement);
  var
    TypeEl: TDOMElement;
  begin
    TypeEl := Doc.CreateElement('pointer');
    TypeEl['name'] := AType.Name;
    AddTypeNode(TypeEl, AType.DestType);
    DoVisibility(AType, Element);
    AddSourceInfo(AType,Element);

    Element.AppendChild(TypeEl);
  end;

  procedure ProcessAliasType(AType: TPasAliasType; Element: TDOMElement);
  var
    TypeEl: TDOMElement;
  begin
    TypeEl := Doc.CreateElement('alias');
    TypeEl['name'] := AType.Name;
    AddTypeNode(TypeEl, AType.DestType);
    DoVisibility(AType, Element);
    AddSourceInfo(AType,Element);
    Element.AppendChild(TypeEl);
  end;

  procedure ProcessVariable(AVar: TPasVariable; Element: TDOMElement);
  var
    VarEl: TDOMElement;
  begin
    VarEl := Result.CreateElement('var');
    Element.AppendChild(VarEl);
    VarEl['name'] := AVar.Name;
    if not AVar.VarType.InheritsFrom(TPasArrayType) then
      AddTypeNode(VarEl, AVar.VarType)
    else
    begin
      VarEl['array'] := 'true';
      ProcessArrayType(TPasArrayType(AVar.VarType), VarEl);
    end;
    DoVisibility(Avar, VarEl);
    AddSourceInfo(AVar,VarEl);
  end;

  procedure ProcessProperty(AProp: TPasProperty; Element: TDOMElement);
  var
    PropEl: TDOMElement;
  begin
    PropEl := Doc.CreateElement('property');
    Element.AppendChild(PropEl);

    PropEl.AttribStrings['name'] := AProp.Name;
    AddTypeNode(PropEL, AProp.ResolvedType);

    if AProp.IndexValue <> '' then
      PropEl['index'] := AProp.IndexValue;

    if AProp.DefaultValue <> '' then
      PropEl['default'] := AProp.DefaultValue;


    if AProp.WriteAccessorName <> '' then
      PropEl.AttribStrings['writable'] := 'true';

    ProcessArgs(AProp.Args, PropEl);
    DoVisibility(AProp, Element);
    AddSourceInfo(AProp,PropEl);

    // this isn't quite right
    //if AProp.ReadAccessorName = '' then
    //  PropEl.AttribStrings['inherited'] := 'true';
  end;

  procedure ProcessOverloadedProcedure(AOverload: TPasOverloadedProc; Element: TDOMElement);
  var
    OverEl: TDOMElement;
    i: Integer;
  begin
    for i := 0 to AOverload.Overloads.Count-1 do
    begin
      OverEl := ProcessProcedure(TPasProcedure(AOverload.Overloads.Items[i]), Element);
      OverEl['overload'] := 'true';
    end;
  end;

  procedure ProcessConst(AConst: TPasConst; Element: TDOMElement);
  var
    ConstEl: TDOMElement;
  begin
    ConstEl := Doc.CreateElement('const');
    ConstEl['name'] := AConst.name;
    ConstEl['value'] := AConst.Value;
    Element.AppendChild(ConstEl);
    AddSourceInfo(AConst,ConstEl);
  end;

  procedure ProcessEnumType(AType: TPasEnumType; Element: TDOMElement);
  var
    TypeEl: TDOMElement;
    ValEl: TDOMELement;
    i: Integer;
  begin
    TypeEl := Doc.CreateElement('enum');
    TypeEl['name'] := AType.name;
    AddSourceInfo(AType,TypeEl);
    //ConstEl['value'] := AConst.Value;
    for i := 0 to AType.Values.Count-1 do
    begin
      ValEl := Doc.CreateElement('enumvalue');
      ValEl['name'] := TPasEnumValue(AType.Values.Items[i]).Name;
      AddSourceInfo(TPasEnumValue(AType.Values.Items[i]),ValEl);
      TypeEl.AppendChild(ValEl);

    end;
    Element.AppendChild(TypeEl);
  end;

  procedure ProcessSetType(AType: TPasSetType; Element: TDOMElement);
  var
    SetEl: TDOMElement;
  begin
    SetEl := Doc.CreateElement('set');
    SetEl['name'] := AType.name;
    AddTypeNode(SetEl, AType.EnumType);
    AddSourceInfo(AType,SetEl);
    Element.AppendChild(SetEl);
  end;

  procedure ProcessProcedureType(AType: TPasProcedureType; Element: TDOMElement);
  var
    TypeEl: TDOMElement;
  begin
    TypeEl := Doc.CreateElement(AType.TypeName);
    TypeEl['name'] := AType.name;
    TypeEl['istype'] := 'true';
    if AType.IsOfObject then
      TypeEl['object'] := 'true';
    ProcessArgs(AType.Args, TypeEl);
    AddSourceInfo(AType,TypeEl);
    Element.AppendChild(TypeEl);
  end;

  procedure ProcessRecordType(AType: TPasRecordType; Element: TDOMElement);
  var
    TypeEl: TDOMElement;
    Decl: TPasElement;
    i: Integer;
  begin
    TypeEl := Doc.CreateElement('record');
    TypeEl['name'] := AType.name;

    Element.AppendChild(TypeEl);
    AddSourceInfo(AType,TypeEl);

    if Assigned(AType.Members) then
      for i := 0 to AType.Members.Count - 1 do
      begin
        Decl := TPasElement(AType.Members[i]);
        if Decl.InheritsFrom(TPasProcedure)then
          ProcessProcedure(TPasProcedure(Decl), TypeEl)
        else if Decl.ClassType = TPasVariable then
          ProcessVariable(TPasVariable(Decl), TypeEl)
        else if Decl.ClassType = TPasProperty then
          ProcessProperty(TPasProperty(Decl), TypeEl)
        else writeln('Unhandled record member: ', Decl.ClassName, ' ', Decl.Name);
      end;
  end;

  procedure ProcessGenericTypes(AGenericTypes: TFPList; ANode: TDOMElement);
  var
    i: Integer;
    Node: TDOMElement;
  begin
    for i := 0 to AGenericTypes.Count-1 do
    begin
      Node := Doc.CreateElement('t');
      Node['name'] := TPasGenericTemplateType(AGenericTypes.Items[i]).Name;
      ANode.AppendChild(Node);
      AddSourceInfo(TPasGenericTemplateType(AGenericTypes.Items[i]),Node);
    end;
  end;

  procedure ProcessRangeType(AType: TPasRangeType; Element: TDOMElement);
  var
    TypeEl: TDOMElement;
  begin
    TypeEl := Doc.CreateElement('range');
    TypeEl['name'] := AType.Name;
    TypeEl['start'] := AType.RangeStart;
    TypeEl['end'] := AType.RangeEnd;
    AddSourceInfo(AType,TypeEl);

    Element.AppendChild(TypeEl);

  end;

  procedure ProcessClassType(AClass: TPasClassType; Element: TDOMElement); forward;

  function ProcessType(AType: TPasElement; Element: TDOMElement): Boolean;
  begin
    Result := True;
    if AType.ClassType = TPasVariable then
      ProcessVariable(TPasVariable(AType), Element)
    else if AType.ClassType = TPasProperty then
      ProcessProperty(TPasProperty(AType), Element)
    else if AType.InheritsFrom(TPasOverloadedProc) then
      ProcessOverloadedProcedure(TPasOverloadedProc(AType), Element)
    else if AType.InheritsFrom(TPasConst) then
      ProcessConst(TPasConst(AType), Element)
    else if AType.InheritsFrom(TPasEnumType) then
      ProcessEnumType(TPasEnumType(AType), Element)
    else if AType.InheritsFrom(TPasClassType) then
      ProcessClassType(TPasClassType(AType), Element)
    else if AType.InheritsFrom(TPasAliasType) then
      ProcessAliasType(TPasAliasType(AType), Element)
    else if AType.InheritsFrom(TPasSetType) then
      ProcessSetType(TPasSetType(AType), Element)
    else if AType.InheritsFrom(TPasProcedureType) then
      ProcessProcedureType(TPasProcedureType(AType), Element)
    else if AType.InheritsFrom(TPasRecordType) then
      ProcessRecordType(TPasRecordType(AType), Element)
    else if AType.InheritsFrom(TPasArrayType) then
      ProcessArrayType(TPasArrayType(AType), Element)
    else if AType.InheritsFrom(TPasPointerType) then
      ProcessPointerType(TPasPointerType(AType), Element)
    else if AType.InheritsFrom(TPasRangeType) then
      ProcessRangeType(TPasRangeType(AType), Element)
    else
      Result := False;
  end;

  procedure ProcessClassType(AClass: TPasClassType; Element: TDOMElement);
  var
    ClassEl: TDOMElement = nil;
    i: Integer;
    Decl: TPasElement;
    SubNode: TDomElement;
    InterfaceEl: TDomElement;
    Vis: TPasMemberVisibilities = DefaultVisibility;
  begin
    if not Engine.HidePrivate then Include(Vis, visPrivate);
    if Engine.HideProtected then Exclude(Vis, visProtected);
    case AClass.ObjKind of
      okClass: ClassEl := Result.CreateElement('class');
      okObject: ClassEl := Result.CreateElement('object');
      okInterface: ClassEl := Result.CreateElement('interface');
      okSpecialize: ClassEl := Result.CreateElement('classspecialized');
      //okGeneric: Result.CreateElement('generic');
      //okClassHelper: Result.CreateElement('classhelper');
      //okRecordHelper: Result.CreateElement('recordhelper');
      //okTypeHelper: Result.CreateElement('typehelper');

    else
      //raise Exception.Create('ProcessClass: unknown class kind');
      WriteLn('Unhandled Class kind: ', AClass.ObjKind);
    end;

    if Assigned(ClassEl) then
    begin
      Element.AppendChild(ClassEl);
      ClassEl['name'] := AClass.Name;
      if Assigned(AClass.AncestorType) then
        ClassEl['parentclass'] := AClass.AncestorType.Name;

      if AClass.ObjKind = okSpecialize then
      begin
        ProcessGenericTypes(AClass.GenericTemplateTypes, ClassEl);
      end;
      AddSourceInfo(AClass,ClassEl);

      if Assigned(AClass.Interfaces) then
        for i := 0 to AClass.Interfaces.Count-1 do
        begin
          InterfaceEl := Doc.CreateElement('interface');
          ClassEl.AppendChild(InterfaceEl);
          InterfaceEl['name'] := TPasElement(AClass.Interfaces.Items[i]).Name;
        end;

      if Assigned(AClass.Members) then
      for i := 0 to AClass.Members.Count - 1 do
      begin
        Decl := TPasElement(AClass.Members[i]);
        if not (Decl.Visibility in Vis) then
          continue;
        if Decl.InheritsFrom(TPasProcedure)then
        begin
          SubNode := ProcessProcedure(TPasProcedure(Decl), ClassEl);
          if Assigned(SubNode) then
          begin
            if SubNode.InheritsFrom(TPasClassConstructor) then
              SubNode.SetAttribute('type', 'constructor')
            else if SubNode.InheritsFrom(TPasClassDestructor) then
              SubNode.SetAttribute('type', 'destructor');
          end;
        end
        else if not ProcessType(Decl, ClassEl) then
          writeln('Unhandled class member: ', Decl.ClassName, ' ', Decl.Name);
      end;
    end;
  end;

  function FindInList(AName: String; AList: TFPList): Boolean;
  var
    El: TPasElement;
    I: Integer;
  begin
    Result := False;
    I := 0;
    while not Result and (I < AList.Count) do
    begin
      El := TPasElement(AList[I]);
      if El.Name = AName then
        Result := True;
      Inc(I);
    end;
  end;


  procedure ProcessSection(ASection: TPasSection; const Name: DOMString);
  var
    Element, UsesElement, UnitElement: TDOMElement;
    i: Integer;
    Decl: TPasElement;
  begin
    Element := Result.CreateElement(Name);
    ModuleElement.AppendChild(Element);
    if ASection.UsesList.Count > 0 then
    begin
      UsesElement := Result.CreateElement('uses');
      Element.AppendChild(UsesElement);
      for i := 0 to ASection.UsesList.Count - 1 do
      begin
        UnitElement := Result.CreateElement('unit-ref');
        UnitElement['name'] := TPasType(ASection.UsesList[i]).Name;
        UsesElement.AppendChild(UnitElement);
      end;
    end;

    for i := 0 to ASection.Classes.Count -1 do
    begin
      Decl := TPasElement(ASection.Classes[i]);
      ProcessClassType(TPasClassType(Decl), Element);
    end;

    for i := 0 to ASection.Consts.Count - 1 do
    begin
      Decl := TPasElement(ASection.Consts[i]);
      ProcessConst(TPasConst(Decl), Element)
    end;

    for i := 0 to ASection.Types.Count - 1 do
    begin
      Decl := TPasElement(ASection.Types[i]);
      if not ProcessType(Decl, Element) then
        WriteLn('Unhandled type: ',Decl.ClassName, ' ', Decl.Name);
    end;

    for i := 0 to ASection.Declarations.Count - 1 do
    begin
      Decl := TPasElement(ASection.Declarations[i]);
      if Decl.InheritsFrom(TPasProcedure) then
        ProcessProcedure(TPasProcedure(Decl), Element)
      else if Decl.ClassType = TPasVariable then
        ProcessVariable(TPasVariable(Decl), Element);
    end;

    for i := 0 to ASection.Functions.Count - 1 do
    begin
      // many of these (all?) seem to be in ASection.Declarations
      Decl := TPasElement(ASection.Functions[i]);
      if FindInList(Decl.Name, ASection.Declarations) then
        WriteLn('Duplicate proc definition in declarations. Skipping: ', Decl.Name)
      else
        WriteLn('Unhandled function: ',Decl.ClassName, ' ', Decl.Name);

    end;

    for i := 0 to ASection.Properties.Count - 1 do
    begin
      Decl := TPasElement(ASection.Properties[i]);
      ProcessProperty(TPasProperty(Decl), Element);
    end;
  end;


begin
  Result := TXMLDocument.Create;
  Result.AppendChild(Result.CreateComment(SDocGeneratedByComment));
  Result.AppendChild(Result.CreateElement('fp-refdoc'));
  ModuleElement := Result.CreateElement('unit');
  ModuleElement['name'] := AModule.Name;
  Result.DocumentElement.AppendChild(ModuleElement);
  ProcessSection(AModule.InterfaceSection, 'interface');
end;

{ TXMLWriter }

procedure TXMLWriter.WriteDoc;
var
  doc: TXMLDocument;
  i: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  if Engine.Output <> '' then
    Engine.Output := IncludeTrailingBackSlash(Engine.Output);

  for i := 0 to Package.Modules.Count - 1 do
  begin
    doc := ModuleToXMLStruct(TPasModule(Package.Modules[i]));
    WriteXMLFile(doc, Engine.Output + TPasModule(Package.Modules[i]).Name + '.xml' );
    doc.Free;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
end;

class procedure TXMLWriter.Usage(List: TStrings);
begin
  List.AddStrings(['--source-info', SXMLUsageSource]);
end;

function TXMLWriter.InterPretOption(const Cmd, Arg: String): boolean;
begin
  Result := True;
  if Cmd = '--source-info' then
    FShowSourceInfo:=True
  else
    Result:=inherited InterPretOption(Cmd, Arg);
=======
begin
  if Engine.Output <> '' then
    Engine.Output := IncludeTrailingBackSlash(Engine.Output);

  for i := 0 to Package.Modules.Count - 1 do
  begin
    doc := ModuleToXMLStruct(TPasModule(Package.Modules[i]));
    WriteXMLFile(doc, Engine.Output + TPasModule(Package.Modules[i]).Name + '.xml' );
    doc.Free;
  end;
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

initialization
  // Do not localize.
  RegisterWriter(TXMLWriter,'xml','fpdoc XML output.');
finalization
  UnRegisterWriter('xml');
end.
