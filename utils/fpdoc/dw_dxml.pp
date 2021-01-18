unit dw_dXML;

{$mode objfpc}{$H+}

interface

uses
  PasTree, dwriter, SysUtils;
//uses DOM, PasTree, dwriter, xmlWrite, SysUtils;

type
  { TXMLWriter }

  TDXMLWriter = class(TFPDocWriter)
    procedure DoWriteDocumentation; override;
  end;

  { TDocumentation }

  TDocumentation = class(TPassTreeVisitor)
    f:   Text;
    lvl: integer;

    procedure GenerateDoc(OutputName: string; Module: TPasModule);

    procedure DocParameters(obj: TPasProcedureType);
    function DocProcFlags(obj: TPasProcedure): string;

    procedure Visit(obj: TPasElement); override;

    procedure DoVisit(obj: TPasSection); virtual;

    procedure DoVisit(obj: TPasRecordType); virtual;
    procedure DoVisit(obj: TPasEnumType); virtual;
    procedure DoVisit(obj: TPasProperty); virtual;
    procedure DoVisit(obj: TPasConst); virtual;
    procedure DoVisit(obj: TPasVariable); virtual;
    procedure DoVisit(obj: TPasProcedure); virtual;
    procedure DoVisit(obj: TPasDestructor); virtual;
    procedure DoVisit(obj: TPasConstructor); virtual;
    procedure DoVisit(obj: TPasFunction); virtual;
    procedure DoVisit(obj: TPasClassType); virtual;
    procedure DoVisit(obj: TPasElement); virtual;
    procedure DoVisit(obj: TPasOverloadedProc); virtual;
    procedure DoVisit(obj: TPasPointerType); virtual;
    procedure DoVisit(obj: TPasArrayType); virtual;
    procedure DoVisit(obj: TPasProcedureType); virtual;
    procedure DoVisit(obj: TPasFunctionType); virtual;
    procedure DoVisit(obj: TPasResString); virtual;
  end;

implementation

function EscapeXml(const s: string): string;
begin
  Result := StringReplace(s, '&', '&amp;', [rfReplaceAll]);
  Result := StringReplace(Result, '<', '&lt;', [rfReplaceAll]);
  Result := StringReplace(Result, '>', '&gt;', [rfReplaceAll]);
end;

{ TDocumentation }

procedure TDocumentation.Visit(obj: TPasElement); 

begin
  If (Obj.ClassType=TPasSection) then
    DoVisit(TPasSection(Obj))
  else if (Obj.ClassType=TPasRecordType) then
    DoVisit(TPasRecordType(Obj))
  else if (Obj.ClassType=TPasEnumType) then
    DoVisit(TPasEnumType(Obj))
  else if (Obj.ClassType=TPasProperty) then
    DoVisit(TPasProperty(Obj))
  else if (Obj.ClassType=TPasConst) then
    DoVisit(TPasConst(Obj))
  else if (Obj.ClassType=TPasVariable) then
    DoVisit(TPasVariable(Obj))
  else if (Obj.ClassType=TPasProcedure) then
    DoVisit(TPasProcedure(Obj))
  else if (Obj.ClassType=TPasDestructor) then
    DoVisit(TPasDestructor(Obj))
  else if (Obj.ClassType=TPasConstructor) then
    DoVisit(TPasConstructor(Obj))
  else if (Obj.ClassType=TPasFunction) then
    DoVisit(TPasFunction(Obj))
  else if (Obj.ClassType=TPasClassType) then
    DoVisit(TPasClassType(Obj))
  else if (Obj.ClassType=TPasOverloadedProc) then
    DoVisit(TPasOverloadedProc(Obj))
  else if (Obj.ClassType=TPasPointerType) then
    DoVisit(TPasPointerType(Obj))
  else if (Obj.ClassType=TPasArrayType) then
    DoVisit(TPasArrayType(Obj))
  else if (Obj.ClassType=TPasProcedureType) then
    DoVisit(TPasProcedureType(Obj))
  else if (Obj.ClassType=TPasFunctionType) then
    DoVisit(TPasFunctionType(Obj))
  else if (Obj.ClassType=TPasResString) then
    DoVisit(TPasResString(Obj));
end;

procedure TDocumentation.GenerateDoc(OutputName: string; Module: TPasModule);
begin
  lvl := 0;
  Assign(f, OutputName);
  Rewrite(f);
  WriteLn(f, '<?xml version="1.0" encoding="utf-8"?>');
  WriteLn(f, '<namespace name="', Module.Name, '">');

  Module.InterfaceSection.Accept(Self);
  //Module.Accept(Self);

  WriteLn(f, '</namespace>');
  Close(f);
end;

procedure TDocumentation.DocParameters(obj: TPasProcedureType);
var
  I: integer;
begin
  for I := 0 to obj.Args.Count - 1 do
  begin
    Write(f, ' ': lvl * 2, '<parameter name="' + TPasArgument(obj.Args[i]).Name + '"');

    if TPasArgument(obj.Args[i]).ArgType <> nil then
      Write(f, ' type="' + TPasArgument(obj.Args[i]).ArgType.Name + '"');

    if TPasArgument(obj.Args[i]).Access <> argDefault then
      if (TPasArgument(obj.Args[i]).ArgType is TPasClassType) then
        Write(f, ' paramflags="' + 'var' + '"')
      else
        Write(f, ' paramflags="' +
          Trim(AccessNames[TPasArgument(obj.Args[i]).Access]) + '"');

    if TPasArgument(obj.Args[i]).Value <> '' then
    begin
      WriteLn(f, '>');
      WriteLn(f, ' ': lvl * 2 + 2, '<value>');
      WriteLn(f, ' ': lvl * 2 + 4, EscapeXml(TPasArgument(obj.Args[i]).Value));
      WriteLn(f, ' ': lvl * 2 + 2, '</value>');
      WriteLn(f, ' ': lvl * 2, '</parameter>');
    end
    else
      WriteLn(f, ' />');

  end;
end;

function TDocumentation.DocProcFlags(obj: TPasProcedure): string;

  procedure DoAdd(B: boolean; S: string);
  begin
    if B then
    begin
      if Result <> '' then
        Result := Result + ' ';
      Result   := Result + S;
    end;
  end;

begin
  Result := '';
  DoAdd(obj.IsAbstract, 'abstract');
  Doadd(obj.IsVirtual, 'virtual');
  DoAdd(obj.IsDynamic, 'dynamic');
  DoAdd(obj.IsOverride, 'override');
  DoAdd(obj.IsOverload, 'overload');
  DoAdd(obj.IsReintroduced, 'reintroduce');
  DoAdd(obj.IsStatic, 'static');
  DoAdd(obj.IsMessage, 'message');
end;

procedure TDocumentation.DoVisit(obj: TPasSection);
var
  i: integer;
begin
  Inc(lvl);
  for i := 0 to obj.Declarations.Count - 1 do
    TPasElement(obj.Declarations[i]).Accept(Self);
  Dec(lvl);
end;

procedure TDocumentation.DoVisit(obj: TPasRecordType);
var
  I: integer;
begin
  Write(f, StringOfChar(' ', lvl * 2) + '<struct');
  if obj.Name <> '' then
    Write(f, ' name="' + obj.Name + '"');
  if obj.IsPacked then
    Write(f, ' packed="true"');
  WriteLn(f, '>');
  Inc(lvl);
  for I := 0 to obj.Members.Count - 1 do
    TPasVariable(obj.Members[i]).Accept(Self);
  Dec(lvl);
  WriteLn(f, StringOfChar(' ', lvl * 2) + '</struct>');
end;

procedure TDocumentation.DoVisit(obj: TPasEnumType);
var
  I: integer;
begin
  for I := 0 to obj.Values.Count - 1 do
  begin
    WriteLn(f, ' ': lvl * 2, '<const name="' + TPasEnumValue(obj.Values[i]).Name + '" type="' +
      obj.Name + '">');
    WriteLn(f, ' ': lvl * 2 + 2, '<value>');
    WriteLn(f, ' ': lvl * 2 + 4, TPasEnumValue(obj.Values[i]).Name);
    WriteLn(f, ' ': lvl * 2 + 2, '</value>');
    WriteLn(f, ' ': lvl * 2, '</const>');
  end;

  WriteLn(f, ' ': lvl * 2, '<enum name="' + obj.Name + '">');
  for I := 0 to obj.Values.Count - 1 do
    WriteLn(f, ' ': lvl * 2 + 2, '<element name="' + TPasEnumValue(obj.Values[i]).Name + '" />');
  WriteLn(f, ' ': lvl * 2, '</enum>');
end;

procedure TDocumentation.DoVisit(obj: TPasProperty);
begin
  if (obj.VarType <> nil) and (obj.VarType is TPasProcedureType) and
    (TPasProcedureType(obj.VarType).IsOfObject) then
    Write(f, ' ': lvl * 2, '<event name="' + obj.Name + '" visibility="' +
      VisibilityNames[obj.Visibility] + '"')
  else
    Write(f, ' ': lvl * 2, '<property name="' + obj.Name + '" visibility="' +
      VisibilityNames[obj.Visibility] + '"');
  if obj.ReadAccessorName <> '' then
    Write(f, ' read="' + obj.ReadAccessorName + '"');
  if obj.WriteAccessorName <> '' then
    Write(f, ' write="' + obj.WriteAccessorName + '"');
  if obj.VarType <> nil then
    Write(f, ' type="' + obj.VarType.Name + '"');
  if obj.DefaultValue <> '' then
    Write(f, ' default="' + obj.DefaultValue + '"');
  WriteLn(f, ' />');
end;

procedure TDocumentation.DoVisit(obj: TPasConst);
begin
  Write(f, ' ': lvl * 2, '<const name="' + obj.Name + '"');
  if (obj.VarType <> nil) and (obj.VarType.Name <> '') then
    Write(f, ' type="' + obj.VarType.Name + '"');
  WriteLn(f, '>');
  WriteLn(f, ' ': lvl * 2 + 2, '<value>');
  WriteLn(f, ' ': lvl * 2 + 4, EscapeXml(obj.Value));
  WriteLn(f, ' ': lvl * 2 + 2, '</value>');
  WriteLn(f, ' ': lvl * 2, '</const>');
end;

procedure TDocumentation.DoVisit(obj: TPasVariable);
begin
  Write(f, ' ': lvl * 2, '<field name="' + obj.Name + '"');
  if (obj.VarType <> nil) and (obj.VarType.Name <> '') then
    Write(f, ' type="' + obj.VarType.Name {.GetDeclaration(True)} + '"');
  if obj.Visibility <> visDefault then
    Write(f, ' visibility="' + VisibilityNames[obj.Visibility] + '"');

  if (obj.VarType <> nil) and (obj.VarType.Name = '')
  {(VarType.ElementTypeName <> SPasTreeType) and (VarType.ElementTypeName <> SPasTreeUnresolvedTypeRef)}
  then
  begin
    WriteLn(f, '>');
    Inc(lvl);
    obj.VarType.Accept(Self);
    Dec(lvl);
    WriteLn(f, ' ': lvl * 2, '</field>');
  end
  else
    WriteLn(f, ' />');
end;

procedure TDocumentation.DoVisit(obj: TPasProcedure);
var
  t: string;
begin
  Write(f, ' ': lvl * 2, '<procedure name="' + obj.Name + '"');
  if obj.Visibility <> visDefault then
    Write(f, ' visibility="' + VisibilityNames[obj.Visibility] + '"');
  t := DocProcFlags(obj);
  if t <> '' then
    Write(f, ' procflags="' + t + '"');
  WriteLn(f, '>');
  Inc(lvl);

  if obj.ProcType.Args.Count > 0 then
  begin
    WriteLn(f, ' ': lvl * 2, '<parameters>');
    Inc(lvl);
    DocParameters(obj.ProcType);
    Dec(lvl);
    WriteLn(f, ' ': lvl * 2, '</parameters>');
  end;

  Dec(lvl);
  WriteLn(f, ' ': lvl * 2, '</procedure>');
end;

procedure TDocumentation.DoVisit(obj: TPasDestructor);
begin
  Write(f, ' ': lvl * 2, '<destructor name="' + obj.Name + '"');
  if obj.Visibility <> visDefault then
    Write(f, ' visibility="' + VisibilityNames[obj.Visibility] + '"');
  WriteLn(f, '>');
  Inc(lvl);
  WriteLn(f, ' ': lvl * 2, '<parameters>');
  Inc(lvl);
  DocParameters(obj.ProcType);
  Dec(lvl);
  WriteLn(f, ' ': lvl * 2, '</parameters>');
  Dec(lvl);
  WriteLn(f, ' ': lvl * 2, '</destructor>');
end;

procedure TDocumentation.DoVisit(obj: TPasConstructor);
begin
  Write(f, ' ': lvl * 2, '<constructor name="' + obj.Name + '"');
  if obj.Visibility <> visDefault then
    Write(f, ' visibility="' + VisibilityNames[obj.Visibility] + '"');
  WriteLn(f, '>');
  Inc(lvl);
  WriteLn(f, ' ': lvl * 2, '<parameters>');
  Inc(lvl);
  DocParameters(obj.ProcType);
  Dec(lvl);
  WriteLn(f, ' ': lvl * 2, '</parameters>');
  Dec(lvl);
  WriteLn(f, ' ': lvl * 2, '</constructor>');
end;

procedure TDocumentation.DoVisit(obj: TPasFunction);
var
  t: string;
begin
  Write(f, ' ': lvl * 2, '<function name="' + obj.Name + '"');
  if obj.Visibility <> visDefault then
    Write(f, ' visibility="' + VisibilityNames[obj.Visibility] + '"');
  t := DocProcFlags(obj);
  if t <> '' then
    Write(f, ' procflags="' + t + '"');
  WriteLn(f, '>');
  Inc(lvl);
  WriteLn(f, ' ': lvl * 2, '<parameters>');
  Inc(lvl);
  DocParameters(obj.ProcType);
  WriteLn(f, ' ': lvl * 2, '<retval type="' +
    TPasFunctionType(obj.ProcType).ResultEl.ResultType.Name + '" />');
  Dec(lvl);
  WriteLn(f, ' ': lvl * 2, '</parameters>');
  Dec(lvl);
  WriteLn(f, ' ': lvl * 2, '</function>');
end;

procedure TDocumentation.DoVisit(obj: TPasClassType);
var
  i: integer;
begin
  case obj.ObjKind of
    okObject: WriteLn(f, ' ': lvl * 2, '<object name="' + obj.Name + '">');
    okClass: WriteLn(f, ' ': lvl * 2, '<class name="' + obj.Name + '">');
    okInterface: WriteLn(f, ' ': lvl * 2, '<interface name="' + obj.Name + '">');
  end;

  Inc(lvl);

  if obj.AncestorType <> nil then
    WriteLn(f, ' ': lvl * 2, '<ancestor name="' + obj.AncestorType.GetDeclaration(True) +
      '" namespace="StdCtrls2">')
  else
    WriteLn(f, ' ': lvl * 2, '<ancestor name="TObject" namespace="System">');
  WriteLn(f, ' ': lvl * 2, '</ancestor>');

  if obj.Members.Count > 0 then
  begin
    WriteLn(f, ' ': lvl * 2, '<members>');
    Inc(lvl);
    for i := 0 to obj.Members.Count - 1 do
      TPasProperty(obj.Members[i]).Accept(Self);
    Dec(lvl);
    WriteLn(f, ' ': lvl * 2, '</members>');
  end;

  Dec(lvl);

  case obj.ObjKind of
    okObject: WriteLn(f, ' ': lvl * 2, '</object>');
    okClass: WriteLn(f, ' ': lvl * 2, '</class>');
    okInterface: WriteLn(f, ' ': lvl * 2, '</interface>');
  end;
end;

procedure TDocumentation.DoVisit(obj: TPasElement);
begin
  WriteLn('Warning: NOT supported: ' + obj.ClassName + ' (' + obj.Name + ')');
end;

procedure TDocumentation.DoVisit(obj: TPasOverloadedProc);
var
  i: integer;
begin
  for i := 0 to obj.Overloads.Count - 1 do
    TPasProcedure(obj.Overloads[i]).Accept(Self);
end;

procedure TDocumentation.DoVisit(obj: TPasPointerType);
begin
  Write(f, ' ': lvl * 2, '<pointer name="' + obj.Name + '"');
  if obj.DestType <> nil then
    Write(f, ' type="' + obj.DestType.Name + '"');
  WriteLn(f, ' indircnt="1" />');
end;

procedure TDocumentation.DoVisit(obj: TPasArrayType);
begin
  Write(f, ' ': lvl * 2, '<array name="' + obj.Name + '"');
  if obj.IndexRange <> '' then
  begin
    if Pos('..', obj.IndexRange) <> 0 then
    begin
      Write(f, ' low="' + Copy(obj.IndexRange, 1, Pos('..', obj.IndexRange) - 1) + '"');
      Write(f, ' high="' + Copy(obj.IndexRange, Pos('..', obj.IndexRange) + 2,
        MaxInt) + '"');
    end
    else
      Write(f, ' high="' + obj.IndexRange + '"');
  end;
  WriteLn(f, '>');

  WriteLn(f, '    <element type="' + obj.ElType.Name + '" />');
  WriteLn(f, '  </array>');
end;

procedure TDocumentation.DoVisit(obj: TPasProcedureType);
begin
  Write(f, ' ': lvl * 2, '<procedureDef name="' + obj.Name + '"');
  if obj.Visibility <> visDefault then
    Write(f, ' visibility="' + VisibilityNames[obj.Visibility] + '"');
  WriteLn(f, '>');

  if obj.Args.Count > 0 then
  begin
    WriteLn(f, ' ': lvl * 2 + 2, '<parameters>');
    DocParameters(obj);
    WriteLn(f, ' ': lvl * 2 + 2, '</parameters>');
  end;

  WriteLn(f, ' ': lvl * 2, '</procedureDef>');
end;

procedure TDocumentation.DoVisit(obj: TPasFunctionType);
begin
  Write(f, ' ': lvl * 2, '<functionDef name="' + obj.Name + '"');
  if obj.Visibility <> visDefault then
    Write(f, ' visibility="' + VisibilityNames[obj.Visibility] + '"');
  WriteLn(f, '>');
  WriteLn(f, ' ': lvl * 2 + 2, '<parameters>');
  DocParameters(obj);
  WriteLn(f, ' ': lvl * 2 + 4, '<retval type="' + obj.ResultEl.ResultType.Name + '" />');
  WriteLn(f, ' ': lvl * 2 + 2, '</parameters>');
  WriteLn(f, ' ': lvl * 2, '</functionDef>');
end;

procedure TDocumentation.DoVisit(obj: TPasResString);
begin
  WriteLn(f, ' ': lvl * 2, '<resourceString name="' + obj.Name + '">');
  WriteLn(f, ' ': lvl * 2 + 2, '<value>');
  WriteLn(f, ' ': lvl * 2 + 4, EscapeXml(obj.GetDeclaration(false)));
  WriteLn(f, ' ': lvl * 2 + 2, '</value>');
  WriteLn(f, ' ': lvl * 2, '</resourceString>');
end;

{ TXMLWriter }

procedure TDXMLWriter.DoWriteDocumentation;
var
  i: integer;
begin
  if Engine.Output <> '' then
    Engine.Output := IncludeTrailingBackSlash(Engine.Output);

  for i := 0 to Package.Modules.Count - 1 do
  begin
    with TDocumentation.Create do
    begin
      GenerateDoc(Engine.Output + TPasModule(Package.Modules[i]).Name +
        '.xml', TPasModule(Package.Modules[i]));
      Free;
    end;
  end;
end;

initialization
  // Do not localize.
  RegisterWriter(TDXMLWriter, 'dxml', 'fpdoc Delphi XML output.');

finalization
  UnRegisterWriter('dxml');
end.

