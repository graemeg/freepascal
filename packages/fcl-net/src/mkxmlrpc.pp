{

    Automatic XML-RPC wrapper generator
    Copyright (c) 2003 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
program MkXMLRPC;

{$mode objfpc}
{$H+}

uses SysUtils, Classes, PParser, PasTree, PasWrite;

resourcestring
  SCmdLineInvalidOption = 'Ignoring unknown option "%s"';
  SNoServerClassNameProvided =
    'No server class name provided (use --serverclass=<name>)';
  SNoUnitNameProvided =
    'No name for generated unit provided (use --unitname=<name>)';

type
  TParserEngine = class(TPasTreeContainer)
  protected
    Modules, UsedModules: TList;
    CurModule: TPasModule;
  public
    constructor Create;
    destructor Destroy; override;
    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement; AVisibility: TPasMemberVisibility;
      const ASourceFilename: String; ASourceLinenumber: Integer): TPasElement;
      override;
    function FindElement(const AName: String): TPasElement; override;
    function FindModule(const AName: String): TPasModule; override;
  end;

  TServerClass = class
    Element: TPasClassType;
    ImplName: String;
  end;

  TRPCList = class
    constructor Create;
    destructor Destroy; override;
    procedure AddServerClass(const AClassName: String);
    ServerClasses: TList;
    UsedModules: TStringList;
  end;

var
  Engine: TParserEngine;


constructor TParserEngine.Create;
begin
  inherited Create;
  Modules := TList.Create;
  UsedModules := TList.Create;
end;

destructor TParserEngine.Destroy;
begin
  UsedModules.Free;
  Modules.Free;
  inherited Destroy;
end;

function TParserEngine.CreateElement(AClass: TPTreeElement; const AName: String;
  AParent: TPasElement; AVisibility: TPasMemberVisibility;
  const ASourceFilename: String; ASourceLinenumber: Integer): TPasElement;
begin
  Result := AClass.Create(AName, AParent);
  Result.Visibility := AVisibility;
  if AClass.InheritsFrom(TPasModule) then
  begin
    Modules.Add(Result);
    CurModule := TPasModule(Result);
  end;
end;

function TParserEngine.FindElement(const AName: String): TPasElement;

  function FindInModule(AModule: TPasModule; const LocalName: String): TPasElement;
  var
    l: TList;
    i, j: Integer;
    Found: Boolean;
  begin
    l := AModule.InterfaceSection.Declarations;
    for i := 0 to l.Count - 1 do
    begin
      Result := TPasElement(l[i]);
      if CompareText(Result.Name, LocalName) = 0 then
      begin
        Found := False;
        for j := 0 to UsedModules.Count - 1 do
          if CompareText(TPasModule(UsedModules[j]).Name, AModule.Name) = 0 then
          begin
            Found := True;
            break;
          end;
        if not Found then
          UsedModules.Add(AModule);
        exit;
      end;
    end;
    Result := nil;
 end;

var
  i: Integer;
  //ModuleName, LocalName: String;
  Module: TPasElement;
begin
{!!!: Don't know if we ever will have to use the following:
  i := Pos('.', AName);
  if i <> 0 then
  begin
    WriteLn('Dot found in name: ', AName);
    Result := nil;
  end else
  begin}
    Result := FindInModule(CurModule, AName);
    if not Assigned(Result) then
      for i := CurModule.InterfaceSection.UsesList.Count - 1 downto 0 do
      begin
        Module := TPasElement(CurModule.InterfaceSection.UsesList[i]);
        if Module.ClassType = TPasModule then
        begin
          Result := FindInModule(TPasModule(Module), AName);
          if Assigned(Result) then
            exit;
        end;
      end;
  {end;}
end;

function TParserEngine.FindModule(const AName: String): TPasModule;
var
  i: Integer;
begin
  for i := Modules.Count - 1 downto 0 do
  begin
    Result := TPasModule(Modules[i]);
    if CompareText(Result.Name, AName) = 0 then
      exit;
  end;
  Result := nil;
end;


constructor TRPCList.Create;
begin
  ServerClasses := TList.Create;
  UsedModules := TStringList.Create;
end;

destructor TRPCList.Destroy;
var
  i: Integer;
begin
  UsedModules.Free;
  for i := 0 to ServerClasses.Count - 1 do
    TServerClass(ServerClasses[i]).Free;
  ServerClasses.Free;
end;

procedure TRPCList.AddServerClass(const AClassName: String);
var
  Element: TPasClassType;
  ServerClass: TServerClass;
begin
  Element := TPasClassType(Engine.FindElement(AClassName));
  if not Assigned(Element) then
  begin
    WriteLn(StdErr, 'Server class "', AClassName, '" not found!');
    Halt(3);
  end;
  if (not Element.InheritsFrom(TPasClassType)) or
    (Element.ObjKind <> okClass) then
  begin
    WriteLn('"', AClassName, '" is not a class!');
    Halt(4);
  end;
  ServerClass := TServerClass.Create;
  ServerClasses.Add(ServerClass);
  ServerClass.Element := Element;
  ServerClass.ImplName := Copy(Element.Name, 2, Length(Element.Name));
  UsedModules.Add(Element.GetModule.Name);
end;


var
  OutputFilename, UnitName: String;
  RPCList: TRPCList;

procedure WriteClassServerSource(ServerClass: TPasClassType;
  ImplementationSection: TPasSection; Method, ProcImpl: TPasProcedureImpl;
  const MethodPrefix: String; NestingLevel: Integer);

{ Method: Main server method
  ProcImpl: Current procedure (may be identical with Method) }

type
  TConversionInfo = record
    ConverterName, TypecastFunction: String;
    ArgIsParent: Boolean;
  end;

  function MakeStructConverter(AClass: TPasClassType;
    Referrer: TPasProcedureImpl): TPasProcedureImpl; forward;

  function MakeArrayConverter(Member, ArraySizeProp: TPasProperty;
    ProcessProc, Referrer: TPasProcedureImpl): TPasProcedureImpl; forward;

  function FindArraySizeProperty(AArrayProp: TPasProperty): TPasProperty;
  var
    i: Integer;
    Name: String;
  begin
    Name := Copy(AArrayProp.Name, 1, Length(AArrayProp.Name) - 1) + 'Count';
    for i := 0 to TPasClassType(AArrayProp.Parent).Members.Count - 1 do
    begin
      Result := TPasProperty(TPasClassType(AArrayProp.Parent).Members[i]);
      if (Result.ClassType = TPasProperty) and (Result.Visibility = visPublic)
        and (CompareStr(Result.Name, Name) = 0) then
        exit;
    end;

    Name := AArrayProp.Name + 'Count';
    for i := 0 to TPasClassType(AArrayProp.Parent).Members.Count - 1 do
    begin
      Result := TPasProperty(TPasClassType(AArrayProp.Parent).Members[i]);
      if (Result.ClassType = TPasProperty) and (Result.Visibility = visPublic)
        and (CompareStr(Result.Name, Name) = 0) then
        exit;
    end;
    Result := nil;
  end;

  function GetConversionInfo(Element: TPasElement;
    Referrer: TPasProcedureImpl): TConversionInfo;
  var
    s: String;
    ArraySizeProp: TPasProperty;
  begin
    FillChar(Result, SizeOf(Result), 0);
    Result.ArgIsParent := False;

    if Element.ClassType = TPasProperty then
    begin
      ArraySizeProp := FindArraySizeProperty(TPasProperty(Element));
      if Assigned(ArraySizeProp) then
      begin
        Result.ConverterName := MakeArrayConverter(TPasProperty(Element),
          ArraySizeProp, ProcImpl, Referrer).Name;
        Result.ArgIsParent := True;
        exit;
      end else
        Element := TPasProperty(Element).VarType;
    end;

    if Element.ClassType = TPasUnresolvedTypeRef then
    begin
      s := UpperCase(Element.Name);
      if (s = 'BYTE') or (s = 'SHORTINT') or (S = 'SMALLINT') or
        (s = 'INTEGER') or (s = 'LONGINT') or (s = 'CARDINAL') or
        (s = 'INT64') or (s = 'QUADWORD') then
        Result.ConverterName := 'AWriter.CreateIntValue'
      else if (s = 'BOOLEAN') or (s = 'WORDBOOL') or (s = 'LONGBOOL') then
        Result.ConverterName := 'AWriter.CreateBooleanValue'
      else if s = 'STRING' then
        Result.ConverterName := 'AWriter.CreateStringValue'
      else if (s = 'FLOAT') or (s = 'SINGLE') or (s = 'DOUBLE') or
        (s = 'EXTENDED') then
        Result.ConverterName := 'AWriter.CreateDoubleValue'
      else if s = 'TDATETIME' then
        Result.ConverterName := 'AWriter.CreateDateTimeValue';
    end else if Element.ClassType = TPasClassType then
      Result.ConverterName := MakeStructConverter(TPasClassType(Element), Referrer).Name
    else if Element.ClassType = TPasEnumType then
    begin
      Result.ConverterName := 'AWriter.CreateIntValue';
      Result.TypecastFunction := 'Ord';
    end;

    if Length(Result.ConverterName) = 0 then
      raise Exception.Create('Result type not supported: ' + Element.ClassName +
        ' ' + Element.Name);
  end;

  function MakeAccessor(ConversionInfo: TConversionInfo;
    const DataSource, ArrayIndex: String): String;
  begin
    Result := ConversionInfo.ConverterName + '(';
    if ConversionInfo.TypecastFunction <> '' then
      Result := Result + ConversionInfo.TypecastFunction + '(';
    Result := Result + DataSource;
    if ConversionInfo.TypecastFunction <> '' then
      Result := Result + ')';
    if ArrayIndex <> '' then
      Result := Result + '[' + ArrayIndex + ']';
    Result := Result + ')';
  end;

  function GetParseValueFnName(PasType: TPasElement): String;
  var
    s: String;
  begin
    SetLength(Result, 0);
    if PasType.ClassType = TPasArgument then
    begin
      if TPasArgument(PasType).Access = argVar then
        raise Exception.Create('"var" arguments are not allowed');
      PasType := TPasArgument(PasType).ArgType;
    end;

    if PasType.ClassType = TPasUnresolvedTypeRef then
    begin
      s := UpperCase(PasType.Name);
      if (s = 'BYTE') or (s = 'SHORTINT') or (S = 'SMALLINT') or
        (s = 'INTEGER') or (s = 'LONGINT') or (s = 'CARDINAL') or
        (s = 'INT64') or (s = 'QUADWORD') then
        Result := 'Int'
      else if (s = 'BOOLEAN') or (s = 'WORDBOOL') or (s = 'LONGBOOL') then
        Result := 'Boolean'
      else if s = 'STRING' then
        Result := 'String'
      else if (s = 'FLOAT') or (s = 'SINGLE') or (s = 'DOUBLE') or
        (s = 'EXTENDED') then
        Result := 'Double'
      else if s = 'TDATETIME' then
        Result := 'DateTime';
    end;
    if Length(Result) = 0 then
      raise Exception.Create('Argument type not supported: ' +
        PasType.ClassName + ' ' + PasType.Name);
  end;

  function NeedLocalProc(const ProcName: String;
    Referrer: TPasProcedureImpl): TPasProcedureImpl;
  var
    i, j: Integer;
  begin
    for i := 0 to Method.Locals.Count - 1 do
    begin
      Result := TPasProcedureImpl(Method.Locals[i]);
      if Result.Name = ProcName then
      begin
        j := Method.Locals.IndexOf(Referrer);
        if (j >= 0) and (i >= j) then
        begin
          // Move existing converter to the top and exit
          Method.Locals.Delete(i);
          j := Method.Locals.IndexOf(ProcImpl);
          if j < 0 then
            j := 0;
          Method.Locals.Insert(j, Result);
        end;
        exit;
      end;
    end;
    Result := nil;
  end;

  function MakeStructConverter(AClass: TPasClassType;
    Referrer: TPasProcedureImpl): TPasProcedureImpl;
  var
    ConverterName, s: String;
    Commands: TPasImplCommands;
    i: Integer;
    LocalMember: TPasElement;
    ConversionInfo: TConversionInfo;
  begin
    ConverterName := 'Convert' + AClass.Name;
    Result := NeedLocalProc(ConverterName, Referrer);
    if Assigned(Result) then
      exit;

    Result := TPasProcedureImpl.Create(ConverterName, Method);
    i := Method.Locals.IndexOf(Referrer);
    if i < 0 then
      i := 0;
    Method.Locals.Insert(i, Result);
    Result.ProcType := TPasFunctionType.Create('', Result);
    Result.ProcType.CreateArgument('Inst', AClass.Name);
    TPasFunctionType(Result.ProcType).ResultEl :=
      TPasResultElement.Create('', Result);
    TPasFunctionType(Result.ProcType).ResultEl.ResultType :=
      TPasUnresolvedTypeRef.Create('TXMLRPCStruct', Result);

    Result.Body := TPasImplBlock.Create('', Result);
    Commands := Result.Body.AddCommands;
    Commands.Commands.Add('Result := AWriter.CreateStruct');
    for i := 0 to AClass.Members.Count - 1 do
    begin
      LocalMember := TPasElement(AClass.Members[i]);
      if LocalMember.ClassType = TPasProperty then
      begin
        ConversionInfo := GetConversionInfo(LocalMember, Result);
        if ConversionInfo.ArgIsParent then
          s := 'Inst'
        else
          s := 'Inst.' + LocalMember.Name;
        s := 'AWriter.AddStructMember(Result, ''' + LocalMember.Name + ''', ' +
          MakeAccessor(ConversionInfo, s, '') + ')';
        Commands.Commands.Add(s);
      end;
    end;
  end;

  function MakeArrayConverter(Member, ArraySizeProp: TPasProperty;
    ProcessProc, Referrer: TPasProcedureImpl): TPasProcedureImpl;
  var
    i: Integer;
    ConverterName, s: String;
    Commands: TPasImplCommands;
    VarMember: TPasVariable;
    ForLoop: TPasImplForLoop;
    ConversionInfo: TConversionInfo;
  begin
    ConverterName := 'Convert' + Member.Parent.Name + '_' + Member.Name;
    Result := NeedLocalProc(ConverterName, Referrer);
    if Assigned(Result) then
      exit;

    Result := TPasProcedureImpl.Create(ConverterName, Method);
    i := Method.Locals.IndexOf(Referrer);
    if i < 0 then
      i := 0;
    Method.Locals.Insert(i, Result);
    Result.ProcType := TPasFunctionType.Create('', Result);
    Result.ProcType.CreateArgument('Inst', Member.Parent.Name);
    TPasFunctionType(Result.ProcType).ResultEl :=
      TPasResultElement.Create('', Result);
    TPasFunctionType(Result.ProcType).ResultEl.ResultType :=
      TPasUnresolvedTypeRef.Create('TXMLRPCArray', Result);

    Result.Body := TPasImplBlock.Create('', Result);
    Commands := Result.Body.AddCommands;
    Commands.Commands.Add('Result := AWriter.CreateArray');

    VarMember := TPasVariable.Create('i', Result);
    Result.Locals.Add(VarMember);
    VarMember.VarType := TPasUnresolvedTypeRef.Create('Integer', VarMember);

    ForLoop := Result.Body.AddForLoop(TPasVariable.Create('i', Result),
      '0', MethodPrefix + ArraySizeProp.Name + ' - 1');
    ForLoop.Body := TPasImplCommand.Create('', ForLoop);
    ConversionInfo := GetConversionInfo(Member.VarType, Result);
    if ConversionInfo.ArgIsParent then
      s := 'Inst'
    else
      s := 'Inst.' + Member.Name + '[i]';
    s := 'AWriter.AddArrayElement(Result, ' +
      MakeAccessor(ConversionInfo, s, '') + ')';
    TPasImplCommand(ForLoop.Body).Command := s;
  end;

  function CreateDispatcher(VarType: TPasClassType;
    Referrer: TPasProcedureImpl): TPasProcedureImpl;
  var
    DispatcherName: String;
  begin
    DispatcherName := 'Dispatch' + VarType.Name;
    Result := NeedLocalProc(DispatcherName, Referrer);
    if Assigned(Result) then
      exit;

    // Create new dispatcher method
    Result := TPasProcedureImpl.Create(DispatcherName, Method);
    if ProcImpl = Method then
      Method.Locals.Insert(0, Result)
    else
      Method.Locals.Insert(Method.Locals.IndexOf(Referrer), Result);
    Result.ProcType := TPasProcedureType.Create('', Result);
    Result.ProcType.CreateArgument('Inst', VarType.Name);
    Result.ProcType.CreateArgument('Level', 'Integer');
    WriteClassServerSource(VarType,
      ImplementationSection, Method, Result, 'Inst.', NestingLevel + 1);
  end;


var
  IfElse, ParentIfElse: TPasImplIfElse;

  procedure CreateBranch(const MethodName: String);
  begin
    if Assigned(ParentIfElse) then
    begin
      IfElse := TPasImplIfElse.Create('', ParentIfElse);
      ParentIfElse.ElseBranch := IfElse;
    end else
    begin
      IfElse := TPasImplIfElse.Create('', ProcImpl.Body);
      ProcImpl.Body.Elements.Add(IfElse);
    end;
    ParentIfElse := IfElse;
    IfElse.Condition := 's = ''' + UpperCase(MethodName) + '''';
  end;

  procedure ProcessMethodCall(Member: TPasProcedure);

    function MakeProcArgs(Args: TList): String;
    var
      i: Integer;
    begin
      if (not Assigned(Args)) or (Args.Count = 0) then
        Result := ''
      else
      begin
        Result := '(';
        for i := 0 to Args.Count - 1 do
        begin
          if i > 0 then
            Result := Result + ', ';
          Result := Result + 'AParser.GetPrev' + GetParseValueFnName(TPasType(Args[i]));
        end;
        Result := Result + ')';
      end;
    end;

  var
    Commands: TPasImplCommands;
    s: String;
  begin
    CreateBranch(Member.Name);
    Commands := TPasImplCommands.Create('', IfElse);
    IfElse.IfBranch := Commands;

    if TPasProcedure(Member).ProcType.Args.Count > 0 then
      Commands.Commands.Add('AParser.ResetValueCursor');
    if Member.ClassType = TPasProcedure then
    begin
      Commands.Commands.Add(MethodPrefix + Member.Name +
        MakeProcArgs(TPasProcedure(Member).ProcType.Args));
      Commands.Commands.Add('AWriter.WriteResponse(nil)');
    end else
    begin
      // function
      s := MethodPrefix + Member.Name +
        MakeProcArgs(TPasProcedure(Member).ProcType.Args);
      Commands.Commands.Add('AWriter.WriteResponse(' +
        MakeAccessor(GetConversionInfo(TPasFunctionType(TPasFunction(Member).
          ProcType).ResultEl.ResultType, ProcImpl), s, '') + ')');
    end;
  end;

  procedure ProcessProperty(Member: TPasProperty);
  var
    LocalIfElse: TPasImplIfElse;
    IsArray, IsStruct: Boolean;
    s, s2: String;
    Commands: TPasImplCommands;
    Command: TPasImplCommand;
    ConversionInfo: TConversionInfo;
  begin
    if Member.ReadAccessorName <> '' then
    begin
      CreateBranch('Get' + Member.Name);

      IsArray := (Member.Args.Count = 1) and
        Assigned(FindArraySizeProperty(Member));
      IsStruct := Member.VarType.ClassType = TPasClassType;

      if IsStruct then
        s := CreateDispatcher(TPasClassType(Member.VarType), ProcImpl).Name +
          '(' + MethodPrefix + Member.Name;

      if NestingLevel = 0 then
        s2 := '1'
      else
        s2 := 'Level + 1';

      if IsArray or (IsStruct and (Member.Args.Count = 0)) then
      begin
        LocalIfElse := TPasImplIfElse.Create('', IfElse);
        IfElse.IfBranch := LocalIfElse;
        LocalIfElse.Condition := 'APath.Count <= ' + s2;
      end;

      if IsStruct then
        if IsArray then
        begin
          LocalIfElse.IfBranch := TPasImplCommand.Create('', LocalIfElse);
          TPasImplCommand(LocalIfElse.IfBranch).Command :=
            'AWriter.WriteResponse(' +
            MakeAccessor(GetConversionInfo(Member, ProcImpl),
              Copy(MethodPrefix, 1, Length(MethodPrefix) - 1), '') + ')';

          LocalIfElse.ElseBranch := TPasImplCommand.Create('', LocalIfElse);
          TPasImplCommand(LocalIfElse.ElseBranch).Command :=
            s + '[AParser.GetNext' +
            GetParseValueFnName(TPasArgument(Member.Args[0]).ArgType) + '], ' +
            s2 + ')';
        end else
        begin
          if Member.Args.Count = 0 then
          begin
            LocalIfElse.IfBranch := TPasImplCommand.Create('', LocalIfElse);
            TPasImplCommand(LocalIfElse.IfBranch).Command :=
               'AWriter.WriteResponse(' +
               MakeAccessor(GetConversionInfo(Member, ProcImpl),
                 MethodPrefix + Member.Name, '') + ')';
            LocalIfElse.ElseBranch := TPasImplCommand.Create('', LocalIfElse);
            TPasImplCommand(LocalIfElse.ElseBranch).Command := s + ', ' + s2 + ')';
          end else
          begin
            IfElse.IfBranch := TPasImplCommand.Create('', IfElse);
            TPasImplCommand(IfElse.IfBranch).Command := s + '[AParser.GetNext' +
            GetParseValueFnName(TPasArgument(Member.Args[0]).ArgType) + '], ' +
            s2 + ')';
          end;
        end
      else if IsArray then
      begin
        LocalIfElse.IfBranch := TPasImplCommand.Create('', LocalIfElse);
        TPasImplCommand(LocalIfElse.IfBranch).Command :=
           'AWriter.WriteResponse(' +
           MakeAccessor(GetConversionInfo(Member, ProcImpl),
             Copy(MethodPrefix, 1, Length(MethodPrefix) - 1), '') + ')';

        LocalIfElse.ElseBranch := TPasImplCommand.Create('', LocalIfElse);
        TPasImplCommand(LocalIfElse.ElseBranch).Command :=
          'AWriter.WriteResponse(' +
          MakeAccessor(GetConversionInfo(Member.VarType, ProcImpl),
            MethodPrefix + Member.Name, 'AParser.GetNext' +
            GetParseValueFnName(TPasArgument(Member.Args[0]).ArgType)) + ')';
      end else
      begin
        IfElse.IfBranch := TPasImplCommand.Create('', IfElse);
        TPasImplCommand(IfElse.IfBranch).Command := 'AWriter.WriteResponse(' +
          MakeAccessor(GetConversionInfo(Member.VarType, ProcImpl),
            MethodPrefix + Member.Name, '') + ')';
      end;
    end;

    if Member.WriteAccessorName <> '' then
    begin
      CreateBranch('Set' + Member.Name);
      Commands := TPasImplCommands.Create('', IfElse);
      IfElse.IfBranch := Commands;
      Commands.Commands.Add('// Not supported by mkxmlrpc yet');
    end;
  end;

var
  VarMember: TPasVariable;
  i: Integer;
  Command: TPasImplCommand;
  Member: TPasElement;
begin
  VarMember := TPasVariable.Create('s', ProcImpl);
  ProcImpl.Locals.Add(VarMember);
  VarMember.VarType := TPasUnresolvedTypeRef.Create('String', VarMember);
  ProcImpl.Body := TPasImplBlock.Create('', ProcImpl);
  if NestingLevel = 0 then
    ProcImpl.Body.AddCommand('s := APath[' + IntToStr(NestingLevel) + ']')
  else
    ProcImpl.Body.AddCommand('s := APath[Level]');
  ParentIfElse := nil;
  for i := 0 to ServerClass.Members.Count - 1 do
  begin
    Member := TPasElement(ServerClass.Members[i]);
    if Member.Visibility <> visPublic then
      continue;

    if (Member.ClassType = TPasProcedure) or (Member.ClassType = TPasFunction)
    then
      ProcessMethodCall(TPasProcedure(Member))
    else if Member.ClassType = TPasProperty then
      ProcessProperty(TPasProperty(Member))
    else if (Member.ClassType <> TPasConstructor) and
      (Member.ClassType <> TPasDestructor) then
      WriteLn('Warning: Unsupportet member type: ', Member.ElementTypeName);
  end;

  if Assigned(ParentIfElse) then
  begin
    Command := TPasImplCommand.Create('', ParentIfElse);
    ParentIfElse.ElseBranch := Command;
  end else
  begin
    Command := TPasImplCommand.Create('', ProcImpl.Body);
    ProcImpl.Body.Elements.Add(Command);
  end;
  Command.Command := 'AWriter.WriteFaultResponse(2, ''Invalid method name'')';
end;

procedure WriteFPCServerSource;
var
  i, j: Integer;
  Module: TPasModule;
  InterfaceSection, ImplementationSection: TPasSection;
  VarMember: TPasVariable;
  PropertyMember: TPasProperty;
  ProcMember: TPasProcedure;
  Arg: TPasArgument;
  ServerClass: TPasClassType;
  Stream: TStream;
  ProcImpl: TPasProcedureImpl;
  Found: Boolean;
begin
  Module := TPasModule.Create(UnitName, nil);
  try
    InterfaceSection := TPasSection.Create('', Module);
    Module.InterfaceSection := InterfaceSection;
    ImplementationSection := TPasSection.Create('', Module);
    Module.ImplementationSection := ImplementationSection;
    InterfaceSection.AddUnitToUsesList('Classes');
    InterfaceSection.AddUnitToUsesList('XMLRPC');
    for i := 0 to RPCList.UsedModules.Count - 1 do
      InterfaceSection.AddUnitToUsesList(RPCList.UsedModules[i]);

    for i := 0 to RPCList.ServerClasses.Count - 1 do
      with TServerClass(RPCList.ServerClasses[i]) do
      begin
        ServerClass := TPasClassType.Create('T' + ImplName + 'XMLRPCServlet',
          InterfaceSection);
        InterfaceSection.Declarations.Add(ServerClass);
        ServerClass.ObjKind := okClass;
        ServerClass.AncestorType :=
          TPasUnresolvedTypeRef.Create('TXMLRPCServlet', ServerClass);

        // Create private field which holds the implementation instance
        VarMember := TPasVariable.Create('F' + ImplName, ServerClass);
        VarMember.Visibility := visPrivate;
        VarMember.VarType := TPasUnresolvedTypeRef.Create(Element.Name, VarMember);
        ServerClass.Members.Add(VarMember);

        // Create dispatcher method
        ProcMember := TPasProcedure.Create('Dispatch', ServerClass);
        ProcMember.Visibility := visProtected;
        ProcMember.IsOverride := True;
        ProcMember.ProcType := TPasProcedureType.Create('', ProcMember);
        ProcMember.ProcType.CreateArgument('AParser', 'TXMLRPCParser').
          Visibility := visPublic;
        ProcMember.ProcType.CreateArgument('AWriter', 'TXMLRPCWriter').
          Visibility := visPublic;
        ProcMember.ProcType.CreateArgument('APath', 'TStrings').
          Visibility := visPublic;
        ServerClass.Members.Add(ProcMember);

        // Create published property for implementation instance
        PropertyMember := TPasProperty.Create(ImplName, ServerClass);
        PropertyMember.Visibility := visPublished;
        PropertyMember.VarType := VarMember.VarType;
        VarMember.VarType.AddRef;
        PropertyMember.ReadAccessorName := 'F' + ImplName;
        PropertyMember.WriteAccessorName := 'F' + ImplName;
        ServerClass.Members.Add(PropertyMember);

        // Create dispatcher implementation
        ProcImpl := TPasProcedureImpl.Create('Dispatch', ServerClass);
        ImplementationSection.Declarations.Add(ProcImpl);
        ProcImpl.ProcType := ProcMember.ProcType;
        ProcMember.ProcType.AddRef;
        ProcImpl.ProcType.AddRef;
        WriteClassServerSource(Element, ImplementationSection, ProcImpl,
          ProcImpl, ImplName + '.', 0);
      end;

    for i := 0 to Engine.UsedModules.Count - 1 do
    begin
      Found := False;
      for j := 0 to RPCList.UsedModules.Count - 1 do
        if CompareText(RPCList.UsedModules[j],
          TPasModule(Engine.UsedModules[i]).Name) = 0 then
        begin
          Found := True;
          break;
        end;
      if not Found then
        ImplementationSection.AddUnitToUsesList(
          TPasModule(Engine.UsedModules[i]).Name);
    end;

    Stream := THandleStream.Create(StdOutputHandle);
    try
      WritePasFile(Module, Stream);
    finally
      Stream.Free;
    end;

    Stream := TFileStream.Create(OutputFilename, fmCreate);
    try
      WritePasFile(Module, Stream);
    finally
      Stream.Free;
    end;
  finally
    Module.Free;
  end;
end;


var
  i, j: Integer;
  s, Cmd, Arg: String;
  InputFiles, ClassList: TStringList;
begin
  InputFiles := TStringList.Create;
  ClassList := TStringList.Create;
  try
    for i := 1 to ParamCount do
    begin
      s := ParamStr(i);
      j := Pos('=', s);
      if j > 0 then
      begin
        Cmd := Copy(s, 1, j - 1);
        Arg := Copy(s, j + 1, Length(s));
      end else
      begin
        Cmd := s;
        SetLength(Arg, 0);
      end;
      if (Cmd = '-i') or (Cmd = '--input') then
        InputFiles.Add(Arg)
      else if Cmd = '--output' then
        OutputFilename := Arg
      else if Cmd = '--unitname' then
        UnitName := Arg
      else if Cmd = '--serverclass' then
        ClassList.Add(Arg)
      else
        WriteLn(StdErr, Format(SCmdLineInvalidOption, [s]));
    end;

    if ClassList.Count = 0 then
    begin
      WriteLn(StdErr, SNoServerClassNameProvided);
      Halt(2);
    end;

    if UnitName = '' then
    begin
      WriteLn(StdErr, SNoUnitNameProvided);
      Halt(2);
    end;

    Engine := TParserEngine.Create;
    try
      // Engine.SetPackageName('XMLRPC');
      for i := 0 to InputFiles.Count - 1 do
        ParseSource(Engine, InputFiles[i], '', '');

      RPCList := TRPCList.Create;
      try
        for i := 0 to ClassList.Count - 1 do
          RPCList.AddServerClass(ClassList[i]);
        WriteFPCServerSource;
      finally
        RPCList.Free;
      end;
    finally
      Engine.Free;
    end;
  finally
    InputFiles.Free;
    ClassList.Free;
  end;
end.
