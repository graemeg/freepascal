{
  Examples:
    ./testpassrc --suite=TTestResolver.TestEmpty
}
unit tcuseanalyzer;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, PasTree, PScanner, PasResolver, tcbaseparser,
  testregistry, strutils, tcresolver, PasUseAnalyzer, PasResolveEval;

type

  { TCustomTestUseAnalyzer }

  TCustomTestUseAnalyzer = Class(TCustomTestResolver)
  private
    FAnalyzer: TPasAnalyzer;
    FPAMessages: TFPList; // list of TPAMessage
    FPAGoodMessages: TFPList;
    FProcAnalyzer: TPasAnalyzer;
    function GetPAMessages(Index: integer): TPAMessage;
    procedure OnAnalyzerMessage(Sender: TObject; Msg: TPAMessage);
  protected
    procedure SetUp; override;
    procedure TearDown; override;
    procedure AnalyzeModule; virtual;
    procedure AnalyzeProgram; virtual;
    procedure AnalyzeUnit; virtual;
    procedure AnalyzeWholeProgram; virtual;
    procedure CheckUsedMarkers; virtual;
    procedure CheckUseAnalyzerHint(MsgType: TMessageType; MsgNumber: integer;
      const MsgText: string); virtual;
    procedure CheckUseAnalyzerUnexpectedHints; virtual;
    procedure CheckUnitUsed(const aFilename: string; Used: boolean); virtual;
    procedure CheckScopeReferences(const ScopeName: string;
      const RefNames: array of string);
  public
    property Analyzer: TPasAnalyzer read FAnalyzer;
    property ProcAnalyzer: TPasAnalyzer read FProcAnalyzer;
    function PAMessageCount: integer;
    property PAMessages[Index: integer]: TPAMessage read GetPAMessages;
  end;

  { TTestUseAnalyzer }

  TTestUseAnalyzer = Class(TCustomTestUseAnalyzer)
  published
    // single module
    procedure TestM_ProgramLocalVar;
    procedure TestM_AssignStatement;
    procedure TestM_BeginBlock;
    procedure TestM_ForLoopStatement;
    procedure TestM_AsmStatement;
    procedure TestM_CaseOfStatement;
    procedure TestM_IfThenElseStatement;
    procedure TestM_WhileDoStatement;
    procedure TestM_RepeatUntilStatement;
    procedure TestM_TryFinallyStatement;
    procedure TestM_TypeAlias;
    procedure TestM_TypeAliasTypeInfo;
    procedure TestM_RangeType;
    procedure TestM_Unary;
    procedure TestM_Const;
    procedure TestM_ResourceString;
    procedure TestM_Record;
    procedure TestM_RecordGeneric;
    procedure TestM_PointerTyped_Record;
    procedure TestM_Array;
    procedure TestM_NestedFuncResult;
    procedure TestM_Enums;
    procedure TestM_ProcedureType;
    procedure TestM_AnonymousProc;
    procedure TestM_Params;
    procedure TestM_Class;
    procedure TestM_ClassForward;
    procedure TestM_Class_Property;
    procedure TestM_ClassForward_Generic;
    procedure TestM_Class_PropertyProtected;
    procedure TestM_Class_PropertyOverride;
    procedure TestM_Class_PropertyOverride2;
    procedure TestM_Class_PropertyInherited;
    procedure TestM_Class_MethodOverride;
    procedure TestM_Class_MethodOverride2;
    procedure TestM_Class_NestedClass;
    procedure TestM_ClassInterface_Corba;
    procedure TestM_ClassInterface_NoHintsForMethod;
    procedure TestM_ClassInterface_NoHintsForImpl;
    procedure TestM_ClassInterface_Delegation;
    procedure TestM_ClassInterface_COM;
    procedure TestM_TryExceptStatement;

    // single module hints
    procedure TestM_Hint_UnitNotUsed;
    procedure TestM_Hint_UnitNotUsed_No_OnlyExternal;
    procedure TestM_Hint_UnitUsed;
    procedure TestM_Hint_UnitUsedVarArgs;
    procedure TestM_Hint_ParameterNotUsed;
    procedure TestM_Hint_ParameterNotUsedOff;
    procedure TestM_Hint_ParameterInOverrideNotUsed;
    procedure TestM_Hint_ParameterAssignedButNotReadVarParam;
    procedure TestM_Hint_ParameterNotUsed_Abstract;
    procedure TestM_Hint_ParameterNotUsedTypecast;
    procedure TestM_Hint_OutParam_No_AssignedButNeverUsed;
    procedure TestM_Hint_ArgPassed_No_ParameterNotUsed;
    procedure TestM_Hint_ArrayArg_No_ParameterNotUsed;
    procedure TestM_Hint_ArrayArg_No_ParameterNotUsed2;
    procedure TestM_Hint_InheritedWithoutParams;
    procedure TestM_Hint_LocalVariableNotUsed;
    procedure TestM_HintsOff_LocalVariableNotUsed;
    procedure TestM_Hint_ForVar_No_LocalVariableNotUsed;
    procedure TestM_Hint_InterfaceUnitVariableUsed;
    procedure TestM_Hint_ValueParameterIsAssignedButNeverUsed;
    procedure TestM_Hint_LocalVariableIsAssignedButNeverUsed;
    procedure TestM_Hint_PropertyIsAssignedButNeverUsed;
    procedure TestM_Hint_LocalXYNotUsed;
    procedure TestM_Hint_PrivateFieldIsNeverUsed;
    procedure TestM_Hint_PrivateFieldIsAssignedButNeverUsed;
    procedure TestM_Hint_PrivateFieldExtClassNoIsAssignedButNeverUsed;
    procedure TestM_Hint_PrivateMethodIsNeverUsed;
    procedure TestM_Hint_LocalDestructor_No_IsNeverUsed;
    procedure TestM_Hint_PrivateTypeNeverUsed;
    procedure TestM_Hint_PrivateConstNeverUsed;
    procedure TestM_Hint_PrivatePropertyNeverUsed;
    procedure TestM_Hint_LocalClassInProgramNotUsed;
    procedure TestM_Hint_LocalMethodInProgramNotUsed;
    procedure TestM_Hint_LocalVarOfNotUsedProc;
    procedure TestM_Hint_LocalVarOfNotUsedMethod;
    procedure TestM_Hint_AssemblerParameterIgnored;
    procedure TestM_Hint_AssemblerDelphiParameterIgnored;
    procedure TestM_Hint_FunctionResultDoesNotSeemToBeSet;
    procedure TestM_Hint_FunctionResultDoesNotSeemToBeSet_Abstract;
    procedure TestM_Hint_FunctionResultRecord;
    procedure TestM_Hint_FunctionResultRecordEmpty;
    procedure TestM_Hint_FunctionResultPassRecordElement;
    procedure TestM_Hint_FunctionResultAssembler;
    procedure TestM_Hint_FunctionResultExit;
    procedure TestM_Hint_AbsoluteVar;
    procedure TestM_Hint_GenFunctionResultArgNotUsed;
    procedure TestM_Hint_GenFunc_LocalInsideImplUsed;

    // whole program optimization
    procedure TestWP_LocalVar;
    procedure TestWP_UnitUsed;
    procedure TestWP_UnitUsed_ResourceString;
    procedure TestWP_UnitNotUsed;
    procedure TestWP_UnitInitialization;
    procedure TestWP_UnitFinalization;
    procedure TestWP_CallInherited;
    procedure TestWP_ProgramPublicDeclarations;
    procedure TestWP_ClassOverride;
    procedure TestWP_ClassDefaultProperty;
    procedure TestWP_BeforeConstruction;
    procedure TestWP_Published;
    procedure TestWP_PublishedSetType;
    procedure TestWP_PublishedArrayType;
    procedure TestWP_PublishedClassOfType;
    procedure TestWP_PublishedRecordType;
    procedure TestWP_PublishedProcType;
    procedure TestWP_PublishedProperty;
    procedure TestWP_BuiltInFunctions;
    procedure TestWP_TypeInfo;
    procedure TestWP_TypeInfo_PropertyEnumType;
    procedure TestWP_TypeInfo_Alias;
    procedure TestWP_TypeInfo_Specialize;
    procedure TestWP_ForInClass;
    procedure TestWP_AssertSysUtils;
    procedure TestWP_RangeErrorSysUtils;
    procedure TestWP_ClassInterface;
    procedure TestWP_ClassInterface_OneWayIntfToObj;
    procedure TestWP_ClassInterface_Delegation;
    procedure TestWP_ClassInterface_COM;
    procedure TestWP_ClassInterface_COM_Unit;
    procedure TestWP_ClassInterface_Typeinfo;
    procedure TestWP_ClassInterface_TGUID;
    procedure TestWP_ClassHelper;
    procedure TestWP_ClassHelper_ClassConstrucor_Used;
    procedure TestWP_Attributes;
    procedure TestWP_Attributes_ForwardClass;
    procedure TestWP_Attributes_Params;
    procedure TestWP_Attributes_PublishedFields; // ToDo

    // scope references
    procedure TestSR_Proc_UnitVar;
    procedure TestSR_Init_UnitVar;
  end;

function dbgs(a: TPSRefAccess) : string;

implementation

function dbgs(a: TPSRefAccess): string;
begin
  str(a,Result);
end;

{ TCustomTestUseAnalyzer }

procedure TCustomTestUseAnalyzer.OnAnalyzerMessage(Sender: TObject;
  Msg: TPAMessage);
begin
  Msg.AddRef;
  FPAMessages.Add(Msg);
end;

function TCustomTestUseAnalyzer.GetPAMessages(Index: integer): TPAMessage;
begin
  Result:=TPAMessage(FPAMessages[Index]);
end;

procedure TCustomTestUseAnalyzer.SetUp;
begin
  inherited SetUp;
  FPAMessages:=TFPList.Create;
  FPAGoodMessages:=TFPList.Create;
  FAnalyzer:=TPasAnalyzer.Create;
  FAnalyzer.Resolver:=ResolverEngine;
  Analyzer.OnMessage:=@OnAnalyzerMessage;
end;

procedure TCustomTestUseAnalyzer.TearDown;
var
  i: Integer;
begin
  FreeAndNil(FPAGoodMessages);
  for i:=0 to FPAMessages.Count-1 do
    TPAMessage(FPAMessages[i]).Release;
  FreeAndNil(FPAMessages);
  FreeAndNil(FAnalyzer);
  FreeAndNil(FProcAnalyzer);
  inherited TearDown;
end;

procedure TCustomTestUseAnalyzer.AnalyzeModule;
begin
  Analyzer.AnalyzeModule(Module);
  Analyzer.EmitModuleHints(Module);
  CheckUsedMarkers;
end;

procedure TCustomTestUseAnalyzer.AnalyzeProgram;
begin
  ParseProgram;
  AnalyzeModule;
end;

procedure TCustomTestUseAnalyzer.AnalyzeUnit;
begin
  ParseUnit;
  AnalyzeModule;
end;

procedure TCustomTestUseAnalyzer.AnalyzeWholeProgram;
begin
  ParseProgram;
  Analyzer.AnalyzeWholeProgram(Module as TPasProgram);
  CheckUsedMarkers;
end;

procedure TCustomTestUseAnalyzer.CheckUsedMarkers;
type
  TUsed = (
    uUsed,
    uNotUsed,
    uTypeInfo,
    uNoTypeinfo
    );
var
  aMarker: PSrcMarker;
  p: SizeInt;
  Postfix: String;
  Elements: TFPList;
  i: Integer;
  El, FoundEl: TPasElement;
  ExpectedUsed: TUsed;
begin
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    writeln('TCustomTestUseAnalyzer.CheckUsedMarkers ',aMarker^.Identifier,' Line=',aMarker^.Row,' StartCol=',aMarker^.StartCol,' EndCol=',aMarker^.EndCol);
    p:=RPos('_',aMarker^.Identifier);
    if p>1 then
      begin
      Postfix:=copy(aMarker^.Identifier,p+1);

      if Postfix='used' then
        ExpectedUsed:=uUsed
      else if Postfix='notused' then
        ExpectedUsed:=uNotUsed
      else if Postfix='typeinfo' then
        ExpectedUsed:=uTypeInfo
      else if Postfix='notypeinfo' then
        ExpectedUsed:=uNoTypeInfo
      else
        RaiseErrorAtSrcMarker('TCustomTestUseAnalyzer.CheckUsedMarkers unknown postfix "'+Postfix+'"',aMarker);

      Elements:=FindElementsAt(aMarker);
      try
        FoundEl:=nil;
        for i:=0 to Elements.Count-1 do
          begin
          El:=TPasElement(Elements[i]);
          writeln('TCustomTestUseAnalyzer.CheckUsedMarkers ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
          case ExpectedUsed of
          uUsed,uNotUsed:
            if Analyzer.IsUsed(El) then
              begin
              FoundEl:=El;
              break;
              end;
          uTypeInfo,uNoTypeinfo:
            if Analyzer.IsTypeInfoUsed(El) then
              begin
              FoundEl:=El;
              break;
              end;
          end;
          end;
        if FoundEl<>nil then
          case ExpectedUsed of
          uNotUsed:
            RaiseErrorAtSrcMarker('expected element to be *not* used, but it is marked',aMarker);
          uNoTypeinfo:
            RaiseErrorAtSrcMarker('expected element to have *no* typeinfo, but it is marked',aMarker);
          end
        else
          case ExpectedUsed of
          uUsed:
            RaiseErrorAtSrcMarker('expected element to be used, but it is not marked',aMarker);
          uTypeInfo:
            RaiseErrorAtSrcMarker('expected element to have typeinfo, but it is not marked',aMarker);
          end;
      finally
        Elements.Free;
      end;
      end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TCustomTestUseAnalyzer.CheckUseAnalyzerHint(MsgType: TMessageType;
  MsgNumber: integer; const MsgText: string);
var
  i: Integer;
  Msg: TPAMessage;
  s: string;
begin
  i:=PAMessageCount-1;
  while i>=0 do
    begin
    Msg:=PAMessages[i];
    if (Msg.MsgNumber=MsgNumber) then
      begin
      if (Msg.MsgType=MsgType) and (Msg.MsgText=MsgText) then
        begin
        FPAGoodMessages.Add(Msg);
        exit;
        end;
      end;
    dec(i);
    end;
  // mismatch
  writeln('TCustomTestUseAnalyzer.CheckHasHint: ');
  for i:=0 to PAMessageCount-1 do
    begin
    Msg:=PAMessages[i];
    writeln('  ',i,'/',PAMessageCount,': [',Msg.Id,'] ',Msg.MsgType,': (',Msg.MsgNumber,') {',Msg.MsgText,'}');
    end;
  s:='';
  str(MsgType,s);
  Fail('Analyzer Message not found: '+s+': ('+IntToStr(MsgNumber)+') {'+MsgText+'}');
end;

procedure TCustomTestUseAnalyzer.CheckUseAnalyzerUnexpectedHints;
var
  i: Integer;
  Msg: TPAMessage;
  s: String;
begin
  for i:=0 to PAMessageCount-1 do
    begin
    Msg:=PAMessages[i];
    if FPAGoodMessages.IndexOf(Msg)>=0 then continue;
    s:='';
    str(Msg.MsgType,s);
    Fail('Unexpected analyzer message found ['+IntToStr(Msg.Id)+'] '+s+': ('+IntToStr(Msg.MsgNumber)+') {'+Msg.MsgText+'}');
    end;
end;

procedure TCustomTestUseAnalyzer.CheckUnitUsed(const aFilename: string;
  Used: boolean);
var
  aResolver: TTestEnginePasResolver;
  PAEl: TPAElement;
begin
  aResolver:=FindModuleWithFilename(aFilename);
  AssertNotNull('unit not found "'+aFilename+'"',aResolver);
  AssertNotNull('unit module not found "'+aFilename+'"',aResolver.Module);
  PAEl:=Analyzer.FindElement(aResolver.Module);
  if PAEl<>nil then
    begin
    // unit is used
    if not Used then
      Fail('expected unit "'+aFilename+'" not used, but it is used');
    end
  else
    begin
    // unit is not used
    if Used then
      Fail('expected unit "'+aFilename+'" used, but it is not used');
    end;
end;

procedure TCustomTestUseAnalyzer.CheckScopeReferences(
  const ScopeName: string; const RefNames: array of string);
type
  TEntry = record
    Name: string;
    Access: TPSRefAccess;
  end;

var
  Entries: array of TEntry;

  procedure CheckRefs(ScopeRefs: TPasScopeReferences; const Prefix: string);

    procedure DumpRefsAndFail(Refs: TFPList; const Msg: string);
    var
      i: Integer;
      Ref: TPasScopeReference;
    begin
      {$IFDEF VerbosePasAnalyzer}
      if Refs.Count=0 then
        writeln('DumpRefsAndFail ',Prefix,' NO REFS');
      {$ENDIF}
      for i:=0 to Refs.Count-1 do
        begin
        Ref:=TPasScopeReference(Refs[i]);
        if Ref=nil then break;
        {$IFDEF VerbosePasAnalyzer}
        writeln('DumpRefsAndFail ',Prefix,' ',i,' ',GetObjName(Ref.Element),' ',Ref.Access);
        {$ENDIF}
        end;
      Fail(Prefix+': '+Msg);
    end;

  var
    Refs: TFPList;
    j, i: Integer;
    o: TObject;
    Ref: TPasScopeReference;
  begin
    if ScopeRefs=nil then
      Refs:=TFPList.Create
    else
      Refs:=ScopeRefs.GetList;
    try
      // check that Refs only contains TPasProcScopeReference
      for i:=0 to Refs.Count-1 do
        begin
        o:=TObject(Refs[i]);
        if not (o is TPasScopeReference) then
          Fail(Prefix+': Refs['+IntToStr(i)+'] '+GetObjName(o));
        end;
      // check that all Entries are referenced
      for i:=0 to length(Entries)-1 do
        begin
        j:=Refs.Count-1;
        while (j>=0)
            and (CompareText(Entries[i].Name,TPasScopeReference(Refs[j]).Element.Name)<>0) do
          dec(j);
        if j<0 then
          DumpRefsAndFail(Refs,'Missing reference "'+Entries[i].Name+'"');
        Ref:=TPasScopeReference(Refs[j]);
        if (Entries[i].Access<>psraNone) and (Ref.Access<>Entries[i].Access) then
          DumpRefsAndFail(Refs,'Wrong reference access "'+Entries[i].Name+'",'
            +' expected '+dbgs(Entries[i].Access)+', but got '+dbgs(Ref.Access));
        end;
      // check that no other references are in Refs
      for i:=0 to Refs.Count-1 do
        begin
        Ref:=TPasScopeReference(Refs[i]);
        j:=length(Entries)-1;
        while (j>=0)
            and (CompareText(Ref.Element.Name,Entries[j].Name)<>0) do
          dec(j);
        if j<0 then
          DumpRefsAndFail(Refs,'Unneeded reference "'+GetObjName(Ref.Element)+'"');
        end;
    finally
      Refs.Free;
    end;
  end;

  function FindProc(Section: TPasSection): boolean;
  var
    i: Integer;
    El: TPasElement;
    Proc: TPasProcedure;
    Scope: TPasProcedureScope;
  begin
    for i:=0 to Section.Declarations.Count-1 do
      begin
      El:=TPasElement(Section.Declarations[i]);
      if CompareText(El.Name,ScopeName)<>0 then continue;
      if not (El is TPasProcedure) then
        Fail('El is not proc '+GetObjName(El));
      Proc:=TPasProcedure(El);
      Scope:=Proc.CustomData as TPasProcedureScope;
      if Scope.DeclarationProc<>nil then continue;

      // check references created by AnalyzeModule
      CheckRefs(Scope.References,'AnalyzeModule');

      exit(true);
      end;
    Result:=false;
  end;

  procedure CheckInitialFinalization(El: TPasImplBlock);
  var
    Scope: TPasInitialFinalizationScope;
  begin
    Scope:=El.CustomData as TPasInitialFinalizationScope;
    CheckRefs(Scope.References,'AnalyzeModule');
  end;

var
  i: Integer;
begin
  Entries:=nil;
  SetLength(Entries,High(RefNames)-low(RefNames)+1);
  for i:=low(RefNames) to high(RefNames) do
    begin
    Entries[i].Name:=RefNames[i];
    Entries[i].Access:=psraNone;
    end;

  if Module is TPasProgram then
    begin
    if CompareText(ScopeName,'begin')=0 then
      begin
      // check begin-block references created by AnalyzeModule
      CheckInitialFinalization(Module.InitializationSection);
      exit;
      end
    else if FindProc(TPasProgram(Module).ProgramSection) then
      exit;
    end
  else if Module is TPasLibrary then
    begin
    if CompareText(ScopeName,'begin')=0 then
      begin
      // check begin-block references created by AnalyzeModule
      CheckInitialFinalization(Module.InitializationSection);
      exit;
      end
    else if FindProc(TPasLibrary(Module).LibrarySection) then
      exit;
    end
  else if Module.ClassType=TPasModule then
    begin
    if CompareText(ScopeName,'initialization')=0 then
      begin
      // check initialization references created by AnalyzeModule
      CheckInitialFinalization(Module.InitializationSection);
      exit;
      end
    else if CompareText(ScopeName,'finalization')=0 then
      begin
      // check finalization references created by AnalyzeModule
      CheckInitialFinalization(Module.FinalizationSection);
      exit;
      end
    else if FindProc(Module.InterfaceSection) then
      exit
    else if FindProc(Module.ImplementationSection) then
      exit;
    end;
  Fail('missing proc '+ScopeName);
end;

function TCustomTestUseAnalyzer.PAMessageCount: integer;
begin
  Result:=FPAMessages.Count;
end;

{ TTestUseAnalyzer }

procedure TTestUseAnalyzer.TestM_ProgramLocalVar;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt;');
  Add('var {#l_notused}l: longint;');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_AssignStatement;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt;');
  Add('var');
  Add('  {#a_notused}a: longint;');
  Add('  {#b_used}b: longint;');
  Add('  {#c_used}c: longint;');
  Add('begin');
  Add('  b:=c;');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_BeginBlock;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt;');
  Add('var');
  Add('  {#a_used}a: longint;');
  Add('begin');
  Add('  begin');
  Add('  a:=1;');
  Add('  end;');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_ForLoopStatement;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt;');
  Add('var');
  Add('  {#a_used}a: longint;');
  Add('  {#b_used}b: longint;');
  Add('  {#c_used}c: longint;');
  Add('  {#d_used}d: longint;');
  Add('begin');
  Add('  for a:=b to c do d:=a;');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_AsmStatement;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt;');
  Add('begin');
  Add('  asm end;');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_CaseOfStatement;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt;');
  Add('const');
  Add('  {#a_used}a = 1;');
  Add('  {#b_used}b = 2;');
  Add('var');
  Add('  {#c_used}c: longint;');
  Add('  {#d_used}d: longint;');
  Add('begin');
  Add('  case a of');
  Add('    b: c:=1;');
  Add('  else');
  Add('    d:=2;');
  Add('  end;');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_IfThenElseStatement;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt;');
  Add('var');
  Add('  {#a_used}a: longint;');
  Add('  {#b_used}b: longint;');
  Add('  {#c_used}c: longint;');
  Add('begin');
  Add('  if a=0 then b:=1 else c:=2;');
  Add('  if a=0 then else ;');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_WhileDoStatement;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt;');
  Add('var');
  Add('  {#a_used}a: longint;');
  Add('  {#b_used}b: longint;');
  Add('begin');
  Add('  while a>0 do b:=1;');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_RepeatUntilStatement;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt;');
  Add('var');
  Add('  {#a_used}a: longint;');
  Add('  {#b_used}b: longint;');
  Add('begin');
  Add('  repeat a:=1; until b>1;');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_TryFinallyStatement;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt;');
  Add('var');
  Add('  {#a_used}a: longint;');
  Add('  {#b_used}b: longint;');
  Add('begin');
  Add('  try');
  Add('    a:=1;');
  Add('  finally');
  Add('    b:=2;');
  Add('  end;');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_TypeAlias;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt;');
  Add('type');
  Add('  {#integer_used}integer = longint;');
  Add('var');
  Add('  {#a_used}a: integer;');
  Add('  {#b_used}b: integer;');
  Add('  {#c_notused}c: integer;');
  Add('begin');
  Add('  a:=b;');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_TypeAliasTypeInfo;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  {#integer_typeinfo}integer = type longint;',
  '  {tobject_used}TObject = class',
  '  private',
  '    type {#tcolor_notypeinfo}tcolor = type longint;',
  '  protected',
  '    type {#tsize_typeinfo}tsize = type longint;',
  '  end;',
  'implementation',
  '']);
  AnalyzeUnit;
end;

procedure TTestUseAnalyzer.TestM_RangeType;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt;');
  Add('const');
  Add('  {#neg1_used}Neg1 = -1;');
  Add('  {#pos1_used}Pos1 = +1;');
  Add('type');
  Add('  {#trg_used}TRg = Neg1..Pos1;');
  Add('var');
  Add('  {#a_used}a: trg;');
  Add('begin');
  Add('  a:=0;');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_Unary;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt;');
  Add('var');
  Add('  {#a_used}a: longint;');
  Add('  {#b_used}b: longint;');
  Add('  {#c_used}c: longint;');
  Add('  {#d_used}d: longint;');
  Add('begin');
  Add('  a:=+b;');
  Add('  a:=c+d;');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_Const;
begin
  StartProgram(false);
  Add([
  'procedure {#DoIt_used}DoIt;',
  'var',
  '  {#a_used}a: longint;',
  '  {#b_used}b: boolean;',
  '  {#c_used}c: array of longint;',
  '  {#d_used}d: string;',
  'begin',
  '  a:=+1;',
  '  b:=true;',
  '  c:=nil;',
  '  d:=''foo'';',
  'end;',
  'begin',
  '  DoIt;']);
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_ResourceString;
begin
  StartProgram(false);
  Add([
  'resourcestring',
  '  {#a_used}a = ''txt'';',
  '  {#b_used}b = ''foo'';',
  '  {#c_notused}c = ''bar'';',
  'procedure {#DoIt_used}DoIt(s: string);',
  'var',
  '  {#d_used}d: string;',
  'begin',
  '  d:=b;',
  'end;',
  'begin',
  '  DoIt(a);']);
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_Record;
begin
  StartProgram(false);
  Add([
  'procedure {#DoIt_used}DoIt;',
  'type',
  '  {#integer_used}integer = longint;',
  '  {#trec_used}TRec = record',
  '    {#a_used}a: integer;',
  '    {#b_notused}b: integer;',
  '    {#c_used}c: integer;',
  '  end;',
  'var',
  '  {#r_used}r: TRec;',
  'const',
  '  ci = 2;',
  '  cr: TRec = (a:0;b:ci;c:2);',
  'begin',
  '  r.a:=3;',
  '  with r do c:=4;',
  '  r:=cr;',
  'end;',
  'begin',
  '  DoIt;']);
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_RecordGeneric;
begin
  StartProgram(false);
  Add([
  'procedure {#DoIt_used}DoIt;',
  'type',
  '  {#integer_used}integer = longint;',
  '  {#number_used}number = word;',
  '  generic {#trec_used}TRec<{#trec_t_notused}T> = record',
  '    {#a_used}a: integer;',
  '    {#b_notused}b: integer;',
  '    {#c_used}c: T;',
  '  end;',
  'var',
  '  {#r_used}r: specialize TRec<number>;',
  'const',
  '  ci = 2;',
  '  cr: specialize TRec<number> = (a:0;b:ci;c:2);',
  'begin',
  '  r.a:=3;',
  '  with r do c:=4;',
  '  r:=cr;',
  'end;',
  'begin',
  '  DoIt;']);
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_PointerTyped_Record;
begin
  StartProgram(false);
  Add([
  'procedure {#DoIt_used}DoIt;',
  'type',
  '  {#prec_used}PRec = ^TRec;',
  '  {#trec_used}TRec = record',
  '    {#a_used}a: longint;',
  '    {#b_notused}b: longint;',
  '    {#c_used}c: longint;',
  '    {#d_used}d: longint;',
  '    {#e_used}e: longint;',
  '  end;',
  'var',
  '  r: TRec;',
  '  p: PRec;',
  '  i: longint;',
  'begin',
  '  p:=@r;',
  '  i:=p^.a;',
  '  p^.c:=i;',
  '  if i=p^.d then;',
  '  if p^.e=i then;',
  'end;',
  'begin',
  '  DoIt;']);
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPAFieldNotUsed,'Field "b" not used');
  CheckUseAnalyzerHint(mtHint,nPAFieldIsAssignedButNeverUsed,
    'Field "c" is assigned but never used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Array;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt;');
  Add('type');
  Add('  {#integer_used}integer = longint;');
  Add('  {#tarrayint_used}TArrayInt = array of integer;');
  Add('var');
  Add('  {#a_used}a: TArrayInt;');
  Add('  {#b_used}b: integer;');
  Add('  {#c_used}c: TArrayInt;');
  Add('  {#d_used}d: integer;');
  Add('  {#e_used}e: TArrayInt;');
  Add('  {#f_used}f: integer;');
  Add('  {#g_used}g: TArrayInt;');
  Add('  {#h_used}h: TArrayInt;');
  Add('  {#i_used}i: TArrayInt;');
  Add('begin');
  Add('  a[b]:=c[d];');
  Add('  SetLength(e,f);');
  Add('  if low(g)=high(h)+length(i) then');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_NestedFuncResult;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt;');
  Add('type');
  Add('  {#integer_used}integer = longint;');
  Add('  {#tarrayint_used}TArrayInt = array of integer;');
  Add('  function {#nestedfunc_used}NestedFunc({#b_notused}b: longint): TArrayInt;');
  Add('  begin');
  Add('  end;');
  Add('var');
  Add('  {#d_used}d: longint;');
  Add('begin');
  Add('  NestedFunc(d);');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_Enums;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt(const o);');
  Add('type');
  Add('  {#TEnum_used}TEnum = (red,blue);');
  Add('  {#TEnums_used}TEnums = set of TEnum;');
  Add('var');
  Add('  {#a_used}a: TEnum;');
  Add('  {#b_used}b: TEnums;');
  Add('  {#c_used}c: TEnum;');
  Add('  {#d_used}d: TEnums;');
  Add('  {#e_used}e: TEnums;');
  Add('  {#f_used}f: TEnums;');
  Add('  {#g_used}g: TEnum;');
  Add('  {#h_used}h: TEnum;');
  Add('begin');
  Add('  b:=[a];');
  Add('  if c in d then;');
  Add('  if low(e)=high(f) then;');
  Add('  if pred(g)=succ(h) then;');
  Add('end;');
  Add('var {#s_used}s: string;');
  Add('begin');
  Add('  DoIt(s);');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_ProcedureType;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt;');
  Add('type');
  Add('  {#TProc_used}TProc = procedure;');
  Add('  {#TFunc_used}TFunc = function(): longint;');
  Add('var');
  Add('  {#p_used}p: TProc;');
  Add('  {#f_used}f: TFunc;');
  Add('begin');
  Add('  p:=nil;');
  Add('  f:=nil;');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_AnonymousProc;
begin
  StartProgram(false);
  Add([
  'type',
  '  {#TProc_used}TProc = reference to procedure;',
  'procedure {#DoIt_used}DoIt;',
  'var',
  '  {#p_used}p: TProc;',
  '  {#i_used}i: longint;',
  'begin',
  '  p:=procedure',
  '    begin',
  '      i:=3;',
  '    end;',
  'end;',
  'begin',
  '  DoIt;']);
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_Params;
begin
  StartProgram(false);
  Add('procedure {#DoIt_used}DoIt(const o);');
  Add('type');
  Add('  {#TEnum_used}TEnum = (red,blue);');
  Add('var');
  Add('  {#a_used}a: longint;');
  Add('  {#b_used}b: string;');
  Add('  {#c_used}c: longint;');
  Add('  {#d_used}d: TEnum;');
  Add('begin');
  Add('  DoIt(a);');
  Add('  DoIt(b[c]);');
  Add('  DoIt([d]);');
  Add('  DoIt(red);');
  Add('end;');
  Add('var {#s_used}s: string;');
  Add('begin');
  Add('  DoIt(s);');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_Class;
begin
  StartProgram(false);
  Add('type');
  Add('  {#integer_used}integer = longint;');
  Add('  {tobject_used}TObject = class');
  Add('    {#a_used}a: integer;');
  Add('  end;');
  Add('var Obj: TObject;');
  Add('begin');
  Add('  Obj.a:=3;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_ClassForward;
begin
  StartProgram(false);
  Add('type');
  Add('  {#integer_notused}integer = longint;');
  Add('  {#TObject_used}TObject = class end;');
  Add('  TFelidae = class;');
  Add('  {#TCheetah_used}TCheetah = class');
  Add('  public');
  Add('    {#i_notused}i: integer;');
  Add('    {#f_used}f: TFelidae;');
  Add('  end;');
  Add('  {TFelidae_used}TFelidae = class');
  Add('  end;');
  Add('var {#c_used}c: TCheetah;');
  Add('begin');
  Add('  c.f:=nil;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_Class_Property;
begin
  StartProgram(false);
  Add('type');
  Add('  {#integer_used}integer = longint;');
  Add('  {tobject_used}TObject = class');
  Add('    {#fa_used}Fa: integer;');
  Add('    {#fb_used}Fb: integer;');
  Add('    {#fc_used}Fc: integer;');
  Add('    {#fd_used}Fd: integer;');
  Add('    {#fe_notused}Fe: integer;');
  Add('    function {#getfc_used}GetFC: integer;');
  Add('    procedure {#setfd_used}SetFD({#setfd_value_used}Value: integer);');
  Add('    property {#A_used}A: integer read Fa write Fb;');
  Add('    property {#C_used}C: integer read GetFC write SetFD;');
  Add('  end;');
  Add('function TObject.GetFC: integer;');
  Add('begin');
  Add('  Result:=Fc;');
  Add('end;');
  Add('procedure TObject.SetFD({#setfd_value_impl_notused}Value: integer);');
  Add('begin');
  Add('  Fd:=Value;');
  Add('end;');
  Add('var Obj: TObject;');
  Add('begin');
  Add('  Obj.A:=Obj.A;');
  Add('  Obj.C:=Obj.C;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_ClassForward_Generic;
begin
  StartUnit(false);
  Add([
  '{$mode delphi}',
  'interface',
  'type',
  '  {tobject_used}TObject = class',
  '  end;',
  '  TBird = class;',
  '  TAnt = class end;',
  '  TBird = class end;',
  'implementation',
  'type',
  '  TBird2 = class;',
  '  TAnt2 = class end;',
  '  TBird2 = class end;',
  'var Bird2: TBird2;',
  'begin',
  '  if Bird2=nil then;',
  '']);
  AnalyzeUnit;
end;

procedure TTestUseAnalyzer.TestM_Class_PropertyProtected;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  {#integer_used}integer = longint;',
  '  {tobject_used}TObject = class',
  '  private',
  '    {#fb_used}Fb: integer;',
  '    {#fc_used}Fc: integer;',
  '    {#fd_used}Fd: integer;',
  '    {#fe_notused}Fe: integer;',
  '    function {#iscstored_used}IsCStored: boolean;',
  '  protected',
  '    property {#C_used}C: integer read FC write FD stored IsCStored;',
  '  end;',
  'implementation',
  'function TObject.IsCStored: boolean;',
  'begin',
  '  Result:=Fb<>0;',
  'end;']);
  AnalyzeUnit;
end;

procedure TTestUseAnalyzer.TestM_Class_PropertyOverride;
begin
  StartProgram(false);
  Add(['type',
  '  {#integer_used}integer = longint;',
  '  {tobject_used}TObject = class',
  '    {#fa_used}FA: integer;',
  '    {#fb_notused}FB: integer;',
  '    property {#obj_a_notused}A: integer read FA write FB;',
  '  end;',
  '  {tmobile_used}TMobile = class(TObject)',
  '    {#fc_used}FC: integer;',
  '    property {#mob_a_used}A write FC;',
  '  end;',
  'var {#m_used}M: TMobile;',
  'begin',
  '  M.A:=M.A;']);
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_Class_PropertyOverride2;
begin
  StartProgram(false);
  Add(['type',
  '  {#integer_used}integer = longint;',
  '  {tobject_used}TObject = class',
  '    {#fa_used}FA: integer;',
  '    {#fb_used}FB: integer;',
  '    property {#obj_a_used}A: integer read FA write FB;',
  '  end;',
  '  {tmobile_used}TMobile = class(TObject)',
  '    {#fc_notused}FC: integer;',
  '    property {#mob_a_notused}A write FC;',
  '  end;',
  'var',
  '  {#m_used}M: TMobile;',
  '  {#o_used}o: TObject;',
  'begin',
  '  o:=m;',
  '  o.A:=o.A;',
  '']);
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_Class_PropertyInherited;
begin
  StartProgram(false);
  Add(['type',
  '  {tobject_used}TObject = class',
  '    {#fa_used}FA: word;',
  '    {#fb_used}FB: word;',
  '    property {#obj_a_used}A: word write FA;',
  '    property {#obj_b_used}B: word read FB;',
  '  end;',
  '  {tbird_used}TBird = class(TObject)',
  '    {#fc_notused}FC: word;',
  '    {#fd_notused}FD: word;',
  '    procedure {#run_used}Run({#run_value_used}Value: word);',
  '    property {#bird_a_notused}A write FC;',
  '    property {#bird_b_notused}B write FD;',
  '  end;',
  'procedure TBird.Run(Value: word);',
  'begin',
  '  inherited A:=Value;',
  '  Value:=inherited B;',
  'end;',
  'var',
  '  {#b_used}b: TBird;',
  'begin',
  '  b.Run(3);',
  '']);
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_Class_MethodOverride;
begin
  StartProgram(false);
  Add('type');
  Add('  {tobject_used}TObject = class');
  Add('    procedure {#obj_doa_used}DoA; virtual; abstract;');
  Add('    procedure {#obj_dob_notused}DoB; virtual; abstract;');
  Add('  end;');
  Add('  {tmobile_used}TMobile = class(TObject)');
  Add('    constructor {#mob_create_used}Create;');
  Add('    procedure {#mob_doa_used}DoA; override;');
  Add('    procedure {#mob_dob_used}DoB; override;');
  Add('  end;');
  Add('constructor TMobile.Create; begin end;');
  Add('procedure TMobile.DoA; begin end;');
  Add('procedure TMobile.DoB; begin end;');
  Add('var {#o_used}o: TObject;');
  Add('begin');
  Add('  o:=TMobile.Create;'); // use TMobile before o.DoA
  Add('  o.DoA;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_Class_MethodOverride2;
begin
  StartProgram(false);
  Add('type');
  Add('  {#tobject_used}TObject = class');
  Add('    procedure {#obj_doa_used}DoA; virtual; abstract;');
  Add('  end;');
  Add('  {#tmobile_used}TMobile = class(TObject)');
  Add('    constructor {#mob_create_used}Create;');
  Add('    procedure {#mob_doa_used}DoA; override;');
  Add('  end;');
  Add('constructor TMobile.Create; begin end;');
  Add('procedure TMobile.DoA; begin end;');
  Add('var {#o_used}o: TObject;');
  Add('begin');
  Add('  o.DoA;');
  Add('  o:=TMobile.Create;'); // use TMobile after o.DoA
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_Class_NestedClass;
begin
  StartUnit(true,[supTObject]);
  Add([
  'interface',
  'type',
  '  TBird = class',
  '  public type',
  '    TWing = class',
  '    private',
  '      function GetCurrent: TBird;',
  '    public',
  '      function MoveNext: Boolean; reintroduce;',
  '      property Current: TBird read GetCurrent;',
  '    end;',
  '  end;',
  'implementation',
  'function TBird.TWing.GetCurrent: TBird;',
  'begin',
  '  Result:=nil;',
  'end;',
  'function TBird.TWing.MoveNext: Boolean; reintroduce;',
  'begin',
  '  Result:=false;',
  'end;',
  '']);
  AnalyzeUnit;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_ClassInterface_Corba;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  {#iunknown_used}IUnknown = interface',
  '    procedure {#iunknown_run_used}Run;',
  '    procedure {#iunknown_walk_notused}Walk;',
  '  end;',
  '  {#tobject_used}TObject = class',
  '  end;',
  '  {#tbird_used}TBird = class(TObject,IUnknown)',
  '  strict private',
  '    procedure IUnknown.Run = Fly;',
  '    procedure {#tbird_fly_used}Fly; virtual; abstract;',
  '    procedure {#tbird_walk_used}Walk; virtual; abstract;',
  '  end;',
  '  {#teagle_used}TEagle = class(TBird)',
  '  strict private',
  '    procedure {#teagle_fly_used}Fly; override;',
  '    procedure {#teagle_walk_used}Walk; override;',
  '  end;',
  'procedure TEagle.Fly; begin end;',
  'procedure TEagle.Walk; begin end;',
  'var',
  '  e: TEagle;',
  '  i: IUnknown;',
  'begin',
  '  i:=e;',
  '  i.Run;',
  '']);
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_ClassInterface_NoHintsForMethod;
begin
  StartUnit(false);
  Add([
  '{$interfaces corba}',
  'interface',
  'type',
  '  {#iunknown_used}IUnknown = interface',
  '    procedure {#iunknown_run_used}Run(i: longint);',
  '    function {#iunknown_walk_used}Walk: boolean;',
  '  end;',
  'implementation',
  '']);
  AnalyzeUnit;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_ClassInterface_NoHintsForImpl;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    '{$interfaces corba}',
    'type',
    '  IBird = interface',
    '    procedure DoIt;',
    '  end;',
    '']),
    LinesToStr([
    '']));

  StartUnit(true);
  Add([
  '{$interfaces corba}',
  'interface',
  'uses unit2;',
  'type',
  '  {#tobject_used}TObject = class(IBird)',
  '  strict private',
  '    procedure {#tobject_doit_used}DoIt;',
  '  end;',
  'implementation',
  'procedure TObject.DoIt; begin end;',
  '']);
  AnalyzeUnit;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_ClassInterface_Delegation;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  {#iunknown_used}IUnknown = interface',
  '    procedure {#iunknown_run_used}Run;',
  '    procedure {#iunknown_walk_notused}Walk;',
  '  end;',
  '  {#tobject_used}TObject = class',
  '  end;',
  '  {#tbird_used}TBird = class(TObject,IUnknown)',
  '  strict private',
  '    procedure IUnknown.Run = Fly;',
  '    procedure {#tbird_fly_used}Fly;',
  '    procedure {#tbird_walk_used}Walk;',
  '  end;',
  '  {#teagle_used}TEagle = class(TObject,IUnknown)',
  '  strict private',
  '    {#teagle_fbird_used}FBird: TBird;',
  '    property {#teagle_bird_used}Bird: TBird read FBird implements IUnknown;',
  '  end;',
  'procedure TBird.Fly; begin end;',
  'procedure TBird.Walk; begin end;',
  'var',
  '  e: TEagle;',
  '  i: IUnknown;',
  'begin',
  '  i:=e;',
  '  i.Run;',
  '']);
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_ClassInterface_COM;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  {#tguid_used}TGuid = string;',
  '  {#integer_used}integer = longint;',
  '  {#iunknown_used}IUnknown = interface',
  '    function {#iunknown_queryintf_used}QueryInterface(const iid: TGuid; out obj): Integer;',
  '    function {#iunknown_addref_used}_AddRef: Integer;',
  '    function {#iunknown_release_used}_Release: Integer;',
  '    procedure {#iunknown_doit_notused}DoIt;',
  '  end;',
  '  {#tobject_used}TObject = class',
  '  end;',
  '  {#tbird_used}TBird = class(TObject,IUnknown)',
  '  strict private',
  '    function {#tbird_queryintf_used}QueryInterface(const iid: TGuid; out obj): Integer;',
  '    function {#tbird_addref_used}_AddRef: Integer;',
  '    function {#tbird_release_used}_Release: Integer;',
  '    procedure {#tbird_doit_used}DoIt;',
  '  end;',
  '  {#teagle_used}TEagle = class(TBird)',
  '  end;',
  'function TBird.QueryInterface(const iid: TGuid; out obj): Integer;',
  'begin',
  '  if iid='''' then obj:=nil;',
  '  Result:=0;',
  'end;',
  'function TBird._AddRef: Integer; begin Result:=1; end;',
  'function TBird._Release: Integer; begin Result:=2; end;',
  'procedure TBird.DoIt; begin end;',
  'var',
  '  e: TEagle;',
  '  i: IUnknown;',
  'begin',
  '  i:=e;',
  '  if i=nil then ;',
  '']);
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local procedure "DoIt" not used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_TryExceptStatement;
begin
  StartProgram(false);
  Add('type');
  Add('  {tobject_used}TObject = class');
  Add('    constructor Create; external name ''create'';');
  Add('  end;');
  Add('  {texception_used}Exception = class(TObject);');
  Add('  {tdivbyzero_used}EDivByZero = class(Exception);');
  Add('procedure {#DoIt_used}DoIt;');
  Add('var');
  Add('  {#a_used}a: Exception;');
  Add('  {#b_used}b: Exception;');
  Add('  {#c_used}c: Exception;');
  Add('  {#d_used}d: Exception;');
  Add('  {#f_used}f: Exception;');
  Add('begin');
  Add('  try');
  Add('    a:=nil;');
  Add('  except');
  Add('    raise b;');
  Add('  end;');
  Add('  try');
  Add('    if Assigned(c) then ;');
  Add('  except');
  Add('    on {#e1_used}E1: Exception do raise;');
  Add('    on {#e2_notused}E2: EDivByZero do raise d;');
  Add('    else f:=nil;');
  Add('  end;');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
end;

procedure TTestUseAnalyzer.TestM_Hint_UnitNotUsed;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'var i: longint;',
    'procedure DoIt;',
    '']),
    LinesToStr([
    'procedure DoIt; begin end;']));

  StartProgram(true);
  Add('uses unit2;');
  Add('begin');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPAUnitNotUsed,'Unit "unit2" not used in afile');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_UnitNotUsed_No_OnlyExternal;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'var State: longint; external name ''state'';',
    'procedure DoIt; external name ''doing'';',
    '']),
    LinesToStr([
    ]));

  StartProgram(true);
  Add('uses unit2;');
  Add('begin');
  Add('  State:=3;');
  Add('  DoIt;');
  AnalyzeProgram;

  // unit hints: no hint, even though no code is actually used
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_UnitUsed;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'var i: longint;',
    '']),
    LinesToStr(['']));

  StartProgram(true);
  Add('uses unit2;');
  Add('begin');
  Add('  i:=3;');
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_UnitUsedVarArgs;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'var i: longint;',
    '']),
    LinesToStr(['']));

  StartProgram(true);
  Add('uses unit2;');
  Add('procedure Writeln(); varargs;');
  Add('begin end;');
  Add('begin');
  Add('  writeln(i);');
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_ParameterNotUsed;
begin
  StartProgram(true);
  Add('procedure DoIt(i: longint);');
  Add('begin end;');
  Add('begin');
  Add('  DoIt(1);');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPAParameterNotUsed,'Parameter "i" not used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_ParameterNotUsedOff;
begin
  StartProgram(true);
  Add('{$warn '+IntToStr(nPAParameterNotUsed)+' off}');
  Add('procedure DoIt(i: longint);');
  Add('begin end;');
  Add('begin');
  Add('  DoIt(1);');
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_ParameterInOverrideNotUsed;
begin
  StartProgram(true);
  Add([
  'type',
  '  TObject = class',
  '    procedure DoIt(i: longint); virtual;',
  '  end;',
  '  TBird = class',
  '    procedure DoIt(j: longint); override;',
  '  end;',
  'procedure TObject.DoIt(i: longint);',
  'begin',
  'end;',
  'procedure TBird.DoIt(j: longint);',
  'begin',
  'end;',
  'var b: TBird;',
  'begin',
  '  TObject(b).DoIt(1);']);
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPAParameterInOverrideNotUsed,'Parameter "i" not used');
  CheckUseAnalyzerHint(mtHint,nPAParameterInOverrideNotUsed,'Parameter "j" not used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_ParameterAssignedButNotReadVarParam;
begin
  StartUnit(false);
  Add([
  'interface',
  'procedure DoIt(i: longint);',
  'implementation',
  'procedure DoIt(i: longint);',
  'begin',
  '{$Hints off}',
  'end;',
  'begin',
  '  DoIt(3);']);
  AnalyzeUnit;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_ParameterNotUsed_Abstract;
begin
  StartProgram(true);
  Add('type');
  Add('  TObject = class');
  Add('    class procedure DoIt(i: longint); virtual; abstract;');
  Add('  end;');
  Add('begin');
  Add('  TObject.DoIt(3);');
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_ParameterNotUsedTypecast;
begin
  StartProgram(true);
  Add('type');
  Add('  TObject = class end;');
  Add('  TSortCompare = function(a,b: Pointer): integer;');
  Add('  TObjCompare = function(a,b: TObject): integer;');
  Add('procedure Sort(const Compare: TSortCompare);');
  Add('begin');
  Add('  Compare(nil,nil);');
  Add('end;');
  Add('procedure DoIt(const Compare: TObjCompare);');
  Add('begin');
  Add('  Sort(TSortCompare(Compare));');
  Add('end;');
  Add('begin');
  Add('  DoIt(nil);');
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_OutParam_No_AssignedButNeverUsed;
begin
  StartProgram(true);
  Add('procedure DoIt(out x: longint);');
  Add('begin');
  Add('  x:=3;');
  Add('end;');
  Add('var i: longint;');
  Add('begin');
  Add('  DoIt(i);');
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_ArgPassed_No_ParameterNotUsed;
begin
  StartProgram(false);
  Add([
  'procedure AssertTrue(b: boolean);',
  'begin',
  '  if b then ;',
  'end;',
  'procedure AssertFalse(b: boolean);',
  'begin',
  '  AssertTrue(not b);',
  'end;',
  'begin',
  '  AssertFalse(true);',
  '']);
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_ArrayArg_No_ParameterNotUsed;
begin
  StartProgram(false);
  Add([
  'type TArr = array of boolean;',
  'procedure Fly(a: TArr);',
  'begin',
  '  a[1]:=true;',
  'end;',
  'begin',
  '  Fly(nil);',
  '']);
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_ArrayArg_No_ParameterNotUsed2;
begin
  StartProgram(false);
  Add([
  'type {#Tarr_used}TArr = array of boolean;',
  'procedure {#Run_used}Run({#b_used}b: boolean);',
  'begin',
  '  if b then ;',
  'end;',
  'procedure {#Fly_used}Fly({#a_used}a: TArr);',
  'begin',
  '  Run(a[1]);',
  'end;',
  'begin',
  '  Fly(nil);',
  '']);
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_InheritedWithoutParams;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    constructor Create(i: longint); virtual;',
  '  end;',
  '  TBird = class',
  '    constructor Create(i: longint); override;',
  '  end;',
  'constructor TObject.Create(i: longint);',
  'begin',
  '  if i=0 then ;',
  'end;',
  'constructor TBird.Create(i: longint);',
  'begin',
  '  inherited;',
  'end;',
  'begin',
  '  TBird.Create(3);']);
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_LocalVariableNotUsed;
begin
  StartProgram(true);
  Add([
  'procedure DoIt;',
  'const',
  '  a = 13;',
  '  b: longint = 14;',
  'var',
  '  c: char;',
  '  d: longint = 15;',
  'begin',
  'end;',
  'begin',
  '  DoIt;']);
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local constant "a" not used');
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local constant "b" not used');
  CheckUseAnalyzerHint(mtHint,nPALocalVariableNotUsed,'Local variable "c" not used');
  CheckUseAnalyzerHint(mtHint,nPALocalVariableNotUsed,'Local variable "d" not used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_HintsOff_LocalVariableNotUsed;
begin
  StartProgram(true);
  Add([
  'procedure DoIt;',
  'const',
  '  a = 13;',
  '  b: longint = 14;',
  'var',
  '  c: char;',
  '  d: longint = 15;',
  'begin',
  '{$Hints off}',
  'end;',
  'begin',
  '  DoIt;']);
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_ForVar_No_LocalVariableNotUsed;
begin
  StartProgram(false);
  Add([
  'procedure DoIt;',
  'var i: longint;',
  'begin',
  '  for i:=1 to 2 do ;',
  'end;',
  'begin',
  '  DoIt;',
  '']);
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_InterfaceUnitVariableUsed;
begin
  StartUnit(true);
  Add('interface');
  Add('const {#a_used}a = 1;');
  Add('const {#b_used}b: longint = 2;');
  Add('var {#c_used}c: longint = 3;');
  Add('type');
  Add('  {#TColor_used}TColor = longint;');
  Add('  {#TFlag_used}TFlag = (red,green);');
  Add('  {#TFlags_used}TFlags = set of TFlag;');
  Add('  {#TArrInt_used}TArrInt = array of integer;');
  Add('implementation');
  Add('const {#d_notused}d = 1;');
  Add('const {#e_notused}e: longint = 2;');
  Add('var {#f_notused}f: longint = 3;');
  Add('type');
  Add('  {#ImpTColor_notused}ImpTColor = longint;');
  Add('  {#ImpTFlag_notused}ImpTFlag = (red,green);');
  Add('  {#ImpTFlags_notused}ImpTFlags = set of TFlag;');
  Add('  {#ImpTArrInt_notused}ImpTArrInt = array of integer;');
  AnalyzeUnit;
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local constant "d" not used');
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local constant "e" not used');
  CheckUseAnalyzerHint(mtHint,nPALocalVariableNotUsed,'Local variable "f" not used');
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local alias type "ImpTColor" not used');
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local enumeration type "ImpTFlag" not used');
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local set type "ImpTFlags" not used');
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local array type "ImpTArrInt" not used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_ValueParameterIsAssignedButNeverUsed;
begin
  StartProgram(true);
  Add('procedure DoIt(i: longint);');
  Add('begin');
  Add('  i:=3;');
  Add('end;');
  Add('begin');
  Add('  DoIt(1);');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPAValueParameterIsAssignedButNeverUsed,
    'Value parameter "i" is assigned but never used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_LocalVariableIsAssignedButNeverUsed;
begin
  StartProgram(true);
  Add('procedure DoIt;');
  Add('const');
  Add('  a: longint = 14;');
  Add('var');
  Add('  b: char;');
  Add('  c: longint = 15;');
  Add('begin');
  Add('  a:=16;');
  Add('  b:=#65;');
  Add('  c:=17;');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPALocalVariableIsAssignedButNeverUsed,
    'Local variable "a" is assigned but never used');
  CheckUseAnalyzerHint(mtHint,nPALocalVariableIsAssignedButNeverUsed,
    'Local variable "b" is assigned but never used');
  CheckUseAnalyzerHint(mtHint,nPALocalVariableIsAssignedButNeverUsed,
    'Local variable "c" is assigned but never used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_PropertyIsAssignedButNeverUsed;
begin
  StartProgram(true);
  Add([
  'type',
  '  TObject = class',
  '  private',
  '    FSize: word;',
  '  public',
  '    property ReadSize: word read FSize;',
  '    property WriteSize: word write FSize;',
  '  end;',
  'var o: TObject;',
  'begin',
  '  o.WriteSize:=o.ReadSize;',
  '']);
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_LocalXYNotUsed;
begin
  StartProgram(true);
  Add('procedure DoIt;');
  Add('type');
  Add('  TColor = longint;');
  Add('  TFlag = (red,green);');
  Add('  TFlags = set of TFlag;');
  Add('  TArrInt = array of integer;');
  Add('  procedure Sub; begin end;');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  DoIt;');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local alias type "TColor" not used');
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local enumeration type "TFlag" not used');
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local set type "TFlags" not used');
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local array type "TArrInt" not used');
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local procedure "Sub" not used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_PrivateFieldIsNeverUsed;
begin
  StartProgram(true,[supTObject]);
  Add('type');
  Add('  TMobile = class');
  Add('  private');
  Add('    a: longint;');
  Add('  end;');
  Add('var m: TMobile;');
  Add('begin');
  Add('  m:=nil;');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPAPrivateFieldIsNeverUsed,
    'Private field "TMobile.a" is never used');
  CheckUseAnalyzerHint(mtHint,nPALocalVariableIsAssignedButNeverUsed,
    'Local variable "m" is assigned but never used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_PrivateFieldIsAssignedButNeverUsed;
begin
  StartProgram(true,[supTObject]);
  Add('type');
  Add('  TMobile = class');
  Add('  private');
  Add('    a: longint;');
  Add('  public');
  Add('    constructor Create;');
  Add('  end;');
  Add('constructor TMobile.Create;');
  Add('begin');
  Add('  a:=3;');
  Add('end;');
  Add('begin');
  Add('  TMobile.Create;');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPAPrivateFieldIsAssignedButNeverUsed,
    'Private field "TMobile.a" is assigned but never used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.
  TestM_Hint_PrivateFieldExtClassNoIsAssignedButNeverUsed;
begin
  StartProgram(false,[]);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TMobile = class external name ''foo''',
  '  private',
  '    FA: longint;',
  '  public',
  '    property A: longint write FA;',
  '  end;',
  'var m: TMobile;',
  'begin',
  '  m.A:=3;',
  '']);
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_PrivateMethodIsNeverUsed;
begin
  StartProgram(true,[supTObject]);
  Add('type');
  Add('  TMobile = class');
  Add('  private');
  Add('    procedure DoSome; external name ''foo'';');
  Add('  public');
  Add('    constructor Create;');
  Add('  end;');
  Add('constructor TMobile.Create;');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  TMobile.Create;');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPAPrivateMethodIsNeverUsed,
    'Private method "TMobile.DoSome" is never used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_LocalDestructor_No_IsNeverUsed;
begin
  StartProgram(true,[supTObject]);
  Add('type');
  Add('  TMobile = class');
  Add('  private');
  Add('  public');
  Add('    constructor Create;');
  Add('    destructor Destroy; override;');
  Add('  end;');
  Add('var DestroyCount: longint = 0;');
  Add('constructor TMobile.Create;');
  Add('begin');
  Add('end;');
  Add('destructor TMobile.Destroy;');
  Add('begin');
  Add('  inc(DestroyCount);');
  Add('  inherited;');
  Add('end;');
  Add('var o: TObject;');
  Add('begin');
  Add('  o:=TMobile.Create;');
  Add('  o.Destroy;');
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_PrivateTypeNeverUsed;
begin
  StartProgram(true,[supTObject]);
  Add('type');
  Add('  TMobile = class');
  Add('  private');
  Add('    type t = longint;');
  Add('  public');
  Add('    constructor Create;');
  Add('  end;');
  Add('constructor TMobile.Create;');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  TMobile.Create;');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPAPrivateTypeXNeverUsed,
    'Private type "TMobile.t" never used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_PrivateConstNeverUsed;
begin
  StartProgram(true,[supTObject]);
  Add('type');
  Add('  TMobile = class');
  Add('  private');
  Add('    const c = 3;');
  Add('  public');
  Add('    constructor Create;');
  Add('  end;');
  Add('constructor TMobile.Create;');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  TMobile.Create;');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPAPrivateConstXNeverUsed,
    'Private const "TMobile.c" never used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_PrivatePropertyNeverUsed;
begin
  StartProgram(true,[supTObject]);
  Add('type');
  Add('  TMobile = class');
  Add('  private');
  Add('    FA: longint;');
  Add('    property A: longint read FA;');
  Add('  public');
  Add('    constructor Create;');
  Add('  end;');
  Add('constructor TMobile.Create;');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  TMobile.Create;');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPAPrivatePropertyXNeverUsed,
    'Private property "TMobile.A" never used');
  CheckUseAnalyzerHint(mtHint,nPAPrivateFieldIsNeverUsed,
    'Private field "TMobile.FA" is never used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_LocalClassInProgramNotUsed;
begin
  StartProgram(true,[supTObject]);
  Add('type');
  Add('  TMobile = class');
  Add('  public');
  Add('    constructor Create;');
  Add('  end;');
  Add('constructor TMobile.Create;');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  m: TMobile;');
  Add('begin');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local class "TMobile" not used');
  CheckUseAnalyzerHint(mtHint,nPALocalVariableNotUsed,'Local variable "m" not used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_LocalMethodInProgramNotUsed;
begin
  StartProgram(true,[supTObject]);
  Add('type');
  Add('  TMobile = class');
  Add('  public');
  Add('    constructor Create;');
  Add('  end;');
  Add('constructor TMobile.Create;');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  m: TMobile;');
  Add('begin');
  Add('  if m=nil then ;');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local constructor "Create" not used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_LocalVarOfNotUsedProc;
begin
  StartProgram(true,[]);
  Add('type');
  Add('procedure DoIt;');
  Add('var i: longint;');
  Add('begin');
  Add('end;');
  Add('begin');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPALocalXYNotUsed,'Local procedure "DoIt" not used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_LocalVarOfNotUsedMethod;
begin
  StartProgram(true,[supTObject]);
  Add('type');
  Add('  TMobile = class');
  Add('  private');
  Add('    procedure DoIt;');
  Add('  end;');
  Add('procedure TMobile.DoIt;');
  Add('var i: longint;');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  m: TMobile;');
  Add('begin');
  Add('  if m=nil then ;');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPAPrivateMethodIsNeverUsed,'Private method "TMobile.DoIt" is never used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_AssemblerParameterIgnored;
begin
  StartProgram(true);
  Add('procedure DoIt(i: longint); assembler;');
  Add('type');
  Add('  {#tcolor_notused}TColor = longint;');
  Add('  {#tflag_notused}TFlag = (red,green);');
  Add('  {#tflags_notused}TFlags = set of TFlag;');
  Add('  {#tarrint_notused}TArrInt = array of integer;');
  Add('const');
  Add('  {#a_notused}a = 13;');
  Add('  {#b_notused}b: longint = 14;');
  Add('var');
  Add('  {#c_notused}c: char;');
  Add('  {#d_notused}d: longint = 15;');
  Add('  procedure {#sub_notused}Sub; begin end;');
  Add('asm end;');
  Add('begin');
  Add('  DoIt(1);');
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_AssemblerDelphiParameterIgnored;
begin
  StartProgram(true);
  Add([
  '{$mode Delphi}',
  'procedure DoIt(i: longint);',
  'type',
  '  {#tcolor_notused}TColor = longint;',
  '  {#tflag_notused}TFlag = (red,green);',
  '  {#tflags_notused}TFlags = set of TFlag;',
  '  {#tarrint_notused}TArrInt = array of integer;',
  'const',
  '  {#a_notused}a = 13;',
  '  {#b_notused}b: longint = 14;',
  'var',
  '  {#c_notused}c: char;',
  '  {#d_notused}d: longint = 15;',
  '  procedure {#sub_notused}Sub; begin end;',
  'asm end;',
  'begin',
  '  DoIt(1);',
  '']);
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_FunctionResultDoesNotSeemToBeSet;
begin
  StartProgram(true);
  Add('function DoIt: longint;');
  Add('begin end;');
  Add('begin');
  Add('  DoIt();');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPAFunctionResultDoesNotSeemToBeSet,
    sPAFunctionResultDoesNotSeemToBeSet);
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_FunctionResultDoesNotSeemToBeSet_Abstract;
begin
  StartProgram(true);
  Add('type');
  Add('  TObject = class');
  Add('    class function DoIt: longint; virtual; abstract;');
  Add('  end;');
  Add('begin');
  Add('  TObject.DoIt;');
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_FunctionResultRecord;
begin
  StartProgram(true);
  Add('type');
  Add('  TPoint = record X,Y:longint; end;');
  Add('function Point(Left: longint): TPoint;');
  Add('begin');
  Add('  Result.X:=Left;');
  Add('end;');
  Add('begin');
  Add('  Point(1);');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPAFieldIsAssignedButNeverUsed,
    'Field "X" is assigned but never used');
  CheckUseAnalyzerHint(mtHint,nPAFieldNotUsed,'Field "Y" not used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_FunctionResultRecordEmpty;
begin
  StartProgram(true);
  Add([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  TEmpty = record',
  '    class function Create: TEmpty; static;',
  '  end;',
  'class function TEmpty.Create: TEmpty;',
  'begin',
  'end;',
  'begin',
  '  TEmpty.Create;',
  '']);
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_FunctionResultPassRecordElement;
begin
  StartProgram(true);
  Add('type');
  Add('  TPoint = record X,Y:longint; end;');
  Add('procedure Three(out x: longint);');
  Add('begin');
  Add('  x:=3;');
  Add('end;');
  Add('function Point(): TPoint;');
  Add('begin');
  Add('  Three(Result.X)');
  Add('end;');
  Add('begin');
  Add('  Point();');
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPAFieldNotUsed,'Field "Y" not used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_FunctionResultAssembler;
begin
  StartProgram(false);
  Add([
  'function GetIt: longint; assembler;',
  'asm',
  'end;',
  'begin',
  '  GetIt;']);
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_FunctionResultExit;
begin
  StartProgram(false);
  Add([
  'function GetIt: longint;',
  'begin',
  '  exit(3);',
  'end;',
  'begin',
  '  GetIt;']);
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_AbsoluteVar;
begin
  StartProgram(false);
  Add([
  'procedure {#DoIt_used}DoIt({#p_used}p: pointer);',
  'var',
  '  {#i_used}i: longint absolute p;',
  '  {#j_used}j: longint absolute i;',
  'begin',
  '  if j=3 then ;',
  'end;',
  'begin',
  '  DoIt(nil);']);
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_GenFunctionResultArgNotUsed;
begin
  StartProgram(true);
  Add([
  'type',
  '  generic TPoint<U> = record X,Y: U; end;',
  'generic procedure Three<S>(out x: S);',
  'begin',
  '  x:=3;',
  'end;',
  'generic function Point<T>(): specialize TPoint<T>;',
  'begin',
  '  specialize Three<T>(Result.X)',
  'end;',
  'begin',
  '  specialize Point<word>();',
  '']);
  AnalyzeProgram;
  CheckUseAnalyzerHint(mtHint,nPAFieldNotUsed,'Field "Y" not used');
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestM_Hint_GenFunc_LocalInsideImplUsed;
begin
  StartProgram(true,[supTObject]);
  Add([
  '{$mode delphi}',
  'procedure Run<T>;',
  'var',
  '  WhileV: T;',
  '  RepeatV: T;',
  '  ForR, ForV: T;',
  '  IfCond: boolean;',
  '  IfThenV,IfElseV: T;',
  '  CaseV, CaseSt, CaseElse: T;',
  '  TryFinallyV, TryFinallyX: T;',
  '  TryExceptV, TryExceptOn, TryExceptElse: T;',
  '  WithExpr: TObject;',
  '  WithV: T;',
  'begin',
  '  while true do WhileV:=WhileV+1;',
  '  repeat RepeatV:=RepeatV+1; until false;',
  '  for ForR:=1 to 3 do ForV:=ForV+1;',
  '  if IfCond then IfThenV:=IfThenV+1 else IfElseV:=IfElseV+1;',
  '  case CaseV of',
  '  1: CaseSt:=CaseSt+1;',
  '  else',
  '    CaseElse:=CaseElse+1;',
  '  end;',
  '  try TryFinallyV:=TryFinallyV+1; finally TryFinallyX:=TryFinallyX+1; end;',
  '  try',
  '    TryExceptV:=TryExceptV+1;',
  '  except',
  '  on TryExceptE: TObject do TryExceptOn:=TryExceptOn+1;',
  '  else',
  '    TryExceptElse:=TryExceptElse+1;',
  '  end;',
  '  with WithExpr do WithV:=WithV+1',
  'end;',
  'begin',
  '  Run<word>();']);
  AnalyzeProgram;
  CheckUseAnalyzerUnexpectedHints;
end;

procedure TTestUseAnalyzer.TestWP_LocalVar;
begin
  StartProgram(false);
  Add('var {#a_notused}a: longint;');
  Add('var {#b_used}b: longint;');
  Add('var {#c_used}c: longint;');
  Add('begin');
  Add('  b:=2;');
  Add('  afile.c:=3;');
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_UnitUsed;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'var i: longint;',
    'procedure DoIt;',
    '']),
    LinesToStr([
    'procedure DoIt; begin end;']));

  StartProgram(true);
  Add('uses unit2;');
  Add('begin');
  Add('  i:=3;');
  AnalyzeWholeProgram;

  CheckUnitUsed('unit2.pp',true);
end;

procedure TTestUseAnalyzer.TestWP_UnitUsed_ResourceString;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'resourcestring rs = ''txt'';',
    'procedure DoIt;',
    '']),
    LinesToStr([
    'procedure DoIt; begin end;']));

  StartProgram(true);
  Add('uses unit2;');
  Add('begin');
  Add('  if rs='''' then ;');
  AnalyzeWholeProgram;

  CheckUnitUsed('unit2.pp',true);
end;

procedure TTestUseAnalyzer.TestWP_UnitNotUsed;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'var i: longint;',
    'procedure DoIt;',
    '']),
    LinesToStr([
    'procedure DoIt; begin end;']));

  StartProgram(true);
  Add('uses');
  Add('  unit2;');
  Add('begin');
  AnalyzeWholeProgram;

  CheckUnitUsed('unit2.pp',false);
end;

procedure TTestUseAnalyzer.TestWP_UnitInitialization;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'var i: longint;',
    '']),
    LinesToStr([
    '']));

  AddModuleWithIntfImplSrc('unit1.pp',
    LinesToStr([
    'uses unit2;',
    '']),
    LinesToStr([
    'initialization',
    'i:=2;']));

  StartProgram(true);
  Add('uses unit1;');
  Add('begin');
  AnalyzeWholeProgram;

  CheckUnitUsed('unit1.pp',true);
  CheckUnitUsed('unit2.pp',true);
end;

procedure TTestUseAnalyzer.TestWP_UnitFinalization;
begin
  AddModuleWithIntfImplSrc('unit1.pp',
    LinesToStr([
    'uses unit2;',
    '']),
    LinesToStr([
    'finalization',
    'i:=2;']));

  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'var i: longint;',
    '']),
    LinesToStr([
    '']));

  StartProgram(true);
  Add('uses unit1;');
  Add('begin');
  AnalyzeWholeProgram;

  CheckUnitUsed('unit1.pp',true);
  CheckUnitUsed('unit2.pp',true);
end;

procedure TTestUseAnalyzer.TestWP_CallInherited;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TObject_used}TObject = class');
  Add('    procedure {#TObjectDoA_used}DoA;');
  Add('    procedure {#TObjectDoB_used}DoB;');
  Add('  end;');
  Add('  {#TMobile_used}TMobile = class');
  Add('    procedure {#TMobileDoA_used}DoA;');
  Add('    procedure {#TMobileDoC_used}DoC;');
  Add('  end;');
  Add('procedure TObject.DoA; begin end;');
  Add('procedure TObject.DoB; begin end;');
  Add('procedure TMobile.DoA;');
  Add('begin');
  Add('  inherited;');
  Add('end;');
  Add('procedure TMobile.DoC;');
  Add('begin');
  Add('  inherited DoB;');
  Add('end;');
  Add('var o: TMobile;');
  Add('begin');
  Add('  o.DoA;');
  Add('  o.DoC;');
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_ProgramPublicDeclarations;
begin
  StartProgram(false);
  Add('var');
  Add('  {#vPublic_used}vPublic: longint; public;');
  Add('  {#vPrivate_notused}vPrivate: longint;');
  Add('procedure {#DoPublic_used}DoPublic; public; begin end;');
  Add('procedure {#DoPrivate_notused}DoPrivate; begin end;');
  Add('begin');
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_ClassOverride;
begin
  StartProgram(false);
  Add([
  'type',
  '  {#TObject_used}TObject = class',
  '  protected',
  '    function {#TObject_getcount_used}GetCount: longint; virtual; abstract;',
  '  public',
  '    property {#TObject_count_used}Count: longint read GetCount;',
  '  end;',
  '',
  '  {#tb_used}TB = class(TObject)',
  '  private',
  '    {#tb_fcount_used}FCount: longint;',
  '  protected',
  '    function {#tb_getcount_used}GetCount: longint; override;',
  '  end;',
  '',
  'function TB.GetCount: longint;',
  'begin',
  '  Result:=FCount;',
  'end;',
  '',
  'procedure {#doit_used}DoIt;',
  'var',
  '  {#l_used}l: TB;',
  'begin',
  '  if l.count=3 then ;',
  'end;',
  '',
  'begin',
  '  DoIt;']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_ClassDefaultProperty;
begin
  StartProgram(false);
  Add('type');
  Add('  {#tobject_used}TObject = class');
  Add('    function {#getitems_notused}Getitems(Index: longint): string;');
  Add('    procedure {#setitems_used}Setitems(Index: longint; Value: String);');
  Add('    property {#items_used}Items[Index: longint]: string read GetItems write SetItems; default;');
  Add('  end;');
  Add('function TObject.Getitems(Index: longint): string; begin end;');
  Add('procedure TObject.Setitems(Index: longint; Value: String); begin end;');
  Add('var');
  Add('  {#l_used}L: TObject;');
  Add('begin');
  Add('  L[0]:=''birdy'';');
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_BeforeConstruction;
begin
  StartProgram(false);
  Add([
  'type',
  ' {#tobject_used}TObject = class',
  '    procedure {#oAfter_used}AfterConstruction; virtual;',
  '    procedure {#oBefore_used}BeforeDestruction; virtual;',
  '    procedure {#oFree_used}Free;',
  '    constructor {#oCreate_used}Create;',
  '    destructor {#oDestroy_used}Destroy; virtual;',
  '    procedure {#oDoIt_notused}DoIt; virtual; abstract;',
  '  end;',
  '  TBird = class',
  '    procedure {#bAfter_used}AfterConstruction; override;',
  '    procedure {#bBefore_used}BeforeDestruction; override;',
  '  end;',
  'procedure TObject.AfterConstruction; begin end;',
  'procedure TObject.BeforeDestruction; begin end;',
  'procedure TObject.Free; begin Destroy; end;',
  'constructor TObject.Create; begin end;',
  'destructor TObject.Destroy; begin end;',
  'procedure TBird.AfterConstruction; begin end;',
  'procedure TBird.BeforeDestruction; begin end;',
  'var',
  '  {#b_used}b: TBird;',
  'begin',
  '  b:=TBird.Create;',
  '  b.Free;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_Published;
begin
  StartProgram(false);
  Add('type');
  Add('  {#tobject_notypeinfo}TObject = class');
  Add('  end;');
  Add('  {#tobject_typeinfo}TBird = class');
  Add('  private');
  Add('    {#fcol_used}FCol: string;');
  Add('    {#fbird_notused}FBird: string;');
  Add('  published');
  Add('    {#fielda_used}FieldA: longint;');
  Add('    procedure {#doit_used}ProcA; virtual; abstract;');
  Add('    property {#col_used}Col: string read FCol;');
  Add('  end;');
  Add('var');
  Add('  {#b_used}b: TBird;');
  Add('begin');
  Add('  b:=nil;');
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_PublishedSetType;
begin
  StartProgram(false);
  Add('type');
  Add('  {#tflag_used}TFlag = (red, green);');
  Add('  {#tflags_used}TFlags = set of TFlag;');
  Add('  {#tobject_used}TObject = class');
  Add('  published');
  Add('    {#fielda_used}FieldA: TFlag;');
  Add('    {#fieldb_used}FieldB: TFlags;');
  Add('  end;');
  Add('var');
  Add('  {#o_used}o: TObject;');
  Add('begin');
  Add('  o:=nil;');
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_PublishedArrayType;
begin
  StartProgram(false);
  Add('type');
  Add('  {#tdynarr_used}TDynArr = array of longint;');
  Add('  {#tstatarr_used}TStatArr = array[boolean] of longint;');
  Add('  {#tobject_used}TObject = class');
  Add('  published');
  Add('    {#fielda_used}FieldA: TDynArr;');
  Add('    {#fieldb_used}FieldB: TStatArr;');
  Add('  end;');
  Add('var');
  Add('  {#o_used}o: TObject;');
  Add('begin');
  Add('  o:=nil;');
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_PublishedClassOfType;
begin
  StartProgram(false);
  Add('type');
  Add('  {#tobjectclass_used}TObjectClass = class of TObject;');
  Add('  {#tobject_used}TObject = class');
  Add('  published');
  Add('    {#fielda_used}FieldA: TObjectClass;');
  Add('  end;');
  Add('  {#tclass_used}TClass = class of TObject;');
  Add('var');
  Add('  {#c_used}c: TClass;');
  Add('begin');
  Add('  c:=nil;');
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_PublishedRecordType;
begin
  StartProgram(false);
  Add([
  'type',
  '  {#trec_used}TRec = record',
  '    {treci_used}i: longint;',
  '  end;',
  'const c: TRec = (i:1);',
  'type',
  '  {#tobject_used}TObject = class',
  '  published',
  '    {#fielda_used}FieldA: TRec;',
  '  end;',
  'var',
  '  {#o_used}o: TObject;',
  'begin',
  '  o:=nil;']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_PublishedProcType;
begin
  StartProgram(false);
  Add('type');
  Add('  {#ta_used}ta = array of longint;');
  Add('  {#tb_used}tb = array of longint;');
  Add('  {#tproca_used}TProcA = procedure;');
  Add('  {#tfunca_used}TFuncA = function: ta;');
  Add('  {#tprocb_used}TProcB = procedure(a: tb);');
  Add('  {#tobject_used}TObject = class');
  Add('  published');
  Add('    {#fielda_used}FieldA: TProcA;');
  Add('    {#fieldb_used}FieldB: TFuncA;');
  Add('    {#fieldc_used}FieldC: TProcB;');
  Add('  end;');
  Add('var');
  Add('  {#o_used}o: TObject;');
  Add('begin');
  Add('  o:=nil;');
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_PublishedProperty;
begin
  StartProgram(false);
  Add('const');
  Add('  {#defcol_used}DefCol = 3;');
  Add('  {#defsize_notused}DefSize = 43;');
  Add('type');
  Add('  {#tobject_used}TObject = class');
  Add('  private');
  Add('    {#fcol_used}FCol: longint;');
  Add('    {#fsize_used}FSize: longint;');
  Add('    {#fbird_notused}FBird: string;');
  Add('    {#fcolstored_used}FColStored: boolean;');
  Add('    {#fsizestored_notused}FSizeStored: boolean;');
  Add('  public');
  Add('    property {#size_used}Size: longint read FSize stored FSizeStored default DefSize;');
  Add('  published');
  Add('    property {#col_used}Col: longint read FCol stored FColStored default DefCol;');
  Add('  end;');
  Add('var');
  Add('  {#o_used}o: TObject;');
  Add('begin');
  Add('  if o.Size=13 then ;');
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_BuiltInFunctions;
begin
  StartProgram(false);
  Add([
  'type',
  '  {#tordenum_used}TOrdEnum = (ordenum1,ordenum2);',
  'begin',
  '  if ord(ordenum1)=1 then ;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_TypeInfo;
begin
  StartProgram(false);
  Add([
  'type',
  '  {#integer_used}integer = longint;',
  '  {#trec_used}TRec = record',
  '    {#trecv_used}v: integer;',
  '  end;',
  '  {#tclass_used}TClass = class of TObject;',
  '  {#tobject_used}TObject = class',
  '    class function {#tobject_classtype_used}ClassType: TClass; virtual; abstract;',
  '  end;',
  '  {#tbirds_used}TBirds = class of TBird;',
  '  {#tbird_used}TBird = class',
  '  end;',
  'function {#getbirdclass_used}GetBirdClass: TBirds;',
  'begin',
  '  Result:=nil;',
  'end;',
  'var',
  '  {#i_used}i: integer;',
  '  {#s_used}s: string;',
  '  {#p_used}p: pointer;',
  '  {#r_used}r: TRec;',
  '  {#o_used}o: TObject;',
  '  {#c_used}c: TClass;',
  'begin',
  '  p:=typeinfo(integer);',
  '  p:=typeinfo(longint);',
  '  p:=typeinfo(i);',
  '  p:=typeinfo(s);',
  '  p:=typeinfo(p);',
  '  p:=typeinfo(r.v);',
  '  p:=typeinfo(TObject.ClassType);',
  '  p:=typeinfo(o.ClassType);',
  '  p:=typeinfo(o);',
  '  p:=typeinfo(c);',
  '  p:=typeinfo(c.ClassType);',
  '  p:=typeinfo(GetBirdClass);',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_TypeInfo_PropertyEnumType;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  {#talign_typeinfo}TAlign = (alLeft,alRight);',
  '  {$M+}',
  '  TPersistent = class',
  '  private',
  '    FAlign: TAlign;',
  '  public',
  '    property {#tpersistent_align_notypeinfo}Align: TAlign read FAlign write FAlign;',
  '  end;',
  '  {$M-}',
  '  {#tbutton_typeinfo}TButton = class(TPersistent)',
  '  published',
  '    property {#tbutton_align_typeinfo}Align;',
  '  end;',
  'var',
  '  {#p_notypeinfo}p: pointer;',
  'begin',
  '  p:=typeinfo(TButton);',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_TypeInfo_Alias;
begin
  AddModuleWithIntfImplSrc('mysystem.pp',
    LinesToStr([
    'type',
    '  integer = longint;',
    '  PTypeInfo = pointer;',
    '  {#tdatetime_typeinfo}TDateTime = type double;',
    '']),
    '');
  AddModuleWithIntfImplSrc('unit1.pp',
    LinesToStr([
    'uses mysystem;',
    'type',
    '  {#ttime_typeinfo}TTime = type TDateTime;',
    '  TDate = TDateTime;',
    'var',
    '  dt: TDateTime;',
    '  t: TTime;',
    '  d: TDate;',
    '  TI: PTypeInfo;',
    '']),'');
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'uses unit1;',
    '']),
    LinesToStr([
    'initialization',
    '  dt:=1.0;',
    '  t:=2.0;',
    '  d:=3.0;',
    '  ti:=typeinfo(dt);',
    '  ti:=typeinfo(t);',
    '  ti:=typeinfo(d);',
    '']));
  StartProgram(true);
  Add([
    'uses mysystem, unit2;',
    'var',
    '  PInfo: PTypeInfo;',
    'begin',
    '  PInfo:=typeinfo(TDateTime);',
    'end.']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_TypeInfo_Specialize;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  generic TProc<T> = procedure(a: T) of object;',
  '  TWordProc = specialize TProc<word>;',
  '  {$M+}',
  '  TPersistent = class',
  '  private',
  '    FWordProc: TWordProc;',
  '  published',
  '    property Proc: TWordProc read FWordProc write FWordProc;',
  '  end;',
  '  {$M-}',
  'var',
  '  {#p_notypeinfo}p: pointer;',
  'begin',
  '  p:=typeinfo(TPersistent);',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_ForInClass;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  {#tenumerator_used}TEnumerator = class',
  '  strict private',
  '    {#fcurrent_used}FCurrent: longint;',
  '  public',
  '    {#v_notused}v: string;',
  '    function {#movenext_used}MoveNext: boolean;',
  '    property {#current_used}Current: longint read FCurrent;',
  '  end;',
  '  {#tbird_used}TBird = class',
  '    function {#getenumerator_used}GetEnumerator: TEnumerator;',
  '  end;',
  'function TEnumerator.MoveNext: boolean;',
  'begin',
  'end;',
  'function TBird.GetEnumerator: TEnumerator;',
  'begin',
  'end;',
  'var',
  '  {#b_used}b: TBird;',
  '  {#i_used}i: longint;',
  'begin',
  '  for i in b do ;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_AssertSysUtils;
begin
  AddModuleWithIntfImplSrc('SysUtils.pas',
    LinesToStr([
    'type',
    '  TObject = class',
    '    constructor {#a_used}Create;',
    '  end;',
    '  {#e_used}EAssertionFailed = class',
    '    constructor {#b_used}Create(s: string);',
    '  end;',
    '']),
    LinesToStr([
    'constructor TObject.Create;',
    'begin end;',
    'constructor EAssertionFailed.Create(s: string);',
    'begin end;',
    '']) );

  StartProgram(true);
  Add([
  'uses sysutils;',
  'procedure DoIt;',
  'var',
  '  b: boolean;',
  '  s: string;',
  'begin',
  '  {$Assertions on}',
  '  Assert(b);',
  '  Assert(b,s);',
  'end;',
  'begin',
  '  DoIt;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_RangeErrorSysUtils;
begin
  AddModuleWithIntfImplSrc('SysUtils.pas',
    LinesToStr([
    'type',
    '  TObject = class',
    '    constructor {#a_used}Create;',
    '  end;',
    '  {#e_used}ERangeError = class',
    '  end;',
    '']),
    LinesToStr([
    'constructor TObject.Create;',
    'begin end;',
    '']) );

  StartProgram(true);
  Add([
  'uses sysutils;',
  'procedure DoIt;',
  'var',
  '  b: byte;',
  'begin',
  '  {$R+}',
  '  b:=1;',
  'end;',
  'begin',
  '  DoIt;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_ClassInterface;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  {#iunknown_used}IUnknown = interface',
  '    procedure {#iunknown_run_used}Run;',
  '    procedure {#iunknown_walk_notused}Walk;',
  '  end;',
  '  {#tobject_used}TObject = class',
  '  end;',
  '  {#tbird_used}TBird = class(TObject,IUnknown)',
  '  strict private',
  '    procedure IUnknown.Run = Fly;',
  '    procedure {#tbird_fly_used}Fly; virtual; abstract;',
  '    procedure {#tbird_walk_notused}Walk; virtual; abstract;',
  '  end;',
  '  {#teagle_used}TEagle = class(TBird)',
  '  strict private',
  '    procedure {#teagle_fly_used}Fly; override;',
  '    procedure {#teagle_walk_notused}Walk; override;',
  '  end;',
  'procedure TEagle.Fly; begin end;',
  'procedure TEagle.Walk; begin end;',
  'var',
  '  e: TEagle;',
  '  i: IUnknown;',
  'begin',
  '  i:=e;',
  '  i.Run;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_ClassInterface_OneWayIntfToObj;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  {#iunknown_used}IUnknown = interface',
  '    procedure {#iunknown_run_used}Run;',
  '    procedure {#iunknown_walk_notused}Walk;',// not used
  '  end;',
  '  {#tobject_used}TObject = class',
  '  end;',
  '  {#tbird_used}TBird = class(TObject,IUnknown)',
  '  strict private',
  '    procedure IUnknown.Run = Fly;',
  '    procedure {#tbird_fly_used}Fly; virtual; abstract;',
  '    procedure {#tbird_walk_notused}Walk; virtual; abstract;', // used
  '  end;',
  '  {#teagle_used}TEagle = class(TBird)',
  '  private',
  '    procedure {#teagle_fly_used}Fly; override;',
  '    procedure {#teagle_walk_used}Walk; override;',
  '  end;',
  'procedure TEagle.Fly; begin end;',
  'procedure TEagle.Walk; begin end;',
  'var',
  '  e: TEagle;',
  '  i: IUnknown;',
  'begin',
  '  i:=e;',
  '  i.Run;',  // using IUnknown.Walk must mark TEagle.Walk
  '  e.Walk;', // using TEagle.Walk must not mark IUnknown.Walk
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_ClassInterface_Delegation;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  {#iunknown_used}IUnknown = interface',
  '    procedure {#iunknown_run_used}Run;',
  '    procedure {#iunknown_walk_notused}Walk;',
  '  end;',
  '  {#tobject_used}TObject = class',
  '  end;',
  '  {#tbird_used}TBird = class(TObject,IUnknown)',
  '  strict private',
  '    procedure IUnknown.Run = Fly;',
  '    procedure {#tbird_fly_used}Fly;',
  '    procedure {#tbird_walk_notused}Walk;',
  '  end;',
  '  {#teagle_used}TEagle = class(TObject,IUnknown)',
  '  strict private',
  '    {#teagle_fbird_used}FBird: TBird;',
  '    property {#teagle_bird_used}Bird: TBird read FBird implements IUnknown;',
  '  end;',
  'procedure TBird.Fly; begin end;',
  'procedure TBird.Walk; begin end;',
  'var',
  '  e: TEagle;',
  '  i: IUnknown;',
  'begin',
  '  i:=e;',
  '  i.Run;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_ClassInterface_COM;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  {#tguid_used}TGuid = string;',
  '  {#integer_used}integer = longint;',
  '  {#iunknown_used}IUnknown = interface',
  '    function {#iunknown_queryintf_used}QueryInterface(const iid: TGuid; out obj): Integer;',
  '    function {#iunknown_addref_used}_AddRef: Integer;',
  '    function {#iunknown_release_used}_Release: Integer;',
  '    procedure {#iunknown_doit_notused}DoIt;',
  '  end;',
  '  {#tobject_used}TObject = class',
  '  end;',
  '  {#tbird_used}TBird = class(TObject,IUnknown)',
  '  strict private',
  '    function {#tbird_queryintf_used}QueryInterface(const iid: TGuid; out obj): Integer;',
  '    function {#tbird_addref_used}_AddRef: Integer;',
  '    function {#tbird_release_used}_Release: Integer;',
  '    procedure {#tbird_doit_notused}DoIt;',
  '  end;',
  '  {#teagle_used}TEagle = class(TBird)',
  '  end;',
  'function TBird.QueryInterface(const iid: TGuid; out obj): Integer;',
  'begin',
  '  if iid='''' then obj:=nil;',
  '  Result:=0;',
  'end;',
  'function TBird._AddRef: Integer; begin Result:=1; end;',
  'function TBird._Release: Integer; begin Result:=2; end;',
  'procedure TBird.DoIt; begin end;',
  'var',
  '  e: TEagle;',
  '  i: IUnknown;',
  'begin',
  '  i:=e;',
  '  if i=nil then ;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_ClassInterface_COM_Unit;
begin
  AddModuleWithIntfImplSrc('SysUtils.pas',
    LinesToStr([
    '{$interfaces com}',
    'type',
    '  {#tguid_used}TGuid = string;',
    '  {#integer_used}integer = longint;',
    '  {#iunknown_used}IUnknown = interface',
    '    function {#iunknown_queryintf_used}QueryInterface(const iid: TGuid; out obj): Integer;',
    '    function {#iunknown_addref_used}_AddRef: Integer;',
    '    function {#iunknown_release_used}_Release: Integer;',
    '    procedure {#iunknown_doit_notused}DoIt;',
    '  end;',
    '  IBird = interface(IUnknown)',
    '    procedure {#ibird_fly_used}Fly;',
    '  end;',
    '  {#tobject_used}TObject = class',
    '  end;',
    '  {#tbird_used}TBird = class(TObject,IBird)',
    '  strict private',
    '    function {#tbird_queryintf_used}QueryInterface(const iid: TGuid; out obj): Integer;',
    '    function {#tbird_addref_used}_AddRef: Integer;',
    '    function {#tbird_release_used}_Release: Integer;',
    '    procedure {#tbird_doit_notused}DoIt;',
    '    procedure {#tbird_fly_used}Fly;',
    '  end;',
    '']),
    LinesToStr([
    'function TBird.QueryInterface(const iid: TGuid; out obj): Integer;',
    'begin',
    '  if iid='''' then obj:=nil;',
    '  Result:=0;',
    'end;',
    'function TBird._AddRef: Integer; begin Result:=1; end;',
    'function TBird._Release: Integer; begin Result:=2; end;',
    'procedure TBird.DoIt; begin end;',
    'procedure TBird.Fly; begin end;',
    '']) );

  StartProgram(true);
  Add([
  'uses sysutils;',
  'type',
  '  {#teagle_used}TEagle = class(TBird)',
  '  end;',
  'var',
  '  e: TEagle;',
  '  i: IBird;',
  'begin',
  '  i:=e;',
  '  if i=nil then ;',
  '  i.Fly;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_ClassInterface_Typeinfo;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  {#iunknown_typeinfo}IUnknown = interface',
  '    function {#iunknown_getflag_typeinfo}GetFlag: boolean;',
  '    procedure {#iunknown_setflag_typeinfo}SetFlag(Value: boolean);',
  '    procedure {#iunknown_doit_notypeinfo}DoIt;',
  '    property {#iunknown_flag_typeinfo}Flag: boolean read GetFlag write SetFlag;',
  '  end;',
  '  {#ibird_notused}IBird = interface(IUnknown)',
  '  end;',
  'var',
  '  t: pointer;',
  '  i: IUnknown;',
  'begin',
  '  t:=typeinfo(IUnknown);',
  '  if i.Flag then ;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_ClassInterface_TGUID;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  TGuid = record',
  '    {#d1_used}D1: longword;',
  '    {#d2_used}D2: word;',
  '    {#d3_used}D3: word;',
  '    {#d4_used}D4: array[0..7] of byte;',
  '  end;',
  'var g,h: TGuid;',
  'begin',
  '  if g=h then ;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_ClassHelper;
begin
  StartProgram(false);
  Add([
  'type',
  '  {#TObject_used}TObject = class',
  '  end;',
  '  {#TBird_used}TBird = class',
  '    {#TBird_A_notused}A: word;',
  '  end;',
  '  {#TAnt_used}TAnt = class',
  '    {#TAnt_B_notused}B: word;',
  '  type',
  '    {#TMouth_used}TMouth = class',
  '      {#TMouth_C_notused}C: word;',
  '    type',
  '      {#TBirdHelper_used}TBirdHelper = class helper for TBird',
  '        procedure {#TBirdHelper_Fly_used}Fly;',
  '      end;',
  '    end;',
  '  end;',
  'procedure TAnt.TMouth.TBirdHelper.Fly;',
  'begin',
  'end;',
  'var b: TBird;',
  'begin',
  '  b.Fly;;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_ClassHelper_ClassConstrucor_Used;
begin
  StartProgram(false);
  Add([
  'type',
  '  {#TObject_used}TObject = class',
  '    class constructor {#TObject_Init_used}Init;',
  '    class destructor {#TObject_Done_used}Done;',
  '  end;',
  '  {#TBird_used}TBird = class',
  '    {#TBird_A_notused}A: word;',
  '    class constructor {#TBird_Init_used}Init;',
  '    class destructor {#TBird_Done_used}Done;',
  '  end;',
  '  {#TBirdHelper_used}TBirdHelper = class helper for TBird',
  '    procedure {#TBirdHelper_Fly_used}Fly;',
  '    class constructor {#TBirdHelper_Init_used}Init;',
  '    class destructor {#TBirdHelper_Done_used}Done;',
  '  end;',
  '  TAnt = class',
  '    class constructor {#TAnt_Init_notused}Init;',
  '    class destructor {#TAnt_Done_notused}Done;',
  '  end;',
  'class constructor TObject.Init;',
  'begin',
  'end;',
  'class destructor TObject.Done;',
  'begin',
  'end;',
  'class constructor TBird.Init;',
  'begin',
  'end;',
  'class destructor TBird.Done;',
  'begin',
  'end;',
  'procedure TBirdHelper.Fly;',
  'begin',
  'end;',
  'class constructor TBirdHelper.Init;',
  'begin',
  'end;',
  'class destructor TBirdHelper.Done;',
  'begin',
  'end;',
  'class constructor TAnt.Init;',
  'begin',
  'end;',
  'class destructor TAnt.Done;',
  'begin',
  'end;',
  'var b: TBird;',
  'begin',
  '  b.Fly;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_Attributes;
begin
  StartProgram(false);
  Add([
  '{$modeswitch prefixedattributes}',
  'type',
  '  TObject = class',
  '    constructor {#TObject_Create_notused}Create;',
  '  end;',
  '  {#TCustomAttribute_used}TCustomAttribute = class',
  '  end;',
  '  {#RedAttribute_used}RedAttribute = class(TCustomAttribute)',
  '    constructor {#Red_A_used}Create(Id: word = 3; Deep: boolean = false); overload;',
  '    constructor {#Red_B_notused}Create(Size: double); overload;',
  '  end;',
  '  {#Red_notused}Red = word;',
  'constructor TObject.Create; begin end;',
  'constructor RedAttribute.Create(Id: word; Deep: boolean); begin end;',
  'constructor RedAttribute.Create(Size: double); begin end;',
  'var',
  '  [NotExisting]',
  '  [Red]',
  '  o: TObject;',
  'begin',
  '  if typeinfo(o)=nil then ;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_Attributes_ForwardClass;
begin
  StartProgram(false);
  Add([
  '{$modeswitch prefixedattributes}',
  'type',
  '  TObject = class',
  '    constructor {#TObject_Create_used}Create;',
  '  end;',
  '  {#TRedAttribute_notused}TRedAttribute = class',
  '  end;',
  '  {#TCustomAttribute_used}TCustomAttribute = class',
  '  end;',
  '  [TCustom]',
  '  TBird = class;',
  '  TMyInt = word;',
  '  TBird = class end;',
  'constructor TObject.Create;',
  'begin',
  'end;',
  'var b: TBird;',
  'begin',
  '  b:=TBird.Create;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_Attributes_Params;
begin
  StartProgram(false);
  Add([
  '{$modeswitch prefixedattributes}',
  'type',
  '  TObject = class',
  '    constructor {#TObject_Create_notused}Create;',
  '    destructor {#TObject_Destroy_used}Destroy; virtual;',
  '  end;',
  '  {#TCustomAttribute_used}TCustomAttribute = class',
  '  end;',
  '  {#BigAttribute_used}BigAttribute = class(TCustomAttribute)',
  '    constructor {#Big_A_used}Create(Id: word = 3); overload;',
  '    destructor {#Big_B_used}Destroy; override;',
  '  end;',
  'constructor TObject.Create; begin end;',
  'destructor TObject.Destroy; begin end;',
  'constructor BigAttribute.Create(Id: word); begin end;',
  'destructor BigAttribute.Destroy; begin end;',
  'var',
  '  [Big(3)]',
  '  o: TObject;',
  '  a: TCustomAttribute;',
  'begin',
  '  if typeinfo(o)=nil then ;',
  '  a.Destroy;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestWP_Attributes_PublishedFields;
begin
  exit;

  StartProgram(false);
  Add([
  '{$modeswitch prefixedattributes}',
  'type',
  '  TObject = class',
  '    constructor {#TObject_Create_notused}Create;',
  '    destructor {#TObject_Destroy_used}Destroy; virtual;',
  '  end;',
  '  {#TCustomAttribute_used}TCustomAttribute = class',
  '  end;',
  '  {#BigAttribute_used}BigAttribute = class(TCustomAttribute)',
  '    constructor {#Big_A_used}Create(Id: word = 3); overload;',
  '    destructor {#Big_B_used}Destroy; override;',
  '  end;',
  '  {$M+}',
  '  TBird = class',
  '  public',
  '    FColor: word;',
  '  published',
  '    Size: word;',
  '    procedure Fly;',
  '    [Big(3)]',
  '    property Color: word read FColor;',
  '  end;',
  'constructor TObject.Create; begin end;',
  'destructor TObject.Destroy; begin end;',
  'constructor BigAttribute.Create(Id: word); begin end;',
  'destructor BigAttribute.Destroy; begin end;',
  'var',
  '  b: TBird;',
  'begin',
  '  if typeinfo(b)=nil then ;',
  '']);
  AnalyzeWholeProgram;
end;

procedure TTestUseAnalyzer.TestSR_Proc_UnitVar;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TColor = longint;',
  '  TIntColor = TColor;',
  'var',
  '  i: longint;',
  '  j: longint;',
  'procedure DoIt;',
  'implementation',
  'procedure DoIt;',
  'type',
  '  TSubColor = TIntColor;',
  'var',
  '  b: TSubColor;',
  'begin',
  '  b:=i;',
  'end;',
  '']);
  Analyzer.Options:=Analyzer.Options+[paoImplReferences];
  AnalyzeUnit;
  CheckScopeReferences('DoIt',['i','tintcolor']);
end;

procedure TTestUseAnalyzer.TestSR_Init_UnitVar;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TColor = longint;',
  '  TIntColor = TColor;',
  'var',
  '  i: longint;',
  '  j: longint;',
  'implementation',
  'type',
  '  TSubColor = TIntColor;',
  'var',
  '  b: TSubColor;',
  'initialization',
  '  b:=i;',
  'finalization',
  '  b:=j;',
  'end.',
  '']);
  Analyzer.Options:=Analyzer.Options+[paoImplReferences];
  AnalyzeUnit;
  CheckScopeReferences('initialization',['b','i']);
  CheckScopeReferences('finalization',['b','j']);
end;

initialization
  RegisterTests([TTestUseAnalyzer]);

end.


