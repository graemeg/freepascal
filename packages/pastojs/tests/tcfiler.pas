{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2018 by Michael Van Canneyt

    Unit tests for Pascal-to-Javascript precompile class.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

 Examples:
   ./testpas2js --suite=TTestPrecompile.TestPC_EmptyUnit
}
unit TCFiler;

{$i ../src/pastojs.inc}

interface

uses
  Classes, SysUtils, fpcunit, testregistry,
  jstree,
  PasTree, PScanner, PParser, PasResolveEval, PasResolver, PasUseAnalyzer,
  Pas2jsUseAnalyzer, FPPas2Js, Pas2JsFiler,
  tcmodules;

type
  TPCCheckFlag = (
    PCCGeneric // inside generic proc body
    );
  TPCCheckFlags = set of TPCCheckFlag;

  TPCCheckedElementPair = class
  public
    Orig, Rest: TPasElement;
  end;

  { TCustomTestPrecompile }

  TCustomTestPrecompile = Class(TCustomTestModule)
  private
    FAnalyzer: TPas2JSAnalyzer;
    FInitialFlags: TPCUInitialFlags;
    FPCUReader: TPCUReader;
    FPCUWriter: TPCUWriter;
    FRestAnalyzer: TPas2JSAnalyzer;
    FCheckedElements: TPasAnalyzerKeySet; // keyset of TPCCheckedElementPair, key is Orig
    procedure OnFilerGetSrc(Sender: TObject; aFilename: string; out p: PChar;
      out Count: integer);
    function OnConverterIsElementUsed(Sender: TObject; El: TPasElement): boolean;
    function OnConverterIsTypeInfoUsed(Sender: TObject; El: TPasElement): boolean;
    function OnRestConverterIsElementUsed(Sender: TObject; El: TPasElement): boolean;
    function OnRestConverterIsTypeInfoUsed(Sender: TObject; El: TPasElement): boolean;
    function OnRestResolverFindUnit(const aUnitName: String): TPasModule;
  protected
    procedure SetUp; override;
    procedure TearDown; override;
    function CreateConverter: TPasToJSConverter; override;
    procedure ParseUnit; override;
    procedure WriteReadUnit; virtual;
    procedure StartParsing; override;
    function CheckRestoredObject(const Path: string; Orig, Rest: TObject): boolean; virtual;
    procedure CheckRestoredJS(const Path, Orig, Rest: string); virtual;
    procedure CheckRestoredStringList(const Path: string; Orig, Rest: TStrings); virtual;
    // check restored parser+resolver
    procedure CheckRestoredResolver(Original, Restored: TPas2JSResolver; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredDeclarations(const Path: string; Orig, Rest: TPasDeclarations; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredSection(const Path: string; Orig, Rest: TPasSection; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredModule(const Path: string; Orig, Rest: TPasModule; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredScopeReference(const Path: string; Orig, Rest: TPasScope; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredElementBase(const Path: string; Orig, Rest: TPasElementBase; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredResolveData(const Path: string; Orig, Rest: TResolveData; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredPasScope(const Path: string; Orig, Rest: TPasScope; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredLocalVar(const Path: string; Orig, Rest: TPas2JSStoredLocalVar); virtual;
    procedure CheckRestoredModuleScope(const Path: string; Orig, Rest: TPas2JSModuleScope; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredIdentifierScope(const Path: string; Orig, Rest: TPasIdentifierScope; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredSectionScope(const Path: string; Orig, Rest: TPas2JSSectionScope; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredInitialFinalizationScope(const Path: string; Orig, Rest: TPas2JSInitialFinalizationScope; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredEnumTypeScope(const Path: string; Orig, Rest: TPasEnumTypeScope; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredRecordScope(const Path: string; Orig, Rest: TPas2jsRecordScope; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredClassScope(const Path: string; Orig, Rest: TPas2JSClassScope; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredProcScope(const Path: string; Orig, Rest: TPas2JSProcedureScope; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredProcTypeScope(const Path: string; Orig, Rest: TPas2JSProcTypeScope; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredArrayScope(const Path: string; Orig, Rest: TPas2JSArrayScope; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredPrecompiledJS(const Path: string; OrigEl: TPasElement; Orig: TPas2JSPrecompiledJS; RestEl: TPasElement; Rest: TPas2JSPrecompiledJS; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredScopeRefs(const Path: string; Orig, Rest: TPasScopeReferences; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredPropertyScope(const Path: string; Orig, Rest: TPasPropertyScope; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredGenericParamScope(const Path: string; Orig, Rest: TPasGenericParamsScope; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredSpecializeTypeData(const Path: string; Orig, Rest: TPasSpecializeTypeData; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredResolvedReference(const Path: string; Orig, Rest: TResolvedReference; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredEvalValue(const Path: string; Orig, Rest: TResEvalValue); virtual;
    procedure CheckRestoredCustomData(const Path: string; RestoredEl: TPasElement; Orig, Rest: TObject; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredReference(const Path: string; Orig, Rest: TPasElement); virtual;
    procedure CheckRestoredElOrRef(const Path: string; Orig, OrigProp, Rest, RestProp: TPasElement; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredAnalyzerElement(const Path: string; Orig, Rest: TPasElement); virtual;
    procedure CheckRestoredElement(const Path: string; Orig, Rest: TPasElement; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredElementList(const Path: string; Orig, Rest: TFPList; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredElementArray(const Path: string; Orig, Rest: TPasElementArray; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredElRefList(const Path: string; OrigParent: TPasElement;
      Orig: TFPList; RestParent: TPasElement; Rest: TFPList; AllowInSitu: boolean; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredPasExpr(const Path: string; Orig, Rest: TPasExpr; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredUnaryExpr(const Path: string; Orig, Rest: TUnaryExpr; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredBinaryExpr(const Path: string; Orig, Rest: TBinaryExpr; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredPrimitiveExpr(const Path: string; Orig, Rest: TPrimitiveExpr; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredBoolConstExpr(const Path: string; Orig, Rest: TBoolConstExpr; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredParamsExpr(const Path: string; Orig, Rest: TParamsExpr; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredProcedureExpr(const Path: string; Orig, Rest: TProcedureExpr; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredRecordValues(const Path: string; Orig, Rest: TRecordValues; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredPasExprArray(const Path: string; Orig, Rest: TPasExprArray; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredArrayValues(const Path: string; Orig, Rest: TArrayValues; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredResString(const Path: string; Orig, Rest: TPasResString; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredAliasType(const Path: string; Orig, Rest: TPasAliasType; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredPointerType(const Path: string; Orig, Rest: TPasPointerType; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredSpecializedType(const Path: string; Orig, Rest: TPasSpecializeType; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredInlineSpecializedExpr(const Path: string; Orig, Rest: TInlineSpecializeExpr; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredGenericTemplateType(const Path: string; Orig, Rest: TPasGenericTemplateType; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredRangeType(const Path: string; Orig, Rest: TPasRangeType; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredArrayType(const Path: string; Orig, Rest: TPasArrayType; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredFileType(const Path: string; Orig, Rest: TPasFileType; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredEnumValue(const Path: string; Orig, Rest: TPasEnumValue; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredEnumType(const Path: string; Orig, Rest: TPasEnumType; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredSetType(const Path: string; Orig, Rest: TPasSetType; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredVariant(const Path: string; Orig, Rest: TPasVariant; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredRecordType(const Path: string; Orig, Rest: TPasRecordType; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredClassType(const Path: string; Orig, Rest: TPasClassType; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredArgument(const Path: string; Orig, Rest: TPasArgument; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredProcedureType(const Path: string; Orig, Rest: TPasProcedureType; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredResultElement(const Path: string; Orig, Rest: TPasResultElement; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredFunctionType(const Path: string; Orig, Rest: TPasFunctionType; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredStringType(const Path: string; Orig, Rest: TPasStringType; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredVariable(const Path: string; Orig, Rest: TPasVariable; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredExportSymbol(const Path: string; Orig, Rest: TPasExportSymbol; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredConst(const Path: string; Orig, Rest: TPasConst; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredProperty(const Path: string; Orig, Rest: TPasProperty; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredMethodResolution(const Path: string; Orig, Rest: TPasMethodResolution; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredProcNameParts(const Path: string; Orig, Rest: TPasProcedure; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredProcedure(const Path: string; Orig, Rest: TPasProcedure; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredOperator(const Path: string; Orig, Rest: TPasOperator; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredProcedureBody(const Path: string; Orig, Rest: TProcedureBody; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredAttributes(const Path: string; Orig, Rest: TPasAttributes; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplCommand(const Path: string; Orig, Rest: TPasImplCommand; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplBeginBlock(const Path: string; Orig, Rest: TPasImplBeginBlock; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplAsmStatement(const Path: string; Orig, Rest: TPasImplAsmStatement; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplRepeatUntil(const Path: string; Orig, Rest: TPasImplRepeatUntil; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplIfElse(const Path: string; Orig, Rest: TPasImplIfElse; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplWhileDo(const Path: string; Orig, Rest: TPasImplWhileDo; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplWithDo(const Path: string; Orig, Rest: TPasImplWithDo; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplCaseOf(const Path: string; Orig, Rest: TPasImplCaseOf; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplCaseStatement(const Path: string; Orig, Rest: TPasImplCaseStatement; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplCaseElse(const Path: string; Orig, Rest: TPasImplCaseElse; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplForLoop(const Path: string; Orig, Rest: TPasImplForLoop; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplAssign(const Path: string; Orig, Rest: TPasImplAssign; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplSimple(const Path: string; Orig, Rest: TPasImplSimple; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplTry(const Path: string; Orig, Rest: TPasImplTry; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplTryHandler(const Path: string; Orig, Rest: TPasImplTryHandler; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplExceptOn(const Path: string; Orig, Rest: TPasImplExceptOn; Flags: TPCCheckFlags); virtual;
    procedure CheckRestoredImplRaise(const Path: string; Orig, Rest: TPasImplRaise; Flags: TPCCheckFlags); virtual;
  public
    property Analyzer: TPas2JSAnalyzer read FAnalyzer;
    property RestAnalyzer: TPas2JSAnalyzer read FRestAnalyzer;
    property PCUWriter: TPCUWriter read FPCUWriter write FPCUWriter;
    property PCUReader: TPCUReader read FPCUReader write FPCUReader;
    property InitialFlags: TPCUInitialFlags read FInitialFlags;
  end;

  { TTestPrecompile }

  TTestPrecompile = class(TCustomTestPrecompile)
  published
    procedure Test_Base256VLQ;
    procedure TestPC_EmptyUnit;

    procedure TestPC_Const;
    procedure TestPC_Var;
    procedure TestPC_Enum;
    procedure TestPC_Set;
    procedure TestPC_Set_InFunction;
    procedure TestPC_SetOfAnonymousEnumType;
    procedure TestPC_Record;
    procedure TestPC_Record_InFunction;
    procedure TestPC_RecordAdv;
    procedure TestPC_JSValue;
    procedure TestPC_Array;
    procedure TestPC_ArrayOfAnonymous;
    procedure TestPC_Array_InFunction;
    procedure TestPC_Proc;
    procedure TestPC_Proc_Nested;
    procedure TestPC_Proc_LocalConst;
    procedure TestPC_Proc_UTF8;
    procedure TestPC_Proc_Arg;
    procedure TestPC_ProcType;
    procedure TestPC_Proc_Anonymous;
    procedure TestPC_Proc_ArrayOfConst;
    procedure TestPC_Class;
    procedure TestPC_ClassForward;
    procedure TestPC_ClassConstructor;
    procedure TestPC_ClassDestructor;
    procedure TestPC_ClassDispatchMessage;
    procedure TestPC_Initialization;
    procedure TestPC_BoolSwitches;
    procedure TestPC_ClassInterface;
    procedure TestPC_Attributes;

    procedure TestPC_GenericFunction_Assign;
    procedure TestPC_GenericFunction_Asm;
    procedure TestPC_GenericFunction_RepeatUntil;
    procedure TestPC_GenericFunction_IfElse;
    procedure TestPC_GenericFunction_WhileDo;
    procedure TestPC_GenericFunction_WithDo;
    procedure TestPC_GenericFunction_CaseOf;
    procedure TestPC_GenericFunction_ForLoop;
    procedure TestPC_GenericFunction_Simple;
    procedure TestPC_GenericFunction_TryFinally;
    procedure TestPC_GenericFunction_TryExcept;
    procedure TestPC_GenericFunction_LocalProc;
    procedure TestPC_GenericFunction_AnonymousProc;
    procedure TestPC_GenericClass;
    procedure TestPC_GenericMethod;
    // ToDo: GenericMethod Calls, ProcTypes
    procedure TestPC_SpecializeClassSameUnit;
    procedure TestPC_Specialize_LocalTypeInUnit;
    procedure TestPC_Specialize_ClassForward;
    procedure TestPC_InlineSpecialize_LocalTypeInUnit;
    procedure TestPC_Specialize_Array;
    procedure TestPC_Specialize_ProcType;
    // ToDo: half specialize TBird<T> = class a: TAnt<word,T>; end;
    // ToDo: no specialize: TBird<T> = class a: TBird<T>; end;
    procedure TestPC_Constraints;
    // ToDo: constraints
    // ToDo: unit impl declarations used by generics
    procedure TestPC_GenericClass_InlineSpecialize;

    procedure TestPC_UseUnit;
    procedure TestPC_UseUnit_Class;
    procedure TestPC_UseIndirectUnit;
  end;

function CompareListOfProcScopeRef(Item1, Item2: Pointer): integer;
function CompareCheckedElementPairs(Item1, Item2: Pointer): integer;
function CompareElWithCheckedElementPair(Key, Item: Pointer): integer;

implementation

function CompareListOfProcScopeRef(Item1, Item2: Pointer): integer;
var
  Ref1: TPasScopeReference absolute Item1;
  Ref2: TPasScopeReference absolute Item2;
begin
  Result:=CompareText(GetObjPath(Ref1.Element),GetObjPath(Ref2.Element));
  if Result<>0 then exit;
  Result:=ComparePointer(Ref1.Element,Ref2.Element);
end;

function CompareCheckedElementPairs(Item1, Item2: Pointer): integer;
var
  Pair1: TPCCheckedElementPair absolute Item1;
  Pair2: TPCCheckedElementPair absolute Item2;
begin
  Result:=ComparePointer(Pair1.Orig,Pair2.Orig);
end;

function CompareElWithCheckedElementPair(Key, Item: Pointer): integer;
var
  El: TPasElement absolute Key;
  Pair: TPCCheckedElementPair absolute Item;
begin
  Result:=ComparePointer(El,Pair.Orig);
end;

{ TCustomTestPrecompile }

procedure TCustomTestPrecompile.OnFilerGetSrc(Sender: TObject;
  aFilename: string; out p: PChar; out Count: integer);
var
  i: Integer;
  aModule: TTestEnginePasResolver;
  Src: String;
begin
  for i:=0 to ResolverCount-1 do
    begin
    aModule:=Resolvers[i];
    if aModule.Filename<>aFilename then continue;
    Src:=aModule.Source;
    p:=PChar(Src);
    Count:=length(Src);
    end;
end;

function TCustomTestPrecompile.OnConverterIsElementUsed(Sender: TObject;
  El: TPasElement): boolean;
begin
  Result:=Analyzer.IsUsed(El);
end;

function TCustomTestPrecompile.OnConverterIsTypeInfoUsed(Sender: TObject;
  El: TPasElement): boolean;
begin
  Result:=Analyzer.IsTypeInfoUsed(El);
end;

function TCustomTestPrecompile.OnRestConverterIsElementUsed(Sender: TObject;
  El: TPasElement): boolean;
begin
  Result:=RestAnalyzer.IsUsed(El);
end;

function TCustomTestPrecompile.OnRestConverterIsTypeInfoUsed(Sender: TObject;
  El: TPasElement): boolean;
begin
  Result:=RestAnalyzer.IsTypeInfoUsed(El);
end;

function TCustomTestPrecompile.OnRestResolverFindUnit(const aUnitName: String
  ): TPasModule;

  function FindRestUnit(Name: string): TPasModule;
  var
    i: Integer;
    CurEngine: TTestEnginePasResolver;
    CurUnitName: String;
  begin
    for i:=0 to ResolverCount-1 do
      begin
      CurEngine:=Resolvers[i];
      CurUnitName:=ExtractFileUnitName(CurEngine.Filename);
      {$IFDEF VerbosePCUFiler}
      //writeln('TCustomTestPrecompile.FindRestUnit Checking ',i,'/',ResolverCount,' ',CurEngine.Filename,' ',CurUnitName);
      {$ENDIF}
      if CompareText(Name,CurUnitName)=0 then
        begin
        Result:=CurEngine.Module;
        if Result<>nil then
          begin
          {$IFDEF VerbosePCUFiler}
          //writeln('TCustomTestPrecompile.FindRestUnit Found parsed module: ',Result.Filename);
          {$ENDIF}
          exit;
          end;
        {$IFDEF VerbosePCUFiler}
        writeln('TCustomTestPrecompile.FindRestUnit PARSING unit "',CurEngine.Filename,'"');
        {$ENDIF}
        Fail('not parsed');
        end;
      end;
  end;

var
  DefNamespace: String;
begin
  if (Pos('.',aUnitName)<1) then
    begin
    DefNamespace:=GetDefaultNamespace;
    if DefNamespace<>'' then
      begin
      Result:=FindRestUnit(DefNamespace+'.'+aUnitName);
      if Result<>nil then exit;
      end;
    end;
  Result:=FindRestUnit(aUnitName);
end;

procedure TCustomTestPrecompile.SetUp;
begin
  inherited SetUp;
  FInitialFlags:=TPCUInitialFlags.Create;
  FAnalyzer:=TPas2JSAnalyzer.Create;
  FCheckedElements:=TPasAnalyzerKeySet.Create(@CompareCheckedElementPairs,@CompareElWithCheckedElementPair);
  Analyzer.Resolver:=Engine;
  Analyzer.Options:=Analyzer.Options+[paoImplReferences];
  Converter.OnIsElementUsed:=@OnConverterIsElementUsed;
  Converter.OnIsTypeInfoUsed:=@OnConverterIsTypeInfoUsed;
end;

procedure TCustomTestPrecompile.TearDown;
begin
  if FCheckedElements<>nil then
    begin
    FCheckedElements.FreeItems;
    FreeAndNil(FCheckedElements);
    end;
  FreeAndNil(FAnalyzer);
  FreeAndNil(FPCUWriter);
  FreeAndNil(FPCUReader);
  FreeAndNil(FInitialFlags);
  inherited TearDown;
end;

function TCustomTestPrecompile.CreateConverter: TPasToJSConverter;
begin
  Result:=inherited CreateConverter;
  Result.Options:=Result.Options+[coStoreImplJS];
end;

procedure TCustomTestPrecompile.ParseUnit;
begin
  inherited ParseUnit;
  Analyzer.AnalyzeModule(Module);
end;

procedure TCustomTestPrecompile.WriteReadUnit;
var
  ms: TMemoryStream;
  PCU, RestJSSrc, OrigJSSrc: string;
  // restored classes:
  RestResolver: TTestEnginePasResolver;
  RestFileResolver: TFileResolver;
  RestScanner: TPas2jsPasScanner;
  RestParser: TPasParser;
  RestConverter: TPasToJSConverter;
  RestJSModule: TJSSourceElements;
  InitialParserOptions: TPOptions;
begin
  InitialParserOptions:=Parser.Options;
  Analyzer.Options:=Analyzer.Options+[paoSkipGenericProc];
  Converter.Options:=Converter.Options+[coShortRefGlobals];
  ConvertUnit;

  FPCUWriter:=TPCUWriter.Create;
  FPCUReader:=TPCUReader.Create;
  ms:=TMemoryStream.Create;
  RestParser:=nil;
  RestScanner:=nil;
  RestResolver:=nil;
  RestFileResolver:=nil;
  RestConverter:=nil;
  RestJSModule:=nil;
  try
    try
      PCUWriter.OnGetSrc:=@OnFilerGetSrc;
      PCUWriter.OnIsElementUsed:=@OnConverterIsElementUsed;
      PCUWriter.WritePCU(Engine,Converter,InitialFlags,ms,false);
    except
      on E: Exception do
      begin
        {$IFDEF VerbosePas2JS}
        writeln('TCustomTestPrecompile.WriteReadUnit WRITE failed');
        {$ENDIF}
        Fail('Write failed('+E.ClassName+'): '+E.Message);
      end;
    end;

    try
      PCU:='';
      SetLength(PCU,ms.Size);
      System.Move(ms.Memory^,PCU[1],length(PCU));

      writeln('TCustomTestPrecompile.WriteReadUnit PCU START-----');
      writeln(PCU);
      writeln('TCustomTestPrecompile.WriteReadUnit PCU END-------');

      RestFileResolver:=TFileResolver.Create;
      RestScanner:=TPas2jsPasScanner.Create(RestFileResolver);
      InitScanner(RestScanner);
      RestResolver:=TTestEnginePasResolver.Create;
      RestResolver.Filename:=Engine.Filename;
      RestResolver.AddObjFPCBuiltInIdentifiers(btAllJSBaseTypes,bfAllJSBaseProcs);
      RestResolver.OnFindUnit:=@OnRestResolverFindUnit;
      RestParser:=TPasParser.Create(RestScanner,RestFileResolver,RestResolver);
      RestParser.Options:=InitialParserOptions;
      RestResolver.CurrentParser:=RestParser;
      ms.Position:=0;
      PCUReader.ReadPCU(RestResolver,ms);
      if not PCUReader.ReadContinue then
        Fail('ReadContinue=false, pending used interfaces');
    except
      on E: Exception do
      begin
        {$IFDEF VerbosePas2JS}
        writeln('TCustomTestPrecompile.WriteReadUnit READ failed');
        {$ENDIF}
        Fail('Read failed('+E.ClassName+'): '+E.Message);
      end;
    end;

    // analyze
    FRestAnalyzer:=TPas2JSAnalyzer.Create;
    FRestAnalyzer.Resolver:=RestResolver;
    FRestAnalyzer.Options:=FRestAnalyzer.Options+[paoSkipGenericProc];
    try
      RestAnalyzer.AnalyzeModule(RestResolver.RootElement);
    except
      on E: Exception do
      begin
        {$IFDEF VerbosePas2JS}
        writeln('TCustomTestPrecompile.WriteReadUnit ANALYZEMODULE failed');
        {$ENDIF}
        Fail('AnalyzeModule precompiled failed('+E.ClassName+'): '+E.Message);
      end;
    end;
    // check parser+resolver+analyzer
    CheckRestoredResolver(Engine,RestResolver,[]);

    // convert using the precompiled procs
    RestConverter:=CreateConverter;
    RestConverter.Options:=Converter.Options;
    RestConverter.OnIsElementUsed:=@OnRestConverterIsElementUsed;
    RestConverter.OnIsTypeInfoUsed:=@OnRestConverterIsTypeInfoUsed;
    try
      RestJSModule:=RestConverter.ConvertPasElement(RestResolver.RootElement,RestResolver) as TJSSourceElements;
    except
      on E: Exception do
      begin
        {$IFDEF VerbosePas2JS}
        writeln('TCustomTestPrecompile.WriteReadUnit CONVERTER failed');
        {$ENDIF}
        Fail('Convert precompiled failed('+E.ClassName+'): '+E.Message);
      end;
    end;

    OrigJSSrc:=JSToStr(JSModule);
    RestJSSrc:=JSToStr(RestJSModule);

    if OrigJSSrc<>RestJSSrc then
      begin
      writeln('TCustomTestPrecompile.WriteReadUnit OrigJSSrc:---------START');
      writeln(OrigJSSrc);
      writeln('TCustomTestPrecompile.WriteReadUnit OrigJSSrc:---------END');
      writeln('TCustomTestPrecompile.WriteReadUnit RestJSSrc:---------START');
      writeln(RestJSSrc);
      writeln('TCustomTestPrecompile.WriteReadUnit RestJSSrc:---------END');
      CheckDiff('WriteReadUnit JS diff',OrigJSSrc,RestJSSrc);
      end;

  finally
    RestJSModule.Free;
    RestConverter.Free;
    FreeAndNil(FRestAnalyzer);
    RestParser.Free;
    RestScanner.Free;
    if (RestResolver<>nil) and (RestResolver.RootElement<>nil) then
      begin
      RestResolver.RootElement.ReleaseUsedUnits;
      RestResolver.RootElement.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      end;
    RestResolver.Free; // free parser before resolver
    RestFileResolver.Free;

    ms.Free;
  end;
end;

procedure TCustomTestPrecompile.StartParsing;
begin
  inherited StartParsing;
  FInitialFlags.ParserOptions:=Parser.Options;
  FInitialFlags.ModeSwitches:=Scanner.CurrentModeSwitches;
  FInitialFlags.BoolSwitches:=Scanner.CurrentBoolSwitches;
  FInitialFlags.ConverterOptions:=Converter.Options;
  FInitialFlags.TargetPlatform:=Converter.Globals.TargetPlatform;
  FInitialFlags.TargetProcessor:=Converter.Globals.TargetProcessor;
  // ToDo: defines
end;

function TCustomTestPrecompile.CheckRestoredObject(const Path: string; Orig,
  Rest: TObject): boolean;
begin
  if Orig=nil then
    begin
    if Rest<>nil then
      Fail(Path+': Orig=nil Rest='+GetObjPath(Rest));
    exit(false);
    end
  else if Rest=nil then
    Fail(Path+': Orig='+GetObjPath(Orig)+' Rest=nil');
  if Orig.ClassType<>Rest.ClassType then
    Fail(Path+': Orig='+GetObjPath(Orig)+' Rest='+GetObjPath(Rest));
  Result:=true;
end;

procedure TCustomTestPrecompile.CheckRestoredJS(const Path, Orig, Rest: string);
var
  OrigList, RestList: TStringList;
begin
  if Orig=Rest then exit;
  writeln('TCustomTestPrecompile.CheckRestoredJS ORIG START--------------');
  writeln(Orig);
  writeln('TCustomTestPrecompile.CheckRestoredJS ORIG END----------------');
  writeln('TCustomTestPrecompile.CheckRestoredJS REST START--------------');
  writeln(Rest);
  writeln('TCustomTestPrecompile.CheckRestoredJS REST END----------------');
  OrigList:=TStringList.Create;
  RestList:=TStringList.Create;
  try
    OrigList.Text:=Orig;
    RestList.Text:=Rest;
    CheckRestoredStringList(Path,OrigList,RestList);
 finally
    OrigList.Free;
    RestList.Free;
  end;
end;

procedure TCustomTestPrecompile.CheckRestoredStringList(const Path: string;
  Orig, Rest: TStrings);
var
  i: Integer;
begin
  CheckRestoredObject(Path,Orig,Rest);
  if Orig=nil then exit;
  if Orig.Text=Rest.Text then exit;
  for i:=0 to Orig.Count-1 do
    begin
    if i>=Rest.Count then
      Fail(Path+' missing: '+Orig[i]);
    writeln('  ',i,': '+Orig[i]);
    end;
  if Orig.Count<Rest.Count then
    Fail(Path+' too much: '+Rest[Orig.Count]);
end;

procedure TCustomTestPrecompile.CheckRestoredResolver(Original,
  Restored: TPas2JSResolver; Flags: TPCCheckFlags);
var
  OrigParser, RestParser: TPasParser;
begin
  AssertNotNull('CheckRestoredResolver Original',Original);
  AssertNotNull('CheckRestoredResolver Restored',Restored);
  if Original.ClassType<>Restored.ClassType then
    Fail('CheckRestoredResolver Original='+Original.ClassName+' Restored='+Restored.ClassName);
  CheckRestoredElement('RootElement',Original.RootElement,Restored.RootElement,Flags);
  OrigParser:=Original.CurrentParser;
  RestParser:=Restored.CurrentParser;
  if OrigParser.Options<>RestParser.Options then
    Fail('CheckRestoredResolver Parser.Options');
  if OrigParser.Scanner.CurrentBoolSwitches<>RestParser.Scanner.CurrentBoolSwitches then
    Fail('CheckRestoredResolver Scanner.BoolSwitches');
  if OrigParser.Scanner.CurrentModeSwitches<>RestParser.Scanner.CurrentModeSwitches then
    Fail('CheckRestoredResolver Scanner.ModeSwitches');
end;

procedure TCustomTestPrecompile.CheckRestoredDeclarations(const Path: string;
  Orig, Rest: TPasDeclarations; Flags: TPCCheckFlags);

  function IsSpecialization(El: TPasElement): boolean;
  begin
    Result:=(El.CustomData is TPasGenericScope)
        and (TPasGenericScope(El.CustomData).SpecializedFromItem<>nil);
  end;

  function GetSubPath(const Path: string; OrigIndex: integer; OrigDecl: TPasElement): string;
  begin
    Result:=Path+'['+IntToStr(OrigIndex)+']';
    if OrigDecl.Name<>'' then
      Result:=Result+'"'+OrigDecl.Name+'"'
    else
      Result:=Result+'?noname?';
  end;

{  procedure WriteList;
  var
    i: Integer;
  begin
    writeln('CheckRestoredDeclarations.WriteList');
    for i:=0 to Orig.Declarations.Count-1 do
      if i<Rest.Declarations.Count then
        writeln('  ',i,' Orig=',TPasElement(Orig.Declarations[i]).Name,' Rest=',TPasElement(Rest.Declarations[i]).Name);
  end;}

var
  OrigIndex, RestIndex: Integer;
  OrigDecl, RestDecl: TPasElement;
  SubPath: String;
begin
  //WriteList;
  // check non specializations
  RestIndex:=0;
  for OrigIndex:=0 to Orig.Declarations.Count-1 do
    begin
    OrigDecl:=TPasElement(Orig.Declarations[OrigIndex]);
    if IsSpecialization(OrigDecl) then
      continue;
    SubPath:=GetSubPath(Path,OrigIndex,OrigDecl);
    // skip to next non specialization in restored declarations
    while RestIndex<Rest.Declarations.Count do
      begin
      RestDecl:=TPasElement(Rest.Declarations[RestIndex]);
      if not IsSpecialization(RestDecl) then
        break;
      inc(RestIndex)
      end;
    if RestIndex=Rest.Declarations.Count then
      Fail(SubPath+' missing in restored Declarations');
    // check
    CheckRestoredElement(SubPath,OrigDecl,RestDecl,Flags);
    inc(RestIndex);
    end;

  // check specializations
  for OrigIndex:=0 to Orig.Declarations.Count-1 do
    begin
    OrigDecl:=TPasElement(Orig.Declarations[OrigIndex]);
    if not IsSpecialization(OrigDecl) then
      continue;
    SubPath:=GetSubPath(Path,OrigIndex,OrigDecl);
    // search specialization with same name
    RestIndex:=0;
    repeat
      if RestIndex=Rest.Declarations.Count then
        Fail(SubPath+' missing in restored Declarations');
      RestDecl:=TPasElement(Rest.Declarations[RestIndex]);
      if IsSpecialization(RestDecl) and (OrigDecl.Name=RestDecl.Name) then
        break;
      inc(RestIndex);
    until false;

    if (OrigIndex<Rest.Declarations.Count) and (OrigIndex<>RestIndex) then
      begin
      // move restored element to original place to generate the same JS
      //writeln('TCustomTestPrecompile.CheckRestoredDeclarations Orig[',OrigIndex,']=',GetObjName(OrigDecl),' Rest[',RestIndex,']=',GetObjName(RestDecl));
      if RestIndex>OrigIndex then
        Rest.Declarations.Move(RestIndex,OrigIndex)
      else
        Rest.Declarations.Exchange(RestIndex,OrigIndex);
      //writeln('TCustomTestPrecompile.CheckRestoredDeclarations RestIndex=',RestIndex,' ->',OrigIndex);
      //WriteList;
      end;

    // check
    CheckRestoredElement(SubPath,OrigDecl,RestDecl,Flags);
    end;
  AssertEquals(Path+'.Declarations.Count',Orig.Declarations.Count,Rest.Declarations.Count);

  //WriteList;
  for OrigIndex:=0 to Orig.Declarations.Count-1 do
    begin
    OrigDecl:=TPasElement(Orig.Declarations[OrigIndex]);
    RestDecl:=TPasElement(Rest.Declarations[OrigIndex]);
    if OrigDecl.Name<>RestDecl.Name then
      begin
      SubPath:=GetSubPath(Path,OrigIndex,OrigDecl);
      AssertEquals(SubPath+'.Name',GetObjPath(OrigDecl),GetObjPath(RestDecl));
      end;
    end;
end;

procedure TCustomTestPrecompile.CheckRestoredSection(const Path: string; Orig,
  Rest: TPasSection; Flags: TPCCheckFlags);
begin
  if length(Orig.UsesClause)>0 then
    ; // ToDo
  CheckRestoredDeclarations(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredModule(const Path: string; Orig,
  Rest: TPasModule; Flags: TPCCheckFlags);

  procedure CheckInitFinal(const Path: string; OrigBlock, RestBlock: TPasImplBlock);
  begin
    CheckRestoredObject(Path,OrigBlock,RestBlock);
    if OrigBlock=nil then exit;
    CheckRestoredCustomData(Path+'.CustomData',RestBlock,OrigBlock.CustomData,RestBlock.CustomData,Flags);
  end;

begin
  if not (Orig.CustomData is TPas2JSModuleScope) then
    Fail(Path+'.CustomData is not TPasModuleScope'+GetObjName(Orig.CustomData));

  CheckRestoredElement(Path+'.InterfaceSection',Orig.InterfaceSection,Rest.InterfaceSection,Flags);
  CheckRestoredElement(Path+'.ImplementationSection',Orig.ImplementationSection,Rest.ImplementationSection,Flags);
  if Orig is TPasProgram then
    CheckRestoredElement(Path+'.ProgramSection',TPasProgram(Orig).ProgramSection,TPasProgram(Rest).ProgramSection,Flags)
  else if Orig is TPasLibrary then
    CheckRestoredElement(Path+'.LibrarySection',TPasLibrary(Orig).LibrarySection,TPasLibrary(Rest).LibrarySection,Flags);

  CheckInitFinal(Path+'.InitializationSection',Orig.InitializationSection,Rest.InitializationSection);
  CheckInitFinal(Path+'.FnializationSection',Orig.FinalizationSection,Rest.FinalizationSection);
end;

procedure TCustomTestPrecompile.CheckRestoredScopeReference(const Path: string;
  Orig, Rest: TPasScope; Flags: TPCCheckFlags);
begin
  if not CheckRestoredObject(Path,Orig,Rest) then exit;
  CheckRestoredReference(Path+'.Element',Orig.Element,Rest.Element);
  if Flags=[] then ;
end;

procedure TCustomTestPrecompile.CheckRestoredElementBase(const Path: string;
  Orig, Rest: TPasElementBase; Flags: TPCCheckFlags);
begin
  CheckRestoredObject(Path+'.CustomData',Orig.CustomData,Rest.CustomData);
  if Flags=[] then ;
end;

procedure TCustomTestPrecompile.CheckRestoredResolveData(const Path: string;
  Orig, Rest: TResolveData; Flags: TPCCheckFlags);
begin
  CheckRestoredElementBase(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredPasScope(const Path: string; Orig,
  Rest: TPasScope; Flags: TPCCheckFlags);
begin
  CheckRestoredReference(Path+'.VisibilityContext',Orig.VisibilityContext,Rest.VisibilityContext);
  CheckRestoredResolveData(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredLocalVar(const Path: string; Orig,
  Rest: TPas2JSStoredLocalVar);
begin
  AssertEquals(Path+'.Name',Orig.Name,Rest.Name);
  CheckRestoredReference(Path+'.Id',Orig.Element,Rest.Element);
end;

procedure TCustomTestPrecompile.CheckRestoredModuleScope(const Path: string;
  Orig, Rest: TPas2JSModuleScope; Flags: TPCCheckFlags);
var
  OrigLocalVars, RestLocalVars: TPas2JSStoredLocalVarArray;
  i, j: Integer;
  OrigLocalVar, RestLocalVar: TPas2JSStoredLocalVar;
begin
  AssertEquals(Path+'.FirstName',Orig.FirstName,Rest.FirstName);
  if Orig.Flags<>Rest.Flags then
    Fail(Path+'.Flags');
  if Orig.BoolSwitches<>Rest.BoolSwitches then
    Fail(Path+'.BoolSwitches');
  CheckRestoredReference(Path+'.AssertClass',Orig.AssertClass,Rest.AssertClass);
  CheckRestoredReference(Path+'.AssertDefConstructor',Orig.AssertDefConstructor,Rest.AssertDefConstructor);
  CheckRestoredReference(Path+'.AssertMsgConstructor',Orig.AssertMsgConstructor,Rest.AssertMsgConstructor);
  CheckRestoredReference(Path+'.RangeErrorClass',Orig.RangeErrorClass,Rest.RangeErrorClass);
  CheckRestoredReference(Path+'.RangeErrorConstructor',Orig.RangeErrorConstructor,Rest.RangeErrorConstructor);
  CheckRestoredReference(Path+'.SystemTVarRec',Orig.SystemTVarRec,Rest.SystemTVarRec);
  CheckRestoredReference(Path+'.SystemVarRecs',Orig.SystemVarRecs,Rest.SystemVarRecs);

  // StoreJSLocalVars
  OrigLocalVars:=Orig.StoreJSLocalVars;
  RestLocalVars:=Rest.StoreJSLocalVars;
  //for i:=0 to length(RestLocalVars)-1 do
  //  writeln('TCustomTestPrecompile.CheckRestoredModuleScope Rest ',i,'/',length(RestLocalVars),' ',RestLocalVars[i].Name);
  for i:=0 to length(OrigLocalVars)-1 do
  begin
    OrigLocalVar:=OrigLocalVars[i];
    //writeln('TCustomTestPrecompile.CheckRestoredModuleScope Orig ',i,'/',length(OrigLocalVars),' ',OrigLocalVar.Name);
    j:=length(OrigLocalVars)-1;
    while (j>=0) do
      begin
      RestLocalVar:=RestLocalVars[j];
      if OrigLocalVar.Name=RestLocalVar.Name then
        begin
        CheckRestoredLocalVar(Path+'.LocalVars['+IntToStr(i)+']',OrigLocalVar,RestLocalVar);
        break;
        end;
      dec(j);
      end;
    if j<0 then
      Fail(Path+'.LocalVars['+IntToStr(i)+'] Name="'+OrigLocalVar.Name+'" missing in Rest');
  end;
  AssertEquals('LocalVars.Count',length(OrigLocalVars),length(RestLocalVars));

  CheckRestoredPasScope(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredIdentifierScope(
  const Path: string; Orig, Rest: TPasIdentifierScope; Flags: TPCCheckFlags);
var
  OrigList: TFPList;
  i: Integer;
  OrigIdentifier, RestIdentifier: TPasIdentifier;
begin
  OrigList:=nil;
  try
    OrigList:=Orig.GetLocalIdentifiers;
    for i:=0 to OrigList.Count-1 do
    begin
      OrigIdentifier:=TPasIdentifier(OrigList[i]);
      RestIdentifier:=Rest.FindLocalIdentifier(OrigIdentifier.Identifier);
      if RestIdentifier=nil then
        Fail(Path+'.Local['+OrigIdentifier.Identifier+'] Missing RestIdentifier Orig='+OrigIdentifier.Identifier);
      repeat
        AssertEquals(Path+'.Local.Identifier',OrigIdentifier.Identifier,RestIdentifier.Identifier);
        CheckRestoredReference(Path+'.Local',OrigIdentifier.Element,RestIdentifier.Element);
        if OrigIdentifier.Kind<>RestIdentifier.Kind then
          Fail(Path+'.Local['+OrigIdentifier.Identifier+'] Orig='+PCUIdentifierKindNames[OrigIdentifier.Kind]+' Rest='+PCUIdentifierKindNames[RestIdentifier.Kind]);
        if OrigIdentifier.NextSameIdentifier=nil then
        begin
          if RestIdentifier.NextSameIdentifier<>nil then
            Fail(Path+'.Local['+OrigIdentifier.Identifier+'] Too many RestIdentifier.NextSameIdentifier='+GetObjName(RestIdentifier.Element));
          break;
        end
        else begin
          if RestIdentifier.NextSameIdentifier=nil then
            Fail(Path+'.Local['+OrigIdentifier.Identifier+'] Missing RestIdentifier.NextSameIdentifier Orig='+GetObjName(OrigIdentifier.NextSameIdentifier.Element));
        end;
        if CompareText(OrigIdentifier.Identifier,OrigIdentifier.NextSameIdentifier.Identifier)<>0 then
          Fail(Path+'.Local['+OrigIdentifier.Identifier+'] Cur.Identifier<>Next.Identifier '+OrigIdentifier.Identifier+'<>'+OrigIdentifier.NextSameIdentifier.Identifier);
        OrigIdentifier:=OrigIdentifier.NextSameIdentifier;
        RestIdentifier:=RestIdentifier.NextSameIdentifier;
      until false;
    end;
  finally
    OrigList.Free;
  end;
  CheckRestoredPasScope(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredSectionScope(const Path: string;
  Orig, Rest: TPas2JSSectionScope; Flags: TPCCheckFlags);
var
  i: Integer;
  OrigUses, RestUses: TPas2JSSectionScope;
  OrigHelperEntry, RestHelperEntry: TPRHelperEntry;
begin
  if Orig.BoolSwitches<>Rest.BoolSwitches then
    Fail(Path+'.BoolSwitches Orig='+BoolSwitchesToStr(Orig.BoolSwitches)+' Rest='+BoolSwitchesToStr(Rest.BoolSwitches));
  if Orig.ModeSwitches<>Rest.ModeSwitches then
    Fail(Path+'.ModeSwitches');
  AssertEquals(Path+' UsesScopes.Count',Orig.UsesScopes.Count,Rest.UsesScopes.Count);
  for i:=0 to Orig.UsesScopes.Count-1 do
    begin
    OrigUses:=TPas2JSSectionScope(Orig.UsesScopes[i]);
    if not (TObject(Rest.UsesScopes[i]) is TPas2JSSectionScope) then
      Fail(Path+'.UsesScopes['+IntToStr(i)+'] Rest='+GetObjName(TObject(Rest.UsesScopes[i])));
    RestUses:=TPas2JSSectionScope(Rest.UsesScopes[i]);
    if OrigUses.ClassType<>RestUses.ClassType then
      Fail(Path+'.UsesScopes['+IntToStr(i)+'] Orig='+GetObjName(OrigUses)+' Rest='+GetObjName(RestUses));
    CheckRestoredReference(Path+'.UsesScopes['+IntToStr(i)+']',OrigUses.Element,RestUses.Element);
    end;
  AssertEquals(Path+' length(Helpers)',length(Orig.Helpers),length(Rest.Helpers));
  for i:=0 to length(Orig.Helpers)-1 do
    begin
    OrigHelperEntry:=TPRHelperEntry(Orig.Helpers[i]);
    RestHelperEntry:=TPRHelperEntry(Rest.Helpers[i]);
    if OrigHelperEntry.ClassType<>RestHelperEntry.ClassType then
      Fail(Path+'.Helpers['+IntToStr(i)+'] Orig='+GetObjName(OrigHelperEntry)+' Rest='+GetObjName(RestHelperEntry));
    AssertEquals(Path+'.Helpers['+IntToStr(i)+'].Added',OrigHelperEntry.Added,RestHelperEntry.Added);
    CheckRestoredReference(Path+'.Helpers['+IntToStr(i)+'].Helper',OrigHelperEntry.Helper,RestHelperEntry.Helper);
    CheckRestoredReference(Path+'.Helpers['+IntToStr(i)+'].HelperForType',OrigHelperEntry.HelperForType,RestHelperEntry.HelperForType);
    end;

  AssertEquals(Path+'.Finished',Orig.Finished,Rest.Finished);
  CheckRestoredIdentifierScope(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredInitialFinalizationScope(
  const Path: string; Orig, Rest: TPas2JSInitialFinalizationScope;
  Flags: TPCCheckFlags);
begin
  CheckRestoredScopeRefs(Path+'.References',Orig.References,Rest.References,Flags);
  CheckRestoredPrecompiledJS(Path+'.ImplJS',Orig.Element,Orig.ImplJS,Rest.Element,Rest.ImplJS,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredEnumTypeScope(const Path: string;
  Orig, Rest: TPasEnumTypeScope; Flags: TPCCheckFlags);
begin
  CheckRestoredReference(Path+'.CanonicalSet',Orig.CanonicalSet,Rest.CanonicalSet);
  CheckRestoredIdentifierScope(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredRecordScope(const Path: string;
  Orig, Rest: TPas2jsRecordScope; Flags: TPCCheckFlags);
begin
  CheckRestoredReference(Path+'.DefaultProperty',Orig.DefaultProperty,Rest.DefaultProperty);
  CheckRestoredIdentifierScope(Path,Orig,Rest,Flags);
  // ok -> use same JSName
  Rest.JSName:=Orig.JSName;
end;

procedure TCustomTestPrecompile.CheckRestoredClassScope(const Path: string;
  Orig, Rest: TPas2JSClassScope; Flags: TPCCheckFlags);
var
  i, j: Integer;
  OrigObj, RestObj: TObject;
  OrigMap, RestMap: TPasClassIntfMap;
  SubPath: String;
begin
  CheckRestoredScopeReference(Path+'.AncestorScope',Orig.AncestorScope,Rest.AncestorScope,Flags);
  CheckRestoredElement(Path+'.CanonicalClassOf',Orig.CanonicalClassOf,Rest.CanonicalClassOf,Flags);
  CheckRestoredReference(Path+'.DirectAncestor',Orig.DirectAncestor,Rest.DirectAncestor);
  CheckRestoredReference(Path+'.DefaultProperty',Orig.DefaultProperty,Rest.DefaultProperty);
  if Orig.Flags<>Rest.Flags then
    Fail(Path+'.Flags');
  AssertEquals(Path+'.AbstractProcs.length',length(Orig.AbstractProcs),length(Rest.AbstractProcs));
  for i:=0 to length(Orig.AbstractProcs)-1 do
    CheckRestoredReference(Path+'.AbstractProcs['+IntToStr(i)+']',Orig.AbstractProcs[i],Rest.AbstractProcs[i]);

  CheckRestoredReference(Path+'.NewInstanceFunction',Orig.NewInstanceFunction,Rest.NewInstanceFunction);
  AssertEquals(Path+'.GUID',Orig.GUID,Rest.GUID);
  AssertEquals(Path+'.DispatchField',Orig.DispatchField,Rest.DispatchField);
  AssertEquals(Path+'.DispatchStrField',Orig.DispatchStrField,Rest.DispatchStrField);

  CheckRestoredObject('.Interfaces',Orig.Interfaces,Rest.Interfaces);
  if Orig.Interfaces<>nil then
    begin
    AssertEquals(Path+'.Interfaces.Count',Orig.Interfaces.Count,Rest.Interfaces.Count);
    for i:=0 to Orig.Interfaces.Count-1 do
      begin
      SubPath:=Path+'.Interfaces['+IntToStr(i)+']';
      OrigObj:=TObject(Orig.Interfaces[i]);
      RestObj:=TObject(Rest.Interfaces[i]);
      CheckRestoredObject(SubPath,OrigObj,RestObj);
      if OrigObj is TPasProperty then
        CheckRestoredReference(SubPath+'(TPasProperty)',
          TPasProperty(OrigObj),TPasProperty(RestObj))
      else if OrigObj is TPasClassIntfMap then
        begin
        OrigMap:=TPasClassIntfMap(OrigObj);
        RestMap:=TPasClassIntfMap(RestObj);
        repeat
          AssertNotNull(SubPath+'.Intf Orig',OrigMap.Intf);
          CheckRestoredObject(SubPath+'.Intf',OrigMap.Intf,RestMap.Intf);
          SubPath:=SubPath+'.Map('+OrigMap.Intf.Name+')';
          CheckRestoredObject(SubPath+'.Element',OrigMap.Element,RestMap.Element);
          CheckRestoredObject(SubPath+'.Procs',OrigMap.Procs,RestMap.Procs);
          if OrigMap.Procs=nil then
            begin
            if OrigMap.Intf.Members.Count>0 then
              Fail(SubPath+' expected '+IntToStr(OrigMap.Intf.Members.Count)+' procs, but Procs=nil');
            end
          else
            for j:=0 to OrigMap.Procs.Count-1 do
              begin
              OrigObj:=TObject(OrigMap.Procs[j]);
              RestObj:=TObject(RestMap.Procs[j]);
              CheckRestoredReference(SubPath+'.Procs['+IntToStr(j)+']',TPasElement(OrigObj),TPasElement(RestObj));
              end;
          AssertEquals(Path+'.Procs.Count',OrigMap.Procs.Count,RestMap.Procs.Count);

          CheckRestoredObject(SubPath+'.AncestorMap',OrigMap.AncestorMap,RestMap.AncestorMap);
          OrigMap:=OrigMap.AncestorMap;
          RestMap:=RestMap.AncestorMap;
        until OrigMap=nil;
        end
      else
        Fail(SubPath+' unknown class '+GetObjName(OrigObj));
      end;
    end;

  CheckRestoredIdentifierScope(Path,Orig,Rest,Flags);

  // ok -> use same JSName
  Rest.JSName:=Orig.JSName;
end;

procedure TCustomTestPrecompile.CheckRestoredProcScope(const Path: string;
  Orig, Rest: TPas2JSProcedureScope; Flags: TPCCheckFlags);
var
  DeclProc: TPasProcedure;
begin
  CheckRestoredReference(Path+'.DeclarationProc',Orig.DeclarationProc,Rest.DeclarationProc);
  CheckRestoredReference(Path+'.ImplProc',Orig.ImplProc,Rest.ImplProc);
  CheckRestoredPrecompiledJS(Path+'.ImplJS',Orig.Element,Orig.ImplJS,Rest.Element,Rest.ImplJS,Flags);

  if Rest.DeclarationProc=nil then
    begin
    DeclProc:=TPasProcedure(Rest.Element);
    AssertEquals(Path+'.ResultVarName',Orig.ResultVarName,Rest.ResultVarName);
    CheckRestoredReference(Path+'.OverriddenProc',Orig.OverriddenProc,Rest.OverriddenProc);

    CheckRestoredScopeReference(Path+'.ClassScope',Orig.ClassRecScope,Rest.ClassRecScope,Flags);
    CheckRestoredElement(Path+'.SelfArg',Orig.SelfArg,Rest.SelfArg,Flags);
    if Orig.Flags<>Rest.Flags then
      Fail(Path+'.Flags');
    if Orig.BoolSwitches<>Rest.BoolSwitches then
      Fail(Path+'.BoolSwitches');
    if Orig.ModeSwitches<>Rest.ModeSwitches then
      Fail(Path+'.ModeSwitches');

    if Engine.ProcCanBePrecompiled(DeclProc) then
      begin
      CheckRestoredScopeRefs(Path+'.References',Orig.References,Rest.References,Flags);
      end;
    //CheckRestoredIdentifierScope(Path,Orig,Rest);
    end
  else
    begin
    // ImplProc
    end;

  // ok -> use same JSName
  Rest.JSName:=Orig.JSName;
end;

procedure TCustomTestPrecompile.CheckRestoredProcTypeScope(const Path: string;
  Orig, Rest: TPas2JSProcTypeScope; Flags: TPCCheckFlags);
begin
  if Path='' then ;
  if Flags=[] then ;

  // ok -> use same JSName
  Rest.JSName:=Orig.JSName;
end;

procedure TCustomTestPrecompile.CheckRestoredArrayScope(const Path: string;
  Orig, Rest: TPas2JSArrayScope; Flags: TPCCheckFlags);
begin
  if Path='' then ;
  if Flags=[] then ;

  // ok -> use same JSName
  Rest.JSName:=Orig.JSName;
end;

procedure TCustomTestPrecompile.CheckRestoredPrecompiledJS(const Path: string;
  OrigEl: TPasElement; Orig: TPas2JSPrecompiledJS; RestEl: TPasElement;
  Rest: TPas2JSPrecompiledJS; Flags: TPCCheckFlags);
begin
  CheckRestoredObject(Path,Orig,Rest);
  if Orig=nil then exit;
  if Flags=[] then ;

  AssertEquals(Path+'.EmptyJS',Orig.EmptyJS,Rest.EmptyJS);
  if Orig.BodyJS<>Rest.BodyJS then
    CheckRestoredJS(Path+'.BodyJS',Orig.BodyJS,Rest.BodyJS);
  if Orig.BodyJS<>'' then
    begin
    CheckRestoredStringList(Path+'.GlobalJS',Orig.GlobalJS,Rest.GlobalJS);
    CheckRestoredElRefList(Path+'.ShortRefs',OrigEl,Orig.ShortRefs,RestEl,Rest.ShortRefs,false,Flags);
    end;
end;

procedure TCustomTestPrecompile.CheckRestoredScopeRefs(const Path: string;
  Orig, Rest: TPasScopeReferences; Flags: TPCCheckFlags);
var
  OrigList, RestList: TFPList;
  i: Integer;
  OrigRef, RestRef: TPasScopeReference;
  ok: Boolean;
begin
  if Flags=[] then ;
  CheckRestoredObject(Path,Orig,Rest);
  if Orig=nil then exit;
  OrigList:=nil;
  RestList:=nil;
  ok:=false;
  try
    OrigList:=Orig.GetList;
    RestList:=Rest.GetList;
    OrigList.Sort(@CompareListOfProcScopeRef);
    RestList.Sort(@CompareListOfProcScopeRef);
    for i:=0 to OrigList.Count-1 do
      begin
      OrigRef:=TPasScopeReference(OrigList[i]);
      if i>=RestList.Count then
        Fail(Path+'['+IntToStr(i)+'] Missing in Rest: "'+OrigRef.Element.Name+'"');
      RestRef:=TPasScopeReference(RestList[i]);
      CheckRestoredReference(Path+'['+IntToStr(i)+'].Name="'+OrigRef.Element.Name+'"',OrigRef.Element,RestRef.Element);
      if OrigRef.Access<>RestRef.Access then
        AssertEquals(Path+'['+IntToStr(i)+']"'+OrigRef.Element.Name+'".Access',
          PCUPSRefAccessNames[OrigRef.Access],PCUPSRefAccessNames[RestRef.Access]);
      end;
    if RestList.Count>OrigList.Count then
      begin
      i:=OrigList.Count;
      RestRef:=TPasScopeReference(RestList[i]);
      Fail(Path+'['+IntToStr(i)+'] Too many in Rest: "'+RestRef.Element.Name+'"');
      end;
    ok:=true;
  finally
    if not ok then
      begin
      for i:=0 to OrigList.Count-1 do
        begin
        OrigRef:=TPasScopeReference(OrigList[i]);
        writeln('TCustomTestPrecompile.CheckRestoredScopeRefs Orig[',i,']=',GetObjPath(OrigRef.Element));
        end;
      for i:=0 to RestList.Count-1 do
        begin
        RestRef:=TPasScopeReference(RestList[i]);
        writeln('TCustomTestPrecompile.CheckRestoredScopeRefs Rest[',i,']=',GetObjPath(RestRef.Element));
        end;
      end;
    OrigList.Free;
    RestList.Free;
  end;
end;

procedure TCustomTestPrecompile.CheckRestoredPropertyScope(const Path: string;
  Orig, Rest: TPasPropertyScope; Flags: TPCCheckFlags);
begin
  CheckRestoredReference(Path+'.AncestorProp',Orig.AncestorProp,Rest.AncestorProp);
  CheckRestoredIdentifierScope(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredGenericParamScope(
  const Path: string; Orig, Rest: TPasGenericParamsScope; Flags: TPCCheckFlags);
begin
  // Orig.GenericType only needed during parsing
  if Path='' then ;
  if Orig<>nil then ;
  if Rest<>nil then ;
  if Flags=[] then ;
end;

procedure TCustomTestPrecompile.CheckRestoredSpecializeTypeData(
  const Path: string; Orig, Rest: TPasSpecializeTypeData; Flags: TPCCheckFlags);
begin
  if Flags<>[] then ;
  CheckRestoredReference(Path+'.SpecializedType',Orig.SpecializedType,Rest.SpecializedType);
end;

procedure TCustomTestPrecompile.CheckRestoredResolvedReference(
  const Path: string; Orig, Rest: TResolvedReference; Flags: TPCCheckFlags);
var
  C: TClass;
begin
  if Orig.Flags<>Rest.Flags then
    Fail(Path+'.Flags');
  if Orig.Access<>Rest.Access then
    AssertEquals(Path+'.Access',PCUResolvedRefAccessNames[Orig.Access],PCUResolvedRefAccessNames[Rest.Access]);
  if not CheckRestoredObject(Path+'.Context',Orig.Context,Rest.Context) then exit;
  if Orig.Context<>nil then
    begin
    C:=Orig.Context.ClassType;
    if C=TResolvedRefCtxConstructor then
      CheckRestoredReference(Path+'.Context[TResolvedRefCtxConstructor].Typ',
        TResolvedRefCtxConstructor(Orig.Context).Typ,
        TResolvedRefCtxConstructor(Rest.Context).Typ);
    end;
  CheckRestoredScopeReference(Path+'.WithExprScope',Orig.WithExprScope,Rest.WithExprScope,Flags);
  CheckRestoredReference(Path+'.Declaration',Orig.Declaration,Rest.Declaration);

  CheckRestoredResolveData(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredEvalValue(const Path: string;
  Orig, Rest: TResEvalValue);
var
  i: Integer;
begin
  if not CheckRestoredObject(Path,Orig,Rest) then exit;
  if Orig.Kind<>Rest.Kind then
    Fail(Path+'.Kind');
  if not CheckRestoredObject(Path+'.Element',Orig.Element,Rest.Element) then exit;
  CheckRestoredReference(Path+'.IdentEl',Orig.IdentEl,Rest.IdentEl);
  case Orig.Kind of
    revkNone: Fail(Path+'.Kind=revkNone');
    revkCustom: Fail(Path+'.Kind=revkNone');
    revkNil: ;
    revkBool: AssertEquals(Path+'.B',TResEvalBool(Orig).B,TResEvalBool(Rest).B);
    revkInt: AssertEquals(Path+'.Int',TResEvalInt(Orig).Int,TResEvalInt(Rest).Int);
    revkUInt:
      if TResEvalUInt(Orig).UInt<>TResEvalUInt(Rest).UInt then
        Fail(Path+'.UInt');
    revkFloat: AssertEquals(Path+'.FloatValue',TResEvalFloat(Orig).FloatValue,TResEvalFloat(Rest).FloatValue);
    revkString: AssertEquals(Path+'.S,Raw',TResEvalString(Orig).S,TResEvalString(Rest).S);
    revkUnicodeString: AssertEquals(Path+'.S,UTF16',String(TResEvalUTF16(Orig).S),String(TResEvalUTF16(Rest).S));
    revkEnum:
      begin
      AssertEquals(Path+'.Index',TResEvalEnum(Orig).Index,TResEvalEnum(Rest).Index);
      CheckRestoredReference(Path+'.ElType',TResEvalEnum(Orig).ElType,TResEvalEnum(Rest).ElType);
      end;
    revkRangeInt:
      begin
      if TResEvalRangeInt(Orig).ElKind<>TResEvalRangeInt(Rest).ElKind then
        Fail(Path+'.Int/ElKind');
      CheckRestoredReference(Path+'.Int/ElType',TResEvalRangeInt(Orig).ElType,TResEvalRangeInt(Rest).ElType);
      AssertEquals(Path+'.Int/RangeStart',TResEvalRangeInt(Orig).RangeStart,TResEvalRangeInt(Rest).RangeStart);
      AssertEquals(Path+'.Int/RangeEnd',TResEvalRangeInt(Orig).RangeEnd,TResEvalRangeInt(Rest).RangeEnd);
      end;
    revkRangeUInt:
      begin
      if TResEvalRangeUInt(Orig).RangeStart<>TResEvalRangeUInt(Rest).RangeStart then
        Fail(Path+'.UInt/RangeStart');
      if TResEvalRangeUInt(Orig).RangeEnd<>TResEvalRangeUInt(Rest).RangeEnd then
        Fail(Path+'.UInt/RangeEnd');
      end;
    revkSetOfInt:
      begin
      if TResEvalSet(Orig).ElKind<>TResEvalSet(Rest).ElKind then
        Fail(Path+'.SetInt/ElKind');
      CheckRestoredReference(Path+'.SetInt/ElType',TResEvalSet(Orig).ElType,TResEvalSet(Rest).ElType);
      AssertEquals(Path+'.SetInt/RangeStart',TResEvalSet(Orig).RangeStart,TResEvalSet(Rest).RangeStart);
      AssertEquals(Path+'.SetInt/RangeEnd',TResEvalSet(Orig).RangeEnd,TResEvalSet(Rest).RangeEnd);
      AssertEquals(Path+'.SetInt/length(Items)',length(TResEvalSet(Orig).Ranges),length(TResEvalSet(Rest).Ranges));
      for i:=0 to length(TResEvalSet(Orig).Ranges)-1 do
        begin
        AssertEquals(Path+'.SetInt/Items['+IntToStr(i)+'].RangeStart',
          TResEvalSet(Orig).Ranges[i].RangeStart,TResEvalSet(Rest).Ranges[i].RangeStart);
        AssertEquals(Path+'.SetInt/Items['+IntToStr(i)+'].RangeEnd',
          TResEvalSet(Orig).Ranges[i].RangeEnd,TResEvalSet(Rest).Ranges[i].RangeEnd);
        end;
      end;
  end;
end;

procedure TCustomTestPrecompile.CheckRestoredCustomData(const Path: string;
  RestoredEl: TPasElement; Orig, Rest: TObject; Flags: TPCCheckFlags);
var
  C: TClass;
begin
  if PCCGeneric in Flags then
    begin
    if (Rest=nil) and (Orig<>nil) then
      begin
      C:=Orig.ClassType;
      if (C=TResolvedReference)
          or (C=TPasWithScope)
          or (C=TPas2JSWithExprScope)
          or (C=TPasForLoopScope)
          or (C=TPasExceptOnScope)
          or C.InheritsFrom(TResEvalValue) then
        exit
      else
        Fail(Path+': Generic Orig='+GetObjName(Orig)+' Rest=nil');
      end;
    end;

  if not CheckRestoredObject(Path,Orig,Rest) then exit;

  C:=Orig.ClassType;
  if C=TResolvedReference then
    CheckRestoredResolvedReference(Path+'[TResolvedReference]',TResolvedReference(Orig),TResolvedReference(Rest),Flags)
  else if C=TPas2JSModuleScope then
    CheckRestoredModuleScope(Path+'[TPas2JSModuleScope]',TPas2JSModuleScope(Orig),TPas2JSModuleScope(Rest),Flags)
  else if C=TPas2JSSectionScope then
    CheckRestoredSectionScope(Path+'[TPas2JSSectionScope]',TPas2JSSectionScope(Orig),TPas2JSSectionScope(Rest),Flags)
  else if C=TPas2JSInitialFinalizationScope then
    CheckRestoredInitialFinalizationScope(Path+'[TPas2JSInitialFinalizationScope]',TPas2JSInitialFinalizationScope(Orig),TPas2JSInitialFinalizationScope(Rest),Flags)
  else if C=TPasEnumTypeScope then
    CheckRestoredEnumTypeScope(Path+'[TPasEnumTypeScope]',TPasEnumTypeScope(Orig),TPasEnumTypeScope(Rest),Flags)
  else if C=TPas2jsRecordScope then
    CheckRestoredRecordScope(Path+'[TPas2jsRecordScope]',TPas2jsRecordScope(Orig),TPas2jsRecordScope(Rest),Flags)
  else if C=TPas2JSClassScope then
    CheckRestoredClassScope(Path+'[TPas2JSClassScope]',TPas2JSClassScope(Orig),TPas2JSClassScope(Rest),Flags)
  else if C=TPas2JSProcedureScope then
    CheckRestoredProcScope(Path+'[TPas2JSProcedureScope]',TPas2JSProcedureScope(Orig),TPas2JSProcedureScope(Rest),Flags)
  else if C=TPas2JSArrayScope then
    CheckRestoredArrayScope(Path+'[TPas2JSArrayScope]',TPas2JSArrayScope(Orig),TPas2JSArrayScope(Rest),Flags)
  else if C=TPas2JSProcTypeScope then
    CheckRestoredProcTypeScope(Path+'[TPas2JSProcTypeScope]',TPas2JSProcTypeScope(Orig),TPas2JSProcTypeScope(Rest),Flags)
  else if C=TPasPropertyScope then
    CheckRestoredPropertyScope(Path+'[TPasPropertyScope]',TPasPropertyScope(Orig),TPasPropertyScope(Rest),Flags)
  else if C=TPasGenericParamsScope then
    CheckRestoredGenericParamScope(Path+'[TPasGenericParamScope]',TPasGenericParamsScope(Orig),TPasGenericParamsScope(Rest),Flags)
  else if C=TPasSpecializeTypeData then
    CheckRestoredSpecializeTypeData(Path+'[TPasSpecializeTypeData]',TPasSpecializeTypeData(Orig),TPasSpecializeTypeData(Rest),Flags)
  else if C.InheritsFrom(TResEvalValue) then
    CheckRestoredEvalValue(Path+'['+Orig.ClassName+']',TResEvalValue(Orig),TResEvalValue(Rest))
  else
    Fail(Path+': unknown CustomData "'+GetObjName(Orig)+'" El='+GetObjName(RestoredEl));
end;

procedure TCustomTestPrecompile.CheckRestoredReference(const Path: string;
  Orig, Rest: TPasElement);
begin
  if not CheckRestoredObject(Path,Orig,Rest) then exit;
  AssertEquals(Path+'.Name',Orig.Name,Rest.Name);

  if Orig is TPasUnresolvedSymbolRef then
    exit; // compiler types and procs are the same in every unit -> skip checking unit

  CheckRestoredReference(Path+'.Parent',Orig.Parent,Rest.Parent);
end;

procedure TCustomTestPrecompile.CheckRestoredElOrRef(const Path: string; Orig,
  OrigProp, Rest, RestProp: TPasElement; Flags: TPCCheckFlags);
begin
  if not CheckRestoredObject(Path,OrigProp,RestProp) then exit;
  if Orig<>OrigProp.Parent then
    begin
    if Rest=RestProp.Parent then
      Fail(Path+' Orig "'+GetObjName(OrigProp)+'" is reference Orig.Parent='+GetObjName(Orig)+', Rest "'+GetObjName(RestProp)+'" is insitu');
    CheckRestoredReference(Path,OrigProp,RestProp);
    end
  else
    CheckRestoredElement(Path,OrigProp,RestProp,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredAnalyzerElement(
  const Path: string; Orig, Rest: TPasElement);
var
  OrigUsed, RestUsed: TPAElement;
begin
  //writeln('TCustomTestPrecompile.CheckRestoredAnalyzerElement ',GetObjName(RestAnalyzer));
  if RestAnalyzer=nil then exit;
  if Orig.ClassType=TPasArgument then exit;
  OrigUsed:=Analyzer.FindUsedElement(Orig);
  //writeln('TCustomTestPrecompile.CheckRestoredAnalyzerElement ',GetObjName(Orig),'=',OrigUsed<>nil,' ',GetObjName(Rest),'=',RestAnalyzer.FindUsedElement(Rest)<>nil);
  if OrigUsed<>nil then
    begin
    RestUsed:=RestAnalyzer.FindUsedElement(Rest);
    if RestUsed=nil then
      Fail(Path+': used in OrigAnalyzer, but not used in RestAnalyzer');
    if OrigUsed.Access<>RestUsed.Access then
      begin
      if (OrigUsed.Access in [paiaReadWrite,paiaWriteRead])
          and (RestUsed.Access in [paiaReadWrite,paiaWriteRead])
          and not (Orig.Parent is TProcedureBody) then
        // readwrite or writeread is irrelevant for globals
      else
        AssertEquals(Path+'->Analyzer.Access',dbgs(OrigUsed.Access),dbgs(RestUsed.Access));
      end;
    end
  else if RestAnalyzer.IsUsed(Rest) then
    begin
    Fail(Path+': not used in OrigAnalyzer, but used in RestAnalyzer');
    end;
end;

procedure TCustomTestPrecompile.CheckRestoredElement(const Path: string; Orig,
  Rest: TPasElement; Flags: TPCCheckFlags);
var
  C: TClass;
  AModule: TPasModule;
  Pair: TPCCheckedElementPair;
begin
  //writeln('TCustomTestPrecompile.CheckRestoredElement START Orig=',GetObjName(Orig),' Rest=',GetObjName(Rest));
  if not CheckRestoredObject(Path,Orig,Rest) then exit;
  //writeln('TCustomTestPrecompile.CheckRestoredElement CheckRestoredObject Orig=',GetObjName(Orig),' Rest=',GetObjName(Rest));

  Pair:=TPCCheckedElementPair(FCheckedElements.FindKey(Orig));
  if Pair<>nil then
    begin
    if Pair.Rest<>Rest then
      Fail(Path+': Orig='+GetObjPath(Orig)+' Rest='+GetObjPath(Rest));
    exit;
    end
  else
    begin
    Pair:=TPCCheckedElementPair.Create;
    Pair.Orig:=Orig;
    Pair.Rest:=Rest;
    FCheckedElements.Add(Pair,false);
    end;

  AModule:=Orig.GetModule;
  if AModule<>Module then
    begin
    if (Orig is TPasUnresolvedSymbolRef) then
      begin
      // built-in identifier
      if not SameText(Orig.Name,Rest.Name) then
        AssertEquals(Path+'.Name',Orig.Name,Rest.Name);
      if not CheckRestoredObject(Path+'.CustomData',Orig.CustomData,Rest.CustomData) then exit;
      exit;
      end;
    Fail(Path+' wrong module: Orig='+GetObjName(AModule)+' '+GetObjName(Module));
    end;

  AssertEquals(Path+'.Name',Orig.Name,Rest.Name);
  AssertEquals(Path+'.SourceFilename',Orig.SourceFilename,Rest.SourceFilename);
  AssertEquals(Path+'.SourceLinenumber',Orig.SourceLinenumber,Rest.SourceLinenumber);
  //AssertEquals(Path+'.SourceEndLinenumber',Orig.SourceEndLinenumber,Rest.SourceEndLinenumber);
  if Orig.Visibility<>Rest.Visibility then
    Fail(Path+'.Visibility '+PCUMemberVisibilityNames[Orig.Visibility]+' '+PCUMemberVisibilityNames[Rest.Visibility]);
  if Orig.Hints<>Rest.Hints then
    Fail(Path+'.Hints');
  AssertEquals(Path+'.HintMessage',Orig.HintMessage,Rest.HintMessage);

  //writeln('TCustomTestPrecompile.CheckRestoredElement Checking Parent... Orig=',GetObjName(Orig),' Rest=',GetObjName(Rest));
  CheckRestoredReference(Path+'.Parent',Orig.Parent,Rest.Parent);

  C:=Orig.ClassType;
  //writeln('TCustomTestPrecompile.CheckRestoredElement Checking CustomData... Orig=',GetObjName(Orig),' Rest=',GetObjName(Rest));
  if C=TPasGenericTemplateType then
    begin
    // TPasGenericParamsScope is only needed during parsing
    if Orig.CustomData=nil then
    else if not (Orig.CustomData is TPasGenericParamsScope) then
      Fail(Path+'Orig.CustomData='+GetObjName(Orig.CustomData))
    else if Rest.CustomData<>nil then
      CheckRestoredCustomData(Path+'.CustomData',Rest,Orig.CustomData,Rest.CustomData,Flags);
    end
  else
    CheckRestoredCustomData(Path+'.CustomData',Rest,Orig.CustomData,Rest.CustomData,Flags);

  if C=TUnaryExpr then
    CheckRestoredUnaryExpr(Path,TUnaryExpr(Orig),TUnaryExpr(Rest),Flags)
  else if C=TBinaryExpr then
    CheckRestoredBinaryExpr(Path,TBinaryExpr(Orig),TBinaryExpr(Rest),Flags)
  else if C=TPrimitiveExpr then
    CheckRestoredPrimitiveExpr(Path,TPrimitiveExpr(Orig),TPrimitiveExpr(Rest),Flags)
  else if C=TBoolConstExpr then
    CheckRestoredBoolConstExpr(Path,TBoolConstExpr(Orig),TBoolConstExpr(Rest),Flags)
  else if (C=TNilExpr)
      or (C=TInheritedExpr)
      or (C=TSelfExpr) then
    CheckRestoredPasExpr(Path,TPasExpr(Orig),TPasExpr(Rest),Flags)
  else if C=TParamsExpr then
    CheckRestoredParamsExpr(Path,TParamsExpr(Orig),TParamsExpr(Rest),Flags)
  else if C=TProcedureExpr then
    CheckRestoredProcedureExpr(Path,TProcedureExpr(Orig),TProcedureExpr(Rest),Flags)
  else if C=TRecordValues then
    CheckRestoredRecordValues(Path,TRecordValues(Orig),TRecordValues(Rest),Flags)
  else if C=TArrayValues then
    CheckRestoredArrayValues(Path,TArrayValues(Orig),TArrayValues(Rest),Flags)
  // TPasDeclarations is a base class
  // TPasUsesUnit is checked in usesclause
  // TPasSection is a base class
  else if C=TPasResString then
    CheckRestoredResString(Path,TPasResString(Orig),TPasResString(Rest),Flags)
  // TPasType is a base clas
  else if (C=TPasAliasType)
      or (C=TPasTypeAliasType)
      or (C=TPasClassOfType) then
    CheckRestoredAliasType(Path,TPasAliasType(Orig),TPasAliasType(Rest),Flags)
  else if C=TPasPointerType then
    CheckRestoredPointerType(Path,TPasPointerType(Orig),TPasPointerType(Rest),Flags)
  else if C=TPasSpecializeType then
    CheckRestoredSpecializedType(Path,TPasSpecializeType(Orig),TPasSpecializeType(Rest),Flags)
  else if C=TInlineSpecializeExpr then
    CheckRestoredInlineSpecializedExpr(Path,TInlineSpecializeExpr(Orig),TInlineSpecializeExpr(Rest),Flags)
  else if C=TPasGenericTemplateType then
    CheckRestoredGenericTemplateType(Path,TPasGenericTemplateType(Orig),TPasGenericTemplateType(Rest),Flags)
  else if C=TPasRangeType then
    CheckRestoredRangeType(Path,TPasRangeType(Orig),TPasRangeType(Rest),Flags)
  else if C=TPasArrayType then
    CheckRestoredArrayType(Path,TPasArrayType(Orig),TPasArrayType(Rest),Flags)
  else if C=TPasFileType then
    CheckRestoredFileType(Path,TPasFileType(Orig),TPasFileType(Rest),Flags)
  else if C=TPasEnumValue then
    CheckRestoredEnumValue(Path,TPasEnumValue(Orig),TPasEnumValue(Rest),Flags)
  else if C=TPasEnumType then
    CheckRestoredEnumType(Path,TPasEnumType(Orig),TPasEnumType(Rest),Flags)
  else if C=TPasSetType then
    CheckRestoredSetType(Path,TPasSetType(Orig),TPasSetType(Rest),Flags)
  else if C=TPasVariant then
    CheckRestoredVariant(Path,TPasVariant(Orig),TPasVariant(Rest),Flags)
  else if C=TPasRecordType then
    CheckRestoredRecordType(Path,TPasRecordType(Orig),TPasRecordType(Rest),Flags)
  else if C=TPasClassType then
    CheckRestoredClassType(Path,TPasClassType(Orig),TPasClassType(Rest),Flags)
  else if C=TPasArgument then
    CheckRestoredArgument(Path,TPasArgument(Orig),TPasArgument(Rest),Flags)
  else if C=TPasProcedureType then
    CheckRestoredProcedureType(Path,TPasProcedureType(Orig),TPasProcedureType(Rest),Flags)
  else if C=TPasResultElement then
    CheckRestoredResultElement(Path,TPasResultElement(Orig),TPasResultElement(Rest),Flags)
  else if C=TPasFunctionType then
    CheckRestoredFunctionType(Path,TPasFunctionType(Orig),TPasFunctionType(Rest),Flags)
  else if C=TPasStringType then
    CheckRestoredStringType(Path,TPasStringType(Orig),TPasStringType(Rest),Flags)
  else if C=TPasVariable then
    CheckRestoredVariable(Path,TPasVariable(Orig),TPasVariable(Rest),Flags)
  else if C=TPasExportSymbol then
    CheckRestoredExportSymbol(Path,TPasExportSymbol(Orig),TPasExportSymbol(Rest),Flags)
  else if C=TPasConst then
    CheckRestoredConst(Path,TPasConst(Orig),TPasConst(Rest),Flags)
  else if C=TPasProperty then
    CheckRestoredProperty(Path,TPasProperty(Orig),TPasProperty(Rest),Flags)
  else if C=TPasMethodResolution then
    CheckRestoredMethodResolution(Path,TPasMethodResolution(Orig),TPasMethodResolution(Rest),Flags)
  else if (C=TPasProcedure)
      or (C=TPasFunction)
      or (C=TPasConstructor)
      or (C=TPasClassConstructor)
      or (C=TPasDestructor)
      or (C=TPasClassDestructor)
      or (C=TPasClassProcedure)
      or (C=TPasClassFunction)
      then
    CheckRestoredProcedure(Path,TPasProcedure(Orig),TPasProcedure(Rest),Flags)
  else if (C=TPasOperator)
      or (C=TPasClassOperator) then
    CheckRestoredOperator(Path,TPasOperator(Orig),TPasOperator(Rest),Flags)
  else if (C=TPasImplCommand) then
    CheckRestoredImplCommand(Path,TPasImplCommand(Orig),TPasImplCommand(Rest),Flags)
  else if (C=TPasImplBeginBlock) then
    CheckRestoredImplBeginBlock(Path,TPasImplBeginBlock(Orig),TPasImplBeginBlock(Rest),Flags)
  else if (C=TPasImplAsmStatement) then
    CheckRestoredImplAsmStatement(Path,TPasImplAsmStatement(Orig),TPasImplAsmStatement(Rest),Flags)
  else if (C=TPasImplRepeatUntil) then
    CheckRestoredImplRepeatUntil(Path,TPasImplRepeatUntil(Orig),TPasImplRepeatUntil(Rest),Flags)
  else if (C=TPasImplIfElse) then
    CheckRestoredImplIfElse(Path,TPasImplIfElse(Orig),TPasImplIfElse(Rest),Flags)
  else if (C=TPasImplWhileDo) then
    CheckRestoredImplWhileDo(Path,TPasImplWhileDo(Orig),TPasImplWhileDo(Rest),Flags)
  else if (C=TPasImplWithDo) then
    CheckRestoredImplWithDo(Path,TPasImplWithDo(Orig),TPasImplWithDo(Rest),Flags)
  else if (C=TPasImplCaseOf) then
    CheckRestoredImplCaseOf(Path,TPasImplCaseOf(Orig),TPasImplCaseOf(Rest),Flags)
  else if (C=TPasImplCaseStatement) then
    CheckRestoredImplCaseStatement(Path,TPasImplCaseStatement(Orig),TPasImplCaseStatement(Rest),Flags)
  else if (C=TPasImplCaseElse) then
    CheckRestoredImplCaseElse(Path,TPasImplCaseElse(Orig),TPasImplCaseElse(Rest),Flags)
  else if (C=TPasImplForLoop) then
    CheckRestoredImplForLoop(Path,TPasImplForLoop(Orig),TPasImplForLoop(Rest),Flags)
  else if (C=TPasImplAssign) then
    CheckRestoredImplAssign(Path,TPasImplAssign(Orig),TPasImplAssign(Rest),Flags)
  else if (C=TPasImplSimple) then
    CheckRestoredImplSimple(Path,TPasImplSimple(Orig),TPasImplSimple(Rest),Flags)
  else if (C=TPasImplTry) then
    CheckRestoredImplTry(Path,TPasImplTry(Orig),TPasImplTry(Rest),Flags)
  else if (C=TPasImplTryFinally)
      or (C=TPasImplTryExcept)
      or (C=TPasImplTryExceptElse) then
    CheckRestoredImplTryHandler(Path,TPasImplTryHandler(Orig),TPasImplTryHandler(Rest),Flags)
  else if (C=TPasImplExceptOn) then
    CheckRestoredImplExceptOn(Path,TPasImplExceptOn(Orig),TPasImplExceptOn(Rest),Flags)
  else if (C=TPasImplRaise) then
    CheckRestoredImplRaise(Path,TPasImplRaise(Orig),TPasImplRaise(Rest),Flags)
  else if (C=TPasModule)
        or (C=TPasProgram)
        or (C=TPasLibrary) then
    CheckRestoredModule(Path,TPasModule(Orig),TPasModule(Rest),Flags)
  else if C.InheritsFrom(TPasSection) then
    CheckRestoredSection(Path,TPasSection(Orig),TPasSection(Rest),Flags)
  else if C=TPasAttributes then
    CheckRestoredAttributes(Path,TPasAttributes(Orig),TPasAttributes(Rest),Flags)
  else
    Fail(Path+': unknown class '+C.ClassName);

  CheckRestoredAnalyzerElement(Path,Orig,Rest);
end;

procedure TCustomTestPrecompile.CheckRestoredElementList(const Path: string;
  Orig, Rest: TFPList; Flags: TPCCheckFlags);
var
  OrigItem, RestItem: TObject;
  i: Integer;
  SubPath: String;
begin
  if not CheckRestoredObject(Path,Orig,Rest) then exit;
  AssertEquals(Path+'.Count',Orig.Count,Rest.Count);
  for i:=0 to Orig.Count-1 do
    begin
    SubPath:=Path+'['+IntToStr(i)+']';
    OrigItem:=TObject(Orig[i]);
    if not (OrigItem is TPasElement) then
      Fail(SubPath+' Orig='+GetObjName(OrigItem));
    RestItem:=TObject(Rest[i]);
    if not (RestItem is TPasElement) then
      Fail(SubPath+' Rest='+GetObjName(RestItem));
    //writeln('TCustomTestPrecompile.CheckRestoredElementList ',GetObjName(OrigItem),' ',GetObjName(RestItem));
    SubPath:=Path+'['+IntToStr(i)+']"'+TPasElement(OrigItem).Name+'"';
    CheckRestoredElement(SubPath,TPasElement(OrigItem),TPasElement(RestItem),Flags);
    end;
end;

procedure TCustomTestPrecompile.CheckRestoredElementArray(const Path: string;
  Orig, Rest: TPasElementArray; Flags: TPCCheckFlags);
var
  OrigItem, RestItem: TPasElement;
  i: Integer;
  SubPath: String;
begin
  AssertEquals(Path+'.length',length(Orig),length(Rest));
  for i:=0 to length(Orig)-1 do
    begin
    SubPath:=Path+'['+IntToStr(i)+']';
    OrigItem:=Orig[i];
    if not (OrigItem is TPasElement) then
      Fail(SubPath+' Orig='+GetObjName(OrigItem));
    RestItem:=Rest[i];
    if not (RestItem is TPasElement) then
      Fail(SubPath+' Rest='+GetObjName(RestItem));
    //writeln('TCustomTestPrecompile.CheckRestoredElementList ',GetObjName(OrigItem),' ',GetObjName(RestItem));
    SubPath:=Path+'['+IntToStr(i)+']"'+TPasElement(OrigItem).Name+'"';
    CheckRestoredElement(SubPath,TPasElement(OrigItem),TPasElement(RestItem),Flags);
    end;
end;

procedure TCustomTestPrecompile.CheckRestoredElRefList(const Path: string;
  OrigParent: TPasElement; Orig: TFPList; RestParent: TPasElement;
  Rest: TFPList; AllowInSitu: boolean; Flags: TPCCheckFlags);
var
  OrigItem, RestItem: TObject;
  i: Integer;
  SubPath: String;
begin
  if not CheckRestoredObject(Path,Orig,Rest) then exit;
  AssertEquals(Path+'.Count',Orig.Count,Rest.Count);
  for i:=0 to Orig.Count-1 do
    begin
    SubPath:=Path+'['+IntToStr(i)+']';
    OrigItem:=TObject(Orig[i]);
    if not (OrigItem is TPasElement) then
      Fail(SubPath+' Orig='+GetObjName(OrigItem));
    RestItem:=TObject(Rest[i]);
    if not (RestItem is TPasElement) then
      Fail(SubPath+' Rest='+GetObjName(RestItem));
    if AllowInSitu then
      CheckRestoredElOrRef(SubPath,OrigParent,TPasElement(OrigItem),RestParent,TPasElement(RestItem),Flags)
    else
      CheckRestoredReference(SubPath,TPasElement(OrigItem),TPasElement(RestItem));
    end;
end;

procedure TCustomTestPrecompile.CheckRestoredPasExpr(const Path: string; Orig,
  Rest: TPasExpr; Flags: TPCCheckFlags);
begin
  if Orig.Kind<>Rest.Kind then
    Fail(Path+'.Kind');
  if Orig.OpCode<>Rest.OpCode then
    Fail(Path+'.OpCode');
  CheckRestoredElement(Path+'.Format1',Orig.format1,Rest.format1,Flags);
  CheckRestoredElement(Path+'.Format2',Orig.format2,Rest.format2,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredUnaryExpr(const Path: string;
  Orig, Rest: TUnaryExpr; Flags: TPCCheckFlags);
begin
  CheckRestoredElement(Path+'.Operand',Orig.Operand,Rest.Operand,Flags);
  CheckRestoredPasExpr(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredBinaryExpr(const Path: string;
  Orig, Rest: TBinaryExpr; Flags: TPCCheckFlags);
begin
  CheckRestoredElement(Path+'.left',Orig.left,Rest.left,Flags);
  CheckRestoredElement(Path+'.right',Orig.right,Rest.right,Flags);
  CheckRestoredPasExpr(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredPrimitiveExpr(const Path: string;
  Orig, Rest: TPrimitiveExpr; Flags: TPCCheckFlags);
begin
  AssertEquals(Path+'.Value',Orig.Value,Rest.Value);
  CheckRestoredPasExpr(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredBoolConstExpr(const Path: string;
  Orig, Rest: TBoolConstExpr; Flags: TPCCheckFlags);
begin
  AssertEquals(Path+'.Value',Orig.Value,Rest.Value);
  CheckRestoredPasExpr(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredParamsExpr(const Path: string;
  Orig, Rest: TParamsExpr; Flags: TPCCheckFlags);
begin
  CheckRestoredElement(Path+'.Value',Orig.Value,Rest.Value,Flags);
  CheckRestoredPasExprArray(Path+'.Params',Orig.Params,Rest.Params,Flags);
  CheckRestoredPasExpr(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredProcedureExpr(const Path: string;
  Orig, Rest: TProcedureExpr; Flags: TPCCheckFlags);
begin
  CheckRestoredProcedure(Path+'$Ano',Orig.Proc,Rest.Proc,Flags);
  CheckRestoredPasExpr(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredRecordValues(const Path: string;
  Orig, Rest: TRecordValues; Flags: TPCCheckFlags);
var
  i: Integer;
begin
  AssertEquals(Path+'.Fields.length',length(Orig.Fields),length(Rest.Fields));
  for i:=0 to length(Orig.Fields)-1 do
    begin
    AssertEquals(Path+'.Field['+IntToStr(i)+'].Name',Orig.Fields[i].Name,Rest.Fields[i].Name);
    CheckRestoredElement(Path+'.Field['+IntToStr(i)+'].ValueExp',Orig.Fields[i].ValueExp,Rest.Fields[i].ValueExp,Flags);
    end;
  CheckRestoredPasExpr(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredPasExprArray(const Path: string;
  Orig, Rest: TPasExprArray; Flags: TPCCheckFlags);
var
  i: Integer;
begin
  AssertEquals(Path+'.length',length(Orig),length(Rest));
  for i:=0 to length(Orig)-1 do
    CheckRestoredElement(Path+'['+IntToStr(i)+']',Orig[i],Rest[i],Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredArrayValues(const Path: string;
  Orig, Rest: TArrayValues; Flags: TPCCheckFlags);
begin
  CheckRestoredPasExprArray(Path+'.Values',Orig.Values,Rest.Values,Flags);
  CheckRestoredPasExpr(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredResString(const Path: string;
  Orig, Rest: TPasResString; Flags: TPCCheckFlags);
begin
  CheckRestoredElement(Path+'.Expr',Orig.Expr,Rest.Expr,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredAliasType(const Path: string;
  Orig, Rest: TPasAliasType; Flags: TPCCheckFlags);
begin
  CheckRestoredElOrRef(Path+'.DestType',Orig,Orig.DestType,Rest,Rest.DestType,Flags);
  CheckRestoredElement(Path+'.Expr',Orig.Expr,Rest.Expr,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredPointerType(const Path: string;
  Orig, Rest: TPasPointerType; Flags: TPCCheckFlags);
begin
  CheckRestoredElOrRef(Path+'.DestType',Orig,Orig.DestType,Rest,Rest.DestType,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredSpecializedType(
  const Path: string; Orig, Rest: TPasSpecializeType; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.Params',Orig.Params,Rest.Params,Flags);
  CheckRestoredElOrRef(Path+'.DestType',Orig,Orig.DestType,Rest,Rest.DestType,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredInlineSpecializedExpr(
  const Path: string; Orig, Rest: TInlineSpecializeExpr; Flags: TPCCheckFlags);
begin
  CheckRestoredElement(Path+'.Name',Orig.NameExpr,Rest.NameExpr,Flags);
  CheckRestoredElementList(Path+'.Params',Orig.Params,Rest.Params,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredGenericTemplateType(
  const Path: string; Orig, Rest: TPasGenericTemplateType; Flags: TPCCheckFlags
  );
begin
  CheckRestoredElementArray(Path+'.Constraints',Orig.Constraints,Rest.Constraints,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredRangeType(const Path: string;
  Orig, Rest: TPasRangeType; Flags: TPCCheckFlags);
begin
  CheckRestoredElement(Path+'.RangeExpr',Orig.RangeExpr,Rest.RangeExpr,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredArrayType(const Path: string;
  Orig, Rest: TPasArrayType; Flags: TPCCheckFlags);
begin
  CheckRestoredPasExprArray(Path+'.Ranges',Orig.Ranges,Rest.Ranges,Flags);
  CheckRestoredElementList(Path+'.GenericTemplateTypes',Orig.GenericTemplateTypes,Rest.GenericTemplateTypes,Flags);
  if Orig.PackMode<>Rest.PackMode then
    Fail(Path+'.PackMode Orig='+PCUPackModeNames[Orig.PackMode]+' Rest='+PCUPackModeNames[Rest.PackMode]);
  CheckRestoredElOrRef(Path+'.ElType',Orig,Orig.ElType,Rest,Rest.ElType,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredFileType(const Path: string; Orig,
  Rest: TPasFileType; Flags: TPCCheckFlags);
begin
  CheckRestoredElOrRef(Path+'.ElType',Orig,Orig.ElType,Rest,Rest.ElType,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredEnumValue(const Path: string;
  Orig, Rest: TPasEnumValue; Flags: TPCCheckFlags);
begin
  CheckRestoredElement(Path+'.Value',Orig.Value,Rest.Value,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredEnumType(const Path: string; Orig,
  Rest: TPasEnumType; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.Values',Orig.Values,Rest.Values,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredSetType(const Path: string; Orig,
  Rest: TPasSetType; Flags: TPCCheckFlags);
begin
  CheckRestoredElOrRef(Path+'.EnumType',Orig,Orig.EnumType,Rest,Rest.EnumType,Flags);
  AssertEquals(Path+'.IsPacked',Orig.IsPacked,Rest.IsPacked);
end;

procedure TCustomTestPrecompile.CheckRestoredVariant(const Path: string; Orig,
  Rest: TPasVariant; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.Values',Orig.Values,Rest.Values,Flags);
  CheckRestoredElement(Path+'.Members',Orig.Members,Rest.Members,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredRecordType(const Path: string;
  Orig, Rest: TPasRecordType; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.GenericTemplateTypes',Orig.GenericTemplateTypes,Rest.GenericTemplateTypes,Flags);
  if Orig.PackMode<>Rest.PackMode then
    Fail(Path+'.PackMode Orig='+PCUPackModeNames[Orig.PackMode]+' Rest='+PCUPackModeNames[Rest.PackMode]);
  CheckRestoredElementList(Path+'.Members',Orig.Members,Rest.Members,Flags);
  CheckRestoredElOrRef(Path+'.VariantEl',Orig,Orig.VariantEl,Rest,Rest.VariantEl,Flags);
  CheckRestoredElementList(Path+'.Variants',Orig.Variants,Rest.Variants,Flags);
  CheckRestoredElementList(Path+'.GenericTemplateTypes',Orig.GenericTemplateTypes,Rest.GenericTemplateTypes,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredClassType(const Path: string;
  Orig, Rest: TPasClassType; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.GenericTemplateTypes',Orig.GenericTemplateTypes,Rest.GenericTemplateTypes,Flags);
  if Orig.PackMode<>Rest.PackMode then
    Fail(Path+'.PackMode Orig='+PCUPackModeNames[Orig.PackMode]+' Rest='+PCUPackModeNames[Rest.PackMode]);
  if Orig.ObjKind<>Rest.ObjKind then
    Fail(Path+'.ObjKind Orig='+PCUObjKindNames[Orig.ObjKind]+' Rest='+PCUObjKindNames[Rest.ObjKind]);
  if Orig.InterfaceType<>Rest.InterfaceType then
    Fail(Path+'.ObjKind Orig='+PCUClassInterfaceTypeNames[Orig.InterfaceType]+' Rest='+PCUClassInterfaceTypeNames[Rest.InterfaceType]);
  CheckRestoredReference(Path+'.AncestorType',Orig.AncestorType,Rest.AncestorType);
  CheckRestoredReference(Path+'.HelperForType',Orig.HelperForType,Rest.HelperForType);
  AssertEquals(Path+'.IsForward',Orig.IsForward,Rest.IsForward);
  AssertEquals(Path+'.IsExternal',Orig.IsExternal,Rest.IsExternal);
  // irrelevant: IsShortDefinition
  CheckRestoredElement(Path+'.GUIDExpr',Orig.GUIDExpr,Rest.GUIDExpr,Flags);
  CheckRestoredElementList(Path+'.Members',Orig.Members,Rest.Members,Flags);
  AssertEquals(Path+'.Modifiers',Orig.Modifiers.Text,Rest.Modifiers.Text);
  CheckRestoredElRefList(Path+'.Interfaces',Orig,Orig.Interfaces,Rest,Rest.Interfaces,false,Flags);
  CheckRestoredElementList(Path+'.GenericTemplateTypes',Orig.GenericTemplateTypes,Rest.GenericTemplateTypes,Flags);
  AssertEquals(Path+'.ExternalNameSpace',Orig.ExternalNameSpace,Rest.ExternalNameSpace);
  AssertEquals(Path+'.ExternalName',Orig.ExternalName,Rest.ExternalName);
end;

procedure TCustomTestPrecompile.CheckRestoredArgument(const Path: string; Orig,
  Rest: TPasArgument; Flags: TPCCheckFlags);
begin
  if Orig.Access<>Rest.Access then
    Fail(Path+'.Access Orig='+PCUArgumentAccessNames[Orig.Access]+' Rest='+PCUArgumentAccessNames[Rest.Access]);
  CheckRestoredElOrRef(Path+'.ArgType',Orig,Orig.ArgType,Rest,Rest.ArgType,Flags);
  CheckRestoredElement(Path+'.ValueExpr',Orig.ValueExpr,Rest.ValueExpr,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredProcedureType(const Path: string;
  Orig, Rest: TPasProcedureType; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.GenericTemplateTypes',Orig.GenericTemplateTypes,Rest.GenericTemplateTypes,Flags);
  CheckRestoredElementList(Path+'.Args',Orig.Args,Rest.Args,Flags);
  if Orig.CallingConvention<>Rest.CallingConvention then
    Fail(Path+'.CallingConvention Orig='+PCUCallingConventionNames[Orig.CallingConvention]+' Rest='+PCUCallingConventionNames[Rest.CallingConvention]);
  if Orig.Modifiers<>Rest.Modifiers then
    Fail(Path+'.Modifiers');
end;

procedure TCustomTestPrecompile.CheckRestoredResultElement(const Path: string;
  Orig, Rest: TPasResultElement; Flags: TPCCheckFlags);
begin
  CheckRestoredElOrRef(Path+'.ResultType',Orig,Orig.ResultType,Rest,Rest.ResultType,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredFunctionType(const Path: string;
  Orig, Rest: TPasFunctionType; Flags: TPCCheckFlags);
begin
  CheckRestoredElement(Path+'.ResultEl',Orig.ResultEl,Rest.ResultEl,Flags);
  CheckRestoredProcedureType(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredStringType(const Path: string;
  Orig, Rest: TPasStringType; Flags: TPCCheckFlags);
begin
  AssertEquals(Path+'.LengthExpr',Orig.LengthExpr,Rest.LengthExpr);
  if Flags=[] then ;
end;

procedure TCustomTestPrecompile.CheckRestoredVariable(const Path: string; Orig,
  Rest: TPasVariable; Flags: TPCCheckFlags);
begin
  CheckRestoredElOrRef(Path+'.VarType',Orig,Orig.VarType,Rest,Rest.VarType,Flags);
  if Orig.VarModifiers<>Rest.VarModifiers then
    Fail(Path+'.VarModifiers');
  CheckRestoredElement(Path+'.LibraryName',Orig.LibraryName,Rest.LibraryName,Flags);
  CheckRestoredElement(Path+'.ExportName',Orig.ExportName,Rest.ExportName,Flags);
  CheckRestoredElement(Path+'.AbsoluteExpr',Orig.AbsoluteExpr,Rest.AbsoluteExpr,Flags);
  CheckRestoredElement(Path+'.Expr',Orig.Expr,Rest.Expr,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredExportSymbol(const Path: string;
  Orig, Rest: TPasExportSymbol; Flags: TPCCheckFlags);
begin
  CheckRestoredElement(Path+'.NameExpr',Orig.NameExpr,Rest.NameExpr,Flags);
  CheckRestoredElement(Path+'.ExportName',Orig.ExportName,Rest.ExportName,Flags);
  CheckRestoredElement(Path+'.ExportIndex',Orig.ExportIndex,Rest.ExportIndex,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredConst(const Path: string; Orig,
  Rest: TPasConst; Flags: TPCCheckFlags);
begin
  AssertEquals(Path+'.IsConst',Orig.IsConst,Rest.IsConst);
  CheckRestoredVariable(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredProperty(const Path: string; Orig,
  Rest: TPasProperty; Flags: TPCCheckFlags);
begin
  CheckRestoredElement(Path+'.IndexExpr',Orig.IndexExpr,Rest.IndexExpr,Flags);
  CheckRestoredElement(Path+'.ReadAccessor',Orig.ReadAccessor,Rest.ReadAccessor,Flags);
  CheckRestoredElement(Path+'.WriteAccessor',Orig.WriteAccessor,Rest.WriteAccessor,Flags);
  CheckRestoredElement(Path+'.DispIDExpr',Orig.DispIDExpr,Rest.DispIDExpr,Flags);
  CheckRestoredPasExprArray(Path+'.Implements',Orig.Implements,Rest.Implements,Flags);
  CheckRestoredElement(Path+'.StoredAccessor',Orig.StoredAccessor,Rest.StoredAccessor,Flags);
  CheckRestoredElement(Path+'.DefaultExpr',Orig.DefaultExpr,Rest.DefaultExpr,Flags);
  CheckRestoredElementList(Path+'.Args',Orig.Args,Rest.Args,Flags);
  // not needed: ReadAccessorName, WriteAccessorName, ImplementsName, StoredAccessorName
  AssertEquals(Path+'.DispIDReadOnly',Orig.DispIDReadOnly,Rest.DispIDReadOnly);
  AssertEquals(Path+'.IsDefault',Orig.IsDefault,Rest.IsDefault);
  AssertEquals(Path+'.IsNodefault',Orig.IsNodefault,Rest.IsNodefault);
  CheckRestoredVariable(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredMethodResolution(
  const Path: string; Orig, Rest: TPasMethodResolution; Flags: TPCCheckFlags);
begin
  AssertEquals(Path+'.ProcClass',Orig.ProcClass,Rest.ProcClass);
  CheckRestoredElement(Path+'.InterfaceName',Orig.InterfaceName,Rest.InterfaceName,Flags);
  CheckRestoredElement(Path+'.InterfaceProc',Orig.InterfaceProc,Rest.InterfaceProc,Flags);
  CheckRestoredElement(Path+'.ImplementationProc',Orig.ImplementationProc,Rest.ImplementationProc,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredProcNameParts(const Path: string;
  Orig, Rest: TPasProcedure; Flags: TPCCheckFlags);
var
  OrigNameParts, RestNameParts: TProcedureNameParts;
  i: Integer;
  SubPath: String;
  OrigTemplates, RestTemplates: TFPList;
begin
  OrigNameParts:=Orig.NameParts;
  RestNameParts:=Rest.NameParts;
  AssertEquals(Path+'.NameParts<>nil',OrigNameParts<>nil,RestNameParts<>nil);
  if OrigNameParts<>nil then
    begin
    AssertEquals(Path+'.NameParts.Count',OrigNameParts.Count,RestNameParts.Count);
    for i:=0 to OrigNameParts.Count-1 do
      begin
      SubPath:=Path+'.NameParts['+IntToStr(i)+']';
      AssertEquals(SubPath+'.Name',TProcedureNamePart(OrigNameParts[i]).Name,TProcedureNamePart(RestNameParts[i]).Name);
      OrigTemplates:=TProcedureNamePart(OrigNameParts[i]).Templates;
      RestTemplates:=TProcedureNamePart(RestNameParts[i]).Templates;
      CheckRestoredObject(SubPath+'.Templates',OrigTemplates,RestTemplates);
      if OrigTemplates=nil then continue;
      CheckRestoredElementList(SubPath+'.Templates',OrigTemplates,RestTemplates,Flags);
      end;
    end;
end;

procedure TCustomTestPrecompile.CheckRestoredProcedure(const Path: string;
  Orig, Rest: TPasProcedure; Flags: TPCCheckFlags);
var
  RestScope, OrigScope: TPas2JSProcedureScope;
  DeclProc: TPasProcedure;
begin
  CheckRestoredObject(Path+'.CustomData',Orig.CustomData,Rest.CustomData);
  OrigScope:=Orig.CustomData as TPas2JSProcedureScope;
  RestScope:=Rest.CustomData as TPas2JSProcedureScope;
  if OrigScope=nil then
    exit; // msIgnoreInterfaces
  CheckRestoredReference(Path+'.CustomData[TPas2JSProcedureScope].DeclarationProc [20201018123102]',
    OrigScope.DeclarationProc,RestScope.DeclarationProc);
  AssertEquals(Path+'.CustomData[TPas2JSProcedureScope].ResultVarName [20201018123057]',OrigScope.ResultVarName,RestScope.ResultVarName);
  DeclProc:=RestScope.DeclarationProc;
  if DeclProc=nil then
    begin
    DeclProc:=Rest;
    CheckRestoredProcNameParts(Path,Orig,Rest,Flags);
    CheckRestoredElement(Path+'.ProcType',Orig.ProcType,Rest.ProcType,Flags);
    CheckRestoredElement(Path+'.PublicName',Orig.PublicName,Rest.PublicName,Flags);
    CheckRestoredElement(Path+'.LibrarySymbolName',Orig.LibrarySymbolName,Rest.LibrarySymbolName,Flags);
    CheckRestoredElement(Path+'.LibraryExpr',Orig.LibraryExpr,Rest.LibraryExpr,Flags);
    CheckRestoredElement(Path+'.DispIDExpr',Orig.DispIDExpr,Rest.DispIDExpr,Flags);
    AssertEquals(Path+'.AliasName',Orig.AliasName,Rest.AliasName);
    if Orig.Modifiers<>Rest.Modifiers then
      Fail(Path+'.Modifiers');
    AssertEquals(Path+'.MessageName',Orig.MessageName,Rest.MessageName);
    if Orig.MessageType<>Rest.MessageType then
      Fail(Path+'.MessageType Orig='+PCUProcedureMessageTypeNames[Orig.MessageType]+' Rest='+PCUProcedureMessageTypeNames[Rest.MessageType]);
    end
  else
    begin
    // ImplProc
    if Orig.Modifiers*PCUProcedureModifiersImplProc<>Rest.Modifiers*PCUProcedureModifiersImplProc then
      Fail(Path+'.Impl-Modifiers');
    end;
  // Body
  if Orig.Body<>nil then
    begin
    if not Engine.ProcCanBePrecompiled(DeclProc) then
      begin
      // generic body
      if OrigScope.ImplJS<>nil then
        Fail(Path+'.CustomData[TPas2JSProcedureScope].ImplJS [20201018123049] OrigScope.ImplJS<>nil');
      if RestScope.ImplJS<>nil then
        Fail(Path+'.CustomData[TPas2JSProcedureScope].ImplJS [20201018123139] RestScope.ImplJS<>nil');
      CheckRestoredProcedureBody(Path+'.Body',Orig.Body,Rest.Body,Flags+[PCCGeneric]);
      end;
    end
  else if Rest.Body<>nil then
    Fail(Path+'.Body<>nil, expected =nil');
end;

procedure TCustomTestPrecompile.CheckRestoredOperator(const Path: string; Orig,
  Rest: TPasOperator; Flags: TPCCheckFlags);
begin
  if Orig.OperatorType<>Rest.OperatorType then
    Fail(Path+'.OperatorType Orig='+PCUOperatorTypeNames[Orig.OperatorType]+' Rest='+PCUOperatorTypeNames[Rest.OperatorType]);
  AssertEquals(Path+'.TokenBased',Orig.TokenBased,Rest.TokenBased);
  CheckRestoredProcedure(Path,Orig,Rest,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredProcedureBody(const Path: string;
  Orig, Rest: TProcedureBody; Flags: TPCCheckFlags);
begin
  CheckRestoredObject(Path+'.CustomData',Orig.CustomData,Rest.CustomData);
  CheckRestoredDeclarations(Path,Orig,Rest,Flags);
  CheckRestoredElement(Path+'.Body',Orig.Body,Rest.Body,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredAttributes(const Path: string;
  Orig, Rest: TPasAttributes; Flags: TPCCheckFlags);
begin
  CheckRestoredPasExprArray(Path+'.Calls',Orig.Calls,Rest.Calls,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredImplCommand(const Path: string;
  Orig, Rest: TPasImplCommand; Flags: TPCCheckFlags);
begin
  if Path='' then ;
  if Flags=[] then ;
  if Orig=nil then ;
  if Rest=nil then ;
end;

procedure TCustomTestPrecompile.CheckRestoredImplBeginBlock(const Path: string;
  Orig, Rest: TPasImplBeginBlock; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.Elements',Orig.Elements,Rest.Elements,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredImplAsmStatement(
  const Path: string; Orig, Rest: TPasImplAsmStatement; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.Elements',Orig.Elements,Rest.Elements,Flags);
  CheckRestoredStringList(Path+'.Tokens',Orig.Tokens,Rest.Tokens);
end;

procedure TCustomTestPrecompile.CheckRestoredImplRepeatUntil(
  const Path: string; Orig, Rest: TPasImplRepeatUntil; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.Elements',Orig.Elements,Rest.Elements,Flags);
  CheckRestoredElement(Path+'.ConditionExpr',Orig.ConditionExpr,Rest.ConditionExpr,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredImplIfElse(const Path: string;
  Orig, Rest: TPasImplIfElse; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.Elements',Orig.Elements,Rest.Elements,Flags);
  CheckRestoredElement(Path+'.ConditionExpr',Orig.ConditionExpr,Rest.ConditionExpr,Flags);
  CheckRestoredElement(Path+'.IfBranch',Orig.IfBranch,Rest.IfBranch,Flags);
  CheckRestoredElement(Path+'.ElseBranch',Orig.ElseBranch,Rest.ElseBranch,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredImplWhileDo(const Path: string;
  Orig, Rest: TPasImplWhileDo; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.Elements',Orig.Elements,Rest.Elements,Flags);
  CheckRestoredElement(Path+'.ConditionExpr',Orig.ConditionExpr,Rest.ConditionExpr,Flags);
  CheckRestoredElement(Path+'.Body',Orig.Body,Rest.Body,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredImplWithDo(const Path: string;
  Orig, Rest: TPasImplWithDo; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.Elements',Orig.Elements,Rest.Elements,Flags);
  CheckRestoredElementList(Path+'.ConditionExpr',Orig.Expressions,Rest.Expressions,Flags);
  CheckRestoredElement(Path+'.Body',Orig.Body,Rest.Body,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredImplCaseOf(const Path: string;
  Orig, Rest: TPasImplCaseOf; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.Elements',Orig.Elements,Rest.Elements,Flags);
  CheckRestoredElement(Path+'.CaseExpr',Orig.CaseExpr,Rest.CaseExpr,Flags);
  CheckRestoredElement(Path+'.ElseBranch',Orig.ElseBranch,Rest.ElseBranch,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredImplCaseStatement(
  const Path: string; Orig, Rest: TPasImplCaseStatement; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.Elements',Orig.Elements,Rest.Elements,Flags);
  CheckRestoredElementList(Path+'.Expressions',Orig.Expressions,Rest.Expressions,Flags);
  CheckRestoredElement(Path+'.Body',Orig.Body,Rest.Body,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredImplCaseElse(const Path: string;
  Orig, Rest: TPasImplCaseElse; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.Elements',Orig.Elements,Rest.Elements,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredImplForLoop(const Path: string;
  Orig, Rest: TPasImplForLoop; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.Elements',Orig.Elements,Rest.Elements,Flags);
  if Orig.LoopType<>Rest.LoopType then
    AssertEquals(Path+'.LoopType',PCUForLoopType[Orig.LoopType],PCUForLoopType[Rest.LoopType]);
  CheckRestoredElement(Path+'.VariableName',Orig.VariableName,Rest.VariableName,Flags);
  CheckRestoredElement(Path+'.StartExpr',Orig.StartExpr,Rest.StartExpr,Flags);
  CheckRestoredElement(Path+'.EndExpr',Orig.EndExpr,Rest.EndExpr,Flags);
  CheckRestoredElement(Path+'.Body',Orig.Body,Rest.Body,Flags);
  CheckRestoredElement(Path+'.Variable',Orig.Variable,Rest.Variable,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredImplAssign(const Path: string;
  Orig, Rest: TPasImplAssign; Flags: TPCCheckFlags);
begin
  CheckRestoredElement(Path+'.left',Orig.left,Rest.left,Flags);
  CheckRestoredElement(Path+'.right',Orig.right,Rest.right,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredImplSimple(const Path: string;
  Orig, Rest: TPasImplSimple; Flags: TPCCheckFlags);
begin
  CheckRestoredElement(Path+'.Expr',Orig.Expr,Rest.Expr,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredImplTry(const Path: string; Orig,
  Rest: TPasImplTry; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.Elements',Orig.Elements,Rest.Elements,Flags);
  CheckRestoredElement(Path+'.FinallyExcept',Orig.FinallyExcept,Rest.FinallyExcept,Flags);
  CheckRestoredElement(Path+'.ElseBranch',Orig.ElseBranch,Rest.ElseBranch,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredImplTryHandler(const Path: string;
  Orig, Rest: TPasImplTryHandler; Flags: TPCCheckFlags);
begin
  CheckRestoredElementList(Path+'.Elements',Orig.Elements,Rest.Elements,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredImplExceptOn(const Path: string;
  Orig, Rest: TPasImplExceptOn; Flags: TPCCheckFlags);
begin
  CheckRestoredElement(Path+'.VarEl',Orig.VarEl,Rest.VarEl,Flags);
  CheckRestoredElOrRef(Path+'.TypeEl',Orig,Orig.TypeEl,Rest,Rest.TypeEl,Flags);
  CheckRestoredElement(Path+'.Body',Orig.Body,Rest.Body,Flags);
end;

procedure TCustomTestPrecompile.CheckRestoredImplRaise(const Path: string;
  Orig, Rest: TPasImplRaise; Flags: TPCCheckFlags);
begin
  CheckRestoredElement(Path+'.ExceptObject',Orig.ExceptObject,Rest.ExceptObject,Flags);
  CheckRestoredElement(Path+'.ExceptAddr',Orig.ExceptAddr,Rest.ExceptAddr,Flags);
end;

{ TTestPrecompile }

procedure TTestPrecompile.Test_Base256VLQ;

  procedure Test(i: TMaxPrecInt);
  var
    s: String;
    p: PByte;
    j: TMaxPrecInt;
  begin
    s:=EncodeVLQ(i);
    p:=PByte(s);
    j:=DecodeVLQ(p);
    if i<>j then
      Fail('Encode/DecodeVLQ OrigIndex='+IntToStr(i)+' Code="'+s+'" NewIndex='+IntToStr(j));
  end;

  procedure TestStr(i: TMaxPrecInt; Expected: string);
  var
    Actual: String;
  begin
    Actual:=EncodeVLQ(i);
    AssertEquals('EncodeVLQ('+IntToStr(i)+')',Expected,Actual);
  end;

var
  i: Integer;
begin
  TestStr(0,#0);
  TestStr(1,#2);
  TestStr(-1,#3);
  for i:=-8200 to 8200 do
    Test(i);
  Test(High(TMaxPrecInt));
  Test(High(TMaxPrecInt)-1);
  Test(Low(TMaxPrecInt)+2);
  Test(Low(TMaxPrecInt)+1);
  //Test(Low(TMaxPrecInt)); such a high number is not needed by pastojs
end;

procedure TTestPrecompile.TestPC_EmptyUnit;
begin
  StartUnit(false);
  Add([
  'interface',
  'implementation']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Const;
begin
  StartUnit(false);
  Add([
  'interface',
  'const',
  '  Three = 3;',
  '  FourPlusFive: longint = 4+5 deprecated ''deprtext'';',
  '  Four: byte = +6-2*2 platform;',
  '  Affirmative = true;',
  '  BFalse = false;', // bool lit
  '  NotBFalse = not BFalse;', // boolconst
  '  UnaryMinus = -3;', // unary minus
  '  FloatA = -31.678E-012;', // float lit
  '  HighInt = High(longint);', // func params, built-in function
  '  s = ''abc'';', // string lit
  '  c: char = s[1];', // array params
  '  a: array[1..2] of longint = (3,4);', // anonymous array, range, array values
  '  PI: Double; external name ''Math.PI'';',
  'resourcestring',
  '  rs = ''rs'';',
  'implementation']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Var;
begin
  StartUnit(false);
  Add([
  'interface',
  'var',
  '  FourPlusFive: longint = 4+5 deprecated ''deprtext'';',
  '  e: double external name ''Math.e'';',
  '  AnoArr: array of longint = (1,2,3);',
  '  s: string = ''aaaäö'';',
  '  s2: string = ''😊'';', // 1F60A
  '  a,b: array of longint;',
  'implementation']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Enum;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TEnum = (red,green,blue);',
  '  TEnumRg = green..blue;',
  '  TArrOfEnum = array of TEnum;',
  '  TArrOfEnumRg = array of TEnumRg;',
  '  TArrEnumOfInt = array[TEnum] of longint;',
  'var',
  '  HighEnum: TEnum = high(TEnum);',
  'implementation']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Set;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TEnum = (red,green,blue);',
  '  TEnumRg = green..blue;',
  '  TEnumAlias = TEnum;', // alias
  '  TSetOfEnum = set of TEnum;',
  '  TSetOfEnumRg = set of TEnumRg;',
  '  TSetOfDir = set of (west,east);',
  'var',
  '  Empty: TSetOfEnum = [];', // empty set lit
  '  All: TSetOfEnum = [low(TEnum)..pred(high(TEnum)),high(TEnum)];', // full set lit, range in set
  'implementation']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Set_InFunction;
begin
  StartUnit(false);
  Add([
  'interface',
  'procedure DoIt;',
  'implementation',
  'procedure DoIt;',
  'type',
  '  TEnum = (red,green,blue);',
  '  TEnumRg = green..blue;',
  '  TEnumAlias = TEnum;', // alias
  '  TSetOfEnum = set of TEnum;',
  '  TSetOfEnumRg = set of TEnumRg;',
  '  TSetOfDir = set of (west,east);',
  'var',
  '  Empty: TSetOfEnum = [];', // empty set lit
  '  All: TSetOfEnum = [low(TEnum)..pred(high(TEnum)),high(TEnum)];', // full set lit, range in set
  '  Dirs: TSetOfDir;',
  'begin',
  '  Dirs:=[east];',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_SetOfAnonymousEnumType;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TSetOfDir = set of (west,east);',
  'implementation']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Record;
begin
  StartUnit(false);
  Add([
  '{$ModeSwitch externalclass}',
  'interface',
  'type',
  '  TRec = record',
  '    i: longint;',
  '    s: string;',
  '    b: boolean external name ''ext'';',
  '  end;',
  '  P = pointer;', // alias type to built-in type
  '  TArrOfRec = array of TRec;',
  'var',
  '  r: TRec;', // full set lit, range in set
  'implementation']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Record_InFunction;
begin
  StartUnit(false);
  Add([
  'interface',
  'procedure DoIt;',
  'implementation',
  'procedure DoIt;',
  'type',
  '  TRec = record',
  '    i: longint;',
  '    s: string;',
  '  end;',
  '  P = ^TRec;',
  '  TArrOfRec = array of TRec;',
  'var',
  '  r: TRec;',
  'begin',
  'end;']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_RecordAdv;
begin
  StartUnit(false);
  Add([
  '{$ModeSwitch advancedrecords}',
  'interface',
  'type',
  '  TRec = record',
  '  private',
  '    FInt: longint;',
  '    procedure SetInt(Value: longint);',
  '    function GetItems(Value: word): word;',
  '    procedure SetItems(Index, Value: word);',
  '  public',
  '    property Int: longint read FInt write SetInt default 3;',
  '    property Items[Index: word]: word read GetItems write SetItems; default;',
  '  end;',
  'var',
  '  r: trec;',
  'implementation',
  'procedure TRec.SetInt(Value: longint);',
  'begin',
  'end;',
  'function TRec.GetItems(Value: word): word;',
  'begin',
  'end;',
  'procedure TRec.SetItems(Index, Value: word);',
  'begin',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_JSValue;
begin
  StartUnit(false);
  Add([
  'interface',
  'var',
  '  p: pointer = nil;', // pointer, nil lit
  '  js: jsvalue = 13 div 4;', // jsvalue
  'implementation']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Array;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TEnum = (red,green);',
  '  TArrInt = array of longint;',
  '  TArrInt2 = array[1..2] of longint;',
  '  TArrEnum1 = array[red..green] of longint;',
  '  TArrEnum2 = array[TEnum] of longint;',
  'implementation']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_ArrayOfAnonymous;
begin
  StartUnit(false);
  Add([
  'interface',
  'var',
  '  a: array of pointer;',
  'implementation']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Array_InFunction;
begin
  StartUnit(false);
  Add([
  'interface',
  'procedure DoIt;',
  'implementation',
  'procedure DoIt;',
  'type',
  '  TArr = array[1..2] of word;',
  'var',
  '  arr: TArr;',
  'begin',
  '  arr[2]:=arr[1];',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Proc;
begin
  StartUnit(false);
  Add([
  'interface',
  '  function Abs(d: double): double; external name ''Math.Abs'';',
  '  function GetIt(d: double): double;',
  '  procedure DoArgs(const a; var b: array of char; out c: jsvalue); inline;',
  '  procedure DoMulti(a,b: byte);',
  'implementation',
  'var k: double;',
  'function GetIt(d: double): double;',
  'var j: double;',
  'begin',
  '  j:=Abs(d+k);',
  '  Result:=j;',
  'end;',
  'procedure DoArgs(const a; var b: array of char; out c: jsvalue); inline;',
  'begin',
  'end;',
  'procedure DoMulti(a,b: byte);',
  'begin',
  'end;',
  'procedure NotUsed;',
  'begin',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Proc_Nested;
begin
  StartUnit(false);
  Add([
  'interface',
  '  function GetIt(d: longint): longint;',
  'implementation',
  'var k: double;',
  'function GetIt(d: longint): longint;',
  'var j: double;',
  '  function GetSum(a,b: longint): longint; forward;',
  '  function GetMul(a,b: longint): longint; ',
  '  begin',
  '    Result:=a*b;',
  '  end;',
  '  function GetSum(a,b: longint): longint;',
  '  begin',
  '    Result:=a+b;',
  '  end;',
  '  procedure NotUsed;',
  '  begin',
  '  end;',
  'begin',
  '  Result:=GetMul(GetSum(d,2),3);',
  'end;',
  'procedure NotUsed;',
  'begin',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Proc_LocalConst;
begin
  StartUnit(false);
  Add([
  'interface',
  'function GetIt(d: double): double;',
  'implementation',
  'function GetIt(d: double): double;',
  'const',
  '  c: double = 3.3;',
  '  e: double = 2.7;', // e is not used
  'begin',
  '  Result:=d+c;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Proc_UTF8;
begin
  StartUnit(false);
  Add([
  'interface',
  'function DoIt: string;',
  'implementation',
  'function DoIt: string;',
  'const',
  '  c = ''äöü😊'';',
  'begin',
  '  Result:=''ÄÖÜ😊''+c;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Proc_Arg;
begin
  StartUnit(false);
  Add([
  'interface',
  'procedure DoIt(var a; out b,c: longint; const e,f: array of byte; g: boolean = true);',
  'implementation',
  'procedure DoIt(var a; out b,c: longint; const e,f: array of byte; g: boolean = true);',
  'begin',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_ProcType;
begin
  StartUnit(false);
  Add([
  '{$modeswitch arrayoperators}',
  'interface',
  'type',
  '  TProc = procedure;',
  '  TArrProc = array of tproc;',
  'procedure Mark;',
  'procedure DoIt(const a: TArrProc);',
  'implementation',
  'procedure Mark;',
  'var',
  '  p: TProc;',
  '  a: TArrProc;',
  'begin',
  '  DoIt([@Mark,p]+a);',
  'end;',
  'procedure DoIt(const a: TArrProc);',
  'begin',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Proc_Anonymous;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TFunc = reference to function(w: word): word;',
  '  function GetIt(f: TFunc): longint;',
  'implementation',
  'var k: byte;',
  'function GetIt(f: TFunc): longint;',
  'begin',
  '  f:=function(w: word): word',
  '    var j: byte;',
  '      function GetMul(a,b: longint): longint; ',
  '      begin',
  '        Result:=a*b;',
  '      end;',
  '    begin',
  '      Result:=j*GetMul(1,2)*k;',
  '    end;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Proc_ArrayOfConst;
begin
  StartUnit(true,[supTVarRec]);
  Add([
  'interface',
  'procedure Fly(arr: array of const);',
  'implementation',
  'procedure Fly(arr: array of const);',
  'begin',
  '  if arr[1].VType=1 then ;',
  '  if arr[2].VInteger=1 then ;',
  '  Fly([true,0.3]);',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Class;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TObject = class',
  '  protected',
  '    FInt: longint;',
  '    procedure SetInt(Value: longint); virtual; abstract;',
  '  public',
  '    property Int: longint read FInt write SetInt default 3;',
  '  end;',
  '  TBird = class',
  '  protected',
  '    procedure SetInt(Value: longint); override;',
  '  published',
  '    property Int;',
  '  end;',
  'var',
  '  o: tobject;',
  'implementation',
  'procedure TBird.SetInt(Value: longint);',
  'begin',
  'end;'
  ]);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_ClassForward;
begin
  Converter.Options:=Converter.Options-[coNoTypeInfo];
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TObject = class end;',
  '  TFish = class;',
  '  TBird = class;',
  '  TBirdClass = class of TBird;',
  '  TFish = class',
  '    B: TBird;',
  '  end;',
  '  TBird = class',
  '    F: TFish;',
  '  end;',
  '  TFishClass = class of TFish;',
  'var',
  '  b: tbird;',
  '  f: tfish;',
  '  bc: TBirdClass;',
  '  fc: TFishClass;',
  'implementation',
  'end.'
  ]);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_ClassConstructor;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TObject = class',
  '    constructor Create; virtual;',
  '  end;',
  '  TBird = class',
  '    constructor Create; override;',
  '  end;',
  'procedure DoIt;',
  'implementation',
  'constructor TObject.Create;',
  'begin',
  'end;',
  'constructor TBird.Create;',
  'begin',
  '  inherited;',
  'end;',
  'procedure DoIt;',
  'var b: TBird;',
  'begin',
  '  b:=TBird.Create;',
  'end;',
  'end.'
  ]);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_ClassDestructor;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TObject = class',
  '    destructor Destroy; virtual;',
  '  end;',
  '  TBird = class',
  '    destructor Destroy; override;',
  '  end;',
  'procedure DoIt;',
  'implementation',
  'destructor TObject.Destroy;',
  'begin',
  'end;',
  'destructor TBird.Destroy;',
  'begin',
  '  inherited;',
  'end;',
  'procedure DoIt;',
  'var b: TBird;',
  'begin',
  '  b.Destroy;',
  'end;',
  'end.'
  ]);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_ClassDispatchMessage;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  {$DispatchField DispInt}',
  '  {$DispatchStrField DispStr}',
  '  TObject = class',
  '  end;',
  '  THopMsg = record',
  '    DispInt: longint;',
  '  end;',
  '  TPutMsg = record',
  '    DispStr: string;',
  '  end;',
  '  TBird = class',
  '    procedure Fly(var Msg); virtual; abstract; message 2;',
  '    procedure Run; overload; virtual; abstract;',
  '    procedure Run(var Msg); overload; message ''Fast'';',
  '    procedure Hop(var Msg: THopMsg); virtual; abstract; message 3;',
  '    procedure Put(var Msg: TPutMsg); virtual; abstract; message ''foo'';',
  '  end;',
  'implementation',
  'procedure TBird.Run(var Msg);',
  'begin',
  'end;',
  'end.',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Initialization;
begin
  StartUnit(false);
  Add([
  'interface',
  'implementation',
  'type',
  '  TCaption = string;',
  '  TRec = record h: string; end;',
  'var',
  '  s: TCaption;',
  '  r: TRec;',
  'initialization',
  '  s:=''ö😊'';',
  '  r.h:=''Ä😊'';',
  'end.',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_BoolSwitches;
begin
  StartUnit(false);
  Add([
  'interface',
  '{$R+}',
  '{$C+}',
  'type',
  '  TObject = class',
  '{$C-}',
  '    procedure DoIt;',
  '  end;',
  '{$C+}',
  'implementation',
  '{$R-}',
  'procedure TObject.DoIt;',
  'begin',
  'end;',
  '{$C-}',
  'initialization',
  '{$R+}',
  'end.',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_ClassInterface;
begin
  StartUnit(false);
  Add([
  'interface',
  '{$interfaces corba}',
  'type',
  '  IUnknown = interface',
  '  end;',
  '  IFlying = interface',
  '    procedure SetItems(Index: longint; Value: longint);',
  '  end;',
  '  IBird = interface(IFlying)',
  '    [''{D44C1F80-44F9-4E88-8443-C518CCDC1FE8}'']',
  '    function GetItems(Index: longint): longint;',
  '    property Items[Index: longint]: longint read GetItems write SetItems;',
  '  end;',
  '  TObject = class',
  '  end;',
  '  TBird = class(TObject,IBird)',
  '  strict private',
  '    function IBird.GetItems = RetItems;',
  '    function RetItems(Index: longint): longint; virtual; abstract;',
  '    procedure SetItems(Index: longint; Value: longint); virtual; abstract;',
  '  end;',
  '  TEagle = class(TObject,IBird)',
  '  strict private',
  '    FBird: IBird;',
  '    property Bird: IBird read FBird implements IBird;',
  '  end;',
  'implementation',
  'end.',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Attributes;
begin
  StartUnit(false);
  Add([
  'interface',
  '{$modeswitch PrefixedAttributes}',
  'type',
  '  TObject = class',
  '    constructor Create;',
  '  end;',
  '  TCustomAttribute = class',
  '    constructor Create(Id: word);',
  '  end;',
  '  [Missing]',
  '  TBird = class',
  '    [TCustom]',
  '    FField: word;',
  '  end;',
  '  TRec = record',
  '    [TCustom]',
  '    Size: word;',
  '  end;',
  'var',
  '  [TCustom, TCustom(3)]',
  '  o: TObject;',
  'implementation',
  '[TCustom]',
  'constructor TObject.Create; begin end;',
  'constructor TCustomAttribute.Create(Id: word); begin end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_GenericFunction_Assign;
begin
  StartUnit(false);
  Parser.Options:=Parser.Options+[po_cassignments];
  Add([
  'interface',
  'generic function Run<T>(a: T): T;',
  'implementation',
  'generic function Run<T>(a: T): T;',
  'var b: T;',
  '  i: word;',
  'begin',
  '  b:=a;',
  '  Result:=b;',
  '  i+=1;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_GenericFunction_Asm;
begin
  StartUnit(false);
  Add([
  'interface',
  'generic function Run<T>(a: T): T;',
  'generic function Fly<T>(b: T): T;',
  'implementation',
  'generic function Run<T>(a: T): T; assembler;',
  'asm',
  '  console.log(a);',
  'end;',
  'generic function Fly<T>(b: T): T;',
  'begin',
  '  asm end;',
  '  asm',
  '    console.log(b);',
  '  end;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_GenericFunction_RepeatUntil;
begin
  StartUnit(false);
  Add([
  'interface',
  'generic function Run<T>(a: T): T;',
  'implementation',
  'generic function Run<T>(a: T): T;',
  'begin',
  '  repeat until a>1;',
  '  repeat',
  '    Result:=a;',
  '  until false',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_GenericFunction_IfElse;
begin
  StartUnit(false);
  Add([
  'interface',
  'generic function Run<T>(a: T): T;',
  'implementation',
  'generic function Run<T>(a: T): T;',
  'begin',
  '  if true then ;',
  '  if false then else ;',
  '  if false then Result:=a else ;',
  '  if false then else Result:=a;',
  '  if true then a:=a else Result:=a;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_GenericFunction_WhileDo;
begin
  StartUnit(false);
  Add([
  'interface',
  'generic function Run<T>(a: T): T;',
  'implementation',
  'generic function Run<T>(a: T): T;',
  'begin',
  '  while true do ;',
  '  while true do a:=a;',
  '  while true do while false do Result:=a;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_GenericFunction_WithDo;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TRec = record w: word; end;',
  'generic function Run<T>(a: T): T;',
  'implementation',
  'generic function Run<T>(a: T): T;',
  'var r,s: TRec;',
  'begin',
  '  with r do ;',
  '  with r do a:=a;',
  '  with r do begin w:=w; end;',
  '  with r,s do w:=w;',
  '  with r do with s do w:=w;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_GenericFunction_CaseOf;
begin
  StartUnit(false);
  Add([
  'interface',
  'generic function Run<T>(a: T): T;',
  'implementation',
  'generic function Run<T>(a: T): T;',
  'var i,j,k,l,m,n,o: word;',
  'begin',
  '  case i of',
  '  1: ;',
  '  end;',
  '  case j of',
  '  1: ;',
  '  2..3: ;',
  '  4,5: ;',
  '  end;',
  '  case k of',
  '  1: ;',
  '  else',
  '  end;',
  '  case l of',
  '  1: ;',
  '  else m:=m;',
  '  end;',
  '  case n of',
  '  1: o:=o;',
  '  end;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_GenericFunction_ForLoop;
begin
  StartUnit(false);
  Add([
  'interface',
  'generic function Run<T>(a: T): T;',
  'implementation',
  'generic function Run<T>(a: T): T;',
  'var i,j,k,l: word;',
  '  c: char;',
  'begin',
  '  for i:=1 to 3 do ;',
  '  for j:=1+4 to 3*7 do ;',
  '  for k:=-1 to 2 do l:=l;',
  '  for c in char do ;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_GenericFunction_Simple;
begin
  StartUnit(false);
  Add([
  'interface',
  'generic function Run<T>(a: T): T;',
  'implementation',
  'procedure Fly(w: word = 0); begin end;',
  'generic function Run<T>(a: T): T;',
  'begin',
  '  Fly;',
  '  Fly();',
  '  Fly(3);',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_GenericFunction_TryFinally;
begin
  StartUnit(false);
  Add([
  'interface',
  'generic function Run<T>(a: T): T;',
  'implementation',
  'generic function Run<T>(a: T): T;',
  'var i: word;',
  'begin',
  '  try',
  '  finally;',
  '  end;',
  '  try',
  '    i:=i;',
  '  finally;',
  '  end;',
  '  try',
  '  finally;',
  '    i:=i;',
  '  end;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_GenericFunction_TryExcept;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TObject = class end;',
  '  Exception = class Msg: string; end;',
  '  EInvalidCast = class(Exception) end;',
  'generic function Run<T>(a: T): T;',
  'implementation',
  'generic function Run<T>(a: T): T;',
  'var vI: longint;',
  'begin',
  '  try',
  '    vi:=1;',
  '  except',
  '    vi:=2',
  '  end;',
  '  try',
  '  except',
  '    raise;',
  '  end;',
  '  try',
  '    VI:=4;',
  '  except',
  '    on einvalidcast do',
  '      raise;',
  '    on E: exception do',
  '      if e.msg='''' then',
  '        raise e;',
  '    else',
  '      vi:=5',
  '  end;',
  '  try',
  '    VI:=6;',
  '  except',
  '    on einvalidcast do ;',
  '  end;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_GenericFunction_LocalProc;
begin
  StartUnit(false);
  Add([
  'interface',
  'generic function Run<T>(a: T): T;',
  'implementation',
  'generic function Run<T>(a: T): T;',
  'var vI: longint;',
  '  procedure SubA; forward;',
  '  procedure SubB;',
  '  begin',
  '    SubA;',
  '    vI:=vI;',
  '  end;',
  '  procedure SubA;',
  '  begin',
  '    SubB;',
  '    vI:=vI;',
  '  end;',
  'begin',
  '  SubB;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_GenericFunction_AnonymousProc;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TFunc = reference to function(x: word): word;',
  'var Func: TFunc;',
  'generic function Run<T>(a: T): T;',
  'implementation',
  'generic function Run<T>(a: T): T;',
  'begin',
  '  Func:=function(b:word): word',
  '  begin',
  '    exit(b);',
  '    exit(Result);',
  '  end;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_GenericClass;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TObject = class',
  '  end;',
  '  generic TBird<T> = class',
  '    a: T;',
  '    function Run: T;',
  '  end;',
  'implementation',
  'function TBird.Run: T;',
  'var b: T;',
  'begin',
  '  b:=a; Result:=b;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_GenericMethod;
begin
  StartUnit(false);
  Add([
  '{$mode delphi}',
  'interface',
  'type',
  '  TObject = class',
  '  end;',
  '  TBird = class',
  '    function Run<T>(a: T): T;',
  '  end;',
  'implementation',
  'function TBird.Run<T>(a: T): T;',
  'var b: T;',
  'begin',
  '  b:=a;',
  '  Result:=b;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_SpecializeClassSameUnit;
begin
  StartUnit(false);
  Add([
  '{$mode delphi}',
  'interface',
  'type',
  '  TObject = class',
  '  end;',
  '  TBird<T> = class',
  '    a: T;',
  '  end;',
  '  TBigBird = TBIrd<double>;',
  'var',
  '  b: TBigBird;',
  'implementation',
  'begin',
  '  b.a:=1.3;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Specialize_LocalTypeInUnit;
begin
  StartUnit(false);
  Add([
  '{$mode delphi}',
  'interface',
  'type',
  '  TObject = class',
  '  end;',
  '  TBird<T> = class',
  '    a: T;',
  '  end;',
  '  TDoubleBird = TBIrd<double>;',
  'var',
  '  db: TDoubleBird;',
  'procedure Fly;',
  'implementation',
  'type',
  '  TWordBird = TBird<word>;',
  'procedure Run;',
  'type TShortIntBird = TBird<shortint>;',
  'var',
  '  shb: TShortIntBird;',
  '  wb: TWordBird;',
  'begin',
  '  shb.a:=3;',
  '  wb.a:=4;',
  'end;',
  'procedure Fly;',
  'type TByteBird = TBird<byte>;',
  'var bb: TByteBird;',
  'begin',
  '  bb.a:=5;',
  '  Run;',
  'end;',
  'begin',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Specialize_ClassForward;
begin
  StartUnit(false);
  Add([
  '{$mode delphi}',
  'interface',
  'type',
  '  TObject = class',
  '  end;',
  '  TBird<T> = class;',
  '  TAnt = class',
  '    b: TBird<word>;',
  '  end;',
  '  TBird<T> = class',
  '    a: TAnt;',
  '  end;',
  'procedure Fly;',
  'implementation',
  'procedure Fly;',
  'var b: TBird<Double>;',
  'begin',
  '  b.a:=nil;',
  'end;',
  'begin',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_InlineSpecialize_LocalTypeInUnit;
begin
  StartUnit(false);
  Add([
  '{$mode delphi}',
  'interface',
  'type',
  '  TObject = class',
  '    constructor Create;',
  '  end;',
  '  TBird<T> = class',
  '    a: T;',
  '  end;',
  'var',
  '  db: TBIrd<double>;',
  'procedure Fly;',
  'implementation',
  'constructor TObject.Create;',
  'begin',
  'end;',
  'var wb: TBird<word>;',
  'procedure Run;',
  'var',
  '  shb: TBird<shortint>;',
  '  bb: TBird<boolean>;',
  'begin',
  '  shb.a:=3;',
  '  wb.a:=4;',
  '  bb.a:=true;',
  '  TBird<string>.Create;',
  'end;',
  'procedure Fly;',
  'var lb: TBird<longint>;',
  'begin',
  '  lb.a:=5;',
  '  Run;',
  'end;',
  'begin',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Specialize_Array;
begin
  StartUnit(false);
  Add([
  '{$mode delphi}',
  'interface',
  'type',
  '  TArray<T> = array of T;',
  'var',
  '  da: TArray<double>;',
  'procedure Fly;',
  'implementation',
  'var wa: TArray<word>;',
  'procedure Run;',
  'var',
  '  sha: TArray<shortint>;',
  '  ba: TArray<boolean>;',
  'begin',
  '  sha[1]:=3;',
  '  wa[2]:=4;',
  '  ba[3]:=true;',
  'end;',
  'procedure Fly;',
  'var la: TArray<longint>;',
  'begin',
  '  la[4]:=5;',
  '  Run;',
  'end;',
  'begin',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Specialize_ProcType;
begin
  StartUnit(false);
  Add([
  '{$mode delphi}',
  'interface',
  'type',
  '  TFunc<R,P> = function(a: P): R;',
  'var',
  '  a: TFunc<word,double>;',
  'procedure Fly;',
  'implementation',
  'var b: TFunc<byte,word>;',
  'procedure Run;',
  'var',
  '  c: TFunc<shortint,string>;',
  'begin',
  '  a(3.3);',
  '  b(4);',
  '  c(''abc'');',
  'end;',
  'procedure Fly;',
  'var d: TFunc<longint,boolean>;',
  'begin',
  '  d(true);',
  '  Run;',
  'end;',
  'begin',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_Constraints;
begin
  StartUnit(true,[supTObject]);
  Add([
  '{$mode delphi}',
  'interface',
  'type',
  '  TBird<T: class> = class',
  '  end;',
  '  TEagle<T: record> = class',
  '  end;',
  '  TAnt<T: constructor> = class',
  '  end;',
  '  TFish = class end;',
  '  TBirdFish = TBird<TFish>;',
  '  TAntFish = TAnt<TFish>;',
  '  TWater<T: TFish> = class',
  '  end;',
  '  TRec = record end;',
  'var',
  '  bf: TBirdFish;',
  '  af: TAntFish;',
  '  er: TEagle<TRec>;',
  '  wf: TWater<TFish>;',
  'implementation',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_GenericClass_InlineSpecialize;
begin
  StartUnit(true,[supTObject]);
  Add([
  '{$mode delphi}',
  'interface',
  'type',
  '  TBird<T: class> = class',
  '  end;',
  '  TEagle<T: class> = class(TBird<T>)',
  '  type',
  '    TMyEagle = TEagle<T>;',
  '    function Fly(v: T): T;',
  '  end;',
  'implementation',
  'function TEagle<T>.Fly(v: T): T;',
  'begin',
  '  TEagle<T>.Create;',
  'end;',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_UseUnit;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'type',
    '  TColor = longint;',
    '  TRec = record h: TColor; end;',
    '  TEnum = (red,green);',
    'var',
    '  c: TColor;',
    '  r: TRec;',
    '  e: TEnum;']),
    LinesToStr([
    '']));

  StartUnit(true);
  Add([
  'interface',
  'uses unit2;',
  'var',
  '  i: system.longint;',
  '  e2: TEnum;',
  'implementation',
  'initialization',
  '  c:=1;',
  '  r.h:=2;',
  '  e:=red;',
  'end.',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_UseUnit_Class;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'type',
    '  TObject = class',
    '  private',
    '    FA: longint;',
    '  public',
    '    type',
    '      TEnum = (red,green);',
    '  public',
    '    i: longint;',
    '    e: TEnum;',
    '    procedure DoIt; virtual; abstract;',
    '    property A: longint read FA write FA;',
    '  end;',
    'var',
    '  o: TObject;']),
    LinesToStr([
    '']));

  StartUnit(true);
  Add([
  'interface',
  'uses unit2;',
  'var',
  '  b: TObject;',
  'implementation',
  'initialization',
  '  o.DoIt;',
  '  o.i:=b.A;',
  '  o.e:=red;',
  'end.',
  '']);
  WriteReadUnit;
end;

procedure TTestPrecompile.TestPC_UseIndirectUnit;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'type',
    '  TObject = class',
    '  public',
    '    i: longint;',
    '  end;']),
    LinesToStr([
    '']));

  AddModuleWithIntfImplSrc('unit1.pp',
    LinesToStr([
    'uses unit2;',
    'var o: TObject;']),
    LinesToStr([
    '']));

  StartUnit(true);
  Add([
  'interface',
  'uses unit1;',
  'implementation',
  'initialization',
  '  o.i:=3;',
  'end.',
  '']);
  WriteReadUnit;
end;

Initialization
  RegisterTests([TTestPrecompile]);
  RegisterPCUFormat;
end.

