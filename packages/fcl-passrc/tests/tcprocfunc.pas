unit tcprocfunc;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, typinfo, fpcunit, pastree, pscanner, pparser, tcbaseparser,testregistry;

type

  { TTestProcedureFunction }

  TTestProcedureFunction= class(TTestParser)
  private
    FAddComment: Boolean;
    FFunc: TPasFunction;
    FHint: String;
    FProc: TPasProcedure;
    FOperator:TPasOperator;
    procedure AddDeclaration(const ASource: string; const AHint: String='');
    procedure AssertArg(ProcType: TPasProcedureType; AIndex: Integer;
      AName: String; AAccess: TArgumentAccess; const TypeName: String;
      AValue: String='');
    procedure AssertArrayArg(ProcType: TPasProcedureType; AIndex: Integer;
      AName: String; AAccess: TArgumentAccess; const ElementTypeName: String);
    procedure AssertFunc(const Mods: TProcedureModifiers; const TypeMods: TProcTypeModifiers; CC: TCallingConvention; ArgCount: Integer; P: TPasFunction=nil);
    procedure AssertProc(const Mods: TProcedureModifiers; const TypeMods: TProcTypeModifiers; CC: TCallingConvention; ArgCount: Integer; P: TPasProcedure=nil);
    function BaseAssertArg(ProcType: TPasProcedureType; AIndex: Integer;
      AName: String; AAccess: TArgumentAccess; AValue: String=''): TPasArgument;
    procedure CreateForwardTest;
    function GetFT: TPasFunctionType;
    function GetPT: TPasProcedureType;
    Procedure ParseProcedure;
    function ParseProcedure(const ASource: string; const AHint: String=''): TPasProcedure;
    Procedure ParseFunction;
    function ParseFunction(const ASource : String; AResult: string = ''; const AHint: String=''; CC : TCallingConvention = ccDefault): TPasProcedure;
    Procedure ParseOperator;
  protected
    procedure SetUp; override;
    procedure TearDown; override;
    Procedure AssertComment;
    Property AddComment : Boolean Read FAddComment Write FAddComment;
    Property Hint : String Read FHint Write FHint;
    Property Proc : TPasProcedure Read FProc;
    Property ProcType : TPasProcedureType Read GetPT;
    Property Func : TPasFunction Read FFunc;
    Property FuncType : TPasFunctionType Read GetFT;
  published
    procedure TestEmptyProcedure;
    procedure TestEmptyProcedureComment;
    Procedure TestEmptyFunction;
    Procedure TestEmptyFunctionComment;
    procedure TestEmptyProcedureDeprecated;
    Procedure TestEmptyFunctionDeprecated;
    procedure TestEmptyProcedurePlatform;
    Procedure TestEmptyFunctionPlatform;
    procedure TestEmptyProcedureExperimental;
    Procedure TestEmptyFunctionExperimental;
    procedure TestEmptyProcedureUnimplemented;
    Procedure TestEmptyFunctionUnimplemented;
    procedure TestProcedureOneArg;
    Procedure TestFunctionOneArg;
    procedure TestProcedureOneVarArg;
    Procedure TestFunctionOneVarArg;
    procedure TestProcedureOneConstArg;
    Procedure TestFunctionOneConstArg;
    procedure TestProcedureOneOutArg;
    Procedure TestFunctionOneOutArg;
    procedure TestProcedureOneConstRefArg;
    Procedure TestFunctionOneConstRefArg;
    procedure TestProcedureTwoArgs;
    Procedure TestFunctionTwoArgs;
    procedure TestProcedureTwoArgsSeparate;
    Procedure TestFunctionTwoArgsSeparate;
    procedure TestProcedureOneArgDefault;
    Procedure TestFunctionOneArgDefault;
    procedure TestProcedureOneArgDefaultSet;
    Procedure TestFunctionOneArgDefaultSet;
    procedure TestProcedureOneArgDefaultExpr;
    Procedure TestFunctionOneArgDefaultExpr;
    procedure TestProcedureTwoArgsDefault;
    Procedure TestFunctionTwoArgsDefault;
    procedure TestFunctionOneArgEnumeratedExplicit;
    procedure TestProcedureOneUntypedVarArg;
    Procedure TestFunctionOneUntypedVarArg;
    procedure TestProcedureTwoUntypedVarArgs;
    Procedure TestFunctionTwoUntypedVarArgs;
    procedure TestProcedureOneUntypedConstArg;
    Procedure TestFunctionOneUntypedConstArg;
    procedure TestProcedureTwoUntypedConstArgs;
    Procedure TestFunctionTwoUntypedConstArgs;
    procedure TestProcedureOpenArrayArg;
    Procedure TestFunctionOpenArrayArg;
    procedure TestProcedureTwoOpenArrayArgs;
    Procedure TestFunctionTwoOpenArrayArgs;
    procedure TestProcedureConstOpenArrayArg;
    Procedure TestFunctionConstOpenArrayArg;
    procedure TestProcedureVarOpenArrayArg;
    Procedure TestFunctionVarOpenArrayArg;
    procedure TestProcedureArrayOfConstArg;
    Procedure TestFunctionArrayOfConstArg;
    procedure TestProcedureConstArrayOfConstArg;
    Procedure TestFunctionConstArrayOfConstArg;
    procedure TestProcedureOnePointerArg;
    procedure TestFUnctionPointerResult;

    Procedure TestProcedureCdecl;
    Procedure TestFunctionCdecl;
    Procedure TestProcedureCdeclDeprecated;
    Procedure TestFunctionCdeclDeprecated;
    Procedure TestProcedureSafeCall;
    Procedure TestFunctionSafeCall;
    //ccDefault,ccRegister,ccPascal,ccCDecl,ccStdCall,ccOldFPCCall,ccSafeCall);
    Procedure TestProcedurePascal;
    Procedure TestFunctionPascal;
    Procedure TestProcedureStdCall;
    Procedure TestFunctionStdCall;
    Procedure TestProcedureOldFPCCall;
    Procedure TestFunctionOldFPCCall;
    procedure TestCallingConventionHardFloat;
    procedure TestCallingConventionMS_ABI_CDecl;
    procedure TestCallingConventionMS_ABI_Default;
    procedure TestCallingConventionMWPascal;
    procedure TestCallingConventionSysV_ABI_CDec;
    procedure TestCallingConventionSysV_ABI_Default;
    procedure TestCallingConventionVectorCall;
    procedure TestCallingConventionSysCall;
    procedure TestCallingConventionSysCallExecbase;
    procedure TestCallingConventionSysCallUtilitybase;
    procedure TestCallingConventionSysCallConsoleDevice;
    Procedure TestProcedurePublic;
    Procedure TestProcedurePublicIdent;
    Procedure TestFunctionPublic;
    Procedure TestProcedureCdeclPublic;
    Procedure TestFunctionCdeclPublic;
    Procedure TestProcedureOverload;
    Procedure TestFunctionOverload;
    Procedure TestProcedureVarargs;
    Procedure TestFunctionVarArgs;
    Procedure TestProcedureCDeclVarargs;
    Procedure TestFunctionCDeclVarArgs;
    procedure TestFunctionDiscardResult;
    Procedure TestProcedureForwardInterface;
    Procedure TestFunctionForwardInterface;
    Procedure TestProcedureForward;
    Procedure TestFunctionForward;
    Procedure TestProcedureFar;
    Procedure TestFunctionFar;
    Procedure TestProcedureCdeclForward;
    Procedure TestFunctionCDeclForward;
    Procedure TestProcedureCompilerProc;
    Procedure TestProcedureNoReturn;
    Procedure TestFunctionCompilerProc;
    Procedure TestProcedureCDeclCompilerProc;
    Procedure TestFunctionCDeclCompilerProc;
    Procedure TestProcedureAssembler;
    Procedure TestFunctionAssembler;
    Procedure TestProcedureCDeclAssembler;
    Procedure TestFunctionCDeclAssembler;
    Procedure TestProcedureExport;
    Procedure TestFunctionExport;
    Procedure TestProcedureCDeclExport;
    Procedure TestFunctionCDeclExport;
    Procedure TestProcedureExternal;
    Procedure TestFunctionExternal;
    Procedure TestFunctionForwardNoReturnDelphi;
    procedure TestFunctionForwardNoReturnNoDelphi;
    Procedure TestProcedureExternalLibName;
    Procedure TestFunctionExternalLibName;
    Procedure TestProcedureExternalLibNameName;
    Procedure TestFunctionExternalLibNameName;
    Procedure TestProcedureExternalName;
    Procedure TestFunctionExternalName;
    Procedure TestProcedureCdeclExternal;
    Procedure TestProcedureAlias;
    Procedure TestFunctionCdeclExternal;
    Procedure TestProcedureCdeclExternalLibName;
    Procedure TestFunctionCdeclExternalLibName;
    Procedure TestProcedureCdeclExternalLibNameName;
    Procedure TestFunctionCdeclExternalLibNameName;
    Procedure TestProcedureCdeclExternalName;
    Procedure TestFunctionCdeclExternalName;
    Procedure TestFunctionAlias;
    Procedure TestOperatorNamedResult;
    Procedure TestOperatorTokens;
    procedure TestOperatorNames;
    Procedure TestAssignOperatorAfterObject;
    Procedure TestFunctionNoResult;
  end;

implementation


procedure TTestProcedureFunction.AddDeclaration(const ASource: string;
  const AHint: String);

Var
  D : String;
begin
  Hint:=AHint;
  D:=ASource;
  If Hint<>'' then
    D:=D+' '+Hint;
  if (D[Length(D)]<>';') then
    D:=D+';';
  Add(D);
end;

function TTestProcedureFunction.GetPT: TPasProcedureType;
begin
  AssertNotNull('have procedure to get type from',Proc);
  Result:=Proc.ProcType;
end;

function TTestProcedureFunction.ParseProcedure(const ASource: string;
  const AHint: String): TPasProcedure;


begin
  If AddComment then
    begin
    Add('// A comment');
    Engine.NeedComments:=True;
    end;
  AddDeclaration('procedure A '+ASource,AHint);
  Self.ParseProcedure;
  Result:=Fproc;
  If AddComment then
    AssertComment;
end;

procedure TTestProcedureFunction.ParseProcedure;

begin
  //  Writeln(source.text);
  ParseDeclarations;
  AssertEquals('One variable definition',1,Declarations.Functions.Count);
  AssertEquals('First declaration is procedure declaration.',TPasProcedure,TObject(Declarations.Functions[0]).ClassType);
  FProc:=TPasProcedure(Declarations.Functions[0]);
  Definition:=FProc;
  AssertEquals('First declaration has correct name.','A',FProc.Name);
  if (Hint<>'') then
    CheckHint(TPasMemberHint(Getenumvalue(typeinfo(TPasMemberHint),'h'+Hint)));
end;

function TTestProcedureFunction.ParseFunction(const ASource : String;AResult: string = ''; const AHint: String = ''; CC : TCallingConvention = ccDefault): TPasProcedure;
Var
  D :String;
  aType : TPasType;
begin
  if (AResult='') then
    AResult:='Integer';
  D:='Function A '+ASource+' : '+AResult;
  If (cc<>ccDefault) then
    D:=D+'; '+cCallingConventions[cc]+';';
  AddDeclaration(D,AHint);
  Self.ParseFunction;
  Result:=FFunc;
  AssertNotNull('Have function result element',FuncType.ResultEl);
  aType:=FuncType.ResultEl.ResultType;
  AssertNotNull('Have function result type element',aType);
  if aResult[1]='^' then
    begin
    Delete(aResult,1,1);
    AssertEquals('Result is pointer type',TPasPointerType,aType.ClassType);
    aType:=TPasPointerType(aType).DestType;
    AssertNotNull('Result pointer type has destination type',aType);
    end;
  AssertEquals('Correct function result type name',AResult,aType.Name);
end;

procedure TTestProcedureFunction.ParseOperator;
begin
  //  Writeln(source.text);
  ParseDeclarations;
  AssertEquals('One operator definition',1,Declarations.Functions.Count);
  AssertEquals('First declaration is function declaration.',TPasOperator,TObject(Declarations.Functions[0]).ClassType);
  FOperator:=TPasOperator(Declarations.Functions[0]);
  Definition:=FOperator;
  if (Hint<>'') then
    CheckHint(TPasMemberHint(Getenumvalue(typeinfo(TPasMemberHint),'h'+Hint)));
end;

procedure TTestProcedureFunction.ParseFunction;
begin
  //  Writeln(source.text);
  ParseDeclarations;
  AssertEquals('One variable definition',1,Declarations.Functions.Count);
  AssertEquals('First declaration is function declaration.',TPasFunction,TObject(Declarations.Functions[0]).ClassType);
  FFunc:=TPasFunction(Declarations.Functions[0]);
  Definition:=FFunc;
  AssertEquals('First declaration has correct name.','A',FFunc.Name);
  if (Hint<>'') then
    CheckHint(TPasMemberHint(Getenumvalue(typeinfo(TPasMemberHint),'h'+Hint)));
end;

procedure TTestProcedureFunction.AssertProc(const Mods: TProcedureModifiers;
  const TypeMods: TProcTypeModifiers; CC: TCallingConvention; ArgCount: Integer;
  P: TPasProcedure);

begin
  If P=Nil then
    P:=Proc;
  AssertNotNull('No proc to assert',P);
  AssertEquals('Procedure modifiers',Mods,P.Modifiers);
  AssertEquals('Procedure type modifiers',TypeMods,P.ProcType.Modifiers);
  AssertEquals('Procedue calling convention',CC,P.CallingConvention);
  AssertEquals('No message name','',p.MessageName);
  AssertEquals('No message type',pmtNone,P.MessageType);
  AssertNotNull('Have procedure type to assert',P.ProcType);
  AssertEquals('Correct number of arguments',ArgCount,P.ProcType.Args.Count);
  AssertEquals('Not of object',False,P.ProcType.IsOfObject);
  AssertEquals('Not is nested',False,P.ProcType.IsNested);
end;

procedure TTestProcedureFunction.AssertFunc(const Mods: TProcedureModifiers;
  const TypeMods: TProcTypeModifiers; CC: TCallingConvention; ArgCount: Integer;
  P: TPasFunction);

begin
  If P=Nil then
    P:=Func;
  AssertNotNull('No func to assert',P);
  AssertEquals('Procedure modifiers',Mods,P.Modifiers);
  AssertEquals('Procedure type modifiers',TypeMods,P.ProcType.Modifiers);
  AssertEquals('Procedue calling convention',CC,P.CallingConvention);
  AssertEquals('No message name','',p.MessageName);
  AssertEquals('No message type',pmtNone,P.MessageType);
  AssertNotNull('Have procedure type to assert',P.ProcType);
  AssertEquals('Correct number of arguments',ArgCount,P.ProcType.Args.Count);
  AssertEquals('Not of object',False,P.ProcType.IsOfObject);
  AssertEquals('Not is nested',False,P.ProcType.IsNested);
end;

function TTestProcedureFunction.BaseAssertArg(ProcType: TPasProcedureType;
  AIndex: Integer; AName: String; AAccess: TArgumentAccess; AValue: String
  ): TPasArgument;

Var
  A : TPasArgument;
  N : String;

begin
  AssertNotNull('Have proctype to test argument',ProcType);
  if AIndex>=Proctype.Args.Count then
    Fail(Format('Cannot test argument: index %d is larger than the number of arguments (%d).',[AIndex,Proctype.Args.Count]));
  AssertNotNull('Have argument at position '+IntToStr(AIndex),Proctype.Args[AIndex]);
  AssertEquals('Have argument type at position '+IntToStr(AIndex),TPasArgument,TObject(Proctype.Args[AIndex]).ClassType);
  N:='Argument '+IntToStr(AIndex+1)+' : ';
  A:=TPasArgument(Proctype.Args[AIndex]);
  AssertEquals(N+'Argument name',AName,A.Name);
  AssertEquals(N+'Argument access',AAccess,A.Access);
  if (AValue='') then
    AssertNull(N+' no default value',A.ValueExpr)
  else
    begin
    AssertNotNull(N+' Have default value',A.ValueExpr);
    AssertEquals(N+' Default value',AValue,A.Value);
    end;
  Result:=A;
end;

procedure TTestProcedureFunction.AssertArg(ProcType: TPasProcedureType;
  AIndex: Integer; AName: String; AAccess: TArgumentAccess;
  const TypeName: String; AValue: String);

Var
  A : TPasArgument;
  T : TPasType;
  N : String;

begin
  A:=BaseAssertArg(ProcType,AIndex,ANAme,AAccess,AValue);
  N:='Argument '+IntToStr(AIndex+1)+' : ';
  if (TypeName='') then
    AssertNull(N+' No argument type',A.ArgType)
  else if TypeName[1]<>'^' then
    begin
    AssertNotNull(N+' Have argument type',A.ArgType);
    AssertEquals(N+' Correct argument type name',TypeName,A.ArgType.Name);
    end
  else  
    begin
    AssertNotNull(N+' Have argument type',A.ArgType);
    T:=A.ArgType;
    AssertEquals(N+' type Is pointer type',TPasPointerType,T.CLassType);
    T:=TPasPointerType(T).DestType;
    AssertNotNull(N+'Have dest type',T);
    AssertEquals(N+' Correct argument dest type name',Copy(TypeName,2,MaxInt),T.Name);
    end;
    
end;

procedure TTestProcedureFunction.AssertArrayArg(ProcType: TPasProcedureType;
  AIndex: Integer; AName: String; AAccess: TArgumentAccess;
  const ElementTypeName: String);
Var
  A : TPasArgument;
  AT : TPasArrayType;
  N : String;

begin
  A:=BaseAssertArg(ProcType,AIndex,ANAme,AAccess,'');
  N:='Argument '+IntToStr(AIndex+1)+' : ';
  AssertNotNull(N+' Have argument type',A.ArgType);
  AssertEquals(N+' is arrayType',TPasArrayType,A.ArgType.ClassType);
  AT:=TPasArrayType(A.ArgType);
  if (ElementTypeName='') then
    AssertNull(N+' No array element type',AT.ElType)
  else
    begin
    AssertNotNull(N+' Have array element type',AT.ElType);
    AssertEquals(N+' Correct array element type name',ElementTypeName,AT.ElType.Name);
    end;
end;

function TTestProcedureFunction.GetFT: TPasFunctionType;
begin
  AssertNotNull('have function to get type from',Func);
  AssertEquals('Function type is correct type',TPasFunctionType,Func.ProcType.ClassType);
  Result:=Func.FuncType;
end;

//TProcedureMessageType

procedure TTestProcedureFunction.TestEmptyProcedure;
begin
  ParseProcedure('');
  AssertProc([],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestEmptyProcedureComment;
begin
  AddComment:=True;
  TestEmptyProcedure;
end;

procedure TTestProcedureFunction.TestEmptyFunction;
begin
  ParseFunction('');
  AssertFunc([],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestEmptyFunctionComment;
begin
  AddComment:=True;
  TestEmptyProcedure;
end;

procedure TTestProcedureFunction.TestEmptyProcedureDeprecated;
begin
  ParseProcedure('','deprecated');
  AssertProc([],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestEmptyFunctionDeprecated;
begin
  ParseFunction('','deprecated');
  AssertFunc([],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestEmptyProcedurePlatform;
begin
  ParseProcedure('','platform');
  AssertProc([],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestEmptyFunctionPlatform;
begin
  ParseFunction('','platform');
  AssertFunc([],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestEmptyProcedureExperimental;
begin
  ParseProcedure('','experimental');
  AssertProc([],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestEmptyFunctionExperimental;
begin
  ParseFunction('','experimental');
  AssertFunc([],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestEmptyProcedureUnimplemented;
begin
  ParseProcedure('','unimplemented');
  AssertProc([],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestEmptyFunctionUnimplemented;
begin
  ParseFunction('','unimplemented');
  AssertFunc([],[],ccDefault,0);
end;



procedure TTestProcedureFunction.TestProcedureOneArg;
begin
  ParseProcedure('(B : Integer)');
  AssertProc([],[],ccDefault,1);
  AssertArg(ProcType,0,'B',argDefault,'Integer','');
end;

procedure TTestProcedureFunction.TestProcedureOnePointerArg;
begin
  ParseProcedure('(B : ^Integer)');
  AssertProc([],[],ccDefault,1);
  AssertArg(ProcType,0,'B',argDefault,'^Integer','');
end;

procedure TTestProcedureFunction.TestFunctionPointerResult;
begin
  ParseFunction('()','^LongInt');
  AssertFunc([],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestFunctionOneArg;
begin
  ParseFunction('(B : Integer)');
  AssertFunc([],[],ccDefault,1);
  AssertArg(FuncType,0,'B',argDefault,'Integer','');
end;

procedure TTestProcedureFunction.TestProcedureOneVarArg;
begin
  ParseProcedure('(Var B : Integer)');
  AssertProc([],[],ccDefault,1);
  AssertArg(ProcType,0,'B',argVar,'Integer','');
end;

procedure TTestProcedureFunction.TestFunctionOneVarArg;
begin
  ParseFunction('(Var B : Integer)');
  AssertFunc([],[],ccDefault,1);
  AssertArg(FuncType,0,'B',argVar,'Integer','');
end;

procedure TTestProcedureFunction.TestProcedureOneConstArg;
begin
  ParseProcedure('(Const B : Integer)');
  AssertProc([],[],ccDefault,1);
  AssertArg(ProcType,0,'B',argConst,'Integer','');
end;

procedure TTestProcedureFunction.TestFunctionOneConstArg;
begin
  ParseFunction('(Const B : Integer)');
  AssertFunc([],[],ccDefault,1);
  AssertArg(FuncType,0,'B',argConst,'Integer','');
end;

procedure TTestProcedureFunction.TestProcedureOneOutArg;
begin
  Parser.CurrentModeswitches:=[msObjfpc];
  ParseProcedure('(Out B : Integer)');
  AssertProc([],[],ccDefault,1);
  AssertArg(ProcType,0,'B',argOut,'Integer','');
end;

procedure TTestProcedureFunction.TestFunctionOneOutArg;
begin
  Parser.CurrentModeswitches:=[msObjfpc];
  ParseFunction('(Out B : Integer)');
  AssertFunc([],[],ccDefault,1);
  AssertArg(FuncType,0,'B',argOut,'Integer','');
end;

procedure TTestProcedureFunction.TestProcedureOneConstRefArg;
begin
  ParseProcedure('(Constref B : Integer)');
  AssertProc([],[],ccDefault,1);
  AssertArg(ProcType,0,'B',argConstRef,'Integer','');
end;

procedure TTestProcedureFunction.TestFunctionOneConstRefArg;
begin
  ParseFunction('(ConstRef B : Integer)');
  AssertFunc([],[],ccDefault,1);
  AssertArg(FuncType,0,'B',argConstref,'Integer','');
end;

procedure TTestProcedureFunction.TestProcedureTwoArgs;
begin
  ParseProcedure('(B,C : Integer)');
  AssertProc([],[],ccDefault,2);
  AssertArg(ProcType,0,'B',argDefault,'Integer','');
  AssertArg(ProcType,1,'C',argDefault,'Integer','');
end;

procedure TTestProcedureFunction.TestFunctionTwoArgs;
begin
  ParseFunction('(B,C : Integer)');
  AssertFunc([],[],ccDefault,2);
  AssertArg(FuncType,0,'B',argDefault,'Integer','');
  AssertArg(FuncType,1,'C',argDefault,'Integer','');
end;

procedure TTestProcedureFunction.TestProcedureTwoArgsSeparate;
begin
  ParseProcedure('(B : Integer; C : Integer)');
  AssertProc([],[],ccDefault,2);
  AssertArg(ProcType,0,'B',argDefault,'Integer','');
  AssertArg(ProcType,1,'C',argDefault,'Integer','');
end;

procedure TTestProcedureFunction.TestFunctionTwoArgsSeparate;
begin
  ParseFunction('(B : Integer;C : Integer)');
  AssertFunc([],[],ccDefault,2);
  AssertArg(FuncType,0,'B',argDefault,'Integer','');
  AssertArg(FuncType,1,'C',argDefault,'Integer','');
end;

procedure TTestProcedureFunction.TestProcedureOneArgDefault;
begin
  ParseProcedure('(B : Integer = 1)');
  AssertProc([],[],ccDefault,1);
  AssertArg(ProcType,0,'B',argDefault,'Integer','1');
end;

procedure TTestProcedureFunction.TestFunctionOneArgDefault;
begin
  ParseFunction('(B : Integer = 1)');
  AssertFunc([],[],ccDefault,1);
  AssertArg(FuncType,0,'B',argDefault,'Integer','1');
end;

procedure TTestProcedureFunction.TestFunctionOneArgEnumeratedExplicit;
begin
  ParseFunction('(B : TSomeEnum = TSomeEnum.False)');
  AssertFunc([],[],ccDefault,1);
  AssertArg(FuncType,0,'B',argDefault,'TSomeEnum','TSomeEnum.False');
end;

procedure TTestProcedureFunction.TestProcedureOneArgDefaultSet;
begin
  ParseProcedure('(B : MySet = [1,2])');
  AssertProc([],[],ccDefault,1);
  AssertArg(ProcType,0,'B',argDefault,'MySet','[1, 2]');
end;

procedure TTestProcedureFunction.TestFunctionOneArgDefaultSet;
begin
  ParseFunction('(B : MySet = [1,2])');
  AssertFunc([],[],ccDefault,1);
  AssertArg(FuncType,0,'B',argDefault,'MySet','[1, 2]');
end;

procedure TTestProcedureFunction.TestProcedureOneArgDefaultExpr;
begin
  ParseProcedure('(B : Integer = 1 + 2)');
  AssertProc([],[],ccDefault,1);
  AssertArg(ProcType,0,'B',argDefault,'Integer','1 + 2');
end;

procedure TTestProcedureFunction.TestFunctionOneArgDefaultExpr;
begin
  ParseFunction('(B : Integer = 1 + 2)');
  AssertFunc([],[],ccDefault,1);
  AssertArg(FuncType,0,'B',argDefault,'Integer','1 + 2');
end;

procedure TTestProcedureFunction.TestProcedureTwoArgsDefault;
begin
  ParseProcedure('(B : Integer = 1; C : Integer = 2)');
  AssertProc([],[],ccDefault,2);
  AssertArg(ProcType,0,'B',argDefault,'Integer','1');
  AssertArg(ProcType,1,'C',argDefault,'Integer','2');
end;

procedure TTestProcedureFunction.TestFunctionTwoArgsDefault;
begin
  ParseFunction('(B : Integer = 1; C : Integer = 2)');
  AssertFunc([],[],ccDefault,2);
  AssertArg(FuncType,0,'B',argDefault,'Integer','1');
  AssertArg(FuncType,1,'C',argDefault,'Integer','2');
end;

procedure TTestProcedureFunction.TestProcedureOneUntypedVarArg;
begin
  ParseProcedure('(Var B)');
  AssertProc([],[],ccDefault,1);
  AssertArg(ProcType,0,'B',argVar,'','');
end;

procedure TTestProcedureFunction.TestFunctionOneUntypedVarArg;
begin
  ParseFunction('(Var B)');
  AssertFunc([],[],ccDefault,1);
  AssertArg(FuncType,0,'B',argVar,'','');
end;

procedure TTestProcedureFunction.TestProcedureTwoUntypedVarArgs;
begin
  ParseProcedure('(Var B; Var C)');
  AssertProc([],[],ccDefault,2);
  AssertArg(ProcType,0,'B',argVar,'','');
  AssertArg(ProcType,1,'C',argVar,'','');
end;

procedure TTestProcedureFunction.TestFunctionTwoUntypedVarArgs;
begin
  ParseFunction('(Var B; Var C)');
  AssertFunc([],[],ccDefault,2);
  AssertArg(FuncType,0,'B',argVar,'','');
  AssertArg(FuncType,1,'C',argVar,'','');
end;

procedure TTestProcedureFunction.TestProcedureOneUntypedConstArg;
begin
  ParseProcedure('(Const B)');
  AssertProc([],[],ccDefault,1);
  AssertArg(ProcType,0,'B',argConst,'','');
end;

procedure TTestProcedureFunction.TestFunctionOneUntypedConstArg;
begin
  ParseFunction('(Const B)');
  AssertFunc([],[],ccDefault,1);
  AssertArg(FuncType,0,'B',argConst,'','');
end;

procedure TTestProcedureFunction.TestProcedureTwoUntypedConstArgs;
begin
  ParseProcedure('(Const B; Const C)');
  AssertProc([],[],ccDefault,2);
  AssertArg(ProcType,0,'B',argConst,'','');
  AssertArg(ProcType,1,'C',argConst,'','');
end;

procedure TTestProcedureFunction.TestFunctionTwoUntypedConstArgs;
begin
  ParseFunction('(Const B; Const C)');
  AssertFunc([],[],ccDefault,2);
  AssertArg(FuncType,0,'B',argConst,'','');
  AssertArg(FuncType,1,'C',argConst,'','');
end;

procedure TTestProcedureFunction.TestProcedureOpenArrayArg;
begin
  ParseProcedure('(B : Array of Integer)');
  AssertProc([],[],ccDefault,1);
  AssertArrayArg(ProcType,0,'B',argDefault,'Integer');
end;

procedure TTestProcedureFunction.TestFunctionOpenArrayArg;
begin
  ParseFunction('(B : Array of Integer)');
  AssertFunc([],[],ccDefault,1);
  AssertArrayArg(FuncType,0,'B',argDefault,'Integer');
end;

procedure TTestProcedureFunction.TestProcedureTwoOpenArrayArgs;
begin
  ParseProcedure('(B : Array of Integer;C : Array of Integer)');
  AssertProc([],[],ccDefault,2);
  AssertArrayArg(ProcType,0,'B',argDefault,'Integer');
  AssertArrayArg(ProcType,1,'C',argDefault,'Integer');
end;

procedure TTestProcedureFunction.TestFunctionTwoOpenArrayArgs;
begin
  ParseFunction('(B : Array of Integer;C : Array of Integer)');
  AssertFunc([],[],ccDefault,2);
  AssertArrayArg(FuncType,0,'B',argDefault,'Integer');
  AssertArrayArg(FuncType,1,'C',argDefault,'Integer');
end;

procedure TTestProcedureFunction.TestProcedureConstOpenArrayArg;
begin
  ParseProcedure('(Const B : Array of Integer)');
  AssertProc([],[],ccDefault,1);
  AssertArrayArg(ProcType,0,'B',argConst,'Integer');
end;

procedure TTestProcedureFunction.TestFunctionConstOpenArrayArg;
begin
  ParseFunction('(Const B : Array of Integer)');
  AssertFunc([],[],ccDefault,1);
  AssertArrayArg(FuncType,0,'B',argConst,'Integer');
end;

procedure TTestProcedureFunction.TestProcedureVarOpenArrayArg;
begin
  ParseProcedure('(Var B : Array of Integer)');
  AssertProc([],[],ccDefault,1);
  AssertArrayArg(ProcType,0,'B',argVar,'Integer');
end;

procedure TTestProcedureFunction.TestFunctionVarOpenArrayArg;
begin
  ParseFunction('(Var B : Array of Integer)');
  AssertFunc([],[],ccDefault,1);
  AssertArrayArg(FuncType,0,'B',argVar,'Integer');
end;

procedure TTestProcedureFunction.TestProcedureArrayOfConstArg;
begin
  ParseProcedure('(B : Array of Const)');
  AssertProc([],[],ccDefault,1);
  AssertArrayArg(ProcType,0,'B',argDefault,'');
end;

procedure TTestProcedureFunction.TestFunctionArrayOfConstArg;
begin
  ParseFunction('(B : Array of Const)');
  AssertFunc([],[],ccDefault,1);
  AssertArrayArg(FuncType,0,'B',argDefault,'');
end;

procedure TTestProcedureFunction.TestProcedureConstArrayOfConstArg;
begin
  ParseProcedure('(Const B : Array of Const)');
  AssertProc([],[],ccDefault,1);
  AssertArrayArg(ProcType,0,'B',argConst,'');
end;

procedure TTestProcedureFunction.TestFunctionConstArrayOfConstArg;
begin
  ParseFunction('(Const B : Array of Const)');
  AssertFunc([],[],ccDefault,1);
  AssertArrayArg(FuncType,0,'B',argConst,'');
end;

procedure TTestProcedureFunction.TestCallingConventionSysV_ABI_Default;
begin
  ParseProcedure('; SysV_ABI_Default');
  AssertProc([],[],ccSysV_ABI_Default,0);
end;

procedure TTestProcedureFunction.TestCallingConventionSysV_ABI_CDec;
begin
  ParseProcedure('; SysV_ABI_CDecl');
  AssertProc([],[],ccSysV_ABI_CDecl,0);
end;

procedure TTestProcedureFunction.TestCallingConventionMS_ABI_Default;
begin
  ParseProcedure('; MS_ABI_Default');
  AssertProc([],[],ccMS_ABI_Default,0);
end;

procedure TTestProcedureFunction.TestCallingConventionMS_ABI_CDecl;
begin
  ParseProcedure('; MS_ABI_CDecl');
  AssertProc([],[],ccMS_ABI_CDecl,0);
end;

procedure TTestProcedureFunction.TestCallingConventionVectorCall;
begin
  ParseProcedure('; VectorCall');
  AssertProc([],[],ccVectorCall,0);
end;

procedure TTestProcedureFunction.TestCallingConventionSysCall;
begin
  ParseProcedure('; syscall abc');
  AssertProc([],[],ccSysCall,0);
end;

procedure TTestProcedureFunction.TestCallingConventionSysCallExecbase;
begin
  ParseProcedure('; syscall _execBase 123');
  AssertProc([],[],ccSysCall,0);
end;

procedure TTestProcedureFunction.TestCallingConventionSysCallUtilitybase;
begin
  ParseProcedure('; syscall _utilityBase 123');
  AssertProc([],[],ccSysCall,0);
end;

procedure TTestProcedureFunction.TestCallingConventionSysCallConsoleDevice;
begin
  ParseProcedure('; syscall ConsoleDevice 123');
  AssertProc([],[],ccSysCall,0);
end;

procedure TTestProcedureFunction.TestFunctionDiscardResult;
begin
  AddDeclaration('function A : Integer; discardresult');
  ParseFunction;
  AssertFunc([pmDiscardResult],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestCallingConventionHardFloat;
begin
  ParseProcedure('; HardFloat');
  AssertProc([],[],ccHardFloat,0);
end;

procedure TTestProcedureFunction.TestCallingConventionMWPascal;

begin
  ParseProcedure('; mwpascal');
  AssertProc([],[],ccMWPascal,0);
end;

procedure TTestProcedureFunction.TestProcedureCdecl;
begin
  ParseProcedure('; cdecl');
  AssertProc([],[],ccCdecl,0);
end;

procedure TTestProcedureFunction.TestFunctionCdecl;
begin
  ParseFunction('','','',ccCdecl);
  AssertFunc([],[],ccCdecl,0);
end;

procedure TTestProcedureFunction.TestProcedureCdeclDeprecated;
begin
  ParseProcedure('; cdecl;','deprecated');
  AssertProc([],[],ccCdecl,0);
end;

procedure TTestProcedureFunction.TestFunctionCdeclDeprecated;
begin
  ParseFunction('','','deprecated',ccCdecl);
  AssertFunc([],[],ccCdecl,0);
end;

procedure TTestProcedureFunction.TestProcedureSafeCall;
begin
  ParseProcedure('; safecall;','');
  AssertProc([],[],ccSafeCall,0);
end;

procedure TTestProcedureFunction.TestFunctionSafeCall;
begin
  ParseFunction('','','',ccSafecall);
  AssertFunc([],[],ccSafecall,0);
end;

procedure TTestProcedureFunction.TestProcedurePascal;
begin
  ParseProcedure('; pascal;','');
  AssertProc([],[],ccPascal,0);
end;

procedure TTestProcedureFunction.TestFunctionPascal;
begin
  ParseFunction('','','',ccPascal);
  AssertFunc([],[],ccPascal,0);
end;

procedure TTestProcedureFunction.TestProcedureStdCall;
begin
  ParseProcedure('; stdcall;','');
  AssertProc([],[],ccstdcall,0);
end;

procedure TTestProcedureFunction.TestFunctionStdCall;
begin
  ParseFunction('','','',ccStdCall);
  AssertFunc([],[],ccStdCall,0);
end;

procedure TTestProcedureFunction.TestProcedureOldFPCCall;
begin
  ParseProcedure('; oldfpccall;','');
  AssertProc([],[],ccoldfpccall,0);
end;

procedure TTestProcedureFunction.TestFunctionOldFPCCall;
begin
  ParseFunction('','','',ccOldFPCCall);
  AssertFunc([],[],ccOldFPCCall,0);
end;

procedure TTestProcedureFunction.TestProcedurePublic;
begin
  ParseProcedure('; public name ''myfunc'';','');
  AssertProc([pmPublic],[],ccDefault,0);
  AssertExpression('Public name',Proc.PublicName,pekString,'''myfunc''');
end;

procedure TTestProcedureFunction.TestProcedurePublicIdent;
begin
  ParseProcedure('; public name exportname;','');
  AssertProc([pmPublic],[],ccDefault,0);
  AssertExpression('Public name',Proc.PublicName,pekIdent,'exportname');
end;

procedure TTestProcedureFunction.TestFunctionPublic;
begin
  AddDeclaration('function A : Integer; public name exportname');
  ParseFunction;
  AssertFunc([pmPublic],[],ccDefault,0);
  AssertExpression('Public name',Func.PublicName,pekIdent,'exportname');
end;

procedure TTestProcedureFunction.TestProcedureCdeclPublic;
begin
  ParseProcedure('; cdecl; public name exportname;','');
  AssertProc([pmPublic],[],ccCDecl,0);
  AssertExpression('Public name',Proc.PublicName,pekIdent,'exportname');
end;

procedure TTestProcedureFunction.TestFunctionCdeclPublic;
begin
  AddDeclaration('function A : Integer; cdecl; public name exportname');
  ParseFunction;
  AssertFunc([pmPublic],[],ccCDecl,0);
  AssertExpression('Public name',Func.PublicName,pekIdent,'exportname');
end;

procedure TTestProcedureFunction.TestProcedureOverload;
begin
  ParseProcedure('; overload;','');
  AssertProc([pmOverload],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestFunctionOverload;
begin
  AddDeclaration('function A : Integer; overload');
  ParseFunction;
  AssertFunc([pmOverload],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestProcedureVarargs;
begin
  ParseProcedure('; varargs;','');
  AssertProc([],[ptmVarArgs],ccDefault,0);
end;

procedure TTestProcedureFunction.TestFunctionVarArgs;
begin
  AddDeclaration('function A : Integer; varargs');
  ParseFunction;
  AssertFunc([],[ptmVarArgs],ccDefault,0);
end;

procedure TTestProcedureFunction.TestProcedureCDeclVarargs;
begin
  ParseProcedure(';cdecl; varargs;','');
  AssertProc([],[ptmVarArgs],ccCDecl,0);
end;

procedure TTestProcedureFunction.TestFunctionCDeclVarArgs;
begin
  AddDeclaration('function A : Integer; cdecl; varargs');
  ParseFunction;
  AssertFunc([],[ptmVarArgs],ccCdecl,0);
end;

procedure TTestProcedureFunction.TestProcedureForwardInterface;
begin
  AddDeclaration('procedure A; forward;');
  AssertException(EParserError,@ParseProcedure);
end;

procedure TTestProcedureFunction.TestFunctionForwardInterface;
begin
  AddDeclaration('function A : integer; forward;');
  AssertException(EParserError,@ParseFunction);
end;

procedure TTestProcedureFunction.TestProcedureForward;
begin
  UseImplementation:=True;
  AddDeclaration('procedure A; forward;');
  ParseProcedure;
  AssertProc([pmforward],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestFunctionForward;
begin
  UseImplementation:=True;
  AddDeclaration('function A : integer; forward;');
  ParseFunction;
  AssertFunc([pmforward],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestProcedureFar;
begin
  AddDeclaration('procedure A; far;');
  ParseProcedure;
  AssertProc([pmfar],[ptmfar],ccDefault,0);
end;

procedure TTestProcedureFunction.TestFunctionFar;
begin
  AddDeclaration('function A : integer; far;');
  ParseFunction;
  AssertFunc([pmfar],[ptmfar],ccDefault,0);
end;

procedure TTestProcedureFunction.TestProcedureCdeclForward;
begin
  UseImplementation:=True;
  AddDeclaration('procedure A; cdecl; forward;');
  ParseProcedure;
  AssertProc([pmforward],[],ccCDecl,0);
end;

procedure TTestProcedureFunction.TestFunctionCDeclForward;
begin
  UseImplementation:=True;
  AddDeclaration('function A : integer; cdecl; forward;');
  ParseFunction;
  AssertFunc([pmforward],[],ccCDecl,0);
end;

procedure TTestProcedureFunction.TestProcedureCompilerProc;
begin
  ParseProcedure(';compilerproc;','');
  AssertProc([pmCompilerProc],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestProcedureNoReturn;
begin
  ParseProcedure(';noreturn;','');
  AssertProc([pmnoreturn],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestFunctionCompilerProc;
begin
  AddDeclaration('function A : Integer; compilerproc');
  ParseFunction;
  AssertFunc([pmCompilerProc],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestProcedureCDeclCompilerProc;
begin
  ParseProcedure(';cdecl;compilerproc;','');
  AssertProc([pmCompilerProc],[],ccCDecl,0);
end;

procedure TTestProcedureFunction.TestFunctionCDeclCompilerProc;
begin
  AddDeclaration('function A : Integer; cdecl; compilerproc');
  ParseFunction;
  AssertFunc([pmCompilerProc],[],ccCDecl,0);
end;

procedure TTestProcedureFunction.TestProcedureAssembler;
begin
  ParseProcedure(';assembler;','');
  AssertProc([pmAssembler],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestFunctionAssembler;
begin
  AddDeclaration('function A : Integer; assembler');
  ParseFunction;
  AssertFunc([pmAssembler],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestProcedureCDeclAssembler;
begin
  ParseProcedure(';cdecl;assembler;','');
  AssertProc([pmAssembler],[],ccCDecl,0);
end;

procedure TTestProcedureFunction.TestFunctionCDeclAssembler;
begin
  AddDeclaration('function A : Integer; cdecl; assembler');
  ParseFunction;
  AssertFunc([pmAssembler],[],ccCDecl,0);
end;

procedure TTestProcedureFunction.TestProcedureExport;
begin
  ParseProcedure(';export;','');
  AssertProc([pmExport],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestFunctionExport;
begin
  AddDeclaration('function A : Integer; export');
  ParseFunction;
  AssertFunc([pmExport],[],ccDefault,0);
end;

procedure TTestProcedureFunction.TestProcedureCDeclExport;
begin
  ParseProcedure('cdecl;export;','');
  AssertProc([pmExport],[],ccCDecl,0);
end;

procedure TTestProcedureFunction.TestFunctionCDeclExport;
begin
  AddDeclaration('function A : Integer; cdecl; export');
  ParseFunction;
  AssertFunc([pmExport],[],ccCDecl,0);
end;

procedure TTestProcedureFunction.TestProcedureExternal;
begin
  ParseProcedure(';external','');
  AssertProc([pmExternal],[],ccDefault,0);
  AssertNull('No Library name expression',Proc.LibraryExpr);
end;

procedure TTestProcedureFunction.TestFunctionExternal;
begin
  AddDeclaration('function A : Integer; external');
  ParseFunction;
  AssertFunc([pmExternal],[],ccDefault,0);
  AssertNull('No Library name expression',Func.LibraryExpr);
end;

procedure TTestProcedureFunction.CreateForwardTest;

begin
  With Source do
    begin
    Add('type');
    Add('');
    Add('Entity=object');
    Add('  function test:Boolean;');
    Add('end;');
    Add('');
    Add('Function Entity.test;');
    Add('begin');
    Add('end;');
    Add('');
    Add('begin');
    // End is added by ParseModule
    end;
end;

procedure TTestProcedureFunction.TestFunctionForwardNoReturnDelphi;
begin
  Source.Add('{$MODE DELPHI}');
  CreateForwardTest;
  ParseModule;
end;

procedure TTestProcedureFunction.TestFunctionForwardNoReturnNoDelphi;
begin
  CreateForwardTest;
  AssertException('Only in delphi mode can result be omitted',EParserError,@ParseModule);
end;

procedure TTestProcedureFunction.TestProcedureExternalLibName;
begin
  ParseProcedure(';external ''libname''','');
  AssertProc([pmExternal],[],ccDefault,0);
  AssertExpression('Library name expression',Proc.LibraryExpr,pekString,'''libname''');
end;

procedure TTestProcedureFunction.TestFunctionExternalLibName;
begin
  AddDeclaration('function A : Integer; external ''libname''');
  ParseFunction;
  AssertFunc([pmExternal],[],ccDefault,0);
  AssertExpression('Library name expression',Func.LibraryExpr,pekString,'''libname''');
end;

procedure TTestProcedureFunction.TestProcedureExternalLibNameName;
begin
  ParseProcedure(';external ''libname'' name ''symbolname''','');
  AssertProc([pmExternal],[],ccDefault,0);
  AssertExpression('Library name expression',Proc.LibraryExpr,pekString,'''libname''');
  AssertExpression('Library symbol expression',Proc.LibrarySymbolName,pekString,'''symbolname''');
end;

procedure TTestProcedureFunction.TestFunctionExternalLibNameName;
begin
  AddDeclaration('function A : Integer; external ''libname'' name ''symbolname''');
  ParseFunction;
  AssertFunc([pmExternal],[],ccDefault,0);
  AssertExpression('Library name expression',Func.LibraryExpr,pekString,'''libname''');
  AssertExpression('Library symbol expression',Func.LibrarySymbolName,pekString,'''symbolname''');
end;

procedure TTestProcedureFunction.TestProcedureExternalName;
begin
  ParseProcedure(';external name ''symbolname''','');
  AssertProc([pmExternal],[],ccDefault,0);
  AssertNull('No Library name expression',Proc.LibraryExpr);
  AssertExpression('Library symbol expression',Proc.LibrarySymbolName,pekString,'''symbolname''');
end;

procedure TTestProcedureFunction.TestFunctionExternalName;
begin
  AddDeclaration('function A : Integer; external name ''symbolname''');
  ParseFunction;
  AssertFunc([pmExternal],[],ccDefault,0);
  AssertNull('No Library name expression',Func.LibraryExpr);
  AssertExpression('Library symbol expression',Func.LibrarySymbolName,pekString,'''symbolname''');
end;

procedure TTestProcedureFunction.TestProcedureCdeclExternal;
begin
  ParseProcedure('; cdecl; external','');
  AssertProc([pmExternal],[],ccCdecl,0);
  AssertNull('No Library name expression',Proc.LibraryExpr);
end;

procedure TTestProcedureFunction.TestFunctionCdeclExternal;
begin
  AddDeclaration('function A : Integer; cdecl; external');
  ParseFunction;
  AssertFunc([pmExternal],[],ccCdecl,0);
  AssertNull('No Library name expression',Func.LibraryExpr);
end;

procedure TTestProcedureFunction.TestProcedureCdeclExternalLibName;
begin
  ParseProcedure('; cdecl; external ''libname''','');
  AssertProc([pmExternal],[],ccCdecl,0);
  AssertExpression('Library name expression',Proc.LibraryExpr,pekString,'''libname''');
end;

procedure TTestProcedureFunction.TestFunctionCdeclExternalLibName;
begin
  AddDeclaration('function A : Integer; cdecl; external ''libname''');
  ParseFunction;
  AssertFunc([pmExternal],[],ccCdecl,0);
  AssertExpression('Library name expression',Func.LibraryExpr,pekString,'''libname''');
end;

procedure TTestProcedureFunction.TestProcedureCdeclExternalLibNameName;
begin
  ParseProcedure('; cdecl; external ''libname'' name ''symbolname''','');
  AssertProc([pmExternal],[],ccCdecl,0);
  AssertExpression('Library name expression',Proc.LibraryExpr,pekString,'''libname''');
  AssertExpression('Library symbol expression',Proc.LibrarySymbolName,pekString,'''symbolname''');
end;

procedure TTestProcedureFunction.TestFunctionCdeclExternalLibNameName;
begin
  AddDeclaration('function A : Integer; cdecl; external ''libname'' name ''symbolname''');
  ParseFunction;
  AssertFunc([pmExternal],[],ccCdecl,0);
  AssertExpression('Library name expression',Func.LibraryExpr,pekString,'''libname''');
  AssertExpression('Library symbol expression',Func.LibrarySymbolName,pekString,'''symbolname''');
end;

procedure TTestProcedureFunction.TestProcedureCdeclExternalName;
begin
  ParseProcedure('; cdecl; external name ''symbolname''','');
  AssertProc([pmExternal],[],ccCdecl,0);
  AssertNull('No Library name expression',Proc.LibraryExpr);
  AssertExpression('Library symbol expression',Proc.LibrarySymbolName,pekString,'''symbolname''');
end;

procedure TTestProcedureFunction.TestFunctionCdeclExternalName;
begin
  AddDeclaration('function A : Integer; cdecl; external name ''symbolname''');
  ParseFunction;
  AssertFunc([pmExternal],[],ccCdecl,0);
  AssertNull('No Library name expression',Func.LibraryExpr);
  AssertExpression('Library symbol expression',Func.LibrarySymbolName,pekString,'''symbolname''');
end;

procedure TTestProcedureFunction.TestFunctionAlias;
begin
  AddDeclaration('function A : Integer; alias: ''myalias''');
  ParseFunction;
  AssertFunc([],[],ccDefault,0);
  AssertEquals('Alias name','''myalias''',Func.AliasName);
end;

procedure TTestProcedureFunction.TestOperatorNamedResult;
begin
  AddDeclaration('operator = (a,b : T) z : Integer;');
  ParseOperator;
  AssertEquals('Correct operator type',otEqual,FOperator.OperatorType);
end;

procedure TTestProcedureFunction.TestProcedureAlias;
begin
  AddDeclaration('Procedure A; Alias : ''myalias''');
  ParseProcedure;
  AssertProc([],[],ccDefault,0);
  AssertEquals('Alias name','''myalias''',Proc.AliasName);
end;

procedure TTestProcedureFunction.TestOperatorTokens;

Var
  t : TOperatorType;
  s : string;

begin
  For t:=otMul to High(TOperatorType) do
    begin
    if OperatorTokens[t]='' then continue;
    // No way to distinguish between logical/bitwise or/and/Xor
    if t in [otBitWiseOr,otBitwiseAnd,otbitwiseXor] then continue;

    S:=GetEnumName(TypeInfo(TOperatorType),Ord(T));
    ResetParser;
    if t in UnaryOperators then
      AddDeclaration(Format('operator %s (a: Integer) : te',[OperatorTokens[t]]))
    else
      AddDeclaration(Format('operator %s (a: Integer; b: integer) : te',[OperatorTokens[t]]));
    ParseOperator;
    AssertEquals(S+': Token based ',Not (T in [otInc,otDec,otEnumerator]),FOperator.TokenBased);
    AssertEquals(S+': Correct operator type',T,FOperator.OperatorType);
    if t in UnaryOperators then
      AssertEquals(S+': Correct operator name',format('%s(Integer):te',[OperatorNames[t]]),FOperator.Name)
    else
      AssertEquals(S+': Correct operator name',format('%s(Integer,Integer):te',[OperatorNames[t]]),FOperator.Name);
    end;
end;

procedure TTestProcedureFunction.TestOperatorNames;

Var
  t : TOperatorType;
  S: String;

begin
  For t:=Succ(otUnknown) to High(TOperatorType) do
    begin
    if OperatorNames[t]='' then continue;
    // otInitialize has no result
    if t=otInitialize then continue;
    writeln('TTestProcedureFunction.TestOperatorTokens ',t);
    S:=GetEnumName(TypeInfo(TOperatorType),Ord(T));
    ResetParser;
    if t in UnaryOperators then
      AddDeclaration(Format('operator %s (a: Integer) : te',[OperatorNames[t]]))
    else
      AddDeclaration(Format('operator %s (a: Integer; b: integer) : te',[OperatorNames[t]]));
    ParseOperator;
    AssertEquals(S+': Token based',t in [otIn],FOperator.TokenBased);
    AssertEquals(S+': Correct operator type',T,FOperator.OperatorType);
    if t in UnaryOperators then
      AssertEquals('Correct operator name',format('%s(Integer):te',[OperatorNames[t]]),FOperator.Name)
    else
      AssertEquals('Correct operator name',format('%s(Integer,Integer):te',[OperatorNames[t]]),FOperator.Name);
    end;
end;

procedure TTestProcedureFunction.TestAssignOperatorAfterObject;
begin
  Add('unit afile;');
  Add('{$mode delphi}');
  Add('interface');
  Add('type');
  Add('  TA =object');
  Add('      data:integer;');
  Add('      function transpose:integer;');
  Add('  end;');
  Add('');
  Add('operator := (const v:Tvector2_single) result:Tvector2_double;');
  Add('implementation');
  EndSource;
  Parser.Options:=[po_delphi];
  ParseModule;
end;

procedure TTestProcedureFunction.TestFunctionNoResult;
begin
  Add('unit afile;');
  Add('{$mode delphi}');
  Add('interface');
  Add('function TestDelphiModeFuncs(d:double):string;');
  Add('implementation');
  Add('function TestDelphiModeFuncs;');
  Add('begin');
  Add('end;');
  EndSource;
  Parser.Options:=[po_delphi];
  ParseModule;
end;

procedure TTestProcedureFunction.SetUp;
begin
   Inherited;
end;

procedure TTestProcedureFunction.TearDown;
begin
   Inherited;
end;

procedure TTestProcedureFunction.AssertComment;
begin
  AssertEquals('Correct comment',' A comment'+sLineBreak,FProc.DocComment);
end;

initialization

  RegisterTest(TTestProcedureFunction);
end.

