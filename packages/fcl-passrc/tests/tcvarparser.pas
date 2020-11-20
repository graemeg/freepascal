unit tcvarparser;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, pastree, pscanner,
  tcbaseparser, testregistry;

Type
  { TTestVarParser }

  TTestVarParser = Class(TTestParser)
  private
    FHint: string;
    FVar: TPasVariable;
  Protected
    Function ParseVar(ASource : String; Const AHint : String = '') : TPasVariable; virtual; overload;
    Procedure AssertVariableType(Const ATypeName : String);
    Procedure AssertVariableType(Const AClass : TClass);
    Procedure AssertParseVarError(ASource : String);
    Property TheVar : TPasVariable Read FVar;
    Property Hint : string Read FHint Write FHint;
    procedure SetUp; override;
    Procedure TearDown; override;
  Published
    Procedure TestSimpleVar;
    Procedure TestSimpleVarAbsoluteName;
    Procedure TestSimpleVarHelperName;
    procedure TestSimpleVarHelperType;
    Procedure TestSimpleVarDeprecated;
    Procedure TestSimpleVarPlatform;
    Procedure TestSimpleVarInitialized;
    procedure TestSimpleVarInitializedDeprecated;
    procedure TestSimpleVarInitializedPlatform;
    Procedure TestSimpleVarAbsolute;
    Procedure TestSimpleVarAbsoluteAddress;
    Procedure TestSimpleVarAbsoluteDot;
    Procedure TestSimpleVarAbsolute2Dots;
    Procedure TestVarProcedure;
    procedure TestVarProcedureCdecl;
    procedure TestVarFunctionFar;
    Procedure TestVarFunctionINitialized;
    Procedure TestVarProcedureDeprecated;
    Procedure TestVarRecord;
    Procedure TestVarRecordDeprecated;
    Procedure TestVarRecordPlatform;
    Procedure TestVarArray;
    Procedure TestVarArrayDeprecated;
    Procedure TestVarDynArray;
    Procedure TestVarExternal;
    Procedure TestVarExternalLib;
    Procedure TestVarExternalLibName;
    procedure TestVarExternalNoSemiColon;
    procedure TestVarExternalLibNoName;
    Procedure TestVarCVar;
    Procedure TestVarCVarExternal;
    Procedure TestVarPublic;
    Procedure TestVarPublicName;
    Procedure TestVarDeprecatedExternalName;
    Procedure TestVarHintPriorToInit;
  end;

implementation

uses typinfo;

{ TTestVarParser }

function TTestVarParser.ParseVar(ASource: String; const AHint: String
  ): TPasVariable;
Var
  D : String;
begin
  Hint:=AHint;
  Add('Var');
  D:='A : '+ASource;
  If Hint<>'' then
    D:=D+' '+Hint;
  Add('  '+D+';');
//  Writeln(source.text);
  ParseDeclarations;
  AssertEquals('One variable definition',1,Declarations.Variables.Count);
  AssertEquals('First declaration is type definition.',TPasVariable,TObject(Declarations.Variables[0]).ClassType);
  Result:=TPasVariable(Declarations.Variables[0]);
  AssertEquals('First declaration has correct name.','A',Result.Name);
  FVar:=Result;
  Definition:=Result;
  if (Hint<>'') then
    CheckHint(TPasMemberHint(Getenumvalue(typeinfo(TPasMemberHint),'h'+Hint)));
end;

procedure TTestVarParser.AssertVariableType(const ATypeName: String);
begin
  AssertVariableType(TPasUnresolvedTypeRef);
  AssertEquals('Correct unresolved type name',ATypeName,theVar.VarType.Name);
end;

procedure TTestVarParser.AssertVariableType(const AClass: TClass);
begin
  AssertNotNull('Have variable type',theVar.VarType);
  AssertEquals('Correct type class',AClass,theVar.VarType.ClassType);
end;

procedure TTestVarParser.AssertParseVarError(ASource: String);
begin
  try
    ParseVar(ASource,'');
    Fail('Expected parser error');
  except
    // all OK.
  end;
end;

procedure TTestVarParser.SetUp;
begin
  inherited SetUp;
  FHint:='';
  FVar:=Nil;
end;

procedure TTestVarParser.TearDown;
begin
  FVar:=Nil;
  inherited TearDown;
end;

procedure TTestVarParser.TestSimpleVar;
begin
  ParseVar('b','');
  AssertVariableType('b');
end;

procedure TTestVarParser.TestSimpleVarAbsoluteName;
Var
  R : TPasVariable;

begin
  Add('Var');
  Add('  Absolute : integer;');
//  Writeln(source.text);
  ParseDeclarations;
  AssertEquals('One variable definition',1,Declarations.Variables.Count);
  AssertEquals('First declaration is type definition.',TPasVariable,TObject(Declarations.Variables[0]).ClassType);
  R:=TPasVariable(Declarations.Variables[0]);
  AssertEquals('First declaration has correct name.','Absolute',R.Name);
end;

procedure TTestVarParser.TestSimpleVarHelperName;

Var
  R : TPasVariable;

begin
  Add('Var');
  Add('  Helper : integer;');
//  Writeln(source.text);
  ParseDeclarations;
  AssertEquals('One variable definition',1,Declarations.Variables.Count);
  AssertEquals('First declaration is type definition.',TPasVariable,TObject(Declarations.Variables[0]).ClassType);
  R:=TPasVariable(Declarations.Variables[0]);
  AssertEquals('First declaration has correct name.','Helper',R.Name);
end;

procedure TTestVarParser.TestSimpleVarHelperType;
begin
  ParseVar('helper','');
  AssertVariableType('helper');
end;

procedure TTestVarParser.TestSimpleVarDeprecated;
begin
  ParseVar('b','deprecated');
  AssertVariableType('b');
end;

procedure TTestVarParser.TestSimpleVarPlatform;
begin
  ParseVar('b','platform');
  AssertVariableType('b');
end;

procedure TTestVarParser.TestSimpleVarInitialized;
begin
  ParseVar('b = 123','');
  AssertVariableType('b');
  AssertNotNull(TheVar.expr);
  AssertExpression('Variable value',TheVar.expr,pekNumber,'123');
end;

procedure TTestVarParser.TestSimpleVarInitializedDeprecated;
begin
  ParseVar('b = 123','deprecated');
  AssertVariableType('b');
  AssertNotNull(TheVar.expr);
  AssertExpression('Variable value',TheVar.expr,pekNumber,'123');
end;

procedure TTestVarParser.TestSimpleVarInitializedPlatform;
begin
  ParseVar('b = 123','platform');
  AssertVariableType('b');
  AssertNotNull(TheVar.expr);
  AssertExpression('Variable value',TheVar.expr,pekNumber,'123');
end;

procedure TTestVarParser.TestSimpleVarAbsolute;
begin
  ParseVar('q absolute v','');
  AssertVariableType('q');
  AssertExpression('correct absolute location',TheVar.AbsoluteExpr,pekIdent,'v');
end;

procedure TTestVarParser.TestSimpleVarAbsoluteAddress;
begin
  ParseVar('q absolute $123','');
  AssertVariableType('q');
  AssertExpression('correct absolute location',TheVar.AbsoluteExpr,pekNumber,'$123');
end;

procedure TTestVarParser.TestSimpleVarAbsoluteDot;
var
  B: TBinaryExpr;
begin
  ParseVar('q absolute v.w','');
  AssertVariableType('q');
  B:=AssertExpression('binary',TheVar.AbsoluteExpr,eopSubIdent);
  AssertExpression('correct absolute expr v',B.left,pekIdent,'v');
  AssertExpression('correct absolute expr w',B.right,pekIdent,'w');
end;

procedure TTestVarParser.TestSimpleVarAbsolute2Dots;
var
  B: TBinaryExpr;
begin
  ParseVar('q absolute v.w.x','');
  AssertVariableType('q');
  B:=AssertExpression('binary',TheVar.AbsoluteExpr,eopSubIdent);
  AssertExpression('correct absolute expr x',B.right,pekIdent,'x');
  B:=AssertExpression('binary',B.left,eopSubIdent);
  AssertExpression('correct absolute expr w',B.right,pekIdent,'w');
  AssertExpression('correct absolute expr v',B.left,pekIdent,'v');
end;

procedure TTestVarParser.TestVarProcedure;
begin
  ParseVar('procedure','');
  AssertVariableType(TPasProcedureType);
end;

procedure TTestVarParser.TestVarProcedureCdecl;
begin
  ParseVar('procedure; cdecl;','');
  AssertVariableType(TPasProcedureType);
end;

procedure TTestVarParser.TestVarFunctionFar;
begin
  ParseVar('function (cinfo : j_decompress_ptr) : int; far;','');
  AssertVariableType(TPasFunctionType);
end;

procedure TTestVarParser.TestVarFunctionINitialized;
begin
  ParseVar('function (device: pointer): pointer; cdecl = nil','');
  AssertVariableType(TPasFunctionType);
end;

procedure TTestVarParser.TestVarProcedureDeprecated;
begin
  ParseVar('procedure','deprecated');
  AssertVariableType(TPasProcedureType);
end;

procedure TTestVarParser.TestVarRecord;

Var
  R : TPasRecordtype;
begin
  ParseVar('record x,y : intger; end','');
  AssertVariableType(TPasRecordType);
  R:=TheVar.VarType as TPasRecordType;
  AssertEquals('Correct number of fields',2,R.Members.Count);
end;

procedure TTestVarParser.TestVarRecordDeprecated;
Var
  R : TPasRecordtype;
begin
  ParseVar('record x,y : integer; end','deprecated');
  AssertVariableType(TPasRecordType);
  R:=TheVar.VarType as TPasRecordType;
  AssertEquals('Correct number of fields',2,R.Members.Count);
end;

procedure TTestVarParser.TestVarRecordPlatform;
Var
  R : TPasRecordtype;
begin
  ParseVar('record x,y : integer; end','platform');
  AssertVariableType(TPasRecordType);
  R:=TheVar.VarType as TPasRecordType;
  AssertEquals('Correct number of fields',2,R.Members.Count);
end;

procedure TTestVarParser.TestVarArray;

Var
  R : TPasArrayType;

begin
  ParseVar('Array[1..20] of integer','');
  AssertVariableType(TPasArrayType);
  R:=TheVar.VarType as TPasArrayType;
  AssertNotNull('Correct array type name',R.ElType);
  AssertEquals('Correct array type name',TPasunresolvedTypeRef,R.ElType.ClassType);
end;

procedure TTestVarParser.TestVarArrayDeprecated;

Var
  R : TPasArrayType;

begin
  ParseVar('Array[1..20] of integer','Deprecated');
  AssertVariableType(TPasArrayType);
  R:=TheVar.VarType as TPasArrayType;
  AssertNotNull('Correct array type name',R.ElType);
  AssertEquals('Correct array type name',TPasunresolvedTypeRef,R.ElType.ClassType);
end;

procedure TTestVarParser.TestVarDynArray;

Var
  R : TPasArrayType;

begin
  ParseVar('Array of integer','');
  AssertVariableType(TPasArrayType);
  R:=TheVar.VarType as TPasArrayType;
  AssertEquals('No index','',R.IndexRange);
  AssertNotNull('Correct array type name',R.ElType);
  AssertEquals('Correct array type name',TPasunresolvedTypeRef,R.ElType.ClassType);
end;

procedure TTestVarParser.TestVarExternal;
begin
  ParseVar('integer; external','');
  AssertEquals('Variable modifiers',[vmexternal],TheVar.VarModifiers);
end;

procedure TTestVarParser.TestVarExternalNoSemiColon;
begin
  ParseVar('integer external','');
  AssertEquals('Variable modifiers',[vmexternal],TheVar.VarModifiers);
end;

procedure TTestVarParser.TestVarExternalLib;
begin
  ParseVar('integer; external name ''mylib''','');
  AssertEquals('Variable modifiers',[vmexternal],TheVar.VarModifiers);
  AssertNull('Library name',TheVar.LibraryName);
  AssertNotNull('Library symbol',TheVar.ExportName);
end;


procedure TTestVarParser.TestVarExternalLibNoName;
begin
  // Found in e.g.apache headers
  ParseVar('integer; external ''mylib''','');
  AssertEquals('Variable modifiers',[vmexternal],TheVar.VarModifiers);
  AssertNotNull('Library name',TheVar.LibraryName);

end;


procedure TTestVarParser.TestVarExternalLibName;
begin
  ParseVar('integer; external ''mylib'' name ''de''','');
  AssertEquals('Variable modifiers',[vmexternal],TheVar.VarModifiers);
  AssertNotNull('Library name',TheVar.LibraryName);
  AssertNotNull('Library symbol',TheVar.ExportName);
end;

procedure TTestVarParser.TestVarCVar;
begin
  ParseVar('integer; cvar','');
  AssertEquals('Variable modifiers',[vmcvar],TheVar.VarModifiers);
end;

procedure TTestVarParser.TestVarCVarExternal;
begin
  ParseVar('integer; cvar;external','');
  AssertEquals('Variable modifiers',[vmcvar,vmexternal],TheVar.VarModifiers);
end;

procedure TTestVarParser.TestVarPublic;
begin
  ParseVar('integer; public','');
  AssertEquals('Variable modifiers',[vmpublic],TheVar.VarModifiers);
end;

procedure TTestVarParser.TestVarPublicName;
begin
  ParseVar('integer; public name ''ce''','');
  AssertEquals('Variable modifiers',[vmpublic],TheVar.VarModifiers);
  AssertNotNull('Public export name',TheVar.ExportName);
end;

procedure TTestVarParser.TestVarDeprecatedExternalName;
begin
  ParseVar('integer deprecated; external name ''me''','');
  CheckHint(TPasMemberHint(Getenumvalue(typeinfo(TPasMemberHint),'hdeprecated')));
  AssertEquals('Variable modifiers',[vmexternal],TheVar.VarModifiers);
  AssertNull('Library name',TheVar.LibraryName);
  AssertNotNull('Library symbol',TheVar.ExportName);
end;

procedure TTestVarParser.TestVarHintPriorToInit;

Var
  E : TBoolConstExpr;

begin
  ParseVar('boolean platform = false','');
  CheckHint(TPasMemberHint(Getenumvalue(typeinfo(TPasMemberHint),'hplatform')));
  AssertNotNull('Correctly initialized',Thevar.Expr);
  AssertEquals('Correctly initialized',TBoolConstExpr,Thevar.Expr.ClassType);
  E:=Thevar.Expr as TBoolConstExpr;
  AssertEquals('Correct initialization value',False, E.Value);
end;

initialization

  RegisterTests([TTestVarParser]);
end.

