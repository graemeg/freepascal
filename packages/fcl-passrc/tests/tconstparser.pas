unit tconstparser;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, pastree, pscanner, tcbaseparser, testregistry, pparser;

Type
    { TTestConstParser }

  TTestConstParser = Class(TTestParser)
  private
    FConst: TPasConst;
    FExpr: TPasExpr;
    FHint : string;
    FTyped: String;
    procedure DoParseConstUnTypedRange;
  Protected
    Function ParseConst(ASource : String) : TPasConst;
    Procedure CheckExprNameKindClass(AKind : TPasExprKind; AClass : TClass);
    Property TheConst : TPasConst Read FConst;
    Property TheExpr : TPasExpr Read FExpr;
    Property Hint : string Read FHint Write FHint;
    Property Typed : String Read FTyped Write FTyped;
    procedure SetUp; override;
  Public
    Procedure DoTestSimpleIntConst;
    Procedure DoTestSimpleFloatConst;
    Procedure DoTestSimpleStringConst;
    Procedure DoTestSimpleNilConst;
    Procedure DoTestSimpleBoolConst;
    Procedure DoTestSimpleIdentifierConst;
    Procedure DoTestSimpleSetConst;
    Procedure DoTestSimpleExprConst;
  Published
    Procedure TestSimpleIntConst;
    Procedure TestSimpleFloatConst;
    Procedure TestSimpleStringConst;
    Procedure TestSimpleNilConst;
    Procedure TestSimpleBoolConst;
    Procedure TestSimpleIdentifierConst;
    Procedure TestSimpleSetConst;
    Procedure TestSimpleExprConst;
    Procedure TestSimpleAbsoluteConst;
    Procedure TestSimpleIntConstDeprecatedMsg;
    Procedure TestSimpleIntConstDeprecated;
    Procedure TestSimpleFloatConstDeprecated;
    Procedure TestSimpleStringConstDeprecated;
    Procedure TestSimpleNilConstDeprecated;
    Procedure TestSimpleBoolConstDeprecated;
    Procedure TestSimpleIdentifierConstDeprecated;
    Procedure TestSimpleSetConstDeprecated;
    Procedure TestSimpleExprConstDeprecated;
    Procedure TestSimpleIntConstPlatform;
    Procedure TestSimpleFloatConstPlatform;
    Procedure TestSimpleStringConstPlatform;
    Procedure TestSimpleNilConstPlatform;
    Procedure TestSimpleBoolConstPlatform;
    Procedure TestSimpleIdentifierConstPlatform;
    Procedure TestSimpleSetConstPlatform;
    Procedure TestSimpleExprConstPlatform;
    Procedure TestSimpleIntConstExperimental;
    Procedure TestSimpleFloatConstExperimental;
    Procedure TestSimpleStringConstExperimental;
    Procedure TestSimpleNilConstExperimental;
    Procedure TestSimpleBoolConstExperimental;
    Procedure TestSimpleIdentifierConstExperimental;
    Procedure TestSimpleSetConstExperimental;
    Procedure TestSimpleExprConstExperimental;
    Procedure TestTypedIntConst;
    Procedure TestTypedFloatConst;
    Procedure TestTypedStringConst;
    Procedure TestTypedNilConst;
    Procedure TestTypedBoolConst;
    Procedure TestTypedIdentifierConst;
    Procedure TestTypedSetConst;
    Procedure TestTypedExprConst;
    Procedure TestRecordConst;
    Procedure TestArrayConst;
    Procedure TestRangeConst;
    Procedure TestRangeConstUnTyped;
    Procedure TestArrayOfRangeConst;
  end;

  { TTestResourcestringParser }

  TTestResourcestringParser = Class(TTestParser)
  private
    FExpr: TPasExpr;
    FHint : string;
    FTheStr: TPasResString;
  Protected
    Function ParseResourcestring(ASource : String) : TPasResString;
    Procedure CheckExprNameKindClass(AKind : TPasExprKind; AClass : TClass);
    Property Hint : string Read FHint Write FHint;
    Property TheStr : TPasResString Read FTheStr;
    Property TheExpr : TPasExpr Read FExpr;
  Public
    Procedure DoTestSimple;
    Procedure DoTestSum;
    Procedure DoTestSum2;
  Published
    Procedure TestSimple;
    Procedure TestSimpleDeprecated;
    Procedure TestSimplePlatform;
    Procedure TestSum1;
    Procedure TestSum1Deprecated;
    Procedure TestSum1Platform;
    Procedure TestSum2;
    Procedure TestSum2Deprecated;
    Procedure TestSum2Platform;
  end;


implementation
{ TTestConstParser }

function TTestConstParser.ParseConst(ASource: String): TPasConst;

Var
  D : String;
begin
  Add('Const');
  D:=' A ';
  If (Typed<>'') then
    D:=D+' : '+Typed+' ';
  D:=D+' = '+ASource;
  If Hint<>'' then
    D:=D+' '+Hint;
  Add('  '+D+';');
  ParseDeclarations;
  AssertEquals('One constant definition',1,Declarations.Consts.Count);
  AssertEquals('First declaration is constant definition.',TPasConst,TObject(Declarations.Consts[0]).ClassType);
  Result:=TPasConst(Declarations.Consts[0]);
  AssertNotNull(Result.Expr);
  FExpr:=Result.Expr;
  FConst:=Result;
  Definition:=Result;
end;


procedure TTestConstParser.CheckExprNameKindClass(
  AKind: TPasExprKind; AClass : TClass);
begin
  AssertEquals('Correct name','A',TheConst.Name);
  AssertExpression('Const', TheExpr,aKind,AClass);
end;

procedure TTestConstParser.SetUp;
begin
  inherited SetUp;
  Hint:='';
end;

procedure TTestConstParser.DoTestSimpleIntConst;

begin
  ParseConst('1');
  AssertExpression('Integer Const',TheExpr,pekNumber,'1');
end;

procedure TTestConstParser.DoTestSimpleFloatConst;
begin
  ParseConst('1.2');
  AssertExpression('Float const', TheExpr,pekNumber,'1.2');
end;

procedure TTestConstParser.DoTestSimpleStringConst;
begin
  ParseConst('''test''');
  AssertExpression('String const', TheExpr,pekString,'''test''');
end;

procedure TTestConstParser.DoTestSimpleNilConst;
begin
  ParseConst('Nil');
  CheckExprNameKindClass(pekNil,TNilExpr);
end;

procedure TTestConstParser.DoTestSimpleBoolConst;
begin
  ParseConst('True');
  CheckExprNameKindClass(pekBoolConst,TBoolconstExpr);
  AssertEquals('Correct expression value',True,TBoolconstExpr(TheExpr).Value);
end;

procedure TTestConstParser.DoTestSimpleIdentifierConst;
begin
  ParseConst('taCenter');
  AssertExpression('Enumeration const', theExpr,pekIdent,'taCenter');
end;

procedure TTestConstParser.DoTestSimpleSetConst;
begin
  ParseConst('[taLeftJustify,taRightJustify]');
  CheckExprNameKindClass(pekSet,TParamsExpr);
  AssertEquals('Correct set count',2,Length(TParamsExpr(TheExpr).Params));
  AssertExpression('Set element 1',TParamsExpr(TheExpr).Params[0],pekIdent,'taLeftJustify');
  AssertExpression('Set element 2',TParamsExpr(TheExpr).Params[1],pekIdent,'taRightJustify');
end;

procedure TTestConstParser.DoTestSimpleExprConst;

Var
  B : TBinaryExpr;

begin
  ParseConst('1 + 2');
  CheckExprNameKindClass(pekBinary,TBinaryExpr);
  B:=TBinaryExpr(TheExpr);
  TAssert.AssertSame('B.Left.Parent=B',B,B.left.Parent);
  TAssert.AssertSame('B.right.Parent=B',B,B.right.Parent);
  AssertExpression('Left expression',B.Left,pekNumber,'1');
  AssertExpression('Right expression',B.Right,pekNumber,'2');
end;

procedure TTestConstParser.TestSimpleIntConst;
begin
  DoTestSimpleIntConst
end;

procedure TTestConstParser.TestSimpleFloatConst;
begin
  DoTestSimpleFloatConst
end;

procedure TTestConstParser.TestSimpleStringConst;
begin
  DoTestSimpleStringConst
end;

procedure TTestConstParser.TestSimpleNilConst;
begin
  DoTestSimpleNilConst
end;

procedure TTestConstParser.TestSimpleBoolConst;
begin
  DoTestSimpleBoolConst
end;

procedure TTestConstParser.TestSimpleIdentifierConst;
begin
  DoTestSimpleIdentifierConst
end;

procedure TTestConstParser.TestSimpleSetConst;
begin
  DoTestSimpleSetConst
end;

procedure TTestConstParser.TestSimpleExprConst;
begin
  DoTestSimpleExprConst;
end;

procedure TTestConstParser.TestSimpleAbsoluteConst;

// Found in xi.pp

begin
  Add('Const');
  Add('  Absolute = 1;');
  ParseDeclarations;
  AssertEquals('One constant definition',1,Declarations.Consts.Count);
  AssertEquals('First declaration is constant definition.',TPasConst,TObject(Declarations.Consts[0]).ClassType);

end;

procedure TTestConstParser.TestSimpleIntConstDeprecatedMsg;
begin
  Hint:='deprecated ''this is old''' ;
  DoTestSimpleIntConst;
  CheckHint(hDeprecated);
end;

procedure TTestConstParser.TestSimpleIntConstDeprecated;
begin
  Hint:='deprecated';
  DoTestSimpleIntConst;
  CheckHint(hDeprecated);
end;

procedure TTestConstParser.TestSimpleFloatConstDeprecated;
begin
  Hint:='deprecated';
  DoTestSimpleIntConst;
  CheckHint(hDeprecated);
end;

procedure TTestConstParser.TestSimpleStringConstDeprecated;
begin
  Hint:='deprecated';
  DoTestSimpleStringConst;
  CheckHint(hDeprecated);
end;

procedure TTestConstParser.TestSimpleNilConstDeprecated;
begin
  Hint:='deprecated';
  DoTestSimpleNilConst;
  CheckHint(hDeprecated);
end;

procedure TTestConstParser.TestSimpleBoolConstDeprecated;
begin
  Hint:='deprecated';
  DoTestSimpleBoolConst;
  CheckHint(hDeprecated);
end;

procedure TTestConstParser.TestSimpleIdentifierConstDeprecated;
begin
  Hint:='deprecated';
  DoTestSimpleIdentifierConst;
  CheckHint(hDeprecated);
end;

procedure TTestConstParser.TestSimpleSetConstDeprecated;
begin
  Hint:='deprecated';
  DoTestSimpleSetConst;
  CheckHint(hDeprecated);
end;

procedure TTestConstParser.TestSimpleExprConstDeprecated;
begin
  Hint:='deprecated';
  DoTestSimpleExprConst;
  CheckHint(hDeprecated);
end;

procedure TTestConstParser.TestSimpleIntConstPlatform;
begin
  Hint:='Platform';
  DoTestSimpleIntConst;
  CheckHint(hPlatform);
end;

procedure TTestConstParser.TestSimpleFloatConstPlatform;
begin
  Hint:='Platform';
  DoTestSimpleIntConst;
  CheckHint(hPlatform);
end;

procedure TTestConstParser.TestSimpleStringConstPlatform;
begin
  Hint:='Platform';
  DoTestSimpleStringConst;
  CheckHint(hPlatform);
end;

procedure TTestConstParser.TestSimpleNilConstPlatform;
begin
  Hint:='Platform';
  DoTestSimpleNilConst;
  CheckHint(hPlatform);
end;

procedure TTestConstParser.TestSimpleBoolConstPlatform;
begin
  Hint:='Platform';
  DoTestSimpleBoolConst;
  CheckHint(hPlatform);
end;

procedure TTestConstParser.TestSimpleIdentifierConstPlatform;
begin
  Hint:='Platform';
  DoTestSimpleIdentifierConst;
  CheckHint(hPlatform);
end;

procedure TTestConstParser.TestSimpleExprConstPlatform;
begin
  Hint:='Platform';
  DoTestSimpleExprConst;
  CheckHint(hPlatform);
end;

procedure TTestConstParser.TestSimpleSetConstPlatform;
begin
  Hint:='Platform';
  DoTestSimpleSetConst;
  CheckHint(hPlatform);
end;

procedure TTestConstParser.TestSimpleIntConstExperimental;
begin
  Hint:='Experimental';
  DoTestSimpleIntConst;
  CheckHint(hExperimental);
end;

procedure TTestConstParser.TestSimpleFloatConstExperimental;
begin
  Hint:='Experimental';
  DoTestSimpleIntConst;
  CheckHint(hExperimental);
end;

procedure TTestConstParser.TestSimpleStringConstExperimental;
begin
  Hint:='Experimental';
  DoTestSimpleStringConst;
  CheckHint(hExperimental);
end;

procedure TTestConstParser.TestSimpleNilConstExperimental;
begin
  Hint:='Experimental';
  DoTestSimpleNilConst;
  CheckHint(hExperimental);
end;

procedure TTestConstParser.TestSimpleBoolConstExperimental;
begin
  Hint:='Experimental';
  DoTestSimpleBoolConst;
  CheckHint(hExperimental);
end;

procedure TTestConstParser.TestSimpleIdentifierConstExperimental;
begin
  Hint:='Experimental';
  DoTestSimpleIdentifierConst;
  CheckHint(hExperimental);
end;

procedure TTestConstParser.TestSimpleSetConstExperimental;
begin
  Hint:='Experimental';
  DoTestSimpleSetConst;
  CheckHint(hExperimental);
end;

procedure TTestConstParser.TestSimpleExprConstExperimental;
begin
  Hint:='Experimental';
  DoTestSimpleExprConst;
  CheckHint(hExperimental);
end;

procedure TTestConstParser.TestTypedIntConst;
begin
  Typed:='Integer';
  DoTestSimpleIntConst
end;

procedure TTestConstParser.TestTypedFloatConst;
begin
  Typed:='Double';
  DoTestSimpleFloatConst
end;

procedure TTestConstParser.TestTypedStringConst;
begin
  Typed:='shortstring';
  DoTestSimpleStringConst
end;

procedure TTestConstParser.TestTypedNilConst;
begin
  Typed:='PChar';
  DoTestSimpleNilConst
end;

procedure TTestConstParser.TestTypedBoolConst;
begin
  Typed:='Boolean';
  DoTestSimpleBoolConst
end;

procedure TTestConstParser.TestTypedIdentifierConst;
begin
  Typed:='TAlign';
  DoTestSimpleIdentifierConst
end;

procedure TTestConstParser.TestTypedSetConst;
begin
  Typed:='TAligns';
  DoTestSimpleSetConst
end;

procedure TTestConstParser.TestTypedExprConst;
begin
  Typed:='ShortInt';
  DoTestSimpleExprConst;
end;

procedure TTestConstParser.TestRecordConst;
Var
  R : TRecordValues;
  Fi : TRecordValuesItem;
begin
  Typed := 'TPoint';
  ParseConst('(x:1;y: 2)');
  AssertEquals('Record Values',TRecordValues,TheExpr.ClassType);
  R:=TheExpr as TRecordValues;
  AssertEquals('Expression list of ',pekListOfExp,TheExpr.Kind);
  AssertEquals('2 elements',2,Length(R.Fields));
  FI:=R.Fields[0];
  AssertEquals('Name field 1','x',Fi.Name);
  AssertExpression('Field 1 value',Fi.ValueExp,pekNumber,'1');
  FI:=R.Fields[1];
  AssertEquals('Name field 2','y',Fi.Name);
  AssertExpression('Field 2 value',Fi.ValueExp,pekNumber,'2');
end;

procedure TTestConstParser.TestArrayConst;

Var
  R : TArrayValues;
begin
  Typed := 'TMyArray';
  ParseConst('(1 , 2)');
  AssertEquals('Array Values',TArrayValues,TheExpr.ClassType);
  R:=TheExpr as TArrayValues;
  AssertEquals('Expression list of ',pekListOfExp,TheExpr.Kind);
  AssertEquals('2 elements',2,Length(R.Values));
  AssertExpression('Element 1 value',R.Values[0],pekNumber,'1');
  AssertExpression('Element 2 value',R.Values[1],pekNumber,'2');
end;

procedure TTestConstParser.TestRangeConst;
begin
  Typed:='0..1';
  ParseConst('1');
  AssertEquals('Range type',TPasRangeType,TheConst.VarType.ClassType);
  AssertExpression('Float const', TheExpr,pekNumber,'1');
end;

procedure TTestConstParser.DoParseConstUnTypedRange;

begin
  ParseConst('1..2');
end;

procedure TTestConstParser.TestRangeConstUnTyped;
begin
  AssertException('Range const is not allowed',EParserError,@DoParseConstUnTypedRange);
end;

procedure TTestConstParser.TestArrayOfRangeConst;
Var
  R : TArrayValues;
begin
  Typed:='array [0..7] of 0..1';
  ParseConst('(0, 0, 0, 0, 0, 0, 0, 0)');
  AssertEquals('Array Values',TArrayValues,TheExpr.ClassType);
  R:=TheExpr as TArrayValues;
  AssertEquals('Expression list of ',pekListOfExp,TheExpr.Kind);
  AssertEquals('elements',8,Length(R.Values));
//  AssertEquals('Range type',TPasRangeType,TheConst.VarType.ClassType);
//  AssertExpression('Float const', TheExpr,pekNumber,'1');
end;

{ TTestResourcestringParser }

function TTestResourcestringParser.ParseResourcestring(ASource: String
  ): TPasResString;

Var
  D : String;
begin
  Add('Resourcestring');
  D:=' A = '+ASource;
  If Hint<>'' then
    D:=D+' '+Hint;
  Add('  '+D+';');
  Add('end.');
  //Writeln(source.text);
  ParseDeclarations;
  AssertEquals('One resourcestring definition',1,Declarations.ResStrings.Count);
  AssertEquals('First declaration is constant definition.',TPasResString,TObject(Declarations.ResStrings[0]).ClassType);
  Result:=TPasResString(Declarations.ResStrings[0]);
  FTheStr:=Result;
  FExpr:=Result.Expr;
  Definition:=Result;
end;

procedure TTestResourcestringParser.CheckExprNameKindClass(AKind: TPasExprKind;
  AClass: TClass);
begin
  AssertEquals('Correct name','A',TheStr.Name);
  AssertEquals('Correct expression kind',aKind,TheExpr.Kind);
  AssertEquals('Correct expression class',AClass,TheExpr.ClassType);
  // Writeln('Delcaration : ',TheStr.GetDeclaration(True));
end;

procedure TTestResourcestringParser.DoTestSimple;
begin
  ParseResourcestring('''Something''');
  CheckExprNameKindClass(pekString,TPrimitiveExpr);
  AssertEquals('Correct expression value','''Something''',TPrimitiveExpr(TheExpr).Value);
end;

procedure TTestResourcestringParser.DoTestSum;
var
  B: TBinaryExpr;
begin
  ParseResourcestring('''Something''+'' else''');
  CheckExprNameKindClass(pekBinary,TBinaryExpr);
  B:=TBinaryExpr(TheExpr);
  TAssert.AssertSame('B.left.parent=B',B,B.left.Parent);
  TAssert.AssertSame('B.right.parent=B',B,B.right.Parent);
  AssertEquals('Correct left',TPrimitiveExpr,B.Left.ClassType);
  AssertEquals('Correct right',TPrimitiveExpr,B.Right.ClassType);
  AssertEquals('Correct left expression value','''Something''',TPrimitiveExpr(B.Left).Value);
  AssertEquals('Correct right expression value',''' else''',TPrimitiveExpr(B.Right).Value);
end;

procedure TTestResourcestringParser.DoTestSum2;
var
  B: TBinaryExpr;
begin
  ParseResourcestring('''Something''+different');
  CheckExprNameKindClass(pekBinary,TBinaryExpr);
  B:=TBinaryExpr(TheExpr);
  TAssert.AssertSame('B.left.parent=B',B,B.left.Parent);
  TAssert.AssertSame('B.right.parent=B',B,B.right.Parent);
  AssertEquals('Correct left',TPrimitiveExpr,B.Left.ClassType);
  AssertEquals('Correct right',TPrimitiveExpr,B.Right.ClassType);
  AssertEquals('Correct left expression value','''Something''',TPrimitiveExpr(B.Left).Value);
  AssertEquals('Correct right expression value','different',TPrimitiveExpr(B.Right).Value);
end;

procedure TTestResourcestringParser.TestSimple;
begin
  DoTestSimple;
end;

procedure TTestResourcestringParser.TestSimpleDeprecated;
begin
  Hint:='deprecated';
  DoTestSimple;
  CheckHint(hDeprecated);
end;

procedure TTestResourcestringParser.TestSimplePlatform;
begin
  Hint:='platform';
  DoTestSimple;
  CheckHint(hPlatform);
end;

procedure TTestResourcestringParser.TestSum2;
begin
  DoTestSum2;
end;

procedure TTestResourcestringParser.TestSum2Deprecated;
begin
  Hint:='deprecated';
  DoTestSum2;
  CheckHint(hDeprecated);
end;

procedure TTestResourcestringParser.TestSum2Platform;
begin
  Hint:='platform';
  DoTestSum2;
  CheckHint(hplatform);
end;
procedure TTestResourcestringParser.TestSum1;
begin
  DoTestSum;
end;

procedure TTestResourcestringParser.TestSum1Deprecated;
begin
  Hint:='deprecated';
  DoTestSum;
  CheckHint(hDeprecated);
end;

procedure TTestResourcestringParser.TestSum1Platform;
begin
  Hint:='platform';
  DoTestSum;
  CheckHint(hplatform);
end;

initialization
  RegisterTests([TTestConstParser,TTestResourcestringParser]);


end.

