{
  Examples:
    ./testpassrc --suite=TTestStatementParser.TestCallQualified2
}
unit tcPasWriteStatements;

{$mode objfpc}{$H+}

interface

uses
    Classes, SysUtils, fpcunit, PasTree, PScanner, PParser, PasWrite,
    tcbaseparser, testregistry;

type
    { TTestStatementWriterBase }

    TTestStatementWriterBase = class(TTestParser)
    private
        FPasWriter: TPasWriter;
        FStatement: TPasImplBlock;
        FTestStream: TMemoryStream;
        FVariables: TStrings;
        procedure TestCallFormat(FN: string; AddPrecision: boolean;
            AddSecondParam: boolean = False);
    protected
        procedure SetUp; override;
        procedure TearDown; override;
        procedure AddStatements(ASource: array of string);
        function BuildString(ASource: array of string): string;
        procedure DeclareVar(const AVarType: string; const AVarName: string = 'A');
        function TestStatement(ASource: string): TPasImplElement;
        function TestStatement(ASource: array of string): TPasImplElement;
        procedure ExpectParserError(const Msg: string);
        procedure ExpectParserError(const Msg: string; ASource: array of string);
        function AssertStatement(Msg: string;
            AClass: TClass; AIndex: integer = 0): TPasImplBlock;
        procedure AssertPasWriteOutput(Msg, ExpResult: string; aProgram: TPasElement);
        property Statement: TPasImplBlock read FStatement;
    published
    end;

    { TTestStatementWriterEmpty }

    TTestStatementWriterEmpty = class(TTestStatementWriterBase)
    published
        procedure TestEmpty;
        procedure TestEmptyStatement;
        procedure TestEmptyStatements;
    end;

    { TTestStatementWriterBlock }

    TTestStatementWriterBlock = class(TTestStatementWriterBase)
    published
        procedure TestBlock;
        procedure TestBlockComment;
        procedure TestBlock2Comments;
    end;

    { TTestStatementWriterAssignment }

    TTestStatementWriterAssignment = class(TTestStatementWriterBase)
    published
        procedure TestAssignment;
        procedure TestAssignmentAdd;
        procedure TestAssignmentMinus;
        procedure TestAssignmentMul;
        procedure TestAssignmentDivision;
        //   Procedure TestAssignmentMissingSemicolonError;

    end;

    { TTestStatementWriterCall }

    TTestStatementWriterCall = class(TTestStatementWriterBase)
    published
        procedure TestCall;
        procedure TestCallComment;
        procedure TestCallQualified;
        procedure TestCallQualified2;
        procedure TestCallNoArgs;
        procedure TestCallOneArg;
        procedure TestCallWriteFormat1;
        procedure TestCallWriteFormat2;
        procedure TestCallWriteFormat3;
        procedure TestCallWriteFormat4;
        procedure TestCallWritelnFormat1;
        procedure TestCallWritelnFormat2;
        procedure TestCallStrFormat1;
        procedure TestCallStrFormat2;
    end;

    { TTestStatementWriterIf }

    TTestStatementWriterIf = class(TTestStatementWriterBase)
    published
        procedure TestIf;
        procedure TestIfBlock;
        procedure TestIfAssignment;
        procedure TestIfElse;
        procedure TestIfElseBlock;
        procedure TestIfElseInBlock;
        procedure TestIfforElseBlock;
        procedure TestIfRaiseElseBlock;
        procedure TestIfWithBlock;
        procedure TestNestedIf;
        procedure TestNestedIfElse;
        procedure TestNestedIfElseElse;
        procedure TestIfIfElseElseBlock;
    end;

    { TTestStatementWriterLoops }

    TTestStatementWriterLoops = class(TTestStatementWriterBase)
    published
        procedure TestWhile;
        procedure TestWhileBlock;
        procedure TestWhileNested;
        procedure TestRepeat;
        procedure TestRepeatBlock;
        procedure TestRepeatBlockNosemicolon;
        procedure TestRepeatNested;
        procedure TestFor;
        procedure TestForIn;
        procedure TestForExpr;
        procedure TestForBlock;
        procedure TestDowntoBlock;
        procedure TestForNested;
    end;

    { TTestStatementWriterWith }

    TTestStatementWriterWith = class(TTestStatementWriterBase)
    published
        procedure TestWith;
        procedure TestWithMultiple;
    end;

    { TTestStatementWriterCase }

    TTestStatementWriterCase = class(TTestStatementWriterBase)
    published
        //Procedure TestCaseEmpty;
        procedure TestCaseOneInteger;
        procedure TestCaseTwoIntegers;
        procedure TestCaseRange;
        procedure TestCaseRangeSeparate;
        procedure TestCase2Cases;
        procedure TestCaseBlock;
        procedure TestCaseElseBlockEmpty;
        procedure TestCaseOtherwiseBlockEmpty;
        procedure TestCaseElseBlockAssignment;
        procedure TestCaseElseBlock2Assignments;
        procedure TestCaseIfCaseElse;
        procedure TestCaseIfCaseElseElse;
        procedure TestCaseIfElse;
        procedure TestCaseElseNoSemicolon;
        procedure TestCaseIfElseNoSemicolon;
        procedure TestCaseIfOtherwiseNoSemicolon;
    end;

    { TTestStatementWriterRaise }

    TTestStatementWriterRaise = class(TTestStatementWriterBase)
    published
        procedure TestRaise;
        procedure TestRaiseEmpty;
        procedure TestRaiseAt;
    end;

    { TTestStatementWriterTry }

    TTestStatementWriterTry = class(TTestStatementWriterBase)
    published
        procedure TestTryFinally;
        procedure TestTryFinallyEmpty;
        procedure TestTryFinallyNested;
        procedure TestTryExcept;
        procedure TestTryExceptNested;
        procedure TestTryExceptEmpty;
        procedure TestTryExceptOn;
        procedure TestTryExceptOn2;
        procedure TestTryExceptOnElse;
        procedure TestTryExceptOnIfElse;
        procedure TestTryExceptOnElseNoSemicolo;
        procedure TestTryExceptRaise;
    end;

    { TTestStatementWriterAsm }

    TTestStatementWriterAsm = class(TTestStatementWriterBase)
    published
        procedure TestAsm;
        procedure TestAsmBlock;
        procedure TestAsmBlockWithEndLabel;
        procedure TestAsmBlockInIfThen;
    end;

    { TTestStatementWriterSpecials }

    TTestStatementWriterSpecials = class(TTestStatementWriterBase)
    published
        procedure TestGotoInIfThen;
        procedure TestAssignToAddress;
        procedure TestFinalizationNoSemicolon;
        procedure TestMacroComment;
        procedure TestPlatformIdentifier;
        procedure TestPlatformIdentifier2;
        procedure TestArgumentNameOn;
    end;


implementation

{ TTestStatementWriterBase }

procedure TTestStatementWriterBase.SetUp;
begin
    inherited SetUp;
    FVariables := TStringList.Create;
    FTestStream := TMemoryStream.Create;
    FPasWriter := TPasWriter.Create(FTestStream);
end;

procedure TTestStatementWriterBase.TearDown;
begin
    FreeAndNil(FPasWriter);
    FreeAndNil(FTestStream);
    FreeAndNil(FVariables);
    inherited TearDown;
end;

procedure TTestStatementWriterBase.AddStatements(ASource: array of string);

var
    I: integer;
begin
    StartProgram(ExtractFileUnitName(MainFilename));
    if FVariables.Count > 0 then
      begin
        Add('Var');
        for I := 0 to FVariables.Count - 1 do
            Add('  ' + Fvariables[I]);
      end;
    Add('begin');
    for I := Low(ASource) to High(ASource) do
        Add('  ' + ASource[i]);
end;

function TTestStatementWriterBase.BuildString(ASource: array of string): string;
begin
    Result := string.Join(LineEnding, ASource);
end;

procedure TTestStatementWriterBase.DeclareVar(const AVarType: string;
    const AVarName: string);
begin
    FVariables.Add(AVarName + ' : ' + AVarType + ';');
end;

function TTestStatementWriterBase.TestStatement(ASource: string): TPasImplElement;
begin
    Result := TestStatement([ASource]);
end;

function TTestStatementWriterBase.TestStatement(ASource: array of string):
TPasImplElement;

begin
    Result := nil;
    FStatement := nil;
    AddStatements(ASource);
    ParseModule;
    AssertEquals('Have program', TPasProgram, Module.ClassType);
    AssertNotNull('Have program section', PasProgram.ProgramSection);
    AssertNotNull('Have initialization section', PasProgram.InitializationSection);
    if (PasProgram.InitializationSection.Elements.Count > 0) then
        if TObject(PasProgram.InitializationSection.Elements[0]) is TPasImplBlock then
            FStatement := TPasImplBlock(PasProgram.InitializationSection.Elements[0]);
    Result := FStatement;
end;

procedure TTestStatementWriterBase.ExpectParserError(const Msg: string);
begin
    AssertException(Msg, EParserError, @ParseModule);
end;

procedure TTestStatementWriterBase.ExpectParserError(const Msg: string;
    ASource: array of string);
begin
    AddStatements(ASource);
    ExpectParserError(Msg);
end;

function TTestStatementWriterBase.AssertStatement(Msg: string;
    AClass: TClass; AIndex: integer): TPasImplBlock;
begin
    if not (AIndex < PasProgram.InitializationSection.Elements.Count) then
        Fail(Msg + ': No such statement : ' + IntToStr(AIndex));
    AssertNotNull(Msg + ' Have statement', PasProgram.InitializationSection.Elements[AIndex]);
    AssertEquals(Msg + ' statement class', AClass, TObject(
        PasProgram.InitializationSection.Elements[AIndex]).ClassType);
    Result := TObject(PasProgram.InitializationSection.Elements[AIndex]) as TPasImplBlock;
end;

procedure TTestStatementWriterBase.AssertPasWriteOutput(Msg, ExpResult: string;
    aProgram: TPasElement);
var
    aString: string;
begin
    FPasWriter.WriteElement(aProgram);
    FTestStream.Seek(0, soBeginning);
    setlength(aString, FTestStream.Size);
    FTestStream.ReadBuffer(aString[1], FTestStream.Size);
    AssertEquals(Testname + ': ' + Msg, ExpResult, aString);
    AssertEquals(Testname + ': Streamsize', length(expResult), FTestStream.Size);
end;

// Tests -----------------------------------------------------------------

procedure TTestStatementWriterEmpty.TestEmpty;
begin
    //TestStatement(';');
    TestStatement('');
    AssertEquals('No statements', 0, PasProgram.InitializationSection.Elements.Count);

    AssertPasWriteOutput('output', 'program afile;'#13#10#13#10#13#10'begin'#13#10'end.'#13#10, PasProgram);
end;

procedure TTestStatementWriterEmpty.TestEmptyStatement;
begin
    TestStatement(';');
    AssertEquals('0 statement', 0, PasProgram.InitializationSection.Elements.Count);
    AssertPasWriteOutput('output', 'program afile;'#13#10#13#10#13#10'begin'#13#10'end.'#13#10, PasProgram);
end;

procedure TTestStatementWriterEmpty.TestEmptyStatements;
begin
    TestStatement(';;');
    AssertEquals('0 statement', 0, PasProgram.InitializationSection.Elements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterBlock.TestBlock;

var
    B: TPasImplBeginBlock;
begin
    TestStatement(['begin', 'end']);
    AssertEquals('1 statement', 1, PasProgram.InitializationSection.Elements.Count);
    AssertNotNull('Statement assigned', PasProgram.InitializationSection.Elements[0]);
    AssertEquals('Block statement', TPasImplBeginBlock, Statement.ClassType);
    B := Statement as TPasImplBeginBlock;
    AssertEquals('Empty block', 0, B.Elements.Count);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', 'begin', 'end;', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterBlock.TestBlockComment;
var
    B: TPasImplBeginBlock;
begin
    Engine.NeedComments := True;
    TestStatement(['{ This is a comment }', 'begin', 'end']);
    AssertEquals('1 statement', 1, PasProgram.InitializationSection.Elements.Count);
    AssertNotNull('Statement assigned', PasProgram.InitializationSection.Elements[0]);
    AssertEquals('Block statement', TPasImplBeginBlock, Statement.ClassType);
    B := Statement as TPasImplBeginBlock;
    AssertEquals('Empty block', 0, B.Elements.Count);
    AssertEquals('No DocComment', '', B.DocComment);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', 'begin', 'end;', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterBlock.TestBlock2Comments;
var
    B: TPasImplBeginBlock;
begin
    Engine.NeedComments := True;
    TestStatement(['{ This is a comment }', '// Another comment', 'begin', 'end']);
    AssertEquals('1 statement', 1, PasProgram.InitializationSection.Elements.Count);
    AssertNotNull('Statement assigned', PasProgram.InitializationSection.Elements[0]);
    AssertEquals('Block statement', TPasImplBeginBlock, Statement.ClassType);
    B := Statement as TPasImplBeginBlock;
    AssertEquals('Empty block', 0, B.Elements.Count);
    AssertEquals('No DocComment', '', B.DocComment);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', 'begin', 'end;', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterAssignment.TestAssignment;

var
    A: TPasImplAssign;
begin
    DeclareVar('integer');
    TestStatement(['a:=1;']);
    AssertEquals('1 statement', 1, PasProgram.InitializationSection.Elements.Count);
    AssertEquals('Assignment statement', TPasImplAssign, Statement.ClassType);
    A := Statement as TPasImplAssign;
    AssertEquals('Normal assignment', akDefault, A.Kind);
    AssertExpression('Right side is constant', A.Right, pekNumber, '1');
    AssertExpression('Left side is variable', A.Left, pekIdent, 'a');

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '', 'begin',
        '  a := 1;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterAssignment.TestAssignmentAdd;

var
    A: TPasImplAssign;
begin
    Parser.Scanner.Options := [po_cassignments];
    DeclareVar('integer');
    TestStatement(['a+=1;']);
    AssertEquals('1 statement', 1, PasProgram.InitializationSection.Elements.Count);
    AssertEquals('Assignment statement', TPasImplAssign, Statement.ClassType);
    A := Statement as TPasImplAssign;
    AssertEquals('Add assignment', akAdd, A.Kind);
    AssertExpression('Right side is constant', A.Right, pekNumber, '1');
    AssertExpression('Left side is variable', A.Left, pekIdent, 'a');

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '', 'begin',
        '  a += 1;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterAssignment.TestAssignmentMinus;
var
    A: TPasImplAssign;
begin
    Parser.Scanner.Options := [po_cassignments];
    DeclareVar('integer');
    TestStatement(['a-=1;']);
    AssertEquals('1 statement', 1, PasProgram.InitializationSection.Elements.Count);
    AssertEquals('Assignment statement', TPasImplAssign, Statement.ClassType);
    A := Statement as TPasImplAssign;
    AssertEquals('Minus assignment', akMinus, A.Kind);
    AssertExpression('Right side is constant', A.Right, pekNumber, '1');
    AssertExpression('Left side is variable', A.Left, pekIdent, 'a');

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '', 'begin',
        '  a -= 1;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterAssignment.TestAssignmentMul;
var
    A: TPasImplAssign;
begin
    Parser.Scanner.Options := [po_cassignments];
    DeclareVar('integer');
    TestStatement(['a*=1;']);
    AssertEquals('1 statement', 1, PasProgram.InitializationSection.Elements.Count);
    AssertEquals('Assignment statement', TPasImplAssign, Statement.ClassType);
    A := Statement as TPasImplAssign;
    AssertEquals('Mul assignment', akMul, A.Kind);
    AssertExpression('Right side is constant', A.Right, pekNumber, '1');
    AssertExpression('Left side is variable', A.Left, pekIdent, 'a');

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '', 'begin',
        '  a *= 1;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterAssignment.TestAssignmentDivision;
var
    A: TPasImplAssign;
begin
    Parser.Scanner.Options := [po_cassignments];
    DeclareVar('integer');
    TestStatement(['a/=1;']);
    AssertEquals('1 statement', 1, PasProgram.InitializationSection.Elements.Count);
    AssertEquals('Assignment statement', TPasImplAssign, Statement.ClassType);
    A := Statement as TPasImplAssign;
    AssertEquals('Division assignment', akDivision, A.Kind);
    AssertExpression('Right side is constant', A.Right, pekNumber, '1');
    AssertExpression('Left side is variable', A.Left, pekIdent, 'a');

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '', 'begin',
        '  a /= 1;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCall.TestCall;

var
    S: TPasImplSimple;
begin
    TestStatement('Doit;');
    AssertEquals('1 statement', 1, PasProgram.InitializationSection.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, Statement.ClassType);
    S := Statement as TPasImplSimple;
    AssertExpression('Doit call', S.Expr, pekIdent, 'Doit');

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  Doit;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCall.TestCallComment;

var
    S: TPasImplSimple;
begin
    Engine.NeedComments := True;
    TestStatement(['//comment line', 'Doit;']);
    AssertEquals('1 statement', 1, PasProgram.InitializationSection.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, Statement.ClassType);
    AssertEquals('1 statement', 1, PasProgram.InitializationSection.Elements.Count);
    S := Statement as TPasImplSimple;
    AssertExpression('Doit call', S.Expr, pekIdent, 'Doit');
    AssertEquals('No DocComment', '', S.DocComment);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  Doit;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCall.TestCallQualified;

var
    S: TPasImplSimple;
    B: TBinaryExpr;
begin
    TestStatement('Unita.Doit;');
    AssertEquals('1 statement', 1, PasProgram.InitializationSection.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, Statement.ClassType);
    S := Statement as TPasImplSimple;
    AssertExpression('Doit call', S.Expr, pekBinary, TBinaryExpr);
    B := S.Expr as TBinaryExpr;
    TAssert.AssertSame('B.left.Parent=B', B, B.left.Parent);
    TAssert.AssertSame('B.right.Parent=B', B, B.right.Parent);
    AssertExpression('Unit name', B.Left, pekIdent, 'Unita');
    AssertExpression('Doit call', B.Right, pekIdent, 'Doit');

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  Unita.Doit;', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterCall.TestCallQualified2;
var
    S: TPasImplSimple;
    B: TBinaryExpr;
begin
    TestStatement('Unita.ClassB.Doit;');
    AssertEquals('1 statement', 1, PasProgram.InitializationSection.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, Statement.ClassType);
    S := Statement as TPasImplSimple;
    AssertExpression('Doit call', S.Expr, pekBinary, TBinaryExpr);
    B := S.Expr as TBinaryExpr;
    AssertExpression('Doit call', B.Right, pekIdent, 'Doit');
    AssertExpression('First two parts of unit name', B.left, pekBinary, TBinaryExpr);
    B := B.left as TBinaryExpr;
    AssertExpression('Unit name part 1', B.Left, pekIdent, 'Unita');
    AssertExpression('Unit name part 2', B.right, pekIdent, 'ClassB');

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  Unita.ClassB.Doit;', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterCall.TestCallNoArgs;

var
    S: TPasImplSimple;
    P: TParamsExpr;
begin
    TestStatement('Doit();');
    AssertEquals('1 statement', 1, PasProgram.InitializationSection.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, Statement.ClassType);
    S := Statement as TPasImplSimple;
    AssertExpression('Doit call', S.Expr, pekFuncParams, TParamsExpr);
    P := S.Expr as TParamsExpr;
    AssertExpression('Correct function call name', P.Value, pekIdent, 'Doit');
    AssertEquals('No params', 0, Length(P.Params));

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  Doit();', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCall.TestCallOneArg;

var
    S: TPasImplSimple;
    P: TParamsExpr;
begin
    TestStatement('Doit(1);');
    AssertEquals('1 statement', 1, PasProgram.InitializationSection.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, Statement.ClassType);
    S := Statement as TPasImplSimple;
    AssertExpression('Doit call', S.Expr, pekFuncParams, TParamsExpr);
    P := S.Expr as TParamsExpr;
    AssertExpression('Correct function call name', P.Value, pekIdent, 'Doit');
    AssertEquals('One param', 1, Length(P.Params));
    AssertExpression('Parameter is constant', P.Params[0], pekNumber, '1');

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  Doit(1);', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterBase.TestCallFormat(FN: string;
    AddPrecision: boolean; AddSecondParam: boolean);
var
    P: TParamsExpr;

    procedure CheckParam(Index: integer; const aParamName: string);
    begin
        AssertExpression('Parameter[' + IntToStr(Index) + '] is identifier',
            P.Params[Index], pekIdent, aParamName);
        AssertExpression('Parameter[' + IntToStr(Index) + '] has formatting constant 1'
            , P.Params[Index].format1, pekNumber, '3');
        if AddPrecision then
            AssertExpression('Parameter[' + IntToStr(Index) + '] has formatting constant 2',
                P.Params[Index].format2, pekNumber, '2');
    end;

var
    S: TPasImplSimple;
    N: string;
    ArgCnt: integer;
begin
    N := fn + '(a:3';
    if AddPrecision then
        N := N + ':2';
    ArgCnt := 1;
    if AddSecondParam then
      begin
        ArgCnt := 2;
        N := N + ',b:3';
        if AddPrecision then
            N := N + ':2';
      end;
    N := N + ');';
    TestStatement(N);
    AssertEquals('1 statement', 1, PasProgram.InitializationSection.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, Statement.ClassType);
    S := Statement as TPasImplSimple;
    AssertExpression('Doit call', S.Expr, pekFuncParams, TParamsExpr);
    P := S.Expr as TParamsExpr;
    AssertExpression('Correct function call name', P.Value, pekIdent, FN);
    AssertEquals(IntToStr(ArgCnt) + ' param', ArgCnt, Length(P.Params));
    CheckParam(0, 'a');
    if AddSecondParam then
        CheckParam(1, 'b');
end;

procedure TTestStatementWriterCall.TestCallWriteFormat1;

begin
    TestCallFormat('write', False);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  write(a:3);', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterCall.TestCallWriteFormat2;

begin
    TestCallFormat('write', True);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  write(a:3:2);', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterCall.TestCallWriteFormat3;
begin
    TestCallFormat('write', False, True);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  write(a:3, b:3);', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterCall.TestCallWriteFormat4;
begin
    TestCallFormat('write', True, True);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  write(a:3:2, b:3:2);',
        'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCall.TestCallWritelnFormat1;
begin
    TestCallFormat('writeln', False);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  writeln(a:3);', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterCall.TestCallWritelnFormat2;
begin
    TestCallFormat('writeln', True);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  writeln(a:3:2);', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterCall.TestCallStrFormat1;
begin
    TestCallFormat('str', False);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  str(a:3);', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCall.TestCallStrFormat2;
begin
    TestCallFormat('str', True);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  str(a:3:2);', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterIf.TestIf;

var
    I: TPasImplIfElse;

begin
    DeclareVar('boolean');
    TestStatement(['if a then', ';']);
    I := AssertStatement('If statement', TPasImplIfElse) as TPasImplIfElse;
    AssertExpression('IF condition', I.ConditionExpr, pekIdent, 'a');
    AssertNull('No else', i.ElseBranch);
    AssertNull('No if branch', I.IfBranch);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '', 'begin',
        '  if a then;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterIf.TestIfBlock;

var
    I: TPasImplIfElse;

begin
    DeclareVar('boolean');
    TestStatement(['if a then', '  begin', '  end']);
    I := AssertStatement('If statement', TPasImplIfElse) as TPasImplIfElse;
    AssertExpression('IF condition', I.ConditionExpr, pekIdent, 'a');
    AssertNull('No else', i.ElseBranch);
    AssertNotNull('if branch', I.IfBranch);
    AssertEquals('begin end block', TPasImplBeginBlock, I.ifBranch.ClassType);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '', 'begin',
        '  if a then', '    begin', '    end;', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterIf.TestIfAssignment;

var
    I: TPasImplIfElse;

begin
    DeclareVar('boolean');
    TestStatement(['if a then', '  a:=False;']);
    I := AssertStatement('If statement', TPasImplIfElse) as TPasImplIfElse;
    AssertExpression('IF condition', I.ConditionExpr, pekIdent, 'a');
    AssertNull('No else', i.ElseBranch);
    AssertNotNull('if branch', I.IfBranch);
    AssertEquals('assignment statement', TPasImplAssign, I.ifBranch.ClassType);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '', 'begin',
        '  if a then', '    a := False;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterIf.TestIfElse;

var
    I: TPasImplIfElse;

begin
    DeclareVar('boolean');
    TestStatement(['if a then', '  begin', '  end', 'else', ';']);
    I := AssertStatement('If statement', TPasImplIfElse) as TPasImplIfElse;
    AssertExpression('IF condition', I.ConditionExpr, pekIdent, 'a');
    AssertNull('No else', i.ElseBranch);
    AssertNotNull('if branch', I.IfBranch);
    AssertEquals('begin end block', TPasImplBeginBlock, I.ifBranch.ClassType);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '', 'begin',
        '  if a then', '    begin', '    end;', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterIf.TestIfElseBlock;
var
    I: TPasImplIfElse;

begin
    DeclareVar('boolean');
    TestStatement(['if a then', '  begin', '  end', 'else', '  begin', '  end']);
    I := AssertStatement('If statement', TPasImplIfElse) as TPasImplIfElse;
    AssertExpression('IF condition', I.ConditionExpr, pekIdent, 'a');
    AssertNotNull('if branch', I.IfBranch);
    AssertEquals('begin end block', TPasImplBeginBlock, I.ifBranch.ClassType);
    AssertNotNull('Else branch', i.ElseBranch);
    AssertEquals('begin end block', TPasImplBeginBlock, I.ElseBranch.ClassType);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '', 'begin',
        '  if a then', '  begin', '  end else', '  begin',
        '  end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterIf.TestIfElseInBlock;
var
    B: TPasImplBeginBlock;
    I: TPasImplIfElse;

begin
    DeclareVar('boolean');
    TestStatement(['begin', '  if a then', '    DoA',
        '  else', 'end']);

    B := AssertStatement('begin block', TPasImplBeginBlock) as TPasImplBeginBlock;
    AssertEquals('One Element', 1, B.Elements.Count);
    AssertEquals('If statement', TPasImplIfElse, TObject(B.Elements[0]).ClassType);
    I := TPasImplIfElse(B.Elements[0]);
    AssertExpression('IF condition', I.ConditionExpr, pekIdent, 'a');
    AssertNotNull('if branch', I.IfBranch);
    AssertEquals('i_br: simple command', TPasImplSimple, I.ifBranch.ClassType);
    AssertExpression('Doit call', TPasImplSimple(I.ifBranch).Expr, pekIdent, 'DoA');
    AssertNull('Else branch', i.ElseBranch);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '', 'begin',
        'begin', '  if a then', '    DoA;', 'end;',
        'end.', '']), PasProgram);
end;

procedure TTestStatementWriterIf.TestIfforElseBlock;

var
    I: TPasImplIfElse;

begin
    TestStatement(['if a then', 'for X := 1 downto 0 do Writeln(X)', 'else',
        'for X := 0 to 1 do Writeln(X)']);
    I := AssertStatement('If statement', TPasImplIfElse) as TPasImplIfElse;
    AssertExpression('IF condition', I.ConditionExpr, pekIdent, 'a');
    AssertEquals('For statement', TPasImplForLoop, I.ifBranch.ClassType);
    AssertEquals('For statement', TPasImplForLoop, I.ElseBranch.ClassType);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  if a then', '  begin',
        '    for X:=1 downto 0 do', '      Writeln(X);', '  end else',
        '    for X:=0 to 1 do', '      Writeln(X);', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterIf.TestIfRaiseElseBlock;
// Error: to be searched for
var
    I: TPasImplIfElse;
begin
    TestStatement(['if a then', 'raise', 'else', 'for X := 0 to 1 do Writeln(X)']);
    I := AssertStatement('If statement', TPasImplIfElse) as TPasImplIfElse;
    AssertExpression('IF condition', I.ConditionExpr, pekIdent, 'a');
    AssertEquals('For statement', TPasImplRaise, I.ifBranch.ClassType);
    AssertEquals('For statement', TPasImplForLoop, I.ElseBranch.ClassType);

    AssertPasWriteOutput('output', BuildString(['program afile;', '',
        '', 'begin', '  if a then', '  begin', '    raise;', '  end else',
        '    for X:=0 to 1 do', '      Writeln(X);', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterIf.TestIfWithBlock;
// Error: With not implemented
var
    I: TPasImplIfElse;
begin
    TestStatement(['if a then', 'with b do something', 'else',
        'for X := 0 to 1 do Writeln(X)']);
    I := AssertStatement('If statement', TPasImplIfElse) as TPasImplIfElse;
    AssertExpression('IF condition', I.ConditionExpr, pekIdent, 'a');
    AssertEquals('For statement', TPasImplWithDo, I.ifBranch.ClassType);
    AssertEquals('For statement', TPasImplForLoop, I.ElseBranch.ClassType);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '', 'begin',
        '  if a then', '    with b do', '      something',
        '  else', '    for X:=0 to 1 do', '      Writeln(X);',
        'end.', '']), PasProgram);
end;

procedure TTestStatementWriterIf.TestNestedIf;
var
    I: TPasImplIfElse;
begin
    DeclareVar('boolean');
    DeclareVar('boolean', 'b');
    TestStatement(['if a then', '  if b then', '    begin', '    end',
        'else', '  begin', '  end']);
    I := AssertStatement('If statement', TPasImplIfElse) as TPasImplIfElse;
    AssertExpression('IF condition', I.ConditionExpr, pekIdent, 'a');
    AssertNotNull('if branch', I.IfBranch);
    AssertNull('Else branch', i.ElseBranch);
    AssertEquals('if in if branch', TPasImplIfElse, I.ifBranch.ClassType);
    I := I.Ifbranch as TPasImplIfElse;
    AssertEquals('begin end block', TPasImplBeginBlock, I.ElseBranch.ClassType);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '  b: Boolean;',
        '', 'begin', '  if a then', '    if b then',
        '    begin', '    end else', '    begin', '    end;',
        'end.', '']), PasProgram);
end;

procedure TTestStatementWriterIf.TestNestedIfElse;

var
    I: TPasImplIfElse;

begin
    DeclareVar('boolean');
    TestStatement(['if a then', '  if b then', '    begin', '    end',
        '  else', '    begin', '    end', 'else', '  begin', 'end']);
    I := AssertStatement('If statement', TPasImplIfElse) as TPasImplIfElse;
    AssertExpression('IF condition', I.ConditionExpr, pekIdent, 'a');
    AssertNotNull('if branch', I.IfBranch);
    AssertNotNull('Else branch', i.ElseBranch);
    AssertEquals('begin end block', TPasImplBeginBlock, I.ElseBranch.ClassType);
    AssertEquals('if in if branch', TPasImplIfElse, I.ifBranch.ClassType);
    I := I.Ifbranch as TPasImplIfElse;
    AssertEquals('begin end block', TPasImplBeginBlock, I.ElseBranch.ClassType);

    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '', 'begin',
        '  if a then', '  begin', '    if b then', '    begin',
        '    end else', '    begin', '    end;', '  end else',
        '  begin', '  end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterIf.TestNestedIfElseElse;

// Bug ID 37760

var
    I, I2: TPasImplIfElse;

begin
    DeclareVar('boolean');
    TestStatement(['if a then', '  if b then',
        '    DoA ', '   else', ' else',
        '   DoB']);
    I := AssertStatement('If statement', TPasImplIfElse) as TPasImplIfElse;
    AssertExpression('IF condition', I.ConditionExpr, pekIdent, 'a');
    AssertNotNull('if branch', I.IfBranch);
    AssertNotNull('Have else for outer if', I.ElseBranch);
    AssertEquals('Have if in if branch', TPasImplIfElse, I.ifBranch.ClassType);
    I2 := I.Ifbranch as TPasImplIfElse;
    AssertExpression('IF condition', I2.ConditionExpr, pekIdent, 'b');
    AssertNotNull('Have then for inner if', I2.ifBranch);
    AssertnotNull('Empty else for inner if', I2.ElseBranch);
    AssertEquals('Have a commend for inner if else', TPasImplCommand,
        I2.ElseBranch.ClassType);
    AssertEquals('... an empty command', '', TPasImplCommand(I2.ElseBranch).Command);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '', 'begin',
        '  if a then', '  begin', '    if b then', '    begin',
        '      DoA;', '    end else', '  end else', '    DoB;',
        'end.', '']), PasProgram);
end;

procedure TTestStatementWriterIf.TestIfIfElseElseBlock;

var
    OuterIf, InnerIf: TPasImplIfElse;
begin
    DeclareVar('boolean');
    DeclareVar('boolean', 'B');
    TestStatement(['if a then', 'if b then', '  begin', '  end', 'else',
        'else', '  begin', '  end']);
    OuterIf := AssertStatement('If statement', TPasImplIfElse) as TPasImplIfElse;
    AssertExpression('IF condition', OuterIf.ConditionExpr, pekIdent, 'a');
    AssertNotNull('if branch', OuterIf.IfBranch);
    AssertEquals('if else block', TPasImplIfElse, OuterIf.ifBranch.ClassType);
    InnerIf := OuterIf.IfBranch as TPasImplIfElse;
    AssertExpression('IF condition', InnerIf.ConditionExpr, pekIdent, 'b');
    AssertNotNull('if branch', InnerIf.IfBranch);
    AssertEquals('begin end block', TPasImplBeginBlock, InnerIf.ifBranch.ClassType);
    AssertNotNull('Else branch', InnerIf.ElseBranch);
    AssertEquals('empty statement', TPasImplCommand, InnerIf.ElseBranch.ClassType);
    AssertEquals('empty command', '', TPasImplCommand(InnerIf.ElseBranch).Command);
    AssertNotNull('Else branch', OuterIf.ElseBranch);
    AssertEquals('begin end block', TPasImplBeginBlock, OuterIf.ElseBranch.ClassType);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '  B: Boolean;',
        '', 'begin', '  if a then', '  begin',
        '    if b then', '    begin', '    end else',
        '  end else', '  begin', '  end;', 'end.', '']), PasProgram);
end;


procedure TTestStatementWriterLoops.TestWhile;

var
    W: TPasImplWhileDo;

begin
    DeclareVar('boolean');
    TestStatement(['While a do ;']);
    W := AssertStatement('While statement', TPasImplWhileDo) as TPasImplWhileDo;
    AssertExpression('While condition', W.ConditionExpr, pekIdent, 'a');
    AssertNull('Empty body', W.Body);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '', 'begin',
        '  While a do;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterLoops.TestWhileBlock;
var
    W: TPasImplWhileDo;

begin
    DeclareVar('boolean');
    TestStatement(['While a do', '  begin', '  end']);
    W := AssertStatement('While statement', TPasImplWhileDo) as TPasImplWhileDo;
    AssertExpression('While condition', W.ConditionExpr, pekIdent, 'a');
    AssertNotNull('Have while body', W.Body);
    AssertEquals('begin end block', TPasImplBeginBlock, W.Body.ClassType);
    AssertEquals('Empty block', 0, TPasImplBeginBlock(W.Body).ELements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '', 'begin',
        '  While a do', '  begin', '  end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterLoops.TestWhileNested;

var
    W: TPasImplWhileDo;

begin
    DeclareVar('boolean');
    DeclareVar('boolean', 'b');
    TestStatement(['While a do', '  while b do', '    begin', '    end']);
    W := AssertStatement('While statement', TPasImplWhileDo) as TPasImplWhileDo;
    AssertExpression('While condition', W.ConditionExpr, pekIdent, 'a');
    AssertNotNull('Have while body', W.Body);
    AssertEquals('Nested while', TPasImplWhileDo, W.Body.ClassType);
    W := W.Body as TPasImplWhileDo;
    AssertExpression('While condition', W.ConditionExpr, pekIdent, 'b');
    AssertNotNull('Have nested while body', W.Body);
    AssertEquals('Nested begin end block', TPasImplBeginBlock, W.Body.ClassType);
    AssertEquals('Empty nested block', 0, TPasImplBeginBlock(W.Body).ELements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '  b: Boolean;',
        '', 'begin', '  While a do', '    While b do',
        '    begin', '    end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterLoops.TestRepeat;

var
    R: TPasImplRepeatUntil;

begin
    DeclareVar('boolean');
    TestStatement(['Repeat', 'Until a;']);
    R := AssertStatement('Repeat statement', TPasImplRepeatUntil) as TPasImplRepeatUntil;
    AssertExpression('repeat condition', R.ConditionExpr, pekIdent, 'a');
    AssertEquals('Empty body', 0, R.Elements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '', 'begin',
        '  repeat', '  until a;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterLoops.TestRepeatBlock;

var
    R: TPasImplRepeatUntil;

begin
    DeclareVar('boolean');
    TestStatement(['Repeat', 'begin', 'end;', 'Until a;']);
    R := AssertStatement('repeat statement', TPasImplRepeatUntil) as TPasImplRepeatUntil;
    AssertExpression('repeat condition', R.ConditionExpr, pekIdent, 'a');
    AssertEquals('Have statement', 1, R.Elements.Count);
    AssertEquals('begin end block', TPasImplBeginBlock, TObject(R.Elements[0]).ClassType);
    AssertEquals('Empty block', 0, TPasImplBeginBlock(R.Elements[0]).ELements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '', 'begin',
        '  repeat', '  begin', '  end;', '  until a;',
        'end.', '']), PasProgram);
end;

procedure TTestStatementWriterLoops.TestRepeatBlockNosemicolon;

var
    R: TPasImplRepeatUntil;

begin
    DeclareVar('boolean');
    TestStatement(['Repeat', 'begin', 'end', 'Until a;']);
    R := AssertStatement('repeat statement', TPasImplRepeatUntil) as TPasImplRepeatUntil;
    AssertExpression('repeat condition', R.ConditionExpr, pekIdent, 'a');
    AssertEquals('Have statement', 1, R.Elements.Count);
    AssertEquals('begin end block', TPasImplBeginBlock, TObject(R.Elements[0]).ClassType);
    AssertEquals('Empty block', 0, TPasImplBeginBlock(R.Elements[0]).ELements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '', 'begin',
        '  repeat', '  begin', '  end;', '  until a;',
        'end.', '']), PasProgram);
end;

procedure TTestStatementWriterLoops.TestRepeatNested;

var
    R: TPasImplRepeatUntil;

begin
    DeclareVar('boolean');
    DeclareVar('boolean', 'b');
    TestStatement(['Repeat', 'repeat', 'begin', 'end', 'until b', 'Until a;']);
    R := AssertStatement('repeat statement', TPasImplRepeatUntil) as TPasImplRepeatUntil;
    AssertExpression('repeat condition', R.ConditionExpr, pekIdent, 'a');
    AssertEquals('Have statement', 1, R.Elements.Count);
    AssertEquals('Nested repeat', TPasImplRepeatUntil, TObject(R.Elements[0]).ClassType);
    R := TPasImplRepeatUntil(R.Elements[0]);
    AssertExpression('repeat condition', R.ConditionExpr, pekIdent, 'b');
    AssertEquals('Have statement', 1, R.Elements.Count);
    AssertEquals('begin end block', TPasImplBeginBlock, TObject(R.Elements[0]).ClassType);
    AssertEquals('Empty block', 0, TPasImplBeginBlock(R.Elements[0]).ELements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Boolean;', '  b: Boolean;',
        '', 'begin', '  repeat', '    repeat', '    begin',
        '    end;', '    until b;', '  until a;', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterLoops.TestFor;

var
    F: TPasImplForLoop;

begin
    DeclareVar('integer');
    TestStatement(['For a:=1 to 10 do', ';']);
    F := AssertStatement('For statement', TPasImplForLoop) as TPasImplForLoop;
    AssertExpression('Loop variable name', F.VariableName, pekIdent, 'a');
    AssertEquals('Loop type', ltNormal, F.Looptype);
    AssertEquals('Up loop', False, F.Down);
    AssertExpression('Start value', F.StartExpr, pekNumber, '1');
    AssertExpression('End value', F.EndExpr, pekNumber, '10');
    AssertNull('Empty body', F.Body);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '', 'begin',
        '  for a:=1 to 10 do;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterLoops.TestForIn;

var
    F: TPasImplForLoop;

begin
    DeclareVar('integer');
    TestStatement(['For a in SomeSet Do', ';']);
    F := AssertStatement('For statement', TPasImplForLoop) as TPasImplForLoop;
    AssertExpression('Loop variable name', F.VariableName, pekIdent, 'a');
    AssertEquals('Loop type', ltIn, F.Looptype);
    AssertEquals('In loop', False, F.Down);
    AssertExpression('Start value', F.StartExpr, pekIdent, 'SomeSet');
    AssertNull('Loop type', F.EndExpr);
    AssertNull('Empty body', F.Body);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '', 'begin',
        '  for a in SomeSet do;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterLoops.TestForExpr;
var
    F: TPasImplForLoop;
    B: TBinaryExpr;

begin
    DeclareVar('integer');
    TestStatement(['For a:=1+1 to 5+5 do', ';']);
    F := AssertStatement('For statement', TPasImplForLoop) as TPasImplForLoop;
    AssertExpression('Loop variable name', F.VariableName, pekIdent, 'a');
    AssertEquals('Up loop', False, F.Down);
    AssertExpression('Start expression', F.StartExpr, pekBinary, TBinaryExpr);
    B := F.StartExpr as TBinaryExpr;
    AssertExpression('Start value left', B.left, pekNumber, '1');
    AssertExpression('Start value right', B.right, pekNumber, '1');
    AssertExpression('Start expression', F.StartExpr, pekBinary, TBinaryExpr);
    B := F.EndExpr as TBinaryExpr;
    AssertExpression('End value left', B.left, pekNumber, '5');
    AssertExpression('End value right', B.right, pekNumber, '5');
    AssertNull('Empty body', F.Body);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '', 'begin',
        '  for a:=1 + 1 to 5 + 5 do;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterLoops.TestForBlock;

var
    F: TPasImplForLoop;

begin
    DeclareVar('integer');
    TestStatement(['For a:=1 to 10 do', 'begin', 'end']);
    F := AssertStatement('For statement', TPasImplForLoop) as TPasImplForLoop;
    AssertExpression('Loop variable name', F.VariableName, pekIdent, 'a');
    AssertEquals('Up loop', False, F.Down);
    AssertExpression('Start value', F.StartExpr, pekNumber, '1');
    AssertExpression('End value', F.EndExpr, pekNumber, '10');
    AssertNotNull('Have for body', F.Body);
    AssertEquals('begin end block', TPasImplBeginBlock, F.Body.ClassType);
    AssertEquals('Empty block', 0, TPasImplBeginBlock(F.Body).ELements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '', 'begin',
        '  for a:=1 to 10 do', '  begin', '  end;', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterLoops.TestDowntoBlock;

var
    F: TPasImplForLoop;

begin
    DeclareVar('integer');
    TestStatement(['For a:=10 downto 1 do', 'begin', 'end']);
    F := AssertStatement('For statement', TPasImplForLoop) as TPasImplForLoop;
    AssertExpression('Loop variable name', F.VariableName, pekIdent, 'a');
    AssertEquals('Down loop', True, F.Down);
    AssertExpression('Start value', F.StartExpr, pekNumber, '10');
    AssertExpression('End value', F.EndExpr, pekNumber, '1');
    AssertNotNull('Have for body', F.Body);
    AssertEquals('begin end block', TPasImplBeginBlock, F.Body.ClassType);
    AssertEquals('Empty block', 0, TPasImplBeginBlock(F.Body).ELements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '', 'begin',
        '  for a:=10 downto 1 do', '  begin', '  end;',
        'end.', '']), PasProgram);
end;

procedure TTestStatementWriterLoops.TestForNested;
var
    F: TPasImplForLoop;

begin
    DeclareVar('integer');
    DeclareVar('integer', 'b');
    TestStatement(['For a:=1 to 10 do', 'For b:=11 to 20 do', 'begin', 'end']);
    F := AssertStatement('For statement', TPasImplForLoop) as TPasImplForLoop;
    AssertExpression('Loop variable name', F.VariableName, pekIdent, 'a');
    AssertEquals('Up loop', False, F.Down);
    AssertExpression('Start value', F.StartExpr, pekNumber, '1');
    AssertExpression('End value', F.EndExpr, pekNumber, '10');
    AssertNotNull('Have while body', F.Body);
    AssertEquals('begin end block', TPasImplForLoop, F.Body.ClassType);
    F := F.Body as TPasImplForLoop;
    AssertExpression('Loop variable name', F.VariableName, pekIdent, 'b');
    AssertEquals('Up loop', False, F.Down);
    AssertExpression('Start value', F.StartExpr, pekNumber, '11');
    AssertExpression('End value', F.EndExpr, pekNumber, '20');
    AssertNotNull('Have for body', F.Body);
    AssertEquals('begin end block', TPasImplBeginBlock, F.Body.ClassType);
    AssertEquals('Empty block', 0, TPasImplBeginBlock(F.Body).ELements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '  b: Integer;',
        '', 'begin', '  for a:=1 to 10 do', '    for b:=11 to 20 do',
        '    begin', '    end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterWith.TestWith;
// not implemented yet
var
    W: TpasImplWithDo;

begin
    DeclareVar('record X,Y : Integer; end');
    TestStatement(['With a do', 'begin', 'end']);
    W := AssertStatement('For statement', TpasImplWithDo) as TpasImplWithDo;
    AssertEquals('1 expression', 1, W.Expressions.Count);
    AssertExpression('With identifier', TPasExpr(W.Expressions[0]), pekIdent, 'a');
    AssertNotNull('Have with body', W.Body);
    AssertEquals('begin end block', TPasImplBeginBlock, W.Body.ClassType);
    AssertEquals('Empty block', 0, TPasImplBeginBlock(W.Body).ELements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: record', '    X,Y: Integer;',
        '  end;', '', 'begin', '  with a do', '    begin',
        '    end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterWith.TestWithMultiple;
// not implemented yet
var
    W: TpasImplWithDo;

begin
    DeclareVar('record X,Y : Integer; end');
    DeclareVar('record W,Z : Integer; end', 'b');
    TestStatement(['With a,b do', 'begin', 'end']);
    W := AssertStatement('For statement', TpasImplWithDo) as TpasImplWithDo;
    AssertEquals('2 expressions', 2, W.Expressions.Count);
    AssertExpression('With identifier 1', TPasExpr(W.Expressions[0]), pekIdent, 'a');
    AssertExpression('With identifier 2', TPasExpr(W.Expressions[1]), pekIdent, 'b');
    AssertNotNull('Have with body', W.Body);
    AssertEquals('begin end block', TPasImplBeginBlock, W.Body.ClassType);
    AssertEquals('Empty block', 0, TPasImplBeginBlock(W.Body).ELements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: record', '    X,Y: Integer;',
        '  end;', '  b: record', '    W,Z: Integer;',
        '  end;', '', 'begin', '  with a, b do',
        '    begin', '    end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCase.TestCaseOneInteger;

var
    C: TPasImplCaseOf;
    S: TPasImplCaseStatement;

begin
    DeclareVar('integer');
    TestStatement(['case a of', '1 : ;', 'end;']);
    C := AssertStatement('Case statement', TpasImplCaseOf) as TpasImplCaseOf;
    AssertNotNull('Have case expression', C.CaseExpr);
    AssertExpression('Case expression', C.CaseExpr, pekIdent, 'a');
    AssertNull('No else branch', C.ElseBranch);
    AssertEquals('One case label', 1, C.Elements.Count);
    AssertEquals('Correct case for case label', TPasImplCaseStatement,
        TPasElement(C.Elements[0]).ClassType);
    S := TPasImplCaseStatement(C.Elements[0]);
    AssertEquals('1 expression for case', 1, S.Expressions.Count);
    AssertExpression('With identifier 1', TPasExpr(S.Expressions[0]), pekNumber, '1');
    AssertEquals('Empty case label statement', 0, S.Elements.Count);
    AssertNull('Empty case label statement', S.Body);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '', 'begin', '  case a of',
        '    1: ;', '  end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCase.TestCaseTwoIntegers;

var
    C: TPasImplCaseOf;
    S: TPasImplCaseStatement;

begin
    DeclareVar('integer');
    TestStatement(['case a of', '1,2 : ;', 'end;']);
    C := AssertStatement('Case statement', TpasImplCaseOf) as TpasImplCaseOf;
    AssertNotNull('Have case expression', C.CaseExpr);
    AssertExpression('Case expression', C.CaseExpr, pekIdent, 'a');
    AssertNull('No else branch', C.ElseBranch);
    AssertEquals('One case label', 1, C.Elements.Count);
    AssertEquals('Correct case for case label', TPasImplCaseStatement,
        TPasElement(C.Elements[0]).ClassType);
    S := TPasImplCaseStatement(C.Elements[0]);
    AssertEquals('2 expressions for case', 2, S.Expressions.Count);
    AssertExpression('With identifier 1', TPasExpr(S.Expressions[0]), pekNumber, '1');
    AssertExpression('With identifier 2', TPasExpr(S.Expressions[1]), pekNumber, '2');
    AssertEquals('Empty case label statement', 0, S.Elements.Count);
    AssertNull('Empty case label statement', S.Body);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '', 'begin', '  case a of',
        '    1, 2: ;', '  end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCase.TestCaseRange;
var
    C: TPasImplCaseOf;
    S: TPasImplCaseStatement;

begin
    DeclareVar('integer');
    TestStatement(['case a of', '1..3 : ;', 'end;']);
    C := AssertStatement('Case statement', TpasImplCaseOf) as TpasImplCaseOf;
    AssertNotNull('Have case expression', C.CaseExpr);
    AssertExpression('Case expression', C.CaseExpr, pekIdent, 'a');
    AssertNull('No else branch', C.ElseBranch);
    AssertEquals('One case label', 1, C.Elements.Count);
    AssertEquals('Correct case for case label', TPasImplCaseStatement,
        TPasElement(C.Elements[0]).ClassType);
    S := TPasImplCaseStatement(C.Elements[0]);
    AssertEquals('1 expression for case', 1, S.Expressions.Count);
    AssertExpression('With identifier 1', TPasExpr(S.Expressions[0]), pekRange, TBinaryExpr);
    AssertEquals('Empty case label statement', 0, S.Elements.Count);
    AssertNull('Empty case label statement', S.Body);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '', 'begin', '  case a of',
        '    1..3: ;', '  end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCase.TestCaseRangeSeparate;
var
    C: TPasImplCaseOf;
    S: TPasImplCaseStatement;

begin
    DeclareVar('integer');
    TestStatement(['case a of', '1..3,5 : ;', 'end;']);
    C := AssertStatement('Case statement', TpasImplCaseOf) as TpasImplCaseOf;
    AssertNotNull('Have case expression', C.CaseExpr);
    AssertExpression('Case expression', C.CaseExpr, pekIdent, 'a');
    AssertNull('No else branch', C.ElseBranch);
    AssertEquals('One case label', 1, C.Elements.Count);
    AssertEquals('Correct case for case label', TPasImplCaseStatement,
        TPasElement(C.Elements[0]).ClassType);
    S := TPasImplCaseStatement(C.Elements[0]);
    AssertEquals('2 expressions for case', 2, S.Expressions.Count);
    AssertExpression('With identifier 1', TPasExpr(S.Expressions[0]), pekRange, TBinaryExpr);
    AssertExpression('With identifier 2', TPasExpr(S.Expressions[1]), pekNumber, '5');
    AssertEquals('Empty case label statement', 0, S.Elements.Count);
    AssertNull('Empty case label statement', S.Body);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '', 'begin', '  case a of',
        '    1..3, 5: ;', '  end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCase.TestCase2Cases;
var
    C: TPasImplCaseOf;
    S: TPasImplCaseStatement;

begin
    DeclareVar('integer');
    TestStatement(['case a of', '1 : ;', '2 : ;', 'end;']);
    C := AssertStatement('Case statement', TpasImplCaseOf) as TpasImplCaseOf;
    AssertNotNull('Have case expression', C.CaseExpr);
    AssertExpression('Case expression', C.CaseExpr, pekIdent, 'a');
    AssertNull('No else branch', C.ElseBranch);
    AssertEquals('Two case labels', 2, C.Elements.Count);
    AssertEquals('Correct case for case label 1', TPasImplCaseStatement,
        TPasElement(C.Elements[0]).ClassType);
    S := TPasImplCaseStatement(C.Elements[0]);
    AssertEquals('2 expressions for case 1', 1, S.Expressions.Count);
    AssertExpression('Case 1 With identifier 1', TPasExpr(S.Expressions[0]), pekNumber, '1');
    AssertEquals('Empty case label statement 1', 0, S.Elements.Count);
    AssertNull('Empty case label statement 1', S.Body);
    // Two
    AssertEquals('Correct case for case label 2', TPasImplCaseStatement,
        TPasElement(C.Elements[1]).ClassType);
    S := TPasImplCaseStatement(C.Elements[1]);
    AssertEquals('2 expressions for case 2', 1, S.Expressions.Count);
    AssertExpression('Case 2 With identifier 1', TPasExpr(S.Expressions[0]), pekNumber, '2');
    AssertEquals('Empty case label statement 2', 0, S.Elements.Count);
    AssertNull('Empty case label statement 2', S.Body);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '', 'begin', '  case a of',
        '    1: ;', '    2: ;', '  end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCase.TestCaseBlock;

var
    C: TPasImplCaseOf;
    S: TPasImplCaseStatement;
    B: TPasImplbeginBlock;

begin
    DeclareVar('integer');
    TestStatement(['case a of', '1 : begin end;', 'end;']);
    C := AssertStatement('Case statement', TpasImplCaseOf) as TpasImplCaseOf;
    AssertNotNull('Have case expression', C.CaseExpr);
    AssertExpression('Case expression', C.CaseExpr, pekIdent, 'a');
    AssertNull('No else branch', C.ElseBranch);
    AssertEquals('Two case labels', 1, C.Elements.Count);
    AssertEquals('Correct case for case label 1', TPasImplCaseStatement,
        TPasElement(C.Elements[0]).ClassType);
    S := TPasImplCaseStatement(C.Elements[0]);
    AssertEquals('2 expressions for case 1', 1, S.Expressions.Count);
    AssertExpression('Case With identifier 1', TPasExpr(S.Expressions[0]), pekNumber, '1');
    AssertEquals('1 case label statement', 1, S.Elements.Count);
    AssertEquals('Correct case for case label 1', TPasImplbeginBlock,
        TPasElement(S.Elements[0]).ClassType);
    B := TPasImplbeginBlock(S.Elements[0]);
    AssertEquals('0 statements in block', 0, B.Elements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  A: Integer;', '', 'begin', '  case a of',
        '    1: begin', '      end;', '  end;', 'end.', '']), PasProgram);

end;

procedure TTestStatementWriterCase.TestCaseElseBlockEmpty;

var
    C: TPasImplCaseOf;
    S: TPasImplCaseStatement;
    B: TPasImplbeginBlock;

begin
    DeclareVar('integer');
    TestStatement(['case a of', '1 : begin end;', 'else', ' end;']);
    C := AssertStatement('Case statement', TpasImplCaseOf) as TpasImplCaseOf;
    AssertNotNull('Have case expression', C.CaseExpr);
    AssertExpression('Case expression', C.CaseExpr, pekIdent, 'a');
    AssertEquals('Two case labels', 2, C.Elements.Count);
    AssertEquals('Correct case for case label 1', TPasImplCaseStatement,
        TPasElement(C.Elements[0]).ClassType);
    S := TPasImplCaseStatement(C.Elements[0]);
    AssertEquals('2 expressions for case 1', 1, S.Expressions.Count);
    AssertExpression('Case With identifier 1', TPasExpr(S.Expressions[0]), pekNumber, '1');
    AssertEquals('1 case label statement', 1, S.Elements.Count);
    AssertEquals('Correct case for case label 1', TPasImplbeginBlock,
        TPasElement(S.Elements[0]).ClassType);
    B := TPasImplbeginBlock(S.Elements[0]);
    AssertEquals('0 statements in block', 0, B.Elements.Count);
    AssertNotNull('Have else branch', C.ElseBranch);
    AssertEquals('Correct else branch class', TPasImplCaseElse, C.ElseBranch.ClassType);
    AssertEquals('Zero statements ', 0, TPasImplCaseElse(C.ElseBranch).Elements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
'',
'var',
'  A: Integer;',
'',
'begin',
'  case a of',
'    1: begin',
'      end',
'    else',
'  end;',
'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCase.TestCaseOtherwiseBlockEmpty;

var
    C: TPasImplCaseOf;
begin
    DeclareVar('integer');
    TestStatement(['case a of', '1 : begin end;', 'otherwise', ' end;']);
    C := AssertStatement('Case statement', TpasImplCaseOf) as TpasImplCaseOf;
    AssertNotNull('Have case expression', C.CaseExpr);
    AssertNotNull('Have else branch', C.ElseBranch);
    AssertEquals('Correct else branch class', TPasImplCaseElse, C.ElseBranch.ClassType);
    AssertEquals('Zero statements ', 0, TPasImplCaseElse(C.ElseBranch).Elements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
'',
'var',
'  A: Integer;',
'',
'begin',
'  case a of',
'    1: begin',
'      end',
'    else',
'  end;',
'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCase.TestCaseElseBlockAssignment;
var
    C: TPasImplCaseOf;
    S: TPasImplCaseStatement;
    B: TPasImplbeginBlock;

begin
    DeclareVar('integer');
    TestStatement(['case a of', '1 : begin end;', 'else', 'a:=1', ' end;']);
    C := AssertStatement('Case statement', TpasImplCaseOf) as TpasImplCaseOf;
    AssertNotNull('Have case expression', C.CaseExpr);
    AssertExpression('Case expression', C.CaseExpr, pekIdent, 'a');
    AssertEquals('Two case labels', 2, C.Elements.Count);
    AssertEquals('Correct case for case label 1', TPasImplCaseStatement,
        TPasElement(C.Elements[0]).ClassType);
    S := TPasImplCaseStatement(C.Elements[0]);
    AssertEquals('2 expressions for case 1', 1, S.Expressions.Count);
    AssertExpression('Case With identifier 1', TPasExpr(S.Expressions[0]), pekNumber, '1');
    AssertEquals('1 case label statement', 1, S.Elements.Count);
    AssertEquals('Correct case for case label 1', TPasImplbeginBlock,
        TPasElement(S.Elements[0]).ClassType);
    B := TPasImplbeginBlock(S.Elements[0]);
    AssertEquals('0 statements in block', 0, B.Elements.Count);
    AssertNotNull('Have else branch', C.ElseBranch);
    AssertEquals('Correct else branch class', TPasImplCaseElse, C.ElseBranch.ClassType);
    AssertEquals('1 statement in else branch ', 1, TPasImplCaseElse(
        C.ElseBranch).Elements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
'',
'var',
'  A: Integer;',
'',
'begin',
'  case a of',
'    1: begin',
'      end',
'    else',
'      a := 1;',
'  end;',
'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCase.TestCaseElseBlock2Assignments;

var
    C: TPasImplCaseOf;
    S: TPasImplCaseStatement;
    B: TPasImplbeginBlock;

begin
    DeclareVar('integer');
    TestStatement(['case a of', '1 : begin end;', 'else', 'a:=1;', 'a:=32;', ' end;']);
    C := AssertStatement('Case statement', TpasImplCaseOf) as TpasImplCaseOf;
    AssertNotNull('Have case expression', C.CaseExpr);
    AssertExpression('Case expression', C.CaseExpr, pekIdent, 'a');
    AssertEquals('Two case labels', 2, C.Elements.Count);
    AssertEquals('Correct case for case label 1', TPasImplCaseStatement,
        TPasElement(C.Elements[0]).ClassType);
    S := TPasImplCaseStatement(C.Elements[0]);
    AssertEquals('2 expressions for case 1', 1, S.Expressions.Count);
    AssertExpression('Case With identifier 1', TPasExpr(S.Expressions[0]), pekNumber, '1');
    AssertEquals('1 case label statement', 1, S.Elements.Count);
    AssertEquals('Correct case for case label 1', TPasImplbeginBlock,
        TPasElement(S.Elements[0]).ClassType);
    B := TPasImplbeginBlock(S.Elements[0]);
    AssertEquals('0 statements in block', 0, B.Elements.Count);
    AssertNotNull('Have else branch', C.ElseBranch);
    AssertEquals('Correct else branch class', TPasImplCaseElse, C.ElseBranch.ClassType);
    AssertEquals('2 statements in else branch ', 2, TPasImplCaseElse(
        C.ElseBranch).Elements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
'',
'var',
'  A: Integer;',
'',
'begin',
'  case a of',
'    1: begin',
'      end',
'    else',
'      a := 1;',
'      a := 32;',
'  end;',
'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCase.TestCaseIfCaseElse;

var
    C: TPasImplCaseOf;

begin
    DeclareVar('integer');
    DeclareVar('boolean', 'b');
    TestStatement(['case a of', '1 : if b then', ' begin end;', 'else', ' end;']);
    C := AssertStatement('Case statement', TpasImplCaseOf) as TpasImplCaseOf;
    AssertNotNull('Have case expression', C.CaseExpr);
    AssertExpression('Case expression', C.CaseExpr, pekIdent, 'a');
    AssertEquals('Two case labels', 2, C.Elements.Count);
    AssertNotNull('Have else branch', C.ElseBranch);
    AssertEquals('Correct else branch class', TPasImplCaseElse, C.ElseBranch.ClassType);
    AssertEquals('0 statement in else branch ', 0, TPasImplCaseElse(
        C.ElseBranch).Elements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
'',
'var',
'  A: Integer;',
'  b: Boolean;',
'',
'begin',
'  case a of',
'    1: begin',
'        if b then',
'      end',
'    else',
'  end;',
'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCase.TestCaseIfElse;
var
    C: TPasImplCaseOf;
    S: TPasImplCaseStatement;

begin
    DeclareVar('integer');
    DeclareVar('boolean', 'b');
    TestStatement(['case a of', '1 : if b then', ' begin end', 'else', 'begin', 'end', ' end;']);
    C := AssertStatement('Case statement', TpasImplCaseOf) as TpasImplCaseOf;
    AssertNotNull('Have case expression', C.CaseExpr);
    AssertExpression('Case expression', C.CaseExpr, pekIdent, 'a');
    AssertEquals('One case label', 1, C.Elements.Count);
    AssertNull('Have no else branch', C.ElseBranch);
    S := TPasImplCaseStatement(C.Elements[0]);
    AssertEquals('2 expressions for case 1', 1, S.Expressions.Count);
    AssertExpression('Case With identifier 1', TPasExpr(S.Expressions[0]), pekNumber, '1');
    AssertEquals('1 case label statement', 1, S.Elements.Count);
    AssertEquals('If statement in case label 1', TPasImplIfElse, TPasElement(
        S.Elements[0]).ClassType);
    AssertNotNull('If statement has else block', TPasImplIfElse(S.Elements[0]).ElseBranch);
    AssertPasWriteOutput('output', BuildString(['program afile;',
'',
'var',
'  A: Integer;',
'  b: Boolean;',
'',
'begin',
'  case a of',
'    1: begin',
'        if b then',
'        begin',
'        end else',
'        begin',
'        end;',
'      end;',
'  end;',
'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCase.TestCaseIfCaseElseElse;
var
    C: TPasImplCaseOf;
    S: TPasImplCaseStatement;

begin
    DeclareVar('integer');
    DeclareVar('boolean', 'b');
    TestStatement(['case a of', '1 : if b then', ' begin end', 'else',
        'else', 'DoElse', ' end;']);
    C := AssertStatement('Case statement', TpasImplCaseOf) as TpasImplCaseOf;
    AssertNotNull('Have case expression', C.CaseExpr);
    AssertExpression('Case expression', C.CaseExpr, pekIdent, 'a');
    AssertEquals('Two case labels', 2, C.Elements.Count);
    AssertNotNull('Have an else branch', C.ElseBranch);
    S := TPasImplCaseStatement(C.Elements[0]);
    AssertEquals('2 expressions for case 1', 1, S.Expressions.Count);
    AssertExpression('Case With identifier 1', TPasExpr(S.Expressions[0]), pekNumber, '1');
    AssertEquals('1 case label statement', 1, S.Elements.Count);
    AssertEquals('If statement in case label 1', TPasImplIfElse, TPasElement(
        S.Elements[0]).ClassType);
    AssertNotNull('If statement has else block', TPasImplIfElse(S.Elements[0]).ElseBranch);
    AssertEquals('If statement has a commend as else block', TPasImplCommand,
        TPasImplIfElse(S.Elements[0]).ElseBranch.ClassType);
    AssertEquals('But ... an empty command', '', TPasImplCommand(
        TPasImplIfElse(S.Elements[0]).ElseBranch).Command);
    AssertPasWriteOutput('output', BuildString(['program afile;',
'',
'var',
'  A: Integer;',
'  b: Boolean;',
'',
'begin',
'  case a of',
'    1: begin',
'        if b then',
'        begin',
'        end else',
'      end',
'    else',
'      DoElse;',
'  end;',
'end.','']), PasProgram);
end;

procedure TTestStatementWriterCase.TestCaseElseNoSemicolon;
var
    C: TPasImplCaseOf;
    S: TPasImplCaseStatement;
begin
    DeclareVar('integer');
    TestStatement(['case a of', '1 : dosomething;', '2 : dosomethingmore',
        'else', 'a:=1;', 'end;']);
    C := AssertStatement('Case statement', TpasImplCaseOf) as TpasImplCaseOf;
    AssertNotNull('Have case expression', C.CaseExpr);
    AssertExpression('Case expression', C.CaseExpr, pekIdent, 'a');
    AssertEquals('case label count', 3, C.Elements.Count);
    S := TPasImplCaseStatement(C.Elements[0]);
    AssertEquals('case 1', 1, S.Expressions.Count);
    AssertExpression('Case With identifier 1', TPasExpr(S.Expressions[0]), pekNumber, '1');
    S := TPasImplCaseStatement(C.Elements[1]);
    AssertEquals('case 2', 1, S.Expressions.Count);
    AssertExpression('Case With identifier 1', TPasExpr(S.Expressions[0]), pekNumber, '2');
    AssertEquals('third is else', TPasImplCaseElse, TObject(C.Elements[2]).ClassType);
    AssertNotNull('Have else branch', C.ElseBranch);
    AssertEquals('Correct else branch class', TPasImplCaseElse, C.ElseBranch.ClassType);
    AssertEquals('1 statements in else branch ', 1, TPasImplCaseElse(
        C.ElseBranch).Elements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
'',
'var',
'  A: Integer;',
'',
'begin',
'  case a of',
'    1: begin',
'        dosomething;',
'      end;',
'    2: begin',
'        dosomethingmore;',
'      end',
'    else',
'      a := 1;',
'  end;',
'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCase.TestCaseIfElseNoSemicolon;
var
    C: TPasImplCaseOf;
    S: TPasImplCaseStatement;
begin
    DeclareVar('integer');
    TestStatement(['case a of', '1 : dosomething;', '2: if b then',
        ' dosomething', 'else  dosomethingmore', 'else', 'a:=1;', 'end;']);
    C := AssertStatement('Case statement', TpasImplCaseOf) as TpasImplCaseOf;
    AssertNotNull('Have case expression', C.CaseExpr);
    AssertExpression('Case expression', C.CaseExpr, pekIdent, 'a');
    AssertEquals('case label count', 3, C.Elements.Count);
    S := TPasImplCaseStatement(C.Elements[0]);
    AssertEquals('case 1', 1, S.Expressions.Count);
    AssertExpression('Case With identifier 1', TPasExpr(S.Expressions[0]), pekNumber, '1');
    S := TPasImplCaseStatement(C.Elements[1]);
    AssertEquals('case 2', 1, S.Expressions.Count);
    AssertExpression('Case With identifier 1', TPasExpr(S.Expressions[0]), pekNumber, '2');
    AssertEquals('third is else', TPasImplCaseElse, TObject(C.Elements[2]).ClassType);
    AssertNotNull('Have else branch', C.ElseBranch);
    AssertEquals('Correct else branch class', TPasImplCaseElse, C.ElseBranch.ClassType);
    AssertEquals('1 statements in else branch ', 1, TPasImplCaseElse(
        C.ElseBranch).Elements.Count);

    AssertPasWriteOutput('output', BuildString(['program afile;',
'',
'var',
'  A: Integer;',
'',
'begin',
'  case a of',
'    1: begin',
'        dosomething;',
'      end;',
'    2: begin',
'        if b then',
'        begin',
'          dosomething;',
'        end else',
'          dosomethingmore;',
'      end',
'    else',
'      a := 1;',
'  end;',
'end.', '']), PasProgram);
end;

procedure TTestStatementWriterCase.TestCaseIfOtherwiseNoSemicolon;
var
    C: TPasImplCaseOf;
    S: TPasImplCaseStatement;
begin
    DeclareVar('integer');
    TestStatement(['case a of', '1 : dosomething;', '2: if b then',
        ' dosomething', 'else  dosomethingmore', 'otherwise', 'a:=1;', 'end;']);
    C := AssertStatement('Case statement', TpasImplCaseOf) as TpasImplCaseOf;
    AssertNotNull('Have case expression', C.CaseExpr);
    AssertExpression('Case expression', C.CaseExpr, pekIdent, 'a');
    AssertEquals('case label count', 3, C.Elements.Count);
    S := TPasImplCaseStatement(C.Elements[0]);
    AssertEquals('case 1', 1, S.Expressions.Count);
    AssertExpression('Case With identifier 1', TPasExpr(S.Expressions[0]), pekNumber, '1');
    S := TPasImplCaseStatement(C.Elements[1]);
    AssertEquals('case 2', 1, S.Expressions.Count);
    AssertExpression('Case With identifier 1', TPasExpr(S.Expressions[0]), pekNumber, '2');
    AssertEquals('third is else', TPasImplCaseElse, TObject(C.Elements[2]).ClassType);
    AssertNotNull('Have else branch', C.ElseBranch);
    AssertEquals('Correct else branch class', TPasImplCaseElse, C.ElseBranch.ClassType);
    AssertEquals('1 statements in else branch ', 1, TPasImplCaseElse(
        C.ElseBranch).Elements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
'',
'var',
'  A: Integer;',
'',
'begin',
'  case a of',
'    1: begin',
'        dosomething;',
'      end;',
'    2: begin',
'        if b then',
'        begin',
'          dosomething;',
'        end else',
'          dosomethingmore;',
'      end',
'    else',
'      a := 1;',
'  end;',
'end.', '']), PasProgram);
end;



procedure TTestStatementWriterRaise.TestRaise;

var
    R: TPasImplRaise;

begin
    DeclareVar('Exception');
    TestStatement('Raise A;');
    R := AssertStatement('Raise statement', TPasImplRaise) as TPasImplRaise;
    AssertEquals(0, R.Elements.Count);
    AssertNotNull(R.ExceptObject);
    AssertNull(R.ExceptAddr);
    AssertExpression('Expression object', R.ExceptObject, pekIdent, 'A');
    AssertPasWriteOutput('output', BuildString(['program afile;',
'',
'var',
'  A: Exception;',
'',
'begin',
'  raise A;',
'end.', '']), PasProgram);
end;

procedure TTestStatementWriterRaise.TestRaiseEmpty;
var
    R: TPasImplRaise;

begin
    TestStatement('Raise;');
    R := AssertStatement('Raise statement', TPasImplRaise) as TPasImplRaise;
    AssertEquals(0, R.Elements.Count);
    AssertNull(R.ExceptObject);
    AssertNull(R.ExceptAddr);
    AssertPasWriteOutput('output', BuildString(['program afile;',
'',
'',
'begin',
'  raise;',
'end.', '']), PasProgram);
end;

procedure TTestStatementWriterRaise.TestRaiseAt;

var
    R: TPasImplRaise;

begin
    DeclareVar('Exception');
    DeclareVar('Pointer', 'B');
    TestStatement('Raise A at B;');
    R := AssertStatement('Raise statement', TPasImplRaise) as TPasImplRaise;
    AssertEquals(0, R.Elements.Count);
    AssertNotNull(R.ExceptObject);
    AssertNotNull(R.ExceptAddr);
    AssertExpression('Expression object', R.ExceptAddr, pekIdent, 'B');
    AssertPasWriteOutput('output', BuildString(['program afile;',
'',
'var',
'  A: Exception;',
'  B: Pointer;',
'',
'begin',
'  raise A at B;',
'end.', '']), PasProgram);
end;

procedure TTestStatementWriterTry.TestTryFinally;

var
    T: TPasImplTry;
    S: TPasImplSimple;
    F: TPasImplTryFinally;

begin
    TestStatement(['Try', '  DoSomething;', 'finally', '  DoSomethingElse', 'end']);
    T := AssertStatement('Try statement', TPasImplTry) as TPasImplTry;
    AssertEquals(1, T.Elements.Count);
    AssertNotNull(T.FinallyExcept);
    AssertNull(T.ElseBranch);
    AssertNotNull(T.Elements[0]);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    S := TPasImplSimple(T.Elements[0]);
    AssertExpression('DoSomething call', S.Expr, pekIdent, 'DoSomething');
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    AssertEquals('Finally statement', TPasImplTryFinally, T.FinallyExcept.ClassType);
    F := TPasImplTryFinally(T.FinallyExcept);
    AssertEquals(1, F.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(F.Elements[0]).ClassType);
    S := TPasImplSimple(F.Elements[0]);
    AssertExpression('DoSomethingElse call', S.Expr, pekIdent, 'DoSomethingElse');
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  try', '    DoSomething;',
        '  finally', '    DoSomethingElse;', '  end;', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterTry.TestTryFinallyEmpty;
var
    T: TPasImplTry;
    F: TPasImplTryFinally;

begin
    TestStatement(['Try', 'finally', 'end;']);
    T := AssertStatement('Try statement', TPasImplTry) as TPasImplTry;
    AssertEquals(0, T.Elements.Count);
    AssertNotNull(T.FinallyExcept);
    AssertNull(T.ElseBranch);
    AssertEquals('Finally statement', TPasImplTryFinally, T.FinallyExcept.ClassType);
    F := TPasImplTryFinally(T.FinallyExcept);
    AssertEquals(0, F.Elements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  try', '  finally', '  end;',
        'end.', '']), PasProgram);
end;

procedure TTestStatementWriterTry.TestTryFinallyNested;
var
    T: TPasImplTry;
    S: TPasImplSimple;
    F: TPasImplTryFinally;

begin
    TestStatement(['Try', '  DoSomething1;', '  Try', '    DoSomething2;',
        '  finally', '    DoSomethingElse2', '  end;', 'Finally', '  DoSomethingElse1', 'end']);
    T := AssertStatement('Try statement', TPasImplTry) as TPasImplTry;
    AssertEquals(2, T.Elements.Count);
    AssertNotNull(T.FinallyExcept);
    AssertNull(T.ElseBranch);
    AssertNotNull(T.Elements[0]);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    S := TPasImplSimple(T.Elements[0]);
    AssertExpression('DoSomething call', S.Expr, pekIdent, 'DoSomething1');
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    AssertEquals('Finally statement', TPasImplTryFinally, T.FinallyExcept.ClassType);
    F := TPasImplTryFinally(T.FinallyExcept);
    AssertEquals(1, F.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(F.Elements[0]).ClassType);
    S := TPasImplSimple(F.Elements[0]);
    AssertExpression('DoSomethingElse call', S.Expr, pekIdent, 'DoSomethingElse1');
    // inner statement
    AssertNotNull(T.Elements[1]);
    AssertEquals('Nested try statement', TPasImplTry, TPasElement(T.Elements[1]).ClassType);
    T := TPasImplTry(T.Elements[1]);
    AssertEquals(1, T.Elements.Count);
    AssertNotNull(T.FinallyExcept);
    AssertNull(T.ElseBranch);
    AssertNotNull(T.Elements[0]);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    S := TPasImplSimple(T.Elements[0]);
    AssertExpression('DoSomething call', S.Expr, pekIdent, 'DoSomething2');
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    AssertEquals('Finally statement', TPasImplTryFinally, T.FinallyExcept.ClassType);
    F := TPasImplTryFinally(T.FinallyExcept);
    AssertEquals(1, F.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(F.Elements[0]).ClassType);
    S := TPasImplSimple(F.Elements[0]);
    AssertExpression('DoSomethingElse call', S.Expr, pekIdent, 'DoSomethingElse2');
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  try', '    DoSomething1;',
        '    try', '      DoSomething2;', '    finally',
        '      DoSomethingElse2;', '    end;', '  finally',
        '    DoSomethingElse1;', '  end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterTry.TestTryExcept;

var
    T: TPasImplTry;
    S: TPasImplSimple;
    E: TPasImplTryExcept;

begin
    TestStatement(['Try', '  DoSomething;', 'except', '  DoSomethingElse', 'end']);
    T := AssertStatement('Try statement', TPasImplTry) as TPasImplTry;
    AssertEquals(1, T.Elements.Count);
    AssertNotNull(T.FinallyExcept);
    AssertNull(T.ElseBranch);
    AssertNotNull(T.Elements[0]);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    S := TPasImplSimple(T.Elements[0]);
    AssertExpression('DoSomething call', S.Expr, pekIdent, 'DoSomething');
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    AssertEquals('Except statement', TPasImplTryExcept, T.FinallyExcept.ClassType);
    E := TPasImplTryExcept(T.FinallyExcept);
    AssertEquals(1, E.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(E.Elements[0]).ClassType);
    S := TPasImplSimple(E.Elements[0]);
    AssertExpression('DoSomethingElse call', S.Expr, pekIdent, 'DoSomethingElse');
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  try', '    DoSomething;',
        '  except', '    DoSomethingElse;', '  end;', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterTry.TestTryExceptNested;
var
    T: TPasImplTry;
    S: TPasImplSimple;
    E: TPasImplTryExcept;

begin
    TestStatement(['Try', '  DoSomething1;', '  try', '    DoSomething2;',
        '  except', '    DoSomethingElse2', '  end', 'except', '  DoSomethingElse1', 'end']);
    T := AssertStatement('Try statement', TPasImplTry) as TPasImplTry;
    AssertEquals(2, T.Elements.Count);
    AssertNotNull(T.FinallyExcept);
    AssertNull(T.ElseBranch);
    AssertNotNull(T.Elements[0]);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    S := TPasImplSimple(T.Elements[0]);
    AssertExpression('DoSomething call', S.Expr, pekIdent, 'DoSomething1');
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    AssertEquals('Except statement', TPasImplTryExcept, T.FinallyExcept.ClassType);
    E := TPasImplTryExcept(T.FinallyExcept);
    AssertEquals(1, E.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(E.Elements[0]).ClassType);
    S := TPasImplSimple(E.Elements[0]);
    AssertExpression('DoSomethingElse call', S.Expr, pekIdent, 'DoSomethingElse1');
    AssertNotNull(T.Elements[1]);
    AssertEquals('Simple statement', TPasImplTry, TPasElement(T.Elements[1]).ClassType);
    T := TPasImplTry(T.Elements[1]);
    AssertEquals(1, T.Elements.Count);
    AssertNotNull(T.FinallyExcept);
    AssertNull(T.ElseBranch);
    AssertNotNull(T.Elements[0]);
    AssertEquals('Simple statement 2', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    S := TPasImplSimple(T.Elements[0]);
    AssertExpression('DoSomething2 call ', S.Expr, pekIdent, 'DoSomething2');
    AssertEquals('Simple statement2', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    AssertEquals('Except statement2', TPasImplTryExcept, T.FinallyExcept.ClassType);
    E := TPasImplTryExcept(T.FinallyExcept);
    AssertEquals(1, E.Elements.Count);
    AssertEquals('Simple statement2', TPasImplSimple, TPasElement(E.Elements[0]).ClassType);
    S := TPasImplSimple(E.Elements[0]);
    AssertExpression('DoSomethingElse2 call', S.Expr, pekIdent, 'DoSomethingElse2');
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  try', '    DoSomething1;',
        '    try', '      DoSomething2;', '    except',
        '      DoSomethingElse2;', '    end;', '  except',
        '    DoSomethingElse1;', '  end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterTry.TestTryExceptEmpty;

var
    T: TPasImplTry;
    E: TPasImplTryExcept;

begin
    TestStatement(['Try', 'except', 'end;']);
    T := AssertStatement('Try statement', TPasImplTry) as TPasImplTry;
    AssertEquals(0, T.Elements.Count);
    AssertNotNull(T.FinallyExcept);
    AssertNull(T.ElseBranch);
    AssertEquals('Except statement', TPasImplTryExcept, T.FinallyExcept.ClassType);
    E := TPasImplTryExcept(T.FinallyExcept);
    AssertEquals(0, E.Elements.Count);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  try', '  except', '  end;',
        'end.', '']), PasProgram);
end;

procedure TTestStatementWriterTry.TestTryExceptOn;

var
    T: TPasImplTry;
    S: TPasImplSimple;
    E: TPasImplTryExcept;
    O: TPasImplExceptOn;

begin
    TestStatement(['Try', '  DoSomething;', 'except', 'On E : Exception do',
        'DoSomethingElse;', 'end']);
    T := AssertStatement('Try statement', TPasImplTry) as TPasImplTry;
    AssertEquals(1, T.Elements.Count);
    AssertNotNull(T.FinallyExcept);
    AssertNull(T.ElseBranch);
    AssertNotNull(T.Elements[0]);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    S := TPasImplSimple(T.Elements[0]);
    AssertExpression('DoSomething call', S.Expr, pekIdent, 'DoSomething');
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    AssertEquals('Except statement', TPasImplTryExcept, T.FinallyExcept.ClassType);
    E := TPasImplTryExcept(T.FinallyExcept);
    AssertEquals(1, E.Elements.Count);
    AssertEquals('Except on handler', TPasImplExceptOn, TPasElement(
        E.Elements[0]).ClassType);
    O := TPasImplExceptOn(E.Elements[0]);
    AssertEquals(1, O.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(O.Elements[0]).ClassType);
    AssertEquals('Exception Variable name', 'E', O.VariableName);
    AssertEquals('Exception Type name', 'Exception', O.TypeName);
    S := TPasImplSimple(O.Elements[0]);
    AssertExpression('DoSomethingElse call', S.Expr, pekIdent, 'DoSomethingElse');
    //  AssertEquals('Variable name',

    AssertPasWriteOutput('output', BuildString(['program afile;', '',
        '', 'begin', '  try', '    DoSomething;', '  except',
        '    On E : Exception do', '    DoSomethingElse;', '  end;',
        'end.', '']), PasProgram);
end;

procedure TTestStatementWriterTry.TestTryExceptOn2;

var
    T: TPasImplTry;
    S: TPasImplSimple;
    E: TPasImplTryExcept;
    O: TPasImplExceptOn;

begin
    TestStatement(['Try', '  DoSomething;', 'except',
        'On E : Exception do', 'DoSomethingElse;',
        'On Y : Exception2 do', 'DoSomethingElse2;', 'end']);
    T := AssertStatement('Try statement', TPasImplTry) as TPasImplTry;
    AssertEquals(1, T.Elements.Count);
    AssertNotNull(T.FinallyExcept);
    AssertNull(T.ElseBranch);
    AssertNotNull(T.Elements[0]);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    S := TPasImplSimple(T.Elements[0]);
    AssertExpression('DoSomething call', S.Expr, pekIdent, 'DoSomething');
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    AssertEquals('Except statement', TPasImplTryExcept, T.FinallyExcept.ClassType);
    E := TPasImplTryExcept(T.FinallyExcept);
    AssertEquals(2, E.Elements.Count);
    // Exception handler 1
    AssertEquals('Except on handler', TPasImplExceptOn, TPasElement(
        E.Elements[0]).ClassType);
    O := TPasImplExceptOn(E.Elements[0]);
    AssertEquals(1, O.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(O.Elements[0]).ClassType);
    AssertEquals('Exception Variable name', 'E', O.VariableName);
    AssertEquals('Exception Type name', 'Exception', O.TypeName);
    S := TPasImplSimple(O.Elements[0]);
    AssertExpression('DoSomethingElse call', S.Expr, pekIdent, 'DoSomethingElse');
    // Exception handler 2
    AssertEquals('Except on handler', TPasImplExceptOn, TPasElement(
        E.Elements[1]).ClassType);
    O := TPasImplExceptOn(E.Elements[1]);
    AssertEquals(1, O.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(O.Elements[0]).ClassType);
    AssertEquals('Exception Variable name', 'Y', O.VariableName);
    AssertEquals('Exception Type name', 'Exception2', O.TypeName);
    S := TPasImplSimple(O.Elements[0]);
    AssertExpression('DoSomethingElse call', S.Expr, pekIdent, 'DoSomethingElse2');
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  try', '    DoSomething;',
        '  except', '    On E : Exception do', '    DoSomethingElse;',
        '    On Y : Exception2 do', '    DoSomethingElse2;', '  end;',
        'end.', '']), PasProgram);
end;

procedure TTestStatementWriterTry.TestTryExceptOnElse;
var
    T: TPasImplTry;
    S: TPasImplSimple;
    E: TPasImplTryExcept;
    O: TPasImplExceptOn;
    EE: TPasImplTryExceptElse;
    I: TPasImplIfElse;

begin
    DeclareVar('Boolean', 'b');
    // Check that Else belongs to Except, not to IF

    TestStatement(['Try', '  DoSomething;', 'except', 'On E : Exception do',
        'if b then', 'DoSomethingElse;', 'else', 'DoSomethingMore;', 'end']);
    T := AssertStatement('Try statement', TPasImplTry) as TPasImplTry;
    AssertEquals(1, T.Elements.Count);
    AssertNotNull(T.FinallyExcept);
    AssertNotNull(T.ElseBranch);
    AssertNotNull(T.Elements[0]);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    S := TPasImplSimple(T.Elements[0]);
    AssertExpression('DoSomething call', S.Expr, pekIdent, 'DoSomething');
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    AssertEquals('Except statement', TPasImplTryExcept, T.FinallyExcept.ClassType);
    E := TPasImplTryExcept(T.FinallyExcept);
    AssertEquals(1, E.Elements.Count);
    AssertEquals('Except on handler', TPasImplExceptOn, TPasElement(
        E.Elements[0]).ClassType);
    O := TPasImplExceptOn(E.Elements[0]);
    AssertEquals('Exception Variable name', 'E', O.VariableName);
    AssertEquals('Exception Type name', 'Exception', O.TypeName);
    AssertEquals(1, O.Elements.Count);
    AssertEquals('Simple statement', TPasImplIfElse, TPasElement(O.Elements[0]).ClassType);
    I := TPasImplIfElse(O.Elements[0]);
    AssertEquals(1, I.Elements.Count);
    AssertNull('No else barcnh for if', I.ElseBranch);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(I.Elements[0]).ClassType);
    S := TPasImplSimple(I.Elements[0]);
    AssertExpression('DoSomethingElse call', S.Expr, pekIdent, 'DoSomethingElse');
    AssertEquals('Except Else statement', TPasImplTryExceptElse, T.ElseBranch.ClassType);
    EE := TPasImplTryExceptElse(T.ElseBranch);
    AssertEquals(1, EE.Elements.Count);
    AssertNotNull(EE.Elements[0]);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(EE.Elements[0]).ClassType);
    S := TPasImplSimple(EE.Elements[0]);
    AssertExpression('DoSomething call', S.Expr, pekIdent, 'DoSomethingMore');
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'var', '  b: Boolean;', '', 'begin',
        '  try', '    DoSomething;', '  except', '    On E : Exception do',
        '    if b then', '      DoSomethingElse;', '  else',
        '    DoSomethingMore;', '  end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterTry.TestTryExceptOnIfElse;
var
    T: TPasImplTry;
    S: TPasImplSimple;
    E: TPasImplTryExcept;
    O: TPasImplExceptOn;
    EE: TPasImplTryExceptElse;

begin
    TestStatement(['Try', '  DoSomething;', 'except', 'On E : Exception do',
        'DoSomethingElse;', 'else', 'DoSomethingMore;', 'end']);
    T := AssertStatement('Try statement', TPasImplTry) as TPasImplTry;
    AssertEquals(1, T.Elements.Count);
    AssertNotNull(T.FinallyExcept);
    AssertNotNull(T.ElseBranch);
    AssertNotNull(T.Elements[0]);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    S := TPasImplSimple(T.Elements[0]);
    AssertExpression('DoSomething call', S.Expr, pekIdent, 'DoSomething');
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    AssertEquals('Except statement', TPasImplTryExcept, T.FinallyExcept.ClassType);
    E := TPasImplTryExcept(T.FinallyExcept);
    AssertEquals(1, E.Elements.Count);
    AssertEquals('Except on handler', TPasImplExceptOn, TPasElement(
        E.Elements[0]).ClassType);
    O := TPasImplExceptOn(E.Elements[0]);
    AssertEquals('Exception Variable name', 'E', O.VariableName);
    AssertEquals('Exception Type name', 'Exception', O.TypeName);
    AssertEquals(1, O.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(O.Elements[0]).ClassType);
    S := TPasImplSimple(O.Elements[0]);
    AssertExpression('DoSomethingElse call', S.Expr, pekIdent, 'DoSomethingElse');
    AssertEquals('Except Else statement', TPasImplTryExceptElse, T.ElseBranch.ClassType);
    EE := TPasImplTryExceptElse(T.ElseBranch);
    AssertEquals(1, EE.Elements.Count);
    AssertNotNull(EE.Elements[0]);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(EE.Elements[0]).ClassType);
    S := TPasImplSimple(EE.Elements[0]);
    AssertExpression('DoSomething call', S.Expr, pekIdent, 'DoSomethingMore');
    AssertPasWriteOutput('output', BuildString(
        ['program afile;', '', '', 'begin', '  try', '    DoSomething;',
        '  except', '    On E : Exception do', '    DoSomethingElse;', '  else',
        '    DoSomethingMore;', '  end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterTry.TestTryExceptOnElseNoSemicolo;
var
    T: TPasImplTry;
    S: TPasImplSimple;
    E: TPasImplTryExcept;
    O: TPasImplExceptOn;
    EE: TPasImplTryExceptElse;
begin
    TestStatement(['Try', '  DoSomething;', 'except', 'On E : Exception do',
        'DoSomethingElse', 'else', 'DoSomethingMore', 'end']);
    T := AssertStatement('Try statement', TPasImplTry) as TPasImplTry;
    AssertEquals(1, T.Elements.Count);
    AssertNotNull(T.FinallyExcept);
    AssertNotNull(T.ElseBranch);
    AssertNotNull(T.Elements[0]);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    S := TPasImplSimple(T.Elements[0]);
    AssertExpression('DoSomething call', S.Expr, pekIdent, 'DoSomething');
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    AssertEquals('Except statement', TPasImplTryExcept, T.FinallyExcept.ClassType);
    E := TPasImplTryExcept(T.FinallyExcept);
    AssertEquals(1, E.Elements.Count);
    AssertEquals('Except on handler', TPasImplExceptOn, TPasElement(
        E.Elements[0]).ClassType);
    O := TPasImplExceptOn(E.Elements[0]);
    AssertEquals('Exception Variable name', 'E', O.VariableName);
    AssertEquals('Exception Type name', 'Exception', O.TypeName);
    AssertEquals(1, O.Elements.Count);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(O.Elements[0]).ClassType);
    S := TPasImplSimple(O.Elements[0]);
    AssertExpression('DoSomethingElse call', S.Expr, pekIdent, 'DoSomethingElse');
    AssertEquals('Except Else statement', TPasImplTryExceptElse, T.ElseBranch.ClassType);
    EE := TPasImplTryExceptElse(T.ElseBranch);
    AssertEquals(1, EE.Elements.Count);
    AssertNotNull(EE.Elements[0]);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(EE.Elements[0]).ClassType);
    S := TPasImplSimple(EE.Elements[0]);
    AssertExpression('DoSomething call', S.Expr, pekIdent, 'DoSomethingMore');
    AssertPasWriteOutput('output', BuildString(
        ['program afile;', '', '', 'begin', '  try', '    DoSomething;',
        '  except', '    On E : Exception do', '    DoSomethingElse;', '  else',
        '    DoSomethingMore;', '  end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterTry.TestTryExceptRaise;
var
    T: TPasImplTry;
    S: TPasImplSimple;
    E: TPasImplTryExcept;

begin
    TestStatement(['Try', '  DoSomething;', 'except', '  raise', 'end']);
    T := AssertStatement('Try statement', TPasImplTry) as TPasImplTry;
    AssertEquals(1, T.Elements.Count);
    AssertNotNull(T.FinallyExcept);
    AssertNull(T.ElseBranch);
    AssertNotNull(T.Elements[0]);
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    S := TPasImplSimple(T.Elements[0]);
    AssertExpression('DoSomething call', S.Expr, pekIdent, 'DoSomething');
    AssertEquals('Simple statement', TPasImplSimple, TPasElement(T.Elements[0]).ClassType);
    AssertEquals('Except statement', TPasImplTryExcept, T.FinallyExcept.ClassType);
    E := TPasImplTryExcept(T.FinallyExcept);
    AssertEquals(1, E.Elements.Count);
    AssertEquals('Raise statement', TPasImplRaise, TPasElement(E.Elements[0]).ClassType);
    AssertPasWriteOutput('output', BuildString(
        ['program afile;', '', '', 'begin', '  try', '    DoSomething;',
        '  except', '    raise;', '  end;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterAsm.TestAsm;

var
    T: TPasImplAsmStatement;

begin
    TestStatement(['asm', '  mov eax,1', 'end;']);
    T := AssertStatement('Asm statement', TPasImplAsmStatement) as TPasImplAsmStatement;
    AssertEquals('Asm tokens', 4, T.Tokens.Count);
    AssertEquals('token 1 ', 'mov', T.Tokens[0]);
    AssertEquals('token 2 ', 'eax', T.Tokens[1]);
    AssertEquals('token 3 ', ',', T.Tokens[2]);
    AssertEquals('token 4 ', '1', T.Tokens[3]);
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterAsm.TestAsmBlock;
begin
    Source.Add('{$MODE DELPHI}');
    Source.Add('function BitsHighest(X: Cardinal): Integer;');
    Source.Add('asm');
    Source.Add('end;');
    Source.Add('begin');
    Source.Add('end.');
    ParseModule;
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'function BitsHighest(X: Cardinal): Integer;', 'begin',
        'end;', '', '', 'begin', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterAsm.TestAsmBlockWithEndLabel;
begin
    Source.Add('{$MODE DELPHI}');
    Source.Add('function BitsHighest(X: Cardinal): Integer;');
    Source.Add('asm');
    Source.Add('  MOV ECX, EAX');
    Source.Add('  MOV EAX, -1');
    Source.Add('  BSR EAX, ECX');
    Source.Add('  JNZ @@End');
    Source.Add('  MOV EAX, -1');
    Source.Add('@@End:');
    Source.Add('end;');
    Source.Add('begin');
    Source.Add('end.');
    ParseModule;
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'function BitsHighest(X: Cardinal): Integer;', 'begin',
        'end;', '', '', 'begin', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterAsm.TestAsmBlockInIfThen;
begin
    Source.Add('{$MODE DELPHI}');
    Source.Add('function Get8087StatusWord(ClearExceptions: Boolean): Word;');
    Source.Add('  begin');
    Source.Add('    if ClearExceptions then');
    Source.Add('    asm');
    Source.Add('    end');
    Source.Add('    else');
    Source.Add('    asm');
    Source.Add('    end;');
    Source.Add('  end;');
    Source.Add('  begin');
    Source.Add('  end.');
    ParseModule;
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterSpecials.TestAssignToAddress;

begin
    AddStatements(['@Proc:=Nil']);
    ParseModule;
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  @ Proc := Nil;', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterSpecials.TestFinalizationNoSemicolon;
begin
    Source.Add('unit afile;');
    Source.Add('{$mode objfpc}');
    Source.Add('interface');
    Source.Add('implementation');
    Source.Add('initialization');
    Source.Add('  writeln(''qqq'')');
    Source.Add('finalization');
    Source.Add('  write(''rrr'')');
    ParseModule;
    AssertPasWriteOutput('output', BuildString(['unit afile;',
'',
'interface',
'',
'',
'',
'implementation',
'',
'',
'initialization',
'  writeln(''qqq'');',
'finalization',
'  write(''rrr'');',
'end.','']), Module);
end;

procedure TTestStatementWriterSpecials.TestMacroComment;
begin
    AddStatements(['{$MACRO ON}', '{$DEFINE func := //}', '  calltest;',
        '  func (''1'',''2'',''3'');', 'CallTest2;']);
    ParseModule;
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  calltest;', '  CallTest2;',
        'end.', '']), PasProgram);
end;

procedure TTestStatementWriterSpecials.TestPlatformIdentifier;
begin
    AddStatements(['write(platform);']);
    ParseModule;
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  write(platform);', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterSpecials.TestPlatformIdentifier2;
begin
    AddStatements(['write(libs+platform);']);
    ParseModule;
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  write(libs + platform);', 'end.', '']),
        PasProgram);
end;

procedure TTestStatementWriterSpecials.TestArgumentNameOn;
begin
    Source.Add('function TryOn(const on: boolean): boolean;');
    Source.Add('  begin');
    Source.Add('  end;');
    Source.Add('  begin');
    Source.Add('  end.');
    ParseModule;
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', 'function TryOn(const on: Boolean): Boolean;', 'begin',
        'end;', '', '', 'begin', 'end.', '']), PasProgram);
end;

procedure TTestStatementWriterSpecials.TestGotoInIfThen;

begin
    AddStatements(['{$goto on}', 'if expr then', '  dosomething',
        '   else if expr2 then', '    goto try_qword', '  else',
        '    dosomething;', '  try_qword:', '  dosomething;']);
    ParseModule;
    AssertPasWriteOutput('output', BuildString(['program afile;',
        '', '', 'begin', '  if expr then', '    dosomething',
        '  else if expr2 then', '    goto try_qword', '  else',
        '    dosomething;', '  try_qword:', '    dosomething;',
        'end.', '']), PasProgram);
end;

initialization
    RegisterTests('TestPasSrcWriter',
        [TTestStatementWriterEmpty, TTestStatementWriterBlock, TTestStatementWriterAssignment,
        TTestStatementWriterCall, TTestStatementWriterIf, TTestStatementWriterCase,
        TTestStatementWriterWith, TTestStatementWriterLoops, TTestStatementWriterRaise,
        TTestStatementWriterTry, TTestStatementWriterAsm, TTestStatementWriterSpecials]);

end.

