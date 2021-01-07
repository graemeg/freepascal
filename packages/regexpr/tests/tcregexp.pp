{ %norun }
unit tcregexp;

{$mode objfpc}{$H+}

{ $DEFINE DUMPTESTS} //define this to dump a

interface

uses
  Classes, SysUtils, fpcunit, testregistry, regexpr;

type

  { TTestRegexpr }

  TTestRegexpr= class(TTestCase)
  private
    FRE: TRegExpr;
  protected
    class function PrintableString(AString: string): string;
    Procedure RunRETest(aIndex : Integer);
    procedure SetUp; override;
    procedure TearDown; override;
    Property RE : TRegExpr read FRE;
  published
    procedure TestEmpty;
    Procedure RunTest1;
    Procedure RunTest2;
    Procedure RunTest3;
    Procedure RunTest4;
    Procedure RunTest5;
    Procedure RunTest6;
    Procedure RunTest7;
    Procedure RunTest8;
    Procedure RunTest9;
    Procedure RunTest10;
    Procedure RunTest11;
    Procedure RunTest12;
    Procedure RunTest13;
    Procedure RunTest14;
    Procedure RunTest15;
    Procedure RunTest16;
    Procedure RunTest17;
    Procedure RunTest18;
    Procedure RunTest19;
    Procedure RunTest20;
    Procedure RunTest21;
    Procedure RunTest22;
    Procedure RunTest23;
    Procedure RunTest24;
    Procedure RunTest25;
  end;

implementation

Type
  TRegExTest = record
    Expression: string;
    InputText: string;
    SubstitutionText: string;
    ExpectedResult: string;
    MatchStart: integer;
  end;

const
  testCases: array [1..25] of TRegExTest = (
    (
    expression: '\nd';
    inputText: 'abc'#13#10'def';
    substitutionText: '\n\x{10}\r\\';
    expectedResult: 'abc'#13#10#16#13'\ef';
    MatchStart: 0
    ),
    (
    expression: '(\w*)';
    inputText: 'name.ext';
    substitutionText: '$1.new';
    expectedResult: 'name.new.new.ext.new.new';
    MatchStart: 0
    ),
    (
    expression: #$d'('#$a')';
    inputText: 'word'#$d#$a;
    substitutionText: '$1';
    expectedResult: 'word'#$a;
    MatchStart: 0
    ),
    (
    expression: '(word)';
    inputText: 'word';
    substitutionText: '\U$1\\r';
    expectedResult: 'WORD\r';
    MatchStart: 0
    ),
    (
    expression: '(word)';
    inputText: 'word';
    substitutionText: '$1\n';
    expectedResult: 'word'#$a;
    MatchStart: 0
    ),
    (
    expression: '[A-Z]';
    inputText: '234578923457823659GHJK38';
    substitutionText: '';
    expectedResult: 'G';
    matchStart: 19;
    ),
    (
    expression: '[A-Z]*?';
    inputText: '234578923457823659ARTZU38';
    substitutionText: '';
    expectedResult: '';
    matchStart: 1
    ),
    (
    expression: '[A-Z]+';
    inputText: '234578923457823659ARTZU38';
    substitutionText: '';
    expectedResult: 'ARTZU';
    matchStart: 19
    ),
    (
    expression: '[A-Z][A-Z]*';
    inputText: '234578923457823659ARTZU38';
    substitutionText: '';
    expectedResult: 'ARTZU';
    matchStart: 19
    ),
    (
    expression: '[A-Z][A-Z]?';
    inputText: '234578923457823659ARTZU38';
    substitutionText: '';
    expectedResult: 'AR';
    matchStart: 19
    ),
    (
    expression: '[^\d]+';
    inputText: '234578923457823659ARTZU38';
    substitutionText: '';
    expectedResult: 'ARTZU';
    matchStart: 19
    ),
    (
    expression: '[A-Z][A-Z]?[A-Z]';
    inputText: '234578923457823659ARTZU38';
    substitutionText: '';
    expectedResult: 'ART';
    matchStart: 19
    ),
    (
    expression: '[A-Z][A-Z]*[0-9]';
    inputText: '234578923457823659ARTZU38';
    substitutionText: '';
    expectedResult: 'ARTZU3';
    matchStart: 19
    ),
    (
    expression: '[A-Z]+[0-9]';
    inputText: '234578923457823659ARTZU38';
    substitutionText: '';
    expectedResult: 'ARTZU3';
    matchStart: 19
    ),
    (
    expression: '(?i)[A-Z]';
    inputText: '234578923457823659a38';
    substitutionText: '';
    expectedResult: 'a';
    matchStart: 19
    ),
    (
    expression: '(?i)[a-z]';
    inputText: '234578923457823659A38';
    substitutionText: '';
    expectedResult: 'A';
    matchStart: 19
    ),
    (
    expression: '(foo)1234';
    inputText: '1234   foo1234XXXX';
    substitutionText: '';
    expectedResult: 'foo1234';
    matchStart: 8
    ),
    (
    expression: '(((foo)))1234';
    inputText: '1234   foo1234XXXX';
    substitutionText: '';
    expectedResult: 'foo1234';
    matchStart: 8
    ),
    (
    expression: '(foo)(1234)';
    inputText: '1234   foo1234XXXX';
    substitutionText: '';
    expectedResult: 'foo1234';
    matchStart: 8
    ),
    (
    expression: 'nofoo|foo';
    inputText: '1234   foo1234XXXX';
    substitutionText: '';
    expectedResult: 'foo';
    matchStart: 8
    ),
    (
    expression: '(nofoo|foo)1234';
    inputText: '1234   nofoo1234XXXX';
    substitutionText: '';
    expectedResult: 'nofoo1234';
    matchStart: 8
    ),
    (
    expression: '(nofoo|foo|anotherfoo)1234';
    inputText: '1234   nofoo1234XXXX';
    substitutionText: '';
    expectedResult: 'nofoo1234';
    matchStart: 8
    ),
    (
    expression: 'nofoo1234|foo1234';
    inputText: '1234   foo1234XXXX';
    substitutionText: '';
    expectedResult: 'foo1234';
    matchStart: 8
    ),
    (
    expression: '(\w*)';
    inputText: 'name.ext';
    substitutionText: '';
    expectedResult: 'name';
    matchStart: 1
    ),
    (
    expression: '\r(\n)';
    inputText: #$d#$a;
    substitutionText: '';
    expectedResult: #$d#$a;
    matchStart: 1
    )
  );


procedure TTestRegexpr.TestEmpty;
begin
  AssertNotNull('Have RE',RE);
  AssertFalse('UseOsLineEndOnReplace correcly set', RE.UseOsLineEndOnReplace);
end;

procedure TTestRegexpr.RunTest1;
begin
  RunRETest(1);
end;

procedure TTestRegexpr.RunTest2;
begin
  RunRETest(2);
end;

procedure TTestRegexpr.RunTest3;
begin
  RunRETest(3);
end;

procedure TTestRegexpr.RunTest4;
begin
  RunRETest(4);
end;

procedure TTestRegexpr.RunTest5;
begin
  RunRETest(5);
end;

procedure TTestRegexpr.RunTest6;
begin
  RunRETest(6);
end;

procedure TTestRegexpr.RunTest7;
begin
  RunRETest(7);
end;

procedure TTestRegexpr.RunTest8;
begin
  RunRETest(8);
end;

procedure TTestRegexpr.RunTest9;
begin
  RunRETest(9);
end;

procedure TTestRegexpr.RunTest10;
begin
  RunRETest(10);
end;

procedure TTestRegexpr.RunTest11;
begin
  RunRETest(11);
end;

procedure TTestRegexpr.RunTest12;
begin
  RunRETest(12);
end;

procedure TTestRegexpr.RunTest13;
begin
  RunRETest(13);
end;

procedure TTestRegexpr.RunTest14;
begin
  RunRETest(14);
end;

procedure TTestRegexpr.RunTest15;
begin
  RunRETest(15);
end;

procedure TTestRegexpr.RunTest16;
begin
  RunRETest(16);
end;

procedure TTestRegexpr.RunTest17;
begin
  RunRETest(17);
end;

procedure TTestRegexpr.RunTest18;
begin
  RunRETest(18);
end;

procedure TTestRegexpr.RunTest19;
begin
  RunRETest(19);
end;

procedure TTestRegexpr.RunTest20;
begin
  RunRETest(20);
end;

procedure TTestRegexpr.RunTest21;
begin
  RunRETest(21);
end;

procedure TTestRegexpr.RunTest22;
begin
  RunRETest(22);
end;

procedure TTestRegexpr.RunTest23;
begin
  RunRETest(23);
end;

procedure TTestRegexpr.RunTest24;
begin
  RunRETest(24);
end;

procedure TTestRegexpr.RunTest25;
begin
  RunRETest(25);
end;


Class function TTestRegexpr.PrintableString(AString: string): string;

var
    ch: Char;

begin
  Result := '';
  for ch in AString do
    if ch < #31 then
      Result := Result + '#' + IntToStr(Ord(ch))
    else
      Result := Result + ch;
end;

procedure TTestRegexpr.RunRETest(aIndex: Integer);


var
  T: TRegExTest;
  act : String;

begin
  T:=testCases[aIndex];
  RE.Expression:=T.Expression;
  RE.Compile;
{$IFDEF DUMPTESTS}
  Writeln('Test: ',TestName);
  writeln('  Modifiers "', RE.ModifierStr, '"');
  writeln('  Regular expression: ', T.Expression,' ,');
  writeln('  compiled into p-code: ');
  writeln('  ',RE.Dump);
  writeln('  Input text: "', PrintableString(T.inputText), '"');
  if (T.substitutionText <> '')  then
    Writeln('  Substitution text: "', PrintableString(T.substitutionText), '"');
{$ENDIF}
  if (T.SubstitutionText <> '') then
    begin
    act:=RE.Replace(T.InputText,T.SubstitutionText,True);
    AssertEquals('Replace failed', T.ExpectedResult,Act)
    end
  else
    begin
    RE.Exec(T.inputText);
    AssertEquals('Search position',T.MatchStart,RE.MatchPos[0]);
    AssertEquals('Matched text',T.ExpectedResult,RE.Match[0]);
    end;
end;

procedure TTestRegexpr.SetUp;
begin
  Inherited;
  FRE := TRegExpr.Create;
  FRE.UseOsLineEndOnReplace:=False;
end;

procedure TTestRegexpr.TearDown;
begin
  FreeAndNil(FRE);
  Inherited;
end;

initialization

  RegisterTest(TTestRegexpr);
end.

