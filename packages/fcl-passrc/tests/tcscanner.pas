unit tcscanner;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, typinfo, fpcunit, testregistry, pscanner;

type

  { TTestTokenFinder }

  TTestTokenFinder = class(TTestCase)
  Published
    Procedure TestFind;
  end;

  { TTestStreamLineReader }


  TTestStreamLineReader = class(TTestCase)
  Private
    FReader: TStreamLineReader;
  Protected
    procedure NewSource(Const Source : string);
    Procedure TestLine(Const ALine : String; ExpectEOF : Boolean = True);
    procedure TearDown; override;
  Published
    Procedure TestCreate;
    Procedure TestEOF;
    Procedure TestEmptyLine;
    Procedure TestEmptyLineCR;
    Procedure TestEmptyLineLF;
    Procedure TestEmptyLineCRLF;
    Procedure TestEmptyLineLFCR;
    Procedure TestOneLine;
    Procedure TestTwoLines;
  end;

  { TTestingPascalScanner }

  TTestingPascalScanner = Class(TPascalScanner)
  private
    FDoSpecial: Boolean;
  protected
    function HandleMacro(AIndex: integer): TToken;override;
  Public
    Property DoSpecial : Boolean Read FDoSpecial Write FDoSpecial;
  end;

  { TTestScanner }
  TTestScanner= class(TTestCase)
  Private
    FLI: String;
    FScanner : TPascalScanner;
    FResolver : TStreamResolver;
    FDoCommentCalled : Boolean;
    FComment: string;
    FPathPrefix : String;
    FTestTokenString: String;
  protected
    procedure DoComment(Sender: TObject; aComment: String);
    procedure SetUp; override;
    procedure TearDown; override;
    Procedure DoMultilineError;
    Function TokenToString(tk : TToken) : string;
    Procedure AssertEquals(Msg : String; Expected,Actual : TToken); overload;
    Procedure AssertEquals(Msg : String; Expected,Actual : TModeSwitch); overload;
    Procedure AssertEquals(Msg : String; Expected,Actual : TModeSwitches); overload;
    // creates a virtual source file with name 'afile.pp', prepended with PathPrefix
    procedure NewSource(Const Source : string; DoClear : Boolean = True);
    Procedure DoTestToken(t : TToken; Const ASource : String; Const CheckEOF : Boolean = True);
    Procedure TestToken(t : TToken; Const ASource : String; Const CheckEOF : Boolean = True);
    Procedure TestTokens(t : array of TToken; Const ASource : String; Const CheckEOF : Boolean = True;Const DoClear : Boolean = True);
    Property LastIDentifier : String Read FLI Write FLi;
    Property Scanner : TPascalScanner Read FScanner;
    // Path for source filename.
    Property PathPrefix : String Read FPathPrefix Write FPathPrefix;
    Property TestTokenString : String Read FTestTokenString;
  published
    Procedure TestEmpty;
    procedure TestEOF;
    procedure TestWhitespace;
    procedure TestComment1;
    procedure TestComment2;
    procedure TestComment3;
    procedure TestComment4;
    procedure TestComment5;
    procedure TestComment6;
    procedure TestComment7;
    procedure TestComment8;
    procedure TestComment9;
    procedure TestNestedComment1;
    procedure TestNestedComment2;
    procedure TestNestedComment3;
    procedure TestNestedComment4;
    procedure TestNestedComment5;
    procedure TestonComment;
    procedure TestIdentifier;
    procedure TestSelf;
    procedure TestSelfNoToken;
    procedure TestString;
    procedure TestMultilineStringError;
    procedure TestMultilineStringSource;
    Procedure TestMultilineStringLF;
    Procedure TestMultilineStringCR;
    Procedure TestMultilineStringCRLF;
    Procedure TestMultilineStringPlatform;
    Procedure TestMultilineLineEndingDirective;
    Procedure TestMultilineTrimLeftDirective;
    procedure TestMultilineStringTrimAll;
    procedure TestMultilineStringTrimAuto;
    procedure TestMultilineStringTrim2;
    procedure TestNumber;
    procedure TestChar;
    procedure TestCharString;
    procedure TestCaretString;
    procedure TestBraceOpen;
    procedure TestBraceClose;
    procedure TestMul;
    procedure TestPlus;
    procedure TestComma;
    procedure TestMinus;
    procedure TestDot;
    procedure TestDivision;
    procedure TestColon;
    procedure TestSemicolon;
    procedure TestLessThan;
    procedure TestEqual;
    procedure TestGreaterThan;
    procedure TestAt;
    procedure TestSquaredBraceOpen;
    procedure TestSquaredBraceClose;
    procedure TestCaret;
    procedure TestBackslash;
    procedure TestDotDot;
    procedure TestAssign;
    procedure TestAssignPlus;
    procedure TestAssignMinus;
    procedure TestAssignMul;
    procedure TestAssignDivision;
    procedure TestNotEqual;
    procedure TestLessEqualThan;
    procedure TestGreaterEqualThan;
    procedure TestPower;
    procedure TestSymmetricalDifference;
    procedure TestAbsolute;
    procedure TestAnd;
    procedure TestArray;
    procedure TestAs;
    procedure TestAsm;
    procedure TestBegin;
    procedure TestBitpacked;
    procedure TestCase;
    procedure TestClass;
    procedure TestConst;
    procedure TestConstructor;
    procedure TestDestructor;
    procedure TestDispinterface;
    procedure TestDiv;
    procedure TestDo;
    procedure TestDownto;
    procedure TestElse;
    procedure TestEnd;
    procedure TestExcept;
    procedure TestExports;
    procedure TestFalse;
    procedure TestFile;
    procedure TestFinalization;
    procedure TestFinally;
    procedure TestFor;
    procedure TestFunction;
    procedure TestGeneric;
    procedure TestGoto;
    Procedure TestHelper;
    procedure TestIf;
    procedure TestImplementation;
    procedure TestIn;
    procedure TestInherited;
    procedure TestInitialization;
    procedure TestInline;
    procedure TestInterface;
    procedure TestIs;
    procedure TestLabel;
    procedure TestLibrary;
    procedure TestMod;
    procedure TestNil;
    procedure TestNot;
    procedure TestObject;
    procedure TestOf;
    procedure TestOn;
    procedure TestOperator;
    procedure TestOr;
    procedure TestPacked;
    procedure TestProcedure;
    procedure TestProgram;
    procedure TestProperty;
    procedure TestRaise;
    procedure TestRecord;
    procedure TestRepeat;
    procedure TestResourceString;
    procedure TestSet;
    procedure TestShl;
    procedure TestShr;
    procedure TestShlC;
    procedure TestShrC;
    procedure TestSpecialize;
    procedure TestThen;
    procedure TestThreadvar;
    procedure TestTo;
    procedure TestTrue;
    procedure TestTry;
    procedure TestType;
    procedure TestUnit;
    procedure TestUntil;
    procedure TestUses;
    procedure TestVar;
    procedure TestWhile;
    procedure TestWith;
    procedure TestXor;
    procedure TestLineEnding;
    procedure TestObjCClass;
    procedure TestObjCClass2;
    procedure TestObjCProtocol;
    procedure TestObjCProtocol2;
    procedure TestObjCCategory;
    procedure TestObjCCategory2;
    procedure TestTab;
    Procedure TestEscapedKeyWord;
    Procedure TestTokenSeries;
    Procedure TestTokenSeriesNoWhiteSpace;
    Procedure TestTokenSeriesComments;
    Procedure TestTokenSeriesNoComments;
    Procedure TestDefine0;
    procedure TestDefine0Spaces;
    procedure TestDefine0Spaces2;
    procedure TestDefine01;
    Procedure TestDefine1;
    Procedure TestDefine2;
    Procedure TestDefine21;
    procedure TestDefine22;
    Procedure TestDefine3;
    Procedure TestDefine4;
    Procedure TestDefine5;
    Procedure TestDefine6;
    Procedure TestDefine7;
    Procedure TestDefine8;
    Procedure TestDefine9;
    Procedure TestDefine10;
    Procedure TestDefine11;
    Procedure TestDefine12;
    Procedure TestDefine13;
    Procedure TestDefine14;
    Procedure TestInclude;
    Procedure TestInclude2;
    Procedure TestInclude3;
    Procedure TestIncludeString;
    Procedure TestIncludeStringFile;
    Procedure TestIncludeString2Lines;
    Procedure TestUnDefine1;
    Procedure TestMacro1;
    procedure TestMacro2;
    procedure TestMacro3;
    procedure TestMacroHandling;
    procedure TestIFDefined;
    procedure TestIFUnDefined;
    procedure TestIFAnd;
    procedure TestIFAndShortEval;
    procedure TestIFOr;
    procedure TestIFOrShortEval;
    procedure TestIFXor;
    procedure TestIFAndOr;
    procedure TestIFEqual;
    procedure TestIFNotEqual;
    procedure TestIFGreaterThan;
    procedure TestIFGreaterEqualThan;
    procedure TestIFLesserThan;
    procedure TestIFLesserEqualThan;
    procedure TestIFDefinedElseIf;
    procedure TestIfError;
    procedure TestIFCDefined;
    procedure TestIFCNotDefined;
    procedure TestIFCAndDefined;
    procedure TestIFCFalse;
    Procedure TestModeSwitch;
    Procedure TestOperatorIdentifier;
    Procedure TestUTF8BOM;
    Procedure TestBooleanSwitch;
  end;

implementation

{ TTestingPascalScanner }

function TTestingPascalScanner.HandleMacro(AIndex: integer): TToken;
begin
  if DoSpecial then
    begin
    Result:=tkIdentifier;
    SetCurTokenstring('somethingweird');
    end
  else
    Result:=inherited HandleMacro(AIndex);
end;

{ TTestTokenFinder }

procedure TTestTokenFinder.TestFind;

Var
  tk,tkr : TToken;
  S : string;
  B : Boolean;

begin
  For tk:=tkAbsolute to tkXor do
    begin
    S:=tokenInfos[tk];
    B:=IsNamedToken(S,tkr);
    AssertEquals('Token '+S+' is a token',true,B);
    AssertEquals('Token '+S+' returns correct token',Ord(tk),Ord(tkr));
    end;
end;

{ TTestStreamLineReader }

procedure TTestStreamLineReader.NewSource(Const Source: string);
begin
  FReader:=TStringStreamLineReader.Create('afile',Source);
end;

procedure TTestStreamLineReader.TestLine(const ALine: String; ExpectEOF: Boolean);
begin
  AssertNotNull('Have reader',FReader);
  AssertEquals('Reading source line',ALine,FReader.ReadLine);
  if ExpectEOF then
    AssertEquals('End of file reached',True,FReader.IsEOF);
end;

procedure TTestStreamLineReader.TearDown;
begin
  inherited TearDown;
  If Assigned(FReader) then
    FreeAndNil(Freader);
end;

procedure TTestStreamLineReader.TestCreate;
begin
  FReader:=TStreamLineReader.Create('afile');
  AssertEquals('Correct filename','afile',FReader.FileName);
  AssertEquals('Initially empty',True,FReader.isEOF);
end;

procedure TTestStreamLineReader.TestEOF;
begin
  NewSource('');
  AssertEquals('Empty stream',True,FReader.IsEOF);
end;

procedure TTestStreamLineReader.TestEmptyLine;
begin
  NewSource('');
  TestLine('');
end;

procedure TTestStreamLineReader.TestEmptyLineCR;
begin
  NewSource(#13);
  TestLine('');
end;

procedure TTestStreamLineReader.TestEmptyLineLF;
begin
  NewSource(#10);
  TestLine('');
end;

procedure TTestStreamLineReader.TestEmptyLineCRLF;
begin
  NewSource(#13#10);
  TestLine('');
end;

procedure TTestStreamLineReader.TestEmptyLineLFCR;
begin
  NewSource(#10#13);
  TestLine('',False);
  TestLine('');
end;

procedure TTestStreamLineReader.TestOneLine;

Const
    S = 'a line with text';
begin
  NewSource(S);
  TestLine(S);
end;

procedure TTestStreamLineReader.TestTwoLines;
Const
    S = 'a line with text';
begin
  NewSource(S+sLineBreak+S);
  TestLine(S,False);
  TestLine(S);
end;

{ ---------------------------------------------------------------------
  TTestScanner
  ---------------------------------------------------------------------}

procedure TTestScanner.DoComment(Sender: TObject; aComment: String);
begin
  FDoCommentCalled:=True;
  FComment:=aComment;
end;

procedure TTestScanner.SetUp;
begin
  FTestTokenString:='';
  FDoCommentCalled:=False;
  FResolver:=TStreamResolver.Create;
  FResolver.OwnsStreams:=True;
  FScanner:=TTestingPascalScanner.Create(FResolver);
  // Do nothing
end; 

procedure TTestScanner.TearDown; 
begin
  FreeAndNil(FScanner);
  FreeAndNil(FResolver);
end;

procedure TTestScanner.DoMultilineError;
begin
  TestToken(pscanner.tkString,'`A '#10'multiline string`');
end;

function TTestScanner.TokenToString(tk: TToken): string;
begin
  Result:=GetEnumName(TypeInfo(TToken),Ord(tk));
end;

procedure TTestScanner.AssertEquals(Msg: String; Expected, Actual: TToken);
begin
  AssertEquals(Msg,TokenToString(Expected),TokenToString(Actual));
end;

procedure TTestScanner.AssertEquals(Msg: String; Expected, Actual: TModeSwitch);
begin
  AssertEquals(Msg,GetEnumName(TypeInfo(TModeSwitch),Ord(Expected)),
                   GetEnumName(TypeInfo(TModeSwitch),Ord(Actual)))
end;

procedure TTestScanner.AssertEquals(Msg: String; Expected, Actual: TModeSwitches);

  Function ToString(S : TModeSwitches) : String;

  Var
    M : TModeSwitch;

  begin
    Result:='';
    For M in TModeswitch do
      if M in S then
        begin
        If (Result<>'') then
          Result:=Result+', ';
        Result:=Result+GetEnumName(TypeInfo(TModeSwitch), Ord(M));
        end;
  end;

begin
  AssertEquals(Msg,ToString(Expected),ToString(Actual));
end;

procedure TTestScanner.NewSource(const Source: string; DoClear : Boolean = True);

Var
  aFile : String;

begin
  aFile:='';
  if DoClear then
    FResolver.Clear;
  if (FPathPrefix<>'') then
     aFile:=IncludeTrailingPathDelimiter(FPathPrefix);
  aFile:=aFile+'afile.pp';
  FResolver.AddStream(aFile,TStringStream.Create(Source));
  {$ifndef NOCONSOLE} // JC: To get the tests to run with GUI
  Writeln('// '+TestName);
  Writeln(Source);
  {$EndIf}
//  FreeAndNil(FScanner);
//  FScanner:=TTestingPascalScanner.Create(FResolver);
  FScanner.OpenFile(aFile);
end;

procedure TTestScanner.DoTestToken(t: TToken; const ASource: String;
  const CheckEOF: Boolean);

Var
  tk : ttoken;

begin
  NewSource(ASource);
  tk:=FScanner.FetchToken;
  AssertEquals('Read token equals expected token.',t,tk);
  FTestTokenString:=FScanner.CurTokenString;
  if CheckEOF then
    begin
    tk:=FScanner.FetchToken;
    if (tk=tkLineEnding) and not (t in [tkEOF,tkLineEnding]) then
      tk:=FScanner.FetchToken;
    AssertEquals('EOF reached.',tkEOF,FScanner.FetchToken);
    end;
end;

procedure TTestScanner.TestToken(t: TToken; const ASource: String;
  const CheckEOF: Boolean);
Var
  S : String;
begin
  DoTestToken(t,ASource);
  if (ASource<>'') then
    begin
    S:=ASource;
    S[1]:=Upcase(S[1]);
    DoTestToken(t,S);
    end;
  DoTestToken(t,UpperCase(ASource));
  DoTestToken(t,LowerCase(ASource),CheckEOF);
end;

procedure TTestScanner.TestTokens(t: array of TToken; const ASource: String;
  const CheckEOF: Boolean; const DoClear: Boolean);
Var
  tk : ttoken;
  i : integer;

begin
  NewSource(ASource,DoClear);
  For I:=Low(t) to High(t) do
    begin
    tk:=FScanner.FetchToken;
    AssertEquals(Format('Read token %d equals expected token.',[i]),t[i],tk);
    if tk=tkIdentifier then
      LastIdentifier:=FScanner.CurtokenString
    else if tk=tkString then
      fTestTokenString:=FScanner.CurTokenString;
    end;
  if CheckEOF then
    begin
    tk:=FScanner.FetchToken;
    if (tk=tkLineEnding) then
      tk:=FScanner.FetchToken;
    AssertEquals('EOF reached.',tkEOF,FScanner.FetchToken);
    end;
end;

procedure TTestScanner.TestEmpty;
begin
  AssertNotNull('Have Scanner',Scanner);
  AssertTrue('Options is empty',[]=Scanner.Options);
  AssertEquals('FPC modes is default',FPCModeSwitches,Scanner.CurrentModeSwitches);
end;

procedure TTestScanner.TestEOF;
begin
  TestToken(tkEOF,'')
end;

procedure TTestScanner.TestWhitespace;

begin
  TestToken(tkWhitespace,' ');
  TestToken(tkWhitespace,' ');
end;


procedure TTestScanner.TestComment1;

begin
  TestToken(tkComment,'{ comment }');
end;


procedure TTestScanner.TestComment2;

begin
  TestToken(tkComment,'(* comment *)');
end;


procedure TTestScanner.TestComment3;

begin
  TestToken(tkComment,'//');
end;

procedure TTestScanner.TestComment4;

begin
  DoTestToken(tkComment,'(* abc *)',False);
  AssertEquals('Correct comment',' abc ',Scanner.CurTokenString);
end;

procedure TTestScanner.TestComment5;

begin
  DoTestToken(tkComment,'(* abc'+LineEnding+'def *)',False);
  AssertEquals('Correct comment',' abc'+LineEnding+'def ',Scanner.CurTokenString);
end;

procedure TTestScanner.TestComment6;

begin
  DoTestToken(tkComment,'{ abc }',False);
  AssertEquals('Correct comment',' abc ',Scanner.CurTokenString);
end;

procedure TTestScanner.TestComment7;

begin
  DoTestToken(tkComment,'{ abc'+LineEnding+'def }',False);
  AssertEquals('Correct comment',' abc'+LineEnding+'def ',Scanner.CurTokenString);
end;

procedure TTestScanner.TestComment8;

begin
  DoTestToken(tkComment,'// abc ',False);
  AssertEquals('Correct comment',' abc ',Scanner.CurTokenString);
end;

procedure TTestScanner.TestComment9;

begin
  DoTestToken(tkComment,'// abc '+LineEnding,False);
  AssertEquals('Correct comment',' abc ',Scanner.CurTokenString);
end;

procedure TTestScanner.TestNestedComment1;
begin
  TestToken(tkComment,'// { comment } ');
end;

procedure TTestScanner.TestNestedComment2;
begin
  TestToken(tkComment,'(* { comment } *)');
end;

procedure TTestScanner.TestNestedComment3;
begin
  TestToken(tkComment,'{ { comment } }');
end;

procedure TTestScanner.TestNestedComment4;
begin
  TestToken(tkComment,'{ (* comment *) }');
end;

procedure TTestScanner.TestNestedComment5;
begin
  TestToken(tkComment,'(* (* comment *) *)');
end;

procedure TTestScanner.TestonComment;
begin
  FScanner.OnComment:=@DoComment;
  DoTestToken(tkComment,'(* abc *)',False);
  assertTrue('Comment called',FDoCommentCalled);
  AssertEquals('Correct comment',' abc ',Scanner.CurTokenString);
  AssertEquals('Correct comment token',' abc ',FComment);
end;


procedure TTestScanner.TestIdentifier;

begin
  TestToken(tkIdentifier,'identifier');
end;


procedure TTestScanner.TestString;

begin
  TestToken(pscanner.tkString,'''A string''');
end;

procedure TTestScanner.TestMultilineStringError;
begin
  AssertException('Need modeswitch',EScannerError,@DoMultilineError);
end;

procedure TTestScanner.TestMultilineStringSource;

const
  S = '''AB'#13#10'CD''';

begin
  Scanner.CurrentModeSwitches:=[msMultiLineStrings];
  Scanner.MultilineLineFeedStyle:=elSource;
  DoTestToken(pscanner.tkString,'`AB'#13#10'CD`');
  AssertEquals('Correct lineending',S,TestTokenString);
end;

procedure TTestScanner.TestMultilineStringLF;

const
  S = '''AB'#10'CD''';

begin
  Scanner.CurrentModeSwitches:=[msMultiLineStrings];
  Scanner.MultilineLineFeedStyle:=elLF;
  DoTestToken(pscanner.tkString,'`AB'#13#10'CD`');
  AssertEquals('Correct lineending',S,TestTokenString);
end;

procedure TTestScanner.TestMultilineStringCR;
const
  S = '''AB'#13'CD''';

begin
  Scanner.CurrentModeSwitches:=[msMultiLineStrings];
  Scanner.MultilineLineFeedStyle:=elCR;
  DoTestToken(pscanner.tkString,'`AB'#10'CD`');
  AssertEquals('Correct lineending',S,TestTokenString);
end;

procedure TTestScanner.TestMultilineStringCRLF;
const
  S = '''AB'#13#10'CD''';

begin
  Scanner.CurrentModeSwitches:=[msMultiLineStrings];
  Scanner.MultilineLineFeedStyle:=elCRLF;
  DoTestToken(pscanner.tkString,'`AB'#10'CD`');
  AssertEquals('Correct lineending',S,TestTokenString);
end;

procedure TTestScanner.TestMultilineStringPlatform;

const
  S = '''AB'+sLineBreak+'CD''';

begin
  Scanner.CurrentModeSwitches:=[msMultiLineStrings];
  Scanner.MultilineLineFeedStyle:=elPlatform;
  DoTestToken(pscanner.tkString,'`AB'#13#10'CD`');
  AssertEquals('Correct lineending',S,TestTokenString);
end;

procedure TTestScanner.TestMultilineLineEndingDirective;
begin
  AssertTrue('Default platform', FSCanner.MultilineLineFeedStyle=elPlatform);
  TestTokens([tkComment],'{$MULTILINESTRINGLINEENDING CR}');
  AssertTrue('CR', FSCanner.MultilineLineFeedStyle=elCR);
  TestTokens([tkComment],'{$MULTILINESTRINGLINEENDING LF}');
  AssertTrue('LF', FSCanner.MultilineLineFeedStyle=elLF);
  TestTokens([tkComment],'{$MULTILINESTRINGLINEENDING CRLF}');
  AssertTrue('CRLF', FSCanner.MultilineLineFeedStyle=elCRLF);
  TestTokens([tkComment],'{$MULTILINESTRINGLINEENDING SOURCE}');
  AssertTrue('SOURCE', FSCanner.MultilineLineFeedStyle=elSOURCE);
  TestTokens([tkComment],'{$MULTILINESTRINGLINEENDING PLATFORM}');
  AssertTrue('Platform', FSCanner.MultilineLineFeedStyle=elPlatform);

end;

procedure TTestScanner.TestMultilineTrimLeftDirective;
begin
  AssertTrue('Default', FSCanner.MultilineLineTrimLeft=0);
  TestTokens([tkComment],'{$MULTILINESTRINGTRIMLEFT 1}');
  AssertTrue('1', FSCanner.MultilineLineTrimLeft=1);
  TestTokens([tkComment],'{$MULTILINESTRINGTRIMLEFT 2}');
  AssertTrue('2', FSCanner.MultilineLineTrimLeft=2);
  TestTokens([tkComment],'{$MULTILINESTRINGTRIMLEFT ALL}');
  AssertTrue('ALL', FSCanner.MultilineLineTrimLeft=-2);
  TestTokens([tkComment],'{$MULTILINESTRINGTRIMLEFT AUTO}');
  AssertTrue('AUTO', FSCanner.MultilineLineTrimLeft=-1);
end;

procedure TTestScanner.TestMultilineStringTrimAll;

const
   S = '''AB'#10'CD''';

begin
  SCanner.MultilineLineTrimLeft:=-2;
  Scanner.CurrentModeSwitches:=[msMultiLineStrings];
  Scanner.MultilineLineFeedStyle:=elLF;
  DoTestToken(pscanner.tkString,'`AB'#13#10'    CD`');
  AssertEquals('Correct trim',S,TestTokenString);

end;

procedure TTestScanner.TestMultilineStringTrimAuto;
const
   S = '''AB'#10' CD''';

begin
  SCanner.MultilineLineTrimLeft:=-1;
  Scanner.CurrentModeSwitches:=[msMultiLineStrings];
  Scanner.MultilineLineFeedStyle:=elLF;
  Scanner.SkipWhiteSpace:=True;
  DoTestToken(pscanner.tkString,' `AB'#13#10'   CD`');
  AssertEquals('Correct trim',S,TestTokenString);
end;

procedure TTestScanner.TestMultilineStringTrim2;

const
  S = '''AB'#10' CD''';
  S2 = '''AB'#10'CD''';

begin
  SCanner.MultilineLineTrimLeft:=2;
  Scanner.CurrentModeSwitches:=[msMultiLineStrings];
  Scanner.MultilineLineFeedStyle:=elLF;
  Scanner.SkipWhiteSpace:=True;
  DoTestToken(pscanner.tkString,' `AB'#13#10'   CD`');
  AssertEquals('Correct trim',S,TestTokenString);
  DoTestToken(pscanner.tkString,' `AB'#13#10' CD`');
  AssertEquals('Correct trim 2',S2,TestTokenString);
end;

procedure TTestScanner.TestCharString;

begin
  TestToken(pscanner.tkChar,'''A''');
end;

procedure TTestScanner.TestCaretString;
begin

  TestTokens([tkIdentifier,tkWhiteSpace,tkEqual,tkwhiteSpace,pscanner.tkString,tkSemicolon],'a = ^C''abc'';',false);
end;

procedure TTestScanner.TestNumber;

begin
  TestToken(tkNumber,'123');
end;


procedure TTestScanner.TestChar;

begin
  TestToken(pscanner.tkChar,'#65 ', false);
end;


procedure TTestScanner.TestBraceOpen;

begin
  TestToken(tkBraceOpen,'(');
end;


procedure TTestScanner.TestBraceClose;

begin
  TestToken(tkBraceClose,')');
end;


procedure TTestScanner.TestMul;

begin
  TestToken(tkMul,'*');
end;


procedure TTestScanner.TestPlus;

begin
  TestToken(tkPlus,'+');
end;


procedure TTestScanner.TestComma;

begin
  TestToken(tkComma,',');
end;


procedure TTestScanner.TestMinus;

begin
  TestToken(tkMinus,'-');
end;


procedure TTestScanner.TestDot;

begin
  TestToken(tkDot,'.');
end;


procedure TTestScanner.TestDivision;

begin
  TestToken(tkDivision,'/');
end;


procedure TTestScanner.TestColon;

begin
  TestToken(tkColon,':');
end;


procedure TTestScanner.TestSemicolon;

begin
  TestToken(tkSemicolon,';');
end;


procedure TTestScanner.TestLessThan;

begin
  TestToken(tkLessThan,'<');
end;


procedure TTestScanner.TestEqual;

begin
  TestToken(tkEqual,'=');
end;


procedure TTestScanner.TestGreaterThan;

begin
  TestToken(tkGreaterThan,'>');
end;


procedure TTestScanner.TestAt;

begin
  TestToken(tkAt,'@');
end;


procedure TTestScanner.TestSquaredBraceOpen;

begin
  TestToken(tkSquaredBraceOpen,'[');
  TestToken(tkSquaredBraceOpen,'(.'); // JC: Test for the BraceDotOpen
end;


procedure TTestScanner.TestSquaredBraceClose;

begin
  TestToken(tkSquaredBraceClose,']');
  TestToken(tkSquaredBraceClose,'.)'); // JC: Test for the DotBraceClose
  TestTokens([tkNumber,tkSquaredBraceClose],'1.)'); // JC: Test for a Number followed by DotBraceClose
end;


procedure TTestScanner.TestCaret;

begin
  TestToken(tkCaret,'^');
end;


procedure TTestScanner.TestBackslash;

begin
  TestToken(tkBackslash,'\');
end;


procedure TTestScanner.TestDotDot;

begin
  TestToken(tkDotDot,'..');
end;


procedure TTestScanner.TestAssign;

begin
  TestToken(tkAssign,':=');
end;

procedure TTestScanner.TestAssignPlus;
begin
  TestTokens([tkPlus,tkEqual],'+=');
  FScanner.Options:=[po_cassignments];
  TestToken(tkAssignPlus,'+=');
end;

procedure TTestScanner.TestAssignMinus;
begin
  TestTokens([tkMinus,tkEqual],'-=');
  FScanner.Options:=[po_cassignments];
  TestToken(tkAssignMinus,'-=');
end;

procedure TTestScanner.TestAssignMul;
begin
  TestTokens([tkMul,tkEqual],'*=');
  FScanner.Options:=[po_cassignments];
  TestToken(tkAssignMul,'*=');
end;

procedure TTestScanner.TestAssignDivision;
begin
  TestTokens([tkDivision,tkEqual],'/=');
  FScanner.Options:=[po_cassignments];
  TestToken(tkAssignDivision,'/=');
end;


procedure TTestScanner.TestNotEqual;

begin
  TestToken(tkNotEqual,'<>');
end;


procedure TTestScanner.TestLessEqualThan;

begin
  TestToken(tkLessEqualThan,'<=');
end;


procedure TTestScanner.TestGreaterEqualThan;

begin
  TestToken(tkGreaterEqualThan,'>=');
end;


procedure TTestScanner.TestPower;

begin
  TestToken(tkPower,'**');
end;


procedure TTestScanner.TestSymmetricalDifference;

begin
  TestToken(tkSymmetricalDifference,'><');
end;


procedure TTestScanner.TestAbsolute;

begin
  TestToken(tkabsolute,'absolute');
end;


procedure TTestScanner.TestAnd;

begin
  TestToken(tkand,'and');
end;


procedure TTestScanner.TestArray;

begin
  TestToken(tkarray,'array');
end;


procedure TTestScanner.TestAs;

begin
  TestToken(tkas,'as');
end;


procedure TTestScanner.TestAsm;

begin
  TestToken(tkasm,'asm');
end;


procedure TTestScanner.TestBegin;

begin
  TestToken(tkbegin,'begin');
end;


procedure TTestScanner.TestBitpacked;

begin
  TestToken(tkbitpacked,'bitpacked');
end;


procedure TTestScanner.TestCase;

begin
  TestToken(tkcase,'case');
end;


procedure TTestScanner.TestClass;

begin
  TestToken(tkclass,'class');
end;


procedure TTestScanner.TestConst;

begin
  TestToken(tkconst,'const');
end;


procedure TTestScanner.TestConstructor;

begin
  TestToken(tkconstructor,'constructor');
end;


procedure TTestScanner.TestDestructor;

begin
  TestToken(tkdestructor,'destructor');
end;

procedure TTestScanner.TestDispinterface;
begin
  TestToken(tkdispinterface,'dispinterface');
end;

procedure TTestScanner.TestDiv;

begin
  TestToken(tkdiv,'div');
end;


procedure TTestScanner.TestDo;

begin
  TestToken(tkdo,'do');
end;


procedure TTestScanner.TestDownto;

begin
  TestToken(tkdownto,'downto');
end;


procedure TTestScanner.TestElse;

begin
  TestToken(tkelse,'else');
end;


procedure TTestScanner.TestEnd;

begin
  TestToken(tkend,'end');
end;


procedure TTestScanner.TestExcept;

begin
  TestToken(tkexcept,'except');
end;


procedure TTestScanner.TestExports;

begin
  TestToken(tkexports,'exports');
end;


procedure TTestScanner.TestFalse;

begin
  TestToken(tkfalse,'false');
end;


procedure TTestScanner.TestFile;

begin
  TestToken(tkfile,'file');
end;


procedure TTestScanner.TestFinalization;

begin
  TestToken(tkfinalization,'finalization');
end;


procedure TTestScanner.TestFinally;

begin
  TestToken(tkfinally,'finally');
end;


procedure TTestScanner.TestFor;

begin
  TestToken(tkfor,'for');
end;


procedure TTestScanner.TestFunction;

begin
  TestToken(tkfunction,'function');
end;


procedure TTestScanner.TestGeneric;

begin
  TestToken(tkgeneric,'generic');
end;


procedure TTestScanner.TestGoto;

begin
  TestToken(tkgoto,'goto');
end;

procedure TTestScanner.TestHelper;
begin
  TestToken(tkIdentifier,'helper');
end;


procedure TTestScanner.TestIf;

begin
  TestToken(tkif,'if');
end;


procedure TTestScanner.TestImplementation;

begin
  TestToken(tkimplementation,'implementation');
end;


procedure TTestScanner.TestIn;

begin
  TestToken(tkin,'in');
end;


procedure TTestScanner.TestInherited;

begin
  TestToken(tkinherited,'inherited');
end;


procedure TTestScanner.TestInitialization;

begin
  TestToken(tkinitialization,'initialization');
end;


procedure TTestScanner.TestInline;

begin
  TestToken(tkinline,'inline');
end;


procedure TTestScanner.TestInterface;

begin
  TestToken(tkinterface,'interface');
end;


procedure TTestScanner.TestIs;

begin
  TestToken(tkis,'is');
end;


procedure TTestScanner.TestLabel;

begin
  TestToken(tklabel,'label');
end;


procedure TTestScanner.TestLibrary;

begin
  TestToken(tklibrary,'library');
end;


procedure TTestScanner.TestMod;

begin
  TestToken(tkmod,'mod');
end;


procedure TTestScanner.TestNil;

begin
  TestToken(tknil,'nil');
end;


procedure TTestScanner.TestNot;

begin
  TestToken(tknot,'not');
end;


procedure TTestScanner.TestObject;

begin
  TestToken(tkobject,'object');
end;


procedure TTestScanner.TestOf;

begin
  TestToken(tkof,'of');
end;


procedure TTestScanner.TestOn;

begin
  TestToken(tkIdentifier,'on');
end;


procedure TTestScanner.TestOperator;

begin
  Scanner.SetTokenOption(toOperatorToken);
  TestToken(tkoperator,'operator');
end;


procedure TTestScanner.TestOr;

begin
  TestToken(tkor,'or');
end;


procedure TTestScanner.TestPacked;

begin
  TestToken(tkpacked,'packed');
end;


procedure TTestScanner.TestProcedure;

begin
  TestToken(tkprocedure,'procedure');
end;


procedure TTestScanner.TestProgram;

begin
  TestToken(tkprogram,'program');
end;


procedure TTestScanner.TestProperty;

begin
  TestToken(tkproperty,'property');
end;


procedure TTestScanner.TestRaise;

begin
  TestToken(tkraise,'raise');
end;


procedure TTestScanner.TestRecord;

begin
  TestToken(tkrecord,'record');
end;


procedure TTestScanner.TestRepeat;

begin
  TestToken(tkrepeat,'repeat');
end;


procedure TTestScanner.TestResourceString;

begin
  TestToken(tkResourceString,'resourcestring');
end;


procedure TTestScanner.TestSelf;

begin
  FScanner.Options:=FScanner.Options + [po_selftoken];
  TestToken(tkself,'self');
end;

procedure TTestScanner.TestSelfNoToken;
begin
  TestToken(tkIdentifier,'self');
end;


procedure TTestScanner.TestSet;

begin
  TestToken(tkset,'set');
end;


procedure TTestScanner.TestShl;

begin
  TestToken(tkshl,'shl');
end;


procedure TTestScanner.TestShr;

begin
  TestToken(tkshr,'shr');
end;

procedure TTestScanner.TestShlC;
begin
  TestToken(tkshl,'<<');
end;

procedure TTestScanner.TestShrC;
begin
  TestToken(tkshr,'>>');
end;


procedure TTestScanner.TestSpecialize;

begin
  TestToken(tkspecialize,'specialize');
end;


procedure TTestScanner.TestThen;

begin
  TestToken(tkthen,'then');
end;


procedure TTestScanner.TestThreadvar;

begin
  TestToken(tkthreadvar,'threadvar');
end;


procedure TTestScanner.TestTo;

begin
  TestToken(tkto,'to');
end;


procedure TTestScanner.TestTrue;

begin
  TestToken(tktrue,'true');
end;


procedure TTestScanner.TestTry;

begin
  TestToken(tktry,'try');
end;


procedure TTestScanner.TestType;

begin
  TestToken(tktype,'type');
end;


procedure TTestScanner.TestUnit;

begin
  TestToken(tkunit,'unit');
end;


procedure TTestScanner.TestUntil;

begin
  TestToken(tkuntil,'until');
end;


procedure TTestScanner.TestUses;

begin
  TestToken(tkuses,'uses');
end;


procedure TTestScanner.TestVar;

begin
  TestToken(tkvar,'var');
end;


procedure TTestScanner.TestWhile;

begin
  TestToken(tkwhile,'while');
end;


procedure TTestScanner.TestWith;

begin
  TestToken(tkwith,'with');
end;


procedure TTestScanner.TestXor;

begin
  TestToken(tkxor,'xor');
end;


procedure TTestScanner.TestLineEnding;

begin
  TestToken(tkLineEnding,#10);
end;

procedure TTestScanner.TestObjCClass;
begin
  TestToken(tkObjCClass,'objcclass');
end;

procedure TTestScanner.TestObjCClass2;
begin
  TestTokens([tkComment,tkWhitespace,tkidentifier],'{$mode fpc} objcclass');
end;

procedure TTestScanner.TestObjCProtocol;
begin
  TestToken(tkObjCProtocol,'objcprotocol');
end;

procedure TTestScanner.TestObjCProtocol2;
begin
  TestTokens([tkComment,tkWhitespace,tkidentifier],'{$mode fpc} objcprotocol');
end;

procedure TTestScanner.TestObjCCategory;

begin
  TestToken(tkObjCCategory,'objccategory');
end;

procedure TTestScanner.TestObjCCategory2;
begin
  TestTokens([tkComment,tkWhitespace,tkidentifier],'{$mode fpc} objccategory');
end;


procedure TTestScanner.TestTab;

begin
  TestToken(tkTab,#9);
end;

procedure TTestScanner.TestEscapedKeyWord;
begin
  TestToken(tkIdentifier,'&xor');
end;

procedure TTestScanner.TestTokenSeries;
begin
  TestTokens([tkin,tkWhitespace,tkOf,tkWhiteSpace,tkthen,tkWhiteSpace,tkIdentifier],'in of then aninteger')
end;

procedure TTestScanner.TestTokenSeriesNoWhiteSpace;
begin
  FScanner.SkipWhiteSpace:=True;
  TestTokens([tkin,tkOf,tkthen,tkIdentifier],'in of then aninteger')
end;

procedure TTestScanner.TestTokenSeriesComments;
begin
  TestTokens([tkin,tkWhitespace,tkOf,tkWhiteSpace,tkComment,tkWhiteSpace,tkIdentifier],'in of {then} aninteger')
end;

procedure TTestScanner.TestTokenSeriesNoComments;
begin
  FScanner.SkipComments:=True;
  TestTokens([tkin,tkWhitespace,tkOf,tkWhiteSpace,tkWhiteSpace,tkIdentifier],'in of {then} aninteger')
end;

procedure TTestScanner.TestDefine0;
begin
  TestTokens([tkComment],'{$DEFINE NEVER}');
  AssertTrue('Define not defined', FSCanner.Defines.IndexOf('NEVER')<>-1);
end;

procedure TTestScanner.TestDefine0Spaces;
begin
  TestTokens([tkComment],'{$DEFINE  NEVER}');
  AssertTrue('Define not defined',FSCanner.Defines.IndexOf('NEVER')<>-1);
end;

procedure TTestScanner.TestDefine0Spaces2;
begin
  TestTokens([tkComment],'{$DEFINE NEVER }');
  AssertTrue('Define not defined',FSCanner.Defines.IndexOf('NEVER')<>-1);
end;

procedure TTestScanner.TestDefine01;
begin
  TestTokens([tkComment],'(*$DEFINE NEVER*)');
  AssertTrue('Define not defined',FSCanner.Defines.IndexOf('NEVER')<>-1);
end;

procedure TTestScanner.TestDefine1;
begin
  TestTokens([tkComment],'{$IFDEF NEVER} of {$ENDIF}');
end;

procedure TTestScanner.TestDefine2;

begin
  FSCanner.Defines.Add('ALWAYS');
  TestTokens([tkComment,tkWhitespace,tkOf,tkWhitespace,tkcomment],'{$IFDEF ALWAYS comment} of {$ENDIF}');
end;

procedure TTestScanner.TestDefine21;
begin
  FSCanner.Defines.Add('ALWAYS');
  TestTokens([tkComment,tkWhitespace,tkOf,tkWhitespace,tkcomment],'(*$IFDEF ALWAYS*) of (*$ENDIF*)');
end;

procedure TTestScanner.TestDefine22;
begin
  FSCanner.Defines.Add('ALWAYS');
  // No whitespace. Test border of *)
  TestTokens([tkComment,tkOf,tkWhitespace,tkcomment],'(*$IFDEF ALWAYS*)of (*$ENDIF*)');
end;

procedure TTestScanner.TestDefine3;
begin
  FSCanner.Defines.Add('ALWAYS');
  TestTokens([tkComment,tkWhitespace,tkOf,tkWhitespace,tkcomment],'{$IFDEF ALWAYS} of {$ELSE} in {$ENDIF}');
end;

procedure TTestScanner.TestDefine4;
begin
  TestTokens([tkComment,tkWhitespace,tkin,tkWhitespace,tkcomment],'{$IFDEF ALWAYS} of {$ELSE} in {$ENDIF}');
end;

procedure TTestScanner.TestDefine5;
begin
  FScanner.SkipComments:=True;
  TestTokens([tkLineEnding],'{$IFDEF NEVER} of {$ENDIF}');
end;

procedure TTestScanner.TestDefine6;

begin
  FSCanner.Defines.Add('ALWAYS');
  FScanner.SkipComments:=True;
  TestTokens([tkWhitespace,tkOf,tkWhitespace],'{$IFDEF ALWAYS} of {$ENDIF}');
end;

procedure TTestScanner.TestDefine7;
begin
  FSCanner.Defines.Add('ALWAYS');
  FScanner.SkipComments:=True;
  TestTokens([tkWhitespace,tkOf,tkWhitespace],'{$IFDEF ALWAYS} of {$ELSE} in {$ENDIF}');
end;

procedure TTestScanner.TestDefine8;
begin
  FScanner.SkipComments:=True;
  TestTokens([tkWhitespace,tkin,tkWhitespace],'{$IFDEF ALWAYS} of {$ELSE} in {$ENDIF}');
end;

procedure TTestScanner.TestDefine9;
begin
  FScanner.SkipWhiteSpace:=True;
  TestTokens([],'{$IFDEF NEVER} of {$ENDIF}');
end;

procedure TTestScanner.TestDefine10;

begin
  FSCanner.Defines.Add('ALWAYS');
  FScanner.SkipComments:=True;
  TestTokens([tkWhitespace,tkOf,tkWhitespace],'{$IFDEF ALWAYS} of {$ENDIF}');
end;

procedure TTestScanner.TestDefine11;
begin
  FSCanner.Defines.Add('ALWAYS');
  FScanner.SkipComments:=True;
  FScanner.SkipWhiteSpace:=True;
  TestTokens([tkOf],'{$IFDEF ALWAYS} of {$ELSE} in {$ENDIF}');
end;

procedure TTestScanner.TestDefine12;
begin
  FScanner.SkipComments:=True;
  FScanner.SkipWhiteSpace:=True;
  TestTokens([tkin],'{$IFDEF ALWAYS} of {$ELSE} in {$ENDIF}');
end;

procedure TTestScanner.TestDefine13;
begin
  FScanner.SkipComments:=True;
  FScanner.SkipWhiteSpace:=True;
  TestTokens([tkin],'{$IFDEF ALWAYS} }; ą è {$ELSE} in {$ENDIF}');
end;

procedure TTestScanner.TestDefine14;
Const
   Source = '{$ifdef NEVER_DEFINED}' +sLineBreak+
            'type'+sLineBreak+
            '  TNPEventModel = ('+sLineBreak+
            '  NPEventModelCarbon = 0,'+sLineBreak+
            '  NPEventModelCocoa = 1'+sLineBreak+
            '}; // yes, this is an error... except this code should never be included.'+sLineBreak+
            'ą'+sLineBreak+
            '|'+sLineBreak+
            '{$endif}'+sLineBreak+
            ''+sLineBreak+
            'begin'+sLineBreak+
            'end.'+sLineBreak;
begin
  NewSource(Source,True);
  While FScanner.fetchToken<>tkEOF do

end;

procedure TTestScanner.TestInclude;
begin
  FResolver.AddStream('myinclude.inc',TStringStream.Create('if true then'));
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  TestTokens([tkIf,tkTrue,tkThen],'{$I myinclude.inc}',True,False);
end;

procedure TTestScanner.TestInclude2;
begin
  FResolver.AddStream('myinclude.inc',TStringStream.Create('if true then'));
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  TestTokens([tkIf,tkTrue,tkThen,tkElse],'{$I myinclude.inc} else',True,False);
end;

procedure TTestScanner.TestInclude3;
begin
  PathPrefix:='src';
  FResolver.AddStream('src/myinclude2.inc',TStringStream.Create(' true '));
  FResolver.AddStream('src/myinclude1.inc',TStringStream.Create('if {$i myinclude2.inc} then '));
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  TestTokens([tkIf,tkTrue,tkThen,tkElse],'{$I src/myinclude1.inc} else',True,False);
end;

procedure TTestScanner.TestIncludeString;
begin
  FResolver.AddStream('myinclude.inc',TStringStream.Create('if true then'));
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  TestTokens([tkString],'{$INCLUDESTRING myinclude.inc}',False,False);
  AssertEquals('Correct string','''if true then''',TestTokenString)
end;

procedure TTestScanner.TestIncludeStringFile;
begin
  FResolver.AddStream('myinclude.inc',TStringStream.Create('if true then'));
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  TestTokens([tkString],'{$INCLUDESTRINGFILE myinclude.inc}',False,False);
  AssertEquals('Correct string','''if true then''',TestTokenString)
end;

procedure TTestScanner.TestIncludeString2Lines;
begin
  FResolver.AddStream('myinclude.inc',TStringStream.Create('if true then'#10'else'));
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  FScanner.MultilineLineFeedStyle:=elCRLF;
  TestTokens([tkString],'{$INCLUDESTRING myinclude.inc}',False,False);
  AssertEquals('Correct string','''if true then'#13#10'else''',TestTokenString)
end;

procedure TTestScanner.TestUnDefine1;
begin
  FSCanner.Defines.Add('ALWAYS');
  TestTokens([tkComment],'{$UNDEF ALWAYS}');
  AssertEquals('No more define',-1,FScanner.Defines.INdexOf('ALWAYS'));
end;

procedure TTestScanner.TestMacro1;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  TestTokens([tkbegin,tkend,tkDot],'{$MACRO on}{$DEFINE MM:=begin end.}'#13#10'MM',True,False);
end;

procedure TTestScanner.TestMacro2;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  TestTokens([tkbegin,tkend,tkDot],'{$MACRO on}{$DEFINE MM:=begin end}'#13#10'MM .',True,False);
end;

procedure TTestScanner.TestMacro3;
begin
  FScanner.SkipComments:=True;
  FScanner.SkipWhiteSpace:=True;
  TestTokens([tkof],'{$MACRO on}{$DEFINE MM:=begin end}'#13#10'{$IFDEF MM} of {$ELSE} in {$ENDIF}');
end;

procedure TTestScanner.TestMacroHandling;
begin
  TTestingPascalScanner(FScanner).DoSpecial:=True;
  FScanner.SkipComments:=True;
  FScanner.SkipWhiteSpace:=True;
  TestTokens([tkIdentifier],'{$MACRO on}{$DEFINE MM:=begin end}'#13#10'MM');
  AssertEQuals('Correct identifier', 'somethingweird',LastIdentifier);
end;

procedure TTestScanner.TestIFDefined;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  TestTokens([tkbegin,tkend,tkDot],'{$DEFINE A}{$IF defined(A)}begin{$ENDIF}end.',True,False);
end;

procedure TTestScanner.TestIFUnDefined;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  TestTokens([tkbegin,tkend,tkDot],'{$IF undefined(A)}begin{$ENDIF}end.',True,False);
end;

procedure TTestScanner.TestIFAnd;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  TestTokens([tkbegin,tkend,tkDot],
    '{$DEFINE A}{$IF defined(A) and undefined(B)}begin{$ENDIF}end.',True,False);
end;

procedure TTestScanner.TestIFAndShortEval;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  TestTokens([tkbegin,tkend,tkDot],
    '{$UNDEFINE A}{$IF defined(A) and undefined(B)}wrong{$ELSE}begin{$ENDIF}end.',
    True,False);
end;

procedure TTestScanner.TestIFOr;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  TestTokens([tkbegin,tkend,tkDot],
    '{$DEFINE B}{$IF defined(A) or defined(B)}begin{$ENDIF}end.',True,False);
end;

procedure TTestScanner.TestIFOrShortEval;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  TestTokens([tkbegin,tkend,tkDot],
    '{$DEFINE A}{$IF defined(A) or defined(B)}begin{$ENDIF}end.',True,False);
end;

procedure TTestScanner.TestIFXor;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  TestTokens([tkbegin,tkend,tkDot],
    '{$DEFINE B}{$IF defined(A) xor defined(B)}begin{$ENDIF}end.',True,False);
end;

procedure TTestScanner.TestIFAndOr;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  TestTokens([tkbegin,tkend,tkDot],
     '{$IF   defined(A) and   defined(B) or   defined(C)}wrong1{$ENDIF}'+LineEnding
    +'{$IF   defined(A) and   defined(B) or undefined(C)}{$ELSE}wrong2{$ENDIF}'+LineEnding
    +'{$IF   defined(A) and undefined(B) or   defined(C)}wrong3{$ENDIF}'+LineEnding
    +'{$IF   defined(A) and undefined(B) or undefined(C)}{$ELSE}wrong4{$ENDIF}'+LineEnding
    +'{$IF undefined(A) and   defined(B) or   defined(C)}wrong5{$ENDIF}'+LineEnding
    +'{$IF undefined(A) and   defined(B) or undefined(C)}{$ELSE}wrong6{$ENDIF}'+LineEnding
    +'{$IF undefined(A) and undefined(B) or   defined(C)}{$ELSE}wrong7{$ENDIF}'+LineEnding
    +'{$IF undefined(A) and undefined(B) or undefined(C)}begin{$ENDIF}end.',
    True,False);
end;

procedure TTestScanner.TestIFEqual;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  FScanner.AddMacro('Version','30101');
  TestTokens([tkbegin,tkend,tkDot],
    '{$IF Version=30101}begin{$ENDIF}end.',True,False);
end;

procedure TTestScanner.TestIFNotEqual;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  FScanner.AddMacro('Version','30101');
  TestTokens([tkbegin,tkend,tkDot],
    '{$IF Version<>30000}begin{$ENDIF}end.',True,False);
end;

procedure TTestScanner.TestIFGreaterThan;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  FScanner.AddMacro('Version','30101');
  TestTokens([tkbegin,tkend,tkDot],
    '{$IF Version>30000}begin{$ENDIF}end.',True,False);
end;

procedure TTestScanner.TestIFGreaterEqualThan;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  FScanner.AddMacro('Version','30101');
  TestTokens([tkbegin,tkend,tkDot],
    '{$IF Version>=30000}begin{$ENDIF}end.',True,False);
end;

procedure TTestScanner.TestIFLesserThan;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  FScanner.AddMacro('Version','30101');
  TestTokens([tkbegin,tkend,tkDot],
    '{$IF Version<40000}begin{$ENDIF}end.',True,False);
end;

procedure TTestScanner.TestIFLesserEqualThan;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  FScanner.AddMacro('Version','30101');
  TestTokens([tkbegin,tkend,tkDot],
    '{$IF Version<=30101}begin{$ENDIF}end.',True,False);
end;

procedure TTestScanner.TestIFDefinedElseIf;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  FScanner.AddDefine('cpu32');
  TestTokens([tkconst,tkIdentifier,tkEqual,tkString,tkSemicolon,tkbegin,tkend,tkDot],
    'const platform = '+LineEnding
    +'{$if defined(cpu32)} ''x86'''+LineEnding
    +'{$elseif defined(cpu64)} ''x64'''+LineEnding
    +'{$else} {$error unknown platform} {$endif};'+LineEnding
    +'begin end.',True,False);
end;

procedure TTestScanner.TestIfError;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  TestTokens([tkprogram,tkIdentifier,tkSemicolon,tkbegin,tkend,tkDot],
    'program Project1;'+LineEnding
    +'begin'+LineEnding
    +'{$if sizeof(integer) <> 4} {$error wrong sizeof(integer)} {$endif}'+LineEnding
    +'end.',True,False);
end;

procedure TTestScanner.TestIFCDefined;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  FScanner.AddDefine('cpu32');
  TestTokens([tkconst,tkIdentifier,tkEqual,tkString,tkSemicolon,tkbegin,tkend,tkDot],
    'const platform = '+LineEnding
    +'{$ifc defined cpu32} ''x86'''+LineEnding
    +'{$elseif defined(cpu64)} 1 '+LineEnding
    +'{$else} {$error unknown platform} {$endc};'+LineEnding
    +'begin end.',True,False);
end;

procedure TTestScanner.TestIFCNotDefined;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  FScanner.AddDefine('cpu32');
  TestTokens([tkconst,tkIdentifier,tkEqual,tkNumber,tkSemicolon,tkbegin,tkend,tkDot],
    'const platform = '+LineEnding
    +'{$ifc not defined cpu32} ''x86'''+LineEnding
    +'{$else} 1 '+LineEnding
    +'{$endc};'+LineEnding
    +'begin end.',True,False);
end;

procedure TTestScanner.TestIFCAndDefined;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  FScanner.AddDefine('cpu32');
  FScanner.AddDefine('alpha');
  TestTokens([tkconst,tkIdentifier,tkEqual,tkstring,tkSemicolon,tkbegin,tkend,tkDot],
    'const platform = '+LineEnding
    +'{$ifc defined cpu32 and defined alpha} ''x86'''+LineEnding
    +'{$else} 1 '+LineEnding
    +'{$endc};'+LineEnding
    +'begin end.',True,False);
end;

procedure TTestScanner.TestIFCFalse;
begin
  FScanner.SkipWhiteSpace:=True;
  FScanner.SkipComments:=True;
  FScanner.AddDefine('cpu32');
  FScanner.AddDefine('alpha');
  FScanner.AddMacro('MY','FALSE');
  TestTokens([tkconst,tkIdentifier,tkEqual,tkNumber,tkSemicolon,tkbegin,tkend,tkDot],
    'const platform = '+LineEnding
    +'{$IFC MY} ''x86'''+LineEnding
    +'{$else} 1 '+LineEnding
    +'{$endc};'+LineEnding
    +'begin end.',True,False);
end;

procedure TTestScanner.TestModeSwitch;

Const
   PlusMinus = [' ','+','-'];

Var
  M : TModeSwitch;
  C : Char;
begin
  For M in TModeSwitch do
    for C in PlusMinus do
      if SModeSwitchNames[M]<>'' then
        begin
        Scanner.CurrentModeSwitches:=[];
        NewSource('{$MODESWITCH '+SModeSwitchNames[M]+C+'}');
        While not (Scanner.FetchToken=tkEOF) do;
        if C in [' ','+'] then
          AssertTrue(SModeSwitchNames[M]+C+' sets '+GetEnumName(TypeInfo(TModeSwitch),Ord(M)),M in Scanner.CurrentModeSwitches)
        else
          AssertFalse(SModeSwitchNames[M]+C+' removes '+GetEnumName(TypeInfo(TModeSwitch),Ord(M)),M in Scanner.CurrentModeSwitches);
        end;
end;

procedure TTestScanner.TestOperatorIdentifier;
begin
  Scanner.SetNonToken(tkoperator);
  TestToken(tkidentifier,'operator',True);
end;

procedure TTestScanner.TestUTF8BOM;

begin
  DoTestToken(tkLineEnding,#$EF+#$BB+#$BF);
end;

procedure TTestScanner.TestBooleanSwitch;

begin
  Scanner.CurrentBoolSwitches:=[bsHints];
  // end space intentional.
  NewSource('{$HINTS OFF }');
  While not (Scanner.FetchToken=tkEOF) do;
  AssertFalse('Hints off',bshints in Scanner.CurrentBoolSwitches);
end;

initialization
  RegisterTests([TTestTokenFinder,TTestStreamLineReader,TTestScanner]);
end.

