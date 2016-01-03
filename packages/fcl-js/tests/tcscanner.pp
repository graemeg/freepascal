unit tcscanner;

{$mode objfpc}{$H+}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Classes, SysUtils, Typinfo, fpcunit, testutils, testregistry, jstoken, jsscanner;
=======
  Classes, SysUtils, Typinfo, fpcunit, testutils, testregistry, jsscanner;
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, Typinfo, fpcunit, testutils, testregistry, jsscanner;
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, Typinfo, fpcunit, testutils, testregistry, jsscanner;
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, Typinfo, fpcunit, testutils, testregistry, jsscanner;
>>>>>>> origin/cpstrnew

type

  { TTestLineReader }

  TTestLineReader = Class(TTestCase)
  Private
    FData: TStringStream;
    FReader : TStreamLineReader;
  protected
    Procedure CreateReader(AInput : String);
    procedure TearDown; override;
  published
    Procedure TestEmpty;
    Procedure TestReadLine;
    Procedure TestReadLines13;
    Procedure TestReadLines10;
    Procedure TestReadLines1310;
    procedure TestReadLinesEOF13;
    procedure TestReadLinesEOF10;
    procedure TestReadLinesEOF1310;
    procedure TestReadEmptyLines101010;
  end;

  { TTestJSScanner }

  TTestJSScanner = class(TTestCase)
  Private
    FStream : TStream;
    FLineReader : TLineReader;
    FScanner : TJSScanner;
    FErrorSource : String;
    procedure AssertEquals(AMessage: String; AExpected, AActual : TJSToken); overload;
    procedure CheckToken(AToken: TJSToken; ASource: String);
    procedure CheckTokens(ASource: String; ATokens: array of TJSToken);
    procedure DoTestFloat(F: Double);
    procedure DoTestFloat(F: Double; S: String);
    procedure DoTestString(S: String);
    procedure TestErrorSource;
  protected
    Function CreateScanner(AInput : String) : TJSScanner;
    procedure FreeScanner;
    procedure SetUp; override;
    procedure TearDown; override;

    Property Scanner : TJSScanner Read FScanner;
  published
    Procedure TestEmpty;
    procedure TestAndAnd;
    procedure TestAndEq;
    procedure TestAssign;
    procedure TestBraceClose;
    procedure TestBraceOpen;
    procedure TestColon;
    procedure TestComma;
    procedure TestCurlyBraceClose;
    procedure TestCurlyBraceOpen;
    procedure TestDiv;
    procedure TestDiveq;
    procedure TestXor;
    procedure TestXoreq;
    procedure TestDot;
    procedure TestEq;
    procedure TestGE;
    procedure TestFalse;
    procedure TestInv;
    procedure TestNot;
    procedure TestString;
    procedure TestTrue;
    procedure TestGreaterThan;
    procedure TestLE;
    procedure TestLessThan;
    procedure TestLSHIFT;
    procedure TestLSHIFTEQ;
    procedure TestMinus;
    procedure TestMinusEQ;
    procedure TestMinusMinus;
    procedure TestModeq;
    procedure TestMul;
    procedure TestNE;
    procedure TestNSE;
    procedure TestOREQ;
    procedure TestOROR;
    procedure TestPlus;
    procedure TestPlusEq;
    procedure TestPlusPlus;
    procedure TestRShift;
    procedure TestRShiftEq;
    procedure TestSemicolon;
    procedure TestSEq;
    procedure TestSquaredBraceClose;
    procedure TestSquaredBraceOpen;
    procedure TestStarEq;
    procedure TestURShift;
    procedure TestURShiftEq;
    procedure TestBreak;
    procedure TestCase;
    procedure TestCatch;
    procedure TestContinue;
    procedure TestDefault;
    procedure TestDelete;
    procedure TestDO;
    procedure TestElse;
    procedure TestFinally;
    procedure TestFor;
    procedure TestFunction;
    procedure TestIf;
    procedure TestIn;
    procedure TestInstanceOf;
    procedure TestNew;
    procedure TestReturn;
    procedure TestSwitch;
    procedure TestThis;
    procedure TestThrow;
    procedure TestTry;
    procedure TestTypeOf;
    procedure TestVar;
    procedure TestVoid;
    procedure TestWhile;
    procedure TestWith;
    Procedure Test2Words;
    procedure Test3Words;
    procedure TestIdentifier;
    procedure TestIdentifier2;
    procedure TestIdentifier3;
    procedure TestIdentifier4;
    procedure TestIdentifier5;
    procedure TestIdentifierDotIdentifier;
    procedure TestEOLN;
    procedure TestEOLN2;
    procedure TestEOLN3;
    procedure TestEOLN4;
    procedure TestComment1;
    procedure TestComment2;
    procedure TestComment3;
    procedure TestComment4;
    procedure TestComment5;
    procedure TestComment6;
    procedure TestFloat;
    procedure TestStringError;
    procedure TestFloatError;
  end;


implementation

Function TTestJSScanner.CreateScanner(AInput : String) : TJSScanner;

begin
  FStream:=TStringStream.Create(AInput);
  FLineReader:=TStreamLineReader.Create(Fstream);
  FScanner:=TJSScanner.Create(FLineReader);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=FScanner;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

procedure TTestJSScanner.FreeScanner;
begin
  FreeAndNil(FScanner);
  FreeAndNil(FLineReader);
  FreeAndNil(FStream);
end;

procedure TTestJSScanner.SetUp;
begin
  inherited SetUp;
end;



procedure TTestJSScanner.TestEmpty;

Var
  J : TJSToken;

begin
  CreateScanner('');
  J:=Scanner.FetchToken;
  If (J<>tjsEOF) then
    Fail('Empty returns EOF');
end;

procedure TTestJSScanner.AssertEquals(AMessage : String; AExpected, AActual: TJSToken);

Var
  J : TJSToken;
  S,EN1,EN2 : String;

begin
  If (AActual<>AExpected) then
    begin
    EN1:=GetEnumName(TypeINfo(TJSToken),Ord(AExpected));
    EN2:=GetEnumName(TypeINfo(TJSToken),Ord(AActual));
    S:=Format('%s : %s <> %s',[AMessage,EN1,EN2]);
    Fail(S);
    end;
end;

procedure TTestJSScanner.CheckToken(AToken : TJSToken; ASource : String);

Var
  J : TJSToken;
  EN2 : String;

begin
  CreateScanner(ASource);
  J:=Scanner.FetchToken;
  EN2:=GetEnumName(TypeINfo(TJSToken),Ord(AToken));
  AssertEquals(Format('Source %s should result in %s.',[ASource,EN2]),AToken,J);
end;


procedure TTestJSScanner.TestAndAnd;

begin
  CheckToken(tjsAndAnd,'&&');
end;

procedure TTestJSScanner.TestAndEq;

begin
  CheckToken(tjsAndEq,'&=');
end;

procedure TTestJSScanner.TestBraceOpen;

begin
  CheckToken(tjsBraceOpen,'(');
end;

procedure TTestJSScanner.TestBraceClose;

begin
  CheckToken(tjsBraceClose,')');
end;

procedure TTestJSScanner.TestSquaredBraceClose;

begin
  CheckToken(tjsSquaredBraceClose,']');
end;

procedure TTestJSScanner.TestSquaredBraceOpen;

begin
  CheckToken(tjssQuaredBraceOpen,'[');
end;

procedure TTestJSScanner.TestCurlyBraceOpen;

begin
  CheckToken(tjsCurlyBraceOpen,'{');
end;

procedure TTestJSScanner.TestCurlyBraceClose;

begin
  CheckToken(tjsCurlyBraceClose,'}');
end;

procedure TTestJSScanner.TestComma;

begin
  CheckToken(tjsComma,',');
end;

procedure TTestJSScanner.TestColon;

begin
  CheckToken(tjsColon,':');
end;

procedure TTestJSScanner.TestDot;

begin
  CheckToken(tjsDot,'.');
end;

procedure TTestJSScanner.TestSemicolon;

begin
  CheckToken(tjsSemicolon,';');
end;

procedure TTestJSScanner.TestAssign;

begin
  CheckToken(tjsAssign,'=');
end;

procedure TTestJSScanner.TestGreaterThan;

begin
  CheckToken(tjsGT,'>');
end;

procedure TTestJSScanner.TestLessThan;

begin
  CheckToken(tjsLT,'<');
end;

procedure TTestJSScanner.TestPlus;

begin
  CheckToken(tjsPlus,'+');
end;

procedure TTestJSScanner.TestMinus;

begin
  CheckToken(tjsMinus,'-');
end;

procedure TTestJSScanner.TestMul;

begin
  CheckToken(tjsMul,'*');
end;

procedure TTestJSScanner.TestDiv;

begin
  CheckToken(tjsDiv,'/');
end;

procedure TTestJSScanner.TestEq;

begin
  CheckToken(tjsEq,'==');
end;

procedure TTestJSScanner.TestGE;

begin
  CheckToken(tjsGE,'>=');
end;

procedure TTestJSScanner.TestLE;

begin
  CheckToken(tjsLE,'<=');
end;

procedure TTestJSScanner.TestLSHIFT;

begin
  CheckToken(tjsLShift,'<<');
end;

procedure TTestJSScanner.TestLSHIFTEQ;

begin
  CheckToken(tjsLShiftEq,'<<=');
end;

procedure TTestJSScanner.TestMinusEQ;

begin
  CheckToken(tjsMinusEq,'-=');
end;

procedure TTestJSScanner.TestMinusMinus;

begin
  CheckToken(tjsMinusMinus,'--');
end;

procedure TTestJSScanner.TestModeq;

begin
  CheckToken(tjsModeq,'%=');
end;


procedure TTestJSScanner.TestDiveq;

begin
  CheckToken(tjsDiveq,'/=');
end;

procedure TTestJSScanner.TestXor;
begin
  CheckToken(tjsXOR,'^');
end;

procedure TTestJSScanner.TestXoreq;
begin
  CheckToken(tjsXOREQ,'^=');
end;

procedure TTestJSScanner.TestNE;

begin
  CheckToken(tjsNE,'!=');
end;

procedure TTestJSScanner.TestInv;

begin
  CheckToken(tjsInv,'~');
end;

procedure TTestJSScanner.TestNot;

begin
  CheckToken(tjsNot,'!');
end;

procedure TTestJSScanner.TestTrue;

begin
  CheckToken(tjsTrue,'true');
end;

procedure TTestJSScanner.TestFalse;

begin
  CheckToken(tjsFalse,'false');
end;

procedure TTestJSScanner.TestOREQ;

begin
  CheckToken(tjsOREQ,'|=');
end;

procedure TTestJSScanner.TestOROR;

begin
  CheckToken(tjsOROR,'||');
end;

procedure TTestJSScanner.TestPlusEq;

begin
  CheckToken(tjsPlusEq,'+=');
end;

procedure TTestJSScanner.TestPlusPlus;

begin
  CheckToken(tjsPlusPlus,'++');
end;

procedure TTestJSScanner.TestURShift;

begin
  CheckToken(tjsURSHIFT,'>>>');
end;

procedure TTestJSScanner.TestURShiftEq;

begin
  CheckToken(tjsURSHIFTEQ,'>>>=');
end;

procedure TTestJSScanner.TestRShift;

begin
  CheckToken(tjsRSHIFT,'>>');
end;

procedure TTestJSScanner.TestRShiftEq;

begin
  CheckToken(tjsRSHIFTEQ,'>>=');
end;

procedure TTestJSScanner.TestSEq;

begin
  CheckToken(tjsSEQ,'===');
end;

procedure TTestJSScanner.TestNSE;

begin
  CheckToken(tjsSNE,'!==');
end;

procedure TTestJSScanner.TestStarEq;

begin
  CheckToken(tjsMulEq,'*=');
end;

procedure TTestJSScanner.TestBreak;

begin
  CheckToken(tjsBreak,'break');
end;

procedure TTestJSScanner.TestCase;

begin
  CheckToken(tjscase,'case');
end;

procedure TTestJSScanner.TestCatch;

begin
  CheckToken(tjscatch,'catch');
end;

procedure TTestJSScanner.TestContinue;

begin
  CheckToken(tjscontinue,'continue');
end;

procedure TTestJSScanner.TestDefault;

begin
  CheckToken(tjsdefault,'default');
end;

procedure TTestJSScanner.TestDelete;

begin
  CheckToken(tjsdelete,'delete');
end;

procedure TTestJSScanner.TestDO;

begin
  CheckToken(tjsdo,'do');
end;

procedure TTestJSScanner.TestElse;

begin
  CheckToken(tjselse,'else');
end;

procedure TTestJSScanner.TestFinally;

begin
  CheckToken(tjsfinally,'finally');
end;

procedure TTestJSScanner.TestFor;

begin
  CheckToken(tjsfor,'for');
end;

procedure TTestJSScanner.TestFunction;

begin
  CheckToken(tjsfunction,'function');
end;

procedure TTestJSScanner.TestIf;

begin
  CheckToken(tjsif,'if');
end;

procedure TTestJSScanner.TestIn;

begin
  CheckToken(tjsin,'in');
end;

procedure TTestJSScanner.TestInstanceOf;

begin
  CheckToken(tjsinstanceof,'instanceof');
end;

procedure TTestJSScanner.TestNew;

begin
  CheckToken(tjsnew,'new');
end;

procedure TTestJSScanner.TestReturn;

begin
  CheckToken(tjsreturn,'return');
end;

procedure TTestJSScanner.TestSwitch;

begin
  CheckToken(tjsswitch,'switch');
end;

procedure TTestJSScanner.TestThis;

begin
  CheckToken(tjsThis,'this');
end;

procedure TTestJSScanner.TestThrow;

begin
  CheckToken(tjsThrow,'throw');
end;

procedure TTestJSScanner.TestTry;

begin
  CheckToken(tjsTry,'try');
end;

procedure TTestJSScanner.TestTypeOf;

begin
  CheckToken(tjstypeof,'typeof');
end;

procedure TTestJSScanner.TestVar;

begin
  CheckToken(tjsvar,'var');
end;

procedure TTestJSScanner.TestVoid;

begin
  CheckToken(tjsvoid,'void');
end;

procedure TTestJSScanner.TestWhile;

begin
  CheckToken(tjswhile,'while');
end;

procedure TTestJSScanner.TestWith;

begin
  CheckToken(tjswith,'with');
end;

procedure TTestJSScanner.CheckTokens(ASource : String; ATokens : Array of TJSToken);

Var
  I : Integer;
  J : TJSToken;
  S : String;

begin
  CreateScanner(ASource);
  For I:=Low(ATokens) to High(ATokens) do
    begin
    J:=FScanner.FetchToken;
    S:=GetEnumName(TypeINfo(TJSToken),Ord(ATokens[i]));
    S:=Format('Source "%s", token %d (%s): expected %s',[ASource,I,FScanner.CurTokenString,S]);
    AssertEquals(S,ATokens[i],J);
    end;
end;

procedure TTestJSScanner.Test2Words;
begin
  CheckTokens('with do',[tjsWith,tjsDo]);
end;

procedure TTestJSScanner.Test3Words;
begin
  CheckTokens('with do for',[tjsWith,tjsDo,tjsFor]);
end;

procedure TTestJSScanner.TestIdentifier;
begin
  CheckToken(tjsIdentifier,'something');
  AssertEquals('Correct identifier','something',FScanner.CurTokenString);
end;

procedure TTestJSScanner.TestIdentifier2;
begin
  CheckToken(tjsIdentifier,'_something');
  AssertEquals('Correct identifier','_something',FScanner.CurTokenString);
end;

procedure TTestJSScanner.TestIdentifier3;
begin
  CheckToken(tjsIdentifier,'$');
  AssertEquals('Correct identifier','$',FScanner.CurTokenString);
end;

procedure TTestJSScanner.TestIdentifier4;
begin
  CheckToken(tjsIdentifier,'_0');
  AssertEquals('Correct identifier','_0',FScanner.CurTokenString);
end;

procedure TTestJSScanner.TestIdentifier5;
begin
  CheckToken(tjsIdentifier,'$0');
  AssertEquals('Correct identifier','$0',FScanner.CurTokenString);
end;

procedure TTestJSScanner.TestIdentifierDotIdentifier;
begin
  CheckTokens('something.different',[tjsIdentifier,tjsdot,tjsIdentifier]);
//  AssertEquals('Correct identifier','something',FScanner.CurTokenString);
end;

procedure TTestJSScanner.TestEOLN;
begin
  CreateScanner('something');
  FScanner.FetchToken;
  AssertEquals('Got to end of line after reading single token at EOF',True,FScanner.IsEndOfLine);
//  AssertEquals('Correct identifier','something',FScanner.CurTokenString);
end;

procedure TTestJSScanner.TestEOLN2;
begin
  CreateScanner('something different');
  FScanner.FetchToken;
  AssertEquals('Not yet end of line after reading single token at EOF',False,FScanner.IsEndOfLine);
end;

procedure TTestJSScanner.TestEOLN3;
begin
  CreateScanner('something'#13#10'different');
  FScanner.FetchToken;
  AssertEquals('End of line after reading single token',True,FScanner.IsEndOfLine);
end;

procedure TTestJSScanner.TestEOLN4;
begin
  CreateScanner('something'#10'different');
  FScanner.FetchToken;
  AssertEquals('End of line after reading first token',True,FScanner.IsEndOfLine);
  FScanner.FetchToken;
  AssertEquals('End of line after reading second token',True,FScanner.IsEndOfLine);
end;

procedure TTestJSScanner.TestComment1;
begin
  CreateScanner('// some comment string');
  AssertEquals('Comment line is skipped',tjsEOF,FScanner.FetchToken);
end;

procedure TTestJSScanner.TestComment2;
begin
  CreateScanner('// some comment string');
  FScanner.ReturnComments:=True;
  AssertEquals('Comment line is returned',tjsComment,FScanner.FetchToken);
  AssertEquals('Comment contents is returned',' some comment string',FScanner.CurTokenString);
end;

procedure TTestJSScanner.TestComment3;
begin
  CreateScanner('/* some comment string */');
  AssertEquals('Comment line is skipped',tjsEOF,FScanner.FetchToken);
end;

procedure TTestJSScanner.TestComment4;
begin
  CreateScanner('/* some comment string */');
  FScanner.ReturnComments:=True;
  AssertEquals('Comment line is returned',tjsComment,FScanner.FetchToken);
  AssertEquals('Comment contents is returned',' some comment string ',FScanner.CurTokenString);
end;

procedure TTestJSScanner.TestComment5;
begin
  CreateScanner('/* some nested comment // string */');
  FScanner.ReturnComments:=True;
  AssertEquals('Comment line is returned',tjsComment,FScanner.FetchToken);
  AssertEquals('Comment contents is returned',' some nested comment // string ',FScanner.CurTokenString);
end;

procedure TTestJSScanner.TestComment6;
begin
  CreateScanner('// /* some nested comment string */');
  FScanner.ReturnComments:=True;
  AssertEquals('Comment line is returned',tjsComment,FScanner.FetchToken);
  AssertEquals('Comment contents is returned',' /* some nested comment string */',FScanner.CurTokenString);
end;

procedure TTestJSScanner.TearDown; 
begin
  FreeScanner;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Inherited;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

procedure TTestJSScanner.DoTestFloat(F : Double);

Var
  S : String;

begin
  Str(F,S);
  DoTestFloat(F,S);
end;

procedure TTestJSScanner.DoTestFloat(F : Double; S : String);

Var
  J : TJSToken;
  C : Double;
  I : integer;
  V : String;

begin
  CreateScanner(S);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  try
    J:=FScanner.FetchToken;
    AssertEquals(S+' is a number',tjsNumber,J);
    V:=FScanner.CurTokenString;
    If (Copy(V,1,2)='0x') then
      begin
      Flush(output);
      V:='$'+Copy(V,3,Length(V)-2);
      C:=StrToInt(V);
      end
    else
      begin
      Val(V,C,I);
      If (I<>0) then
        Fail(FScanner.CurTokenString+' does not contain a float value');
      end;
    AssertEquals('Parsed float equals original float',F,C);
  finally
    FreeScanner;
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  J:=FScanner.FetchToken;
  AssertEquals(S+' is a number',tjsNumber,J);
  V:=FScanner.CurTokenString;
  If (Copy(V,1,2)='0x') then
    begin
    Flush(output);
    V:='$'+Copy(V,3,Length(V)-2);
    C:=StrToInt(V);
    end
  else
    begin
    Val(V,C,I);
    If (I<>0) then
      Fail(FScanner.CurTokenString+' does not contain a float value');
    end;
  AssertEquals('Parsed float equals original float',F,C);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

procedure TTestJSScanner.TestFloat;


begin
  DoTestFloat(1.2);
  DoTestFloat(-1.2);
  DoTestFloat(0);
  DoTestFloat(1.2e1);
  DoTestFloat(-1.2e1);
  DoTestFloat(0);
  DoTestFloat(1.2,'1.2');
  DoTestFloat(-1.2,'-1.2');
  DoTestFloat(0,'0.0');
  DoTestFloat(255,'0xff')
end;

procedure TTestJSScanner.TestFloatError;

begin
  FErrorSource:='1xz';
  AssertException('Wrong float',EJSScannerError,@TestErrorSource);
end;


procedure TTestJSScanner.DoTestString(S: String);

Var
  J : TJSToken;
  T : String;
begin
  CreateScanner(S);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  try
    J:=FScanner.FetchToken;
    AssertEquals(S+' is a string',tjsString,J);
    If (Length(S)>0) and (S[1] in ['"','''']) then
      S:=Copy(S,2,Length(S)-2);
    AssertEquals('Correct string is returned',S,FScanner.CurTokenString);
  finally
    FreeScanner;
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  J:=FScanner.FetchToken;
  AssertEquals(S+' is a string',tjsString,J);
  If (Length(S)>0) and (S[1] in ['"','''']) then
    S:=Copy(S,2,Length(S)-2);
  AssertEquals('Correct string is returned',S,FScanner.CurTokenString);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

procedure TTestJSScanner.TestString;

begin
  DoTestString('"A string"');
  DoTestString('""');
  DoTestString('''''');
  DoTestString('''A string''');
end;

procedure TTestJSScanner.TestErrorSource;

begin
  CreateScanner(FErrorSource);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  try
    While (FScanner.FetchToken<>tjsEOF) do ;
  finally
    FreeScanner;
  end;
=======
  While (FScanner.FetchToken<>tjsEOF) do ;
>>>>>>> graemeg/cpstrnew
=======
  While (FScanner.FetchToken<>tjsEOF) do ;
>>>>>>> graemeg/cpstrnew
=======
  While (FScanner.FetchToken<>tjsEOF) do ;
>>>>>>> graemeg/cpstrnew
=======
  While (FScanner.FetchToken<>tjsEOF) do ;
>>>>>>> origin/cpstrnew
end;

procedure TTestJSScanner.TestStringError;

begin
  FErrorSource:='"A string';
  AssertException('Unterminated string',EJSScannerError,@TestErrorSource);
  FErrorSource:='''A string';
  AssertException('Unterminated string',EJSScannerError,@TestErrorSource);
end;


{ TTestLineReader }

procedure TTestLineReader.CreateReader(AInput: String);
begin
  FData:=TStringStream.Create(AInput);
  FReader:=TStreamLineReader.Create(FData);
end;


procedure TTestLineReader.TearDown;
begin
  FreeAndNil(FReader);
  FreeAndNil(FData);
end;

procedure TTestLineReader.TestEmpty;
begin
  CreateReader('');
  AssertEquals('Empty reader returns EOF',True,FReader.IsEOF);
  AssertEquals('Empty reader returns empty string','',FReader.ReadLine);
end;

procedure TTestLineReader.TestReadLine;
begin
  CreateReader('Something');
  AssertEquals('Reader with 1 line returns 1 line','Something',FReader.ReadLine);
  AssertEquals('EOF true after reading line',True,FReader.IsEOF);
end;

procedure TTestLineReader.TestReadLines13;
begin
  CreateReader('Something'#13'else');
  AssertEquals('Reader with 2 lines returns 1st line','Something',FReader.ReadLine);
  AssertEquals('Reader with 2 lines returns 2nd line','else',FReader.ReadLine);
  AssertEquals('EOF true after reading lines',True,FReader.IsEOF);
end;

procedure TTestLineReader.TestReadLines10;
begin
  CreateReader('Something'#10'else');
  AssertEquals('Reader with 2 lines returns 1st line','Something',FReader.ReadLine);
  AssertEquals('Reader with 2 lines returns 2nd line','else',FReader.ReadLine);
  AssertEquals('EOF true after reading lines',True,FReader.IsEOF);
end;

procedure TTestLineReader.TestReadLines1310;
begin
  CreateReader('Something'#13#10'else');
  AssertEquals('Reader with 2 lines returns 1st line','Something',FReader.ReadLine);
  AssertEquals('Reader with 2 lines returns 2nd line','else',FReader.ReadLine);
  AssertEquals('EOF true after reading lines',True,FReader.IsEOF);
end;

procedure TTestLineReader.TestReadLinesEOF13;
begin
  CreateReader('Something'#13);
  AssertEquals('Reader with 2 lines + CR returns 1st line','Something',FReader.ReadLine);
  AssertEquals('Reader with 1 lines + CR returns empty 2nd line','',FReader.ReadLine);
  AssertEquals('EOF true after reading lines',True,FReader.IsEOF);
end;

procedure TTestLineReader.TestReadLinesEOF10;
begin
  CreateReader('Something'#10);
  AssertEquals('Reader with 2 lines + LF returns 1st line','Something',FReader.ReadLine);
  AssertEquals('Reader with 1 lines + LF returns empty 2nd line','',FReader.ReadLine);
  AssertEquals('EOF true after reading lines',True,FReader.IsEOF);
end;

procedure TTestLineReader.TestReadLinesEOF1310;
begin
  CreateReader('Something'#13#10);
  AssertEquals('Reader with 2 lines + CRLF returns 1st line','Something',FReader.ReadLine);
  AssertEquals('Reader with 1 lines + CRLF returns empty 2nd line','',FReader.ReadLine);
  AssertEquals('EOF true after reading lines',True,FReader.IsEOF);
end;

procedure TTestLineReader.TestReadEmptyLines101010;

begin
  CreateReader('Something'#10#10#10);
  AssertEquals('Reader with 1 line + LFLFLF returns 1st line','Something',FReader.ReadLine);
  AssertEquals('EOF false after reading line 1',False,FReader.IsEOF);
  AssertEquals('Reader with 1 line + LFLFLF returns empty 2nd line','',FReader.ReadLine);
  AssertEquals('EOF false after reading line 2',False,FReader.IsEOF);
  AssertEquals('Reader with 1 line + LFLFLF returns empty 3nd line','',FReader.ReadLine);
  AssertEquals('EOF true after reading lines',True,FReader.IsEOF);
end;

initialization

  RegisterTests([TTestLineReader,TTestJSScanner]);
end.

