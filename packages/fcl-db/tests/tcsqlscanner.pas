{
    This file is part of the Free Component Library
    Copyright (c) 2010-2014 by the Free Pascal development team

    SQL source lexical scanner test suite

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit tcsqlscanner;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testregistry, fpsqlscanner;

type

  { TTestSQLScanner }

  TTestSQLScanner= class(TTestCase)
  Private
    FStream : TStream;
    FLineReader : TLineReader;
    FScanner : TSQLScanner;
    FErrorSource : String;
    FErrorOptions : TSQLScannerOptions;
    procedure AssertEquals(AMessage: String; AExpected, AActual : TSQLToken); overload;
    procedure CheckToken(AToken: TSQLToken; ASource: String);
    procedure CheckTokens(ASource: String; ATokens: array of TSQLToken);
    procedure DoTestFloat(F: Double);
    procedure DoTestFloat(F: Double; S: String);
    procedure DoTestString(S: String; DoubleDelim : Boolean = False);
    procedure TestErrorSource;
  protected
    Function CreateScanner(AInput : String; AOptions : TSQLScannerOptions = []) : TSQLScanner;
    procedure FreeScanner;
    procedure SetUp; override;
    procedure TearDown; override;
    Property Scanner : TSQLScanner Read FScanner;
  published
    procedure TestAction;
    procedure TestAdd;
    procedure TestAdmin;
    procedure TestAfter;
    procedure TestALTER;
    procedure TestAnd;
    procedure TestAny;
    procedure TestAs;
    procedure TestASC;
    procedure TestASCENDING;
    Procedure TestAt;
    procedure TestAuto;
    procedure TestAVG;
    procedure TestAssign;
    procedure TestBefore;
    Procedure TestBegin;
    procedure TestBetween;
    procedure TestBlob;
    procedure TestBraceClose;
    procedure TestBraceOpen;
    procedure TestBy;
    Procedure TestCache;
    procedure TestCascade;
    procedure TestChar;
    procedure TestCharacter;
    procedure TestCheck;
    procedure TestCollate;
    procedure TestColon;
    procedure TestColumn;
    procedure TestComma;
    procedure TestCommit;
    procedure TestComputed;
    procedure TestConcatenate;
    Procedure TestConditional;
    Procedure TestConnect;
    procedure TestConstraint;
    procedure TestContaining;
    procedure TestCount;
    procedure TestCreate;
    procedure TestCString;
    Procedure TestDatabase;
    Procedure TestDate;
    Procedure TestDecimal;
    procedure TestDeclare;
    procedure TestDefault;
    procedure TestDelete;
    procedure TestDesc;
    procedure TestDescending;
    procedure TestDistinct;
    procedure TestDiv;
    procedure TestDO;
    procedure TestDomain;
    procedure TestDot;
    procedure TestDrop;
    procedure TestElse;
    procedure TestEmpty;
    procedure TestEnd;
    Procedure TestEntryPoint;
    procedure TestEq;
    procedure TestEscape;
    procedure TestException;
    procedure TestExists;
    procedure TestExit;
    Procedure TestExternal;
    Procedure TestExtract;
    procedure TestFile;
    Procedure TestFloat;
    procedure TestFor;
    procedure TestForeign;
    procedure TestFreeIt;
    procedure TestFunction;
    Procedure TestGDSError;
    procedure TestGE;
    procedure TestGenerator;
    procedure TestGrant;
    procedure TestGreaterThan;
    procedure TestGroup;
    Procedure TestHaving;
    procedure TestIf;
    procedure TestIn;
    procedure TestIndex;
    procedure TestInner;
    procedure TestInsert;
    procedure TestInt;
    procedure TestInteger;
    procedure TestInto;
    procedure TestIs;
    procedure TestJoin;
    procedure TestKey;
    procedure TestLeft;
    Procedure TestLength;
    procedure TestLike;
    procedure TestLE;
    procedure TestLessThan;
    Procedure TestManual;
    Procedure TestModuleName;
    procedure TestMax;
    procedure TestMerge;
    procedure TestMin;
    procedure TestMinus;
    procedure TestMul;
    procedure TestNational;
    procedure TestNatural;
    procedure TestNE;
    procedure TestNo;
    procedure TestNot;
    procedure TestNull;
    procedure TestNumeric;
    Procedure TestOn;
    Procedure TestOption;
    Procedure TestOr;
    Procedure TestOrder;
    Procedure TestOuter;
    Procedure TestPage;
    Procedure TestPages;
    Procedure TestPageSize;
    Procedure TestPassword;
    procedure TestPlus;
    procedure TestPosition;
    Procedure TestPostEvent;
    procedure TestPrimary;
    procedure TestPrivileges;
    procedure TestProcedure;
    procedure TestPublic;
    Procedure TestReferences;
    Procedure TestRelease;
    Procedure TestReturningValues;
    Procedure TestReturns;
    Procedure TestRetain;
    Procedure TestRevoke;
    Procedure TestRight;
    Procedure TestRole;
    Procedure TestRollback;
    Procedure TestSegment;
    Procedure TestSelect;
    procedure TestSemicolon;
    Procedure TestSet;
    Procedure TestSchema;
    Procedure TestShadow;
    Procedure TestSingular;
    Procedure TestSize;
    Procedure TestSkip;
    Procedure TestSome;
    Procedure TestSort;
    Procedure TestSnapshot;
    Procedure TestSQLCode;
    procedure TestSquareBraceClose;
    procedure TestSquareBraceOpen;
    Procedure TestStarting;
    procedure TestString;
    procedure TestSubtype;
    procedure TestSum;
    procedure TestSuspend;
    Procedure TestTable;
    Procedure TestThen;
    Procedure TestTime;
    Procedure TestTimeStamp;
    Procedure TestTo;
    Procedure TestTransaction;
    Procedure TestTrigger;
    Procedure TestType;
    Procedure TestUnion;
    Procedure TestUnique;
    Procedure TestUpdate;
    Procedure TestUpper;
    Procedure TestUser;
    procedure TestValue;
    procedure TestValues;
    procedure TestVariable;
    procedure TestVarChar;
    Procedure TestVarying;
    Procedure TestView;
    Procedure TestWhen;
    Procedure TestWhere;
    procedure TestWhile;
    procedure TestWith;
    procedure TestWork;
    procedure TestTerm;
    procedure TestTermExclude;
    procedure TestRecreate;
    procedure TestRestart;
    procedure TestSequence;
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
    procedure TestFloatLiteral;
    procedure TestStringLiteral1;
    procedure TestStringLiteral2;
    procedure TestSymbolLiteral1;
    procedure TestSymbolLiteral2;
    procedure TestStringError;
    procedure TestFloatError;
    Procedure TestOptionsoDoubleQuoteStringLiteral;
    Procedure TestOptionsoSingleQuoteIdentifier;
    procedure TestOptionsoBackQuoteIdentifier;
    procedure TestOptionNosoBackQuoteIdentifier;
    procedure TestOptionNosoBackslashEscapes;
    procedure TestOptionsoBackslashEscapesTab;
    procedure TestOptionsoBackslashEscapesNewLine;
    procedure TestOptionsoBackslashEscapesBackSlash;
    procedure TestOptionsoBackslashEscapesBackspace;
    procedure TestOptionsoBackslashEscapesLineFeed;
    procedure TestOptionsoBackslashEscapesNewPage;
    procedure TestOptionsoBackslashEscapesSlash;
    procedure TestOptionsoBackslashEscapesQuote;
    procedure TestOptionsoBackslashEscapesDoubleQuote;
  end;

implementation

uses typinfo;

procedure TTestSQLScanner.AssertEquals(AMessage: String; AExpected,
  AActual: TSQLToken);

Var
  S,EN1,EN2 : String;

begin
  If (AActual<>AExpected) then
    begin
    EN1:=GetEnumName(TypeINfo(TSQLToken),Ord(AExpected));
    EN2:=GetEnumName(TypeINfo(TSQLToken),Ord(AActual));
    S:=Format('%s : %s <> %s',[AMessage,EN1,EN2]);
    Fail(S);
    end;
end;

procedure TTestSQLScanner.CheckToken(AToken: TSQLToken; ASource: String);

Var
  J : TSqlToken;
  EN2 : String;

begin
  CreateScanner(ASource);
  J:=Scanner.FetchToken;
  EN2:=GetEnumName(TypeInfo(TSQLToken),Ord(AToken));
  AssertEquals(Format('Source %s should result in %s.',[ASource,EN2]),AToken,J);
end;


procedure TTestSQLScanner.DoTestFloat(F: Double);
Var
  S : String;

begin
  Str(F,S);
  DoTestFloat(F,S);
end;

procedure TTestSQLScanner.DoTestFloat(F: Double; S: String);
Var
  J : TSQLToken;
  C : Double;
  I : integer;
  V : String;

begin
  CreateScanner(S);
  try
    J:=FScanner.FetchToken;
    AssertEquals(S+' is a number',tsqlFloatNumber,J);
    V:=FScanner.CurTokenString;
    Val(V,C,I);
    If (I<>0) then
      Fail(FScanner.CurTokenString+' does not contain a float value');
    AssertEquals('Parsed float equals original float',F,C);
  finally
    FreeScanner;
  end;
end;

procedure TTestSQLScanner.DoTestString(S: String; DoubleDelim : Boolean = False);
Var
  J : TSQLToken;
  C : Char;

begin
  CreateScanner(S);
  try
    J:=FScanner.FetchToken;
    AssertEquals(S+' is a string',tsqlString,J);
    If (Length(S)>0) and (S[1] in ['"','''']) then
      begin
      C:=S[1];
      S:=Copy(S,2,Length(S)-2);
      end;
    If DoubleDelim then
      S:=StringReplace(S,C+C,C,[rfReplaceAll]);
    AssertEquals('Correct string is returned',S,FScanner.CurTokenString);
  finally
    FreeScanner;
  end;
end;

procedure TTestSQLScanner.TestErrorSource;
begin
  CreateScanner(FErrorSource,FErrorOptions);
  While (FScanner.FetchToken<>tsqlEOF) do ;
end;

function TTestSQLScanner.CreateScanner(AInput: String; AOptions : TSQLScannerOptions = []): TSQLScanner;
begin
  FStream:=TStringStream.Create(AInput);
  FLineReader:=TStreamLineReader.Create(Fstream);
  FScanner:=TSQLScanner.Create(FLineReader);
  FScanner.Options:=AOptions;
  Result:=FScanner;
end;

procedure TTestSQLScanner.FreeScanner;
begin
  FreeAndNil(FScanner);
  FreeAndNil(FLineReader);
  FreeAndNil(FStream);
end;

procedure TTestSQLScanner.SetUp;
begin
  inherited SetUp;
  FErrorSource:='';
  FErrorOptions:=[];
end;

procedure TTestSQLScanner.TearDown;
begin
  FreeScanner;
  iNHERITED;
end;

procedure TTestSQLScanner.TestAction;
begin
  CheckToken(tsqlAction,'ACTION');
end;

procedure TTestSQLScanner.TestAdd;
begin
  CheckToken(tsqlAdd,'ADD');
end;

procedure TTestSQLScanner.TestAdmin;
begin
  CheckToken(tsqlAdmin,'ADMIN');
end;

procedure TTestSQLScanner.TestAfter;
begin
  CheckToken(tsqlAfter,'AFTER');
end;

procedure TTestSQLScanner.TestEmpty;

Var
  J : TSQLToken;
begin
  CreateScanner('');
  J:=Scanner.FetchToken;
  If (J<>tsqlEOF) then
    Fail('Empty returns EOF');
end;

procedure TTestSQLScanner.TestEnd;
begin
  CheckToken(tsqlEnd,'END');
end;

procedure TTestSQLScanner.TestEntryPoint;
begin
  CheckToken(tsqlEntryPoint,'ENTRY_POINT');
end;

procedure TTestSQLScanner.TestEscape;
begin
  CheckToken(tsqlEscape,'ESCAPE');
end;

procedure TTestSQLScanner.TestException;
begin
  CheckToken(tsqlException,'EXCEPTION');
end;

procedure TTestSQLScanner.TestExists;
begin
  CheckToken(tsqlExists,'EXISTS');
end;

procedure TTestSQLScanner.TestExit;
begin
  CheckToken(tsqlExit,'EXIT');
end;

procedure TTestSQLScanner.TestExternal;
begin
  CheckToken(tsqlExternal,'external');
end;

procedure TTestSQLScanner.TestExtract;
begin
  CheckToken(tsqlExtract,'EXTRACT');
end;

procedure TTestSQLScanner.TestAnd;

begin
  CheckToken(tsqlAnd,'and');
end;

procedure TTestSQLScanner.TestAny;
begin
  CheckToken(tsqlAny,'any');
end;

procedure TTestSQLScanner.TestAs;
begin
  CheckToken(tsqlAs,'as');
end;

procedure TTestSQLScanner.TestAt;
begin
  CheckToken(tsqlAt,'at');
end;

procedure TTestSQLScanner.TestAuto;
begin
  CheckToken(tsqlAUTO,'auto');
end;

procedure TTestSQLScanner.TestASC;
begin
  CheckToken(tsqlASC,'asc');
end;

procedure TTestSQLScanner.TestASCENDING;
begin
  CheckToken(tsqlASCENDING,'ascending');
end;

procedure TTestSQLScanner.TestAVG;
begin
  CheckToken(tsqlAVG,'avg');
end;

procedure TTestSQLScanner.TestALTER;
begin
  CheckToken(tsqlALTER,'alter');
end;

procedure TTestSQLScanner.TestBraceOpen;

begin
  CheckToken(tsqlBraceOpen,'(');
end;

procedure TTestSQLScanner.TestBraceClose;

begin
  CheckToken(tsqlBraceClose,')');
end;


procedure TTestSQLScanner.TestComma;

begin
  CheckToken(tsqlComma,',');
end;

procedure TTestSQLScanner.TestCommit;
begin
  CheckToken(tsqlCommit,'COMMIT');
end;

procedure TTestSQLScanner.TestComputed;
begin
  CheckToken(tsqlComputed,'COMPUTED');
end;

procedure TTestSQLScanner.TestConditional;
begin
  CheckToken(tsqlConditional,'CONDITIONAL');
end;

procedure TTestSQLScanner.TestConnect;
begin
  CheckToken(tsqlConnect,'CONNECT');
end;

procedure TTestSQLScanner.TestConstraint;
begin
  CheckToken(tsqlConstraint,'CONSTRAINT');
end;

procedure TTestSQLScanner.TestColon;

begin
  CheckToken(tsqlColon,':');
end;

procedure TTestSQLScanner.TestColumn;
begin
  CheckToken(tsqlColumn,'COLUMN');
end;

procedure TTestSQLScanner.TestDot;

begin
  CheckToken(tsqlDot,'.');
end;

procedure TTestSQLScanner.TestDrop;
begin
  CheckToken(tsqldrop,'DROP');
end;

procedure TTestSQLScanner.TestSemicolon;

begin
  CheckToken(tsqlSemicolon,';');
end;

procedure TTestSQLScanner.TestSet;
begin
  CheckToken(tsqlSet,'SET');
end;

procedure TTestSQLScanner.TestSchema;
begin
  CheckToken(tsqlSchema,'SCHEMA');
end;

procedure TTestSQLScanner.TestShadow;
begin
  CheckToken(tsqlShadow,'SHADOW');
end;

procedure TTestSQLScanner.TestSingular;
begin
  CheckToken(tsqlSINGULAR,'SINGULAR');
end;

procedure TTestSQLScanner.TestSize;
begin
  CheckToken(tsqlSize,'size');
end;

procedure TTestSQLScanner.TestSkip;
begin
  CheckToken(tsqlSkip,'skip');
end;

procedure TTestSQLScanner.TestSome;
begin
  CheckToken(tsqlSome,'some');
end;

procedure TTestSQLScanner.TestSort;
begin
  CheckToken(tsqlSort,'sort');
end;

procedure TTestSQLScanner.TestSnapshot;
begin
  CheckToken(tsqlSnapshot,'snapshot');
end;

procedure TTestSQLScanner.TestSQLCode;
begin
  CheckToken(tsqlSQLCOde,'SQLCODE');
end;

procedure TTestSQLScanner.TestSquareBraceClose;
begin
  CheckToken(tsqlSquareBraceClose,']');
end;

procedure TTestSQLScanner.TestSquareBraceOpen;
begin
  CheckToken(tsqlSquareBraceOpen,'[');
end;

procedure TTestSQLScanner.TestSubtype;
begin
  CheckToken(tsqlSubtype,'sub_type');
end;

procedure TTestSQLScanner.TestSuspend;
begin
  CheckToken(tsqlSuspend,'Suspend');
end;

procedure TTestSQLScanner.TestSymbolLiteral1;
begin
  CheckToken(tsqlSymbolString,'%');
end;

procedure TTestSQLScanner.TestSymbolLiteral2;
begin
  CheckToken(tsqlSymbolString,'%^');
end;

procedure TTestSQLScanner.TestStarting;
begin
  CheckToken(tsqlStarting,'starting');
end;

procedure TTestSQLScanner.TestSum;
begin
  CheckToken(tsqlSum,'sum');
end;

procedure TTestSQLScanner.TestString;
begin
  DoTestString('''A string''');
  DoTestString('''''');
  DoTestString(''' " ''');
  DoTestString('''A string''');
  DoTestString(''' '''' ''',True);
  DoTestString('''12345'#10'67890''');
  DoTestString('''12345'#13'67890''');
  DoTestString('''12345'#13#10'67890''');
end;

procedure TTestSQLScanner.TestTable;
begin
  CheckToken(tsqltable,'TABLE');
end;

procedure TTestSQLScanner.TestThen;
begin
  CheckToken(tsqlThen,'THEN');
end;

procedure TTestSQLScanner.TestTime;
begin
  CheckToken(tsqlTime,'TIME');
end;

procedure TTestSQLScanner.TestTimeStamp;
begin
  CheckToken(tsqlTimeStamp,'TIMESTAMP');
end;

procedure TTestSQLScanner.TestTo;
begin
  CheckToken(tsqlTo,'TO');
end;

procedure TTestSQLScanner.TestTransaction;
begin
  CheckToken(tsqlTransaction,'TRANSACTION');
end;

procedure TTestSQLScanner.TestTrigger;
begin
  CheckToken(tsqlTrigger,'TRIGGER');
end;

procedure TTestSQLScanner.TestType;
begin
  CheckToken(tsqltype,'TYPE');
end;

procedure TTestSQLScanner.TestUnion;
begin
  CheckToken(tsqlUnion,'UNION');
end;

procedure TTestSQLScanner.TestUnique;
begin
  CheckToken(tsqlUnique,'UNIQUE');
end;

procedure TTestSQLScanner.TestUpdate;
begin
  CheckToken(tsqlUPDATE,'UPDATE');
end;

procedure TTestSQLScanner.TestUpper;
begin
  CheckToken(tsqlUPPER,'UPPER');
end;

procedure TTestSQLScanner.TestUser;
begin
  CheckToken(tsqlUSER,'USER');
end;

procedure TTestSQLScanner.TestValues;
begin
  CheckToken(tsqlVALUES,'VALUES');
end;

procedure TTestSQLScanner.TestValue;
begin
  CheckToken(tsqlVALUE,'VALUE');
end;

procedure TTestSQLScanner.TestAssign;

begin
  CheckToken(tsqlEq,'=');
end;

procedure TTestSQLScanner.TestBefore;
begin
  CheckToken(tsqlbefore,'BEFORE');
end;

procedure TTestSQLScanner.TestBegin;
begin
  CheckToken(tsqlbegin,'BEGIN');
end;

procedure TTestSQLScanner.TestBetween;
begin
  CheckToken(tsqlBetween,'BETWEEN');
end;

procedure TTestSQLScanner.TestGreaterThan;

begin
  CheckToken(tsqlGT,'>');
end;

procedure TTestSQLScanner.TestGroup;
begin
  CheckToken(tsqlGroup,'group');
end;

procedure TTestSQLScanner.TestHaving;
begin
  CheckToken(tsqlHaving,'HAVING');
end;

procedure TTestSQLScanner.TestLessThan;

begin
  CheckToken(tsqlLT,'<');
end;

procedure TTestSQLScanner.TestManual;
begin
  Checktoken(tsqlManual,'manual');
end;

procedure TTestSQLScanner.TestModuleName;
begin
  Checktoken(tsqlModuleName,'module_name');
end;

procedure TTestSQLScanner.TestMax;
begin
  Checktoken(tsqlMax,'max');
end;

procedure TTestSQLScanner.TestMerge;
begin
  Checktoken(tsqlMerge,'merge');
end;

procedure TTestSQLScanner.TestMin;
begin
  Checktoken(tsqlMin,'min');
end;

procedure TTestSQLScanner.TestPlus;

begin
  CheckToken(tsqlPlus,'+');
end;

procedure TTestSQLScanner.TestPosition;
begin
  Checktoken(tsqlposition,'position');
end;

procedure TTestSQLScanner.TestPostEvent;
begin
  Checktoken(tsqlpostevent,'post_event');
end;

procedure TTestSQLScanner.TestPrimary;
begin
  Checktoken(tsqlprimary,'primary');
end;

procedure TTestSQLScanner.TestPrivileges;
begin
  Checktoken(tsqlprivileges,'privileges');
end;

procedure TTestSQLScanner.TestProcedure;
begin
  Checktoken(tsqlprocedure,'procedure');
end;

procedure TTestSQLScanner.TestPublic;
begin
  CheckToken(tsqlPublic,'PUBLIC');
end;

procedure TTestSQLScanner.TestReferences;
begin
  CheckToken(tsqlReferences,'REFERENCES');
end;

procedure TTestSQLScanner.TestRelease;
begin
  CheckToken(tsqlrelease,'release');
end;

procedure TTestSQLScanner.TestReturningValues;
begin
  CheckToken(tsqlreturningvalues,'returning_values');
end;

procedure TTestSQLScanner.TestReturns;
begin
  CheckToken(tsqlreturns,'returns');
end;

procedure TTestSQLScanner.TestRetain;
begin
  Checktoken(tsqlRetain,'retain');
end;

procedure TTestSQLScanner.TestRevoke;
begin
  Checktoken(tsqlRevoke,'revoke');
end;

procedure TTestSQLScanner.TestRight;
begin
  Checktoken(tsqlright,'right');
end;

procedure TTestSQLScanner.TestRole;
begin
  Checktoken(tsqlrole,'role');
end;

procedure TTestSQLScanner.TestRollback;
begin
  Checktoken(tsqlrollback,'rollback');
end;

procedure TTestSQLScanner.TestSegment;
begin
  CheckToken(tsqlSegment,'SEGMENT');
end;

procedure TTestSQLScanner.TestSelect;
begin
  CheckToken(tsqlSelect,'SELECT');
end;

procedure TTestSQLScanner.TestMinus;

begin
  CheckToken(tsqlMinus,'-');
end;

procedure TTestSQLScanner.TestMul;

begin
  CheckToken(tsqlMul,'*');
end;

procedure TTestSQLScanner.TestNational;
begin
  CheckToken(tsqlNational,'NATIONAL');
end;

procedure TTestSQLScanner.TestNatural;
begin
  CheckToken(tsqlNatural,'NATURAL');
end;

procedure TTestSQLScanner.TestDiv;

begin
  CheckToken(tsqlDiv,'/');
end;

procedure TTestSQLScanner.TestEq;

begin
  CheckToken(tsqlEq,'==');
end;

procedure TTestSQLScanner.TestFloat;
begin
  CheckToken(tsqlFloat,'FLOAT');
end;

procedure TTestSQLScanner.TestGE;

begin
  CheckToken(tsqlGE,'>=');
end;

procedure TTestSQLScanner.TestGenerator;
begin
  CheckToken(tsqlGenerator,'generator');
end;

procedure TTestSQLScanner.TestGrant;
begin
  CheckToken(tsqlGrant,'grant');
end;

procedure TTestSQLScanner.TestLE;

begin
  CheckToken(tsqlLE,'<=');
end;


procedure TTestSQLScanner.TestNE;

begin
  CheckToken(tsqlNE,'<>');
end;

procedure TTestSQLScanner.TestNo;
begin
  CheckToken(tsqlNo,'no');
end;

procedure TTestSQLScanner.TestNot;

begin
  CheckToken(tsqlNot,'not');
end;

procedure TTestSQLScanner.TestNull;
begin
  CheckToken(tsqlnull,'null');
end;

procedure TTestSQLScanner.TestNumeric;
begin
  CheckToken(tsqlNumeric,'NUMERIC');
end;

procedure TTestSQLScanner.TestOn;
begin
  CheckToken(tsqlON,'on');
end;

procedure TTestSQLScanner.TestOption;
begin
  CheckToken(tsqlOption,'option');
end;

procedure TTestSQLScanner.TestOr;
begin
  CheckToken(tsqlOR,'or');
end;

procedure TTestSQLScanner.TestOrder;
begin
  CheckToken(tsqlORDER,'order');
end;

procedure TTestSQLScanner.TestOuter;
begin
  CheckToken(tsqlOUTER,'outer');
end;

procedure TTestSQLScanner.TestPage;
begin
  CheckToken(tsqlPage,'PAGE');
end;

procedure TTestSQLScanner.TestPages;
begin
  CheckToken(tsqlPages,'PAGES');
end;

procedure TTestSQLScanner.TestPageSize;
begin
  CheckToken(tsqlPageSize,'PAGE_SIZE');
end;

procedure TTestSQLScanner.TestPassword;
begin
  CheckToken(tsqlPassword,'PASSWORD');
end;

{
procedure TTestSQLScanner.TestTrue;

begin
  CheckToken(tsqlTrue,'true');
end;

procedure TTestSQLScanner.TestFalse;

begin
  CheckToken(tsqlFalse,'false');
end;
}

procedure TTestSQLScanner.TestBy;

begin
  CheckToken(tsqlBy,'by');
end;

procedure TTestSQLScanner.TestCache;
begin
  CheckToken(tsqlCache,'CACHE');
end;

procedure TTestSQLScanner.TestCascade;
begin
  CheckToken(tsqlCascade,'cascade');
end;

procedure TTestSQLScanner.TestBlob;

begin
  CheckToken(tsqlBlob,'blob');
end;

procedure TTestSQLScanner.TestChar;
begin
  CheckToken(tsqlChar,'char');
end;

procedure TTestSQLScanner.TestCharacter;
begin
  CheckToken(tsqlCharacter,'character');
end;

procedure TTestSQLScanner.TestCheck;
begin
  CheckToken(tsqlCHECK,'check');
end;

procedure TTestSQLScanner.TestCollate;

begin
  CheckToken(tsqlCollate,'collate');
end;

procedure TTestSQLScanner.TestConcatenate;
begin
  CheckToken(tsqlConcatenate,'||');
end;

procedure TTestSQLScanner.TestContaining;

begin
  CheckToken(tsqlContaining,'containing');
end;

procedure TTestSQLScanner.TestCount;

begin
  CheckToken(tsqlCount,'count');
end;

procedure TTestSQLScanner.TestCreate;
begin
  CheckToken(tsqlCreate,'create');
end;

procedure TTestSQLScanner.TestCString;
begin
  CheckToken(tsqlCString,'CSTRING');
end;

procedure TTestSQLScanner.TestDatabase;
begin
  CheckToken(tsqlDatabase,'database');
end;

procedure TTestSQLScanner.TestDate;
begin
  CheckToken(tsqlDate,'date');
end;

procedure TTestSQLScanner.TestDecimal;
begin
  CheckToken(tsqlDecimal,'decimal');
end;

procedure TTestSQLScanner.TestDefault;

begin
  CheckToken(tsqldefault,'default');
end;

procedure TTestSQLScanner.TestDelete;

begin
  CheckToken(tsqldelete,'delete');
end;

procedure TTestSQLScanner.TestDeclare;
begin
  CheckToken(tsqlDeclare,'DECLARE');
end;

procedure TTestSQLScanner.TestDesc;
begin
  CheckToken(tsqlDESC,'DESC');
end;

procedure TTestSQLScanner.TestDescending;
begin
 CheckToken(tsqlDescending,'DESCENDING');
end;

procedure TTestSQLScanner.TestDistinct;
begin
 CheckToken(tsqlDistinct,'DISTINCT');
end;

procedure TTestSQLScanner.TestDO;

begin
  CheckToken(tsqldo,'do');
end;

procedure TTestSQLScanner.TestDomain;
begin
  CheckToken(tsqlDomain,'domain');
end;

procedure TTestSQLScanner.TestElse;

begin
  CheckToken(tsqlelse,'else');
end;

procedure TTestSQLScanner.TestFor;

begin
  CheckToken(tsqlfor,'for');
end;

procedure TTestSQLScanner.TestForeign;
begin
  CheckToken(tsqlForeign,'foreign');
end;

procedure TTestSQLScanner.TestFreeIt;
begin
  CheckToken(tsqlFreeIt,'FREE_IT');
end;

procedure TTestSQLScanner.TestFile;
begin
  CheckToken(tsqlFile,'FILE');
end;

procedure TTestSQLScanner.TestFunction;

begin
  CheckToken(tsqlfunction,'function');
end;

procedure TTestSQLScanner.TestGDSError;
begin
  CheckToken(tsqlGDSCODE,'GDSCODE');
end;

procedure TTestSQLScanner.TestIf;

begin
  CheckToken(tsqlif,'if');
end;

procedure TTestSQLScanner.TestIn;

begin
  CheckToken(tsqlin,'in');
end;

procedure TTestSQLScanner.TestInner;

begin
  CheckToken(tsqlInner,'inner');
end;

procedure TTestSQLScanner.TestInsert;

begin
  CheckToken(tsqlInsert,'insert');
end;

procedure TTestSQLScanner.TestInt;

begin
  CheckToken(tsqlInt,'int');
end;

procedure TTestSQLScanner.TestInteger;

begin
  CheckToken(tsqlInteger,'integer');
end;

procedure TTestSQLScanner.TestInto;
begin
  CheckToken(tsqlInto,'into');
end;

procedure TTestSQLScanner.TestIs;
begin
  CheckToken(tsqlis,'is');
end;

procedure TTestSQLScanner.TestJoin;
begin
  CheckToken(tsqlJoin,'JOIN');
end;

procedure TTestSQLScanner.TestKey;
begin
  CheckToken(tsqlKey,'KEY');
end;

procedure TTestSQLScanner.TestLeft;
begin
  CheckToken(tsqlLEFT,'LEFT');
end;

procedure TTestSQLScanner.TestLength;
begin
  CheckToken(tsqlLength,'LENGTH');
end;

procedure TTestSQLScanner.TestLike;
begin
  CheckToken(tsqlLIKE,'LIKE');
end;

procedure TTestSQLScanner.TestIndex;
begin
  CheckToken(tsqlindex,'index');
end;

procedure TTestSQLScanner.TestVariable;

begin
  CheckToken(tsqlvariable,'variable');
end;

procedure TTestSQLScanner.TestVarChar;
begin
  CheckToken(tsqlvarchar,'varchar');
end;

procedure TTestSQLScanner.TestVarying;
begin
  CheckToken(tsqlvarying,'varying');
end;

procedure TTestSQLScanner.TestView;
begin
  CheckToken(tsqlview,'view');
end;

procedure TTestSQLScanner.TestWhen;
begin
  CheckToken(tsqlWhen,'when');
end;

procedure TTestSQLScanner.TestWhere;
begin
  CheckToken(tsqlwhere,'where');
end;

procedure TTestSQLScanner.TestWhile;

begin
  CheckToken(tsqlwhile,'while');
end;

procedure TTestSQLScanner.TestWith;

begin
  CheckToken(tsqlwith,'with');
end;

procedure TTestSQLScanner.TestWork;
begin
  CheckToken(tsqlWork,'work');
end;

procedure TTestSQLScanner.TestTerm;
begin
  CheckToken(tsqlTerm,'term');
end;

procedure TTestSQLScanner.TestTermExclude;
begin
  CreateScanner('term');
  FScanner.Excludekeywords.Add('term');
  AssertEquals('Term is identifier',tsqlIdentifier,FScanner.FetchToken);
end;

procedure TTestSQLScanner.TestRecreate;
begin
  CheckToken(tsqlRecreate,'recreate');
end;

procedure TTestSQLScanner.TestRestart;
begin
  CheckToken(tsqlRestart,'restart');
end;

procedure TTestSQLScanner.TestSequence;
begin
  CheckToken(tsqlSequence,'sequence');
end;

procedure TTestSQLScanner.CheckTokens(ASource : String; ATokens : Array of TSQLToken);

Var
  I : Integer;
  J : TSQLToken;
  S : String;

begin
  CreateScanner(ASource);
  For I:=Low(ATokens) to High(ATokens) do
    begin
    J:=FScanner.FetchToken;
    S:=GetEnumName(TypeINfo(TSQLToken),Ord(ATokens[i]));
    S:=Format('Source "%s", token %d (%s): expected %s',[ASource,I,FScanner.CurTokenString,S]);
    AssertEquals(S,ATokens[i],J);
    end;
end;

procedure TTestSQLScanner.Test2Words;
begin
  CheckTokens('with do',[tsqlWith,tsqlDo]);
end;

procedure TTestSQLScanner.Test3Words;
begin
  CheckTokens('with do for',[tsqlWith,tsqlDo,tsqlFor]);
end;

procedure TTestSQLScanner.TestIdentifier;
begin
  CheckToken(tsqlIdentifier,'something');
  AssertEquals('Correct identifier','something',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestIdentifier2;
begin
  CheckToken(tsqlIdentifier,'"_something"');
  AssertEquals('Correct identifier','_something',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestIdentifier3;
begin
  CheckToken(tsqlIdentifier,'RDB$');
  AssertEquals('Correct identifier','RDB$',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestIdentifier4;
begin
  CheckToken(tsqlIdentifier,'A_0');
  AssertEquals('Correct identifier','A_0',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestIdentifier5;
begin
  // $0 should not be parsed as an identifier but as a symbol literal
  CheckToken(tsqlSymbolString,'$0');
end;

procedure TTestSQLScanner.TestIdentifierDotIdentifier;
begin
  CheckTokens('something.different',[tsqlIdentifier,tsqldot,tsqlIdentifier]);
//  AssertEquals('Correct identifier','something',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestEOLN;
begin
  CreateScanner('something');
  FScanner.FetchToken;
  AssertEquals('Got to end of line after reading single token at EOF',True,FScanner.IsEndOfLine);
//  AssertEquals('Correct identifier','something',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestEOLN2;
begin
  CreateScanner('something different');
  FScanner.FetchToken;
  AssertEquals('Not yet end of line after reading single token at EOF',False,FScanner.IsEndOfLine);
end;

procedure TTestSQLScanner.TestEOLN3;
begin
  CreateScanner('something'#13#10'different');
  FScanner.FetchToken;
  AssertEquals('End of line after reading single token',True,FScanner.IsEndOfLine);
end;

procedure TTestSQLScanner.TestEOLN4;
begin
  CreateScanner('something'#10'different');
  FScanner.FetchToken;
  AssertEquals('End of line after reading first token',True,FScanner.IsEndOfLine);
  FScanner.FetchToken;
  AssertEquals('End of line after reading second token',True,FScanner.IsEndOfLine);
end;

procedure TTestSQLScanner.TestComment1;
begin
  CreateScanner('-- some comment string');
  AssertEquals('Comment line is skipped',tsqlEOF,FScanner.FetchToken);
end;

procedure TTestSQLScanner.TestComment2;
begin
  CreateScanner('--  some comment string');
  FScanner.Options:=[soReturnComments];
  AssertEquals('Comment line is returned',tsqlComment,FScanner.FetchToken);
  AssertEquals('Comment contents is returned',' some comment string',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestComment3;
begin
  CreateScanner('/* some comment string */');
  AssertEquals('Comment line is skipped',tsqlEOF,FScanner.FetchToken);
end;

procedure TTestSQLScanner.TestComment4;
begin
  CreateScanner('/* some comment string */');
  FScanner.Options:=[soReturnComments];
  AssertEquals('Comment line is returned',tsqlComment,FScanner.FetchToken);
  AssertEquals('Comment contents is returned',' some comment string ',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestComment5;
begin
  CreateScanner('/* some nested comment -- string */');
  FScanner.Options:=[soReturnComments];
  AssertEquals('Comment line is returned',tsqlComment,FScanner.FetchToken);
  AssertEquals('Comment contents is returned',' some nested comment -- string ',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestComment6;
begin
  CreateScanner('-- /* some nested comment string */');
  FScanner.Options:=[soReturnComments];
  AssertEquals('Comment line is returned',tsqlComment,FScanner.FetchToken);
  AssertEquals('Comment contents is returned','/* some nested comment string */',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestFloatLiteral;
begin
  DoTestFloat(1.2);
  DoTestFloat(-1.2);
  DoTestFloat(1.2e1);
  DoTestFloat(-1.2e1);
  DoTestFloat(1.2,'1.2');
  DoTestFloat(-1.2,'-1.2');
  DoTestFloat(0,'0.0');
end;

procedure TTestSQLScanner.TestStringLiteral1;
begin
  CreateScanner('''A''');
  FScanner.Options:=[];
  AssertEquals('String is returned',tsqlString,FScanner.FetchToken);
  AssertEquals('Correct string','A',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestStringLiteral2;
begin
  CreateScanner('"A"',[soDoubleQuoteStringLiteral]);
  AssertEquals('String is returned',tsqlString,FScanner.FetchToken);
  AssertEquals('Correct string','A',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestStringError;
begin

end;

procedure TTestSQLScanner.TestFloatError;
begin
  FErrorSource:='1xz';
  AssertException('Wrong float',ESQLScannerError,@TestErrorSource);
end;

procedure TTestSQLScanner.TestOptionsoDoubleQuoteStringLiteral;
begin
  CreateScanner('"A"',[soDoubleQuoteStringLiteral]);
  AssertEquals('String is returned',tsqlString,FScanner.FetchToken);
  AssertEquals('Correct string','A',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestOptionsoSingleQuoteIdentifier;
begin
  CreateScanner('''A''',[soSingleQuoteIdentifier,soDoubleQuoteStringLiteral]);
  If (Scanner.Options<>[soSingleQuoteIdentifier,soDoubleQuoteStringLiteral]) then
    Fail('Options not correctly set');
  AssertEquals('Identifier is returned',tsqlIdentifier,FScanner.FetchToken);
  AssertEquals('Correct string','A',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestOptionsoBackQuoteIdentifier;
begin
  CreateScanner('`A`',[soBackQuoteIdentifier]);
  AssertEquals('String is returned',tsqlIdentifier,FScanner.FetchToken);
  AssertEquals('Correct string','A',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestOptionNosoBackQuoteIdentifier;
begin
  FErrorSource:='`A`';
  AssertException('Wrong token',ESQLScannerError,@TestErrorSource);
end;

procedure TTestSQLScanner.TestOptionsoBackslashEscapesTab;
begin
  CreateScanner('''\t''',[soBackslashEscapes]);
  AssertEquals('String is returned',tsqlString,FScanner.FetchToken);
  AssertEquals('Correct string',#9,FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestOptionsoBackslashEscapesNewLine;
begin
  CreateScanner('''\n''',[soBackslashEscapes]);
  AssertEquals('String is returned',tsqlString,FScanner.FetchToken);
  AssertEquals('Correct string',#10,FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestOptionsoBackslashEscapesLineFeed;

begin
  CreateScanner('''\r''',[soBackslashEscapes]);
  AssertEquals('String is returned',tsqlString,FScanner.FetchToken);
  AssertEquals('Correct string',#13,FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestOptionsoBackslashEscapesBackSlash;

begin
  CreateScanner('''\\''',[soBackslashEscapes]);
  AssertEquals('String is returned',tsqlString,FScanner.FetchToken);
  AssertEquals('Correct string','\',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestOptionsoBackslashEscapesNewPage;

begin
  CreateScanner('''\f''',[soBackslashEscapes]);
  AssertEquals('String is returned',tsqlString,FScanner.FetchToken);
  AssertEquals('Correct string',#12,FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestOptionsoBackslashEscapesSlash;
begin
  CreateScanner('''\/''',[soBackslashEscapes]);
  AssertEquals('String is returned',tsqlString,FScanner.FetchToken);
  AssertEquals('Correct string','/',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestOptionsoBackslashEscapesBackspace;

begin
  CreateScanner('''\f''',[soBackslashEscapes]);
  AssertEquals('String is returned',tsqlString,FScanner.FetchToken);
  AssertEquals('Correct string',#12,FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestOptionsoBackslashEscapesQuote;

begin
  CreateScanner('''\''''',[soBackslashEscapes]);
  AssertEquals('String is returned',tsqlString,FScanner.FetchToken);
  AssertEquals('Correct string','''',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestOptionsoBackslashEscapesDoubleQuote;
begin
  CreateScanner('''\"''',[soBackslashEscapes]);
  AssertEquals('String is returned',tsqlString,FScanner.FetchToken);
  AssertEquals('Correct string','"',FScanner.CurTokenString);
end;

procedure TTestSQLScanner.TestOptionNosoBackslashEscapes;
begin
  CreateScanner('''\"''',[]);
  AssertEquals('String is returned',tsqlString,FScanner.FetchToken);
  AssertEquals('Correct string','\"',FScanner.CurTokenString);
end;


initialization

  RegisterTest(TTestSQLScanner);
end.

