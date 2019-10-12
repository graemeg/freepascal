{
    This file is part of the Free Component Library
    Copyright (c) 2010-2014 by the Free Pascal development team

    SQL source lexical scanner

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$mode objfpc}
{$h+}

unit fpsqlscanner;

interface

uses SysUtils, Classes, Contnrs;

resourcestring
  SErrInvalidCharacter = 'Invalid character ''%s''';
  SErrOpenString = 'string exceeds end of line';
  SErrIncludeFileNotFound = 'Could not find include file ''%s''';
  SErrIfXXXNestingLimitReached = 'Nesting of $IFxxx too deep';
  SErrInvalidPPElse = '$ELSE without matching $IFxxx';
  SErrInvalidPPEndif = '$ENDIF without matching $IFxxx';
  SInvalidHexadecimalNumber = 'Invalid hexadecimal number';
  SErrInvalidNonEqual = 'SyntaxError: != or !== expected';
  SBarExpected = '| character expected';

type

   TSQLToken = (tsqlUnknown,
   // Specials
   tsqlEOF,tsqlWhiteSpace,
   tsqlString {string literal},
   tsqlIdentifier {a table etc name},
   tsqlSymbolString {a string containing symbols/punctuation marks; only rarely used - e.g. in SET TERM ^ ;},
   tsqlIntegerNumber,tsqlFloatNumber,tsqlComment,
   tsqlBraceOpen,tsqlBraceClose,tsqlSquareBraceOpen,tsqlSquareBraceClose,
   tsqlPlaceHolder {question mark},
   tsqlCOMMA,tsqlCOLON,tsqlDOT,tsqlSEMICOLON,tsqlTerminator,
   tsqlGT,tsqlLT,tsqlPLUS,tsqlMINUS,tsqlMUL,tsqlDIV,tsqlConcatenate,
   tsqlEQ,tsqlGE,tsqlLE,tsqlNE,
   { Reserved words/keywords start here. They must be last }
   { Note: if adding before tsqlALL or after tsqlWHEN please update FirstKeyword/LastKeyword }
   tsqlALL, tsqlAND, tsqlANY, tsqlASC, tsqlASCENDING, tsqlAVG, tsqlALTER, tsqlAdd, tsqlActive, tsqlAction, tsqlAs,tsqlAt, tsqlAuto, tsqlAfter,tsqlAdmin,
   tsqlBETWEEN, tsqlBinary, tsqlBY, tsqlBLOB, tsqlBegin, tsqlBefore,
   tsqlCOLLATE, tsqlCONTAINING, tsqlCOUNT, tsqlCREATE, tsqlCOLUMN, tsqlCONSTRAINT, tsqlChar,tsqlCHARACTER, tsqlCHECK, tsqlComputed,tsqlCASCADE, tsqlCast, tsqlCommit,tsqlConnect,tsqlCache,tsqlConditional,tsqlCString,
   tsqlDESC, tsqlDESCENDING, tsqlDISTINCT, tsqlDEFAULT, tsqlDELETE, tsqlDO, tsqlDouble, tsqlDECLARE, tsqlDROP, tsqlDomain, tsqlDecimal, tsqlDate,tsqlDatabase,
   tsqlESCAPE, tsqlEXISTS, tsqlELSE, tsqlException,   tsqlExternal, tsqlExecute, tsqlEnd,tsqlExit,tsqlEntrypoint,tsqlExtract,
   tsqlFROM, tsqlFULL, tsqlFOREIGN, tsqlFOR, tsqlFUNCTION, tsqlFLOAT, tsqlFile,tsqlFreeIt,
   tsqlGenerator, tsqlGROUP, tsqlGenID,tsqlGDSCODE,tsqlGrant,
   tsqlHAVING,
   tsqlIF, tsqlIN, tsqlINNER, tsqlINSERT, tsqlINT, tsqlINTEGER, tsqlINTO, tsqlIS, tsqlINDEX,  tsqlInactive,
   tsqlJOIN,
   tsqlKEY,
   tsqlLEFT, tsqlLIKE, tsqlLength,
   tsqlMAX, tsqlMIN, tsqlMERGE, tsqlManual, tsqlModuleName,
   tsqlNOT, tsqlNULL, tsqlNUMERIC , tsqlNChar, tsqlNATIONAL,tsqlNO, tsqlNatural,
   tsqlOFF {not an FB reserved word; used in isql scripts}, tsqlON, tsqlOR, tsqlORDER, tsqlOUTER, tsqlOption,
   tsqlPrecision, tsqlPRIMARY,  tsqlProcedure, tsqlPosition, tsqlPlan, tsqlPassword, tsqlPage,tsqlPages,tsqlPageSize,tsqlPostEvent,tsqlPrivileges,tsqlPublic,
   tsqlRIGHT, tsqlROLE, tsqlReferences, tsqlRollBack, tsqlRelease,  tsqlretain,  tsqlReturningValues,tsqlReturns, tsqlrevoke,
   tsqlSELECT, tsqlSET, tsqlSINGULAR, tsqlSOME, tsqlSTARTING, tsqlSUM, tsqlSKIP,tsqlSUBTYPE,tsqlSize,tsqlSegment, tsqlSORT, tsqlSnapShot,tsqlSchema,tsqlShadow,tsqlSuspend,tsqlSQLCode,tsqlSmallint,
   tSQLTABLE, tsqlText, tsqlTrigger, tsqlTime, tsqlTimeStamp, tsqlType, tsqlTo, tsqlTransaction, tsqlThen,
   tsqlUNION, tsqlUPDATE, tsqlUPPER,  tsqlUNIQUE, tsqlUSER,
   tsqlValue, tsqlVALUES, tsqlVARIABLE,  tsqlVIEW, tsqlVARCHAR,TSQLVARYING,
   tsqlWHERE, tsqlWITH, tsqlWHILE, tsqlWork, tsqlWhen,tsqlSequence,tsqlRestart,tsqlrecreate,tsqlterm
 );
   TSQLTokens = set of TSQLToken;

const
  FirstKeyword = tsqlAll;
  LastKeyWord = tsqlTerm;
  sqlComparisons = [tsqleq,tsqlGE,tsqlLE,tsqlNE,tsqlGT,tsqlLT,tsqlIn,tsqlIS,
                    tsqlbetween,tsqlLike,tsqlContaining,tsqlStarting,tsqlNOT];
  sqlInvertableComparisons = [tsqlLike,tsqlContaining,tsqlStarting,tsqlIN,tsqlIS, tsqlBetween];

  // Strings that represent tokens in TSQLToken
  TokenInfos: array[TSQLToken] of string = ('unknown',
       // Specials
       'EOF','whitespace',
       'String',
       'identifier',
       'symbol string',
       'integer number','float number', 'comment',
       '(',')', '[',']',
       '?',',',':','.',';','',
       '>','<',
       '+','-','*','/','||',
       '=','>=','<=','<>',
       // Identifiers last:
       'ALL', 'AND', 'ANY', 'ASC', 'ASCENDING', 'AVG', 'ALTER', 'ADD','ACTIVE','ACTION', 'AS', 'AT', 'AUTO', 'AFTER', 'ADMIN',
       'BETWEEN', 'BINARY', 'BY', 'BLOB','BEGIN', 'BEFORE',
       'COLLATE', 'CONTAINING', 'COUNT', 'CREATE', 'COLUMN', 'CONSTRAINT', 'CHAR','CHARACTER','CHECK', 'COMPUTED','CASCADE','CAST', 'COMMIT', 'CONNECT', 'CACHE','CONDITIONAL', 'CSTRING',
       'DESC', 'DESCENDING', 'DISTINCT',  'DEFAULT', 'DELETE', 'DO', 'DOUBLE', 'DECLARE', 'DROP', 'DOMAIN', 'DECIMAL', 'DATE','DATABASE',
       'ESCAPE', 'EXISTS', 'ELSE', 'EXCEPTION', 'EXTERNAL','EXECUTE', 'END','EXIT','ENTRY_POINT','EXTRACT',
       'FROM', 'FULL','FOREIGN', 'FOR', 'FUNCTION', 'FLOAT','FILE', 'FREE_IT',
       'GENERATOR', 'GROUP', 'GEN_ID','GDSCODE','GRANT',
       'HAVING',
       'IF', 'IN', 'INNER', 'INSERT', 'INT', 'INTEGER', 'INTO', 'IS', 'INDEX', 'INACTIVE',
       'JOIN',
       'KEY',
       'LEFT', 'LIKE', 'LENGTH',
       'MAX', 'MIN', 'MERGE', 'MANUAL', 'MODULE_NAME',
       'NOT', 'NULL', 'NUMERIC','NCHAR','NATIONAL', 'NO', 'NATURAL',
       'OFF', 'ON', 'OR', 'ORDER', 'OUTER', 'OPTION',
       'PRECISION', 'PRIMARY', 'PROCEDURE','POSITION','PLAN', 'PASSWORD','PAGE','PAGES','PAGE_SIZE','POST_EVENT','PRIVILEGES','PUBLIC',
       'RIGHT', 'ROLE', 'REFERENCES', 'ROLLBACK','RELEASE', 'RETAIN', 'RETURNING_VALUES', 'RETURNS','REVOKE',
       'SELECT', 'SET', 'SINGULAR', 'SOME', 'STARTING', 'SUM', 'SKIP','SUB_TYPE', 'SIZE', 'SEGMENT', 'SORT', 'SNAPSHOT','SCHEMA','SHADOW','SUSPEND','SQLCODE','SMALLINT',
       'TABLE', 'TEXT', 'TRIGGER', 'TIME', 'TIMESTAMP', 'TYPE', 'TO', 'TRANSACTION', 'THEN',
       'UNION', 'UPDATE', 'UPPER', 'UNIQUE', 'USER',
       'VALUE','VALUES','VARIABLE', 'VIEW','VARCHAR','VARYING',
       'WHERE', 'WITH', 'WHILE','WORK','WHEN','SEQUENCE','RESTART','RECREATE','TERM'
  );

Type
  TLineReader = class
  public
    function IsEOF: Boolean; virtual; abstract;
    function ReadLine: string; virtual; abstract;
  end;

  { TStreamLineReader }

  TStreamLineReader = class(TLineReader)
  private
    FStream : TStream;
    Buffer : Array[0..1024] of Byte;
    FBufPos,
    FBufLen : Integer;
    procedure FillBuffer;
  public
    Constructor Create(AStream : TStream);
    function IsEOF: Boolean; override;
    function ReadLine: string; override;
  end;

  TFileLineReader = class(TLineReader)
  private
    FTextFile: Text;
    FileOpened: Boolean;
  public
    constructor Create(const AFilename: string);
    destructor Destroy; override;
    function IsEOF: Boolean; override;
    function ReadLine: string; override;
  end;

  ESQLScannerError       = class(Exception);

  { TSQLScanner }
  TSQLScannerOption = (soReturnComments,
                       soReturnWhiteSpace,
                       soBackslashEscapes,
                       soNoDoubleDelimIsChar,
                       soDoubleQuoteStringLiteral,  // Default: single quote is string literal
                       soSingleQuoteIdentifier,     // Default: double quote is identifier. Ignored if soDoubleQuoteStringLiteral is not specified
                       soBackQuoteIdentifier        // Default: double quote is identifier
                       );
  TSQLScannerOptions = Set of TSQLScannerOption;

  TSQLScanner = class
  private
    FAlternateTerminator: String;
    FOptions: TSQLScannerOptions;
    FSourceFile: TLineReader;
    FSourceFilename: string;
    FCurRow: Integer;
    FCurToken: TSQLToken;
    FCurTokenString: string;
    FCurLine: string;
    TokenStr: PChar;
    FSourceStream : TStream;
    FOwnSourceFile : Boolean;
    FKeyWords : TFPHashList;
    FExclude : TStringList;
    function CommentDiv: TSQLToken;
    // Used to parse out an identifier/name and store it in the list of identifiers
    function DoIdentifier : TSQLToken;
    // Used to parse out a string containing symbols
    function DoSymbolString : TSQLToken;
    function DoMultiLineComment: TSQLToken;
    function DoNumericLiteral: TSQLToken;
    function DoSingleLineComment: TSQLToken;
    function DoStringLiteral: TSQLToken;
    function DoWhiteSpace: TSQLToken;
    // Reads a new line into TokenStr and returns true
    // If no new lines, returns false
    function FetchLine: Boolean;
    function GetCurColumn: Integer;
    function GetExcludeKeywords: TStrings;
    function ReadUnicodeEscape: WideChar;
    procedure SetExcludeKeywords(const AValue: TStrings);
    procedure Setoptions(const AValue: TSQLScannerOptions);
    procedure ClearKeywords(Sender: TObject);
  protected
    Procedure BuildKeyWords; virtual;
    procedure Error(const Msg: string);overload;
    procedure Error(const Msg: string; Args: array of Const);overload;
  public
    constructor Create(ALineReader: TLineReader);
    constructor Create(AStream : TStream);
    destructor Destroy; override;
    procedure OpenFile(const AFilename: string);
    Function FetchToken: TSQLToken;
    Function IsEndOfLine : Boolean;
    Property Options : TSQLScannerOptions Read FOptions Write Setoptions;
    property SourceFile: TLineReader read FSourceFile;
    property CurFilename: string read FSourceFilename;
    property CurLine: string read FCurLine;
    property CurRow: Integer read FCurRow;
    property CurColumn: Integer read GetCurColumn;
    property CurToken: TSQLToken read FCurToken;
    property CurTokenString: string read FCurTokenString;
    Property ExcludeKeywords : TStrings Read GetExcludeKeywords Write SetExcludeKeywords;
    Property AlternateTerminator : String Read FAlternateTerminator Write FAlternateTerminator;
  end;


implementation

Var
  // Keeps track of identifiers used
  IdentifierTokens : array[FirstKeyword..LastKeyWord] of TSQLToken;
  IdentifierTokensOK : Boolean;

Resourcestring
  SErrUnknownToken = 'Unknown token: %s';

Procedure BuildIdentifierTokens;

Var
  T : TSQLToken;

begin
  For T:=FirstKeyword to LastKeyWord do
    IdentifierTokens[T]:=T;
  IdentifierTokensOK:=True;
end;

constructor TFileLineReader.Create(const AFilename: string);
begin
  inherited Create;
  Assign(FTextFile, AFilename);
  Reset(FTextFile);
  FileOpened := true;
end;

destructor TFileLineReader.Destroy;
begin
  if FileOpened then
    Close(FTextFile);
  inherited Destroy;
end;

function TFileLineReader.IsEOF: Boolean;
begin
  Result := EOF(FTextFile);
end;

function TFileLineReader.ReadLine: string;
begin
  ReadLn(FTextFile, Result);
end;

constructor TSQLScanner.Create(ALineReader: TLineReader);
begin
  inherited Create;
  FSourceFile := ALineReader;
  FKeywords:=TFPHashList.Create;
end;

constructor TSQLScanner.Create(AStream: TStream);
begin
  FSourceStream:=ASTream;
  FOwnSourceFile:=True;
  Create(TStreamLineReader.Create(AStream));
end;

destructor TSQLScanner.Destroy;
begin
  If FOwnSourceFile then
    FSourceFile.Free;
  FreeAndNil(FKeywords);
  inherited Destroy;
end;

procedure TSQLScanner.OpenFile(const AFilename: string);
begin
  FSourceFile := TFileLineReader.Create(AFilename);
  FOwnSourceFile:=True;
  FSourceFilename := AFilename;
end;


procedure TSQLScanner.Error(const Msg: string);
begin
  raise ESQLScannerError.Create(Msg);
end;

procedure TSQLScanner.Error(const Msg: string; Args: array of Const);
begin
  raise ESQLScannerError.CreateFmt(Msg, Args);
end;

function TSQLScanner.FetchLine: Boolean;
begin
  if FSourceFile.IsEOF then
  begin
    FCurLine := '';
    TokenStr := nil;
    Result := false;
  end else
  begin
    FCurLine := FSourceFile.ReadLine;
    TokenStr := PChar(CurLine);
    Result := true;
    Inc(FCurRow);
  end;
end;

function TSQLScanner.DoWhiteSpace : TSQLToken;

begin
  Result:=tsqlWhitespace;
  repeat
    Inc(TokenStr);
    if TokenStr[0] = #0 then
      if not FetchLine then
        begin
        FCurToken := Result;
        exit;
        end;
  until not (TokenStr[0] in [#9, ' ']);
end;

function TSQLScanner.DoSingleLineComment : TSQLToken;

Var
  TokenStart : PChar;
  Len : Integer;

begin
  Inc(TokenStr);
  TokenStart := TokenStr;
  while TokenStr[0] <> #0 do
     Inc(TokenStr);
  Len:=TokenStr-TokenStart;
  SetLength(FCurTokenString, Len);
  if (Len>0) then
    Move(TokenStart^,FCurTokenString[1],Len);
  Result := tsqlComment;
end;

function TSQLScanner.DoMultiLineComment : TSQLToken;

Var
  TokenStart : PChar;
  Len,OLen : Integer;
  PrevToken : Char;

begin
  Inc(TokenStr);
  TokenStart := TokenStr;
  FCurTokenString := '';
  OLen:= 0;
  PrevToken:=#0;
  while Not ((TokenStr[0]='/') and (PrevToken='*')) do
    begin
    if (TokenStr[0]=#0) then
      begin
      Len:=TokenStr-TokenStart+1;
      SetLength(FCurTokenString,OLen+Len);
      if Len>1 then
        Move(TokenStart^,FCurTokenString[OLen+1],Len-1);
      Inc(OLen,Len);
      FCurTokenString[OLen]:=#10;
      if not FetchLine then
        begin
        Result := tsqlEOF;
        FCurToken := Result;
        exit;
        end;
      TokenStart := TokenStr;
      PrevToken:=#0;
      end
    else
      begin
      PrevToken:=TokenStr[0];
      Inc(TokenStr);
      end;
    end;
  Len:=TokenStr-TokenStart-1; // -1 for *
  SetLength(FCurTokenString, Olen+Len);
  if (Len>0) then
    begin
    Move(TokenStart^, FCurTokenString[Olen + 1], Len);
    end;
  If TokenStr[0]<>#0 then
    Inc(TokenStr);
  Result := tsqlComment;
end;

function TSQLScanner.CommentDiv : TSQLToken;

begin
  FCurTokenString := '';
  Inc(TokenStr);
  if (TokenStr[0]='*') then
    Result:=DoMultiLineComment
  else
    Result:=tsqlDiv;
end;

Function TSQLScanner.ReadUnicodeEscape : WideChar;

Var
  S : String;
  I : Integer;

begin
  S:='0000';
  For I:=1 to 4 do
    begin
    Inc(TokenStr);
    Case TokenStr[0] of
      '0'..'9','A'..'F','a'..'f' :
        S[i]:=Upcase(TokenStr[0]);
    else
      Error(SErrInvalidCharacter, [TokenStr[0]]);
    end;
    end;
  // Takes care of conversion... This needs improvement !!
  Result:=WideChar(StrToInt('$'+S));
end;

procedure TSQLScanner.SetExcludeKeywords(const AValue: TStrings);
begin
  With ExcludeKeywords do
    begin
    Clear;
    AddStrings(AValue);
    end;
end;

procedure TSQLScanner.Setoptions(const AValue: TSQLScannerOptions);

Const
   F = [soDoubleQuoteStringLiteral,soSingleQuoteIdentifier];

begin
  FOptions:=AValue;
  if ((Foptions * F) = [soSingleQuoteIdentifier]) then
    Exclude(FOptions,soSingleQuoteIdentifier);
end;

procedure TSQLScanner.BuildKeyWords;

Var
  I : TSQLToken;

begin
  If Not IdentifierTokensOK then
    BuildIdentifierTokens;
  If FKeywords.Count>0 then
    FKeywords.Clear;
  for I:=FirstKeyword to LastKeyword do
    if (not Assigned(FExclude)) or (FExclude.IndexOf(TokenInfos[I])=-1) then
      FKeywords.Add(TokenInfos[I],@IdentifierTokens[i]);
end;

function TSQLScanner.DoStringLiteral: TSQLToken;

Var
  Delim : Char;
  TokenStart : PChar;
  Len,OLen : Integer;
  S : UnicodeString;

  Procedure AppendBufToTokenString(DoNextToken : Boolean);

  begin
    SetLength(FCurTokenString, OLen + Len+Length(S));
    if Len > 0 then
      Move(TokenStart^, FCurTokenString[OLen + 1], Len);
    If Length(S)>0 then
      Move(S[1],FCurTokenString[OLen + Len+1],Length(S));
    Inc(OLen, Len+Length(S));
    If DoNextToken then
      Inc(TokenStr);
    TokenStart := TokenStr+1;
  end;

  Function CheckTokenBuf : Boolean;

  begin
    Result:=(TokenStr[0]<>#0);
    If Not Result then
      begin
      S:='';
      Len:=TokenStr-TokenStart;
      AppendBufToTokenString(False);
      Result:=FetchLine;
      TokenStart:=TokenStr;
      end;
  end;

begin
  Delim:=TokenStr[0];
  Inc(TokenStr);
  TokenStart := TokenStr;
  OLen := 0;
  FCurTokenString := '';
  while not (TokenStr[0]=#0) do
    begin
    If (TokenStr[0]=Delim) then
      begin
      if (not (soNoDoubleDelimIsChar in options)) and (TokenStr[1]=Delim) then
        begin
        S:=Delim;
        Len := TokenStr - TokenStart;
        AppendBufToTokenString(True);
        end
      else
        Break;
      end
    else if (TokenStr[0]='\') and (soBackSlashEscapes in Options) then
      begin
      // Save length
      Len := TokenStr - TokenStart;
      Inc(TokenStr);
      if not CheckTokenBuf then
        Error(SErrOpenString);
      // Read escaped token
      Case TokenStr[0] of
        '"' : S:='"';
        '''' : S:='''';
        't' : S:=#9;
        'b' : S:=#8;
        'n' : S:=#10;
        'r' : S:=#13;
        'f' : S:=#12;
        '\' : S:='\';
        '/' : S:='/';
        'u' : begin
              S:=ReadUniCodeEscape;
              end;
      else
        Error(SErrInvalidCharacter, [TokenStr[0]]);
      end;
      AppendBufToTokenString(False);
      end;
    Inc(TokenStr);
    if not CheckTokenBuf then
      Error(SErrOpenString);
    end;
  if Not CheckTokenBuf then
    Error(SErrOpenString);
  S:='';
  Len := TokenStr - TokenStart;
  AppendBufToTokenString(True);
  Result := tsqlString;
end;

function TSQLScanner.DoNumericLiteral :TSQLToken;

Var
  TokenStart : PChar;
  Len : Integer;
  isFloat : boolean;

begin
  TokenStart := TokenStr;
  IsFloat:=False;
  while true do
    begin
    Inc(TokenStr);
    case TokenStr[0] of
      'x':
        If (TokenStart[0]='0') and ((TokenStr-TokenStart)=1) then
          begin
          Inc(TokenStr);
          while Upcase(TokenStr[0]) in ['0'..'9','A'..'F'] do
            Inc(TokenStr);
          end
        else
          Error(SInvalidHexadecimalNumber);
      '.':
        begin
          isfloat:=true;
          if TokenStr[1] in ['0'..'9', 'e', 'E'] then
          begin
            Inc(TokenStr);
            repeat
              Inc(TokenStr);
            until not (TokenStr[0] in ['0'..'9', 'e', 'E','-','+']);
          end;
          break;
        end;
      '0'..'9': ;
      'e', 'E':
        begin
          isFloat:=true;
          Inc(TokenStr);
          if TokenStr[0] in ['-','+']  then
            Inc(TokenStr);
          while TokenStr[0] in ['0'..'9'] do
            Inc(TokenStr);
          break;
        end;
      else
        break;
    end;
  end;
  Len:=TokenStr-TokenStart;
  Setlength(FCurTokenString, Len);
  if (Len>0) then
  Move(TokenStart^,FCurTokenString[1],Len);
  If IsFloat then
    Result := tsqlFloatNumber
  else
    Result:=tsqlIntegerNumber;
end;

function TSQLScanner.DoIdentifier : TSQLToken;

Var
  TokenStart:PChar;
  Len : Integer;
  {I : TSQLToken;}
  S : ShortString;
  P : ^TSQLToken;

begin
  Result:=tsqlIdentifier;
  TokenStart := TokenStr;
  repeat
    Inc(TokenStr);
    If (TokenStr[0]='\') and (TokenStr[1]='u') then
  until not (TokenStr[0] in ['A'..'Z', 'a'..'z', '0'..'9', '_','$']);
  Len:=(TokenStr-TokenStart);
  SetLength(FCurTokenString,Len);
  if Len > 0 then
    Move(TokenStart^,FCurTokenString[1],Len);
  S:=UpperCase(FCurTokenString);
  // Check if this is a keyword or identifier
  // to do: Optimize this!
  If FKeyWords.Count=0 then
    BuildKeyWords;
  P:=FKeyWords.Find(S);
  If (P<>Nil) then
    Result:=P^ //keyword found
  else if (AlternateTerminator<>'') and (S=AlternateTerminator) then
    Result:=tsqlTerminator;

  { I:=FirstKeyword;
  While (Result=tsqlIdentifier) and (I<=Lastkeyword) do
    begin
    if (S=TokenInfos[i]) then
      begin
      Result := i;
      FCurToken := Result;
      exit;
      end;
    I:=Succ(I);
    end;}
end;

function TSQLScanner.DoSymbolString : TSQLToken;

Var
  Len : Integer;
  P : ^TSQLToken;
  TokenStart : PChar;
begin
  Result:=tsqlUnknown;

  // Get "word" finalized by end of string, space/tab/line ending.
  TokenStart:=TokenStr;
  repeat
    Inc(TokenStr);
  until (TokenStr[0] in [#0, #9, #10, #13, ' ']);
  Len:=(TokenStr-TokenStart);
  if Len > 0 then
    begin
    result:=tsqlSymbolString;
    SetLength(FCurTokenString,Len);
    Move(TokenStart^,FCurTokenString[1],Len);
    if (AlternateTerminator<>'') and (CurtokenString=AlternateTerminator) then
      Exit(tsqlTerminator);

    // Check if this is a keyword or identifier/literal
    // Probably not (due to naming rules) but it doesn't hurt
    If FKeyWords.Count=0 then
      BuildKeyWords;
    P:=FKeyWords.Find(FCurTokenString); //case-sensitive search
    If (P<>Nil) then
      Result:=P^; //keyword found, just in case
    end;
end;

function TSQLScanner.FetchToken: TSQLToken;

begin
  Repeat
    if TokenStr = nil then
      if not FetchLine then
        begin
        Result := tsqlEOF;
        FCurToken := Result;
        exit;
        end;
    FCurTokenString := '';
    case TokenStr[0] of
      #0:         // Empty line
        begin
        FetchLine;
        Result := tsqlWhitespace;
        end;
      '/' :
         Result:=CommentDiv;
      #9, ' ',#10,#13:
         Result := DoWhiteSpace;
      '''':
        begin
        Result:=DoStringLiteral;
        if (soSingleQuoteIdentifier in Options) then
          result:=tsqlIdentifier;
        end;
      '"':
        begin
        Result:=DoStringLiteral;
        If (soDoubleQuoteStringLiteral in options) then
          Result:=tsqlString
        else
          Result:=tsqlIdentifier;
        end;
      '`':
        begin
        Result:=DoStringLiteral;
        If (soBackQuoteIdentifier in options) then
          Result:=tsqlIdentifier
        else
          Error(SErrUnknownToken,['`']);
        end;
      '0'..'9':
         Result:=DoNumericLiteral;
      '?':
         begin
         Inc(TokenStr);
         Result:=tsqlPlaceHolder;
         end;
      '!':
        begin
        Inc(TokenStr);
        If TokenStr[0]='>' then
          Result:=tsqlLE
        else if (TokenStr[0]='<') then
          Result:=tsqlGE
        else if (TokenStr[0]='=') then
          Result:=tsqlNE
        else
          Result:=tsqlUnknown;
        Inc(TokenStr);
        end;
     '|':
         begin
         Inc(TokenStr);
         If Tokenstr[0]='|' then
           begin
           Inc(TokenStr);
           Result := tsqlConcatenate
           end
         else
           Error(SBarExpected);
         end;
    '(':
      begin
      Inc(TokenStr);
      Result := tsqlBraceOpen;
      end;
    ')':
      begin
      Inc(TokenStr);
      Result := tsqlBraceClose;
      end;
    '[':
      begin
      Inc(TokenStr);
      Result := tsqlSquareBraceOpen;
      end;
    ']':
      begin
      Inc(TokenStr);
      Result := tsqlSquareBraceClose;
      end;
    '*':
      begin
      Inc(TokenStr);
      Result := tsqlMul;
      end;
    '+':
      begin
      Inc(TokenStr);
      Result := tsqlPlus;
      end;
    ',':
      begin
        Inc(TokenStr);
        Result := tsqlComma;
      end;
    '-':
      begin
      Inc(TokenStr);
      If (TokenStr[0]='-') then
        begin
        Inc(TokenStr);
        Result:=DoSingleLineComment
        end
      else if (TokenStr[0] in ['0'..'9']) then
        begin
        Result:=DoNumericLiteral;
        If (Result in [tsqlIntegerNumber,tsqlFloatNumber]) then
          FCurTokenString:='-'+FCurTokenString;
        end
      else
        Result := tsqlMinus;
      end;
    '.':
      begin
      Inc(TokenStr);
      Result := tsqlDot;
      end;
    ':':
      begin
      Inc(TokenStr);
      Result := tsqlColon;
      end;
    ';':
      begin
      Inc(TokenStr);
      Result := tsqlSemicolon;
      end;
    '<':
      begin
      Inc(TokenStr);
      if TokenStr[0] = '>' then
        begin
        Inc(TokenStr);
        Result := tsqlNE;
        end
      else if (TokenStr[0] = '=') then
        begin
        Inc(TokenStr);
        Result := tsqlLE;
        end
      else
        Result := tsqlLT;
      end;
    '=':
      begin
      Inc(TokenStr);
      Result := tsqleQ;
      end;
    '>':
      begin
      Inc(TokenStr);
      if TokenStr[0] = '=' then
	      begin
        Inc(TokenStr);
        Result:=tsqlGE;
        end
      else
        Result := tsqlGT;
      end;
   'a'..'z',
   'A'..'Z':
     Result:=DoIdentifier;
   else
     // Symbol of some sort
     Result:=DoSymbolString;
     //Error(SErrUnknownToken,[TokenStr[0]]);
   end; // Case
  Until (Not (Result in [tsqlComment,tsqlWhitespace])) or
        ((Result=tsqlComment) and (soReturnComments in options)) or
        ((Result=tsqlWhiteSpace) and (soReturnWhiteSpace in Options));
  FCurToken:=Result;
end;

function TSQLScanner.IsEndOfLine: Boolean;
begin
  Result:=(TokenStr=Nil) or (TokenStr[0] in [#0,#10,#13]);
end;

function TSQLScanner.GetCurColumn: Integer;
begin
  Result := TokenStr - PChar(FCurLine);
end;

Procedure TSQLScanner.ClearKeywords(Sender : TObject);

begin
  If Assigned(FKeywords) then
    FKeywords.Clear;
end;

function TSQLScanner.GetExcludeKeywords: TStrings;
begin
  If FExclude=Nil then
    begin
    FExclude:=TStringList.Create;
    FExclude.Duplicates:=dupIgnore;
    FExclude.Sorted:=true;
    FExclude.OnChange:=@ClearKeywords;
    end;
  Result:=FExclude;
end;

{ TStreamLineReader }

constructor TStreamLineReader.Create(AStream: TStream);
begin
  FStream:=AStream;
  FBufPos:=0;
  FBufLen:=0;
end;

function TStreamLineReader.IsEOF: Boolean;
begin
  Result:=(FBufPos>=FBufLen);
  If Result then
    begin
    FillBuffer;
    Result:=(FBufLen=0);
    end;
end;

procedure TStreamLineReader.FillBuffer;

begin
  FBufLen:=FStream.Read(Buffer,SizeOf(Buffer)-1);
  Buffer[FBufLen]:=0;
  FBufPos:=0;
end;

function TStreamLineReader.ReadLine: string;

Var
  FPos,OLen,Len: Integer;
  PRun : PByte;

begin
  FPos:=FBufPos;
  Result:='';
  Repeat
    PRun:=@Buffer[FBufPos];
    While (FBufPos<FBufLen) and Not (PRun^ in [10,13]) do
      begin
      Inc(PRun);
      Inc(FBufPos);
      end;
    If (FBufPos=FBufLen) then
      begin
      Len:=FBufPos-FPos;
      If (Len>0) then
        begin
        Olen:=Length(Result);
        SetLength(Result,OLen+Len);
        Move(Buffer[FPos],Result[OLen+1],Len);
        end;
      FillBuffer;
      FPos:=FBufPos;
      end;
  until (FBufPos=FBufLen) or (PRun^ in [10,13]);
  Len:=FBufPos-FPos+1;
  If (Len>0) then
    begin
    Olen:=Length(Result);
    SetLength(Result,OLen+Len);
    Move(Buffer[FPos],Result[OLen+1],Len);
    end;
  If (PRun^ in [10,13]) and (FBufPos<FBufLen) then
    begin
    Inc(FBufPos);
    // Check #13#10
    If (PRun^=13) then
      begin
      If (FBufPos=FBufLen) then
        FillBuffer;
      If (FBufPos<FBufLen) and (Buffer[FBufpos]=10) then
        begin
        Inc(FBufPos);
        Result:=Result+#10;
        end;
      end;
    end;
end;

end.
