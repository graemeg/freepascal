{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2018  Mattias Gaertner  mattias@freepascal.org

    Pascal to Javascript converter class.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

  Abstract:
    Logging to stdout or file.
    Filtering messages by number and type.
    Registering messages with number, pattern and type (error, warning, note, etc).
}
unit Pas2jsLogger;

{$mode objfpc}{$H+}

{$i pas2js_defines.inc}

interface

uses
  {$IFDEF Pas2JS}
  JS,
  {$IFDEF NodeJS}
  NodeJSFS,
  {$ENDIF}
  {$ENDIF}
  pas2jsutils,
  {$IFDEF HASFILESYSTEM}
  pas2jsfileutils,
  {$ENDIF}
  Types, Classes, SysUtils,
  PasTree, PScanner,
  jstree, jsbase, jswriter, fpjson;

const
  ExitCodeErrorInternal = 1; // internal error
  ExitCodeErrorInParams = 2; // error in command line parameters
  ExitCodeErrorInConfig = 3; // error in config file
  ExitCodeFileNotFound = 4;
  ExitCodeWriteError = 5;
  ExitCodeSyntaxError = 6;
  ExitCodeConverterError = 7;
  ExitCodePCUError = 8;
  ExitCodeToolError = 9;

const
  DefaultLogMsgTypes = [mtFatal..mtDebug]; // by default show everything

type
  {$IFDEF Pas2JS}

  { TPas2jsStream }

  TPas2jsStream = class
  public
    procedure Write(const s: string); virtual; abstract;
  end;

  { TPas2jsFileStream }

  TPas2jsFileStream = class(TPas2JSStream)
  public
    constructor Create(Filename: string; Mode: cardinal);
    destructor Destroy; override;
    procedure Write(const s: string); override;
  end;
const
  fmCreate        = $FF00;
  fmOpenRead      = 0;
  //fmOpenWrite     = 1;
  //fmOpenReadWrite = 2;
  { Share modes}
  //fmShareCompat    = $0000;
  //fmShareExclusive = $0010;
  //fmShareDenyWrite = $0020;
  //fmShareDenyRead  = $0030;
  fmShareDenyNone  = $0040;

  {$ELSE}
  TPas2jsStream = TStream;
  TPas2jsFileStream = TFileStream;
  {$ENDIF}

type

  { TPas2jsMessage }

  TPas2jsMessage = class
  public
    Number: integer;
    Typ: TMessageType;
    Pattern: string;
  end;

  TPas2jsLogEvent = Procedure (Sender : TObject; Const Msg : String) Of Object;


  { TConsoleFileWriter }

  TConsoleFileWriter = Class(TTextWriter)
  Public
    Constructor Create(aFileName : String); reintroduce;
    Function DoWrite(Const S : TJSWriterString) : Integer; override;
    Procedure Flush;
  end;

  { TPas2jsLogger }

  TPas2jsLogger = class
  private
    FDebugLog: TPas2JSStream;
    FEncoding: string;
    FIndent: integer;
    FLastMsgCol: integer;
    FLastMsgFile: string;
    FLastMsgLine: integer;
    FLastMsgNumber: integer;
    FLastMsgTxt: string;
    FLastMsgType: TMessageType;
    FLineLen: integer;
    FMsgNumberDisabled: TIntegerDynArray;// sorted ascending
    FMsg: TFPList; // list of TPas2jsMessage
    FOnFormatPath: TPScannerFormatPathEvent;
    FOnLog: TPas2jsLogEvent;
    FOutputFile: TTextWriter; // TFileWriter;
    FOutputFilename: string;
    FShowMsgNumbers: boolean;
    FShowMsgTypes: TMessageTypes;
    FSorted: boolean;
    {$IFDEF HasStdErr}
    FWriteMsgToStdErr: boolean;
    {$ENDIF}
    function GetMsgCount: integer;
    function GetMsgNumberDisabled(MsgNumber: integer): boolean;
    function GetMsgs(Index: integer): TPas2jsMessage; inline;
    function FindMsgNumberDisabled(MsgNumber: integer; FindInsertPos: boolean): integer;
    procedure SetEncoding(const AValue: string);
    procedure SetMsgNumberDisabled(MsgNumber: integer; AValue: boolean);
    procedure SetOutputFilename(AValue: string);
    procedure SetSorted(AValue: boolean);
    procedure DoLogRaw(const Msg: string; SkipEncoding : Boolean);
  Protected
    // so it can be overridden
    function CreateTextWriter(const aFileName: string): TTextWriter; virtual;
  public
    {$IFDEF EnableLogFile}
    LogFile: TStringList;
    procedure LogF(args: array of const);
    {$ENDIF}
    constructor Create;
    destructor Destroy; override;
    procedure RegisterMsg(MsgType: TMessageType; MsgNumber: integer; Pattern: string);
    function FindMsg(MsgNumber: integer; ExceptionOnNotFound: boolean): TPas2jsMessage;
    procedure Sort;
    procedure LogRaw(const Msg: string); overload;
    procedure LogRaw(Args: array of {$IFDEF Pas2JS}jsvalue{$ELSE}const{$ENDIF}); overload;
    procedure LogLn;
    procedure LogPlain(const Msg: string); overload;
    procedure LogPlain(Args: array of {$IFDEF Pas2JS}jsvalue{$ELSE}const{$ENDIF}); overload;
    procedure LogMsg(MsgNumber: integer;
      Args: array of {$IFDEF Pas2JS}jsvalue{$ELSE}const{$ENDIF};
      const Filename: string = ''; Line: integer = 0; Col: integer = 0;
      UseFilter: boolean = true);
    procedure Log(MsgType: TMessageType; Msg: string; MsgNumber: integer = 0;
      const Filename: string = ''; Line: integer = 0; Col: integer = 0;
      UseFilter: boolean = true);
    procedure LogMsgIgnoreFilter(MsgNumber: integer;
      Args: array of {$IFDEF Pas2JS}jsvalue{$ELSE}const{$ENDIF});
    procedure LogExceptionBackTrace(E: Exception);
    function MsgTypeToStr(MsgType: TMessageType): string;
    function GetMsgText(MsgNumber: integer;
      Args: array of {$IFDEF Pas2JS}jsvalue{$ELSE}const{$ENDIF}): string;
    function FormatMsg(MsgType: TMessageType; Msg: string; MsgNumber: integer = 0;
      const Filename: string = ''; Line: integer = 0; Col: integer = 0): string;
    function FormatJSONMsg(MsgType: TMessageType; Msg: string; MsgNumber: integer = 0;
      const Filename: string = ''; Line: integer = 0; Col: integer = 0): string;
    procedure OpenOutputFile;
    procedure Flush;
    procedure CloseOutputFile;
    procedure Reset;
    procedure ClearLastMsg;
    procedure OpenDebugLog;
    procedure CloseDebugLog;
    procedure DebugLogWriteLn(Msg: string); overload;
    function GetEncodingCaption: string;
    class function Concatenate(Args: array of {$IFDEF Pas2JS}jsvalue{$ELSE}const{$ENDIF}): string;
  public
    property Encoding: string read FEncoding write SetEncoding; // normalized
    property MsgCount: integer read GetMsgCount;
    property Msgs[Index: integer]: TPas2jsMessage read GetMsgs;
    property MsgNumberDisabled[MsgNumber: integer]: boolean read GetMsgNumberDisabled write SetMsgNumberDisabled;
    property OnFormatPath: TPScannerFormatPathEvent read FOnFormatPath write FOnFormatPath;
    property OutputFilename: string read FOutputFilename write SetOutputFilename;
    property ShowMsgNumbers: boolean read FShowMsgNumbers write FShowMsgNumbers;
    property ShowMsgTypes: TMessageTypes read FShowMsgTypes write FShowMsgTypes;
    {$IFDEF HasStdErr}
    property WriteMsgToStdErr: boolean read FWriteMsgToStdErr write FWriteMsgToStdErr;
    {$ENDIF}
    property Sorted: boolean read FSorted write SetSorted;
    property OnLog: TPas2jsLogEvent read FOnLog write FOnLog;
    property LastMsgType: TMessageType read FLastMsgType write FLastMsgType;
    property LastMsgFile: string read FLastMsgFile write FLastMsgFile;
    property LastMsgLine: integer read FLastMsgLine write FLastMsgLine;
    property LastMsgCol: integer read FLastMsgCol write FLastMsgCol;
    property LastMsgTxt: string read FLastMsgTxt write FLastMsgTxt;
    property LastMsgNumber: integer read FLastMsgNumber write FLastMsgNumber;
    property DebugLog: TPas2jsStream read FDebugLog write FDebugLog;
    property LineLen: integer read FLineLen write FLineLen; // used by LogPlainText
    property Indent: integer read FIndent write FIndent; // used by LogPlainText
  end;

function CompareP2JMessage(Item1, Item2: {$IFDEF Pas2JS}JSValue{$ELSE}Pointer{$ENDIF}): Integer;

function QuoteStr(const s: string; Quote: char = '"'): string;
function DeQuoteStr(const s: string; Quote: char = '"'): string;
function AsString(Element: TPasElement; Full: boolean = true): string; overload;
function AsString(Element: TJSElement): string; overload;
function DbgString(Element: TJSElement; Indent: integer): string; overload;
function DbgAsString(Element: TJSValue; Indent: integer): string; overload;
function DbgAsString(Element: TJSArrayLiteralElements; Indent: integer): string; overload;
function DbgAsString(Element: TJSObjectLiteralElements; Indent: integer): string; overload;
function DbgAsString(Element: TJSObjectLiteralElement; Indent: integer): string; overload;
{$IFDEF UsePChar}
function DbgHexMem(p: Pointer; Count: integer): string;
{$ENDIF}
function DbgStr(const s: string): string;

implementation

function CompareP2JMessage(Item1, Item2: {$IFDEF Pas2JS}JSValue{$ELSE}Pointer{$ENDIF}): Integer;
var
  Msg1: TPas2jsMessage absolute Item1;
  Msg2: TPas2jsMessage absolute Item2;
begin
  Result:=Msg1.Number-Msg2.Number;
end;

function QuoteStr(const s: string; Quote: char): string;
begin
  Result:={$IFDEF Pas2JS}SysUtils.QuotedStr{$ELSE}AnsiQuotedStr{$ENDIF}(S,Quote);
end;

function DeQuoteStr(const s: string; Quote: char): string;
begin
  Result:={$IFDEF Pas2JS}SysUtils.DeQuoteString{$ELSE}AnsiDequotedStr{$ENDIF}(S,Quote);
end;

function AsString(Element: TPasElement; Full: boolean): string;
begin
  if Element=nil then
    Result:='(no element)'
  else begin
    Result:=Element.GetDeclaration(Full);
  end;
end;

function AsString(Element: TJSElement): string;
var
  aTextWriter: TBufferWriter;
  aWriter: TJSWriter;
begin
  aTextWriter:=TBufferWriter.Create(120);
  aWriter:=TJSWriter.Create(aTextWriter);
  aWriter.WriteJS(Element);
  Result:=aTextWriter.AsString;
  aWriter.Free;
  aTextWriter.Free;
end;

function DbgString(Element: TJSElement; Indent: integer): string;
begin
  if Element=nil then
    Result:='(*no element*)'
  else if Element is TJSLiteral then
  begin
    Result:=DbgAsString(TJSLiteral(Element).Value,Indent+2);
  end else if Element is TJSPrimaryExpressionIdent then
  begin
    Result:=String(TJSPrimaryExpressionIdent(Element).Name);

  // array literal
  end else if Element is TJSArrayLiteral then
  begin
    Result:='['+DbgAsString(TJSArrayLiteral(Element).Elements,Indent+2)+']';

  // object literal
  end else if Element is TJSObjectLiteral then
  begin
    Result:='['+DbgAsString(TJSObjectLiteral(Element).Elements,Indent+2)+']';

  // arguments
  end else if Element is TJSArguments then
  begin
    Result:='('+DbgAsString(TJSArguments(Element).Elements,Indent+2)+')';

  // member
  end else if Element is TJSMemberExpression then
  begin
    Result:='('+DbgString(TJSMemberExpression(Element).MExpr,Indent+2)+')';
  // ToDo: TJSNewMemberExpression
  // ToDo: TJSDotMemberExpression
  // ToDo: TJSBracketMemberExpression

  // call
  end else if Element is TJSCallExpression then
  begin
    Result:=DbgString(TJSCallExpression(Element).Expr,Indent+2)
           +DbgString(TJSCallExpression(Element).Args,Indent+2);

  // unary
  end else if Element is TJSUnary then
  begin
    Result:=TJSUnary(Element).PrefixOperator
           +DbgString(TJSUnary(Element).A,Indent+2)
           +TJSUnary(Element).PostFixOperator;

  // binary
  end else if Element is TJSBinary then
  begin
    if Element is TJSStatementList then
    begin
      Result:=DbgString(TJSBinaryExpression(Element).A,Indent+2)+';'+LineEnding
             +StringOfChar(' ',Indent)+DbgString(TJSBinaryExpression(Element).B,Indent);
    end else if Element is TJSVariableDeclarationList then
    begin
      Result:=DbgString(TJSBinaryExpression(Element).A,Indent+2)+';'+LineEnding
             +StringOfChar(' ',Indent)+DbgString(TJSBinaryExpression(Element).B,Indent);
    end else if Element is TJSWithStatement then
    begin
      Result:='with ('+DbgString(TJSBinaryExpression(Element).A,Indent+2)+'){'+LineEnding
             +StringOfChar(' ',Indent)+DbgString(TJSBinaryExpression(Element).B,Indent+2)+LineEnding
             +StringOfChar(' ',Indent)+'}';
    end else if Element is TJSBinaryExpression then
    begin
      Result:=DbgString(TJSBinaryExpression(Element).A,Indent+2);
      if TJSBinaryExpression(Element).AllowCompact then
        Result+=TJSBinaryExpression(Element).OperatorString
      else
        Result+=' '+TJSBinaryExpression(Element).OperatorString+' ';
      Result+=DbgString(TJSBinaryExpression(Element).B,Indent+2);
    end else begin
      Result:='{: unknown binary Element: '+Element.Classname+':}';
    end;

  // ? :
  end else if Element is TJSConditionalExpression then
  begin
    Result:=DbgString(TJSConditionalExpression(Element).A,Indent+2)
           +'?'+DbgString(TJSConditionalExpression(Element).B,Indent+2)
           +':'+DbgString(TJSConditionalExpression(Element).C,Indent+2);

  // assignment
  end else if Element is TJSAssignStatement then
  begin
    Result:=DbgString(TJSAssignStatement(Element).LHS,Indent+2)
           +TJSAssignStatement(Element).OperatorString
           +DbgString(TJSAssignStatement(Element).Expr,Indent+2);

  // var
  end else if Element is TJSVarDeclaration then
  begin
    Result:=TJSVarDeclaration(Element).Name;
    if TJSVarDeclaration(Element).Init<>nil then
      Result+='='+DbgString(TJSVarDeclaration(Element).Init,Indent+2);

  // if(){} else {}
  end else if Element is TJSIfStatement then
  begin
    Result:='if('+DbgString(TJSIfStatement(Element).Cond,Indent+2)+'){'+LineEnding
       +StringOfChar(' ',Indent+2)+DbgString(TJSIfStatement(Element).BTrue,Indent+2)+LineEnding
       +StringOfChar(' ',Indent);
    if TJSIfStatement(Element).BFalse<>nil then
      Result+=' else {'+LineEnding
         +StringOfChar(' ',Indent+2)+DbgString(TJSIfStatement(Element).BFalse,Indent+2)+LineEnding
         +StringOfChar(' ',Indent)+'}';

  // body
  end else if Element is TJSBodyStatement then
  begin

    // while(){}
    if Element is TJSWhileStatement then
    begin
      Result:='while('+DbgString(TJSWhileStatement(Element).Cond,Indent+2)+')';
      if TJSWhileStatement(Element).Body<>nil then
        Result+=DbgString(TJSWhileStatement(Element).Body,Indent)
      else
        Result+='{}';

    // do{}while()
    end else if Element is TJSDoWhileStatement then
    begin
      Result:='do';
      if TJSDoWhileStatement(Element).Body<>nil then
        Result+=DbgString(TJSDoWhileStatement(Element).Body,Indent)
      else
        Result+='{}';
      Result+='('+DbgString(TJSDoWhileStatement(Element).Cond,Indent+2)+')';

    // for(Init;Incr;Cond)Body
    end else if Element is TJSForStatement then
    begin
      Result:='for(';
      if TJSForStatement(Element).Init<>nil then
        Result+=DbgString(TJSForStatement(Element).Init,Indent+2);
      Result+=';';
      if TJSForStatement(Element).Cond<>nil then
        Result+=DbgString(TJSForStatement(Element).Cond,Indent+2);
      Result+=';';
      if TJSForStatement(Element).Incr<>nil then
        Result+=DbgString(TJSForStatement(Element).Incr,Indent+2);
      Result+=')';
      if TJSForStatement(Element).Body<>nil then
        Result+=DbgString(TJSForStatement(Element).Body,Indent)
      else
        Result+='{}';

    // {}
    end else begin
      if TJSBodyStatement(Element).Body<>nil then
        Result+='{'+LineEnding
          +StringOfChar(' ',Indent+2)+DbgString(TJSBodyStatement(Element).Body,Indent+2)+LineEnding
          +StringOfChar(' ',Indent)+'}'
      else
        Result+='{}';
    end;

  end else begin
    Result:='{: unknown Element: '+Element.Classname+':}';
  end;
end;

function DbgAsString(Element: TJSValue; Indent: integer): string;
begin
  if Element=nil then
    Result:='(no value)'
  else begin
    case Element.ValueType of
    jstUNDEFINED: Result:='undefined';
    jstNull: Result:='null';
    jstBoolean: Result:=BoolToStr(Element.AsBoolean,'true','false');
    jstNumber: str(Element.AsNumber,Result);
    jstString: Result:=QuoteStr(String(Element.AsString),'''');
    jstObject: Result:='{:OBJECT:}';
    jstReference: Result:='{:REFERENCE:}';
    JSTCompletion: Result:='{:COMPLETION:}';
    else Result:='{:Unknown ValueType '+IntToStr(ord(Element.ValueType))+':}';
    end;
  end;
  Result:=StringOfChar(' ',Indent)+Result;
end;

function DbgAsString(Element: TJSArrayLiteralElements; Indent: integer): string;
var
  i: Integer;
begin
  Result:='';
  for i:=0 to TJSArrayLiteralElements(Element).Count-1 do begin
    if i>0 then Result+=',';
    Result+=DbgString(TJSArrayLiteralElements(Element).Elements[i].Expr,Indent+2);
  end;
end;

function DbgAsString(Element: TJSObjectLiteralElements; Indent: integer): string;
var
  i: Integer;
begin
  Result:='';
  for i:=0 to TJSObjectLiteralElements(Element).Count-1 do begin
    if i>0 then Result+=',';
    Result+=DbgString(TJSObjectLiteralElements(Element).Elements[i].Expr,Indent+2);
  end;
end;

function DbgAsString(Element: TJSObjectLiteralElement; Indent: integer): string;
begin
  Result:=String(TJSObjectLiteralElement(Element).Name)
          +':'+DbgString(TJSObjectLiteralElement(Element).Expr,Indent+2);
end;

{$IFDEF UsePChar}
function DbgHexMem(p: Pointer; Count: integer): string;
var
  i: Integer;
begin
  Result:='';
  for i:=0 to Count-1 do
    Result:=Result+HexStr(ord(PChar(p)[i]),2);
end;
{$ENDIF}

function DbgStr(const s: string): string;
var
  i: Integer;
  c: Char;
begin
  Result:='';
  for i:=1 to length(s) do begin
    c:=s[i];
    case c of
    #0..#31,#127..#255: Result+='$'+HexStr(ord(c),2);
    else Result+=c;
    end;
  end;
end;

{ TConsoleFileWriter }

constructor TConsoleFileWriter.Create(aFileName: String);
begin
  Inherited Create;
  Write('Opening console log: '+aFileName);
end;

Function TConsoleFileWriter.DoWrite(Const S : TJSWriterString) : Integer;

begin
  Result:=Length(S);
  {AllowWriteln}
  Writeln(S);
  {AllowWriteln-}
end;

procedure TConsoleFileWriter.FLush;

begin
end;


{$IFDEF Pas2JS}
{ TPas2jsFileStream }

constructor TPas2jsFileStream.Create(Filename: string; Mode: cardinal);
begin
  {AllowWriteln}
  writeln('TPas2JSFileStream.Create TODO ',Filename,' Mode=',Mode);
  {AllowWriteln-}
  raise Exception.Create('TPas2JSFileStream.Create');
end;

destructor TPas2jsFileStream.Destroy;
begin
  {AllowWriteln}
  writeln('TPas2JSFileStream.Destroy TODO');
  {AllowWriteln-}
  raise Exception.Create('TPas2JSFileStream.Destroy');
  inherited Destroy;
end;

procedure TPas2jsFileStream.Write(const s: string);
begin
  {AllowWriteln}
  writeln('TPas2JSFileStream.Write TODO s="',s,'"');
  {AllowWriteln-}
  raise Exception.Create('TPas2JSFileStream.Write');
end;
{$ENDIF}

{ TPas2jsLogger }

function TPas2jsLogger.GetMsgs(Index: integer): TPas2jsMessage;
begin
  Result:=TPas2jsMessage(FMsg[Index]);
end;

function TPas2jsLogger.FindMsgNumberDisabled(MsgNumber: integer;
  FindInsertPos: boolean): integer;
var
  l, r, m, CurMsgNumber: Integer;
begin
  l:=0;
  r:=length(FMsgNumberDisabled)-1;
  m:=0;
  while l<=r do begin
    m:=(l+r) div 2;
    CurMsgNumber:=FMsgNumberDisabled[m];
    if MsgNumber<CurMsgNumber then
      r:=m-1
    else if MsgNumber>CurMsgNumber then
      l:=m+1
    else
      exit(m);
  end;
  if FindInsertPos then
  begin
    Result:=m;
    if l>m then inc(Result);
  end else begin
    Result:=-1;
  end;
end;

procedure TPas2jsLogger.SetEncoding(const AValue: string);
var
  NewValue: String;
begin
  {$IFDEF Pas2JS}
  NewValue:=Trim(lowercase(AValue));
  {$ELSE}
  NewValue:=NormalizeEncoding(AValue);
  {$ENDIF}
  if FEncoding=NewValue then Exit;
  //LogPlain(ClassName+': Encoding changed from "'+FEncoding+'" to "'+NewValue+'"');
  FEncoding:=NewValue;
end;

function TPas2jsLogger.GetMsgNumberDisabled(MsgNumber: integer): boolean;
begin
  Result:=FindMsgNumberDisabled(MsgNumber,false)>=0;
end;

procedure TPas2jsLogger.SetMsgNumberDisabled(MsgNumber: integer; AValue: boolean
  );
  {$IF defined(FPC) and (FPC_FULLVERSION<30101)}
  procedure Delete(var A: TIntegerDynArray; Index, Count: integer); overload;
  var
    i: Integer;
  begin
    for i:=Index+Count to length(A)-1 do
      A[i-Count]:=A[i];
    SetLength(A,length(A)-Count);
  end;

  procedure Insert(Item: integer; var A: TIntegerDynArray; Index: integer); overload;
  var
    i: Integer;
  begin
    SetLength(A,length(A)+1);
    for i:=length(A)-1 downto Index+1 do
      A[i]:=A[i-1];
    A[Index]:=Item;
  end;
  {$ENDIF}
var
  InsertPos, OldCount: Integer;
begin
  OldCount:=length(FMsgNumberDisabled);
  if AValue then
  begin
    // enable
    InsertPos:=FindMsgNumberDisabled(MsgNumber,true);
    if (InsertPos<OldCount) and (FMsgNumberDisabled[InsertPos]=MsgNumber) then
      exit; // already disabled
    // insert into array
    Insert(MsgNumber,FMsgNumberDisabled,InsertPos);
  end else begin
    // disable
    InsertPos:=FindMsgNumberDisabled(MsgNumber,false);
    if InsertPos<0 then exit;
    // delete from array
    Delete(FMsgNumberDisabled,InsertPos,1);
  end;
end;

procedure TPas2jsLogger.SetOutputFilename(AValue: string);
begin
  if FOutputFilename=AValue then Exit;
  CloseOutputFile;
  FOutputFilename:=AValue;
  if OutputFilename<>'' then
    OpenOutputFile;
end;

procedure TPas2jsLogger.SetSorted(AValue: boolean);
begin
  if FSorted=AValue then Exit;
  FSorted:=AValue;
  if FSorted then Sort;
end;

procedure TPas2jsLogger.DoLogRaw(const Msg: string; SkipEncoding : Boolean);
var
  S: String;
begin
  if SkipEncoding then
    S:=Msg
  else begin
    {$IFDEF FPC_HAS_CPSTRING}
    if (Encoding='utf8') or (Encoding='json') then
      S:=Msg
    else if Encoding='console' then
      S:=UTF8ToConsole(Msg)
    else if Encoding='system' then
      S:=UTF8ToSystemCP(Msg)
    else begin
      // default: write UTF-8 to outputfile and console codepage to console
      if FOutputFile=nil then
        S:=UTF8ToConsole(Msg);
    end;
    {$ELSE}
    S:=Msg;
    {$ENDIF}
  end;
  //writeln('TPas2jsLogger.LogPlain "',Encoding,'" "',DbgStr(S),'"');
  if DebugLog<>nil then
    DebugLogWriteLn(S);
  if FOnLog<>Nil then
    FOnLog(Self,S)
  else if FOutputFile<>nil then
    FOutputFile.Write(S+LineEnding)
  else begin
    {$IFDEF FPC_HAS_CPSTRING}
    // prevent codepage conversion magic
    SetCodePage(RawByteString(S), CP_OEMCP, False);
    {$ENDIF}
    {AllowWriteln}
    {$IFDEF HasStdErr}
    if WriteMsgToStdErr then
      writeln(StdErr,S)
    else
    {$ENDIF}
      writeln(S);
    {AllowWriteln-}
  end;
end;

constructor TPas2jsLogger.Create;
begin
  FMsg:=TFPList.Create;
  FShowMsgTypes:=DefaultLogMsgTypes;
  FLineLen:=78;
  FIndent:=2;
end;

destructor TPas2jsLogger.Destroy;
var
  i: Integer;
begin
  CloseOutputFile;
  CloseDebugLog;
  for i:=0 to FMsg.Count-1 do
    TObject(FMsg[i]).{$IFDEF Pas2JS}Destroy{$ELSE}Free{$ENDIF};
  FreeAndNil(FMsg);
  FMsgNumberDisabled:=nil;
  inherited Destroy;
end;

procedure TPas2jsLogger.RegisterMsg(MsgType: TMessageType; MsgNumber: integer;
  Pattern: string);
var
  Msg: TPas2jsMessage;
begin
  if MsgNumber=0 then
    raise Exception.Create('internal error: TPas2jsLogger.RegisterMsg MsgNumber=0');
  Msg:=TPas2jsMessage.Create;
  Msg.Number:=MsgNumber;
  Msg.Typ:=MsgType;
  Msg.Pattern:=Pattern;
  FMsg.Add(Msg);
  FSorted:=false;
end;

function TPas2jsLogger.GetMsgCount: integer;
begin
  Result:=FMsg.Count;
end;

function TPas2jsLogger.FindMsg(MsgNumber: integer; ExceptionOnNotFound: boolean
  ): TPas2jsMessage;
var
  l, r, m: Integer;
  Msg: TPas2jsMessage;
begin
  if not FSorted then Sort;
  l:=0;
  r:=GetMsgCount-1;
  while l<=r do begin
    m:=(l+r) div 2;
    Msg:=Msgs[m];
    if MsgNumber<Msg.Number then
      r:=m-1
    else if MsgNumber>Msg.Number then
      l:=m+1
    else
      exit(Msg);
  end;
  Result:=nil;
  if ExceptionOnNotFound then
    raise Exception.Create('invalid message number '+IntToStr(MsgNumber));
end;

procedure TPas2jsLogger.Sort;
var
  i: Integer;
  LastMsg, CurMsg: TPas2jsMessage;
begin
  if FMsg.Count>1 then
  begin;
    FMsg.Sort(@CompareP2JMessage);

    // check for duplicates
    LastMsg:=TPas2jsMessage(FMsg[0]);
    for i:=1 to FMsg.Count-1 do begin
      CurMsg:=TPas2jsMessage(FMsg[i]);
      if LastMsg.Number=CurMsg.Number then
        raise Exception.Create('duplicate message number '+IntToStr(CurMsg.Number)+'. 1st="'+LastMsg.Pattern+'" 2nd="'+CurMsg.Pattern+'"');
      LastMsg:=CurMsg;
    end;
  end;
  FSorted:=true;
end;

function TPas2jsLogger.GetMsgText(MsgNumber: integer;
  Args: array of {$IFDEF Pas2JS}jsvalue{$ELSE}const{$ENDIF}): string;
var
  Msg: TPas2jsMessage;
begin
  Msg:=FindMsg(MsgNumber,true);
  Result:=MsgTypeToStr(Msg.Typ)+': '+Format(Msg.Pattern,Args);
end;

procedure TPas2jsLogger.LogRaw(const Msg: string);
begin
  ClearLastMsg;
  DoLogRaw(Msg,False);
end;

procedure TPas2jsLogger.LogRaw(
  Args: array of {$IFDEF Pas2JS}jsvalue{$ELSE}const{$ENDIF});
begin
  LogRaw(Concatenate(Args));
end;

procedure TPas2jsLogger.LogLn;
begin
  LogRaw('');
end;

procedure TPas2jsLogger.DebugLogWriteLn(Msg: string);
begin
  if FDebugLog=nil then exit;
  Msg:=Msg+LineEnding;
  {$IFDEF Pas2JS}
  FDebugLog.Write(Msg);
  {$ELSE}
  FDebugLog.Write(Msg[1],length(Msg));
  {$ENDIF}
end;

function TPas2jsLogger.GetEncodingCaption: string;
begin
  Result:=Encoding;
  if Result='' then
  begin
    {$IFDEF FPC_HAS_CPSTRING}
    if FOutputFile=nil then
      Result:='console'
    else
    {$ENDIF}
      Result:='utf-8';
  end;
  if Result='console' then
  begin
    {$IFDEF Unix}
    if not IsNonUTF8System then
      Result:='utf-8';
    {$ENDIF}
  end;
  if Result='utf8' then
    Result:='utf-8';
end;

class function TPas2jsLogger.Concatenate(
  Args: array of {$IFDEF Pas2JS}jsvalue{$ELSE}const{$ENDIF}): string;
var
  s: String;
  i: Integer;
  {$IFDEF Pas2JS}
  V: JSValue;
  {$ELSE}
  V: TVarRec;
  {$ENDIF}
begin
  s:='';
  for i:=Low(Args) to High(Args) do
  begin
    V:=Args[i];
    {$IFDEF Pas2JS}
    case jsTypeOf(V) of
    'boolean':
      if V then s+='true' else s+='false';
    'number':
      if isInteger(V) then
        s+=str(NativeInt(V))
      else
        s+=str(Double(V));
    'string':
      s+=String(V);
    else continue;
    end;
    {$ELSE}
    case V.VType of
      vtInteger:      s += IntToStr(V.VInteger);
      vtBoolean:      s += BoolToStr(V.VBoolean);
      vtChar:         s += V.VChar;
      {$ifndef FPUNONE}
      vtExtended:     ; //  V.VExtended^;
      {$ENDIF}
      vtString:       s += V.VString^;
      vtPointer:      ; //  V.VPointer;
      vtPChar:        s += V.VPChar;
      vtObject:       ; //  V.VObject;
      vtClass:        ; //  V.VClass;
      vtWideChar:     s += AnsiString(V.VWideChar);
      vtPWideChar:    s += AnsiString(V.VPWideChar);
      vtAnsiString:   s += AnsiString(V.VAnsiString);
      vtCurrency:     ; //  V.VCurrency^);
      vtVariant:      ; //  V.VVariant^);
      vtInterface:    ; //  V.VInterface^);
      vtWidestring:   s += AnsiString(WideString(V.VWideString));
      vtInt64:        s += IntToStr(V.VInt64^);
      vtQWord:        s += IntToStr(V.VQWord^);
      vtUnicodeString:s += AnsiString(UnicodeString(V.VUnicodeString));
    end;
    {$ENDIF}
  end;
  Result:=s;
end;

procedure TPas2jsLogger.LogPlain(const Msg: string);
var
  s: String;
begin
  ClearLastMsg;
  if Encoding='json' then
  begin
    s:=FormatJSONMsg(mtInfo,Msg,0,'',0,0);
    DoLogRaw(s,True);
  end else
    DoLogRaw(Msg,False);
end;

procedure TPas2jsLogger.LogPlain(
  Args: array of {$IFDEF Pas2JS}jsvalue{$ELSE}const{$ENDIF});
begin
  LogPlain(Concatenate(Args));
end;

procedure TPas2jsLogger.LogMsg(MsgNumber: integer;
  Args: array of {$IFDEF Pas2JS}jsvalue{$ELSE}const{$ENDIF};
  const Filename: string; Line: integer; Col: integer; UseFilter: boolean);
var
  Msg: TPas2jsMessage;
begin
  Msg:=FindMsg(MsgNumber,true);
  Log(Msg.Typ,SafeFormat(Msg.Pattern,Args),MsgNumber,Filename,Line,Col,UseFilter);
end;

procedure TPas2jsLogger.Log(MsgType: TMessageType; Msg: string;
  MsgNumber: integer; const Filename: string; Line: integer; Col: integer;
  UseFilter: boolean);
var
  s: String;
begin
  if UseFilter and not (MsgType in FShowMsgTypes) then exit;
  if MsgNumberDisabled[MsgNumber] then exit;
  if encoding='json' then
    s:=FormatJSONMsg(MsgType,Msg,MsgNumber,Filename,Line,Col)
  else
    s:=FormatMsg(MsgType,Msg,MsgNumber,Filename,Line,Col);
  FLastMsgType:=MsgType;
  FLastMsgNumber:=MsgNumber;
  FLastMsgTxt:=Msg;
  FLastMsgFile:=Filename;
  FLastMsgLine:=Line;
  FLastMsgCol:=Col;
  DoLogRaw(s,False);
end;

procedure TPas2jsLogger.LogMsgIgnoreFilter(MsgNumber: integer;
  Args: array of {$IFDEF Pas2JS}jsvalue{$ELSE}const{$ENDIF});
begin
  LogMsg(MsgNumber,Args,'',0,0,false);
end;

procedure TPas2jsLogger.LogExceptionBackTrace(E: Exception);
{$IFDEF Pas2js}
begin
  {$IFDEF NodeJS}
  if (E<>nil) and (E.NodeJSError<>nil) then
    {AllowWriteln}
    writeln(E.NodeJSError.Stack);
    {AllowWriteln-}
  {$ENDIF}
end;
{$ELSE}
var
  lErrorAddr: CodePointer;
  FrameCount: LongInt;
  Frames: PCodePointer;
  FrameNumber: Integer;
begin
  lErrorAddr:=ExceptAddr;
  FrameCount:=ExceptFrameCount;
  Frames:=ExceptFrames;
  Log(mtDebug,BackTraceStrFunc(lErrorAddr));
  for FrameNumber := 0 to FrameCount-1 do
    Log(mtDebug,BackTraceStrFunc(Frames[FrameNumber]));
  if E=nil then ;
end;
{$ENDIF}

function TPas2jsLogger.MsgTypeToStr(MsgType: TMessageType): string;
begin
  case MsgType of
  mtFatal: Result:='Fatal';
  mtError: Result:='Error';
  mtWarning: Result:='Warning';
  mtNote: Result:='Note';
  mtHint: Result:='Hint';
  mtInfo: Result:='Info';
  mtDebug: Result:='Debug';
  else Result:='Verbose';
  end;
end;

function TPas2jsLogger.FormatMsg(MsgType: TMessageType; Msg: string;
  MsgNumber: integer; const Filename: string; Line: integer; Col: integer
  ): string;
// e.g. file(line,col) type: (number) msg
var
  s: String;
begin
  s:='';
  if Filename<>'' then
  begin
    if Assigned(OnFormatPath) then
      s+=OnFormatPath(Filename)
    else
      s+=Filename;
    if Line>0 then
    begin
      s+='('+IntToStr(Line);
      if Col>0 then s+=','+IntToStr(Col);
      s+=')';
    end;
    if s<>'' then s+=' ';
  end;
  s+=MsgTypeToStr(MsgType)+': ';
  if ShowMsgNumbers and (MsgNumber<>0) then
    s+='('+IntToStr(MsgNumber)+') ';
  s+=Msg;
  Result:=s;
end;

function TPas2jsLogger.FormatJSONMsg(MsgType: TMessageType; Msg: string; MsgNumber: integer; const Filename: string; Line: integer;
  Col: integer): string;

Var
  J : TJSONObject;
  FN : String;

begin
  if Assigned(OnFormatPath) then
    FN:=OnFormatPath(Filename)
  else
    FN:=Filename;
  J:=TJSONObject.Create([
    'message',Msg,
    'line',Line,
    'col',Col,
    'number',MsgNumber,
    'filename',FN,
    'type',MsgTypeToStr(MsgType)
    ]);
  try
    Result:=J.AsJSON;
  finally
    J.Free;
  end;
end;

function TPas2jsLogger.CreateTextWriter(const aFileName: string): TTextWriter;

begin
{$IFDEF HASFILESYSTEM}
  Result:=TFileWriter.Create(aFilename);
{$ELSE}
  Result:=TConsoleFileWriter.Create(aFileName);
{$ENDIF}
end;

{$IFDEF EnableLogFile}
procedure TPas2jsLogger.LogF(args: array of const);
begin
  if LogFile=nil then
    LogFile:=TStringList.Create;
  LogFile.Add(TPas2jsLogger.Concatenate(args));
  LogFile.SaveToFile('c:\tmp\libpas2jsparams.txt');
end;
{$ENDIF}

procedure TPas2jsLogger.OpenOutputFile;
begin
{$IFDEF HASFILESYSTEM}
  if FOutputFile<>nil then exit;
  if OutputFilename='' then
    raise Exception.Create('Log has empty OutputFilename');
  if DirectoryExists(OutputFilename) then
    raise Exception.Create('Log is directory: "'+OutputFilename+'"');
{$ENDIF}
  FOutputFile:=CreateTextWriter(OutputFileName);
  {$IFDEF FPC_HAS_CPSTRING}
  if (Encoding='') or (Encoding='utf8') then
    FOutputFile.Write(UTF8BOM);
  {$ENDIF}
end;

procedure TPas2jsLogger.Flush;
begin
{$IFDEF HASFILESYSTEM}
  if Assigned(FOutputFile) and (FoutputFile is TFileWriter) then
    TFileWriter(FOutputFile).Flush;
{$ENDIF}
end;

procedure TPas2jsLogger.CloseOutputFile;
begin
  if FOutputFile=nil then exit;
  Flush;
  FreeAndNil(FOutputFile);
end;

procedure TPas2jsLogger.Reset;
begin
  OutputFilename:='';
  FMsgNumberDisabled:=nil;
  ShowMsgNumbers:=false;
  FShowMsgTypes:=DefaultLogMsgTypes;
end;

procedure TPas2jsLogger.ClearLastMsg;
begin
  FLastMsgType:=mtInfo;
  FLastMsgNumber:=0;
  FLastMsgTxt:='';
  FLastMsgFile:='';
  FLastMsgLine:=0;
  FLastMsgCol:=0;
end;

procedure TPas2jsLogger.OpenDebugLog;
const
  DbgLogFilename = 'pas2jsdebug.log';
begin
  FDebugLog:=TPas2jsFileStream.Create(DbgLogFilename,fmCreate or fmShareDenyNone);
end;

procedure TPas2jsLogger.CloseDebugLog;
begin
  FreeAndNil(FDebugLog);
end;

end.

