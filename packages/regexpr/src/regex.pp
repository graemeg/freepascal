{
    This file is part of the Free Pascal packages library.
    Copyright (c) 2008 by Joost van der Sluis, member of the
    Free Pascal development team
    
    Regexpression parser
    
    This code is based on the examples in the book
    'Tomes of Delphi: Algorithms and Data Structures' by Julian M Bucknall
    The code is used with his permission. For an excellent explanation of
    this unit, see the book...

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit Regex;

{$mode Delphi}{$H+}
{$INLINE ON}

interface

{Notes:
  these classes parse regular expressions that follow this grammar:

    <anchorexpr> ::= <expr> |
                     '^' <expr> |
                     <expr> '$' |
                     '^' <expr> '$'
    <expr> ::= <term> |
               <term> '|' <expr>                 - alternation
    <term> ::= <factor> |
               <factor><term>                    - concatenation
    <factor> ::= <atom> |
                 <atom> '?' |                    - zero or one
                 <atom> '*' |                    - zero or more
                 <atom> 'n,m' |                  - min n, max m (added by Joost)
                 <atom> '+'                      - one or more
    <atom> ::= <char> |
               '.' |                             - any char
               '(' <expr> ') |                   - parentheses
               '[' <charclass> ']' |             - normal class
               '[^' <charclass> ']'              - negated class
    <charclass> ::= <charrange> |
                    <charrange><charclass>
    <charrange> ::= <ccchar> |
                    <ccchar> '-' <ccchar>
    <char> ::= <any character except metacharacters> |
               '\' <any character at all>
    <ccchar> ::= <any character except '-' and ']'> |
                 '\' <any character at all>

  This means that parentheses have maximum precedence, followed
  by square brackets, followed by the closure operators,
  followed by concatenation, finally followed by alternation.
}

uses
  SysUtils,
  Classes;

type
  TUpcaseFunc = function(aCh : AnsiChar) : AnsiChar;

  TNFAMatchType = (  {types of matching performed...}
     mtNone,           {..no match (an epsilon no-cost move)}
     mtAnyChar,        {..any character}
     mtChar,           {..a particular character}
     mtClass,          {..a character class}
<<<<<<< HEAD
<<<<<<< HEAD
     mtDupClass,       {..a character class beying referenced}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     mtNegClass,       {..a negated character class}
     mtTerminal,       {..the final state--no matching}
     mtUnused);        {..an unused state--no matching}

  TRegexError = (    {error codes for invalid regex strings}
     recNone,          {..no error}
     recSuddenEnd,     {..unexpected end of string}
     recMetaChar,      {..read metacharacter, but needed normal char}
     recNoCloseParen,  {..expected close paren, but not there}
     recExtraChars     {..not at end of string after parsing regex}
     );

  TRegexType = (
     rtRegEx,
     rtChars,
     rtSingleChar
     );

  PCharSet = ^TCharSet;
  TCharSet = set of Char;

  { TtdRegexEngine }

  TNFAState = record
    sdNextState1: integer;
    sdNextState2: integer;
    sdClass     : PCharSet;
    sdMatchType : TNFAMatchType;
    sdChar      : AnsiChar;
  end;


  { TRegexEngine }

  TRegexEngine = class
    private
      FAnchorEnd  : boolean;
      FAnchorStart: boolean;
      FErrorCode  : TRegexError;
      FIgnoreCase : boolean;
      FMultiLine  : boolean;
      FPosn       : PAnsiChar;
      FRegexStr   : string;
      FStartState : integer;
      FStateTable : Array of TNFAState;
      FStateCount : integer;
      FUpcase     : TUpcaseFunc;

      // The deque (double-ended queue)
      FList : array of integer;
      FCapacity : integer;
      FHead : integer;
      FTail : integer;
      
      FRegexType : TRegexType;
    protected
      procedure DequeEnqueue(aValue : integer);
      procedure DequePush(aValue : integer);
      function DequePop : integer;
      procedure DequeGrow;

      procedure rcSetIgnoreCase(aValue : boolean); virtual;
      procedure rcSetRegexStr(const aRegexStr : string); virtual;
      procedure rcSetUpcase(aValue : TUpcaseFunc); virtual;
      procedure rcSetMultiLine(aValue : Boolean); virtual;

      procedure rcClear; virtual;
      procedure rcError(aIndex      : integer); virtual;
      procedure rcLevel1Optimize; virtual;
      function rcMatchSubString(const S   : string;
                                StartPosn : integer;
                                var Len   : integer) : boolean; virtual;
      function rcAddState(aMatchType : TNFAMatchType;
                          aChar      : AnsiChar;
                          aCharClass : PCharSet;
                          aNextState1: integer;
                          aNextState2: integer) : integer;
      function rcSetState(aState     : integer;
                          aNextState1: integer;
                          aNextState2: integer) : integer;

      function rcParseAnchorExpr : integer; virtual;
      function rcParseAtom : integer; virtual;
      function rcParseCCChar(out EscapeChar : Boolean) : AnsiChar; virtual;
      function rcParseChar : integer; virtual;
      function rcParseCharClass(aClass : PCharSet) : boolean; virtual;
      function rcParseCharRange(aClass : PCharSet) : boolean; virtual;
      function rcParseExpr : integer; virtual;
      function rcParseFactor : integer; virtual;
      function rcParseTerm : integer; virtual;
      Function rcReturnEscapeChar : AnsiChar; virtual;
    public
      procedure WriteTable;
      constructor Create(const aRegexStr : string);
      destructor Destroy; override;

<<<<<<< HEAD
<<<<<<< HEAD
      function Parse(out aErrorPos : integer;
                     out aErrorCode: TRegexError) : boolean; virtual;
=======
      function Parse(var aErrorPos : integer;
                     var aErrorCode: TRegexError) : boolean; virtual;
>>>>>>> graemeg/fixes_2_2
=======
      function Parse(var aErrorPos : integer;
                     var aErrorCode: TRegexError) : boolean; virtual;
>>>>>>> origin/fixes_2_2
      function MatchString(const S : string; out MatchPos : integer; var Offset : integer) : boolean; virtual;
      function ReplaceAllString(const src, newstr: ansistring; out DestStr : string): Integer;


      property IgnoreCase : boolean
                  read FIgnoreCase write rcSetIgnoreCase;
      property MultiLine : boolean
                  read FMultiLine write rcSetMultiLine;
      property RegexString : string
                  read FRegexStr write rcSetRegexStr;
      property Upcase : TUpcaseFunc
                  read FUpcase write rcSetUpcase;
  end;


Resourcestring
  eRegexParseError = 'Error at %d when parsing regular expression';

implementation

uses strutils;

const
  MetaCharacters : set of AnsiChar =
                   ['[', ']', '(', ')', '|', '*', '+', '?', '-', '.',
                    '^', '$', '{', '}'];
  newline : TCharSet = [#10,#13,#$85];
  {some handy constants}
  UnusedState = -1;
  NewFinalState = -2;
  CreateNewState = -3;
  ErrorState = -4;
  MustScan = -5;

  cs_allchars : tcharset = [#0..#255];
  cs_wordchars : tcharset = ['A'..'Z','a'..'z','_','0'..'9'];
  cs_newline : tcharset = [#10];
  cs_digits : tcharset = ['0'..'9'];
  cs_whitespace : tcharset = [' ',#9];


{===Helper routines==================================================}
function SystemUpcase(aCh : AnsiChar) : AnsiChar; far;
begin
  Result := System.Upcase(aCh);
end;
{====================================================================}


{===TRegexEngine===================================================}
constructor TRegexEngine.Create(const aRegexStr : string);
begin
  inherited Create;
  FRegexStr := aRegexStr;
  FIgnoreCase := false;
  FUpcase := SystemUpcase;
  SetLength(FStateTable,64);
  FStateCount:=0;
  FCapacity:=64;
  setlength(FList,FCapacity);
  {let's help out the user of the deque by putting the head and
   tail pointers in the middle: it's probably more efficient}
  FHead := FCapacity div 2;
  FTail := FHead;

  MultiLine:=False;
end;
{--------}
destructor TRegexEngine.Destroy;
begin
  if (FStateTable <> nil) then
    rcClear;
  inherited Destroy;
end;
{--------}
function TRegexEngine.MatchString(const S : string; out MatchPos : integer; var Offset : integer): boolean;
var
  i : integer;
  ErrorPos  : integer;
  ErrorCode : TRegexError;
  pc : pchar;
  x:integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if Offset>length(S) then
    begin
    Result := False;
    MatchPos := 0;
    Exit;
    end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {if the regex string hasn't been parsed yet, do so}
  if (FStateCount = 0) then begin
    if not Parse(ErrorPos, ErrorCode) then
      rcError(ErrorPos);
  end;

  case FRegexType of
    rtSingleChar :
      begin
      MatchPos := PosEx(char(FRegexStr[1]),s,Offset);
      Offset := MatchPos+1;
      Result := (MatchPos>0);
      end;
    rtChars :
      begin
      MatchPos := PosEx(FRegexStr,s,Offset);
      Offset := MatchPos+length(FRegexStr);
      Result := (MatchPos>0);
      end
    else
      begin
      {now try and see if the string matches (empty strings don't)}
      Result := False;
      MatchPos := 0;
      if (S <> '') then
        {if the regex specified a start anchor then we
         need to check the string starting at the first position}
        if FAnchorStart then begin
          if rcMatchSubString(S, 1, Offset) then
            begin
            MatchPos:=1;
            Result := True;
            end
          {If the first position did not match ang MultiLine is false, the string
           doesn't match. If MultiLine is true, start at every position after a
           newline }
          else if FMultiLine then begin
            for i := Offset to length(S)-1 do
              if S[i] in newline then
               if rcMatchSubString(S, i+1, Offset) then begin
                MatchPos := i+1;
                Result := True;
                Break;
              end;
          end
        end
        {otherwise we try and match the string at every position and
         return at the first success}
        else begin
          for i := Offset to length(S) do
            if rcMatchSubString(S, i, Offset) then begin
              MatchPos:=i;
              Result := True;
              Break;
            end;
        end;
      end;
    end; {case}
end;

function TRegexEngine.ReplaceAllString(const src, newstr: ansistring; out DestStr : string): Integer;

type TReplRec = record
                  Pos : integer;
                  Len : integer;
                end;

var ofs         : Integer;
    size_newstr,
    size, pos   : Integer;
    ReplArr     : array of TReplRec;
    racount     : integer;
    MatchPos    : integer;
    DestSize    : integer;
    LastPos     : integer;
    MoveLen     : integer;
    i           : integer;

begin
  setlength(ReplArr,64);

  racount := 0;
  DestSize:=length(src);
  size_newstr := length(newstr);
  Ofs := 1;
  while MatchString(src,MatchPos,Ofs) do
    begin
    if racount = length(ReplArr) then
      setlength(ReplArr,racount+racount div 2);
    ReplArr[racount].Pos := MatchPos;
    ReplArr[racount].Len := ofs;
    DestSize:=DestSize-ofs+MatchPos+size_newstr;
    inc(racount);
    end;

  SetLength(DestStr, SizeOf(Char)*DestSize);
  MatchPos:=1; LastPos:=1;

  if size_newstr<>0 then for i := 0 to racount -1 do
    begin
    MoveLen := ReplArr[i].Pos-LastPos;
    move(src[LastPos],DestStr[MatchPos],MoveLen);
    MatchPos:=MatchPos+MoveLen;
    LastPos := ReplArr[i].Len;
    move(newstr[1],DestStr[MatchPos],size_newstr);
    Matchpos := MatchPos+size_newstr;
    end
  else for i := 0 to racount -1 do
    begin
    MoveLen := ReplArr[i].Pos-LastPos;
    move(src[LastPos],DestStr[MatchPos],MoveLen);
    MatchPos:=MatchPos+MoveLen;
    LastPos := ReplArr[i].Len;
    end;

  move(src[LastPos],DestStr[MatchPos],length(src)-LastPos+1);
  Result := racount;
end;

{--------}
<<<<<<< HEAD
<<<<<<< HEAD
function TRegexEngine.Parse(out aErrorPos : integer;
                              out aErrorCode: TRegexError)
=======
function TRegexEngine.Parse(var aErrorPos : integer;
                              var aErrorCode: TRegexError)
>>>>>>> graemeg/fixes_2_2
=======
function TRegexEngine.Parse(var aErrorPos : integer;
                              var aErrorCode: TRegexError)
>>>>>>> origin/fixes_2_2
                                                            : boolean;
begin
  {clear the current transition table}
  rcClear;
  {empty regex strings are not allowed}
  if (FRegexStr = '') then begin
    Result := false;
    aErrorPos := 1;
    aErrorCode := recSuddenEnd;

    Exit;
  end;
  {parse the regex string}
  if not IgnoreCase then
    begin
    if length(FRegexStr)=1 then
      FRegexType:=rtSingleChar
    else
      FRegexType:=rtChars
    end
  else
    FRegexType:=rtRegEx;

  FPosn := PAnsiChar(FRegexStr);
  FStartState := rcParseAnchorExpr;
  {if an error occurred or we're not at the end of the regex string,
   clear the transition table, return false and the error position}
  if (FStartState = ErrorState) or (FPosn^ <> #0) then begin
    if (FStartState <> ErrorState) and (FPosn^ <> #0) then
      FErrorCode := recExtraChars;
    rcClear;
    Result := false;
    aErrorPos := succ(FPosn - PAnsiChar(FRegexStr));
    aErrorCode := FErrorCode;
  end
  {otherwise add a terminal state, optimize, return true}
  else begin
    rcAddState(mtTerminal, #0, nil, UnusedState, UnusedState);
    rcLevel1Optimize;
    if FAnchorStart or FAnchorEnd then FRegexType:= rtRegEx;
    Result := true;
    aErrorPos := 0;
    aErrorCode := recNone;
  end;
end;
{--------}
function TRegexEngine.rcAddState(aMatchType : TNFAMatchType;
                                   aChar      : AnsiChar;
                                   aCharClass : PCharSet;
                                   aNextState1: integer;
                                   aNextState2: integer) : integer;
begin
  {set up the fields in the state record}
  with FStateTable[FStateCount] do
    begin
    if (aNextState1 = NewFinalState) then
      sdNextState1 := FStateCount+1
    else
      sdNextState1 := aNextState1;
    sdNextState2 := aNextState2;
    sdMatchType := aMatchType;
    if (aMatchType = mtChar) then
      sdChar := aChar
<<<<<<< HEAD
<<<<<<< HEAD
    else if aMatchType in [mtClass, mtDupClass, mtNegClass] then
=======
    else if (aMatchType = mtClass) or (aMatchType = mtNegClass) then
>>>>>>> graemeg/fixes_2_2
=======
    else if (aMatchType = mtClass) or (aMatchType = mtNegClass) then
>>>>>>> origin/fixes_2_2
      sdClass := aCharClass;
    end;
  Result := FStateCount;
  inc(FStateCount);
  if FStateCount=length(FStateTable) then
    setlength(FStateTable,(FStateCount * 3) div 2);

<<<<<<< HEAD
<<<<<<< HEAD
  if not (aMatchType in [mtChar,mtTerminal,mtNone]) then FRegexType := rtRegEx;
=======
  if not (aMatchType in [mtChar,mtTerminal]) then FRegexType := rtRegEx;
>>>>>>> graemeg/fixes_2_2
=======
  if not (aMatchType in [mtChar,mtTerminal]) then FRegexType := rtRegEx;
>>>>>>> origin/fixes_2_2
end;
{--------}
procedure TRegexEngine.rcClear;
var
<<<<<<< HEAD
<<<<<<< HEAD
  i, j : integer;
=======
  i : integer;
>>>>>>> graemeg/fixes_2_2
=======
  i : integer;
>>>>>>> origin/fixes_2_2
begin
  {free all items in the state transition table}
  for i := 0 to FStateCount-1 do begin
    with FStateTable[i] do begin
      if (sdMatchType = mtClass) or
<<<<<<< HEAD
<<<<<<< HEAD
         (sdMatchType = mtNegClass) and
         (sdClass <> nil) then
        begin
          for j := i+1 to FStateCount-1 do
           if (FStateTable[j].sdClass = sdClass) then
             FStateTable[j].sdClass := nil;
          FreeMem(sdClass, sizeof(TCharSet));
        end;
      // I am not sure if the next line is necessary. rcAddState set all values, so
      // it shouldn't be necessary to clear its contents?
      // FillChar(FStateTable[i],SizeOf(FStateTable[i]),#0);
=======
         (sdMatchType = mtNegClass) then
        if (sdClass <> nil) then
          FreeMem(sdClass, sizeof(TCharSet));
>>>>>>> graemeg/fixes_2_2
=======
         (sdMatchType = mtNegClass) then
        if (sdClass <> nil) then
          FreeMem(sdClass, sizeof(TCharSet));
>>>>>>> origin/fixes_2_2
    end;
  end;
  {clear the state transition table}
  FStateCount:=0;
  FAnchorStart := false;
  FAnchorEnd := false;
end;
{--------}
procedure TRegexEngine.rcError(aIndex      : integer);
begin
  raise Exception.Create(Format(eRegexParseError,[aIndex]));
end;
{--------}
procedure TRegexEngine.rcLevel1Optimize;
var
  i : integer;
  Walker : integer;
begin
  {level 1 optimization removes all states that have only a single
   no-cost move to another state}

  {cycle through all the state records, except for the last one}
  for i := 0 to FStateCount - 2 do begin
    {get this state}
    with FStateTable[i] do begin
      {walk the chain pointed to by the first next state, unlinking
       the states that are simple single no-cost moves}
      Walker := sdNextState1;
      while (FStateTable[walker].sdMatchType = mtNone) and
            (FStateTable[walker].sdNextState2 = UnusedState) do begin
        sdNextState1 := FStateTable[walker].sdNextState1;
        Walker := sdNextState1;
      end;
      {walk the chain pointed to by the second next state, unlinking
       the states that are simple single no-cost moves}
      if (sdNextState2 <> UnusedState) then begin
        Walker := sdNextState2;
        while (FStateTable[walker].sdMatchType = mtNone) and
              (FStateTable[walker].sdNextState2 = UnusedState) do begin
          sdNextState2 := FStateTable[walker].sdNextState1;
          Walker := sdNextState2;
        end;
      end;
    end;
  end;

  {cycle through all the state records, except for the last one,
   marking unused ones--not strictly necessary but good for debugging}
  for i := 0 to FStateCount - 2 do begin
    with FStateTable[i] do begin
      if (sdMatchType = mtNone) and
         (sdNextState2 = UnusedState) then
        sdMatchType := mtUnused;
    end;
  end;
end;
{--------}
function TRegexEngine.rcMatchSubString(const s   : string;
                                         StartPosn : integer;
                                         var Len   : integer)
                                                            : boolean;
var
  Ch     : AnsiChar;
  State  : integer;
  StrInx : integer;
<<<<<<< HEAD
<<<<<<< HEAD
  LenStr : integer;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  {assume we fail to match}
  Result := false;
  Len := StartPosn;
<<<<<<< HEAD
<<<<<<< HEAD
  LenStr := Length(s);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {clear the deque}
  FHead := FCapacity div 2;
  FTail := FHead;

  
  {enqueue the special value to start scanning}
  DequeEnqueue(MustScan);
  {enqueue the first state}
  DequeEnqueue(FStartState);
  {prepare the string index}
  StrInx := StartPosn;
  {loop until the deque is empty or we run out of string}
  repeat
    {pop the top state from the deque}
    State := DequePop;
    {process the "must scan" state first}
    if (State = MustScan) then begin
      {if the deque is empty at this point, we might as well give up
       since there are no states left to process new characters}
      if (FHead <> FTail) then begin
        {if we haven't run out of string, get the character, and
         enqueue the "must scan" state again}
          if IgnoreCase then
            Ch := Upcase(s[StrInx])
          else
            Ch := s[StrInx];
          DequeEnqueue(MustScan);
        inc(StrInx);
      end;
    end
    {otherwise, process the state}
    else with FStateTable[State] do begin
      case sdMatchType of
        mtChar :
          begin
            {for a match of a character, enqueue the next state}
            if (Ch = sdChar) then
              DequeEnqueue(sdNextState1);
          end;
        mtAnyChar :
          begin
            {for a match of any character, enqueue the next state}
            if not (Ch in newline) then
              DequeEnqueue(sdNextState1);
          end;
<<<<<<< HEAD
<<<<<<< HEAD
        mtClass, mtDupClass :
=======
        mtClass :
>>>>>>> graemeg/fixes_2_2
=======
        mtClass :
>>>>>>> origin/fixes_2_2
          begin
            {for a match within a class, enqueue the next state}
            if (Ch in sdClass^) then
              DequeEnqueue(sdNextState1);
          end;
        mtNegClass :
          begin
            {for a match not within a class, enqueue the next state}
            if not (Ch in sdClass^) then
              DequeEnqueue(sdNextState1);
          end;
        mtTerminal :
          begin
            {for a terminal state, the string successfully matched
             if the regex had no end anchor, or we're at the end
             of the string or line}
            if (not FAnchorEnd) or (ch=#0) or (FMultiLine and (ch in newline)) then begin
              Result := true;
              Len := StrInx-1;
//                Exit;
            end;
          end;
        mtNone :
          begin
            {for free moves, push the next states onto the deque}
            Assert(sdNextState2 <> UnusedState,
                   'optimization should remove all states with one no-cost move');
            DequePush(sdNextState2);
            DequePush(sdNextState1);
          end;
        mtUnused :
          begin
            Assert(false, 'unused states shouldn''t be seen');
          end;
      end;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
  until (FHead = FTail) or (StrInx > LenStr); // deque empty or end of string
=======
  until (FHead = FTail) or (ch = #0); // deque empty or end of string
>>>>>>> graemeg/fixes_2_2
=======
  until (FHead = FTail) or (ch = #0); // deque empty or end of string
>>>>>>> origin/fixes_2_2
  {if we reach this point we've either exhausted the deque or we've
   run out of string; if the former, the substring did not match
   since there are no more states. If the latter, we need to check
   the states left on the deque to see if one is the terminating
   state; if so the string matched the regular expression defined by
   the transition table}
<<<<<<< HEAD
<<<<<<< HEAD
  while (FHead <> FTail) and (StrInx<=LenStr) do begin
=======
  while (FHead <> FTail) do begin
>>>>>>> graemeg/fixes_2_2
=======
  while (FHead <> FTail) do begin
>>>>>>> origin/fixes_2_2
    State := DequePop;
    with FStateTable[State] do begin
      case sdMatchType of
        mtNone :
          begin
            {for free moves, push the next states onto the deque}
            Assert(sdNextState2 <> UnusedState,
                   'optimization should remove all states with one no-cost move');
            DequePush(sdNextState2);
            DequePush(sdNextState1);
          end;
        mtTerminal :
          begin
            {for a terminal state, the string successfully matched
             if the regex had no end anchor, or we're at the end
             of the string or line}
            if (not FAnchorEnd) or (ch=#0) or (FMultiLine and (ch in newline)) then begin
              Result := true;
              Len := StrInx -1;
              Exit;
            end;
          end;
      end;{case}
    end;
  end;
end;
{--------}
function TRegexEngine.rcParseAnchorExpr : integer;
begin
  {check for an initial '^'}
  if (FPosn^ = '^') then begin
    FAnchorStart := true;
    inc(FPosn);
  end;

  {parse an expression}
  Result := rcParseExpr;

  {if we were successful, check for the final '$'}
  if (Result <> ErrorState) then begin
    if (FPosn^ = '$') then begin
      FAnchorEnd := true;
      inc(FPosn);
    end;
  end;
end;
{--------}
function TRegexEngine.rcParseAtom : integer;
var
  MatchType : TNFAMatchType;
  CharClass : PCharSet;
begin
  case FPosn^ of
    '(' :
      begin
        {move past the open parenthesis}
        inc(FPosn);

        {parse a complete regex between the parentheses}
        Result := rcParseExpr;
        if (Result = ErrorState) then
          Exit;
        {if the current character is not a close parenthesis,
         there's an error}
        if (FPosn^ <> ')') then begin
          FErrorCode := recNoCloseParen;
          Result := ErrorState;
          Exit;
        end;
        {move past the close parenthesis}
        inc(FPosn);
<<<<<<< HEAD
<<<<<<< HEAD
        {always handle expressions with parentheses as regular-expression}
        FRegexType := rtRegEx;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;
    '[' :
      begin
        {move past the open square bracket}
        inc(FPosn);

        {if the first character in the class is a '^' then the
         class if negated, otherwise it's a normal one}
        if (FPosn^ = '^') then begin
          inc(FPosn);
          MatchType := mtNegClass;
        end
        else begin
          MatchType := mtClass;
        end;
        {allocate the class character set and parse the character
         class; this will return either with an error, or when the
         closing square bracket is encountered}
        New(CharClass);
        CharClass^ := [];
        if not rcParseCharClass(CharClass) then begin
          Dispose(CharClass);
          Result := ErrorState;
          Exit;
        end;
        {move past the closing square bracket}
        Assert(FPosn^ = ']',
               'the rcParseCharClass terminated without finding a "]"');
        inc(FPosn);

        {add a new state for the character class}
        Result := rcAddState(MatchType, #0, CharClass,
                             NewFinalState, UnusedState);
      end;
    '.' :
      begin
        {move past the period metacharacter}
        inc(FPosn);

        {add a new state for the 'any character' token}
        Result := rcAddState(mtAnyChar, #0, nil,
                             NewFinalState, UnusedState);
      end;
<<<<<<< HEAD
<<<<<<< HEAD
    '\' :
      begin
        if (FPosn+1)^ in ['d','D','s','S','w','W'] then begin
          New(CharClass);
          CharClass^ := [];
          if not rcParseCharRange(CharClass) then begin
            Dispose(CharClass);
            Result := ErrorState;
            Exit;
          end;
          Result := rcAddState(mtClass, #0, CharClass,
                               NewFinalState, UnusedState);
        end
        else
          Result := rcParseChar;
      end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  else
    {otherwise parse a single character}
    Result := rcParseChar;
  end;{case}
end;
{--------}
function TRegexEngine.rcParseCCChar(out EscapeChar : Boolean) : AnsiChar;
begin
  EscapeChar:=False;
  {if we hit the end of the string, it's an error}
  if (FPosn^ = #0) then begin
    FErrorCode := recSuddenEnd;
    Result := #0;
    Exit;
  end;
  {if the current char is a metacharacter (at least in terms of a
   character class), it's an error}
  if FPosn^ in [']', '-'] then begin
    FErrorCode := recMetaChar;
    Result := #0;
    Exit;
  end;
  {otherwise return the character and advance past it}
  if (FPosn^ = '\') then
    {..it's an escaped character: get the next character instead}
    begin
    inc(FPosn);
    EscapeChar:=True;
    Result := rcReturnEscapeChar;
    end
  else
    Result := FPosn^;
  inc(FPosn);
end;
{--------}
function TRegexEngine.rcParseChar : integer;
var
  Ch : AnsiChar;
begin
  {if we hit the end of the string, it's an error}
  if (FPosn^ = #0) then begin
    Result := ErrorState;
    FErrorCode := recSuddenEnd;
    Exit;
  end;
  {if the current char is one of the metacharacters, it's an error}
  if FPosn^ in MetaCharacters then begin
    Result := ErrorState;
    FErrorCode := recMetaChar;
    Exit;
  end;
  {otherwise add a state for the character}
  {..if it's an escaped character: get the next character instead}
  if (FPosn^ = '\') then
    begin
    inc(FPosn);
    ch := rcReturnEscapeChar;
<<<<<<< HEAD
<<<<<<< HEAD
    FRegexType := rtRegEx;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end
  else
    ch :=FPosn^;
  if IgnoreCase then
    Ch := Upcase(ch);
  Result := rcAddState(mtChar, Ch, nil, NewFinalState, UnusedState);
  inc(FPosn);
end;
{--------}
function TRegexEngine.rcParseCharClass(aClass : PCharSet) : boolean;
begin
  {assume we can't parse a character class properly}
  Result := false;
  {parse a character range; if we can't there was an error and the
   caller will take care of it}
  if not rcParseCharRange(aClass) then
    Exit;
  {if the current character was not the right bracket, parse another
   character class (note: we're removing the tail recursion here)}
  while (FPosn^ <> ']') do begin
    if not rcParseCharRange(aClass) then
      Exit;
  end;
  {if we reach here we were successful}
  Result := true;
end;
{--------}
function TRegexEngine.rcParseCharRange(aClass : PCharSet) : boolean;
var
  StartChar : AnsiChar;
  EndChar   : AnsiChar;
  Ch        : AnsiChar;
  EscChar   : Boolean;
begin
  {assume we can't parse a character range properly}
  Result := false;
  {parse a single character; if it's null there was an error}
  StartChar := rcParseCCChar(EscChar);
  if (StartChar = #0) then
    Exit;
  if EscChar then
    begin
    case StartChar of
      'd' : aClass^ := aClass^ + cs_digits;
      'D' : aClass^ := aClass^ + cs_allchars-cs_digits;
      's' : aClass^ := aClass^ + cs_whitespace;
      'S' : aClass^ := aClass^ + cs_allchars-cs_whitespace;
      'w' : aClass^ := aClass^ + cs_wordchars;
      'W' : aClass^ := aClass^ + cs_allchars-cs_wordchars
    else
      EscChar := False;
    end;
    if EscChar then
      begin
      Result := True;
      Exit;
      end;
    end;
  {if the current character is not a dash, the range consisted of a
   single character}
  if (FPosn^ <> '-') then begin
    if IgnoreCase then
      Include(aClass^, Upcase(StartChar))
    else
      Include(aClass^, StartChar)
  end
  {otherwise it's a real range, so get the character at the end of the
   range; if that's null, there was an error}
  else begin
    inc(FPosn); {move past the '-'}
    EndChar := rcParseCCChar(EscChar);
    if (EndChar = #0) then
      Exit;
    {build the range as a character set}
    if (StartChar > EndChar) then begin
      Ch := StartChar;
      StartChar := EndChar;
      EndChar := Ch;
    end;
    for Ch := StartChar to EndChar do begin
      Include(aClass^, Ch);
      if IgnoreCase then
        Include(aClass^, Upcase(Ch));
    end;
  end;
  {if we reach here we were successful}
  Result := true;
end;
{--------}
function TRegexEngine.rcParseExpr : integer;
var
  StartState1 : integer;
  StartState2 : integer;
  EndState1   : integer;
  OverallStartState : integer;
begin
  {assume the worst}
  Result := ErrorState;
  {parse an initial term}
  StartState1 := rcParseTerm;
  if (StartState1 = ErrorState) then
    Exit;
  {if the current character is *not* a pipe character, no alternation
   is present so return the start state of the initial term as our
   start state}
  if (FPosn^ <> '|') then
    Result := StartState1
  {otherwise, we need to parse another expr and join the two together
   in the transition table}
  else begin

    {advance past the pipe}
    inc(FPosn);
    {the initial term's end state does not exist yet (although there
     is a state in the term that points to it), so create it}
    EndState1 := rcAddState(mtNone, #0, nil, UnusedState, UnusedState);
    {for the OR construction we need a new initial state: it will
     point to the initial term and the second just-about-to-be-parsed
     expr}
    OverallStartState := rcAddState(mtNone, #0, nil,
                                    UnusedState, UnusedState);
    {parse another expr}
    StartState2 := rcParseExpr;
    if (StartState2 = ErrorState) then
      Exit;
    {alter the state state for the overall expr so that the second
     link points to the start of the second expr}
    Result := rcSetState(OverallStartState, StartState1, StartState2);
    {now set the end state for the initial term to point to the final
     end state for the second expr and the overall expr}
    rcSetState(EndState1, FStateCount, UnusedState);
<<<<<<< HEAD
<<<<<<< HEAD
    {always handle expressions with a pipe as regular-expression}
    FRegexType := rtRegEx;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
end;
{--------}
function TRegexEngine.rcParseFactor : integer;
var
  StartStateAtom : integer;
  EndStateAtom   : integer;
  TempEndStateAtom : integer;
  Int            : string;
  n,m,nState     : integer;
  i              : integer;
begin
  {assume the worst}
  Result := ErrorState;
  {first parse an atom}
  StartStateAtom := rcParseAtom;
  if (StartStateAtom = ErrorState) then
    Exit;
  {check for a closure operator}
  case FPosn^ of
    '?' : begin
            {move past the ? operator}
            inc(FPosn);
            {the atom's end state doesn't exist yet, so create one}
            EndStateAtom := rcAddState(mtNone, #0, nil,
                                       UnusedState, UnusedState);
            {create a new start state for the overall regex}
            Result := rcAddState(mtNone, #0, nil,
                                 StartStateAtom, EndStateAtom);
            {make sure the new end state points to the next unused
             state}
            rcSetState(EndStateAtom, FStateCount, UnusedState);
          end;
    '*' : begin
            {move past the * operator}
            inc(FPosn);
            {the atom's end state doesn't exist yet, so create one;
             it'll be the start of the overall regex subexpression}
            Result := rcAddState(mtNone, #0, nil,
                                 NewFinalState, StartStateAtom);
          end;
    '+' : begin
            {move past the + operator}
            inc(FPosn);
            {the atom's end state doesn't exist yet, so create one}
            rcAddState(mtNone, #0, nil, NewFinalState, StartStateAtom);
            {the start of the overall regex subexpression will be the
             atom's start state}
            Result := StartStateAtom;
          end;
    '{' : begin // {n,m}
            {move past the brace }
            inc(FPosn);

            {Parse the value of n}
            Int := '';
            while not (FPosn^ in [',','}',#0]) do
              begin
              int := int+FPosn^;
              inc(FPosn);
              end;
            if FPosn^ = #0 then exit; // No end-brace or comma -> invalid regex
            if int <> '' then
              n := StrToIntDef(Int,-2)
            else
              n := -1; // if n is 'empty', set it to -1
            if n = -2 then exit; // Invalid value for n -> invalid RegEx

            if FPosn^ <> '}' then
              begin

              {move past the , }
              inc(FPosn);
              {Parse the value of m}
              Int := '';
              while not (FPosn^ in ['}',#0]) do
                begin
                int := int+FPosn^;
                inc(FPosn);
                end;
              if FPosn^ <> '}' then exit; // No end-brace -> invalid regex
              if int <> '' then m := StrToIntDef(Int,-2)
              else m := -1;
              if m = -2 then exit; // Invalid RegEx
              end
            else
              m := -3;

            {move past the brace }
            inc(FPosn);

            if (n=0) and (m=-1) then
            {the atom's end state doesn't exist yet, so create one;
             it'll be the start of the overall regex subexpression}
              Result := rcAddState(mtNone, #0, nil, NewFinalState, StartStateAtom)
            else
              begin
              EndStateAtom := FStateCount-1;
              TempEndStateAtom:=StartStateAtom;
              for i := 1 to n-1 do
                begin
                TempEndStateAtom:=FStateCount;
                for nState:=StartStateAtom to EndStateAtom do
                  begin
                  FStateTable[FStateCount]:=FStateTable[nState];
                  if FStateTable[FStateCount].sdNextState1 in [StartStateAtom..EndStateAtom+1] then
                    FStateTable[FStateCount].sdNextState1 := i+FStateTable[FStateCount].sdNextState1+ (EndStateAtom-StartStateAtom) *i;
                  if FStateTable[FStateCount].sdNextState2 in [StartStateAtom..EndStateAtom+1] then
                    FStateTable[FStateCount].sdNextState2 := i+FStateTable[FStateCount].sdNextState2 + (EndStateAtom-StartStateAtom) *i;
<<<<<<< HEAD
<<<<<<< HEAD
                  if FStateTable[FStateCount].sdMatchType = mtClass then
                    FStateTable[FStateCount].sdMatchType := mtDupClass;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                  inc(FStateCount);

                  if FStateCount=length(FStateTable) then
                    setlength(FStateTable,(FStateCount * 3) div 2);
                  end;
                end;

            for i := n to m-1 do
              begin
              rcAddState(mtNone, #0, nil, NewFinalState, EndStateAtom+(EndStateAtom-StartStateAtom+1) * (m-1) + (m-n)+1);

              TempEndStateAtom:=FStateCount;
              for nState:=StartStateAtom to EndStateAtom do
                begin
                FStateTable[FStateCount]:=FStateTable[nState];
                if FStateTable[FStateCount].sdNextState1 in [StartStateAtom..EndStateAtom+1] then
                  FStateTable[FStateCount].sdNextState1 := i+FStateTable[FStateCount].sdNextState1+ (EndStateAtom-StartStateAtom) * i+(i-n+1);
                if FStateTable[FStateCount].sdNextState2 in [StartStateAtom..EndStateAtom+1] then
                  FStateTable[FStateCount].sdNextState2 := i+FStateTable[FStateCount].sdNextState2 + (EndStateAtom-StartStateAtom) * i+(i-n+1);
<<<<<<< HEAD
<<<<<<< HEAD
                if FStateTable[FStateCount].sdMatchType = mtClass then
                  FStateTable[FStateCount].sdMatchType := mtDupClass;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                inc(FStateCount);

                if FStateCount=length(FStateTable) then
                  setlength(FStateTable,(FStateCount * 3) div 2);
                end;
              end;

              if m = -1 then
                rcAddState(mtNone, #0, nil, NewFinalState, TempEndStateAtom);

              Result := StartStateAtom;
              end;
<<<<<<< HEAD
<<<<<<< HEAD
            {always handle expressions with braces as regular-expression}
            FRegexType := rtRegEx;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          end;

  else
    Result := StartStateAtom;
  end;{case}
end;
{--------}
function TRegexEngine.rcParseTerm : integer;
var
  StartState2 : integer;
  EndState1   : integer;
begin
  {parse an initial factor, the state number returned will also be our
   return state number}
  Result := rcParseFactor;
  if (Result = ErrorState) then
    Exit;
  {Note: we have to "break the grammar" here. We've parsed a regular
         subexpression and we're possibly following on with another
         regular subexpression. There's no nice operator to key off
         for concatenation: we just have to know that for
         concatenating two subexpressions, the current character will
         be
           - an open parenthesis
           - an open square bracket
           - an any char operator
           - a character that's not a metacharacter
         i.e., the three possibilities for the start of an "atom" in
         our grammar}
  if (FPosn^ = '(') or
     (FPosn^ = '[') or
     (FPosn^ = '.') or
     ((FPosn^ <> #0) and not (FPosn^ in MetaCharacters)) then begin

    {the initial factor's end state does not exist yet (although there
     is a state in the term that points to it), so create it}
    EndState1 := rcAddState(mtNone, #0, nil, UnusedState, UnusedState);
    {parse another term}
    StartState2 := rcParseTerm;
    if (StartState2 = ErrorState) then begin
      Result := ErrorState;
      Exit;
    end;
    {join the first factor to the second term}
    rcSetState(EndState1, StartState2, UnusedState);
  end;
end;

procedure TRegexEngine.WriteTable;
var i : integer;
begin
  for i := 0 to FStateCount-1 do with FStateTable[i] do
<<<<<<< HEAD
<<<<<<< HEAD
    writeln('s:',i,' mt:',sdMatchType ,' ns1:',sdNextState1,' ns2:',sdNextState2,' char:',sdChar);
=======
//  FPC version 2.2.1 can not write enum-types. Disabled in fixes_2_2 only
//    writeln('s:',i,' mt:',sdMatchType ,' ns1:',sdNextState1,' ns2:',sdNextState2,' char:',sdChar);
    writeln('s:',i,' ns1:',sdNextState1,' ns2:',sdNextState2,' char:',sdChar);
>>>>>>> graemeg/fixes_2_2
=======
//  FPC version 2.2.1 can not write enum-types. Disabled in fixes_2_2 only
//    writeln('s:',i,' mt:',sdMatchType ,' ns1:',sdNextState1,' ns2:',sdNextState2,' char:',sdChar);
    writeln('s:',i,' ns1:',sdNextState1,' ns2:',sdNextState2,' char:',sdChar);
>>>>>>> origin/fixes_2_2
end;

procedure TRegexEngine.DequeEnqueue(aValue: integer);
begin
  FList[FTail] := aValue;
  inc(FTail);
  if (FTail = FCapacity) then
    FTail := 0
  else if (FTail = FHead) then
    DequeGrow;
end;

procedure TRegexEngine.DequePush(aValue: integer);
begin
  if (FHead = 0) then
    FHead := FCapacity;
  dec(FHead);
  FList[FHead] := aValue;
  if (FTail = FHead) then
    DequeGrow;
end;

function TRegexEngine.DequePop: integer;
begin
  Result := FList[FHead];
  inc(FHead);
  if (FHead = FCapacity) then
    FHead := 0;
end;

procedure TRegexEngine.DequeGrow;
var
  OldCount : integer;
  i, j     : integer;
begin
  {grow the list by 50%}
  OldCount := FCapacity;
  FCapacity:=(OldCount * 3) div 2;
  SetLength(FList,FCapacity);
  {expand the data into the increased space, maintaining the deque}
  if (FHead = 0) then
    FTail := OldCount
  else begin
    j := FCapacity;
    for i := pred(OldCount) downto FHead do begin
      dec(j);
      FList[j] := FList[i]
    end;
    FHead := j;
  end;
end;

function TRegexEngine.rcReturnEscapeChar: AnsiChar;
begin
  case FPosn^ of
    't' : Result := #9;
    'n' : Result := #10;
    'r' : Result := #13;
    'f' : Result := #12;
    'a' : Result := #7;
  else
    Result := FPosn^;
  end;
end;

{--------}
procedure TRegexEngine.rcSetIgnoreCase(aValue : boolean);
begin
  if (aValue <> FIgnoreCase) then begin
    rcClear;
    FIgnoreCase := aValue;
  end;
end;
{--------}
procedure TRegexEngine.rcSetRegexStr(const aRegexStr : string);
begin
  if (aRegexStr <> FRegexStr) then begin
    rcClear;
    FRegexStr := aRegexStr;
  end;
end;
{--------}
function TRegexEngine.rcSetState(aState     : integer;
                                   aNextState1: integer;
                                   aNextState2: integer) : integer;
begin
  Assert((0 <= aState) and (aState < FStateCount),
         'trying to change an invalid state');

  {get the state record and change the transition information}
  FStateTable[aState].sdNextState1 := aNextState1;
  FStateTable[aState].sdNextState2 := aNextState2;
  Result := aState;
end;
{--------}
procedure TRegexEngine.rcSetUpcase(aValue : TUpcaseFunc);
begin
  if not Assigned(aValue) then
    FUpcase := SystemUpcase
  else
    FUpcase := aValue;
end;

procedure TRegexEngine.rcSetMultiLine(aValue: Boolean);
begin
  FMultiLine:=aValue;
end;

{====================================================================}

end.
