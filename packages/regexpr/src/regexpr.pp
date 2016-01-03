{
    This file is part of the Free Pascal packages library.
    Copyright (c) 2008 by Joost van der Sluis, member of the
    Free Pascal development team

    Compatibility unit for the old regexpr unit.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit RegExpr;

{$mode objfpc}{$H+}

interface

uses
  Regex;

type
   tregexprflag = (
     ref_singleline,
     {** This indicates that a start of line is either the
         start of the pattern or a linebreak. }
     ref_multiline,
     {** The match will be done in a case-insensitive way
          according to US-ASCII character set. }
     ref_caseinsensitive);
   tregexprflags = set of tregexprflag;

   TRegExprEngine = TRegexEngine;

function GenerateRegExprEngine(regexpr : pchar;flags : tregexprflags;var RegExprEngine: TRegExprEngine): boolean;
function GenerateRegExprEngine(regexpr : pchar;flags : tregexprflags): TREGExprEngine;
procedure DestroyRegExprEngine(var regexpr : TRegExprEngine);

function RegExprPos(RegExprEngine : TRegExprEngine;p : pchar;var index,len : longint) : boolean;
function RegExprReplaceAll(RegExprEngine : TRegExprEngine;const src,newstr : ansistring;var dest : ansistring) : sizeint;

function RegExprEscapeStr (const S : string) : string;

implementation

function GenerateRegExprEngine(regexpr: pchar; flags: tregexprflags;
  var RegExprEngine: TRegExprEngine): boolean;
var ErrorPos  : Integer;
    ErrorCode : TRegexError;

begin
  RegExprEngine := TRegExprEngine.Create(regexpr);
  if ref_multiline in flags then RegExprEngine.MultiLine:=True;
  if ref_caseinsensitive in flags then RegExprEngine.IgnoreCase:=True;
  Result := RegExprEngine.Parse(ErrorPos,ErrorCode);
end;

function GenerateRegExprEngine(regexpr: pchar; flags: tregexprflags
  ): TREGExprEngine;
  
var r: TRegExprEngine;

begin
  GenerateRegExprEngine(regexpr,flags,r);
  GenerateRegExprEngine:=r;
end;

procedure DestroyRegExprEngine(var regexpr: TRegExprEngine);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if regexpr <> nil then
    regexpr.Free;
  regexpr := nil;
=======
  regexpr.Free;
>>>>>>> graemeg/fixes_2_2
=======
  regexpr.Free;
>>>>>>> origin/fixes_2_2
end;

function RegExprPos(RegExprEngine: TRegExprEngine; p: pchar; var index,
  len: longint): boolean;
begin
  Len := 1;
  Result := RegExprEngine.MatchString(p,index,len);
  Len := Len - index;
  Dec(Index);
<<<<<<< HEAD
<<<<<<< HEAD
  if not Result then
    begin
      index := -1;
      len := 0;
    end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function RegExprReplaceAll(RegExprEngine: TRegExprEngine; const src,
  newstr: ansistring; var dest: ansistring): sizeint;
begin
  result := RegExprEngine.ReplaceAllString(src,newstr,Dest);
end;

function RegExprEscapeStr(const S: string): string;
var
  i, len   : integer;
  s1: string;
begin
  RegExprEscapeStr:= '';
  s1:='';
  if (S = '') then
   exit;

  len := Length (S);

  for i := 1 to len do
    begin
      if (S [i] in ['(','|', '.', '*', '?', '^', '$', '-', '[', '{', '}', ']', ')', '\']) then
        begin
          s1 := s1 + '\';
        end;

      s1 := s1 + S[i];
    end;
  RegExprEscapeStr:=s1;
end;

end.

