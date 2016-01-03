{ based on string/tester.c of glibc 2.3.6 

* Tester for string functions.
   Copyright (C) 1995-2000, 2001, 2003 Free Software Foundation, Inc.
   This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.  */
}

{$ifdef fpc}
{$mode delphi}
{$endif fpc}

uses
{$ifdef unix}
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef darwin}iosxwstr{$else}cwstring{$endif},
=======
  cwstring,
>>>>>>> graemeg/fixes_2_2
=======
  cwstring,
>>>>>>> origin/fixes_2_2
{$endif unix}
  SysUtils;

var
  teststr: string;
  goterror: boolean;

procedure check(b: boolean; testnr: longint);
begin
  if not (b) then
    begin
      writeln(teststr,' error nr ',testnr);
      goterror:=true;
    end;
end;

procedure testAnsiCompareText;
begin
  teststr:='AnsiCompareText';
  check(ansicomparetext('a', 'a') = 0, 1);
  check(ansicomparetext('a', 'A') = 0, 2);
  check(ansicomparetext('A', 'a') = 0, 3);
  check(ansicomparetext('a', 'b') < 0, 4);
  check(ansicomparetext('c', 'b') > 0, 5);
  check(ansicomparetext('abc', 'AbC') = 0, 6);
  check(ansicomparetext('0123456789', '0123456789') = 0, 7);
  check(ansicomparetext('', '0123456789') < 0, 8);
  check(ansicomparetext('AbC', '') > 0, 9);
  check(ansicomparetext('AbC', 'A') > 0, 10);
  check(ansicomparetext('AbC', 'Ab') > 0, 11);
  check(ansicomparetext('AbC', 'ab') > 0, 12);
  check(ansicomparetext('Ab'#0'C', 'ab'#0) > 0, 13);
end;


procedure testAnsiStrIComp;
begin
  teststr:='AnsiStrIComp';
  check(ansistricomp('a', 'a') = 0, 1);
  check(ansistricomp('a', 'A') = 0, 2);
  check(ansistricomp('A', 'a') = 0, 3);
  check(ansistricomp('a', 'b') < 0, 4);
  check(ansistricomp('c', 'b') > 0, 5);
  check(ansistricomp('abc', 'AbC') = 0, 6);
  check(ansistricomp('0123456789', '0123456789') = 0, 7);
  check(ansistricomp('', '0123456789') < 0, 8);
  check(ansistricomp('AbC', '') > 0, 9);
  check(ansistricomp('AbC', 'A') > 0, 10);
  check(ansistricomp('AbC', 'Ab') > 0, 11);
  check(ansistricomp('AbC', 'ab') > 0, 12);
  check(ansistricomp('Ab'#0'C', 'ab'#0) = 0, 13);
end;


procedure testAnsiStrLComp;
begin
  teststr:='AnsiStrLComp';
  check (ansistrlcomp ('', '', 0) = 0, 1); { Trivial case. }
  check (ansistrlcomp ('a', 'a', 1) = 0, 2);       { Identity. }
  check (ansistrlcomp ('abc', 'abc', 3) = 0, 3);   { Multicharacter. }
  check (ansistrlcomp ('abc'#0, 'abcd', 4) < 0, 4);   { Length unequal. }
  check (ansistrlcomp ('abcd', 'abc'#0, 4) > 0, 5);
  check (ansistrlcomp ('abcd', 'abce', 4) < 0, 6);  { Honestly unequal. }
  check (ansistrlcomp ('abce', 'abcd', 4) > 0, 7);
  check (ansistrlcomp ('abce', 'abcd', 3) = 0, 10); { Count limited. }
  check (ansistrlcomp ('abce', 'abc', 3) = 0, 11);  { Count = length. }
  check (ansistrlcomp ('abcd', 'abce', 4) < 0, 12);  { Nudging limit. }
  check (ansistrlcomp ('abc', 'def', 0) = 0, 13);   { Zero count. }
  check (ansistrlcomp ('abc'#0'e', 'abc'#0'd', 5) > 0, 14);
end;


procedure testAnsiCompareStr;
begin
  teststr:='AnsiCompareStr';
  check (ansicomparestr ('', '') = 0, 1);              { Trivial case. }
  check (ansicomparestr ('a', 'a') = 0, 2);            { Identity. }
  check (ansicomparestr ('abc', 'abc') = 0, 3);        { Multicharacter. }
  check (ansicomparestr ('abc', 'abcd') < 0, 4);        { Length mismatches. }
  check (ansicomparestr ('abcd', 'abc') > 0, 5);
  check (ansicomparestr ('abcd', 'abce') < 0, 6);       { Honest miscompares. }
  check (ansicomparestr ('abce', 'abcd') > 0, 7);
  check (ansicomparestr ('abc'#0'e', 'abc'#0'd') > 0, 8);
end;


procedure testAnsiStrComp;
begin
  teststr:='AnsiStrComp';
  check (ansistrcomp ('', '') = 0, 1);              { Trivial case. }
  check (ansistrcomp ('a', 'a') = 0, 2);            { Identity. }
  check (ansistrcomp ('abc', 'abc') = 0, 3);        { Multicharacter. }
  check (ansistrcomp ('abc', 'abcd') < 0, 4);        { Length mismatches. }
  check (ansistrcomp ('abcd', 'abc') > 0, 5);
  check (ansistrcomp ('abcd', 'abce') < 0, 6);       { Honest miscompares. }
  check (ansistrcomp ('abce', 'abcd') > 0, 7);
  check (ansistrcomp ('abc'#0'e', 'abc'#0'd') = 0, 8);
end;


procedure testAnsiStrLIComp;
begin
  teststr:='AnsiStrLIComp';
  check(ansistrlicomp('a', 'a', 1) = 0, 1);
  check(ansistrlicomp('a', 'A', 1) = 0, 2);
  check(ansistrlicomp('A', 'a', 1) = 0, 3);
  check(ansistrlicomp('a', 'b', 1) < 0, 4);
  check(ansistrlicomp('c', 'b', 1) > 0, 5);
  check(ansistrlicomp('abc', 'AbC', 3) = 0, 6);
  check(ansistrlicomp('0123456789', '0123456789', 10) = 0, 7);
  check(ansistrlicomp(#0'123456789', #0'123456799', 10) < 0, 8);
  check(ansistrlicomp(#0'bD', #0'bC', 3) > 0, 9);
  check(ansistrlicomp('AbC', 'A'#0#0,3) > 0, 10);
  check(ansistrlicomp('AbC', 'Ab'#0, 3) > 0, 11);
  check(ansistrlicomp('AbC', 'ab'#0, 3) > 0, 12);
  check(ansistrlicomp('0123456789', 'AbC', 0) = 0, 13);
  check(ansistrlicomp('AbC', 'abc', 1) = 0, 14);
  check(ansistrlicomp('AbC', 'abc', 2) = 0, 15);
  check(ansistrlicomp('AbC', 'abc', 3) = 0, 16);
  check(ansistrlicomp('AbC', 'abcd', 3) = 0, 17);
  check(ansistrlicomp('AbCc', 'abcd', 4) < 0, 18);
  check(ansistrlicomp('ADC', 'abcd', 1) = 0, 19);
  check(ansistrlicomp('ADC', 'abcd', 2) > 0, 20);
  check(ansistrlicomp('abc'#0'e', 'abc'#0'd', 5) > 0, 21);
end;


begin
  goterror:=false;
  testAnsiCompareText;
  testAnsiStrIComp;
  testAnsiStrLComp;
  testAnsiCompareStr;
  testAnsiStrComp;
  testAnsiStrLIComp;
  if goterror then
    halt(1);
end.
