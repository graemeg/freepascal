{
    Copyright (c) 1998-2002 by Peter Vreman

    This unit implements the message object

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}
unit cmsgs;

{$i fpcdefs.inc}

interface

uses
  globtype;

const
  maxmsgidxparts = 20;

type
  ppchar=^pchar;
  TMsgStr = AnsiString;

  TArrayOfPChar = array[0..1000] of pchar;
  PArrayOfPChar = ^TArrayOfPChar;

  TArrayOfState = array[0..1000] of tmsgstate;
  PArrayOfState = ^TArrayOfState;

  PMessage=^TMessage;
  TMessage=object
    msgfilename : string;
    msgintern   : boolean;
    msgallocsize,
    msgsize,
    msgparts,
    msgs        : longint;
    msgtxt      : pchar;
    msgidx      : array[1..maxmsgidxparts] of PArrayOfPChar;
    msgidxmax   : array[1..maxmsgidxparts] of longint;
    msgstates   : array[1..maxmsgidxparts] of PArrayOfState;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    { set if changes with $WARN need to be cleared at next module change }
    has_local_changes : boolean;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    constructor Init(n:longint;const idxmax:array of longint);
    destructor  Done;
    function  LoadIntern(p:pointer;n:longint):boolean;
    function  LoadExtern(const fn:string):boolean;
    procedure ClearIdx;
    procedure ResetStates;
    procedure CreateIdx;
    function  GetPChar(nr:longint):pchar;
<<<<<<< HEAD
<<<<<<< HEAD
    { function  ClearVerbosity(nr:longint):boolean; not used anymore }
    function  SetVerbosity(nr:longint;newstate:tmsgstate):boolean;
    function  Get(nr:longint;const args:array of TMsgStr):ansistring;
=======
    function  Get(nr:longint;const args:array of string):ansistring;
>>>>>>> graemeg/fixes_2_2
=======
    function  Get(nr:longint;const args:array of string):ansistring;
>>>>>>> origin/fixes_2_2
  end;

{ this will read a line until #10 or #0 and also increase p }
function GetMsgLine(var p:pchar):string;


implementation

uses
  SysUtils,
  cutils;


<<<<<<< HEAD
<<<<<<< HEAD
function MsgReplace(const s:TMsgStr;const args:array of TMsgStr):ansistring;
var
  last,
  i  : longint;
  hs : TMsgStr;
=======
=======
>>>>>>> origin/fixes_2_2
function MsgReplace(const s:string;const args:array of string):ansistring;
var
  last,
  i  : longint;
  hs : ansistring;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  if s='' then
    begin
      MsgReplace:='';
      exit;
    end;
  hs:='';
  i:=0;
  last:=0;
  while (i<length(s)-1) do
    begin
      inc(i);
      if (s[i]='$') and (s[i+1] in ['1'..'9']) then
        begin
          hs:=hs+copy(s,last+1,i-last-1)+args[byte(s[i+1])-byte('1')];
          inc(i);
          last:=i;
        end;
    end;
  MsgReplace:=hs+copy(s,last+1,length(s)-last);
end;



constructor TMessage.Init(n:longint;const idxmax:array of longint);
var
  i,j : longint;
begin
  msgtxt:=nil;
  has_local_changes:=false;
  msgsize:=0;
  msgparts:=n;
  if n<>high(idxmax)+1 then
   fail;
  for i:=1 to n do
   begin
     msgidxmax[i]:=idxmax[i-1];
     { create array of msgidx }
     getmem(msgidx[i],msgidxmax[i]*sizeof(pointer));
     fillchar(msgidx[i]^,msgidxmax[i]*sizeof(pointer),0);
     { create array of states }
     getmem(msgstates[i],msgidxmax[i]*sizeof(tmsgstate));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     { default value for msgstate is ms_on_global }
     for j:=0 to msgidxmax[i]-1 do
       msgstates[i]^[j]:=ms_on_global;
=======
     fillchar(msgstates[i]^,msgidxmax[i]*sizeof(tmsgstate),0);
>>>>>>> graemeg/cpstrnew
=======
     fillchar(msgstates[i]^,msgidxmax[i]*sizeof(tmsgstate),0);
>>>>>>> graemeg/cpstrnew
=======
     fillchar(msgstates[i]^,msgidxmax[i]*sizeof(tmsgstate),0);
>>>>>>> graemeg/cpstrnew
=======
     fillchar(msgstates[i]^,msgidxmax[i]*sizeof(tmsgstate),0);
>>>>>>> origin/cpstrnew
   end;
end;


destructor TMessage.Done;
var
  i : longint;
begin
  for i:=1 to msgparts do
  begin
   freemem(msgidx[i],msgidxmax[i]*sizeof(pointer));
   freemem(msgstates[i],msgidxmax[i]*sizeof(tmsgstate));
  end;
  if msgallocsize>0 then
   begin
     freemem(msgtxt,msgsize);
     msgallocsize:=0;
   end;
  msgtxt:=nil;
  msgsize:=0;
  msgparts:=0;
end;


function TMessage.LoadIntern(p:pointer;n:longint):boolean;
begin
  msgtxt:=pchar(p);
  msgsize:=n;
  msgallocsize:=0;
  msgintern:=true;
  ClearIdx;
  CreateIdx;
  LoadIntern:=true;
end;


function TMessage.LoadExtern(const fn:string):boolean;
const
  bufsize=8192;
var
  f       : text;
  error,multiline : boolean;
  line,i,j : longint;
  ptxt    : pchar;
  s,s1    : string;
  buf     : pointer;

  procedure err(const msgstr:TMsgStr);
  begin
    writeln('*** PPC, file ',fn,', error in line ',line,': ',msgstr);
    error:=true;
  end;

begin
  LoadExtern:=false;
  getmem(buf,bufsize);
  { Read the message file }
  assign(f,fn);
  {$push}{$I-}
   reset(f);
  {$pop}
  if ioresult<>0 then
   begin
     WriteLn('*** PPC, can not open message file ',fn);
     exit;
   end;
  settextbuf(f,buf^,bufsize);
  { First parse the file and count bytes needed }
  error:=false;
  line:=0;
  multiline:=false;
  msgsize:=0;
  while not eof(f) do
   begin
     readln(f,s);
     inc(line);
     if multiline then
      begin
        if s=']' then
         multiline:=false
        else
         inc(msgsize,length(s)+1); { +1 for linebreak }
      end
     else
      begin
        if (s<>'') and not(s[1] in ['#',';','%']) then
         begin
           i:=pos('=',s);
           if i>0 then
            begin
              j:=i+1;
              if not(s[j] in ['0'..'9']) then
               err('no number found')
              else
               begin
                 while (s[j] in ['0'..'9']) do
                  inc(j);
               end;
              if j-i-1<>5 then
               err('number length is not 5');
              if s[j+1]='[' then
               begin
                 inc(msgsize,j-i);
                 multiline:=true
               end
              else
               inc(msgsize,length(s)-i+1);
            end
           else
            err('no = found');
         end;
      end;
   end;
  if multiline then
   err('still in multiline mode');
  if error then
   begin
     freemem(buf,bufsize);
     close(f);
     exit;
   end;
  { now read the buffer in mem }
  msgallocsize:=msgsize;
  getmem(msgtxt,msgallocsize);
  ptxt:=msgtxt;
  reset(f);
  while not eof(f) do
   begin
     readln(f,s);
     if multiline then
      begin
        if s=']' then
         begin
           multiline:=false;
           { overwrite last eol }
           dec(ptxt);
           ptxt^:=#0;
           inc(ptxt);
         end
        else
         begin
           move(s[1],ptxt^,length(s));
           inc(ptxt,length(s));
           ptxt^:=#10;
           inc(ptxt);
         end;
      end
     else
      begin
        if (s<>'') and not(s[1] in ['#',';','%']) then
         begin
           i:=pos('=',s);
           if i>0 then
            begin
              j:=i+1;
              while (s[j] in ['0'..'9']) do
               inc(j);
              { multiline start then no txt }
              if s[j+1]='[' then
               begin
                 s1:=Copy(s,i+1,j-i);
                 move(s1[1],ptxt^,length(s1));
                 inc(ptxt,length(s1));
                 multiline:=true;
               end
              else
               begin
                 { txt including number }
                 s1:=Copy(s,i+1,255);
                 move(s1[1],ptxt^,length(s1));
                 inc(ptxt,length(s1));
                 ptxt^:=#0;
                 inc(ptxt);
               end;
            end;
         end;
      end;
   end;
  close(f);
  freemem(buf,bufsize);
{ now we can create the index, clear if the previous load was also
  an external file, because those can't be reused }
  if not msgintern then
   ClearIdx;
  CreateIdx;
{ set that we've loaded an external file }
  msgintern:=false;
  LoadExtern:=true;
end;


procedure TMessage.ClearIdx;
var
  i : longint;
begin
  { clear }
  for i:=1 to msgparts do
   fillchar(msgidx[i]^,msgidxmax[i]*sizeof(pointer),0);
end;


procedure TMessage.CreateIdx;
var
  hp1,
  hp,hpend : pchar;
  code : integer;
  num  : longint;
  number : string[5];
  i   : longint;
  numpart,numidx : longint;
begin
  { process msgtxt buffer }
  number:='00000';
  hp:=msgtxt;
  hpend:=@msgtxt[msgsize];
  while (hp<hpend) do
   begin
     hp1:=hp;
     for i:=1 to 5 do
      begin
        number[i]:=hp1^;
        inc(hp1);
      end;
     val(number,num,code);
     numpart:=num div 1000;
     numidx:=num mod 1000;
     { check range }
     if (numpart <= msgparts) and (numidx < msgidxmax[numpart]) then
      begin
        { skip _ }
        inc(hp1);
        { set default verbosity to off is '-' is found just after the '_' }
        if hp1^='-' then
         begin
           msgstates[numpart]^[numidx]:=ms_off_global;
           inc(hp1);
         end;
        { put the address in the idx, the numbers are already checked }
        msgidx[numpart]^[numidx]:=hp1;
      end;
     { next string }
     hp:=pchar(@hp[strlen(hp)+1]);
   end;
end;


function GetMsgLine(var p:pchar):string;
var
  i  : longint;
begin
  i:=0;
  while not(p^ in [#0,#10]) and (i<256) do
   begin
     inc(i);
     GetMsgLine[i]:=p^;
     inc(p);
   end;
  { skip #10 }
  if p^=#10 then
   inc(p);
  { if #0 then set p to nil }
  if p^=#0 then
   p:=nil;
  { return string }
  GetMsgLine[0]:=chr(i);
end;


function TMessage.GetPChar(nr:longint):pchar;
begin
  if (nr div 1000 < msgparts) and
     (nr mod 1000 <  msgidxmax[nr div 1000]) then
    GetPChar:=msgidx[nr div 1000]^[nr mod 1000]
  else
    GetPChar:='';
end;

function TMessage.SetVerbosity(nr:longint;newstate:tmsgstate):boolean;
var
  i: longint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  oldstate : tmsgstate;
  is_global : boolean;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  result:=false;
  i:=nr div 1000;
  if (i < low(msgstates)) or
     (i > msgparts) then
    exit;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if (nr mod 1000 < msgidxmax[i]) then
    begin
      is_global:=(ord(newstate) and ms_global_mask) <> 0;
      oldstate:=msgstates[i]^[nr mod 1000];
      if not is_global then
        newstate:= tmsgstate((ord(newstate) and ms_local_mask) or (ord(oldstate) and ms_global_mask));
      if newstate<>oldstate then
        has_local_changes:=true;
      msgstates[i]^[nr mod 1000]:=newstate;
      result:=true;
    end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  msgstates[i]^[nr mod 1000]:=ms_off;
  result:=true;
>>>>>>> graemeg/cpstrnew
end;

{
function TMessage.ClearVerbosity(nr:longint):boolean;
begin
  ClearVerbosity:=SetVerbosity(nr,ms_off);
end;
}

<<<<<<< HEAD
<<<<<<< HEAD
function TMessage.Get(nr:longint;const args:array of TMsgStr):ansistring;
=======
function TMessage.Get(nr:longint;const args:array of string):ansistring;
>>>>>>> graemeg/fixes_2_2
=======
function TMessage.Get(nr:longint;const args:array of string):ansistring;
>>>>>>> origin/fixes_2_2
var
  hp : pchar;
begin
  if (nr div 1000 < msgparts) and
     (nr mod 1000 <  msgidxmax[nr div 1000]) then
    hp:=msgidx[nr div 1000]^[nr mod 1000]
  else
    hp:=nil;
  if hp=nil then
    Get:='msg nr '+tostr(nr)
  else
    Get:=MsgReplace(system.strpas(hp),args);
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2_2
end;

procedure TMessage.ResetStates;
var
  i,j,glob : longint;
  state : tmsgstate;
begin
  if not has_local_changes then
    exit;
  for i:=1 to msgparts do
    for j:=0 to msgidxmax[i] - 1 do
      begin
        state:=msgstates[i]^[j];
        glob:=(ord(state) and ms_global_mask) shr ms_shift;
        state:=tmsgstate((glob shl ms_shift) or glob);
        msgstates[i]^[j]:=state;
      end;
  has_local_changes:=false;
=======
>>>>>>> graemeg/fixes_2_2
end;


end.
