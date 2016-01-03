{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2005 by Florian Klaempfl,
    member of the Free Pascal development team.

    This file implements support routines for WideStrings/Unicode with FPC

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
  This file contains the implementation of the WideString type,
  and all things that are needed for it.
  WideString is defined as a 'silent' pwidechar :
  a pwidechar that points to :

  @-8  : SizeInt for reference count;
  @-4  : SizeInt for size; size=number of bytes, not the number of chars. Divide or multiply
         with sizeof(WideChar) to convert. This is needed to be compatible with Delphi and
         Windows COM BSTR.
  @    : String + Terminating #0;
  Pwidechar(Widestring) is a valid typecast.
  So WS[i] is converted to the address @WS+i-1.

  Constants should be assigned a reference count of -1
  Meaning that they can't be disposed of.
}

Type
  PWideRec = ^TWideRec;
  TWideRec = Packed Record
{$ifdef FPC_WINLIKEWIDESTRING}
    Len   : DWord;
{$else FPC_WINLIKEWIDESTRING}
    Ref : SizeInt;
    Len : SizeInt;
{$endif FPC_WINLIKEWIDESTRING}
    First : WideChar;
  end;

Const
  WideRecLen = SizeOf(TWideRec);
  WideFirstOff = SizeOf(TWideRec)-sizeof(WideChar);

{
  Default WideChar <-> Char conversion is to only convert the
  lower 127 chars, all others are translated to spaces.

  These routines can be overwritten for the Current Locale
}

procedure DefaultWide2AnsiMove(source:pwidechar;var dest:ansistring;len:SizeInt);
var
  i : SizeInt;
begin
  setlength(dest,len);
  for i:=1 to len do
    begin
      if word(source^)<256 then
        dest[i]:=char(word(source^))
      else
        dest[i]:='?';
      inc(source);
    end;
end;


procedure DefaultAnsi2WideMove(source:pchar;var dest:widestring;len:SizeInt);
var
  i : SizeInt;
begin
  setlength(dest,len);
  for i:=1 to len do
    begin
      dest[i]:=widechar(byte(source^));
      inc(source);
    end;
end;


Procedure GetWideStringManager (Var Manager : TWideStringManager);
begin
  manager:=widestringmanager;
end;


Procedure SetWideStringManager (Const New : TWideStringManager; Var Old: TWideStringManager);
begin
  Old:=widestringmanager;
  widestringmanager:=New;
end;


Procedure SetWideStringManager (Const New : TWideStringManager);
begin
  widestringmanager:=New;
end;

(*
Procedure UniqueWideString(Var S : WideString); [Public,Alias : 'FPC_WIDESTR_UNIQUE'];
{
  Make sure reference count of S is 1,
  using copy-on-write semantics.
}

begin
end;
*)


{****************************************************************************
                    Internal functions, not in interface.
****************************************************************************}


procedure WideStringError;
  begin
    HandleErrorFrame(204,get_frame);
  end;


{$ifdef WideStrDebug}
Procedure DumpWideRec(S : Pointer);
begin
  If S=Nil then
    Writeln ('String is nil')
  Else
    Begin
      With PWideRec(S-WideFirstOff)^ do
       begin
         Write   ('(Len:',len);
         Writeln (' Ref: ',ref,')');
       end;
    end;
end;
{$endif}


Function NewWideString(Len : SizeInt) : Pointer;
{
  Allocate a new WideString on the heap.
  initialize it to zero length and reference count 1.
}
Var
  P : Pointer;
begin
{$ifdef MSWINDOWS}
  if winwidestringalloc then
    begin
      P:=SysAllocStringLen(nil,Len);
      if P=nil then
        WideStringError;
    end
  else
{$endif MSWINDOWS}
    begin
      GetMem(P,Len*sizeof(WideChar)+WideRecLen);
      If P<>Nil then
        begin
         PWideRec(P)^.Len:=Len*2;     { Initial length }
{$ifndef FPC_WINLIKEWIDESTRING}
         PWideRec(P)^.Ref:=1;         { Initial Refcount }
{$endif FPC_WINLIKEWIDESTRING}
         PWideRec(P)^.First:=#0;      { Terminating #0 }
         inc(p,WideFirstOff);         { Points to string now }
        end
      else
        WideStringError;
    end;
  NewWideString:=P;
end;


Procedure DisposeWideString(Var S : Pointer);
{
  Deallocates a WideString From the heap.
}
begin
  If S=Nil then
    exit;
{$ifndef MSWINDOWS}
  Dec (S,WideFirstOff);
  Freemem(S);
{$else MSWINDOWS}
  if winwidestringalloc then
    SysFreeString(S)
  else
    begin
      Dec (S,WideFirstOff);
      Freemem(S);
    end;
{$endif MSWINDOWS}
  S:=Nil;
end;

{$ifdef FPC_WINLIKEWIDESTRING}
var
  __data_start: byte; external name '__data_start__';
  __data_end: byte; external name '__data_end__';

function IsWideStringConstant(S: pointer): boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
{
  Returns True if widestring is constant (located in .data section);
}
begin
  Result:=(S>=@__data_start) and (S<@__data_end);
end;
{$endif FPC_WINLIKEWIDESTRING}

Procedure fpc_WideStr_Decr_Ref (Var S : Pointer);[Public,Alias:'FPC_WIDESTR_DECR_REF']; compilerproc;
{
  Decreases the ReferenceCount of a non constant widestring;
  If the reference count is zero, deallocate the string;
}
Type
  pSizeInt = ^SizeInt;
{$ifndef FPC_WINLIKEWIDESTRING}
Var
  l : pSizeInt;
{$endif FPC_WINLIKEWIDESTRING}
Begin
  { Zero string }
  if S=Nil then
    exit;
{$ifndef FPC_WINLIKEWIDESTRING}
  { check for constant strings ...}
  l:=@PWideRec(S-WideFirstOff)^.Ref;
  if l^<0 then
    exit;

  { declocked does a MT safe dec and returns true, if the counter is 0 }
  if declocked(l^) then
    { Ref count dropped to zero ...
      ... remove }
{$else}
  if not IsWideStringConstant(S) then
{$endif FPC_WINLIKEWIDESTRING}
    DisposeWideString(S);
end;

{ alias for internal use }
Procedure fpc_WideStr_Decr_Ref (Var S : Pointer);[external name 'FPC_WIDESTR_DECR_REF'];

Procedure fpc_WideStr_Incr_Ref(Var S : Pointer);[Public,Alias:'FPC_WIDESTR_INCR_REF']; compilerproc;
{$ifdef FPC_WINLIKEWIDESTRING}
  var
    p : pointer;
{$endif FPC_WINLIKEWIDESTRING}
  Begin
    If S=Nil then
      exit;
{$ifdef FPC_WINLIKEWIDESTRING}
    p:=NewWidestring(length(WideString(S)));
    move(s^,p^,(length(WideString(s))+1)*sizeof(widechar)); // double #0 too
    s:=p;
{$else FPC_WINLIKEWIDESTRING}
    { Let's be paranoid : Constant string ??}
    If PWideRec(S-WideFirstOff)^.Ref<0 then
      exit;
    inclocked(PWideRec(S-WideFirstOff)^.Ref);
{$endif FPC_WINLIKEWIDESTRING}
  end;

{ alias for internal use }
Procedure fpc_WideStr_Incr_Ref (Var S : Pointer);[external name 'FPC_WIDESTR_INCR_REF'];

{$ifndef FPC_STRTOSHORTSTRINGPROC}
function fpc_WideStr_To_ShortStr (high_of_res: SizeInt;const S2 : WideString): shortstring;[Public, alias: 'FPC_WIDESTR_TO_SHORTSTR'];  compilerproc;
{
  Converts a WideString to a ShortString;
}
Var
  Size : SizeInt;
  temp : ansistring;
begin
  result:='';
  Size:=Length(S2);
  if Size>0 then
    begin
      If Size>high_of_res then
        Size:=high_of_res;
      widestringmanager.Wide2AnsiMoveProc(PWideChar(S2),temp,Size);
      result:=temp;
    end;
end;
{$else FPC_STRTOSHORTSTRINGPROC}
procedure fpc_WideStr_To_ShortStr (out res: ShortString;const S2 : WideString); [Public, alias: 'FPC_WIDESTR_TO_SHORTSTR'];compilerproc;
{
  Converts a WideString to a ShortString;
}
Var
  Size : SizeInt;
  temp : ansistring;
begin
  res:='';
  Size:=Length(S2);
  if Size>0 then
    begin
      If Size>high(res) then
        Size:=high(res);
      widestringmanager.Wide2AnsiMoveProc(PWideChar(S2),temp,Size);
      res:=temp;
    end;
end;
{$endif FPC_STRTOSHORTSTRINGPROC}


Function fpc_ShortStr_To_WideStr (Const S2 : ShortString): WideString;compilerproc;
{
  Converts a ShortString to a WideString;
}
Var
  Size : SizeInt;
begin
  result:='';
  Size:=Length(S2);
  if Size>0 then
    begin
      widestringmanager.Ansi2WideMoveProc(PChar(@S2[1]),result,Size);
      { Terminating Zero }
      PWideChar(Pointer(fpc_ShortStr_To_WideStr)+Size*sizeof(WideChar))^:=#0;
    end;
end;


Function fpc_WideStr_To_AnsiStr (const S2 : WideString): AnsiString; compilerproc;
{
  Converts a WideString to an AnsiString
}
Var
  Size : SizeInt;
begin
  result:='';
  Size:=Length(S2);
  if Size>0 then
    widestringmanager.Wide2AnsiMoveProc(PWideChar(Pointer(S2)),result,Size);
end;


Function fpc_AnsiStr_To_WideStr (Const S2 : AnsiString): WideString; compilerproc;
{
  Converts an AnsiString to a WideString;
}
Var
  Size : SizeInt;
begin
  result:='';
  Size:=Length(S2);
  if Size>0 then
    widestringmanager.Ansi2WideMoveProc(PChar(S2),result,Size);
end;


Function fpc_PWideChar_To_AnsiStr(const p : pwidechar): ansistring; compilerproc;
var
  Size : SizeInt;
begin
  result:='';
  if p=nil then
    exit;
  Size := IndexWord(p^, -1, 0);
  if Size>0 then
    widestringmanager.Wide2AnsiMoveProc(P,result,Size);
end;


Function fpc_PWideChar_To_WideStr(const p : pwidechar): widestring; compilerproc;
var
  Size : SizeInt;
begin
  result:='';
  if p=nil then
    exit;
  Size := IndexWord(p^, -1, 0);
  Setlength(result,Size);
  if Size>0 then
   begin
      Move(p^,PWideChar(Pointer(result))^,Size*sizeof(WideChar));
      { Terminating Zero }
      PWideChar(Pointer(result)+Size*sizeof(WideChar))^:=#0;
   end;
end;


{$ifndef FPC_STRTOSHORTSTRINGPROC}
Function fpc_PWideChar_To_ShortStr(const p : pwidechar): shortstring; compilerproc;
var
  Size : SizeInt;
  temp: ansistring;
begin
  result:='';
  if p=nil then
    exit;
  Size := IndexWord(p^, $7fffffff, 0);
  if Size>0 then
    begin
      widestringmanager.Wide2AnsiMoveProc(p,temp,Size);
      result:=temp;
    end;
end;
{$else FPC_STRTOSHORTSTRINGPROC}
procedure fpc_PWideChar_To_ShortStr(out res : shortstring;const p : pwidechar); compilerproc;
var
  Size : SizeInt;
  temp: ansistring;
begin
  res:='';
  if p=nil then
    exit;
  Size:=IndexWord(p^, high(PtrInt), 0);
  if Size>0 then
    begin
      widestringmanager.Wide2AnsiMoveProc(p,temp,Size);
      res:=temp;
    end;
end;
{$endif FPC_STRTOSHORTSTRINGPROC}


{ checked against the ansistring routine, 2001-05-27 (FK) }
Procedure fpc_WideStr_Assign (Var S1 : Pointer;S2 : Pointer);[Public,Alias:'FPC_WIDESTR_ASSIGN']; compilerproc;
{
  Assigns S2 to S1 (S1:=S2), taking in account reference counts.
}
begin
{$ifdef FPC_WINLIKEWIDESTRING}
  if S1=S2 then exit;
  if S2<>nil then
    begin
      if IsWideStringConstant(S1) then
        begin
          S1:=NewWidestring(length(WideString(S2)));
          move(s2^,s1^,(length(WideString(s1))+1)*sizeof(widechar));
        end
      else
{$ifdef MSWINDOWS}
        if winwidestringalloc then
          begin
            if SysReAllocStringLen(S1, S2, Length(WideString(S2))) = 0 then
              WideStringError;
          end
        else
{$endif MSWINDOWS}
          begin
            SetLength(WideString(S1),length(WideString(S2)));
            move(s2^,s1^,(length(WideString(s1))+1)*sizeof(widechar));
          end;
    end
  else
    begin
      { Free S1 }
      fpc_widestr_decr_ref (S1);
      S1:=nil;
    end;
{$else FPC_WINLIKEWIDESTRING}
  If S2<>nil then
    If PWideRec(S2-WideFirstOff)^.Ref>0 then
      inclocked(PWideRec(S2-WideFirstOff)^.ref);
  { Decrease the reference count on the old S1 }
  fpc_widestr_decr_ref (S1);
  s1:=s2;
{$endif FPC_WINLIKEWIDESTRING}
end;


{ alias for internal use }
Procedure fpc_WideStr_Assign (Var S1 : Pointer;S2 : Pointer);[external name 'FPC_WIDESTR_ASSIGN'];

{$ifndef STR_CONCAT_PROCS}

function fpc_WideStr_Concat (const S1,S2 : WideString): WideString; compilerproc;
Var
  Size,Location : SizeInt;
  pc : pwidechar;
begin
  { only assign if s1 or s2 is empty }
  if (S1='') then
    begin
      result:=s2;
      exit;
    end;
  if (S2='') then
    begin
      result:=s1;
      exit;
    end;
  Location:=Length(S1);
  Size:=length(S2);
  SetLength(result,Size+Location);
  pc:=pwidechar(result);
  Move(S1[1],pc^,Location*sizeof(WideChar));
  inc(pc,location);
  Move(S2[1],pc^,(Size+1)*sizeof(WideChar));
end;


function fpc_WideStr_Concat_multi (const sarr:array of Widestring): widestring; compilerproc;
Var
  i  : Longint;
  p  : pointer;
  pc : pwidechar;
  Size,NewSize : SizeInt;
begin
  { First calculate size of the result so we can do
    a single call to SetLength() }
  NewSize:=0;
  for i:=low(sarr) to high(sarr) do
    inc(Newsize,length(sarr[i]));
  SetLength(result,NewSize);
  pc:=pwidechar(result);
  for i:=low(sarr) to high(sarr) do
    begin
      p:=pointer(sarr[i]);
      if assigned(p) then
        begin
          Size:=length(widestring(p));
          Move(pwidechar(p)^,pc^,(Size+1)*sizeof(WideChar));
          inc(pc,size);
        end;
    end;
end;

{$else STR_CONCAT_PROCS}

procedure fpc_WideStr_Concat (var DestS:Widestring;const S1,S2 : WideString); compilerproc;
Var
  Size,Location : SizeInt;
  same : boolean;
begin
  { only assign if s1 or s2 is empty }
  if (S1='') then
    begin
      DestS:=s2;
      exit;
    end;
  if (S2='') then
    begin
      DestS:=s1;
      exit;
    end;
  Location:=Length(S1);
  Size:=length(S2);
  { Use Pointer() typecasts to prevent extra conversion code }
  if Pointer(DestS)=Pointer(S1) then
    begin
      same:=Pointer(S1)=Pointer(S2);
      SetLength(DestS,Size+Location);
      if same then
        Move(Pointer(DestS)^,(Pointer(DestS)+Location*sizeof(WideChar))^,(Size)*sizeof(WideChar))
      else
        Move(Pointer(S2)^,(Pointer(DestS)+Location*sizeof(WideChar))^,(Size+1)*sizeof(WideChar));
    end
  else if Pointer(DestS)=Pointer(S2) then
    begin
      SetLength(DestS,Size+Location);
      Move(Pointer(DestS)^,(Pointer(DestS)+Location*sizeof(WideChar))^,(Size+1)*sizeof(WideChar));
      Move(Pointer(S1)^,Pointer(DestS)^,Location*sizeof(WideChar));
    end
  else
    begin
      DestS:='';
      SetLength(DestS,Size+Location);
      Move(Pointer(S1)^,Pointer(DestS)^,Location*sizeof(WideChar));
      Move(Pointer(S2)^,(Pointer(DestS)+Location*sizeof(WideChar))^,(Size+1)*sizeof(WideChar));
    end;
end;


procedure fpc_WideStr_Concat_multi (var DestS:Widestring;const sarr:array of Widestring); compilerproc;
Var
  i           : Longint;
  p,pc        : pointer;
  Size,NewLen : SizeInt;
{$ifndef FPC_WINLIKEWIDESTRING}
  lowstart    : longint;
  destcopy    : pointer;
  OldDestLen  : SizeInt;
{$else FPC_WINLIKEWIDESTRING}
  DestTmp     : Widestring;
{$endif FPC_WINLIKEWIDESTRING}
begin
  if high(sarr)=0 then
    begin
      DestS:='';
      exit;
    end;
{$ifndef FPC_WINLIKEWIDESTRING}
  destcopy:=nil;
  lowstart:=low(sarr);
  if Pointer(DestS)=Pointer(sarr[lowstart]) then
    inc(lowstart);
  { Check for another reuse, then we can't use
    the append optimization }
  for i:=lowstart to high(sarr) do
    begin
      if Pointer(DestS)=Pointer(sarr[i]) then
        begin
          { if DestS is used somewhere in the middle of the expression,
            we need to make sure the original string still exists after
            we empty/modify DestS.
            This trick only works with reference counted strings. Therefor
            this optimization is disabled for WINLIKEWIDESTRING }
          destcopy:=pointer(dests);
          fpc_WideStr_Incr_Ref(destcopy);
          lowstart:=low(sarr);
          break;
        end;
    end;
  { Start with empty DestS if we start with concatting
    the first array element }
  if lowstart=low(sarr) then
    DestS:='';
  OldDestLen:=length(DestS);
  { Calculate size of the result so we can do
    a single call to SetLength() }
  NewLen:=0;
  for i:=low(sarr) to high(sarr) do
    inc(NewLen,length(sarr[i]));
  SetLength(DestS,NewLen);
  { Concat all strings, except the string we already
    copied in DestS }
  pc:=Pointer(DestS)+OldDestLen*sizeof(WideChar);
  for i:=lowstart to high(sarr) do
    begin
      p:=pointer(sarr[i]);
      if assigned(p) then
        begin
          Size:=length(widestring(p));
          Move(p^,pc^,(Size+1)*sizeof(WideChar));
          inc(pc,size*sizeof(WideChar));
        end;
    end;
  fpc_WideStr_Decr_Ref(destcopy);
{$else FPC_WINLIKEWIDESTRING}
  { First calculate size of the result so we can do
    a single call to SetLength() }
  NewLen:=0;
  for i:=low(sarr) to high(sarr) do
    inc(NewLen,length(sarr[i]));
  SetLength(DestTmp,NewLen);
  pc:=pwidechar(DestTmp);
  for i:=low(sarr) to high(sarr) do
    begin
      p:=pointer(sarr[i]);
      if assigned(p) then
        begin
          Size:=length(widestring(p));
          Move(p^,pc^,(Size+1)*sizeof(WideChar));
          inc(pc,size*sizeof(WideChar));
        end;
    end;
  DestS:=DestTmp;
{$endif FPC_WINLIKEWIDESTRING}
end;

{$endif STR_CONCAT_PROCS}

Function fpc_Char_To_WChar(const c : Char): WideChar; compilerproc;
var
  w: widestring;
begin
  widestringmanager.Ansi2WideMoveProc(@c, w, 1);
  fpc_Char_To_WChar:= w[1];
end;



Function fpc_Char_To_WideStr(const c : Char): WideString; compilerproc;
{
  Converts a Char to a WideString;
}
begin
  Setlength(fpc_Char_To_WideStr,1);
  fpc_Char_To_WideStr[1]:=c;
  { Terminating Zero }
  PWideChar(Pointer(fpc_Char_To_WideStr)+sizeof(WideChar))^:=#0;
end;


Function fpc_WChar_To_Char(const c : WideChar): Char; compilerproc;
{
  Converts a WideChar to a Char;
}
var
  s: ansistring;
begin
  widestringmanager.Wide2AnsiMoveProc(@c, s, 1);
  if length(s)=1 then
    fpc_WChar_To_Char:= s[1]
  else
    fpc_WChar_To_Char:='?';
end;


Function fpc_WChar_To_WideStr(const c : WideChar): WideString; compilerproc;
{
  Converts a WideChar to a WideString;
}
begin
  Setlength (fpc_WChar_To_WideStr,1);
  fpc_WChar_To_WideStr[1]:= c;
end;


Function fpc_WChar_To_AnsiStr(const c : WideChar): AnsiString; compilerproc;
{
  Converts a WideChar to a AnsiString;
}
begin
  widestringmanager.Wide2AnsiMoveProc(@c, fpc_WChar_To_AnsiStr, 1);
end;


{$ifndef FPC_STRTOSHORTSTRINGPROC}
Function fpc_WChar_To_ShortStr(const c : WideChar): ShortString; compilerproc;
{
  Converts a WideChar to a ShortString;
}
var
  s: ansistring;
begin
  widestringmanager.Wide2AnsiMoveProc(@c, s, 1);
  fpc_WChar_To_ShortStr:= s;
end;
{$else FPC_STRTOSHORTSTRINGPROC}
procedure fpc_WChar_To_ShortStr(out res : shortstring;const c : WideChar) compilerproc;
{
  Converts a WideChar to a ShortString;
}
var
  s: ansistring;
begin
  widestringmanager.Wide2AnsiMoveProc(@c,s,1);
  res:=s;
end;
{$endif FPC_STRTOSHORTSTRINGPROC}


Function fpc_PChar_To_WideStr(const p : pchar): WideString; compilerproc;
Var
  L : SizeInt;
begin
  if (not assigned(p)) or (p[0]=#0) Then
  begin
    fpc_pchar_to_widestr := '';
    exit;
  end;
  l:=IndexChar(p^,-1,#0);
  widestringmanager.Ansi2WideMoveProc(P,fpc_PChar_To_WideStr,l);
end;


Function fpc_CharArray_To_WideStr(const arr: array of char; zerobased: boolean = true): WideString; compilerproc;
var
  i  : SizeInt;
begin
  if (zerobased) then
    begin
      if (arr[0]=#0) Then
      begin
        fpc_chararray_to_widestr := '';
        exit;
      end;
      i:=IndexChar(arr,high(arr)+1,#0);
      if i = -1 then
        i := high(arr)+1;
    end
  else
    i := high(arr)+1;
  SetLength(fpc_CharArray_To_WideStr,i);
  widestringmanager.Ansi2WideMoveProc (pchar(@arr),fpc_CharArray_To_WideStr,i);
end;


{$ifndef FPC_STRTOSHORTSTRINGPROC}
function fpc_WideCharArray_To_ShortStr(const arr: array of widechar; zerobased: boolean = true): shortstring;[public,alias:'FPC_WIDECHARARRAY_TO_SHORTSTR']; compilerproc;
var
  l: longint;
 index: longint;
 len: byte;
 temp: ansistring;
begin
  l := high(arr)+1;
  if l>=256 then
    l:=255
  else if l<0 then
    l:=0;
  if zerobased then
    begin
      index:=IndexWord(arr[0],l,0);
      if (index < 0) then
        len := l
      else
        len := index;
    end
  else
    len := l;
  widestringmanager.Wide2AnsiMoveProc (pwidechar(@arr),temp,len);
  fpc_WideCharArray_To_ShortStr := temp;
end;
{$else FPC_STRTOSHORTSTRINGPROC}
procedure fpc_WideCharArray_To_ShortStr(out res : shortstring;const arr: array of widechar; zerobased: boolean = true);[public,alias:'FPC_WIDECHARARRAY_TO_SHORTSTR']; compilerproc;
var
  l: longint;
  index: ptrint;
  len: byte;
  temp: ansistring;
begin
  l := high(arr)+1;
  if l>=high(res)+1 then
    l:=high(res)
  else if l<0 then
    l:=0;
  if zerobased then
    begin
      index:=IndexWord(arr[0],l,0);
      if index<0 then
        len:=l
      else
        len:=index;
    end
  else
    len:=l;
  widestringmanager.Wide2AnsiMoveProc (pwidechar(@arr),temp,len);
  res:=temp;
end;
{$endif FPC_STRTOSHORTSTRINGPROC}

Function fpc_WideCharArray_To_AnsiStr(const arr: array of widechar; zerobased: boolean = true): AnsiString; compilerproc;
var
  i  : SizeInt;
begin
  if (zerobased) then
    begin
      i:=IndexWord(arr,high(arr)+1,0);
      if i = -1 then
        i := high(arr)+1;
    end
  else
    i := high(arr)+1;
  SetLength(fpc_WideCharArray_To_AnsiStr,i);
  widestringmanager.Wide2AnsiMoveProc (pwidechar(@arr),fpc_WideCharArray_To_AnsiStr,i);
end;

Function fpc_WideCharArray_To_WideStr(const arr: array of widechar; zerobased: boolean = true): WideString; compilerproc;
var
  i  : SizeInt;
begin
  if (zerobased) then
    begin
      i:=IndexWord(arr,high(arr)+1,0);
      if i = -1 then
        i := high(arr)+1;
    end
  else
    i := high(arr)+1;
  SetLength(fpc_WideCharArray_To_WideStr,i);
  Move(arr[0], Pointer(fpc_WideCharArray_To_WideStr)^,i*sizeof(WideChar));
end;

{$ifndef FPC_STRTOCHARARRAYPROC}

{ inside the compiler, the resulttype is modified to that of the actual }
{ chararray we're converting to (JM)                                    }
function fpc_widestr_to_chararray(arraysize: SizeInt; const src: WideString): fpc_big_chararray;[public,alias: 'FPC_WIDESTR_TO_CHARARRAY']; compilerproc;
var
  len: SizeInt;
  temp: ansistring;
begin
  len := length(src);
  { make sure we don't dereference src if it can be nil (JM) }
  if len > 0 then
    widestringmanager.wide2ansimoveproc(pwidechar(@src[1]),temp,len);
  len := length(temp);
  if len > arraysize then
    len := arraysize;
{$r-}
  move(temp[1],fpc_widestr_to_chararray[0],len);
  fillchar(fpc_widestr_to_chararray[len],arraysize-len,0);
{$ifdef RangeCheckWasOn}
{$r+}
{$endif}
end;


{ inside the compiler, the resulttype is modified to that of the actual }
{ widechararray we're converting to (JM)                                }
function fpc_widestr_to_widechararray(arraysize: SizeInt; const src: WideString): fpc_big_widechararray;[public,alias: 'FPC_WIDESTR_TO_WIDECHARARRAY']; compilerproc;
var
  len: SizeInt;
begin
  len := length(src);
  if len > arraysize then
    len := arraysize;
{$r-}
  { make sure we don't try to access element 1 of the ansistring if it's nil }
  if len > 0 then
    move(src[1],fpc_widestr_to_widechararray[0],len*SizeOf(WideChar));
  fillchar(fpc_widestr_to_widechararray[len],(arraysize-len)*SizeOf(WideChar),0);
{$ifdef RangeCheckWasOn}
{$r+}
{$endif}
end;


{ inside the compiler, the resulttype is modified to that of the actual }
{ chararray we're converting to (JM)                                    }
function fpc_ansistr_to_widechararray(arraysize: SizeInt; const src: AnsiString): fpc_big_widechararray;[public,alias: 'FPC_ANSISTR_TO_WIDECHARARRAY']; compilerproc;
var
  len: SizeInt;
  temp: widestring;
begin
  len := length(src);
  { make sure we don't dereference src if it can be nil (JM) }
  if len > 0 then
    widestringmanager.ansi2widemoveproc(pchar(@src[1]),temp,len);
  len := length(temp);
  if len > arraysize then
    len := arraysize;

{$r-}
  move(temp[1],fpc_ansistr_to_widechararray[0],len*sizeof(widechar));
  fillchar(fpc_ansistr_to_widechararray[len],(arraysize-len)*SizeOf(WideChar),0);
{$ifdef RangeCheckWasOn}
{$r+}
{$endif}
end;

function fpc_shortstr_to_widechararray(arraysize: SizeInt; const src: ShortString): fpc_big_widechararray;[public,alias: 'FPC_SHORTSTR_TO_WIDECHARARRAY']; compilerproc;
var
  len: longint;
  temp : widestring;
begin
  len := length(src);
  { make sure we don't access char 1 if length is 0 (JM) }
  if len > 0 then
    widestringmanager.ansi2widemoveproc(pchar(@src[1]),temp,len);
  len := length(temp);
  if len > arraysize then
    len := arraysize;
{$r-}
  move(temp[1],fpc_shortstr_to_widechararray[0],len*sizeof(widechar));
  fillchar(fpc_shortstr_to_widechararray[len],(arraysize-len)*SizeOf(WideChar),0);
{$ifdef RangeCheckWasOn}
{$r+}
{$endif}
end;

{$else ndef FPC_STRTOCHARARRAYPROC}

procedure fpc_widestr_to_chararray(out res: array of char; const src: WideString); compilerproc;
var
  len: SizeInt;
  temp: ansistring;
begin
  len := length(src);
  { make sure we don't dereference src if it can be nil (JM) }
  if len > 0 then
    widestringmanager.wide2ansimoveproc(pwidechar(@src[1]),temp,len);
  len := length(temp);
  if len > length(res) then
    len := length(res);
{$r-}
  move(temp[1],res[0],len);
  fillchar(res[len],length(res)-len,0);
{$ifdef RangeCheckWasOn}
{$r+}
{$endif}
end;


procedure fpc_widestr_to_widechararray(out res: array of widechar; const src: WideString); compilerproc;
var
  len: SizeInt;
begin
  len := length(src);
  if len > length(res) then
    len := length(res);
{$r-}
  { make sure we don't try to access element 1 of the ansistring if it's nil }
  if len > 0 then
    move(src[1],res[0],len*SizeOf(WideChar));
  fillchar(res[len],(length(res)-len)*SizeOf(WideChar),0);
{$ifdef RangeCheckWasOn}
{$r+}
{$endif}
end;


procedure fpc_ansistr_to_widechararray(out res: array of widechar; const src: AnsiString); compilerproc;
var
  len: SizeInt;
  temp: widestring;
begin
  len := length(src);
  { make sure we don't dereference src if it can be nil (JM) }
  if len > 0 then
    widestringmanager.ansi2widemoveproc(pchar(@src[1]),temp,len);
  len := length(temp);
  if len > length(res) then
    len := length(res);

{$r-}
  move(temp[1],res[0],len*sizeof(widechar));
  fillchar(res[len],(length(res)-len)*SizeOf(WideChar),0);
{$ifdef RangeCheckWasOn}
{$r+}
{$endif}
end;

procedure fpc_shortstr_to_widechararray(out res: array of widechar; const src: ShortString); compilerproc;
var
  len: longint;
  temp : widestring;
begin
  len := length(src);
  { make sure we don't access char 1 if length is 0 (JM) }
  if len > 0 then
    widestringmanager.ansi2widemoveproc(pchar(@src[1]),temp,len);
  len := length(temp);
  if len > length(res) then
    len := length(res);
{$r-}
  move(temp[1],res[0],len*sizeof(widechar));
  fillchar(res[len],(length(res)-len)*SizeOf(WideChar),0);
{$ifdef RangeCheckWasOn}
{$r+}
{$endif}
end;

{$endif ndef FPC_STRTOCHARARRAYPROC}

Function fpc_WideStr_Compare(const S1,S2 : WideString): SizeInt;[Public,Alias : 'FPC_WIDESTR_COMPARE']; compilerproc;
{
  Compares 2 WideStrings;
  The result is
   <0 if S1<S2
   0 if S1=S2
   >0 if S1>S2
}
Var
  MaxI,Temp : SizeInt;
begin
  if pointer(S1)=pointer(S2) then
   begin
     fpc_WideStr_Compare:=0;
     exit;
   end;
  Maxi:=Length(S1);
  temp:=Length(S2);
  If MaxI>Temp then
   MaxI:=Temp;
  Temp:=CompareWord(S1[1],S2[1],MaxI);
  if temp=0 then
   temp:=Length(S1)-Length(S2);
  fpc_WideStr_Compare:=Temp;
end;

Function fpc_WideStr_Compare_Equal(const S1,S2 : WideString): SizeInt;[Public,Alias : 'FPC_WIDESTR_COMPARE_EQUAL']; compilerproc;
{
  Compares 2 WideStrings for equality only;
  The result is
   0 if S1=S2
   <>0 if S1<>S2
}
Var
  MaxI : SizeInt;
begin
  if pointer(S1)=pointer(S2) then
    exit(0);
  Maxi:=Length(S1);
  If MaxI<>Length(S2) then
    exit(-1)
  else
    exit(CompareWord(S1[1],S2[1],MaxI));
end;

Procedure fpc_WideStr_CheckZero(p : pointer);[Public,Alias : 'FPC_WIDESTR_CHECKZERO']; compilerproc;
begin
  if p=nil then
    HandleErrorFrame(201,get_frame);
end;


Procedure fpc_WideStr_CheckRange(len,index : SizeInt);[Public,Alias : 'FPC_WIDESTR_RANGECHECK']; compilerproc;
begin
  if (index>len div 2) or (Index<1) then
    HandleErrorFrame(201,get_frame);
end;

Procedure fpc_WideStr_SetLength(Var S : WideString; l : SizeInt);[Public,Alias : 'FPC_WIDESTR_SETLENGTH']; compilerproc;
{
  Sets The length of string S to L.
  Makes sure S is unique, and contains enough room.
}
Var
  Temp : Pointer;
  movelen: SizeInt;
begin
   if (l>0) then
    begin
      if Pointer(S)=nil then
       begin
         { Need a complete new string...}
         Pointer(s):=NewWideString(l);
       end
      { windows doesn't support reallocing widestrings, this code
        is anyways subject to be removed because widestrings shouldn't be
        ref. counted anymore (FK) }
      else
        if
{$ifdef MSWINDOWS}
              not winwidestringalloc and
{$endif MSWINDOWS}
{$ifdef FPC_WINLIKEWIDESTRING}
              not IsWideStringConstant(pointer(S))
{$else}
              (PWideRec(Pointer(S)-WideFirstOff)^.Ref = 1)
{$endif FPC_WINLIKEWIDESTRING}
              then
        begin
          Dec(Pointer(S),WideFirstOff);
          if SizeUInt(L*sizeof(WideChar)+WideRecLen)>MemSize(Pointer(S)) then
              reallocmem(pointer(S), L*sizeof(WideChar)+WideRecLen);
          Inc(Pointer(S), WideFirstOff);
        end
      else
        begin
          { Reallocation is needed... }
          Temp:=Pointer(NewWideString(L));
          if Length(S)>0 then
            begin
              if l < succ(length(s)) then
                movelen := l
              { also move terminating null }
              else
                movelen := succ(length(s));
              Move(Pointer(S)^,Temp^,movelen * Sizeof(WideChar));
            end;
          fpc_widestr_decr_ref(Pointer(S));
          Pointer(S):=Temp;
        end;
      { Force nil termination in case it gets shorter }
      PWord(Pointer(S)+l*sizeof(WideChar))^:=0;
{$ifdef MSWINDOWS}
      if not winwidestringalloc then
{$endif MSWINDOWS}
        PWideRec(Pointer(S)-WideFirstOff)^.Len:=l*sizeof(WideChar);
    end
  else
    begin
      { Length=0 }
      if Pointer(S)<>nil then
        fpc_widestr_decr_ref (Pointer(S));
      Pointer(S):=Nil;
    end;
end;

{*****************************************************************************
                     Public functions, In interface.
*****************************************************************************}

function WideCharToString(S : PWideChar) : AnsiString;
  begin
     result:=WideCharLenToString(s,Length(WideString(s)));
  end;

function StringToWideChar(const Src : AnsiString;Dest : PWideChar;DestSize : SizeInt) : PWideChar;
  var
    temp:widestring;
  begin
     widestringmanager.Ansi2WideMoveProc(PChar(Src),temp,Length(Src));
     if Length(temp)<DestSize then
       move(temp[1],Dest^,Length(temp)*SizeOf(WideChar))
     else
       move(temp[1],Dest^,(DestSize-1)*SizeOf(WideChar));

     Dest[DestSize-1]:=#0;

     result:=Dest;

  end;

function WideCharLenToString(S : PWideChar;Len : SizeInt) : AnsiString;
  begin
     //SetLength(result,Len);
     widestringmanager.Wide2AnsiMoveproc(S,result,Len);
  end;

procedure WideCharLenToStrVar(Src : PWideChar;Len : SizeInt;out Dest : AnsiString);
  begin
     Dest:=WideCharLenToString(Src,Len);
  end;

procedure WideCharToStrVar(S : PWideChar;out Dest : AnsiString);
  begin
     Dest:=WideCharToString(S);
  end;


Function fpc_widestr_Unique(Var S : Pointer): Pointer; [Public,Alias : 'FPC_WIDESTR_UNIQUE']; compilerproc;
{$ifdef FPC_WINLIKEWIDESTRING}
  begin
    pointer(result) := pointer(s);
  end;
{$else FPC_WINLIKEWIDESTRING}
{
  Make sure reference count of S is 1,
  using copy-on-write semantics.
}
Var
  SNew : Pointer;
  L    : SizeInt;
begin
  pointer(result) := pointer(s);
  If Pointer(S)=Nil then
    exit;
  if PWideRec(Pointer(S)-WideFirstOff)^.Ref<>1 then
   begin
     L:=PWideRec(Pointer(S)-WideFirstOff)^.len div sizeof(WideChar);
     SNew:=NewWideString (L);
     Move (PWideChar(S)^,SNew^,(L+1)*sizeof(WideChar));
     PWideRec(SNew-WideFirstOff)^.len:=L * sizeof(WideChar);
     fpc_widestr_decr_ref (Pointer(S));  { Thread safe }
     pointer(S):=SNew;
     pointer(result):=SNew;
   end;
end;
{$endif FPC_WINLIKEWIDESTRING}


Function Fpc_WideStr_Copy (Const S : WideString; Index,Size : SizeInt) : WideString;compilerproc;
var
  ResultAddress : Pointer;
begin
  ResultAddress:=Nil;
  dec(index);
  if Index < 0 then
    Index := 0;
  { Check Size. Accounts for Zero-length S, the double check is needed because
    Size can be maxint and will get <0 when adding index }
  if (Size>Length(S)) or
     (Index+Size>Length(S)) then
   Size:=Length(S)-Index;
  If Size>0 then
   begin
     If Index<0 Then
      Index:=0;
     ResultAddress:=Pointer(NewWideString (Size));
     if ResultAddress<>Nil then
      begin
        Move (PWideChar(S)[Index],ResultAddress^,Size*sizeof(WideChar));
        PWideRec(ResultAddress-WideFirstOff)^.Len:=Size*sizeof(WideChar);
        PWideChar(ResultAddress+Size*sizeof(WideChar))^:=#0;
      end;
   end;
  Pointer(fpc_widestr_Copy):=ResultAddress;
end;


Function Pos (Const Substr : WideString; Const Source : WideString) : SizeInt;
var
  i,MaxLen : SizeInt;
  pc : pwidechar;
begin
  Pos:=0;
  if Length(SubStr)>0 then
   begin
     MaxLen:=Length(source)-Length(SubStr);
     i:=0;
     pc:=@source[1];
     while (i<=MaxLen) do
      begin
        inc(i);
        if (SubStr[1]=pc^) and
           (CompareWord(Substr[1],pc^,Length(SubStr))=0) then
         begin
           Pos:=i;
           exit;
         end;
        inc(pc);
      end;
   end;
end;


{ Faster version for a widechar alone }
Function Pos (c : WideChar; Const s : WideString) : SizeInt;
var
  i: SizeInt;
  pc : pwidechar;
begin
  pc:=@s[1];
  for i:=1 to length(s) do
   begin
     if pc^=c then
      begin
        pos:=i;
        exit;
      end;
     inc(pc);
   end;
  pos:=0;
end;


Function Pos (c : WideChar; Const s : AnsiString) : SizeInt;
  begin
    result:=Pos(c,WideString(s));
  end;


Function Pos (c : AnsiString; Const s : WideString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    result:=Pos(WideString(c),s);
  end;


Function Pos (c : ShortString; Const s : WideString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    result:=Pos(WideString(c),s);
  end;


Function Pos (c : WideString; Const s : AnsiString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    result:=Pos(c,WideString(s));
  end;

{ Faster version for a char alone. Must be implemented because   }
{ pos(c: char; const s: shortstring) also exists, so otherwise   }
{ using pos(char,pchar) will always call the shortstring version }
{ (exact match for first argument), also with $h+ (JM)           }
Function Pos (c : Char; Const s : WideString) : SizeInt;
var
  i: SizeInt;
  wc : widechar;
  pc : pwidechar;
begin
  wc:=c;
  pc:=@s[1];
  for i:=1 to length(s) do
   begin
     if pc^=wc then
      begin
        pos:=i;
        exit;
      end;
     inc(pc);
   end;
  pos:=0;
end;



Procedure Delete (Var S : WideString; Index,Size: SizeInt);
Var
  LS : SizeInt;
begin
  If Length(S)=0 then
   exit;
  if index<=0 then
   exit;
  LS:=PWideRec(Pointer(S)-WideFirstOff)^.Len div sizeof(WideChar);
  if (Index<=LS) and (Size>0) then
   begin
     UniqueString (S);
     if Size+Index>LS then
      Size:=LS-Index+1;
     if Index+Size<=LS then
      begin
        Dec(Index);
        Move(PWideChar(S)[Index+Size],PWideChar(S)[Index],(LS-Index-Size+1)*sizeof(WideChar));
      end;
     Setlength(s,LS-Size);
   end;
end;


Procedure Insert (Const Source : WideString; Var S : WideString; Index : SizeInt);
var
  Temp : WideString;
  LS : SizeInt;
begin
  If Length(Source)=0 then
   exit;
  if index <= 0 then
   index := 1;
  Ls:=Length(S);
  if index > LS then
   index := LS+1;
  Dec(Index);
  Pointer(Temp) := NewWideString(Length(Source)+LS);
  SetLength(Temp,Length(Source)+LS);
  If Index>0 then
    move (PWideChar(S)^,PWideChar(Temp)^,Index*sizeof(WideChar));
  Move (PWideChar(Source)^,PWideChar(Temp)[Index],Length(Source)*sizeof(WideChar));
  If (LS-Index)>0 then
    Move(PWideChar(S)[Index],PWideChar(temp)[Length(Source)+index],(LS-Index)*sizeof(WideChar));
  S:=Temp;
end;


function UpCase(const s : WideString) : WideString;
begin
  result:=widestringmanager.UpperWideStringProc(s);
end;


Procedure SetString (Out S : WideString; Buf : PWideChar; Len : SizeInt);
var
  BufLen: SizeInt;
begin
  SetLength(S,Len);
  If (Buf<>Nil) and (Len>0) then
    begin
      BufLen := IndexWord(Buf^, Len+1, 0);
      If (BufLen>0) and (BufLen < Len) then
        Len := BufLen;
      Move (Buf[0],S[1],Len*sizeof(WideChar));
      PWideChar(Pointer(S)+Len*sizeof(WideChar))^:=#0;
    end;
end;


Procedure SetString (Out S : WideString; Buf : PChar; Len : SizeInt);
var
  BufLen: SizeInt;
begin
  SetLength(S,Len);
  If (Buf<>Nil) and (Len>0) then
    begin
      BufLen := IndexByte(Buf^, Len+1, 0);
      If (BufLen>0) and (BufLen < Len) then
        Len := BufLen;
      widestringmanager.Ansi2WideMoveProc(Buf,S,Len);
      //PWideChar(Pointer(S)+Len*sizeof(WideChar))^:=#0;
    end;
end;


Function fpc_Val_Real_WideStr(Const S : WideString; out Code : ValSInt): ValReal; [public, alias:'FPC_VAL_REAL_WIDESTR']; compilerproc;
Var
  SS : String;
begin
  fpc_Val_Real_WideStr := 0;
  if length(S) > 255 then
    code := 256
  else
    begin
      SS := S;
      Val(SS,fpc_Val_Real_WideStr,code);
    end;
end;

{$ifdef FPC_HAS_VALSTR_ENUM}
function fpc_val_enum_widestr(str2ordindex:pointer;const s:widestring;out code:valsint):longint;compilerproc;

var ss:shortstring;

begin
  if length(s)>255 then
    code:=256
  else
    begin
      ss:=s;
      val(ss,fpc_val_enum_widestr,code);
    end;
end;
{$endif FPC_HAS_VALSTR_ENUM}

Function fpc_Val_Currency_WideStr(Const S : WideString; out Code : ValSInt): Currency; [public, alias:'FPC_VAL_CURRENCY_WIDESTR']; compilerproc;
Var
  SS : String;
begin
  if length(S) > 255 then
    begin
      fpc_Val_Currency_WideStr:=0;
      code := 256;
    end
  else
    begin
      SS := S;
      Val(SS,fpc_Val_Currency_WideStr,code);
    end;
end;


Function fpc_Val_UInt_WideStr (Const S : WideString; out Code : ValSInt): ValUInt; [public, alias:'FPC_VAL_UINT_WIDESTR']; compilerproc;
Var
  SS : ShortString;
begin
  fpc_Val_UInt_WideStr := 0;
  if length(S) > 255 then
    code := 256
  else
    begin
      SS := S;
      Val(SS,fpc_Val_UInt_WideStr,code);
    end;
end;


Function fpc_Val_SInt_WideStr (DestSize: SizeInt; Const S : WideString; out Code : ValSInt): ValSInt; [public, alias:'FPC_VAL_SINT_WIDESTR']; compilerproc;
Var
  SS : ShortString;
begin
  fpc_Val_SInt_WideStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
      SS := S;
      fpc_Val_SInt_WideStr := int_Val_SInt_ShortStr(DestSize,SS,Code);
    end;
end;


{$ifndef CPU64}

Function fpc_Val_qword_WideStr (Const S : WideString; out Code : ValSInt): qword; [public, alias:'FPC_VAL_QWORD_WIDESTR']; compilerproc;
Var
  SS : ShortString;
begin
  fpc_Val_qword_WideStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS := S;
       Val(SS,fpc_Val_qword_WideStr,Code);
    end;
end;


Function fpc_Val_int64_WideStr (Const S : WideString; out Code : ValSInt): Int64; [public, alias:'FPC_VAL_INT64_WIDESTR']; compilerproc;
Var
  SS : ShortString;
begin
  fpc_Val_int64_WideStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS := S;
       Val(SS,fpc_Val_int64_WideStr,Code);
    end;
end;

{$endif CPU64}


procedure fpc_WideStr_Float(d : ValReal;len,fr,rt : SizeInt;out s : WideString);compilerproc;
var
  ss : shortstring;
begin
  str_real(len,fr,d,treal_type(rt),ss);
  s:=ss;
end;

{$ifdef FPC_HAS_VALSTR_ENUM}
procedure fpc_widestr_enum(ordinal,len:sizeint;typinfo,ord2strindex:pointer;out s:widestring);compilerproc;

var ss:shortstring;

begin
  fpc_shortstr_enum(ordinal,len,typinfo,ord2strindex,ss);
  s:=ss;
end;
{$endif FPC_HAS_VALSTR_ENUM}

{$ifdef FPC_HAS_STR_CURRENCY}
procedure fpc_WideStr_Currency(c : Currency;len,fr : SizeInt;out s : WideString);compilerproc;
var
  ss : shortstring;
begin
  str(c:len:fr,ss);
  s:=ss;
end;
{$endif FPC_HAS_STR_CURRENCY}

Procedure fpc_WideStr_SInt(v : ValSint; Len : SizeInt; out S : WideString);compilerproc;
Var
  SS : ShortString;
begin
  Str (v:Len,SS);
  S:=SS;
end;


Procedure fpc_WideStr_UInt(v : ValUInt;Len : SizeInt; out S : WideString);compilerproc;
Var
  SS : ShortString;
begin
  str(v:Len,SS);
  S:=SS;
end;


{$ifndef CPU64}

Procedure fpc_WideStr_Int64(v : Int64; Len : SizeInt; out S : WideString);compilerproc;
Var
  SS : ShortString;
begin
  Str (v:Len,SS);
  S:=SS;
end;


Procedure fpc_WideStr_Qword(v : Qword;Len : SizeInt; out S : WideString);compilerproc;
Var
  SS : ShortString;
begin
  str(v:Len,SS);
  S:=SS;
end;

{$endif CPU64}

function UnicodeToUtf8(Dest: PChar; Source: PWideChar; MaxBytes: SizeInt): SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    if assigned(Source) then
      Result:=UnicodeToUtf8(Dest,MaxBytes,Source,IndexWord(Source^,-1,0))
    else
      Result:=0;
  end;


function UnicodeToUtf8(Dest: PChar; MaxDestBytes: SizeUInt; Source: PWideChar; SourceChars: SizeUInt): SizeUInt;
  var
    i,j : SizeUInt;
    w : word;
  begin
    result:=0;
    if source=nil then
      exit;
    i:=0;
    j:=0;
    if assigned(Dest) then
      begin
        while (i<SourceChars) and (j<MaxDestBytes) do
          begin
            w:=word(Source[i]);
            case w of
              0..$7f:
                begin
                  Dest[j]:=char(w);
                  inc(j);
                end;
              $80..$7ff:
                begin
                  if j+1>=MaxDestBytes then
                    break;
                  Dest[j]:=char($c0 or (w shr 6));
                  Dest[j+1]:=char($80 or (w and $3f));
                  inc(j,2);
                end;
              else
                begin
                    if j+2>=MaxDestBytes then
                      break;
                    Dest[j]:=char($e0 or (w shr 12));
                    Dest[j+1]:=char($80 or ((w shr 6)and $3f));
                    Dest[j+2]:=char($80 or (w and $3f));
                    inc(j,3);
                end;
            end;
            inc(i);
          end;

        if j>MaxDestBytes-1 then
          j:=MaxDestBytes-1;

        Dest[j]:=#0;
      end
    else
      begin
        while i<SourceChars do
          begin
            case word(Source[i]) of
              $0..$7f:
                inc(j);
              $80..$7ff:
                inc(j,2);
              else
                inc(j,3);
            end;
            inc(i);
          end;
      end;
    result:=j+1;
  end;


function Utf8ToUnicode(Dest: PWideChar; Source: PChar; MaxChars: SizeInt): SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    if assigned(Source) then
      Result:=Utf8ToUnicode(Dest,MaxChars,Source,strlen(Source))
    else
      Result:=0;
  end;


function Utf8ToUnicode(Dest: PWideChar; MaxDestChars: SizeUInt; Source: PChar; SourceBytes: SizeUInt): SizeUInt;

var
  i,j : SizeUInt;
  w: SizeUInt;
  b : byte;
begin
  if not assigned(Source) then
  begin
    result:=0;
    exit;
  end;
  result:=SizeUInt(-1);
  i:=0;
  j:=0;
  if assigned(Dest) then
    begin
      while (j<MaxDestChars) and (i<SourceBytes) do
        begin
          b:=byte(Source[i]);
          w:=b;
          inc(i);
          // 2 or 3 bytes?
          if b>=$80 then
            begin
              w:=b and $3f;
              if i>=SourceBytes then
                exit;
              // 3 bytes?
              if (b and $20)<>0 then
                begin
                  b:=byte(Source[i]);
                  inc(i);
                  if i>=SourceBytes then
                    exit;
                  if (b and $c0)<>$80 then
                    exit;
                  w:=(w shl 6) or (b and $3f);
                end;
              b:=byte(Source[i]);
              w:=(w shl 6) or (b and $3f);
              if (b and $c0)<>$80 then
                exit;
              inc(i);
            end;
          Dest[j]:=WideChar(w);
          inc(j);
        end;
      if j>=MaxDestChars then j:=MaxDestChars-1;
      Dest[j]:=#0;
    end
  else
    begin
      while i<SourceBytes do
        begin
          b:=byte(Source[i]);
          inc(i);
          // 2 or 3 bytes?
          if b>=$80 then
            begin
              if i>=SourceBytes then
                exit;
              // 3 bytes?
              b := b and $3f;
              if (b and $20)<>0 then
                begin
                  b:=byte(Source[i]);
                  inc(i);
                  if i>=SourceBytes then
                    exit;
                  if (b and $c0)<>$80 then
                    exit;
                end;
              if (byte(Source[i]) and $c0)<>$80 then
                exit;
              inc(i);
            end;
          inc(j);
        end;
    end;
  result:=j+1;
end;


function UTF8Encode(const s : WideString) : UTF8String;
  var
    i : SizeInt;
    hs : UTF8String;
  begin
    result:='';
    if s='' then
      exit;
    SetLength(hs,length(s)*3);
    i:=UnicodeToUtf8(pchar(hs),length(hs)+1,PWideChar(s),length(s));
    if i>0 then
      begin
        SetLength(hs,i-1);
        result:=hs;
      end;
  end;


function UTF8Decode(const s : UTF8String): WideString;
  var
    i : SizeInt;
    hs : WideString;
  begin
    result:='';
    if s='' then
      exit;
    SetLength(hs,length(s));
    i:=Utf8ToUnicode(PWideChar(hs),length(hs)+1,pchar(s),length(s));
    if i>0 then
      begin
        SetLength(hs,i-1);
        result:=hs;
      end;
  end;


function AnsiToUtf8(const s : ansistring): UTF8String;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=Utf8Encode(s);
  end;


function Utf8ToAnsi(const s : UTF8String) : ansistring;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=Utf8Decode(s);
  end;


{ converts an utf-16 code point or surrogate pair to utf-32 }
function utf16toutf32(const S: WideString; const index: SizeInt; out len: longint): UCS4Char; [public, alias: 'FPC_UTF16TOUTF32'];
var
  w: widechar;
begin
  { UTF-16 points in the range #$0-#$D7FF and #$E000-#$FFFF }
  { are the same in UTF-32                                  }
  w:=s[index];
  if (w<=#$d7ff) or
     (w>=#$e000) then
    begin
      result:=UCS4Char(w);
      len:=1;
    end
  { valid surrogate pair? }
  else if (w<=#$dbff) and
          { w>=#$d7ff check not needed, checked above }
          (index<length(s)) and
          (s[index+1]>=#$dc00) and
          (s[index+1]<=#$dfff) then
      { convert the surrogate pair to UTF-32 }
    begin
      result:=(UCS4Char(w)-$d800) shl 10 + (UCS4Char(s[index+1])-$dc00) + $10000;
      len:=2;
    end
  else
    { invalid surrogate -> do nothing }
    begin
      result:=UCS4Char(w);
      len:=1;
    end;
end;


function WideStringToUCS4String(const s : WideString) : UCS4String;
  var
    i, slen,
    destindex : SizeInt;
    len       : longint;
  begin
    slen:=length(s);
    setlength(result,slen+1);
    i:=1;
    destindex:=0;
    while (i<=slen) do
      begin
        result[destindex]:=utf16toutf32(s,i,len);
        inc(destindex);
        inc(i,len);
      end;
    { destindex <= slen (surrogate pairs may have been merged) }
    { destindex+1 for terminating #0 (dynamic arrays are       }
    { implicitely filled with zero)                            }
    setlength(result,destindex+1);
  end;


{ concatenates an utf-32 char to a widestring. S *must* be unique when entering. }
procedure ConcatUTF32ToWideStr(const nc: UCS4Char; var S: WideString; var index: SizeInt);
var
  p : PWideChar;
begin
  { if nc > $ffff, we need two places }
  if (index+ord(nc > $ffff)>length(s)) then
    if (length(s) < 10*256) then
      setlength(s,length(s)+10)
    else
      setlength(s,length(s)+length(s) shr 8);
  { we know that s is unique -> avoid uniquestring calls}
  p:=@s[index];
  if (nc<$ffff) then
    begin
      p^:=widechar(nc);
      inc(index);
    end
  else if (dword(nc)<=$10ffff) then
    begin
      p^:=widechar((nc - $10000) shr 10 + $d800);
      (p+1)^:=widechar((nc - $10000) and $3ff + $dc00);
      inc(index,2);
    end
  else
    { invalid code point }
    begin
      p^:='?';
      inc(index);
    end;
end;


function UCS4StringToWideString(const s : UCS4String) : WideString;
  var
    i        : SizeInt;
    resindex : SizeInt;
  begin
    { skip terminating #0 }
    SetLength(result,length(s)-1);
    resindex:=1;
    for i:=0 to high(s)-1 do
      ConcatUTF32ToWideStr(s[i],result,resindex);
    { adjust result length (may be too big due to growing }
    { for surrogate pairs)                                }
    setlength(result,resindex-1);
  end;

const
  SNoWidestrings = 'This binary has no widestrings support compiled in.';
  SRecompileWithWidestrings = 'Recompile the application with a widestrings-manager in the program uses clause.';

procedure unimplementedwidestring;
  begin
{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
    If IsConsole then
      begin
      Writeln(StdErr,SNoWidestrings);
      Writeln(StdErr,SRecompileWithWidestrings);
      end;
{$endif FPC_HAS_FEATURE_CONSOLEIO}
    HandleErrorFrame(233,get_frame);
  end;

{$warnings off}
function GenericWideCase(const s : WideString) : WideString;
  begin
    unimplementedwidestring;
  end;


function CompareWideString(const s1, s2 : WideString) : PtrInt;
  begin
    unimplementedwidestring;
  end;


function CompareTextWideString(const s1, s2 : WideString): PtrInt;
  begin
    unimplementedwidestring;
  end;


function CharLengthPChar(const Str: PChar): PtrInt;
  begin
    unimplementedwidestring;
  end;
{$warnings on}

procedure initwidestringmanager;
  begin
    fillchar(widestringmanager,sizeof(widestringmanager),0);
{$ifndef HAS_WIDESTRINGMANAGER}
    widestringmanager.Wide2AnsiMoveProc:=@defaultWide2AnsiMove;
    widestringmanager.Ansi2WideMoveProc:=@defaultAnsi2WideMove;
    widestringmanager.UpperWideStringProc:=@GenericWideCase;
    widestringmanager.LowerWideStringProc:=@GenericWideCase;
{$endif HAS_WIDESTRINGMANAGER}
    widestringmanager.CompareWideStringProc:=@CompareWideString;
    widestringmanager.CompareTextWideStringProc:=@CompareTextWideString;
    widestringmanager.CharLengthPCharProc:=@CharLengthPChar;
  end;
