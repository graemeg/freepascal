{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2005 by Florian Klaempfl,
    member of the Free Pascal development team.

    This file implements support routines for WideStrings with FPC

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
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
    Len   : DWord;
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure DefaultWide2AnsiMove(source:pwidechar;var dest:ansistring;cp : TSystemCodePage;len:SizeInt);
var
  i : SizeInt;
  destp: PChar;
begin
  setlength(dest,len);
  destp := PChar(Pointer(dest));
  for i:=1 to len do
    begin
      if word(source^)<256 then
        destp^:=char(word(source^))
      else
        destp^:='?';
      inc(source);
      inc(destp);
    end;
end;


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
procedure DefaultAnsi2WideMove(source:pchar;cp : TSystemCodePage;var dest:widestring;len:SizeInt);
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


{****************************************************************************
                    Internal functions, not in interface.
****************************************************************************}


procedure WideStringError;
  begin
    HandleErrorAddrFrameInd(204,get_pc_addr,get_frame);
  end;


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
         PWideRec(P)^.First:=#0;      { Terminating #0 }
         inc(p,WideFirstOff);         { Points to string now }
        end
      else
        WideStringError;
    end;
  NewWideString:=P;
end;


Procedure fpc_WideStr_Decr_Ref (Var S : Pointer);[Public,Alias:'FPC_WIDESTR_DECR_REF']; compilerproc;
{
  Decreases the ReferenceCount of a non constant widestring;
  If the reference count is zero, deallocate the string;
}
Begin
  If S=Nil then
    exit;
{$ifdef MSWINDOWS}
  if winwidestringalloc then
    SysFreeString(S)
  else
{$endif MSWINDOWS}
    begin
      Dec (S,WideFirstOff);
      Freemem(S);
    end;
  S:=Nil;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Procedure fpc_WideStr_Decr_Ref (Var S : Pointer);[Public,Alias:'FPC_WIDESTR_DECR_REF']; compilerproc;
{
  Decreases the ReferenceCount of a non constant widestring;
  If the reference count is zero, deallocate the string;
}
Begin
  DisposeWideString(S);  { does test for nil }
end;

>>>>>>> graemeg/cpstrnew
{ alias for internal use }
Procedure fpc_WideStr_Decr_Ref (Var S : Pointer);[external name 'FPC_WIDESTR_DECR_REF'];

Procedure fpc_WideStr_Incr_Ref(Var S : Pointer);[Public,Alias:'FPC_WIDESTR_INCR_REF']; compilerproc;
  var
    p : pointer;
  Begin
    If S=Nil then
      exit;
    p:=NewWidestring(length(WideString(S)));
    move(s^,p^,(length(WideString(s))+1)*sizeof(widechar)); // double #0 too
    s:=p;
  end;

{ alias for internal use }
Procedure fpc_WideStr_Incr_Ref (Var S : Pointer);[external name 'FPC_WIDESTR_INCR_REF'];

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
      widestringmanager.Wide2AnsiMoveProc(PWideChar(S2),temp,DefaultSystemCodePage,Size);
      res:=temp;
    end;
end;


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
<<<<<<< HEAD
    widestringmanager.Ansi2WideMoveProc(PChar(@S2[1]),DefaultSystemCodePage,result,Size);
=======
    begin
      widestringmanager.Ansi2WideMoveProc(PChar(@S2[1]),DefaultSystemCodePage,result,Size);
      { Terminating Zero }
      PWideChar(Pointer(fpc_ShortStr_To_WideStr)+Size*sizeof(WideChar))^:=#0;
    end;
>>>>>>> graemeg/cpstrnew
end;


Function fpc_WideStr_To_AnsiStr (const S2 : WideString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}): AnsiString; compilerproc;
{
  Converts a WideString to an AnsiString
}
Var
  Size : SizeInt;
{$ifndef FPC_HAS_CPSTRING}
  cp : TSystemCodePage;
{$endif FPC_HAS_CPSTRING}
begin
{$ifndef FPC_HAS_CPSTRING}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  cp:=DefaultSystemCodePage;
=======
  cp:=$ffff;
>>>>>>> graemeg/cpstrnew
=======
  cp:=$ffff;
>>>>>>> graemeg/cpstrnew
=======
  cp:=$ffff;
>>>>>>> graemeg/cpstrnew
=======
  cp:=$ffff;
>>>>>>> origin/cpstrnew
{$endif FPC_HAS_CPSTRING}
  result:='';
  Size:=Length(S2);
  if Size>0 then
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    cp:=TranslatePlaceholderCP(cp);
=======
    if cp=$ffff then
      cp:=DefaultSystemCodePage;
>>>>>>> graemeg/cpstrnew
=======
    if cp=$ffff then
      cp:=DefaultSystemCodePage;
>>>>>>> graemeg/cpstrnew
=======
    if cp=$ffff then
      cp:=DefaultSystemCodePage;
>>>>>>> graemeg/cpstrnew
=======
    if cp=$ffff then
      cp:=DefaultSystemCodePage;
>>>>>>> origin/cpstrnew
    widestringmanager.Wide2AnsiMoveProc(PWideChar(Pointer(S2)),result,cp,Size);
  end;
end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function fpc_AnsiStr_To_WideStr(Const S2 : RawByteString): WideString; compilerproc;
=======
Function fpc_AnsiStr_To_WideStr (Const S2 : RawByteString): WideString; compilerproc;
>>>>>>> graemeg/cpstrnew
=======
Function fpc_AnsiStr_To_WideStr (Const S2 : RawByteString): WideString; compilerproc;
>>>>>>> graemeg/cpstrnew
=======
Function fpc_AnsiStr_To_WideStr (Const S2 : RawByteString): WideString; compilerproc;
>>>>>>> graemeg/cpstrnew
=======
Function fpc_AnsiStr_To_WideStr (Const S2 : RawByteString): WideString; compilerproc;
>>>>>>> origin/cpstrnew
{
  Converts an AnsiString to a WideString;
}
Var
  Size : SizeInt;
  cp: TSystemCodePage;
begin
  result:='';
  Size:=Length(S2);
  if Size>0 then
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    cp:=TranslatePlaceholderCP(StringCodePage(S2));
=======
    cp:=StringCodePage(S2);
    if cp=$ffff then
      cp:=DefaultSystemCodePage;
>>>>>>> graemeg/cpstrnew
=======
    cp:=StringCodePage(S2);
    if cp=$ffff then
      cp:=DefaultSystemCodePage;
>>>>>>> graemeg/cpstrnew
=======
    cp:=StringCodePage(S2);
    if cp=$ffff then
      cp:=DefaultSystemCodePage;
>>>>>>> graemeg/cpstrnew
=======
    cp:=StringCodePage(S2);
    if cp=$ffff then
      cp:=DefaultSystemCodePage;
>>>>>>> origin/cpstrnew
    widestringmanager.Ansi2WideMoveProc(PChar(S2),cp,result,Size);
  end;
end;


Function fpc_PWideChar_To_WideStr(const p : pwidechar): widestring; compilerproc;
var
  Size : SizeInt;
begin
  result:='';
  if p=nil then
    exit;
  Size := IndexWord(p^, -1, 0);
  Setlength(result,Size);  // zero-terminates
  if Size>0 then
    Move(p^,PWideChar(Pointer(result))^,Size*sizeof(WideChar));
end;


{ checked against the ansistring routine, 2001-05-27 (FK) }
Procedure fpc_WideStr_Assign (Var S1 : Pointer;S2 : Pointer);[Public,Alias:'FPC_WIDESTR_ASSIGN']; compilerproc;
{
  Assigns S2 to S1 (S1:=S2), taking in account reference counts.
}
begin
  if S1=S2 then exit;
  if S2<>nil then
    begin
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
    fpc_widestr_decr_ref (S1);  // will set S1 to nil
end;


{ alias for internal use }
Procedure fpc_WideStr_Assign (Var S1 : Pointer;S2 : Pointer);[external name 'FPC_WIDESTR_ASSIGN'];


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
  DestTmp     : Widestring;
begin
  if high(sarr)=0 then
    begin
      DestS:='';
      exit;
    end;
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
end;



Function fpc_Char_To_WideStr(const c : Char): WideString; compilerproc;
{
  Converts a Char to a WideString;
}
begin
<<<<<<< HEAD
  widestringmanager.Ansi2WideMoveProc(@c,DefaultSystemCodePage,fpc_char_To_WideStr,1);
=======
  Setlength(fpc_Char_To_WideStr,1);
  fpc_Char_To_WideStr[1]:=c;
  { Terminating Zero }
  PWideChar(Pointer(fpc_Char_To_WideStr)+sizeof(WideChar))^:=#0;
end;


Function fpc_WChar_To_WideStr(const c : WideChar): WideString; compilerproc;
{
  Converts a WideChar to a WideString;
}
begin
  Setlength (fpc_WChar_To_WideStr,1);
  fpc_WChar_To_WideStr[1]:= c;
end;


Function fpc_WChar_To_AnsiStr(const c : WideChar{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}): AnsiString; compilerproc;
{
  Converts a WideChar to a AnsiString;
}
begin
  widestringmanager.Wide2AnsiMoveProc(@c, fpc_WChar_To_AnsiStr,{$ifdef FPC_HAS_CPSTRING}cp{$else}TSystemCodePage(0){$endif FPC_HAS_CPSTRING}, 1);
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


Function fpc_UChar_To_WideStr(const c : WideChar): WideString; compilerproc;
{
  Converts a WideChar to a WideString;
}
begin
  Setlength (fpc_UChar_To_WideStr,1);
  fpc_UChar_To_WideStr[1]:= c;
end;


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
  widestringmanager.Ansi2WideMoveProc(P,DefaultSystemCodePage,fpc_PChar_To_WideStr,l);
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
<<<<<<< HEAD
  widestringmanager.Ansi2WideMoveProc(pchar(@arr),DefaultSystemCodePage,fpc_CharArray_To_WideStr,i);
=======
  SetLength(fpc_CharArray_To_WideStr,i);
  widestringmanager.Ansi2WideMoveProc(pchar(@arr),DefaultSystemCodePage,fpc_CharArray_To_WideStr,i);
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
>>>>>>> graemeg/cpstrnew
end;


procedure fpc_widestr_to_chararray(out res: array of char; const src: WideString); compilerproc;
var
  len: SizeInt;
  temp: ansistring;
begin
  len := length(src);
  { make sure we don't dereference src if it can be nil (JM) }
  if len > 0 then
    widestringmanager.wide2ansimoveproc(pwidechar(@src[1]),temp,DefaultSystemCodePage,len);
  len := length(temp);
  if len > length(res) then
    len := length(res);
{$push}
{$r-}
  move(temp[1],res[0],len);
  fillchar(res[len],length(res)-len,0);
{$pop}
end;


procedure fpc_widestr_to_widechararray(out res: array of widechar; const src: WideString); compilerproc;
var
  len: SizeInt;
begin
  len := length(src);
  if len > length(res) then
    len := length(res);
{$push}
{$r-}
  { make sure we don't try to access element 1 of the ansistring if it's nil }
  if len > 0 then
    move(src[1],res[0],len*SizeOf(WideChar));
  fillchar(res[len],(length(res)-len)*SizeOf(WideChar),0);
{$pop}
end;



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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure fpc_WideStr_RangeCheck(p: Pointer; index: SizeInt);[Public,Alias : 'FPC_WIDESTR_RANGECHECK']; compilerproc;
begin
  if (p=nil) or (index>PWideRec(p-WideFirstOff)^.len div 2) or (Index<1) then
    HandleErrorAddrFrameInd(201,get_pc_addr,get_frame);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

Procedure fpc_WideStr_CheckRange(p: Pointer; index: SizeInt);[Public,Alias : 'FPC_WIDESTR_RANGECHECK']; compilerproc;
begin
  if (p=nil) or (index>PWideRec(p-WideFirstOff)^.len div 2) or (Index<1) then
    HandleErrorFrame(201,get_frame);
>>>>>>> graemeg/cpstrnew
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
              True
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
  else  // length=0, deallocate the string
    fpc_widestr_decr_ref (Pointer(S));
end;

{*****************************************************************************
                     Public functions, In interface.
*****************************************************************************}

Function fpc_widestr_Unique(Var S : Pointer): Pointer; [Public,Alias : 'FPC_WIDESTR_UNIQUE']; compilerproc;
  begin
    pointer(result) := pointer(s);
  end;


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
     { NewWideString won't return a nil pointer }
     ResultAddress:=NewWideString(Size);
     Move (PWideChar(S)[Index],ResultAddress^,Size*sizeof(WideChar));
     PWideChar(ResultAddress+Size*sizeof(WideChar))^:=#0;
   end;
  fpc_widestr_decr_ref(Pointer(fpc_widestr_copy));
  Pointer(fpc_widestr_Copy):=ResultAddress;
end;


Function Pos (Const Substr : WideString; Const Source : WideString; Offset : SizeInt = 1) : SizeInt;
var
  i,MaxLen : SizeInt;
  pc : pwidechar;
begin
  Pos:=0;
  if (Length(SubStr)>0) and (Offset>0) and (Offset<Length(Source)) then
   begin
     MaxLen:=Length(source)-Length(SubStr);
     i:=Offset-1;
     pc:=@source[Offset];
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
Function Pos (c : WideChar; Const s : WideString; Offset : Sizeint = 1) : SizeInt;
var
  i: SizeInt;
  pc : pwidechar;
begin
  pos:=0;
  if (Offset<1) or (Offset>Length(s)) then exit;
  pc:=@s[Offset];
  for i:=Offset to length(s) do
   begin
     if pc^=c then
      begin
        pos:=i;
        exit;
      end;
     inc(pc);
   end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ DO NOT inline these! Inlining a managed typecast creates an implicit try..finally
  block, which is significant bloat without any sensible speed improvement. }
Function Pos (c : WideChar; Const s : RawByteString; Offset : SizeInt = 1) : SizeInt;
=======

Function Pos (c : WideChar; Const s : RawByteString) : SizeInt;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======

Function Pos (c : WideChar; Const s : RawByteString) : SizeInt;
>>>>>>> graemeg/cpstrnew
=======

Function Pos (c : WideChar; Const s : RawByteString) : SizeInt;
>>>>>>> origin/cpstrnew
  begin
    result:=Pos(c,WideString(s),Offset);
  end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function Pos (const c : RawByteString; Const s : WideString;Offset : SizeInt = 1) : SizeInt;
=======
Function Pos (c : RawByteString; Const s : WideString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======
Function Pos (c : RawByteString; Const s : WideString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======
Function Pos (c : RawByteString; Const s : WideString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======
Function Pos (c : RawByteString; Const s : WideString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> origin/cpstrnew
  begin
    result:=Pos(WideString(c),s,Offset);
  end;


Function Pos (const c : ShortString; Const s : WideString;Offset : SizeInt = 1) : SizeInt;
  begin
    result:=Pos(WideString(c),s,Offset);
  end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function Pos (const c : WideString; Const s : RawByteString;Offset : SizeInt = 1) : SizeInt;
=======
Function Pos (c : WideString; Const s : RawByteString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======
Function Pos (c : WideString; Const s : RawByteString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======
Function Pos (c : WideString; Const s : RawByteString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======
Function Pos (c : WideString; Const s : RawByteString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> origin/cpstrnew
  begin
    result:=Pos(c,WideString(s),Offset);
  end;

{ Faster version for a char alone. Must be implemented because   }
{ pos(c: char; const s: shortstring) also exists, so otherwise   }
{ using pos(char,pchar) will always call the shortstring version }
{ (exact match for first argument), also with $h+ (JM)           }
Function Pos (c : Char; Const s : WideString;Offset : SizeInt = 1) : SizeInt;
var
  i: SizeInt;
  wc : widechar;
  pc : pwidechar;
begin
  Pos:=0;
  if (Offset<1) or (OffSet>Length(S)) then 
    exit;
  wc:=c;
  
  pc:=@s[offset];
  for i:=Offset to length(s) do
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
  LS:=Length(S);
  if (Index>LS) or (Index<=0) or (Size<=0) then
    exit;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  UniqueString (S);
>>>>>>> graemeg/cpstrnew
=======
  UniqueString (S);
>>>>>>> graemeg/cpstrnew
=======
  UniqueString (S);
>>>>>>> graemeg/cpstrnew
=======
  UniqueString (S);
>>>>>>> origin/cpstrnew
  { (Size+Index) will overflow if Size=MaxInt. }
  if Size>LS-Index then
    Size:=LS-Index+1;
  if Size<=LS-Index then
  begin
    Dec(Index);
    Move(PWideChar(S)[Index+Size],PWideChar(S)[Index],(LS-Index-Size+1)*sizeof(WideChar));
  end;
  Setlength(s,LS-Size);
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


<<<<<<< HEAD
Procedure {$ifdef FPC_HAS_CPSTRING}fpc_setstring_widestr_pwidechar{$else}SetString{$endif}(Out S : WideString; Buf : PWideChar; Len : SizeInt); {$ifdef FPC_HAS_CPSTRING} compilerproc; {$endif FPC_HAS_CPSTRING}
=======
Procedure SetString (Out S : WideString; Buf : PWideChar; Len : SizeInt);
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
begin
  SetLength(S,Len);
  If (Buf<>Nil) and (Len>0) then
    Move (Buf[0],S[1],Len*sizeof(WideChar));
end;


<<<<<<< HEAD
Procedure {$ifdef FPC_HAS_CPSTRING}fpc_setstring_widestr_pansichar{$else}SetString{$endif}(Out S : WideString; Buf : PChar; Len : SizeInt); {$ifdef FPC_HAS_CPSTRING} compilerproc; {$endif FPC_HAS_CPSTRING}
=======
Procedure SetString (Out S : WideString; Buf : PChar; Len : SizeInt);
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
begin
  If (Buf<>Nil) and (Len>0) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    widestringmanager.Ansi2WideMoveProc(Buf,DefaultSystemCodePage,S,Len)
  else
    SetLength(s,len);
=======
    widestringmanager.Ansi2WideMoveProc(Buf,DefaultSystemCodePage,S,Len);
>>>>>>> graemeg/cpstrnew
=======
    widestringmanager.Ansi2WideMoveProc(Buf,DefaultSystemCodePage,S,Len);
>>>>>>> graemeg/cpstrnew
=======
    widestringmanager.Ansi2WideMoveProc(Buf,DefaultSystemCodePage,S,Len);
>>>>>>> graemeg/cpstrnew
=======
    widestringmanager.Ansi2WideMoveProc(Buf,DefaultSystemCodePage,S,Len);
>>>>>>> origin/cpstrnew
end;


{$ifndef FPUNONE}
Function fpc_Val_Real_WideStr(Const S : WideString; out Code : ValSInt): ValReal; [public, alias:'FPC_VAL_REAL_WIDESTR']; compilerproc;
Var
  SS: ShortString;
begin
  fpc_Val_Real_WideStr := 0;
  if length(S)>255 then
    code:=256
  else
    begin
      SS:=ShortString(S);
      Val(SS,fpc_Val_Real_WideStr,code);
    end;
end;
{$endif}

function fpc_val_enum_widestr(str2ordindex:pointer;const s:widestring;out code:valsint):longint;compilerproc;
var
  ss: ShortString;
begin
  if length(s)>255 then
    code:=256
  else
    begin
      ss:=ShortString(s);
      val(ss,fpc_val_enum_widestr,code);
    end;
end;

Function fpc_Val_Currency_WideStr(Const S : WideString; out Code : ValSInt): Currency; [public, alias:'FPC_VAL_CURRENCY_WIDESTR']; compilerproc;
Var
  SS: ShortString;
begin
  if length(S)>255 then
    begin
      fpc_Val_Currency_WideStr:=0;
      code:=256;
    end
  else
    begin
      SS:=ShortString(S);
      Val(SS,fpc_Val_Currency_WideStr,code);
    end;
end;


Function fpc_Val_UInt_WideStr (Const S : WideString; out Code : ValSInt): ValUInt; [public, alias:'FPC_VAL_UINT_WIDESTR']; compilerproc;
Var
  SS: ShortString;
begin
  fpc_Val_UInt_WideStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
      SS:=ShortString(S);
      Val(SS,fpc_Val_UInt_WideStr,code);
    end;
end;


Function fpc_Val_SInt_WideStr (DestSize: SizeInt; Const S : WideString; out Code : ValSInt): ValSInt; [public, alias:'FPC_VAL_SINT_WIDESTR']; compilerproc;
Var
  SS: ShortString;
begin
  fpc_Val_SInt_WideStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
      SS:=ShortString(S);
      fpc_Val_SInt_WideStr := int_Val_SInt_ShortStr(DestSize,SS,Code);
    end;
end;


{$ifndef CPU64}

Function fpc_Val_qword_WideStr (Const S : WideString; out Code : ValSInt): qword; [public, alias:'FPC_VAL_QWORD_WIDESTR']; compilerproc;
Var
  SS: ShortString;
begin
  fpc_Val_qword_WideStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS:=ShortString(S);
       Val(SS,fpc_Val_qword_WideStr,Code);
    end;
end;


Function fpc_Val_int64_WideStr (Const S : WideString; out Code : ValSInt): Int64; [public, alias:'FPC_VAL_INT64_WIDESTR']; compilerproc;
Var
  SS: ShortString;
begin
  fpc_Val_int64_WideStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS:=ShortString(S);
       Val(SS,fpc_Val_int64_WideStr,Code);
    end;
end;

{$endif CPU64}


{$ifndef FPUNONE}
procedure fpc_WideStr_Float(d : ValReal;len,fr,rt : SizeInt;out s : WideString);compilerproc;
var
  ss: ShortString;
begin
  str_real(len,fr,d,treal_type(rt),ss);
  s:=WideString(ss);
end;
{$endif}

procedure fpc_widestr_enum(ordinal,len:sizeint;typinfo,ord2strindex:pointer;out s:widestring);compilerproc;
var
  ss: ShortString;
begin
  fpc_shortstr_enum(ordinal,len,typinfo,ord2strindex,ss);
  s:=WideString(ss);
end;

procedure fpc_widestr_bool(b : boolean;len:sizeint;out s:widestring);compilerproc;
var
  ss: ShortString;
begin
  fpc_shortstr_bool(b,len,ss);
  s:=WideString(ss);
end;

<<<<<<< HEAD
=======
procedure fpc_widestr_bool(b : boolean;len:sizeint;out s:widestring);compilerproc;

var ss:shortstring;

begin
  fpc_shortstr_bool(b,len,ss);
  s:=ss;
end;

procedure fpc_widestr_bool(b : boolean;len:sizeint;out s:widestring);compilerproc;

var ss:shortstring;

begin
  fpc_shortstr_bool(b,len,ss);
  s:=ss;
end;

procedure fpc_widestr_bool(b : boolean;len:sizeint;out s:widestring);compilerproc;

var ss:shortstring;

begin
  fpc_shortstr_bool(b,len,ss);
  s:=ss;
end;

procedure fpc_widestr_bool(b : boolean;len:sizeint;out s:widestring);compilerproc;

var ss:shortstring;

begin
  fpc_shortstr_bool(b,len,ss);
  s:=ss;
end;

{$ifdef FPC_HAS_STR_CURRENCY}
>>>>>>> graemeg/cpstrnew
procedure fpc_WideStr_Currency(c : Currency;len,fr : SizeInt;out s : WideString);compilerproc;
var
  ss: ShortString;
begin
  str(c:len:fr,ss);
  s:=WideString(ss);
end;

Procedure fpc_WideStr_SInt(v : ValSint; Len : SizeInt; out S : WideString);compilerproc;
Var
  SS: ShortString;
begin
  Str (v:Len,SS);
  S:=WideString(SS);
end;


Procedure fpc_WideStr_UInt(v : ValUInt;Len : SizeInt; out S : WideString);compilerproc;
Var
  SS: ShortString;
begin
  str(v:Len,SS);
  S:=WideString(SS);
end;


{$ifndef CPU64}

Procedure fpc_WideStr_Int64(v : Int64; Len : SizeInt; out S : WideString);compilerproc;
Var
  SS: ShortString;
begin
  Str(v:Len,SS);
  S:=WideString(SS);
end;


Procedure fpc_WideStr_Qword(v : Qword;Len : SizeInt; out S : WideString);compilerproc;
Var
  SS: ShortString;
begin
  str(v:Len,SS);
  S:=WideString(SS);
end;

{$endif CPU64}

<<<<<<< HEAD
function UTF8Encode(const s : WideString) : RawByteString;
=======
{ converts an utf-16 code point or surrogate pair to utf-32 }
function utf16toutf32(const S: WideString; const index: SizeInt; out len: longint): UCS4Char; [public, alias: 'FPC_WIDETOUTF32'];
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
    lw : longword;
    len : longint;
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
              $800..$d7ff,$e000..$ffff:
                begin
                  if j+2>=MaxDestBytes then
                    break;
                  Dest[j]:=char($e0 or (w shr 12));
                  Dest[j+1]:=char($80 or ((w shr 6) and $3f));
                  Dest[j+2]:=char($80 or (w and $3f));
                  inc(j,3);
                end;
              $d800..$dbff:
                {High Surrogates}
                begin
                  if j+3>=MaxDestBytes then
                    break;
                  if (i<sourcechars-1) and
                     (word(Source[i+1]) >= $dc00) and
                     (word(Source[i+1]) <= $dfff) then
                    begin
                      lw:=longword(utf16toutf32(Source[i] + Source[i+1], 1, len));
                      Dest[j]:=char($f0 or (lw shr 18));
                      Dest[j+1]:=char($80 or ((lw shr 12) and $3f));
                      Dest[j+2]:=char($80 or ((lw shr 6) and $3f));
                      Dest[j+3]:=char($80 or (lw and $3f));
                      inc(j,4);
                      inc(i);
                    end;
                end;
              end;
            inc(i);
          end;

        if j>SizeUInt(MaxDestBytes-1) then
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
              $800..$d7ff,$e000..$ffff:
                inc(j,3);
              $d800..$dbff:
                begin
                  if (i<sourcechars-1) and
                     (word(Source[i+1]) >= $dc00) and
                     (word(Source[i+1]) <= $dfff) then
                    begin
                      inc(j,4);
                      inc(i);
                    end;
                end;
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


function UTF8ToUnicode(Dest: PWideChar; MaxDestChars: SizeUInt; Source: PChar; SourceBytes: SizeUInt): SizeUInt;
  const
    UNICODE_INVALID=63;
  var
    InputUTF8: SizeUInt;
    IBYTE: BYTE;
    OutputUnicode: SizeUInt;
    PRECHAR: SizeUInt;
    TempBYTE: BYTE;
    CharLen: SizeUint;
    LookAhead: SizeUInt;
    UC: SizeUInt;
  begin
    if not assigned(Source) then
      begin
        result:=0;
        exit;
      end;
    result:=SizeUInt(-1);
    InputUTF8:=0;
    OutputUnicode:=0;
    PreChar:=0;
    if Assigned(Dest) Then
      begin
        while (OutputUnicode<MaxDestChars) and (InputUTF8<SourceBytes) do
          begin
            IBYTE:=byte(Source[InputUTF8]);
            if (IBYTE and $80) = 0 then
              begin
                //One character US-ASCII, convert it to unicode
                if IBYTE = 10 then
                  begin
                    If (PreChar<>13) and FALSE then
                      begin
                        //Expand to crlf, conform UTF-8.
                        //This procedure will break the memory alocation by
                        //FPC for the widestring, so never use it. Condition never true due the "and FALSE".
                        if OutputUnicode+1<MaxDestChars then
                          begin
                            Dest[OutputUnicode]:=WideChar(13);
                            inc(OutputUnicode);
                            Dest[OutputUnicode]:=WideChar(10);
                            inc(OutputUnicode);
                            PreChar:=10;
                          end
                        else
                          begin
                            Dest[OutputUnicode]:=WideChar(13);
                            inc(OutputUnicode);
                          end;
                      end
                    else
                      begin
                        Dest[OutputUnicode]:=WideChar(IBYTE);
                        inc(OutputUnicode);
                        PreChar:=IBYTE;
                      end;
                  end
                else
                  begin
                    Dest[OutputUnicode]:=WideChar(IBYTE);
                    inc(OutputUnicode);
                    PreChar:=IBYTE;
                  end;
                inc(InputUTF8);
              end
            else
              begin
                TempByte:=IBYTE;
                CharLen:=0;
                while (TempBYTE and $80)<>0 do
                  begin
                    TempBYTE:=(TempBYTE shl 1) and $FE;
                    inc(CharLen);
                  end;
                //Test for the "CharLen" conforms UTF-8 string
                //This means the 10xxxxxx pattern.
                if SizeUInt(InputUTF8+CharLen-1)>SourceBytes then
                  begin
                    //Insuficient chars in string to decode
                    //UTF-8 array. Fallback to single char.
                    CharLen:= 1;
                  end;
                for LookAhead := 1 to CharLen-1 do
                  begin
                    if ((byte(Source[InputUTF8+LookAhead]) and $80)<>$80) or
                       ((byte(Source[InputUTF8+LookAhead]) and $40)<>$00) then
                      begin
                        //Invalid UTF-8 sequence, fallback.
                        CharLen:= LookAhead;
                        break;
                      end;
                  end;
                UC:=$FFFF;
                case CharLen of
                  1:  begin
                        //Not valid UTF-8 sequence
                        UC:=UNICODE_INVALID;
                      end;
                  2:  begin
                        //Two bytes UTF, convert it
                        UC:=(byte(Source[InputUTF8]) and $1F) shl 6;
                        UC:=UC or (byte(Source[InputUTF8+1]) and $3F);
                        if UC <= $7F then
                          begin
                            //Invalid UTF sequence.
                            UC:=UNICODE_INVALID;
                          end;
                      end;
                  3:  begin
                        //Three bytes, convert it to unicode
                        UC:= (byte(Source[InputUTF8]) and $0F) shl 12;
                        UC:= UC or ((byte(Source[InputUTF8+1]) and $3F) shl 6);
                        UC:= UC or ((byte(Source[InputUTF8+2]) and $3F));
                        if (UC <= $7FF) or (UC >= $FFFE) or ((UC >= $D800) and (UC <= $DFFF)) then
                          begin
                            //Invalid UTF-8 sequence
                            UC:= UNICODE_INVALID;
                          End;
                      end;
                  4:  begin
                        //Four bytes, convert it to two unicode characters
                        UC:= (byte(Source[InputUTF8]) and $07) shl 18;
                        UC:= UC or ((byte(Source[InputUTF8+1]) and $3F) shl 12);
                        UC:= UC or ((byte(Source[InputUTF8+2]) and $3F) shl 6);
                        UC:= UC or ((byte(Source[InputUTF8+3]) and $3F));
                        if (UC < $10000) or (UC > $10FFFF) then
                          begin
                            UC:= UNICODE_INVALID;
                          end
                        else
                          begin
                            { only store pair if room }
                            dec(UC,$10000);
                            if (OutputUnicode<MaxDestChars-1) then
                              begin
                                Dest[OutputUnicode]:=WideChar(UC shr 10 + $D800);
                                inc(OutputUnicode);
                                UC:=(UC and $3ff) + $DC00;
                              end
                            else
                              begin
                                InputUTF8:= InputUTF8 + CharLen;
                                { don't store anything }
                                CharLen:=0;
                              end;
                          end;
                      end;
                  5,6,7:  begin
                            //Invalid UTF8 to unicode conversion,
                            //mask it as invalid UNICODE too.
                            UC:=UNICODE_INVALID;
                          end;
                end;
                if CharLen > 0 then
                  begin
                    PreChar:=UC;
                    Dest[OutputUnicode]:=WideChar(UC);
                    inc(OutputUnicode);
                  end;
                InputUTF8:= InputUTF8 + CharLen;
              end;
          end;
        Result:=OutputUnicode+1;
      end
    else
      begin
        while (InputUTF8<SourceBytes) do
          begin
            IBYTE:=byte(Source[InputUTF8]);
            if (IBYTE and $80) = 0 then
              begin
                //One character US-ASCII, convert it to unicode
                if IBYTE = 10 then
                  begin
                    if (PreChar<>13) and FALSE then
                      begin
                        //Expand to crlf, conform UTF-8.
                        //This procedure will break the memory alocation by
                        //FPC for the widestring, so never use it. Condition never true due the "and FALSE".
                        inc(OutputUnicode,2);
                        PreChar:=10;
                      end
                    else
                      begin
                        inc(OutputUnicode);
                        PreChar:=IBYTE;
                      end;
                  end
                else
                  begin
                    inc(OutputUnicode);
                    PreChar:=IBYTE;
                  end;
                inc(InputUTF8);
              end
            else
              begin
                TempByte:=IBYTE;
                CharLen:=0;
                while (TempBYTE and $80)<>0 do
                  begin
                    TempBYTE:=(TempBYTE shl 1) and $FE;
                    inc(CharLen);
                  end;
                //Test for the "CharLen" conforms UTF-8 string
                //This means the 10xxxxxx pattern.
                if SizeUInt(InputUTF8+CharLen-1)>SourceBytes then
                  begin
                    //Insuficient chars in string to decode
                    //UTF-8 array. Fallback to single char.
                    CharLen:= 1;
                  end;
                for LookAhead := 1 to CharLen-1 do
                  begin
                    if ((byte(Source[InputUTF8+LookAhead]) and $80)<>$80) or
                       ((byte(Source[InputUTF8+LookAhead]) and $40)<>$00) then
                      begin
                        //Invalid UTF-8 sequence, fallback.
                        CharLen:= LookAhead;
                        break;
                      end;
                  end;
                UC:=$FFFF;
                case CharLen of
                  1:  begin
                        //Not valid UTF-8 sequence
                        UC:=UNICODE_INVALID;
                      end;
                  2:  begin
                        //Two bytes UTF, convert it
                        UC:=(byte(Source[InputUTF8]) and $1F) shl 6;
                        UC:=UC or (byte(Source[InputUTF8+1]) and $3F);
                        if UC <= $7F then
                          begin
                            //Invalid UTF sequence.
                            UC:=UNICODE_INVALID;
                          end;
                      end;
                  3:  begin
                        //Three bytes, convert it to unicode
                        UC:= (byte(Source[InputUTF8]) and $0F) shl 12;
                        UC:= UC or ((byte(Source[InputUTF8+1]) and $3F) shl 6);
                        UC:= UC or ((byte(Source[InputUTF8+2]) and $3F));
                        If (UC <= $7FF) or (UC >= $FFFE) or ((UC >= $D800) and (UC <= $DFFF)) then
                          begin
                            //Invalid UTF-8 sequence
                            UC:= UNICODE_INVALID;
                          end;
                      end;
                  4:  begin
                        //Four bytes, convert it to two unicode characters
                        UC:= (byte(Source[InputUTF8]) and $07) shl 18;
                        UC:= UC or ((byte(Source[InputUTF8+1]) and $3F) shl 12);
                        UC:= UC or ((byte(Source[InputUTF8+2]) and $3F) shl 6);
                        UC:= UC or ((byte(Source[InputUTF8+3]) and $3F));
                        if (UC < $10000) or (UC > $10FFFF) then
                          UC:= UNICODE_INVALID
                        else
                          { extra character character }
                          inc(OutputUnicode);
                      end;
                  5,6,7:  begin
                            //Invalid UTF8 to unicode conversion,
                            //mask it as invalid UNICODE too.
                            UC:=UNICODE_INVALID;
                          end;
                end;
                if CharLen > 0 then
                  begin
                    PreChar:=UC;
                    inc(OutputUnicode);
                  end;
                InputUTF8:= InputUTF8 + CharLen;
              end;
          end;
        Result:=OutputUnicode+1;
      end;
  end;


function UTF8Encode(const s : WideString) : UTF8String;
>>>>>>> graemeg/cpstrnew
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


<<<<<<< HEAD
=======
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

{$warnings on}

function DefaultCharLengthPChar(const Str: PChar): PtrInt;forward;
function DefaultCodePointLength(const Str: PChar; MaxLookAead: PtrInt): Ptrint;forward;


procedure initwidestringmanager;
  begin
    fillchar(widestringmanager,sizeof(widestringmanager),0);
{$ifndef HAS_WIDESTRINGMANAGER}
    widestringmanager.Wide2AnsiMoveProc:=@DefaultWide2AnsiMove;
    widestringmanager.Ansi2WideMoveProc:=@DefaultAnsi2WideMove;
    widestringmanager.UpperWideStringProc:=@GenericWideCase;
    widestringmanager.LowerWideStringProc:=@GenericWideCase;
{$endif HAS_WIDESTRINGMANAGER}
    widestringmanager.CompareWideStringProc:=@CompareWideString;
    widestringmanager.CompareTextWideStringProc:=@CompareTextWideString;
    widestringmanager.CharLengthPCharProc:=@DefaultCharLengthPChar;
    widestringmanager.CodePointLengthProc:=@DefaultCodePointLength;
  end;
>>>>>>> graemeg/cpstrnew
=======
{$i wustrings.inc}
>>>>>>> graemeg/fixes_2_2
=======
{$i wustrings.inc}
>>>>>>> origin/fixes_2_2
