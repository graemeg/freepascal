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

{
  This file contains the implementation of the WideString type,
  and all things that are needed for it.
  WideString is defined as a 'silent' pwidechar :
  a pwidechar that points to:

  @-4  : DWord for size; size=number of bytes, not the number of chars. Divide or multiply
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
    widestringmanager.Ansi2WideMoveProc(PChar(@S2[1]),DefaultSystemCodePage,result,Size);
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
  cp:=DefaultSystemCodePage;
{$endif FPC_HAS_CPSTRING}
  result:='';
  Size:=Length(S2);
  if Size>0 then
  begin
    cp:=TranslatePlaceholderCP(cp);
    widestringmanager.Wide2AnsiMoveProc(PWideChar(Pointer(S2)),result,cp,Size);
  end;
end;


Function fpc_AnsiStr_To_WideStr(Const S2 : RawByteString): WideString; compilerproc;
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
    cp:=TranslatePlaceholderCP(StringCodePage(S2));
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
  widestringmanager.Ansi2WideMoveProc(@c,DefaultSystemCodePage,fpc_char_To_WideStr,1);
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
  widestringmanager.Ansi2WideMoveProc(pchar(@arr),DefaultSystemCodePage,fpc_CharArray_To_WideStr,i);
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

Procedure fpc_WideStr_RangeCheck(p: Pointer; index: SizeInt);[Public,Alias : 'FPC_WIDESTR_RANGECHECK']; compilerproc;
begin
  if (p=nil) or (index>PWideRec(p-WideFirstOff)^.len div 2) or (Index<1) then
    HandleErrorAddrFrameInd(201,get_pc_addr,get_frame);
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

Function fpc_widestr_Unique_func(Var S : WideString): Pointer; external name 'FPC_WIDESTR_UNIQUE';


Procedure UniqueString (Var S : WideString);{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    fpc_widestr_Unique_func(S);
  end;


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
  if (Length(SubStr)>0) and (Offset>0) and (Offset<=Length(Source)) then
   begin
     MaxLen:=Length(source)-Length(SubStr)-(Offset-1);
     i:=0;
     pc:=@source[Offset];
     while (i<=MaxLen) do
      begin
        inc(i);
        if (SubStr[1]=pc^) and
           (CompareWord(Substr[1],pc^,Length(SubStr))=0) then
         begin
           Pos:=Offset+i-1;
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
  if (Offset>0) and (Offset<=length(s)) then
   begin
     pc:=@s[OffSet];
     for i:=OffSet to length(s) do
       begin
         if pc^=c then
           begin
             pos:=i;
             exit;
           end;
         inc(pc);
       end;
   end;
  pos:=0;
end;

{ DO NOT inline these! Inlining a managed typecast creates an implicit try..finally
  block, which is significant bloat without any sensible speed improvement. }
Function Pos (c : WideChar; Const s : RawByteString; Offset : SizeInt = 1) : SizeInt;
  begin
    result:=Pos(c,WideString(s),Offset);
  end;


Function Pos (const c : RawByteString; Const s : WideString;Offset : SizeInt = 1) : SizeInt;
  begin
    result:=Pos(WideString(c),s,Offset);
  end;


Function Pos (const c : ShortString; Const s : WideString;Offset : SizeInt = 1) : SizeInt;
  begin
    result:=Pos(WideString(c),s,Offset);
  end;


Function Pos (const c : WideString; Const s : RawByteString;Offset : SizeInt = 1) : SizeInt;
  begin
    result:=Pos(c,WideString(s),Offset);
  end;

{ Faster version for a char alone. Must be implemented because   }
{ pos(c: char; const s: shortstring) also exists, so otherwise   }
{ using pos(char,pchar) will always call the shortstring version }
{ (exact match for first argument), also with $h+ (JM)           }
Function Pos (c : Char; Const s : WideString; Offset : SizeInt = 1) : SizeInt;
var
  i: SizeInt;
  wc : widechar;
  pc : pwidechar;
begin
  if (Offset>0) and (Offset<=Length(S)) then
    begin
      wc:=c;
      pc:=@s[OffSet];
      for i:=OffSet to length(s) do
       begin
         if pc^=wc then
          begin
            pos:=i;
            exit;
          end;
         inc(pc);
       end;
    end;
  pos:=0;
end;



Procedure {$ifdef VER3_0}Delete{$else}fpc_widestr_delete{$endif}(Var S : WideString; Index,Size: SizeInt);
Var
  LS : SizeInt;
begin
  LS:=Length(S);
  if (Index>LS) or (Index<=0) or (Size<=0) then
    exit;

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


Procedure {$ifdef VER3_0}Insert{$else}fpc_widestr_insert{$endif}(Const Source : WideString; Var S : WideString; Index : SizeInt);
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


Procedure {$ifdef FPC_HAS_CPSTRING}fpc_setstring_widestr_pwidechar{$else}SetString{$endif}(Out S : WideString; Buf : PWideChar; Len : SizeInt); {$ifdef FPC_HAS_CPSTRING} compilerproc; {$endif FPC_HAS_CPSTRING}
begin
  SetLength(S,Len);
  If (Buf<>Nil) and (Len>0) then
    Move (Buf[0],S[1],Len*sizeof(WideChar));
end;


Procedure {$ifdef FPC_HAS_CPSTRING}fpc_setstring_widestr_pansichar{$else}SetString{$endif}(Out S : WideString; Buf : PChar; Len : SizeInt); {$ifdef FPC_HAS_CPSTRING} compilerproc; {$endif FPC_HAS_CPSTRING}
begin
  If (Buf<>Nil) and (Len>0) then
    widestringmanager.Ansi2WideMoveProc(Buf,DefaultSystemCodePage,S,Len)
  else
    SetLength(s,len);
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

function UTF8Encode(const s : WideString) : RawByteString;
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


