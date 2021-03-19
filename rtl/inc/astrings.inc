{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt,
    member of the Free Pascal development team.

    This file implements AnsiStrings for FPC

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ This will release some functions for special shortstring support }
{ define EXTRAANSISHORT}


{$ifndef FPC_ANSISTRING_TYPE_DEFINED}
{$define FPC_ANSISTRING_TYPE_DEFINED}
{
  This file contains the implementation of the AnsiString type,
  and all things that are needed for it.
  AnsiString is defined as a 'silent' pchar :
  a pchar that points to (S= SizeOf(SizeInt)):

  @-3*S   : Code page indicator.
  @-3*S+2 : Character size (2 bytes)
  @-2*S   : SizeInt for reference count;
  @-S  : SizeInt for size;
  @    : String + Terminating #0;
  Pchar(Ansistring) is a valid typecast.
  So AS[i] is converted to the address @AS+i-1.

  Constants should be assigned a reference count of -1
  Meaning that they can't be disposed of.
}

Type
  PAnsiRec = ^TAnsiRec;
  TAnsiRec = Record
    CodePage    : TSystemCodePage;
    ElementSize : Word;
{$ifdef CPU64}	
    { align fields  }
	Dummy       : DWord;
{$endif CPU64}
    Ref         : SizeInt;
    Len         : SizeInt;
  end;

Const
  AnsiFirstOff = SizeOf(TAnsiRec);
{$endif FPC_ANSISTRING_TYPE_DEFINED}

{****************************************************************************
                    Internal functions, not in interface.
****************************************************************************}

{$ifndef FPC_HAS_TRANSLATEPLACEHOLDERCP}
{$define FPC_HAS_TRANSLATEPLACEHOLDERCP}
function TranslatePlaceholderCP(cp: TSystemCodePage): TSystemCodePage; {$ifdef SYSTEMINLINE}inline;{$endif}
begin
  TranslatePlaceholderCP:=cp;
  case cp of
    CP_OEMCP,
    CP_ACP:
      TranslatePlaceholderCP:=DefaultSystemCodePage;
  end;
end;
{$endif FPC_HAS_TRANSLATEPLACEHOLDERCP}


{$ifndef FPC_HAS_PCHAR_ANSISTR_INTERN_CHARMOVE}
{$define FPC_HAS_PCHAR_ANSISTR_INTERN_CHARMOVE}
procedure fpc_pchar_ansistr_intern_charmove(const src: pchar; const srcindex: sizeint; var dst: rawbytestring; const dstindex, len: sizeint); {$ifdef FPC_HAS_CPSTRING}rtlproc;{$endif} {$ifdef SYSTEMINLINE}inline;{$endif}
begin
  move(src[srcindex],pbyte(pointer(dst))[dstindex],len);
end;
{$endif FPC_HAS_PCHAR_ANSISTR_INTERN_CHARMOVE}


{$ifndef FPC_HAS_PCHAR_PCHAR_INTERN_CHARMOVE}
{$define FPC_HAS_PCHAR_PCHAR_INTERN_CHARMOVE}
procedure fpc_pchar_pchar_intern_charmove(const src: pchar; const srcindex: sizeint; const dst: pchar; const dstindex, len: sizeint); {$ifdef FPC_HAS_CPSTRING}rtlproc;{$endif} {$ifdef SYSTEMINLINE}inline;{$endif}
begin
  move(src[srcindex],dst[dstindex],len);
end;
{$endif FPC_HAS_PCHAR_PCHAR_INTERN_CHARMOVE}


{$ifndef FPC_HAS_SHORTSTR_ANSISTR_INTERN_CHARMOVE}
{$define FPC_HAS_SHORTSTR_ANSISTR_INTERN_CHARMOVE}
procedure fpc_shortstr_ansistr_intern_charmove(const src: shortstring; const srcindex: sizeint; var dst: rawbytestring; const dstindex, len: sizeint); {$ifdef FPC_HAS_CPSTRING}rtlproc;{$endif} {$ifdef SYSTEMINLINE}inline;{$endif}
begin
  move(src[srcindex],pbyte(pointer(dst))[dstindex],len);
end;
{$endif FPC_HAS_SHORTSTR_ANSISTR_INTERN_CHARMOVE}



{$ifndef FPC_HAS_NEWANSISTR}
{$define FPC_HAS_NEWANSISTR}
Function NewAnsiString(Len : SizeInt) : Pointer;
{
  Allocate a new AnsiString on the heap.
  initialize it to zero length and reference count 1.
}
Var
  P : Pointer;
begin
  { request a multiple of 16 because the heap manager alloctes anyways chunks of 16 bytes }
  GetMem(P,Len+(AnsiFirstOff+sizeof(char)));
  If P<>Nil then
   begin
     PAnsiRec(P)^.Ref:=1;         { Set reference count }
     PAnsiRec(P)^.Len:=0;         { Initial length }
     PAnsiRec(P)^.CodePage:=DefaultSystemCodePage;
     PAnsiRec(P)^.ElementSize:=SizeOf(AnsiChar);
     inc(p,AnsiFirstOff);         { Points to string now }
     PAnsiChar(P)^:=#0;           { Terminating #0 }
   end;
  NewAnsiString:=P;
end;
{$endif FPC_HAS_NEWANSISTR}


{$ifndef FPC_SYSTEM_HAS_ANSISTR_DECR_REF}
{$define FPC_SYSTEM_HAS_ANSISTR_DECR_REF}
Procedure fpc_ansistr_decr_ref (Var S : Pointer); [Public,Alias:'FPC_ANSISTR_DECR_REF'];  compilerproc;
{
  Decreases the ReferenceCount of a non constant ansistring;
  If the reference count is zero, deallocate the string;
}
Var
  p: PAnsiRec;
Begin
  { Zero string }
  If S=Nil then 
    exit;
  { check for constant strings ...}
  p:=PAnsiRec(S-AnsiFirstOff);
  s:=nil;
  If p^.ref<0 then exit;
  { declocked does a MT safe dec and returns true, if the counter is 0 }
  If declocked(p^.ref) then
    FreeMem(p);
end;
{$endif FPC_SYSTEM_HAS_ANSISTR_DECR_REF}

{ also define alias for internal use in the system unit }
Procedure fpc_ansistr_decr_ref (Var S : Pointer); [external name 'FPC_ANSISTR_DECR_REF'];


{$ifndef FPC_SYSTEM_HAS_ANSISTR_INCR_REF}
{$define FPC_SYSTEM_HAS_ANSISTR_INCR_REF}
Procedure fpc_AnsiStr_Incr_Ref (S : Pointer); [Public,Alias:'FPC_ANSISTR_INCR_REF'];  compilerproc; inline;
Begin
  If S=Nil then
    exit;
  { Let's be paranoid : Constant string ??}
  If PAnsiRec(S-AnsiFirstOff)^.Ref<0 then exit;
  inclocked(PAnsiRec(S-AnsiFirstOff)^.Ref);
end;
{$endif FPC_SYSTEM_HAS_ANSISTR_DECR_REF}

{ also define alias which can be used inside the system unit }
Procedure fpc_AnsiStr_Incr_Ref (S : Pointer); [external name 'FPC_ANSISTR_INCR_REF'];


{$ifndef FPC_HAS_ANSISTR_ASSIGN}
{$define FPC_HAS_ANSISTR_ASSIGN}
Procedure fpc_AnsiStr_Assign (Var DestS : Pointer;S2 : Pointer);[Public,Alias:'FPC_ANSISTR_ASSIGN'];  compilerproc;
{
  Assigns S2 to S1 (S1:=S2), taking in account reference counts.
}
begin
  if DestS=S2 then
    exit;
  If S2<>nil then
    If PAnsiRec(S2-AnsiFirstOff)^.Ref>0 then
      inclocked(PAnsiRec(S2-AnsiFirstOff)^.Ref);
  { Decrease the reference count on the old S1 }
  fpc_ansistr_decr_ref (DestS);
  { And finally, have DestS pointing to S2 (or its copy) }
  DestS:=S2;
end;
{$endif FPC_HAS_ANSISTR_ASSIGN}


{ alias for internal use }
Procedure fpc_AnsiStr_Assign (Var S1 : Pointer;S2 : Pointer);[external name 'FPC_ANSISTR_ASSIGN'];

{$ifndef FPC_HAS_ANSISTR_CONCAT_COMPLEX}
{$define FPC_HAS_ANSISTR_CONCAT_COMPLEX}
{ keeps implicit try..finally block out from primary control flow }
procedure ansistr_concat_complex(var DestS: RawByteString; const S1,S2: RawByteString; cp: TSystemCodePage);
var
  U: UnicodeString;
begin
  U:=UnicodeString(S1)+UnicodeString(S2);
  widestringmanager.Unicode2AnsiMoveProc(PUnicodeChar(Pointer(U)),DestS,cp,Length(U));
end;
{$endif FPC_HAS_ANSISTR_CONCAT_COMPLEX}

{$ifndef FPC_HAS_ANSISTR_CONCAT}
{$define FPC_HAS_ANSISTR_CONCAT}
procedure fpc_AnsiStr_Concat (var DestS:RawByteString;const S1,S2 : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
Var
  S1Len, S2Len: SizeInt;
  same : boolean;
  S1CP, S2CP, DestCP: TSystemCodePage;
begin
{$ifdef FPC_HAS_CPSTRING}
  DestCP:=cp;
  if DestCp=CP_NONE then
    DestCP:=DefaultSystemCodePage;
{$else FPC_HAS_CPSTRING}
  DestCP:=StringCodePage(DestS);
{$endif FPC_HAS_CPSTRING}
  DestCP:=TranslatePlaceholderCP(DestCP);
  { if codepages are different then concat using unicodestring,
    but avoid conversions if either addend is empty (StringCodePage will return
    DefaultSystemCodePage in that case, which may differ from other addend/dest) }
  if S1='' then
    S1CP:=DestCP
  else
    S1CP:=StringCodePage(S1);
  S1CP:=TranslatePlaceholderCP(S1CP);
  if S2='' then
    S2CP:=DestCP
  else
    S2CP:=StringCodePage(S2);
  S2CP:=TranslatePlaceholderCP(S2CP);
{$ifdef FPC_HAS_CPSTRING}
  { if the result is rawbytestring and both strings have the same code page,
    keep that code page or keep the code page if the other string is empty }
  if cp=CP_NONE then
    begin
     if (S1CP=S2CP) or (S2='') then
       DestCP:=S1CP
     else if S1='' then
       DestCP:=S2CP;
    end;
{$endif FPC_HAS_CPSTRING}
  if ((S1CP<>DestCP) and (s1<>'')) or ((S2CP<>DestCP) and (s2<>'')) then
    begin
      ansistr_concat_complex(DestS,S1,S2,DestCP);
      exit;
    end;
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
  S1Len:=Length(S1);
  S2Len:=length(S2);
  { Use Pointer() typecasts to prevent extra conversion code }
  if Pointer(DestS)=Pointer(S1) then
    begin
      same:=Pointer(S1)=Pointer(S2);
      SetLength(DestS,S1Len+S2Len);
      if same then
        fpc_pchar_ansistr_intern_charmove(PAnsiChar(DestS),0,DestS,S1Len,S2Len)
      else
        fpc_pchar_ansistr_intern_charmove(PAnsiChar(S2),0,DestS,S1Len,S2Len+1)
    end
  else if Pointer(DestS)=Pointer(S2) then
    begin
      SetLength(DestS,S1Len+S2Len);
      fpc_pchar_ansistr_intern_charmove(PAnsiChar(DestS),0,DestS,S1Len,S2Len+1);
      fpc_pchar_ansistr_intern_charmove(PAnsiChar(S1),0,DestS,0,S1Len);
    end
  else
    begin
      SetLength(DestS,S1Len+S2Len);
      fpc_pchar_ansistr_intern_charmove(PAnsiChar(S1),0,DestS,0,S1Len);
      fpc_pchar_ansistr_intern_charmove(PAnsiChar(S2),0,DestS,S1Len,S2Len+1);
    end;
  SetCodePage(DestS,DestCP,false);
end;
{$endif FPC_HAS_ANSISTR_CONCAT}


{$ifndef FPC_HAS_ANSISTR_CONCAT_MULTI}
{$define FPC_HAS_ANSISTR_CONCAT_MULTI}
procedure fpc_AnsiStr_Concat_multi (var DestS:RawByteString;const sarr:array of RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
Var
  lowstart,
  nonemptystart,
  i           : ObjpasInt;
  p,pc        : pointer;
  Size,NewLen,
  OldDestLen  : SizeInt;
  destcopy    : pointer;
  U           : UnicodeString;
  DestCP,
  tmpCP       : TSystemCodePage;
  sameCP      : Boolean;
begin
  if high(sarr)=0 then
    begin
      DestS:='';
      exit;
    end;
{$ifdef FPC_HAS_CPSTRING}
  DestCP:=cp;
  if DestCp=CP_NONE then
    DestCP:=DefaultSystemCodePage;
{$else FPC_HAS_CPSTRING}
  DestCP:=StringCodePage(DestS);
{$endif FPC_HAS_CPSTRING}
  lowstart:=low(sarr);
  { skip empty strings }
  while (lowstart<=high(sarr)) and
        (sarr[lowstart]='') do
    inc(lowstart);
  if lowstart>high(sarr) then
    begin
      DestS:=''; { All source strings empty }
      exit;
    end;
  DestCP:=TranslatePlaceholderCP(DestCP);
  sameCP:=true;
  tmpCP:=TranslatePlaceholderCP(StringCodePage(sarr[lowstart]));
  for i:=lowstart+1 to high(sarr) do
    begin
      { ignore the code page of empty strings, it will always be
        DefaultSystemCodePage but it doesn't matter for the outcome }
      if (sarr[i]<>'') and
         (tmpCP<>TranslatePlaceholderCP(StringCodePage(sarr[i]))) then
        begin
          sameCP:=false;
          break;
        end;
    end;
  if not sameCP then
    begin
      U:='';
      for i:=lowstart to high(sarr) do
        if sarr[i]<>'' then
          U:=U+UnicodeString(sarr[i]);

      DestS:='';
      widestringmanager.Unicode2AnsiMoveProc(PUnicodeChar(Pointer(U)),DestS,DestCP,Length(U));
      exit;
    end;
{$ifdef FPC_HAS_CPSTRING}
  { if the result is rawbytestring and all strings have the same code page,
    keep that code page }
  if cp=CP_NONE then
    DestCP:=tmpCP;
{$endif FPC_HAS_CPSTRING}
  destcopy:=nil;
  nonemptystart:=lowstart;
  { Check for another reuse, then we can't use
    the append optimization }
  if DestS<>'' then
    begin
      if Pointer(DestS)=Pointer(sarr[lowstart]) then
        inc(lowstart);
      for i:=lowstart to high(sarr) do
        begin
          if Pointer(DestS)=Pointer(sarr[i]) then
            begin
              { if DestS is used somewhere in the middle of the expression,
                we need to make sure the original string still exists after
                we empty/modify DestS                                       }
              destcopy:=pointer(dests);
              fpc_AnsiStr_Incr_Ref(destcopy);
              lowstart:=nonemptystart;
              break;
            end;
        end;
    end;
  { Start with empty DestS if we start with concatting
    the first (non-empty) array element }
  if lowstart=nonemptystart then
    DestS:='';
  OldDestLen:=length(DestS);
  { Calculate size of the result so we can do
    a single call to SetLength() }
  NewLen:=0;
  for i:=nonemptystart to high(sarr) do
    inc(NewLen,length(sarr[i]));
  SetLength(DestS,NewLen);
  { Concat all strings, except the string we already
    copied in DestS }
  pc:=Pointer(DestS)+OldDestLen;
  for i:=lowstart to high(sarr) do
    begin
      p:=pointer(sarr[i]);
      if assigned(p) then
        begin
          Size:=length(ansistring(p));
          Move(p^,pc^,Size+1);
          inc(pc,size);
        end;
    end;
  SetCodePage(DestS,tmpCP,False);
  SetCodePage(DestS,DestCP,True);
  fpc_AnsiStr_Decr_Ref(destcopy);
end;
{$endif FPC_HAS_ANSISTR_CONCAT_MULTI}

{$ifdef EXTRAANSISHORT}
Procedure AnsiStr_ShortStr_Concat (Var S1: AnsiString; Var S2 : ShortString);
{
  Concatenates a Ansi with a short string; : S2 + S2
}
Var
  Size,Location : SizeInt;
begin
  Size:=Length(S2);
  Location:=Length(S1);
  If Size=0 then
    exit;
  { Setlength takes case of uniqueness
    and alllocated memory. We need to use length,
    to take into account possibility of S1=Nil }
  SetLength (S1,Size+Length(S1));
  Move (S2[1],Pointer(Pointer(S1)+Location)^,Size);
  PByte( Pointer(S1)+length(S1) )^:=0; { Terminating Zero }
end;
{$endif EXTRAANSISHORT}

{$ifdef FPC_HAS_CPSTRING}
{$ifndef FPC_HAS_ANSISTR_TO_ANSISTR}
{$define FPC_HAS_ANSISTR_TO_ANSISTR}
Function fpc_AnsiStr_To_AnsiStr (const S : RawByteString;cp : TSystemCodePage): RawByteString; [Public, alias: 'FPC_ANSISTR_TO_ANSISTR']; compilerproc;
{
  Converts an AnsiString to an AnsiString taking code pages into care
}
Var
  Size : SizeInt;
  temp : UnicodeString;
  orgcp: TSystemCodePage;
begin
  result:='';
  Size:=Length(S);
  if Size>0 then
    begin
      cp:=TranslatePlaceholderCP(cp);
      orgcp:=TranslatePlaceholderCP(StringCodePage(S));
      if (orgcp=cp) or (orgcp=CP_NONE) then
        begin
          SetLength(result,Size);
          Move(S[1],result[1],Size);
          PAnsiRec(Pointer(result)-AnsiFirstOff)^.CodePage:=cp;
        end
      else
        begin
          temp:=UnicodeString(S);
          Size:=Length(temp);
          widestringmanager.Unicode2AnsiMoveProc(PUnicodeChar(Pointer(temp)),result,cp,Size);
        end;
    end;
end;

Function fpc_AnsiStr_To_AnsiStr (const S : RawByteString;cp : TSystemCodePage): RawByteString; [external name 'FPC_ANSISTR_TO_ANSISTR'];
{$endif FPC_HAS_CPSTRING}
{$endif FPC_HAS_ANSISTR_TO_ANSISTR}

{$ifndef FPC_HAS_ANSISTR_TO_SHORTSTR}
{$define FPC_HAS_ANSISTR_TO_SHORTSTR}
procedure fpc_AnsiStr_To_ShortStr (out res: shortstring; const S2 : RawByteString);[Public, alias: 'FPC_ANSISTR_TO_SHORTSTR'];  compilerproc;
{
  Converts a AnsiString to a ShortString;
}
Var
  Size : SizeInt;
begin
  if S2='' then
   res:=''
  else
   begin
     Size:=Length(S2);
     If Size>high(res) then
      Size:=high(res);
     Move (S2[1],res[1],Size);
     byte(res[0]):=byte(Size);
   end;
end;
{$endif FPC_HAS_ANSISTR_TO_SHORTSTR}



{$ifndef FPC_HAS_SHORTSTR_TO_ANSISTR}
{$define FPC_HAS_SHORTSTR_TO_ANSISTR}
Function fpc_ShortStr_To_AnsiStr (Const S2 : ShortString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}): RawByteString; compilerproc;
{
  Converts a ShortString to a AnsiString;
}
Var
  Size : SizeInt;
{$ifndef FPC_HAS_CPSTRING}
  cp : TSystemCodePage;
{$endif FPC_HAS_CPSTRING}
begin
{$ifdef FPC_HAS_CPSTRING}
  cp:=TranslatePlaceholderCP(cp);
{$else FPC_HAS_CPSTRING}
  cp:=DefaultSystemCodePage;
{$endif FPC_HAS_CPSTRING}
  Size:=Length(S2);
  Setlength(fpc_ShortStr_To_AnsiStr,Size);
  if Size>0 then
    begin
      fpc_shortstr_ansistr_intern_charmove(S2,1,fpc_ShortStr_To_AnsiStr,0,Size);
      SetCodePage(fpc_ShortStr_To_AnsiStr,cp,False);
    end
end;
{$endif FPC_HAS_SHORTSTR_TO_ANSISTR}


{$ifndef FPC_HAS_CHAR_TO_ANSISTR}
{$define FPC_HAS_CHAR_TO_ANSISTR}
Function fpc_Char_To_AnsiStr(const c : AnsiChar{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}): RawByteString; compilerproc;
{
  Converts a Char to a AnsiString;
}
{$ifndef FPC_HAS_CPSTRING}
var
  cp : TSystemCodePage;
{$endif FPC_HAS_CPSTRING}
begin
{$ifdef FPC_HAS_CPSTRING}
  cp:=TranslatePlaceholderCP(cp);
{$else FPC_HAS_CPSTRING}
  cp:=DefaultSystemCodePage;
{$endif FPC_HAS_CPSTRING}
  Setlength (fpc_Char_To_AnsiStr,1);
  PAnsiChar(fpc_Char_To_AnsiStr)^:=c;
  { Terminating Zero already set by SetLength above }
  SetCodePage(fpc_Char_To_AnsiStr,cp,False);
end;
{$endif FPC_HAS_CHAR_TO_ANSISTR}


{$ifndef FPC_HAS_PCHAR_TO_ANSISTR}
{$define FPC_HAS_PCHAR_TO_ANSISTR}
Function fpc_PChar_To_AnsiStr(const p : PAnsiChar{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}): RawByteString; compilerproc;
Var
  L : SizeInt;
{$ifndef FPC_HAS_CPSTRING}
  cp : TSystemCodePage;
{$endif FPC_HAS_CPSTRING}
begin
  if (not assigned(p)) or (p[0]=#0) Then
    L := 0
  else
    L:=IndexChar(p^,-1,#0);
  SetLength(fpc_PChar_To_AnsiStr,L);
  if L > 0 then
    begin
{$ifdef FPC_HAS_CPSTRING}
      cp:=TranslatePlaceholderCP(cp);
{$else FPC_HAS_CPSTRING}
      cp:=DefaultSystemCodePage;
{$endif FPC_HAS_CPSTRING}
      fpc_pchar_ansistr_intern_charmove(p,0,fpc_PChar_To_AnsiStr,0,L);
      SetCodePage(fpc_PChar_To_AnsiStr,cp,False);
    end;
end;
{$endif FPC_HAS_PCHAR_TO_ANSISTR}


{$ifndef FPC_HAS_CHARARRAY_TO_ANSISTR}
{$define FPC_HAS_CHARARRAY_TO_ANSISTR}
Function fpc_CharArray_To_AnsiStr(const arr: array of ansichar; {$ifdef FPC_HAS_CPSTRING}cp : TSystemCodePage;{$endif FPC_HAS_CPSTRING}zerobased: boolean = true): RawByteString; compilerproc;
var
  i  : SizeInt;
{$ifndef FPC_HAS_CPSTRING}
  cp : TSystemCodePage;
{$endif FPC_HAS_CPSTRING}
begin
  if (zerobased) then
    begin
      if (arr[0]=#0) Then
        i := 0
      else
      begin
        i:=IndexChar(arr,high(arr)+1,#0);
        if i = -1 then
          i := high(arr)+1;
      end;
    end
  else
    i := high(arr)+1;
  SetLength(fpc_CharArray_To_AnsiStr,i);
  if i > 0 then
    begin
{$ifdef FPC_HAS_CPSTRING}
      cp:=TranslatePlaceholderCP(cp);
{$else FPC_HAS_CPSTRING}
      cp:=DefaultSystemCodePage;
{$endif FPC_HAS_CPSTRING}
      fpc_pchar_ansistr_intern_charmove(pansichar(@arr),0,fpc_CharArray_To_AnsiStr,0,i);
      SetCodePage(fpc_CharArray_To_AnsiStr,cp,False);
    end;
end;
{$endif FPC_HAS_ANSISTR_TO_CHARARRAY}


{$ifndef FPC_HAS_ANSISTR_TO_CHARARRAY}
{$define FPC_HAS_ANSISTR_TO_CHARARRAY}
procedure  fpc_ansistr_to_chararray(out res: array of AnsiChar; const src: RawByteString); compilerproc;
var
  len: SizeInt;
begin
  len := length(src);
  if len > length(res) then
    len := length(res);
{$push}{$r-}
  { make sure we don't try to access element 1 of the ansistring if it's nil }
  if len > 0 then
    move(src[1],res[0],len);
  fillchar(res[len],length(res)-len,0);
{$pop}
end;
{$endif FPC_HAS_ANSISTR_TO_CHARARRAY}


{$ifndef FPC_HAS_ANSISTR_COMPARE}
{$define FPC_HAS_ANSISTR_COMPARE}

Function fpc_utf8_Compare(const S1,S2 : RawByteString): SizeInt;

var
  r1,r2 : RawByteString;
begin
  r1:=S1;
  r2:=S2;
  //convert them to utf8 then compare
  SetCodePage(r1,65001);
  SetCodePage(r2,65001);
  Result:=fpc_AnsiStr_Compare(r1,r2);
end;

Function fpc_AnsiStr_Compare(const S1,S2 : RawByteString): SizeInt;[Public,Alias : 'FPC_ANSISTR_COMPARE'];  compilerproc;
{
  Compares 2 AnsiStrings;
  The result is
   <0 if S1<S2
   0 if S1=S2
   >0 if S1>S2
}
Var
  MaxI,Temp : SizeInt;
  cp1,cp2 : TSystemCodePage;
  r1,r2 : RawByteString;
begin
  if pointer(S1)=pointer(S2) then
    begin
      result:=0;
      exit;
    end;
  if (pointer(S1)=nil) then
    begin
      result:=-Length(S2);
      exit;
    end;
  if (pointer(S2)=nil) then
    begin
      result:=Length(S1);
      exit;
    end;
  cp1:=TranslatePlaceholderCP(StringCodePage(S1));
  cp2:=TranslatePlaceholderCP(StringCodePage(S2));
  if cp1=cp2 then
    begin
      Maxi:=Length(S1);
      temp:=Length(S2);
      If MaxI>Temp then
        MaxI:=Temp;
      if MaxI>0 then
        begin
          result:=CompareByte(S1[1],S2[1],MaxI);
          if result=0 then
            result:=Length(S1)-Length(S2);
        end
      else
        result:=Length(S1)-Length(S2);
    end
  else
    Result:=fpc_utf8_compare(s1,s2);
end;
{$endif FPC_HAS_ANSISTR_COMPARE}


{$ifndef FPC_HAS_ANSISTR_COMPARE_EQUAL}
{$define FPC_HAS_ANSISTR_COMPARE_EQUAL}

Function fpc_utf8_Compare_equal(Const S1,S2 : RawByteString): SizeInt;

Var
  r1,r2 : RawByteString;
  L1,L2 : SizeInt;
begin
  r1:=S1;
  r2:=S2;
  //convert them to utf8 then compare
  SetCodePage(r1,65001);
  SetCodePage(r2,65001);
  L1:=Length(r1);
  L2:=Length(r2);
  Result:=L1-L2;
  if Result = 0 then
    if L1>0 then
      result:=CompareByte(r1[1],r2[1],L1);
end;

Function fpc_AnsiStr_Compare_equal(const S1,S2 : RawByteString): SizeInt;[Public,Alias : 'FPC_ANSISTR_COMPARE_EQUAL'];  compilerproc;
{
  Compares 2 AnsiStrings for equality/inequality only;
  The result is
   0 if S1=S2
   <>0 if S1<>S2
}
Var
  MaxI,Temp : SizeInt;
  cp1,cp2 : TSystemCodePage;
begin
  if pointer(S1)=pointer(S2) then
    Exit(0);
  { don't compare strings if one of them is empty }
  if (pointer(S1)=nil) or (pointer(S2)=Nil) then
    Exit(-1);
  cp1:=TranslatePlaceholderCP(StringCodePage(S1));
  cp2:=TranslatePlaceholderCP(StringCodePage(S2));
  if cp1=cp2 then
    begin
      Maxi:=Length(S1);
      temp:=Length(S2);
      Result := Maxi - temp;
      if Result = 0 then
        if MaxI>0 then
          result:=CompareByte(S1[1],S2[1],MaxI);
    end
  else
    begin
    Result:=fpc_utf8_Compare_equal(S1,S2);
    end;
end;
{$endif FPC_HAS_ANSISTR_COMPARE_EQUAL}

{$ifndef FPC_HAS_ANSISTR_RANGECHECK}
{$define FPC_HAS_ANSISTR_RANGECHECK}
Procedure fpc_AnsiStr_RangeCheck(p: Pointer; index: SizeInt);[Public,Alias : 'FPC_ANSISTR_RANGECHECK'];  compilerproc;
begin
  if (p=nil) or (index>PAnsiRec(p-AnsiFirstOff)^.Len) or (Index<1) then
    HandleErrorAddrFrameInd(201,get_pc_addr,get_frame);
end;
{$endif FPC_HAS_ANSISTR_RANGECHECK}

{$ifndef FPC_HAS_ANSISTR_SETLENGTH}
{$define FPC_HAS_ANSISTR_SETLENGTH}
Procedure fpc_AnsiStr_SetLength (Var S : RawByteString; l : SizeInt{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING});[Public,Alias : 'FPC_ANSISTR_SETLENGTH'];  compilerproc;
{
  Sets The length of string S to L.
  Makes sure S is unique, and contains enough room.
}
Var
  Temp : Pointer;
  lens, lena : SizeInt;
begin
  if (l>0) then
    begin
      if Pointer(S)=nil then
        begin
          Pointer(S):=NewAnsiString(L);
{$ifdef FPC_HAS_CPSTRING}
          cp:=TranslatePlaceholderCP(cp);
          PAnsiRec(Pointer(S)-AnsiFirstOff)^.CodePage:=cp;
{$else}
          PAnsiRec(Pointer(S)-AnsiFirstOff)^.CodePage:=DefaultSystemCodePage;
{$endif FPC_HAS_CPSTRING}
        end
      else if PAnsiRec(Pointer(S)-AnsiFirstOff)^.Ref=1 then
        begin
          Temp:=Pointer(s)-AnsiFirstOff;
          lens:=MemSize(Temp);
          lena:=AnsiFirstOff+L+sizeof(AnsiChar);
          { allow shrinking string if that saves at least half of current size }
          if (lena>lens) or ((lens>32) and (lena<=(lens div 2))) then
            begin
              reallocmem(Temp,lena);
              Pointer(S):=Temp+AnsiFirstOff;
            end;
        end
      else
        begin
          { Reallocation is needed... }
          Temp:=NewAnsiString(L);
          PAnsiRec(Pointer(Temp)-AnsiFirstOff)^.CodePage:=PAnsiRec(Pointer(S)-AnsiFirstOff)^.CodePage;
          lens:=length(s);
          if l<=lens then
            lens:=l
          else
            { Also copy a trailing implicit #0 of the original string
              to the new larger string }
            Inc(lens);
          Move(Pointer(S)^,Temp^,lens);
          fpc_ansistr_decr_ref(Pointer(s));
          Pointer(S):=Temp;
        end;
      { Force nil termination in case it gets shorter }
      PByte(Pointer(S)+l)^:=0;
      PAnsiRec(Pointer(S)-AnsiFirstOff)^.Len:=l;
    end
  else  { length=0, deallocate the string }
    fpc_ansistr_decr_ref (Pointer(S));
end;
{$endif FPC_HAS_ANSISTR_SETLENGTH}


{$ifdef EXTRAANSISHORT}
Function fpc_AnsiStr_ShortStr_Compare (Var S1 : Pointer; Var S2 : ShortString): SizeInt;  compilerproc;
{
  Compares a AnsiString with a ShortString;
  The result is
   <0 if S1<S2
   0 if S1=S2
   >0 if S1>S2
}
Var
  i,MaxI,Temp : SizeInt;
begin
  Temp:=0;
  i:=0;
  MaxI:=Length(AnsiString(S1));
  if MaxI>byte(S2[0]) then
    MaxI:=Byte(S2[0]);
  While (i<MaxI) and (Temp=0) do
   begin
     Temp:= PByte(S1+I)^ - Byte(S2[i+1]);
     inc(i);
   end;
  AnsiStr_ShortStr_Compare:=Temp;
end;
{$endif EXTRAANSISHORT}


{*****************************************************************************
                     Public functions, In interface.
*****************************************************************************}

{$ifndef FPC_SYSTEM_HAS_TRUELY_ANSISTR_UNIQUE}
{$define FPC_SYSTEM_HAS_TRUELY_ANSISTR_UNIQUE}
function fpc_truely_ansistr_unique(Var S : Pointer): Pointer;
Var
  SNew : Pointer;
  L    : SizeInt;
begin
  L:=PAnsiRec(Pointer(S)-AnsiFirstOff)^.len;
  SNew:=NewAnsiString (L);
  Move (Pointer(S)^,SNew^,L+1);
  PAnsiRec(SNew-AnsiFirstOff)^.len:=L;
  PAnsiRec(SNew-AnsiFirstOff)^.CodePage:=PAnsiRec(Pointer(S)-AnsiFirstOff)^.CodePage;
  fpc_ansistr_decr_ref (Pointer(S));  { Thread safe }
  pointer(S):=SNew;
  pointer(result):=SNew;
end;
{$endif FPC_SYSTEM_HAS_TRUELY_ANSISTR_UNIQUE}


Function fpc_ansistr_Unique_func(Var S : RawByteString): Pointer; external name 'FPC_ANSISTR_UNIQUE';


Procedure UniqueString(var S : RawByteString);{$ifdef FPC_HAS_CPSTRING}rtlproc;{$endif FPC_HAS_CPSTRING}{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    fpc_ansistr_Unique_func(S);
  end;


{$ifndef FPC_SYSTEM_HAS_ANSISTR_UNIQUE}
{$define FPC_SYSTEM_HAS_ANSISTR_UNIQUE}
// MV: inline the basic checks for case that S is already unique.
// Rest is too complex to inline, so factor that out as a call.
Function fpc_ansistr_Unique(Var S : Pointer): Pointer; [Public,Alias : 'FPC_ANSISTR_UNIQUE']; compilerproc; inline;
{
  Make sure reference count of S is 1,
  using copy-on-write semantics.
}
begin
  pointer(result) := pointer(s);
  If Pointer(S)=Nil then
    exit;
  if PAnsiRec(Pointer(S)-AnsiFirstOff)^.Ref<>1 then
    result:=fpc_truely_ansistr_unique(s);
end;
{$endif FPC_SYSTEM_HAS_ANSISTR_UNIQUE}

{$ifndef FPC_HAS_ANSISTR_COPY}
{$define FPC_HAS_ANSISTR_COPY}
Function Fpc_Ansistr_Copy(Const S : RawByteString; Index,Size : SizeInt): RawByteString;compilerproc;
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
     ResultAddress:=NewAnsiString(Size);
     if ResultAddress<>Nil then
      begin
        Move(Pointer(Pointer(S)+index)^,ResultAddress^,Size);
        PByte(ResultAddress+Size)^:=0;
        PAnsiRec(ResultAddress-AnsiFirstOff)^.Len:=Size;
        PAnsiRec(ResultAddress-AnsiFirstOff)^.CodePage:=PAnsiRec(Pointer(S)-AnsiFirstOff)^.CodePage;
      end;
   end;
   fpc_ansistr_decr_ref(Pointer(fpc_ansistr_copy));
  Pointer(fpc_ansistr_Copy):=ResultAddress;
end;
{$endif FPC_HAS_ANSISTR_COPY}


{$ifndef FPC_HAS_POS_SHORTSTR_ANSISTR}
{$define FPC_HAS_POS_SHORTSTR_ANSISTR}
Function Pos(Const Substr : ShortString; Const Source : RawByteString; Offset : Sizeint = 1) : SizeInt;

var
  i,MaxLen : SizeInt;
  pc : PAnsiChar;
begin
  Pos:=0;
  if (Length(SubStr)>0) and (Offset>0) and (Offset<=Length(Source)) then
   begin
     MaxLen:=Length(source)-Length(SubStr);
     i:=Offset-1;
     pc:=@source[Offset];
     while (i<=MaxLen) do
      begin
        inc(i);
        if (SubStr[1]=pc^) and
           (CompareByte(Substr[1],pc^,Length(SubStr))=0) then
         begin
           Pos:=i;
           exit;
         end;
        inc(pc);
      end;
   end;
end;
{$endif FPC_HAS_POS_SHORTSTR_ANSISTR}


{$ifndef FPC_HAS_POS_ANSISTR_ANSISTR}
{$define FPC_HAS_POS_ANSISTR_ANSISTR}
Function Pos(Const Substr : RawByteString; Const Source : RawByteString; Offset : Sizeint = 1) : SizeInt;
var
  i,MaxLen : SizeInt;
  pc : PAnsiChar;
begin
  Pos:=0;
  if (Length(SubStr)>0) and (Offset>0) and (Offset<=Length(Source)) then
   begin
     MaxLen:=Length(source)-Length(SubStr);
     i:=Offset-1;
     pc:=@source[Offset];
     while (i<=MaxLen) do
      begin
        inc(i);
        if (SubStr[1]=pc^) and
           (CompareByte(Substr[1],pc^,Length(SubStr))=0) then
         begin
           Pos:=i;
           exit;
         end;
        inc(pc);
      end;
   end;
end;
{$endif FPC_HAS_POS_ANSISTR_ANSISTR}


{$ifndef FPC_HAS_POS_ANSICHAR_ANSISTR}
{$define FPC_HAS_POS_ANSICHAR_ANSISTR}
{ Faster version for a char alone. Must be implemented because   }
{ pos(c: char; const s: shortstring) also exists, so otherwise   }
{ using pos(char,pchar) will always call the shortstring version }
{ (exact match for first argument), also with $h+ (JM)           }
Function Pos(c : AnsiChar; Const s : RawByteString; Offset : Sizeint = 1) : SizeInt;
var
  i: SizeInt;
  pc : PAnsiChar;
begin
  Pos:=0;
  If (Offset<1) or (Offset>Length(S)) then 
    exit;
  pc:=@s[OffSet];
  for i:=Offset to length(s) do
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
{$endif FPC_HAS_POS_ANSICHAR_ANSISTR}


{$ifndef FPUNONE}
Function fpc_Val_Real_AnsiStr(Const S : RawByteString; out Code : ValSInt): ValReal; [public, alias:'FPC_VAL_REAL_ANSISTR']; compilerproc;
Var
  SS : String;
begin
  fpc_Val_Real_AnsiStr := 0;
  if length(S) > 255 then
    code := 256
  else
    begin
      SS := S;
      Val(SS,fpc_Val_Real_AnsiStr,code);
    end;
end;
{$endif}


Function fpc_Val_Currency_AnsiStr(Const S : RawByteString; out Code : ValSInt): Currency; [public, alias:'FPC_VAL_CURRENCY_ANSISTR']; compilerproc;
Var
  SS : String;
begin
  if length(S) > 255 then
    begin
      fpc_Val_Currency_AnsiStr := 0;
      code := 256;
    end
  else
    begin
      SS := S;
      Val(SS,fpc_Val_Currency_AnsiStr,code);
    end;
end;


Function fpc_Val_UInt_AnsiStr (Const S : RawByteString; out Code : ValSInt): ValUInt; [public, alias:'FPC_VAL_UINT_ANSISTR']; compilerproc;
Var
  SS : ShortString;
begin
  fpc_Val_UInt_AnsiStr := 0;
  if length(S) > 255 then
    code := 256
  else
    begin
      SS := S;
      Val(SS,fpc_Val_UInt_AnsiStr,code);
    end;
end;


Function fpc_Val_SInt_AnsiStr (DestSize: SizeInt; Const S : RawByteString; out Code : ValSInt): ValSInt; [public, alias:'FPC_VAL_SINT_ANSISTR']; compilerproc;
Var
  SS : ShortString;
begin
  fpc_Val_SInt_AnsiStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS := S;
       fpc_Val_SInt_AnsiStr := int_Val_SInt_ShortStr(DestSize,SS,Code);
    end;
end;


{$ifndef CPU64}

Function fpc_Val_qword_AnsiStr (Const S : RawByteString; out Code : ValSInt): qword; [public, alias:'FPC_VAL_QWORD_ANSISTR']; compilerproc;
Var
  SS : ShortString;
begin
  fpc_Val_qword_AnsiStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS := S;
       Val(SS,fpc_Val_qword_AnsiStr,Code);
    end;
end;


Function fpc_Val_int64_AnsiStr (Const S : RawByteString; out Code : ValSInt): Int64; [public, alias:'FPC_VAL_INT64_ANSISTR']; compilerproc;
Var
  SS : ShortString;
begin
  fpc_Val_int64_AnsiStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS := s;
       Val(SS,fpc_Val_int64_AnsiStr,Code);
    end;
end;

{$endif CPU64}


{$if defined(CPU16) or defined(CPU8)}
Function fpc_Val_longword_AnsiStr (Const S : RawByteString; out Code : ValSInt): longword; [public, alias:'FPC_VAL_LONGWORD_ANSISTR']; compilerproc;
Var
  SS : ShortString;
begin
  fpc_Val_longword_AnsiStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS := S;
       Val(SS,fpc_Val_longword_AnsiStr,Code);
    end;
end;


Function fpc_Val_longint_AnsiStr (Const S : RawByteString; out Code : ValSInt): LongInt; [public, alias:'FPC_VAL_LONGINT_ANSISTR']; compilerproc;
Var
  SS : ShortString;
begin
  fpc_Val_longint_AnsiStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS := s;
       Val(SS,fpc_Val_longint_AnsiStr,Code);
    end;
end;


Function fpc_Val_word_AnsiStr (Const S : RawByteString; out Code : ValSInt): word; [public, alias:'FPC_VAL_WORD_ANSISTR']; compilerproc;
Var
  SS : ShortString;
begin
  fpc_Val_word_AnsiStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS := S;
       Val(SS,fpc_Val_word_AnsiStr,Code);
    end;
end;


Function fpc_Val_smallint_AnsiStr (Const S : RawByteString; out Code : ValSInt): smallint; [public, alias:'FPC_VAL_SMALLINT_ANSISTR']; compilerproc;
Var
  SS : ShortString;
begin
  fpc_Val_smallint_AnsiStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS := s;
       Val(SS,fpc_Val_smallint_AnsiStr,Code);
    end;
end;
{$endif CPU16 or CPU8}


{$if defined(CPU8)}
Function fpc_Val_word_AnsiStr (Const S : RawByteString; out Code : ValSInt): longword; [public, alias:'FPC_VAL_WORD_ANSISTR']; compilerproc;
Var
  SS : ShortString;
begin
  fpc_Val_longword_AnsiStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS := S;
       Val(SS,fpc_Val_longword_AnsiStr,Code);
    end;
end;


Function fpc_Val_smallint_AnsiStr (Const S : RawByteString; out Code : ValSInt): LongInt; [public, alias:'FPC_VAL_SMALLINT_ANSISTR']; compilerproc;
Var
  SS : ShortString;
begin
  fpc_Val_longint_AnsiStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS := s;
       Val(SS,fpc_Val_longint_AnsiStr,Code);
    end;
end;
{$endif CPU8}


{$ifndef FPUNONE}
procedure fpc_AnsiStr_Float(d : ValReal;len,fr,rt : SizeInt;out s : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING});[public,alias:'FPC_ANSISTR_FLOAT']; compilerproc; inline;
var
  ss: ShortString;
begin
  str_real(len,fr,d,treal_type(rt),ss);
  s:=ss;
  {$ifdef FPC_HAS_CPSTRING}
  SetCodePage(s,cp,false);
  {$endif FPC_HAS_CPSTRING}
end;
{$endif}

{$ifndef FPC_STR_ENUM_INTERN}
procedure fpc_ansistr_enum(ordinal,len:sizeint;typinfo,ord2strindex:pointer;out s:RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING});[public,alias:'FPC_ANSISTR_ENUM'];compilerproc; inline;

var ss:shortstring;

begin
  fpc_shortstr_enum(ordinal,len,typinfo,ord2strindex,ss);
  s:=ss;
  {$ifdef FPC_HAS_CPSTRING}
  SetCodePage(s,cp,false);
  {$endif FPC_HAS_CPSTRING}
end;
{$endif FPC_STR_ENUM_INTERN}


procedure fpc_ansistr_bool(b : boolean;len:sizeint;out s:RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING});[public,alias:'FPC_ANSISTR_BOOL'];compilerproc; inline;
var
  ss:shortstring;
begin
  fpc_shortstr_bool(b,len,ss);
  s:=ss;
  {$ifdef FPC_HAS_CPSTRING}
  SetCodePage(s,cp,false);
  {$endif FPC_HAS_CPSTRING}
end;


{$ifndef FPC_STR_ENUM_INTERN}
function fpc_val_enum_ansistr(str2ordindex:pointer;const s:RawByteString;out code:valsint):longint; [public, alias:'FPC_VAL_ENUM_ANSISTR']; compilerproc;

begin
  fpc_val_enum_ansistr:=fpc_val_enum_shortstr(str2ordindex,s,code);
end;
{$endif FPC_STR_ENUM_INTERN}


procedure fpc_AnsiStr_Currency(c : currency;len,fr : SizeInt;out s : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING});[public,alias:'FPC_ANSISTR_CURRENCY']; compilerproc; inline;
var
  ss: ShortString;
begin
  str(c:len:fr,ss);
  s:=ss;
  {$ifdef FPC_HAS_CPSTRING}
  SetCodePage(s,cp,false);
  {$endif FPC_HAS_CPSTRING}
end;

Procedure fpc_AnsiStr_UInt(v : ValUInt;Len : SizeInt; out S : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING});[Public,Alias : 'FPC_ANSISTR_VALUINT']; compilerproc; inline;
Var
  SS : ShortString;
begin
  str(v:Len,SS);
  S:=SS;
  {$ifdef FPC_HAS_CPSTRING}
  SetCodePage(s,cp,false);
  {$endif FPC_HAS_CPSTRING}
end;



Procedure fpc_AnsiStr_SInt(v : ValSInt;Len : SizeInt; out S : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING});[Public,Alias : 'FPC_ANSISTR_VALSINT']; compilerproc; inline;
Var
  SS : ShortString;
begin
  str (v:Len,SS);
  S:=SS;
  {$ifdef FPC_HAS_CPSTRING}
  SetCodePage(s,cp,false);
  {$endif FPC_HAS_CPSTRING}
end;


{$ifndef CPU64}

Procedure fpc_AnsiStr_QWord(v : QWord;Len : SizeInt; out S : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING});[Public,Alias : 'FPC_ANSISTR_QWORD']; compilerproc; inline;
Var
  SS : ShortString;
begin
  str(v:Len,SS);
  S:=SS;
  {$ifdef FPC_HAS_CPSTRING}
  SetCodePage(s,cp,false);
  {$endif FPC_HAS_CPSTRING}
end;

Procedure fpc_AnsiStr_Int64(v : Int64; Len : SizeInt; out S : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING});[Public,Alias : 'FPC_ANSISTR_INT64']; compilerproc; inline;
Var
  SS : ShortString;
begin
  str (v:Len,SS);
  S:=SS;
  {$ifdef FPC_HAS_CPSTRING}
  SetCodePage(s,cp,false);
  {$endif FPC_HAS_CPSTRING}
end;

{$endif CPU64}

{$if defined(CPU16) or defined(CPU8)}
Procedure fpc_AnsiStr_LongWord(v : LongWord;Len : SizeInt; out S : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING});[Public,Alias : 'FPC_ANSISTR_LONGWORD']; compilerproc; inline;
Var
  SS : ShortString;
begin
  str(v:Len,SS);
  S:=SS;
  {$ifdef FPC_HAS_CPSTRING}
  SetCodePage(s,cp,false);
  {$endif FPC_HAS_CPSTRING}
end;

Procedure fpc_AnsiStr_LongInt(v : LongInt; Len : SizeInt; out S : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING});[Public,Alias : 'FPC_ANSISTR_LONGINT']; compilerproc; inline;
Var
  SS : ShortString;
begin
  str (v:Len,SS);
  S:=SS;
  {$ifdef FPC_HAS_CPSTRING}
  SetCodePage(s,cp,false);
  {$endif FPC_HAS_CPSTRING}
end;

Procedure fpc_AnsiStr_Word(v : Word;Len : SizeInt; out S : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING});[Public,Alias : 'FPC_ANSISTR_WORD']; compilerproc; inline;
Var
  SS : ShortString;
begin
  str(v:Len,SS);
  S:=SS;
  {$ifdef FPC_HAS_CPSTRING}
  SetCodePage(s,cp,false);
  {$endif FPC_HAS_CPSTRING}
end;

Procedure fpc_AnsiStr_SmallInt(v : SmallInt; Len : SizeInt; out S : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING});[Public,Alias : 'FPC_ANSISTR_SMALLINT']; compilerproc; inline;
Var
  SS : ShortString;
begin
  str (v:Len,SS);
  S:=SS;
  {$ifdef FPC_HAS_CPSTRING}
  SetCodePage(s,cp,false);
  {$endif FPC_HAS_CPSTRING}
end;
{$endif CPU16 or CPU8}

Procedure {$ifdef VER3_0}Delete{$else}fpc_ansistr_delete{$endif}(Var S : RawByteString; Index,Size: SizeInt);
Var
  LS : SizeInt;
begin
  ls:=Length(S);
  If (Index>LS) or (Index<=0) or (Size<=0) then
    exit;
  UniqueString(S);
  If (Size>LS-Index) then   // Size+Index gives overflow ??
     Size:=LS-Index+1;
  If (Size<=LS-Index) then
    begin
      Dec(Index);
      fpc_pchar_ansistr_intern_charmove(pchar(S),Index+Size,S,Index,LS-Index-Size+1);
    end;
  Setlength(S,LS-Size);
end;


Procedure {$ifdef VER3_0}Insert{$else}fpc_ansistr_insert{$endif}(Const Source : RawByteString; Var S : RawByteString; Index : SizeInt);
var
  Temp : RawByteString;
  LS : SizeInt;
  cp : TSystemCodePage;
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
  if length(S)<>0 then
    cp:=TranslatePlaceholderCP(StringCodePage(S))
  else
    cp:=TranslatePlaceholderCP(StringCodePage(Source));
  SetCodePage(Temp,cp,false);
  If Index>0 then
    fpc_pchar_ansistr_intern_charmove(pchar(S),0,Temp,0,Index);
  fpc_pchar_ansistr_intern_charmove(pchar(Source),0,Temp,Index,Length(Source));
  If (LS-Index)>0 then
    fpc_pchar_ansistr_intern_charmove(pchar(S),Index,Temp,Length(Source)+Index,LS-Index);
  S:=Temp;
end;


{$ifndef FPC_HAS_ANSISTR_OF_CHAR}
{$define FPC_HAS_ANSISTR_OF_CHAR}
Function StringOfChar(c : Ansichar;l : SizeInt) : AnsiString;
begin
  SetLength(StringOfChar,l);
  FillChar(Pointer(StringOfChar)^,Length(StringOfChar),c);
end;
{$endif FPC_HAS_ANSISTR_OF_CHAR}


{$ifdef FPC_HAS_CPSTRING}
Procedure fpc_setstring_ansistr_pansichar(out S : RawByteString; Buf : PAnsiChar; Len : SizeInt; cp: TSystemCodePage); rtlproc; compilerproc;
{$else}
Procedure SetString(out S : AnsiString; Buf : PAnsiChar; Len : SizeInt);
{$endif}
begin
  SetLength(S,Len);
{$ifdef FPC_HAS_CPSTRING}
  SetCodePage(S,cp,false);
{$endif}
  If (Buf<>Nil) then
    fpc_pchar_ansistr_intern_charmove(Buf,0,S,0,Len);
end;


{$ifdef FPC_HAS_CPSTRING}
Procedure fpc_setstring_ansistr_pwidechar(out S : RawByteString; Buf : PWideChar; Len : SizeInt; cp: TSystemCodePage); rtlproc; compilerproc;
{$else}
Procedure SetString(out S : AnsiString; Buf : PWideChar; Len : SizeInt);
{$endif}
begin
  if (Buf<>nil) and (Len>0) then
    widestringmanager.Wide2AnsiMoveProc(Buf,S,{$ifdef FPC_HAS_CPSTRING}cp{$else}DefaultSystemCodePage{$endif},Len)
  else
    begin
      SetLength(S, Len);
{$ifdef FPC_HAS_CPSTRING}
      SetCodePage(S,cp,false);
{$endif}
    end;
end;


{$ifndef FPC_HAS_UPCASE_ANSISTR}
{$define FPC_HAS_UPCASE_ANSISTR}
function upcase(const s : ansistring) : ansistring;
var
  i : SizeInt;
begin
  Setlength(result,length(s));
  for i := 1 to length (s) do
    result[i] := upcase(s[i]);
end;
{$endif FPC_HAS_UPCASE_ANSISTR}


{$ifndef FPC_HAS_LOWERCASE_ANSISTR}
{$define FPC_HAS_LOWERCASE_ANSISTR}
function lowercase(const s : ansistring) : ansistring;
var
  i : SizeInt;
begin
  Setlength(result,length(s));
  for i := 1 to length (s) do
    result[i] := lowercase(s[i]);
end;
{$endif FPC_HAS_LOWERCASE_ANSISTR}


{$ifndef FPC_HAS_ANSISTR_STRINGCODEPAGE}
{$define FPC_HAS_ANSISTR_STRINGCODEPAGE}
function StringCodePage(const S: RawByteString): TSystemCodePage; overload;
  begin
{$ifdef FPC_HAS_CPSTRING}
    if assigned(Pointer(S)) then
      Result:=PAnsiRec(pointer(S)-AnsiFirstOff)^.CodePage
    else
{$endif FPC_HAS_CPSTRING}
      Result:=DefaultSystemCodePage;
  end;
{$endif FPC_HAS_ANSISTR_STRINGCODEPAGE}


{$ifndef FPC_HAS_ANSISTR_STRINGELEMENTSIZE}
{$define FPC_HAS_ANSISTR_STRINGELEMENTSIZE}
function StringElementSize(const S: RawByteString): Word; overload;
  begin
    if assigned(Pointer(S)) then
      Result:=PAnsiRec(pointer(S)-AnsiFirstOff)^.ElementSize
    else
      Result:=SizeOf(AnsiChar);
  end;
{$endif FPC_HAS_ANSISTR_STRINGELEMENTSIZE}


{$ifndef FPC_HAS_ANSISTR_STRINGREFCOUNT}
{$define FPC_HAS_ANSISTR_STRINGREFCOUNT}
function StringRefCount(const S: RawByteString): SizeInt; overload;
  begin
    if assigned(Pointer(S)) then
      Result:=PAnsiRec(pointer(S)-AnsiFirstOff)^.Ref
    else
      Result:=0;
  end;
{$endif FPC_HAS_ANSISTR_STRINGREFCOUNT}


{$ifndef FPC_HAS_ANSISTR_SETCODEPAGE}
{$define FPC_HAS_ANSISTR_SETCODEPAGE}
procedure InternalSetCodePage(var s : RawByteString; CodePage : TSystemCodePage; Convert : Boolean = True);
  begin
    if Convert then
      begin
{$ifdef FPC_HAS_CPSTRING}
        s:=fpc_AnsiStr_To_AnsiStr(s,CodePage);
{$else FPC_HAS_CPSTRING}
        UniqueString(s);
        PAnsiRec(pointer(s)-AnsiFirstOff)^.CodePage:=CodePage;
{$endif FPC_HAS_CPSTRING}
      end
    else
      begin
        UniqueString(s);
        PAnsiRec(pointer(s)-AnsiFirstOff)^.CodePage:=CodePage;
      end;
  end;


{ use this wrapper for the simple case to avoid the generation of a temp. ansistring which causes
  extra exception frames }
procedure SetCodePage(var s : RawByteString; CodePage : TSystemCodePage; Convert : Boolean = True);
  var
    OrgCodePage,
    TranslatedCodePage,
    TranslatedCurrentCodePage: TSystemCodePage;
  begin
    if (S='') then
      exit;
    { if the codepage are identical, we don't have to do anything (even if the
      string has multiple references) }
    OrgCodePage:=PAnsiRec(pointer(S)-AnsiFirstOff)^.CodePage;
    if OrgCodePage=CodePage then
      exit;
    { if we're just replacing a placeholder code page with its actual value or
      vice versa, we don't have to perform any conversion }
    TranslatedCurrentCodePage:=TranslatePlaceholderCP(OrgCodePage);
    TranslatedCodePage:=TranslatePlaceholderCP(CodePage);
    Convert:=Convert and
      (TranslatedCurrentCodePage<>TranslatedCodePage);
    if not Convert and (PAnsiRec(pointer(S)-AnsiFirstOff)^.Ref=1) then
      PAnsiRec(pointer(S)-AnsiFirstOff)^.CodePage:=CodePage
    else
      InternalSetCodePage(S,CodePage,Convert);
  end;
{$endif FPC_HAS_ANSISTR_SETCODEPAGE}


procedure SetMultiByteConversionCodePage(CodePage: TSystemCodePage);
  begin
    DefaultSystemCodePage:=CodePage;
  end;


procedure SetMultiByteFileSystemCodePage(CodePage: TSystemCodePage);
  begin
    DefaultFileSystemCodePage:=CodePage;
  end;


procedure SetMultiByteRTLFileSystemCodePage(CodePage: TSystemCodePage);
  begin
    DefaultRTLFileSystemCodePage:=CodePage;
  end;

