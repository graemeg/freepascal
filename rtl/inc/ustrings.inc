{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2005 by Florian Klaempfl,
    member of the Free Pascal development team.

    This file implements support routines for UTF-8 strings with FPC

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{$ifndef FPC_UNICODESTRING_TYPE_DEFINED}
{$define FPC_UNICODESTRING_TYPE_DEFINED}
{
  This file contains the implementation of the UnicodeString type,
  and all things that are needed for it.
  UnicodeString is defined as a 'silent' punicodechar :
  a punicodechar that points to :

  @-8  : SizeInt for reference count;
  @-4  : SizeInt for size; size=number of chars. Multiply with
         sizeof(UnicodeChar) to get the number of bytes. This is compatible with Delphi.
  @    : String + Terminating #0;
  Punicodechar(Unicodestring) is a valid typecast.
  So WS[i] is converted to the address @WS+i-1.

  Constants should be assigned a reference count of -1
  Meaning that they can't be disposed of.
}

Type
  PUnicodeRec = ^TUnicodeRec;
<<<<<<< HEAD
  TUnicodeRec = Record
    CodePage    : TSystemCodePage;
    ElementSize : Word;
=======
  TUnicodeRec = Packed Record
    CodePage    : TSystemCodePage;
	ElementSize : Word;
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
{$ifdef CPU64}	
    { align fields  }
	Dummy       : DWord;
{$endif CPU64}
    Ref         : SizeInt;
    Len         : SizeInt;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    First       : UnicodeChar;
>>>>>>> graemeg/cpstrnew
=======
    First       : UnicodeChar;
>>>>>>> graemeg/cpstrnew
=======
    First       : UnicodeChar;
>>>>>>> graemeg/cpstrnew
=======
    First       : UnicodeChar;
>>>>>>> origin/cpstrnew
  end;

Const
  UnicodeFirstOff = SizeOf(TUnicodeRec);
{$endif FPC_UNICODESTRING_TYPE_DEFINED}

{
  Default UnicodeChar <-> Char conversion is to only convert the
  lower 127 chars, all others are translated to '?'.

  These routines can be overridden for the Current Locale
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPC_HAS_DEFAULT_UNICODE_2_ANSI_MOVE}
{$define FPC_HAS_DEFAULT_UNICODE_2_ANSI_MOVE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure DefaultUnicode2AnsiMove(source:punicodechar;var dest:RawByteString;cp : TSystemCodePage;len:SizeInt);
var
  i : SizeInt;
  p : PAnsiChar;
begin
  setlength(dest,len);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if not assigned(pointer(dest)) then
    exit;
  SetCodePage(dest,cp,false);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  p:=pointer(dest);         {SetLength guarantees that dest is unique}
  for i:=1 to len do
    begin
      if word(source^)<256 then
        p^:=char(word(source^))
      else
        p^:='?';
      inc(source);
      inc(p);
    end;
end;
<<<<<<< HEAD
{$endif FPC_HAS_DEFAULT_UNICODE_2_ANSI_MOVE}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPC_HAS_DEFAULT_ANSI_2_UNICODE}
{$define FPC_HAS_DEFAULT_ANSI_2_UNICODE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======


>>>>>>> origin/cpstrnew
procedure DefaultAnsi2UnicodeMove(source:pchar;cp : TSystemCodePage;var dest:unicodestring;len:SizeInt);
var
  i : SizeInt;
  p : PUnicodeChar;
begin
  setlength(dest,len);
  p:=pointer(dest);         {SetLength guarantees that dest is unique}
  for i:=1 to len do
    begin
      p^:=unicodechar(byte(source^));
      inc(source);
      inc(p);
    end;
end;
{$endif FPC_HAS_DEFAULT_ANSI_2_UNICODE}


{$ifndef FPC_HAS_BUILTIN_WIDESTR_MANAGER}
function DefaultCharLengthPChar(const Str: PChar): PtrInt;
  begin
    DefaultCharLengthPChar:=length(Str);
  end;


function DefaultCodePointLength(const Str: PChar; MaxLookAead: PtrInt): Ptrint;
  begin
    if str[0]<>#0 then
      DefaultCodePointLength:=1
    else
      DefaultCodePointLength:=0;
  end;
{$endif FPC_HAS_BUILTIN_WIDESTR_MANAGER}


<<<<<<< HEAD
function DefaultGetStandardCodePage(const stdcp: TStandardCodePageEnum): TSystemCodePage;
  begin
    { don't raise an exception here. We need this for text file handling }
    if stdcp<>scpFileSystemSingleByte then
      Result:=DefaultSystemCodePage
    else
      { we could return UTF-8 here in case of FPCRTL_FILESYSTEM_UTF8, but
        without a fully functional widestring manager that will probably cause
        more problems that it solves }
      Result:=DefaultFileSystemCodePage
  end;

=======
function DefaultCharLengthPChar(const Str: PChar): PtrInt;
  begin
    DefaultCharLengthPChar:=length(Str);
  end;


function DefaultCodePointLength(const Str: PChar; MaxLookAead: PtrInt): Ptrint;
  begin
    if str[0]<>#0 then
      DefaultCodePointLength:=1
    else
      DefaultCodePointLength:=0;
  end;


<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
function DefaultCharLengthPChar(const Str: PChar): PtrInt;
  begin
    DefaultCharLengthPChar:=length(Str);
  end;


function DefaultCodePointLength(const Str: PChar; MaxLookAead: PtrInt): Ptrint;
  begin
    if str[0]<>#0 then
      DefaultCodePointLength:=1
    else
      DefaultCodePointLength:=0;
  end;


<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
function DefaultCharLengthPChar(const Str: PChar): PtrInt;
  begin
    DefaultCharLengthPChar:=length(Str);
  end;


<<<<<<< HEAD
=======
function DefaultCharLengthPChar(const Str: PChar): PtrInt;
  begin
    DefaultCharLengthPChar:=length(Str);
  end;


>>>>>>> origin/cpstrnew
function DefaultCodePointLength(const Str: PChar; MaxLookAead: PtrInt): Ptrint;
  begin
    if str[0]<>#0 then
      DefaultCodePointLength:=1
    else
      DefaultCodePointLength:=0;
  end;


<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Procedure GetUnicodeStringManager (Var Manager : TUnicodeStringManager);
begin
  manager:=widestringmanager;
end;


Procedure SetUnicodeStringManager (Const New : TUnicodeStringManager; Var Old: TUnicodeStringManager);
begin
  Old:=widestringmanager;
  widestringmanager:=New;
end;


Procedure SetUnicodeStringManager (Const New : TUnicodeStringManager);
begin
  widestringmanager:=New;
end;


Procedure GetWideStringManager (Var Manager : TUnicodeStringManager);
begin
  manager:=widestringmanager;
end;


Procedure SetWideStringManager (Const New : TUnicodeStringManager; Var Old: TUnicodeStringManager);
begin
  Old:=widestringmanager;
  widestringmanager:=New;
end;


Procedure SetWideStringManager (Const New : TUnicodeStringManager);
begin
  widestringmanager:=New;
end;


{****************************************************************************
                    Internal functions, not in interface.
****************************************************************************}

procedure UnicodeStringError;
  begin
    HandleErrorAddrFrameInd(204,get_pc_addr,get_frame);
  end;


{$ifndef FPC_HAS_NEW_UNICODESTRING}
{$define FPC_HAS_NEW_UNICODESTRING}
Function NewUnicodeString(Len : SizeInt) : Pointer;
{
  Allocate a new UnicodeString on the heap.
  initialize it to zero length and reference count 1.
}
Var
  P : Pointer;
begin
  GetMem(P,Len*sizeof(UnicodeChar)+(UnicodeFirstOff+sizeof(UnicodeChar)));
  If P<>Nil then
    begin
      PUnicodeRec(P)^.Len:=Len;       { Initial length }
      PUnicodeRec(P)^.Ref:=1;         { Initial Refcount }
      PUnicodeRec(P)^.CodePage:=DefaultUnicodeCodePage;
      PUnicodeRec(P)^.ElementSize:=SizeOf(UnicodeChar);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      inc(p,UnicodeFirstOff);         { Points to string now }
      PUnicodeChar(P)^:=#0;           { Terminating #0 }
=======
      PUnicodeRec(P)^.First:=#0;      { Terminating #0 }
      inc(p,UnicodeFirstOff);         { Points to string now }
>>>>>>> graemeg/cpstrnew
=======
      PUnicodeRec(P)^.First:=#0;      { Terminating #0 }
      inc(p,UnicodeFirstOff);         { Points to string now }
>>>>>>> graemeg/cpstrnew
=======
      PUnicodeRec(P)^.First:=#0;      { Terminating #0 }
      inc(p,UnicodeFirstOff);         { Points to string now }
>>>>>>> graemeg/cpstrnew
=======
      PUnicodeRec(P)^.First:=#0;      { Terminating #0 }
      inc(p,UnicodeFirstOff);         { Points to string now }
>>>>>>> origin/cpstrnew
    end
  else
    UnicodeStringError;
  NewUnicodeString:=P;
end;
{$endif FPC_HAS_NEW_UNICODESTRING}


{$ifndef FPC_HAS_UNICODESTR_DECR_REF}
{$define FPC_HAS_UNICODESTR_DECR_REF}
Procedure fpc_UnicodeStr_Decr_Ref (Var S : Pointer);[Public,Alias:'FPC_UNICODESTR_DECR_REF']; compilerproc;
{
  Decreases the ReferenceCount of a non constant unicodestring;
  If the reference count is zero, deallocate the string;
}
Var
  p: PUnicodeRec;
Begin
  { Zero string }
  if S=Nil then
    exit;
  { check for constant strings ...}
  p:=PUnicodeRec(S-UnicodeFirstOff);
  S:=nil;
  if p^.Ref<0 then
    exit;

  { declocked does a MT safe dec and returns true, if the counter is 0 }
<<<<<<< HEAD
  if declocked(p^.Ref) then
    FreeMem(p);
=======
  if declocked(l^) then
    { Ref count dropped to zero remove }
    DisposeUnicodeString(S);
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

{ alias for internal use }
Procedure fpc_UnicodeStr_Decr_Ref (Var S : Pointer);[external name 'FPC_UNICODESTR_DECR_REF'];
{$endif FPC_HAS_UNICODESTR_DECR_REF}


{$ifndef FPC_HAS_UNICODESTR_INCR_REF}
{$define FPC_HAS_UNICODESTR_INCR_REF}
Procedure fpc_UnicodeStr_Incr_Ref(S : Pointer);[Public,Alias:'FPC_UNICODESTR_INCR_REF']; compilerproc;
  Begin
    If S=Nil then
      exit;
    { constant string ? }
    If PUnicodeRec(S-UnicodeFirstOff)^.Ref<0 then
      exit;
    inclocked(PUnicodeRec(S-UnicodeFirstOff)^.Ref);
  end;

{ alias for internal use }
Procedure fpc_UnicodeStr_Incr_Ref (S : Pointer);[external name 'FPC_UNICODESTR_INCR_REF'];
{$endif FPC_HAS_UNICODESTR_INCR_REF}

{$ifndef FPC_HAS_UNICODESTR_TO_SHORTSTR}
{$define FPC_HAS_UNICODESTR_TO_SHORTSTR}
procedure fpc_UnicodeStr_To_ShortStr (out res: ShortString;const S2 : UnicodeString); [Public, alias: 'FPC_UNICODESTR_TO_SHORTSTR'];compilerproc;
{
  Converts a UnicodeString to a ShortString;
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
      widestringmanager.Unicode2AnsiMoveProc(PUnicodeChar(S2),temp,DefaultSystemCodePage,Size);
      res:=temp;
    end;
end;
{$endif FPC_HAS_UNICODESTR_TO_SHORTSTR}


{$ifndef FPC_HAS_SHORTSTR_TO_UNICODESTR}
{$define FPC_HAS_SHORTSTR_TO_UNICODESTR}
Function fpc_ShortStr_To_UnicodeStr (Const S2 : ShortString): UnicodeString;compilerproc;
{
  Converts a ShortString to a UnicodeString;
}
Var
  Size : SizeInt;
begin
  result:='';
  Size:=Length(S2);
  if Size>0 then
<<<<<<< HEAD
    widestringmanager.Ansi2UnicodeMoveProc(PChar(@S2[1]),DefaultSystemCodePage,result,Size);
=======
    begin
      widestringmanager.Ansi2UnicodeMoveProc(PChar(@S2[1]),DefaultSystemCodePage,result,Size);
      { Terminating Zero }
      PUnicodeChar(Pointer(fpc_ShortStr_To_UnicodeStr)+Size*sizeof(UnicodeChar))^:=#0;
    end;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
end;
{$endif FPC_HAS_SHORTSTR_TO_UNICODESTR}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPC_HAS_UNICODESTR_TO_ANSISTR}
{$define FPC_HAS_UNICODESTR_TO_ANSISTR}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
end;


>>>>>>> origin/cpstrnew
Function fpc_UnicodeStr_To_AnsiStr (const S2 : UnicodeString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}): AnsiString; compilerproc;
{
  Converts a UnicodeString to an AnsiString
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
    widestringmanager.Unicode2AnsiMoveProc(PUnicodeChar(Pointer(S2)),result,cp,Size);
  end;
end;
{$endif FPC_HAS_UNICODESTR_TO_ANSISTR}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPC_HAS_ANSISTR_TO_UNICODESTR}
{$define FPC_HAS_ANSISTR_TO_UNICODESTR}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
    if cp=$ffff then
      cp:=DefaultSystemCodePage;
    widestringmanager.Unicode2AnsiMoveProc(PUnicodeChar(Pointer(S2)),result,cp,Size);
  end;
end;


>>>>>>> origin/cpstrnew
Function fpc_AnsiStr_To_UnicodeStr (Const S2 : RawByteString): UnicodeString; compilerproc;
{
  Converts an AnsiString to a UnicodeString;
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
    widestringmanager.Ansi2UnicodeMoveProc(PChar(S2),cp,result,Size);
  end;
end;
{$endif FPC_HAS_ANSISTR_TO_UNICODESTR}


{$ifndef FPC_HAS_UNICODESTR_TO_WIDESTR}
{$define FPC_HAS_UNICODESTR_TO_WIDESTR}
Function fpc_UnicodeStr_To_WideStr (const S2 : UnicodeString): WideString; compilerproc;
  begin
    SetLength(Result,Length(S2));
    Move(pointer(S2)^,Pointer(Result)^,Length(S2)*sizeof(WideChar));
  end;
{$endif FPC_HAS_UNICODESTR_TO_WIDESTR}


{$ifndef FPC_HAS_WIDESTR_TO_UNICODESTR}
{$define FPC_HAS_WIDESTR_TO_UNICODESTR}
Function fpc_WideStr_To_UnicodeStr (Const S2 : WideString): UnicodeString; compilerproc;
  begin
    SetLength(Result,Length(S2));
    Move(pointer(S2)^,Pointer(Result)^,Length(S2)*sizeof(WideChar));
  end;
<<<<<<< HEAD
{$endif FPC_HAS_WIDESTR_TO_UNICODESTR}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPC_HAS_PWIDECHAR_TO_UNICODESTR}
{$define FPC_HAS_PWIDECHAR_TO_UNICODESTR}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======


>>>>>>> origin/cpstrnew
Function fpc_PUnicodeChar_To_AnsiStr(const p : punicodechar{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}): ansistring; compilerproc;
var
  Size : SizeInt;
{$ifndef FPC_HAS_CPSTRING}
  cp : TSystemCodePage;
{$endif FPC_HAS_CPSTRING}
begin
{$ifndef FPC_HAS_CPSTRING}
  cp:=$ffff;
{$endif FPC_HAS_CPSTRING}
  result:='';
  if p=nil then
    exit;
  Size := IndexWord(p^, -1, 0);
  if Size>0 then
    widestringmanager.Unicode2AnsiMoveProc(P,result,cp,Size);
end;


Function fpc_PUnicodeChar_To_UnicodeStr(const p : punicodechar): unicodestring; compilerproc;
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
      Move(p^,PUnicodeChar(Pointer(result))^,Size*sizeof(UnicodeChar));
      { Terminating Zero }
      PUnicodeChar(Pointer(result)+Size*sizeof(UnicodeChar))^:=#0;
   end;
end;


>>>>>>> graemeg/cpstrnew
Function fpc_PWideChar_To_UnicodeStr(const p : pwidechar): unicodestring; compilerproc;
var
  Size : SizeInt;
begin
  result:='';
  if p=nil then
    exit;
  Size := IndexWord(p^, -1, 0);
  Setlength(result,Size);
  if Size>0 then
<<<<<<< HEAD
    Move(p^,PUnicodeChar(Pointer(result))^,Size*sizeof(UnicodeChar));
=======
   begin
      Move(p^,PUnicodeChar(Pointer(result))^,Size*sizeof(UnicodeChar));
      { Terminating Zero }
      PUnicodeChar(Pointer(result)+Size*sizeof(UnicodeChar))^:=#0;
   end;
end;


{$ifndef FPC_STRTOSHORTSTRINGPROC}
Function fpc_PUnicodeChar_To_ShortStr(const p : punicodechar): shortstring; compilerproc;
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
      widestringmanager.Unicode2AnsiMoveProc(p,temp,Size);
      result:=temp;
    end;
end;
{$else FPC_STRTOSHORTSTRINGPROC}
procedure fpc_PUnicodeChar_To_ShortStr(out res : shortstring;const p : punicodechar); compilerproc;
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
      widestringmanager.Unicode2AnsiMoveProc(p,temp,DefaultSystemCodePage,Size);
      res:=temp;
    end;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
end;
{$endif FPC_HAS_PWIDECHAR_TO_UNICODESTR}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPC_HAS_PWIDECHAR_TO_ANSISTR}
{$define FPC_HAS_PWIDECHAR_TO_ANSISTR}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
end;
{$endif FPC_STRTOSHORTSTRINGPROC}


>>>>>>> origin/cpstrnew
Function fpc_PWideChar_To_AnsiStr(const p : pwidechar{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}): ansistring; compilerproc;
var
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
  if p=nil then
    exit;
  Size := IndexWord(p^, -1, 0);
  if Size>0 then
    widestringmanager.Wide2AnsiMoveProc(P,result,cp,Size);
end;
{$endif FPC_HAS_PWIDECHAR_TO_ANSISTR}


{$ifndef FPC_HAS_PWIDECHAR_TO_SHORTSTR}
{$define FPC_HAS_PWIDECHAR_TO_SHORTSTR}
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
      widestringmanager.Wide2AnsiMoveProc(p,temp,DefaultSystemCodePage,Size);
      res:=temp;
    end;
end;
{$endif FPC_HAS_PWIDECHAR_TO_SHORTSTR}


{$ifndef FPC_HAS_UNICODESTR_ASSIGN}
{$define FPC_UNICODESTR_ASSIGN}
{ checked against the ansistring routine, 2001-05-27 (FK) }
Procedure fpc_UnicodeStr_Assign (Var S1 : Pointer;S2 : Pointer);[Public,Alias:'FPC_UNICODESTR_ASSIGN']; compilerproc;
{
  Assigns S2 to S1 (S1:=S2), taking in account reference counts.
}
begin
  If S2<>nil then
    If PUnicodeRec(S2-UnicodeFirstOff)^.Ref>0 then
      inclocked(PUnicodeRec(S2-UnicodeFirstOff)^.ref);
  { Decrease the reference count on the old S1 }
  fpc_unicodestr_decr_ref (S1);
  s1:=s2;
end;


{ alias for internal use }
Procedure fpc_UnicodeStr_Assign (Var S1 : Pointer;S2 : Pointer);[external name 'FPC_UNICODESTR_ASSIGN'];
{$endif FPC_UNICODESTR_ASSIGN}


{$ifndef FPC_HAS_UNICODESTR_CONCAT}
{$define FPC_HAS_UNICODESTR_CONCAT}
procedure fpc_UnicodeStr_Concat (var DestS:Unicodestring;const S1,S2 : UnicodeString); compilerproc;
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
        Move(Pointer(DestS)^,(Pointer(DestS)+Location*sizeof(UnicodeChar))^,(Size)*sizeof(UnicodeChar))
      else
        Move(Pointer(S2)^,(Pointer(DestS)+Location*sizeof(UnicodeChar))^,(Size+1)*sizeof(UnicodeChar));
    end
  else if Pointer(DestS)=Pointer(S2) then
    begin
      SetLength(DestS,Size+Location);
      Move(Pointer(DestS)^,(Pointer(DestS)+Location*sizeof(UnicodeChar))^,(Size+1)*sizeof(UnicodeChar));
      Move(Pointer(S1)^,Pointer(DestS)^,Location*sizeof(UnicodeChar));
    end
  else
    begin
      DestS:='';
      SetLength(DestS,Size+Location);
      Move(Pointer(S1)^,Pointer(DestS)^,Location*sizeof(UnicodeChar));
      Move(Pointer(S2)^,(Pointer(DestS)+Location*sizeof(UnicodeChar))^,(Size+1)*sizeof(UnicodeChar));
    end;
end;
{$endif FPC_HAS_UNICODESTR_CONCAT}


{$ifndef FPC_HAS_UNICODESTR_CONCAT_MULTI}
{$define FPC_HAS_UNICODESTR_CONCAT_MULTI}
procedure fpc_UnicodeStr_Concat_multi (var DestS:Unicodestring;const sarr:array of Unicodestring); compilerproc;
Var
  i           : Longint;
  p,pc        : pointer;
  Size,NewLen : SizeInt;
  lowstart    : longint;
  destcopy    : pointer;
  OldDestLen  : SizeInt;
begin
  if high(sarr)=0 then
    begin
      DestS:='';
      exit;
    end;
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
            this optimization is disabled for WINLIKEUNICODESTRING }
          destcopy:=pointer(dests);
          fpc_UnicodeStr_Incr_Ref(destcopy);
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
  pc:=Pointer(DestS)+OldDestLen*sizeof(UnicodeChar);
  for i:=lowstart to high(sarr) do
    begin
      p:=pointer(sarr[i]);
      if assigned(p) then
        begin
          Size:=length(unicodestring(p));
          Move(p^,pc^,(Size+1)*sizeof(UnicodeChar));
          inc(pc,size*sizeof(UnicodeChar));
        end;
    end;
  fpc_UnicodeStr_Decr_Ref(destcopy);
end;
{$endif FPC_HAS_UNICODESTR_CONCAT_MULTI}


{$ifndef FPC_HAS_CHAR_TO_UCHAR}
{$define FPC_HAS_CHAR_TO_UCHAR}
Function fpc_Char_To_UChar(const c : Char): UnicodeChar; compilerproc;
var
  w: unicodestring;
begin
  widestringmanager.Ansi2UnicodeMoveProc(@c,DefaultSystemCodePage,w,1);
  fpc_Char_To_UChar:=w[1];
end;
{$endif FPC_HAS_CHAR_TO_UCHAR}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPC_HAS_CHAR_TO_UNICODESTR}
{$define FPC_HAS_CHAR_TO_UNICODESTR}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Function fpc_Char_To_UnicodeStr(const c : Char): UnicodeString; compilerproc;
{
  Converts a Char to a UnicodeString;
}
begin
  widestringmanager.Ansi2UnicodeMoveProc(@c,DefaultSystemCodePage,result,1);
end;
{$endif FPC_HAS_CHAR_TO_UNICODESTR}


{$ifndef FPC_HAS_UCHAR_TO_CHAR}
{$define FPC_HAS_UCHAR_TO_CHAR}
Function fpc_UChar_To_Char(const c : UnicodeChar): Char; compilerproc;
{
  Converts a UnicodeChar to a Char;
}
var
  s: ansistring;
begin
  widestringmanager.Unicode2AnsiMoveProc(@c, s, DefaultSystemCodePage, 1);
  if length(s)=1 then
    fpc_UChar_To_Char:= s[1]
  else
    fpc_UChar_To_Char:='?';
end;
{$endif FPC_HAS_UCHAR_TO_CHAR}


<<<<<<< HEAD
{$ifndef FPC_HAS_UCHAR_TO_SHORTSTR}
{$define FPC_HAS_UCHAR_TO_SHORTSTR}
{$ifdef VER2_6}
procedure fpc_UChar_To_ShortStr(out result : shortstring;const c : WideChar); compilerproc;
{$else}
function fpc_UChar_To_ShortStr(const c : WideChar): shortstring; compilerproc;
{$endif}
=======
Function fpc_WChar_To_UnicodeStr(const c : WideChar): UnicodeString; compilerproc;
{
  Converts a WideChar to a UnicodeString;
}
begin
  Setlength (Result,1);
  Result[1]:= c;
end;


Function fpc_Char_To_WChar(const c : Char): WideChar; compilerproc;
var
  w: widestring;
begin
  widestringmanager.Ansi2WideMoveProc(@c,DefaultSystemCodePage,w,1);
  fpc_Char_To_WChar:=w[1];
end;


Function fpc_WChar_To_Char(const c : WideChar): Char; compilerproc;
{
  Converts a WideChar to a Char;
}
var
  s: ansistring;
begin
  widestringmanager.Wide2AnsiMoveProc(@c, s, DefaultSystemCodePage, 1);
  if length(s)=1 then
    fpc_WChar_To_Char:= s[1]
  else
    fpc_WChar_To_Char:='?';
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
>>>>>>> graemeg/cpstrnew
{
  Converts a WideChar to a ShortString;
}
var
  s: ansistring;
begin
  widestringmanager.Wide2AnsiMoveProc(@c,s,DefaultSystemCodePage,1);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  result:=s;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  res:=s;
>>>>>>> graemeg/cpstrnew
end;
{$endif FPC_HAS_UCHAR_TO_SHORTSTR}


{$ifndef FPC_HAS_UCHAR_TO_UNICODESTR}
{$define FPC_HAS_UCHAR_TO_UNICODESTR}
Function fpc_UChar_To_UnicodeStr(const c : UnicodeChar): UnicodeString; compilerproc;
{
  Converts a UnicodeChar to a UnicodeString;
}
begin
  Setlength (fpc_UChar_To_UnicodeStr,1);
  fpc_UChar_To_UnicodeStr[1]:= c;
end;
<<<<<<< HEAD
{$endif FPC_HAS_UCHAR_TO_UNICODESTR}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPC_HAS_UCHAR_TO_ANSISTR}
{$define FPC_HAS_UCHAR_TO_ANSISTR}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======


>>>>>>> origin/cpstrnew
Function fpc_UChar_To_AnsiStr(const c : UnicodeChar{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}): AnsiString; compilerproc;
{
  Converts a UnicodeChar to a AnsiString;
}
{$ifndef FPC_HAS_CPSTRING}
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
var
  cp : TSystemCodePage;
{$endif FPC_HAS_CPSTRING}
begin
{$ifndef FPC_HAS_CPSTRING}
  cp:=$ffff;
{$endif FPC_HAS_CPSTRING}     
  if cp=$ffff then
    cp:=DefaultSystemCodePage;
  widestringmanager.Unicode2AnsiMoveProc(@c, fpc_UChar_To_AnsiStr, cp, 1);
end;


{$ifndef FPC_STRTOSHORTSTRINGPROC}
Function fpc_UChar_To_ShortStr(const c : UnicodeChar): ShortString; compilerproc;
{
  Converts a UnicodeChar to a ShortString;
}
>>>>>>> graemeg/cpstrnew
var
  cp : TSystemCodePage;
{$endif FPC_HAS_CPSTRING}
begin
{$ifndef FPC_HAS_CPSTRING}
  cp:=DefaultSystemCodePage;
{$endif FPC_HAS_CPSTRING}     
  cp:=TranslatePlaceholderCP(cp);
  widestringmanager.Unicode2AnsiMoveProc(@c, fpc_UChar_To_AnsiStr, cp, 1);
end;
<<<<<<< HEAD
{$endif FPC_HAS_UCHAR_TO_ANSISTR}
=======
{$else FPC_STRTOSHORTSTRINGPROC}
procedure fpc_UChar_To_ShortStr(out res : shortstring;const c : UnicodeChar) compilerproc;
{
  Converts a UnicodeChar to a ShortString;
}
var
  s: ansistring;
begin
  widestringmanager.Unicode2AnsiMoveProc(@c,s,DefaultSystemCodePage,1);
  res:=s;
end;
{$endif FPC_STRTOSHORTSTRINGPROC}
>>>>>>> graemeg/cpstrnew


{$ifndef FPC_HAS_PCHAR_TO_UNICODESTR}
{$define FPC_HAS_PCHAR_TO_UNICODESTR}
Function fpc_PChar_To_UnicodeStr(const p : pchar): UnicodeString; compilerproc;
Var
  L : SizeInt;
begin
  if (not assigned(p)) or (p[0]=#0) Then
  begin
    fpc_pchar_to_unicodestr := '';
    exit;
  end;
  l:=IndexChar(p^,-1,#0);
  widestringmanager.Ansi2UnicodeMoveProc(P,DefaultSystemCodePage,fpc_PChar_To_UnicodeStr,l);
end;
{$endif FPC_HAS_PCHAR_TO_UNICODESTR}


{$ifndef FPC_HAS_CHARARRAY_TO_UNICODESTR}
{$define FPC_HAS_CHARARRAY_TO_UNICODESTR}
Function fpc_CharArray_To_UnicodeStr(const arr: array of char; zerobased: boolean = true): UnicodeString; compilerproc;
var
  i  : SizeInt;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if zerobased then
    begin
      if arr[0]=#0 Then
        begin
          fpc_chararray_to_unicodestr:='';
          exit;
        end;
      i:=IndexChar(arr,high(arr)+1,#0);
      if i=-1 then
        i:=high(arr)+1;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  if zerobased then
    begin
      if arr[0]=#0 Then
        begin
          fpc_chararray_to_unicodestr:='';
          exit;
        end;
      i:=IndexChar(arr,high(arr)+1,#0);
      if i=-1 then
        i:=high(arr)+1;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    end
  else
    i:=high(arr)+1;
  SetLength(fpc_CharArray_To_UnicodeStr,i);
  widestringmanager.Ansi2UnicodeMoveProc(pchar(@arr),DefaultSystemCodePage,fpc_CharArray_To_UnicodeStr,i);
end;


{$ifndef FPC_STRTOSHORTSTRINGPROC}
function fpc_UnicodeCharArray_To_ShortStr(const arr: array of unicodechar; zerobased: boolean = true): shortstring;[public,alias:'FPC_UNICODECHARARRAY_TO_SHORTSTR']; compilerproc;
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
  widestringmanager.Unicode2AnsiMoveProc (punicodechar(@arr),temp,len);
  fpc_UnicodeCharArray_To_ShortStr := temp;
end;
{$else FPC_STRTOSHORTSTRINGPROC}
procedure fpc_UnicodeCharArray_To_ShortStr(out res : shortstring;const arr: array of unicodechar; zerobased: boolean = true);[public,alias:'FPC_UNICODECHARARRAY_TO_SHORTSTR']; compilerproc;
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
  widestringmanager.Unicode2AnsiMoveProc (punicodechar(@arr),temp,DefaultSystemCodePage,len);
  res:=temp;
end;
{$endif FPC_STRTOSHORTSTRINGPROC}

Function fpc_UnicodeCharArray_To_AnsiStr(const arr: array of unicodechar; {$ifdef FPC_HAS_CPSTRING}cp : TSystemCodePage;{$endif FPC_HAS_CPSTRING}zerobased: boolean = true): AnsiString; compilerproc;
var
  i  : SizeInt;
{$ifndef FPC_HAS_CPSTRING}
  cp : TSystemCodePage;
{$endif FPC_HAS_CPSTRING}
begin
{$ifndef FPC_HAS_CPSTRING}
  cp:=$ffff;
{$endif FPC_HAS_CPSTRING}
  if (zerobased) then
    begin
      i:=IndexWord(arr,high(arr)+1,0);
      if i = -1 then
        i := high(arr)+1;
    end
  else
    i := high(arr)+1;
  SetLength(fpc_UnicodeCharArray_To_AnsiStr,i);
  widestringmanager.Unicode2AnsiMoveProc (punicodechar(@arr),fpc_UnicodeCharArray_To_AnsiStr,cp,i);
end;


Function fpc_UnicodeCharArray_To_UnicodeStr(const arr: array of unicodechar; zerobased: boolean = true): UnicodeString; compilerproc;
var
  i  : SizeInt;
begin
  if (zerobased) then
    begin
      i:=IndexWord(arr,high(arr)+1,0);
      if i = -1 then
        i := high(arr)+1;
>>>>>>> graemeg/cpstrnew
    end
  else
    i:=high(arr)+1;
  widestringmanager.Ansi2UnicodeMoveProc(pchar(@arr),DefaultSystemCodePage,fpc_CharArray_To_UnicodeStr,i);
end;
{$endif FPC_HAS_CHARARRAY_TO_UNICODESTR}


{$ifndef FPC_HAS_WIDECHARARRAY_TO_UNICODESTR}
{$define FPC_HAS_WIDECHARARRAY_TO_UNICODESTR}
Function fpc_WideCharArray_To_UnicodeStr(const arr: array of widechar; zerobased: boolean = true): UnicodeString; compilerproc;
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
  SetLength(fpc_WideCharArray_To_UnicodeStr,i);
  Move(arr[0], Pointer(fpc_WideCharArray_To_UnicodeStr)^,i*sizeof(WideChar));
end;
{$endif FPC_HAS_WIDECHARARRAY_TO_UNICODESTR}


{$ifndef FPC_HAS_WIDECHARARRAY_TO_SHORTSTR}
{$define FPC_HAS_WIDECHARARRAY_TO_SHORTSTR}
{ due to their names, the following procedures should be in wstrings.inc,
  however, the compiler generates code using this functions on all platforms }
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
  widestringmanager.Wide2AnsiMoveProc (pwidechar(@arr),temp,DefaultSystemCodePage,len);
  res:=temp;
end;
<<<<<<< HEAD
{$endif FPC_HAS_WIDECHARARRAY_TO_SHORTSTR}

<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
{$ifndef FPC_HAS_WIDECHARARRAY_TO_ANSISTR}
{$define FPC_HAS_WIDECHARARRAY_TO_ANSISTR}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
{$endif FPC_STRTOSHORTSTRINGPROC}

>>>>>>> origin/cpstrnew
Function fpc_WideCharArray_To_AnsiStr(const arr: array of widechar; {$ifdef FPC_HAS_CPSTRING}cp : TSystemCodePage;{$endif FPC_HAS_CPSTRING} zerobased: boolean = true): AnsiString; compilerproc;
var
  i  : SizeInt;
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
  if (zerobased) then
    begin
      i:=IndexWord(arr,high(arr)+1,0);
      if i = -1 then
        i := high(arr)+1;
    end
  else
    i := high(arr)+1;
<<<<<<< HEAD
  widestringmanager.Wide2AnsiMoveProc (pwidechar(@arr),RawByteString(fpc_WideCharArray_To_AnsiStr),cp,i);
=======
  SetLength(fpc_WideCharArray_To_AnsiStr,i);
  widestringmanager.Wide2AnsiMoveProc (pwidechar(@arr),fpc_WideCharArray_To_AnsiStr,cp,i);
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
{$endif FPC_HAS_WIDECHARARRAY_TO_ANSISTR}


{$ifndef FPC_HAS_WIDECHARARRAY_TO_WIDESTR}
{$define FPC_HAS_WIDECHARARRAY_TO_WIDESTR}
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
{$endif FPC_HAS_WIDECHARARRAY_TO_WIDESTR}


{$ifndef FPC_HAS_UNICODESTR_TO_CHARARRAY}
{$define FPC_HAS_UNICODESTR_TO_CHARARRAY}
procedure fpc_unicodestr_to_chararray(out res: array of char; const src: UnicodeString); compilerproc;
var
  len: SizeInt;
  temp: ansistring;
begin
  len := length(src);
  { make sure we don't dereference src if it can be nil (JM) }
  if len > 0 then
    widestringmanager.unicode2ansimoveproc(punicodechar(@src[1]),temp,DefaultSystemCodePage,len);
  len := length(temp);
  if len > length(res) then
    len := length(res);
{$push}
{$r-}
  move(temp[1],res[0],len);
  fillchar(res[len],length(res)-len,0);
{$pop}
end;
{$endif FPC_HAS_UNICODESTR_TO_UNICODECHARARRAY}


<<<<<<< HEAD
{$ifndef FPC_HAS_ANSISTR_TO_WIDECHARARRAY}
{$define FPC_HAS_ANSISTR_TO_WIDECHARARRAY}
procedure fpc_ansistr_to_widechararray(out res: array of widechar; const src: RawByteString); compilerproc;
=======
procedure fpc_unicodestr_to_unicodechararray(out res: array of unicodechar; const src: UnicodeString); compilerproc;
var
  len: SizeInt;
begin
  len := length(src);
  if len > length(res) then
    len := length(res);
{$r-}
  { make sure we don't try to access element 1 of the ansistring if it's nil }
  if len > 0 then
    move(src[1],res[0],len*SizeOf(UnicodeChar));
  fillchar(res[len],(length(res)-len)*SizeOf(UnicodeChar),0);
{$ifdef RangeCheckWasOn}
{$r+}
{$endif}
end;


procedure fpc_ansistr_to_unicodechararray(out res: array of unicodechar; const src: AnsiString); compilerproc;
var
  len: SizeInt;
  temp: unicodestring;
begin
  len := length(src);
  { make sure we don't dereference src if it can be nil (JM) }
  if len > 0 then
    widestringmanager.ansi2unicodemoveproc(pchar(@src[1]),DefaultSystemCodePage,temp,len);
  len := length(temp);
  if len > length(res) then
    len := length(res);

{$r-}
  move(temp[1],res[0],len*sizeof(unicodechar));
  fillchar(res[len],(length(res)-len)*SizeOf(UnicodeChar),0);
{$ifdef RangeCheckWasOn}
{$r+}
{$endif}
end;

procedure fpc_shortstr_to_unicodechararray(out res: array of unicodechar; const src: ShortString); compilerproc;
var
  len: longint;
  temp : unicodestring;
begin
  len := length(src);
  { make sure we don't access char 1 if length is 0 (JM) }
  if len > 0 then
    widestringmanager.ansi2unicodemoveproc(pchar(@src[1]),DefaultSystemCodePage,temp,len);
  len := length(temp);
  if len > length(res) then
    len := length(res);
{$r-}
  move(temp[1],res[0],len*sizeof(unicodechar));
  fillchar(res[len],(length(res)-len)*SizeOf(UnicodeChar),0);
{$ifdef RangeCheckWasOn}
{$r+}
{$endif}
end;

procedure fpc_ansistr_to_widechararray(out res: array of widechar; const src: AnsiString); compilerproc;
>>>>>>> graemeg/cpstrnew
var
  len: SizeInt;
  temp: widestring;
begin
  len := length(src);
  { make sure we don't dereference src if it can be nil (JM) }
  if len > 0 then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    widestringmanager.ansi2widemoveproc(pchar(@src[1]),TranslatePlaceholderCP(StringCodePage(src)),temp,len);
=======
    widestringmanager.ansi2widemoveproc(pchar(@src[1]),StringCodePage(src),temp,len);
>>>>>>> graemeg/cpstrnew
=======
    widestringmanager.ansi2widemoveproc(pchar(@src[1]),StringCodePage(src),temp,len);
>>>>>>> graemeg/cpstrnew
=======
    widestringmanager.ansi2widemoveproc(pchar(@src[1]),StringCodePage(src),temp,len);
>>>>>>> graemeg/cpstrnew
=======
    widestringmanager.unicode2ansimoveproc(punicodechar(@src[1]),temp,DefaultSystemCodePage,len);
>>>>>>> origin/cpstrnew
  len := length(temp);
  if len > length(res) then
    len := length(res);
{$push}
{$r-}
  move(temp[1],res[0],len*sizeof(widechar));
  fillchar(res[len],(length(res)-len)*SizeOf(WideChar),0);
{$pop}
end;
{$endif FPC_HAS_ANSISTR_TO_WIDECHARARRAY}


{$ifndef FPC_HAS_SHORTSTR_TO_WIDECHARARRAY}
{$define FPC_HAS_SHORTSTR_TO_WIDECHARARRAY}
procedure fpc_shortstr_to_widechararray(out res: array of widechar; const src: ShortString); compilerproc;
var
  len: longint;
  temp : widestring;
begin
  len := length(src);
  { make sure we don't access char 1 if length is 0 (JM) }
  if len > 0 then
    widestringmanager.ansi2widemoveproc(pchar(@src[1]),DefaultSystemCodePage,temp,len);
  len := length(temp);
  if len > length(res) then
    len := length(res);
{$push}
{$r-}
<<<<<<< HEAD
  move(temp[1],res[0],len*sizeof(widechar));
  fillchar(res[len],(length(res)-len)*SizeOf(WideChar),0);
{$pop}
=======
  { make sure we don't try to access element 1 of the ansistring if it's nil }
  if len > 0 then
    move(src[1],res[0],len*SizeOf(UnicodeChar));
  fillchar(res[len],(length(res)-len)*SizeOf(UnicodeChar),0);
{$ifdef RangeCheckWasOn}
{$r+}
{$endif}
end;


procedure fpc_ansistr_to_unicodechararray(out res: array of unicodechar; const src: AnsiString); compilerproc;
var
  len: SizeInt;
  temp: unicodestring;
begin
  len := length(src);
  { make sure we don't dereference src if it can be nil (JM) }
  if len > 0 then
    widestringmanager.ansi2unicodemoveproc(pchar(@src[1]),DefaultSystemCodePage,temp,len);
  len := length(temp);
  if len > length(res) then
    len := length(res);

{$r-}
  move(temp[1],res[0],len*sizeof(unicodechar));
  fillchar(res[len],(length(res)-len)*SizeOf(UnicodeChar),0);
{$ifdef RangeCheckWasOn}
{$r+}
{$endif}
end;

procedure fpc_shortstr_to_unicodechararray(out res: array of unicodechar; const src: ShortString); compilerproc;
var
  len: longint;
  temp : unicodestring;
begin
  len := length(src);
  { make sure we don't access char 1 if length is 0 (JM) }
  if len > 0 then
    widestringmanager.ansi2unicodemoveproc(pchar(@src[1]),DefaultSystemCodePage,temp,len);
  len := length(temp);
  if len > length(res) then
    len := length(res);
{$r-}
  move(temp[1],res[0],len*sizeof(unicodechar));
  fillchar(res[len],(length(res)-len)*SizeOf(UnicodeChar),0);
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
    widestringmanager.ansi2widemoveproc(pchar(@src[1]),StringCodePage(src),temp,len);
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
    widestringmanager.ansi2widemoveproc(pchar(@src[1]),DefaultSystemCodePage,temp,len);
  len := length(temp);
  if len > length(res) then
    len := length(res);
{$r-}
  move(temp[1],res[0],len*sizeof(widechar));
  fillchar(res[len],(length(res)-len)*SizeOf(WideChar),0);
{$ifdef RangeCheckWasOn}
{$r+}
{$endif}
>>>>>>> origin/cpstrnew
end;
{$endif FPC_HAS_SHORTSTR_TO_WIDECHARARRAY}


{$ifndef FPC_HAS_UNICODESTR_TO_WIDECHARARRAY}
{$define FPC_HAS_UNICODESTR_TO_WIDECHARARRAY}
procedure fpc_unicodestr_to_widechararray(out res: array of widechar; const src: UnicodeString); compilerproc;
var
  len: SizeInt;
begin
  len := length(src);
  if len > length(res) then
    len := length(res);
{$push}
{$r-}
  { make sure we don't try to access element 1 of the widestring if it's nil }
  if len > 0 then
    move(src[1],res[0],len*SizeOf(WideChar));
  fillchar(res[len],(length(res)-len)*SizeOf(WideChar),0);
{$pop}
end;
{$endif FPC_HAS_UNICODESTR_TO_WIDECHARARRAY}



{$ifndef FPC_HAS_UNICODESTR_COMPARE}
{$define FPC_HAS_UNICODESTR_COMPARE}
Function fpc_UnicodeStr_Compare(const S1,S2 : UnicodeString): SizeInt;[Public,Alias : 'FPC_UNICODESTR_COMPARE']; compilerproc;
{
  Compares 2 UnicodeStrings;
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
     fpc_UnicodeStr_Compare:=0;
     exit;
   end;
  Maxi:=Length(S1);
  temp:=Length(S2);
  If MaxI>Temp then
   MaxI:=Temp;
  Temp:=CompareWord(S1[1],S2[1],MaxI);
  if temp=0 then
   temp:=Length(S1)-Length(S2);
  fpc_UnicodeStr_Compare:=Temp;
end;
{$endif FPC_HAS_UNICODESTR_COMPARE}


{$ifndef FPC_HAS_UNICODESTR_COMPARE_EQUAL}
{$define FPC_HAS_UNICODESTR_COMPARE_EQUAL}
Function fpc_UnicodeStr_Compare_Equal(const S1,S2 : UnicodeString): SizeInt;[Public,Alias : 'FPC_UNICODESTR_COMPARE_EQUAL']; compilerproc;
{
  Compares 2 UnicodeStrings for equality only;
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
{$endif FPC_HAS_UNICODESTR_COMPARE_EQUAL}

<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
{$ifndef FPC_HAS_UNICODESTR_RANGECHECK}
{$define FPC_HAS_UNICODESTR_RANGECHECK}
Procedure fpc_UnicodeStr_RangeCheck(p: Pointer; index: SizeInt);[Public,Alias : 'FPC_UNICODESTR_RANGECHECK']; compilerproc;
begin
  if (p=nil) or (index>PUnicodeRec(p-UnicodeFirstOff)^.len) or (Index<1) then
    HandleErrorAddrFrameInd(201,get_pc_addr,get_frame);
=======


=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
Procedure fpc_UnicodeStr_CheckRange(p: Pointer; index: SizeInt);[Public,Alias : 'FPC_UNICODESTR_RANGECHECK']; compilerproc;
begin
  if (p=nil) or (index>PUnicodeRec(p-UnicodeFirstOff)^.len div 2) or (Index<1) then
    HandleErrorFrame(201,get_frame);
>>>>>>> graemeg/cpstrnew
end;
{$endif FPC_HAS_UNICODESTR_RANGECHECK}


{$ifndef FPC_HAS_UNICODESTR_SETLENGTH}
{$define FPC_HAS_UNICODESTR_SETLENGTH}
Procedure fpc_UnicodeStr_SetLength(Var S : UnicodeString; l : SizeInt);[Public,Alias : 'FPC_UNICODESTR_SETLENGTH']; compilerproc;
{
  Sets The length of string S to L.
  Makes sure S is unique, and contains enough room.
}
Var
  Temp : Pointer;
  movelen: SizeInt;
  nl,lens, lena : SizeUInt;
begin
  nl:=l;
{$IFDEF VER2_6}
  nl:=nl*2;
{$ENDIF}
   if (l>0) then
    begin
      if Pointer(S)=nil then
        begin
          { Need a complete new string...}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          Pointer(s):=NewUnicodeString(nl);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          Pointer(s):=NewUnicodeString(l);
        end
      else
        if (PUnicodeRec(Pointer(S)-UnicodeFirstOff)^.Ref = 1) then
          begin
            Dec(Pointer(S),UnicodeFirstOff);
            if SizeUInt(L*sizeof(UnicodeChar)+UnicodeRecLen)>MemSize(Pointer(S)) then
              reallocmem(pointer(S), L*sizeof(UnicodeChar)+UnicodeRecLen);
            Inc(Pointer(S), UnicodeFirstOff);
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
        end
      else
        if (PUnicodeRec(Pointer(S)-UnicodeFirstOff)^.Ref = 1) then
          begin
            Temp:=Pointer(s)-UnicodeFirstOff;
            lens:=MemSize(Temp);
            lena:=SizeUInt(L*sizeof(UnicodeChar)+(UnicodeFirstOff+sizeof(UnicodeChar)));
            if (lena>lens) or ((lens>32) and (lena<=(lens div 2))) then
              begin
                reallocmem(Temp, lena);
                Pointer(S):=Temp+UnicodeFirstOff;
              end;
          end
      else
        begin
          { Reallocation is needed... }
          Temp:=NewUnicodeString(nL);
          if Length(S)>0 then
            begin
              if l < succ(length(s)) then
                movelen := l
              { also move terminating null }
              else
                movelen := succ(length(s));
              Move(Pointer(S)^,Temp^,movelen * Sizeof(UnicodeChar));
            end;
          fpc_unicodestr_decr_ref(Pointer(S));
          Pointer(S):=Temp;
        end;
      { Force nil termination in case it gets shorter }
      PWord(Pointer(S)+l*sizeof(UnicodeChar))^:=0;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      PUnicodeRec(Pointer(S)-UnicodeFirstOff)^.Len:=nl;
=======
      PUnicodeRec(Pointer(S)-UnicodeFirstOff)^.Len:=l;
>>>>>>> graemeg/cpstrnew
=======
      PUnicodeRec(Pointer(S)-UnicodeFirstOff)^.Len:=l;
>>>>>>> graemeg/cpstrnew
=======
      PUnicodeRec(Pointer(S)-UnicodeFirstOff)^.Len:=l;
>>>>>>> graemeg/cpstrnew
=======
      PUnicodeRec(Pointer(S)-UnicodeFirstOff)^.Len:=l;
>>>>>>> origin/cpstrnew
    end
  else  { length=0, deallocate the string }
    fpc_unicodestr_decr_ref (Pointer(S));
end;
{$endif FPC_HAS_UNICODESTR_SETLENGTH}

{*****************************************************************************
                     Public functions, In interface.
*****************************************************************************}

function UnicodeCharToString(S : PUnicodeChar) : UnicodeString;
  begin
     result:=UnicodeCharLenToString(s,Length(UnicodeString(s)));
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

function StringToUnicodeChar(const Src : RawByteString;Dest : PUnicodeChar;DestSize : SizeInt) : PUnicodeChar;
  var
    temp:unicodestring;
  begin
     widestringmanager.Ansi2UnicodeMoveProc(PChar(Src),StringCodePage(Src),temp,Length(Src));
     if Length(temp)<DestSize then
       move(temp[1],Dest^,Length(temp)*SizeOf(UnicodeChar))
     else
       move(temp[1],Dest^,(DestSize-1)*SizeOf(UnicodeChar));

     Dest[DestSize-1]:=#0;

     result:=Dest;
>>>>>>> graemeg/cpstrnew

{$ifndef FPC_HAS_STRING_TO_UNICODECHAR}
{$define FPC_HAS_STRING_TO_UNICODECHAR}
function StringToUnicodeChar(const Src : RawByteString;Dest : PUnicodeChar;DestSize : SizeInt) : PUnicodeChar;
  begin
    result:=StringToWideChar(Src,Dest,DestSize);
  end;
{$endif FPC_HAS_STRING_TO_UNICODECHAR}


function WideCharToString(S : PWideChar) : UnicodeString;
  begin
     result:=WideCharLenToString(s,Length(WideString(s)));
  end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPC_HAS_STRING_LEN_TO_WIDECHAR}
{$define FPC_HAS_STRING_LEN_TO_WIDECHAR}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function StringToWideChar(const Src : RawByteString;Dest : PWideChar;DestSize : SizeInt) : PWideChar;
  var
    temp: widestring;
    Len: SizeInt;
  begin
     widestringmanager.Ansi2WideMoveProc(PChar(Src),StringCodePage(Src),temp,Length(Src));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Len:=Length(temp);
     if DestSize<=Len then
       Len:=Destsize-1;
     move(temp[1],Dest^,Len*SizeOf(WideChar));
     Dest[Len]:=#0;
     result:=Dest;
  end;
{$endif FPC_HAS_STRING_LEN_TO_WIDECHAR}


{$ifndef FPC_HAS_UNICODECHAR_LEN_TO_STRING}
{$define FPC_HAS_UNICODECHAR_LEN_TO_STRING}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     if Length(temp)<DestSize then
       move(temp[1],Dest^,Length(temp)*SizeOf(WideChar))
     else
       move(temp[1],Dest^,(DestSize-1)*SizeOf(WideChar));

     Dest[DestSize-1]:=#0;

     result:=Dest;
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
function UnicodeCharLenToString(S : PUnicodeChar;Len : SizeInt) : UnicodeString;
  begin
    SetLength(result,Len);
    Move(S^,Pointer(Result)^,Len*2);
  end;
{$endif FPC_HAS_UNICODECHAR_LEN_TO_STRING}


procedure UnicodeCharLenToStrVar(Src : PUnicodeChar;Len : SizeInt;out Dest : UnicodeString);
  begin
    Dest:=UnicodeCharLenToString(Src,Len);
  end;


procedure UnicodeCharLenToStrVar(Src : PUnicodeChar;Len : SizeInt;out Dest : AnsiString);
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Dest:=AnsiString(UnicodeCharLenToString(Src,Len));
=======
    Dest:=UnicodeCharLenToString(Src,Len);
>>>>>>> graemeg/cpstrnew
=======
    Dest:=UnicodeCharLenToString(Src,Len);
>>>>>>> graemeg/cpstrnew
=======
    Dest:=UnicodeCharLenToString(Src,Len);
>>>>>>> graemeg/cpstrnew
=======
    Dest:=UnicodeCharLenToString(Src,Len);
>>>>>>> origin/cpstrnew
  end;


procedure UnicodeCharToStrVar(S : PUnicodeChar;out Dest : AnsiString);
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Dest:=AnsiString(UnicodeCharToString(S));
  end;


{$ifndef FPC_HAS_WIDECHAR_LEN_TO_STRING}
{$define FPC_HAS_WIDECHAR_LEN_TO_STRING}
=======
    Dest:=UnicodeCharToString(S);
  end;


>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
    Dest:=UnicodeCharToString(S);
  end;


<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
    Dest:=UnicodeCharToString(S);
  end;


>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function WideCharLenToString(S : PWideChar;Len : SizeInt) : UnicodeString;
  begin
    SetLength(result,Len);
    Move(S^,Pointer(Result)^,Len*2);
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endif FPC_HAS_WIDECHAR_LEN_TO_STRING}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


procedure WideCharLenToStrVar(Src : PWideChar;Len : SizeInt;out Dest : UnicodeString);
  begin
    Dest:=WideCharLenToString(Src,Len);
  end;


procedure WideCharLenToStrVar(Src : PWideChar;Len : SizeInt;out Dest : AnsiString);
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Dest:=AnsiString(WideCharLenToString(Src,Len));
=======
    Dest:=WideCharLenToString(Src,Len);
>>>>>>> graemeg/cpstrnew
=======
    Dest:=WideCharLenToString(Src,Len);
>>>>>>> graemeg/cpstrnew
=======
    Dest:=WideCharLenToString(Src,Len);
>>>>>>> graemeg/cpstrnew
=======
    Dest:=WideCharLenToString(Src,Len);
>>>>>>> origin/cpstrnew
  end;


procedure WideCharToStrVar(S : PWideChar;out Dest : UnicodeString);
  begin
    Dest:=WideCharToString(S);
  end;


procedure WideCharToStrVar(S : PWideChar;out Dest : AnsiString);
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Dest:=AnsiString(WideCharToString(S));
=======
    Dest:=WideCharToString(S);
>>>>>>> graemeg/cpstrnew
=======
    Dest:=WideCharToString(S);
>>>>>>> graemeg/cpstrnew
=======
    Dest:=WideCharToString(S);
>>>>>>> graemeg/cpstrnew
=======
    Dest:=WideCharToString(S);
>>>>>>> origin/cpstrnew
  end;


{$ifndef FPC_HAS_UNICODESTR_UNIQUE}
{$define FPC_HAS_UNICODESTR_UNIQUE}
Function fpc_unicodestr_Unique(Var S : Pointer): Pointer; [Public,Alias : 'FPC_UNICODESTR_UNIQUE']; compilerproc;
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
  if PUnicodeRec(Pointer(S)-UnicodeFirstOff)^.Ref<>1 then
   begin
     L:=PUnicodeRec(Pointer(S)-UnicodeFirstOff)^.len;
     SNew:=NewUnicodeString (L);
     Move (PUnicodeChar(S)^,SNew^,(L+1)*sizeof(UnicodeChar));
     PUnicodeRec(SNew-UnicodeFirstOff)^.len:=L;
     fpc_unicodestr_decr_ref (Pointer(S));  { Thread safe }
     pointer(S):=SNew;
     pointer(result):=SNew;
   end;
end;
{$endif FPC_HAS_UNICODESTR_UNIQUE}


{$ifndef FPC_HAS_UNICODESTR_COPY}
{$define FPC_HAS_UNICODESTR_COPY}
Function Fpc_UnicodeStr_Copy (Const S : UnicodeString; Index,Size : SizeInt) : UnicodeString;compilerproc;
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
<<<<<<< HEAD
     ResultAddress:=NewUnicodeString(Size);
     Move (PUnicodeChar(S)[Index],ResultAddress^,Size*sizeof(UnicodeChar));
     PUnicodeRec(ResultAddress-UnicodeFirstOff)^.Len:=Size;
     PUnicodeChar(ResultAddress+Size*sizeof(UnicodeChar))^:=#0;
=======
     If Index<0 Then
      Index:=0;
     ResultAddress:=Pointer(NewUnicodeString (Size));
     if ResultAddress<>Nil then
      begin
        Move (PUnicodeChar(S)[Index],ResultAddress^,Size*sizeof(UnicodeChar));
        PUnicodeRec(ResultAddress-UnicodeFirstOff)^.Len:=Size;
        PUnicodeChar(ResultAddress+Size*sizeof(UnicodeChar))^:=#0;
      end;
>>>>>>> graemeg/cpstrnew
   end;
  fpc_unicodestr_decr_ref(Pointer(fpc_unicodestr_copy));
  Pointer(fpc_unicodestr_Copy):=ResultAddress;
end;
{$endif FPC_HAS_UNICODESTR_COPY}


{$ifndef FPC_HAS_POS_UNICODESTR_UNICODESTR}
{$define FPC_HAS_POS_UNICODESTR_UNICODESTR}
Function Pos (Const Substr : UnicodeString; Const Source : UnicodeString) : SizeInt;
var
  i,MaxLen : SizeInt;
  pc : punicodechar;
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
{$endif FPC_HAS_POS_UNICODESTR_UNICODESTR}


{$ifndef FPC_HAS_POS_UNICODECHAR_UNICODESTR}
{$define FPC_HAS_POS_UNICODECHAR_UNICODESTR}
{ Faster version for a unicodechar alone }
Function Pos (c : UnicodeChar; Const s : UnicodeString) : SizeInt;
var
  i: SizeInt;
  pc : punicodechar;
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
<<<<<<< HEAD
{$endif FPC_HAS_POS_UNICODECHAR_UNICODESTR}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ DO NOT inline these! Inlining a managed typecast creates an implicit try..finally
  block, which is significant bloat without any sensible speed improvement. }
Function Pos (const c : RawByteString; Const s : UnicodeString) : SizeInt;
=======
Function Pos (c : RawByteString; Const s : UnicodeString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======
Function Pos (c : RawByteString; Const s : UnicodeString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======
Function Pos (c : RawByteString; Const s : UnicodeString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======


Function Pos (c : RawByteString; Const s : UnicodeString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> origin/cpstrnew
  begin
    result:=Pos(UnicodeString(c),s);
  end;


Function Pos (const c : ShortString; Const s : UnicodeString) : SizeInt;
  begin
    result:=Pos(UnicodeString(c),s);
  end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function Pos (const c : UnicodeString; Const s : RawByteString) : SizeInt;
=======
Function Pos (c : UnicodeString; Const s : RawByteString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======
Function Pos (c : UnicodeString; Const s : RawByteString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======
Function Pos (c : UnicodeString; Const s : RawByteString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======
Function Pos (c : UnicodeString; Const s : RawByteString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> origin/cpstrnew
  begin
    result:=Pos(c,UnicodeString(s));
  end;

{$ifndef FPC_HAS_POS_CHAR_UNICODESTR}
{$define FPC_HAS_POS_CHAR_UNICODESTR}
{ Faster version for a char alone. Must be implemented because   }
{ pos(c: char; const s: shortstring) also exists, so otherwise   }
{ using pos(char,pchar) will always call the shortstring version }
{ (exact match for first argument), also with $h+ (JM)           }
Function Pos (c : Char; Const s : UnicodeString) : SizeInt;
var
  i: SizeInt;
  wc : unicodechar;
  pc : punicodechar;
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
{$endif FPC_HAS_POS_CHAR_UNICODESTR}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPC_HAS_DELETE_UNICODESTR}
{$define FPC_HAS_DELETE_UNICODESTR}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Procedure Delete (Var S : UnicodeString; Index,Size: SizeInt);
Var
  LS : SizeInt;
begin
  LS:=Length(S);
  if (Index>LS) or (Index<=0) or (Size<=0) then
    exit;

  UniqueString (S);
  { (Size+Index) will overflow if Size=MaxInt. }
  if Size>LS-Index then
    Size:=LS-Index+1;
  if Size<=LS-Index then
  begin
    Dec(Index);
    Move(PUnicodeChar(S)[Index+Size],PUnicodeChar(S)[Index],(LS-Index-Size+1)*sizeof(UnicodeChar));
  end;
  Setlength(s,LS-Size);
end;
{$endif FPC_HAS_DELETE_UNICODESTR}


{$ifndef FPC_HAS_INSERT_UNICODESTR}
{$define FPC_HAS_INSERT_UNICODESTR}
Procedure Insert (Const Source : UnicodeString; Var S : UnicodeString; Index : SizeInt);
var
  Temp : UnicodeString;
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
    move (PUnicodeChar(S)^,PUnicodeChar(Temp)^,Index*sizeof(UnicodeChar));
  Move (PUnicodeChar(Source)^,PUnicodeChar(Temp)[Index],Length(Source)*sizeof(UnicodeChar));
  If (LS-Index)>0 then
    Move(PUnicodeChar(S)[Index],PUnicodeChar(temp)[Length(Source)+index],(LS-Index)*sizeof(UnicodeChar));
  S:=Temp;
end;
{$endif FPC_HAS_INSERT_UNICODESTR}


{$ifndef FPC_HAS_UPCASE_UNICODECHAR}
{$define FPC_HAS_UPCASE_UNICODECHAR}
Function  UpCase(c:UnicodeChar):UnicodeChar;
var
  s : UnicodeString;
begin
  s:=c;
  result:=widestringmanager.UpperUnicodeStringProc(s)[1];
end;
{$endif FPC_HAS_UPCASE_UNICODECHAR}


{$ifndef FPC_HAS_UPCASE_UNICODESTR}
{$define FPC_HAS_UPCASE_UNICODESTR}
function UpCase(const s : UnicodeString) : UnicodeString;
begin
  result:=widestringmanager.UpperUnicodeStringProc(s);
end;
{$endif FPC_HAS_UPCASE_UNICODESTR}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPC_HAS_LOWERCASE_UNICODECHAR}
{$define FPC_HAS_LOWERCASE_UNICODECHAR}
Function  LowerCase(c:UnicodeChar):UnicodeChar;
var
  s : UnicodeString;
begin
  s:=c;
  result:=widestringmanager.LowerUnicodeStringProc(s)[1];
end;
{$endif FPC_HAS_LOWERCASE_UNICODECHAR}


{$ifndef FPC_HAS_LOWERCASE_UNICODESTR}
{$define FPC_HAS_LOWERCASE_UNICODESTR}
function LowerCase(const s : UnicodeString) : UnicodeString;
begin
  result:=widestringmanager.LowerUnicodeStringProc(s);
end;
{$endif FPC_HAS_LOWERCASE_UNICODESTR}


{$ifndef FPC_HAS_SETSTRING_UNICODESTR_PUNICODECHAR}
{$define FPC_HAS_SETSTRING_UNICODESTR_PUNICODECHAR}
Procedure {$ifdef FPC_HAS_CPSTRING}fpc_setstring_unicodestr_pwidechar{$else}SetString{$endif}(Out S : UnicodeString; Buf : PUnicodeChar; Len : SizeInt); {$ifdef FPC_HAS_CPSTRING} compilerproc; {$endif FPC_HAS_CPSTRING}
=======
Procedure SetString (Out S : UnicodeString; Buf : PUnicodeChar; Len : SizeInt);
>>>>>>> graemeg/cpstrnew
=======
Procedure SetString (Out S : UnicodeString; Buf : PUnicodeChar; Len : SizeInt);
>>>>>>> graemeg/cpstrnew
=======
Procedure SetString (Out S : UnicodeString; Buf : PUnicodeChar; Len : SizeInt);
>>>>>>> graemeg/cpstrnew
=======
Procedure SetString (Out S : UnicodeString; Buf : PUnicodeChar; Len : SizeInt);
>>>>>>> origin/cpstrnew
begin
  SetLength(S,Len);
  If (Buf<>Nil) and (Len>0) then
    Move (Buf[0],S[1],Len*sizeof(UnicodeChar));
end;
{$endif FPC_HAS_SETSTRING_UNICODESTR_PUNICODECHAR}


{$ifndef FPC_HAS_SETSTRING_UNICODESTR_PCHAR}
{$define FPC_HAS_SETSTRING_UNICODESTR_PCHAR}
Procedure {$ifdef FPC_HAS_CPSTRING}fpc_setstring_unicodestr_pansichar{$else}SetString{$endif}(Out S : UnicodeString; Buf : PChar; Len : SizeInt); {$ifdef FPC_HAS_CPSTRING} compilerproc; {$endif FPC_HAS_CPSTRING}
begin
  If (Buf<>Nil) and (Len>0) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    widestringmanager.Ansi2UnicodeMoveProc(Buf,DefaultSystemCodePage,S,Len)
  else
    SetLength(S,Len);
=======
    widestringmanager.Ansi2UnicodeMoveProc(Buf,DefaultSystemCodePage,S,Len);
>>>>>>> graemeg/cpstrnew
=======
    widestringmanager.Ansi2UnicodeMoveProc(Buf,DefaultSystemCodePage,S,Len);
>>>>>>> graemeg/cpstrnew
=======
    widestringmanager.Ansi2UnicodeMoveProc(Buf,DefaultSystemCodePage,S,Len);
>>>>>>> graemeg/cpstrnew
=======
    widestringmanager.Ansi2UnicodeMoveProc(Buf,DefaultSystemCodePage,S,Len);
>>>>>>> origin/cpstrnew
end;
{$endif FPC_HAS_SETSTRING_UNICODESTR_PCHAR}


{$ifndef FPUNONE}
Function fpc_Val_Real_UnicodeStr(Const S : UnicodeString; out Code : ValSInt): ValReal; [public, alias:'FPC_VAL_REAL_UNICODESTR']; compilerproc;
Var
  SS: ShortString;
begin
  fpc_Val_Real_UnicodeStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
      SS:=ShortString(S);
      Val(SS,fpc_Val_Real_UnicodeStr,code);
    end;
end;
{$endif}


{$ifndef FPC_STR_ENUM_INTERN}
function fpc_val_enum_unicodestr(str2ordindex:pointer;const s:unicodestring;out code:valsint):longint;compilerproc;
var
  ss: ShortString;
begin
  if length(s)>255 then
    code:=256
  else
    begin
      ss:=ShortString(s);
      val(ss,fpc_val_enum_unicodestr,code);
    end;
end;
{$endif FPC_STR_ENUM_INTERN}


Function fpc_Val_Currency_UnicodeStr(Const S : UnicodeString; out Code : ValSInt): Currency; [public, alias:'FPC_VAL_CURRENCY_UNICODESTR']; compilerproc;
Var
  SS: ShortString;
begin
  if length(S)>255 then
    begin
      fpc_Val_Currency_UnicodeStr:=0;
      code:=256;
    end
  else
    begin
      SS:=ShortString(S);
      Val(SS,fpc_Val_Currency_UnicodeStr,code);
    end;
end;


Function fpc_Val_UInt_UnicodeStr (Const S : UnicodeString; out Code : ValSInt): ValUInt; [public, alias:'FPC_VAL_UINT_UNICODESTR']; compilerproc;
Var
  SS: ShortString;
begin
  fpc_Val_UInt_UnicodeStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
      SS:=ShortString(S);
      Val(SS,fpc_Val_UInt_UnicodeStr,code);
    end;
end;


Function fpc_Val_SInt_UnicodeStr (DestSize: SizeInt; Const S : UnicodeString; out Code : ValSInt): ValSInt; [public, alias:'FPC_VAL_SINT_UNICODESTR']; compilerproc;
Var
  SS: ShortString;
begin
  fpc_Val_SInt_UnicodeStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
      SS:=ShortString(S);
      fpc_Val_SInt_UnicodeStr := int_Val_SInt_ShortStr(DestSize,SS,Code);
    end;
end;


{$ifndef CPU64}

Function fpc_Val_qword_UnicodeStr (Const S : UnicodeString; out Code : ValSInt): qword; [public, alias:'FPC_VAL_QWORD_UNICODESTR']; compilerproc;
Var
  SS: ShortString;
begin
  fpc_Val_qword_UnicodeStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS:=ShortString(S);
       Val(SS,fpc_Val_qword_UnicodeStr,Code);
    end;
end;


Function fpc_Val_int64_UnicodeStr (Const S : UnicodeString; out Code : ValSInt): Int64; [public, alias:'FPC_VAL_INT64_UNICODESTR']; compilerproc;
Var
  SS: ShortString;
begin
  fpc_Val_int64_UnicodeStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS:=ShortString(S);
       Val(SS,fpc_Val_int64_UnicodeStr,Code);
    end;
end;

{$endif CPU64}


{$if defined(CPU16) or defined(CPU8)}
Function fpc_Val_longword_UnicodeStr (Const S : UnicodeString; out Code : ValSInt): longword; [public, alias:'FPC_VAL_LONGWORD_UNICODESTR']; compilerproc;
Var
  SS: ShortString;
begin
  fpc_Val_longword_UnicodeStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS:=ShortString(S);
       Val(SS,fpc_Val_longword_UnicodeStr,Code);
    end;
end;


Function fpc_Val_longint_UnicodeStr (Const S : UnicodeString; out Code : ValSInt): LongInt; [public, alias:'FPC_VAL_LONGINT_UNICODESTR']; compilerproc;
Var
  SS: ShortString;
begin
  fpc_Val_longint_UnicodeStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS:=ShortString(S);
       Val(SS,fpc_Val_longint_UnicodeStr,Code);
    end;
end;


Function fpc_Val_word_UnicodeStr (Const S : UnicodeString; out Code : ValSInt): word; [public, alias:'FPC_VAL_WORD_UNICODESTR']; compilerproc;
Var
  SS: ShortString;
begin
  fpc_Val_word_UnicodeStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS:=ShortString(S);
       Val(SS,fpc_Val_word_UnicodeStr,Code);
    end;
end;


Function fpc_Val_smallint_UnicodeStr (Const S : UnicodeString; out Code : ValSInt): SmallInt; [public, alias:'FPC_VAL_SMALLINT_UNICODESTR']; compilerproc;
Var
  SS: ShortString;
begin
  fpc_Val_smallint_UnicodeStr:=0;
  if length(S)>255 then
    code:=256
  else
    begin
       SS:=ShortString(S);
       Val(SS,fpc_Val_smallint_UnicodeStr,Code);
    end;
end;
{$endif CPU16 or CPU8}


{$ifndef FPUNONE}
procedure fpc_UnicodeStr_Float(d : ValReal;len,fr,rt : SizeInt;out s : UnicodeString);compilerproc;
var
  ss: shortstring;
begin
  str_real(len,fr,d,treal_type(rt),ss);
  s:=UnicodeString(ss);
end;
{$endif}


{$ifndef FPC_STR_ENUM_INTERN}
procedure fpc_unicodestr_enum(ordinal,len:sizeint;typinfo,ord2strindex:pointer;out s:unicodestring);compilerproc;
var
  ss: ShortString;
begin
  fpc_shortstr_enum(ordinal,len,typinfo,ord2strindex,ss);
  s:=UnicodeString(ss);
end;
{$endif FPC_STR_ENUM_INTERN}

procedure fpc_unicodestr_bool(b : boolean;len:sizeint;out s:unicodestring);compilerproc;
var
  ss: ShortString;
begin
  fpc_shortstr_bool(b,len,ss);
  s:=UnicodeString(ss);
end;

<<<<<<< HEAD
=======
procedure fpc_unicodestr_bool(b : boolean;len:sizeint;out s:unicodestring);compilerproc;

var ss:shortstring;

begin
  fpc_shortstr_bool(b,len,ss);
  s:=ss;
end;

procedure fpc_unicodestr_bool(b : boolean;len:sizeint;out s:unicodestring);compilerproc;

var ss:shortstring;

begin
  fpc_shortstr_bool(b,len,ss);
  s:=ss;
end;

procedure fpc_unicodestr_bool(b : boolean;len:sizeint;out s:unicodestring);compilerproc;

var ss:shortstring;

begin
  fpc_shortstr_bool(b,len,ss);
  s:=ss;
end;

procedure fpc_unicodestr_bool(b : boolean;len:sizeint;out s:unicodestring);compilerproc;

var ss:shortstring;

begin
  fpc_shortstr_bool(b,len,ss);
  s:=ss;
end;

{$ifdef FPC_HAS_STR_CURRENCY}
>>>>>>> graemeg/cpstrnew
procedure fpc_UnicodeStr_Currency(c : Currency;len,fr : SizeInt;out s : UnicodeString);compilerproc;
var
  ss: shortstring;
begin
  str(c:len:fr,ss);
  s:=UnicodeString(ss);
end;

Procedure fpc_UnicodeStr_SInt(v : ValSint; Len : SizeInt; out S : UnicodeString);compilerproc;
Var
  SS: ShortString;
begin
  Str (v:Len,SS);
  S:=UnicodeString(SS);
end;


Procedure fpc_UnicodeStr_UInt(v : ValUInt;Len : SizeInt; out S : UnicodeString);compilerproc;
Var
  SS: ShortString;
begin
  str(v:Len,SS);
  S:=UnicodeString(SS);
end;


{$ifndef CPU64}

Procedure fpc_UnicodeStr_Int64(v : Int64; Len : SizeInt; out S : UnicodeString);compilerproc;
Var
  SS: ShortString;
begin
  Str (v:Len,SS);
  S:=UnicodeString(SS);
end;


Procedure fpc_UnicodeStr_Qword(v : Qword;Len : SizeInt; out S : UnicodeString);compilerproc;
Var
  SS: ShortString;
begin
  str(v:Len,SS);
  S:=UnicodeString(SS);
end;

{$endif CPU64}


{$if defined(CPU16) or defined(CPU8)}

Procedure fpc_UnicodeStr_LongInt(v : LongInt; Len : SizeInt; out S : UnicodeString);compilerproc;
Var
  SS: ShortString;
begin
  Str (v:Len,SS);
  S:=UnicodeString(SS);
end;


Procedure fpc_UnicodeStr_LongWord(v : LongWord;Len : SizeInt; out S : UnicodeString);compilerproc;
Var
  SS: ShortString;
begin
  str(v:Len,SS);
  S:=UnicodeString(SS);
end;


Procedure fpc_UnicodeStr_SmallInt(v : SmallInt; Len : SizeInt; out S : UnicodeString);compilerproc;
Var
  SS: ShortString;
begin
  Str (v:Len,SS);
  S:=UnicodeString(SS);
end;


Procedure fpc_UnicodeStr_Word(v : Word;Len : SizeInt; out S : UnicodeString);compilerproc;
Var
  SS: ShortString;
begin
  str(v:Len,SS);
  S:=UnicodeString(SS);
end;

{$endif CPU16 or CPU8}


function UnicodeToUtf8(Dest: PChar; Source: PUnicodeChar; MaxBytes: SizeInt): SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    if assigned(Source) then
      Result:=UnicodeToUtf8(Dest,MaxBytes,Source,Length(Source))
    else
      Result:=0;
  end;


function UnicodeToUtf8(Dest: PChar; MaxDestBytes: SizeUInt; Source: PUnicodeChar; SourceChars: SizeUInt): SizeUInt;
  var
    i,j : SizeUInt;
    lw : longword;
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
            lw:=ord(Source[i]);
            case lw of
              0..$7f:
                begin
                  Dest[j]:=char(lw);
                  inc(j);
                end;
              $80..$7ff:
                begin
                  if j+1>=MaxDestBytes then
                    break;
                  Dest[j]:=char($c0 or (lw shr 6));
                  Dest[j+1]:=char($80 or (lw and $3f));
                  inc(j,2);
                end;
              $800..$d7ff,$e000..$ffff:
                begin
                  if j+2>=MaxDestBytes then
                    break;
                  Dest[j]:=char($e0 or (lw shr 12));
                  Dest[j+1]:=char($80 or ((lw shr 6) and $3f));
                  Dest[j+2]:=char($80 or (lw and $3f));
                  inc(j,3);
                end;
              $d800..$dbff:
                {High Surrogates}
                begin
                  if j+3>=MaxDestBytes then
                    break;
                  if (i+1<sourcechars) and
                     (word(Source[i+1]) >= $dc00) and
                     (word(Source[i+1]) <= $dfff) then
                    begin
                      { $d7c0 is ($d800 - ($10000 shr 10)) }
                      lw:=(longword(lw-$d7c0) shl 10) + (ord(source[i+1]) xor $dc00);
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
                  if (i+1<sourcechars) and
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


function Utf8ToUnicode(Dest: PUnicodeChar; Source: PChar; MaxChars: SizeInt): SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    if assigned(Source) then
      Result:=Utf8ToUnicode(Dest,MaxChars,Source,length(Source))
    else
      Result:=0;
  end;


function UTF8ToUnicode(Dest: PUnicodeChar; MaxDestChars: SizeUInt; Source: PChar; SourceBytes: SizeUInt): SizeUInt;
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


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function UTF8Encode(const s : RawByteString) : RawByteString; inline;
=======
function UTF8Encode(const s : RawByteString) : UTF8String; inline;
>>>>>>> graemeg/cpstrnew
=======
function UTF8Encode(const s : RawByteString) : UTF8String; inline;
>>>>>>> graemeg/cpstrnew
=======
function UTF8Encode(const s : RawByteString) : UTF8String; inline;
>>>>>>> graemeg/cpstrnew
=======
function UTF8Encode(const s : RawByteString) : UTF8String; inline;
>>>>>>> origin/cpstrnew
  begin
    Result:=UTF8Encode(UnicodeString(s));
  end;


{$ifndef FPC_HAS_UTF8ENCODE_UNICODESTRING}
{$define FPC_HAS_UTF8ENCODE_UNICODESTRING}
function UTF8Encode(const s : UnicodeString) : RawByteString;
  var
    i : SizeInt;
    hs : UTF8String;
  begin
    result:='';
    if s='' then
      exit;
    SetLength(hs,length(s)*3);
    i:=UnicodeToUtf8(pchar(hs),length(hs)+1,PUnicodeChar(s),length(s));
    if i>0 then
      begin
        SetLength(hs,i-1);
        result:=hs;
      end;
  end;
{$endif FPC_HAS_UTF8ENCODE_UNICODESTRING}


{$ifndef FPC_HAS_UTF8DECODE_UNICODESTRING}
{$define FPC_HAS_UTF8DECODE_UNICODESTRING}
function UTF8Decode(const s : RawByteString): UnicodeString;
  var
    i : SizeInt;
    hs : UnicodeString;
  begin
    result:='';
    if s='' then
      exit;
    SetLength(hs,length(s));
    i:=Utf8ToUnicode(PUnicodeChar(hs),length(hs)+1,pchar(s),length(s));
    if i>0 then
      begin
        SetLength(hs,i-1);
        result:=hs;
      end;
  end;
<<<<<<< HEAD
{$endif FPC_HAS_UTF8DECODE_UNICODESTRING}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function AnsiToUtf8(const s : RawByteString): RawByteString;{$ifdef SYSTEMINLINE}inline;{$endif}
=======
function AnsiToUtf8(const s : RawByteString): UTF8String;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======
function AnsiToUtf8(const s : RawByteString): UTF8String;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======


function AnsiToUtf8(const s : RawByteString): UTF8String;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======
function AnsiToUtf8(const s : RawByteString): UTF8String;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> origin/cpstrnew
  begin
    Result:=Utf8Encode(s);
  end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function Utf8ToAnsi(const s : RawByteString) : RawByteString;{$ifdef SYSTEMINLINE}inline;{$endif}
=======
function Utf8ToAnsi(const s : UTF8String) : RawByteString;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======
function Utf8ToAnsi(const s : UTF8String) : RawByteString;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======
function Utf8ToAnsi(const s : UTF8String) : RawByteString;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> graemeg/cpstrnew
=======
function Utf8ToAnsi(const s : UTF8String) : RawByteString;{$ifdef SYSTEMINLINE}inline;{$endif}
>>>>>>> origin/cpstrnew
  begin
    Result:=RawByteString(Utf8Decode(s));
  end;


procedure UCS4Encode(p: PWideChar; len: sizeint; out res: UCS4String);
  var
    i, reslen: sizeint;
    w: longint;
  begin
    reslen:=0;
    i:=0;
    { calculate required length }
    while (i<len) do
      begin
        if (p[i]<=#$d7ff) or (p[i]>=#$e000) then
          inc(i)
        else if (p[i]<=#$dbff) and
          (i+1<len) and
          (p[i+1]>=#$dc00) and
          (p[i+1]<=#$dfff) then
          inc(i,2)
        else
          inc(i);
        inc(reslen);
      end;
    SetLength(res,reslen+1); { +1 for null termination }
    reslen:=0;
    i:=0;
    { do conversion }
    while (i<len) do
      begin
        w:=ord(p[i]);
        if (w<=$d7ff) or (w>=$e000) then
          res[reslen]:=w
        else if (w<=$dbff) and
          (i+1<len) and
          (p[i+1]>=#$dc00) and
          (p[i+1]<=#$dfff) then
          begin
            res[reslen]:=(UCS4Char(w-$d7c0) shl 10)+(UCS4Char(p[i+1]) xor $dc00);
            inc(i);
          end
        else { invalid surrogate pair }
          res[reslen]:=w;
        inc(i);
        inc(reslen);
      end;
    res[reslen]:=0;
  end;

{$ifndef FPC_HAS_UCS4STRING_TO_UNICODESTR}
{$define FPC_HAS_UCS4STRING_TO_UNICODESTR}
function UnicodeStringToUCS4String(const s : UnicodeString) : UCS4String;
  begin
    UCS4Encode(PWideChar(s),Length(s),result);
  end;
{$endif FPC_HAS_UCS4STRING_TO_UNICODESTR}

{$ifndef FPC_HAS_WIDESTR_TO_UCS4STRING}
{$define FPC_HAS_WIDESTR_TO_UCS4STRING}
function WideStringToUCS4String(const s : WideString) : UCS4String;
  begin
    UCS4Encode(PWideChar(s),Length(s),result);
  end;
{$endif FPC_HAS_WIDESTR_TO_UCS4STRING}


{$ifndef FPC_HAS_UCS4STRING_TO_WIDESTR}
{$define FPC_HAS_UCS4STRING_TO_WIDESTR}
{ dest should point to previously allocated wide/unicodestring }
procedure UCS4Decode(const s: UCS4String; dest: PWideChar);
var
  i: sizeint;
  nc: UCS4Char;
begin
  for i:=0 to length(s)-2 do  { -2 because s contains explicit terminating #0 }
    begin
      nc:=s[i];
      if (nc<=$ffff) then
        dest^:=widechar(nc)
      else if (dword(nc)<=$10ffff) then
        begin
          dest^:=widechar(nc shr 10 + $d7c0);
          { subtracting $10000 doesn't change low 10 bits }
          dest[1]:=widechar(nc and $3ff + $dc00);
          inc(dest);
        end
      else  { invalid code point }
        dest^:='?';
      inc(dest);
    end;
end;


function UCS4StringToUnicodeString(const s : UCS4String) : UnicodeString;
  var
    i        : SizeInt;
    reslen   : SizeInt;
  begin
    reslen:=0;
    for i:=0 to length(s)-2 do     { skip terminating #0 }
      Inc(reslen,1+ord((s[i]>$ffff) and (s[i]<=$10ffff)));
    SetLength(result,reslen);
    UCS4Decode(s,pointer(result));
  end;


function UCS4StringToWideString(const s : UCS4String) : WideString;
  var
    i      : SizeInt;
    reslen : SizeInt;
  begin
    reslen:=0;
    for i:=0 to length(s)-2 do     { skip terminating #0 }
      Inc(reslen,1+ord((s[i]>$ffff) and (s[i]<=$10ffff)));
    SetLength(result,reslen);
    UCS4Decode(s,pointer(result));
  end;
{$endif FPC_HAS_UCS4STRING_TO_WIDESTR}


{$ifndef FPC_HAS_BUILTIN_WIDESTR_MANAGER}
const
  SNoUnicodestrings = 'This binary has no string conversion support compiled in.';
  SRecompileWithUnicodestrings = 'Recompile the application with a unit that installs a unicodestring manager in the program uses clause.';

procedure unimplementedunicodestring;
  begin
{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
    If IsConsole then
      begin
      Writeln(StdErr,SNoUnicodestrings);
      Writeln(StdErr,SRecompileWithUnicodestrings);
      end;
{$endif FPC_HAS_FEATURE_CONSOLEIO}
    HandleErrorAddrFrameInd(233,get_pc_addr,get_frame);
  end;


function StringElementSize(const S: UnicodeString): Word; overload;
  begin
    if assigned(Pointer(S)) then
      Result:=PUnicodeRec(pointer(S)-UnicodeFirstOff)^.ElementSize
    else
      Result:=SizeOf(UnicodeChar);
  end;


<<<<<<< HEAD
function StringRefCount(const S: UnicodeString): SizeInt; overload;
  begin
    if assigned(Pointer(S)) then
      Result:=PUnicodeRec(pointer(S)-UnicodeFirstOff)^.Ref
    else
      Result:=0;
<<<<<<< HEAD
  end;


<<<<<<< HEAD
function StringCodePage(const S: UnicodeString): TSystemCodePage; overload;
  begin
{$ifdef FPC_HAS_CPSTRING}
    if assigned(Pointer(S)) then
      Result:=PUnicodeRec(pointer(S)-UnicodeFirstOff)^.CodePage
    else
{$endif FPC_HAS_CPSTRING}
      Result:=DefaultUnicodeCodePage;
  end;


{$warnings off}
function StubUnicodeCase(const s : UnicodeString) : UnicodeString;
=======
  end;


function StringCodePage(const S: UnicodeString): TSystemCodePage; overload;
  begin
{$ifdef FPC_HAS_CPSTRING}
    if assigned(Pointer(S)) then
      Result:=PUnicodeRec(pointer(S)-UnicodeFirstOff)^.CodePage
    else
{$endif FPC_HAS_CPSTRING}
      Result:=DefaultUnicodeCodePage;
  end;


<<<<<<< HEAD
{$warnings off}
function GenericUnicodeCase(const s : UnicodeString) : UnicodeString;
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> graemeg/cpstrnew
function StringElementSize(const S: UnicodeString): Word; overload;
  begin
    if assigned(Pointer(S)) then
      Result:=PUnicodeRec(pointer(S)-UnicodeFirstOff)^.ElementSize
    else
      Result:=SizeOf(UnicodeChar);
  end;


function StringRefCount(const S: UnicodeString): SizeInt; overload;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
  begin
    if assigned(Pointer(S)) then
      Result:=PUnicodeRec(pointer(S)-UnicodeFirstOff)^.Ref
    else
      Result:=0;
  end;

function StubCompareUnicodeString(const s1, s2 : UnicodeString; Options : TCompareOptions) : PtrInt;
=======
function StringElementSize(const S: UnicodeString): Word; overload;
>>>>>>> origin/cpstrnew
  begin
    if assigned(Pointer(S)) then
      Result:=PUnicodeRec(pointer(S)-UnicodeFirstOff)^.ElementSize
    else
      Result:=SizeOf(UnicodeChar);
  end;

<<<<<<< HEAD
<<<<<<< HEAD
function StubWideCase(const s: WideString): WideString;
=======
function CompareUnicodeString(const s1, s2 : UnicodeString) : PtrInt;
>>>>>>> graemeg/cpstrnew
=======
function StringCodePage(const S: UnicodeString): TSystemCodePage; overload;
  begin
{$ifdef FPC_HAS_CPSTRING}
    if assigned(Pointer(S)) then
      Result:=PUnicodeRec(pointer(S)-UnicodeFirstOff)^.CodePage
    else
{$endif FPC_HAS_CPSTRING}
      Result:=DefaultUnicodeCodePage;
  end;


<<<<<<< HEAD
{$warnings off}
function GenericUnicodeCase(const s : UnicodeString) : UnicodeString;
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
  begin
    if assigned(Pointer(S)) then
      Result:=PUnicodeRec(pointer(S)-UnicodeFirstOff)^.Ref
    else
      Result:=0;
  end;

<<<<<<< HEAD
function StubCompareWideString(const s1, s2 : WideString; Options : TCompareOptions) : PtrInt;
=======

<<<<<<< HEAD
<<<<<<< HEAD
function CompareTextUnicodeString(const s1, s2 : UnicodeString): PtrInt;
>>>>>>> graemeg/cpstrnew
  begin
    unimplementedunicodestring;
  end;
=======
function CompareUnicodeString(const s1, s2 : UnicodeString) : PtrInt;
=======
function StringCodePage(const S: UnicodeString): TSystemCodePage; overload;
  begin
{$ifdef FPC_HAS_CPSTRING}
    if assigned(Pointer(S)) then
      Result:=PUnicodeRec(pointer(S)-UnicodeFirstOff)^.CodePage
    else
{$endif FPC_HAS_CPSTRING}
      Result:=DefaultUnicodeCodePage;
  end;


{$warnings off}
function GenericUnicodeCase(const s : UnicodeString) : UnicodeString;
>>>>>>> graemeg/cpstrnew
=======
function StringRefCount(const S: UnicodeString): SizeInt; overload;
>>>>>>> origin/cpstrnew
  begin
    if assigned(Pointer(S)) then
      Result:=PUnicodeRec(pointer(S)-UnicodeFirstOff)^.Ref
    else
      Result:=0;
  end;


<<<<<<< HEAD
<<<<<<< HEAD
function CompareTextUnicodeString(const s1, s2 : UnicodeString): PtrInt;
=======
function StringCodePage(const S: UnicodeString): TSystemCodePage; overload;
  begin
{$ifdef FPC_HAS_CPSTRING}
    if assigned(Pointer(S)) then
      Result:=PUnicodeRec(pointer(S)-UnicodeFirstOff)^.CodePage
    else
{$endif FPC_HAS_CPSTRING}
      Result:=DefaultUnicodeCodePage;
  end;


{$warnings off}
function GenericUnicodeCase(const s : UnicodeString) : UnicodeString;
>>>>>>> origin/cpstrnew
  begin
    unimplementedunicodestring;
  end;
>>>>>>> graemeg/cpstrnew
=======
function CompareUnicodeString(const s1, s2 : UnicodeString) : PtrInt;
  begin
    unimplementedunicodestring;
  end;


<<<<<<< HEAD
function CompareTextUnicodeString(const s1, s2 : UnicodeString): PtrInt;
  begin
    unimplementedunicodestring;
  end;
>>>>>>> graemeg/cpstrnew
=======
function CompareUnicodeString(const s1, s2 : UnicodeString) : PtrInt;
  begin
    unimplementedunicodestring;
  end;


function CompareTextUnicodeString(const s1, s2 : UnicodeString): PtrInt;
  begin
    unimplementedunicodestring;
  end;
>>>>>>> origin/cpstrnew
{$warnings on}

procedure initunicodestringmanager;
  begin
{$ifndef HAS_WIDESTRINGMANAGER}
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
    widestringmanager.Unicode2AnsiMoveProc:=@DefaultUnicode2AnsiMove;
    widestringmanager.Ansi2UnicodeMoveProc:=@DefaultAnsi2UnicodeMove;
    widestringmanager.UpperUnicodeStringProc:=@GenericUnicodeCase;
    widestringmanager.LowerUnicodeStringProc:=@GenericUnicodeCase;
{$endif HAS_WIDESTRINGMANAGER}
    widestringmanager.CompareUnicodeStringProc:=@CompareUnicodeString;
    widestringmanager.CompareTextUnicodeStringProc:=@CompareTextUnicodeString;

>>>>>>> graemeg/cpstrnew
{$ifdef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    widestringmanager.Ansi2WideMoveProc:=@defaultAnsi2UnicodeMove;
<<<<<<< HEAD
{$else FPC_WIDESTRING_EQUAL_UNICODESTRING}
    widestringmanager.Ansi2WideMoveProc:=@DefaultAnsi2WideMove;
=======
    widestringmanager.UpperWideStringProc:=@GenericUnicodeCase;
    widestringmanager.LowerWideStringProc:=@GenericUnicodeCase;
{$endif HAS_WIDESTRINGMANAGER}
    widestringmanager.CompareWideStringProc:=@CompareUnicodeString;
    widestringmanager.CompareTextWideStringProc:=@CompareTextUnicodeString;
    widestringmanager.CharLengthPCharProc:=@DefaultCharLengthPChar;
    widestringmanager.CodePointLengthProc:=@DefaultCodePointLength;
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
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}
    widestringmanager.Wide2AnsiMoveProc:=@DefaultUnicode2AnsiMove;
    widestringmanager.UpperWideStringProc:=@StubWideCase;
    widestringmanager.LowerWideStringProc:=@StubWideCase;
    widestringmanager.Unicode2AnsiMoveProc:=@DefaultUnicode2AnsiMove;
    widestringmanager.Ansi2UnicodeMoveProc:=@DefaultAnsi2UnicodeMove;
    widestringmanager.UpperUnicodeStringProc:=@StubUnicodeCase;
    widestringmanager.LowerUnicodeStringProc:=@StubUnicodeCase;
{$endif HAS_WIDESTRINGMANAGER}
    widestringmanager.CompareWideStringProc:=@StubCompareWideString;
//    widestringmanager.CompareTextWideStringProc:=@StubCompareWideString;
    widestringmanager.CompareUnicodeStringProc:=@StubCompareUnicodeString;
    widestringmanager.CharLengthPCharProc:=@DefaultCharLengthPChar;
    widestringmanager.CodePointLengthProc:=@DefaultCodePointLength;
    widestringmanager.GetStandardCodePageProc:=@DefaultGetStandardCodePage;
  end;
{$endif FPC_HAS_BUILTIN_WIDESTR_MANAGER}


{$ifndef FPC_HAS_TOSINGLEBYTEFILESYSTEMENCODEDFILENAME_UNICODESTRING}
{$define FPC_HAS_TOSINGLEBYTEFILESYSTEMENCODEDFILENAME_UNICODESTRING}
Function ToSingleByteFileSystemEncodedFileName(const Str: UnicodeString): RawByteString;
Begin
  widestringmanager.Unicode2AnsiMoveProc(punicodechar(Str),Result,
    DefaultFileSystemCodePage,Length(Str));
End;
{$endif FPC_HAS_TOSINGLEBYTEFILESYSTEMENCODEDFILENAME_UNICODESTRING}


{$ifndef FPC_HAS_TOSINGLEBYTEFILESYSTEMENCODEDFILENAME_UNICODECHARARRAY}
{$define FPC_HAS_TOSINGLEBYTEFILESYSTEMENCODEDFILENAME_UNICODECHARARRAY}
Function ToSingleByteFileSystemEncodedFileName(const arr: array of widechar): RawByteString;
Begin
  widestringmanager.Unicode2AnsiMoveProc(@arr[0],Result,
    DefaultFileSystemCodePage,length(pwidechar(@arr[0])));
End;
{$endif FPC_HAS_TOSINGLEBYTEFILESYSTEMENCODEDFILENAME_UNICODECHARARRAY}

Function ToSingleByteFileSystemEncodedFileName(const Str: RawByteString): RawByteString;
Begin
  Result:=Str;
  SetCodePage(Result,DefaultFileSystemCodePage,True);
End;

function UTF8ToString(const S: UTF8String): UnicodeString; inline;
begin
  Result := UTF8Decode(S);
end;

function UTF8ToString(const S: RawByteString): UnicodeString; inline;

Var
  UTF8 : UTF8String;

begin
  UTF8:=S;
  Result := UTF8Decode(UTF8);
end;

function UTF8ToString(const S: ShortString): UnicodeString; 

Var
  UTF8 : UTF8String;

begin
  UTF8:=S;
  Result := UTF8Decode(UTF8);
end;

function UTF8ToString(const S: PAnsiChar): UnicodeString;
var
  UTF: UTF8String;
  Count: Integer;
begin
  Count := StrLen(S);
  SetLength(UTF, Count);
  if Count > 0 then
    Move(S^, UTF[1], Count);
  Result := UTF8ToString(UTF);
end;

function UTF8ToString(const S: array of AnsiChar): UnicodeString;
var
  UTF: UTF8String;
  Count: Integer;
begin
  Count := Length(S);
  SetLength(UTF, Count);
  if Count > 0 then
    Move(S[Low(S)], UTF[1], Count);
  Result := UTF8ToString(UTF);
end;

function UTF8ToString(const S: array of Byte): UnicodeString;
var
  UTF: UTF8String;
  Count: Integer;
begin
  Count := Length(S);
  SetLength(UTF, Count);
  if Count > 0 then
    Move(S[Low(S)], UTF[1], Count);
  Result := UTF8ToString(UTF);
end;
