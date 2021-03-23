{
    This file is part of the Free Pascal Run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    This file contains the declarations of internal compiler helper
    routines. That means you can *NOT* call these directly, as they may
    be changed or even removed at any time. The only reason they are
    included in the interface of the system unit, is so that the
    compiler doesn't need special code to access their parameter
    list information etc.

    Note that due to the "compilerproc" directive, it isn't even possible
    to use these routines in your programs.

    See the File COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ some dummy types necessary to have generic resulttypes for certain compilerprocs }
type
  fpc_small_set = bitpacked array[0..31] of 0..1;
  fpc_normal_set = bitpacked array[0..255] of 0..1;
  fpc_normal_set_byte = array[0..31] of byte;
  fpc_normal_set_long = array[0..7] of longint;
  fpc_stub_dynarray = array of byte;


{$ifdef FPC_HAS_FEATURE_HEAP}
{ Needed to solve overloading problem with call from assembler (PFV) }
Function  fpc_getmem(size:ptruint):pointer;compilerproc;
Procedure fpc_freemem(p:pointer);compilerproc;
{$endif FPC_HAS_FEATURE_HEAP}

{ used by Default() in code blocks }
procedure fpc_zeromem(p:pointer;len:sizeuint);inline;compilerproc;
procedure fpc_fillmem(out data;len:sizeuint;b : byte);inline;compilerproc;

procedure fpc_Shortstr_SetLength(var s:shortstring;len:SizeInt); compilerproc;
procedure fpc_shortstr_assign(len:{$ifdef cpu16}smallint{$else}longint{$endif};sstr,dstr:pointer); compilerproc;
procedure fpc_shortstr_to_shortstr(out res:shortstring; const sstr: shortstring); compilerproc;

procedure fpc_shortstr_concat(var dests:shortstring;const s1,s2:shortstring);compilerproc;
procedure fpc_shortstr_concat_multi(var dests:shortstring;const sarr:array of pshortstring);compilerproc;
procedure fpc_shortstr_append_shortstr(var s1:shortstring;const s2:shortstring); compilerproc;
function fpc_shortstr_compare(const left,right:shortstring) : longint; compilerproc;
function fpc_shortstr_compare_equal(const left,right:shortstring) : longint; compilerproc;

procedure fpc_pchar_to_shortstr(out res : shortstring;p:pchar); compilerproc;

function fpc_pchar_length(p:pchar):sizeint; compilerproc;
function fpc_pwidechar_length(p:pwidechar):sizeint; compilerproc;

procedure fpc_chararray_to_shortstr(out res : shortstring;const arr: array of char; zerobased: boolean = true); compilerproc;
procedure fpc_shortstr_to_chararray(out res: array of char; const src: ShortString); compilerproc;

Function  fpc_shortstr_Copy(const s:shortstring;index:SizeInt;count:SizeInt):shortstring;compilerproc{$ifndef VER3_0}:fpc_in_copy_x{$endif VER3_0};
function  fpc_char_copy(c:char;index : SizeInt;count : SizeInt): shortstring;compilerproc{$ifndef VER3_0}:fpc_in_copy_x{$endif VER3_0};
{$ifndef VER3_0}
Procedure fpc_shortstr_delete(var s:shortstring;index:SizeInt;count:SizeInt); compilerproc:fpc_in_delete_x_y_z;
Procedure fpc_shortstr_insert(const source:shortstring;var s:shortstring;index:SizeInt); compilerproc:fpc_in_insert_x_y_z;
Procedure fpc_shortstr_insert_char(source:Char;var s:shortstring;index:SizeInt); compilerproc:fpc_in_insert_x_y_z;
{$endif VER3_0}

{$ifdef FPC_HAS_FEATURE_DYNARRAYS}
{$if defined(VER3_0) or defined(VER3_2)}
function fpc_dynarray_copy(psrc : pointer;ti : pointer;
    lowidx,count:tdynarrayindex) : fpc_stub_dynarray;compilerproc;
{$endif VER3_0 or VER3_2}
function fpc_array_to_dynarray_copy(psrc : pointer;ti : pointer;
    lowidx,count,maxcount:tdynarrayindex;
    elesize : sizeint;
    eletype : pointer
    ) : fpc_stub_dynarray;compilerproc;
function  fpc_dynarray_length(p : pointer) : tdynarrayindex; compilerproc;
function  fpc_dynarray_high(p : pointer) : tdynarrayindex; compilerproc;
procedure fpc_dynarray_clear(var p : pointer;ti : pointer); compilerproc;
procedure fpc_dynarray_decr_ref(var p : pointer;ti : pointer); compilerproc;
procedure fpc_dynarray_incr_ref(p : pointer); compilerproc;
procedure fpc_dynarray_setlength(var p : pointer;pti : pointer; dimcount : sizeint;dims : pdynarrayindex); compilerproc;
procedure fpc_dynarray_assign(var dest : pointer; src : pointer; ti: pointer); compilerproc;
{$ifndef VER3_0}
{ no reference to the Delete()/Insert() intrinsic, due to typeinfo parameter }
procedure fpc_dynarray_delete(var p : pointer;source,count : SizeInt;pti : pointer);compilerproc;
procedure fpc_dynarray_insert(var p : pointer;source : SizeInt;data : pointer;count : SizeInt;pti : pointer);compilerproc;
procedure fpc_dynarray_concat_multi(var dest : pointer; pti: pointer; const sarr:array of pointer); compilerproc;
procedure fpc_dynarray_concat(var dest : pointer; pti: pointer; const src1,src2 : pointer); compilerproc;
{$endif VER3_0}
{$endif FPC_HAS_FEATURE_DYNARRAYS}

{ Str() support }
procedure fpc_ShortStr_sint(v : valsint;len : SizeInt;out s : shortstring); compilerproc;
procedure fpc_shortstr_uint(v : valuint;len : SizeInt;out s : shortstring); compilerproc;
{$ifndef FPUNONE}
procedure fpc_ShortStr_Float(d : ValReal;len,fr,rt : SizeInt;out s : shortstring); compilerproc;
{$endif}
procedure fpc_shortstr_enum(ordinal,len:sizeint;typinfo,ord2strindex:pointer;out s:shortstring);compilerproc;
procedure fpc_shortstr_bool(b : boolean;len:sizeint;out s:shortstring);compilerproc;
procedure fpc_ShortStr_Currency(c : currency; len,f : SizeInt; out s : shortstring); compilerproc;

procedure fpc_chararray_sint(v : valsint;len : SizeInt;out a : array of char); compilerproc;
procedure fpc_chararray_uint(v : valuint;len : SizeInt;out a : array of char); compilerproc;
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
procedure fpc_AnsiStr_sint(v : valsint; Len : SizeInt; out S : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
procedure fpc_AnsiStr_uint(v : valuint;Len : SizeInt; out S : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
{$ifndef FPUNONE}
procedure fpc_AnsiStr_Float(d : ValReal;len,fr,rt : SizeInt;out s : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
{$endif}
procedure fpc_ansistr_enum(ordinal,len:sizeint;typinfo,ord2strindex:pointer;out s:RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
procedure fpc_ansistr_bool(b : boolean;len:sizeint;out s:RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
procedure fpc_AnsiStr_Currency(c : currency;len,fr : SizeInt;out s : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}

{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
  {$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
  procedure fpc_WideStr_sint(v : valsint; Len : SizeInt; out S : WideString); compilerproc;
  procedure fpc_WideStr_uint(v : valuint;Len : SizeInt; out S : WideString); compilerproc;
  {$endif ndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
  procedure fpc_UnicodeStr_sint(v : valsint; Len : SizeInt; out S : UnicodeString); compilerproc;
  procedure fpc_UnicodeStr_uint(v : valuint;Len : SizeInt; out S : UnicodeString); compilerproc;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}

{$ifndef CPU64}
  procedure fpc_shortstr_qword(v : qword;len : SizeInt;out s : shortstring); compilerproc;
  procedure fpc_shortstr_int64(v : int64;len : SizeInt;out s : shortstring); compilerproc;
  procedure fpc_chararray_qword(v : qword;len : SizeInt;out a : array of char); compilerproc;
  procedure fpc_chararray_int64(v : int64;len : SizeInt;out a : array of char); compilerproc;
  {$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
  procedure fpc_ansistr_qword(v : qword;len : SizeInt;out s : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
  procedure fpc_ansistr_int64(v : int64;len : SizeInt;out s : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
  {$endif FPC_HAS_FEATURE_ANSISTRINGS}

  {$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
    {$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    procedure fpc_widestr_qword(v : qword;len : SizeInt;out s : widestring); compilerproc;
    procedure fpc_widestr_int64(v : int64;len : SizeInt;out s : widestring); compilerproc;
    {$endif ndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    procedure fpc_UnicodeStr_qword(v : qword;len : SizeInt;out s : UnicodeString); compilerproc;
    procedure fpc_UnicodeStr_int64(v : int64;len : SizeInt;out s : UnicodeString); compilerproc;
  {$endif FPC_HAS_FEATURE_WIDESTRINGS}
{$endif CPU64}
{$if defined(CPU16) or defined(CPU8)}
  procedure fpc_shortstr_longword(v : longword;len : SizeInt;out s : shortstring); compilerproc;
  procedure fpc_shortstr_longint(v : longint;len : SizeInt;out s : shortstring); compilerproc;
  procedure fpc_chararray_longword(v : longword;len : SizeInt;out a : array of char); compilerproc;
  procedure fpc_chararray_longint(v : longint;len : SizeInt;out a : array of char); compilerproc;
  {$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
  procedure fpc_ansistr_longword(v : longword;len : SizeInt;out s : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
  procedure fpc_ansistr_longint(v : longint;len : SizeInt;out s : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
  {$endif FPC_HAS_FEATURE_ANSISTRINGS}

  {$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
    {$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    procedure fpc_widestr_longword(v : longword;len : SizeInt;out s : widestring); compilerproc;
    procedure fpc_widestr_longint(v : longint;len : SizeInt;out s : widestring); compilerproc;
    {$endif ndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    procedure fpc_UnicodeStr_longword(v : longword;len : SizeInt;out s : UnicodeString); compilerproc;
    procedure fpc_UnicodeStr_longint(v : longint;len : SizeInt;out s : UnicodeString); compilerproc;
  {$endif FPC_HAS_FEATURE_WIDESTRINGS}

  procedure fpc_shortstr_word(v : word;len : SizeInt;out s : shortstring); compilerproc;
  procedure fpc_shortstr_smallint(v : smallint;len : SizeInt;out s : shortstring); compilerproc;
  procedure fpc_chararray_word(v : word;len : SizeInt;out a : array of char); compilerproc;
  procedure fpc_chararray_smallint(v : smallint;len : SizeInt;out a : array of char); compilerproc;
  {$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
  procedure fpc_ansistr_word(v : word;len : SizeInt;out s : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
  procedure fpc_ansistr_smallint(v : smallint;len : SizeInt;out s : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
  {$endif FPC_HAS_FEATURE_ANSISTRINGS}

  {$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
    {$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    procedure fpc_widestr_word(v : word;len : SizeInt;out s : widestring); compilerproc;
    procedure fpc_widestr_smallint(v : smallint;len : SizeInt;out s : widestring); compilerproc;
    {$endif ndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    procedure fpc_UnicodeStr_word(v : word;len : SizeInt;out s : UnicodeString); compilerproc;
    procedure fpc_UnicodeStr_smallint(v : smallint;len : SizeInt;out s : UnicodeString); compilerproc;
  {$endif FPC_HAS_FEATURE_WIDESTRINGS}
{$endif CPU16 or CPU8}
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
  {$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    {$ifndef FPUNONE}
    procedure fpc_WideStr_Float(d : ValReal;len,fr,rt : SizeInt;out s : WideString); compilerproc;
    {$endif}
    procedure fpc_widestr_enum(ordinal,len:sizeint;typinfo,ord2strindex:pointer;out s:widestring);compilerproc;
    procedure fpc_widestr_bool(b : boolean;len:sizeint;out s:widestring);compilerproc;
    procedure fpc_WideStr_Currency(c : Currency;len,fr : SizeInt;out s : WideString);compilerproc;
  {$endif ndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    {$ifndef FPUNONE}
    procedure fpc_UnicodeStr_Float(d : ValReal;len,fr,rt : SizeInt;out s : UnicodeString); compilerproc;
    {$endif}
    procedure fpc_unicodestr_enum(ordinal,len:sizeint;typinfo,ord2strindex:pointer;out s:unicodestring);compilerproc;
    procedure fpc_unicodestr_bool(b : boolean;len:sizeint;out s:unicodestring);compilerproc;
    procedure fpc_UnicodeStr_Currency(c : Currency;len,fr : SizeInt;out s : UnicodeString);compilerproc;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}

{$ifndef FPUNONE}
procedure fpc_chararray_Float(d : ValReal;len,fr,rt : SizeInt;out a : array of char); compilerproc;
{$endif}
{$ifndef CPUAVR}
{ currently, the avr code generator fails on this procedure, so we disable it, 
  this is not a good solution but fixing compilation of this procedure for
  avr is hard, requires significant changes to the register allocator to take
  care of different register classes }
procedure fpc_chararray_enum(ordinal,len:sizeint;typinfo,ord2strindex:pointer;out a : array of char);compilerproc;
{$endif CPUAVR}

procedure fpc_chararray_bool(b : boolean;len:sizeint;out a : array of char);compilerproc;
procedure fpc_chararray_Currency(c : Currency;len,fr : SizeInt;out a : array of char);compilerproc;

{ Val() support }
{$ifndef FPUNONE}
Function fpc_Val_Real_ShortStr(const s : shortstring; out code : ValSInt): ValReal; compilerproc;
{$endif}
Function fpc_Val_SInt_ShortStr(DestSize: SizeInt; Const S: ShortString; out Code: ValSInt): ValSInt; compilerproc;
Function fpc_Val_UInt_Shortstr(Const S: ShortString; out Code: ValSInt): ValUInt; compilerproc;
function fpc_val_enum_shortstr(str2ordindex:pointer;const s:shortstring;out code:valsint):longint; compilerproc;
Function fpc_Val_Currency_ShortStr(const s : shortstring; out Code : ValSInt): currency; compilerproc;
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
{$ifndef FPUNONE}
Function fpc_Val_Real_AnsiStr(Const S : RawByteString; out Code : ValSInt): ValReal; compilerproc;
{$endif}
Function fpc_Val_UInt_AnsiStr (Const S : RawByteString; out Code : ValSInt): ValUInt; compilerproc;
Function fpc_Val_SInt_AnsiStr (DestSize: SizeInt; Const S : RawByteString; out Code : ValSInt): ValSInt; compilerproc;
Function fpc_Val_Currency_AnsiStr(Const S : RawByteString; out Code : ValSInt): Currency; compilerproc;
function fpc_Val_enum_ansistr(str2ordindex:pointer;const s:RawByteString;out code:valsint):longint; compilerproc;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}

{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
  {$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
  {$ifndef FPUNONE}
  Function fpc_Val_Real_WideStr(Const S : WideString; out Code : ValSInt): ValReal; compilerproc;
  {$endif}
  Function fpc_Val_SInt_WideStr (DestSize: SizeInt; Const S : WideString; out Code : ValSInt): ValSInt; compilerproc;
  Function fpc_Val_UInt_WideStr (Const S : WideString; out Code : ValSInt): ValUInt; compilerproc;
  function fpc_val_Enum_WideStr (str2ordindex:pointer;const s:WideString;out code:valsint):longint;compilerproc;
  Function fpc_Val_Currency_WideStr(Const S : WideString; out Code : ValSInt): Currency; compilerproc;
  {$endif ndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
  {$ifndef FPUNONE}
  Function fpc_Val_Real_UnicodeStr(Const S : UnicodeString; out Code : ValSInt): ValReal; compilerproc;
  {$endif}
  Function fpc_Val_SInt_UnicodeStr (DestSize: SizeInt; Const S : UnicodeString; out Code : ValSInt): ValSInt; compilerproc;
  Function fpc_Val_UInt_UnicodeStr (Const S : UnicodeString; out Code : ValSInt): ValUInt; compilerproc;
  function fpc_val_Enum_UnicodeStr(str2ordindex:pointer;const s:UnicodeString;out code:valsint):longint;compilerproc;
  Function fpc_Val_Currency_UnicodeStr(Const S : UnicodeString; out Code : ValSInt): Currency; compilerproc;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}

{$ifndef CPU64}
Function fpc_val_int64_shortstr(Const S: ShortString; out Code: ValSInt): Int64; compilerproc;
Function fpc_val_qword_shortstr(Const S: ShortString; out Code: ValSInt): QWord; compilerproc;
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Function fpc_Val_qword_AnsiStr (Const S : RawByteString; out Code : ValSInt): qword;compilerproc;
Function fpc_Val_int64_AnsiStr (Const S : RawByteString; out Code : ValSInt): Int64; compilerproc;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}

{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
Function fpc_Val_qword_WideStr (Const S : WideString; out Code : ValSInt): qword; compilerproc;
Function fpc_Val_int64_WideStr (Const S : WideString; out Code : ValSInt): Int64; compilerproc;
{$endif ndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
Function fpc_Val_qword_UnicodeStr (Const S : UnicodeString; out Code : ValSInt): qword; compilerproc;
Function fpc_Val_int64_UnicodeStr (Const S : UnicodeString; out Code : ValSInt): Int64; compilerproc;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}

{$endif CPU64}

{$if defined(CPU16) or defined(CPU8)}
Function fpc_val_longint_shortstr(Const S: ShortString; out Code: ValSInt): LongInt; compilerproc;
Function fpc_val_longword_shortstr(Const S: ShortString; out Code: ValSInt): LongWord; compilerproc;
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Function fpc_Val_longword_AnsiStr (Const S : RawByteString; out Code : ValSInt): LongWord;compilerproc;
Function fpc_Val_longint_AnsiStr (Const S : RawByteString; out Code : ValSInt): LongInt; compilerproc;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}

{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
Function fpc_Val_longword_WideStr (Const S : WideString; out Code : ValSInt): LongWord; compilerproc;
Function fpc_Val_longint_WideStr (Const S : WideString; out Code : ValSInt): LongInt; compilerproc;
{$endif ndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
Function fpc_Val_longword_UnicodeStr (Const S : UnicodeString; out Code : ValSInt): LongWord; compilerproc;
Function fpc_Val_longint_UnicodeStr (Const S : UnicodeString; out Code : ValSInt): LongInt; compilerproc;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}

Function fpc_val_smallint_shortstr(Const S: ShortString; out Code: ValSInt): SmallInt; compilerproc;
Function fpc_val_word_shortstr(Const S: ShortString; out Code: ValSInt): Word; compilerproc;
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Function fpc_Val_word_AnsiStr (Const S : RawByteString; out Code : ValSInt): Word;compilerproc;
Function fpc_Val_smallint_AnsiStr (Const S : RawByteString; out Code : ValSInt): SmallInt; compilerproc;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}

{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
Function fpc_Val_word_WideStr (Const S : WideString; out Code : ValSInt): Word; compilerproc;
Function fpc_Val_smallint_WideStr (Const S : WideString; out Code : ValSInt): SmallInt; compilerproc;
{$endif ndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
Function fpc_Val_word_UnicodeStr (Const S : UnicodeString; out Code : ValSInt): Word; compilerproc;
Function fpc_Val_smallint_UnicodeStr (Const S : UnicodeString; out Code : ValSInt): SmallInt; compilerproc;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
{$endif CPU16 or CPU8}

{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Procedure fpc_ansistr_decr_ref (Var S : Pointer); compilerproc;
Procedure fpc_ansistr_incr_ref (S : Pointer); compilerproc;
Procedure fpc_AnsiStr_Assign (Var DestS : Pointer;S2 : Pointer); compilerproc;
Procedure fpc_AnsiStr_Concat (Var DestS : RawByteString;const S1,S2 : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
Procedure fpc_AnsiStr_Concat_multi (Var DestS : RawByteString;const sarr:array of RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
{$ifdef EXTRAANSISHORT}
Procedure fpc_AnsiStr_ShortStr_Concat (Var S1: AnsiString; Var S2 : ShortString); compilerproc;
{$endif EXTRAANSISHORT}
procedure fpc_AnsiStr_To_ShortStr (out res : shortstring;const S2 : RawByteString); compilerproc;
{$ifdef FPC_HAS_CPSTRING}
Function fpc_AnsiStr_To_AnsiStr (const S : RawByteString;cp : TSystemCodePage): RawByteString; compilerproc;
{$endif FPC_HAS_CPSTRING}
Function fpc_ShortStr_To_AnsiStr (Const S2 : ShortString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}): RawByteString; compilerproc;
Function fpc_Char_To_AnsiStr(const c : AnsiChar{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}): RawByteString; compilerproc;

Function fpc_PChar_To_AnsiStr(const p : PAnsiChar{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}): RawByteString; compilerproc;
Function fpc_CharArray_To_AnsiStr(const arr: array of AnsiChar; {$ifdef FPC_HAS_CPSTRING}cp : TSystemCodePage;{$endif FPC_HAS_CPSTRING}zerobased: boolean = true): RawByteString; compilerproc;
procedure fpc_ansistr_to_chararray(out res: array of AnsiChar; const src: RawByteString)compilerproc;
Function fpc_AnsiStr_Compare(const S1,S2 : RawByteString): SizeInt; compilerproc;
Function fpc_AnsiStr_Compare_equal(const S1,S2 : RawByteString): SizeInt; compilerproc;
Procedure fpc_AnsiStr_RangeCheck(p : Pointer; index : SizeInt); compilerproc;

Procedure fpc_AnsiStr_SetLength (Var S : RawByteString; l : SizeInt{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
Function  fpc_ansistr_Copy (Const S : RawByteString; Index,Size : SizeInt): RawByteString;compilerproc{$ifndef VER3_0}:fpc_in_copy_x{$endif VER3_0};
{$ifndef VER3_0}
Procedure fpc_ansistr_insert (const Source : RawByteString; var S : RawByteString; Index : SizeInt); compilerproc:fpc_in_insert_x_y_z; rtlproc;
Procedure fpc_ansistr_delete (var S : RawByteString; Index,Size: SizeInt); compilerproc:fpc_in_delete_x_y_z; rtlproc;
{$endif VER3_0}
{$ifdef EXTRAANSISHORT}
Function fpc_AnsiStr_ShortStr_Compare (Var S1 : Pointer; Var S2 : ShortString): SizeInt; compilerproc;
{$endif EXTRAANSISHORT}
{ pointer argument because otherwise when calling this, we get
  an endless loop since a 'var s: ansistring' must be made
  unique as well                                               }
Function fpc_ansistr_Unique(Var S : Pointer): Pointer; compilerproc;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}

{*****************************************************************************
                        Widestring support
*****************************************************************************}

{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure fpc_WideStr_Decr_Ref (Var S : Pointer); compilerproc;
Procedure fpc_WideStr_Incr_Ref (Var S : Pointer); compilerproc;
procedure fpc_WideStr_To_ShortStr (out res: ShortString;const S2 : WideString); compilerproc;
Function fpc_ShortStr_To_WideStr (Const S2 : ShortString): WideString; compilerproc;
Function fpc_WideStr_To_AnsiStr (const S2 : WideString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}): AnsiString; compilerproc;
Function fpc_AnsiStr_To_WideStr (Const S2 : RawByteString): WideString; compilerproc;
Procedure fpc_WideStr_Assign (Var S1 : Pointer;S2 : Pointer); compilerproc;
Procedure fpc_WideStr_Concat (Var DestS : Widestring;const S1,S2 : WideString); compilerproc;
Procedure fpc_WideStr_Concat_multi (Var DestS : Widestring;const sarr:array of Widestring); compilerproc;
Function fpc_Char_To_WideStr(const c : Char): WideString; compilerproc;
Function fpc_PChar_To_WideStr(const p : pchar): WideString; compilerproc;
Function fpc_CharArray_To_WideStr(const arr: array of char; zerobased: boolean = true): WideString; compilerproc;
procedure fpc_widestr_to_chararray(out res: array of char; const src: WideString); compilerproc;
procedure fpc_widestr_to_widechararray(out res: array of widechar; const src: WideString); compilerproc;
Function fpc_WideStr_Compare(const S1,S2 : WideString): SizeInt; compilerproc;
Function fpc_WideStr_Compare_equal(const S1,S2 : WideString): SizeInt; compilerproc;
Procedure fpc_WideStr_RangeCheck(p: Pointer; index : SizeInt); compilerproc;

Procedure fpc_WideStr_SetLength (Var S : WideString; l : SizeInt); compilerproc;
Function  fpc_widestr_Copy (Const S : WideString; Index,Size : SizeInt) : WideString;compilerproc{$ifndef VER3_0}:fpc_in_copy_x{$endif VER3_0};
{$ifndef VER3_0}
Procedure fpc_widestr_insert (Const Source : WideString; Var S : WideString; Index : SizeInt); compilerproc:fpc_in_insert_x_y_z;
Procedure fpc_widestr_delete (Var S : WideString; Index,Size: SizeInt); compilerproc:fpc_in_delete_x_y_z;
{$endif VER3_0}
{$ifndef FPC_WINLIKEWIDESTRING}
function fpc_widestr_Unique(Var S : Pointer): Pointer; compilerproc;
{$endif FPC_WINLIKEWIDESTRING}
Function fpc_UChar_To_WideStr(const c : WideChar): WideString; compilerproc;
Function fpc_PWideChar_To_WideStr(const p : pwidechar): widestring; compilerproc;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
{$endif ndef FPC_WIDESTRING_EQUAL_UNICODESTRING}

{*****************************************************************************
                        Unicode string support
*****************************************************************************}


{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure fpc_UnicodeStr_Decr_Ref (Var S : Pointer); compilerproc;
Procedure fpc_UnicodeStr_Incr_Ref (S : Pointer); compilerproc;
procedure fpc_UnicodeStr_To_ShortStr (out res: ShortString;const S2 : UnicodeString); compilerproc;
Function fpc_ShortStr_To_UnicodeStr (Const S2 : ShortString): UnicodeString; compilerproc;
Function fpc_UnicodeStr_To_AnsiStr (const S2 : UnicodeString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}): AnsiString; compilerproc;
Function fpc_AnsiStr_To_UnicodeStr (Const S2 : RawByteString): UnicodeString; compilerproc;
Function fpc_UnicodeStr_To_WideStr (const S2 : UnicodeString): WideString; compilerproc;
Function fpc_WideStr_To_UnicodeStr (Const S2 : WideString): UnicodeString; compilerproc;
Procedure fpc_UnicodeStr_Assign (Var S1 : Pointer;S2 : Pointer); compilerproc;
Procedure fpc_UnicodeStr_Concat (Var DestS : Unicodestring;const S1,S2 : UnicodeString); compilerproc;
Procedure fpc_UnicodeStr_Concat_multi (Var DestS : Unicodestring;const sarr:array of Unicodestring); compilerproc;
Function fpc_Char_To_UnicodeStr(const c : Char): UnicodeString; compilerproc;
Function fpc_PChar_To_UnicodeStr(const p : pchar): UnicodeString; compilerproc;
Function fpc_CharArray_To_UnicodeStr(const arr: array of char; zerobased: boolean = true): UnicodeString; compilerproc;
procedure fpc_unicodestr_to_chararray(out res: array of char; const src: UnicodeString); compilerproc;
procedure fpc_WideCharArray_To_ShortStr(out res : shortstring;const arr: array of widechar; zerobased: boolean = true); compilerproc;
Function fpc_WideCharArray_To_AnsiStr(const arr: array of widechar; {$ifdef FPC_HAS_CPSTRING}cp : TSystemCodePage;{$endif FPC_HAS_CPSTRING} zerobased: boolean = true): AnsiString; compilerproc;
{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
Function fpc_WideCharArray_To_WideStr(const arr: array of widechar; zerobased: boolean = true): WideString; compilerproc;
{$endif}
Function fpc_WideCharArray_To_UnicodeStr(const arr: array of widechar; zerobased: boolean = true): UnicodeString; compilerproc;
procedure fpc_shortstr_to_widechararray(out res: array of widechar; const src: ShortString); compilerproc;
procedure fpc_ansistr_to_widechararray(out res: array of widechar; const src: RawByteString); compilerproc;
procedure fpc_unicodestr_to_widechararray(out res: array of widechar; const src: UnicodeString); compilerproc;
Function fpc_UnicodeStr_Compare(const S1,S2 : UnicodeString): SizeInt; compilerproc;
Function fpc_UnicodeStr_Compare_equal(const S1,S2 : UnicodeString): SizeInt; compilerproc;
Procedure fpc_UnicodeStr_RangeCheck(p: Pointer; index : SizeInt); compilerproc;

Procedure fpc_UnicodeStr_SetLength (Var S : UnicodeString; l : SizeInt); compilerproc;
Function  fpc_unicodestr_Copy (Const S : UnicodeString; Index,Size : SizeInt) : UnicodeString;compilerproc{$ifndef VER3_0}:fpc_in_copy_x{$endif VER3_0};
{$ifndef VER3_0}
Procedure fpc_unicodestr_insert (Const Source : UnicodeString; Var S : UnicodeString; Index : SizeInt); compilerproc:fpc_in_insert_x_y_z;
Procedure fpc_unicodestr_delete (Var S : UnicodeString; Index,Size: SizeInt); compilerproc:fpc_in_delete_x_y_z;
{$endif VER3_0}
function fpc_unicodestr_Unique(Var S : Pointer): Pointer; compilerproc;
Function fpc_Char_To_UChar(const c : Char): UnicodeChar; compilerproc;
Function fpc_UChar_To_Char(const c : UnicodeChar): Char; compilerproc;
Function fpc_UChar_To_UnicodeStr(const c : UnicodeChar): UnicodeString; compilerproc;
Function fpc_UChar_To_AnsiStr(const c : UnicodeChar{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}): AnsiString; compilerproc;
function fpc_UChar_To_ShortStr(const c : WideChar): shortstring; compilerproc;

Function fpc_PWideChar_To_UnicodeStr(const p : pwidechar): unicodestring; compilerproc;
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Function fpc_PWideChar_To_AnsiStr(const p : pwidechar{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}): ansistring; compilerproc;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
procedure fpc_PWideChar_To_ShortStr(out res : shortstring;const p : pwidechar); compilerproc;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}


{ $ifdef FPC_HAS_FEATURE_TEXTIO}
{ from text.inc }
Function fpc_get_input:PText;compilerproc;
Function fpc_get_output:PText;compilerproc;
Procedure fpc_textinit_iso(var t : Text;nr : DWord);compilerproc;
Procedure fpc_textinit_filename_iso(var t : Text;nr : DWord;const filename : string);compilerproc;
Procedure fpc_textclose_iso(var t : Text);compilerproc;
Procedure fpc_Write_End(var f:Text); compilerproc;
Procedure fpc_Writeln_End(var f:Text); compilerproc;
Procedure fpc_Write_Text_ShortStr(Len : Longint;var f : Text;const s : String); compilerproc;
Procedure fpc_Write_Text_ShortStr_Iso(Len : Longint;var f : Text;const s : String); compilerproc;
Procedure fpc_Write_Text_Pchar_as_Array(Len : Longint;var f : Text;const s : array of char; zerobased: boolean = true); compilerproc;
Procedure fpc_Write_Text_Pchar_as_Array_Iso(Len : Longint;var f : Text;const s : array of char; zerobased: boolean = true); compilerproc;
Procedure fpc_Write_Text_PChar_As_Pointer(Len : Longint;var f : Text;p : PChar); compilerproc;
Procedure fpc_Write_Text_AnsiStr (Len : Longint; Var f : Text; const S : RawByteString); compilerproc;
{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
Procedure fpc_Write_Text_WideStr (Len : Longint; Var f : Text; const S : WideString); compilerproc;
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure fpc_Write_Text_UnicodeStr (Len : Longint; Var f : Text; const S : UnicodeString); compilerproc;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
Procedure fpc_Write_Text_SInt(Len : Longint;var t : Text;l : ValSInt); compilerproc;
Procedure fpc_Write_Text_UInt(Len : Longint;var t : Text;l : ValUInt); compilerproc;
Procedure fpc_Write_Text_SInt_Iso(Len : Longint;var t : Text;l : ValSInt); compilerproc;
Procedure fpc_Write_Text_UInt_Iso(Len : Longint;var t : Text;l : ValUInt); compilerproc;
{$ifndef CPU64}
procedure fpc_write_text_qword(len : longint;var t : text;q : qword); compilerproc;
procedure fpc_write_text_int64(len : longint;var t : text;i : int64); compilerproc;
procedure fpc_write_text_qword_iso(len : longint;var t : text;q : qword); compilerproc;
procedure fpc_write_text_int64_iso(len : longint;var t : text;i : int64); compilerproc;
{$endif CPU64}
{$if defined(CPU16) or defined(CPU8)}
procedure fpc_write_text_longword(len : longint;var t : text;q : longword); compilerproc;
procedure fpc_write_text_longint(len : longint;var t : text;i : longint); compilerproc;
procedure fpc_write_text_longword_iso(len : longint;var t : text;q : longword); compilerproc;
procedure fpc_write_text_longint_iso(len : longint;var t : text;i : longint); compilerproc;

procedure fpc_write_text_word(len : longint;var t : text;q : word); compilerproc;
procedure fpc_write_text_smallint(len : longint;var t : text;i : smallint); compilerproc;
procedure fpc_write_text_word_iso(len : longint;var t : text;q : word); compilerproc;
procedure fpc_write_text_smallint_iso(len : longint;var t : text;i : smallint); compilerproc;
{$endif CPU16 or CPU8}
{$ifndef FPUNONE}
Procedure fpc_Write_Text_Float(rt,fixkomma,Len : Longint;var t : Text;r : ValReal); compilerproc;
Procedure fpc_Write_Text_Float_Iso(rt,fixkomma,Len : Longint;var t : Text;r : ValReal); compilerproc;
{$endif}
procedure fpc_write_text_enum(typinfo,ord2strindex:pointer;len:sizeint;var t:text;ordinal:longint); compilerproc;
Procedure fpc_Write_Text_Currency(fixkomma,Len : Longint;var t : Text;c : Currency); compilerproc;
Procedure fpc_Write_Text_Boolean(Len : Longint;var t : Text;b : Boolean); compilerproc;
Procedure fpc_Write_Text_Boolean_Iso(Len : Longint;var t : Text;b : Boolean); compilerproc;
Procedure fpc_Write_Text_Char(Len : Longint;var t : Text;c : Char); compilerproc;
Procedure fpc_Write_Text_Char_Iso(Len : Longint;var t : Text;c : Char); compilerproc;
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure fpc_Write_Text_WideChar(Len : Longint;var t : Text;c : WideChar); compilerproc;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}

{ all var rather than out, because they must not be trashed/finalized as they
  can appear inside the other arguments of writerstr }
procedure fpc_SetupWriteStr_Shortstr(var ReadWriteStrText: text; var s: shortstring); compilerproc;
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
procedure fpc_SetupWriteStr_Ansistr(var ReadWriteStrText: text; var s: ansistring; cp: TSystemCodePage); compilerproc;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
procedure fpc_SetupWriteStr_Unicodestr(var ReadWriteStrText: text; var s: unicodestring); compilerproc;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
procedure fpc_SetupWriteStr_Widestr(var ReadWriteStrText: text; var s: widestring); compilerproc;
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}

procedure fpc_SetupReadStr_Shortstr(var ReadWriteStrText: text; const s: shortstring); compilerproc;
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
procedure fpc_SetupReadStr_Ansistr(var ReadWriteStrText: text; const s: ansistring); compilerproc;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
procedure fpc_SetupReadStr_Unicodestr(var ReadWriteStrText: text; const s: unicodestring); compilerproc;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
procedure fpc_SetupReadStr_Widestr(var ReadWriteStrText: text; const s: widestring); compilerproc;
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}
{ $endif FPC_HAS_FEATURE_TEXTIO}

{$ifdef FPC_HAS_FEATURE_VARIANTS}
procedure fpc_variant_init(var v: tvardata);compilerproc;
procedure fpc_variant_clear(var v: tvardata);compilerproc;
{$ifdef FPC_VARIANTCOPY_FIXED}
procedure fpc_variant_copy(var d: tvardata; const s : tvardata);compilerproc;
procedure fpc_variant_copy_overwrite(constref source: tvardata; var dest : tvardata);compilerproc;
{$else FPC_VARIANTCOPY_FIXED}
procedure fpc_variant_copy(d,s : pointer);compilerproc;
procedure fpc_variant_copy_overwrite(source, dest : pointer);compilerproc;
{$endif FPC_VARIANTCOPY_FIXED}
procedure fpc_write_text_variant(Len : Longint;var f : Text;const v : variant); compilerproc;
function fpc_variant_to_dynarray(const v : variant;typeinfo : pointer) : fpc_stub_dynarray;compilerproc;
function fpc_dynarray_to_variant(dynarr : pointer;typeinfo : pointer) : variant;compilerproc;
function fpc_variant_to_interface(const v : variant) : iinterface;compilerproc;
function fpc_interface_to_variant(const i : iinterface) : variant;compilerproc;
function fpc_variant_to_idispatch(const v : variant) : idispatch;compilerproc;
function fpc_idispatch_to_variant(const i : idispatch) : variant;compilerproc;
procedure fpc_vararray_get(var d : variant;const s : variant;indices : plongint;len : sizeint);compilerproc;
procedure fpc_vararray_put(var d : variant;const s : variant;indices : plongint;len : sizeint);compilerproc;
procedure fpc_dispinvoke_variant(dest : pvardata;var source : tvardata;  calldesc : pcalldesc;params : pointer);compilerproc;
{$endif FPC_HAS_FEATURE_VARIANTS}

{$ifdef FPC_HAS_FEATURE_TEXTIO}
Procedure fpc_Read_End(var f:Text); compilerproc;
Procedure fpc_ReadLn_End(var f : Text); compilerproc;
Procedure fpc_ReadLn_End_Iso(var f : Text); compilerproc;
Procedure fpc_Read_Text_ShortStr(var f : Text;out s : String); compilerproc;
Procedure fpc_Read_Text_PChar_As_Pointer(var f : Text; const s : PChar); compilerproc;
Procedure fpc_Read_Text_PChar_As_Array(var f : Text;out s : array of char; zerobased: boolean = false); compilerproc;
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Procedure fpc_Read_Text_AnsiStr(var f : Text;out s : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); compilerproc;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure fpc_Read_Text_UnicodeStr(var f : Text;out us : UnicodeString); compilerproc;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
Procedure fpc_Read_Text_WideStr(var f : Text;out ws : WideString); compilerproc;
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}
Procedure fpc_Read_Text_Char(var f : Text; out c : char); compilerproc;
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
procedure fpc_Read_Text_WideChar(var f : Text; out wc: widechar); compilerproc;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
Procedure fpc_Read_Text_Char_Iso(var f : Text; out c : char); compilerproc;
Procedure fpc_Read_Text_SInt(var f : Text; out l :ValSInt); compilerproc;
Procedure fpc_Read_Text_SInt_Iso(var f : Text; out l : ValSInt); compilerproc;
Procedure fpc_Read_Text_UInt(var f : Text; out u :ValUInt); compilerproc;
Procedure fpc_Read_Text_UInt_Iso(var f : Text; out u : ValUInt); compilerproc;
{$ifndef FPUNONE}
Procedure fpc_Read_Text_Float(var f : Text; out v :ValReal); compilerproc;
Procedure fpc_Read_Text_Float_Iso(var f : Text; out v : ValReal); compilerproc;
{$endif}
procedure fpc_read_text_enum(str2ordindex:pointer;var t:text;out ordinal:longint); compilerproc;
procedure fpc_Read_Text_Currency(var f : Text; out v : Currency); compilerproc;
procedure fpc_Read_Text_Currency_Iso(var f : Text; out v : Currency); compilerproc;
{$ifndef CPU64}
Procedure fpc_Read_Text_QWord(var f : text; out q : qword); compilerproc;
procedure fpc_Read_Text_QWord_Iso(var f : text; out q : qword); compilerproc;
Procedure fpc_Read_Text_Int64(var f : text; out i : int64); compilerproc;
procedure fpc_Read_Text_Int64_Iso(var f : text; out i : int64); compilerproc;
{$endif CPU64}
{$if defined(CPU16) or defined(CPU8)}
Procedure fpc_Read_Text_LongWord(var f : text; out q : longword); compilerproc;
Procedure fpc_Read_Text_LongInt(var f : text; out i : longint); compilerproc;
{$endif CPU16 or CPU8}
function fpc_GetBuf_Text(var f : Text) : pchar; compilerproc;
function fpc_GetBuf_TypedFile(var f : TypedFile) : pointer; compilerproc;
{$endif FPC_HAS_FEATURE_TEXTIO}

{$ifdef FPC_INCLUDE_SOFTWARE_MOD_DIV}
function fpc_div_dword(n,z : dword) : dword; compilerproc;
function fpc_mod_dword(n,z : dword) : dword; compilerproc;
function fpc_div_longint(n,z : longint) : longint; compilerproc;
function fpc_mod_longint(n,z : longint) : longint; compilerproc;
function fpc_div_word(n,z : word) : word; compilerproc;
function fpc_mod_word(n,z : word) : word; compilerproc;
function fpc_div_smallint(n,z : smallint) : smallint; compilerproc;
function fpc_mod_smallint(n,z : smallint) : smallint; compilerproc;
function fpc_div_byte(n,z : byte) : byte; compilerproc;
function fpc_mod_byte(n,z : byte) : byte; compilerproc;
function fpc_div_shortint(n,z : shortint) : shortint; compilerproc;
function fpc_mod_shortint(n,z : shortint) : shortint; compilerproc;
{$endif FPC_INCLUDE_SOFTWARE_MOD_DIV}

{$ifdef FPC_INCLUDE_SOFTWARE_MUL}
{$ifdef VER3_0}
function fpc_mul_integer(f1,f2 : integer;checkoverflow : boolean) : integer; compilerproc;
function fpc_mul_word(f1,f2 : word;checkoverflow : boolean) : word; compilerproc;
function fpc_mul_longint(f1,f2 : longint;checkoverflow : boolean) : longint; compilerproc;
function fpc_mul_dword(f1,f2 : dword;checkoverflow : boolean) : dword; compilerproc;
{$else VER3_0}

function fpc_mul_shortint(f1,f2 : shortint) : shortint; compilerproc;
function fpc_mul_shortint_checkoverflow(f1,f2 : shortint) : shortint; compilerproc;
function fpc_mul_byte(f1,f2 : byte) : byte; compilerproc;
function fpc_mul_byte_checkoverflow(f1,f2 : byte) : byte; compilerproc;
function fpc_mul_integer(f1,f2 : integer) : integer; compilerproc;
function fpc_mul_integer_checkoverflow(f1,f2 : integer) : integer; compilerproc;
function fpc_mul_word(f1,f2 : word) : word; compilerproc;
function fpc_mul_word_checkoverflow(f1,f2 : word) : word; compilerproc;
function fpc_mul_longint(f1,f2 : longint) : longint; compilerproc;
function fpc_mul_longint_checkoverflow(f1,f2 : longint) : longint; compilerproc;
function fpc_mul_dword(f1,f2 : dword) : dword; compilerproc;
function fpc_mul_dword_checkoverflow(f1,f2 : dword) : dword; compilerproc;
{$endif VER3_0}
{$endif FPC_INCLUDE_SOFTWARE_MUL}

{ from int64.inc }
function fpc_div_qword(n,z : qword) : qword; compilerproc;
function fpc_mod_qword(n,z : qword) : qword; compilerproc;
function fpc_div_int64(n,z : int64) : int64; compilerproc;
function fpc_mod_int64(n,z : int64) : int64; compilerproc;
{$ifdef VER3_0}
function fpc_mul_qword(f1,f2 : qword;checkoverflow : longbool) : qword; compilerproc;
function fpc_mul_int64(f1,f2 : int64;checkoverflow : longbool) : int64; compilerproc;
{$else VER3_0}
function fpc_mul_qword(f1,f2 : qword) : qword; compilerproc;
function fpc_mul_qword_checkoverflow(f1,f2 : qword) : qword; compilerproc;
function fpc_mul_int64(f1,f2 : int64) : int64; compilerproc;
function fpc_mul_int64_checkoverflow(f1,f2 : int64) : int64; compilerproc;
{$endif VER3_0}
function fpc_mul_dword_to_qword(f1,f2 : dword) : qword; compilerproc;
function fpc_mul_longint_to_int64(f1,f2 : longint) : int64; compilerproc;

function fpc_div_currency(n,z : currency) : currency; compilerproc;
function fpc_mod_currency(n,z : currency) : currency; compilerproc;

{$ifdef FPC_INCLUDE_SOFTWARE_SHIFT_INT64}
function fpc_shl_qword(value : qword; shift : ALUUInt) : qword; compilerproc;
function fpc_shr_qword(value : qword; shift : ALUUInt) : qword; compilerproc;
function fpc_shl_int64(value : int64; shift : ALUUInt) : int64; compilerproc;
function fpc_shr_int64(value : int64; shift : ALUUInt) : int64; compilerproc;
procedure fpc_shl_assign_qword(var value : qword; shift : ALUUInt); compilerproc;
procedure fpc_shr_assign_qword(var value : qword; shift : ALUUInt); compilerproc;
procedure fpc_shl_assign_int64(var value : int64; shift : ALUUInt); compilerproc;
procedure fpc_shr_assign_int64(var value : int64; shift : ALUUInt); compilerproc;
{$endif  FPC_INCLUDE_SOFTWARE_SHIFT_INT64}
{$ifndef FPC_HAS_INTERNAL_SAR_ASSIGN_QWORD}
procedure fpc_sar_assign_int64(var AValue : Int64;const Shift : Byte);compilerproc;
procedure fpc_sar_assign_qword(var AValue : qword;const Shift : Byte);compilerproc;
{$endif FPC_HAS_INTERNAL_SAR_ASSIGN_QWORD}
{$ifndef FPC_HAS_INTERNAL_ROX_ASSIGN_QWORD}
procedure fpc_ror_assign_int64(var AValue : int64;const Dist : Byte);compilerproc;
procedure fpc_ror_assign_qword(var AValue : QWord;const Dist : Byte);compilerproc;
procedure fpc_rol_assign_int64(var AValue : int64;const Dist : Byte);compilerproc;
procedure fpc_rol_assign_qword(var AValue : QWord;const Dist : Byte);compilerproc;
{$endif FPC_HAS_INTERNAL_ROX_ASSIGN_QWORD}


function fpc_popcnt_byte(AValue : Byte): Byte;compilerproc;
function fpc_popcnt_word(AValue : Word): Word;compilerproc;
function fpc_popcnt_dword(AValue : DWord): DWord;compilerproc;
function fpc_popcnt_qword(AValue : QWord): QWord;compilerproc;


{$ifndef FPUNONE}
function fpc_abs_real(d : ValReal) : ValReal;compilerproc;
function fpc_arctan_real(d : ValReal) : ValReal;compilerproc;{$ifdef MATHINLINE}inline;{$endif}
function fpc_cos_real(d : ValReal) : ValReal;compilerproc;{$ifdef MATHINLINE}inline;{$endif}
function fpc_exp_real(d : ValReal) : ValReal;compilerproc;{$ifdef MATHINLINE}inline;{$endif}
function fpc_frac_real(d : ValReal) : ValReal;compilerproc;
function fpc_int_real(d : ValReal) : ValReal;compilerproc;{$ifdef MATHINLINE}inline;{$endif}
function fpc_ln_real(d : ValReal) : ValReal;compilerproc;{$ifdef MATHINLINE}inline;{$endif}
function fpc_sin_real(d : ValReal) : ValReal;compilerproc;{$ifdef MATHINLINE}inline;{$endif}
function fpc_sqr_real(d : ValReal) : ValReal;compilerproc;{$ifdef MATHINLINE}inline;{$endif}
function fpc_sqrt_real(d : ValReal) : ValReal;compilerproc;
function fpc_round_real(d : ValReal) : int64;compilerproc;
function fpc_trunc_real(d : ValReal) : int64;compilerproc;
{$endif}

{$ifdef FPC_HAS_FEATURE_CLASSES}
function fpc_do_is(aclass : tclass;aobject : tobject) : boolean; compilerproc;
function fpc_do_as(aclass : tclass;aobject : tobject): tobject; compilerproc;
procedure fpc_intf_decr_ref(var i: pointer); compilerproc;
procedure fpc_intf_incr_ref(i: pointer); compilerproc;
procedure fpc_intf_assign(var D: pointer; const S: pointer); compilerproc;
//procedure fpc_intf_assign_by_iid(var D: pointer; const S: pointer; const iid: TGUID); compilerproc;
function fpc_intf_is(const S: pointer; const iid: TGUID): Boolean; compilerproc;
function fpc_intf_is_class(const S: pointer; const aclass: tclass): Boolean; compilerproc;
function fpc_class_is_intf(const S: pointer; const iid: TGUID): Boolean; compilerproc;
function fpc_class_is_corbaintf(const S: pointer; const iid: Shortstring): Boolean; compilerproc;
function fpc_intf_cast(const S: pointer; const iid: TGUID): IInterface; compilerproc;
function fpc_intf_cast_class(const S: pointer; const aclass: tclass): pointer; compilerproc;
function fpc_class_cast_intf(const S: pointer; const iid: TGUID): IInterface; compilerproc;
function fpc_class_cast_corbaintf(const S: pointer; const iid: Shortstring): Pointer; compilerproc;
function fpc_intf_as(const S: pointer; const iid: TGUID): IInterface; compilerproc;
function fpc_intf_as_class(const S: pointer; const aclass: tclass): pointer; compilerproc;
function fpc_class_as_intf(const S: pointer; const iid: TGUID): IInterface; compilerproc;
function fpc_class_as_corbaintf(const S: pointer; const iid: Shortstring): Pointer; compilerproc;
{$ifdef FPC_HAS_FEATURE_VARIANTS}
procedure fpc_dispatch_by_id(Result: Pointer; const Dispatch: pointer;DispDesc: Pointer; Params: Pointer); compilerproc;
{$endif FPC_HAS_FEATURE_VARIANTS}
{$endif FPC_HAS_FEATURE_CLASSES}


{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
Function fpc_PushExceptAddr (Ft: {$ifdef CPU16}SmallInt{$else}Longint{$endif};_buf,_newaddr : pointer): PJmp_buf ; compilerproc;
procedure fpc_Raiseexception (Obj : TObject; AnAddr : CodePointer; AFrame : Pointer); compilerproc;
Procedure fpc_PopAddrStack; compilerproc;
function fpc_PopObjectStack : TObject; compilerproc;
function fpc_PopSecondObjectStack : TObject; compilerproc;
Procedure fpc_ReRaise; compilerproc;
Function fpc_Catches(Objtype : TClass) : TObject; compilerproc;
function fpc_safecallhandler(obj: TObject): HResult; compilerproc;
function fpc_safecallcheck(res : hresult) : hresult; compilerproc; {$ifdef CPU86} register; {$endif}
procedure fpc_doneexception; compilerproc;
procedure fpc_raise_nested; compilerproc;
{$endif FPC_HAS_FEATURE_EXCEPTIONS}


{$ifdef FPC_HAS_FEATURE_OBJECTS}
function fpc_help_constructor(_self:pointer;var _vmt:pointer;_vmt_pos:cardinal):pointer;compilerproc;
procedure fpc_help_destructor(_self,_vmt:pointer;vmt_pos:cardinal);compilerproc;
procedure fpc_help_fail(_self:pointer;var _vmt:pointer;vmt_pos:cardinal);compilerproc;

procedure fpc_check_object(_vmt:pointer); compilerproc;
procedure fpc_check_object_ext(vmt,expvmt:pointer);compilerproc;
{$endif FPC_HAS_FEATURE_OBJECTS}


{$ifdef FPC_HAS_FEATURE_RTTI}
Procedure fpc_Initialize (Data,TypeInfo : pointer); compilerproc;
Procedure fpc_finalize (Data,TypeInfo: Pointer); compilerproc;
Procedure fpc_Addref (Data,TypeInfo : Pointer); compilerproc;
Procedure fpc_DecRef (Data,TypeInfo : Pointer);  compilerproc;
procedure fpc_initialize_array(data,typeinfo : pointer;count : SizeInt); compilerproc;
procedure fpc_finalize_array(data,typeinfo : pointer;count : SizeInt); compilerproc;
procedure fpc_addref_array(data,typeinfo: pointer; count: SizeInt); compilerproc;
procedure fpc_decref_array(data,typeinfo: pointer; count: sizeint); compilerproc;
Function fpc_Copy (Src, Dest, TypeInfo : Pointer) : SizeInt; compilerproc;
Procedure fpc_Copy_proc (Src, Dest, TypeInfo : Pointer); compilerproc; inline;
{$endif FPC_HAS_FEATURE_RTTI}


{$ifdef FPC_SETBASE_USED}
procedure fpc_varset_load(const l;sourcesize : longint;var dest;size,srcminusdstbase : ptrint); compilerproc;
{$else}
procedure fpc_varset_load(const l;sourcesize : longint;var dest;size : ptrint); compilerproc;
{$endif}
procedure fpc_varset_create_element(b,size : ptrint; var data); compilerproc;
procedure fpc_varset_set(const source;var dest; b,size : ptrint); compilerproc;
procedure fpc_varset_unset(const source;var dest; b,size : ptrint); compilerproc;
procedure fpc_varset_set_range(const orgset; var dest;l,h,size : ptrint); compilerproc;
procedure fpc_varset_add_sets(const set1,set2; var dest;size : ptrint); compilerproc;
procedure fpc_varset_mul_sets(const set1,set2; var dest;size : ptrint); compilerproc;
procedure fpc_varset_sub_sets(const set1,set2; var dest;size : ptrint); compilerproc;
procedure fpc_varset_symdif_sets(const set1,set2; var dest;size : ptrint); compilerproc;
function fpc_varset_comp_sets(const set1,set2;size : ptrint):boolean; compilerproc;
function fpc_varset_contains_sets(const set1,set2;size : ptrint):boolean; compilerproc;

{$ifdef LARGESETS}
procedure fpc_largeset_set_word(p : pointer;b : word); compilerproc;
procedure fpc_largeset_in_word(p : pointer;b : word); compilerproc;
procedure fpc_largeset_add_sets(set1,set2,dest : pointer;size : longint); compilerproc;
procedure fpc_largeset_sets(set1,set2,dest : pointer;size : longint); compilerproc;
procedure fpc_largeset_sub_sets(set1,set2,dest : pointer;size : longint); compilerproc;
procedure fpc_largeset_symdif_sets(set1,set2,dest : pointer;size : longint); compilerproc;
procedure fpc_largeset_comp_sets(set1,set2 : pointer;size : longint); compilerproc;
procedure fpc_largeset_contains_sets(set1,set2 : pointer; size: longint); compilerproc;
{$endif LARGESETS}

procedure fpc_objecterror; compilerproc;
procedure fpc_rangeerror; compilerproc;
procedure fpc_divbyzero; compilerproc;
procedure fpc_overflow; compilerproc;
procedure fpc_iocheck; compilerproc;
{$ifndef FPC_NO_GENERIC_STACK_CHECK}
procedure fpc_stackcheck(stack_size:SizeUInt); compilerproc;
{$endif}

procedure fpc_InitializeUnits; compilerproc;
procedure fpc_LibInitializeUnits;compilerproc;
procedure fpc_do_exit;compilerproc;
// not generated by compiler, called directly in system unit
// procedure fpc_FinalizeUnits; compilerproc;

{
Procedure fpc_do_exit; compilerproc;
Procedure fpc_HandleErrorAddrFrame (Errno : longint;addr,frame : pointer); compilerproc;
}
Procedure fpc_lib_exit; compilerproc;
Procedure fpc_HandleError (Errno : TExitCode); compilerproc;

procedure fpc_AbstractErrorIntern;compilerproc;
procedure fpc_assert(Const Msg,FName:Shortstring;LineNo:Longint;ErrorAddr:Pointer); compilerproc;

{$ifdef FPC_HAS_FEATURE_FILEIO}
Procedure fpc_reset_typed(var f : TypedFile;Size : Longint); compilerproc;
Procedure fpc_rewrite_typed(var f : TypedFile;Size : Longint); compilerproc;
Procedure fpc_reset_typed_iso(var f : TypedFile;Size : Longint); compilerproc;
Procedure fpc_rewrite_typed_iso(var f : TypedFile;Size : Longint); compilerproc;
Procedure fpc_reset_typed_name_iso(var f : TypedFile;const FileName : String;Size : Longint); compilerproc;
Procedure fpc_rewrite_typed_name_iso(var f : TypedFile;const FileName : String;Size : Longint); compilerproc;

Procedure fpc_typed_write(TypeSize : Longint;var f : TypedFile;const Buf); compilerproc;
Procedure fpc_typed_read(TypeSize : Longint;var f : TypedFile;out Buf); compilerproc;
Procedure fpc_typed_read_iso(TypeSize : Longint;var f : TypedFile;out Buf); compilerproc;

Procedure fpc_typedfile_init_iso(var t : TypedFile;nr : DWord);compilerproc;
Procedure fpc_typedfile_init_filename_iso(var t : TypedFile;nr : DWord;const filename : string); compilerproc;
Procedure fpc_typedfile_close_iso(var t : TypedFile); compilerproc;

{$endif FPC_HAS_FEATURE_FILEIO}

{$ifdef FPC_INCLUDE_SOFTWARE_INT64_TO_DOUBLE}
function fpc_int64_to_double(i: int64): double; compilerproc;
function fpc_qword_to_double(q: qword): double; compilerproc;
{$endif FPC_INCLUDE_SOFTWARE_INT64_TO_DOUBLE}
{$ifdef FPC_INCLUDE_SOFTWARE_LONGWORD_TO_DOUBLE}
function fpc_longword_to_double(i: longword): double; compilerproc;
{$endif FPC_INCLUDE_SOFTWARE_LONGWORD_TO_DOUBLE}

function fpc_setjmp(var s : jmp_buf) : {$if defined(CPUINT8)}shortint{$elseif defined(CPUINT16)}smallint{$else}longint{$endif}; compilerproc;
procedure fpc_longjmp(var s : jmp_buf; value : {$if defined(CPUINT8)}shortint{$elseif defined(CPUINT16)}smallint{$else}longint{$endif}); compilerproc;

{$ifdef cpui8086}
{ i8086 stack checking }
procedure fpc_stackcheck_i8086; compilerproc;

{ i8086 huge pointer helpers }
function fpc_hugeptr_add_longint(p: HugePointer; n: LongInt): HugePointer; compilerproc;
function fpc_hugeptr_add_longint_normalized(p: HugePointer; n: LongInt): HugePointer; compilerproc;
function fpc_hugeptr_sub_longint(p: HugePointer; n: LongInt): HugePointer; compilerproc;
function fpc_hugeptr_sub_longint_normalized(p: HugePointer; n: LongInt): HugePointer; compilerproc;
function fpc_hugeptr_sub_hugeptr(p1, p2: HugePointer): LongInt; compilerproc;
procedure fpc_hugeptr_inc_longint(var p: HugePointer; n: LongInt); compilerproc;
procedure fpc_hugeptr_inc_longint_normalized(var p: HugePointer; n: LongInt); compilerproc;
procedure fpc_hugeptr_dec_longint(var p: HugePointer; n: LongInt); compilerproc;
procedure fpc_hugeptr_dec_longint_normalized(var p: HugePointer; n: LongInt); compilerproc;

function fpc_hugeptr_cmp_normalized_e(p1, p2: HugePointer): Boolean; compilerproc;
function fpc_hugeptr_cmp_normalized_ne(p1, p2: HugePointer): Boolean; compilerproc;
function fpc_hugeptr_cmp_normalized_b(p1, p2: HugePointer): Boolean; compilerproc;
function fpc_hugeptr_cmp_normalized_be(p1, p2: HugePointer): Boolean; compilerproc;
function fpc_hugeptr_cmp_normalized_a(p1, p2: HugePointer): Boolean; compilerproc;
function fpc_hugeptr_cmp_normalized_ae(p1, p2: HugePointer): Boolean; compilerproc;
{$endif cpui8086}

{$ifdef cpullvm}
{$i llvmintr.inc}
{$endif cpullvm}
