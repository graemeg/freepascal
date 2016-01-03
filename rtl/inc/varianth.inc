{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by the Free Pascal development team

    This include file contains the declarations for variants
    support in FPC

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ Variant types. Changes to these consts must be synchronized with
  similar list in compiler code, in implementation part of symdef.pas }
const
   varempty = 0;
   varnull = 1;
   varsmallint = 2;
   varinteger = 3;
{$ifndef FPUNONE}
   varsingle = 4;
   vardouble = 5;
   vardate = 7;
{$endif}
   varcurrency = 6;
   varolestr = 8;
   vardispatch = 9;
   varerror = 10;
   varboolean = 11;
   varvariant = 12;
   varunknown = 13;
   vardecimal = 14;
   varshortint = 16;
   varbyte = 17;
   varword = 18;
   varlongword = 19;
   varint64 = 20;
   varqword = 21;

   varrecord = 36;

   { The following values never appear as TVarData.VType, but are used in
     TCallDesc.Args[] as aliases for compiler-specific types.
     (since it provides only 1 byte per element, actual values won't fit)
     The choice of values is pretty much arbitrary. }

   varstrarg = $48;         { maps to varstring }
   varustrarg = $49;        { maps to varustring }

   { Compiler-specific variant types (not known to COM) are kept in
    'pseudo-custom' range of $100-$10E. Real custom types start with $10F. }

   varstring = $100;
   varany = $101;
   varustring = $102;
   vartypemask = $fff;
   vararray = $2000;
   varbyref = $4000;

   varword64 = varqword;
   varuint64 = varqword; // Delphi alias

type
   tvartype = word;

   pvararrayboundarray = ^tvararrayboundarray;
   pvararraycoorarray = ^tvararraycoorarray;
   pvararraybound = ^tvararraybound;
   pvararray = ^tvararray;

   tvararraybound = record
     elementcount,lowbound  : longint;
   end;

   tvararrayboundarray = array[0..0] of tvararraybound;
   tvararraycoorarray = array[0..0] of Longint;

   tvararray = record
      dimcount,flags : word;
      elementsize : longint;
      lockcount : longint;
      data : pointer;
      bounds : tvararrayboundarray;
   end;


   tvarop = (opadd,opsubtract,opmultiply,opdivide,opintdivide,opmodulus,
             opshiftleft,opshiftright,opand,opor,opxor,opcompare,opnegate,
             opnot,opcmpeq,opcmpne,opcmplt,opcmple,opcmpgt,opcmpge,oppower);

   tvardata = packed record
      vtype : tvartype;
      case integer of
         0:(res1 : word;
            case integer of
               0:
                 (res2,res3 : word;
                  case word of
                     varsmallint : (vsmallint : smallint);
                     varinteger : (vinteger : longint);
{$ifndef FPUNONE}
                     varsingle : (vsingle : single);
                     vardouble : (vdouble : double);
                     vardate : (vdate : tdatetime);
{$endif}
                     varcurrency : (vcurrency : currency);
                     varolestr : (volestr : pwidechar);
                     vardispatch : (vdispatch : pointer);
                     varerror : (verror : hresult);
                     varboolean : (vboolean : wordbool);
                     varunknown : (vunknown : pointer);
                     // vardecimal : ( : );
                     varshortint : (vshortint : shortint);
                     varbyte : (vbyte : byte);
                     varword : (vword : word);
                     varlongword : (vlongword : dword);
                     varint64 : (vint64 : int64);
                     varqword : (vqword : qword);
                     varword64 : (vword64 : qword);
                     varstring : (vstring : pointer);
                     varany :  (vany : pointer);
                     vararray : (varray : pvararray);
                     varbyref : (vpointer : pointer);
                     { unused so far, only to fill up space }
                     varrecord : (vrecord : pointer;precinfo : pointer);
                );
               1:
                 (vlongs : array[0..2] of longint);
           );
         1:(vwords : array[0..6] of word);
         2:(vbytes : array[0..13] of byte);
      end;
   pvardata = ^tvardata;

   pcalldesc = ^tcalldesc;
   tcalldesc = packed record
      calltype,argcount,namedargcount : byte;
      argtypes : array[0..255] of byte;
   end;

   pdispdesc = ^tdispdesc;
   tdispdesc = packed record
      dispid : longint;
      { not used by fpc }
      restype : byte;
      calldesc : tcalldesc;
   end;

   tvariantmanager = record
      vartoint : function(const v : variant) : longint;
      vartoint64 : function(const v : variant) : int64;
      vartoword64 : function(const v : variant) : qword;
      vartobool : function(const v : variant) : boolean;
{$ifndef FPUNONE}
      vartoreal : function(const v : variant) : extended;
      vartotdatetime : function(const v : variant) : tdatetime;
{$endif}
      vartocurr : function(const v : variant) : currency;
      vartopstr : procedure(var s ;const v : variant);
      vartolstr : procedure(var s : ansistring;const v : variant);
      vartowstr : procedure(var s : widestring;const v : variant);
      vartointf : procedure(var intf : iinterface;const v : variant);
      vartodisp : procedure(var disp : idispatch;const v : variant);
      vartodynarray : procedure(var dynarr : pointer;const v : variant;
         typeinfo : pointer);

      varfrombool : procedure(var dest : variant;const source : Boolean);
      varfromint : procedure(var dest : variant;const source,Range : longint);
      varfromint64 : procedure(var dest : variant;const source : int64);
      varfromword64 : procedure(var dest : variant;const source : qword);
{$ifndef FPUNONE}
      varfromreal : procedure(var dest : variant;const source : extended);
      varfromtdatetime : procedure(var dest : Variant;const source : TDateTime);
{$endif}
      varfromcurr : procedure(var dest : Variant;const source : Currency);
      varfrompstr: procedure(var dest : variant; const source : ShortString);
      varfromlstr: procedure(var dest : variant; const source : ansistring);
      varfromwstr: procedure(var dest : variant; const source : WideString);
      varfromintf: procedure(var dest : variant;const source : iinterface);
      varfromdisp: procedure(var dest : variant;const source : idispatch);
      varfromdynarray: procedure(var dest : variant;const source : pointer; typeinfo: pointer);
      olevarfrompstr: procedure(var dest : olevariant; const source : shortstring);
      olevarfromlstr: procedure(var dest : olevariant; const source : ansistring);
      olevarfromvar: procedure(var dest : olevariant; const source : variant);
      olevarfromint: procedure(var dest : olevariant; const source : longint;const range : shortint);

      { operators }
      varop : procedure(var left : variant;const right : variant;opcode : tvarop);
      cmpop : function(const left,right : variant;const opcode : tvarop) : boolean;
      varneg : procedure(var v : variant);
      varnot : procedure(var v : variant);

      { misc }
      varinit : procedure(var v : variant);
      varclear : procedure(var v : variant);
      varaddref : procedure(var v : variant);
      varcopy : procedure(var dest : variant;const source : variant);
      varcast : procedure(var dest : variant;const source : variant;vartype : longint);
      varcastole : procedure(var dest : variant; const source : variant;vartype : longint);

      dispinvoke: procedure(dest : pvardata;var source : tvardata;
        calldesc : pcalldesc;params : pointer);cdecl;

      vararrayredim : procedure(var a : variant;highbound : SizeInt);
      vararrayget : function(const a : variant;indexcount : SizeInt;indices : plongint) : variant;cdecl;
      vararrayput: procedure(var a : variant; const value : variant;
        indexcount : SizeInt;indices : plongint);cdecl;
      writevariant : function(var t : text;const v : variant;width : longint) : Pointer;
      write0Variant : function(var t : text;const v : Variant) : Pointer;
   end;
   pvariantmanager = ^tvariantmanager;

procedure GetVariantManager(var VarMgr: TVariantManager);
procedure SetVariantManager(const VarMgr: TVariantManager);

{ Global constants. Needed here only for compatibility. }

function Unassigned: Variant; // Unassigned standard constant
function Null: Variant;       // Null standard constant

const
  VarClearProc :  procedure(var v : TVarData) = nil;
  VarAddRefProc : procedure(var v : TVarData) = nil;
  VarCopyProc :   procedure(var d : TVarData;const s : TVarData) = nil;
  VarToLStrProc : procedure(var d : AnsiString;const s : TVarData) = nil;
  VarToWStrProc : procedure(var d : WideString;const s : TVarData) = nil;

{**********************************************************************
                       to Variant assignments
 **********************************************************************}

{ Integer }
operator :=(const source : byte) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : shortint) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : word) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : smallint) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : dword) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : longint) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : qword) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : int64) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}

{ Boolean }
operator :=(const source : boolean) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : wordbool) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : longbool) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}

{ Chars }
operator :=(const source : char) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : widechar) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}

{ Strings }
operator :=(const source : shortstring) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : ansistring) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : widestring) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : UTF8String) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : UCS4String) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
operator :=(const source : UnicodeString) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}

{ Floats }
{$ifdef SUPPORT_SINGLE}
operator :=(const source : single) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif SUPPORT_SINGLE}
{$ifdef SUPPORT_DOUBLE}
operator :=(const source : double) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif SUPPORT_DOUBLE}
{$ifdef SUPPORT_EXTENDED}
operator :=(const source : extended) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif SUPPORT_EXTENDED}
{$ifdef SUPPORT_COMP}
operator :=(const source : comp) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif SUPPORT_COMP}
{$ifndef FPUNONE}
operator :=(const source : real) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif}

{ Misc. }
operator :=(const source : currency) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$ifndef FPUNONE}
operator :=(const source : tdatetime) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif}
operator :=(const source : terror) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}

{**********************************************************************
                       from Variant assignments
 **********************************************************************}

{ Integer }
operator :=(const source : variant) dest : byte;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : variant) dest : shortint;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : variant) dest : word;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : variant) dest : smallint;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : variant) dest : dword;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : variant) dest : longint;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : variant) dest : qword;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : variant) dest : int64;{$ifdef SYSTEMINLINE}inline;{$endif}

{ Boolean }
operator :=(const source : variant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : variant) dest : wordbool;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : variant) dest : longbool;{$ifdef SYSTEMINLINE}inline;{$endif}

{ Chars }
operator :=(const source : variant) dest : char;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : variant) dest : widechar;{$ifdef SYSTEMINLINE}inline;{$endif}

{ Strings }
operator :=(const source : variant) dest : shortstring;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : variant) dest : ansistring;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : variant) dest : widestring;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : variant) dest : UTF8String;{$ifdef SYSTEMINLINE}inline;{$endif}
// operator :=(const source : variant) dest : UCS4String;{$ifdef SYSTEMINLINE}inline;{$endif}
{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
operator :=(const source : variant) dest : unicodestring;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}

{ Floats }
{$ifdef SUPPORT_SINGLE}
operator :=(const source : variant) dest : single;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif SUPPORT_SINGLE}
{$ifdef SUPPORT_DOUBLE}
operator :=(const source : variant) dest : double;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif SUPPORT_DOUBLE}
{$ifdef SUPPORT_EXTENDED}
operator :=(const source : variant) dest : extended;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif SUPPORT_EXTENDED}
{$ifdef SUPPORT_COMP}
operator :=(const source : variant) dest : comp;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif SUPPORT_COMP}
{$ifndef FPUNONE}
operator :=(const source : variant) dest : real;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif}

operator :=(const source : variant) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}

{ Misc. }
operator :=(const source : variant) dest : currency;{$ifdef SYSTEMINLINE}inline;{$endif}
{$ifndef FPUNONE}
operator :=(const source : variant) dest : tdatetime;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif}
operator :=(const source : variant) dest : terror;{$ifdef SYSTEMINLINE}inline;{$endif}

{**********************************************************************
                         Operators
 **********************************************************************}

operator or(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator and(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator xor(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator not(const op : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator shl(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator shr(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator +(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator -(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator *(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator /(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator **(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator div(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator mod(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator -(const op : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator =(const op1,op2 : variant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
operator <(const op1,op2 : variant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
operator >(const op1,op2 : variant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
operator >=(const op1,op2 : variant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
operator <=(const op1,op2 : variant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}

{ variant helpers }
procedure VarArrayRedim(var A: Variant; HighBound: SizeInt);
<<<<<<< HEAD
<<<<<<< HEAD
procedure VarArrayPut(var A: Variant; const Value: Variant; const Indices: array of Longint);
function VarArrayGet(const A: Variant; const Indices: array of Longint): Variant;
=======
procedure VarArrayPut(var A: Variant; const Value: Variant; const Indices: array of SizeInt);
function VarArrayGet(const A: Variant; const Indices: array of SizeInt): Variant;
>>>>>>> graemeg/fixes_2_2
=======
procedure VarArrayPut(var A: Variant; const Value: Variant; const Indices: array of SizeInt);
function VarArrayGet(const A: Variant; const Indices: array of SizeInt): Variant;
>>>>>>> origin/fixes_2_2
procedure VarCast(var dest : variant;const source : variant;vartype : longint);

{**********************************************************************
                        from OLEVariant assignments
 **********************************************************************}

{ Integer }
operator :=(const source : olevariant) dest : byte;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : olevariant) dest : shortint;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : olevariant) dest : word;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : olevariant) dest : smallint;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : olevariant) dest : dword;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : olevariant) dest : longint;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : olevariant) dest : qword;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : olevariant) dest : int64;{$ifdef SYSTEMINLINE}inline;{$endif}

{ Boolean }
operator :=(const source : olevariant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : olevariant) dest : wordbool;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : olevariant) dest : longbool;{$ifdef SYSTEMINLINE}inline;{$endif}

{ Chars }
operator :=(const source : olevariant) dest : char;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : olevariant) dest : widechar;{$ifdef SYSTEMINLINE}inline;{$endif}

{ Strings }
operator :=(const source : olevariant) dest : shortstring;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : olevariant) dest : ansistring;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : olevariant) dest : widestring;{$ifdef SYSTEMINLINE}inline;{$endif}
{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
operator :=(const source : olevariant) dest : UnicodeString;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}

{ Floats }
{$ifdef SUPPORT_SINGLE}
operator :=(const source : olevariant) dest : single;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif SUPPORT_SINGLE}
{$ifdef SUPPORT_DOUBLE}
operator :=(const source : olevariant) dest : double;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif SUPPORT_DOUBLE}
{$ifdef SUPPORT_EXTENDED}
operator :=(const source : olevariant) dest : extended;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif SUPPORT_EXTENDED}
{$ifdef SUPPORT_COMP}
operator :=(const source : olevariant) dest : comp;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif SUPPORT_COMP}
{$ifndef FPUNONE}
operator :=(const source : olevariant) dest : real;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif}

{ Misc. }
operator :=(const source : olevariant) dest : currency;{$ifdef SYSTEMINLINE}inline;{$endif}
{$ifndef FPUNONE}
operator :=(const source : olevariant) dest : tdatetime;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif}
operator :=(const source : olevariant) dest : terror;{$ifdef SYSTEMINLINE}inline;{$endif}

{**********************************************************************
                         to OLEVariant assignments
 **********************************************************************}

{ Integer }
operator :=(const source : byte) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : shortint) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : word) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : smallint) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : dword) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : longint) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : qword) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : int64) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}

{ Boolean }
operator :=(const source : boolean) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : wordbool) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : longbool) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}

{ Chars }
operator :=(const source : char) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : widechar) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}

{ Strings }
operator :=(const source : shortstring) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : ansistring) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator :=(const source : widestring) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
operator :=(const source : UnicodeString) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}

{ Floats }
{$ifdef SUPPORT_SINGLE}
operator :=(const source : single) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif SUPPORT_SINGLE}
{$ifdef SUPPORT_DOUBLE}
operator :=(const source : double) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif SUPPORT_DOUBLE}
{$ifdef SUPPORT_EXTENDED}
operator :=(const source : extended) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif SUPPORT_EXTENDED}
{$ifdef SUPPORT_COMP}
operator :=(const source : comp) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif SUPPORT_COMP}
{$ifndef FPUNONE}
operator :=(const source : real) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif}

{ Misc. }
operator :=(const source : currency) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$ifndef FPUNONE}
operator :=(const source : tdatetime) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif}
operator :=(const source : terror) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}

{ silly, but how else should the compiler know what to do with pos(<string type>,<variant>)? (FK) }
Function Pos (c : Char; Const v : Variant) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
Function Pos (s : ShortString; Const v : Variant) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
Function Pos (const a : AnsiString; Const v : Variant) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
Function Pos (const w : WideString; Const v : Variant) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
Function Pos (const w : UnicodeString; Const v : Variant) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}
Function Pos (const v : Variant; Const c : Char) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
Function Pos (const v : Variant; Const s : ShortString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
Function Pos (const v : Variant; Const a : AnsiString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
Function Pos (const v : Variant; Const w : WideString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
Function Pos (const v : Variant; Const w : UnicodeString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}
Function Pos (const v1 : Variant; Const v2 : Variant) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}

{**********************************************************************
                             OLEVariant Operators
 **********************************************************************}
(*
operator or(const op1,op2 : olevariant) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator and(const op1,op2 : olevariant) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator xor(const op1,op2 : olevariant) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator not(const op : olevariant) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator shl(const op1,op2 : olevariant) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator shr(const op1,op2 : olevariant) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator +(const op1,op2 : olevariant) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator -(const op1,op2 : olevariant) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator *(const op1,op2 : olevariant) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator /(const op1,op2 : olevariant) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator **(const op1,op2 : olevariant) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator div(const op1,op2 : olevariant) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator mod(const op1,op2 : olevariant) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator -(const op : olevariant) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
operator =(const op1,op2 : olevariant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
operator <(const op1,op2 : olevariant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
operator >(const op1,op2 : olevariant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
operator >=(const op1,op2 : olevariant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
operator <=(const op1,op2 : olevariant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
*)
