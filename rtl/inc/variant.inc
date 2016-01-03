{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by the Free Pascal development team

    This include file contains the implementation for variants
    support in FPC as far as it is part of the system unit

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

var
   variantmanager : tvariantmanager;

{ ---------------------------------------------------------------------
    Compiler helper routines.
  ---------------------------------------------------------------------}

procedure fpc_variant_init(var v : tvardata);[Public,Alias:'FPC_VARIANT_INIT'];compilerproc;
  begin
     { calling the variant manager here is a problem because the static/global variants
       are initialized while the variant manager isn't assigned }
     fillchar(v,sizeof(variant),0);
  end;


procedure fpc_variant_clear(var v : tvardata);[Public,Alias:'FPC_VARIANT_CLEAR'];compilerproc;
  begin
    if assigned(VarClearProc) then
      VarClearProc(v);
  end;

{ declare aliases for local use }
procedure variant_init(var v: tvardata); external name 'FPC_VARIANT_INIT';
procedure variant_clear(var v: tvardata); external name 'FPC_VARIANT_CLEAR';

procedure variant_addref(var v : tvardata);[Public,Alias:'FPC_VARIANT_ADDREF'];
  begin
    if assigned(VarAddRefProc) then
      VarAddRefProc(v);
  end;

{$ifdef FPC_VARIANTCOPY_FIXED}
procedure fpc_variant_copy(var d: tvardata; const s : tvardata);[Public,Alias:'FPC_VARIANT_COPY']; compilerproc;
  begin
    if assigned(VarCopyProc) then
      VarCopyProc(d,s);
  end;

procedure fpc_variant_copy_overwrite(constref source: tvardata; var dest : tvardata);[Public,Alias:'FPC_VARIANT_COPY_OVERWRITE']; compilerproc;
  begin
    dest.VType := varEmpty;
    if assigned(VarCopyProc) then
      VarCopyProc(dest,source);
  end;
{$else FPC_VARIANTCOPY_FIXED}
{ using pointers as argument here makes life for the compiler easier }
procedure fpc_variant_copy(d,s : pointer);[Public,Alias:'FPC_VARIANT_COPY']; compilerproc;
  begin
    if assigned(VarCopyProc) then
      VarCopyProc(tvardata(d^),tvardata(s^));
  end;

{ using pointers as argument here makes life for the compiler easier, overwrites target without finalizing }
procedure fpc_variant_copy_overwrite(source, dest : pointer);[Public,Alias:'FPC_VARIANT_COPY_OVERWRITE']; compilerproc;
  begin
    tvardata(dest^).VType := varEmpty;
    if assigned(VarCopyProc) then
      VarCopyProc(tvardata(dest^),tvardata(source^));
  end;
{$endif FPC_VARIANTCOPY_FIXED}

Procedure fpc_write_text_variant(Len : Longint;var f : Text;const v : variant); iocheck; [Public,Alias:'FPC_WRITE_TEXT_VARIANT']; compilerproc;
  begin
    if (InOutRes<>0) then
     exit;
    case TextRec(f).mode of
      { fmAppend gets changed to fmOutPut in do_open (JM) }
      fmOutput:
        if len=-1 then
          variantmanager.write0variant(f,v)
        else
          variantmanager.writevariant(f,v,len);
      fmInput:
        InOutRes:=105
      else InOutRes:=103;
    end;
  end;


procedure fpc_vararray_get(var d : variant;const s : variant;indices : plongint;len : sizeint);compilerproc;
  begin
    d:=variantmanager.vararrayget(s,len,indices);
  end;


procedure fpc_vararray_put(var d : variant;const s : variant;indices : plongint;len : sizeint);compilerproc;
  begin
    variantmanager.vararrayput(d,s,len,indices);
  end;


function fpc_variant_to_dynarray(const v : variant;typeinfo : pointer) : fpc_stub_dynarray;compilerproc;
  begin
    fpc_dynarray_clear(pointer(result),typeinfo);
    variantmanager.vartodynarray(pointer(result),v,typeinfo);
  end;


function fpc_dynarray_to_variant(dynarr : pointer;typeinfo : pointer) : variant;compilerproc;
  begin
    variantmanager.varfromdynarray(result,dynarr,typeinfo);
  end;


function fpc_variant_to_interface(const v : variant) : iinterface;compilerproc;
  begin
    variantmanager.vartointf(result,v);
  end;


function fpc_interface_to_variant(const i : iinterface) : variant;compilerproc;
  begin
    variantmanager.varfromintf(result,i);
  end;


function fpc_variant_to_idispatch(const v : variant) : idispatch;compilerproc;
  begin
    variantmanager.vartodisp(result,v);
  end;


function fpc_idispatch_to_variant(const i : idispatch) : variant;compilerproc;
  begin
    variantmanager.varfromdisp(result,i);
  end;


procedure fpc_dispinvoke_variant(dest : pvardata;var source : tvardata;
  calldesc : pcalldesc;params : pointer); compilerproc;
  begin
  	variantmanager.dispinvoke(dest,source,calldesc,params);
  end;


{ ---------------------------------------------------------------------
    Overloaded operators.
  ---------------------------------------------------------------------}


{ Integer }

operator :=(const source : byte) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  Variantmanager.varfromInt(Dest,Source,1);
end;


operator :=(const source : shortint) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  Variantmanager.varfromInt(Dest,Source,-1);
end;


operator :=(const source : word) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  Variantmanager.varfromInt(Dest,Source,2);
end;


operator :=(const source : smallint) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  Variantmanager.varfromInt(Dest,Source,-2);
end;


operator :=(const source : dword) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  Variantmanager.varfromInt(Dest,Source,4);
end;


operator :=(const source : longint) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  Variantmanager.varfromInt(Dest,Source,-4);
end;


operator :=(const source : qword) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  Variantmanager.varfromWord64(Dest,Source);
end;


operator :=(const source : int64) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  Variantmanager.varfromInt64(Dest,Source);
end;

{ Boolean }

operator :=(const source : boolean) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  Variantmanager.varfromBool(Dest,Source);
end;


operator :=(const source : wordbool) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  Variantmanager.varfromBool(Dest,Boolean(Source));
end;


operator :=(const source : longbool) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  Variantmanager.varfromBool(Dest,Boolean(Source));
end;


{ Chars }

operator :=(const source : char) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  VariantManager.VarFromPStr(Dest,Source);
end;


operator :=(const source : widechar) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
var
   ws : WideString;
begin
  ws:=source;
  Variantmanager.varfromwstr(Dest,ws);
end;

{ Strings }

operator :=(const source : shortstring) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  VariantManager.VarFromPStr(Dest,Source);
end;


operator :=(const source : ansistring) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  VariantManager.VarFromLStr(Dest,Source);
end;


operator :=(const source : widestring) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  VariantManager.VarFromWStr(Dest,Source);
end;


operator :=(const source : UTF8String) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  VariantManager.VarFromWStr(Dest,UTF8Decode(Source));
end;


operator :=(const source : UCS4String) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  VariantManager.VarFromWStr(Dest,UCS4StringToWideString(Source));
end;


{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
operator :=(const source : UnicodeString) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  VariantManager.VarFromWStr(Dest,Source);
end;
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}

{ Floats }

{$ifdef SUPPORT_SINGLE}
operator :=(const source : single) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  VariantManager.VarFromReal(Dest,Source);
end;
{$endif SUPPORT_SINGLE}


{$ifdef SUPPORT_DOUBLE}
operator :=(const source : double) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  VariantManager.VarFromReal(Dest,Source);
end;
{$endif SUPPORT_DOUBLE}


{$ifdef SUPPORT_EXTENDED}
operator :=(const source : extended) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  VariantManager.VarFromReal(Dest,Source);
end;
{$endif SUPPORT_EXTENDED}


{$ifdef SUPPORT_COMP}
Operator :=(const source : comp) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  VariantManager.VarFromReal(Dest,Source);
end;
{$endif SUPPORT_COMP}


{$ifndef FPUNONE}
Operator :=(const source : real) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  VariantManager.VarFromReal(Dest,Source);
end;
{$endif}


{ Misc. }
operator :=(const source : currency) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    VariantManager.VarFromCurr(Dest,Source);
  end;


{$ifndef FPUNONE}
operator :=(const source : tdatetime) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    VariantManager.VarFromTDateTime(Dest,Source);
  end;
{$endif}


operator :=(const source : terror) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Variantmanager.varfromInt(Dest,Source,-sizeof(terror));
  end;

{**********************************************************************
                       from Variant assignments
 **********************************************************************}

{ Integer }

operator :=(const source : variant) dest : byte;{$ifdef SYSTEMINLINE}inline;{$endif}

begin
  dest:=variantmanager.vartoint(source);
end;


operator :=(const source : variant) dest : shortint;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoint(source);
  end;


operator :=(const source : variant) dest : word;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoint(source);
  end;


operator :=(const source : variant) dest : smallint;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoint(source);
  end;


operator :=(const source : variant) dest : dword;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoint(source);
  end;


operator :=(const source : variant) dest : longint;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoint(source);
  end;


operator :=(const source : variant) dest : qword;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  dest:=variantmanager.vartoword64(source);
end;


operator :=(const source : variant) dest : int64;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  dest:=variantmanager.vartoint64(source);
end;


{ Boolean }

operator :=(const source : variant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}

begin
  dest:=variantmanager.vartobool(source);
end;


operator :=(const source : variant) dest : wordbool;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  dest:=variantmanager.vartobool(source);
end;


operator :=(const source : variant) dest : longbool;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   dest:=variantmanager.vartobool(source);
end;


{ Chars }

operator :=(const source : variant) dest : char;{$ifdef SYSTEMINLINE}inline;{$endif}
Var
  S : String;
begin
  VariantManager.VarToPStr(S,Source);
  If Length(S)>0 then
    Dest:=S[1];
end;


operator :=(const source : variant) dest : widechar;{$ifdef SYSTEMINLINE}inline;{$endif}
Var
  S : WideString;
begin
  VariantManager.vartowstr(S,Source);
  If Length(S)>0 then
    Dest:=S[1];
end;


{ Strings }

operator :=(const source : variant) dest : shortstring;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  VariantManager.VarToPStr(Dest,Source);
end;


operator :=(const source : variant) dest : ansistring;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  VariantManager.vartolstr(dest,source);
end;


operator :=(const source : variant) dest : widestring;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  variantmanager.vartowstr(dest,source);
end;


operator :=(const source : variant) dest : UTF8String;{$ifdef SYSTEMINLINE}inline;{$endif}
var
  temp : Widestring;
begin
  VariantManager.VarToWStr(temp,Source);
  dest:=UTF8Encode(temp);
end;


{$ifdef dummy}
operator :=(const source : variant) dest : UCS4String;{$ifdef SYSTEMINLINE}inline;{$endif}
var
  temp : Widestring;
begin
  VariantManager.VarToWStr(temp,Source);
  dest:=WideStringToUCS4String(temp);
end;
{$endif dummy}

{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
operator :=(const source : variant) dest : UnicodeString;{$ifdef SYSTEMINLINE}inline;{$endif}
var
  res : WideString;
begin
  variantmanager.vartowstr(res,source);
  dest:=res;
end;
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}


{ Floats }

{$ifdef SUPPORT_SINGLE}
operator :=(const source : variant) dest : single;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  dest:=variantmanager.vartoreal(source);
end;
{$endif SUPPORT_SINGLE}


{$ifdef SUPPORT_DOUBLE}
operator :=(const source : variant) dest : double;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  dest:=variantmanager.vartoreal(source);
end;
{$endif SUPPORT_DOUBLE}


{$ifdef SUPPORT_EXTENDED}
operator :=(const source : variant) dest : extended;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  dest:=variantmanager.vartoreal(source);
end;
{$endif SUPPORT_EXTENDED}


{$ifdef SUPPORT_COMP}
operator :=(const source : variant) dest : comp;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  dest:=comp(variantmanager.vartoreal(source));
end;
{$endif SUPPORT_COMP}


{$ifndef FPUNONE}
operator :=(const source : variant) dest : real;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  dest:=variantmanager.vartoreal(source);
end;
{$endif}

{ Misc. }
operator :=(const source : variant) dest : currency;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartocurr(source);
  end;


{$ifndef FPUNONE}
operator :=(const source : variant) dest : tdatetime;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartotdatetime(source);
  end;
{$endif}

operator :=(const source : variant) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.olevarfromvar(dest,source);
  end;


operator :=(const source : variant) dest : terror;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoint(source);
  end;
{**********************************************************************
                               Operators
 **********************************************************************}

operator or(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=op1;
     variantmanager.varop(dest,op2,opor);
  end;

operator and(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=op1;
     variantmanager.varop(dest,op2,opand);
  end;

operator xor(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=op1;
     variantmanager.varop(dest,op2,opxor);
  end;

operator not(const op : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=op;
     variantmanager.varnot(dest);
  end;

operator shl(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=op1;
     variantmanager.varop(dest,op2,opshiftleft);
  end;

operator shr(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=op1;
     variantmanager.varop(dest,op2,opshiftright);
  end;

operator +(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=op1;
     variantmanager.varop(dest,op2,opadd);
  end;

operator -(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=op1;
     variantmanager.varop(dest,op2,opsubtract);
  end;

operator *(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=op1;
     variantmanager.varop(dest,op2,opmultiply);
  end;

operator /(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=op1;
     variantmanager.varop(dest,op2,opdivide);
  end;

operator **(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=op1;
     variantmanager.varop(dest,op2,oppower);
  end;

operator div(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=op1;
     variantmanager.varop(dest,op2,opintdivide);
  end;

operator mod(const op1,op2 : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=op1;
     variantmanager.varop(dest,op2,opmodulus);
  end;

operator -(const op : variant) dest : variant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=op;
     variantmanager.varneg(dest);
  end;

operator =(const op1,op2 : variant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=variantmanager.cmpop(op1,op2,opcmpeq);
  end;

operator <(const op1,op2 : variant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=variantmanager.cmpop(op1,op2,opcmplt);
  end;

operator >(const op1,op2 : variant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=variantmanager.cmpop(op1,op2,opcmpgt);
  end;

operator >=(const op1,op2 : variant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=variantmanager.cmpop(op1,op2,opcmpge);
  end;

operator <=(const op1,op2 : variant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
     dest:=variantmanager.cmpop(op1,op2,opcmple);
  end;

procedure VarArrayRedim(var A: Variant; HighBound: SizeInt);
  begin
    variantmanager.vararrayredim(a,highbound);
  end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure VarArrayPut(var A: Variant; const Value: Variant; const Indices: array of Longint);
  begin
    if Length(Indices)>0 then
=======
procedure VarArrayPut(var A: Variant; const Value: Variant; const Indices: array of SizeInt);
  begin
    if Length(Indices)>0 then 
>>>>>>> graemeg/fixes_2_2
=======
procedure VarArrayPut(var A: Variant; const Value: Variant; const Indices: array of SizeInt);
  begin
    if Length(Indices)>0 then 
>>>>>>> origin/fixes_2_2
      variantmanager.vararrayput(A, Value, Length(Indices), @Indices[0])
    else
      variantmanager.vararrayput(A, Value, 0, nil);
  end;


<<<<<<< HEAD
<<<<<<< HEAD
function VarArrayGet(const A: Variant; const Indices: array of Longint): Variant;
  begin
    if Length(Indices)>0 then
      Result:=variantmanager.vararrayget(A, Length(Indices), @Indices[0])
    else
      Result:=variantmanager.vararrayget(A, 0, nil);
  end;


=======
=======
>>>>>>> origin/fixes_2_2
function VarArrayGet(const A: Variant; const Indices: array of SizeInt): Variant;
  begin
    if Length(Indices)>0 then 
      Result:=variantmanager.vararrayget(A, Length(Indices), @Indices[0])
    else 
      Result:=variantmanager.vararrayget(A, 0, nil);
  end;
  
 
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure VarCast(var dest : variant;const source : variant;vartype : longint);

  begin
    variantmanager.varcast(dest,source,vartype);
  end;


{**********************************************************************
                        from OLEVariant assignments
 **********************************************************************}
{ Integer }
operator :=(const source : olevariant) dest : byte;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    { cast away olevar to var conversion and avoid
      endless recursion }
    dest:=variantmanager.vartoint(variant(tvardata(source)));
  end;


operator :=(const source : olevariant) dest : shortint;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoint(variant(tvardata(source)));
  end;


operator :=(const source : olevariant) dest : word;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoint(variant(tvardata(source)));
  end;


operator :=(const source : olevariant) dest : smallint;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoint(variant(tvardata(source)));
  end;


operator :=(const source : olevariant) dest : dword;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoint(variant(tvardata(source)));
  end;


operator :=(const source : olevariant) dest : longint;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoint(variant(tvardata(source)));
  end;


operator :=(const source : olevariant) dest : qword;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoint64(variant(tvardata(source)));
  end;


operator :=(const source : olevariant) dest : int64;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoword64(variant(tvardata(source)));
  end;


{ Boolean }
operator :=(const source : olevariant) dest : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartobool(variant(tvardata(source)));
  end;


operator :=(const source : olevariant) dest : wordbool;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartobool(variant(tvardata(source)));
  end;


operator :=(const source : olevariant) dest : longbool;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartobool(variant(tvardata(source)));
  end;


{ Chars }
operator :=(const source : olevariant) dest : char;{$ifdef SYSTEMINLINE}inline;{$endif}
  var
    S : String;
  begin
    VariantManager.VarToPStr(S,Source);
    If Length(S)>0 then
      Dest:=S[1]
    else
      Dest:=#0;
  end;


operator :=(const source : olevariant) dest : widechar;{$ifdef SYSTEMINLINE}inline;{$endif}
  Var
    WS : WideString;
  begin
    VariantManager.VarToWStr(WS,Source);
    If Length(WS)>0 then
      Dest:=WS[1]
    else
      Dest:=#0;
  end;


{ Strings }
operator :=(const source : olevariant) dest : shortstring;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.vartopstr(dest,variant(tvardata(source)));
  end;


operator :=(const source : olevariant) dest : ansistring;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.vartolstr(dest,variant(tvardata(source)));
  end;


operator :=(const source : olevariant) dest : widestring;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.vartowstr(dest,variant(tvardata(source)));
  end;


{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
operator :=(const source : olevariant) dest : UnicodeString;{$ifdef SYSTEMINLINE}inline;{$endif}
  var
    res : WideString;
  begin
    variantmanager.vartowstr(res,variant(tvardata(source)));
    dest:=res;
  end;
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}

{ Floats }
{$ifdef SUPPORT_SINGLE}
operator :=(const source : olevariant) dest : single;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoreal(variant(tvardata(source)));
  end;
{$endif SUPPORT_SINGLE}


{$ifdef SUPPORT_DOUBLE}
operator :=(const source : olevariant) dest : double;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoreal(variant(tvardata(source)));
  end;
{$endif SUPPORT_DOUBLE}


{$ifdef SUPPORT_EXTENDED}
operator :=(const source : olevariant) dest : extended;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoreal(variant(tvardata(source)));
  end;
{$endif SUPPORT_EXTENDED}


{$ifdef SUPPORT_COMP}
operator :=(const source : olevariant) dest : comp;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPUNONE}
    dest:=comp(variantmanager.vartoint64(variant(tvardata(source))));
{$else}
    dest:=comp(variantmanager.vartoreal(variant(tvardata(source))));
{$endif}
=======
    dest:=comp(variantmanager.vartoreal(variant(tvardata(source))));
>>>>>>> graemeg/fixes_2_2
=======
    dest:=comp(variantmanager.vartoreal(variant(tvardata(source))));
>>>>>>> origin/fixes_2_2
  end;
{$endif SUPPORT_COMP}


{$ifndef FPUNONE}
operator :=(const source : olevariant) dest : real;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoreal(variant(tvardata(source)));
  end;
{$endif}

{ Misc. }
operator :=(const source : olevariant) dest : currency;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartocurr(variant(tvardata(source)));
  end;


{$ifndef FPUNONE}
operator :=(const source : olevariant) dest : tdatetime;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartotdatetime(variant(tvardata(source)));
  end;
{$endif}


operator :=(const source : olevariant) dest : terror;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    dest:=variantmanager.vartoint(variant(tvardata(source)));
  end;

{**********************************************************************
                          to OLEVariant assignments
 **********************************************************************}

operator :=(const source : byte) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.olevarfromint(dest,source,1);
  end;


operator :=(const source : shortint) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.olevarfromint(dest,source,-1);
  end;


operator :=(const source : word) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.olevarfromint(dest,source,2);
  end;


operator :=(const source : smallint) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.olevarfromint(dest,source,-2);
  end;


operator :=(const source : dword) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.olevarfromint(dest,source,4);
  end;


operator :=(const source : longint) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.olevarfromint(dest,source,-4);
  end;


operator :=(const source : qword) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.olevarfromint(dest,source,8);
  end;


operator :=(const source : int64) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.olevarfromint(dest,source,-8);
  end;

{ Boolean }
operator :=(const source : boolean) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.varfromBool(variant(tvardata(dest)),Source);
   end;


operator :=(const source : wordbool) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.varfromBool(variant(tvardata(Dest)),Source);
   end;


operator :=(const source : longbool) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.varfromBool(variant(tvardata(Dest)),Source);
   end;


{ Chars }
operator :=(const source : char) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.olevarfrompstr(dest,source);
  end;


operator :=(const source : widechar) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.varfromwstr(variant(tvardata(dest)),source);
  end;


{ Strings }
operator :=(const source : shortstring) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.olevarfrompstr(dest,source);
  end;


operator :=(const source : ansistring) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.olevarfromlstr(dest,source);
  end;


operator :=(const source : widestring) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.varfromwstr(variant(tvardata(dest)),source);
  end;


{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
operator :=(const source : UnicodeString) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.varfromwstr(variant(tvardata(dest)),source);
  end;
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}


{ Floats }
{$ifdef SUPPORT_SINGLE}
operator :=(const source : single) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.varfromreal(variant(tvardata(dest)),source);
  end;
{$endif SUPPORT_SINGLE}


{$ifdef SUPPORT_DOUBLE}
operator :=(const source : double) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.varfromreal(variant(tvardata(dest)),source);
  end;
{$endif SUPPORT_DOUBLE}


{$ifdef SUPPORT_EXTENDED}
operator :=(const source : extended) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.varfromreal(variant(tvardata(dest)),source);
  end;
{$endif SUPPORT_EXTENDED}


{$ifdef SUPPORT_COMP}
operator :=(const source : comp) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.varfromreal(variant(tvardata(dest)),source);
  end;
{$endif SUPPORT_COMP}


{$ifndef FPUNONE}
operator :=(const source : real) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.varfromreal(variant(tvardata(dest)),source);
  end;
{$endif}


{ Misc. }
operator :=(const source : currency) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.varfromcurr(variant(tvardata(dest)),source);
  end;


{$ifndef FPUNONE}
operator :=(const source : tdatetime) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.varfromtdatetime(variant(tvardata(dest)),source);
  end;
{$endif}


operator :=(const source : terror) dest : olevariant;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    variantmanager.olevarfromint(dest,source,-sizeof(terror));
  end;


function Unassigned: Variant; // Unassigned standard constant
begin
  VarClearProc(TVarData(Result));
  TVarData(Result).VType := varempty;
end;


function Null: Variant;       // Null standard constant
  begin
    VarClearProc(TVarData(Result));
    TVarData(Result).VType := varnull;
  end;

{**********************************************************************
                      Variant manager functions
 **********************************************************************}

procedure GetVariantManager(var VarMgr: TVariantManager);
begin
  VarMgr:=VariantManager;
end;

procedure SetVariantManager(const VarMgr: TVariantManager);
begin
  VariantManager:=VarMgr;
end;


Function Pos (c : Char; Const v : Variant) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=Pos(c,ShortString(v));
  end;


Function Pos (s : ShortString; Const v : Variant) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=Pos(s,ShortString(v));
  end;


Function Pos (const a : AnsiString; Const v : Variant) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=Pos(a,AnsiString(v));
  end;


Function Pos (const w : WideString; Const v : Variant) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=Pos(w,WideString(v));
  end;


{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
Function Pos (const w : UnicodeString; Const v : Variant) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=Pos(w,UnicodeString(v));
  end;
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}


Function Pos (const v : Variant; Const c : Char) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=Pos(ShortString(v),c);
  end;


Function Pos (const v : Variant; Const s : ShortString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=Pos(ShortString(v),s);
  end;


Function Pos (const v : Variant; Const a : AnsiString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=Pos(AnsiString(v),a);
  end;


Function Pos (const v : Variant; Const w : WideString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=Pos(WideString(v),w);
  end;


{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
Function Pos (const v : Variant; Const w : UnicodeString) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=Pos(UnicodeString(v),w);
  end;
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}


Function Pos (const v1 : Variant; Const v2 : Variant) : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=Pos(WideString(v1),WideString(v2));
  end;


