{
    Copyright (c) 1998-2006 by Florian Klaempfl

    This unit provides some help routines for type handling

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
unit defutil;

{$i fpcdefs.inc}

interface

    uses
       globtype,globals,constexp,
       symconst,symtype,symdef,
       cgbase,cpubase;

    type
       tmmxtype = (mmxno,mmxu8bit,mmxs8bit,mmxu16bit,mmxs16bit,
                   mmxu32bit,mmxs32bit,mmxfixed16,mmxsingle,mmxs64bit,mmxu64bit);


{*****************************************************************************
                          Basic type functions
 *****************************************************************************}

    {# Returns true, if definition defines an ordinal type }
    function is_ordinal(def : tdef) : boolean;

    {# Returns true, if definition defines a string type }
    function is_string(def : tdef): boolean;

    {# Returns True, if definition defines a type that behaves like a string,
       namely that can be joined and compared with another string-like type }
    function is_stringlike(def : tdef) : boolean;

    {# Returns True, if definition defines an enumeration type }
    function is_enum(def : tdef) : boolean;

    {# Returns True, if definition defines a set type }
    function is_set(def : tdef) : boolean;

    {# Returns the minimal integer value of the type }
    function get_min_value(def : tdef) : TConstExprInt;

    {# Returns the maximal integer value of the type }
    function get_max_value(def : tdef) : TConstExprInt;

    {# Returns basetype of the specified integer range }
    function range_to_basetype(const l,h:TConstExprInt):tordtype;

    procedure range_to_type(const l,h:TConstExprInt;var def:tdef);

    procedure int_to_type(const v:TConstExprInt;var def:tdef);

    {# Return true if the type (orddef or enumdef) spans its entire bitrange }
    function spans_entire_range(def: tdef): boolean;

    {# Returns true, if definition defines an integer type }
    function is_integer(def : tdef) : boolean;

    {# Returns true if definition is a boolean }
    function is_boolean(def : tdef) : boolean;

    {# Returns true if definition is a Pascal-style boolean (1 = true, zero = false) }
    function is_pasbool(def : tdef) : boolean;

    {# Returns true if definition is a C-style boolean (non-zero value = true, zero = false) }
    function is_cbool(def : tdef) : boolean;

    {# Returns true if definition is a char

       This excludes the unicode char.
    }
    function is_char(def : tdef) : boolean;

    {# Returns true if definition is a widechar }
    function is_widechar(def : tdef) : boolean;

    {# Returns true if definition is either an AnsiChar or a WideChar }
    function is_anychar(def : tdef) : boolean;

    {# Returns true if definition is a void}
    function is_void(def : tdef) : boolean;

    {# Returns true if definition is a smallset}
    function is_smallset(p : tdef) : boolean;

    {# Returns true, if def defines a signed data type
       (only for ordinal types)
    }
    function is_signed(def : tdef) : boolean;

    {# Returns an unsigned integer type of the same size as def; def must be
       an ordinal or enum }
    function get_unsigned_inttype(def: tdef): torddef;

    {# Returns whether def_from's range is comprised in def_to's if both are
      orddefs, false otherwise                                              }
    function is_in_limit(def_from,def_to : tdef) : boolean;

    {# Returns whether def is reference counted }
    function is_managed_type(def: tdef) : boolean;{$ifdef USEINLINE}inline;{$endif}

    { # Returns whether def is needs to load RTTI for reference counting }
    function is_rtti_managed_type(def: tdef) : boolean;

{    function is_in_limit_value(val_from:TConstExprInt;def_from,def_to : tdef) : boolean;}

{*****************************************************************************
                              Array helper functions
 *****************************************************************************}

    {# Returns true, if p points to a zero based (non special like open or
      dynamic array def).

      This is mainly used to see if the array
      is convertable to a pointer
    }
    function is_zero_based_array(p : tdef) : boolean;

    {# Returns true if p points to an open array definition }
    function is_open_array(p : tdef) : boolean;

    {# Returns true if p points to a dynamic array definition }
    function is_dynamic_array(p : tdef) : boolean;

    {# Returns true, if p points to an array of const definition }
    function is_array_constructor(p : tdef) : boolean;

    {# Returns true, if p points to a variant array }
    function is_variant_array(p : tdef) : boolean;

    {# Returns true, if p points to an array of const }
    function is_array_of_const(p : tdef) : boolean;

    {# Returns true, if p points any kind of special array

       That is if the array is an open array, a variant
       array, an array constants constructor, or an
       array of const.

       Bitpacked arrays aren't special in this regard though.
    }
    function is_special_array(p : tdef) : boolean;

    {# Returns true, if p points to a normal array, bitpacked arrays are included }
    function is_normal_array(p : tdef) : boolean;

    {# Returns true if p is a bitpacked array }
    function is_packed_array(p: tdef) : boolean;

    {# Returns true if p is a bitpacked record }
    function is_packed_record_or_object(p: tdef) : boolean;

    {# Returns true if p is a char array def }
    function is_chararray(p : tdef) : boolean;

    {# Returns true if p is a wide char array def }
    function is_widechararray(p : tdef) : boolean;

    {# Returns true if p is a open char array def }
    function is_open_chararray(p : tdef) : boolean;

    {# Returns true if p is a open wide char array def }
    function is_open_widechararray(p : tdef) : boolean;

{*****************************************************************************
                          String helper functions
 *****************************************************************************}

    {# Returns true if p points to an open string type }
    function is_open_string(p : tdef) : boolean;

    {# Returns true if p is an ansi string type }
    function is_ansistring(p : tdef) : boolean;

    {# Returns true if p is an ansi string type with codepage 0 }
    function is_rawbytestring(p : tdef) : boolean;

    {# Returns true if p is a long string type }
    function is_longstring(p : tdef) : boolean;

    {# returns true if p is a wide string type }
    function is_widestring(p : tdef) : boolean;

    {# true if p is an unicode string def }
    function is_unicodestring(p : tdef) : boolean;

    {# true if p is an unicode/wide/ansistring string def }
    function is_dynamicstring(p : tdef) : boolean;

    {# returns true if p is a wide or unicode string type }
    function is_wide_or_unicode_string(p : tdef) : boolean;

    {# Returns true if p is a short string type }
    function is_shortstring(p : tdef) : boolean;

    {# Returns true if p is any pointer def }
    function is_pointer(p : tdef) : boolean;

    {# Returns true if p is a pchar def }
    function is_pchar(p : tdef) : boolean;

    {# Returns true if p is a pwidechar def }
    function is_pwidechar(p : tdef) : boolean;

    {# Returns true if p is a voidpointer def }
    function is_voidpointer(p : tdef) : boolean;

    {# Returns true, if definition is a float }
    function is_fpu(def : tdef) : boolean;

    {# Returns true, if def is a currency type }
    function is_currency(def : tdef) : boolean;

    {# Returns true, if def is a single type }
    function is_single(def : tdef) : boolean;

    {# Returns true, if def is a double type }
    function is_double(def : tdef) : boolean;

    {# Returns true, if def is an extended type }
    function is_extended(def : tdef) : boolean;

    {# Returns true, if definition is a "real" real (i.e. single/double/extended) }
    function is_real(def : tdef) : boolean;

    {# Returns true for single,double,extended and cextended }
    function is_real_or_cextended(def : tdef) : boolean;

    { true, if def is a 8 bit int type }
    function is_8bitint(def : tdef) : boolean;

    { true, if def is a 8 bit ordinal type }
    function is_8bit(def : tdef) : boolean;

    { true, if def is a 16 bit int type }
    function is_16bitint(def : tdef) : boolean;

    { true, if def is a 16 bit ordinal type }
    function is_16bit(def : tdef) : boolean;

    {# Returns true, if def is a 32 bit integer type }
    function is_32bitint(def : tdef) : boolean;

    {# Returns true, if def is a 32 bit ordinal type }
    function is_32bit(def : tdef) : boolean;

    {# Returns true, if def is a 64 bit integer type }
    function is_64bitint(def : tdef) : boolean;

    {# Returns true, if def is a 64 bit type }
    function is_64bit(def : tdef) : boolean;

    { true, if def1 and def2 are both integers of the same bit size and sign }
    function are_equal_ints(def1, def2: tdef): boolean;

    { true, if def is an int type, larger than the processor's native int size }
    function is_oversizedint(def : tdef) : boolean;

    { true, if def is an ordinal type, larger than the processor's native int size }
    function is_oversizedord(def : tdef) : boolean;

    { true, if def is an int type, equal in size to the processor's native int size }
    function is_nativeint(def : tdef) : boolean;

    { true, if def is an ordinal type, equal in size to the processor's native int size }
    function is_nativeord(def : tdef) : boolean;

    { true, if def is an unsigned int type, equal in size to the processor's native int size }
    function is_nativeuint(def : tdef) : boolean;

    { true, if def is a signed int type, equal in size to the processor's native int size }
    function is_nativesint(def : tdef) : boolean;

  type
    tperformrangecheck = (
      rc_internal,  { nothing, internal conversion }
      rc_explicit,  { no, but this is an explcit user conversion and hence can still give warnings in some cases (or errors in case of enums) }
      rc_implicit,  { no, but this is an implicit conversion and hence can still give warnings/errors in some cases }
      rc_yes        { yes }
    );
    {# If @var(l) isn't in the range of todef a range check error (if not explicit) is generated and
      the value is placed within the range
    }
    procedure adaptrange(todef : tdef;var l : tconstexprint; rangecheck: tperformrangecheck);
    { for when used with nf_explicit/nf_internal/cs_check_range nodeflags }
    procedure adaptrange(todef : tdef;var l : tconstexprint; internal, explicit, rangecheckstate: boolean);

    {# Returns the range of def, where @var(l) is the low-range and @var(h) is
      the high-range.
    }
    procedure getrange(def : tdef;out l, h : TConstExprInt);
    procedure getrangedefmasksize(def: tdef; out rangedef: tdef; out mask: TConstExprInt; out size: longint);

    { Returns the range type of an ordinal type in the sense of ISO-10206 }
    function get_iso_range_type(def: tdef): tdef;

    { type being a vector? }
    function is_vector(p : tdef) : boolean;

    { some type helper routines for MMX support }
    function is_mmx_able_array(p : tdef) : boolean;

    {# returns the mmx type }
    function mmx_type(p : tdef) : tmmxtype;

    { returns if the passed type (array) fits into an mm register }
    function fits_in_mm_register(p : tdef) : boolean;

    {# From a definition return the abstract code generator size enum. It is
       to note that the value returned can be @var(OS_NO) }
    function def_cgsize(def: tdef): tcgsize;

    { #Return an orddef (integer) correspondig to a tcgsize }
    function cgsize_orddef(size: tcgsize): torddef;

    {# Same as def_cgsize, except that it will interpret certain arrays as
       vectors and return OS_M* sizes for them }
    function def_cgmmsize(def: tdef): tcgsize;

    {# returns true, if the type passed is can be used with windows automation }
    function is_automatable(p : tdef) : boolean;

    { # returns true if the procdef has no parameters and no specified return type }
    function is_bareprocdef(pd : tprocdef): boolean;

    { returns true if the procdef is a C-style variadic function }
    function is_c_variadic(pd: tabstractprocdef): boolean; {$ifdef USEINLINE}inline;{$endif}

    { # returns the smallest base integer type whose range encompasses that of
        both ld and rd; if keep_sign_if_equal, then if ld and rd have the same
        signdness, the result will also get that signdness }
    function get_common_intdef(ld, rd: torddef; keep_sign_if_equal: boolean): torddef;

    { # returns whether the type is potentially a valid type of/for an "univ" parameter
        (basically: it must have a compile-time size) }
    function is_valid_univ_para_type(def: tdef): boolean;

    { # returns whether the procdef/procvardef represents a nested procedure
        or not }
    function is_nested_pd(def: tabstractprocdef): boolean;{$ifdef USEINLINE}inline;{$endif}

    { # returns whether def is a type parameter of a generic }
    function is_typeparam(def : tdef) : boolean;{$ifdef USEINLINE}inline;{$endif}

    { returns true of def is a methodpointer }
    function is_methodpointer(def : tdef) : boolean;

    { returns true if def is a C "block" }
    function is_block(def: tdef): boolean;

    { returns the TTypeKind value of the def }
    function get_typekind(def: tdef): byte;

implementation

    uses
       verbose,cutils;

    { returns true, if def uses FPU }
    function is_fpu(def : tdef) : boolean;
      begin
         is_fpu:=(def.typ=floatdef);
      end;


    { returns true, if def is a currency type }
    function is_currency(def : tdef) : boolean;
      begin
         case s64currencytype.typ of
           orddef :
             result:=(def.typ=orddef) and
                     (torddef(s64currencytype).ordtype=torddef(def).ordtype);
           floatdef :
             result:=(def.typ=floatdef) and
                     (tfloatdef(s64currencytype).floattype=tfloatdef(def).floattype);
           else
             internalerror(200304222);
         end;
      end;


    { returns true, if def is a single type }
    function is_single(def : tdef) : boolean;
      begin
        result:=(def.typ=floatdef) and
          (tfloatdef(def).floattype=s32real);
      end;


    { returns true, if def is a double type }
    function is_double(def : tdef) : boolean;
      begin
        result:=(def.typ=floatdef) and
          (tfloatdef(def).floattype=s64real);
      end;


    function is_extended(def : tdef) : boolean;
      begin
        result:=(def.typ=floatdef) and
          (tfloatdef(def).floattype in [s80real,sc80real]);
      end;


    { returns true, if definition is a "real" real (i.e. single/double/extended) }
    function is_real(def : tdef) : boolean;
      begin
        result:=(def.typ=floatdef) and
          (tfloatdef(def).floattype in [s32real,s64real,s80real]);
      end;


    function is_real_or_cextended(def: tdef): boolean;
      begin
        result:=(def.typ=floatdef) and
          (tfloatdef(def).floattype in [s32real,s64real,s80real,sc80real]);
      end;


    function range_to_basetype(const l,h:TConstExprInt):tordtype;
      begin
        { prefer signed over unsigned }
        if (l>=int64(-128)) and (h<=127) then
         range_to_basetype:=s8bit
        else if (l>=0) and (h<=255) then
         range_to_basetype:=u8bit
        else if (l>=int64(-32768)) and (h<=32767) then
         range_to_basetype:=s16bit
        else if (l>=0) and (h<=65535) then
         range_to_basetype:=u16bit
        else if (l>=int64(low(longint))) and (h<=high(longint)) then
         range_to_basetype:=s32bit
        else if (l>=low(cardinal)) and (h<=high(cardinal)) then
         range_to_basetype:=u32bit
        else if (l>=low(int64)) and (h<=high(int64)) then
         range_to_basetype:=s64bit
        else
         range_to_basetype:=u64bit;
      end;


    procedure range_to_type(const l,h:TConstExprInt;var def:tdef);
      begin
        { prefer signed over unsigned }
        if (l>=int64(-128)) and (h<=127) then
         def:=s8inttype
        else if (l>=0) and (h<=255) then
         def:=u8inttype
        else if (l>=int64(-32768)) and (h<=32767) then
         def:=s16inttype
        else if (l>=0) and (h<=65535) then
         def:=u16inttype
        else if (l>=int64(low(longint))) and (h<=high(longint)) then
         def:=s32inttype
        else if (l>=low(cardinal)) and (h<=high(cardinal)) then
         def:=u32inttype
        else if (l>=low(int64)) and (h<=high(int64)) then
         def:=s64inttype
        else
         def:=u64inttype;
      end;


    procedure int_to_type(const v:TConstExprInt;var def:tdef);
      begin
        range_to_type(v,v,def);
      end;


    { true if p is an ordinal }
    function is_ordinal(def : tdef) : boolean;
      var
         dt : tordtype;
      begin
         case def.typ of
           orddef :
             begin
               dt:=torddef(def).ordtype;
               is_ordinal:=dt in [uchar,uwidechar,
                                  u8bit,u16bit,u32bit,u64bit,
                                  s8bit,s16bit,s32bit,s64bit,
                                  pasbool1,pasbool8,pasbool16,pasbool32,pasbool64,
                                  bool8bit,bool16bit,bool32bit,bool64bit,customint];
             end;
           enumdef :
             is_ordinal:=true;
           else
             is_ordinal:=false;
         end;
      end;

    { true if p is a string }
    function is_string(def : tdef) : boolean;
      begin
        is_string := (assigned(def) and (def.typ = stringdef));
      end;

    function is_stringlike(def : tdef) : boolean;
      begin
        result := is_string(def) or
                  is_anychar(def) or
                  is_pchar(def) or
                  is_pwidechar(def) or
                  is_chararray(def) or
                  is_widechararray(def) or
                  is_open_chararray(def) or
                  is_open_widechararray(def) or
                  (def=java_jlstring);
      end;

    function is_enum(def : tdef) : boolean;
      begin
        result:=def.typ=enumdef;
      end;

    function is_set(def : tdef) : boolean;
      begin
        result:=def.typ=setdef;
      end;

    { returns the min. value of the type }
    function get_min_value(def : tdef) : TConstExprInt;
      begin
         case def.typ of
           orddef:
             result:=torddef(def).low;
           enumdef:
             result:=int64(tenumdef(def).min);
           else
             result:=0;
         end;
      end;


    { returns the max. value of the type }
    function get_max_value(def : tdef) : TConstExprInt;
      begin
         case def.typ of
           orddef:
             result:=torddef(def).high;
           enumdef:
             result:=tenumdef(def).max;
           else
             result:=0;
         end;
      end;


    function spans_entire_range(def: tdef): boolean;
      var
         lv, hv: Tconstexprint;
         mask: qword;
         size: longint;
      begin
        case def.typ of
          orddef,
          enumdef:
            getrange(def,lv,hv);
          else
            internalerror(2019062203);
        end;
        size:=def.size;
        case size of
          1: mask:=$ff;
          2: mask:=$ffff;
          4: mask:=$ffffffff;
          8: mask:=qword(-1);
          else
            internalerror(2019062204);
        end;
        result:=false;
        if is_signed(def) then
          begin
            if (lv.uvalue and mask)<>(qword(1) shl (size*8-1)) then
              exit;
            if (hv.uvalue and mask)<>(mask shr 1) then
              exit;
          end
        else
          begin
            if lv<>0 then
              exit;
            if hv.uvalue<>mask then
              exit;
          end;
        result:=true;
      end;


    { true if p is an integer }
    function is_integer(def : tdef) : boolean;
      begin
        result:=(def.typ=orddef) and
                    (torddef(def).ordtype in [u8bit,u16bit,u32bit,u64bit,
                                          s8bit,s16bit,s32bit,s64bit,
                                          customint]);
      end;


    { true if p is a boolean }
    function is_boolean(def : tdef) : boolean;
      begin
        result:=(def.typ=orddef) and
                    (torddef(def).ordtype in [pasbool1,pasbool8,pasbool16,pasbool32,pasbool64,bool8bit,bool16bit,bool32bit,bool64bit]);
      end;


    function is_pasbool(def : tdef) : boolean;
      begin
        result:=(def.typ=orddef) and
                    (torddef(def).ordtype in [pasbool1,pasbool8,pasbool16,pasbool32,pasbool64]);
      end;

    { true if def is a C-style boolean (non-zero value = true, zero = false) }
    function is_cbool(def : tdef) : boolean;
      begin
        result:=(def.typ=orddef) and
                    (torddef(def).ordtype in [bool8bit,bool16bit,bool32bit,bool64bit]);
      end;


    { true if p is a void }
    function is_void(def : tdef) : boolean;
      begin
        result:=(def.typ=orddef) and
                 (torddef(def).ordtype=uvoid);
      end;


    { true if p is a char }
    function is_char(def : tdef) : boolean;
      begin
        result:=(def.typ=orddef) and
                 (torddef(def).ordtype=uchar);
      end;


    { true if p is a wchar }
    function is_widechar(def : tdef) : boolean;
      begin
        result:=(def.typ=orddef) and
                 (torddef(def).ordtype=uwidechar);
      end;


    { true if p is a char or wchar }
    function is_anychar(def : tdef) : boolean;
      begin
        result:=(def.typ=orddef) and
                 (torddef(def).ordtype in [uchar,uwidechar])
      end;


    { true if p is signed (integer) }
    function is_signed(def : tdef) : boolean;
      begin
         case def.typ of
           orddef :
             result:=torddef(def).low < 0;
           enumdef :
             result:=tenumdef(def).min < 0;
           arraydef :
             result:=is_signed(tarraydef(def).rangedef);
           else
             result:=false;
         end;
      end;


    function get_unsigned_inttype(def: tdef): torddef;
      begin
        case def.typ of
          orddef,
          enumdef:
            result:=cgsize_orddef(tcgsize2unsigned[def_cgsize(def)]);
          else
            internalerror(2016062001);
        end;
      end;


    function is_in_limit(def_from,def_to : tdef) : boolean;

      begin
         if (def_from.typ<>def_to.typ) or
            not(def_from.typ in [orddef,enumdef,setdef]) then
           begin
             is_in_limit := false;
             exit;
           end;
         case def_from.typ of
           orddef:
             is_in_limit:=(torddef(def_from).low>=torddef(def_to).low) and
                          (torddef(def_from).high<=torddef(def_to).high);
           enumdef:
             is_in_limit:=(tenumdef(def_from).min>=tenumdef(def_to).min) and
                          (tenumdef(def_from).max<=tenumdef(def_to).max);
           setdef:
             is_in_limit:=(tsetdef(def_from).setbase>=tsetdef(def_to).setbase) and
                          (tsetdef(def_from).setmax<=tsetdef(def_to).setmax);
         else
           is_in_limit:=false;
         end;
      end;


    function is_managed_type(def: tdef): boolean;{$ifdef USEINLINE}inline;{$endif}
      begin
        result:=def.needs_inittable;
      end;


    function is_rtti_managed_type(def: tdef): boolean;
      begin
        result:=def.needs_inittable and not (
          is_interfacecom_or_dispinterface(def) or
          (def.typ=variantdef) or
          (
            (def.typ=stringdef) and
            (tstringdef(def).stringtype in [st_ansistring,st_widestring,st_unicodestring])
          )
        );
      end;


    { true, if p points to an open array def }
    function is_open_string(p : tdef) : boolean;
      begin
         is_open_string:=(p.typ=stringdef) and
                         (tstringdef(p).stringtype=st_shortstring) and
                         (tstringdef(p).len=0);
      end;


    { true, if p points to a zero based array def }
    function is_zero_based_array(p : tdef) : boolean;
      begin
         result:=(p.typ=arraydef) and
                 (tarraydef(p).lowrange=0) and
                 not(is_special_array(p));
      end;

    { true if p points to a dynamic array def }
    function is_dynamic_array(p : tdef) : boolean;
      begin
         result:=(p.typ=arraydef) and
                 (ado_IsDynamicArray in tarraydef(p).arrayoptions);
      end;


    { true, if p points to an open array def }
    function is_open_array(p : tdef) : boolean;
      begin
         { check for sizesinttype is needed, because for unsigned the high
           range is also -1 ! (PFV) }
         result:=(p.typ=arraydef) and
                 (tarraydef(p).rangedef=sizesinttype) and
                 (tarraydef(p).lowrange=0) and
                 (tarraydef(p).highrange=-1) and
                 ((tarraydef(p).arrayoptions * [ado_IsVariant,ado_IsArrayOfConst,ado_IsConstructor,ado_IsDynamicArray])=[]);
      end;

    { true, if p points to an array of const def }
    function is_array_constructor(p : tdef) : boolean;
      begin
         result:=(p.typ=arraydef) and
                 (ado_IsConstructor in tarraydef(p).arrayoptions);
      end;

    { true, if p points to a variant array }
    function is_variant_array(p : tdef) : boolean;
      begin
         result:=(p.typ=arraydef) and
                 (ado_IsVariant in tarraydef(p).arrayoptions);
      end;

    { true, if p points to an array of const }
    function is_array_of_const(p : tdef) : boolean;
      begin
         result:=(p.typ=arraydef) and
                 (ado_IsArrayOfConst in tarraydef(p).arrayoptions);
      end;

    { true, if p points to a special array, bitpacked arrays aren't special in this regard though }
    function is_special_array(p : tdef) : boolean;
      begin
         result:=(p.typ=arraydef) and
                 (
                  ((tarraydef(p).arrayoptions * [ado_IsVariant,ado_IsArrayOfConst,ado_IsConstructor,ado_IsDynamicArray])<>[]) or
                  is_open_array(p)
                 );
      end;

    { true, if p points to a normal array, bitpacked arrays are included }
    function is_normal_array(p : tdef) : boolean;
      begin
         result:=(p.typ=arraydef) and
                 ((tarraydef(p).arrayoptions * [ado_IsVariant,ado_IsArrayOfConst,ado_IsConstructor,ado_IsDynamicArray])=[]) and
                 not(is_open_array(p));
      end;

    { true if p is an ansi string def }
    function is_ansistring(p : tdef) : boolean;
      begin
         is_ansistring:=(p.typ=stringdef) and
                        (tstringdef(p).stringtype=st_ansistring);
      end;

    { true if p is an ansi string def with codepage CP_NONE }
    function is_rawbytestring(p : tdef) : boolean;
      begin
        is_rawbytestring:=(p.typ=stringdef) and
                       (tstringdef(p).stringtype=st_ansistring) and
                       (tstringdef(p).encoding=globals.CP_NONE);
      end;

    { true if p is an long string def }
    function is_longstring(p : tdef) : boolean;
      begin
         is_longstring:=(p.typ=stringdef) and
                        (tstringdef(p).stringtype=st_longstring);
      end;


    { true if p is an wide string def }
    function is_widestring(p : tdef) : boolean;
      begin
         is_widestring:=(p.typ=stringdef) and
                        (tstringdef(p).stringtype=st_widestring);
      end;


    function is_dynamicstring(p: tdef): boolean;
      begin
         is_dynamicstring:=(p.typ=stringdef) and
                        (tstringdef(p).stringtype in [st_ansistring,st_widestring,st_unicodestring]);
      end;


    { true if p is an wide string def }
    function is_wide_or_unicode_string(p : tdef) : boolean;
      begin
         is_wide_or_unicode_string:=(p.typ=stringdef) and
                        (tstringdef(p).stringtype in [st_widestring,st_unicodestring]);
      end;


    { true if p is an unicode string def }
    function is_unicodestring(p : tdef) : boolean;
      begin
         is_unicodestring:=(p.typ=stringdef) and
                        (tstringdef(p).stringtype=st_unicodestring);
      end;


    { true if p is an short string def }
    function is_shortstring(p : tdef) : boolean;
      begin
         is_shortstring:=(p.typ=stringdef) and
                         (tstringdef(p).stringtype=st_shortstring);
      end;


    { true if p is bit packed array def }
    function is_packed_array(p: tdef) : boolean;
      begin
        is_packed_array :=
           (p.typ = arraydef) and
           (ado_IsBitPacked in tarraydef(p).arrayoptions);
      end;


    { true if p is bit packed record def }
    function is_packed_record_or_object(p: tdef) : boolean;
      begin
        is_packed_record_or_object :=
           (p.typ in [recorddef,objectdef]) and
           (tabstractrecorddef(p).is_packed);
      end;


    { true if p is a char array def }
    function is_chararray(p : tdef) : boolean;
      begin
        is_chararray:=(p.typ=arraydef) and
                      is_char(tarraydef(p).elementdef) and
                      not(is_special_array(p));
      end;

    { true if p is a widechar array def }
    function is_widechararray(p : tdef) : boolean;
      begin
        is_widechararray:=(p.typ=arraydef) and
                          is_widechar(tarraydef(p).elementdef) and
                          not(is_special_array(p));
      end;


    { true if p is a open char array def }
    function is_open_chararray(p : tdef) : boolean;
      begin
        is_open_chararray:= is_open_array(p) and
                            is_char(tarraydef(p).elementdef);
      end;

    { true if p is a open wide char array def }
    function is_open_widechararray(p : tdef) : boolean;
      begin
        is_open_widechararray:= is_open_array(p) and
                                is_widechar(tarraydef(p).elementdef);
      end;

    { true if p is any pointer def }
    function is_pointer(p : tdef) : boolean;
      begin
        is_pointer:=(p.typ=pointerdef);
      end;

    { true if p is a pchar def }
    function is_pchar(p : tdef) : boolean;
      begin
        is_pchar:=(p.typ=pointerdef) and
                  (is_char(tpointerdef(p).pointeddef) or
                   (is_zero_based_array(tpointerdef(p).pointeddef) and
                    is_chararray(tpointerdef(p).pointeddef)));
      end;

    { true if p is a pchar def }
    function is_pwidechar(p : tdef) : boolean;
      begin
        is_pwidechar:=(p.typ=pointerdef) and
                      (is_widechar(tpointerdef(p).pointeddef) or
                       (is_zero_based_array(tpointerdef(p).pointeddef) and
                        is_widechararray(tpointerdef(p).pointeddef)));
      end;


    { true if p is a voidpointer def }
    function is_voidpointer(p : tdef) : boolean;
      begin
        is_voidpointer:=(p.typ=pointerdef) and
                        (tpointerdef(p).pointeddef.typ=orddef) and
                        (torddef(tpointerdef(p).pointeddef).ordtype=uvoid);
      end;


    { true, if def is a 8 bit int type }
    function is_8bitint(def : tdef) : boolean;
      begin
         result:=(def.typ=orddef) and (torddef(def).ordtype in [u8bit,s8bit])
      end;

    { true, if def is a 8 bit ordinal type }
    function is_8bit(def : tdef) : boolean;
      begin
         result:=(def.typ=orddef) and (torddef(def).ordtype in [u8bit,s8bit,pasbool1,pasbool8,bool8bit,uchar])
      end;

    { true, if def is a 16 bit int type }
    function is_16bitint(def : tdef) : boolean;
      begin
         result:=(def.typ=orddef) and (torddef(def).ordtype in [u16bit,s16bit])
      end;

    { true, if def is a 16 bit ordinal type }
    function is_16bit(def : tdef) : boolean;
      begin
         result:=(def.typ=orddef) and (torddef(def).ordtype in [u16bit,s16bit,pasbool16,bool16bit,uwidechar])
      end;

    { true, if def is a 32 bit int type }
    function is_32bitint(def : tdef) : boolean;
      begin
         result:=(def.typ=orddef) and (torddef(def).ordtype in [u32bit,s32bit])
      end;

    { true, if def is a 32 bit ordinal type }
    function is_32bit(def: tdef): boolean;
      begin
         result:=(def.typ=orddef) and (torddef(def).ordtype in [u32bit,s32bit,pasbool32,bool32bit])
      end;

    { true, if def is a 64 bit int type }
    function is_64bitint(def : tdef) : boolean;
      begin
         is_64bitint:=(def.typ=orddef) and (torddef(def).ordtype in [u64bit,s64bit])
      end;


    { true, if def is a 64 bit type }
    function is_64bit(def : tdef) : boolean;
      begin
         is_64bit:=(def.typ=orddef) and (torddef(def).ordtype in [u64bit,s64bit,scurrency,pasbool64,bool64bit])
      end;


    { true, if def1 and def2 are both integers of the same bit size and sign }
    function are_equal_ints(def1, def2: tdef): boolean;
      begin
        result:=(def1.typ=orddef) and (def2.typ=orddef) and
          (torddef(def1).ordtype in [u8bit,u16bit,u32bit,u64bit,
                                     s8bit,s16bit,s32bit,s64bit,customint]) and
          (torddef(def1).ordtype=torddef(def2).ordtype) and
          ((torddef(def1).ordtype<>customint) or
           ((torddef(def1).low=torddef(def2).low) and
            (torddef(def1).high=torddef(def2).high)));
      end;


    { true, if def is an int type, larger than the processor's native int size }
    function is_oversizedint(def : tdef) : boolean;
      begin
{$if defined(cpu8bitalu)}
         result:=is_64bitint(def) or is_32bitint(def) or is_16bitint(def);
{$elseif defined(cpu16bitalu)}
         result:=is_64bitint(def) or is_32bitint(def);
{$elseif defined(cpu32bitaddr)}
         result:=is_64bitint(def);
{$elseif defined(cpu64bitaddr)}
         result:=false;
{$endif}
      end;

    { true, if def is an ordinal type, larger than the processor's native int size }
    function is_oversizedord(def : tdef) : boolean;
      begin
{$if defined(cpu8bitalu)}
         result:=is_64bit(def) or is_32bit(def) or is_16bit(def);
{$elseif defined(cpu16bitalu)}
         result:=is_64bit(def) or is_32bit(def);
{$elseif defined(cpu32bitaddr)}
         result:=is_64bit(def);
{$elseif defined(cpu64bitaddr)}
         result:=false;
{$endif}
      end;


    { true, if def is an int type, equal in size to the processor's native int size }
    function is_nativeint(def: tdef): boolean;
      begin
{$if defined(cpu8bitalu)}
         result:=is_8bitint(def);
{$elseif defined(cpu16bitalu)}
         result:=is_16bitint(def);
{$elseif defined(cpu32bitaddr)}
         result:=is_32bitint(def);
{$elseif defined(cpu64bitaddr)}
         result:=is_64bitint(def);
{$endif}
      end;

    { true, if def is an ordinal type, equal in size to the processor's native int size }
    function is_nativeord(def: tdef): boolean;
      begin
{$if defined(cpu8bitalu)}
         result:=is_8bit(def);
{$elseif defined(cpu16bitalu)}
         result:=is_16bit(def);
{$elseif defined(cpu32bitaddr)}
         result:=is_32bit(def);
{$elseif defined(cpu64bitaddr)}
         result:=is_64bit(def);
{$endif}
      end;

    { true, if def is an unsigned int type, equal in size to the processor's native int size }
    function is_nativeuint(def: tdef): boolean;
      begin
         result:=is_nativeint(def) and (def.typ=orddef) and (torddef(def).ordtype in [u64bit,u32bit,u16bit,u8bit]);
      end;

    { true, if def is a signed int type, equal in size to the processor's native int size }
    function is_nativesint(def: tdef): boolean;
      begin
         result:=is_nativeint(def) and (def.typ=orddef) and (torddef(def).ordtype in [s64bit,s32bit,s16bit,s8bit]);
      end;

    { if l isn't in the range of todef a range check error (if not explicit) is generated and
      the value is placed within the range }
    procedure adaptrange(todef : tdef;var l : tconstexprint; rangecheck: tperformrangecheck);
      var
         lv,hv,oldval,sextval,mask: TConstExprInt;
         rangedef: tdef;
         rangedefsize: longint;
         warned: boolean;
      begin
         getrange(todef,lv,hv);
         if (l<lv) or (l>hv) then
           begin
             warned:=false;
             if rangecheck in [rc_implicit,rc_yes] then
               begin
                 if (rangecheck=rc_yes) or
                    (todef.typ=enumdef) then
                   Message3(type_e_range_check_error_bounds,tostr(l),tostr(lv),tostr(hv))
                 else
                   Message3(type_w_range_check_error_bounds,tostr(l),tostr(lv),tostr(hv));
                 warned:=true;
               end
             { give warnings about range errors with explicit typeconversions if the target
               type does not span the entire range that can be represented by its bits
               (subrange type or enum), because then the result is undefined }
             else if (rangecheck<>rc_internal) and
                     (not is_pasbool(todef) and
                      not spans_entire_range(todef)) then
               begin
                 Message3(type_w_range_check_error_bounds,tostr(l),tostr(lv),tostr(hv));
                 warned:=true;
               end;

             { Fix the value to fit in the allocated space for this type of variable }
             oldval:=l;
             getrangedefmasksize(todef,rangedef,mask,rangedefsize);
             l:=l and mask;
             {reset sign, i.e. converting -1 to qword changes the value to high(qword)}
             l.signed:=false;
             sextval:=0;
             { do sign extension if necessary (JM) }
             case rangedefsize of
               1: sextval.svalue:=shortint(l.svalue);
               2: sextval.svalue:=smallint(l.svalue);
               4: sextval.svalue:=longint(l.svalue);
               8: sextval.svalue:=l.svalue;
               else
                 internalerror(201906230);
              end;
              sextval.signed:=true;
              { Detect if the type spans the entire range, but more bits were specified than
                the type can contain, e.g. shortint($fff).
                However, none of the following should result in a warning:
                  1) shortint($ff) (-> $ff -> $ff -> $ffff ffff ffff ffff)
                  2) shortint(longint(-1)) ($ffff ffff ffff ffff ffff -> $ff -> $ffff ffff ffff ffff
                  3) cardinal(-1) (-> $ffff ffff ffff ffff -> $ffff ffff)
              }
              if not warned and
                (rangecheck<>rc_internal) and
                (oldval.uvalue<>l.uvalue) and
                (oldval.uvalue<>sextval.uvalue) then
               begin
                 Message3(type_w_range_check_error_bounds,tostr(oldval),tostr(lv),tostr(hv));
               end;
              if is_signed(rangedef) then
                l:=sextval;
           end;
      end;


    procedure adaptrange(todef: tdef; var l: tconstexprint; internal, explicit, rangecheckstate: boolean);
      begin
        if internal then
          adaptrange(todef, l, rc_internal)
        else if explicit then
          adaptrange(todef, l, rc_explicit)
        else if not rangecheckstate then
          adaptrange(todef, l, rc_implicit)
        else
          adaptrange(todef, l, rc_yes)
      end;


    { return the range from def in l and h }
    procedure getrange(def : tdef;out l, h : TConstExprInt);
      begin
        case def.typ of
          orddef :
            begin
              l:=torddef(def).low;
              h:=torddef(def).high;
            end;
          enumdef :
            begin
              l:=int64(tenumdef(def).min);
              h:=int64(tenumdef(def).max);
            end;
          arraydef :
            begin
              l:=int64(tarraydef(def).lowrange);
              h:=int64(tarraydef(def).highrange);
            end;
          undefineddef:
            begin
              l:=torddef(sizesinttype).low;
              h:=torddef(sizesinttype).high;
            end;
          else
            internalerror(200611054);
        end;
      end;


    procedure getrangedefmasksize(def: tdef; out rangedef: tdef; out mask: TConstExprInt; out size: longint);
      begin
        case def.typ of
          orddef, enumdef:
            begin
              rangedef:=def;
              size:=def.size;
              case size of
                1: mask:=$ff;
                2: mask:=$ffff;
                4: mask:=$ffffffff;
                8: mask:=$ffffffffffffffff;
                else
                  internalerror(2019062305);
                end;
            end;
          arraydef:
            begin
              rangedef:=tarraydef(def).rangedef;
              getrangedefmasksize(rangedef,rangedef,mask,size);
            end;
          undefineddef:
            begin
              rangedef:=sizesinttype;
              size:=rangedef.size;
              mask:=-1;
            end;
          else
            internalerror(2019062306);
        end;
      end;


    function mmx_type(p : tdef) : tmmxtype;
      begin
         mmx_type:=mmxno;
         if is_mmx_able_array(p) then
           begin
              if tarraydef(p).elementdef.typ=floatdef then
                case tfloatdef(tarraydef(p).elementdef).floattype of
                  s32real:
                    mmx_type:=mmxsingle;
                  else
                    ;
                end
              else
                case torddef(tarraydef(p).elementdef).ordtype of
                   u8bit:
                     mmx_type:=mmxu8bit;
                   s8bit:
                     mmx_type:=mmxs8bit;
                   u16bit:
                     mmx_type:=mmxu16bit;
                   s16bit:
                     mmx_type:=mmxs16bit;
                   u32bit:
                     mmx_type:=mmxu32bit;
                   s32bit:
                     mmx_type:=mmxs32bit;
                   else
                     ;
                end;
           end;
      end;


    { The range-type of an ordinal-type that is a subrange-type shall be the host-type (see 6.4.2.4) of the subrange-type.
      The range-type of an ordinal-type that is not a subrange-type shall be the ordinal-type.

      The subrange-bounds shall be of compatible ordinal-types, and the range-type (see 6.4.2.1) of the ordinal-types shall
      be designated the host-type of the subrange-type. }
    function get_iso_range_type(def: tdef): tdef;
      begin
        result:=nil;
        case def.typ of
           orddef:
             begin
               if is_integer(def) then
                 begin
                   if (torddef(def).low>=torddef(sinttype).low) and
                      (torddef(def).high<=torddef(sinttype).high) then
                     result:=sinttype
                   else
                     range_to_type(torddef(def).low,torddef(def).high,result);
                 end
               else case torddef(def).ordtype of
                 pasbool1:
                   result:=pasbool1type;
                 pasbool8:
                   result:=pasbool8type;
                 pasbool16:
                   result:=pasbool16type;
                 pasbool32:
                   result:=pasbool32type;
                 pasbool64:
                   result:=pasbool64type;
                 bool8bit:
                   result:=bool8type;
                 bool16bit:
                   result:=bool16type;
                 bool32bit:
                   result:=bool32type;
                 bool64bit:
                   result:=bool64type;
                 uchar:
                   result:=cansichartype;
                 uwidechar:
                   result:=cwidechartype;
                 scurrency:
                   result:=s64currencytype;
                 else
                   internalerror(2018010901);
               end;
             end;
           enumdef:
             begin
               while assigned(tenumdef(def).basedef) do
                 def:=tenumdef(def).basedef;
               result:=def;
             end
           else
             internalerror(2018010701);
        end;
      end;


    function is_vector(p : tdef) : boolean;
      begin
        result:=(p.typ=arraydef) and
                not(is_special_array(p)) and
                (tarraydef(p).elementdef.typ=floatdef) and
                (tfloatdef(tarraydef(p).elementdef).floattype in [s32real,s64real]);
      end;


    { returns if the passed type (array) fits into an mm register }
    function fits_in_mm_register(p : tdef) : boolean;
      begin
{$ifdef x86}
        result:= is_vector(p) and
                 (
                  (tarraydef(p).elementdef.typ=floatdef) and
                  (
                   (tarraydef(p).lowrange=0) and
                   (tarraydef(p).highrange=3) and
                   (tfloatdef(tarraydef(p).elementdef).floattype=s32real)
                  )
                 ) or

                 (
                  (tarraydef(p).elementdef.typ=floatdef) and
                  (
                   (tarraydef(p).lowrange=0) and
                   (tarraydef(p).highrange=1) and
                   (tfloatdef(tarraydef(p).elementdef).floattype=s64real)
                  )
                 );
{$else x86}
        result:=false;
{$endif x86}
      end;


    function is_mmx_able_array(p : tdef) : boolean;
      begin
{$ifdef SUPPORT_MMX}
         if (cs_mmx_saturation in current_settings.localswitches) then
           begin
              is_mmx_able_array:=(p.typ=arraydef) and
                not(is_special_array(p)) and
                (
                 (
                  (tarraydef(p).elementdef.typ=orddef) and
                  (
                   (
                    (tarraydef(p).lowrange=0) and
                    (tarraydef(p).highrange=1) and
                    (torddef(tarraydef(p).elementdef).ordtype in [u32bit,s32bit])
                   )
                   or
                   (
                    (tarraydef(p).lowrange=0) and
                    (tarraydef(p).highrange=3) and
                    (torddef(tarraydef(p).elementdef).ordtype in [u16bit,s16bit])
                   )
                  )
                 )
                 or
                (
                 (
                  (tarraydef(p).elementdef.typ=floatdef) and
                  (
                   (tarraydef(p).lowrange=0) and
                   (tarraydef(p).highrange=1) and
                   (tfloatdef(tarraydef(p).elementdef).floattype=s32real)
                  )
                 )
                )
              );
           end
         else
           begin
              is_mmx_able_array:=(p.typ=arraydef) and
                (
                 (
                  (tarraydef(p).elementdef.typ=orddef) and
                  (
                   (
                    (tarraydef(p).lowrange=0) and
                    (tarraydef(p).highrange=1) and
                    (torddef(tarraydef(p).elementdef).ordtype in [u32bit,s32bit])
                   )
                   or
                   (
                    (tarraydef(p).lowrange=0) and
                    (tarraydef(p).highrange=3) and
                    (torddef(tarraydef(p).elementdef).ordtype in [u16bit,s16bit])
                   )
                   or
                   (
                    (tarraydef(p).lowrange=0) and
                    (tarraydef(p).highrange=7) and
                    (torddef(tarraydef(p).elementdef).ordtype in [u8bit,s8bit])
                   )
                  )
                 )
                 or
                 (
                  (tarraydef(p).elementdef.typ=floatdef) and
                  (
                   (tarraydef(p).lowrange=0) and
                   (tarraydef(p).highrange=1) and
                   (tfloatdef(tarraydef(p).elementdef).floattype=s32real)
                  )
                 )
                );
           end;
{$else SUPPORT_MMX}
         is_mmx_able_array:=false;
{$endif SUPPORT_MMX}
      end;


    function def_cgsize(def: tdef): tcgsize;
      begin
        case def.typ of
          orddef,
          enumdef,
          setdef:
            begin
              result:=int_cgsize(def.size);
              if is_signed(def) then
                result:=tcgsize(ord(result)+(ord(OS_S8)-ord(OS_8)));
            end;
          classrefdef,
          pointerdef:
            begin
              result:=int_cgsize(def.size);
              { can happen for far/huge pointers on non-i8086 }
              if result=OS_NO then
                internalerror(2013052201);
            end;
          formaldef:
            result := int_cgsize(voidpointertype.size);
          procvardef:
            result:=int_cgsize(def.size);
          stringdef :
            result:=int_cgsize(def.size);
          objectdef :
            result:=int_cgsize(def.size);
          floatdef:
            if cs_fp_emulation in current_settings.moduleswitches then
              result:=int_cgsize(def.size)
            else
              result:=tfloat2tcgsize[tfloatdef(def).floattype];
          recorddef :
            result:=int_cgsize(def.size);
          arraydef :
            begin
              if is_dynamic_array(def) or not is_special_array(def) then
                begin
                  if (cs_support_vectors in current_settings.globalswitches) and is_vector(def) and ((TArrayDef(def).elementdef.typ = floatdef) and not (cs_fp_emulation in current_settings.moduleswitches)) then
                    begin
                      { Determine if, based on the floating-point type and the size
                        of the array, if it can be made into a vector }
                      case TFloatDef(def).floattype of
                        s32real:
                          result := float_array_cgsize(def.size);
                        s64real:
                          result := double_array_cgsize(def.size);
                        else
                          { If not, fall back }
                          result := int_cgsize(def.size);
                      end;
                    end
                  else
                    result := int_cgsize(def.size);
                end
              else
                result := OS_NO;
            end;
          else
            begin
              { undefined size }
              result:=OS_NO;
            end;
        end;
      end;

    function cgsize_orddef(size: tcgsize): torddef;
      begin
        case size of
          OS_8:
            result:=torddef(u8inttype);
          OS_S8:
            result:=torddef(s8inttype);
          OS_16:
            result:=torddef(u16inttype);
          OS_S16:
            result:=torddef(s16inttype);
          OS_32:
            result:=torddef(u32inttype);
          OS_S32:
            result:=torddef(s32inttype);
          OS_64:
            result:=torddef(u64inttype);
          OS_S64:
            result:=torddef(s64inttype);
          else
            internalerror(2012050401);
        end;
      end;

    function def_cgmmsize(def: tdef): tcgsize;
      begin
        case def.typ of
          arraydef:
            begin
              case tarraydef(def).elementdef.typ of
                orddef:
                  begin
                    { this is not correct, OS_MX normally mean that the vector
                      contains elements of size X. However, vectors themselves
                      can also have different sizes (e.g. a vector of 2 singles on
                      SSE) and the total size is currently more important }
                    case def.size of
                      1: result:=OS_M8;
                      2: result:=OS_M16;
                      4: result:=OS_M32;
                      8: result:=OS_M64;
                      16: result:=OS_M128;
                      32: result:=OS_M256;
                      64: result:=OS_M512;
                      else
                        internalerror(2013060103);
                    end;
                  end;
                floatdef:
                  begin
                    case TFloatDef(tarraydef(def).elementdef).floattype of
                      s32real:
                        case def.size of
                          4:  result:=OS_MF32;
                          16: result:=OS_MF128;
                          32: result:=OS_MF256;
                          64: result:=OS_MF512;
                          else
                            internalerror(2017121400);
                        end;
                      s64real:
                        case def.size of
                          8:  result:=OS_MD64;
                          16: result:=OS_MD128;
                          32: result:=OS_MD256;
                          64: result:=OS_MD512;
                          else
                            internalerror(2017121401);
                        end;
                      else
                        internalerror(2017121402);
                    end;
                  end;
                else
                  result:=def_cgsize(def);
              end;
            end
          else
            result:=def_cgsize(def);
        end;
      end;

    { In Windows 95 era, ordinals were restricted to [u8bit,s32bit,s16bit,bool16bit]
      As of today, both signed and unsigned types from 8 to 64 bits are supported. }
    function is_automatable(p : tdef) : boolean;
      begin
        case p.typ of
          orddef:
            result:=torddef(p).ordtype in [u8bit,s8bit,u16bit,s16bit,u32bit,s32bit,
              u64bit,s64bit,bool16bit,scurrency];
          floatdef:
            result:=tfloatdef(p).floattype in [s64currency,s64real,s32real];
          stringdef:
            result:=tstringdef(p).stringtype in [st_ansistring,st_widestring,st_unicodestring];
          variantdef:
            result:=true;
          objectdef:
            result:=tobjectdef(p).objecttype in [odt_interfacecom,odt_dispinterface,odt_interfacecorba];
          else
            result:=false;
        end;
      end;


    {# returns true, if the type passed is a varset }
    function is_smallset(p : tdef) : boolean;
      begin
        {$if defined(cpu8bitalu)}
          result:=(p.typ=setdef) and (p.size = 1)
        {$elseif defined(cpu16bitalu)}
          result:=(p.typ=setdef) and (p.size in [1,2])
        {$else}
          result:=(p.typ=setdef) and (p.size in [1,2,4])
        {$endif}
      end;


    function is_bareprocdef(pd : tprocdef): boolean;
      begin
        result:=(pd.maxparacount=0) and
                (is_void(pd.returndef) or
                 (pd.proctypeoption = potype_constructor));
      end;

    function is_c_variadic(pd: tabstractprocdef): boolean;
      begin
        result:=
          (po_varargs in pd.procoptions) or
          (po_variadic in pd.procoptions);
      end;

    function get_common_intdef(ld, rd: torddef; keep_sign_if_equal: boolean): torddef;
      var
        llow, lhigh: tconstexprint;
      begin
        llow:=min(ld.low,rd.low);
        lhigh:=max(ld.high,rd.high);
        case range_to_basetype(llow,lhigh) of
          s8bit:
            result:=torddef(s8inttype);
          u8bit:
            result:=torddef(u8inttype);
          s16bit:
            result:=torddef(s16inttype);
          u16bit:
            result:=torddef(u16inttype);
          s32bit:
            result:=torddef(s32inttype);
          u32bit:
            result:=torddef(u32inttype);
          s64bit:
            result:=torddef(s64inttype);
          u64bit:
            result:=torddef(u64inttype);
          else
            begin
              { avoid warning }
              result:=nil;
              internalerror(200802291);
            end;
        end;
        if keep_sign_if_equal and
           (is_signed(ld)=is_signed(rd)) and
           (is_signed(result)<>is_signed(ld)) then
          case result.ordtype of
            s8bit:
              result:=torddef(u8inttype);
            u8bit:
              result:=torddef(s16inttype);
            s16bit:
              result:=torddef(u16inttype);
            u16bit:
              result:=torddef(s32inttype);
            s32bit:
              result:=torddef(u32inttype);
            u32bit:
              result:=torddef(s64inttype);
            s64bit:
              result:=torddef(u64inttype);
            else
              ;
          end;
      end;


    function is_valid_univ_para_type(def: tdef): boolean;
      begin
        result:=
          not is_open_array(def) and
          not is_void(def) and
          (def.typ<>formaldef);
      end;


    function is_nested_pd(def: tabstractprocdef): boolean;{$ifdef USEINLINE}inline;{$endif}
      begin
        result:=def.parast.symtablelevel>normal_function_level;
      end;


    function is_typeparam(def : tdef) : boolean;{$ifdef USEINLINE}inline;{$endif}
      begin
        result:=(def.typ=undefineddef);
      end;


    function is_methodpointer(def: tdef): boolean;
      begin
        result:=(def.typ=procvardef) and (po_methodpointer in tprocvardef(def).procoptions);
      end;


    function is_block(def: tdef): boolean;
      begin
        result:=(def.typ=procvardef) and (po_is_block in tprocvardef(def).procoptions)
      end;


    function get_typekind(def:tdef):byte;
      begin
        case def.typ of
          arraydef:
            if ado_IsDynamicArray in tarraydef(def).arrayoptions then
              result:=tkDynArray
            else
              result:=tkArray;
          recorddef:
            result:=tkRecord;
          pointerdef:
            result:=tkPointer;
          orddef:
            case torddef(def).ordtype of
              u8bit,
              u16bit,
              u32bit,
              s8bit,
              s16bit,
              s32bit:
                result:=tkInteger;
              u64bit:
                result:=tkQWord;
              s64bit:
                result:=tkInt64;
              pasbool1,
              pasbool8,
              pasbool16,
              pasbool32,
              pasbool64,
              bool8bit,
              bool16bit,
              bool32bit,
              bool64bit:
                result:=tkBool;
              uchar:
                result:=tkChar;
              uwidechar:
                result:=tkWChar;
              scurrency:
                result:=tkFloat;
              else
                result:=tkUnknown;
            end;
          stringdef:
            case tstringdef(def).stringtype of
              st_shortstring:
                result:=tkSString;
              st_longstring:
                result:=tkLString;
              st_ansistring:
                result:=tkAString;
              st_widestring:
                result:=tkWString;
              st_unicodestring:
                result:=tkUString;
            end;
          enumdef:
            result:=tkEnumeration;
          objectdef:
            case tobjectdef(def).objecttype of
              odt_class,
              odt_javaclass:
                result:=tkClass;
              odt_object:
                result:=tkObject;
              odt_interfacecom,
              odt_dispinterface,
              odt_interfacejava:
                result:=tkInterface;
              odt_interfacecorba:
                result:=tkInterfaceCorba;
              odt_helper:
                result:=tkHelper;
              else
                result:=tkUnknown;
            end;
          { currently tkFile is not used }
          {filedef:
            result:=tkFile;}
          setdef:
            result:=tkSet;
          procvardef:
            if tprocvardef(def).is_methodpointer then
              result:=tkMethod
            else
              result:=tkProcVar;
          floatdef:
            result:=tkFloat;
          classrefdef:
            result:=tkClassRef;
          variantdef:
            result:=tkVariant;
          else
            result:=tkUnknown;
        end;
      end;

end.
