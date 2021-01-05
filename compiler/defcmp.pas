{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Compare definitions and parameter lists

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
unit defcmp;

{$i fpcdefs.inc}

interface

    uses
       cclasses,
       globtype,globals,
       node,
       symconst,symtype,symdef,symbase;

     type
       { if acp is cp_all the var const or nothing are considered equal }
       tcompare_paras_type = ( cp_none, cp_value_equal_const, cp_all,cp_procvar);
       tcompare_paras_option = (
          cpo_allowdefaults,
          cpo_ignorehidden,           // ignore hidden parameters
          cpo_allowconvert,
          cpo_comparedefaultvalue,
          cpo_openequalisexact,
          cpo_ignoreuniv,
          cpo_warn_incompatible_univ,
          cpo_ignorevarspez,          // ignore parameter access type
          cpo_ignoreframepointer,     // ignore frame pointer parameter (for assignment-compatibility of global procedures to nested procvars)
          cpo_compilerproc,
          cpo_rtlproc,
          cpo_generic                 // two different undefined defs (or a constraint in the forward) alone or in open arrays are
                                      // treated as exactly equal (also in open arrays) if they are owned by their respective procdefs
       );

       tcompare_paras_options = set of tcompare_paras_option;

       tcompare_defs_option = (
          cdo_internal,
          cdo_explicit,
          cdo_check_operator,
          cdo_allow_variant,
          cdo_parameter,
          cdo_warn_incompatible_univ,
          cdo_strict_undefined_check,  // undefined defs are incompatible to everything except other undefined defs
          cdo_equal_check              // this call is only to check equality -> shortcut some expensive checks
       );
       tcompare_defs_options = set of tcompare_defs_option;

       tconverttype = (tc_none,
          tc_equal,
          tc_not_possible,
          tc_string_2_string,
          tc_char_2_string,
          tc_char_2_chararray,
          tc_pchar_2_string,
          tc_cchar_2_pchar,
          tc_cstring_2_pchar,
          tc_cstring_2_int,
          tc_ansistring_2_pchar,
          tc_string_2_chararray,
          tc_chararray_2_string,
          tc_array_2_pointer,
          tc_pointer_2_array,
          tc_int_2_int,
          tc_int_2_bool,
          tc_bool_2_bool,
          tc_bool_2_int,
          tc_real_2_real,
          tc_int_2_real,
          tc_real_2_currency,
          tc_proc_2_procvar,
          tc_nil_2_methodprocvar,
          tc_arrayconstructor_2_set,
          tc_set_to_set,
          tc_cord_2_pointer,
          tc_intf_2_string,
          tc_intf_2_guid,
          tc_class_2_intf,
          tc_char_2_char,
          tc_dynarray_2_openarray,
          tc_pwchar_2_string,
          tc_variant_2_dynarray,
          tc_dynarray_2_variant,
          tc_variant_2_enum,
          tc_enum_2_variant,
          tc_interface_2_variant,
          tc_variant_2_interface,
          tc_array_2_dynarray,
          tc_elem_2_openarray,
          tc_arrayconstructor_2_dynarray,
          tc_arrayconstructor_2_array
       );

    function compare_defs_ext(def_from,def_to : tdef;
                              fromtreetype : tnodetype;
                              var doconv : tconverttype;
                              var operatorpd : tprocdef;
                              cdoptions:tcompare_defs_options):tequaltype;

    { Returns if the type def_from can be converted to def_to or if both types are equal }
    function compare_defs(def_from,def_to:tdef;fromtreetype:tnodetype):tequaltype;

    { Returns true, if def1 and def2 are semantically the same }
    function equal_defs(def_from,def_to:tdef):boolean;

    { Checks for type compatibility (subgroups of type)
      used for case statements... probably missing stuff
      to use on other types }
    function is_subequal(def1, def2: tdef): boolean;

     {# true, if two parameter lists are equal
      if acp is cp_all, all have to match exactly
      if acp is cp_value_equal_const call by value
      and call by const parameter are assumed as
      equal
      if acp is cp_procvar then the varspez have to match,
      and all parameter types must be at least te_equal
      if acp is cp_none, then we don't check the varspez at all
      allowdefaults indicates if default value parameters
      are allowed (in this case, the search order will first
      search for a routine with default parameters, before
      searching for the same definition with no parameters)

      para1 is expected to be parameter list of the first encountered
      declaration (interface, forward), and para2 that of the second one
      (important in case of cpo_comparedefaultvalue)
    }
    function compare_paras(para1,para2 : TFPObjectList; acp : tcompare_paras_type; cpoptions: tcompare_paras_options):tequaltype;

    { True if a function can be assigned to a procvar }
    { changed first argument type to pabstractprocdef so that it can also be }
    { used to test compatibility between two pprocvardefs (JM)               }
    function proc_to_procvar_equal(def1:tabstractprocdef;def2:tprocvardef;checkincompatibleuniv: boolean):tequaltype;

    { Parentdef is the definition of a method defined in a parent class or interface }
    { Childdef is the definition of a method defined in a child class, interface or  }
    { a class implementing an interface with parentdef.                              }
    { Returns true if the resultdef of childdef can be used to implement/override    }
    { parentdef's resultdef                                                          }
    function compatible_childmethod_resultdef(parentretdef, childretdef: tdef): boolean;

    { Checks whether the class impldef or one of its parent classes implements }
    { the interface intfdef and returns the corresponding "implementation link }
    function find_implemented_interface(impldef,intfdef:tobjectdef):timplementedinterface;

    { Checks whether to defs are related to each other. Thereby the following  }
    { cases of curdef are implemented:                                         }
    { - stringdef: on JVM JLObject, JLString and AnsiString are compatible     }
    { - recorddef: on JVM records are compatible to java_fpcbaserecordtype     }
    {              and JLObject                                                }
    { - objectdef: if it inherits from otherdef or they are equal              }
    function def_is_related(curdef,otherdef:tdef):boolean;

    { Checks whether two defs for parameters or result types of a generic }
    { routine can be considered as equal. Requires the symtables of the   }
    { procdefs the parameters defs shall belong to.                       }
    function equal_genfunc_paradefs(fwdef,currdef:tdef;fwpdst,currpdst:tsymtable):boolean;


implementation

    uses
      verbose,systems,constexp,
      symtable,symsym,symcpu,
      defutil,symutil;


    function compare_defs_ext(def_from,def_to : tdef;
                              fromtreetype : tnodetype;
                              var doconv : tconverttype;
                              var operatorpd : tprocdef;
                              cdoptions:tcompare_defs_options):tequaltype;

      { tordtype:
           uvoid,
           u8bit,u16bit,u32bit,u64bit,
           s8bit,s16bit,s32bit,s64bit,
           pasbool, bool8bit,bool16bit,bool32bit,bool64bit,
           uchar,uwidechar,scurrency,customint }

      type
        tbasedef=(bvoid,bchar,bint,bbool);
      const
        basedeftbl:array[tordtype] of tbasedef =
          (bvoid,
           bint,bint,bint,bint,bint,
           bint,bint,bint,bint,bint,
           bbool,bbool,bbool,bbool,bbool,
           bbool,bbool,bbool,bbool,
           bchar,bchar,bint,bint);

        basedefconvertsimplicit : array[tbasedef,tbasedef] of tconverttype =
          { void, char, int, bool }
         ((tc_not_possible,tc_not_possible,tc_not_possible,tc_not_possible),
          (tc_not_possible,tc_char_2_char,tc_not_possible,tc_not_possible),
          (tc_not_possible,tc_not_possible,tc_int_2_int,tc_not_possible),
          (tc_not_possible,tc_not_possible,tc_not_possible,tc_bool_2_bool));
        basedefconvertsexplicit : array[tbasedef,tbasedef] of tconverttype =
          { void, char, int, bool }
         ((tc_not_possible,tc_not_possible,tc_not_possible,tc_not_possible),
          (tc_not_possible,tc_char_2_char,tc_int_2_int,tc_int_2_bool),
          (tc_not_possible,tc_int_2_int,tc_int_2_int,tc_int_2_bool),
          (tc_not_possible,tc_bool_2_int,tc_bool_2_int,tc_bool_2_bool));

      var
         subeq,eq : tequaltype;
         hd1,hd2 : tdef;
         def_generic : tstoreddef;
         hct : tconverttype;
         hobjdef : tobjectdef;
         hpd : tprocdef;
         i : longint;
         diff : boolean;
         symfrom,symto : tsym;
      begin
         eq:=te_incompatible;
         doconv:=tc_not_possible;

         { safety check }
         if not(assigned(def_from) and assigned(def_to)) then
          begin
            compare_defs_ext:=te_incompatible;
            exit;
          end;

         { resolve anonymous external definitions }
         if def_from.typ=objectdef then
           def_from:=find_real_class_definition(tobjectdef(def_from),false);
         if def_to.typ=objectdef then
           def_to:=find_real_class_definition(tobjectdef(def_to),false);

         { same def? then we've an exact match }
         if def_from=def_to then
          begin
            doconv:=tc_equal;
            compare_defs_ext:=te_exact;
            exit;
          end;

         if cdo_strict_undefined_check in cdoptions then
           begin
             { two different undefined defs are not considered equal }
             if (def_from.typ=undefineddef) and
                (def_to.typ=undefineddef) then
              begin
                doconv:=tc_not_possible;
                compare_defs_ext:=te_incompatible;
                exit;
              end;

             { if only one def is a undefined def then they are not considered as
               equal}
             if (
                   (def_from.typ=undefineddef) or
                   assigned(tstoreddef(def_from).genconstraintdata)
                 ) or (
                   (def_to.typ=undefineddef) or
                   assigned(tstoreddef(def_to).genconstraintdata)
                 ) then
              begin
                doconv:=tc_not_possible;
                compare_defs_ext:=te_incompatible;
                exit;
              end;
           end
         else
           begin
             { undefined defs are considered equal to everything }
             if (def_from.typ=undefineddef) or
                 (def_to.typ=undefineddef) then
               begin
                 doconv:=tc_equal;
                 compare_defs_ext:=te_exact;
                 exit;
               end;

             { either type has constraints }
             if assigned(tstoreddef(def_from).genconstraintdata) or
                 assigned(tstoreddef(def_to).genconstraintdata) then
               begin
                 { this is bascially a poor man's type checking, if there is a chance
                   that the types are equal considering the constraints, this needs probably
                   to be improved and maybe factored out or even result in a recursive compare_defs_ext }
                 if (def_from.typ<>def_to.typ) and
                   { formaldefs are compatible with everything }
                   not(def_from.typ in [formaldef]) and
                   not(def_to.typ in [formaldef]) and
                   { constants could get another deftype (e.g. niln) }
                   not(fromtreetype in nodetype_const) then
                   begin
                     { not compatible anyway }
                     doconv:=tc_not_possible;
                     compare_defs_ext:=te_incompatible;
                     exit;
                   end;

                 { maybe we are in generic type declaration/implementation.
                   In this case constraint in comparison to not specialized generic
                   is not "exact" nor "incompatible" }
                 if not(((df_genconstraint in def_from.defoptions) and
                        ([df_generic,df_specialization]*def_to.defoptions=[df_generic])
                      ) or
                      (
                        (df_genconstraint in def_to.defoptions) and
                        ([df_generic,df_specialization]*def_from.defoptions=[df_generic]))
                    ) then
                   begin
                     { one is definitely a constraint, for the other we don't
                       care right now }
                     doconv:=tc_equal;
                     compare_defs_ext:=te_exact;
                     exit;
                   end;
               end;
           end;

         { two specializations are considered equal if they specialize the same
           generic with the same types }
         if (df_specialization in def_from.defoptions) and
             (df_specialization in def_to.defoptions) and
             (tstoreddef(def_from).genericdef=tstoreddef(def_to).genericdef) then
           begin
             if assigned(tstoreddef(def_from).genericparas) xor
                 assigned(tstoreddef(def_to).genericparas) then
               internalerror(2013030901);
             diff:=false;
             if assigned(tstoreddef(def_from).genericparas) then
               begin
                 if tstoreddef(def_from).genericparas.count<>tstoreddef(def_to).genericparas.count then
                   internalerror(2012091301);
                 for i:=0 to tstoreddef(def_from).genericparas.count-1 do
                   begin
                     if tstoreddef(def_from).genericparas.nameofindex(i)<>tstoreddef(def_to).genericparas.nameofindex(i) then
                       internalerror(2012091302);
                     symfrom:=ttypesym(tstoreddef(def_from).genericparas[i]);
                     symto:=ttypesym(tstoreddef(def_to).genericparas[i]);
                     if not (symfrom.typ in [typesym,constsym]) or not (symto.typ in [typesym,constsym]) then
                       internalerror(2012121401);
                     if symto.typ<>symfrom.typ then
                       diff:=true
                     else if (symfrom.typ=constsym) and (symto.typ=constsym) and not equal_constsym(tconstsym(symfrom),tconstsym(symto),true) then
                       diff:=true
                     else if not equal_defs(ttypesym(symfrom).typedef,ttypesym(symto).typedef) then
                       diff:=true;
                     if diff then
                       break;
                   end;
               end;
             if not diff then
               begin
                 doconv:=tc_equal;
                 { the definitions are not exactly the same, but only equal }
                 compare_defs_ext:=te_equal;
                 exit;
               end;
           end;
         { handling of partial specializations }
         if (
               (df_generic in def_to.defoptions) and
               (df_specialization in def_from.defoptions) and
               (tstoreddef(def_from).genericdef=def_to)
             ) or (
               (df_generic in def_from.defoptions) and
               (df_specialization in def_to.defoptions) and
               (tstoreddef(def_to).genericdef=def_from)
             ) then
           begin
             if tstoreddef(def_from).genericdef=def_to then
               def_generic:=tstoreddef(def_to)
             else
               def_generic:=tstoreddef(def_from);
             if not assigned(def_generic.genericparas) then
               internalerror(2014052306);
             diff:=false;
             for i:=0 to def_generic.genericparas.count-1 do
               begin
                 symfrom:=tsym(def_generic.genericparas[i]);
                 if symfrom.typ<>typesym then
                   internalerror(2014052307);
                 if ttypesym(symfrom).typedef.typ<>undefineddef then
                   diff:=true;
                 if diff then
                   break;
               end;
             if not diff then
               begin
                 doconv:=tc_equal;
                 { the definitions are not exactly the same, but only equal }
                 compare_defs_ext:=te_equal;
                 exit;
               end;
           end;

         { we walk the wanted (def_to) types and check then the def_from
           types if there is a conversion possible }
         case def_to.typ of
           orddef :
             begin
               case def_from.typ of
                 orddef :
                   begin
                     if (torddef(def_from).ordtype=torddef(def_to).ordtype) then
                      begin
                        case torddef(def_from).ordtype of
                          uchar,uwidechar,
                          u8bit,u16bit,u32bit,u64bit,
                          s8bit,s16bit,s32bit,s64bit:
                            begin
                              if (torddef(def_from).low>=torddef(def_to).low) and
                                 (torddef(def_from).high<=torddef(def_to).high) then
                                eq:=te_equal
                              else
                                begin
                                  doconv:=tc_int_2_int;
                                  eq:=te_convert_l1;
                                end;
                            end;
                          uvoid,
                          pasbool1,pasbool8,pasbool16,pasbool32,pasbool64,
                          bool8bit,bool16bit,bool32bit,bool64bit,
                          scurrency:
                            eq:=te_equal;
                          else
                            internalerror(200210061);
                        end;
                      end
                     { currency cannot be implicitly converted to an ordinal
                       type }
                     else if not is_currency(def_from) or
                             (cdo_explicit in cdoptions) then
                      begin
                        if cdo_explicit in cdoptions then
                          doconv:=basedefconvertsexplicit[basedeftbl[torddef(def_from).ordtype],basedeftbl[torddef(def_to).ordtype]]
                        else
                          doconv:=basedefconvertsimplicit[basedeftbl[torddef(def_from).ordtype],basedeftbl[torddef(def_to).ordtype]];
                        if (doconv=tc_not_possible) then
                          eq:=te_incompatible
                        else if (not is_in_limit(def_from,def_to)) then
                          { "punish" bad type conversions :) (JM) }
                          eq:=te_convert_l3
                        else
                          eq:=te_convert_l1;
                      end;
                   end;
                 enumdef :
                   begin
                     { needed for char(enum) }
                     if cdo_explicit in cdoptions then
                      begin
                        doconv:=tc_int_2_int;
                        eq:=te_convert_l1;
                      end;
                   end;
                 floatdef :
                   begin
                     if is_currency(def_to) then
                      begin
                        doconv:=tc_real_2_currency;
                        eq:=te_convert_l2;
                      end;
                   end;
                 objectdef:
                   begin
                     if (m_delphi in current_settings.modeswitches) and
                        is_implicit_pointer_object_type(def_from) and
                        (cdo_explicit in cdoptions) then
                      begin
                        eq:=te_convert_l1;
                        if (fromtreetype=niln) then
                         begin
                           { will be handled by the constant folding }
                           doconv:=tc_equal;
                         end
                        else
                         doconv:=tc_int_2_int;
                      end;
                   end;
                 classrefdef,
                 procvardef,
                 pointerdef :
                   begin
                     if cdo_explicit in cdoptions then
                      begin
                        eq:=te_convert_l1;
                        if (fromtreetype=niln) then
                         begin
                           { will be handled by the constant folding }
                           doconv:=tc_equal;
                         end
                        else
                         doconv:=tc_int_2_int;
                      end;
                   end;
                 arraydef :
                   begin
                     if (m_mac in current_settings.modeswitches) and
                        is_integer(def_to) and
                        (fromtreetype=stringconstn) then
                       begin
                         eq:=te_convert_l3;
                         doconv:=tc_cstring_2_int;
                       end;
                   end;
                 else
                   ;
               end;
             end;

           stringdef :
             begin
               case def_from.typ of
                 stringdef :
                   begin
                     { Constant string }
                     if (fromtreetype=stringconstn) and
                        is_shortstring(def_from) and
                        is_shortstring(def_to) then
                        eq:=te_equal
                     else if (tstringdef(def_to).stringtype=st_ansistring) and
                             (tstringdef(def_from).stringtype=st_ansistring) then 
                      begin
                        { don't convert ansistrings if any condition is true:
                          1) same encoding
                          2) from explicit codepage ansistring to ansistring and vice versa
                          3) from any ansistring to rawbytestring 
                          4) from rawbytestring to any ansistring }
                        if (tstringdef(def_from).encoding=tstringdef(def_to).encoding) or
                           ((tstringdef(def_to).encoding=0) and (tstringdef(def_from).encoding=getansistringcodepage)) or
                           ((tstringdef(def_to).encoding=getansistringcodepage) and (tstringdef(def_from).encoding=0)) or
                           (tstringdef(def_to).encoding=globals.CP_NONE) or
                           (tstringdef(def_from).encoding=globals.CP_NONE) then
                         begin
                           eq:=te_equal;
                         end
                        else
                         begin        
                           doconv := tc_string_2_string;

                           { prefere conversion to utf8 codepage }
                           if tstringdef(def_to).encoding = globals.CP_UTF8 then
                             eq:=te_convert_l1
                           { else to AnsiString type }
                           else if def_to=getansistringdef then
                             eq:=te_convert_l2
                           { else to AnsiString with other codepage }
                           else
                             eq:=te_convert_l3;
                         end
                      end          
                     else
                     { same string type ? }
                      if (tstringdef(def_from).stringtype=tstringdef(def_to).stringtype) and
                        { for shortstrings also the length must match }
                         ((tstringdef(def_from).stringtype<>st_shortstring) or
                          (tstringdef(def_from).len=tstringdef(def_to).len)) and
                         { for ansi- and unicodestrings also the encoding must match }
                         (not(tstringdef(def_from).stringtype in [st_ansistring,st_unicodestring]) or
                          (tstringdef(def_from).encoding=tstringdef(def_to).encoding)) then
                        eq:=te_equal
                     else
                       begin
                         doconv:=tc_string_2_string;
                         case tstringdef(def_from).stringtype of
                           st_widestring :
                             begin
                               case tstringdef(def_to).stringtype of
                                 { Prefer conversions to unicodestring }
                                 st_unicodestring: eq:=te_convert_l1;
                                 { else prefer conversions to ansistring }
                                 st_ansistring: eq:=te_convert_l2;
                                 else
                                   eq:=te_convert_l3;
                               end;
                             end;
                           st_unicodestring :
                             begin
                               case tstringdef(def_to).stringtype of
                                 { Prefer conversions to widestring }
                                 st_widestring: eq:=te_convert_l1;
                                 { else prefer conversions to ansistring }
                                 st_ansistring: eq:=te_convert_l2;
                                 else
                                   eq:=te_convert_l3;
                               end;
                             end;
                           st_shortstring :
                             begin
                               { Prefer shortstrings of different length or conversions
                                 from shortstring to ansistring }
                               case tstringdef(def_to).stringtype of
                                 st_shortstring: eq:=te_convert_l1;
                                 st_ansistring:
                                   if tstringdef(def_to).encoding=globals.CP_UTF8 then
                                     eq:=te_convert_l2
                                   else if def_to=getansistringdef then
                                     eq:=te_convert_l3
                                   else
                                     eq:=te_convert_l4;
                                 st_unicodestring: eq:=te_convert_l5;
                                 else
                                   eq:=te_convert_l6;
                               end;
                             end;
                           st_ansistring :
                             begin
                               { Prefer conversion to widestrings }
                               case tstringdef(def_to).stringtype of
                                 st_unicodestring: eq:=te_convert_l4;
                                 st_widestring: eq:=te_convert_l5;
                                 else
                                   eq:=te_convert_l6;
                               end;
                             end;
                           else
                             ;
                         end;
                       end;
                   end;
                 orddef :
                   begin
                   { char to string}
                     if is_char(def_from) then
                       begin
                         doconv:=tc_char_2_string;
                         case tstringdef(def_to).stringtype of
                           st_shortstring: eq:=te_convert_l1;
                           st_ansistring: eq:=te_convert_l2;
                           st_unicodestring: eq:=te_convert_l3;
                           st_widestring: eq:=te_convert_l4;
                         else
                           eq:=te_convert_l5;
                         end;
                       end
                     else
                     if is_widechar(def_from) then
                      begin
                        doconv:=tc_char_2_string;
                        case tstringdef(def_to).stringtype of
                          st_unicodestring: eq:=te_convert_l1;
                          st_widestring: eq:=te_convert_l2;
                          st_ansistring: eq:=te_convert_l3;
                          st_shortstring: eq:=te_convert_l4;
                        else
                          eq:=te_convert_l5;
                        end;
                      end;
                   end;
                 arraydef :
                   begin
                     { array of char to string, the length check is done by the firstpass of this node }
                     if (is_chararray(def_from) or
                         is_open_chararray(def_from)) and
                        { bitpacked arrays of char whose element bitsize is not
                          8 cannot be auto-converted to strings }
                        (not is_packed_array(def_from) or
                         (tarraydef(def_from).elementdef.packedbitsize=8)) then
                      begin
                        { "Untyped" stringconstn is an array of char }
                        if fromtreetype=stringconstn then
                          begin
                            doconv:=tc_string_2_string;
                            { prefered string type depends on the $H switch }
                            if (m_default_unicodestring in current_settings.modeswitches) and
                               (cs_refcountedstrings in current_settings.localswitches) then
                              case tstringdef(def_to).stringtype of
                                st_unicodestring: eq:=te_equal;
                                st_widestring: eq:=te_convert_l1;
                                // widechar: eq:=te_convert_l2;
                                // ansichar: eq:=te_convert_l3;
                                st_ansistring: eq:=te_convert_l4;
                                st_shortstring: eq:=te_convert_l5;
                              else
                                eq:=te_convert_l6;
                              end
                            else if not(cs_refcountedstrings in current_settings.localswitches) and
                               (tstringdef(def_to).stringtype=st_shortstring) then
                              eq:=te_equal
                            else if not(m_default_unicodestring in current_settings.modeswitches) and
                               (cs_refcountedstrings in current_settings.localswitches) and
                               (tstringdef(def_to).stringtype=st_ansistring) then
                              eq:=te_equal
                            else if tstringdef(def_to).stringtype in [st_widestring,st_unicodestring] then
                              eq:=te_convert_l3
                            else
                              eq:=te_convert_l1;
                          end
                        else
                          begin
                          doconv:=tc_chararray_2_string;
                          if is_open_array(def_from) then
                            begin
                              if is_ansistring(def_to) then
                                eq:=te_convert_l1
                              else if is_wide_or_unicode_string(def_to) then
                                eq:=te_convert_l3
                              else
                                eq:=te_convert_l2;
                            end
                          else
                            begin
                              if is_shortstring(def_to) then
                                begin
                                  { Only compatible with arrays that fit
                                    smaller than 255 chars }
                                  if (def_from.size <= 255) then
                                    eq:=te_convert_l1;
                                end
                              else if is_ansistring(def_to) then
                                begin
                                  if (def_from.size > 255) then
                                    eq:=te_convert_l1
                                  else
                                    eq:=te_convert_l2;
                                end
                              else if is_wide_or_unicode_string(def_to) then
                                eq:=te_convert_l3
                              else
                                eq:=te_convert_l2;
                            end;
                          end;
                      end
                     else
                     { array of widechar to string, the length check is done by the firstpass of this node }
                      if is_widechararray(def_from) or is_open_widechararray(def_from) then
                       begin
                         doconv:=tc_chararray_2_string;
                         if is_wide_or_unicode_string(def_to) then
                           eq:=te_convert_l1
                         else
                           { size of widechar array is double due the sizeof a widechar }
                           if not(is_shortstring(def_to) and (is_open_widechararray(def_from) or (def_from.size>255*sizeof(widechar)))) then
                             eq:=te_convert_l3
                         else
                           eq:=te_convert_l2;
                       end;
                   end;
                 pointerdef :
                   begin
                   { pchar can be assigned to short/ansistrings,
                     but not in tp7 compatible mode }
                     if not(m_tp7 in current_settings.modeswitches) then
                       begin
                          if is_pchar(def_from) then
                           begin
                             doconv:=tc_pchar_2_string;
                             { prefer ansistrings/unicodestrings because pchars
                               can overflow shortstrings; don't use l1/l2/l3
                               because then pchar -> ansistring has the same
                               preference as conststring -> pchar, and this
                               breaks webtbs/tw3328.pp }
                             if is_ansistring(def_to) then
                               eq:=te_convert_l2
                             else if is_wide_or_unicode_string(def_to) then
                               eq:=te_convert_l3
                             else
                              eq:=te_convert_l4
                           end
                          else if is_pwidechar(def_from) then
                           begin
                             doconv:=tc_pwchar_2_string;
                             if is_wide_or_unicode_string(def_to) then
                               eq:=te_convert_l1
                             else
                               { shortstring and ansistring can both result in
                                 data loss, so don't prefer one over the other }
                               eq:=te_convert_l3;
                           end;
                       end;
                   end;
                 objectdef :
                   begin
                     { corba interface -> id string }
                     if is_interfacecorba(def_from) then
                      begin
                        doconv:=tc_intf_2_string;
                        eq:=te_convert_l1;
                      end
                     else if (def_from=java_jlstring) then
                       begin
                         if is_wide_or_unicode_string(def_to) then
                           begin
                             doconv:=tc_equal;
                             eq:=te_equal;
                           end
                         else if def_to.typ=stringdef then
                           begin
                             doconv:=tc_string_2_string;
                             if is_ansistring(def_to) then
                               eq:=te_convert_l2
                             else
                               eq:=te_convert_l3
                           end;
                      end;
                   end;
                 else
                   ;
               end;
             end;

           floatdef :
             begin
               case def_from.typ of
                 orddef :
                   begin { ordinal to real }
                     { only for implicit and internal typecasts in tp }
                     if (([cdo_explicit,cdo_internal] * cdoptions <> [cdo_explicit]) or
                         (not(m_tp7 in current_settings.modeswitches))) and
                        (is_integer(def_from) or
                         (is_currency(def_from) and
                          (s64currencytype.typ = floatdef))) then
                       begin
                         doconv:=tc_int_2_real;

                         { prefer single over others }
                         if is_single(def_to) then
                           eq:=te_convert_l3
                         else
                           eq:=te_convert_l4;
                       end
                     else if is_currency(def_from)
                             { and (s64currencytype.typ = orddef)) } then
                       begin
                         { prefer conversion to orddef in this case, unless    }
                         { the orddef < currency (then it will get convert l3, }
                         { and conversion to float is favoured)                }
                         doconv:=tc_int_2_real;
                         if is_extended(def_to) then
                           eq:=te_convert_l1
                         else if is_double(def_to) then
                           eq:=te_convert_l2
                         else if is_single(def_to) then
                           eq:=te_convert_l3
                         else
                           eq:=te_convert_l2;
                       end;
                   end;
                 floatdef :
                   begin
                     if tfloatdef(def_from).floattype=tfloatdef(def_to).floattype then
                       eq:=te_equal
                     else
                       begin
                         { Delphi does not allow explicit type conversions for float types like:
                             single_var:=single(double_var);
                           But if such conversion is inserted by compiler (internal) for some purpose,
                           it should be allowed even in Delphi mode. }
                         if (fromtreetype=realconstn) or
                            not((cdoptions*[cdo_explicit,cdo_internal]=[cdo_explicit]) and
                                (m_delphi in current_settings.modeswitches)) then
                           begin
                             doconv:=tc_real_2_real;
                             { do we lose precision? }
                             if (def_to.size<def_from.size) or
                               (is_currency(def_from) and (tfloatdef(def_to).floattype in [s32real,s64real])) then
                               begin
                                 if is_currency(def_from) and (tfloatdef(def_to).floattype=s32real) then
                                   eq:=te_convert_l3
                                 else
                                   eq:=te_convert_l2
                               end
                             else
                               eq:=te_convert_l1;
                           end;
                       end;
                   end;
                 else
                   ;
               end;
             end;

           enumdef :
             begin
               case def_from.typ of
                 enumdef :
                   begin
                     if cdo_explicit in cdoptions then
                      begin
                        eq:=te_convert_l1;
                        doconv:=tc_int_2_int;
                      end
                     else
                      begin
                        hd1:=def_from;
                        while assigned(tenumdef(hd1).basedef) do
                          hd1:=tenumdef(hd1).basedef;
                        hd2:=def_to;
                        while assigned(tenumdef(hd2).basedef) do
                          hd2:=tenumdef(hd2).basedef;
                        if (hd1=hd2) then
                          begin
                            eq:=te_convert_l1;
                            { because of packenum they can have different sizes! (JM) }
                            doconv:=tc_int_2_int;
                          end
                        else
                          begin
                            { assignment of an enum symbol to an unique type? }
                            if (fromtreetype=ordconstn) and
                              (tenumsym(tenumdef(hd1).getfirstsym)=tenumsym(tenumdef(hd2).getfirstsym)) then
                              begin
                                { because of packenum they can have different sizes! (JM) }
                                eq:=te_convert_l1;
                                doconv:=tc_int_2_int;
                              end;
                          end;
                      end;
                   end;
                 orddef :
                   begin
                     if cdo_explicit in cdoptions then
                      begin
                        eq:=te_convert_l1;
                        doconv:=tc_int_2_int;
                      end;
                   end;
                 variantdef :
                   begin
                     eq:=te_convert_l1;
                     doconv:=tc_variant_2_enum;
                   end;
                 pointerdef :
                   begin
                     { ugly, but delphi allows it }
                     if cdo_explicit in cdoptions then
                       begin
                         if target_info.system in systems_jvm then
                           begin
                             doconv:=tc_equal;
                             eq:=te_convert_l1;
                           end
                         else if m_delphi in current_settings.modeswitches then
                           begin
                             doconv:=tc_int_2_int;
                             eq:=te_convert_l1;
                           end
                       end;
                   end;
                 objectdef:
                   begin
                     { ugly, but delphi allows it }
                     if (cdo_explicit in cdoptions) and
                        is_class_or_interface_or_dispinterface_or_objc_or_java(def_from) then
                       begin
                         { in Java enums /are/ class instances, and hence such
                           typecasts must not be treated as integer-like
                           conversions
                         }
                         if target_info.system in systems_jvm then
                           begin
                             doconv:=tc_equal;
                             eq:=te_convert_l1;
                           end
                         else if m_delphi in current_settings.modeswitches then
                           begin
                             doconv:=tc_int_2_int;
                             eq:=te_convert_l1;
                           end;
                       end;
                   end;
                 else
                   ;
               end;
             end;

           arraydef :
             begin
               { open array is also compatible with a single element of its base type.
                 the extra check for deftyp is needed because equal defs can also return
                 true if the def types are not the same, for example with dynarray to pointer. }
               if is_open_array(def_to) and
                  (def_from.typ=tarraydef(def_to).elementdef.typ) and
                  equal_defs(def_from,tarraydef(def_to).elementdef) then
                begin
                  doconv:=tc_elem_2_openarray;
                  { also update in htypechk.pas/var_para_allowed if changed
                    here }
                  eq:=te_convert_l3;
                end
               else
                begin
                  case def_from.typ of
                    arraydef :
                      begin
                        { from/to packed array -- packed chararrays are      }
                        { strings in ISO Pascal (at least if the lower bound }
                        { is 1, but GPC makes all equal-length chararrays    }
                        { compatible), so treat those the same as regular    }
                        { char arrays -- except if they use subrange types   }
                        if (is_packed_array(def_from) and
                            (not is_chararray(def_from) or
                             (tarraydef(def_from).elementdef.packedbitsize<>8)) and
                            not is_widechararray(def_from)) xor
                           (is_packed_array(def_to) and
                            (not is_chararray(def_to) or
                             (tarraydef(def_to).elementdef.packedbitsize<>8)) and
                            not is_widechararray(def_to)) then
                          { both must be packed }
                          begin
                            compare_defs_ext:=te_incompatible;
                            exit;
                          end
                        { to dynamic array }
                        else if is_dynamic_array(def_to) then
                         begin
                           if is_array_constructor(def_from) then
                             begin
                               { array constructor -> dynamic array }
                               if is_void(tarraydef(def_from).elementdef) then
                                 begin
                                   { only needs to loose to [] -> open array }
                                   eq:=te_convert_l2;
                                   doconv:=tc_arrayconstructor_2_dynarray;
                                 end
                               else
                                 begin
                                   { this should loose to the array constructor -> open array conversions,
                                     but it might happen that the end of the convert levels is reached :/ }
                                   subeq:=compare_defs_ext(tarraydef(def_from).elementdef,
                                                        tarraydef(def_to).elementdef,
                                                        { reason for cdo_allow_variant: see webtbs/tw7070a and webtbs/tw7070b }
                                                        arrayconstructorn,hct,hpd,[cdo_check_operator,cdo_allow_variant]);
                                   if (subeq>=te_equal) then
                                     begin
                                       eq:=te_convert_l2;
                                     end
                                   else
                                     { an array constructor is not a dynamic array, so
                                       use a lower level of compatibility than that one of
                                       of the elements }
                                     if subeq>te_convert_l5 then
                                      begin
                                        eq:=pred(pred(subeq));
                                      end
                                    else if subeq>te_convert_l6 then
                                      eq:=pred(subeq)
                                    else if subeq=te_convert_operator then
                                      { the operater needs to be applied by element, so we tell
                                        the caller that it's some unpreffered conversion and let
                                        it handle the per-element stuff }
                                      eq:=te_convert_l6
                                    else
                                      eq:=subeq;
                                   doconv:=tc_arrayconstructor_2_dynarray;
                                 end;
                             end
                           else if equal_defs(tarraydef(def_from).elementdef,tarraydef(def_to).elementdef) then
                             begin
                               { dynamic array -> dynamic array }
                               if is_dynamic_array(def_from) then
                                 eq:=te_equal
                               { regular array -> dynamic array }
                               else if (m_array2dynarray in current_settings.modeswitches) and
                                 not(is_special_array(def_from)) and
                                 is_zero_based_array(def_from) then
                                 begin
                                   eq:=te_convert_l2;
                                   doconv:=tc_array_2_dynarray;
                                 end;
                             end
                         end
                        else
                         { to open array }
                         if is_open_array(def_to) then
                          begin
                            { array constructor -> open array }
                            if is_array_constructor(def_from) then
                             begin
                               if is_void(tarraydef(def_from).elementdef) then
                                begin
                                  doconv:=tc_equal;
                                  eq:=te_convert_l1;
                                end
                               else
                                begin
                                  subeq:=compare_defs_ext(tarraydef(def_from).elementdef,
                                                       tarraydef(def_to).elementdef,
                                                       { reason for cdo_allow_variant: see webtbs/tw7070a and webtbs/tw7070b }
                                                       arrayconstructorn,hct,hpd,[cdo_check_operator,cdo_allow_variant]);
                                  if (subeq>=te_equal) then
                                    begin
                                      doconv:=tc_equal;
                                      eq:=te_convert_l1;
                                    end
                                  else
                                    { an array constructor is not an open array, so
                                      use a lower level of compatibility than that one of
                                      of the elements }
                                    if subeq>te_convert_l6 then
                                     begin
                                       doconv:=hct;
                                       eq:=pred(subeq);
                                     end
                                   else
                                     eq:=subeq;
                                end;
                             end
                            else
                             { dynamic array -> open array }
                             if is_dynamic_array(def_from) and
                                equal_defs(tarraydef(def_from).elementdef,tarraydef(def_to).elementdef) then
                               begin
                                 doconv:=tc_dynarray_2_openarray;
                                 eq:=te_convert_l2;
                               end
                            else
                             { open array -> open array }
                             if is_open_array(def_from) and
                                equal_defs(tarraydef(def_from).elementdef,tarraydef(def_to).elementdef) then
                               if tarraydef(def_from).elementdef=tarraydef(def_to).elementdef then
                                 eq:=te_exact
                               else
                                 eq:=te_equal
                            else
                             { array -> open array }
                             if not(cdo_parameter in cdoptions) and
                                equal_defs(tarraydef(def_from).elementdef,tarraydef(def_to).elementdef) then
                               begin
                                 if fromtreetype=stringconstn then
                                   eq:=te_convert_l1
                                 else
                                   eq:=te_equal;
                               end;
                          end
                        else
                         { to array of const }
                         if is_array_of_const(def_to) then
                          begin
                            if is_array_of_const(def_from) or
                               is_array_constructor(def_from) then
                             begin
                               eq:=te_equal;
                             end
                            else
                             { array of tvarrec -> array of const }
                             if equal_defs(tarraydef(def_to).elementdef,tarraydef(def_from).elementdef) then
                              begin
                                doconv:=tc_equal;
                                eq:=te_convert_l1;
                              end;
                          end
                        else
                          { to array of char, from "Untyped" stringconstn (array of char) }
                          if (fromtreetype=stringconstn) and
                             ((is_chararray(def_to) and
                               { bitpacked arrays of char whose element bitsize is not
                                 8 cannot be auto-converted from strings }
                               (not is_packed_array(def_to) or
                                (tarraydef(def_to).elementdef.packedbitsize=8))) or
                              is_widechararray(def_to)) then
                            begin
                              eq:=te_convert_l1;
                              doconv:=tc_string_2_chararray;
                            end
                        else
                          { to normal array }
                          if is_normal_array(def_to) and is_array_constructor(def_from) then
                            begin
                              { element count must match exactly }
                              if tarraydef(def_to).elecount=tarraydef(def_from).elecount then
                                begin
                                  eq:=te_convert_l2;
                                  doconv:=tc_arrayconstructor_2_array;
                                end;
                            end
                        else
                         { other arrays }
                          begin
                            { open array -> array }
                            if not(cdo_parameter in cdoptions) and
                               is_open_array(def_from) and
                               equal_defs(tarraydef(def_from).elementdef,tarraydef(def_to).elementdef) then
                              begin
                                eq:=te_equal
                              end
                            else
                            { array -> array }
                             if not(m_tp7 in current_settings.modeswitches) and
                                not(m_delphi in current_settings.modeswitches) and
                                (tarraydef(def_from).lowrange=tarraydef(def_to).lowrange) and
                                (tarraydef(def_from).highrange=tarraydef(def_to).highrange) and
                                equal_defs(tarraydef(def_from).elementdef,tarraydef(def_to).elementdef) and
                                equal_defs(tarraydef(def_from).rangedef,tarraydef(def_to).rangedef) then
                              begin
                                eq:=te_equal
                              end;
                          end;
                      end;
                    pointerdef :
                      begin
                        { nil and voidpointers are compatible with dyn. arrays }
                        if is_dynamic_array(def_to) and
                           ((fromtreetype=niln) or
                            is_voidpointer(def_from)) then
                         begin
                           doconv:=tc_equal;
                           eq:=te_convert_l1;
                         end
                        else
                         if is_zero_based_array(def_to) and
                            equal_defs(tpointerdef(def_from).pointeddef,tarraydef(def_to).elementdef) then
                          begin
                            doconv:=tc_pointer_2_array;
                            eq:=te_convert_l1;
                          end;
                      end;
                    stringdef :
                      begin
                        { string to char array }
                        if not is_special_array(def_to) and
                           ((is_char(tarraydef(def_to).elementdef) and
                             { bitpacked arrays of char whose element bitsize is not
                               8 cannot be auto-converted from strings }
                             (not is_packed_array(def_to) or
                              (tarraydef(def_to).elementdef.packedbitsize=8))) or
                            is_widechar(tarraydef(def_to).elementdef)) then
                         begin
                           doconv:=tc_string_2_chararray;
                           eq:=te_convert_l1;
                         end;
                      end;
                    orddef:
                      begin
                        if is_chararray(def_to) and
                           is_char(def_from) then
                          begin
                            doconv:=tc_char_2_chararray;
                            eq:=te_convert_l2;
                          end;
                      end;
                    recorddef :
                      begin
                        { tvarrec -> array of const }
                         if is_array_of_const(def_to) and
                            equal_defs(def_from,tarraydef(def_to).elementdef) then
                          begin
                            doconv:=tc_equal;
                            eq:=te_convert_l1;
                          end;
                      end;
                    variantdef :
                      begin
                         if is_dynamic_array(def_to) then
                           begin
                              doconv:=tc_variant_2_dynarray;
                              eq:=te_convert_l1;
                           end;
                      end;
                    setdef :
                      begin
                        { special case: an empty set constant is compatible as
                          well }
                        if not assigned(tsetdef(def_from).elementdef)
                            and (fromtreetype=setconstn) then
                          begin
                            doconv:=tc_arrayconstructor_2_dynarray;
                            eq:=te_convert_l1;
                          end;
                      end;
                    else
                      ;
                  end;
                end;
             end;

           variantdef :
             begin
               if (cdo_allow_variant in cdoptions) then
                 begin
                   case def_from.typ of
                     enumdef :
                       begin
                         doconv:=tc_enum_2_variant;
                         eq:=te_convert_l1;
                       end;
                     arraydef :
                       begin
                          if is_dynamic_array(def_from) then
                            begin
                               doconv:=tc_dynarray_2_variant;
                               eq:=te_convert_l1;
                            end;
                       end;
                     objectdef :
                       begin
                         { corbainterfaces not accepted, until we have
                           runtime support for them in Variants (sergei) }
                          if is_interfacecom_or_dispinterface(def_from) then
                            begin
                               doconv:=tc_interface_2_variant;
                               eq:=te_convert_l1;
                            end;
                       end;
                     variantdef :
                       begin
                         { doing this in the compiler avoids a lot of unncessary
                           copying }
                         if (tvariantdef(def_from).varianttype=vt_olevariant) and
                           (tvariantdef(def_to).varianttype=vt_normalvariant) then
                           begin
                             doconv:=tc_equal;
                             eq:=te_convert_l1;
                           end;
                       end;
                     else
                       ;
                   end;
                 end;
             end;

           pointerdef :
             begin
               case def_from.typ of
                 stringdef :
                   begin
                     { string constant (which can be part of array constructor)
                       to zero terminated string constant }
                     if (fromtreetype = stringconstn) and
                        (is_pchar(def_to) or is_pwidechar(def_to)) then
                      begin
                        doconv:=tc_cstring_2_pchar;
                        if is_pwidechar(def_to)=(m_default_unicodestring in current_settings.modeswitches) then
                          eq:=te_convert_l2
                        else
                          eq:=te_convert_l3
                      end
                     else
                      if (cdo_explicit in cdoptions) or (fromtreetype = arrayconstructorn) then
                       begin
                         { pchar(ansistring) }
                         if is_pchar(def_to) and
                            is_ansistring(def_from) then
                          begin
                            doconv:=tc_ansistring_2_pchar;
                            eq:=te_convert_l1;
                          end
                         else
                          { pwidechar(widestring) }
                          if is_pwidechar(def_to) and
                            is_wide_or_unicode_string(def_from) then
                           begin
                             doconv:=tc_ansistring_2_pchar;
                             eq:=te_convert_l1;
                           end;
                       end;
                   end;
                 orddef :
                   begin
                     { char constant to zero terminated string constant }
                     if (fromtreetype in [ordconstn,arrayconstructorn]) then
                      begin
                        if (is_char(def_from) or is_widechar(def_from)) and
                           (is_pchar(def_to) or is_pwidechar(def_to)) then
                         begin
                           doconv:=tc_cchar_2_pchar;
                           if is_pwidechar(def_to)=(m_default_unicodestring in current_settings.modeswitches) then
                             eq:=te_convert_l1
                           else
                             eq:=te_convert_l2
                         end
                        else
                         if (m_delphi in current_settings.modeswitches) and is_integer(def_from) then
                          begin
                            doconv:=tc_cord_2_pointer;
                            eq:=te_convert_l5;
                          end;
                      end;
                     { allow explicit typecasts from ordinals to pointer.
                       Support for delphi compatibility
                       Support constructs like pointer(cardinal-cardinal) or pointer(longint+cardinal) where
                        the result of the ordinal operation is int64 also on 32 bit platforms.
                       It is also used by the compiler internally for inc(pointer,ordinal) }
                     if (eq=te_incompatible) and
                        not is_void(def_from) and
                        (
                         (
                          (cdo_explicit in cdoptions) and
                          (
                           (m_delphi in current_settings.modeswitches) or
                           { Don't allow pchar(char) in fpc modes }
                           is_integer(def_from)
                          )
                         ) or
                         (cdo_internal in cdoptions)
                        ) then
                       begin
                         doconv:=tc_int_2_int;
                         eq:=te_convert_l1;
                       end;
                   end;
                 enumdef :
                   begin
                     { allow explicit typecasts from enums to pointer.
                       Support for delphi compatibility
                     }
                     { in Java enums /are/ class instances, and hence such
                       typecasts must not be treated as integer-like conversions
                     }
                     if (((cdo_explicit in cdoptions) and
                          ((m_delphi in current_settings.modeswitches) or
                           (target_info.system in systems_jvm)
                          )
                         ) or
                         (cdo_internal in cdoptions)
                        ) then
                       begin
                         { in Java enums /are/ class instances, and hence such
                           typecasts must not be treated as integer-like
                           conversions
                         }
                         if target_info.system in systems_jvm then
                           begin
                             doconv:=tc_equal;
                             eq:=te_convert_l1;
                           end
                         else if m_delphi in current_settings.modeswitches then
                           begin
                             doconv:=tc_int_2_int;
                             eq:=te_convert_l1;
                           end;
                       end;
                   end;
                 arraydef :
                   begin
                     { string constant (which can be part of array constructor)
                       to zero terminated string constant }
                     if (((fromtreetype = arrayconstructorn) and
                          { can't use is_chararray, because returns false for }
                          { array constructors                                }
                          is_char(tarraydef(def_from).elementdef)) or
                         (fromtreetype = stringconstn)) and
                        (is_pchar(def_to) or is_pwidechar(def_to)) then
                      begin
                        doconv:=tc_cstring_2_pchar;
                        if ((m_default_unicodestring in current_settings.modeswitches) xor
                           is_pchar(def_to)) then
                          eq:=te_convert_l2
                        else
                          eq:=te_convert_l3;
                      end
                     else
                      { chararray to pointer }
                      if (is_zero_based_array(def_from) or
                          is_open_array(def_from)) and
                          equal_defs(tarraydef(def_from).elementdef,tpointerdef(def_to).pointeddef) then
                        begin
                          doconv:=tc_array_2_pointer;
                          { don't prefer the pchar overload when a constant
                            string was passed }
                          if fromtreetype=stringconstn then
                            eq:=te_convert_l2
                          else
                            eq:=te_convert_l1;
                        end
                     else
                       { dynamic array to pointer, delphi only }
                       if (m_delphi in current_settings.modeswitches) and
                          is_dynamic_array(def_from) and
                          is_voidpointer(def_to) then
                        begin
                          eq:=te_equal;
                        end;
                   end;
                 pointerdef :
                   begin
                     { check for far pointers }
                     if not tpointerdef(def_from).compatible_with_pointerdef_size(tpointerdef(def_to)) then
                       begin
                         if fromtreetype=niln then
                           eq:=te_equal
                         else
                           eq:=te_incompatible;
                       end
                     { the types can be forward type, handle before normal type check !! }
                     else
                      if assigned(def_to.typesym) and
                         ((tpointerdef(def_to).pointeddef.typ=forwarddef) or
                          (tpointerdef(def_from).pointeddef.typ=forwarddef)) then
                       begin
                         if (def_from.typesym=def_to.typesym) or
                            (fromtreetype=niln) then
                          eq:=te_equal
                       end
                     else
                      { same types }
                      if equal_defs(tpointerdef(def_from).pointeddef,tpointerdef(def_to).pointeddef) then
                       begin
                         eq:=te_equal
                       end
                     else
                      { child class pointer can be assigned to anchestor pointers }
                      if (
                          (tpointerdef(def_from).pointeddef.typ=objectdef) and
                          (tpointerdef(def_to).pointeddef.typ=objectdef) and
                          def_is_related(tobjectdef(tpointerdef(def_from).pointeddef),
                            tobjectdef(tpointerdef(def_to).pointeddef))
                         ) then
                       begin
                         doconv:=tc_equal;
                         eq:=te_convert_l1;
                       end
                     else
                      { all pointers can be assigned to void-pointer }
                      if is_void(tpointerdef(def_to).pointeddef) then
                       begin
                         doconv:=tc_equal;
                         { give pwidechar,pchar a penalty so it prefers
                           conversion to ansistring }
                         if is_pchar(def_from) or
                            is_pwidechar(def_from) then
                           eq:=te_convert_l2
                         else
                           eq:=te_convert_l1;
                       end
                     else
                      { all pointers can be assigned from void-pointer }
                      if is_void(tpointerdef(def_from).pointeddef) or
                      { all pointers can be assigned from void-pointer or formaldef pointer, check
                        tw3777.pp if you change this }
                        (tpointerdef(def_from).pointeddef.typ=formaldef) then
                       begin
                         doconv:=tc_equal;
                         { give pwidechar a penalty so it prefers
                           conversion to pchar }
                         if is_pwidechar(def_to) then
                           eq:=te_convert_l2
                         else
                           eq:=te_convert_l1;
                       end
                     { id = generic class instance. metaclasses are also
                       class instances themselves.  }
                     else if ((def_from=objc_idtype) and
                              (def_to=objc_metaclasstype)) or
                             ((def_to=objc_idtype) and
                              (def_from=objc_metaclasstype)) then
                       begin
                         doconv:=tc_equal;
                         eq:=te_convert_l2;
                       end;
                   end;
                 procvardef :
                   begin
                     { procedure variable can be assigned to an void pointer,
                       this is not allowed for complex procvars }
                     if (is_void(tpointerdef(def_to).pointeddef) or
                         (m_mac_procvar in current_settings.modeswitches)) and
                        tprocvardef(def_from).compatible_with_pointerdef_size(tpointerdef(def_to)) then
                      begin
                        doconv:=tc_equal;
                        eq:=te_convert_l1;
                      end;
                   end;
                 procdef :
                   begin
                     { procedure variable can be assigned to an void pointer,
                       this not allowed for methodpointers }
                     if (m_mac_procvar in current_settings.modeswitches) and
                        tprocdef(def_from).compatible_with_pointerdef_size(tpointerdef(def_to)) then
                      begin
                        doconv:=tc_proc_2_procvar;
                        eq:=te_convert_l2;
                      end;
                   end;
                 classrefdef,
                 objectdef :
                   begin
                     { implicit pointer object and class reference types
                       can be assigned to void pointers, but it is less
                       preferred than assigning to a related objectdef }
                     if (
                         is_implicit_pointer_object_type(def_from) or
                         (def_from.typ=classrefdef)
                        ) and
                        (tpointerdef(def_to).pointeddef.typ=orddef) and
                        (torddef(tpointerdef(def_to).pointeddef).ordtype=uvoid) then
                       begin
                         doconv:=tc_equal;
                         eq:=te_convert_l2;
                       end
                     else if (is_objc_class_or_protocol(def_from) and
                              (def_to=objc_idtype)) or
                             { classrefs are also instances in Objective-C,
                               hence they're also assignment-cpmpatible with
                               id }
                             (is_objcclassref(def_from) and
                              ((def_to=objc_metaclasstype) or
                               (def_to=objc_idtype))) then
                       begin
                         doconv:=tc_equal;
                         eq:=te_convert_l2;
                       end;
                   end;
                 else
                   ;
               end;
             end;

           setdef :
             begin
               case def_from.typ of
                 setdef :
                   begin
                     if assigned(tsetdef(def_from).elementdef) and
                        assigned(tsetdef(def_to).elementdef) then
                      begin
                        { sets with the same size (packset setting), element
                          base type and the same range are equal }
                        if equal_defs(tsetdef(def_from).elementdef,tsetdef(def_to).elementdef) and
                           (tsetdef(def_from).setbase=tsetdef(def_to).setbase) and
                           (tsetdef(def_from).setmax=tsetdef(def_to).setmax) and
                           (def_from.size=def_to.size) then
                          eq:=te_equal
                        else if is_subequal(tsetdef(def_from).elementdef,tsetdef(def_to).elementdef) then
                          begin
                            eq:=te_convert_l1;
                            doconv:=tc_set_to_set;
                          end;
                      end
                     else
                      begin
                        { empty set is compatible with everything }
                        eq:=te_convert_l1;
                        doconv:=tc_set_to_set;
                      end;
                   end;
                 arraydef :
                   begin
                     { automatic arrayconstructor -> set conversion }
                     if is_array_constructor(def_from) then
                      begin
                        doconv:=tc_arrayconstructor_2_set;
                        eq:=te_convert_l1;
                      end;
                   end;
                 else
                   ;
               end;
             end;

           procvardef :
             begin
               case def_from.typ of
                 procdef :
                   begin
                     { proc -> procvar }
                     if (m_tp_procvar in current_settings.modeswitches) or
                        (m_mac_procvar in current_settings.modeswitches) then
                      begin
                        subeq:=proc_to_procvar_equal(tprocdef(def_from),tprocvardef(def_to),cdo_warn_incompatible_univ in cdoptions);
                        if subeq>te_incompatible then
                         begin
                           doconv:=tc_proc_2_procvar;
                           if subeq>te_convert_l5 then
                             eq:=pred(subeq)
                           else
                             eq:=subeq;
                         end;
                      end;
                   end;
                 procvardef :
                   begin
                     { procvar -> procvar }
                     eq:=proc_to_procvar_equal(tprocvardef(def_from),tprocvardef(def_to),cdo_warn_incompatible_univ in cdoptions);
                     if eq<te_equal then
                       doconv:=tc_proc_2_procvar
                     else
                       doconv:=tc_equal;
                   end;
                 pointerdef :
                   begin
                     { nil is compatible with procvars }
                     if (fromtreetype=niln) then
                      begin
                        if not Tprocvardef(def_to).is_addressonly then
                          {Nil to method pointers requires to convert a single
                           pointer nil value to a two pointer procvardef.}
                          doconv:=tc_nil_2_methodprocvar
                        else
                          doconv:=tc_equal;
                        eq:=te_convert_l1;
                      end
                     else
                      { for example delphi allows the assignement from pointers }
                      { to procedure variables                                  }
                      if (m_pointer_2_procedure in current_settings.modeswitches) and
                         is_void(tpointerdef(def_from).pointeddef) and
                         tprocvardef(def_to).is_addressonly and
                         tprocvardef(def_to).compatible_with_pointerdef_size(tpointerdef(def_from)) then
                       begin
                         doconv:=tc_equal;
                         eq:=te_convert_l1;
                       end;
                   end;
                 else
                   ;
               end;
             end;

           objectdef :
             begin
               { object pascal objects }
               { don't call def_is_related if we came here from equal_defs, because
                   1) this can never result in an "equal result", and
                   2) def_is_related itself calls equal_defs again for each class in
                      the hierarchy, which will call compare_defs_ext, which will again
                      call def_is_related -> quadratic complexity explosion }
               if not(cdo_equal_check in cdoptions) and
                  (def_from.typ=objectdef) and
                  (def_is_related(tobjectdef(def_from),tobjectdef(def_to))) then
                begin
                  doconv:=tc_equal;
                  { also update in htypechk.pas/var_para_allowed if changed
                    here }
                  eq:=te_convert_l3;
                end
               { string -> java.lang.string }
               else if (def_to=java_jlstring) and
                       ((def_from.typ=stringdef) or
                        (fromtreetype=stringconstn)) then
                 begin
                   if is_wide_or_unicode_string(def_from) or
                      ((fromtreetype=stringconstn) and
                       (cs_refcountedstrings in current_settings.localswitches) and
                       (m_default_unicodestring in current_settings.modeswitches)) then
                     begin
                       doconv:=tc_equal;
                       eq:=te_equal
                     end
                   else
                     begin
                       doconv:=tc_string_2_string;
                       eq:=te_convert_l2;
                     end;
                 end
               else if (def_to=java_jlstring) and
                       is_anychar(def_from) then
                 begin
                   doconv:=tc_char_2_string;
                   eq:=te_convert_l2
                 end
               else
               { specific to implicit pointer object types }
                if is_implicit_pointer_object_type(def_to) then
                 begin
                   { void pointer also for delphi mode }
                   if (m_delphi in current_settings.modeswitches) and
                      is_voidpointer(def_from) then
                    begin
                      doconv:=tc_equal;
                      { prefer pointer-pointer assignments }
                      eq:=te_convert_l2;
                    end
                   else
                   { nil is compatible with class instances and interfaces }
                    if (fromtreetype=niln) then
                     begin
                       doconv:=tc_equal;
                       eq:=te_convert_l1;
                     end
                   { All Objective-C classes are compatible with ID }
                   else if is_objc_class_or_protocol(def_to) and
                           (def_from=objc_idtype) then
                      begin
                       doconv:=tc_equal;
                       eq:=te_convert_l2;
                     end
                   { classes can be assigned to interfaces
                     (same with objcclass and objcprotocol) }
                   else if ((is_interface(def_to) and
                             is_class(def_from)) or
                            (is_objcprotocol(def_to) and
                             is_objcclass(def_from)) or
                            (is_javainterface(def_to) and
                             is_javaclass(def_from))) and
                           assigned(tobjectdef(def_from).ImplementedInterfaces) then
                     begin
                        { we've to search in parent classes as well }
                        hobjdef:=tobjectdef(def_from);
                        while assigned(hobjdef) do
                          begin
                             if find_implemented_interface(hobjdef,tobjectdef(def_to))<>nil then
                               begin
                                  if is_interface(def_to) then
                                    doconv:=tc_class_2_intf
                                  else
                                    { for Objective-C, we don't have to do anything special }
                                    doconv:=tc_equal;
                                  { don't prefer this over objectdef->objectdef }
                                  eq:=te_convert_l2;
                                  break;
                               end;
                             hobjdef:=hobjdef.childof;
                          end;
                     end
                   { Interface 2 GUID handling }
                   else if (def_to=tdef(rec_tguid)) and
                           (fromtreetype=typen) and
                           is_interface(def_from) and
                           assigned(tobjectdef(def_from).iidguid) then
                     begin
                       eq:=te_convert_l1;
                       doconv:=tc_equal;
                     end
                   else if (def_from.typ=variantdef) and is_interfacecom_or_dispinterface(def_to) then
                     begin
                     { corbainterfaces not accepted, until we have
                       runtime support for them in Variants (sergei) }
                       doconv:=tc_variant_2_interface;
                       eq:=te_convert_l2;
                     end
                   { ugly, but delphi allows it (enables typecasting ordinals/
                     enums of any size to pointer-based object defs) }
                   { in Java enums /are/ class instances, and hence such
                     typecasts must not be treated as integer-like conversions;
                     arbitrary constants cannot be converted into classes/
                     pointer-based values either on the JVM -> always return
                     false and let it be handled by the regular explicit type
                     casting code
                   }
                   else if (not(target_info.system in systems_jvm) and
                       ((def_from.typ=enumdef) or
                        (def_from.typ=orddef))) and
                      (m_delphi in current_settings.modeswitches) and
                      (cdo_explicit in cdoptions) then
                     begin
                       doconv:=tc_int_2_int;
                       eq:=te_convert_l1;
                     end;
                 end;
             end;

           classrefdef :
             begin
               { similar to pointerdef wrt forwards }
               if assigned(def_to.typesym) and
                  (tclassrefdef(def_to).pointeddef.typ=forwarddef) or
                  ((def_from.typ=classrefdef) and
                   (tclassrefdef(def_from).pointeddef.typ=forwarddef)) then
                 begin
                   if (def_from.typesym=def_to.typesym) or
                      (fromtreetype=niln) then
                    eq:=te_equal;
                 end
               else
                { class reference types }
                if (def_from.typ=classrefdef) then
                 begin
                   if equal_defs(tclassrefdef(def_from).pointeddef,tclassrefdef(def_to).pointeddef) then
                    begin
                      eq:=te_equal;
                    end
                   else
                    begin
                      doconv:=tc_equal;
                      if (cdo_explicit in cdoptions) or
                         def_is_related(tobjectdef(tclassrefdef(def_from).pointeddef),
                           tobjectdef(tclassrefdef(def_to).pointeddef)) then
                        eq:=te_convert_l1;
                    end;
                 end
               else
                 if (m_delphi in current_settings.modeswitches) and
                    is_voidpointer(def_from) then
                  begin
                    doconv:=tc_equal;
                    { prefer pointer-pointer assignments }
                    eq:=te_convert_l2;
                  end
                 else
                { nil is compatible with class references }
                if (fromtreetype=niln) then
                 begin
                   doconv:=tc_equal;
                   eq:=te_convert_l1;
                 end
               else
                 { id is compatible with all classref types }
                 if (def_from=objc_idtype) then
                   begin
                     doconv:=tc_equal;
                     eq:=te_convert_l1;
                   end;
             end;

           filedef :
             begin
               { typed files are all equal to the abstract file type
               name TYPEDFILE in system.pp in is_equal in types.pas
               the problem is that it sholud be also compatible to FILE
               but this would leed to a problem for ASSIGN RESET and REWRITE
               when trying to find the good overloaded function !!
               so all file function are doubled in system.pp
               this is not very beautiful !!}
               if (def_from.typ=filedef) then
                begin
                  if (tfiledef(def_from).filetyp=tfiledef(def_to).filetyp) then
                   begin
                     if
                        (
                         (tfiledef(def_from).typedfiledef=nil) and
                         (tfiledef(def_to).typedfiledef=nil)
                        ) or
                        (
                         (tfiledef(def_from).typedfiledef<>nil) and
                         (tfiledef(def_to).typedfiledef<>nil) and
                         equal_defs(tfiledef(def_from).typedfiledef,tfiledef(def_to).typedfiledef)
                        ) or
                        (
                         (tfiledef(def_from).filetyp = ft_typed) and
                         (tfiledef(def_to).filetyp = ft_typed) and
                         (
                          (tfiledef(def_from).typedfiledef = tdef(voidtype)) or
                          (tfiledef(def_to).typedfiledef = tdef(voidtype))
                         )
                        ) then
                      begin
                        eq:=te_equal;
                      end;
                   end
                  else
                   if ((tfiledef(def_from).filetyp = ft_untyped) and
                       (tfiledef(def_to).filetyp = ft_typed)) or
                      ((tfiledef(def_from).filetyp = ft_typed) and
                       (tfiledef(def_to).filetyp = ft_untyped)) then
                    begin
                      doconv:=tc_equal;
                      eq:=te_convert_l1;
                    end;
                end;
             end;

           recorddef :
             begin
               { interface -> guid }
               if (def_to=rec_tguid) and
                  (is_interfacecom_or_dispinterface(def_from)) then
                begin
                  doconv:=tc_intf_2_guid;
                  eq:=te_convert_l1;
                end;
             end;

           formaldef :
             begin
               doconv:=tc_equal;
               if (def_from.typ=formaldef) then
                 eq:=te_equal
               else
                { Just about everything can be converted to a formaldef...}
                if not (def_from.typ in [abstractdef,errordef]) then
                  eq:=te_convert_l6;
             end;
           else
             ;
        end;

        { if we didn't find an appropriate type conversion yet
          then we search also the := operator }
        if (eq=te_incompatible) and
           { make sure there is not a single variant if variants   }
           { are not allowed (otherwise if only cdo_check_operator }
           { and e.g. fromdef=stringdef and todef=variantdef, then }
           { the test will still succeed                           }
           ((cdo_allow_variant in cdoptions) or
            ((def_from.typ<>variantdef) and
             (def_to.typ<>variantdef) and
             { internal typeconversions always have to be bitcasts (except for
               variants) }
             not(cdo_internal in cdoptions)
            )
           ) and
           (
            { Check for variants? }
            (
             (cdo_allow_variant in cdoptions) and
             ((def_from.typ=variantdef) or (def_to.typ=variantdef))
            ) or
            { Check for operators? }
            (
             (cdo_check_operator in cdoptions) and
             ((def_from.typ<>variantdef) or (def_to.typ<>variantdef))
            )
           ) then
          begin
            operatorpd:=search_assignment_operator(def_from,def_to,cdo_explicit in cdoptions);
            if assigned(operatorpd) then
             eq:=te_convert_operator;
          end;

        { update convtype for te_equal when it is not yet set }
        if (eq=te_equal) and
           (doconv=tc_not_possible) then
          doconv:=tc_equal;

        compare_defs_ext:=eq;
      end;


    function equal_defs(def_from,def_to:tdef):boolean;
      var
        convtyp : tconverttype;
        pd : tprocdef;
      begin
        { Compare defs with nothingn and no explicit typecasts and
          searching for overloaded operators is not needed }
        equal_defs:=(compare_defs_ext(def_from,def_to,nothingn,convtyp,pd,[cdo_equal_check])>=te_equal);
      end;


    function compare_defs(def_from,def_to:tdef;fromtreetype:tnodetype):tequaltype;
      var
        doconv : tconverttype;
        pd : tprocdef;
      begin
        compare_defs:=compare_defs_ext(def_from,def_to,fromtreetype,doconv,pd,[cdo_check_operator,cdo_allow_variant]);
      end;


    function is_subequal(def1, def2: tdef): boolean;
      var
         basedef1,basedef2 : tenumdef;

      Begin
        is_subequal := false;
        if assigned(def1) and assigned(def2) then
         Begin
           if (def1.typ = orddef) and (def2.typ = orddef) then
            Begin
              { see p.47 of Turbo Pascal 7.01 manual for the separation of types }
              { range checking for case statements is done with adaptrange        }
              case torddef(def1).ordtype of
                u8bit,u16bit,u32bit,u64bit,
                s8bit,s16bit,s32bit,s64bit :
                  is_subequal:=(torddef(def2).ordtype in [s64bit,u64bit,s32bit,u32bit,u8bit,s8bit,s16bit,u16bit]);
                pasbool1,pasbool8,pasbool16,pasbool32,pasbool64,
                bool8bit,bool16bit,bool32bit,bool64bit :
                  is_subequal:=(torddef(def2).ordtype in [pasbool1,pasbool8,pasbool16,pasbool32,pasbool64,bool8bit,bool16bit,bool32bit,bool64bit]);
                uchar :
                  is_subequal:=(torddef(def2).ordtype=uchar);
                uwidechar :
                  is_subequal:=(torddef(def2).ordtype=uwidechar);
                customint:
                  is_subequal:=(torddef(def2).low=torddef(def1).low) and (torddef(def2).high=torddef(def1).high);
                u128bit, s128bit,
                scurrency,
                uvoid:
                  ;
              end;
            end
           else
            Begin
              { Check if both basedefs are equal }
              if (def1.typ=enumdef) and (def2.typ=enumdef) then
                Begin
                   { get both basedefs }
                   basedef1:=tenumdef(def1);
                   while assigned(basedef1.basedef) do
                     basedef1:=basedef1.basedef;
                   basedef2:=tenumdef(def2);
                   while assigned(basedef2.basedef) do
                     basedef2:=basedef2.basedef;
                   is_subequal:=(basedef1=basedef2);
                end;
            end;
         end;
      end;


    function potentially_incompatible_univ_paras(def1, def2: tdef): boolean;
      begin
        result :=
          { not entirely safe: different records can be passed differently
            depending on the types of their fields, but they're hard to compare
            (variant records, bitpacked vs non-bitpacked) }
          ((def1.typ in [floatdef,recorddef,arraydef,filedef,variantdef]) and
           (def1.typ<>def2.typ)) or
          { pointers, ordinals and small sets are all passed the same}
          (((def1.typ in [orddef,enumdef,pointerdef,procvardef,classrefdef]) or
            (is_class_or_interface_or_objc(def1)) or
            is_dynamic_array(def1) or
            is_smallset(def1) or
            is_ansistring(def1) or
            is_unicodestring(def1)) <>
           (def2.typ in [orddef,enumdef,pointerdef,procvardef,classrefdef]) or
            (is_class_or_interface_or_objc(def2)) or
            is_dynamic_array(def2) or
             is_smallset(def2) or
            is_ansistring(def2) or
            is_unicodestring(def2)) or
           { shortstrings }
           (is_shortstring(def1)<>
            is_shortstring(def2)) or
           { winlike widestrings }
           (is_widestring(def1)<>
            is_widestring(def2)) or
           { TP-style objects }
           (is_object(def1) <>
            is_object(def2));
      end;


    function compare_paras(para1,para2 : TFPObjectList; acp : tcompare_paras_type; cpoptions: tcompare_paras_options):tequaltype;

      var
        currpara1,
        currpara2 : tparavarsym;
        eq,lowesteq : tequaltype;
        hpd       : tprocdef;
        convtype  : tconverttype;
        cdoptions : tcompare_defs_options;
        i1,i2     : byte;
      begin
         compare_paras:=te_incompatible;
         cdoptions:=[cdo_parameter,cdo_check_operator,cdo_allow_variant,cdo_strict_undefined_check];
         { we need to parse the list from left-right so the
           not-default parameters are checked first }
         lowesteq:=high(tequaltype);
         i1:=0;
         i2:=0;
         if cpo_ignorehidden in cpoptions then
           begin
             while (i1<para1.count) and
                   (vo_is_hidden_para in tparavarsym(para1[i1]).varoptions) do
               inc(i1);
             while (i2<para2.count) and
                   (vo_is_hidden_para in tparavarsym(para2[i2]).varoptions) do
               inc(i2);
           end;
         if cpo_ignoreframepointer in cpoptions then
           begin
             if (i1<para1.count) and
                (vo_is_parentfp in tparavarsym(para1[i1]).varoptions) then
               inc(i1);
             if (i2<para2.count) and
                (vo_is_parentfp in tparavarsym(para2[i2]).varoptions) then
               inc(i2);
           end;
         while (i1<para1.count) and (i2<para2.count) do
           begin
             eq:=te_incompatible;

             currpara1:=tparavarsym(para1[i1]);
             currpara2:=tparavarsym(para2[i2]);

             { Unique types must match exact }
             if ((df_unique in currpara1.vardef.defoptions) or (df_unique in currpara2.vardef.defoptions)) and
                (currpara1.vardef<>currpara2.vardef) then
               exit;

             { Handle hidden parameters separately, because self is
               defined as voidpointer for methodpointers }
             if (vo_is_hidden_para in currpara1.varoptions) or
                (vo_is_hidden_para in currpara2.varoptions) then
              begin
                { both must be hidden }
                if (vo_is_hidden_para in currpara1.varoptions)<>(vo_is_hidden_para in currpara2.varoptions) then
                  exit;
                eq:=te_exact;
                if (([vo_is_self,vo_is_vmt]*currpara1.varoptions)=[]) and
                   (([vo_is_self,vo_is_vmt]*currpara2.varoptions)=[]) then
                 begin
                   if not(cpo_ignorevarspez in cpoptions) and
                      (currpara1.varspez<>currpara2.varspez) then
                    exit;
                   eq:=compare_defs_ext(currpara1.vardef,currpara2.vardef,nothingn,
                                        convtype,hpd,cdoptions);
                 end
                else if ([vo_is_self,vo_is_vmt]*currpara1.varoptions)<>
                         ([vo_is_self,vo_is_vmt]*currpara2.varoptions) then
                   eq:=te_incompatible;
              end
             else
              begin
                case acp of
                  cp_value_equal_const :
                    begin
                       { this one is used for matching parameters from a call
                         statement to a procdef -> univ state can't be equal
                         in any case since the call statement does not contain
                         any information about that }
                       if (
                           not(cpo_ignorevarspez in cpoptions) and
                           (currpara1.varspez<>currpara2.varspez) and
                           ((currpara1.varspez in [vs_var,vs_out,vs_constref]) or
                            (currpara2.varspez in [vs_var,vs_out,vs_constref]))
                          ) then
                         exit;
                       eq:=compare_defs_ext(currpara1.vardef,currpara2.vardef,nothingn,
                                            convtype,hpd,cdoptions);
                    end;
                  cp_all :
                    begin
                       { used to resolve forward definitions -> headers must
                         match exactly, including the "univ" specifier }
                       if (not(cpo_ignorevarspez in cpoptions) and
                           (currpara1.varspez<>currpara2.varspez)) or
                          (currpara1.univpara<>currpara2.univpara) then
                         exit;
                       eq:=compare_defs_ext(currpara1.vardef,currpara2.vardef,nothingn,
                                            convtype,hpd,cdoptions);
                    end;
                  cp_procvar :
                    begin
                       if not(cpo_ignorevarspez in cpoptions) and
                          (currpara1.varspez<>currpara2.varspez) then
                         exit;
                       { "univ" state doesn't matter here: from univ to non-univ
                          matches if the types are compatible (i.e., as usual),
                          from from non-univ to univ also matches if the types
                          have the same size (checked below) }
                       eq:=compare_defs_ext(currpara1.vardef,currpara2.vardef,nothingn,
                                            convtype,hpd,cdoptions);
                       { Parameters must be at least equal otherwise the are incompatible }
                       if (eq<te_equal) then
                         eq:=te_incompatible;
                    end;
                  else
                    eq:=compare_defs_ext(currpara1.vardef,currpara2.vardef,nothingn,
                                         convtype,hpd,cdoptions);
                 end;
               end;
              { check type }
              if eq=te_incompatible then
                begin
                  { special case: "univ" parameters match if their size is equal }
                  if not(cpo_ignoreuniv in cpoptions) and
                     currpara2.univpara and
                     is_valid_univ_para_type(currpara1.vardef) and
                     (currpara1.vardef.size=currpara2.vardef.size) then
                    begin
                      { only pick as last choice }
                      eq:=te_convert_l5;
                      if (acp=cp_procvar) and
                         (cpo_warn_incompatible_univ in cpoptions) then
                        begin
                          { if the types may be passed in different ways by the
                            calling convention then this can lead to crashes
                            (note: not an exhaustive check, and failing this
                             this check does not mean things will crash on all
                             platforms) }
                          if potentially_incompatible_univ_paras(currpara1.vardef,currpara2.vardef) then
                            Message2(type_w_procvar_univ_conflicting_para,currpara1.vardef.typename,currpara2.vardef.typename)
                        end;
                    end
                  else if (cpo_generic in cpoptions) then
                    begin
                      if equal_genfunc_paradefs(currpara1.vardef,currpara2.vardef,currpara1.owner,currpara2.owner) then
                        eq:=te_exact
                      else
                        exit;
                    end
                  else
                    exit;
                end;
              if (eq=te_equal) and
                  (cpo_generic in cpoptions) then
                begin
                  if is_open_array(currpara1.vardef) and
                      is_open_array(currpara2.vardef) then
                    begin
                      if equal_genfunc_paradefs(tarraydef(currpara1.vardef).elementdef,tarraydef(currpara2.vardef).elementdef,currpara1.owner,currpara2.owner) then
                        eq:=te_exact;
                    end
                  else
                    { for the purpose of forward declarations two equal specializations
                      are considered as exactly equal }
                    if (df_specialization in tstoreddef(currpara1.vardef).defoptions) and
                        (df_specialization in tstoreddef(currpara2.vardef).defoptions) then
                      eq:=te_exact;
                end;
              { open strings can never match exactly, since you cannot define }
              { a separate "open string" type -> we have to be able to        }
              { consider those as exact when resolving forward definitions.   }
              { The same goes for array of const. Open arrays are handled     }
              { already (if their element types match exactly, they are       }
              { considered to be an exact match)                              }
              { And also for "inline defined" function parameter definitions  }
              { (i.e., function types directly declared in a parameter list)  }
              if (is_array_of_const(currpara1.vardef) or
                  is_open_string(currpara1.vardef) or
                  ((currpara1.vardef.typ = procvardef) and
                   not(assigned(currpara1.vardef.typesym)))) and
                 (eq=te_equal) and
                 (cpo_openequalisexact in cpoptions) then
                eq:=te_exact;
              if eq<lowesteq then
                lowesteq:=eq;
              { also check default value if both have it declared }
              if (cpo_comparedefaultvalue in cpoptions) then
                begin
                  if assigned(currpara1.defaultconstsym) and
                     assigned(currpara2.defaultconstsym) then
                    begin
                      if not equal_constsym(tconstsym(currpara1.defaultconstsym),tconstsym(currpara2.defaultconstsym),true) then
                        exit;
                    end
                  { cannot have that the second (= implementation) has a default value declared and the
                    other (interface) doesn't }
                  else if not assigned(currpara1.defaultconstsym) and assigned(currpara2.defaultconstsym) then
                    exit;
                end;
              if not(cpo_compilerproc in cpoptions) and
                 not(cpo_rtlproc in cpoptions) and
                 is_ansistring(currpara1.vardef) and
                 is_ansistring(currpara2.vardef) and
                 (tstringdef(currpara1.vardef).encoding<>tstringdef(currpara2.vardef).encoding) and
                 ((tstringdef(currpara1.vardef).encoding=globals.CP_NONE) or
                  (tstringdef(currpara2.vardef).encoding=globals.CP_NONE)
                 ) then
                eq:=te_convert_l1;
              if eq<lowesteq then
                lowesteq:=eq;
              inc(i1);
              inc(i2);
              if cpo_ignorehidden in cpoptions then
                begin
                  while (i1<para1.count) and
                        (vo_is_hidden_para in tparavarsym(para1[i1]).varoptions) do
                    inc(i1);
                  while (i2<para2.count) and
                        (vo_is_hidden_para in tparavarsym(para2[i2]).varoptions) do
                    inc(i2);
                end;
              if cpo_ignoreframepointer in cpoptions then
                begin
                  if (i1<para1.count) and
                     (vo_is_parentfp in tparavarsym(para1[i1]).varoptions) then
                    inc(i1);
                  if (i2<para2.count) and
                     (vo_is_parentfp in tparavarsym(para2[i2]).varoptions) then
                    inc(i2);
                end;
           end;
         { when both lists are empty then the parameters are equal. Also
           when one list is empty and the other has a parameter with default
           value assigned then the parameters are also equal }
         if ((i1>=para1.count) and (i2>=para2.count)) or
            ((cpo_allowdefaults in cpoptions) and
             (((i1<para1.count) and assigned(tparavarsym(para1[i1]).defaultconstsym)) or
              ((i2<para2.count) and assigned(tparavarsym(para2[i2]).defaultconstsym)))) then
           compare_paras:=lowesteq;
      end;


    function proc_to_procvar_equal(def1:tabstractprocdef;def2:tprocvardef;checkincompatibleuniv: boolean):tequaltype;
      var
        eq: tequaltype;
        po_comp: tprocoptions;
        pa_comp: tcompare_paras_options;
      begin
         proc_to_procvar_equal:=te_incompatible;
         if not(assigned(def1)) or not(assigned(def2)) then
           exit;
         { check for method pointer and local procedure pointer:
             a) anything but procvars can be assigned to blocks
             b) if one is a procedure of object, the other also has to be one
                ("object static procedure" is equal to procedure as well)
                (except for block)
             c) if one is a pure address, the other also has to be one
                except if def1 is a global proc and def2 is a nested procdef
                (global procedures can be converted into nested procvars)
             d) if def1 is a nested procedure, then def2 has to be a nested
                procvar and def1 has to have the po_delphi_nested_cc option
                or does not use parentfp
             e) if def1 is a procvar, def1 and def2 both have to be nested or
                non-nested (we don't allow assignments from non-nested to
                nested procvars to make sure that we can still implement
                nested procvars using trampolines -- e.g., this would be
                necessary for LLVM or CIL as long as they do not have support
                for Delphi-style frame pointer parameter passing) }
         if is_block(def2) then                                     { a) }
           { can't explicitly check against procvars here, because
             def1 may already be a procvar due to a proc_to_procvar;
             this is checked in the type conversion node itself -> ok }
         else if
            ((def1.is_methodpointer and not (po_staticmethod in def1.procoptions))<> { b) }
             (def2.is_methodpointer and not (po_staticmethod in def2.procoptions))) or
            ((def1.is_addressonly<>def2.is_addressonly) and         { c) }
             (is_nested_pd(def1) or
              not is_nested_pd(def2))) or
            ((def1.typ=procdef) and                                 { d) }
             is_nested_pd(def1) and
             (not(po_delphi_nested_cc in def1.procoptions) or
              not is_nested_pd(def2))) or
            ((def1.typ=procvardef) and                              { e) }
             (is_nested_pd(def1)<>is_nested_pd(def2))) then
           exit;
         pa_comp:=[cpo_ignoreframepointer];
         if is_block(def2) then
           include(pa_comp,cpo_ignorehidden);
         if checkincompatibleuniv then
           include(pa_comp,cpo_warn_incompatible_univ);
         { check return value and options, methodpointer is already checked }
         po_comp:=[po_interrupt,po_iocheck,po_varargs,po_far];
         { check static only if we compare method pointers }
         if def1.is_methodpointer and def2.is_methodpointer then
           include(po_comp,po_staticmethod);
         if (m_delphi in current_settings.modeswitches) then
           exclude(po_comp,po_varargs);
         { for blocks, the calling convention doesn't matter because we have to
           generate a wrapper anyway }
         if ((po_is_block in def2.procoptions) or
             (def1.proccalloption=def2.proccalloption)) and
            ((po_comp * def1.procoptions)= (po_comp * def2.procoptions)) and
            equal_defs(def1.returndef,def2.returndef) then
          begin
            { return equal type based on the parameters, but a proc->procvar
              is never exact, so map an exact match of the parameters to
              te_equal }
            eq:=compare_paras(def1.paras,def2.paras,cp_procvar,pa_comp);
            if eq=te_exact then
             eq:=te_equal;
            if (eq=te_equal) then
              begin
                { prefer non-nested to non-nested over non-nested to nested }
                if (is_nested_pd(def1)<>is_nested_pd(def2)) then
                  eq:=te_convert_l1;
                { in case of non-block to block, we need a type conversion }
                if (po_is_block in def1.procoptions) <> (po_is_block in def2.procoptions) then
                  eq:=te_convert_l1;
              end;
            proc_to_procvar_equal:=eq;
          end;
      end;


    function compatible_childmethod_resultdef(parentretdef, childretdef: tdef): boolean;
      begin
        compatible_childmethod_resultdef :=
          (equal_defs(parentretdef,childretdef)) or
          ((parentretdef.typ=objectdef) and
           (childretdef.typ=objectdef) and
           is_class_or_interface_or_objc_or_java(parentretdef) and
           is_class_or_interface_or_objc_or_java(childretdef) and
           (def_is_related(tobjectdef(childretdef),tobjectdef(parentretdef))))
      end;


    function find_implemented_interface(impldef,intfdef:tobjectdef):timplementedinterface;
      var
        implintf : timplementedinterface;
        i : longint;
      begin
        if not assigned(impldef) then
          internalerror(2013102301);
        if not assigned(intfdef) then
          internalerror(2013102302);
        result:=nil;
        if not assigned(impldef.implementedinterfaces) then
          exit;
        for i:=0 to impldef.implementedinterfaces.count-1 do
          begin
            implintf:=timplementedinterface(impldef.implementedinterfaces[i]);
            if equal_defs(implintf.intfdef,intfdef) then
              begin
                result:=implintf;
                exit;
              end;
          end;
      end;


    function stringdef_is_related(curdef:tstringdef;otherdef:tdef):boolean;
      begin
        result:=
          (target_info.system in systems_jvm) and
          (((curdef.stringtype in [st_unicodestring,st_widestring]) and
            ((otherdef=java_jlobject) or
             (otherdef=java_jlstring))) or
           ((curdef.stringtype=st_ansistring) and
            ((otherdef=java_jlobject) or
             (otherdef=java_ansistring))));
      end;


    function recorddef_is_related(curdef:trecorddef;otherdef:tdef):boolean;
      begin
        { records are implemented via classes in the JVM target, and are
          all descendents of the java_fpcbaserecordtype class }
        result:=false;
        if (target_info.system in systems_jvm) then
          begin
            if otherdef.typ=objectdef then
              begin
                otherdef:=find_real_class_definition(tobjectdef(otherdef),false);
                if (otherdef=java_jlobject) or
                   (otherdef=java_fpcbaserecordtype) then
                  result:=true
              end;
          end;
      end;


    { true if prot implements d (or if they are equal) }
    function is_related_interface_multiple(prot:tobjectdef;d:tdef):boolean;
      var
        i : longint;
      begin
        { objcprotocols have multiple inheritance, all protocols from which
          the current protocol inherits are stored in implementedinterfaces }
        result:=prot=d;
        if result then
          exit;

        for i:=0 to prot.implementedinterfaces.count-1 do
          begin
            result:=is_related_interface_multiple(timplementedinterface(prot.implementedinterfaces[i]).intfdef,d);
            if result then
              exit;
          end;
      end;


    function objectdef_is_related(curdef:tobjectdef;otherdef:tdef):boolean;
      var
         realself,
         hp : tobjectdef;
      begin
        if (otherdef.typ=objectdef) then
          otherdef:=find_real_class_definition(tobjectdef(otherdef),false);
        realself:=find_real_class_definition(curdef,false);
        if realself=otherdef then
          begin
            result:=true;
            exit;
          end;

        if (realself.objecttype in [odt_objcclass,odt_objcprotocol]) and
           (otherdef=objc_idtype) then
          begin
            result:=true;
            exit;
          end;

        if (otherdef.typ<>objectdef) then
          begin
            result:=false;
            exit;
          end;

        { Objective-C protocols and Java interfaces can use multiple
           inheritance }
        if (realself.objecttype in [odt_objcprotocol,odt_interfacejava]) then
          begin
            result:=is_related_interface_multiple(realself,otherdef);
            exit;
          end;

        { formally declared Objective-C and Java classes match Objective-C/Java
          classes with the same name. In case of Java, the package must also
          match (still required even though we looked up the real definitions
          above, because these may be two different formal declarations that
          cannot be resolved yet) }
        if (realself.objecttype in [odt_objcclass,odt_javaclass]) and
           (tobjectdef(otherdef).objecttype=curdef.objecttype) and
           ((oo_is_formal in curdef.objectoptions) or
            (oo_is_formal in tobjectdef(otherdef).objectoptions)) and
           (curdef.objrealname^=tobjectdef(otherdef).objrealname^) then
          begin
            { check package name for Java }
            if curdef.objecttype=odt_objcclass then
              result:=true
            else
              begin
                result:=
                  assigned(curdef.import_lib)=assigned(tobjectdef(otherdef).import_lib);
                if result and
                   assigned(curdef.import_lib) then
                  result:=curdef.import_lib^=tobjectdef(otherdef).import_lib^;
              end;
            exit;
          end;

        hp:=realself.childof;
        while assigned(hp) do
          begin
             if equal_defs(hp,otherdef) then
               begin
                  result:=true;
                  exit;
               end;
             hp:=hp.childof;
          end;
        result:=false;
      end;


    function def_is_related(curdef,otherdef:tdef):boolean;
      begin
        if not assigned(curdef) then
          internalerror(2013102303);
        case curdef.typ of
          stringdef:
            result:=stringdef_is_related(tstringdef(curdef),otherdef);
          recorddef:
            result:=recorddef_is_related(trecorddef(curdef),otherdef);
          objectdef:
            result:=objectdef_is_related(tobjectdef(curdef),otherdef);
          else
            result:=false;
        end;
      end;


    function equal_genfunc_paradefs(fwdef,currdef:tdef;fwpdst,currpdst:tsymtable): boolean;
      begin
        result:=false;
        { for open array parameters, typesym might not be assigned }
        if assigned(fwdef.typesym) and (sp_generic_para in fwdef.typesym.symoptions) and
           assigned(currdef.typesym) and (sp_generic_para in currdef.typesym.symoptions) and
            (fwdef.owner=fwpdst) and
            (currdef.owner=currpdst) then
          begin
            { the forward declaration may have constraints }
            if not (df_genconstraint in currdef.defoptions) and (currdef.typ=undefineddef) and
                ((fwdef.typ=undefineddef) or (df_genconstraint in fwdef.defoptions)) then
              result:=true;
          end
      end;

end.
