{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Reads typed constants

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
unit ptconst;

{$i fpcdefs.inc}

interface

   uses symtype,symsym,aasmdata;

    procedure read_typed_const(list:tasmlist;sym:tstaticvarsym;in_structure:boolean);


implementation

    uses
<<<<<<< HEAD
       globtype,systems,globals,verbose,cutils,tokens,
       aasmbase,aasmtai,
       procinfo,fmodule,
       scanner,pbase,pdecvar,
       node,nbas,ngtcon,ngenutil,
       symconst,symbase,symdef
       ;

    procedure read_typed_const(list:tasmlist;sym:tstaticvarsym;in_structure:boolean);
      var
        storefilepos : tfileposinfo;
        section      : ansistring;
        tcbuilder    : ttypedconstbuilder;
        reslist,
        datalist     : tasmlist;
        restree,
        previnit     : tnode;
=======
       SysUtils,
       globtype,systems,tokens,verbose,constexp,
       cutils,globals,widestr,scanner,
       symconst,symbase,symdef,symtable,
       aasmbase,aasmtai,aasmcpu,defutil,defcmp,
       { pass 1 }
       node,htypechk,procinfo,
       nmat,nadd,ncal,nmem,nset,ncnv,ninl,ncon,nld,nflw,
       { parser specific stuff }
       pbase,pexpr,pdecvar,
       { codegen }
       cpuinfo,cgbase,dbgbase,
       wpobase,asmutils
       ;

{$maxfpuregisters 0}

{*****************************************************************************
                          Bitpacked value helpers
*****************************************************************************}

    type
      tbitpackedval = record
        curval, nextval: aword;
        curbitoffset: smallint;
        loadbitsize,packedbitsize: byte;
      end;


    procedure initbitpackval(out bp: tbitpackedval; packedbitsize: byte);
      begin
        bp.curval:=0;
        bp.nextval:=0;
        bp.curbitoffset:=0;
        bp.packedbitsize:=packedbitsize;
        bp.loadbitsize:=packedbitsloadsize(bp.packedbitsize)*8;
      end;


{$ifopt r+}
{$define rangeon}
{$r-}
{$endif}

{$ifopt q+}
{$define overflowon}
{$q-}
{$endif}
    { (values between quotes below refer to fields of bp; fields not         }
    {  mentioned are unused by this routine)                                 }
    { bitpacks "value" as bitpacked value of bitsize "packedbitsize" into    }
    { "curval", which has already been filled up to "curbitoffset", and      }
    { stores the spillover if any into "nextval". It also updates            }
    { curbitoffset to reflect how many bits of currval are now used (can be  }
    { > AIntBits in case of spillover)                                       }
    procedure bitpackval(value: aword; var bp: tbitpackedval);
      var
        shiftcount: longint;
      begin
        if (target_info.endian=endian_big) then
          begin
            { bitpacked format: left-aligned (i.e., "big endian bitness") }
            bp.curval:=bp.curval or ((value shl (AIntBits-bp.packedbitsize)) shr bp.curbitoffset);
            shiftcount:=((AIntBits-bp.packedbitsize)-bp.curbitoffset);
            { carry-over to the next element? }
            if (shiftcount<0) then
              bp.nextval:=(value and ((aword(1) shl (-shiftcount))-1)) shl
                          (AIntBits+shiftcount)
          end
        else
          begin
            { bitpacked format: right aligned (i.e., "little endian bitness") }
            bp.curval:=bp.curval or (value shl bp.curbitoffset);
            { carry-over to the next element? }
            if (bp.curbitoffset+bp.packedbitsize>AIntBits) then
              bp.nextval:=value shr (AIntBits-bp.curbitoffset)
          end;
        inc(bp.curbitoffset,bp.packedbitsize);
      end;

{$ifdef rangeon}
{$r+}
{$undef rangeon}
{$endif}

{$ifdef overflowon}
{$q+}
{$undef overflowon}
{$endif}


    procedure flush_packed_value(list: tasmlist; var bp: tbitpackedval);
      var
        bitstowrite: longint;
        writeval : byte;
      begin
        if (bp.curbitoffset < AIntBits) then
          begin
            { forced flush -> write multiple of loadsize }
            bitstowrite:=align(bp.curbitoffset,bp.loadbitsize);
            bp.curbitoffset:=0;
          end
        else
          begin
            bitstowrite:=AIntBits;
            dec(bp.curbitoffset,AIntBits);
          end;
        while (bitstowrite>=8) do
          begin
            if (target_info.endian=endian_little) then
              begin
                { write lowest byte }
                writeval:=byte(bp.curval);
                bp.curval:=bp.curval shr 8;
              end
            else
              begin
                { write highest byte }
                writeval:=bp.curval shr (AIntBits-8);
                bp.curval:=(bp.curval and (not($ff shl (AIntBits-8)))) shl 8;
              end;
            list.concat(tai_const.create_8bit(writeval));
            dec(bitstowrite,8);
          end;
        bp.curval:=bp.nextval;
        bp.nextval:=0;
      end;


{*****************************************************************************
                             read typed const
*****************************************************************************}

    type
      { context used for parsing complex types (arrays/records/objects) }
      threc = record
        list   : tasmlist;
        origsym: tstaticvarsym;
        offset:  asizeint;
      end;

    { this procedure reads typed constants }
    procedure read_typed_const_data(var hr:threc;def:tdef); forward;

      procedure parse_orddef(list:tasmlist;def:torddef);
        var
          n : tnode;
          intvalue : tconstexprint;

        procedure do_error;
          begin
            if is_constnode(n) then
              IncompatibleTypes(n.resultdef, def)
            else
              Message(parser_e_illegal_expression);
          end;

        begin
           n:=comp_expr(true,false);
           { for C-style booleans, true=-1 and false=0) }
           if is_cbool(def) then
             inserttypeconv(n,def);
           case def.ordtype of
              pasbool,
              bool8bit :
                begin
                   if is_constboolnode(n) then
                     list.concat(Tai_const.Create_8bit(byte(tordconstnode(n).value.svalue)))
                   else
                     do_error;
                end;
              bool16bit :
                begin
                   if is_constboolnode(n) then
                     list.concat(Tai_const.Create_16bit(word(tordconstnode(n).value.svalue)))
                   else
                     do_error;
                end;
              bool32bit :
                begin
                   if is_constboolnode(n) then
                     list.concat(Tai_const.Create_32bit(longint(tordconstnode(n).value.svalue)))
                   else
                     do_error;
                end;
              bool64bit :
                begin
                   if is_constboolnode(n) then
                     list.concat(Tai_const.Create_64bit(int64(tordconstnode(n).value.svalue)))
                   else
                     do_error;
                end;
              uchar :
                begin
                   if is_constwidecharnode(n) then 
                     inserttypeconv(n,cchartype); 
                   if is_constcharnode(n) or
                     ((m_delphi in current_settings.modeswitches) and
                      is_constwidecharnode(n) and
                      (tordconstnode(n).value <= 255)) then
                     list.concat(Tai_const.Create_8bit(byte(tordconstnode(n).value.svalue)))
                   else
                     do_error;
                end;
              uwidechar :
                begin
                   if is_constcharnode(n) then
                     inserttypeconv(n,cwidechartype);
                   if is_constwidecharnode(n) then
                     list.concat(Tai_const.Create_16bit(word(tordconstnode(n).value.svalue)))
                   else
                     do_error;
                end;
              s8bit,u8bit,
              u16bit,s16bit,
              s32bit,u32bit,
              s64bit,u64bit :
                begin
                   if is_constintnode(n) then
                     begin
<<<<<<< HEAD
<<<<<<< HEAD
                       testrange(def,tordconstnode(n).value,false,false);
=======
                       testrange(n.resultdef,def,tordconstnode(n).value,false);
>>>>>>> graemeg/fixes_2_2
=======
                       testrange(n.resultdef,def,tordconstnode(n).value,false);
>>>>>>> origin/fixes_2_2
                       case def.size of
                         1 :
                           list.concat(Tai_const.Create_8bit(byte(tordconstnode(n).value.svalue)));
                         2 :
                           list.concat(Tai_const.Create_16bit(word(tordconstnode(n).value.svalue)));
                         4 :
                           list.concat(Tai_const.Create_32bit(longint(tordconstnode(n).value.svalue)));
                         8 :
                           list.concat(Tai_const.Create_64bit(tordconstnode(n).value.svalue));
                       end;
                     end
                   else
                     do_error;
                end;
              scurrency:
                begin
                   if is_constintnode(n) then
                     intvalue := tordconstnode(n).value
                   { allow bootstrapping }
                   else if is_constrealnode(n) then
                     intvalue:=PInt64(@trealconstnode(n).value_currency)^
                   else
                     begin
                       intvalue:=0;
                       IncompatibleTypes(n.resultdef, def);
                     end;
                  list.concat(Tai_const.Create_64bit(intvalue));
                end;
              else
                internalerror(200611052);
           end;
           n.free;
        end;

        procedure parse_floatdef(list:tasmlist;def:tfloatdef);
        var
          n : tnode;
          value : bestreal;
        begin
          n:=comp_expr(true,false);
          if is_constrealnode(n) then
            value:=trealconstnode(n).value_real
          else if is_constintnode(n) then
            value:=tordconstnode(n).value
          else if is_constnode(n) then
            IncompatibleTypes(n.resultdef, def)
          else
            Message(parser_e_illegal_expression);

          case def.floattype of
             s32real :
               list.concat(Tai_real_32bit.Create(ts32real(value)));
             s64real :
{$ifdef ARM}
               if is_double_hilo_swapped then
                 list.concat(Tai_real_64bit.Create_hiloswapped(ts64real(value)))
               else
{$endif ARM}
                 list.concat(Tai_real_64bit.Create(ts64real(value)));
             s80real :
               list.concat(Tai_real_80bit.Create(value,s80floattype.size));
             sc80real :
               list.concat(Tai_real_80bit.Create(value,sc80floattype.size));
             s64comp :
               { the round is necessary for native compilers where comp isn't a float }
               list.concat(Tai_comp_64bit.Create(round(value)));
             s64currency:
               list.concat(Tai_comp_64bit.Create(round(value*10000)));
             s128real:
               list.concat(Tai_real_128bit.Create(value));
             else
               internalerror(200611053);
          end;
          n.free;
        end;

        procedure parse_classrefdef(list:tasmlist;def:tclassrefdef);
        var
          n : tnode;
        begin
          n:=comp_expr(true,false);
          case n.nodetype of
            loadvmtaddrn:
              begin
                if not Tobjectdef(tclassrefdef(n.resultdef).pointeddef).is_related(tobjectdef(def.pointeddef)) then
                  IncompatibleTypes(n.resultdef, def);
                list.concat(Tai_const.Create_sym(current_asmdata.RefAsmSymbol(Tobjectdef(tclassrefdef(n.resultdef).pointeddef).vmt_mangledname)));
                { update wpo info }
                if not assigned(current_procinfo) or
                   (po_inline in current_procinfo.procdef.procoptions) or
                   wpoinfomanager.symbol_live(current_procinfo.procdef.mangledname) then
                  tobjectdef(tclassrefdef(n.resultdef).pointeddef).register_maybe_created_object_type;
              end;
             niln:
               list.concat(Tai_const.Create_sym(nil));
             else if is_constnode(n) then
               IncompatibleTypes(n.resultdef, def)
             else
               Message(parser_e_illegal_expression);
          end;
          n.free;
        end;

        procedure parse_pointerdef(list:tasmlist;def:tpointerdef);
        var
          hp,p      : tnode;
          srsym     : tsym;
          pd        : tprocdef;
          ca        : pchar;
          pw        : pcompilerwidestring;
          i,len     : longint;
          base,
          offset    : aint;
          v         : Tconstexprint;
          ll        : tasmlabel;
          varalign  : shortint;
        begin
          p:=comp_expr(true,false);
          { remove equal typecasts for pointer/nil addresses }
          if (p.nodetype=typeconvn) then
            with Ttypeconvnode(p) do
              if (left.nodetype in [addrn,niln]) and equal_defs(def,p.resultdef) then
                begin
                  hp:=left;
                  left:=nil;
                  p.free;
                  p:=hp;
                end;
          { allows horrible ofs(typeof(TButton)^) code !! }
          if (p.nodetype=addrn) then
            with Taddrnode(p) do
              if left.nodetype=derefn then
                begin
                  hp:=tderefnode(left).left;
                  tderefnode(left).left:=nil;
                  p.free;
                  p:=hp;
               end;
          { const pointer ? }
          if (p.nodetype = pointerconstn) then
            begin
              {$if sizeof(TConstPtrUInt)=8}
                list.concat(Tai_const.Create_64bit(int64(tpointerconstnode(p).value)));
              {$else}
                {$if sizeof(TConstPtrUInt)=4}
                  list.concat(Tai_const.Create_32bit(longint(tpointerconstnode(p).value)));
                {$else}
                  internalerror(200404122);
              {$endif} {$endif}
            end
          { nil pointer ? }
          else if p.nodetype=niln then
            list.concat(Tai_const.Create_sym(nil))
          { maybe pchar ? }
          else
            if is_char(def.pointeddef) and
               (p.nodetype<>addrn) then
              begin
                current_asmdata.getdatalabel(ll);
                list.concat(Tai_const.Create_sym(ll));
                if p.nodetype=stringconstn then
                 varalign:=size_2_align(tstringconstnode(p).len)
                else
                 varalign:=0;
                varalign:=const_align(varalign);
                new_section(current_asmdata.asmlists[al_const], sec_rodata, ll.name, varalign);
                current_asmdata.asmlists[al_const].concat(Tai_label.Create(ll));
                if p.nodetype=stringconstn then
                  begin
                    len:=tstringconstnode(p).len;
                    { For tp7 the maximum lentgh can be 255 }
                    if (m_tp7 in current_settings.modeswitches) and
                       (len>255) then
                     len:=255;
                    getmem(ca,len+2);
                    move(tstringconstnode(p).value_str^,ca^,len+1);
                    current_asmdata.asmlists[al_const].concat(Tai_string.Create_pchar(ca,len+1));
                  end
                else
                  if is_constcharnode(p) then
                    current_asmdata.asmlists[al_const].concat(Tai_string.Create(char(byte(tordconstnode(p).value.svalue))+#0))
                else
                  IncompatibleTypes(p.resultdef, def);
            end
          { maybe pwidechar ? }
          else
            if is_widechar(def.pointeddef) and
               (p.nodetype<>addrn) then
              begin
                current_asmdata.getdatalabel(ll);
                list.concat(Tai_const.Create_sym(ll));
                current_asmdata.asmlists[al_typedconsts].concat(tai_align.create(const_align(sizeof(pint))));
                current_asmdata.asmlists[al_typedconsts].concat(Tai_label.Create(ll));
                if (p.nodetype in [stringconstn,ordconstn]) then
                  begin
                    { convert to widestring stringconstn }
                    inserttypeconv(p,cwidestringtype);
                    if (p.nodetype=stringconstn) and
                       (tstringconstnode(p).cst_type in [cst_widestring,cst_unicodestring]) then
                     begin
                       pw:=pcompilerwidestring(tstringconstnode(p).value_str);
                       for i:=0 to tstringconstnode(p).len-1 do
                         current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_16bit(pw^.data[i]));
                       { ending #0 }
                       current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_16bit(0))
                     end;
                  end
                else
                  IncompatibleTypes(p.resultdef, def);
            end
          else
            if (p.nodetype=addrn) or
               is_proc2procvar_load(p,pd) then
              begin
                { insert typeconv }
                inserttypeconv(p,def);
                hp:=p;
                while assigned(hp) and (hp.nodetype in [addrn,typeconvn,subscriptn,vecn]) do
                  hp:=tunarynode(hp).left;
                if (hp.nodetype=loadn) then
                  begin
                    hp:=p;
                    offset:=0;
                    while assigned(hp) and (hp.nodetype<>loadn) do
                      begin
                         case hp.nodetype of
                           vecn :
                             begin
                               case tvecnode(hp).left.resultdef.typ of
                                 stringdef :
                                   begin
                                      { this seems OK for shortstring and ansistrings PM }
                                      { it is wrong for widestrings !! }
                                      len:=1;
                                      base:=0;
                                   end;
                                 arraydef :
                                   begin
                                      if not is_packed_array(tvecnode(hp).left.resultdef) then
                                        begin
                                          len:=tarraydef(tvecnode(hp).left.resultdef).elesize;
                                          base:=tarraydef(tvecnode(hp).left.resultdef).lowrange;
                                        end
                                      else
                                        begin
                                          Message(parser_e_packed_dynamic_open_array);
                                          len:=1;
                                          base:=0;
                                        end;
                                   end
                                 else
                                   Message(parser_e_illegal_expression);
                               end;
                               if is_constintnode(tvecnode(hp).right) then
                                 begin
                                   {Prevent overflow.}
                                   v:=get_ordinal_value(tvecnode(hp).right)-base;
                                   if (v<int64(low(offset))) or (v>int64(high(offset))) then
                                     message(parser_e_range_check_error);
                                   if high(offset)-offset div len>v then
                                     inc(offset,len*v.svalue)
                                   else
                                     message(parser_e_range_check_error);
                                 end
                               else
                                 Message(parser_e_illegal_expression);
                             end;
                           subscriptn :
                             inc(offset,tsubscriptnode(hp).vs.fieldoffset);
                           typeconvn :
                             begin
                               if not(ttypeconvnode(hp).convtype in [tc_equal,tc_proc_2_procvar]) then
                                 Message(parser_e_illegal_expression);
                             end;
                           addrn :
                             ;
                           else
                             Message(parser_e_illegal_expression);
                         end;
                         hp:=tunarynode(hp).left;
                      end;
                    srsym:=tloadnode(hp).symtableentry;
                    case srsym.typ of
                      procsym :
                        begin
                          pd:=tprocdef(tprocsym(srsym).ProcdefList[0]);
                          if Tprocsym(srsym).ProcdefList.Count>1 then
                            Message(parser_e_no_overloaded_procvars);
                          if po_abstractmethod in pd.procoptions then
                            Message(type_e_cant_take_address_of_abstract_method)
                          else
                            list.concat(Tai_const.Createname(pd.mangledname,offset));
                        end;
                      staticvarsym :
                        list.concat(Tai_const.Createname(tstaticvarsym(srsym).mangledname,offset));
                      labelsym :
                        list.concat(Tai_const.Createname(tlabelsym(srsym).mangledname,offset));
                      constsym :
                        if tconstsym(srsym).consttyp=constresourcestring then
<<<<<<< HEAD
<<<<<<< HEAD
                          list.concat(Tai_const.Createname(make_mangledname('RESSTR',tconstsym(srsym).owner,tconstsym(srsym).name),sizeof(pint)))
=======
                          list.concat(Tai_const.Createname(make_mangledname('RESSTR',tconstsym(srsym).owner,tconstsym(srsym).name),sizeof(aint)))
>>>>>>> graemeg/fixes_2_2
=======
                          list.concat(Tai_const.Createname(make_mangledname('RESSTR',tconstsym(srsym).owner,tconstsym(srsym).name),sizeof(aint)))
>>>>>>> origin/fixes_2_2
                        else
                          Message(type_e_variable_id_expected);
                      else
                        Message(type_e_variable_id_expected);
                    end;
                  end
                else
                  Message(parser_e_illegal_expression);
              end
          else
          { allow typeof(Object type)}
            if (p.nodetype=inlinen) and
               (tinlinenode(p).inlinenumber=in_typeof_x) then
              begin
                if (tinlinenode(p).left.nodetype=typen) then
                  begin
                    list.concat(Tai_const.createname(
                      tobjectdef(tinlinenode(p).left.resultdef).vmt_mangledname,0));
                  end
                else
                  Message(parser_e_illegal_expression);
              end
          else
            Message(parser_e_illegal_expression);
          p.free;
        end;

        procedure parse_setdef(list:tasmlist;def:tsetdef);
        type
           setbytes = array[0..31] of byte;
           Psetbytes = ^setbytes;
        var
          p : tnode;
          i : longint;
        begin
          p:=comp_expr(true,false);
          if p.nodetype=setconstn then
            begin
              { be sure to convert to the correct result, else
                it can generate smallset data instead of normalset (PFV) }
              inserttypeconv(p,def);
              { we only allow const sets }
              if (p.nodetype<>setconstn) or
                 assigned(tsetconstnode(p).left) then
                Message(parser_e_illegal_expression)
              else
                begin
                  tsetconstnode(p).adjustforsetbase;
<<<<<<< HEAD
<<<<<<< HEAD
                  { this writing is endian-dependant   }
=======
=======
>>>>>>> origin/fixes_2_2
                  { this writing is endian independant   }
                  { untrue - because they are considered }
                  { arrays of 32-bit values CEC          }
>>>>>>> graemeg/fixes_2_2
                  if source_info.endian = target_info.endian then
                    begin
{$if defined(FPC_NEW_BIGENDIAN_SETS) or defined(FPC_LITTLE_ENDIAN)}
                      for i:=0 to p.resultdef.size-1 do
                        list.concat(tai_const.create_8bit(Psetbytes(tsetconstnode(p).value_set)^[i]));
{$else}
                      for i:=0 to p.resultdef.size-1 do
                        list.concat(tai_const.create_8bit(reverse_byte(Psetbytes(tsetconstnode(p).value_set)^[i xor 3])));
{$endif}
                    end
                  else
                    begin
                      for i:=0 to p.resultdef.size-1 do
                        list.concat(tai_const.create_8bit(reverse_byte(Psetbytes(tsetconstnode(p).value_set)^[i])));
                    end;
                end;
            end
          else
            Message(parser_e_illegal_expression);
          p.free;
        end;

        procedure parse_enumdef(list:tasmlist;def:tenumdef);
        var
          p : tnode;
        begin
          p:=comp_expr(true,false);
          if p.nodetype=ordconstn then
            begin
              if equal_defs(p.resultdef,def) or
                 is_subequal(p.resultdef,def) then
                begin
                  case longint(p.resultdef.size) of
                    1 : list.concat(Tai_const.Create_8bit(Byte(tordconstnode(p).value.svalue)));
                    2 : list.concat(Tai_const.Create_16bit(Word(tordconstnode(p).value.svalue)));
                    4 : list.concat(Tai_const.Create_32bit(Longint(tordconstnode(p).value.svalue)));
                  end;
                end
              else
                IncompatibleTypes(p.resultdef,def);
            end
          else
            Message(parser_e_illegal_expression);
          p.free;
        end;


        procedure parse_stringdef(const hr:threc;def:tstringdef);
        var
          n : tnode;
          strlength : aint;
          strval    : pchar;
          strch     : char;
          ll,ll2    : tasmlabel;
          ca        : pchar;
          winlike   : boolean;
        begin
          n:=comp_expr(true,false);
          { load strval and strlength of the constant tree }
          if (n.nodetype=stringconstn) or is_wide_or_unicode_string(def) or is_constwidecharnode(n) or
            ((n.nodetype=typen) and is_interfacecorba(ttypenode(n).typedef)) then
            begin
              { convert to the expected string type so that
                for widestrings strval is a pcompilerwidestring }
              inserttypeconv(n,def);
              if (not codegenerror) and
                 (n.nodetype=stringconstn) then
                begin
                  strlength:=tstringconstnode(n).len;
                  strval:=tstringconstnode(n).value_str;
                end
              else
                begin
                  { an error occurred trying to convert the result to a string }
                  strlength:=-1;
                  { it's possible that the type conversion could not be
                    evaluated at compile-time }
                  if not codegenerror then
                    CGMessage(parser_e_widestring_to_ansi_compile_time);
                end;
            end
          else if is_constcharnode(n) then
            begin
              { strval:=pchar(@tordconstnode(n).value);
                THIS FAIL on BIG_ENDIAN MACHINES PM }
              strch:=chr(tordconstnode(n).value.svalue and $ff);
              strval:=@strch;
              strlength:=1
            end
          else if is_constresourcestringnode(n) then
            begin
              strval:=pchar(tconstsym(tloadnode(n).symtableentry).value.valueptr);
              strlength:=tconstsym(tloadnode(n).symtableentry).value.len;
            end
          else
            begin
              Message(parser_e_illegal_expression);
              strlength:=-1;
            end;
          if strlength>=0 then
            begin
              case def.stringtype of
                st_shortstring:
                  begin
                    if strlength>=def.size then
                     begin
                       message2(parser_w_string_too_long,strpas(strval),tostr(def.size-1));
                       strlength:=def.size-1;
                     end;
                    hr.list.concat(Tai_const.Create_8bit(strlength));
                    { this can also handle longer strings }
                    getmem(ca,strlength+1);
                    move(strval^,ca^,strlength);
                    ca[strlength]:=#0;
                    hr.list.concat(Tai_string.Create_pchar(ca,strlength));
                    { fillup with spaces if size is shorter }
                    if def.size>strlength then
                     begin
                       getmem(ca,def.size-strlength);
                       { def.size contains also the leading length, so we }
                       { we have to subtract one                       }
                       fillchar(ca[0],def.size-strlength-1,' ');
                       ca[def.size-strlength-1]:=#0;
                       { this can also handle longer strings }
                       hr.list.concat(Tai_string.Create_pchar(ca,def.size-strlength-1));
                     end;
                  end;
                st_ansistring:
                  begin
                     { an empty ansi string is nil! }
                     if (strlength=0) then
                       ll := nil
                     else
<<<<<<< HEAD
                       ll := emit_ansistring_const(current_asmdata.asmlists[al_const],strval,strlength,def.encoding);
                     hr.list.concat(Tai_const.Create_sym(ll));
=======
                       begin
                         current_asmdata.getdatalabel(ll);
                         list.concat(Tai_const.Create_sym(ll));
                         current_asmdata.getdatalabel(ll2);
                         current_asmdata.asmlists[al_const].concat(tai_align.create(const_align(sizeof(aint))));
                         current_asmdata.asmlists[al_const].concat(Tai_label.Create(ll2));
                         current_asmdata.asmlists[al_const].concat(Tai_const.Create_aint(-1));
                         current_asmdata.asmlists[al_const].concat(Tai_const.Create_aint(strlength));
                         { make sure the string doesn't get dead stripped if the header is referenced }
                         if (target_info.system in systems_darwin) then
                           current_asmdata.asmlists[al_typedconsts].concat(tai_directive.create(asd_reference,ll.name));
                         current_asmdata.asmlists[al_const].concat(Tai_label.Create(ll));
                         { ... and vice versa }
                         if (target_info.system in systems_darwin) then
                           list.concat(tai_directive.create(asd_reference,ll2.name));
                         getmem(ca,strlength+1);
                         move(strval^,ca^,strlength);
                         { The terminating #0 to be stored in the .data section (JM) }
                         ca[strlength]:=#0;
                         current_asmdata.asmlists[al_const].concat(Tai_string.Create_pchar(ca,strlength+1));
                       end;
>>>>>>> graemeg/fixes_2_2
                  end;
                st_unicodestring,
                st_widestring:
                  begin
                     { an empty wide/unicode string is nil! }
                     if (strlength=0) then
                       ll := nil
                     else
                     begin
                       winlike := (def.stringtype=st_widestring) and (tf_winlikewidestring in target_info.flags);
                       ll := emit_unicodestring_const(current_asmdata.asmlists[al_const],
                              strval,
                              def.encoding,
                              winlike);

                       { collect global Windows widestrings }
                       if winlike then
                       begin
<<<<<<< HEAD
                         current_asmdata.WideInits.Concat(
                            TTCInitItem.Create(hr.origsym, hr.offset, ll)
                         );
                         ll := nil;
=======
                         current_asmdata.getdatalabel(ll);
                         list.concat(Tai_const.Create_sym(ll));
                         current_asmdata.getdatalabel(ll2);
                         current_asmdata.asmlists[al_const].concat(tai_align.create(const_align(sizeof(aint))));
                         current_asmdata.asmlists[al_const].concat(Tai_label.Create(ll2));
                         if tf_winlikewidestring in target_info.flags then
                           current_asmdata.asmlists[al_const].concat(Tai_const.Create_32bit(strlength*cwidechartype.size))
                         else
                           begin
                             current_asmdata.asmlists[al_const].concat(Tai_const.Create_aint(-1));
                             current_asmdata.asmlists[al_const].concat(Tai_const.Create_aint(strlength*cwidechartype.size));
                           end;
                         { make sure the string doesn't get dead stripped if the header is referenced }
                         if (target_info.system in systems_darwin) then
                           current_asmdata.asmlists[al_typedconsts].concat(tai_directive.create(asd_reference,ll.name));
                         current_asmdata.asmlists[al_const].concat(Tai_label.Create(ll));
                         { ... and vice versa }
                         if (target_info.system in systems_darwin) then
                           current_asmdata.asmlists[al_typedconsts].concat(tai_directive.create(asd_reference,ll2.name));
                         for i:=0 to strlength-1 do
                           current_asmdata.asmlists[al_const].concat(Tai_const.Create_16bit(pcompilerwidestring(strval)^.data[i]));
                         { ending #0 }
                         current_asmdata.asmlists[al_const].concat(Tai_const.Create_16bit(0))
>>>>>>> graemeg/fixes_2_2
                       end;
                     end;
                     hr.list.concat(Tai_const.Create_sym(ll));
                  end;
                else
                  internalerror(200107081);
              end;
            end;
          n.free;
        end;


        { parse a single constant and add it to the packed const info  }
        { represented by curval etc (see explanation of bitpackval for }
        { what the different parameters mean)                          }
        function parse_single_packed_const(list: tasmlist; def: tdef; var bp: tbitpackedval): boolean;
          var
            n : tnode;
          begin
            result:=true;
            n:=comp_expr(true,false);
            if (n.nodetype <> ordconstn) or
               (not equal_defs(n.resultdef,def) and
                not is_subequal(n.resultdef,def)) then
              begin
                n.free;
                incompatibletypes(n.resultdef,def);
                consume_all_until(_SEMICOLON);
                result:=false;
                exit;
              end;
            if (Tordconstnode(n).value<qword(low(Aword))) or (Tordconstnode(n).value>qword(high(Aword))) then
              message(parser_e_range_check_error)
            else
              bitpackval(Tordconstnode(n).value.uvalue,bp);
            if (bp.curbitoffset>=AIntBits) then
              flush_packed_value(list,bp);
            n.free;
          end;


        { parses a packed array constant }
        procedure parse_packed_array_def(list: tasmlist; def: tarraydef);
          var
            i  : aint;
            bp : tbitpackedval;
          begin
            if not(def.elementdef.typ in [orddef,enumdef]) then
              internalerror(2007022010);
            { begin of the array }
            consume(_LKLAMMER);
            initbitpackval(bp,def.elepackedbitsize);
            i:=def.lowrange;
            { can't use for-loop, fails when cross-compiling from }
            { 32 to 64 bit because i is then 64 bit               }
            while (i<def.highrange) do
              begin
                { get next item of the packed array }
                if not parse_single_packed_const(list,def.elementdef,bp) then
                  exit;
                consume(_COMMA);
                inc(i);
              end;
            { final item }
            if not parse_single_packed_const(list,def.elementdef,bp) then
              exit;
            { flush final incomplete value if necessary }
            if (bp.curbitoffset <> 0) then
              flush_packed_value(list,bp);
            consume(_RKLAMMER);
          end;


        procedure parse_arraydef(hr:threc;def:tarraydef);
        var
          n : tnode;
          i : longint;
          len : asizeint;
          ch  : array[0..1] of char;
          ca  : pbyte;
          int_const: tai_const;
          char_size: integer;
        begin
          { dynamic array nil }
          if is_dynamic_array(def) then
            begin
              { Only allow nil initialization }
              consume(_NIL);
              hr.list.concat(Tai_const.Create_sym(nil));
            end
          { packed array constant }
          else if is_packed_array(def) and
                  ((def.elepackedbitsize mod 8 <> 0) or
                   not ispowerof2(def.elepackedbitsize div 8,i)) then
            begin
              parse_packed_array_def(hr.list,def);
            end
          { normal array const between brackets }
          else if try_to_consume(_LKLAMMER) then
            begin
              hr.offset:=0;
              for i:=def.lowrange to def.highrange-1 do
                begin
                  read_typed_const_data(hr,def.elementdef);
                  Inc(hr.offset,def.elementdef.size);
                  if token=_RKLAMMER then
                    begin
                      Message1(parser_e_more_array_elements_expected,tostr(def.highrange-i));
                      consume(_RKLAMMER);
                      exit;
                    end
                  else
                    consume(_COMMA);
                end;
              read_typed_const_data(hr,def.elementdef);
              consume(_RKLAMMER);
            end
          { if array of char then we allow also a string }
          else if is_anychar(def.elementdef) then
            begin
               char_size:=def.elementdef.size;
               n:=comp_expr(true,false);
               if n.nodetype=stringconstn then
                 begin
                   len:=tstringconstnode(n).len;
                    case char_size of
                      1:
                        begin
                          if (tstringconstnode(n).cst_type in [cst_unicodestring,cst_widestring]) then
                            inserttypeconv(n,cansistringtype);
                          if n.nodetype<>stringconstn then
                            internalerror(2010033003);
                          ca:=pointer(tstringconstnode(n).value_str);
                        end;
                      2:
                        begin
                          inserttypeconv(n,cwidestringtype);
                          if n.nodetype<>stringconstn then
                            internalerror(2010033003);
                          ca:=pointer(pcompilerwidestring(tstringconstnode(n).value_str)^.data)
                        end;
                      else
                        internalerror(2010033005);
                    end;
                   { For tp7 the maximum lentgh can be 255 }
                   if (m_tp7 in current_settings.modeswitches) and
                      (len>255) then
                    len:=255;
                 end
               else if is_constcharnode(n) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                  begin
                    case char_size of
                      1:
                        ch[0]:=chr(tordconstnode(n).value.uvalue and $ff);
                      2:
                        begin
                          inserttypeconv(n,cwidechartype);
                          if not is_constwidecharnode(n) then
                            internalerror(2010033001);
                          widechar(ch):=widechar(tordconstnode(n).value.uvalue and $ffff);
                        end;
                      else
                        internalerror(2010033002);
                    end;
                    ca:=@ch;
                    len:=1;
                  end
               else if is_constwidecharnode(n) and (current_settings.sourcecodepage<>'utf8') then
                  begin
                    case char_size of
                      1:
<<<<<<< HEAD
<<<<<<< HEAD
=======
                  begin
                    case char_size of
                      1:
                        ch[0]:=chr(tordconstnode(n).value.uvalue and $ff);
                      2:
                        begin
                          inserttypeconv(n,cwidechartype);
                          if not is_constwidecharnode(n) then
                            internalerror(2010033001);
                          widechar(ch):=widechar(tordconstnode(n).value.uvalue and $ffff);
                        end;
                      else
                        internalerror(2010033002);
                    end;
                    ca:=@ch;
                    len:=1;
                  end
               else if is_constwidecharnode(n) and (current_settings.sourcecodepage<>'utf8') then
                  begin
                    case char_size of
                      1:
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                        begin
                          inserttypeconv(n,cchartype);
                          if not is_constcharnode(n) then
                            internalerror(2010033001);
                          ch[0]:=chr(tordconstnode(n).value.uvalue and $ff);
                        end;
                      2:
                        widechar(ch):=widechar(tordconstnode(n).value.uvalue and $ffff);
                      else
                        internalerror(2010033002);
                    end;
                    ca:=@ch;
                    len:=1;
                  end
               else
                 begin
                   Message(parser_e_illegal_expression);
                   len:=0;
                 end;
               if len>(def.highrange-def.lowrange+1) then
                 Message(parser_e_string_larger_array);
               for i:=0 to def.highrange-def.lowrange do
                 begin
                   if i<len then
                     begin
                       case char_size of
                         1:
                          int_const:=Tai_const.Create_char(char_size,pbyte(ca)^);
                         2:
                          int_const:=Tai_const.Create_char(char_size,pword(ca)^);
                         else
                           internalerror(2010033004);
                       end;
                       inc(ca, char_size);
                     end
                   else
                     {Fill the remaining positions with #0.}
                     int_const:=Tai_const.Create_char(char_size,0);
                   hr.list.concat(int_const)
                 end;
               n.free;
            end
          else
            begin
              { we want the ( }
              consume(_LKLAMMER);
            end;
        end;

        procedure parse_procvardef(list:tasmlist;def:tprocvardef);
        var
          tmpn,n : tnode;
          pd   : tprocdef;
        begin
          { Procvars and pointers are no longer compatible.  }
          { under tp:  =nil or =var under fpc: =nil or =@var }
          if try_to_consume(_NIL) then
            begin
               list.concat(Tai_const.Create_sym(nil));
               if not def.is_addressonly then
                 list.concat(Tai_const.Create_sym(nil));
               exit;
            end;
          { you can't assign a value other than NIL to a typed constant  }
          { which is a "procedure of object", because this also requires }
          { address of an object/class instance, which is not known at   }
          { compile time (JM)                                            }
          if (po_methodpointer in def.procoptions) then
            Message(parser_e_no_procvarobj_const);
          { parse the rest too, so we can continue with error checking }
          getprocvardef:=def;
          n:=comp_expr(true,false);
          getprocvardef:=nil;
          if codegenerror then
            begin
              n.free;
              exit;
            end;
          { let type conversion check everything needed }
          inserttypeconv(n,def);
          if codegenerror then
            begin
              n.free;
              exit;
            end;
          { remove typeconvs, that will normally insert a lea
            instruction which is not necessary for us }
          while n.nodetype=typeconvn do
            begin
              tmpn:=ttypeconvnode(n).left;
              ttypeconvnode(n).left:=nil;
              n.free;
              n:=tmpn;
            end;
          { remove addrn which we also don't need here }
          if n.nodetype=addrn then
            begin
              tmpn:=taddrnode(n).left;
              taddrnode(n).left:=nil;
              n.free;
              n:=tmpn;
            end;
          { we now need to have a loadn with a procsym }
          if (n.nodetype=loadn) and
             (tloadnode(n).symtableentry.typ=procsym) then
            begin
              pd:=tloadnode(n).procdef;
              list.concat(Tai_const.createname(pd.mangledname,0));
              { nested procvar typed consts can only be initialised with nil
                (checked above) or with a global procedure (checked here),
                because in other cases we need a valid frame pointer }
              if is_nested_pd(def) then
                begin
                  if is_nested_pd(pd) then
                    Message(parser_e_no_procvarnested_const);
                  list.concat(Tai_const.Create_sym(nil));
                end
            end
          else
            Message(parser_e_illegal_expression);
          n.free;
        end;

      procedure parse_recorddef(hr:threc;def:trecorddef);
        var
          n       : tnode;
          symidx  : longint;
          recsym,
          srsym   : tsym;
          hs      : string;
          sorg,s  : TIDString;
          tmpguid : tguid;
          curroffset,
          fillbytes  : aint;
          bp   : tbitpackedval;
          error,
          is_packed: boolean;
          startoffset: aint;

        procedure handle_stringconstn;
          var
            i       : longint;
          begin
            hs:=strpas(tstringconstnode(n).value_str);
            if string2guid(hs,tmpguid) then
              begin
                hr.list.concat(Tai_const.Create_32bit(longint(tmpguid.D1)));
                hr.list.concat(Tai_const.Create_16bit(tmpguid.D2));
                hr.list.concat(Tai_const.Create_16bit(tmpguid.D3));
                for i:=Low(tmpguid.D4) to High(tmpguid.D4) do
                  hr.list.concat(Tai_const.Create_8bit(tmpguid.D4[i]));
              end
            else
              Message(parser_e_improper_guid_syntax);
          end;

        var
          i : longint;

        begin
          { GUID }
          if (def=rec_tguid) and (token=_ID) then
            begin
              n:=comp_expr(true,false);
              if n.nodetype=stringconstn then
                handle_stringconstn
              else
                begin
                  inserttypeconv(n,rec_tguid);
                  if n.nodetype=guidconstn then
                    begin
                      tmpguid:=tguidconstnode(n).value;
                      hr.list.concat(Tai_const.Create_32bit(longint(tmpguid.D1)));
                      hr.list.concat(Tai_const.Create_16bit(tmpguid.D2));
                      hr.list.concat(Tai_const.Create_16bit(tmpguid.D3));
                      for i:=Low(tmpguid.D4) to High(tmpguid.D4) do
                        hr.list.concat(Tai_const.Create_8bit(tmpguid.D4[i]));
                    end
                  else
                    Message(parser_e_illegal_expression);
                end;
              n.free;
              exit;
            end;
          if (def=rec_tguid) and ((token=_CSTRING) or (token=_CCHAR)) then
            begin
              n:=comp_expr(true,false);
              inserttypeconv(n,cshortstringtype);
              if n.nodetype=stringconstn then
                handle_stringconstn
              else
                Message(parser_e_illegal_expression);
              n.free;
              exit;
            end;
          { bitpacked record? }
          is_packed:=is_packed_record_or_object(def);
          if (is_packed) then
            begin
              { loadbitsize = 8, bitpacked records are always padded to    }
              { a multiple of a byte. packedbitsize will be set separately }
              { for each field                                             }
              initbitpackval(bp,0);
              bp.loadbitsize:=8;
            end;
          { normal record }
          consume(_LKLAMMER);
          curroffset:=0;
          symidx:=0;
          sorg:='';
          srsym:=tsym(def.symtable.SymList[symidx]);
          recsym := nil;
          startoffset:=hr.offset;
          while token<>_RKLAMMER do
            begin
              s:=pattern;
              sorg:=orgpattern;
              consume(_ID);
              consume(_COLON);
              error := false;
              recsym := tsym(def.symtable.Find(s));
              if not assigned(recsym) then
                begin
                  Message1(sym_e_illegal_field,sorg);
                  error := true;
                end;
              if (not error) and
                 (not assigned(srsym) or
                  (s <> srsym.name)) then
                { possible variant record (JM) }
                begin
                  { All parts of a variant start at the same offset      }
                  { Also allow jumping from one variant part to another, }
                  { as long as the offsets match                         }
                  if (assigned(srsym) and
                      (tfieldvarsym(recsym).fieldoffset = tfieldvarsym(srsym).fieldoffset)) or
                     { srsym is not assigned after parsing w2 in the      }
                     { typed const in the next example:                   }
                     {   type tr = record case byte of                    }
                     {          1: (l1,l2: dword);                        }
                     {          2: (w1,w2: word);                         }
                     {        end;                                        }
                     {   const r: tr = (w1:1;w2:1;l2:5);                  }
                     (tfieldvarsym(recsym).fieldoffset = curroffset) then
                    begin
                      srsym := recsym;
                      symidx := def.symtable.SymList.indexof(srsym)
                    end
                  { going backwards isn't allowed in any mode }
                  else if (tfieldvarsym(recsym).fieldoffset<curroffset) then
                    begin
                      Message(parser_e_invalid_record_const);
                      error := true;
                    end
                  { Delphi allows you to skip fields }
                  else if (m_delphi in current_settings.modeswitches) then
                    begin
                      Message1(parser_w_skipped_fields_before,sorg);
                      srsym := recsym;
                    end
                  { FPC and TP don't }
                  else
                    begin
                      Message1(parser_e_skipped_fields_before,sorg);
                      error := true;
                    end;
                end;
              if error then
                consume_all_until(_SEMICOLON)
              else
                begin
                  { if needed fill (alignment) }
                  if tfieldvarsym(srsym).fieldoffset>curroffset then
                    begin
                      if not(is_packed) then
                        fillbytes:=tfieldvarsym(srsym).fieldoffset-curroffset
                      else
                        begin
                          flush_packed_value(hr.list,bp);
                          { curoffset is now aligned to the next byte }
                          curroffset:=align(curroffset,8);
                          { offsets are in bits in this case }
                          fillbytes:=(tfieldvarsym(srsym).fieldoffset-curroffset) div 8;
                        end;
                      for i:=1 to fillbytes do
                        hr.list.concat(Tai_const.Create_8bit(0))
                    end;

                  { new position }
                  curroffset:=tfieldvarsym(srsym).fieldoffset;
                  if not(is_packed) then
                    inc(curroffset,tfieldvarsym(srsym).vardef.size)
                   else
                     inc(curroffset,tfieldvarsym(srsym).vardef.packedbitsize);

                  { read the data }
                  if not(is_packed) or
                     { only orddefs and enumdefs are bitpacked, as in gcc/gpc }
                     not(tfieldvarsym(srsym).vardef.typ in [orddef,enumdef]) then
                    begin
                      if is_packed then
                        begin
                          flush_packed_value(hr.list,bp);
                          curroffset:=align(curroffset,8);
                        end;
<<<<<<< HEAD
<<<<<<< HEAD
                      hr.offset:=startoffset+tfieldvarsym(srsym).fieldoffset;
                      read_typed_const_data(hr,tfieldvarsym(srsym).vardef);
=======
=======
>>>>>>> origin/fixes_2_2
                      read_typed_const_data(list,tfieldvarsym(srsym).vardef);
>>>>>>> graemeg/fixes_2_2
                    end
                  else
                    begin
                      bp.packedbitsize:=tfieldvarsym(srsym).vardef.packedbitsize;
                      parse_single_packed_const(hr.list,tfieldvarsym(srsym).vardef,bp);
                    end;

                  { keep previous field for checking whether whole }
                  { record was initialized (JM)                    }
                  recsym := srsym;
                  { goto next field }
                  inc(symidx);
                  if symidx<def.symtable.SymList.Count then
                    srsym:=tsym(def.symtable.SymList[symidx])
                  else
                    srsym:=nil;

                  if token=_SEMICOLON then
                    consume(_SEMICOLON)
                  else if (token=_COMMA) and (m_mac in current_settings.modeswitches) then
                    consume(_COMMA)
                  else
                    break;
                end;
            end;

          { are there any fields left, but don't complain if there only
            come other variant parts after the last initialized field }
          if assigned(srsym) and
             (
              (recsym=nil) or
              (tfieldvarsym(srsym).fieldoffset > tfieldvarsym(recsym).fieldoffset)
             ) then
            Message1(parser_w_skipped_fields_after,sorg);

          if not(is_packed) then
            fillbytes:=def.size-curroffset
          else
            begin
              flush_packed_value(hr.list,bp);
              curroffset:=align(curroffset,8);
              fillbytes:=def.size-(curroffset div 8);
            end;
          for i:=1 to fillbytes do
            hr.list.concat(Tai_const.Create_8bit(0));

          consume(_RKLAMMER);
        end;

        { note: hr is passed by value }
        procedure parse_objectdef(hr:threc;def:tobjectdef);
        var
          n      : tnode;
          i      : longint;
          obj    : tobjectdef;
          srsym  : tsym;
          st     : tsymtable;
          curroffset : aint;
          s,sorg : TIDString;
          vmtwritten : boolean;
          startoffset:aint;
        begin
          { no support for packed object }
          if is_packed_record_or_object(def) then
            begin
              Message(type_e_no_const_packed_record);
              exit;
            end;

          { only allow nil for implicit pointer object types }
          if is_implicit_pointer_object_type(def) then
            begin
              n:=comp_expr(true,false);
              if n.nodetype<>niln then
                begin
                  Message(parser_e_type_const_not_possible);
                  consume_all_until(_SEMICOLON);
                end
              else
                hr.list.concat(Tai_const.Create_sym(nil));
              n.free;
              exit;
            end;

          { for objects we allow it only if it doesn't contain a vmt }
          if (oo_has_vmt in def.objectoptions) and
             (m_fpc in current_settings.modeswitches) then
            begin
              Message(parser_e_type_object_constants);
              exit;
            end;

          consume(_LKLAMMER);
          startoffset:=hr.offset;
          curroffset:=0;
          vmtwritten:=false;
          while token<>_RKLAMMER do
            begin
              s:=pattern;
              sorg:=orgpattern;
              consume(_ID);
              consume(_COLON);
              srsym:=nil;
              obj:=tobjectdef(def);
              st:=obj.symtable;
              while (srsym=nil) and assigned(st) do
                begin
                  srsym:=tsym(st.Find(s));
                  if assigned(obj) then
                    obj:=obj.childof;
                  if assigned(obj) then
                    st:=obj.symtable
                  else
                    st:=nil;
                end;

              if (srsym=nil) or
                 (srsym.typ<>fieldvarsym) then
                begin
                  if (srsym=nil) then
                    Message1(sym_e_id_not_found,sorg)
                  else
                    Message1(sym_e_illegal_field,sorg);
                  consume_all_until(_RKLAMMER);
                  break;
                end
              else
                with tfieldvarsym(srsym) do
                  begin
                    { check position }
                    if fieldoffset<curroffset then
                      message(parser_e_invalid_record_const);

                    { check in VMT needs to be added for TP mode }
                    if not(vmtwritten) and
                       not(m_fpc in current_settings.modeswitches) and
                       (oo_has_vmt in def.objectoptions) and
                       (def.vmt_offset<fieldoffset) then
                      begin
                        for i:=1 to def.vmt_offset-curroffset do
                          hr.list.concat(tai_const.create_8bit(0));
                        hr.list.concat(tai_const.createname(def.vmt_mangledname,0));
                        { this is more general }
                        curroffset:=def.vmt_offset + sizeof(pint);
                        vmtwritten:=true;
                      end;

                    { if needed fill }
                    if fieldoffset>curroffset then
                      for i:=1 to fieldoffset-curroffset do
                        hr.list.concat(Tai_const.Create_8bit(0));

                    { new position }
                    curroffset:=fieldoffset+vardef.size;

                    { read the data }
                    hr.offset:=startoffset+fieldoffset;
                    read_typed_const_data(hr,vardef);

                    if not try_to_consume(_SEMICOLON) then
                      break;
                  end;
            end;
          if not(m_fpc in current_settings.modeswitches) and
             (oo_has_vmt in def.objectoptions) and
             (def.vmt_offset>=curroffset) then
            begin
              for i:=1 to def.vmt_offset-curroffset do
                hr.list.concat(tai_const.create_8bit(0));
              hr.list.concat(tai_const.createname(def.vmt_mangledname,0));
              { this is more general }
              curroffset:=def.vmt_offset + sizeof(pint);
            end;
          for i:=1 to def.size-curroffset do
            hr.list.concat(Tai_const.Create_8bit(0));
          consume(_RKLAMMER);
        end;

    procedure read_typed_const_data(var hr:threc;def:tdef);
      var
        old_block_type : tblock_type;
      begin
        old_block_type:=block_type;
        block_type:=bt_const;
        case def.typ of
          orddef :
            parse_orddef(hr.list,torddef(def));
          floatdef :
            parse_floatdef(hr.list,tfloatdef(def));
          classrefdef :
            parse_classrefdef(hr.list,tclassrefdef(def));
          pointerdef :
            parse_pointerdef(hr.list,tpointerdef(def));
          setdef :
            parse_setdef(hr.list,tsetdef(def));
          enumdef :
            parse_enumdef(hr.list,tenumdef(def));
          stringdef :
            parse_stringdef(hr,tstringdef(def));
          arraydef :
            parse_arraydef(hr,tarraydef(def));
          procvardef:
            parse_procvardef(hr.list,tprocvardef(def));
          recorddef:
            parse_recorddef(hr,trecorddef(def));
          objectdef:
            parse_objectdef(hr,tobjectdef(def));
          errordef:
            begin
               { try to consume something useful }
               if token=_LKLAMMER then
                 consume_all_until(_RKLAMMER)
               else
                 consume_all_until(_SEMICOLON);
            end;
          else
            Message(parser_e_type_const_not_possible);
        end;
        block_type:=old_block_type;
      end;

{$maxfpuregisters default}

    procedure read_typed_const(list:tasmlist;sym:tstaticvarsym;in_structure:boolean);
      var
        storefilepos : tfileposinfo;
        cursectype   : TAsmSectionType;
<<<<<<< HEAD
<<<<<<< HEAD
        hrec         : threc;
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
=======
        valuelist    : tasmlist;
>>>>>>> graemeg/fixes_2_2
=======
        valuelist    : tasmlist;
>>>>>>> origin/fixes_2_2
      begin
        { mark the staticvarsym as typedconst }
        include(sym.varoptions,vo_is_typed_const);
        { The variable has a value assigned }
        sym.varstate:=vs_initialised;
        { the variable can't be placed in a register }
        sym.varregable:=vr_none;

        { generate data for typed const }
        storefilepos:=current_filepos;
        current_filepos:=sym.fileinfo;

        if not(target_info.system in systems_typed_constants_node_init) then
          begin
            maybe_new_object_file(list);
            tcbuilder:=tasmlisttypedconstbuilderclass(ctypedconstbuilder).create(sym);
            tasmlisttypedconstbuilder(tcbuilder).parse_into_asmlist;
          end
        else
<<<<<<< HEAD
          begin
            if assigned(current_structdef) then
              previnit:=current_structdef.tcinitcode
            else
              previnit:=tnode(current_module.tcinitcode);
            tcbuilder:=tnodetreetypedconstbuilderclass(ctypedconstbuilder).create(sym,previnit);
            restree:=tnodetreetypedconstbuilder(tcbuilder).parse_into_nodetree;
            if assigned(current_structdef) then
              current_structdef.tcinitcode:=restree
            else
              current_module.tcinitcode:=restree;
          end;
=======
          cursectype:=sec_data;
        maybe_new_object_file(list);
<<<<<<< HEAD
<<<<<<< HEAD
        hrec.list:=tasmlist.create;
        hrec.origsym:=sym;
        hrec.offset:=0;
        read_typed_const_data(hrec,sym.vardef);
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

        { Parse hints }
        try_consume_hintdirective(sym.symoptions,sym.deprecatedmsg);
=======
=======
>>>>>>> origin/fixes_2_2
        valuelist:=tasmlist.create;
        read_typed_const_data(valuelist,sym.vardef);

        { Parse hints }
        try_consume_hintdirective(sym.symoptions);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

        consume(_SEMICOLON);

        { parse public/external/export/... }
<<<<<<< HEAD
<<<<<<< HEAD
        if not in_structure and
           (
            (
             (token = _ID) and
             (idtoken in [_EXPORT,_EXTERNAL,_WEAKEXTERNAL,_PUBLIC,_CVAR]) and
=======
=======
>>>>>>> origin/fixes_2_2
        if (
            (
             (token = _ID) and
             (idtoken in [_EXPORT,_EXTERNAL,_PUBLIC,_CVAR]) and
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             (m_cvar_support in current_settings.modeswitches)
            ) or
            (
             (m_mac in current_settings.modeswitches) and
             (
              (cs_external_var in current_settings.localswitches) or
              (cs_externally_visible in current_settings.localswitches)
             )
            )
           ) then
          read_public_and_external(sym);

<<<<<<< HEAD
<<<<<<< HEAD

        { try to parse a section directive }
        if not in_structure and (target_info.system in systems_allow_section) and
           (symtablestack.top.symtabletype in [staticsymtable,globalsymtable]) and
           (idtoken=_SECTION) then
          begin
            try_consume_sectiondirective(section);
            if section<>'' then
              begin
                if (sym.varoptions *[vo_is_external,vo_is_weak_external])<>[] then
                  Message(parser_e_externals_no_section);
                if sym.typ<>staticvarsym then
                  Message(parser_e_section_no_locals);
                tstaticvarsym(sym).section:=section;
                include(sym.varoptions, vo_has_section);
              end;
          end;

        if not(target_info.system in systems_typed_constants_node_init) then
          begin
            { only now get the final asmlist, because inserting the symbol
              information depends on potential section information set above }
            tasmlisttypedconstbuilder(tcbuilder).get_final_asmlists(reslist,datalist);
             { add the parsed value }
            list.concatlist(reslist);
            { and pointed data, if any }
            current_asmdata.asmlists[al_const].concatlist(datalist);
            { the (empty) lists themselves are freed by tcbuilder }
          end
        else
          begin
            { nothing to do }
          end;

<<<<<<< HEAD
        tcbuilder.free;
=======
        { add the parsed value }
        list.concatlist(hrec.list);
        hrec.list.free;
        list.concat(tai_symbol_end.Createname(sym.mangledname));
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
        { only now add items based on the symbolname, because it may }
        { have been modified by the directives parsed above          }
        new_section(list,cursectype,lower(sym.mangledname),const_align(sym.vardef.alignment));
        if (sym.owner.symtabletype=globalsymtable) or
           create_smartlink or
           (assigned(current_procinfo) and
            (po_inline in current_procinfo.procdef.procoptions)) or
           DLLSource then
          list.concat(Tai_symbol.Createname_global(sym.mangledname,AT_DATA,0))
        else
          list.concat(Tai_symbol.Createname(sym.mangledname,AT_DATA,0));

        { add the parsed value }
        list.concatlist(valuelist);
        valuelist.free;
        list.concat(tai_symbol_end.Createname(sym.mangledname));
>>>>>>> graemeg/fixes_2_2
        current_filepos:=storefilepos;
      end;

end.
