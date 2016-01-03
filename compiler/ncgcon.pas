{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate assembler for constant nodes which are the same for
    all (most) processors

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
unit ncgcon;

{$i fpcdefs.inc}

interface

    uses
       aasmbase,
       symtype,
       node,ncon;

    type
       tcgdataconstnode = class(tdataconstnode)
          procedure pass_generate_code;override;
       end;

       tcgrealconstnode = class(trealconstnode)
          procedure pass_generate_code;override;
       end;

       tcgordconstnode = class(tordconstnode)
          procedure pass_generate_code;override;
       end;

       tcgpointerconstnode = class(tpointerconstnode)
          procedure pass_generate_code;override;
       end;

       tcgstringconstnode = class(tstringconstnode)
          procedure pass_generate_code;override;
       protected
         procedure load_dynstring(const strpointerdef: tdef; const elementdef: tdef; const winlikewidestring: boolean); virtual;
       end;

       tcgsetconstnode = class(tsetconstnode)
         protected
          function emitvarsetconst: tasmsymbol; virtual;
          procedure handlevarsetconst;
         public
          procedure pass_generate_code;override;
       end;

       tcgnilnode = class(tnilnode)
          procedure pass_generate_code;override;
       end;

       tcgguidconstnode = class(tguidconstnode)
          procedure pass_generate_code;override;
       end;


implementation

    uses
      globtype,widestr,systems,
      verbose,globals,cutils,
<<<<<<< HEAD
<<<<<<< HEAD
      aasmcnst,
      symconst,symdef,aasmtai,aasmdata,aasmcpu,defutil,
=======
=======
>>>>>>> origin/fixes_2_2
      symconst,symdef,aasmbase,aasmtai,aasmdata,aasmcpu,defutil,
>>>>>>> graemeg/fixes_2_2
      cpuinfo,cpubase,
      cgbase,cgobj,cgutils,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      ncgutil,hlcgobj,cclasses,tgobj
=======
      ncgutil, cclasses,asmutils
>>>>>>> graemeg/cpstrnew
=======
      ncgutil, cclasses,asmutils
>>>>>>> graemeg/cpstrnew
=======
      ncgutil, cclasses,asmutils
>>>>>>> graemeg/cpstrnew
=======
      ncgutil, cclasses,asmutils
>>>>>>> origin/cpstrnew
      ;


{*****************************************************************************
                           TCGDATACONSTNODE
*****************************************************************************}

    procedure tcgdataconstnode.pass_generate_code;
      var
        l : tasmlabel;
        i : longint;
        b : byte;
      begin
        location_reset_ref(location,LOC_CREFERENCE,OS_NO,const_align(maxalign));
        current_asmdata.getglobaldatalabel(l);
        maybe_new_object_file(current_asmdata.asmlists[al_typedconsts]);
        new_section(current_asmdata.asmlists[al_typedconsts],sec_rodata_norel,l.name,const_align(maxalign));
        current_asmdata.asmlists[al_typedconsts].concat(Tai_label.Create(l));
        data.seek(0);
        for i:=0 to data.size-1 do
          begin
            data.read(b,1);
            current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_8bit(b));
          end;
        location.reference.symbol:=l;
      end;

{*****************************************************************************
                           TCGREALCONSTNODE
*****************************************************************************}

    procedure tcgrealconstnode.pass_generate_code;
      { I suppose the parser/pass_1 must make sure the generated real  }
      { constants are actually supported by the target processor? (JM) }
      const
<<<<<<< HEAD
        floattype2ait:array[tfloattype] of tairealconsttype=
          (aitrealconst_s32bit,aitrealconst_s64bit,aitrealconst_s80bit,aitrealconst_s80bit,aitrealconst_s64comp,aitrealconst_s64comp,aitrealconst_s128bit);

      { Since the value is stored always as bestreal, we share a single pool
        between all float types. This requires type and hiloswapped flag to
        be matched along with the value }
      type
        tfloatkey = record
          value: bestreal;
          typ: tfloattype;
          swapped: boolean;
        end;

=======
        floattype2ait:array[tfloattype] of taitype=
          (ait_real_32bit,ait_real_64bit,ait_real_80bit,ait_real_80bit,ait_comp_64bit,ait_comp_64bit,ait_real_128bit);
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
      var
         lastlabel : tasmlabel;
         realait : tairealconsttype;
         entry : PHashSetItem;
         key: tfloatkey;
{$ifdef ARM}
         hiloswapped : boolean;
{$endif ARM}

      begin
        location_reset_ref(location,LOC_CREFERENCE,def_cgsize(resultdef),const_align(resultdef.alignment));
        lastlabel:=nil;
        realait:=floattype2ait[tfloatdef(resultdef).floattype];
{$ifdef ARM}
        hiloswapped:=is_double_hilo_swapped;
{$endif ARM}
        { const already used ? }
        if not assigned(lab_real) then
          begin
            { there may be gap between record fields, zero it out }
            fillchar(key,sizeof(key),0);
            key.value:=value_real;
            key.typ:=tfloatdef(resultdef).floattype;
{$ifdef ARM}
            key.swapped:=hiloswapped;
{$endif ARM}
<<<<<<< HEAD
            entry := current_asmdata.ConstPools[sp_floats].FindOrAdd(@key, sizeof(key));

            lab_real := TAsmLabel(entry^.Data);  // is it needed anymore?

=======
                                 (tai_real_64bit(hp1).value=value_real) and is_number_float(tai_real_64bit(hp1).value) and (get_real_sign(value_real) = get_real_sign(tai_real_64bit(hp1).value))) or
                               ((realait=ait_real_80bit) and (tai_real_80bit(hp1).value=value_real) and (tai_real_80bit(hp1).savesize=resultdef.size) and is_number_float(tai_real_80bit(hp1).value) and (get_real_sign(value_real) = get_real_sign(tai_real_80bit(hp1).value))) or
{$ifdef cpufloat128}
                               ((realait=ait_real_128bit) and (tai_real_128bit(hp1).value=value_real) and is_number_float(tai_real_128bit(hp1).value) and (get_real_sign(value_real) = get_real_sign(tai_real_128bit(hp1).value))) or
{$endif cpufloat128}
                               ((realait=ait_comp_64bit) and (tai_comp_64bit(hp1).value=value_real) and is_number_float(tai_comp_64bit(hp1).value) and (get_real_sign(value_real) = get_real_sign(tai_comp_64bit(hp1).value)))
                              ) then
                              begin
                                { found! }
                                lab_real:=lastlabel;
                                break;
                              end;
                         end;
                       lastlabel:=nil;
                    end;
                  hp1:=tai(hp1.next);
               end;
>>>>>>> graemeg/cpstrnew
             { :-(, we must generate a new entry }
             if not(assigned(lab_real)) then
               begin
                  current_asmdata.getglobaldatalabel(lastlabel);
                  entry^.Data:=lastlabel;
                  lab_real:=lastlabel;
                  maybe_new_object_file(current_asmdata.asmlists[al_typedconsts]);
<<<<<<< HEAD
<<<<<<< HEAD
                  new_section(current_asmdata.asmlists[al_typedconsts],sec_rodata_norel,lastlabel.name,const_align(resultdef.alignment));
=======
                  new_section(current_asmdata.asmlists[al_typedconsts],sec_rodata_norel,lastlabel.name,const_align(resultdef.size));
>>>>>>> graemeg/fixes_2_2
=======
                  new_section(current_asmdata.asmlists[al_typedconsts],sec_rodata_norel,lastlabel.name,const_align(resultdef.size));
>>>>>>> origin/fixes_2_2
                  current_asmdata.asmlists[al_typedconsts].concat(Tai_label.Create(lastlabel));
                  case realait of
                    aitrealconst_s32bit :
                      begin
                        current_asmdata.asmlists[al_typedconsts].concat(tai_realconst.create_s32real(ts32real(value_real)));
                        { range checking? }
                        if floating_point_range_check_error and
<<<<<<< HEAD
<<<<<<< HEAD
                           (tai_realconst(current_asmdata.asmlists[al_typedconsts].last).value.s32val=MathInf.Value) then
=======
=======
>>>>>>> origin/fixes_2_2
                          (tai_real_32bit(current_asmdata.asmlists[al_typedconsts].last).value=MathInf.Value) then
>>>>>>> graemeg/fixes_2_2
                          Message(parser_e_range_check_error);
                      end;

                    aitrealconst_s64bit :
                      begin
{$ifdef ARM}
                        if hiloswapped then
                          current_asmdata.asmlists[al_typedconsts].concat(tai_realconst.create_s64real_hiloswapped(ts64real(value_real)))
                        else
{$endif ARM}
                          current_asmdata.asmlists[al_typedconsts].concat(tai_realconst.create_s64real(ts64real(value_real)));

                        { range checking? }
                        if floating_point_range_check_error and
<<<<<<< HEAD
<<<<<<< HEAD
                           (tai_realconst(current_asmdata.asmlists[al_typedconsts].last).value.s64val=MathInf.Value) then
=======
=======
>>>>>>> origin/fixes_2_2
                          (tai_real_64bit(current_asmdata.asmlists[al_typedconsts].last).value=MathInf.Value) then
>>>>>>> graemeg/fixes_2_2
                          Message(parser_e_range_check_error);
                     end;

                    aitrealconst_s80bit :
                      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                        current_asmdata.asmlists[al_typedconsts].concat(tai_realconst.create_s80real(value_real,tfloatdef(resultdef).size));
=======
                        current_asmdata.asmlists[al_typedconsts].concat(Tai_real_80bit.Create(value_real,resultdef.size));
>>>>>>> graemeg/cpstrnew
=======
                        current_asmdata.asmlists[al_typedconsts].concat(Tai_real_80bit.Create(value_real,resultdef.size));
>>>>>>> graemeg/cpstrnew
=======
                        current_asmdata.asmlists[al_typedconsts].concat(Tai_real_80bit.Create(value_real,resultdef.size));
>>>>>>> graemeg/cpstrnew
=======
                        current_asmdata.asmlists[al_typedconsts].concat(Tai_real_80bit.Create(value_real,resultdef.size));
>>>>>>> origin/cpstrnew

                        { range checking? }
                        if floating_point_range_check_error and
<<<<<<< HEAD
<<<<<<< HEAD
                           (tai_realconst(current_asmdata.asmlists[al_typedconsts].last).value.s80val=MathInf.Value) then
=======
=======
>>>>>>> origin/fixes_2_2
                          (tai_real_80bit(current_asmdata.asmlists[al_typedconsts].last).value=MathInf.Value) then
>>>>>>> graemeg/fixes_2_2
                          Message(parser_e_range_check_error);
                      end;
{$ifdef cpufloat128}
                    aitrealconst_s128bit :
                      begin
                        current_asmdata.asmlists[al_typedconsts].concat(tai_realconst.create_s128real(value_real));

                        { range checking? }
                        if floating_point_range_check_error and
<<<<<<< HEAD
<<<<<<< HEAD
                           (tai_realconst(current_asmdata.asmlists[al_typedconsts].last).value.s128val=MathInf.Value) then
=======
=======
>>>>>>> origin/fixes_2_2
                          (tai_real_128bit(current_asmdata.asmlists[al_typedconsts].last).value=MathInf.Value) then
>>>>>>> graemeg/fixes_2_2
                          Message(parser_e_range_check_error);
                      end;
{$endif cpufloat128}

                    { the round is necessary for native compilers where comp isn't a float }
                    aitrealconst_s64comp :
                      if (value_real>9223372036854775807.0) or (value_real<-9223372036854775808.0) then
                        message(parser_e_range_check_error)
                      else
                        current_asmdata.asmlists[al_typedconsts].concat(tai_realconst.create_s64compreal(round(value_real)));
                  else
                    internalerror(10120);
                  end;
               end;
          end;
        location.reference.symbol:=lab_real;
      end;

{*****************************************************************************
                            TCGORDCONSTNODE
*****************************************************************************}

    procedure tcgordconstnode.pass_generate_code;
      begin
         location_reset(location,LOC_CONSTANT,def_cgsize(resultdef));
{$ifdef cpu64bitalu}
         location.value:=value.svalue;
{$else cpu64bitalu}
         location.value64:=value.svalue;
{$endif cpu64bitalu}
      end;


{*****************************************************************************
                          TCGPOINTERCONSTNODE
*****************************************************************************}

    procedure tcgpointerconstnode.pass_generate_code;
      begin
         { an integer const. behaves as a memory reference }
         location_reset(location,LOC_CONSTANT,OS_ADDR);
         location.value:=aint(value);
      end;


{*****************************************************************************
                          TCGSTRINGCONSTNODE
*****************************************************************************}

    procedure tcgstringconstnode.pass_generate_code;
      var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         lastlabel: tasmlabofs;
         pc: pchar;
         l: longint;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
         hp1,hp2 : tai;
         l1,
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
         lastlabel   : tasmlabel;
         pc       : pchar;
         l: longint;
         href: treference;
         pooltype: TConstPoolType;
>>>>>>> graemeg/cpstrnew
         pool: THashSet;
         entry: PHashSetItem;
         winlikewidestring: boolean;
         elementdef: tdef;
         strpointerdef: tdef;
         datatcb: ttai_typedconstbuilder;
         datadef: tdef;

      const
        PoolMap: array[tconststringtype] of TConstPoolType = (
          sp_conststr,
          sp_shortstr,
          sp_longstr,
          sp_ansistr,
          sp_widestr,
          sp_unicodestr
        );
      begin
         case cst_type of
           cst_shortstring,
           cst_conststring,
           cst_ansistring:
             begin
               elementdef:=cansichartype;
               strpointerdef:=charpointertype;
             end;
           cst_widestring,
           cst_unicodestring:
             begin
               elementdef:=cwidechartype;
               strpointerdef:=widecharpointertype;
             end;
           else
             internalerror(2014032803);
         end;
         { for empty ansistrings we could return a constant 0 }
         if (cst_type in [cst_ansistring,cst_widestring,cst_unicodestring]) and (len=0) then
          begin
            location_reset(location,LOC_CONSTANT,def_cgsize(strpointerdef));
            location.value:=0;
            exit;
          end;
         winlikewidestring:=(cst_type=cst_widestring) and (tf_winlikewidestring in target_info.flags);
         { const already used ? }
         if not assigned(lab_str) then
           begin
              pool := current_asmdata.ConstPools[PoolMap[cst_type]];

              if cst_type in [cst_widestring, cst_unicodestring] then
                entry := pool.FindOrAdd(pcompilerwidestring(value_str)^.data,len*cwidechartype.size)
              else
              if cst_type = cst_ansistring then
                entry := PHashSetItem(TTagHashSet(pool).FindOrAdd(value_str,len,tstringdef(resultdef).encoding))
              else
                entry := pool.FindOrAdd(value_str,len);

              lab_str := TAsmLabel(entry^.Data);  // is it needed anymore?

              { :-(, we must generate a new entry }
              if not assigned(entry^.Data) then
                begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  datatcb:=ctai_typedconstbuilder.create([tcalo_is_lab,tcalo_make_dead_strippable]);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
                   current_asmdata.getdatalabel(lastlabel);
                   lab_str:=lastlabel;
                   maybe_new_object_file(current_asmdata.asmlists[al_typedconsts]);
                   if (len=0) or
                      not(cst_type in [cst_ansistring,cst_widestring]) then
                     new_section(current_asmdata.asmlists[al_typedconsts],sec_rodata_norel,lastlabel.name,const_align(sizeof(aint)))
                   else
                     new_section(current_asmdata.asmlists[al_typedconsts],sec_rodata,lastlabel.name,const_align(sizeof(aint)));
                   current_asmdata.asmlists[al_typedconsts].concat(Tai_label.Create(lastlabel));
                   { generate an ansi string ? }
>>>>>>> graemeg/fixes_2_2
                   case cst_type of
                      cst_ansistring:
                        begin
                           if len=0 then
                             InternalError(2008032301)   { empty string should be handled above }
                           else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                             begin
<<<<<<< HEAD
                               lastlabel:=datatcb.emit_ansistring_const(current_asmdata.AsmLists[al_typedconsts],value_str,len,tstringdef(resultdef).encoding);
                               { because we hardcode the offset below due to it
                                 not being stored in the hashset, check here }
                               if lastlabel.ofs<>datatcb.get_string_symofs(st_ansistring,false) then
                                 internalerror(2012051703);
=======
                                current_asmdata.getdatalabel(l1);
                                current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_sym(l1));
                                current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_aint(-1));
                                current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_aint(len));
                                { make sure the string doesn't get dead stripped if the header is referenced }
                                if (target_info.system in systems_darwin) then
                                  current_asmdata.asmlists[al_typedconsts].concat(tai_directive.create(asd_reference,l1.name));
                                current_asmdata.asmlists[al_typedconsts].concat(Tai_label.Create(l1));
                                { ... and vice versa }
                                if (target_info.system in systems_darwin) then
                                  current_asmdata.asmlists[al_typedconsts].concat(tai_directive.create(asd_reference,lab_str.name));
                                { include also terminating zero }
                                getmem(pc,len+1);
                                move(value_str^,pc^,len);
                                pc[len]:=#0;
                                current_asmdata.asmlists[al_typedconsts].concat(Tai_string.Create_pchar(pc,len+1));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                             end;
                           { no contents of the datatcb itself to concatenate,
                             as we will just return the address of the emitted
                             ansistring constant record }
=======
                             lastlabel:=emit_ansistring_const(current_asmdata.AsmLists[al_typedconsts],value_str,len,tstringdef(resultdef).encoding);
>>>>>>> graemeg/cpstrnew
=======
                             lastlabel:=emit_ansistring_const(current_asmdata.AsmLists[al_typedconsts],value_str,len,tstringdef(resultdef).encoding);
>>>>>>> graemeg/cpstrnew
=======
                             lastlabel:=emit_ansistring_const(current_asmdata.AsmLists[al_typedconsts],value_str,len,tstringdef(resultdef).encoding);
>>>>>>> graemeg/cpstrnew
=======
                             lastlabel:=emit_ansistring_const(current_asmdata.AsmLists[al_typedconsts],value_str,len,tstringdef(resultdef).encoding);
>>>>>>> origin/cpstrnew
                        end;
                      cst_unicodestring,
                      cst_widestring:
                        begin
                           if len=0 then
                             InternalError(2008032302)   { empty string should be handled above }
                           else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                             begin
<<<<<<< HEAD
                               lastlabel:=datatcb.emit_unicodestring_const(current_asmdata.AsmLists[al_typedconsts],
                                               value_str,
                                               tstringdef(resultdef).encoding,
                                               winlikewidestring);
                               { because we hardcode the offset below due to it
                                 not being stored in the hashset, check here }
                               if lastlabel.ofs<>datatcb.get_string_symofs(tstringdef(resultdef).stringtype,winlikewidestring) then
                                 internalerror(2012051704);
=======
                                current_asmdata.getdatalabel(l1);
                                current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_sym(l1));

                                { we use always UTF-16 coding for constants }
                                { at least for now                          }
                                { Consts.concat(Tai_const.Create_8bit(2)); }
                                if tf_winlikewidestring in target_info.flags then
                                  current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_32bit(len*cwidechartype.size))
                                else
                                  begin
                                    current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_aint(-1));
                                    current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_aint(len*cwidechartype.size));
                                  end;
                                { make sure the string doesn't get dead stripped if the header is referenced }
                                if (target_info.system in systems_darwin) then
                                  current_asmdata.asmlists[al_typedconsts].concat(tai_directive.create(asd_reference,l1.name));
                                current_asmdata.asmlists[al_typedconsts].concat(Tai_label.Create(l1));
                                { ... and vice versa }
                                if (target_info.system in systems_darwin) then
                                  current_asmdata.asmlists[al_typedconsts].concat(tai_directive.create(asd_reference,lab_str.name));
                                for i:=0 to len-1 do
                                  current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_16bit(pcompilerwidestring(value_str)^.data[i]));
                                { terminating zero }
                                current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_16bit(0));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                             end;
                           { no contents of the datatcb itself to concatenate,
                             as we will just return the address of the emitted
                             unicode/widestring constant record }
                        end;
                      cst_shortstring:
                        begin
                          current_asmdata.getglobaldatalabel(lastlabel.lab);

=======
                             lastlabel := emit_unicodestring_const(current_asmdata.AsmLists[al_typedconsts],
                                             value_str,
                                             tstringdef(resultdef).encoding,
                                             (cst_type=cst_widestring) and (tf_winlikewidestring in target_info.flags));
                        end;
                      cst_shortstring:
                        begin
=======
=======
>>>>>>> origin/cpstrnew
                             lastlabel := emit_unicodestring_const(current_asmdata.AsmLists[al_typedconsts],
                                             value_str,
                                             tstringdef(resultdef).encoding,
                                             (cst_type=cst_widestring) and (tf_winlikewidestring in target_info.flags));
                        end;
                      cst_shortstring:
                        begin
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
                             lastlabel := emit_unicodestring_const(current_asmdata.AsmLists[al_typedconsts],
                                             value_str,
                                             tstringdef(resultdef).encoding,
                                             (cst_type=cst_widestring) and (tf_winlikewidestring in target_info.flags));
                        end;
                      cst_shortstring:
                        begin
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                          current_asmdata.getdatalabel(lastlabel);
                          maybe_new_object_file(current_asmdata.asmlists[al_typedconsts]);
                          new_section(current_asmdata.asmlists[al_typedconsts],sec_rodata_norel,lastlabel.name,const_align(sizeof(pint)));

                          current_asmdata.asmlists[al_typedconsts].concat(Tai_label.Create(lastlabel));
>>>>>>> graemeg/cpstrnew
                          { truncate strings larger than 255 chars }
                          if len>255 then
                           l:=255
                          else
                           l:=len;
                          { include length and terminating zero for quick conversion to pchar }
                          getmem(pc,l+2);
                          move(value_str^,pc[1],l);
                          pc[0]:=chr(l);
                          pc[l+1]:=#0;
                          datadef:=carraydef.getreusable(cansichartype,l+2);
                          datatcb.maybe_begin_aggregate(datadef);
                          datatcb.emit_tai(Tai_string.Create_pchar(pc,l+2),datadef);
                          datatcb.maybe_end_aggregate(datadef);
                          current_asmdata.asmlists[al_typedconsts].concatList(
                            datatcb.get_final_asmlist(lastlabel.lab,datadef,sec_rodata_norel,lastlabel.lab.name,const_align(sizeof(pint)))
                          );
                        end;
                      cst_conststring:
                        begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                          current_asmdata.getglobaldatalabel(lastlabel.lab);

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                          current_asmdata.getdatalabel(lastlabel);
                          maybe_new_object_file(current_asmdata.asmlists[al_typedconsts]);
                          new_section(current_asmdata.asmlists[al_typedconsts],sec_rodata_norel,lastlabel.name,const_align(sizeof(pint)));

                          current_asmdata.asmlists[al_typedconsts].concat(Tai_label.Create(lastlabel));
>>>>>>> graemeg/cpstrnew
                          { include terminating zero }
                          getmem(pc,len+1);
                          move(value_str^,pc[0],len);
                          pc[len]:=#0;
                          { the data includes the terminating #0 because this
                            string can be used for pchar assignments (but it's
                            also used for array-of-char assignments, in which
                            case the terminating #0 is not part of the data) }
                          datadef:=carraydef.getreusable(cansichartype,len+1);
                          datatcb.maybe_begin_aggregate(datadef);
                          datatcb.emit_tai(Tai_string.Create_pchar(pc,len+1),datadef);
                          datatcb.maybe_end_aggregate(datadef);
                          current_asmdata.asmlists[al_typedconsts].concatList(
                            datatcb.get_final_asmlist(lastlabel.lab,datadef,sec_rodata_norel,lastlabel.lab.name,const_align(sizeof(pint)))
                          );
                        end;
                      else
                        internalerror(2013120103);
                   end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                   datatcb.free;
                   lab_str:=lastlabel.lab;
                   entry^.Data:=lastlabel.lab;
=======
                   lab_str:=lastlabel;
                   entry^.Data:=lastlabel;
>>>>>>> graemeg/cpstrnew
=======
                   lab_str:=lastlabel;
                   entry^.Data:=lastlabel;
>>>>>>> graemeg/cpstrnew
=======
                   lab_str:=lastlabel;
                   entry^.Data:=lastlabel;
>>>>>>> graemeg/cpstrnew
=======
                   lab_str:=lastlabel;
                   entry^.Data:=lastlabel;
>>>>>>> origin/cpstrnew
                end;
           end;
         if cst_type in [cst_ansistring, cst_widestring, cst_unicodestring] then
           begin
             location_reset(location, LOC_REGISTER, def_cgsize(strpointerdef));
             location.register:=hlcg.getaddressregister(current_asmdata.CurrAsmList,strpointerdef);
             load_dynstring(strpointerdef, elementdef, winlikewidestring);
           end
         else
           begin
             location_reset_ref(location, LOC_CREFERENCE, def_cgsize(resultdef), const_align(strpointerdef.size));
             location.reference.symbol:=lab_str;
           end;
      end;


    procedure tcgstringconstnode.load_dynstring(const strpointerdef: tdef; const elementdef: tdef; const winlikewidestring: boolean);
      var
        href: treference;
      begin
        reference_reset_symbol(href, lab_str,
          ctai_typedconstbuilder.get_string_symofs(tstringdef(resultdef).stringtype, winlikewidestring),
          const_align(strpointerdef.size));
        hlcg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList, elementdef, strpointerdef, href, location.register)
      end;


{*****************************************************************************
                           TCGSETCONSTNODE
*****************************************************************************}

<<<<<<< HEAD
    function tcgsetconstnode.emitvarsetconst: tasmsymbol;
=======
    procedure tcgsetconstnode.pass_generate_code;
      var
         hp1         : tai;
         lastlabel   : tasmlabel;
         i, diff     : longint;
         neededtyp   : taiconst_type;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      type
        setbytes=array[0..31] of byte;
        Psetbytes=^setbytes;
      var
        lab: tasmlabel;
        i: longint;
        tcb: ttai_typedconstbuilder;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
        current_asmdata.getglobaldatalabel(lab);
        result:=lab;
        lab_set:=lab;
        tcb:=ctai_typedconstbuilder.create([tcalo_is_lab,tcalo_make_dead_strippable]);
        tcb.maybe_begin_aggregate(resultdef);
        if (source_info.endian=target_info.endian) then
          for i:=0 to resultdef.size-1 do
            tcb.emit_tai(tai_const.create_8bit(Psetbytes(value_set)^[i]),u8inttype)
        else
          for i:=0 to resultdef.size-1 do
            tcb.emit_tai(tai_const.create_8bit(reverse_byte(Psetbytes(value_set)^[i])),u8inttype);
        tcb.maybe_end_aggregate(resultdef);
        current_asmdata.asmlists[al_typedconsts].concatlist(tcb.get_final_asmlist(
          result,resultdef,sec_rodata_norel,result.name,const_align(8)));
        tcb.free;
      end;


    procedure tcgsetconstnode.handlevarsetconst;
      var
         entry       : PHashSetItem;
      begin
        location_reset_ref(location,LOC_CREFERENCE,OS_NO,const_align(8));
        { const already used ? }
        if not assigned(lab_set) then
          begin
            entry := current_asmdata.ConstPools[sp_varsets].FindOrAdd(value_set, resultdef.size);

             { :-(, we must generate a new entry }
             if not assigned(entry^.Data) then
               entry^.Data:=emitvarsetconst;
             lab_set := TAsmSymbol(entry^.Data);
=======
=======
>>>>>>> origin/fixes_2_2
        adjustforsetbase;

        { small sets are loaded as constants }
        if not(is_varset(resultdef)) then
         begin
           location_reset(location,LOC_CONSTANT,int_cgsize(resultdef.size));
           if (source_info.endian=target_info.endian) then
             begin
{$if defined(FPC_NEW_BIGENDIAN_SETS) or defined(FPC_LITTLE_ENDIAN)}
               { not plongint, because that will "sign extend" the set on 64 bit platforms }
               { if changed to "paword", please also modify "32-resultdef.size*8" and      }
               { cross-endian code below                                                   }
               location.value:=pCardinal(value_set)^
{$else}
               location.value:=reverse_byte(Psetbytes(value_set)^[0]);
               location.value:=location.value or (reverse_byte(Psetbytes(value_set)^[1]) shl 8);
               location.value:=location.value or (reverse_byte(Psetbytes(value_set)^[2]) shl 16);
               location.value:=location.value or (reverse_byte(Psetbytes(value_set)^[3]) shl 24);
{$endif}
             end
           else
             begin
               location.value:=swapendian(Pcardinal(value_set)^);
               location.value:= reverse_byte (location.value         and $ff)         or
                               (reverse_byte((location.value shr  8) and $ff) shl  8) or
                               (reverse_byte((location.value shr 16) and $ff) shl 16) or
                               (reverse_byte((location.value shr 24) and $ff) shl 24);
             end;
           if (target_info.endian=endian_big) then
             location.value:=location.value shr (32-resultdef.size*8);
           exit;
         end;
        location_reset(location,LOC_CREFERENCE,OS_NO);
        neededtyp:=aitconst_8bit;
        lastlabel:=nil;
        { const already used ? }
        if not assigned(lab_set) then
          begin
             { tries to found an old entry }
             hp1:=tai(current_asmdata.asmlists[al_typedconsts].first);
             while assigned(hp1) do
               begin
                  if hp1.typ=ait_label then
                    lastlabel:=tai_label(hp1).labsym
                  else
                    begin
                      if (lastlabel<>nil) and
                        (hp1.typ=ait_const) and
                        (tai_const(hp1).consttype=neededtyp) then
                        begin
                          if (tai_const(hp1).consttype=aitconst_8bit) then
                           begin
                             { compare normal set }
                             i:=0;
                             while assigned(hp1) and (i<32) do
                              begin
                                if (source_info.endian=target_info.endian) then
                                  begin
{$if defined(FPC_NEW_BIGENDIAN_SETS) or defined(FPC_LITTLE_ENDIAN)}
                                    if tai_const(hp1).value<>Psetbytes(value_set)^[i ] then
{$else}
                                    if tai_const(hp1).value<>reverse_byte(Psetbytes(value_set)^[i xor 3]) then
{$endif}
                                      break
                                  end
                                else if tai_const(hp1).value<>reverse_byte(Psetbytes(value_set)^[i]) then
                                  break;
                                inc(i);
                                hp1:=tai(hp1.next);
                              end;
                             if i=32 then
                              begin
                                { found! }
                                lab_set:=lastlabel;
                                break;
                              end;
                             { leave when the end of consts is reached, so no
                               hp1.next is done }
                             if not assigned(hp1) then
                              break;
                           end
                          else
                           begin
                             { compare small set }
                             if paint(value_set)^=tai_const(hp1).value then
                              begin
                                { found! }
                                lab_set:=lastlabel;
                                break;
                              end;
                           end;
                        end;
                      lastlabel:=nil;
                    end;
                  hp1:=tai(hp1.next);
               end;
             { :-(, we must generate a new entry }
             if not assigned(lab_set) then
               begin
                 current_asmdata.getdatalabel(lastlabel);
                 lab_set:=lastlabel;
                 maybe_new_object_file(current_asmdata.asmlists[al_typedconsts]);
                 new_section(current_asmdata.asmlists[al_typedconsts],sec_rodata_norel,lastlabel.name,const_align(sizeof(aint)));
                 current_asmdata.asmlists[al_typedconsts].concat(Tai_label.Create(lastlabel));
                 { already handled at the start of this method?? (JM)
                 if tsetdef(resultdef).settype=smallset then
                  begin
                    move(value_set^,i,sizeof(longint));
                    Consts.concat(Tai_const.Create_32bit(i));
                  end
                 else
                 }
                  begin
                    if (source_info.endian=target_info.endian) then
{$if defined(FPC_NEW_BIGENDIAN_SETS) or defined(FPC_LITTLE_ENDIAN)}
                      for i:=0 to 31 do
                        current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_8bit(Psetbytes(value_set)^[i]))
{$else}
                      for i:=0 to 31 do
                        current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_8bit(reverse_byte(Psetbytes(value_set)^[i xor 3])))
{$endif}
                    else
                      for i:=0 to 31 do
                        current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_8bit(reverse_byte(Psetbytes(value_set)^[i])));
                  end;
               end;
>>>>>>> graemeg/fixes_2_2
          end;
        location.reference.symbol:=lab_set;
      end;


    procedure tcgsetconstnode.pass_generate_code;
       type
         setbytes=array[0..31] of byte;
         Psetbytes=^setbytes;

        procedure smallsetconst;
<<<<<<< HEAD
          begin
            location_reset(location,LOC_CONSTANT,int_cgsize(resultdef.size));
            if (source_info.endian=target_info.endian) then
              begin
                { not plongint, because that will "sign extend" the set on 64 bit platforms }
                { if changed to "paword", please also modify "32-resultdef.size*8" and      }
                { cross-endian code below                                                   }
                { Extra aint type cast to avoid range errors                                }
                location.value:=aint(pCardinal(value_set)^)
              end
            else
              begin
                location.value:=aint(swapendian(Pcardinal(value_set)^));
                location.value:=aint(
                                   reverse_byte (location.value         and $ff)         or
                                  (reverse_byte((location.value shr  8) and $ff) shl  8) or
                                  (reverse_byte((location.value shr 16) and $ff) shl 16) or
                                  (reverse_byte((location.value shr 24) and $ff) shl 24)
                                );
              end;
            if (target_info.endian=endian_big) then
              location.value:=location.value shr (32-resultdef.size*8);
          end;
=======
        begin
          location_reset(location,LOC_CONSTANT,int_cgsize(resultdef.size));
          if (source_info.endian=target_info.endian) then
            begin
              { not plongint, because that will "sign extend" the set on 64 bit platforms }
              { if changed to "paword", please also modify "32-resultdef.size*8" and      }
              { cross-endian code below                                                   }
              { Extra aint type cast to avoid range errors                                }
              location.value:=aint(pCardinal(value_set)^)
            end
          else
            begin
              location.value:=swapendian(Pcardinal(value_set)^);
              location.value:=aint(
                                 reverse_byte (location.value         and $ff)         or
                                (reverse_byte((location.value shr  8) and $ff) shl  8) or
                                (reverse_byte((location.value shr 16) and $ff) shl 16) or
                                (reverse_byte((location.value shr 24) and $ff) shl 24)
                              );
            end;
          if (target_info.endian=endian_big) then
            location.value:=location.value shr (32-resultdef.size*8);
        end;

        procedure varsetconst;
        var
           hp1         : tai;
           lastlabel   : tasmlabel;
           i           : longint;
           neededtyp   : taiconst_type;
        begin
          location_reset_ref(location,LOC_CREFERENCE,OS_NO,const_align(8));
          neededtyp:=aitconst_8bit;
          lastlabel:=nil;
          { const already used ? }
          if not assigned(lab_set) then
            begin
              { tries to found an old entry }
              hp1:=tai(current_asmdata.asmlists[al_typedconsts].first);
              while assigned(hp1) do
                begin
                   if hp1.typ=ait_label then
                     lastlabel:=tai_label(hp1).labsym
                   else
                     begin
                       if (lastlabel<>nil) and
                         (hp1.typ=ait_const) and
                         (tai_const(hp1).consttype=neededtyp) then
                         begin
                           if (tai_const(hp1).consttype=aitconst_8bit) then
                            begin
                              { compare normal set }
                              i:=0;
                              while assigned(hp1) and (i<32) do
                               begin
                                 if (source_info.endian=target_info.endian) then
                                   begin
                                     if tai_const(hp1).value<>Psetbytes(value_set)^[i ] then
                                       break
                                   end
                                 else if tai_const(hp1).value<>reverse_byte(Psetbytes(value_set)^[i]) then
                                   break;
                                 inc(i);
                                 hp1:=tai(hp1.next);
                               end;
                              if i=32 then
                               begin
                                 { found! }
                                 lab_set:=lastlabel;
                                 break;
                               end;
                              { leave when the end of consts is reached, so no
                                hp1.next is done }
                              if not assigned(hp1) then
                               break;
                            end
                           else
                            begin
                              { compare small set }
                              if paint(value_set)^=tai_const(hp1).value then
                               begin
                                 { found! }
                                 lab_set:=lastlabel;
                                 break;
                               end;
                            end;
                         end;
                       lastlabel:=nil;
                     end;
                   hp1:=tai(hp1.next);
                 end;
               { :-(, we must generate a new entry }
               if not assigned(lab_set) then
                 begin
                   current_asmdata.getdatalabel(lastlabel);
                   lab_set:=lastlabel;
                   maybe_new_object_file(current_asmdata.asmlists[al_typedconsts]);
                   new_section(current_asmdata.asmlists[al_typedconsts],sec_rodata_norel,lastlabel.name,const_align(8));
                   current_asmdata.asmlists[al_typedconsts].concat(Tai_label.Create(lastlabel));
                   if (source_info.endian=target_info.endian) then
                     for i:=0 to 31 do
                       current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_8bit(Psetbytes(value_set)^[i]))
                   else
                     for i:=0 to 31 do
                       current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_8bit(reverse_byte(Psetbytes(value_set)^[i])));
                 end;
            end;
          location.reference.symbol:=lab_set;
        end;
>>>>>>> graemeg/cpstrnew

      begin
        adjustforsetbase;

        { small sets are loaded as constants }
        if is_smallset(resultdef) then
          smallsetconst
        else
          handlevarsetconst;
      end;


{*****************************************************************************
                             TCGNILNODE
*****************************************************************************}

    procedure tcgnilnode.pass_generate_code;
      begin
         location_reset(location,LOC_CONSTANT,OS_ADDR);
         location.value:=0;
      end;


{*****************************************************************************
                          TCGGUIDCONSTNODE
*****************************************************************************}

    procedure tcgguidconstnode.pass_generate_code;
      var
         lastlabel   : tasmlabel;
         entry       : PHashSetItem;
         datatcb     : ttai_typedconstbuilder;
      begin
        location_reset_ref(location,LOC_CREFERENCE,OS_NO,const_align(16));
        lastlabel:=nil;
        { const already used ? }
        if not assigned(lab_set) then
          begin
            entry:=current_asmdata.ConstPools[sp_guids].FindOrAdd(@value,sizeof(value));
            lab_set:=TAsmLabel(entry^.Data);  // is it needed anymore?

             { :-(, we must generate a new entry }
             if not assigned(entry^.Data) then
               begin
                 current_asmdata.getglobaldatalabel(lastlabel);
                 datatcb:=ctai_typedconstbuilder.create([tcalo_is_lab,tcalo_make_dead_strippable]);
                 datatcb.emit_guid_const(value);
                 current_asmdata.asmlists[al_typedconsts].concatList(
                   datatcb.get_final_asmlist(lastlabel,rec_tguid,sec_rodata_norel,lastlabel.name,const_align(16)));
                 datatcb.free;
                 lab_set:=lastlabel;
                 entry^.Data:=lastlabel;
               end;
          end;
        location.reference.symbol:=lab_set;
      end;


begin
   cdataconstnode:=tcgdataconstnode;
   crealconstnode:=tcgrealconstnode;
   cordconstnode:=tcgordconstnode;
   cpointerconstnode:=tcgpointerconstnode;
   cstringconstnode:=tcgstringconstnode;
   csetconstnode:=tcgsetconstnode;
   cnilnode:=tcgnilnode;
   cguidconstnode:=tcgguidconstnode;
end.
