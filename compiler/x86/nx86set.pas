{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate x86 assembler for in/case nodes

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
unit nx86set;

{$i fpcdefs.inc}

interface

    uses
      globtype,constexp,
      node,nset,pass_1,ncgset;

    type
       tx86innode = class(tinnode)
         procedure pass_generate_code;override;
         function pass_1 : tnode;override;
       end;

      tx86casenode = class(tcgcasenode)
         function  has_jumptable : boolean;override;
         procedure genjumptable(hp : pcaselabel;min_,max_ : int64);override;
         procedure genlinearlist(hp : pcaselabel);override;
         procedure genjmptreeentry(p : pcaselabel;parentvalue : TConstExprInt);override;
      end;

implementation

    uses
      systems,
      verbose,globals,
      symconst,symdef,defutil,cutils,
      aasmbase,aasmtai,aasmdata,aasmcpu,
      cgbase,pass_2,tgobj,
      ncon,
      cpubase,
      cga,cgobj,hlcgobj,cgutils,ncgutil,
      cgx86,
      procinfo;

{*****************************************************************************
                                  TX86CASENODE
*****************************************************************************}

    function tx86casenode.has_jumptable : boolean;
      begin
        has_jumptable:=true;
      end;


    procedure tx86casenode.genjumptable(hp : pcaselabel;min_,max_ : int64);
      var
        table : tasmlabel;
        last : TConstExprInt;
        indexreg : tregister;
        href : treference;
        jtlist: tasmlist;
        opcgsize: tcgsize;
        jumpreg: tregister;
        labeltyp: taiconst_type;
        AlmostExhaustive: Boolean;
        lv, hv: TConstExprInt;
        ExhaustiveLimit, Range, x, oldmin : int64;

      const
        ExhaustiveLimitBase = 32;

        procedure genitem(list:TAsmList;t : pcaselabel);
          var
            i : TConstExprInt;
          begin
            if assigned(t^.less) then
              genitem(list,t^.less);

            { fill possible hole }
            i:=last+1;
            while i<=t^._low-1 do
              begin
                list.concat(Tai_const.Create_type_sym(labeltyp,elselabel));
                i:=i+1;
              end;
            i:=t^._low;
            while i<=t^._high do
              begin
                list.concat(Tai_const.Create_type_sym(labeltyp,blocklabel(t^.blockid)));
                i:=i+1;
              end;
            last:=t^._high;
            if assigned(t^.greater) then
              genitem(list,t^.greater);
          end;

      begin
        lv:=0;
        hv:=0;
        oldmin:=0;
        last:=min_;
        { This generates near pointers on i8086 }
        labeltyp:=aitconst_ptr;
        opcgsize:=def_cgsize(opsize);

        AlmostExhaustive := False;

        if not(jumptable_no_range) then
          begin

            getrange(left.resultdef,lv,hv);
            Range := aint(max_)-aint(min_);

            if (cs_opt_size in current_settings.optimizerswitches) then
              { Limit size of jump tables for small enumerations so they have
                to be at least two-thirds full before being considered for the
                "almost exhaustive" treatment }
              ExhaustiveLimit := min(ExhaustiveLimitBase, labelcoverage shl 1)
            else
              ExhaustiveLimit := ExhaustiveLimitBase;

            { If true, then this indicates that almost every possible value of x is covered by
              a label.  As such, it's more cost-efficient to remove the initial range check and
              instead insert the remaining values into the jump table, pointing at elselabel. [Kit] }
            if ((hv - lv) - Range <= ExhaustiveLimit) then
              begin
                oldmin := min_;
                min_ := lv.svalue;
                AlmostExhaustive := True;
              end
            else
              begin
                { a <= x <= b <-> unsigned(x-a) <= (b-a) }
                cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SUB,opcgsize,aint(min_),hregister);
                { case expr greater than max_ => goto elselabel }
                cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opcgsize,OC_A,aint(max_)-aint(min_),hregister,elselabel);
                min_:=0;
                { do not sign extend when we load the index register, as we applied an offset above }
                opcgsize:=tcgsize2unsigned[opcgsize];
              end;
          end;

        current_asmdata.getglobaldatalabel(table);
        { make it a 32bit register }
        indexreg:=cg.makeregsize(current_asmdata.CurrAsmList,hregister,OS_INT);
        cg.a_load_reg_reg(current_asmdata.CurrAsmList,opcgsize,OS_INT,hregister,indexreg);
        { create reference }
        reference_reset_symbol(href,table,0,sizeof(pint),[]);
        href.offset:=(-aint(min_))*sizeof(aint);
        href.index:=indexreg;
{$ifdef i8086}
        cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SHL,OS_INT,1,indexreg);
{$else i8086}
        href.scalefactor:=sizeof(aint);
{$endif i8086}

        if (not (target_info.system in [system_i386_darwin,system_i386_iphonesim])) and
           (cs_create_pic in current_settings.moduleswitches) then
          begin
            labeltyp:=aitconst_gotoff_symbol;
            jumpreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_ADDR);
            cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,href,jumpreg);
            cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_ADD,OS_ADDR,current_procinfo.got,jumpreg);
            emit_reg(A_JMP,S_NO,jumpreg);
            include(current_procinfo.flags,pi_needs_got);
          end
        else
          emit_ref(A_JMP,S_NO,href);
        { generate jump table }
        if (target_info.system in [system_i386_darwin,system_i386_iphonesim]) then
          jtlist:=current_asmdata.asmlists[al_const]
        else
          jtlist:=current_procinfo.aktlocaldata;
        new_section(jtlist,sec_rodata,current_procinfo.procdef.mangledname,sizeof(aint));
        jtlist.concat(Tai_label.Create(table));

        if AlmostExhaustive then
          begin
            { Fill the table with the values below _min }
            x := lv.svalue;
            while x < oldmin do
              begin
                jtlist.concat(Tai_const.Create_type_sym(labeltyp, elselabel));
                Inc(x);
              end;

            genitem(jtlist,hp);

            { Fill the table with the values above _max }
            { Subtracting one from hv and not adding 1 to max averts the risk of an overflow }
            x := max_;
            hv := hv - 1;
            while x <= hv.svalue do
              begin
                jtlist.concat(Tai_const.Create_type_sym(labeltyp, elselabel));
                Inc(x);
              end;
          end
        else
          genitem(jtlist,hp)
      end;


    procedure tx86casenode.genlinearlist(hp : pcaselabel);
      var
        first : boolean;
        lastrange : boolean;
        last : TConstExprInt;
        cond_lt,cond_le : tresflags;
        opcgsize: tcgsize;

        procedure genitem(t : pcaselabel);
          var
             range, gap: aint;
          begin
             if assigned(t^.less) then
               genitem(t^.less);
             { need we to test the first value }
             if first and (t^._low>get_min_value(left.resultdef)) then
               begin
                 cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opcgsize,jmp_lt,aint(t^._low.svalue),hregister,elselabel);
               end;
             if t^._low=t^._high then
               begin
                  if t^._low-last=0 then
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opcgsize, OC_EQ,0,hregister,blocklabel(t^.blockid))
                  else
                    begin
                      cg.a_op_const_reg(current_asmdata.CurrAsmList, OP_SUB, opcgsize, aint(t^._low.svalue-last.svalue), hregister);
                      cg.a_jmp_flags(current_asmdata.CurrAsmList,F_E,blocklabel(t^.blockid));
                    end;
                  last:=t^._low;
                  lastrange:=false;
               end
             else
               begin
                  range := aint(t^._high.svalue - t^._low.svalue);
                  { it begins with the smallest label, if the value }
                  { is even smaller then jump immediately to the    }
                  { ELSE-label                                }
                  if first then
                    begin
                       { have we to ajust the first value ? }
                       if (t^._low>get_min_value(left.resultdef)) or (get_min_value(left.resultdef)<>0) then
                         cg.a_op_const_reg(current_asmdata.CurrAsmList, OP_SUB, opcgsize, aint(t^._low.svalue), hregister);
                    end
                  else
                    begin
                      gap := aint(t^._low.svalue - last.svalue);
                      { if there is no unused label between the last and the }
                      { present label then the lower limit can be checked    }
                      { immediately. else check the range in between:       }

                      { we need to use A_SUB, if cond_lt uses the carry flags
                        because A_DEC does not set the correct flags, therefor
                        using a_op_const_reg(OP_SUB) is not possible }
                      if (gap = 1) and (cond_lt in [F_C,F_NC,F_A,F_AE,F_B,F_BE]) then
                        emit_const_reg(A_SUB, TCGSize2OpSize[opcgsize], gap, hregister)
                      else
                        cg.a_op_const_reg(current_asmdata.CurrAsmList, OP_SUB, opcgsize, gap, hregister);
                      { no jump necessary here if the new range starts at
                        at the value following the previous one           }
                      if (gap <> 1) or
                         (not lastrange) then
                        cg.a_jmp_flags(current_asmdata.CurrAsmList,cond_lt,elselabel);
                    end;
                  { we need to use A_SUB, if cond_le uses the carry flags
                    because A_DEC does not set the correct flags, therefor
                    using a_op_const_reg(OP_SUB) is not possible }
                  if (cond_le in [F_C,F_NC,F_A,F_AE,F_B,F_BE]) and (range = 1) then
                    emit_const_reg(A_SUB,TCGSize2OpSize[opcgsize], range, hregister)
                  else
                    cg.a_op_const_reg(current_asmdata.CurrAsmList, OP_SUB, opcgsize, range, hregister);

                  cg.a_jmp_flags(current_asmdata.CurrAsmList,cond_le,blocklabel(t^.blockid));
                  last:=t^._high;
                  lastrange:=true;
               end;
             first:=false;
             if assigned(t^.greater) then
               genitem(t^.greater);
          end;

        begin
           opcgsize:=def_cgsize(opsize);
           if with_sign then
             begin
                cond_lt:=F_L;
                cond_le:=F_LE;
             end
           else
              begin
                cond_lt:=F_B;
                cond_le:=F_BE;
             end;
           { do we need to generate cmps? }
{$ifdef i8086}
           if (with_sign and (min_label<0)) or (opcgsize in [OS_32, OS_S32]) then
{$else i8086}
           if (with_sign and (min_label<0)) then
{$endif i8086}
             genlinearcmplist(hp)
           else
             begin
                if (labelcnt>1) or not(cs_opt_level1 in current_settings.optimizerswitches) then
                  begin
                    last:=0;
                    lastrange:=false;
                    first:=true;
                    genitem(hp);
                  end
                else
                  begin
                    { If only one label exists, we can greatly simplify the checks to a simple comparison }
                    if hp^._low=hp^._high then
                      cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opcgsize, OC_EQ, tcgint(hp^._low.svalue), hregister, blocklabel(hp^.blockid))
                    else
                      begin
                        cg.a_op_const_reg(current_asmdata.CurrAsmList, OP_SUB, opcgsize, tcgint(hp^._low.svalue), hregister);
                        cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opcgsize, OC_BE, tcgint(hp^._high.svalue - hp^._low.svalue), hregister,blocklabel(hp^.blockid));
                      end;
                  end;
                cg.a_jmp_always(current_asmdata.CurrAsmList,elselabel);
             end;
        end;

      procedure tx86casenode.genjmptreeentry(p : pcaselabel;parentvalue : TConstExprInt);
        var
          lesslabel,greaterlabel : tasmlabel;
          cond_gt: TResFlags;
          cmplow : Boolean;
        begin
           if with_sign then
             cond_gt:=F_G
           else
             cond_gt:=F_A;
          current_asmdata.CurrAsmList.concat(cai_align.Create(current_settings.alignment.jumpalign));
          cg.a_label(current_asmdata.CurrAsmList,p^.labellabel);

          { calculate labels for left and right }
          if p^.less=nil then
            lesslabel:=elselabel
          else
            lesslabel:=p^.less^.labellabel;
          if p^.greater=nil then
            greaterlabel:=elselabel
          else
            greaterlabel:=p^.greater^.labellabel;

          { calculate labels for left and right }
          { no range label: }
          if p^._low=p^._high then
            begin
              if greaterlabel=lesslabel then
                hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize,OC_NE,p^._low,hregister,lesslabel)
              else
                begin
                  cmplow:=p^._low-1<>parentvalue;
                  if cmplow then
                    hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize,jmp_lt,p^._low,hregister,lesslabel);
                  if p^._high+1<>parentvalue then
                    begin
                      if cmplow then
                        hlcg.a_jmp_flags(current_asmdata.CurrAsmList,cond_gt,greaterlabel)
                      else
                        hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize,jmp_gt,p^._low,hregister,greaterlabel);
                    end;
                end;
              hlcg.a_jmp_always(current_asmdata.CurrAsmList,blocklabel(p^.blockid));
            end
          else
            begin
              if p^._low-1<>parentvalue then
                hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize,jmp_lt,p^._low,hregister,lesslabel);
              if p^._high+1<>parentvalue then
                hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize,jmp_gt,p^._high,hregister,greaterlabel);
              hlcg.a_jmp_always(current_asmdata.CurrAsmList,blocklabel(p^.blockid));
            end;
           if assigned(p^.less) then
             genjmptreeentry(p^.less,p^._low);
           if assigned(p^.greater) then
             genjmptreeentry(p^.greater,p^._high);
        end;


{*****************************************************************************
                              TX86INNODE
*****************************************************************************}

    function tx86innode.pass_1 : tnode;
      begin
         result:=nil;
         { this is the only difference from the generic version }
         expectloc:=LOC_FLAGS;

         firstpass(right);
         firstpass(left);
         if codegenerror then
           exit;
      end;

    procedure tx86innode.pass_generate_code;
       type
         Tsetpart=record
           range : boolean;      {Part is a range.}
           start,stop : byte;    {Start/stop when range; Stop=element when an element.}
         end;
       var
         hreg,{$ifndef i8086} hreg2, {$endif}
         pleftreg   : tregister;
         opsize     : tcgsize;
         opdef      : torddef;
         orgopsize  : tcgsize;
         setparts   : array[1..8] of Tsetpart;
         setbase    : aint;
         adjustment : longint;
         l,l2       : tasmlabel;
         i,numparts : byte;
         genjumps,
         use_small,
         ranges     : boolean;
{$ifdef CORRECT_SET_IN_FPC}
         AM         : tasmop;
{$endif CORRECT_SET_IN_FPC}
{$ifdef i8086}
         extra_offset_reg: TRegister;
{$endif i8086}

         function analizeset(Aset:pconstset;is_small:boolean):boolean;
           var
             compares,maxcompares:word;
             i:byte;
           begin
             if tnormalset(Aset^)=[] then
                {The expression...
                    if expr in []
                 ...is allways false. It should be optimized away in the
                 resultdef pass, and thus never occur here. Since we
                 do generate wrong code for it, do internalerror.}
                internalerror(2002072301);
             analizeset:=false;
             ranges:=false;
             numparts:=0;
             compares:=0;
             { Lots of comparisions take a lot of time, so do not allow
               too much comparisions. 8 comparisions are, however, still
               smalller than emitting the set }
             if cs_opt_size in current_settings.optimizerswitches then
               maxcompares:=8
             else
               maxcompares:=5;
             { when smallset is possible allow only 3 compares the smallset
               code is for littlesize also smaller when more compares are used }
             if is_small then
               maxcompares:=3;
             for i:=0 to 255 do
              if i in tnormalset(Aset^) then
               begin
                 if (numparts=0) or (i<>setparts[numparts].stop+1) then
                  begin
                  {Set element is a separate element.}
                    inc(compares);
                    if compares>maxcompares then
                         exit;
                    inc(numparts);
                    setparts[numparts].range:=false;
                    setparts[numparts].stop:=i;
                  end
                 else
                  {Set element is part of a range.}
                  if not setparts[numparts].range then
                   begin
                     {Transform an element into a range.}
                     setparts[numparts].range:=true;
                     setparts[numparts].start:=setparts[numparts].stop;
                     setparts[numparts].stop:=i;
                     ranges := true;
                   end
                  else
                   begin
                    {Extend a range.}
                    setparts[numparts].stop:=i;
                   end;
              end;
             analizeset:=true;
           end;

{$ifdef i8086}
         procedure add_extra_offset(offset_reg:TRegister;var ref:treference);
           var
             reg: TRegister;
           begin
             if ref.index=NR_NO then
               ref.index:=offset_reg
             else if ref.base=NR_NO then
               ref.base:=offset_reg
             else
               begin
                 reg:=cg.getaddressregister(current_asmdata.CurrAsmList);
                 cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,ref.index,reg);
                 cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_ADD,OS_ADDR,offset_reg,reg);
                 ref.index:=reg;
               end;
           end;
{$endif i8086}

       begin
         ranges:=false;
         numparts:=0;
         fillchar(setparts,sizeof(setparts),0);

         { We check first if we can generate jumps, this can be done
           because the resultdef is already set in firstpass }

         { check if we can use smallset operation using btl which is limited
           to 32 bits, the left side may also not contain higher values or be signed !! }
         use_small:=is_smallset(right.resultdef) and
                    not is_signed(left.resultdef) and
                    ((left.resultdef.typ=orddef) and (torddef(left.resultdef).high.svalue<{$ifdef i8086}16{$else}32{$endif}) or
                     (left.resultdef.typ=enumdef) and (tenumdef(left.resultdef).max<{$ifdef i8086}16{$else}32{$endif}));

         { Can we generate jumps? Possible for all types of sets }
         genjumps:=(right.nodetype=setconstn) and
                   analizeset(tsetconstnode(right).value_set,use_small);
         { calculate both operators }
         { the complex one first }
         { not in case of genjumps, because then we don't secondpass    }
         { right at all (so we have to make sure that "right" really is }
         { "right" and not "swapped left" in that case)                 }
         if not(genjumps) then
           firstcomplex(self);
         secondpass(left);
         { Only process the right if we are not generating jumps }
         if not genjumps then
          begin
            secondpass(right);
          end;
         if codegenerror then
          exit;

         { ofcourse not commutative }
         if nf_swapped in flags then
          swapleftright;

         orgopsize := def_cgsize(left.resultdef);
{$ifdef i8086}
         opsize := OS_16;
{$else i8086}
         opsize := OS_32;
{$endif i8086}
         if is_signed(left.resultdef) then
           opsize := tcgsize(ord(opsize)+(ord(OS_S8)-ord(OS_8)));
         opdef:=cgsize_orddef(opsize);

         if not(left.location.loc in [LOC_REGISTER,LOC_CREGISTER,LOC_REFERENCE,LOC_CREFERENCE,LOC_CONSTANT]) then
           hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,opdef,true);
         if (right.location.loc in [LOC_SUBSETREG,LOC_CSUBSETREG]) then
           hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,left.resultdef,opdef,true);

         if genjumps then
          begin
            { It gives us advantage to check for the set elements
              separately instead of using the SET_IN_BYTE procedure.
              To do: Build in support for LOC_JUMP }

            { load and zero or sign extend as necessary }
            hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,opdef,false);
            pleftreg:=left.location.register;

            { Get a label to jump to the end }
            location_reset(location,LOC_FLAGS,OS_NO);

            { It's better to use the zero flag when there are
              no ranges }
            if ranges then
              location.resflags:=F_C
            else
              location.resflags:=F_E;

            current_asmdata.getjumplabel(l);

            { how much have we already substracted from the x in the }
            { "x in [y..z]" expression                               }
            adjustment := 0;

            for i:=1 to numparts do
             if setparts[i].range then
              { use fact that a <= x <= b <=> cardinal(x-a) <= cardinal(b-a) }
              begin
                { is the range different from all legal values? }
                if (setparts[i].stop-setparts[i].start <> 255) or not (orgopsize = OS_8) then
                  begin
                    { yes, is the lower bound <> 0? }
                    if (setparts[i].start <> 0) then
                      begin
                        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,opdef,false);
                        hreg:=left.location.register;
                        pleftreg:=hreg;
                        cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SUB,opsize,setparts[i].start-adjustment,pleftreg);
                      end;

                    { new total value substracted from x:           }
                    { adjustment + (setparts[i].start - adjustment) }
                    adjustment := setparts[i].start;

                    { check if result < b-a+1 (not "result <= b-a", since }
                    { we need a carry in case the element is in the range }
                    { (this will never overflow since we check at the     }
                    { beginning whether stop-start <> 255)                }
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize,OC_B,setparts[i].stop-setparts[i].start+1,pleftreg,l);
                  end
                else
                  { if setparts[i].start = 0 and setparts[i].stop = 255,  }
                  { it's always true since "in" is only allowed for bytes }
                  begin
                    current_asmdata.CurrAsmList.concat(taicpu.op_none(A_STC,S_NO));
                    cg.a_jmp_always(current_asmdata.CurrAsmList,l);
                  end;
              end
             else
              begin
                { Emit code to check if left is an element }
                current_asmdata.CurrAsmList.concat(taicpu.op_const_reg(A_CMP,TCGSize2OpSize[opsize],setparts[i].stop-adjustment,
                  pleftreg));
                { Result should be in carry flag when ranges are used }
                if ranges then
                  current_asmdata.CurrAsmList.concat(taicpu.op_none(A_STC,S_NO));
                { If found, jump to end }
                cg.a_jmp_flags(current_asmdata.CurrAsmList,F_E,l);
              end;
             if ranges and
                { if the last one was a range, the carry flag is already }
                { set appropriately                                      }
                not(setparts[numparts].range) then
                  current_asmdata.CurrAsmList.concat(taicpu.op_none(A_CLC,S_NO));
             { To compensate for not doing a second pass }
             right.location.reference.symbol:=nil;
             { Now place the end label }
             cg.a_label(current_asmdata.CurrAsmList,l);
          end
         else
          begin
            location_reset(location,LOC_FLAGS,OS_NO);
            setbase:=tsetdef(right.resultdef).setbase;

            { We will now generated code to check the set itself, no jmps,
              handle smallsets separate, because it allows faster checks }
            if use_small then
             begin
               if left.location.loc=LOC_CONSTANT then
                begin
                  location.resflags:=F_NE;
                  case right.location.loc of
                    LOC_REGISTER,
                    LOC_CREGISTER:
                      begin
                         emit_const_reg(A_TEST,TCGSize2OpSize[right.location.size],
                           1 shl ((left.location.value-setbase) and 31),right.location.register);
                      end;
                    LOC_REFERENCE,
                    LOC_CREFERENCE :
                      begin
                        emit_const_ref(A_TEST,TCGSize2OpSize[right.location.size],1 shl ((left.location.value-setbase) and 31),
                           right.location.reference);
                      end;
                    else
                      internalerror(200203312);
                  end;
                end
               else
                begin
{$ifdef i8086}
                  register_maybe_adjust_setbase(current_asmdata.CurrAsmList,left.resultdef,left.location,setbase);
                  cg.getcpuregister(current_asmdata.CurrAsmList,NR_CX);
                  if TCGSize2Size[left.location.size] > 2 then
                    left.location.size := OS_16;
                  cg.a_load_loc_reg(current_asmdata.CurrAsmList,OS_16,left.location,NR_CX);

                  if (tcgsize2size[right.location.size] < 2) or
                     (right.location.loc = LOC_CONSTANT) then
                    hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,u16inttype,true);

                  hreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_16);
                  emit_const_reg(A_MOV,S_W,1,hreg);
                  emit_reg_reg(A_SHL,S_W,NR_CL,hreg);

                  case right.location.loc of
                    LOC_REGISTER,
                    LOC_CREGISTER :
                      begin
                        emit_reg_reg(A_TEST,S_W,hreg,right.location.register);
                      end;
                     LOC_CREFERENCE,
                     LOC_REFERENCE :
                       begin
                         emit_reg_ref(A_TEST,S_W,hreg,right.location.reference);
                       end;
                     else
                       internalerror(2002032210);
                  end;
                  cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_CX);
                  location.resflags:=F_NE;
{$else i8086}
                  hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,u32inttype,true);
                  register_maybe_adjust_setbase(current_asmdata.CurrAsmList,u32inttype,left.location,setbase);
                  if (tcgsize2size[right.location.size] < 4) or
                     (right.location.loc = LOC_CONSTANT) then
                    hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,u32inttype,true);
                  hreg:=left.location.register;

                  case right.location.loc of
                    LOC_REGISTER,
                    LOC_CREGISTER :
                      begin
                        emit_reg_reg(A_BT,S_L,hreg,right.location.register);
                      end;
                     LOC_CREFERENCE,
                     LOC_REFERENCE :
                       begin
                         emit_reg_ref(A_BT,S_L,hreg,right.location.reference);
                       end;
                     else
                       internalerror(2002032210);
                  end;
                  location.resflags:=F_C;
{$endif i8086}
                end;
             end
            else
             begin
               if right.location.loc=LOC_CONSTANT then
                begin
{$ifdef i8086}
                  location.resflags:=F_NE;
                  current_asmdata.getjumplabel(l);
                  current_asmdata.getjumplabel(l2);

                  { load constants to a register }
                  if (left.location.loc=LOC_CONSTANT) or
                     (setbase<>0) then
                    begin
                      hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,opdef,true);
                      register_maybe_adjust_setbase(current_asmdata.CurrAsmList,opdef,left.location,setbase);
                    end;

                  cg.getcpuregister(current_asmdata.CurrAsmList,NR_CX);
                  if TCGSize2Size[left.location.size] > 2 then
                    left.location.size := OS_16;
                  cg.a_load_loc_reg(current_asmdata.CurrAsmList,OS_16,left.location,NR_CX);
                  cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize,OC_BE,15,NR_CX,l);
                  { set the zero flag }
                  current_asmdata.CurrAsmList.concat(taicpu.op_const_reg(A_TEST,S_B,0,NR_AL));
                  cg.a_jmp_always(current_asmdata.CurrAsmList,l2);

                  hreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_16);
                  cg.a_label(current_asmdata.CurrAsmList,l);
                  emit_const_reg(A_MOV,S_W,1,hreg);
                  emit_reg_reg(A_SHL,S_W,NR_CL,hreg);
                  cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_CX);
                  emit_const_reg(A_TEST,S_W,right.location.value,hreg);

                  cg.a_label(current_asmdata.CurrAsmList,l2);
{$else i8086}
                  location.resflags:=F_C;
                  current_asmdata.getjumplabel(l);
                  current_asmdata.getjumplabel(l2);

                  { load constants to a register }
                  if (left.location.loc=LOC_CONSTANT) or
                     (setbase<>0) then
                    begin
                      hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,opdef,true);
                      register_maybe_adjust_setbase(current_asmdata.CurrAsmList,opdef,left.location,setbase);
                    end;

                  case left.location.loc of
                     LOC_REGISTER,
                     LOC_CREGISTER:
                       begin
                          hreg:=cg.makeregsize(current_asmdata.CurrAsmList,left.location.register,opsize);
                          cg.a_load_reg_reg(current_asmdata.CurrAsmList,left.location.size,opsize,left.location.register,hreg);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize,OC_BE,31,hreg,l);
                          { reset carry flag }
                          current_asmdata.CurrAsmList.concat(taicpu.op_none(A_CLC,S_NO));
                          cg.a_jmp_always(current_asmdata.CurrAsmList,l2);
                          cg.a_label(current_asmdata.CurrAsmList,l);
                          { We have to load the value into a register because
                            btl does not accept values only refs or regs (PFV) }
                          hreg2:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
                          cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_32,right.location.value,hreg2);
                          emit_reg_reg(A_BT,S_L,hreg,hreg2);
                       end;
                     else
                       begin
                          emit_const_ref(A_CMP,TCGSize2OpSize[orgopsize],31,left.location.reference);
                          cg.a_jmp_flags(current_asmdata.CurrAsmList,F_BE,l);
                          { reset carry flag }
                          current_asmdata.CurrAsmList.concat(taicpu.op_none(A_CLC,S_NO));
                          cg.a_jmp_always(current_asmdata.CurrAsmList,l2);
                          cg.a_label(current_asmdata.CurrAsmList,l);
                          hreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
                          cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_32,OS_32,left.location.reference,hreg);
                          { We have to load the value into a register because
                            btl does not accept values only refs or regs (PFV) }
                          hreg2:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
                          cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_32,right.location.value,hreg2);
                          emit_reg_reg(A_BT,S_L,hreg,hreg2);
                       end;
                  end;
                  cg.a_label(current_asmdata.CurrAsmList,l2);
{$endif i8086}
                end { of right.location.loc=LOC_CONSTANT }
               { do search in a normal set which could have >32 elementsm
                 but also used if the left side contains values > 32 or < 0 }
               else if left.location.loc=LOC_CONSTANT then
                begin
                  if (left.location.value<setbase) or (((left.location.value-setbase) shr 3) >= right.resultdef.size) then
                    {should be caught earlier }
                    internalerror(2007020201);

                  location.resflags:=F_NE;
                  case right.location.loc of
                    LOC_REFERENCE,LOC_CREFERENCE:
                      begin
                        inc(right.location.reference.offset,(left.location.value-setbase) shr 3);
                        emit_const_ref(A_TEST,S_B,1 shl ((left.location.value-setbase) and 7),right.location.reference);
                      end;
                    LOC_REGISTER,LOC_CREGISTER:
                      begin
                        emit_const_reg(A_TEST,TCGSize2OpSize[right.location.size],1 shl (left.location.value-setbase),right.location.register);
                      end;
                    else
                      internalerror(2007051901);
                  end;
                end
               else
                begin
{$ifdef i8086}
                  hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,opdef,false);
                  register_maybe_adjust_setbase(current_asmdata.CurrAsmList,opdef,left.location,setbase);

                  if TCGSize2Size[left.location.size] > 2 then
                    left.location.size := OS_16;

                  if not use_small then
                    begin
                      extra_offset_reg:=cg.getintregister(current_asmdata.CurrAsmList,OS_16);
                      cg.a_load_loc_reg(current_asmdata.CurrAsmList,OS_16,left.location,extra_offset_reg);
                      cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SHR,OS_16,4,extra_offset_reg);
                      cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SHL,OS_16,1,extra_offset_reg);
                    end
                  else
                    extra_offset_reg:=NR_NO;

                  cg.getcpuregister(current_asmdata.CurrAsmList,NR_CX);
                  cg.a_load_loc_reg(current_asmdata.CurrAsmList,OS_16,left.location,NR_CX);
                  if not use_small then
                    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg(A_AND,S_B,15,NR_CL));

                  pleftreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_16);

                  if (right.location.loc in [LOC_REGISTER,LOC_CREGISTER]) then
                    hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,opdef,true);

                  if (opsize >= OS_S8) or { = if signed }
                     ((left.resultdef.typ=orddef) and
                      ((torddef(left.resultdef).low < int64(tsetdef(right.resultdef).setbase)) or
                       (torddef(left.resultdef).high > int64(tsetdef(right.resultdef).setmax)))) or
                     ((left.resultdef.typ=enumdef) and
                      ((tenumdef(left.resultdef).min < aint(tsetdef(right.resultdef).setbase)) or
                       (tenumdef(left.resultdef).max > aint(tsetdef(right.resultdef).setmax)))) then
                   begin

                    { we have to check if the value is < 0 or > setmax }

                    current_asmdata.getjumplabel(l);
                    current_asmdata.getjumplabel(l2);

                    { BE will be false for negative values }
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize,OC_BE,tsetdef(right.resultdef).setmax-tsetdef(right.resultdef).setbase,pleftreg,l);
                    { set the zero flag }
                    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg(A_TEST,S_B,0,NR_AL));
                    cg.a_jmp_always(current_asmdata.CurrAsmList,l2);

                    cg.a_label(current_asmdata.CurrAsmList,l);

                    emit_const_reg(A_MOV,S_W,1,pleftreg);
                    emit_reg_reg(A_SHL,S_W,NR_CL,pleftreg);
                    cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_CX);
                    case right.location.loc of
                      LOC_REGISTER, LOC_CREGISTER :
                        emit_reg_reg(A_TEST,S_W,pleftreg,right.location.register);
                      LOC_CREFERENCE, LOC_REFERENCE :
                        begin
                          if not use_small then
                            add_extra_offset(extra_offset_reg,right.location.reference);
                          emit_reg_ref(A_TEST,S_W,pleftreg,right.location.reference);
                        end;
                    else
                      internalerror(2007020301);
                    end;

                    cg.a_label(current_asmdata.CurrAsmList,l2);

                    location.resflags:=F_NE;

                   end
                  else
                   begin
                      emit_const_reg(A_MOV,S_W,1,pleftreg);
                      emit_reg_reg(A_SHL,S_W,NR_CL,pleftreg);
                      cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_CX);
                      case right.location.loc of
                        LOC_REGISTER, LOC_CREGISTER :
                          emit_reg_reg(A_TEST,S_W,pleftreg,right.location.register);
                        LOC_CREFERENCE, LOC_REFERENCE :
                          begin
                            if not use_small then
                              add_extra_offset(extra_offset_reg,right.location.reference);
                            emit_reg_ref(A_TEST,S_W,pleftreg,right.location.reference);
                          end;
                      else
                        internalerror(2007020302);
                      end;
                      location.resflags:=F_NE;
                   end;
{$else i8086}
                  hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,opdef,false);
                  register_maybe_adjust_setbase(current_asmdata.CurrAsmList,opdef,left.location,setbase);
                  if (right.location.loc in [LOC_REGISTER,LOC_CREGISTER]) then
                    hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,opdef,true);
                  pleftreg:=left.location.register;

                  if (opsize >= OS_S8) or { = if signed }
                     ((left.resultdef.typ=orddef) and
                      ((torddef(left.resultdef).low < int64(tsetdef(right.resultdef).setbase)) or
                       (torddef(left.resultdef).high > int64(tsetdef(right.resultdef).setmax)))) or
                     ((left.resultdef.typ=enumdef) and
                      ((tenumdef(left.resultdef).min < aint(tsetdef(right.resultdef).setbase)) or
                       (tenumdef(left.resultdef).max > aint(tsetdef(right.resultdef).setmax)))) then
                   begin

                    { we have to check if the value is < 0 or > setmax }

                    current_asmdata.getjumplabel(l);
                    current_asmdata.getjumplabel(l2);

                    { BE will be false for negative values }
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize,OC_BE,tsetdef(right.resultdef).setmax-tsetdef(right.resultdef).setbase,pleftreg,l);
                    { reset carry flag }
                    current_asmdata.CurrAsmList.concat(taicpu.op_none(A_CLC,S_NO));
                    cg.a_jmp_always(current_asmdata.CurrAsmList,l2);

                    cg.a_label(current_asmdata.CurrAsmList,l);

                    pleftreg:=left.location.register;
                    case right.location.loc of
                      LOC_REGISTER, LOC_CREGISTER :
                        emit_reg_reg(A_BT,S_L,pleftreg,right.location.register);
                      LOC_CREFERENCE, LOC_REFERENCE :
                        emit_reg_ref(A_BT,S_L,pleftreg,right.location.reference);
                    else
                      internalerror(2007020301);
                    end;

                    cg.a_label(current_asmdata.CurrAsmList,l2);

                    location.resflags:=F_C;

                   end
                  else
                   begin
                      case right.location.loc of
                        LOC_REGISTER, LOC_CREGISTER :
                          emit_reg_reg(A_BT,S_L,pleftreg,right.location.register);
                        LOC_CREFERENCE, LOC_REFERENCE :
                          emit_reg_ref(A_BT,S_L,pleftreg,right.location.reference);
                      else
                        internalerror(2007020302);
                      end;
                      location.resflags:=F_C;
                   end;
{$endif i8086}
                end;
             end;
          end;
          if not genjumps then
            location_freetemp(current_asmdata.CurrAsmList,right.location);
       end;

begin
   cinnode:=tx86innode;
   ccasenode:=tx86casenode;
end.
