{
   Copyright (c) 1998-2002 by Florian Klaempfl

    Generate arm assembler for in set/case nodes

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
unit narmset;

{$i fpcdefs.inc}

interface

    uses
      globtype,constexp,
      symtype,
      cgbase,
      node,nset,pass_1,ncgset;

    type

       { tarminnode }

       tarminnode = class(tcginnode)
         function pass_1: tnode; override;
         procedure in_smallset(opdef: tdef; setbase: aint); override;
       end;

      tarmcasenode = class(tcgcasenode)
         procedure optimizevalues(var max_linear_list:int64;var max_dist:qword);override;
         function  has_jumptable : boolean;override;
         procedure genjumptable(hp : pcaselabel;min_,max_ : int64);override;
         procedure genlinearlist(hp : pcaselabel);override;
         procedure genjmptreeentry(p : pcaselabel;parentvalue : TConstExprInt);override;
      end;


implementation

    uses
      verbose,globals,defutil,systems,
      aasmbase,aasmtai,aasmdata,aasmcpu,
      cpubase,cpuinfo,
      cgutils,cgobj,ncgutil,
      cgcpu,hlcgobj;

{*****************************************************************************
                            TARMINNODE
*****************************************************************************}

    function tarminnode.pass_1: tnode;
      var
        setparts: Tsetparts;
        numparts: byte;
        use_small: boolean;
      begin
        result:=inherited pass_1;

        if not(assigned(result)) then
          begin
            if not(checkgenjumps(setparts,numparts,use_small)) and
              use_small and
              (target_info.endian=endian_little) then
              expectloc:=LOC_FLAGS;
          end;
      end;

    procedure tarminnode.in_smallset(opdef: tdef; setbase: aint);
      var
        so : tshifterop;
        hregister : tregister;
      begin
        { the code below needs changes for big endian targets (they start
          counting from the most significant bit)
        }
        if target_info.endian=endian_big then
          begin
            inherited;
            exit;
          end;
        location_reset(location,LOC_FLAGS,OS_NO);
        location.resflags:=F_NE;
        if (left.location.loc=LOC_CONSTANT) and not(GenerateThumbCode) then
          begin
            hlcg.location_force_reg(current_asmdata.CurrAsmList, right.location,
              right.resultdef, right.resultdef, true);

            cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_TST,right.location.register,1 shl (left.location.value-setbase)));
          end
        else
          begin
            hlcg.location_force_reg(current_asmdata.CurrAsmList, left.location,
             left.resultdef, opdef, true);
            register_maybe_adjust_setbase(current_asmdata.CurrAsmList, opdef,
             left.location, setbase);
            hlcg.location_force_reg(current_asmdata.CurrAsmList, right.location,
             right.resultdef, right.resultdef, true);

            hregister:=hlcg.getintregister(current_asmdata.CurrAsmList, opdef);
            hlcg.a_load_const_reg(current_asmdata.CurrAsmList,opdef,1,hregister);

            if GenerateThumbCode or GenerateThumb2Code then
              begin
                hlcg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_SHL,opdef,left.location.register,hregister);
                cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_TST,right.location.register,hregister));
              end
            else
              begin
                shifterop_reset(so);
                so.rs:=left.location.register;
                so.shiftmode:=SM_LSL;
                cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_shifterop(A_TST,right.location.register,hregister,so));
              end;
          end;
      end;


{*****************************************************************************
                            TARMCASENODE
*****************************************************************************}

    procedure tarmcasenode.optimizevalues(var max_linear_list:int64;var max_dist:qword);
      begin
        inc(max_linear_list,2)
      end;


    function tarmcasenode.has_jumptable : boolean;
      begin
        has_jumptable:=true;
      end;


    procedure tarmcasenode.genjumptable(hp : pcaselabel;min_,max_ : int64);
      var
        last : TConstExprInt;
        tmpreg,
        basereg,
        indexreg : tregister;
        href : treference;
        tablelabel, piclabel : TAsmLabel;
        opcgsize : tcgsize;
        picoffset : int64;

        procedure genitem(list:TAsmList;t : pcaselabel);
          var
            i : int64;
          begin
            if assigned(t^.less) then
              genitem(list,t^.less);
            { fill possible hole }
            i:=last+1;
            while i<=t^._low-1 do
              begin
                if cs_create_pic in current_settings.moduleswitches then
                  list.concat(Tai_const.Create_rel_sym_offset(aitconst_ptr,piclabel,elselabel,picoffset))
                else
                  list.concat(Tai_const.Create_sym(elselabel));
                i:=i+1;
              end;
            i:=t^._low;
            while i<=t^._high do
              begin
                if cs_create_pic in current_settings.moduleswitches then
                  list.concat(Tai_const.Create_rel_sym_offset(aitconst_ptr,piclabel,blocklabel(t^.blockid),picoffset))
                else
                  list.concat(Tai_const.Create_sym(blocklabel(t^.blockid)));
                i:=i+1;
              end;
            last:=t^._high;
            if assigned(t^.greater) then
              genitem(list,t^.greater);
          end;

        procedure genitem_thumb2(list:TAsmList;t : pcaselabel);
          var
            i : int64;
          begin
            if assigned(t^.less) then
              genitem_thumb2(list,t^.less);
            { fill possible hole }
            i:=last.svalue+1;
            while i<=t^._low.svalue-1 do
              begin
                list.concat(Tai_const.Create_rel_sym(aitconst_half16bit,tablelabel,elselabel));
                i:=i+1;
              end;
            i:=t^._low.svalue;
            while i<=t^._high.svalue do
              begin
                list.concat(Tai_const.Create_rel_sym(aitconst_half16bit,tablelabel,blocklabel(t^.blockid)));
                i:=i+1;
              end;
            last:=t^._high.svalue;
            if assigned(t^.greater) then
              genitem_thumb2(list,t^.greater);
          end;

      begin
        opcgsize:=def_cgsize(opsize);
        if not(jumptable_no_range) then
          begin
             { case expr less than min_ => goto elselabel }
             cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opcgsize,jmp_lt,aint(min_),hregister,elselabel);
             { case expr greater than max_ => goto elselabel }
             cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opcgsize,jmp_gt,aint(max_),hregister,elselabel);
          end;
        { make it a 32bit register }
        indexreg:=cg.makeregsize(current_asmdata.CurrAsmList,hregister,OS_INT);
        cg.a_load_reg_reg(current_asmdata.CurrAsmList,opcgsize,OS_INT,hregister,indexreg);

        if GenerateThumb2Code then
          begin
            if cs_create_pic in current_settings.moduleswitches then
              internalerror(2013082101);
            { adjust index }
            cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SUB,OS_ADDR,min_,indexreg,indexreg);
            { create reference and generate jump table }
            reference_reset(href,4,[]);
            href.base:=NR_PC;
            href.index:=indexreg;
            href.shiftmode:=SM_LSL;
            href.shiftimm:=1;
            current_asmdata.CurrAsmList.Concat(taicpu.op_ref(A_TBH,href));
            { generate jump table }
            current_asmdata.getjumplabel(tablelabel);
            cg.a_label(current_asmdata.CurrAsmList,tablelabel);
            last:=min_;
            genitem_thumb2(current_asmdata.CurrAsmList,hp);
          end
        else if GenerateThumbCode then
          begin
            if cs_create_pic in current_settings.moduleswitches then
              internalerror(2013082102);
            cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SUB,OS_ADDR,min_,indexreg,indexreg);
            current_asmdata.getaddrlabel(tablelabel);

            cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SHL,OS_ADDR,2,indexreg);

            basereg:=cg.getintregister(current_asmdata.CurrAsmList, OS_ADDR);
            reference_reset_symbol(href,tablelabel,0,4,[]);
            cg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList, href, basereg);

            reference_reset(href,0,[]);
            href.base:=basereg;
            href.index:=indexreg;

            tmpreg:=cg.getintregister(current_asmdata.CurrAsmList, OS_ADDR);
            cg.a_load_ref_reg(current_asmdata.CurrAsmList, OS_ADDR, OS_ADDR, href, tmpreg);

            { do not use BX here to avoid switching into arm mode }
            current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg(A_MOV, NR_PC, tmpreg));

            current_asmdata.CurrAsmList.Concat(tai_align.Create(4));
            cg.a_label(current_asmdata.CurrAsmList,tablelabel);
            { generate jump table }
            last:=min_;
            genitem(current_asmdata.CurrAsmList,hp);
          end
        else
          begin
            { adjust index }
            cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SUB,OS_ADDR,
              min_+ord(not(cs_create_pic in current_settings.moduleswitches)),
              indexreg,indexreg);
            { create reference and generate jump table }
            reference_reset(href,4,[]);
            href.base:=NR_PC;
            href.index:=indexreg;
            href.shiftmode:=SM_LSL;
            href.shiftimm:=2;
            if cs_create_pic in current_settings.moduleswitches then
              begin
                picoffset:=-8;
                current_asmdata.getaddrlabel(piclabel);
                indexreg:=cg.getaddressregister(current_asmdata.CurrAsmList);
                cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,href,indexreg);
                cg.a_label(current_asmdata.CurrAsmList,piclabel);
                cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_ADD,OS_ADDR,indexreg,NR_PC);
              end
            else
              cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,href,NR_PC);
            { generate jump table }
            last:=min_;
            genitem(current_asmdata.CurrAsmList,hp);
          end;
      end;


    procedure tarmcasenode.genlinearlist(hp : pcaselabel);
      var
        first : boolean;
        lastrange : boolean;
        last : TConstExprInt;
        cond_lt,cond_le : tresflags;
        opcgsize : tcgsize;

        procedure genitem(t : pcaselabel);
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
                      tbasecgarm(cg).cgsetflags:=true;
                      { use OS_32 here to avoid uncessary sign extensions, at this place hregister will never be negative, because
                        then genlinearlist wouldn't be used }
                      cg.a_op_const_reg(current_asmdata.CurrAsmList, OP_SUB, OS_32, aint(int64(t^._low-last)), hregister);
                      tbasecgarm(cg).cgsetflags:=false;
                      cg.a_jmp_flags(current_asmdata.CurrAsmList,F_EQ,blocklabel(t^.blockid));
                    end;
                  last:=t^._low;
                  lastrange:=false;
               end
             else
               begin
                  { it begins with the smallest label, if the value }
                  { is even smaller then jump immediately to the    }
                  { ELSE-label                                }
                  if first then
                    begin
                       { have we to ajust the first value ? }
                       if (t^._low>get_min_value(left.resultdef)) or (get_min_value(left.resultdef)<>0) then
                         begin
                           tbasecgarm(cg).cgsetflags:=true;
                           { use OS_32 here to avoid uncessary sign extensions, at this place hregister will never be negative, because
                             then genlinearlist wouldn't be use }
                           cg.a_op_const_reg(current_asmdata.CurrAsmList, OP_SUB, OS_32, aint(int64(t^._low)), hregister);
                           tbasecgarm(cg).cgsetflags:=false;
                         end;
                    end
                  else
                    begin
                      { if there is no unused label between the last and the }
                      { present label then the lower limit can be checked    }
                      { immediately. else check the range in between:       }

                      tbasecgarm(cg).cgsetflags:=true;
                      { use OS_32 here to avoid uncessary sign extensions, at this place hregister will never be negative, because
                        then genlinearlist wouldn't be use }
                      cg.a_op_const_reg(current_asmdata.CurrAsmList, OP_SUB, OS_32, aint(int64(t^._low-last)), hregister);
                      tbasecgarm(cg).cgsetflags:=false;
                      { no jump necessary here if the new range starts at }
                      { at the value following the previous one           }
                      if ((t^._low-last) <> 1) or
                         (not lastrange) then
                        cg.a_jmp_flags(current_asmdata.CurrAsmList,cond_lt,elselabel);
                    end;
                  tbasecgarm(cg).cgsetflags:=true;
                  { use OS_32 here to avoid uncessary sign extensions, at this place hregister will never be negative, because
                    then genlinearlist wouldn't be use }
                  cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SUB,OS_32,aint(int64(t^._high-t^._low)),hregister);
                  tbasecgarm(cg).cgsetflags:=false;
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
                cond_lt:=F_LT;
                cond_le:=F_LE;
             end
           else
              begin
                cond_lt:=F_CC;
                cond_le:=F_LS;
             end;
           { do we need to generate cmps? }
           if (with_sign and (min_label<0)) then
             genlinearcmplist(hp)
           else
             begin
                last:=0;
                lastrange:=false;
                first:=true;
                genitem(hp);
                cg.a_jmp_always(current_asmdata.CurrAsmList,elselabel);
             end;
        end;


      procedure tarmcasenode.genjmptreeentry(p : pcaselabel;parentvalue : TConstExprInt);
        var
          lesslabel,greaterlabel : tasmlabel;
          cond_gt: TResFlags;
          cmplow : Boolean;
        begin
           if with_sign then
             cond_gt:=F_GT
           else
             cond_gt:=F_HI;
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

begin
  cinnode:=tarminnode;
  ccasenode:=tarmcasenode;
end.
