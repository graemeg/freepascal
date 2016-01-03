{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Helper routines for all code generators

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
unit ncgutil;

{$i fpcdefs.inc}

interface

    uses
      node,cpuinfo,
      globtype,
      cpubase,cgbase,parabase,cgutils,
      aasmbase,aasmtai,aasmdata,aasmcpu,
      symconst,symbase,symdef,symsym,symtype,symtable
{$ifndef cpu64bitalu}
      ,cg64f32
{$endif not cpu64bitalu}
      ;

    type
      tloadregvars = (lr_dont_load_regvars, lr_load_regvars);

      pusedregvars = ^tusedregvars;
      tusedregvars = record
        intregvars, fpuregvars, mmregvars: Tsuperregisterworklist;
      end;

{
      Not used currently, implemented because I thought we had to
      synchronise around if/then/else as well, but not needed. May
      still be useful for SSA once we get around to implementing
      that (JM)

      pusedregvarscommon = ^tusedregvarscommon;
      tusedregvarscommon = record
        allregvars, commonregvars, myregvars: tusedregvars;
      end;
}

    procedure firstcomplex(p : tbinarynode);
    procedure maketojumpboollabels(list: TAsmList; p: tnode; truelabel, falselabel: tasmlabel);
//    procedure remove_non_regvars_from_loc(const t: tlocation; var regs:Tsuperregisterset);

    procedure location_force_mmreg(list:TAsmList;var l: tlocation;maybeconst:boolean);
<<<<<<< HEAD
<<<<<<< HEAD
    procedure location_allocate_register(list:TAsmList;out l: tlocation;def: tdef;constant: boolean);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    { load a tlocation into a cgpara }
    procedure gen_load_loc_cgpara(list: TAsmList; vardef: tdef; const l: tlocation; const cgpara: tcgpara);
>>>>>>> graemeg/cpstrnew
=======
    { load a tlocation into a cgpara }
    procedure gen_load_loc_cgpara(list: TAsmList; vardef: tdef; const l: tlocation; const cgpara: tcgpara);
>>>>>>> graemeg/cpstrnew
=======
    { load a tlocation into a cgpara }
    procedure gen_load_loc_cgpara(list: TAsmList; vardef: tdef; const l: tlocation; const cgpara: tcgpara);
>>>>>>> graemeg/cpstrnew
=======
    { load a tlocation into a cgpara }
    procedure gen_load_loc_cgpara(list: TAsmList; vardef: tdef; const l: tlocation; const cgpara: tcgpara);
>>>>>>> origin/cpstrnew
    { loads a cgpara into a tlocation; assumes that loc.loc is already
      initialised }
    procedure gen_load_cgpara_loc(list: TAsmList; vardef: tdef; const para: TCGPara; var destloc: tlocation; reusepara: boolean);

    { allocate registers for a tlocation; assumes that loc.loc is already
      set to LOC_CREGISTER/LOC_CFPUREGISTER/... }
    procedure gen_alloc_regloc(list:TAsmList;var loc: tlocation);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure register_maybe_adjust_setbase(list: TAsmList; opdef: tdef; var l: tlocation; setbase: aint);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure register_maybe_adjust_setbase(list: TAsmList; var l: tlocation; setbase: aint);
>>>>>>> graemeg/cpstrnew
=======
    procedure register_maybe_adjust_setbase(list: TAsmList; var l: tlocation; setbase: aint);
>>>>>>> graemeg/fixes_2_2
=======
    procedure register_maybe_adjust_setbase(list: TAsmList; var l: tlocation; setbase: aint);
>>>>>>> origin/fixes_2_2


    function  has_alias_name(pd:tprocdef;const s:string):boolean;
    procedure alloc_proc_symbol(pd: tprocdef);
    procedure gen_proc_entry_code(list:TAsmList);
    procedure gen_proc_exit_code(list:TAsmList);
    procedure gen_stack_check_size_para(list:TAsmList);
    procedure gen_stack_check_call(list:TAsmList);
    procedure gen_save_used_regs(list:TAsmList);
    procedure gen_restore_used_regs(list:TAsmList);
    procedure gen_load_para_value(list:TAsmList);
<<<<<<< HEAD
=======
    procedure gen_load_return_value(list:TAsmList);

    procedure gen_external_stub(list:TAsmList;pd:tprocdef;const externalname:string);
    procedure gen_intf_wrappers(list:TAsmList;st:TSymtable;nested:boolean);
    procedure gen_load_vmt_register(list:TAsmList;objdef:tobjectdef;selfloc:tlocation;var vmtreg:tregister);
>>>>>>> graemeg/cpstrnew

    procedure get_used_regvars(n: tnode; var rv: tusedregvars);
    { adds the regvars used in n and its children to rv.allregvars,
      those which were already in rv.allregvars to rv.commonregvars and
      uses rv.myregvars as scratch (so that two uses of the same regvar
      in a single tree to make it appear in commonregvars). Useful to
      find out which regvars are used in two different node trees
      e.g. in the "else" and "then" path, or in various case blocks }
//    procedure get_used_regvars_common(n: tnode; var rv: tusedregvarscommon);
    procedure gen_sync_regvars(list:TAsmList; var rv: tusedregvars);

   {  Allocate the buffers for exception management and setjmp environment.
      Return a pointer to these buffers, send them to the utility routine
      so they are registered, and then call setjmp.

      Then compare the result of setjmp with 0, and if not equal
      to zero, then jump to exceptlabel.

      Also store the result of setjmp to a temporary space by calling g_save_exception_reason

      It is to note that this routine may be called *after* the stackframe of a
      routine has been called, therefore on machines where the stack cannot
      be modified, all temps should be allocated on the heap instead of the
      stack. }
    type
      texceptiontemps=record
        jmpbuf,
        envbuf,
        reasonbuf  : treference;
      end;

    procedure get_exception_temps(list:TAsmList;var t:texceptiontemps);
    procedure unget_exception_temps(list:TAsmList;const t:texceptiontemps);
    procedure new_exception(list:TAsmList;const t:texceptiontemps;exceptlabel:tasmlabel);
    procedure free_exception(list:TAsmList;const t:texceptiontemps;a:aint;endexceptlabel:tasmlabel;onlyfree:boolean);

    procedure gen_alloc_symtable(list:TAsmList;pd:tprocdef;st:TSymtable);
    procedure gen_free_symtable(list:TAsmList;st:TSymtable);

    procedure location_free(list: TAsmList; const location : TLocation);

    function getprocalign : shortint;

<<<<<<< HEAD
    procedure gen_fpc_dummy(list : TAsmList);
    procedure gen_load_frame_for_exceptfilter(list : TAsmList);

    procedure gen_fpc_dummy(list : TAsmList);

    procedure InsertInterruptTable;

    procedure gen_fpc_dummy(list : TAsmList);

    procedure InsertInterruptTable;

    procedure gen_fpc_dummy(list : TAsmList);

    procedure InsertInterruptTable;

    procedure gen_fpc_dummy(list : TAsmList);

    procedure InsertInterruptTable;
=======
    procedure gen_pic_helpers(list : TAsmList);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

implementation

  uses
    version,
    cutils,cclasses,
    globals,systems,verbose,export,
    ppu,defutil,
    procinfo,paramgr,fmodule,
    regvars,dbgbase,
    pass_1,pass_2,
<<<<<<< HEAD
    nbas,ncon,nld,nmem,nutils,ngenutil,
    tgobj,cgobj,hlcgobj,hlcgcpu
{$ifdef llvm}
    { override create_hlcodegen from hlcgcpu }
    , hlcgllvm
{$endif}
=======
    nbas,ncon,nld,nmem,nutils,
    tgobj,cgobj,cgcpu
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
{$ifdef powerpc}
    , cpupi
{$endif}
{$ifdef powerpc64}
    , cpupi
{$endif}
{$ifdef SUPPORT_MMX}
    , cgx86
{$endif SUPPORT_MMX}
;


{*****************************************************************************
                                  Misc Helpers
*****************************************************************************}
{$if first_mm_imreg = 0}
  {$WARN 4044 OFF} { Comparison might be always false ... }
{$endif}

    procedure location_free(list: TAsmList; const location : TLocation);
      begin
        case location.loc of
          LOC_VOID:
            ;
          LOC_REGISTER,
          LOC_CREGISTER:
            begin
{$ifdef cpu64bitalu}
                { x86-64 system v abi:
                  structs with up to 16 bytes are returned in registers }
                if location.size in [OS_128,OS_S128] then
                  begin
                    if getsupreg(location.register)<first_int_imreg then
                      cg.ungetcpuregister(list,location.register);
                    if getsupreg(location.registerhi)<first_int_imreg then
                      cg.ungetcpuregister(list,location.registerhi);
                  end
{$else cpu64bitalu}
                if location.size in [OS_64,OS_S64] then
                  begin
                    if getsupreg(location.register64.reglo)<first_int_imreg then
                      cg.ungetcpuregister(list,location.register64.reglo);
                    if getsupreg(location.register64.reghi)<first_int_imreg then
                      cg.ungetcpuregister(list,location.register64.reghi);
                  end
{$endif cpu64bitalu}
                else
                  if getsupreg(location.register)<first_int_imreg then
                    cg.ungetcpuregister(list,location.register);
            end;
          LOC_FPUREGISTER,
          LOC_CFPUREGISTER:
            begin
              if getsupreg(location.register)<first_fpu_imreg then
                cg.ungetcpuregister(list,location.register);
            end;
          LOC_MMREGISTER,
          LOC_CMMREGISTER :
            begin
              if getsupreg(location.register)<first_mm_imreg then
                cg.ungetcpuregister(list,location.register);
            end;
          LOC_REFERENCE,
          LOC_CREFERENCE :
            begin
              if paramanager.use_fixed_stack then
                location_freetemp(list,location);
            end;
          else
            internalerror(2004110211);
        end;
      end;


    procedure firstcomplex(p : tbinarynode);
      var
        fcl, fcr: longint;
        ncl, ncr: longint;
      begin
         { always calculate boolean AND and OR from left to right }
         if (p.nodetype in [orn,andn]) and
            is_boolean(p.left.resultdef) then
           begin
             if nf_swapped in p.flags then
               internalerror(200709253);
           end
         else
           begin
             fcl:=node_resources_fpu(p.left);
             fcr:=node_resources_fpu(p.right);
             ncl:=node_complexity(p.left);
             ncr:=node_complexity(p.right);
             { We swap left and right if
                a) right needs more floating point registers than left, and
                   left needs more than 0 floating point registers (if it
                   doesn't need any, swapping won't change the floating
                   point register pressure)
                b) both left and right need an equal amount of floating
                   point registers or right needs no floating point registers,
                   and in addition right has a higher complexity than left
                   (+- needs more integer registers, but not necessarily)
             }
             if ((fcr>fcl) and
                 (fcl>0)) or
                (((fcr=fcl) or
                  (fcr=0)) and
                 (ncr>ncl)) then
               p.swapleftright
           end;
      end;


    procedure maketojumpboollabels(list: TAsmList; p: tnode; truelabel, falselabel: tasmlabel);
    {
      produces jumps to true respectively false labels using boolean expressions
    }
      var
        opsize : tcgsize;
        storepos : tfileposinfo;
        tmpreg : tregister;
      begin
         if nf_error in p.flags then
           exit;
         storepos:=current_filepos;
         current_filepos:=p.fileinfo;
         if is_boolean(p.resultdef) then
           begin
              if is_constboolnode(p) then
                begin
                   if Tordconstnode(p).value.uvalue<>0 then
                     cg.a_jmp_always(list,truelabel)
                   else
                     cg.a_jmp_always(list,falselabel)
                end
              else
                begin
                   opsize:=def_cgsize(p.resultdef);
                   case p.location.loc of
                     LOC_SUBSETREG,LOC_CSUBSETREG,
                     LOC_SUBSETREF,LOC_CSUBSETREF:
                       begin
                         tmpreg := cg.getintregister(list,OS_INT);
                         hlcg.a_load_loc_reg(list,p.resultdef,osuinttype,p.location,tmpreg);
                         cg.a_cmp_const_reg_label(list,OS_INT,OC_NE,0,tmpreg,truelabel);
                         cg.a_jmp_always(list,falselabel);
                       end;
                     LOC_CREGISTER,LOC_REGISTER,LOC_CREFERENCE,LOC_REFERENCE :
                       begin
{$ifdef cpu64bitalu}
                         if opsize in [OS_128,OS_S128] then
                           begin
                             hlcg.location_force_reg(list,p.location,p.resultdef,cgsize_orddef(opsize),true);
                             tmpreg:=cg.getintregister(list,OS_64);
                             cg.a_op_reg_reg_reg(list,OP_OR,OS_64,p.location.register128.reglo,p.location.register128.reghi,tmpreg);
                             location_reset(p.location,LOC_REGISTER,OS_64);
                             p.location.register:=tmpreg;
                             opsize:=OS_64;
                           end;
{$else cpu64bitalu}
                         if opsize in [OS_64,OS_S64] then
                           begin
                             hlcg.location_force_reg(list,p.location,p.resultdef,cgsize_orddef(opsize),true);
                             tmpreg:=cg.getintregister(list,OS_32);
                             cg.a_op_reg_reg_reg(list,OP_OR,OS_32,p.location.register64.reglo,p.location.register64.reghi,tmpreg);
                             location_reset(p.location,LOC_REGISTER,OS_32);
                             p.location.register:=tmpreg;
                             opsize:=OS_32;
                           end;
{$endif cpu64bitalu}
                         cg.a_cmp_const_loc_label(list,opsize,OC_NE,0,p.location,truelabel);
                         cg.a_jmp_always(list,falselabel);
                       end;
                     LOC_JUMP:
                       begin
                         if truelabel<>p.location.truelabel then
                           begin
                             cg.a_label(list,p.location.truelabel);
                             cg.a_jmp_always(list,truelabel);
                           end;
                         if falselabel<>p.location.falselabel then
                           begin
                             cg.a_label(list,p.location.falselabel);
                             cg.a_jmp_always(list,falselabel);
                           end;
                       end;
{$ifdef cpuflags}
                     LOC_FLAGS :
                       begin
                         cg.a_jmp_flags(list,p.location.resflags,truelabel);
                         cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
                         cg.a_jmp_always(list,falselabel);
                       end;
{$endif cpuflags}
                     else
                       begin
                         printnode(output,p);
                         internalerror(200308241);
                       end;
                   end;
                end;
              location_reset_jump(p.location,truelabel,falselabel);
           end
         else
           internalerror(200112305);
         current_filepos:=storepos;
      end;


        (*
        This code needs fixing. It is not safe to use rgint; on the m68000 it
        would be rgaddr.

    procedure remove_non_regvars_from_loc(const t: tlocation; var regs:Tsuperregisterset);
      begin
        case t.loc of
          LOC_REGISTER:
            begin
              { can't be a regvar, since it would be LOC_CREGISTER then }
              exclude(regs,getsupreg(t.register));
              if t.register64.reghi<>NR_NO then
                exclude(regs,getsupreg(t.register64.reghi));
            end;
          LOC_CREFERENCE,LOC_REFERENCE:
            begin
              if not(cs_opt_regvar in current_settings.optimizerswitches) or
                 (getsupreg(t.reference.base) in cg.rgint.usableregs) then
                exclude(regs,getsupreg(t.reference.base));
              if not(cs_opt_regvar in current_settings.optimizerswitches) or
                 (getsupreg(t.reference.index) in cg.rgint.usableregs) then
                exclude(regs,getsupreg(t.reference.index));
            end;
        end;
      end;
        *)


{*****************************************************************************
                            EXCEPTION MANAGEMENT
*****************************************************************************}

    procedure get_exception_temps(list:TAsmList;var t:texceptiontemps);
     begin
        tg.gethltemp(list,rec_exceptaddr,rec_exceptaddr.size,tt_persistent,t.envbuf);
        tg.gethltemp(list,rec_jmp_buf,rec_jmp_buf.size,tt_persistent,t.jmpbuf);
        tg.gethltemp(list,ossinttype,ossinttype.size,tt_persistent,t.reasonbuf);
      end;


    procedure unget_exception_temps(list:TAsmList;const t:texceptiontemps);
      begin
        tg.Ungettemp(list,t.jmpbuf);
        tg.ungettemp(list,t.envbuf);
        tg.ungettemp(list,t.reasonbuf);
      end;


    procedure new_exception(list:TAsmList;const t:texceptiontemps;exceptlabel:tasmlabel);
      var
        paraloc1, paraloc2, paraloc3, pushexceptres, setjmpres: tcgpara;
        pd: tprocdef;
        tmpresloc: tlocation;
      begin
        paraloc1.init;
        paraloc2.init;
        paraloc3.init;
<<<<<<< HEAD

        { fpc_pushexceptaddr(exceptionframetype, setjmp_buffer, exception_address_chain_entry) }
        pd:=search_system_proc('fpc_pushexceptaddr');
        paramanager.getintparaloc(current_asmdata.CurrAsmList,pd,1,paraloc1);
        paramanager.getintparaloc(current_asmdata.CurrAsmList,pd,2,paraloc2);
        paramanager.getintparaloc(current_asmdata.CurrAsmList,pd,3,paraloc3);
        if pd.is_pushleftright then
          begin
            { type of exceptionframe }
            hlcg.a_load_const_cgpara(list,paraloc1.def,1,paraloc1);
            { setjmp buffer }
            hlcg.a_loadaddr_ref_cgpara(list,rec_jmp_buf,t.jmpbuf,paraloc2);
            { exception address chain entry }
            hlcg.a_loadaddr_ref_cgpara(list,rec_exceptaddr,t.envbuf,paraloc3);
          end
        else
          begin
            hlcg.a_loadaddr_ref_cgpara(list,rec_exceptaddr,t.envbuf,paraloc3);
            hlcg.a_loadaddr_ref_cgpara(list,rec_jmp_buf,t.jmpbuf,paraloc2);
            hlcg.a_load_const_cgpara(list,paraloc1.def,1,paraloc1);
          end;
        paramanager.freecgpara(list,paraloc3);
        paramanager.freecgpara(list,paraloc2);
        paramanager.freecgpara(list,paraloc1);
        { perform the fpc_pushexceptaddr call }
        pushexceptres:=hlcg.g_call_system_proc(list,pd,[@paraloc1,@paraloc2,@paraloc3],nil);
=======
        paramanager.getintparaloc(pocall_default,1,paraloc1);
        paramanager.getintparaloc(pocall_default,2,paraloc2);
        paramanager.getintparaloc(pocall_default,3,paraloc3);
        cg.a_loadaddr_ref_cgpara(list,t.envbuf,paraloc3);
        cg.a_loadaddr_ref_cgpara(list,t.jmpbuf,paraloc2);
        { push type of exceptionframe }
        cg.a_load_const_cgpara(list,OS_S32,1,paraloc1);
        paramanager.freecgpara(list,paraloc3);
        paramanager.freecgpara(list,paraloc2);
        paramanager.freecgpara(list,paraloc1);
        cg.allocallcpuregisters(list);
        cg.a_call_name(list,'FPC_PUSHEXCEPTADDR',false);
        cg.deallocallcpuregisters(list);

        paramanager.getintparaloc(pocall_default,1,paraloc1);
        cg.a_load_reg_cgpara(list,OS_ADDR,NR_FUNCTION_RESULT_REG,paraloc1);
        paramanager.freecgpara(list,paraloc1);
        cg.allocallcpuregisters(list);
        cg.a_call_name(list,'FPC_SETJMP',false);
        cg.deallocallcpuregisters(list);
        cg.alloccpuregisters(list,R_INTREGISTER,[RS_FUNCTION_RESULT_REG]);

        cg.g_exception_reason_save(list, t.reasonbuf);
        cg.a_cmp_const_reg_label(list,OS_S32,OC_NE,0,cg.makeregsize(list,NR_FUNCTION_RESULT_REG,OS_S32),exceptlabel);
        cg.dealloccpuregisters(list,R_INTREGISTER,[RS_FUNCTION_RESULT_REG]);
>>>>>>> graemeg/cpstrnew
        paraloc1.done;
        paraloc2.done;
        paraloc3.done;

        { get the result }
        location_reset(tmpresloc,LOC_REGISTER,def_cgsize(pushexceptres.def));
        tmpresloc.register:=hlcg.getaddressregister(list,pushexceptres.def);
        hlcg.gen_load_cgpara_loc(list,pushexceptres.def,pushexceptres,tmpresloc,true);
        pushexceptres.resetiftemp;

        { fpc_setjmp(result_of_pushexceptaddr_call) }
        pd:=search_system_proc('fpc_setjmp');
        paramanager.getintparaloc(current_asmdata.CurrAsmList,pd,1,paraloc1);

<<<<<<< HEAD
        hlcg.a_load_reg_cgpara(list,pushexceptres.def,tmpresloc.register,paraloc1);
        paramanager.freecgpara(list,paraloc1);
        { perform the fpc_setjmp call }
        setjmpres:=hlcg.g_call_system_proc(list,pd,[@paraloc1],nil);
        paraloc1.done;
        location_reset(tmpresloc,LOC_REGISTER,def_cgsize(setjmpres.def));
        tmpresloc.register:=hlcg.getintregister(list,setjmpres.def);
        hlcg.gen_load_cgpara_loc(list,setjmpres.def,setjmpres,tmpresloc,true);
        hlcg.g_exception_reason_save(list,setjmpres.def,ossinttype,tmpresloc.register,t.reasonbuf);
        { if we get 0 here in the function result register, it means that we
          longjmp'd back here }
        hlcg.a_cmp_const_reg_label(list,setjmpres.def,OC_NE,0,tmpresloc.register,exceptlabel);
        setjmpres.resetiftemp;
=======
{$ifndef cpu64bitalu}
    { 32-bit version }
    procedure location_force_reg(list:TAsmList;var l:tlocation;dst_size:TCGSize;maybeconst:boolean);
      var
        hregister,
        hregisterhi : tregister;
        hreg64 : tregister64;
        hl : tasmlabel;
        oldloc : tlocation;
        const_location: boolean;
     begin
        oldloc:=l;
        if dst_size=OS_NO then
          internalerror(200309144);
        { handle transformations to 64bit separate }
        if dst_size in [OS_64,OS_S64] then
         begin
           if not (l.size in [OS_64,OS_S64]) then
            begin
              { load a smaller size to OS_64 }
              if l.loc=LOC_REGISTER then
               begin
                 hregister:=cg.makeregsize(list,l.register64.reglo,OS_32);
                 cg.a_load_reg_reg(list,l.size,OS_32,l.register64.reglo,hregister);
               end
              else
               hregister:=cg.getintregister(list,OS_INT);
              { load value in low register }
              case l.loc of
{$ifdef cpuflags}
                LOC_FLAGS :
                  cg.g_flags2reg(list,OS_INT,l.resflags,hregister);
{$endif cpuflags}
                LOC_JUMP :
                  begin
                    cg.a_label(list,current_procinfo.CurrTrueLabel);
                    cg.a_load_const_reg(list,OS_INT,1,hregister);
                    current_asmdata.getjumplabel(hl);
                    cg.a_jmp_always(list,hl);
                    cg.a_label(list,current_procinfo.CurrFalseLabel);
                    cg.a_load_const_reg(list,OS_INT,0,hregister);
                    cg.a_label(list,hl);
                  end;
                else
                  cg.a_load_loc_reg(list,OS_INT,l,hregister);
              end;
              { reset hi part, take care of the signed bit of the current value }
              hregisterhi:=cg.getintregister(list,OS_INT);
              if (l.size in [OS_S8,OS_S16,OS_S32]) then
               begin
                 if l.loc=LOC_CONSTANT then
                  begin
                    if (longint(l.value)<0) then
                     cg.a_load_const_reg(list,OS_32,aint($ffffffff),hregisterhi)
                    else
                     cg.a_load_const_reg(list,OS_32,0,hregisterhi);
                  end
                 else
                  begin
                    cg.a_op_const_reg_reg(list,OP_SAR,OS_32,31,hregister,
                      hregisterhi);
                  end;
               end
              else
               cg.a_load_const_reg(list,OS_32,0,hregisterhi);
              location_reset(l,LOC_REGISTER,dst_size);
              l.register64.reglo:=hregister;
              l.register64.reghi:=hregisterhi;
            end
           else
            begin
              { 64bit to 64bit }
              if ((l.loc=LOC_CREGISTER) and maybeconst) then
               begin
                 hregister:=l.register64.reglo;
                 hregisterhi:=l.register64.reghi;
                 const_location := true;
               end
              else
               begin
                 hregister:=cg.getintregister(list,OS_INT);
                 hregisterhi:=cg.getintregister(list,OS_INT);
                 const_location := false;
               end;
              hreg64.reglo:=hregister;
              hreg64.reghi:=hregisterhi;
              { load value in new register }
              cg64.a_load64_loc_reg(list,l,hreg64);
              if not const_location then
                location_reset(l,LOC_REGISTER,dst_size)
              else
                location_reset(l,LOC_CREGISTER,dst_size);
              l.register64.reglo:=hregister;
              l.register64.reghi:=hregisterhi;
            end;
         end
        else
         begin
           {Do not bother to recycle the existing register. The register
            allocator eliminates unnecessary moves, so it's not needed
            and trying to recycle registers can cause problems because
            the registers changes size and may need aditional constraints.

            Not if it's about LOC_CREGISTER's (JM)
            }
           const_location :=
              (maybeconst) and
              (l.loc = LOC_CREGISTER) and
              (TCGSize2Size[l.size] = TCGSize2Size[dst_size]) and
              ((l.size = dst_size) or
               (TCGSize2Size[l.size] = sizeof(aint)));
           if not const_location then
             hregister:=cg.getintregister(list,dst_size)
           else
             hregister := l.register;
           { load value in new register }
           case l.loc of
{$ifdef cpuflags}
             LOC_FLAGS :
               cg.g_flags2reg(list,dst_size,l.resflags,hregister);
{$endif cpuflags}
             LOC_JUMP :
               begin
                 cg.a_label(list,current_procinfo.CurrTrueLabel);
                 cg.a_load_const_reg(list,dst_size,1,hregister);
                 current_asmdata.getjumplabel(hl);
                 cg.a_jmp_always(list,hl);
                 cg.a_label(list,current_procinfo.CurrFalseLabel);
                 cg.a_load_const_reg(list,dst_size,0,hregister);
                 cg.a_label(list,hl);
               end;
             else
               begin
                 { load_loc_reg can only handle size >= l.size, when the
                   new size is smaller then we need to adjust the size
                   of the orignal and maybe recalculate l.register for i386 }
                 if (TCGSize2Size[dst_size]<TCGSize2Size[l.size]) then
                  begin
                    if (l.loc in [LOC_REGISTER,LOC_CREGISTER]) then
                      l.register:=cg.makeregsize(list,l.register,dst_size);
                    { for big endian systems, the reference's offset must }
                    { be increased in this case, since they have the      }
                    { MSB first in memory and e.g. byte(word_var) should  }
                    { return  the second byte in this case (JM)           }
                    if (target_info.endian = ENDIAN_BIG) and
                       (l.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
                      begin
                        inc(l.reference.offset,TCGSize2Size[l.size]-TCGSize2Size[dst_size]);
                        l.reference.alignment:=newalignment(l.reference.alignment,TCGSize2Size[l.size]-TCGSize2Size[dst_size]);
                      end;
{$ifdef x86}
                  if not (l.loc in [LOC_SUBSETREG,LOC_CSUBSETREG]) then
                     l.size:=dst_size;
{$endif x86}
                  end;
                 cg.a_load_loc_reg(list,dst_size,l,hregister);
                 if (TCGSize2Size[dst_size]<TCGSize2Size[l.size])
{$ifdef x86}
                    and (l.loc in [LOC_SUBSETREG,LOC_CSUBSETREG])
{$endif x86}
                   then
                     l.size:=dst_size;
               end;
           end;
           if not const_location then
             location_reset(l,LOC_REGISTER,dst_size)
           else
             location_reset(l,LOC_CREGISTER,dst_size);
           l.register:=hregister;
         end;
       { Release temp when it was a reference }
       if oldloc.loc=LOC_REFERENCE then
         location_freetemp(list,oldloc);
>>>>>>> graemeg/cpstrnew
     end;


    procedure free_exception(list:TAsmList;const t:texceptiontemps;a:aint;endexceptlabel:tasmlabel;onlyfree:boolean);
      var
        reasonreg: tregister;
      begin
         hlcg.g_call_system_proc(list,'fpc_popaddrstack',[],nil);
         if not onlyfree then
          begin
            reasonreg:=hlcg.getintregister(list,osuinttype);
            hlcg.g_exception_reason_load(list,osuinttype,osuinttype,t.reasonbuf,reasonreg);
            hlcg.a_cmp_const_reg_label(list,osuinttype,OC_EQ,a,reasonreg,endexceptlabel);
          end;
      end;


<<<<<<< HEAD
{*****************************************************************************
                                     TLocation
*****************************************************************************}


    procedure register_maybe_adjust_setbase(list: TAsmList; opdef: tdef; var l: tlocation; setbase: aint);
=======
    procedure location_force_mmregscalar(list:TAsmList;var l: tlocation;maybeconst:boolean);
      var
        reg : tregister;
        href : treference;
        newsize : tcgsize;
      begin
        if (l.loc<>LOC_MMREGISTER)  and
           ((l.loc<>LOC_CMMREGISTER) or (not maybeconst)) then
          begin
            { if it's in an fpu register, store to memory first }
            if (l.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER]) then
              begin
                tg.GetTemp(list,tcgsize2size[l.size],tcgsize2size[l.size],tt_normal,href);
                cg.a_loadfpu_reg_ref(list,l.size,l.size,l.register,href);
                location_reset_ref(l,LOC_REFERENCE,l.size,0);
                l.reference:=href;
              end;
{$ifndef cpu64bitalu}
            if (l.loc in [LOC_REGISTER,LOC_CREGISTER]) and
               (l.size in [OS_64,OS_S64]) then
              begin
                reg:=cg.getmmregister(list,OS_F64);
                cg64.a_loadmm_intreg64_reg(list,OS_F64,l.register64,reg);
                l.size:=OS_F64
              end
            else
{$endif not cpu64bitalu}
              begin
                 { on ARM, CFP values may be located in integer registers,
                   and its second_int_to_real() also uses this routine to
                   force integer (memory) values in an mmregister }
                 if (l.size in [OS_32,OS_S32]) then
                   newsize:=OS_F32
                 else if (l.size in [OS_64,OS_S64]) then
                   newsize:=OS_F64
                 else
                   newsize:=l.size;
                 reg:=cg.getmmregister(list,newsize);
                 cg.a_loadmm_loc_reg(list,newsize,l,reg,mms_movescalar);
                 l.size:=newsize;
               end;
            location_freetemp(list,l);
            location_reset(l,LOC_MMREGISTER,l.size);
            l.register:=reg;
          end;
      end;


<<<<<<< HEAD
<<<<<<< HEAD
    procedure gen_loadfpu_loc_cgpara(list: TAsmList; const l: tlocation;const cgpara: tcgpara;locintsize: longint);
    var
{$ifdef i386}
        href   : treference;
        size   : longint;
{$endif i386}
        locsize : tcgsize;
        tmploc : tlocation;
      begin
           if not(l.size in [OS_32,OS_S32,OS_64,OS_S64,OS_128,OS_S128]) then
             locsize:=l.size
           else
             locsize:=int_float_cgsize(tcgsize2size[l.size]);
{$ifdef i386}
           case l.loc of
             LOC_FPUREGISTER,
             LOC_CFPUREGISTER:
               begin
                 case cgpara.location^.loc of
                   LOC_REFERENCE:
                     begin
                       size:=align(locintsize,cgpara.alignment);
                       if (not paramanager.use_fixed_stack) and
                          (cgpara.location^.reference.index=NR_STACK_POINTER_REG) then
                         begin
                           cg.g_stackpointer_alloc(list,size);
                           reference_reset_base(href,NR_STACK_POINTER_REG,0,sizeof(pint));
                         end
                       else
                         reference_reset_base(href,cgpara.location^.reference.index,cgpara.location^.reference.offset,cgpara.alignment);
                       cg.a_loadfpu_reg_ref(list,locsize,locsize,l.register,href);
                     end;
                   LOC_FPUREGISTER:
                     begin
                       cg.a_loadfpu_reg_reg(list,locsize,cgpara.location^.size,l.register,cgpara.location^.register);
                     end;
                   { can happen if a record with only 1 "single field" is
                     returned in a floating point register and then is directly
                     passed to a regcall parameter }
                   LOC_REGISTER:
                     begin
                       tmploc:=l;
                       location_force_mem(list,tmploc);
                       case locsize of
                         OS_F32:
                           tmploc.size:=OS_32;
                         OS_F64:
                           tmploc.size:=OS_64;
                         else
                           internalerror(2010053116);
                       end;
                       cg.a_load_loc_cgpara(list,tmploc,cgpara);
                       location_freetemp(list,tmploc);
                     end
                   else
                     internalerror(2010053003);
                 end;
               end;
             LOC_MMREGISTER,
             LOC_CMMREGISTER:
               begin
                 case cgpara.location^.loc of
                   LOC_REFERENCE:
                     begin
                       { can't use TCGSize2Size[l.size], because the size of an
                         80 bit extended parameter can be either 10 or 12 bytes }
                       size:=align(locintsize,cgpara.alignment);
                       if (not paramanager.use_fixed_stack) and
                          (cgpara.location^.reference.index=NR_STACK_POINTER_REG) then
                         begin
                           cg.g_stackpointer_alloc(list,size);
                           reference_reset_base(href,NR_STACK_POINTER_REG,0,sizeof(pint));
                         end
                       else
                         reference_reset_base(href,cgpara.location^.reference.index,cgpara.location^.reference.offset,cgpara.alignment);
                       cg.a_loadmm_reg_ref(list,locsize,locsize,l.register,href,mms_movescalar);
                     end;
                   LOC_FPUREGISTER:
                     begin
                       tmploc:=l;
                       location_force_mem(list,tmploc);
                       cg.a_loadfpu_ref_cgpara(list,tmploc.size,tmploc.reference,cgpara);
                       location_freetemp(list,tmploc);
                     end;
                   else
                     internalerror(2010053004);
                 end;
               end;
             LOC_REFERENCE,
             LOC_CREFERENCE :
               begin
                 case cgpara.location^.loc of
                   LOC_REFERENCE:
                     begin
                       size:=align(locintsize,cgpara.alignment);
                       if (not paramanager.use_fixed_stack) and
                          (cgpara.location^.reference.index=NR_STACK_POINTER_REG) then
                         cg.a_load_ref_cgpara(list,locsize,l.reference,cgpara)
                       else
                         begin
                           reference_reset_base(href,cgpara.location^.reference.index,cgpara.location^.reference.offset,cgpara.alignment);
                           cg.g_concatcopy(list,l.reference,href,size);
                         end;
                     end;
                   LOC_FPUREGISTER:
                     begin
                       cg.a_loadfpu_ref_cgpara(list,locsize,l.reference,cgpara);
                     end;
                   else
                     internalerror(2010053005);
                 end;
               end;
             else
               internalerror(2002042430);
           end;
{$else i386}
           case l.loc of
             LOC_MMREGISTER,
             LOC_CMMREGISTER:
               case cgpara.location^.loc of
                 LOC_REFERENCE,
                 LOC_CREFERENCE,
                 LOC_MMREGISTER,
                 LOC_CMMREGISTER,
                 LOC_REGISTER,
                 LOC_CREGISTER :
                   cg.a_loadmm_reg_cgpara(list,locsize,l.register,cgpara,mms_movescalar);
                 LOC_FPUREGISTER,
                 LOC_CFPUREGISTER:
                   begin
                     tmploc:=l;
                     location_force_fpureg(list,tmploc,false);
                     cg.a_loadfpu_reg_cgpara(list,tmploc.size,tmploc.register,cgpara);
                   end;
                 else
                   internalerror(200204249);
               end;
             LOC_FPUREGISTER,
             LOC_CFPUREGISTER:
               case cgpara.location^.loc of
                 LOC_MMREGISTER,
                 LOC_CMMREGISTER:
                   begin
                     tmploc:=l;
                     location_force_mmregscalar(list,tmploc,false);
                     cg.a_loadmm_reg_cgpara(list,tmploc.size,tmploc.register,cgpara,mms_movescalar);
                   end;
                 { Some targets pass floats in normal registers }
                 LOC_REGISTER,
                 LOC_CREGISTER,
                 LOC_REFERENCE,
                 LOC_CREFERENCE,
                 LOC_FPUREGISTER,
                 LOC_CFPUREGISTER:
                   cg.a_loadfpu_reg_cgpara(list,locsize,l.register,cgpara);
                 else
                   internalerror(2002042433);
               end;
             LOC_REFERENCE,
             LOC_CREFERENCE:
               case cgpara.location^.loc of
                 LOC_MMREGISTER,
                 LOC_CMMREGISTER:
                   cg.a_loadmm_ref_cgpara(list,locsize,l.reference,cgpara,mms_movescalar);
                 { Some targets pass floats in normal registers }
                 LOC_REGISTER,
                 LOC_CREGISTER,
                 LOC_REFERENCE,
                 LOC_CREFERENCE,
                 LOC_FPUREGISTER,
                 LOC_CFPUREGISTER:
                   cg.a_loadfpu_ref_cgpara(list,locsize,l.reference,cgpara);
                 else
                   internalerror(2002042431);
               end;
             LOC_REGISTER,
             LOC_CREGISTER :
               begin
{$ifndef cpu64bitalu}
                  { Only a_load_ref_cgpara supports multiple locations, when the
                    value is still a const or in a register then write it
                    to a reference first. This situation can be triggered
                    by typecasting an int64 constant to a record of 8 bytes }
                  if locsize = OS_F64 then
                    begin
                      tmploc:=l;
                      location_force_mem(list,tmploc);
                      cg.a_load_loc_cgpara(list,tmploc,cgpara);
                      location_freetemp(list,tmploc);
                    end
                  else
{$endif not cpu64bitalu}
                    cg.a_load_loc_cgpara(list,l,cgpara);
               end;
             else
               internalerror(2002042432);
           end;
{$endif i386}
      end;


    procedure gen_load_loc_cgpara(list: TAsmList; vardef: tdef; const l: tlocation; const cgpara: tcgpara);
{$ifndef cpu64bitalu}
      var
        tmploc: tlocation;
{$endif not cpu64bitalu}
      begin
        { Handle Floating point types differently

          This doesn't depend on emulator settings, emulator settings should
          be handled by cpupara }
        if (vardef.typ=floatdef) or
           { some ABIs return certain records in an fpu register }
           (l.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER]) or
           (assigned(cgpara.location) and
            (cgpara.Location^.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER])) then
          begin
            gen_loadfpu_loc_cgpara(list,l,cgpara,vardef.size);
            exit;
          end;

        case l.loc of
          LOC_CONSTANT,
          LOC_REGISTER,
          LOC_CREGISTER,
          LOC_REFERENCE,
          LOC_CREFERENCE :
            begin
{$ifndef cpu64bitalu}
              { use cg64 only for int64, not for 8 byte records }
              if is_64bit(vardef) then
                cg64.a_load64_loc_cgpara(list,l,cgpara)
              else
{$endif not cpu64bitalu}
                begin
{$ifndef cpu64bitalu}
                  { Only a_load_ref_cgpara supports multiple locations, when the
                    value is still a const or in a register then write it
                    to a reference first. This situation can be triggered
                    by typecasting an int64 constant to a record of 8 bytes }
                  if l.size in [OS_64,OS_S64] then
                    begin
                      tmploc:=l;
                      location_force_mem(list,tmploc);
                      cg.a_load_loc_cgpara(list,tmploc,cgpara);
                      location_freetemp(list,tmploc);
                    end
                  else
{$endif not cpu64bitalu}
                    cg.a_load_loc_cgpara(list,l,cgpara);
                end;
            end;
          LOC_MMREGISTER,
          LOC_CMMREGISTER:
            begin
              case l.size of
                OS_F32,
                OS_F64:
                  cg.a_loadmm_loc_cgpara(list,l,cgpara,mms_movescalar);
                else
                  cg.a_loadmm_loc_cgpara(list,l,cgpara,nil);
              end;
            end;
{$ifdef SUPPORT_MMX}
          LOC_MMXREGISTER,
          LOC_CMMXREGISTER:
            cg.a_loadmm_reg_cgpara(list,OS_M64,l.register,cgpara,nil);
{$endif SUPPORT_MMX}
          else
            internalerror(200204241);
        end;
      end;


    procedure register_maybe_adjust_setbase(list: TAsmList; var l: tlocation; setbase: aint);
>>>>>>> graemeg/cpstrnew
=======
    procedure register_maybe_adjust_setbase(list: TAsmList; var l: tlocation; setbase: aint);
>>>>>>> graemeg/fixes_2_2
=======
    procedure register_maybe_adjust_setbase(list: TAsmList; var l: tlocation; setbase: aint);
>>>>>>> origin/fixes_2_2
      var
        tmpreg: tregister;
      begin
        if (setbase<>0) then
          begin
            if not(l.loc in [LOC_REGISTER,LOC_CREGISTER]) then
              internalerror(2007091502);
            { subtract the setbase }
            case l.loc of
              LOC_CREGISTER:
                begin
<<<<<<< HEAD
<<<<<<< HEAD
                  tmpreg := hlcg.getintregister(list,opdef);
                  hlcg.a_op_const_reg_reg(list,OP_SUB,opdef,setbase,l.register,tmpreg);
=======
                  tmpreg := cg.getintregister(list,l.size);
                  cg.a_op_const_reg_reg(list,OP_SUB,l.size,setbase,l.register,tmpreg);
>>>>>>> graemeg/fixes_2_2
=======
                  tmpreg := cg.getintregister(list,l.size);
                  cg.a_op_const_reg_reg(list,OP_SUB,l.size,setbase,l.register,tmpreg);
>>>>>>> origin/fixes_2_2
                  l.loc:=LOC_REGISTER;
                  l.register:=tmpreg;
                end;
              LOC_REGISTER:
                begin
<<<<<<< HEAD
<<<<<<< HEAD
                  hlcg.a_op_const_reg(list,OP_SUB,opdef,setbase,l.register);
=======
                  cg.a_op_const_reg(list,OP_SUB,l.size,setbase,l.register);
>>>>>>> graemeg/fixes_2_2
=======
                  cg.a_op_const_reg(list,OP_SUB,l.size,setbase,l.register);
>>>>>>> origin/fixes_2_2
                end;
            end;
          end;
      end;


    procedure location_force_mmreg(list:TAsmList;var l: tlocation;maybeconst:boolean);
      var
        reg : tregister;
      begin
        if (l.loc<>LOC_MMREGISTER)  and
           ((l.loc<>LOC_CMMREGISTER) or (not maybeconst)) then
          begin
            reg:=cg.getmmregister(list,OS_VECTOR);
            cg.a_loadmm_loc_reg(list,OS_VECTOR,l,reg,nil);
            location_freetemp(list,l);
            location_reset(l,LOC_MMREGISTER,OS_VECTOR);
            l.register:=reg;
          end;
      end;


    procedure location_allocate_register(list: TAsmList;out l: tlocation;def: tdef;constant: boolean);
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
      begin
        l.size:=def_cgsize(def);
        if (def.typ=floatdef) and
           not(cs_fp_emulation in current_settings.moduleswitches) then
          begin
            if use_vectorfpu(def) then
              begin
                if constant then
                  location_reset(l,LOC_CMMREGISTER,l.size)
                else
                  location_reset(l,LOC_MMREGISTER,l.size);
                l.register:=cg.getmmregister(list,l.size);
              end
            else
              begin
                if constant then
                  location_reset(l,LOC_CFPUREGISTER,l.size)
                else
                  location_reset(l,LOC_FPUREGISTER,l.size);
                l.register:=cg.getfpuregister(list,l.size);
              end;
          end
        else
          begin
            if constant then
              location_reset(l,LOC_CREGISTER,l.size)
            else
              location_reset(l,LOC_REGISTER,l.size);
{$ifndef cpu64bitalu}
            if l.size in [OS_64,OS_S64,OS_F64] then
              begin
                l.register64.reglo:=cg.getintregister(list,OS_32);
                l.register64.reghi:=cg.getintregister(list,OS_32);
              end
            else
{$endif not cpu64bitalu}
              l.register:=cg.getintregister(list,l.size);
          end;
      end;


    procedure location_force_mem(list:TAsmList;var l:tlocation);
      var
        r : treference;
      begin
        case l.loc of
          LOC_FPUREGISTER,
          LOC_CFPUREGISTER :
            begin
              tg.GetTemp(list,TCGSize2Size[l.size],TCGSize2Size[l.size],tt_normal,r);
              cg.a_loadfpu_reg_ref(list,l.size,l.size,l.register,r);
              location_reset_ref(l,LOC_REFERENCE,l.size,0);
              l.reference:=r;
            end;
          LOC_MMREGISTER,
          LOC_CMMREGISTER:
            begin
              tg.GetTemp(list,TCGSize2Size[l.size],TCGSize2Size[l.size],tt_normal,r);
              cg.a_loadmm_reg_ref(list,l.size,l.size,l.register,r,mms_movescalar);
              location_reset_ref(l,LOC_REFERENCE,l.size,0);
              l.reference:=r;
            end;
          LOC_CONSTANT,
          LOC_REGISTER,
          LOC_CREGISTER :
            begin
              tg.GetTemp(list,TCGSize2Size[l.size],TCGSize2Size[l.size],tt_normal,r);
{$ifndef cpu64bitalu}
              if l.size in [OS_64,OS_S64] then
                cg64.a_load64_loc_ref(list,l,r)
              else
{$endif not cpu64bitalu}
                cg.a_load_loc_ref(list,l.size,l,r);
              location_reset_ref(l,LOC_REFERENCE,l.size,0);
              l.reference:=r;
            end;
          LOC_SUBSETREG,
          LOC_CSUBSETREG,
          LOC_SUBSETREF,
          LOC_CSUBSETREF:
            begin
              tg.GetTemp(list,TCGSize2Size[l.size],TCGSize2Size[l.size],tt_normal,r);
              cg.a_load_loc_ref(list,l.size,l,r);
              location_reset_ref(l,LOC_REFERENCE,l.size,0);
              l.reference:=r;
            end;
          LOC_CREFERENCE,
          LOC_REFERENCE : ;
          else
            internalerror(200203219);
        end;
      end;


    procedure location_get_data_ref(list:TAsmList;const l:tlocation;var ref:treference;loadref:boolean; alignment: longint);
      begin
        case l.loc of
          LOC_REGISTER,
          LOC_CREGISTER :
            begin
              if not loadref then
                internalerror(200410231);
              reference_reset_base(ref,l.register,0,alignment);
            end;
          LOC_REFERENCE,
          LOC_CREFERENCE :
            begin
              if loadref then
                begin
                  reference_reset_base(ref,cg.getaddressregister(list),0,alignment);
                  cg.a_load_ref_reg(list,OS_ADDR,OS_ADDR,l.reference,ref.base);
                end
              else
                ref:=l.reference;
            end;
          else
            internalerror(200309181);
        end;
      end;


{****************************************************************************
                            Init/Finalize Code
****************************************************************************}

    procedure copyvalueparas(p:TObject;arg:pointer);
      var
        href : treference;
        hreg : tregister;
        list : TAsmList;
        hsym : tparavarsym;
        l    : longint;
        localcopyloc : tlocation;
>>>>>>> graemeg/cpstrnew
      begin
<<<<<<< HEAD
        l.size:=def_cgsize(def);
        if (def.typ=floatdef) and
           not(cs_fp_emulation in current_settings.moduleswitches) then
          begin
            if use_vectorfpu(def) then
              begin
<<<<<<< HEAD
                if constant then
                  location_reset(l,LOC_CMMREGISTER,l.size)
                else
                  location_reset(l,LOC_MMREGISTER,l.size);
                l.register:=cg.getmmregister(list,l.size);
=======
                { cdecl functions don't have a high pointer so it is not possible to generate
                  a local copy }
                if not(current_procinfo.procdef.proccalloption in cdecl_pocalls) then
=======
        list:=TAsmList(arg);
        if (tsym(p).typ=paravarsym) and
           (tparavarsym(p).varspez=vs_value) and
          (paramanager.push_addr_param(tparavarsym(p).varspez,tparavarsym(p).vardef,current_procinfo.procdef.proccalloption)) then
          begin
            location_get_data_ref(list,tparavarsym(p).initialloc,href,true);
            if is_open_array(tparavarsym(p).vardef) or
               is_array_of_const(tparavarsym(p).vardef) then
              begin
                { cdecl functions don't have a high pointer so it is not possible to generate
                  a local copy }
                if not(current_procinfo.procdef.proccalloption in [pocall_cdecl,pocall_cppdecl]) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                  begin
                    hsym:=tparavarsym(tsym(p).owner.Find('high'+tsym(p).name));
                    if not assigned(hsym) then
                      internalerror(200306061);
                    hreg:=cg.getaddressregister(list);
                    if not is_packed_array(tparavarsym(p).vardef) then
                      cg.g_copyvaluepara_openarray(list,href,hsym.initialloc,tarraydef(tparavarsym(p).vardef).elesize,hreg)
                    else
                      internalerror(2006080401);
//                      cg.g_copyvaluepara_packedopenarray(list,href,hsym.intialloc,tarraydef(tparavarsym(p).vardef).elepackedbitsize,hreg);
                    cg.a_load_reg_loc(list,OS_ADDR,hreg,tparavarsym(p).initialloc);
                  end;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
              end
            else
              begin
                if constant then
                  location_reset(l,LOC_CFPUREGISTER,l.size)
                else
                  location_reset(l,LOC_FPUREGISTER,l.size);
                l.register:=cg.getfpuregister(list,l.size);
              end;
          end
        else
          begin
            if constant then
              location_reset(l,LOC_CREGISTER,l.size)
            else
              location_reset(l,LOC_REGISTER,l.size);
{$ifdef cpu64bitalu}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            if l.size in [OS_128,OS_S128,OS_F128] then
              begin
                l.register128.reglo:=cg.getintregister(list,OS_64);
                l.register128.reghi:=cg.getintregister(list,OS_64);
              end
            else
{$else cpu64bitalu}
            if l.size in [OS_64,OS_S64,OS_F64] then
              begin
                l.register64.reglo:=cg.getintregister(list,OS_32);
                l.register64.reghi:=cg.getintregister(list,OS_32);
              end
            else
{$endif cpu64bitalu}
            { Note: for widths of records (and maybe objects, classes, etc.) an
                    address register could be set here, but that is later
                    changed to an intregister neverthless when in the
                    tcgassignmentnode thlcgobj.maybe_change_load_node_reg is
                    called for the temporary node; so the workaround for now is
                    to fix the symptoms... }
              l.register:=cg.getintregister(list,l.size);
=======
=======
>>>>>>> origin/fixes_2_2
              end
            else
              begin
                { Allocate space for the local copy }
                l:=tparavarsym(p).getsize;
                localcopyloc.loc:=LOC_REFERENCE;
                localcopyloc.size:=int_cgsize(l);
                tg.GetLocal(list,l,tparavarsym(p).vardef,localcopyloc.reference);
                { Copy data }
                if is_shortstring(tparavarsym(p).vardef) then
                  begin
                    { this code is only executed before the code for the body and the entry/exit code is generated
                      so we're allowed to include pi_do_call here; after pass1 is run, this isn't allowed anymore
                    }
                    include(current_procinfo.flags,pi_do_call);
                    cg.g_copyshortstring(list,href,localcopyloc.reference,tstringdef(tparavarsym(p).vardef).len)
                  end
                else if tparavarsym(p).vardef.typ = variantdef then
                  begin
                    { this code is only executed before the code for the body and the entry/exit code is generated
                      so we're allowed to include pi_do_call here; after pass1 is run, this isn't allowed anymore
                    }
                    include(current_procinfo.flags,pi_do_call);
                    cg.g_copyvariant(list,href,localcopyloc.reference)
                  end
                else
                  begin
                    { pass proper alignment info }
                    localcopyloc.reference.alignment:=tparavarsym(p).vardef.alignment;
                    cg.g_concatcopy(list,href,localcopyloc.reference,tparavarsym(p).vardef.size);
                  end;
                { update localloc of varsym }
                tg.Ungetlocal(list,tparavarsym(p).localloc.reference);
                tparavarsym(p).localloc:=localcopyloc;
                tparavarsym(p).initialloc:=localcopyloc;
              end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          end;
      end;


{****************************************************************************
                            Init/Finalize Code
****************************************************************************}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      trashintvalues: array[0..nroftrashvalues-1] of aint = ($5555555555555555,aint($AAAAAAAAAAAAAAAA),aint($EFEFEFEFEFEFEFEF),0);
{$endif cpu64bitalu}
{$ifdef cpu32bitalu}
      trashintvalues: array[0..nroftrashvalues-1] of aint = ($55555555,aint($AAAAAAAA),aint($EFEFEFEF),0);
{$endif cpu32bitalu}
{$ifdef cpu8bitalu}
      trashintvalues: array[0..nroftrashvalues-1] of aint = ($55,aint($AA),aint($EF),0);
{$endif cpu8bitalu}

    procedure trash_reference(list: TAsmList; const ref: treference; size: aint);
      var
        countreg, valuereg: tregister;
        hl: tasmlabel;
        trashintval: aint;
        tmpref: treference;
      begin
        trashintval := trashintvalues[localvartrashing];
        case size of
          0: ; { empty record }
          1: cg.a_load_const_ref(list,OS_8,byte(trashintval),ref);
          2: cg.a_load_const_ref(list,OS_16,word(trashintval),ref);
          4: cg.a_load_const_ref(list,OS_32,longint(trashintval),ref);
          {$ifdef cpu64bitalu}
          8: cg.a_load_const_ref(list,OS_64,int64(trashintval),ref);
          {$endif cpu64bitalu}
          else
            begin
              countreg := cg.getintregister(list,OS_ADDR);
              valuereg := cg.getintregister(list,OS_8);
              cg.a_load_const_reg(list,OS_INT,size,countreg);
              cg.a_load_const_reg(list,OS_8,byte(trashintval),valuereg);
              current_asmdata.getjumplabel(hl);
              tmpref := ref;
              if (tmpref.index <> NR_NO) then
                internalerror(200607201);
              tmpref.index := countreg;
              dec(tmpref.offset);
              cg.a_label(list,hl);
              cg.a_load_reg_ref(list,OS_8,OS_8,valuereg,tmpref);
              cg.a_op_const_reg(list,OP_SUB,OS_INT,1,countreg);
              cg.a_cmp_const_reg_label(list,OS_INT,OC_NE,0,countreg,hl);
              cg.a_reg_sync(list,tmpref.base);
              cg.a_reg_sync(list,valuereg);
            end;
        end;
      end;


    { trash contents of local variables or parameters (function result) }
    procedure trash_variable(p:TObject;arg:pointer);
      var
        trashintval: aint;
        list: TAsmList absolute arg;
      begin
        if ((tsym(p).typ=localvarsym) or
            ((tsym(p).typ=paravarsym) and
             (vo_is_funcret in tparavarsym(p).varoptions))) and
           not(is_managed_type(tabstractnormalvarsym(p).vardef)) and
           not(assigned(tabstractnormalvarsym(p).defaultconstsym)) then
         begin
           trashintval := trashintvalues[localvartrashing];
           case tabstractnormalvarsym(p).initialloc.loc of
             LOC_CREGISTER :
{$ifopt q+}
{$define overflowon}
{$q-}
{$endif}
               begin
                 { avoid problems with broken x86 shifts }
                 case tcgsize2size[tabstractnormalvarsym(p).initialloc.size] of
                   1: cg.a_load_const_reg(list,OS_8,byte(trashintval),tabstractnormalvarsym(p).initialloc.register);
                   2: cg.a_load_const_reg(list,OS_16,word(trashintval),tabstractnormalvarsym(p).initialloc.register);
                   4: cg.a_load_const_reg(list,OS_32,longint(trashintval),tabstractnormalvarsym(p).initialloc.register);
                   8:
                     begin
{$ifdef cpu64bitalu}
                       cg.a_load_const_reg(list,OS_64,aint(trashintval),tabstractnormalvarsym(p).initialloc.register);
{$else}
                       cg64.a_load64_const_reg(list,int64(trashintval) shl 32 or int64(trashintval),tabstractnormalvarsym(p).initialloc.register64);
{$endif}
                     end;
                   else
                     internalerror(2010060801);
                 end;
               end;
{$ifdef overflowon}
{$undef overflowon}
{$q+}
{$endif}
             LOC_REFERENCE :
               begin
                   if ((tsym(p).typ=localvarsym) and
                       not(vo_is_funcret in tabstractvarsym(p).varoptions)) or
                      not is_shortstring(tabstractnormalvarsym(p).vardef) then
                     trash_reference(list,tabstractnormalvarsym(p).initialloc.reference,
                       tlocalvarsym(p).getsize)
                   else
                     { may be an open string, even if is_open_string() returns }
                     { false (for some helpers in the system unit)             }
                     { an open string has at least size 2                      }
                     trash_reference(list,tabstractnormalvarsym(p).initialloc.reference,
                       2);
               end;
             LOC_CMMREGISTER :
               ;
             LOC_CFPUREGISTER :
               ;
             else
               internalerror(200410124);
           end;
         end;
      end;

>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
    { generates the code for incrementing the reference count of parameters and
      initialize out parameters }
    procedure init_paras(p:TObject;arg:pointer);
      var
        href : treference;
        hsym : tparavarsym;
        eldef : tdef;
        list : TAsmList;
        needs_inittable : boolean;
=======
    { initializes the regvars from staticsymtable with 0 }
    procedure initialize_regvars(p:TObject;arg:pointer);
      var
        href : treference;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      begin
        list:=TAsmList(arg);
        if (tsym(p).typ=paravarsym) then
         begin
           needs_inittable:=is_managed_type(tparavarsym(p).vardef);
           if not needs_inittable then
             exit;
           case tparavarsym(p).varspez of
             vs_value :
               begin
                 { variants are already handled by the call to fpc_variant_copy_overwrite if
                   they are passed by reference }
                 if not((tparavarsym(p).vardef.typ=variantdef) and
                    paramanager.push_addr_param(tparavarsym(p).varspez,tparavarsym(p).vardef,current_procinfo.procdef.proccalloption)) then
                   begin
                     hlcg.location_get_data_ref(list,tparavarsym(p).vardef,tparavarsym(p).initialloc,href,
                       is_open_array(tparavarsym(p).vardef) or
                       ((target_info.system in systems_caller_copy_addr_value_para) and
                        paramanager.push_addr_param(vs_value,tparavarsym(p).vardef,current_procinfo.procdef.proccalloption)),
                        sizeof(pint));
                     if is_open_array(tparavarsym(p).vardef) then
                       begin
                         { open arrays do not contain correct element count in their rtti,
                           the actual count must be passed separately. }
                         hsym:=tparavarsym(get_high_value_sym(tparavarsym(p)));
                         eldef:=tarraydef(tparavarsym(p).vardef).elementdef;
                         if not assigned(hsym) then
                           internalerror(201003031);
                         hlcg.g_array_rtti_helper(list,eldef,href,hsym.initialloc,'fpc_addref_array');
                       end
                     else
                      hlcg.g_incrrefcount(list,tparavarsym(p).vardef,href);
                   end;
               end;
             vs_out :
               begin
                 { we have no idea about the alignment at the callee side,
                   and the user also cannot specify "unaligned" here, so
                   assume worst case }
                 hlcg.location_get_data_ref(list,tparavarsym(p).vardef,tparavarsym(p).initialloc,href,true,1);
                 if is_open_array(tparavarsym(p).vardef) then
                   begin
                     hsym:=tparavarsym(get_high_value_sym(tparavarsym(p)));
                     eldef:=tarraydef(tparavarsym(p).vardef).elementdef;
                     if not assigned(hsym) then
                       internalerror(201103033);
                     hlcg.g_array_rtti_helper(list,eldef,href,hsym.initialloc,'fpc_initialize_array');
                   end
                 else
                   hlcg.g_initialize(list,tparavarsym(p).vardef,href);
               end;
<<<<<<< HEAD
=======
             LOC_CMMREGISTER :
               { clear the whole register }
               cg.a_opmm_reg_reg(TAsmList(arg),OP_XOR,reg_cgsize(tstaticvarsym(p).initialloc.register),
                 tstaticvarsym(p).initialloc.register,
                 tstaticvarsym(p).initialloc.register,
                 nil);
             LOC_CFPUREGISTER :
               begin
                 { initialize fpu regvar by loading from memory }
                 reference_reset_symbol(href,
                   current_asmdata.RefAsmSymbol(tstaticvarsym(p).mangledname), 0);
                 cg.a_loadfpu_ref_reg(TAsmList(arg), tstaticvarsym(p).initialloc.size,
                   tstaticvarsym(p).initialloc.size, href, tstaticvarsym(p).initialloc.register);
               end;
             LOC_INVALID :
               ;
             else
               internalerror(200410124);
>>>>>>> graemeg/fixes_2_2
           end;
         end;
      end;


<<<<<<< HEAD
    procedure gen_alloc_regloc(list:TAsmList;var loc: tlocation);
      begin
        case loc.loc of
          LOC_CREGISTER:
            begin
{$ifdef cpu64bitalu}
              if loc.size in [OS_128,OS_S128] then
                begin
                  loc.register128.reglo:=cg.getintregister(list,OS_64);
                  loc.register128.reghi:=cg.getintregister(list,OS_64);
                end
              else
{$else cpu64bitalu}
              if loc.size in [OS_64,OS_S64] then
                begin
                  loc.register64.reglo:=cg.getintregister(list,OS_32);
                  loc.register64.reghi:=cg.getintregister(list,OS_32);
                end
              else
{$endif cpu64bitalu}
                loc.register:=cg.getintregister(list,loc.size);
=======
    { generates the code for initialisation of local data }
    procedure initialize_data(p:TObject;arg:pointer);
      var
        OldAsmList : TAsmList;
        hp : tnode;
      begin
<<<<<<< HEAD
        if (tsym(p).typ = localvarsym) and
           { local (procedure or unit) variables only need initialization if
             they are used }
=======
        if (tsym(p).typ in [staticvarsym,localvarsym]) and
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           ((tabstractvarsym(p).refs>0) or
            { managed return symbols must be inited }
            ((tsym(p).typ=localvarsym) and (vo_is_funcret in tlocalvarsym(p).varoptions))
           ) and
           not(vo_is_typed_const in tabstractvarsym(p).varoptions) and
           not(vo_is_external in tabstractvarsym(p).varoptions) and
           (is_managed_type(tabstractvarsym(p).vardef) or
            ((m_iso in current_settings.modeswitches) and (tabstractvarsym(p).vardef.typ=filedef))
           ) then
          begin
            OldAsmList:=current_asmdata.CurrAsmList;
            current_asmdata.CurrAsmList:=TAsmList(arg);
            hp:=initialize_data_node(cloadnode.create(tsym(p),tsym(p).owner));
            firstpass(hp);
            secondpass(hp);
            hp.free;
            current_asmdata.CurrAsmList:=OldAsmList;
          end;
      end;


    procedure finalize_sym(asmlist:TAsmList;sym:tsym);
      var
        hp : tnode;
        OldAsmList : TAsmList;
      begin
        include(current_procinfo.flags,pi_needs_implicit_finally);
        OldAsmList:=current_asmdata.CurrAsmList;
        current_asmdata.CurrAsmList:=asmlist;
        hp:=finalize_data_node(cloadnode.create(sym,sym.owner));
        firstpass(hp);
        secondpass(hp);
        hp.free;
        current_asmdata.CurrAsmList:=OldAsmList;
      end;


    { generates the code for finalisation of local variables }
    procedure finalize_local_vars(p:TObject;arg:pointer);
      begin
        if (tsym(p).typ=localvarsym) and
           (tlocalvarsym(p).refs>0) and
           not(vo_is_external in tlocalvarsym(p).varoptions) and
           not(vo_is_funcret in tlocalvarsym(p).varoptions) and
           is_managed_type(tlocalvarsym(p).vardef) then
          finalize_sym(TAsmList(arg),tsym(p));
      end;


    { generates the code for finalization of static symtable and
      all local (static) typed consts }
    procedure finalize_static_data(p:TObject;arg:pointer);
      var
        i : longint;
        pd : tprocdef;
      begin
        case tsym(p).typ of
          staticvarsym :
            begin
                  { local (procedure or unit) variables only need finalization
                    if they are used
                  }
              if ((tstaticvarsym(p).refs>0) or
                  { global (unit) variables always need finalization, since
                    they may also be used in another unit
                  }
                  (tstaticvarsym(p).owner.symtabletype=globalsymtable)) and
                 (tstaticvarsym(p).varspez<>vs_const) and
                 not(vo_is_funcret in tstaticvarsym(p).varoptions) and
                 not(vo_is_external in tstaticvarsym(p).varoptions) and
                 is_managed_type(tstaticvarsym(p).vardef) then
                finalize_sym(TAsmList(arg),tsym(p));
>>>>>>> graemeg/cpstrnew
            end;
          LOC_CFPUREGISTER:
            begin
              loc.register:=cg.getfpuregister(list,loc.size);
            end;
          LOC_CMMREGISTER:
            begin
             loc.register:=cg.getmmregister(list,loc.size);
            end;
        end;
      end;


<<<<<<< HEAD
    procedure gen_alloc_regvar(list:TAsmList;sym: tabstractnormalvarsym; allocreg: boolean);
      begin
        if allocreg then
          gen_alloc_regloc(list,sym.initialloc);
        if (pi_has_label in current_procinfo.flags) then
          begin
            { Allocate register already, to prevent first allocation to be
              inside a loop }
{$if defined(cpu64bitalu)}
            if sym.initialloc.size in [OS_128,OS_S128] then
              begin
                cg.a_reg_sync(list,sym.initialloc.register128.reglo);
                cg.a_reg_sync(list,sym.initialloc.register128.reghi);
              end
            else
{$elseif defined(cpu32bitalu)}
            if sym.initialloc.size in [OS_64,OS_S64] then
              begin
                cg.a_reg_sync(list,sym.initialloc.register64.reglo);
                cg.a_reg_sync(list,sym.initialloc.register64.reghi);
              end
            else
{$elseif defined(cpu16bitalu)}
            if sym.initialloc.size in [OS_64,OS_S64] then
              begin
                cg.a_reg_sync(list,sym.initialloc.register64.reglo);
                cg.a_reg_sync(list,GetNextReg(sym.initialloc.register64.reglo));
                cg.a_reg_sync(list,sym.initialloc.register64.reghi);
                cg.a_reg_sync(list,GetNextReg(sym.initialloc.register64.reghi));
              end
            else
            if sym.initialloc.size in [OS_32,OS_S32] then
              begin
                cg.a_reg_sync(list,sym.initialloc.register);
                cg.a_reg_sync(list,GetNextReg(sym.initialloc.register));
              end
            else
{$elseif defined(cpu8bitalu)}
            if sym.initialloc.size in [OS_64,OS_S64] then
              begin
                cg.a_reg_sync(list,sym.initialloc.register64.reglo);
                cg.a_reg_sync(list,GetNextReg(sym.initialloc.register64.reglo));
                cg.a_reg_sync(list,GetNextReg(GetNextReg(sym.initialloc.register64.reglo)));
                cg.a_reg_sync(list,GetNextReg(GetNextReg(GetNextReg(sym.initialloc.register64.reglo))));
                cg.a_reg_sync(list,sym.initialloc.register64.reghi);
                cg.a_reg_sync(list,GetNextReg(sym.initialloc.register64.reghi));
                cg.a_reg_sync(list,GetNextReg(GetNextReg(sym.initialloc.register64.reghi)));
                cg.a_reg_sync(list,GetNextReg(GetNextReg(GetNextReg(sym.initialloc.register64.reghi))));
              end
            else
            if sym.initialloc.size in [OS_32,OS_S32] then
              begin
                cg.a_reg_sync(list,sym.initialloc.register);
                cg.a_reg_sync(list,GetNextReg(sym.initialloc.register));
                cg.a_reg_sync(list,GetNextReg(GetNextReg(sym.initialloc.register)));
                cg.a_reg_sync(list,GetNextReg(GetNextReg(GetNextReg(sym.initialloc.register))));
              end
            else
            if sym.initialloc.size in [OS_16,OS_S16] then
              begin
                cg.a_reg_sync(list,sym.initialloc.register);
                cg.a_reg_sync(list,GetNextReg(sym.initialloc.register));
              end
            else
{$endif}
             cg.a_reg_sync(list,sym.initialloc.register);
=======
    { generates the code for incrementing the reference count of parameters and
      initialize out parameters }
    procedure init_paras(p:TObject;arg:pointer);
      var
        href : treference;
        hsym : tparavarsym;
        eldef : tdef;
        tmpreg : tregister;
        list : TAsmList;
        needs_inittable,
        do_trashing       : boolean;
      begin
        list:=TAsmList(arg);
        if (tsym(p).typ=paravarsym) then
         begin
<<<<<<< HEAD
           needs_inittable:=is_managed_type(tparavarsym(p).vardef);
           do_trashing:=
=======
           needs_inittable :=
             not is_class(tparavarsym(p).vardef) and
             tparavarsym(p).vardef.needs_inittable;
           do_trashing :=
>>>>>>> graemeg/fixes_2_2
             (localvartrashing <> -1) and
             (not assigned(tparavarsym(p).defaultconstsym)) and
             not needs_inittable;
           case tparavarsym(p).varspez of
             vs_value :
               if needs_inittable then
                 begin
                   { variants are already handled by the call to fpc_variant_copy_overwrite if
                     they are passed by reference }
                   if not((tparavarsym(p).vardef.typ=variantdef) and
                     paramanager.push_addr_param(tparavarsym(p).varspez,tparavarsym(p).vardef,current_procinfo.procdef.proccalloption)) then
                     begin
<<<<<<< HEAD
<<<<<<< HEAD
                       location_get_data_ref(list,tparavarsym(p).initialloc,href,is_open_array(tparavarsym(p).vardef),sizeof(pint));
                       if is_open_array(tparavarsym(p).vardef) then
                         begin
                           { open arrays do not contain correct element count in their rtti,
                             the actual count must be passed separately. }
                           hsym:=tparavarsym(tsym(p).owner.Find('high'+tsym(p).name));
                           eldef:=tarraydef(tparavarsym(p).vardef).elementdef;
                           if not assigned(hsym) then
                             internalerror(201003031);
                           cg.g_array_rtti_helper(list,eldef,href,hsym.initialloc,'FPC_ADDREF_ARRAY');
                         end
                       else
                         cg.g_incrrefcount(list,tparavarsym(p).vardef,href);
=======
                       location_get_data_ref(list,tparavarsym(p).initialloc,href,is_open_array(tparavarsym(p).vardef));
                       cg.g_incrrefcount(list,tparavarsym(p).vardef,href);
>>>>>>> graemeg/fixes_2_2
=======
                       location_get_data_ref(list,tparavarsym(p).initialloc,href,is_open_array(tparavarsym(p).vardef));
                       cg.g_incrrefcount(list,tparavarsym(p).vardef,href);
>>>>>>> origin/fixes_2_2
                     end;
                 end;
             vs_out :
               begin
                 if needs_inittable or
                    do_trashing then
                   begin
                     tmpreg:=cg.getaddressregister(list);
                     cg.a_load_loc_reg(list,OS_ADDR,tparavarsym(p).initialloc,tmpreg);
                     { we have no idea about the alignment at the callee side,
                       and the user also cannot specify "unaligned" here, so
                       assume worst case }
                     reference_reset_base(href,tmpreg,0,1);
                     if do_trashing and
                        { needs separate implementation to trash open arrays }
                        { since their size is only known at run time         }
                        not is_special_array(tparavarsym(p).vardef) then
                        { may be an open string, even if is_open_string() returns }
                        { false (for some helpers in the system unit)             }
                       if not is_shortstring(tparavarsym(p).vardef) then
                         trash_reference(list,href,tparavarsym(p).vardef.size)
                       else
                         trash_reference(list,href,2);
                     if needs_inittable then
                       begin
                         if is_open_array(tparavarsym(p).vardef) then
                           begin
                             hsym:=tparavarsym(tsym(p).owner.Find('high'+tsym(p).name));
                             eldef:=tarraydef(tparavarsym(p).vardef).elementdef;
                             if not assigned(hsym) then
                               internalerror(201103033);
                             cg.g_array_rtti_helper(list,eldef,href,hsym.initialloc,'FPC_INITIALIZE_ARRAY');
                           end
                         else
                           cg.g_initialize(list,tparavarsym(p).vardef,href);
                       end;
                   end;
               end;
             else if do_trashing and
                     ([vo_is_funcret,vo_is_hidden_para] * tparavarsym(p).varoptions = [vo_is_funcret,vo_is_hidden_para]) then
                   begin
                     tmpreg:=cg.getaddressregister(list);
                     cg.a_load_loc_reg(list,OS_ADDR,tparavarsym(p).initialloc,tmpreg);
                     { should always have standard alignment. If a function is assigned
                       to a non-aligned variable, the optimisation to pass this variable
                       directly as hidden function result must/cannot be performed
                       (see tcallnode.funcret_can_be_reused)
                     }
                     reference_reset_base(href,tmpreg,0,
                       used_align(tparavarsym(p).vardef.alignment,current_settings.alignment.localalignmin,current_settings.alignment.localalignmax));
                     { may be an open string, even if is_open_string() returns }
                     { false (for some helpers in the system unit)             }
                     if not is_shortstring(tparavarsym(p).vardef) then
                       trash_reference(list,href,tparavarsym(p).vardef.size)
                     else
                       { an open string has at least size 2 }
                       trash_reference(list,href,2);
                   end
           end;
         end;
      end;


    { generates the code for decrementing the reference count of parameters }
    procedure final_paras(p:TObject;arg:pointer);
      var
        list : TAsmList;
        href : treference;
        hsym : tparavarsym;
        eldef : tdef;
      begin
        if not(tsym(p).typ=paravarsym) then
          exit;
        list:=TAsmList(arg);
<<<<<<< HEAD
<<<<<<< HEAD
        if is_managed_type(tparavarsym(p).vardef) then
=======
=======
>>>>>>> origin/fixes_2_2
        if not is_class(tparavarsym(p).vardef) and
           tparavarsym(p).vardef.needs_inittable then
>>>>>>> graemeg/fixes_2_2
         begin
           if (tparavarsym(p).varspez=vs_value) then
            begin
              include(current_procinfo.flags,pi_needs_implicit_finally);
              location_get_data_ref(list,tparavarsym(p).localloc,href,is_open_array(tparavarsym(p).vardef),sizeof(pint));
              if is_open_array(tparavarsym(p).vardef) then
                begin
                  hsym:=tparavarsym(tsym(p).owner.Find('high'+tsym(p).name));
                  eldef:=tarraydef(tparavarsym(p).vardef).elementdef;
                  if not assigned(hsym) then
                    internalerror(201003032);
                  cg.g_array_rtti_helper(list,eldef,href,hsym.initialloc,'FPC_DECREF_ARRAY');
                end
              else
                cg.g_decrrefcount(list,tparavarsym(p).vardef,href);
            end;
         end;
        { open arrays can contain elements requiring init/final code, so the else has been removed here }
        if (tparavarsym(p).varspez=vs_value) and
           (is_open_array(tparavarsym(p).vardef) or
            is_array_of_const(tparavarsym(p).vardef)) then
          begin
            { cdecl functions don't have a high pointer so it is not possible to generate
              a local copy }
<<<<<<< HEAD
<<<<<<< HEAD
            if not(current_procinfo.procdef.proccalloption in cdecl_pocalls) then
              cg.g_releasevaluepara_openarray(list,tparavarsym(p).localloc);
>>>>>>> graemeg/cpstrnew
          end;
        sym.localloc:=sym.initialloc;
=======
            if not(current_procinfo.procdef.proccalloption in [pocall_cdecl,pocall_cppdecl]) then
              cg.g_releasevaluepara_openarray(list,tparavarsym(p).localloc);
          end;
>>>>>>> graemeg/fixes_2_2
=======
            if not(current_procinfo.procdef.proccalloption in [pocall_cdecl,pocall_cppdecl]) then
              cg.g_releasevaluepara_openarray(list,tparavarsym(p).localloc);
          end;
>>>>>>> origin/fixes_2_2
      end;


<<<<<<< HEAD
    procedure gen_load_cgpara_loc(list: TAsmList; vardef: tdef; const para: TCGPara; var destloc: tlocation; reusepara: boolean);
=======
    { Initialize temp ansi/widestrings,interfaces }
    procedure inittempvariables(list:TAsmList);
      var
        hp : ptemprecord;
        href : treference;
      begin
        hp:=tg.templist;
        while assigned(hp) do
         begin
           if assigned(hp^.def) and
              is_managed_type(hp^.def) then
            begin
              reference_reset_base(href,current_procinfo.framepointer,hp^.pos,sizeof(pint));
              cg.g_initialize(list,hp^.def,href);
            end;
           hp:=hp^.next;
         end;
      end;
>>>>>>> graemeg/cpstrnew

      procedure unget_para(const paraloc:TCGParaLocation);
        begin
           case paraloc.loc of
             LOC_REGISTER :
               begin
                 if getsupreg(paraloc.register)<first_int_imreg then
                   cg.ungetcpuregister(list,paraloc.register);
               end;
             LOC_MMREGISTER :
               begin
                 if getsupreg(paraloc.register)<first_mm_imreg then
                   cg.ungetcpuregister(list,paraloc.register);
               end;
             LOC_FPUREGISTER :
               begin
                 if getsupreg(paraloc.register)<first_fpu_imreg then
                   cg.ungetcpuregister(list,paraloc.register);
               end;
           end;
        end;

      var
        paraloc   : pcgparalocation;
        href      : treference;
        sizeleft  : aint;
        tempref   : treference;
{$ifdef mips}
        //tmpreg   : tregister;
{$endif mips}
{$ifndef cpu64bitalu}
        tempreg  : tregister;
        reg64    : tregister64;
{$if defined(cpu8bitalu)}
        curparaloc : PCGParaLocation;
{$endif defined(cpu8bitalu)}
{$endif not cpu64bitalu}
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        paraloc:=para.location;
        if not assigned(paraloc) then
          internalerror(200408203);
        { skip e.g. empty records }
        if (paraloc^.loc = LOC_VOID) then
          exit;
        case destloc.loc of
          LOC_REFERENCE :
            begin
<<<<<<< HEAD
              { If the parameter location is reused we don't need to copy
                anything }
              if not reusepara then
=======
              include(current_procinfo.flags,pi_needs_implicit_finally);
              reference_reset_base(href,current_procinfo.framepointer,hp^.pos);
              cg.g_finalize(list,hp^.def,href);
            end;
           hp:=hp^.next;
         end;
      end;


    procedure gen_load_return_value(list:TAsmList);
      var
{$ifndef cpu64bit}
        href   : treference;
{$endif cpu64bit}
        ressym : tabstractnormalvarsym;
        resloc,
        restmploc : tlocation;
        hreg   : tregister;
        funcretloc : tlocation;
      begin
        { Is the loading needed? }
        if (current_procinfo.procdef.funcretloc[calleeside].loc=LOC_VOID) or
           (
            (po_assembler in current_procinfo.procdef.procoptions) and
            (not(assigned(current_procinfo.procdef.funcretsym)) or
             (tabstractvarsym(current_procinfo.procdef.funcretsym).refs=0))
           ) then
           exit;

        funcretloc:=current_procinfo.procdef.funcretloc[calleeside];

        { constructors return self }
        if (current_procinfo.procdef.proctypeoption=potype_constructor) then
          ressym:=tabstractnormalvarsym(current_procinfo.procdef.parast.Find('self'))
        else
          ressym:=tabstractnormalvarsym(current_procinfo.procdef.funcretsym);
        if (ressym.refs>0) or (ressym.vardef.needs_inittable) then
          begin
{$ifdef OLDREGVARS}
            case ressym.localloc.loc of
              LOC_CFPUREGISTER,
              LOC_FPUREGISTER:
                begin
                  location_reset(restmploc,LOC_CFPUREGISTER,funcretloc^.size);
                  restmploc.register:=ressym.localloc.register;
                end;

              LOC_CREGISTER,
              LOC_REGISTER:
                begin
                  location_reset(restmploc,LOC_CREGISTER,funcretloc^.size);
                  restmploc.register:=ressym.localloc.register;
                end;

              LOC_MMREGISTER:
                begin
                  location_reset(restmploc,LOC_CMMREGISTER,funcretloc^.size);
                  restmploc.register:=ressym.localloc.register;
                end;

              LOC_REFERENCE:
                begin
                  location_reset(restmploc,LOC_REFERENCE,funcretloc^.size);
                  restmploc.reference:=ressym.localloc.reference;
                end;
              else
                internalerror(200309184);
            end;
{$else}
            restmploc:=ressym.localloc;
{$endif}

            { Here, we return the function result. In most architectures, the value is
              passed into the FUNCTION_RETURN_REG, but in a windowed architecure like sparc a
              function returns in a register and the caller receives it in an other one }
            case funcretloc.loc of
              LOC_REGISTER:
>>>>>>> graemeg/fixes_2_2
                begin
                  href:=destloc.reference;
                  sizeleft:=para.intsize;
                  while assigned(paraloc) do
                    begin
                      if (paraloc^.size=OS_NO) then
                        begin
                          { Can only be a reference that contains the rest
                            of the parameter }
                          if (paraloc^.loc<>LOC_REFERENCE) or
                             assigned(paraloc^.next) then
                            internalerror(2005013010);
                          cg.a_load_cgparaloc_ref(list,paraloc^,href,sizeleft,destloc.reference.alignment);
                          inc(href.offset,sizeleft);
                          sizeleft:=0;
                        end
                      else
                        begin
                          cg.a_load_cgparaloc_ref(list,paraloc^,href,tcgsize2size[paraloc^.size],destloc.reference.alignment);
                          inc(href.offset,TCGSize2Size[paraloc^.size]);
                          dec(sizeleft,TCGSize2Size[paraloc^.size]);
                        end;
                      unget_para(paraloc^);
                      paraloc:=paraloc^.next;
                    end;
                end;
            end;
          LOC_REGISTER,
          LOC_CREGISTER :
            begin
{$ifdef cpu64bitalu}
              if (para.size in [OS_128,OS_S128,OS_F128]) and
                 ({ in case of fpu emulation, or abi's that pass fpu values
                    via integer registers }
                  (vardef.typ=floatdef) or
                   is_methodpointer(vardef) or
                   is_record(vardef)) then
                begin
                  case paraloc^.loc of
                    LOC_REGISTER:
                      begin
                        if not assigned(paraloc^.next) then
                          internalerror(200410104);
                        if (target_info.endian=ENDIAN_BIG) then
                          begin
                            { paraloc^ -> high
                              paraloc^.next -> low }
                            unget_para(paraloc^);
                            gen_alloc_regloc(list,destloc);
                            { reg->reg, alignment is irrelevant }
                            cg.a_load_cgparaloc_anyreg(list,OS_64,paraloc^,destloc.register128.reghi,8);
                            unget_para(paraloc^.next^);
                            cg.a_load_cgparaloc_anyreg(list,OS_64,paraloc^.next^,destloc.register128.reglo,8);
                          end
                        else
                          begin
                            { paraloc^ -> low
                              paraloc^.next -> high }
                            unget_para(paraloc^);
                            gen_alloc_regloc(list,destloc);
                            cg.a_load_cgparaloc_anyreg(list,OS_64,paraloc^,destloc.register128.reglo,8);
                            unget_para(paraloc^.next^);
                            cg.a_load_cgparaloc_anyreg(list,OS_64,paraloc^.next^,destloc.register128.reghi,8);
                          end;
                      end;
                    LOC_REFERENCE:
                      begin
                        gen_alloc_regloc(list,destloc);
                        reference_reset_base(href,paraloc^.reference.index,paraloc^.reference.offset,para.alignment);
                        cg128.a_load128_ref_reg(list,href,destloc.register128);
                        unget_para(paraloc^);
                      end;
                    else
                      internalerror(2012090607);
                  end
                end
              else
{$else cpu64bitalu}
              if (para.size in [OS_64,OS_S64,OS_F64]) and
                 (is_64bit(vardef) or
                  { in case of fpu emulation, or abi's that pass fpu values
                    via integer registers }
                  (vardef.typ=floatdef) or
                   is_methodpointer(vardef) or
                   is_record(vardef)) then
                begin
                  case paraloc^.loc of
                    LOC_REGISTER:
                      begin
                        case para.locations_count of
{$if defined(cpu8bitalu)}
                          { 8 paralocs? }
                          8:
                            if (target_info.endian=ENDIAN_BIG) then
                              begin
                                { is there any big endian 8 bit ALU/16 bit Addr CPU? }
                                internalerror(2015041003);
                                { paraloc^ -> high
                                  paraloc^.next^.next^.next^.next -> low }
                                unget_para(paraloc^);
                                gen_alloc_regloc(list,destloc);
                                { reg->reg, alignment is irrelevant }
                                cg.a_load_cgparaloc_anyreg(list,OS_16,paraloc^,GetNextReg(destloc.register64.reghi),1);
                                unget_para(paraloc^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_16,paraloc^.next^,destloc.register64.reghi,1);
                                unget_para(paraloc^.next^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_16,paraloc^.next^.next^,GetNextReg(destloc.register64.reglo),1);
                                unget_para(paraloc^.next^.next^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_16,paraloc^.next^.next^.next^,destloc.register64.reglo,1);
                              end
                            else
                              begin
                                { paraloc^ -> low
                                  paraloc^.next^.next^.next^.next -> high }
                                curparaloc:=paraloc;
                                unget_para(curparaloc^);
                                gen_alloc_regloc(list,destloc);
                                cg.a_load_cgparaloc_anyreg(list,OS_8,curparaloc^,destloc.register64.reglo,2);
                                unget_para(curparaloc^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_8,curparaloc^.next^,GetNextReg(destloc.register64.reglo),1);
                                unget_para(curparaloc^.next^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_8,curparaloc^.next^.next^,GetNextReg(GetNextReg(destloc.register64.reglo)),1);
                                unget_para(curparaloc^.next^.next^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_8,curparaloc^.next^.next^.next^,GetNextReg(GetNextReg(GetNextReg(destloc.register64.reglo))),1);

                                curparaloc:=paraloc^.next^.next^.next^.next;
                                unget_para(curparaloc^);
                                cg.a_load_cgparaloc_anyreg(list,OS_8,curparaloc^,destloc.register64.reghi,2);
                                unget_para(curparaloc^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_8,curparaloc^.next^,GetNextReg(destloc.register64.reghi),1);
                                unget_para(curparaloc^.next^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_8,curparaloc^.next^.next^,GetNextReg(GetNextReg(destloc.register64.reghi)),1);
                                unget_para(curparaloc^.next^.next^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_8,curparaloc^.next^.next^.next^,GetNextReg(GetNextReg(GetNextReg(destloc.register64.reghi))),1);
                              end;
{$endif defined(cpu8bitalu)}
{$if defined(cpu16bitalu) or defined(cpu8bitalu)}
                          { 4 paralocs? }
                          4:
                            if (target_info.endian=ENDIAN_BIG) then
                              begin
                                { paraloc^ -> high
                                  paraloc^.next^.next -> low }
                                unget_para(paraloc^);
                                gen_alloc_regloc(list,destloc);
                                { reg->reg, alignment is irrelevant }
                                cg.a_load_cgparaloc_anyreg(list,OS_16,paraloc^,GetNextReg(destloc.register64.reghi),2);
                                unget_para(paraloc^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_16,paraloc^.next^,destloc.register64.reghi,2);
                                unget_para(paraloc^.next^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_16,paraloc^.next^.next^,GetNextReg(destloc.register64.reglo),2);
                                unget_para(paraloc^.next^.next^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_16,paraloc^.next^.next^.next^,destloc.register64.reglo,2);
                              end
                            else
                              begin
                                { paraloc^ -> low
                                  paraloc^.next^.next -> high }
                                unget_para(paraloc^);
                                gen_alloc_regloc(list,destloc);
                                cg.a_load_cgparaloc_anyreg(list,OS_16,paraloc^,destloc.register64.reglo,2);
                                unget_para(paraloc^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_16,paraloc^.next^,GetNextReg(destloc.register64.reglo),2);
                                unget_para(paraloc^.next^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_16,paraloc^.next^.next^,destloc.register64.reghi,2);
                                unget_para(paraloc^.next^.next^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_16,paraloc^.next^.next^.next^,GetNextReg(destloc.register64.reghi),2);
                              end;
{$endif defined(cpu16bitalu) or defined(cpu8bitalu)}
                          2:
                            if (target_info.endian=ENDIAN_BIG) then
                              begin
                                { paraloc^ -> high
                                  paraloc^.next -> low }
                                unget_para(paraloc^);
                                gen_alloc_regloc(list,destloc);
                                { reg->reg, alignment is irrelevant }
                                cg.a_load_cgparaloc_anyreg(list,OS_32,paraloc^,destloc.register64.reghi,4);
                                unget_para(paraloc^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_32,paraloc^.next^,destloc.register64.reglo,4);
                              end
                            else
                              begin
                                { paraloc^ -> low
                                  paraloc^.next -> high }
                                unget_para(paraloc^);
                                gen_alloc_regloc(list,destloc);
                                cg.a_load_cgparaloc_anyreg(list,OS_32,paraloc^,destloc.register64.reglo,4);
                                unget_para(paraloc^.next^);
                                cg.a_load_cgparaloc_anyreg(list,OS_32,paraloc^.next^,destloc.register64.reghi,4);
                              end;
                          else
                            { unexpected number of paralocs }
                            internalerror(200410104);
                        end;
                      end;
                    LOC_REFERENCE:
                      begin
                        gen_alloc_regloc(list,destloc);
                        reference_reset_base(href,paraloc^.reference.index,paraloc^.reference.offset,para.alignment);
                        cg64.a_load64_ref_reg(list,href,destloc.register64);
                        unget_para(paraloc^);
                      end;
                    else
                      internalerror(2005101501);
                  end
                end
              else
{$endif cpu64bitalu}
                begin
                  if assigned(paraloc^.next) then
                    begin
                      if (destloc.size in [OS_PAIR,OS_SPAIR]) and
                        (para.Size in [OS_PAIR,OS_SPAIR]) then
                        begin
                          unget_para(paraloc^);
                          gen_alloc_regloc(list,destloc);
                          cg.a_load_cgparaloc_anyreg(list,OS_INT,paraloc^,destloc.register,sizeof(aint));
                          unget_para(paraloc^.Next^);
                          {$if defined(cpu16bitalu) or defined(cpu8bitalu)}
                            cg.a_load_cgparaloc_anyreg(list,OS_INT,paraloc^.Next^,GetNextReg(destloc.register),sizeof(aint));
                          {$else}
                            cg.a_load_cgparaloc_anyreg(list,OS_INT,paraloc^.Next^,destloc.registerhi,sizeof(aint));
                          {$endif}
                        end
{$if defined(cpu8bitalu)}
                      else if (destloc.size in [OS_32,OS_S32]) and
                        (para.Size in [OS_32,OS_S32]) then
                        begin
                          unget_para(paraloc^);
                          gen_alloc_regloc(list,destloc);
                          cg.a_load_cgparaloc_anyreg(list,OS_8,paraloc^,destloc.register,sizeof(aint));
                          unget_para(paraloc^.Next^);
                          cg.a_load_cgparaloc_anyreg(list,OS_8,paraloc^.Next^,GetNextReg(destloc.register),sizeof(aint));
                          unget_para(paraloc^.Next^.Next^);
                          cg.a_load_cgparaloc_anyreg(list,OS_8,paraloc^.Next^.Next^,GetNextReg(GetNextReg(destloc.register)),sizeof(aint));
                          unget_para(paraloc^.Next^.Next^.Next^);
                          cg.a_load_cgparaloc_anyreg(list,OS_8,paraloc^.Next^.Next^.Next^,GetNextReg(GetNextReg(GetNextReg(destloc.register))),sizeof(aint));
                        end
{$endif defined(cpu8bitalu)}
                      else
                        begin
                          { this can happen if a parameter is spread over
                            multiple paralocs, e.g. if a record with two single
                            fields must be passed in two single precision
                            registers }
                          { does it fit in the register of destloc? }
                          sizeleft:=para.intsize;
                          if sizeleft<>vardef.size then
                            internalerror(2014122806);
                          if sizeleft<>tcgsize2size[destloc.size] then
                            internalerror(200410105);
                          { store everything first to memory, then load it in
                            destloc }
                          tg.gettemp(list,sizeleft,sizeleft,tt_persistent,tempref);
                          gen_alloc_regloc(list,destloc);
                          while sizeleft>0 do
                            begin
                              if not assigned(paraloc) then
                                internalerror(2014122807);
                              unget_para(paraloc^);
                              cg.a_load_cgparaloc_ref(list,paraloc^,tempref,sizeleft,newalignment(para.alignment,para.intsize-sizeleft));
                              if (paraloc^.size=OS_NO) and
                                 assigned(paraloc^.next) then
                                internalerror(2014122805);
                              inc(tempref.offset,tcgsize2size[paraloc^.size]);
                              dec(sizeleft,tcgsize2size[paraloc^.size]);
                              paraloc:=paraloc^.next;
                            end;
                          dec(tempref.offset,para.intsize);
                          cg.a_load_ref_reg(list,para.size,para.size,tempref,destloc.register);
                          tg.ungettemp(list,tempref);
                        end;
                    end
                  else
                    begin
                      unget_para(paraloc^);
                      gen_alloc_regloc(list,destloc);
                      { we can't directly move regular registers into fpu
                        registers }
                      if getregtype(paraloc^.register)=R_FPUREGISTER then
                        begin
                          { store everything first to memory, then load it in
                            destloc }
                          tg.gettemp(list,tcgsize2size[paraloc^.size],para.intsize,tt_persistent,tempref);
                          cg.a_load_cgparaloc_ref(list,paraloc^,tempref,tcgsize2size[paraloc^.size],tempref.alignment);
                          cg.a_load_ref_reg(list,int_cgsize(tcgsize2size[paraloc^.size]),destloc.size,tempref,destloc.register);
                          tg.ungettemp(list,tempref);
                        end
                      else
                        cg.a_load_cgparaloc_anyreg(list,destloc.size,paraloc^,destloc.register,sizeof(aint));
                    end;
                end;
            end;
          LOC_FPUREGISTER,
          LOC_CFPUREGISTER :
            begin
{$ifdef mips}
              if (destloc.size = paraloc^.Size) and
                 (paraloc^.Loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER,LOC_REFERENCE,LOC_CREFERENCE]) then
                begin
                  unget_para(paraloc^);
                  gen_alloc_regloc(list,destloc);
                  cg.a_load_cgparaloc_anyreg(list,destloc.size,paraloc^,destloc.register,para.alignment);
                end
              else if (destloc.size = OS_F32) and
                 (paraloc^.Loc in [LOC_REGISTER,LOC_CREGISTER]) then
                begin
                  gen_alloc_regloc(list,destloc);
                  unget_para(paraloc^);
                  list.Concat(taicpu.op_reg_reg(A_MTC1,paraloc^.register,destloc.register));
                end
{ TODO: Produces invalid code, needs fixing together with regalloc setup. }
{
              else if (destloc.size = OS_F64) and
                      (paraloc^.Loc in [LOC_REGISTER,LOC_CREGISTER]) and
                      (paraloc^.next^.Loc in [LOC_REGISTER,LOC_CREGISTER]) then
                begin
                  gen_alloc_regloc(list,destloc);

                  tmpreg:=destloc.register;
                  unget_para(paraloc^);
                  list.Concat(taicpu.op_reg_reg(A_MTC1,paraloc^.register,tmpreg));
                  setsupreg(tmpreg,getsupreg(tmpreg)+1);
                  unget_para(paraloc^.next^);
                  list.Concat(taicpu.op_reg_reg(A_MTC1,paraloc^.Next^.register,tmpreg));
                end
}
              else
                begin
                  sizeleft := TCGSize2Size[destloc.size];
                  tg.GetTemp(list,sizeleft,sizeleft,tt_normal,tempref);
                  href:=tempref;
                  while assigned(paraloc) do
                    begin
                      unget_para(paraloc^);
                      cg.a_load_cgparaloc_ref(list,paraloc^,href,sizeleft,destloc.reference.alignment);
                      inc(href.offset,TCGSize2Size[paraloc^.size]);
                      dec(sizeleft,TCGSize2Size[paraloc^.size]);
                      paraloc:=paraloc^.next;
                    end;
                  gen_alloc_regloc(list,destloc);
                  cg.a_loadfpu_ref_reg(list,destloc.size,destloc.size,tempref,destloc.register);
                  tg.UnGetTemp(list,tempref);
                end;
{$else mips}
{$if defined(sparc) or defined(arm)}
              { Arm and Sparc passes floats in int registers, when loading to fpu register
                we need a temp }
              sizeleft := TCGSize2Size[destloc.size];
              tg.GetTemp(list,sizeleft,sizeleft,tt_normal,tempref);
              href:=tempref;
              while assigned(paraloc) do
                begin
                  unget_para(paraloc^);
                  cg.a_load_cgparaloc_ref(list,paraloc^,href,sizeleft,destloc.reference.alignment);
                  inc(href.offset,TCGSize2Size[paraloc^.size]);
                  dec(sizeleft,TCGSize2Size[paraloc^.size]);
                  paraloc:=paraloc^.next;
                end;
              gen_alloc_regloc(list,destloc);
              cg.a_loadfpu_ref_reg(list,destloc.size,destloc.size,tempref,destloc.register);
              tg.UnGetTemp(list,tempref);
{$else defined(sparc) or defined(arm)}
              unget_para(paraloc^);
              gen_alloc_regloc(list,destloc);
              { from register to register -> alignment is irrelevant }
              cg.a_load_cgparaloc_anyreg(list,destloc.size,paraloc^,destloc.register,0);
              if assigned(paraloc^.next) then
                internalerror(200410109);
{$endif defined(sparc) or defined(arm)}
{$endif mips}
            end;
          LOC_MMREGISTER,
          LOC_CMMREGISTER :
            begin
{$ifndef cpu64bitalu}
              { ARM vfp floats are passed in integer registers }
              if (para.size=OS_F64) and
                 (paraloc^.size in [OS_32,OS_S32]) and
                 use_vectorfpu(vardef) then
                begin
                  { we need 2x32bit reg }
                  if not assigned(paraloc^.next) or
                     assigned(paraloc^.next^.next) then
                    internalerror(2009112421);
                  unget_para(paraloc^.next^);
                  case paraloc^.next^.loc of
                    LOC_REGISTER:
                      tempreg:=paraloc^.next^.register;
                    LOC_REFERENCE:
                      begin
                        tempreg:=cg.getintregister(list,OS_32);
                        cg.a_load_cgparaloc_anyreg(list,OS_32,paraloc^.next^,tempreg,4);
                      end;
                    else
                      internalerror(2012051301);
                  end;
                  { don't free before the above, because then the getintregister
                    could reallocate this register and overwrite it }
                  unget_para(paraloc^);
                  gen_alloc_regloc(list,destloc);
                  if (target_info.endian=endian_big) then
                    { paraloc^ -> high
                      paraloc^.next -> low }
                    reg64:=joinreg64(tempreg,paraloc^.register)
                  else
                    reg64:=joinreg64(paraloc^.register,tempreg);
                  cg64.a_loadmm_intreg64_reg(list,OS_F64,reg64,destloc.register);
                end
              else
{$endif not cpu64bitalu}
                begin
                  if not assigned(paraloc^.next) then
                    begin
                      unget_para(paraloc^);
                      gen_alloc_regloc(list,destloc);
                      { from register to register -> alignment is irrelevant }
                      cg.a_load_cgparaloc_anyreg(list,destloc.size,paraloc^,destloc.register,0);
                    end
                  else
                    begin
                      internalerror(200410108);
                    end;
                  { data could come in two memory locations, for now
                    we simply ignore the sanity check (FK)
                  if assigned(paraloc^.next) then
                    internalerror(200410108);
                  }
                end;
=======
        hp:=tg.templist;
        while assigned(hp) do
         begin
           if assigned(hp^.def) and
              is_managed_type(hp^.def) then
            begin
=======
        hp:=tg.templist;
        while assigned(hp) do
         begin
           if assigned(hp^.def) and
              is_managed_type(hp^.def) then
            begin
>>>>>>> graemeg/cpstrnew
=======
        hp:=tg.templist;
        while assigned(hp) do
         begin
           if assigned(hp^.def) and
              is_managed_type(hp^.def) then
            begin
>>>>>>> graemeg/cpstrnew
=======
        hp:=tg.templist;
        while assigned(hp) do
         begin
           if assigned(hp^.def) and
              is_managed_type(hp^.def) then
            begin
>>>>>>> origin/cpstrnew
              include(current_procinfo.flags,pi_needs_implicit_finally);
              reference_reset_base(href,current_procinfo.framepointer,hp^.pos,sizeof(pint));
              cg.g_finalize(list,hp^.def,href);
            end;
           hp:=hp^.next;
         end;
      end;


    procedure gen_load_return_value(list:TAsmList);
      var
        ressym : tabstractnormalvarsym;
        funcretloc : TCGPara;
      begin
        { Is the loading needed? }
        if is_void(current_procinfo.procdef.returndef) or
           (
            (po_assembler in current_procinfo.procdef.procoptions) and
            (not(assigned(current_procinfo.procdef.funcretsym)) or
             (tabstractvarsym(current_procinfo.procdef.funcretsym).refs=0))
           ) then
           exit;

        funcretloc:=current_procinfo.procdef.funcretloc[calleeside];

        { constructors return self }
        if (current_procinfo.procdef.proctypeoption=potype_constructor) then
          ressym:=tabstractnormalvarsym(current_procinfo.procdef.parast.Find('self'))
        else
          ressym:=tabstractnormalvarsym(current_procinfo.procdef.funcretsym);
        if (ressym.refs>0) or
           is_managed_type(ressym.vardef) then
          begin
            { was: don't do anything if funcretloc.loc in [LOC_INVALID,LOC_REFERENCE] }
            if not paramanager.ret_in_param(current_procinfo.procdef.returndef,current_procinfo.procdef.proccalloption) then
              gen_load_loc_cgpara(list,ressym.vardef,ressym.localloc,funcretloc);
          end
{$ifdef x86}
         else
          begin
            { the caller will pop a value from the fpu stack }
            if assigned(funcretloc.location) and
               (funcretloc.location^.loc = LOC_FPUREGISTER) then
              list.concat(taicpu.op_none(A_FLDZ));
          end;
{$endif x86}
      end;


    procedure gen_alloc_regloc(list:TAsmList;var loc: tlocation);
      begin
        case loc.loc of
          LOC_CREGISTER:
            begin
{$ifndef cpu64bitalu}
              if loc.size in [OS_64,OS_S64] then
                begin
                  loc.register64.reglo:=cg.getintregister(list,OS_32);
                  loc.register64.reghi:=cg.getintregister(list,OS_32);
                end
              else
{$endif cpu64bitalu}
                loc.register:=cg.getintregister(list,loc.size);
            end;
          LOC_CFPUREGISTER:
            begin
              loc.register:=cg.getfpuregister(list,loc.size);
            end;
          LOC_CMMREGISTER:
            begin
             loc.register:=cg.getmmregister(list,loc.size);
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
          else
            internalerror(2010052903);
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
      end;


=======
      end;


>>>>>>> graemeg/cpstrnew
=======
      end;


>>>>>>> origin/cpstrnew
    procedure gen_alloc_regvar(list:TAsmList;sym: tabstractnormalvarsym; allocreg: boolean);
      begin
        if allocreg then
          gen_alloc_regloc(list,sym.initialloc);
        if (pi_has_label in current_procinfo.flags) then
          begin
            { Allocate register already, to prevent first allocation to be
              inside a loop }
{$ifndef cpu64bitalu}
            if sym.initialloc.size in [OS_64,OS_S64] then
              begin
                cg.a_reg_sync(list,sym.initialloc.register64.reglo);
                cg.a_reg_sync(list,sym.initialloc.register64.reghi);
              end
            else
{$endif not cpu64bitalu}
             cg.a_reg_sync(list,sym.initialloc.register);
          end;
        sym.localloc:=sym.initialloc;
>>>>>>> graemeg/cpstrnew
      end;


    procedure gen_load_cgpara_loc(list: TAsmList; vardef: tdef; const para: TCGPara; var destloc: tlocation; reusepara: boolean);

      procedure unget_para(const paraloc:TCGParaLocation);
        begin
           case paraloc.loc of
             LOC_REGISTER :
               begin
                 if getsupreg(paraloc.register)<first_int_imreg then
                   cg.ungetcpuregister(list,paraloc.register);
               end;
             LOC_MMREGISTER :
               begin
                 if getsupreg(paraloc.register)<first_mm_imreg then
                   cg.ungetcpuregister(list,paraloc.register);
               end;
             LOC_FPUREGISTER :
               begin
                 if getsupreg(paraloc.register)<first_fpu_imreg then
                   cg.ungetcpuregister(list,paraloc.register);
               end;
           end;
        end;

      var
        paraloc  : pcgparalocation;
        href     : treference;
        sizeleft : aint;
{$if defined(sparc) or defined(arm)}
        tempref  : treference;
{$endif sparc}
{$ifndef cpu64bitalu}
        reg64: tregister64;
{$endif not cpu64bitalu}
      begin
        paraloc:=para.location;
        if not assigned(paraloc) then
          internalerror(200408203);
        { skip e.g. empty records }
        if (paraloc^.loc = LOC_VOID) then
          exit;
        case destloc.loc of
          LOC_REFERENCE :
            begin
              { If the parameter location is reused we don't need to copy
                anything }
              if not reusepara then
                begin
                  href:=destloc.reference;
                  sizeleft:=para.intsize;
                  while assigned(paraloc) do
                    begin
                      if (paraloc^.size=OS_NO) then
                        begin
                          { Can only be a reference that contains the rest
                            of the parameter }
                          if (paraloc^.loc<>LOC_REFERENCE) or
                             assigned(paraloc^.next) then
                            internalerror(2005013010);
                          cg.a_load_cgparaloc_ref(list,paraloc^,href,sizeleft,destloc.reference.alignment);
                          inc(href.offset,sizeleft);
                          sizeleft:=0;
                        end
                      else
                        begin
                          cg.a_load_cgparaloc_ref(list,paraloc^,href,tcgsize2size[paraloc^.size],destloc.reference.alignment);
                          inc(href.offset,TCGSize2Size[paraloc^.size]);
                          dec(sizeleft,TCGSize2Size[paraloc^.size]);
                        end;
                      unget_para(paraloc^);
                      paraloc:=paraloc^.next;
                    end;
                end;
            end;
          LOC_REGISTER,
          LOC_CREGISTER :
            begin
{$ifndef cpu64bitalu}
              if (para.size in [OS_64,OS_S64,OS_F64]) and
                 (is_64bit(vardef) or
                  { in case of fpu emulation, or abi's that pass fpu values
                    via integer registers }
                  (vardef.typ=floatdef)) then
                begin
                  case paraloc^.loc of
                    LOC_REGISTER:
                      begin
                        if not assigned(paraloc^.next) then
                          internalerror(200410104);
                        if (target_info.endian=ENDIAN_BIG) then
                          begin
                            { paraloc^ -> high
                              paraloc^.next -> low }
                            unget_para(paraloc^);
                            gen_alloc_regloc(list,destloc);
                            { reg->reg, alignment is irrelevant }
                            cg.a_load_cgparaloc_anyreg(list,OS_32,paraloc^,destloc.register64.reghi,4);
                            unget_para(paraloc^.next^);
                            cg.a_load_cgparaloc_anyreg(list,OS_32,paraloc^.next^,destloc.register64.reglo,4);
                          end
                        else
                          begin
                            { paraloc^ -> low
                              paraloc^.next -> high }
                            unget_para(paraloc^);
                            gen_alloc_regloc(list,destloc);
                            cg.a_load_cgparaloc_anyreg(list,OS_32,paraloc^,destloc.register64.reglo,4);
                            unget_para(paraloc^.next^);
                            cg.a_load_cgparaloc_anyreg(list,OS_32,paraloc^.next^,destloc.register64.reghi,4);
                          end;
                      end;
                    LOC_REFERENCE:
                      begin
                        gen_alloc_regloc(list,destloc);
                        reference_reset_base(href,paraloc^.reference.index,paraloc^.reference.offset,para.alignment);
                        cg64.a_load64_ref_reg(list,href,destloc.register64);
                        unget_para(paraloc^);
                      end;
                    else
                      internalerror(2005101501);
                  end
                end
              else
{$endif not cpu64bitalu}
<<<<<<< HEAD
<<<<<<< HEAD
                begin
                  if assigned(paraloc^.next) then
                    internalerror(200410105);
                  unget_para(paraloc^);
                  gen_alloc_regloc(list,destloc);
                  cg.a_load_cgparaloc_anyreg(list,destloc.size,paraloc^,destloc.register,sizeof(aint));
                end;
            end;
<<<<<<< HEAD
=======
          LOC_FPUREGISTER,
          LOC_CFPUREGISTER :
            begin
{$if defined(sparc) or defined(arm)}
              { Arm and Sparc passes floats in int registers, when loading to fpu register
                we need a temp }
              sizeleft := TCGSize2Size[destloc.size];
              tg.GetTemp(list,sizeleft,sizeleft,tt_normal,tempref);
              href:=tempref;
              while assigned(paraloc) do
                begin
                  unget_para(paraloc^);
                  cg.a_load_cgparaloc_ref(list,paraloc^,href,sizeleft,destloc.reference.alignment);
                  inc(href.offset,TCGSize2Size[paraloc^.size]);
                  dec(sizeleft,TCGSize2Size[paraloc^.size]);
                  paraloc:=paraloc^.next;
=======
                begin
                  if assigned(paraloc^.next) then
                    internalerror(200410105);
                  unget_para(paraloc^);
                  gen_alloc_regloc(list,destloc);
                  cg.a_load_cgparaloc_anyreg(list,destloc.size,paraloc^,destloc.register,sizeof(aint));
>>>>>>> graemeg/cpstrnew
                end;
              gen_alloc_regloc(list,destloc);
              cg.a_loadfpu_ref_reg(list,destloc.size,destloc.size,tempref,destloc.register);
              tg.UnGetTemp(list,tempref);
{$else sparc}
              unget_para(paraloc^);
              gen_alloc_regloc(list,destloc);
              { from register to register -> alignment is irrelevant }
              cg.a_load_cgparaloc_anyreg(list,destloc.size,paraloc^,destloc.register,0);
              if assigned(paraloc^.next) then
                internalerror(200410109);
{$endif sparc}
            end;
<<<<<<< HEAD
          LOC_MMREGISTER,
          LOC_CMMREGISTER :
            begin
{$ifndef cpu64bitalu}
              { ARM vfp floats are passed in integer registers }
              if (para.size=OS_F64) and
                 (paraloc^.size in [OS_32,OS_S32]) and
                 use_vectorfpu(vardef) then
                begin
                  { we need 2x32bit reg }
                  if not assigned(paraloc^.next) or
                     assigned(paraloc^.next^.next) then
                    internalerror(2009112421);
                  unget_para(paraloc^);
                  unget_para(paraloc^.next^);
                  gen_alloc_regloc(list,destloc);
                  if (target_info.endian=endian_big) then
                    { paraloc^ -> high
                      paraloc^.next -> low }
                    reg64:=joinreg64(paraloc^.next^.register,paraloc^.register)
                  else
                    reg64:=joinreg64(paraloc^.register,paraloc^.next^.register);
                  cg64.a_loadmm_intreg64_reg(list,OS_F64,reg64,destloc.register);
                end
              else
{$endif not cpu64bitalu}
                begin
                  unget_para(paraloc^);
                  gen_alloc_regloc(list,destloc);
                  { from register to register -> alignment is irrelevant }
                  cg.a_load_cgparaloc_anyreg(list,destloc.size,paraloc^,destloc.register,0);
                  { data could come in two memory locations, for now
                    we simply ignore the sanity check (FK)
                  if assigned(paraloc^.next) then
                    internalerror(200410108);
                  }
                end;
            end;
          else
            internalerror(2010052903);
        end;
      end;


    procedure gen_load_cgpara_loc(list: TAsmList; vardef: tdef; const para: TCGPara; var destloc: tlocation; reusepara: boolean);

      procedure unget_para(const paraloc:TCGParaLocation);
        begin
           case paraloc.loc of
             LOC_REGISTER :
               begin
                 if getsupreg(paraloc.register)<first_int_imreg then
                   cg.ungetcpuregister(list,paraloc.register);
               end;
             LOC_MMREGISTER :
               begin
                 if getsupreg(paraloc.register)<first_mm_imreg then
                   cg.ungetcpuregister(list,paraloc.register);
               end;
             LOC_FPUREGISTER :
               begin
                 if getsupreg(paraloc.register)<first_fpu_imreg then
                   cg.ungetcpuregister(list,paraloc.register);
               end;
           end;
        end;

      var
        paraloc  : pcgparalocation;
        href     : treference;
        sizeleft : aint;
{$if defined(sparc) or defined(arm)}
        tempref  : treference;
{$endif sparc}
{$ifndef cpu64bitalu}
        reg64: tregister64;
{$endif not cpu64bitalu}
      begin
        paraloc:=para.location;
        if not assigned(paraloc) then
          internalerror(200408203);
        { skip e.g. empty records }
        if (paraloc^.loc = LOC_VOID) then
          exit;
        case destloc.loc of
          LOC_REFERENCE :
            begin
              { If the parameter location is reused we don't need to copy
                anything }
              if not reusepara then
                begin
<<<<<<< HEAD
                  if getsupreg(paraloc.register)<first_int_imreg then
                    cg.getcpuregister(list,paraloc.register);
=======
                begin
                  if assigned(paraloc^.next) then
                    internalerror(200410105);
                  unget_para(paraloc^);
                  gen_alloc_regloc(list,destloc);
                  cg.a_load_cgparaloc_anyreg(list,destloc.size,paraloc^,destloc.register,sizeof(aint));
                end;
            end;
          LOC_FPUREGISTER,
          LOC_CFPUREGISTER :
            begin
{$if defined(sparc) or defined(arm)}
              { Arm and Sparc passes floats in int registers, when loading to fpu register
                we need a temp }
              sizeleft := TCGSize2Size[destloc.size];
              tg.GetTemp(list,sizeleft,sizeleft,tt_normal,tempref);
              href:=tempref;
              while assigned(paraloc) do
                begin
                  unget_para(paraloc^);
                  cg.a_load_cgparaloc_ref(list,paraloc^,href,sizeleft,destloc.reference.alignment);
                  inc(href.offset,TCGSize2Size[paraloc^.size]);
                  dec(sizeleft,TCGSize2Size[paraloc^.size]);
                  paraloc:=paraloc^.next;
>>>>>>> graemeg/cpstrnew
                end;
              gen_alloc_regloc(list,destloc);
              cg.a_loadfpu_ref_reg(list,destloc.size,destloc.size,tempref,destloc.register);
              tg.UnGetTemp(list,tempref);
{$else sparc}
              unget_para(paraloc^);
              gen_alloc_regloc(list,destloc);
              { from register to register -> alignment is irrelevant }
              cg.a_load_cgparaloc_anyreg(list,destloc.size,paraloc^,destloc.register,0);
              if assigned(paraloc^.next) then
                internalerror(200410109);
{$endif sparc}
            end;
          LOC_MMREGISTER,
          LOC_CMMREGISTER :
            begin
{$ifndef cpu64bitalu}
              { ARM vfp floats are passed in integer registers }
              if (para.size=OS_F64) and
                 (paraloc^.size in [OS_32,OS_S32]) and
                 use_vectorfpu(vardef) then
                begin
<<<<<<< HEAD
                  if getsupreg(paraloc.register)<first_mm_imreg then
                    cg.getcpuregister(list,paraloc.register);
                end;
              LOC_FPUREGISTER :
=======
                  href:=destloc.reference;
                  sizeleft:=para.intsize;
                  while assigned(paraloc) do
                    begin
                      if (paraloc^.size=OS_NO) then
                        begin
                          { Can only be a reference that contains the rest
                            of the parameter }
                          if (paraloc^.loc<>LOC_REFERENCE) or
                             assigned(paraloc^.next) then
                            internalerror(2005013010);
                          cg.a_load_cgparaloc_ref(list,paraloc^,href,sizeleft,destloc.reference.alignment);
                          inc(href.offset,sizeleft);
                          sizeleft:=0;
                        end
                      else
                        begin
                          cg.a_load_cgparaloc_ref(list,paraloc^,href,tcgsize2size[paraloc^.size],destloc.reference.alignment);
                          inc(href.offset,TCGSize2Size[paraloc^.size]);
                          dec(sizeleft,TCGSize2Size[paraloc^.size]);
                        end;
                      unget_para(paraloc^);
                      paraloc:=paraloc^.next;
                    end;
                end;
            end;
          LOC_REGISTER,
          LOC_CREGISTER :
            begin
{$ifndef cpu64bitalu}
              if (para.size in [OS_64,OS_S64,OS_F64]) and
                 (is_64bit(vardef) or
                  { in case of fpu emulation, or abi's that pass fpu values
                    via integer registers }
                  (vardef.typ=floatdef)) then
                begin
                  case paraloc^.loc of
                    LOC_REGISTER:
                      begin
                        if not assigned(paraloc^.next) then
                          internalerror(200410104);
                        if (target_info.endian=ENDIAN_BIG) then
                          begin
                            { paraloc^ -> high
                              paraloc^.next -> low }
                            unget_para(paraloc^);
                            gen_alloc_regloc(list,destloc);
                            { reg->reg, alignment is irrelevant }
                            cg.a_load_cgparaloc_anyreg(list,OS_32,paraloc^,destloc.register64.reghi,4);
                            unget_para(paraloc^.next^);
                            cg.a_load_cgparaloc_anyreg(list,OS_32,paraloc^.next^,destloc.register64.reglo,4);
                          end
                        else
                          begin
                            { paraloc^ -> low
                              paraloc^.next -> high }
                            unget_para(paraloc^);
                            gen_alloc_regloc(list,destloc);
                            cg.a_load_cgparaloc_anyreg(list,OS_32,paraloc^,destloc.register64.reglo,4);
                            unget_para(paraloc^.next^);
                            cg.a_load_cgparaloc_anyreg(list,OS_32,paraloc^.next^,destloc.register64.reghi,4);
                          end;
                      end;
                    LOC_REFERENCE:
                      begin
                        gen_alloc_regloc(list,destloc);
                        reference_reset_base(href,paraloc^.reference.index,paraloc^.reference.offset,para.alignment);
                        cg64.a_load64_ref_reg(list,href,destloc.register64);
                        unget_para(paraloc^);
                      end;
                    else
                      internalerror(2005101501);
                  end
                end
              else
{$endif not cpu64bitalu}
>>>>>>> origin/cpstrnew
                begin
                  if assigned(paraloc^.next) then
                    internalerror(200410105);
                  unget_para(paraloc^);
                  gen_alloc_regloc(list,destloc);
                  cg.a_load_cgparaloc_anyreg(list,destloc.size,paraloc^,destloc.register,sizeof(aint));
                end;
            end;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
                  { we need 2x32bit reg }
                  if not assigned(paraloc^.next) or
                     assigned(paraloc^.next^.next) then
                    internalerror(2009112421);
                  unget_para(paraloc^);
                  unget_para(paraloc^.next^);
                  gen_alloc_regloc(list,destloc);
                  if (target_info.endian=endian_big) then
                    { paraloc^ -> high
                      paraloc^.next -> low }
                    reg64:=joinreg64(paraloc^.next^.register,paraloc^.register)
                  else
                    reg64:=joinreg64(paraloc^.register,paraloc^.next^.register);
                  cg64.a_loadmm_intreg64_reg(list,OS_F64,reg64,destloc.register);
                end
              else
{$endif not cpu64bitalu}
                begin
                  unget_para(paraloc^);
                  gen_alloc_regloc(list,destloc);
                  { from register to register -> alignment is irrelevant }
                  cg.a_load_cgparaloc_anyreg(list,destloc.size,paraloc^,destloc.register,0);
                  { data could come in two memory locations, for now
                    we simply ignore the sanity check (FK)
                  if assigned(paraloc^.next) then
                    internalerror(200410108);
                  }
                end;
            end;
=======
          LOC_FPUREGISTER,
          LOC_CFPUREGISTER :
            begin
{$if defined(sparc) or defined(arm)}
              { Arm and Sparc passes floats in int registers, when loading to fpu register
                we need a temp }
              sizeleft := TCGSize2Size[destloc.size];
              tg.GetTemp(list,sizeleft,sizeleft,tt_normal,tempref);
              href:=tempref;
              while assigned(paraloc) do
                begin
                  unget_para(paraloc^);
                  cg.a_load_cgparaloc_ref(list,paraloc^,href,sizeleft,destloc.reference.alignment);
                  inc(href.offset,TCGSize2Size[paraloc^.size]);
                  dec(sizeleft,TCGSize2Size[paraloc^.size]);
                  paraloc:=paraloc^.next;
                end;
              gen_alloc_regloc(list,destloc);
              cg.a_loadfpu_ref_reg(list,destloc.size,destloc.size,tempref,destloc.register);
              tg.UnGetTemp(list,tempref);
{$else sparc}
              unget_para(paraloc^);
              gen_alloc_regloc(list,destloc);
              { from register to register -> alignment is irrelevant }
              cg.a_load_cgparaloc_anyreg(list,destloc.size,paraloc^,destloc.register,0);
              if assigned(paraloc^.next) then
                internalerror(200410109);
{$endif sparc}
            end;
          LOC_MMREGISTER,
          LOC_CMMREGISTER :
            begin
{$ifndef cpu64bitalu}
              { ARM vfp floats are passed in integer registers }
              if (para.size=OS_F64) and
                 (paraloc^.size in [OS_32,OS_S32]) and
                 use_vectorfpu(vardef) then
                begin
                  { we need 2x32bit reg }
                  if not assigned(paraloc^.next) or
                     assigned(paraloc^.next^.next) then
                    internalerror(2009112421);
                  unget_para(paraloc^);
                  unget_para(paraloc^.next^);
                  gen_alloc_regloc(list,destloc);
                  if (target_info.endian=endian_big) then
                    { paraloc^ -> high
                      paraloc^.next -> low }
                    reg64:=joinreg64(paraloc^.next^.register,paraloc^.register)
                  else
                    reg64:=joinreg64(paraloc^.register,paraloc^.next^.register);
                  cg64.a_loadmm_intreg64_reg(list,OS_F64,reg64,destloc.register);
                end
              else
{$endif not cpu64bitalu}
                begin
=======
          LOC_FPUREGISTER,
          LOC_CFPUREGISTER :
            begin
{$if defined(sparc) or defined(arm)}
              { Arm and Sparc passes floats in int registers, when loading to fpu register
                we need a temp }
              sizeleft := TCGSize2Size[destloc.size];
              tg.GetTemp(list,sizeleft,sizeleft,tt_normal,tempref);
              href:=tempref;
              while assigned(paraloc) do
                begin
                  unget_para(paraloc^);
                  cg.a_load_cgparaloc_ref(list,paraloc^,href,sizeleft,destloc.reference.alignment);
                  inc(href.offset,TCGSize2Size[paraloc^.size]);
                  dec(sizeleft,TCGSize2Size[paraloc^.size]);
                  paraloc:=paraloc^.next;
                end;
              gen_alloc_regloc(list,destloc);
              cg.a_loadfpu_ref_reg(list,destloc.size,destloc.size,tempref,destloc.register);
              tg.UnGetTemp(list,tempref);
{$else sparc}
              unget_para(paraloc^);
              gen_alloc_regloc(list,destloc);
              { from register to register -> alignment is irrelevant }
              cg.a_load_cgparaloc_anyreg(list,destloc.size,paraloc^,destloc.register,0);
              if assigned(paraloc^.next) then
                internalerror(200410109);
{$endif sparc}
            end;
          LOC_MMREGISTER,
          LOC_CMMREGISTER :
            begin
{$ifndef cpu64bitalu}
              { ARM vfp floats are passed in integer registers }
              if (para.size=OS_F64) and
                 (paraloc^.size in [OS_32,OS_S32]) and
                 use_vectorfpu(vardef) then
                begin
                  { we need 2x32bit reg }
                  if not assigned(paraloc^.next) or
                     assigned(paraloc^.next^.next) then
                    internalerror(2009112421);
                  unget_para(paraloc^);
                  unget_para(paraloc^.next^);
                  gen_alloc_regloc(list,destloc);
                  if (target_info.endian=endian_big) then
                    { paraloc^ -> high
                      paraloc^.next -> low }
                    reg64:=joinreg64(paraloc^.next^.register,paraloc^.register)
                  else
                    reg64:=joinreg64(paraloc^.register,paraloc^.next^.register);
                  cg64.a_loadmm_intreg64_reg(list,OS_F64,reg64,destloc.register);
                end
              else
{$endif not cpu64bitalu}
                begin
>>>>>>> origin/cpstrnew
                  unget_para(paraloc^);
                  gen_alloc_regloc(list,destloc);
                  { from register to register -> alignment is irrelevant }
                  cg.a_load_cgparaloc_anyreg(list,destloc.size,paraloc^,destloc.register,0);
                  { data could come in two memory locations, for now
                    we simply ignore the sanity check (FK)
                  if assigned(paraloc^.next) then
                    internalerror(200410108);
                  }
                end;
            end;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          else
            internalerror(2010052903);
        end;
      end;
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

    procedure gen_load_para_value(list:TAsmList);

<<<<<<< HEAD
=======
    procedure gen_load_para_value(list:TAsmList);

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
    procedure gen_load_para_value(list:TAsmList);

>>>>>>> origin/cpstrnew
       procedure get_para(const paraloc:TCGParaLocation);
         begin
            case paraloc.loc of
              LOC_REGISTER :
                begin
                  if getsupreg(paraloc.register)<first_int_imreg then
                    cg.getcpuregister(list,paraloc.register);
                end;
              LOC_MMREGISTER :
                begin
                  if getsupreg(paraloc.register)<first_mm_imreg then
                    cg.getcpuregister(list,paraloc.register);
                end;
              LOC_FPUREGISTER :
                begin
                  if getsupreg(paraloc.register)<first_fpu_imreg then
                    cg.getcpuregister(list,paraloc.register);
                end;
            end;
>>>>>>> graemeg/cpstrnew
         end;


      var
        i : longint;
        currpara : tparavarsym;
        paraloc  : pcgparalocation;
      begin
        if (po_assembler in current_procinfo.procdef.procoptions) or
        { exceptfilters have a single hidden 'parentfp' parameter, which
          is handled by tcg.g_proc_entry. }
           (current_procinfo.procdef.proctypeoption=potype_exceptfilter) then
          exit;

        { Allocate registers used by parameters }
        for i:=0 to current_procinfo.procdef.paras.count-1 do
          begin
            currpara:=tparavarsym(current_procinfo.procdef.paras[i]);
            paraloc:=currpara.paraloc[calleeside].location;
            while assigned(paraloc) do
              begin
                if paraloc^.loc in [LOC_REGISTER,LOC_FPUREGISTER,LOC_MMREGISTER] then
                  get_para(paraloc^);
                paraloc:=paraloc^.next;
              end;
          end;
<<<<<<< HEAD
=======

        { Copy parameters to local references/registers }
        for i:=0 to current_procinfo.procdef.paras.count-1 do
          begin
            currpara:=tparavarsym(current_procinfo.procdef.paras[i]);
            gen_load_cgpara_loc(list,currpara.vardef,currpara.paraloc[calleeside],currpara.initialloc,paramanager.param_use_paraloc(currpara.paraloc[calleeside]));
            { gen_load_cgpara_loc() already allocated the initialloc
              -> don't allocate again }
            if currpara.initialloc.loc in [LOC_CREGISTER,LOC_CFPUREGISTER,LOC_CMMREGISTER] then
              gen_alloc_regvar(list,currpara,false);
          end;

        { generate copies of call by value parameters, must be done before
          the initialization and body is parsed because the refcounts are
          incremented using the local copies }
        current_procinfo.procdef.parast.SymList.ForEachCall(@copyvalueparas,list);
{$ifdef powerpc}
        { unget the register that contains the stack pointer before the procedure entry, }
        { which is used to access the parameters in their original callee-side location  }
        if (tppcprocinfo(current_procinfo).needs_frame_pointer) then
          cg.a_reg_dealloc(list,NR_R12);
{$endif powerpc}
{$ifdef powerpc64}
        { unget the register that contains the stack pointer before the procedure entry, }
        { which is used to access the parameters in their original callee-side location  }
        if (tppcprocinfo(current_procinfo).needs_frame_pointer) then
          cg.a_reg_dealloc(list, NR_OLD_STACK_POINTER_REG);
{$endif powerpc64}
        if not(po_assembler in current_procinfo.procdef.procoptions) then
          begin
            { has to be done here rather than in gen_initialize_code, because
              the initialisation code is generated a) later and b) with
              rad_backwards, so the register allocator would generate
              information as if this code comes before loading the parameters
              from their original registers to their local location }
            if (localvartrashing <> -1) then
              current_procinfo.procdef.localst.SymList.ForEachCall(@trash_variable,list);
            { initialize refcounted paras, and trash others. Needed here
              instead of in gen_initialize_code, because when a reference is
              intialised or trashed while the pointer to that reference is kept
              in a regvar, we add a register move and that one again has to
              come after the parameter loading code as far as the register
              allocator is concerned }
            current_procinfo.procdef.parast.SymList.ForEachCall(@init_paras,list);
          end;
      end;


    procedure gen_initialize_code(list:TAsmList);
      begin
        { initialize local data like ansistrings }
        case current_procinfo.procdef.proctypeoption of
           potype_unitinit:
             begin
                { this is also used for initialization of variables in a
                  program which does not have a globalsymtable }
                if assigned(current_module.globalsymtable) then
                  TSymtable(current_module.globalsymtable).SymList.ForEachCall(@initialize_data,list);
                TSymtable(current_module.localsymtable).SymList.ForEachCall(@initialize_data,list);
                TSymtable(current_module.localsymtable).SymList.ForEachCall(@initialize_regvars,list);
             end;
           { units have seperate code for initilization and finalization }
           potype_unitfinalize: ;
           { program init/final is generated in separate procedure }
           potype_proginit:
             begin
               TSymtable(current_module.localsymtable).SymList.ForEachCall(@initialize_regvars,list);
             end;
           else
             current_procinfo.procdef.localst.SymList.ForEachCall(@initialize_data,list);
        end;

        { initialisizes temp. ansi/wide string data }
        inittempvariables(list);

{$ifdef OLDREGVARS}
        load_regvars(list,nil);
{$endif OLDREGVARS}
      end;


    procedure gen_finalize_code(list:TAsmList);
      begin
{$ifdef OLDREGVARS}
        cleanup_regvars(list);
{$endif OLDREGVARS}

        { finalize temporary data }
        finalizetempvariables(list);

        { finalize local data like ansistrings}
        case current_procinfo.procdef.proctypeoption of
           potype_unitfinalize:
             begin
                { this is also used for initialization of variables in a
                  program which does not have a globalsymtable }
                if assigned(current_module.globalsymtable) then
                  TSymtable(current_module.globalsymtable).SymList.ForEachCall(@finalize_static_data,list);
                TSymtable(current_module.localsymtable).SymList.ForEachCall(@finalize_static_data,list);
             end;
           { units/progs have separate code for initialization and finalization }
           potype_unitinit: ;
           { program init/final is generated in separate procedure }
           potype_proginit: ;
           else
             current_procinfo.procdef.localst.SymList.ForEachCall(@finalize_local_vars,list);
        end;

        { finalize paras data }
        if assigned(current_procinfo.procdef.parast) and
           not(po_assembler in current_procinfo.procdef.procoptions) then
          current_procinfo.procdef.parast.SymList.ForEachCall(@final_paras,list);
      end;


    procedure gen_entry_code(list:TAsmList);
      begin
        { the actual profile code can clobber some registers,
          therefore if the context must be saved, do it before
          the actual call to the profile code
        }
        if (cs_profile in current_settings.moduleswitches) and
           not(po_assembler in current_procinfo.procdef.procoptions) then
          begin
            { non-win32 can call mcout even in main }
            if not (target_info.system in [system_i386_win32,system_i386_wdosx]) or
               not (current_procinfo.procdef.proctypeoption=potype_proginit) then
              begin
                cg.g_profilecode(list);
              end;
          end;

        { call startup helpers from main program }
        if (current_procinfo.procdef.proctypeoption=potype_proginit) then
         begin
           { initialize units }
           cg.allocallcpuregisters(list);
           if not(current_module.islibrary) then
             cg.a_call_name(list,'FPC_INITIALIZEUNITS',false)
           else
             cg.a_call_name(list,'FPC_LIBINITIALIZEUNITS',false);
           cg.deallocallcpuregisters(list);
         end;

        list.concat(Tai_force_line.Create);

{$ifdef OLDREGVARS}
        load_regvars(list,nil);
{$endif OLDREGVARS}
      end;
>>>>>>> graemeg/cpstrnew

        { Copy parameters to local references/registers }
        for i:=0 to current_procinfo.procdef.paras.count-1 do
          begin
            currpara:=tparavarsym(current_procinfo.procdef.paras[i]);
            gen_load_cgpara_loc(list,currpara.vardef,currpara.paraloc[calleeside],currpara.initialloc,paramanager.param_use_paraloc(currpara.paraloc[calleeside]));
            { gen_load_cgpara_loc() already allocated the initialloc
              -> don't allocate again }
            if currpara.initialloc.loc in [LOC_CREGISTER,LOC_CFPUREGISTER,LOC_CMMREGISTER] then
              gen_alloc_regvar(list,currpara,false);
          end;

        { generate copies of call by value parameters, must be done before
          the initialization and body is parsed because the refcounts are
          incremented using the local copies }
        current_procinfo.procdef.parast.SymList.ForEachCall(@hlcg.g_copyvalueparas,list);
        if not(po_assembler in current_procinfo.procdef.procoptions) then
          begin
            { initialize refcounted paras, and trash others. Needed here
              instead of in gen_initialize_code, because when a reference is
              intialised or trashed while the pointer to that reference is kept
              in a regvar, we add a register move and that one again has to
              come after the parameter loading code as far as the register
              allocator is concerned }
            current_procinfo.procdef.parast.SymList.ForEachCall(@init_paras,list);
          end;
      end;


{****************************************************************************
                                Entry/Exit
****************************************************************************}

    function has_alias_name(pd:tprocdef;const s:string):boolean;
      var
        item : TCmdStrListItem;
      begin
        result:=true;
        if pd.mangledname=s then
          exit;
        item := TCmdStrListItem(pd.aliasnames.first);
        while assigned(item) do
          begin
            if item.str=s then
              exit;
            item := TCmdStrListItem(item.next);
          end;
        result:=false;
      end;


    procedure alloc_proc_symbol(pd: tprocdef);
      var
        item : TCmdStrListItem;
      begin
        item := TCmdStrListItem(pd.aliasnames.first);
        while assigned(item) do
          begin
<<<<<<< HEAD
            { The condition to use global or local symbol must match
              the code written in hlcg.gen_proc_symbol to 
              avoid change from AB_LOCAL to AB_GLOBAL, which generates
              erroneous code (at least for targets using GOT) } 
=======
            current_asmdata.DefineAsmSymbol(item.str,AB_GLOBAL,AT_FUNCTION);
            item := TCmdStrListItem(item.next);
          end;
       end;


    procedure gen_proc_symbol(list:TAsmList);
      var
        item,
        previtem : TCmdStrListItem;
      begin
        previtem:=nil;
        item := TCmdStrListItem(current_procinfo.procdef.aliasnames.first);
        while assigned(item) do
          begin
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef arm}
            if current_settings.cputype in cpu_thumb2 then
              list.concat(tai_thumb_func.create);
{$endif arm}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            { "double link" all procedure entry symbols via .reference }
            { directives on darwin, because otherwise the linker       }
            { sometimes strips the procedure if only on of the symbols }
            { is referenced                                            }
            if assigned(previtem) and
               (target_info.system in systems_darwin) then
              list.concat(tai_directive.create(asd_reference,item.str));
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            if (cs_profile in current_settings.moduleswitches) or
               (po_global in current_procinfo.procdef.procoptions) then
              current_asmdata.DefineAsmSymbol(item.str,AB_GLOBAL,AT_FUNCTION)
            else
<<<<<<< HEAD
              current_asmdata.DefineAsmSymbol(item.str,AB_LOCAL,AT_FUNCTION);
           item := TCmdStrListItem(item.next);
         end;
=======
              list.concat(Tai_symbol.createname(item.str,AT_FUNCTION,0));
            if assigned(previtem) and
               (target_info.system in systems_darwin) then
              list.concat(tai_directive.create(asd_reference,previtem.str));
<<<<<<< HEAD
<<<<<<< HEAD
            if not(af_stabs_use_function_absolute_addresses in target_asm.flags) then
=======
=======
>>>>>>> origin/fixes_2_2
            if tf_use_function_relative_addresses in target_info.flags then
>>>>>>> graemeg/fixes_2_2
              list.concat(Tai_function_name.create(item.str));
            previtem:=item;
            item := TCmdStrListItem(item.next);
          end;
        current_procinfo.procdef.procstarttai:=tai(list.last);
      end;



    procedure gen_proc_symbol_end(list:TAsmList);
      begin
        list.concat(Tai_symbol_end.Createname(current_procinfo.procdef.mangledname));

        current_procinfo.procdef.procendtai:=tai(list.last);

        if (current_module.islibrary) then
          if (current_procinfo.procdef.proctypeoption = potype_proginit) then
<<<<<<< HEAD
<<<<<<< HEAD
            { setinitname may generate a new section -> don't add to the
              current list, because we assume this remains a text section }
            exportlib.setinitname(current_asmdata.AsmLists[al_exports],current_procinfo.procdef.mangledname);
=======
            exportlib.setinitname(list,current_procinfo.procdef.mangledname);
>>>>>>> graemeg/fixes_2_2
=======
            exportlib.setinitname(list,current_procinfo.procdef.mangledname);
>>>>>>> origin/fixes_2_2

        if (current_procinfo.procdef.proctypeoption=potype_proginit) then
          begin
           if (target_info.system in (systems_darwin+[system_powerpc_macos])) and
              not(current_module.islibrary) then
             begin
              new_section(list,sec_code,'',4);
              list.concat(tai_symbol.createname_global(
                target_info.cprefix+mainaliasname,AT_FUNCTION,0));
              { keep argc, argv and envp properly on the stack }
              cg.a_jmp_name(list,target_info.cprefix+'FPC_SYSTEMMAIN');
             end;
          end;
>>>>>>> graemeg/cpstrnew
      end;


    procedure gen_proc_entry_code(list:TAsmList);
      var
        hitemp,
        lotemp, stack_frame_size : longint;
      begin
        { generate call frame marker for dwarf call frame info }
        current_asmdata.asmcfi.start_frame(list);

        { All temps are know, write offsets used for information }
        if (cs_asm_source in current_settings.globalswitches) and
           (current_procinfo.tempstart<>tg.lasttemp) then
          begin
            if tg.direction>0 then
              begin
                lotemp:=current_procinfo.tempstart;
                hitemp:=tg.lasttemp;
              end
            else
              begin
                lotemp:=tg.lasttemp;
                hitemp:=current_procinfo.tempstart;
              end;
            list.concat(Tai_comment.Create(strpnew('Temps allocated between '+std_regname(current_procinfo.framepointer)+
              tostr_with_plus(lotemp)+' and '+std_regname(current_procinfo.framepointer)+tostr_with_plus(hitemp))));
          end;

         { generate target specific proc entry code }
         stack_frame_size := current_procinfo.calc_stackframe_size;
         if (stack_frame_size <> 0) and
            (po_nostackframe in current_procinfo.procdef.procoptions) then
           message1(parser_e_nostackframe_with_locals,tostr(stack_frame_size));

         hlcg.g_proc_entry(list,stack_frame_size,(po_nostackframe in current_procinfo.procdef.procoptions));
      end;


    procedure gen_proc_exit_code(list:TAsmList);
      var
        parasize : longint;
      begin
        { c style clearstack does not need to remove parameters from the stack, only the
          return value when it was pushed by arguments }
        if current_procinfo.procdef.proccalloption in clearstack_pocalls then
          begin
            parasize:=0;
            { For safecall functions with safecall-exceptions enabled the funcret is always returned as a para
              which is considered a normal para on the c-side, so the funcret has to be pop'ed normally. }
            if not ( (current_procinfo.procdef.proccalloption=pocall_safecall) and
                     (tf_safecall_exceptions in target_info.flags) ) and
                   paramanager.ret_in_param(current_procinfo.procdef.returndef,current_procinfo.procdef) then
              inc(parasize,sizeof(pint));
          end
        else
          begin
            parasize:=current_procinfo.para_stack_size;
            { the parent frame pointer para has to be removed by the caller in
              case of Delphi-style parent frame pointer passing }
            if not paramanager.use_fixed_stack and
               (po_delphi_nested_cc in current_procinfo.procdef.procoptions) then
              dec(parasize,sizeof(pint));
          end;

        { generate target specific proc exit code }
        hlcg.g_proc_exit(list,parasize,(po_nostackframe in current_procinfo.procdef.procoptions));

        { release return registers, needed for optimizer }
        if not is_void(current_procinfo.procdef.returndef) then
          paramanager.freecgpara(list,current_procinfo.procdef.funcretloc[calleeside]);

        { end of frame marker for call frame info }
        current_asmdata.asmcfi.end_frame(list);
      end;


    procedure gen_stack_check_size_para(list:TAsmList);
      var
        paraloc1 : tcgpara;
        pd       : tprocdef;
      begin
        pd:=search_system_proc('fpc_stackcheck');
        paraloc1.init;
<<<<<<< HEAD
        paramanager.getintparaloc(current_asmdata.CurrAsmList,pd,1,paraloc1);
=======
        paramanager.getintparaloc(pocall_default,1,paraloc1);
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
        cg.a_load_const_cgpara(list,OS_INT,current_procinfo.calc_stackframe_size,paraloc1);
        paramanager.freecgpara(list,paraloc1);
        paraloc1.done;
      end;


    procedure gen_stack_check_call(list:TAsmList);
      var
        paraloc1 : tcgpara;
        pd       : tprocdef;
      begin
        pd:=search_system_proc('fpc_stackcheck');
        paraloc1.init;
        { Also alloc the register needed for the parameter }
<<<<<<< HEAD
        paramanager.getintparaloc(current_asmdata.CurrAsmList,pd,1,paraloc1);
=======
        paramanager.getintparaloc(pocall_default,1,paraloc1);
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
        paramanager.freecgpara(list,paraloc1);
        { Call the helper }
        cg.allocallcpuregisters(list);
        cg.a_call_name(list,'FPC_STACKCHECK',false);
        cg.deallocallcpuregisters(list);
        paraloc1.done;
      end;


    procedure gen_save_used_regs(list:TAsmList);
      begin
        { Pure assembler routines need to save the registers themselves }
        if (po_assembler in current_procinfo.procdef.procoptions) then
          exit;

        { oldfpccall expects all registers to be destroyed }
        if current_procinfo.procdef.proccalloption<>pocall_oldfpccall then
            cg.g_save_registers(list);
      end;


    procedure gen_restore_used_regs(list:TAsmList);
      begin
        { Pure assembler routines need to save the registers themselves }
        if (po_assembler in current_procinfo.procdef.procoptions) then
          exit;

        { oldfpccall expects all registers to be destroyed }
        if current_procinfo.procdef.proccalloption<>pocall_oldfpccall then
          cg.g_restore_registers(list);
      end;


{****************************************************************************
<<<<<<< HEAD
                               Const Data
****************************************************************************}

    procedure gen_alloc_symtable(list:TAsmList;pd:tprocdef;st:TSymtable);
=======
                           External handling
****************************************************************************}

    procedure gen_external_stub(list:TAsmList;pd:tprocdef;const externalname:string);
      begin
        create_codegen;
        { add the procedure to the al_procedures }
        maybe_new_object_file(list);
        new_section(list,sec_code,lower(pd.mangledname),current_settings.alignment.procalign);
        list.concat(Tai_align.create(current_settings.alignment.procalign));
        if (po_global in pd.procoptions) then
          list.concat(Tai_symbol.createname_global(pd.mangledname,AT_FUNCTION,0))
        else
          list.concat(Tai_symbol.createname(pd.mangledname,AT_FUNCTION,0));

        cg.g_external_wrapper(list,pd,externalname);
<<<<<<< HEAD
<<<<<<< HEAD
        destroy_codegen;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;

{****************************************************************************
                               Const Data
****************************************************************************}

    procedure insertbssdata(sym : tstaticvarsym);
      var
        l : asizeint;
        varalign : shortint;
        storefilepos : tfileposinfo;
        list : TAsmList;
        sectype : TAsmSectiontype;
      begin
        storefilepos:=current_filepos;
        current_filepos:=sym.fileinfo;
        l:=sym.getsize;
        varalign:=sym.vardef.alignment;
        if (varalign=0) then
          varalign:=var_align_size(l)
        else
          varalign:=var_align(varalign);
        if tf_section_threadvars in target_info.flags then
          begin
            if (vo_is_thread_var in sym.varoptions) then
              begin
                list:=current_asmdata.asmlists[al_threadvars];
                sectype:=sec_threadvar;
              end
            else
              begin
                list:=current_asmdata.asmlists[al_globals];
                sectype:=sec_bss;
              end;
          end
        else
          begin
            if (vo_is_thread_var in sym.varoptions) then
              begin
                inc(l,sizeof(pint));
                { it doesn't help to set a higher alignment, as  }
                { the first sizeof(pint) bytes field will offset }
                { everything anyway                              }
                varalign:=sizeof(pint);
              end;
            list:=current_asmdata.asmlists[al_globals];
            sectype:=sec_bss;
          end;
        maybe_new_object_file(list);
        if sym.section<>'' then
          new_section(list,sec_user,sym.section,varalign)
        else
         new_section(list,sectype,lower(sym.mangledname),varalign);
        if (sym.owner.symtabletype=globalsymtable) or
           create_smartlink or
           DLLSource or
           (assigned(current_procinfo) and
            (po_inline in current_procinfo.procdef.procoptions)) or
           (vo_is_public in sym.varoptions) then
          list.concat(Tai_datablock.create_global(sym.mangledname,l))
        else
          list.concat(Tai_datablock.create(sym.mangledname,l));
        current_filepos:=storefilepos;
      end;


    procedure gen_alloc_symtable(list:TAsmList;st:TSymtable);

        procedure setlocalloc(vs:tabstractnormalvarsym);
        begin
          if cs_asm_source in current_settings.globalswitches then
            begin
              case vs.initialloc.loc of
                LOC_REFERENCE :
                  begin
                    if not assigned(vs.initialloc.reference.symbol) then
                      list.concat(Tai_comment.Create(strpnew('Var '+vs.realname+' located at '+
                         std_regname(vs.initialloc.reference.base)+tostr_with_plus(vs.initialloc.reference.offset))));
                  end;
              end;
            end;
          vs.localloc:=vs.initialloc;
        end;
>>>>>>> graemeg/cpstrnew

      var
        i       : longint;
        highsym,
        sym     : tsym;
        vs      : tabstractnormalvarsym;
        ptrdef  : tdef;
        isaddr  : boolean;
      begin
        for i:=0 to st.SymList.Count-1 do
          begin
            sym:=tsym(st.SymList[i]);
            case sym.typ of
              staticvarsym :
                begin
                  vs:=tabstractnormalvarsym(sym);
                  { The code in loadnode.pass_generatecode will create the
                    LOC_REFERENCE instead for all none register variables. This is
                    required because we can't store an asmsymbol in the localloc because
                    the asmsymbol is invalid after an unit is compiled. This gives
                    problems when this procedure is inlined in another unit (PFV) }
                  if vs.is_regvar(false) then
                    begin
                      vs.initialloc.loc:=tvarregable2tcgloc[vs.varregable];
                      vs.initialloc.size:=def_cgsize(vs.vardef);
                      gen_alloc_regvar(list,vs,true);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                      hlcg.varsym_set_localloc(list,vs);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                      setlocalloc(vs);
>>>>>>> graemeg/cpstrnew
                    end;
                end;
              paravarsym :
                begin
                  vs:=tabstractnormalvarsym(sym);
                  { Parameters passed to assembler procedures need to be kept
                    in the original location }
                  if (po_assembler in pd.procoptions) then
                    tparavarsym(vs).paraloc[calleeside].get_location(vs.initialloc)
                  { exception filters receive their frame pointer as a parameter }
                  else if (pd.proctypeoption=potype_exceptfilter) and
                    (vo_is_parentfp in vs.varoptions) then
                    begin
                      location_reset(vs.initialloc,LOC_REGISTER,OS_ADDR);
                      vs.initialloc.register:=NR_FRAME_POINTER_REG;
                    end
                  else
                    begin
                      { if an open array is used, also its high parameter is used,
                        since the hidden high parameters are inserted after the corresponding symbols,
                        we can increase the ref. count here }
                      if is_open_array(vs.vardef) or is_array_of_const(vs.vardef) then
                        begin
                          highsym:=get_high_value_sym(tparavarsym(vs));
                          if assigned(highsym) then
                            inc(highsym.refs);
                        end;

                      isaddr:=paramanager.push_addr_param(vs.varspez,vs.vardef,pd.proccalloption);
                      if isaddr then
                        vs.initialloc.size:=def_cgsize(voidpointertype)
                      else
                        vs.initialloc.size:=def_cgsize(vs.vardef);

                      if vs.is_regvar(isaddr) then
                        vs.initialloc.loc:=tvarregable2tcgloc[vs.varregable]
                      else
                        begin
                          vs.initialloc.loc:=LOC_REFERENCE;
                          { Reuse the parameter location for values to are at a single location on the stack }
                          if paramanager.param_use_paraloc(tparavarsym(vs).paraloc[calleeside]) then
                            begin
                              hlcg.paravarsym_set_initialloc_to_paraloc(tparavarsym(vs));
                            end
                          else
                            begin
                              if isaddr then
                                begin
                                  ptrdef:=cpointerdef.getreusable(vs.vardef);
                                  tg.GetLocal(list,ptrdef.size,ptrdef,vs.initialloc.reference)
                                end
                              else
                                tg.GetLocal(list,vs.getsize,tparavarsym(vs).paraloc[calleeside].alignment,vs.vardef,vs.initialloc.reference);
                            end;
                        end;
                    end;
                  hlcg.varsym_set_localloc(list,vs);
                end;
              localvarsym :
                begin
                  vs:=tabstractnormalvarsym(sym);
                  vs.initialloc.size:=def_cgsize(vs.vardef);
                  if ([po_assembler,po_nostackframe] * pd.procoptions = [po_assembler,po_nostackframe]) and
                     (vo_is_funcret in vs.varoptions) then
                    begin
                      paramanager.create_funcretloc_info(pd,calleeside);
                      if assigned(pd.funcretloc[calleeside].location^.next) then
                        begin
                          { can't replace references to "result" with a complex
                            location expression inside assembler code }
                          location_reset(vs.initialloc,LOC_INVALID,OS_NO);
                        end
                      else
                        pd.funcretloc[calleeside].get_location(vs.initialloc);
                    end
                  else if (m_delphi in current_settings.modeswitches) and
                     (po_assembler in pd.procoptions) and
                     (vo_is_funcret in vs.varoptions) and
                     (vs.refs=0) then
                    begin
                      { not referenced, so don't allocate. Use dummy to }
                      { avoid ie's later on because of LOC_INVALID      }
                      vs.initialloc.loc:=LOC_REGISTER;
                      vs.initialloc.size:=OS_INT;
                      vs.initialloc.register:=NR_FUNCTION_RESULT_REG;
                    end
                  else if vs.is_regvar(false) then
                    begin
                      vs.initialloc.loc:=tvarregable2tcgloc[vs.varregable];
                      gen_alloc_regvar(list,vs,true);
                    end
                  else
                    begin
                      vs.initialloc.loc:=LOC_REFERENCE;
                      tg.GetLocal(list,vs.getsize,vs.vardef,vs.initialloc.reference);
                    end;
                  hlcg.varsym_set_localloc(list,vs);
                end;
            end;
          end;
      end;


    procedure add_regvars(var rv: tusedregvars; const location: tlocation);
      begin
        case location.loc of
          LOC_CREGISTER:
{$if defined(cpu64bitalu)}
            if location.size in [OS_128,OS_S128] then
              begin
                rv.intregvars.addnodup(getsupreg(location.register128.reglo));
                rv.intregvars.addnodup(getsupreg(location.register128.reghi));
              end
            else
{$elseif defined(cpu32bitalu)}
            if location.size in [OS_64,OS_S64] then
              begin
                rv.intregvars.addnodup(getsupreg(location.register64.reglo));
                rv.intregvars.addnodup(getsupreg(location.register64.reghi));
              end
            else
{$elseif defined(cpu16bitalu)}
            if location.size in [OS_64,OS_S64] then
              begin
                rv.intregvars.addnodup(getsupreg(location.register64.reglo));
                rv.intregvars.addnodup(getsupreg(GetNextReg(location.register64.reglo)));
                rv.intregvars.addnodup(getsupreg(location.register64.reghi));
                rv.intregvars.addnodup(getsupreg(GetNextReg(location.register64.reghi)));
              end
            else
            if location.size in [OS_32,OS_S32] then
              begin
                rv.intregvars.addnodup(getsupreg(location.register));
                rv.intregvars.addnodup(getsupreg(GetNextReg(location.register)));
              end
            else
{$elseif defined(cpu8bitalu)}
            if location.size in [OS_64,OS_S64] then
              begin
                rv.intregvars.addnodup(getsupreg(location.register64.reglo));
                rv.intregvars.addnodup(getsupreg(GetNextReg(location.register64.reglo)));
                rv.intregvars.addnodup(getsupreg(GetNextReg(GetNextReg(location.register64.reglo))));
                rv.intregvars.addnodup(getsupreg(GetNextReg(GetNextReg(GetNextReg(location.register64.reglo)))));
                rv.intregvars.addnodup(getsupreg(location.register64.reghi));
                rv.intregvars.addnodup(getsupreg(GetNextReg(location.register64.reghi)));
                rv.intregvars.addnodup(getsupreg(GetNextReg(GetNextReg(location.register64.reghi))));
                rv.intregvars.addnodup(getsupreg(GetNextReg(GetNextReg(GetNextReg(location.register64.reghi)))));
              end
            else
            if location.size in [OS_32,OS_S32] then
              begin
                rv.intregvars.addnodup(getsupreg(location.register));
                rv.intregvars.addnodup(getsupreg(GetNextReg(location.register)));
                rv.intregvars.addnodup(getsupreg(GetNextReg(GetNextReg(location.register))));
                rv.intregvars.addnodup(getsupreg(GetNextReg(GetNextReg(GetNextReg(location.register)))));
              end
            else
            if location.size in [OS_16,OS_S16] then
              begin
                rv.intregvars.addnodup(getsupreg(location.register));
                rv.intregvars.addnodup(getsupreg(GetNextReg(location.register)));
              end
            else
{$endif}
              rv.intregvars.addnodup(getsupreg(location.register));
          LOC_CFPUREGISTER:
            rv.fpuregvars.addnodup(getsupreg(location.register));
          LOC_CMMREGISTER:
            rv.mmregvars.addnodup(getsupreg(location.register));
        end;
      end;


    function do_get_used_regvars(var n: tnode; arg: pointer): foreachnoderesult;
      var
        rv: pusedregvars absolute arg;
      begin
        case (n.nodetype) of
          temprefn:
            { We only have to synchronise a tempnode before a loop if it is }
            { not created inside the loop, and only synchronise after the   }
            { loop if it's not destroyed inside the loop. If it's created   }
            { before the loop and not yet destroyed, then before the loop   }
            { is secondpassed tempinfo^.valid will be true, and we get the  }
            { correct registers. If it's not destroyed inside the loop,     }
            { then after the loop has been secondpassed tempinfo^.valid     }
            { be true and we also get the right registers. In other cases,  }
            { tempinfo^.valid will be false and so we do not add            }
            { unnecessary registers. This way, we don't have to look at     }
            { tempcreate and tempdestroy nodes to get this info (JM)        }
            if (ti_valid in ttemprefnode(n).tempinfo^.flags) then
              add_regvars(rv^,ttemprefnode(n).tempinfo^.location);
          loadn:
            if (tloadnode(n).symtableentry.typ in [staticvarsym,localvarsym,paravarsym]) then
              add_regvars(rv^,tabstractnormalvarsym(tloadnode(n).symtableentry).localloc);
          vecn:
            { range checks sometimes need the high parameter }
            if (cs_check_range in current_settings.localswitches) and
               (is_open_array(tvecnode(n).left.resultdef) or
                is_array_of_const(tvecnode(n).left.resultdef)) and
               not(current_procinfo.procdef.proccalloption in cdecl_pocalls) then
              add_regvars(rv^,tabstractnormalvarsym(get_high_value_sym(tparavarsym(tloadnode(tvecnode(n).left).symtableentry))).localloc)

        end;
        result := fen_true;
      end;


    procedure get_used_regvars(n: tnode; var rv: tusedregvars);
      begin
        foreachnodestatic(n,@do_get_used_regvars,@rv);
      end;

(*
    See comments at declaration of pusedregvarscommon

    function do_get_used_regvars_common(var n: tnode; arg: pointer): foreachnoderesult;
      var
        rv: pusedregvarscommon absolute arg;
      begin
        if (n.nodetype = loadn) and
           (tloadnode(n).symtableentry.typ in [staticvarsym,localvarsym,paravarsym]) then
          with tabstractnormalvarsym(tloadnode(n).symtableentry).localloc do
            case loc of
              LOC_CREGISTER:
                  { if not yet encountered in this node tree }
                if (rv^.myregvars.intregvars.addnodup(getsupreg(register))) and
                  { but nevertheless already encountered somewhere }
                   not(rv^.allregvars.intregvars.addnodup(getsupreg(register))) then
                  { then it's a regvar used in two or more node trees }
                  rv^.commonregvars.intregvars.addnodup(getsupreg(register));
              LOC_CFPUREGISTER:
                if (rv^.myregvars.intregvars.addnodup(getsupreg(register))) and
                   not(rv^.allregvars.intregvars.addnodup(getsupreg(register))) then
                  rv^.commonregvars.intregvars.addnodup(getsupreg(register));
              LOC_CMMREGISTER:
                if (rv^.myregvars.intregvars.addnodup(getsupreg(register))) and
                   not(rv^.allregvars.intregvars.addnodup(getsupreg(register))) then
                  rv^.commonregvars.intregvars.addnodup(getsupreg(register));
            end;
        result := fen_true;
      end;


    procedure get_used_regvars_common(n: tnode; var rv: tusedregvarscommon);
      begin
        rv.myregvars.intregvars.clear;
        rv.myregvars.fpuregvars.clear;
        rv.myregvars.mmregvars.clear;
        foreachnodestatic(n,@do_get_used_regvars_common,@rv);
      end;
*)

    procedure gen_sync_regvars(list:TAsmList; var rv: tusedregvars);
      var
        count: longint;
      begin
        for count := 1 to rv.intregvars.length do
          cg.a_reg_sync(list,newreg(R_INTREGISTER,rv.intregvars.readidx(count-1),R_SUBWHOLE));
        for count := 1 to rv.fpuregvars.length do
          cg.a_reg_sync(list,newreg(R_FPUREGISTER,rv.fpuregvars.readidx(count-1),R_SUBWHOLE));
        for count := 1 to rv.mmregvars.length do
          cg.a_reg_sync(list,newreg(R_MMREGISTER,rv.mmregvars.readidx(count-1),R_SUBWHOLE));
      end;


<<<<<<< HEAD
=======
{*****************************************************************************
                              SSA support
*****************************************************************************}

    type
      preplaceregrec = ^treplaceregrec;
      treplaceregrec = record
        old, new: tregister;
{$ifndef cpu64bitalu}
        oldhi, newhi: tregister;
{$endif not cpu64bitalu}
        ressym: tsym;
      end;


    function doreplace(var n: tnode; para: pointer): foreachnoderesult;
      var
        rr: preplaceregrec absolute para;
      begin
        result := fen_false;
        if (nf_is_funcret in n.flags) and (fc_exit in flowcontrol) then
          exit;
        case n.nodetype of
          loadn:
            begin
              if (tabstractvarsym(tloadnode(n).symtableentry).varoptions * [vo_is_dll_var, vo_is_thread_var] = []) and
                 not assigned(tloadnode(n).left) and
                 ((tloadnode(n).symtableentry <> rr^.ressym) or
                  not(fc_exit in flowcontrol)
                 ) and
                 (tabstractnormalvarsym(tloadnode(n).symtableentry).localloc.loc in [LOC_CREGISTER,LOC_CFPUREGISTER,LOC_CMMXREGISTER,LOC_CMMREGISTER]) and
                 (tabstractnormalvarsym(tloadnode(n).symtableentry).localloc.register = rr^.old) then
                begin
{$ifndef cpu64bitalu}
                  { it's possible a 64 bit location was shifted and/xor typecasted }
                  { in a 32 bit value, so only 1 register was left in the location }
                  if (tabstractnormalvarsym(tloadnode(n).symtableentry).localloc.size in [OS_64,OS_S64]) then
                    if (tabstractnormalvarsym(tloadnode(n).symtableentry).localloc.register64.reghi = rr^.oldhi) then
                      tabstractnormalvarsym(tloadnode(n).symtableentry).localloc.register64.reghi := rr^.newhi
                    else
                      exit;
{$endif not cpu64bitalu}
                  tabstractnormalvarsym(tloadnode(n).symtableentry).localloc.register := rr^.new;
                  result := fen_norecurse_true;
                end;
            end;
          temprefn:
            begin
              if (ti_valid in ttemprefnode(n).tempinfo^.flags) and
                 (ttemprefnode(n).tempinfo^.location.loc in [LOC_CREGISTER,LOC_CFPUREGISTER,LOC_CMMXREGISTER,LOC_CMMREGISTER]) and
                 (ttemprefnode(n).tempinfo^.location.register = rr^.old) and
                 (not(ti_is_inlined_result in ttemprefnode(n).tempinfo^.flags) or
                  not(fc_exit in flowcontrol)) then
                begin
{$ifndef cpu64bitalu}
                  { it's possible a 64 bit location was shifted and/xor typecasted }
                  { in a 32 bit value, so only 1 register was left in the location }
                  if (ttemprefnode(n).tempinfo^.location.size in [OS_64,OS_S64]) then
                    if (ttemprefnode(n).tempinfo^.location.register64.reghi = rr^.oldhi) then
                      ttemprefnode(n).tempinfo^.location.register64.reghi := rr^.newhi
                    else
                      exit;
{$endif not cpu64bitalu}
                  ttemprefnode(n).tempinfo^.location.register := rr^.new;
                  result := fen_norecurse_true;
                end;
            end;
          { optimize the searching a bit }
          derefn,addrn,
          calln,inlinen,casen,
          addn,subn,muln,
          andn,orn,xorn,
          ltn,lten,gtn,gten,equaln,unequaln,
          slashn,divn,shrn,shln,notn,
          inn,
          asn,isn:
            result := fen_norecurse_false;
        end;
      end;


    procedure maybechangeloadnodereg(list: TAsmList; var n: tnode; reload: boolean);
      var
        rr: treplaceregrec;
      begin
        if not (n.location.loc in [LOC_CREGISTER,LOC_CFPUREGISTER,LOC_CMMXREGISTER,LOC_CMMREGISTER]) or
          ([fc_inflowcontrol,fc_gotolabel,fc_lefthandled] * flowcontrol <> []) then
          exit;
        rr.old := n.location.register;
        rr.ressym := nil;
      {$ifndef cpu64bitalu}
        rr.oldhi := NR_NO;
      {$endif not cpu64bitalu}
        case n.location.loc of
          LOC_CREGISTER:
            begin
      {$ifndef cpu64bitalu}
              if (n.location.size in [OS_64,OS_S64]) then
                begin
                  rr.oldhi := n.location.register64.reghi;
                  rr.new := cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
                  rr.newhi := cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
                end
              else
      {$endif not cpu64bitalu}
                rr.new := cg.getintregister(current_asmdata.CurrAsmList,n.location.size);
            end;
          LOC_CFPUREGISTER:
            rr.new := cg.getfpuregister(current_asmdata.CurrAsmList,n.location.size);
      {$ifdef SUPPORT_MMX}
          LOC_CMMXREGISTER:
            rr.new := tcgx86(cg).getmmxregister(current_asmdata.CurrAsmList);
      {$endif SUPPORT_MMX}
          LOC_CMMREGISTER:
            rr.new := cg.getmmregister(current_asmdata.CurrAsmList,n.location.size);
          else
            exit;
        end;

        if not is_void(current_procinfo.procdef.returndef) and
           assigned(current_procinfo.procdef.funcretsym) and
           (tabstractvarsym(current_procinfo.procdef.funcretsym).refs <> 0) then
          if (current_procinfo.procdef.proctypeoption=potype_constructor) then
            rr.ressym:=tsym(current_procinfo.procdef.parast.Find('self'))
         else
            rr.ressym:=current_procinfo.procdef.funcretsym;

        if not foreachnodestatic(n,@doreplace,@rr) then
          exit;

        if reload then
          case n.location.loc of
            LOC_CREGISTER:
              begin
      {$ifndef cpu64bitalu}
                if (n.location.size in [OS_64,OS_S64]) then
                  cg64.a_load64_reg_reg(list,n.location.register64,joinreg64(rr.new,rr.newhi))
                else
      {$endif not cpu64bitalu}
                  cg.a_load_reg_reg(list,n.location.size,n.location.size,n.location.register,rr.new);
              end;
            LOC_CFPUREGISTER:
              cg.a_loadfpu_reg_reg(list,n.location.size,n.location.size,n.location.register,rr.new);
      {$ifdef SUPPORT_MMX}
            LOC_CMMXREGISTER:
              cg.a_loadmm_reg_reg(list,OS_M64,OS_M64,n.location.register,rr.new,nil);
      {$endif SUPPORT_MMX}
            LOC_CMMREGISTER:
              cg.a_loadmm_reg_reg(list,n.location.size,n.location.size,n.location.register,rr.new,nil);
            else
              internalerror(2006090920);
          end;

        { now that we've change the loadn/temp, also change the node result location }
      {$ifndef cpu64bitalu}
        if (n.location.size in [OS_64,OS_S64]) then
          begin
            n.location.register64.reglo := rr.new;
            n.location.register64.reghi := rr.newhi;
          end
        else
      {$endif not cpu64bitalu}
          n.location.register := rr.new;
      end;


>>>>>>> graemeg/cpstrnew
    procedure gen_free_symtable(list:TAsmList;st:TSymtable);
      var
        i   : longint;
        sym : tsym;
      begin
        for i:=0 to st.SymList.Count-1 do
          begin
            sym:=tsym(st.SymList[i]);
            if (sym.typ in [staticvarsym,localvarsym,paravarsym]) then
              begin
                with tabstractnormalvarsym(sym) do
                  begin
                    { Note: We need to keep the data available in memory
                      for the sub procedures that can access local data
                      in the parent procedures }
                    case localloc.loc of
                      LOC_CREGISTER :
<<<<<<< HEAD
<<<<<<< HEAD
                        if (pi_has_label in current_procinfo.flags) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$if defined(cpu64bitalu)}
                          if def_cgsize(vardef) in [OS_128,OS_S128] then
                            begin
                              cg.a_reg_sync(list,localloc.register128.reglo);
                              cg.a_reg_sync(list,localloc.register128.reghi);
                            end
                          else
{$elseif defined(cpu32bitalu)}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifndef cpu64bitalu}
>>>>>>> graemeg/cpstrnew
                          if def_cgsize(vardef) in [OS_64,OS_S64] then
                            begin
                              cg.a_reg_sync(list,localloc.register64.reglo);
                              cg.a_reg_sync(list,localloc.register64.reghi);
                            end
                          else
{$elseif defined(cpu16bitalu)}
                          if def_cgsize(vardef) in [OS_64,OS_S64] then
                            begin
                              cg.a_reg_sync(list,localloc.register64.reglo);
                              cg.a_reg_sync(list,GetNextReg(localloc.register64.reglo));
                              cg.a_reg_sync(list,localloc.register64.reghi);
                              cg.a_reg_sync(list,GetNextReg(localloc.register64.reghi));
                            end
                          else
                          if def_cgsize(vardef) in [OS_32,OS_S32] then
                            begin
                              cg.a_reg_sync(list,localloc.register);
                              cg.a_reg_sync(list,GetNextReg(localloc.register));
                            end
                          else
{$elseif defined(cpu8bitalu)}
=======
                        if (pi_has_goto in current_procinfo.flags) then
{$ifndef cpu64bit}
>>>>>>> graemeg/fixes_2_2
=======
                        if (pi_has_goto in current_procinfo.flags) then
{$ifndef cpu64bit}
>>>>>>> origin/fixes_2_2
                          if def_cgsize(vardef) in [OS_64,OS_S64] then
                            begin
                              cg.a_reg_sync(list,localloc.register64.reglo);
                              cg.a_reg_sync(list,GetNextReg(localloc.register64.reglo));
                              cg.a_reg_sync(list,GetNextReg(GetNextReg(localloc.register64.reglo)));
                              cg.a_reg_sync(list,GetNextReg(GetNextReg(GetNextReg(localloc.register64.reglo))));
                              cg.a_reg_sync(list,localloc.register64.reghi);
                              cg.a_reg_sync(list,GetNextReg(localloc.register64.reghi));
                              cg.a_reg_sync(list,GetNextReg(GetNextReg(localloc.register64.reghi)));
                              cg.a_reg_sync(list,GetNextReg(GetNextReg(GetNextReg(localloc.register64.reghi))));
                            end
                          else
                          if def_cgsize(vardef) in [OS_32,OS_S32] then
                            begin
                              cg.a_reg_sync(list,localloc.register);
                              cg.a_reg_sync(list,GetNextReg(localloc.register));
                              cg.a_reg_sync(list,GetNextReg(GetNextReg(localloc.register)));
                              cg.a_reg_sync(list,GetNextReg(GetNextReg(GetNextReg(localloc.register))));
                            end
                          else
                          if def_cgsize(vardef) in [OS_16,OS_S16] then
                            begin
                              cg.a_reg_sync(list,localloc.register);
                              cg.a_reg_sync(list,GetNextReg(localloc.register));
                            end
                          else
{$endif}
                            cg.a_reg_sync(list,localloc.register);
                      LOC_CFPUREGISTER,
                      LOC_CMMREGISTER:
                        if (pi_has_label in current_procinfo.flags) then
                          cg.a_reg_sync(list,localloc.register);
                      LOC_REFERENCE :
                        begin
                          if typ in [localvarsym,paravarsym] then
                            tg.Ungetlocal(list,localloc.reference);
                        end;
                    end;
                  end;
              end;
          end;
      end;


<<<<<<< HEAD
=======
    procedure gen_intf_wrapper(list:TAsmList;_class:tobjectdef);
      var
        i,j  : longint;
        tmps : string;
        pd   : TProcdef;
        ImplIntf : TImplementedInterface;
      begin
        for i:=0 to _class.ImplementedInterfaces.count-1 do
          begin
            ImplIntf:=TImplementedInterface(_class.ImplementedInterfaces[i]);
            if (ImplIntf=ImplIntf.VtblImplIntf) and
               assigned(ImplIntf.ProcDefs) then
              begin
                maybe_new_object_file(list);
                for j:=0 to ImplIntf.ProcDefs.Count-1 do
                  begin
                    pd:=TProcdef(ImplIntf.ProcDefs[j]);
                    tmps:=make_mangledname('WRPR',_class.owner,_class.objname^+'_$_'+
                      ImplIntf.IntfDef.objname^+'_$_'+tostr(j)+'_$_'+pd.mangledname);
                    { create wrapper code }
                    new_section(list,sec_code,tmps,0);
                    cg.init_register_allocators;
                    cg.g_intf_wrapper(list,pd,tmps,ImplIntf.ioffset);
                    cg.done_register_allocators;
                  end;
              end;
          end;
      end;


    procedure gen_intf_wrappers(list:TAsmList;st:TSymtable;nested:boolean);
      var
        i   : longint;
        def : tdef;
      begin
        if not nested then
          create_codegen;
        for i:=0 to st.DefList.Count-1 do
          begin
            def:=tdef(st.DefList[i]);
            { if def can contain nested types then handle it symtable }
            if def.typ in [objectdef,recorddef] then
              gen_intf_wrappers(list,tabstractrecorddef(def).symtable,true);
            if is_class(def) then
              gen_intf_wrapper(list,tobjectdef(def));
          end;
        if not nested then
          destroy_codegen;
      end;


    procedure gen_load_vmt_register(list:TAsmList;objdef:tobjectdef;selfloc:tlocation;var vmtreg:tregister);
      var
        href : treference;
      begin
        if is_object(objdef) then
          begin
            case selfloc.loc of
              LOC_CREFERENCE,
              LOC_REFERENCE:
                begin
                  reference_reset_base(href,cg.getaddressregister(list),objdef.vmt_offset,sizeof(pint));
                  cg.a_loadaddr_ref_reg(list,selfloc.reference,href.base);
                end;
              else
                internalerror(200305056);
            end;
          end
        else
          { This is also valid for Objective-C classes: vmt_offset is 0 there,
            and the first "field" of an Objective-C class instance is a pointer
            to its "meta-class".  }
          begin
            case selfloc.loc of
              LOC_REGISTER:
                begin
{$ifdef cpu_uses_separate_address_registers}
                  if getregtype(left.location.register)<>R_ADDRESSREGISTER then
                    begin
                      reference_reset_base(href,cg.getaddressregister(list),objdef.vmt_offset,sizeof(pint));
                      cg.a_load_reg_reg(list,OS_ADDR,OS_ADDR,selfloc.register,href.base);
                    end
                  else
{$endif cpu_uses_separate_address_registers}
                    reference_reset_base(href,selfloc.register,objdef.vmt_offset,sizeof(pint));
                end;
              LOC_CREGISTER,
              LOC_CREFERENCE,
              LOC_REFERENCE:
                begin
<<<<<<< HEAD
<<<<<<< HEAD
                  reference_reset_base(href,cg.getaddressregister(list),objdef.vmt_offset,sizeof(pint));
=======
                  reference_reset_base(href,cg.getaddressregister(list),objdef.vmt_offset);
>>>>>>> graemeg/fixes_2_2
=======
                  reference_reset_base(href,cg.getaddressregister(list),objdef.vmt_offset);
>>>>>>> origin/fixes_2_2
                  cg.a_load_loc_reg(list,OS_ADDR,selfloc,href.base);
                end;
              else
                internalerror(200305057);
            end;
          end;
        vmtreg:=cg.getaddressregister(list);
        cg.g_maybe_testself(list,href.base);
        cg.a_load_ref_reg(list,OS_ADDR,OS_ADDR,href,vmtreg);

        { test validity of VMT }
        if not(is_interface(objdef)) and
           not(is_cppclass(objdef)) and
           not(is_objc_class_or_protocol(objdef)) then
           cg.g_maybe_testvmt(list,vmtreg,objdef);
      end;


>>>>>>> graemeg/cpstrnew
    function getprocalign : shortint;
      begin
        { gprof uses 16 byte granularity }
        if (cs_profile in current_settings.moduleswitches) then
          result:=16
        else
         result:=current_settings.alignment.procalign;
      end;


<<<<<<< HEAD
    procedure gen_fpc_dummy(list : TAsmList);
      begin
{$ifdef i386}
        { fix me! }
        list.concat(Taicpu.Op_const_reg(A_MOV,S_L,1,NR_EAX));
        list.concat(Taicpu.Op_const(A_RET,S_W,12));
{$endif i386}
      end;


<<<<<<< HEAD
    procedure gen_load_frame_for_exceptfilter(list : TAsmList);
      var
        para: tparavarsym;
      begin
        para:=tparavarsym(current_procinfo.procdef.paras[0]);
        if not (vo_is_parentfp in para.varoptions) then
          InternalError(201201142);
        if (para.paraloc[calleeside].location^.loc<>LOC_REGISTER) or
          (para.paraloc[calleeside].location^.next<>nil) then
          InternalError(201201143);
        cg.a_load_reg_reg(list,OS_ADDR,OS_ADDR,para.paraloc[calleeside].location^.register,
          NR_FRAME_POINTER_REG);
      end;

=======
    procedure gen_fpc_dummy(list : TAsmList);
      begin
=======
    procedure gen_pic_helpers(list : TAsmList);
{$ifdef i386}
      var
        href : treference;
{$endif i386}
      begin
        { if other cpus require such helpers as well, it can be solved more cleanly }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef i386}
        { fix me! }
        list.concat(Taicpu.Op_const_reg(A_MOV,S_L,1,NR_EAX));
        list.concat(Taicpu.Op_const(A_RET,S_W,12));
{$endif i386}
      end;


    procedure InsertInterruptTable;

      procedure WriteVector(const name: string);
        var
          ai: taicpu;
        begin
{$IFDEF arm}
          if current_settings.cputype in [cpu_armv7m, cpu_cortexm3] then
            current_asmdata.asmlists[al_globals].concat(tai_const.Createname(name,0))
          else
            begin
              ai:=taicpu.op_sym(A_B,current_asmdata.RefAsmSymbol(name));
              ai.is_jmp:=true;
              current_asmdata.asmlists[al_globals].concat(ai);
            end;
{$ENDIF arm}
        end;

      function GetInterruptTableLength: longint;
        begin
{$if defined(ARM)}
          result:=interruptvectors[current_settings.controllertype];
{$else}
          result:=0;
{$endif}
        end;

      var
        hp: tused_unit;
        sym: tsym;
        i, i2: longint;
        interruptTable: array of tprocdef;
        pd: tprocdef;
      begin
        SetLength(interruptTable, GetInterruptTableLength);
        FillChar(interruptTable[0], length(interruptTable)*sizeof(pointer), 0);

        hp:=tused_unit(usedunits.first);
        while assigned(hp) do
          begin
            for i := 0 to hp.u.symlist.Count-1 do
              begin
                sym:=tsym(hp.u.symlist[i]);
                if not assigned(sym) then
                  continue;
                if sym.typ = procsym then
                  begin
                    for i2 := 0 to tprocsym(sym).ProcdefList.Count-1 do
                      begin
                        pd:=tprocdef(tprocsym(sym).ProcdefList[i2]);
                        if pd.interruptvector >= 0 then
                          begin
                            if pd.interruptvector > high(interruptTable) then
                              Internalerror(2011030602);
                            if interruptTable[pd.interruptvector] <> nil then
                              internalerror(2011030601);

                            interruptTable[pd.interruptvector]:=pd;
                            break;
                          end;
                      end;
                  end;
              end;
            hp:=tused_unit(hp.next);
          end;

        new_section(current_asmdata.asmlists[al_globals],sec_init,'VECTORS',sizeof(pint));
        current_asmdata.asmlists[al_globals].concat(Tai_symbol.Createname_global('VECTORS',AT_DATA,0));
{$IFDEF arm}
        if current_settings.cputype in [cpu_armv7m, cpu_cortexm3] then
          current_asmdata.asmlists[al_globals].concat(tai_const.Createname('_stack_top',0)); { ARMv7-M processors have the initial stack value at address 0 }
{$ENDIF arm}

        for i:=0 to high(interruptTable) do
          begin
            if interruptTable[i]<>nil then
              writeVector(interruptTable[i].mangledname)
            else
              writeVector('DefaultHandler'); { Default handler name }
          end;
      end;


<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
    procedure gen_fpc_dummy(list : TAsmList);
      begin
{$ifdef i386}
        { fix me! }
        list.concat(Taicpu.Op_const_reg(A_MOV,S_L,1,NR_EAX));
        list.concat(Taicpu.Op_const(A_RET,S_W,12));
{$endif i386}
      end;

<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
=======
    procedure gen_fpc_dummy(list : TAsmList);
      begin
{$ifdef i386}
        { fix me! }
        list.concat(Taicpu.Op_const_reg(A_MOV,S_L,1,NR_EAX));
        list.concat(Taicpu.Op_const(A_RET,S_W,12));
{$endif i386}
      end;


<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
    procedure gen_fpc_dummy(list : TAsmList);
      begin
{$ifdef i386}
        { fix me! }
        list.concat(Taicpu.Op_const_reg(A_MOV,S_L,1,NR_EAX));
        list.concat(Taicpu.Op_const(A_RET,S_W,12));
{$endif i386}
      end;


>>>>>>> origin/cpstrnew
    procedure InsertInterruptTable;

      procedure WriteVector(const name: string);
        var
          ai: taicpu;
        begin
{$IFDEF arm}
          if current_settings.cputype in [cpu_armv7m, cpu_cortexm3] then
            current_asmdata.asmlists[al_globals].concat(tai_const.Createname(name,0))
          else
            begin
              ai:=taicpu.op_sym(A_B,current_asmdata.RefAsmSymbol(name));
              ai.is_jmp:=true;
              current_asmdata.asmlists[al_globals].concat(ai);
            end;
{$ENDIF arm}
        end;

      function GetInterruptTableLength: longint;
        begin
{$if defined(ARM)}
          result:=interruptvectors[current_settings.controllertype];
{$else}
          result:=0;
{$endif}
        end;

      var
        hp: tused_unit;
        sym: tsym;
        i, i2: longint;
        interruptTable: array of tprocdef;
        pd: tprocdef;
      begin
        SetLength(interruptTable, GetInterruptTableLength);
        FillChar(interruptTable[0], length(interruptTable)*sizeof(pointer), 0);

        hp:=tused_unit(usedunits.first);
        while assigned(hp) do
          begin
            for i := 0 to hp.u.symlist.Count-1 do
              begin
                sym:=tsym(hp.u.symlist[i]);
                if not assigned(sym) then
                  continue;
                if sym.typ = procsym then
                  begin
                    for i2 := 0 to tprocsym(sym).ProcdefList.Count-1 do
                      begin
                        pd:=tprocdef(tprocsym(sym).ProcdefList[i2]);
                        if pd.interruptvector >= 0 then
                          begin
                            if pd.interruptvector > high(interruptTable) then
                              Internalerror(2011030602);
                            if interruptTable[pd.interruptvector] <> nil then
                              internalerror(2011030601);

                            interruptTable[pd.interruptvector]:=pd;
                            break;
                          end;
                      end;
                  end;
              end;
            hp:=tused_unit(hp.next);
          end;

        new_section(current_asmdata.asmlists[al_globals],sec_init,'VECTORS',sizeof(pint));
        current_asmdata.asmlists[al_globals].concat(Tai_symbol.Createname_global('VECTORS',AT_DATA,0));
{$IFDEF arm}
        if current_settings.cputype in [cpu_armv7m, cpu_cortexm3] then
          current_asmdata.asmlists[al_globals].concat(tai_const.Createname('_stack_top',0)); { ARMv7-M processors have the initial stack value at address 0 }
{$ENDIF arm}

        for i:=0 to high(interruptTable) do
          begin
            if interruptTable[i]<>nil then
              writeVector(interruptTable[i].mangledname)
            else
              writeVector('DefaultHandler'); { Default handler name }
          end;
      end;


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.
