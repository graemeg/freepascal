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
    procedure maketojumpbool(list:TAsmList; p : tnode; loadregvars: tloadregvars);
//    procedure remove_non_regvars_from_loc(const t: tlocation; var regs:Tsuperregisterset);

    procedure location_force_fpureg(list:TAsmList;var l: tlocation;maybeconst:boolean);
    procedure location_force_mmregscalar(list:TAsmList;var l: tlocation;maybeconst:boolean);
    procedure location_force_mmreg(list:TAsmList;var l: tlocation;maybeconst:boolean);
    procedure location_allocate_register(list:TAsmList;out l: tlocation;def: tdef;constant: boolean);

    { loads a cgpara into a tlocation; assumes that loc.loc is already
      initialised }
    procedure gen_load_cgpara_loc(list: TAsmList; vardef: tdef; const para: TCGPara; var destloc: tlocation; reusepara: boolean);

    { allocate registers for a tlocation; assumes that loc.loc is already
      set to LOC_CREGISTER/LOC_CFPUREGISTER/... }
    procedure gen_alloc_regloc(list:TAsmList;var loc: tlocation);

    procedure register_maybe_adjust_setbase(list: TAsmList; var l: tlocation; setbase: aint);


    function  has_alias_name(pd:tprocdef;const s:string):boolean;
    procedure alloc_proc_symbol(pd: tprocdef);
    procedure gen_proc_symbol(list:TAsmList);
    procedure gen_proc_entry_code(list:TAsmList);
    procedure gen_proc_exit_code(list:TAsmList);
    procedure gen_stack_check_size_para(list:TAsmList);
    procedure gen_stack_check_call(list:TAsmList);
    procedure gen_save_used_regs(list:TAsmList);
    procedure gen_restore_used_regs(list:TAsmList);
    procedure gen_load_para_value(list:TAsmList);

    procedure gen_external_stub(list:TAsmList;pd:tprocdef;const externalname:string);
    procedure gen_intf_wrappers(list:TAsmList;st:TSymtable;nested:boolean);
    procedure gen_load_vmt_register(list:TAsmList;objdef:tobjectdef;selfloc:tlocation;var vmtreg:tregister);

    procedure get_used_regvars(n: tnode; var rv: tusedregvars);
    { adds the regvars used in n and its children to rv.allregvars,
      those which were already in rv.allregvars to rv.commonregvars and
      uses rv.myregvars as scratch (so that two uses of the same regvar
      in a single tree to make it appear in commonregvars). Useful to
      find out which regvars are used in two different node trees
      (e.g. in the "else" and "then" path, or in various case blocks }
//    procedure get_used_regvars_common(n: tnode; var rv: tusedregvarscommon);
    procedure gen_sync_regvars(list:TAsmList; var rv: tusedregvars);

    { if the result of n is a LOC_C(..)REGISTER, try to find the corresponding }
    { loadn and change its location to a new register (= SSA). In case reload  }
    { is true, transfer the old to the new register                            }
    procedure maybechangeloadnodereg(list: TAsmList; var n: tnode; reload: boolean);

   {#
      Allocate the buffers for exception management and setjmp environment.
      Return a pointer to these buffers, send them to the utility routine
      so they are registered, and then call setjmp.

      Then compare the result of setjmp with 0, and if not equal
      to zero, then jump to exceptlabel.

      Also store the result of setjmp to a temporary space by calling g_save_exception_reason

      It is to note that this routine may be called *after* the stackframe of a
      routine has been called, therefore on machines where the stack cannot
      be modified, all temps should be allocated on the heap instead of the
      stack.
    }

    const

      EXCEPT_BUF_SIZE = 3*sizeof(pint);
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

    procedure gen_alloc_symtable(list:TAsmList;st:TSymtable);
    procedure gen_free_symtable(list:TAsmList;st:TSymtable);

    procedure location_free(list: TAsmList; const location : TLocation);

    function getprocalign : shortint;

    procedure gen_fpc_dummy(list : TAsmList);

    procedure InsertInterruptTable;

implementation

  uses
    version,
    cutils,cclasses,
    globals,systems,verbose,export,
    ppu,defutil,
    procinfo,paramgr,fmodule,
    regvars,dbgbase,
    pass_1,pass_2,
    nbas,ncon,nld,nmem,nutils,ngenutil,
    tgobj,cgobj,cgcpu,hlcgobj,hlcgcpu
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
{$endif}
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


    procedure maketojumpbool(list:TAsmList; p : tnode; loadregvars: tloadregvars);
    {
      produces jumps to true respectively false labels using boolean expressions

      depending on whether the loading of regvars is currently being
      synchronized manually (such as in an if-node) or automatically (most of
      the other cases where this procedure is called), loadregvars can be
      "lr_load_regvars" or "lr_dont_load_regvars"
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
{$ifdef OLDREGVARS}
              if loadregvars = lr_load_regvars then
                load_all_regvars(list);
{$endif OLDREGVARS}
              if is_constboolnode(p) then
                begin
                   if Tordconstnode(p).value.uvalue<>0 then
                     cg.a_jmp_always(list,current_procinfo.CurrTrueLabel)
                   else
                     cg.a_jmp_always(list,current_procinfo.CurrFalseLabel)
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
                         cg.a_cmp_const_reg_label(list,OS_INT,OC_NE,0,tmpreg,current_procinfo.CurrTrueLabel);
                         cg.a_jmp_always(list,current_procinfo.CurrFalseLabel);
                       end;
                     LOC_CREGISTER,LOC_REGISTER,LOC_CREFERENCE,LOC_REFERENCE :
                       begin
{$ifndef cpu64bitalu}
                         if opsize in [OS_64,OS_S64] then
                           begin
                             hlcg.location_force_reg(list,p.location,p.resultdef,hlcg.tcgsize2orddef(opsize),true);
                             tmpreg:=cg.getintregister(list,OS_32);
                             cg.a_op_reg_reg_reg(list,OP_OR,OS_32,p.location.register64.reglo,p.location.register64.reghi,tmpreg);
                             location_reset(p.location,LOC_REGISTER,OS_32);
                             p.location.register:=tmpreg;
                             opsize:=OS_32;
                           end;
{$endif not cpu64bitalu}
                         cg.a_cmp_const_loc_label(list,opsize,OC_NE,0,p.location,current_procinfo.CurrTrueLabel);
                         cg.a_jmp_always(list,current_procinfo.CurrFalseLabel);
                       end;
                     LOC_JUMP:
                       ;
{$ifdef cpuflags}
                     LOC_FLAGS :
                       begin
                         cg.a_jmp_flags(list,p.location.resflags,current_procinfo.CurrTrueLabel);
                         cg.a_jmp_always(list,current_procinfo.CurrFalseLabel);
                       end;
{$endif cpuflags}
                     else
                       begin
                         printnode(output,p);
                         internalerror(200308241);
                       end;
                   end;
                end;
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
      var
        srsym : ttypesym;
      begin
        if jmp_buf_size=-1 then
          begin
            srsym:=search_system_type('JMP_BUF');
            jmp_buf_size:=srsym.typedef.size;
            jmp_buf_align:=srsym.typedef.alignment;
          end;
        tg.GetTemp(list,EXCEPT_BUF_SIZE,sizeof(pint),tt_persistent,t.envbuf);
        tg.GetTemp(list,jmp_buf_size,jmp_buf_align,tt_persistent,t.jmpbuf);
        tg.GetTemp(list,sizeof(pint),sizeof(pint),tt_persistent,t.reasonbuf);
      end;


    procedure unget_exception_temps(list:TAsmList;const t:texceptiontemps);
      begin
        tg.Ungettemp(list,t.jmpbuf);
        tg.ungettemp(list,t.envbuf);
        tg.ungettemp(list,t.reasonbuf);
      end;


    procedure new_exception(list:TAsmList;const t:texceptiontemps;exceptlabel:tasmlabel);
      var
        paraloc1,paraloc2,paraloc3 : tcgpara;
      begin
        paraloc1.init;
        paraloc2.init;
        paraloc3.init;
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
        paraloc1.done;
        paraloc2.done;
        paraloc3.done;
     end;


    procedure free_exception(list:TAsmList;const t:texceptiontemps;a:aint;endexceptlabel:tasmlabel;onlyfree:boolean);
     begin
         cg.allocallcpuregisters(list);
         cg.a_call_name(list,'FPC_POPADDRSTACK',false);
         cg.deallocallcpuregisters(list);

         if not onlyfree then
          begin
            { g_exception_reason_load already allocates NR_FUNCTION_RESULT_REG }
            cg.g_exception_reason_load(list, t.reasonbuf);
            cg.a_cmp_const_reg_label(list,OS_INT,OC_EQ,a,NR_FUNCTION_RESULT_REG,endexceptlabel);
            cg.a_reg_dealloc(list,NR_FUNCTION_RESULT_REG);
          end;
     end;


{*****************************************************************************
                                     TLocation
*****************************************************************************}


    procedure location_force_fpureg(list:TAsmList;var l: tlocation;maybeconst:boolean);
      var
        reg : tregister;
        href : treference;
      begin
        if (l.loc<>LOC_FPUREGISTER)  and
           ((l.loc<>LOC_CFPUREGISTER) or (not maybeconst)) then
          begin
            { if it's in an mm register, store to memory first }
            if (l.loc in [LOC_MMREGISTER,LOC_CMMREGISTER]) then
              begin
                tg.GetTemp(list,tcgsize2size[l.size],tcgsize2size[l.size],tt_normal,href);
                cg.a_loadmm_reg_ref(list,l.size,l.size,l.register,href,mms_movescalar);
                location_reset_ref(l,LOC_REFERENCE,l.size,0);
                l.reference:=href;
              end;
            reg:=cg.getfpuregister(list,l.size);
            cg.a_loadfpu_loc_reg(list,l.size,l,reg);
            location_freetemp(list,l);
            location_reset(l,LOC_FPUREGISTER,l.size);
            l.register:=reg;
          end;
      end;


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
                 hlcg.a_loadmm_loc_reg(list,l.size,newsize,l,reg,mms_movescalar);
                 l.size:=newsize;
               end;
            location_freetemp(list,l);
            location_reset(l,LOC_MMREGISTER,l.size);
            l.register:=reg;
          end;
      end;


    procedure register_maybe_adjust_setbase(list: TAsmList; var l: tlocation; setbase: aint);
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
                  tmpreg := cg.getintregister(list,l.size);
                  cg.a_op_const_reg_reg(list,OP_SUB,l.size,setbase,l.register,tmpreg);
                  l.loc:=LOC_REGISTER;
                  l.register:=tmpreg;
                end;
              LOC_REGISTER:
                begin
                  cg.a_op_const_reg(list,OP_SUB,l.size,setbase,l.register);
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
            hlcg.a_loadmm_loc_reg(list,l.size,OS_VECTOR,l,reg,nil);
            location_freetemp(list,l);
            location_reset(l,LOC_MMREGISTER,OS_VECTOR);
            l.register:=reg;
          end;
      end;


    procedure location_allocate_register(list: TAsmList;out l: tlocation;def: tdef;constant: boolean);
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
        sizedef : tdef;
      begin
        list:=TAsmList(arg);
        if (tsym(p).typ=paravarsym) and
           (tparavarsym(p).varspez=vs_value) and
          (paramanager.push_addr_param(tparavarsym(p).varspez,tparavarsym(p).vardef,current_procinfo.procdef.proccalloption)) then
          begin
            { we have no idea about the alignment at the caller side }
            hlcg.location_get_data_ref(list,tparavarsym(p).vardef,tparavarsym(p).initialloc,href,true,1);
            if is_open_array(tparavarsym(p).vardef) or
               is_array_of_const(tparavarsym(p).vardef) then
              begin
                { cdecl functions don't have a high pointer so it is not possible to generate
                  a local copy }
                if not(current_procinfo.procdef.proccalloption in cdecl_pocalls) then
                  begin
                    hsym:=tparavarsym(get_high_value_sym(tparavarsym(p)));
                    if not assigned(hsym) then
                      internalerror(200306061);
                    hreg:=cg.getaddressregister(list);
                    if not is_packed_array(tparavarsym(p).vardef) then
                      cg.g_copyvaluepara_openarray(list,href,hsym.initialloc,tarraydef(tparavarsym(p).vardef).elesize,hreg)
                    else
                      internalerror(2006080401);
//                      cg.g_copyvaluepara_packedopenarray(list,href,hsym.intialloc,tarraydef(tparavarsym(p).vardef).elepackedbitsize,hreg);
                    sizedef:=getpointerdef(tparavarsym(p).vardef);
                    hlcg.a_load_reg_loc(list,sizedef,sizedef,hreg,tparavarsym(p).initialloc);
                  end;
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
          end;
      end;


    { generates the code for incrementing the reference count of parameters and
      initialize out parameters }
    procedure init_paras(p:TObject;arg:pointer);
      var
        href : treference;
        hsym : tparavarsym;
        eldef : tdef;
        list : TAsmList;
        needs_inittable : boolean;
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
                     hlcg.location_get_data_ref(list,tparavarsym(p).vardef,tparavarsym(p).initialloc,href,is_open_array(tparavarsym(p).vardef),sizeof(pint));
                     if is_open_array(tparavarsym(p).vardef) then
                       begin
                         { open arrays do not contain correct element count in their rtti,
                           the actual count must be passed separately. }
                         hsym:=tparavarsym(get_high_value_sym(tparavarsym(p)));
                         eldef:=tarraydef(tparavarsym(p).vardef).elementdef;
                         if not assigned(hsym) then
                           internalerror(201003031);
                         cg.g_array_rtti_helper(list,eldef,href,hsym.initialloc,'FPC_ADDREF_ARRAY');
                       end
                     else
                      cg.g_incrrefcount(list,tparavarsym(p).vardef,href);
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
                     cg.g_array_rtti_helper(list,eldef,href,hsym.initialloc,'FPC_INITIALIZE_ARRAY');
                   end
                 else
                   cg.g_initialize(list,tparavarsym(p).vardef,href);
               end;
           end;
         end;
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
            end;
        end;
      end;


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
{$if defined(sparc) or defined(arm) or defined(mips)}
        tempref  : treference;
{$endif defined(sparc) or defined(arm) or defined(mips)}
{$ifdef mips}
        tmpreg   : tregister;
{$endif mips}
{$ifndef cpu64bitalu}
        tempreg  : tregister;
        reg64    : tregister64;
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
{$ifdef mips}
              if (destloc.size = paraloc^.Size) and
                 (paraloc^.Loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER]) then
                begin
                  gen_alloc_regloc(list,destloc);
                  cg.a_loadfpu_reg_reg(list,paraloc^.Size, destloc.size, paraloc^.register, destloc.register);
                end
              else if (destloc.size = OS_F32) and
                 (paraloc^.Loc in [LOC_REGISTER,LOC_CREGISTER]) then
                begin
                  gen_alloc_regloc(list,destloc);
                  list.Concat(taicpu.op_reg_reg(A_MTC1,paraloc^.register,destloc.register));
                end
              else if (destloc.size = OS_F64) and
                      (paraloc^.Loc in [LOC_REGISTER,LOC_CREGISTER]) and
                      (paraloc^.next^.Loc in [LOC_REGISTER,LOC_CREGISTER]) then
                begin
                  gen_alloc_regloc(list,destloc);

                  tmpreg:=destloc.register;
                  list.Concat(taicpu.op_reg_reg(A_MTC1,paraloc^.register,tmpreg));
                  setsupreg(tmpreg,getsupreg(tmpreg)+1);
                  list.Concat(taicpu.op_reg_reg(A_MTC1,paraloc^.Next^.register,tmpreg));
                end
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


    procedure gen_load_para_value(list:TAsmList);

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
{$ifdef arm}
            if current_settings.cputype in cpu_thumb2 then
              list.concat(tai_thumb_func.create);
{$endif arm}
            { "double link" all procedure entry symbols via .reference }
            { directives on darwin, because otherwise the linker       }
            { sometimes strips the procedure if only on of the symbols }
            { is referenced                                            }
            if assigned(previtem) and
               (target_info.system in systems_darwin) then
              list.concat(tai_directive.create(asd_reference,item.str));
            if (cs_profile in current_settings.moduleswitches) or
              (po_global in current_procinfo.procdef.procoptions) then
              list.concat(Tai_symbol.createname_global(item.str,AT_FUNCTION,0))
            else
              list.concat(Tai_symbol.createname(item.str,AT_FUNCTION,0));
            if assigned(previtem) and
               (target_info.system in systems_darwin) then
              list.concat(tai_directive.create(asd_reference,previtem.str));
            if not(af_stabs_use_function_absolute_addresses in target_asm.flags) then
              list.concat(Tai_function_name.create(item.str));
            previtem:=item;
            item := TCmdStrListItem(item.next);
          end;
        current_procinfo.procdef.procstarttai:=tai(list.last);
      end;



    procedure gen_proc_entry_code(list:TAsmList);
      var
        hitemp,
        lotemp : longint;
      begin
        { generate call frame marker for dwarf call frame info }
        current_asmdata.asmcfi.start_frame(list);

        { All temps are know, write offsets used for information }
        if (cs_asm_source in current_settings.globalswitches) then
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
         hlcg.g_proc_entry(list,current_procinfo.calc_stackframe_size,(po_nostackframe in current_procinfo.procdef.procoptions));
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
            if paramanager.ret_in_param(current_procinfo.procdef.returndef,current_procinfo.procdef.proccalloption) then
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
        paraloc1   : tcgpara;
      begin
        paraloc1.init;
        paramanager.getintparaloc(pocall_default,1,paraloc1);
        cg.a_load_const_cgpara(list,OS_INT,current_procinfo.calc_stackframe_size,paraloc1);
        paramanager.freecgpara(list,paraloc1);
        paraloc1.done;
      end;


    procedure gen_stack_check_call(list:TAsmList);
      var
        paraloc1   : tcgpara;
      begin
        paraloc1.init;
        { Also alloc the register needed for the parameter }
        paramanager.getintparaloc(pocall_default,1,paraloc1);
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
                           External handling
****************************************************************************}

    procedure gen_external_stub(list:TAsmList;pd:tprocdef;const externalname:string);
      begin
        create_hlcodegen;
        { add the procedure to the al_procedures }
        maybe_new_object_file(list);
        new_section(list,sec_code,lower(pd.mangledname),current_settings.alignment.procalign);
        list.concat(Tai_align.create(current_settings.alignment.procalign));
        if (po_global in pd.procoptions) then
          list.concat(Tai_symbol.createname_global(pd.mangledname,AT_FUNCTION,0))
        else
          list.concat(Tai_symbol.createname(pd.mangledname,AT_FUNCTION,0));

        cg.g_external_wrapper(list,pd,externalname);
        destroy_hlcodegen;
      end;

{****************************************************************************
                               Const Data
****************************************************************************}

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

      var
        i       : longint;
        sym     : tsym;
        vs      : tabstractnormalvarsym;
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
                      setlocalloc(vs);
                    end;
                end;
              paravarsym :
                begin
                  vs:=tabstractnormalvarsym(sym);
                  { Parameters passed to assembler procedures need to be kept
                    in the original location }
                  if (po_assembler in current_procinfo.procdef.procoptions) then
                    tparavarsym(vs).paraloc[calleeside].get_location(vs.initialloc)
                  { exception filters receive their frame pointer as a parameter }
                  else if (current_procinfo.procdef.proctypeoption=potype_exceptfilter) and
                    (vo_is_parentfp in vs.varoptions) then
                    begin
                      location_reset(vs.initialloc,LOC_REGISTER,OS_ADDR);
                      vs.initialloc.register:=NR_FRAME_POINTER_REG;
                    end
                  else
                    begin
                      isaddr:=paramanager.push_addr_param(vs.varspez,vs.vardef,current_procinfo.procdef.proccalloption);
                      if isaddr then
                        vs.initialloc.size:=OS_ADDR
                      else
                        vs.initialloc.size:=def_cgsize(vs.vardef);

                      if vs.is_regvar(isaddr) then
                        vs.initialloc.loc:=tvarregable2tcgloc[vs.varregable]
                      else
                        begin
                          vs.initialloc.loc:=LOC_REFERENCE;
                          { Reuse the parameter location for values to are at a single location on the stack }
                          if paramanager.param_use_paraloc(tparavarsym(sym).paraloc[calleeside]) then
                            begin
                              reference_reset_base(vs.initialloc.reference,tparavarsym(sym).paraloc[calleeside].location^.reference.index,
                                  tparavarsym(sym).paraloc[calleeside].location^.reference.offset,tparavarsym(sym).paraloc[calleeside].alignment);
                            end
                          else
                            begin
                              if isaddr then
                                tg.GetLocal(list,sizeof(pint),voidpointertype,vs.initialloc.reference)
                              else
                                tg.GetLocal(list,vs.getsize,tparavarsym(sym).paraloc[calleeside].alignment,vs.vardef,vs.initialloc.reference);
                            end;
                        end;
                    end;
                  setlocalloc(vs);
                end;
              localvarsym :
                begin
                  vs:=tabstractnormalvarsym(sym);
                  vs.initialloc.size:=def_cgsize(vs.vardef);
                  if (m_delphi in current_settings.modeswitches) and
                     (po_assembler in current_procinfo.procdef.procoptions) and
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
                  setlocalloc(vs);
                end;
            end;
          end;
      end;


    procedure add_regvars(var rv: tusedregvars; const location: tlocation);
      begin
        case location.loc of
          LOC_CREGISTER:
{$ifndef cpu64bitalu}
            if location.size in [OS_64,OS_S64] then
              begin
                rv.intregvars.addnodup(getsupreg(location.register64.reglo));
                rv.intregvars.addnodup(getsupreg(location.register64.reghi));
              end
            else
{$endif not cpu64bitalu}
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
        { moved sym }
        sym : tsym;
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
                  rr^.sym := tabstractnormalvarsym(tloadnode(n).symtableentry);
                  result := fen_norecurse_true;
                end;
            end;
          temprefn:
            begin
              if (ti_valid in ttemprefnode(n).tempinfo^.flags) and
                 (ttemprefnode(n).tempinfo^.location.loc in [LOC_CREGISTER,LOC_CFPUREGISTER,LOC_CMMXREGISTER,LOC_CMMREGISTER]) and
                 (ttemprefnode(n).tempinfo^.location.register = rr^.old) then
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
{$ifdef jvm}
        exit;
{$endif}
        if not (n.location.loc in [LOC_CREGISTER,LOC_CFPUREGISTER,LOC_CMMXREGISTER,LOC_CMMREGISTER]) or
          ([fc_inflowcontrol,fc_gotolabel,fc_lefthandled] * flowcontrol <> []) then
          exit;
        rr.old := n.location.register;
        rr.ressym := nil;
        rr.sym := nil;
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
            if assigned(rr.sym) then
              list.concat(tai_varloc.create64(rr.sym,rr.new,rr.newhi));
          end
        else
      {$endif not cpu64bitalu}
          begin
            n.location.register := rr.new;
            if assigned(rr.sym) then
              list.concat(tai_varloc.create(rr.sym,rr.new));
          end;
      end;


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
                        if (pi_has_label in current_procinfo.flags) then
{$ifndef cpu64bitalu}
                          if def_cgsize(vardef) in [OS_64,OS_S64] then
                            begin
                              cg.a_reg_sync(list,localloc.register64.reglo);
                              cg.a_reg_sync(list,localloc.register64.reghi);
                            end
                          else
{$endif not cpu64bitalu}
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
                    { we don't track method calls via interfaces yet ->
                      assume that every method called via an interface call
                      is reachable for now }
                    if (po_virtualmethod in pd.procoptions) and
                       not is_objectpascal_helper(tprocdef(pd).struct) then
                      tobjectdef(tprocdef(pd).struct).register_vmt_call(tprocdef(pd).extnumber);
                    tmps:=make_mangledname('WRPR',_class.owner,_class.objname^+'_$_'+
                      ImplIntf.IntfDef.objname^+'_$_'+tostr(j)+'_$_'+pd.mangledname);
                    { create wrapper code }
                    new_section(list,sec_code,tmps,0);
                    hlcg.init_register_allocators;
                    cg.g_intf_wrapper(list,pd,tmps,ImplIntf.ioffset);
                    hlcg.done_register_allocators;
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
          create_hlcodegen;
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
          destroy_hlcodegen;
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
              LOC_CONSTANT,
              LOC_CREGISTER,
              LOC_CREFERENCE,
              LOC_REFERENCE:
                begin
                  reference_reset_base(href,cg.getaddressregister(list),objdef.vmt_offset,sizeof(pint));
                  { todo: pass actual vmt pointer type to hlcg }
                  hlcg.a_load_loc_reg(list,voidpointertype,voidpointertype,selfloc,href.base);
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


    function getprocalign : shortint;
      begin
        { gprof uses 16 byte granularity }
        if (cs_profile in current_settings.moduleswitches) then
          result:=16
        else
         result:=current_settings.alignment.procalign;
      end;


    procedure gen_fpc_dummy(list : TAsmList);
      begin
{$ifdef i386}
        { fix me! }
        list.concat(Taicpu.Op_const_reg(A_MOV,S_L,1,NR_EAX));
        list.concat(Taicpu.Op_const(A_RET,S_W,12));
{$endif i386}
      end;


    procedure InsertInterruptTable;

      procedure WriteVector(const name: string);
{$IFDEF arm}
        var
          ai: taicpu;
{$ENDIF arm}
        begin
{$IFDEF arm}
          if current_settings.cputype in [cpu_armv7m] then
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
          result:=embedded_controllers[current_settings.controllertype].interruptvectors;
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
            try
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
            except
            end;  
            hp:=tused_unit(hp.next);
          end;

        new_section(current_asmdata.asmlists[al_globals],sec_init,'VECTORS',sizeof(pint));
        current_asmdata.asmlists[al_globals].concat(Tai_symbol.Createname_global('VECTORS',AT_DATA,0));
{$IFDEF arm}
        if current_settings.cputype in [cpu_armv7m] then
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


end.
