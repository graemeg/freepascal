{
    Copyright (c) 2010, 2013 by Jonas Maebe
    Member of the Free Pascal development team

    This unit implements the LLVM high level code generator

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
unit hlcgllvm;

{$i fpcdefs.inc}

interface

uses
  globtype,cclasses,
  aasmbase,aasmdata,
  symbase,symconst,symtype,symdef,symsym,
  cpubase, hlcgobj, cgbase, cgutils, parabase, tgobj;

  type

    { thlcgllvm }

    thlcgllvm = class(thlcgobj)
      constructor create;

      procedure a_load_ref_cgpara(list: TAsmList; size: tdef; const r: treference; const cgpara: TCGPara); override;
      procedure a_load_const_cgpara(list: TAsmList; tosize: tdef; a: tcgint; const cgpara: TCGPara); override;
     protected
       procedure a_load_ref_cgpara_init_src(list: TAsmList; const para: tcgpara; const initialref: treference; var refsize: tdef; out newref: treference);
     public
      procedure getcpuregister(list: TAsmList; r: Tregister); override;
      procedure ungetcpuregister(list: TAsmList; r: Tregister); override;
      procedure alloccpuregisters(list: TAsmList; rt: Tregistertype; const r: Tcpuregisterset); override;
      procedure allocallcpuregisters(list: TAsmList); override;
      procedure deallocallcpuregisters(list: TAsmList); override;

      procedure a_bit_test_reg_reg_reg(list: TAsmList; bitnumbersize, valuesize, destsize: tdef; bitnumber, value, destreg: tregister); override;
      procedure a_bit_set_reg_reg(list: TAsmList; doset: boolean; bitnumbersize, destsize: tdef; bitnumber, dest: tregister); override;

     protected
      procedure a_call_common(list: TAsmList; pd: tabstractprocdef; const paras: array of pcgpara; const forceresdef: tdef; out res: tregister; out hlretdef: tdef; out llvmretdef: tdef; out callparas: tfplist);
     public
      function a_call_name(list : TAsmList;pd : tprocdef;const s : TSymStr; const paras: array of pcgpara; forceresdef: tdef; weak: boolean): tcgpara;override;
      function a_call_reg(list: TAsmList; pd: tabstractprocdef; reg: tregister; const paras: array of pcgpara): tcgpara; override;

      procedure a_load_const_reg(list : TAsmList;tosize : tdef;a : tcgint;register : tregister);override;
      procedure a_load_const_ref(list: TAsmList; tosize: tdef; a: tcgint; const ref: treference);override;
      procedure a_load_reg_ref(list : TAsmList;fromsize, tosize : tdef;register : tregister;const ref : treference);override;
      procedure a_load_reg_reg(list : TAsmList;fromsize, tosize : tdef;reg1,reg2 : tregister);override;

     protected
      procedure gen_load_refaddrfull_anyreg(list: TAsmList; fromsize, tosize : tdef; const simpleref: treference; register: tregister; shuffle: pmmshuffle);
      function  handle_agg_load_ref_anyreg(list: TasmList; var fromsize, tosize: tdef; var simpleref: treference; register: tregister; shuffle: pmmshuffle): boolean;
     public
      procedure a_load_ref_reg(list : TAsmList;fromsize, tosize : tdef;const ref : treference;register : tregister);override;
      procedure a_load_ref_ref(list: TAsmList; fromsize, tosize: tdef; const sref: treference; const dref: treference); override;
     protected
      procedure a_loadaddr_ref_reg_intern(list : TAsmList;fromsize, tosize : tdef;const ref : treference;r : tregister; makefromsizepointer: boolean);
     public
      procedure a_loadaddr_ref_reg(list : TAsmList;fromsize, tosize : tdef;const ref : treference;r : tregister);override;

      procedure a_op_const_reg(list: TAsmList; Op: TOpCG; size: tdef; a: tcgint; reg: TRegister); override;
      procedure a_op_const_reg_reg(list: TAsmList; op: TOpCg; size: tdef; a: tcgint; src, dst: tregister); override;

      procedure a_op_reg_reg_reg(list: TAsmList; op: TOpCg; size: tdef; src1, src2, dst: tregister); override;
      procedure a_op_reg_reg(list: TAsmList; Op: TOpCG; size: tdef; reg1, reg2: TRegister); override;
      procedure a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tdef; a: tcgint; src, dst: tregister;setflags : boolean;var ovloc : tlocation); override;
      procedure a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tdef; src1, src2, dst: tregister;setflags : boolean;var ovloc : tlocation); override;

      procedure a_cmp_const_reg_label(list: TAsmList; size: tdef; cmp_op: topcmp; a: tcgint; reg: tregister; l: tasmlabel); override;
      procedure a_cmp_reg_reg_label(list: TAsmList; size: tdef; cmp_op: topcmp; reg1, reg2: tregister; l: tasmlabel); override;

      procedure a_jmp_always(list : TAsmList;l: tasmlabel); override;

      procedure g_unreachable(list: TAsmList); override;

      procedure g_concatcopy(list : TAsmList;size: tdef; const source,dest : treference);override;

      procedure a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tdef; const ref: treference; reg: tregister); override;
      procedure a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tdef; reg: tregister; const ref: treference); override;
      procedure a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize: tdef; reg1, reg2: tregister); override;

      procedure gen_proc_symbol(list: TAsmList); override;
      procedure handle_external_proc(list: TAsmList; pd: tprocdef; const importname: TSymStr); override;
      procedure g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean); override;
      procedure g_proc_exit(list : TAsmList;parasize:longint;nostackframe:boolean); override;
     protected
      procedure gen_load_uninitialized_function_result(list: TAsmList; pd: tprocdef; resdef: tdef; const resloc: tcgpara); override;
     public
      procedure g_overflowcheck(list: TAsmList; const Loc: tlocation; def: tdef); override;
      procedure g_overflowCheck_loc(List:TAsmList;const Loc:TLocation;def:TDef;var ovloc : tlocation); override;

      procedure g_ptrtypecast_reg(list: TAsmList; fromdef, todef: tdef; var reg: tregister); override;
      procedure g_ptrtypecast_ref(list: TAsmList; fromdef, todef: tdef; var ref: treference); override;

      procedure g_set_addr_nonbitpacked_field_ref(list: TAsmList; recdef: tabstractrecorddef; field: tfieldvarsym; var recref: treference); override;

      procedure a_loadmm_ref_reg(list: TAsmList; fromsize, tosize: tdef; const ref: treference; reg: tregister; shuffle: pmmshuffle); override;
      procedure a_loadmm_reg_ref(list: TAsmList; fromsize, tosize: tdef; reg: tregister; const ref: treference; shuffle: pmmshuffle); override;
      procedure a_loadmm_reg_reg(list: TAsmList; fromsize, tosize: tdef; reg1, reg2: tregister; shuffle: pmmshuffle); override;
      procedure a_opmm_reg_reg(list: TAsmList; Op: TOpCG; size: tdef; src, dst: tregister; shuffle: pmmshuffle); override;
      procedure a_loadmm_intreg_reg(list: TAsmList; fromsize, tosize: tdef; intreg, mmreg: tregister; shuffle: pmmshuffle); override;
      procedure a_loadmm_reg_intreg(list: TAsmList; fromsize, tosize: tdef; mmreg, intreg: tregister; shuffle: pmmshuffle); override;

      function get_call_result_cgpara(pd: tabstractprocdef; forceresdef: tdef): tcgpara; override;
     protected
      procedure gen_load_loc_function_result(list: TAsmList; vardef: tdef; const l: tlocation); override;
     public
      procedure gen_load_loc_cgpara(list: TAsmList; vardef: tdef; const l: tlocation; const cgpara: tcgpara); override;
      procedure gen_load_cgpara_loc(list: TAsmList; vardef: tdef; const para: TCGPara; var destloc: tlocation; reusepara: boolean); override;
{$ifdef cpuflags}
      { llvm doesn't have flags, but cpuflags is defined in case the real cpu
        has flags and we have to override the abstract methods to prevent
        warnings }
      procedure a_jmp_flags(list: TAsmList; const f: TResFlags; l: tasmlabel); override;
      procedure g_flags2reg(list: TAsmList; size: tdef; const f: tresflags; reg: TRegister); override;
      procedure g_flags2ref(list: TAsmList; size: tdef; const f: tresflags; const ref: TReference); override;
{$endif cpuflags}

      { unimplemented or unnecessary routines }
      procedure a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; srcsize, dstsize: tdef; src, dst: tregister); override;
      procedure g_stackpointer_alloc(list: TAsmList; size: longint); override;
      procedure g_intf_wrapper(list: TAsmList; procdef: tprocdef; const labelname: string; ioffset: longint); override;
      procedure g_adjust_self_value(list: TAsmList; procdef: tprocdef; ioffset: aint); override;
      procedure g_local_unwind(list: TAsmList; l: TAsmLabel); override;
      procedure gen_stack_check_size_para(list: TAsmList); override;
      procedure gen_stack_check_call(list: TAsmList); override;

      procedure varsym_set_localloc(list: TAsmList; vs: tabstractnormalvarsym); override;
      procedure paravarsym_set_initialloc_to_paraloc(vs: tparavarsym); override;

      procedure g_external_wrapper(list: TAsmList; procdef: tprocdef; const wrappername, externalname: string; global: boolean); override;

     { def is a pointerdef or implicit pointer type (class, classref, procvar,
       dynamic array, ...).  }
     function make_simple_ref_ptr(list: TAsmList; const ref: treference; ptrdef: tdef): treference;
      { def is the type of the data stored in memory pointed to by ref, not
        a pointer to this type }
      function make_simple_ref(list: TAsmList; const ref: treference; def: tdef): treference;
    protected
      procedure paraloctoloc(const paraloc: pcgparalocation; out hloc: tlocation);
      procedure set_call_function_result(const list: TAsmList; const pd: tabstractprocdef; const llvmretdef, hlretdef: tdef; const resval: tregister; var retpara: tcgpara);
    end;

implementation

  uses
    verbose,cutils,globals,fmodule,constexp,systems,
    defutil,llvmdef,llvmsym,
    aasmtai,aasmcpu,
    aasmllvm,llvmbase,llvminfo,tgllvm,
    symtable,symllvm,
    paramgr,
    pass_2,procinfo,llvmpi,cpuinfo,cgobj,cgllvm,cghlcpu,
    cgcpu,hlcgcpu;

  var
    create_hlcodegen_cpu: TProcedure = nil;

  const
    topcg2llvmop: array[topcg] of tllvmop =
     { OP_NONE  OP_MOVE     OP_ADD  OP_AND  OP_DIV   OP_IDIV  OP_IMUL OP_MUL }
      (la_none, la_none, la_add, la_and, la_udiv, la_sdiv, la_mul, la_mul,
     { OP_NEG   OP_NOT   OP_OR  OP_SAR   OP_SHL  OP_SHR   OP_SUB  OP_XOR }
       la_none, la_none, la_or, la_ashr, la_shl, la_lshr, la_sub, la_xor,
     { OP_ROL   OP_ROR }
       la_none, la_none);


  constructor thlcgllvm.create;
    begin
      inherited
    end;


  procedure thlcgllvm.a_load_ref_cgpara(list: TAsmList; size: tdef; const r: treference; const cgpara: TCGPara);
    var
      tmpref, initialref, ref: treference;
      fielddef,
      orgsize: tdef;
      location: pcgparalocation;
      sizeleft,
      totaloffset: asizeint;
      paralocidx: longint;
      userecord: boolean;
    begin
      location:=cgpara.location;
      sizeleft:=cgpara.intsize;
      totaloffset:=0;
      orgsize:=size;
      a_load_ref_cgpara_init_src(list,cgpara,r,size,initialref);
      if initialref.refaddr=addr_full then
        begin
          cgpara.check_simple_location;
          location^.llvmvalueloc:=true;
          location^.llvmloc.loc:=LOC_CREFERENCE;
          location^.llvmloc.sym:=initialref.symbol;
          exit;
        end;
      userecord:=
        (orgsize<>size) and
        assigned(cgpara.location^.next);
      paralocidx:=0;
      fielddef:=nil;
      while assigned(location) do
        begin
          if userecord then
            begin
              { llvmparadef is a record in this case, with every field
                corresponding to a single paraloc (fielddef is unused, because
                it will be equivalent to location^.def -- see below) }
              g_setup_load_field_by_name(list,trecorddef(size),'F'+tostr(paralocidx),initialref,tmpref,fielddef);
            end
          else
            tmpref:=initialref;
          paramanager.allocparaloc(list,location);
          case location^.loc of
            LOC_REGISTER,LOC_CREGISTER:
              begin
                 { byval parameter -> load the address rather than the value }
                 if not location^.llvmvalueloc then
                   a_loadaddr_ref_reg(list,tpointerdef(location^.def).pointeddef,location^.def,tmpref,location^.register)
                 { if this parameter is split into multiple paralocs via
                   record fields, load the current paraloc. The type of the
                   paraloc and of the current record field will match by
                   construction (the record is build from the paraloc
                   types) }
                 else if userecord then
                   a_load_ref_reg(list,fielddef,location^.def,tmpref,location^.register)
                 { if the parameter is passed in a single paraloc, the
                   paraloc's type may be different from the declared type
                   -> use the original complete parameter size as source so
                   we can insert a type conversion if necessary }
                 else
                   a_load_ref_reg(list,size,location^.def,tmpref,location^.register)
             end;
            LOC_REFERENCE,LOC_CREFERENCE:
              begin
                 if assigned(location^.next) then
                   internalerror(2010052906);
                 reference_reset_base(ref,cpointerdef.getreusable(size),location^.reference.index,location^.reference.offset,ctempposinvalid,newalignment(cgpara.alignment,cgpara.intsize-sizeleft),[]);
                 if (def_cgsize(size)<>OS_NO) and
                    (size.size=sizeleft) and
                    (sizeleft<=sizeof(aint)) then
                   a_load_ref_ref(list,size,location^.def,tmpref,ref)
                 else
                   { use concatcopy, because the parameter can be larger than }
                   { what the OS_* constants can handle                       }
                   g_concatcopy(list,location^.def,tmpref,ref);
              end;
            LOC_MMREGISTER,LOC_CMMREGISTER:
              begin
                 case location^.size of
                   OS_F32,
                   OS_F64,
                   OS_F128:
                     a_loadmm_ref_reg(list,location^.def,location^.def,tmpref,location^.register,mms_movescalar);
                   OS_M8..OS_M128,
                   OS_MS8..OS_MS128,
                   OS_32..OS_128,
                   { OS_NO is for records of non-power-of-two sizes that have to
                     be passed in MM registers -> never scalar floats }
                   OS_NO:
                     a_loadmm_ref_reg(list,location^.def,location^.def,tmpref,location^.register,nil);
                   else
                     internalerror(2010053101);
                 end;
              end
            else
              internalerror(2010053111);
          end;
          inc(totaloffset,tcgsize2size[location^.size]);
          dec(sizeleft,tcgsize2size[location^.size]);
          location:=location^.next;
          inc(paralocidx);
        end;
    end;


  procedure thlcgllvm.a_load_const_cgpara(list: TAsmList; tosize: tdef; a: tcgint; const cgpara: TCGPara);
    begin
      if is_ordinal(cgpara.def) then
        begin
          cgpara.check_simple_location;
          paramanager.alloccgpara(list,cgpara);
          if cgpara.location^.shiftval<0 then
            a:=a shl -cgpara.location^.shiftval;
          cgpara.location^.llvmloc.loc:=LOC_CONSTANT;
          cgpara.location^.llvmloc.value:=a;
        end
      else
        inherited;
    end;


  procedure thlcgllvm.a_load_ref_cgpara_init_src(list: TAsmList; const para: tcgpara; const initialref: treference; var refsize: tdef; out newref: treference);
    var
      newrefsize: tdef;
      reg: tregister;
    begin
      newrefsize:=llvmgetcgparadef(para,true,callerside);
      if (refsize<>newrefsize) and
         (initialref.refaddr<>addr_full) then
        begin
          reg:=getaddressregister(list,cpointerdef.getreusable(newrefsize));
          a_loadaddr_ref_reg(list,refsize,cpointerdef.getreusable(newrefsize),initialref,reg);
          reference_reset_base(newref,cpointerdef.getreusable(newrefsize),reg,0,initialref.temppos,initialref.alignment,initialref.volatility);
          refsize:=newrefsize;
        end
      else
        newref:=initialref;
    end;


  procedure thlcgllvm.getcpuregister(list: TAsmList; r: Tregister);
    begin
      { don't do anything }
    end;


  procedure thlcgllvm.ungetcpuregister(list: TAsmList; r: Tregister);
    begin
      { don't do anything }
    end;


  procedure thlcgllvm.alloccpuregisters(list: TAsmList; rt: Tregistertype; const r: Tcpuregisterset);
    begin
      { don't do anything }
    end;


  procedure thlcgllvm.allocallcpuregisters(list: TAsmList);
    begin
      { don't do anything }
    end;


  procedure thlcgllvm.deallocallcpuregisters(list: TAsmList);
    begin
      { don't do anything }
    end;


  procedure thlcgllvm.a_bit_test_reg_reg_reg(list: TAsmList; bitnumbersize, valuesize, destsize: tdef; bitnumber, value, destreg: tregister);
    var
      tmpbitnumberreg: tregister;
    begin
      { unlike other architectures, llvm requires the bitnumber register to
        have the same size as the shifted register }
      if bitnumbersize.size<>valuesize.size then
        begin
          tmpbitnumberreg:=hlcg.getintregister(list,valuesize);
          a_load_reg_reg(list,bitnumbersize,valuesize,bitnumber,tmpbitnumberreg);
          bitnumbersize:=valuesize;
          bitnumber:=tmpbitnumberreg;
        end;
      inherited;
    end;


  procedure thlcgllvm.a_bit_set_reg_reg(list: TAsmList; doset: boolean; bitnumbersize, destsize: tdef; bitnumber, dest: tregister);
    var
      tmpbitnumberreg: tregister;
    begin
      { unlike other architectures, llvm requires the bitnumber register to
        have the same size as the shifted register }
      if bitnumbersize.size<>destsize.size then
        begin
          tmpbitnumberreg:=hlcg.getintregister(list,destsize);
          a_load_reg_reg(list,bitnumbersize,destsize,bitnumber,tmpbitnumberreg);
          bitnumbersize:=destsize;
          bitnumber:=tmpbitnumberreg;
        end;
      inherited;
    end;


  function get_call_pd(pd: tabstractprocdef): tdef;
    begin
      if (pd.typ=procdef) or
         not pd.is_addressonly then
        { we get a pointerdef rather than a procvardef so that if we have to
          insert an external declaration for this procdef in llvmtype, we don't
          have to create another procdef from the procvardef we've just created.
          With a pointerdef, we can just get the pointeddef again. A pointerdef
          is also much cheaper to create, and in llvm a provardef is a "function
          pointer", so a pointer to a procdef is the same as a procvar as far
          as llvm is concerned }
        result:=cpointerdef.getreusable(pd)
      else
        result:=pd
    end;


  procedure thlcgllvm.a_call_common(list: TAsmList; pd: tabstractprocdef; const paras: array of pcgpara; const forceresdef: tdef; out res: tregister; out hlretdef: tdef; out llvmretdef: tdef; out callparas: tfplist);

    procedure load_ref_anyreg(def: tdef; const ref: treference; reg: tregister);
      begin
        case getregtype(reg) of
          R_INTREGISTER,
          R_ADDRESSREGISTER:
            begin
              a_load_ref_reg(list,def,def,ref,reg);
            end;
          R_FPUREGISTER:
            begin
              a_loadfpu_ref_reg(list,def,def,ref,reg);
            end;
          R_MMREGISTER:
            begin
              a_loadmm_ref_reg(list,def,def,ref,reg,mms_movescalar);
            end;
          else
            internalerror(2014012213);
        end;
      end;

  var
    i: longint;
    href: treference;
    callpara: pllvmcallpara;
    paraloc: pcgparalocation;
    firstparaloc: boolean;
  begin
    callparas:=tfplist.Create;
    for i:=0 to high(paras) do
      begin
        { skip parameters without data }
        if paras[i]^.isempty then
          continue;
        paraloc:=paras[i]^.location;
        firstparaloc:=true;
        while assigned(paraloc) do
          begin
            new(callpara);
            callpara^.def:=paraloc^.def;
            if firstparaloc then
              callpara^.alignment:=paras[i]^.Alignment
            else
              callpara^.alignment:=std_param_align;
            { if the paraloc doesn't contain the value itself, it's a byval
              parameter }
            if paraloc^.retvalloc then
              begin
                callpara^.sret:=true;
                callpara^.byval:=false;
              end
            else
              begin
                callpara^.sret:=false;
                callpara^.byval:=not paraloc^.llvmvalueloc;
              end;
            llvmextractvalueextinfo(paras[i]^.def, callpara^.def, callpara^.valueext);
            case paraloc^.llvmloc.loc of
              LOC_CONSTANT:
                begin
                  callpara^.typ:=top_const;
                  callpara^.value:=paraloc^.llvmloc.value;
                end;
              LOC_CREFERENCE:
                begin
                  callpara^.typ:=top_ref;
                  callpara^.sym:=paraloc^.llvmloc.sym;
                end
              else
                begin
                  case paraloc^.loc of
                    LOC_REFERENCE:
                      begin
                        if paraloc^.llvmvalueloc then
                          internalerror(2014012307)
                        else
                          begin
                            callpara^.typ:=top_reg;
                            reference_reset_base(href, cpointerdef.getreusable(callpara^.def), paraloc^.reference.index, paraloc^.reference.offset, ctempposinvalid, paraloc^.def.alignment, []);
                            res:=getregisterfordef(list, paraloc^.def);
                            load_ref_anyreg(callpara^.def, href, res);
                          end;
                        callpara^.register:=res
                      end;
                    LOC_REGISTER,
                    LOC_FPUREGISTER,
                    LOC_MMREGISTER:
                      begin
                        callpara^.typ:=top_reg;
                        { undo explicit value extension }
                        if callpara^.valueext<>lve_none then
                          begin
                            res:=getregisterfordef(list, callpara^.def);
                            a_load_reg_reg(list, paraloc^.def, callpara^.def, paraloc^.register, res);
                            paraloc^.register:=res;
                          end;
                        callpara^.register:=paraloc^.register
                      end;
                    { empty records }
                    LOC_VOID:
                      begin
                        callpara^.typ:=top_undef;
                      end
                    else
                      internalerror(2014010605);
                  end;
                end;
              end;
            callparas.add(callpara);
            paraloc:=paraloc^.next;
            firstparaloc:=false;
          end;
      end;
    { the Pascal level may expect a different returndef compared to the
      declared one }
    if pd.generate_safecall_wrapper then
      begin
        hlretdef:=ossinttype;
        llvmretdef:=ossinttype;
      end
    else
      begin
        if not assigned(forceresdef) then
          hlretdef:=pd.returndef
        else
          hlretdef:=forceresdef;
        { llvm will always expect the original return def }
        if not paramanager.ret_in_param(hlretdef, pd) or
           pd.generate_safecall_wrapper then
          llvmretdef:=llvmgetcgparadef(pd.funcretloc[callerside], true, callerside)
        else
          llvmretdef:=voidtype;
      end;
    if not is_void(llvmretdef) then
      res:=getregisterfordef(list, llvmretdef)
    else
      res:=NR_NO;

    { if this is a complex procvar, get the non-tmethod-like equivalent }
    if (pd.typ=procvardef) and
       not pd.is_addressonly then
      pd:=tprocvardef(cprocvardef.getreusableprocaddr(pd));
  end;


  function thlcgllvm.a_call_name(list: TAsmList; pd: tprocdef; const s: TSymStr; const paras: array of pcgpara; forceresdef: tdef; weak: boolean): tcgpara;
    var
      callparas: tfplist;
      llvmretdef,
      hlretdef: tdef;
      res: tregister;
      nextinslab,
      exceptlab: TAsmLabel;
    begin
      a_call_common(list,pd,paras,forceresdef,res,hlretdef,llvmretdef,callparas);
      if not(fc_catching_exceptions in flowcontrol) or
         { no invoke for intrinsics }
         (copy(s,1,5)='llvm.') then
        list.concat(taillvm.call_size_name_paras(get_call_pd(pd),pd.proccalloption,res,llvmretdef,current_asmdata.RefAsmSymbol(s,AT_FUNCTION),callparas))
      else
        begin
          current_asmdata.getjumplabel(nextinslab);
          exceptlab:=tllvmprocinfo(current_procinfo).CurrExceptLabel;
          list.concat(taillvm.invoke_size_name_paras_retlab_exceptlab(get_call_pd(pd),pd.proccalloption,res,llvmretdef,current_asmdata.RefAsmSymbol(s,AT_FUNCTION),callparas,nextinslab,exceptlab));
          a_label(list,nextinslab);
        end;
      result:=get_call_result_cgpara(pd,forceresdef);
      set_call_function_result(list,pd,llvmretdef,hlretdef,res,result);
    end;


  function thlcgllvm.a_call_reg(list: TAsmList; pd: tabstractprocdef; reg: tregister; const paras: array of pcgpara): tcgpara;
    var
      callparas: tfplist;
      llvmretdef,
      hlretdef: tdef;
      res: tregister;
      nextinslab,
      exceptlab: TAsmLabel;
    begin
      a_call_common(list,pd,paras,nil,res,hlretdef,llvmretdef,callparas);
      if not(fc_catching_exceptions in flowcontrol) then
        list.concat(taillvm.call_size_reg_paras(get_call_pd(pd),pd.proccalloption,res,llvmretdef,reg,callparas))
      else
        begin
          current_asmdata.getjumplabel(nextinslab);
          exceptlab:=tllvmprocinfo(current_procinfo).CurrExceptLabel;
          list.concat(taillvm.invoke_size_reg_paras_retlab_exceptlab(get_call_pd(pd),pd.proccalloption,res,llvmretdef,reg,callparas,nextinslab,exceptlab));
          a_label(list,nextinslab);
        end;
      result:=get_call_result_cgpara(pd,nil);
      set_call_function_result(list,pd,llvmretdef,hlretdef,res,result);
    end;


  procedure thlcgllvm.a_load_const_reg(list: TAsmList; tosize: tdef; a: tcgint; register: tregister);
    var
      fromsize: tdef;
    begin
      if tosize.size<=ptrsinttype.size then
        fromsize:=ptrsinttype
      else
        fromsize:=tosize;
      list.concat(taillvm.op_reg_size_const_size(llvmconvop(fromsize,tosize,false),register,fromsize,a,tosize))
    end;


  procedure thlcgllvm.a_load_const_ref(list: TAsmList; tosize: tdef; a: tcgint; const ref: treference);
    var
      sref: treference;
    begin
      { llvm instructions do not support pointer constants -> only directly
        encode for integers; a_load_const_reg() handles pointers properly }
      if is_ordinal(tosize) or
         is_64bit(tosize) then
        begin
          sref:=make_simple_ref(list,ref,tosize);
          list.concat(taillvm.op_size_const_size_ref(la_store,tosize,a,cpointerdef.getreusable(tosize),sref))
        end
      else
        inherited;
    end;


  function def2intdef(fromsize, tosize: tdef): tdef;
    begin
      { we cannot zero-extend from/to anything but ordinal/enum
        types }
      if not(tosize.typ in [orddef,enumdef]) then
        internalerror(2014012305);
      { will give an internalerror if def_cgsize() returns OS_NO, which is
        what we want }
      result:=cgsize_orddef(def_cgsize(fromsize));
    end;


  procedure thlcgllvm.a_load_reg_ref(list: TAsmList; fromsize, tosize: tdef; register: tregister; const ref: treference);
    var
      tmpref,
      sref: treference;
      hreg,
      hreg2: tregister;
      tmpsize: tdef;
    begin
      sref:=make_simple_ref(list,ref,tosize);
      hreg:=register;
      (* typecast the pointer to the value instead of the value itself if
        they have the same size but are of different kinds, because we can't
        e.g. typecast a loaded <{i32, i32}> to an i64 *)
      if (llvmaggregatetype(fromsize) or
          llvmaggregatetype(tosize)) and
         (fromsize<>tosize) then
        begin
          if fromsize.size>tosize.size then
            begin
              { if source size is larger than the target size, we have to
                truncate it before storing. Unfortunately, we cannot truncate
                records (nor bitcast them to integers), so we first have to
                store them to memory and then bitcast the pointer to them

                We can't truncate an integer to 3/5/6/7 bytes either, so also
                pass via a temp in that case
              }
              if (fromsize.typ in [arraydef,recorddef]) or
                 (tosize.size in [3,5,6,7]) then
                begin
                  { store struct/array-in-register to memory }
                  tg.gethltemp(list,fromsize,fromsize.size,tt_normal,tmpref);
                  a_load_reg_ref(list,fromsize,fromsize,register,tmpref);
                  { typecast pointer to memory into pointer to integer type }
                  hreg:=getaddressregister(list,cpointerdef.getreusable(tosize));
                  a_loadaddr_ref_reg(list,fromsize,cpointerdef.getreusable(tosize),tmpref,hreg);
                  reference_reset_base(sref,cpointerdef.getreusable(tosize),hreg,0,tmpref.temppos,tmpref.alignment,tmpref.volatility);
                  { load the integer from the temp into the destination }
                  a_load_ref_ref(list,tosize,tosize,sref,ref);
                  tg.ungettemp(list,tmpref);
                end
              else
                begin
                  tmpsize:=def2intdef(tosize,fromsize);
                  hreg:=getintregister(list,tmpsize);
                  { truncate the integer }
                  a_load_reg_reg(list,fromsize,tmpsize,register,hreg);
                  { store it to memory (it will now be of the same size as the
                    struct, and hence another path will be followed in this
                    method) }
                  a_load_reg_ref(list,tmpsize,tosize,hreg,sref);
                end;
                exit;
            end
          else
            begin
              hreg2:=getaddressregister(list,cpointerdef.getreusable(fromsize));
              a_loadaddr_ref_reg(list,tosize,cpointerdef.getreusable(fromsize),sref,hreg2);
              reference_reset_base(sref,cpointerdef.getreusable(fromsize),hreg2,0,sref.temppos,sref.alignment,sref.volatility);
              tosize:=fromsize;
            end;
        end
      else if fromsize<>tosize then
        begin
          hreg:=getregisterfordef(list,tosize);
          a_load_reg_reg(list,fromsize,tosize,register,hreg);
        end;
      list.concat(taillvm.op_size_reg_size_ref(la_store,tosize,hreg,cpointerdef.getreusable(tosize),sref));
    end;


  procedure thlcgllvm.a_load_reg_reg(list: TAsmList; fromsize, tosize: tdef; reg1, reg2: tregister);
    var
      op: tllvmop;
      tmpreg: tregister;
      tmpintdef: tdef;
    begin
      op:=llvmconvop(fromsize,tosize,true);
      { converting from pointer to something else and vice versa is only
        possible via an intermediate pass to integer. Same for "something else"
        to pointer. }
      case op of
        la_ptrtoint_to_x,
        la_x_to_inttoptr:
          begin
            { convert via an integer with the same size as "x" }
            if op=la_ptrtoint_to_x then
              begin
                tmpintdef:=cgsize_orddef(def_cgsize(tosize));
                op:=la_bitcast
              end
            else
              begin
                tmpintdef:=cgsize_orddef(def_cgsize(fromsize));
                op:=la_inttoptr;
              end;
            tmpreg:=getintregister(list,tmpintdef);
            a_load_reg_reg(list,fromsize,tmpintdef,reg1,tmpreg);
            reg1:=tmpreg;
            fromsize:=tmpintdef;
          end;
        else
          ;
      end;
      { reg2 = bitcast fromsize reg1 to tosize }
      list.concat(taillvm.op_reg_size_reg_size(op,reg2,fromsize,reg1,tosize));
    end;


  procedure thlcgllvm.gen_load_refaddrfull_anyreg(list: TAsmList; fromsize, tosize: tdef; const simpleref: treference; register: tregister; shuffle: pmmshuffle);
    var
      tmpref,
      tmpref2: treference;
    begin
      { can't bitcast records/arrays }
      if (llvmaggregatetype(fromsize) or
          llvmaggregatetype(tosize)) and
         (fromsize<>tosize) then
        begin
          if fromsize.size>tosize.size then
            begin
              tg.gethltemp(list,fromsize,fromsize.size,tt_normal,tmpref);
              tmpref2:=tmpref;
              g_ptrtypecast_ref(list,cpointerdef.getreusable(fromsize),cpointerdef.getreusable(tosize),tmpref2);
            end
          else
            begin
              tg.gethltemp(list,tosize,tosize.size,tt_normal,tmpref);
              tmpref2:=tmpref;
              g_ptrtypecast_ref(list,cpointerdef.getreusable(tosize),cpointerdef.getreusable(fromsize),tmpref);
            end;
          list.concat(taillvm.op_size_ref_size_ref(la_store,fromsize,simpleref,cpointerdef.getreusable(fromsize),tmpref));
          case getregtype(register) of
            R_INTREGISTER,
            R_ADDRESSREGISTER:
              a_load_ref_reg(list,tosize,tosize,tmpref2,register);
            R_FPUREGISTER:
              a_loadfpu_ref_reg(list,tosize,tosize,tmpref2,register);
            R_MMREGISTER:
              a_loadmm_ref_reg(list,tosize,tosize,tmpref2,register,shuffle);
            else
              internalerror(2016061901);
          end;
          tg.ungettemp(list,tmpref);
        end
      else
        list.concat(taillvm.op_reg_size_ref_size(llvmconvop(fromsize,tosize,false),register,fromsize,simpleref,tosize))
    end;


  function thlcgllvm.handle_agg_load_ref_anyreg(list: TasmList; var fromsize, tosize: tdef; var simpleref: treference; register: tregister; shuffle: pmmshuffle): boolean;
    var
      tmpref,
      tmpref2: treference;
      firstshuffle: pmmshuffle;
    begin
      if fromsize.size<tosize.size then
        begin
          { allocate a temp of size tosize, typecast it to the
            (smaller) fromsize, load the source in it, and then
            load the destination from it. The extra bits will contain
            garbage, but they should never be used. }
          tg.gethltemp(list,tosize,tosize.size,tt_persistent,tmpref);
          tmpref2:=tmpref;
          g_ptrtypecast_ref(list,cpointerdef.getreusable(tosize),cpointerdef.getreusable(fromsize),tmpref2);
          case getregtype(register) of
            R_INTREGISTER,
            R_ADDRESSREGISTER:
              begin
                a_load_ref_ref(list,fromsize,fromsize,simpleref,tmpref2);
                a_load_ref_reg(list,tosize,tosize,tmpref,register);
              end;
            R_FPUREGISTER:
              begin
                a_loadfpu_ref_ref(list,fromsize,fromsize,simpleref,tmpref2);
                a_loadfpu_ref_reg(list,tosize,tosize,tmpref,register);
              end;
            R_MMREGISTER:
              begin
                { don't shuffle twice }
                if shuffle=mms_movescalar then
                  firstshuffle:=shuffle
                else
                  firstshuffle:=nil;
                a_loadmm_ref_ref(list,fromsize,fromsize,simpleref,tmpref2,firstshuffle);
                a_loadmm_ref_reg(list,tosize,tosize,tmpref,register,shuffle);
              end;
            else
              internalerror(2019051040);
          end;
          tg.ungettemp(list,tmpref);
          result:=true;
        end
      else
        begin
          (* typecast the pointer to the value instead of the value
             itself if tosize<=fromsize but they are of different
             kinds, because we can't e.g. bitcast a loaded <{i32, i32}>
             to an i64 *)
          g_ptrtypecast_ref(list,cpointerdef.getreusable(fromsize),cpointerdef.getreusable(tosize),simpleref);
          fromsize:=tosize;
          result:=false;
        end;
    end;


  procedure thlcgllvm.a_load_ref_reg(list: TAsmList; fromsize, tosize: tdef; const ref: treference; register: tregister);
    var
      sref: treference;
      hreg: tregister;
    begin
      sref:=make_simple_ref(list,ref,fromsize);
      { "named register"? }
      if sref.refaddr=addr_full then
        gen_load_refaddrfull_anyreg(list,fromsize,tosize,sref,register,nil)
      else
        begin
          if ((fromsize.typ in [arraydef,recorddef]) or
              (tosize.typ in [arraydef,recorddef])) and
             (fromsize<>tosize) then
            begin
              if handle_agg_load_ref_anyreg(list,fromsize,tosize,sref,register,nil) then
                exit;
            end;
          hreg:=register;
          if fromsize<>tosize then
            hreg:=getregisterfordef(list,fromsize);
          list.concat(taillvm.op_reg_size_ref(la_load,hreg,cpointerdef.getreusable(fromsize),sref));
          if hreg<>register then
            a_load_reg_reg(list,fromsize,tosize,hreg,register);
        end;
    end;


  procedure thlcgllvm.a_load_ref_ref(list: TAsmList; fromsize, tosize: tdef; const sref: treference; const dref: treference);
    var
      sdref: treference;
    begin
      if (fromsize=tosize) and
         (sref.refaddr=addr_full) then
        begin
          sdref:=make_simple_ref(list,dref,tosize);
          list.concat(taillvm.op_size_ref_size_ref(la_store,fromsize,sref,cpointerdef.getreusable(tosize),sdref));
        end
      else if (fromsize=tosize) and
              not(fromsize.typ in [orddef,floatdef,enumdef]) and
              (sref.refaddr<>addr_full) and
              (fromsize.size>2*sizeof(aint)) then
         g_concatcopy(list,fromsize,sref,dref)
      else
        inherited
    end;


  procedure thlcgllvm.a_loadaddr_ref_reg_intern(list: TAsmList; fromsize, tosize: tdef; const ref: treference; r: tregister; makefromsizepointer: boolean);
    var
      sref: treference;
    begin
      { can't take the address of a 'named register' }
      if ref.refaddr=addr_full then
        internalerror(2013102306);
      if makefromsizepointer then
        fromsize:=cpointerdef.getreusable(fromsize);
      sref:=make_simple_ref_ptr(list,ref,fromsize);
      list.concat(taillvm.op_reg_size_ref_size(la_bitcast,r,fromsize,sref,tosize));
    end;


  procedure thlcgllvm.a_loadaddr_ref_reg(list: TAsmList; fromsize, tosize: tdef; const ref: treference; r: tregister);
    begin
      a_loadaddr_ref_reg_intern(list,fromsize,tosize,ref,r,true);
    end;


  procedure thlcgllvm.a_op_const_reg(list: TAsmList; Op: TOpCG; size: tdef; a: tcgint; reg: TRegister);
    begin
      a_op_const_reg_reg(list,op,size,a,reg,reg);
    end;


  procedure thlcgllvm.a_op_const_reg_reg(list: TAsmList; op: TOpCg; size: tdef; a: tcgint; src, dst: tregister);
    var
      tmpreg: tregister;
    begin
      if (def2regtyp(size)=R_INTREGISTER) and
         (topcg2llvmop[op]<>la_none) then
        list.concat(taillvm.op_reg_size_reg_const(topcg2llvmop[op],dst,size,src,a))
      else
        begin
          { default implementation is not SSA-safe }
          tmpreg:=getregisterfordef(list,size);
          a_load_const_reg(list,size,a,tmpreg);
          a_op_reg_reg_reg(list,op,size,tmpreg,src,dst);
        end;
    end;


  procedure thlcgllvm.a_op_reg_reg_reg(list: TAsmList; op: TOpCg; size: tdef; src1, src2, dst: tregister);
    var
      orgdst,
      tmpreg1,
      tmpreg2,
      tmpreg3: tregister;
      opsize: tdef;
    begin
      orgdst:=dst;
      opsize:=size;
      { always perform using integer registers, because math operations on
        pointers are not supported (except via getelementptr, possible future
        optimization) }
      if def2regtyp(size)=R_ADDRESSREGISTER then
        begin
          opsize:=ptruinttype;

          tmpreg1:=getintregister(list,ptruinttype);
          a_load_reg_reg(list,size,ptruinttype,src1,tmpreg1);
          src1:=tmpreg1;

          tmpreg1:=getintregister(list,ptruinttype);
          a_load_reg_reg(list,size,ptruinttype,src2,tmpreg1);
          src2:=tmpreg1;

          dst:=getintregister(list,ptruinttype);
        end;
     if topcg2llvmop[op]<>la_none then
       list.concat(taillvm.op_reg_size_reg_reg(topcg2llvmop[op],dst,opsize,src2,src1))
     else
       begin
         case op of
           OP_NEG:
             { %dst = sub size 0, %src1 }
             list.concat(taillvm.op_reg_size_const_reg(la_sub,dst,opsize,0,src1));
           OP_NOT:
             { %dst = xor size -1, %src1 }
             list.concat(taillvm.op_reg_size_const_reg(la_xor,dst,opsize,-1,src1));
           OP_ROL:
             begin
               tmpreg1:=getintregister(list,opsize);
               tmpreg2:=getintregister(list,opsize);
               tmpreg3:=getintregister(list,opsize);
               { tmpreg1 := (tcgsize2size[size]*8 - (src1 and (tcgsize2size[size]*8-1) }
               list.concat(taillvm.op_reg_size_const_reg(la_and,tmpreg1,opsize,opsize.size*8-1,src1));
               list.concat(taillvm.op_reg_size_const_reg(la_sub,tmpreg2,opsize,opsize.size*8,tmpreg1));
               { tmpreg3 := src2 shr tmpreg2 }
               a_op_reg_reg_reg(list,OP_SHR,opsize,tmpreg2,src2,tmpreg3);
               { tmpreg2:= src2 shl tmpreg1 }
               tmpreg2:=getintregister(list,opsize);
               a_op_reg_reg_reg(list,OP_SHL,opsize,tmpreg1,src2,tmpreg2);
               { dst := tmpreg2 or tmpreg3 }
               a_op_reg_reg_reg(list,OP_OR,opsize,tmpreg2,tmpreg3,dst);
             end;
           OP_ROR:
             begin
               tmpreg1:=getintregister(list,size);
               tmpreg2:=getintregister(list,size);
               tmpreg3:=getintregister(list,size);
               { tmpreg1 := (tcgsize2size[size]*8 - (src1 and (tcgsize2size[size]*8-1) }
               list.concat(taillvm.op_reg_size_const_reg(la_and,tmpreg1,opsize,opsize.size*8-1,src1));
               list.concat(taillvm.op_reg_size_const_reg(la_sub,tmpreg2,opsize,opsize.size*8,tmpreg1));
               { tmpreg3 := src2 shl tmpreg2 }
               a_op_reg_reg_reg(list,OP_SHL,opsize,tmpreg2,src2,tmpreg3);
               { tmpreg2:= src2 shr tmpreg1 }
               tmpreg2:=getintregister(list,opsize);
               a_op_reg_reg_reg(list,OP_SHR,opsize,tmpreg1,src2,tmpreg2);
               { dst := tmpreg2 or tmpreg3 }
               a_op_reg_reg_reg(list,OP_OR,opsize,tmpreg2,tmpreg3,dst);
             end;
           else
             internalerror(2010081310);
         end;
       end;
     if dst<>orgdst then
       a_load_reg_reg(list,opsize,size,dst,orgdst);
   end;


  procedure thlcgllvm.a_op_reg_reg(list: TAsmList; Op: TOpCG; size: tdef; reg1, reg2: TRegister);
    begin
       a_op_reg_reg_reg(list,op,size,reg1,reg2,reg2);
    end;


  procedure thlcgllvm.a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tdef; a: tcgint; src, dst: tregister; setflags: boolean; var ovloc: tlocation);
    var
      hreg: tregister;
    begin
      if not setflags then
        begin
          inherited;
          exit;
        end;
      hreg:=getintregister(list,size);
      a_load_const_reg(list,size,a,hreg);
      a_op_reg_reg_reg_checkoverflow(list,op,size,hreg,src,dst,setflags,ovloc);
    end;


  procedure thlcgllvm.a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tdef; src1, src2, dst: tregister; setflags: boolean; var ovloc: tlocation);
    var
      calcsize: tdef;
      tmpsrc1,
      tmpsrc2,
      tmpdst: tregister;
      signed,
      docheck: boolean;
    begin
      docheck:=size.size>=ossinttype.size;
      if not setflags or
         not docheck then
        begin
          inherited a_op_reg_reg_reg_checkoverflow(list,op,size,src1,src2,dst,false,ovloc);
          exit;
        end;
      { extend values to twice their original width (one bit extra is enough,
        but adding support for 9/17/33/65 bit types just for this is overkill) }
      signed:=is_signed(size);
      case size.size of
        1:
          if signed then
            calcsize:=s16inttype
          else
            calcsize:=u16inttype;
        2:
          if signed then
            calcsize:=s32inttype
          else
            calcsize:=u32inttype;
        4:
          if signed then
            calcsize:=s64inttype
          else
            calcsize:=u64inttype;
        8:
          if signed then
            calcsize:=s128inttype
          else
            calcsize:=u128inttype;
        else
          internalerror(2015122503);
      end;
      tmpsrc1:=getintregister(list,calcsize);
      a_load_reg_reg(list,size,calcsize,src1,tmpsrc1);
      tmpsrc2:=getintregister(list,calcsize);
      a_load_reg_reg(list,size,calcsize,src2,tmpsrc2);
      tmpdst:=getintregister(list,calcsize);
      { perform the calculation with twice the width }
      a_op_reg_reg_reg(list,op,calcsize,tmpsrc1,tmpsrc2,tmpdst);
      { signed/unsigned overflow occurs if signed/unsigned truncation of the
        result is different from the actual result -> extend again and compare }
      a_load_reg_reg(list,calcsize,size,tmpdst,dst);
      tmpsrc1:=getintregister(list,calcsize);
      a_load_reg_reg(list,size,calcsize,dst,tmpsrc1);
      location_reset(ovloc,LOC_REGISTER,OS_8);
      ovloc.register:=getintregister(list,llvmbool1type);
      list.concat(taillvm.op_reg_cond_size_reg_reg(la_icmp,ovloc.register,OC_NE,calcsize,tmpsrc1,tmpdst));
    end;


  procedure thlcgllvm.a_cmp_const_reg_label(list: TAsmList; size: tdef; cmp_op: topcmp; a: tcgint; reg: tregister; l: tasmlabel);
    var
      tmpreg : tregister;
      invert: boolean;
      fallthroughlab, falselab, tmplab: tasmlabel;
    begin
      tmpreg:=getregisterfordef(list,size);
      a_load_const_reg(list,size,a,tmpreg);
      a_cmp_reg_reg_label(list,size,cmp_op,tmpreg,reg,l);
    end;


  procedure thlcgllvm.a_cmp_reg_reg_label(list: TAsmList; size: tdef; cmp_op: topcmp; reg1, reg2: tregister; l: tasmlabel);
    var
      resreg: tregister;
      falselab: tasmlabel;
    begin
      if getregtype(reg1)<>getregtype(reg2) then
        internalerror(2012111105);
      resreg:=getintregister(list,llvmbool1type);
      current_asmdata.getjumplabel(falselab);
      { invert order of registers. In FPC, cmp_reg_reg(reg1,reg2) means that
        e.g. OC_GT is true if "subl %reg1,%reg2" in x86 AT&T is >0. In LLVM,
        OC_GT is true if op1>op2 }
      list.concat(taillvm.op_reg_cond_size_reg_reg(la_icmp,resreg,cmp_op,size,reg2,reg1));
      list.concat(taillvm.op_size_reg_lab_lab(la_br,llvmbool1type,resreg,l,falselab));
      a_label(list,falselab);
    end;


  procedure thlcgllvm.a_jmp_always(list: TAsmList; l: tasmlabel);
    begin
      { implement in tcg because required by the overridden a_label; doesn't use
        any high level stuff anyway }
      cg.a_jmp_always(list,l);
    end;

  procedure thlcgllvm.g_unreachable(list: TAsmList);
    begin
      list.Concat(taillvm.op_none(la_unreachable));
    end;


  procedure thlcgllvm.g_concatcopy(list: TAsmList; size: tdef; const source, dest: treference);
    var
      pd: tprocdef;
      sourcepara, destpara, sizepara, alignpara, volatilepara: tcgpara;
      maxalign: longint;
      indivalign: boolean;
    begin
      { perform small copies directly; not larger ones, because then llvm
        will try to load the entire large datastructure into registers and
        starts spilling like crazy; too small copies must not be done via
        llvm.memcpy either, because then you get crashes in llvm }
      if (size.typ in [orddef,floatdef,enumdef]) or
         (size.size<=2*sizeof(aint)) then
        begin
          a_load_ref_ref(list,size,size,source,dest);
          exit;
        end;
      indivalign:=llvmflag_memcpy_indiv_align in llvmversion_properties[current_settings.llvmversion];
      if indivalign then
        pd:=search_system_proc('llvm_memcpy64_indivalign')
      else
        pd:=search_system_proc('llvm_memcpy64');
      sourcepara.init;
      destpara.init;
      sizepara.init;
      alignpara.init;
      volatilepara.init;
      paramanager.getcgtempparaloc(list,pd,1,destpara);
      paramanager.getcgtempparaloc(list,pd,2,sourcepara);
      paramanager.getcgtempparaloc(list,pd,3,sizepara);
      if indivalign then
        begin
          paramanager.getcgtempparaloc(list,pd,4,volatilepara);
          destpara.Alignment:=-dest.alignment;
          sourcepara.Alignment:=-source.alignment;
        end
      else
        begin
          paramanager.getcgtempparaloc(list,pd,4,alignpara);
          paramanager.getcgtempparaloc(list,pd,5,volatilepara);
          maxalign:=newalignment(max(source.alignment,dest.alignment),min(source.alignment,dest.alignment));
          a_load_const_cgpara(list,u32inttype,maxalign,alignpara);
        end;
      a_loadaddr_ref_cgpara(list,size,dest,destpara);
      a_loadaddr_ref_cgpara(list,size,source,sourcepara);
      a_load_const_cgpara(list,u64inttype,size.size,sizepara);
      a_load_const_cgpara(list,llvmbool1type,ord((vol_read in source.volatility) or (vol_write in dest.volatility)),volatilepara);
      if indivalign then
        g_call_system_proc(list,pd,[@destpara,@sourcepara,@sizepara,@volatilepara],nil).resetiftemp
      else
        g_call_system_proc(list,pd,[@destpara,@sourcepara,@sizepara,@alignpara,@volatilepara],nil).resetiftemp;
      sourcepara.done;
      destpara.done;
      sizepara.done;
      alignpara.done;
      volatilepara.done;
    end;


  procedure thlcgllvm.a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tdef; const ref: treference; reg: tregister);
    var
       tmpreg: tregister;
       href: treference;
       fromcompcurr,
       tocompcurr: boolean;
     begin
       href:=make_simple_ref(list,ref,fromsize);
       { named register -> use generic code }
       if ref.refaddr=addr_full then
         begin
           gen_load_refaddrfull_anyreg(list,fromsize,tosize,href,reg,mms_movescalar);
           exit
         end;
       { comp and currency are handled by the x87 in this case. They cannot
         be represented directly in llvm, and llvmdef translates them into i64
         (since that's their storage size and internally they also are int64).
         Solve this by changing the type to s80real once they are loaded into
         a register. }
       fromcompcurr:=
         (fromsize.typ=floatdef) and
         (tfloatdef(fromsize).floattype in [s64comp,s64currency]);
       tocompcurr:=
         (tosize.typ=floatdef) and
         (tfloatdef(tosize).floattype in [s64comp,s64currency]);
       if tocompcurr then
         tosize:=s80floattype;
       { don't generate different code for loading e.g. extended into cextended,
         but to take care of loading e.g. comp (=int64) into double }
       if (fromsize.size<>tosize.size) then
         tmpreg:=getfpuregister(list,fromsize)
       else
         tmpreg:=reg;
       { handle aggregate loads (happens if a struct needs to be passed in a
         floating point register) }
       if (fromsize.typ in [arraydef,recorddef]) or
          (tosize.typ in [arraydef,recorddef]) then
         begin
           if handle_agg_load_ref_anyreg(list,fromsize,tosize,href,reg,mms_movescalar) then
             exit;
         end;
       { %tmpreg = load size* %ref }
       list.concat(taillvm.op_reg_size_ref(la_load,tmpreg,cpointerdef.getreusable(fromsize),href));
       if tmpreg<>reg then
         if fromcompcurr then
           { treat as extended as long as it's in a register }
           list.concat(taillvm.op_reg_size_reg_size(la_sitofp,reg,fromsize,tmpreg,tosize))
         else
           a_loadfpu_reg_reg(list,fromsize,tosize,tmpreg,reg);
     end;


  procedure thlcgllvm.a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tdef; reg: tregister; const ref: treference);
    var
       tmpreg: tregister;
       href: treference;
       fromcompcurr,
       tocompcurr: boolean;
     begin
       { see comment in a_loadfpu_ref_reg }
       fromcompcurr:=
         (fromsize.typ=floatdef) and
         (tfloatdef(fromsize).floattype in [s64comp,s64currency]);
       tocompcurr:=
         (tosize.typ=floatdef) and
         (tfloatdef(tosize).floattype in [s64comp,s64currency]);
       if fromcompcurr then
         fromsize:=s80floattype;
       href:=make_simple_ref(list,ref,tosize);
       { don't generate different code for loading e.g. extended into cextended,
         but to take care of storing e.g. comp (=int64) into double  }
       if (fromsize.size<>tosize.size) then
         begin
           tmpreg:=getfpuregister(list,tosize);
           if tocompcurr then
             { store back an int64 rather than an extended }
             list.concat(taillvm.op_reg_size_reg_size(la_fptosi,tmpreg,fromsize,reg,tosize))
           else
             a_loadfpu_reg_reg(list,fromsize,tosize,reg,tmpreg);
         end
       else
         tmpreg:=reg;
       { store tosize tmpreg, tosize* href }
       list.concat(taillvm.op_size_reg_size_ref(la_store,tosize,tmpreg,cpointerdef.getreusable(tosize),href));
     end;


  procedure thlcgllvm.a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize: tdef; reg1, reg2: tregister);
    var
      op: tllvmop;
    begin
      op:=llvmconvop(fromsize,tosize,true);
      { reg2 = bitcast fromllsize reg1 to tollsize }
      list.concat(taillvm.op_reg_size_reg_size(op,reg2,fromsize,reg1,tosize));
    end;


  procedure thlcgllvm.gen_proc_symbol(list: TAsmList);
    var
      item: TCmdStrListItem;
      mangledname: TSymStr;
      asmsym: tasmsymbol;
    begin
      if po_external in current_procinfo.procdef.procoptions then
        exit;
      item:=TCmdStrListItem(current_procinfo.procdef.aliasnames.first);
      mangledname:=current_procinfo.procdef.mangledname;
      { predefine the real function name as local/global, so the aliases can
        refer to the symbol and get the binding correct }
      if (cs_profile in current_settings.moduleswitches) or
         (po_global in current_procinfo.procdef.procoptions) then
        asmsym:=current_asmdata.DefineAsmSymbol(mangledname,AB_GLOBAL,AT_FUNCTION,current_procinfo.procdef)
      else
        asmsym:=current_asmdata.DefineAsmSymbol(mangledname,AB_LOCAL,AT_FUNCTION,current_procinfo.procdef);
      while assigned(item) do
        begin
          if mangledname<>item.Str then
            list.concat(taillvmalias.create(asmsym,item.str,current_procinfo.procdef,asmsym.bind));
          item:=TCmdStrListItem(item.next);
        end;
      list.concat(taillvmdecl.createdef(asmsym,current_procinfo.procdef,nil,sec_code,current_procinfo.procdef.alignment));
    end;


  procedure thlcgllvm.handle_external_proc(list: TAsmList; pd: tprocdef; const importname: TSymStr);
    begin
      { don't do anything, because at this point we can't know yet for certain
        whether the aliased routine is internal to the current routine or not.
        If it's internal, we would have to generate an llvm alias, while if it's
        external, we would have to generate a declaration. Additionally, aliases
        cannot refer to declarations, so always creating aliases doesn't work
        either -> handle in llvmtype }
    end;


  procedure thlcgllvm.g_proc_entry(list: TAsmList; localsize: longint; nostackframe: boolean);
    begin
      list.concatlist(ttgllvm(tg).alloclist)
      { rest: todo }
    end;


  procedure thlcgllvm.g_proc_exit(list: TAsmList; parasize: longint; nostackframe: boolean);
    var
      retdef: tdef;
      retreg,
      hreg: tregister;
      retpara: tcgpara;
    begin
      { the function result type is the type of the first location, which can
        differ from the real result type (e.g. int64 for a record consisting of
        two longint fields on x86-64 -- we are responsible for lowering the
        result types like that) }
      retpara:=get_call_result_cgpara(current_procinfo.procdef,nil);
      retpara.check_simple_location;
      retdef:=retpara.location^.def;
      if (is_void(retdef) or
          { don't check retdef here, it is e.g. a pshortstring in case it's
            shortstring that's returned in a parameter }
          paramanager.ret_in_param(current_procinfo.procdef.returndef,current_procinfo.procdef)) and
         not current_procinfo.procdef.generate_safecall_wrapper then
        list.concat(taillvm.op_size(la_ret,voidtype))
      else
        begin
          case retpara.location^.loc of
            LOC_REGISTER,
            LOC_FPUREGISTER,
            LOC_MMREGISTER:
              begin
                { sign/zeroextension of function results is handled implicitly
                  via the signext/zeroext modifiers of the result, rather than
                  in the code generator -> remove any explicit extensions here }
                retreg:=retpara.location^.register;
                if (current_procinfo.procdef.returndef.typ in [orddef,enumdef]) and
                   (retdef.typ in [orddef,enumdef]) and
                   not current_procinfo.procdef.generate_safecall_wrapper then
                  begin
                    if (current_procinfo.procdef.returndef.size<retpara.location^.def.size) then
                      begin
                        hreg:=getintregister(list,current_procinfo.procdef.returndef);
                        a_load_reg_reg(list,retdef,current_procinfo.procdef.returndef,retreg,hreg);
                        retreg:=hreg;
                        retdef:=current_procinfo.procdef.returndef;
                      end;
                   end;
                list.concat(taillvm.op_size_reg(la_ret,retdef,retreg))
              end;
            LOC_VOID:
              begin
                { zero-sized records: return an undefined zero-sized record of
                  the correct type }
                list.concat(taillvm.op_size_undef(la_ret,retdef));
              end
            else
              { todo: complex returns }
              internalerror(2012111106);
          end;
        end;
      retpara.resetiftemp;
    end;


  procedure thlcgllvm.gen_load_uninitialized_function_result(list: TAsmList; pd: tprocdef; resdef: tdef; const resloc: tcgpara);
    begin
      if not paramanager.ret_in_param(resdef,pd) then
        begin
          case resloc.location^.loc of
            LOC_REGISTER,
            LOC_FPUREGISTER,
            LOC_MMREGISTER:
              begin
                if not llvmaggregatetype(resdef) then
                  list.concat(taillvm.op_reg_size_undef(la_bitcast,resloc.location^.register,llvmgetcgparadef(resloc,true,calleeside)))
                else
                  { bitcast doesn't work for aggregates -> just load from the
                    (uninitialised) function result memory location }
                  gen_load_loc_function_result(list,resdef,tabstractnormalvarsym(pd.funcretsym).localloc)
              end;
            { for empty record returns }
            LOC_VOID:
              ;
            else
              internalerror(2015042301);
          end;
        end;
    end;


  procedure thlcgllvm.g_overflowcheck(list: TAsmList; const Loc: tlocation; def: tdef);
    begin
      { not possible, need ovloc }
      internalerror(2012111107);
    end;


  procedure thlcgllvm.g_overflowCheck_loc(List: TAsmList; const Loc: TLocation; def: TDef; var ovloc: tlocation);
    var
      hl: tasmlabel;
    begin
      if not(cs_check_overflow in current_settings.localswitches) then
        exit;
      if ovloc.size<>OS_8 then
        internalerror(2015122504);
      current_asmdata.getjumplabel(hl);
      a_cmp_const_loc_label(list,llvmbool1type,OC_EQ,0,ovloc,hl);
      g_call_system_proc(list,'fpc_overflow',[],nil).resetiftemp;
      a_label(list,hl);
    end;


  procedure thlcgllvm.g_ptrtypecast_reg(list: TAsmList; fromdef, todef: tdef; var reg: tregister);
    var
      hreg: tregister;
    begin
      { will insert a bitcast if necessary }
      if fromdef<>todef then
        begin
          hreg:=getregisterfordef(list,todef);
          a_load_reg_reg(list,fromdef,todef,reg,hreg);
          reg:=hreg;
        end;
    end;


  procedure thlcgllvm.g_ptrtypecast_ref(list: TAsmList; fromdef, todef: tdef; var ref: treference);
    var
      hreg: tregister;
    begin
      hreg:=getaddressregister(list,todef);
      a_loadaddr_ref_reg_intern(list,fromdef,todef,ref,hreg,false);
      reference_reset_base(ref,todef,hreg,0,ref.temppos,ref.alignment,ref.volatility);
    end;


  procedure thlcgllvm.g_set_addr_nonbitpacked_field_ref(list: TAsmList; recdef: tabstractrecorddef; field: tfieldvarsym; var recref: treference);
    var
      parentdef,
      subscriptdef,
      currentstructdef,
      llvmfielddef: tdef;
      llvmfield: tllvmshadowsymtableentry;
      newbase: tregister;
      implicitpointer: boolean;
    begin
      implicitpointer:=is_implicit_pointer_object_type(recdef);
      currentstructdef:=recdef;
      { in case the field is part of a parent of the current object,
        index into the parents until we're at the parent containing the
        field; if it's an implicit pointer type, these embedded parents
        will be of the structure type of the class rather than of the
        class time itself -> one indirection fewer }
      while field.owner<>tabstractrecorddef(currentstructdef).symtable do
        begin
          { only objectdefs have parents and hence the owner of the
            fieldvarsym can be different from the current def's owner }
          parentdef:=tobjectdef(currentstructdef).childof;
          if implicitpointer then
            newbase:=getaddressregister(list,parentdef)
          else
            newbase:=getaddressregister(list,cpointerdef.getreusable(parentdef));
          recref:=make_simple_ref(list,recref,recdef);
          if implicitpointer then
            subscriptdef:=currentstructdef
          else
            subscriptdef:=cpointerdef.getreusable(currentstructdef);
          { recurse into the first field }
          list.concat(taillvm.getelementptr_reg_size_ref_size_const(newbase,subscriptdef,recref,s32inttype,0,true));
          reference_reset_base(recref,subscriptdef,newbase,field.offsetfromllvmfield,recref.temppos,newalignment(recref.alignment,field.fieldoffset),recref.volatility);
          { go to the parent }
          currentstructdef:=parentdef;
        end;
      { get the corresponding field in the llvm shadow symtable }
      llvmfield:=tabstractrecordsymtable(tabstractrecorddef(currentstructdef).symtable).llvmst[field];
      if implicitpointer then
        subscriptdef:=currentstructdef
      else
        subscriptdef:=cpointerdef.getreusable(currentstructdef);
      { load the address of that shadow field }
      newbase:=getaddressregister(list,cpointerdef.getreusable(llvmfield.def));
      recref:=make_simple_ref(list,recref,recdef);
      list.concat(taillvm.getelementptr_reg_size_ref_size_const(newbase,subscriptdef,recref,s32inttype,field.llvmfieldnr,true));
      reference_reset_base(recref,subscriptdef,newbase,field.offsetfromllvmfield,recref.temppos,newalignment(recref.alignment,llvmfield.fieldoffset+field.offsetfromllvmfield),recref.volatility);
      { in case of an 80 bits extended type, typecast from an array of 10
        bytes (used because otherwise llvm will allocate the ABI-defined
        size for extended, which is usually larger) into an extended }
      if (llvmfield.def.typ=floatdef) and
         (tfloatdef(llvmfield.def).floattype=s80real) then
        g_ptrtypecast_ref(list,cpointerdef.getreusable(carraydef.getreusable(u8inttype,10)),cpointerdef.getreusable(s80floattype),recref);
      { if it doesn't match the requested field exactly (variant record),
        adjust the type of the pointer }
      if (field.offsetfromllvmfield<>0) or
         (llvmfield.def<>field.vardef) then
        g_ptrtypecast_ref(list,cpointerdef.getreusable(llvmfield.def),cpointerdef.getreusable(field.vardef),recref);
    end;


  procedure thlcgllvm.a_loadmm_ref_reg(list: TAsmList; fromsize, tosize: tdef; const ref: treference; reg: tregister; shuffle: pmmshuffle);
    var
      href: treference;
    begin
      if shuffle=mms_movescalar then
        a_loadfpu_ref_reg(list,fromsize,tosize,ref,reg)
      else
        begin
          href:=make_simple_ref(list,ref,fromsize);
          if ref.refaddr=addr_full then
            gen_load_refaddrfull_anyreg(list,fromsize,tosize,href,reg,shuffle)
          else
            begin
              { handle aggregate loads (happens if a struct needs to be passed
                in an mmregister) }
              if (fromsize.typ in [arraydef,recorddef]) or
                 (tosize.typ in [arraydef,recorddef]) then
                begin
                  if handle_agg_load_ref_anyreg(list,fromsize,tosize,href,reg,mms_movescalar) then
                    exit;
                end;
              if fromsize<>tosize then
                g_ptrtypecast_ref(list,cpointerdef.getreusable(fromsize),cpointerdef.getreusable(tosize),href);
              { %reg = load size* %ref }
              list.concat(taillvm.op_reg_size_ref(la_load,reg,cpointerdef.getreusable(tosize),href));
            end;
        end;
    end;


  procedure thlcgllvm.a_loadmm_reg_ref(list: TAsmList; fromsize, tosize: tdef; reg: tregister; const ref: treference; shuffle: pmmshuffle);
    var
      href: treference;
    begin
      if shuffle=mms_movescalar then
        a_loadfpu_reg_ref(list,fromsize,tosize,reg,ref)
      else
        begin
          { todo }
          if fromsize<>tosize then
            internalerror(2013060220);
          href:=make_simple_ref(list,ref,tosize);
          { store tosize reg, tosize* href }
          list.concat(taillvm.op_size_reg_size_ref(la_store,tosize,reg,cpointerdef.getreusable(tosize),href))
        end;
    end;


  procedure thlcgllvm.a_loadmm_reg_reg(list: TAsmList; fromsize, tosize: tdef; reg1, reg2: tregister; shuffle: pmmshuffle);
    begin
      if shuffle=mms_movescalar then
        a_loadfpu_reg_reg(list,fromsize,tosize,reg1,reg2)
      else
        { reg2 = bitcast fromllsize reg1 to tollsize }
        list.concat(taillvm.op_reg_size_reg_size(la_bitcast,reg2,fromsize,reg1,tosize));
    end;


  procedure thlcgllvm.a_opmm_reg_reg(list: TAsmList; Op: TOpCG; size: tdef; src, dst: tregister; shuffle: pmmshuffle);
    begin
      if (op=OP_XOR) and
         (src=dst) then
        a_load_const_reg(list,size,0,dst)
      else
        { todo }
        internalerror(2013060221);
    end;


  procedure thlcgllvm.a_loadmm_intreg_reg(list: TAsmList; fromsize, tosize: tdef; intreg, mmreg: tregister; shuffle: pmmshuffle);
    begin
      internalerror(2013060222);
    end;


  procedure thlcgllvm.a_loadmm_reg_intreg(list: TAsmList; fromsize, tosize: tdef; mmreg, intreg: tregister; shuffle: pmmshuffle);
    begin
      internalerror(2013060223);
    end;


  function thlcgllvm.get_call_result_cgpara(pd: tabstractprocdef; forceresdef: tdef): tcgpara;
    var
      paraloc: pcgparalocation;
    begin
      result:=inherited;
      { we'll change the paraloc, make sure we don't modify the original one }
      if not result.temporary then
        begin
          result:=result.getcopy;
          result.temporary:=true;
        end;
      { get the LLVM representation of the function result (e.g. a
        struct with two i64 fields for a record with 4 i32 fields) }
      result.def:=llvmgetcgparadef(result,true,callerside);
      if assigned(result.location^.next) then
        begin
          { unify the result into a sinlge location; unlike for parameters,
            we are not responsible for splitting up results into multiple
            locations }
          { set the first location to the type of the function result }
          result.location^.def:=result.def;
          result.location^.size:=result.size;
          { free all extra paralocs }
          while assigned(result.location^.next) do
            begin
              paraloc:=result.location^.next^.next;
              freemem(result.location^.next);
              result.location^.next:=paraloc;
            end;
        end;
      paraloc:=result.location;
      paraloc^.def:=result.def;
      case paraloc^.loc of
        LOC_VOID:
          ;
        LOC_REGISTER,
        LOC_FPUREGISTER,
        LOC_MMREGISTER:
          begin
            paraloc^.llvmloc.loc:=paraloc^.loc;
            paraloc^.llvmloc.reg:=paraloc^.register;
            paraloc^.llvmvalueloc:=true;
          end;
        LOC_REFERENCE:
          if not paramanager.ret_in_param(pd.returndef,pd) then
            { TODO, if this can happen at all }
            internalerror(2014011901);
        else
          internalerror(2014011902);
      end;
    end;


  procedure thlcgllvm.gen_load_loc_function_result(list: TAsmList; vardef: tdef; const l: tlocation);
    var
      retlocpara: tcgpara;
    begin
      retlocpara:=get_call_result_cgpara(current_procinfo.procdef,nil);
      gen_load_loc_cgpara(list,vardef,l,retlocpara);
      retlocpara.resetiftemp;
    end;


  procedure thlcgllvm.gen_load_loc_cgpara(list: TAsmList; vardef: tdef; const l: tlocation; const cgpara: tcgpara);
    var
      memloc: tlocation;
    begin
      if not(cgpara.location^.llvmvalueloc) then
        begin
          memloc:=l;
          location_force_mem(list,memloc,vardef);
          a_loadaddr_ref_cgpara(list,vardef,memloc.reference,cgpara);
        end
      else
        inherited;
    end;


  procedure thlcgllvm.gen_load_cgpara_loc(list: TAsmList; vardef: tdef; const para: TCGPara; var destloc: tlocation; reusepara: boolean);
    var
      ploc        : pcgparalocation;
      hloc        : tlocation;
      href, href2 : treference;
      hreg        : tregister;
      fielddef,
      llvmparadef : tdef;
      index       : longint;
      offset      : pint;
      userecord   : boolean;
    begin
      { ignore e.g. empty records }
      if (para.location^.loc=LOC_VOID) then
        exit;
      { If the parameter location is reused we don't need to copy
        anything }
      if (destloc.loc=LOC_REFERENCE) and
         reusepara then
        exit;
      { get the equivalent llvm def used to pass the parameter (e.g. a record
        with two int64 fields for passing a record consisiting of 8 bytes on
        x86-64) }
      llvmparadef:=llvmgetcgparadef(para,true,calleeside);
      userecord:=
        (llvmparadef<>para.def) and
        assigned(para.location^.next);
      if userecord then
        begin
          { llvmparadef is a record in this case, with every field corresponding
            to a single paraloc }
          if destloc.loc<>LOC_REFERENCE then
            tg.gethltemp(list,llvmparadef,llvmparadef.size,tt_normal,href)
          else
            begin
              hreg:=getaddressregister(list,cpointerdef.getreusable(llvmparadef));
              a_loadaddr_ref_reg(list,vardef,cpointerdef.getreusable(llvmparadef),destloc.reference,hreg);
              reference_reset_base(href,cpointerdef.getreusable(llvmparadef),hreg,0,destloc.reference.temppos,destloc.reference.alignment,destloc.reference.volatility);
            end;
          index:=0;
          ploc:=para.location;
          repeat
            paraloctoloc(ploc,hloc);
            g_setup_load_field_by_name(list,trecorddef(llvmparadef),'F'+tostr(index),href,href2,fielddef);
            a_load_loc_ref(list,ploc^.def,fielddef,hloc,href2);
            inc(index);
            ploc:=ploc^.next;
          until not assigned(ploc);
          if destloc.loc<>LOC_REFERENCE then
            tg.ungettemp(list,href);
        end
      else
        begin
          para.check_simple_location;
          paraloctoloc(para.location,hloc);
          case destloc.loc of
            LOC_REFERENCE :
              begin
                case def2regtyp(llvmparadef) of
                  R_INTREGISTER,
                  R_ADDRESSREGISTER:
                    a_load_loc_ref(list,llvmparadef,vardef,hloc,destloc.reference);
                  R_FPUREGISTER:
                    a_loadfpu_loc_ref(list,llvmparadef,vardef,hloc,destloc.reference);
                  R_MMREGISTER:
                    a_loadmm_loc_ref(list,llvmparadef,vardef,hloc,destloc.reference,nil);
                  else
                    internalerror(2014080801);
                  end;
              end;
            LOC_REGISTER:
              begin
                a_load_loc_reg(list,llvmparadef,vardef,hloc,destloc.register);
              end;
            LOC_FPUREGISTER:
              begin
                a_loadfpu_loc_reg(list,llvmparadef,vardef,hloc,destloc.register);
              end;
            LOC_MMREGISTER:
              begin
                a_loadmm_loc_reg(list,llvmparadef,vardef,hloc,destloc.register,nil);
              end;
            { TODO other possible locations }
            else
              internalerror(2013102304);
          end;
        end;
    end;


  procedure thlcgllvm.a_jmp_flags(list: TAsmList; const f: TResFlags; l: tasmlabel);
    begin
      internalerror(2013060224);
    end;


  procedure thlcgllvm.g_flags2reg(list: TAsmList; size: tdef; const f: tresflags; reg: TRegister);
    begin
      internalerror(2013060225);
    end;


  procedure thlcgllvm.g_flags2ref(list: TAsmList; size: tdef; const f: tresflags; const ref: TReference);
    begin
      internalerror(2013060226);
    end;


  procedure thlcgllvm.a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; srcsize, dstsize: tdef; src, dst: tregister);
    begin
      internalerror(2012090201);
    end;


  procedure thlcgllvm.g_stackpointer_alloc(list: TAsmList; size: longint);
    begin
      internalerror(2012090203);
    end;


  procedure thlcgllvm.g_intf_wrapper(list: TAsmList; procdef: tprocdef; const labelname: string; ioffset: longint);
    begin
      internalerror(2012090204);
    end;


  procedure thlcgllvm.g_adjust_self_value(list: TAsmList; procdef: tprocdef; ioffset: aint);
    begin
      internalerror(2012090205);
    end;


  procedure thlcgllvm.g_local_unwind(list: TAsmList; l: TAsmLabel);
    begin
      internalerror(2012090206);
    end;


  procedure thlcgllvm.gen_stack_check_size_para(list: TAsmList);
    begin
      { this is implemented in a very hackish way, whereby first the call
        to fpc_stackcheck() is emitted, then the prolog is generated and
        registers are allocated, and finally the code to load the parameter
        is inserted before the call to fpc_stackcheck(). Since parameters are
        explicitly passed to call instructions for llvm, that does not work
        here. It could be solved by patching the call instruction later, but
        that's a lot of engineering for functionality that's only marginally
        useful at best. }
    end;


  procedure thlcgllvm.gen_stack_check_call(list: TAsmList);
    begin
      { see explanation in thlcgllvm.gen_stack_check_size_para() }
    end;


  function thlcgllvm.make_simple_ref(list: TAsmList; const ref: treference; def: tdef): treference;
    begin
      result:=make_simple_ref_ptr(list,ref,cpointerdef.getreusable(def));
    end;


  function thlcgllvm.make_simple_ref_ptr(list: TAsmList; const ref: treference; ptrdef: tdef): treference;
    var
      ptrindex: tcgint;
      hreg1,
      hreg2: tregister;
      tmpref: treference;
      pointedsize: asizeint;
    begin
      if ref.alignment=0 then
        internalerror(2016072203);
      { already simple? }
      if (not assigned(ref.symbol) or
          (ref.base=NR_NO)) and
         (ref.index=NR_NO) and
         (ref.offset=0) then
        begin
          result:=ref;
          exit;
        end;
      case ptrdef.typ of
        pointerdef:
          begin
            pointedsize:=tpointerdef(ptrdef).pointeddef.size;
            { void, formaldef }
            if pointedsize=0 then
              pointedsize:=1;
          end;
        else
          begin
            { pointedsize is only used if the offset <> 0, to see whether we
              can use getelementptr if it's an exact multiple -> set pointedsize
              to a value that will never be a multiple as we can't "index" other
              types }
            pointedsize:=ref.offset+1;
          end;
      end;
      hreg2:=getaddressregister(list,ptrdef);
      { symbol+offset or base+offset with offset a multiple of the size ->
        use getelementptr }
      if (ref.index=NR_NO) and
         (ref.offset mod pointedsize=0) then
        begin
          ptrindex:=ref.offset div pointedsize;
          if assigned(ref.symbol) then
            reference_reset_symbol(tmpref,ref.symbol,0,ref.alignment,ref.volatility)
          else
            reference_reset_base(tmpref,ptrdef,ref.base,0,ref.temppos,ref.alignment,ref.volatility);
          list.concat(taillvm.getelementptr_reg_size_ref_size_const(hreg2,ptrdef,tmpref,ptruinttype,ptrindex,assigned(ref.symbol)));
          reference_reset_base(result,ptrdef,hreg2,0,ref.temppos,ref.alignment,ref.volatility);
          exit;
        end;
      { for now, perform all calculations using plain pointer arithmetic. Later
        we can look into optimizations based on getelementptr for structured
        accesses (if only to prevent running out of virtual registers).

        Assumptions:
          * symbol/base register: always type "ptrdef"
          * index/offset: always type "ptruinttype" (llvm bitcode has no sign information, so sign doesn't matter) }
      hreg1:=getintregister(list,ptruinttype);
      if assigned(ref.symbol) then
        begin
          if ref.base<>NR_NO then
            internalerror(2012111301);
          reference_reset_symbol(tmpref,ref.symbol,0,ref.alignment,ref.volatility);
          list.concat(taillvm.getelementptr_reg_size_ref_size_const(hreg1,ptrdef,tmpref,ptruinttype,0,true));
        end
      else if ref.base<>NR_NO then
        begin
          a_load_reg_reg(list,ptrdef,ptruinttype,ref.base,hreg1);
        end
      else
        { for absolute addresses }
        a_load_const_reg(list,ptruinttype,0,hreg1);
      if ref.index<>NR_NO then
        begin
          { SSA... }
          hreg2:=getintregister(list,ptruinttype);
          a_op_reg_reg_reg(list,OP_ADD,ptruinttype,ref.index,hreg1,hreg2);
          hreg1:=hreg2;
        end;
      if ref.offset<>0 then
        begin
          hreg2:=getintregister(list,ptruinttype);
          a_op_const_reg_reg(list,OP_ADD,ptruinttype,ref.offset,hreg1,hreg2);
          hreg1:=hreg2;
        end;
      hreg2:=getaddressregister(list,ptrdef);
      a_load_reg_reg(list,ptruinttype,ptrdef,hreg1,hreg2);
      reference_reset_base(result,ptrdef,hreg2,0,ref.temppos,ref.alignment,ref.volatility);
    end;


  procedure thlcgllvm.set_call_function_result(const list: TAsmList; const pd: tabstractprocdef; const llvmretdef, hlretdef: tdef; const resval: tregister; var retpara: tcgpara);
    var
      hreg: tregister;
      rettemp: treference;
    begin
      if (not is_void(hlretdef) and
          not paramanager.ret_in_param(hlretdef, pd)) or
         pd.generate_safecall_wrapper then
        begin
          { should already be a copy, because it currently describes the llvm
            return location }
          if not retpara.temporary then
            internalerror(2014020101);
          if llvmaggregatetype(hlretdef) then
            begin
              { to ease the handling of aggregate types here, we just store
                everything to memory rather than potentially dealing with aggregates
                in "registers" }
              tg.gethltemp(list, llvmretdef, llvmretdef.size, tt_normal, rettemp);
              case def2regtyp(llvmretdef) of
                R_INTREGISTER,
                R_ADDRESSREGISTER:
                  a_load_reg_ref(list,llvmretdef,llvmretdef,resval,rettemp);
                R_FPUREGISTER:
                  a_loadfpu_reg_ref(list,llvmretdef,llvmretdef,resval,rettemp);
                R_MMREGISTER:
                  a_loadmm_reg_ref(list,llvmretdef,llvmretdef,resval,rettemp,mms_movescalar);
                else
                  ;
              end;
              { the return parameter now contains a value whose type matches the one
                that the high level code generator expects instead of the llvm shim
              }
              retpara.def:=llvmretdef;
              retpara.location^.def:=llvmretdef;
              { for llvm-specific code:  }
              retpara.location^.llvmvalueloc:=false;
              retpara.location^.llvmloc.loc:=LOC_REGISTER;
              retpara.location^.llvmloc.reg:=rettemp.base;
              { for the rest (normally not used, but cleaner to set it correclty) }
              retpara.location^.loc:=LOC_REFERENCE;
              retpara.location^.reference.index:=rettemp.base;
              retpara.location^.reference.offset:=0;
            end
          else
            begin
              retpara.def:=llvmretdef;
              retpara.Location^.def:=llvmretdef;
              retpara.location^.llvmloc.reg:=resval;
              retpara.Location^.llvmloc.loc:=retpara.location^.loc;
              retpara.Location^.llvmvalueloc:=true;
            end;
        end
      else
        retpara.location^.llvmloc.loc:=LOC_VOID;
    end;


  procedure thlcgllvm.paraloctoloc(const paraloc: pcgparalocation; out hloc: tlocation);
    begin
      case paraloc^.llvmloc.loc of
        LOC_REFERENCE:
          begin
            location_reset_ref(hloc,LOC_REFERENCE,def_cgsize(paraloc^.def),paraloc^.def.alignment,[]);
            hloc.reference.symbol:=paraloc^.llvmloc.sym;
            if paraloc^.llvmvalueloc then
              hloc.reference.refaddr:=addr_full;
          end;
        LOC_REGISTER:
          begin
            if paraloc^.llvmvalueloc then
              begin
                location_reset(hloc,LOC_REGISTER,def_cgsize(paraloc^.def));
                hloc.register:=paraloc^.llvmloc.reg;
              end
            else
              begin
                if getregtype(paraloc^.llvmloc.reg)<>R_TEMPREGISTER then
                  internalerror(2014011903);
                location_reset_ref(hloc,LOC_REFERENCE,def_cgsize(paraloc^.def),paraloc^.def.alignment,[]);
                hloc.reference.base:=paraloc^.llvmloc.reg;
              end;
          end;
        LOC_FPUREGISTER,
        LOC_MMREGISTER:
          begin
            if paraloc^.llvmvalueloc then
              begin
                location_reset(hloc,paraloc^.llvmloc.loc,def_cgsize(paraloc^.def));
                hloc.register:=paraloc^.llvmloc.reg;
              end
            else
              internalerror(2014012401);
          end
        else
          internalerror(2014010706);
      end;
    end;


  procedure thlcgllvm.varsym_set_localloc(list: TAsmList; vs: tabstractnormalvarsym);
    begin
      if cs_asm_source in current_settings.globalswitches then
        begin
          case vs.initialloc.loc of
            LOC_REFERENCE :
              begin
                if assigned(vs.initialloc.reference.symbol) then
                  list.concat(Tai_comment.Create(strpnew('Var '+vs.realname+' located at '+
                     vs.initialloc.reference.symbol.name)))
                else
                  list.concat(Tai_comment.Create(strpnew('Var '+vs.realname+' located at %tmp.'+
                     tostr(getsupreg(vs.initialloc.reference.base)))));
              end;
            else
              ;
          end;
        end;
      vs.localloc:=vs.initialloc;
    end;


  procedure thlcgllvm.paravarsym_set_initialloc_to_paraloc(vs: tparavarsym);
    var
      parasym : tasmsymbol;
    begin
      if vs.paraloc[calleeside].location^.llvmloc.loc<>LOC_REFERENCE then
        internalerror(2014010708);
      parasym:=vs.paraloc[calleeside].location^.llvmloc.sym;
      reference_reset_symbol(vs.initialloc.reference,parasym,0,vs.paraloc[calleeside].alignment,[]);
      if vs.paraloc[calleeside].location^.llvmvalueloc then
        vs.initialloc.reference.refaddr:=addr_full;
    end;


  procedure thlcgllvm.g_external_wrapper(list: TAsmList; procdef: tprocdef; const wrappername, externalname: string; global: boolean);
    var
      asmsym: TAsmSymbol;
    begin
      if po_external in procdef.procoptions then
        exit;
      asmsym:=current_asmdata.RefAsmSymbol(externalname,AT_FUNCTION);
      list.concat(taillvmalias.create(asmsym,wrappername,procdef,asmsym.bind));
    end;


  procedure create_hlcodegen_llvm;
    begin
      if not assigned(current_procinfo) or
         not(po_assembler in current_procinfo.procdef.procoptions) then
        begin
          tgobjclass:=ttgllvm;
          hlcg:=thlcgllvm.create;
          cgllvm.create_codegen
        end
      else
        begin
          tgobjclass:=orgtgclass;
          create_hlcodegen_cpu;
          { todo: handle/remove chlcgobj }
        end;
    end;

begin
  chlcgobj:=thlcgllvm;
  { this unit must initialise after hlcgobj;
    message system has not been initialised yet here }
  if not assigned(create_hlcodegen) then
    begin
      writeln('Internalerror 2018052003');
      halt(1);
    end;
  create_hlcodegen_cpu:=create_hlcodegen;
  create_hlcodegen:=@create_hlcodegen_llvm;
end.
