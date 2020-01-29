{
    Copyright (c) 2006 by Florian Klaempfl

    This unit implements the common part of the code generator for the PowerPC

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
unit cgppc;

{$i fpcdefs.inc}
  interface

    uses
       globtype,symtype,symdef,
       cgbase,cgobj,
       aasmbase,aasmdef,aasmcpu,aasmtai,aasmdata,
       cpubase,cpuinfo,cgutils,rgcpu,
       parabase;

    type
      tcgppcgen = class(tcg)
        procedure a_loadaddr_ref_cgpara(list : TAsmList;const r : treference;const paraloc : tcgpara); override;

        procedure a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; srcsize, dstsize: tcgsize; src, dst: TRegister); override;

        procedure a_call_reg(list : TAsmList;reg: tregister); override;

        { stores the contents of register reg to the memory location described by
        ref }
        procedure a_load_reg_ref(list: TAsmList; fromsize, tosize: TCGSize;
          reg: tregister; const ref: treference); override;

        procedure a_loadaddr_ref_reg(list : TAsmList;const ref : treference;r : tregister);override;

        { fpu move instructions }
        procedure a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister); override;
        procedure a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister); override;
        procedure a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference); override;

        { overflow checking }
        procedure g_overflowcheck(list: TAsmList; const l: tlocation; def: tdef);override;

        { entry code }
        procedure g_profilecode(list: TAsmList); override;

        procedure a_jmp_flags(list: TAsmList; const f: TResFlags; l: tasmlabel); override;
        procedure a_jmp_cond(list : TAsmList;cond : TOpCmp;l: tasmlabel);


        procedure g_maybe_got_init(list: TAsmList); override;

        procedure get_aix_toc_sym(list: TAsmList; const symname: string; const flags: tindsymflags; out ref: treference; force_direct_toc: boolean);
        procedure g_load_check_simple(list: TAsmList; const ref: treference; size: aint);
        procedure g_flags2reg(list: TAsmList; size: TCgSize; const f: TResFlags; reg: TRegister); override;

        { returns true if the offset of the given reference can not be  }
        { represented by a 16 bit immediate as required by some PowerPC }
        { instructions                                                  }
        function hasLargeOffset(const ref : TReference) : Boolean; inline;
        function  get_darwin_call_stub(const s: string; weak: boolean): tasmsymbol;
       protected
        function g_indirect_sym_load(list:TAsmList;const symname: string; const flags: tindsymflags): tregister; override;
        { Make sure ref is a valid reference for the PowerPC and sets the }
        { base to the value of the index if (base = R_NO).                }
        { Returns true if the reference contained a base, index and an    }
        { offset or symbol, in which case the base will have been changed }
        { to a tempreg (which has to be freed by the caller) containing   }
        { the sum of part of the original reference                       }
        function  fixref(list: TAsmList; var ref: treference): boolean;
        { contains the common code of a_load_reg_ref and a_load_ref_reg }
        procedure a_load_store(list:TAsmList;op: tasmop;reg:tregister;ref: treference);virtual;

        { creates the correct branch instruction for a given combination }
        { of asmcondflags and destination addressing mode                }
        procedure a_jmp(list: TAsmList; op: tasmop;
                        c: tasmcondflag; crval: longint; l: tasmlabel);

        function save_lr_in_prologue: boolean;

        function load_got_symbol(list : TAsmList; const symbol : string; const flags: tindsymflags) : tregister;
     end;


  TPPCAsmData = class(TAsmDataDef)
   private
    { number of entries in the TOC }
    fdirecttocentries,
    { number of fake TOC subsections we have created }
    ftocsections,
    { number of fake TOC entries in the current TOC subsection }
    fcurrenttocentries: longint;
   public
    procedure GetNextSmallTocEntry(out tocnr, entrynr: longint);
    property DirectTOCEntries: longint read fdirecttocentries write fdirecttocentries;
  end;


  TTOCAsmSymbol = class(TAsmSymbol)
   private
    { we split the toc into several sections of 32KB each, this number
      indicates which subsection this symbol is defined in }
    ftocsecnr: longint;
   public
    property TocSecNr: longint read ftocsecnr;
  end;

  const
    TOpCmp2AsmCond: Array[topcmp] of TAsmCondFlag = (C_NONE,C_EQ,C_GT,
                         C_LT,C_GE,C_LE,C_NE,C_LE,C_LT,C_GE,C_GT);
    TocSecBaseName = 'toc_table';


{$ifdef extdebug}
     function ref2string(const ref : treference) : string;
     function cgsize2string(const size : TCgSize) : string;
     function cgop2string(const op : TOpCg) : String;
{$endif extdebug}

  implementation

    uses
       {$ifdef extdebug}sysutils,{$endif}
       globals,verbose,systems,cutils,
       symconst,symsym,symtable,fmodule,
       rgobj,tgobj,cpupi,procinfo,paramgr;

{ We know that macos_direct_globals is a const boolean
  but we don't care about this warning }
{$NOTE Is macos_direct_globals still useful?}
{$WARN 6018 OFF}

{$ifdef extdebug}
     function ref2string(const ref : treference) : string;
       begin
         result := 'base : ' + inttostr(ord(ref.base)) + ' index : ' + inttostr(ord(ref.index)) + ' refaddr : ' + inttostr(ord(ref.refaddr)) + ' offset : ' + inttostr(ref.offset) + ' symbol : ';
         if (assigned(ref.symbol)) then
           result := result + ref.symbol.name;
       end;

     function cgsize2string(const size : TCgSize) : string;
       const
       (* TCgSize = (OS_NO,
                  OS_8,   OS_16,   OS_32,   OS_64,   OS_128,
                  OS_S8,  OS_S16,  OS_S32,  OS_S64,  OS_S128,
                 { single, double, extended, comp, float128 }
                  OS_F32, OS_F64,  OS_F80,  OS_C64,  OS_F128,
                 { multi-media sizes: split in byte, word, dword, ... }
                 { entities, then the signed counterparts             }
                  OS_M8,  OS_M16,  OS_M32,  OS_M64,  OS_M128,  OS_M256,  OS_M512,
                  OS_MS8, OS_MS16, OS_MS32, OS_MS64, OS_MS128, OS_MS256, OS_MS512,
                 { multi-media sizes: single-precision floating-point }
                  OS_MF32, OS_MF128, OS_MF256, OS_MF512,
                 { multi-media sizes: double-precision floating-point }
                  OS_MD64, OS_MD128, OS_MD256, OS_MD512); *)

          cgsize_strings : array[TCgSize] of string[8] = (
           'OS_NO',
           'OS_8', 'OS_16', 'OS_32', 'OS_64', 'OS_128',
           'OS_S8', 'OS_S16', 'OS_S32', 'OS_S64', 'OS_S128',
           'OS_F32', 'OS_F64', 'OS_F80', 'OS_C64', 'OS_F128',
           'OS_M8', 'OS_M16', 'OS_M32', 'OS_M64', 'OS_M128', 'OS_M256', 'OS_M512',
           'OS_MS8', 'OS_MS16', 'OS_MS32', 'OS_MS64', 'OS_MS128', 'OS_MS256', 'OS_MS512',
           'OS_MF32', 'OS_MF128', 'OS_MF256', 'OS_MF512',
           'OS_MD64', 'OS_MD128', 'OS_MD256', 'OS_MD512');
       begin
         result := cgsize_strings[size];
       end;

     function cgop2string(const op : TOpCg) : String;
       const
         opcg_strings : array[TOpCg] of string[6] = (
           'None', 'Move', 'Add', 'And', 'Div', 'IDiv', 'IMul', 'Mul',
           'Neg', 'Not', 'Or', 'Sar', 'Shl', 'Shr', 'Sub', 'Xor', 'Rol', 'Ror'
         );
       begin
         result := opcg_strings[op];
       end;
{$endif extdebug}


    function tcgppcgen.hasLargeOffset(const ref : TReference) : Boolean;
      begin
        result := aword(ref.offset-low(smallint)) > high(smallint)-low(smallint);
      end;


    function tcgppcgen.save_lr_in_prologue: boolean;
      begin
        result:=
        (not (po_assembler in current_procinfo.procdef.procoptions) and
         ((pi_do_call in current_procinfo.flags) or
          (cs_profile in init_settings.moduleswitches)))  or
        ([cs_lineinfo,cs_debuginfo] * current_settings.moduleswitches <> []);
      end;


    procedure tcgppcgen.a_loadaddr_ref_cgpara(list : TAsmList;const r : treference;const paraloc : tcgpara);
      var
        ref: treference;
        tmpreg: tregister;

      begin
        paraloc.check_simple_location;
        paramanager.allocparaloc(list,paraloc.location);
        case paraloc.location^.loc of
           LOC_REGISTER,LOC_CREGISTER:
             a_loadaddr_ref_reg(list,r,paraloc.location^.register);
           LOC_REFERENCE:
             begin
               reference_reset(ref,paraloc.alignment,[]);
               ref.base := paraloc.location^.reference.index;
               ref.offset := paraloc.location^.reference.offset;
               tmpreg := rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);
               a_loadaddr_ref_reg(list,r,tmpreg);
               a_load_reg_ref(list,OS_ADDR,OS_ADDR,tmpreg,ref);
             end;
           else
             internalerror(2002080701);
        end;
      end;


    procedure tcgppcgen.a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; srcsize, dstsize: tcgsize; src, dst: TRegister);
      var
        tmpreg: tregister;
        cntlzop: tasmop;
        bitsizem1: longint;
      begin
        { we only have a cntlz(w|d) instruction, which corresponds to bsr(x)
         (well, regsize_in_bits - bsr(x), as x86 numbers bits in reverse).
          Fortunately, bsf(x) can be calculated easily based on that, see
          "Figure 5-13. Number of Powers of 2 Code Sequence" in the PowerPC
          Compiler Writer's Guide
        }
        if srcsize in [OS_64,OS_S64] then
          begin
{$ifdef powerpc64}
            cntlzop:=A_CNTLZD;
{$else}
            internalerror(2015022601);
{$endif}
            bitsizem1:=63;
          end
        else
          begin
            cntlzop:=A_CNTLZW;
            bitsizem1:=31;
          end;
        if not reverse then
          begin
            { cntlzw(src and -src) }
            tmpreg:=getintregister(list,srcsize);
            { don't use a_op_reg_reg, as this will adjust the result
              after the neg in case of a non-32/64 bit operation, which
              is not necessary since we're only using it as an
              AND-mask }
            list.concat(taicpu.op_reg_reg(A_NEG,tmpreg,src));
            a_op_reg_reg(list,OP_AND,srcsize,src,tmpreg);
          end
        else
          tmpreg:=src;
        { count leading zeroes }
        list.concat(taicpu.op_reg_reg(cntlzop,dst,tmpreg));
        { (bitsize-1) - cntlz (which is 32/64 in case src was 0) }
        list.concat(taicpu.op_reg_reg_const(A_SUBFIC,dst,dst,bitsizem1));
        { set to 255 is source was 0 }
        a_op_const_reg(list,OP_AND,dstsize,255,dst);
      end;


    procedure tcgppcgen.g_maybe_got_init(list: TAsmList);
      var
         instr: taicpu;
         cond: tasmcond;
        savedlr: boolean;
      begin
        if not(po_assembler in current_procinfo.procdef.procoptions) then
          begin
            if (cs_create_pic in current_settings.moduleswitches) and
               (pi_needs_got in current_procinfo.flags) then
              case target_info.system of
                system_powerpc_darwin,
                system_powerpc64_darwin:
                  begin
                    savedlr:=save_lr_in_prologue;
                    if not savedlr then
                      list.concat(taicpu.op_reg_reg(A_MFSPR,NR_R0,NR_LR));
                    fillchar(cond,sizeof(cond),0);
                    cond.simple:=false;
                    cond.bo:=20;
                    cond.bi:=31;
                    instr:=taicpu.op_sym(A_BCL,current_procinfo.CurrGOTLabel);
                    instr.setcondition(cond);
                    list.concat(instr);
                    a_label(list,current_procinfo.CurrGOTLabel);
                    a_reg_alloc(list,current_procinfo.got);
                    list.concat(taicpu.op_reg_reg(A_MFSPR,current_procinfo.got,NR_LR));
                    if not savedlr or
                       { in the following case lr is saved, but not restored }
                       { (happens e.g. when generating debug info for leaf   }
                       { procedures)                                         }
                       not(pi_do_call in current_procinfo.flags) then
                      list.concat(taicpu.op_reg_reg(A_MTSPR,NR_LR,NR_R0));
                  end;
                else
                  ;
              end;
          end;
      end;


    function tcgppcgen.g_indirect_sym_load(list: TAsmList; const symname: string; const flags: tindsymflags): tregister;
      begin
        case target_info.system of
          system_powerpc_aix,
          system_powerpc64_aix:
            result:=load_got_symbol(list,symname,flags);
          else
            result:=inherited;
        end;
      end;


    function tcgppcgen.get_darwin_call_stub(const s: string; weak: boolean): tasmsymbol;
      var
        stubname: string;
        instr: taicpu;
        href: treference;
        l1: tasmsymbol;
        localgotlab: tasmlabel;
        cond: tasmcond;
        stubalign: byte;
      begin
        { function declared in the current unit? }
        { doesn't work correctly, because this will also return a hit if we }
        { previously took the address of an external procedure. It doesn't  }
        { really matter, the linker will remove all unnecessary stubs.      }
        stubname := 'L'+s+'$stub';
        result := current_asmdata.getasmsymbol(stubname);
        if assigned(result) then
          exit;

        if current_asmdata.asmlists[al_imports]=nil then
          current_asmdata.asmlists[al_imports]:=TAsmList.create;

        if (cs_create_pic in current_settings.moduleswitches) then
          stubalign:=32
        else
          stubalign:=16;
        new_section(current_asmdata.asmlists[al_imports],sec_stub,'',stubalign);
        result := current_asmdata.DefineAsmSymbol(stubname,AB_LOCAL,AT_FUNCTION,voidcodepointertype);
        current_asmdata.asmlists[al_imports].concat(Tai_symbol.Create(result,0));
        { register as a weak symbol if necessary }
        if weak then
          current_asmdata.weakrefasmsymbol(s,AT_FUNCTION);
        current_asmdata.asmlists[al_imports].concat(tai_directive.create(asd_indirect_symbol,s));
        l1 := current_asmdata.DefineAsmSymbol('L'+s+'$lazy_ptr',AB_LOCAL,AT_DATA,voidpointertype);
        reference_reset_symbol(href,l1,0,sizeof(pint),[]);
        href.refaddr := addr_higha;
        if (cs_create_pic in current_settings.moduleswitches) then
          begin
            current_asmdata.getjumplabel(localgotlab);
            href.relsymbol:=localgotlab;
            fillchar(cond,sizeof(cond),0);
            cond.simple:=false;
            cond.bo:=20;
            cond.bi:=31;
            current_asmdata.asmlists[al_imports].concat(taicpu.op_reg(A_MFLR,NR_R0));
            instr:=taicpu.op_sym(A_BCL,localgotlab);
            instr.setcondition(cond);
            current_asmdata.asmlists[al_imports].concat(instr);
            a_label(current_asmdata.asmlists[al_imports],localgotlab);
            current_asmdata.asmlists[al_imports].concat(taicpu.op_reg(A_MFLR,NR_R11));
            current_asmdata.asmlists[al_imports].concat(taicpu.op_reg_reg_ref(A_ADDIS,NR_R11,NR_R11,href));
            current_asmdata.asmlists[al_imports].concat(taicpu.op_reg(A_MTLR,NR_R0));
          end
        else
          current_asmdata.asmlists[al_imports].concat(taicpu.op_reg_ref(A_LIS,NR_R11,href));
        href.refaddr := addr_low;
        href.base := NR_R11;
{$ifndef cpu64bitaddr}
        current_asmdata.asmlists[al_imports].concat(taicpu.op_reg_ref(A_LWZU,NR_R12,href));
{$else cpu64bitaddr}
        { darwin/ppc64 uses a 32 bit absolute address here, strange... }
        current_asmdata.asmlists[al_imports].concat(taicpu.op_reg_ref(A_LDU,NR_R12,href));
{$endif cpu64bitaddr}
        current_asmdata.asmlists[al_imports].concat(taicpu.op_reg(A_MTCTR,NR_R12));
        current_asmdata.asmlists[al_imports].concat(taicpu.op_none(A_BCTR));
        new_section(current_asmdata.asmlists[al_imports],sec_data_lazy,'',sizeof(pint));
        current_asmdata.asmlists[al_imports].concat(Tai_symbol.Create(l1,0));
        current_asmdata.asmlists[al_imports].concat(tai_directive.create(asd_indirect_symbol,s));
        current_asmdata.asmlists[al_imports].concat(tai_const.createname('dyld_stub_binding_helper',0));
      end;


     procedure tcgppcgen.a_loadaddr_ref_reg(list : TAsmList;const ref : treference;r : tregister);

       var
         ref2, tmpref: treference;

       begin
         ref2 := ref;
         fixref(list,ref2);
         if assigned(ref2.symbol) then
           begin
             if target_info.system = system_powerpc_macos then
               begin
                 if macos_direct_globals then
                   begin
                     reference_reset(tmpref,ref2.alignment,ref2.volatility);
                     tmpref.offset := ref2.offset;
                     tmpref.symbol := ref2.symbol;
                     tmpref.base := NR_NO;
                     list.concat(taicpu.op_reg_reg_ref(A_ADDI,r,NR_RTOC,tmpref));
                   end
                 else
                   begin
                     reference_reset(tmpref,ref2.alignment,ref2.volatility);
                     tmpref.symbol := ref2.symbol;
                     tmpref.offset := 0;
                     tmpref.base := NR_RTOC;
                     list.concat(taicpu.op_reg_ref(A_LWZ,r,tmpref));

                     if ref2.offset<>0 then
                       a_op_const_reg(list,OP_ADD,OS_ADDR,ref2.offset,r);
                   end;

                 if ref2.base <> NR_NO then
                   list.concat(taicpu.op_reg_reg_reg(A_ADD,r,r,ref2.base));

                 //list.concat(tai_comment.create(strpnew('*** a_loadaddr_ref_reg')));
               end
             else
               begin

                 { add the symbol's value to the base of the reference, and if the }
                 { reference doesn't have a base, create one                       }
                 reference_reset(tmpref,ref2.alignment,ref2.volatility);
                 tmpref.offset := ref2.offset;
                 tmpref.symbol := ref2.symbol;
                 tmpref.relsymbol := ref2.relsymbol;
                 tmpref.refaddr := addr_higha;
                 if ref2.base<> NR_NO then
                   begin
                     list.concat(taicpu.op_reg_reg_ref(A_ADDIS,r,
                       ref2.base,tmpref));
                   end
                 else
                   list.concat(taicpu.op_reg_ref(A_LIS,r,tmpref));
                 tmpref.base := NR_NO;
                 tmpref.refaddr := addr_low;
                 { can be folded with one of the next instructions by the }
                 { optimizer probably                                     }
                 list.concat(taicpu.op_reg_reg_ref(A_ADDI,r,r,tmpref));
               end
           end
         else if ref2.offset <> 0 Then
           if ref2.base <> NR_NO then
             a_op_const_reg_reg(list,OP_ADD,OS_ADDR,ref2.offset,ref2.base,r)
           { FixRef makes sure that "(ref.index <> R_NO) and (ref.offset <> 0)" never}
           { occurs, so now only ref.offset has to be loaded                         }
           else
             a_load_const_reg(list,OS_ADDR,ref2.offset,r)
         else if ref2.index <> NR_NO Then
           list.concat(taicpu.op_reg_reg_reg(A_ADD,r,ref2.base,ref2.index))
         else if (ref2.base <> NR_NO) and
                 (r <> ref2.base) then
           a_load_reg_reg(list,OS_ADDR,OS_ADDR,ref2.base,r)
         else
           list.concat(taicpu.op_reg_const(A_LI,r,0));
       end;



    { calling a procedure by address }
    procedure tcgppcgen.a_call_reg(list : TAsmList;reg: tregister);
      var
        tmpref: treference;
        tmpreg: tregister;
        toc_offset: longint;
      begin
        tmpreg:=NR_NO;
        if target_info.system in systems_aix then
          begin
            { load function address in R0, and swap "reg" for R0 }
            reference_reset_base(tmpref,reg,0,ctempposinvalid,sizeof(pint),[]);
            a_load_ref_reg(list,OS_ADDR,OS_ADDR,tmpref,NR_R0);
            tmpreg:=reg;
            { no need to allocate/free R0, is already allocated by call node
              because it's a volatile register }
            reg:=NR_R0;
            { save current TOC }
            reference_reset_base(tmpref,NR_STACK_POINTER_REG,LA_RTOC_AIX,ctempposinvalid,sizeof(pint),[]);
            a_load_reg_ref(list,OS_ADDR,OS_ADDR,NR_RTOC,tmpref);
          end;
        list.concat(taicpu.op_reg(A_MTCTR,reg));
        if target_info.system in systems_aix then
          begin
            { load target TOC and possible link register }
            reference_reset_base(tmpref,tmpreg,sizeof(pint),ctempposinvalid,sizeof(pint),[]);
            a_load_ref_reg(list,OS_ADDR,OS_ADDR,tmpref,NR_RTOC);
            tmpref.offset:=2*sizeof(pint);
            a_load_ref_reg(list,OS_ADDR,OS_ADDR,tmpref,NR_R11);
          end
        else if target_info.abi=abi_powerpc_elfv2 then
          begin
            { save current TOC }
            reference_reset_base(tmpref,NR_STACK_POINTER_REG,LA_RTOC_ELFV2,ctempposinvalid,sizeof(pint),[]);
            a_load_reg_ref(list,OS_ADDR,OS_ADDR,NR_RTOC,tmpref);
            { functions must be called via R12 for this ABI }
            if reg<>NR_R12 then
              begin
                getcpuregister(list,NR_R12);
                a_load_reg_reg(list,OS_ADDR,OS_ADDR,reg,NR_R12)
              end;
          end;
        list.concat(taicpu.op_none(A_BCTRL));
        if (target_info.system in systems_aix) or
           (target_info.abi=abi_powerpc_elfv2) then
          begin
            if (target_info.abi=abi_powerpc_elfv2) and
               (reg<>NR_R12) then
              ungetcpuregister(list,NR_R12);
            { restore our TOC }
            if target_info.system in systems_aix then
              toc_offset:=LA_RTOC_AIX
            else
              toc_offset:=LA_RTOC_ELFV2;
            reference_reset_base(tmpref,NR_STACK_POINTER_REG,toc_offset,ctempposinvalid,sizeof(pint),[]);
            a_load_ref_reg(list,OS_ADDR,OS_ADDR,tmpref,NR_RTOC);
          end;
        include(current_procinfo.flags,pi_do_call);
      end;


    procedure tcgppcgen.a_load_reg_ref(list: TAsmList; fromsize, tosize: TCGSize;
      reg: tregister; const ref: treference);

    const
      StoreInstr: array[OS_8..OS_INT, boolean, boolean] of TAsmOp =
      { indexed? updating?}
      (((A_STB, A_STBU), (A_STBX, A_STBUX)),
        ((A_STH, A_STHU), (A_STHX, A_STHUX)),
        ((A_STW, A_STWU), (A_STWX, A_STWUX))
{$ifdef cpu64bitalu}
        ,
        ((A_STD, A_STDU), (A_STDX, A_STDUX))
{$endif cpu64bitalu}
        );
    var
      ref2: TReference;
      op: TAsmOp;
    begin
      if not (fromsize in [OS_8..OS_INT,OS_S8..OS_SINT]) then
        internalerror(2002090904);
      if not (tosize in [OS_8..OS_INT,OS_S8..OS_SINT]) then
        internalerror(2002090905);

      if tosize in [OS_S8..OS_SINT] then
        { storing is the same for signed and unsigned values }
        tosize := tcgsize(ord(tosize) - (ord(OS_S8) - ord(OS_8)));

      ref2 := ref;
      fixref(list, ref2);

      op := storeinstr[tcgsize2unsigned[tosize], ref2.index <> NR_NO, false];
      a_load_store(list, op, reg, ref2);
    end;



     procedure tcgppcgen.a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister);

       var
         op: tasmop;
         instr: taicpu;
       begin
         if not(fromsize in [OS_F32,OS_F64]) or
            not(tosize in [OS_F32,OS_F64]) then
           internalerror(2006123110);
         if (tosize < fromsize) then
           op:=A_FRSP
         else
           op:=A_FMR;
         instr := taicpu.op_reg_reg(op,reg2,reg1);
         list.concat(instr);
         if (op = A_FMR) then
           rg[R_FPUREGISTER].add_move_instruction(instr);
       end;


     procedure tcgppcgen.a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister);

       const
         FpuLoadInstr: Array[OS_F32..OS_F64,boolean, boolean] of TAsmOp =
                          { indexed? updating?}
                    (((A_LFS,A_LFSU),(A_LFSX,A_LFSUX)),
                     ((A_LFD,A_LFDU),(A_LFDX,A_LFDUX)));
       var
         op: tasmop;
         ref2: treference;

       begin
         if target_info.system in systems_aix then
           g_load_check_simple(list,ref,65536);
         if not(fromsize in [OS_F32,OS_F64]) or
            not(tosize in [OS_F32,OS_F64]) then
           internalerror(200201121);
         ref2 := ref;
         fixref(list,ref2);
         op := fpuloadinstr[fromsize,ref2.index <> NR_NO,false];
         a_load_store(list,op,reg,ref2);
         if (fromsize > tosize) then
           a_loadfpu_reg_reg(list,fromsize,tosize,reg,reg);
       end;


     procedure tcgppcgen.a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference);

       const
         FpuStoreInstr: Array[OS_F32..OS_F64,boolean, boolean] of TAsmOp =
                            { indexed? updating?}
                    (((A_STFS,A_STFSU),(A_STFSX,A_STFSUX)),
                     ((A_STFD,A_STFDU),(A_STFDX,A_STFDUX)));
       var
         op: tasmop;
         ref2: treference;
         reg2: tregister;

       begin
         if not(fromsize in [OS_F32,OS_F64]) or
            not(tosize in [OS_F32,OS_F64]) then
           internalerror(200201122);
         ref2 := ref;
         fixref(list,ref2);
         op := fpustoreinstr[tosize,ref2.index <> NR_NO,false];

         { some PPCs have a bug whereby storing a double to memory  }
         { as single corrupts the value -> convert double to single }
         { first (bug confirmed on some G4s, but not on G5s)        }
         if (tosize < fromsize) and
            (current_settings.cputype < cpu_PPC970) then
           begin
             reg2:=getfpuregister(list,tosize);
             a_loadfpu_reg_reg(list,fromsize,tosize,reg,reg2);
             reg:=reg2;
           end;
         a_load_store(list,op,reg,ref2);
       end;


  procedure tcgppcgen.g_overflowcheck(list: TAsmList; const l: tlocation; def: tdef);
    var
      hl : tasmlabel;
      flags : TResFlags;
    begin
      if not(cs_check_overflow in current_settings.localswitches) then
        exit;
      current_asmdata.getjumplabel(hl);
      if not ((def.typ=pointerdef) or
             ((def.typ=orddef) and
              (torddef(def).ordtype in [u64bit,u16bit,u32bit,u8bit,uchar,
                                        pasbool1,pasbool8,pasbool16,pasbool32,pasbool64]))) then
        begin
          if (current_settings.optimizecputype >= cpu_ppc970) or
             (current_settings.cputype >= cpu_ppc970) then
            begin
              { ... instructions setting overflow flag ...
              mfxerf R0
              mtcrf 128, R0
              ble cr0, label }
              list.concat(taicpu.op_reg(A_MFXER, NR_R0));
              list.concat(taicpu.op_const_reg(A_MTCRF, 128, NR_R0));
              flags.cr := RS_CR0;
              flags.flag := F_LE;
              a_jmp_flags(list, flags, hl);
            end
          else
            begin
              list.concat(taicpu.op_reg(A_MCRXR,NR_CR7));
              a_jmp(list,A_BC,C_NO,7,hl)
            end;
        end
      else
        a_jmp_cond(list,OC_AE,hl);
      a_call_name(list,'FPC_OVERFLOW',false);
      a_label(list,hl);
    end;


  procedure tcgppcgen.g_profilecode(list: TAsmList);
    var
      paraloc1 : tcgpara;
      pd : tprocdef;
    begin
      if (target_info.system in [system_powerpc_darwin]) then
        begin
          pd:=search_system_proc('mcount');
          paraloc1.init;
          paramanager.getcgtempparaloc(list,pd,1,paraloc1);
          a_load_reg_cgpara(list,OS_ADDR,NR_R0,paraloc1);
          paramanager.freecgpara(list,paraloc1);
          paraloc1.done;
          allocallcpuregisters(list);
          a_call_name(list,'mcount',false);
          deallocallcpuregisters(list);
          a_reg_dealloc(list,NR_R0);
        end;
    end;


  procedure tcgppcgen.a_jmp_flags(list: TAsmList; const f: TResFlags; l: tasmlabel);
    var
      c: tasmcond;
      f2: TResFlags;
      testbit: longint;
    begin
      f2:=f;
      testbit:=(f.cr-RS_CR0)*4;
      case f.flag of
        F_FA:
          f2.flag:=F_GT;
        F_FAE:
          begin
            list.concat(taicpu.op_const_const_const(A_CROR,testbit+1,testbit+1,testbit+2));
            f2.flag:=F_GT;
          end;
        F_FB:
          f2.flag:=F_LT;
        F_FBE:
          begin
            list.concat(taicpu.op_const_const_const(A_CROR,testbit,testbit,testbit+2));
            f2.flag:=F_LT;
          end;
        else
          ;
      end;
      c := flags_to_cond(f2);
      a_jmp(list,A_BC,c.cond,c.cr-RS_CR0,l);
    end;


  procedure tcgppcgen.a_jmp_cond(list : TAsmList;cond : TOpCmp; l: tasmlabel);
    begin
      a_jmp(list,A_BC,TOpCmp2AsmCond[cond],0,l);
    end;


 procedure tcgppcgen.a_jmp(list: TAsmList; op: tasmop; c: tasmcondflag;
             crval: longint; l: tasmlabel);
   var
     p: taicpu;

   begin
     p := taicpu.op_sym(op,l);
     if op <> A_B then
       create_cond_norm(c,crval,p.condition);
     p.is_jmp := true;
     list.concat(p)
   end;



  function tcgppcgen.load_got_symbol(list: TAsmList; const symbol : string; const flags: tindsymflags) : tregister;
    var
      l: tasmsymbol;
      ref: treference;
    begin
      if target_info.system=system_powerpc64_linux then
        begin
          l:=current_asmdata.getasmsymbol(symbol);
          reference_reset_symbol(ref,l,0,sizeof(pint),[]);
          ref.base:=NR_RTOC;
          ref.refaddr:=addr_pic;
        end
      else if target_info.system in systems_aix then
        get_aix_toc_sym(list,symbol,flags,ref,false)
      else
        internalerror(2007102010);

      result := getaddressregister(list);
{$ifdef cpu64bitaddr}
      list.concat(taicpu.op_reg_ref(A_LD, result, ref));
{$else cpu64bitaddr}
      list.concat(taicpu.op_reg_ref(A_LWZ, result, ref));
{$endif cpu64bitaddr}
    end;


  procedure tcgppcgen.get_aix_toc_sym(list: TAsmList; const symname: string; const flags: tindsymflags; out ref: treference; force_direct_toc: boolean);
    const
      { The TOC on AIX is limited to 32KB worth of entries on AIX. If you need
        more entries, you have to add a level of indirection. In some cases,
        it's not possible to do this (e.g. assembler code). So by default, we
        use direct TOC entries until we're 500 from the maximum, and then start
        using indirect TOC entries. }
      AutoDirectTOCLimit = (high(smallint) div sizeof(pint)) - 500;
    var
      tmpref: treference;
      nlsymname: string;
      newsymname: ansistring;
      sym: TAsmSymbol;
      tocsym: TTOCAsmSymbol;
      tocnr,
      entrynr: longint;
      tmpreg: tregister;
    begin
      { all global symbol accesses always must be done via the TOC }
      nlsymname:='LC..'+symname;
      reference_reset_symbol(ref,current_asmdata.getasmsymbol(nlsymname),0,sizeof(pint),[]);
      if (assigned(ref.symbol) and
          not(ref.symbol is TTOCAsmSymbol)) or
         (not(ts_small_toc in current_settings.targetswitches) and
          (TPPCAsmData(current_asmdata).DirectTOCEntries<AutoDirectTOCLimit)) or
         force_direct_toc then
        begin
          ref.refaddr:=addr_pic_no_got;
          ref.base:=NR_RTOC;
          if not assigned(ref.symbol) then
            begin
              TPPCAsmData(current_asmdata).DirectTOCEntries:=TPPCAsmData(current_asmdata).DirectTOCEntries+1;
              new_section(current_asmdata.AsmLists[al_picdata],sec_toc,'',sizeof(pint));
              ref.symbol:=current_asmdata.DefineAsmSymbol(nlsymname,AB_LOCAL,AT_DATA,voidpointertype);
              current_asmdata.asmlists[al_picdata].concat(tai_symbol.create(ref.symbol,0));
              { do not assign the result of these statements to ref.symbol: the
                access must be done via the LC..symname symbol; these are just
                to define the symbol that's being accessed as either weak or
                not }
              if not(is_weak in flags) then
                current_asmdata.RefAsmSymbol(symname,AT_DATA)
              else if is_data in flags then
                current_asmdata.WeakRefAsmSymbol(symname,AT_DATA)
              else
                current_asmdata.WeakRefAsmSymbol('.'+symname,AT_DATA);
              newsymname:=ReplaceForbiddenAsmSymbolChars(symname);
              current_asmdata.asmlists[al_picdata].concat(tai_directive.Create(asd_toc_entry,newsymname+'[TC],'+newsymname));
            end;
        end
      else
        begin
          if not assigned(ref.symbol) then
            begin
              TPPCAsmData(current_asmdata).GetNextSmallTocEntry(tocnr,entrynr);
              { new TOC entry? }
              if entrynr=0 then
                begin
                  { create new toc entry that contains the address of the next
                    table of addresses }
                  get_aix_toc_sym(list,'tocsubtable'+tostr(tocnr),[is_data],tmpref,true);
                  sym:=tmpref.symbol;
                  { base address for this batch of toc table entries that we'll
                    put in a data block instead }
                  new_section(current_asmdata.AsmLists[al_indirectpicdata],sec_rodata,'',sizeof(pint));
                  sym:=current_asmdata.DefineAsmSymbol('tocsubtable'+tostr(tocnr),AB_LOCAL,AT_DATA,voidpointertype);
                  current_asmdata.asmlists[al_indirectpicdata].concat(tai_symbol.create(sym,0));
                end;
              { add the reference to the actual symbol inside the tocsubtable }
              if not(is_weak in flags) then
                current_asmdata.RefAsmSymbol(symname,AT_DATA)
              else if is_data in flags then
                current_asmdata.WeakRefAsmSymbol(symname,AT_DATA)
              else
                current_asmdata.WeakRefAsmSymbol('.'+symname,AT_DATA);
              tocsym:=TTOCAsmSymbol(current_asmdata.DefineAsmSymbolByClass(TTOCAsmSymbol,nlsymname,AB_LOCAL,AT_DATA,voidpointertype));
              ref.symbol:=tocsym;
              tocsym.ftocsecnr:=tocnr;
              current_asmdata.asmlists[al_indirectpicdata].concat(tai_symbol.create(tocsym,0));
              newsymname:=ReplaceForbiddenAsmSymbolChars(symname);
              sym:=current_asmdata.RefAsmSymbol(newsymname,AT_DATA);
              current_asmdata.asmlists[al_indirectpicdata].concat(tai_const.Create_sym(sym));
            end;
          { first load the address of the table from the TOC }
          get_aix_toc_sym(list,'tocsubtable'+tostr(TTOCAsmSymbol(ref.symbol).ftocsecnr),[is_data],tmpref,true);
          tmpreg:=getaddressregister(list);
          a_load_ref_reg(list,OS_ADDR,OS_ADDR,tmpref,tmpreg);
          { and now set up the address of the entry, relative to the start of
            the table }
          ref.base:=tmpreg;
          ref.refaddr:=addr_pic;
          ref.relsymbol:=current_asmdata.GetAsmSymbol('tocsubtable'+tostr(TTOCAsmSymbol(ref.symbol).ftocsecnr));
        end;
    end;


  procedure tcgppcgen.g_load_check_simple(list: TAsmList; const ref: treference; size: aint);
    var
      reg: tregister;
      lab: tasmlabel;
    begin
      if not(cs_check_low_addr_load in current_settings.localswitches) then
        exit;
      { this is mainly for AIX, which does not trap loads from address 0. A
        global symbol (if not weak) will always map to a proper address, and
        the same goes for stack addresses -> skip }
      if assigned(ref.symbol) and
         (ref.symbol.bind<>AB_WEAK_EXTERNAL) then
        exit;
      if (ref.base=NR_STACK_POINTER_REG) or
         (ref.index=NR_STACK_POINTER_REG) or
         (assigned(current_procinfo) and
          ((ref.base=current_procinfo.framepointer) or
           (ref.index=current_procinfo.framepointer))) then
        exit;
      if assigned(ref.symbol) or
         (ref.offset<>0) or
         ((ref.base<>NR_NO) and (ref.index<>NR_NO)) then
        begin
          { can't allocate register, also used in wrappers and the like }
          reg:=NR_R0;
          a_reg_alloc(list,reg);
          a_loadaddr_ref_reg(list,ref,reg);
        end
      else if ref.base<>NR_NO then
        reg:=ref.base
      else
        reg:=ref.index;
      current_asmdata.getjumplabel(lab);
      if reg=NR_R0 then
        a_reg_dealloc(list,reg);
      a_cmp_const_reg_label(list,OS_ADDR,OC_A,size-1,reg,lab);
      a_call_name(list,'FPC_INVALIDPOINTER',false);
      a_label(list,lab);
    end;


    procedure tcgppcgen.g_flags2reg(list: TAsmList; size: TCgSize; const f: TResFlags; reg: TRegister);
      var
        testbit: byte;
        bitvalue: boolean;
        hreg: tregister;
        needsecondreg: boolean;
      begin
        hreg:=NR_NO;
        needsecondreg:=false;
        { get the bit to extract from the conditional register + its requested value (0 or 1) }
        testbit := ((f.cr - RS_CR0) * 4);
        case f.flag of
          F_EQ, F_NE:
            begin
              inc(testbit, 2);
              bitvalue := f.flag = F_EQ;
            end;
          F_LT, F_GE, F_FB:
            begin
              bitvalue := f.flag in [F_LT,F_FB];
            end;
          F_GT, F_LE, F_FA:
            begin
              inc(testbit);
              bitvalue := f.flag in [F_GT,F_FA];
            end;
          F_FAE:
            begin
              inc(testbit);
              bitvalue:=true;
              needsecondreg:=true;
            end;
          F_FBE:
            begin
              bitvalue:=true;
              needsecondreg:=true;
            end;
        else
          internalerror(200112261);
        end;
        { load the conditional register in the destination reg }
        list.concat(taicpu.op_reg(A_MFCR, reg));
        { we will move the bit that has to be tested to bit 0 by rotating left }
        testbit := (testbit + 1) and 31;

        { for floating-point >= and <=, extract equality bit first }
        if needsecondreg then
          begin
            hreg:=getintregister(list,OS_INT);
            list.concat(taicpu.op_reg_reg_const_const_const(
              A_RLWINM,hreg,reg,(((f.cr-RS_CR0)*4)+3) and 31,31,31));
          end;

        { extract bit }
        list.concat(taicpu.op_reg_reg_const_const_const(
          A_RLWINM,reg,reg,testbit,31,31));

        if needsecondreg then
          list.concat(taicpu.op_reg_reg_reg(A_OR,reg,hreg,reg))
        { if we need the inverse, xor with 1 }
        else if not bitvalue then
          list.concat(taicpu.op_reg_reg_const(A_XORI, reg, reg, 1));
      end;


    function tcgppcgen.fixref(list: TAsmList; var ref: treference): boolean;
      var
        tmpreg: tregister;
      begin
        result := false;

        { Avoid recursion. }
        if (ref.refaddr in [addr_pic,addr_pic_no_got]) then
          exit;

        {$IFDEF EXTDEBUG}
        list.concat(tai_comment.create(strpnew('fixref0 ' + ref2string(ref))));
        {$ENDIF EXTDEBUG}
        if (target_info.system in [system_powerpc_darwin,system_powerpc64_darwin]) and
           assigned(ref.symbol) and
           not assigned(ref.relsymbol) and
           ((ref.symbol.bind in [AB_EXTERNAL,AB_WEAK_EXTERNAL,AB_PRIVATE_EXTERN,AB_COMMON]) or
            (cs_create_pic in current_settings.moduleswitches))then
          begin
            if (ref.symbol.bind in [AB_EXTERNAL,AB_WEAK_EXTERNAL,AB_PRIVATE_EXTERN,AB_COMMON]) or
               ((target_info.system=system_powerpc64_darwin) and
                (ref.symbol.bind=AB_GLOBAL)) then
              begin
                tmpreg := g_indirect_sym_load(list,ref.symbol.name,asmsym2indsymflags(ref.symbol));
                ref.symbol:=nil;
              end
            else
              begin
                include(current_procinfo.flags,pi_needs_got);
                tmpreg := getaddressregister(list);
                a_load_reg_reg(list,OS_ADDR,OS_ADDR,current_procinfo.got,tmpreg);
                if assigned(ref.relsymbol) then
                  internalerror(2007093501);
                ref.relsymbol := current_procinfo.CurrGOTLabel;
              end;
            if (ref.base = NR_NO) then
              ref.base := tmpreg
            else if (ref.index = NR_NO) then
              ref.index := tmpreg
            else
              begin
                list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg,ref.base,tmpreg));
                ref.base := tmpreg;
              end;
          end;

        { if we have to create PIC, add the symbol to the TOC/GOT }
        if (((target_info.system = system_powerpc64_linux) and
             (cs_create_pic in current_settings.moduleswitches)) or
            (target_info.system in systems_aix)) and
           (assigned(ref.symbol) and
            not assigned(ref.relsymbol)) then
          begin
            tmpreg := load_got_symbol(list, ref.symbol.name, asmsym2indsymflags(ref.symbol));
            if (ref.base = NR_NO) then
              ref.base := tmpreg
            else if (ref.index = NR_NO) then
              ref.index := tmpreg
            else begin
              a_op_reg_reg_reg(list, OP_ADD, OS_ADDR, ref.base, tmpreg, tmpreg);
              ref.base := tmpreg;
            end;
            ref.symbol := nil;
            {$IFDEF EXTDEBUG}
            list.concat(tai_comment.create(strpnew('fixref-pic ' + ref2string(ref))));
            {$ENDIF EXTDEBUG}
          end;

        if (ref.base = NR_NO) then
          begin
            ref.base := ref.index;
            ref.index := NR_NO;
          end;
        if (ref.base <> NR_NO) then
          begin
            if (ref.index <> NR_NO) and
               ((ref.offset <> 0) or assigned(ref.symbol)) then
              begin
                result := true;
                tmpreg := rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);
                list.concat(taicpu.op_reg_reg_reg(
                  A_ADD,tmpreg,ref.base,ref.index));
                ref.index := NR_NO;
                ref.base := tmpreg;
              end
          end;
        if (ref.index <> NR_NO) and
           (assigned(ref.symbol) or
            (ref.offset <> 0)) then
          internalerror(200208102);
        {$IFDEF EXTDEBUG}
        list.concat(tai_comment.create(strpnew('fixref1 ' + ref2string(ref))));
        {$ENDIF EXTDEBUG}
       end;


    procedure tcgppcgen.a_load_store(list:TAsmList;op: tasmop;reg:tregister;
       ref: treference);

      var
        tmpreg: tregister;
{$ifdef cpu64bitaddr}
        tmpreg2: tregister;
{$endif cpu64bitaddr}
        tmpref: treference;
        largeOffset: Boolean;

      begin
        tmpreg := NR_NO;
        largeOffset:= hasLargeOffset(ref);

        if target_info.system in ([system_powerpc_macos]+systems_aix) then
          begin

            if assigned(ref.symbol) and
               (ref.refaddr<>addr_pic_no_got) then
              begin {Load symbol's value}
                tmpreg := rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);

                reference_reset(tmpref,sizeof(pint),[]);
                tmpref.symbol := ref.symbol;
                tmpref.base := NR_RTOC;
                tmpref.refaddr := addr_pic_no_got;

                if macos_direct_globals then
                  list.concat(taicpu.op_reg_ref(A_LA,tmpreg,tmpref))
                else
{$ifdef cpu64bitaddr}
                  list.concat(taicpu.op_reg_ref(A_LD,tmpreg,tmpref));
{$else cpu64bitaddr}
                  list.concat(taicpu.op_reg_ref(A_LWZ,tmpreg,tmpref));
{$endif cpu64bitaddr}
              end;

            if largeOffset then
              begin {Add hi part of offset}
                reference_reset(tmpref,ref.alignment,[]);

{$ifdef cpu64bitaddr}
                if (ref.offset < low(longint)) or
                   (ref.offset > high(longint)) then
                  begin
                    { load upper 32 bits of the offset, adjusted for adding
                      the lower 32 bits later }
                    tmpreg2:=getintregister(list,OS_ADDR);
                    a_load_const_reg(list,OS_ADDR,(ref.offset and $ffffffff00000000) + ord(longint(ref.offset)<0),tmpreg2);
                    if tmpreg=NR_NO then
                      tmpreg:=tmpreg2
                    else
                      a_op_reg_reg(list,OP_ADD,OS_ADDR,tmpreg2,tmpreg);
                    ref.offset:=longint(ref.offset);
                  end;
{$endif cpu64bitaddr}
                {Compensate when lo part is negative}
                tmpref.offset := Smallint(ref.offset >> 16) + ord(Smallint(ref.offset) < 0);

                if (tmpreg <> NR_NO) then
                  list.concat(taicpu.op_reg_reg_const(A_ADDIS,tmpreg, tmpreg,tmpref.offset))
                else
                  begin
                    tmpreg := getintregister(list,OS_ADDR);
                    list.concat(taicpu.op_reg_const(A_LIS,tmpreg,tmpref.offset));
                  end;
              end;

            if (tmpreg <> NR_NO) then
              begin
                {Add content of base register}
                if ref.base <> NR_NO then
                  list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg,
                    ref.base,tmpreg));

                {Make ref ready to be used by op}
                ref.symbol:= nil;
                ref.base:= tmpreg;
                if largeOffset then
                  ref.offset := Smallint(ref.offset);

                list.concat(taicpu.op_reg_ref(op,reg,ref));
                //list.concat(tai_comment.create(strpnew('*** a_load_store indirect global')));
              end
            else
              list.concat(taicpu.op_reg_ref(op,reg,ref));
          end
        else {if target_info.system <> system_powerpc_macos}
          begin
            if assigned(ref.symbol) or
               largeOffset then
              begin
                // TODO: offsets > 32 bit
                tmpreg := rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);
                reference_reset(tmpref,ref.alignment,[]);
                tmpref.symbol := ref.symbol;
                tmpref.relsymbol := ref.relsymbol;
                tmpref.offset := ref.offset;
                tmpref.refaddr := addr_higha;
                if ref.base <> NR_NO then
                  list.concat(taicpu.op_reg_reg_ref(A_ADDIS,tmpreg,
                    ref.base,tmpref))
                else
                  list.concat(taicpu.op_reg_ref(A_LIS,tmpreg,tmpref));
                ref.base := tmpreg;
                ref.refaddr := addr_low;
                list.concat(taicpu.op_reg_ref(op,reg,ref));
              end
            else
              list.concat(taicpu.op_reg_ref(op,reg,ref));
          end;
      end;



    { TPPCAsmData }

    procedure TPPCAsmData.GetNextSmallTocEntry(out tocnr, entrynr: longint);
      begin
        if fcurrenttocentries>(high(word) div sizeof(pint)) then
          begin
            fcurrenttocentries:=0;
            inc(ftocsections);
          end;
        tocnr:=ftocsections;
        entrynr:=fcurrenttocentries;
        inc(fcurrenttocentries);
      end;

begin
  casmdata:=TPPCAsmData;
end.

