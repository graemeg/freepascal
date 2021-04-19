{
    Copyright (c) 1998-2010 by Florian Klaempfl and Jonas Maebe
    Member of the Free Pascal development team

    This unit contains routines to create a pass-through high-level code
    generator. This is used by most regular code generators.

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

unit hlcgcpu;

{$i fpcdefs.inc}

interface

  uses
    symtype,
    aasmdata,
    symdef,
    cgbase,cgutils,
    hlcgobj, hlcg2ll;

  type
    thlcgaarch64 = class(thlcg2ll)
      procedure a_load_subsetreg_reg(list: TAsmList; subsetsize, tosize: tdef; const sreg: tsubsetregister; destreg: tregister); override;
      procedure a_load_subsetreg_subsetreg(list: TAsmlist; fromsubsetsize, tosubsetsize: tdef; const fromsreg, tosreg: tsubsetregister); override;

      procedure g_intf_wrapper(list: TAsmList; procdef: tprocdef; const labelname: string; ioffset: longint);override;
     protected
      procedure a_load_regconst_subsetreg_intern(list: TAsmList; fromsize, subsetsize: tdef; fromreg: tregister; const sreg: tsubsetregister; slopt: tsubsetloadopt); override;
    end;

implementation

  uses
    verbose,globtype,fmodule,
    aasmbase,aasmtai,
    symconst,symsym,defutil,
    cpubase,aasmcpu,parabase,
    cgobj,cgcpu;

  procedure thlcgaarch64.a_load_subsetreg_reg(list: TAsmList; subsetsize, tosize: tdef; const sreg: tsubsetregister; destreg: tregister);
    var
      op: tasmop;
      tocgsize: tcgsize;
      tmpdestreg: tregister;
    begin
      tocgsize:=def_cgsize(tosize);
      if (sreg.startbit<>0) or
         not((sreg.subsetregsize in [OS_32,OS_S32]) and
             (sreg.bitlen=32)) or
         not((sreg.subsetregsize in [OS_64,OS_S64]) and
             (sreg.bitlen=64)) then
        begin
          if is_signed(subsetsize) then
            op:=A_SBFX
          else
            op:=A_UBFX;
          { source and destination register of SBFX/UBFX have to be the same size }
          if (sreg.subsetregsize in [OS_64,OS_S64]) and
             not(tocgsize in [OS_64,OS_S64]) then
            tmpdestreg:=cg.getintregister(list,OS_64)
          else if not(sreg.subsetregsize in [OS_64,OS_S64]) and
             (tocgsize in [OS_64,OS_S64]) then
            tmpdestreg:=cg.getintregister(list,OS_32)
          else
            tmpdestreg:=destreg;
          list.concat(taicpu.op_reg_reg_const_const(op,tmpdestreg,sreg.subsetreg,sreg.startbit,sreg.bitlen));
          { need to sign extend further or truncate? }
          if (sreg.subsetregsize=OS_S64) and
             not(tocgsize in [OS_64,OS_S64]) then
            cg.a_load_reg_reg(list,OS_S64,tocgsize,tmpdestreg,destreg)
          else if is_signed(subsetsize) and
             (tocgsize in [OS_8,OS_16]) then
            cg.a_load_reg_reg(list,OS_32,tocgsize,tmpdestreg,destreg)
          else if tmpdestreg<>destreg then
            cg.a_load_reg_reg(list,def_cgsize(subsetsize),tocgsize,tmpdestreg,destreg)
        end
      else
        cg.a_load_reg_reg(list,def_cgsize(subsetsize),tocgsize,sreg.subsetreg,destreg);
    end;


  procedure makeregssamesize(list: tasmlist; fromsize, tosize: tcgsize; orgfromreg, orgtoreg: tregister; out newfromreg, newtoreg: tregister);
    begin
      if (fromsize in [OS_S64,OS_64])<>
         (tosize in [OS_S64,OS_64]) then
        begin
          newfromreg:=cg.makeregsize(list,orgfromreg,OS_64);
          newtoreg:=cg.makeregsize(list,orgtoreg,OS_64);
        end
      else
        begin
          newfromreg:=orgfromreg;
          newtoreg:=orgtoreg;
        end;
    end;


  procedure thlcgaarch64.a_load_subsetreg_subsetreg(list: TAsmlist; fromsubsetsize, tosubsetsize: tdef; const fromsreg, tosreg: tsubsetregister);
    var
      fromreg, toreg: tregister;

    begin
      { BFM can only insert a bitfield that starts at position 0 in the source
        source or destination register }
      if (tosreg.startbit=0) and
         (fromsreg.bitlen>=tosreg.bitlen) then
        begin
          makeregssamesize(list,fromsreg.subsetregsize,tosreg.subsetregsize,fromsreg.subsetreg,tosreg.subsetreg,fromreg,toreg);
          list.concat(taicpu.op_reg_reg_const_const(A_BFXIL,toreg,fromreg,fromsreg.startbit,tosreg.bitlen))
        end
      else if (fromsreg.startbit=0) and
         (fromsreg.bitlen>=tosreg.bitlen) then
        begin
          makeregssamesize(list,fromsreg.subsetregsize,tosreg.subsetregsize,fromsreg.subsetreg,tosreg.subsetreg,fromreg,toreg);
          list.concat(taicpu.op_reg_reg_const_const(A_BFI,toreg,fromreg,tosreg.startbit,tosreg.bitlen))
        end
      else
        inherited;
    end;


  procedure thlcgaarch64.g_intf_wrapper(list: TAsmList; procdef: tprocdef; const labelname: string; ioffset: longint);
    var
      make_global: boolean;
      href: treference;
      hsym: tsym;
      paraloc: pcgparalocation;
      op: tasmop;
    begin
      if not(procdef.proctypeoption in [potype_function,potype_procedure]) then
        Internalerror(200006137);
      if not assigned(procdef.struct) or
         (procdef.procoptions*[po_classmethod, po_staticmethod,
           po_methodpointer, po_interrupt, po_iocheck]<>[]) then
        Internalerror(200006138);
      if procdef.owner.symtabletype<>ObjectSymtable then
        Internalerror(200109191);

      make_global:=false;
      if (not current_module.is_unit) or create_smartlink_library or
         (procdef.owner.defowner.owner.symtabletype=globalsymtable) then
        make_global:=true;

      if make_global then
        list.concat(Tai_symbol.Createname_global(labelname,AT_FUNCTION,0,procdef))
      else
        list.concat(Tai_symbol.Createname_hidden(labelname,AT_FUNCTION,0,procdef));

      { set param1 interface to self  }
      procdef.init_paraloc_info(callerside);
      hsym:=tsym(procdef.parast.Find('self'));
      if not(assigned(hsym) and
        (hsym.typ=paravarsym)) then
        internalerror(2010103101);
      paraloc:=tparavarsym(hsym).paraloc[callerside].location;
      if assigned(paraloc^.next) then
        InternalError(2013020101);

      case paraloc^.loc of
        LOC_REGISTER:
          tcgaarch64(cg).handle_reg_imm12_reg(list,A_SUB,paraloc^.size,paraloc^.register,ioffset,paraloc^.register,NR_IP0,false,true);
        else
          internalerror(2010103102);
      end;

      if (po_virtualmethod in procdef.procoptions) and
          not is_objectpascal_helper(procdef.struct) then
        begin
          if (procdef.extnumber=$ffff) then
            Internalerror(2000061302);
          { mov  0(%rdi),%rax ; load vmt}
          reference_reset_base(href,voidpointertype,paraloc^.register,0,ctempposinvalid,sizeof(pint),[]);
          getcpuregister(list,NR_IP0);
          a_load_ref_reg(list,voidpointertype,voidpointertype,href,NR_IP0);
          { jmp *vmtoffs(%eax) ; method offs }
          reference_reset_base(href,voidpointertype,NR_IP0,tobjectdef(procdef.struct).vmtmethodoffset(procdef.extnumber),ctempposinvalid,sizeof(pint),[]);
          op:=A_LDR;
          tcgaarch64(cg).make_simple_ref(list,op,OS_ADDR,PF_None,href,NR_IP0);
          list.concat(taicpu.op_reg_ref(op,NR_IP0,href));
          ungetcpuregister(list,NR_IP0);
          list.concat(taicpu.op_reg(A_BR,NR_IP0));
        end
      else
        cg.a_jmp_name(list,procdef.mangledname);
      list.concat(Tai_symbol_end.Createname(labelname));
    end;


  procedure thlcgaarch64.a_load_regconst_subsetreg_intern(list: TAsmList; fromsize, subsetsize: tdef; fromreg: tregister; const sreg: tsubsetregister; slopt: tsubsetloadopt);
    var
      toreg: tregister;
    begin
      if slopt in [SL_SETZERO,SL_SETMAX] then
        inherited
      else if not(sreg.bitlen in [32,64]) or
              (sreg.startbit<>0) or
              (getsubreg(fromreg)<getsubreg(sreg.subsetreg)) then
        begin
          makeregssamesize(list,def_cgsize(fromsize),sreg.subsetregsize,fromreg,sreg.subsetreg,fromreg,toreg);
          list.concat(taicpu.op_reg_reg_const_const(A_BFI,toreg,fromreg,sreg.startbit,sreg.bitlen))
        end
      else
        a_load_reg_reg(list,fromsize,subsetsize,fromreg,sreg.subsetreg);
    end;


  procedure create_hlcodegen_cpu;
    begin
      hlcg:=thlcgaarch64.create;
      create_codegen;
    end;


begin
  chlcgobj:=thlcgaarch64;
  create_hlcodegen:=@create_hlcodegen_cpu;
end.
