{
    Copyright (c) 2002 by Florian Klaempfl

    Implements the M68K specific part of call nodes

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
unit n68kcal;

{$i fpcdefs.inc}

interface

    uses
      symdef,node,ncal,ncgcal;

    type
       tm68kcallnode = class(tcgcallnode)
        protected
         procedure gen_syscall_para(para: tcallparanode); override;
        public
         procedure do_syscall;override;
         procedure pop_parasize(pop_size: longint);override;
       end;


implementation

    uses
      globtype,systems,
      cutils,verbose,globals,
      symconst,symbase,symsym,symcpu,symtable,defutil,paramgr,parabase,
      cgbase,pass_2,
      cpuinfo,cpubase,aasmbase,aasmtai,aasmdata,aasmcpu,
      nmem,nld,ncnv,
      ncgutil,cgutils,cgobj,tgobj,regvars,rgobj,rgcpu,
      cg64f32,cgcpu,cpupi,procinfo;


    procedure tm68kcallnode.pop_parasize(pop_size: longint);
      begin
        if pop_size<>0 then
          current_asmdata.CurrAsmList.concat(taicpu.op_const_reg(A_ADD,S_L,pop_size,NR_SP));
      end;


    procedure tm68kcallnode.gen_syscall_para(para: tcallparanode);
      begin
        { lib parameter has no special type but proccalloptions must be a syscall }
        para.left:=cloadnode.create(tcpuprocdef(procdefinition).libsym,tcpuprocdef(procdefinition).libsym.owner);
      end;


    procedure tm68kcallnode.do_syscall;
      var
        tmpref: treference;
	tmpref2: treference;
      begin
        case target_info.system of
          system_m68k_amiga:
            begin
              if po_syscall_legacy in tprocdef(procdefinition).procoptions then
                begin
<<<<<<< HEAD
<<<<<<< HEAD
                  { according to Amiga Developer CD 2.1, system functions destroy the
                    scratch regs D0-D1 and A0-A1, but preserve all other regs. A6 is
                    not used as FP on Amiga any more (we use A5), so we don't need to
                    save it. (KB)
                    http://amigadev.elowar.com/read/ADCD_2.1/Libraries_Manual_guide/node0290.html }
                  reference_reset_base(tmpref,NR_A6,-tprocdef(procdefinition).extnumber,4);
=======
=======
>>>>>>> origin/fixes_2_2
		  { save base pointer on syscalls }
		  { FIXME: probably this will need to be extended to save all regs (KB) }
                  reference_reset_base(tmpref2, NR_STACK_POINTER_REG, 0);
                  tmpref2.direction := dir_dec;
		  current_asmdata.CurrAsmList.concat(taicpu.op_reg_ref(A_MOVE,S_L,NR_FRAME_POINTER_REG,tmpref2));
		  
		  { the actuall call }
                  reference_reset_base(tmpref,NR_A6,-tprocdef(procdefinition).extnumber);
>>>>>>> graemeg/fixes_2_2
                  current_asmdata.CurrAsmList.concat(taicpu.op_ref(A_JSR,S_NO,tmpref));
		  
		  { restore frame pointer }
                  reference_reset_base(tmpref2, NR_STACK_POINTER_REG, 0);
                  tmpref2.direction := dir_inc;
		  current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(A_MOVE,S_L,tmpref2,NR_FRAME_POINTER_REG));
                end
              else
                internalerror(2005010403);
            end;
          else
            internalerror(2004042901);
        end;
      end;


begin
   ccallnode:=tm68kcallnode;
end.
