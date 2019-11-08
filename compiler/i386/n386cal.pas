{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate i386 assembler for in call nodes

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
unit n386cal;

{$i fpcdefs.inc}

interface

{ $define AnsiStrRef}

    uses
      nx86cal,ncal;

    type
       ti386callnode = class(tx86callnode)
       protected
          procedure gen_syscall_para(para: tcallparanode); override;
          procedure pop_parasize(pop_size:longint);override;
          procedure extra_interrupt_code;override;
       public
         procedure do_syscall;override;
       end;


implementation

    uses
      globtype,systems,
      cutils,verbose,globals,
      cgbase,cgutils,
      cpubase,paramgr,
      aasmtai,aasmdata,aasmcpu,
      nbas,nmem,nld,ncnv,
      parabase,
      symdef,symsym,symcpu,symconst,
      cga,cgobj,cgx86,
      cpuinfo;


{*****************************************************************************
                             TI386CALLNODE
*****************************************************************************}


    procedure ti386callnode.do_syscall;
      var
        tmpref: treference;
      begin
        case target_info.system of
          system_i386_aros:
            begin
              if ([po_syscall_baselast, po_syscall_basereg] * tprocdef(procdefinition).procoptions) <> [] then
                begin
                  current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('AROS SysCall')));

                  cg.getcpuregister(current_asmdata.CurrAsmList,NR_EAX);
                  get_syscall_call_ref(tmpref,NR_EAX);

                  current_asmdata.CurrAsmList.concat(taicpu.op_ref(A_CALL,S_NO,tmpref));
                  cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_EAX);
                  exit;
                end;
              internalerror(2016090104);
            end;
          else
            internalerror(2014081801);
        end;
      end;

    procedure ti386callnode.gen_syscall_para(para: tcallparanode);
      begin
        { lib parameter has no special type but proccalloptions must be a syscall }
        para.left:=cloadnode.create(tcpuprocdef(procdefinition).libsym,tcpuprocdef(procdefinition).libsym.owner);
      end;

    procedure ti386callnode.extra_interrupt_code;
      begin
        if not(target_info.system in [system_i386_darwin,system_i386_iphonesim,system_i386_android]) then
          begin
            emit_none(A_PUSHF,S_L);
            emit_reg(A_PUSH,S_L,NR_CS);
          end;
      end;


    procedure ti386callnode.pop_parasize(pop_size:longint);
      var
        hreg : tregister;
        href : treference;
      begin
        if paramanager.use_fixed_stack then
          begin
            { very weird: in this case the callee does a "ret $4" and the }
            { caller immediately a "subl $4,%esp". Possibly this is for   }
            { use_fixed_stack code to be able to transparently call       }
            { old-style code (JM)                                         }
            dec(pop_size,pushedparasize);
            if pop_size<0 then
              begin
                reference_reset_base(href,NR_STACK_POINTER_REG,pop_size,ctempposinvalid,0,[]);
                { it is better to use lea here }
                current_asmdata.CurrAsmList.concat(Taicpu.op_ref_reg(A_LEA,TCGSize2OpSize[OS_ADDR],href,NR_ESP));
              end;
            exit;
          end;

        { on win32, the caller is responsible for removing the funcret     }
        { pointer from the stack, unlike on Linux. Don't know about        }
        { elsewhere (except Darwin, handled above), but since the default  }
        { was "callee removes funcret pointer from stack" until now, we'll }
        { keep that default for everyone else (ncgcal decreases popsize by }
        { sizeof(aint) in case of ret_in_param())                          }
        { This is only correct if the hidden funcret parameter
          is not passed as a register.
          As it is inserted in parast after all other hidden parameters,
          it is always the first parameter (apart from hidden parentfp,
          but this one is never put into a register (vs_nonregable set)
          so funcret is always in EAX for register calling }
        if ((target_info.system = system_i386_win32) and
            not (target_info.abi=abi_old_win32_gnu)) and
            paramanager.ret_in_param(procdefinition.returndef,procdefinition) and
            not ((procdefinition.proccalloption=pocall_register) or
                 ((procdefinition.proccalloption=pocall_internproc) and
                  (pocall_default=pocall_register))) then
          inc(pop_size,sizeof(aint));

        { better than an add on all processors }
        if pop_size=4 then
          begin
            hreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
            current_asmdata.CurrAsmList.concat(taicpu.op_reg(A_POP,S_L,hreg));
          end
        { the pentium has two pipes and pop reg is pairable }
        { but the registers must be different!        }
        else
          if (pop_size=8) and
             not(cs_opt_size in current_settings.optimizerswitches) and
             (current_settings.optimizecputype=cpu_Pentium) then
            begin
               hreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
               current_asmdata.CurrAsmList.concat(taicpu.op_reg(A_POP,S_L,hreg));
               hreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
               current_asmdata.CurrAsmList.concat(taicpu.op_reg(A_POP,S_L,hreg));
            end
        else
          if pop_size<>0 then
            current_asmdata.CurrAsmList.concat(taicpu.op_const_reg(A_ADD,S_L,pop_size,NR_ESP));
      end;


begin
   ccallnode:=ti386callnode;
end.
