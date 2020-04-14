{
    Copyright (c) 2002 by Florian Klaempfl

    Implements the ARM specific part of call nodes

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
unit narmcal;

{$i fpcdefs.inc}

interface

    uses
      symdef,ncal,ncgcal;

    type
       tarmcallnode = class(tcgcallnode)
         procedure gen_syscall_para(para: tcallparanode); override;
         procedure set_result_location(realresdef: tstoreddef);override;
       public
         procedure do_syscall;override;
       end;

implementation

  uses
    verbose,globtype,globals,aasmdata,aasmtai,
    symconst,symtype,symbase,symsym,symcpu,parabase,paramgr,
    cgbase,cgobj,cgutils,cpuinfo,cpubase,cutils,
    ncgutil,tgobj,nld,
    systems;

  procedure tarmcallnode.gen_syscall_para(para: tcallparanode);
    begin
      { lib parameter has no special type but proccalloptions must be a syscall }
      para.left:=cloadnode.create(tcpuprocdef(procdefinition).libsym,tcpuprocdef(procdefinition).libsym.owner);
    end;

  procedure tarmcallnode.do_syscall;
    var
      tmpref: treference;
    begin
      case target_info.system of
        system_arm_aros:
            begin
              if (po_syscall_baselast in tprocdef(procdefinition).procoptions) then
                begin
                  current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('AROS SysCall')));

                  cg.getcpuregister(current_asmdata.CurrAsmList,NR_R12);
                  get_syscall_call_ref(tmpref,NR_R12);

                  cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,tmpref,NR_R12);
                  cg.a_call_reg(current_asmdata.CurrAsmList,NR_R12);
                  cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_R12);
                  exit;
                end;
              internalerror(2016110601);
            end;
        else
          internalerror(2016110602);
      end;
    end;

  procedure tarmcallnode.set_result_location(realresdef: tstoreddef);
    begin
      if (realresdef.typ=floatdef) and 
         (target_info.abi<>abi_eabihf) and
         (procdefinition.proccalloption<>pocall_hardfloat) and
         ((cs_fp_emulation in current_settings.moduleswitches) or
          (FPUARM_HAS_VFP_EXTENSION in fpu_capabilities[current_settings.fputype])) then
        begin
          { keep the fpu values in integer registers for now, the code
            generator will move them to memory or an mmregister when necessary
            (avoids double moves in case a function result is assigned to
             another function result, or passed as a parameter) }
          case retloc.size of
            OS_32,
            OS_F32:
              location_allocate_register(current_asmdata.CurrAsmList,location,s32inttype,false);
            OS_64,
            OS_F64:
              location_allocate_register(current_asmdata.CurrAsmList,location,s64inttype,false);
            else
              internalerror(2010053008);
          end
        end
      else if (resultdef.typ=floatdef) and
         (location.loc=LOC_REGISTER) and
         (current_settings.fputype in [fpu_fpa,fpu_fpa10,fpu_fpa11]) then
        begin
          location_reset_ref(location,LOC_REFERENCE,location.size,resultdef.alignment,[]);
          tg.gethltemp(current_asmdata.CurrAsmList,resultdef,resultdef.size,tt_normal,location.reference);
        end
      else
        inherited;
    end;




begin
   ccallnode:=tarmcallnode;
end.
