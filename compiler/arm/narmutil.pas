{
    Copyright (c) 2019 by Florian Klämpfl

    ARM version of some node tree helper routines

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
unit narmutil;

{$i fpcdefs.inc}

interface

  uses
    cclasses,ngenutil;

  type
    tarmnodeutils = class(tnodeutils)
      class procedure InsertObjectInfo; override;
      class procedure insert_init_final_table(entries: tfplist); override;
    end;


  implementation

    uses
      verbose,
      systems,
      globals,
      cpuinfo,cpubase,
      cgbase,cgutils,
      aasmbase,aasmdata,aasmtai,aasmcpu,
      symdef;

    const
      Tag_File = 1;
      Tag_Section = 2;
      Tag_Symbol = 3;
      Tag_CPU_raw_name = 4;
      Tag_CPU_name = 5;
      Tag_CPU_arch = 6;
      Tag_CPU_arch_profile = 7;
      Tag_ARM_ISA_use = 8;
      Tag_THUMB_ISA_use = 9;
      Tag_FP_Arch = 10;
      Tag_WMMX_arch = 11;
      Tag_Advanced_SIMD_arch = 12;
      Tag_PCS_config = 13;
      Tag_ABI_PCS_R9_use = 14;
      Tag_ABI_PCS_RW_data = 15;
      Tag_ABI_PCS_RO_data = 16;
      Tag_ABI_PCS_GOT_use = 17;
      Tag_ABI_PCS_wchar_t = 18;
      Tag_ABI_FP_rounding = 19;
      Tag_ABI_FP_denormal = 20;
      Tag_ABI_FP_exceptions = 21;
      Tag_ABI_FP_user_exceptions = 22;
      Tag_ABI_FP_number_model = 23;
      Tag_ABI_align_needed = 24;
      Tag_ABI_align8_preserved = 25;
      Tag_ABI_enum_size = 26;
      Tag_ABI_HardFP_use = 27;
      Tag_ABI_VFP_args = 28;
      Tag_ABI_WMMX_args = 29;
      Tag_ABI_optimization_goals = 30;
      Tag_ABI_FP_optimization_goals = 31;
      Tag_compatiblity = 32;
      Tag_CPU_unaligned_access = 34;
      Tag_FP_HP_extension = 36;
      Tag_ABI_FP_16bit_format = 38;
      Tag_MPextension_use = 42;
      Tag_DIV_use = 44;
      Tag_nodefaults = 64;
      Tag_also_compatible_with = 65;
      Tag_conformance = 67;
      Tag_T2EE_use = 66;
      Tag_Virtualization_use = 68;

    class procedure tarmnodeutils.InsertObjectInfo;
      begin
        inherited InsertObjectInfo;
        { write eabi attributes to object file? }
        if (target_info.system in [system_arm_linux]) and (target_info.abi in [abi_eabihf,abi_eabi]) then
          begin
            case current_settings.cputype of
              cpu_armv3:
                begin
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch,0));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch_profile,0));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_name,''));
                end;
              cpu_armv4:
                begin
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch,1));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch_profile,0));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_name,'4'));
                end;
              cpu_armv4t:
                begin
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch,2));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch_profile,0));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_name,'4T'));
                end;
              cpu_armv5t:
                begin
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch,3));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch_profile,0));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_name,'5T'));
                end;
              cpu_armv5te:
                begin
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch,4));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch_profile,0));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_name,'5TE'));
                end;
              cpu_armv5tej:
                begin
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch,5));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch_profile,0));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_name,'5TEJ'));
                end;
              cpu_armv6:
                begin
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch,6));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch_profile,0));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_name,'6'));
                end;
              cpu_armv6k:
                begin
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch,9));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch_profile,0));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_name,'6K'));
                end;
              cpu_armv6t2:
                begin
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch,8));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch_profile,0));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_name,'T2'));
                end;
              cpu_armv6z:
                begin
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch,7));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch_profile,0));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_name,'6Z'));
                end;
              cpu_armv6m:
                begin
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch,11));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch_profile,0));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_name,'6-M'));
                end;
              cpu_armv7:
                begin
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch,10));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch_profile,0));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_name,'7'));
                end;
              cpu_armv7a:
                begin
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch,10));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch_profile,$41));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_name,'7-A'));
                end;
              cpu_armv7r:
                begin
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch,10));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch_profile,$52));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_name,'7-R'));
                end;
              cpu_armv7m:
                begin
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch,10));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch_profile,$4D));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_name,'7-M'));
                end;
              cpu_armv7em:
                begin
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch,13));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_arch_profile,$4D));
                  current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_CPU_name,'7E-M'));
                end;
              else
                Internalerror(2019100602);
            end;
            case current_settings.fputype of
              fpu_none,
              fpu_soft,
              fpu_libgcc,
              fpu_fpa,
              fpu_fpa10,
              fpu_fpa11:
                current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_FP_Arch,0));
              fpu_vfpv2:
                current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_FP_Arch,2));
              fpu_vfpv3,
              fpu_neon_vfpv3:
                current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_FP_Arch,3));
              fpu_vfpv3_d16:
                current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_FP_Arch,4));
              fpu_fpv4_sp_d16,
              fpu_fpv4_s16:
                current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_FP_Arch,6));
              fpu_vfpv4,
              fpu_neon_vfpv4:
                current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_FP_Arch,5));
              { else not needed anymore PM 2020/04/13
                Internalerror(2019100603); }
            end;
            if FPUARM_HAS_FMA in fpu_capabilities[current_settings.fputype] then
              current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_Advanced_SIMD_arch,2))
            else if FPUARM_HAS_NEON in fpu_capabilities[current_settings.fputype] then
              current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_Advanced_SIMD_arch,1))
            else
              current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_Advanced_SIMD_arch,0));
            current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_ARM_ISA_use,1));
            if CPUARM_HAS_THUMB2 in cpu_capabilities[current_settings.cputype] then
              current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_THUMB_ISA_use,2))
            else
              current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_THUMB_ISA_use,1));
            current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_ABI_VFP_args,1));
            current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_ABI_FP_denormal,1));
            current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_ABI_FP_exceptions,1));
            current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_ABI_FP_number_model,3));
            current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_ABI_align_needed,0));
            current_asmdata.asmlists[al_start].Concat(tai_eabi_attribute.create(Tag_ABI_align8_preserved,1));
            { gcc typically writes more like enum size, wchar size, optimization goal, however, this
              is normally not module global in FPC }
          end;
      end;

    class procedure tarmnodeutils.insert_init_final_table(entries:tfplist);

      procedure genentry(list : TAsmList);
        var
          ref: treference;
        begin
          if GenerateThumbCode then
            list.concat(taicpu.op_regset(A_PUSH,R_INTREGISTER,R_SUBWHOLE,[RS_R14]))
          else
            begin
              reference_reset(ref,4,[]);
              ref.index:=NR_STACK_POINTER_REG;
              ref.addressmode:=AM_PREINDEXED;
              list.concat(setoppostfix(taicpu.op_ref_regset(A_STM,ref,R_INTREGISTER,R_SUBWHOLE,[RS_R14]),PF_FD));
            end;
        end;

      procedure genexit(list : TAsmList);
        var
          ref: treference;
        begin
          if GenerateThumbCode then
            list.concat(taicpu.op_regset(A_POP,R_INTREGISTER,R_SUBWHOLE,[RS_R15]))
          else
            begin
              reference_reset(ref,4,[]);
              ref.index:=NR_STACK_POINTER_REG;
              ref.addressmode:=AM_PREINDEXED;
              list.concat(setoppostfix(taicpu.op_ref_regset(A_LDM,ref,R_INTREGISTER,R_SUBWHOLE,[RS_R15]),PF_FD));
            end;
        end;

      var
        initList, finalList, header: TAsmList;
        entry : pinitfinalentry;
        i : longint;
      begin
        if not(tf_init_final_units_by_calls in target_info.flags) then
          begin
            inherited insert_init_final_table(entries);
            exit;
          end;
        initList:=TAsmList.create;
        finalList:=TAsmList.create;

        genentry(finalList);
        genentry(initList);

        for i:=0 to entries.count-1 do
          begin
            entry:=pinitfinalentry(entries[i]);
            if entry^.finifunc<>'' then
              finalList.Concat(taicpu.op_sym(A_BL,current_asmdata.RefAsmSymbol(entry^.finifunc,AT_FUNCTION)));
            if entry^.initfunc<>'' then
              initList.Concat(taicpu.op_sym(A_BL,current_asmdata.RefAsmSymbol(entry^.initfunc,AT_FUNCTION)));
          end;

        genexit(finalList);
        genexit(initList);

        header:=TAsmList.create;
        new_section(header, sec_code, 'FPC_INIT_FUNC_TABLE', 1);
        header.concat(tai_symbol.Createname_global('FPC_INIT_FUNC_TABLE',AT_FUNCTION,0,voidcodepointertype));

        initList.insertList(header);
        header.free;

        current_asmdata.AsmLists[al_procedures].concatList(initList);

        header:=TAsmList.create;
        new_section(header, sec_code, 'FPC_FINALIZE_FUNC_TABLE', 1);
        header.concat(tai_symbol.Createname_global('FPC_FINALIZE_FUNC_TABLE',AT_FUNCTION,0,voidcodepointertype));

        finalList.insertList(header);
        header.free;

        current_asmdata.AsmLists[al_procedures].concatList(finalList);

        initList.Free;
        finalList.Free;

        inherited insert_init_final_table(entries);
      end;

  begin
    cnodeutils:=tarmnodeutils;
  end.

