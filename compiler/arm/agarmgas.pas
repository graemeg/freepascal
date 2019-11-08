{
    Copyright (c) 2003 by Florian Klaempfl

    This unit implements an asm for the ARM

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
{ This unit implements the GNU Assembler writer for the ARM
}

unit agarmgas;

{$i fpcdefs.inc}

  interface

    uses
       globtype,systems,
       aasmtai,
       assemble,aggas,
       cpubase,cpuinfo;

    type
      TARMGNUAssembler=class(TGNUassembler)
        constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
        function MakeCmdLine: TCmdStr; override;
        procedure WriteExtraHeader; override;
      end;

      TArmInstrWriter=class(TCPUInstrWriter)
        unified_syntax: boolean;

        procedure WriteInstruction(hp : tai);override;
      end;

      TArmAppleGNUAssembler=class(TAppleGNUassembler)
        constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
        function MakeCmdLine: TCmdStr; override;
        procedure WriteExtraHeader; override;
      end;


    const
      gas_shiftmode2str : array[tshiftmode] of string[3] = (
        '','lsl','lsr','asr','ror','rrx');

    const 
      cputype_to_gas_march : array[tcputype] of string = (
        '', // cpu_none
        'armv3',
        'armv4',
        'armv4t',
        'armv5',
        'armv5t',
        'armv5te',
        'armv5tej',
        'armv6',
        'armv6k',
        'armv6t2',
        'armv6z',
        'armv6-m',
        'armv7',
        'armv7-a',
        'armv7-r',
        'armv7-m',
        'armv7e-m');

  implementation

    uses
       cutils,globals,verbose,
       aasmcpu,
       itcpugas,
       cgbase,cgutils;

{****************************************************************************}
{                         GNU Arm Assembler writer                           }
{****************************************************************************}

    constructor TArmGNUAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter := TArmInstrWriter.create(self);
{$ifndef llvm}
        if GenerateThumb2Code then
{$endif}
          TArmInstrWriter(InstrWriter).unified_syntax:=true;
      end;


    function TArmGNUAssembler.MakeCmdLine: TCmdStr;
      begin
        result:=inherited MakeCmdLine;
        case current_settings.fputype of
          fpu_soft:
            result:='-mfpu=softvfp '+result;
          fpu_vfpv2:
            result:='-mfpu=vfpv2 '+result;
          fpu_vfpv3:
            result:='-mfpu=vfpv3 '+result;
          fpu_neon_vfpv3:
            result:='-mfpu=neon-vfpv3 '+result;
          fpu_vfpv3_d16:
            result:='-mfpu=vfpv3-d16 '+result;
          fpu_fpv4_s16:
            result:='-mfpu=fpv4-sp-d16 '+result;
          fpu_vfpv4:
            result:='-mfpu=vfpv4 '+result;
          fpu_neon_vfpv4:
            result:='-mfpu=neon-vfpv4 '+result;
          else
            ;
        end;

        if GenerateThumb2Code then
          result:='-march='+cputype_to_gas_march[current_settings.cputype]+' -mthumb -mthumb-interwork '+result
        else if GenerateThumbCode then
          result:='-march='+cputype_to_gas_march[current_settings.cputype]+' -mthumb -mthumb-interwork '+result
        else
          result:='-march='+cputype_to_gas_march[current_settings.cputype]+' '+result;

        if target_info.abi = abi_eabihf then
          { options based on what gcc uses on debian armhf }
          result:='-mfloat-abi=hard -meabi=5 '+result
        else if (target_info.abi = abi_eabi) and not(current_settings.fputype = fpu_soft) then
          result:='-mfloat-abi=softfp -meabi=5 '+result
        else if (target_info.abi = abi_eabi) and (current_settings.fputype = fpu_soft) then
          result:='-mfloat-abi=soft -meabi=5 '+result;
      end;

    procedure TArmGNUAssembler.WriteExtraHeader;
      begin
        inherited WriteExtraHeader;
        if TArmInstrWriter(InstrWriter).unified_syntax then
          writer.AsmWriteLn(#9'.syntax unified');
      end;

{****************************************************************************}
{                      GNU/Apple ARM Assembler writer                        }
{****************************************************************************}

    constructor TArmAppleGNUAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter := TArmInstrWriter.create(self);
        TArmInstrWriter(InstrWriter).unified_syntax:=true;
      end;


    function TArmAppleGNUAssembler.MakeCmdLine: TCmdStr;
      begin
        result:=inherited MakeCmdLine;
	if (asminfo^.id = as_clang) then
          begin
            if fputypestrllvm[current_settings.fputype] <> '' then
              result:='-m'+fputypestrllvm[current_settings.fputype]+' '+result;
            { Apple arm always uses softfp floating point ABI }
            result:='-mfloat-abi=softfp '+result;
          end;
      end;

    procedure TArmAppleGNUAssembler.WriteExtraHeader;
      begin
        inherited WriteExtraHeader;
        if TArmInstrWriter(InstrWriter).unified_syntax then
          writer.AsmWriteLn(#9'.syntax unified');
      end;


{****************************************************************************}
{                  Helper routines for Instruction Writer                    }
{****************************************************************************}

    function getreferencestring(var ref : treference) : string;
      var
        s : string;
      begin
         with ref do
          begin
{$ifdef extdebug}
            // if base=NR_NO then
            //   internalerror(200308292);

            // if ((index<>NR_NO) or (shiftmode<>SM_None)) and ((offset<>0) or (symbol<>nil)) then
            //   internalerror(200308293);
{$endif extdebug}

            if assigned(symbol) then
              begin
                if (base<>NR_NO) and not(is_pc(base)) then
                  internalerror(200309011);
                s:=symbol.name;
                if offset<>0 then
                  s:=s+tostr_with_plus(offset);
                if refaddr=addr_pic then
                  s:=s+'(PLT)'
                else if refaddr=addr_tlscall then
                  s:=s+'(tlscall)';
              end
            else
              begin
                s:='['+gas_regname(base);
                if addressmode=AM_POSTINDEXED then
                  s:=s+']';
                if index<>NR_NO then
                  begin
                     if signindex<0 then
                       s:=s+', -'
                     else
                       s:=s+', ';

                     s:=s+gas_regname(index);

                     {RRX always rotates by 1 bit and does not take an imm}
                     if shiftmode = SM_RRX then
                       s:=s+', rrx'
                     else if shiftmode <> SM_None then
                       s:=s+', '+gas_shiftmode2str[shiftmode]+' #'+tostr(shiftimm);
                     if offset<>0 then
                       Internalerror(2019012601);
                  end
                else if offset<>0 then
                  s:=s+', #'+tostr(offset);

                case addressmode of
                  AM_OFFSET:
                    s:=s+']';
                  AM_PREINDEXED:
                    s:=s+']!';
                  else
                    ;
                end;
              end;

          end;
        getreferencestring:=s;
      end;

    function getopstr(const o:toper) : string;
      var
        hs : string;
        first : boolean;
        r, rs : tsuperregister;
      begin
        case o.typ of
          top_reg:
            getopstr:=gas_regname(o.reg);
          top_shifterop:
            begin
              {RRX is special, it only rotates by 1 and does not take any shiftervalue}
              if o.shifterop^.shiftmode=SM_RRX then
                getopstr:='rrx'
              else if (o.shifterop^.rs<>NR_NO) and (o.shifterop^.shiftimm=0) then
                getopstr:=gas_shiftmode2str[o.shifterop^.shiftmode]+' '+gas_regname(o.shifterop^.rs)
              else if (o.shifterop^.rs=NR_NO) then
                getopstr:=gas_shiftmode2str[o.shifterop^.shiftmode]+' #'+tostr(o.shifterop^.shiftimm)
              else internalerror(200308282);
            end;
          top_const:
            getopstr:='#'+tostr(longint(o.val));
          top_regset:
            begin
              getopstr:='{';
              first:=true;
              if R_SUBFS=o.subreg then
                begin
                  for r:=0 to 31 do // S0 to S31
                    if r in o.regset^ then
                      begin
                        if not(first) then
                          getopstr:=getopstr+',';
                        if odd(r) then
                          rs:=(r shr 1)+RS_S1
                        else
                          rs:=(r shr 1)+RS_S0;
                        getopstr:=getopstr+gas_regname(newreg(o.regtyp,rs,o.subreg));
                        first:=false;
                      end;
                end
              else if R_SUBFD=o.subreg then
                begin
                  for r:=0 to 31 do
                    if r in o.regset^ then
                      begin
                        if not(first) then
                          getopstr:=getopstr+',';
                        rs:=r+RS_D0;
                        getopstr:=getopstr+gas_regname(newreg(o.regtyp,rs,o.subreg));
                        first:=false;
                      end;
                end
              else
                begin
                  for r:=RS_R0 to RS_R15 do
                    if r in o.regset^ then
                      begin
                        if not(first) then
                          getopstr:=getopstr+',';
                        getopstr:=getopstr+gas_regname(newreg(o.regtyp,r,o.subreg));
                        first:=false;
                      end;
                end;
              getopstr:=getopstr+'}';
              if o.usermode then
                getopstr:=getopstr+'^';
            end;
          top_conditioncode:
            getopstr:=cond2str[o.cc];
          top_modeflags:
            begin
              getopstr:='';
              if mfA in o.modeflags then getopstr:=getopstr+'a';
              if mfI in o.modeflags then getopstr:=getopstr+'i';
              if mfF in o.modeflags then getopstr:=getopstr+'f';
            end;
          top_ref:
            if o.ref^.refaddr=addr_full then
              begin
                hs:=o.ref^.symbol.name;
                if o.ref^.offset>0 then
                 hs:=hs+'+'+tostr(o.ref^.offset)
                else
                 if o.ref^.offset<0 then
                  hs:=hs+tostr(o.ref^.offset);
                getopstr:=hs;
              end
            else
              getopstr:=getreferencestring(o.ref^);
          top_specialreg:
            begin
              getopstr:=gas_regname(o.specialreg);
              if o.specialflags<>[] then
                begin
                  getopstr:=getopstr+'_';
                  if srC in o.specialflags then getopstr:=getopstr+'c';
                  if srX in o.specialflags then getopstr:=getopstr+'x';
                  if srF in o.specialflags then getopstr:=getopstr+'f';
                  if srS in o.specialflags then getopstr:=getopstr+'s';
                end;
            end;
          top_realconst:
            begin
              str(o.val_real,Result);
              Result:='#'+Result;
            end
          else
            internalerror(2002070604);
        end;
      end;


    Procedure TArmInstrWriter.WriteInstruction(hp : tai);
    var op: TAsmOp;
        postfix,s: string;
        i: byte;
        sep: string[3];
    begin
      op:=taicpu(hp).opcode;
      postfix:='';
      if GenerateThumb2Code then
        begin
          if taicpu(hp).wideformat then
            postfix:='.w';
        end;
      if unified_syntax then
        begin
          if taicpu(hp).ops = 0 then
            s:=#9+gas_op2str[op]+cond2str[taicpu(hp).condition]+oppostfix2str[taicpu(hp).oppostfix]
          else if taicpu(hp).oppostfix in [PF_8..PF_U32F64] then
            s:=#9+gas_op2str[op]+cond2str[taicpu(hp).condition]+oppostfix2str[taicpu(hp).oppostfix]
          else
            s:=#9+gas_op2str[op]+oppostfix2str[taicpu(hp).oppostfix]+cond2str[taicpu(hp).condition]+postfix; // Conditional infixes are deprecated in unified syntax
        end
      else
        s:=#9+gas_op2str[op]+cond2str[taicpu(hp).condition]+oppostfix2str[taicpu(hp).oppostfix];
      if taicpu(hp).ops<>0 then
        begin
          sep:=#9;
          for i:=0 to taicpu(hp).ops-1 do
            begin
               // debug code
               // writeln(s);
               // writeln(taicpu(hp).fileinfo.line);

               { LDM and STM use references as first operand but they are written like a register }
               if (i=0) and (op in [A_LDM,A_STM,A_FSTM,A_FLDM,A_VSTM,A_VLDM,A_SRS,A_RFE]) then
                 begin
                   case taicpu(hp).oper[0]^.typ of
                     top_ref:
                       begin
                         s:=s+sep+gas_regname(taicpu(hp).oper[0]^.ref^.index);
                         if taicpu(hp).oper[0]^.ref^.addressmode=AM_PREINDEXED then
                           s:=s+'!';
                       end;
                     top_reg:
                       s:=s+sep+gas_regname(taicpu(hp).oper[0]^.reg);
                     else
                       internalerror(200311292);
                   end;
                 end
               { register count of SFM and LFM is written without # }
               else if (i=1) and (op in [A_SFM,A_LFM]) then
                 begin
                   case taicpu(hp).oper[1]^.typ of
                     top_const:
                       s:=s+sep+tostr(taicpu(hp).oper[1]^.val);
                     else
                       internalerror(200311292);
                   end;
                 end
               else
                 s:=s+sep+getopstr(taicpu(hp).oper[i]^);

               sep:=',';
            end;
        end;
      owner.writer.AsmWriteLn(s);
    end;


    const
       as_arm_gas_info : tasminfo =
          (
            id     : as_gas;

            idtxt  : 'AS';
            asmbin : 'as';
            asmcmd : '-o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_arm_linux,system_arm_netbsd,system_arm_wince,system_arm_gba,system_arm_palmos,system_arm_nds,
                                 system_arm_embedded,system_arm_symbian,system_arm_android,system_arm_aros];
            flags : [af_needar,af_smartlink_sections];
            labelprefix : '.L';
            comment : '# ';
            dollarsign: '$';
          );

       as_arm_gas_darwin_info : tasminfo =
          (
            id     : as_darwin;
            idtxt  : 'AS-DARWIN';
            asmbin : 'as';
            asmcmd : '-o $OBJ $EXTRAOPT $ASM -arch $ARCH';
            supported_targets : [system_arm_darwin];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf,af_stabs_use_function_absolute_addresses];
            labelprefix : 'L';
            comment : '# ';
            dollarsign: '$';
          );


       as_arm_clang_darwin_info : tasminfo =
          (
            id     : as_clang;
            idtxt  : 'CLANG';
            asmbin : 'clang';
            asmcmd : '-c -o $OBJ $EXTRAOPT -arch $ARCH $DARWINVERSION -x assembler $ASM';
            supported_targets : [system_arm_darwin];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : 'L';
            comment : '# ';
            dollarsign: '$';
          );


begin
  RegisterAssembler(as_arm_gas_info,TARMGNUAssembler);
  RegisterAssembler(as_arm_gas_darwin_info,TArmAppleGNUAssembler);
  RegisterAssembler(as_arm_clang_darwin_info,TArmAppleGNUAssembler);
end.
