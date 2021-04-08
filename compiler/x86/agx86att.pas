{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements an asmoutput class for i386 AT&T syntax

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
{ This unit implements an asmoutput class for i386 AT&T syntax
}
unit agx86att;

{$i fpcdefs.inc}

interface

    uses
      cpubase,systems,
      globtype,cgutils,
      aasmtai,assemble,aggas;

    type
      Tx86ATTAssembler=class(TGNUassembler)
        constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
        function MakeCmdLine: TCmdStr; override;
      end;

      Tx86AppleGNUAssembler=class(TAppleGNUassembler)
        constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
      end;

      Tx86AoutGNUAssembler=class(TAoutGNUassembler)
        constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
      end;


     Tx86InstrWriter=class(TCPUInstrWriter)
       private
        procedure WriteReference(var ref : treference);
        procedure WriteOper(const o:toper);
        procedure WriteOper_jmp(const o:toper);
       protected
        fskipPopcountSuffix: boolean;
        { http://gcc.gnu.org/bugzilla/show_bug.cgi?id=56656 }
        fNoInterUnitMovQ: boolean;
       public
        procedure WriteInstruction(hp: tai);override;
     end;



  implementation

    uses
      cutils,
      verbose,
      itcpugas,
      cgbase,
      aasmcpu;


{****************************************************************************
                            Tx86ATTAssembler
 ****************************************************************************}

    constructor Tx86ATTAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter := Tx86InstrWriter.create(self);
      end;

    function TX86ATTAssembler.MakeCmdLine: TCmdStr;
      var
        FormatName : string;
      begin
        result:=Inherited MakeCmdLine;
{$ifdef i386}
        case target_info.system of
          system_i386_go32v2:
            FormatName:='coff';
          system_i386_wdosx,
          system_i386_win32:
            FormatName:='win32';
          system_i386_embedded:
            FormatName:='obj';
          system_i386_linux,
          system_i386_beos:
            FormatName:='elf';
          system_i386_darwin:
            FormatName:='macho32';
        else
          FormatName:='elf';
        end;
{$endif i386}
{$ifdef x86_64}
        case target_info.system of
          system_x86_64_win64:
            FormatName:='win64';
          system_x86_64_darwin:
            FormatName:='macho64';
          system_x86_64_embedded:
            FormatName:='obj';
          system_x86_64_linux:
            FormatName:='elf64';
        else
          FormatName:='elf64';
        end;
{$endif x86_64}
        Replace(result,'$FORMAT',FormatName);
      end;

{****************************************************************************
                          Tx86AppleGNUAssembler
 ****************************************************************************}

    constructor Tx86AppleGNUAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter := Tx86InstrWriter.create(self);
        { Apple's assembler does not support a size suffix for popcount }
        Tx86InstrWriter(InstrWriter).fskipPopcountSuffix := true;
        { Apple's assembler is broken regarding some movq suffix handling }
        Tx86InstrWriter(InstrWriter).fNoInterUnitMovQ := true;
      end;

{****************************************************************************
                          Tx86AoutGNUAssembler
 ****************************************************************************}

    constructor Tx86AoutGNUAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter := Tx86InstrWriter.create(self);
      end;

{****************************************************************************
                            Tx86InstrWriter
 ****************************************************************************}

    procedure Tx86InstrWriter.WriteReference(var ref : treference);
      begin
        with ref do
         begin
           { do we have a segment prefix ? }
           { These are probably not correctly handled under GAS }
           { should be replaced by coding the segment override  }
           { directly! - DJGPP FAQ                              }
           if segment<>NR_NO then
             owner.writer.AsmWrite(gas_regname(segment)+':');
           if assigned(symbol) then
             owner.writer.AsmWrite(symbol.name);
           if assigned(relsymbol) then
             owner.writer.AsmWrite('-'+relsymbol.name);
           case ref.refaddr of
             addr_pic:
               begin
                 { @GOT and @GOTPCREL references are only allowed for symbol alone,
                   indexing, relsymbol or offset cannot be present. }
                 if assigned(relsymbol) or (offset<>0) or (index<>NR_NO) then
                   InternalError(2015011801);
{$ifdef x86_64}
                 if (base<>NR_RIP) then
                   InternalError(2015011802);
                 owner.writer.AsmWrite('@GOTPCREL');
{$else x86_64}
                 owner.writer.AsmWrite('@GOT');
{$endif x86_64}
               end;
{$ifdef i386}
             addr_ntpoff:
               owner.writer.AsmWrite('@ntpoff');
             addr_tlsgd:
               owner.writer.AsmWrite('@tlsgd');
{$endif i386}
{$ifdef x86_64}
             addr_tpoff:
               owner.writer.AsmWrite('@tpoff');
             addr_tlsgd:
               owner.writer.AsmWrite('@tlsgd');
{$endif x86_64}
             else
               ;
           end;

           if offset<0 then
             owner.writer.AsmWrite(tostr(offset))
           else
            if (offset>0) then
             begin
               if assigned(symbol) then
                owner.writer.AsmWrite('+'+tostr(offset))
               else
                owner.writer.AsmWrite(tostr(offset));
             end
           else if (index=NR_NO) and (base=NR_NO) and (not assigned(symbol)) then
             owner.writer.AsmWrite('0');
           if (index<>NR_NO) and (base=NR_NO) then
            begin
              owner.writer.AsmWrite('(,'+gas_regname(index));
              if scalefactor<>0 then
                owner.writer.AsmWrite(','+tostr(scalefactor));
              owner.writer.AsmWrite(')');
            end
           else
            if (index=NR_NO) and (base<>NR_NO) then
              owner.writer.AsmWrite('('+gas_regname(base)+')')
            else
             if (index<>NR_NO) and (base<>NR_NO) then
              begin
                owner.writer.AsmWrite('('+gas_regname(base)+','+gas_regname(index));
                if scalefactor<>0 then
                 owner.writer.AsmWrite(','+tostr(scalefactor));
                owner.writer.AsmWrite(')');
              end;
         end;
      end;


    procedure Tx86InstrWriter.WriteOper(const o:toper);
      begin
        if o.vopext and OTVE_VECTOR_SAE = OTVE_VECTOR_SAE then
         owner.writer.AsmWrite('{sae},');

        if o.vopext and OTVE_VECTOR_ER_MASK = OTVE_VECTOR_RNSAE then
         owner.writer.AsmWrite('{rn-sae},');

        if o.vopext and OTVE_VECTOR_ER_MASK = OTVE_VECTOR_RDSAE then
         owner.writer.AsmWrite('{rd-sae},');

        if o.vopext and OTVE_VECTOR_ER_MASK = OTVE_VECTOR_RUSAE then
         owner.writer.AsmWrite('{ru-sae},');

        if o.vopext and OTVE_VECTOR_ER_MASK = OTVE_VECTOR_RZSAE then
         owner.writer.AsmWrite('{rz-sae},');

        case o.typ of
          top_reg :
            { Solaris assembler does not accept %st instead of %st(0) }
            if (owner.asminfo^.id=as_solaris_as) and (o.reg=NR_ST) then
              owner.writer.AsmWrite(gas_regname(NR_ST0))
            else
              owner.writer.AsmWrite(gas_regname(o.reg));
          top_ref :
            if o.ref^.refaddr in [addr_no,addr_pic,addr_pic_no_got
              {$ifdef i386},addr_ntpoff,addr_tlsgd{$endif i386}
              {$ifdef x86_64},addr_tpoff,addr_tlsgd{$endif x86_64}
              ] then
              WriteReference(o.ref^)
            else
              begin
                owner.writer.AsmWrite('$');
                if assigned(o.ref^.symbol) then
                 owner.writer.AsmWrite(o.ref^.symbol.name);
                if o.ref^.offset>0 then
                 owner.writer.AsmWrite('+'+tostr(o.ref^.offset))
                else
                 if o.ref^.offset<0 then
                  owner.writer.AsmWrite(tostr(o.ref^.offset))
                else
                 if not(assigned(o.ref^.symbol)) then
                   owner.writer.AsmWrite('0');
              end;
          top_const :
              owner.writer.AsmWrite('$'+tostr(o.val));
          else
            internalerror(2020100810);
        end;

           if o.vopext and OTVE_VECTOR_WRITEMASK = OTVE_VECTOR_WRITEMASK then
            begin
              owner.writer.AsmWrite('{%k' + tostr(o.vopext and $07) + '} ');
              if o.vopext and OTVE_VECTOR_ZERO = OTVE_VECTOR_ZERO then
               owner.writer.AsmWrite('{z}');
            end;


           if o.vopext and OTVE_VECTOR_BCST = OTVE_VECTOR_BCST then
            begin
              case o.vopext and (OTVE_VECTOR_BCST2 or OTVE_VECTOR_BCST4 or OTVE_VECTOR_BCST8 or OTVE_VECTOR_BCST16) of
                 OTVE_VECTOR_BCST2: owner.writer.AsmWrite('{1to2}');
                 OTVE_VECTOR_BCST4: owner.writer.AsmWrite('{1to4}');
                 OTVE_VECTOR_BCST8: owner.writer.AsmWrite('{1to8}');
                OTVE_VECTOR_BCST16: owner.writer.AsmWrite('{1to16}');
                               else ; //TG TODO errormsg
              end;
            end;

      end;


    procedure Tx86InstrWriter.WriteOper_jmp(const o:toper);
      begin
        case o.typ of
          top_reg :
            owner.writer.AsmWrite('*'+gas_regname(o.reg));
          top_ref :
            begin
              if o.ref^.refaddr in [addr_no,addr_pic_no_got] then
                begin
                  owner.writer.AsmWrite('*');
                  WriteReference(o.ref^);
                end
              else
                begin
                  owner.writer.AsmWrite(o.ref^.symbol.name);
                  if o.ref^.refaddr=addr_pic then
                    owner.writer.AsmWrite('@PLT');
                  if o.ref^.offset>0 then
                   owner.writer.AsmWrite('+'+tostr(o.ref^.offset))
                  else
                   if o.ref^.offset<0 then
                    owner.writer.AsmWrite(tostr(o.ref^.offset));
                end;
            end;
          top_const :
            owner.writer.AsmWrite(tostr(o.val));
          else
            internalerror(2020100811);
        end;
      end;


    procedure Tx86InstrWriter.WriteInstruction(hp: tai);
      var
       op       : tasmop;
       calljmp  : boolean;
       i        : integer;
      begin
        if hp.typ <> ait_instruction then
          exit;
        taicpu(hp).SetOperandOrder(op_att);
        op:=taicpu(hp).opcode;
        calljmp:=is_calljmp(op);

        { see fNoInterUnitMovQ declaration comment }
        if fNoInterUnitMovQ then
          begin
            if ((op=A_MOVQ) or
                (op=A_VMOVQ)) and
               (((taicpu(hp).oper[0]^.typ=top_reg) and
                 (getregtype(taicpu(hp).oper[0]^.reg)=R_INTREGISTER)) or
                ((taicpu(hp).oper[1]^.typ=top_reg) and
                 (getregtype(taicpu(hp).oper[1]^.reg)=R_INTREGISTER))) then
              begin
                if op=A_MOVQ then
                  op:=A_MOVD
                else
                  op:=A_VMOVD;
                taicpu(hp).opcode:=op;
              end;
          end;
        owner.writer.AsmWrite(#9);
        { movsd should not be translated to movsl when there
          are (xmm) arguments }
        if (op=A_MOVSD) and (taicpu(hp).ops>0) then
          owner.writer.AsmWrite('movsd')
        { the same applies to cmpsd as well }
        else if (op=A_CMPSD) and (taicpu(hp).ops>0) then
          owner.writer.AsmWrite('cmpsd')
        else
          owner.writer.AsmWrite(gas_op2str[op]);
        owner.writer.AsmWrite(cond2str[taicpu(hp).condition]);
        { suffix needed ?  fnstsw,fldcw don't support suffixes
          with binutils 2.9.5 under linux }
{        if (Taicpu(hp).oper[0]^.typ=top_reg) and
            (Taicpu(hp).oper[0]^.reg.enum>lastreg) then
          internalerror(200301081);}

        if (not calljmp) and
           (gas_needsuffix[op]<>AttSufNONE) and
           (op<>A_FNSTSW) and
           (op<>A_FSTSW) and
           (op<>A_FNSTCW) and
           (op<>A_FSTCW) and
           (op<>A_FLDCW) and
           (not fskipPopcountSuffix or
            (op<>A_POPCNT)) and
           ((owner.asminfo^.id<>as_solaris_as) or (op<>A_Jcc) and (op<>A_SETcc)) and
           not(
               (taicpu(hp).ops<>0) and
               (taicpu(hp).oper[0]^.typ=top_reg) and
               (getregtype(taicpu(hp).oper[0]^.reg)=R_FPUREGISTER)
              ) then
        begin
          if (gas_needsuffix[op]<>AttSufMMX) or
	     (taicpu(hp).opsize in [S_XMM,S_YMM]) then
          owner.writer.AsmWrite(gas_opsize2str[taicpu(hp).opsize]);
        end;

        { process operands }
        if taicpu(hp).ops<>0 then
          begin
            if calljmp then
             begin
               owner.writer.AsmWrite(#9);
               WriteOper_jmp(taicpu(hp).oper[0]^);
             end
            else
             begin
               for i:=0 to taicpu(hp).ops-1 do
                 begin
                   if i=0 then
                     owner.writer.AsmWrite(#9)
                   else
                     owner.writer.AsmWrite(',');
                   WriteOper(taicpu(hp).oper[i]^);
                 end;
             end;
          end;
        owner.writer.AsmLn;
      end;


{*****************************************************************************
                                  Initialize
*****************************************************************************}

    const
{$ifdef x86_64}
       as_x86_64_as_info : tasminfo =
          (
            id     : as_gas;
            idtxt  : 'AS';
            asmbin : 'as';
            asmcmd : '--64 -o $OBJ $BIGOBJ $EXTRAOPT $ASM';
            supported_targets : [system_x86_64_linux,system_x86_64_freebsd,
                                 system_x86_64_win64,system_x86_64_embedded,
                                 system_x86_64_openbsd,system_x86_64_netbsd,
                                 system_x86_64_dragonfly,system_x86_64_aros,
                                 system_x86_64_android,system_x86_64_haiku];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            labelmaxlen : -1;
            comment : '# ';
            dollarsign: '$';
          );

       as_x86_64_yasm_info : tasminfo =
          (
            id     : as_yasm;
            idtxt  : 'YASM';
            asmbin : 'yasm';
            asmcmd : '-a x86 -p gas -f $FORMAT -o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_x86_64_linux,system_x86_64_freebsd,system_x86_64_win64,system_x86_64_embedded];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            labelmaxlen : -1;
            comment : '# ';
            dollarsign: '$';
          );

       as_x86_64_gas_info : tasminfo =
          (
            id     : as_ggas;
            idtxt  : 'GAS';
            asmbin : 'gas';
            asmcmd : '--64 -o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_x86_64_solaris];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            labelmaxlen : -1;
            comment : '# ';
            dollarsign: '$';
          );


       as_x86_64_solaris_info : tasminfo =
          (
            id     : as_solaris_as;
            idtxt  : 'AS-SOL';
            asmbin : 'as';
            asmcmd : ' -m64 -o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_x86_64_solaris];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            labelmaxlen : -1;
            comment : '# ';
            dollarsign: '$';
          );



       as_x86_64_gas_darwin_info : tasminfo =
          (
            id     : as_darwin;
            idtxt  : 'AS-DARWIN';
            asmbin : 'as';
            asmcmd : '-o $OBJ $EXTRAOPT $ASM -arch x86_64';
            supported_targets : [system_x86_64_darwin,system_x86_64_iphonesim];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : 'L';
            labelmaxlen : -1;
            comment : '# ';
            dollarsign: '$';
          );

       as_x86_64_clang_darwin_info : tasminfo =
          (
            id     : as_clang_asdarwin;
            idtxt  : 'CLANG';
            asmbin : 'clang';
            asmcmd : '-x assembler -c -target $TRIPLET -o $OBJ $EXTRAOPT -x assembler $ASM';
            supported_targets : [system_x86_64_darwin,system_x86_64_iphonesim];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf,af_no_stabs,af_llvm,af_supports_hlcfi];
            labelprefix : 'L';
            labelmaxlen : -1;
            comment : '# ';
            dollarsign: '$';
          );

{$else x86_64}
       as_i386_as_info : tasminfo =
          (
            id     : as_gas;
            idtxt  : 'AS';
            asmbin : 'as';
            asmcmd : '--32 -o $OBJ $BIGOBJ $EXTRAOPT $ASM';
            supported_targets : [system_i386_GO32V2,system_i386_linux,system_i386_Win32,system_i386_freebsd,system_i386_solaris,system_i386_beos,
                                system_i386_netbsd,system_i386_Netware,system_i386_wdosx,system_i386_openbsd,
                                system_i386_netwlibc,system_i386_wince,system_i386_embedded,system_i386_symbian,system_i386_haiku,system_x86_6432_linux,
                                system_i386_nativent,system_i386_android,system_i386_aros];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            labelmaxlen : -1;
            comment : '# ';
            dollarsign: '$';
          );

       as_i386_yasm_info : tasminfo =
          (
            id     : as_yasm;
            idtxt  : 'YASM';
            asmbin : 'yasm';
            asmcmd : '-a x86 -p gas -f $FORMAT -o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_i386_GO32V2,system_i386_linux,system_i386_Win32,system_i386_freebsd,system_i386_solaris,system_i386_beos,
                                system_i386_netbsd,system_i386_Netware,system_i386_wdosx,system_i386_openbsd,
                                system_i386_netwlibc,system_i386_wince,system_i386_embedded,system_i386_symbian,system_i386_haiku,system_x86_6432_linux,
                                system_i386_nativent];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            labelmaxlen : -1;
            comment : '# ';
            dollarsign: '$';
          );


       as_i386_as_aout_info : tasminfo =
          (
            id           : as_i386_as_aout;
            idtxt  : 'AS_AOUT';
            asmbin : 'as';
            asmcmd : '-o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_i386_linux,system_i386_OS2,system_i386_freebsd,system_i386_netbsd,system_i386_openbsd,system_i386_EMX,system_i386_embedded];
            flags : [af_needar,af_stabs_use_function_absolute_addresses];
            labelprefix : 'L';
            labelmaxlen : -1;
            comment : '# ';
            dollarsign: '$';
          );


       as_i386_gas_darwin_info : tasminfo =
          (
            id     : as_darwin;
            idtxt  : 'AS-DARWIN';
            asmbin : 'as';
            asmcmd : '-o $OBJ $EXTRAOPT $ASM -arch i386';
            supported_targets : [system_i386_darwin,system_i386_iphonesim];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf,af_stabs_use_function_absolute_addresses];
            labelprefix : 'L';
            labelmaxlen : -1;
            comment : '# ';
            dollarsign: '$';
          );

       as_i386_clang_darwin_info : tasminfo =
          (
            id     : as_clang_asdarwin;
            idtxt  : 'CLANG';
            asmbin : 'clang';
            asmcmd : '-x assembler -c -target $TRIPLET -o $OBJ $EXTRAOPT -x assembler $ASM';
            supported_targets : [system_i386_darwin,system_i386_iphonesim];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf,af_no_stabs,af_llvm,af_supports_hlcfi];
            labelprefix : 'L';
            labelmaxlen : -1;
            comment : '# ';
            dollarsign: '$';
          );

       as_i386_gas_info : tasminfo =
          (
            id     : as_ggas;
            idtxt  : 'GAS';
            asmbin : 'gas';
            asmcmd : '--32 -o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_i386_GO32V2,system_i386_linux,system_i386_Win32,system_i386_freebsd,system_i386_solaris,system_i386_beos,
                                system_i386_netbsd,system_i386_Netware,system_i386_wdosx,system_i386_openbsd,
                                system_i386_netwlibc,system_i386_wince,system_i386_embedded,system_i386_symbian,system_i386_haiku,
                                system_x86_6432_linux,system_i386_android];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            labelmaxlen : -1;
            comment : '# ';
            dollarsign: '$';
          );

       as_i386_solaris_info : tasminfo =
          (
            id     : as_solaris_as;
            idtxt  : 'AS-SOL';
            asmbin : 'as';
            asmcmd : ' -o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_i386_solaris];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            labelmaxlen : -1;
            comment : '# ';
            dollarsign: '$';
          );


{$endif x86_64}

initialization
{$ifdef x86_64}
  RegisterAssembler(as_x86_64_as_info,Tx86ATTAssembler);
  RegisterAssembler(as_x86_64_yasm_info,Tx86ATTAssembler);
  RegisterAssembler(as_x86_64_gas_info,Tx86ATTAssembler);
  RegisterAssembler(as_x86_64_gas_darwin_info,Tx86AppleGNUAssembler);
  RegisterAssembler(as_x86_64_clang_darwin_info,Tx86AppleGNUAssembler);
  RegisterAssembler(as_x86_64_solaris_info,Tx86ATTAssembler);
{$else x86_64}
  RegisterAssembler(as_i386_as_info,Tx86ATTAssembler);
  RegisterAssembler(as_i386_gas_info,Tx86ATTAssembler);
  RegisterAssembler(as_i386_yasm_info,Tx86ATTAssembler);
  RegisterAssembler(as_i386_gas_darwin_info,Tx86AppleGNUAssembler);
  RegisterAssembler(as_i386_clang_darwin_info,Tx86AppleGNUAssembler);
  RegisterAssembler(as_i386_as_aout_info,Tx86AoutGNUAssembler);
  RegisterAssembler(as_i386_solaris_info,Tx86ATTAssembler);
{$endif x86_64}
end.
